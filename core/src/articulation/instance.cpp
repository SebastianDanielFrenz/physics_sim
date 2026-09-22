#include "instance.h"

#include "gravity.h"

#include <cstring>

namespace ps::articulation {

using spatial::SpatialVector;

namespace {

// FNV-1a over a POD value's bytes, matching the convention
// IRigidBackend::compute_state_hash uses (core/src/backend/jolt/jolt_backend.cpp)
// so World::state_hash's fold order is uniform across backends.
template <typename T>
std::uint64_t fnv1a(std::uint64_t h, const T& v) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

Pose joint_transform_pose(const BodyModel& b, const ArticulationState& st) {
    switch (b.joint) {
        case BodyJointKind::Revolute:
            return Pose{Vec3::zero(), Quat::from_axis_angle(b.axis, st.q[static_cast<std::size_t>(b.q_index)])};
        case BodyJointKind::Prismatic:
            return Pose{b.axis * st.q[static_cast<std::size_t>(b.q_index)], Quat::identity()};
        case BodyJointKind::Fixed:
        case BodyJointKind::Floating6:
            return Pose::identity();
    }
    return Pose::identity();
}

} // namespace

Instance::Instance(Model model, const articulation::ArticulationDesc& desc) : model_(std::move(model)) {
    state_.q.assign(static_cast<std::size_t>(model_.num_q), real(0.0));
    state_.qd.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
    state_.root_pose = desc.root.pose; // meaningful as the (immovable) root pose either way; only floating updates it after this

    direct_tau_.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
    ext_wrench_world_.assign(model_.bodies.size(), Wrench{});
    body_world_pose_.assign(model_.bodies.size(), Pose::identity());
    body_world_motion_.assign(model_.bodies.size(), Motion{});

    link_to_body_.assign(desc.links.size(), -1);
    for (std::size_t i = 0; i < model_.bodies.size(); ++i) {
        if (model_.bodies[i].source_link >= 0) link_to_body_[static_cast<std::size_t>(model_.bodies[i].source_link)] = static_cast<int>(i);
    }

    // Seed q/qd from each link's initial condition, and the floating root's
    // initial spatial velocity (its own axes - see model.h's convention;
    // desc.root.velocity is given in world axes at q=root.pose, which at
    // t=0 coincide with the root's own axes only up to root.pose's
    // rotation, so rotate it in).
    for (std::size_t i = 0; i < desc.links.size(); ++i) {
        const BodyModel& b = model_.bodies[static_cast<std::size_t>(link_to_body_[i])];
        if (b.ndof == 1) {
            state_.q[static_cast<std::size_t>(b.q_index)] = desc.links[i].q0;
            state_.qd[static_cast<std::size_t>(b.dof_index)] = desc.links[i].qd0;
        }
    }
    if (model_.floating_root) {
        const Mat3 r_inv = desc.root.pose.orientation.inverse().to_mat3();
        const Vec3 w_body = r_inv * desc.root.velocity.angular;
        const Vec3 v_body = r_inv * desc.root.velocity.linear;
        state_.qd[0] = w_body.x; state_.qd[1] = w_body.y; state_.qd[2] = w_body.z;
        state_.qd[3] = v_body.x; state_.qd[4] = v_body.y; state_.qd[5] = v_body.z;
    }

    loop_closures_ = build_loop_closure_runtime(model_, link_to_body_);
    loop_closure_scratch_.ensure(model_, loop_closures_.size());

    // --- Preallocate every step_tick()-time scratch buffer HERE, on the
    // caller's thread at construction time - never inside a job (PHYS-009
    // Step 1 rule 2). Every buffer below is sized once, from model_'s fixed
    // dimensions, and never resized again for the rest of this instance's
    // lifetime; every write to it from here on is an in-place overwrite of
    // already-allocated storage.
    aba_scratch_.ensure(model_.bodies.size());
    aba_scratch_.ain.tau.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
    aba_scratch_.ain.ext_force.assign(model_.bodies.size(), SpatialVector{});
    ext_force_scratch_.assign(model_.bodies.size(), SpatialVector{});
    ext_force_g_scratch_.assign(model_.bodies.size(), SpatialVector{});
    contact_wrench_scratch_.assign(model_.bodies.size(), Wrench{});
    combined_ext_force_scratch_.assign(model_.bodies.size(), SpatialVector{});
    fk_pose_scratch_.assign(model_.bodies.size(), Pose::identity());
    tau_scratch_.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
    qdd_scratch_.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
    v_local_scratch_.assign(model_.bodies.size(), SpatialVector{});
    for (ArticulationState* s : {&rk4_s2_, &rk4_s3_, &rk4_s4_}) {
        s->q.assign(static_cast<std::size_t>(model_.num_q), real(0.0));
        s->qd.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
        s->root_pose = state_.root_pose;
    }
    for (Derivative* d : {&rk4_k1_, &rk4_k2_, &rk4_k3_, &rk4_k4_}) {
        d->qdot.assign(static_cast<std::size_t>(model_.num_q), real(0.0));
        d->qddot.assign(static_cast<std::size_t>(model_.num_v), real(0.0));
    }

    recompute_world_kinematics();
}

void Instance::set_joint_force(int link_index, real tau) {
    const int body = link_to_body_[static_cast<std::size_t>(link_index)];
    const BodyModel& b = model_.bodies[static_cast<std::size_t>(body)];
    if (b.dof_index >= 0) direct_tau_[static_cast<std::size_t>(b.dof_index)] = tau;
}

void Instance::accumulate_external_wrench(int body_index, const Pose& frame_world_pose, const Wrench& wrench) {
    // Shift the applied wrench's moment from the frame's own application
    // point to the body's (link's) origin, world axes - same "moment
    // about a reference point" bookkeeping spatial.h's SpatialInertia
    // comment uses, just for a force instead of a mass distribution.
    const Vec3 offset_world = frame_world_pose.position - body_world_pose_[static_cast<std::size_t>(body_index)].position;
    Wrench shifted = wrench;
    shifted.torque = wrench.torque + cross(offset_world, wrench.force);
    ext_wrench_world_[static_cast<std::size_t>(body_index)] += shifted;
}

void Instance::clear_external_wrenches() {
    std::fill(ext_wrench_world_.begin(), ext_wrench_world_.end(), Wrench{});
}

void Instance::compute_tau_into(const std::vector<real>& qd, std::vector<real>& out) const {
    out = direct_tau_; // same size every call (num_v); vector::operator= reuses existing capacity, no realloc
    for (const auto& b : model_.bodies) {
        if (b.ndof != 1) continue;
        const std::size_t di = static_cast<std::size_t>(b.dof_index);
        // Viscous joint damping.
        out[di] -= b.joint_damping * qd[di];
        // Force-limited velocity actuator (a hydraulic-cylinder preview,
        // PLAN.md P1.5a scope item 2): always pushes at +-max_force toward
        // target_speed. Against a load needing more than max_force to
        // track it, qd never reaches target_speed and the joint stalls
        // with the actuator permanently saturated at max_force - which is
        // exactly the clamp below, unconditionally applied every substep
        // rather than only "when far from target", so the stall force is
        // exact rather than approached asymptotically by a P-controller.
        if (b.vel_actuator.enabled) {
            const real err = b.vel_actuator.target_speed - qd[di];
            const real cmd = (err > real(0.0)) ? b.vel_actuator.max_force : (err < real(0.0) ? -b.vel_actuator.max_force : real(0.0));
            out[di] += cmd;
        }
    }
}

void Instance::step_tick(const InstanceConfig& cfg, real tick_dt) {
    const int num_substeps = std::max(1, static_cast<int>(cfg.substep_rate_hz * tick_dt + real(0.5)));
    const real h = tick_dt / static_cast<real>(num_substeps);

    // Convert the tick's accumulated external wrenches to per-body spatial
    // forces, body-local axes, ONCE - held constant across every substep
    // (PLAN.md D3). Written into ext_force_scratch_ (preallocated at
    // construction) instead of a fresh per-tick vector (PHYS-009 Step 1
    // rule 2 - step_tick itself now runs inside a job, System::step_all).
    for (std::size_t i = 0; i < model_.bodies.size(); ++i) {
        const Wrench& w = ext_wrench_world_[i];
        const Mat3 r_inv = body_world_pose_[i].orientation.inverse().to_mat3();
        ext_force_scratch_[i] = SpatialVector{r_inv * w.torque, r_inv * w.force};
    }

    for (int s = 0; s < num_substeps; ++s) {
        const std::vector<SpatialVector>* ext_force_for_substep = &ext_force_scratch_;

        // PLAN.md P1.5b (contact_source.h's header comment): a stiff
        // compliant contact spring must be resampled every substep, not
        // held constant across the whole tick like every other external
        // wrench above - recompute_world_kinematics() here gives the
        // source THIS substep's actual state (not the tick-start state the
        // loop began with).
        if (contact_source_) {
            recompute_world_kinematics();
            for (auto& w : contact_wrench_scratch_) w = Wrench{};
            contact_source_->evaluate_substep_contacts(*this, *contact_backend_, h, tick_dt, contact_wrench_scratch_);
            for (std::size_t i = 0; i < model_.bodies.size(); ++i) {
                const Wrench& cw = contact_wrench_scratch_[i];
                const Mat3 r_inv = body_world_pose_[i].orientation.inverse().to_mat3();
                combined_ext_force_scratch_[i] = ext_force_scratch_[i] + SpatialVector{r_inv * cw.torque, r_inv * cw.force};
            }
            ext_force_for_substep = &combined_ext_force_scratch_;
        }

        if (cfg.integrator == IntegratorKind::Rk4) {
            step_rk4(cfg, *ext_force_for_substep, h);
        } else {
            step_semi_implicit_euler(cfg, *ext_force_for_substep, h);
        }
        clamp_joint_limits();
        // Drift stabilisation (PLAN.md P1.5a scope item 4), every substep:
        // measured (P1.5a report) against once-per-tick - per-substep
        // corrections are individually much smaller, which measured lower
        // energy drift overall despite the extra ABA-probe cost.
        if (!loop_closures_.empty()) project_onto_constraints(model_, state_, loop_closures_, &aba_scratch_, &loop_closure_scratch_);
    }

    recompute_world_kinematics();
}

void Instance::clamp_joint_limits() {
    for (const auto& b : model_.bodies) {
        if (b.ndof != 1 || !b.limits.enabled) continue;
        real& q = state_.q[static_cast<std::size_t>(b.q_index)];
        real& qd = state_.qd[static_cast<std::size_t>(b.dof_index)];
        if (q < b.limits.min) { q = b.limits.min; qd = std::max(real(0.0), qd); }
        if (q > b.limits.max) { q = b.limits.max; qd = std::min(real(0.0), qd); }
    }
}

void Instance::add_gravity_into(const ArticulationState& st, const std::vector<SpatialVector>& base_ext_force,
                                 const GravityField& field, std::vector<SpatialVector>& out) {
    forward_kinematics_poses_into(model_, st, fk_pose_scratch_);
    out = base_ext_force; // same size every call (bodies.size()); reuses capacity
    for (std::size_t i = 0; i < model_.bodies.size(); ++i) {
        const auto& b = model_.bodies[i];
        if (b.inertia.mass <= real(0.0)) continue;
        const Vec3 com_local = b.inertia.mass > real(0.0) ? b.inertia.h / b.inertia.mass : Vec3::zero();
        const Vec3 com_world = fk_pose_scratch_[i].transform_point(com_local);
        const Vec3 g_world = field.at(com_world);
        const Mat3 r_inv = fk_pose_scratch_[i].orientation.inverse().to_mat3();
        const Vec3 f_local = r_inv * (g_world * b.inertia.mass);
        out[i] += SpatialVector{cross(com_local, f_local), f_local};
    }
}

void Instance::step_semi_implicit_euler(const InstanceConfig& cfg, const std::vector<SpatialVector>& ext_force, real h) {
    compute_tau_into(state_.qd, tau_scratch_);
    // Gravity applied per body as an external spatial force at its own
    // COM (gravity.h), not Featherstone's a_0 base-acceleration trick -
    // gravity=zero() below so aba()'s own (still-present, unused here)
    // trick contributes nothing.
    const GravityField field{cfg.gravity};
    add_gravity_into(state_, ext_force, field, ext_force_g_scratch_);
    solve_constrained_dynamics_into(model_, state_, loop_closures_, tau_scratch_, Vec3::zero(), ext_force_g_scratch_, aba_scratch_, qdd_scratch_,
                                     loop_closure_scratch_);

    // Semi-implicit (symplectic) Euler: velocity first, then position using
    // the NEW velocity.
    for (int i = 0; i < model_.num_v; ++i) state_.qd[static_cast<std::size_t>(i)] += qdd_scratch_[static_cast<std::size_t>(i)] * h;
    for (const auto& b : model_.bodies) {
        if (b.ndof == 1) state_.q[static_cast<std::size_t>(b.q_index)] += state_.qd[static_cast<std::size_t>(b.dof_index)] * h;
    }
    if (model_.floating_root) {
        const Vec3 w_body{state_.qd[0], state_.qd[1], state_.qd[2]};
        const Vec3 v_body{state_.qd[3], state_.qd[4], state_.qd[5]};
        state_.root_pose.position += state_.root_pose.orientation.rotate(v_body) * h;
        const real angle = w_body.length();
        if (angle > real(1e-14)) {
            state_.root_pose.orientation = (state_.root_pose.orientation * Quat::from_axis_angle(w_body / angle, angle * h)).normalized();
        }
    }
}

void Instance::evaluate_derivative_into(const ArticulationState& st, const std::vector<real>& tau,
                                         const std::vector<SpatialVector>& ext_force, const Vec3& gravity, Derivative& out) {
    const GravityField field{gravity};
    add_gravity_into(st, ext_force, field, ext_force_g_scratch_);
    solve_constrained_dynamics_into(model_, st, loop_closures_, tau, Vec3::zero(), ext_force_g_scratch_, aba_scratch_, out.qddot, loop_closure_scratch_);

    for (const auto& b : model_.bodies) {
        if (b.ndof == 1) out.qdot[static_cast<std::size_t>(b.q_index)] = st.qd[static_cast<std::size_t>(b.dof_index)];
    }
    if (model_.floating_root) {
        const Vec3 w_body{st.qd[0], st.qd[1], st.qd[2]};
        const Vec3 v_body{st.qd[3], st.qd[4], st.qd[5]};
        out.root_pos_dot = st.root_pose.orientation.rotate(v_body);
        // Standard quaternion kinematic relation q_dot = 0.5*q(x)(w,0)
        // (Hamilton product, w expressed in BODY axes matches this file's
        // "q maps body->world" convention throughout).
        const Quat wq{w_body.x, w_body.y, w_body.z, real(0.0)};
        const Quat qdot = st.root_pose.orientation * wq; // *0.5 folded into the writes below
        out.root_quat_dot[0] = real(0.5) * qdot.x; out.root_quat_dot[1] = real(0.5) * qdot.y;
        out.root_quat_dot[2] = real(0.5) * qdot.z; out.root_quat_dot[3] = real(0.5) * qdot.w;
    }
}

void Instance::add_scaled_into(const ArticulationState& base, const Derivative& d, real scale, ArticulationState& out) const {
    for (std::size_t i = 0; i < out.q.size(); ++i) out.q[i] = base.q[i] + scale * d.qdot[i];
    for (std::size_t i = 0; i < out.qd.size(); ++i) out.qd[i] = base.qd[i] + scale * d.qddot[i];
    if (model_.floating_root) {
        out.root_pose.position = base.root_pose.position + d.root_pos_dot * scale;
        Quat q = base.root_pose.orientation;
        q.x += scale * d.root_quat_dot[0]; q.y += scale * d.root_quat_dot[1];
        q.z += scale * d.root_quat_dot[2]; q.w += scale * d.root_quat_dot[3];
        out.root_pose.orientation = q.normalized();
    } else {
        out.root_pose = base.root_pose;
    }
}

void Instance::step_rk4(const InstanceConfig& cfg, const std::vector<SpatialVector>& ext_force, real h) {
    compute_tau_into(state_.qd, tau_scratch_);

    evaluate_derivative_into(state_, tau_scratch_, ext_force, cfg.gravity, rk4_k1_);
    add_scaled_into(state_, rk4_k1_, h * real(0.5), rk4_s2_);
    evaluate_derivative_into(rk4_s2_, tau_scratch_, ext_force, cfg.gravity, rk4_k2_);
    add_scaled_into(state_, rk4_k2_, h * real(0.5), rk4_s3_);
    evaluate_derivative_into(rk4_s3_, tau_scratch_, ext_force, cfg.gravity, rk4_k3_);
    add_scaled_into(state_, rk4_k3_, h, rk4_s4_);
    evaluate_derivative_into(rk4_s4_, tau_scratch_, ext_force, cfg.gravity, rk4_k4_);

    const Derivative &k1 = rk4_k1_, &k2 = rk4_k2_, &k3 = rk4_k3_, &k4 = rk4_k4_;

    for (std::size_t i = 0; i < state_.q.size(); ++i) {
        state_.q[i] += (h / real(6.0)) * (k1.qdot[i] + real(2.0) * k2.qdot[i] + real(2.0) * k3.qdot[i] + k4.qdot[i]);
    }
    for (std::size_t i = 0; i < state_.qd.size(); ++i) {
        state_.qd[i] += (h / real(6.0)) * (k1.qddot[i] + real(2.0) * k2.qddot[i] + real(2.0) * k3.qddot[i] + k4.qddot[i]);
    }
    if (model_.floating_root) {
        const Vec3 pos_dot = (k1.root_pos_dot + k2.root_pos_dot * real(2.0) + k3.root_pos_dot * real(2.0) + k4.root_pos_dot) * (real(1.0) / real(6.0));
        state_.root_pose.position += pos_dot * h;
        Quat q = state_.root_pose.orientation;
        for (int c = 0; c < 4; ++c) {
            const real rate = (k1.root_quat_dot[c] + real(2.0) * k2.root_quat_dot[c] + real(2.0) * k3.root_quat_dot[c] + k4.root_quat_dot[c]) / real(6.0);
            if (c == 0) q.x += h * rate; else if (c == 1) q.y += h * rate; else if (c == 2) q.z += h * rate; else q.w += h * rate;
        }
        state_.root_pose.orientation = q.normalized();
    }
}

void Instance::recompute_world_kinematics() {
    const std::size_t n = model_.bodies.size();
    // PHYS-009 Step 1 rule 2: was a fresh local vector every call (this
    // runs once per step_tick, i.e. once per job); v_local_scratch_ is
    // preallocated at construction like every other step-time buffer.
    std::vector<SpatialVector>& v_local = v_local_scratch_; // body-local spatial velocity, for world_motion conversion
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = model_.bodies[i];
        if (model_.floating_root && i == 0) {
            body_world_pose_[i] = state_.root_pose;
        } else {
            const Pose parent_pose = (b.parent < 0) ? Pose::identity() : body_world_pose_[static_cast<std::size_t>(b.parent)];
            body_world_pose_[i] = parent_pose * b.joint_pose_in_parent * joint_transform_pose(b, state_);
        }

        const SpatialVector v_parent_local = (b.parent < 0) ? SpatialVector{} : v_local[static_cast<std::size_t>(b.parent)];
        const spatial::SpatialTransform xup = compute_xup(model_, static_cast<int>(i), state_);
        // Qualified: Instance::joint_velocity(int) below would otherwise
        // shadow the free function of the same name from model.h.
        const SpatialVector vj = ::ps::articulation::joint_velocity(b, state_);
        v_local[i] = xup.apply_motion(v_parent_local) + vj;

        const Quat rot = body_world_pose_[i].orientation;
        body_world_motion_[i] = Motion{rot.rotate(v_local[i].bottom), rot.rotate(v_local[i].top)};
    }
}

real Instance::joint_position(int link_index) const {
    const int body = link_to_body_[static_cast<std::size_t>(link_index)];
    const BodyModel& b = model_.bodies[static_cast<std::size_t>(body)];
    return b.q_index >= 0 ? state_.q[static_cast<std::size_t>(b.q_index)] : real(0.0);
}

real Instance::joint_velocity(int link_index) const {
    const int body = link_to_body_[static_cast<std::size_t>(link_index)];
    const BodyModel& b = model_.bodies[static_cast<std::size_t>(body)];
    return b.dof_index >= 0 ? state_.qd[static_cast<std::size_t>(b.dof_index)] : real(0.0);
}

Pose Instance::link_world_pose(int link_index) const {
    if (link_index < 0) return state_.root_pose;
    return body_world_pose_[static_cast<std::size_t>(link_to_body_[static_cast<std::size_t>(link_index)])];
}

Motion Instance::link_world_motion(int link_index) const {
    if (link_index < 0) return model_.floating_root ? body_world_motion_[0] : Motion{};
    return body_world_motion_[static_cast<std::size_t>(link_to_body_[static_cast<std::size_t>(link_index)])];
}

std::uint64_t Instance::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (real q : state_.q) h = fnv1a(h, q);
    for (real qd : state_.qd) h = fnv1a(h, qd);
    if (model_.floating_root) {
        h = fnv1a(h, state_.root_pose.position);
        h = fnv1a(h, state_.root_pose.orientation);
    }
    return h;
}

} // namespace ps::articulation
