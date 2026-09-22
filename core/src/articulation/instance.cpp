#include "instance.h"

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

std::vector<real> Instance::compute_tau(const std::vector<real>& qd) const {
    std::vector<real> tau = direct_tau_;
    for (const auto& b : model_.bodies) {
        if (b.ndof != 1) continue;
        const std::size_t di = static_cast<std::size_t>(b.dof_index);
        // Viscous joint damping.
        tau[di] -= b.joint_damping * qd[di];
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
            tau[di] += cmd;
        }
    }
    return tau;
}

void Instance::step_tick(const InstanceConfig& cfg, real tick_dt) {
    const int num_substeps = std::max(1, static_cast<int>(cfg.substep_rate_hz * tick_dt + real(0.5)));
    const real h = tick_dt / static_cast<real>(num_substeps);

    // Convert the tick's accumulated external wrenches to per-body spatial
    // forces, body-local axes, ONCE - held constant across every substep
    // (PLAN.md D3).
    std::vector<SpatialVector> ext_force(model_.bodies.size());
    for (std::size_t i = 0; i < model_.bodies.size(); ++i) {
        const Wrench& w = ext_wrench_world_[i];
        const Mat3 r_inv = body_world_pose_[i].orientation.inverse().to_mat3();
        ext_force[i] = SpatialVector{r_inv * w.torque, r_inv * w.force};
    }

    for (int s = 0; s < num_substeps; ++s) {
        if (cfg.integrator == IntegratorKind::Rk4) {
            step_rk4(cfg, ext_force, h);
        } else {
            step_semi_implicit_euler(cfg, ext_force, h);
        }
        clamp_joint_limits();
        // Drift stabilisation (PLAN.md P1.5a scope item 4), every substep:
        // measured (P1.5a report) against once-per-tick - per-substep
        // corrections are individually much smaller, which measured lower
        // energy drift overall despite the extra ABA-probe cost.
        if (!loop_closures_.empty()) project_onto_constraints(model_, state_, loop_closures_);
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

void Instance::step_semi_implicit_euler(const InstanceConfig& cfg, const std::vector<SpatialVector>& ext_force, real h) {
    const std::vector<real> tau = compute_tau(state_.qd);
    const AbaOutput aout = solve_constrained_dynamics(model_, state_, loop_closures_, tau, cfg.gravity, ext_force);

    // Semi-implicit (symplectic) Euler: velocity first, then position using
    // the NEW velocity.
    for (int i = 0; i < model_.num_v; ++i) state_.qd[static_cast<std::size_t>(i)] += aout.qdd[static_cast<std::size_t>(i)] * h;
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

Instance::Derivative Instance::evaluate_derivative(const ArticulationState& st, const std::vector<real>& tau,
                                                     const std::vector<SpatialVector>& ext_force, const Vec3& gravity) const {
    const AbaOutput aout = solve_constrained_dynamics(model_, st, loop_closures_, tau, gravity, ext_force);

    Derivative d;
    d.qdot.assign(static_cast<std::size_t>(model_.num_q), real(0.0));
    for (const auto& b : model_.bodies) {
        if (b.ndof == 1) d.qdot[static_cast<std::size_t>(b.q_index)] = st.qd[static_cast<std::size_t>(b.dof_index)];
    }
    d.qddot = aout.qdd;
    if (model_.floating_root) {
        const Vec3 w_body{st.qd[0], st.qd[1], st.qd[2]};
        const Vec3 v_body{st.qd[3], st.qd[4], st.qd[5]};
        d.root_pos_dot = st.root_pose.orientation.rotate(v_body);
        // Standard quaternion kinematic relation q_dot = 0.5*q(x)(w,0)
        // (Hamilton product, w expressed in BODY axes matches this file's
        // "q maps body->world" convention throughout).
        const Quat wq{w_body.x, w_body.y, w_body.z, real(0.0)};
        const Quat qdot = st.root_pose.orientation * wq; // *0.5 folded into the writes below
        d.root_quat_dot[0] = real(0.5) * qdot.x; d.root_quat_dot[1] = real(0.5) * qdot.y;
        d.root_quat_dot[2] = real(0.5) * qdot.z; d.root_quat_dot[3] = real(0.5) * qdot.w;
    }
    return d;
}

void Instance::step_rk4(const InstanceConfig& cfg, const std::vector<SpatialVector>& ext_force, real h) {
    const std::vector<real> tau = compute_tau(state_.qd);

    auto add_scaled = [&](const ArticulationState& base, const Derivative& d, real scale) {
        ArticulationState out = base;
        for (std::size_t i = 0; i < out.q.size(); ++i) out.q[i] = state_.q[i] + scale * d.qdot[i];
        for (std::size_t i = 0; i < out.qd.size(); ++i) out.qd[i] = state_.qd[i] + scale * d.qddot[i];
        if (model_.floating_root) {
            out.root_pose.position = state_.root_pose.position + d.root_pos_dot * scale;
            Quat q = state_.root_pose.orientation;
            q.x += scale * d.root_quat_dot[0]; q.y += scale * d.root_quat_dot[1];
            q.z += scale * d.root_quat_dot[2]; q.w += scale * d.root_quat_dot[3];
            out.root_pose.orientation = q.normalized();
        }
        return out;
    };

    const Derivative k1 = evaluate_derivative(state_, tau, ext_force, cfg.gravity);
    const ArticulationState s2 = add_scaled(state_, k1, h * real(0.5));
    const Derivative k2 = evaluate_derivative(s2, tau, ext_force, cfg.gravity);
    const ArticulationState s3 = add_scaled(state_, k2, h * real(0.5));
    const Derivative k3 = evaluate_derivative(s3, tau, ext_force, cfg.gravity);
    const ArticulationState s4 = add_scaled(state_, k3, h);
    const Derivative k4 = evaluate_derivative(s4, tau, ext_force, cfg.gravity);

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
    std::vector<SpatialVector> v_local(n); // body-local spatial velocity, for world_motion conversion
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
