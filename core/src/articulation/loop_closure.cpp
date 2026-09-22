#include "loop_closure.h"

#include <algorithm>
#include <cmath>

namespace ps::articulation {

using spatial::SpatialVector;
using spatial::cross_motion;

namespace {

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

// Plain Gauss-Jordan solve with partial pivoting for the (small, m <= a
// handful of constraint rows) Schur-complement system - not the fixed-6
// SmallMatrix (that is sized for a single joint's DoF, <=6; a system with
// several loop closures can exceed that). A small Tikhonov regularisation
// (PLAN.md: correctness first) guards the rare near-singular case (e.g. a
// closure's constraint direction momentarily aligned with a kinematic
// singularity of the open chain) rather than producing a NaN from an
// exact-zero pivot.
std::vector<real> solve_spd(std::vector<real> a, std::vector<real> b, int n) {
    for (int i = 0; i < n; ++i) a[static_cast<std::size_t>(i * n + i)] += real(1e-9);
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        real best = std::abs(a[static_cast<std::size_t>(col * n + col)]);
        for (int r = col + 1; r < n; ++r) {
            const real v = std::abs(a[static_cast<std::size_t>(r * n + col)]);
            if (v > best) { best = v; pivot = r; }
        }
        if (pivot != col) {
            for (int c = 0; c < n; ++c) std::swap(a[static_cast<std::size_t>(col * n + c)], a[static_cast<std::size_t>(pivot * n + c)]);
            std::swap(b[static_cast<std::size_t>(col)], b[static_cast<std::size_t>(pivot)]);
        }
        const real p = a[static_cast<std::size_t>(col * n + col)];
        if (std::abs(p) < real(1e-14)) continue;
        for (int r = col + 1; r < n; ++r) {
            const real f = a[static_cast<std::size_t>(r * n + col)] / p;
            for (int c = col; c < n; ++c) a[static_cast<std::size_t>(r * n + c)] -= f * a[static_cast<std::size_t>(col * n + c)];
            b[static_cast<std::size_t>(r)] -= f * b[static_cast<std::size_t>(col)];
        }
    }
    std::vector<real> x(static_cast<std::size_t>(n), real(0.0));
    for (int r = n - 1; r >= 0; --r) {
        real sum = b[static_cast<std::size_t>(r)];
        for (int c = r + 1; c < n; ++c) sum -= a[static_cast<std::size_t>(r * n + c)] * x[static_cast<std::size_t>(c)];
        const real p = a[static_cast<std::size_t>(r * n + r)];
        x[static_cast<std::size_t>(r)] = (std::abs(p) < real(1e-14)) ? real(0.0) : sum / p;
    }
    return x;
}

Vec3 anchor_world(const std::vector<Pose>& poses, int body, const Vec3& anchor_local) {
    return body < 0 ? anchor_local : poses[static_cast<std::size_t>(body)].transform_point(anchor_local);
}

// World-frame point velocity for `body` (or zero for the fixed world) at
// world offset `r` from the body's own origin, given per-body spatial
// velocities `v_local` (body-local axes, as produced by a kinematic pass
// with an arbitrary qd - real or a probe).
Vec3 point_velocity(const std::vector<Pose>& poses, const std::vector<SpatialVector>& v_local, int body, const Vec3& r) {
    if (body < 0) return Vec3::zero();
    const Quat rot = poses[static_cast<std::size_t>(body)].orientation;
    const Motion m{rot.rotate(v_local[static_cast<std::size_t>(body)].bottom), rot.rotate(v_local[static_cast<std::size_t>(body)].top)};
    return m.velocity_at(r);
}

std::vector<spatial::SpatialTransform> compute_all_xup(const Model& model, const ArticulationState& state) {
    std::vector<spatial::SpatialTransform> xup(model.bodies.size());
    for (std::size_t i = 0; i < model.bodies.size(); ++i) xup[i] = compute_xup(model, static_cast<int>(i), state);
    return xup;
}

std::vector<SpatialVector> local_velocities(const Model& model, const std::vector<spatial::SpatialTransform>& xup, const ArticulationState& state_with_qd) {
    std::vector<SpatialVector> v(model.bodies.size());
    for (std::size_t i = 0; i < model.bodies.size(); ++i) {
        const BodyModel& b = model.bodies[i];
        const SpatialVector v_parent = (b.parent < 0) ? SpatialVector{} : v[static_cast<std::size_t>(b.parent)];
        const SpatialVector vj = ::ps::articulation::joint_velocity(b, state_with_qd);
        v[i] = xup[i].apply_motion(v_parent) + vj;
    }
    return v;
}

// Per-body spatial acceleration with qdd == 0 (i.e. purely the velocity-
// product/Coriolis-centrifugal bias term propagated down the tree), using
// the REAL v (so this is exactly Jdot*qd's per-body building block, no
// gravity term - gravity belongs to qdd_free, computed separately via a
// plain aba() call in the caller).
std::vector<SpatialVector> bias_accel(const Model& model, const std::vector<spatial::SpatialTransform>& xup,
                                       const std::vector<SpatialVector>& v_real, const ArticulationState& real_state) {
    const std::size_t n = model.bodies.size();
    std::vector<SpatialVector> a(n);
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = model.bodies[i];
        const SpatialVector vj = ::ps::articulation::joint_velocity(b, real_state);
        const SpatialVector c = cross_motion(v_real[i], vj);
        const SpatialVector a_parent = (b.parent < 0) ? SpatialVector{} : a[static_cast<std::size_t>(b.parent)];
        a[i] = xup[i].apply_motion(a_parent) + c;
    }
    return a;
}

Vec3 point_bias_accel(const std::vector<Pose>& poses, const std::vector<SpatialVector>& v_local, const std::vector<SpatialVector>& a_bias_local,
                       int body, const Vec3& r) {
    if (body < 0) return Vec3::zero();
    const Quat rot = poses[static_cast<std::size_t>(body)].orientation;
    const Vec3 omega = rot.rotate(v_local[static_cast<std::size_t>(body)].top);
    const Vec3 alpha = rot.rotate(a_bias_local[static_cast<std::size_t>(body)].top);
    const Vec3 a_lin = rot.rotate(a_bias_local[static_cast<std::size_t>(body)].bottom);
    return a_lin + cross(alpha, r) + cross(omega, cross(omega, r));
}

} // namespace

std::vector<Pose> forward_kinematics_poses(const Model& model, const ArticulationState& state) {
    const std::size_t n = model.bodies.size();
    std::vector<Pose> poses(n);
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = model.bodies[i];
        if (model.floating_root && i == 0) {
            poses[i] = state.root_pose;
        } else {
            const Pose parent_pose = (b.parent < 0) ? Pose::identity() : poses[static_cast<std::size_t>(b.parent)];
            poses[i] = parent_pose * b.joint_pose_in_parent * joint_transform_pose(b, state);
        }
    }
    return poses;
}

std::vector<LoopClosureRuntime> build_loop_closure_runtime(const Model& model, const std::vector<int>& link_to_body) {
    std::vector<LoopClosureRuntime> out;
    out.reserve(model.loop_closures.size());
    for (const auto& lc : model.loop_closures) {
        LoopClosureRuntime rt;
        rt.body_a = lc.link_a < 0 ? -1 : link_to_body[static_cast<std::size_t>(lc.link_a)];
        rt.body_b = lc.link_b < 0 ? -1 : link_to_body[static_cast<std::size_t>(lc.link_b)];
        rt.anchor_a = lc.anchor_a;
        rt.anchor_b = lc.anchor_b;
        out.push_back(rt);
    }
    return out;
}

std::vector<real> constraint_error(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures) {
    const std::vector<Pose> poses = forward_kinematics_poses(model, state);
    std::vector<real> g;
    g.reserve(closures.size() * 3);
    for (const auto& c : closures) {
        const Vec3 pa = anchor_world(poses, c.body_a, c.anchor_a);
        const Vec3 pb = anchor_world(poses, c.body_b, c.anchor_b);
        const Vec3 d = pa - pb;
        g.push_back(d.x); g.push_back(d.y); g.push_back(d.z);
    }
    return g;
}

namespace {

// Shared machinery: builds J (m x nv, row-major) and, for every
// constraint row, M^-1*J_row (an nv-vector each) via aba() probes -
// exactly what both solve_constrained_dynamics and
// project_onto_constraints need (the latter also wants J*qd and g(q), the
// former J*qdd_free and Jdot*qd).
struct ClosureLinearAlgebra {
    int m = 0;
    std::vector<real> J;              // m x nv, row-major
    std::vector<std::vector<real>> minv_jrow; // m entries, each an nv-vector = M^-1 * J_row_i
    std::vector<real> A;              // m x m = J * M^-1 * J^T
};

ClosureLinearAlgebra build_linear_algebra(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures) {
    ClosureLinearAlgebra out;
    const int nv = model.num_v;
    out.m = static_cast<int>(closures.size()) * 3;
    if (out.m == 0) return out;

    const std::vector<Pose> poses = forward_kinematics_poses(model, state);
    const std::vector<spatial::SpatialTransform> xup = compute_all_xup(model, state);

    // World-frame anchor offsets from each body's own origin (q-only, same
    // for every probe).
    std::vector<Vec3> r_a(closures.size()), r_b(closures.size());
    for (std::size_t k = 0; k < closures.size(); ++k) {
        const auto& c = closures[k];
        const Vec3 pa = anchor_world(poses, c.body_a, c.anchor_a);
        const Vec3 pb = anchor_world(poses, c.body_b, c.anchor_b);
        r_a[k] = c.body_a < 0 ? Vec3::zero() : pa - poses[static_cast<std::size_t>(c.body_a)].position;
        r_b[k] = c.body_b < 0 ? Vec3::zero() : pb - poses[static_cast<std::size_t>(c.body_b)].position;
    }

    out.J.assign(static_cast<std::size_t>(out.m) * static_cast<std::size_t>(nv), real(0.0));
    ArticulationState probe = state;
    for (int col = 0; col < nv; ++col) {
        std::fill(probe.qd.begin(), probe.qd.end(), real(0.0));
        probe.qd[static_cast<std::size_t>(col)] = real(1.0);
        const std::vector<SpatialVector> v_probe = local_velocities(model, xup, probe);
        for (std::size_t k = 0; k < closures.size(); ++k) {
            const Vec3 va = point_velocity(poses, v_probe, closures[k].body_a, r_a[k]);
            const Vec3 vb = point_velocity(poses, v_probe, closures[k].body_b, r_b[k]);
            const Vec3 d = va - vb;
            out.J[static_cast<std::size_t>((3 * k + 0) * nv + col)] = d.x;
            out.J[static_cast<std::size_t>((3 * k + 1) * nv + col)] = d.y;
            out.J[static_cast<std::size_t>((3 * k + 2) * nv + col)] = d.z;
        }
    }

    out.minv_jrow.assign(static_cast<std::size_t>(out.m), std::vector<real>(static_cast<std::size_t>(nv), real(0.0)));
    ArticulationState zero_v_state = state;
    std::fill(zero_v_state.qd.begin(), zero_v_state.qd.end(), real(0.0));
    for (int row = 0; row < out.m; ++row) {
        std::vector<real> j_row(static_cast<std::size_t>(nv));
        for (int c = 0; c < nv; ++c) j_row[static_cast<std::size_t>(c)] = out.J[static_cast<std::size_t>(row * nv + c)];
        AbaInput ain;
        ain.model = &model;
        ain.state = &zero_v_state;
        ain.tau = j_row;
        ain.gravity = Vec3::zero();
        const AbaOutput aout = aba(ain);
        out.minv_jrow[static_cast<std::size_t>(row)] = aout.qdd;
    }

    out.A.assign(static_cast<std::size_t>(out.m) * static_cast<std::size_t>(out.m), real(0.0));
    for (int i = 0; i < out.m; ++i) {
        for (int j = 0; j < out.m; ++j) {
            real sum = 0.0;
            for (int c = 0; c < nv; ++c) sum += out.J[static_cast<std::size_t>(i * nv + c)] * out.minv_jrow[static_cast<std::size_t>(j)][static_cast<std::size_t>(c)];
            out.A[static_cast<std::size_t>(i * out.m + j)] = sum;
        }
    }
    return out;
}

std::vector<real> apply_j(const ClosureLinearAlgebra& la, const std::vector<real>& v, int nv) {
    std::vector<real> out(static_cast<std::size_t>(la.m), real(0.0));
    for (int r = 0; r < la.m; ++r) {
        real sum = 0.0;
        for (int c = 0; c < nv; ++c) sum += la.J[static_cast<std::size_t>(r * nv + c)] * v[static_cast<std::size_t>(c)];
        out[static_cast<std::size_t>(r)] = sum;
    }
    return out;
}

std::vector<real> apply_minv_jt(const ClosureLinearAlgebra& la, const std::vector<real>& lambda, int nv) {
    std::vector<real> out(static_cast<std::size_t>(nv), real(0.0));
    for (int i = 0; i < la.m; ++i) {
        for (int c = 0; c < nv; ++c) out[static_cast<std::size_t>(c)] += lambda[static_cast<std::size_t>(i)] * la.minv_jrow[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)];
    }
    return out;
}

} // namespace

AbaOutput solve_constrained_dynamics(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                      const std::vector<real>& tau, const Vec3& gravity, const std::vector<SpatialVector>& ext_force) {
    AbaInput ain;
    ain.model = &model;
    ain.state = &state;
    ain.tau = tau;
    ain.gravity = gravity;
    ain.ext_force = ext_force;
    AbaOutput free_out = aba(ain);
    if (closures.empty()) return free_out;

    const int nv = model.num_v;
    const ClosureLinearAlgebra la = build_linear_algebra(model, state, closures);

    // Jdot*qd per closure (bias point acceleration at qdd=0, real qd).
    const std::vector<Pose> poses = forward_kinematics_poses(model, state);
    const std::vector<spatial::SpatialTransform> xup = compute_all_xup(model, state);
    const std::vector<SpatialVector> v_real = local_velocities(model, xup, state);
    const std::vector<SpatialVector> a_bias = bias_accel(model, xup, v_real, state);

    std::vector<real> jdot_qd(static_cast<std::size_t>(la.m), real(0.0));
    for (std::size_t k = 0; k < closures.size(); ++k) {
        const Vec3 pa = anchor_world(poses, closures[k].body_a, closures[k].anchor_a);
        const Vec3 pb = anchor_world(poses, closures[k].body_b, closures[k].anchor_b);
        const Vec3 ra = closures[k].body_a < 0 ? Vec3::zero() : pa - poses[static_cast<std::size_t>(closures[k].body_a)].position;
        const Vec3 rb = closures[k].body_b < 0 ? Vec3::zero() : pb - poses[static_cast<std::size_t>(closures[k].body_b)].position;
        const Vec3 ba = point_bias_accel(poses, v_real, a_bias, closures[k].body_a, ra);
        const Vec3 bb = point_bias_accel(poses, v_real, a_bias, closures[k].body_b, rb);
        const Vec3 d = ba - bb;
        jdot_qd[3 * k + 0] = d.x; jdot_qd[3 * k + 1] = d.y; jdot_qd[3 * k + 2] = d.z;
    }

    const std::vector<real> j_qdd_free = apply_j(la, free_out.qdd, nv);
    std::vector<real> rhs(static_cast<std::size_t>(la.m));
    for (int i = 0; i < la.m; ++i) rhs[static_cast<std::size_t>(i)] = -(j_qdd_free[static_cast<std::size_t>(i)] + jdot_qd[static_cast<std::size_t>(i)]);

    const std::vector<real> lambda = solve_spd(la.A, rhs, la.m);
    const std::vector<real> delta = apply_minv_jt(la, lambda, nv);

    AbaOutput out;
    out.qdd.assign(static_cast<std::size_t>(nv), real(0.0));
    for (int i = 0; i < nv; ++i) out.qdd[static_cast<std::size_t>(i)] = free_out.qdd[static_cast<std::size_t>(i)] + delta[static_cast<std::size_t>(i)];
    return out;
}

void project_velocity_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures) {
    if (closures.empty()) return;
    const int nv = model.num_v;
    const ClosureLinearAlgebra la = build_linear_algebra(model, state, closures);

    // Orthogonal projection in the mass inner product: removes the
    // constraint-violating component of qd, so 0.5*qd^T*M*qd can only
    // decrease (never increase) - see loop_closure.h's file comment.
    const std::vector<real> j_qd = apply_j(la, state.qd, nv);
    std::vector<real> rhs_v(static_cast<std::size_t>(la.m));
    for (int i = 0; i < la.m; ++i) rhs_v[static_cast<std::size_t>(i)] = -j_qd[static_cast<std::size_t>(i)];
    const std::vector<real> mu_v = solve_spd(la.A, rhs_v, la.m);
    const std::vector<real> dqd = apply_minv_jt(la, mu_v, nv);
    for (int i = 0; i < nv; ++i) state.qd[static_cast<std::size_t>(i)] += dqd[static_cast<std::size_t>(i)];
}

void project_position_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures) {
    if (closures.empty()) return;
    const int nv = model.num_v;
    const ClosureLinearAlgebra la = build_linear_algebra(model, state, closures);

    // First-order Newton step toward g(q) == 0, mass-weighted (PLAN.md:
    // not a plain/unweighted projection).
    const std::vector<real> g = constraint_error(model, state, closures);
    std::vector<real> rhs_q(static_cast<std::size_t>(la.m));
    for (int i = 0; i < la.m; ++i) rhs_q[static_cast<std::size_t>(i)] = -g[static_cast<std::size_t>(i)];
    const std::vector<real> mu_q = solve_spd(la.A, rhs_q, la.m);
    const std::vector<real> dq = apply_minv_jt(la, mu_q, nv);

    for (const auto& b : model.bodies) {
        if (b.ndof == 1) state.q[static_cast<std::size_t>(b.q_index)] += dq[static_cast<std::size_t>(b.dof_index)];
    }
    if (model.floating_root) {
        const Vec3 w{dq[0], dq[1], dq[2]};
        const Vec3 v{dq[3], dq[4], dq[5]};
        state.root_pose.position += v;
        const real angle = w.length();
        if (angle > real(1e-14)) state.root_pose.orientation = (state.root_pose.orientation * Quat::from_axis_angle(w / angle, angle)).normalized();
    }
}

} // namespace ps::articulation
