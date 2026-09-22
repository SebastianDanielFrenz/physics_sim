#include "loop_closure.h"

#include <algorithm>
#include <cmath>

namespace ps::articulation {

using spatial::SpatialVector;
using spatial::cross_motion;

void LoopClosureScratch::ensure(const Model& model, std::size_t num_closures) {
    const std::size_t n = model.bodies.size();
    const std::size_t nv = static_cast<std::size_t>(model.num_v);
    const std::size_t nq = static_cast<std::size_t>(model.num_q);
    const std::size_t m = num_closures * 3;

    la.m = static_cast<int>(m);
    la.J.assign(m * nv, real(0.0));
    la.minv_jrow.assign(m * nv, real(0.0));
    la.A.assign(m * m, real(0.0));

    poses.assign(n, Pose::identity());
    xup.assign(n, spatial::SpatialTransform{});
    r_a.assign(num_closures, Vec3::zero());
    r_b.assign(num_closures, Vec3::zero());
    v_probe.assign(n, SpatialVector{});
    v_real.assign(n, SpatialVector{});
    a_bias.assign(n, SpatialVector{});

    probe_state.root_pose = Pose::identity();
    probe_state.q.assign(nq, real(0.0));
    probe_state.qd.assign(nv, real(0.0));
    zero_v_state.root_pose = Pose::identity();
    zero_v_state.q.assign(nq, real(0.0));
    zero_v_state.qd.assign(nv, real(0.0));

    j_row.assign(nv, real(0.0));
    g.assign(m, real(0.0));
    jdot_qd.assign(m, real(0.0));
    free_qdd.assign(nv, real(0.0));
    probe_qdd.assign(nv, real(0.0));
    j_qdd_free.assign(m, real(0.0));
    rhs.assign(m, real(0.0));
    mu.assign(m, real(0.0));
    delta.assign(nv, real(0.0));
    spd_a.assign(m * m, real(0.0));
    spd_b.assign(m, real(0.0));
}

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
//
// In-place variant (PHYS-009 Step 3): operates on caller-owned `a` (a
// mutable copy of A the caller must have already filled - this function
// destroys it) and `b` (likewise a mutable copy of the rhs), writing the
// solution into caller-owned `x`. No allocation of its own; the two
// call sites below either reuse LoopClosureScratch::spd_a/spd_b/mu (hot
// path) or a same-shaped local (non-hot-path, still allocates once, same
// as the rest of this file's "no scratch given" fallback).
void solve_spd_into(std::vector<real>& a, std::vector<real>& b, int n, std::vector<real>& x) {
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
    for (int r = n - 1; r >= 0; --r) {
        real sum = b[static_cast<std::size_t>(r)];
        for (int c = r + 1; c < n; ++c) sum -= a[static_cast<std::size_t>(r * n + c)] * x[static_cast<std::size_t>(c)];
        const real p = a[static_cast<std::size_t>(r * n + r)];
        x[static_cast<std::size_t>(r)] = (std::abs(p) < real(1e-14)) ? real(0.0) : sum / p;
    }
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

void compute_all_xup_into(const Model& model, const ArticulationState& state, std::vector<spatial::SpatialTransform>& xup) {
    xup.resize(model.bodies.size());
    for (std::size_t i = 0; i < model.bodies.size(); ++i) xup[i] = compute_xup(model, static_cast<int>(i), state);
}

void local_velocities_into(const Model& model, const std::vector<spatial::SpatialTransform>& xup, const ArticulationState& state_with_qd,
                            std::vector<SpatialVector>& v) {
    v.resize(model.bodies.size());
    for (std::size_t i = 0; i < model.bodies.size(); ++i) {
        const BodyModel& b = model.bodies[i];
        const SpatialVector v_parent = (b.parent < 0) ? SpatialVector{} : v[static_cast<std::size_t>(b.parent)];
        const SpatialVector vj = ::ps::articulation::joint_velocity(b, state_with_qd);
        v[i] = xup[i].apply_motion(v_parent) + vj;
    }
}

// Per-body spatial acceleration with qdd == 0 (i.e. purely the velocity-
// product/Coriolis-centrifugal bias term propagated down the tree), using
// the REAL v (so this is exactly Jdot*qd's per-body building block, no
// gravity term - gravity belongs to qdd_free, computed separately via a
// plain aba() call in the caller).
void bias_accel_into(const Model& model, const std::vector<spatial::SpatialTransform>& xup, const std::vector<SpatialVector>& v_real,
                      const ArticulationState& real_state, std::vector<SpatialVector>& a) {
    const std::size_t n = model.bodies.size();
    a.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = model.bodies[i];
        const SpatialVector vj = ::ps::articulation::joint_velocity(b, real_state);
        const SpatialVector c = cross_motion(v_real[i], vj);
        const SpatialVector a_parent = (b.parent < 0) ? SpatialVector{} : a[static_cast<std::size_t>(b.parent)];
        a[i] = xup[i].apply_motion(a_parent) + c;
    }
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

void forward_kinematics_poses_into(const Model& model, const ArticulationState& state, std::vector<Pose>& out) {
    const std::size_t n = model.bodies.size();
    out.resize(n); // no-op at a stable n (PHYS-009 Step 1 rule 2)
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = model.bodies[i];
        if (model.floating_root && i == 0) {
            out[i] = state.root_pose;
        } else {
            const Pose parent_pose = (b.parent < 0) ? Pose::identity() : out[static_cast<std::size_t>(b.parent)];
            out[i] = parent_pose * b.joint_pose_in_parent * joint_transform_pose(b, state);
        }
    }
}

std::vector<Pose> forward_kinematics_poses(const Model& model, const ArticulationState& state) {
    std::vector<Pose> poses;
    forward_kinematics_poses_into(model, state, poses);
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

void constraint_error_into(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                            std::vector<Pose>& poses_scratch, std::vector<real>& out) {
    forward_kinematics_poses_into(model, state, poses_scratch);
    out.resize(closures.size() * 3);
    for (std::size_t k = 0; k < closures.size(); ++k) {
        const Vec3 pa = anchor_world(poses_scratch, closures[k].body_a, closures[k].anchor_a);
        const Vec3 pb = anchor_world(poses_scratch, closures[k].body_b, closures[k].anchor_b);
        const Vec3 d = pa - pb;
        out[3 * k + 0] = d.x; out[3 * k + 1] = d.y; out[3 * k + 2] = d.z;
    }
}

std::vector<real> constraint_error(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures) {
    std::vector<Pose> poses;
    std::vector<real> g;
    constraint_error_into(model, state, closures, poses, g);
    return g;
}

namespace {

// Shared machinery: builds J (m x nv, row-major) and, for every constraint
// row, M^-1*J_row (an nv-vector each, row-major in lc.la.minv_jrow) via
// aba_into() probes - exactly what both solve_constrained_dynamics_into and
// project_onto_constraints need (the latter also wants J*qd and g(q), the
// former J*qdd_free and Jdot*qd). Writes into lc.la; every other lc member
// used along the way (poses/xup/r_a/r_b/v_probe/probe_state/zero_v_state/
// j_row) is scratch reused in place, and `aba_scratch->ain` is reused as
// the throwaway per-probe AbaInput (PHYS-009 Step 3: a freshly
// default-constructed AbaInput would allocate its own tau/ext_force vectors
// every probe - m * nv of them per call otherwise).
void build_linear_algebra(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                           AbaScratch& aba_scratch, LoopClosureScratch& lc) {
    ClosureLinearAlgebra& out = lc.la;
    const int nv = model.num_v;
    out.m = static_cast<int>(closures.size()) * 3;
    if (out.m == 0) return;

    forward_kinematics_poses_into(model, state, lc.poses);
    compute_all_xup_into(model, state, lc.xup);

    // World-frame anchor offsets from each body's own origin (q-only, same
    // for every probe).
    lc.r_a.resize(closures.size());
    lc.r_b.resize(closures.size());
    for (std::size_t k = 0; k < closures.size(); ++k) {
        const auto& c = closures[k];
        const Vec3 pa = anchor_world(lc.poses, c.body_a, c.anchor_a);
        const Vec3 pb = anchor_world(lc.poses, c.body_b, c.anchor_b);
        lc.r_a[k] = c.body_a < 0 ? Vec3::zero() : pa - lc.poses[static_cast<std::size_t>(c.body_a)].position;
        lc.r_b[k] = c.body_b < 0 ? Vec3::zero() : pb - lc.poses[static_cast<std::size_t>(c.body_b)].position;
    }

    const std::size_t m = static_cast<std::size_t>(out.m);
    const std::size_t nv_u = static_cast<std::size_t>(nv);
    out.J.assign(m * nv_u, real(0.0));

    lc.probe_state.root_pose = state.root_pose;
    lc.probe_state.q = state.q;
    std::fill(lc.probe_state.qd.begin(), lc.probe_state.qd.end(), real(0.0));
    for (int col = 0; col < nv; ++col) {
        if (col > 0) lc.probe_state.qd[static_cast<std::size_t>(col - 1)] = real(0.0);
        lc.probe_state.qd[static_cast<std::size_t>(col)] = real(1.0);
        local_velocities_into(model, lc.xup, lc.probe_state, lc.v_probe);
        for (std::size_t k = 0; k < closures.size(); ++k) {
            const Vec3 va = point_velocity(lc.poses, lc.v_probe, closures[k].body_a, lc.r_a[k]);
            const Vec3 vb = point_velocity(lc.poses, lc.v_probe, closures[k].body_b, lc.r_b[k]);
            const Vec3 d = va - vb;
            out.J[(3 * k + 0) * nv_u + static_cast<std::size_t>(col)] = d.x;
            out.J[(3 * k + 1) * nv_u + static_cast<std::size_t>(col)] = d.y;
            out.J[(3 * k + 2) * nv_u + static_cast<std::size_t>(col)] = d.z;
        }
    }

    out.minv_jrow.assign(m * nv_u, real(0.0));
    lc.zero_v_state.root_pose = state.root_pose;
    lc.zero_v_state.q = state.q;
    std::fill(lc.zero_v_state.qd.begin(), lc.zero_v_state.qd.end(), real(0.0));
    lc.j_row.resize(nv_u);
    for (int row = 0; row < out.m; ++row) {
        for (int c = 0; c < nv; ++c) lc.j_row[static_cast<std::size_t>(c)] = out.J[static_cast<std::size_t>(row) * nv_u + static_cast<std::size_t>(c)];
        aba_scratch.ain.model = &model;
        aba_scratch.ain.state = &lc.zero_v_state;
        aba_scratch.ain.tau = lc.j_row;
        aba_scratch.ain.gravity = Vec3::zero();
        aba_scratch.ain.ext_force.clear();
        aba_into(aba_scratch.ain, aba_scratch, lc.probe_qdd);
        std::copy(lc.probe_qdd.begin(), lc.probe_qdd.end(), out.minv_jrow.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(row) * nv_u));
    }

    out.A.assign(m * m, real(0.0));
    for (int i = 0; i < out.m; ++i) {
        for (int j = 0; j < out.m; ++j) {
            real sum = 0.0;
            for (int c = 0; c < nv; ++c) {
                sum += out.J[static_cast<std::size_t>(i) * nv_u + static_cast<std::size_t>(c)] *
                       out.minv_jrow[static_cast<std::size_t>(j) * nv_u + static_cast<std::size_t>(c)];
            }
            out.A[static_cast<std::size_t>(i * out.m + j)] = sum;
        }
    }
}

void apply_j_into(const ClosureLinearAlgebra& la, const std::vector<real>& v, int nv, std::vector<real>& out) {
    out.resize(static_cast<std::size_t>(la.m));
    for (int r = 0; r < la.m; ++r) {
        real sum = 0.0;
        for (int c = 0; c < nv; ++c) sum += la.J[static_cast<std::size_t>(r * nv + c)] * v[static_cast<std::size_t>(c)];
        out[static_cast<std::size_t>(r)] = sum;
    }
}

void apply_minv_jt_into(const ClosureLinearAlgebra& la, const std::vector<real>& lambda, int nv, std::vector<real>& out) {
    out.assign(static_cast<std::size_t>(nv), real(0.0));
    for (int i = 0; i < la.m; ++i) {
        for (int c = 0; c < nv; ++c) {
            out[static_cast<std::size_t>(c)] += lambda[static_cast<std::size_t>(i)] * la.minv_jrow[static_cast<std::size_t>(i * nv + c)];
        }
    }
}

// Core constrained-dynamics solve, entirely in terms of lc's preallocated
// buffers - both solve_constrained_dynamics_into (hot path) and
// solve_constrained_dynamics (the AbaOutput-returning compatibility
// wrapper, non-hot-path) funnel through this.
void solve_constrained_dynamics_core(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                      const std::vector<real>& tau, const Vec3& gravity, const std::vector<SpatialVector>& ext_force,
                                      AbaScratch& scratch, LoopClosureScratch& lc, std::vector<real>& qdd_out) {
    const int nv = model.num_v;
    scratch.ain.model = &model;
    scratch.ain.state = &state;
    scratch.ain.tau = tau;
    scratch.ain.gravity = gravity;
    scratch.ain.ext_force = ext_force;
    aba_into(scratch.ain, scratch, lc.free_qdd);
    if (closures.empty()) {
        qdd_out = lc.free_qdd;
        return;
    }

    build_linear_algebra(model, state, closures, scratch, lc);
    const ClosureLinearAlgebra& la = lc.la;

    // Jdot*qd per closure (bias point acceleration at qdd=0, real qd).
    forward_kinematics_poses_into(model, state, lc.poses);
    compute_all_xup_into(model, state, lc.xup);
    local_velocities_into(model, lc.xup, state, lc.v_real);
    bias_accel_into(model, lc.xup, lc.v_real, state, lc.a_bias);

    lc.jdot_qd.assign(static_cast<std::size_t>(la.m), real(0.0));
    for (std::size_t k = 0; k < closures.size(); ++k) {
        const Vec3 pa = anchor_world(lc.poses, closures[k].body_a, closures[k].anchor_a);
        const Vec3 pb = anchor_world(lc.poses, closures[k].body_b, closures[k].anchor_b);
        const Vec3 ra = closures[k].body_a < 0 ? Vec3::zero() : pa - lc.poses[static_cast<std::size_t>(closures[k].body_a)].position;
        const Vec3 rb = closures[k].body_b < 0 ? Vec3::zero() : pb - lc.poses[static_cast<std::size_t>(closures[k].body_b)].position;
        const Vec3 ba = point_bias_accel(lc.poses, lc.v_real, lc.a_bias, closures[k].body_a, ra);
        const Vec3 bb = point_bias_accel(lc.poses, lc.v_real, lc.a_bias, closures[k].body_b, rb);
        const Vec3 d = ba - bb;
        lc.jdot_qd[3 * k + 0] = d.x; lc.jdot_qd[3 * k + 1] = d.y; lc.jdot_qd[3 * k + 2] = d.z;
    }

    apply_j_into(la, lc.free_qdd, nv, lc.j_qdd_free);
    lc.rhs.resize(static_cast<std::size_t>(la.m));
    for (int i = 0; i < la.m; ++i) lc.rhs[static_cast<std::size_t>(i)] = -(lc.j_qdd_free[static_cast<std::size_t>(i)] + lc.jdot_qd[static_cast<std::size_t>(i)]);

    lc.spd_a = la.A; // mutable copy for solve_spd_into's in-place elimination
    lc.spd_b = lc.rhs;
    lc.mu.resize(static_cast<std::size_t>(la.m));
    solve_spd_into(lc.spd_a, lc.spd_b, la.m, lc.mu);
    apply_minv_jt_into(la, lc.mu, nv, lc.delta);

    qdd_out.resize(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i) qdd_out[static_cast<std::size_t>(i)] = lc.free_qdd[static_cast<std::size_t>(i)] + lc.delta[static_cast<std::size_t>(i)];
}

// Shared by project_velocity_onto_constraints/project_position_onto_constraints:
// solves A*mu = rhs (rhs already written into lc.rhs by the caller) and
// returns M^-1*J^T*mu in lc.delta.
void solve_projection_core(const ClosureLinearAlgebra& la, LoopClosureScratch& lc, int nv) {
    lc.spd_a = la.A;
    lc.spd_b = lc.rhs;
    lc.mu.resize(static_cast<std::size_t>(la.m));
    solve_spd_into(lc.spd_a, lc.spd_b, la.m, lc.mu);
    apply_minv_jt_into(la, lc.mu, nv, lc.delta);
}

} // namespace

AbaOutput solve_constrained_dynamics(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                      const std::vector<real>& tau, const Vec3& gravity, const std::vector<SpatialVector>& ext_force,
                                      AbaScratch* scratch, LoopClosureScratch* lc_scratch) {
    AbaScratch local_aba;
    AbaScratch& aba_ref = scratch ? *scratch : local_aba;
    if (!scratch) aba_ref.ensure(model.bodies.size());

    LoopClosureScratch local_lc;
    LoopClosureScratch& lc_ref = lc_scratch ? *lc_scratch : local_lc;
    if (!lc_scratch) lc_ref.ensure(model, closures.size());

    AbaOutput out;
    solve_constrained_dynamics_core(model, state, closures, tau, gravity, ext_force, aba_ref, lc_ref, out.qdd);
    return out;
}

void solve_constrained_dynamics_into(const Model& model, const ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                      const std::vector<real>& tau, const Vec3& gravity, const std::vector<SpatialVector>& ext_force,
                                      AbaScratch& scratch, std::vector<real>& qdd_out, LoopClosureScratch& lc_scratch) {
    solve_constrained_dynamics_core(model, state, closures, tau, gravity, ext_force, scratch, lc_scratch, qdd_out);
}

void project_velocity_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                        AbaScratch* scratch, LoopClosureScratch* lc_scratch) {
    if (closures.empty()) return;
    AbaScratch local_aba;
    AbaScratch& aba_ref = scratch ? *scratch : local_aba;
    if (!scratch) aba_ref.ensure(model.bodies.size());
    LoopClosureScratch local_lc;
    LoopClosureScratch& lc = lc_scratch ? *lc_scratch : local_lc;
    if (!lc_scratch) lc.ensure(model, closures.size());

    const int nv = model.num_v;
    build_linear_algebra(model, state, closures, aba_ref, lc);
    const ClosureLinearAlgebra& la = lc.la;

    // Orthogonal projection in the mass inner product: removes the
    // constraint-violating component of qd, so 0.5*qd^T*M*qd can only
    // decrease (never increase) - see loop_closure.h's file comment.
    apply_j_into(la, state.qd, nv, lc.j_qdd_free); // borrowed as scratch: J*qd, not live across this call
    lc.rhs.resize(static_cast<std::size_t>(la.m));
    for (int i = 0; i < la.m; ++i) lc.rhs[static_cast<std::size_t>(i)] = -lc.j_qdd_free[static_cast<std::size_t>(i)];
    solve_projection_core(la, lc, nv);
    for (int i = 0; i < nv; ++i) state.qd[static_cast<std::size_t>(i)] += lc.delta[static_cast<std::size_t>(i)];
}

void project_position_onto_constraints(const Model& model, ArticulationState& state, const std::vector<LoopClosureRuntime>& closures,
                                        AbaScratch* scratch, LoopClosureScratch* lc_scratch) {
    if (closures.empty()) return;
    AbaScratch local_aba;
    AbaScratch& aba_ref = scratch ? *scratch : local_aba;
    if (!scratch) aba_ref.ensure(model.bodies.size());
    LoopClosureScratch local_lc;
    LoopClosureScratch& lc = lc_scratch ? *lc_scratch : local_lc;
    if (!lc_scratch) lc.ensure(model, closures.size());

    const int nv = model.num_v;
    build_linear_algebra(model, state, closures, aba_ref, lc);
    const ClosureLinearAlgebra& la = lc.la;

    // First-order Newton step toward g(q) == 0, mass-weighted (PLAN.md:
    // not a plain/unweighted projection).
    constraint_error_into(model, state, closures, lc.poses, lc.g);
    lc.rhs.resize(static_cast<std::size_t>(la.m));
    for (int i = 0; i < la.m; ++i) lc.rhs[static_cast<std::size_t>(i)] = -lc.g[static_cast<std::size_t>(i)];
    solve_projection_core(la, lc, nv);
    const std::vector<real>& dq = lc.delta;

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
