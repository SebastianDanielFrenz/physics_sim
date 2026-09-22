#include "aba.h"
#include "small_matrix.h"

#include <vector>

namespace ps::articulation {

using spatial::SpatialInertia;
using spatial::SpatialTransform;
using spatial::SpatialVector;
using spatial::cross_force;
using spatial::cross_motion;
using spatial::spatial_dot;

namespace {

// Dense, general 6x6 spatial operator - needed only for the articulated-
// body inertia IA (Featherstone ch. 7 notes explicitly that after the
// first rank reduction (see below) IA generally no longer has the
// restricted (mass, first-moment, rotational-inertia-about-a-point)
// structure a single rigid body's SpatialInertia assumes, so it cannot be
// carried in that compact form the way CRBA's composite inertia can).
struct Mat6 {
    real m[6][6] = {};

    static Mat6 from_rigid(const SpatialInertia& i) {
        Mat6 out;
        const Mat3 hx = skew(i.h);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                out.m[r][c] = i.io(r, c);
                out.m[r][c + 3] = hx(r, c);
                out.m[r + 3][c] = -hx(r, c);
                out.m[r + 3][c + 3] = (r == c) ? i.mass : real(0.0);
            }
        }
        return out;
    }

    [[nodiscard]] SpatialVector apply(const SpatialVector& v) const {
        const real x[6] = {v.top.x, v.top.y, v.top.z, v.bottom.x, v.bottom.y, v.bottom.z};
        real y[6] = {};
        for (int r = 0; r < 6; ++r) {
            real s = 0.0;
            for (int c = 0; c < 6; ++c) s += m[r][c] * x[c];
            y[r] = s;
        }
        return SpatialVector{Vec3{y[0], y[1], y[2]}, Vec3{y[3], y[4], y[5]}};
    }

    Mat6 operator-(const Mat6& o) const {
        Mat6 r;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) r.m[i][j] = m[i][j] - o.m[i][j];
        return r;
    }

    Mat6 operator+(const Mat6& o) const {
        Mat6 r;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) r.m[i][j] = m[i][j] + o.m[i][j];
        return r;
    }

    Mat6 operator*(const Mat6& o) const {
        Mat6 r;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) {
                real s = 0.0;
                for (int k = 0; k < 6; ++k) s += m[i][k] * o.m[k][j];
                r.m[i][j] = s;
            }
        return r;
    }

    // Column c is X applied to the c-th canonical basis spatial vector -
    // i.e. the dense matrix representation of the linear map `apply`.
    static Mat6 from_linear_map(const SpatialTransform& x, bool force_map) {
        Mat6 out;
        const SpatialVector basis[6] = {
            {Vec3::unit_x(), Vec3::zero()}, {Vec3::unit_y(), Vec3::zero()}, {Vec3::unit_z(), Vec3::zero()},
            {Vec3::zero(), Vec3::unit_x()}, {Vec3::zero(), Vec3::unit_y()}, {Vec3::zero(), Vec3::unit_z()},
        };
        for (int c = 0; c < 6; ++c) {
            const SpatialVector col = force_map ? x.apply_force(basis[static_cast<std::size_t>(c)])
                                                 : x.apply_motion(basis[static_cast<std::size_t>(c)]);
            const real v[6] = {col.top.x, col.top.y, col.top.z, col.bottom.x, col.bottom.y, col.bottom.z};
            for (int r = 0; r < 6; ++r) out.m[r][c] = v[r];
        }
        return out;
    }
};

// Transforms a child-frame Mat6 "inertia-like" operator up into the
// parent's frame, given Xup (parent -> child, motion sense): M_A = Xf(A<-B) * M_B * Xm(A->B),
// where Xf(A<-B) is the force map of Xup's inverse (child->parent) and
// Xm(A->B) is Xup's own motion map (parent->child) - see aba.h's file
// comment / model.h for the underlying Xup convention.
Mat6 transform_inertia_to_parent(const SpatialTransform& xup, const Mat6& child) {
    const Mat6 f_child_to_parent = Mat6::from_linear_map(xup.inverse(), /*force_map=*/true);
    const Mat6 m_parent_to_child = Mat6::from_linear_map(xup, /*force_map=*/false);
    return f_child_to_parent * child * m_parent_to_child;
}

} // namespace

namespace {

// Per-body DoF-indexed scratch (U columns, u entries): fixed capacity
// kMaxJointDof (== 6, a floating base is the largest joint this project
// has) instead of a std::vector, so the tip->base / base->tip passes below
// never heap-allocate per body (PLAN.md D5 "fix the biggest costs first" /
// PHYS-009 Step 2 profiling: this - plus the per-call n-sized vectors -
// was the dominant cost of the original implementation, one heap
// allocation of some kind for essentially every body on every aba() call).
struct DofScratch {
    std::array<SpatialVector, kMaxJointDof> U{};
    std::array<real, kMaxJointDof> u{};
};

// Reusable per-thread scratch for the n-sized arrays every aba() call
// needs. thread_local (not just static) because, although System::step_all
// steps articulations sequentially today (tests/determinism/
// test_worker_count_hash.cpp's comment: articulation stepping never
// touches the job system), aba() has no way to know that and must stay
// safe if that changes later; a plain function-local static vector would
// be a data race the moment two threads call aba() concurrently. Each
// thread that ever calls aba() pays the vector-growth allocation once
// (first call, or first call after the largest n seen so far), then
// .resize() to an equal-or-smaller n is a no-op (capacity is kept, no
// reallocation, no deallocation) - the whole point of reusing these across
// calls instead of declaring them fresh every time.
struct AbaScratch {
    std::vector<SpatialTransform> xup;
    std::vector<SpatialVector> v;
    std::vector<SpatialVector> c; // velocity-product bias term, v_i x (S_i*qd_i)
    std::vector<Mat6> IA;
    std::vector<SpatialVector> pA;
    std::vector<DofScratch> dof;
    std::vector<SmallMatrix> Dinv;
    std::vector<SpatialVector> a;

    void ensure(std::size_t n) {
        xup.resize(n);
        v.resize(n);
        c.resize(n);
        IA.resize(n);
        pA.resize(n);
        dof.resize(n);
        Dinv.resize(n);
        a.resize(n);
    }
};

} // namespace

AbaOutput aba(const AbaInput& in) {
    const Model& m = *in.model;
    const ArticulationState& st = *in.state;
    const std::size_t n = m.bodies.size();

    AbaOutput out;
    out.qdd.assign(static_cast<std::size_t>(m.num_v), real(0.0));

    thread_local AbaScratch sc;
    sc.ensure(n);
    std::vector<SpatialTransform>& xup = sc.xup;
    std::vector<SpatialVector>& v = sc.v;
    std::vector<SpatialVector>& c = sc.c;
    std::vector<Mat6>& IA = sc.IA;
    std::vector<SpatialVector>& pA = sc.pA;
    std::vector<DofScratch>& dof = sc.dof;
    std::vector<SmallMatrix>& Dinv = sc.Dinv;

    const SpatialVector a_world{Vec3::zero(), -in.gravity};

    // Pass 1 (base -> tip): velocities and the initial (unreduced)
    // articulated-body inertia/bias force at each body.
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = m.bodies[i];
        xup[i] = compute_xup(m, static_cast<int>(i), st);
        const SpatialVector v_parent = (b.parent < 0) ? SpatialVector{} : v[static_cast<std::size_t>(b.parent)];
        const SpatialVector vj = joint_velocity(b, st);
        v[i] = xup[i].apply_motion(v_parent) + vj;
        c[i] = cross_motion(v[i], vj);

        IA[i] = Mat6::from_rigid(b.inertia);
        const SpatialVector f_ext = in.ext_force.empty() ? SpatialVector{} : in.ext_force[i];
        pA[i] = cross_force(v[i], b.inertia * v[i]) - f_ext;
    }

    // Pass 2 (tip -> base): reduce each body's inertia/bias force in terms
    // of its own DoF (if any) and fold the remainder into the parent.
    for (std::size_t idx = n; idx-- > 0;) {
        const BodyModel& b = m.bodies[idx];
        if (b.ndof > 0) {
            const MotionSubspace s = motion_subspace(b);
            const std::size_t ns = static_cast<std::size_t>(s.count);
            DofScratch& ds = dof[idx];
            for (std::size_t a = 0; a < ns; ++a) ds.U[a] = IA[idx].apply(s.cols[a]);

            SmallMatrix D = SmallMatrix::zero(static_cast<int>(ns), static_cast<int>(ns));
            for (std::size_t a = 0; a < ns; ++a)
                for (std::size_t bcol = 0; bcol < ns; ++bcol) D.at(static_cast<int>(a), static_cast<int>(bcol)) = spatial_dot(s.cols[a], ds.U[bcol]);
            Dinv[idx] = D.inverse();

            for (std::size_t a = 0; a < ns; ++a) {
                const real tau_a = (b.joint == BodyJointKind::Floating6) ? in.tau[a] : in.tau[static_cast<std::size_t>(b.dof_index)];
                ds.u[a] = tau_a - spatial_dot(s.cols[a], pA[idx]);
            }

            // Ia = IA_i - U*Dinv*U^T (rank-|s| reduction); pa = pA_i + Ia*c_i + U*Dinv*u.
            Mat6 udu;
            for (int r = 0; r < 6; ++r)
                for (int cc = 0; cc < 6; ++cc) {
                    real sum = 0.0;
                    for (std::size_t a = 0; a < ns; ++a)
                        for (std::size_t bcol = 0; bcol < ns; ++bcol) {
                            const real ur[6] = {ds.U[a].top.x, ds.U[a].top.y, ds.U[a].top.z, ds.U[a].bottom.x, ds.U[a].bottom.y, ds.U[a].bottom.z};
                            const real uc[6] = {ds.U[bcol].top.x, ds.U[bcol].top.y, ds.U[bcol].top.z, ds.U[bcol].bottom.x, ds.U[bcol].bottom.y, ds.U[bcol].bottom.z};
                            sum += ur[r] * Dinv[idx].at(static_cast<int>(a), static_cast<int>(bcol)) * uc[cc];
                        }
                    udu.m[r][cc] = sum;
                }
            const Mat6 Ia = IA[idx] - udu;

            std::array<real, kMaxJointDof> dinv_u{};
            for (std::size_t a = 0; a < ns; ++a) {
                real sum = 0.0;
                for (std::size_t bcol = 0; bcol < ns; ++bcol) sum += Dinv[idx].at(static_cast<int>(a), static_cast<int>(bcol)) * ds.u[bcol];
                dinv_u[a] = sum;
            }
            SpatialVector u_term{};
            for (std::size_t a = 0; a < ns; ++a) u_term += ds.U[a] * dinv_u[a];
            const SpatialVector pa = pA[idx] + Ia.apply(c[idx]) + u_term;

            if (b.parent >= 0) {
                IA[static_cast<std::size_t>(b.parent)] = IA[static_cast<std::size_t>(b.parent)] + transform_inertia_to_parent(xup[idx], Ia);
                pA[static_cast<std::size_t>(b.parent)] += xup[idx].apply_force_inverse(pa);
            }
        } else {
            // Fixed (0-DoF) joint: nothing to reduce, fold straight up.
            if (b.parent >= 0) {
                IA[static_cast<std::size_t>(b.parent)] = IA[static_cast<std::size_t>(b.parent)] + transform_inertia_to_parent(xup[idx], IA[idx]);
                pA[static_cast<std::size_t>(b.parent)] += xup[idx].apply_force_inverse(pA[idx]);
            }
        }
    }

    // Pass 3 (base -> tip): resolve accelerations and generalized accelerations.
    std::vector<SpatialVector>& a = sc.a;
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = m.bodies[i];
        const SpatialVector a_parent = (b.parent < 0) ? a_world : a[static_cast<std::size_t>(b.parent)];
        a[i] = xup[i].apply_motion(a_parent) + c[i];

        if (b.ndof > 0) {
            const MotionSubspace s = motion_subspace(b);
            const std::size_t ns = static_cast<std::size_t>(s.count);
            const DofScratch& ds = dof[i];
            std::array<real, kMaxJointDof> rhs{};
            for (std::size_t k = 0; k < ns; ++k) rhs[k] = ds.u[k] - spatial_dot(a[i], ds.U[k]);
            std::array<real, kMaxJointDof> qdd_local{};
            for (std::size_t r = 0; r < ns; ++r) {
                real sum = 0.0;
                for (std::size_t cidx = 0; cidx < ns; ++cidx) sum += Dinv[i].at(static_cast<int>(r), static_cast<int>(cidx)) * rhs[cidx];
                qdd_local[r] = sum;
            }
            for (std::size_t k = 0; k < ns; ++k) a[i] += s.cols[k] * qdd_local[k];

            if (b.joint == BodyJointKind::Floating6) {
                for (int k = 0; k < 6; ++k) out.qdd[static_cast<std::size_t>(k)] = qdd_local[static_cast<std::size_t>(k)];
            } else {
                out.qdd[static_cast<std::size_t>(b.dof_index)] = qdd_local[0];
            }
        }
    }

    return out;
}

} // namespace ps::articulation
