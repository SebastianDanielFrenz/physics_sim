#include "aba.h"

#include <vector>

namespace ps::articulation {

using spatial::SpatialInertia;
using spatial::SpatialTransform;
using spatial::SpatialVector;
using spatial::cross_force;
using spatial::cross_motion;
using spatial::spatial_dot;

Mat6 Mat6::from_rigid(const SpatialInertia& i) {
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

SpatialVector Mat6::apply(const SpatialVector& v) const {
    const real x[6] = {v.top.x, v.top.y, v.top.z, v.bottom.x, v.bottom.y, v.bottom.z};
    real y[6] = {};
    for (int r = 0; r < 6; ++r) {
        real s = 0.0;
        for (int c = 0; c < 6; ++c) s += m[r][c] * x[c];
        y[r] = s;
    }
    return SpatialVector{Vec3{y[0], y[1], y[2]}, Vec3{y[3], y[4], y[5]}};
}

Mat6 Mat6::operator-(const Mat6& o) const {
    Mat6 r;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) r.m[i][j] = m[i][j] - o.m[i][j];
    return r;
}

Mat6 Mat6::operator+(const Mat6& o) const {
    Mat6 r;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) r.m[i][j] = m[i][j] + o.m[i][j];
    return r;
}

Mat6 Mat6::operator*(const Mat6& o) const {
    Mat6 r;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) {
            real s = 0.0;
            for (int k = 0; k < 6; ++k) s += m[i][k] * o.m[k][j];
            r.m[i][j] = s;
        }
    return r;
}

// Column c is X applied to the c-th canonical basis spatial vector - i.e.
// the dense matrix representation of the linear map `apply`.
Mat6 Mat6::from_linear_map(const SpatialTransform& x, bool force_map) {
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

Mat6Blocks blocks_of(const Mat6& m) {
    Mat6Blocks out;
    out.a = Mat3{
        Vec3{m.m[0][0], m.m[0][1], m.m[0][2]},
        Vec3{m.m[1][0], m.m[1][1], m.m[1][2]},
        Vec3{m.m[2][0], m.m[2][1], m.m[2][2]},
    };
    out.b = Mat3{
        Vec3{m.m[0][3], m.m[0][4], m.m[0][5]},
        Vec3{m.m[1][3], m.m[1][4], m.m[1][5]},
        Vec3{m.m[2][3], m.m[2][4], m.m[2][5]},
    };
    out.d = Mat3{
        Vec3{m.m[3][3], m.m[3][4], m.m[3][5]},
        Vec3{m.m[4][3], m.m[4][4], m.m[4][5]},
        Vec3{m.m[5][3], m.m[5][4], m.m[5][5]},
    };
    return out;
}

Mat6 mat6_from_blocks(const Mat6Blocks& blocks) {
    Mat6 out;
    const Mat3 bt = blocks.b.transposed();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = blocks.a(r, c);
            out.m[r][c + 3] = blocks.b(r, c);
            out.m[r + 3][c] = bt(r, c);
            out.m[r + 3][c + 3] = blocks.d(r, c);
        }
    }
    return out;
}

// Reference implementation (PHYS-009 Step 2 rule 3): transforms a
// child-frame Mat6 "inertia-like" operator up into the parent's frame,
// given Xup (parent -> child, motion sense): M_A = Xf(A<-B) * M_B * Xm(A->B),
// where Xf(A<-B) is the force map of Xup's inverse (child->parent) and
// Xm(A->B) is Xup's own motion map (parent->child) - see aba.h's file
// comment / model.h for the underlying Xup convention. Kept byte-identical
// to what this file shipped before PHYS-009 Step 2, as a test-only
// reference (tests/unit/test_aba_fast_transform.cpp) for
// transform_inertia_to_parent_fast below - never called from aba_into's
// own hot path any more (two dense 6x6 builds plus two dense 6x6*6x6
// multiplies, profiled at ~70% of a whole aba() call, PHYS-009's
// motivation for the fast version).
Mat6 transform_inertia_to_parent_reference(const SpatialTransform& xup, const Mat6& child) {
    const Mat6 f_child_to_parent = Mat6::from_linear_map(xup.inverse(), /*force_map=*/true);
    const Mat6 m_parent_to_child = Mat6::from_linear_map(xup, /*force_map=*/false);
    return f_child_to_parent * child * m_parent_to_child;
}

// Fast implementation (PHYS-009 Step 2): the SAME congruence, computed
// directly from Xup's own (E, r) block structure instead of two dense 6x6
// builds and multiplies.
//
// Derivation. Write Xup's motion map (parent->child) in block form, from
// SpatialTransform::apply_motion's own formula (w_b = E*w_a; v_b = E*v_a -
// E*rx*w_a, where E = xup.rot, r = xup.r, rx = skew(r)):
//   Xm = [ E      0 ]
//        [ -E*rx  E ]
// The dense reference above computes Xf(A<-B) - the force map of
// Xup.inverse() = {E^T, -E*r} (SpatialTransform::inverse) - which, expanded
// the same way and simplified with the standard rotation identity
// skew(E*v) = E*skew(v)*E^T, works out to exactly Xm^T:
//   Xf(A<-B) = [ E^T    rx*E^T ]  =  Xm^T
//              [ 0      E^T    ]
// (this is the general fact that a Plucker force map is always its motion
// map's inverse-transpose, Xforce = Xmotion^-T - so the "inverse-then-
// force-map" the reference takes is just a roundabout way to reach Xm^T).
// So the whole operation is the ordinary symmetric congruence transform
//   M_parent = Xm^T * M_child * Xm.
// Writing M_child in blocks [[A,B],[B^T,D]] (A, D symmetric - true for
// every IA this recursion ever produces) and multiplying out M_child*Xm
// then Xm^T*(...) in block form (each block product is a 3x3 Mat3
// operation, not a dense 6x6 one) gives, with Arot=E^T*A*E, Brot=E^T*B*E,
// Drot=E^T*D*E (three ordinary 3x3 congruences) and rx=skew(r):
//   A' = Arot - Brot*rx + rx*Brot^T - rx*Drot*rx
//   B' = Brot + rx*Drot
//   D' = Drot
// (D' symmetric since Drot is; A' symmetric by construction - verified
// independently by the fast-vs-reference cross-check test, not just by
// this derivation). About a third of the reference's multiply-add count:
// 3 congruences (2 Mat3*Mat3 each) plus 4 more Mat3*Mat3 products for the
// rx terms, vs. the reference's 24 SpatialTransform::apply_* calls (each
// several cross products) to build two dense Mat6 plus two dense 6x6*6x6
// multiplies (216 multiply-adds each).
Mat6 transform_inertia_to_parent_fast(const SpatialTransform& xup, const Mat6& child) {
    const Mat6Blocks m = blocks_of(child);
    const Mat3 Et = xup.rot.transposed();
    const Mat3 rx = skew(xup.r);

    const Mat3 Arot = Et * m.a * xup.rot;
    const Mat3 Brot = Et * m.b * xup.rot;
    const Mat3 Drot = Et * m.d * xup.rot;

    Mat6Blocks out;
    out.d = Drot;
    out.b = Brot + rx * Drot;
    out.a = Arot - Brot * rx + rx * Brot.transposed() - rx * Drot * rx;
    return mat6_from_blocks(out);
}

AbaOutput aba(const AbaInput& in, AbaScratch* scratch_in) {
    // See aba.h's AbaScratch comment: hot-path callers (Instance) always
    // pass their own preallocated scratch; other callers (unit-test cross-
    // checks, never run inside a job) fall back to a stack-local instance -
    // still one allocation for THIS call's fresh AbaOutput::qdd, acceptable
    // since these callers never run inside a job.
    AbaScratch local;
    AbaScratch& sc = scratch_in ? *scratch_in : local;
    AbaOutput out;
    aba_into(in, sc, out.qdd);
    return out;
}

void aba_into(const AbaInput& in, AbaScratch& sc, std::vector<real>& qdd_out) {
    aba_core(in, sc, qdd_out, /*use_fast_transform=*/true);
}

AbaOutput aba_reference(const AbaInput& in, AbaScratch* scratch_in) {
    // PHYS-009 Step 2 rule 3: identical to aba() above, except pass 2 uses
    // transform_inertia_to_parent_reference instead of _fast - a test-only
    // entry point (tests/unit/test_aba_fast_transform.cpp) for cross-
    // checking a full ABA run, not just the isolated transform function,
    // fast vs. reference on random trees.
    AbaScratch local;
    AbaScratch& sc = scratch_in ? *scratch_in : local;
    AbaOutput out;
    aba_core(in, sc, out.qdd, /*use_fast_transform=*/false);
    return out;
}

void aba_core(const AbaInput& in, AbaScratch& sc, std::vector<real>& qdd_out, bool use_fast_transform) {
    Mat6 (*const transform)(const SpatialTransform&, const Mat6&) =
        use_fast_transform ? transform_inertia_to_parent_fast : transform_inertia_to_parent_reference;

    const Model& m = *in.model;
    const ArticulationState& st = *in.state;
    const std::size_t n = m.bodies.size();

    // Idempotent at a stable num_v (assign() to an already-current size
    // neither allocates nor deallocates - PHYS-009 Step 1 rule 2, same
    // reasoning as AbaScratch::ensure below).
    qdd_out.assign(static_cast<std::size_t>(m.num_v), real(0.0));
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
                IA[static_cast<std::size_t>(b.parent)] = IA[static_cast<std::size_t>(b.parent)] + transform(xup[idx], Ia);
                pA[static_cast<std::size_t>(b.parent)] += xup[idx].apply_force_inverse(pa);
            }
        } else {
            // Fixed (0-DoF) joint: nothing to reduce, fold straight up.
            if (b.parent >= 0) {
                IA[static_cast<std::size_t>(b.parent)] = IA[static_cast<std::size_t>(b.parent)] + transform(xup[idx], IA[idx]);
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
                for (int k = 0; k < 6; ++k) qdd_out[static_cast<std::size_t>(k)] = qdd_local[static_cast<std::size_t>(k)];
            } else {
                qdd_out[static_cast<std::size_t>(b.dof_index)] = qdd_local[0];
            }
        }
    }
}

} // namespace ps::articulation
