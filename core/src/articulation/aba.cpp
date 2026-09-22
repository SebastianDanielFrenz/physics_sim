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

AbaOutput aba(const AbaInput& in) {
    const Model& m = *in.model;
    const ArticulationState& st = *in.state;
    const std::size_t n = m.bodies.size();

    AbaOutput out;
    out.qdd.assign(static_cast<std::size_t>(m.num_v), real(0.0));

    std::vector<SpatialTransform> xup(n);
    std::vector<SpatialVector> v(n);
    std::vector<SpatialVector> c(n); // velocity-product bias term, v_i x (S_i*qd_i)
    std::vector<Mat6> IA(n);
    std::vector<SpatialVector> pA(n);
    std::vector<std::vector<SpatialVector>> U(n);
    std::vector<SmallMatrix> Dinv(n);
    std::vector<std::vector<real>> u(n);

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
            const auto s = motion_subspace(b);
            U[idx].resize(s.size());
            for (std::size_t a = 0; a < s.size(); ++a) U[idx][a] = IA[idx].apply(s[a]);

            SmallMatrix D = SmallMatrix::zero(static_cast<int>(s.size()), static_cast<int>(s.size()));
            for (std::size_t a = 0; a < s.size(); ++a)
                for (std::size_t bcol = 0; bcol < s.size(); ++bcol) D.at(static_cast<int>(a), static_cast<int>(bcol)) = spatial_dot(s[a], U[idx][bcol]);
            Dinv[idx] = D.inverse();

            u[idx].resize(s.size());
            for (std::size_t a = 0; a < s.size(); ++a) {
                const real tau_a = (b.joint == BodyJointKind::Floating6) ? in.tau[a] : in.tau[static_cast<std::size_t>(b.dof_index)];
                u[idx][a] = tau_a - spatial_dot(s[a], pA[idx]);
            }

            // Ia = IA_i - U*Dinv*U^T (rank-|s| reduction); pa = pA_i + Ia*c_i + U*Dinv*u.
            Mat6 udu;
            for (int r = 0; r < 6; ++r)
                for (int cc = 0; cc < 6; ++cc) {
                    real sum = 0.0;
                    for (std::size_t a = 0; a < s.size(); ++a)
                        for (std::size_t bcol = 0; bcol < s.size(); ++bcol) {
                            const real ur[6] = {U[idx][a].top.x, U[idx][a].top.y, U[idx][a].top.z, U[idx][a].bottom.x, U[idx][a].bottom.y, U[idx][a].bottom.z};
                            const real uc[6] = {U[idx][bcol].top.x, U[idx][bcol].top.y, U[idx][bcol].top.z, U[idx][bcol].bottom.x, U[idx][bcol].bottom.y, U[idx][bcol].bottom.z};
                            sum += ur[r] * Dinv[idx].at(static_cast<int>(a), static_cast<int>(bcol)) * uc[cc];
                        }
                    udu.m[r][cc] = sum;
                }
            const Mat6 Ia = IA[idx] - udu;

            std::vector<real> dinv_u(s.size(), real(0.0));
            for (std::size_t a = 0; a < s.size(); ++a) {
                real sum = 0.0;
                for (std::size_t bcol = 0; bcol < s.size(); ++bcol) sum += Dinv[idx].at(static_cast<int>(a), static_cast<int>(bcol)) * u[idx][bcol];
                dinv_u[a] = sum;
            }
            SpatialVector u_term{};
            for (std::size_t a = 0; a < s.size(); ++a) u_term += U[idx][a] * dinv_u[a];
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
    std::vector<SpatialVector> a(n);
    for (std::size_t i = 0; i < n; ++i) {
        const BodyModel& b = m.bodies[i];
        const SpatialVector a_parent = (b.parent < 0) ? a_world : a[static_cast<std::size_t>(b.parent)];
        a[i] = xup[i].apply_motion(a_parent) + c[i];

        if (b.ndof > 0) {
            const auto s = motion_subspace(b);
            std::vector<real> rhs(s.size());
            for (std::size_t k = 0; k < s.size(); ++k) rhs[k] = u[i][k] - spatial_dot(a[i], U[i][k]);
            std::vector<real> qdd_local(s.size(), real(0.0));
            for (std::size_t r = 0; r < s.size(); ++r) {
                real sum = 0.0;
                for (std::size_t cidx = 0; cidx < s.size(); ++cidx) sum += Dinv[i].at(static_cast<int>(r), static_cast<int>(cidx)) * rhs[cidx];
                qdd_local[r] = sum;
            }
            for (std::size_t k = 0; k < s.size(); ++k) a[i] += s[k] * qdd_local[k];

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
