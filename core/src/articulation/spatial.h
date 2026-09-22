// core/src/articulation/spatial.h — 6D spatial vector algebra (internal).
//
// Source: R. Featherstone, "Rigid Body Dynamics Algorithms", Springer 2008,
// ch. 2 (spatial vector algebra: spatial motion/force vectors, spatial
// cross products, spatial rigid-body inertia, Plucker/spatial transforms).
// Motion vectors stack [angular; linear] = [w; v] (top = angular, bottom =
// linear), force vectors stack [n; f] (top = moment, bottom = linear
// force) - Featherstone's own convention (ch. 2, "spatial vectors"). All
// derivations below are carried out explicitly in the comments rather than
// quoted from memory of exact equation numbers, so every formula can be
// checked independently of the source text; the equations match ch. 2's
// definitions of the spatial cross-product operators (v x, v x*, Table 2.2
// in the 2nd edition), rigid-body spatial inertia (ch. 2, "spatial
// inertia"), and Plucker coordinate transforms (ch. 2, "spatial transforms
// / coordinate transforms of motion and force vectors").
//
// Internal to core/src/articulation; not part of the public API
// (core/include/ps/articulation is descriptor-only, PLAN.md rule "No Jolt
// types in public headers" plus our own choice to keep spatial-algebra
// implementation details out of the public surface entirely).

#pragma once

#include "ps/math/mat3.h"
#include "ps/math/pose.h"
#include "ps/math/vec3.h"
#include "ps/types.h"

namespace ps::articulation::spatial {

// ---------------------------------------------------------------------
// SpatialVector: a single 6D motion or force vector. The same storage is
// used for both; which one a given value represents is tracked by the
// call site (as in Featherstone's own pseudocode), not by the type.
// ---------------------------------------------------------------------
struct SpatialVector {
    Vec3 top = Vec3::zero();    // angular (motion: omega) / moment (force: n)
    Vec3 bottom = Vec3::zero(); // linear (motion: v)       / force (force: f)

    constexpr SpatialVector() = default;
    constexpr SpatialVector(const Vec3& t, const Vec3& b) : top(t), bottom(b) {}

    constexpr SpatialVector operator+(const SpatialVector& o) const { return {top + o.top, bottom + o.bottom}; }
    constexpr SpatialVector operator-(const SpatialVector& o) const { return {top - o.top, bottom - o.bottom}; }
    constexpr SpatialVector operator-() const { return {-top, -bottom}; }
    constexpr SpatialVector operator*(real s) const { return {top * s, bottom * s}; }
    constexpr SpatialVector& operator+=(const SpatialVector& o) { top += o.top; bottom += o.bottom; return *this; }
};

inline constexpr SpatialVector operator*(real s, const SpatialVector& v) { return v * s; }

// Spatial dot product: for a motion vector m and a force vector f this is
// the instantaneous power m.f = w.n + v.f (ch. 2). Used by RNEA's energy
// bookkeeping cross-checks and by the loop-closure constraint solve.
inline constexpr real spatial_dot(const SpatialVector& a, const SpatialVector& b) {
    return dot(a.top, b.top) + dot(a.bottom, b.bottom);
}

// Spatial cross product for motion vectors, v x m (ch. 2, Table 2.2,
// "motion motion cross product"). Derivation: a motion vector [w;u]
// represents the velocity field of a rigid body about some reference
// point (v(point) = u + w x (point - ref)). Differentiating a body-fixed
// motion vector m=[wm;um] as it is carried by a frame moving with
// spatial velocity v=[w;u] gives the transport terms below - this is
// exactly the operator RNEA needs to propagate the velocity-product
// (Coriolis/centrifugal) acceleration term down the kinematic tree.
//   (v x m).top    = w x wm
//   (v x m).bottom = w x um + u x wm
inline SpatialVector cross_motion(const SpatialVector& v, const SpatialVector& m) {
    return SpatialVector{cross(v.top, m.top), cross(v.top, m.bottom) + cross(v.bottom, m.top)};
}

// Spatial cross product for force vectors, v x* f (ch. 2, Table 2.2,
// "motion force cross product" - the dual/negative-transpose of
// cross_motion, required so that power is frame-invariant:
// (v x m).f + m.(v x* f) == 0 for all m, f). Derivation of the dual: if
// cross_motion is represented as a 6x6 matrix C(v) acting on m, then
// v x* f = -C(v)^T f. Expanding C(v) = [[skew(w), 0], [skew(u), skew(w)]]
// (from cross_motion above) gives C(v)^T = [[-skew(w), -skew(u)],
// [0, -skew(w)]], so -C(v)^T = [[skew(w), skew(u)], [0, skew(w)]], i.e.:
//   (v x* f).top    = w x n + u x f
//   (v x* f).bottom = w x f
inline SpatialVector cross_force(const SpatialVector& v, const SpatialVector& f) {
    return SpatialVector{cross(v.top, f.top) + cross(v.bottom, f.bottom), cross(v.top, f.bottom)};
}

// ---------------------------------------------------------------------
// SpatialInertia: rigid-body spatial inertia about a reference point O,
// expressed in some frame's axes (ch. 2, "spatial rigid-body inertia").
// Stored as (mass, first moment h = m*c where c is the COM offset from O,
// rotational inertia Io about O) rather than the dense 6x6 form - cheaper
// to transform and exactly the parametrisation Featherstone's own
// articulated-body inertia recursion (ch. 7) updates in place.
//
// Derivation of the action on a motion vector [w;v] (v = linear velocity
// of the reference point O): the body's COM velocity is v + w x c, so
//   linear momentum p      = m*(v + w x c) = m*v - m*(c x w) = m*v - skew(h)*w
//   angular momentum (about O) L = Ic*w + m*c x (v + w x c)
// Expanding c x (w x c) = (|c|^2*I - c*c^T)*w (BAC-CAB) and using the
// parallel-axis theorem Io = Ic + m*(|c|^2*I - c*c^T) (inertia about O
// instead of about the COM):
//   L = Io*w + m*(c x v) = Io*w + skew(h)*v
// so, stacking [L;p] = I_spatial * [w;v]:
//   I_spatial = [ Io         skew(h) ]
//               [ -skew(h)   m*I3    ]
struct SpatialInertia {
    real mass = 0.0;
    Vec3 h = Vec3::zero();       // first moment about the reference point, m*c
    Mat3 io = Mat3::zero();      // rotational inertia about the reference point

    static SpatialInertia from_com(real mass, const Vec3& com, const Mat3& inertia_about_com) {
        // parallel-axis: Io = Ic + m*(|c|^2*I - c*c^T). skew(c)*skew(c) ==
        // c*c^T - |c|^2*I (standard identity), so c*c^T == skew(c)*skew(c) + |c|^2*I,
        // and therefore |c|^2*I - c*c^T == |c|^2*I - skew(c)*skew(c) - |c|^2*I
        // == -(skew(c)*skew(c)) - the |c|^2*I terms cancel exactly, leaving
        // just the negated skew-square (found by a failing ABA-vs-CRBA
        // cross-check test on multi-link trees with COM offsets: an earlier
        // version of this line kept a spurious extra +|c|^2*I term).
        const Mat3 shift = (skew(com) * skew(com)) * real(-1.0);
        return SpatialInertia{mass, com * mass, inertia_about_com + shift * mass};
    }

    SpatialVector operator*(const SpatialVector& v) const {
        const Vec3 hxb = cross(h, v.bottom);
        const Vec3 hxa = cross(h, v.top);
        return SpatialVector{io * v.top + hxb, v.bottom * mass - hxa};
    }

    SpatialInertia operator+(const SpatialInertia& o) const {
        return SpatialInertia{mass + o.mass, h + o.h, io + o.io};
    }
};

// ---------------------------------------------------------------------
// SpatialTransform: Plucker coordinate transform from a frame A to a
// frame B, given B's pose relative to A (rotation matrix rot such that a
// direction vector expressed in A becomes rot*vector when expressed in
// B's axes, and r = offset of B's origin from A's origin, expressed in
// A's axes) - ch. 2, "spatial transforms of motion and force vectors".
//
// Derivation (motion transform): a rigid body's velocity field obeys
// v(point) = v(ref) + w x (point - ref). The same physical motion vector
// [w;v_A] (velocity of the material point instantaneously at A's origin,
// expressed in A's axes) re-expressed at B's origin, in A's axes, is
// v_A + w x (B - A) = v_A + w x r. Rotating both components into B's
// axes: w_B = rot*w_A, v_B = rot*(v_A + w_A x r) = rot*v_A - rot*skew(r)*w_A.
// In block form:
//   X_motion = [ rot            0   ]
//              [ -rot*skew(r)   rot ]
//
// Force transforms are the dual: X_force = X_motion^-T, which works out
// (standard result, e.g. Featherstone ch. 2) to:
//   X_force  = [ rot   -rot*skew(r) ]
//              [ 0      rot         ]
// This is exactly the transform that keeps power invariant: for any
// motion m_A/force f_A pair, (X_motion*m_A).(X_force*f_A) == m_A.f_A.
struct SpatialTransform {
    Mat3 rot = Mat3::identity(); // rotation A -> B
    Vec3 r = Vec3::zero();       // B's origin, in A's axes

    static SpatialTransform from_pose(const Mat3& rot_a_to_b, const Vec3& origin_b_in_a) {
        return SpatialTransform{rot_a_to_b, origin_b_in_a};
    }

    [[nodiscard]] SpatialVector apply_motion(const SpatialVector& v) const {
        const Vec3 w_b = rot * v.top;
        const Vec3 v_b = rot * (v.bottom - cross(r, v.top));
        return SpatialVector{w_b, v_b};
    }

    [[nodiscard]] SpatialVector apply_force(const SpatialVector& f) const {
        const Vec3 n_b = rot * (f.top - cross(r, f.bottom));
        const Vec3 f_b = rot * f.bottom;
        return SpatialVector{n_b, f_b};
    }

    // Inverse transform (B -> A): A's origin expressed in B's axes is
    // rot*(0 - r) = -rot*r; the rotation inverts to rot^T.
    [[nodiscard]] SpatialTransform inverse() const {
        return SpatialTransform{rot.transposed(), (rot * r) * real(-1.0)};
    }

    [[nodiscard]] SpatialVector apply_motion_inverse(const SpatialVector& v_b) const {
        // w_A = rot^T * w_B ; v_A = rot^T*v_B + r x w_A
        const Vec3 w_a = rot.transposed() * v_b.top;
        const Vec3 v_a = rot.transposed() * v_b.bottom + cross(r, w_a);
        return SpatialVector{w_a, v_a};
    }

    [[nodiscard]] SpatialVector apply_force_inverse(const SpatialVector& f_b) const {
        const Vec3 f_a = rot.transposed() * f_b.bottom;
        const Vec3 n_a = rot.transposed() * f_b.top + cross(r, f_a);
        return SpatialVector{n_a, f_a};
    }

    // Congruence-transform a spatial inertia expressed in A's axes about
    // A's origin into B's axes about B's origin (I_B = X_force*I_A*X_motion^-1,
    // which - since X_force == X_motion^-T - is exactly the operation that
    // keeps I_B*apply_motion(v) == apply_force(I_A*v) for every motion v,
    // i.e. dynamically equivalent under the same coordinate change as
    // apply_motion/apply_force above). Implemented by transforming the
    // (mass, h, io) parametrisation directly rather than a dense 6x6
    // multiply (ch. 2 gives the closed form for a rigid transform of a
    // spatial inertia; re-derived here the same way apply_motion was):
    //   c_A = h/mass (COM in A axes, about A's origin)
    //   c_B = rot*(c_A - r)                      (COM re-expressed about B, in B axes)
    //   Io_B = rot*Io_A*rot^T - mass*skew(c_B... ) -- see full derivation in .cpp reference test;
    // implemented below by transforming Io about the shared origin first
    // (pure rotation, standard tensor rule Io' = R Io R^T) and then
    // re-applying the parallel-axis shift for the change of reference
    // point from A's origin to B's origin.
    [[nodiscard]] SpatialInertia apply_inertia(const SpatialInertia& ia) const {
        const Vec3 c_a = ia.mass > real(0.0) ? ia.h / ia.mass : Vec3::zero();
        // Io about A's own origin -> Io about the COM (remove parallel axis
        // at A): inverse of from_com's shift = -skew(c)*skew(c) above.
        const Mat3 io_about_com = ia.io + (skew(c_a) * skew(c_a)) * ia.mass;
        // Rotate the about-COM inertia tensor into B's axes.
        const Mat3 io_com_b_axes = rot * io_about_com * rot.transposed();
        // COM position relative to B's origin, expressed in B's axes.
        const Vec3 c_b = rot * (c_a - r);
        return SpatialInertia::from_com(ia.mass, c_b, io_com_b_axes);
    }
};

// Composes two Plucker transforms A->B and B->C into A->C. Derivation:
// rot_AC = rot_BC*rot_AB (successive basis changes). r_AC (C's origin in
// A's axes) = r_AB (B's origin in A axes) + the B->C offset r_BC
// re-expressed in A's axes, i.e. rot_AB^T * r_BC (rot_AB rotates A-axis
// vectors into B axes, so its transpose/inverse rotates B-axis vectors
// back into A axes).
inline SpatialTransform compose(const SpatialTransform& a_to_b, const SpatialTransform& b_to_c) {
    return SpatialTransform{b_to_c.rot * a_to_b.rot, a_to_b.r + a_to_b.rot.transposed() * b_to_c.r};
}

// Plucker transform "parent axes -> child axes" for a child rigidly posed
// at child_in_parent relative to the parent frame (Pose semantics match
// ps::Pose elsewhere: a child-local point p maps to
// child_in_parent.position + child_in_parent.orientation.rotate(p) in
// parent axes). The direction-vector rotation parent->child is therefore
// the INVERSE of child_in_parent.orientation (which itself maps
// child -> parent), and the child origin is already given in parent axes
// by child_in_parent.position directly.
inline SpatialTransform spatial_transform_from_pose(const Pose& child_in_parent) {
    return SpatialTransform{child_in_parent.orientation.inverse().to_mat3(), child_in_parent.position};
}

} // namespace ps::articulation::spatial
