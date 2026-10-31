// core/src/machine/coupling_solver.h — one substep of a socket coupling. Pure
// math: two rigid bodies' states in, one wrench pair out.
//
// The rows of a hitch are the socket separation e = (pa + ra) - (pb + rb), three
// of them, plus one per rotational axis the class constrains, each with position
// c and rate J v. With v_free each body's velocity after this substep's other
// forces, and lambda the row forces solved against the end-of-substep state, the
// rate at the end of the substep is J v_free + h A lambda, A = J M^-1 J^T.
//
// A spring and damper scaled by A, k = w^2 A^-1 and d = 2 z w A^-1, ring at
// frequency w whatever the masses are. Solved implicitly, with g = h w^2 + 2 z w:
//
//     A lambda = -(w^2 c + g J v_free) / (1 + h g)
//
// All rows at once (a 6x6 at most): a direct solve, so neither the mass ratio
// nor the coupling between the rows enters the convergence. Point and rotational
// rows solved in turn cost the same per pass and hold a locked implement 8.7x
// worse at one pass, 1.4x at eight.
//
// A rotational row's position is a Cardan angle of the towed hitch frame in the
// towing one, q = (yaw, pitch, roll) about e1 = towing z, e2 = towing z x towed x,
// e3 = towed x. omega_b - omega_a = sum q_dot_i e_i, so q_dot_i = d_i .
// (omega_b - omega_a) with d_i the dual basis of the e_i, and d_i is also the
// direction of the torque that does work on q_i alone.
//
// A limited axis has a row only while it is at its stop or would pass it within
// the substep, and the row only ever pushes back: one that would pull comes out
// and the system is solved again.
//
// A linkage (solve_linkage) has the same rows on other geometry. The two links are
// distance rows between the tractor's pivot and the implement's pin; a lateral row
// and the yaw and roll rows hold the implement in the tractor's centre plane; the
// sixth row is the lift, on the lower link's angle q in the towing chassis:
//
//     q_dot = t . (v_pin - v_pivot) / l - y . omega_a,    t = y x (pin - pivot) / l
//
// so lambda on it is the torque about the lower pivot, a force lambda / l across
// the link at the pin and its reaction at the pivot plus the couple on the tractor.
// The row lifts only, up to what the lift can give: past that it comes out of the
// matrix as a known force. Past a link stop it turns into a unilateral stop row,
// and in float that is all that is left of it.

#pragma once

#include "ps/machine/hitch_class.h"
#include "ps/machine/linkage.h"
#include "ps/math/mat3.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"

#include <array>

namespace ps::machine {

struct SocketBody {
    Vec3 linear{};      // chassis velocity after this substep's other forces
    Vec3 angular{};
    Vec3 socket_offset{}; // socket relative to the centre of mass, world axes
    Vec3 socket_position{};
    Quat hitch_frame = Quat::identity(); // world
    real inv_mass = 0.0;
    Mat3 inv_inertia_world = Mat3::zero();
};

// The rows' positions and rates between the two bodies; the rotational ones only for a class that has any.
struct HitchMeasure {
    Vec3 error{}, error_rate{};
    std::array<real, 3> angle{}, angle_rate{}; // yaw, pitch, roll
};

[[nodiscard]] HitchMeasure measure_hitch(const SocketBody& a, const SocketBody& b, const HitchClassDesc& hitch);

// What the rows are held to instead of zero while a latch ramps in.
struct LatchTarget {
    Vec3 error{}, error_rate{};
    std::array<real, 3> angle{}, angle_rate{}; // yaw, pitch, roll; locked axes only
};

struct SocketSolve {
    Vec3 force_on_a{}; // at a's centre of mass; b takes the opposite
    Vec3 torque_on_a{}, torque_on_b{}; // about each centre of mass, socket lever included
    Vec3 error{};
    std::array<real, 3> angle{}; // yaw, pitch, roll; zero when the class has no rotational row
    real work_j = 0.0;           // work on the relative motion over this substep
    int rows = 0;
};

[[nodiscard]] SocketSolve solve_socket_coupling(const SocketBody& a, const SocketBody& b, const HitchClassDesc& hitch, real h, const LatchTarget& latch = {});

// A chassis as the linkage rows see it.
struct LinkageBody {
    Vec3 position{}; // centre of mass
    Quat orientation = Quat::identity();
    Quat hitch_frame = Quat::identity(); // world
    Vec3 linear{}, angular{};
    real inv_mass = 0.0;
    Mat3 inv_inertia_world = Mat3::zero();
};

// The linkage rows' positions and rates. Row order: lower link, top link, lateral.
struct LinkageMeasure {
    Vec3 error{}, error_rate{};
    std::array<real, 3> angle{}, angle_rate{}; // yaw, pitch, roll of the towed frame
    real lift_angle = 0.0, lift_rate = 0.0;    // the lower link, from the hitch point's pose
};

[[nodiscard]] LinkageMeasure measure_linkage(const LinkageBody& a, const LinkageBody& b, const LinkageGeometry& g);

struct LiftCommand {
    bool active = true; // false: float, no lift row between the stops
    real target_rad = 0.0, target_rate = 0.0;
};

struct LinkageSolve {
    SocketSolve wrench; // its error holds the rows' errors in LinkageMeasure's order
    real lift_angle = 0.0;
    real lift_torque_nm = 0.0;    // the lift's about the lower pivot; one out of force on the lower stop: what it would take
    real lower_tension_n = 0.0;   // positive pulls the pin towards the pivot
    real top_compression_n = 0.0; // positive pushes the top pin away from the pivot
    bool lift_acting = false;     // the lift row carried force
    bool saturated = false;       // the lift ran out of force
};

[[nodiscard]] LinkageSolve solve_linkage(const LinkageBody& a, const LinkageBody& b, const LinkageGeometry& g, const HitchClassDesc& hitch, real h,
                                         const LiftCommand& lift, const LatchTarget& latch = {});

} // namespace ps::machine
