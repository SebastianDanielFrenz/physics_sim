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

#pragma once

#include "ps/machine/hitch_class.h"
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

} // namespace ps::machine
