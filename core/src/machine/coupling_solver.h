// core/src/machine/coupling_solver.h — one substep of a socket coupling
// (attachment spike A0). Pure math: two rigid bodies' states in, one force out.
//
// e = (pa + ra) - (pb + rb) is the socket separation, edot_free its rate once
// each body has taken this substep's other forces. The force F on a (and -F on
// b, at the sockets) is solved implicitly against the end-of-substep state:
//
//     edot_new = edot_free + h W F
//     F = -k (e + h edot_new) - c edot_new
//
// W = Wa + Wb, Wi = I/mi + [ri]x Ii^-1 [ri]x^T the inverse mass at a socket. The
// 3x3 is solved directly, so the mass ratio does not enter the convergence: it
// is exact for one coupling. Plain form: (I + h(hk + c) W) F = -(k e + (hk + c)
// edot_free). Mass-normalised form (k = w^2 W^-1, c = 2 z w W^-1), with
// g = h w^2 + 2 z w: F = -W^-1 (w^2 e + g edot_free) / (1 + h g).

#pragma once

#include "ps/machine/coupling.h"
#include "ps/math/mat3.h"

namespace ps::machine {

struct SocketBody {
    Vec3 linear{};      // chassis velocity after this substep's other forces
    Vec3 angular{};
    Vec3 socket_offset{}; // socket relative to the centre of mass, world axes
    Vec3 socket_position{};
    real inv_mass = 0.0;
    Mat3 inv_inertia_world = Mat3::zero();
};

struct SocketSolve {
    Vec3 force_on_a{};
    Vec3 error{};
    real work_j = 0.0; // work on the relative motion over this substep
};

[[nodiscard]] SocketSolve solve_socket_coupling(const SocketBody& a, const SocketBody& b, const CouplingDesc& desc, real h);

} // namespace ps::machine
