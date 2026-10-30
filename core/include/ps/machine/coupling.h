// ps/machine/coupling.h — a hitch between two vehicles (attachment spike A0).
//
// Two sockets, one fixed in each chassis, held together by an implicit
// spring-damper evaluated once per substep inside the vehicles' shadow chassis
// (see core/src/machine/coupling_solver.h for the equations). Both vehicles of a
// coupling step together in one island (PLAN.md section 4), so the force reaches
// both shadows in the same substep and their wheels see it at once.
//
// Spike quality: no rotational limits, no drawbar length, no release at runtime.

#pragma once

#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/world/ids.h"

namespace ps::machine {

struct CouplingDesc {
    VehicleId towing;
    VehicleId towed;
    Vec3 socket_towing_local{}; // chassis-local, relative to the chassis centre of mass
    Vec3 socket_towed_local{};

    // Mass-normalised form, used when frequency_hz > 0: the hitch rings at this
    // frequency whatever the two masses are (k = omega^2 W^-1, W = the inverse
    // mass seen at the sockets), so one value suits a 1500 kg and a 40 t trailer.
    real frequency_hz = 0.0;
    real damping_ratio = 1.0;
    // Plain form, used when frequency_hz == 0: an isotropic spring and damper.
    real stiffness_n_m = 0.0;
    real damping_n_s_m = 0.0;
};

// Last tick's outcome (sim thread only, unhashed: the coupling has no state of
// its own, everything it reads is already in the hash).
struct CouplingState {
    Vec3 mean_force_on_towing_world{};
    Vec3 error_world{};     // socket separation in the shadows at the last substep
    real max_error_m = 0.0; // largest socket separation in the shadows during the tick
    real work_j = 0.0;      // cumulative work the coupling did on the relative motion (negative = removed energy)
};

} // namespace ps::machine
