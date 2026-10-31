// ps/machine/coupling.h — a hitch between two vehicles.
//
// Two sockets, one fixed in each chassis, held together by an implicit
// spring-damper evaluated once per substep inside the vehicles' shadow chassis
// (see core/src/machine/coupling_solver.h for the equations), with the rotational
// rows the hitch class asks for. Both vehicles of a coupling step together in one
// island (PLAN.md section 4), so the force reaches both shadows in the same
// substep and their wheels see it at once.

#pragma once

#include "ps/machine/hitch_class.h"
#include "ps/math/quat.h"
#include "ps/math/vec3.h"
#include "ps/types.h"
#include "ps/world/ids.h"

#include <array>
#include <string>

namespace ps::machine {

struct CouplingDesc {
    VehicleId towing;
    VehicleId towed;
    Vec3 socket_towing_local{}; // chassis-local, relative to the chassis centre of mass
    Vec3 socket_towed_local{};
    // The hitch frames in the chassis: the angles the class limits are those of the towed frame in the towing frame.
    Quat frame_towing_local = Quat::identity();
    Quat frame_towed_local = Quat::identity();
    HitchClassDesc hitch;
    // The vehicles' hitch points this coupling sits on (World::attach by name fills them); empty for one built by hand.
    std::string towing_point, towed_point;
};

// What the hitch would see if it latched now.
struct AttachCheck {
    bool ok = false;
    real error_m = 0.0;   // distance between the sockets
    real speed_m_s = 0.0; // their relative speed
    std::array<real, 3> angle_rad{}; // yaw, pitch, roll of the towed frame; zero for a class with every axis free
    std::string message;  // why not, with the measured and allowed values; empty when ok
};

// Last tick's outcome (sim thread only, unhashed: everything in it is derived).
struct CouplingState {
    Vec3 mean_force_on_towing_world{};
    Vec3 error_world{};     // socket separation in the shadows at the last substep
    real max_error_m = 0.0; // largest socket separation in the shadows during the tick
    real yaw_rad = 0.0, pitch_rad = 0.0, roll_rad = 0.0; // at the last substep; 0 for a class with every axis free
    real work_j = 0.0;      // cumulative work the coupling did on the relative motion (negative = removed energy)
    real latch_progress = 1.0; // 0 at attach, 1 once the rows hold the sockets themselves
    real tow_brake = 0.0;      // the towed vehicle's brake demand through this hitch, 0..1
};

} // namespace ps::machine
