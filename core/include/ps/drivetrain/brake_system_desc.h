// ps/drivetrain/brake_system_desc.h — public descriptor for direct-torque
// brakes (PHYS-033 P4 I2 plan section 2, F1/Q1 friction rows). Pure data;
// see powertrain_desc.h's header comment for the general D2/D10 pattern.

#pragma once

#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::drivetrain {

struct BrakeCornerDesc {
    std::string wheel; // WheelDesc::name, must exist and be unique across corners
    real max_torque_nm = 0.0;
};

struct HandbrakeDesc {
    std::vector<std::string> wheels; // WheelDesc::name list
    real max_torque_nm = 0.0;
};

enum class BrakeSystemType { DirectTorque }; // I5 adds Hydraulic

// Brake blending (regeneration first), docs/electric_drive.md "Brake blending". The pedal asks for the total service
// brake torque (pedal x corner capacities, as always); the electric motors take it first, the pads only the rest.
// Used only on a vehicle with electric_motor components (no effect otherwise). All numbers [AS].
struct RegenBlendDesc {
    bool enabled = true;           // false = the pre-blending behaviour (regen torque ~ pedal, in parallel with the pads)
    // Charge acceptance of the pack: full below taper_start_soc, falling linearly to zero at full_soc (a pack at or
    // above full_soc takes no regeneration; the pads carry the whole request).
    real taper_start_soc = 0.90;
    real full_soc = 0.98;
    // Low-speed fade of the regeneration, on the mean rim speed of the wheels the motor drives: full above
    // fade_start_speed_mps, zero at and below fade_end_speed_mps (the pads hold the car to a stop).
    real fade_start_speed_mps = 2.5;
    real fade_end_speed_mps = 0.8;
    // Sensor-only ABS handshake: regeneration on a motor is withdrawn while ABS acts on any wheel it drives and for
    // this long after the last intervention (so it does not chatter back into the lock).
    real abs_hold_s = 0.3;
};

struct BrakeSystemDesc {
    BrakeSystemType type = BrakeSystemType::DirectTorque;
    std::vector<BrakeCornerDesc> corners;
    HandbrakeDesc handbrake;
    // direct_torque: 1.0 reproduces P3 (no static/kinetic distinction) -
    // cap_static = static_factor * cap_kinetic (>= 1).
    real static_factor = 1.0;
    RegenBlendDesc regen_blend;
};

} // namespace ps::drivetrain
