// ps/drivetrain/b_mode.h - the "B" one-pedal regeneration mode of electric drives (docs/electric_drive.md "B mode").
//
// B is a selector position above D on a car whose traction motors sit on the wheel path (a pure EV, or a hybrid driven in its
// Electric mode). The pedal commands the SYSTEM power linearly between the most the car can regenerate right now and the most
// it can drive with right now:
//
//   P(pedal) = -R + pedal * (R + P_max)        R  = regeneration capacity [W, shaft side, >= 0]
//                                              P_max = peak drive power now [W, shaft side, >= 0]
//
// Pedal 0 is full regeneration at every speed (down to the low-speed fade), pedal 1 is the car's full drive power, and the
// zero-power (coasting) point sits at pedal R / (R + P_max). These functions are pure and public so a dashboard or a garage
// chart can draw the same line the car follows. The runtime (core/src/drivetrain/powertrain.cpp) calls exactly these.
#pragma once

#include "ps/types.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {

// Configuration (vehicle file: brake_system.regen_blend.b_mode, docs/data_formats/vehicle.md). All numbers [AS].
struct BModeDesc {
    bool enabled = true; // false = the car has no B position
    // Sensor-only wheel-slip guard (the TC regulator's structure, in braking direction): the motor's B regeneration is scaled
    // down while one of the wheels it drives slips more than slip_target (peripheral speed below the ground speed), and ramps
    // back when grip returns. Slip = (ground - wheel) / max(slip_floor_mps, ground).
    real slip_target = 0.08;
    real slip_floor_mps = 2.0;
    real kp = 2.0, ki_per_s = 6.0, attack_rate_per_s = 20.0, recovery_rate_per_s = 8.0;
    // Auto hold: with B selected, the pedal at or below the zero-power point and the car slower than the regeneration fade
    // (the motors cannot hold a stop), the pads hold the car until the pedal rises above the zero-power point by
    // hold_pedal_margin; the pads then release over hold_release_s.
    bool auto_hold = true;
    real hold_release_s = 0.3;
    real hold_rise_per_s = 3.0; // the pads' hold level rises at most this fast (a stop, not a lock-up)
    real hold_pedal_margin = 0.03;
};

// System power command for pedal in [0, 1] [W]: negative = regeneration.
[[nodiscard]] inline real b_system_power(real regen_capacity_w, real drive_capacity_w, real pedal) {
    const real p = std::clamp(pedal, real(0), real(1));
    return -regen_capacity_w + p * (regen_capacity_w + drive_capacity_w);
}

// The pedal at which the system power is zero; 0 with no regeneration capacity, 1 with no drive capacity.
[[nodiscard]] inline real b_zero_power_pedal(real regen_capacity_w, real drive_capacity_w) {
    const real sum = regen_capacity_w + drive_capacity_w;
    return sum > real(0) ? regen_capacity_w / sum : real(0);
}

// One substep of the slip regulator: returns the new regeneration fraction in [0, 1] (1 = full B regeneration).
[[nodiscard]] inline real b_slip_fraction_step(const BModeDesc& d, real fraction, real slip, real dt) {
    fraction = std::clamp(fraction, real(0), real(1));
    const real error = slip - d.slip_target;
    if (error > 0) {
        const real desired = std::max(real(0), std::min(fraction - d.ki_per_s * error * dt, real(1) - d.kp * error));
        return std::max(desired, fraction - d.attack_rate_per_s * dt);
    }
    const real recovery = std::clamp(-error / d.slip_target, real(.1), real(1));
    return std::min(real(1), fraction + d.recovery_rate_per_s * recovery * dt);
}

} // namespace ps::drivetrain
