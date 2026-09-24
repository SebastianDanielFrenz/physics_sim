// ps/drivetrain/powertrain_state.h — public, additive per-vehicle powertrain
// snapshot/debug payloads (PHYS-033 P4 I3 plan section 4.3). Pure data; no
// algorithm code, no Jolt/Godot type (same D2/D10 pattern as
// ps/articulation/articulation_desc.h and ps/drivetrain/powertrain_desc.h).
//
// I3 commit 5 scope: this header only, filled with defaults everywhere it is
// referenced (VehicleInstance::powertrain_state_, World::powertrain_state/
// powertrain_debug) - commit 6 wires VehicleInstance's real engine/clutch/
// gearbox/TCU state into it.

#pragma once

#include "ps/types.h"

#include <array>
#include <cstdint>
#include <vector>

namespace ps::drivetrain {

enum class EngineState : std::uint8_t { Off, Stalled, Cranking, Running };
enum class ShiftPhase : std::uint8_t { Idle, ClutchOpening, Releasing, Selecting, Synchronising, ClutchClosing };

// spec section 5.5, I3 subset (additive later).
struct EngineSoundState {
    real crank_angle = 0.0; // rad, mod 4*pi. NOT hashed (like WheelInstance::spin_angle_)
    real omega = 0.0, load = 0.0; // load = effective combustion throttle theta_c (0..1)
    std::uint64_t cut_mask = 0; // all ones while the limiter cuts or DFCO is active
    EngineState engine_state = EngineState::Off;
    bool starter_active = false, limiter_active = false, dfco_active = false;
};

struct DrivelineSoundState {
    real gearbox_in_omega = 0.0, gearbox_out_omega = 0.0, gear_load_torque = 0.0;
    real clutch_slip = 0.0, clutch_torque = 0.0;
    std::int8_t gear = 0;
    bool grind_active = false;
    std::uint32_t grind_events = 0; // sound only, not hashed
};

// The adapter-facing per-vehicle powertrain snapshot (additive to
// VehicleSnapshot, ps/world/snapshot.h). Filled with defaults only until
// commit 6 wires VehicleInstance's real engine/clutch/gearbox/TCU state
// into it.
struct PowertrainSnapshot {
    std::int8_t gear = 0, gear_target = 0;
    ShiftPhase shift_phase = ShiftPhase::Idle;
    real engine_rpm = 0.0;
    EngineState engine_state = EngineState::Off;
    real speedo_mps = 0.0; // gearbox-output-derived, like a real speedo (spec section 7.6)
    real clutch_engagement = 0.0; // effective (min of pedal and auto)
    std::uint32_t assist_active_bits = 0; // bit0 auto_clutch, bit1 auto_blip, bit2 auto_shift
    std::vector<EngineSoundState> engines; // 0 or 1 in I3; capacity reused per snapshot slot (D8)
    DrivelineSoundState driveline;
};

// Debug-only accessor payload for tests (World::powertrain_debug). Filled
// with defaults only until commit 6.
struct PowertrainDebug {
    real clutch_torque = 0.0, clutch_slip = 0.0, clutch_cap_kinetic = 0.0, clutch_cap_static = 0.0;
    bool clutch_stuck = false;
    real gearbox_mesh_torque = 0.0, diff_mesh_torque = 0.0;
    std::array<real, 8> hub_shaft_torque{};
    real throttle_eff = 0.0, idle_integrator = 0.0, synchro_torque = 0.0;
};

} // namespace ps::drivetrain
