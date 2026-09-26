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

// PHYS-033 P4 I4 C8 (plan section 3.1): one differential's adapter-facing
// torque-split summary. bias = signed (T_left - T_right)/(|T_left| +
// |T_right|), 0 when both are 0 (an open diff at rest/zero-torque reads 0,
// not NaN). locked = a rigid lock is currently active (kind Locked, or a
// lockable diff with its topology-state bit set). lock_pending = C5's
// engagement-requested-but-mismatch-too-large flag. Debug/telemetry only -
// never hashed.
struct DiffSnapshot {
    real bias = 0.0;
    bool locked = false;
    bool lock_pending = false;
};

// Up to 4 differentials per vehicle (plan section 3.1's own cap - every
// shipped/test vehicle has at most 3, car_sedan_awd's centre+2 axle diffs).
// A fixed array, not a vector, so PowertrainSnapshot stays a flat, capacity-
// free-to-copy value type (D8: no allocation in the triple-buffer publish
// path this rides along with).
inline constexpr std::size_t kMaxPowertrainDiffs = 4;

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
    // PHYS-033 P4 I4 C8: per-differential torque-split summary, component-
    // array order (same index as World::create_vehicle's diff telemetry
    // channel registration). Filled by Powertrain::fill_diff_snapshot.
    std::array<DiffSnapshot, kMaxPowertrainDiffs> diffs{};
    std::uint8_t diff_count = 0;
    // PHYS-033 P4 I4 C9 (plan section 3.3): the MFWD transfer case's clutch
    // engagement (e in [0,1]), or 0 for every vehicle with no transfer case
    // (car_sedan.json and every other pre-C9 file).
    real awd_engagement = 0.0;
    // PHYS-033 P4 I4 C10 (plan section 3.4, owner ruling): the park pawl's
    // own engaged state (1 = holding, 0 = released or no park_lock at all).
    real park_lock_engaged = 0.0;
};

// Debug-only accessor payload for tests (World::powertrain_debug). Filled
// with defaults only until commit 6.
struct PowertrainDebug {
    real clutch_torque = 0.0, clutch_slip = 0.0, clutch_cap_kinetic = 0.0, clutch_cap_static = 0.0;
    bool clutch_stuck = false;
    real gearbox_mesh_torque = 0.0, diff_mesh_torque = 0.0;
    std::array<real, 8> hub_shaft_torque{};
    real throttle_eff = 0.0, idle_integrator = 0.0, synchro_torque = 0.0;
    // The engine's demanded net crank torque this substep (rev limiter
    // ignored - the torque-following clutch close's feed-forward) and the
    // TCU request it was computed with (upshift cut / pedal ceiling).
    real engine_torque_demand = 0.0;
    bool ignition_cut = false;
    real throttle_ceiling = 1.0;
    // PHYS-033 P4 I4 C8 (plan section 3.1): each differential's lock/LSD/
    // viscous transfer torque (Differential::transfer_torque, component-
    // array order) - see Powertrain::fill_diff_transfer_torque_debug and
    // powertrain.h's fill_hub_shaft_torque_debug, which now folds this in so
    // hub_shaft_torque[] is the REAL total shaft torque, not just the diff
    // row's own mesh-torque share.
    std::array<real, kMaxPowertrainDiffs> diff_transfer_torque{};
    // PHYS-033 P4 I4 C9: the MFWD transfer case's own `front` friction row
    // torque/slip (Powertrain::awd_clutch_torque/awd_clutch_slip), 0 for
    // every vehicle with no transfer case.
    real awd_clutch_torque = 0.0, awd_clutch_slip = 0.0;
    // PHYS-033 P4 I4 C10: the park pawl's own friction row torque, 0 for
    // every vehicle with no park_lock.
    real park_lock_torque = 0.0;
};

} // namespace ps::drivetrain
