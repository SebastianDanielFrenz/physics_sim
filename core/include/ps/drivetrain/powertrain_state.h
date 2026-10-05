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

#include "ps/drivetrain/engine_thermal_desc.h" // EngineThermalSnapshot
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
    real omega = 0.0, load = 0.0; // load = post-limiter combustion fraction theta_c (0..1)
    std::uint64_t cut_mask = 0; // all ones while the limiter cuts or DFCO is active
    EngineState engine_state = EngineState::Off;
    bool starter_active = false, limiter_active = false, dfco_active = false;
    // PHYS-033 P4 I7a C3: the throttle-command stage's combined command this
    // tick (pedal, through an optional pedal_map/linkage_curve, maxed with an
    // optional hand lever) - sound only, NOT hashed, like crank_angle above.
    real throttle_command = 0.0;
    // PHYS-033 P4 I7a C5, broadened C5d: this substep's fuel flow (g/s), read
    // from PreSolveResult::fuel_flow_g_s - nonzero for a fuel map OR the
    // crude no-map estimate, 0 only for an engine with no fuel model at all.
    // NOT hashed.
    real fuel_flow_g_s = 0.0;
    // PHYS-033 P4 I7b C12: mean-value engine state for sound (from
    // PreSolveResult; 0 for the graph engine). NOT hashed. Exhaust
    // temperatures exist for turbo engines only (fields below).
    real throttle_position = 0.0;      // plate position in [0, 1]
    real turbo_rpm = 0.0, boost_bar = 0.0, p_exhaust_pa = 0.0;
    real p_charge_pa = 0.0, t_charge_k = 0.0; // compressor supply, 0 = atmospheric path
    // PHYS-033 P4 I7d D1: further turbo state (docs/data_formats/turbo.md),
    // 0 without a turbo. NOT hashed.
    real wastegate_opening = 0.0;          // wastegate actuator position, 1 = fully open
    real compressor_flow_kg_s = 0.0;       // total compressor mass flow (the pair)
    real compressor_pressure_ratio = 0.0;  // compressor outlet over ambient, >= 1 with a turbo
    // PHYS-033 P4 I7d D2: blow-off valve, 0 without a valve. NOT hashed.
    real bov_opening = 0.0;                // valve position in [0, 1]
    real bov_flow_kg_s = 0.0;              // vent flow (part of compressor_flow_kg_s)
    std::uint8_t bov_vent = 0;             // 0 atmosphere, 1 recirculate
    std::uint32_t bov_events = 0;          // opening events (rising edge > 0.1, re-armed below 0.05)
    // PHYS-033 P4 I7d D3: compressor surge flag, 0 without a surge line on the turbo hardware file. NOT hashed.
    std::uint8_t surge_active = 0;         // 1 while the operating point is left of the surge line
    std::uint32_t surge_events = 0;        // entries into surge (hysteresis: leave above 1.05 x the line)
    // PHYS-033 P4 I7d D5: anti-lag, 0 without an anti_lag block in the turbo configuration. NOT hashed.
    std::uint8_t anti_lag_active = 0;      // 1 while anti-lag acts (DFCO inhibited, plate floor, spark retard)
    std::uint32_t anti_lag_activations = 0; // activations (rising edges of anti_lag_active)
    real anti_lag_extra_exhaust_w = 0.0;   // indicated work rate the retard removes (W); it stays in the exhaust gas
    // Exhaust temperatures (exhaust_thermal, docs/data_formats/turbo.md), 0 without a turbo. NOT hashed.
    real exhaust_port_k = 0.0;             // gas temperature at the exhaust ports
    real turbine_inlet_k = 0.0;            // gas temperature at the turbine inlet (after the exhaust wall)
    real exhaust_wall_k = 0.0;             // lumped exhaust manifold / turbine housing wall temperature
    real anti_lag_manifold_burn_w = 0.0;   // anti-lag cylinder cut: fuel energy released in the exhaust manifold (W)
    real p_intake_pa = 0.0;           // intake manifold pressure
    real lambda = 0.0;                 // commanded lambda
    real spark_advance_deg = 0.0;      // applied spark advance
    real exhaust_mass_flow_kg_s = 0.0; // air + fuel mass flow out of the cylinders
    // N2O plan N1 (section 6): the nitrous system's state, all 0 without a kit. NOT hashed, never read back
    // for physics (the engine ignores the N2O in N1).
    std::uint8_t n2o_active = 0;           // 1 while the solenoid is open and N2O flows
    std::uint8_t n2o_safety_cut = 0;       // NitrousSafetyCut: 0 none, 1 bottle pressure, 2 fuel starve, 3 duty time
    real n2o_flow_g_s = 0.0;               // delivered N2O flow
    real n2o_purge_flow_g_s = 0.0;         // purge vent flow
    real n2o_bottle_kg = 0.0;              // bottle content
    real n2o_bottle_bar = 0.0;             // bottle gauge pressure (bar)
    real n2o_retard_deg = 0.0;             // spark retard the kit asks for
    std::uint32_t n2o_activations = 0;     // solenoid openings since the vehicle was built
    std::uint32_t n2o_purge_events = 0;    // purge starts since the vehicle was built
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
// PHYS-033 P4 I4 C13: at most this many compliant shafts per vehicle (the
// Powertrain constructor rejects more) - PowertrainDebug's shaft_* arrays.
inline constexpr std::size_t kMaxCompliantShafts = 8;

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
    // PHYS-033 P4 I4 C11 (plan section 3.5): the pto's own engagement (e in
    // [0,1]), current mode as a real (540.0 or 1000.0), and out-node speed
    // in rpm - all 0 for every vehicle with no pto (car_sedan.json,
    // tractor.json pre-C11).
    real pto_engagement = 0.0;
    real pto_mode = 0.0;
    real pto_rpm = 0.0;
    // PHYS-033 P4 I4 C12 (governor_setpoint_rpm); PHYS-033 P4 I7a C3
    // (hand_throttle reworked to read the throttle-command stage's own raw
    // hand lever, ThrottleStage::raw_hand_lever via
    // Powertrain::governor_hand_throttle - not literally the governor's
    // channel any more, despite the name): governor_setpoint_rpm is the
    // all-speed governor's own no-load setpoint n_nl this tick, 0 for every
    // vehicle with no governor (car_sedan.json, every pre-C12 file).
    // hand_throttle is 0 when the throttle stage has no hand lever (no
    // explicit hand_lever_channel and no governor to fall back to) -
    // nonzero for a governed engine (the governor's own channel, whether or
    // not an explicit throttle_control stage is also present) OR an
    // ungoverned engine whose explicit stage declares its own
    // hand_lever_channel. Never hashed (plain reads of
    // Powertrain::governor_setpoint_rpm/governor_hand_throttle).
    real governor_setpoint_rpm = 0.0;
    real hand_throttle = 0.0;
    // PHYS-033 P4 I7a C5, broadened C5d: this tick's fuel flow (g/s), mirroring
    // EngineSoundState::fuel_flow_g_s above (same value - the adapter-facing
    // copy) - 0 only for an engine with no fuel model at all. Never hashed.
    real fuel_flow_g_s = 0.0;
    // PHYS-033 P4 I7a C6: the fuel tank's content (kg) at the end of this
    // tick - 0 for a vehicle without a fuel_tank. A read-only copy (the
    // hashed state lives in the Powertrain's tank).
    real fuel_mass_kg = 0.0;
    // N2O plan N1: the bottle content (kg) at the end of this tick - 0 without a kit.
    real n2o_bottle_kg = 0.0;
    // Engine thermal plan T1: coolant/oil temperature, gauge, warnings, fan. has_thermal false (all zeros)
    // for a vehicle without a cooling system. Never hashed.
    EngineThermalSnapshot thermal;
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
    // PHYS-033 P4 I4 C11: the pto's own friction row torque/slip, 0 for
    // every vehicle with no pto. dyno_torque is the first load_dyno
    // component's own applied torque (test/bench only, 0 with none).
    real pto_clutch_torque = 0.0, pto_clutch_slip = 0.0;
    real dyno_torque = 0.0;
    // PHYS-033 P4 I4 C13: each COMPLIANT shaft's twist theta (rad) and last
    // substep's shaft torque tau_s (N*m, acting -tau_s on ports[0]'s node and
    // +tau_s on ports[1]'s), slot i = the i-th compliant shaft in shafts[]
    // order (rigid shafts are skipped); all 0 for a Rigid vehicle. Tests
    // only, never hashed (theta itself is hashed by Powertrain).
    std::array<real, kMaxCompliantShafts> shaft_twist_rad{};
    std::array<real, kMaxCompliantShafts> shaft_torque{};
};

} // namespace ps::drivetrain
