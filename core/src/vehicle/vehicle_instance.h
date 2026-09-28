// core/src/vehicle/vehicle_instance.h — one vehicle: chassis extrapolation
// across a tick's substeps (PLAN.md D3), Ackermann steering, brake/drive
// torque split, one WheelInstance per corner. Internal (owned by
// system.h).

#pragma once

#include "../drivetrain/powertrain.h"
#include "wheel_instance.h"

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/ambient.h"
#include "ps/world/ids.h"

#include <cstdint>
#include <string_view>
#include <vector>

namespace ps::vehicle {

// PLAN.md P2/M1 step 3's named control channels, read by World and handed
// down here (out/handover_p3.md decision 5: global, not per-vehicle, for
// P3/M1's one-car scope).
struct ControlInputs {
    real steer = 0.0;     // -1..1
    real throttle = 0.0;  // 0..1
    real brake = 0.0;     // 0..1
    real handbrake = 0.0; // 0..1
};

// PLAN.md P4 I0 (PHYS-033 section 8, Q4): this vehicle's four control
// channels, resolved to indices into World's flat control_channel_values_
// ONCE at create_vehicle (ps::World::create_vehicle, world.cpp) - plain
// indices, so step_tick below never hashes a string or touches World's
// std::unordered_map<std::string, ...> on the hot path.
struct ControlChannelIndices {
    std::size_t steer = 0, throttle = 0, brake = 0, handbrake = 0;
    // PHYS-033 P4 I3 plan section 7.1: additive channels for the shaft-
    // network powertrain's clutch/gearshift/ignition/assist controls. Not
    // yet read by step_tick anywhere (commit 6 consumes them) - resolved to
    // real indices in World::create_vehicle exactly like the four fields
    // above; these zero defaults are only this struct's own pre-resolution
    // placeholder.
    std::size_t clutch = 0;
    std::size_t shift_up_count = 0, shift_down_count = 0;
    std::size_t gear_request = 0;
    std::size_t ignition = 0, starter = 0;
    std::size_t assist_auto_clutch = 0, assist_auto_blip = 0, assist_auto_shift = 0;
    // PHYS-033 P4 I4 C3 (plan section 2.4): one resolved World control-
    // channel index per drivetrain::Powertrain::declare_control_channels(...)
    // entry, same order - resolved in World::create_vehicle exactly like the
    // fields above. Empty for every shipped vehicle today (no component
    // declares a channel yet).
    std::vector<std::size_t> component;
};

// PHYS-033 I1 W1: environment inputs step_tick's shadow chassis needs but
// that VehicleInstance itself has no other way to see (it only ever talks
// to IRigidBackend, never World) - just World::config_.gravity today.
struct VehicleTickEnv {
    Vec3 gravity = Vec3{0.0, 0.0, -9.81};
    // PHYS-033 P4 I3 plan section 4.5: kg/m^3, mirrors WorldConfig::
    // air_density's default. Unused until commit 6c wires aero drag into
    // step_tick.
    real air_density = 1.225;
    // PHYS-033 P4 I7b C9: ambient atmospheric sample (ps/world/ambient.h),
    // mirrors World::ambient() - see that header's comment. Plumbed through
    // to drivetrain::ComponentSubstepCtx::ambient (below) for a future
    // simulated engine; the graph engine ignores it.
    AmbientSample ambient{};
};

class VehicleInstance {
public:
    VehicleInstance(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                     const ControlChannelIndices& control_indices);

    // Runs a per-substep SHADOW chassis (PHYS-033 I1 W1, PLAN.md D3): a
    // local pose/twist integrated substep-by-substep from this tick's
    // start state, using exactly the inputs and update rule
    // IRigidBackend::get_integration_params + apply_wrench will reproduce
    // in the backend's own next step() (see vehicle_instance.cpp's own
    // comment for the exact conditions this depends on). Applies the
    // substep-averaged wrench once via backend.apply_wrench (unchanged from
    // before W1). `channel_values` is World's flat control_channel_values_
    // array, indexed by control_indices_ below.
    void step_tick(IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                    real tick_dt, real substep_rate_hz);

    // World::reset_vehicle (plan section 4.5). Call only between ticks,
    // never from a job (same contract as World::reset_vehicle itself).
    // Reads the chassis's CURRENT pose/motion from `backend` (for
    // opts.hubs_roll_with_chassis's per-wheel hub speed - chassis-local +x
    // forward speed / wheel_radius, same wheels for every corner regardless
    // of steer angle, a reasonable approximation for a teleport), clears
    // every wheel's own tyre relaxation state, then hands off to
    // Powertrain::reset for the network/gearbox/engine/TCU side.
    void reset(const vehicle::VehicleResetOptions& opts, const IRigidBackend& backend);

    [[nodiscard]] std::size_t wheel_count() const { return wheels_.size(); }
    [[nodiscard]] const WheelTelemetry& wheel_telemetry(std::size_t i) const { return wheels_[i]->telemetry(); }
    [[nodiscard]] const std::string& wheel_name(std::size_t i) const { return wheels_[i]->desc().name; }
    // PLAN.md M1, task 2 of the P3 fix-up phase: see wheel_state.h.
    [[nodiscard]] WheelState wheel_state(std::size_t i) const { return wheels_[i]->wheel_state(); }
    // PHYS-033 P4 I2 commit 5a: which wheels the /2 powertrain graph
    // actually drives (WheelDesc::driven is gone - see vehicle_desc.h's
    // VehicleDesc comment). Backs World::wheel_is_driven, the adapter's
    // replacement for the old WheelDesc::driven read (ps_simulation.cpp's
    // get_wheel_driven). Commit 5b: now answered by the real
    // drivetrain::Powertrain (a hub is driven iff its shaft port was
    // connected at all - powertrain.h's own comment) instead of the P3 shim.
    [[nodiscard]] bool wheel_is_driven(std::size_t i) const { return powertrain_.wheel_is_driven(i); }
    // PHYS-033 P4 I2 commit 5b (plan section 2): the drivetrain hub's own
    // angular velocity (network state - WheelInstance no longer has one)
    // and the cumulative/instantaneous mechanical energy ledger. Sim-
    // thread-only live accessors, same contract as wheel_telemetry above.
    [[nodiscard]] real wheel_omega(std::size_t i) const { return powertrain_.hub_omega(i); }
    [[nodiscard]] const drivetrain::PowertrainEnergy& powertrain_energy() const { return powertrain_.ledger(); }
    // PHYS-033 P4 I7a C6: backs World::fuel_tank_mass/set_fuel_tank_mass
    // (the set is applied at the next tick boundary - Powertrain::begin_tick).
    [[nodiscard]] bool has_fuel_tank() const { return powertrain_.has_fuel_tank(); }
    [[nodiscard]] real fuel_tank_mass() const { return powertrain_.fuel_tank_mass(); }
    void request_fuel_tank_mass(real kg) { powertrain_.request_fuel_tank_mass(kg); }
    // PHYS-033 P4 I7c C16b: backs World::replace_engine_maps (queues) and
    // vehicle::System::step_all's sequential pre-pass (applies, outside any
    // job - see Powertrain::apply_pending_engine_maps_swap's own comment).
    void request_replace_engine_maps(std::string_view component_id, drivetrain::EngineMapsPtr new_maps) {
        powertrain_.request_replace_engine_maps(component_id, std::move(new_maps));
    }
    void apply_pending_engine_maps_swap() { powertrain_.apply_pending_engine_maps_swap(); }
    [[nodiscard]] std::uint64_t simulated_engine_maps_content_hash() const {
        return powertrain_.simulated_engine_maps_content_hash();
    }
    [[nodiscard]] int powertrain_friction_cap_hits() const { return powertrain_.friction_cap_hits(); }
    // PHYS-033 P4 I4 C13: whether telemetry "stored" adds shaft_pe (only then).
    [[nodiscard]] bool powertrain_has_compliant_shafts() const { return powertrain_.has_compliant_shafts(); }
    [[nodiscard]] int powertrain_nonspd_hits() const { return powertrain_.nonspd_hits(); }
    [[nodiscard]] int powertrain_lcp_stage3_hits() const { return powertrain_.lcp_stage3_hits(); }
    // PHYS-033 P4 I3 plan section 4.3/4.5: the adapter-facing snapshot and a
    // by-value debug payload. Commit 6 wired powertrain_state_; commit 7
    // wires this debug accessor (powertrain_state_ itself stays a snapshot
    // filled once per tick by step_tick, this reads the Powertrain's live
    // per-substep-cached state directly - both are sim-thread-only, same
    // contract as wheel_telemetry). Backs World::powertrain_state/
    // powertrain_debug.
    [[nodiscard]] const drivetrain::PowertrainSnapshot& powertrain_state() const { return powertrain_state_; }
    [[nodiscard]] drivetrain::PowertrainDebug powertrain_debug() const {
        drivetrain::PowertrainDebug dbg;
        dbg.clutch_torque = powertrain_.clutch_torque();
        dbg.clutch_slip = powertrain_.clutch_slip();
        dbg.clutch_cap_kinetic = powertrain_.clutch_cap_kinetic();
        dbg.clutch_cap_static = powertrain_.clutch_cap_static();
        dbg.clutch_stuck = powertrain_.clutch_stuck();
        dbg.gearbox_mesh_torque = powertrain_.gearbox_mesh_torque_debug();
        dbg.diff_mesh_torque = powertrain_.diff_mesh_torque_debug();
        powertrain_.fill_hub_shaft_torque_debug(dbg.hub_shaft_torque);
        powertrain_.fill_diff_transfer_torque_debug(dbg.diff_transfer_torque); // PHYS-033 P4 I4 C8
        dbg.throttle_eff = powertrain_.throttle_eff();
        dbg.idle_integrator = powertrain_.idle_integrator();
        dbg.synchro_torque = powertrain_.synchro_torque_debug();
        dbg.engine_torque_demand = powertrain_.engine_torque_demand();
        dbg.ignition_cut = powertrain_.tcu_request_debug().ignition_cut;
        dbg.throttle_ceiling = powertrain_.tcu_request_debug().throttle_ceiling;
        dbg.awd_clutch_torque = powertrain_.awd_clutch_torque(); // PHYS-033 P4 I4 C9
        dbg.awd_clutch_slip = powertrain_.awd_clutch_slip();
        dbg.park_lock_torque = powertrain_.park_lock_torque(); // PHYS-033 P4 I4 C10
        // PHYS-033 P4 I4 C11 (plan section 3.5): the pto's own friction-row
        // torque/slip, and the first load_dyno's own applied torque - 0 for
        // every vehicle with neither.
        dbg.pto_clutch_torque = powertrain_.pto_clutch_torque();
        dbg.pto_clutch_slip = powertrain_.pto_clutch_slip();
        dbg.dyno_torque = powertrain_.dyno_torque();
        powertrain_.fill_shaft_debug(dbg.shaft_twist_rad, dbg.shaft_torque); // PHYS-033 P4 I4 C13
        return dbg;
    }

    [[nodiscard]] BodyId chassis() const { return chassis_; }
    // PHYS-033 I1 W1: called ONLY from vehicle::System::note_body_wrench,
    // itself called only from World::stage_gather (sequential, fixed
    // element order - PLAN.md D8) - never concurrently for the same
    // vehicle. Accumulates this tick's gathered force-element wrench for
    // the shadow chassis to use as a constant external acceleration next
    // tick (mirrors F12: the vehicle stage runs before force_elements, so
    // it always lags by exactly one tick, same as today's real
    // apply_wrench-based external forces).
    void add_external_wrench(const Wrench& w) { ext_wrench_accum_ += w; }

    // Diagnostic only (PHYS-033 I1 section 3.5): how far Jolt's actual
    // end-of-tick chassis velocity was from what the shadow predicted at
    // the end of the PREVIOUS tick. Near the float noise floor (~1e-6) when
    // W1's claim (section 2) holds; large whenever something outside the
    // shadow's model touches the chassis (a contact, a constraint, a
    // scenario's between-forces hook, or a test's set_motion).
    [[nodiscard]] real shadow_dv_error() const { return shadow_dv_error_; }
    [[nodiscard]] real shadow_dw_error() const { return shadow_dw_error_; }

    // PHYS-033 P4 I4 C3 (plan section 2.4): this tick's copy of every
    // generic component-declared control channel's value, control_indices_.
    // component order - the exact buffer ComponentSubstepCtx::channels
    // points at each substep. White-box test hook (same "plain public
    // accessor" pattern as e.g. TileManager::precomputed_keys()); also a
    // legitimate read for a future adapter/debug consumer.
    [[nodiscard]] const std::vector<real>& component_channel_values() const { return component_channel_values_; }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    [[nodiscard]] real ackermann_wheel_angle(std::size_t wheel_index, real delta_avg) const;

    BodyId chassis_;
    const io::SurfaceTable* surfaces_;
    ControlChannelIndices control_indices_;
    VehicleDesc desc_;
    // Declared AFTER desc_ (built FROM desc_ in the member-init list, and
    // member-init order follows DECLARATION order regardless of the list's
    // own order - reordering desc_/powertrain_ here would silently read a
    // not-yet-constructed desc_). PHYS-033 P4 I2 commit 5b: replaces the
    // TEMPORARY P3 shim (commit 5a) - the real shaft-network drivetrain,
    // built once here from desc_.powertrain/desc_.brake_system/desc_.wheels
    // (ps::drivetrain::Powertrain's constructor throws std::runtime_error
    // on any graph error, exactly like ps::io::load_vehicle_json throws on
    // a data error).
    drivetrain::Powertrain powertrain_;
    std::vector<std::unique_ptr<WheelInstance>> wheels_;
    // PHYS-033 P4 I3 plan section 6.7 (rotor reaction on the chassis): each
    // wheel hub's omega just BEFORE this substep's powertrain_.solve() call,
    // sized once at construction (zero-alloc scratch, PLAN.md D8) - wheels_
    // is never resized after the constructor, same invariant this reuses.
    std::vector<real> hub_omega_old_scratch_;

    // PHYS-033 P4 I4 C3 (plan section 2.4): this tick's copy of every
    // generic component-declared control channel's value, control_indices_.
    // component order - sized ONCE at construction (never resized after,
    // PLAN.md D8), refilled once per tick (step_tick, not per substep) from
    // `channel_values`, then pointed to by ComponentSubstepCtx::channels
    // each substep.
    std::vector<real> component_channel_values_;

    // PHYS-033 P4 I3 plan section 4.3/4.5: default-constructed until commit
    // 6 fills it from the real engine/clutch/gearbox/TCU state - see
    // powertrain_state() above.
    drivetrain::PowertrainSnapshot powertrain_state_;

    // Ackermann geometry, cached at construction from wheel attachment
    // points (vehicle_desc.h's SteeringDesc comment).
    real wheelbase_ = 2.5;
    real rear_axle_x_ = 0.0;

    real prev_steer_avg_ = 0.0; // for SteeringDesc::steer_rate_limit

    // PHYS-033 I1 W1 (section 3.2/3.3). ext_wrench_accum_ is written ONLY by
    // add_external_wrench above (World::stage_gather, sequential) and
    // consumed (then cleared) once per tick by step_tick, never hashed
    // (re-seeded every tick from already-hashed state, same reasoning as
    // articulation::Instance's external-wrench accumulator).
    Wrench ext_wrench_accum_{};
    Motion shadow_end_pred_{};   // shadow's predicted Jolt velocity at the end of the last tick (diagnostic only)
    bool shadow_valid_ = false;  // false until the first tick has run once (no prediction to compare against yet)
    real shadow_dv_error_ = 0.0;
    real shadow_dw_error_ = 0.0;
};

} // namespace ps::vehicle
