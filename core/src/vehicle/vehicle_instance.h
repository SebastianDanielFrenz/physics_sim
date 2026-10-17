// core/src/vehicle/vehicle_instance.h — one vehicle: chassis extrapolation
// across a tick's substeps (PLAN.md D3), Ackermann steering, brake/drive
// torque split, one WheelInstance per corner. Internal (owned by
// system.h).

#pragma once

#include "../drivetrain/powertrain.h"
#include "abs_esc.h"
#include "launch_assist.h"
#include "wheel_instance.h"

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/ambient.h"
#include "ps/world/ids.h"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace ps::vehicle {

// Assists I7c: a control-channel write a vehicle queues during its step (index into World's flat channel array).
struct PendingChannelWrite {
    std::size_t index = 0;
    real value = 0;
};

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
    std::size_t assist_auto_clutch = 0, assist_auto_blip = 0, assist_auto_shift = 0, assist_traction_control = 0;
    std::size_t assist_launch_control = 0; // engine garage P8: only read when the vehicle has a launch_control controller
    std::size_t assist_aero_control = 0;   // assists I5: only read when the car's assist_unit aero_control is fitted
    std::size_t assist_drive_mode = 0;     // assists I7: only read when the car has a drive_modes part
    std::size_t epb = 0;                   // Q3 / I8: only read when the car has an epb part
    std::size_t assist_abs = 0;            // ABS (I5 part 2): only read when the car's assist unit carries an abs block
    std::size_t assist_esc = 0;            // ESC (I6): only read when the car's assist unit carries an esc block
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
    Vec3 wind_world{};
    std::span<const aero::WakeSource> wakes{}, extra_wakes{};
    // W6: the installed environment's surface-condition grip (nullptr = none);
    // World fills it per tick from the environment. Without it the wheel's
    // friction is exactly the class lambda_mu (bit-identical to before W6).
    const environment::ConditionGrip* grip = nullptr;
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

    // Assists I7c: the channel writes the last step_tick queued (drive-mode defaults), applied by World::step after the
    // pipeline in vehicle id order. Valid until the next step_tick / reset.
    [[nodiscard]] std::span<const PendingChannelWrite> pending_channel_writes() const {
        return std::span<const PendingChannelWrite>(pending_writes_.data(), pending_write_count_);
    }

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
    // Assists I2a (diag only): wheel i's telemetry after substep s of the last tick (s < substep_telemetry_count()).
    static constexpr std::size_t kMaxSubstepTelemetry = 16;
    [[nodiscard]] std::size_t substep_telemetry_count() const { return substep_telemetry_count_; }
    [[nodiscard]] const WheelTelemetry& substep_wheel_telemetry(std::size_t s, std::size_t i) const {
        return substep_telemetry_[s * wheels_.size() + i];
    }
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
    // N2O plan N1: backs World::n2o_bottle_mass/set_n2o_bottle_mass and the kit-only telemetry channels.
    [[nodiscard]] bool has_nitrous() const { return powertrain_.has_nitrous(); }
    [[nodiscard]] real n2o_bottle_mass() const { return powertrain_.n2o_bottle_mass(); }
    void request_n2o_bottle_mass(real kg) { powertrain_.request_n2o_bottle_mass(kg); }
    // Engine thermal plan T1: backs World::engine_thermal_state/set_engine_thermal_state and the thermal telemetry.
    [[nodiscard]] bool has_thermal() const { return powertrain_.has_thermal(); }
    [[nodiscard]] const drivetrain::EngineThermalSnapshot& thermal_snapshot() const { return powertrain_.thermal_snapshot(); }
    [[nodiscard]] const drivetrain::EngineThermal* thermal() const { return powertrain_.thermal(); }
    void request_thermal_state(const drivetrain::EngineThermalState& s) { powertrain_.request_thermal_state(s); }
    void request_cooling_capacity(const drivetrain::CoolingCapacity& c) { powertrain_.request_cooling_capacity(c); }
    void request_engine_wear(const drivetrain::EngineWear& w) { powertrain_.request_engine_wear(w); }
    [[nodiscard]] const drivetrain::NitrousSystem* nitrous_system() const { return powertrain_.nitrous(); }
    // PHYS-033 P4 I7c C16b: backs World::replace_engine_maps (queues) and
    // vehicle::System::apply_pending_engine_maps_swaps (applies, from
    // World::step before the tick pipeline, outside any job - see
    // Powertrain::apply_pending_engine_maps_swap's own comment).
    drivetrain::PreparedTurboEnvelope prepare_turbo_envelope(std::string_view component_id,const drivetrain::TurboPairDesc& desc) const {
        return powertrain_.prepare_turbo_envelope(component_id,desc);
    }
    void replace_turbo_configuration(std::string_view component_id,const drivetrain::TurboPairDesc& desc,
                                     const drivetrain::PreparedTurboEnvelope* prepared=nullptr) {
        powertrain_.replace_turbo_configuration(component_id,desc,prepared);
    }
    void request_replace_engine_maps(std::string_view component_id, drivetrain::EngineMapsPtr new_maps) {
        powertrain_.request_replace_engine_maps(component_id, std::move(new_maps));
    }
    void apply_pending_engine_maps_swap() { powertrain_.apply_pending_engine_maps_swap(); }
    [[nodiscard]] std::uint64_t simulated_engine_maps_content_hash() const {
        return powertrain_.simulated_engine_maps_content_hash();
    }
    // Weather plan W4: backs World::vehicle_engine_steady_rebuild_count / vehicle_engine_ambient (tests, diagnostics).
    [[nodiscard]] std::uint64_t engine_steady_rebuild_count() const { return powertrain_.simulated_engine_steady_rebuild_count(); }
    [[nodiscard]] const AmbientSample& engine_sensors_ambient() const { return powertrain_.last_engine_sensors_ambient(); }
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
        dbg.clutch_capacity_cap_tc = powertrain_.clutch_capacity_cap_tc(); // assists I2
        dbg.tc_via_clutch = powertrain_.tc_via_clutch();
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
        for (std::size_t i = 0; i < wheel_brake_.size() && i < 8; ++i) { // assists I5 part 2 / I6
            dbg.abs_wheel_scale[i] = abs_esc_state_.applied[i];
            dbg.esc_wheel_brake[i] = abs_esc_state_.esc_brake[i];
            dbg.wheel_brake_command[i] = wheel_brake_[i];
        }
        return dbg;
    }

    [[nodiscard]] BodyId chassis() const { return chassis_; }
    [[nodiscard]] const aero::WakeDesc& aero_wake() const { return desc_.aero.wake; }
    [[nodiscard]] const aero::Telemetry& aero_telemetry() const { return aero_telemetry_; }
    void set_aero_surface_command(std::size_t surface, real command);
    void set_aero_fan_command(std::size_t fan, real command);
    void credit_aero_fan_energy(real joules);
    // Assists I5: the hook a launch control (assists I4) sets each tick BEFORE step_tick - while true the aero
    // controller asks for launch_offset_deg (yield order LC > airbrake > cruise). Never hashed, always false in I5.
    void set_aero_launch_request(bool request) { aero_launch_request_ = request; }

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
    bool has_axle_balance_=false;
    real wheelbase_ = 2.5;
    real rear_axle_x_ = 0.0;

    // Engine garage P8 (A15): which assists this car has FITTED (a not-fitted assist ignores its channel). Computed
    // once in the constructor from the TCU blocks, traction_control.fitted and the launch_control controller.
    bool fit_clutch_ = true, fit_blip_ = true, fit_shift_ = true, fit_traction_ = true, fit_launch_ = false;
    bool launch_on_dct_ = false; // the assist unit's launch control drives a DualClutch's launch clutch
    bool has_manual_tcu_ = false; // automatic gearbox plan A1: the assist unit's launch control needs the manual_tcu
    std::uint32_t fitted_bits_ = 0; // snapshot layout: bit0 auto_clutch .. bit3 traction_control, bit4 launch_control, bit5 anti_lag switch, bit6 aero_control (assists I5)
    // Assists I1: per-mode controller parameters (index = TractionMode code; 0 and 1 are the car's own desc) and the
    // offered-mode bits. Both are pure functions of desc_, computed once in the constructor.
    std::array<TractionControlDesc, kTractionModeCount> tc_mode_desc_{};
    std::uint8_t tc_modes_available_bits_ = kTractionModesBaseBits;
    // Assists I7 / I7c: drive-mode state, hashed only on a car with a drive_modes part. initialised = the first tick has
    // seen the mode (no write at spawn: the TC / LC channel defaults already equal the default mode's); last_mode = the
    // effective drive mode of the previous tick. There is NO override state (I7c): on a mode change the mode's
    // traction_control_default / launch_control_default is used for that tick and queued as a pending channel write
    // (pending_writes_) that World::step applies to the control channel after the pipeline, so the channel always
    // equals what the car does and the driver's next write (any value) wins at once.
    struct DriveModeState {
        bool initialised = false;
        std::uint8_t last_mode = 0;
    };
    DriveModeState drive_state_{};
    // Q3 / I8: electronic parking brake state, hashed only on a car with an epb part. last_level = the epb channel level
    // (>= 0.5) of the previous tick (rising-edge detection); refused = the held request was refused (dropped until the
    // level falls); state = EpbState code. Kept across reset() (a teleport does not release a parking brake).
    struct EpbRuntime {
        bool last_level = false;
        bool refused = false;
        std::uint8_t state = 0;
    };
    EpbRuntime epb_{};
    bool lc_default_state_ = false; // I7b: some offered drive mode sets launch_control_default
    // Assists I7c: channel writes a drive-mode change queues from inside step_tick (a vehicle job cannot touch World's
    // channel map). At most two (TC code, LC enable); cleared at the start of every step_tick; never hashed.
    std::array<PendingChannelWrite, 4> pending_writes_{}; // TC, LC, ABS, ESC
    std::size_t pending_write_count_ = 0;
    // Assists I5: the assist unit's rear-wing automation. fit_aero_ = block present AND the car has the named surface;
    // the surface index and the launch-control hook are plain per-tick inputs, none of it hashed.
    bool fit_aero_ = false;
    std::size_t aero_control_surface_ = 0;
    bool aero_launch_request_ = false;
    // Assists I4: the assist unit's launch control (present iff the unit carries the block, which is also what fits the
    // assist). Its hashed state is folded only then; launch_slip_target_ is the last substep's snapshot value.
    std::optional<LaunchAssist> launch_;
    real launch_slip_target_ = 0;
    // Assists I5 part 2 / I6: ABS and ESC (abs_esc.h). fit_* = the assist unit carries the block; the state is hashed
    // only on a fitted car. wheel_brake_ is the per-wheel service command handed to the brake rows (ctx.wheel_brake) on a
    // substep where either runs; is_front_ caches WheelDesc::is_front. All sized once in the constructor.
    bool fit_abs_ = false, fit_esc_ = false;
    AbsEscState abs_esc_state_{};
    std::vector<real> wheel_brake_;
    std::vector<std::uint8_t> is_front_;
    real esc_front_angle_ = 0; // last substep's mean front road-wheel angle (snapshot only)
    TractionControlState traction_state_{};
    std::vector<TractionWheelSensor> traction_sensors_;
    std::vector<std::uint8_t> traction_brake_bits_; // per wheel: bit0 service brake fitted, bit1 handbrake fitted (TC brake signal)
    std::vector<WheelTelemetry> substep_telemetry_; // assists I2a diag: kMaxSubstepTelemetry x wheels, sized at construction
    std::size_t substep_telemetry_count_ = 0;
    Vec3 imu_previous_velocity_world_{}; // virtual accelerometer plumbing only; never supplied as TC speed
    bool imu_have_previous_=false;
    aero::State aero_state_{};
    aero::Telemetry aero_telemetry_{};
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
