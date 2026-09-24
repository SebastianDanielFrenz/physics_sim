// core/src/vehicle/vehicle_instance.h — one vehicle: chassis extrapolation
// across a tick's substeps (PLAN.md D3), Ackermann steering, brake/drive
// torque split, one WheelInstance per corner. Internal (owned by
// system.h).

#pragma once

#include "wheel_instance.h"

#include "ps/io/surface_table.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/ids.h"

#include <cstdint>
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
};

// PHYS-033 I1 W1: environment inputs step_tick's shadow chassis needs but
// that VehicleInstance itself has no other way to see (it only ever talks
// to IRigidBackend, never World) - just World::config_.gravity today.
struct VehicleTickEnv {
    Vec3 gravity = Vec3{0.0, 0.0, -9.81};
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

    [[nodiscard]] std::size_t wheel_count() const { return wheels_.size(); }
    [[nodiscard]] const WheelTelemetry& wheel_telemetry(std::size_t i) const { return wheels_[i]->telemetry(); }
    [[nodiscard]] const std::string& wheel_name(std::size_t i) const { return wheels_[i]->desc().name; }
    // PLAN.md M1, task 2 of the P3 fix-up phase: see wheel_state.h.
    [[nodiscard]] WheelState wheel_state(std::size_t i) const { return wheels_[i]->wheel_state(); }

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

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    [[nodiscard]] real ackermann_wheel_angle(std::size_t wheel_index, real delta_avg) const;

    BodyId chassis_;
    const io::SurfaceTable* surfaces_;
    ControlChannelIndices control_indices_;
    VehicleDesc desc_;
    std::vector<std::unique_ptr<WheelInstance>> wheels_;

    // Ackermann geometry, cached at construction from wheel attachment
    // points (vehicle_desc.h's SteeringDesc comment).
    real wheelbase_ = 2.5;
    real rear_axle_x_ = 0.0;

    real prev_steer_avg_ = 0.0; // for SteeringDesc::steer_rate_limit

    // PLAN.md P4 I0 (PHYS-033): preallocated at construction (outside any
    // job), sized to wheels_.size() and never resized again - step_tick ran
    // inside System::step_all's parallel_for job and used to heap-allocate
    // a fresh std::vector<real> here every single tick (D8 violation, found
    // by tests/unit/test_vehicle_parallel_alloc.cpp).
    std::vector<real> brake_torque_scratch_;

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
