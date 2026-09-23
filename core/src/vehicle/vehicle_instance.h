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

class VehicleInstance {
public:
    VehicleInstance(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                     const ControlChannelIndices& control_indices);

    // Runs every substep in this tick (PLAN.md D3: chassis state
    // extrapolated within the tick from its start pose/motion), then
    // applies the substep-averaged wrench once via backend.apply_wrench
    // (out/handover_p3.md decision 3b). `channel_values` is World's flat
    // control_channel_values_ array, indexed by control_indices_ below.
    void step_tick(IRigidBackend& backend, const std::vector<real>& channel_values, real tick_dt, real substep_rate_hz);

    [[nodiscard]] std::size_t wheel_count() const { return wheels_.size(); }
    [[nodiscard]] const WheelTelemetry& wheel_telemetry(std::size_t i) const { return wheels_[i]->telemetry(); }
    [[nodiscard]] const std::string& wheel_name(std::size_t i) const { return wheels_[i]->desc().name; }
    // PLAN.md M1, task 2 of the P3 fix-up phase: see wheel_state.h.
    [[nodiscard]] WheelState wheel_state(std::size_t i) const { return wheels_[i]->wheel_state(); }

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
};

} // namespace ps::vehicle
