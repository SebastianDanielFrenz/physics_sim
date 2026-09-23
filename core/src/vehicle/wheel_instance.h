// core/src/vehicle/wheel_instance.h — one wheel: suspension travel
// (measured kinematically via a cylinder shape cast, out/handover_p3.md
// decision 1), tyre slip relaxation state (decision 2) and spin, brakes.
// Internal (PLAN.md D2/D10 pattern: no Jolt type here; owned by
// vehicle_instance.h, itself owned by system.h - never included from a
// public header).

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/surface_table.h"
#include "ps/vehicle/tyre_model.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/wheel_telemetry.h"

#include <cstdint>
#include <memory>

namespace ps::vehicle {

struct WheelSubstepInput {
    Pose chassis_pose;
    Motion chassis_motion;
    Vec3 chassis_com_world = Vec3::zero();
    real steer_angle = 0.0;
    real brake_torque_cmd = 0.0; // N*m, magnitude, opposes spin (see .cpp for the lock-to-zero clamp)
    real drive_torque_cmd = 0.0; // N*m, signed, applied directly to wheel spin
    real dt = 0.0;
    real partner_compression = 0.0;
    const io::SurfaceTable* surfaces = nullptr;
};

struct WheelSubstepOutput {
    Vec3 suspension_force_world = Vec3::zero();
    Vec3 attachment_point_world = Vec3::zero(); // where suspension_force_world acts
    Vec3 tyre_force_world = Vec3::zero();       // Fx (incl. rolling resistance) + Fy, ground plane
    Vec3 contact_point_world = Vec3::zero();    // where tyre_force_world (and Mz/Mx below) act
    Vec3 extra_torque_world = Vec3::zero();     // Mz + Mx as a pure couple (no torque-arm needed)
};

class WheelInstance {
public:
    WheelInstance(WheelDesc desc, std::unique_ptr<ITyreModel> tyre_model);

    [[nodiscard]] real compression() const { return x_; }
    [[nodiscard]] real spin_speed() const { return omega_; }

    // Pass 1 (every substep, before step() below - ARB needs every wheel's
    // fresh measurement first): cylinder shape cast along the suspension
    // axis to find ground contact and this substep's compression.
    void measure_contact(const Pose& chassis_pose, real steer_angle, const IRigidBackend& backend);

    // Pass 2: suspension force from the measured compression, tyre force
    // from relaxed slip, wheel spin integration.
    [[nodiscard]] WheelSubstepOutput step(const WheelSubstepInput& in);

    [[nodiscard]] const WheelTelemetry& telemetry() const { return last_telemetry_; }
    [[nodiscard]] const WheelDesc& desc() const { return desc_; }

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    WheelDesc desc_;
    std::unique_ptr<ITyreModel> tyre_;

    real x_ = 0.0;
    real x_prev_ = 0.0;
    bool hit_ground_ = false;
    Vec3 contact_point_world_ = Vec3::zero();
    SurfaceId contact_surface_ = kInvalidSurfaceId;

    real omega_ = 0.0;
    SlipRelaxationState relax_;

    WheelTelemetry last_telemetry_;
};

} // namespace ps::vehicle
