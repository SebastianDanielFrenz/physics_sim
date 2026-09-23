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
#include "ps/vehicle/wheel_state.h"
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

    // PLAN.md M1, task 2 of the P3 fix-up phase: the render-facing subset
    // (see wheel_state.h's own header comment for why this is a separate
    // type from WheelTelemetry above). Computed on demand from state
    // already held by this instance - no extra storage duplicating
    // last_telemetry_'s fields.
    [[nodiscard]] WheelState wheel_state() const {
        WheelState s;
        s.steer_angle = last_steer_angle_;
        s.suspension_travel = x_;
        s.spin_angle = spin_angle_;
        s.load = last_telemetry_.fz;
        s.slip_ratio = last_telemetry_.slip_ratio;
        s.slip_angle = last_telemetry_.slip_angle;
        s.surface = last_telemetry_.surface;
        return s;
    }

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

    // PLAN.md M1, task 2: render-only state, deliberately NOT folded into
    // fold_state_hash() below (see wheel_instance.cpp's step() for why -
    // both are write-only as far as the simulation itself is concerned,
    // never read back by anything that affects future dynamics, so
    // excluding them cannot hide a worker-count/scheduling-dependent
    // divergence - and this task's own requirement is that every existing
    // scenario's state_hash stays bit-identical).
    real spin_angle_ = 0.0;
    real last_steer_angle_ = 0.0;

    WheelTelemetry last_telemetry_;
};

} // namespace ps::vehicle
