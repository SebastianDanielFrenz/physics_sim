// ps/vehicle/wheel_state.h — per-wheel RENDER/adapter state (PLAN.md M1,
// task 2 of the P3 fix-up phase): everything a render/adapter thread needs
// to place and orient one wheel mesh - steer angle, suspension travel,
// spin angle, contact surface, load and slip. Public so World::wheel_state
// (ps/world/world.h) and WorldSnapshot (ps/world/snapshot.h) can return/
// carry it without pulling in any internal vehicle header (same D2/D10
// public-descriptor seam as ps/vehicle/wheel_telemetry.h).
//
// Deliberately a SEPARATE type from WheelTelemetry, not an extension of
// it: WheelTelemetry also carries Fx/Fy/omega (CSV telemetry/debugging,
// PLAN.md P3 scope item 7) that a mesh-placement consumer does not need,
// and carries neither of this type's two fields (steer_angle, the
// integrated spin_angle - WheelTelemetry only ever had the spin SPEED,
// omega, not the integrated angle).

#pragma once

#include "ps/types.h"

namespace ps::vehicle {

struct WheelState {
    real steer_angle = 0.0;       // rad, this wheel's own (post-Ackermann) steer angle, current tick
    real suspension_travel = 0.0; // m, 0 = full extension, WheelDesc::suspension.travel_max = full bump
    real spin_angle = 0.0;        // rad, integrated wheel spin angle about its own axis - UNBOUNDED (not wrapped to [0, 2*pi)); a renderer wraps it itself if needed
    real load = 0.0;              // N, vertical tyre force (== WheelTelemetry::fz)
    real slip_ratio = 0.0;        // == WheelTelemetry::slip_ratio
    real slip_angle = 0.0;        // == WheelTelemetry::slip_angle
    SurfaceId surface = kInvalidSurfaceId;
};

} // namespace ps::vehicle
