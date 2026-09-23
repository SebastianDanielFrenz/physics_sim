// ps/vehicle/wheel_telemetry.h — per-wheel telemetry snapshot (PLAN.md
// P3 scope item 7: "writing telemetry channels per wheel"). Public so
// World::wheel_telemetry (ps/world/world.h) can return it without pulling
// in any internal vehicle header.

#pragma once

#include "ps/types.h"

namespace ps::vehicle {

struct WheelTelemetry {
    real slip_ratio = 0.0;
    real slip_angle = 0.0;
    real fx = 0.0;
    real fy = 0.0;
    real fz = 0.0;
    real omega = 0.0;
    real compression = 0.0;
    SurfaceId surface = kInvalidSurfaceId;
};

} // namespace ps::vehicle
