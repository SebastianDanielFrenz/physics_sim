// ps/drivetrain/brake_system_desc.h — public descriptor for direct-torque
// brakes (PHYS-033 P4 I2 plan section 2, F1/Q1 friction rows). Pure data;
// see powertrain_desc.h's header comment for the general D2/D10 pattern.

#pragma once

#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::drivetrain {

struct BrakeCornerDesc {
    std::string wheel; // WheelDesc::name, must exist and be unique across corners
    real max_torque_nm = 0.0;
};

struct HandbrakeDesc {
    std::vector<std::string> wheels; // WheelDesc::name list
    real max_torque_nm = 0.0;
};

enum class BrakeSystemType { DirectTorque }; // I5 adds Hydraulic

struct BrakeSystemDesc {
    BrakeSystemType type = BrakeSystemType::DirectTorque;
    std::vector<BrakeCornerDesc> corners;
    HandbrakeDesc handbrake;
    // direct_torque: 1.0 reproduces P3 (no static/kinetic distinction) -
    // cap_static = static_factor * cap_kinetic (>= 1).
    real static_factor = 1.0;
};

} // namespace ps::drivetrain
