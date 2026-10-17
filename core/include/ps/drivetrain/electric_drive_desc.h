#pragma once
#include "ps/math/vec3.h"
#include "ps/types.h"
#include <string>
namespace ps::drivetrain {
// Independent electrical stores and shaft machines: an ICE can coexist on the graph.
struct BatteryDesc {
    real capacity_kwh = 90;
    real initial_soc = .9;
    real max_discharge_w = 1000000;
    real max_charge_w = 250000;
    real mass_kg = 500;
};
struct ElectricMotorDesc {
    std::string battery; // component id; multiple machines may share a store
    real peak_torque_nm = 600;
    real max_power_w = 400000;
    real max_rpm = 20000;
    real inertia_kgm2 = .25;
    real efficiency = .95;
    real regen_torque_nm = 150;
    real mass_kg = 80;
    Vec3 axis_local = Vec3::unit_y();
    // Bare throttle/brake/gear_request are defaults. Independent channels allow hybrid axle control.
    std::string ready_channel="electric.ready"; // independent of ICE ignition for hybrids
    std::string throttle_channel;
    std::string regen_channel;
    std::string direction_channel;
};
}
