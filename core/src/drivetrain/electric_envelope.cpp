// ps/drivetrain/electric_envelope.h and hybrid_split.h - the one implementation of the motor law, the DC supply budget and
// the hybrid power split; the runtime (electric_drive.h, powertrain.cpp) and the dyno both call these. The expressions keep
// the exact operation order of the code they were extracted from (bit-identical runtime, test_electric_pins.cpp).
#include "ps/drivetrain/electric_envelope.h"
#include "ps/drivetrain/hybrid_split.h"

#include <algorithm>
#include <cmath>

namespace ps::drivetrain {

namespace {
constexpr real kPi = 3.14159265358979323846;
}

const char* to_string(MotorLimit l) {
    switch (l) {
    case MotorLimit::Idle: return "idle";
    case MotorLimit::PeakTorque: return "peak_torque";
    case MotorLimit::RatedPower: return "rated_power";
    case MotorLimit::BatteryDischarge: return "battery_discharge";
    case MotorLimit::MaxRpm: return "max_rpm";
    case MotorLimit::Empty: return "empty";
    }
    return "unknown";
}

MotorDriveRequest motor_drive_request(const ElectricMotorDesc& d, real abs_omega, real command, real h) {
    MotorDriveRequest r;
    real demand = d.peak_torque_nm * command;
    // Conservative forward-power prediction using rotor inertia; shaft dynamics are solved by the network.
    const real projected = abs_omega + h * demand / d.inertia_kgm2;
    const real corner = d.max_power_w / std::max(real(1), projected);
    r.power_limited = corner < demand;
    demand = std::min(demand, corner);
    r.torque_nm = demand;
    r.wanted_j = h * demand * std::max(real(1), projected) / d.efficiency;
    return r;
}

real motor_drive_torque(const ElectricMotorDesc& d, const MotorDriveRequest& request, real granted_j, real content_j, real h,
                        MotorLimit* limit) {
    real demand = request.torque_nm;
    const real wanted = request.wanted_j;
    if (wanted > 0) demand *= granted_j / wanted;
    // Strict residual-energy bound near empty, including a back-driven hybrid shaft.
    const real max_omega = d.max_rpm * (2 * kPi) / 60;
    const real residual = content_j * d.efficiency / (h * std::max(real(1), max_omega));
    const bool residual_limits = residual < demand;
    demand = std::min(demand, residual);
    if (limit != nullptr) {
        if (wanted > 0 && !(granted_j > 0)) *limit = MotorLimit::Empty;
        else if (granted_j < wanted || residual_limits) *limit = demand > 0 ? MotorLimit::BatteryDischarge : MotorLimit::Empty;
        else if (request.power_limited) *limit = MotorLimit::RatedPower;
        else *limit = MotorLimit::PeakTorque;
    }
    return demand;
}

bool motor_drives(const ElectricMotorDesc& d, real pedal, real regen, real direction, real rpm) {
    return pedal > .02 && regen <= .05 && direction != 0 && rpm < d.max_rpm;
}

MotorFullLoadPoint motor_full_load_point(const ElectricMotorDesc& d, real rpm, real electrical_budget_w, real h, real content_j) {
    MotorFullLoadPoint p;
    const real omega = std::abs(rpm) * (2 * kPi) / 60;
    if (!(std::abs(rpm) < d.max_rpm)) {
        p.limit = MotorLimit::MaxRpm;
        return p;
    }
    const MotorDriveRequest r = motor_drive_request(d, omega, real(1), h);
    // Battery::reserve_discharge: min(budget left, max(0, wanted)).
    const real granted = std::min(std::max(real(0), electrical_budget_w) * h, std::max(real(0), r.wanted_j));
    p.torque_nm = motor_drive_torque(d, r, granted, content_j, h, &p.limit);
    p.power_w = p.torque_nm * omega;
    p.electrical_w = p.power_w / d.efficiency;
    return p;
}

real battery_discharge_budget_j(const BatteryDesc& b, real content_j, real h) { return std::min(content_j, b.max_discharge_w * h); }

real battery_discharge_budget_w(const BatteryDesc& b, real soc, real h) {
    return h > 0 ? battery_discharge_budget_j(b, b.capacity_kwh * 3600000 * soc, h) / h : real(0);
}

real battery_charge_budget_j(const BatteryDesc& b, real content_j, real h) {
    return std::min(b.capacity_kwh * 3600000 - content_j, b.max_charge_w * h);
}

real fuel_cell_request_j(const FuelCellDesc& f, real hydrogen_kg, real battery_soc, real h) {
    if (!(hydrogen_kg > 0) || !(h > 0)) return 0;
    const real fraction = std::clamp((f.target_soc - battery_soc) / real(.05), real(0), real(1));
    const real available = hydrogen_kg * f.specific_energy_j_kg * f.efficiency;
    return std::min(available, h * f.max_power_w * fraction);
}

real fuel_cell_supply_w(const FuelCellDesc& f, const BatteryDesc& b, real soc, real hydrogen_kg, real h) {
    if (!(h > 0)) return 0;
    const real capacity = b.capacity_kwh * 3600000;
    const real content = capacity * soc;
    // Battery::reserve_charge then Battery::supply (set_content clamps to the capacity).
    const real reserved = std::min(battery_charge_budget_j(b, content, h), std::max(real(0), fuel_cell_request_j(f, hydrogen_kg, soc, h)));
    const real added = std::max(real(0), std::clamp(content + reserved, real(0), capacity) - content);
    return added / h;
}

bool hybrid_update_charge_latch(const HybridControlDesc& c, real battery_soc, bool latched) {
    if (battery_soc <= c.low_soc) latched = true;
    if (battery_soc >= c.charge_target_soc - real(1e-6)) latched = false;
    return latched;
}

HybridSplit hybrid_power_split(const HybridControlDesc& c, const HybridSplitInput& in) {
    HybridSplit s;
    s.charge_latched = hybrid_update_charge_latch(c, in.battery_soc, in.charge_latched);
    const int mode = in.drive_mode;
    s.requested_w = in.driver_command * c.requested_power_w;
    s.extra_power = mode != 0 && s.requested_w > in.available_w;
    s.engine_coupled = in.selector != 0 && mode != 0 && (s.extra_power || mode >= 2);
    s.engine_wanted = in.ready && (s.charge_latched || mode >= 2 || s.extra_power);
    s.electric_command = in.ready ? std::clamp(s.requested_w / std::max(real(1), in.available_w), real(0), real(1)) : real(0);
    s.engine_command = s.engine_coupled
                           ? std::clamp((s.requested_w - in.available_w) / std::max(real(1), c.requested_power_w - in.available_w), real(0), real(1))
                           : real(0);
    return s;
}

HybridSplit hybrid_full_pedal_split(const HybridControlDesc& c, real available_w, real pedal, int drive_mode, real battery_soc,
                                    bool charge_latched) {
    HybridSplitInput in;
    in.driver_command = std::clamp(pedal, real(0), real(1));
    in.available_w = available_w;
    in.drive_mode = std::clamp(drive_mode, 0, 3);
    in.battery_soc = battery_soc;
    in.charge_latched = charge_latched;
    return hybrid_power_split(c, in);
}

} // namespace ps::drivetrain
