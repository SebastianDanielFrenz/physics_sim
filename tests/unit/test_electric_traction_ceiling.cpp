// Traction control on an electric motor: the ceiling is a fraction of the torque the motor delivers NOW (after the
// constant-power corner), not of its peak torque. Before the fix the ceiling f x peak never bit in the constant-power
// region (owner marks 2026-10-21: TC intervening at 0.26-0.45 with the dashboard still at full power).
#include "vehicle_test_helpers.h"
#include "ps/drivetrain/electric_envelope.h"
#include "ps/io/vehicle_io.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace ps;
using namespace ps::drivetrain;

namespace {
ElectricMotorDesc big_motor() {
    ElectricMotorDesc d;
    d.peak_torque_nm = 1200;
    d.max_power_w = 600000;
    d.inertia_kgm2 = 0.25;
    d.efficiency = 0.95;
    d.max_rpm = 20000;
    return d;
}
}

TEST_CASE("Electric TC ceiling is a fraction of the available torque in both regions", "[electric][traction_control]") {
    const auto d = big_motor();
    const real h = 1.0 / 960.0;
    for (const real f : {0.262, 0.45, 0.8}) {
        // constant-power region: 15000 rpm
        const real w_hi = 15000 * 2 * 3.14159265358979 / 60;
        const auto full = motor_drive_request(d, w_hi, 1.0, h);
        const auto cut = motor_drive_request(d, w_hi, 1.0, h, f);
        CHECK(full.power_limited);
        CHECK(cut.torque_nm == Catch::Approx(f * full.torque_nm).epsilon(0.02));
        // the old behaviour (command = f) left the corner untouched at this speed
        const auto old = motor_drive_request(d, w_hi, f, h);
        if (f < 0.5) CHECK(old.torque_nm > 1.5 * cut.torque_nm);
        CHECK(old.torque_nm >= cut.torque_nm);
        // constant-torque region: unchanged law (command f == scale f at pedal 1)
        const real w_lo = 100;
        const auto lo_old = motor_drive_request(d, w_lo, f, h);
        const auto lo_new = motor_drive_request(d, w_lo, 1.0, h, f);
        CHECK(lo_new.torque_nm == Catch::Approx(lo_old.torque_nm).epsilon(1e-9));
    }
    // scale 1 is bit-identical
    const auto a = motor_drive_request(d, 900, 0.7, h), b = motor_drive_request(d, 900, 0.7, h, 1.0);
    CHECK(a.torque_nm == b.torque_nm);
    CHECK(a.wanted_j == b.wanted_j);
}

TEST_CASE("Electric TC cuts motor power in a low-grip pull through the constant-power region", "[electric][traction_control][vehicle][long]") {
    auto desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_electric.json");
    for (auto& c : desc.powertrain.components) {
        if (auto* m = std::get_if<ElectricMotorDesc>(&c.params)) { m->peak_torque_nm *= 2; m->max_power_w *= 2; }
        if (auto* b = std::get_if<BatteryDesc>(&c.params)) b->max_discharge_w = 3.0e6;
    }
    for (auto& w : desc.wheels) { w.tyre.pdx1 *= 0.35; w.tyre.pdy1 *= 0.35; }
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1850, {2, .4, .15}, .6, 0, 4);
    World& w = *rig.world;
    w.set_control("assist.traction_control", 1);
    ps_test::settle(w, .5);
    w.set_control("gear_request", 1);
    w.set_control("throttle", 1);
    int n = 0;
    double sum_p = 0, sum_f = 0, sum_ratio = 0, peak_kappa = 0, peak_p = 0;
    for (int i = 0; i < 240 * 14; ++i) {
        w.step();
        const auto p = w.powertrain_state(rig.vehicle);
        const real v = w.get_motion(rig.chassis).linear.x;
        peak_p = std::max<double>(peak_p, p.electric_power_w);
        if (v > 26 && p.traction_control_intervening && p.motor_rpm > 6000) {
            ++n;
            sum_p += p.electric_power_w;
            sum_f += p.traction_torque_fraction;
            sum_ratio += p.electric_power_w / (p.traction_torque_fraction * 1.2e6 + 1);
            for (int k = 0; k < 4; ++k) peak_kappa = std::max<double>(peak_kappa, std::abs(w.wheel_telemetry(rig.vehicle, k).slip_ratio));
        }
    }
    REQUIRE(n > 50);
    std::printf("EV TC low-mu pull: %d samples, mean power %.1f kW (peak %.1f), mean f %.3f, power/(f*1200kW) %.3f, peak |kappa| %.3f\n",
                n, sum_p / n / 1e3, peak_p / 1e3, sum_f / n, sum_ratio / n, peak_kappa);
    CHECK(sum_p / n < 0.85 * 1.2e6);
    CHECK(sum_ratio / n < 1.15);
    CHECK(peak_kappa < 0.5);
}
