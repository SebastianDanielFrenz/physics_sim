// tests/unit/test_powertrain_dyno.cpp - powertrain dyno (2026-10-19): the electric envelope, the hybrid split, the
// proportional discharge allocation and run_powertrain_dyno.
//
// E1 envelope: motor_full_load_point against hand values (peak torque, the rated-power corner with the rotor-inertia
//    projection, a battery budget, an empty budget, max rpm); battery_discharge_budget_w and fuel_cell_supply_w likewise.
// S1 split: hybrid_full_pedal_split per drive mode and pedal against hand values, plus the charge latch.
// A1 allocation: Battery::reserve_discharge shares a short budget in the claims' ratio in either order, keeps the old rule
//    when the pack is not the limit or nothing was claimed.
// P1 EV: car_electric on the motor axis - peak torque, rated-power corner, the diff's lagged mesh loss
//    (P_out = P_in / (2 - eta)), the pack's draw; with a 100 kW pack both motors get half the pack (A1 in the car).
// P2 FCEV: car_fuel_cell - the stack's supply equals the hand fuel-cell law; with a 10 kW pack and a 30 kW stack the motors
//    share 40 kW of electrical power.
// P3 parallel hybrid: car_hybrid on the engine axis in 3rd - Normal: engine = WOT curve, motor at the pack limit, system
//    power from both through the gear/reduction/diff losses, supervisor numbers; Eco: engine off, neutral, motor only.
// P4 ICE only: car_sedan (torque map) and car_sedan_sim (simulated NA) net crank torque against run_dyno_sweep (1e-12);
//    P4b [long] car_hyper (twin turbo, Sketch maps) in 4th against run_dyno_sweep (1e-5; measured <= 4e-7).
// P5 1 vs 10 workers vs serial: bit-identical in every field. P6 cancel and rejected specs.
// A1-A6 (end of file): automatic, dual-clutch and CVT gearboxes, see the comment there.
//
// Sabotages (each reverted; results in the increment's report): reserve_discharge back to component order (A1, P1 fail);
// dropping the engine's Coulomb row from the dyno's net torque (P4 fails); hybrid engine command from requested/requested
// instead of the deficit (S1, P3 fail); fuel-cell SOC ramp width 0.05 -> 0.1 (E1, P2 fail); the motor's electrical power
// without the efficiency in motor_full_load_point (E1 fails); the EngineRpm axis probed on the gearbox output (P3 fails).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "drivetrain/electric_drive.h"
#include "ps/drivetrain/electric_envelope.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/hybrid_split.h"
#include "ps/drivetrain/powertrain_dyno.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr real kH = 1.0 / 960.0;
constexpr real kPi = 3.14159265358979323846;

vehicle::VehicleDesc load(const char* name) { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/" + name); }

void set_pack(vehicle::VehicleDesc& d, real discharge_w) {
    for (auto& c : d.powertrain.components)
        if (auto* b = std::get_if<BatteryDesc>(&c.params)) b->max_discharge_w = discharge_w;
}

template <class T> const T& first(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* p = std::get_if<T>(&c.params)) return *p;
    throw std::runtime_error("component not found");
}

template <class T> const T& by_id(const vehicle::VehicleDesc& d, const char* id) {
    for (const auto& c : d.powertrain.components)
        if (c.id == id) return std::get<T>(c.params);
    throw std::runtime_error(std::string("component not found: ") + id);
}

real rad_s(real rpm) { return rpm * 2 * kPi / 60; }

void print(const char* title, const PowertrainDynoResult& r) {
    std::printf("%s\n", title);
    for (const auto& p : r.points) {
        std::printf("  x %8.1f v %6.2f eng %7.1f rpm %7.2f Nm cmd %.3f run %d gear %d | sys %8.2f kW %8.2f Nm wheel %8.2f | el %8.2f bat %8.2f fc %7.2f kW lim %d conv %d %.1fs",
                    p.axis_value, p.road_speed_mps, p.engine_rpm, p.engine_torque_nm, p.engine_command, p.engine_running ? 1 : 0, p.gear,
                    p.system_power_w / 1000, p.system_torque_nm, p.system_wheel_torque_nm, p.electric_power_w / 1000, p.battery_power_w / 1000,
                    p.fuel_cell_power_w / 1000, p.battery_limited ? 1 : 0, p.converged ? 1 : 0, p.settle_s);
        for (const auto& m : p.motors)
            std::printf(" | m %7.1f rpm %7.2f Nm %7.2f kW %s", m.rpm, m.torque_nm, m.power_w / 1000, to_string(m.limit));
        std::printf("\n");
    }
}

bool same(const PowertrainDynoResult& a, const PowertrainDynoResult& b) {
    if (a.points.size() != b.points.size()) return false;
    for (std::size_t i = 0; i < a.points.size(); ++i) {
        const auto& p = a.points[i];
        const auto& q = b.points[i];
        const real fa[] = {p.axis_value, p.road_speed_mps, p.engine_rpm, p.gearbox_output_rpm, p.engine_torque_nm, p.engine_power_w,
                           p.engine_command, p.electric_power_w, p.battery_power_w, p.fuel_cell_power_w, p.system_power_w,
                           p.system_wheel_torque_nm, p.system_torque_nm, p.boost_bar, p.settle_s, p.hybrid_requested_w, p.hybrid_available_w};
        const real fb[] = {q.axis_value, q.road_speed_mps, q.engine_rpm, q.gearbox_output_rpm, q.engine_torque_nm, q.engine_power_w,
                           q.engine_command, q.electric_power_w, q.battery_power_w, q.fuel_cell_power_w, q.system_power_w,
                           q.system_wheel_torque_nm, q.system_torque_nm, q.boost_bar, q.settle_s, q.hybrid_requested_w, q.hybrid_available_w};
        if (std::memcmp(fa, fb, sizeof fa) != 0) return false;
        if (p.engine_running != q.engine_running || p.gear != q.gear || p.battery_limited != q.battery_limited || p.converged != q.converged ||
            p.hybrid_engine_coupled != q.hybrid_engine_coupled || p.hybrid_charging != q.hybrid_charging || p.motors.size() != q.motors.size())
            return false;
        for (std::size_t m = 0; m < p.motors.size(); ++m) {
            const auto& x = p.motors[m];
            const auto& y = q.motors[m];
            if (std::memcmp(&x.rpm, &y.rpm, sizeof(real)) != 0 || std::memcmp(&x.torque_nm, &y.torque_nm, sizeof(real)) != 0 ||
                std::memcmp(&x.power_w, &y.power_w, sizeof(real)) != 0 || std::memcmp(&x.electrical_w, &y.electrical_w, sizeof(real)) != 0 ||
                x.limit != y.limit || x.crank_machine != y.crank_machine)
                return false;
        }
    }
    return true;
}

ElectricMotorDesc test_motor() {
    ElectricMotorDesc m;
    m.peak_torque_nm = 600;
    m.max_power_w = 300000;
    m.max_rpm = 16000;
    m.inertia_kgm2 = 0.05;
    m.efficiency = 0.95;
    return m;
}

} // namespace

TEST_CASE("Powertrain dyno E1: motor envelope, pack budget and fuel-cell supply against hand values", "[drivetrain][electric][dyno]") {
    const ElectricMotorDesc m = test_motor();
    const real unlimited = 1e9;
    SECTION("peak torque below the corner") {
        const MotorFullLoadPoint p = motor_full_load_point(m, 1000, unlimited, kH);
        CHECK(p.limit == MotorLimit::PeakTorque);
        CHECK(p.torque_nm == 600);
        CHECK_THAT(p.power_w, WithinRel(600 * rad_s(1000), 1e-12));
        CHECK_THAT(p.electrical_w, WithinRel(600 * rad_s(1000) / 0.95, 1e-12));
    }
    SECTION("rated-power corner with the rotor-inertia projection") {
        const real w = rad_s(9000);
        const real projected = w + kH * 600 / 0.05; // the substep's predicted speed at peak torque
        const MotorFullLoadPoint p = motor_full_load_point(m, 9000, unlimited, kH);
        CHECK(p.limit == MotorLimit::RatedPower);
        CHECK_THAT(p.torque_nm, WithinRel(300000 / projected, 1e-12));
        CHECK_THAT(p.power_w, WithinRel(300000 * w / projected, 1e-12)); // just under the rating: the corner uses the projected speed
    }
    SECTION("battery budget") {
        const real w = rad_s(3000);
        const real projected = w + kH * 600 / 0.05;
        // 50 kW at the pack: torque = budget * eff / projected (the request is cut in the grant's ratio).
        const MotorFullLoadPoint p = motor_full_load_point(m, 3000, 50000, kH);
        CHECK(p.limit == MotorLimit::BatteryDischarge);
        CHECK_THAT(p.torque_nm, WithinRel(50000 * 0.95 / projected, 1e-12));
        const MotorFullLoadPoint e = motor_full_load_point(m, 3000, 0, kH);
        CHECK(e.limit == MotorLimit::Empty);
        CHECK(e.torque_nm == 0);
    }
    SECTION("max rpm") {
        const MotorFullLoadPoint p = motor_full_load_point(m, 16000, unlimited, kH);
        CHECK(p.limit == MotorLimit::MaxRpm);
        CHECK(p.torque_nm == 0);
    }
    SECTION("pack discharge budget") {
        BatteryDesc b;
        b.capacity_kwh = 90;
        b.max_discharge_w = 700000;
        CHECK_THAT(battery_discharge_budget_w(b, 0.5, kH), WithinRel(700000.0, 1e-12));
        b.capacity_kwh = 1;
        // 1e-4 of 1 kWh = 360 J < 700 kW * h = 729 J: the content is the budget.
        CHECK_THAT(battery_discharge_budget_w(b, 1e-4, kH), WithinRel(360.0 * 960, 1e-9));
        CHECK(battery_discharge_budget_w(b, 0, kH) == 0);
    }
    SECTION("fuel-cell supply") {
        FuelCellDesc f;
        f.max_power_w = 150000;
        f.target_soc = 0.6;
        BatteryDesc b;
        b.capacity_kwh = 20;
        b.max_charge_w = 200000;
        CHECK_THAT(fuel_cell_supply_w(f, b, 0.4, 5, kH), WithinRel(150000.0, 1e-9));            // ramp fully open
        CHECK_THAT(fuel_cell_supply_w(f, b, 0.58, 5, kH), WithinRel(150000.0 * 0.4, 1e-9));     // (0.6 - 0.58) / 0.05
        CHECK(fuel_cell_supply_w(f, b, 0.6, 5, kH) == 0);                                        // at the target
        CHECK(fuel_cell_supply_w(f, b, 0.4, 0, kH) == 0);                                        // no hydrogen
        b.max_charge_w = 50000;
        CHECK_THAT(fuel_cell_supply_w(f, b, 0.4, 5, kH), WithinRel(50000.0, 1e-9));             // charge acceptance
        // 1 ug of hydrogen = 0.066 J per substep at most, below 150 kW * h = 156 J: the fuel is the limit.
        b.max_charge_w = 200000;
        CHECK_THAT(fuel_cell_supply_w(f, b, 0.4, 1e-9, kH), WithinRel(1e-9 * 120e6 * 0.55 * 960, 1e-6)); // 0.066 J on a 29 MJ pack: rounding
    }
}

TEST_CASE("Powertrain dyno S1: hybrid full-pedal split per drive mode against hand values", "[drivetrain][electric][dyno][hybrid]") {
    HybridControlDesc c;
    c.requested_power_w = 250000;
    c.low_soc = 0.2;
    c.charge_target_soc = 0.8;
    const real avail = 150000;
    struct Case {
        int mode;
        real pedal, soc;
        real electric, engine;
        bool coupled, wanted, latched;
    };
    // requested = pedal * 250 kW; electric = min(1, requested / 150 kW); engine (when coupled) = (requested - 150) / 100 kW.
    const Case cases[] = {
        {0, 1.0, 0.5, 1.0, 0.0, false, false, false},           // Eco: never coupled, no start at a healthy SOC
        {1, 1.0, 0.5, 1.0, 1.0, true, true, false},             // Normal: 250 > 150 -> extra power, engine full
        {1, 0.5, 0.5, 125.0 / 150, 0.0, false, false, false},   // Normal below the electric potential
        {1, 0.8, 0.5, 1.0, 0.5, true, true, false},             // Normal: 200 kW -> engine (200-150)/100
        {2, 0.5, 0.5, 125.0 / 150, 0.0, true, true, false},     // Sport: coupled and running, no deficit
        {2, 0.8, 0.5, 1.0, 0.5, true, true, false},
        {3, 1.0, 0.5, 1.0, 1.0, true, true, false},             // Racing = Sport
        {0, 1.0, 0.19, 1.0, 0.0, false, true, true},            // Eco at low SOC: the charge latch starts the engine, decoupled
    };
    for (const Case& k : cases) {
        INFO("mode " << k.mode << " pedal " << k.pedal << " soc " << k.soc);
        const HybridSplit s = hybrid_full_pedal_split(c, avail, k.pedal, k.mode, k.soc);
        CHECK_THAT(s.requested_w, WithinRel(k.pedal * 250000, 1e-12));
        CHECK_THAT(s.electric_command, WithinAbs(k.electric, 1e-12));
        CHECK_THAT(s.engine_command, WithinAbs(k.engine, 1e-12));
        CHECK(s.engine_coupled == k.coupled);
        CHECK(s.engine_wanted == k.wanted);
        CHECK(s.charge_latched == k.latched);
    }
    // The latch holds above low_soc until charge_target_soc.
    CHECK(hybrid_update_charge_latch(c, 0.5, true));
    CHECK_FALSE(hybrid_update_charge_latch(c, 0.8, true));
    CHECK_FALSE(hybrid_update_charge_latch(c, 0.5, false));
}

TEST_CASE("Powertrain dyno A1: a short pack budget is shared in the claims' ratio", "[drivetrain][electric][dyno]") {
    BatteryDesc d;
    d.capacity_kwh = 90;
    d.initial_soc = 0.9;
    d.max_discharge_w = 960 * 2.0; // 2 J per substep
    Battery b(d);
    SECTION("limited: proportional, in either order") {
        b.begin_substep(kH);
        b.add_drive_claim(3);
        b.add_drive_claim(1);
        CHECK_THAT(b.reserve_discharge(3), WithinRel(1.5, 1e-12));
        CHECK_THAT(b.reserve_discharge(1), WithinRel(0.5, 1e-12));
        b.begin_substep(kH);
        b.add_drive_claim(1);
        b.add_drive_claim(3);
        CHECK_THAT(b.reserve_discharge(1), WithinRel(0.5, 1e-12));
        CHECK_THAT(b.reserve_discharge(3), WithinRel(1.5, 1e-12));
    }
    SECTION("not limited, or no claims: min(left, request) as before") {
        b.begin_substep(kH);
        b.add_drive_claim(1);
        b.add_drive_claim(0.5);
        CHECK(b.reserve_discharge(1) == 1);
        CHECK(b.reserve_discharge(0.5) == 0.5);
        b.begin_substep(kH);
        CHECK(b.reserve_discharge(3) == 2);
        CHECK(b.reserve_discharge(1) == 0);
    }
}

TEST_CASE("Powertrain dyno P1: battery EV on the motor axis", "[drivetrain][electric][dyno][ev]") {
    const auto d = load("car_electric.json");
    const ElectricMotorDesc& m = by_id<ElectricMotorDesc>(d, "front_motor");
    const real eta_diff = by_id<DifferentialDesc>(d, "front_diff").efficiency;
    PowertrainDynoSpec s;
    s.axis = PowertrainDynoAxis::MotorRpm;
    s.points = {1000, 9000};
    const auto r = run_powertrain_dyno(d, nullptr, s);
    print("car_electric", r);
    REQUIRE(r.points.size() == 2);
    for (const auto& p : r.points) {
        CHECK(p.converged);
        REQUIRE(p.motors.size() == 2);
        CHECK_THAT(p.motors[0].rpm, WithinRel(p.axis_value, 1e-3));
        // Both motors are identical and drive identical axles.
        CHECK_THAT(p.motors[0].torque_nm, WithinRel(p.motors[1].torque_nm, 1e-9));
        // The diff's lagged mesh loss: P_out = P_in / (2 - eta).
        CHECK_THAT(p.system_power_w, WithinRel(p.electric_power_w / (2 - eta_diff), 2e-4));
        CHECK_THAT(p.battery_power_w, WithinRel(p.electric_power_w / m.efficiency, 1e-9));
        CHECK_FALSE(p.battery_limited);
    }
    CHECK(r.points[0].motors[0].limit == MotorLimit::PeakTorque);
    CHECK_THAT(r.points[0].motors[0].torque_nm, WithinRel(m.peak_torque_nm, 1e-12));
    CHECK(r.points[1].motors[0].limit == MotorLimit::RatedPower);
    {
        const real w = rad_s(r.points[1].motors[0].rpm);
        CHECK_THAT(r.points[1].motors[0].torque_nm, WithinRel(m.max_power_w / (w + kH * m.peak_torque_nm / m.inertia_kgm2), 1e-4));
    }
    CHECK_THAT(r.points[0].system_torque_nm, WithinRel(2 * m.peak_torque_nm / (2 - eta_diff), 1e-3));

    SECTION("100 kW pack: both motors share it") {
        auto e = d;
        set_pack(e, 100000);
        PowertrainDynoSpec t = s;
        t.points = {3000, 9000};
        const auto q = run_powertrain_dyno(e, nullptr, t);
        print("car_electric 100 kW pack", q);
        for (const auto& p : q.points) {
            CHECK(p.battery_limited);
            CHECK(p.motors[0].limit == MotorLimit::BatteryDischarge);
            CHECK(p.motors[1].limit == MotorLimit::BatteryDischarge);
            CHECK_THAT(p.motors[0].power_w, WithinRel(p.motors[1].power_w, 1e-6));
            // The grant is 100 kW at the projected speed; the booked draw at the solved speed is just below it.
            CHECK(p.battery_power_w <= 100000);
            CHECK(p.battery_power_w > 99000);
        }
    }
}

TEST_CASE("Powertrain dyno P2: fuel-cell EV", "[drivetrain][electric][dyno][fuel_cell]") {
    const auto d = load("car_fuel_cell.json");
    const FuelCellDesc& f = first<FuelCellDesc>(d);
    const BatteryDesc& b = first<BatteryDesc>(d);
    PowertrainDynoSpec s;
    s.points = {3000, 9000};
    const auto r = run_powertrain_dyno(d, nullptr, s);
    print("car_fuel_cell", r);
    for (const auto& p : r.points) {
        CHECK(p.converged);
        CHECK_THAT(p.fuel_cell_power_w, WithinRel(fuel_cell_supply_w(f, b, b.initial_soc, f.initial_hydrogen_kg, kH), 1e-9));
        CHECK_THAT(p.fuel_cell_power_w, WithinRel(f.max_power_w, 1e-9)); // SOC 0.4 << target 0.8: the stack at its rating
        // The packs' net draw = the motors' electrical power minus the stack.
        const real motors_el = p.motors[0].electrical_w + p.motors[1].electrical_w;
        CHECK_THAT(p.battery_power_w, WithinAbs(motors_el - p.fuel_cell_power_w, 1e-6 * motors_el));
    }
    SECTION("10 kW pack + 30 kW stack: the motors share 40 kW") {
        auto e = d;
        set_pack(e, 10000);
        for (auto& c : e.powertrain.components)
            if (auto* fc = std::get_if<FuelCellDesc>(&c.params)) fc->max_power_w = 30000;
        const auto q = run_powertrain_dyno(e, nullptr, s);
        print("car_fuel_cell 10 kW pack + 30 kW stack", q);
        for (const auto& p : q.points) {
            CHECK(p.battery_limited);
            CHECK_THAT(p.fuel_cell_power_w, WithinRel(30000.0, 1e-9));
            const real motors_el = p.motors[0].electrical_w + p.motors[1].electrical_w;
            CHECK(motors_el <= 40000);
            CHECK(motors_el > 39500);
            CHECK_THAT(p.motors[0].electrical_w, WithinRel(p.motors[1].electrical_w, 1e-6));
        }
    }
    SECTION("hydrogen_fraction 0: battery only") {
        PowertrainDynoSpec t = s;
        t.hydrogen_fraction = 0;
        const auto q = run_powertrain_dyno(d, nullptr, t);
        for (const auto& p : q.points) CHECK(p.fuel_cell_power_w == 0);
    }
}

TEST_CASE("Powertrain dyno P3: parallel hybrid on the engine axis", "[drivetrain][electric][dyno][hybrid]") {
    const auto d = load("car_hybrid.json");
    const TorqueMapEngineDesc& eng = first<TorqueMapEngineDesc>(d);
    const real eta_red = by_id<GearPairDesc>(d, "motor_reduction").efficiency;
    const real eta_diff = first<DifferentialDesc>(d).efficiency;
    const real eta_g3 = first<GearboxDesc>(d).forward_efficiency[2];
    const BatteryDesc& b = first<BatteryDesc>(d);
    PowertrainDynoSpec s;
    s.axis = PowertrainDynoAxis::EngineRpm;
    s.gear = 3;
    s.points = {1500, 3000, 6000};
    SECTION("Normal: engine and motor together") {
        s.drive_mode = 1;
        const auto r = run_powertrain_dyno(d, nullptr, s);
        print("car_hybrid Normal 3rd", r);
        for (const auto& p : r.points) {
            INFO("rpm " << p.axis_value);
            CHECK(p.converged);
            CHECK(p.engine_running);
            CHECK(p.gear == 3);
            CHECK(p.hybrid_engine_coupled);
            CHECK_THAT(p.hybrid_requested_w, WithinRel(250000.0, 1e-12));
            CHECK_THAT(p.hybrid_available_w, WithinRel(150000.0, 1e-12)); // min(motor 200 kW, pack 150 kW)
            CHECK_THAT(p.engine_rpm, WithinRel(p.axis_value, 1e-4));
            CHECK_THAT(p.engine_command, WithinAbs(1.0, 1e-12));
            CHECK_THAT(p.engine_torque_nm, WithinRel(eng.wot_torque_nm_vs_rpm.evaluate(p.engine_rpm), 1e-6));
            // Motor at gearbox.out through the reduction; both through the diff; each mesh P_out = P_in / (2 - eta).
            const real motor_w = p.motors[0].power_w;
            const real expect = (p.engine_power_w / (2 - eta_g3) + motor_w / (2 - eta_red)) / (2 - eta_diff);
            CHECK_THAT(p.system_power_w, WithinRel(expect, 2e-3));
            CHECK(p.battery_power_w <= b.max_discharge_w);
        }
        CHECK(r.points[0].motors[0].limit == MotorLimit::PeakTorque);
        CHECK(r.points[2].battery_limited);
        CHECK(r.points[2].battery_power_w > 0.99 * b.max_discharge_w);
    }
    SECTION("Eco: motor only, engine off, gearbox in neutral") {
        s.drive_mode = 0;
        const auto r = run_powertrain_dyno(d, nullptr, s);
        print("car_hybrid Eco 3rd", r);
        for (const auto& p : r.points) {
            CHECK_FALSE(p.engine_running);
            CHECK(p.gear == 0);
            CHECK(p.engine_torque_nm == 0);
            CHECK_THAT(p.system_power_w, WithinRel(p.motors[0].power_w / ((2 - eta_red) * (2 - eta_diff)), 2e-3));
        }
    }
}

TEST_CASE("Powertrain dyno P4: an ICE-only car equals run_dyno_sweep", "[drivetrain][dyno]") {
    for (const char* name : {"car_sedan.json", "car_sedan_sim.json"}) {
        INFO(name);
        const auto d = load(name);
        PowertrainDynoSpec s;
        s.axis = PowertrainDynoAxis::EngineRpm;
        s.gear = 3;
        s.points = {1500, 3000, 4500, 6000};
        const auto r = run_powertrain_dyno(d, nullptr, s);
        print(name, r);
        DynoSweepSpec e;
        for (const auto& p : r.points) e.rpm.push_back(p.engine_rpm);
        const ComponentParams* engine = nullptr;
        for (const auto& c : d.powertrain.components)
            if (std::holds_alternative<TorqueMapEngineDesc>(c.params) || std::holds_alternative<SimulatedEngineDesc>(c.params)) engine = &c.params;
        REQUIRE(engine != nullptr);
        const DynoResult ref = run_dyno_sweep(*engine, nullptr, e);
        for (std::size_t i = 0; i < r.points.size(); ++i) {
            INFO("rpm " << r.points[i].engine_rpm << " dyno " << r.points[i].engine_torque_nm << " sweep " << ref.points[i].torque_nm);
            std::printf("  %s %.0f rpm: powertrain dyno %.6f N*m, run_dyno_sweep %.6f N*m (%+.2e)\n", name, r.points[i].engine_rpm,
                        r.points[i].engine_torque_nm, ref.points[i].torque_nm, r.points[i].engine_torque_nm / ref.points[i].torque_nm - 1);
            CHECK(r.points[i].converged);
            CHECK_THAT(r.points[i].engine_rpm, WithinRel(s.points[i], 1e-4));
            // Measured 2026-10-19: equal to 1e-14 for both (the torque map is instantaneous; the NA simulated engine's stepped
            // runtime settles onto its steady solve at the held speed).
            CHECK_THAT(r.points[i].engine_torque_nm, WithinRel(ref.points[i].torque_nm, 1e-12));
        }
    }
}

TEST_CASE("Powertrain dyno P4b: the turbo hypercar's crank torque equals run_dyno_sweep (Sketch maps)", "[drivetrain][dyno][turbo][long]") {
    jobs::JobSystem pool(10);
    io::VehicleLoadOptions o;
    o.maps.quality = MapQuality::Sketch;
    o.maps.pool = &pool;
    o.maps.cache_dir = (std::filesystem::temp_directory_path() / "ps_powertrain_dyno_maps").string();
    std::filesystem::create_directories(o.maps.cache_dir);
    const auto d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", o);
    PowertrainDynoSpec s;
    s.axis = PowertrainDynoAxis::EngineRpm;
    s.gear = 4;
    s.points = {3000, 5000, 7000};
    const auto r = run_powertrain_dyno(d, nullptr, s, &pool);
    print("car_hyper 4th", r);
    DynoSweepSpec e;
    for (const auto& p : r.points) e.rpm.push_back(p.engine_rpm);
    const ComponentParams& engine = [&]() -> const ComponentParams& {
        for (const auto& c : d.powertrain.components)
            if (std::holds_alternative<SimulatedEngineDesc>(c.params)) return c.params;
        throw std::runtime_error("no engine");
    }();
    const DynoResult ref = run_dyno_sweep(engine, nullptr, e, &pool);
    for (std::size_t i = 0; i < r.points.size(); ++i) {
        std::printf("  car_hyper %.0f rpm: powertrain dyno %.3f N*m %.3f bar, run_dyno_sweep %.3f N*m %.3f bar (%+.2e), settle %.1f / conv %d\n",
                    r.points[i].engine_rpm, r.points[i].engine_torque_nm, r.points[i].boost_bar, ref.points[i].torque_nm, ref.points[i].boost_bar,
                    r.points[i].engine_torque_nm / ref.points[i].torque_nm - 1, r.points[i].settle_s, r.points[i].converged ? 1 : 0);
        CHECK(r.points[i].converged);
        CHECK_THAT(r.points[i].engine_torque_nm, WithinRel(ref.points[i].torque_nm, 1e-5)); // measured <= 4e-7 (2026-10-19)
    }
}

TEST_CASE("Powertrain dyno P5: 1 vs 10 workers vs serial are bit-identical", "[drivetrain][electric][dyno][determinism]") {
    jobs::JobSystem one(1), ten(10);
    struct Run {
        const char* file;
        PowertrainDynoAxis axis;
        int gear;
        std::vector<real> points;
    };
    const Run runs[] = {{"car_electric.json", PowertrainDynoAxis::MotorRpm, 1, {1000, 4000, 8000, 12000}},
                        {"car_hybrid.json", PowertrainDynoAxis::EngineRpm, 2, {1500, 2500, 3500, 4500, 5500}},
                        {"car_fuel_cell.json", PowertrainDynoAxis::RoadSpeed, 1, {5, 20, 40}}};
    for (const Run& run : runs) {
        INFO(run.file);
        const auto d = load(run.file);
        PowertrainDynoSpec s;
        s.axis = run.axis;
        s.gear = run.gear;
        s.points = run.points;
        const auto serial = run_powertrain_dyno(d, nullptr, s);
        CHECK(same(serial, run_powertrain_dyno(d, nullptr, s, &one)));
        CHECK(same(serial, run_powertrain_dyno(d, nullptr, s, &ten)));
    }
}

TEST_CASE("Powertrain dyno P6: cancel and rejected specs", "[drivetrain][electric][dyno]") {
    const auto ev = load("car_electric.json");
    PowertrainDynoSpec s;
    s.points = {1000};
    std::atomic<bool> cancel{true};
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, s, nullptr, &cancel), GenerationCancelled);
    jobs::JobSystem pool(4);
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, s, &pool, &cancel), GenerationCancelled);

    PowertrainDynoSpec bad = s;
    bad.points = {};
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, bad), std::invalid_argument);
    bad = s;
    bad.points = {-5};
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, bad), std::invalid_argument);
    bad = s;
    bad.axis = PowertrainDynoAxis::EngineRpm; // no gearbox on the EV
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, bad), std::invalid_argument);
    bad = s;
    bad.motor_index = 2;
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, bad), std::invalid_argument);
    bad = s;
    bad.battery_soc = 1.5;
    CHECK_THROWS_AS(run_powertrain_dyno(ev, nullptr, bad), std::invalid_argument);

    PowertrainDynoSpec a;
    a.axis = PowertrainDynoAxis::RoadSpeed;
    a.points = {10};
    // (automatic, DCT and CVT gearboxes are measured since 2026-10-21: A1-A6)
    const auto sedan = load("car_sedan.json");
    a.gear = 9;
    CHECK_THROWS_AS(run_powertrain_dyno(sedan, nullptr, a), std::invalid_argument);
    a.gear = 0;
    a.axis = PowertrainDynoAxis::EngineRpm;
    a.points = {2000};
    CHECK_THROWS_AS(run_powertrain_dyno(sedan, nullptr, a), std::invalid_argument);
}

TEST_CASE("Powertrain dyno cost", "[.][diag][dyno]") {
    jobs::JobSystem ten(10);
    const struct {
        const char* file;
        PowertrainDynoAxis axis;
        int gear, mode;
    } runs[] = {{"car_electric.json", PowertrainDynoAxis::MotorRpm, 1, 1},
                {"car_fuel_cell.json", PowertrainDynoAxis::MotorRpm, 1, 1},
                {"car_hybrid.json", PowertrainDynoAxis::EngineRpm, 3, 1},
                {"car_hybrid.json", PowertrainDynoAxis::EngineRpm, 3, 0},
                {"car_sedan.json", PowertrainDynoAxis::EngineRpm, 3, 1}};
    for (const auto& run : runs) {
        const auto d = load(run.file);
        PowertrainDynoSpec s;
        s.axis = run.axis;
        s.gear = run.gear;
        s.drive_mode = run.mode;
        for (real x = 1000; x <= (run.axis == PowertrainDynoAxis::MotorRpm ? 15000 : 6500); x += 500) s.points.push_back(x);
        for (int pass = 0; pass < 2; ++pass) {
            const auto t0 = std::chrono::steady_clock::now();
            const auto r = run_powertrain_dyno(d, nullptr, s, pass == 0 ? nullptr : &ten);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            std::printf("%-20s mode %d %zu points %s: %.1f ms\n", run.file, run.mode, s.points.size(), pass == 0 ? "serial" : "10 workers", ms);
            if (pass == 0) print(run.file, r);
        }
    }
}

// ---- Automatic, dual-clutch and CVT gearboxes (2026-10-21): the dyno holds one gear / one variator ratio ----------------
// A1 DualClutch ICE (car_sedan_dct): gears 2 and 4 at the same crank rpm - the crank turns at the axis value (starting clutch
//    locked), the gearbox output at axis / ratio, the held gear is the reported one, the net crank torque is the WOT curve and
//    the wheel torque ratio is the ratio of the gear ratios times the mesh-loss factor (2 - eta).
// A2 hybrid DualClutch (car_hybrid_dct): with an empty pack it is the ICE car (wheel torque equal to A1's car to 1e-9); with
//    the pack, engine and motor together obey the P3 power balance in both gears.
// A3 Automatic (car_sedan_at, car_hybrid_at): the converter is locked in the held gear (crank = turbine speed), gear held.
// A4 CVT (car_sedan_cvt, car_hybrid_cvt): `gear` k holds the variator at virtual step k's ratio, `cvt_ratio` at an explicit
//    ratio; out of range / on a non-CVT is rejected; gear 1 is the reported position.
// A5 manual unchanged: car_sedan and car_hybrid (6MT) 3rd gear equal the numbers recorded before this change, bit for bit.
// A6 1 vs 10 workers vs serial bit-identical on the DCT hybrid; gear 0 on an automatic box = neutral.
// (Fixtures tests/support/fixtures/vehicles/car_hybrid_{dct,at,cvt}.json: car_hybrid.json with the clutch component dropped,
//  the sedan_7dct / sedan_8at / sedan_cvt gearbox and the automatic_tcu of car_sedan_{dct,at,cvt}.json. No EV fixture: the
//  automatic_tcu controller needs an engine, so a battery EV cannot have an automatic box in this model.)
//
// Sabotages (each reverted afterwards; failures measured on this branch are listed in the commit message / report):
//  S-a AutoTransmission::choose_target's hold branch removed -> the TCU schedule picks the gear: A1/A2/A3 fail (gear != held,
//      gearbox output rpm off by the ratio ratio);
//  S-b update_launch's hold branch removed -> the DCT starting clutch creeps/slips instead of being locked: A1 fails;
//  S-c update_lockup's hold branch removed -> the AT converter slips below the lock-up engage rpm: A3 fails (crank != turbine);
//  S-d update_cvt's hold branch removed -> the variator follows its ratio law: A4 fails (output rpm off the held ratio).
namespace {

const GearboxDesc& gearbox_of(const vehicle::VehicleDesc& d) { return first<GearboxDesc>(d); }

vehicle::VehicleDesc fixture(const char* name) {
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/" + name);
}

PowertrainDynoSpec engine_axis(int gear, std::vector<real> rpm) {
    PowertrainDynoSpec s;
    s.axis = PowertrainDynoAxis::EngineRpm;
    s.gear = gear;
    s.points = std::move(rpm);
    return s;
}

// Wheel torque ratio of two gears at equal crank speed: ratio of gear ratios x the lagged mesh-loss factor.
real expected_torque_ratio(const GearboxDesc& g, int a, int b) {
    const real ra = g.forward_ratios[static_cast<std::size_t>(a - 1)], rb = g.forward_ratios[static_cast<std::size_t>(b - 1)];
    const real ea = g.forward_efficiency[static_cast<std::size_t>(a - 1)], eb = g.forward_efficiency[static_cast<std::size_t>(b - 1)];
    return (ra / rb) * ((2 - eb) / (2 - ea));
}

} // namespace

TEST_CASE("Powertrain dyno A1: dual-clutch ICE holds the requested gear", "[drivetrain][dyno][auto_gearbox]") {
    const auto d = fixture("car_sedan_dct.json");
    const GearboxDesc& g = gearbox_of(d);
    const TorqueMapEngineDesc& eng = first<TorqueMapEngineDesc>(d);
    // 900 rpm is below the launch law's bite rpm (1000): only the held (locked) starting clutch carries the engine there.
    const auto lo = run_powertrain_dyno(d, nullptr, engine_axis(2, {900, 2500, 4000}));
    const auto hi = run_powertrain_dyno(d, nullptr, engine_axis(4, {900, 2500, 4000}));
    for (int pass = 0; pass < 2; ++pass) {
        const int gear = pass == 0 ? 2 : 4;
        const auto& r = pass == 0 ? lo : hi;
        for (const auto& p : r.points) {
            INFO("gear " << gear << " rpm " << p.axis_value);
            CHECK(p.converged);
            CHECK(p.gear == gear);
            CHECK(p.engine_running);
            CHECK_THAT(p.engine_rpm, WithinRel(p.axis_value, 1e-4)); // starting clutch locked: the crank is the axis
            CHECK_THAT(p.gearbox_output_rpm, WithinRel(p.axis_value / g.forward_ratios[static_cast<std::size_t>(gear - 1)], 1e-3));
            CHECK_THAT(p.engine_torque_nm, WithinRel(eng.wot_torque_nm_vs_rpm.evaluate(p.engine_rpm), 1e-4));
        }
    }
    for (std::size_t i = 0; i < lo.points.size(); ++i) {
        const real ratio = lo.points[i].system_wheel_torque_nm / hi.points[i].system_wheel_torque_nm;
        INFO("wheel torque ratio " << ratio);
        CHECK_THAT(ratio, WithinRel(expected_torque_ratio(g, 2, 4), 0.01));
    }
}

TEST_CASE("Powertrain dyno A2: hybrid dual-clutch car, empty pack = the ICE car, full pack = P3 balance", "[drivetrain][electric][dyno][hybrid][auto_gearbox]") {
    const auto ice = fixture("car_sedan_dct.json");
    const auto d = fixture("car_hybrid_dct.json");
    const GearboxDesc& g = gearbox_of(d);
    SECTION("pack empty: wheel torque equals the ICE car's") {
        PowertrainDynoSpec s = engine_axis(2, {2500, 4000});
        s.battery_soc = 0.0;
        for (int gear : {2, 4}) {
            s.gear = gear;
            const auto h = run_powertrain_dyno(d, nullptr, s);
            const auto c = run_powertrain_dyno(ice, nullptr, s);
            for (std::size_t i = 0; i < h.points.size(); ++i) {
                INFO("gear " << gear << " rpm " << s.points[i]);
                CHECK(h.points[i].gear == gear);
                CHECK(h.points[i].motors[0].torque_nm == 0);
                CHECK_THAT(h.points[i].system_wheel_torque_nm, WithinRel(c.points[i].system_wheel_torque_nm, 1e-9));
            }
        }
    }
    SECTION("full pack: engine + motor, both gears") {
        const BatteryDesc& b = first<BatteryDesc>(d);
        const real eta_red = by_id<GearPairDesc>(d, "motor_reduction").efficiency;
        const real eta_diff = first<DifferentialDesc>(d).efficiency;
        PowertrainDynoSpec s = engine_axis(2, {3000});
        s.drive_mode = 1;
        PowertrainDynoPoint pt[2];
        const int gears[2] = {2, 4};
        for (int k = 0; k < 2; ++k) {
            s.gear = gears[k];
            const auto r = run_powertrain_dyno(d, nullptr, s);
            pt[k] = r.points[0];
            const auto& p = pt[k];
            INFO("gear " << gears[k]);
            CHECK(p.converged);
            CHECK(p.gear == gears[k]);
            CHECK(p.hybrid_engine_coupled);
            CHECK_THAT(p.engine_rpm, WithinRel(3000.0, 1e-4));
            CHECK_THAT(p.gearbox_output_rpm, WithinRel(3000.0 / g.forward_ratios[static_cast<std::size_t>(gears[k] - 1)], 1e-3));
            const real eta_g = g.forward_efficiency[static_cast<std::size_t>(gears[k] - 1)];
            const real expect = (p.engine_power_w / (2 - eta_g) + p.motors[0].power_w / (2 - eta_red)) / (2 - eta_diff);
            CHECK_THAT(p.system_power_w, WithinRel(expect, 2e-3));
            CHECK(p.battery_power_w <= b.max_discharge_w);
        }
        // The same crank speed in the lower gear turns the wheels slower.
        CHECK_THAT(pt[0].road_speed_mps / pt[1].road_speed_mps, WithinRel(g.forward_ratios[3] / g.forward_ratios[1], 1e-3));
    }
}

TEST_CASE("Powertrain dyno A2b: hybrid dual-clutch, low crank rpm in a high gear is not rev-matched", "[drivetrain][electric][dyno][hybrid][auto_gearbox]") {
    // The hybrid supervisor's ICE-connect (rev-match: clutch held open while the engine revs toward the SCHEDULE's fit gear's
    // input speed) must not run in a dyno hold: at a low crank rpm in 5th the fit gear is 1st / 2nd, whose input speed is far
    // above the held gear's, so the open clutch left the crank at 2-3x the axis (found in racing_game's garage dyno).
    // Sabotage: the dyno-hold clause removed from `hybrid_connect_s_ = 0` in Powertrain::pre_solve_components -> engine_rpm
    // far above the axis on the first points (measured in the commit message).
    const auto d = fixture("car_hybrid_dct.json");
    const GearboxDesc& g = gearbox_of(d);
    PowertrainDynoSpec s = engine_axis(5, {1100, 1500, 2000});
    s.drive_mode = 1;
    const auto r = run_powertrain_dyno(d, nullptr, s);
    for (const auto& p : r.points) {
        INFO("rpm " << p.axis_value);
        CHECK(p.converged);
        CHECK(p.gear == 5);
        CHECK(p.hybrid_engine_coupled);
        CHECK_THAT(p.engine_rpm, WithinRel(p.axis_value, 1e-3));
        CHECK_THAT(p.gearbox_output_rpm, WithinRel(p.axis_value / g.forward_ratios[4], 1e-3));
    }
}

TEST_CASE("Powertrain dyno A3: automatic gearbox holds the gear and locks the converter", "[drivetrain][electric][dyno][hybrid][auto_gearbox]") {
    for (const char* name : {"car_sedan_at.json", "car_hybrid_at.json"}) {
        INFO(name);
        const auto d = fixture(name);
        const GearboxDesc& g = gearbox_of(d);
        PowertrainDynoSpec s = engine_axis(2, {2500, 4000});
        s.battery_soc = 0.0; // hybrid: the ICE alone (the motor is covered by A2)
        PowertrainDynoResult r[2];
        const int gears[2] = {2, 4};
        for (int k = 0; k < 2; ++k) {
            s.gear = gears[k];
            r[k] = run_powertrain_dyno(d, nullptr, s);
            for (const auto& p : r[k].points) {
                INFO("gear " << gears[k] << " rpm " << p.axis_value);
                CHECK(p.converged);
                CHECK(p.gear == gears[k]);
                CHECK_THAT(p.engine_rpm, WithinRel(p.axis_value, 1e-4)); // locked: crank (pump) = turbine = axis
                CHECK_THAT(p.gearbox_output_rpm, WithinRel(p.axis_value / g.forward_ratios[static_cast<std::size_t>(gears[k] - 1)], 1e-3));
            }
        }
        for (std::size_t i = 0; i < r[0].points.size(); ++i)
            CHECK_THAT(r[0].points[i].system_wheel_torque_nm / r[1].points[i].system_wheel_torque_nm, WithinRel(expected_torque_ratio(g, 2, 4), 0.01));
    }
}

TEST_CASE("Powertrain dyno A4: CVT holds a variator ratio", "[drivetrain][electric][dyno][hybrid][auto_gearbox]") {
    for (const char* name : {"car_sedan_cvt.json", "car_hybrid_cvt.json"}) {
        INFO(name);
        const auto d = fixture(name);
        const GearboxDesc& g = gearbox_of(d);
        REQUIRE(g.cvt.has_value());
        PowertrainDynoSpec s = engine_axis(2, {2500, 4000});
        s.battery_soc = 0.0;
        PowertrainDynoResult r[2];
        const int steps[2] = {2, 4};
        for (int k = 0; k < 2; ++k) {
            s.gear = steps[k];
            r[k] = run_powertrain_dyno(d, nullptr, s);
            for (const auto& p : r[k].points) {
                INFO("step " << steps[k] << " rpm " << p.axis_value);
                CHECK(p.converged);
                CHECK(p.gear == 1); // a CVT has one forward position, D
                CHECK_THAT(p.engine_rpm, WithinRel(p.axis_value, 1e-4));
                CHECK_THAT(p.gearbox_output_rpm, WithinRel(p.axis_value / g.forward_ratios[static_cast<std::size_t>(steps[k] - 1)], 1e-3));
            }
        }
        for (std::size_t i = 0; i < r[0].points.size(); ++i)
            CHECK_THAT(r[0].points[i].system_wheel_torque_nm / r[1].points[i].system_wheel_torque_nm, WithinRel(expected_torque_ratio(g, 2, 4), 0.02));
        // An explicit variator ratio overrides the step.
        s.gear = 1;
        s.cvt_ratio = 1.5;
        const auto e = run_powertrain_dyno(d, nullptr, s);
        for (const auto& p : e.points) {
            CHECK(p.converged);
            CHECK_THAT(p.gearbox_output_rpm, WithinRel(p.axis_value / 1.5, 1e-3));
        }
        s.cvt_ratio = g.cvt->ratio_low * 1.5; // outside the variator's range
        CHECK_THROWS_AS(run_powertrain_dyno(d, nullptr, s), std::invalid_argument);
        s.cvt_ratio = g.cvt->ratio_high * 0.5;
        CHECK_THROWS_AS(run_powertrain_dyno(d, nullptr, s), std::invalid_argument);
    }
    // cvt_ratio on a gearbox that is not a CVT.
    PowertrainDynoSpec s = engine_axis(2, {2500});
    s.cvt_ratio = 1.5;
    CHECK_THROWS_AS(run_powertrain_dyno(fixture("car_sedan_dct.json"), nullptr, s), std::invalid_argument);
    CHECK_THROWS_AS(run_powertrain_dyno(load("car_sedan.json"), nullptr, s), std::invalid_argument);
}

TEST_CASE("Powertrain dyno A5: manual gearboxes are unchanged (bit-identical to before the auto-gearbox hold)", "[drivetrain][dyno][auto_gearbox]") {
    // Recorded on a757c5c (before this change), 3rd gear on the engine axis, %.17g.
    struct Pin {
        real system_power_w, wheel_torque_nm, engine_torque_nm, output_rpm;
    };
    const Pin sedan[] = {{24385.9297223332, 882.40218862623362, 164.0004789520406, 1071.4456768585949},
                         {89215.658062189992, 1076.0964588172387, 199.99988318284039, 3214.306574493161}};
    const Pin hybrid[] = {{130082.54249088862, 4706.6968523807745, 164.00255471041592, 1071.519811086252},
                          {224087.54250383817, 2702.8601305971565, 199.99970658722566, 3214.3381094241199}};
    for (const char* car : {"car_sedan.json", "car_hybrid.json"}) {
        const Pin* pins = std::strcmp(car, "car_sedan.json") == 0 ? sedan : hybrid;
        const auto r = run_powertrain_dyno(load(car), nullptr, engine_axis(3, {1500, 4500}));
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& p = r.points[i];
            INFO(car << " " << p.axis_value);
            CHECK(p.system_power_w == pins[i].system_power_w);
            CHECK(p.system_wheel_torque_nm == pins[i].wheel_torque_nm);
            CHECK(p.engine_torque_nm == pins[i].engine_torque_nm);
            CHECK(p.gearbox_output_rpm == pins[i].output_rpm);
        }
    }
}

TEST_CASE("Powertrain dyno A6: auto-gearbox sweeps are worker-count invariant; gear 0 holds neutral", "[drivetrain][electric][dyno][determinism][auto_gearbox]") {
    const auto d = fixture("car_hybrid_dct.json");
    const PowertrainDynoSpec s = engine_axis(3, {1500, 2500, 3500, 4500, 5500});
    const auto serial = run_powertrain_dyno(d, nullptr, s);
    jobs::JobSystem p1(1), p10(10);
    CHECK(same(serial, run_powertrain_dyno(d, nullptr, s, &p1)));
    CHECK(same(serial, run_powertrain_dyno(d, nullptr, s, &p10)));
    // Gear 0 on the road-speed axis: neutral (ICE automatic car, no hybrid supervisor).
    PowertrainDynoSpec n;
    n.axis = PowertrainDynoAxis::RoadSpeed;
    n.gear = 0;
    n.points = {10};
    const auto r = run_powertrain_dyno(fixture("car_sedan_at.json"), nullptr, n);
    CHECK(r.points[0].gear == 0);
}
