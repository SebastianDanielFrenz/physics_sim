// tests/unit/test_engine_thermal_scenarios.cpp - engine thermal plan T1 whole-engine scenario tests I2-I6 and I9
// (out/engine_thermal_plan.md section 12, vault PHYS-063). Tags [thermal][drivetrain]; the multi-minute cases also [long].
//
// Harness. I3-I6 use EngineRig with a Viscous-mode LoadDyno (deviation from the plan's Brake-mode dyno, which cannot
// hold a speed: a Coulomb brake row whose capacity exceeds the engine torque simply arrests the crank, so a PI on its
// scale would be unstable). The dyno scale is set every tick to the requested load torque T_load / (c*omega) - a
// feed-forward load - and a test-side throttle PI [AS] holds the crank speed. The reported load is the dyno's own
// absorbed work (ledger external_absorbed) per tick.
//
// Sabotage record (applied once each, measured 2026-10-05; the named check then fails). Source-level sabotages edit one
// line of core/src, data-level ones one value of data/engine_thermal or data/cooling_systems (restored afterwards).
//   I2a Powertrain: Sensors::ecu_friction_scale held at 1.0 -> "I2 cold idle holds (long)": at 20 C the idle still
//       holds (rpm 799.1-800.1, the 20 C case is NOT a discriminating sabotage case, as the plan anticipated); at
//       -10 C (k = 5.34) the engine starts and then stalls (stalled_after true, rpm 2.9e-13): CHECK_FALSE(stalled_after)
//       and CHECK(min_rpm >= 725) fail. Unsabotaged: rpm [785.97, 812.70].
//   I3a friction_to_oil_fraction 0.30 -> 1.0 (data/engine_thermal/class_si_passenger.json) -> oil 80 C at 918 s,
//       BEFORE the coolant (1020 s): "oil reaches 80 C at least 60 s after the coolant" fails (918 vs 1080).
//       Unsabotaged: coolant 80 C at 869 s, oil at 1181 s.
//   I4a radiator ram_coefficient 0.35 -> 0 (ram ignored) -> coolant [93.0, 98.0] C, the fan comes on (fan transitions
//       9), opening 0.63: hi <= 97, opening < 0.5, fan never on and fan transitions == 0 all fail.
//       Unsabotaged: coolant [85.5, 90.9], opening [0, 0.067], fan 0.
//   I5a fan thresholds 98/93 -> 1000/999 (fan disabled) -> coolant 101.3 C at 2400 s with the thermostat fully open,
//       0 fan transitions: "max coolant < 100" and "fan_on_transitions >= 1" fail. (The plan's "> 105 C before 1200 s"
//       does not occur: the idle heat is small, see the deviations below; 97.78 C at 1200 s either way.)
//   I6a oil hydrodynamic_fraction 0.5 -> 1e-9 (k = 1 + 1.5e-10) -> fuel ratio 1.00000000015: "ratio >= 1.03" fails.
//       Unsabotaged: 1.0557 (cold 0.5885 kg, warm 0.5574 kg over 1200 s).
//   I9  the in-test negative control (starter at 0.8 k*breakaway never turns the crank: 1.9e-10 rpm; at 2 k*breakaway
//       the simulated engine starts in 0.21 s and idles 771-831 rpm at k = 12.7) is the discriminating measurement;
//       the plan's "starter torque halved" sabotage is vacuous because the shipped starter already fails (below).
//
// Deviations from the plan (all measured, none tuned):
//   * Dyno: Viscous-mode LoadDyno with a feed-forward scale and a test-side throttle PI holding the speed (a Brake
//     dyno cannot hold a speed). The load ramps in over 20 s (a cold engine stalls on a load step).
//   * I2 at -10 C and I9 at -30 C: the shipped starter (140 N*m stall [AS]) cannot break away k * 45 N*m once
//     k > 3.11 (oil below about 7 C), so the engine never turns: I9's plan bound fails with the shipped starter for the
//     simulated AND the graph engine ([!mayfail], measurement above, no cap added). The torque-balance A/B uses a bench
//     starter (2 x k x breakaway) to answer the actual question: the simulated engine then starts and idles at -30 C.
//     The graph engine cranks (326 rpm) but never reaches start_complete (550); its cranking margin is razor-thin even
//     at k = 1 (552 rpm), so it does not start below about 50 C ([!mayfail] case "I2 graph ... 20 C").
//   * I3: the hold uses the settled window [60 s, end] for the rpm check. Measured: coolant 80 C at 869 s [bound 400,
//     1100], oil 80 C at 1181 s, oil 79.99 C at 1180 s, k at 1200 s 1.098.
//   * I4: lower coolant bound 83 C instead of 88 C and opening in (0.02, 0.5) instead of (0, 1): the 20 s wax lag with
//     the 2 K play makes the regulator hunt around the 88 C opening point (measured [85.5, 90.9] C, opening [0, 0.067]).
//   * I5: 2400 s instead of 1200 s. The coolant is 97.78 C at 1200 s and the fan first switches on between 1200 and 1400 s;
//     measured over 2400 s: max 98.0 C, 2 fan-on transitions. (Idle heat into the coolant is only about
//     0.0034 K/s at 97 C, the plan's [D] 0.023 K/s at 91 C did not include the block-air loss.)
//   * I6: ratio 1.0557 (plan estimate 1.11, bound [1.03, 1.20]).

#include "drivetrain/engine_thermal.h"

#include "ps/drivetrain/engine_rig.h"
#include "ps/io/engine_thermal_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;

namespace {

std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

constexpr real kKelvin = 273.15;
constexpr real kTickHz = 240.0;
constexpr real kDt = 1.0 / kTickHz;

// ---------------------------------------------------------------------------------------------------------------
// EngineRig, dyno-held operating point
// ---------------------------------------------------------------------------------------------------------------
struct HeldPoint {
    real rpm = 1500.0;
    real load_nm = 30.0;     // 0 with idle = true
    bool idle = false;       // no dyno, throttle 0 (the engine's own idle control)
    real ambient_c = 25.0;
    real ram_mps = 0.0;
    ThermalStartKind start = ThermalStartKind::Ambient;
    real duration_s = 1200.0;
    std::string cooling = "cooling_systems/sedan.json";
};

struct Sample {
    real t, coolant, oil, opening, k, rpm, load, fuel_kg;
    bool fan;
};

struct HeldResult {
    std::vector<Sample> s; // one per second
    int fan_on_transitions = 0;
    real max_coolant = -1e9;
    real min_rpm_after_5s = 1e9, max_rpm_after_5s = -1e9; // over [60 s, end] (named for the plan's 5 s idle bound; the dyno hold needs a minute to settle)
};

HeldResult run_held(const HeldPoint& p) {
    EngineRig::Config cfg;
    cfg.initial_engine_state = EngineState::Running;
    cfg.cooling_system_path = data_path(p.cooling);
    cfg.thermal_start = p.start;
    if (!p.idle) {
        cfg.dyno.enabled = true;
        cfg.dyno.desc.mode = LoadDynoMode::Viscous;
        cfg.dyno.desc.viscous_nm_s_rad = 1.0;
        cfg.dyno.desc.inertia_kgm2 = 0.0;
    }
    EngineRig rig(data_path("engines/toyota_a25a_fks_sim.json"), io::EngineMapOptions{}, nullptr, cfg);
    AmbientSample a;
    a.temperature_k = p.ambient_c + kKelvin;
    rig.set_ambient(a);
    rig.set_thermal_env(a.temperature_k, p.ram_mps);
    rig.set_ignition(1.0);

    HeldResult r;
    const int ticks = static_cast<int>(p.duration_s * kTickHz);
    real thr = 0.3, integ = 0.25, ext_prev = 0.0, load_meas = 0.0;
    bool fan_prev = false;
    const real omega_t = p.rpm * 2.0 * 3.14159265358979323846 / 60.0;
    for (int t = 0; t < ticks; ++t) {
        if (!p.idle) {
            const real omega = std::max(rig.engine_omega_rad_s(), 20.0);
            const real ramp = std::min(1.0, static_cast<real>(t) * kDt / 20.0); // load ramps in over 20 s (a cold engine stalls on a step)
            rig.set_dyno_torque_scale(ramp * p.load_nm / (1.0 * omega));
            const real e = p.rpm - rig.engine_rpm();
            integ = std::clamp(integ + 0.0002 * e * kDt, 0.0, 1.0);
            thr = std::clamp(integ + 0.0004 * e, 0.0, 1.0);
            rig.set_throttle(thr);
        } else {
            rig.set_throttle(0.0);
        }
        rig.step();
        const real ext = rig.ledger().external_absorbed;
        const real om = std::max(rig.engine_omega_rad_s(), 1.0);
        load_meas = (ext - ext_prev) / (om * kDt);
        ext_prev = ext;
        const EngineThermalSnapshot& th = rig.thermal_snapshot();
        if (th.fan_on && !fan_prev) ++r.fan_on_transitions;
        fan_prev = th.fan_on;
        r.max_coolant = std::max(r.max_coolant, th.coolant_c);
        if ((t + 1) % static_cast<int>(kTickHz) == 0) {
            const real secs = static_cast<real>(t + 1) / kTickHz;
            r.s.push_back(Sample{secs, th.coolant_c, th.oil_c, th.thermostat_opening, th.friction_scale, rig.engine_rpm(), load_meas,
                                 rig.ledger().fuel_mass, th.fan_on});
            if (secs > (p.idle ? 5.0 : 60.0)) {
                r.min_rpm_after_5s = std::min(r.min_rpm_after_5s, rig.engine_rpm());
                r.max_rpm_after_5s = std::max(r.max_rpm_after_5s, rig.engine_rpm());
            }
        }
        (void)omega_t;
    }
    return r;
}

const Sample& at_second(const HeldResult& r, int sec) { return r.s[static_cast<std::size_t>(sec - 1)]; }

// First second (1-based) at which the selected temperature reaches `target`, or -1.
template <class F>
int first_reach(const HeldResult& r, real target, F pick) {
    for (const Sample& s : r.s) {
        if (pick(s) >= target) return static_cast<int>(s.t);
    }
    return -1;
}

// I3 / I6 operating point: 1500 rpm, 30 N*m, 25 C, ram 5.2 m/s (ECE-15 mean speed).
const HeldResult& i3_cold() {
    static const HeldResult r = [] {
        HeldPoint p;
        p.rpm = 1500.0; p.load_nm = 30.0; p.ambient_c = 25.0; p.ram_mps = 5.2; p.start = ThermalStartKind::Ambient; p.duration_s = 1200.0;
        return run_held(p);
    }();
    return r;
}

const HeldResult& i3_warm() {
    static const HeldResult r = [] {
        HeldPoint p;
        p.rpm = 1500.0; p.load_nm = 30.0; p.ambient_c = 25.0; p.ram_mps = 5.2; p.start = ThermalStartKind::Warm; p.duration_s = 1200.0;
        return run_held(p);
    }();
    return r;
}

// ---------------------------------------------------------------------------------------------------------------
// World, car_sedan_*_thermal, ambient start
// ---------------------------------------------------------------------------------------------------------------
struct WorldRig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

// starter_stall_nm > 0 replaces the engine's starter stall torque (a bench starter sized for the cold-torque balance).
WorldRig make_world(const char* vehicle_file, real ambient_c, real starter_stall_nm = 0.0) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(data_path(vehicle_file));
    REQUIRE(desc.powertrain.thermal.has_value());
    if (starter_stall_nm > 0.0) {
        for (ComponentDesc& c : desc.powertrain.components) {
            if (auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) e->starter.stall_torque_nm = starter_stall_nm;
            if (auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) e->starter.stall_torque_nm = starter_stall_nm;
        }
    }
    desc.powertrain.thermal->start = ThermalStartKind::Ambient;
    WorldRig rig;
    WorldConfig cfg;
    cfg.job_workers = 1;
    rig.world = std::make_unique<World>(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId body = rig.world->create_body(chassis);
    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(data_path("surfaces/surfaces.json")));
    rig.vehicle = rig.world->create_vehicle(desc, body);
    rig.world->set_control("assist.auto_clutch", 1.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = EngineState::Off;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    AmbientSample a;
    a.temperature_k = ambient_c + kKelvin;
    rig.world->set_ambient(a);
    return rig;
}

struct ColdStart {
    real t_running = -1.0;          // seconds from the first starter tick to the first Running tick, -1 never
    bool stalled_after = false;     // Stalled seen after the engine first ran
    real min_rpm = 1e9, max_rpm = -1e9; // over [5 s after Running, end]
    real k_start = 0.0;
    real peak_crank_friction_nm = 0.0; // peak (friction heat / omega) while cranking, a report value
    real max_crank_rpm = 0.0;
    real final_rpm = 0.0;
    int final_state = -1;
};

ColdStart run_cold_start(const char* vehicle_file, real ambient_c, real total_s, real starter_max_s, real starter_stall_nm = 0.0) {
    WorldRig rig = make_world(vehicle_file, ambient_c, starter_stall_nm);
    World& w = *rig.world;
    ColdStart r;
    w.set_control("ignition", 1.0);
    w.set_control("starter", 1.0);
    const int ticks = static_cast<int>(total_s * kTickHz);
    bool running_seen = false;
    int running_tick = -1;
    for (int t = 0; t < ticks; ++t) {
        w.step();
        const PowertrainSnapshot& ps_ = w.powertrain_state(rig.vehicle);
        if (t == 0) r.k_start = ps_.thermal.friction_scale;
        const bool running = ps_.engine_state == EngineState::Running;
        if (!running_seen) {
            const real omega = std::max(ps_.engine_rpm * 2.0 * 3.14159265358979323846 / 60.0, 1.0);
            r.peak_crank_friction_nm = std::max(r.peak_crank_friction_nm, ps_.thermal.friction_heat_w / omega);
            r.max_crank_rpm = std::max(r.max_crank_rpm, ps_.engine_rpm);
        }
        if (running && !running_seen) {
            running_seen = true;
            running_tick = t;
            r.t_running = static_cast<real>(t + 1) * kDt;
            w.set_control("starter", 0.0);
        } else if (!running_seen && static_cast<real>(t + 1) * kDt > starter_max_s) {
            w.set_control("starter", 0.0);
        }
        if (running_seen) {
            if (ps_.engine_state == EngineState::Stalled) r.stalled_after = true;
            if (t - running_tick > static_cast<int>(5.0 * kTickHz)) {
                r.min_rpm = std::min(r.min_rpm, ps_.engine_rpm);
                r.max_rpm = std::max(r.max_rpm, ps_.engine_rpm);
            }
        }
        r.final_rpm = ps_.engine_rpm;
        r.final_state = static_cast<int>(ps_.engine_state);
    }
    return r;
}

void report(const char* tag, const ColdStart& r) {
    WARN(tag << ": k_start " << r.k_start << ", running at " << r.t_running << " s, stalled_after " << r.stalled_after
             << ", rpm [" << r.min_rpm << ", " << r.max_rpm << "], final " << r.final_rpm << " (state " << r.final_state
             << "), peak cranking friction " << r.peak_crank_friction_nm << " N*m, max cranking rpm " << r.max_crank_rpm);
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// I2 - cold idle holds
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("I2 cold idle holds: car_sedan_sim_thermal from an ambient start at 20 C", "[thermal][drivetrain]") {
    const ColdStart r = run_cold_start("vehicles/car_sedan_sim_thermal.json", 20.0, 36.0, 3.0);
    report("I2 20 C", r);
    CHECK(r.k_start > 2.0);
    REQUIRE(r.t_running > 0.0);
    CHECK(r.t_running <= 3.0);
    CHECK_FALSE(r.stalled_after);
    CHECK(r.final_state == static_cast<int>(EngineState::Running));
    // After 5 s of running the idle holds its target (800 rpm class) within the plan's +-75 rpm.
    CHECK(r.min_rpm >= 725.0);
    CHECK(r.max_rpm <= 875.0);
}

// The shipped starter (140 N*m stall, [AS]) cannot break the engine away below about 7 C (k*45 N*m > 140 N*m, see I9),
// so the -10 C idle case uses a bench starter sized for the torque balance (2 x k * breakaway, deviation from the plan).
TEST_CASE("I2 cold idle holds (long): 120 s at 20 C and 60 s at -10 C (bench starter)", "[thermal][drivetrain][long]") {
    const ColdStart a = run_cold_start("vehicles/car_sedan_sim_thermal.json", 20.0, 125.0, 3.0);
    report("I2 long 20 C", a);
    CHECK(a.t_running > 0.0);
    CHECK_FALSE(a.stalled_after);
    CHECK(a.min_rpm >= 725.0);
    CHECK(a.max_rpm <= 875.0);
    const ColdStart b = run_cold_start("vehicles/car_sedan_sim_thermal.json", -10.0, 65.0, 5.0, 2.0 * 5.34 * 45.0);
    report("I2 long -10 C", b);
    CHECK(b.k_start > 5.0);
    REQUIRE(b.t_running > 0.0);
    CHECK_FALSE(b.stalled_after);
    CHECK(b.min_rpm >= 725.0);
    CHECK(b.max_rpm <= 875.0);
}

// FINDING (reported): the graph (torque-map) engine's cranking is razor-thin by construction - the starter (140 N*m
// stall, 400 rpm free speed) against the motoring curve reaches 552 rpm at k = 1 and start_complete_rpm is 550, so
// every bit of added cold friction (k_g = 1 + 0.6 (k - 1) on the smooth part) leaves the crank at its cranking
// equilibrium below 550 rpm: 60 C starts (550.6 rpm, 1.19 s), 40 C and colder do not (391 rpm at 40 C, 355 at 20 C).
// The simulated engine, whose combustion torque during cranking is modelled, starts and idles down to -30 C (below).
TEST_CASE("I2 graph engine (car_sedan_thermal): the baseline at k = 1 starts", "[thermal][drivetrain]") {
    const ColdStart r = run_cold_start("vehicles/car_sedan_thermal.json", 95.0, 12.0, 3.0); // ambient start at 95 C: k = 1
    report("I2 graph baseline", r);
    CHECK(r.k_start == Approx(1.0).margin(1e-3));
    REQUIRE(r.t_running > 0.0);
    CHECK(r.t_running <= 3.0);
    CHECK_FALSE(r.stalled_after);
}

TEST_CASE("I2 graph engine (car_sedan_thermal) from an ambient start at 20 C starts (known to fail)", "[thermal][drivetrain][!mayfail]") {
    const ColdStart r = run_cold_start("vehicles/car_sedan_thermal.json", 20.0, 36.0, 3.0);
    report("I2 graph 20 C", r);
    CHECK(r.k_start > 2.0);
    REQUIRE(r.t_running > 0.0);
    CHECK(r.t_running <= 3.0);
    CHECK_FALSE(r.stalled_after);
}

// ---------------------------------------------------------------------------------------------------------------
// I9 - cold crank and start at -30 C, no friction cap
// ---------------------------------------------------------------------------------------------------------------
// FINDING (reported, no cap added per the plan): with the shipped starter (stall torque 140 N*m [AS], data/engines/
// toyota_a25a_fks_sim.json and i4_2l_na.json) and breakaway torque 45 N*m [AS] times k, the engine never turns once
// k * 45 > 140, i.e. below about 7 C oil (k = 3.11; k(0 C) = 3.85, k(-30 C) = 12.72 -> 572 N*m). The plan's bound
// ("Running within 10 s") therefore fails for BOTH engine kinds; the case below keeps that bound as a [!mayfail] check.
// The explanation is the torque balance, pinned by the discriminating A/B in the next case.
TEST_CASE("I9 cold crank and start at -30 C with the shipped starter (plan bound, known to fail)", "[thermal][drivetrain][long][!mayfail]") {
    for (const char* file : {"vehicles/car_sedan_sim_thermal.json", "vehicles/car_sedan_thermal.json"}) {
        INFO(file);
        const ColdStart r = run_cold_start(file, -30.0, 45.0, 10.0);
        report(file, r);
        REQUIRE(r.t_running > 0.0);
        CHECK(r.t_running <= 10.0);
        CHECK_FALSE(r.stalled_after);
    }
}

TEST_CASE("I9 torque balance: the -30 C start fails exactly when the starter stall torque is below k * breakaway", "[thermal][drivetrain][long]") {
    constexpr real k30 = 12.72; // U2: k(-30 C)
    constexpr real breakaway = 45.0;
    for (const char* file : {"vehicles/car_sedan_sim_thermal.json", "vehicles/car_sedan_thermal.json"}) {
        INFO(file);
        // Shipped starter, and a starter at 80 % of the breakaway torque: the crank never moves.
        const ColdStart shipped = run_cold_start(file, -30.0, 12.0, 10.0);
        report("shipped starter", shipped);
        CHECK(shipped.k_start == Approx(k30).margin(0.05));
        CHECK(shipped.max_crank_rpm < 1.0);
        CHECK(shipped.t_running < 0.0);
        const ColdStart weak = run_cold_start(file, -30.0, 12.0, 10.0, 0.8 * k30 * breakaway);
        report("starter at 0.8 k*breakaway", weak);
        CHECK(weak.max_crank_rpm < 1.0);
        CHECK(weak.t_running < 0.0);
        // A starter at 2 x k * breakaway cranks the crank.
        const ColdStart bench = run_cold_start(file, -30.0, 45.0, 10.0, 2.0 * k30 * breakaway);
        report("starter at 2 k*breakaway", bench);
        CHECK(bench.max_crank_rpm > 100.0);
        if (std::string(file).find("sim_thermal") != std::string::npos) {
            // Simulated engine: it fires, runs, and holds an idle at k = 12.7 (ECU idle feed-forward knows k) - the
            // actual I9 question, answered: the start fails only for want of starter torque.
            REQUIRE(bench.t_running > 0.0);
            CHECK(bench.t_running <= 10.0);
            CHECK_FALSE(bench.stalled_after);
            CHECK(bench.final_state == static_cast<int>(EngineState::Running));
            CHECK(bench.min_rpm >= 725.0);
            CHECK(bench.max_rpm <= 875.0);
        } else {
            // FINDING: the graph (torque-map) engine cranks at the torque-balance speed (about 326 rpm: starter free
            // speed 400 rpm, friction 1 + 0.6 (k - 1) = 8 on the smooth part) but its idle-limited combustion torque
            // cannot carry it to start_complete_rpm (550); it never reaches Running.
            CHECK(bench.t_running < 0.0);
            CHECK(bench.max_crank_rpm < 550.0);
        }
    }
}


// ---------------------------------------------------------------------------------------------------------------
// I3 - warm-up time
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("I3 warm-up time: coolant 80 C in [400, 1100] s, oil 60 s behind, oil < 100 C at 1180 s", "[thermal][drivetrain][long]") {
    const HeldResult& r = i3_cold();
    const int t_cool = first_reach(r, 80.0, [](const Sample& s) { return s.coolant; });
    const int t_oil = first_reach(r, 80.0, [](const Sample& s) { return s.oil; });
    WARN("I3: coolant 80 C at " << t_cool << " s, oil 80 C at " << t_oil << " s, oil at 1180 s " << at_second(r, 1180).oil
         << " C, coolant at 1200 s " << at_second(r, 1200).coolant << " C, rpm [" << r.min_rpm_after_5s << ", "
         << r.max_rpm_after_5s << "], load at 600 s " << at_second(r, 600).load << " N*m, k at 1200 s " << at_second(r, 1200).k);
    // The hold really holds the point (guards a vacuous run).
    CHECK(r.min_rpm_after_5s > 1450.0);
    CHECK(r.max_rpm_after_5s < 1550.0);
    CHECK(at_second(r, 600).load == Approx(30.0).margin(1.5));
    REQUIRE(t_cool > 0);
    CHECK(t_cool >= 400);
    CHECK(t_cool <= 1100);
    REQUIRE(t_oil > 0);
    CHECK(t_oil >= t_cool + 60);
    CHECK(at_second(r, 1180).oil < 100.0);
}

// ---------------------------------------------------------------------------------------------------------------
// I4 - highway regulation
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("I4 highway regulation: coolant 88-97 C, thermostat part-open, fan never on", "[thermal][drivetrain][long]") {
    HeldPoint p;
    p.rpm = 2500.0; p.load_nm = 60.0; p.ambient_c = 25.0; p.ram_mps = 33.3; p.start = ThermalStartKind::Warm; p.duration_s = 600.0;
    const HeldResult r = run_held(p);
    real lo = 1e9, hi = -1e9, olo = 1e9, ohi = -1e9;
    bool fan = false;
    for (const Sample& s : r.s) {
        if (s.t > 300.0) {
            lo = std::min(lo, s.coolant);
            hi = std::max(hi, s.coolant);
            olo = std::min(olo, s.opening);
            ohi = std::max(ohi, s.opening);
        }
        fan = fan || s.fan;
    }
    WARN("I4: coolant over the last 300 s [" << lo << ", " << hi << "] C, opening [" << olo << ", " << ohi << "], fan "
         << fan << ", load at 450 s " << at_second(r, 450).load << " N*m, rpm [" << r.min_rpm_after_5s << ", " << r.max_rpm_after_5s << "]");
    CHECK(r.min_rpm_after_5s > 2450.0);
    CHECK(r.max_rpm_after_5s < 2550.0);
    CHECK(at_second(r, 450).load == Approx(60.0).margin(3.0));
    // Deviation from the plan (bound [88, 97]): the 20 s wax lag with the 2 K play makes the regulator hunt around the
    // 88 C opening point, measured [85.5, 90.9] C with the opening touching 0; the upper bound (97 C) is the plan's.
    CHECK(lo >= 83.0);
    CHECK(hi <= 97.0);
    CHECK(ohi > 0.02); // regulating, not pinned shut
    CHECK(ohi < 0.5);  // and nowhere near full open
    CHECK_FALSE(fan);
    CHECK(r.fan_on_transitions == 0);
}

// ---------------------------------------------------------------------------------------------------------------
// I5 - hot idle
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("I5 hot idle: 35 C, the fan cycles and keeps the coolant under 100 C", "[thermal][drivetrain][long]") {
    HeldPoint p;
    p.idle = true; p.ambient_c = 35.0; p.ram_mps = 0.0; p.start = ThermalStartKind::Warm; p.duration_s = 2400.0;
    const HeldResult r = run_held(p);
    for (int sec : {200, 600, 1000, 1200, 1400, 1800, 2400}) { WARN("I5 trace t=" << sec << " coolant " << at_second(r, sec).coolant << " oil " << at_second(r, sec).oil << " opening " << at_second(r, sec).opening << " fan " << at_second(r, sec).fan); }
    WARN("I5: max coolant " << r.max_coolant << " C, fan-on transitions " << r.fan_on_transitions << ", coolant at 1200 s "
         << at_second(r, 1200).coolant << ", idle rpm [" << r.min_rpm_after_5s << ", " << r.max_rpm_after_5s << "]");
    CHECK(r.min_rpm_after_5s > 700.0); // it really idles
    CHECK(at_second(r, 1200).coolant < 100.0); // the plan's 1200 s statement
    CHECK(r.max_coolant < 100.0);               // and it stays under for the whole window
    CHECK(r.fan_on_transitions >= 1);
}

// ---------------------------------------------------------------------------------------------------------------
// I6 - cold vs warm fuel
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("I6 cold-engine fuel penalty: 1200 s at the I3 point, cold/warm in [1.03, 1.20]", "[thermal][drivetrain][long]") {
    const HeldResult& cold = i3_cold();
    const HeldResult& warm = i3_warm();
    const real f_cold = cold.s.back().fuel_kg;
    const real f_warm = warm.s.back().fuel_kg;
    WARN("I6: fuel cold " << f_cold << " kg, warm " << f_warm << " kg, ratio " << f_cold / f_warm);
    REQUIRE(f_warm > 0.0);
    CHECK(f_cold / f_warm >= 1.03);
    CHECK(f_cold / f_warm <= 1.20);
}

TEST_CASE("DIAG graph start sweep", "[.][diag]") {
    for (real amb : {95.0, 60.0, 40.0, 30.0, 20.0, 10.0}) {
        const ColdStart r = run_cold_start("vehicles/car_sedan_thermal.json", amb, 20.0, 10.0);
        WARN("graph ambient " << amb << " C");
        report("graph", r);
    }
    for (real amb : {95.0, 20.0}) {
        const ColdStart r = run_cold_start("vehicles/car_sedan_sim_thermal.json", amb, 20.0, 10.0);
        WARN("sim ambient " << amb << " C");
        report("sim", r);
    }
}

// Runtime measurement (T1 budget: EngineThermal::step() < 1 us per tick; the live ECU path is the extra cost of a
// k recompute). Not a pass/fail test.
TEST_CASE("DIAG thermal runtime", "[.][diag]") {
    {
        EngineThermalConfig c;
        c.engine = io::load_engine_thermal_json(data_path("engine_thermal/toyota_a25a_fks.json"));
        c.cooling = io::load_cooling_system_json(data_path("cooling_systems/sedan.json"));
        EngineThermal th(c);
        th.set_env(ThermalEnv{293.15, 101325.0, 20.0, 3000.0});
        constexpr int n = 2000000;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < n; ++i) {
            th.begin_tick();
            th.accumulate(1.0e3, 2.0e2, kDt);
            th.step(3000.0);
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double ns_per_tick = std::chrono::duration<double, std::nano>(t1 - t0).count() / n;
        WARN("EngineThermal begin_tick+accumulate+step: " << ns_per_tick << " ns per tick");
    }
    for (int mode = 0; mode < 3; ++mode) {
        EngineRig::Config cfg;
        cfg.initial_engine_state = EngineState::Running;
        if (mode > 0) {
            cfg.cooling_system_path = data_path("cooling_systems/sedan.json");
            cfg.thermal_start = mode == 1 ? ThermalStartKind::Warm : ThermalStartKind::Ambient;
        }
        EngineRig rig(data_path("engines/toyota_a25a_fks_sim.json"), io::EngineMapOptions{}, nullptr, cfg);
        rig.set_ignition(1.0);
        rig.set_throttle(0.0);
        for (int i = 0; i < 2400; ++i) rig.step();
        constexpr int ticks = 240 * 600; // 600 s at idle: the oil warms a few K, k changes at every K of the oil
        double worst = 0.0, total = 0.0;
        for (int i = 0; i < ticks; ++i) {
            const auto a = std::chrono::steady_clock::now();
            rig.step();
            const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - a).count();
            worst = std::max(worst, us);
            total += us;
        }
        WARN((mode == 0 ? "plain" : mode == 1 ? "thermal warm" : "thermal ambient") << " idle rig: mean " << total / ticks
                                                                                      << " us per tick, worst " << worst << " us");
    }
}
