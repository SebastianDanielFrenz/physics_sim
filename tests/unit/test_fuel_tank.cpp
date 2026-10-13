// tests/unit/test_fuel_tank.cpp -- PHYS-033 P4 I7a C6: the fuel tank
// (core/src/drivetrain/energy_store.h's FuelTank behind the IEnergyStore
// interface), the "fluid" link that feeds an engine's fuel port from it, fuel
// starvation in EngineTorqueMap::pre_solve, and the public surface
// (World::fuel_tank_mass/set_fuel_tank_mass, PowertrainSnapshot::fuel_mass_kg,
// the vehicle<i>.fuel.tank_kg telemetry channel).
//
// Fixtures (tests/support/fixtures/, never a shipped data file):
//   vehicle_tank_sedan.json          car_sedan.json + an E5 tank (40/30 kg); the
//                                    engine is the shipped crude no-map
//                                    i4_2l_na.json (fuels [e5, e10]).
//   vehicle_tank_sedan_e10.json      the same with an E10 tank (fuels[1]).
//   vehicle_tank_fuel_map_sedan.json vehicle_fuel_map_sedan.json (inline
//                                    engine with a fuel map for gasoline) + a
//                                    gasoline tank.
// Every test drives the production path (load_vehicle_json -> World ->
// VehicleInstance -> Powertrain -> EngineTorqueMap). Expected values are
// literals or recomputed here from the loaded public descs (curves,
// inertias) or literal data-file numbers (LHVs, curve knots), never read
// back from the code under test. T10 is the one white-box case: it calls
// EngineTorqueMap::pre_solve directly on an engine built by the production
// to_engine_desc resolver.

#include "drivetrain/engine_torque_map.h" // T10: white-box pre_solve (core/src on the include path)
#include "drivetrain/powertrain.h"        // T10: the production to_engine_desc resolver
#include "powertrain_multi_diff_desc.h"   // ps_test::load_sedan_desc()

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/fuel_io.h"
#include "ps/io/scenario.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRpmToRadPerSec = real(2.0) * kPi / real(60.0);
real rpm_to_omega(real rpm) { return rpm * kRpmToRadPerSec; }

std::string fixture_path(const std::string& name) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/" + name; }
std::string tank_sedan_path() { return fixture_path("vehicle_tank_sedan.json"); }
std::string tank_sedan_e10_path() { return fixture_path("vehicle_tank_sedan_e10.json"); }
std::string tank_map_sedan_path() { return fixture_path("vehicle_tank_fuel_map_sedan.json"); }
std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

FuelTankDesc& tank_of(vehicle::VehicleDesc& d) {
    for (auto& c : d.powertrain.components) {
        if (auto* t = std::get_if<FuelTankDesc>(&c.params)) return *t;
    }
    FAIL("fixture has no fuel_tank component");
    throw std::logic_error("unreachable");
}
template <typename T>
const T& component_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* p = std::get_if<T>(&c.params)) return *p;
    }
    FAIL("fixture lacks a required component type");
    throw std::logic_error("unreachable");
}

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

// Same ground/chassis bodies as data/scenarios/vehicle_powertrain.json (and
// test_engine_fuel_map.cpp's make_world_rig), configurable rates/gear/workers.
// Manual clutch (pedal 0 = engaged, reset locked), no assists, throttle 0 -
// or, with auto_clutch_launch, the auto-clutch launch law from an open
// clutch (an instant clutch dump from rest in 1st stalls the engine, see
// test_engine_fuel_map.cpp's "Fix 1" comment).
Rig make_rig(const vehicle::VehicleDesc& desc, real tick_hz, real substep_hz, int gear, int workers = 1, bool auto_clutch_launch = false) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = substep_hz;
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
    rig.chassis = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(data_path("surfaces/surfaces.json")));
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);

    rig.world->set_control("assist.auto_clutch", auto_clutch_launch ? 1.0 : 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = gear;
    o.engine_state = EngineState::Running;
    o.clutch_locked = !auto_clutch_launch;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

bool same_pose(const Pose& a, const Pose& b) {
    return a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z &&
           a.orientation.x == b.orientation.x && a.orientation.y == b.orientation.y && a.orientation.z == b.orientation.z &&
           a.orientation.w == b.orientation.w;
}
bool same_motion(const Motion& a, const Motion& b) {
    return a.linear.x == b.linear.x && a.linear.y == b.linear.y && a.linear.z == b.linear.z && a.angular.x == b.angular.x &&
           a.angular.y == b.angular.y && a.angular.z == b.angular.z;
}

real sum_abs_terms(const PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
}

} // namespace

// ---------------------------------------------------------------------------
// T1: a tank that never runs dry changes nothing. vehicle_tank_sedan.json's
// tank holds E5 = fuels[0] of i4_2l_na.json, the fuel car_sedan burns anyway,
// so every dynamic quantity must be BIT-identical per tick to car_sedan over
// the vehicle_powertrain.json control script (s_starve == 1 exactly, x*1.0
// == x with FP contraction off). The plain world must still hash to the
// recorded scenario value; the tank world must hash differently (the tank
// content is simulation state).
// ---------------------------------------------------------------------------
TEST_CASE("C6 T1: a never-empty tank is bit-identical to car_sedan over the vehicle_powertrain script", "[unit][drivetrain][fuel]") {
    io::Scenario sa = io::load_scenario(data_path("scenarios/vehicle_powertrain.json"));
    sa.world_config.job_workers = 1;
    io::Scenario sb = sa;
    REQUIRE(sb.vehicles.size() == 1);
    sb.vehicles[0].desc = io::load_vehicle_json(tank_sedan_path());

    io::BuiltScenario a = io::build_scenario(sa);
    io::BuiltScenario b = io::build_scenario(sb);
    World& wa = *a.world;
    World& wb = *b.world;
    const BodyId ca = a.bodies_by_name.at("chassis");
    const BodyId cb = b.bodies_by_name.at("chassis");
    const VehicleId va = a.vehicles_by_name.at("car");
    const VehicleId vb = b.vehicles_by_name.at("car");
    REQUIRE(wa.vehicle_wheel_count(va) == wb.vehicle_wheel_count(vb));

    const auto num_ticks = static_cast<std::uint64_t>(sa.duration_s * sa.world_config.tick_rate_hz + 0.5);
    REQUIRE(num_ticks == 2400);
    std::size_t next = 0;
    long long first_mismatch = -1;
    for (std::uint64_t i = 0; i < num_ticks; ++i) {
        while (next < sa.control_events.size() && sa.control_events[next].time_s <= wa.sim_time()) {
            wa.set_control(sa.control_events[next].channel, sa.control_events[next].value);
            wb.set_control(sa.control_events[next].channel, sa.control_events[next].value);
            ++next;
        }
        wa.step();
        wb.step();
        bool same = same_pose(wa.get_pose(ca), wb.get_pose(cb)) && same_motion(wa.backend().get_motion(ca), wb.backend().get_motion(cb));
        for (std::size_t w = 0; w < wa.vehicle_wheel_count(va); ++w) same = same && wa.wheel_omega(va, w) == wb.wheel_omega(vb, w);
        const auto& pa = wa.powertrain_state(va);
        const auto& pb = wb.powertrain_state(vb);
        same = same && pa.engine_rpm == pb.engine_rpm && pa.fuel_flow_g_s == pb.fuel_flow_g_s;
        const PowertrainEnergy& ea = wa.powertrain_energy(va);
        const PowertrainEnergy& eb = wb.powertrain_energy(vb);
        same = same && ea.fuel_mass == eb.fuel_mass && ea.fuel_energy_in == eb.fuel_energy_in && ea.engine_heat == eb.engine_heat &&
               ea.engine_out == eb.engine_out;
        if (!same && first_mismatch < 0) first_mismatch = static_cast<long long>(i);
    }
    INFO("first mismatching tick: " << first_mismatch);
    CHECK(first_mismatch == -1);
    CHECK(wa.state_hash() == 0xaadddd7bcd24ac15ULL); // 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0xc72122c8fe8d0228): data/scenarios/vehicle_powertrain.json's recorded hash (re-recorded 2026-10-04 for ca118ec's instant anti-stall release, was 0x5380949669212c71)
    CHECK(wb.state_hash() != wa.state_hash());       // the tank's content is hashed
    // Non-vacuous: fuel really flowed through the tank.
    const PowertrainEnergy& eb = wb.powertrain_energy(vb);
    CHECK(eb.fuel_mass > 0.01);
    CHECK(wb.fuel_tank_mass(vb) < 30.0);
    CHECK(wb.powertrain_state(vb).fuel_mass_kg == wb.fuel_tank_mass(vb));
    WARN("T1: fuel burnt over the script " << eb.fuel_mass << " kg, tank left " << wb.fuel_tank_mass(vb) << " kg");
}

// ---------------------------------------------------------------------------
// T2: mass conservation. initial - m_tank == fuel_mass - fuel_external_added
// every tick to 1e-12 relative to the 30 kg initial (each substep's
// withdraw/booking adds at most half an ulp of 30 kg to either side - about
// 12,500 substeps x 1.8e-15 kg = 2.2e-11 kg worst case), through a WOT
// launch, a DFCO overrun and idle; m_tank >= 0; the snapshot mirrors it.
// ---------------------------------------------------------------------------
TEST_CASE("C6 T2: tank mass is conserved against the fuel ledger every tick (WOT, DFCO, idle)", "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = io::load_vehicle_json(tank_sedan_path());
    Rig rig = make_rig(desc, 240.0, 960.0, 1, 1, /*auto_clutch_launch=*/true);
    World& w = *rig.world;
    const real initial = 30.0;
    REQUIRE(w.fuel_tank_mass(rig.vehicle) == initial);

    real worst = 0.0;
    bool saw_dfco = false;
    auto check_tick = [&]() {
        const real m = w.fuel_tank_mass(rig.vehicle);
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real err = std::abs((initial - m) - (e.fuel_mass - e.fuel_external_added));
        worst = std::max(worst, err);
        CHECK(err <= 1e-12 * initial);
        CHECK(m >= 0.0);
        CHECK(w.powertrain_state(rig.vehicle).fuel_mass_kg == m);
        if (w.powertrain_state(rig.vehicle).engines.size() > 0 && w.powertrain_state(rig.vehicle).engines[0].dfco_active) saw_dfco = true;
    };
    for (int t = 0; t < 240; ++t) { w.step(); check_tick(); } // 1 s idle in gear 1
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 480; ++t) { w.step(); check_tick(); } // 2 s WOT
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 2400; ++t) { w.step(); check_tick(); } // 10 s overrun -> idle
    const auto& snap = w.powertrain_state(rig.vehicle);
    REQUIRE(snap.engines.size() > 0);
    CHECK(saw_dfco);
    CHECK_FALSE(snap.engines[0].dfco_active);
    CHECK(snap.engine_state == EngineState::Running);
    CHECK(snap.fuel_flow_g_s > 0.0); // idle burns fuel
    CHECK(w.powertrain_energy(rig.vehicle).fuel_external_added == 0.0);
    CHECK(w.powertrain_energy(rig.vehicle).fuel_mass > 0.005);
    WARN("T2: worst |(m0 - m) - (fuel_mass - ext)| = " << worst << " kg; burnt " << w.powertrain_energy(rig.vehicle).fuel_mass << " kg");
}

// ---------------------------------------------------------------------------
// T3 / T5: running dry. 1:1 (960/960 Hz, one substep per step()) in neutral
// with the clutch locked, WOT from a small initial tank:
//   - the emptying substep leaves the tank at EXACTLY 0.0, books exactly the
//     remaining mass and reports that mass as this substep's flow;
//   - afterwards fuel_mass never moves again, load (theta_c) and flow are 0;
//   - the engine coasts down on its motoring torque alone and reaches
//     Stalled (rpm < stall_rpm at a substep start) within a bound COMPUTED
//     from the loaded desc (see stall_time_bounds below);
//   - the starter cannot restart it (starter free speed < start_complete_rpm
//     and there is no fuel to fire), while the idle PI winds up to its clamp;
//   - after set_fuel_tank_mass(1.0) the starter restarts it.
// T3 runs the crude no-map engine (vehicle_tank_sedan.json), T5 the fuel-map
// engine (vehicle_tank_fuel_map_sedan.json) through the same helper.
//
// Stall-time model. After the emptying substep only the motoring torque acts
// (the smooth part s*(m + c_k) plus the Coulomb row's -c_k sum to m(rpm)).
// Neutral with the clutch locked spins the engine, the clutch disc and the
// gearbox input shaft together: J_hi = J_engine + J_clutch_in + J_clutch_out
// + J_gearbox_in; J_lo = J_engine alone (clutch side dropped, a strict lower
// bound). With |m| monotone increasing in rpm on [stall, rpm0]:
//   t_lower = J_lo (w0 - w_s) / max|m|,   t_upper = J_hi (w0 - w_s) / min|m| + 2h
//   t_pred  = integral_{w_s}^{w0} J_hi / |m(w)| dw   (midpoint rule, 20000 steps)
// and the measured t = (k_stall - k_empty - 1) h must lie inside
// [t_lower, t_upper] (a loose sanity check) and within 0.3 % + 2h of t_pred
// (linearly-implicit Euler plus the one-substep detection lag; measured
// +0.050 % crude, +0.035 % map, +0.35 % = 0.9 ms at idle, i.e. inside 2h).
// ---------------------------------------------------------------------------
namespace {

constexpr real kStallPredTol = 0.003; // relative, plus 2h - see above

struct StallBounds {
    real t_lower, t_upper, t_pred, J_lo, J_hi, max_m, min_m;
};

StallBounds stall_time_bounds(const vehicle::VehicleDesc& desc, real rpm0, real h) {
    const auto& e = component_of<TorqueMapEngineDesc>(desc);
    const auto& c = component_of<ClutchDesc>(desc);
    const auto& g = component_of<GearboxDesc>(desc);
    StallBounds b{};
    b.J_lo = e.inertia_kgm2;
    b.J_hi = e.inertia_kgm2 + c.in_inertia_kgm2 + c.out_inertia_kgm2 + g.input_inertia_kgm2;
    const real rpm_s = e.stall_rpm;
    b.max_m = 0.0;
    b.min_m = std::numeric_limits<real>::infinity();
    const int n = 20000;
    real t_pred = 0.0;
    const real drpm = (rpm0 - rpm_s) / n;
    for (int i = 0; i <= n; ++i) {
        const real m = -e.motoring_torque_nm_vs_rpm.evaluate(rpm_s + drpm * i);
        b.max_m = std::max(b.max_m, m);
        b.min_m = std::min(b.min_m, m);
        if (i < n) {
            const real m_mid = -e.motoring_torque_nm_vs_rpm.evaluate(rpm_s + drpm * (i + 0.5));
            t_pred += b.J_hi * rpm_to_omega(drpm) / m_mid;
        }
    }
    const real dw = rpm_to_omega(rpm0) - rpm_to_omega(rpm_s);
    b.t_lower = b.J_lo * dw / b.max_m;
    b.t_upper = b.J_hi * dw / b.min_m + 2.0 * h;
    b.t_pred = t_pred;
    return b;
}

// The idle PI integrator's upper clamp, recomputed from the public desc
// (engine_torque_map.cpp: theta_ff = clamp(-m(idle)/(w(idle)-m(idle)), 0,
// max_throttle); integrator in [-theta_ff, max_throttle - theta_ff]).
real idle_integrator_upper_clamp(const TorqueMapEngineDesc& e) {
    const real w = e.wot_torque_nm_vs_rpm.evaluate(e.idle_rpm);
    const real m = e.motoring_torque_nm_vs_rpm.evaluate(e.idle_rpm);
    const real theta_ff = std::clamp(-m / (w - m), real(0.0), e.idle.max_throttle);
    return e.idle.max_throttle - theta_ff;
}

void run_empty_tank_case(const std::string& fixture, const char* label) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(fixture);
    const real initial = 0.005; // kg - a few seconds of WOT at the limiter
    tank_of(desc).initial_kg = initial;
    const auto& eng = component_of<TorqueMapEngineDesc>(desc);
    Rig rig = make_rig(desc, 960.0, 960.0, 0);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;
    const real h = 1.0 / 960.0;

    // --- 1. WOT until the tank runs dry -------------------------------------
    w.set_control("throttle", 1.0);
    long long k = 0;
    long long k_empty = -1;
    real rpm0 = 0.0;
    real fm_empty = 0.0;
    for (; k < 20 * 960; ++k) {
        const real m_prev = w.fuel_tank_mass(v);
        const real fm_prev = w.powertrain_energy(v).fuel_mass;
        w.step();
        const real m = w.fuel_tank_mass(v);
        REQUIRE(m >= 0.0);
        REQUIRE(w.powertrain_state(v).engine_state == EngineState::Running);
        if (m == 0.0) {
            REQUIRE(m_prev > 0.0);
            const real fm = w.powertrain_energy(v).fuel_mass;
            // The whole remaining mass was booked (ledger sum rounding only).
            CHECK(std::abs((fm - fm_prev) - m_prev) <= 4.0 * std::numeric_limits<real>::epsilon() * fm);
            // ...and reported as this substep's delivered flow.
            const real flow_mass = w.powertrain_state(v).fuel_flow_g_s * h / 1000.0;
            CHECK(flow_mass == Catch::Approx(m_prev).epsilon(1e-12));
            k_empty = k;
            rpm0 = w.powertrain_state(v).engine_rpm;
            fm_empty = fm;
            break;
        }
    }
    REQUIRE(k_empty >= 0);
    CHECK(fm_empty == Catch::Approx(initial).epsilon(1e-12));

    // --- 2. coast down to Stalled ------------------------------------------
    long long k_stall = -1;
    real max_integrator_running = -1.0;
    for (++k; k < k_empty + 20 * 960; ++k) {
        w.step();
        const auto& s = w.powertrain_state(v);
        CHECK(w.fuel_tank_mass(v) == 0.0);
        CHECK(w.powertrain_energy(v).fuel_mass == fm_empty);
        REQUIRE(s.engines.size() > 0);
        CHECK(s.engines[0].load == 0.0);
        CHECK(s.fuel_flow_g_s == 0.0);
        if (s.engine_state == EngineState::Stalled) {
            k_stall = k;
            CHECK(w.powertrain_debug(v).idle_integrator == 0.0); // reset on entering Stalled
            break;
        }
        REQUIRE(s.engine_state == EngineState::Running);
        max_integrator_running = std::max(max_integrator_running, w.powertrain_debug(v).idle_integrator);
    }
    REQUIRE(k_stall >= 0);
    const real t_meas = static_cast<real>(k_stall - k_empty - 1) * h;
    const StallBounds b = stall_time_bounds(desc, rpm0, h);
    WARN(label << ": emptied at substep " << k_empty << " (t=" << k_empty * h << " s) at " << rpm0 << " rpm; stalled after t_meas=" << t_meas
               << " s; t_pred=" << b.t_pred << " s (" << (t_meas / b.t_pred - 1.0) * 100.0 << " %), bounds [" << b.t_lower << ", "
               << b.t_upper << "] s (J_lo " << b.J_lo << ", J_hi " << b.J_hi << ", |m| " << b.min_m << ".." << b.max_m
               << " N*m); idle integrator max while starving " << max_integrator_running);
    CHECK(t_meas >= b.t_lower);
    CHECK(t_meas <= b.t_upper);
    CHECK(std::abs(t_meas - b.t_pred) <= kStallPredTol * b.t_pred + 2.0 * h);

    // --- 3. the starter cannot restart a dry engine --------------------------
    w.set_control("throttle", 0.0);
    w.set_control("starter", 1.0);
    bool saw_cranking = false;
    real max_crank_rpm = 0.0;
    real max_integrator_cranking = -1.0;
    for (int i = 0; i < 3 * 960; ++i) {
        w.step();
        const auto& s = w.powertrain_state(v);
        CHECK(s.engine_state != EngineState::Running);
        CHECK(s.engine_rpm < eng.start_complete_rpm);
        CHECK(w.powertrain_energy(v).fuel_mass == fm_empty);
        CHECK(w.fuel_tank_mass(v) == 0.0);
        CHECK(s.engines[0].load == 0.0);
        if (s.engine_state == EngineState::Cranking) {
            saw_cranking = true;
            max_crank_rpm = std::max(max_crank_rpm, s.engine_rpm);
            max_integrator_cranking = std::max(max_integrator_cranking, w.powertrain_debug(v).idle_integrator);
        }
    }
    CHECK(saw_cranking);
    // Non-vacuous: the crank spins above fire_rpm, so combustion WOULD be
    // enabled - only the missing fuel keeps it from firing.
    CHECK(max_crank_rpm > eng.fire_rpm);
    // The idle PI winds up to (exactly) its clamp while cranking dry.
    const real clamp_hi = idle_integrator_upper_clamp(eng);
    CHECK(max_integrator_cranking == Catch::Approx(clamp_hi).epsilon(1e-12));
    WARN(label << ": dry cranking holds " << max_crank_rpm << " rpm (fire " << eng.fire_rpm << ", start_complete " << eng.start_complete_rpm
               << "); idle integrator winds up to " << max_integrator_cranking << " (clamp " << clamp_hi << ")");

    // --- 4. refuel between ticks, then the starter restarts it --------------
    w.set_control("starter", 0.0);
    w.step();
    CHECK(w.powertrain_state(v).engine_state == EngineState::Stalled);
    CHECK(w.powertrain_debug(v).idle_integrator == 0.0);
    w.set_fuel_tank_mass(v, 1.0);
    CHECK(w.fuel_tank_mass(v) == 0.0); // applied at the next tick boundary
    w.step();
    CHECK(w.fuel_tank_mass(v) == 1.0);
    CHECK(w.powertrain_energy(v).fuel_external_added == 1.0);
    CHECK(w.powertrain_energy(v).fuel_mass == fm_empty);

    w.set_control("starter", 1.0);
    int running_after = -1;
    for (int i = 0; i < 2 * 960; ++i) {
        w.step();
        if (w.powertrain_state(v).engine_state == EngineState::Running) {
            running_after = i + 1;
            break;
        }
    }
    REQUIRE(running_after > 0);
    w.set_control("starter", 0.0);
    for (int i = 0; i < 480; ++i) w.step();
    CHECK(w.powertrain_state(v).engine_state == EngineState::Running);
    CHECK(w.powertrain_state(v).engine_rpm > eng.stall_rpm);
    const PowertrainEnergy& e = w.powertrain_energy(v);
    CHECK(e.fuel_mass > fm_empty);
    CHECK(w.fuel_tank_mass(v) < 1.0);
    CHECK(std::abs((initial + e.fuel_external_added - w.fuel_tank_mass(v)) - e.fuel_mass) <= 1e-15 * 100.0);
    WARN(label << ": restarted " << running_after * h << " s after the refuel; fuel since " << e.fuel_mass - fm_empty << " kg");
}

} // namespace

TEST_CASE("C6 T3: a crude-model engine runs dry, stalls within the computed bound, cannot crank, restarts after a refuel",
          "[unit][drivetrain][fuel]") {
    run_empty_tank_case(tank_sedan_path(), "T3 crude");
}

TEST_CASE("C6 T5: a fuel-map engine runs dry, stalls within the computed bound, cannot crank, restarts after a refuel",
          "[unit][drivetrain][fuel]") {
    run_empty_tank_case(tank_map_sedan_path(), "T5 map");
}

// ---------------------------------------------------------------------------
// T3b: running dry at IDLE (the idle-integrator wind-up finding). Throttle 0
// in neutral, crude engine, tiny tank: the idle PI sees a growing error as
// the starved engine sags from 800 rpm to stall_rpm and winds up - bounded
// by its own clamp - and is reset to exactly 0 on entering Stalled, so no
// wound-up integrator can carry into a restart.
// ---------------------------------------------------------------------------
TEST_CASE("C6 T3b: an engine starving at idle winds its idle integrator up to at most the clamp and resets it on stall",
          "[unit][drivetrain][fuel]") {
    vehicle::VehicleDesc desc = io::load_vehicle_json(tank_sedan_path());
    tank_of(desc).initial_kg = 0.0002;
    const auto& eng = component_of<TorqueMapEngineDesc>(desc);
    Rig rig = make_rig(desc, 960.0, 960.0, 0);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;
    const real h = 1.0 / 960.0;

    long long k = 0;
    long long k_empty = -1;
    real integrator_at_empty = 0.0;
    real rpm0 = 0.0;
    for (; k < 30 * 960; ++k) {
        w.step();
        REQUIRE(w.powertrain_state(v).engine_state == EngineState::Running);
        if (w.fuel_tank_mass(v) == 0.0) {
            k_empty = k;
            integrator_at_empty = w.powertrain_debug(v).idle_integrator;
            rpm0 = w.powertrain_state(v).engine_rpm;
            break;
        }
    }
    REQUIRE(k_empty >= 0);
    real max_integrator = integrator_at_empty;
    long long k_stall = -1;
    for (++k; k < k_empty + 10 * 960; ++k) {
        w.step();
        if (w.powertrain_state(v).engine_state == EngineState::Stalled) {
            k_stall = k;
            CHECK(w.powertrain_debug(v).idle_integrator == 0.0);
            break;
        }
        max_integrator = std::max(max_integrator, w.powertrain_debug(v).idle_integrator);
    }
    REQUIRE(k_stall >= 0);
    const real clamp_hi = idle_integrator_upper_clamp(eng);
    const real t_meas = static_cast<real>(k_stall - k_empty - 1) * h;
    const StallBounds b = stall_time_bounds(desc, rpm0, h);
    WARN("T3b idle: emptied at " << k_empty * h << " s at " << rpm0 << " rpm with integrator " << integrator_at_empty << "; stalled after "
                                 << t_meas << " s (t_pred " << b.t_pred << ", bounds [" << b.t_lower << ", " << b.t_upper
                                 << "]); integrator peak while starving " << max_integrator << " (clamp " << clamp_hi << ")");
    CHECK(max_integrator > integrator_at_empty); // it does wind up while the starved engine sags
    CHECK(max_integrator <= clamp_hi);
    CHECK(t_meas >= b.t_lower);
    CHECK(t_meas <= b.t_upper);
    CHECK(std::abs(t_meas - b.t_pred) <= kStallPredTol * b.t_pred + 2.0 * h);
}

// ---------------------------------------------------------------------------
// T4: the tank selects the burnt fuel. E10 tank (fuels[1]) vs E5 tank
// (fuels[0]) over the same WOT free rev: identical torque/rpm trajectory
// (the crude model derives fuel FROM torque), fuel mass in the ratio
// LHV_E5/LHV_E10 (literal LHVs, docs/data_formats/fuel.md's blend table =
// data/fuels/e5.json / e10.json: 43,089,334 and 42,184,298 J/kg); the E5
// tank burns exactly what car_sedan (no tank, fuels[0]) burns.
// ---------------------------------------------------------------------------
TEST_CASE("C6 T4: an E10 tank burns LHV_E5/LHV_E10 times the E5 tank's fuel mass", "[unit][drivetrain][fuel]") {
    const real lhv_e5 = 43089334.0;  // J/kg, fuel.md
    const real lhv_e10 = 42184298.0; // J/kg, fuel.md

    Rig r5 = make_rig(io::load_vehicle_json(tank_sedan_path()), 960.0, 960.0, 0);
    Rig r10 = make_rig(io::load_vehicle_json(tank_sedan_e10_path()), 960.0, 960.0, 0);
    Rig rp = make_rig(ps_test::load_sedan_desc(), 960.0, 960.0, 0);
    for (Rig* r : {&r5, &r10, &rp}) r->world->set_control("throttle", 1.0);
    for (int i = 0; i < 300; ++i) {
        r5.world->step();
        r10.world->step();
        rp.world->step();
        CHECK(r5.world->powertrain_state(r5.vehicle).engine_rpm == r10.world->powertrain_state(r10.vehicle).engine_rpm);
    }
    const real f5 = r5.world->powertrain_energy(r5.vehicle).fuel_mass;
    const real f10 = r10.world->powertrain_energy(r10.vehicle).fuel_mass;
    const real fp = rp.world->powertrain_energy(rp.vehicle).fuel_mass;
    REQUIRE(f5 > 0.0);
    const real ratio = f10 / f5;
    const real expected = lhv_e5 / lhv_e10;
    WARN("T4: fuel E5 tank " << f5 << " kg, E10 tank " << f10 << " kg, ratio " << ratio << " vs LHV_E5/LHV_E10 " << expected);
    CHECK(std::abs(ratio / expected - 1.0) <= 1e-12);
    CHECK(f5 == fp);
    // Same heat released either way (fuel_energy_in = mass * LHV of the burnt fuel).
    CHECK(r10.world->powertrain_energy(r10.vehicle).fuel_energy_in ==
          Catch::Approx(r5.world->powertrain_energy(r5.vehicle).fuel_energy_in).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// T6: World::set_fuel_tank_mass - applied at the next tick boundary, clamped
// to [0, capacity], the last request before a step wins, booked SIGNED into
// fuel_external_added; conservation still closes; NaN and a tank-less
// vehicle throw; the chassis mass does not change (I7a - a later increment
// adds the fuel as a point mass); the telemetry channel sits
// right after vehicle0.fuel.heat, only with a tank.
// ---------------------------------------------------------------------------
TEST_CASE("C6 T6: set_fuel_tank_mass is applied at the tick boundary, clamped and booked signed", "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc desc = io::load_vehicle_json(tank_sedan_path());
    const real capacity = 40.0;
    const real initial = 30.0;
    Rig rig = make_rig(desc, 240.0, 960.0, 0);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;
    REQUIRE(tank_of(const_cast<vehicle::VehicleDesc&>(desc)).capacity_kg == capacity);

    const auto& names = w.telemetry().channel_names();
    const auto heat_it = std::find(names.begin(), names.end(), "vehicle0.fuel.heat");
    REQUIRE(heat_it != names.end());
    REQUIRE(heat_it + 1 != names.end());
    CHECK(*(heat_it + 1) == "vehicle0.fuel.tank_kg");
    CHECK(std::count(names.begin(), names.end(), "vehicle0.fuel.tank_kg") == 1);

    auto conservation_err = [&]() {
        const PowertrainEnergy& e = w.powertrain_energy(v);
        return std::abs((initial + e.fuel_external_added - w.fuel_tank_mass(v)) - e.fuel_mass);
    };

    // While running: a refill to 35 kg during a tick that also burns fuel.
    w.set_control("throttle", 0.5);
    for (int t = 0; t < 240; ++t) w.step();
    const real m_before = w.fuel_tank_mass(v);
    REQUIRE(m_before < initial);
    w.set_fuel_tank_mass(v, 35.0);
    CHECK(w.fuel_tank_mass(v) == m_before);
    CHECK(w.powertrain_energy(v).fuel_external_added == 0.0);
    w.step();
    real ext_expected = 35.0 - m_before;
    CHECK(w.powertrain_energy(v).fuel_external_added == ext_expected);
    CHECK(w.fuel_tank_mass(v) < 35.0); // this tick's burn came out of the refilled tank
    CHECK(w.fuel_tank_mass(v) > 34.99);
    CHECK(conservation_err() <= 1e-12 * capacity);

    // Engine off: nothing burns, so every request lands exactly.
    w.set_control("throttle", 0.0);
    w.set_control("ignition", 0.0);
    w.step();
    w.step();
    REQUIRE(w.powertrain_state(v).engine_state == EngineState::Off);
    const real fm_off = w.powertrain_energy(v).fuel_mass;
    w.step();
    REQUIRE(w.powertrain_energy(v).fuel_mass == fm_off);

    struct Req {
        std::vector<real> requests;
        real expected;
    };
    const std::vector<Req> reqs = {
        {{1000.0}, 40.0},
        {{-5.0}, 0.0},
        {{12.5, 7.25}, 7.25}, // the last request before the step wins
        {{std::numeric_limits<real>::infinity()}, 40.0},
        {{-std::numeric_limits<real>::infinity()}, 0.0},
        {{capacity}, 40.0},
    };
    for (const Req& r : reqs) {
        const real old = w.fuel_tank_mass(v);
        for (const real kg : r.requests) w.set_fuel_tank_mass(v, kg);
        CHECK(w.fuel_tank_mass(v) == old);
        CHECK(w.powertrain_state(v).fuel_mass_kg == old);
        w.step();
        CHECK(w.fuel_tank_mass(v) == r.expected);
        CHECK(w.powertrain_state(v).fuel_mass_kg == r.expected);
        ext_expected += r.expected - old;
        CHECK(w.powertrain_energy(v).fuel_external_added == ext_expected);
        CHECK(w.powertrain_energy(v).fuel_mass == fm_off);
        CHECK(conservation_err() <= 1e-12 * capacity);
        CHECK(w.get_mass(rig.chassis) == 1500.0);
    }

    REQUIRE_THROWS_WITH(w.set_fuel_tank_mass(v, std::numeric_limits<real>::quiet_NaN()), ContainsSubstring("the requested mass is NaN"));
    w.step();
    CHECK(w.fuel_tank_mass(v) == 40.0); // the rejected request left nothing pending
    CHECK(w.powertrain_energy(v).fuel_external_added == ext_expected);
    CHECK(w.get_mass(rig.chassis) == 1500.0);

    // A vehicle without a tank: the setter throws, the getters read 0 and no
    // tank_kg channel exists.
    Rig plain = make_rig(ps_test::load_sedan_desc(), 240.0, 960.0, 0);
    REQUIRE_THROWS_WITH(plain.world->set_fuel_tank_mass(plain.vehicle, 10.0), ContainsSubstring("this vehicle has no fuel_tank"));
    plain.world->step();
    CHECK(plain.world->fuel_tank_mass(plain.vehicle) == 0.0);
    CHECK(plain.world->powertrain_state(plain.vehicle).fuel_mass_kg == 0.0);
    const auto& pnames = plain.world->telemetry().channel_names();
    CHECK(std::find(pnames.begin(), pnames.end(), "vehicle0.fuel.heat") != pnames.end());
    CHECK(std::find(pnames.begin(), pnames.end(), "vehicle0.fuel.tank_kg") == pnames.end());
    CHECK(pnames.size() + 1 == names.size());
}

// ---------------------------------------------------------------------------
// T6b: World::reset_vehicle does NOT refill the tank. Burn fuel, reset, and
// the tank content and fuel_external_added are exactly the pre-reset values
// (literal compare, not a tolerance) - both right after the reset and after
// a step with the ignition off (nothing burns, and no refill was queued for
// the next tick boundary).
// ---------------------------------------------------------------------------
TEST_CASE("C6 T6b: World::reset_vehicle leaves the tank content and fuel_external_added unchanged", "[unit][drivetrain][fuel]") {
    Rig rig = make_rig(io::load_vehicle_json(tank_sedan_path()), 240.0, 960.0, 0);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;
    w.set_control("throttle", 0.5);
    for (int t = 0; t < 240; ++t) w.step();
    const real m_before = w.fuel_tank_mass(v);
    const real ext_before = w.powertrain_energy(v).fuel_external_added;
    REQUIRE(m_before < 30.0); // fuel was burnt
    REQUIRE(m_before > 29.0);
    REQUIRE(ext_before == 0.0);

    w.set_control("throttle", 0.0);
    w.set_control("ignition", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = EngineState::Off;
    o.clutch_locked = true;
    w.reset_vehicle(v, o);
    CHECK(w.fuel_tank_mass(v) == m_before);
    CHECK(w.powertrain_energy(v).fuel_external_added == ext_before);

    w.step();
    REQUIRE(w.powertrain_state(v).engine_state == EngineState::Off);
    CHECK(w.fuel_tank_mass(v) == m_before);
    CHECK(w.powertrain_state(v).fuel_mass_kg == m_before);
    CHECK(w.powertrain_energy(v).fuel_external_added == ext_before);
    WARN("T6b: tank " << m_before << " kg before and after reset_vehicle (initial 30 kg)");
}

// ---------------------------------------------------------------------------
// T7: every loader rejection, pinned to its message (core/src/io/
// vehicle_io.cpp), plus the Powertrain constructor's backstops for a
// code-built desc (core/src/drivetrain/powertrain.cpp). Built from the tank
// fixture with every relative path made absolute and written to a temp file.
// ---------------------------------------------------------------------------
namespace t7 {

struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const nlohmann::json& j, const std::string& tag = "") {
        path = std::filesystem::temp_directory_path() /
               ("ps_test_fuel_tank_" + tag + std::to_string(std::hash<std::string>{}(j.dump())) + ".json");
        std::ofstream out(path);
        out << j.dump(2);
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

std::string abs_from_fixture_dir(const std::string& rel) {
    return (std::filesystem::path(PS_SOURCE_DIR) / "tests/support/fixtures" / rel).lexically_normal().generic_string();
}

// The tank fixture with every relative path (tyres, refs, the tank's fuel,
// an inline engine's fuel/fuels) made absolute.
nlohmann::json absolute_fixture(const std::string& fixture) {
    std::ifstream in(fixture);
    nlohmann::json j = nlohmann::json::parse(in);
    for (auto& wj : j.at("wheels")) {
        if (wj.contains("tyre")) wj["tyre"] = abs_from_fixture_dir(wj["tyre"].get<std::string>());
    }
    for (auto& cj : j.at("powertrain").at("components")) {
        if (cj.contains("ref")) cj["ref"] = abs_from_fixture_dir(cj["ref"].get<std::string>());
        if (cj.contains("fuel")) cj["fuel"] = abs_from_fixture_dir(cj["fuel"].get<std::string>());
        if (cj.contains("fuels")) {
            for (auto& f : cj["fuels"]) f = abs_from_fixture_dir(f.get<std::string>());
        }
    }
    return j;
}

nlohmann::json& component(nlohmann::json& v, const std::string& id) {
    for (auto& cj : v.at("powertrain").at("components")) {
        if (cj.at("id") == id) return cj;
    }
    FAIL("no component " << id);
    throw std::logic_error("unreachable");
}

void expect_rejected(const nlohmann::json& v, const std::string& message) {
    const TempJsonFile f(v);
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(message));
}

void expect_loads(const nlohmann::json& v) {
    const TempJsonFile f(v);
    REQUIRE_NOTHROW(io::load_vehicle_json(f.path.string()));
}

std::string fuel_abs(const std::string& name) { return std::filesystem::path(data_path("fuels/" + name)).lexically_normal().generic_string(); }

// A minimal vehicle: one wheel, one inline engine, one tank, one fluid link
// (the loader validates fluid links before any graph check).
nlohmann::json minimal_vehicle(bool engine_has_fuels) {
    nlohmann::json e;
    e["id"] = "engine";
    e["type"] = "engine";
    e["kind"] = "torque_map";
    e["wot_torque_nm_vs_rpm"] = {{"x", {500.0, 7000.0}}, {"y", {100.0, 100.0}}};
    e["motoring_torque_nm_vs_rpm"] = {{"x", {0.0, 7000.0}}, {"y", {-20.0, -20.0}}};
    e["inertia_kgm2"] = 0.15;
    e["breakaway_torque_nm"] = 20.0;
    if (engine_has_fuels) {
        e["ignition"] = "spark";
        e["fuels"] = {fuel_abs("e5.json")};
    }
    nlohmann::json t = {{"id", "tank"}, {"type", "fuel_tank"}, {"fuel", fuel_abs("e5.json")}, {"capacity_kg", 40.0}, {"initial_kg", 30.0}};
    nlohmann::json v;
    v["format"] = "physics_sim.vehicle/2";
    v["name"] = "mini";
    v["source"] = "test fixture, not a real vehicle (test_fuel_tank.cpp)";
    v["wheels"] = nlohmann::json::array({{{"name", "FL"}, {"tyre", {{"source", "test fixture"}}}}});
    v["powertrain"] = {{"components", nlohmann::json::array({e, t})}, {"fluid", nlohmann::json::array({{"tank.out", "engine.fuel"}})}};
    return v;
}

} // namespace t7

TEST_CASE("C6 T7: the loader rejects every malformed fuel_tank / fluid link with its exact message", "[unit][drivetrain][fuel][io]") {
    using namespace t7;
    const nlohmann::json base = absolute_fixture(tank_sedan_path());
    const std::string tctx = "vehicle 'tank_sedan': powertrain.components['tank']: ";
    const std::string lctx = "vehicle 'tank_sedan': powertrain.fluid[0]: ";

    SECTION("positive controls") {
        expect_loads(base);                                    // crude no-map engine + E5 tank
        expect_loads(absolute_fixture(tank_sedan_e10_path())); // fuels[1]
        expect_loads(absolute_fixture(tank_map_sedan_path())); // fuel-map engine + gasoline tank
        nlohmann::json v = base;
        // An equivalent spelling of the same file is the same fuel.
        component(v, "tank")["fuel"] = std::filesystem::path(data_path("fuels/../fuels/e5.json")).generic_string();
        expect_loads(v);
        expect_loads(minimal_vehicle(true));
        const TempJsonFile f(base);
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        REQUIRE(d.powertrain.fluid.size() == 1);
        CHECK(d.powertrain.fluid[0].from.component == "tank");
        CHECK(d.powertrain.fluid[0].from.port == "out");
        CHECK(d.powertrain.fluid[0].to.component == "engine");
        CHECK(d.powertrain.fluid[0].to.port == "fuel");
        const FuelTankDesc& t = component_of<FuelTankDesc>(d);
        CHECK(t.capacity_kg == 40.0);
        CHECK(t.initial_kg == 30.0);
        CHECK(t.fuel.name == io::load_fuel_json(data_path("fuels/e5.json")).name);
    }
    SECTION("fuel_tank fields") {
        nlohmann::json v = base;
        component(v, "tank")["ref"] = "x.json";
        expect_rejected(v, tctx + "'fuel_tank' components are always inline (no 'ref')");
        v = base;
        component(v, "tank").erase("fuel");
        expect_rejected(v, tctx + "fuel_tank 'fuel' is required and must be a non-empty path string");
        v = base;
        component(v, "tank")["fuel"] = "";
        expect_rejected(v, tctx + "fuel_tank 'fuel' is required and must be a non-empty path string");
        v = base;
        component(v, "tank")["fuel"] = 5;
        expect_rejected(v, tctx + "fuel_tank 'fuel' is required and must be a non-empty path string");
        v = base;
        component(v, "tank").erase("capacity_kg");
        expect_rejected(v, tctx + "fuel_tank 'capacity_kg' is required and must be a number");
        v = base;
        component(v, "tank")["initial_kg"] = "30";
        expect_rejected(v, tctx + "fuel_tank 'initial_kg' is required and must be a number");
        v = base;
        component(v, "tank")["capacity_kg"] = -1.0;
        expect_rejected(v, tctx + "fuel_tank 'capacity_kg' must not be negative, got -1.000000");
        v = base;
        component(v, "tank")["initial_kg"] = -0.5;
        expect_rejected(v, tctx + "fuel_tank 'initial_kg' must not be negative, got -0.500000");
        v = base;
        component(v, "tank")["capacity_kg"] = 0.0;
        component(v, "tank")["initial_kg"] = 0.0;
        expect_rejected(v, tctx + "fuel_tank 'capacity_kg' must be > 0");
        v = base;
        component(v, "tank")["initial_kg"] = 40.5;
        expect_rejected(v, tctx + "fuel_tank 'initial_kg' (40.500000) must not exceed 'capacity_kg' (40.000000)");
        // A non-finite literal survives is_number() but not isfinite() (nlohmann
        // cannot emit one, so the text is patched - see test_engine_fuel_map.cpp's F5).
        std::string text = base.dump(2);
        const std::string needle = "\"capacity_kg\": 40.0";
        const auto pos = text.find(needle);
        REQUIRE(pos != std::string::npos);
        text.replace(pos, needle.size(), "\"capacity_kg\": 1e400");
        const std::filesystem::path raw = std::filesystem::temp_directory_path() / "ps_test_fuel_tank_raw_1e400.json";
        {
            std::ofstream out(raw);
            out << text;
        }
        // nlohmann v3.12 refuses the literal itself (out_of_range.406, "number
        // overflow"), so the loader's own isfinite() check is a backstop no
        // JSON text can reach; pinned here so a parser change is noticed.
        REQUIRE_THROWS_WITH(io::load_vehicle_json(raw.string()), ContainsSubstring("number overflow parsing '1e400'"));
        std::error_code ec;
        std::filesystem::remove(raw, ec);
    }
    SECTION("a tank needs a consumer") {
        nlohmann::json v = base;
        v["powertrain"].erase("fluid");
        expect_rejected(v, "vehicle 'tank_sedan': powertrain.components['tank']: fuel_tank has no consumer (no powertrain.fluid link from \"tank.out\")");
        v["powertrain"]["fluid"] = nlohmann::json::array();
        expect_rejected(v, "fuel_tank has no consumer");
    }
    SECTION("fluid link shape and ports") {
        nlohmann::json v = base;
        v["powertrain"]["fluid"] = "tank.out";
        expect_rejected(v, "vehicle 'tank_sedan': powertrain.fluid must be an array of [\"<tank>.out\", \"<engine>.fuel\"] pairs");
        const std::string pair_msg = "must be a [\"<tank>.out\", \"<engine>.fuel\"] pair of port strings";
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out"})});
        expect_rejected(v, lctx + pair_msg);
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", 3})});
        expect_rejected(v, lctx + pair_msg);
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tankout", "engine.fuel"})});
        expect_rejected(v, lctx + "bad port ref \"tankout\" - " + pair_msg);
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", "engine."})});
        expect_rejected(v, lctx + "bad port ref \"engine.\" - " + pair_msg);
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"nosuch.out", "engine.fuel"})});
        expect_rejected(v, lctx + "unknown component 'nosuch'");
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"clutch.out", "engine.fuel"})});
        expect_rejected(v, lctx + "'clutch' is not a fuel_tank (a fluid link must start at \"<tank>.out\")");
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.in", "engine.fuel"})});
        expect_rejected(v, lctx + "unknown port \"tank.in\" (a fuel_tank's only fluid port is \"out\")");
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", "nosuch.fuel"})});
        expect_rejected(v, lctx + "unknown component 'nosuch'");
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", "gearbox.fuel"})});
        expect_rejected(v, lctx + "'gearbox' is not an engine (a fluid link must end at \"<engine>.fuel\")");
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", "engine.crank"})});
        expect_rejected(v, lctx + "unknown port \"engine.crank\" (an engine's only fluid port is \"fuel\")");
    }
    SECTION("duplicate links and two tanks") {
        nlohmann::json v = base;
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", "engine.fuel"}),
                                                          nlohmann::json::array({"tank.out", "engine.fuel"})});
        expect_rejected(v, "vehicle 'tank_sedan': powertrain.fluid[1]: duplicate fluid link \"tank.out\" -> \"engine.fuel\"");
        v = base;
        nlohmann::json tank2 = component(v, "tank");
        tank2["id"] = "tank2";
        v["powertrain"]["components"].push_back(tank2);
        v["powertrain"]["fluid"] = nlohmann::json::array({nlohmann::json::array({"tank.out", "engine.fuel"}),
                                                          nlohmann::json::array({"tank2.out", "engine.fuel"})});
        expect_rejected(v, "vehicle 'tank_sedan': powertrain.fluid[1]: engine 'engine' fuel port is fed by two tanks ('tank' and 'tank2')");
    }
    SECTION("the tank's fuel must be one of the engine's fuels") {
        nlohmann::json v = base;
        component(v, "tank")["fuel"] = fuel_abs("gasoline.json");
        expect_rejected(v, lctx + "tank fuel \"" + fuel_abs("gasoline.json") + "\" is not in engine \"engine\"'s fuels list");
        // A byte-identical COPY of e5.json elsewhere is a different fuel file.
        const std::filesystem::path copy = std::filesystem::temp_directory_path() / "ps_test_fuel_tank_e5_copy.json";
        std::filesystem::copy_file(data_path("fuels/e5.json"), copy, std::filesystem::copy_options::overwrite_existing);
        v = base;
        component(v, "tank")["fuel"] = copy.generic_string();
        expect_rejected(v, "is not in engine \"engine\"'s fuels list");
        std::error_code ec;
        std::filesystem::remove(copy, ec);
        // The fuel-map engine burns gasoline only.
        v = absolute_fixture(tank_map_sedan_path());
        component(v, "tank")["fuel"] = fuel_abs("e5.json");
        expect_rejected(v, "vehicle 'tank_fuel_map_sedan': powertrain.fluid[0]: tank fuel \"" + fuel_abs("e5.json") +
                               "\" is not in engine \"engine\"'s fuels list");
    }
    SECTION("an engine without any fuel model cannot be fed") {
        expect_rejected(minimal_vehicle(false), "vehicle 'mini': powertrain.fluid[0]: engine 'engine' has no fuel model (no 'fuels', 'fuel' or "
                                                "'fuel_map') - a fluid link cannot feed it");
    }
}

TEST_CASE("C6 T7b: the Powertrain constructor backstops a malformed code-built fuel_tank desc", "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc good = io::load_vehicle_json(tank_sedan_path());
    auto expect_create_throws = [](const vehicle::VehicleDesc& d, const std::string& message) {
        WorldConfig cfg;
        cfg.job_workers = 1;
        World w(cfg);
        BodyDesc chassis;
        chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
        chassis.motion = BodyMotionType::Dynamic;
        chassis.mass = 1500.0;
        chassis.pose.position = Vec3{0.0, 0.0, 0.6};
        const BodyId c = w.create_body(chassis);
        REQUIRE_THROWS_WITH(w.create_vehicle(d, c), ContainsSubstring(message));
    };
    vehicle::VehicleDesc d = good;
    d.powertrain.fluid.clear();
    expect_create_throws(d, "fuel_tank 'tank' has no consumer");
    d = good;
    tank_of(d).fuel = io::load_fuel_json(data_path("fuels/gasoline.json"));
    expect_create_throws(d, "is not in engine 'engine''s fuels list");
    d = good;
    d.powertrain.fluid[0].to.port = "crank";
    expect_create_throws(d, "fluid link target 'engine.crank' is not an engine's \"fuel\" port");
    d = good;
    d.powertrain.fluid[0].from.component = "clutch";
    expect_create_throws(d, "fluid link source 'clutch.out' is not a fuel_tank's \"out\" port");
    d = good;
    d.powertrain.fluid.push_back(d.powertrain.fluid[0]);
    expect_create_throws(d, "fuel port is fed by more than one fluid link");
    d = good;
    for (auto& c : d.powertrain.components) {
        if (auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            e->fuels.clear();
            e->has_fuel_map = false;
        }
    }
    expect_create_throws(d, "engine 'engine' has no fuel model - fuel_tank 'tank' cannot feed it");
    d = good;
    tank_of(d).initial_kg = 41.0;
    expect_create_throws(d, "fuel_tank 'tank' needs finite capacity_kg > 0 and 0 <= initial_kg <= capacity_kg");
    d = good;
    tank_of(d).capacity_kg = std::numeric_limits<real>::infinity();
    expect_create_throws(d, "needs finite capacity_kg > 0");
}

// ---------------------------------------------------------------------------
// T8: the mechanical ledger still closes (balance_error <= 5e-8 relative per
// tick) while the tank runs dry mid-launch - starvation scales tau_comb and
// its share of d together, so the engine's booked work stays consistent.
// ---------------------------------------------------------------------------
namespace {
void run_balance_starvation(const std::string& fixture, const char* label) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(fixture);
    tank_of(desc).initial_kg = 0.004;
    Rig rig = make_rig(desc, 240.0, 960.0, 1, 1, /*auto_clutch_launch=*/true);
    World& w = *rig.world;
    for (int t = 0; t < 240; ++t) w.step(); // settle 1 s (idle in gear 1)
    REQUIRE(w.fuel_tank_mass(rig.vehicle) > 0.0);
    w.set_control("throttle", 1.0);
    real worst = 0.0;
    int empty_tick = -1;
    for (int t = 0; t < 4 * 240; ++t) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) {
            const real ratio = std::abs(e.balance_error) / denom;
            worst = std::max(worst, ratio);
            CHECK(ratio <= 5e-8);
        }
        if (empty_tick < 0 && w.fuel_tank_mass(rig.vehicle) == 0.0) empty_tick = t;
    }
    WARN(label << ": tank empty at WOT tick " << empty_tick << ", worst |balance_error|/denom " << worst << ", final engine state "
               << static_cast<int>(w.powertrain_state(rig.vehicle).engine_state));
    CHECK(empty_tick > 0);
    CHECK(empty_tick < 3 * 240); // starvation happens inside the checked window, with ticks to spare after it
}
} // namespace

TEST_CASE("C6 T8: balance_error stays within 5e-8 relative per tick while the tank runs dry (crude and map engines)",
          "[unit][drivetrain][fuel]") {
    run_balance_starvation(tank_sedan_path(), "T8 crude");
    run_balance_starvation(tank_map_sedan_path(), "T8 map");
}

// ---------------------------------------------------------------------------
// T9: determinism and allocation. 1 vs 10 workers with a tank that runs dry
// and is refilled mid-run hash identically; the tank content is hashed
// (30 vs 29.5 kg initial differ); debug_alloc armed across the emptying and a
// set_fuel_tank_mass sees no allocation of our own.
// ---------------------------------------------------------------------------
namespace {
std::uint64_t run_t9(int workers, bool arm_alloc, real* tank_out) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(tank_sedan_path());
    tank_of(desc).initial_kg = 0.003;
    Rig rig = make_rig(desc, 240.0, 960.0, 1, workers, /*auto_clutch_launch=*/true);
    World& w = *rig.world;
    for (int t = 0; t < 240; ++t) w.step(); // warm-up + settle
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 480; ++t) w.step();
    const bool emptied = w.fuel_tank_mass(rig.vehicle) == 0.0;
    w.set_fuel_tank_mass(rig.vehicle, 0.5);
    w.set_control("throttle", 0.0);
    w.set_control("brake", 1.0);
    w.set_control("starter", 1.0); // restart the dry-stalled engine on the new fuel
    for (int t = 0; t < 240; ++t) w.step();
    if (arm) {
        da::set_armed(false);
        const auto own = da::own_violation_count();
        const auto ext = da::external_violation_count();
        WARN("T9 debug_alloc (" << workers << " workers): own " << own << ", external (Jolt) " << ext);
        CHECK(own == 0);
    }
    CHECK(emptied);
    *tank_out = w.fuel_tank_mass(rig.vehicle);
    return w.state_hash();
}
} // namespace

TEST_CASE("C6 T9: a tank run dry and refilled hashes identically on 1 and 10 workers, allocation-free", "[unit][drivetrain][fuel][jobs][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    real tank1 = 0.0, tank10 = 0.0;
    const std::uint64_t h1 = run_t9(1, true, &tank1);
    const std::uint64_t h10 = run_t9(10, true, &tank10);
    CHECK(h1 == h10);
    CHECK(tank1 == tank10);
    CHECK(tank1 > 0.0);
    CHECK(tank1 < 0.5);

    // The tank content is part of the state hash.
    vehicle::VehicleDesc a = io::load_vehicle_json(tank_sedan_path());
    vehicle::VehicleDesc b = a;
    tank_of(b).initial_kg = 29.5;
    Rig ra = make_rig(a, 240.0, 960.0, 0);
    Rig rb = make_rig(b, 240.0, 960.0, 0);
    for (int t = 0; t < 10; ++t) {
        ra.world->step();
        rb.world->step();
    }
    CHECK(ra.world->powertrain_state(ra.vehicle).engine_rpm == rb.world->powertrain_state(rb.vehicle).engine_rpm);
    CHECK(ra.world->state_hash() != rb.world->state_hash());
}

// ---------------------------------------------------------------------------
// T10 (review S1): starvation scales EVERY combustion term of the engine's
// linearisation d by s, not only tau_comb. White-box EngineTorqueMap::
// pre_solve on the production engine (to_engine_desc over the loaded
// TorqueMapEngineDesc), one fresh engine per call, with the tank able to
// deliver +inf (s = 1), 0 (s = 0) and half the substep demand (s = 1/2).
// Three regimes, each exercising one of pre_solve's three s-scaled d terms:
//   (a) i4_2l_na at WOT, 5250 rpm           - the WOT-slope term
//   (b) i4_2l_na with its limiter turned into a soft cut (6800 rpm, band
//       400) at WOT, 6600 rpm (f_lim = 1/2)  - the soft-cut term
//   (c) tractor diesel, governor lever 1, 2300 rpm (theta_gov = 100/199.98)
//                                            - the governor term
// Expected tau0 and d come from the curves' LITERAL knots (the data files'
// numbers, repeated here), K = 60/(2 pi), c_k = -motoring(0):
//   tau0 = (m + c_k) + s*theta_c*(w - m)
//   d    = -[K*m' + s*theta_c*K*(w' - m') + s*theta_eff*(w - m)*dflim/domega
//            + s*f_lim*(w - m)*dtheta_gov/domega]
// The demand (crude model, the only model these engines have) is
// h*tau_comb*omega/(eta_i*LHV) with the literal Heywood defaults and LHVs.
// ---------------------------------------------------------------------------
namespace {

constexpr real kK = 9.5492965855137201461; // 60/(2 pi), rpm per rad/s

struct T10Case {
    const char* label = "";
    EngineTorqueMap::Desc desc;
    real rpm = 0.0;
    real m = 0.0, w = 0.0, m_slope = 0.0, w_slope = 0.0; // curves at rpm, N*m and N*m/rpm (literal)
    real c_k = 0.0;                                      // -motoring(0), literal
    real theta_eff = 0.0, f_lim = 0.0, theta_c = 0.0;
    real dflim_domega = 0.0; // soft-cut term, 0 when not in band
    real dgov_domega = 0.0;  // governor term, 0 when not engaged
    real eta_lhv = 0.0;      // eta_i * LHV, J/kg
};

void run_t10(const T10Case& c) {
    const real h = 1.0 / 960.0;
    const real omega = c.rpm / kK;
    const real tau_comb_demand = c.theta_c * (c.w - c.m);
    const real m_demand = h * tau_comb_demand * omega / c.eta_lhv;
    REQUIRE(m_demand > 0.0);

    struct Supply {
        real available_kg;
        real s;
    };
    const Supply supplies[] = {
        {std::numeric_limits<real>::infinity(), 1.0},
        {0.0, 0.0},
        {0.5 * m_demand, 0.5},
    };
    for (const Supply& sup : supplies) {
        EngineTorqueMap eng(c.desc);
        Sensors sens;
        sens.omega = omega;
        sens.ignition = 1.0;
        sens.starter = 0.0;
        sens.clutch_engaged_fraction = 1.0;
        sens.gearbox_in_neutral = true;
        sens.fuel_available_kg = sup.available_kg;
        ThrottleCommand thr;
        thr.command = 1.0;
        const PreSolveResult r = eng.pre_solve(sens, thr, TcuRequest{}, h);

        const real s = sup.s;
        const real tau0_exp = (c.m + c.c_k) + s * tau_comb_demand;
        const real dtau = kK * c.m_slope + s * c.theta_c * kK * (c.w_slope - c.m_slope) + s * c.theta_eff * (c.w - c.m) * c.dflim_domega +
                          s * c.f_lim * (c.w - c.m) * c.dgov_domega;
        const real d_exp = std::max(-dtau, real(0.0));
        INFO(c.label << ": available " << sup.available_kg << " kg (demand " << m_demand << " kg), s " << s << ": tau0 " << r.tau0
                     << " vs " << tau0_exp << ", d " << r.d << " vs " << d_exp);
        REQUIRE(r.state == EngineState::Running);
        CHECK(r.fuel_supply_fraction == Catch::Approx(s).margin(1e-12));
        CHECK(r.theta_c == Catch::Approx(s * c.theta_c).margin(1e-12));
        CHECK(r.tau0 == Catch::Approx(tau0_exp).epsilon(1e-10));
        CHECK(r.d == Catch::Approx(d_exp).epsilon(1e-10));
        WARN(c.label << ": s " << s << " tau0 " << r.tau0 << " (exp " << tau0_exp << "), d " << r.d << " (exp " << d_exp << ")");
    }
}

} // namespace

TEST_CASE("C6 T10: starvation scales tau_comb and every combustion d term by s (white-box pre_solve)", "[unit][drivetrain][fuel]") {
    const real kEtaLhvE5 = 0.325 * 43089334.0;    // spark default * LHV_E5 (fuel.md)
    const real kEtaLhvDiesel = 0.48 * 42000000.0; // compression default * LHV_diesel (data/fuels/diesel.json)

    vehicle::VehicleDesc sedan = io::load_vehicle_json(tank_sedan_path());
    const TorqueMapEngineDesc& i4 = component_of<TorqueMapEngineDesc>(sedan);
    const FuelTankDesc& tank = tank_of(sedan);
    const vehicle::VehicleDesc tractor = io::load_vehicle_json(data_path("vehicles/tractor.json"));
    const TorqueMapEngineDesc& diesel = component_of<TorqueMapEngineDesc>(tractor);
    REQUIRE(i4.limiter.type == LimiterType::FuelCut);
    REQUIRE(diesel.has_governor);

    SECTION("(a) WOT slope: i4_2l_na at 5250 rpm") {
        T10Case c;
        c.label = "(a) i4 WOT 5250 rpm";
        c.desc = to_engine_desc(i4, &tank);
        c.rpm = 5250.0;
        c.w = 195.0;  // 198 + 0.5*(192 - 198)
        c.m = -63.75; // -61 + 0.25*(-72 + 61)
        c.w_slope = -6.0 / 500.0;
        c.m_slope = -11.0 / 1000.0;
        c.c_k = 15.0;
        c.theta_eff = 1.0;
        c.f_lim = 1.0;
        c.theta_c = 1.0;
        c.eta_lhv = kEtaLhvE5;
        run_t10(c);
    }
    SECTION("(b) soft cut: i4_2l_na, limiter soft_cut 6800/400, WOT at 6600 rpm") {
        TorqueMapEngineDesc e = i4;
        e.limiter.type = LimiterType::SoftCut;
        e.limiter.rpm = 6800.0;
        e.limiter.band_rpm = 400.0;
        T10Case c;
        c.label = "(b) i4 soft cut 6600 rpm";
        c.desc = to_engine_desc(e, &tank);
        c.rpm = 6600.0;
        c.w = 160.8; // 166 + 0.2*(140 - 166)
        c.m = -79.8; // -72 + 0.6*(-85 + 72)
        c.w_slope = -26.0 / 500.0;
        c.m_slope = -13.0 / 1000.0;
        c.c_k = 15.0;
        c.theta_eff = 1.0;
        c.f_lim = 0.5; // (6800 - 6600)/400
        c.theta_c = 0.5;
        c.dflim_domega = -kK / 400.0;
        c.eta_lhv = kEtaLhvE5;
        run_t10(c);
    }
    SECTION("(c) governor: tractor diesel, lever 1 (n_nl 2400), 2300 rpm") {
        const real band = 0.0909 * 2200.0; // droop * rated
        const real theta_gov = (2400.0 - 2300.0) / band;
        T10Case c;
        c.label = "(c) diesel governor 2300 rpm";
        c.desc = to_engine_desc(diesel);
        c.rpm = 2300.0;
        c.w = 420.0; // 440 + 0.5*(400 - 440)
        c.m = -76.0; // -70 + 0.75*(-78 + 70)
        c.w_slope = -40.0 / 200.0;
        c.m_slope = -8.0 / 400.0;
        c.c_k = 40.0;
        c.theta_eff = theta_gov;
        c.f_lim = 1.0;
        c.theta_c = theta_gov;
        c.dgov_domega = -kK / band;
        c.eta_lhv = kEtaLhvDiesel;
        run_t10(c);
    }
}

// ---------------------------------------------------------------------------
// C7 review F5: tests/support/fixtures/vehicles/tractor_fuel.json (the tractor + a full 182.19 kg
// diesel tank, see that file's own "source" field) is currently loaded by no
// test/script/adapter - a smoke test only, not a physics acceptance test:
// the file loads via the production path, the crude no-map diesel engine
// (indicated_efficiency defaults to the ignition="compression" value 0.48)
// idles in neutral for a few seconds without stalling, the tank measurably
// decreases, balance_error stays within the project-wide 5e-8 relative bound
// every tick, and the fuel actually burnt is confirmed to be diesel by
// cross-checking fuel_energy_in/fuel_mass against data/fuels/diesel.json's
// own literal lhv_j_kg (42,000,000 J/kg) to a tight relative tolerance - the
// crude model books fuel_energy_in = fuel_mass_delta*LHV every substep, so a
// wrong fuel (e.g. gasoline's ~44e6 J/kg) would fail this ratio check
// immediately.
// ---------------------------------------------------------------------------
TEST_CASE("C7 F5: tractor_fuel.json loads, idles without stalling and burns down its diesel tank", "[unit][drivetrain][fuel][vehicle]") {
    vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/tractor_fuel.json");
    const real capacity_kg = tank_of(desc).capacity_kg;
    REQUIRE(capacity_kg == Catch::Approx(182.19).margin(1e-9));

    Rig rig = make_rig(desc, 240.0, 960.0, /*gear=*/0, /*workers=*/1, /*auto_clutch_launch=*/false);
    World& w = *rig.world;
    const VehicleId v = rig.vehicle;

    // Settle 1 s first (the same pattern run_balance_starvation() above uses):
    // right after reset_vehicle the idle PI/energy terms are momentarily near
    // zero, which would otherwise make the balance-error/denom RATIO checked
    // below spuriously huge for a tick or two even though the absolute
    // residual is tiny - not checked here, exactly like every other
    // balance_error-ratio test in this file.
    for (int t = 0; t < 240; ++t) {
        w.step();
        REQUIRE(w.powertrain_state(v).engine_state == EngineState::Running);
    }
    const real initial_tank_kg = w.fuel_tank_mass(v);
    REQUIRE(initial_tank_kg < 182.19); // already idled for 1 s, so a hair below the full 182.19 kg fixture literal
    REQUIRE(initial_tank_kg == Catch::Approx(182.19).margin(1e-3));

    real worst_balance_ratio = 0.0;
    for (int t = 0; t < 5 * 240; ++t) { // 5 more seconds idling in neutral
        w.step();
        REQUIRE(w.powertrain_state(v).engine_state == EngineState::Running);
        const PowertrainEnergy& e = w.powertrain_energy(v);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) {
            const real ratio = std::abs(e.balance_error) / denom;
            worst_balance_ratio = std::max(worst_balance_ratio, ratio);
            CHECK(ratio <= 5e-8);
        }
    }

    const real final_tank_kg = w.fuel_tank_mass(v);
    const PowertrainEnergy& e = w.powertrain_energy(v);
    WARN("F5: tank " << initial_tank_kg << " -> " << final_tank_kg << " kg over 5 s idle, fuel_mass=" << e.fuel_mass
                      << " kg, fuel_energy_in=" << e.fuel_energy_in << " J, worst |balance_error|/denom " << worst_balance_ratio);

    CHECK(final_tank_kg < initial_tank_kg); // measurably decreased
    CHECK(e.fuel_mass > 0.0);
    REQUIRE(e.fuel_mass > 1e-12); // non-vacuous denominator for the LHV ratio check below

    // Diesel's own literal LHV (data/fuels/diesel.json), not some other fuel's.
    constexpr real kDieselLhvJKg = 42000000.0;
    const real measured_lhv = e.fuel_energy_in / e.fuel_mass;
    CHECK(measured_lhv == Catch::Approx(kDieselLhvJKg).epsilon(1e-6));
}
