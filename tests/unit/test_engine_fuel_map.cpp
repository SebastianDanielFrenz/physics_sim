// tests/unit/test_engine_fuel_map.cpp -- PHYS-033 P4 I7a C5: the graph
// engine's optional fuel map (rpm x load -> g/s) and the PowertrainEnergy
// fuel sub-ledger (ps/drivetrain/energy_ledger.h's fuel_mass/fuel_energy_in/
// indicated_work/engine_heat/fuel_efficiency_violations). No tank exists yet
// (C6) - fuel_external_added stays 0 everywhere below.
//
// F2/F3 drive core/src/drivetrain/engine_torque_map.h's EngineTorqueMap
// directly (core/src is on this target's include path, like
// test_throttle_control.cpp/test_powertrain_engine.cpp) with a small
// synthetic engine: FLAT wot/motoring curves (100 / -20 N*m everywhere) make
// tau_smooth_ = s*(m_rpm + c_k_) exactly 0 at every rpm (c_k_ = -motoring(0)
// = 20 cancels the constant -20 exactly), so frac_comb = tau_comb_/tau0_ is
// exactly 1 whenever combustion fires (no starter) and exactly 0 otherwise -
// see engine_torque_map.cpp's post_solve.
//
// F6/F9 (and, since the review fixes below, F4 too) drive a real World +
// Powertrain (tests/support/fixtures/vehicle_fuel_map_sedan.json, a
// car_sedan.json clone with the engine inlined and a synthetic fuel map/fuel
// reference added) - F6/F9 to prove the fuel sub-ledger is entirely a
// spectator (no state_hash bit moves, no CSV column is added to a vehicle
// with no fuel map, balance_error is undisturbed), F4 so the gating/
// summation Powertrain::post_solve_components itself performs is actually
// exercised, not just hand-replicated at the engine level.
//
// Post-review additions (owner ruling: indicated_work is booked for EVERY
// torque_map engine, fuel map or not - see energy_ledger.h/engine.h's own
// comments): the flat synthetic engine above makes tau_smooth_ exactly 0 at
// every rpm, which makes the OLD indicated_work_delta formula's tau0_ -> 0
// singularity structurally unreachable - exactly why that defect escaped
// the original F4's coverage (a hand-replicated ledger over that same flat
// engine, since replaced by the real-Powertrain version above). The
// "car_sedan without a fuel map" test below uses car_sedan's own REAL,
// non-flat i4_2l_na.json engine curves (loaded via ps_test::
// load_sedan_desc(), never a data file mutated): car_sedan_engine_public_
// desc()/to_engine_desc_for_test() build a bare EngineTorqueMap from them
// (to_engine_desc_for_test duplicates core/src/drivetrain/powertrain.cpp's
// own internal, unexported to_engine_desc field-for-field, since that
// function is deliberately anonymous-namespace). The "Fix 1" DFCO test
// further down deliberately does NOT use a bare EngineTorqueMap - see that
// test's own header comment for why a bare engine cannot discriminate the
// defect it targets, and drives the real WorldRig/vehicle_fuel_map_sedan.json
// Powertrain instead.

#include "drivetrain/engine_torque_map.h"
#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/io/scenario.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
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
constexpr real kSubstepH = 1.0 / 960.0;

real rpm_to_omega(real rpm) { return rpm * kRpmToRadPerSec; }

// ---------------------------------------------------------------------------
// F2/F3/F4: a small, fully hand-analysed synthetic engine (see file header).
// ---------------------------------------------------------------------------
EngineTorqueMap::Desc synthetic_engine_desc(bool with_fuel_map = true) {
    EngineTorqueMap::Desc d;
    d.wot_torque_nm_vs_rpm.x = {500.0, 7000.0};
    d.wot_torque_nm_vs_rpm.y = {100.0, 100.0};
    d.motoring_torque_nm_vs_rpm.x = {0.0, 7000.0};
    d.motoring_torque_nm_vs_rpm.y = {-20.0, -20.0};
    d.inertia_kgm2 = 0.15;
    d.breakaway_torque_nm = 20.0; // == -motoring(0) exactly (constructor allows equality)
    // idle_rpm/stall_rpm/fire_rpm/start_complete_rpm/dfco_*/limiter/idle/starter
    // all keep their EngineTorqueMap::Desc defaults (800/400/250/550,
    // 1300/1000, FuelCut@6800, kp 0.0018/ki 0.009/max_throttle 0.3,
    // 140 Nm/400 rpm) - already consistent with the flat curves above.
    d.initial_state = EngineTorqueMap::InitialState::Running;
    if (with_fuel_map) {
        d.has_fuel_map = true;
        d.fuel.name = "test fuel";
        d.fuel.phase = FuelPhase::Liquid;
        d.fuel.lhv_j_kg = 44.0e6;
        d.fuel.afr_stoich = 14.5;
        d.fuel.density_kg_m3 = 750.0;
        d.fuel.source = "synthetic (test_engine_fuel_map.cpp)";
        d.fuel_map.x = {500.0, 7000.0};      // rpm
        d.fuel_map.y = {0.0, 1.0};           // load (theta_c)
        d.fuel_map.values = {0.0, 3.0, 0.0, 12.0}; // row-major [ix*y.size()+iy]: rpm=500 -> {0,3}, rpm=7000 -> {0,12}
    }
    return d;
}

Sensors running_sensors(real rpm) {
    Sensors s;
    s.omega = rpm_to_omega(rpm);
    s.ignition = 1.0;
    s.starter = 0.0;
    s.clutch_engaged_fraction = 1.0;
    s.gearbox_in_neutral = true;
    return s;
}

ThrottleCommand throttle_of(real command, real floor = 0.0, real ceiling = 1.0) {
    ThrottleCommand t;
    t.command = command;
    t.floor = floor;
    t.ceiling = ceiling;
    return t;
}

TcuRequest no_ignition_cut() { return TcuRequest{false}; }

Sensors sensors_at_omega(real omega) {
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.starter = 0.0;
    s.clutch_engaged_fraction = 1.0;
    s.gearbox_in_neutral = true;
    return s;
}

// ---------------------------------------------------------------------------
// Post-review helpers: car_sedan's own REAL, non-flat engine curves (see the
// file header comment above) - needed because the flat synthetic engine
// above makes tau_smooth_ == 0 identically, which structurally hides the
// tau0_ -> 0 singularity fix 1 targets.
// ---------------------------------------------------------------------------

// Extracts the real i4_2l_na.json engine descriptor car_sedan.json ships
// with, via ps_test::load_sedan_desc() (no data file read/mutated directly).
TorqueMapEngineDesc car_sedan_engine_public_desc() {
    const vehicle::VehicleDesc desc = ps_test::load_sedan_desc();
    for (const auto& c : desc.powertrain.components) {
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) return *e;
    }
    throw std::runtime_error("car_sedan_engine_public_desc: no TorqueMapEngineDesc component in car_sedan.json");
}

// Mirrors core/src/drivetrain/powertrain.cpp's own to_engine_desc field-for-
// field. Duplicated here rather than exposed from production code, since
// that function is deliberately anonymous-namespace/internal (powertrain.cpp's
// own header comment on why EngineTorqueMap::Desc and TorqueMapEngineDesc are
// named differently) - this test needs to build a bare EngineTorqueMap
// directly from car_sedan's public VehicleDesc, without going through a full
// Powertrain/World for the DFCO test below (F4's real-Powertrain rewrite,
// further down, covers that path instead).
EngineTorqueMap::Desc to_engine_desc_for_test(const TorqueMapEngineDesc& e) {
    EngineTorqueMap::Desc d;
    d.wot_torque_nm_vs_rpm = e.wot_torque_nm_vs_rpm;
    d.motoring_torque_nm_vs_rpm = e.motoring_torque_nm_vs_rpm;
    d.inertia_kgm2 = e.inertia_kgm2;
    d.breakaway_torque_nm = e.breakaway_torque_nm;
    d.idle_rpm = e.idle_rpm;
    d.stall_rpm = e.stall_rpm;
    d.fire_rpm = e.fire_rpm;
    d.start_complete_rpm = e.start_complete_rpm;
    d.dfco_enable_rpm = e.dfco_enable_rpm;
    d.dfco_reenable_rpm = e.dfco_reenable_rpm;
    d.idle.kp_per_rpm = e.idle.kp_per_rpm;
    d.idle.ki_per_rpm_s = e.idle.ki_per_rpm_s;
    d.idle.max_throttle = e.idle.max_throttle;
    d.limiter.type =
        e.limiter.type == LimiterType::FuelCut ? EngineTorqueMap::LimiterType::FuelCut : EngineTorqueMap::LimiterType::SoftCut;
    d.limiter.rpm = e.limiter.rpm;
    d.limiter.hysteresis_rpm = e.limiter.hysteresis_rpm;
    d.limiter.band_rpm = e.limiter.band_rpm;
    d.starter.stall_torque_nm = e.starter.stall_torque_nm;
    d.starter.free_speed_rpm = e.starter.free_speed_rpm;
    d.starter.interlock = e.starter.interlock == StarterDesc::Interlock::None ? EngineTorqueMap::StarterInterlock::None
                                                                               : EngineTorqueMap::StarterInterlock::ClutchOrNeutral;
    d.initial_state =
        e.initial_state == EngineInitialState::Running ? EngineTorqueMap::InitialState::Running : EngineTorqueMap::InitialState::Off;
    d.has_governor = e.has_governor;
    d.governor.high_idle_rpm = e.governor.high_idle_rpm;
    d.governor.rated_rpm = e.governor.rated_rpm;
    d.governor.droop = e.governor.droop;
    d.has_fuel_map = e.has_fuel_map;
    d.fuel = e.fuel;
    d.fuel_map = e.fuel_map;
    return d;
}

// ---------------------------------------------------------------------------
// F2 (review-fixed): the original version held omega EXACTLY constant with a
// fixed nonzero throttle command, which made theta_c == command every
// substep by construction (nothing else could move it) and pre-solve rpm ==
// post-solve rpm trivially (omega never moved) - it could not tell "fuel is
// drawn at theta_c" from "fuel is drawn at the raw command", nor "at
// pre-solve rpm" from "at post-solve rpm", apart. This version commands a
// constant throttle of 0.0 from a BELOW-idle rpm, so the idle PI (not the
// zero pedal) is what drives theta_c > 0 - a direct, cheap precondition that
// theta_c != command - and free-integrates omega via an explicit dyno-less
// implicit-Euler step (same construction as the DFCO/no-map tests below) so
// omega genuinely changes substep to substep, making pre-solve rpm diverge
// from post-solve rpm. fuel_mass_sum is checked against an INDEPENDENT
// per-substep recomputation from pre_solve's own outputs
// (fuel_map.value(pre.rpm, pre.theta_c)), so a production bug that instead
// read post-solve rpm or the raw command would be caught - see this fix's
// own discrimination demo (temporarily swap each in production, report the
// failure, revert).
// ---------------------------------------------------------------------------
TEST_CASE("F2: fuel integral matches the map at pre-solve rpm/theta_c, not post-solve rpm or the raw throttle command",
          "[unit][drivetrain][fuel]") {
    EngineTorqueMap engine(synthetic_engine_desc());
    const real J = engine.desc().inertia_kgm2;
    real omega = rpm_to_omega(700.0); // below idle_rpm (800): the idle PI, not the zero pedal, drives theta_c
    constexpr int kSubsteps = 500;

    real fuel_mass_sum = 0.0;
    real expected_mass_sum = 0.0; // independently re-derived from the SAME pre_solve outputs each substep
    bool saw_theta_c_ne_command = false;
    const real omega_start = omega;
    bool saw_omega_change = false;

    for (int i = 0; i < kSubsteps; ++i) {
        const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
        const PostSolveResult post = engine.post_solve(omega_new, kSubstepH);

        if (pre.theta_c != 0.0) saw_theta_c_ne_command = true; // command is always 0.0 here
        fuel_mass_sum += post.fuel_mass_delta;
        expected_mass_sum += kSubstepH * (engine.desc().fuel_map.value(pre.rpm, pre.theta_c) / 1000.0);

        omega = omega_new;
        if (std::abs(omega - omega_start) > rpm_to_omega(1.0)) saw_omega_change = true;
    }

    REQUIRE(saw_theta_c_ne_command); // precondition: theta_c really differs from the commanded 0.0 (idle PI)
    REQUIRE(saw_omega_change);       // precondition: omega really varies across the run, not held constant

    CHECK(fuel_mass_sum > 0.0);
    const real rel_err = std::abs(fuel_mass_sum - expected_mass_sum) / std::max(expected_mass_sum, 1e-12);
    CHECK(rel_err <= 1e-12);
}

// ---------------------------------------------------------------------------
// F3: zero fuel in every non-combusting/cut state, nonzero at idle. Every
// case is a single pre_solve() call (fuel_flow_g_s is a PreSolveResult
// field, computed before post_solve ever runs).
// ---------------------------------------------------------------------------
TEST_CASE("F3: fuel flow is exactly zero whenever combustion cannot be firing, nonzero at idle", "[unit][drivetrain][fuel]") {
    SECTION("Off (ignition <= 0.5) even if previously Running") {
        EngineTorqueMap engine(synthetic_engine_desc());
        Sensors s = running_sensors(3000.0);
        s.ignition = 0.0;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Off);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Stalled, starter not pressed, at rest") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Stalled);
        Sensors s = running_sensors(0.0);
        s.starter = 0.0;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Stalled);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Cranking, below fire_rpm (250)") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Stalled);
        Sensors s = running_sensors(100.0); // 100 rpm < fire_rpm 250
        s.starter = 1.0;
        s.gearbox_in_neutral = true;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Cranking);
        CHECK(r.starter_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("DFCO (deceleration fuel cut-off): Running, high rpm, throttle 0") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Running);
        // rpm (2000) > dfco_enable_rpm (1300), throttle 0 < 0.01 -> latches
        // dfco_active_ true within this very call (used later in the same call).
        const PreSolveResult r = engine.pre_solve(running_sensors(2000.0), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.dfco_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Fuel-cut limiter: Running, at/above limiter.rpm (6800), full throttle") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Running);
        const PreSolveResult r = engine.pre_solve(running_sensors(7000.0), throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.limiter_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Ignition cut (upshift cut), contrasted against the same setup without it") {
        EngineTorqueMap engine_cut(synthetic_engine_desc());
        engine_cut.reset(EngineState::Running);
        TcuRequest cut{true};
        const PreSolveResult r_cut = engine_cut.pre_solve(running_sensors(3000.0), throttle_of(1.0), cut, kSubstepH);
        CHECK(r_cut.state == EngineState::Running);
        CHECK(r_cut.fuel_flow_g_s == 0.0);

        EngineTorqueMap engine_no_cut(synthetic_engine_desc());
        engine_no_cut.reset(EngineState::Running);
        const PreSolveResult r_no_cut = engine_no_cut.pre_solve(running_sensors(3000.0), throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r_no_cut.fuel_flow_g_s > 0.0); // same rpm/throttle, no cut -> real fuel flow
    }
    SECTION("Nonzero at idle: Running, foot off pedal, rpm near/below idle_rpm (800)") {
        EngineTorqueMap engine(synthetic_engine_desc());
        engine.reset(EngineState::Running);
        const PreSolveResult r = engine.pre_solve(running_sensors(750.0), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK_FALSE(r.dfco_active);
        CHECK(r.throttle_eff > 0.0); // the idle PI is feeding fuel, not the (zero) pedal
        CHECK(r.fuel_flow_g_s > 0.0);
    }
}

std::string fuel_map_sedan_path() { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicle_fuel_map_sedan.json"; }

// Mirrors test_throttle_control.cpp's WorldRig/make_world_rig exactly (same
// ground/chassis geometry as data/scenarios/vehicle_powertrain.json's own
// bodies) - a fresh copy here rather than a shared header, since that
// helper lives in an anonymous namespace in test_throttle_control.cpp.
// Moved up from below F5 (where F6/F9 still find it, unchanged) so the
// review-fixed F4 below can drive a REAL Powertrain too.
struct WorldRig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

WorldRig make_world_rig(const vehicle::VehicleDesc& desc) {
    WorldRig rig;
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
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
    const BodyId chassis_id = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    rig.vehicle = rig.world->create_vehicle(desc, chassis_id);

    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

// ---------------------------------------------------------------------------
// F4 (review-fixed): the original version hand-replicated Powertrain::
// post_solve_components' exact booking rule (ledger.engine_heat +=
// fuel_energy_in_delta - indicated_work_delta) directly on a bare
// EngineTorqueMap - it could never catch a bug in the GATING or SUMMATION
// that function itself performs (e.g. indicated_work booked only when a map
// is present, or a fuel field left un-gated for a no-map engine). This
// version instead drives a REAL Powertrain (WorldRig/make_world_rig above,
// tests/support/fixtures/vehicle_fuel_map_sedan.json - the same fuel-mapped
// fixture F6/F9 use) through a full World::step WOT-launch-then-lift
// sequence, reading World::powertrain_energy() every TICK (the ledger is a
// per-tick-published cumulative snapshot, not a per-substep one) and
// checking the tick-over-tick engine_heat delta never goes negative.
// ---------------------------------------------------------------------------
TEST_CASE("F4: engine_heat never goes negative (no >100% efficiency) over a WOT launch + overrun, driven through a real Powertrain",
          "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());
    WorldRig rig = make_world_rig(fueled_desc);

    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);

    for (int settle_tick = 0; settle_tick < 240; ++settle_tick) rig.world->step(); // 1 s settle, mirrors F9

    real prev_engine_heat = rig.world->powertrain_energy(rig.vehicle).engine_heat;
    constexpr int kLaunchTicks = 480;  // 2 s WOT at 240 Hz
    constexpr int kOverrunTicks = 480; // 2 s lift
    for (int tick = 0; tick < kLaunchTicks + kOverrunTicks; ++tick) {
        rig.world->set_control("throttle", tick < kLaunchTicks ? 1.0 : 0.0);
        rig.world->step();
        const drivetrain::PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
        const real heat_delta = e.engine_heat - prev_engine_heat;
        INFO("tick " << tick << " heat_delta=" << heat_delta << " engine_heat=" << e.engine_heat);
        CHECK(heat_delta >= -1e-9);
        prev_engine_heat = e.engine_heat;
    }

    const drivetrain::PowertrainEnergy& final_ledger = rig.world->powertrain_energy(rig.vehicle);
    CHECK(final_ledger.fuel_efficiency_violations == 0);
    CHECK(final_ledger.fuel_mass > 0.0);   // non-vacuous: fuel really flowed
    CHECK(final_ledger.engine_heat > 0.0); // non-vacuous: real (imperfect) combustion heat was booked
}

// ---------------------------------------------------------------------------
// Fix 1 (redesigned - see this file's own history/PR notes): a BARE
// EngineTorqueMap cannot discriminate the defect fix 1 targets. In that
// setup omega_new is derived directly from the SAME tau0_/d_ pair
// (omega_new = omega + h*tau0_/(J + h*d_)), so tau_lin = tau0_ -
// d_*(omega_new-omega_old) never actually diverges from tau0_ even as
// tau0_ crosses 0 - confirmed empirically (restoring the old formula on the
// bare-engine version of this test left min_heat_delta at exactly 0 across
// 6000 substeps, zero violations: it could not fail). The old formula's
// singularity (work*tau_comb_/tau0_, tau0_ -> 0 with tau_comb_ and
// domega/dt still nonzero) needs the crank genuinely coupled to something
// OTHER than its own tau0_/d_ - a real Powertrain/ShaftNetwork::solve,
// where omega_new comes from the WHOLE network (engine + clutch + gearbox +
// diff + wheel hub terms from tyre/aero road load), not from the engine's
// isolated ODE alone - so tau_lin can diverge sharply from tau0_ exactly at
// a tau0_ ~= 0 crossing.
//
// Drives the real fuel-mapped fixture (WorldRig/vehicle_fuel_map_sedan.json,
// same fixture F4/F6/F9 use) through an auto-clutch WOT launch in 1st from
// settled rest (assist.auto_clutch=1, the same anti-stall launch law
// test_powertrain_auto_clutch_shifts.cpp/test_powertrain_engine_braking.cpp
// already rely on) to build rpm well above dfco_enable_rpm (1300) with the
// vehicle actually rolling and the clutch fully locked, then lifts to
// throttle 0 and coasts through DFCO -> recovery -> a long idle-speed
// settling tail - real engine braking coupled through the now-moving
// vehicle's own inertia, the tau0_ ~= 0-with-nonzero-domega/dt window fix 1
// targets. An earlier version of this test set o.clutch_locked = true
// directly (an instant clutch dump from a dead stop in 1st) - exactly the
// scenario test_powertrain_engine_braking.cpp's own "stall under load"
// case documents as stalling the engine; diagnostic instrumentation (now
// removed) confirmed this test's engine stalled at tick 0 and stayed at
// 0 rpm for the rest of the run, leaving the DFCO/recovery assertions
// unreachable. Checks World::powertrain_energy's engine_heat over
// TICK-level deltas (the granularity World actually publishes - each tick
// is 4 substeps at 960 Hz) never goes negative, and that
// fuel_efficiency_violations stays 0. See this test's own discrimination
// demo (temporarily restore the old formula, rebuild, rerun, report the
// failure, revert).
// ---------------------------------------------------------------------------
TEST_CASE("Fix 1: engine_heat never goes negative through a DFCO cut and recovery, driven through a real Powertrain",
          "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());
    WorldRig rig = make_world_rig(fueled_desc);

    rig.world->set_control("assist.auto_clutch", 1.0); // anti-stall WOT-from-rest launch law

    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false; // let the auto-clutch launch law manage engagement from open
    rig.world->reset_vehicle(rig.vehicle, o);

    for (int settle_tick = 0; settle_tick < 240; ++settle_tick) rig.world->step(); // 1 s settle, mirrors F4/F9

    constexpr int kBuildTicks = 480;  // 2 s WOT: clears dfco_enable_rpm (1300) well before lift
    constexpr int kCoastTicks = 3600; // 15 s coast: DFCO latch, recovery, and settle toward idle
    bool saw_dfco = false;
    bool saw_recovery_while_falling = false;
    real prev_engine_heat = rig.world->powertrain_energy(rig.vehicle).engine_heat;
    real prev_rpm = rig.world->powertrain_state(rig.vehicle).engine_rpm;
    for (int tick = 0; tick < kBuildTicks + kCoastTicks; ++tick) {
        rig.world->set_control("throttle", tick < kBuildTicks ? 1.0 : 0.0);
        rig.world->step();

        const drivetrain::PowertrainSnapshot& snap = rig.world->powertrain_state(rig.vehicle);
        const real rpm_new = snap.engine_rpm;
        const bool dfco_active = !snap.engines.empty() && snap.engines[0].dfco_active;
        const real theta_c = snap.engines.empty() ? 0.0 : snap.engines[0].load;
        if (dfco_active) saw_dfco = true;
        // "recovery while falling": DFCO has already fired at least once, is
        // no longer active (fuel/idle PI refilling), theta_c is really
        // nonzero (combustion firing again), and rpm is still decreasing -
        // the exact window the old formula's tau0_ -> 0 crossing lives in.
        if (saw_dfco && !dfco_active && theta_c > 0.0 && rpm_new < prev_rpm) saw_recovery_while_falling = true;

        const drivetrain::PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
        const real heat_delta = e.engine_heat - prev_engine_heat;
        INFO("tick " << tick << " rpm=" << rpm_new << " dfco=" << dfco_active << " theta_c=" << theta_c
                      << " heat_delta=" << heat_delta << " engine_heat=" << e.engine_heat);
        CHECK(heat_delta >= -1e-9);

        prev_engine_heat = e.engine_heat;
        prev_rpm = rpm_new;
    }

    const drivetrain::PowertrainEnergy& final_ledger = rig.world->powertrain_energy(rig.vehicle);
    CHECK(saw_dfco);                   // non-vacuous: DFCO really latched
    CHECK(saw_recovery_while_falling); // non-vacuous: the targeted window really occurred
    CHECK(final_ledger.fuel_efficiency_violations == 0);
    // Non-vacuous guard against the earlier version of this test's own
    // failure mode (an instant-clutch-dump launch stalled the engine at
    // tick 0, leaving heat_delta's per-tick check vacuously satisfied - see
    // this test's own header comment): the engine must have actually burned
    // fuel and produced heat over the whole run, like F4's own checks.
    CHECK(final_ledger.fuel_mass > 0.0);
    CHECK(final_ledger.engine_heat > 0.0);
}

// ---------------------------------------------------------------------------
// New (owner ruling replacing fix 2): car_sedan's own engine, loaded WITHOUT
// adding a fuel map (car_sedan_engine_public_desc() alone, via
// to_engine_desc_for_test() - car_sedan.json ships with no fuel_map field at
// all). indicated_work must still be computed and booked (owner ruling: an
// engine without a map still models combustion mechanically), while the
// fuel_mass/fuel_energy_in/engine_heat fields - which need an actual mass/
// LHV to convert against - stay exactly 0 until a future no-map fuel model
// lands. indicated_work is cross-checked against an INDEPENDENT
// recomputation from PUBLIC data alone (pre.theta_c/pre.rpm plus the
// public Desc's own wot/motoring Curve1D's), never engine-internal state.
// ---------------------------------------------------------------------------
TEST_CASE("New: car_sedan without a fuel map still books indicated_work, fuel fields stay 0", "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc_for_test(car_sedan_engine_public_desc());
    REQUIRE_FALSE(desc.has_fuel_map); // precondition: car_sedan really ships with no fuel map
    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Running);
    const real J = engine.desc().inertia_kgm2;

    real omega = rpm_to_omega(900.0);
    PowertrainEnergy ledger;
    real recomputed_sum = 0.0;
    constexpr int kSubsteps = 2000; // ~2.08 s at 960 Hz: WOT launch then lift
    for (int i = 0; i < kSubsteps; ++i) {
        const real throttle_cmd = (i < kSubsteps / 2) ? 1.0 : 0.0;
        const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(throttle_cmd), no_ignition_cut(), kSubstepH);
        const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
        const PostSolveResult post = engine.post_solve(omega_new, kSubstepH);

        // Powertrain::post_solve_components' exact booking rule:
        // indicated_work is unconditional (fix 1/owner ruling); fuel_mass/
        // fuel_energy_in/engine_heat/fuel_efficiency_violations stay gated
        // on has_fuel_map (always false here) - engine_heat must NOT be
        // booked from indicated_work_delta alone or it would wrongly go
        // negative every substep (caught by this test itself before this
        // gate was added: engine_heat read -72354.67 instead of 0).
        ledger.indicated_work += post.indicated_work_delta;
        if (desc.has_fuel_map) {
            ledger.fuel_mass += post.fuel_mass_delta;
            ledger.fuel_energy_in += post.fuel_energy_in_delta;
            ledger.engine_heat += post.fuel_energy_in_delta - post.indicated_work_delta;
        }

        // Public-data-only re-derivation: tau_comb = theta_c*(wot(rpm) -
        // motoring(rpm)), and indicated_work_delta = h*omega_new*tau_comb -
        // fix 1's own formula, recomputed here from PreSolveResult::theta_c/
        // rpm plus the public Desc's curves alone (no access to the
        // engine's private tau_comb_ member).
        const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
        recomputed_sum += kSubstepH * omega_new * tau_comb;

        omega = omega_new;
    }

    CHECK(ledger.indicated_work > 0.0); // non-vacuous: real combustion work really accumulated
    const real rel_err = std::abs(ledger.indicated_work - recomputed_sum) / std::max(std::abs(recomputed_sum), 1e-12);
    CHECK(rel_err <= 1e-9);

    CHECK(ledger.fuel_mass == 0.0);
    CHECK(ledger.fuel_energy_in == 0.0);
    CHECK(ledger.engine_heat == 0.0);
}

// ---------------------------------------------------------------------------
// Fix 1 discrimination demo (formula-level, deterministic): the whole-vehicle
// "Fix 1" DFCO/recovery test above (real World + Powertrain, an auto-clutch
// WOT launch then a long coast) is genuine, non-vacuous regression coverage,
// but empirically does NOT drive tau0_ close enough to exactly 0 while
// tau_comb_ stays meaningfully nonzero to trip the old formula's singularity
// at TICK granularity (confirmed by hand: temporarily restoring the old
// formula in engine_torque_map.cpp and rerunning that test still passes,
// 4085/4085 assertions - the coasting trajectory's tau0_ zero-crossings
// during recovery happen to coincide with small |domega|, so `work` itself
// stays small there too). This test instead pins the actual mathematical
// property directly: using car_sedan's real (non-flat) engine curves, it
// finds an (rpm, throttle) operating point where the real EngineTorqueMap's
// own PreSolveResult::tau0 is very small while combustion is clearly firing
// (theta_c > 0, tau_comb sizeable), then evaluates post_solve at an omega_new
// chosen INDEPENDENTLY of the engine's own local tau0_/d_ ODE - exactly what
// a real ShaftNetwork::solve does when the crank is coupled to something
// else (a vehicle's own inertia via a locked clutch/gearbox/diff, feeding
// engine braking back through the driveline even as the engine's own net
// torque nets near zero). It compares the CURRENT production formula
// (h*omega_new*tau_comb, called for real through EngineTorqueMap::
// post_solve) against a hand-computed OLD formula (work*tau_comb/tau0,
// built from the same PreSolveResult::tau0/d, the public curves' own
// tau_comb, and the same omega_new) - no production code is touched for
// this test to run every time; the numbers below were captured once by
// hand with the old formula temporarily restored in production and
// reported in this commit's message.
// ---------------------------------------------------------------------------
TEST_CASE("Fix 1 discrimination demo: the old proportional-split formula is singular where the new formula stays bounded",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc_for_test(car_sedan_engine_public_desc());

    // Search a grid of (rpm, throttle command) for the operating point with
    // the smallest |tau0| among those where combustion is clearly firing
    // (|tau_comb| > 5 N*m) and the engine is genuinely Running - a fresh
    // EngineTorqueMap per trial so no idle-integrator/DFCO/limiter history
    // from a previous trial leaks in.
    real best_abs_tau0 = std::numeric_limits<real>::infinity();
    real best_rpm = 0.0, best_throttle = 0.0;
    PreSolveResult best_pre{};
    for (real rpm = 700.0; rpm <= 6500.0; rpm += 25.0) {
        for (real throttle_cmd = 0.0; throttle_cmd <= 1.0; throttle_cmd += 0.02) {
            EngineTorqueMap probe(desc);
            probe.reset(EngineState::Running);
            const PreSolveResult pre =
                probe.pre_solve(sensors_at_omega(rpm_to_omega(rpm)), throttle_of(throttle_cmd), no_ignition_cut(), kSubstepH);
            if (pre.state != EngineState::Running) continue;
            const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
            const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
            const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
            if (std::abs(tau_comb) <= 5.0) continue;
            if (std::abs(pre.tau0) < best_abs_tau0) {
                best_abs_tau0 = std::abs(pre.tau0);
                best_rpm = rpm;
                best_throttle = throttle_cmd;
                best_pre = pre;
            }
        }
    }
    REQUIRE(best_abs_tau0 < std::numeric_limits<real>::infinity()); // the search actually found a candidate
    INFO("best rpm=" << best_rpm << " throttle=" << best_throttle << " tau0=" << best_pre.tau0 << " theta_c=" << best_pre.theta_c
                      << " d=" << best_pre.d);

    // Re-run the winning (rpm, throttle) on a fresh engine to get a clean
    // PreSolveResult, then post_solve at an omega_new offset from omega_old
    // by an EXTERNALLY-imposed deceleration (-3 rad/s over this substep,
    // ~-28.6 rpm - a plausible engine-braking-through-a-locked-driveline
    // rate), i.e. NOT the value the isolated tau0_/d_ ODE would produce -
    // the one condition a bare/self-consistent EngineTorqueMap test can
    // never manufacture (see this file's "Fix 1" test header comment).
    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Running);
    const real omega_old = rpm_to_omega(best_rpm);
    const PreSolveResult pre =
        engine.pre_solve(sensors_at_omega(omega_old), throttle_of(best_throttle), no_ignition_cut(), kSubstepH);
    REQUIRE(pre.state == EngineState::Running);
    const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
    const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
    const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
    REQUIRE(std::abs(tau_comb) > 5.0);

    constexpr real kExternalDomega = -3.0; // rad/s, imposed by "the network", not this engine's own tau0_/d_
    const real omega_new = omega_old + kExternalDomega;

    const PostSolveResult post = engine.post_solve(omega_new, kSubstepH); // the REAL production formula
    const real new_formula_value = post.indicated_work_delta;
    const real new_formula_expected = kSubstepH * omega_new * tau_comb;
    CHECK(new_formula_value == Catch::Approx(new_formula_expected).margin(1e-9)); // production matches the documented formula

    // Hand-computed OLD formula (engine_torque_map.cpp's pre-review code):
    // work*tau_comb/tau0, from the SAME pre.tau0/pre.d/tau_comb/omega_new.
    const real tau_lin = pre.tau0 - pre.d * (omega_new - omega_old);
    const real work = kSubstepH * omega_new * tau_lin;
    const real frac_comb_old = pre.tau0 != real(0.0) ? tau_comb / pre.tau0 : real(0.0);
    const real old_formula_value = work * frac_comb_old;

    INFO("tau0=" << pre.tau0 << " tau_comb=" << tau_comb << " d=" << pre.d << " domega=" << kExternalDomega
                 << " new_formula=" << new_formula_value << " old_formula=" << old_formula_value);
    // The new formula is bounded by the physically sane combustion-work
    // scale (h*omega*tau_comb); the old formula, built from a near-zero
    // tau0 denominator, is not - assert its MAGNITUDE is an order of
    // magnitude larger than the new formula's (the discrimination itself).
    // Measured on this grid: new_formula=40.51 N*m*s, old_formula=2304.85
    // N*m*s (~57x) - the 10x bound below leaves ample margin.
    CHECK(std::abs(old_formula_value) > 10.0 * std::abs(new_formula_value));
}

} // namespace (F2/F3/F4 helpers)

// ---------------------------------------------------------------------------
// F5: loader rejections (core/src/io/vehicle_io.cpp's engine/fuel/fuel_map
// validation, read in full to pin every exact message below). Never touches
// data/** - every fixture is built in memory and written to a scratch temp
// file.
// ---------------------------------------------------------------------------
namespace f5 {

struct TempJsonFile {
    std::filesystem::path path;
    explicit TempJsonFile(const nlohmann::json& j) {
        path = std::filesystem::temp_directory_path() /
               ("ps_test_fuel_map_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" +
                std::to_string(std::hash<std::string>{}(j.dump())) + ".json");
        std::ofstream out(path);
        out << j.dump(2);
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

std::string fuel_json_path() {
    return std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string();
}

// A minimal, otherwise-valid torque_map engine component with a fuel map -
// every F5 SECTION mutates a copy of this.
nlohmann::json base_engine_component() {
    nlohmann::json j;
    j["id"] = "engine";
    j["type"] = "engine";
    j["kind"] = "torque_map";
    j["wot_torque_nm_vs_rpm"] = {{"x", {500.0, 7000.0}}, {"y", {100.0, 100.0}}};
    j["motoring_torque_nm_vs_rpm"] = {{"x", {0.0, 7000.0}}, {"y", {-20.0, -20.0}}};
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 20.0;
    j["fuel"] = fuel_json_path();
    j["fuel_map"] = {{"rpm", {500.0, 7000.0}},
                      {"load", {0.0, 1.0}},
                      {"fuel_flow_g_s", {{0.0, 3.0}, {0.0, 12.0}}},
                      {"source", "synthetic test fixture (test_engine_fuel_map.cpp)"}};
    return j;
}

nlohmann::json vehicle_with_engine(const nlohmann::json& engine_component) {
    nlohmann::json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_engine_fuel_map.cpp)";
    v["wheels"] = nlohmann::json::array({{{"name", "FL"}}});
    v["powertrain"] = {{"components", nlohmann::json::array({engine_component})}};
    return v;
}

void expect_rejected(const nlohmann::json& engine_component, const std::string& substring) {
    const TempJsonFile f(vehicle_with_engine(engine_component));
    REQUIRE_THROWS_WITH(io::load_vehicle_json(f.path.string()), ContainsSubstring(substring));
}

} // namespace f5

TEST_CASE("F5: loader rejects every malformed engine/fuel/fuel_map field with its exact message", "[unit][drivetrain][fuel][io]") {
    using f5::base_engine_component;
    using f5::expect_rejected;

    SECTION("valid base engine loads with has_fuel_map true (sanity, not a rejection)") {
        const f5::TempJsonFile f(f5::vehicle_with_engine(base_engine_component()));
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        bool found = false;
        for (const auto& c : d.powertrain.components) {
            if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
                found = true;
                CHECK(e->has_fuel_map);
                CHECK(e->fuel_map.value(500.0, 1.0) == 3.0);
                CHECK(e->fuel_map.value(7000.0, 1.0) == 12.0);
            }
        }
        CHECK(found);
    }

    SECTION("kind == simulated is rejected as not yet supported") {
        nlohmann::json e = base_engine_component();
        e["kind"] = "simulated";
        expect_rejected(e, "not yet supported (PHYS-033 I7b)");
    }
    SECTION("kind == bogus is rejected") {
        nlohmann::json e = base_engine_component();
        e["kind"] = "bogus";
        expect_rejected(e, "\"torque_map\" or \"simulated\"");
    }
    SECTION("fuel_map with no fuel reference is rejected") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        expect_rejected(e, "requires a 'fuel' reference");
    }
    SECTION("fuel present but empty is rejected") {
        nlohmann::json e = base_engine_component();
        e["fuel"] = "";
        expect_rejected(e, "'fuel' must be a non-empty path string");
    }
    SECTION("fuel_map missing rpm") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("rpm");
        expect_rejected(e, "'rpm' array is required");
    }
    SECTION("fuel_map missing load") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("load");
        expect_rejected(e, "'load' array is required");
    }
    SECTION("fuel_map missing fuel_flow_g_s") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("fuel_flow_g_s");
        expect_rejected(e, "'fuel_flow_g_s' array is required");
    }
    SECTION("fuel_map missing source") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"].erase("source");
        expect_rejected(e, "'source' is required");
    }
    SECTION("fuel_map rpm needs at least 2 points") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["rpm"] = {500.0};
        expect_rejected(e, "'rpm' needs at least 2 points");
    }
    SECTION("fuel_map load needs at least 2 points") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {0.0};
        expect_rejected(e, "'load' needs at least 2 points");
    }
    SECTION("fuel_map rpm must be strictly ascending") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["rpm"] = {7000.0, 500.0};
        expect_rejected(e, "'rpm' must be strictly ascending");
    }
    SECTION("fuel_map load must be strictly ascending") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {1.0, 0.0};
        expect_rejected(e, "'load' must be strictly ascending");
    }
    SECTION("fuel_map load must start at exactly 0.0") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {0.2, 1.0};
        expect_rejected(e, "'load' must start at exactly 0.0");
    }
    SECTION("fuel_map load must end at exactly 1.0") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["load"] = {0.0, 0.8};
        expect_rejected(e, "'load' must end at exactly 1.0");
    }
    SECTION("fuel_map fuel_flow_g_s row count must match rpm point count") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["rpm"] = {500.0, 3000.0, 7000.0};
        // fuel_flow_g_s left at 2 rows -> mismatch (expected 3).
        expect_rejected(e, "must have one row per 'rpm' point (3)");
    }
    SECTION("fuel_map fuel_flow_g_s row length must match load point count") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, 3.0, 5.0}, {0.0, 12.0}};
        expect_rejected(e, "row 0 must have one value per 'load' point (2)");
    }
    SECTION("fuel_map fuel_flow_g_s value must be >= 0 and finite") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, -1.0}, {0.0, 12.0}};
        expect_rejected(e, "fuel_flow_g_s[0][1] must be >= 0 and finite");
    }
    SECTION("fuel_map fuel_flow_g_s at load 0 must be exactly 0.0") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{5.0, 3.0}, {0.0, 12.0}};
        expect_rejected(e, "fuel_flow_g_s[0][0] (load 0) must be exactly 0.0");
    }
    SECTION("fuel_map node implying >100% thermal efficiency is rejected") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, 3.0}, {0.0, 0.01}}; // at rpm=7000, load=1: far too little fuel for the mechanical power
        expect_rejected(e, "implies >100% thermal efficiency");
    }
    // Review fix 4: "a missing kind defaulting to torque_map stays" - pin it
    // with a test, since every shipped engine file sets "kind" explicitly
    // (this is the only place the default path itself is exercised).
    SECTION("kind absent defaults to torque_map (pin)") {
        nlohmann::json e = base_engine_component();
        e.erase("kind");
        const f5::TempJsonFile f(f5::vehicle_with_engine(e));
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        bool found = false;
        for (const auto& c : d.powertrain.components) {
            if (std::holds_alternative<TorqueMapEngineDesc>(c.params)) found = true;
        }
        CHECK(found);
    }
    // Review fix 4: a non-numeric fuel_flow_g_s entry (e.g. a JSON string)
    // must give the same pinned std::runtime_error every other rejection
    // gives, not let nlohmann::json::type_error propagate uncaught from
    // .get<real>() - core/src/io/vehicle_io.cpp's is_number() check.
    SECTION("fuel_map fuel_flow_g_s non-numeric entry is rejected") {
        nlohmann::json e = base_engine_component();
        e["fuel_map"]["fuel_flow_g_s"] = {{0.0, "not_a_number"}, {0.0, 12.0}};
        expect_rejected(e, "fuel_flow_g_s[0][1] must be a number");
    }
}

// ---------------------------------------------------------------------------
// F6/F9: whole-World checks with tests/support/fixtures/vehicle_fuel_map_sedan.json
// (a car_sedan.json clone, engine inlined, a synthetic fuel map/fuel added).
// fuel_map_sedan_path()/WorldRig/make_world_rig() now live above (moved up
// so the review-fixed F4 can use them too) - only the F6-only helper stays
// here.
// ---------------------------------------------------------------------------
namespace {

bool any_channel_contains(const std::vector<std::string>& names, const std::string& substr) {
    return std::any_of(names.begin(), names.end(), [&](const std::string& n) { return n.find(substr) != std::string::npos; });
}

} // namespace

TEST_CASE("F6: a fuel map with no tank is bit-identical to no fuel map at all (state_hash + CSV header)", "[unit][drivetrain][fuel][determinism]") {
    struct Ev {
        real t;
        const char* channel;
        real value;
    };
    // data/scenarios/vehicle_powertrain.json's own control_events, replayed
    // by hand against both worlds (not through io::build_scenario - the
    // comparison only needs the two worlds to see the identical script, not
    // to reproduce that scenario file's own recorded hash).
    const std::vector<Ev> events = {{0.2, "shift_up_count", 1.0}, {0.3, "throttle", 0.8}, {3.0, "shift_up_count", 2.0},
                                     {6.0, "brake", 0.5},         {7.0, "shift_down_count", 1.0},
                                     {8.0, "starter", 1.0},       {8.2, "starter", 0.0}};

    const vehicle::VehicleDesc plain_desc = ps_test::load_sedan_desc();
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());

    WorldRig ra = make_world_rig(plain_desc);
    WorldRig rb = make_world_rig(fueled_desc);

    constexpr real kTickHz = 240.0;
    constexpr int kTicks = 2400; // 10 s, matching vehicle_powertrain.json's duration_s
    std::size_t next_event = 0;
    int mismatches = 0;
    int first_mismatch = -1;
    for (int tick = 0; tick < kTicks; ++tick) {
        const real sim_time = tick / kTickHz;
        while (next_event < events.size() && sim_time >= events[next_event].t) {
            ra.world->set_control(events[next_event].channel, events[next_event].value);
            rb.world->set_control(events[next_event].channel, events[next_event].value);
            ++next_event;
        }
        ra.world->step();
        rb.world->step();
        if (ra.world->state_hash() != rb.world->state_hash() && mismatches++ == 0) first_mismatch = tick;
    }
    INFO("first differing tick " << first_mismatch << " (" << mismatches << " of " << kTicks << ")");
    CHECK(mismatches == 0);
    CHECK(next_event == events.size()); // non-vacuous: the whole script actually ran

    const auto& plain_names = ra.world->telemetry().channel_names();
    const auto& fueled_names = rb.world->telemetry().channel_names();
    CHECK_FALSE(any_channel_contains(plain_names, ".fuel."));
    CHECK(any_channel_contains(fueled_names, ".fuel."));
}

TEST_CASE("F9: balance_error stays within bound with a fuel map over a WOT launch", "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());
    WorldRig rig = make_world_rig(fueled_desc);

    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);

    // Settle the suspension onto the ground first (mirrors
    // test_powertrain_energy_balance.cpp's own ps_test::settle(*rig.world,
    // 1.0) before it starts checking balance_error): with throttle still 0
    // the ledger's cumulative terms (denom below) are still near-zero for
    // the first handful of ticks after reset_vehicle, so a tiny, purely
    // numerical (non-physical) residual in balance_error can transiently
    // make the ratio enormous even though nothing is actually leaking -
    // exactly the failure mode measured here before this fix (ratio 4.04
    // at an early tick). Settling first ensures denom is already a
    // meaningful magnitude before the WOT launch below starts being
    // checked.
    for (int settle_tick = 0; settle_tick < 240; ++settle_tick) rig.world->step(); // 1 s

    rig.world->set_control("throttle", 1.0);

    // Mirrors test_powertrain_energy_balance.cpp's own check_tick: balance_error
    // is a per-tick residual, but the ledger terms it closes against are
    // CUMULATIVE since vehicle creation, so the bound must scale with their
    // running sum (denom), not be a bare absolute constant - a bare 5e-8
    // absolute bound is meaningless once the cumulative ledger reaches
    // thousands of joules over a launch.
    auto sum_abs_terms = [](const drivetrain::PowertrainEnergy& e) {
        return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) +
               std::abs(e.gear_loss) + std::abs(e.integrator_loss) + std::abs(e.clutch_loss) +
               std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) + std::abs(e.engagement_loss) +
               std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
    };

    real worst_ratio = 0.0;
    for (int tick = 0; tick < 720; ++tick) { // 3 s at 240 Hz
        rig.world->step();
        const drivetrain::PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) {
            const real ratio = std::abs(e.balance_error) / denom;
            worst_ratio = std::max(worst_ratio, ratio);
            CHECK(ratio <= 5e-8);
        }
    }
    INFO("worst |balance_error|/denom = " << worst_ratio);
    CHECK(worst_ratio <= 5e-8);

    const drivetrain::PowertrainEnergy& final_ledger = rig.world->powertrain_energy(rig.vehicle);
    CHECK(final_ledger.fuel_mass > 0.0);         // non-vacuous: fuel really flowed
    CHECK(final_ledger.fuel_efficiency_violations == 0);
}
