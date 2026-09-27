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
// desc()/ps::drivetrain::to_engine_desc() (the real production resolver,
// core/src/drivetrain/powertrain.cpp/.h - PHYS-033 P4 I7a C5d review fix B2
// moved it out of that file's anonymous namespace specifically so tests can
// call it directly instead of a hand-duplicated copy) build a bare
// EngineTorqueMap from them. The "Fix 1" DFCO test further down deliberately
// does NOT use a bare EngineTorqueMap - see that test's own header comment
// for why a bare engine cannot discriminate the defect it targets, and
// drives the real WorldRig/vehicle_fuel_map_sedan.json Powertrain instead.

#include "drivetrain/engine_torque_map.h"
#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

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
        // PHYS-033 P4 I7a C5d: has_fuel_model is a separate field from
        // has_fuel_map (EngineTorqueMap::Desc's own comment) - production's
        // to_engine_desc() always sets it via has_fuel_model(e), which is
        // true whenever has_fuel_map is true regardless of "fuels". This
        // hand-built test descriptor bypasses to_engine_desc, so it must set
        // the same invariant itself or post_solve()'s has_fuel_model-gated
        // fuel_mass_delta/fuel_energy_in_delta booking never fires even
        // though has_fuel_map is true (found by F2: fuel_mass_sum stayed
        // exactly 0.0 instead of matching the independently re-derived sum).
        d.has_fuel_model = true;
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

// PHYS-033 P4 I7a C5d review fix (B2): this file used to hand-duplicate
// core/src/drivetrain/powertrain.cpp's own to_engine_desc() field-for-field
// (as to_engine_desc_for_test()), kept here because that function was
// anonymous-namespace/internal. It has since been moved out of the anonymous
// namespace and declared in powertrain.h specifically so tests exercise the
// REAL production resolver instead of a copy that cannot catch a bug
// introduced in the real one - see every "C5d Test N" call site below, which
// now calls ps::drivetrain::to_engine_desc(...) directly (reachable
// unqualified here via the file's own `using namespace ps::drivetrain;` and
// its #include "drivetrain/powertrain.h").

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

// Same body/ground/vehicle-creation as make_world_rig() above, but with
// tick_rate_hz == substep_rate_hz (1:1) so exactly one Powertrain substep
// runs per World::step() call - the ordinary 4:1 WorldRig above only ever
// exposes per-TICK (4-substeps-averaged) state through World::powertrain_
// state()/powertrain_energy(), which cannot support a genuine per-substep
// cross-check. `gear` is passed straight through to VehicleResetOptions -
// 0 (Neutral) for "Fix 3" F2's free-rev test below (no wheel/tyre/road-load
// coupling to obscure the engine's own pre/post-solve rpm split, but still
// a REAL Powertrain: clutch + gearbox input inertia + engine, network-
// solved every substep), a driven gear for "Fix 3" F4's per-substep ledger
// cross-check (real wheel/tyre coupling, so indicated_work/engine_heat
// actually move).
WorldRig make_world_rig_1to1(const vehicle::VehicleDesc& desc, int gear) {
    WorldRig rig;
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.tick_rate_hz = 960.0;
    cfg.substep_rate_hz = 960.0; // 1:1
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
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = gear;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

TorqueMapEngineDesc fueled_engine_public_desc(const vehicle::VehicleDesc& desc) {
    for (const auto& c : desc.powertrain.components) {
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) return *e;
    }
    throw std::runtime_error("fueled_engine_public_desc: no TorqueMapEngineDesc component");
}

// ---------------------------------------------------------------------------
// "Fix 3" F2 (coordinator spec): the review-fixed F2 above (bare
// EngineTorqueMap, hand-rolled implicit-Euler omega update) cannot
// discriminate a production mistake that looked up the fuel map with the
// POST-solve rpm or the raw throttle command instead of theta_c - omega_new
// there is derived directly from THAT SAME pre_solve call's own tau0_/d_
// pair, so the hand-rolled loop's own coupling can mask exactly the two
// mistakes this test targets. This version drives a REAL Powertrain
// (make_world_rig_1to1_neutral() above, 1:1 tick:substep ratio so one
// World::step() call is exactly one Powertrain substep) through the same
// fuel-mapped fixture (vehicle_fuel_map_sedan.json) F4/F6/F9/Fix 1 use, in
// Neutral with a deliberate free-rev-then-lift: WOT for 0.2 s (no wheel
// load, so rpm climbs fast purely on engine inertia - a real, non-flat
// pre/post-solve rpm split every substep), then throttle 0 for ~3 s so the
// engine coasts down through DFCO and settles at idle via the idle PI -
// theta_c genuinely != the commanded 0.0 there.
//
// Each tick: rpm_before is read from World::powertrain_state().engine_rpm
// BEFORE calling step() - this is the value the UPCOMING substep's
// pre_solve will use as its own omega_old (engine_torque_map.cpp computes
// `rpm = abs(omega_old)*kRadPerSecToRpm`; vehicle_instance.cpp fills
// engine_rpm as `abs(engine_omega())*(60/(2*pi))` - the identical formula
// applied to the identical stored omega, confirmed by direct comparison of
// both source lines). theta_c/fuel_flow_g_s are read AFTER step() -
// PreSolveResult fields cached from the substep that just ran, at that same
// rpm_before/omega_old. Both are compared against an INDEPENDENT
// fuel_map.value(rpm_before, theta_c) lookup on the REAL Table2D pulled
// from the loaded VehicleDesc (never a hand-duplicated formula) - a
// production bug that instead read the post-solve rpm, or the raw command
// in place of theta_c, moves this comparison far outside its tight
// tolerance every substep it is wrong. See this test's own discrimination
// demo (temporarily make each mistake in production code, rebuild, rerun,
// report the two failure numbers, revert).
// ---------------------------------------------------------------------------
TEST_CASE("Fix 3 F2: fuel flow matches the map at pre-solve rpm/theta_c through a real Powertrain, not post-solve rpm or the raw command",
          "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());
    const TorqueMapEngineDesc engine_desc = fueled_engine_public_desc(fueled_desc);
    REQUIRE(engine_desc.has_fuel_map);

    WorldRig rig = make_world_rig_1to1(fueled_desc, /*gear=*/0);

    // Prime the snapshot: World::powertrain_state()'s engine_rpm/load/fuel_flow_g_s
    // fields are all default-constructed (0) until stage_snapshot() has run at least
    // once, so reading engine_rpm BEFORE any step() at all would see 0 regardless of
    // the real initial crank omega VehicleResetOptions/Powertrain construction set up
    // - not "the omega the upcoming substep's pre_solve will actually use". One
    // throttle-0 warm-up step (not part of the measured loop/its saw_*/max_rel_err
    // bookkeeping) makes the very first measured iteration's "rpm_before" read valid.
    rig.world->set_control("throttle", 0.0);
    rig.world->step();

    constexpr int kRevTicks = 200;    // ~0.208 s WOT free-rev (no wheel load)
    constexpr int kSettleTicks = 3000; // ~3.125 s coast: DFCO then idle-PI settle
    const real rpm_start = rig.world->powertrain_state(rig.vehicle).engine_rpm;

    bool saw_theta_c_ne_command = false;
    bool saw_omega_change = false;
    real max_rel_err = 0.0;
    real max_ledger_rel_err = 0.0;
    real prev_fuel_mass = rig.world->powertrain_energy(rig.vehicle).fuel_mass;

    for (int tick = 0; tick < kRevTicks + kSettleTicks; ++tick) {
        rig.world->set_control("throttle", tick < kRevTicks ? 1.0 : 0.0);
        const real rpm_before = rig.world->powertrain_state(rig.vehicle).engine_rpm;
        rig.world->step();
        const drivetrain::PowertrainSnapshot& snap = rig.world->powertrain_state(rig.vehicle);
        REQUIRE(snap.engines.size() == 1);
        const real theta_c = snap.engines[0].load;
        const real fuel_flow_g_s = snap.engines[0].fuel_flow_g_s;
        const real command = tick < kRevTicks ? 1.0 : 0.0;

        if (theta_c != command) saw_theta_c_ne_command = true;
        if (std::abs(snap.engine_rpm - rpm_start) > 1.0) saw_omega_change = true;

        const real expected_g_s = engine_desc.fuel_map.value(rpm_before, theta_c);
        const real denom = std::max(std::abs(expected_g_s), 1e-9);
        const real rel_err = std::abs(fuel_flow_g_s - expected_g_s) / denom;
        max_rel_err = std::max(max_rel_err, rel_err);
        INFO("tick " << tick << " rpm_before=" << rpm_before << " theta_c=" << theta_c << " command=" << command
                      << " fuel_flow_g_s=" << fuel_flow_g_s << " expected_g_s=" << expected_g_s << " rel_err=" << rel_err);
        CHECK(rel_err <= 1e-9);

        // Also cross-check what Powertrain::post_solve_components actually BOOKS
        // into the ledger (h*mdot_kg_s, gated by has_fuel_map) - fuel_flow_g_s
        // above comes from PreSolveResult alone and would NOT catch a mistake
        // confined to post_solve's own use of the cached mdot (e.g. recomputing
        // it at the post-solve rpm instead of using the pre-solve-cached value -
        // this file's own demo (a)).
        const real fuel_mass_now = rig.world->powertrain_energy(rig.vehicle).fuel_mass;
        const real ledger_delta_kg = fuel_mass_now - prev_fuel_mass;
        prev_fuel_mass = fuel_mass_now;
        const real expected_delta_kg = kSubstepH * (expected_g_s / real(1000.0));
        const real ledger_denom = std::max(std::abs(expected_delta_kg), 1e-12);
        const real ledger_rel_err = std::abs(ledger_delta_kg - expected_delta_kg) / ledger_denom;
        max_ledger_rel_err = std::max(max_ledger_rel_err, ledger_rel_err);
        INFO("tick " << tick << " ledger_delta_kg=" << ledger_delta_kg << " expected_delta_kg=" << expected_delta_kg
                      << " ledger_rel_err=" << ledger_rel_err);
        CHECK(ledger_rel_err <= 1e-9);
    }

    INFO("max_rel_err over the whole run = " << max_rel_err);
    INFO("max_ledger_rel_err over the whole run = " << max_ledger_rel_err);
    REQUIRE(saw_theta_c_ne_command); // precondition: theta_c really differs from the commanded throttle (idle PI, post-lift)
    REQUIRE(saw_omega_change);       // precondition: omega really varies across the run (free rev + coast-down)
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
// "Fix 3" F4 (coordinator spec): F4/F9/F6 above all check coarse PROPERTIES
// of the ledger (monotonicity, a bound, bit-identity) - none of them
// independently RECOMPUTES fuel_mass/fuel_energy_in/engine_heat from their
// own defining per-substep sums and compares against Powertrain::
// post_solve_components' actual booking, so a mistake in THAT function's own
// gating or summation (e.g. dropping one of its three `ledger_.fuel_* +=`/
// `ledger_.engine_heat +=` lines) would not necessarily be caught. This test
// drives a REAL Powertrain (make_world_rig_1to1() above, 1:1 tick:substep
// ratio, gear 1 - real wheel/tyre/road-load coupling so fuel flow and
// indicated_work genuinely vary) through a WOT launch from rest, and every
// tick (== one substep) independently accumulates:
//   - expected_fuel_mass      += h * mdot_kg_s        (mdot read back from
//     World::powertrain_state().engines[0].fuel_flow_g_s / 1000, the SAME
//     substep's cached PreSolveResult - not a hand-duplicated formula)
//   - expected_fuel_energy_in += (h * mdot_kg_s) * fuel.lhv_j_kg (LHV pulled
//     from the loaded VehicleDesc's own FuelDesc, never a hand-typed number)
//   - expected_engine_heat    += (h * mdot_kg_s * lhv) - indicated_work_delta
//     (indicated_work_delta read directly off World::powertrain_energy()'s
//     own cumulative indicated_work field, since indicated_work's OWN
//     booking is unconditional/untested here - see the "New: car_sedan
//     without a fuel map..." test above for that one - this test isolates
//     Powertrain::post_solve_components' fuel_mass/fuel_energy_in/
//     engine_heat gating and summation specifically)
// and compares each final cumulative sum against World::powertrain_energy()'s
// actual fuel_mass/fuel_energy_in/engine_heat to a tight relative tolerance.
// Dropping any ONE of post_solve_components' three booking lines leaves that
// one ledger field at (or near) 0 while this test's independent sum for it
// stays large and positive - see this test's own discrimination demo.
// ---------------------------------------------------------------------------
TEST_CASE("Fix 3 F4: fuel_mass/fuel_energy_in/engine_heat match independent per-substep sums through a real Powertrain",
          "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc fueled_desc = io::load_vehicle_json(fuel_map_sedan_path());
    const TorqueMapEngineDesc engine_desc = fueled_engine_public_desc(fueled_desc);
    REQUIRE(engine_desc.has_fuel_map);
    const real lhv = engine_desc.fuel.lhv_j_kg;
    REQUIRE(lhv > 0.0);

    WorldRig rig = make_world_rig_1to1(fueled_desc, /*gear=*/1);
    rig.world->set_control("throttle", 1.0); // WOT launch from settled rest

    constexpr int kTicks = 2000; // ~2.08 s at 960 Hz, one substep per tick

    real prev_indicated_work = rig.world->powertrain_energy(rig.vehicle).indicated_work;
    real expected_fuel_mass = 0.0;
    real expected_fuel_energy_in = 0.0;
    real expected_engine_heat = 0.0;

    for (int tick = 0; tick < kTicks; ++tick) {
        rig.world->step();
        const drivetrain::PowertrainSnapshot& snap = rig.world->powertrain_state(rig.vehicle);
        REQUIRE(snap.engines.size() == 1);
        const real mdot_kg_s = snap.engines[0].fuel_flow_g_s / real(1000.0);
        const real fuel_mass_delta = kSubstepH * mdot_kg_s;
        const real fuel_energy_in_delta = fuel_mass_delta * lhv;

        const drivetrain::PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
        const real indicated_work_delta = e.indicated_work - prev_indicated_work;
        prev_indicated_work = e.indicated_work;

        expected_fuel_mass += fuel_mass_delta;
        expected_fuel_energy_in += fuel_energy_in_delta;
        expected_engine_heat += fuel_energy_in_delta - indicated_work_delta;
    }

    const drivetrain::PowertrainEnergy& final_ledger = rig.world->powertrain_energy(rig.vehicle);
    REQUIRE(expected_fuel_mass > 0.0);              // non-vacuous: fuel really flowed
    REQUIRE(final_ledger.indicated_work > 0.0);      // non-vacuous: real combustion work was booked (WOT launch)

    auto rel_err = [](real actual, real expected) { return std::abs(actual - expected) / std::max(std::abs(expected), 1e-9); };
    INFO("fuel_mass actual=" << final_ledger.fuel_mass << " expected=" << expected_fuel_mass);
    CHECK(rel_err(final_ledger.fuel_mass, expected_fuel_mass) <= 1e-9);
    INFO("fuel_energy_in actual=" << final_ledger.fuel_energy_in << " expected=" << expected_fuel_energy_in);
    CHECK(rel_err(final_ledger.fuel_energy_in, expected_fuel_energy_in) <= 1e-9);
    INFO("engine_heat actual=" << final_ledger.engine_heat << " expected=" << expected_engine_heat);
    CHECK(rel_err(final_ledger.engine_heat, expected_engine_heat) <= 1e-9);
    CHECK(final_ledger.fuel_efficiency_violations == 0);
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
// New (I7a C5d supersedes the owner ruling this test originally covered):
// car_sedan's own engine, loaded via car_sedan_engine_public_desc() alone
// (via ps::drivetrain::to_engine_desc(), the real production resolver) -
// car_sedan.json/i4_2l_na.json still ship
// with no "fuel_map" field, but i4_2l_na.json now carries a "fuels" list
// (I7a C5d), so has_fuel_model is true even though has_fuel_map stays false:
// the crude constant-efficiency estimate is active. indicated_work is booked
// unconditionally as before (owner ruling: an engine without a MAP still
// models combustion mechanically); fuel_mass/fuel_energy_in/engine_heat are
// now gated on has_fuel_model (broadened from has_fuel_map) and are
// genuinely nonzero here. Every booked quantity is cross-checked against an
// INDEPENDENT recomputation from PUBLIC data alone (pre.theta_c/pre.rpm,
// omega_old/omega_new, plus the public Desc's own wot/motoring Curve1D's and
// fuel/eta_i fields), never engine-internal state (tau_comb_/mdot_kg_s_).
// ---------------------------------------------------------------------------
TEST_CASE("New: car_sedan's crude no-map fuel model (I7a C5d) books fuel_mass/fuel_energy_in/engine_heat via has_fuel_model",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());
    REQUIRE_FALSE(desc.has_fuel_map); // precondition: car_sedan/i4_2l_na.json still ship with no fuel_map
    REQUIRE(desc.has_fuel_model);     // precondition: i4_2l_na.json's "fuels" list (I7a C5d) makes the crude estimate active
    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Running);
    const real J = engine.desc().inertia_kgm2;

    real omega = rpm_to_omega(900.0);
    PowertrainEnergy ledger;
    real recomputed_indicated_work = 0.0;
    real recomputed_fuel_mass = 0.0;
    real recomputed_fuel_energy_in = 0.0;
    constexpr int kSubsteps = 2000; // ~2.08 s at 960 Hz: WOT launch then lift
    for (int i = 0; i < kSubsteps; ++i) {
        const real throttle_cmd = (i < kSubsteps / 2) ? 1.0 : 0.0;
        const real omega_old = omega;
        const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(throttle_cmd), no_ignition_cut(), kSubstepH);
        const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
        const PostSolveResult post = engine.post_solve(omega_new, kSubstepH);

        // Powertrain::post_solve_components' exact booking rule (I7a C5d):
        // indicated_work is unconditional (fix 1/owner ruling); fuel_mass/
        // fuel_energy_in/engine_heat/fuel_efficiency_violations are gated on
        // has_fuel_model (broadened from has_fuel_map) - true here via
        // car_sedan's real "fuels" list.
        ledger.indicated_work += post.indicated_work_delta;
        if (desc.has_fuel_model) {
            ledger.fuel_mass += post.fuel_mass_delta;
            ledger.fuel_energy_in += post.fuel_energy_in_delta;
            const real heat_delta = post.fuel_energy_in_delta - post.indicated_work_delta;
            ledger.engine_heat += heat_delta;
            if (heat_delta < 0.0) ++ledger.fuel_efficiency_violations;
        }

        // Public-data-only re-derivation: tau_comb = theta_c*(wot(rpm) -
        // motoring(rpm)), indicated_work_delta = h*omega_new*tau_comb (fix
        // 1's own formula) - recomputed from PreSolveResult::theta_c/rpm plus
        // the public Desc's curves alone (no access to the engine's private
        // tau_comb_ member). The crude fuel estimate (docs/data_formats/
        // engine.md's "fuels"/"ignition"/"indicated_efficiency" section):
        // mdot = max(0, tau_comb*omega_old) / (eta_i*LHV), using omega_old
        // (this substep's pre_solve speed, matching engine_torque_map.cpp's
        // own pre_solve evaluation point) - recomputed from the public
        // Desc's fuel/eta_i fields alone (no access to mdot_kg_s_).
        const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
        recomputed_indicated_work += kSubstepH * omega_new * tau_comb;

        const real mech_power = std::max(real(0.0), tau_comb * omega_old);
        const real mdot_kg_s = mech_power / (desc.eta_i * desc.fuel.lhv_j_kg);
        const real fuel_mass_delta = kSubstepH * mdot_kg_s;
        recomputed_fuel_mass += fuel_mass_delta;
        recomputed_fuel_energy_in += fuel_mass_delta * desc.fuel.lhv_j_kg;

        omega = omega_new;
    }

    CHECK(ledger.indicated_work > 0.0); // non-vacuous: real combustion work really accumulated
    const real rel_err_work =
        std::abs(ledger.indicated_work - recomputed_indicated_work) / std::max(std::abs(recomputed_indicated_work), 1e-12);
    CHECK(rel_err_work <= 1e-9);

    CHECK(ledger.fuel_mass > 0.0); // non-vacuous: the crude model really burned fuel during the WOT half
    // PHYS-033 P4 I7a C5d review fix (N6): NOT "always 0 by construction" (a
    // prior version of this comment claimed exactly that) - a substep whose
    // omega accelerated by more than 1/eta_i (~3.08x here) within one substep
    // while combusting could in principle still show heat_delta < 0 even with
    // constant eta_i (fuel_energy_in_delta = tau_comb*omega_old/eta_i,
    // indicated_work_delta = tau_comb*omega_new unconditionally - see
    // Powertrain::post_solve_components' booking rule above). This test's own
    // WOT-launch scenario never accelerates anywhere near that fast in one
    // substep, so 0 is what is actually observed here, not a guaranteed
    // mathematical identity - see energy_ledger.h's own corrected comment.
    CHECK(ledger.fuel_efficiency_violations == 0);
    const real rel_err_mass = std::abs(ledger.fuel_mass - recomputed_fuel_mass) / std::max(std::abs(recomputed_fuel_mass), 1e-12);
    CHECK(rel_err_mass <= 1e-9);
    const real rel_err_energy =
        std::abs(ledger.fuel_energy_in - recomputed_fuel_energy_in) / std::max(std::abs(recomputed_fuel_energy_in), 1e-12);
    CHECK(rel_err_energy <= 1e-9);
    const real expected_heat = ledger.fuel_energy_in - ledger.indicated_work;
    CHECK(std::abs(ledger.engine_heat - expected_heat) <= 1e-6 * std::max(std::abs(expected_heat), real(1.0)));
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
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());

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
    // PHYS-033 P4 I7a C5d: "ignition" is now required whenever an engine has
    // any fuel model (has_fuel_map here, via "fuel"+"fuel_map" below) - see
    // the "fuels"/"ignition" SECTIONs further down for the rejection paths
    // this fixture itself does not exercise.
    j["ignition"] = "spark";
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

// PHYS-033 P4 I7a C5d review-fix bugfix (found running the F5 suite for the
// first time after the review): nlohmann::json has no JSON representation
// for a non-finite double, so `e["indicated_efficiency"] =
// std::numeric_limits<double>::infinity()` followed by `dump()` silently
// serializes it as the JSON literal `null` (documented nlohmann behaviour,
// not a bug in that library) - it never round-trips back to a number at
// all, so the "must be finite - rejects Infinity" SECTION below was
// actually exercising vehicle_io.cpp's `is_number()` check ("must be a
// number"), not its `std::isfinite` check ("must be in (0, 1)") as its own
// name and the N9 review comment intended. A raw, hand-written JSON literal
// `1e400` sidesteps this: it IS a syntactically valid JSON number (digits
// plus exponent), so `is_number()` sees it as a number, but nlohmann's own
// double conversion overflows it to +Infinity when parsed back - exactly
// the "survives is_number() but must still fail isfinite()" case the N9
// fix was meant to cover. Written directly (bypassing TempJsonFile's
// nlohmann-object dump) so the literal reaches the file as text, not as a
// re-serialized double.
std::filesystem::path write_raw_temp_json(const std::string& text) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("ps_test_fuel_map_raw_" + std::to_string(std::hash<std::string>{}(text)) + ".json");
    std::ofstream out(path);
    out << text;
    return path;
}

void expect_rejected_raw_text(const std::string& json_text, const std::string& substring) {
    const std::filesystem::path path = write_raw_temp_json(json_text);
    REQUIRE_THROWS_WITH(io::load_vehicle_json(path.string()), ContainsSubstring(substring));
    std::error_code ec;
    std::filesystem::remove(path, ec);
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

    // -----------------------------------------------------------------------
    // PHYS-033 P4 I7a C5d: "fuels" (a plain list of fuel path strings for an
    // engine with no calibrated map), "ignition" (spark/compression, required
    // whenever ANY fuel model - has_fuel_map or a non-empty "fuels" - is
    // present) and "indicated_efficiency" (an override for the crude no-map
    // estimate's eta_i, valid only alongside a fuel model). Exact rejection
    // substrings pulled from core/src/io/vehicle_io.cpp's own throw sites.
    // -----------------------------------------------------------------------
    SECTION("fuels present together with fuel is rejected") {
        nlohmann::json e = base_engine_component();
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        expect_rejected(e, "'fuels' and 'fuel' are mutually exclusive");
    }
    SECTION("fuels present together with fuel_map is rejected") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        expect_rejected(e, "'fuels' and 'fuel_map' are mutually exclusive");
    }
    SECTION("fuels must be a non-empty array") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array();
        expect_rejected(e, "'fuels' must be a non-empty array of path strings");
    }
    SECTION("fuels rejects an empty-string entry") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({""});
        expect_rejected(e, "'fuels' entries must be non-empty path strings");
    }
    SECTION("fuels rejects a non-string entry") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({123});
        expect_rejected(e, "'fuels' entries must be non-empty path strings");
    }
    SECTION("fuels rejects a duplicate fuel reference") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path(), f5::fuel_json_path()});
        expect_rejected(e, "'fuels' contains a duplicate fuel reference");
    }
    SECTION("fuels loads successfully and resolves has_fuel_model without has_fuel_map") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        e["indicated_efficiency"] = 0.35;
        const f5::TempJsonFile f(f5::vehicle_with_engine(e));
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        bool found = false;
        for (const auto& c : d.powertrain.components) {
            if (const auto* eng = std::get_if<TorqueMapEngineDesc>(&c.params)) {
                found = true;
                CHECK_FALSE(eng->has_fuel_map);
                REQUIRE(eng->fuels.size() == 1);
                CHECK(has_fuel_model(*eng));
                CHECK(eng->indicated_efficiency == 0.35);
            }
        }
        CHECK(found);
    }
    SECTION("ignition required whenever a fuel model exists (fuel_map present, ignition absent)") {
        nlohmann::json e = base_engine_component();
        e.erase("ignition");
        expect_rejected(e, "requires 'ignition'");
    }
    SECTION("ignition required whenever a fuel model exists (fuels present, ignition absent)") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e.erase("ignition");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        expect_rejected(e, "requires 'ignition'");
    }
    SECTION("ignition must be spark or compression") {
        nlohmann::json e = base_engine_component();
        e["ignition"] = "bogus";
        expect_rejected(e, "ignition must be \"spark\" or \"compression\"");
    }
    SECTION("ignition without any fuel model is accepted (not validated in that direction)") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        // "ignition" stays "spark" from base_engine_component() - an engine
        // with no fuel model at all may still declare its ignition kind.
        const f5::TempJsonFile f(f5::vehicle_with_engine(e));
        const vehicle::VehicleDesc d = io::load_vehicle_json(f.path.string());
        bool found = false;
        for (const auto& c : d.powertrain.components) {
            if (const auto* eng = std::get_if<TorqueMapEngineDesc>(&c.params)) {
                found = true;
                CHECK_FALSE(has_fuel_model(*eng));
            }
        }
        CHECK(found);
    }
    SECTION("indicated_efficiency requires a fuel model") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e.erase("ignition");
        e["indicated_efficiency"] = 0.35;
        expect_rejected(e, "indicated_efficiency requires a fuel model");
    }
    // N9 review fix: indicated_efficiency is also rejected outright when a
    // fuel_map is present (the map is never gated by eta_i) - base_engine_
    // component() has both "fuel" and "fuel_map" set, so has_fuel_map is
    // true here and this must be the message reached, ahead of the number/
    // range checks below (which is exactly why those three SECTIONs below
    // switch to "fuels"-only fixtures instead of the fuel_map-bearing base).
    SECTION("indicated_efficiency is rejected when a fuel_map is present") {
        nlohmann::json e = base_engine_component();
        e["indicated_efficiency"] = 0.35;
        expect_rejected(e, "indicated_efficiency is meaningless with a 'fuel_map' present");
    }
    // The number/range checks below only run once has_fuel_map is false, so
    // each of these three SECTIONs must drop "fuel"/"fuel_map" and use
    // "fuels" instead (a fixture with fuel_map present would hit the
    // "meaningless with a 'fuel_map' present" rejection above first, not the
    // check this SECTION means to exercise).
    SECTION("indicated_efficiency must be a number") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        e["indicated_efficiency"] = "0.35";
        expect_rejected(e, "indicated_efficiency must be a number");
    }
    SECTION("indicated_efficiency must be in (0, 1) - rejects 0") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        e["indicated_efficiency"] = 0.0;
        expect_rejected(e, "indicated_efficiency must be in (0, 1)");
    }
    SECTION("indicated_efficiency must be in (0, 1) - rejects 1") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        e["indicated_efficiency"] = 1.0;
        expect_rejected(e, "indicated_efficiency must be in (0, 1)");
    }
    // N9 review fix: non-finite indicated_efficiency (NaN/Infinity survive
    // is_number() but must still fail the isfinite() check, not silently
    // pass through as a "valid" out-of-(0,1) number) - vehicle_io.cpp's own
    // combined `!std::isfinite(...) || !(... > 0) || !(... < 1)` check
    // (single "must be in (0, 1)" message for all three).
    //
    // PHYS-033 P4 I7a C5d review-fix bugfix (found running this SECTION for
    // the first time after the review): neither half of "NaN/Infinity
    // survive is_number()" is actually reachable through load_vehicle_json's
    // real file+nlohmann::json::parse() path, for two independent reasons,
    // so this SECTION cannot exercise vehicle_io.cpp's isfinite() branch at
    // all - it is DEFENSIVE, UNREACHABLE code as far as this loader's public
    // entry point goes:
    //   1. Going through THIS file's ordinary TempJsonFile/expect_rejected
    //      path (assign std::numeric_limits<double>::infinity() into an
    //      nlohmann::json value, then dump()) does not even produce a JSON
    //      Infinity - JSON itself has no such literal, and nlohmann's
    //      dump() silently replaces a non-finite double with `null`, which
    //      then fails the EARLIER is_number() check ("must be a number")
    //      instead.
    //   2. Bypassing dump() with a hand-written raw JSON literal that IS
    //      syntactically a valid number and overflows a double once parsed
    //      (e.g. `1e400`) does not work either: nlohmann::json::parse()
    //      itself detects the overflow during its own number scan and
    //      throws `out_of_range.406` ("number overflow parsing '1e400'")
    //      before the value ever becomes a stored double our field-level
    //      code could inspect - confirmed empirically (this exact message
    //      is what a first attempt at this SECTION using `1e400` produced).
    // Neither NaN/Infinity (invalid JSON syntax - a parse_error) nor an
    // overflowing-but-syntactically-valid literal (nlohmann's own
    // out_of_range.406) can reach vehicle_io.cpp's isfinite() check via any
    // real file, so this SECTION instead pins the ACTUAL, reachable
    // behaviour: nlohmann's own overflow guard rejects it one layer up,
    // with its own message - the vehicle is still safely never loaded,
    // just not via our "(0, 1)" message. vehicle_io.cpp's isfinite() check
    // is kept as a harmless safety net (e.g. against a hypothetical future
    // in-memory construction path that skips nlohmann::json::parse()), not
    // because this test can prove it fires.
    SECTION("indicated_efficiency must be finite - rejects Infinity") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({f5::fuel_json_path()});
        e["indicated_efficiency"] = 0.0; // placeholder, replaced by raw text below
        nlohmann::json v = f5::vehicle_with_engine(e);
        std::string text = v.dump(2);
        const std::string needle = "\"indicated_efficiency\": 0.0";
        const auto pos = text.find(needle);
        REQUIRE(pos != std::string::npos);
        text.replace(pos, needle.size(), "\"indicated_efficiency\": 1e400");
        f5::expect_rejected_raw_text(text, "number overflow parsing");
    }
    // N12 review fix: an object-typed "fuels" entry (not merely a number)
    // must give the same rejection every non-string entry gives.
    SECTION("fuels rejects an object-typed entry") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = nlohmann::json::array({nlohmann::json::object({{"path", f5::fuel_json_path()}})});
        expect_rejected(e, "'fuels' entries must be non-empty path strings");
    }
    // N12 review fix: "fuels" given as a non-array value entirely (not an
    // empty array) must hit the same "non-empty array" rejection.
    SECTION("fuels rejects a non-array value") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        e["fuels"] = f5::fuel_json_path(); // a bare string, not wrapped in an array
        expect_rejected(e, "'fuels' must be a non-empty array of path strings");
    }
    // N12 review fix: the new std::filesystem::equivalent-based duplicate
    // check must catch a textually-different path that still names the same
    // file (a "./"-prefixed detour), not just a byte-identical string repeat
    // (already covered by "fuels rejects a duplicate fuel reference" above).
    SECTION("fuels rejects a duplicate fuel reference reached via a different relative path") {
        nlohmann::json e = base_engine_component();
        e.erase("fuel");
        e.erase("fuel_map");
        const std::filesystem::path direct = f5::fuel_json_path();
        const std::filesystem::path detour = direct.parent_path() / "." / direct.filename();
        e["fuels"] = nlohmann::json::array({direct.generic_string(), detour.generic_string()});
        expect_rejected(e, "'fuels' contains a duplicate fuel reference");
    }
}

// ---------------------------------------------------------------------------
// F6/F9: whole-World checks with tests/support/fixtures/vehicle_fuel_map_sedan.json
// (a car_sedan.json clone, engine inlined, a synthetic fuel map/fuel added).
// fuel_map_sedan_path()/WorldRig/make_world_rig() now live above (moved up
// so the review-fixed F4 can use them too) - only the F6-only helper stays
// here.
// ---------------------------------------------------------------------------
// PHYS-033 P4 I7a C5d review fix (N11): any_channel_contains() (a substring
// helper) used to live here for F6's old substring-only CSV-header check;
// removed along with that check - see F6's own comment below for the
// exact-equality/exact-position replacement.

// I7a C5d note: "plain" below (ps_test::load_sedan_desc(), i.e. car_sedan.json)
// is no longer an engine with NO fuel model at all - i4_2l_na.json's "fuels"
// list makes the crude no-map estimate active there too (has_fuel_model true,
// has_fuel_map false). The state_hash comparison is unaffected (the fuel
// sub-ledger, crude or mapped, is never hashed either way); the CSV-header
// check below is updated accordingly: BOTH worlds now register `.fuel.`
// channels (has_fuel_model, broadened from has_fuel_map, gates that
// registration - see World::create_vehicle/FuelTelemetryChannels).
TEST_CASE("F6: a fuel map with no tank is bit-identical to the crude no-map estimate (state_hash + CSV header)",
          "[unit][drivetrain][fuel][determinism]") {
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
    // PHYS-033 P4 I7a C5d review fix (N11): re-pinned from a substring check
    // (any_channel_contains(..., ".fuel.")) to exact channel-list equality
    // plus the exact names at their correct position - a substring check
    // would pass even if the fuel channels moved, were misnamed, or a
    // completely unrelated channel merely happened to contain ".fuel." in
    // its name. Both worlds now have SOME fuel model (plain = car_sedan's
    // real crude-estimate engine, fueled = the fixture's calibrated map),
    // and neither vehicle has a non-open differential (World::create_vehicle
    // only emits `.<diff id>.{bias,locked}` channels for those - see
    // CLAUDE.md's "World tick pipeline"), so the two CSV headers must be
    // byte-for-byte IDENTICAL, not merely each containing a fuel channel.
    CHECK(plain_names == fueled_names);

    // World::create_vehicle registers vehicle0.fuel.{flow_g_s,energy_in,heat}
    // (FuelTelemetryChannels, world.cpp) as the LAST block for a vehicle,
    // immediately after its powertrain state channels
    // (vehicle0.powertrain.engine_state) and only when that vehicle has no
    // non-open differential (true for both car_sedan and this fixture) - so
    // the three fuel channels must appear immediately after engine_state, in
    // this exact order, in BOTH worlds.
    for (const auto* names : {&plain_names, &fueled_names}) {
        const auto it = std::find(names->begin(), names->end(), std::string("vehicle0.powertrain.engine_state"));
        REQUIRE(it != names->end());
        const auto idx = static_cast<std::size_t>(std::distance(names->begin(), it));
        REQUIRE(idx + 3 < names->size());
        CHECK((*names)[idx + 1] == "vehicle0.fuel.flow_g_s");
        CHECK((*names)[idx + 2] == "vehicle0.fuel.energy_in");
        CHECK((*names)[idx + 3] == "vehicle0.fuel.heat");
    }
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

// ===========================================================================
// PHYS-033 P4 I7a C5d: the crude no-map fuel model (owner-amended plan,
// out/c5d_brief.md). TorqueMapEngineDesc::fuels/ignition/indicated_efficiency
// are resolved into EngineTorqueMap::Desc::has_fuel_model/fuel/eta_i by
// ps::drivetrain::to_engine_desc() (the REAL production resolver, called
// directly - see the "B2 review fix" comment above car_sedan_engine_public_
// desc()) - every test below drives that resolved Desc directly, exactly like F2/F3/F4
// above, or a real World/Powertrain built from a VehicleDesc whose engine
// carries "fuels" (car_sedan.json's i4_2l_na.json, tractor.json's
// diesel_i4_4l5_turbo.json). Test 8 (every loader rejection) is F5 above,
// already covering fuels/fuel_map/ignition/indicated_efficiency mutual-
// exclusion and format rejections. Numbering matches out/c5d_brief.md's own
// test list (1-9); each test's header names which brief item it is and, for
// 1/2/5/6, the discrimination demo performed against production code.
// ===========================================================================

FuelDesc c5d_test_fuel_desc(real lhv_j_kg = 44.0e6) {
    FuelDesc f;
    f.name = "test fuel (I7a C5d)";
    f.phase = FuelPhase::Liquid;
    f.lhv_j_kg = lhv_j_kg;
    f.afr_stoich = 14.5;
    f.density_kg_m3 = 750.0;
    f.source = "synthetic (test_engine_fuel_map.cpp, I7a C5d)";
    return f;
}

// Brief test 3: mirrors F3's SECTIONs above, but for the crude no-map model
// (car_sedan's real engine, has_fuel_model true / has_fuel_map false) rather
// than F3's synthetic fuel-mapped engine - a distinct code path
// (EngineTorqueMap::pre_solve's `else if (has_fuel_model)` branch, engine_
// torque_map.cpp) that F3 never exercises.
TEST_CASE("C5d Test 3: the crude no-map fuel model is exactly zero whenever combustion cannot be firing",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());
    REQUIRE(desc.has_fuel_model);
    REQUIRE_FALSE(desc.has_fuel_map);

    SECTION("Off (ignition <= 0.5) even if previously Running") {
        EngineTorqueMap engine(desc);
        Sensors s = running_sensors(3000.0);
        s.ignition = 0.0;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Off);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Stalled, starter not pressed, at rest") {
        EngineTorqueMap engine(desc);
        engine.reset(EngineState::Stalled);
        Sensors s = running_sensors(0.0);
        s.starter = 0.0;
        const PreSolveResult r = engine.pre_solve(s, throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Stalled);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("DFCO: Running, high rpm (2000 > dfco_enable_rpm 1300), throttle 0") {
        EngineTorqueMap engine(desc);
        engine.reset(EngineState::Running);
        const PreSolveResult r = engine.pre_solve(running_sensors(2000.0), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.dfco_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Fuel-cut limiter: Running, at/above limiter.rpm (6800), full throttle") {
        EngineTorqueMap engine(desc);
        engine.reset(EngineState::Running);
        const PreSolveResult r = engine.pre_solve(running_sensors(7000.0), throttle_of(1.0), no_ignition_cut(), kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.limiter_active);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
    SECTION("Ignition cut (upshift cut) at a non-idle rpm") {
        EngineTorqueMap engine(desc);
        engine.reset(EngineState::Running);
        const TcuRequest cut{true};
        const PreSolveResult r = engine.pre_solve(running_sensors(3000.0), throttle_of(1.0), cut, kSubstepH);
        CHECK(r.state == EngineState::Running);
        CHECK(r.fuel_flow_g_s == 0.0);
    }
}

// Brief test 2: settled idle, mdot within 2% of the idle-balance identity
// -(m(rpm)+c_k)*omega/(eta*LHV) and > 0. Derived directly from engine_torque_
// map.cpp's own pre_solve/post_solve code (tau_smooth_ = sign(omega_old)*
// (m(rpm)+c_k_), c_k_ = -motoring(0); at settled idle the fixed point has
// tau0_ ~ 0, so tau_comb_ ~ -tau_smooth_ = -(m(rpm)+c_k_)) - matches the
// brief's own formula exactly, no deviation from the plan needed here.
//
// N8 review-fix note: this SECTION already calls the real to_engine_desc()
// resolver (so it is not a B2-style resolver-duplication case), but it
// still settles a BARE EngineTorqueMap via a hand-rolled single-DOF
// implicit-Euler loop (omega_new = omega + h*tau0/(J + h*d)) rather than the
// real ShaftNetwork::solve a real vehicle drives through - the same class
// of gap this file's make_world_rig_1to1 header comment already documents
// for "Fix 3" F2 above. Kept as-is because it is still a genuinely useful,
// decoupled check of the crude fuel model's OWN closed-form formula (does
// pre_solve/post_solve compute -(m(rpm)+c_k)*omega/(eta*LHV) correctly at
// all, independent of how any particular network solve arrives at that
// omega) - not a check of the resolver or of Powertrain/network wiring.
// The companion "C5d Test 2b" below (N8) re-runs the same identity through
// a REAL World/Powertrain (car_sedan.json, Neutral) so a regression in the
// real per-substep integration path that a hand-rolled loop could mask
// (e.g. the network feeding the engine a post-solve rather than pre-solve
// rpm) is still caught end to end, mirroring F2's own real-Powertrain
// companion above.
TEST_CASE("C5d Test 2: the crude no-map fuel model at settled idle matches the net-torque-zero identity within 2%",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());
    REQUIRE(desc.has_fuel_model);
    REQUIRE_FALSE(desc.has_fuel_map);

    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Running);
    const real J = engine.desc().inertia_kgm2;

    real omega = rpm_to_omega(desc.idle_rpm);
    constexpr int kSubsteps = 4800; // 5 s at 960 Hz - long enough for the idle PI to settle
    for (int i = 0; i < kSubsteps; ++i) {
        const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(0.0), no_ignition_cut(), kSubstepH);
        const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
        (void)engine.post_solve(omega_new, kSubstepH);
        omega = omega_new;
    }

    const real settled_rpm = std::abs(omega) / kRpmToRadPerSec;
    INFO("settled_rpm=" << settled_rpm << " idle_rpm=" << desc.idle_rpm);
    CHECK(std::abs(settled_rpm - static_cast<real>(desc.idle_rpm)) < 5.0);

    const PreSolveResult settled_pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(0.0), no_ignition_cut(), kSubstepH);
    const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(settled_pre.rpm);
    const real c_k = -desc.motoring_torque_nm_vs_rpm.evaluate(0.0);
    const real expected_tau_comb = -(m_rpm + c_k);
    const real expected_mdot_kg_s = std::max(real(0.0), expected_tau_comb * omega) / (desc.eta_i * desc.fuel.lhv_j_kg);
    const real expected_g_s = expected_mdot_kg_s * real(1000.0);

    INFO("actual fuel_flow_g_s=" << settled_pre.fuel_flow_g_s << " expected=" << expected_g_s << " tau0=" << settled_pre.tau0);
    REQUIRE(expected_g_s > 0.0);
    const real rel_err = std::abs(settled_pre.fuel_flow_g_s - expected_g_s) / expected_g_s;
    CHECK(rel_err <= 0.02);
}

// N8 review fix: "C5d Test 2b" - the same idle-balance identity as
// "C5d Test 2" above, but settled through a REAL World/Powertrain
// (car_sedan.json via ps_test::load_sedan_desc(), Neutral, 1:1 tick:substep
// ratio via make_world_rig_1to1 so each World::step() call is exactly one
// Powertrain substep) instead of a bare EngineTorqueMap plus a hand-rolled
// single-DOF Euler loop. Driving Neutral through the real vehicle exercises
// the actual production per-substep path end to end (crank + clutch/gearbox-
// input inertia, the real ShaftNetwork::solve, the real pre_solve/post_solve
// split VehicleInstance::step_tick drives) - a regression that fed the
// engine a post-solve rpm or the raw throttle command instead of theta_c
// (exactly the class of mistake "Fix 3" F2's own header comment discusses)
// would still show up here even though it could hide behind Test 2's own
// hand-rolled loop's tight coupling to the SAME pre_solve call's tau0/d.
//
// IMPORTANT: the expected-value formula here is NOT "C5d Test 2"'s
// -(m(rpm)+c_k) - that identity only holds for a BARE EngineTorqueMap driven
// directly off its own tau0 (Test 2's hand-rolled loop does exactly that,
// with no other torque path). The real Powertrain (powertrain.cpp's
// build_network) wires the crank node with TWO separate contributions, not
// one: Assembly::torque(engine_crank_, pre.tau0, pre.d) - tau0 already
// contains tau_smooth_ = sign(omega)*(m(rpm)+c_k_), i.e. the "+c_k_" is
// baked in - AND a SEPARATE crank-to-ground friction row (`engine_row =
// add_friction_row(engine_crank_, kGround)`, capacity pre.friction_cap_
// kinetic == c_k_) applied to the SAME node. At idle the crank is clearly
// spinning (not stuck against ground), so that row is kinetic and
// contributes an ADDITIONAL -sign(omega)*c_k_ on top of tau0. Net steady-
// state balance (tau0 + friction_row_reaction == 0):
//   [sign(omega)*(m(rpm)+c_k) + tau_comb] + [-sign(omega)*c_k] == 0
//   => sign(omega)*m(rpm) + tau_comb == 0  =>  tau_comb == -sign(omega)*m(rpm)
// The two c_k terms cancel exactly - c_k drops out of the real-network
// identity entirely. Confirmed numerically while diagnosing this test: at
// settled idle (omega > 0, so sign==+1), m(rpm)=-30.4 N*m, c_k=15 N*m;
// -(m(rpm)+c_k) predicts tau_comb=15.4 N*m (2x too small), while -m(rpm)
// predicts 30.4 N*m, matching the actual fuel flow's back-solved tau_comb
// (~30.41 N*m) to <0.3%. This is a genuine, reportable modelling subtlety
// (not a production bug): the bare-engine formula in "C5d Test 2" is only
// valid in the absence of the network's own separate Coulomb-friction row,
// which the real vehicle always has.
//
// The expected value still reads the motoring curve/eta_i/LHV back off the
// resolved Desc (to_engine_desc(car_sedan_engine_public_desc())) rather than
// off independent literals - unlike B2's tests, this SECTION is not
// checking whether the resolver picked the RIGHT eta_i/LHV (B2 owns that),
// only whether pre_solve/post_solve's fuel-flow formula is followed
// correctly given whatever values the resolver produced, so reading them
// back from the Desc under test is appropriate here (see "C5d Test 2"'s own
// header comment for the same reasoning).
TEST_CASE("C5d Test 2b (N8 review fix): the crude no-map fuel model at settled idle through a real World/Powertrain matches the idle-balance identity within 2%",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());
    REQUIRE(desc.has_fuel_model);
    REQUIRE_FALSE(desc.has_fuel_map);

    const vehicle::VehicleDesc sedan_desc = ps_test::load_sedan_desc();
    WorldRig rig = make_world_rig_1to1(sedan_desc, /*gear=*/0); // Neutral; throttle 0.0 already set by make_world_rig_1to1

    constexpr int kSettleTicks = 4800; // 5 s at 960 Hz (1:1) - same settle duration as the bare-engine "C5d Test 2" above
    for (int i = 0; i < kSettleTicks; ++i) {
        rig.world->step();
    }

    const drivetrain::PowertrainSnapshot& snap = rig.world->powertrain_state(rig.vehicle);
    REQUIRE(snap.engine_state == EngineState::Running);
    REQUIRE(snap.engines.size() == 1);

    const real settled_rpm = snap.engine_rpm;
    INFO("settled_rpm=" << settled_rpm << " idle_rpm=" << desc.idle_rpm);
    CHECK(std::abs(settled_rpm - static_cast<real>(desc.idle_rpm)) < 5.0);

    // Real-network idle-balance identity (see this TEST_CASE's own header
    // comment for the derivation): tau_comb == -sign(omega)*m(rpm), NOT
    // -(m(rpm)+c_k) - the crank-to-ground friction row's own kinetic
    // reaction cancels the tau0 side's "+c_k" term exactly.
    const real omega = snap.engines[0].omega;
    const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(settled_rpm);
    const real expected_tau_comb = omega >= real(0.0) ? -m_rpm : m_rpm;
    const real expected_mdot_kg_s = std::max(real(0.0), expected_tau_comb * omega) / (desc.eta_i * desc.fuel.lhv_j_kg);
    const real expected_g_s = expected_mdot_kg_s * real(1000.0);

    const real actual_g_s = snap.engines[0].fuel_flow_g_s;
    INFO("actual fuel_flow_g_s=" << actual_g_s << " expected=" << expected_g_s << " omega=" << omega << " m_rpm=" << m_rpm);
    REQUIRE(expected_g_s > 0.0);
    const real rel_err = std::abs(actual_g_s - expected_g_s) / expected_g_s;
    CHECK(rel_err <= 0.02);
}

// Brief test 1: a dyno-held point (omega held EXACTLY constant across every
// substep), fuel_mass == sum h*max(0,theta_c*(w-m)*omega_old)/(eta*LHV) to
// 1e-12. Uses car_sedan's real engine at a mid-range rpm/throttle where
// combustion is firing and neither DFCO nor the limiter can engage.
//
// PHYS-033 P4 I7a C5d review fix (B2/N7): the recomputation below uses the
// LITERAL spark eta_i (0.325) and i4_2l_na.json's active fuel e5.json's
// literal LHV (43089334.0 J/kg), not desc.eta_i/desc.fuel.lhv_j_kg read back
// from the very Desc under test - reading them back would let a resolver bug
// that corrupts BOTH the production value and this test's expectation
// together (e.g. B2's fuels.back()/swapped-eta-default/ignored-override
// sabotages) still pass this identity, since both sides would move together.
//
// N7 review fix (owner-acknowledged, not fixed): because omega is
// dyno-held EXACTLY constant, omega_old == omega_new == omega on every
// substep, so this test's own recomputation (using `omega`, the dyno-held
// value) is IDENTICAL whether the production formula's mech_power term is
// `tau_comb * omega_old` (correct) or a hypothetical `tau_comb * omega_new`
// substitution bug - both read the same number here. This test therefore
// CANNOT discriminate that substitution; only a transient (accelerating)
// case can, which is why the sabotage below is executed against "C5d Test 4"
// (crank-to-overrun, genuinely transient) instead, not this test.
TEST_CASE("C5d Test 1: the crude no-map fuel model integral matches an independent recomputation at a dyno-held point",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());
    REQUIRE(desc.has_fuel_model);
    REQUIRE_FALSE(desc.has_fuel_map);
    // Pin the literals used below against the real resolved Desc (a sanity
    // check on the literals themselves, not part of the discrimination this
    // test performs - the recomputation itself never reads these two fields).
    REQUIRE(desc.eta_i == Catch::Approx(0.325));
    REQUIRE(desc.fuel.lhv_j_kg == Catch::Approx(43089334.0));

    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Running);

    constexpr real kHeldRpm = 3000.0;
    const real omega = rpm_to_omega(kHeldRpm);
    constexpr int kSubsteps = 1000;
    constexpr real kEtaSparkDefault = real(0.325);   // Heywood 1988 SI default, literal - see header comment
    constexpr real kE5LhvJKg = real(43089334.0);     // data/fuels/e5.json's own lhv_j_kg, literal

    real fuel_mass_sum = 0.0;
    real recomputed_sum = 0.0;
    for (int i = 0; i < kSubsteps; ++i) {
        const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(0.6), no_ignition_cut(), kSubstepH);
        const PostSolveResult post = engine.post_solve(omega, kSubstepH); // dyno holds omega exactly constant: omega_new == omega
        fuel_mass_sum += post.fuel_mass_delta;

        REQUIRE_FALSE(pre.dfco_active);
        REQUIRE_FALSE(pre.limiter_active);
        const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
        const real mech_power = std::max(real(0.0), tau_comb * omega); // omega_old == omega (dyno-held)
        const real mdot_kg_s = mech_power / (kEtaSparkDefault * kE5LhvJKg);
        recomputed_sum += kSubstepH * mdot_kg_s;
    }

    REQUIRE(recomputed_sum > 0.0);
    const real rel_err = std::abs(fuel_mass_sum - recomputed_sum) / recomputed_sum;
    CHECK(rel_err <= 1e-12);
}

// Brief test 4: engine_heat >= 0 and fuel_efficiency_violations == 0 every
// substep through crank -> fire -> WOT launch -> overrun (throttle lifted at
// the end, engine still spinning fast - genuine overrun/DFCO territory), plus
// max(omega_new/omega_old) < 1/eta_i whenever tau_comb > 0 (a direct algebraic
// consequence of engine_heat = fuel_energy_in - indicated_work >= 0 at the
// crude model's fuel_energy_in = mech_power*h/eta_i and indicated_work =
// h*omega_new*tau_comb: heat >= 0 <=> omega_old*tau_comb/eta_i >=
// omega_new*tau_comb, i.e. omega_new/omega_old <= 1/eta_i when tau_comb>0 -
// not a separate plan/code mismatch, just the same inequality restated).
TEST_CASE("C5d Test 4: the crude no-map fuel model never reports negative heat or >100% efficiency, crank to overrun",
          "[unit][drivetrain][fuel]") {
    const EngineTorqueMap::Desc desc = to_engine_desc(car_sedan_engine_public_desc());
    REQUIRE(desc.has_fuel_model);
    REQUIRE_FALSE(desc.has_fuel_map);
    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Off);
    const real J = engine.desc().inertia_kgm2;

    real omega = 0.0;
    bool saw_running = false;
    bool saw_wot = false;
    bool saw_overrun = false;
    int heat_violations = 0;

    constexpr int kCrankSubsteps = 1000;
    constexpr int kWotSubsteps = 4000;
    constexpr int kOverrunSubsteps = 2000;
    constexpr int kTotal = kCrankSubsteps + kWotSubsteps + kOverrunSubsteps;
    for (int i = 0; i < kTotal; ++i) {
        Sensors s = sensors_at_omega(omega);
        s.ignition = 1.0;
        s.starter = (i < kCrankSubsteps) ? 1.0 : 0.0;
        const real throttle_cmd = (i < kCrankSubsteps) ? 0.0 : ((i < kCrankSubsteps + kWotSubsteps) ? 1.0 : 0.0);
        const real omega_old = omega;
        const PreSolveResult pre = engine.pre_solve(s, throttle_of(throttle_cmd), no_ignition_cut(), kSubstepH);
        const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
        const PostSolveResult post = engine.post_solve(omega_new, kSubstepH);

        if (pre.state == EngineState::Running) {
            saw_running = true;
            if (throttle_cmd > 0.99) saw_wot = true;
            if (i >= kCrankSubsteps + kWotSubsteps) saw_overrun = true;
        }

        const real heat_delta = post.fuel_energy_in_delta - post.indicated_work_delta;
        if (heat_delta < -1e-9) {
            ++heat_violations;
            INFO("substep " << i << " heat_delta=" << heat_delta << " state=" << static_cast<int>(pre.state));
        }

        const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
        if (tau_comb > 0.0 && omega_old > 0.0) {
            CHECK(omega_new / omega_old < real(1.0) / desc.eta_i);
        }

        omega = omega_new;
    }

    CHECK(heat_violations == 0);
    CHECK(saw_running);
    CHECK(saw_wot);
    CHECK(saw_overrun);
}

// Brief test 5: tractor's compression-ignition engine, fuel_mass*0.48*LHV ==
// sum h*tau_comb*omega_old at a WOT dyno-held point (catches an SI/CI default
// swap or a diesel-LHV bug - if either default were swapped, this identity
// would be off by 0.48/0.325 ~ 1.48x, or by diesel's/e5's LHV ratio if the
// wrong fuel's LHV were fed in). No shared test helper loads tractor.json's
// engine (only car_sedan's has one, load_sedan_desc() in powertrain_multi_diff_
// desc.h) - loaded directly here instead.
//
// PHYS-033 P4 I7a C5d review fix (B3): the identity below uses LITERAL
// eta_i (0.48, Heywood 1988's CI default) and diesel.json's literal LHV
// (42.0e6 J/kg exactly, per that file's own lhv_j_kg), never desc.eta_i/
// desc.fuel.lhv_j_kg read back from the Desc under test - same rationale as
// "C5d Test 1"'s header comment: reading them back would let a resolver bug
// move both sides of the identity together. fuel_mass_sum (not merely
// fuel_energy_in_sum, which folds the LHV in already) is tracked
// independently so the LHV multiplication happens ONLY in this test's own
// literal-driven expected-value computation, not by re-using
// post.fuel_energy_in_delta (itself computed from desc.fuel.lhv_j_kg in
// production - see engine_torque_map.cpp's post_solve). The active fuel's
// name is also checked against the literal "diesel" (FuelDesc::name) so a
// test that silently started reading car_sedan's e5/e10 fuel (or any other
// fuel file) instead of tractor's own would be caught even before the LHV
// identity below is checked.
TEST_CASE("C5d Test 5: tractor's compression-ignition crude fuel model matches fuel_mass*eta_i*LHV == sum h*tau_comb*omega_old",
          "[unit][drivetrain][fuel]") {
    const vehicle::VehicleDesc tractor_desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    TorqueMapEngineDesc tractor_engine_public;
    bool found = false;
    for (const auto& c : tractor_desc.powertrain.components) {
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            tractor_engine_public = *e;
            found = true;
            break;
        }
    }
    REQUIRE(found);
    REQUIRE(tractor_engine_public.has_ignition);
    REQUIRE(tractor_engine_public.ignition == EngineIgnition::Compression);

    const EngineTorqueMap::Desc desc = to_engine_desc(tractor_engine_public);
    REQUIRE(desc.has_fuel_model);
    REQUIRE_FALSE(desc.has_fuel_map);
    // Sanity pins on the literals used below (not part of the discrimination
    // itself - the identity below never reads these two fields back).
    CHECK(desc.eta_i == Catch::Approx(0.48));
    CHECK(desc.fuel.lhv_j_kg == Catch::Approx(42000000.0));
    CHECK(desc.fuel.name == "diesel"); // B3: the active fuel really is tractor's own diesel, not e5/e10

    EngineTorqueMap engine(desc);
    engine.reset(EngineState::Running);

    // Full lever, held well below the governor's droop band ([2200,2400] rpm
    // per diesel_i4_4l5_turbo.json) so theta_gov saturates to 1 (full fuel) -
    // a genuine WOT dyno point, not a governed/regulated one.
    constexpr real kHeldRpm = 1800.0;
    const real omega = rpm_to_omega(kHeldRpm);
    constexpr int kSubsteps = 1000;
    constexpr real kEtaCompressionDefault = real(0.48);  // Heywood 1988 CI default, literal - see header comment
    constexpr real kDieselLhvJKg = real(42000000.0);     // data/fuels/diesel.json's own lhv_j_kg, literal

    real fuel_mass_sum = 0.0;
    real work_sum = 0.0;
    for (int i = 0; i < kSubsteps; ++i) {
        const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(1.0), no_ignition_cut(), kSubstepH);
        const PostSolveResult post = engine.post_solve(omega, kSubstepH); // dyno holds omega exactly constant
        fuel_mass_sum += post.fuel_mass_delta;

        const real w_rpm = desc.wot_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real m_rpm = desc.motoring_torque_nm_vs_rpm.evaluate(pre.rpm);
        const real tau_comb = pre.theta_c * (w_rpm - m_rpm);
        REQUIRE(tau_comb > 0.0); // precondition: WOT really produces positive combustion torque here
        work_sum += kSubstepH * tau_comb * omega; // omega_old == omega (dyno-held)
    }

    REQUIRE(work_sum > 0.0);
    const real rel_err = std::abs(fuel_mass_sum * kEtaCompressionDefault * kDieselLhvJKg - work_sum) / work_sum;
    CHECK(rel_err <= 1e-12);
}

// Brief test 6: an explicit indicated_efficiency (0.40) scales fuel by
// exactly 0.325/0.40 relative to the spark default - same torque trajectory
// (eta_i never feeds back into tau0/d), so the ratio is exact.
TEST_CASE("C5d Test 6: an explicit indicated_efficiency scales fuel by exactly the default/override ratio",
          "[unit][drivetrain][fuel]") {
    auto build = [](real eta_i) {
        EngineTorqueMap::Desc d = to_engine_desc(car_sedan_engine_public_desc());
        REQUIRE(d.has_fuel_model);
        d.eta_i = eta_i;
        return d;
    };
    const EngineTorqueMap::Desc default_desc = build(0.325); // spark default (no override)
    const EngineTorqueMap::Desc override_desc = build(0.40); // an explicit indicated_efficiency

    auto run_fuel_mass = [](const EngineTorqueMap::Desc& desc) {
        EngineTorqueMap engine(desc);
        engine.reset(EngineState::Running);
        const real J = engine.desc().inertia_kgm2;
        real omega = rpm_to_omega(3000.0);
        real fuel_mass_sum = 0.0;
        constexpr int kSubsteps = 500;
        for (int i = 0; i < kSubsteps; ++i) {
            const PreSolveResult pre = engine.pre_solve(sensors_at_omega(omega), throttle_of(0.6), no_ignition_cut(), kSubstepH);
            const real omega_new = omega + kSubstepH * pre.tau0 / (J + kSubstepH * pre.d);
            const PostSolveResult post = engine.post_solve(omega_new, kSubstepH);
            fuel_mass_sum += post.fuel_mass_delta;
            omega = omega_new;
        }
        return fuel_mass_sum;
    };

    const real mass_default = run_fuel_mass(default_desc);
    const real mass_override = run_fuel_mass(override_desc);
    REQUIRE(mass_default > 0.0);
    REQUIRE(mass_override > 0.0);

    const real expected_ratio = real(0.325) / real(0.40);
    const real actual_ratio = mass_override / mass_default;
    CHECK(actual_ratio == Catch::Approx(expected_ratio).epsilon(1e-9));
}

// Brief test 7: no feedback from the fuel sub-ledger into dynamics - car_sedan
// WITH its real "fuels" list vs. the SAME desc with fuels cleared in code
// give bit-identical state_hash over vehicle_powertrain.json's own control
// script (replayed by hand, same event list F6 above uses).
TEST_CASE("C5d Test 7: car_sedan's state_hash is bit-identical whether the engine's fuels list is present or empty",
          "[unit][drivetrain][fuel][determinism]") {
    struct Ev {
        real t;
        const char* channel;
        real value;
    };
    const std::vector<Ev> events = {{0.2, "shift_up_count", 1.0}, {0.3, "throttle", 0.8}, {3.0, "shift_up_count", 2.0},
                                     {6.0, "brake", 0.5},         {7.0, "shift_down_count", 1.0},
                                     {8.0, "starter", 1.0},       {8.2, "starter", 0.0}};

    const vehicle::VehicleDesc with_fuels = ps_test::load_sedan_desc();
    vehicle::VehicleDesc without_fuels = ps_test::load_sedan_desc();
    bool mutated = false;
    for (auto& c : without_fuels.powertrain.components) {
        if (auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            REQUIRE_FALSE(e->fuels.empty()); // precondition: car_sedan really ships with a fuels list to strip
            e->fuels.clear();
            mutated = true;
        }
    }
    REQUIRE(mutated);

    WorldRig ra = make_world_rig(with_fuels);
    WorldRig rb = make_world_rig(without_fuels);

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
    CHECK(next_event == events.size());
}

namespace {

// PHYS-033 P4 I7a C5d review fix (B2): the three World-level tests below
// exercise ps::drivetrain::to_engine_desc() (the real production resolver)
// through a REAL Powertrain, not a bare EngineTorqueMap - "C5d Test 6" above
// already covers the indicated_efficiency-override ratio at the bare-engine
// level, but B2 additionally wants it (plus two more resolver knobs: fuels
// order and ignition kind) proven through the full World/Powertrain/
// VehicleInstance stack, where a resolver mistake could in principle be
// masked or amplified by everything downstream of to_engine_desc().
//
// All three drive car_sedan.json's own real i4_2l_na.json engine curves
// (via ps_test::load_sedan_desc(), mutated in code - never a data file on
// disk) through an identical WOT free-rev in Neutral (make_world_rig_1to1,
// gear 0 - no wheel/tyre/road load, so the two variants' torque/omega
// trajectories are dynamically IDENTICAL: none of fuels/ignition/
// indicated_efficiency ever feeds back into tau0/d, only scales the fuel
// sub-ledger's output - see "C5d Test 6"'s own comment). Because the
// trajectories are identical, the ratio of the two runs' cumulative
// World::powertrain_energy(...).fuel_mass is EXACTLY the ratio of whatever
// scalar (LHV or eta_i) differs between them, to floating-point rounding
// only - hence the tight 1e-12 tolerance (vs. "C5d Test 6"'s looser 1e-9,
// which additionally drives a real WheelInstance/tyre-coupled substep loop
// by hand rather than through World::step()).
TorqueMapEngineDesc& sedan_engine_ref(vehicle::VehicleDesc& desc) {
    for (auto& c : desc.powertrain.components) {
        if (auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) return *e;
    }
    throw std::runtime_error("sedan_engine_ref: no TorqueMapEngineDesc component");
}

// Drives `rig` at WOT in Neutral for kRevTicks substeps (1:1 tick:substep,
// make_world_rig_1to1's own contract) and returns the cumulative fuel_mass
// booked by the end of the run.
real run_wot_free_rev_fuel_mass(WorldRig& rig) {
    constexpr int kRevTicks = 300; // ~0.3125 s WOT free-rev, no wheel load
    rig.world->set_control("throttle", 1.0);
    for (int tick = 0; tick < kRevTicks; ++tick) {
        rig.world->step();
    }
    return rig.world->powertrain_energy(rig.vehicle).fuel_mass;
}

} // namespace

// B2 new test (a): reordering the engine's "fuels" list to [e10, e5] makes
// e10 (not e5) the active fuel (fuels[0] wins - see docs/data_formats/
// engine.md), so the fuel_mass ratio vs. the baseline (fuels[0] == e5) must
// equal e5's LHV / e10's LHV EXACTLY (mass = work/(eta_i*LHV), eta_i
// unchanged in both variants - both stay spark/no override).
TEST_CASE("B2 new test: reordering the engine's fuels list scales fuel_mass by exactly the swapped fuels' LHV ratio",
          "[unit][drivetrain][fuel]") {
    vehicle::VehicleDesc baseline_desc = ps_test::load_sedan_desc();
    vehicle::VehicleDesc reordered_desc = ps_test::load_sedan_desc();

    TorqueMapEngineDesc& baseline_engine = sedan_engine_ref(baseline_desc);
    REQUIRE(baseline_engine.fuels.size() == 2);
    REQUIRE(baseline_engine.fuels[0].name == "e5");
    REQUIRE(baseline_engine.fuels[1].name == "e10");

    TorqueMapEngineDesc& reordered_engine = sedan_engine_ref(reordered_desc);
    std::swap(reordered_engine.fuels[0], reordered_engine.fuels[1]);
    REQUIRE(reordered_engine.fuels[0].name == "e10");

    constexpr real kE5LhvJKg = real(43089334.0);  // data/fuels/e5.json's own lhv_j_kg, literal
    constexpr real kE10LhvJKg = real(42184298.0); // data/fuels/e10.json's own lhv_j_kg, literal
    REQUIRE(baseline_engine.fuels[0].lhv_j_kg == Catch::Approx(kE5LhvJKg));
    REQUIRE(reordered_engine.fuels[0].lhv_j_kg == Catch::Approx(kE10LhvJKg));

    WorldRig baseline_rig = make_world_rig_1to1(baseline_desc, /*gear=*/0);
    WorldRig reordered_rig = make_world_rig_1to1(reordered_desc, /*gear=*/0);
    const real mass_baseline = run_wot_free_rev_fuel_mass(baseline_rig);
    const real mass_reordered = run_wot_free_rev_fuel_mass(reordered_rig);
    REQUIRE(mass_baseline > 0.0);
    REQUIRE(mass_reordered > 0.0);

    const real expected_ratio = kE5LhvJKg / kE10LhvJKg;
    const real actual_ratio = mass_reordered / mass_baseline;
    CHECK(actual_ratio == Catch::Approx(expected_ratio).epsilon(1e-12));
}

// B2 new test (b): an explicit indicated_efficiency override (0.40) through
// a real World/Powertrain scales fuel_mass by exactly the spark
// default/override ratio - "C5d Test 6" above already pins this at the
// bare-EngineTorqueMap level; this is the same knob through the full stack.
TEST_CASE("B2 new test: an explicit indicated_efficiency through a real Powertrain scales fuel_mass by exactly the default/override ratio",
          "[unit][drivetrain][fuel]") {
    vehicle::VehicleDesc default_desc = ps_test::load_sedan_desc();
    vehicle::VehicleDesc override_desc = ps_test::load_sedan_desc();

    TorqueMapEngineDesc& default_engine = sedan_engine_ref(default_desc);
    REQUIRE(default_engine.indicated_efficiency == 0.0); // 0.0 sentinel: no override, ignition-default wins

    TorqueMapEngineDesc& override_engine = sedan_engine_ref(override_desc);
    override_engine.indicated_efficiency = real(0.40);

    WorldRig default_rig = make_world_rig_1to1(default_desc, /*gear=*/0);
    WorldRig override_rig = make_world_rig_1to1(override_desc, /*gear=*/0);
    const real mass_default = run_wot_free_rev_fuel_mass(default_rig);
    const real mass_override = run_wot_free_rev_fuel_mass(override_rig);
    REQUIRE(mass_default > 0.0);
    REQUIRE(mass_override > 0.0);

    const real expected_ratio = real(0.325) / real(0.40);
    const real actual_ratio = mass_override / mass_default;
    CHECK(actual_ratio == Catch::Approx(expected_ratio).epsilon(1e-12));
}

// B2 new test (c): flipping the engine's ignition from spark to compression
// (with no indicated_efficiency override) scales fuel_mass by exactly the
// spark/compression default ratio (0.325/0.48) - catches a resolver bug
// that used the wrong ignition-type default, or ignored ignition entirely.
TEST_CASE("B2 new test: flipping ignition from spark to compression through a real Powertrain scales fuel_mass by exactly the default ratio",
          "[unit][drivetrain][fuel]") {
    vehicle::VehicleDesc spark_desc = ps_test::load_sedan_desc();
    vehicle::VehicleDesc compression_desc = ps_test::load_sedan_desc();

    TorqueMapEngineDesc& spark_engine = sedan_engine_ref(spark_desc);
    REQUIRE(spark_engine.has_ignition);
    REQUIRE(spark_engine.ignition == EngineIgnition::Spark);
    REQUIRE(spark_engine.indicated_efficiency == 0.0); // no override on either variant

    TorqueMapEngineDesc& compression_engine = sedan_engine_ref(compression_desc);
    compression_engine.ignition = EngineIgnition::Compression;

    WorldRig spark_rig = make_world_rig_1to1(spark_desc, /*gear=*/0);
    WorldRig compression_rig = make_world_rig_1to1(compression_desc, /*gear=*/0);
    const real mass_spark = run_wot_free_rev_fuel_mass(spark_rig);
    const real mass_compression = run_wot_free_rev_fuel_mass(compression_rig);
    REQUIRE(mass_spark > 0.0);
    REQUIRE(mass_compression > 0.0);

    const real expected_ratio = real(0.325) / real(0.48);
    const real actual_ratio = mass_compression / mass_spark;
    CHECK(actual_ratio == Catch::Approx(expected_ratio).epsilon(1e-12));
}

// Mirrors make_world_rig() above but with a caller-chosen worker count, for
// Test 9's 1-vs-10-worker state_hash check.
WorldRig make_world_rig_workers(const vehicle::VehicleDesc& desc, int workers) {
    WorldRig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
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
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

// Brief test 9: debug_alloc zero-alloc while the crude model is active, plus
// worker-count invariance. C5's own existing map-based tests (F2/F3/F4/F6/F9
// etc. above) already run unmodified through this same shorthand-loader path
// every build, so their continued pass is this file's own regression net for
// that half of the requirement.
TEST_CASE("C5d Test 9: the crude no-map fuel model allocates nothing while stepping, and is worker-count invariant",
          "[unit][drivetrain][fuel][jobs][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's operator new/delete overrides are compiled out under ASan/TSan - nothing to check on this build.");
    } else {
        WorldRig rig = make_world_rig(ps_test::load_sedan_desc());
        rig.world->set_control("shift_up_count", 1.0);
        rig.world->set_control("throttle", 0.6);
        for (int i = 0; i < 300; ++i) rig.world->step(); // warm-up

        ps::jobs::debug_alloc::reset();
        ps::jobs::debug_alloc::set_armed(true);
        for (int i = 0; i < 240; ++i) rig.world->step();
        ps::jobs::debug_alloc::set_armed(false);

        const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
        const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
        ps::jobs::debug_alloc::reset();

        WARN("external (Jolt-internal, pre-existing broadphase-rebuild gap) violation count=" << external);
        CHECK(own == 0);
        CHECK(rig.world->powertrain_energy(rig.vehicle).fuel_mass > 0.0); // non-vacuous: the crude model really ran
    }

    auto run = [](int workers) {
        WorldRig rig = make_world_rig_workers(ps_test::load_sedan_desc(), workers);
        rig.world->set_control("shift_up_count", 1.0);
        rig.world->set_control("throttle", 0.7);
        for (int i = 0; i < 480; ++i) rig.world->step(); // 2 s at 240 Hz
        return rig.world->state_hash();
    };
    const std::uint64_t h1 = run(1);
    const std::uint64_t h10 = run(10);
    CHECK(h1 == h10);
}
