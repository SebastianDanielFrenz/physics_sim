// tests/unit/test_simulated_engine_powertrain.cpp -- PHYS-033 P4 I7b C12:
// the mean-value SimulatedEngine wired into the Powertrain and the World.
//
// The whole-vehicle cases build car_sedan.json's VehicleDesc IN CODE with its
// torque-map engine swapped for tests/support/mvem_fixtures.h's electronic or
// mechanical fixture engine (same component id, same crank axis); no data
// file is added (data/engines/*_sim.json and data/vehicles/car_sedan_sim.json
// are the C17a data agent's). Cases, each with a guard shown to fail on a
// sabotaged build (numbers in out/i7b_c12_progress.md):
//   - idle: a settled World idle at the fixture's idle_rpm, the sound fields;
//   - T8/F9: an auto-clutch WOT launch plus lift with a tank - the per-tick
//     balance, the tank against the fuel ledger, the fuel identities, the
//     pumping/friction ledger fields; a tank whose fuel is NOT fuels[0]
//     burns ITS fuel (review R7);
//   - T3: the ambient sample drives the manifold (air per rev at the
//     altitude density), the electronic pedal law holds at altitude (the
//     derived tables are refilled), the refill happens on change only;
//   - T4c: the simulated engine reads the throttle stage's command, not the
//     raw pedal (a flat pedal_map / linkage_curve plateau);
//   - pairing: a throttle_control stage of the other kind is refused;
//   - determinism: 1 vs 10 workers with a mid-run ambient change hash
//     identically, zero own allocations;
//   - C12/C17a integration: the shipped data/vehicles/car_sedan_sim.json
//     through the production loader - idle, WOT in 1st and 2nd, no stall,
//     per-tick balance (review of C12 + C17a).

#include "drivetrain/engine_simulated.h"
#include "drivetrain/powertrain.h"

#include "mvem_fixtures.h"
#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/ambient.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
namespace fx = ps::test::mvem;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;
real w_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

const AmbientSample kIsa{101325.0, 288.15};
const AmbientSample kAlt{79495.2, 275.15}; // ISA at 2000 m

// A second fuel that is NOT the fixture's fuels[0] (values ASSUMED, an
// E10-like blend; only "different LHV and A/F" matters here).
FuelDesc blend() {
    FuelDesc f = fx::gasoline();
    f.name = "E10-like test blend";
    f.lhv_j_kg = 42.6e6;
    f.afr_stoich = 14.1;
    f.density_kg_m3 = 752.0;
    return f;
}

// car_sedan.json with its engine swapped for `eng` (id and crank axis kept).
// A mechanical engine gets auto_blip's throttle terms zeroed (the loader's
// mechanical-path rule; the upshift ignition cut stays). tank_fuel >= 0
// adds a fuel_tank "tank" holding eng.fuels[tank_fuel].
vehicle::VehicleDesc sim_sedan(SimulatedEngineDesc eng, int tank_fuel = -1, real tank_kg = 30.0) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    std::string engine_id;
    for (auto& c : d.powertrain.components) {
        if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            eng.crank_axis_local = tm->crank_axis_local;
            engine_id = c.id;
            c.params = eng;
            break;
        }
    }
    REQUIRE(!engine_id.empty());
    if (eng.throttle_input == ThrottleInput::Mechanical) {
        for (auto& ctl : d.controllers) {
            if (auto* tcu = std::get_if<ManualTcuDesc>(&ctl.params)) {
                tcu->auto_blip.kp_per_rpm = 0.0;
                tcu->auto_blip.max_throttle = 0.0;
                tcu->auto_blip.upshift_torque_ramp_s = 0.0;
            }
        }
    }
    if (tank_fuel >= 0) {
        FuelTankDesc t;
        t.fuel = eng.fuels[static_cast<std::size_t>(tank_fuel)];
        t.capacity_kg = 50.0;
        t.initial_kg = tank_kg;
        d.powertrain.components.push_back(ComponentDesc{"tank", t});
        d.powertrain.fluid.push_back(FluidLinkDesc{PortRef{"tank", "out", false}, PortRef{engine_id, "fuel", false}});
    }
    return d;
}

vehicle::VehicleDesc with_throttle_stage(vehicle::VehicleDesc d, ThrottleInput kind, std::vector<real> x, std::vector<real> y) {
    ThrottleControlDesc t;
    t.engine = "engine";
    t.kind = kind;
    t.has_curve = !x.empty();
    t.curve.x = std::move(x);
    t.curve.y = std::move(y);
    d.powertrain.components.push_back(ComponentDesc{"throttle", t});
    return d;
}

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

// test_fuel_tank.cpp's make_rig: the vehicle_powertrain.json ground/chassis
// bodies; manual clutch locked, no assists, throttle 0 - or, with
// auto_clutch_launch, the auto-clutch launch law from an open clutch.
Rig make_rig(const vehicle::VehicleDesc& desc, int gear, int workers = 1, bool auto_clutch_launch = false) {
    Rig rig;
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
    rig.chassis = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
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

real sum_abs_terms(const PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
}

real speed(const Rig& r) {
    const Motion m = r.world->get_motion(r.chassis);
    return std::sqrt(m.linear.x * m.linear.x + m.linear.y * m.linear.y);
}

Powertrain build_pt(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

const SimulatedEngine& sim_of(const Powertrain& pt) {
    const auto* e = dynamic_cast<const SimulatedEngine*>(&pt.engine());
    REQUIRE(e != nullptr);
    return *e;
}

// Standalone engine held at omega for n substeps under `amb`.
struct Held {
    PreSolveResult pre;
    PostSolveResult post;
};
Held hold(SimulatedEngine& e, const AmbientSample& amb, real omega, real c, int n, real ignition = 1.0) {
    Held r;
    Sensors s;
    s.omega = omega;
    s.ignition = ignition;
    s.ambient = amb;
    for (int i = 0; i < n; ++i) {
        r.pre = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        r.post = e.post_solve(omega, kH);
    }
    return r;
}

} // namespace

// ---------------------------------------------------------------------------
// Idle: both fixture engines, neutral, clutch locked, 5 s from reset. The idle
// PI must hold idle_rpm on the whole engine-side inertia (crank + clutch +
// gearbox input) and the five C12 sound fields must carry the MVEM state (all
// 0 from the graph engine).
// ---------------------------------------------------------------------------
TEST_CASE("C12 idle: a World idle settles at idle_rpm and publishes the MVEM sound fields", "[unit][drivetrain][engine][fuel]") {
    for (const bool mech : {false, true}) {
        const SimulatedEngineDesc eng = mech ? fx::mechanical_engine() : fx::electronic_engine();
        Rig rig = make_rig(sim_sedan(eng), 0);
        World& w = *rig.world;
        for (int t = 0; t < 1200; ++t) w.step();
        const PowertrainSnapshot& s = w.powertrain_state(rig.vehicle);
        REQUIRE(s.engines.size() == 1);
        const EngineSoundState& es = s.engines[0];
        std::printf("[C12 idle] %s: %.2f rpm, plate %.4f, p_im %.1f Pa, lambda %.4f, spark %.2f deg, exhaust %.5f kg/s, fuel %.4f g/s\n",
                    mech ? "mechanical" : "electronic", s.engine_rpm, es.throttle_position, es.p_intake_pa, es.lambda, es.spark_advance_deg,
                    es.exhaust_mass_flow_kg_s, s.fuel_flow_g_s);
        CHECK(s.engine_state == EngineState::Running);
        CHECK(std::abs(s.engine_rpm - eng.idle_rpm) < 50.0);
        CHECK(s.fuel_flow_g_s > 0.0);
        CHECK(es.p_intake_pa > 10000.0);
        CHECK(es.p_intake_pa < kIsa.pressure_pa);
        CHECK(es.lambda > 0.8);
        CHECK(es.lambda < 1.2);
        CHECK(es.spark_advance_deg > 0.0);
        CHECK(es.exhaust_mass_flow_kg_s > s.fuel_flow_g_s * 1e-3); // air + fuel
        if (mech) {
            CHECK(es.throttle_position == 0.0); // the plate follows the pedal; idle air goes through the bypass valve
        } else {
            CHECK(es.throttle_position > 0.0);
            CHECK(es.throttle_position < 0.2);
        }
        // The fuel telemetry is registered for a simulated engine too.
        const auto& names = w.telemetry().channel_names();
        CHECK(std::find(names.begin(), names.end(), "vehicle0.fuel.flow_g_s") != names.end());
    }
}

// ---------------------------------------------------------------------------
// T8/F9: an auto-clutch WOT launch in 1st (3 s) and a lift (5 s: DFCO, then
// idle), with a tank. Every tick: the energy balance, the tank against the
// fuel ledger. At the end: the fuel identities, the pumping/friction split of
// engine_out (both negative, i.e. dissipative) and forward progress.
// ---------------------------------------------------------------------------
TEST_CASE("C12 T8/F9: a simulated-engine car launches, lifts and closes every ledger per tick", "[unit][drivetrain][engine][fuel]") {
    const bool mech = GENERATE(false, true);
    const SimulatedEngineDesc eng = mech ? fx::mechanical_engine() : fx::electronic_engine();
    const real initial = 30.0;
    Rig rig = make_rig(sim_sedan(eng, 0, initial), 1, 1, /*auto_clutch_launch=*/true);
    World& w = *rig.world;
    real worst_balance = 0.0, worst_tank = 0.0;
    bool stalled = false, saw_dfco = false;
    int tick = 0;
    auto check_tick = [&]() {
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        // Tick 0 is skipped: the Powertrain constructor seeds a Running
        // engine's crank at idle without booking it, so the very first
        // tick's rotor_ke delta carries 0.5*J*w_idle^2 (526.4 J here) -
        // pre-existing and shared with the graph engine (same Phase K seed),
        // not a C12 effect; see out/i7b_c12_progress.md.
        if (tick > 0 && denom > 1e-9) worst_balance = std::max(worst_balance, std::abs(e.balance_error) / denom);
        ++tick;
        const real m = w.fuel_tank_mass(rig.vehicle);
        worst_tank = std::max(worst_tank, std::abs((initial - m) - (e.fuel_mass - e.fuel_external_added)));
        const PowertrainSnapshot& s = w.powertrain_state(rig.vehicle);
        if (s.engine_state != EngineState::Running) stalled = true;
        if (!s.engines.empty() && s.engines[0].dfco_active) saw_dfco = true;
    };
    for (int t = 0; t < 120; ++t) { w.step(); check_tick(); }
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 720; ++t) { w.step(); check_tick(); }
    const real v_launch = speed(rig);
    const real rpm_launch = w.powertrain_state(rig.vehicle).engine_rpm;
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 1200; ++t) { w.step(); check_tick(); }

    const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
    std::printf("[C12 T8/F9] %s: v %.3f m/s at %.0f rpm after 3 s WOT; worst balance %.3e rel, worst tank %.3e kg; fuel %.6f kg, "
                "indicated %.1f J, heat %.1f J, pumping %.1f J, friction %.1f J, engine_out %.1f J\n",
                mech ? "mechanical" : "electronic", v_launch, rpm_launch, worst_balance, worst_tank, e.fuel_mass, e.indicated_work,
                e.engine_heat, e.pumping_work, e.friction_work, e.engine_out);
    CHECK_FALSE(stalled);
    CHECK(v_launch > 7.0);
    CHECK(saw_dfco);
    CHECK(worst_balance <= 5e-8);
    CHECK(worst_tank <= 1e-12 * initial);
    CHECK(e.fuel_mass > 0.0);
    CHECK(std::abs(e.fuel_energy_in - e.fuel_mass * eng.fuels[0].lhv_j_kg) <= 1e-9 * e.fuel_energy_in);
    CHECK(e.indicated_work > 0.0);
    CHECK(e.engine_heat > 0.0);
    CHECK(std::abs(e.engine_heat - (e.fuel_energy_in - e.indicated_work)) <= 1e-9 * e.fuel_energy_in);
    CHECK(e.fuel_efficiency_violations == 0);
    CHECK(e.pumping_work < 0.0);
    CHECK(e.friction_work < 0.0);
    CHECK(e.indicated_work + e.pumping_work + e.friction_work > 0.0);
}

// Diagnostic for the tick-0 skip above: the plain graph-engine car_sedan
// shows the same unbooked first-tick rotor_ke jump (pre-existing).
TEST_CASE("C12 diag: tick-0 balance_error of the graph-engine car_sedan (pre-existing seed)", "[.][diag][drivetrain]") {
    Rig rig = make_rig(ps_test::load_sedan_desc(), 1, 1, /*auto_clutch_launch=*/true);
    rig.world->step();
    const PowertrainEnergy& e = rig.world->powertrain_energy(rig.vehicle);
    std::printf("[C12 diag] graph car_sedan tick 0: balance_error %.6e J, rotor_ke %.6e J, sum_abs_terms %.6e\n", e.balance_error, e.rotor_ke,
                sum_abs_terms(e));
    rig.world->step();
    const PowertrainEnergy& e1 = rig.world->powertrain_energy(rig.vehicle);
    std::printf("[C12 diag] graph car_sedan tick 1: balance_error %.6e J\n", e1.balance_error);
}

// Review R7: the Powertrain passes the burnt fuel to the engine. A tank
// holding fuels[1] must book fuels[1]'s LHV (the energy identity is exact per
// substep, so the cumulative ratio is too) and differ from a fuels[0] tank.
TEST_CASE("C12 F9: a tank holding the engine's second fuel burns that fuel, not fuels[0]", "[unit][drivetrain][engine][fuel]") {
    SimulatedEngineDesc eng = fx::electronic_engine();
    eng.fuels = {fx::gasoline(), blend()};
    auto run = [&](int tank_fuel) {
        Rig rig = make_rig(sim_sedan(eng, tank_fuel), 0);
        rig.world->set_control("throttle", 0.3);
        for (int t = 0; t < 480; ++t) rig.world->step();
        return rig.world->powertrain_energy(rig.vehicle);
    };
    const PowertrainEnergy e0 = run(0);
    const PowertrainEnergy e1 = run(1);
    const real lhv0 = e0.fuel_energy_in / e0.fuel_mass;
    const real lhv1 = e1.fuel_energy_in / e1.fuel_mass;
    std::printf("[C12 F9] booked LHV: tank fuels[0] %.6e, tank fuels[1] %.6e J/kg; fuel mass %.6e vs %.6e kg\n", lhv0, lhv1, e0.fuel_mass,
                e1.fuel_mass);
    CHECK(std::abs(lhv0 / fx::gasoline().lhv_j_kg - 1.0) <= 1e-12);
    CHECK(std::abs(lhv1 / blend().lhv_j_kg - 1.0) <= 1e-12);
    CHECK(e1.fuel_mass > e0.fuel_mass); // lower A/F and LHV: more fuel for the same air
}

// ---------------------------------------------------------------------------
// T3: ambient. Standalone engine at 3000 rpm, WOT, 3 s hold (steady to
// rounding), ISA sea level vs ISA 2000 m fed through Sensors::ambient - the
// C12 path, never a direct set_ambient() call.
//   (a) air per crank revolution == VE(rpm, p_im) * p_im / (R * T_amb) *
//       V_d / 2 with the test's own V_d and R, the fixture's own VE
//       polynomial (exact: 3000 rpm is a table knot and VE is linear in p),
//       at BOTH ambients - T_amb and p_amb must reach the manifold model;
//   (b) indicated torque follows the density ratio (report + loose bound: VE
//       rises with p_im and eta_i with the air charge, both lower at
//       altitude);
//   (c) the electronic pedal law (C11 T4a) still holds at altitude - the
//       derived steady tables are refilled for the new ambient;
//   (d) white-box Powertrain: the refill happens on an ambient CHANGE only.
// ---------------------------------------------------------------------------
TEST_CASE("C12 T3: the ambient sample drives the manifold, the pedal law and the steady-table refill", "[unit][drivetrain][engine]") {
    const SimulatedEngineDesc desc = fx::electronic_engine();
    const auto maps = SimulatedEngine::build_maps(desc);
    const real rpm = 3000.0, w = w_of(rpm);
    const real vd = 4.0 * kPi / 4.0 * 0.086 * 0.086 * 0.086;

    real tau_i[2] = {0.0, 0.0};
    int k = 0;
    for (const AmbientSample& amb : {kIsa, kAlt}) {
        SimulatedEngine e(desc, maps);
        const Held h = hold(e, amb, w, 1.0, 2880);
        const real m_air_rev = (h.pre.exhaust_mass_flow_kg_s - h.pre.fuel_flow_g_s * 1e-3) * 2.0 * kPi / w;
        const real p = e.p_im();
        const real ref = fx::fx_ve(rpm, p) * p / (287.05 * amb.temperature_k) * vd * 0.5;
        tau_i[k] = h.post.indicated_work_delta / (kH * w);
        std::printf("[C12 T3a] p_amb %.1f T %.2f: p_im %.2f Pa, air/rev %.9e vs %.9e kg (rel %.2e), tau_i %.3f N*m, p_intake field %.2f\n",
                    amb.pressure_pa, amb.temperature_k, p, m_air_rev, ref, std::abs(m_air_rev / ref - 1.0), tau_i[k], h.pre.p_intake_pa);
        CHECK(std::abs(m_air_rev / ref - 1.0) <= 1e-9);
        CHECK(p < amb.pressure_pa);
        CHECK(p > 0.9 * amb.pressure_pa);
        CHECK(h.pre.p_intake_pa == p);
        ++k;
    }
    const real density_ratio = (kAlt.pressure_pa / kAlt.temperature_k) / (kIsa.pressure_pa / kIsa.temperature_k);
    const real torque_ratio = tau_i[1] / tau_i[0];
    std::printf("[C12 T3b] indicated torque ratio %.5f vs density ratio %.5f (%.3f %%)\n", torque_ratio, density_ratio,
                100.0 * (torque_ratio / density_ratio - 1.0));
    CHECK(std::abs(torque_ratio / density_ratio - 1.0) < 0.05);

    SECTION("(c) the electronic pedal law holds at altitude") {
        const real ck = maps->c_k;
        auto steady_net = [&](real r, real ignition) {
            SimulatedEngine e(desc, maps);
            return hold(e, kAlt, w, r, 2880, ignition).pre.tau0 - ck;
        };
        const real wot = steady_net(1.0, 1.0);
        const real mot = steady_net(0.0, 0.0);
        real worst = 0.0;
        for (real r : {0.2, 0.45, 0.7}) {
            const real t = steady_net(r, 1.0);
            const real err = std::abs(t - (mot + r * (wot - mot))) / (wot - mot);
            std::printf("[C12 T3c] altitude r %.2f: %.4f vs %.4f N*m (%.4f %% of span)\n", r, t, mot + r * (wot - mot), 100.0 * err);
            worst = std::max(worst, err);
        }
        CHECK(worst < 0.01);
    }

    SECTION("(d) the Powertrain's engine refills its tables on a change only, and the refill is cheap") {
        Powertrain pt = build_pt(sim_sedan(desc));
        vehicle::VehicleResetOptions o;
        o.gear = 0;
        o.engine_state = EngineState::Running;
        o.clutch_locked = true;
        pt.reset(o, std::vector<real>(4, 0.0));
        ComponentSubstepCtx ctx;
        ctx.h = kH;
        ctx.ignition = 1.0;
        ctx.gear_request = kGearRequestNone;
        auto run = [&](const AmbientSample& a, int n) {
            ctx.ambient = a;
            for (int i = 0; i < n; ++i) {
                ctx.first_substep_of_tick = (i % 4) == 0;
                pt.begin_substep();
                pt.pre_solve_components(ctx);
                for (std::size_t wi = 0; wi < 4; ++wi) pt.add_hub_terms(wi, -5000.0 * pt.hub_omega(wi), 5000.0);
                pt.solve(ctx.h);
                pt.post_solve_components(ctx);
            }
        };
        const SimulatedEngine& e = sim_of(pt);
        run(kIsa, 96);
        CHECK(e.steady_rebuild_count() == 0);
        run(kAlt, 96);
        CHECK(e.steady_rebuild_count() == 1);
        CHECK(e.steady_tables().p_pa == kAlt.pressure_pa);
        CHECK(e.steady_tables().t_k == kAlt.temperature_k);
        run(kAlt, 96);
        CHECK(e.steady_rebuild_count() == 1);
        run(kIsa, 1);
        CHECK(e.steady_rebuild_count() == 2);
        // The ISA refill reproduces the build-time (ISA) tables bit for bit.
        CHECK(e.steady_tables().inverse_plate.values == e.maps().inverse_plate.values);
        CHECK(e.steady_tables().steady_wot_nm.y == e.maps().steady_wot_nm.y);
        CHECK(e.steady_tables().idle_ff == e.maps().idle_ff);

        // Cost of one refill (report only).
        SimulatedEngine se(desc, maps);
        constexpr int kN = 20;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kN; ++i) se.set_ambient(i % 2 ? kIsa.pressure_pa : kAlt.pressure_pa, i % 2 ? kIsa.temperature_k : kAlt.temperature_k);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / kN;
        std::printf("[C12 T3d] one steady-table refill: %.3f ms (mean of %d, %llu rebuilds)\n", ms, kN,
                    static_cast<unsigned long long>(se.steady_rebuild_count()));
        CHECK(se.steady_rebuild_count() == static_cast<std::uint64_t>(kN));
    }
}

// ---------------------------------------------------------------------------
// T4c: the simulated engine reads the throttle stage's command, never the raw
// pedal. Two pedals on a flat plateau of an explicit stage's curve (electronic
// pedal_map / mechanical linkage_curve) must give bit-identical Worlds (state
// hash every tick, 2 s, neutral free rev on the plateau).
// ---------------------------------------------------------------------------
TEST_CASE("C12 T4c: two pedals on a flat throttle-stage plateau are indistinguishable", "[unit][drivetrain][engine][throttle]") {
    const bool mech = GENERATE(false, true);
    const SimulatedEngineDesc eng = mech ? fx::mechanical_engine() : fx::electronic_engine();
    const vehicle::VehicleDesc d = with_throttle_stage(sim_sedan(eng), eng.throttle_input, {0.0, 0.4, 0.6, 1.0}, {0.0, 0.3, 0.3, 1.0});
    Rig ra = make_rig(d, 0);
    Rig rb = make_rig(d, 0);
    ra.world->set_control("throttle", 0.45);
    rb.world->set_control("throttle", 0.55);
    int mismatches = 0, first = -1;
    real max_rpm = 0.0;
    for (int t = 0; t < 480; ++t) {
        ra.world->step();
        rb.world->step();
        if (ra.world->state_hash() != rb.world->state_hash() && mismatches++ == 0) first = t;
        max_rpm = std::max(max_rpm, ra.world->powertrain_state(ra.vehicle).engine_rpm);
    }
    const PowertrainSnapshot& s = ra.world->powertrain_state(ra.vehicle);
    INFO("first differing tick " << first << " (" << mismatches << " of 480)");
    std::printf("[C12 T4c] %s: %d mismatching ticks, max rpm %.1f, command %.3f\n", mech ? "mechanical" : "electronic", mismatches, max_rpm,
                s.engines.empty() ? -1.0 : s.engines[0].throttle_command);
    CHECK(mismatches == 0);
    REQUIRE(s.engines.size() == 1);
    CHECK(s.engines[0].throttle_command == 0.3); // non-vacuous: both pedals went through the plateau
    CHECK(max_rpm > 3000.0);                     // ... and the engine revved on it
}

// Pairing backstop: a code-built desc whose throttle_control kind differs from
// the simulated engine's throttle_input is refused (the loader rejects the
// JSON form already).
TEST_CASE("C12 pairing: a throttle_control stage of the other kind is refused for a simulated engine", "[unit][drivetrain][engine][throttle]") {
    const vehicle::VehicleDesc e_on_m =
        with_throttle_stage(sim_sedan(fx::mechanical_engine()), ThrottleInput::Electronic, {}, {});
    CHECK_THROWS_WITH(build_pt(e_on_m), ContainsSubstring("kind does not match simulated engine 'engine''s throttle_input"));
    const vehicle::VehicleDesc m_on_e =
        with_throttle_stage(sim_sedan(fx::electronic_engine()), ThrottleInput::Mechanical, {}, {});
    CHECK_THROWS_WITH(build_pt(m_on_e), ContainsSubstring("kind does not match simulated engine 'engine''s throttle_input"));
    const vehicle::VehicleDesc ok = with_throttle_stage(sim_sedan(fx::electronic_engine()), ThrottleInput::Electronic, {}, {});
    CHECK_NOTHROW(build_pt(ok));
}

// ---------------------------------------------------------------------------
// N1 (vault PHYS-049/050 follow-ups): the production loader
// (core/src/io/vehicle_io.cpp) already rejects an empty "fuels" list for a
// simulated engine, so this guard is only reachable via a CODE-BUILT
// SimulatedEngineDesc that skips that check (as this test does directly,
// never going through the loader) - previously sim->fuels.front() on an
// empty vector was UB instead of a clear throw.
// ---------------------------------------------------------------------------
TEST_CASE("C12 N1: a code-built simulated engine with no fuels and no tank throws instead of UB",
          "[unit][drivetrain][engine][fuel]") {
    SimulatedEngineDesc eng = fx::electronic_engine();
    eng.fuels.clear();
    const vehicle::VehicleDesc d = sim_sedan(eng); // tank_fuel = -1: no tank
    CHECK_THROWS_WITH(build_pt(d), ContainsSubstring("empty fuels list and no fuel_tank"));
}

// ---------------------------------------------------------------------------
// Determinism and allocation: 1 vs 10 workers, auto-clutch WOT launch with an
// ambient change to 2000 m in the middle (the refill runs inside the vehicle
// job) - identical state_hash, identical rpm, zero own allocations while
// armed. The ambient is not hashed, so a constant-ambient twin must differ
// only through the dynamics (checked: its rpm differs).
// ---------------------------------------------------------------------------
namespace {
struct DetResult {
    std::uint64_t hash = 0;
    real rpm = 0.0;
};
DetResult run_det(int workers, bool change_ambient, bool arm_alloc) {
    Rig rig = make_rig(sim_sedan(fx::electronic_engine(), 0), 1, workers, /*auto_clutch_launch=*/true);
    World& w = *rig.world;
    for (int t = 0; t < 240; ++t) w.step();
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 240; ++t) w.step();
    if (change_ambient) w.set_ambient(kAlt);
    for (int t = 0; t < 240; ++t) w.step();
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 240; ++t) w.step();
    if (arm) {
        da::set_armed(false);
        const auto own = da::own_violation_count();
        const auto ext = da::external_violation_count();
        WARN("C12 determinism debug_alloc (" << workers << " workers): own " << own << ", external (Jolt) " << ext);
        CHECK(own == 0);
    }
    return DetResult{w.state_hash(), w.powertrain_state(rig.vehicle).engine_rpm};
}
} // namespace

TEST_CASE("C12 determinism: 1 vs 10 workers with a mid-run ambient change hash identically, allocation-free",
          "[unit][drivetrain][engine][jobs][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const DetResult a1 = run_det(1, true, true);
    const DetResult a10 = run_det(10, true, true);
    const DetResult isa = run_det(1, false, false);
    std::printf("[C12 det] hash 1w %016llx 10w %016llx, rpm %.6f / %.6f; constant-ISA twin rpm %.6f\n", static_cast<unsigned long long>(a1.hash),
                static_cast<unsigned long long>(a10.hash), a1.rpm, a10.rpm, isa.rpm);
    CHECK(a1.hash == a10.hash);
    CHECK(a1.rpm == a10.rpm);
    CHECK(a1.rpm != isa.rpm); // the ambient change really reached the dynamics
}

// ---------------------------------------------------------------------------
// Integration (C12 + C17a review): the SHIPPED data/vehicles/car_sedan_sim.json
// (A25A-FKS mean-value engine, tools/enginedata/a25a_mb_engine.py) loaded
// through the production loader - not a code-built fixture. Auto-clutch rig in
// neutral, 2 s idle settle, then 5 s WOT: gear_request 1 at t = 0, gear_request
// 2 at t = 2.5 s. Asserts: the loaded engine IS the simulated one (manifold
// pressure published - the graph engine publishes 0), idle within 50 rpm of
// the file's idle_rpm, never leaves Running, reaches 2nd, a speed bound
// measured with margin (see the printf; the graph-engine car_sedan.json
// reaches less - the bound discriminates the two engines), and the per-tick
// balance_error <= 5e-8 relative from tick 1 (tick 0 carries the pre-existing
// unbooked Running-engine seed, F-T0 in out/i7b_c12_progress.md).
// Sabotages (review run): loading car_sedan.json instead fails the p_im
// check (0 Pa) and the speed bound (17.802 m/s); booking the engine's work
// at omega_old in SimulatedEngine::post_solve fails the balance check
// (2.566e-4 relative vs. 2.123e-14 clean).
// ---------------------------------------------------------------------------
TEST_CASE("C12/C17a integration: car_sedan_sim.json idles, launches WOT in 1st and 2nd without stalling, ledger closed per tick",
          "[unit][drivetrain][vehicle][engine]") {
    const std::string path = std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_sim.json";
    const vehicle::VehicleDesc desc = io::load_vehicle_json(path);
    real idle_rpm_desc = -1.0;
    for (const auto& c : desc.powertrain.components)
        if (const auto* s = std::get_if<SimulatedEngineDesc>(&c.params)) idle_rpm_desc = s->idle_rpm;
    REQUIRE(idle_rpm_desc > 0.0); // the file's engine is a simulated one

    Rig rig = make_rig(desc, 0, 1, /*auto_clutch_launch=*/true);
    World& w = *rig.world;
    real worst_balance = 0.0;
    int tick = 0;
    bool stalled = false;
    int max_gear = 0;
    real max_rpm = 0.0;
    auto check_tick = [&]() {
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (tick > 0 && denom > 1e-9) worst_balance = std::max(worst_balance, std::abs(e.balance_error) / denom);
        ++tick;
        const PowertrainSnapshot& s = w.powertrain_state(rig.vehicle);
        if (s.engine_state != EngineState::Running) stalled = true;
        max_gear = std::max(max_gear, static_cast<int>(s.gear));
        max_rpm = std::max(max_rpm, s.engine_rpm);
    };
    for (int t = 0; t < 480; ++t) { w.step(); check_tick(); }
    const PowertrainSnapshot idle = w.powertrain_state(rig.vehicle);
    REQUIRE(idle.engines.size() == 1);
    const real idle_rpm = idle.engine_rpm;
    const real idle_p_im = idle.engines[0].p_intake_pa;

    w.set_control("throttle", 1.0);
    w.set_control("gear_request", 1.0);
    for (int t = 0; t < 600; ++t) { w.step(); check_tick(); }
    const real v_1st = speed(rig);
    const real rpm_1st = w.powertrain_state(rig.vehicle).engine_rpm;
    w.set_control("gear_request", 2.0);
    for (int t = 0; t < 600; ++t) { w.step(); check_tick(); }
    const PowertrainSnapshot end = w.powertrain_state(rig.vehicle);
    const real v_end = speed(rig);

    std::printf("[C12/C17a car_sedan_sim] idle %.2f rpm (desc %.0f), p_im %.0f Pa; WOT 1st 2.5 s: %.3f m/s at %.0f rpm; "
                "5 s: %.3f m/s at %.0f rpm, gear %d, max rpm %.0f; worst balance %.3e rel\n",
                idle_rpm, idle_rpm_desc, idle_p_im, v_1st, rpm_1st, v_end, end.engine_rpm, static_cast<int>(end.gear), max_rpm, worst_balance);
    CHECK(idle_p_im > 0.0);
    CHECK(std::abs(idle_rpm - idle_rpm_desc) < 50.0);
    CHECK_FALSE(stalled);
    CHECK(max_gear == 2);
    CHECK(static_cast<int>(end.gear) == 2);
    // Measured 19.370 m/s (graph-engine car_sedan.json, same protocol: 17.802).
    CHECK(v_end > 18.5);
    CHECK(worst_balance <= 5e-8);
}
