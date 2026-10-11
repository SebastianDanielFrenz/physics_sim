// tests/unit/test_limit_remedies.cpp - engine garage P3b-2: nitrous in the dyno sweep (DynoSweepSpec::nitrous), the N2O limit
// findings with their Closed remedies, and the Simulated remedies (solve_remedy, solve_power_target; limit_remedies.h).
//
// Fixtures are the shipped hypercar files, loaded with Sketch maps (cached on disk; the cases that need the real N7 numbers load
// Full maps and are [long] only on a cold cache). The hardware under test is crafted in the test (a loaded descriptor mutated,
// no data file). Every case lists the sabotage it was shown to fail on; the sabotages were hooks in the code under test, run once
// and removed.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "drivetrain/engine_simulated.h"
#include "drivetrain/limit_thresholds.h"
#include "drivetrain/nitrous_flow.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_remedies.h"
#include "ps/io/nitrous_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {
const std::string kVehicles = std::string(PS_SOURCE_DIR) + "/data/vehicles/";
const std::string kEngines = std::string(PS_SOURCE_DIR) + "/data/engines/";
constexpr double kPi = 3.14159265358979323846;

jobs::JobSystem& pool10() {
    static jobs::JobSystem p(10);
    return p;
}
io::EngineMapOptions map_options(MapQuality q) {
    io::EngineMapOptions o;
    o.quality = q;
    o.pool = &pool10();
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "ps_garage_p3b2_maps";
    std::filesystem::create_directories(p);
    o.cache_dir = p.string();
    return o;
}

struct Car {
    vehicle::VehicleDesc desc;
    ComponentParams engine;
    std::optional<DynoNitrous> nitrous;
};
// A shipped vehicle file's simulated engine and (if any) its nitrous parts, loaded once per (file, quality).
const Car& car(const std::string& file, MapQuality q) {
    static std::map<std::pair<std::string, int>, Car> cache;
    const auto key = std::make_pair(file, static_cast<int>(q));
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;
    Car c;
    c.desc = io::load_vehicle_json(kVehicles + file, map_options(q));
    for (const ComponentDesc& cd : c.desc.powertrain.components)
        if (std::holds_alternative<SimulatedEngineDesc>(cd.params)) c.engine = cd.params;
    c.nitrous = nitrous_from_powertrain(c.desc.powertrain);
    return cache.emplace(key, std::move(c)).first->second;
}
const Car& n2o_car() { return car("car_hyper_n2o.json", MapQuality::Sketch); }   // the N7b hardware (larger wastegate) + the 800 hp kit
[[maybe_unused]] const Car& stock_car() { return car("car_hyper.json", MapQuality::Sketch); }     // the stock hardware

[[maybe_unused]] SimulatedEngineDesc& sim_of(ComponentParams& c) { return std::get<SimulatedEngineDesc>(c); }

DynoSweepSpec spec_at(std::vector<double> rpm, const std::optional<DynoNitrous>& nitrous) {
    DynoSweepSpec s;
    s.rpm = std::move(rpm);
    s.turbo_count = 2; // the hyper's twin turbo configuration
    s.nitrous = nitrous;
    s.part_files.engine = "engine.json";
    s.part_files.turbo = "turbo.json";
    s.part_files.nitrous_kit = "kit.json";
    s.part_files.n2o_bottle = "bottle.json";
    return s;
}
const LimitFinding* find(const DynoResult& r, LimitKind k, double rpm = -1.0) {
    for (const LimitFinding& f : r.limits)
        if (f.kind == k && (rpm < 0.0 || f.rpm == rpm)) return &f;
    return nullptr;
}
bool any_kind(const DynoResult& r, LimitKind k) { return find(r, k) != nullptr; }

// ---------------------------------------------------------------------------------------------------------------------
// The reference: tests/validation/test_car_hyper_n2o.cpp's dyno-held harness (dyno_shot), copied - independent of NitrousSystem
// (the kit's flows from the descriptor: HEM/Dyer flux x Cd x A, the fuel jet incompressible).
struct RefShot {
    double torque_nm = 0.0, kw = 0.0, mdot_n = 0.0, mdot_kf = 0.0, lambda_c = 0.0;
};
RefShot reference_shot(const SimulatedEngineDesc& d, const DynoNitrous* nit, double rpm, double settle_s = 10.0, double avg_s = 0.5) {
    constexpr double h = 1.0 / 960.0;
    const auto maps = SimulatedEngine::build_maps(d);
    SimulatedEngine e(d, maps);
    const double omega = rpm * kPi / 30.0;
    const int n_settle = static_cast<int>(settle_s / h + 0.5), n_avg = static_cast<int>(avg_s / h + 0.5);
    std::unique_ptr<OxidizerTables> tabs;
    double bottle_abs = 0.0, hin = 0.0, g = 0.0, n_area = 0.0, f_area = 0.0;
    if (nit) {
        tabs = std::make_unique<OxidizerTables>(nit->bottle.oxidizer);
        bottle_abs = (nit->bottle.pressure.gauge_bar + 1.01325) * 1e5;
        hin = tabs->liquid_at_pressure(bottle_abs).h_l_j_kg;
        g = tabs->g_hem(bottle_abs);
        n_area = nit->kit.nozzle_count * kPi / 4.0 * nit->kit.nitrous_jet_diameter_mm * nit->kit.nitrous_jet_diameter_mm * 1e-6;
        f_area = nit->kit.nozzle_count * kPi / 4.0 * nit->kit.fuel_jet_diameter_mm * nit->kit.fuel_jet_diameter_mm * 1e-6;
    }
    const double rho_f = d.fuels[0].density_kg_m3;
    RefShot r;
    double p_im = 0.9e5;
    for (int i = 0; i < n_settle + n_avg; ++i) {
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.clutch_engaged_fraction = 1.0;
        s.gearbox_in_neutral = false;
        double mdn = 0.0, mdk = 0.0;
        if (nit) {
            mdn = nit->kit.nitrous_jet_cd * n_area * g;
            const double p_fuel = (nit->kit.fuel_reference == NitrousFuelReference::Manifold ? p_im : 101325.0) + nit->kit.fuel_pressure_gauge_bar * 1e5;
            mdk = nit->kit.fuel_jet_cd * f_area * std::sqrt(2.0 * rho_f * std::max(0.0, p_fuel - p_im));
            s.nitrous.mdot_n_kg_s = mdn;
            s.nitrous.mdot_kf_kg_s = mdk;
            s.nitrous.h_in_j_kg = hin;
            s.nitrous.retard_deg = nit->kit.controller.retard_is_curve ? nit->kit.controller.retard_vs_fraction.evaluate(1.0) : nit->kit.controller.spark_retard_deg;
            s.nitrous.active = true;
            s.nitrous.oxidizer = tabs.get();
        }
        ThrottleCommand tc;
        tc.command = 1.0;
        const PreSolveResult pre = e.pre_solve(s, tc, TcuRequest{}, h);
        (void)e.post_solve(omega, h);
        p_im = pre.p_intake_pa;
        if (i >= n_settle) {
            r.torque_nm += pre.tau0 - pre.friction_cap_kinetic;
            r.mdot_n += mdn;
            r.mdot_kf += mdk;
            r.lambda_c += pre.n2o_lambda_combined;
        }
    }
    r.torque_nm /= n_avg;
    r.mdot_n /= n_avg;
    r.mdot_kf /= n_avg;
    r.lambda_c /= n_avg;
    r.kw = r.torque_nm * omega * 1e-3;
    return r;
}
} // namespace

// =====================================================================================================================
// Nitrous in the sweep.

TEST_CASE("R1 nitrous sweep equals the dyno-held reference harness (hyper + 800 hp kit, 3000/5000/7000 rpm)", "[drivetrain][dyno][n2o][turbo]") {
    // The sweep reuses NitrousSystem (the car's flow arithmetic); the reference is test_car_hyper_n2o.cpp's dyno_shot, written
    // from the descriptor. Same maps, same engine. They differ in the settle (the sweep stops at the convergence criterion,
    // 1.5 .. 15 s; the reference runs 10 s then averages 0.5 s) and in the bottle pressure expression, so the N2O flow agrees to
    // rounding and the torque to the settle criterion's 0.2 %/s.
    // Recorded sabotage: stepped_point not handing nsys->engine_input() to the engine (Sensors::nitrous left empty) -> the
    // torque is the no-kit one and the flow 0: every check below fails.
    const Car& c = n2o_car();
    REQUIRE(c.nitrous.has_value());
    const DynoResult r = run_dyno_sweep(c.engine, nullptr, spec_at({3000.0, 5000.0, 7000.0}, c.nitrous), &pool10());
    REQUIRE(r.points.size() == 3);
    const SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(c.engine);
    for (std::size_t i = 0; i < 3; ++i) {
        const DynoPoint& p = r.points[i];
        const RefShot ref = reference_shot(d, &*c.nitrous, p.rpm);
        INFO("rpm " << p.rpm << " sweep " << p.torque_nm << " Nm vs reference " << ref.torque_nm << " Nm, lambda_c " << p.lambda_combined << " vs " << ref.lambda_c);
        CHECK(p.converged);
        CHECK(p.n2o_g_s == Catch::Approx(ref.mdot_n * 1000.0).epsilon(1e-9));
        CHECK(p.kit_fuel_g_s == Catch::Approx(ref.mdot_kf * 1000.0).epsilon(1e-2)); // the engine books the kit fuel through its starve/cut share
        CHECK(p.torque_nm == Catch::Approx(ref.torque_nm).epsilon(0.005));
        CHECK(p.power_kw == Catch::Approx(ref.kw).epsilon(0.005));
        CHECK(p.lambda_combined == Catch::Approx(ref.lambda_c).epsilon(0.005));
    }
}

TEST_CASE("R2 a sweep without a kit, or with a kit the controller does not spray, is the plain sweep", "[drivetrain][dyno][n2o][turbo]") {
    // Bit-identity of the no-kit path is also pinned by test_engine_dyno (D*) and L18, which did not change. Here: the kit's rpm
    // window excludes every swept point -> the engine input is the empty block each substep -> every point equals the no-kit sweep
    // bit for bit (both stepped), with the nitrous columns at their defaults. Recorded sabotage: the dyno ignoring the
    // controller's min_rpm (always full command) -> torque differs.
    const Car& c = n2o_car();
    DynoNitrous nit = *c.nitrous;
    nit.kit.controller.min_rpm = 8000.0;
    const DynoResult plain = run_dyno_sweep(c.engine, nullptr, spec_at({3000.0, 5000.0}, std::nullopt), &pool10());
    const DynoResult gated = run_dyno_sweep(c.engine, nullptr, spec_at({3000.0, 5000.0}, nit), &pool10());
    REQUIRE(plain.points.size() == 2);
    for (std::size_t i = 0; i < 2; ++i) {
        CHECK(gated.points[i].torque_nm == plain.points[i].torque_nm);
        CHECK(gated.points[i].fuel_g_s == plain.points[i].fuel_g_s);
        CHECK(gated.points[i].boost_bar == plain.points[i].boost_bar);
        CHECK(gated.points[i].n2o_g_s == 0.0);
        CHECK(gated.points[i].kit_fuel_g_s == 0.0);
        CHECK(std::isnan(gated.points[i].lambda_combined));
        CHECK(plain.points[i].n2o_g_s == 0.0);
        CHECK(std::isnan(plain.points[i].lambda_combined));
    }
    CHECK_FALSE(any_kind(gated, LimitKind::N2OBottleDuration));
    // A part-throttle dyno never sprays either (the controller's min_throttle_command, 0.95 [shipped controller]).
    DynoSweepSpec part = spec_at({5000.0}, c.nitrous);
    part.throttle = 0.5;
    CHECK(run_dyno_sweep(c.engine, nullptr, part, &pool10()).points[0].n2o_g_s == 0.0);
}

TEST_CASE("R3 dyno rules: ramp skipped, duty-time cap off, naturally aspirated engines, torque-map engines refused", "[drivetrain][dyno][n2o]") {
    // Recorded sabotages: dyno_kit() returning the kit unchanged -> (a) the 100 s delay keeps the flow at start_fraction 0.1:
    // n2o_g_s 24.6 instead of 246.4; (b) max_active_s 0.5 cuts the spray after 0.5 s: n2o_g_s 0.
    const Car& c = n2o_car();
    DynoNitrous slow = *c.nitrous;
    slow.kit.controller.progressive = NitrousProgressiveDesc{0.1, 100.0, 0.0};
    const DynoResult ramp = run_dyno_sweep(c.engine, nullptr, spec_at({5000.0}, slow), &pool10());
    const DynoResult full = run_dyno_sweep(c.engine, nullptr, spec_at({5000.0}, c.nitrous), &pool10());
    CHECK(ramp.points[0].n2o_g_s == full.points[0].n2o_g_s);
    CHECK(ramp.points[0].n2o_g_s > 200.0);

    DynoNitrous capped = *c.nitrous;
    capped.kit.controller.safety.max_active_s = 0.5;
    CHECK(run_dyno_sweep(c.engine, nullptr, spec_at({5000.0}, capped), &pool10()).points[0].n2o_g_s == full.points[0].n2o_g_s);

    // A naturally aspirated simulated engine with a kit is stepped, not the instant steady point.
    const ComponentParams a25a = io::load_engine_json(kEngines + "toyota_a25a_fks_sim.json");
    const vehicle::VehicleDesc sedan = io::load_vehicle_json(kVehicles + "car_sedan_gen_n2o.json", map_options(MapQuality::Sketch));
    const auto sedan_kit = nitrous_from_powertrain(sedan.powertrain);
    REQUIRE(sedan_kit.has_value());
    DynoSweepSpec na = spec_at({4500.0}, std::nullopt);
    na.turbo_count = 1;
    const DynoResult na_plain = run_dyno_sweep(a25a, nullptr, na, nullptr);
    na.nitrous = sedan_kit;
    const DynoResult na_shot = run_dyno_sweep(a25a, nullptr, na, nullptr);
    CHECK(na_plain.points[0].settle_s == 0.0); // the instant kind
    CHECK(na_shot.points[0].settle_s > 0.0);   // stepped
    CHECK(na_shot.points[0].n2o_g_s > 10.0);
    CHECK(na_shot.points[0].power_kw > na_plain.points[0].power_kw + 20.0);
    CHECK(na_shot.points[0].boost_bar == 0.0);

    // A torque-map engine ignores a kit in the car: a sweep with one is an error, not a silent no-op.
    const ComponentParams tm = io::load_engine_json(kEngines + "i4_2l_na.json");
    DynoSweepSpec bad = spec_at({4000.0}, c.nitrous);
    CHECK_THROWS_AS(run_dyno_sweep(tm, nullptr, bad, nullptr), std::invalid_argument);
}
