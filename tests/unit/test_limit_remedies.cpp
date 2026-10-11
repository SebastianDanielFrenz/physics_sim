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
#include <limits>
#include <map>
#include <stdexcept>
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
    // rounding and the torque to the settle criterion's 0.2 %/s (bounds below from the measured differences).
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
        // Measured (3000..7000 rpm): flows equal to rounding (the kit fuel jet sees a constant pressure drop here); torque/power
        // -2.0e-4 .. -2.8e-4 below 7000 rpm and -1.2e-3 at 7000 (the sweep stops at the 0.2 %/s criterion, the reference
        // settles 10 s); combined lambda 3.5e-5 .. 5.0e-4. The bounds are 2x the worst.
        CHECK(p.n2o_g_s == Catch::Approx(ref.mdot_n * 1000.0).epsilon(1e-12));
        CHECK(p.kit_fuel_g_s == Catch::Approx(ref.mdot_kf * 1000.0).epsilon(1e-9));
        CHECK(p.torque_nm == Catch::Approx(ref.torque_nm).epsilon(2.5e-3));
        CHECK(p.power_kw == Catch::Approx(ref.kw).epsilon(2.5e-3));
        CHECK(p.lambda_combined == Catch::Approx(ref.lambda_c).epsilon(1e-3));
    }
}

TEST_CASE("R2 a sweep without a kit, or with a kit the controller does not spray, is the plain sweep", "[drivetrain][dyno][n2o][turbo]") {
    // Bit-identity of the no-kit path is also pinned by test_engine_dyno (D*) and L18, which did not change. Here: the kit's rpm
    // window excludes every swept point -> the engine input is the empty block each substep -> every point equals the no-kit sweep
    // bit for bit (both stepped), with the nitrous columns at their defaults. Recorded sabotage: the dyno handing the
    // controller a speed inside its window (843 rad/s = 8050 rpm instead of the held crank speed) -> the kit sprays,
    // torque 3124 vs 1220 Nm and 12 checks fail.
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

// =====================================================================================================================
// Crafted hardware (a loaded descriptor mutated in the test; no data file).
namespace {
// car_hyper_n2o's engine with the STOCK car_hyper wastegate (1.18e-3 m2 per turbo x 2): the owner's example (stock hardware
// + the 800 hp kit). Only the area differs between hyper_turbo_approx.json and hyper_turbo_n2o.json.
ComponentParams stock_hardware_engine() {
    ComponentParams e = n2o_car().engine;
    auto& d = std::get<SimulatedEngineDesc>(e);
    d.turbo_pair->wastegate_area_m2 = 2.0 * 1.18e-3;
    return e;
}
RemedyRequest request_for(const ComponentParams& e, const DynoNitrous& n, std::vector<double> rpm = {5000.0}) {
    RemedyRequest rq;
    rq.engine = e;
    rq.spec = spec_at(std::move(rpm), n);
    return rq;
}
} // namespace

TEST_CASE("P3b2 diag r1", "[.][diag]") {
    const Car& c = n2o_car();
    const DynoResult r = run_dyno_sweep(c.engine, nullptr, spec_at({3000.0, 4000.0, 5000.0, 6000.0, 7000.0}, c.nitrous), &pool10());
    const SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(c.engine);
    for (const DynoPoint& p : r.points) {
        const RefShot ref = reference_shot(d, &*c.nitrous, p.rpm);
        std::printf("rpm %.0f: torque %.6f vs %.6f (rel %.2e)  kw rel %.2e  lam_c %.6f vs %.6f (rel %.2e)  n2o rel %.2e  kitfuel %.5f vs %.5f (rel %.2e) settle %.2f\n", p.rpm, p.torque_nm, ref.torque_nm,
                    p.torque_nm / ref.torque_nm - 1.0, p.power_kw / ref.kw - 1.0, p.lambda_combined, ref.lambda_c, p.lambda_combined / ref.lambda_c - 1.0,
                    p.n2o_g_s / (ref.mdot_n * 1000.0) - 1.0, p.kit_fuel_g_s, ref.mdot_kf * 1000.0, p.kit_fuel_g_s / (ref.mdot_kf * 1000.0) - 1.0, p.settle_s);
    }
}


namespace {
DynoResult sweep(const ComponentParams& e, const std::optional<DynoNitrous>& n, std::vector<double> rpm) {
    return run_dyno_sweep(e, nullptr, spec_at(std::move(rpm), n), &pool10());
}
bool is_gate_kind(LimitKind k) { return k == LimitKind::WastegateSaturated || k == LimitKind::BoostAboveTarget || k == LimitKind::TurboRotorCap; }
bool any_gate_finding(const DynoResult& r) {
    for (const LimitFinding& f : r.limits)
        if (is_gate_kind(f.kind)) return true;
    return false;
}
LimitFinding made_finding(LimitKind k, double rpm) {
    LimitFinding f;
    f.kind = k;
    f.rpm = rpm;
    return f;
}
// The N7b ratio: hyper_turbo_n2o.json 1.4875e-3 m2 over hyper_turbo_approx.json 1.18e-3 m2 (~1.26x; it is the bisected area + 10 %).
constexpr double kN7bRatio = 1.4875e-3 / 1.18e-3;
} // namespace

// =====================================================================================================================
// Kit and bottle findings with their Closed remedies.

TEST_CASE("R4 KitFuelLean: a shrunk fuel jet is found and its Closed jet remedy restores the lambda when applied", "[drivetrain][dyno][n2o]") {
    // The shipped 800 hp kit runs lambda_c ~0.835 (above the 0.82 target, below the 0.90 Near line): no finding. Shrinking the
    // fuel jet to 0.7x leaves 0.49x of the kit fuel -> lambda_c 1.02 (Limiting); 0.5x -> 1.14 (Exceeded); 0.8x -> 0.96 (Near).
    // The remedy sizes the jet from the finding's own window (kit fuel ~ jet area, so d ~ sqrt(flow); ECU fuel and air taken as
    // unchanged - first order): applied and re-swept, required_min lands on lambda_c = 0.90 and required_margin on 0.82.
    // Recorded sabotage: d scaling linearly with the flow (instead of sqrt) -> the applied jets are far too big: lambda_c 0.576
    // at required_margin (target 0.82) and 0.753 at required_min (0.90), both bounds (0.015) fail.
    const Car& c = n2o_car();
    const DynoResult shipped = sweep(c.engine, c.nitrous, {3000.0, 5000.0, 7000.0});
    CHECK_FALSE(any_kind(shipped, LimitKind::KitFuelLean));
    for (const DynoPoint& p : shipped.points) {
        CHECK(p.lambda_combined > 0.82);
        CHECK(p.lambda_combined < 0.86);
    }

    const double d0 = c.nitrous->kit.fuel_jet_diameter_mm;
    const auto lean = [&](double scale) {
        DynoNitrous n = *c.nitrous;
        n.kit.fuel_jet_diameter_mm = d0 * scale;
        return n;
    };
    const struct {
        double scale;
        LimitSeverity severity;
    } ladder[] = {{0.8, LimitSeverity::Near}, {0.7, LimitSeverity::Limiting}, {0.5, LimitSeverity::Exceeded}};
    for (const auto& step : ladder) {
        const DynoResult r = sweep(c.engine, lean(step.scale), {5000.0});
        const LimitFinding* f = find(r, LimitKind::KitFuelLean);
        INFO("fuel jet x" << step.scale << " lambda_c " << r.points[0].lambda_combined);
        REQUIRE(f != nullptr);
        CHECK(f->severity == step.severity);
        CHECK(f->value == r.points[0].lambda_combined);
        CHECK(f->limit == limits::kKitLambdaNear.value);
        CHECK(f->rpm == 5000.0);
    }

    const DynoResult r = sweep(c.engine, lean(0.7), {5000.0});
    const LimitFinding* f = find(r, LimitKind::KitFuelLean);
    REQUIRE(f != nullptr);
    REQUIRE(f->remedy.has_value());
    const Remedy& rem = *f->remedy;
    CHECK(rem.cost == RemedyCost::Closed);
    CHECK(rem.solved);
    CHECK(rem.unit == LimitUnit::Millimetre);
    CHECK(rem.part.slot == "nitrous_kit");
    CHECK(rem.part.file == "kit.json");
    CHECK(rem.part.field == "/fuel_jet_diameter_mm");
    CHECK(rem.current == Catch::Approx(d0 * 0.7).epsilon(1e-12));
    CHECK(rem.required_min > rem.current);
    CHECK(rem.required_margin > rem.required_min);

    const DynoResult at_margin = sweep(c.engine, lean(rem.required_margin / d0), {5000.0});
    const DynoResult at_min = sweep(c.engine, lean(rem.required_min / d0), {5000.0});
    INFO("lambda_c at required_margin " << at_margin.points[0].lambda_combined << ", at required_min " << at_min.points[0].lambda_combined);
    CHECK(at_margin.points[0].lambda_combined == Catch::Approx(limits::kKitLambdaTarget.value).margin(0.015));
    CHECK(at_min.points[0].lambda_combined == Catch::Approx(limits::kKitLambdaNear.value).margin(0.015));
    CHECK_FALSE(any_kind(at_margin, LimitKind::KitFuelLean));
}

TEST_CASE("R5 N2OBottleDuration: a small bottle is found and the Closed bottle remedy gives the target duration", "[drivetrain][dyno][n2o]") {
    // 1 kg at 246.4 g/s lasts 4.06 s (< the 10 s pass [AS]). required_min = 10 s x flow; required_margin adds the 10 % that
    // stays below the pickup. Applied (capacity = fill): required_min lands exactly on 10 s (Near, the Near line is 11 s),
    // required_margin clears, 0.99 required_min is still Limiting.
    // Recorded sabotage: the margin without the reserve factor (kBottleReserve ignored) -> required_margin == required_min and
    // the applied margin is Near, not clear.
    const Car& c = n2o_car();
    const DynoResult shipped = sweep(c.engine, c.nitrous, {3000.0, 5000.0, 7000.0});
    CHECK_FALSE(any_kind(shipped, LimitKind::N2OBottleDuration));
    const double shipped_duration = c.nitrous->bottle.initial_kg / (shipped.points[0].n2o_g_s * 0.001);
    CHECK(shipped_duration > 27.0);
    CHECK(shipped_duration < 28.2);

    const auto with_fill = [&](double kg) {
        DynoNitrous n = *c.nitrous;
        n.bottle.capacity_kg = kg;
        n.bottle.initial_kg = kg;
        return n;
    };
    const DynoResult r = sweep(c.engine, with_fill(1.0), {3000.0, 5000.0, 7000.0});
    const LimitFinding* f = find(r, LimitKind::N2OBottleDuration);
    REQUIRE(f != nullptr);
    const double flow_kg_s = r.points[0].n2o_g_s * 0.001;
    CHECK(f->severity == LimitSeverity::Limiting);
    CHECK(f->unit == LimitUnit::Second);
    CHECK(f->value == Catch::Approx(1.0 / flow_kg_s).epsilon(1e-12));
    CHECK(f->limit == 10.0);
    REQUIRE(f->remedy.has_value());
    const Remedy& rem = *f->remedy;
    CHECK(rem.cost == RemedyCost::Closed);
    CHECK(rem.solved);
    CHECK(rem.unit == LimitUnit::Kilogram);
    CHECK(rem.part.slot == "n2o_bottle");
    CHECK(rem.part.file == "bottle.json");
    CHECK(rem.part.field == "/capacity_kg");
    CHECK(rem.current == 1.0);
    CHECK(rem.required_min == Catch::Approx(10.0 * flow_kg_s).epsilon(1e-12));
    CHECK(rem.required_margin == Catch::Approx(11.0 * flow_kg_s).epsilon(1e-12));

    const DynoResult at_min = sweep(c.engine, with_fill(rem.required_min), {5000.0});
    const LimitFinding* near = find(at_min, LimitKind::N2OBottleDuration);
    REQUIRE(near != nullptr);
    CHECK(near->severity == LimitSeverity::Near);
    CHECK(near->value == Catch::Approx(10.0).epsilon(1e-9));
    CHECK_FALSE(any_kind(sweep(c.engine, with_fill(rem.required_margin * 1.0001), {5000.0}), LimitKind::N2OBottleDuration));
    const DynoResult below = sweep(c.engine, with_fill(rem.required_min * 0.99), {5000.0});
    const LimitFinding* still = find(below, LimitKind::N2OBottleDuration);
    REQUIRE(still != nullptr);
    CHECK(still->severity == LimitSeverity::Limiting);
}

TEST_CASE("R6 N2OBottlePressureDrop: only a bottle whose pressure falls with the fill, severity by the flow it keeps", "[drivetrain][dyno][n2o]") {
    // A crafted vs_fill bottle (linear 20 bar empty .. 65.5 bar full [AS, test data]). The finding is the HEM flux at the end of a
    // 10 s pass over the flux at the start: full bottle 0.885 (Near, < 0.90), 3.0 kg 0.75..0.8 (Limiting, < 0.80). A controller
    // bottle switch (min_bottle_gauge_bar 45) that the end-of-pass gauge falls under is Exceeded. The shipped constant-pressure
    // bottle and a heated vs_fill bottle never drop.
    // Recorded sabotage: the end-of-pass fill not reduced (fill_end = initial) -> no finding at all.
    const Car& c = n2o_car();
    DynoNitrous vs = *c.nitrous;
    vs.kit.controller.safety.min_bottle_gauge_bar = 0.0;
    vs.bottle.pressure.mode = N2oPressureMode::VsFill;
    vs.bottle.pressure.gauge_vs_fill.x = {0.0, 1.0};
    vs.bottle.pressure.gauge_vs_fill.y = {20.0, 65.5};
    const auto drop_at = [&](double kg, double min_gauge) {
        DynoNitrous n = vs;
        n.bottle.initial_kg = kg;
        n.kit.controller.safety.min_bottle_gauge_bar = min_gauge;
        return sweep(c.engine, n, {5000.0});
    };

    const DynoResult full = drop_at(6.804, 0.0);
    const LimitFinding* near = find(full, LimitKind::N2OBottlePressureDrop);
    REQUIRE(near != nullptr);
    CHECK(near->severity == LimitSeverity::Near);
    CHECK(near->value == Catch::Approx(0.885).margin(0.01));
    CHECK(near->limit == limits::kBottleFlowRetentionNear.value);
    CHECK(near->part.field == "/pressure");
    CHECK_FALSE(near->remedy.has_value());

    const DynoResult low_fill = drop_at(3.0, 0.0);
    const LimitFinding* limiting = find(low_fill, LimitKind::N2OBottlePressureDrop);
    REQUIRE(limiting != nullptr);
    CHECK(limiting->severity == LimitSeverity::Limiting);
    CHECK(limiting->value < limits::kBottleFlowRetentionLimiting.value);

    const DynoResult switched = drop_at(5.5, 45.0);
    const LimitFinding* tripped = find(switched, LimitKind::N2OBottlePressureDrop);
    REQUIRE(tripped != nullptr);
    CHECK(tripped->severity == LimitSeverity::Exceeded);

    CHECK_FALSE(any_kind(sweep(c.engine, c.nitrous, {5000.0}), LimitKind::N2OBottlePressureDrop));
    DynoNitrous heated = vs;
    heated.bottle.has_heater = true;
    heated.bottle.heater.channel = "n2o_heater";
    heated.bottle.heater.default_on = 1.0;
    heated.bottle.heater.set_point_gauge_bar = 65.5;
    CHECK_FALSE(any_kind(sweep(c.engine, heated, {5000.0}), LimitKind::N2OBottlePressureDrop));
}

TEST_CASE("R7 shipped nitrous parts produce no kit or bottle finding; InjectorDuty is never produced", "[drivetrain][dyno][n2o]") {
    // No injector model exists anywhere in core/ (grep: only the finding kind's own comment): InjectorDuty has nothing to
    // measure, whatever the sweep.
    const Car& c = n2o_car();
    const DynoResult r = sweep(c.engine, c.nitrous, {3000.0, 4500.0, 6000.0, 7000.0});
    for (const LimitFinding& f : r.limits) {
        CHECK(f.kind != LimitKind::KitFuelLean);
        CHECK(f.kind != LimitKind::N2OBottleDuration);
        CHECK(f.kind != LimitKind::N2OBottlePressureDrop);
        CHECK(f.kind != LimitKind::InjectorDuty);
    }
    DynoNitrous lean = *c.nitrous;
    lean.kit.fuel_jet_diameter_mm *= 0.5;
    lean.bottle.initial_kg = 0.5;
    const DynoResult crafted = sweep(c.engine, lean, {5000.0});
    for (const LimitFinding& f : crafted.limits) CHECK(f.kind != LimitKind::InjectorDuty);
}

// =====================================================================================================================
// Simulated remedies.

TEST_CASE("R8 owner's example: stock hardware + the 800 hp kit -> placeholders, solve, apply, re-sweep", "[drivetrain][dyno][n2o][turbo]") {
    // car_hyper's wastegate (1.18e-3 m2 per turbo) with car_hyper_n2o's kit: the shot saturates the gate. The sweep attaches
    // UNSOLVED placeholders; solve_remedy at 7000 rpm finds the smallest per-turbo area at which the settled point has no
    // wastegate/boost/rotor-cap finding (mean gate opening < 0.92 [AS], boost within the 2 % band). Applied (margin, min) and
    // re-swept 3000-7000 rpm the findings are gone; 3 % below required_min they are not.
    // N7b comparison (reported, not asserted equal): N7b shipped 1.4875e-3 = 1.26x stock, "a bisection + 10 %" by the dyno-held
    // criterion gate <= 0.965. The same hardware under THIS criterion (gate < 0.92) still shows WastegateSaturated (Near) at
    // 7000 rpm, so the solve asks for more: ~1.32x as the minimum, ~1.45x with the 10 %.
    // Recorded sabotages: (a) the predicate always true -> required_min == current (ratio 1.0, the first CHECK range fails);
    // (b) the search returning the lower (non-clear) end of the bracket -> the applied required_min still has findings.
    const Car& c = n2o_car();
    const ComponentParams stock = stock_hardware_engine();
    RemedyRequest rq = request_for(stock, *c.nitrous, {3000.0, 5000.0, 7000.0});
    const DynoResult r = run_dyno_sweep(stock, nullptr, rq.spec, &pool10());

    const LimitFinding* sat = find(r, LimitKind::WastegateSaturated, 7000.0);
    REQUIRE(sat != nullptr);
    CHECK(sat->severity == LimitSeverity::Exceeded);
    const LimitFinding* boost = find(r, LimitKind::BoostAboveTarget, 7000.0);
    const LimitFinding* rotor = find(r, LimitKind::TurboRotorCap, 7000.0);
    REQUIRE(boost != nullptr);
    REQUIRE(rotor != nullptr);
    for (const LimitFinding* f : {sat, boost, rotor}) {
        REQUIRE(f->remedy.has_value());
        CHECK(f->remedy->cost == RemedyCost::Simulated);
        CHECK_FALSE(f->remedy->solved);
        CHECK(f->remedy->current == Catch::Approx(1.18e-3).epsilon(1e-12)); // per turbo, as in the hardware file
        CHECK(f->remedy->unit == LimitUnit::SquareMetre);
        CHECK(std::isnan(f->remedy->required_min));
        CHECK(std::isnan(f->remedy->required_margin));
        CHECK(f->remedy->part.slot == "turbo");
        CHECK(f->remedy->part.file == "turbo.json");
    }
    CHECK(sat->remedy->part.field == "/wastegate_area_m2");

    RemedyOptions o;
    o.pool = &pool10();
    std::size_t evals = 0;
    o.evaluations_out = &evals;
    const auto t0 = std::chrono::steady_clock::now();
    const Remedy rem = solve_remedy(rq, *sat, o);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    REQUIRE(rem.solved);
    const double ratio = rem.required_min / rem.current;
    std::printf("[R8] wastegate solve at 7000 rpm: x%.4f (required_min %.5g m2, margin %.5g m2 = x%.4f), N7b shipped x%.4f; %zu evaluations, %.2f s on 10 workers\n", ratio,
                rem.required_min, rem.required_margin, rem.required_margin / rem.current, kN7bRatio, evals, seconds);
    CHECK(rem.cost == RemedyCost::Simulated);
    CHECK(rem.unit == LimitUnit::SquareMetre);
    CHECK(rem.part.field == "/wastegate_area_m2");
    CHECK(rem.current == Catch::Approx(1.18e-3).epsilon(1e-12));
    CHECK(ratio > 1.25);
    CHECK(ratio < 1.40);
    CHECK(ratio > kN7bRatio); // stricter criterion than N7b's 0.965 (see above)
    CHECK(rem.required_margin == Catch::Approx(rem.required_min * 1.10).epsilon(1e-12));
    CHECK(evals == 16);

    const auto clear_with = [&](double per_turbo_m2) {
        ComponentParams e = stock;
        std::get<SimulatedEngineDesc>(e).turbo_pair->wastegate_area_m2 = per_turbo_m2 * 2.0;
        return !any_gate_finding(sweep(e, c.nitrous, {3000.0, 5000.0, 7000.0}));
    };
    CHECK(clear_with(rem.required_margin));
    CHECK(clear_with(rem.required_min));
    CHECK_FALSE(clear_with(rem.required_min * 0.97));

    // The N7b hardware under the same criterion: the 7000 rpm finding is a Near (gate 0.96), i.e. the solve at that point
    // asks for more than N7b shipped without the margin.
    const DynoResult n7b = sweep(c.engine, c.nitrous, {7000.0});
    const LimitFinding* n7b_sat = find(n7b, LimitKind::WastegateSaturated, 7000.0);
    REQUIRE(n7b_sat != nullptr);
    CHECK(n7b_sat->severity == LimitSeverity::Near);
    CHECK(n7b_sat->value > 0.92);
}

TEST_CASE("R9 TurboRotorCap: the wastegate clears one that came from a saturated gate, a boost target beyond the hardware gives solved = false", "[drivetrain][dyno][n2o][turbo]") {
    // (a) stock hardware at 7000 rpm: the rotor reaches the 180000 rpm table edge because the gate has no authority; the same
    // area clears it as clears the wastegate/boost kinds (the predicate for TurboRotorCap includes all three).
    // (b) target_boost 3 bar abs: at 6000 rpm the gate is shut and the rotor still hits the cap - a bigger gate changes nothing:
    // solved = false, part "turbo hardware" (empty field), no number.
    // Recorded sabotage: the rotor kind left out of the TurboRotorCap predicate (a boost-only predicate) -> (b) reports
    // solved = true with numbers and a non-empty field (4 checks fail).
    const Car& c = n2o_car();
    const ComponentParams stock = stock_hardware_engine();
    {
        RemedyRequest rq = request_for(stock, *c.nitrous, {7000.0});
        const Remedy rotor = solve_remedy(rq, made_finding(LimitKind::TurboRotorCap, 7000.0), {});
        const Remedy gate = solve_remedy(rq, made_finding(LimitKind::WastegateSaturated, 7000.0), {});
        REQUIRE(rotor.solved);
        REQUIRE(gate.solved);
        CHECK(rotor.required_min == Catch::Approx(gate.required_min).epsilon(1e-9));
        CHECK(rotor.part.field == "/wastegate_area_m2");
    }
    {
        ComponentParams e = c.engine;
        std::get<SimulatedEngineDesc>(e).turbo_pair->target_boost_pa = 3.0e5;
        RemedyRequest rq = request_for(e, *c.nitrous, {6000.0});
        const DynoResult r = run_dyno_sweep(e, nullptr, rq.spec, &pool10());
        const LimitFinding* cap = find(r, LimitKind::TurboRotorCap, 6000.0);
        REQUIRE(cap != nullptr);
        const Remedy rem = solve_remedy(rq, *cap, {});
        CHECK_FALSE(rem.solved);
        CHECK(std::isnan(rem.required_min));
        CHECK(std::isnan(rem.required_margin));
        CHECK(rem.part.slot == "turbo");
        CHECK(rem.part.file == "turbo.json");
        CHECK(rem.part.field.empty());
        CHECK(rem.current == Catch::Approx(1.4875e-3).epsilon(1e-12));
    }
}

TEST_CASE("R10 solve_power_target: the N2O jet for a power gain, an unreachable target is solved = false", "[drivetrain][dyno][n2o]") {
    // The kit's dyno-held gain at 5000 rpm is ~590 kW (Sketch maps). Asking for 450 kW solves for a smaller jet pair (the fuel jet
    // keeps the kit's diameter ratio); applied and re-swept the gain reaches the target (>= target, within ~2 % above: the
    // 0.5 % diameter tolerance is ~1 % in flow), and 2 % smaller in diameter it does not.
    // Recorded sabotage: the comparison inverted (gain <= target counts as satisfied) -> the solve collapses to a ~0.002 mm jet
    // (re-swept gain -31 kW, "g >= target" fails) and the unreachable target is reported solved (6 checks fail).
    const Car& c = n2o_car();
    RemedyRequest rq = request_for(c.engine, *c.nitrous, {5000.0});
    RemedyOptions o;
    o.pool = &pool10();
    const double target = 450.0;
    const auto t0 = std::chrono::steady_clock::now();
    const PowerTargetResult res = solve_power_target(rq, PowerTargetPart::NitrousJet, 5000.0, target, o);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[R10] power-target solve at 5000 rpm: jet %.4f mm (shipped %.4f), %zu evaluations, %.2f s on 10 workers\n", res.nitrous_jet.required_min, c.nitrous->kit.nitrous_jet_diameter_mm,
                res.evaluations, seconds);
    REQUIRE(res.solved);
    CHECK(res.nitrous_jet.cost == RemedyCost::Simulated);
    CHECK(res.nitrous_jet.solved);
    CHECK(res.nitrous_jet.unit == LimitUnit::Millimetre);
    CHECK(res.nitrous_jet.part.slot == "nitrous_kit");
    CHECK(res.nitrous_jet.part.field == "/nitrous_jet_diameter_mm");
    CHECK(res.fuel_jet.part.field == "/fuel_jet_diameter_mm");
    const double d_n2o = c.nitrous->kit.nitrous_jet_diameter_mm, d_fuel = c.nitrous->kit.fuel_jet_diameter_mm;
    CHECK(res.nitrous_jet.current == d_n2o);
    CHECK(res.fuel_jet.current == d_fuel);
    CHECK(res.nitrous_jet.required_min < d_n2o);
    CHECK(res.nitrous_jet.required_margin == res.nitrous_jet.required_min); // a power target has no margin
    CHECK(res.fuel_jet.required_min / res.nitrous_jet.required_min == Catch::Approx(d_fuel / d_n2o).epsilon(1e-12));
    CHECK(res.evaluations >= 5);

    const DynoResult base = sweep(c.engine, std::nullopt, {5000.0});
    CHECK(res.baseline_kw == base.points[0].power_kw);
    const auto gain_with = [&](double n2o_scale) {
        DynoNitrous n = *c.nitrous;
        n.kit.nitrous_jet_diameter_mm = res.nitrous_jet.required_min * n2o_scale;
        n.kit.fuel_jet_diameter_mm = res.fuel_jet.required_min * n2o_scale;
        return sweep(c.engine, n, {5000.0}).points[0].power_kw - base.points[0].power_kw;
    };
    const double g = gain_with(1.0);
    INFO("target " << target << " kW, solved jet " << res.nitrous_jet.required_min << " mm (shipped " << d_n2o << "), gain " << g << ", 2 % smaller " << gain_with(0.98));
    CHECK(g >= target);
    CHECK(g < target * 1.02);
    CHECK(res.achieved_gain_kw == Catch::Approx(g).epsilon(1e-9));
    CHECK(gain_with(0.98) < target);

    // Unreachable: no candidate (up to 4x the shipped jets) gives a gigawatt.
    const PowerTargetResult none = solve_power_target(rq, PowerTargetPart::NitrousJet, 5000.0, 1.0e6, o);
    CHECK_FALSE(none.solved);
    CHECK_FALSE(none.nitrous_jet.solved);
    CHECK(std::isnan(none.nitrous_jet.required_min));
    CHECK(std::isnan(none.fuel_jet.required_min));
    CHECK(none.evaluations >= 5);
    // A kit the controller does not spray at that rpm gains nothing: also unreachable.
    DynoNitrous gated = *c.nitrous;
    gated.kit.controller.max_rpm = 4000.0;
    const PowerTargetResult off = solve_power_target(request_for(c.engine, gated, {5000.0}), PowerTargetPart::NitrousJet, 5000.0, 100.0, o);
    CHECK_FALSE(off.solved);
}

TEST_CASE("R11 solve progress and cancel: calling-thread progress, a closing (total, total), a pre-set or mid-solve flag throws GenerationCancelled", "[drivetrain][dyno][n2o][turbo]") {
    // Progress/cancel contract of P1/P3 (EngineMapOptions, run_dyno_sweep), for both solvers. Recorded sabotage: the flag never
    // read (cancelled() returning false, evaluations given no flag) -> the pre-set and the mid-solve THROWS checks fail and the
    // solve runs to the end (17 progress calls, 16 evaluations; 8 checks fail).
    const Car& c = n2o_car();
    const ComponentParams stock = stock_hardware_engine();
    const RemedyRequest wg = request_for(stock, *c.nitrous, {5000.0});
    const RemedyRequest pt = request_for(c.engine, *c.nitrous, {5000.0});
    const LimitFinding fnd = made_finding(LimitKind::WastegateSaturated, 5000.0);
    const auto call_wg = [&](const RemedyOptions& o) { return solve_remedy(wg, fnd, o); };
    const auto call_pt = [&](const RemedyOptions& o) { return solve_power_target(pt, PowerTargetPart::NitrousJet, 5000.0, 450.0, o).solved; };

    for (int which = 0; which < 2; ++which) {
        INFO((which == 0 ? "solve_remedy" : "solve_power_target"));
        const auto call = [&](const RemedyOptions& o) {
            if (which == 0) (void)call_wg(o);
            else (void)call_pt(o);
        };
        // Progress: monotone, on the calling thread, closed by (total, total).
        std::vector<std::pair<std::size_t, std::size_t>> calls;
        std::vector<bool> on_caller;
        const std::thread::id me = std::this_thread::get_id();
        RemedyOptions o;
        o.pool = &pool10();
        o.on_progress = [&](std::size_t done, std::size_t total) {
            calls.emplace_back(done, total);
            on_caller.push_back(std::this_thread::get_id() == me);
        };
        call(o);
        REQUIRE(calls.size() >= 5);
        for (std::size_t i = 0; i < calls.size(); ++i) {
            CHECK(on_caller[i]);
            CHECK(calls[i].second == calls[0].second);
            CHECK(calls[i].first <= calls[i].second);
            CHECK(calls[i].first >= 1);
            if (i > 0) CHECK(calls[i].first >= calls[i - 1].first);
        }
        CHECK(calls.back().first == calls.back().second);

        // Cancel set on entry: nothing runs.
        std::atomic<bool> flag{true};
        std::size_t progress_calls = 0;
        RemedyOptions pre;
        pre.pool = &pool10();
        pre.cancel = &flag;
        pre.on_progress = [&](std::size_t, std::size_t) { ++progress_calls; };
        CHECK_THROWS_AS(call(pre), GenerationCancelled);
        CHECK(progress_calls == 0);

        // Cancel set from the first progress call (after round 0): seen before the next round.
        std::atomic<bool> mid{false};
        std::size_t seen = 0;
        RemedyOptions during;
        during.pool = &pool10();
        during.cancel = &mid;
        during.on_progress = [&](std::size_t, std::size_t) {
            ++seen;
            mid.store(true);
        };
        CHECK_THROWS_AS(call(during), GenerationCancelled);
        CHECK(seen >= 1);
        CHECK(seen < calls.size());
    }
}

TEST_CASE("R12 a solve is bit-identical for no pool, 1 worker and 10 workers", "[drivetrain][dyno][n2o][turbo][long]") {
    // The 4-section search evaluates the same values whatever the pool; each evaluation is a serial single-point sweep.
    // Recorded sabotage: the interior fractions scaled by 1.03 whenever a pool was used -> required_min 1.43252e-3 vs
    // 1.43352e-3 and the power-target jet 3.3578 vs 3.3561 mm (8 checks fail).
    const Car& c = n2o_car();
    const ComponentParams stock = stock_hardware_engine();
    const RemedyRequest wg = request_for(stock, *c.nitrous, {5000.0});
    const RemedyRequest pt = request_for(c.engine, *c.nitrous, {5000.0});
    static jobs::JobSystem one(1);
    jobs::JobSystem* pools[3] = {nullptr, &one, &pool10()};
    Remedy w[3];
    PowerTargetResult p[3];
    for (int i = 0; i < 3; ++i) {
        RemedyOptions o;
        o.pool = pools[i];
        w[i] = solve_remedy(wg, made_finding(LimitKind::WastegateSaturated, 5000.0), o);
        p[i] = solve_power_target(pt, PowerTargetPart::NitrousJet, 5000.0, 450.0, o);
    }
    for (int i = 1; i < 3; ++i) {
        CHECK(w[i].solved == w[0].solved);
        CHECK(w[i].required_min == w[0].required_min);
        CHECK(w[i].required_margin == w[0].required_margin);
        CHECK(p[i].solved == p[0].solved);
        CHECK(p[i].nitrous_jet.required_min == p[0].nitrous_jet.required_min);
        CHECK(p[i].achieved_gain_kw == p[0].achieved_gain_kw);
        CHECK(p[i].evaluations == p[0].evaluations);
    }
}

TEST_CASE("R13 solver argument validation", "[drivetrain][dyno][n2o][turbo]") {
    const Car& c = n2o_car();
    const RemedyRequest rq = request_for(c.engine, *c.nitrous, {5000.0});
    // Kinds without a simulated remedy.
    CHECK_THROWS_AS(solve_remedy(rq, made_finding(LimitKind::KitFuelLean, 5000.0), {}), std::invalid_argument);
    CHECK_THROWS_AS(solve_remedy(rq, made_finding(LimitKind::ClutchCapacity, 5000.0), {}), std::invalid_argument);
    // A static (NaN rpm) finding has nothing to evaluate.
    LimitFinding no_rpm;
    no_rpm.kind = LimitKind::WastegateSaturated;
    CHECK_THROWS_AS(solve_remedy(rq, no_rpm, {}), std::invalid_argument);
    RemedyOptions bad_tol;
    bad_tol.tolerance_rel = 0.0;
    CHECK_THROWS_AS(solve_remedy(rq, made_finding(LimitKind::WastegateSaturated, 5000.0), bad_tol), std::invalid_argument);
    // A torque-map engine has no turbo.
    RemedyRequest tm = rq;
    tm.engine = io::load_engine_json(kEngines + "i4_2l_na.json");
    CHECK_THROWS_AS(solve_remedy(tm, made_finding(LimitKind::WastegateSaturated, 5000.0), {}), std::invalid_argument);
    CHECK_THROWS_AS(solve_power_target(tm, PowerTargetPart::NitrousJet, 5000.0, 100.0, {}), std::invalid_argument);
    // Power target: rpm and target finite and > 0, a kit is required.
    CHECK_THROWS_AS(solve_power_target(rq, PowerTargetPart::NitrousJet, 0.0, 100.0, {}), std::invalid_argument);
    CHECK_THROWS_AS(solve_power_target(rq, PowerTargetPart::NitrousJet, 5000.0, 0.0, {}), std::invalid_argument);
    CHECK_THROWS_AS(solve_power_target(rq, PowerTargetPart::NitrousJet, 5000.0, std::numeric_limits<double>::quiet_NaN(), {}), std::invalid_argument);
    RemedyRequest no_kit = rq;
    no_kit.spec.nitrous.reset();
    CHECK_THROWS_AS(solve_power_target(no_kit, PowerTargetPart::NitrousJet, 5000.0, 100.0, {}), std::invalid_argument);
}

