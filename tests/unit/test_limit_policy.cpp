// tests/unit/test_limit_policy.cpp - increment L0 (owner decision 2026-10-13): refusal limits and advice thresholds are CONFIG.
// ps::drivetrain::LimitPolicy (ps/drivetrain/limit_policy.h), its file format physics_sim.limit_policy/1
// (data/limit_policies/default.json, docs/data_formats/limit_policy.md) and every consumer that reads it:
// DynoSweepSpec::policy (run_dyno_sweep, solve_remedy), check_driveline_limits, check_buildable, check_gearbox_rating,
// has_blocking_issue, has_refusing_finding.
//
// EXPECTED RESULTS (committed before the first run; "RUN" lines at the end of each case header are filled in afterwards):
//   LP1  default.json == the compiled defaults: 58 POLICY entries, value AND source text identical to the table headers
//        (limit_thresholds.h 31 + engine_rules_table.h 27), every entry present exactly once, no extra key; the 13 NOT-POLICY
//        entries (31 + 40 = 71 table entries, 71 - 58 = 13) are exactly the listed names and are in no file; the loaded
//        LimitPolicy equals default_limit_policy() field by field; validate_limit_policy(default) == "".
//   LP2  reachability: for each of the 58 entries, setting it to an extreme (1e-9 or 1e9) through a policy changes the result
//        of at least one consumer probe (findings, build issues, remedies); a probe given nullptr, &default_limit_policy() and
//        a default-constructed LimitPolicy gives the same bytes. An entry that no consumer reads FAILS, naming the entry.
//        LP2a runs the cheap probes (build rules, driveline), LP2b the dyno probes ([long]: hyper sweeps, one solve).
//   LP3  refuse_at / warn_at: the 3 x 4 severity matrices; the default refuses only Exceeded and warns from Near; has_blocking_issue
//        and has_refusing_finding follow the policy (gearbox 1100/1000 = Limiting: not blocking by default, blocking at
//        refuse_at "limiting", never blocking at "never"); the findings of a sweep are IDENTICAL for every refuse_at / warn_at.
//   LP4  extends and the user layer: chain precedence a < b < c < user, a user-layer null restores the COMPILED default, partial
//        files keep the defaults of what they omit, both value forms ({"value","source"} and a bare number) load.
//   LP5  every rejection names the file (or "user layer") and the key; the listed cases all throw.
//   LP6  a policy never reaches a map key, a turbo key, a dyno point or a World state hash: engine_map_cache_key and
//        turbo_table_key unchanged, DynoResult::points byte-identical (as text) for default and extreme policies, the
//        sedan's state hash identical at 1 and 10 workers with consumers running between its ticks.
//
// MEASURED (Windows release, 2026-10-13): all 8 cases pass, 1287 assertions, 10.6 s including the [long] LP2b. First run: three
// of my own probe assumptions were wrong and were corrected in the test, not in production: the loader's top-level message reads
// "top level must be a JSON object" (json_extends.h); piston_speed_near_mps is read only by the dyno (the build rule starts at
// Limiting); the A25A's ECU spark schedule at 3000 rpm is 3.66 deg (not > 10), the Limiting knock case runs on a negative table.
//
// Recorded sabotages (production code changed, case failed with the quoted symptom, change reverted):
//   S1 check_buildable(ComponentParams) building its Sink on default_limit_policy() and has_blocking_issue ignoring its policy:
//      LP2a 25 entries "read by no cheap consumer probe" (piston limiting/exceeded, valves, rod, compression, grid overshoot,
//      cam, lift) + LP3 (blocking-issue rows). S1, S3, S6 and a weaker S4 were applied together in that run: 4 cases failed,
//      46 failed assertions in all (LP2a, LP2b, LP3, LP4).
//   S2 default.json gate_near_opening 0.92 -> 0.93: LP1 (two checks) and LP4 (one check) fail (3 failed assertions).
//   S3 piston_speed_finding reading a literal 23.0 for the Near step: LP2b "piston_speed/piston_speed_near_mps is read by no dyno
//      consumer probe" (contained in the S1 run, whose other failures are LP2b's default-vs-nullptr identity lines).
//   S4 NOT-POLICY entry made configurable (stepped_point's settle tolerances scaled by policy->boost_band_near / 0.02): LP6
//      "hyper point 0" and "hyper point 1" fail (the 5000 and 7000 rpm points differ under the extreme policy); a first variant
//      scaling only the boost tolerance did NOT fail LP6 (the torque criterion still gated the settle), so the sabotage was
//      widened to both.
//   S5 refuse_at ignored (LimitPolicy::refuses always Exceeded) + domain check skipped in build() + unknown entry keys skipped:
//      LP3 7 failures and LP5 13 failures, 20 failed assertions.
//   S6 the user layer merged BEFORE the extends chain (the chain wins): LP4 fails (precedence block).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "drivetrain/engine_rules_table.h"
#include "drivetrain/limit_policy_table.h"
#include "drivetrain/limit_thresholds.h"
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/limit_policy.h"
#include "ps/drivetrain/limit_remedies.h"
#include "ps/io/limit_policy_io.h"
#include "ps/io/nitrous_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
namespace pt = ps::drivetrain::policy_table;

namespace {

const std::string kRoot = PS_SOURCE_DIR;
const std::string kEngines = kRoot + "/data/engines/";
const std::string kVehicles = kRoot + "/data/vehicles/";
const std::string kDefaultJson = kRoot + "/data/limit_policies/default.json";

// ---------------------------------------------------------------------------------------------------------------------
// Text signatures: every number at full precision, so ANY change of a result shows.

std::string num(double v) {
    char b[48];
    std::snprintf(b, sizeof(b), "%.17g", v);
    return b;
}

std::string sig(const LimitFinding& f) {
    std::string s = std::string(to_string(f.kind)) + "|" + to_string(f.severity) + "|" + num(f.rpm) + "|" + num(f.value) + "|" +
                    num(f.limit) + "|" + std::to_string(static_cast<int>(f.unit)) + "|" + f.part.slot + "|" + f.part.file + "|" + f.part.field;
    if (f.remedy.has_value()) {
        const Remedy& r = *f.remedy;
        s += "|R:" + num(r.current) + "," + num(r.required_min) + "," + num(r.required_margin) + "," + (r.solved ? "1" : "0") + "," +
             std::to_string(static_cast<int>(r.cost)) + "," + r.part.field;
    }
    return s;
}
std::string sig(const std::vector<LimitFinding>& v) {
    std::string out;
    for (const LimitFinding& f : v) out += sig(f) + "\n";
    return out;
}
std::string sig(const BuildIssue& b) {
    return std::string(to_string(b.rule)) + "|" + to_string(b.severity) + "|" + b.slot + "|" + b.field_pointer + "|" + num(b.value) + "|" +
           num(b.limit) + "|" + std::to_string(static_cast<int>(b.unit)) + "|" + b.source_label + "|" + b.message;
}
std::string sig(const std::vector<BuildIssue>& v) {
    std::string out;
    for (const BuildIssue& b : v) out += sig(b) + "\n";
    return out;
}
std::string sig(const DynoPoint& p) {
    std::string s;
    for (double v : {p.rpm, p.torque_nm, p.power_kw, p.boost_bar, p.turbo_rpm, p.fuel_g_s, p.bsfc_g_kwh, p.lambda, p.spark_deg,
                     p.p_intake_kpa, p.exhaust_k, p.charge_boost_bar, p.knock_retard_deg, p.n2o_g_s, p.kit_fuel_g_s, p.lambda_combined,
                     p.settle_s, p.lift_u})
        s += num(v) + ",";
    s += std::to_string(p.converged ? 1 : 0) + "," + std::to_string(p.cam_profile);
    return s;
}
std::string sig(const DynoResult& r) {
    std::string s = sig(r.limits);
    s += "points:\n";
    for (const DynoPoint& p : r.points) s += sig(p) + "\n";
    return s;
}
// Findings only (a probe compares what the policy decides, the points are LP6's business).
std::string sig_limits(const DynoResult& r) { return sig(r.limits); }

// ---------------------------------------------------------------------------------------------------------------------
// Policy helpers.

LimitPolicy with(const std::string& name, double v) {
    LimitPolicy p;
    for (const pt::Row& r : pt::kRows) {
        if (name == r.name) {
            p.*r.member = v;
            return p;
        }
    }
    FAIL("no policy entry named " << name);
    return p;
}
// Alternating 1e-9 / 1e9 over every entry: nothing like a sensible policy; consumers must still not change the physics.
LimitPolicy extreme_policy() {
    LimitPolicy p;
    int i = 0;
    for (const pt::Row& r : pt::kRows) p.*r.member = (i++ % 2 == 0) ? 1e-9 : 1e9;
    return p;
}

// ---------------------------------------------------------------------------------------------------------------------
// Engines.

jobs::JobSystem& pool10() {
    static jobs::JobSystem p(10);
    return p;
}
std::string cache_dir() {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "ps_garage_p3b1_maps"; // shared with test_limit_findings
    std::filesystem::create_directories(p);
    return p.string();
}
io::EngineMapOptions sketch_options() {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool10();
    o.cache_dir = cache_dir();
    return o;
}
io::EngineMapOptions validate_only() {
    io::EngineMapOptions o;
    o.generate = false;
    return o;
}
ComponentParams load_vo(const std::string& file) { return io::load_engine_json(kEngines + file, validate_only()); }

const ComponentParams& hyper() {
    static const ComponentParams c = io::load_engine_json(kEngines + "hyper_v8_one1_audio.json", sketch_options());
    return c;
}
const ComponentParams& a25a() {
    static const ComponentParams c = io::load_engine_json(kEngines + "toyota_a25a_fks_sim.json");
    return c;
}
const ComponentParams& hyper_vo() {
    static const ComponentParams c = load_vo("hyper_v8_one1_audio.json");
    return c;
}
const ComponentParams& demo_i4() {
    static const ComponentParams c = load_vo("demo_i4_pipes.json");
    return c;
}
const ComponentParams& k20a() {
    static const ComponentParams c = load_vo("honda_k20a_dc5.json");
    return c;
}
const ComponentParams& n52() {
    static const ComponentParams c = load_vo("bmw_n52b30.json");
    return c;
}
const ComponentParams& i4_torque_map() {
    static const ComponentParams c = load_vo("i4_2l_na.json");
    return c;
}
SimulatedEngineDesc& S(ComponentParams& c) { return std::get<SimulatedEngineDesc>(c); }
const SimulatedEngineDesc& S(const ComponentParams& c) { return std::get<SimulatedEngineDesc>(c); }

ComponentParams clean_i4() {
    ComponentParams c = demo_i4();
    S(c).limiter.rpm = 6500.0;
    return c;
}
ComponentParams clean_hyper() {
    ComponentParams c = hyper_vo();
    S(c).limiter.rpm = 7500.0;
    return c;
}
ComponentParams hyper_crafted(double bore_sq, double wastegate) {
    ComponentParams c = hyper();
    SimulatedEngineDesc& d = S(c);
    d.piston.bore_m *= std::sqrt(bore_sq);
    d.turbo_pair->wastegate_area_m2 = d.turbo_pair->wastegate_area() * wastegate;
    return c;
}
DynoSweepSpec turbo_spec(std::vector<double> rpm, const LimitPolicy* policy) {
    DynoSweepSpec s;
    s.rpm = std::move(rpm);
    s.part_files.engine = "engine.json";
    s.part_files.turbo = "turbo.json";
    s.part_files.turbo_configuration = "turbo_config.json";
    s.policy = policy;
    return s;
}
DynoResult sweep_hyper(double bore_sq, double wastegate, std::vector<double> rpm, const LimitPolicy* policy) {
    return run_dyno_sweep(hyper_crafted(bore_sq, wastegate), nullptr, turbo_spec(std::move(rpm), policy), &pool10());
}

// The N7b hypercar with its 800 hp kit (Sketch maps), as test_limit_remedies.cpp loads it.
struct Car {
    ComponentParams engine;
    std::optional<DynoNitrous> nitrous;
};
const Car& n2o_car() {
    static const Car c = [] {
        Car out;
        const vehicle::VehicleDesc desc = io::load_vehicle_json(kVehicles + "car_hyper_n2o.json", sketch_options());
        for (const ComponentDesc& cd : desc.powertrain.components)
            if (std::holds_alternative<SimulatedEngineDesc>(cd.params)) out.engine = cd.params;
        out.nitrous = nitrous_from_powertrain(desc.powertrain);
        return out;
    }();
    return c;
}
DynoSweepSpec n2o_spec(std::vector<double> rpm, const DynoNitrous& n, const LimitPolicy* policy) {
    DynoSweepSpec s;
    s.rpm = std::move(rpm);
    s.turbo_count = 2;
    s.nitrous = n;
    s.part_files.engine = "engine.json";
    s.part_files.turbo = "turbo.json";
    s.part_files.nitrous_kit = "kit.json";
    s.part_files.n2o_bottle = "bottle.json";
    s.policy = policy;
    return s;
}
DynoResult sweep_n2o(const DynoNitrous& n, std::vector<double> rpm, const LimitPolicy* policy) {
    return run_dyno_sweep(n2o_car().engine, nullptr, n2o_spec(std::move(rpm), n, policy), &pool10());
}

GearboxDesc rated_box(double nm) {
    GearboxDesc g;
    g.forward_ratios = {3.0, 2.0, 1.0};
    g.forward_efficiency = {1.0, 1.0, 1.0};
    g.rated_input_torque_nm = nm;
    return g;
}

// ---------------------------------------------------------------------------------------------------------------------
// The probes: one consumer scenario each, a function (policy pointer) -> text signature of everything the policy decides.

using Probe = std::function<std::string(const LimitPolicy*)>;

// ---- cheap: build rules ----
std::string probe_rules(const LimitPolicy* p) {
    std::string out;
    const auto add = [&](const char* tag, const std::vector<BuildIssue>& v) { out += std::string(tag) + ":\n" + sig(v); };
    // valve layouts 1V / 2V / 3V at the middle of their bands (every one issue-free by default, test_engine_rules C1)
    for (const auto& [ni, ne] : std::vector<std::pair<int, int>>{{1, 1}, {2, 1}, {2, 2}, {3, 2}}) {
        ComponentParams c = clean_i4();
        CycleModelDesc& cy = *S(c).cycle;
        cy.intake_valves = ni;
        cy.exhaust_valves = ne;
        const real bore = cy.bore_m;
        cy.intake_valve_d_m = bore * (ni == 1 ? 0.45 : ni == 2 ? 0.36 : 0.28);
        cy.exhaust_valve_d_m = 0.84 * cy.intake_valve_d_m;
        add(("valves" + std::to_string(ni) + std::to_string(ne)).c_str(), check_buildable(c, p));
    }
    // a valve head that overfills the bore (the packing area share), rod, compression ratio (NA and forced)
    {
        ComponentParams c = clean_i4();
        CycleModelDesc& cy = *S(c).cycle;
        cy.intake_valve_d_m = 0.0369;
        cy.exhaust_valve_d_m = 0.0328;
        add("pack", check_buildable(c, p));
    }
    {
        ComponentParams c = clean_i4();
        S(c).cycle->rod_m = 0.150;
        add("rod_mid", check_buildable(c, p));
    }
    add("hyper", check_buildable(clean_hyper(), p));
    add("i4", check_buildable(clean_i4(), p));
    // limiter beyond the grid: Near / Limiting / Exceeded by the overshoot (test_engine_rules C9)
    for (double rpm : {6800.0, 7200.0, 8000.0}) {
        ComponentParams c = clean_i4();
        S(c).limiter.rpm = rpm;
        add(("overshoot" + std::to_string(static_cast<int>(rpm))).c_str(), check_buildable(c, p));
    }
    // mean piston speed ladder (grid extended so only the piston rule speaks)
    for (double rpm : {8000.0, 9000.0, 9500.0, 10000.0, 10500.0, 11000.0}) {
        ComponentParams c = clean_i4();
        SimulatedEngineDesc& e = S(c);
        e.limiter.rpm = rpm;
        e.cycle->grid_rpm = extend_grid_rpm(*e.cycle, rpm);
        add(("piston" + std::to_string(static_cast<int>(rpm))).c_str(), check_buildable(c, p));
    }
    add("k20a", check_buildable(k20a(), p));
    add("n52", check_buildable(n52(), p));
    add("torque_map", check_buildable(i4_torque_map(), p));
    return out;
}

// ---- cheap: driveline and gearbox rating ----
std::string probe_driveline(const LimitPolicy* p) {
    std::string out;
    DynoResult d;
    DynoPoint pt_;
    pt_.rpm = 4000.0;
    pt_.torque_nm = 500.0;
    d.points.push_back(pt_);
    // clutch capacity (static) and gearbox rating pairs: Limiting/Limiting, Near/Exceeded, none/Near
    const struct {
        double clutch, box;
    } cases[] = {{480.0, 450.0}, {600.0, 420.0}, {10000.0, 480.0}};
    for (const auto& cs : cases) {
        PowertrainDesc pd;
        ClutchDesc cl;
        cl.capacity_nm = cs.clutch;
        cl.static_factor = 1.0;
        pd.components.push_back(ComponentDesc{"clutch", cl});
        pd.components.push_back(ComponentDesc{"gearbox", rated_box(cs.box)});
        out += sig(check_driveline_limits(d, pd, "clutch.json", "gearbox.json", p));
    }
    for (double peak : {1030.0, 1100.0, 1200.0}) out += sig(check_gearbox_rating(rated_box(1000.0), peak, p));
    return out;
}

// ---- dyno: turbo window findings (hyper, Sketch maps) ----
std::string probe_t16(const LimitPolicy* p) { return sig_limits(sweep_hyper(1.6, 1.0, {7000.0}, p)); }
std::string probe_t20(const LimitPolicy* p) { return sig_limits(sweep_hyper(2.0, 1.0, {7000.0}, p)); }
std::string probe_t110(const LimitPolicy* p) { return sig_limits(sweep_hyper(1.10, 1.0, {8000.0}, p)); }
std::string probe_rotor(const LimitPolicy* p) {
    std::string out;
    for (double k : {1.02, 1.05, 1.08}) out += sig_limits(sweep_hyper(k, 1.0, {8000.0}, p));
    return out;
}
std::string probe_surge(const LimitPolicy* p) {
    const auto run_with_line = [&](double y) {
        ComponentParams c = hyper();
        SurgeLineDesc line;
        line.flow_fraction_vs_pr.x = {1.0, 10.0};
        line.flow_fraction_vs_pr.y = {y, y};
        line.source = "test: flat line";
        S(c).turbo_pair->surge_line = line;
        return run_dyno_sweep(c, nullptr, turbo_spec({5000.0}, p), &pool10());
    };
    // the flow fraction is independent of the line (the flag feeds nothing): read it from a default-policy run
    static const double flow_fraction = [&] {
        ComponentParams c = hyper();
        SurgeLineDesc line;
        line.flow_fraction_vs_pr.x = {1.0, 10.0};
        line.flow_fraction_vs_pr.y = {0.99, 0.99};
        line.source = "test: flat line";
        S(c).turbo_pair->surge_line = line;
        const DynoResult r = run_dyno_sweep(c, nullptr, turbo_spec({5000.0}, nullptr), &pool10());
        for (const LimitFinding& f : r.limits)
            if (f.kind == LimitKind::CompressorSurge) return f.value * 0.99;
        return 0.0;
    }();
    REQUIRE(flow_fraction > 0.0);
    return sig_limits(run_with_line(0.99)) + sig_limits(run_with_line(flow_fraction / 1.05));
}
std::string probe_solve(const LimitPolicy* p) {
    RemedyRequest rq;
    rq.engine = hyper_crafted(2.0, 1.0);
    rq.spec = turbo_spec({7000.0}, p);
    rq.spec.turbo_count = 2;
    LimitFinding f;
    f.kind = LimitKind::WastegateSaturated;
    f.rpm = 7000.0;
    RemedyOptions o;
    o.pool = &pool10();
    const Remedy r = solve_remedy(rq, f, o);
    return num(r.current) + "," + num(r.required_min) + "," + num(r.required_margin) + "," + (r.solved ? "1" : "0");
}

// ---- dyno: NA A25A (limiter, piston speed, grid edge, knock) ----
std::string probe_limiter(const LimitPolicy* p) {
    ComponentParams c = a25a();
    SimulatedEngineDesc& d = S(c);
    d.limiter.type = LimiterType::SoftCut;
    d.limiter.rpm = 6500.0;
    d.limiter.band_rpm = 200.0;
    DynoSweepSpec s;
    s.rpm = {5000.0, 6000.0, 6200.0, 6400.0, 6500.0, 6600.0};
    s.policy = p;
    std::string out = sig_limits(run_dyno_sweep(c, nullptr, s, nullptr));
    out += sig_limits(run_dyno_sweep(a25a(), nullptr, s, nullptr)); // FuelCut
    return out;
}
std::string probe_piston(const LimitPolicy* p) {
    std::string out;
    for (double k : {1.05, 1.20, 1.35}) {
        ComponentParams c = a25a();
        S(c).piston.stroke_m *= k;
        DynoSweepSpec s;
        s.rpm = {6500.0};
        s.policy = p;
        out += sig_limits(run_dyno_sweep(c, nullptr, s, nullptr));
    }
    return out;
}
std::string probe_edge(const LimitPolicy* p) {
    std::string out;
    ComponentParams c = a25a();
    for (real& y : S(c).maps.ve.y) y *= 1.04; // top just above the wide-open intake pressure: Near
    DynoSweepSpec s;
    s.rpm = {3000.0};
    s.part_files.engine = "a25a.json";
    s.policy = p;
    out += sig_limits(run_dyno_sweep(c, nullptr, s, nullptr));
    DynoSweepSpec low;
    low.rpm = {500.0}; // below the rpm axis: Exceeded on the lower edge
    low.policy = p;
    out += sig_limits(run_dyno_sweep(a25a(), nullptr, low, nullptr));
    return out;
}
std::string probe_knock(const LimitPolicy* p) {
    const auto with_table = [&](double kl_offset_from_schedule, double schedule) {
        ComponentParams c = a25a();
        SimulatedEngineDesc& d = S(c);
        d.maps.has_theta_kl = true;
        d.maps.theta_kl_deg = d.maps.ve;
        for (real& v : d.maps.theta_kl_deg.values) v = schedule - kl_offset_from_schedule;
        return c;
    };
    // the ECU schedule at the point: a knock table of 0 limits every point to 0 deg, so the retard IS the schedule
    static const double schedule = [&] {
        DynoSweepSpec s;
        s.rpm = {3000.0};
        const DynoResult r = run_dyno_sweep(with_table(0.0, 0.0), nullptr, s, nullptr);
        return r.points[0].knock_retard_deg;
    }();
    REQUIRE(schedule > 0.5); // measured 3.66 deg at 3000 rpm on the A25A; the Limiting case below asks for a retard of 7 deg (kl < 0)
    std::string out;
    for (double retard : {1.0, 7.0}) { // Near, Limiting at the defaults (0.05 / 5 deg)
        DynoSweepSpec s;
        s.rpm = {3000.0};
        s.part_files.fuel = "fuel.json";
        s.policy = p;
        out += sig_limits(run_dyno_sweep(with_table(retard, schedule), nullptr, s, nullptr));
    }
    return out;
}

// ---- dyno: nitrous kit and bottle ----
std::string probe_kit(const LimitPolicy* p) {
    const Car& c = n2o_car();
    std::string out;
    for (double scale : {0.8, 0.7, 0.5}) { // lambda_c Near / Limiting / Exceeded at the defaults
        DynoNitrous n = *c.nitrous;
        n.kit.fuel_jet_diameter_mm *= scale;
        out += sig_limits(sweep_n2o(n, {5000.0}, p));
    }
    return out;
}
std::string probe_bottle(const LimitPolicy* p) {
    const Car& c = n2o_car();
    std::string out;
    const auto fill = [&](double kg) {
        DynoNitrous n = *c.nitrous;
        n.bottle.capacity_kg = kg;
        n.bottle.initial_kg = kg;
        return n;
    };
    const DynoResult small = sweep_n2o(fill(1.0), {5000.0}, p);
    out += sig_limits(small);
    const double flow_kg_s = small.points[0].n2o_g_s * 0.001;
    out += sig_limits(sweep_n2o(fill(10.5 * flow_kg_s), {5000.0}, p)); // 10.5 s: Near at the defaults (pass 10 s, Near line 11 s)
    DynoNitrous vs = *c.nitrous;
    vs.kit.controller.safety.min_bottle_gauge_bar = 0.0;
    vs.bottle.pressure.mode = N2oPressureMode::VsFill;
    vs.bottle.pressure.gauge_vs_fill.x = {0.0, 1.0};
    vs.bottle.pressure.gauge_vs_fill.y = {20.0, 65.5};
    for (double kg : {6.804, 3.0}) { // flow retention 0.885 (Near) and below 0.80 (Limiting)
        DynoNitrous n = vs;
        n.bottle.initial_kg = kg;
        out += sig_limits(sweep_n2o(n, {5000.0}, p));
    }
    return out;
}

// Probe registry: id -> function. `cheap` probes run in LP2a, the rest in LP2b.
struct ProbeDef {
    const char* id;
    bool cheap;
    Probe fn;
};
const std::vector<ProbeDef>& probes() {
    static const std::vector<ProbeDef> v = {
        {"rules", true, probe_rules},   {"driveline", true, probe_driveline}, {"t16", false, probe_t16},       {"t20", false, probe_t20},
        {"t110", false, probe_t110},    {"rotor", false, probe_rotor},        {"surge", false, probe_surge},   {"solve", false, probe_solve},
        {"limiter", false, probe_limiter}, {"piston", false, probe_piston},   {"edge", false, probe_edge},     {"knock", false, probe_knock},
        {"kit", false, probe_kit},      {"bottle", false, probe_bottle},
    };
    return v;
}
const ProbeDef& probe_def(const std::string& id) {
    for (const ProbeDef& d : probes())
        if (id == d.id) return d;
    FAIL("no probe " << id);
    return probes()[0];
}

// Which probes read which entry (every one of the 58 must appear; LP2c checks the list is complete and has no stranger).
const std::map<std::string, std::vector<std::string>>& entry_probes() {
    static const std::map<std::string, std::vector<std::string>> m = {
        // turbo
        {"gate_near_opening", {"t16"}},
        {"gate_full_opening", {"t20"}},
        {"gate_saturated_window_fraction", {"t20"}},
        {"boost_band_near", {"t20"}},
        {"boost_band_exceeded", {"t20"}},
        {"rotor_near_fraction", {"rotor"}},
        {"rotor_limiting_fraction", {"rotor"}},
        {"surge_near_flow_ratio", {"surge"}},
        {"surge_min_window_fraction", {"surge"}},
        {"choke_near_flow_fraction", {"t110"}},
        {"choke_limiting_flow_fraction", {"t16"}},
        {"wastegate_remedy_margin", {"solve"}},
        // limiter, piston speed, grid
        {"limiter_near_fraction", {"limiter"}},
        {"soft_cut_near_bands", {"limiter"}},
        {"piston_speed_near_mps", {"piston"}}, // the build rule starts at Limiting (check_mean_piston_speed), only the dyno has Near
        {"piston_speed_limiting_mps", {"rules", "piston"}},
        {"piston_speed_exceeded_mps", {"rules", "piston"}},
        {"grid_edge_min_window_fraction", {"edge"}},
        {"grid_edge_near_span_fraction", {"edge"}},
        // driveline
        {"driveline_torque_margin", {"driveline"}},
        // nitrous
        {"kit_lambda_near", {"kit"}},
        {"kit_lambda_limiting", {"kit"}},
        {"kit_lambda_exceeded", {"kit"}},
        {"kit_lambda_target", {"kit"}},
        {"bottle_duration_target_s", {"bottle"}},
        {"bottle_duration_near_factor", {"bottle"}},
        {"bottle_reserve_factor", {"bottle"}},
        {"bottle_flow_retention_near", {"bottle"}},
        {"bottle_flow_retention_limiting", {"bottle"}},
        // knock
        {"knock_near_retard_deg", {"knock"}},
        {"knock_limiting_retard_deg", {"knock"}},
        // build rules
        {"intake_d_over_bore_1v_min", {"rules"}},
        {"intake_d_over_bore_1v_max", {"rules"}},
        {"intake_d_over_bore_2v_min", {"rules"}},
        {"intake_d_over_bore_2v_max", {"rules"}},
        {"intake_d_over_bore_3v_min", {"rules"}},
        {"intake_d_over_bore_3v_max", {"rules"}},
        {"exhaust_over_intake_min", {"rules"}},
        {"exhaust_over_intake_max", {"rules"}},
        {"pack_share_of_ideal", {"rules"}},
        {"rod_over_crank_radius_min", {"rules"}},
        {"rod_over_crank_radius_max", {"rules"}},
        {"cr_na_min", {"rules"}},
        {"cr_na_max", {"rules"}},
        {"cr_forced_min", {"rules"}},
        {"cr_forced_max", {"rules"}},
        {"grid_overshoot_limiting", {"rules"}},
        {"grid_overshoot_exceeded", {"rules"}},
        {"cam_switch_over_limiter_min", {"rules"}},
        {"cam_switch_over_limiter_max", {"rules"}},
        {"cam_switch_hysteresis_min_rpm", {"rules"}},
        {"cam_switch_hysteresis_max_rpm", {"rules"}},
        {"cam_peak_lift_over_valve_d_max", {"rules"}},
        {"lift_min_node_peak_min_mm", {"rules"}},
        {"lift_min_node_peak_max_fraction_of_full", {"rules"}},
        // gearbox rating
        {"gearbox_rating_over_limiting", {"driveline"}},
        {"gearbox_rating_over_exceeded", {"driveline"}},
        {"gearbox_rating_remedy_margin", {"driveline"}},
    };
    return m;
}

// The default-policy signature of a probe, cached.
const std::string& baseline(const ProbeDef& d) {
    static std::map<std::string, std::string> cache;
    auto it = cache.find(d.id);
    if (it == cache.end()) it = cache.emplace(d.id, d.fn(nullptr)).first;
    return it->second;
}

// ---------------------------------------------------------------------------------------------------------------------
// Temp files.

const std::filesystem::path& temp_dir() {
    static const std::filesystem::path p = [] {
        std::random_device rd;
        const std::filesystem::path d = std::filesystem::temp_directory_path() / ("ps_limit_policy_" + std::to_string(rd()) + std::to_string(rd()));
        std::filesystem::create_directories(d);
        return d;
    }();
    return p;
}
std::string write_file(const std::string& name, const std::string& body) {
    const std::filesystem::path f = temp_dir() / name;
    std::ofstream o(f);
    o << body;
    return f.string();
}
std::string base_extends() { return "\"extends\": \"" + kDefaultJson + "\""; }
const std::string kFormat = "\"format\": \"physics_sim.limit_policy/1\"";

} // namespace

// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("LP1 default.json equals the compiled defaults entry by entry; the NOT-POLICY entries are in no file", "[limit_policy][drivetrain][io]") {
    // Recorded sabotages: see the RUN lines at the end of this comment.
    //  - a default drifted in default.json (turbo.gate_near_opening 0.92 -> 0.93): the value check names the entry;
    //  - a source text edited: the source check names the entry;
    //  - a row added to limit_policy_table.h without a LimitPolicy field (or the reverse): limit_policy.cpp's static_assert.
    std::ifstream in(kDefaultJson);
    REQUIRE(in.good());
    const nlohmann::json doc = nlohmann::json::parse(in);
    REQUIRE(doc.is_object());
    CHECK(doc["format"] == "physics_sim.limit_policy/1");
    CHECK(doc["refuse_at"] == "exceeded");
    CHECK(doc["warn_at"] == "near");
    CHECK(doc.contains("source"));

    // The groups of the file are exactly the groups of the table; every group holds exactly its rows.
    std::set<std::string> groups;
    for (const pt::Row& r : pt::kRows) groups.insert(r.group);
    std::set<std::string> file_groups;
    for (auto it = doc.begin(); it != doc.end(); ++it) {
        if (it.key() == "format" || it.key() == "source" || it.key() == "refuse_at" || it.key() == "warn_at" || it.key() == "extends") continue;
        file_groups.insert(it.key());
    }
    CHECK(file_groups == groups);

    CHECK(pt::kRowCount == 58);
    std::size_t entries_in_file = 0;
    for (const auto& g : groups) entries_in_file += doc[g].size();
    CHECK(entries_in_file == pt::kRowCount);

    std::set<std::string> names;
    for (const pt::Row& r : pt::kRows) {
        INFO("entry " << r.group << "/" << r.name);
        CHECK(names.insert(r.name).second);
        REQUIRE(doc[r.group].contains(r.name));
        const nlohmann::json& e = doc[r.group][r.name];
        REQUIRE(e.is_object());
        CHECK(e["value"].get<double>() == r.def);   // exact: both are the same decimal literal
        CHECK(e["source"].get<std::string>() == r.source);
        const std::string src = r.source;
        CHECK((src.rfind("[C]", 0) == 0 || src.rfind("[D]", 0) == 0 || src.rfind("[AS]", 0) == 0 || src.rfind("[M]", 0) == 0));
        // The row's default is the table header's constant, found by name.
        int found = 0;
        for (std::size_t i = 0; i < limits::kAllLimitThresholdCount; ++i) {
            const limits::Threshold& t = limits::kAllLimitThresholds[i];
            if (std::string(t.name) != r.name) continue;
            ++found;
            CHECK(t.value == r.def);
            CHECK(std::string(t.source) == r.source);
        }
        for (std::size_t i = 0; i < rules::kAllEngineRuleThresholdCount; ++i) {
            const rules::Threshold& t = rules::kAllEngineRuleThresholds[i];
            if (std::string(t.name) != r.name) continue;
            ++found;
            CHECK(t.value == r.def);
            CHECK(std::string(t.source) == r.source);
        }
        CHECK(found == 1);
    }

    // The loaded file is the compiled default, field by field.
    const LimitPolicy loaded = io::load_limit_policy_json(kDefaultJson);
    const LimitPolicy& def = default_limit_policy();
    CHECK(loaded.refuse_at == def.refuse_at);
    CHECK(loaded.warn_at == def.warn_at);
    for (const pt::Row& r : pt::kRows) {
        INFO("entry " << r.name);
        CHECK(loaded.*r.member == def.*r.member);
        CHECK(def.*r.member == r.def);
    }
    CHECK(def.refuse_at == RefuseAt::Exceeded);
    CHECK(def.warn_at == WarnAt::Near);
    CHECK(validate_limit_policy(def).empty());

    // Classification: 71 table entries = 58 policy + 13 NOT policy; the 13 are named, and no file or field has them.
    CHECK(limits::kAllLimitThresholdCount == 31);
    CHECK(rules::kAllEngineRuleThresholdCount == 40);
    const std::set<std::string> not_policy = {"valve_layouts",
                                              "valve_row_chord_fraction",
                                              "pack_density_n2",
                                              "pack_density_n3",
                                              "pack_density_n4",
                                              "pack_density_n5",
                                              "boost_grid_margin_bar",
                                              "boost_grid_ambient_bar",
                                              "boost_grid_step_pa",
                                              "boost_grid_min_last_step_pa",
                                              "boost_grid_ambient_node_gap_pa",
                                              "na_grid_top_fraction_of_ambient",
                                              "relative_tolerance"};
    CHECK(not_policy.size() == 13);
    CHECK(limits::kAllLimitThresholdCount + rules::kAllEngineRuleThresholdCount == pt::kRowCount + not_policy.size());
    std::set<std::string> table_names;
    for (std::size_t i = 0; i < limits::kAllLimitThresholdCount; ++i) table_names.insert(limits::kAllLimitThresholds[i].name);
    for (std::size_t i = 0; i < rules::kAllEngineRuleThresholdCount; ++i) table_names.insert(rules::kAllEngineRuleThresholds[i].name);
    CHECK(table_names.size() == 71);
    for (const std::string& n : table_names) {
        INFO("table entry " << n);
        CHECK((names.count(n) == 1) != (not_policy.count(n) == 1)); // exactly one of the two sets
    }
    for (const std::string& n : not_policy) {
        INFO("not policy " << n);
        CHECK(names.count(n) == 0);
        CHECK(doc.dump().find("\"" + n + "\"") == std::string::npos);
    }

    // The orderings only name fields that have a row.
    for (const pt::Order& o : pt::kOrders) {
        bool lo = false, hi = false;
        for (const pt::Row& r : pt::kRows) {
            lo = lo || r.member == o.lo;
            hi = hi || r.member == o.hi;
        }
        CHECK(lo);
        CHECK(hi);
    }
}

TEST_CASE("LP2a every build-rule, piston-speed, driveline and gearbox entry moves a consumer result", "[limit_policy][drivetrain][io]") {
    // Table-driven: each entry is set to 1e-9 and to 1e9 (bypassing validation: consumers accept anything) and the probes that
    // read it must change at least once. A consumer that ignores the policy pointer, or an entry no consumer reads, fails here
    // naming the entry. Probes given nullptr, the address of the compiled default and a default-constructed policy are
    // byte-identical (the "no behaviour change" half).
    // RUN sabotage: check_buildable's Sink built on default_limit_policy() instead of the argument fails every "rules" entry.
    for (const ProbeDef& d : probes()) {
        if (!d.cheap) continue;
        const std::string& base = baseline(d);
        CHECK_FALSE(base.empty());
        const LimitPolicy dflt;
        CHECK(d.fn(&default_limit_policy()) == base);
        CHECK(d.fn(&dflt) == base);
    }
    std::size_t checked = 0;
    for (const pt::Row& r : pt::kRows) {
        const std::vector<std::string>& ids = entry_probes().at(r.name);
        std::vector<const ProbeDef*> cheap;
        for (const std::string& id : ids)
            if (probe_def(id).cheap) cheap.push_back(&probe_def(id));
        if (cheap.empty()) continue;
        bool moved = false;
        for (double ext : {1e-9, 1e9}) {
            const LimitPolicy p = with(r.name, ext);
            for (const ProbeDef* d : cheap) moved = moved || d->fn(&p) != baseline(*d);
        }
        INFO("entry " << r.group << "/" << r.name << " is read by no cheap consumer probe");
        CHECK(moved);
        ++checked;
    }
    CHECK(checked >= 30);
}

TEST_CASE("LP2b every dyno entry moves a sweep finding or remedy", "[limit_policy][drivetrain][io][dyno][turbo][long]") {
    // Same table-driven check for the entries read by run_dyno_sweep / solve_remedy (hyper sweeps with Sketch maps, the A25A
    // limiter / piston / edge / knock cases, the N7b kit and bottle cases, one wastegate solve). The fixtures are those of
    // test_limit_findings.cpp and test_limit_remedies.cpp; the scenario for every severity ladder is in the probe functions.
    // RUN sabotage: piston_speed_finding reading limits::kPistonSpeedNear.value instead of pol.piston_speed_near_mps fails the
    // "piston_speed_near_mps" entry in the dyno probe (the rules probe still moves).
    for (const ProbeDef& d : probes()) {
        if (d.cheap) continue;
        const std::string& base = baseline(d);
        INFO("probe " << d.id);
        CHECK_FALSE(base.empty());
        const LimitPolicy dflt;
        CHECK(d.fn(&default_limit_policy()) == base);
        CHECK(d.fn(&dflt) == base);
    }
    std::size_t checked = 0;
    for (const pt::Row& r : pt::kRows) {
        std::vector<const ProbeDef*> dyno;
        for (const std::string& id : entry_probes().at(r.name))
            if (!probe_def(id).cheap) dyno.push_back(&probe_def(id));
        if (dyno.empty()) continue;
        bool moved = false;
        for (double ext : {1e-9, 1e9}) {
            const LimitPolicy p = with(r.name, ext);
            for (const ProbeDef* d : dyno) moved = moved || d->fn(&p) != baseline(*d);
        }
        INFO("entry " << r.group << "/" << r.name << " is read by no dyno consumer probe");
        CHECK(moved);
        ++checked;
    }
    CHECK(checked >= 28);
}

TEST_CASE("LP2c the probe list covers exactly the 58 entries", "[limit_policy][drivetrain][io]") {
    // A new entry needs a probe: the list below must hold every row, and no name that is not a row.
    std::set<std::string> rows;
    for (const pt::Row& r : pt::kRows) rows.insert(r.name);
    std::set<std::string> listed;
    for (const auto& [name, ids] : entry_probes()) {
        listed.insert(name);
        CHECK_FALSE(ids.empty());
        for (const std::string& id : ids) {
            bool known = false;
            for (const ProbeDef& d : probes()) known = known || id == d.id;
            INFO(name << " -> " << id);
            CHECK(known);
        }
    }
    CHECK(listed == rows);
    CHECK(rows.size() == 58);
}

TEST_CASE("LP3 refuse_at / warn_at: severity matrices, blocking issues, refusing findings; findings never depend on them",
          "[limit_policy][drivetrain][io]") {
    // Recorded sabotage: LimitPolicy::refuses ignoring refuse_at (always Exceeded) fails the Limiting/Never rows; has_blocking_issue
    // ignoring its policy argument fails the gearbox rows; a sweep that dropped findings below warn_at fails the identity block.
    const LimitSeverity sevs[] = {LimitSeverity::Near, LimitSeverity::Limiting, LimitSeverity::Exceeded};
    const struct {
        RefuseAt at;
        bool expect[3];
    } refuse[] = {{RefuseAt::Exceeded, {false, false, true}}, {RefuseAt::Limiting, {false, true, true}}, {RefuseAt::Never, {false, false, false}}};
    for (const auto& rf : refuse) {
        LimitPolicy p;
        p.refuse_at = rf.at;
        for (int i = 0; i < 3; ++i) {
            INFO("refuse_at " << to_string(rf.at) << " severity " << to_string(sevs[i]));
            CHECK(p.refuses(sevs[i]) == rf.expect[i]);
        }
    }
    const struct {
        WarnAt at;
        bool expect[3];
    } warn[] = {{WarnAt::Near, {true, true, true}},
                {WarnAt::Limiting, {false, true, true}},
                {WarnAt::Exceeded, {false, false, true}},
                {WarnAt::Never, {false, false, false}}};
    for (const auto& w : warn) {
        LimitPolicy p;
        p.warn_at = w.at;
        for (int i = 0; i < 3; ++i) {
            INFO("warn_at " << to_string(w.at) << " severity " << to_string(sevs[i]));
            CHECK(p.warns(sevs[i]) == w.expect[i]);
        }
    }
    CHECK(std::string(to_string(RefuseAt::Never)) == "never");
    CHECK(std::string(to_string(WarnAt::Exceeded)) == "exceeded");

    // The default is the owner's rule: warn from Near, refuse only Exceeded.
    CHECK(default_limit_policy().refuses(LimitSeverity::Exceeded));
    CHECK_FALSE(default_limit_policy().refuses(LimitSeverity::Limiting));
    CHECK(default_limit_policy().warns(LimitSeverity::Near));

    // has_refusing_finding
    std::vector<LimitFinding> fs(2);
    fs[0].severity = LimitSeverity::Near;
    fs[1].severity = LimitSeverity::Limiting;
    CHECK_FALSE(has_refusing_finding(fs));
    CHECK_FALSE(has_refusing_finding(fs, &default_limit_policy()));
    LimitPolicy at_limiting;
    at_limiting.refuse_at = RefuseAt::Limiting;
    CHECK(has_refusing_finding(fs, &at_limiting));
    fs.push_back(LimitFinding{});
    fs.back().severity = LimitSeverity::Exceeded;
    CHECK(has_refusing_finding(fs));
    LimitPolicy never;
    never.refuse_at = RefuseAt::Never;
    CHECK_FALSE(has_refusing_finding(fs, &never));
    CHECK_FALSE(has_refusing_finding({}));

    // has_blocking_issue and the gearbox message: 1100/1000 = +10 % = Limiting, 1200 = +20 % = Exceeded.
    const GearboxDesc box = rated_box(1000.0);
    const auto limiting = check_gearbox_rating(box, 1100.0);
    const auto exceeded = check_gearbox_rating(box, 1200.0);
    REQUIRE(limiting.size() == 1);
    REQUIRE(exceeded.size() == 1);
    CHECK(limiting[0].severity == LimitSeverity::Limiting);
    CHECK(exceeded[0].severity == LimitSeverity::Exceeded);
    CHECK_FALSE(has_blocking_issue(limiting));
    CHECK(has_blocking_issue(exceeded));
    CHECK_FALSE(has_blocking_issue(limiting, &default_limit_policy()));
    CHECK(has_blocking_issue(limiting, &at_limiting));
    CHECK_FALSE(has_blocking_issue(exceeded, &never));
    CHECK_THAT(exceeded[0].message, ContainsSubstring("refused"));
    const auto exceeded_never = check_gearbox_rating(box, 1200.0, &never);
    REQUIRE(exceeded_never.size() == 1);
    CHECK(exceeded_never[0].severity == LimitSeverity::Exceeded); // the grade is not the policy's refusal rule
    CHECK_THAT(exceeded_never[0].message, !ContainsSubstring("refused"));

    // Findings are complete whatever the refusal / warning rule: the same sweep under every combination is the same text.
    DynoSweepSpec s;
    s.rpm = {5000.0, 6200.0, 6400.0, 6500.0, 6600.0};
    const std::string reference = sig(run_dyno_sweep(a25a(), nullptr, s, nullptr));
    CHECK_FALSE(reference.empty());
    for (RefuseAt r : {RefuseAt::Exceeded, RefuseAt::Limiting, RefuseAt::Never})
        for (WarnAt w : {WarnAt::Near, WarnAt::Limiting, WarnAt::Exceeded, WarnAt::Never}) {
            LimitPolicy p;
            p.refuse_at = r;
            p.warn_at = w;
            s.policy = &p;
            INFO("refuse_at " << to_string(r) << " warn_at " << to_string(w));
            CHECK(sig(run_dyno_sweep(a25a(), nullptr, s, nullptr)) == reference);
        }
    // Build issues likewise.
    const std::string rules_ref = sig(check_buildable(demo_i4()));
    CHECK_FALSE(rules_ref.empty()); // the shipped pipe I4 has its Near limiter finding
    LimitPolicy hush;
    hush.refuse_at = RefuseAt::Never;
    hush.warn_at = WarnAt::Never;
    CHECK(sig(check_buildable(demo_i4(), &hush)) == rules_ref);
}

TEST_CASE("LP4 extends chains, the user layer, partial files, null deletes, both value forms", "[limit_policy][drivetrain][io]") {
    // Recorded sabotage: merging the user layer BEFORE the extends chain (the chain wins) fails the precedence block; the
    // loader starting from zeros instead of the compiled defaults fails the partial-file block.
    const LimitPolicy dflt;

    // A partial file without extends: format + one entry; everything else is the compiled default.
    {
        const std::string f = write_file("partial.json", "{" + kFormat + ", \"turbo\": {\"gate_near_opening\": 0.5}}");
        const LimitPolicy p = io::load_limit_policy_json(f);
        CHECK(p.gate_near_opening == 0.5);
        for (const pt::Row& r : pt::kRows) {
            if (std::string(r.name) == "gate_near_opening") continue;
            INFO(r.name);
            CHECK(p.*r.member == dflt.*r.member);
        }
        CHECK(p.refuse_at == RefuseAt::Exceeded);
        CHECK(p.warn_at == WarnAt::Near);
    }
    // Both value forms; refuse_at / warn_at.
    {
        const std::string f = write_file(
            "forms.json", "{" + kFormat + ", \"refuse_at\": \"limiting\", \"warn_at\": \"exceeded\", \"turbo\": {\"gate_near_opening\": {\"value\": 0.6, \"source\": \"[AS] test\"}, "
                          "\"rotor_near_fraction\": 0.8}}");
        const LimitPolicy p = io::load_limit_policy_json(f);
        CHECK(p.gate_near_opening == 0.6);
        CHECK(p.rotor_near_fraction == 0.8);
        CHECK(p.refuse_at == RefuseAt::Limiting);
        CHECK(p.warn_at == WarnAt::Exceeded);
    }
    // extends the shipped default: the explicit entries win, the rest is the default file's.
    {
        const std::string f = write_file("derived.json", "{" + kFormat + ", " + base_extends() + ", \"knock\": {\"knock_near_retard_deg\": 0.5}}");
        const LimitPolicy p = io::load_limit_policy_json(f);
        CHECK(p.knock_near_retard_deg == 0.5);
        CHECK(p.knock_limiting_retard_deg == dflt.knock_limiting_retard_deg);
    }
    // A three-level chain: c overrides b overrides a; a's other entries survive; user layer last.
    {
        const std::string a = write_file("a.json", "{" + kFormat + ", " + base_extends() + ", \"turbo\": {\"gate_near_opening\": 0.50, \"rotor_near_fraction\": 0.50, \"boost_band_near\": 0.01}}");
        const std::string b = write_file("b.json", "{" + kFormat + ", \"extends\": \"a.json\", \"turbo\": {\"gate_near_opening\": 0.60, \"rotor_near_fraction\": 0.60}}");
        const std::string c = write_file("c.json", "{" + kFormat + ", \"extends\": \"b.json\", \"turbo\": {\"gate_near_opening\": 0.70}}");
        const LimitPolicy chain = io::load_limit_policy_json(c);
        CHECK(chain.gate_near_opening == 0.70);
        CHECK(chain.rotor_near_fraction == 0.60);
        CHECK(chain.boost_band_near == 0.01);
        CHECK(chain.rotor_limiting_fraction == dflt.rotor_limiting_fraction);
        // user layer: wins over the whole chain, a bare number or an object; refuse_at too
        const std::string u = write_file("user.json", "{\"turbo\": {\"gate_near_opening\": 0.75, \"boost_band_near\": {\"value\": 0.02, \"source\": \"[AS] user\"}}, \"refuse_at\": \"never\"}");
        const LimitPolicy with_user = io::load_limit_policy_json(c, u);
        CHECK(with_user.gate_near_opening == 0.75);
        CHECK(with_user.boost_band_near == 0.02);
        CHECK(with_user.rotor_near_fraction == 0.60);
        CHECK(with_user.refuse_at == RefuseAt::Never);
        // a user layer null deletes the entry from the merge: the COMPILED default applies, not the chain's value
        const std::string un = write_file("user_null.json", "{\"turbo\": {\"gate_near_opening\": null}}");
        const LimitPolicy deleted = io::load_limit_policy_json(c, un);
        CHECK(deleted.gate_near_opening == dflt.gate_near_opening);
        CHECK(deleted.rotor_near_fraction == 0.60);
        // a user layer may extend a file of its own
        const std::string ux = write_file("user_ext.json", "{\"extends\": \"a.json\", \"turbo\": {\"rotor_near_fraction\": 0.55}}");
        const LimitPolicy user_ext = io::load_limit_policy_json(c, ux);
        CHECK(user_ext.rotor_near_fraction == 0.55);
        CHECK(user_ext.gate_near_opening == 0.50); // the user layer's own chain (a.json) overrides c's 0.70 as a layer
    }
    // From text: same loader.
    {
        const LimitPolicy p = io::load_limit_policy_json_from_string("{" + kFormat + ", \"piston_speed\": {\"piston_speed_near_mps\": 20}}", "text.json",
                                                                      "{\"piston_speed\": {\"piston_speed_near_mps\": 21}}");
        CHECK(p.piston_speed_near_mps == 21.0);
    }
    // The shipped default through every door is the compiled default.
    {
        const LimitPolicy p = io::load_limit_policy_json(kDefaultJson, write_file("user_empty.json", "{}"));
        for (const pt::Row& r : pt::kRows) CHECK(p.*r.member == dflt.*r.member);
    }
}

TEST_CASE("LP5 loader rejections name the file and the key", "[limit_policy][drivetrain][io]") {
    // Recorded sabotage: read_object skipping the unknown-key check fails the "unknown key" rows; the domain check skipped in
    // build() fails the "must be" rows.
    const auto rejects = [&](const std::string& text, const std::string& needle, const std::string& user = {}) {
        INFO(text << " | " << user);
        CHECK_THROWS_WITH(io::load_limit_policy_json_from_string(text, "limit_policy.json", user), ContainsSubstring(needle));
    };
    const std::string F = "{" + kFormat + ", ";
    rejects("not json", "limit_policy.json");
    rejects("[1,2]", "top level must be a JSON object");
    rejects("{}", "format: is required");
    rejects("{\"format\": \"physics_sim.limit_policy/2\"}", "format: must be");
    rejects("{\"format\": 3}", "format: must be");
    rejects(F + "\"bogus\": 1}", "bogus: unknown key");
    rejects(F + "\"turbo\": {\"gate_bogus\": 1}}", "turbo/gate_bogus: unknown key");
    rejects(F + "\"turbo\": {\"rod_over_crank_radius_min\": 3}}", "turbo/rod_over_crank_radius_min: unknown key"); // right name, wrong group
    rejects(F + "\"turbo\": 5}", "turbo: must be an object");
    rejects(F + "\"turbo\": [1]}", "turbo: must be an object");
    rejects(F + "\"turbo\": {\"gate_near_opening\": \"0.9\"}}", "turbo/gate_near_opening: must be a number");
    rejects(F + "\"turbo\": {\"gate_near_opening\": true}}", "turbo/gate_near_opening: must be a number");
    rejects(F + "\"turbo\": {\"gate_near_opening\": {\"source\": \"[AS] x\"}}}", "turbo/gate_near_opening: the entry has no \"value\"");
    rejects(F + "\"turbo\": {\"gate_near_opening\": {\"value\": 0.9, \"extra\": 1}}}", "turbo/gate_near_opening/extra: unknown key");
    rejects(F + "\"turbo\": {\"gate_near_opening\": {\"value\": 0.9, \"source\": 7}}}", "turbo/gate_near_opening/source: must be a string");
    rejects(F + "\"turbo\": {\"gate_near_opening\": {\"value\": 0.9, \"source\": \"guess\"}}}", "turbo/gate_near_opening/source: must start with [C]");
    rejects(F + "\"source\": 3}", "source: must be a string");
    rejects(F + "\"refuse_at\": \"always\"}", "refuse_at: \"always\" is not one of");
    rejects(F + "\"refuse_at\": 1}", "refuse_at: must be a string");
    rejects(F + "\"warn_at\": \"loud\"}", "warn_at: \"loud\" is not one of");
    rejects(F + "\"warn_at\": false}", "warn_at: must be a string");
    // domains
    rejects(F + "\"turbo\": {\"gate_near_opening\": 1.5}}", "turbo/gate_near_opening: must be <= 1");
    rejects(F + "\"turbo\": {\"gate_near_opening\": 0}}", "turbo/gate_near_opening: must be > 0");
    rejects(F + "\"turbo\": {\"gate_near_opening\": -0.1}}", "turbo/gate_near_opening: must be > 0");
    rejects(F + "\"turbo\": {\"wastegate_remedy_margin\": 0.9}}", "turbo/wastegate_remedy_margin: must be >= 1");
    rejects(F + "\"compression\": {\"cr_na_min\": 0.5}}", "compression/cr_na_min: must be >= 1");
    rejects(F + "\"gearbox\": {\"gearbox_rating_remedy_margin\": 0.99}}", "gearbox/gearbox_rating_remedy_margin: must be >= 1");
    // orderings
    rejects(F + "\"turbo\": {\"gate_near_opening\": 0.99}}", "turbo/gate_near_opening (0.99) must be below turbo/gate_full_opening");
    rejects(F + "\"piston_speed\": {\"piston_speed_near_mps\": 26}}", "piston_speed/piston_speed_near_mps (26) must be below piston_speed/piston_speed_limiting_mps");
    rejects(F + "\"nitrous\": {\"kit_lambda_target\": 0.99}}", "nitrous/kit_lambda_target (0.99) must be at most nitrous/kit_lambda_near");
    rejects(F + "\"gearbox\": {\"gearbox_rating_over_limiting\": 0.2}}", "gearbox/gearbox_rating_over_limiting (0.2) must be below gearbox/gearbox_rating_over_exceeded");
    // user layer
    rejects(F + "\"turbo\": {}}", "user layer: bogus: unknown key", "{\"bogus\": 1}");
    rejects(F + "\"turbo\": {}}", "user layer: turbo/gate_near_opening: must be a number", "{\"turbo\": {\"gate_near_opening\": \"x\"}}");
    rejects(F + "\"turbo\": {}}", "user layer: not valid JSON", "{broken");
    rejects(F + "\"turbo\": {}}", "gate_near_opening (0.99) must be below", "{\"turbo\": {\"gate_near_opening\": 0.99}}"); // the merged result is validated
    // files
    CHECK_THROWS_WITH(io::load_limit_policy_json((temp_dir() / "does_not_exist.json").string()), ContainsSubstring("does_not_exist.json"));
    const std::string bad = write_file("bad_user.json", "{\"nonsense\": 1}");
    CHECK_THROWS_WITH(io::load_limit_policy_json(kDefaultJson, bad), ContainsSubstring("bad_user.json: nonsense: unknown key"));
    const std::string cyc_a = write_file("cyc_a.json", "{" + kFormat + ", \"extends\": \"cyc_b.json\"}");
    write_file("cyc_b.json", "{" + kFormat + ", \"extends\": \"cyc_a.json\"}");
    CHECK_THROWS(io::load_limit_policy_json(cyc_a));
    // validate_limit_policy on code-built policies (the loader's own check, usable by tools)
    LimitPolicy ok;
    CHECK(validate_limit_policy(ok).empty());
    LimitPolicy nan_p;
    nan_p.gate_near_opening = std::nan("");
    CHECK_THAT(validate_limit_policy(nan_p), ContainsSubstring("turbo/gate_near_opening: must be a finite number"));
    LimitPolicy inf_p;
    inf_p.piston_speed_exceeded_mps = std::numeric_limits<double>::infinity();
    CHECK_THAT(validate_limit_policy(inf_p), ContainsSubstring("piston_speed/piston_speed_exceeded_mps: must be a finite number"));
}

TEST_CASE("LP6 a policy never moves a map key, a turbo key, a dyno point or a World state hash", "[limit_policy][drivetrain][io][determinism]") {
    // The NOT-POLICY half of the classification: nothing a generator or loader BUILDS from is configurable. Recorded sabotage
    // ("a NOT-POLICY entry made configurable"): stepped_point's settle criterion reading pol.boost_band_near moves DynoResult::points
    // under the extreme policy and fails the identity block; a World hash cannot move through the policy because the World takes
    // none, so the hash block guards the API (a consumer smuggling a policy into the vehicle step would show here).
    const LimitPolicy ex = extreme_policy();
    REQUIRE_FALSE(validate_limit_policy(ex).empty()); // really not a sensible policy

    // Keys.
    const SimulatedEngineDesc& e = S(demo_i4());
    CycleModelDesc cy = *e.cycle;
    if (cy.intake.has_value()) cy.intake->compressor_supply = e.turbo_pair.has_value();
    const std::uint64_t map_key = engine_map_cache_key(cy, e.fuels.front(), MapQuality::Full);
    const TurboPairDesc& tp = *S(hyper_vo()).turbo_pair;
    const std::uint64_t turbo_key = turbo_table_key(tp);
    (void)check_buildable(demo_i4(), &ex);
    (void)check_buildable(hyper_vo(), &ex);
    CHECK(engine_map_cache_key(cy, e.fuels.front(), MapQuality::Full) == map_key);
    CHECK(turbo_table_key(tp) == turbo_key);
    // The grid helpers are build-time; they take no policy and give the same grids.
    const CycleModelDesc& hcy = *S(hyper_vo()).cycle;
    const std::vector<real> g0 = boost_grid_p_im(hcy, 1.8);
    (void)check_buildable(hyper_vo(), &ex);
    CHECK(boost_grid_p_im(hcy, 1.8) == g0);

    // Dyno points: physics identical under the extreme policy (findings may differ, points may not).
    {
        DynoSweepSpec s;
        s.rpm = {3000.0, 5000.0, 6400.0, 6600.0};
        const DynoResult a = run_dyno_sweep(a25a(), nullptr, s, nullptr);
        s.policy = &ex;
        const DynoResult b = run_dyno_sweep(a25a(), nullptr, s, nullptr);
        REQUIRE(a.points.size() == b.points.size());
        for (std::size_t i = 0; i < a.points.size(); ++i) CHECK(sig(a.points[i]) == sig(b.points[i]));
    }
    {
        const DynoResult a = sweep_hyper(1.6, 1.0, {5000.0, 7000.0}, nullptr);
        const DynoResult b = sweep_hyper(1.6, 1.0, {5000.0, 7000.0}, &ex);
        REQUIRE(a.points.size() == b.points.size());
        for (std::size_t i = 0; i < a.points.size(); ++i) {
            INFO("hyper point " << i);
            CHECK(sig(a.points[i]) == sig(b.points[i]));
        }
        CHECK(sig(a.limits) != sig(b.limits)); // the policy did reach the findings
    }

    // World state hash at 1 and 10 workers: consumers run between the ticks of the second run.
    const auto run = [&](bool consumers, int workers) {
        vehicle::VehicleDesc desc = io::load_vehicle_json(kVehicles + "car_sedan.json");
        auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
        World& w = *rig.world;
        ps_test::settle(w, 0.5);
        w.set_control("throttle", 1.0);
        w.set_control("gear_request", 1.0);
        for (int i = 0; i < 480; ++i) {
            if (consumers && i % 60 == 0) {
                (void)check_buildable(demo_i4(), &ex);
                (void)check_gearbox_rating(desc.powertrain, 240.0, &ex);
                DynoSweepSpec s;
                s.rpm = {4000.0};
                s.policy = &ex;
                (void)run_dyno_sweep(a25a(), nullptr, s, nullptr);
            }
            w.step();
        }
        return w.state_hash();
    };
    const std::uint64_t h1 = run(false, 1);
    CHECK(h1 == run(true, 1));
    CHECK(h1 == run(true, 10));
    CHECK(h1 == run(false, 10));
}
