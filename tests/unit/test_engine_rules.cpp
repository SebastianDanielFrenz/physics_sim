// tests/unit/test_engine_rules.cpp - engine garage P4 (racing_game out/engine_garage_design.md A10-A12):
//   A10 check_buildable      (ps/drivetrain/engine_rules.h; numbers in core/src/drivetrain/engine_rules_table.h)
//   A11 classify_engine_change / list_engine_changes / turbo_table_key
//   A12 boost_grid_p_im / ambient_grid_p_im / extend_grid_rpm
//
// Every guard below lists its recorded sabotage in its case comment: the production line was changed, the case failed with
// the quoted symptom, and the line was reverted (the commit that adds this file contains the reverted code).
//
// Loading: engines are read with EngineMapOptions::generate = false (validate-only, instant, no generator); the only cases
// that generate maps (G1-G3) use Sketch quality on a pool of 10 workers and a per-process temp cache directory.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "drivetrain/engine_rules_table.h"
#include "drivetrain/limit_thresholds.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/turbo_maps.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <set>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {

const std::string kEngines = std::string(PS_SOURCE_DIR) + "/data/engines/";
const std::string kTurboConfigs = std::string(PS_SOURCE_DIR) + "/data/turbo_configurations/";

io::EngineMapOptions validate_only() {
    io::EngineMapOptions o;
    o.generate = false;
    return o;
}
ComponentParams load_vo(const std::string& file) { return io::load_engine_json(kEngines + file, validate_only()); }

const ComponentParams& hyper() {
    static const ComponentParams c = load_vo("hyper_v8_one1_audio.json");
    return c;
}
// 2026-10-18: the shipped hyper is knock-limited (lambda target and fuel octane are map inputs for it). The classification
// rows for lambda_target / fuels[1] were recorded on the schedule-only engine, so they run on the hyper with the knock block off.
const ComponentParams& hyper_schedule() {
    static const ComponentParams c = [] {
        ComponentParams p = load_vo("hyper_v8_one1_audio.json");
        auto& e = std::get<SimulatedEngineDesc>(p);
        e.cycle->knock = {};
        return p;
    }();
    return c;
}
const ComponentParams& demo_i4() {
    static const ComponentParams c = load_vo("demo_i4_pipes.json");
    return c;
}
const ComponentParams& a25a_gen() {
    // 2026-10-18: the shipped A25A gen is knock-limited; the classification tables below were recorded on the schedule-only engine.
    static const ComponentParams c = io::load_engine_json_from_string(
        R"({"extends": "toyota_a25a_fks_gen.json", "ecu": {"ignition_strategy": null, "knock_margin_deg": null}})", kEngines, validate_only());
    return c;
}
const ComponentParams& a25a_sim() {
    static const ComponentParams c = load_vo("toyota_a25a_fks_sim.json");
    return c;
}
const ComponentParams& i4_torque_map() {
    static const ComponentParams c = load_vo("i4_2l_na.json");
    return c;
}
SimulatedEngineDesc& S(ComponentParams& c) { return std::get<SimulatedEngineDesc>(c); }
const SimulatedEngineDesc& S(const ComponentParams& c) { return std::get<SimulatedEngineDesc>(c); }
TorqueMapEngineDesc& T(ComponentParams& c) { return std::get<TorqueMapEngineDesc>(c); }

// The shipped pipe I4 with its limiter inside the generated grid (the file's default limiter 6800 is 4.6 % above the 6500 rpm
// grid - a Near finding that case S1 pins), so a crafted edit shows only its own issue.
ComponentParams clean_i4() {
    ComponentParams c = demo_i4();
    S(c).limiter.rpm = 6500.0;
    return c;
}
// The hyper with its limiter below the piston-speed Near line (stock 8250 rpm = 26.2 m/s).
ComponentParams clean_hyper() {
    ComponentParams c = hyper();
    S(c).limiter.rpm = 7500.0;
    return c;
}

std::string tag(const BuildIssue& i) {
    return std::string(to_string(i.rule)) + ":" + to_string(i.severity) + "@" + i.field_pointer;
}
std::string tags(const std::vector<BuildIssue>& v) {
    std::string out;
    for (const BuildIssue& i : v) {
        if (!out.empty()) out += ",";
        out += tag(i);
    }
    return out;
}
std::string tags(const ComponentParams& c) { return tags(check_buildable(c)); }
bool has_tag(const std::vector<BuildIssue>& v, const std::string& t) {
    return std::any_of(v.begin(), v.end(), [&](const BuildIssue& i) { return tag(i) == t; });
}
bool sourced(const char* s) {
    const std::string t = s;
    return t.rfind("[C]", 0) == 0 || t.rfind("[D]", 0) == 0 || t.rfind("[AS]", 0) == 0 || t.rfind("[M]", 0) == 0;
}

// Every issue carries a sourced label, a message and a pointer; the slot is a known part slot.
void check_well_formed(const std::vector<BuildIssue>& v) {
    for (const BuildIssue& i : v) {
        INFO(tag(i));
        CHECK(sourced(i.source_label.c_str()));
        CHECK_FALSE(i.message.empty());
        CHECK_FALSE(i.field_pointer.empty());
        CHECK(i.field_pointer[0] == '/');
        CHECK((i.slot == "engine" || i.slot == "forced_induction"));
    }
}

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// R1: the table.

TEST_CASE("R1 rule table: every threshold sourced and unique, enum values pinned, nothing named Unknown", "[engine_rules][drivetrain][cycle]") {
    std::set<std::string> names;
    for (std::size_t i = 0; i < rules::kAllEngineRuleThresholdCount; ++i) {
        const rules::Threshold& t = rules::kAllEngineRuleThresholds[i];
        INFO(t.name);
        CHECK(sourced(t.source));
        CHECK(std::isfinite(t.value));
        CHECK(names.insert(t.name).second);
    }
    // Append-only values (a game may store them).
    CHECK(static_cast<int>(BuildRule::ValveLayout) == 0);
    CHECK(static_cast<int>(BuildRule::ValveDiameterBand) == 1);
    CHECK(static_cast<int>(BuildRule::ValvePackingRow) == 2);
    CHECK(static_cast<int>(BuildRule::ValvePackingArea) == 3);
    CHECK(static_cast<int>(BuildRule::ExhaustIntakeRatio) == 4);
    CHECK(static_cast<int>(BuildRule::RodRatio) == 5);
    CHECK(static_cast<int>(BuildRule::MeanPistonSpeed) == 6);
    CHECK(static_cast<int>(BuildRule::CompressionRatio) == 7);
    CHECK(static_cast<int>(BuildRule::LimiterAboveRpmGrid) == 8);
    CHECK(static_cast<int>(BuildRule::BoostAbovePressureGrid) == 9);
    CHECK(static_cast<int>(BuildRule::LimiterAboveCurve) == 10);
    for (int r = 0; r <= 10; ++r) CHECK(std::string(to_string(static_cast<BuildRule>(r))) != "Unknown");
    CHECK(static_cast<int>(EngineChange::None) == 0);
    CHECK(static_cast<int>(EngineChange::Runtime) == 1);
    CHECK(static_cast<int>(EngineChange::TurboTables) == 2);
    CHECK(static_cast<int>(EngineChange::Maps) == 3);
    CHECK(static_cast<int>(EngineChange::Respawn) == 4);
    // The piston-speed rule reuses P3b's thresholds, it does not copy them.
    CHECK(limits::kPistonSpeedLimiting.value == 25.0);
    CHECK(limits::kPistonSpeedExceeded.value == 28.0);
}

// ---------------------------------------------------------------------------------------------------------------------
// S1: the shipped engines.

TEST_CASE("S1 every shipped engine in data/engines passes (no Limiting, no Exceeded); the Near findings are pinned",
          "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: kRodRatioMin 3.0 -> 3.1 fails toyota_a25a_fks_gen (rod ratio exactly 3.0000, Exceeded RodRatio);
    // kIntake2Max 0.44 -> 0.40 fails hyper_v8_one1_audio (0.413 of the bore, Exceeded ValveDiameterBand) - the owner's
    // requirement that the hyper's 0.41 x bore valves pass.
    std::vector<std::string> files;
    for (const auto& e : std::filesystem::directory_iterator(kEngines)) {
        if (e.path().extension() == ".json") files.push_back(e.path().filename().string());
    }
    std::sort(files.begin(), files.end());
    REQUIRE(files.size() >= 9);

    std::vector<std::string> near_lines;
    for (const std::string& f : files) {
        INFO(f);
        const ComponentParams c = load_vo(f);
        const std::vector<BuildIssue> issues = check_buildable(c);
        check_well_formed(issues);
        for (const BuildIssue& i : issues) {
            INFO(tag(i) << ": " << i.message);
            CHECK(i.severity == LimitSeverity::Near);
            near_lines.push_back(f + " " + tag(i));
        }
    }
    // Pinned: the two Near findings the stock data has (measured, see the case S1 comment in the progress file):
    //  - demo_i4_pipes: the default limiter 6800 rpm is 4.6 % over the 6500 rpm grid top;
    //  - hyper: 26.2 m/s mean piston speed at the stock 8250 rpm limiter (Near from 25 m/s).
    const std::vector<std::string> expected = {
        "demo_i4_pipes.json LimiterAboveRpmGrid:Near@/cycle/grid_rpm",
        "hyper_v8_one1_audio.json MeanPistonSpeed:Near@/limiter/rpm",
    };
    CHECK(near_lines == expected);
}

TEST_CASE("S2 the stock cleaned bases are issue-free (the crafted cases start from nothing)", "[engine_rules][drivetrain][cycle]") {
    CHECK(tags(clean_i4()) == "");
    CHECK(tags(clean_hyper()) == "");
    CHECK(tags(a25a_gen()) == "");
    CHECK(tags(a25a_sim()) == "");
    CHECK(tags(i4_torque_map()) == "");
    // Not engines / other kinds: nothing applies.
    CHECK(check_buildable(ComponentParams{ClutchDesc{}}).empty());
    SimulatedEngineDesc rotary = S(clean_i4());
    rotary.architecture = EngineArchitecture::Rotary;
    rotary.limiter.rpm = 100000.0; // piston-speed rule is piston-only
    CHECK(check_buildable(rotary).empty());
}

// ---------------------------------------------------------------------------------------------------------------------
// C1-C9: one crafted violation per rule gives exactly that issue with the right pointer.

TEST_CASE("C1 ValveLayout: 2 intake + 3 exhaust is not a production layout", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: layout_allowed() returns true for (2,3): no issue -> case fails.
    ComponentParams c = clean_i4();
    S(c).cycle->exhaust_valves = 3;
    const auto v = check_buildable(c);
    CHECK(tags(v) == "ValveLayout:Exceeded@/cycle/intake_valves");
    check_well_formed(v);
    CHECK(has_blocking_issue(v));
    // every production layout passes
    for (const auto& [ni, ne] : std::vector<std::pair<int, int>>{{1, 1}, {2, 1}, {2, 2}, {3, 2}}) {
        ComponentParams ok = clean_i4();
        CycleModelDesc& cy = *S(ok).cycle;
        cy.intake_valves = ni;
        cy.exhaust_valves = ne;
        // the diameters of a 4V head are not valid for a 5V one: scale to the middle of each band
        const real bore = cy.bore_m;
        cy.intake_valve_d_m = bore * (ni == 1 ? 0.45 : ni == 2 ? 0.36 : 0.28);
        cy.exhaust_valve_d_m = 0.84 * cy.intake_valve_d_m;
        INFO(ni << "+" << ne);
        CHECK(tags(ok) == "");
    }
}

TEST_CASE("C2 ValveDiameterBand: intake valve diameter over bore outside the layout's band", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: the band check uses kIntake1* for every layout: the 2V clean engine (0.395) then fails the band.
    {
        ComponentParams c = clean_i4();
        CycleModelDesc& cy = *S(c).cycle;
        cy.intake_valve_d_m = 0.022; // 0.256 of the 86 mm bore, below the 4V floor 0.28
        cy.exhaust_valve_d_m = 0.0185;
        const auto v = check_buildable(c);
        CHECK(tags(v) == "ValveDiameterBand:Exceeded@/cycle/intake_valve_d_mm");
        CHECK(v[0].limit == Catch::Approx(0.28));
        CHECK(v[0].value == Catch::Approx(0.022 / cy.bore_m));
        check_well_formed(v);
    }
    {
        ComponentParams c = clean_i4();
        CycleModelDesc& cy = *S(c).cycle;
        cy.intake_valve_d_m = 0.0385; // 0.448 of the bore, above 0.44 (also overfills the row, which is reported too)
        cy.exhaust_valve_d_m = 0.85 * cy.intake_valve_d_m;
        CHECK(has_tag(check_buildable(c), "ValveDiameterBand:Exceeded@/cycle/intake_valve_d_mm"));
    }
}

TEST_CASE("C3 ValvePackingRow: the two intake valves do not fit across the bore", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: kRowChordFraction 0.866 -> 1.0: no row issue -> case fails. 2 x 37.5 mm = 75 mm > 0.866 x 86 mm.
    ComponentParams c = clean_i4();
    CycleModelDesc& cy = *S(c).cycle;
    cy.intake_valve_d_m = 0.0375;
    cy.exhaust_valve_d_m = 0.0300;
    const auto v = check_buildable(c);
    CHECK(tags(v) == "ValvePackingRow:Exceeded@/cycle/intake_valve_d_mm");
    REQUIRE(v.size() == 1);
    CHECK(v[0].unit == LimitUnit::Millimetre);
    CHECK(v[0].value == Catch::Approx(37.5));
    CHECK(v[0].limit == Catch::Approx(0.8660254037844386 * 86.0 / 2.0));
    check_well_formed(v);
}

TEST_CASE("C4 ValvePackingArea: all valve heads exceed the densest packing in the bore", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: kPackShare 0.95 -> 1.2 lets 4874.9 mm^2 through (cap 6.1e3) -> case fails.
    ComponentParams c = clean_i4();
    CycleModelDesc& cy = *S(c).cycle;
    cy.intake_valve_d_m = 0.0369; // 0.429 of the bore, row 73.8 mm <= 74.5 mm
    cy.exhaust_valve_d_m = 0.0328; // 0.889 of the intake valve
    const auto v = check_buildable(c);
    CHECK(tags(v) == "ValvePackingArea:Exceeded@/cycle/intake_valve_d_mm");
    check_well_formed(v);
    // the hyper: closest shipped engine, passes with room (measured ratio to the cap in the progress file)
    const CycleModelDesc& h = *S(hyper()).cycle;
    const real total = 2.0 * h.intake_valve_d_m * h.intake_valve_d_m + 2.0 * h.exhaust_valve_d_m * h.exhaust_valve_d_m;
    const real cap = rules::kPackShare.value * rules::kPackDensity4.value * h.bore_m * h.bore_m;
    CHECK(total < cap);
    CHECK(total / cap > 0.85); // not a vacuous pass: the hyper packs 0.89 of the ideal 4-circle density
}

TEST_CASE("C5 ExhaustIntakeRatio: exhaust valve over intake valve outside 0.78-0.90", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: kExhaustRatioMax 0.90 -> 1.0: the high case below passes silently -> case fails.
    {
        ComponentParams c = clean_i4();
        S(c).cycle->exhaust_valve_d_m = 0.017; // 0.5
        CHECK(tags(c) == "ExhaustIntakeRatio:Exceeded@/cycle/exhaust_valve_d_mm");
    }
    {
        ComponentParams c = clean_i4();
        S(c).cycle->exhaust_valve_d_m = 0.0335; // 0.985
        const auto v = check_buildable(c);
        CHECK(tags(v) == "ExhaustIntakeRatio:Exceeded@/cycle/exhaust_valve_d_mm");
        check_well_formed(v);
    }
}

TEST_CASE("C6 RodRatio: rod over crank radius outside 3.0-4.2", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: rod ratio computed as rod / stroke (the colloquial 1.5-2): every shipped engine fails S1.
    {
        ComponentParams c = clean_i4();
        S(c).cycle->rod_m = 0.100; // 2.33
        const auto v = check_buildable(c);
        CHECK(tags(v) == "RodRatio:Exceeded@/cycle/rod_mm");
        CHECK(v[0].value == Catch::Approx(0.100 / 0.043));
        check_well_formed(v);
    }
    {
        ComponentParams c = clean_i4();
        S(c).cycle->rod_m = 0.250; // 5.8
        CHECK(tags(c) == "RodRatio:Exceeded@/cycle/rod_mm");
    }
}

TEST_CASE("C7 MeanPistonSpeed: Near from 25 m/s, Exceeded from 28 m/s (P3b's thresholds)", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: v computed with the bore instead of the stroke -> S1 hyper pin fails and these two fail.
    for (const auto& [rpm, expect] : std::vector<std::pair<real, std::string>>{
             {9000.0, "MeanPistonSpeed:Near@/limiter/rpm"}, {10500.0, "MeanPistonSpeed:Exceeded@/limiter/rpm"}}) {
        ComponentParams c = clean_i4();
        SimulatedEngineDesc& e = S(c);
        e.limiter.rpm = rpm;
        e.cycle->grid_rpm = extend_grid_rpm(*e.cycle, rpm); // isolate the rule from the grid rule
        INFO(rpm);
        const auto v = check_buildable(c);
        CHECK(tags(v) == expect);
        check_well_formed(v);
    }
    // without a cycle block the rule still applies (bore/stroke are in the desc)
    ComponentParams sim = a25a_sim();
    S(sim).limiter.rpm = 12000.0; // 103.4 mm stroke: 41 m/s
    CHECK(tags(sim) == "MeanPistonSpeed:Exceeded@/limiter/rpm");
}

TEST_CASE("C8 CompressionRatio: NA band 8-14.5, forced induction 7-10.5", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: the forced flag ignored (always the NA band): the hyper at 11 passes -> case fails.
    {
        ComponentParams c = clean_i4();
        S(c).cycle->compression_ratio = 15.0;
        CHECK(tags(c) == "CompressionRatio:Exceeded@/cycle/compression_ratio");
    }
    {
        ComponentParams c = clean_i4();
        S(c).cycle->compression_ratio = 7.0;
        CHECK(tags(c) == "CompressionRatio:Exceeded@/cycle/compression_ratio");
    }
    {
        ComponentParams c = clean_hyper();
        S(c).cycle->compression_ratio = 11.0; // fine for an NA engine, too high with the turbo
        const auto v = check_buildable(c);
        CHECK(tags(v) == "CompressionRatio:Exceeded@/cycle/compression_ratio");
        CHECK(v[0].limit == Catch::Approx(10.5));
        ComponentParams na = c;
        S(na).turbo_pair.reset();
        CHECK(tags(na).find("CompressionRatio") == std::string::npos);
    }
}

TEST_CASE("C9 LimiterAboveRpmGrid: severity grows with the overshoot (Near / Limiting / Exceeded)", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: the rule compares with the idle rpm node (the first, not the last): the 7200 case disappears.
    for (const auto& [rpm, expect] : std::vector<std::pair<real, std::string>>{
             {6800.0, "LimiterAboveRpmGrid:Near@/cycle/grid_rpm"},
             {7200.0, "LimiterAboveRpmGrid:Limiting@/cycle/grid_rpm"},
             {8000.0, "LimiterAboveRpmGrid:Exceeded@/cycle/grid_rpm"}}) {
        ComponentParams c = clean_i4();
        S(c).limiter.rpm = rpm;
        INFO(rpm);
        const auto v = check_buildable(c);
        CHECK(tags(v) == expect);
        check_well_formed(v);
    }
    // extending the grid with the A12 helper clears it
    ComponentParams c = clean_i4();
    S(c).limiter.rpm = 8000.0;
    S(c).cycle->grid_rpm = extend_grid_rpm(*S(c).cycle, 8000.0);
    CHECK(tags(c) == "");
}

TEST_CASE("C10 BoostAbovePressureGrid: ambient + boost target over the last manifold-pressure node", "[engine_rules][drivetrain][cycle][turbo]") {
    // Recorded sabotage: the rule drops kIsaAirPressurePa (compares the gauge target with the absolute node): the Near
    // case below (131.3 kPa needed vs 130 kPa) disappears.
    {
        ComponentParams c = clean_hyper();
        S(c).cycle->grid_p_im = {25000.0, 50000.0, 100000.0};
        const auto v = check_buildable(c);
        CHECK(tags(v) == "BoostAbovePressureGrid:Exceeded@/cycle/grid_p_im_kpa");
        CHECK(v[0].value == Catch::Approx(kIsaAirPressurePa + S(c).turbo_pair->target_boost_pa));
        CHECK(v[0].limit == Catch::Approx(100000.0));
        check_well_formed(v);
    }
    {
        ComponentParams c = clean_hyper();
        S(c).turbo_pair->target_boost_pa = 30000.0;
        S(c).cycle->grid_p_im = {25000.0, 50000.0, 100000.0, 130000.0};
        CHECK(tags(c) == "BoostAbovePressureGrid:Near@/cycle/grid_p_im_kpa");
    }
    // without a turbo the rule does not exist (a node above ambient is the loader's business)
    ComponentParams na = clean_i4();
    S(na).cycle->grid_p_im = {25000.0, 50000.0};
    CHECK(tags(na).find("BoostAbovePressureGrid") == std::string::npos);
}

TEST_CASE("C11 LimiterAboveCurve: torque-map engine limiter above its curves", "[engine_rules][drivetrain]") {
    // Recorded sabotage: the motoring curve skipped: only the wot issue is produced -> case fails.
    ComponentParams c = i4_torque_map();
    TorqueMapEngineDesc& e = T(c);
    const real wot_end = e.wot_torque_nm_vs_rpm.x.back();
    const real mot_end = e.motoring_torque_nm_vs_rpm.x.back();
    e.limiter.rpm = 1.10 * std::max(wot_end, mot_end);
    std::vector<std::string> expected;
    if (e.limiter.rpm > mot_end) expected.push_back("LimiterAboveCurve:Limiting@/motoring_torque_nm_vs_rpm");
    if (e.limiter.rpm > wot_end) expected.push_back("LimiterAboveCurve:Limiting@/wot_torque_nm_vs_rpm");
    std::sort(expected.begin(), expected.end(), [](const std::string& a, const std::string& b) {
        return a.substr(a.find('@')) < b.substr(b.find('@'));
    });
    std::string joined;
    for (const auto& s : expected) joined += (joined.empty() ? "" : ",") + s;
    CHECK(tags(c) == joined);
    check_well_formed(check_buildable(c));
    // a torque-map engine gets no geometry rules, whatever its limiter
    ComponentParams ok = i4_torque_map();
    CHECK(tags(ok) == "");
    // A governed engine is held to its governor's high idle, not to the overspeed limiter above it: the shipped tractor diesel
    // (curves end at 2400 rpm = high idle, limiter 2650) passes; a high idle past the curve is reported.
    ComponentParams diesel = load_vo("diesel_i4_4l5_turbo.json");
    REQUIRE(T(diesel).has_governor);
    CHECK(tags(diesel) == "");
    T(diesel).governor.high_idle_rpm = 3000.0;
    T(diesel).limiter.rpm = 3500.0;
    CHECK(tags(diesel) ==
          "LimiterAboveCurve:Exceeded@/motoring_torque_nm_vs_rpm,LimiterAboveCurve:Exceeded@/wot_torque_nm_vs_rpm");
}

// ---------------------------------------------------------------------------------------------------------------------
// K1-K3: classification.

namespace {

struct ClassRow {
    const char* field;     // design 6.x parameter (JSON pointer or name)
    const char* design;    // the design's Regen column: "none", "T", "M", or "-" (not a regen class: respawn / vehicle level)
    const ComponentParams& (*base)();
    std::function<void(SimulatedEngineDesc&)> edit;
    EngineChange expected;
};

[[nodiscard]] EngineChange from_design(const std::string& label) {
    if (label == "none") return EngineChange::Runtime;
    if (label == "T") return EngineChange::TurboTables;
    if (label == "M") return EngineChange::Maps;
    return EngineChange::Respawn; // "-": never compared
}

const TurboPairDesc& hyper_turbo() { return *S(hyper()).turbo_pair; }

real bump(real v) { return std::nextafter(v, real(1e300)); }

} // namespace

TEST_CASE("K1 classification table: every row of design section 6's Regen column, field by field", "[engine_rules][drivetrain][cycle][turbo]") {
    // Recorded sabotage: classify_engine_change compares only the cycle block's key and ignores the turbo key -> every T row
    // reads Runtime; separately, dropping the piston bore/stroke rule makes the geometry rows read Runtime.
    using E = SimulatedEngineDesc;
    const std::vector<ClassRow> rows = {
        // --- 6.1 ECU ---
        {"/idle_rpm", "none", hyper, [](E& e) { e.idle_rpm += 50.0; }, EngineChange::Runtime},
        {"/limiter/rpm inside the grid", "none", hyper, [](E& e) { e.limiter.rpm -= 100.0; }, EngineChange::Runtime},
        {"/limiter/rpm above the grid, grid extended", "M", demo_i4,
         [](E& e) {
             e.limiter.rpm = 8000.0;
             e.cycle->grid_rpm = extend_grid_rpm(*e.cycle, 8000.0);
         },
         EngineChange::Maps},
        {"/limiter/hysteresis_rpm", "none", hyper, [](E& e) { e.limiter.hysteresis_rpm += 50.0; }, EngineChange::Runtime},
        {"/limiter/type", "none", hyper, [](E& e) { e.limiter.type = LimiterType::SoftCut; }, EngineChange::Runtime},
        {"/dfco_enable_rpm", "none", hyper, [](E& e) { e.dfco_enable_rpm += 100.0; }, EngineChange::Runtime},
        {"/idle_control/kp", "none", hyper, [](E& e) { e.idle.kp_per_rpm *= 1.5; }, EngineChange::Runtime},
        {"/idle_control/max_torque_nm", "none", hyper, [](E& e) { e.idle.max_torque_nm += 5.0; }, EngineChange::Runtime},
        {"/ecu/lambda_target", "none", hyper_schedule, [](E& e) { e.ecu.lambda_target.values.push_back(1.0); }, EngineChange::Runtime},
        {"/ecu/spark_advance_deg", "none", hyper, [](E& e) { e.ecu.spark_advance_deg.values.push_back(1.0); }, EngineChange::Runtime},
        {"/cycle/spark_advance_deg (schedule)", "M", a25a_gen,
         [](E& e) { e.cycle->spark_advance_deg.values.push_back(1.0); }, EngineChange::Maps},
        {"/cycle/cam (phasers)", "M", a25a_gen, [](E& e) { e.cycle->cam.intake_advance_deg.values.push_back(1.0); }, EngineChange::Maps},
        {"/inertia_kgm2 (flywheel)", "none", hyper, [](E& e) { e.inertia_kgm2 *= 0.9; }, EngineChange::Runtime},
        // --- 6.2 valvetrain ---
        {"/cycle/intake_valves", "M", hyper, [](E& e) { e.cycle->intake_valves += 1; }, EngineChange::Maps},
        {"/cycle/exhaust_valves", "M", hyper, [](E& e) { e.cycle->exhaust_valves += 1; }, EngineChange::Maps},
        {"/cycle/intake_valve_d_mm", "M", hyper, [](E& e) { e.cycle->intake_valve_d_m *= 0.97; }, EngineChange::Maps},
        {"/cycle/exhaust_valve_d_mm", "M", hyper, [](E& e) { e.cycle->exhaust_valve_d_m *= 0.97; }, EngineChange::Maps},
        {"/cycle/cd_vs_l_over_d", "M", hyper, [](E& e) { e.cycle->cd_vs_l_over_d.y.front() = bump(e.cycle->cd_vs_l_over_d.y.front()); }, EngineChange::Maps},
        {"/cycle/head_area_factor", "M", hyper, [](E& e) { e.cycle->head_area_factor += 0.05; }, EngineChange::Maps},
        {"/cycle/intake_lift", "M", hyper, [](E& e) { e.cycle->intake_lift.phase_ref_deg += 2.0; }, EngineChange::Maps},
        // --- 6.3 geometry ---
        {"/bore_mm", "M", hyper, [](E& e) { e.piston.bore_m *= 1.03; }, EngineChange::Maps},
        {"/stroke_mm", "M", hyper, [](E& e) { e.piston.stroke_m *= 1.05; }, EngineChange::Maps},
        {"/cycle/rod_mm", "M", hyper, [](E& e) { e.cycle->rod_m *= 1.05; }, EngineChange::Maps},
        {"/cycle/compression_ratio", "M", hyper, [](E& e) { e.cycle->compression_ratio -= 0.5; }, EngineChange::Maps},
        {"/cylinders", "-", hyper, [](E& e) { e.piston.cylinders = 6; e.cycle->cylinders = 6; }, EngineChange::Respawn},
        {"/architecture", "-", hyper, [](E& e) { e.architecture = EngineArchitecture::Rotary; }, EngineChange::Respawn},
        {"/ignition", "-", hyper, [](E& e) { e.ignition = EngineIgnition::Compression; }, EngineChange::Respawn},
        // --- 6.4 forced induction (hyper has the turbo) ---
        {"FI target_boost_bar", "T", hyper, [](E& e) { e.turbo_pair->target_boost_pa -= 20000.0; }, EngineChange::TurboTables},
        {"FI boost_control kp", "T", hyper, [](E& e) { e.turbo_pair->wastegate_kp *= 1.5; }, EngineChange::TurboTables},
        {"FI boost_control kd_s", "T", hyper, [](E& e) { e.turbo_pair->wastegate_kd_s *= 2.0; }, EngineChange::TurboTables},
        // DEVIATION (design says T): the wastegate size is not an input of the derived tables or the engine maps - the loader's
        // turbo key leaves it out, and the engine takes it at construction - so it classifies as Runtime.
        {"TH wastegate_area_m2", "T", hyper,
         [](E& e) { e.turbo_pair->wastegate_area_m2 = e.turbo_pair->wastegate_area() * 1.2; }, EngineChange::Runtime},
        {"FI intercooler_effectiveness", "T", hyper, [](E& e) { e.turbo_pair->intercooler_effectiveness = 0.9; }, EngineChange::TurboTables},
        // DEVIATION (design says T): the intercooler pressure drop is not in the turbo key either.
        {"FI intercooler dp", "T", hyper, [](E& e) { e.turbo_pair->intercooler_dp_pa_at_max_flow = 10000.0; }, EngineChange::Runtime},
        {"FI blow_off_valve", "none", hyper,
         [](E& e) {
             BlowOffValveDesc b;
             b.crack_dp_pa = 20000.0;
             b.full_open_dp_pa = 40000.0;
             b.effective_area_m2 = 1e-3;
             b.tau_s = 0.05;
             e.turbo_pair->blow_off_valve = b;
         },
         EngineChange::Runtime},
        {"FI anti_lag", "none", hyper,
         [](E& e) {
             AntiLagDesc a;
             a.arm_below_command = 0.2;
             a.min_rpm = 2000.0;
             a.max_rpm = 6000.0;
             a.max_active_s = 5.0;
             e.turbo_pair->anti_lag = a;
         },
         EngineChange::Runtime},
        {"TH rotor_inertia (turbo hardware)", "T", hyper, [](E& e) { e.turbo_pair->rotor_inertia_kgm2 *= 1.4; }, EngineChange::TurboTables},
        {"FI count (extensive area x2)", "T", hyper, [](E& e) { e.turbo_pair->turbine_area_m2 *= 2.0; }, EngineChange::TurboTables},
        {"FI boost beyond the grid, grid extended", "T+M", hyper,
         [](E& e) {
             e.turbo_pair->target_boost_pa = 250000.0;
             e.cycle->grid_p_im = boost_grid_p_im(*e.cycle, 2.5);
         },
         EngineChange::Maps},
        // --- 7.1 add / remove ---
        {"add forced induction (pipe engine)", "M", demo_i4, [](E& e) { e.turbo_pair = hyper_turbo(); }, EngineChange::Maps},
        {"add forced induction (lumped engine)", "M", a25a_gen, [](E& e) { e.turbo_pair = hyper_turbo(); }, EngineChange::TurboTables},
        {"add forced induction + grid extended", "M", a25a_gen,
         [](E& e) {
             e.turbo_pair = hyper_turbo();
             e.cycle->grid_p_im = boost_grid_p_im(*e.cycle, 1.8);
         },
         EngineChange::Maps},
        {"remove forced induction", "M", hyper, [](E& e) { e.turbo_pair.reset(); }, EngineChange::Maps},
        // --- 6.5 intake and exhaust ---
        {"/cycle/exhaust/primaries length", "M", hyper, [](E& e) { e.cycle->exhaust->primaries[0].length_m *= 1.4; }, EngineChange::Maps},
        {"/cycle/exhaust/merges length", "M", hyper, [](E& e) { e.cycle->exhaust->merges[0].length_m *= 1.4; }, EngineChange::Maps},
        {"/cycle/intake/runners length", "M", hyper, [](E& e) { e.cycle->intake->runners[0].length_m *= 1.3; }, EngineChange::Maps},
        {"/cycle runner_length_mm (lumped)", "M", a25a_gen, [](E& e) { e.cycle->runner_length_m *= 1.3; }, EngineChange::Maps},
        {"installation exhaust element length", "M", demo_i4,
         [](E& e) { e.cycle->installation->exhaust.elements[0].length_m *= 1.5; }, EngineChange::Maps},
        {"/throttle_body/diameter_mm", "none", hyper, [](E& e) { e.throttle_body.diameter_m += 0.008; }, EngineChange::Runtime},
        {"/cycle/intake/throttle_bore_m", "M", hyper, [](E& e) { e.cycle->intake->throttle_bore_m += 0.008; }, EngineChange::Maps},
        // Outlet and inlet positions come from the car: audio only, never regenerate.
        {"installation outlet position", "none", demo_i4,
         [](E& e) { e.cycle->installation->exhaust.elements.back().position_m = Vec3{9.0, 9.0, 9.0}; }, EngineChange::Runtime},
        // --- fuel ---
        {"fuel lhv (the generator's fuel)", "M", hyper, [](E& e) { e.fuels.front().lhv_j_kg *= 0.9; }, EngineChange::Maps},
        {"fuels[1] only (not the generator's)", "none", hyper_schedule,
         [](E& e) {
             e.fuels.push_back(e.fuels.front());
             e.fuels.back().name = "other";
         },
         EngineChange::Runtime},
    };

    std::vector<std::string> deviations;
    for (const ClassRow& row : rows) {
        INFO(row.field);
        ComponentParams before = row.base();
        ComponentParams after = before;
        row.edit(S(after));
        const EngineChange got = classify_engine_change(before, after);
        CHECK(std::string(to_string(got)) == to_string(row.expected));
        // symmetric
        CHECK(classify_engine_change(after, before) == got);
        // the field list is never empty when the class is not None, and its maximum is the class
        const auto list = list_engine_changes(before, after);
        CHECK_FALSE(list.empty());
        EngineChange worst = EngineChange::None;
        for (const auto& c : list) worst = std::max(worst, c.change);
        CHECK(worst == got);
        const std::string d = row.design;
        if (d != "-" && d != "T+M" && from_design(d) != got) deviations.push_back(row.field);
    }
    // The documented deviations from the design's Regen column - exactly these (progress file, decision D-K1).
    const std::vector<std::string> expected_deviations = {
        "TH wastegate_area_m2",
        "FI intercooler dp",
        "add forced induction (lumped engine)",
    };
    CHECK(deviations == expected_deviations);
}

TEST_CASE("K2 identical descriptors are None; provenance text alone is None; torque-map and kind changes", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: FuelDesc comparison includes `source`: the provenance case reads Runtime -> case fails.
    for (const ComponentParams* c : {&hyper(), &demo_i4(), &a25a_gen(), &a25a_sim(), &i4_torque_map()}) {
        CHECK(classify_engine_change(*c, *c) == EngineChange::None);
        CHECK(list_engine_changes(*c, *c).empty());
    }
    {
        ComponentParams a = hyper();
        ComponentParams b = a;
        S(b).fuels.front().source += " (citation reworded)";
        CHECK(classify_engine_change(a, b) == EngineChange::None);
        S(b).cycle->default_installation_path += "x";
        S(b).cycle->installation_path += "x";
        if (S(b).cycle->intake) S(b).cycle->intake->source += "x";
        if (S(b).cycle->exhaust) S(b).cycle->exhaust->source += "x";
        CHECK(classify_engine_change(a, b) == EngineChange::None);
        // ... but the cycle block's own source IS in the map key (it ends up in EngineMaps::source): Maps
        S(b).cycle->source += "x";
        CHECK(classify_engine_change(a, b) == EngineChange::Maps);
    }
    {
        // torque map: curves and limiter are runtime, ignition is a different engine
        ComponentParams a = i4_torque_map();
        ComponentParams b = a;
        T(b).wot_torque_nm_vs_rpm.y.front() += 1.0;
        CHECK(classify_engine_change(a, b) == EngineChange::Runtime);
        b = a;
        T(b).limiter.rpm += 100.0;
        CHECK(classify_engine_change(a, b) == EngineChange::Runtime);
        b = a;
        T(b).ignition = EngineIgnition::Compression;
        CHECK(classify_engine_change(a, b) == EngineChange::Respawn);
    }
    // torque map <-> simulated is a different engine; a non-engine alternative is refused
    CHECK(classify_engine_change(i4_torque_map(), hyper()) == EngineChange::Respawn);
    CHECK(classify_engine_change(hyper(), i4_torque_map()) == EngineChange::Respawn);
    CHECK_THROWS_AS(classify_engine_change(ComponentParams{ClutchDesc{}}, hyper()), std::invalid_argument);
    CHECK(std::string(to_string(EngineChange::TurboTables)) == "TurboTables");
}

namespace {
struct KeyEdit {
    const char* name;
    std::function<void(SimulatedEngineDesc&)> edit;
    bool moves_key;
};
// The cycle exactly as the loader hands it to the generator.
CycleModelDesc effective(const SimulatedEngineDesc& e) {
    CycleModelDesc c = *e.cycle;
    if (c.intake.has_value()) c.intake->compressor_supply = e.turbo_pair.has_value();
    return c;
}
std::uint64_t key_of(const SimulatedEngineDesc& e) { return engine_map_cache_key(effective(e), e.fuels.front(), MapQuality::Full); }
} // namespace

TEST_CASE("K3 key-coverage walk: a field moves the map key iff it classifies as Maps (H3-style table)", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: the classifier drops the Maps branch for cycle.intake (runners/throttle bore/plenum): those rows read
    // Runtime while the independent key moves -> case fails on exactly those rows.
    ComponentParams base = demo_i4();
    S(base).cycle->knock = {}; // 2026-10-18: recorded on the schedule-only engine (the shipped one is knock-limited now)
    REQUIRE(S(base).cycle->intake.has_value());
    REQUIRE(S(base).cycle->exhaust.has_value());
    REQUIRE(S(base).cycle->installation.has_value());

    using E = SimulatedEngineDesc;
    std::vector<KeyEdit> edits = {
        {"bore_m", [](E& e) { e.cycle->bore_m = bump(e.cycle->bore_m); e.piston.bore_m = e.cycle->bore_m; }, true},
        {"stroke_m", [](E& e) { e.cycle->stroke_m = bump(e.cycle->stroke_m); e.piston.stroke_m = e.cycle->stroke_m; }, true},
        {"rod_m", [](E& e) { e.cycle->rod_m = bump(e.cycle->rod_m); }, true},
        {"pin_offset_m", [](E& e) { e.cycle->pin_offset_m = bump(e.cycle->pin_offset_m + 1e-6); }, true},
        {"compression_ratio", [](E& e) { e.cycle->compression_ratio = bump(e.cycle->compression_ratio); }, true},
        {"intake_valves", [](E& e) { e.cycle->intake_valves += 1; }, true},
        {"exhaust_valves", [](E& e) { e.cycle->exhaust_valves += 1; }, true},
        {"intake_valve_d_m", [](E& e) { e.cycle->intake_valve_d_m = bump(e.cycle->intake_valve_d_m); }, true},
        {"exhaust_valve_d_m", [](E& e) { e.cycle->exhaust_valve_d_m = bump(e.cycle->exhaust_valve_d_m); }, true},
        {"intake_lift curve y", [](E& e) { e.cycle->intake_lift.lift_m_vs_deg.y.front() = bump(e.cycle->intake_lift.lift_m_vs_deg.y.front()); }, true},
        {"exhaust_lift phase_ref_deg", [](E& e) { e.cycle->exhaust_lift.phase_ref_deg = bump(e.cycle->exhaust_lift.phase_ref_deg); }, true},
        {"cd_vs_l_over_d y", [](E& e) { e.cycle->cd_vs_l_over_d.y.front() = bump(e.cycle->cd_vs_l_over_d.y.front()); }, true},
        {"head_area_factor", [](E& e) { e.cycle->head_area_factor = bump(e.cycle->head_area_factor + 1.0); }, true},
        {"wiebe_a", [](E& e) { e.cycle->wiebe_a = bump(e.cycle->wiebe_a + 1.0); }, true},
        {"burn_duration_deg", [](E& e) { e.cycle->burn_duration_deg = bump(e.cycle->burn_duration_deg + 1.0); }, true},
        {"woschni_c2", [](E& e) { e.cycle->woschni_c2 = bump(e.cycle->woschni_c2 + 1.0); }, true},
        {"wall_temperature_k", [](E& e) { e.cycle->wall_temperature_k = bump(e.cycle->wall_temperature_k); }, true},
        {"friction.a_pa", [](E& e) { e.cycle->friction.a_pa = bump(e.cycle->friction.a_pa); }, true},
        {"gas.fresh.cv_a_j_kgk", [](E& e) { e.cycle->gas.fresh.cv_a_j_kgk = bump(e.cycle->gas.fresh.cv_a_j_kgk); }, true},
        {"grid_rpm", [](E& e) { e.cycle->grid_rpm.back() = bump(e.cycle->grid_rpm.back()); }, true},
        {"grid_p_im", [](E& e) { e.cycle->grid_p_im.back() = bump(e.cycle->grid_p_im.back()); }, true},
        {"cam.intake_advance_deg values", [](E& e) { e.cycle->cam.intake_advance_deg.values.push_back(1.0); }, true},
        {"spark_advance_deg values", [](E& e) { e.cycle->spark_advance_deg.values.push_back(1.0); }, true},
        {"ignition_delay_deg", [](E& e) { e.cycle->ignition_delay_deg = bump(e.cycle->ignition_delay_deg + 1.0); }, true},
        {"source", [](E& e) { e.cycle->source += "x"; }, true},
        {"intake.throttle_bore_m", [](E& e) { e.cycle->intake->throttle_bore_m = bump(e.cycle->intake->throttle_bore_m); }, true},
        {"intake.plenum_m3", [](E& e) { e.cycle->intake->plenum_m3 = bump(e.cycle->intake->plenum_m3); }, true},
        {"intake.runners[0].length_m", [](E& e) { e.cycle->intake->runners[0].length_m = bump(e.cycle->intake->runners[0].length_m); }, true},
        {"intake.runners[0].d_in_m", [](E& e) { e.cycle->intake->runners[0].d_in_m = bump(e.cycle->intake->runners[0].d_in_m); }, true},
        {"exhaust.primaries[0].length_m", [](E& e) { e.cycle->exhaust->primaries[0].length_m = bump(e.cycle->exhaust->primaries[0].length_m); }, true},
        {"exhaust.primaries[0].d_m", [](E& e) { e.cycle->exhaust->primaries[0].d_m = bump(e.cycle->exhaust->primaries[0].d_m); }, true},
        {"exhaust.merges[0].length_m", [](E& e) { e.cycle->exhaust->merges[0].length_m = bump(e.cycle->exhaust->merges[0].length_m); }, true},
        {"resolution.power_dx_m", [](E& e) { e.cycle->resolution.power_dx_m = bump(e.cycle->resolution.power_dx_m); }, true},
        {"resolution.audio_dx_exhaust_m", [](E& e) { e.cycle->resolution.audio_dx_exhaust_m = bump(e.cycle->resolution.audio_dx_exhaust_m); }, true},
        {"installation.exhaust.elements[0].length_m", [](E& e) { e.cycle->installation->exhaust.elements[0].length_m = bump(e.cycle->installation->exhaust.elements[0].length_m); }, true},
        {"installation.intake.airbox_m3", [](E& e) { e.cycle->installation->intake.airbox_m3 = bump(e.cycle->installation->intake.airbox_m3 + 1e-3); }, true},
        {"firing_order", [](E& e) { e.cycle->firing_order = {1, 2, 4, 3}; }, true},
        {"turbo appears (compressor_supply)", [](E& e) { e.turbo_pair = hyper_turbo(); }, true},
        {"fuel lhv", [](E& e) { e.fuels.front().lhv_j_kg = bump(e.fuels.front().lhv_j_kg); }, true},
        {"fuel afr_stoich", [](E& e) { e.fuels.front().afr_stoich = bump(e.fuels.front().afr_stoich); }, true},
        // --- not in the key ---
        {"banks", [](E& e) { e.cycle->banks = {{1, 2}, {3, 4}}; }, false},
        {"bank_angle_deg", [](E& e) { e.cycle->bank_angle_deg += 10.0; }, false},
        {"installation.intake.inlet_position_m", [](E& e) { e.cycle->installation->intake.inlet_position_m = Vec3{1.0, 2.0, 3.0}; }, false},
        {"installation outlet position", [](E& e) { e.cycle->installation->exhaust.elements.back().position_m = Vec3{5.0, 5.0, 5.0}; }, false},
        {"intake.source", [](E& e) { e.cycle->intake->source += "x"; }, false},
        {"exhaust.source", [](E& e) { e.cycle->exhaust->source += "x"; }, false},
        {"installation.source", [](E& e) { e.cycle->installation->source += "x"; }, false},
        {"default_installation_path", [](E& e) { e.cycle->default_installation_path += "x"; }, false},
        {"installation_path", [](E& e) { e.cycle->installation_path += "x"; }, false},
        {"fuel source (citation)", [](E& e) { e.fuels.front().source += "x"; }, false},
        {"limiter.rpm", [](E& e) { e.limiter.rpm += 100.0; }, false},
        {"idle_rpm", [](E& e) { e.idle_rpm += 10.0; }, false},
        {"inertia_kgm2", [](E& e) { e.inertia_kgm2 *= 1.1; }, false},
        {"throttle_body.diameter_m", [](E& e) { e.throttle_body.diameter_m += 0.001; }, false},
        {"ecu.lambda_target", [](E& e) { e.ecu.lambda_target.values.push_back(1.0); }, false},
    };

    for (const KeyEdit& k : edits) {
        INFO(k.name);
        ComponentParams after = base;
        k.edit(S(after));
        const bool key_moves = key_of(S(base)) != key_of(S(after));
        // the table's claim matches the independent key computation ...
        CHECK(key_moves == k.moves_key);
        // ... and the classifier agrees
        const EngineChange got = classify_engine_change(base, after);
        if (k.moves_key) {
            CHECK(got == EngineChange::Maps);
        } else {
            CHECK(got != EngineChange::Maps);
            CHECK(got != EngineChange::Respawn);
        }
    }
    // cylinders move the key too, but they are a different engine
    ComponentParams cyl = base;
    S(cyl).cycle->cylinders += 2;
    S(cyl).piston.cylinders += 2;
    CHECK(key_of(S(base)) != key_of(S(cyl)));
    CHECK(classify_engine_change(base, cyl) == EngineChange::Respawn);
}

TEST_CASE("K4 turbo_table_key reproduces the loader's derived-table key on the shipped configurations", "[engine_rules][drivetrain][turbo][io]") {
    // Recorded sabotage: the parameter list drops wastegate_kd_s (or reorders two parameters): the key no longer equals
    // maps->content_hash -> case fails. This is the drift guard against ps::io::load_turbo_configuration_json's list.
    const std::filesystem::path cache = std::filesystem::temp_directory_path() / "ps_garage_p4_turbo";
    std::filesystem::create_directories(cache);
    for (const char* file : {"hyper_twin_default.json", "hyper_twin_n2o.json"}) {
        INFO(file);
        const TurboPairDesc t = io::load_turbo_configuration_json(kTurboConfigs + file, cache.string());
        REQUIRE(t.maps != nullptr);
        CHECK(turbo_table_key(t) == t.maps->content_hash);
    }
    // the key moves with every one of its parameters, not with the wastegate area
    TurboPairDesc t = hyper_turbo();
    const std::uint64_t k0 = turbo_table_key(t);
    const std::vector<std::pair<const char*, std::function<void(TurboPairDesc&)>>> moving = {
        {"reference_head_pa", [](TurboPairDesc& x) { x.reference_head_pa = bump(x.reference_head_pa); }},
        {"rotor_inertia_kgm2", [](TurboPairDesc& x) { x.rotor_inertia_kgm2 = bump(x.rotor_inertia_kgm2); }},
        {"reference_rpm", [](TurboPairDesc& x) { x.reference_rpm = bump(x.reference_rpm); }},
        {"max_mass_flow_kg_s", [](TurboPairDesc& x) { x.max_mass_flow_kg_s = bump(x.max_mass_flow_kg_s); }},
        {"compressor_efficiency", [](TurboPairDesc& x) { x.compressor_efficiency = bump(x.compressor_efficiency); }},
        {"turbine_efficiency", [](TurboPairDesc& x) { x.turbine_efficiency = bump(x.turbine_efficiency); }},
        {"turbine_area_m2", [](TurboPairDesc& x) { x.turbine_area_m2 = bump(x.turbine_area_m2); }},
        {"target_boost_pa", [](TurboPairDesc& x) { x.target_boost_pa = bump(x.target_boost_pa); }},
        {"intercooler_effectiveness", [](TurboPairDesc& x) { x.intercooler_effectiveness = bump(x.intercooler_effectiveness); }},
        {"shaft_loss_w_at_reference", [](TurboPairDesc& x) { x.shaft_loss_w_at_reference = bump(x.shaft_loss_w_at_reference); }},
        {"wastegate_kp", [](TurboPairDesc& x) { x.wastegate_kp = bump(x.wastegate_kp); }},
        {"wastegate_ki_per_s", [](TurboPairDesc& x) { x.wastegate_ki_per_s = bump(x.wastegate_ki_per_s); }},
        {"wastegate_kd_s", [](TurboPairDesc& x) { x.wastegate_kd_s = bump(x.wastegate_kd_s); }},
        {"wastegate_actuator_tau_s", [](TurboPairDesc& x) { x.wastegate_actuator_tau_s = bump(x.wastegate_actuator_tau_s); }},
    };
    for (const auto& [name, edit] : moving) {
        TurboPairDesc u = t;
        edit(u);
        INFO(name);
        CHECK(turbo_table_key(u) != k0);
    }
    TurboPairDesc u = t;
    u.wastegate_area_m2 = t.wastegate_area() * 2.0;
    u.intercooler_dp_pa_at_max_flow = 12345.0;
    CHECK(turbo_table_key(u) == k0);
}

// ---------------------------------------------------------------------------------------------------------------------
// G: the grid helpers.

TEST_CASE("G0 grid helpers: values, errors, determinism", "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: kBoostGridStepPa 60000 -> 50000: the hyper reproduction below fails (a different node set).
    const CycleModelDesc& h = *S(hyper()).cycle;
    // The hyper's own boosted axis is boost_grid_p_im for its 1.8 bar target on its stock part: 100 -> 160 -> 220 -> 300 kPa.
    const std::vector<real> b = boost_grid_p_im(h, 1.8);
    CHECK(b == h.grid_p_im);
    // other targets: top = (1 + target + 0.2) bar, last step 30..90 kPa
    const std::vector<real> b1 = boost_grid_p_im(h, 1.0);
    CHECK(b1.back() == Catch::Approx(220000.0));
    CHECK(b1 == std::vector<real>({25000.0, 50000.0, 100000.0, 160000.0, 220000.0}));
    const std::vector<real> b3 = boost_grid_p_im(h, 0.5);
    CHECK(b3.back() == Catch::Approx(170000.0));
    for (const real t : {0.3, 0.5, 1.0, 1.4, 1.8, 2.2, 3.0}) {
        const std::vector<real> g = boost_grid_p_im(h, t);
        INFO(t);
        CHECK(std::is_sorted(g.begin(), g.end(), std::less_equal<real>()));
        CHECK(g.back() == Catch::Approx((1.0 + t + 0.2) * 1e5).margin(1e-3));
        const real last_step = g.back() - g[g.size() - 2];
        CHECK(last_step >= 30000.0 - 1e-3);
        CHECK(last_step <= 90000.0 + 1e-3);
        CHECK(g[0] == 25000.0);
    }
    CHECK_THROWS_AS(boost_grid_p_im(h, 0.0), std::invalid_argument);
    CHECK_THROWS_AS(boost_grid_p_im(h, -1.0), std::invalid_argument);
    CycleModelDesc tiny = h;
    tiny.grid_p_im = {100000.0, 150000.0, 200000.0}; // only one node at/below ambient
    CHECK_THROWS_AS(boost_grid_p_im(tiny, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(ambient_grid_p_im(tiny), std::invalid_argument);

    // ambient_grid_p_im: the hyper's axis without its boosted rows; the 100 kPa WOT row collapses into the NA cap (0.95 ambient)
    const real na_cap = 0.95 * kIsaAirPressurePa;
    CHECK(ambient_grid_p_im(h) == std::vector<real>({25000.0, 50000.0, na_cap}));
    // an NA engine's own axis: rows up to the cap unchanged, its 100 kPa row capped
    const CycleModelDesc& d = *S(demo_i4()).cycle;
    CHECK(ambient_grid_p_im(d) == std::vector<real>({25000.0, 30000.0, 50000.0, 70000.0, 90000.0, na_cap}));

    // extend_grid_rpm
    CHECK(extend_grid_rpm(d, 6000.0) == d.grid_rpm); // covered: unchanged
    CHECK(extend_grid_rpm(d, 6500.0) == d.grid_rpm);
    const std::vector<real> e = extend_grid_rpm(d, 8000.0);
    CHECK(e.back() == 8000.0);
    CHECK(std::vector<real>(e.begin(), e.begin() + d.grid_rpm.size()) == d.grid_rpm);
    CHECK(e == std::vector<real>({800.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0}));
    // within half an interval of a node: the limiter replaces it (no sliver)
    const std::vector<real> e2 = extend_grid_rpm(d, 7100.0);
    CHECK(e2.back() == 7100.0);
    CHECK(e2.size() == d.grid_rpm.size() + 1);
    CycleModelDesc one = d;
    one.grid_rpm = {1000.0};
    CHECK_THROWS_AS(extend_grid_rpm(one, 7000.0), std::invalid_argument);
}

namespace {

const MapQuality kQ = MapQuality::Sketch;

DynoSweepSpec spec_at(std::vector<double> rpm) {
    DynoSweepSpec s;
    s.rpm = std::move(rpm);
    return s;
}

} // namespace

TEST_CASE("G1 boost_grid_p_im: a turbo attached to the pipe I4 in code generates maps and runs on the dyno", "[engine_rules][drivetrain][cycle][turbo]") {
    // Recorded sabotage: the helper returns the grid unchanged (no rows above ambient): generation of the boosted engine is
    // not even attempted (the node check) - here the axis assertion fails first; with the axis forced by hand and
    // compressor_supply false, generate_engine_maps throws for the 160 kPa row ("above ambient").
    ComponentParams c = clean_i4();
    SimulatedEngineDesc& e = S(c);
    e.turbo_pair = hyper_turbo();
    const std::vector<real> grid = boost_grid_p_im(*e.cycle, hyper_turbo().target_boost_pa / 1e5);
    CHECK(grid == std::vector<real>({25000.0, 30000.0, 50000.0, 70000.0, 90000.0, 100000.0, 160000.0, 220000.0, 300000.0}));
    e.cycle->grid_p_im = grid;
    e.cycle->intake->compressor_supply = true; // what the loader does for a pipe engine with a turbo
    // the rules agree the engine is buildable (compression ratio 10.5 is the forced-induction ceiling)
    CHECK(tags(c) == "");
    // The same turbo with the stock axis (top 100 kPa) is refused statically
    ComponentParams stock = clean_i4();
    S(stock).turbo_pair = hyper_turbo();
    CHECK(tags(stock) == "BoostAbovePressureGrid:Exceeded@/cycle/grid_p_im_kpa");

    const auto t0 = std::chrono::steady_clock::now();
    const EngineMapsPtr maps = generate_engine_maps(*e.cycle, e.fuels.front(), kQ, &pool());
    REQUIRE(maps != nullptr);
    const DynoResult r = run_dyno_sweep(c, maps, spec_at({3000.0, 5000.0}), &pool());
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("G1 boosted pipe I4 (Sketch maps + 2 dyno points): %.2f s\n", s);
    REQUIRE(r.points.size() == 2);
    for (const DynoPoint& p : r.points) {
        std::printf("G1 %6.0f rpm: torque %.1f N*m boost %.3f bar p_intake %.1f kPa converged %d\n", p.rpm, p.torque_nm, p.boost_bar,
                    p.p_intake_kpa, p.converged ? 1 : 0);
        INFO(p.rpm);
        CHECK(std::isfinite(p.torque_nm));
        CHECK(p.torque_nm > 0.0);
        CHECK(p.boost_bar > 0.0); // the 160-300 kPa rows exist, so the manifold can rise above ambient
    }
    // Against the same engine without the turbo (stock axis): the boosted maps give more torque at 5000 rpm, which a
    // held-at-100-kPa axis could not.
    const ComponentParams na = clean_i4();
    const EngineMapsPtr na_maps = generate_engine_maps(*S(na).cycle, S(na).fuels.front(), kQ, &pool());
    const DynoResult rn = run_dyno_sweep(na, na_maps, spec_at({5000.0}), &pool());
    REQUIRE(rn.points.size() == 1);
    std::printf("G1 NA reference at 5000 rpm: %.1f N*m\n", rn.points[0].torque_nm);
    CHECK(r.points[1].torque_nm > rn.points[0].torque_nm);
}

TEST_CASE("G2 ambient_grid_p_im: the hyper with its turbo removed in code generates and runs naturally aspirated",
          "[engine_rules][drivetrain][cycle][turbo]") {
    // Recorded sabotage: ambient_grid_p_im keeps the 160 kPa node: generate_engine_maps throws for the NA engine (a node
    // above ambient needs a compressor-fed throttle) -> case fails. Engine garage P5: the same conversion through
    // the loader's forced_induction "none" slot is guarded by tests/unit/test_forced_induction_conversions.cpp FI4 (including the
    // loader's refusal of the boosted grid); this case keeps the descriptor-level view.
    ComponentParams c = clean_hyper();
    SimulatedEngineDesc& e = S(c);
    e.turbo_pair.reset();
    e.cycle->grid_p_im = ambient_grid_p_im(*e.cycle);
    e.cycle->intake->compressor_supply = false;
    CHECK(tags(c) == "");

    const auto t0 = std::chrono::steady_clock::now();
    const EngineMapsPtr maps = generate_engine_maps(*e.cycle, e.fuels.front(), kQ, &pool());
    REQUIRE(maps != nullptr);
    const DynoResult r = run_dyno_sweep(c, maps, spec_at({3000.0, 6000.0}), &pool());
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("G2 NA hyper (Sketch maps + 2 dyno points): %.2f s\n", s);
    REQUIRE(r.points.size() == 2);
    for (const DynoPoint& p : r.points) {
        INFO(p.rpm);
        CHECK(p.torque_nm > 100.0);
        CHECK(p.boost_bar == 0.0);
    }
    // naturally aspirated: well below the boosted hyper's 1371 N*m
    CHECK(r.points[1].torque_nm < 700.0);
}

TEST_CASE("G3 extend_grid_rpm: a limiter above the grid, covered by the extended axis, generates and runs there",
          "[engine_rules][drivetrain][cycle]") {
    // Recorded sabotage: the helper returns the grid unchanged: the LimiterAboveRpmGrid:Exceeded finding stays and the 7800
    // rpm dyno point is a held-row extrapolation - the first CHECK below fails.
    ComponentParams c = clean_i4();
    SimulatedEngineDesc& e = S(c);
    e.limiter.rpm = 8000.0;
    CHECK(tags(c) == "LimiterAboveRpmGrid:Exceeded@/cycle/grid_rpm");
    e.cycle->grid_rpm = extend_grid_rpm(*e.cycle, 8000.0);
    CHECK(tags(c) == "");
    CHECK(classify_engine_change(clean_i4(), c) == EngineChange::Maps);

    const auto t0 = std::chrono::steady_clock::now();
    const EngineMapsPtr maps = generate_engine_maps(*e.cycle, e.fuels.front(), kQ, &pool());
    REQUIRE(maps != nullptr);
    const DynoResult r = run_dyno_sweep(c, maps, spec_at({5000.0, 7800.0}), &pool());
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("G3 extended-rpm pipe I4 (Sketch maps + 2 dyno points): %.2f s\n", s);
    REQUIRE(r.points.size() == 2);
    for (const DynoPoint& p : r.points) {
        INFO(p.rpm);
        CHECK(std::isfinite(p.torque_nm));
        CHECK(p.torque_nm > 0.0);
    }
    // the maps' last rpm node is the limiter
    CHECK(maps->maps.ve.x.back() == 8000.0);
}
