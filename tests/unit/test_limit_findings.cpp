// tests/unit/test_limit_findings.cpp - engine garage P3b-1: the limit findings of the dyno sweep
// (ps/drivetrain/limit_findings.h, DynoResult::limits, check_driveline_limits).
//
// Every PRODUCED kind is triggered by a crafted descriptor (a loaded engine mutated in the test, no data file) and is
// absent - or pinned with its reason - on the shipped engines. The crafted turbo cases are measured numbers (the hyper
// with Sketch maps, vault PHYS-059 engine), see the case comments; the thresholds themselves live in
// core/src/drivetrain/limit_thresholds.h and are asserted tagged below. Each case lists its recorded sabotage.
//
// Kinds NOT produced in P3b-1 (asserted absent where cheap): ChargeTemperature (no limit in any data file),
// GearboxInputTorque (GearboxDesc has no rating), ValveFlowChoke, Knock (C17b), KitFuelLean / InjectorDuty /
// N2OBottleDuration / N2OBottlePressureDrop (P3b-2).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "drivetrain/engine_simulated.h"
#include "drivetrain/limit_thresholds.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {
const std::string kEngines = std::string(PS_SOURCE_DIR) + "/data/engines/";
const std::string kVehicles = std::string(PS_SOURCE_DIR) + "/data/vehicles/";

std::string cache_dir() {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "ps_garage_p3b1_maps";
    std::filesystem::create_directories(p);
    return p.string();
}
jobs::JobSystem& map_pool() {
    static jobs::JobSystem pool(10);
    return pool;
}
io::EngineMapOptions sketch_options() {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &map_pool();
    o.cache_dir = cache_dir();
    return o;
}
// The hyper (twin-turbo V8, cycle block, generated Sketch maps - cached on disk after the first load) and the shipped NA
// engines; loaded once.
const ComponentParams& hyper() {
    static const ComponentParams c = io::load_engine_json(kEngines + "hyper_v8_one1_audio.json", sketch_options());
    return c;
}
const ComponentParams& a25a() {
    static const ComponentParams c = io::load_engine_json(kEngines + "toyota_a25a_fks_sim.json");
    return c;
}
const ComponentParams& i4_torque_map() {
    static const ComponentParams c = io::load_engine_json(kEngines + "i4_2l_na.json");
    return c;
}

// Crafted hyper: air demand (bore^2 scale) and wastegate area scale; the stock hyper is (1, 1).
ComponentParams hyper_crafted(double bore_sq, double wastegate) {
    ComponentParams c = hyper();
    SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(c);
    d.piston.bore_m *= std::sqrt(bore_sq);
    d.turbo_pair->wastegate_area_m2 = d.turbo_pair->wastegate_area() * wastegate;
    return c;
}

DynoSweepSpec turbo_spec(std::vector<double> rpm) {
    DynoSweepSpec s;
    s.rpm = std::move(rpm);
    s.part_files.engine = "engine.json";
    s.part_files.turbo = "turbo.json";
    s.part_files.turbo_configuration = "turbo_config.json";
    return s;
}

// "Kind:Severity,Kind:Severity" of the findings at `rpm`, in result order (kind-sorted): exact-set comparisons print what
// differs.
std::string at_rpm(const DynoResult& r, double rpm) {
    std::string out;
    for (const LimitFinding& f : r.limits) {
        if (f.rpm != rpm) continue;
        if (!out.empty()) out += ",";
        out += std::string(to_string(f.kind)) + ":" + to_string(f.severity);
    }
    return out;
}
const LimitFinding* find(const DynoResult& r, LimitKind k, double rpm) {
    for (const LimitFinding& f : r.limits)
        if (f.kind == k && f.rpm == rpm) return &f;
    return nullptr;
}
bool any_kind(const DynoResult& r, LimitKind k) {
    return std::any_of(r.limits.begin(), r.limits.end(), [&](const LimitFinding& f) { return f.kind == k; });
}
bool same_finding(const LimitFinding& a, const LimitFinding& b) {
    return a.kind == b.kind && a.severity == b.severity && a.rpm == b.rpm && a.value == b.value && a.limit == b.limit && a.unit == b.unit &&
           a.part.slot == b.part.slot && a.part.file == b.part.file && a.part.field == b.part.field && a.remedy.has_value() == b.remedy.has_value();
}
double peak_torque(const DynoResult& r) {
    double pk = 0.0;
    for (const DynoPoint& p : r.points) pk = std::max(pk, p.torque_nm);
    return pk;
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// The tables.

TEST_CASE("L1 thresholds: every entry is sourced ([C]/[D]/[AS]/[M]), named, finite, unique", "[drivetrain][dyno]") {
    // Recorded sabotage: an entry whose source string lost its tag (kRotorNear "10 % below the table edge") fails the
    // tag check below, naming the entry.
    std::set<std::string> names;
    for (std::size_t i = 0; i < limits::kAllLimitThresholdCount; ++i) {
        const limits::Threshold& t = limits::kAllLimitThresholds[i];
        INFO("entry " << i << " " << t.name);
        REQUIRE(std::string(t.name).size() > 4);
        REQUIRE(std::isfinite(t.value));
        REQUIRE(t.value > 0.0);
        const std::string src = t.source;
        const bool tagged = src.rfind("[C]", 0) == 0 || src.rfind("[D]", 0) == 0 || src.rfind("[AS]", 0) == 0 || src.rfind("[M]", 0) == 0;
        CHECK(tagged);
        CHECK(src.size() > 8); // a tag alone is not a source
        CHECK(names.insert(t.name).second);
    }
    CHECK(limits::kAllLimitThresholdCount == 34); // P7: + kKnockNear, kKnockLimiting; coupling: + the three hitch thresholds
    // The ordered pairs a severity ladder depends on.
    CHECK(limits::kGateNear.value < limits::kGateFull.value);
    CHECK(limits::kBoostBandNear.value < limits::kBoostBandExceeded.value);
    CHECK(limits::kRotorNear.value < limits::kRotorLimiting.value);
    CHECK(limits::kPistonSpeedNear.value < limits::kPistonSpeedLimiting.value);
    CHECK(limits::kPistonSpeedLimiting.value < limits::kPistonSpeedExceeded.value);
    CHECK(limits::kChokeNear.value < limits::kChokeLimiting.value);
}

TEST_CASE("L2 kinds, severities, units: stable values and symbols", "[drivetrain][dyno]") {
    // The values are stored by games: append-only. Pinned literally. Recorded sabotage: BoostAboveTarget = 1 and
    // BoostBelowTarget = 2 (swapped) fails the literal pins (a duplicate value fails to compile in to_string).
    CHECK(static_cast<int>(LimitKind::WastegateSaturated) == 0);
    CHECK(static_cast<int>(LimitKind::BoostBelowTarget) == 1);
    CHECK(static_cast<int>(LimitKind::BoostAboveTarget) == 2);
    CHECK(static_cast<int>(LimitKind::TurboRotorCap) == 3);
    CHECK(static_cast<int>(LimitKind::CompressorSurge) == 4);
    CHECK(static_cast<int>(LimitKind::CompressorChoke) == 5);
    CHECK(static_cast<int>(LimitKind::ChargeTemperature) == 6);
    CHECK(static_cast<int>(LimitKind::ClutchCapacity) == 7);
    CHECK(static_cast<int>(LimitKind::GearboxInputTorque) == 8);
    CHECK(static_cast<int>(LimitKind::KitFuelLean) == 9);
    CHECK(static_cast<int>(LimitKind::InjectorDuty) == 10);
    CHECK(static_cast<int>(LimitKind::N2OBottleDuration) == 11);
    CHECK(static_cast<int>(LimitKind::N2OBottlePressureDrop) == 12);
    CHECK(static_cast<int>(LimitKind::RevLimiter) == 13);
    CHECK(static_cast<int>(LimitKind::FuelCut) == 14);
    CHECK(static_cast<int>(LimitKind::ValveFlowChoke) == 15);
    CHECK(static_cast<int>(LimitKind::MeanPistonSpeed) == 16);
    CHECK(static_cast<int>(LimitKind::MapGridEdge) == 17);
    CHECK(static_cast<int>(LimitKind::Knock) == 18);
    std::set<std::string> kinds;
    for (int k = 0; k <= 18; ++k) {
        const std::string s = to_string(static_cast<LimitKind>(k));
        CHECK(s != "Unknown");
        CHECK(kinds.insert(s).second);
    }
    CHECK(std::string(to_string(static_cast<LimitKind>(99))) == "Unknown");
    CHECK(std::string(to_string(LimitSeverity::Near)) == "Near");
    CHECK(LimitSeverity::Near < LimitSeverity::Limiting);
    CHECK(LimitSeverity::Limiting < LimitSeverity::Exceeded);
    std::set<std::string> units;
    for (int u = 0; u <= 11; ++u) CHECK(units.insert(to_string(static_cast<LimitUnit>(u))).second);
    CHECK(std::string(to_string(LimitUnit::NewtonMetre)) == "N*m");
    CHECK(std::string(to_string(LimitUnit::RevPerMinute)) == "rpm");
    Remedy r;
    r.current = 100.0;
    r.required_min = 150.0;
    r.required_margin = 195.0;
    CHECK(r.ratio_min() == 1.5);
    CHECK(r.ratio_margin() == 1.95);
}

// ---------------------------------------------------------------------------------------------------------------------
// The shipped engines. What they legitimately hit is pinned WITH the reason; anything else is a regression.

TEST_CASE("L3 stock hyper: only its legitimate findings", "[drivetrain][dyno][turbo]") {
    // Measured 2026-10-11 (Sketch maps):
    //  - 2000 rpm: BoostBelowTarget Exceeded - the spool region. A twin-turbo V8 at 2000 rpm and wide open has too little
    //    exhaust enthalpy for the 1.8 bar target (charge +0.21 bar); the stock engine is below target up to ~2200 rpm.
    //  - 8000 rpm: MeanPistonSpeed Limiting - 95.25 mm stroke x 8000 rpm / 30 = 25.4 m/s, over the 25 m/s table entry.
    //  - 8250 rpm: the limiter point itself: FuelCut Limiting (the rpm equals the 8250 limiter) and MeanPistonSpeed.
    //  - 5000 rpm: nothing. In particular no WastegateSaturated (gate well inside its authority), no TurboRotorCap (rotor
    //    145k of the 180k table edge), no choke, no surge (the stock file has no surge_line), no MapGridEdge.
    // Recorded sabotage: kGateFull 0.98 -> 0.0 reports WastegateSaturated at every point; kPistonSpeedLimiting 25 -> 26
    // loses the 8000 rpm MeanPistonSpeed pin.
    static jobs::JobSystem pool(10);
    const DynoResult r = run_dyno_sweep(hyper(), nullptr, turbo_spec({2000.0, 5000.0, 8000.0, 8250.0}), &pool);
    CHECK(at_rpm(r, 2000.0) == "BoostBelowTarget:Exceeded");
    CHECK(at_rpm(r, 5000.0).empty());
    CHECK(at_rpm(r, 8000.0) == "MeanPistonSpeed:Limiting");
    CHECK(at_rpm(r, 8250.0) == "FuelCut:Limiting,MeanPistonSpeed:Limiting");
    for (LimitKind k : {LimitKind::WastegateSaturated, LimitKind::BoostAboveTarget, LimitKind::TurboRotorCap, LimitKind::CompressorSurge,
                        LimitKind::CompressorChoke, LimitKind::MapGridEdge, LimitKind::ChargeTemperature, LimitKind::RevLimiter}) {
        INFO(to_string(k));
        CHECK_FALSE(any_kind(r, k));
    }
    const LimitFinding* below = find(r, LimitKind::BoostBelowTarget, 2000.0);
    REQUIRE(below != nullptr);
    CHECK(below->unit == LimitUnit::Pascal);
    CHECK(below->value < 0.5 * below->limit); // 21.3 kPa against 180 kPa
    CHECK(below->part.slot == "turbo");
    const LimitFinding* cut = find(r, LimitKind::FuelCut, 8250.0);
    REQUIRE(cut != nullptr);
    CHECK(cut->part.slot == "engine");
    CHECK(cut->part.file == "engine.json");
    CHECK(cut->part.field == "/limiter/rpm");
    CHECK(cut->value == 8250.0);
    CHECK(cut->limit == 8250.0);
    // The stock hyper has no wastegate or rotor finding, so no Simulated placeholder; the other P3b-1 kinds carry no remedy.
    for (const LimitFinding& f : r.limits) CHECK_FALSE(f.remedy.has_value());
}

TEST_CASE("L4 stock NA and torque-map engines: legitimate findings only", "[drivetrain][dyno]") {
    // A25A-FKS sim (measured grid): its ve table's p_im axis ends at 97.5 kPa, BELOW the 101.3 kPa ambient a wide-open NA
    // engine breathes at, so every WOT point reads a clamped column - a real hole in the shipped engine's data
    // (MapGridEdge Exceeded on the p axis at every point, field "/maps/ve/p_im_kpa"); its rpm axis ends at 5761.2, so 6500
    // is also clamped in rpm; and 6500 is its limiter (FuelCut Limiting). 22.4 m/s at 6500 rpm is below the piston-speed
    // table, so no MeanPistonSpeed.
    // Torque map i4_2l_na: no clamping notion, no piston geometry; only the limiter point (6800) reports.
    // Recorded sabotage: emit_edge_axis ignoring the `above` count loses every MapGridEdge pin here.
    DynoSweepSpec s;
    s.rpm = {3000.0, 5000.0, 6500.0};
    s.part_files.engine = "a25a.json";
    const DynoResult na = run_dyno_sweep(a25a(), nullptr, s, nullptr);
    CHECK(at_rpm(na, 3000.0) == "MapGridEdge:Exceeded");
    CHECK(at_rpm(na, 5000.0) == "MapGridEdge:Exceeded");
    CHECK(at_rpm(na, 6500.0) == "FuelCut:Limiting,MapGridEdge:Exceeded,MapGridEdge:Exceeded");
    const LimitFinding* e = find(na, LimitKind::MapGridEdge, 3000.0);
    REQUIRE(e != nullptr);
    CHECK(e->unit == LimitUnit::Pascal);
    CHECK(e->value > e->limit);
    CHECK(e->part.file == "a25a.json");
    CHECK(e->part.field == "/maps/ve/p_im_kpa");
    // The two axes of one point: p_im first (emission order), then rpm.
    std::vector<const LimitFinding*> edges;
    for (const LimitFinding& f : na.limits)
        if (f.kind == LimitKind::MapGridEdge && f.rpm == 6500.0) edges.push_back(&f);
    REQUIRE(edges.size() == 2);
    CHECK(edges[0]->part.field == "/maps/ve/p_im_kpa");
    CHECK(edges[1]->part.field == "/maps/ve/rpm");
    CHECK(edges[1]->unit == LimitUnit::RevPerMinute);
    CHECK(edges[1]->value == 6500.0);
    CHECK(edges[1]->limit < 6000.0);

    const DynoResult tm = run_dyno_sweep(i4_torque_map(), nullptr, DynoSweepSpec{}, nullptr);
    REQUIRE(tm.points.size() == 25);
    REQUIRE(tm.limits.size() == 1);
    CHECK(tm.limits[0].kind == LimitKind::FuelCut);
    CHECK(tm.limits[0].severity == LimitSeverity::Limiting);
    CHECK(tm.limits[0].rpm == 6800.0);
    CHECK(tm.point_limits.size() == 25);
    for (std::size_t i = 0; i + 1 < tm.points.size(); ++i) CHECK(tm.point_limits[i].count == 0);
    CHECK(tm.point_limits.back().count == 1);
    CHECK(tm.point_limits.back().first == 0);
}

TEST_CASE("L5 a torque-map engine below its limiter has no findings", "[drivetrain][dyno]") {
    DynoSweepSpec s;
    s.rpm = {800.0, 2500.0, 4550.0, 6000.0};
    const DynoResult tm = run_dyno_sweep(i4_torque_map(), nullptr, s, nullptr);
    CHECK(tm.limits.empty());
    REQUIRE(tm.point_limits.size() == 4);
    for (const LimitRange& g : tm.point_limits) CHECK(g.count == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// Turbo kinds, each from a crafted descriptor.

TEST_CASE("L6 wastegate: owner's example - stock hardware saturates under more air, the larger wastegate does not",
          "[drivetrain][dyno][turbo]") {
    // Air demand x2.0 (bore^2) at 7000 rpm wide open. With the stock wastegate (2.36e-3 m^2) the gate runs fully open and
    // the boost still overshoots the 1.8 bar target by 33 % (238.7 kPa gauge), the rotor leaves the table (328k of 180k).
    // hyper_turbo_n2o.json's wastegate is 1.26x the stock (1.4875e-3 vs 1.18e-3 m^2 per valve): with it the gate holds the
    // target and WastegateSaturated / BoostAboveTarget are gone (the rotor and flow findings stay: they are other
    // hardware). The diagnostic points are not converged (the stock one runs away) - findings come from the last window.
    // Recorded sabotage (2026-10-11): kGateFull 0.98 -> 1.5 (saturation never detected) fails the first block.
    static jobs::JobSystem pool(10);
    const DynoResult stock = run_dyno_sweep(hyper_crafted(2.0, 1.0), nullptr, turbo_spec({7000.0}), &pool);
    const LimitFinding* sat = find(stock, LimitKind::WastegateSaturated, 7000.0);
    REQUIRE(sat != nullptr);
    CHECK(sat->severity == LimitSeverity::Exceeded);
    CHECK(sat->unit == LimitUnit::Fraction);
    CHECK(sat->value >= 0.98);
    CHECK(sat->limit == 0.98);
    CHECK(sat->part.slot == "turbo");
    CHECK(sat->part.file == "turbo.json");
    CHECK(sat->part.field == "/wastegate_area_m2");
    const LimitFinding* above = find(stock, LimitKind::BoostAboveTarget, 7000.0);
    REQUIRE(above != nullptr);
    CHECK(above->severity == LimitSeverity::Exceeded);
    CHECK(above->value > 1.05 * above->limit); // charge pressure over the target by more than 5 % of the target
    CHECK(above->unit == LimitUnit::Pascal);
    const LimitFinding* rotor = find(stock, LimitKind::TurboRotorCap, 7000.0);
    REQUIRE(rotor != nullptr);
    CHECK(rotor->severity == LimitSeverity::Exceeded);

    const DynoResult big = run_dyno_sweep(hyper_crafted(2.0, 1.26), nullptr, turbo_spec({7000.0}), &pool);
    CHECK(find(big, LimitKind::WastegateSaturated, 7000.0) == nullptr);
    CHECK(find(big, LimitKind::BoostAboveTarget, 7000.0) == nullptr);
    CHECK(find(big, LimitKind::TurboRotorCap, 7000.0) != nullptr); // other hardware, still reported
}

TEST_CASE("L7 wastegate Near, rotor and choke severities (air demand x1.6)", "[drivetrain][dyno][turbo]") {
    // Air demand x1.6 at 7000 rpm: gate mean opening 0.932 (Near: 0.92 <= open < 0.98), the rotor at 230.7k of 180k
    // (Exceeded), pair flow 1.21 x the compressor's max_mass_flow (CompressorChoke Limiting). At wastegate x1.26 the gate
    // finding is gone. Recorded sabotage: kGateNear 0.92 -> 0.99 loses the Near; kChokeLimiting 1.0 -> 1.5 turns the
    // Limiting into Near/none.
    static jobs::JobSystem pool(10);
    const DynoResult r = run_dyno_sweep(hyper_crafted(1.6, 1.0), nullptr, turbo_spec({7000.0}), &pool);
    const LimitFinding* g = find(r, LimitKind::WastegateSaturated, 7000.0);
    REQUIRE(g != nullptr);
    CHECK(g->severity == LimitSeverity::Near);
    CHECK(g->value >= 0.92);
    CHECK(g->value < 0.98);
    const LimitFinding* rotor = find(r, LimitKind::TurboRotorCap, 7000.0);
    REQUIRE(rotor != nullptr);
    CHECK(rotor->severity == LimitSeverity::Exceeded);
    CHECK(rotor->limit == 180000.0); // 1.5 x reference_rpm 120000 (head table edge)
    CHECK(rotor->value > rotor->limit);
    CHECK(rotor->unit == LimitUnit::RevPerMinute);
    const LimitFinding* choke = find(r, LimitKind::CompressorChoke, 7000.0);
    REQUIRE(choke != nullptr);
    CHECK(choke->severity == LimitSeverity::Limiting);
    CHECK(choke->value >= 1.0);
    CHECK(choke->part.field == "/max_mass_flow_kg_s");
    CHECK(find(r, LimitKind::BoostAboveTarget, 7000.0) == nullptr); // the gate still holds the target
    CHECK(find(r, LimitKind::BoostBelowTarget, 7000.0) == nullptr);
}

TEST_CASE("L8 rotor cap and choke: Near below the edge, Exceeded beyond (air demand x1.05 / x1.10 at 8000 rpm)",
          "[drivetrain][dyno][turbo]") {
    // Stock hyper 8000 rpm: rotor 144.3k = 0.80 of the 180k edge (no finding), choke fraction 0.84. x1.10 air: rotor 183.3k
    // (Exceeded by 1.9 %), choke 0.927 (Near). x1.05 sits between: rotor in [0.90, 1.0) of the edge (Near or Limiting).
    // Recorded sabotage: kRotorNear 0.90 -> 0.99 and kChokeNear 0.90 -> 0.99 lose the in-between findings.
    static jobs::JobSystem pool(10);
    const DynoResult hi = run_dyno_sweep(hyper_crafted(1.10, 1.0), nullptr, turbo_spec({8000.0}), &pool);
    const LimitFinding* rotor = find(hi, LimitKind::TurboRotorCap, 8000.0);
    REQUIRE(rotor != nullptr);
    CHECK(rotor->severity == LimitSeverity::Exceeded);
    const LimitFinding* choke = find(hi, LimitKind::CompressorChoke, 8000.0);
    REQUIRE(choke != nullptr);
    CHECK(choke->severity == LimitSeverity::Near);
    CHECK(choke->value >= 0.90);
    CHECK(choke->value < 1.0);

    const DynoResult mid = run_dyno_sweep(hyper_crafted(1.05, 1.0), nullptr, turbo_spec({8000.0}), &pool);
    const LimitFinding* r2 = find(mid, LimitKind::TurboRotorCap, 8000.0);
    REQUIRE(r2 != nullptr);
    CHECK(r2->severity < LimitSeverity::Exceeded);
    CHECK(r2->value / r2->limit >= 0.90);
    CHECK(r2->value / r2->limit <= 1.0);
}

TEST_CASE("L9 compressor choke from a small flow capacity", "[drivetrain][dyno][turbo]") {
    // max_mass_flow_kg_s x0.5 (the compressor flows half the air): pair flow 1.107 x capacity at 5000 rpm - Limiting; the
    // rotor runs away to 213k to push the same boost through the small compressor. Absent on the stock hyper (L3).
    static jobs::JobSystem pool(10);
    ComponentParams c = hyper();
    std::get<SimulatedEngineDesc>(c).turbo_pair->max_mass_flow_kg_s *= 0.5;
    const DynoResult r = run_dyno_sweep(c, nullptr, turbo_spec({5000.0}), &pool);
    const LimitFinding* choke = find(r, LimitKind::CompressorChoke, 5000.0);
    REQUIRE(choke != nullptr);
    CHECK(choke->severity == LimitSeverity::Limiting);
    CHECK(choke->value > 1.0);
    CHECK(choke->limit == 1.0);
}

TEST_CASE("L10 boost below target: a target the turbo cannot reach", "[drivetrain][dyno][turbo]") {
    // The stock hyper pins BoostBelowTarget at 2000 rpm in L3 (the spool region). Here the same hardware gets a 3x larger
    // target (540 kPa gauge): at 2000 rpm the charge is still ~21 kPa, far below, and the limit reported is the new
    // target (the finding's `limit` is target x plate, the part is the turbo hardware). The rotor/gate have no authority
    // problem at 5000 rpm and a x1.6 target there is reached (charge +2.8 bar, no BoostBelowTarget).
    // Recorded sabotage: swapping the sign of `over` in turbo_window_findings swaps Above/Below and fails both blocks.
    static jobs::JobSystem pool(10);
    ComponentParams c = hyper();
    std::get<SimulatedEngineDesc>(c).turbo_pair->target_boost_pa *= 3.0;
    const DynoResult r = run_dyno_sweep(c, nullptr, turbo_spec({2000.0}), &pool);
    const LimitFinding* f = find(r, LimitKind::BoostBelowTarget, 2000.0);
    REQUIRE(f != nullptr);
    CHECK(f->severity == LimitSeverity::Exceeded);
    CHECK(f->value < f->limit);
    CHECK(f->limit == Catch::Approx(std::get<SimulatedEngineDesc>(c).turbo_pair->target_boost_pa).epsilon(1e-6));
    CHECK_FALSE(any_kind(r, LimitKind::BoostAboveTarget));

    ComponentParams c2 = hyper();
    std::get<SimulatedEngineDesc>(c2).turbo_pair->target_boost_pa *= 1.6;
    const DynoResult r2 = run_dyno_sweep(c2, nullptr, turbo_spec({5000.0}), &pool);
    CHECK_FALSE(any_kind(r2, LimitKind::BoostBelowTarget));
    CHECK_FALSE(any_kind(r2, LimitKind::BoostAboveTarget));
}

TEST_CASE("L11 compressor surge: needs a surge line; Exceeded / Near / none follow the flow margin", "[drivetrain][dyno][turbo]") {
    // The stock turbo has NO surge_line, so the kind is never produced (L3). A flat surge line at flow fraction y: the
    // finding's value is the window minimum of (flow fraction / y) while the pressure ratio is above 1.05. Step 1: y = 0.99
    // (a line the stock flow at 5000 rpm is below) -> Exceeded, value < 1. The surge flag does not feed the physics, so
    // the flow is the same for every y and y' = flow/1.05 gives a margin of 1.05 (Near, < 1.10 table value) and
    // y'' = flow/1.5 gives none.
    // Recorded sabotage: kSurgeNearMargin 1.10 -> 1.00 loses the Near; kSurgeMinFraction 0.02 -> 2.0 loses the Exceeded.
    static jobs::JobSystem pool(10);
    const auto run_with_line = [&](double y) {
        ComponentParams c = hyper();
        SurgeLineDesc line;
        line.flow_fraction_vs_pr.x = {1.0, 10.0};
        line.flow_fraction_vs_pr.y = {y, y};
        line.source = "test: flat line";
        std::get<SimulatedEngineDesc>(c).turbo_pair->surge_line = line;
        return run_dyno_sweep(c, nullptr, turbo_spec({5000.0}), &pool);
    };
    const DynoResult none = run_dyno_sweep(hyper(), nullptr, turbo_spec({5000.0}), &pool);
    CHECK_FALSE(any_kind(none, LimitKind::CompressorSurge));

    const DynoResult surge = run_with_line(0.99);
    const LimitFinding* s = find(surge, LimitKind::CompressorSurge, 5000.0);
    REQUIRE(s != nullptr);
    CHECK(s->severity == LimitSeverity::Exceeded);
    CHECK(s->value < 1.0);
    CHECK(s->limit == 1.0);
    CHECK(s->part.slot == "turbo");
    CHECK(s->part.field == "/surge_line");
    const double flow_fraction = s->value * 0.99; // min over the window of flow / max_mass_flow

    const DynoResult near = run_with_line(flow_fraction / 1.05);
    const LimitFinding* n = find(near, LimitKind::CompressorSurge, 5000.0);
    REQUIRE(n != nullptr);
    CHECK(n->severity == LimitSeverity::Near);
    CHECK(n->value > 1.0);
    CHECK(n->value < 1.10);
    CHECK(n->value == std::abs(n->value));

    const DynoResult clear = run_with_line(flow_fraction / 1.5);
    CHECK_FALSE(any_kind(clear, LimitKind::CompressorSurge));
}

// ---------------------------------------------------------------------------------------------------------------------
// Instant kinds: limiter, piston speed, grid edge.

TEST_CASE("L12 rev limiter (SoftCut taper) and fuel cut severities", "[drivetrain][dyno]") {
    // SoftCut: zero fuel at lim.rpm, linear taper over band: Near over the two bands below the taper, Limiting inside the
    // taper, Exceeded at/above the zero-fuel rpm. FuelCut: Near from 97 % of the cut rpm, Limiting at it, Exceeded above.
    // Recorded sabotage: the SoftCut Limiting test `rpm > lim.rpm - band` -> `rpm > lim.rpm` loses the 6400 pin;
    // kLimiterNear 0.97 -> 0.5 makes 5000 rpm Near (also fails L5).
    ComponentParams c = a25a();
    SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(c);
    d.limiter.type = LimiterType::SoftCut;
    d.limiter.rpm = 6500.0;
    d.limiter.band_rpm = 200.0;
    DynoSweepSpec s;
    s.rpm = {5000.0, 6200.0, 6400.0, 6500.0, 6600.0};
    const DynoResult soft = run_dyno_sweep(c, nullptr, s, nullptr);
    CHECK(at_rpm(soft, 5000.0).find("RevLimiter") == std::string::npos);
    CHECK(at_rpm(soft, 6200.0).find("RevLimiter:Near") != std::string::npos);
    CHECK(at_rpm(soft, 6400.0).find("RevLimiter:Limiting") != std::string::npos);
    CHECK(at_rpm(soft, 6500.0).find("RevLimiter:Exceeded") != std::string::npos);
    CHECK(at_rpm(soft, 6600.0).find("RevLimiter:Exceeded") != std::string::npos);
    CHECK_FALSE(any_kind(soft, LimitKind::FuelCut));
    const LimitFinding* f = find(soft, LimitKind::RevLimiter, 6400.0);
    REQUIRE(f != nullptr);
    CHECK(f->value == 6400.0);
    CHECK(f->limit == 6500.0);
    CHECK(f->part.field == "/limiter/rpm");

    // FuelCut on the shipped A25A (limiter 6500): 6400 = 98.5 % -> Near; 6500 Limiting; 6600 Exceeded; 5000 none.
    const DynoResult cut = run_dyno_sweep(a25a(), nullptr, s, nullptr);
    CHECK(at_rpm(cut, 5000.0).find("FuelCut") == std::string::npos);
    CHECK(at_rpm(cut, 6200.0).find("FuelCut") == std::string::npos); // 95.4 %
    CHECK(at_rpm(cut, 6400.0).find("FuelCut:Near") != std::string::npos);
    CHECK(at_rpm(cut, 6500.0).find("FuelCut:Limiting") != std::string::npos);
    CHECK(at_rpm(cut, 6600.0).find("FuelCut:Exceeded") != std::string::npos);
    CHECK_FALSE(any_kind(cut, LimitKind::RevLimiter));
}

TEST_CASE("L13 mean piston speed ladder (stroke scaled on the A25A at 6500 rpm)", "[drivetrain][dyno]") {
    // 22.4 m/s at the stock 103.4 mm stroke: nothing (L4). Stroke x1.05 -> 23.5 Near, x1.20 -> 26.9 Limiting, x1.35 -> 30.2
    // Exceeded. A torque-map engine has no geometry: never produced.
    // Recorded sabotage: kPistonSpeedExceeded 28 -> 40 turns the x1.35 case into Limiting.
    const auto run = [&](double stroke_scale) {
        ComponentParams c = a25a();
        std::get<SimulatedEngineDesc>(c).piston.stroke_m *= stroke_scale;
        DynoSweepSpec s;
        s.rpm = {6500.0};
        return run_dyno_sweep(c, nullptr, s, nullptr);
    };
    const auto piston = [&](const DynoResult& r) -> const LimitFinding* { return find(r, LimitKind::MeanPistonSpeed, 6500.0); };
    CHECK(piston(run(1.0)) == nullptr);
    const LimitFinding* near = nullptr;
    const DynoResult r105 = run(1.05);
    near = piston(r105);
    REQUIRE(near != nullptr);
    CHECK(near->severity == LimitSeverity::Near);
    CHECK(near->unit == LimitUnit::MetrePerSecond);
    CHECK(near->value > 23.0);
    const DynoResult r120 = run(1.20);
    REQUIRE(piston(r120) != nullptr);
    CHECK(piston(r120)->severity == LimitSeverity::Limiting);
    const DynoResult r135 = run(1.35);
    REQUIRE(piston(r135) != nullptr);
    CHECK(piston(r135)->severity == LimitSeverity::Exceeded);
    CHECK(piston(r135)->value == std::get<SimulatedEngineDesc>(a25a()).piston.stroke_m * 1.35 * 6500.0 / 30.0);
    CHECK(piston(r135)->part.field == "/limiter/rpm");

    DynoSweepSpec s;
    s.rpm = {6800.0};
    const DynoResult tm = run_dyno_sweep(i4_torque_map(), nullptr, s, nullptr);
    CHECK_FALSE(any_kind(tm, LimitKind::MeanPistonSpeed));
}

TEST_CASE("L14 map grid edge: Exceeded above the grid, Exceeded below it, Near the top, field names the grid file",
          "[drivetrain][dyno]") {
    // A25A: its p axis ends below ambient (L4). Scaling the ve table's p axis up by 4 % puts the top just ABOVE the wide-open
    // intake pressure (100.6 kPa at 3000 rpm vs 101.4 kPa): within 2 % of the span of the top -> Near, not clamped. Scaled
    // 10 %: far above, nothing. rpm below the grid (500 < the lowest knot) is a clamp on the lower edge: Exceeded.
    // Recorded sabotage: passing near_upper = false for the p axis loses the Near; requiring the clamped share >= 2.0
    // (never) loses every Exceeded here and in L4.
    const auto with_ve_p_scale = [&](double k) {
        ComponentParams c = a25a();
        for (real& y : std::get<SimulatedEngineDesc>(c).maps.ve.y) y *= k;
        return c;
    };
    DynoSweepSpec s;
    s.rpm = {3000.0};
    s.part_files.engine = "a25a.json";
    const DynoResult near = run_dyno_sweep(with_ve_p_scale(1.04), nullptr, s, nullptr);
    const LimitFinding* n = find(near, LimitKind::MapGridEdge, 3000.0);
    REQUIRE(n != nullptr);
    CHECK(n->severity == LimitSeverity::Near);
    CHECK(n->value <= n->limit);
    CHECK(n->part.field == "/maps/ve/p_im_kpa");
    const DynoResult clear = run_dyno_sweep(with_ve_p_scale(1.10), nullptr, s, nullptr);
    CHECK_FALSE(any_kind(clear, LimitKind::MapGridEdge));

    DynoSweepSpec low;
    low.rpm = {500.0};
    const DynoResult below = run_dyno_sweep(a25a(), nullptr, low, nullptr);
    bool rpm_low = false;
    for (const LimitFinding& f : below.limits)
        if (f.kind == LimitKind::MapGridEdge && f.unit == LimitUnit::RevPerMinute) {
            rpm_low = true;
            CHECK(f.severity == LimitSeverity::Exceeded);
            CHECK(f.value == 500.0);
            CHECK(f.limit > 500.0);
        }
    CHECK(rpm_low);
}

TEST_CASE("L15 generated engine: the grid fields point into the cycle block", "[drivetrain][dyno][turbo]") {
    // A generated engine (cycle block) owns its grid in "/cycle/grid_rpm" and "/cycle/grid_p_im_kpa", not in a maps block.
    // The crafted x1.6 target puts the manifold above the hyper's 300 kPa grid top at 3000 rpm (measured 387 kPa).
    // Recorded sabotage: using the "/maps/ve/..." names for every engine fails the field check.
    static jobs::JobSystem pool(10);
    ComponentParams c = hyper();
    std::get<SimulatedEngineDesc>(c).turbo_pair->target_boost_pa *= 1.6;
    const DynoResult r = run_dyno_sweep(c, nullptr, turbo_spec({3000.0}), &pool);
    const LimitFinding* e = find(r, LimitKind::MapGridEdge, 3000.0);
    REQUIRE(e != nullptr);
    CHECK(e->severity == LimitSeverity::Exceeded);
    CHECK(e->part.field == "/cycle/grid_p_im_kpa");
    CHECK(e->part.file == "engine.json");
    CHECK(e->value > e->limit);
}

// ---------------------------------------------------------------------------------------------------------------------
// Structure: ordering, ranges, determinism.

TEST_CASE("L16 limits are sorted by rpm then kind; point_limits address them; duplicate rpm shares one block", "[drivetrain][dyno][turbo]") {
    // rpm given out of order and with a duplicate (7000 twice): points keep the spec order, limits are rpm-ascending.
    // Recorded sabotage: sorting the points by index instead of rpm in assemble_limits fails the sortedness check.
    static jobs::JobSystem pool(10);
    DynoSweepSpec s = turbo_spec({8000.0, 5000.0, 7000.0, 7000.0});
    s.max_settle_s = 4.0;
    const DynoResult r = run_dyno_sweep(hyper_crafted(1.6, 1.0), nullptr, s, &pool);
    REQUIRE(r.points.size() == 4);
    REQUIRE(r.point_limits.size() == 4);
    REQUIRE(r.limits.size() >= 4);
    for (std::size_t i = 1; i < r.limits.size(); ++i) {
        const LimitFinding& a = r.limits[i - 1];
        const LimitFinding& b = r.limits[i];
        INFO("index " << i);
        CHECK((a.rpm < b.rpm || (a.rpm == b.rpm && a.kind <= b.kind)));
    }
    std::size_t covered = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const LimitRange g = r.point_limits[i];
        REQUIRE(g.first + g.count <= r.limits.size());
        for (std::uint32_t k = 0; k < g.count; ++k) CHECK(r.limits[g.first + k].rpm == r.points[i].rpm);
    }
    CHECK(r.point_limits[2].first == r.point_limits[3].first);
    CHECK(r.point_limits[2].count == r.point_limits[3].count);
    CHECK(r.point_limits[2].count > 0);
    // Every finding belongs to exactly one distinct block: unique ranges tile `limits`.
    std::set<std::uint32_t> firsts;
    for (std::size_t i = 0; i < 4; ++i)
        if (r.point_limits[i].count > 0 && firsts.insert(r.point_limits[i].first).second) covered += r.point_limits[i].count;
    CHECK(covered == r.limits.size());
    // 7000 findings are not doubled.
    std::size_t at7000 = 0;
    for (const LimitFinding& f : r.limits) at7000 += f.rpm == 7000.0 ? 1 : 0;
    CHECK(at7000 == r.point_limits[2].count);
}

TEST_CASE("L17 findings are identical for 1 and 10 workers and without a pool", "[drivetrain][dyno][turbo]") {
    // The windows are per point (own engine instance), so the result must not depend on the schedule.
    // Sabotage tried (2026-10-11): skipping the per-point kind sort only on the pooled path did NOT fail - the turbo
    // producers already emit in kind order - so this case guards by construction (one slot per point), not by a
    // discriminating sabotage. Open point in out/garage_p3b1_progress.md.
    jobs::JobSystem one(1);
    jobs::JobSystem ten(10);
    DynoSweepSpec s = turbo_spec({3000.0, 5000.0, 7000.0, 8000.0});
    s.max_settle_s = 4.0;
    const ComponentParams c = hyper_crafted(1.6, 1.0);
    const DynoResult a = run_dyno_sweep(c, nullptr, s, nullptr);
    const DynoResult b = run_dyno_sweep(c, nullptr, s, &one);
    const DynoResult d = run_dyno_sweep(c, nullptr, s, &ten);
    REQUIRE(a.limits.size() == b.limits.size());
    REQUIRE(a.limits.size() == d.limits.size());
    CHECK(a.limits.size() >= 4);
    for (std::size_t i = 0; i < a.limits.size(); ++i) {
        INFO("finding " << i << " " << to_string(a.limits[i].kind));
        CHECK(same_finding(a.limits[i], b.limits[i]));
        CHECK(same_finding(a.limits[i], d.limits[i]));
    }
    for (std::size_t i = 0; i < a.point_limits.size(); ++i) {
        CHECK(a.point_limits[i].first == d.point_limits[i].first);
        CHECK(a.point_limits[i].count == d.point_limits[i].count);
    }
}

TEST_CASE("L18 the sweep's points are untouched by the findings (stock hyper values pinned in test_engine_dyno)", "[drivetrain][dyno][turbo]") {
    // The findings are derived from the same windows; the DynoPoint values are those of the P3 sweep. Pinned here against the
    // numbers measured in this increment's diagnostics (Sketch maps): a producer that fed back into the engine would move
    // them. test_engine_dyno.cpp D1-D8 hold the full contract. Recorded sabotage: net_torque += tau0 - 0.5 c_k fails here.
    static jobs::JobSystem pool(10);
    const DynoResult r = run_dyno_sweep(hyper(), nullptr, turbo_spec({5000.0}), &pool);
    REQUIRE(r.points.size() == 1);
    CHECK(r.points[0].converged);
    // Re-pinned 2026-10-19 for the hyper on E85 with knock model v3, unchanged under v4 (was 1378.5 N*m / 91184 rpm on E5 without a knock limit).
    CHECK(r.points[0].torque_nm == Catch::Approx(1369.06).margin(0.1));
    CHECK(r.points[0].boost_bar == Catch::Approx(1.759).margin(0.001));
    CHECK(r.points[0].turbo_rpm == Catch::Approx(91172.9).margin(2.0));
}

// ---------------------------------------------------------------------------------------------------------------------
// Driveline.

TEST_CASE("L19 check_driveline_limits: shipped vehicles hold their engine's peak with margin", "[drivetrain][dyno][io]") {
    // car_hyper: clutch 1650 N*m kinetic x 1.15 static = 1897.5 against a peak of 1378 (Sketch maps; 1371 calibrated at
    // 6000 rpm): 1.3 x peak = 1791 < 1897.5, no finding. car_sedan: 300 x 1.15 = 345 against the i4's 199.8 peak: none.
    // Recorded sabotage: kDrivelineMargin 1.3 -> 1.5 reports Near on the hyper (1.5 x 1378 = 2067 > 1897.5).
    static jobs::JobSystem pool(10);
    const DynoResult hr = run_dyno_sweep(hyper(), nullptr, turbo_spec({5000.0, 6000.0, 7000.0}), &pool);
    const vehicle::VehicleDesc hv = io::load_vehicle_json(kVehicles + "car_hyper.json", sketch_options());
    bool has_clutch = false;
    for (const ComponentDesc& c : hv.powertrain.components)
        if (const auto* cl = std::get_if<ClutchDesc>(&c.params)) {
            has_clutch = true;
            CHECK(cl->capacity_nm == 1650.0);
            CHECK(cl->static_factor == 1.15);
        }
    REQUIRE(has_clutch);
    CHECK(peak_torque(hr) > 1300.0);
    CHECK(check_driveline_limits(hr, hv.powertrain, "clutch.json").empty());

    const DynoResult tm = run_dyno_sweep(i4_torque_map(), nullptr, DynoSweepSpec{}, nullptr);
    CHECK(peak_torque(tm) == Catch::Approx(199.8).margin(0.05));
    const vehicle::VehicleDesc sv = io::load_vehicle_json(kVehicles + "car_sedan.json");
    CHECK(check_driveline_limits(tm, sv.powertrain).empty());
}

TEST_CASE("L20 check_driveline_limits: an undersized clutch gets a Closed remedy (peak x 1.3)", "[drivetrain][dyno][io]") {
    // car_sedan's powertrain with the clutch capacity replaced. i4 peak 199.8 N*m at 4550 rpm.
    //  - static_factor 1.0, capacity 150: Limiting; required_min = peak (199.8), required_margin = 1.3 x peak (259.74).
    //  - static_factor 1.15, capacity 150 (static 172.5): required_min = peak / 1.15 (the KINETIC capacity that holds the
    //    peak statically), margin 1.3 x that.
    //  - static_factor 1.0, capacity 230: holds the peak, but 1.3 x peak = 259.7 > 230: Near.
    //  - capacity 300: none.
    // Recorded sabotage: dropping the `/ static_factor` from required_min fails the 1.15 case; kDrivelineMargin 1.3 -> 1.5
    // fails the margin numbers.
    const DynoResult tm = run_dyno_sweep(i4_torque_map(), nullptr, DynoSweepSpec{}, nullptr);
    const double peak = peak_torque(tm);
    vehicle::VehicleDesc v = io::load_vehicle_json(kVehicles + "car_sedan.json");
    ClutchDesc* clutch = nullptr;
    for (ComponentDesc& c : v.powertrain.components)
        if (auto* cl = std::get_if<ClutchDesc>(&c.params)) clutch = cl;
    REQUIRE(clutch != nullptr);

    const auto check = [&](double capacity, double factor) {
        clutch->capacity_nm = capacity;
        clutch->static_factor = factor;
        return check_driveline_limits(tm, v.powertrain, "clutch.json");
    };
    {
        const auto f = check(150.0, 1.0);
        REQUIRE(f.size() == 1);
        CHECK(f[0].kind == LimitKind::ClutchCapacity);
        CHECK(f[0].severity == LimitSeverity::Limiting);
        CHECK(f[0].rpm == 4550.0);
        CHECK(f[0].value == peak);
        CHECK(f[0].limit == 150.0);
        CHECK(f[0].unit == LimitUnit::NewtonMetre);
        CHECK(f[0].part.slot == "clutch");
        CHECK(f[0].part.file == "clutch.json");
        CHECK(f[0].part.field == "/capacity_nm");
        REQUIRE(f[0].remedy.has_value());
        const Remedy& r = *f[0].remedy;
        CHECK(r.cost == RemedyCost::Closed);
        CHECK(r.solved);
        CHECK(r.unit == LimitUnit::NewtonMetre);
        CHECK(r.current == 150.0);
        CHECK(r.required_min == peak);
        CHECK(r.required_margin == Catch::Approx(1.3 * peak).epsilon(1e-12));
        CHECK(r.ratio_margin() == Catch::Approx(1.3 * peak / 150.0).epsilon(1e-12));
        CHECK(r.part.field == "/capacity_nm");
    }
    {
        const auto f = check(150.0, 1.15);
        REQUIRE(f.size() == 1);
        CHECK(f[0].severity == LimitSeverity::Limiting);
        CHECK(f[0].limit == Catch::Approx(172.5).epsilon(1e-12));
        REQUIRE(f[0].remedy.has_value());
        CHECK(f[0].remedy->required_min == Catch::Approx(peak / 1.15).epsilon(1e-12));
        CHECK(f[0].remedy->required_margin == Catch::Approx(1.3 * peak / 1.15).epsilon(1e-12));
    }
    {
        const auto f = check(230.0, 1.0);
        REQUIRE(f.size() == 1);
        CHECK(f[0].severity == LimitSeverity::Near);
        REQUIRE(f[0].remedy.has_value());
        CHECK(f[0].remedy->current == 230.0);
    }
    CHECK(check(300.0, 1.0).empty());
    // Applying the margin remedy clears the finding.
    {
        const auto f = check(150.0, 1.0);
        REQUIRE(f.size() == 1);
        CHECK(check(f[0].remedy->required_margin * 1.001, 1.0).empty()); // a hair above the margin: clear
        const auto at_min = check(f[0].remedy->required_min * 1.001, 1.0);   // holds the peak, inside the margin band
        REQUIRE(at_min.size() == 1);
        CHECK(at_min[0].severity == LimitSeverity::Near);
    }
    // Degenerate sweeps.
    CHECK(check_driveline_limits(DynoResult{}, v.powertrain).empty());
    DynoResult zero;
    DynoPoint p;
    p.rpm = 1000.0;
    p.torque_nm = -5.0;
    zero.points.push_back(p);
    CHECK(check_driveline_limits(zero, v.powertrain).empty());
}

TEST_CASE("L21 not-produced kinds stay absent", "[drivetrain][dyno][turbo]") {
    // ChargeTemperature, GearboxInputTorque, ValveFlowChoke, Knock and InjectorDuty are never produced; the nitrous kinds only with a kit (test_limit_remedies.cpp).
    static jobs::JobSystem pool(10);
    const DynoResult r = run_dyno_sweep(hyper_crafted(2.0, 1.0), nullptr, turbo_spec({7000.0, 8000.0}), &pool);
    for (LimitKind k : {LimitKind::ChargeTemperature, LimitKind::GearboxInputTorque, LimitKind::ValveFlowChoke, LimitKind::Knock,
                        LimitKind::KitFuelLean, LimitKind::InjectorDuty, LimitKind::N2OBottleDuration, LimitKind::N2OBottlePressureDrop,
                        LimitKind::ClutchCapacity}) {
        INFO(to_string(k));
        CHECK_FALSE(any_kind(r, k));
    }
}
