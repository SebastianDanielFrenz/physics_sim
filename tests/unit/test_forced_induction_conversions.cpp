// tests/unit/test_forced_induction_conversions.cpp - engine garage P5 (racing_game out/engine_garage_design.md A13):
//   adding or removing forced induction on a generated (cycle-block) simulated engine through the loader's forced_induction slot,
//   the turbo hardware family (data/turbos/{small_gt28,medium_gtx30,large_gtx42}_class.json) and the single/twin templates
//   (data/turbo_configurations/{single,twin}_{small,medium,large}.json).
//
// The conversion helper below does what the garage does: read the stock engine validate-only, take the grid from P4's
// boost_grid_p_im / ambient_grid_p_im, write it (and nothing else) into a user engine that "extends" the stock file, and load that
// with the forced_induction slot set to the template (or "none") - the loader sets compressor_supply itself for a pipe engine.
//
// Every guard lists its recorded sabotage in its case comment (changed, failed with the quoted symptom, reverted).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {

const std::string kEngines = std::string(PS_SOURCE_DIR) + "/data/engines/";
const std::string kTurboConfigs = std::string(PS_SOURCE_DIR) + "/data/turbo_configurations/";
const std::string kTurbos = std::string(PS_SOURCE_DIR) + "/data/turbos/";

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

const SimulatedEngineDesc& S(const ComponentParams& c) { return std::get<SimulatedEngineDesc>(c); }

io::EngineMapOptions validate_only() {
    io::EngineMapOptions o;
    o.generate = false;
    return o;
}

std::string kpa_list(const std::vector<real>& pa) {
    std::string s = "[";
    char buf[64];
    for (std::size_t i = 0; i < pa.size(); ++i) {
        std::snprintf(buf, sizeof buf, "%s%.6f", i ? ", " : "", pa[i] / 1000.0);
        s += buf;
    }
    return s + "]";
}

// What a conversion does to the stock engine file `base` (relative to data/engines): `fi` is the template file name in
// data/turbo_configurations (without directory) or "none"; compression_ratio 0 = keep. Returns the engine as loaded with the
// slot, at map quality `q` (generated, on the pool).
struct Conversion {
    std::string base;
    std::string fi;
    MapQuality q = MapQuality::Sketch;
    real compression_ratio = 0.0;
    bool generate = true;
    std::vector<real> grid_override; // non-empty: this axis [Pa] instead of the helper's
};

std::vector<real> conversion_grid(const Conversion& cv) {
    if (cv.fi.empty()) return {};
    if (!cv.grid_override.empty()) return cv.grid_override;
    const ComponentParams stock = io::load_engine_json(kEngines + cv.base, validate_only());
    const CycleModelDesc& cyc = *S(stock).cycle;
    if (cv.fi == "none") return ambient_grid_p_im(cyc);
    const TurboPairDesc t = io::load_turbo_configuration_json(kTurboConfigs + cv.fi);
    return boost_grid_p_im(cyc, t.target_boost_pa / 1e5);
}

ComponentParams convert(const Conversion& cv) {
    const std::vector<real> grid = conversion_grid(cv);
    std::string text = "{\"extends\": \"" + cv.base + "\", \"cycle\": {";
    if (!grid.empty()) text += "\"grid_p_im_kpa\": " + kpa_list(grid);
    if (cv.compression_ratio > 0.0) text += std::string(grid.empty() ? "" : ", ") + "\"compression_ratio\": " + std::to_string(cv.compression_ratio);
    text += "}}";
    io::EngineMapOptions o;
    o.quality = cv.q;
    o.pool = &pool();
    o.generate = cv.generate;
    io::EngineSlotOverrides slots;
    if (!cv.fi.empty()) slots["forced_induction"] = cv.fi == "none" ? "none" : kTurboConfigs + cv.fi;
    return io::load_engine_json_from_string(text, kEngines, o, slots);
}

DynoSweepSpec spec_at(std::vector<double> rpm) {
    DynoSweepSpec s;
    s.rpm = std::move(rpm);
    return s;
}

double seconds_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

void print_dyno(const char* tag, const DynoResult& r) {
    for (const DynoPoint& p : r.points) {
        std::printf("%s %6.0f rpm: torque %7.1f N*m  %6.1f kW  boost %6.3f charge %6.3f bar  p_in %6.1f kPa  turbo %7.0f rpm  conv %d settle %.1f s\n",
                    tag, p.rpm, p.torque_nm, p.power_kw, p.boost_bar, p.charge_boost_bar, p.p_intake_kpa, p.turbo_rpm, p.converged ? 1 : 0,
                    p.settle_s);
    }
    for (const LimitFinding& f : r.limits) std::printf("%s   limit %s %s @%.0f\n", tag, to_string(f.kind), to_string(f.severity), f.rpm);
}

// A cache directory of this process alone (the test binary may run next to other ctest processes).
std::string unique_cache_dir(const char* tag) {
    const auto t = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path p = std::filesystem::temp_directory_path() / (std::string("ps_fi_") + tag + "_" + std::to_string(t));
    std::filesystem::create_directories(p);
    return p.string();
}

struct CacheDirGuard {
    std::string dir;
    explicit CacheDirGuard(const char* tag) : dir(unique_cache_dir(tag)) {}
    ~CacheDirGuard() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
};

// Worst |charge - target| and lowest manifold gauge over the points of a dyno result.
struct BoostSummary {
    double worst_charge_err = 0.0;
    double min_charge = 1e9;
    double min_manifold = 1e9;
    double max_charge = -1e9;
};
BoostSummary summarise(const DynoResult& r, double target_bar) {
    BoostSummary b;
    for (const DynoPoint& p : r.points) {
        b.worst_charge_err = std::max(b.worst_charge_err, std::fabs(p.charge_boost_bar - target_bar));
        b.min_charge = std::min(b.min_charge, p.charge_boost_bar);
        b.max_charge = std::max(b.max_charge, p.charge_boost_bar);
        b.min_manifold = std::min(b.min_manifold, p.boost_bar);
    }
    return b;
}

bool has_issue(const std::vector<BuildIssue>& v, BuildRule r) {
    return std::any_of(v.begin(), v.end(), [r](const BuildIssue& i) { return i.rule == r; });
}

const std::vector<std::string> kTemplates = {"single_small.json", "single_medium.json", "single_large.json",
                                              "twin_small.json",   "twin_medium.json",   "twin_large.json"};

} // namespace

// ---- cases ----

TEST_CASE("FI1 lumped A25A (generated maps, no pipes) + single_medium: boosted rows generate, trapped air rises, charge holds the target",
          "[turbo][drivetrain][cycle]") {
    // The lumped cycle model runs every grid point between fixed reservoirs at the node pressure, so a node above ambient
    // generates without the pipe engine's compressor_supply throttle path (cycle.intake is absent, the loader sets no flag).
    CacheDirGuard cache("fi1");
    std::size_t gens = 0;
    Conversion cv;
    cv.base = "toyota_a25a_fks_gen.json";
    cv.fi = "single_medium.json";
    cv.q = MapQuality::Full;
    cv.compression_ratio = 10.0; // check_buildable: forced induction <= 10.5 (the stock A25A runs 13)
    const auto t0 = std::chrono::steady_clock::now();
    ComponentParams boosted = convert(cv);
    const double gen_s = seconds_since(t0);
    const SimulatedEngineDesc& e = S(boosted);
    REQUIRE(e.cycle.has_value());
    CHECK_FALSE(e.cycle->intake.has_value()); // lumped: no pipe blocks, so no compressor_supply path exists or is needed
    REQUIRE(e.turbo_pair.has_value());
    CHECK(e.turbo_pair->target_boost_pa == Catch::Approx(80000.0));
    CHECK(e.cycle->grid_p_im.back() > kIsaAirPressurePa + 80000.0 - 1.0); // reaches ambient + target
    CHECK_FALSE(has_issue(check_buildable(boosted), BuildRule::BoostAbovePressureGrid));

    // trapped air (ve * p) rises with the node pressure at every tested rpm, strictly
    const Table2D& ve = e.maps.ve;
    for (double rpm : {2000.0, 4500.0, 6000.0}) {
        double prev = 0.0;
        for (real p : e.cycle->grid_p_im) {
            const double air = ve.value(rpm, p) * p;
            INFO("rpm " << rpm << " p " << p);
            CHECK(air > prev);
            prev = air;
        }
    }
    const double air_amb = ve.value(4500.0, kIsaAirPressurePa) * kIsaAirPressurePa;
    const double air_top = ve.value(4500.0, e.cycle->grid_p_im.back()) * e.cycle->grid_p_im.back();
    std::printf("FI1 load %.2f s; trapped air top/ambient row at 4500 rpm: %.3f\n", gen_s, air_top / air_amb);
    CHECK(air_top / air_amb > 1.5);

    const DynoResult rb = run_dyno_sweep(boosted, nullptr, spec_at({3000.0, 4500.0, 6000.0}), &pool());
    print_dyno("FI1 boosted", rb);
    REQUIRE(rb.points.size() == 3);
    const BoostSummary b = summarise(rb, 0.8);
    std::printf("FI1 worst charge error %.4f bar, min manifold %.3f bar\n", b.worst_charge_err, b.min_manifold);
    CHECK(b.worst_charge_err < 0.05); // the brief's criterion: steady WOT charge = target within 0.05 bar
    CHECK(b.min_manifold > 0.74); // measured 0.765: the throttle's WOT loss, not a shortfall of the compressor
    for (const DynoPoint& p : rb.points) CHECK(p.converged);

    // against the same engine at the same compression ratio without the compressor
    Conversion nv;
    nv.base = cv.base;
    nv.q = MapQuality::Full;
    nv.compression_ratio = 10.0;
    ComponentParams na = convert(nv);
    const DynoResult rn = run_dyno_sweep(na, nullptr, spec_at({3000.0, 4500.0, 6000.0}), &pool());
    print_dyno("FI1 NA", rn);
    REQUIRE(rn.points.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        INFO(rb.points[i].rpm);
        CHECK(rn.points[i].boost_bar == 0.0);
        CHECK(rb.points[i].torque_nm > 1.6 * rn.points[i].torque_nm); // measured 1.78-1.81 (about 1.8 bar absolute / 1.0)
    }

    // cache: a second load of the same conversion reads the file and runs no generator
    io::EngineMapOptions o;
    o.quality = MapQuality::Full;
    o.pool = &pool();
    o.cache_dir = cache.dir;
    o.generation_count_out = &gens;
    const std::string text = "{\"extends\": \"" + cv.base + "\", \"cycle\": {\"grid_p_im_kpa\": " + kpa_list(conversion_grid(cv)) +
                             ", \"compression_ratio\": 10.0}}";
    io::EngineSlotOverrides slots;
    slots["forced_induction"] = kTurboConfigs + cv.fi;
    (void)io::load_engine_json_from_string(text, kEngines, o, slots);
    CHECK(gens == 1);
    (void)io::load_engine_json_from_string(text, kEngines, o, slots);
    CHECK(gens == 1);
}

TEST_CASE("FI2 pipe I4 (demo_i4_pipes) + each single template: the charge reaches the target inside the rated rpm band and falls short below it",
          "[turbo][drivetrain][cycle]") {
    // One conversion per hardware size on the 2.0 L demo engine. The map key does not depend on the hardware (same grid, same
    // compressor_supply flag), so the second and third load hit the cache the first one wrote; only the turbo tables differ.
    // The band numbers are the measured operating ranges quoted in docs/data_formats/turbo.md ("Measured fit").
    // Recorded sabotage: single_large's turbine area x 0.5 in the data (a wrong size): the large turbine then spools at 2500 rpm
    // (charge 0.80, not under 0.7), so the shortfall check fails (here and in FI3 for twin_large; FI6's monotone sizes also).
    struct Case {
        const char* tmpl;
        std::vector<double> in_band;
        double short_rpm;
    };
    const std::vector<Case> cases = {
        {"single_small.json", {2500.0, 4500.0, 6000.0}, 1250.0},
        {"single_medium.json", {3000.0, 4500.0, 6000.0}, 1500.0},
        {"single_large.json", {4000.0, 5000.0, 6000.0}, 2500.0},
    };
    CacheDirGuard cache("fi2");
    std::uint64_t first_key = 0;
    for (std::size_t ci = 0; ci < cases.size(); ++ci) {
        const Case& c = cases[ci];
        INFO(c.tmpl);
        std::size_t gens = 0;
        const std::string text_grid = [&] {
            Conversion cv;
            cv.base = "demo_i4_pipes.json";
            cv.fi = c.tmpl;
            return kpa_list(conversion_grid(cv));
        }();
        const std::string text = "{\"extends\": \"demo_i4_pipes.json\", \"cycle\": {\"grid_p_im_kpa\": " + text_grid + "}}";
        io::EngineMapOptions o;
        o.quality = MapQuality::Sketch;
        o.pool = &pool();
        o.cache_dir = cache.dir;
        o.generation_count_out = &gens;
        io::EngineSlotOverrides slots;
        slots["forced_induction"] = kTurboConfigs + c.tmpl;
        const auto t0 = std::chrono::steady_clock::now();
        ComponentParams eng = io::load_engine_json_from_string(text, kEngines, o, slots);
        std::printf("FI2 %s: load %.2f s, generator runs %zu\n", c.tmpl, seconds_since(t0), gens);
        CHECK(gens == (ci == 0 ? 1u : 0u)); // hardware does not move the map key
        const SimulatedEngineDesc& e = S(eng);
        REQUIRE(e.cycle.has_value());
        REQUIRE(e.cycle->intake.has_value());
        CHECK(e.cycle->intake->compressor_supply); // set by the loader for a pipe engine with a turbo
        const std::uint64_t key = engine_map_cache_key(*e.cycle, e.fuels.front(), MapQuality::Sketch);
        if (ci == 0) first_key = key;
        CHECK(key == first_key);
        CHECK_FALSE(has_issue(check_buildable(eng), BuildRule::BoostAbovePressureGrid));

        std::vector<double> rpm = c.in_band;
        rpm.push_back(c.short_rpm);
        const DynoResult r = run_dyno_sweep(eng, nullptr, spec_at(rpm), &pool());
        print_dyno(c.tmpl, r);
        REQUIRE(r.points.size() == rpm.size());
        DynoResult band;
        for (std::size_t i = 0; i < c.in_band.size(); ++i) band.points.push_back(r.points[i]);
        const BoostSummary b = summarise(band, 0.8);
        std::printf("FI2 %s in band: worst charge error %.4f bar, min manifold %.3f\n", c.tmpl, b.worst_charge_err, b.min_manifold);
        CHECK(b.worst_charge_err < 0.05);
        CHECK(b.min_manifold > 0.7);
        // below the band the hardware is too big for the air flow: spool is late and the charge stays under the target
        CHECK(r.points.back().charge_boost_bar < 0.7);
        // the limit advisor says so too: BoostBelowTarget at the short point and at none of the in-band points
        for (const LimitFinding& f : r.limits) {
            if (f.kind != LimitKind::BoostBelowTarget) continue;
            INFO(f.rpm);
            CHECK(f.rpm == c.short_rpm);
        }
        CHECK(std::any_of(r.limits.begin(), r.limits.end(), [&](const LimitFinding& f) {
            return f.kind == LimitKind::BoostBelowTarget && f.rpm == c.short_rpm;
        }));
    }
}

TEST_CASE("FI3 hyper V8 (5 L, pipes, stock twin at 1.8 bar) re-fitted with 0.8 bar templates: the right size holds, too small and too large are modelled",
          "[turbo][drivetrain][cycle]") {
    // The 5 L V8 asks 2-3x the air of the 2 L demo per turbo: single_large and twin_medium hold 0.8 bar over the whole range,
    // twin_small runs out of compressor above 6000 rpm and twin_large barely spools at 2000 rpm. The design (A13) models all of
    // these instead of blocking them; this guard pins that the model tells them apart.
    // Recorded sabotage: single_large's wastegate port at 2.36 x the turbine throat (the first data value, the 'small' hardware's
    // ratio) saturates at 8000 rpm: the charge runs to 0.994 bar (measured) and the hold check fails.
    CacheDirGuard cache("fi3");
    auto run = [&](const char* tmpl, std::vector<double> rpm, DynoResult* out) {
        Conversion cv;
        cv.base = "hyper_v8_one1_audio.json";
        cv.fi = tmpl;
        const std::string text = "{\"extends\": \"hyper_v8_one1_audio.json\", \"cycle\": {\"grid_p_im_kpa\": " + kpa_list(conversion_grid(cv)) + "}}";
        io::EngineMapOptions o;
        o.quality = MapQuality::Sketch;
        o.pool = &pool();
        o.cache_dir = cache.dir;
        io::EngineSlotOverrides slots;
        slots["forced_induction"] = kTurboConfigs + tmpl;
        ComponentParams eng = io::load_engine_json_from_string(text, kEngines, o, slots);
        REQUIRE(S(eng).turbo_pair.has_value());
        *out = run_dyno_sweep(eng, nullptr, spec_at(std::move(rpm)), &pool());
        print_dyno(tmpl, *out);
    };
    DynoResult r;
    for (const char* tmpl : {"single_large.json", "twin_medium.json"}) {
        INFO(tmpl);
        run(tmpl, {2000.0, 4000.0, 6000.0, 8000.0}, &r);
        REQUIRE(r.points.size() == 4);
        const BoostSummary b = summarise(r, 0.8);
        std::printf("FI3 %s: worst charge error %.4f, min manifold %.3f\n", tmpl, b.worst_charge_err, b.min_manifold);
        CHECK(b.worst_charge_err < 0.05);
        CHECK(b.min_manifold > 0.7);
    }
    run("twin_small.json", {4000.0, 6000.0, 8000.0}, &r);
    REQUIRE(r.points.size() == 3);
    CHECK(std::fabs(r.points[0].charge_boost_bar - 0.8) < 0.05); // fine at mid rpm...
    // ...but the small pair's turbine and gate cannot pass the 5 L engine's exhaust at the top: the gate saturates and the
    // charge runs away above the target (rotor overspeed is reported, nothing blocks it)
    CHECK(r.points[2].charge_boost_bar > 0.9);
    CHECK(std::any_of(r.limits.begin(), r.limits.end(), [](const LimitFinding& f) {
        return f.kind == LimitKind::WastegateSaturated && f.rpm == 8000.0;
    }));
    CHECK(std::any_of(r.limits.begin(), r.limits.end(), [](const LimitFinding& f) {
        return f.kind == LimitKind::BoostAboveTarget && f.rpm == 8000.0;
    }));
    run("twin_large.json", {2000.0, 6000.0}, &r);
    REQUIRE(r.points.size() == 2);
    CHECK(r.points[0].charge_boost_bar < 0.4);  // late spool
    CHECK(r.points[1].charge_boost_bar > 0.75); // fine once spinning
}

TEST_CASE("FI4 hyper V8 with its turbo removed through the loader (forced_induction none + ambient_grid_p_im) generates and runs naturally aspirated",
          "[turbo][drivetrain][cycle]") {
    // P4's G2 did this by editing the descriptor; this is the same conversion on the path the garage uses.
    // Recorded sabotage: the stock grid (160-300 kPa rows) kept while the slot says none: the loader refuses it (negative case
    // below), exactly what a hand-edit without the helper would hit.
    const ComponentParams stock = io::load_engine_json(kEngines + "hyper_v8_one1_audio.json", validate_only());
    REQUIRE(S(stock).turbo_pair.has_value());
    REQUIRE(S(stock).cycle->intake.has_value());
    CHECK(S(stock).cycle->intake->compressor_supply);

    Conversion cv;
    cv.base = "hyper_v8_one1_audio.json";
    cv.fi = "none";
    cv.q = MapQuality::Sketch;
    const auto t0 = std::chrono::steady_clock::now();
    ComponentParams na = convert(cv);
    const SimulatedEngineDesc& e = S(na);
    CHECK_FALSE(e.turbo_pair.has_value());
    REQUIRE(e.cycle->intake.has_value());
    CHECK_FALSE(e.cycle->intake->compressor_supply); // the loader clears it with the turbo
    CHECK(e.cycle->grid_p_im.back() <= 0.95 * kIsaAirPressurePa + 1e-6);
    CHECK(classify_engine_change(stock, na) == EngineChange::Maps);
    for (const BuildIssue& i : check_buildable(na)) CHECK(i.severity == LimitSeverity::Near); // the hyper's 26.2 m/s piston speed, as shipped
    const DynoResult r = run_dyno_sweep(na, nullptr, spec_at({3000.0, 6000.0}), &pool());
    std::printf("FI4 NA hyper (Sketch maps + 2 dyno points): %.2f s\n", seconds_since(t0));
    print_dyno("FI4", r);
    REQUIRE(r.points.size() == 2);
    for (const DynoPoint& p : r.points) {
        INFO(p.rpm);
        CHECK(p.torque_nm > 100.0);
        CHECK(p.boost_bar == 0.0);
        CHECK(p.charge_boost_bar == 0.0);
    }
    CHECK(r.points[1].torque_nm < 700.0); // the boosted hyper makes 1371 N*m at 6000 rpm

    // negative: 'none' with the boosted grid is refused with a reason (not silently clamped)
    Conversion bad = cv;
    bad.grid_override = S(stock).cycle->grid_p_im;
    bool threw = false;
    try {
        (void)convert(bad);
    } catch (const std::exception& ex) {
        threw = true;
        std::printf("FI4 refusal: %s\n", ex.what());
        CHECK(std::string(ex.what()).find("grid_p_im") != std::string::npos);
    }
    CHECK(threw);
}

TEST_CASE("FI5 cache keys: boosted vs naturally aspirated differ exactly by the compressor_supply flag and the grid, never by the hardware",
          "[turbo][drivetrain][cycle]") {
    // validate-only loads (no generation); the key is a function of the resolved cycle block, the fuel and the quality.
    // Recorded sabotage: the loader's compressor_supply assignment (vehicle_io.cpp, 'e.cycle->intake->compressor_supply =
    // e.turbo_pair.has_value()') forced to false: here key(flag-set reference) != key(loaded boosted engine) (0xcdf1efee... vs
    // 0x9db968a1...); FI2/FI3 fail at generation ("throttle fully open: mean plenum 101.2 kPa", a boosted node needs the supply
    // path) and FI4's premise on the stock hyper fails.
    Conversion cv;
    cv.base = "demo_i4_pipes.json";
    cv.fi = "single_small.json";
    cv.generate = false;
    const ComponentParams stock = io::load_engine_json(kEngines + cv.base, validate_only());
    const ComponentParams boosted = convert(cv);
    const SimulatedEngineDesc& s = S(stock);
    const SimulatedEngineDesc& b = S(boosted);
    REQUIRE(s.cycle.has_value());
    REQUIRE(b.cycle.has_value());
    REQUIRE(b.cycle->intake.has_value());
    auto key = [&](const CycleModelDesc& c) { return engine_map_cache_key(c, s.fuels.front(), MapQuality::Sketch); };

    // what the loader produced for the boosted engine == the stock desc with exactly those two changes
    CycleModelDesc ref = *s.cycle;
    ref.grid_p_im = b.cycle->grid_p_im;
    ref.intake->compressor_supply = true;
    CHECK(key(ref) == key(*b.cycle));

    CycleModelDesc flag_off = ref;
    flag_off.intake->compressor_supply = false;
    CHECK(key(flag_off) != key(ref));           // the flag alone moves the key
    CycleModelDesc stock_grid_flag = *s.cycle;
    stock_grid_flag.intake->compressor_supply = true;
    CHECK(key(stock_grid_flag) != key(*s.cycle)); // ... on the stock grid too
    CHECK(key(flag_off) != key(*s.cycle));      // the grid alone moves it
    CHECK(key(stock_grid_flag) != key(ref));    // and the pair differs from either single change

    // hardware (size, count) is not part of the map key: all six templates give the same key
    for (const std::string& t : kTemplates) {
        INFO(t);
        Conversion c = cv;
        c.fi = t;
        const ComponentParams e = convert(c);
        CHECK(key(*S(e).cycle) == key(*b.cycle));
    }

    // what each edit costs
    CHECK(classify_engine_change(stock, boosted) == EngineChange::Maps);
    Conversion large = cv;
    large.fi = "single_large.json";
    CHECK(classify_engine_change(boosted, convert(large)) == EngineChange::TurboTables); // different hardware, same maps
    Conversion twin = cv;
    twin.fi = "twin_small.json";
    CHECK(classify_engine_change(boosted, convert(twin)) == EngineChange::TurboTables);  // one turbo -> two
    CHECK(classify_engine_change(boosted, boosted) == EngineChange::None);

    // a lumped engine has no intake block: the flag does not exist, only the grid moves the key
    const ComponentParams a25a_stock = io::load_engine_json(kEngines + "toyota_a25a_fks_gen.json", validate_only());
    Conversion al;
    al.base = "toyota_a25a_fks_gen.json";
    al.fi = "single_medium.json";
    al.generate = false;
    const ComponentParams a25a_boost = convert(al);
    CHECK_FALSE(S(a25a_boost).cycle->intake.has_value());
    CHECK(engine_map_cache_key(*S(a25a_boost).cycle, S(a25a_stock).fuels.front(), MapQuality::Full) !=
          engine_map_cache_key(*S(a25a_stock).cycle, S(a25a_stock).fuels.front(), MapQuality::Full));
    CycleModelDesc same_grid = *S(a25a_stock).cycle;
    same_grid.grid_p_im = S(a25a_boost).cycle->grid_p_im;
    CHECK(engine_map_cache_key(same_grid, S(a25a_stock).fuels.front(), MapQuality::Full) ==
          engine_map_cache_key(*S(a25a_boost).cycle, S(a25a_stock).fuels.front(), MapQuality::Full)); // grid is the only difference
}

TEST_CASE("FI6 hardware family and templates: scaling laws, monotone sizes, count multiplies the pair, every number carries a source",
          "[turbo][drivetrain][io]") {
    // Recorded sabotages: (1) twin_small.json count 2 -> 1: the 'twin has twice the single's inertia/flow/areas' check fails;
    // (2) large_gtx42_class.json turbine_area_m2 halved (0.0003285): the monotone-size check fails (0.0003749 < 0.0003285);
    // (3) large_gtx42_class.json reference_rpm 105000 -> 120000: the tip-speed similarity check (ref_rpm * sqrt(flow) const
    // within 3 %) fails.
    const char* hw[3] = {"small_gt28_class.json", "medium_gtx30_class.json", "large_gtx42_class.json"};
    const char* hw_stem[3] = {"small", "medium", "large"};
    for (int i = 0; i < 3; ++i) {
        std::ifstream f(kTurbos + hw[i]);
        REQUIRE(f.good());
        const nlohmann::json j = nlohmann::json::parse(f);
        INFO(hw[i]);
        CHECK(j.at("format").get<std::string>() == "physics_sim.turbo/1");
        const std::string src = j.at("source").get<std::string>();
        CHECK(src.find("[AS]") != std::string::npos);
        CHECK(src.find("[D]") != std::string::npos);
        CHECK(j.at("rotor_inertia_source").get<std::string>().find("[D]") != std::string::npos);
        const auto& rated = j.at("rated_power_kw_per_turbo");
        CHECK(rated.at("min").get<double>() > 0.0);
        CHECK(rated.at("max").get<double>() > rated.at("min").get<double>());
        CHECK(rated.at("source").get<std::string>().find("[AS]") != std::string::npos);
    }
    const TurboPairDesc s1 = io::load_turbo_configuration_json(kTurboConfigs + "single_small.json");
    const TurboPairDesc m1 = io::load_turbo_configuration_json(kTurboConfigs + "single_medium.json");
    const TurboPairDesc l1 = io::load_turbo_configuration_json(kTurboConfigs + "single_large.json");
    const TurboPairDesc* ones[3] = {&s1, &m1, &l1};
    // sizes grow, speed falls
    CHECK(s1.rotor_inertia_kgm2 < m1.rotor_inertia_kgm2);
    CHECK(m1.rotor_inertia_kgm2 < l1.rotor_inertia_kgm2);
    CHECK(s1.max_mass_flow_kg_s < m1.max_mass_flow_kg_s);
    CHECK(m1.max_mass_flow_kg_s < l1.max_mass_flow_kg_s);
    CHECK(s1.turbine_area_m2 < m1.turbine_area_m2);
    CHECK(m1.turbine_area_m2 < l1.turbine_area_m2);
    CHECK(s1.reference_rpm > m1.reference_rpm);
    CHECK(m1.reference_rpm > l1.reference_rpm);
    // tip-speed similarity: flow ~ D^2 and rpm ~ 1/D, so rpm * sqrt(flow) is the same for every size
    const double k0 = s1.reference_rpm * std::sqrt(s1.max_mass_flow_kg_s);
    for (const TurboPairDesc* t : ones) {
        CHECK(t->reference_rpm * std::sqrt(t->max_mass_flow_kg_s) == Catch::Approx(k0).epsilon(0.03));
        CHECK(t->reference_head_pa == Catch::Approx(450000.0)); // same stage tip speed: head unchanged (bar in the file)
        CHECK(t->wastegate_area() > t->turbine_area_m2);        // the gate is a bypass bigger than the throat
    }
    // the pair multiplies: a twin has twice the inertia, flow, throat, gate and shaft loss; speeds, efficiencies, head do not move
    const char* twins[3] = {"twin_small.json", "twin_medium.json", "twin_large.json"};
    for (int i = 0; i < 3; ++i) {
        INFO(twins[i]);
        const TurboPairDesc t2 = io::load_turbo_configuration_json(kTurboConfigs + twins[i]);
        const TurboPairDesc& t1 = *ones[i];
        CHECK(t2.rotor_inertia_kgm2 == Catch::Approx(2.0 * t1.rotor_inertia_kgm2));
        CHECK(t2.max_mass_flow_kg_s == Catch::Approx(2.0 * t1.max_mass_flow_kg_s));
        CHECK(t2.turbine_area_m2 == Catch::Approx(2.0 * t1.turbine_area_m2));
        CHECK(t2.wastegate_area() == Catch::Approx(2.0 * t1.wastegate_area()));
        CHECK(t2.shaft_loss_w_at_reference == Catch::Approx(2.0 * t1.shaft_loss_w_at_reference));
        CHECK(t2.reference_rpm == t1.reference_rpm);
        CHECK(t2.compressor_efficiency == t1.compressor_efficiency);
        CHECK(t2.reference_head_pa == t1.reference_head_pa);
    }
    // all six: the same 0.8 bar gauge target, intercooler and controller, so only the size and count differ between them
    for (const std::string& t : kTemplates) {
        INFO(t);
        const TurboPairDesc d = io::load_turbo_configuration_json(kTurboConfigs + t);
        CHECK(d.target_boost_pa == Catch::Approx(80000.0));
        CHECK(d.intercooler_effectiveness == Catch::Approx(0.75));
        CHECK(d.wastegate_kp == 1.0);
        CHECK(d.wastegate_ki_per_s == 1.0);
        CHECK(d.wastegate_kd_s == 0.05);
        CHECK(d.wastegate_actuator_tau_s == 0.1);
        CHECK_FALSE(d.blow_off_valve.has_value());
        CHECK_FALSE(d.anti_lag.has_value());
        CHECK(d.intercooler_dp_pa_at_max_flow == 0.0);
        std::ifstream f(kTurboConfigs + t);
        const nlohmann::json j = nlohmann::json::parse(f);
        const std::string src = j.at("source").get<std::string>();
        CHECK(src.find("[AS]") != std::string::npos);
        const std::string stem = t.substr(t.find('_') + 1, t.find('.') - t.find('_') - 1);
        CHECK(std::string(hw_stem[stem == "small" ? 0 : stem == "medium" ? 1 : 2]) == stem);
        CHECK(j.at("turbo").get<std::string>().find(stem + "_") != std::string::npos);
    }
}

TEST_CASE("FI7 boost_grid_p_im adds a node at ambient when the stock axis stops well short of it (the A25A's 97.27 kPa)",
          "[turbo][drivetrain][cycle]") {
    // Without that node the 97.27 -> 161 kPa interval interpolates the plenum near ambient toward the first boosted row; a
    // Preview-quality load of the boosted A25A then fails the engine's own steady-torque monotonicity check.
    // Recorded sabotage: the push_back in boost_grid_p_im removed (engine_rules.cpp, 'Engine garage P5'): the axis case below
    // fails at the first CHECK, and the Preview load throws "steady torque is not non-decreasing in plate position".
    const ComponentParams a25a = io::load_engine_json(kEngines + "toyota_a25a_fks_gen.json", validate_only());
    const CycleModelDesc& cyc = *S(a25a).cycle;
    real last_na = 0.0; // the last stock node at or below ambient
    for (real p : cyc.grid_p_im)
        if (p <= kIsaAirPressurePa) last_na = p;
    CHECK(last_na < kIsaAirPressurePa - 2000.0); // the premise: that node is under ambient by more than 2 kPa (97.27 kPa)
    const std::vector<real> g = boost_grid_p_im(cyc, 0.8);
    REQUIRE(g.size() >= 4);
    CHECK(std::count(g.begin(), g.end(), kIsaAirPressurePa) == 1); // an exact ambient node
    CHECK(std::is_sorted(g.begin(), g.end()));
    CHECK(std::adjacent_find(g.begin(), g.end()) == g.end()); // strictly ascending
    CHECK(g.back() >= 200000.0 - 1.0);                        // (1 + 0.8 + 0.2) bar absolute
    // the stock nodes at or below ambient are kept as they are
    for (real p : cyc.grid_p_im)
        if (p <= kIsaAirPressurePa) CHECK(std::find(g.begin(), g.end(), p) != g.end());

    // an engine whose axis already ends within 2 kPa of ambient (demo: 100 kPa) gets no extra node
    const ComponentParams demo = io::load_engine_json(kEngines + "demo_i4_pipes.json", validate_only());
    const std::vector<real> gd = boost_grid_p_im(*S(demo).cycle, 0.8);
    CHECK(std::count(gd.begin(), gd.end(), kIsaAirPressurePa) == 0);
    CHECK(std::count(gd.begin(), gd.end(), 100000.0) == 1);

    // Preview-quality load and dyno of the boosted A25A: with the helper's axis it works; with the old axis (no ambient node) it throws
    Conversion cv;
    cv.base = "toyota_a25a_fks_gen.json";
    cv.fi = "single_medium.json";
    cv.q = MapQuality::Preview;
    cv.compression_ratio = 10.0;
    ComponentParams c = convert(cv);
    const DynoResult r = run_dyno_sweep(c, nullptr, spec_at({3000.0, 5000.0}), &pool());
    print_dyno("FI7 Preview", r);
    REQUIRE(r.points.size() == 2);
    for (const DynoPoint& p : r.points) {
        INFO(p.rpm);
        CHECK(p.charge_boost_bar == Catch::Approx(0.8).margin(0.05));
        CHECK(p.torque_nm > 300.0);
    }
    Conversion old = cv;
    old.grid_override = g;
    old.grid_override.erase(std::find(old.grid_override.begin(), old.grid_override.end(), kIsaAirPressurePa));
    bool threw = false;
    try {
        ComponentParams co = convert(old);
        const DynoResult ro = run_dyno_sweep(co, nullptr, spec_at({3000.0, 5000.0}), &pool());
        print_dyno("FI7 old axis Preview", ro);
    } catch (const std::exception& ex) {
        threw = true;
        std::printf("FI7 old axis refused: %s\n", ex.what());
    }
    CHECK(threw);
}
// ---- cases-end ----

