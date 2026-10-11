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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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

} // namespace

TEST_CASE("FI-X2 exploration: Sketch on a boosted lumped grid with and without an ambient node", "[.][diag][forced_induction]") {
    Conversion hc;
    hc.base = "toyota_a25a_fks_gen.json";
    hc.fi = "single_medium.json";
    const std::vector<real> helper = conversion_grid(hc);
    std::printf("FI-X2 helper grid: %s\n", kpa_list(helper).c_str());
    std::vector<real> with_amb = helper;
    with_amb.insert(with_amb.end() - 2, 101325.0);
    with_amb.back() = 200000.0;
    for (const auto& g : {helper, with_amb}) {
        try {
            Conversion cv;
            cv.base = "toyota_a25a_fks_gen.json";
            cv.fi = "single_medium.json";
            cv.q = MapQuality::Preview;
            cv.grid_override = g;
            ComponentParams c = convert(cv);
            const DynoResult r = run_dyno_sweep(c, nullptr, spec_at({1000, 3000, 5000}), &pool());
            std::printf("FI-X2 grid %s\n", kpa_list(g).c_str());
            print_dyno("FI-X2", r);
        } catch (const std::exception& e) {
            std::printf("FI-X2 grid %s: EXCEPTION %s\n", kpa_list(g).c_str(), e.what());
        }
    }
}

TEST_CASE("FI-X1 exploration: conversion table", "[.][diag][forced_induction]") {
    struct Row {
        const char* base;
        const char* fi;
        real cr;
        std::vector<double> rpm;
    };
    const std::vector<Row> rows = {
        {"hyper_v8_one1_audio.json", "single_large.json", 0.0, {2000, 3000, 4000, 5000, 6000, 7000, 8000}},
        {"hyper_v8_one1_audio.json", "twin_medium.json", 0.0, {2000, 3000, 4000, 5000, 6000, 7000, 8000}},
        {"hyper_v8_one1_audio.json", "twin_small.json", 0.0, {2000, 3000, 4000, 5000, 6000, 7000, 8000}},
        {"hyper_v8_one1_audio.json", "twin_large.json", 0.0, {2000, 3000, 4000, 5000, 6000, 7000, 8000}},
        {"demo_i4_pipes.json", "single_small.json", 0.0, {1000, 1250, 1500, 1750}},
        {"demo_i4_pipes.json", "single_medium.json", 0.0, {2250, 2500, 2750}},
        {"demo_i4_pipes.json", "single_large.json", 0.0, {3250, 3500, 3750}},
    };
    for (const Row& row : rows) {
        try {
            const auto t0 = std::chrono::steady_clock::now();
            Conversion cv;
            cv.base = row.base;
            cv.fi = row.fi;
            cv.compression_ratio = row.cr;
            cv.q = MapQuality::Sketch;
            ComponentParams c = convert(cv);
            const double gen = seconds_since(t0);
            std::printf("FI-X1 %s + '%s' CR %.1f: loaded %.2f s\n", row.base, row.fi, row.cr, gen);
            const DynoResult r = run_dyno_sweep(c, nullptr, spec_at(row.rpm), &pool());
            std::printf("FI-X1 total %.2f s\n", seconds_since(t0));
            print_dyno("FI-X1", r);
        } catch (const std::exception& e) {
            std::printf("FI-X1 %s + '%s' CR %.1f: EXCEPTION %s\n", row.base, row.fi, row.cr, e.what());
        }
    }
}
