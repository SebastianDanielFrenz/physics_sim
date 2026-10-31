// tests/unit/test_held_rpm_rows.cpp - held rpm rows in the engine map generator.
//
// Owner 2026-10-23: a rev limit raised in the garage extends the engine's grid_rpm past its breathing limit, and
// those rows must give maps, not a load error. When the maps cannot be assembled, EngineMapGenTask::finish holds the
// rows r0..top (r0 = the lowest rpm row whose points fail: trim not settled, throttle saturated, no net fresh air,
// VE/eta_i out of range, air not strictly increasing in p_im, ...): they are generated as copies of row r0 - 1 - the
// old Table2D edge clamp, at the engine's breathing limit. Held when every row from r0 up fails, or when the top
// manifold node's air flow at r0 - 1, or on every valid row above r0, lies below its maximum P over the rows below r0
// (an isolated failure between computable rows past the breathing limit; the hypercar at a 20000 rpm limit fails only
// at 17062.5 rpm, the generated A25A only at 13450 rpm). A failure in row 0, or one followed by a valid row reaching P
// while the flow at r0 - 1 is P, throws the unchanged message.
//
// Fixtures (measured, Preview): the generic lumped I4 with grid_p_im 10/20/30 kPa fails at 18000 and 30000 rpm with
// 24000 valid (no breathing node at 30 kPa) - held 18000..30000 from 13000 (top-node air flow at 13000 rpm 0.218 vs
// its peak 0.853 below it, trapped kg per cycle x rpm); the boosted pipe I4 with 12/20 kPa fails at
// 13000..30000 - held from 9000. No fixture exercises the wide-open-throttle row path of an ambient-fed pipe engine:
// its WOT row always breathes.
//
// Recorded sabotages (2026-10-24, each fails only the named cases):
//   - case (b)'s air-flow guard removed (every isolated failure held): "held: a failure followed by more air flow
//     still throws" (nothing thrown).
//   - the "never comes back" half of the guard removed: "held: a failure right after the air-flow peak is held ..."
//     (it throws).
//   - held rows built from row r0 - 2: both "... past its breathing limit" cases and the "right after the peak" case
//     (168 checks; lumped VE 0.2826 = the 9000 rpm row vs 0.0299 at 13000 rpm).

#define _CRT_SECURE_NO_WARNINGS // std::getenv in the hidden diagnostics

#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"
#include "mvem_fixtures.h"
#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/fuel_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <exception>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
namespace fx = ps::test::mvem;
using Catch::Matchers::ContainsSubstring;

namespace {

CycleModelDesc lumped_engine() {
    CycleModelDesc d = ps::test::make_generic_i4_cycle_desc();
    d.grid_p_im = {30000.0, 60000.0, 95000.0};
    d.friction.model = FrictionModelDesc::Model::PolynomialPistonSpeed;
    d.friction.a_pa = 66619.52;
    d.friction.b_pa = 91.085;
    d.friction.c_pa = 346.81;
    return d;
}

CycleModelDesc boosted_pipe_engine() {
    CycleModelDesc d = ps::test::make_pipe_i4_cycle_desc();
    d.intake->compressor_supply = true; // the hyper's path: no WOT rows, trim failures deferred from run_point
    d.grid_p_im = {30000.0, 60000.0, 90000.0, 150000.0};
    return d;
}

CycleModelDesc ambient_pipe_engine() { return ps::test::make_pipe_i4_cycle_desc(); }

void print_maps(const char* name, const EngineMapsPtr& m, double sec) {
    const Table2D& ve = m->maps.ve;
    std::printf("%s: %zu x %zu, held_rpm_rows %d, nonbreathing %d, held plenum %d, wot dropped %d, %.2f s, hash 0x%016llx\n",
                name, ve.x.size(), ve.y.size(), m->held_rpm_rows, m->nonbreathing_points, m->held_plenum_points,
                m->wot_dropped_rows, sec, static_cast<unsigned long long>(m->content_hash));
    for (std::size_t ir = 0; ir < ve.x.size(); ++ir) {
        std::printf("  %7.0f rpm VE", ve.x[ir]);
        for (std::size_t ip = 0; ip < ve.y.size(); ++ip) std::printf(" %.4f", ve.values[ir * ve.y.size() + ip]);
        std::printf("\n");
    }
}

// One rpm on its own (a two-row grid rpm-1, rpm): prints whether it generates and, if not, why.
void probe_rpm(const char* name, CycleModelDesc d, real rpm, MapQuality q) {
    d.grid_rpm = {rpm - 1.0, rpm};
    try {
        const EngineMapsPtr m = generate_engine_maps(d, fx::gasoline(), q, nullptr);
        std::printf("  %s %7.0f rpm: ok (held %d)\n", name, rpm, m->held_rpm_rows);
    } catch (const std::exception& e) { // hidden diagnostic only (never on the asan leg)
        std::printf("  %s %7.0f rpm: %s\n", name, rpm, e.what());
    }
}

} // namespace

TEST_CASE("held rpm rows diag: breathing limits of the fixtures", "[.][diag][cycle][drivetrain]") {
    jobs::JobSystem pool(10);
    const std::vector<real> lumped_rpm = {1000.0, 2500.0, 4000.0, 5500.0, 7000.0, 9000.0, 12000.0, 16000.0, 20000.0, 25000.0};
    const std::vector<real> pipe_rpm = {1500.0, 3000.0, 4500.0, 6000.0, 8000.0, 11000.0, 14000.0, 18000.0, 22000.0};
    struct Case {
        const char* name;
        CycleModelDesc d;
        std::vector<real> rpm;
    };
    for (Case c : {Case{"lumped", lumped_engine(), lumped_rpm}, Case{"boosted pipe", boosted_pipe_engine(), pipe_rpm},
                   Case{"ambient pipe", ambient_pipe_engine(), pipe_rpm}}) {
        for (real rpm : c.rpm) probe_rpm(c.name, c.d, rpm, MapQuality::Preview);
        c.d.grid_rpm = c.rpm;
        const auto t0 = std::chrono::steady_clock::now();
        try {
            const EngineMapsPtr m = generate_engine_maps(c.d, fx::gasoline(), MapQuality::Preview, &pool);
            print_maps(c.name, m, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        } catch (const std::exception& e) {
            std::printf("%s full grid: %s\n", c.name, e.what());
        }
    }
}

// Hidden: the hypercar engine with its grid_rpm extended the way racing_game extends it for a 20000 rpm limit (the
// last spacing widened so at most 8 nodes are added). PS_HELD_DIAG_ENGINE overrides the engine file,
// PS_HELD_DIAG_QUALITY (sketch/preview/full, default full) the quality.
TEST_CASE("held rpm rows diag: hypercar at a 20000 rpm limit", "[.][diag][cycle][drivetrain]") {
    jobs::JobSystem pool(10);
    std::string path = std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json";
    if (const char* e = std::getenv("PS_HELD_DIAG_ENGINE")) path = e;
    MapQuality q = MapQuality::Full;
    if (const char* e = std::getenv("PS_HELD_DIAG_QUALITY")) {
        const std::string v = e;
        q = v == "sketch" ? MapQuality::Sketch : v == "preview" ? MapQuality::Preview : MapQuality::Full;
    }
    io::EngineMapOptions opt;
    opt.generate = false;
    const auto loaded = io::load_engine_json(path, opt);
    const auto& eng = std::get<SimulatedEngineDesc>(loaded);
    REQUIRE(eng.cycle.has_value());
    CycleModelDesc d = *eng.cycle;
    const real top = d.grid_rpm.back(), step0 = top - d.grid_rpm[d.grid_rpm.size() - 2];
    const real step = std::max(step0, (real(20000.0) - top) / real(8.0));
    for (real n = top + step; real(20000.0) - n > real(0.5) * step; n += step) d.grid_rpm.push_back(n);
    d.grid_rpm.push_back(20000.0);
    for (real rpm : d.grid_rpm) {
        if (rpm < top) continue;
        CycleModelDesc p = d;
        p.grid_rpm = {rpm - 1.0, rpm};
        try {
            const EngineMapsPtr m = generate_engine_maps(p, eng.fuels.front(), q, &pool);
            std::printf("  %8.2f rpm: ok (held %d)\n", rpm, m->held_rpm_rows);
        } catch (const std::exception& e) {
            std::printf("  %8.2f rpm: %s\n", rpm, e.what());
        }
    }
    const auto t0 = std::chrono::steady_clock::now();
    try {
        const EngineMapsPtr m = generate_engine_maps(d, eng.fuels.front(), q, &pool);
        print_maps("hyper 20000", m, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    } catch (const std::exception& e) {
        std::printf("hyper 20000 full grid: %s\n", e.what());
    }
}

TEST_CASE("held rpm rows diag: low-pressure grids", "[.][diag][cycle][drivetrain]") {
    jobs::JobSystem pool(10);
    const std::vector<real> rpm = {1500.0, 3000.0, 4500.0, 6000.0, 9000.0, 13000.0, 18000.0, 24000.0, 30000.0};
    struct Case {
        const char* name;
        CycleModelDesc d;
    };
    CycleModelDesc lumped = lumped_engine();
    lumped.grid_p_im = {10000.0, 20000.0, 30000.0};
    CycleModelDesc boosted = boosted_pipe_engine();
    boosted.grid_p_im = {12000.0, 20000.0, 30000.0};
    CycleModelDesc ambient = ambient_pipe_engine();
    ambient.grid_p_im = {12000.0, 20000.0, 30000.0};
    for (Case c : {Case{"lumped low p", lumped}, Case{"boosted pipe low p", boosted}, Case{"ambient pipe low p", ambient}}) {
        for (real r : rpm) probe_rpm(c.name, c.d, r, MapQuality::Preview);
        c.d.grid_rpm = rpm;
        const auto t0 = std::chrono::steady_clock::now();
        try {
            const EngineMapsPtr m = generate_engine_maps(c.d, fx::gasoline(), MapQuality::Preview, &pool);
            print_maps(c.name, m, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        } catch (const std::exception& e) {
            std::printf("%s full grid: %s\n", c.name, e.what());
        }
    }
}

TEST_CASE("held rpm rows diag: pipe engines with a closed-throttle grid", "[.][diag][cycle][drivetrain]") {
    jobs::JobSystem pool(10);
    const std::vector<real> rpm = {1500.0, 3000.0, 4500.0, 6000.0, 9000.0, 13000.0, 18000.0, 24000.0, 30000.0};
    for (const std::vector<real>& p : {std::vector<real>{10000.0, 15000.0}, std::vector<real>{12000.0, 20000.0}}) {
        CycleModelDesc boosted = boosted_pipe_engine();
        boosted.grid_p_im = p;
        CycleModelDesc ambient = ambient_pipe_engine();
        ambient.grid_p_im = p;
        for (CycleModelDesc d : {boosted, ambient}) {
            const char* name = d.intake->compressor_supply ? "boosted pipe" : "ambient pipe";
            std::printf("%s, grid_p_im %.0f..%.0f Pa\n", name, p.front(), p.back());
            for (real r : rpm) probe_rpm(name, d, r, MapQuality::Preview);
            d.grid_rpm = rpm;
            try {
                const EngineMapsPtr m = generate_engine_maps(d, fx::gasoline(), MapQuality::Preview, &pool);
                print_maps(name, m, 0.0);
            } catch (const std::exception& e) {
                std::printf("%s full grid: %s\n", name, e.what());
            }
        }
    }
}

TEST_CASE("held rpm rows diag: trim failures of a boosted pipe engine", "[.][diag][cycle][drivetrain]") {
    CycleModelDesc d = boosted_pipe_engine();
    d.grid_p_im = {25000.0, 50000.0, 100000.0, 160000.0};
    for (real r = 7000.0; r <= 20000.0; r += 1000.0) probe_rpm("boosted pipe", d, r, MapQuality::Sketch);
}

namespace {

const std::vector<real> kPastBreathingRpm = {1500.0, 3000.0, 4500.0, 6000.0, 9000.0, 13000.0, 18000.0, 24000.0, 30000.0};

CycleModelDesc lumped_past_breathing() {
    CycleModelDesc d = lumped_engine();
    d.grid_p_im = {10000.0, 20000.0, 30000.0};
    d.grid_rpm = kPastBreathingRpm;
    return d;
}

CycleModelDesc pipe_past_breathing() {
    CycleModelDesc d = boosted_pipe_engine();
    d.grid_p_im = {12000.0, 20000.0};
    d.grid_rpm = kPastBreathingRpm;
    return d;
}

// Rows r0.. of every rpm x load table equal row src exactly (the enthalpy flow is per cycle, so it scales with rpm).
void check_rows_held(const EngineMaps& m, std::size_t r0, std::size_t src) {
    std::vector<const Table2D*> tabs = {&m.maps.ve, &m.maps.eta_i, &m.t_exh_k}; // theta_mbt_deg is a constant 2 x 2 table
    if (m.maps.has_pmep_excess) tabs.push_back(&m.maps.pmep_excess_pa);
    if (m.maps.has_theta_kl) tabs.push_back(&m.maps.theta_kl_deg);
    for (const Table2D* t : tabs) {
        const std::size_t ny = t->y.size();
        REQUIRE(t->x.size() == kPastBreathingRpm.size());
        for (std::size_t ir = r0; ir < t->x.size(); ++ir) {
            CHECK(t->x[ir] == kPastBreathingRpm[ir]);
            for (std::size_t ip = 0; ip < ny; ++ip) CHECK(t->values[ir * ny + ip] == t->values[src * ny + ip]);
        }
    }
    const Table2D& h = m.h_exh_w;
    const std::size_t ny = h.y.size();
    for (std::size_t ir = r0; ir < h.x.size(); ++ir) {
        for (std::size_t ip = 0; ip < ny; ++ip) {
            const real per_rev = h.values[ir * ny + ip] / h.x[ir], src_per_rev = h.values[src * ny + ip] / h.x[src];
            CHECK(std::abs(per_rev - src_per_rev) <= real(1e-12) * std::max(real(1.0), std::abs(src_per_rev)));
        }
    }
}

// The rows below r0 equal a generation of the grid truncated to them (the eta_i load axis included: a held row
// adds the same air masses per revolution as its source row).
void check_rows_below(const EngineMaps& m, const EngineMaps& truncated, std::size_t r0) {
    CHECK(truncated.held_rpm_rows == 0);
    const std::pair<const Table2D*, const Table2D*> tabs[] = {{&m.maps.ve, &truncated.maps.ve},
                                                              {&m.maps.eta_i, &truncated.maps.eta_i},
                                                              {&m.maps.pmep_excess_pa, &truncated.maps.pmep_excess_pa},
                                                              {&m.h_exh_w, &truncated.h_exh_w}};
    for (const auto& [a, b] : tabs) {
        REQUIRE(a->y == b->y);
        REQUIRE(b->x.size() == r0);
        const std::size_t ny = a->y.size();
        for (std::size_t k = 0; k < r0 * ny; ++k) CHECK(a->values[k] == b->values[k]);
    }
}

void check_held_case(const CycleModelDesc& d, std::size_t r0, int held) {
    jobs::JobSystem pool(10);
    const EngineMapsPtr m = generate_engine_maps(d, fx::gasoline(), MapQuality::Preview, &pool);
    CHECK(m->held_rpm_rows == held);
    check_rows_held(*m, r0, r0 - 1);
    CycleModelDesc t = d;
    t.grid_rpm.resize(r0);
    const EngineMapsPtr mt = generate_engine_maps(t, fx::gasoline(), MapQuality::Preview, &pool);
    check_rows_below(*m, *mt, r0);
}

} // namespace

TEST_CASE("held: lumped engine past its breathing limit (isolated failure, falling air flow)", "[cycle][drivetrain]") {
    // 18000 and 30000 rpm fail, 24000 does not: held 18000..30000 from 13000.
    check_held_case(lumped_past_breathing(), 6, 3);
}

TEST_CASE("held: boosted pipe engine past its breathing limit (failing top rows)", "[cycle][drivetrain][pipe]") {
    // 13000..30000 rpm fail: held from 9000.
    check_held_case(pipe_past_breathing(), 5, 4);
}

TEST_CASE("held: a failure followed by more air flow still throws", "[cycle][drivetrain]") {
    // 10000 rpm fails between valid rows, and the top node's air flow at 13000 rpm exceeds every row below the failure
    // (600 rpm): not past the breathing limit on this grid.
    CycleModelDesc d = lumped_past_breathing();
    d.grid_rpm = {600.0, 10000.0, 13000.0};
    CHECK_THROWS_WITH(generate_engine_maps(d, fx::gasoline(), MapQuality::Preview, nullptr),
                      ContainsSubstring("no breathing node above the held node at 10000.000000 rpm"));
}

TEST_CASE("held: a failure right after the air-flow peak is held when the flow does not come back", "[cycle][drivetrain]") {
    // 18000 rpm fails right after the peak row 4500 rpm; the valid 24000 rpm row stays below that peak.
    CycleModelDesc d = lumped_past_breathing();
    d.grid_rpm = {1500.0, 3000.0, 4500.0, 18000.0, 24000.0};
    const EngineMapsPtr m = generate_engine_maps(d, fx::gasoline(), MapQuality::Preview, nullptr);
    CHECK(m->held_rpm_rows == 2);
    const Table2D& ve = m->maps.ve;
    const std::size_t ny = ve.y.size();
    for (std::size_t ir = 3; ir < 5; ++ir)
        for (std::size_t ip = 0; ip < ny; ++ip) CHECK(ve.values[ir * ny + ip] == ve.values[2 * ny + ip]);
}

TEST_CASE("held: no valid row throws", "[cycle][drivetrain]") {
    CycleModelDesc d = lumped_past_breathing();
    d.grid_rpm = {18000.0, 30000.0};
    CHECK_THROWS_WITH(generate_engine_maps(d, fx::gasoline(), MapQuality::Preview, nullptr),
                      ContainsSubstring("no breathing node above the held node at 18000.000000 rpm"));
}

void check_workers(const CycleModelDesc& d) {
    jobs::JobSystem one(1), ten(10);
    const EngineMapsPtr a = generate_engine_maps(d, fx::gasoline(), MapQuality::Sketch, &one);
    const EngineMapsPtr b = generate_engine_maps(d, fx::gasoline(), MapQuality::Sketch, &ten);
    CHECK(a->held_rpm_rows > 0);
    CHECK(a->held_rpm_rows == b->held_rpm_rows);
    CHECK(a->content_hash == b->content_hash);
    CHECK(a->maps.ve.values == b->maps.ve.values);
    CHECK(a->maps.eta_i.values == b->maps.eta_i.values);
    CHECK(a->h_exh_w.values == b->h_exh_w.values);
}

TEST_CASE("held: 1 vs 10 workers give the same maps (lumped)", "[cycle][drivetrain]") {
    check_workers(lumped_past_breathing());
}

TEST_CASE("held: 1 vs 10 workers give the same maps (boosted pipe)", "[cycle][drivetrain][pipe]") {
    CycleModelDesc d = pipe_past_breathing();
    d.grid_rpm = {4500.0, 9000.0, 13000.0, 18000.0}; // held 13000, 18000 from 9000
    check_workers(d);
}

// Hidden: the garage advisor's generation of an extended engine (racing_game: adaptive pressure sampling as
// vehicle_setup.cpp requests it for a garage edit, Sketch quality). PS_HELD_DIAG_ENGINE, PS_HELD_DIAG_QUALITY as above.
TEST_CASE("held rpm rows diag: advisor-style adaptive generation at a 20000 rpm limit", "[.][diag][cycle][drivetrain]") {
    jobs::JobSystem pool(10);
    std::string path = std::string(PS_SOURCE_DIR) + "/data/engines/toyota_a25a_fks_gen.json";
    if (const char* e = std::getenv("PS_HELD_DIAG_ENGINE")) path = e;
    MapQuality q = MapQuality::Sketch;
    if (const char* e = std::getenv("PS_HELD_DIAG_QUALITY")) {
        const std::string v = e;
        q = v == "full" ? MapQuality::Full : v == "preview" ? MapQuality::Preview : MapQuality::Sketch;
    }
    io::EngineMapOptions opt;
    opt.generate = false;
    const auto loaded = io::load_engine_json(path, opt);
    const auto& eng = std::get<SimulatedEngineDesc>(loaded);
    REQUIRE(eng.cycle.has_value());
    CycleModelDesc d = *eng.cycle;
    const real top = d.grid_rpm.back(), step0 = top - d.grid_rpm[d.grid_rpm.size() - 2];
    const real step = std::max(step0, (real(20000.0) - top) / real(8.0));
    for (real n = top + step; real(20000.0) - n > real(0.5) * step; n += step) d.grid_rpm.push_back(n);
    d.grid_rpm.push_back(20000.0);
    d.pressure_sampling.enabled = true;
    d.pressure_sampling.allow_budget_limit = true;
    d.pressure_sampling.min_pa = d.grid_p_im.front();
    d.pressure_sampling.max_pa = d.grid_p_im.back();
    d.pressure_sampling.relative_error = 0.05;
    d.pressure_sampling.max_points = 24;
    d.pressure_sampling.max_passes = 8;
    std::printf("grid_p_im %.0f..%.0f Pa (%zu), %zu rpm rows\n", d.grid_p_im.front(), d.grid_p_im.back(), d.grid_p_im.size(),
                d.grid_rpm.size());
    try {
        const EngineMapsPtr m = generate_engine_maps(d, eng.fuels.front(), q, &pool);
        print_maps("adaptive 20000", m, 0.0);
    } catch (const std::exception& e) {
        std::printf("adaptive 20000: %s\n", e.what());
    }
}

TEST_CASE("held rpm rows diag: low rpm", "[.][diag][cycle][drivetrain]") {
    CycleModelDesc lumped = lumped_engine();
    lumped.grid_p_im = {10000.0, 20000.0, 30000.0};
    CycleModelDesc boosted = boosted_pipe_engine();
    boosted.grid_p_im = {12000.0, 20000.0};
    for (real r : {150.0, 250.0, 400.0, 600.0, 800.0, 1000.0, 1200.0, 7000.0, 8000.0, 10000.0, 11000.0, 12000.0}) {
        probe_rpm("lumped low p", lumped, r, MapQuality::Preview);
        probe_rpm("boosted low p", boosted, r, MapQuality::Preview);
    }
}
