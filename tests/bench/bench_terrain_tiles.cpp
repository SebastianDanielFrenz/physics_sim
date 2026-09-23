// tests/bench/bench_terrain_tiles — PLAN.md M1 terrain-tile acceptance:
// "tiles stream around a vehicle driving 50 km without hitches in the
// 240 Hz tick (per-tick cost measured)". Plain main() (not Catch2, so its
// measured numbers are always on stdout), following bench_contact_coupling's
// pattern; registered with CTest via add_test() as a smoke check (must run
// to completion) but asserts a generous, documented bound rather than
// nothing at all - unlike bench_force_elements' PLAN.md-mandated scaling
// number, there is no fixed acceptance threshold for this measurement, so
// the bound here exists to catch a gross regression (an accidental O(n^2)
// or a budget that stopped being honoured), not to certify performance.
//
// Uses the REAL data/tracks/test_ground/terrain.json (PS_SOURCE_DIR, same
// convention as tests/unit/test_scenario_articulation.cpp) - the golden-
// point test and the tile-manager unit tests both exercise this code with
// only a small synthetic spec, so this is the one place that runs the
// actual M1 test-ground spec end to end.
//
// Simulates "driving 50 km" as a position-only exercise of the streaming
// system: an interest point advances in x by just under one tile width
// every tick. This is deliberately NOT a realistic vehicle speed (50 km at
// a real 30 m/s and 240 Hz would take >400k ticks, far more wall-clock time
// than a CI smoke bench should cost) - what matters for this measurement is
// exercising the tile-crossing streaming logic continuously, which depends
// on how many tile boundaries are crossed, not on how much simulated time
// elapses between crossings.

#include "headless_env.h"
#include "ps/terrain/procedural_terrain.h"
#include "ps/world/world.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace ps;

int main() {
    ps_tools::install_headless_crt_handlers();

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined (see tests/bench/CMakeLists.txt)"
#endif
    const std::string terrain_json = std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground/terrain.json";
    auto source = ps::terrain::load_procedural_terrain_source(terrain_json);

    WorldConfig cfg;
    cfg.job_workers = 0; // auto (PLAN.md D8 default)
    World world(cfg);

    ps::terrain::TerrainConfig tc;
    tc.samples_per_tile = 64;      // representative resolution (256 m tile / 64 samples ~= 4 m spacing)
    tc.max_resident_tiles = 400;   // covers the stream radius below with headroom
    tc.max_tile_fills_per_tick = 8;
    world.set_terrain_source(source, tc);

    constexpr real kStreamRadiusM = 600.0; // -> 5x5 = 25 tiles steady-state at 256 m tiles
    constexpr std::uint64_t kInterestId = 1;
    constexpr real kStepM = 250.0;   // just under one 256 m tile per tick
    constexpr int kTicks = 200;      // 200 * 250 m = 50000 m = 50 km

    world.set_terrain_interest_point(kInterestId, Vec3{0, 0, 0}, kStreamRadiusM);

    // Warm-up: let the initial 5x5 tile set fully stream in before timing
    // (this measurement is about STEADY-STATE streaming cost while driving,
    // not the one-time cost of the very first spawn).
    for (int i = 0; i < 50; ++i) world.step();

    double total_ms = 0.0;
    double worst_ms = 0.0;
    int worst_tick = -1;
    std::vector<double> tick_ms;
    tick_ms.reserve(kTicks);

    for (int t = 0; t < kTicks; ++t) {
        world.set_terrain_interest_point(kInterestId, Vec3{static_cast<real>(t + 1) * kStepM, 0, 0}, kStreamRadiusM);

        const auto t0 = std::chrono::steady_clock::now();
        world.step();
        const auto t1 = std::chrono::steady_clock::now();

        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        tick_ms.push_back(ms);
        total_ms += ms;
        if (ms > worst_ms) {
            worst_ms = ms;
            worst_tick = t;
        }
    }

    std::sort(tick_ms.begin(), tick_ms.end());
    const double median_ms = tick_ms[tick_ms.size() / 2];
    const double p99_ms = tick_ms[static_cast<std::size_t>(tick_ms.size() * 0.99)];

    const std::size_t resident = world.terrain_resident_tile_count();
    const std::size_t starved = world.terrain_starved_tile_count();
    const double bytes_per_tile =
        static_cast<double>(tc.samples_per_tile) * tc.samples_per_tile * (sizeof(real) + sizeof(std::uint16_t));
    const double pool_mb = bytes_per_tile * static_cast<double>(tc.max_resident_tiles) / (1024.0 * 1024.0);

    std::printf("bench_terrain_tiles: %d ticks, %.0f km driven (%.0f m/tick)\n", kTicks, kTicks * kStepM / 1000.0, kStepM);
    std::printf("  tick time (ms): avg=%.4f median=%.4f p99=%.4f worst=%.4f (tick #%d)\n", total_ms / kTicks, median_ms,
                 p99_ms, worst_ms, worst_tick);
    std::printf("  resident tiles at end: %zu, starved: %zu\n", resident, starved);
    std::printf("  tile sample pool: %u x %u samples/tile, %zu tiles resident capacity, ~%.2f MB\n", tc.samples_per_tile,
                 tc.samples_per_tile, tc.max_resident_tiles, pool_mb);

    // Smoke bound, not a PLAN.md-mandated number (see header comment) - and
    // deliberately generous rather than tuned to the 240 Hz/4.17 ms tick
    // budget: this bench runs under every CI leg, including unoptimized
    // debug and ASan builds where wall-clock cost is not representative of
    // release (same caveat every other bench in this directory documents -
    // e.g. bench_force_elements/bench_contact_coupling). Measured on this
    // machine: debug ~58-70 ms/tick, release ~1-2 ms/tick (see this file's
    // companion note in out/handover_terrain.md for the exact numbers this
    // task's report cites). 250 ms is far above even the debug number with
    // margin, while still catching an actual hang/regression (e.g. the
    // per-tick fill budget silently stopping being honoured).
    if (worst_ms > 250.0) {
        std::fprintf(stderr, "bench_terrain_tiles: worst tick %.4f ms exceeds the 250 ms smoke bound\n", worst_ms);
        return 1;
    }
    if (starved != 0) {
        std::fprintf(stderr, "bench_terrain_tiles: %zu starved tiles (pool too small for the stream radius)\n", starved);
        return 1;
    }
    return 0;
}
