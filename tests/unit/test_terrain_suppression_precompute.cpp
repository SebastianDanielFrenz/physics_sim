// tests/unit/test_terrain_suppression_precompute.cpp - PHYS-021 Task B:
// TileManager::precompute_suppression (load-time, parallel across tiles)
// must produce hole lists bit-identical to the lazy per-fill path's, for
// every precomputed tile, independent of worker count, and a runtime fill
// of a precomputed tile must be a cache hit. White-box (core/src on the
// include path): ps::terrain::TileManager is internal.
//
// Uses the real test_ground static collision + terrain spec (PS_SOURCE_DIR),
// loaded ONCE for all checks - the collision load is the slow part on the
// WSL sanitizer legs.

#include "terrain/tile_manager.h"

#include "ps/io/surface_table.h"
#include "ps/io/test_ground_loader.h"
#include "ps/jobs/job_system.h"
#include "ps/terrain/procedural_terrain.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ps;
using ps::terrain::TerrainConfig;
using ps::terrain::TileKey;
using ps::terrain::TileManager;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

std::map<std::int64_t, std::vector<std::uint32_t>> snapshot_precomputed(const TileManager& mgr) {
    std::map<std::int64_t, std::vector<std::uint32_t>> out;
    for (const TileKey& k : mgr.precomputed_keys()) {
        const std::vector<std::uint32_t>* cached = mgr.cached_suppression(k);
        REQUIRE(cached != nullptr);
        out.emplace(k.packed, *cached);
    }
    return out;
}

} // namespace

TEST_CASE("Load-time suppression precompute is bit-identical to the lazy path, worker-count invariant, and hit at runtime",
          "[terrain][tiles][PHYS-021][precompute]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    const std::string track_dir = std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground";
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    io::load_test_ground_collision(world, track_dir, *surfaces);
    std::shared_ptr<const terrain::ITerrainSource> source = terrain::load_procedural_terrain_source(track_dir + "/terrain.json", 64);

    TerrainConfig tc;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 64;

    std::map<std::int64_t, std::vector<std::uint32_t>> reference;
    {
        jobs::JobSystem jobs(8);
        TileManager mgr(source, tc, world.backend());
        mgr.precompute_suppression(jobs);
        REQUIRE(mgr.precomputed_tile_count() > 0);
        reference = snapshot_precomputed(mgr);

        // 1. Bit-identical to the lazy path, for EVERY precomputed tile.
        std::size_t tiles_with_holes = 0;
        std::size_t total_holes = 0;
        double worst_tile_ms = 0.0;
        double sum_ms = 0.0;
        for (const TileKey& k : mgr.precomputed_keys()) {
            const auto t0 = std::chrono::steady_clock::now();
            const std::vector<std::uint32_t> lazy = mgr.compute_suppression_uncached(k);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            worst_tile_ms = std::max(worst_tile_ms, ms);
            sum_ms += ms;
            INFO("tile key " << k.packed);
            REQUIRE(lazy == reference.at(k.packed));
            if (!lazy.empty()) ++tiles_with_holes;
            total_holes += lazy.size();
        }
        // The comparison is only meaningful if real holes exist.
        REQUIRE(tiles_with_holes > 0);
        std::printf("test_terrain_suppression_precompute: %zu tiles precomputed, %zu with holes, %zu holes total; "
                    "single-thread lazy cost sum=%.1f ms, worst single tile=%.1f ms\n",
                    mgr.precomputed_tile_count(), tiles_with_holes, total_holes, sum_ms, worst_tile_ms);
    }

    // 2. Worker-count invariance: a fresh manager precomputed on a
    // 2-worker pool yields the identical key set and hole lists.
    {
        jobs::JobSystem jobs(2);
        TileManager mgr(source, tc, world.backend());
        mgr.precompute_suppression(jobs);
        REQUIRE(snapshot_precomputed(mgr) == reference);
    }

    // 3. Runtime fills: streaming around the split_mu spawn after the
    // precompute takes the lazy path ONLY for tiles that were not
    // precomputed, and every such tile's lazy result is empty (nothing
    // static touches it, so the triangle binning missed nothing).
    {
        jobs::JobSystem jobs(2);
        TileManager mgr(source, tc, world.backend());
        mgr.precompute_suppression(jobs);
        const Vec3 kSplitMu{150.0, -480.0, 0.0};
        constexpr real kRadius = 700.0; // 7x7 = 49 tiles at 256 m
        mgr.set_interest_point(1, kSplitMu, kRadius);
        for (int i = 0; i < 4; ++i) mgr.update(jobs);
        REQUIRE(mgr.starved_tile_count() == 0);

        std::set<std::int64_t> precomputed;
        for (const TileKey& k : mgr.precomputed_keys()) precomputed.insert(k.packed);
        const TileKey center = source->key_for_world_xy(kSplitMu.x, kSplitMu.y);
        const auto r = static_cast<std::int32_t>(std::ceil(kRadius / source->tile_size_m()));
        std::size_t streamed = 0;
        std::size_t streamed_precomputed = 0;
        for (std::int32_t dy = -r; dy <= r; ++dy) {
            for (std::int32_t dx = -r; dx <= r; ++dx) {
                const TileKey k = source->key_offset(center, dx, dy);
                ++streamed;
                const std::vector<std::uint32_t>* cached = mgr.cached_suppression(k);
                REQUIRE(cached != nullptr);
                if (precomputed.count(k.packed)) {
                    ++streamed_precomputed;
                } else {
                    INFO("non-precomputed tile key " << k.packed);
                    REQUIRE(cached->empty());
                }
            }
        }
        REQUIRE(mgr.resident_tile_count() == streamed);
        REQUIRE(streamed_precomputed > 0);
        REQUIRE(mgr.lazy_suppression_count() == streamed - streamed_precomputed);
    }
}
