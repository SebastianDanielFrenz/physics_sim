// PLAN.md M1 tile-system acceptance: tiles stream around an interest point
// without hitches, no seams between tiles (continuous height/surface across
// a border), determinism-friendly (no allocation once warmed up). White-box
// (core/src on the include path) since ps::terrain::TileManager is internal.

#include "terrain/procedural_terrain_source.h"
#include "terrain/tile_manager.h"

#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>

using namespace ps;
using namespace ps::terrain;

namespace {

// A small, hand-built spec (not the real ~50 km test_ground one) so this
// test is fast and self-contained - one rolling octave, no flat zones, one
// surface rule, a small tile size for a manageable sample count in tests.
TerrainSpec make_test_spec() {
    TerrainSpec spec;
    spec.seed = 12345;
    spec.rolling_octaves.push_back(Octave{50.0, 3.0, 1});
    spec.rolling_octaves.push_back(Octave{17.0, 0.8, 2});
    spec.hill_octave = Octave{200.0, 10.0, 5};
    spec.roughness_near_m = 1.0e9; // effectively disables the hill octave (no zone is ever "far")
    spec.roughness_far_m = 2.0e9;
    spec.default_surface = "grass";
    spec.surface_rules.push_back(SurfaceRule{"dirt", 40.0, 101, 0.5});
    spec.surface_names = {"grass", "dirt"};
    spec.tile_size_m = 32.0;
    return spec;
}

} // namespace

TEST_CASE("Adjacent tiles agree exactly at their shared border (no seams)", "[terrain][tiles][seams]") {
    const TerrainSpec spec = make_test_spec();
    ProceduralTerrainSource source(spec, /*samples_per_tile=*/8);

    const TileKey a = source.key_for_world_xy(0.0, 0.0);
    const TileKey b = source.key_offset(a, 1, 0); // neighbour to the east
    const TileKey c = source.key_offset(a, 0, 1); // neighbour to the north

    TileSample sa, sb, sc;
    const std::uint32_t n = source.samples_per_tile();
    sa.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
    sa.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), 0);
    sb = sa;
    sc = sa;

    source.fill_tile(a, sa);
    source.fill_tile(b, sb);
    source.fill_tile(c, sc);

    // a's rightmost column (ix = n-1) must equal b's leftmost column (ix = 0) -
    // both are the SAME world (x, y) samples (see tile_manager.h's header
    // comment: the shared boundary lands on the same world coordinate by
    // construction), so this must hold EXACTLY, not just within a tolerance.
    for (std::uint32_t iy = 0; iy < n; ++iy) {
        const real ha = sa.heights[iy * n + (n - 1)];
        const real hb = sb.heights[iy * n + 0];
        INFO("iy=" << iy << " a.right=" << ha << " b.left=" << hb);
        REQUIRE(ha == hb);
    }
    // a's topmost row (iy = n-1) must equal c's bottommost row (iy = 0).
    for (std::uint32_t ix = 0; ix < n; ++ix) {
        const real ha = sa.heights[(n - 1) * n + ix];
        const real hc = sc.heights[0 * n + ix];
        INFO("ix=" << ix << " a.top=" << ha << " c.bottom=" << hc);
        REQUIRE(ha == hc);
    }
}

TEST_CASE("TileManager streams tiles around an interest point and reports resident count", "[terrain][tiles]") {
    auto source = std::make_shared<ProceduralTerrainSource>(make_test_spec(), 8);
    ps::jobs::JobSystem jobs(2);

    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    TerrainConfig tc;
    tc.samples_per_tile = 8;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 100; // no throttling for this test - just check the end state
    TileManager mgr(source, tc, world.backend());

    mgr.set_interest_point(1, Vec3{0, 0, 0}, 64.0); // radius 64 m, tile_size 32 m -> 5x5 = 25 tiles needed
    for (int i = 0; i < 5; ++i) mgr.update(jobs); // a few ticks to fully stream in (well under the fill budget anyway)

    REQUIRE(mgr.resident_tile_count() == 25);
    REQUIRE(mgr.starved_tile_count() == 0);
}

TEST_CASE("TileManager throttles fills per tick and streams in over several ticks", "[terrain][tiles]") {
    auto source = std::make_shared<ProceduralTerrainSource>(make_test_spec(), 8);
    ps::jobs::JobSystem jobs(2);
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    TerrainConfig tc;
    tc.samples_per_tile = 8;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 3;
    TileManager mgr(source, tc, world.backend());

    mgr.set_interest_point(1, Vec3{0, 0, 0}, 64.0); // 25 tiles needed
    mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() == 3); // exactly the per-tick budget on the first tick

    std::size_t ticks = 1;
    while (mgr.resident_tile_count() < 25 && ticks < 100) {
        mgr.update(jobs);
        ++ticks;
    }
    REQUIRE(mgr.resident_tile_count() == 25);
    REQUIRE(ticks == 9); // ceil(25 / 3)
}

TEST_CASE("World::set_terrain_source wires into the terrain_tiles pipeline stage", "[terrain][tiles][world]") {
    auto source = std::make_shared<ProceduralTerrainSource>(make_test_spec(), 8);

    WorldConfig cfg;
    cfg.job_workers = 2;
    World world(cfg);

    TerrainConfig tc;
    tc.samples_per_tile = 8;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 25;
    world.set_terrain_source(source, tc);
    world.set_terrain_interest_point(7, Vec3{0, 0, 0}, 64.0);

    for (int i = 0; i < 4; ++i) world.step();
    REQUIRE(world.terrain_resident_tile_count() == 25);
    REQUIRE(world.terrain_starved_tile_count() == 0);

    // A body dropped over the streamed-in terrain should find collision
    // geometry there (proves update_heightfield_body + set_pose actually
    // reached a real Jolt body, not just bookkeeping).
    BodyDesc ball;
    ball.shape = SphereShape{0.3};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{5.0, 5.0, 50.0};
    ball.allow_sleep = false;
    const BodyId id = world.create_body(ball);
    const real z0 = world.get_pose(id).position.z;
    for (int i = 0; i < static_cast<int>(3.0 * 240.0); ++i) world.step();
    const real z1 = world.get_pose(id).position.z;
    REQUIRE(z1 < z0); // fell and (given the terrain amplitude here, +-3.8 m) should have landed well before z=50
    REQUIRE(z1 > -20.0);
}

// Zero-allocation scope (see tile_manager.h's header comment): TileManager
// deliberately does NOT claim allocation-free behaviour for the (bounded,
// infrequent) bookkeeping that runs exactly when a tile crosses into or out
// of the needed set - std::unordered_map's own per-insertion node
// allocation for key_to_slot_ is the known cost there, accepted because it
// only ever happens up to max_tile_fills_per_tick times per tick, not once
// per substep. What DOES need to be allocation-free, because it is the
// overwhelmingly common case during a steady drive (nothing crosses a tile
// boundary on most ticks), is calling update() when the needed set has not
// changed at all - this test checks exactly that steady state, not tile
// reassignment.
TEST_CASE("TileManager::update allocates nothing when the needed tile set is unchanged", "[terrain][tiles][alloc]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook unavailable under this build (ASan/TSan) - skipping");
        return;
    }
    auto source = std::make_shared<ProceduralTerrainSource>(make_test_spec(), 8);
    ps::jobs::JobSystem jobs(2);
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    TerrainConfig tc;
    tc.samples_per_tile = 8;
    tc.max_resident_tiles = 9;
    tc.max_tile_fills_per_tick = 9;
    TileManager mgr(source, tc, world.backend());

    // Warm-up: stream the tile set in fully (allocation expected here -
    // Jolt's own body/shape bookkeeping, TileManager's own scratch growth,
    // and the key_to_slot_ insertions this comment describes as accepted).
    mgr.set_interest_point(1, Vec3{0, 0, 0}, 32.0); // 3x3 = 9 tiles, fills the whole pool
    for (int i = 0; i < 4; ++i) mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() == 9);

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    ps::jobs::JobSystem::WaitGroup group;
    for (int i = 0; i < 8; ++i) {
        // update() runs INSIDE a job here (matching how World's real
        // terrain_tiles pipeline stage calls it) so debug_alloc::in_job()
        // is true - calling it directly from this test's own thread would
        // never trip the hook regardless of what it allocates (same caveat
        // as test_mesh_heightfield_shapes.cpp's equivalent check).
        jobs.submit([&] { mgr.update(jobs); }, &group);
        jobs.wait(group);
    }
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t violations = ps::jobs::debug_alloc::violation_count();
    ps::jobs::debug_alloc::reset();
    REQUIRE(violations == 0);
    REQUIRE(mgr.resident_tile_count() == 9); // unchanged - confirms nothing was reassigned during the measurement
}
