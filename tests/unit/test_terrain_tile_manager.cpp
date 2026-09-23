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

// PLAN.md M1 / PHYS-021 second bug (owner-reported "fell through the
// floor"): every pool slot's Jolt body is created UP FRONT, and an
// unassigned slot used to sit at its BodyDesc::pose default - world
// (0,0,0) - until first assigned a real tile. tile_manager.cpp's own
// header comment records a direct A/B repro: with that default, a probe
// as little as 2-6 m from world origin found a REAL (if fake, from the
// placeholder's own non-degenerate sentinel corner) surface between z=-3.5
// and z=0, immediately adjacent to a near-vertical drop - a hidden hazard
// at exactly the point PLAN.md's own README calls "the main spawn point".
// Fixed by parking an unassigned slot far below any reachable content
// (kUnassignedSlotParkingZ) instead. This test keeps the interest point
// away from world origin (most of a deliberately oversized pool - 64 slots
// for the handful of tiles actually needed - stays unassigned, exactly the
// failure condition) and checks that world origin's whole neighbourhood is
// now UNIFORMLY "no collision" (honest streaming-lag absence, same as any
// other never-streamed area of the world) rather than the old mix of a
// fake floor next to a cliff.
TEST_CASE("Unassigned pool slots do not leave a stray collision hazard near world origin", "[terrain][tiles][PHYS-021]") {
    auto source = std::make_shared<ProceduralTerrainSource>(make_test_spec(), 8);
    ps::jobs::JobSystem jobs(2);
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    TerrainConfig tc;
    tc.max_resident_tiles = 64; // deliberately oversized - most slots stay unassigned
    tc.max_tile_fills_per_tick = 25;
    TileManager mgr(source, tc, world.backend());

    // Interest point far from world origin (tile_size_m=32 here) - origin's
    // own tile, and every never-visited slot, stays unassigned.
    mgr.set_interest_point(1, Vec3{1000.0, 1000.0, 0.0}, 32.0);
    for (int i = 0; i < 4; ++i) mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() > 0);
    REQUIRE(mgr.resident_tile_count() < 64); // most of the pool is still unassigned - the condition this test needs

    int hits = 0;
    for (real offset = 0.5; offset <= 15.0; offset += 0.5) {
        const RayCastHit hit = world.backend().ray_cast(Vec3{offset, offset, 50.0}, Vec3{0.0, 0.0, -1.0}, 100.0);
        if (hit.hit) {
            INFO("unexpected hit at offset " << offset << ": point.z=" << hit.point.z);
            ++hits;
        }
    }
    REQUIRE(hits == 0);
}

// Cache correctness (tile_manager.cpp's suppress_covered_samples, point a):
// suppression_cache_ must reproduce EXACTLY the same hole set on a cache
// hit as the fresh probe that first computed it - the world's static
// geometry never changes at runtime, so this is a pure-function guarantee,
// checked here via an OBSERVABLE effect (which body a query resolves
// against) rather than reaching into TileManager's own private state.
TEST_CASE("Suppression cache reproduces the same result on a forced re-fetch of the same tile", "[terrain][tiles][PHYS-021]") {
    TerrainSpec spec = make_test_spec();
    // A small flat zone at world origin, well within one 32 m tile, so
    // suppress_covered_samples' "exactly height 0" gate actually fires here
    // (make_test_spec() alone has no flat zone at all).
    FlatZone zone;
    zone.kind = FlatZone::Kind::Disc;
    zone.center = Vec3{0.0, 0.0, 0.0};
    zone.radius = 12.0;
    zone.blend = 2.0;
    spec.flat_zones.push_back(zone);
    auto source = std::make_shared<ProceduralTerrainSource>(spec, 8);
    ps::jobs::JobSystem jobs(2);
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    // A static box sitting on the flat zone's centre, well inside the disc,
    // so the tile covering world origin has real "foreign static" geometry
    // to suppress terrain in favour of.
    BodyDesc box;
    box.shape = BoxShape{Vec3{4.0, 4.0, 0.1}};
    box.motion = BodyMotionType::Static;
    box.pose.position = Vec3{0.0, 0.0, 0.0};
    const BodyId box_id = world.create_body(box);

    TerrainConfig tc;
    tc.max_resident_tiles = 1; // forces genuine eviction+re-fetch of the SAME key below, not just "already resident"
    tc.max_tile_fills_per_tick = 1;
    tc.suppress_terrain_under_static_geometry = true;
    TileManager mgr(source, tc, world.backend());

    const Vec3 kOnBox{0.0, 0.0, 0.0};   // under the box - should resolve to the box, not holed-out terrain
    const Vec3 kElsewhere{500.0, 500.0, 0.0};

    mgr.set_interest_point(1, kOnBox, 8.0);
    for (int i = 0; i < 3; ++i) mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() == 1);
    const RayCastHit first = world.backend().ray_cast(Vec3{0.0, 0.0, 50.0}, Vec3{0.0, 0.0, -1.0}, 100.0);
    REQUIRE(first.hit);
    REQUIRE(first.body == box_id); // terrain is suppressed here - the box is the only thing left to hit

    // Evict (move far away) then come straight back to the SAME TileKey -
    // update()'s cache lookup should short-circuit the whole probe.
    mgr.set_interest_point(1, kElsewhere, 8.0);
    for (int i = 0; i < 3; ++i) mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() == 1);
    mgr.set_interest_point(1, kOnBox, 8.0);
    for (int i = 0; i < 3; ++i) mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() == 1);

    const RayCastHit second = world.backend().ray_cast(Vec3{0.0, 0.0, 50.0}, Vec3{0.0, 0.0, -1.0}, 100.0);
    REQUIRE(second.hit);
    REQUIRE(second.body == box_id); // identical outcome via the cache path
}
