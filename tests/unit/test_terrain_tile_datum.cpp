// tests/unit/test_terrain_tile_datum.cpp — PLAN.md M1 follow-up (R1: a
// downstream game feeds real-world terrain, ~90-880 m ASL, through
// ps::terrain::ITerrainSource). Covers ps::terrain::TileManager's per-tile
// vertical datum (see tile_manager.h's own header comment and
// terrain_source.h's TileSample doc comment): a tile's absolute heights are
// shifted by a per-tile datum before upload (so Jolt's fixed +-200 m
// quantisation range only ever has to cover one tile's local relief) and
// the datum is added back via the tile body's own pose z, so collision
// still resolves at the correct ABSOLUTE height. White-box (core/src on the
// include path, same as test_terrain_tile_manager.cpp) since TileManager is
// internal.
//
// Uses a small hand-written ITerrainSource (AnalyticTerrainSource below)
// instead of ProceduralTerrainSource so every case's true height is exact
// and known analytically - the point here is the datum arithmetic and
// Jolt's own collision quantisation, not terrain generation.

#include "terrain/procedural_terrain_source.h"
#include "terrain/tile_manager.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>

using namespace ps;
using namespace ps::terrain;

namespace {

// A minimal, exact ITerrainSource: height_fn(x, y) is evaluated directly at
// every sample point, with no caching/interpolation of its own. TileKey
// packs (tile_x, tile_y) as two int32 halves of an int64 - opaque to every
// caller either way (PLAN.md section 6), same shape as
// ProceduralTerrainSource's own packing but independent of it.
class AnalyticTerrainSource final : public ITerrainSource {
public:
    AnalyticTerrainSource(real tile_size_m, std::uint32_t samples_per_tile, std::function<real(real, real)> height_fn)
        : tile_size_m_(tile_size_m), samples_per_tile_(samples_per_tile), height_fn_(std::move(height_fn)) {}

    [[nodiscard]] real tile_size_m() const override { return tile_size_m_; }
    [[nodiscard]] std::uint32_t samples_per_tile() const override { return samples_per_tile_; }

    [[nodiscard]] TileKey key_for_world_xy(real x, real y) const override {
        const auto tx = static_cast<std::int32_t>(std::floor(x / tile_size_m_));
        const auto ty = static_cast<std::int32_t>(std::floor(y / tile_size_m_));
        return pack(tx, ty);
    }
    [[nodiscard]] TileKey key_offset(TileKey key, std::int32_t dx, std::int32_t dy) const override {
        const auto [tx, ty] = unpack(key);
        return pack(tx + dx, ty + dy);
    }
    [[nodiscard]] Vec3 tile_origin_xy(TileKey key) const override {
        const auto [tx, ty] = unpack(key);
        return Vec3{static_cast<real>(tx) * tile_size_m_, static_cast<real>(ty) * tile_size_m_, 0.0};
    }
    void fill_tile(TileKey key, TileSample& out) const override {
        const Vec3 origin = tile_origin_xy(key);
        const std::uint32_t n = samples_per_tile_;
        const real spacing = tile_size_m_ / static_cast<real>(n - 1);
        for (std::uint32_t iy = 0; iy < n; ++iy) {
            for (std::uint32_t ix = 0; ix < n; ++ix) {
                const real wx = origin.x + static_cast<real>(ix) * spacing;
                const real wy = origin.y + static_cast<real>(iy) * spacing;
                out.heights[static_cast<std::size_t>(iy) * n + ix] = height_fn_(wx, wy);
            }
        }
        std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), kInvalidSurfaceId);
        out.samples_per_tile = n;
        out.tile_size_m = tile_size_m_;
        out.tile_origin_xy = origin;
    }

    [[nodiscard]] real height_at(real x, real y) const { return height_fn_(x, y); }
    [[nodiscard]] real spacing() const { return tile_size_m_ / static_cast<real>(samples_per_tile_ - 1); }

private:
    static TileKey pack(std::int32_t tx, std::int32_t ty) {
        return TileKey{(static_cast<std::int64_t>(static_cast<std::uint32_t>(tx)) << 32) | static_cast<std::int64_t>(static_cast<std::uint32_t>(ty))};
    }
    static std::pair<std::int32_t, std::int32_t> unpack(TileKey key) {
        return {static_cast<std::int32_t>(static_cast<std::uint32_t>(static_cast<std::uint64_t>(key.packed) >> 32)),
                static_cast<std::int32_t>(static_cast<std::uint32_t>(static_cast<std::uint64_t>(key.packed) & 0xffffffffu))};
    }

    real tile_size_m_;
    std::uint32_t samples_per_tile_;
    std::function<real(real, real)> height_fn_;
};

// Mirrors tile_manager.cpp's own kTileDatumStepM - not exposed by
// TileManager's header, so this test predicts the snap independently to
// check the production formula, not just re-read its own answer.
constexpr real kExpectedDatumStepM = 128.0;
real expected_datum_for(const AnalyticTerrainSource& source, TileKey key) {
    TileSample s;
    const std::uint32_t n = source.samples_per_tile();
    s.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
    s.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), kInvalidSurfaceId);
    s.samples_per_tile = n;
    source.fill_tile(key, s);
    const auto [min_it, max_it] = std::minmax_element(s.heights.begin(), s.heights.end());
    const real mid = (*min_it + *max_it) * real(0.5);
    return std::round(mid / kExpectedDatumStepM) * kExpectedDatumStepM;
}

// Straight-down probe through a terrain-only backend; fails the calling
// REQUIRE if nothing was hit.
real probe_height(World& world, real x, real y) {
    constexpr real kAbove = 2000.0;
    const RayCastHit hit = world.backend().ray_cast(Vec3{x, y, kAbove}, Vec3{0, 0, -1}, kAbove + 2000.0);
    REQUIRE(hit.hit);
    return hit.point.z;
}

} // namespace

TEST_CASE("Per-tile datum places real-world-elevation terrain at the correct absolute height", "[terrain][tiles][datum]") {
    constexpr real kTileSize = 64.0;
    constexpr std::uint32_t kSamples = 16;

    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    ps::jobs::JobSystem jobs(2);

    SECTION("+800 m ASL, gentle relief") {
        auto source = std::make_shared<AnalyticTerrainSource>(
            kTileSize, kSamples, [](real x, real y) { return real(800.0) + real(5.0) * std::sin(x * 0.1) * std::cos(y * 0.1); });
        TerrainConfig tc;
        tc.max_resident_tiles = 4;
        tc.max_tile_fills_per_tick = 4;
        TileManager mgr(source, tc, world.backend());
        mgr.set_interest_point(1, Vec3{kTileSize * 0.5, kTileSize * 0.5, 0.0}, 0.0); // radius 0 -> exactly 1 tile
        for (int i = 0; i < 3; ++i) mgr.update(jobs);
        REQUIRE(mgr.resident_tile_count() == 1);

        const TileKey key = source->key_for_world_xy(kTileSize * 0.5, kTileSize * 0.5);
        const real expected = expected_datum_for(*source, key);
        const real actual = mgr.resident_tile_datum(key);
        REQUIRE(actual == expected);
        REQUIRE(actual == 768.0); // mid ~800 snaps to the nearest multiple of 128

        // Probe exactly AT this tile's own sample-grid points (spacing
        // kTileSize/(kSamples-1), ~4.27 m here) rather than arbitrary world
        // coordinates: this section's height function is curved (sin*cos),
        // and at only 16 samples across 64 m an off-grid probe measures
        // Jolt's own bilinear interpolation of an under-sampled curve, not
        // the datum/quantisation accuracy this test is actually about (the
        // "Collision height resolution" test case below covers grid/
        // interpolation fidelity on its own, at the downstream game's real
        // 1 m spacing).
        const real spacing = kTileSize / static_cast<real>(kSamples - 1);
        real max_err = 0.0;
        for (std::uint32_t ix = 2; ix < kSamples - 2; ix += 3) {
            for (std::uint32_t iy = 2; iy < kSamples - 2; iy += 3) {
                const real x = static_cast<real>(ix) * spacing;
                const real y = static_cast<real>(iy) * spacing;
                const real measured = probe_height(world, x, y);
                const real truth = source->height_at(x, y);
                max_err = std::max(max_err, std::abs(measured - truth));
            }
        }
        WARN("+800 m ASL tile: max |probe - truth| at sample points = " << max_err << " m (datum=" << actual << ")");
        REQUIRE(max_err < 0.01); // within 1 cm, as required
    }

    SECTION("-350 m ASL, gentle relief") {
        auto source = std::make_shared<AnalyticTerrainSource>(
            kTileSize, kSamples, [](real x, real y) { return real(-350.0) + real(5.0) * std::sin(x * 0.1) * std::cos(y * 0.1); });
        TerrainConfig tc;
        tc.max_resident_tiles = 4;
        tc.max_tile_fills_per_tick = 4;
        TileManager mgr(source, tc, world.backend());
        mgr.set_interest_point(1, Vec3{kTileSize * 0.5, kTileSize * 0.5, 0.0}, 0.0);
        for (int i = 0; i < 3; ++i) mgr.update(jobs);
        REQUIRE(mgr.resident_tile_count() == 1);

        const TileKey key = source->key_for_world_xy(kTileSize * 0.5, kTileSize * 0.5);
        const real expected = expected_datum_for(*source, key);
        const real actual = mgr.resident_tile_datum(key);
        REQUIRE(actual == expected);
        REQUIRE(actual == -384.0); // mid ~-350 snaps to the nearest multiple of 128

        // See the +800 m section's comment above for why this probes exact
        // sample-grid points rather than arbitrary world coordinates.
        const real spacing = kTileSize / static_cast<real>(kSamples - 1);
        real max_err = 0.0;
        for (std::uint32_t ix = 2; ix < kSamples - 2; ix += 3) {
            for (std::uint32_t iy = 2; iy < kSamples - 2; iy += 3) {
                const real x = static_cast<real>(ix) * spacing;
                const real y = static_cast<real>(iy) * spacing;
                const real measured = probe_height(world, x, y);
                const real truth = source->height_at(x, y);
                max_err = std::max(max_err, std::abs(measured - truth));
            }
        }
        WARN("-350 m ASL tile: max |probe - truth| at sample points = " << max_err << " m (datum=" << actual << ")");
        REQUIRE(max_err < 0.01);
    }
}

TEST_CASE("A dropped body lands at the correct absolute height on a datum-shifted tile", "[terrain][tiles][datum]") {
    constexpr real kTileSize = 64.0;
    constexpr std::uint32_t kSamples = 16;
    auto source =
        std::make_shared<AnalyticTerrainSource>(kTileSize, kSamples, [](real, real) { return real(800.0); }); // flat, exact drop target
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    TerrainConfig tc;
    tc.max_resident_tiles = 4;
    tc.max_tile_fills_per_tick = 4;
    world.set_terrain_source(source, tc);
    world.set_terrain_interest_point(1, Vec3{kTileSize * 0.5, kTileSize * 0.5, 0.0}, 0.0);
    for (int i = 0; i < 3; ++i) world.step();
    REQUIRE(world.terrain_resident_tile_count() == 1);

    BodyDesc ball;
    ball.shape = SphereShape{0.3};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{kTileSize * 0.5, kTileSize * 0.5, 810.0}; // 10 m above the 800 m ground
    ball.allow_sleep = false;
    const BodyId id = world.create_body(ball);
    // Free fall from 10 m takes ~1.43 s; 5 s gives ample time to land and settle.
    for (int i = 0; i < static_cast<int>(5.0 * 240.0); ++i) world.step();
    const real z = world.get_pose(id).position.z;
    WARN("dropped sphere rest z=" << z << " (expected ~800.3)");
    REQUIRE(std::abs(z - 800.3) < 0.02); // radius 0.3 m above a flat 800 m surface
}

// Deliberately steep (2 m rise per 1 m run - unrealistic, chosen only to
// force adjacent tiles to snap to DIFFERENT datums at this small test tile
// size) so tile (0,0) and tile (1,0) get different 128 m datums, and the
// shared border between them (world x=64) is reconstructed from two
// INDEPENDENTLY quantised tiles. Checks the border stays continuous within
// Jolt's own quantisation, not exact equality (see this file's header
// comment).
TEST_CASE("Adjacent tiles with different datums stay seam-continuous", "[terrain][tiles][datum][seams]") {
    constexpr real kTileSize = 64.0;
    constexpr std::uint32_t kSamples = 16;
    constexpr real kBase = 700.0;
    constexpr real kSlope = 2.0; // m per m - see the comment above
    auto source = std::make_shared<AnalyticTerrainSource>(kTileSize, kSamples, [](real x, real) { return kBase + kSlope * x; });

    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    ps::jobs::JobSystem jobs(2);
    TerrainConfig tc;
    tc.max_resident_tiles = 16;
    tc.max_tile_fills_per_tick = 16;
    TileManager mgr(source, tc, world.backend());
    mgr.set_interest_point(1, Vec3{kTileSize, 0.0, 0.0}, kTileSize * 0.5); // covers tiles (0,0) and (1,0) at least
    for (int i = 0; i < 4; ++i) mgr.update(jobs);

    const TileKey key0 = source->key_for_world_xy(kTileSize * 0.5, 0.0);  // tile (0,0)
    const TileKey key1 = source->key_for_world_xy(kTileSize * 1.5, 0.0); // tile (1,0)
    const real datum0 = mgr.resident_tile_datum(key0);
    const real datum1 = mgr.resident_tile_datum(key1);
    REQUIRE(datum0 == expected_datum_for(*source, key0));
    REQUIRE(datum1 == expected_datum_for(*source, key1));
    INFO("datum0=" << datum0 << " datum1=" << datum1);
    REQUIRE(datum0 != datum1); // the whole point of this test

    const real border_x = kTileSize; // world x=64, shared by both tiles exactly
    const real eps = 0.05;           // stay inside each tile's own edge quad, off the shared vertex itself
    const real left = probe_height(world, border_x - eps, 0.0);
    const real right = probe_height(world, border_x + eps, 0.0);
    const real truth_left = source->height_at(border_x - eps, 0.0);
    const real truth_right = source->height_at(border_x + eps, 0.0);
    const real err_left = std::abs(left - truth_left);
    const real err_right = std::abs(right - truth_right);
    WARN("seam probe: left err=" << err_left << " m, right err=" << err_right << " m, true gap=" << (truth_right - truth_left)
                                  << " m, measured gap=" << (right - left) << " m");
    // Bound for this deliberately steep artificial slope (see the comment
    // above) - the point is continuity, not sub-mm accuracy on a 200%-grade
    // test fixture. Measured left/right error is ~1.1/3.2 cm (this probe's
    // own coarse-quad interpolation error at kTileSize/(kSamples-1) ~4.27 m
    // spacing on a 200%-grade slope, not a datum artefact - the "Collision
    // height resolution" test case measures the same quantity on a much
    // gentler, realistic 30% slope at the downstream game's actual 1 m
    // spacing and finds ~1 mm); this bound is a wide margin above that, not
    // a tight pin.
    REQUIRE(err_left < 0.1);
    REQUIRE(err_right < 0.1);
}

TEST_CASE("A tile whose own relief overflows the datum window is counted, not silently dropped", "[terrain][tiles][datum]") {
    constexpr real kTileSize = 64.0;
    constexpr std::uint32_t kSamples = 16;
    // 0 to 500 m across one tile - relief far beyond +-200 m from any datum.
    auto source = std::make_shared<AnalyticTerrainSource>(kTileSize, kSamples, [](real x, real) { return real(500.0) * (x / kTileSize); });

    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    ps::jobs::JobSystem jobs(2);
    TerrainConfig tc;
    tc.max_resident_tiles = 4;
    tc.max_tile_fills_per_tick = 4;
    TileManager mgr(source, tc, world.backend());
    mgr.set_interest_point(1, Vec3{kTileSize * 0.5, kTileSize * 0.5, 0.0}, 0.0);
    for (int i = 0; i < 3; ++i) mgr.update(jobs);
    REQUIRE(mgr.resident_tile_count() == 1);
    REQUIRE(mgr.relief_overflow_count() >= 1);
}

// R1 downstream requirement: their tiles are 255 m with 256 samples (1 m
// spacing), heights quantised to 1/256 m by their own source; they need to
// know whether COLLISION resolves a <5 mm road/blend step at that spacing,
// and whether the datum degrades it. Measured directly (not derived from
// Jolt's own quantisation formulas alone - see this task's report for the
// numbers), at both datum 0 and datum ~768 (base height 800 m), for flat
// ground, a 30% slope, and a 5 mm step, at sample points AND at cell
// centres (off-sample points collision must interpolate to reach).
TEST_CASE("Collision height resolution at 1 m sample spacing (R1)", "[terrain][tiles][datum][resolution]") {
    constexpr real kTileSize = 255.0;
    constexpr std::uint32_t kSamples = 256; // 1 m spacing, matching the downstream source

    auto measure = [&](const char* label, real base, const std::function<real(real, real)>& height_fn) {
        auto source = std::make_shared<AnalyticTerrainSource>(kTileSize, kSamples, height_fn);
        WorldConfig cfg;
        cfg.job_workers = 1;
        World world(cfg);
        ps::jobs::JobSystem jobs(2);
        TerrainConfig tc;
        tc.max_resident_tiles = 2;
        tc.max_tile_fills_per_tick = 2;
        TileManager mgr(source, tc, world.backend());
        mgr.set_interest_point(1, Vec3{kTileSize * 0.5, kTileSize * 0.5, 0.0}, 0.0);
        for (int i = 0; i < 3; ++i) mgr.update(jobs);
        REQUIRE(mgr.resident_tile_count() == 1);
        const TileKey key = source->key_for_world_xy(kTileSize * 0.5, kTileSize * 0.5);
        const real datum = mgr.resident_tile_datum(key);

        // Sample-grid points (integer x/y, away from the tile's own edges).
        real max_err_samples = 0.0;
        for (real x = 20.5; x < kTileSize - 20.0; x += 37.0) {
            for (real y = 20.5; y < kTileSize - 20.0; y += 37.0) {
                const real sx = std::round(x); // land exactly on an integer sample
                const real sy = std::round(y);
                const real measured = probe_height(world, sx, sy);
                const real truth = height_fn(sx, sy);
                max_err_samples = std::max(max_err_samples, std::abs(measured - truth));
            }
        }
        // Cell-centre points (half-integer x/y - Jolt's own bilinear/
        // triangle interpolation between the 4 surrounding samples).
        real max_err_centres = 0.0;
        for (real x = 20.5; x < kTileSize - 20.0; x += 37.0) {
            for (real y = 20.5; y < kTileSize - 20.0; y += 37.0) {
                const real cx = std::round(x) + 0.5;
                const real cy = std::round(y) + 0.5;
                const real measured = probe_height(world, cx, cy);
                const real truth = height_fn(cx, cy);
                max_err_centres = std::max(max_err_centres, std::abs(measured - truth));
            }
        }
        WARN(label << " base=" << base << " datum=" << datum << ": max|err| at samples=" << max_err_samples
                    << " m, at cell centres=" << max_err_centres << " m");
        return std::max(max_err_samples, max_err_centres);
    };

    SECTION("flat ground") {
        REQUIRE(measure("flat, datum 0", 0.0, [](real, real) { return real(0.0); }) < 0.01);
        REQUIRE(measure("flat, datum ~768", 800.0, [](real, real) { return real(800.0); }) < 0.01);
    }
    SECTION("30% slope") {
        REQUIRE(measure("30% slope, datum 0", 0.0, [](real x, real) { return real(0.30) * x; }) < 0.01);
        REQUIRE(measure("30% slope, datum ~768/~896", 800.0, [](real x, real) { return real(800.0) + real(0.30) * x; }) < 0.01);
    }
    SECTION("5 mm step") {
        constexpr real kStepX = 127.5; // mid-tile, on a whole-metre sample boundary
        auto step_fn0 = [](real x, real) { return x < kStepX ? real(0.0) : real(0.005); };
        auto step_fn800 = [](real x, real) { return (x < kStepX ? real(0.0) : real(0.005)) + real(800.0); };
        REQUIRE(measure("5 mm step, datum 0", 0.0, step_fn0) < 0.01);
        REQUIRE(measure("5 mm step, datum ~768", 800.0, step_fn800) < 0.01);
    }
}

// Diagnostic only ([.][diag], same convention as e.g.
// test_vehicle_standstill_wiggle.cpp's "zeta sweep diagnostic" - not run by
// default). Measures, with real numbers rather than amplitude arithmetic
// alone, the actual height range of this project's two existing
// ProceduralTerrainSource specs, to state whether their tiles snap to
// datum 0 (an exact no-op subtraction).
//
// Measured (100 m grid over the full +-50000 m world,
// data/tracks/test_ground/terrain.json): min=-83.16 m, max=78.28 m. This
// EXCEEDS a single +-64 m half-window, so NOT every test_ground tile snaps
// to datum 0 - specifically tiles far from the origin, where the roughness
// term ramps the 55 m-amplitude hill octave in (0 at/inside
// roughness.near_radius_m=8000, full strength at/beyond far_radius_m=30000;
// world half_extent_m=50000 reaches well past that), can get a real nonzero
// datum. This is fine and expected, not a bug: test_ground's own height
// range was already comfortably inside Jolt's original fixed +-200 m window
// even before this feature (the datum feature exists for real-world terrain
// at 90-880 m ASL, far outside that window, not for this synthetic spec),
// and every test_ground-based test (test_terrain_tile_manager.cpp,
// test_terrain_static_overlap.cpp, test_terrain_suppression_precompute.cpp,
// bench_terrain_tiles) still passes unchanged with a real per-tile datum
// now applying on some of its tiles - the datum/pose-z round trip preserves
// absolute height regardless of whether the datum happens to be 0. None of
// these tests hash-compare, so this is a functional (ctest pass), not a
// bit-identity, guarantee.
//
// Measured (2 m grid over the full +-500 m world,
// data/scenarios/terrain_spec_small.json): min=-2.35 m, max=2.41 m - by
// construction (rolling-octave amplitude budget 2.0+0.5=2.5 m; the hill
// octave's roughness term is 0 everywhere in this 500 m-radius world since
// roughness.near_radius_m=1e6 is never reached), comfortably inside +-64 m
// with wide margin. EVERY tile this spec can ever produce therefore snaps
// to datum 0 - an exact no-op subtraction (see the `datum == 0.0` skip in
// TileManager::update) - which is what matters for bit-identity:
// data/scenarios/terrain_tiles.json and terrain_multi_interest.json (the
// two state_hash-checked scenarios, 0x5afffd7d796d4f34 / 0x6275dc6d271a8906
// per this feature's own commit message) are BOTH built on this spec, never
// on test_ground.
TEST_CASE("Diagnostic: real terrain.json / terrain_spec_small.json height range", "[terrain][.][diag]") {
#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined (see tests/unit/CMakeLists.txt)"
#endif
    auto scan_range = [](const std::string& path, real half_extent, real step) {
        const TerrainSpec spec = load_terrain_spec(path);
        real min_h = std::numeric_limits<real>::max();
        real max_h = std::numeric_limits<real>::lowest();
        for (real x = -half_extent; x <= half_extent; x += step) {
            for (real y = -half_extent; y <= half_extent; y += step) {
                const real h = terrain_height(spec, x, y);
                min_h = std::min(min_h, h);
                max_h = std::max(max_h, h);
            }
        }
        return std::make_pair(min_h, max_h);
    };

    // data/tracks/test_ground/terrain.json: world half_extent_m=50000, tile
    // 256 m - a 100 m-spaced grid covers every tile at least once. Only
    // sanity-checked against Jolt's ORIGINAL fixed +-200 m window (true
    // regardless of this feature) - see the comment above for why this one
    // is not expected to stay inside +-64 m everywhere.
    {
        const auto [min_h, max_h] = scan_range(std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground/terrain.json", 50000.0, 100.0);
        WARN("data/tracks/test_ground/terrain.json: min=" << min_h << " m, max=" << max_h << " m, mid=" << (min_h + max_h) * 0.5
                                                            << " m");
        CHECK(min_h > -200.0);
        CHECK(max_h < 200.0);
    }
    // data/scenarios/terrain_spec_small.json: world half_extent_m=500, tile
    // 16 m - a 2 m-spaced grid covers every tile many times over. Backs
    // BOTH state-hash-checked scenarios, so this one IS asserted to stay
    // inside +-64 m everywhere (every tile snaps to datum 0).
    {
        const auto [min_h, max_h] = scan_range(std::string(PS_SOURCE_DIR) + "/data/scenarios/terrain_spec_small.json", 500.0, 2.0);
        WARN("data/scenarios/terrain_spec_small.json: min=" << min_h << " m, max=" << max_h << " m, mid=" << (min_h + max_h) * 0.5
                                                              << " m");
        CHECK(min_h > -64.0);
        CHECK(max_h < 64.0);
    }
}
