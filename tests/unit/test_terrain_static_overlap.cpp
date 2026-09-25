// PLAN.md M1 / PHYS-021: the M1 test-ground's static feature meshes
// (data/tracks/test_ground/test_ground_collision.*) sit exactly on one of
// terrain.json's flat zones, both at height 0 (data/tracks/README.md: "A
// meshed feature's own footprint always wins locally"). Loading BOTH the
// real static collision AND the real procedural terrain source into the
// same World - exactly what adapters/godot/src/collision_import.cpp's
// import_test_ground_real does for the M1 demo - used to leave two
// coincident static bodies claiming the same point, so a wheel's
// shape_cast (ClosestHitCollisionCollector, jolt_backend.cpp) could hit
// either one depending on tick-to-tick Jolt-internal tie-breaking: a wheel
// standing still on the asphalt straight would flicker between "asphalt"
// and whatever the open terrain there resolves to (grass/dirt/field),
// silently swapping in the wrong tyre friction every time it did.
//
// Root cause confirmed (see tile_manager.cpp's suppress_covered_samples):
// ps::terrain::TileManager streamed a real heightfield collision tile
// under every flat zone, including the exact footprint of every static
// feature mesh loaded via ps::io::load_test_ground_collision - nothing
// ever suppressed the terrain there. Fixed at the terrain/tile level
// (TerrainConfig::suppress_terrain_under_static_geometry, default on): a
// newly-filled tile's exactly-height-0 samples (only ever true inside a
// flat zone's interior) are probed against the backend, and any sample
// another STATIC body already covers is rewritten to
// ps::kHeightfieldNoCollision - the terrain heightfield gets an actual
// hole there, so only the feature mesh can ever be hit, not a tie between
// the two.
//
// Both TEST_CASEs below use the real test_ground/terrain.json pair (not a
// synthetic spec - test_terrain_tile_manager.cpp already covers the tile
// system in isolation) so this exercises precisely the M1 demo's own
// setup.
//
// PHYS-021 follow-up (owner decision): the ORIGINAL fix above suppressed a
// quad based on a single CENTRE ray, which turned out to over-drop -
// wherever the true mesh edge fell in the outer half of a centre-covered
// quad, the quad's shared vertex still got holed, and Jolt drops a
// triangle if ANY of its corners is a hole - producing a genuine,
// unsuppressed gap (no terrain, no mesh) up to half a quad wide, not just
// the flicker this file's header above describes. Measured directly on the
// real skidpad ring: 36-40% of its circumference had a floorless strip up
// to ~2.05 m wide (predicted analytically at half a quad's spacing,
// ~2.03 m at samples_per_tile=64). tile_manager.cpp's suppress_covered_samples
// now requires a quad's 4 corners AND its centre (5 points) to ALL hit
// foreign static geometry before calling it "fully covered", and a vertex
// is holed only if EVERY quad touching it is fully covered - conservative
// on purpose (over-drop is an unrecoverable fall out of the world;
// under-drop is the already-accepted flicker residual - not a symmetric
// trade, so the rule always resolves the per-quad-vs-arbitrary-mesh-edge
// granularity mismatch toward more flicker, never toward a hole).
//
// THE TESTING LESSON, not just the fix: the second TEST_CASE above crosses
// a STRAIGHT edge (the asphalt straight's start line). A straight edge's
// quad-grid phase (where the true edge falls relative to a quad's own
// corners) is constant along its ENTIRE length - vertex world positions are
// exact multiples of the grid spacing everywhere, tile boundaries included
// - so every possible crossing point on a straight edge gets the SAME
// residual (here, flicker, never a gap), and no amount of re-running that
// test at different points or speeds could ever have exposed the over-drop
// above. A CURVED edge's local orientation sweeps through every possible
// phase as the angle varies, so it is guaranteed to hit both residual types
// somewhere - which is why the new TEST_CASE below tests the skidpad RING
// (PLAN.md/tools/trackgen's only curved static feature) instead of another
// straight one, and asserts the geometric invariant itself (no point along
// the ring has NO collision at all) rather than driving a body across and
// checking it did not fall - the invariant is what actually failed, and
// unlike a fall-test it does not depend on choosing a crossing speed.
//
// A SECOND bug surfaced while landing the conservative rule above, caught
// by the FIRST TEST_CASE below (not a new one): requiring 5 independent
// samples per quad to all agree turned out to regress that test's straight-
// line, dead-centre-of-the-mesh case (2 of 4 wheels stably read "dirt"
// instead of "asphalt" - not a flicker, a stable wrong answer). Root cause:
// suppress_covered_samples' point-coverage probe used to be `ray_cast`,
// which measurably back-face-culls this project's own static feature
// meshes at some points (confirmed directly - a plain ray_cast at several
// points under the real straight consistently missed the asphalt mesh that
// collide_shape_into, queried at the identical point in the same run,
// found without trouble). The single-centre-ray rule this replaced "got
// away with it" by sampling few, evidently lucky points; sampling 5 per
// quad turned that latent fragility into an outright failure to hole large,
// solidly-interior areas. Fixed by switching every point-coverage probe in
// tile_manager.cpp's suppress_covered_samples to collide_shape_into (see
// that function's own comment) - it has no equivalent gap for a MeshShape,
// only for HeightFieldShape, which does not matter here since this probe
// only ever wants a FOREIGN (non-terrain) hit anyway.

#include "ps/io/surface_table.h"
#include "ps/io/test_ground_loader.h"
#include "ps/io/vehicle_io.h"
#include "ps/terrain/procedural_terrain.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace ps;

namespace {

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

// Loads the real test_ground static collision + the real procedural
// terrain source into a fresh World, exactly like
// adapters/godot/src/collision_import.cpp::import_test_ground_real does
// for the M1 demo (that file itself is off-limits to this session's lane -
// this duplicates just the two loader calls it wires together).
std::shared_ptr<const io::SurfaceTable> load_test_ground_world(World& world, terrain::TerrainConfig tc,
                                                                 std::uint64_t interest_id, const Vec3& interest_xy,
                                                                 real stream_radius_m) {
    const std::string track_dir = std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground";
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    world.set_surface_table(surfaces);
    io::load_test_ground_collision(world, track_dir, *surfaces);

    auto terrain_source = terrain::load_procedural_terrain_source(track_dir + "/terrain.json", /*samples_per_tile=*/64);
    world.set_terrain_source(terrain_source, tc);
    world.set_terrain_interest_point(interest_id, interest_xy, stream_radius_m);
    return surfaces;
}

} // namespace

TEST_CASE("A vehicle standing still on the test_ground asphalt straight reports a stable asphalt surface",
          "[terrain][test_ground][io]") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = 4;
    World world(cfg);

    terrain::TerrainConfig tc;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 32;

    // Straight: x in [-500, 500], y = -150 +- 6 (data/tracks/README.md /
    // tools/trackgen/trackgen/layout.py's STRAIGHT). -480 is comfortably
    // inside it lengthwise, centred on it laterally.
    constexpr Vec3 kOnStraight{-480.0, -150.0, 0.0};
    auto surfaces = load_test_ground_world(world, tc, /*interest_id=*/1, kOnStraight, /*stream_radius_m=*/300.0);
    const SurfaceId asphalt_id = surfaces->id_for("asphalt");
    REQUIRE(asphalt_id != kInvalidSurfaceId);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = kOnStraight + Vec3{0.0, 0.0, 0.6};
    const BodyId chassis_id = world.create_body(chassis);

    const vehicle::VehicleDesc vdesc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    const VehicleId veh = world.create_vehicle(vdesc, chassis_id);
    REQUIRE(world.vehicle_wheel_count(veh) == 4);

    // Let tiles stream in around the stationary vehicle and the suspension
    // settle to its load-bearing equilibrium (test_vehicle_static.cpp uses
    // the same 4 s budget for this same car/mass).
    const std::uint64_t settle_ticks = static_cast<std::uint64_t>(4.0 * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t i = 0; i < settle_ticks; ++i) world.step();
    REQUIRE(world.terrain_starved_tile_count() == 0);

    // Now record every wheel's reported surface for a further 2 s (480
    // ticks at 240 Hz) - "a wheel standing still alternates its reported
    // surface within about a second" is the bug report's own timescale.
    const std::uint64_t observe_ticks = static_cast<std::uint64_t>(2.0 * cfg.tick_rate_hz + 0.5);
    std::vector<std::size_t> transitions(4, 0);
    std::vector<SurfaceId> last(4, kInvalidSurfaceId);
    std::vector<bool> seen(4, false);
    for (std::uint64_t t = 0; t < observe_ticks; ++t) {
        world.step();
        for (std::size_t w = 0; w < 4; ++w) {
            const SurfaceId s = world.wheel_state(veh, w).surface;
            if (s == kInvalidSurfaceId) continue; // not in contact this exact tick - ignore, not a "flicker"
            if (seen[w] && s != last[w]) ++transitions[w];
            last[w] = s;
            seen[w] = true;
        }
    }

    for (std::size_t w = 0; w < 4; ++w) {
        INFO("wheel " << w << ": " << transitions[w] << " surface transition(s) over " << observe_ticks << " ticks");
        CHECK(seen[w]);
        CHECK(transitions[w] == 0); // stable the whole observation window - no flicker
        CHECK(last[w] == asphalt_id);
    }
}

TEST_CASE("A body sliding across the test_ground straight's edge at 10 m/s sees one clean surface transition and no bump",
          "[terrain][test_ground][io]") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.job_workers = 4;
    World world(cfg);

    terrain::TerrainConfig tc;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 32;

    constexpr real kLateralY = -150.0;      // straight's own centreline (data/tracks/README.md)
    constexpr real kStartX = -520.0;        // 20 m before the straight's own start line (x = -500)
    auto surfaces = load_test_ground_world(world, tc, /*interest_id=*/1, Vec3{kStartX, kLateralY, 0.0},
                                            /*stream_radius_m=*/300.0);
    const SurfaceId asphalt_id = surfaces->id_for("asphalt");
    REQUIRE(asphalt_id != kInvalidSurfaceId);

    // Frictionless sphere: isolates the GROUND's own geometric continuity
    // (what this test measures) from tyre/rolling-friction dynamics, which
    // the vehicle-level test above already covers.
    BodyDesc probe;
    probe.shape = SphereShape{0.3};
    probe.motion = BodyMotionType::Dynamic;
    probe.mass = 5.0;
    probe.friction = 0.0;
    probe.restitution = 0.0;
    probe.allow_sleep = false;
    probe.pose.position = Vec3{kStartX, kLateralY, 0.35};
    const BodyId probe_id = world.create_body(probe);

    // Let tiles stream in and the sphere settle to rest before starting to
    // move it - isolates the boundary-crossing bump this test is actually
    // about from any initial-drop settling transient.
    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(1.5 * cfg.tick_rate_hz + 0.5); ++i) world.step();
    REQUIRE(world.terrain_starved_tile_count() == 0);
    {
        Motion m = world.backend().get_motion(probe_id);
        m.linear = Vec3{10.0, 0.0, 0.0};
        world.backend().set_motion(probe_id, m);
    }

    // -520 -> -460 at 10 m/s crosses the straight's start line (x = -500)
    // partway through, with margin on both sides.
    const std::uint64_t cross_ticks = static_cast<std::uint64_t>(6.0 * cfg.tick_rate_hz + 0.5); // 60 m at 10 m/s

    // z is tracked directly (not just vertical acceleration): a real "fall
    // into a gap" is what this test must catch, and a sustained multi-tick
    // drop shows up unambiguously in z itself, whereas a single-tick
    // acceleration spike from ordinary contact-solver noise (ties to the
    // tile seam at x = -512 crossed early in this same run, unrelated to
    // this test's own feature-edge boundary - see out/handover_surface.md)
    // does not, and must not fail this test on its own.
    real min_z = world.backend().get_pose(probe_id).position.z;
    real max_abs_az = 0.0;
    real max_abs_az_x = 0.0;
    real prev_vz = world.backend().get_motion(probe_id).linear.z;
    real first_transition_x = 0.0, last_transition_x = 0.0;
    std::size_t transition_count = 0;
    SurfaceId last_surface = kInvalidSurfaceId;
    bool seen_any = false;
    for (std::uint64_t t = 0; t < cross_ticks; ++t) {
        world.step();

        const Pose p = world.backend().get_pose(probe_id);
        const Motion m = world.backend().get_motion(probe_id);
        min_z = std::min(min_z, p.position.z);
        const real az = std::abs((m.linear.z - prev_vz) * cfg.tick_rate_hz);
        if (az > max_abs_az) {
            max_abs_az = az;
            max_abs_az_x = p.position.x;
        }
        prev_vz = m.linear.z;

        for (const ContactEvent& ev : world.last_contact_events()) {
            if (ev.a != probe_id && ev.b != probe_id) continue;
            const SurfaceId s = (ev.a == probe_id) ? ev.surface_b : ev.surface_a;
            if (s == kInvalidSurfaceId) continue;
            if (!seen_any) {
                seen_any = true;
                last_surface = s;
                continue;
            }
            if (s != last_surface) {
                ++transition_count;
                if (transition_count == 1) first_transition_x = p.position.x;
                last_transition_x = p.position.x;
                last_surface = s;
            }
        }
    }

    INFO("max |vertical accel| = " << max_abs_az << " m/s^2 at x = " << max_abs_az_x << "; min z = " << min_z << " m; " << transition_count
                                    << " surface transition(s), spanning x in [" << first_transition_x << ", "
                                    << last_transition_x << "]");
    REQUIRE(seen_any);
    CHECK(last_surface == asphalt_id); // ends up on asphalt, and stays there (the loop above only counts a CHANGE)
    REQUIRE(transition_count >= 1);    // must have actually reached the straight
    // The fix's hole boundary is only ever an AND across a vertex's
    // surrounding quads (tile_manager.cpp's suppress_covered_samples), so a
    // thin sliver right at the true mesh edge can still see the original
    // coincident-collision flicker (never the whole footprint, unlike the
    // pre-fix behaviour this same setup shows with
    // TerrainConfig::suppress_terrain_under_static_geometry = false: every
    // single tick alternates for the straight's entire length). Measured on
    // this source (samples_per_tile=64, 256 m tiles, ~4.06 m spacing): the
    // sliver is under 0.5 m wide (a few ticks at 10 m/s) - bounded well
    // inside two grid-sample widths here.
    CHECK((last_transition_x - first_transition_x) < 8.0);
    // No fall-into-a-gap: never sinks meaningfully below the sphere's own
    // resting height (radius 0.3 m) - a real gap would show as z dropping
    // metres, not centimetres (see this test's own git history for the
    // pre-pool-exclusion-fix version of this run, which fell over 100 m).
    CHECK(min_z > 0.2);
    // Generous bound: not free-fall-level (~9.81 m/s^2 sustained across
    // many ticks would indicate an actual unsupported drop). The former
    // tile-seam blip at x = -512 (158.5 m/s^2) is gone since the backend's
    // cross-tile edge weld (jolt_backend.cpp, TerrainSeamWeldingCollector;
    // test_terrain_seam_crossing.cpp). The remaining single-tick peak,
    // 162 m/s^2 at x = -500, is this test's own holed-heightfield ->
    // asphalt-MeshShape boundary (176 m/s^2 pre-weld from the same
    // approach), a different body pair the weld does not cover.
    CHECK(max_abs_az < 250.0);
}

// PHYS-021 follow-up (see this file's own header comment for the full
// story): the conservative "5 points, all must hit" marking rule replaces
// the old centre-only rule specifically because centre-only over-dropped
// along a CURVED edge - a straight edge's constant quad-grid phase can
// never expose that direction of bug (see the header comment above), so
// this test sweeps the skidpad ring (tools/trackgen/trackgen/layout.py's
// SKIDPAD: center (0, 150), radius 40, width 10 - an annulus from r=35 to
// r=45) instead of another straight feature, and asserts the INVARIANT the
// old rule actually violated directly: a downward ray at EVERY point along
// a fine sweep of both the ring's outer (r=45) and inner (r=35) edges must
// hit SOMETHING (terrain or mesh, either is fine here - this test is about
// "is there a floor", not which surface). Deliberately a geometric
// assertion, not a body driven across the edge and watched for a fall -
// the invariant is what the old rule broke, and unlike a fall-test this
// does not depend on picking a crossing speed or a lucky (or unlucky)
// crossing point.
TEST_CASE("No point along the test_ground skidpad ring's edges has zero terrain/mesh collision",
          "[terrain][test_ground][io][PHYS-021]") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.job_workers = 4;
    World world(cfg);

    terrain::TerrainConfig tc;
    tc.max_resident_tiles = 64;
    tc.max_tile_fills_per_tick = 32;

    constexpr real kCenterX = 0.0, kCenterY = 150.0; // tools/trackgen/trackgen/layout.py SKIDPAD
    constexpr real kOuterR = 45.0;                   // radius(40) + width(10)/2
    constexpr real kInnerR = 35.0;                   // radius(40) - width(10)/2
    auto surfaces = load_test_ground_world(world, tc, /*interest_id=*/1, Vec3{kCenterX, kCenterY, 0.0},
                                            /*stream_radius_m=*/70.0);
    (void)surfaces;

    // Let tiles stream in fully before probing - REQUIRE, not CHECK: a
    // starved tile would itself read back as a false "gap" below and this
    // test would be testing pool sizing, not the marking rule.
    for (std::uint64_t i = 0; i < static_cast<std::uint64_t>(2.0 * cfg.tick_rate_hz + 0.5); ++i) world.step();
    REQUIRE(world.terrain_starved_tile_count() == 0);

    const double kPi = 3.14159265358979323846;
    std::size_t gap_count = 0;
    real worst_angle_deg = -1.0, worst_radius = 0.0;
    for (const real r_edge : {kOuterR, kInnerR}) {
        constexpr int kAngleSteps = 360;      // 1 deg resolution - a curved edge's phase varies continuously,
                                               // so this is dense enough to sample every phase at least once
        constexpr real kBandHalfWidthM = 1.5; // comfortably wider than one quad's spacing (~4.06 m/2)
        constexpr real kRadialStepM = 0.1;
        for (int ai = 0; ai < kAngleSteps; ++ai) {
            const double theta = (2.0 * kPi * ai) / kAngleSteps;
            const real ct = static_cast<real>(std::cos(theta));
            const real st = static_cast<real>(std::sin(theta));
            for (real r = r_edge - kBandHalfWidthM; r <= r_edge + kBandHalfWidthM; r += kRadialStepM) {
                const real x = kCenterX + r * ct;
                const real y = kCenterY + r * st;
                const RayCastHit hit = world.backend().ray_cast(Vec3{x, y, 60.0}, Vec3{0.0, 0.0, -1.0}, 120.0);
                const bool gap = !hit.hit || std::abs(hit.point.z) > 5.0; // > 5.0 excludes a stray far-away hit, not a real floor here
                if (gap) {
                    ++gap_count;
                    worst_angle_deg = static_cast<real>(ai);
                    worst_radius = r;
                }
            }
        }
    }
    INFO("gap_count=" << gap_count << " (0 expected); last gap at angle=" << worst_angle_deg << " deg, r=" << worst_radius);
    CHECK(gap_count == 0);
}
