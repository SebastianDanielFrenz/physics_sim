// PLAN.md M1 step 4: data/tracks/test_ground's own static collision data
// (test_ground_collision.json + .bin), loaded via
// ps::io::load_test_ground_collision, must produce real Jolt static mesh
// bodies whose surface ids (ps::io::SurfaceTable, data/surfaces/
// surfaces.json) survive an actual collision - a dynamic body dropped onto
// asphalt and onto the grass patch must report the RIGHT surface id at
// contact (ContactEvent::surface_a/surface_b, PLAN.md section 6).
//
// The two drop points below were found empirically with a throwaway probe
// (ps::IRigidBackend::shape_cast straight down at a grid of candidate
// (x, y) - the same technique test_mesh_heightfield_shapes.cpp's
// orientation test uses) rather than guessed from the JSON's own AABBs:
// test_ground_collision.json's per-surface AABB spans a much larger area
// than that surface's actual triangles cover (it is a real winding track,
// not a filled rectangle), so most "clearly inside the AABB" points are
// actually off the mesh entirely and would make the ball fall forever.
// (-300, -1100) hits asphalt at z ~= 0.4715 m; (-300, -480) hits the grass
// patch at z ~= 0.05 m (both from an actual probe run against this exact
// file, not a formula) - see this file's own header comment history in
// out/handover_terrain.md for the probe's full output.

#include "ps/io/surface_table.h"
#include "ps/io/test_ground_loader.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

using namespace ps;

namespace {

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

// Drops a small dynamic sphere from `drop_z` straight down at (x, y) and
// returns the SurfaceId reported by the first contact event involving it
// (kInvalidSurfaceId if none was ever seen within `max_ticks`).
SurfaceId drop_and_find_contact_surface(World& world, real x, real y, real drop_z, int max_ticks) {
    BodyDesc ball;
    ball.shape = SphereShape{0.3};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{x, y, drop_z};
    ball.allow_sleep = false;
    const BodyId ball_id = world.create_body(ball);

    for (int i = 0; i < max_ticks; ++i) {
        world.step();
        for (const ContactEvent& ev : world.last_contact_events()) {
            if (ev.a != ball_id && ev.b != ball_id) continue;
            const SurfaceId s = (ev.a == ball_id) ? ev.surface_b : ev.surface_a;
            if (s != kInvalidSurfaceId) return s;
        }
    }
    return kInvalidSurfaceId;
}

} // namespace

TEST_CASE("load_test_ground_collision creates one static body per surface group", "[io][test_ground]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    const std::vector<BodyId> ids = io::load_test_ground_collision(world, std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground", surfaces);

    // test_ground_collision.json currently lists 5 surface groups (asphalt,
    // dirt, grass, kerb, low_mu) - not hardcoded elsewhere in the loader
    // itself, but this test's own expectation of that file's shape.
    REQUIRE(ids.size() == 5);
    for (BodyId id : ids) REQUIRE(id.valid());
}

TEST_CASE("A body dropped on the test_ground asphalt reports the asphalt surface id at contact", "[io][test_ground][contact]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    io::load_test_ground_collision(world, std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground", surfaces);

    const SurfaceId expected = surfaces.id_for("asphalt");
    REQUIRE(expected != kInvalidSurfaceId);

    const SurfaceId found = drop_and_find_contact_surface(world, -300.0, -1100.0, /*drop_z=*/5.0, /*max_ticks=*/480);
    REQUIRE(found == expected);
}

TEST_CASE("A body dropped on the test_ground grass patch reports the grass surface id at contact", "[io][test_ground][contact]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    io::load_test_ground_collision(world, std::string(PS_SOURCE_DIR) + "/data/tracks/test_ground", surfaces);

    const SurfaceId expected = surfaces.id_for("grass");
    REQUIRE(expected != kInvalidSurfaceId);

    const SurfaceId found = drop_and_find_contact_surface(world, -300.0, -480.0, /*drop_z=*/2.0, /*max_ticks=*/480);
    REQUIRE(found == expected);
}

TEST_CASE("SurfaceTable assigns stable, distinct ids to every surfaces.json entry", "[io][surface_table]") {
    io::SurfaceTable surfaces(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    // surfaces.json currently defines asphalt, dirt, field, grass, kerb, low_mu.
    REQUIRE(surfaces.size() == 6);
    REQUIRE(surfaces.id_for("asphalt") != kInvalidSurfaceId);
    REQUIRE(surfaces.id_for("grass") != kInvalidSurfaceId);
    REQUIRE(surfaces.id_for("asphalt") != surfaces.id_for("grass"));
    REQUIRE(surfaces.id_for("does_not_exist") == kInvalidSurfaceId);
    REQUIRE(surfaces.name_for(surfaces.id_for("dirt")) == "dirt");
}
