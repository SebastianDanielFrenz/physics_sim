// tests/unit/test_wheel_contact_flag.cpp - BodyDesc::wheel_contact (farm sim: a wheel's shape cast must not read the side of a parked
// machine's box, a woken vehicle or a bale as ground). The flag default (true) reproduces today's cast; false makes the body invisible to
// the wheel path only (shape_cast(..., skip_non_wheel_bodies = true), D0 re-cast included); the default shape_cast and every other query
// are unchanged.
// Sabotage WC-S1: pass `false` for the filter in the first CastShape call of jolt_backend.cpp's shape_cast -> WC6 and WC4 fail (WC2 is rescued by the D0 re-cast, which carries the same filter: sabotage that too and WC2 fails).

#include "vehicle_test_helpers.h"

#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ps;
using Catch::Approx;

namespace {
struct Scene {
    std::unique_ptr<World> world;
    BodyId ground, wall;
};

// Ground top at z = 0; a wall x in [0, 1], 6 m wide, top at z = 3. A wheel (radius 0.5, width 0.4, axis Y) at x = -0.3 overlaps the wall's side
// for its whole height: Jolt reports the wall (fraction 0, or the wall's face), not the ground.
Scene make_scene(bool wall_wheel_contact, int workers = 1) {
    WorldConfig cfg;
    cfg.job_workers = workers;
    Scene s;
    s.world = std::make_unique<World>(cfg);
    BodyDesc g;
    g.shape = BoxShape{Vec3{50.0, 50.0, 0.5}};
    g.motion = BodyMotionType::Static;
    g.pose.position = Vec3{0.0, 0.0, -0.5};
    s.ground = s.world->create_body(g);
    BodyDesc w;
    w.shape = BoxShape{Vec3{0.5, 3.0, 1.5}};
    w.motion = BodyMotionType::Static;
    w.pose.position = Vec3{0.5, 0.0, 1.5};
    w.wheel_contact = wall_wheel_contact;
    s.wall = s.world->create_body(w);
    return s;
}

ShapeCastHit cast(Scene& s, bool wheel_path) {
    const CylinderShape wheel{0.2, 0.5};
    Pose p = Pose::identity();
    p.position = Vec3{-0.3, 0.0, 1.2};
    return s.world->backend().shape_cast(wheel, p, Vec3{0.0, 0.0, -1.0}, 1.0, wheel_path);
}
} // namespace

TEST_CASE("wheel_contact: with the flag true the wheel cast hits the wall as before", "[world][backend][wheel_contact]") {
    Scene s = make_scene(true);
    const ShapeCastHit h = cast(s, true);
    REQUIRE(h.hit);
    CHECK(h.body.index == s.wall.index); // WC1: today's behaviour, the wall is read as the thing under the wheel
}

TEST_CASE("wheel_contact: a wheel next to a flagged-out body gets the ground under it", "[world][backend][wheel_contact]") {
    Scene s = make_scene(false);
    const ShapeCastHit h = cast(s, true);
    REQUIRE(h.hit);
    CHECK(h.body.index == s.ground.index);        // WC2
    CHECK(h.fraction == Approx(0.7).margin(0.02)); // centre 1.2 -> wheel bottom reaches z = 0 at centre 0.5
    CHECK(h.normal.z == Approx(1.0).margin(0.01));
}

TEST_CASE("wheel_contact: the non-wheel cast still sees the flagged-out body", "[world][backend][wheel_contact]") {
    Scene s = make_scene(false);
    const ShapeCastHit h = cast(s, false);
    REQUIRE(h.hit);
    CHECK(h.body.index == s.wall.index); // WC3: only the wheel path skips it
}

TEST_CASE("wheel_contact: set_wheel_contact flips a body between steps", "[world][backend][wheel_contact]") {
    Scene s = make_scene(true);
    CHECK(cast(s, true).body.index == s.wall.index);
    s.world->backend().set_wheel_contact(s.wall, false);
    CHECK(cast(s, true).body.index == s.ground.index); // WC4
    s.world->backend().set_wheel_contact(s.wall, true);
    CHECK(cast(s, true).body.index == s.wall.index);
}

TEST_CASE("wheel_contact: a car driving past a flagged box gives the same state hash at 1 and 10 workers", "[world][backend][vehicle][wheel_contact][determinism]") {
    auto run = [](int workers) {
        auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0, Vec3{1.2, 0.35, 0.45}, 0.8, 0.0, workers);
        BodyDesc w;
        w.shape = BoxShape{Vec3{3.0, 0.5, 1.5}};
        w.motion = BodyMotionType::Static;
        w.pose.position = Vec3{0.0, 1.55, 1.5}; // beside the car, its side face inside the wheel casts
        w.wheel_contact = false;
        rig.world->create_body(w);
        for (int i = 0; i < 240 * 2; ++i) rig.world->step();
        return rig.world->state_hash();
    };
    CHECK(run(1) == run(10)); // WC5
}

TEST_CASE("wheel_contact: a low box under the wheel (a plough box) is not read as ground", "[world][backend][wheel_contact]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    BodyDesc g;
    g.shape = BoxShape{Vec3{50.0, 50.0, 0.5}};
    g.motion = BodyMotionType::Static;
    g.pose.position = Vec3{0.0, 0.0, -0.5};
    const BodyId ground = world.create_body(g);
    BodyDesc b;
    b.shape = BoxShape{Vec3{1.0, 1.0, 0.15}}; // top at z = 0.3
    b.motion = BodyMotionType::Static;
    b.pose.position = Vec3{0.0, 0.0, 0.15};
    b.wheel_contact = false;
    const BodyId low = world.create_body(b);
    const CylinderShape wheel{0.2, 0.5};
    Pose p = Pose::identity();
    p.position = Vec3{0.0, 0.0, 1.2};
    const ShapeCastHit w = world.backend().shape_cast(wheel, p, Vec3{0.0, 0.0, -1.0}, 1.0, true);
    REQUIRE(w.hit);
    CHECK(w.body.index == ground.index); // WC6: the wheel reaches z = 0.5 above the ground, not 0.8 above the box top
    CHECK(w.fraction == Approx(0.7).margin(0.02));
    const ShapeCastHit all = world.backend().shape_cast(wheel, p, Vec3{0.0, 0.0, -1.0}, 1.0, false);
    REQUIRE(all.hit);
    CHECK(all.body.index == low.index);
}
