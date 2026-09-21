// Sanity tests for ps::World: construction, body creation, stepping, ids.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

using namespace ps;

TEST_CASE("World constructs with an explicit worker count", "[world]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);
    REQUIRE(world.job_system().worker_count() == 1);
    REQUIRE(world.tick() == 0);
}

TEST_CASE("World creates a dynamic body and steps it under gravity", "[world]") {
    WorldConfig cfg;
    cfg.job_workers = 2;
    cfg.gravity = Vec3{0, 0, -9.81};
    World world(cfg);

    BodyDesc desc;
    desc.shape = SphereShape{0.5};
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = 1.0;
    desc.pose.position = Vec3{0, 0, 100.0};
    desc.gravity_enabled = true;
    desc.allow_sleep = false;
    const BodyId id = world.create_body(desc);
    REQUIRE(id.valid());

    const Pose before = world.get_pose(id);
    for (int i = 0; i < 24; ++i) world.step(); // 0.1 s at 240 Hz
    const Pose after = world.get_pose(id);

    REQUIRE(after.position.z < before.position.z);
    REQUIRE(world.tick() == 24);
}

TEST_CASE("World state_hash is stable across two identical runs", "[world]") {
    auto run = [] {
        WorldConfig cfg;
        cfg.job_workers = 2;
        World world(cfg);
        BodyDesc desc;
        desc.shape = SphereShape{0.5};
        desc.pose.position = Vec3{0, 0, 50.0};
        desc.allow_sleep = false;
        world.create_body(desc);
        for (int i = 0; i < 50; ++i) world.step();
        return world.state_hash();
    };
    REQUIRE(run() == run());
}
