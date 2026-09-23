// PLAN.md D10/P1.5a + P2 adapter report, this task's step 3: WorldSnapshot
// grew two additive fields for a future Godot (or other) adapter running on
// its own thread (PLAN.md D3) - articulation link poses (previously only
// reachable via World::joint_position/link_frame, which are NOT
// thread-safe to call from a reader thread while the sim thread is
// stepping) and a race-free per-tick copy of contact events (previously
// only reachable via World::last_contact_events(), a live reference the
// sim thread's NEXT tick overwrites in place with no synchronisation -
// see snapshot.h's own comment on ArticulationSnapshot/contacts for the
// full reasoning). Both fields travel through the existing
// SnapshotTripleBuffer::publish hand-off, so this file checks they show up
// there with the same values World's own (non-thread-safe) accessors report
// for the same tick - not a new synchronisation mechanism, just additive
// payload on the same trusted path.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

using namespace ps;
using namespace ps::articulation;

namespace {

ArticulationId make_fixed_single_link_articulation(World& world) {
    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose::identity();

    LinkDesc link;
    link.parent = -1;
    link.joint = articulation::JointType::Revolute;
    link.axis = Vec3::unit_y();
    link.mass = 2.0;
    link.com = Vec3{0, 0, -1.5};
    link.inertia = Mat3::diagonal(0.01, 0.01, 0.01);
    link.q0 = 0.3; // released away from equilibrium so link_world_pose is non-trivial
    desc.links.push_back(link);

    return world.create_articulation(desc);
}

} // namespace

TEST_CASE("A World with no articulations publishes an empty WorldSnapshot::articulations (regression)", "[world][snapshot]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    BodyDesc desc;
    desc.shape = SphereShape{0.5};
    desc.pose.position = Vec3{0, 0, 50.0};
    desc.allow_sleep = false;
    world.create_body(desc);

    for (int i = 0; i < 5; ++i) world.step();

    const WorldSnapshot& snap = world.snapshots().read();
    REQUIRE_FALSE(snap.bodies.empty());
    REQUIRE(snap.articulations.empty());
}

TEST_CASE("WorldSnapshot::articulations mirrors System's own link poses", "[world][snapshot][articulation]") {
    WorldConfig cfg;
    cfg.job_workers = 2;
    World world(cfg);

    const ArticulationId art = make_fixed_single_link_articulation(world);
    for (int i = 0; i < 10; ++i) world.step();

    // Ground truth via World's own (non-thread-safe, sim-thread-only) accessors.
    const Pose expected_root_pose = world.root_pose(art);
    const Motion expected_root_motion = world.root_motion(art);
    const Frame link0_frame = world.link_frame(art, 0);
    const Pose expected_link0_pose = link0_frame.world_pose(world.backend());
    const Motion expected_link0_motion = link0_frame.world_motion(world.backend());

    const WorldSnapshot& snap = world.snapshots().read();
    REQUIRE(snap.articulations.size() == 1);
    const ArticulationSnapshot& asnap = snap.articulations[0];
    REQUIRE(asnap.id == art);
    REQUIRE(asnap.links.size() == 2); // root + the one link

    const ArticulationLinkSnapshot& root_link = asnap.links[0];
    REQUIRE(root_link.link_index == -1);
    REQUIRE(root_link.pose.position.x == expected_root_pose.position.x);
    REQUIRE(root_link.pose.position.y == expected_root_pose.position.y);
    REQUIRE(root_link.pose.position.z == expected_root_pose.position.z);
    REQUIRE(root_link.motion.linear.x == expected_root_motion.linear.x);

    const ArticulationLinkSnapshot& link0 = asnap.links[1];
    REQUIRE(link0.link_index == 0);
    REQUIRE(link0.pose.position.x == expected_link0_pose.position.x);
    REQUIRE(link0.pose.position.y == expected_link0_pose.position.y);
    REQUIRE(link0.pose.position.z == expected_link0_pose.position.z);
    REQUIRE(link0.motion.angular.y == expected_link0_motion.angular.y);
}

TEST_CASE("WorldSnapshot::contacts is a race-free copy of last_contact_events for the same tick", "[world][snapshot][contacts]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    World world(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{5.0, 5.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    world.create_body(ground);

    BodyDesc ball;
    ball.shape = SphereShape{0.3};
    ball.motion = BodyMotionType::Dynamic;
    ball.mass = 1.0;
    ball.pose.position = Vec3{0, 0, 2.0};
    ball.allow_sleep = false;
    world.create_body(ball);

    bool saw_nonempty_contacts = false;
    for (int i = 0; i < 240 && !saw_nonempty_contacts; ++i) {
        world.step();
        const auto& live = world.last_contact_events();
        const auto& snap_contacts = world.snapshots().read().contacts;
        // Same tick, same content (this test's own single sim thread calls
        // both back to back, so there is no actual race here - the point is
        // that the SNAPSHOT'S copy independently matches what the live
        // accessor reported for this same tick, which is what a
        // cross-thread reader depends on).
        REQUIRE(snap_contacts.size() == live.size());
        for (std::size_t j = 0; j < live.size(); ++j) {
            REQUIRE(snap_contacts[j].a == live[j].a);
            REQUIRE(snap_contacts[j].b == live[j].b);
            REQUIRE(snap_contacts[j].surface_a == live[j].surface_a);
            REQUIRE(snap_contacts[j].surface_b == live[j].surface_b);
        }
        if (!live.empty()) saw_nonempty_contacts = true;
    }
    REQUIRE(saw_nonempty_contacts); // the ball must actually have landed on the ground within 1 s
}
