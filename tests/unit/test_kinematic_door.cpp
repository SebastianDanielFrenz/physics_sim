// tests/unit/test_kinematic_door.cpp - kinematic bodies and the door driver
// (World::move_kinematic, World::create_door, ps/world/door.h). Tag [kinematic].
//
// Recorded sabotages (each makes the named cases fail; results in the commit message):
//   S1 Jolt move_kinematic -> SetPositionAndRotation (teleport): K1, K2
//   S2 door probe ignores every body: K3, K4, K5
//   S3 stop_kinematic does not deactivate: K6
//   S4 door state folded into the hash even with no door / not folded at all: K7
//   S5 require_kinematic accepts any body: K8
//   S6 mass limit ignored (push everything): K3, K4, K5

#include "ps/jobs/debug_alloc.h"
#include "ps/world/door.h"
#include "ps/world/world.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ps;
using Catch::Approx;

namespace {

std::unique_ptr<World> make_world(int workers = 2) {
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.gravity = Vec3{0, 0, 0};
    return std::make_unique<World>(cfg);
}

BodyId add_door_body(World& w, const Pose& pose, const Vec3& half) {
    BodyDesc d;
    d.shape = BoxShape{half};
    d.motion = BodyMotionType::Kinematic;
    d.pose = pose;
    d.allow_sleep = true;
    d.gravity_enabled = false;
    return w.create_body(d);
}

BodyId add_box(World& w, const Vec3& pos, const Vec3& half, real mass) {
    BodyDesc d;
    d.shape = BoxShape{half};
    d.motion = BodyMotionType::Dynamic;
    d.mass = mass;
    d.pose = Pose{pos, Quat::identity()};
    d.friction = 0.0;
    d.gravity_enabled = false;
    d.allow_sleep = false;
    return w.create_body(d);
}

// Sliding door: 0.1 m thick slab at x=0, slides +x by 3 m.
DoorDesc sliding_desc(real speed, const Vec3& half = Vec3{0.05, 1.0, 1.0}) {
    DoorDesc d;
    d.kind = DoorKind::Sliding;
    d.closed = Pose{Vec3{0, 0, 0}, Quat::identity()};
    d.slide = Vec3{3.0, 0, 0};
    d.speed = speed;
    d.probe_shape = BoxShape{half};
    return d;
}

} // namespace

TEST_CASE("K1 a sliding door pushes a free box at the door speed", "[kinematic][world][backend]") {
    auto wp = make_world();
    World& w = *wp;
    const BodyId door_body = add_door_body(w, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
    const BodyId box = add_box(w, Vec3{0.5, 0, 0}, Vec3{0.25, 0.25, 0.25}, 10.0);
    const DoorId door = w.create_door(door_body, sliding_desc(1.0));
    w.set_door_command(door, 1.0);
    for (int i = 0; i < 360; ++i) w.step(); // 1.5 s
    const DoorState st = w.door_state(door);
    WARN("K1 box vx " << w.get_motion(box).linear.x << " fraction " << st.fraction << " door speed " << w.get_motion(door_body).linear.x);
    CHECK_FALSE(st.blocked);
    CHECK(w.get_motion(box).linear.x == Approx(1.0).margin(0.02));
    CHECK(st.fraction == Approx(0.5).margin(0.01));
    // The box rides on the door face: rear face of the box at the door front face.
    const real gap = (w.get_pose(box).position.x - 0.25) - (w.get_pose(door_body).position.x + 0.05);
    CHECK(std::abs(gap) < 0.02);
}

TEST_CASE("K2 the backend sees a real contact velocity on a sliding and a hinged door", "[kinematic][world][backend]") {
    SECTION("sliding") {
        auto wp = make_world();
        World& w = *wp;
        const BodyId sb = add_door_body(w, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
        const DoorId sd = w.create_door(sb, sliding_desc(1.0));
        w.set_door_command(sd, 1.0);
        for (int i = 0; i < 10; ++i) w.step();
        CHECK(w.get_motion(sb).linear.x == Approx(1.0).margin(1e-3));
    }
    SECTION("hinged") {
        auto hp = make_world();
        World& h = *hp;
        const BodyId hb = add_door_body(h, Pose{Vec3{0.5, 0, 1.0}, Quat::identity()}, Vec3{0.5, 0.05, 1.0});
        DoorDesc d;
        d.kind = DoorKind::Hinged;
        d.closed = Pose{Vec3{0.5, 0, 1.0}, Quat::identity()};
        d.pivot = Vec3{0, 0, 0};
        d.axis = Vec3{0, 0, 1};
        d.open_angle_rad = 1.5707963267948966;
        d.speed = 1.0;
        d.probe_shape = BoxShape{Vec3{0.5, 0.05, 1.0}};
        const DoorId hd = h.create_door(hb, d);
        h.set_door_command(hd, 1.0);
        for (int i = 0; i < 10; ++i) h.step();
        CHECK(h.get_motion(hb).angular.z == Approx(1.0).margin(1e-3));
        CHECK(h.get_motion(hb).linear.length() == Approx(0.5).margin(0.01)); // omega * 0.5 m lever
        for (int i = 0; i < 480; ++i) h.step();
        const Pose p = h.get_pose(hb);
        CHECK(p.position.x == Approx(0.0).margin(2e-3));
        CHECK(p.position.y == Approx(0.5).margin(2e-3));
        CHECK(h.door_state(hd).fraction == Approx(1.0));
    }
}

TEST_CASE("K3 a parked car stays put against a closing door that reports blocked", "[kinematic][vehicle]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    World& w = *rig.world;
    w.set_control("handbrake", 1.0);
    ps_test::settle(w, 3.0);
    const Vec3 start = w.get_pose(rig.chassis).position;

    const BodyId door_body = add_door_body(w, Pose{Vec3{6.0, 0, 1.0}, Quat::identity()}, Vec3{0.1, 1.5, 1.0});
    DoorDesc d;
    d.kind = DoorKind::Sliding;
    d.closed = Pose{Vec3{6.0, 0, 1.0}, Quat::identity()};
    d.slide = Vec3{-4.5, 0, 0};
    d.speed = 1.0;
    d.max_push_mass_kg = 100.0;
    d.probe_shape = BoxShape{Vec3{0.1, 1.5, 1.0}};
    const DoorId door = w.create_door(door_body, d);
    w.set_door_command(door, 1.0);
    bool saw_blocked = false;
    for (int i = 0; i < 240 * 8; ++i) {
        w.step();
        saw_blocked = saw_blocked || w.door_state(door).blocked;
    }
    const DoorState st = w.door_state(door);
    WARN("K3 car shift dx " << (w.get_pose(rig.chassis).position.x - start.x) << " dy " << (w.get_pose(rig.chassis).position.y - start.y) << " door fraction " << st.fraction << " blocker pen " << st.blocker_penetration_m << " speed " << w.get_motion(rig.chassis).linear.length());
    CHECK(saw_blocked);
    CHECK(st.blocked);
    CHECK(st.blocker == rig.chassis);
    CHECK(st.fraction < 1.0);
    const Vec3 end = w.get_pose(rig.chassis).position;
    CHECK(std::abs(end.x - start.x) < 0.02);
    CHECK(std::abs(end.y - start.y) < 0.02);
    CHECK(w.get_motion(rig.chassis).linear.length() < 0.05); // not launched
    CHECK_FALSE(w.body_is_active(door_body));                // a held door is asleep
}

TEST_CASE("K4 a blocked door reopens, and resumes closing once the obstacle is gone", "[kinematic][world][backend]") {
    auto wp = make_world();
    World& w = *wp;
    const BodyId door_body = add_door_body(w, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
    const BodyId box = add_box(w, Vec3{0.6, 0, 0}, Vec3{0.25, 0.25, 0.25}, 500.0); // heavier than the push limit
    DoorDesc d = sliding_desc(1.0);
    d.max_push_mass_kg = 100.0;
    const DoorId door = w.create_door(door_body, d);
    w.set_door_command(door, 1.0);
    for (int i = 0; i < 240; ++i) w.step();
    REQUIRE(w.door_state(door).blocked);
    CHECK(w.door_state(door).blocker == box);
    const real held = w.door_state(door).fraction;
    CHECK(w.get_pose(box).position.x == Approx(0.6).margin(0.01)); // never shoved

    w.set_door_command(door, 0.0); // reopen
    for (int i = 0; i < 24; ++i) w.step();
    CHECK_FALSE(w.door_state(door).blocked);
    CHECK(w.door_state(door).fraction < held);

    w.set_door_command(door, 1.0);
    w.destroy_body(box); // remove the obstacle
    for (int i = 0; i < 240; ++i) w.step();
    CHECK_FALSE(w.door_state(door).blocked);
    CHECK(w.door_state(door).fraction > held);
}

TEST_CASE("K5 a door pushes a light box but holds against a heavy one", "[kinematic][world][backend]") {
    for (const real mass : {20.0, 2000.0}) {
        auto wp = make_world();
        World& w = *wp;
        const BodyId door_body = add_door_body(w, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
        const BodyId box = add_box(w, Vec3{0.4, 0, 0}, Vec3{0.25, 0.25, 0.25}, mass);
        DoorDesc d = sliding_desc(1.0);
        d.max_push_mass_kg = 100.0;
        const DoorId door = w.create_door(door_body, d);
        w.set_door_command(door, 1.0);
        for (int i = 0; i < 240; ++i) w.step();
        if (mass < 100.0) {
            CHECK_FALSE(w.door_state(door).blocked);
            CHECK(w.get_pose(box).position.x > 1.0);
        } else {
            CHECK(w.door_state(door).blocked);
            CHECK(w.get_pose(box).position.x < 0.45);
        }
    }
}

TEST_CASE("K6 a door at rest is asleep and costs nothing", "[kinematic][world][backend]") {
    auto wp = make_world();
    World& w = *wp;
    const BodyId door_body = add_door_body(w, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
    const DoorId door = w.create_door(door_body, sliding_desc(1.0));
    CHECK_FALSE(w.body_is_active(door_body));
    for (int i = 0; i < 20; ++i) w.step();
    CHECK_FALSE(w.body_is_active(door_body));
    w.set_door_command(door, 1.0);
    w.step();
    CHECK(w.body_is_active(door_body));
    for (int i = 0; i < 240 * 4; ++i) w.step(); // 3 m at 1 m/s, done
    CHECK(w.door_state(door).fraction == Approx(1.0));
    CHECK_FALSE(w.door_state(door).moving);
    CHECK_FALSE(w.body_is_active(door_body));
    const Pose p = w.get_pose(door_body);
    CHECK(p.position.x == Approx(3.0).margin(1e-4));
    CHECK(w.get_motion(door_body).linear.length() == 0.0);
}

TEST_CASE("K7 doors change the hash only while one exists; 1 vs 10 workers agree", "[kinematic][world][determinism]") {
    auto run = [](int workers, bool with_door) {
        auto wp = make_world(workers);
        World& w = *wp;
        const BodyId door_body = add_door_body(w, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
        add_box(w, Vec3{0.4, 0, 0}, Vec3{0.25, 0.25, 0.25}, 20.0);
        add_box(w, Vec3{4.0, 0.2, 0}, Vec3{0.25, 0.25, 0.25}, 2000.0);
        if (with_door) {
            DoorDesc d = sliding_desc(1.0);
            d.max_push_mass_kg = 100.0;
            const DoorId door = w.create_door(door_body, d);
            w.set_door_command(door, 1.0);
        }
        std::vector<std::uint64_t> hashes;
        for (int i = 0; i < 480; ++i) {
            w.step();
            hashes.push_back(w.state_hash());
        }
        return hashes;
    };
    const auto a1 = run(1, true), a10 = run(10, true);
    CHECK(a1 == a10);
    const auto n1 = run(1, false), n10 = run(10, false);
    CHECK(n1 == n10);
    CHECK(n1.back() != a1.back());
}

TEST_CASE("K7b a world with a door only adds door terms once the door exists", "[kinematic][world][determinism]") {
    auto build = [](bool create) {
        auto wp = make_world(2);
        const BodyId db = add_door_body(*wp, Pose{Vec3{0, 0, 0}, Quat::identity()}, Vec3{0.05, 1.0, 1.0});
        add_box(*wp, Vec3{3.0, 0, 0}, Vec3{0.25, 0.25, 0.25}, 20.0);
        if (create) wp->create_door(db, sliding_desc(1.0));
        return wp;
    };
    auto without = build(false);
    auto with = build(true);
    // Same bodies, door at rest and asleep: every step the hashes differ only by the door term, and the door-less world
    // equals itself between runs.
    auto without2 = build(false);
    for (int i = 0; i < 10; ++i) {
        without->step();
        without2->step();
        with->step();
    }
    CHECK(without->state_hash() == without2->state_hash());
    CHECK(without->state_hash() != with->state_hash());
}

TEST_CASE("K8 rejected calls", "[kinematic][world][backend]") {
    auto wp = make_world();
    World& w = *wp;
    const BodyId door_body = add_door_body(w, Pose::identity(), Vec3{0.05, 1.0, 1.0});
    const BodyId dyn = add_box(w, Vec3{5, 0, 0}, Vec3{0.25, 0.25, 0.25}, 1.0);
    BodyDesc sd;
    sd.shape = BoxShape{};
    sd.motion = BodyMotionType::Static;
    sd.pose = Pose{Vec3{9, 0, 0}, Quat::identity()};
    const BodyId stat = w.create_body(sd);
    const Pose target{Vec3{1, 0, 0}, Quat::identity()};

    CHECK_NOTHROW(w.move_kinematic(door_body, target));
    CHECK_THROWS_AS(w.move_kinematic(dyn, target), std::invalid_argument);
    CHECK_THROWS_AS(w.move_kinematic(stat, target), std::invalid_argument);
    CHECK_THROWS_AS(w.move_kinematic(BodyId{}, target), std::invalid_argument);
    CHECK_THROWS_AS(w.move_kinematic(BodyId{999, 1}, target), std::invalid_argument);
    w.destroy_body(dyn);
    CHECK_THROWS_AS(w.move_kinematic(dyn, target), std::invalid_argument); // stale

    const BodyId dyn2 = add_box(w, Vec3{5, 0, 0}, Vec3{0.25, 0.25, 0.25}, 1.0);
    CHECK_THROWS_AS(w.create_door(dyn2, sliding_desc(1.0)), std::invalid_argument);
    CHECK_THROWS_AS(w.create_door(stat, sliding_desc(1.0)), std::invalid_argument);
    DoorDesc bad = sliding_desc(0.0);
    CHECK_THROWS_AS(w.create_door(door_body, bad), std::invalid_argument);
    bad = sliding_desc(1.0);
    bad.slide = Vec3{0, 0, 0};
    CHECK_THROWS_AS(w.create_door(door_body, bad), std::invalid_argument);
    CHECK_THROWS_AS(w.set_door_command(DoorId{}, 1.0), std::invalid_argument);
    CHECK_THROWS_AS((void)w.door_state(DoorId{7, 1}), std::invalid_argument);
}

TEST_CASE("K9 DoorDriver::tick allocates nothing", "[kinematic][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook compiled out on this build");
        return;
    }
    DoorDriver drv(sliding_desc(1.0));
    DoorState s;
    s.command = 1.0;
    const auto probe = [](const Pose&) { return DoorProbe{}; };
    drv.tick(s, 1.0 / 240.0, probe); // warm-up
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    ps::jobs::debug_alloc::enter_job();
    for (int i = 0; i < 1000; ++i) drv.tick(s, 1.0 / 240.0, probe);
    ps::jobs::debug_alloc::exit_job();
    ps::jobs::debug_alloc::set_armed(false);
    CHECK(ps::jobs::debug_alloc::own_violation_count() == 0);
    CHECK(s.fraction > 0.9);
}

TEST_CASE("K10 DoorDriver path is closed form", "[kinematic]") {
    DoorDriver drv(sliding_desc(1.0));
    CHECK(drv.pose_at(0.5).position.x == Approx(1.5));
    CHECK(drv.pose_at(2.0).position.x == Approx(3.0)); // clamped
    DoorDesc h;
    h.kind = DoorKind::Hinged;
    h.closed = Pose{Vec3{1, 0, 0}, Quat::identity()};
    h.pivot = Vec3{0, 0, 0};
    h.axis = Vec3{0, 0, 1};
    h.open_angle_rad = 3.141592653589793;
    h.speed = 1.0;
    DoorDriver hd(h);
    CHECK(hd.pose_at(1.0).position.x == Approx(-1.0).margin(1e-12));
    CHECK(hd.approach(0.0, 1.0, 1.0 / 240.0) == Approx(1.0 / 240.0 / 3.141592653589793));
}
