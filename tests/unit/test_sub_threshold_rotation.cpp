// tests/unit/test_sub_threshold_rotation.cpp — parking-slope creep fix,
// defect D2: Jolt's own Body::AddRotationStep (Jolt/Physics/Body/Body.inl)
// silently drops any rotation step with |omega * dt_substep| <= 1.0e-6f
// rad, so a body can carry a small but real, steady angular velocity that
// never integrates into its orientation. See
// IRigidBackend::set_integrate_sub_threshold_rotation's own comment
// (ps/backend/i_rigid_backend.h) and CLAUDE.md's "Jolt backend" section for
// the full mechanism and why World::create_vehicle opts the chassis body in
// while every other body stays opted out (default off, hash-neutral for
// every existing resting-body scenario).

#include "ps/math/transcendental.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ps;

namespace {

// A free box, zero gravity, spinning slowly about world Z with no applied
// torque - the angular velocity itself stays exactly constant throughout
// (no gyroscopic coupling on a call with a single fixed-axis omega and no
// forces), so any change in orientation is due entirely to whether the
// dropped rotation step is being reapplied.
BodyId make_free_box(World& world, real omega_z) {
    BodyDesc desc;
    desc.shape = BoxShape{Vec3{0.5, 0.5, 0.5}};
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = 1.0;
    desc.gravity_enabled = false;
    desc.allow_sleep = false;
    desc.velocity.angular = Vec3{0.0, 0.0, omega_z};
    return world.create_body(desc);
}

// The rotation is exactly about world Z throughout (constant omega, no
// other torque), so the quaternion's own (z, w) pair gives the exact swept
// angle via 2*atan2(z, w) - robust at both the near-zero (opted-out) and
// small-but-nonzero (opted-in) ends, unlike a small-angle approximation.
real z_angle_between(const Quat& before, const Quat& after) {
    const Quat rel = before.inverse() * after;
    return real(2.0) * ps::math::atan2(rel.z, rel.w);
}

} // namespace

TEST_CASE("Sub-threshold rotation is dropped by default", "[world][backend]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.gravity = Vec3{0.0, 0.0, 0.0};
    World world(cfg);

    const BodyId id = make_free_box(world, 1e-4); // 240 Hz tick -> |omega*dt| ~= 4.2e-7 rad, under Jolt's 1e-6f dead zone
    const Quat before = world.get_pose(id).orientation;
    for (int i = 0; i < 2400; ++i) world.step(); // 10 s at 240 Hz
    const Quat after = world.get_pose(id).orientation;

    CHECK(z_angle_between(before, after) == real(0.0));
}

TEST_CASE("Opted-in sub-threshold rotation integrates to the expected angle", "[world][backend]") {
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.gravity = Vec3{0.0, 0.0, 0.0};
    World world(cfg);

    const BodyId id = make_free_box(world, 1e-4);
    world.backend().set_integrate_sub_threshold_rotation(id, true);

    const Quat before = world.get_pose(id).orientation;
    for (int i = 0; i < 2400; ++i) world.step(); // 10 s at 240 Hz
    const Quat after = world.get_pose(id).orientation;

    // omega * t = 1e-4 rad/s * 10 s = 1e-3 rad exactly, matching the task's
    // own spec ("rotates 1e-3 rad in 10 s when opted in"). Margin, not a
    // tighter epsilon: Jolt's own rotation state is float32 (JPH::Quat),
    // and 2400 ticks of Quat::sRotation + Normalized() accumulate float32-
    // level round-off (measured ~1.3e-8 rad here) - still 5 orders of
    // magnitude away from mistaking this for the "dropped" (0 rad) case.
    CHECK(z_angle_between(before, after) == Catch::Approx(1e-3).margin(1e-6));
}
