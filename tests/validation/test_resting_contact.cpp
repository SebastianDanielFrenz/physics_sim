// Validation: resting-contact penetration (PLAN.md session Part B.3b).
//
// A 1500 kg box (car-sized, 4.5x1.8x1.4 m) and a 20 t box (combine-sized,
// 8x4x3.5 m) dropped from 0.5 m onto a static ground box. After settling,
// penetration must be < 1 cm and stable (no drift) over 30 s.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ps;

namespace {

struct SettleResult {
    double max_penetration_m = 0.0; // over the whole tail window
    double drift_m = 0.0;           // max(z) - min(z) over the tail window
};

SettleResult drop_box_on_ground(const Vec3& half_extents, double mass, double drop_height,
                                 double settle_s = 5.0, double tail_s = 30.0) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = 1;
    World world(cfg);

    constexpr double kGroundHalfZ = 1.0;
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{50.0, 50.0, kGroundHalfZ}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 0.8;
    world.create_body(ground);

    BodyDesc box;
    box.shape = BoxShape{half_extents};
    box.motion = BodyMotionType::Dynamic;
    box.mass = mass;
    box.pose.position = Vec3{0, 0, kGroundHalfZ + half_extents.z + drop_height};
    box.friction = 0.8;
    box.allow_sleep = false;
    box.linear_damping = 0.0;
    box.angular_damping = 0.0;
    const BodyId box_id = world.create_body(box);

    const double ground_top_z = kGroundHalfZ;
    const double resting_center_z = ground_top_z + half_extents.z;

    // 5 s to fall (drop_height is small; free fall to 0.5 m takes ~0.32 s)
    // and settle, then a 30 s tail window used for both the penetration and
    // drift checks (PLAN.md: "stable (no drift over 30 s)"). settle_s/tail_s
    // are parameters so the short [sanitizer] variant below can reuse this
    // function with a shorter tail - penetration and drift are steady-state
    // properties here (the box has long since stopped moving), not
    // quantities that grow with run length, so the same thresholds hold.
    const std::uint64_t settle_ticks = static_cast<std::uint64_t>(settle_s * cfg.tick_rate_hz + 0.5);
    const std::uint64_t tail_ticks = static_cast<std::uint64_t>(tail_s * cfg.tick_rate_hz + 0.5);

    for (std::uint64_t i = 0; i < settle_ticks; ++i) world.step();

    double min_z = world.get_pose(box_id).position.z;
    double max_z = min_z;
    double max_penetration = 0.0;
    for (std::uint64_t i = 0; i < tail_ticks; ++i) {
        world.step();
        const double z = world.get_pose(box_id).position.z;
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
        const double penetration = resting_center_z - z; // positive == sunk into the ground
        max_penetration = std::max(max_penetration, penetration);
    }

    SettleResult r;
    r.max_penetration_m = max_penetration;
    r.drift_m = max_z - min_z;
    return r;
}

} // namespace

TEST_CASE("A 1500 kg car-sized box settles on the ground with < 1 cm penetration and no drift", "[validation][resting_contact][long][backend]") {
    const SettleResult r = drop_box_on_ground(Vec3{2.25, 0.9, 0.7}, 1500.0, 0.5);
    INFO("max_penetration_mm=" << (r.max_penetration_m * 1000.0) << " drift_mm=" << (r.drift_m * 1000.0));
    CHECK(r.max_penetration_m < 0.01);
    CHECK(r.drift_m < 0.01);
}

TEST_CASE("A 20 t combine-sized box settles on the ground with < 1 cm penetration and no drift", "[validation][resting_contact][long][backend]") {
    const SettleResult r = drop_box_on_ground(Vec3{4.0, 2.0, 1.75}, 20000.0, 0.5);
    INFO("max_penetration_mm=" << (r.max_penetration_m * 1000.0) << " drift_mm=" << (r.drift_m * 1000.0));
    CHECK(r.max_penetration_m < 0.01);
    CHECK(r.drift_m < 0.01);
}

// Short [sanitizer] variant of the car-sized case only (same contact code
// path as the combine-sized one, just a different mass/size - one smoke
// test is enough here). 1 s settle + 2 s tail instead of 5 s + 30 s, same
// thresholds (see drop_box_on_ground's comment on why they still hold).
// Runs on the debug/asan/linux-asan-ubsan/linux-tsan legs by default -
// exercises Jolt resting contact under those sanitizers, which those legs
// would otherwise skip entirely once the two tests above are [long].
TEST_CASE("A 1500 kg car-sized box settles on the ground, short settle+tail (short)", "[resting_contact][sanitizer][backend]") {
    const SettleResult r = drop_box_on_ground(Vec3{2.25, 0.9, 0.7}, 1500.0, 0.5, 1.0, 2.0);
    INFO("max_penetration_mm=" << (r.max_penetration_m * 1000.0) << " drift_mm=" << (r.drift_m * 1000.0));
    CHECK(r.max_penetration_m < 0.01);
    CHECK(r.drift_m < 0.01);
}
