// Validation: huge forces (PLAN.md session Part B.3c: "Sebastian cares
// about errors that change behaviour: huge forces, objects clipping
// through terrain").
//
// A 1500 kg box resting on static ground, with a static wall to one side.
// Phase 1: settle under gravity alone. Phase 2: a 1 MN downward force for
// exactly one tick (pressing it into the ground). Phase 3: a 100 kN
// continuous force sideways for 2 s (pressing it into the wall). Require:
// no tunnelling through ground or wall at any point; penetration < 2 cm
// while a huge force is actively driving the box into a surface and < 1 cm
// once it has settled again; and no energy gain - the box's mechanical
// energy must never exceed what our own applied force actually put in
// (gravity's own contribution is already inside the mechanical-energy
// bookkeeping via the PE term, so this isolates energy invented by the
// solver itself, e.g. a buggy restitution/Baumgarte impulse, from energy
// legitimately supplied by us or by gravity).

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace ps;

namespace {

// Applies an externally-settable constant wrench to one body. evaluate() is
// only ever called from World's own force-element stage (single element
// here, so no concurrent-access concern - PLAN.md D8's "safe to call
// concurrently with OTHER elements' evaluate()" still holds trivially);
// set_wrench() is called from the test's main thread strictly between
// world.step() calls, so there is a happens-before edge through step()
// itself and no data race.
class ExternalWrenchElement final : public IForceElement {
public:
    explicit ExternalWrenchElement(BodyId body) : frame_(body) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext&) const override { return wrench_; }
    void set_wrench(const Wrench& w) { wrench_ = w; }

private:
    Frame frame_;
    Wrench wrench_{};
};

double mechanical_energy(World& world, BodyId id, double z_ref) {
    const double m = world.get_mass(id);
    const Motion mo = world.get_motion(id);
    const Pose p = world.get_pose(id);
    const Mat3 inertia_world = world.backend().get_inertia_world(id);
    const double ke_lin = 0.5 * m * mo.linear.length_squared();
    const Vec3 i_w = inertia_world * mo.angular;
    const double ke_ang = 0.5 * dot(mo.angular, i_w);
    const double pe = m * 9.81 * (p.position.z - z_ref);
    return ke_lin + ke_ang + pe;
}

} // namespace

TEST_CASE("A 1500 kg box under huge forces does not tunnel, over-penetrate or gain energy", "[validation][huge_forces]") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = 1;
    World world(cfg);

    constexpr double kGroundHalfZ = 1.0;
    constexpr Vec3 kBoxHalf{2.25, 0.9, 0.7};
    constexpr double kMass = 1500.0;
    constexpr double kWallGap = 0.05;
    constexpr double kWallHalfX = 0.1;

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{50.0, 50.0, kGroundHalfZ}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 0.8;
    world.create_body(ground);

    const double wall_inner_x = kBoxHalf.x + kWallGap; // box starts at x=0
    BodyDesc wall;
    wall.shape = BoxShape{Vec3{kWallHalfX, 5.0, 2.0}};
    wall.motion = BodyMotionType::Static;
    wall.friction = 0.8;
    wall.pose.position = Vec3{wall_inner_x + kWallHalfX, 0, kGroundHalfZ};
    world.create_body(wall);

    BodyDesc box;
    box.shape = BoxShape{kBoxHalf};
    box.motion = BodyMotionType::Dynamic;
    box.mass = kMass;
    box.pose.position = Vec3{0, 0, kGroundHalfZ + kBoxHalf.z + 0.01};
    box.friction = 0.8;
    box.allow_sleep = false;
    box.motion_quality = MotionQuality::LinearCast; // a 1 MN impulsive tick is exactly the huge-relative-velocity case LinearCast exists for
    const BodyId box_id = world.create_body(box);

    ExternalWrenchElement wrench_element(box_id);
    world.add_force_element(&wrench_element);

    const double ground_top = kGroundHalfZ;
    const double resting_center_z = ground_top + kBoxHalf.z;

    auto ground_penetration_m = [&] {
        return std::max(0.0, resting_center_z - world.get_pose(box_id).position.z);
    };
    auto wall_penetration_m = [&] {
        const double box_face_x = world.get_pose(box_id).position.x + kBoxHalf.x;
        return std::max(0.0, box_face_x - wall_inner_x);
    };

    // --- Phase 1: settle under gravity alone. ---
    for (int i = 0; i < 480; ++i) world.step();
    REQUIRE(ground_penetration_m() < 0.01);

    // --- Phase 2: 1 MN downward for exactly one tick. ---
    const double e_before_phase2 = mechanical_energy(world, box_id, ground_top);
    double work_phase2 = 0.0;
    double max_energy_violation_phase2 = 0.0;
    double max_ground_pen_during = 0.0;
    {
        const Vec3 f{0, 0, -1.0e6};
        wrench_element.set_wrench(Wrench{f, Vec3::zero()});
        const Vec3 pos_before = world.get_pose(box_id).position;
        world.step();
        const Vec3 pos_after = world.get_pose(box_id).position;
        work_phase2 += dot(f, pos_after - pos_before);
        wrench_element.set_wrench(Wrench{});
        max_ground_pen_during = std::max(max_ground_pen_during, ground_penetration_m());
        const double e_now = mechanical_energy(world, box_id, ground_top);
        max_energy_violation_phase2 = std::max(max_energy_violation_phase2, e_now - e_before_phase2 - work_phase2);
    }
    INFO("phase2: max_ground_pen_during_mm=" << (max_ground_pen_during * 1000.0)
         << " work_J=" << work_phase2 << " energy_violation_J=" << max_energy_violation_phase2);
    CHECK(max_ground_pen_during < 0.02);
    CHECK(max_energy_violation_phase2 < std::max(1.0, 0.05 * std::abs(work_phase2)));

    // Settle for 1 s after the impulsive tick.
    for (int i = 0; i < 240; ++i) world.step();
    INFO("phase2 settled: ground_pen_mm=" << (ground_penetration_m() * 1000.0));
    CHECK(ground_penetration_m() < 0.01);

    // --- Phase 3: 100 kN sideways (+X, toward the wall) for 2 s. ---
    const double e_before_phase3 = mechanical_energy(world, box_id, ground_top);
    double work_phase3 = 0.0;
    double max_energy_violation_phase3 = 0.0;
    double max_wall_pen_during = 0.0;
    double max_ground_pen_phase3 = 0.0;
    {
        const Vec3 f{1.0e5, 0, 0};
        wrench_element.set_wrench(Wrench{f, Vec3::zero()});
        for (int i = 0; i < 480; ++i) {
            const Vec3 pos_before = world.get_pose(box_id).position;
            world.step();
            const Vec3 pos_after = world.get_pose(box_id).position;
            work_phase3 += dot(f, pos_after - pos_before);
            max_wall_pen_during = std::max(max_wall_pen_during, wall_penetration_m());
            max_ground_pen_phase3 = std::max(max_ground_pen_phase3, ground_penetration_m());
            const double e_now = mechanical_energy(world, box_id, ground_top);
            max_energy_violation_phase3 = std::max(max_energy_violation_phase3, e_now - e_before_phase3 - work_phase3);
        }
        wrench_element.set_wrench(Wrench{});
    }
    INFO("phase3: max_wall_pen_during_mm=" << (max_wall_pen_during * 1000.0)
         << " max_ground_pen_during_mm=" << (max_ground_pen_phase3 * 1000.0)
         << " work_J=" << work_phase3 << " energy_violation_J=" << max_energy_violation_phase3);
    CHECK(max_wall_pen_during < 0.02);
    CHECK(max_ground_pen_phase3 < 0.02);
    CHECK(max_energy_violation_phase3 < std::max(1.0, 0.05 * std::abs(work_phase3)));

    // Settle for 1 s after the sideways push stops.
    for (int i = 0; i < 240; ++i) world.step();
    INFO("phase3 settled: wall_pen_mm=" << (wall_penetration_m() * 1000.0) << " ground_pen_mm=" << (ground_penetration_m() * 1000.0));
    CHECK(wall_penetration_m() < 0.01);
    CHECK(ground_penetration_m() < 0.01);
}
