// Validation: box on slope sticks below and slides above the friction
// angle (PLAN.md P1 acceptance).
//
// Static ground box tilted by theta about Y; a dynamic box is placed flush
// against its top face (offset computed with our own Quat/Vec3 math, not
// hand-derived trig, so the contact starts exactly touching regardless of
// rotation sign convention) with matching orientation. Both bodies get the
// same friction coefficient mu, so the combined friction Jolt uses is mu
// regardless of which symmetric combine rule it applies (min/max/sqrt(a*b)
// all equal mu when both inputs are mu). Classical result (Wong, "Theory
// of Ground Vehicles", statics of a body on an incline): the body is in
// static equilibrium iff tan(theta) <= mu (theta <= the friction angle
// atan(mu)); above that angle, gravity's component along the slope
// exceeds the maximum available friction force and the body accelerates
// down the slope at g*(sin(theta) - mu*cos(theta)).

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ps;

namespace {

double slide_distance(double theta_rad, double mu, double duration_s) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = 4;
    World world(cfg);

    const Quat rot = Quat::from_axis_angle(Vec3::unit_y(), theta_rad);
    constexpr double kGroundHalfZ = 1.0;
    constexpr double kBoxHalfZ = 0.5;

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{50.0, 50.0, kGroundHalfZ}};
    ground.motion = BodyMotionType::Static;
    ground.pose.orientation = rot;
    ground.friction = mu;
    world.create_body(ground);

    const Vec3 box_pos = rot.rotate(Vec3{0.0, 0.0, kGroundHalfZ + kBoxHalfZ});
    BodyDesc box;
    box.shape = BoxShape{Vec3{0.5, 0.5, kBoxHalfZ}};
    box.motion = BodyMotionType::Dynamic;
    box.mass = 10.0;
    box.pose.orientation = rot;
    box.pose.position = box_pos;
    box.friction = mu;
    box.allow_sleep = false;
    box.linear_damping = 0.0;
    box.angular_damping = 0.0;
    const BodyId box_id = world.create_body(box);

    const Vec3 start_pos = world.get_pose(box_id).position;
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t i = 0; i < num_ticks; ++i) world.step();
    const Vec3 end_pos = world.get_pose(box_id).position;

    // Displacement projected onto the slope's downhill tangent direction.
    const Vec3 tangent = rot.rotate(Vec3::unit_x());
    return dot(end_pos - start_pos, tangent);
}

} // namespace

TEST_CASE("Box on a slope below the friction angle does not slide", "[validation][box_on_slope]") {
    const double mu = 0.5;                 // friction angle atan(0.5) = 26.57 deg
    const double theta = 15.0 * 3.14159265358979323846 / 180.0; // well below the friction angle
    const double dist = slide_distance(theta, mu, 1.5);
    INFO("slide_distance=" << dist);
    REQUIRE(std::abs(dist) < 0.01); // < 1 cm of settling wobble, no sustained slide
}

TEST_CASE("Box on a slope above the friction angle slides", "[validation][box_on_slope]") {
    const double mu = 0.5;
    const double theta = 40.0 * 3.14159265358979323846 / 180.0; // well above the friction angle
    const double dist = slide_distance(theta, mu, 1.0);
    INFO("slide_distance=" << dist);
    REQUIRE(dist > 0.3); // clearly sliding downhill (analytic accel ~2.5 m/s^2 -> ~1.3 m in 1 s)
}
