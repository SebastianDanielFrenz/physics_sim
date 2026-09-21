// Validation: pendulum period within 0.1% (PLAN.md P1 acceptance).
//
// Physical pendulum (not point-mass simple pendulum, since the bob is a
// solid sphere of non-zero radius): released from rest at a SMALL angle
// theta0 from the vertical so the small-angle period formula applies
// (amplitude correction factor is 1 + theta0^2/16 + ...; at theta0 = 2 deg
// = 0.0349 rad that is 1 + 7.6e-5, ~50x smaller than the 0.1% budget, so it
// is not the dominant error source). Source: Milliken "Race Car Vehicle
// Dynamics" ch. on general rigid-body dynamics / any classical mechanics
// text (physical pendulum): T = 2*pi*sqrt(I_pivot / (m*g*L)), with
// I_pivot = I_cm + m*L^2 (parallel axis theorem) and I_cm = (2/5)*m*r^2
// for a solid sphere.
//
// Period is measured from the simulation as twice the time between two
// consecutive equilibrium crossings of the hinge angular velocity (a
// pendulum crosses its equilibrium twice per period), linearly
// interpolated between the two ticks straddling each crossing so the
// 1/240 s tick resolution (~0.15% of the ~2.8 s period on its own) does
// not dominate the measurement.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace ps;

TEST_CASE("Pendulum period matches the physical-pendulum formula within 0.1%", "[validation][pendulum]") {
    constexpr double kPi = 3.14159265358979323846;
    const double g = 9.81;
    const double L = 2.0;     // pivot-to-COM distance (m)
    const double r = 0.05;    // sphere radius (m)
    const double m = 1.0;     // mass (kg)
    const double theta0 = 2.0 * kPi / 180.0; // 2 degrees

    const double I_cm = 0.4 * m * r * r;
    const double I_pivot = I_cm + m * L * L;
    const double expected_period = 2.0 * kPi * std::sqrt(I_pivot / (m * g * L));

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -g};
    cfg.job_workers = 4;
    World world(cfg);

    const Vec3 bob_pos{L * std::sin(theta0), 0.0, -L * std::cos(theta0)};
    BodyDesc desc;
    desc.shape = SphereShape{r};
    desc.motion = BodyMotionType::Dynamic;
    desc.mass = m;
    desc.pose.position = bob_pos;
    desc.gravity_enabled = true;
    desc.allow_sleep = false;
    desc.linear_damping = 0.0;
    desc.angular_damping = 0.0;
    const BodyId bob = world.create_body(desc);

    JointDesc joint;
    joint.type = JointType::Hinge;
    joint.body_a = bob;
    joint.anchor_a_local = Vec3::zero() - bob_pos; // world origin, in body-local space (identity orientation)
    joint.anchor_b_local = Vec3::zero();           // world point (body_b invalid)
    joint.axis_a_local = Vec3::unit_y();
    joint.axis_b_local = Vec3::unit_y();
    joint.friction_torque = 0.0;
    world.create_joint(joint);

    std::vector<double> crossing_times;
    double prev_w = 0.0;
    double t = 0.0;
    const double dt = world.dt();

    // Run long enough for ~4 periods (small margin) at up to 3 crossings needed.
    const int max_ticks = static_cast<int>((expected_period * 4.0) / dt);
    for (int i = 0; i < max_ticks && crossing_times.size() < 5; ++i) {
        world.step();
        t += dt;
        const double w = world.get_motion(bob).angular.y;
        if (i > 0 && prev_w * w < 0.0) {
            // Linear interpolation of the zero-crossing between (t-dt, prev_w) and (t, w).
            const double frac = -prev_w / (w - prev_w);
            crossing_times.push_back((t - dt) + frac * dt);
        }
        prev_w = w;
    }

    REQUIRE(crossing_times.size() >= 3);
    const double half_period_1 = crossing_times[1] - crossing_times[0];
    const double half_period_2 = crossing_times[2] - crossing_times[1];
    const double measured_period = half_period_1 + half_period_2;

    const double rel_err = std::abs(measured_period - expected_period) / expected_period;
    INFO("expected_period=" << expected_period << " measured_period=" << measured_period << " rel_err=" << rel_err);
    REQUIRE(rel_err < 1e-3); // PLAN.md: "within 0.1%"
}
