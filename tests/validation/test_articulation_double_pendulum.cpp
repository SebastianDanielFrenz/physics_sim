// Validation: double-pendulum articulation normal-mode frequencies match
// the analytic small-amplitude values within 0.1% (PLAN.md P1.5a
// acceptance).
//
// Classic point-mass double pendulum, equal masses m and equal lengths l
// (each link's bob has negligible own radius, so its rotational inertia
// about its own COM is negligible - a "rod + point mass" idealisation).
// Standard small-oscillation Lagrangian linearisation (e.g. Goldstein,
// "Classical Mechanics", the double-pendulum normal-mode example) gives,
// with lambda = omega^2*l/g:
//   M/(m l^2) = [[2,1],[1,1]],  K/(mgl) = [[2,0],[0,1]]
//   det(K - lambda*M) = 0  =>  lambda^2 - 4*lambda + 2 = 0
//   lambda = 2 -+ sqrt(2)
// with eigenvectors theta2/theta1 = +sqrt(2) (lower frequency, in-phase)
// and -sqrt(2) (higher frequency, out-of-phase) - both re-derived in this
// comment from the Lagrangian above rather than only quoted, so the
// numbers can be checked independently.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace ps;
using namespace ps::articulation;

namespace {
constexpr double kPi = 3.14159265358979323846;

// theta1_0/theta2_0 are ABSOLUTE angles from vertical (matching the
// Lagrangian derivation in this file's header comment, which uses
// absolute angles). link1's own q0 is a joint angle RELATIVE to link0
// (standard serial-chain convention, model.h/model.cpp) - since both
// joints share the same axis, the absolute angle of link1 is q0+q1, so
// the relative q0 to request is theta2_0 - theta1_0.
ArticulationId build(World& world, double l, double m, double theta1_0, double theta2_0) {
    ArticulationDesc desc;
    desc.root.floating = false;

    LinkDesc link0;
    link0.parent = -1;
    link0.joint = articulation::JointType::Revolute;
    link0.axis = Vec3::unit_y();
    link0.mass = m;
    link0.com = Vec3{0, 0, -l};
    link0.inertia = Mat3::diagonal(1e-6, 1e-6, 1e-6);
    link0.q0 = theta1_0;
    desc.links.push_back(link0);

    LinkDesc link1;
    link1.parent = 0;
    link1.joint = articulation::JointType::Revolute;
    link1.joint_frame_in_parent.position = Vec3{0, 0, -l};
    link1.axis = Vec3::unit_y();
    link1.mass = m;
    link1.com = Vec3{0, 0, -l};
    link1.inertia = Mat3::diagonal(1e-6, 1e-6, 1e-6);
    link1.q0 = theta2_0 - theta1_0;
    desc.links.push_back(link1);

    return world.create_articulation(desc);
}

// Measures the dominant oscillation frequency of theta1(t) via zero
// crossings (same technique test_pendulum.cpp/test_articulation_pendulum
// use) - valid here because each run is seeded with a pure eigenvector, so
// theta1(t) oscillates at a single frequency, not a superposition.
double measure_frequency(World& world, ArticulationId art, double expected_period) {
    std::vector<double> crossing_times;
    double prev_w = world.joint_velocity(art, 0);
    double t = 0.0;
    const double dt = world.dt();
    const int max_ticks = static_cast<int>((expected_period * 6.5) / dt);
    for (int i = 0; i < max_ticks && crossing_times.size() < 5; ++i) {
        world.step();
        t += dt;
        const double w = world.joint_velocity(art, 0);
        if (prev_w * w < 0.0) {
            const double frac = -prev_w / (w - prev_w);
            crossing_times.push_back((t - dt) + frac * dt);
        }
        prev_w = w;
    }
    REQUIRE(crossing_times.size() >= 3);
    const double period = (crossing_times[1] - crossing_times[0]) + (crossing_times[2] - crossing_times[1]);
    return 2.0 * kPi / period;
}

void run_mode(bool lower_mode) {
    const double g = 9.81;
    const double l = 1.0;
    const double m = 1.0;
    const double eps = 0.01; // rad, small-amplitude (linear) regime

    const double lambda = lower_mode ? (2.0 - std::sqrt(2.0)) : (2.0 + std::sqrt(2.0));
    const double omega_expected = std::sqrt(lambda * g / l);
    const double period_expected = 2.0 * kPi / omega_expected;
    const double ratio = lower_mode ? std::sqrt(2.0) : -std::sqrt(2.0);

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -g};
    World world(cfg);

    const ArticulationId art = build(world, l, m, eps, eps * ratio);
    const double omega_measured = measure_frequency(world, art, period_expected);
    const double rel_err = std::abs(omega_measured - omega_expected) / omega_expected;
    INFO("mode=" << (lower_mode ? "lower" : "upper") << " expected=" << omega_expected << " measured=" << omega_measured << " rel_err=" << rel_err);
    REQUIRE(rel_err < 1e-3);
}

} // namespace

TEST_CASE("Double-pendulum lower normal-mode frequency matches analytic value", "[validation][articulation][double_pendulum]") {
    run_mode(true);
}

TEST_CASE("Double-pendulum upper normal-mode frequency matches analytic value", "[validation][articulation][double_pendulum]") {
    run_mode(false);
}
