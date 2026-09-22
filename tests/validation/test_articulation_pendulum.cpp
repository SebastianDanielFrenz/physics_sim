// Validation: single-pendulum articulation period matches the exact
// elliptic-integral large-amplitude period within 0.1% at 90 and 170 deg
// (PLAN.md P1.5a acceptance).
//
// For ANY physical pendulum (point mass or not) released from rest at
// angle theta0 from the downward equilibrium, the exact large-amplitude
// period is T = 4*sqrt(I_pivot/(m*g*L)) * K(sin(theta0/2)), where K is the
// complete elliptic integral of the first kind and L is the pivot-to-COM
// distance (standard result - the pendulum ODE theta'' = -(mgL/I_pivot)*sin(theta)
// is the same equation regardless of I_pivot's origin, so the same
// elliptic-integral solution applies with omega0^2 = mgL/I_pivot in place
// of g/L). Here the bob's own moment of inertia about its COM is made
// negligible (tiny radius) so I_pivot ~= m*L^2 and the point-mass formula
// applies directly, which also makes this a clean point-mass sanity check.
//
// K(k) computed via the arithmetic-geometric mean (Gauss AGM):
// K(k) = pi / (2*AGM(1, sqrt(1-k^2))) - standard, fast-converging (roughly
// doubles correct digits per iteration).

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace ps;
using namespace ps::articulation;

namespace {
constexpr double kPi = 3.14159265358979323846;

double agm(double a, double b) {
    for (int i = 0; i < 40; ++i) {
        const double a2 = 0.5 * (a + b);
        const double b2 = std::sqrt(a * b);
        a = a2; b = b2;
        if (std::abs(a - b) < 1e-16) break;
    }
    return a;
}

double complete_elliptic_K(double k) {
    return kPi / (2.0 * agm(1.0, std::sqrt(1.0 - k * k)));
}

double measure_period(World& world, ArticulationId art, double expected_period) {
    std::vector<double> crossing_times;
    double prev_w = 0.0;
    double t = 0.0;
    const double dt = world.dt();
    const int max_ticks = static_cast<int>((expected_period * 4.5) / dt);
    for (int i = 0; i < max_ticks && crossing_times.size() < 5; ++i) {
        world.step();
        t += dt;
        const double w = world.joint_velocity(art, 0);
        if (i > 0 && prev_w * w < 0.0) {
            const double frac = -prev_w / (w - prev_w);
            crossing_times.push_back((t - dt) + frac * dt);
        }
        prev_w = w;
    }
    REQUIRE(crossing_times.size() >= 3);
    return (crossing_times[1] - crossing_times[0]) + (crossing_times[2] - crossing_times[1]);
}

void run_case(double theta0_deg) {
    const double g = 9.81;
    const double L = 1.5;
    const double m = 2.0;
    const double r = 0.005; // tiny, so I_cm is negligible next to m*L^2 (point-mass limit)
    const double theta0 = theta0_deg * kPi / 180.0;

    const double I_cm = 0.4 * m * r * r;
    const double I_pivot = I_cm + m * L * L;
    const double omega0_sq_equiv = m * g * L / I_pivot; // == g/L in the point-mass limit
    const double expected_period = 4.0 * std::sqrt(1.0 / omega0_sq_equiv) * complete_elliptic_K(std::sin(theta0 / 2.0));

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -g};
    World world(cfg);

    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose::identity();

    LinkDesc link;
    link.parent = -1;
    link.joint = articulation::JointType::Revolute;
    link.axis = Vec3::unit_y();
    link.mass = m;
    link.com = Vec3{0, 0, -L};
    link.inertia = Mat3::diagonal(I_cm, I_cm, I_cm);
    link.q0 = theta0;
    desc.links.push_back(link);

    const ArticulationId art = world.create_articulation(desc);
    const double measured = measure_period(world, art, expected_period);
    const double rel_err = std::abs(measured - expected_period) / expected_period;
    INFO("theta0_deg=" << theta0_deg << " expected=" << expected_period << " measured=" << measured << " rel_err=" << rel_err);
    REQUIRE(rel_err < 1e-3);
}

} // namespace

TEST_CASE("Articulation single-pendulum period matches the elliptic-integral formula at 90 deg", "[validation][articulation][pendulum]") {
    run_case(90.0);
}

TEST_CASE("Articulation single-pendulum period matches the elliptic-integral formula at 170 deg", "[validation][articulation][pendulum]") {
    run_case(170.0);
}
