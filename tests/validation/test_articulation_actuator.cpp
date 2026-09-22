// Validation: a force-limited prismatic velocity actuator stalls at
// exactly its force limit against a load that exceeds it (PLAN.md P1.5a
// acceptance) - the "hydraulic cylinder preview" of
// ps::articulation::VelocityActuator.
//
// Vertical prismatic joint, link hanging under gravity, actuator commands
// target_speed = 0 (trying to hold position) with max_force = k*m*g,
// k < 1 (weaker than the load it must hold). Once the link starts falling
// (qd < 0, immediately - gravity acts from t=0 regardless of the
// actuator), the actuator's error (target_speed - qd) is permanently
// positive, so instance.cpp's compute_tau() saturates at exactly
// +max_force every substep (see its comment: unconditional saturation,
// not a proportional controller approached asymptotically). Net joint
// force is then the constant (max_force - m*g), giving an EXACTLY solvable
// constant-acceleration fall: qdd = max_force/m - g, qd(t) = qdd*t,
// q(t) = q0 + 0.5*qdd*t^2 - a closed form this test checks the simulation
// against directly, which is a stronger, more specific claim than just
// "does not exceed max_force".

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ps;
using namespace ps::articulation;

TEST_CASE("Force-limited prismatic actuator stalls at exactly the limit force", "[validation][articulation][actuator]") {
    const double g = 9.81;
    const double m = 10.0;
    const double k = 0.4; // actuator can supply only 40% of the weight
    const double max_force = k * m * g;

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
    link.joint = articulation::JointType::Prismatic;
    link.axis = Vec3::unit_z();
    link.mass = m;
    link.com = Vec3::zero();
    link.inertia = Mat3::diagonal(0.01, 0.01, 0.01);
    link.velocity_actuator.enabled = true;
    link.velocity_actuator.target_speed = 0.0; // trying to hold position
    link.velocity_actuator.max_force = max_force;
    desc.links.push_back(link);

    const ArticulationId art = world.create_articulation(desc);

    const double expected_qdd = max_force / m - g; // constant, exact
    const double duration_s = 2.0;
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t t = 0; t < num_ticks; ++t) world.step();

    const double expected_qd = expected_qdd * duration_s;
    const double expected_q = 0.5 * expected_qdd * duration_s * duration_s;
    const double measured_qd = world.joint_velocity(art, 0);
    const double measured_q = world.joint_position(art, 0);

    INFO("expected_qdd=" << expected_qdd << " expected_qd=" << expected_qd << " measured_qd=" << measured_qd
                          << " expected_q=" << expected_q << " measured_q=" << measured_q);
    REQUIRE(std::isfinite(measured_qd));
    CHECK(std::abs(measured_qd - expected_qd) / std::abs(expected_qd) < 1e-3);
    CHECK(std::abs(measured_q - expected_q) / std::abs(expected_q) < 1e-3);
}

TEST_CASE("Force-limited prismatic actuator holds position when strong enough", "[validation][articulation][actuator]") {
    const double g = 9.81;
    const double m = 10.0;
    const double k = 2.0; // actuator can supply 2x the weight - easily holds
    const double max_force = k * m * g;

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -g};
    World world(cfg);

    ArticulationDesc desc;
    desc.root.floating = false;

    LinkDesc link;
    link.parent = -1;
    link.joint = articulation::JointType::Prismatic;
    link.axis = Vec3::unit_z();
    link.mass = m;
    link.inertia = Mat3::diagonal(0.01, 0.01, 0.01);
    link.velocity_actuator.enabled = true;
    link.velocity_actuator.target_speed = 0.0;
    link.velocity_actuator.max_force = max_force;
    desc.links.push_back(link);

    const ArticulationId art = world.create_articulation(desc);

    const std::uint64_t num_ticks = static_cast<std::uint64_t>(2.0 * cfg.tick_rate_hz + 0.5);
    double max_abs_q = 0.0;
    for (std::uint64_t t = 0; t < num_ticks; ++t) {
        world.step();
        max_abs_q = std::max(max_abs_q, std::abs(world.joint_position(art, 0)));
    }
    // A strong-enough bang-bang actuator chatters around the target but
    // never drifts far - well short of the free-fall distance
    // (0.5*g*(2s)^2 ~ 19.6 m) an unpowered load would have covered.
    INFO("max_abs_q=" << max_abs_q);
    CHECK(max_abs_q < 0.5);
}
