// Validation: a floating-base articulation with no gravity conserves
// linear and angular momentum to 1e-6 relative over 60 s (PLAN.md P1.5a
// acceptance) - a spinning, flexing two-link body (floating root + one
// link swinging on a frictionless, unactuated joint). With no external
// force (gravity off, no actuation, no damping) every joint torque is
// purely internal, so total system momentum (about a fixed world point)
// must be exactly conserved regardless of how the joint swings - this is
// a strong, representation-independent physics check quite different from
// the energy-drift tests (a broken external-force sign, a broken Xup
// direction, or an asymmetric mass-matrix bug would all typically show up
// here even if they happened to conserve energy).

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace ps;
using namespace ps::articulation;

namespace {

struct Momentum {
    Vec3 linear;
    Vec3 angular; // about the world origin
};

Momentum total_momentum(World& world, ArticulationId art, real root_mass, const Vec3& root_com, const Mat3& root_i_cm,
                         real link_mass, real link_len, const Mat3& link_i_cm) {
    Momentum out;

    {
        const Frame com_frame = world.link_frame(art, -1, Pose{root_com, Quat::identity()});
        const Pose p = com_frame.world_pose(world.backend());
        const Motion mo = com_frame.world_motion(world.backend());
        const Pose root_pose = world.link_frame(art, -1).world_pose(world.backend());
        const Mat3 r = root_pose.orientation.to_mat3();
        const Mat3 i_world = r * root_i_cm * r.transposed();
        out.linear += mo.linear * root_mass;
        out.angular += i_world * mo.angular + cross(p.position, mo.linear * root_mass);
    }
    {
        const Frame com_frame = world.link_frame(art, 0, Pose{Vec3{0, 0, -link_len}, Quat::identity()});
        const Pose p = com_frame.world_pose(world.backend());
        const Motion mo = com_frame.world_motion(world.backend());
        const Pose link_pose = world.link_frame(art, 0).world_pose(world.backend());
        const Mat3 r = link_pose.orientation.to_mat3();
        const Mat3 i_world = r * link_i_cm * r.transposed();
        out.linear += mo.linear * link_mass;
        out.angular += i_world * mo.angular + cross(p.position, mo.linear * link_mass);
    }
    return out;
}

} // namespace

TEST_CASE("Floating-base articulation without gravity conserves momentum over 60 s", "[validation][articulation][floating_base]") {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3::zero(); // no gravity: every remaining torque is internal
    World world(cfg);

    const real root_mass = 5.0;
    const Vec3 root_com{0.1, 0.05, -0.2};
    const Mat3 root_i_cm = Mat3::diagonal(0.3, 0.4, 0.5);
    const real link_mass = 1.5;
    const real link_len = 0.6;
    const Mat3 link_i_cm = Mat3::diagonal(0.02, 0.02, 0.02);

    ArticulationDesc desc;
    desc.root.floating = true;
    desc.root.pose = Pose::identity();
    desc.root.mass = root_mass;
    desc.root.com = root_com;
    desc.root.inertia = root_i_cm;
    desc.root.velocity = Motion{Vec3{0.3, -0.2, 0.1}, Vec3{0.5, 0.8, -0.4}}; // spinning and translating

    LinkDesc link;
    link.parent = -1; // attaches to the floating root
    link.joint = articulation::JointType::Revolute;
    link.joint_frame_in_parent.position = Vec3{0.2, 0, 0};
    link.axis = Vec3::unit_z();
    link.mass = link_mass;
    link.com = Vec3{0, 0, -link_len};
    link.inertia = link_i_cm;
    link.q0 = 0.3;
    link.qd0 = 1.2; // flexing
    desc.links.push_back(link);

    const ArticulationId art = world.create_articulation(desc);

    const Momentum p0 = total_momentum(world, art, root_mass, root_com, root_i_cm, link_mass, link_len, link_i_cm);

    const std::uint64_t num_ticks = static_cast<std::uint64_t>(60.0 * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t t = 0; t < num_ticks; ++t) world.step();

    const Momentum p1 = total_momentum(world, art, root_mass, root_com, root_i_cm, link_mass, link_len, link_i_cm);

    const real lin_scale = std::max(real(1e-9), p0.linear.length());
    const real ang_scale = std::max(real(1e-9), p0.angular.length());
    const real lin_drift = (p1.linear - p0.linear).length() / lin_scale;
    const real ang_drift = (p1.angular - p0.angular).length() / ang_scale;

    INFO("lin0=(" << p0.linear.x << "," << p0.linear.y << "," << p0.linear.z << ") lin1=(" << p1.linear.x << "," << p1.linear.y << "," << p1.linear.z << ") lin_drift=" << lin_drift);
    INFO("ang0=(" << p0.angular.x << "," << p0.angular.y << "," << p0.angular.z << ") ang1=(" << p1.angular.x << "," << p1.angular.y << "," << p1.angular.z << ") ang_drift=" << ang_drift);
    CHECK(lin_drift < 1e-6);
    CHECK(ang_drift < 1e-6);
}
