// Validation: articulation energy drift < 0.5% over 60 s without damping
// (PLAN.md P1.5a acceptance): single link at 90 and 170 deg, and a 3-link
// chain at 90 deg (the articulation counterpart of
// tests/validation/test_joint_chain_energy.cpp). Unlike the Jolt joint
// chain, there is no Baumgarte/position-projection energy leak here by
// construction (PLAN.md D10) - the semi-implicit-Euler integrator itself
// still has bounded, non-systematic energy oscillation, which this test
// bounds the same way the Jolt version does.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace ps;
using namespace ps::articulation;

namespace {
constexpr double kPi = 3.14159265358979323846;

struct LinkSpec {
    double mass;
    double length;   // pivot-to-COM (and, for a chain, pivot-to-next-pivot) distance
    double q0;
};

// Mass+inertia+pose bookkeeping mirrors what World actually simulates -
// kept here (not queried back from World, which has no public mass/inertia
// accessor for articulation links) since the test authors these values
// itself.
struct LinkRuntime {
    double mass;
    double length;
    Mat3 i_cm_local; // small but nonzero, so this is a real rigid link, not a pure point mass
};

ArticulationId build_chain(World& world, const std::vector<LinkSpec>& links, std::vector<LinkRuntime>& runtime_out) {
    ArticulationDesc desc;
    desc.root.floating = false;
    double cumulative = 0.0;
    for (std::size_t i = 0; i < links.size(); ++i) {
        LinkDesc ld;
        ld.parent = (i == 0) ? -1 : static_cast<int>(i) - 1;
        ld.joint = articulation::JointType::Revolute;
        ld.axis = Vec3::unit_y();
        if (i > 0) ld.joint_frame_in_parent.position = Vec3{0, 0, -links[i - 1].length};
        ld.mass = links[i].mass;
        ld.com = Vec3{0, 0, -links[i].length};
        const double r = 0.05;
        const Mat3 i_cm = Mat3::diagonal(0.4 * links[i].mass * r * r, 0.4 * links[i].mass * r * r, 0.4 * links[i].mass * r * r);
        ld.inertia = i_cm;
        ld.q0 = links[i].q0;
        desc.links.push_back(ld);
        runtime_out.push_back(LinkRuntime{links[i].mass, links[i].length, i_cm});
        cumulative += links[i].length;
    }
    (void)cumulative;
    return world.create_articulation(desc);
}

double total_energy(World& world, ArticulationId art, const std::vector<LinkRuntime>& runtime, double g, double z_ref) {
    double e = 0.0;
    for (std::size_t i = 0; i < runtime.size(); ++i) {
        const Frame f = world.link_frame(art, static_cast<int>(i));
        const Pose p = f.world_pose(world.backend());
        const Motion mo = f.world_motion(world.backend());
        // COM pose/velocity: link frame origin is the joint/pivot; the COM
        // sits at local offset (0,0,-length) - use Frame's own local-offset
        // machinery (identical math to a body-anchored Frame with an
        // offset) rather than recomputing it by hand.
        const Frame com_frame = world.link_frame(art, static_cast<int>(i), Pose{Vec3{0, 0, -runtime[i].length}, Quat::identity()});
        const Pose com_pose = com_frame.world_pose(world.backend());
        const Motion com_motion = com_frame.world_motion(world.backend());

        const Mat3 r = p.orientation.to_mat3();
        const Mat3 i_world = r * runtime[i].i_cm_local * r.transposed();
        const double ke_lin = 0.5 * runtime[i].mass * com_motion.linear.length_squared();
        const Vec3 iw = i_world * mo.angular; // angular velocity is the same for every point on a rigid body
        const double ke_ang = 0.5 * dot(mo.angular, iw);
        const double pe = runtime[i].mass * g * (com_pose.position.z - z_ref);
        e += ke_lin + ke_ang + pe;
    }
    return e;
}

void run_energy_case(const std::vector<LinkSpec>& links, double duration_s) {
    const double g = 9.81;
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -g};
    World world(cfg);

    std::vector<LinkRuntime> runtime;
    const ArticulationId art = build_chain(world, links, runtime);

    double total_len = 0.0;
    for (const auto& l : links) total_len += l.length;
    const double z_ref = -(total_len + 2.0);

    const double e_start = total_energy(world, art, runtime, g, z_ref);
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t i = 0; i < num_ticks; ++i) world.step();
    const double e_end = total_energy(world, art, runtime, g, z_ref);

    const double rel_drift = std::abs(e_end - e_start) / std::abs(e_start);
    INFO("links=" << links.size() << " e_start=" << e_start << " e_end=" << e_end << " rel_drift=" << rel_drift);
    REQUIRE(rel_drift < 0.005);
}

} // namespace

TEST_CASE("Articulation single link at 90 deg conserves energy within 0.5% over 60 s", "[validation][articulation][energy][long]") {
    run_energy_case({LinkSpec{1.0, 1.0, 90.0 * kPi / 180.0}}, 60.0);
}

TEST_CASE("Articulation single link at 170 deg conserves energy within 0.5% over 60 s", "[validation][articulation][energy][long]") {
    run_energy_case({LinkSpec{1.0, 1.0, 170.0 * kPi / 180.0}}, 60.0);
}

TEST_CASE("Articulation 3-link chain at 90 deg conserves energy within 0.5% over 60 s", "[validation][articulation][energy][long]") {
    run_energy_case({
        LinkSpec{1.0, 0.6, 90.0 * kPi / 180.0},
        LinkSpec{1.0, 0.6, 0.0},
        LinkSpec{1.0, 0.6, 0.0},
    }, 60.0);
}
