// Validation: 2000:1 mass ratio articulation chain, both orders (PLAN.md
// P1.5a acceptance) - the reduced-coordinate counterpart of
// tests/validation/test_mass_ratio.cpp's [!mayfail] Jolt joint case (which
// stays as documentation of the problem D10 exists to fix). In joint
// coordinates the joint cannot come apart at all (there is no maximal-
// coordinate position error to project), so "joint separation zero by
// construction" is not a numerical outcome to measure here - it is a
// structural property of the representation - what this test actually
// checks is that the mass ratio does not blow up the ABA solve (its D
// matrix inversion, spatial.h/aba.cpp) or the integrator.

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace ps;
using namespace ps::articulation;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

struct LinkSpec {
    double mass;
    double length;
};

struct RunResult {
    double energy_start = 0.0;
    double energy_end = 0.0;
    double max_qd = 0.0;
    bool finite = true;
};

RunResult run_chain(const std::vector<LinkSpec>& links, double duration_s = 60.0) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -kG};
    World world(cfg);

    ArticulationDesc desc;
    desc.root.floating = false;
    std::vector<Mat3> i_cm_local;
    const double theta0 = 30.0 * kPi / 180.0;
    for (std::size_t i = 0; i < links.size(); ++i) {
        LinkDesc ld;
        ld.parent = (i == 0) ? -1 : static_cast<int>(i) - 1;
        ld.joint = articulation::JointType::Revolute;
        ld.axis = Vec3::unit_y();
        if (i > 0) ld.joint_frame_in_parent.position = Vec3{0, 0, -links[i - 1].length};
        ld.mass = links[i].mass;
        ld.com = Vec3{0, 0, -links[i].length};
        const double r = 0.1;
        const Mat3 i_cm = Mat3::diagonal(0.4 * links[i].mass * r * r, 0.4 * links[i].mass * r * r, 0.4 * links[i].mass * r * r);
        ld.inertia = i_cm;
        ld.q0 = (i == 0) ? theta0 : 0.0;
        desc.links.push_back(ld);
        i_cm_local.push_back(i_cm);
    }
    const ArticulationId art = world.create_articulation(desc);

    auto total_energy = [&]() {
        double e = 0.0;
        for (std::size_t i = 0; i < links.size(); ++i) {
            const Frame com_frame = world.link_frame(art, static_cast<int>(i), Pose{Vec3{0, 0, -links[i].length}, Quat::identity()});
            const Pose com_pose = com_frame.world_pose(world.backend());
            const Motion com_motion = com_frame.world_motion(world.backend());
            const Pose link_pose = world.link_frame(art, static_cast<int>(i)).world_pose(world.backend());
            const Mat3 r = link_pose.orientation.to_mat3();
            const Mat3 i_world = r * i_cm_local[i] * r.transposed();
            const double ke_lin = 0.5 * links[i].mass * com_motion.linear.length_squared();
            const double ke_ang = 0.5 * dot(com_motion.angular, i_world * com_motion.angular);
            e += ke_lin + ke_ang + links[i].mass * kG * com_pose.position.z;
        }
        return e;
    };

    RunResult result;
    result.energy_start = total_energy();
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(duration_s * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t t = 0; t < num_ticks; ++t) {
        world.step();
        for (std::size_t i = 0; i < links.size(); ++i) {
            const double qd = world.joint_velocity(art, static_cast<int>(i));
            if (!std::isfinite(qd)) result.finite = false;
            result.max_qd = std::max(result.max_qd, std::abs(qd));
        }
    }
    result.energy_end = total_energy();
    return result;
}

} // namespace

TEST_CASE("Articulation: 20t link carried by a 10kg link stays stable (heavy at the far end)", "[validation][articulation][mass_ratio]") {
    const std::vector<LinkSpec> links = {LinkSpec{10.0, 0.35}, LinkSpec{20000.0, 1.0}};
    const RunResult r = run_chain(links);
    INFO("energy_start=" << r.energy_start << " energy_end=" << r.energy_end << " max_qd=" << r.max_qd << " finite=" << r.finite);
    REQUIRE(r.finite);
    CHECK(std::abs(r.energy_end - r.energy_start) < 0.005 * std::abs(r.energy_start));
}

TEST_CASE("Articulation: 10kg link carried by a 20t link stays stable (heavy near the pivot)", "[validation][articulation][mass_ratio]") {
    const std::vector<LinkSpec> links = {LinkSpec{20000.0, 1.0}, LinkSpec{10.0, 0.35}};
    const RunResult r = run_chain(links);
    INFO("energy_start=" << r.energy_start << " energy_end=" << r.energy_end << " max_qd=" << r.max_qd << " finite=" << r.finite);
    REQUIRE(r.finite);
    CHECK(std::abs(r.energy_end - r.energy_start) < 0.005 * std::abs(r.energy_start));
}
