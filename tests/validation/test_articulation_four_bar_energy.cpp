// Validation: four-bar linkage (Grashof crank-rocker) energy drift < 0.5%
// over 60 s, closure error < 0.1 mm throughout (PLAN.md P1.5a acceptance;
// see tests/unit/test_articulation_four_bar.cpp for the companion
// closed-form kinematic check of the same geometry).
//
// Released from a consistent (closure-satisfying) configuration under
// gravity and left to swing freely - no actuation, no damping - so total
// mechanical energy of the three links should be conserved exactly the
// same way the open-chain energy tests are, with the loop closure held
// throughout by loop_closure.cpp's acceleration-level solve plus periodic
// drift stabilisation (project_onto_constraints, once per tick).

#include "articulation/loop_closure.h"
#include "articulation/model.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace ps;
using namespace ps::articulation;

namespace {

constexpr double kA = 2.0, kB = 5.0, kC = 4.0, kD = 4.0;

ArticulationDesc make_desc(double q_crank, double q_coupler, double q_rocker) {
    ArticulationDesc desc;
    desc.root.floating = false;

    LinkDesc crank;
    crank.parent = -1;
    crank.joint = articulation::JointType::Revolute;
    crank.axis = Vec3::unit_y();
    crank.mass = 1.0;
    crank.com = Vec3{0, 0, -kA / 2.0};
    crank.inertia = Mat3::diagonal(1.0 * kA * kA / 12.0, 1.0 * kA * kA / 12.0, 1e-4);
    crank.q0 = q_crank;
    desc.links.push_back(crank);

    LinkDesc coupler;
    coupler.parent = 0;
    coupler.joint = articulation::JointType::Revolute;
    coupler.joint_frame_in_parent.position = Vec3{0, 0, -kA};
    coupler.axis = Vec3::unit_y();
    coupler.mass = 1.5;
    coupler.com = Vec3{0, 0, -kB / 2.0};
    coupler.inertia = Mat3::diagonal(1.5 * kB * kB / 12.0, 1.5 * kB * kB / 12.0, 1e-4);
    coupler.q0 = q_coupler;
    desc.links.push_back(coupler);

    LinkDesc rocker;
    rocker.parent = -1;
    rocker.joint = articulation::JointType::Revolute;
    rocker.joint_frame_in_parent.position = Vec3{kD, 0, 0};
    rocker.axis = Vec3::unit_y();
    rocker.mass = 1.0;
    rocker.com = Vec3{0, 0, -kC / 2.0};
    rocker.inertia = Mat3::diagonal(1.0 * kC * kC / 12.0, 1.0 * kC * kC / 12.0, 1e-4);
    rocker.q0 = q_rocker;
    desc.links.push_back(rocker);

    LoopClosureDesc lc;
    lc.type = articulation::LoopClosureType::Point;
    lc.link_a = 1;
    lc.anchor_a = Vec3{0, 0, -kB};
    lc.link_b = 2;
    lc.anchor_b = Vec3{0, 0, -kC};
    desc.loop_closures.push_back(lc);

    return desc;
}

double total_energy(World& world, ArticulationId art, double g) {
    struct LinkInfo { int idx; double mass, half_len; Mat3 i_cm; };
    const Mat3 ia = Mat3::diagonal(1.0 * kA * kA / 12.0, 1.0 * kA * kA / 12.0, 1e-4);
    const Mat3 ib = Mat3::diagonal(1.5 * kB * kB / 12.0, 1.5 * kB * kB / 12.0, 1e-4);
    const Mat3 ic = Mat3::diagonal(1.0 * kC * kC / 12.0, 1.0 * kC * kC / 12.0, 1e-4);
    const LinkInfo links[3] = {{0, 1.0, kA / 2.0, ia}, {1, 1.5, kB / 2.0, ib}, {2, 1.0, kC / 2.0, ic}};

    double e = 0.0;
    for (const auto& li : links) {
        const Frame com_frame = world.link_frame(art, li.idx, Pose{Vec3{0, 0, -li.half_len}, Quat::identity()});
        const Pose com_pose = com_frame.world_pose(world.backend());
        const Motion com_motion = com_frame.world_motion(world.backend());
        const Pose link_pose = world.link_frame(art, li.idx).world_pose(world.backend());
        const Mat3 r = link_pose.orientation.to_mat3();
        const Mat3 i_world = r * li.i_cm * r.transposed();
        e += 0.5 * li.mass * com_motion.linear.length_squared();
        e += 0.5 * dot(com_motion.angular, i_world * com_motion.angular);
        e += li.mass * g * com_pose.position.z;
    }
    return e;
}

} // namespace

TEST_CASE("Four-bar linkage conserves energy and holds closure over 60 s", "[validation][articulation][four_bar]") {
    const double g = 9.81;

    // Find a closure-consistent initial configuration at crank angle 0.4 rad
    // by the same Newton projection the unit test uses, starting from a
    // reasonable folded guess.
    ArticulationDesc seed_desc = make_desc(0.4, -1.2, 2.6);
    Model seed_model = build_model(seed_desc);
    ArticulationState seed_state;
    seed_state.q = {0.4, -1.2, 2.6};
    seed_state.qd.assign(3, 0.0);
    std::vector<int> link_to_body = {0, 1, 2};
    const std::vector<LoopClosureRuntime> closures = build_loop_closure_runtime(seed_model, link_to_body);
    for (int i = 0; i < 60; ++i) project_onto_constraints(seed_model, seed_state, closures);
    const std::vector<real> g_err = constraint_error(seed_model, seed_state, closures);
    REQUIRE(Vec3{g_err[0], g_err[1], g_err[2]}.length() < 1e-9);

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -g};
    World world(cfg);

    const ArticulationDesc desc = make_desc(seed_state.q[0], seed_state.q[1], seed_state.q[2]);
    const ArticulationId art = world.create_articulation(desc);

    const double e_start = total_energy(world, art, g);
    double max_closure_mm = 0.0;
    const std::uint64_t num_ticks = static_cast<std::uint64_t>(60.0 * cfg.tick_rate_hz + 0.5);
    for (std::uint64_t t = 0; t < num_ticks; ++t) {
        world.step();
        const Vec3 pa = world.link_frame(art, 1, Pose{Vec3{0, 0, -kB}, Quat::identity()}).world_pose(world.backend()).position;
        const Vec3 pb = world.link_frame(art, 2, Pose{Vec3{0, 0, -kC}, Quat::identity()}).world_pose(world.backend()).position;
        max_closure_mm = std::max(max_closure_mm, (pa - pb).length() * 1000.0);
    }
    const double e_end = total_energy(world, art, g);

    const double rel_drift = std::abs(e_end - e_start) / std::abs(e_start);
    INFO("e_start=" << e_start << " e_end=" << e_end << " rel_drift=" << rel_drift << " max_closure_mm=" << max_closure_mm);
    CHECK(rel_drift < 0.005);
    CHECK(max_closure_mm < 0.1);
}
