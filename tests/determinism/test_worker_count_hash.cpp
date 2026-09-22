// Determinism: state hash identical with 1, 2, 4, 10, 24 workers, and
// under a randomised-scheduling stress test (PLAN.md P1 acceptance / D7/D8).
//
// The scenario mixes gravity (Jolt's own internal per-body parallel work,
// via the JPH::JobSystem bridge) with a batch of CounterForceElement
// instances evaluated through World's own parallel_for force-element stage
// and gathered in fixed id order (PLAN.md D8: scatter -> compute -> ordered
// gather) - so this test exercises determinism of both the Jolt bridge and
// our own job-scheduled code, not just one of them.

#include "counter_force_element.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <random>
#include <vector>

using namespace ps;
using ps_test::CounterForceElement;

namespace {

std::uint64_t run_scenario(int workers, std::size_t num_bodies, std::size_t grain, int ticks) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = workers;
    cfg.force_element_grain = grain;
    World world(cfg);

    std::vector<BodyId> bodies;
    std::vector<std::unique_ptr<CounterForceElement>> elements;
    bodies.reserve(num_bodies);
    elements.reserve(num_bodies);

    for (std::size_t i = 0; i < num_bodies; ++i) {
        BodyDesc desc;
        desc.shape = SphereShape{0.3};
        desc.motion = BodyMotionType::Dynamic;
        desc.mass = 1.0;
        desc.pose.position = Vec3{static_cast<double>(i) * 1.5, 0.0, 20.0 + static_cast<double>(i % 5)};
        desc.gravity_enabled = true;
        desc.allow_sleep = false;
        const BodyId id = world.create_body(desc);
        bodies.push_back(id);
        elements.push_back(std::make_unique<CounterForceElement>(Frame(id), static_cast<std::uint64_t>(i), 3.0));
        world.add_force_element(elements.back().get());
    }

    for (int t = 0; t < ticks; ++t) world.step();
    return world.state_hash();
}

} // namespace

TEST_CASE("State hash is identical across 1, 2, 4, 10 and 24 workers", "[determinism][heavy]") {
    constexpr std::size_t kBodies = 64;
    constexpr int kTicks = 40;

    const std::uint64_t reference = run_scenario(1, kBodies, 0, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario(workers, kBodies, 0, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

namespace {

// PLAN.md P1.5a/D8, PHYS-009 Step 1 rule 3: "add articulations to the
// thread-count determinism test (1/2/4/10/24 workers, several
// articulations at once)". Several DIFFERENT-SIZED open chains (1..5
// links), a floating-base chain, and a four-bar (loop-closure) linkage -
// mixed with the same CounterForceElement Jolt bodies run_scenario above
// already uses, so Jolt/force-element/articulation stages are all
// exercised together in one hash. Unlike the original version of this
// test, System::step_all (core/src/articulation/system.cpp) now runs one
// job per articulation batch through ps::jobs::parallel_for - this test's
// job is therefore no longer trivially true by construction: it actually
// exercises articulation-stepping determinism under work-stealing
// scheduling, in addition to checking that sharing a World/pipeline with
// the worker-count-sensitive Jolt/force-element stages doesn't disturb
// either side's determinism.
using namespace ps::articulation;

ArticulationDesc make_four_bar_desc(double x_offset, double q_crank) {
    // Same Grashof crank-rocker geometry as
    // tests/validation/test_articulation_four_bar_energy.cpp, offset in x
    // so several instances don't overlap in space (irrelevant to the
    // dynamics - articulations don't collide, PLAN.md P1.5a scope - but
    // keeps scenario construction easy to reason about).
    constexpr double kA = 2.0, kB = 5.0, kC = 4.0, kD = 4.0;
    ArticulationDesc desc;
    desc.root.floating = false;
    desc.root.pose = Pose{Vec3{x_offset, 0, 0}, Quat::identity()};

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
    coupler.q0 = 1.0;
    desc.links.push_back(coupler);

    LinkDesc rocker;
    rocker.parent = -1;
    rocker.joint = articulation::JointType::Revolute;
    rocker.joint_frame_in_parent.position = Vec3{kD, 0, 0};
    rocker.axis = Vec3::unit_y();
    rocker.mass = 1.0;
    rocker.com = Vec3{0, 0, -kC / 2.0};
    rocker.inertia = Mat3::diagonal(1.0 * kC * kC / 12.0, 1.0 * kC * kC / 12.0, 1e-4);
    rocker.q0 = 1.8;
    desc.links.push_back(rocker);

    LoopClosureDesc lc;
    lc.type = LoopClosureType::Point;
    lc.link_a = 1;
    lc.anchor_a = Vec3{0, 0, -kB};
    lc.link_b = 2;
    lc.anchor_b = Vec3{0, 0, -kC};
    desc.loop_closures.push_back(lc);
    return desc;
}

std::uint64_t run_articulation_scenario(int workers, int ticks) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = workers;
    World world(cfg);

    // Open chains of DIFFERENT sizes (1..5 links), so the parallel_for
    // batches mix small and large instances across jobs.
    constexpr int kNumChains = 8;
    for (int a = 0; a < kNumChains; ++a) {
        const int links = 1 + (a % 5);
        articulation::ArticulationDesc desc;
        desc.root.floating = false;
        desc.root.pose = Pose{Vec3{static_cast<double>(a) * 3.0, 0.0, 0.0}, Quat::identity()};
        for (int i = 0; i < links; ++i) {
            articulation::LinkDesc link;
            link.parent = (i == 0) ? -1 : (i - 1);
            link.joint = articulation::JointType::Revolute;
            link.axis = Vec3::unit_y();
            if (i > 0) link.joint_frame_in_parent.position = Vec3{0, 0, -0.6};
            link.mass = 1.0;
            link.com = Vec3{0, 0, -0.3};
            link.inertia = Mat3::diagonal(0.02, 0.02, 0.02);
            link.q0 = 0.3 + 0.05 * static_cast<double>(a) + 0.02 * static_cast<double>(i);
            desc.links.push_back(link);
        }
        world.create_articulation(desc);
    }

    // A floating-base chain (PHYS-009 Step 1 rule 3).
    {
        articulation::ArticulationDesc desc;
        desc.root.floating = true;
        desc.root.pose = Pose{Vec3{0.0, 6.0, 5.0}, Quat::identity()};
        desc.root.velocity.angular = Vec3{0.1, 0.4, -0.2};
        desc.root.velocity.linear = Vec3{0.0, 0.0, 0.5};
        desc.root.mass = 2.0;
        desc.root.com = Vec3::zero();
        desc.root.inertia = Mat3::diagonal(0.08, 0.08, 0.08);
        for (int i = 0; i < 2; ++i) {
            articulation::LinkDesc link;
            link.parent = i - 1;
            link.joint = articulation::JointType::Revolute;
            link.axis = Vec3::unit_x();
            if (i > 0) link.joint_frame_in_parent.position = Vec3{0, 0, -0.5};
            link.mass = 0.5;
            link.com = Vec3{0, 0, -0.25};
            link.inertia = Mat3::diagonal(0.01, 0.01, 0.01);
            link.q0 = 0.1 * static_cast<double>(i + 1);
            desc.links.push_back(link);
        }
        world.create_articulation(desc);
    }

    // A four-bar (loop-closure) linkage (PHYS-009 Step 1 rule 3).
    world.create_articulation(make_four_bar_desc(30.0, 0.6));

    for (int t = 0; t < ticks; ++t) world.step();
    return world.state_hash();
}

} // namespace

TEST_CASE("State hash with several articulations is identical across 1, 2, 4, 10 and 24 workers", "[determinism][articulation][heavy]") {
    constexpr int kTicks = 100;
    const std::uint64_t reference = run_articulation_scenario(1, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_articulation_scenario(workers, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

TEST_CASE("State hash survives 1000 runs of randomised worker count and scheduling grain", "[determinism][stress]") {
    constexpr std::size_t kBodies = 8;
    constexpr int kTicks = 10;

    const std::uint64_t reference = run_scenario(1, kBodies, 0, kTicks);

    std::mt19937 rng(12345); // seeds only the TEST's choice of worker count/grain, never the simulation itself
    std::uniform_int_distribution<int> worker_dist(1, 8);
    std::uniform_int_distribution<int> grain_dist(0, 5); // 0 == World's own auto grain

    for (int run = 0; run < 1000; ++run) {
        const int workers = worker_dist(rng);
        const std::size_t grain = static_cast<std::size_t>(grain_dist(rng));
        const std::uint64_t h = run_scenario(workers, kBodies, grain, kTicks);
        if (h != reference) {
            INFO("run=" << run << " workers=" << workers << " grain=" << grain << " hash=0x" << std::hex << h);
            REQUIRE(h == reference);
        }
    }
}
