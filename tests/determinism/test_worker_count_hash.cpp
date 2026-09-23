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
#include "ps/io/scenario.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <random>
#include <string>
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

// PLAN.md P1.5b measurement (g): System::step_all steps any contact-source-
// bearing instance (Compliant only - contact_source.h) SEQUENTIALLY FIRST,
// in fixed creation order, before the ordinary parallel_for over everything
// else (system.cpp's own comment on why - apply_wrench to the SAME other
// body from two different worker threads at once would race). That changes
// this codebase's usual "one job per articulation" execution order relative
// to every other determinism scenario in this file, so it needs its own
// check: several Compliant AND ProxyBody contact-bearing rail rigs (offset
// in x so they never touch each other or share a Jolt body, and dropped
// from different heights so they are at different points of falling/first-
// contact/settling by kTicks, exercising the actual contact code paths, not
// just steady state) mixed with a few plain (contact_strategy == None)
// chains so the parallel_for batch still has non-contact work to interleave
// with the sequential contact pass.
std::uint64_t run_contact_scenario(int workers, int ticks) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.gravity = Vec3{0, 0, -9.81};
    cfg.job_workers = workers;
    World world(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{50, 5, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 0.8;
    ground.pose.position = Vec3{0, 0, -0.5};
    world.create_body(ground);

    auto make_rail = [](articulation::ContactStrategy strategy, real x, real root_z) {
        articulation::ArticulationDesc desc;
        desc.root.floating = false;
        desc.root.pose = Pose{Vec3{x, 0, root_z}, Quat::identity()};
        desc.contact_strategy = strategy;

        articulation::LinkDesc link;
        link.parent = -1;
        link.joint = articulation::JointType::Prismatic;
        link.axis = Vec3::unit_z();
        link.mass = 1.0;
        link.com = Vec3::zero();
        link.inertia = Mat3::diagonal(0.01, 0.01, 0.01);
        link.contact.enabled = (strategy != articulation::ContactStrategy::None);
        link.contact.shape = SphereShape{0.3};
        link.contact.material.stiffness = 1.0e6;
        link.contact.material.stiffness_exponent = 1.5;
        link.contact.material.damping = 1.0e6;
        link.contact.material.friction = 0.8;
        desc.links.push_back(link);
        return desc;
    };

    for (int a = 0; a < 3; ++a) {
        world.create_articulation(make_rail(articulation::ContactStrategy::Compliant, static_cast<real>(a) * 3.0, 0.6 + 0.1 * static_cast<real>(a)));
    }
    for (int a = 0; a < 3; ++a) {
        world.create_articulation(
            make_rail(articulation::ContactStrategy::ProxyBody, 20.0 + static_cast<real>(a) * 3.0, 0.6 + 0.1 * static_cast<real>(a)));
    }
    // A few plain (non-contact) open chains mixed in, same shape as
    // run_articulation_scenario above, so parallel_for's batch has ordinary
    // work alongside the sequential contact pass.
    for (int a = 0; a < 4; ++a) {
        const int links = 1 + (a % 3);
        articulation::ArticulationDesc desc;
        desc.root.floating = false;
        desc.root.pose = Pose{Vec3{40.0 + static_cast<double>(a) * 3.0, 0.0, 0.0}, Quat::identity()};
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

TEST_CASE("State hash with contact-bearing articulations is identical across 1, 2, 4, 10 and 24 workers", "[determinism][articulation][contact][heavy]") {
    constexpr int kTicks = 100;
    const std::uint64_t reference = run_contact_scenario(1, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_contact_scenario(workers, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

namespace {

// PLAN.md P3 acceptance ("every phase P3-P8 additionally repeats the
// thread-count determinism test with its new components included") +
// out/handover_p3.md decision 3: vehicle::System::step_all runs one job per
// vehicle via parallel_for, exactly like articulation::System::step_all
// above, and each wheel's own internal state (compression, spin, relaxed
// slip - not otherwise observable from the chassis body alone) folds into
// World::state_hash() via vehicle::System::fold_state_hash. Two vehicles
// (so the parallel_for batch has more than one job) built directly from a
// VehicleDesc in code (not a data/vehicles/*.json file - this test only
// needs a plausible wheel/tyre setup, not the shipped car/tractor data),
// each on its own chassis body, driven straight then given a step-steer
// input mid-run so the tyre model's slip-relaxation ODE and Ackermann
// steering are actually exercised, not just a resting/idle vehicle.
vehicle::VehicleDesc make_test_vehicle_desc() {
    vehicle::VehicleDesc desc;
    desc.name = "det_test_car";
    desc.source = "test-only synthetic vehicle, not a real car - see this file's header comment";
    desc.drive.torque_curve_nm_vs_rpm.x = {900.0, 6500.0};
    desc.drive.torque_curve_nm_vs_rpm.y = {150.0, 150.0};

    struct Corner { const char* name; real x, y; bool front, driven; };
    const Corner corners[4] = {
        {"FL", 1.35, 0.78, true, false}, {"FR", 1.35, -0.78, true, false},
        {"RL", -1.35, 0.78, false, true}, {"RR", -1.35, -0.78, false, true},
    };
    for (const Corner& c : corners) {
        vehicle::WheelDesc w;
        w.name = c.name;
        w.attachment_local = Vec3{c.x, c.y, -0.1};
        w.is_front = c.front;
        w.driven = c.driven;
        w.steered = c.front;
        w.tyre.name = std::string(c.name) + "_tyre";
        w.tyre.source = "defaults from ps/vehicle/tyre_desc.h, test-only";
        desc.wheels.push_back(w);
    }
    desc.wheels[0].arb_partner = 1; desc.wheels[1].arb_partner = 0;
    desc.wheels[2].arb_partner = 3; desc.wheels[3].arb_partner = 2;
    return desc;
}

std::uint64_t run_vehicle_scenario(int workers, int ticks) {
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = workers;
    World world(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{500, 500, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0, 0, -0.5};
    world.create_body(ground);

    const vehicle::VehicleDesc desc = make_test_vehicle_desc();
    for (int v = 0; v < 2; ++v) {
        BodyDesc chassis;
        chassis.shape = BoxShape{Vec3{1.6, 0.5, 0.15}};
        chassis.motion = BodyMotionType::Dynamic;
        chassis.mass = 1400.0;
        chassis.friction = 0.5;
        chassis.allow_sleep = false;
        chassis.pose.position = Vec3{0.0, static_cast<real>(v) * 6.0, 0.5};
        chassis.velocity.linear = Vec3{15.0, 0.0, 0.0};
        const BodyId body = world.create_body(chassis);
        world.create_vehicle(desc, body);
    }

    for (int t = 0; t < ticks; ++t) {
        const real time_s = static_cast<real>(t) * world.dt();
        world.set_control("steer", time_s < 0.3 ? real(0.0) : real(0.2));
        world.set_control("throttle", real(0.3));
        world.step();
    }
    return world.state_hash();
}

} // namespace

TEST_CASE("State hash with vehicles is identical across 1, 2, 4, 10 and 24 workers", "[determinism][vehicle][heavy]") {
    constexpr int kTicks = 150;
    const std::uint64_t reference = run_vehicle_scenario(1, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_vehicle_scenario(workers, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// data/scenarios/terrain_multi_interest.json (vault PHYS-026): TWO terrain
// interest points streaming at once, a static pad that triggers terrain
// suppression, dynamic balls on the streamed terrain. The same file is in
// tools/ci.ps1's Windows-vs-Linux hash compare; this covers worker count.
namespace {
std::uint64_t run_scenario_file(const char* file, int workers) {
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/" + file);
    sc.world_config.job_workers = workers;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    const auto ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    for (int i = 0; i < ticks; ++i) built.world->step();
    return built.world->state_hash();
}
} // namespace

TEST_CASE("State hash with two terrain interest points and suppression is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][terrain][heavy]") {
    const std::uint64_t reference = run_scenario_file("terrain_multi_interest.json", 1);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("terrain_multi_interest.json", workers);
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
