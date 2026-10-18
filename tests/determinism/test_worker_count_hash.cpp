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
#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/environment/environment.h"
#include "ps/environment/vehicle_air.h"
#include "ps/io/scenario.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
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

TEST_CASE("State hash is identical across 1, 2, 4, 10 and 24 workers", "[determinism][heavy][world][backend]") {
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

TEST_CASE("State hash with several articulations is identical across 1, 2, 4, 10 and 24 workers", "[determinism][articulation][heavy][world]") {
    constexpr int kTicks = 100;
    const std::uint64_t reference = run_articulation_scenario(1, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_articulation_scenario(workers, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

TEST_CASE("State hash with contact-bearing articulations is identical across 1, 2, 4, 10 and 24 workers", "[determinism][articulation][contact][heavy][world]") {
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

    struct Corner { const char* name; real x, y; bool front; };
    const Corner corners[4] = {
        {"FL", 1.35, 0.78, true}, {"FR", 1.35, -0.78, true},
        {"RL", -1.35, 0.78, false}, {"RR", -1.35, -0.78, false},
    };
    for (const Corner& c : corners) {
        vehicle::WheelDesc w;
        w.name = c.name;
        w.attachment_local = Vec3{c.x, c.y, -0.1};
        w.is_front = c.front;
        w.steered = c.front;
        w.tyre.name = std::string(c.name) + "_tyre";
        w.tyre.source = "defaults from ps/vehicle/tyre_desc.h, test-only";
        desc.wheels.push_back(w);
    }
    desc.wheels[0].arb_partner = 1; desc.wheels[1].arb_partner = 0;
    desc.wheels[2].arb_partner = 3; desc.wheels[3].arb_partner = 2;

    // PHYS-033 P4 I3 plan section 11: migrated from I2's engine/final_drive/
    // diff-only graph to the full I3 chain (engine -> clutch -> gearbox ->
    // open diff -> RL/RR), so this determinism fixture actually exercises
    // the clutch/gearbox/TCU state this increment adds - a bare engine-plus-
    // final-drive graph (I2's shape) would never touch any of that new
    // state. Motoring curve reused verbatim from data/engines/i4_2l_na.json
    // (plan section 10.1); everything else is a plausible round number, not
    // meant to match any shipped vehicle.
    drivetrain::TorqueMapEngineDesc engine;
    engine.wot_torque_nm_vs_rpm.x = {900.0, 6500.0};
    engine.wot_torque_nm_vs_rpm.y = {150.0, 150.0};
    engine.motoring_torque_nm_vs_rpm.x = {0.0, 500.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0};
    engine.motoring_torque_nm_vs_rpm.y = {-15.0, -28.0, -32.0, -38.0, -45.0, -52.0, -61.0, -72.0, -85.0};
    engine.inertia_kgm2 = 0.16;
    engine.breakaway_torque_nm = 20.0;
    engine.idle_rpm = 900.0;
    engine.stall_rpm = 400.0;
    engine.fire_rpm = 250.0;
    engine.start_complete_rpm = 550.0;
    engine.limiter.type = drivetrain::LimiterType::FuelCut;
    engine.limiter.rpm = 6500.0;
    engine.limiter.hysteresis_rpm = 150.0;

    drivetrain::ClutchDesc clutch;
    clutch.capacity_nm = 300.0;
    clutch.static_factor = 1.15;
    clutch.engagement_vs_pedal.x = {0.0, 1.0};
    clutch.engagement_vs_pedal.y = {1.0, 0.0};
    // Non-zero: in Neutral, the gearbox's own gear row is not registered,
    // so N1 (clutch.out + gearbox.in, joined by the same shaft) becomes a
    // FREE reduced coordinate - it needs SOME inertia or the cluster's
    // reduced mass matrix in state N is singular (NetworkBuilder::build's
    // own Cholesky check, plan section 4.3).
    clutch.out_inertia_kgm2 = 0.004;

    drivetrain::GearboxDesc gearbox;
    gearbox.type = drivetrain::GearboxType::Manual;
    gearbox.forward_ratios = {3.0, 1.6, 1.0};
    gearbox.forward_efficiency = {1.0, 1.0, 1.0};

    drivetrain::DifferentialDesc diff;
    diff.kind = drivetrain::DifferentialKind::Open;
    diff.ratio = 3.7;
    diff.efficiency = 1.0;

    desc.powertrain.components = {
        {"engine", engine}, {"clutch", clutch}, {"gearbox", gearbox}, {"diff", diff},
    };
    desc.powertrain.shafts = {
        {{{"engine", "crank", false}, {"clutch", "in", false}}},
        {{{"clutch", "out", false}, {"gearbox", "in", false}}},
        {{{"gearbox", "out", false}, {"diff", "carrier", false}}},
        {{{"diff", "left", false}, {"RL", "", true}}},
        {{{"diff", "right", false}, {"RR", "", true}}},
    };
    desc.powertrain.compliance = drivetrain::Compliance::Rigid;
    desc.powertrain.rotor_reaction = true;

    desc.controllers = {
        {"tcu", drivetrain::ManualTcuDesc{"gearbox", "clutch", "engine", {}, {}, {}, 2}},
    };

    desc.brake_system.type = drivetrain::BrakeSystemType::DirectTorque;
    desc.brake_system.corners = {
        {"FL", 2200.0}, {"FR", 2200.0}, {"RL", 1400.0}, {"RR", 1400.0},
    };
    desc.brake_system.handbrake = {{"RL", "RR"}, 1400.0};
    desc.brake_system.static_factor = 1.0;

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
        const real steer = time_s < 0.3 ? real(0.0) : real(0.2);
        // PLAN.md P4 I0 (PHYS-033 section 8, Q4): a bare channel name now
        // addresses only the designated input vehicle (index 0); vehicle 1
        // needs its own "v1."-prefixed channel to keep receiving the same
        // steer/throttle command both vehicles shared before per-vehicle
        // channel resolution existed (this test's own header comment: both
        // vehicles are meant to be "driven straight then given a step-steer
        // input mid-run").
        world.set_control("steer", steer);
        world.set_control("v1.steer", steer);
        world.set_control("throttle", real(0.3));
        world.set_control("v1.throttle", real(0.3));
        // PHYS-033 P4 I3 plan section 11: engage 1st at the start, then
        // upshift to 2nd partway through, on both vehicles - exercises the
        // gearbox's phase machine and a topology-state switch under the
        // determinism harness. The plan's own text says "tick 240"; this
        // fixture's kTicks is 150, so the upshift is moved to tick 60
        // (still well clear of the launch transient) to actually fire
        // within the run.
        if (t == 0) {
            world.set_control("gear_request", real(1.0));
            world.set_control("v1.gear_request", real(1.0));
        } else if (t == 60) {
            world.set_control("gear_request", real(2.0));
            world.set_control("v1.gear_request", real(2.0));
        }
        world.step();
    }
    return world.state_hash();
}

} // namespace

TEST_CASE("State hash with vehicles is identical across 1, 2, 4, 10 and 24 workers", "[determinism][vehicle][heavy][world]") {
    constexpr int kTicks = 150;
    const std::uint64_t reference = run_vehicle_scenario(1, kTicks);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_vehicle_scenario(workers, kTicks);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

namespace {
// PHYS-033 P4 I3 plan section 13.9: the same migrated-vehicle fixture
// (make_test_vehicle_desc), but with assist.auto_shift = 1 (the TCU's own
// RPM-threshold upshift path, not this test's own gear_request stepping)
// and full throttle to force at least one auto-triggered shift, run long
// enough that its own gearbox.synchro (make_test_vehicle_desc never sets
// GearboxDesc::synchro, so forward_capacity_nm is empty - "0 = no synchro
// (dog only)" per SynchroDesc's own field comment) cannot speed-match
// within Gearbox::StepResult's grind_after_s window, so the shift GRINDS
// (DrivelineSoundState::grind_events > 0) - the plan's own explicit ask,
// exercising the LCP/topology-switch code path under a grind, which the
// original vehicle case (gear_request stepping, no auto-shift, presumably
// clean synchronisation) never touched. grind_events itself is sound-only
// (not hashed, powertrain_state.h's own comment), so the CHECK below is a
// scenario-sanity precondition (this fixture must actually grind, or it
// tests nothing new), not part of the hash-equality gate itself.
std::uint64_t run_vehicle_scenario_auto_shift_grind(int workers, int ticks, std::uint32_t* grind_events_out) {
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
    VehicleId vid0{};
    for (int v = 0; v < 2; ++v) {
        BodyDesc chassis;
        chassis.shape = BoxShape{Vec3{1.6, 0.5, 0.15}};
        chassis.motion = BodyMotionType::Dynamic;
        chassis.mass = 1400.0;
        chassis.friction = 0.5;
        chassis.allow_sleep = false;
        chassis.pose.position = Vec3{0.0, static_cast<real>(v) * 6.0, 0.5};
        chassis.velocity.linear = Vec3{15.0, 0.0, 0.0}; // rolling start, matches run_vehicle_scenario above -
                                                          // a dead-stop full-throttle launch produced extreme
                                                          // wheelspin that crashed Jolt's own Quat::sRotation
                                                          // axis-normalize assert, not this fixture's intent
        const BodyId body = world.create_body(chassis);
        const VehicleId vid = world.create_vehicle(desc, body);
        if (v == 0) vid0 = vid;
    }

    world.set_control("gear_request", real(1.0));
    world.set_control("v1.gear_request", real(1.0));
    world.set_control("assist.auto_shift", real(1.0));
    world.set_control("v1.assist.auto_shift", real(1.0));
    world.set_control("throttle", real(1.0));
    world.set_control("v1.throttle", real(1.0));
    for (int t = 0; t < ticks; ++t) {
        if (t == 1) {
            // Hand off to the TCU's own auto-shift path (manual_tcu.cpp's
            // arbitration: an in-range gear_request permanently overrides
            // the counter/auto-shift path, same rule
            // test_powertrain_energy_balance.cpp's own comment documents).
            world.set_control("gear_request", real(drivetrain::kGearRequestNone));
            world.set_control("v1.gear_request", real(drivetrain::kGearRequestNone));
        }
        world.step();
    }
    if (grind_events_out != nullptr) *grind_events_out = world.powertrain_state(vid0).driveline.grind_events;
    return world.state_hash();
}
} // namespace

TEST_CASE("State hash with an auto-shift-triggered gearbox grind is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][vehicle][heavy][world]") {
    constexpr int kTicks = 200;
    std::uint32_t grind_events_reference = 0;
    const std::uint64_t reference = run_vehicle_scenario_auto_shift_grind(1, kTicks, &grind_events_reference);
    INFO("reference grind_events=" << grind_events_reference);
    CHECK(grind_events_reference > 0); // scenario-sanity precondition, see this fixture's own comment
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_vehicle_scenario_auto_shift_grind(workers, kTicks, nullptr);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// data/scenarios/terrain_multi_interest.json (vault PHYS-026): TWO terrain
// interest points streaming at once, a static pad that triggers terrain
// suppression, dynamic balls on the streamed terrain. The same file is in
// tools/ci.ps1's Windows-vs-Linux hash compare; this covers worker count.
namespace {
// Applies scenario.control_events in time order exactly the way
// tools/ps_run/main.cpp's own tick loop does, so a scenario file with
// control_events (e.g. vehicle_tractor_i4.json) hashes the same driven
// state here as it does under ps_run - a scenario with none (e.g.
// terrain_multi_interest.json) sees this loop never fire, unchanged from
// the pre-control-events version of this helper.
std::uint64_t run_scenario_file(const char* file, int workers) {
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/" + file);
    sc.world_config.job_workers = workers;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    const auto ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    std::size_t next_control_event = 0;
    std::size_t next_environment_event = 0; // weather plan W2: same helper ps_run calls
    for (int i = 0; i < ticks; ++i) {
        ps::io::apply_environment_events(world, sc, next_environment_event);
        while (next_control_event < sc.control_events.size() && sc.control_events[next_control_event].time_s <= world.sim_time()) {
            const auto& ev = sc.control_events[next_control_event];
            world.set_control(ev.channel, ev.value);
            ++next_control_event;
        }
        world.step();
    }
    return world.state_hash();
}
} // namespace

// Weather plan W2: data/scenarios/environment_day.json - car_sedan driving under an
// installed environment (clock + calendar + sun, scale 20000 across an autumn -> winter
// boundary, a time jump at t=3 s and a scale change at t=5 s). The same file is in
// tools/ci.ps1's Windows-vs-Linux hash compare once the coordinator registers it; this
// covers worker count. The environment folds its clock anchor into the hash, so the
// events must have run (asserted below) for the equality to mean anything.
TEST_CASE("State hash with an installed environment is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][environment][heavy][world][io]") {
    const std::uint64_t reference = run_scenario_file("environment_day.json", 1);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("environment_day.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

TEST_CASE("environment_day.json: the environment events really run and move the hash",
          "[determinism][environment][world][io]") {
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/environment_day.json");
    sc.world_config.job_workers = 1;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    const auto ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    std::size_t next_control_event = 0, next_environment_event = 0;
    std::string first_season, last_season;
    double first_elev = 0.0, last_elev = 0.0;
    for (int i = 0; i < ticks; ++i) {
        ps::io::apply_environment_events(world, sc, next_environment_event);
        while (next_control_event < sc.control_events.size() && sc.control_events[next_control_event].time_s <= world.sim_time()) {
            world.set_control(sc.control_events[next_control_event].channel, sc.control_events[next_control_event].value);
            ++next_control_event;
        }
        world.step();
        const auto& env = world.snapshots().read().environment;
        if (i == 0) { first_season = env.season_name; first_elev = env.sun_elevation_rad; }
        last_season = env.season_name;
        last_elev = env.sun_elevation_rad;
    }
    CHECK(next_environment_event == 2);
    CHECK(first_season == "autumn");
    CHECK(last_season == "winter");          // 8 s of 20000x plus a 40000 s jump crossed the season boundary
    CHECK(first_elev != last_elev);          // the sun moved
    CHECK(world.environment_snapshot()->time_scale == 5000.0);

    // Without the environment events the same run hashes differently (the clock anchor is folded in).
    ps::io::Scenario sc2 = sc;
    sc2.environment->events.clear();
    ps::io::BuiltScenario built2 = ps::io::build_scenario(sc2);
    next_control_event = 0;
    next_environment_event = 0;
    for (int i = 0; i < ticks; ++i) {
        while (next_control_event < sc2.control_events.size() && sc2.control_events[next_control_event].time_s <= built2.world->sim_time()) {
            built2.world->set_control(sc2.control_events[next_control_event].channel, sc2.control_events[next_control_event].value);
            ++next_control_event;
        }
        built2.world->step();
    }
    CHECK(built2.world->state_hash() != world.state_hash());
}

TEST_CASE("State hash with two terrain interest points and suppression is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][terrain][heavy][world][io]") {
    const std::uint64_t reference = run_scenario_file("terrain_multi_interest.json", 1);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("terrain_multi_interest.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// Terrain prefetch ring (data/scenarios/terrain_prefetch_*.json): the ring
// only moves WHERE a tile's content is built (a background lane, swapped in
// at install) - never which key lands in which pool slot at which tick - so
// prefetch on must hash exactly like prefetch off. The off reference is the
// same scenario with both prefetch fields zeroed (TileManager's plain path),
// at 1 worker; on runs at 1, 4 and 24 workers. The static case is what
// ps_run (and tools/ci.ps1's Windows-vs-Linux hash compare) runs; the driven
// case moves terrain_prefetch_drive.json's interest point 2 across tile rows
// in C++ (a scenario's interest points are static) and compares every 60
// ticks as well as at the end. Each on-run must actually install tiles from
// the lane, or the comparison proves nothing.
namespace {
struct PrefetchRun {
    std::vector<std::uint64_t> hashes; // every 60 ticks, then the final one
    ps::terrain::PrefetchStats stats;
};

PrefetchRun run_prefetch_scenario(const char* file, int workers, bool prefetch, bool drive) {
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/" + file);
    sc.world_config.job_workers = workers;
    REQUIRE(sc.terrain.has_value());
    REQUIRE(sc.terrain->config.prefetch_margin_tiles > 0);
    REQUIRE(sc.terrain->config.prefetch_max_tiles > 0);
    if (!prefetch) {
        sc.terrain->config.prefetch_margin_tiles = 0;
        sc.terrain->config.prefetch_max_tiles = 0;
    }
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    const auto ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    Vec3 start;
    real radius = 0;
    for (const auto& ip : sc.terrain->interest_points) {
        if (ip.id == 2) {
            start = ip.world_xy;
            radius = ip.radius_m;
        }
    }
    REQUIRE(radius > 0);
    PrefetchRun run;
    for (int i = 0; i < ticks; ++i) {
        if (drive) {
            // 0.35 m/tick east and 0.12 m/tick north (84 m/s): a tile row or
            // column (16 m) every ~46 / ~133 ticks, the prefetch margin
            // ahead of it.
            const real t = static_cast<real>(i);
            world.set_terrain_interest_point(2, Vec3{start.x + 0.35 * t, start.y + 0.12 * t, 0.0}, radius);
        }
        world.step();
        if ((i + 1) % 60 == 0) run.hashes.push_back(world.state_hash());
    }
    run.hashes.push_back(world.state_hash());
    run.stats = world.terrain_prefetch_stats();
    return run;
}

void require_prefetch_matches_off(const char* file, bool drive) {
    const PrefetchRun reference = run_prefetch_scenario(file, 1, false, drive);
    CHECK(reference.stats.installed == 0);
    CHECK(reference.stats.enqueued == 0);
    for (int workers : {1, 4, 24}) {
        const PrefetchRun on = run_prefetch_scenario(file, workers, true, drive);
        INFO(file << " drive=" << drive << " workers=" << workers << " installed=" << on.stats.installed
                  << " late_sync=" << on.stats.late_sync << " late_wait=" << on.stats.late_wait
                  << " cancelled=" << on.stats.cancelled << " skipped_suppression=" << on.stats.skipped_suppression);
        REQUIRE(on.stats.installed > 0);
        REQUIRE(on.hashes.size() == reference.hashes.size());
        for (std::size_t k = 0; k < on.hashes.size(); ++k) {
            INFO("checkpoint " << k << " hash=0x" << std::hex << on.hashes[k] << " reference=0x" << reference.hashes[k]);
            REQUIRE(on.hashes[k] == reference.hashes[k]);
        }
    }
}
} // namespace

TEST_CASE("Terrain prefetch on hashes like prefetch off at 1, 4 and 24 workers (static scenarios)",
          "[determinism][terrain][prefetch][heavy][world][io]") {
    require_prefetch_matches_off("terrain_prefetch_drive.json", false);
    require_prefetch_matches_off("terrain_prefetch_multi_interest.json", false);
}

TEST_CASE("Terrain prefetch on hashes like prefetch off at 1, 4 and 24 workers (interest point driven across tile rows)",
          "[determinism][terrain][prefetch][heavy][world][io]") {
    require_prefetch_matches_off("terrain_prefetch_drive.json", true);
}

// data/scenarios/vehicle_tractor_i4.json (PHYS-033 P4 I4 C15): a single
// tractor exercising every tractor-only control channel in one 10 s run -
// hand_throttle, pto, three incremental shift_up_count steps (N->1->2->3),
// awd, the C15 diff_lock on/off, brake to a stop, then park_lock. Pins the
// literal hash recorded when the scenario file was authored (ps_run at
// 1/2/4/10 workers all agreed before this test existed).
// PHYS-033 P4 I7a C5b (owner ruling 2026-09-26; reviewer-measured 2026-09-27):
// this scenario's hash was checked and confirmed UNCHANGED by C5b's reorder
// (ManualTcu now reads the throttle-command stage's command - here
// max(hand_throttle, pedal) via the engine's governor hand lever - instead
// of the raw pedal). Instrumented over the full run (ps_run --workers 1,
// 9600 substeps at 960 Hz over the scenario's 10 s): assist_auto_clutch is
// on for all 9600 substeps and auto_shift is off; the TCU sits in Launch
// mode for 3556 of those substeps, not just at the N->1st shift. Of those,
// throttle_request differs from the raw pedal only for one substep at EACH
// of the three gear engagements (N->1, 1->2, 2->3) - three substeps total.
// At each of those three substeps the engine is settled on the governor's
// no-load setpoint under the pto's load, ~1758 rpm (droop-reduced from the
// unloaded idle + hand_throttle*(high_idle - idle) = 850 + 0.6*(2400-850) =
// 1780 rpm quoted in an earlier draft of this comment - the pto load pulls
// it down via the droop band), which exceeds BOTH candidate full-engagement
// thresholds (full_rpm_min=1200, raw pedal=0; full_rpm_min + 0.6*(1700-1200)
// = 1500, the command) - e_tgt = clamp((rpm-bite)/(full-bite), 0, 1)
// saturates to 1.0 identically either way, and slip is already under
// lock_slip_rpm, so each engagement locks immediately regardless of which
// full-rpm hypothesis fed it. The remaining ~2237 Launch substeps are the
// post-stop anti-stall re-entry after t~6.5 s, where hand_throttle reads 0
// (the scenario has already dropped it), so throttle_request == the raw
// pedal there too - no divergence. Net: 3 substeps see the underlying
// signal actually differ, 0 differences in ManualTcu's outputs or the TCU's
// own state, hence the hash is bit-identical. See
// tests/unit/test_throttle_control.cpp's "C5b: the tractor's hand_throttle
// reaches the auto-clutch launch law" test for a scripted case with a lower
// hand_throttle where the two full-rpm thresholds bracket the engine rpm
// and the difference IS observable (used there as C5b's actual behavioural
// discriminator).
// Left at the pre-C5b recorded value - do not re-record without first
// re-deriving this saturation argument for the new build.
TEST_CASE("State hash for the tractor I4 control-channel exercise scenario is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][vehicle][drivetrain][heavy][world]") {
    constexpr std::uint64_t kRecorded = 0xee2c08f11ee76f55ULL; // 2026-10-14 D0 dog release spike: JoltRigidBackend::shape_cast re-casts a phantom wheel hit (this scenario has one, a rear wheel cast at fraction 0 / penetration 0.186 m with a true fraction of 0.5005) - moved this hash (was 0x44327cc4b10c57b5); 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0xe471f1f3e5ad4328): re-recorded 2026-10-04: ca118ec's instant anti-stall release (Locked below anti_stall_rpm on engine OR signed input rpm) with the rolling re-close restored to rolling_reengage_rpm (vault PHYS-040); was 0xac386af7129b9ffe
    const std::uint64_t reference = run_scenario_file("vehicle_tractor_i4.json", 1);
    INFO("reference hash=0x" << std::hex << reference << " recorded=0x" << kRecorded);
    REQUIRE(reference == kRecorded);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("vehicle_tractor_i4.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// C15a review companion: the hash-equality case above proves the tractor's
// end state is reproducible across worker counts, but reproducibility alone
// is not evidence any individual control event actually did anything - a
// scenario that silently ignored every event would hash just as
// reproducibly. This runs vehicle_tractor_i4.json once (1 worker) and reads
// World's live powertrain/chassis state at points between successive
// events, asserting each event's expected effect directly rather than
// inferring it from telemetry deltas.
TEST_CASE("Tractor I4 scenario's control events produce the expected live powertrain state",
          "[determinism][vehicle][drivetrain][world]") {
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/vehicle_tractor_i4.json");
    sc.world_config.job_workers = 1;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    const ps::VehicleId vehicle = built.vehicles_by_name.at("tractor");
    const ps::BodyId chassis = built.bodies_by_name.at("chassis");

    std::size_t next_control_event = 0;
    auto step_until = [&](ps::real t_target) {
        while (world.sim_time() < t_target) {
            while (next_control_event < sc.control_events.size() && sc.control_events[next_control_event].time_s <= world.sim_time()) {
                const auto& ev = sc.control_events[next_control_event];
                world.set_control(ev.channel, ev.value);
                ++next_control_event;
            }
            world.step();
        }
    };

    // Just before the first shift (t=1.0): the pto event fired at t=0.3 and
    // tractor.json's pto engage_time_s is 1.5 s, so full engagement isn't
    // guaranteed yet here - only check it has started ramping.
    step_until(0.9);
    {
        const auto& state = world.powertrain_state(vehicle);
        CHECK(state.pto_engagement > 0.0);
        INFO("t=0.9 pto_engagement=" << state.pto_engagement);
    }

    // Just before the awd event (t=4.0): three shift_up_count events (at
    // t=1,2,3) and tractor.json's pto ramp (fully engaged by t=1.8) have
    // both had time to complete.
    step_until(3.9);
    {
        const auto& state = world.powertrain_state(vehicle);
        CHECK(state.gear == 3);
        CHECK(state.pto_engagement == Catch::Approx(1.0).margin(1e-6));
        CHECK(state.pto_rpm > 0.0);
        INFO("t=3.9 gear=" << int(state.gear) << " pto_engagement=" << state.pto_engagement
                            << " pto_rpm=" << state.pto_rpm);
    }

    // Just before the diff_lock-on event (t=5.0): the awd event at t=4.0 has
    // had the transfer case's 0.3 s engage_time_s to fully ramp.
    step_until(4.9);
    {
        const auto& state = world.powertrain_state(vehicle);
        CHECK(state.awd_engagement == Catch::Approx(1.0).margin(1e-6));
        INFO("t=4.9 awd_engagement=" << state.awd_engagement);
    }

    // Just before the diff_lock-off event (t=6.0): the rear diff (component
    // index 0, PowertrainSnapshot::diffs' component-array order) should be
    // locked.
    step_until(5.9);
    {
        const auto& state = world.powertrain_state(vehicle);
        REQUIRE(state.diff_count >= 1);
        CHECK(state.diffs[0].locked);
        INFO("t=5.9 diffs[0].locked=" << state.diffs[0].locked);
    }

    // Just before the brake event (t=7.0): the diff_lock-off event at t=6.0
    // releases immediately (components.h's own comment), so the rear diff
    // should be unlocked again.
    step_until(6.9);
    {
        const auto& state = world.powertrain_state(vehicle);
        CHECK_FALSE(state.diffs[0].locked);
        INFO("t=6.9 diffs[0].locked=" << state.diffs[0].locked);
    }

    // End of run (t=10): braking since t=7.0 should have brought the
    // tractor to a stop, and the park_lock event at t=9.0 should have
    // engaged (ParkLock's own max_engage_speed_rad_s gate needs the
    // gearbox-output shaft slow enough, which a stopped vehicle in gear
    // satisfies).
    step_until(10.0);
    {
        const auto& state = world.powertrain_state(vehicle);
        const ps::real speed = world.get_motion(chassis).linear.length();
        INFO("t=10 speed=" << speed << " park_lock_engaged=" << state.park_lock_engaged
                            << " engine_state=" << static_cast<int>(state.engine_state));
        CHECK(speed < 0.1);
        CHECK(state.park_lock_engaged == Catch::Approx(1.0).margin(1e-6));
        // Reported, not silently assumed: braking a tractor in gear 3 to a
        // standstill with no neutral/clutch event beforehand risks stalling
        // the engine unless the auto-clutch opens fast enough. See this
        // test's own commit message for the measured engine_state value.
        CHECK(state.engine_state != drivetrain::EngineState::Stalled);
    }
}

// data/scenarios/vehicle_fuel_tank.json (PHYS-033 P4 I7a C7; switched from
// Neutral to the in-gear variant by C7 review round 1's F4 fix - see
// tests/support/fixtures/vehicle_fuel_tank_car.json's own "source" field for
// the full measurement history of both variants): a car launched in 1st gear
// at WOT with the fixture's deliberately tiny 0.005 kg initial tank - the
// tank runs dry, the engine can no longer hold the driveline's speed, and the
// auto-clutch's anti-stall panic release dumps the clutch open before the
// wheels can drag the dead engine below stall_rpm; the engine stalls (with
// the clutch open) well within the scenario's 10 s duration. Pins the literal
// hash recorded when the in-gear scenario/fixture were authored (ps_run at
// 1/2/4/10 workers all agreed before this test existed).
TEST_CASE("State hash for the fuel tank starvation scenario is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][vehicle][drivetrain][fuel][heavy][world]") {
    constexpr std::uint64_t kRecorded = 0xb08736ed4b3a8d4fULL; // 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0xc89f5ed4df82b2dc): re-recorded 2026-10-04: ca118ec's instant anti-stall release (Locked below anti_stall_rpm on engine OR signed input rpm) with the rolling re-close restored to rolling_reengage_rpm (vault PHYS-040); was 0xe545d5f57136415f, recorded 2026-09-28 (in-gear F4 fix, fixed B1/B2/B3/F1 engine+fuel data)
    const std::uint64_t reference = run_scenario_file("vehicle_fuel_tank.json", 1);
    INFO("reference hash=0x" << std::hex << reference << " recorded=0x" << kRecorded);
    REQUIRE(reference == kRecorded);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("vehicle_fuel_tank.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// F12 companion (mirrors the tractor I4 "live effect" companion above): the
// hash-equality case above proves reproducibility, not that the tank
// actually emptied and the engine actually stalled through the intended
// mechanism - a scenario that silently ignored the fuel model, or stalled
// for some unrelated reason, would hash just as reproducibly. This runs
// vehicle_fuel_tank.json once (1 worker) and reads World's live fuel/engine/
// clutch state directly.
//
// C7 review round 1 (F4): the pre-review version of this test ran the
// scenario in Neutral, where the auto-clutch's Launch law fully closes the
// clutch regardless of engine rpm (manual_tcu.cpp's own comment: "Neutral
// (current == 0) is unaffected - the Launch law closes fully in N anyway"),
// so that stall was pure fuel exhaustion, never anti-stall-mediated. Now
// that the scenario launches in 1st gear (gear_request 1), the stall is
// expected to happen via the auto-clutch's anti-stall panic release
// (AutoClutchDesc::anti_stall_release_rate_per_s) opening the clutch as
// engine rpm falls through anti_stall_rpm - asserted below via the engine
// stalling AND the effective clutch engagement being low (open) at the tick
// it does, not merely that the two events occur in some order.
//
// For context, a pure-Neutral WOT rev of the same 0.005 kg tank (not what
// this scenario runs any more) has a closed-form empty-to-stall gap:
// integral of J/|m(omega)| from 400 to ~6450-6600 rpm, J = 0.16+0.004+0.010 =
// 0.174 kg*m^2 (engine+clutch+gearbox-input inertia, clutch locked, no
// driveline load), gives 3.73-3.80 s - matching the pre-review Neutral
// scenario's own measured 3.80 s gap (2.04 s empty, 5.84 s stall) almost
// exactly. That is a useful independent physics cross-check of the
// idle-integrator/fuel-starvation coupling, but it no longer describes what
// THIS scenario measures now that it launches in gear (the in-gear stall is
// mediated by the auto-clutch/driveline, not a closed-form coast-down), so
// it is not re-asserted here - see the fixture's own "source" field for the
// full numbers of every variant considered.
TEST_CASE("Fuel tank starvation scenario's tank empties and the engine stalls via the anti-stall path",
          "[determinism][vehicle][drivetrain][fuel][world]") {
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/vehicle_fuel_tank.json");
    sc.world_config.job_workers = 1;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    const ps::VehicleId vehicle = built.vehicles_by_name.at("car");

    const ps::real initial_tank_kg = world.fuel_tank_mass(vehicle);
    REQUIRE(initial_tank_kg > 0.0);
    REQUIRE(initial_tank_kg < 0.01); // the fixture's deliberately tiny 0.005 kg start

    std::size_t next_control_event = 0;
    int empty_tick = -1;
    int stall_tick = -1;
    ps::real clutch_engagement_at_stall = -1.0;
    ps::real prev_tank_kg = initial_tank_kg;
    bool tank_ever_decreased = false;
    const auto ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    for (int i = 0; i < ticks; ++i) {
        while (next_control_event < sc.control_events.size() && sc.control_events[next_control_event].time_s <= world.sim_time()) {
            const auto& ev = sc.control_events[next_control_event];
            world.set_control(ev.channel, ev.value);
            ++next_control_event;
        }
        world.step();

        const ps::real tank_kg = world.fuel_tank_mass(vehicle);
        if (tank_kg < prev_tank_kg) tank_ever_decreased = true;
        prev_tank_kg = tank_kg;
        if (empty_tick < 0 && tank_kg == 0.0) empty_tick = i;
        if (stall_tick < 0 && world.powertrain_state(vehicle).engine_state == drivetrain::EngineState::Stalled) {
            stall_tick = i;
            clutch_engagement_at_stall = world.powertrain_state(vehicle).clutch_engagement;
        }
    }

    CHECK(tank_ever_decreased);
    REQUIRE(empty_tick >= 0);
    REQUIRE(stall_tick >= 0);
    CHECK(empty_tick < stall_tick); // the tank must run dry before the engine stalls, not the other way round
    const ps::real empty_time_s = static_cast<ps::real>(empty_tick) / sc.world_config.tick_rate_hz;
    const ps::real stall_time_s = static_cast<ps::real>(stall_tick) / sc.world_config.tick_rate_hz;
    INFO("empty_tick=" << empty_tick << " (" << empty_time_s << " s) stall_tick=" << stall_tick << " (" << stall_time_s
                        << " s) clutch_engagement_at_stall=" << clutch_engagement_at_stall);
    // Measured 2026-09-28 (in-gear variant, current fixed B1/B2/B3/F1
    // engine+fuel data): empty_tick=283 (1.17917 s), stall_tick=1569
    // (6.5375 s), clutch_engagement_at_stall=0.0 (fully open - the
    // anti-stall panic release, not a wheel-dragged stall with the clutch
    // still closed). Bounds below carry margin around these measured
    // values, not tight-fitted to them.
    CHECK(empty_time_s < real(2.0));
    CHECK(stall_time_s < real(8.0));
    // The anti-stall path specifically: the clutch must be substantially open
    // (not locked/closed) at the exact tick the engine is recorded Stalled -
    // a stall that happened with the clutch still closed would mean the
    // wheels dragged the engine down directly, not the anti-stall release
    // this scenario is meant to exercise.
    CHECK(clutch_engagement_at_stall < real(0.2));
    CHECK(world.fuel_tank_mass(vehicle) == 0.0); // stays empty (no refuel event in this scenario)

    const drivetrain::PowertrainEnergy& e = world.powertrain_energy(vehicle);
    // Closed-form mass balance: everything the tank lost was booked as burnt
    // fuel (no external refuel in this scenario, so fuel_external_added == 0).
    CHECK(std::abs((initial_tank_kg - world.fuel_tank_mass(vehicle)) - e.fuel_mass) <= 1e-12);
}

// PHYS-033 P4 I7c C17a M-A S5: data/scenarios/vehicle_gen_engine.json -
// vehicle_sim_engine.json's control events plus a lift to throttle 0 at
// t = 5.5 s, on car_sedan_gen.json, whose engine maps are GENERATED at load
// time from the engine file's cycle block (C16a). The scenario is loaded ONCE
// (maps generated on a 10-worker pool - the maps are pool-size invariant,
// test_engine_map_generator M3) and then built and run per worker count.
namespace {
struct GenScenarioRun {
    std::uint64_t hash = 0;
    real rpm_at_throttle = 0.0, max_rpm_under_throttle = 0.0;
    int max_gear = 0;
    real first_low_p_im_after_lift_s = -1.0; // p_im < 40 kPa
    real fuel_mass_kg = 0.0;
};

ps::io::Scenario load_gen_engine_scenario() {
    jobs::JobSystem pool(10);
    ps::io::EngineMapOptions opt;
    opt.pool = &pool;
    return ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/vehicle_gen_engine.json", opt);
}

GenScenarioRun run_gen_engine_scenario(ps::io::Scenario sc, int workers) {
    sc.world_config.job_workers = workers;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    const ps::VehicleId vehicle = built.vehicles_by_name.at("car");
    GenScenarioRun r;
    const auto ticks = static_cast<int>(sc.duration_s * sc.world_config.tick_rate_hz + 0.5);
    std::size_t next_control_event = 0;
    for (int i = 0; i < ticks; ++i) {
        while (next_control_event < sc.control_events.size() && sc.control_events[next_control_event].time_s <= world.sim_time()) {
            const auto& ev = sc.control_events[next_control_event];
            world.set_control(ev.channel, ev.value);
            ++next_control_event;
        }
        const real t0 = world.sim_time();
        world.step();
        const drivetrain::PowertrainSnapshot& s = world.powertrain_state(vehicle);
        if (t0 < 0.3) r.rpm_at_throttle = s.engine_rpm;
        if (t0 >= 0.3 && t0 < 5.5) r.max_rpm_under_throttle = std::max(r.max_rpm_under_throttle, s.engine_rpm);
        r.max_gear = std::max(r.max_gear, static_cast<int>(s.gear));
        if (t0 >= 5.5 && r.first_low_p_im_after_lift_s < 0.0 && !s.engines.empty() && s.engines[0].p_intake_pa < 40e3)
            r.first_low_p_im_after_lift_s = world.sim_time() - 5.5;
    }
    r.fuel_mass_kg = world.powertrain_energy(vehicle).fuel_mass;
    r.hash = world.state_hash();
    return r;
}
} // namespace

TEST_CASE("State hash for the generated-map engine scenario is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][vehicle][drivetrain][heavy][world]") {
    constexpr std::uint64_t kRecorded = 0x82bf30120b3212bbULL; // 2026-10-18 knock opt-in of toyota_a25a_fks_gen moved this hash (was 0x2164909873144ac3); 2026-10-13 pedal plan P1 (manual pedal law T* = Z + c (F - Z) on electronic engines) moved this hash (was 0x194fc20db754f52c); 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0x84518e7edcdc3b6a): re-recorded 2026-10-04: ca118ec's instant anti-stall release (Locked below anti_stall_rpm on engine OR signed input rpm) with the rolling re-close restored to rolling_reengage_rpm (vault PHYS-040); was 0x4030651924c4dbd2, recorded 2026-09-29 (owner-requested SimulatedEngine idle rework: pedal-released one-sided PI, DFCO dashpot, manifold-fill term; was 0x5e88a765a7e748e4, recorded 2026-09-28 at the C17a second follow-up: cited burn_duration_deg/ignition_delay_deg, PLAN.md "Second follow-up rulings" commit 69a3856, vault PHYS-055; ps_run 1/2/4/10/24 workers; was 0xbc61f26e2d05379e at the measured-spark-schedule follow-up)
    const ps::io::Scenario sc = load_gen_engine_scenario();
    const std::uint64_t reference = run_gen_engine_scenario(sc, 1).hash;
    INFO("reference hash=0x" << std::hex << reference << " recorded=0x" << kRecorded);
    REQUIRE(reference == kRecorded);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_gen_engine_scenario(sc, workers).hash;
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// S5 effect companion: the hash case proves reproducibility, not that the
// scenario drives the generated engine the intended way.
TEST_CASE("Generated-map engine scenario: rpm rises under throttle, upshifts, p_im falls after lift, fuel is booked",
          "[determinism][vehicle][drivetrain][world]") {
    const GenScenarioRun r = run_gen_engine_scenario(load_gen_engine_scenario(), 1);
    std::printf("[S5 gen engine] rpm at throttle %.1f, max under throttle %.1f, max gear %d, p_im < 40 kPa %.4f s "
                "after lift, fuel %.6f kg\n",
                r.rpm_at_throttle, r.max_rpm_under_throttle, r.max_gear, r.first_low_p_im_after_lift_s, r.fuel_mass_kg);
    CHECK(r.max_rpm_under_throttle > r.rpm_at_throttle + 1500.0);
    CHECK(r.max_gear >= 2);
    CHECK(r.first_low_p_im_after_lift_s >= 0.0);
    CHECK(r.first_low_p_im_after_lift_s <= 0.5);
    CHECK(r.fuel_mass_kg > 0.0);
}

// 2026-10-06: the tyre rolling-resistance work (derived qsy3/qsy4 speed term in every car/tractor tyre file) moved every vehicle hash below; the old values are the ones quoted in the comment and the line-end notes, the W4 claim (a weather-less World is unchanged by W4) is re-pinned against the new baseline.
// Weather plan W4: without a weather environment nothing changed. The hashes below are the values
// recorded on master BEFORE W4 (CLAUDE.md scenario list, 2026-10-04/05); the W4 branch reproduced every
// one at 1 and 10 workers. vehicle_step_steer 0xbab69b300eba41a3 (2026-09-25 60/40 split),
// environment_day 0x2af395ee8a54f0b5 (W2: a clock-only environment, no weather). A sabotage that made
// the vehicle step read the environment air for a weather-less World, or folded the weather into the
// hash unconditionally, moves these.
TEST_CASE("W4: scenarios without a weather environment keep their pre-W4 state hash at 1 and 10 workers",
          "[determinism][environment][vehicle][heavy][world][io]") {
    struct Pin { const char* file; std::uint64_t hash; };
    const Pin pins[] = {
        {"vehicle_step_steer.json", 0xe77fd6569d82786cULL},
        {"vehicle_powertrain.json", 0xec3f01c51673f663ULL}, // pedal P1, was 0xaadddd7bcd24ac15
        {"vehicle_tractor_i4.json", 0xee2c08f11ee76f55ULL}, // D0 phantom wheel cast fix, was 0x44327cc4b10c57b5
        {"vehicle_fuel_tank.json", 0xb08736ed4b3a8d4fULL},
        {"vehicle_sim_engine.json", 0x92b0090ff4b70656ULL}, // pedal P1, was 0x3aed0da9057e5829
        {"vehicle_gen_engine.json", 0x82bf30120b3212bbULL}, // knock opt-in 2026-10-18, was 0x2164909873144ac3; pedal P1, was 0x194fc20db754f52c
        {"environment_day.json", 0xce55f12949e0ee73ULL},
    };
    for (const Pin& p : pins) {
        for (const int workers : {1, 10}) {
            const std::uint64_t h = run_scenario_file(p.file, workers);
            INFO(p.file << " workers=" << workers << " hash=0x" << std::hex << h << " pinned=0x" << p.hash);
            CHECK(h == p.hash);
        }
    }
}

// Weather plan W4: data/scenarios/environment_weather.json - car_sedan_sim (simulated engine, live ambient)
// driving under dynamic weather (x3000 clock, 800 m above the station), per-vehicle ISA air, gusts on the
// sim clock. The same file belongs in tools/ci.ps1's Windows-vs-Linux hash compare (coordinator).
TEST_CASE("W4: the weather scenario hashes identically across 1, 2, 4, 10 and 24 workers and really runs the weather",
          "[determinism][environment][vehicle][heavy][world][io]") {
    constexpr std::uint64_t kRecorded = 0x6dd1b38a446aed23ULL; // 2026-10-13 pedal plan P1 (manual pedal law T* = Z + c (F - Z) on electronic engines) moved this hash (was 0xa3ca509638ae73cc); 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0x2520d724148fa822): recorded 2026-10-05 (W4), ps_run at 1 and 10 workers
    const std::uint64_t reference = run_scenario_file("environment_weather.json", 1);
    INFO("reference hash=0x" << std::hex << reference << " recorded=0x" << kRecorded);
    CHECK(reference == kRecorded);
    for (const int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("environment_weather.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h);
        REQUIRE(h == reference);
    }

    // The weather and the gusts really ran: the vehicle's air is valid, 800 m up, and the clock moved.
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/environment_weather.json");
    sc.world_config.job_workers = 1;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    for (int i = 0; i < 480; ++i) world.step();
    const ps::environment::VehicleAir air = world.vehicle_air(built.vehicles_by_name.at("car"));
    CHECK(air.valid);
    CHECK(air.altitude_m > 900.0);
    CHECK(air.pressure_pa < 95000.0);
    CHECK(world.snapshots().read().environment.has_weather);
}

// Weather plan W5: data/scenarios/environment_surface.json - car_sedan under dynamic weather with the
// surface condition grid (data/surface_conditions/default.json on surfaces.json's classes), x36000 clock
// (two or three 60 s grid steps per tick) and a one-day backward jump at t = 5 s (a spin-up inside a tick).
// The grid is folded into the hash (installed with surfaces only). Belongs in tools/ci.ps1's
// Windows-vs-Linux hash compare (coordinator).
TEST_CASE("W5: the surface condition scenario hashes identically across 1, 2, 4, 10 and 24 workers and really steps the grid",
          "[determinism][environment][vehicle][heavy][world][io]") {
    constexpr std::uint64_t kRecorded = 0xa413eb2036d292e8ULL; // 2026-10-13 pedal plan P1 (manual pedal law T* = Z + c (F - Z) on electronic engines) moved this hash (was 0xcef127a9c2643925); 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0x64326439851d8dfc): re-recorded 2026-10-05 (W6: water film/drain/soil state hashed, wet grip active; was 0x030df2de793ce721 at W5)
    const std::uint64_t reference = run_scenario_file("environment_surface.json", 1);
    INFO("reference hash=0x" << std::hex << reference << " recorded=0x" << kRecorded);
    CHECK(reference == kRecorded);
    for (const int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("environment_surface.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h);
        REQUIRE(h == reference);
    }

    // The grid ran: surfaces published, the grid holds the interval of the published env time.
    ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/environment_surface.json");
    sc.world_config.job_workers = 1;
    ps::io::BuiltScenario built = ps::io::build_scenario(sc);
    ps::World& world = *built.world;
    double max_water_mm = 0.0;
    double max_soil = 0.0;
    for (int i = 0; i < 480; ++i) {
        world.step();
        const ps::environment::EnvironmentSnapshot* sn = world.environment_snapshot();
        if (sn != nullptr && sn->has_surfaces)
            for (int c = 0; c < sn->surface_count; ++c) {
                max_water_mm = std::max(max_water_mm, static_cast<double>(sn->surfaces[c].water_mm));
                max_soil = std::max(max_soil, static_cast<double>(sn->surfaces[c].soil_moisture));
            }
    }
    INFO("max surface water " << max_water_mm << " mm, max soil moisture " << max_soil);
    // W6: the water state that is folded into the hash is live in this scenario (soil moisture starts at field
    // capacity; film water needs rain, which this seed/season may not produce within 2 s of sim time).
    CHECK(max_soil > 0.0);
    const ps::environment::EnvironmentSnapshot* s = world.environment_snapshot();
    REQUIRE(s != nullptr);
    CHECK(s->has_surfaces);
    CHECK(s->surface_count == 6);
    CHECK(s->surfaces[0].has_state);
    // The grid index matches the published env time (it may be negative: the climate epoch sets t = 0).
    const ps::environment::ConditionGrid* g = world.environment()->surface_conditions();
    REQUIRE(g != nullptr);
    CHECK(g->grid_index() == g->grid_index_at(s->t_env_s));
}

// Engine thermal plan T1 I7 (vault PHYS-063): data/scenarios/vehicle_thermal.json - car_sedan_sim_thermal from an
// "ambient" cold soak (15 C, k about 2.6), ignition cycle and starter, idle, WOT 1st -> 2nd. The thermal state
// (T_e, T_o, wax, play, fan) is hashed, so the hash must be worker-count invariant; and a 1e-3 K perturbation of
// the coolant temperature at the last tick (the mechanics of that tick still use last tick's k) must move it. Sabotage (Powertrain::fold_state_hash
// without the thermal fold, measured 2026-10-05): the pinned-hash CHECK and the perturbation CHECK both fail (hash
// 0xba94bad06472d25c, a.hash == b.hash).
TEST_CASE("Engine thermal: vehicle_thermal.json hashes identically across 1, 2, 4, 10 and 24 workers; T_o is hashed",
          "[determinism][thermal][vehicle][drivetrain][heavy][world][io]") {
    constexpr std::uint64_t kRecorded = 0x79bdd9ac60402967ULL; // 2026-10-16 block cooling: every shipped engine gets a water jacket (class_si_passenger.json), the A25A's metal lump is now hashed - moved this hash (was 0xfb37e62eaf06cabb); 2026-10-13 pedal plan P1 (manual pedal law T* = Z + c (F - Z) on electronic engines) moved this hash (was 0x3178e9e670d1cb5d); 2026-10-06 tyre rolling-resistance derived speed term moved this hash (was 0xd7205b0b06295bd8): re-recorded 2026-10-05 (T1b friction split, fire_rpm 100; T1 C6 was 0x74d75aa6d229faa0), ps_run at 1/2/4/10/24 workers
    const std::uint64_t reference = run_scenario_file("vehicle_thermal.json", 1);
    INFO("reference hash=0x" << std::hex << reference << " recorded=0x" << kRecorded);
    CHECK(reference == kRecorded);
    for (const int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("vehicle_thermal.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h);
        REQUIRE(h == reference);
    }

    // The same scenario's thermal model really ran, and T_o is part of the hash.
    auto run_to = [](int ticks, bool perturb) {
        ps::io::Scenario sc = ps::io::load_scenario(std::string(PS_SOURCE_DIR) + "/data/scenarios/vehicle_thermal.json");
        sc.world_config.job_workers = 1;
        ps::io::BuiltScenario built = ps::io::build_scenario(sc);
        ps::World& world = *built.world;
        const ps::VehicleId car = built.vehicles_by_name.at("car");
        for (int i = 0; i < ticks; ++i) {
            if (perturb && i == ticks - 1) { // applied at the last tick's begin; the tick's mechanics use the old k, so only the hash fold can show it
                ps::drivetrain::EngineThermalState s = world.engine_thermal_state(car);
                s.coolant_c += 1e-3;
                world.set_engine_thermal_state(car, s);
            }
            world.step();
        }
        struct Out { std::uint64_t hash; ps::real coolant_c, oil_c, k; };
        return Out{world.state_hash(), world.engine_thermal_state(car).coolant_c, world.engine_thermal_state(car).oil_c,
                   world.powertrain_state(car).thermal.friction_scale};
    };
    const auto a = run_to(600, false);
    const auto b = run_to(600, true);
    CHECK(a.k > 2.0);           // 15 C oil: the cold-friction scale is far above 1
    CHECK(a.coolant_c > 15.0);  // idle heat warmed the engine
    CHECK(a.hash != b.hash);    // the thermal state is in the hash
}

TEST_CASE("State hash survives 1000 runs of randomised worker count and scheduling grain", "[determinism][stress][jobs][world][backend]") {
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
