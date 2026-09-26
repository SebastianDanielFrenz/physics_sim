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
#include "ps/io/scenario.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
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
    for (int i = 0; i < ticks; ++i) {
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

TEST_CASE("State hash with two terrain interest points and suppression is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][terrain][heavy][world][io]") {
    const std::uint64_t reference = run_scenario_file("terrain_multi_interest.json", 1);
    for (int workers : {2, 4, 10, 24}) {
        const std::uint64_t h = run_scenario_file("terrain_multi_interest.json", workers);
        INFO("workers=" << workers << " hash=0x" << std::hex << h << " reference=0x" << reference);
        REQUIRE(h == reference);
    }
}

// data/scenarios/vehicle_tractor_i4.json (PHYS-033 P4 I4 C15): a single
// tractor exercising every tractor-only control channel in one 10 s run -
// hand_throttle, pto, three incremental shift_up_count steps (N->1->2->3),
// awd, the C15 diff_lock on/off, brake to a stop, then park_lock. Pins the
// literal hash recorded when the scenario file was authored (ps_run at
// 1/2/4/10 workers all agreed before this test existed).
TEST_CASE("State hash for the tractor I4 control-channel exercise scenario is identical across 1, 2, 4, 10 and 24 workers",
          "[determinism][vehicle][drivetrain][heavy][world]") {
    constexpr std::uint64_t kRecorded = 0xac386af7129b9ffeULL;
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
