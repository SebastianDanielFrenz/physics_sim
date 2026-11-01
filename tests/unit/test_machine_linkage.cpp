// tests/unit/test_machine_linkage.cpp — the three-point linkage: the rows of
// core/src/machine/coupling_solver.cpp on their own, then a tractor carrying a mounted
// body through vehicle::System, and the PTO shaft into a mounted rotor.
//
// Sabotages, run one at a time against "[machine]~[diag]~Coupling*" on 2026-10-31, each with
// the case that caught it:
//   the angle row's lever on the wrong side of the link (+ p.y)  -> a mounted body loads the rear axle
//   the lift row two-sided                                       -> the lift only lifts; the ground may lift the implement
//   the held lift left out of the wrench                         -> an implement over the lift's rating
//   a lift that never leaves the lower stop                      -> the lift, an implement over the lift's rating
//   float keeps the lift row                                     -> float rides up a ramp
//   the rotor's inertia not on the shaft                         -> the PTO rotor's inertia
//   the rotor's work not in the ledger                           -> the PTO shaft drives the rotor
//   detach leaves the load on the shaft                          -> the PTO shaft comes off with the hitch
//   a saturated lift not flagged to the rating                   -> an implement over the lift's rating
// Tool-side sabotages are listed in test_machine_tools.cpp.

#include "ps/drivetrain/energy_ledger.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/machine/coupling.h"
#include "ps/machine/hitch_limits.h"
#include "ps/machine/linkage.h"
#include "ps/vehicle/wheel_telemetry.h"
#include "machine_test_rig.h"

#include "machine/coupling_solver.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <string>

using namespace ps;
using Catch::Approx;
using ps_test::build_mount;
using ps_test::MountCfg;
using ps_test::MountRig;

namespace {

constexpr real kG = 9.81;

const machine::LinkageDesc& tractor_lift() {
    static const vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    for (const machine::HitchPointDesc& p : d.hitch_points) {
        if (p.linkage) return *p.linkage;
    }
    throw std::logic_error("tractor.json has no linkage");
}

machine::LinkageGeometry tractor_geometry() {
    machine::LinkageGeometry g;
    g.lower_pivot_local = Vec3{-1.16, 0.0, -0.76};
    g.top_pivot_local = Vec3{-1.57, 0.0, -0.08};
    g.lower_pin_local = Vec3{1.2, 0.0, 0.0};
    g.top_pin_local = Vec3{1.12, 0.0, 0.64};
    const Vec3 lower_pin_a{-2.04, 0.0, -0.68}, top_pin_a{-2.12, 0.0, -0.04};
    g.lower_length_m = (lower_pin_a - g.lower_pivot_local).length();
    g.top_length_m = (top_pin_a - g.top_pivot_local).length();
    g.reference_angle_rad = machine::linkage_angle(lower_pin_a - g.lower_pivot_local);
    g.min_angle_rad = -0.349;
    g.max_angle_rad = 0.436;
    g.max_lift_torque_nm = 40000.0 * g.lower_length_m;
    return g;
}

machine::HitchClassDesc locked_class() {
    machine::HitchClassDesc d;
    d.name = "test";
    d.source = "test";
    d.frequency_hz = 60.0;
    d.yaw.mode = machine::HitchAxisMode::Locked;
    d.roll.mode = machine::HitchAxisMode::Locked;
    return d;
}

struct Pair {
    machine::LinkageBody a, b;
};

// A tractor that cannot move and an 1100 kg implement with its pins where the links want them.
Pair resting_pair() {
    Pair p;
    p.a.position = Vec3{0.0, 0.0, 1.28};
    p.a.inv_mass = 0.0;
    p.b.position = p.a.position + Vec3{-2.04, 0.0, -0.68} - Vec3{1.2, 0.0, 0.0};
    p.b.inv_mass = 1.0 / 1100.0;
    p.b.inv_inertia_world = Mat3::diagonal(1.0 / 300.0, 1.0 / 90.0, 1.0 / 370.0);
    return p;
}

// One substep's solve applied to the bodies' velocities.
void apply(Pair& p, const machine::SocketSolve& s, real h) {
    p.a.linear += s.force_on_a * (p.a.inv_mass * h);
    p.a.angular += (p.a.inv_inertia_world * s.torque_on_a) * h;
    p.b.linear -= s.force_on_a * (p.b.inv_mass * h);
    p.b.angular += (p.b.inv_inertia_world * s.torque_on_b) * h;
}

} // namespace

TEST_CASE("linkage rows: one substep takes each row where the implicit spring puts it", "[machine][vehicle]") {
    machine::LinkageGeometry g = tractor_geometry();
    g.max_lift_torque_nm = 1e9; // the row itself here; its limit has a case of its own
    const machine::HitchClassDesc hitch = locked_class();
    constexpr real h = 1.0 / 960.0;
    const real omega = 6.283185307179586 * hitch.frequency_hz;
    const real rate_gain = h * omega * omega + 2.0 * hitch.damping_ratio * omega;

    std::mt19937_64 rng(7);
    std::uniform_real_distribution<real> u(-1.0, 1.0);
    for (int trial = 0; trial < 200; ++trial) {
        Pair p = resting_pair();
        p.a.inv_mass = 1.0 / 6000.0;
        p.a.inv_inertia_world = Mat3::diagonal(1.0 / 4000.0, 1.0 / 9000.0, 1.0 / 9000.0);
        p.a.linear = Vec3{u(rng), u(rng), u(rng)};
        p.b.linear = p.a.linear + Vec3{u(rng), u(rng), u(rng)} * 0.1;
        p.a.angular = Vec3{u(rng), u(rng), u(rng)} * 0.2;
        p.b.angular = p.a.angular + Vec3{u(rng), u(rng), u(rng)} * 0.1;
        p.b.position += Vec3{u(rng), u(rng), u(rng)} * 0.005;

        const machine::LinkageMeasure before = machine::measure_linkage(p.a, p.b, g);
        const machine::LiftCommand lift{true, before.lift_angle + 0.02, 0.0}; // lift pulling, its row in the matrix
        const machine::LinkageSolve s = machine::solve_linkage(p.a, p.b, g, hitch, h, lift);
        REQUIRE(s.lift_acting);
        REQUIRE_FALSE(s.saturated);
        apply(p, s.wrench, h);
        const machine::LinkageMeasure after = machine::measure_linkage(p.a, p.b, g);
        for (int k = 0; k < 3; ++k) {
            const real pushed = (omega * omega * before.error[k] + rate_gain * before.error_rate[k]) * h / (1.0 + h * rate_gain);
            CHECK(after.error_rate[k] == Approx(before.error_rate[k] - pushed).margin(1e-9));
        }
        const real pushed = (omega * omega * (before.lift_angle - lift.target_rad) + rate_gain * before.lift_rate) * h / (1.0 + h * rate_gain);
        CHECK(after.lift_rate == Approx(before.lift_rate - pushed).margin(1e-9));
    }
}

TEST_CASE("linkage rows: the lift only lifts, and stops at its force", "[machine][vehicle]") {
    machine::LinkageGeometry g = tractor_geometry();
    const machine::HitchClassDesc hitch = locked_class();
    constexpr real h = 1.0 / 960.0;
    Pair p = resting_pair();
    p.b.linear = Vec3{0.0, 0.0, -kG * h};
    const real angle = machine::measure_linkage(p.a, p.b, g).lift_angle;

    SECTION("asked to go down it lets go") {
        const machine::LinkageSolve s = machine::solve_linkage(p.a, p.b, g, hitch, h, machine::LiftCommand{true, angle - 0.1, 0.0});
        CHECK_FALSE(s.lift_acting);
        CHECK(s.lift_torque_nm == 0.0);
    }
    SECTION("float never holds anything") {
        const machine::LinkageSolve s = machine::solve_linkage(p.a, p.b, g, hitch, h, machine::LiftCommand{false, angle + 0.1, 0.0});
        CHECK_FALSE(s.lift_acting);
    }
    SECTION("past its force it holds that force and the row's work is the wrench's") {
        g.max_lift_torque_nm = 50.0;
        const machine::LinkageSolve s = machine::solve_linkage(p.a, p.b, g, hitch, h, machine::LiftCommand{true, angle + 0.1, 0.0});
        CHECK(s.saturated);
        CHECK(s.lift_acting);
        CHECK(s.lift_torque_nm == Approx(50.0));
    }
    SECTION("the upper stop takes the link whatever the lift wants") {
        g.max_angle_rad = angle + 1e-4;
        g.min_angle_rad = angle - 0.5;
        p.b.linear = Vec3{0.0, 0.0, 1.0}; // moving up through the stop
        const real rate = machine::measure_linkage(p.a, p.b, g).lift_rate;
        const machine::LinkageSolve s = machine::solve_linkage(p.a, p.b, g, hitch, h, machine::LiftCommand{true, angle + 0.3, 0.0});
        apply(p, s.wrench, h);
        CHECK(rate > 0.5);
        CHECK(machine::measure_linkage(p.a, p.b, g).lift_rate < 0.6 * rate);
        CHECK_FALSE(s.lift_acting);
    }
    SECTION("the lower stop holds a lift that cannot raise, and says what raising would take") {
        g.min_angle_rad = angle + 1e-4; // already under it
        g.max_lift_torque_nm = 50.0;
        p.b.linear = Vec3{0.0, 0.0, -0.5};
        const machine::LinkageSolve s = machine::solve_linkage(p.a, p.b, g, hitch, h, machine::LiftCommand{true, angle + 0.3, 0.0});
        CHECK(s.saturated);
        CHECK(s.lift_torque_nm > 50.0);
        apply(p, s.wrench, h);
        CHECK(p.b.linear.z > -0.5); // the stop pushed back
    }
}

TEST_CASE("a tractor lifts a mounted body", "[machine][vehicle]") {
    MountRig r = build_mount();
    r.run(1.5); // the latch, the tyres settling
    const machine::CouplingState& state = r.world->coupling_state(r.hitch);
    const machine::LinkageDesc& lift = tractor_lift();
    const real span = lift.max_angle_rad - lift.min_angle_rad;

    // The lower pins' height in the tractor's axes: the tractor itself pitches under the load, and the implement tilts,
    // because the top link is the shorter of the two.
    const auto pin_height = [&r] {
        const Vec3 pin = r.world->get_pose(r.implement_body).transform_point(Vec3{1.2, 0.0, 0.0});
        const Pose tractor = r.world->get_pose(r.tractor_body);
        return tractor.orientation.conjugate().rotate(pin - tractor.position).z;
    };
    const real z0 = pin_height();
    const real start = state.linkage.angle_rad;
    CHECK(state.latch_progress == 1.0);
    CHECK(std::abs(start) < 0.02);
    // Nothing moved by itself: the lever was left where attach found the implement.
    CHECK(r.world->get_control(r.world->vehicle_channel(r.tractor, "lift")) == Approx(-lift.min_angle_rad / span).margin(1e-6));

    // The statics of the two links: their lines meet in the instant centre, and about that the lift's force at the pin
    // balances the weight. Literal pivots and pins of tractor.json, level tractor.
    const real slope_lower = (-0.68 + 0.76) / (-2.04 + 1.16), slope_top = (-0.04 + 0.08) / (-2.12 + 1.57);
    const real x_ic = (-0.08 + 0.76 + slope_lower * -1.16 - slope_top * -1.57) / (slope_lower - slope_top);
    const real x_com = -3.24;
    const real expected = 1100.0 * kG * (x_com - x_ic) / (-2.04 - x_ic);
    printf("lift force %.1f N, instant centre at x %.1f m, statics %.1f N\n", state.linkage.lift_force_n, x_ic, expected);
    CHECK(state.linkage.lift_force_n == Approx(expected).epsilon(0.03));
    // Hanging behind the pins the implement turns away from the tractor: the top link holds it, in tension, and the
    // lower links push. Moment about the lower pins, 1.2 m of lever over 0.64 m of mast.
    const real couple = 1100.0 * kG * 1.2 / 0.64;
    CHECK(state.linkage.top_force_n == Approx(-couple).epsilon(0.03));
    CHECK(state.linkage.lower_tension_n == Approx(-couple).epsilon(0.08));

    // At the stroke rate: a quarter of the span in a quarter of the stroke, and the body comes up with the links.
    const real goal = state.linkage.position + 0.25;
    r.set("lift", goal);
    r.run(0.25 * lift.full_stroke_s);
    CHECK(state.linkage.position > goal - 0.05);
    r.run(0.5);
    CHECK(state.linkage.position == Approx(goal).margin(0.01));
    const real lifted = pin_height() - z0;
    const real link_length = (Vec3{-2.04, 0.0, -0.68} - lift.lower_pivot_local).length();
    const real rest_angle = machine::linkage_angle(Vec3{-2.04, 0.0, -0.68} - lift.lower_pivot_local);
    const real expected_lift = link_length * (std::sin(rest_angle + state.linkage.angle_rad) - std::sin(rest_angle + start));
    printf("lifted %.4f m, the links' arc %.4f m\n", lifted, expected_lift);
    CHECK(lifted == Approx(expected_lift).epsilon(0.01));
    CHECK(lifted > 0.15);

    r.set("lift", -lift.min_angle_rad / span);
    r.run(2.5);
    CHECK(pin_height() == Approx(z0).margin(0.005));
}

TEST_CASE("a mounted body loads the rear axle and unloads the front by the lever rule", "[machine][vehicle]") {
    MountCfg no_implement;
    no_implement.bare_tractor = true;
    MountRig bare = build_mount(no_implement);
    bare.run(2.0);
    const real front0 = bare.axle_load(true), rear0 = bare.axle_load(false);

    MountRig r = build_mount();
    r.set("lift", 0.8);
    r.run(4.0);
    const real weight = 1100.0 * kG;
    const real com_x = r.world->get_pose(r.implement_body).position.x - r.world->get_pose(r.tractor_body).position.x;
    const real behind_rear = -0.98 - com_x;
    const real wheelbase = 2.8;
    const real rear_expected = weight * (wheelbase + behind_rear) / wheelbase;
    const real front_expected = -weight * behind_rear / wheelbase;
    printf("rear %.0f -> %.0f N (+%.0f, lever rule +%.0f), front %.0f -> %.0f N (%.0f, lever rule %.0f)\n", rear0, r.axle_load(false),
           r.axle_load(false) - rear0, rear_expected, front0, r.axle_load(true), r.axle_load(true) - front0, front_expected);
    CHECK(r.axle_load(false) - rear0 == Approx(rear_expected).epsilon(0.02));
    CHECK(r.axle_load(true) - front0 == Approx(front_expected).epsilon(0.03));
    CHECK(r.axle_load(true) + r.axle_load(false) == Approx(front0 + rear0 + weight).epsilon(0.002));
}

TEST_CASE("lifted transport: the implement rides, pins tight, through a drive and a stop", "[machine][vehicle]") {
    MountRig r = build_mount();
    r.set("lift", 0.9);
    r.run(4.0);
    const machine::CouplingState& state = r.world->coupling_state(r.hitch);
    const real held = state.linkage.angle_rad;

    // Road gear: a lower one puts more axle torque on the tractor than the 12 kN left on its nose can hold.
    r.set("gear_request", 10.0);
    r.set("throttle", 1.0);
    real max_error = 0.0, min_front = 1e9, min_clearance = 1e9, max_speed = 0.0, min_angle = 1e9, max_angle = -1e9;
    const int ticks = 14 * 240;
    for (int i = 0; i < ticks; ++i) {
        if (i == 11 * 240) {
            r.set("throttle", 0.0);
            r.set("brake", 1.0);
        }
        r.world->step();
        if (i < 120) continue;
        max_error = std::max(max_error, state.max_error_m);
        min_front = std::min(min_front, r.axle_load(true));
        min_clearance = std::min(min_clearance, r.world->get_pose(r.implement_body).transform_point(Vec3{-1.55, 0.0, -0.6}).z);
        max_speed = std::max(max_speed, r.world->get_motion(r.tractor_body).linear.length());
        min_angle = std::min(min_angle, state.linkage.angle_rad);
        max_angle = std::max(max_angle, state.linkage.angle_rad);
    }
    printf("transport: speed %.2f m/s, pin error %.2f mm, angle %.4f..%.4f rad (held %.4f), edge clearance %.3f m, front axle min %.0f N\n", max_speed,
           max_error * 1e3, min_angle, max_angle, held, min_clearance, min_front);
    CHECK(max_speed > 3.0);
    CHECK(max_error < 0.01);
    CHECK(max_angle - min_angle < 0.03);
    CHECK(min_clearance > 0.2);
    CHECK(min_front > 200.0); // the nose stays down, if only just in the clutch-in
    CHECK(r.world->get_motion(r.tractor_body).linear.length() < 0.2);
}

TEST_CASE("float: the implement lies on what is under it and rides up a ramp", "[machine][vehicle]") {
    // The edge on a roller under the centre of mass. With the fixture's edge 2.75 m behind the pins the two links' centre
    // comes in towards the edge as they drop, the force the soil must give grows without end, and the machine noses down
    // onto the lower stop and jacks up the tractor's rear: true of this geometry, and why such a tool works in position
    // or draft and not in float.
    MountCfg cfg;
    cfg.edit_implement = [](vehicle::VehicleDesc& d) {
        d.machine->tools[0].edge_left_local.x = -0.2;
        d.machine->tools[0].edge_right_local.x = -0.2;
    };
    MountRig r = build_mount(cfg);
    const auto soil = std::make_shared<ps_test::UniformSoil>(ps_test::field_soil());
    r.world->set_soil_source(soil);
    // A strip between the rear wheels, 8 cm up over 10 m, under the edge's middle only.
    BodyDesc ramp;
    ramp.motion = BodyMotionType::Static;
    ramp.shape = BoxShape{Vec3{7.0, 0.4, 0.05}};
    ramp.pose.position = Vec3{8.0, 0.0, -0.05 + 0.04};
    ramp.pose.orientation = Quat::from_axis_angle(Vec3::unit_y(), -0.008);
    r.world->create_body(ramp);
    r.run(0.6);
    r.world->set_tool_gauge_depth(r.implement, 0, 0.05);
    r.set("lift_mode", 2.0);
    r.run(2.0);
    const machine::CouplingState& state = r.world->coupling_state(r.hitch);
    CHECK(state.linkage.mode == machine::LiftMode::Float);
    CHECK(state.linkage.lift_force_n == 0.0);
    const machine::ToolState& tool = r.world->tool_state(r.implement, 0);
    CHECK(tool.vertical_n > 0.9 * 1100.0 * kG); // the soil carries it now
    const real depth_flat = tool.depth_m;
    CHECK(depth_flat > 0.0);

    r.set("gear_request", 3.0);
    r.set("throttle", 1.0);
    real angle0 = state.linkage.angle_rad, angle1 = angle0;
    const real start_x = r.world->get_pose(r.tractor_body).position.x;
    for (int i = 0; i < 40 * 240; ++i) {
        r.world->step();
        angle1 = state.linkage.angle_rad;
        if (r.world->get_pose(r.implement_body).position.x > 13.0) break;
    }
    printf("float: depth %.3f m, link angle %.4f -> %.4f rad over the ramp, moved %.1f m, speed %.3f m/s, gear %d, %.0f rpm\n", depth_flat, angle0, angle1, r.world->get_pose(r.tractor_body).position.x - start_x,
           r.world->get_motion(r.tractor_body).linear.x, r.world->powertrain_state(r.tractor).gear, r.world->powertrain_state(r.tractor).engine_rpm);
    CHECK(r.world->get_pose(r.implement_body).position.x > 13.0);
    CHECK(angle1 - angle0 > 0.06); // 8 cm over a 0.88 m link
    CHECK(tool.depth_m > 0.0);
}

TEST_CASE("position: the ground may lift the implement, the lever is not wound away", "[machine][vehicle]") {
    MountRig r = build_mount();
    r.run(0.6);
    const machine::LinkageDesc& lift = tractor_lift();
    const real span = lift.max_angle_rad - lift.min_angle_rad;
    const machine::CouplingState& state = r.world->coupling_state(r.hitch);
    // A block rises under the implement's body while the lever asks for the bottom.
    BodyDesc block;
    block.motion = BodyMotionType::Static;
    block.shape = BoxShape{Vec3{0.3, 0.3, 0.2}};
    block.pose.position = r.world->get_pose(r.implement_body).position + Vec3{0.0, 0.0, -0.15 - 0.2 - 0.002};
    r.world->create_body(block);
    r.set("lift", 0.0);
    r.run(3.0);
    CHECK(state.linkage.angle_rad > -0.06); // it did not go through
    CHECK(state.linkage.lift_force_n == Approx(0.0).margin(50.0)); // the block carries it, the lift lets go
    const real slack = state.linkage.angle_rad - state.linkage.target_rad;
    CHECK(slack == Approx(lift.follow_slack_rad).margin(0.002));
    (void)span;
}

TEST_CASE("an implement over the lift's rating stays down, and the rating says so", "[machine][vehicle]") {
    // A weak lift rather than a heavier implement: 5.5 t hung 2.3 m behind the rear axle tips this tractor over backwards
    // before its 40 kN are used up.
    MountCfg cfg;
    cfg.edit_tractor = [](vehicle::VehicleDesc& d) {
        for (machine::HitchPointDesc& p : d.hitch_points) {
            if (p.linkage) p.linkage->max_lift_force_n = 8000.0;
        }
    };
    MountRig r = build_mount(cfg);
    r.set("lift", 1.0);
    r.run(3.0);
    const machine::CouplingState& state = r.world->coupling_state(r.hitch);
    printf("heavy: position %.3f, lift force %.0f N, saturated %d\n", state.linkage.position, state.linkage.lift_force_n, state.linkage.saturated ? 1 : 0);
    CHECK(state.linkage.saturated);
    CHECK(state.linkage.position < 0.3); // lever up, and it sank from 0.44 to where the two links ask less of it
    CHECK(state.linkage.position > 0.1); // held there by the 8 kN, off the stop and off the ground
    CHECK(state.linkage.lift_force_n == Approx(8000.0).margin(100.0));
    const auto findings = r.world->hitch_limits(r.hitch);
    const auto lift = std::find_if(findings.begin(), findings.end(), [](const drivetrain::LimitFinding& f) { return f.kind == drivetrain::LimitKind::LiftCapacity; });
    REQUIRE(lift != findings.end());
    CHECK(lift->severity == drivetrain::LimitSeverity::Exceeded);
}

TEST_CASE("the linkage refuses what does not fit, and a plain socket stays a socket", "[machine][vehicle]") {
    SECTION("a trailer has no three pins") {
        MountCfg cfg;
        cfg.attach = false;
        cfg.edit_implement = [](vehicle::VehicleDesc& d) { d.hitch_points[0].three_point = false; };
        MountRig r = build_mount(cfg);
        const machine::AttachCheck check = r.world->check_attach(r.tractor, "socket_3pt", r.implement, "socket_3pt");
        CHECK_FALSE(check.ok);
        CHECK_THAT(check.message, Catch::Matchers::ContainsSubstring("has no three pins"));
    }
    SECTION("pins out of reach are refused with the distance") {
        MountCfg cfg;
        cfg.attach = false;
        cfg.edit_implement = [](vehicle::VehicleDesc& d) {
            d.hitch_points[0].position_local.z += 0.3;
            d.hitch_points[0].lower_pin_local.z += 0.3;
            d.hitch_points[0].top_pin_local.z += 0.3;
        };
        MountRig r = build_mount(cfg);
        CHECK_FALSE(r.world->check_attach(r.tractor, "socket_3pt", r.implement, "socket_3pt").ok);
    }
    SECTION("detaching leaves the implement to the ground and the tractor free") {
        MountRig r = build_mount();
        r.run(1.0);
        const real rear = r.axle_load(false);
        r.world->detach(r.hitch);
        r.run(3.0);
        CHECK(r.axle_load(false) < rear - 10000.0);
        CHECK(r.world->get_pose(r.implement_body).position.z < 0.5);
    }
}

TEST_CASE("a linkage on the tractor file is inert until something attaches to it", "[machine][vehicle]") {
    const auto hash_after = [](bool strip) {
        MountCfg cfg;
        cfg.bare_tractor = true;
        cfg.edit_tractor = [strip](vehicle::VehicleDesc& d) {
            if (!strip) return;
            for (machine::HitchPointDesc& p : d.hitch_points) p.linkage.reset();
        };
        MountRig r = build_mount(cfg);
        r.set("gear_request", 3.0);
        r.set("throttle", 0.6);
        r.run(1.0);
        return r.world->state_hash();
    };
    CHECK(hash_after(false) == hash_after(true));
}

TEST_CASE("the lift's hash is the same on 1 and 10 workers, and the tick allocates nothing", "[machine][vehicle]") {
    const auto scenario = [](int workers, bool measure) {
        MountCfg cfg;
        cfg.workers = workers;
        MountRig r = build_mount(cfg);
        r.set("gear_request", 4.0);
        r.set("throttle", 0.7);
        r.set("lift", 0.9);
        r.run(1.0);
        if (measure) {
            if (jobs::debug_alloc::hook_available()) {
                jobs::debug_alloc::reset();
                jobs::debug_alloc::set_armed(true);
            }
        }
        r.set("lift", 0.3);
        r.run(1.0);
        if (measure && jobs::debug_alloc::hook_available()) {
            jobs::debug_alloc::set_armed(false);
            CHECK(jobs::debug_alloc::own_violation_count() == 0);
        }
        return r.world->state_hash();
    };
    const std::uint64_t one = scenario(1, false);
    CHECK(one == scenario(1, false));
    CHECK(one == scenario(10, false));
    (void)scenario(4, true);
}

// --- The PTO shaft. ---

namespace {

MountRig pto_rig() {
    MountRig r = build_mount(MountCfg{}.rotary());
    r.set("pto_speed", 1000.0);
    r.set("lift", 0.9);
    r.run(3.0);
    return r;
}

} // namespace

TEST_CASE("the PTO shaft drives the implement's rotor off the tractor's own shaft network", "[machine][vehicle]") {
    MountRig r = pto_rig();
    REQUIRE(r.world->check_pto(r.hitch).ok);
    r.world->connect_pto(r.hitch);
    CHECK(r.world->pto_connected(r.hitch));
    const machine::ToolDesc& tool = r.world->tool_desc(r.implement, 0);

    // Hand throttle up, nothing else on the engine: rated speed in the 1000 mode.
    r.set("hand_throttle", 1.0);
    r.run(2.0);
    r.set("pto", 1.0);
    real spin_time = -1.0, peak_slip = 0.0;
    const real target_rpm = 0.9 * tool.pto.nominal_rpm;
    for (int i = 0; i < 6 * 240; ++i) {
        r.world->step();
        peak_slip = std::max(peak_slip, std::abs(r.world->powertrain_debug(r.tractor).pto_clutch_slip));
        if (spin_time < 0.0 && r.world->powertrain_state(r.tractor).pto_rpm > target_rpm) spin_time = i / 240.0;
    }
    const drivetrain::PowertrainSnapshot& p = r.world->powertrain_state(r.tractor);
    printf("pto: spun up in %.3f s, %.1f rpm, engine %.1f rpm, peak slip %.1f rad/s, torque %.2f N*m\n", spin_time, p.pto_rpm, p.engine_rpm, peak_slip, r.world->tool_state(r.implement, 0).pto_torque_nm);
    REQUIRE(spin_time > 0.0);
    CHECK(peak_slip > 1.0); // the clutch took it up, the shaft did not jump
    CHECK(p.pto_rpm == Approx(p.engine_rpm / 1.95).epsilon(1e-6));
    CHECK(std::abs(r.world->powertrain_debug(r.tractor).pto_clutch_slip) < 1e-6);
    // Idle losses at rated: the part's no-load and per-metre figures, as a torque.
    const real nominal = tool.pto.nominal_rpm * 6.283185307179586 / 60.0;
    CHECK(r.world->tool_state(r.implement, 0).pto_torque_nm == Approx(machine::tool_pto_power_w(tool, 0.0, 0.0) / nominal).epsilon(1e-9));

    // The ledger books that torque at the shaft as absorbed work, to the watt.
    const real absorbed0 = r.world->powertrain_energy(r.tractor).external_absorbed;
    const int ticks = 240;
    real expected = 0.0;
    for (int i = 0; i < ticks; ++i) {
        r.world->step();
        const real omega = r.world->powertrain_state(r.tractor).pto_rpm * 6.283185307179586 / 60.0;
        expected += r.world->tool_state(r.implement, 0).pto_torque_nm * omega / 240.0;
    }
    const drivetrain::PowertrainEnergy& e = r.world->powertrain_energy(r.tractor);
    CHECK(e.external_absorbed - absorbed0 == Approx(expected).epsilon(1e-3));
    CHECK(std::abs(e.balance_error) < 1e-3);
}

TEST_CASE("the PTO rotor's inertia is the implement's, on the tractor's clutch", "[machine][vehicle]") {
    const auto spin_up = [](real inertia) {
        MountCfg cfg = MountCfg{}.rotary();
        cfg.edit_implement = [inertia](vehicle::VehicleDesc& d) { d.machine->tools[0].pto.inertia_kgm2 = inertia; };
        MountRig r = build_mount(cfg);
        r.set("pto_speed", 1000.0);
        r.run(1.0);
        r.world->connect_pto(r.hitch);
        r.set("hand_throttle", 1.0);
        r.run(1.5);
        r.set("pto", 1.0);
        for (int i = 1; i < 20 * 240; ++i) {
            r.world->step();
            if (r.world->powertrain_state(r.tractor).pto_rpm > 500.0) return i / 240.0;
        }
        return -1.0;
    };
    const real light = spin_up(2.0), heavy = spin_up(40.0);
    printf("pto spin-up to 500 rpm: %.3f s at 2 kg m2, %.3f s at 40 kg m2\n", light, heavy);
    CHECK(light > 0.0);
    CHECK(heavy > 1.5 * light);
}

TEST_CASE("the PTO shaft comes off with the hitch and refuses what the tractor cannot give", "[machine][vehicle]") {
    SECTION("detach") {
        MountRig r = pto_rig();
        r.world->connect_pto(r.hitch);
        r.set("hand_throttle", 1.0);
        r.set("pto", 1.0);
        r.run(4.0);
        const real absorbed = r.world->powertrain_energy(r.tractor).external_absorbed;
        const real rpm = r.world->powertrain_state(r.tractor).pto_rpm;
        CHECK(rpm > 800.0);
        r.world->detach(r.hitch);
        CHECK_FALSE(r.world->pto_connected(r.hitch));
        r.run(1.0);
        // The tractor's stub spins on free of the rotor: nothing absorbs, no leftover load.
        CHECK(r.world->powertrain_energy(r.tractor).external_absorbed == Approx(absorbed).margin(1e-6));
    }
    SECTION("disconnect by hand") {
        MountRig r = pto_rig();
        r.world->connect_pto(r.hitch);
        r.set("hand_throttle", 1.0);
        r.set("pto", 1.0);
        r.run(3.0);
        r.world->disconnect_pto(r.hitch);
        const real absorbed = r.world->powertrain_energy(r.tractor).external_absorbed;
        r.run(0.5);
        CHECK(r.world->powertrain_energy(r.tractor).external_absorbed == Approx(absorbed).margin(1e-6));
        CHECK(r.world->tool_state(r.implement, 0).pto_torque_nm >= 0.0);
        CHECK(r.world->check_pto(r.hitch).ok);
    }
    SECTION("a shaft mode the tractor does not have") {
        MountCfg cfg = MountCfg{}.rotary();
        cfg.edit_implement = [](vehicle::VehicleDesc& d) { d.machine->tools[0].pto.nominal_rpm = 750.0; };
        MountRig r = build_mount(cfg);
        const machine::PtoCheck check = r.world->check_pto(r.hitch);
        CHECK_FALSE(check.ok);
        CHECK_THAT(check.message, Catch::Matchers::ContainsSubstring("750"));
        CHECK_THROWS_AS(r.world->connect_pto(r.hitch), std::invalid_argument);
    }
    SECTION("a machine with no shaft") {
        MountRig r = build_mount();
        CHECK_FALSE(r.world->check_pto(r.hitch).ok);
    }
}
