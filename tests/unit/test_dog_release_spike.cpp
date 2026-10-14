// tests/unit/test_dog_release_spike.cpp - D0: the one-tick driveline spike at a 1->2 dog release in
// car_hyper_n2o (V11-all with the pedal P1 part (b) prototype, tick 411: gearbox.out 15.1 -> 2067.7 rad/s,
// RR hub 4.5 -> 1264.9 rad/s, both right wheels' fz -> 0, vx 5.165 -> 1.142 m/s, roll rate -3.26 rad/s).
//
// Root cause (not the drivetrain - the tick-410 topology switch conserved momentum exactly): the RR wheel's
// CylinderShape cast in WheelInstance::measure_contact got a PHANTOM hit from Jolt's default GJK shape cast against
// the test rig's 1000 m ground box - fraction 0, "started penetrating", penetration 0.1755 m, with a true clearance
// of 0.0723 m. The wheel read full bump for one substep: xdot 69.3 m/s, Fz 542 kN, the MF tyre at ~100 x fz0
// gave Fx -5.75 MN, the hub took +1300 rad/s, the contact force kicked the chassis; the next substep the wheel
// dropped back (xdot -73.9 m/s, fz 0). The dog release only coincided with it. Fix: JoltRigidBackend::shape_cast
// re-casts a suspicious result (miss, or fraction <= 0) with the shrunken shape plus convex radius and adopts that
// re-cast's hit when it is clean (fraction > 0); every other cast is bit-identical.
//
//   S1  the exact phantom pose (bits from the V11-all trace) casts to the analytic fraction, not penetrating.
//   S2  200000 near-axis-aligned wheel casts over a 1000 m box: every fraction within 0.02 of the analytic one
//       (3 wrong without the fix: 2 fraction-0, 1 miss).
//   W1  whole car: an LC launch in 1st, a forced 1->2; at the first released tick the chassis is moved (velocity
//       kept) so that the RR wheel's substep-0 cast starts at the phantom pose; 24 ticks across the release:
//       per-tick hub omega jump, gearbox.out jump, roll rate, vx drop and rear fz bounded (bands below, written
//       before the first run); the same at 1 and 10 workers (state hash every tick); own allocation 0 in the window.
//
// Recorded sabotage (the fix reverted - re-cast removed from shape_cast): see the D0 progress notes; S1, S2 and W1
// fail.

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps;
using ps::vehicle::VehicleDesc;
namespace fs = std::filesystem;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";

const VehicleDesc& hyper_n2o() {
    static const VehicleDesc d = [] {
        static jobs::JobSystem pool(10);
        io::VehicleLoadOptions o;
        o.maps.pool = &pool;
        o.maps.cache_dir = std::string(PS_SOURCE_DIR) + "/out/d0_map_cache";
        fs::create_directories(o.maps.cache_dir);
        return io::load_vehicle_json(kData + "/vehicles/car_hyper_n2o.json", o);
    }();
    return d;
}

// The RR wheel's substep-0 cast pose at V11-all tick 411 (printed with 17 digits from the trace; the orientation is
// the chassis orientation - an unsteered wheel - and already float-representable, it comes from Jolt's body).
const Vec3 kPhantomPos{0.9890761896331286, -0.82501345212230615, 0.43631795552827701};
const Quat kPhantomRot{-2.0494110231084051e-06, -0.0024049375206232071, -1.7424210341232538e-07, 0.99999713897705078};
constexpr real kRadius = 0.364;        // car_hyper rear wheel_radius
constexpr real kHalfWidth = 0.1625;    // car_hyper rear wheel_width / 2
constexpr real kTravel = 0.105;        // car_hyper suspension travel_max

std::unique_ptr<World> make_ground_world(real half_extent) {
    WorldConfig cfg;
    cfg.job_workers = 1;
    auto w = std::make_unique<World>(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{half_extent, half_extent, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.pose.position = Vec3{0, 0, -0.5};
    w->create_body(ground);
    return w;
}

} // namespace

TEST_CASE("D0 S1: the V11-all phantom wheel pose casts cleanly against a 1000 m ground box", "[backend][vehicle]") {
    auto wp = make_ground_world(1000.0);
    World& w = *wp;
    const Vec3 sweep = -kPhantomRot.rotate(Vec3::unit_z()) * kTravel;
    const ShapeCastHit h = w.backend().shape_cast(CylinderShape{kHalfWidth, kRadius}, Pose{kPhantomPos, kPhantomRot}, sweep, kTravel);
    // [D] a rotation about the spin axis leaves the lowest point at z - r, so the cylinder reaches z = 0 after
    // (z - r)/|sweep_z| of the sweep; band 0.005 of the sweep (0.5 mm) [AS].
    const real expect = (kPhantomPos.z - kRadius) / -sweep.z;
    INFO("hit " << h.hit << " fraction " << h.fraction << " expected " << expect << " started_penetrating "
                << h.started_penetrating << " penetration " << h.penetration_depth);
    REQUIRE(h.hit);
    CHECK_FALSE(h.started_penetrating);
    CHECK(std::abs(h.fraction - expect) <= 0.005);
}

TEST_CASE("D0 S2: near-axis-aligned wheel casts over a 1000 m ground box land at the analytic fraction", "[backend][vehicle]") {
    auto wp = make_ground_world(1000.0);
    World& w = *wp;
    const CylinderShape cyl{kHalfWidth, kRadius};
    std::uint64_t s = 12345;
    auto u = [&s] {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return double(s >> 11) / double(1ULL << 53);
    };
    const int n = 200000;
    int wrong = 0, started = 0, miss = 0;
    for (int i = 0; i < n; ++i) {
        // A chassis-level wheel: yaw within +-1e-6 rad, roll within +-5e-6 rad, pitch (about the spin axis) +-0.02.
        const real yaw = 2e-6 * (u() - 0.5), roll = 1e-5 * (u() - 0.5), pitch = 0.04 * (u() - 0.5);
        Pose q;
        q.orientation = Quat::from_axis_angle(Vec3::unit_z(), yaw) * Quat::from_axis_angle(Vec3::unit_x(), roll) *
                        Quat::from_axis_angle(Vec3::unit_y(), pitch);
        const real clear = 0.005 + 0.09 * u();
        // Lowest point of the cylinder rolled about x: z - r*cos(roll) - hw*|sin(roll)|.
        q.position = Vec3{40.0 * (u() - 0.5), 40.0 * (u() - 0.5), clear + kRadius * std::cos(roll) + kHalfWidth * std::abs(std::sin(roll))};
        const ShapeCastHit h = w.backend().shape_cast(cyl, q, Vec3{0, 0, -kTravel}, kTravel);
        const real expect = clear / kTravel;
        const real got = h.hit ? h.fraction : real(1.0);
        if (std::abs(got - expect) > 0.02) { // [AS] 2.1 mm, far above GJK tolerance, far below a phantom
            ++wrong;
            if (h.hit && h.started_penetrating) ++started;
            if (!h.hit) ++miss;
        }
    }
    INFO("wrong " << wrong << " of " << n << " (started_penetrating " << started << ", miss " << miss << ")");
    CHECK(wrong == 0);
}

namespace {

struct W1Run {
    bool teleported = false;
    int teleport_tick = -1;
    std::vector<std::uint64_t> hashes;
    double max_hub_jump_ratio = 0;  // max over ticks and hubs of |d omega| / own bound
    double max_out_jump = 0;
    double max_roll_rate = 0;
    double max_vx_drop = 0;
    double min_rear_fz = 1e30;
    double max_rear_fz = 0;
    bool both_rear_low = false;
    std::uint64_t own_alloc = 0;
};

W1Run run_w1(int workers) {
    const VehicleDesc& d = hyper_n2o();
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0.0, workers);
    World& w = *rig.world;
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("assist.traction_control", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 1.0);
    w.set_control("assist.launch_control", 1.0);
    w.set_control("nitrous_arm", 1.0);
    w.set_control("brake", 1.0);
    w.set_control("throttle", 1.0);
    w.set_control("handbrake", 0.0);

    // Bands [D]: a hub's per-tick omega change is bounded by the largest tyre torque over its inertia,
    // mu_max * Fz_max * R / J / 240 Hz, with mu_max 2.0 [AS, above every shipped tyre's peak] and Fz_max 3 x the
    // static corner load (42/58 split of 1500 kg) [AS]; brakes are off and the gear is released in the window, so
    // no engine or brake torque reaches the hubs.
    const real g = 9.81, m = 1500.0;
    real hub_bound[4];
    for (int i = 0; i < 4; ++i) {
        const real fz_static = m * g * (i < 2 ? 0.42 : 0.58) * 0.5;
        hub_bound[i] = 2.0 * 3.0 * fz_static * d.wheels[i].wheel_radius / d.wheels[i].wheel_inertia / 240.0;
    }
    const ps::vehicle::WheelDesc& rr = d.wheels[3];
    const Vec3 bump_local = rr.suspension.axis.normalized() * rr.suspension.travel_max + rr.attachment_local;

    W1Run r;
    namespace da = jobs::debug_alloc;
    real prev_hub[4] = {0, 0, 0, 0};
    real prev_out = 0, prev_vx = 0;
    const int n_window = 24;
    for (int k = 0; k < 600; ++k) {
        if (k == 144) w.set_control("brake", 0.0);
        if (k == 400) w.set_control("gear_request", 2.0);
        const auto& pt0 = w.powertrain_state(rig.vehicle);
        const int phase = static_cast<int>(pt0.shift_phase);
        // Dogs released: Selecting (3) or Synchronising (4).
        if (!r.teleported && k > 400 && (phase == 3 || phase == 4)) {
            Pose p;
            p.orientation = kPhantomRot;
            p.position = kPhantomPos - kPhantomRot.rotate(bump_local);
            w.backend().set_pose(rig.chassis, p);
            r.teleported = true;
            r.teleport_tick = k;
            if (da::hook_available()) {
                da::reset();
                da::set_armed(true);
            }
        }
        w.step();
        r.hashes.push_back(w.state_hash());
        const auto& pt = w.powertrain_state(rig.vehicle);
        const Motion mo = w.get_motion(rig.chassis);
        if (r.teleported && k < r.teleport_tick + n_window) {
            for (int i = 0; i < 4; ++i)
                r.max_hub_jump_ratio = std::max(r.max_hub_jump_ratio, double(std::abs(w.wheel_omega(rig.vehicle, i) - prev_hub[i]) / hub_bound[i]));
            r.max_out_jump = std::max(r.max_out_jump, double(std::abs(pt.driveline.gearbox_out_omega - prev_out)));
            r.max_roll_rate = std::max(r.max_roll_rate, double(std::abs(mo.angular.x)));
            r.max_vx_drop = std::max(r.max_vx_drop, double(prev_vx - mo.linear.x));
            const real fzl = w.wheel_telemetry(rig.vehicle, 2).fz, fzr = w.wheel_telemetry(rig.vehicle, 3).fz;
            r.min_rear_fz = std::min(r.min_rear_fz, double(std::min(fzl, fzr)));
            r.max_rear_fz = std::max(r.max_rear_fz, double(std::max(fzl, fzr)));
            const real low = 0.25 * m * g * 0.58 * 0.5;
            if (fzl < low && fzr < low) r.both_rear_low = true;
        }
        if (r.teleported && k == r.teleport_tick + n_window - 1) {
            if (da::hook_available()) {
                da::set_armed(false);
                r.own_alloc = da::own_violation_count();
            }
            break;
        }
        for (int i = 0; i < 4; ++i) prev_hub[i] = w.wheel_omega(rig.vehicle, i);
        prev_out = pt.driveline.gearbox_out_omega;
        prev_vx = mo.linear.x;
    }
    return r;
}

} // namespace

TEST_CASE("D0 W1: whole car - the phantom wheel pose at the 1->2 dog release leaves the car undisturbed", "[vehicle][drivetrain][backend]") {
    const W1Run a = run_w1(1);
    const W1Run b = run_w1(10);
    INFO("teleport tick " << a.teleport_tick << " hub jump / bound " << a.max_hub_jump_ratio << " gearbox.out jump " << a.max_out_jump
                          << " roll rate " << a.max_roll_rate << " vx drop " << a.max_vx_drop << " rear fz min " << a.min_rear_fz
                          << " max " << a.max_rear_fz << " both rear low " << a.both_rear_low << " own alloc " << a.own_alloc);
    REQUIRE(a.teleported);
    CHECK(a.max_hub_jump_ratio <= 1.0);
    // [D] gearbox.out = final drive 3.35 x the carrier, the carrier = the mean rear hub: 3.35 x the rear hub bound.
    const VehicleDesc& d = hyper_n2o();
    const real rear_bound = 2.0 * 3.0 * 1500.0 * 9.81 * 0.58 * 0.5 * d.wheels[3].wheel_radius / d.wheels[3].wheel_inertia / 240.0;
    CHECK(a.max_out_jump <= 3.35 * rear_bound);
    CHECK(a.max_roll_rate <= 0.5);    // [AS] a launch in 1st on flat ground; the phantom gave 3.26 rad/s
    CHECK(a.max_vx_drop <= 0.1);      // [D] 2.4 g over one 1/240 s tick; the phantom gave 4.0 m/s
    CHECK(a.min_rear_fz > 0.0);       // [D] a rear wheel on flat ground under acceleration never unloads
    CHECK(a.max_rear_fz <= 5.0 * 1500.0 * 9.81 * 0.58 * 0.5); // [AS] 5 x static; the phantom gave 542 kN
    CHECK_FALSE(a.both_rear_low);     // [AS] both rear wheels below 0.25 x static in one tick
    CHECK(a.own_alloc == 0);
    CHECK(a.teleport_tick == b.teleport_tick);
    CHECK(a.hashes == b.hashes);
}

// Hidden: V11-all's launch (LC armed, brake release at 0.6 s), auto-shift OFF, then a forced gear_request 2 at tick
// PS_D0_SHIFT_TICK (default 400); prints one line per tick from PS_D0_PRINT_FROM.
TEST_CASE("D0 diag: forced 1->2 after an LC launch", "[.][diag]") {
    auto env_int = [](const char* name, int def) {
#ifdef _WIN32
        char* buf = nullptr;
        std::size_t len = 0;
        if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return def;
        const int v = std::atoi(buf);
        std::free(buf);
        return v;
#else
        const char* v = std::getenv(name);
        return v ? std::atoi(v) : def;
#endif
    };
    auto rig = ps_test::build_vehicle_rig_from_desc(hyper_n2o(), 1500, {2, .4, .15}, .5, 0.0, 1);
    World& w = *rig.world;
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("assist.traction_control", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 1.0);
    const int shift_tick = env_int("PS_D0_SHIFT_TICK", 400);
    const int print_from = env_int("PS_D0_PRINT_FROM", 0);
    const int n = env_int("PS_D0_TICKS", 600);
    w.set_control("assist.launch_control", 1.0);
    w.set_control("nitrous_arm", 1.0);
    w.set_control("brake", 1.0);
    w.set_control("throttle", 1.0);
    w.set_control("handbrake", 0.0);
    double max_jump = 0, prev_out = 0;
    for (int k = 0; k < n; ++k) {
        if (k == 144) w.set_control("brake", 0.0);
        if (k == shift_tick) w.set_control("gear_request", 2.0);
        w.step();
        const auto& pt = w.powertrain_state(rig.vehicle);
        const Motion m = w.get_motion(rig.chassis);
        const double out = pt.driveline.gearbox_out_omega;
        if (k > 0) max_jump = std::max(max_jump, std::abs(out - prev_out));
        prev_out = out;
        if (k >= print_from)
            std::printf("k %4d v %6.3f gear %d tgt %d ph %d rpm %6.0f out %8.2f hubs %7.2f %7.2f %7.2f %7.2f fz %6.0f %6.0f %6.0f %6.0f wx %6.2f\n", k,
                        m.linear.x, pt.gear, pt.gear_target, static_cast<int>(pt.shift_phase), pt.engine_rpm, out, w.wheel_omega(rig.vehicle, 0),
                        w.wheel_omega(rig.vehicle, 1), w.wheel_omega(rig.vehicle, 2), w.wheel_omega(rig.vehicle, 3), w.wheel_telemetry(rig.vehicle, 0).fz,
                        w.wheel_telemetry(rig.vehicle, 1).fz, w.wheel_telemetry(rig.vehicle, 2).fz, w.wheel_telemetry(rig.vehicle, 3).fz, m.angular.x);
    }
    std::printf("D0 max |d gearbox_out| per tick %.2f rad/s\n", max_jump);
}
