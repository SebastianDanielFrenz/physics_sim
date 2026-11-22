// PHYS-1 (owner report): in the game the handbrake often does not lock the rear wheels of the hypercar.
// Diagnosis: brake_system.handbrake max_torque_nm 1600 per rear corner (the handbrake adds to the service row capacity,
// DirectTorqueBrakes::pre_solve: cap = service*corner_max + handbrake_max*handbrake) is below the rear tyre grip torque
// Fz_rear*mu*r (measured peak 1630-1815 N*m at 15-50 m/s, rising with downforce), so the rows never saturate the tyre.
//
// Rig: the shipped car on flat asphalt, rolling straight at v in a gear with the hubs rolling and the clutch locked,
// handbrake = 1 from tick 0, brake 0, assist TC/auto-shift off, auto-clutch on. "Locked" = both rear hubs
// |omega| * r < kLockSpeed while the chassis still moves faster than kStillMoving. The numbers are printed.
// Bands [AS]: lock within kLockTime at throttle 0 (the owner's case). The light-throttle case (0.2) only reports.
#include "vehicle_test_helpers.h"
#include "engine_map_test_cache.h"

#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ps;

namespace {

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
constexpr real kLockSpeed = 0.5;    // m/s rim speed
constexpr real kStillMoving = 2.0;  // m/s chassis
constexpr real kLockTime = 0.75;    // s [AS]
constexpr real kRunTime = 2.0;

struct Result {
    real lock_time = -1.0;     // first time both rear rims < kLockSpeed, -1 = never
    real speed_at_lock = 0.0;
    real fz_rear = 0.0;        // mean rear Fz over the run
    real max_fx_torque = 0.0;  // max over rears |fx| * r (grip torque actually transmitted)
    real final_rim = 0.0;
    real final_speed = 0.0;
};

Result run_case(const std::string& vehicle, real v, int gear, real throttle, real mass, real rear_radius, const char* tag) {
    io::VehicleLoadOptions o;
    o.maps = ps_test::shared_map_options();
    const vehicle::VehicleDesc desc = io::load_vehicle_json(kData + "/vehicles/" + vehicle, o);
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, mass, {2, .4, .15}, .5, 0.0, 4);
    World& w = *rig.world;
    w.set_control("assist.traction_control", 0.0);
    w.set_control("assist.aero_control", 0.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("throttle", 0.0);
    w.set_control("brake", 0.0);
    w.set_control("handbrake", 0.0);
    {
        Motion m;
        w.backend().set_motion(rig.chassis, m);
        vehicle::VehicleResetOptions ro;
        ro.gear = 0;
        w.reset_vehicle(rig.vehicle, ro);
    }
    ps_test::settle(w, 0.5);
    {
        Motion m;
        m.linear = Vec3{v, 0, 0};
        w.backend().set_motion(rig.chassis, m);
        vehicle::VehicleResetOptions ro;
        ro.gear = gear;
        ro.hubs_roll_with_chassis = true;
        ro.clutch_locked = true;
        w.reset_vehicle(rig.vehicle, ro);
    }
    w.set_control("throttle", throttle);
    w.set_control("handbrake", 1.0);
    std::size_t rl = 99, rr = 99;
    for (std::size_t i = 0; i < w.vehicle_wheel_count(rig.vehicle); ++i) {
        if (w.wheel_name(rig.vehicle, i) == "RL") rl = i;
        if (w.wheel_name(rig.vehicle, i) == "RR") rr = i;
    }
    REQUIRE(rl != 99);
    REQUIRE(rr != 99);
    Result res;
    const int ticks = static_cast<int>(kRunTime / w.dt());
    real fz_sum = 0.0;
    std::printf("[handbrake] %s v=%.0f gear=%d throttle=%.2f\n", tag, v, gear, throttle);
    for (int k = 0; k < ticks; ++k) {
        w.step();
        const real t = (k + 1) * w.dt();
        const real rim_l = std::abs(w.wheel_omega(rig.vehicle, rl)) * rear_radius;
        const real rim_r = std::abs(w.wheel_omega(rig.vehicle, rr)) * rear_radius;
        const real speed = w.get_motion(rig.chassis).linear.length();
        const auto& tl = w.wheel_telemetry(rig.vehicle, rl);
        const auto& tr = w.wheel_telemetry(rig.vehicle, rr);
        fz_sum += 0.5 * (tl.fz + tr.fz);
        res.max_fx_torque =
            std::max({res.max_fx_torque, std::abs(tl.fx) * rear_radius, std::abs(tr.fx) * rear_radius});
        if (res.lock_time < 0 && std::max(rim_l, rim_r) < kLockSpeed && speed > kStillMoving) {
            res.lock_time = t;
            res.speed_at_lock = speed;
        }
        if (k % 24 == 23)
            std::printf("  t=%.2f speed=%.2f rimL=%.2f rimR=%.2f FzL=%.0f FzR=%.0f fxL*r=%.0f\n", t, speed, rim_l,
                        rim_r, tl.fz, tr.fz, std::abs(tl.fx) * rear_radius);
        res.final_rim = std::max(rim_l, rim_r);
        res.final_speed = speed;
    }
    res.fz_rear = fz_sum / ticks;
    std::printf("[handbrake] %s v=%.0f thr=%.2f: lock_time=%.3f speed_at_lock=%.1f meanFzRear=%.0f maxFx*r=%.0f finalRim=%.2f\n",
                tag, v, throttle, res.lock_time, res.speed_at_lock, res.fz_rear, res.max_fx_torque, res.final_rim);
    return res;
}

struct Case {
    real v;
    int gear;
};

} // namespace

TEST_CASE("PHYS-1 car_hyper handbrake locks the rear wheels, throttle 0", "[vehicle][brake][long]") {
    for (const Case c : {Case{15.0, 1}, Case{30.0, 2}, Case{50.0, 4}}) {
        const Result r = run_case("car_hyper.json", c.v, c.gear, 0.0, 1500.0, 0.364, "hyper");
        CHECK(r.lock_time >= 0.0);
        CHECK(r.lock_time <= kLockTime);
    }
}

TEST_CASE("PHYS-1 car_hyper handbrake, light throttle (report only)", "[vehicle][brake][long]") {
    for (const Case c : {Case{15.0, 1}, Case{30.0, 2}, Case{50.0, 4}}) {
        const Result r = run_case("car_hyper.json", c.v, c.gear, 0.2, 1500.0, 0.364, "hyper-thr0.2");
        (void)r; // reported only: engine torque through the locked clutch is a design question (coordinator)
        SUCCEED();
    }
}

TEST_CASE("PHYS-1 car_sedan handbrake locks the rear wheels, throttle 0", "[vehicle][brake][long]") {
    for (const Case c : {Case{15.0, 2}, Case{30.0, 3}}) {
        const Result r = run_case("car_sedan.json", c.v, c.gear, 0.0, 1500.0, 0.317, "sedan");
        CHECK(r.lock_time >= 0.0);
        CHECK(r.lock_time <= kLockTime);
    }
}
