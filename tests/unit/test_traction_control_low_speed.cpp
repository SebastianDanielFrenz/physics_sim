// Traction control at crawl speed (owner report, racing_game 435498f: "TC cuts all acceleration at 5-6 km/h").
//
// Root cause: the TC speed estimate is propagated with the IMU while the pedal is held and re-anchors only on coast.
// Any offset it picks up (a slide, a bounce, a collision: the IMU integrates a_x, not d(v.x_hat)/dt) is therefore
// permanent while the pedal is down. Slip is (v_driven - estimate) / max(2 m/s floor, |estimate|), so at crawl speed
// a 0.5 m/s offset reads as 25 % slip against an 8 % target: the controller cuts to its floor, the cut removes all
// real wheelspin, and the offset alone holds the cut for as long as the pedal is down (log: estimate 0.62 m/s vs
// speed 1.14 m/s, positive_slip 0.259, torque_fraction 0.0 for 17 s). Fix: stuck-cut escape in step_traction_control
// (TractionControlDesc::stuck_hold_s / stuck_reanchor_per_s).
//
// Bands (written before the first run): the car passes 30 km/h (8.33 m/s) within 2.5 s of a full-throttle launch
// (measured on the unchanged code: 1.3 s); the latch case recovers a torque fraction above 0.9 within 2 s after the
// 1.25 s hold; a genuine wheelspin burst is still cut to below 0.1 within 0.3 s.
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/traction_control.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cstdio>
#include <filesystem>
using namespace ps;
using namespace ps::vehicle;
namespace {
std::array<TractionWheelSensor, 4> rwd_sensors(real front, real rear) {
    return {{{front, 0, 1.5, .8, false}, {front, 0, 1.5, -.8, false}, {rear, 0, -1.1, .8, true}, {rear, 0, -1.1, -.8, true}}};
}
const VehicleDesc& hyper() {
    static const VehicleDesc d = [] {
        jobs::JobSystem pool(10);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_assists_diag_cache").string();
        std::filesystem::create_directories(opt.cache_dir);
        return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    }();
    return d;
}
// Full-throttle launch from standstill, 1st gear, auto-clutch, TC mode `mode` (1 On, 2 Relaxed, 3 Sport).
// Returns the time to 30 km/h (negative: not reached) and the lowest torque fraction after 0.5 s.
struct Launch { real t30 = -1, v_end = 0, min_fraction_late = 1; };
Launch launch(int mode, real stuck_hold_s) {
    VehicleDesc desc = hyper();
    desc.traction_control.stuck_hold_s = stuck_hold_s;
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0, 4);
    World& w = *rig.world;
    w.set_control("assist.traction_control", mode);
    w.set_control("assist.auto_clutch", 1.0);
    ps_test::settle(w, .5);
    w.set_control("gear_request", 1);
    w.set_control("throttle", 1.0);
    Launch r;
    for (int k = 0; k < 240 * 4; ++k) {
        w.step();
        const real v = w.get_motion(rig.chassis).linear.x;
        if (r.t30 < 0 && v >= 30.0 / 3.6) r.t30 = (k + 1) / 240.0;
        if (k > 120) r.min_fraction_late = std::min(r.min_fraction_late, w.powertrain_state(rig.vehicle).traction_torque_fraction);
        r.v_end = v;
    }
    return r;
}
}

TEST_CASE("Sensor TC does not hold a cut on a stale speed estimate at crawl speed", "[traction_control][vehicle][unit][low_speed]") {
    const real dt = .004;
    const auto run = [&](real hold_s, real seconds) {
        TractionControlDesc d;
        d.stuck_hold_s = hold_s;
        TractionControlState s;
        // The car rolls at 1.14 m/s with all four wheels free of slip, but the estimate sits 0.52 m/s low
        // (the logged state: estimate 0.62, positive slip 0.259 against the 2 m/s floor).
        auto w = rwd_sensors(1.14, 1.14);
        step_traction_control(d, s, w, 0, 0, 1, 0, true, dt);
        s.estimated_speed_m_s = 0.62;
        real fraction = 1;
        for (int i = 0; i < int(seconds / dt); ++i) fraction = step_traction_control(d, s, w, 0, 0, 1, 0, true, dt);
        return std::pair{fraction, s.estimated_speed_m_s};
    };
    // Control: without the escape (hold = never) the cut stays at 0 for the whole run - the reported behaviour.
    const auto latched = run(1e9, 6.0);
    CHECK(latched.first < 0.01);
    CHECK(latched.second < 0.7);
    // With the escape: cut during the hold, then the estimate is pulled to the wheels and the torque returns.
    CHECK(run(.75, .6).first < 0.05);
    const auto freed = run(.75, 3.0);
    CHECK(freed.first > 0.9);
    CHECK(freed.second > 1.0);
}

TEST_CASE("Sensor TC still cuts a real wheelspin burst despite the stuck-cut escape", "[traction_control][vehicle][unit][low_speed]") {
    TractionControlDesc d;
    TractionControlState s;
    auto w = rwd_sensors(0, 0);
    step_traction_control(d, s, w, 0, 0, 1, 0, true, .004);
    // Rears spin up to 6 m/s while the fronts and the IMU say the car is at 1 m/s.
    for (int i = 0; i < 250; ++i) {
        const real car = i * .004 * 1.0;
        w = rwd_sensors(car, 6.0);
        step_traction_control(d, s, w, 1.0, 0, 1, 0, true, .004);
    }
    CHECK(s.torque_fraction < 0.1);
    CHECK(s.intervening);
}

TEST_CASE("Hypercar with TC on launches from standstill through 30 km/h", "[traction_control][vehicle][low_speed]") {
    for (int mode : {1, 2, 3}) {
        const Launch r = launch(mode, TractionControlDesc{}.stuck_hold_s);
        std::printf("TC launch mode=%d t30=%.2f s v_end=%.1f m/s min_fraction(after 0.5 s)=%.3f\n", mode, r.t30, r.v_end, r.min_fraction_late);
        CHECK(r.t30 > 0);
        CHECK(r.t30 < 2.5);
        CHECK(r.v_end > 25.0);
    }
}
