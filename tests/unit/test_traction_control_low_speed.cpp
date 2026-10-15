// Traction control at crawl speed (owner report, racing_game 435498f: "TC cuts all acceleration at 5-6 km/h").
//
// Root cause: the TC speed estimate was propagated with the IMU while the pedal was held and re-anchored only on coast,
// toward the slowest wheel. Any offset it picked up (locked wheels under braking dragged it to zero or below, a slide,
// a bounce, TC switched off and on with a stale IMU sample) was therefore permanent while the pedal was down. Slip is
// (v_driven - estimate) / max(2 m/s floor, |estimate|), so at crawl speed a 0.5 m/s offset reads as 25 % slip against
// an 8 % target: the controller cut to its floor and the offset alone held the cut (log: estimate 0.62 m/s vs speed
// 1.14 m/s, positive_slip 0.259, torque_fraction 0.0 for 17 s). Fix: the estimator fuses the wheels every call
// (free wheels as a measurement, braked/driven wheels as bounds, a standstill update) - step_traction_control's
// comment; the drift on the full car is in test_traction_control_drift.cpp.
//
// Bands (written before the first run): a stale estimate on a RWD car (free fronts) is back within 0.05 m/s and the
// fraction above 0.9 within 0.3 s; on an AWD car (no free wheel) within 2 s; a genuine wheelspin burst is still cut to
// below 0.1; the hypercar passes 30 km/h within 2.5 s of a full-throttle launch.
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/traction_control.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
using namespace ps;
using namespace ps::vehicle;
namespace {
std::array<TractionWheelSensor, 4> rwd_sensors(real front, real rear) {
    return {{{front, 0, 1.5, .8, false}, {front, 0, 1.5, -.8, false}, {rear, 0, -1.1, .8, true}, {rear, 0, -1.1, -.8, true}}};
}
std::array<TractionWheelSensor, 4> awd_sensors(real speed) {
    return {{{speed, 0, 1.5, .8, true}, {speed, 0, 1.5, -.8, true}, {speed, 0, -1.1, .8, true}, {speed, 0, -1.1, -.8, true}}};
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
Launch launch(int mode) {
    VehicleDesc desc = hyper();
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
    // The car rolls at 1.14 m/s with every wheel free of slip, but the estimate sits 0.52 m/s low (the logged state:
    // estimate 0.62, positive slip 0.259 against the 2 m/s floor). Returns the time until the estimate is within
    // 0.05 m/s and the fraction above 0.9 (negative: never within `seconds`).
    const auto run = [&](bool awd, real seconds) {
        TractionControlDesc d;
        TractionControlState s;
        const auto w = awd ? awd_sensors(1.14) : rwd_sensors(1.14, 1.14);
        step_traction_control(d, s, w, 0, 0, 1, 0, true, dt);
        s.estimated_speed_m_s = 0.62;
        s.torque_fraction = 0;
        s.integral = 1;
        for (int i = 0; i < int(seconds / dt); ++i) {
            const real f = step_traction_control(d, s, w, 0, 0, 1, 0, true, dt);
            if (std::abs(s.estimated_speed_m_s - 1.14) < .05 && f > .9) return (i + 1) * dt;
        }
        return real(-1);
    };
    const real rwd = run(false, 3.0), awd = run(true, 5.0);
    std::printf("TC stale estimate recovery rwd=%.3f s awd=%.3f s\n", rwd, awd);
    CHECK(rwd > 0);
    CHECK(rwd < 0.3);
    CHECK(awd > 0);
    CHECK(awd < 2.0);
}

TEST_CASE("Sensor TC speed estimate ignores locked wheels under braking", "[traction_control][vehicle][unit][low_speed]") {
    // Car decelerates from 20 m/s at 8 m/s^2 with all four wheels locked (0 m/s), then the brake is released at
    // 2 m/s and the car crawls on throttle. The old estimator re-anchored toward the slowest wheel while braking.
    const real dt = .004;
    TractionControlDesc d;
    TractionControlState s;
    auto w = rwd_sensors(20, 20);
    step_traction_control(d, s, w, 0, 0, 1, 0, true, dt);
    real car = 20, worst = 0;
    while (car > 2) {
        car -= 8 * dt;
        w = rwd_sensors(0, 0);
        for (auto& x : w) x.braked = true;
        step_traction_control(d, s, w, -8, 0, 0, 1, true, dt);
        worst = std::max(worst, std::abs(s.estimated_speed_m_s - car));
    }
    w = rwd_sensors(car, car);
    for (int i = 0; i < 250; ++i) step_traction_control(d, s, w, 0, 0, .3, 0, true, dt);
    std::printf("TC braking lock worst error %.4f m/s, after crawl %.4f\n", worst, std::abs(s.estimated_speed_m_s - car));
    CHECK(worst < .01);
    CHECK(std::abs(s.estimated_speed_m_s - car) < .01);
    CHECK(s.torque_fraction == 1);
}

TEST_CASE("Sensor TC still cuts a real wheelspin burst with the wheel-fused estimate", "[traction_control][vehicle][unit][low_speed]") {
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
        const Launch r = launch(mode);
        std::printf("TC launch mode=%d t30=%.2f s v_end=%.1f m/s min_fraction(after 0.5 s)=%.3f\n", mode, r.t30, r.v_end, r.min_fraction_late);
        CHECK(r.t30 > 0);
        CHECK(r.t30 < 2.5);
        CHECK(r.v_end > 25.0);
    }
}
