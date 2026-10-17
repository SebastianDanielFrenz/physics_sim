#ifdef _MSC_VER
// Diagnostic environment reads do not modify the environment.
#define _CRT_SECURE_NO_WARNINGS
#endif
// racing_game PR #5 report "TC is still broken (low speed / standstill, and not enough wheelspin prevention at
// ~180 km/h)". Drives the full car_hyper (Racing assist unit, auto-clutch, auto-shift) through full- and part-throttle
// launches, launch-control launches in every drive mode and a full-throttle pull through 180 km/h, and reports the true
// driven-wheel slip (fastest rear wheel against the chassis velocity at the contact patch), TC's own slip, estimate,
// torque fraction, gear, rpm. The [tc_repro] cases are hidden diagnostics:
//   ps_unit_tests "[tc_repro]" -s        (PS_TC_REPRO_TRACE=1 for the per-0.05 s trace, PS_TC_REPRO_ONLY=<substring>)
// The last case is the regression test with the bounds.
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
using namespace ps;
using namespace ps::vehicle;
namespace {
const VehicleDesc& hyper_repro() {
    static const VehicleDesc d = [] {
        jobs::JobSystem pool(4);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_assists_diag_cache").string();
        std::filesystem::create_directories(opt.cache_dir);
        return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    }();
    return d;
}
struct Sample { real t, vx, rear, front, est, tc_slip, true_slip, frac; int gear; real rpm, clutch; };
struct RunStats {
    real t30 = -1, t60 = -1, t100 = -1, v_end = 0;
    real slip_peak = 0, slip_mean = 0, frac_min = 1, frac_mean = 0, cut_time = 0; // over the scored window
    real stall_time = 0; // pedal down, car below 2 m/s, fraction < 0.2
    int n = 0;
    real band_peak = 0, band_mean = 0, band_frac_min = 1, band_over = 0; int band_n = 0; int band_gear_lo = 99, band_gear_hi = -99; // 150..210 km/h
};
// tc < 0: leave the channel at the drive mode's default. throttle(t, vx). Scored window [score_from, end].
RunStats run(const char* name, int drive_mode, real tc, real seconds, real score_from,
             const std::function<real(real, real)>& throttle, real start_speed = 0, real lc_hold_s = 0) {
    if (const char* only = std::getenv("PS_TC_REPRO_ONLY"); only && !std::strstr(name, only)) return {};
    VehicleDesc desc = hyper_repro();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .12}, .5, 0, 4, Vec3::unit_y(), 240, 960, 6000);
    World& w = *rig.world;
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("handbrake", 1.0);
    ps_test::settle(w, .4);
    // a drive-mode CHANGE (as in the game) applies the mode's TC default
    w.set_control("assist.drive_mode", drive_mode);
    ps_test::settle(w, .1);
    if (start_speed <= 0 && tc >= 0) w.set_control("assist.traction_control", tc);
    w.set_control("handbrake", 0.0);
    if (lc_hold_s > 0) {
        w.set_control("brake", 1.0);
        w.set_control("throttle", 1.0);
        for (int i = 0; i < int(lc_hold_s * 240); ++i) w.step();
        w.set_control("brake", 0.0);
    }
    // no gear_request: auto-shift selects 1st from N and shifts up (gear_request would pin the gear)
    const real contact_z = desc.wheels[0].attachment_local.z - desc.wheels[0].wheel_radius;
    const bool trace = std::getenv("PS_TC_REPRO_TRACE") != nullptr;
    // Optional run-up to start_speed (full throttle, TC On), then 1.5 s held at that speed; not scored. The TC mode
    // under test is selected at the end of the hold.
    if (start_speed > 0) {
        const real saved_tc = w.get_control("assist.traction_control");
        w.set_control("assist.traction_control", 1.0);
        for (int k = 0; k < 240 * 30; ++k) {
            const Pose p = w.get_pose(rig.chassis);
            const real vx = p.orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x;
            if (vx >= start_speed - 2) break;
            w.set_control("throttle", 1.0);
            w.step();
        }
        for (int k = 0; k < 360; ++k) {
            const Pose p = w.get_pose(rig.chassis);
            const real vx = p.orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x;
            w.set_control("throttle", std::clamp(real(0.35) + (start_speed - vx) * real(0.1), real(0), real(0.8)));
            w.step();
        }
        w.set_control("assist.traction_control", tc >= 0 ? tc : saved_tc);
        std::printf("  %s run-up done: v=%.1f km/h gear=%d rpm=%.0f\n", name,
                    w.get_pose(rig.chassis).orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x * 3.6,
                    int(w.powertrain_state(rig.vehicle).gear), w.powertrain_state(rig.vehicle).engine_rpm);
    }
    RunStats r;
    int tcm = -1;
    const int ticks = int(seconds * 240);
    for (int k = 0; k < ticks; ++k) {
        const real t = (k + 1) / 240.0;
        const Pose p0 = w.get_pose(rig.chassis);
        const real vx0 = p0.orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x;
        const real thr = throttle(t, vx0);
        w.set_control("throttle", thr);
        w.step();
        const auto pt = w.powertrain_state(rig.vehicle);
        const auto snap_rpm = pt.engine_rpm;
        tcm = pt.traction_mode_effective;
        const Pose p = w.get_pose(rig.chassis);
        const Vec3 omega_body = p.orientation.inverse().rotate(w.get_motion(rig.chassis).angular);
        const real vx = p.orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x + omega_body.y * contact_z;
        const real rear = .5 * (w.wheel_omega(rig.vehicle, 2) * desc.wheels[2].wheel_radius + w.wheel_omega(rig.vehicle, 3) * desc.wheels[3].wheel_radius);
        const real rmax = std::max(w.wheel_omega(rig.vehicle, 2) * desc.wheels[2].wheel_radius, w.wheel_omega(rig.vehicle, 3) * desc.wheels[3].wheel_radius);
        const real front = .5 * (w.wheel_omega(rig.vehicle, 0) * desc.wheels[0].wheel_radius + w.wheel_omega(rig.vehicle, 1) * desc.wheels[1].wheel_radius);
        const real true_slip = (rmax - vx) / std::max(real(2), std::abs(vx));
        Sample s{t, vx, rear, front, pt.traction_estimated_speed_m_s, pt.traction_positive_slip, true_slip, pt.traction_torque_fraction,
                 pt.gear, snap_rpm, pt.clutch_engagement};
        if (r.t30 < 0 && vx >= 30 / 3.6) r.t30 = t;
        if (r.t60 < 0 && vx >= 60 / 3.6) r.t60 = t;
        if (r.t100 < 0 && vx >= 100 / 3.6) r.t100 = t;
        if (t >= score_from && thr > .05) {
            r.slip_peak = std::max(r.slip_peak, true_slip);
            r.slip_mean += true_slip;
            r.frac_min = std::min(r.frac_min, s.frac);
            r.frac_mean += s.frac;
            if (s.frac < .5) r.cut_time += 1.0 / 240;
            ++r.n;
        }
        if (thr > .05 && vx < 2 && s.frac < .2) r.stall_time += 1.0 / 240;
        if (thr > .95 && vx >= 150 / 3.6 && vx <= 210 / 3.6) {
            r.band_peak = std::max(r.band_peak, true_slip); r.band_mean += true_slip; ++r.band_n;
            r.band_frac_min = std::min(r.band_frac_min, s.frac);
            if (true_slip > .15) r.band_over += 1.0 / 240;
            r.band_gear_lo = std::min(r.band_gear_lo, s.gear); r.band_gear_hi = std::max(r.band_gear_hi, s.gear);
        }
        if (trace && k % 12 == 0)
            std::printf("  %s t=%.2f vx=%.2f rear=%.2f front=%.2f est=%.2f tcslip=%.3f slip=%.3f frac=%.3f g=%d rpm=%.0f cl=%.2f thr=%.2f lc=%d\n", name,
                        t, vx, rear, front, s.est, s.tc_slip, true_slip, s.frac, s.gear, s.rpm, s.clutch, thr, int(pt.launch_phase));
        r.v_end = vx;
    }
    if (r.n) { r.slip_mean /= r.n; r.frac_mean /= r.n; }
    if (r.band_n) {
        r.band_mean /= r.band_n;
        std::printf("TCBAND  %-24s tc=%d 150-210 km/h gears %d-%d: slip peak=%.3f mean=%.3f t(slip>0.15)=%.2fs frac min=%.3f\n", name, tcm,
                    r.band_gear_lo, r.band_gear_hi, r.band_peak, r.band_mean, r.band_over, r.band_frac_min);
    }
    std::printf("TCREPRO %-24s tc=%d t30=%.2f t60=%.2f t100=%.2f v_end=%.1f km/h | slip peak=%.3f mean=%.3f | frac min=%.3f mean=%.3f cut<0.5=%.2fs | crawl_cut=%.2fs\n",
                name, tcm, r.t30, r.t60, r.t100, r.v_end * 3.6, r.slip_peak, r.slip_mean, r.frac_min, r.frac_mean, r.cut_time, r.stall_time);
    return r;
}
}

TEST_CASE("TC owner repro: launches from standstill", "[.][tc_repro][launch]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "full_%s_default", modes[m]);
        run(n, m, -1, 5, .3, [](real, real) { return real(1); });
        std::snprintf(n, sizeof n, "full_%s_tcoff", modes[m]);
        run(n, m, 0, 5, .3, [](real, real) { return real(1); });
        std::snprintf(n, sizeof n, "part30_%s_default", modes[m]);
        run(n, m, -1, 5, .3, [](real, real) { return real(.3); });
        std::snprintf(n, sizeof n, "crawl15_%s_default", modes[m]);
        run(n, m, -1, 6, .3, [](real, real) { return real(.15); });
    }
    for (int tc : {1, 2, 3, 4}) {
        char n[64];
        std::snprintf(n, sizeof n, "full_normal_tc%d", tc);
        run(n, 1, tc, 5, .3, [](real, real) { return real(1); });
    }
}

TEST_CASE("TC owner repro: launch control", "[.][tc_repro][lc]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "lc_%s_default", modes[m]);
        run(n, m, -1, 5, .3, [](real, real) { return real(1); }, 0, 1.5);
    }
}

TEST_CASE("TC owner repro: full throttle at 180 km/h", "[.][tc_repro][highspeed]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "v180_%s_default", modes[m]);
        run(n, m, -1, 3, 0, [](real, real) { return real(1); }, 175 / 3.6);
        std::snprintf(n, sizeof n, "v180_%s_tcoff", modes[m]);
        run(n, m, 0, 3, 0, [](real, real) { return real(1); }, 175 / 3.6);
    }
}

TEST_CASE("TC owner repro: full-throttle pull through 180 km/h", "[.][tc_repro][pull]") {
    const char* modes[] = {"eco", "normal", "sport", "racing"};
    for (int m = 0; m < 4; ++m) {
        char n[64];
        std::snprintf(n, sizeof n, "pull_%s_default", modes[m]);
        run(n, m, -1, 14, .3, [](real, real) { return real(1); });
        std::snprintf(n, sizeof n, "pull_%s_tcoff", modes[m]);
        run(n, m, 0, 14, .3, [](real, real) { return real(1); });
    }
}

// Regression (not hidden) for the racing_game PR #5 report, measured on this rig (true slip of the faster rear wheel):
//   Racing drive mode (TC Sport, target 0.18), launch control from standstill: peak 2.03 before (the soft hand-over
//   ramped the clutch open-loop onto the held 5000 rpm engine at full boost), 0.24 after.
//   Racing drive mode, full-throttle pull, 150-210 km/h band: peak 0.242 before (TC Sport closed the throttle plate,
//   the boost stayed and the torque came down too slowly), 0.186 after.
//   Sport drive mode (TC Relaxed, target 0.12), launch control: peak 0.346 before, 0.177 after.
TEST_CASE("TC regression: car_hyper launch control and the 180 km/h pull keep the wheelspin bounded", "[assist][traction_control][launch_control]") {
    const auto wot = [](real, real) { return real(1); };
    const RunStats lc_racing = run("reg_lc_racing", 3, -1, 5, .3, wot, 0, 1.5);
    CHECK(lc_racing.slip_peak < 0.30);
    CHECK(lc_racing.t100 > 0);
    CHECK(lc_racing.t100 < 4.5);
    const RunStats lc_sport = run("reg_lc_sport", 2, -1, 5, .3, wot, 0, 1.5);
    CHECK(lc_sport.slip_peak < 0.25);
    const RunStats pull_racing = run("reg_pull_racing", 3, -1, 14, .3, wot);
    REQUIRE(pull_racing.band_n > 0);
    CHECK(pull_racing.band_peak < 0.22);
    CHECK(pull_racing.band_mean < 0.18);
    const RunStats pull_normal = run("reg_pull_normal", 1, -1, 14, .3, wot);
    REQUIRE(pull_normal.band_n > 0);
    CHECK(pull_normal.band_peak < 0.15);
}
