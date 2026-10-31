// tests/unit/test_hyper_idle_recovery.cpp - return to idle after a free rev on a
// turbo engine (owner bug report 2026-10-06, racing_game: car_hyper_n2o "wiggles
// up to 1300 rpm multiple times, then slowly (over more than 10 s) settles around
// 1000"; the stock car_hyper did the same in a headless repro).
//
// Cause: after the lift the turbo rotors (2.3e-4 kg*m^2 each) spin down over tens
// of seconds and hold the pressure ahead of the throttle above ambient. The idle
// feed-forward plate was solved against AMBIENT supply, so it admitted too much
// air; the one-sided idle PI (owner 2026-09-29: it only adds air) cannot take it
// back, and the engine cycled between the DFCO enable (1300) and re-enable (1000)
// rpm until the rotors slowed. Fix: a turbo engine solves its idle feed-forward
// and authority cap against the turbo's current charge pressure/temperature
// (supply) and exhaust pressure while the pedal is released.
//
// R1 car_hyper (neutral, WOT from 1.0 s to the limiter, pedal released at 2.5 s):
//    after the overrun DFCO ends there is no further DFCO activation, and the
//    rpm stays within kSettleBand of idle from kSettleTime after the lift on.
// Measured 2026-10-06 (release): before the fix 1130..1290 rpm still at +19 s, DFCO re-activated every
// 0.35-0.6 s (19 re-activations, worst |dev| 350 rpm from +5 s) with the charge pressure at 3.1 bar abs at the lift and 1.2 bar at
// 22 s; with the fix 949.6..950.1 rpm from +4.5 s, no DFCO re-activation. The pre-fix code is the
// recorded sabotage (both CHECKs fail).
// The [.][diag] case prints the traces (rpm, charge/exhaust/intake pressure, turbo rpm, plate, DFCO).

#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include "vehicle_test_helpers.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace ps;
using namespace ps::drivetrain;

namespace {
constexpr int kTickHz = 240;
constexpr real kWotStart = 1.0, kLift = 2.5, kEnd = 22.0;
constexpr real kIdleRpm = 950.0; // engines/hyper_v8_one1_audio.json idle_rpm
constexpr real kSettleTime = 5.0;  // s after the lift (measured with the fix: within 0.4 rpm from 4.5 s on)
constexpr real kSettleBand = 25.0; // rpm

const vehicle::VehicleDesc& hyper_desc() {
    static jobs::JobSystem pool(10);
    static io::EngineMapOptions opt = [] {
        io::EngineMapOptions o;
        o.pool = &pool;
        return o;
    }();
    static const vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    return d;
}

struct Sample {
    real t = 0, rpm = 0, boost = 0, turbo_rpm = 0, plate = 0, p_charge = 0, p_exh = 0, p_im = 0;
    bool dfco = false;
};

template <class F>
void run_free_rev(F&& on_tick) {
    auto rig = ps_test::build_vehicle_rig_from_desc(hyper_desc(), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0);
    rig.world->set_control("brake", 0.0);
    rig.world->set_control("handbrake", 1.0);
    const int n = static_cast<int>(kEnd * kTickHz);
    for (int tick = 0; tick < n; ++tick) {
        const real t = tick / real(kTickHz);
        rig.world->set_control("throttle", (t >= kWotStart && t < kLift) ? 1.0 : 0.0);
        rig.world->step();
        const auto st = rig.world->powertrain_state(rig.vehicle);
        Sample s;
        s.t = (tick + 1) / real(kTickHz);
        s.rpm = st.engine_rpm;
        if (!st.engines.empty()) {
            const auto& e = st.engines[0];
            s.boost = e.boost_bar;
            s.turbo_rpm = e.turbo_rpm;
            s.plate = e.throttle_position;
            s.dfco = e.dfco_active;
            s.p_charge = e.p_charge_pa;
            s.p_exh = e.p_exhaust_pa;
            s.p_im = e.p_intake_pa;
        }
        on_tick(s);
    }
}
} // namespace

TEST_CASE("R1 car_hyper: a free rev in neutral returns to idle without DFCO cycling", "[drivetrain][turbo][vehicle][long]") {
    int dfco_on_after_overrun = 0;
    bool overrun_done = false, prev_dfco = false;
    real worst_dev = 0.0, max_rpm_late = 0.0, min_rpm_late = 1e9;
    real peak = 0.0;
    run_free_rev([&](const Sample& s) {
        peak = std::max(peak, s.rpm);
        if (s.t > kLift) {
            if (overrun_done && s.dfco && !prev_dfco) ++dfco_on_after_overrun;
            if (prev_dfco && !s.dfco) overrun_done = true;
            if (s.t >= kLift + kSettleTime) {
                worst_dev = std::max(worst_dev, std::abs(s.rpm - kIdleRpm));
                max_rpm_late = std::max(max_rpm_late, s.rpm);
                min_rpm_late = std::min(min_rpm_late, s.rpm);
            }
        }
        prev_dfco = s.dfco;
    });
    std::printf("R1 peak %.0f rpm; after +%.1f s: %.1f..%.1f rpm (worst |dev| %.1f); DFCO re-activations %d\n", peak, kSettleTime,
                min_rpm_late, max_rpm_late, worst_dev, dfco_on_after_overrun);
    REQUIRE(peak > 8000.0); // the rev really reached the limiter region
    REQUIRE(overrun_done);
    CHECK(dfco_on_after_overrun == 0);
    CHECK(worst_dev < kSettleBand);
}

TEST_CASE("R1 diag: car_hyper free-rev return-to-idle trace", "[.][diag]") {
    real next = kLift;
    run_free_rev([&](const Sample& s) {
        if (s.t + 1e-9 >= next) {
            std::printf("t %5.2f rpm %7.1f charge %7.0f exh %7.0f p_im %6.0f Pa turbo %6.0f rpm plate %.5f dfco %d\n", s.t, s.rpm, s.p_charge,
                        s.p_exh, s.p_im, s.turbo_rpm, s.plate, s.dfco ? 1 : 0);
            next += (s.t < kLift + 4.0) ? 0.1 : 0.5;
        }
    });
}
