// tests/unit/test_powertrain_reverse_launch.cpp - auto-clutch hill start while
// rolling BACKWARDS in a forward gear (owner ruling 2026-10-04, ManualTcu's
// reverse-rotation hold gated on the driver throttle).
//
// Ruling: "When rolling backwards without throttle, the clutch should
// disengage. When going backwards and applying throttle, the clutch should
// gradually engage while keeping the rpm above stall."
//
// Rig: car_hyper.json and car_sedan.json (1500 kg test chassis) on an uphill
// slope (kSlopeGrade 12 %; the helper's +X is DOWNhill for a positive grade, so
// the grade is passed negative to face the car uphill), handbrake on while the
// suspension settles, engine Running in 1st with the clutch open, then the
// handbrake is released and the car rolls back (~1.15 m/s^2, far above rolling
// resistance; -5 m/s after 5 s).
//
// H1 no throttle while rolling back: the clutch stays open, the engine runs.
// H2 throttle (WOT, and part throttle 0.5) applied at a measured roll-back
//    speed (2 m/s): engagement rises gradually, engine rpm never below
//    anti_stall_rpm, the roll-back stops, the car climbs, the clutch locks.
//    (0.3 and 0.4 are NOT enough for these cars on 12 %: the Launch law holds
//    the engine at ~bite_rpm with e ~ 0.03-0.2, the clutch carries too little
//    torque and the car keeps rolling back / creeps - law behaviour, not the
//    hold; recorded in the progress file, not asserted.)
// H3 throttle lifted again while still rolling back after a partial
//    engagement: the clutch opens again, no stall.
// The white-box case for the throttle gate itself is in test_powertrain_tcu.cpp
// ("Reverse-rotation hold applies only without driver throttle").

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include "vehicle_test_helpers.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace ps;
using namespace ps::drivetrain;

namespace {
constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
constexpr real kSlopeGrade = -0.12; // uphill in the car's +X
constexpr int kTickHz = 240;

struct CarSpec {
    const char* name;
    const char* json;
    real anti_stall_rpm; // the file's auto_clutch value
    real origin_height;
};
const CarSpec kHyper{"car_hyper", "/data/vehicles/car_hyper.json", 850.0, 0.5};
const CarSpec kSedan{"car_sedan", "/data/vehicles/car_sedan.json", 750.0, 0.6};

const vehicle::VehicleDesc& desc_of(const CarSpec& car) {
    // Loaded once per car: car_hyper generates its engine maps at load.
    static jobs::JobSystem pool(10);
    static io::EngineMapOptions opt = [] {
        io::EngineMapOptions o;
        o.pool = &pool;
        return o;
    }();
    static const vehicle::VehicleDesc hyper = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + kHyper.json, opt);
    static const vehicle::VehicleDesc sedan = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + kSedan.json, opt);
    return &car == &kHyper ? hyper : sedan;
}

struct Rig {
    ps_test::VehicleTestRig rig;
    explicit Rig(const CarSpec& car)
        : rig(ps_test::build_vehicle_rig_from_desc(desc_of(car), 1500.0, Vec3{2.0, 0.4, 0.15}, car.origin_height, kSlopeGrade)) {
        // Assists I2 (owner Q2): car_hyper carries the Racing assist unit (TC default On); these cases test the
        // auto-clutch's reverse-rotation hold alone, so TC is off explicitly (a no-op on car_sedan).
        rig.world->set_control("assist.traction_control", 0.0);
        rig.world->set_control("assist.aero_control", 0.0); // and the unit's wing automation (assists I5)
        rig.world->set_control("handbrake", 1.0);
        ps_test::settle(*rig.world, 1.0);
        vehicle::VehicleResetOptions reset;
        reset.gear = 1;
        reset.clutch_locked = false;
        rig.world->reset_vehicle(rig.vehicle, reset);
        rig.world->set_control("gear_request", 1.0);
        rig.world->set_control("throttle", 0.0);
        rig.world->set_control("brake", 0.0);
        rig.world->set_control("handbrake", 0.0);
    }
    // Signed speed along the car's own +X (negative = rolling back).
    real speed_x() {
        const auto m = rig.world->get_motion(rig.chassis);
        const auto q = rig.world->get_pose(rig.chassis).orientation;
        return q.conjugate().rotate(m.linear).x;
    }
    void step() { rig.world->step(); }
    PowertrainSnapshot state() { return rig.world->powertrain_state(rig.vehicle); }
    PowertrainDebug debug() { return rig.world->powertrain_debug(rig.vehicle); }
    void set_throttle(real t) { rig.world->set_control("throttle", t); }
};

struct Trace {
    real t = 0, v = 0, rpm = 0, e = 0, clutch_torque = 0, slip_rpm = 0;
    bool stuck = false;
    EngineState engine = EngineState::Off;
};

Trace sample(Rig& r, int tick) {
    const auto st = r.state();
    const auto dbg = r.debug();
    Trace tr;
    tr.t = tick / real(kTickHz);
    tr.v = r.speed_x();
    tr.rpm = st.engine_rpm;
    tr.e = st.clutch_engagement;
    tr.clutch_torque = dbg.clutch_torque;
    tr.slip_rpm = dbg.clutch_slip * kRadPerSecToRpm;
    tr.stuck = dbg.clutch_stuck;
    tr.engine = st.engine_state;
    return tr;
}

// Roll back (no throttle) until the body-x speed reaches -v_back; returns the tick count, -1 if never.
int roll_back_to(Rig& r, real v_back, int max_ticks = 6 * kTickHz) {
    for (int tick = 0; tick < max_ticks; ++tick) {
        r.step();
        if (r.speed_x() <= -v_back) return tick + 1;
    }
    return -1;
}

struct LaunchMetrics {
    real v_at_throttle = 0;
    real e_first_tick = 0;
    real t_first_engage = -1, t_e_half = -1; // seconds after the throttle step
    real max_rise_per_tick = 0;
    real min_rpm = 1e9, max_rpm = 0;
    real t_stop_rollback = -1, t_lock = -1;
    real slip_energy_j = 0, v_end = 0, v_max = -1e9;
    bool stalled = false;
};

LaunchMetrics hill_start(const CarSpec& car, real throttle, real v_back, real seconds) {
    Rig r(car);
    const int back_tick = roll_back_to(r, v_back);
    REQUIRE(back_tick > 0);
    LaunchMetrics m;
    m.v_at_throttle = r.speed_x();
    r.set_throttle(throttle);
    real prev_e = r.state().clutch_engagement;
    const int n = static_cast<int>(seconds * kTickHz);
    for (int tick = 0; tick < n; ++tick) {
        r.step();
        const Trace tr = sample(r, tick + 1);
        if (tick == 0) m.e_first_tick = tr.e;
        if (m.t_first_engage < 0 && tr.e > 0) m.t_first_engage = tr.t;
        if (m.t_e_half < 0 && tr.e >= 0.5) m.t_e_half = tr.t;
        m.max_rise_per_tick = std::max(m.max_rise_per_tick, tr.e - prev_e);
        prev_e = tr.e;
        m.min_rpm = std::min(m.min_rpm, tr.rpm);
        m.max_rpm = std::max(m.max_rpm, tr.rpm);
        m.v_max = std::max(m.v_max, tr.v);
        if (m.t_stop_rollback < 0 && tr.v >= 0) m.t_stop_rollback = tr.t;
        if (m.t_lock < 0 && tr.stuck && tr.e >= 0.999) m.t_lock = tr.t;
        m.slip_energy_j += std::abs(tr.clutch_torque * tr.slip_rpm / kRadPerSecToRpm) / kTickHz;
        if (tr.engine != EngineState::Running) m.stalled = true;
    }
    m.v_end = r.speed_x();
    return m;
}
} // namespace

TEST_CASE("Reverse launch H1: no throttle while rolling backwards keeps the clutch open", "[tcu][hyper_clutch][drivetrain][vehicle][long]") {
    for (const CarSpec* car : {&kHyper, &kSedan}) {
        INFO(car->name);
        Rig r(*car);
        real max_e = 0, min_rpm = 1e9, max_clutch_torque = 0, v_min = 0;
        for (int tick = 0; tick < 5 * kTickHz; ++tick) {
            r.step();
            const Trace tr = sample(r, tick);
            max_e = std::max(max_e, tr.e);
            min_rpm = std::min(min_rpm, tr.rpm);
            max_clutch_torque = std::max(max_clutch_torque, std::abs(tr.clutch_torque));
            v_min = std::min(v_min, tr.v);
            CHECK(tr.engine == EngineState::Running);
        }
        std::printf("H1 %s: v_min=%.2f m/s, max e=%.4f, min rpm=%.0f, max |clutch torque|=%.3f\n", car->name, v_min, max_e, min_rpm,
                    max_clutch_torque);
        CHECK(v_min < -3.0); // it really rolls back, well inside the hold's regime
        CHECK(max_e == real(0));
        CHECK(max_clutch_torque < 1e-6);
        CHECK(min_rpm > car->anti_stall_rpm);
    }
}

TEST_CASE("Reverse launch H2: throttle while rolling backwards engages gradually and climbs away", "[tcu][hyper_clutch][drivetrain][vehicle][long]") {
    for (const CarSpec* car : {&kHyper, &kSedan}) {
        for (const real throttle : {1.0, 0.5})
        for (const real v_back : {2.0, 4.0}) {
            INFO(car->name << " throttle " << throttle << " roll-back " << v_back);
            const bool wot = throttle >= 1.0;
            const LaunchMetrics m = hill_start(*car, throttle, v_back, wot ? 5.0 : 12.0);
            std::printf("H2 %s throttle %.1f: v0=%.2f first_engage=%.3f s half=%.3f s e(tick1)=%.4f max_rise/tick=%.4f rpm[%.0f,%.0f] "
                        "stop_rollback=%.3f s lock=%.3f s slipE=%.0f J v_end=%.2f v_max=%.2f\n",
                        car->name, throttle, m.v_at_throttle, m.t_first_engage, m.t_e_half, m.e_first_tick, m.max_rise_per_tick, m.min_rpm,
                        m.max_rpm, m.t_stop_rollback, m.t_lock, m.slip_energy_j, m.v_end, m.v_max);
            CHECK(m.v_at_throttle < -v_back + 0.1);
            CHECK(m.v_at_throttle > -v_back - 0.2);
            // The clutch engages (the pre-change hold kept it at 0 for the whole run)...
            CHECK(m.t_first_engage > 0.0);
            CHECK(m.t_first_engage < 1.0);
            // ... gradually: never a jump to full within one tick (the Launch law's
            // engage_rate_per_s 4/s = 0.0167 per tick; the lock hand-over's torque-
            // following close, 1/close_time_s = 10/s = 0.0417 per tick, is the bound).
            CHECK(m.e_first_tick < 0.05);
            CHECK(m.max_rise_per_tick <= 0.045);
            // The engine never goes below the anti-stall threshold and never stalls.
            CHECK(m.min_rpm >= car->anti_stall_rpm);
            CHECK_FALSE(m.stalled);
            // Roll-back stops, the car climbs, the clutch locks.
            CHECK(m.t_stop_rollback > 0.0);
            CHECK(m.t_stop_rollback < (wot ? 1.5 : 6.0));
            CHECK(m.v_max > 1.0); // (a WOT 1st-gear start of these test chassis may later spin the tyres on the slope; v_end not asserted)
            CHECK(m.t_lock > 0.0);
            CHECK(m.t_lock < (wot ? 3.0 : 12.0));
        }
    }
}

TEST_CASE("Reverse launch H3: lifting the throttle while still rolling back opens the clutch again", "[tcu][hyper_clutch][drivetrain][vehicle][long]") {
    for (const CarSpec* car : {&kHyper, &kSedan}) {
        INFO(car->name);
        Rig r(*car);
        REQUIRE(roll_back_to(r, 2.0) > 0);
        r.set_throttle(1.0);
        real e_partial = 0;
        for (int tick = 0; tick < 36; ++tick) { // 0.15 s of throttle
            r.step();
            e_partial = std::max(e_partial, r.state().clutch_engagement);
        }
        REQUIRE(r.speed_x() < -1.0); // still rolling back
        // Partial engagement reached (the pre-change hold never let it start).
        CHECK(e_partial > 0.1);
        CHECK(e_partial < 0.6);
        r.set_throttle(0.0);
        int t_open = -1;
        real max_e_after = 0, max_torque_after = 0, min_rpm = 1e9;
        bool stalled = false;
        for (int tick = 0; tick < 3 * kTickHz; ++tick) {
            r.step();
            const Trace tr = sample(r, tick + 1);
            if (t_open < 0 && tr.e == 0) t_open = tick + 1;
            if (t_open >= 0) {
                max_e_after = std::max(max_e_after, tr.e);
                max_torque_after = std::max(max_torque_after, std::abs(tr.clutch_torque));
            }
            min_rpm = std::min(min_rpm, tr.rpm);
            if (tr.engine != EngineState::Running) stalled = true;
        }
        std::printf("H3 %s: e after 0.15 s of throttle %.3f, open %d ticks (%.3f s) after the lift, min rpm %.0f\n", car->name, e_partial,
                    t_open, t_open / real(kTickHz), min_rpm);
        CHECK(t_open > 0);
        CHECK(t_open <= 14); // within 60 ms (measured: hyper 10 ticks - the Launch law opens it as the rpm falls through bite; sedan 1 tick - the hold latches)
        CHECK(max_e_after == real(0)); // and stays open while it keeps rolling back
        CHECK(max_torque_after < 1e-6);
        CHECK(r.speed_x() < -2.0);
        CHECK_FALSE(stalled);
        CHECK(min_rpm >= car->anti_stall_rpm);
    }
}
