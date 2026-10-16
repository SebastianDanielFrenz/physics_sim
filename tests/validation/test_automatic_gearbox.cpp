// tests/validation/test_automatic_gearbox.cpp - automatic gearbox plan A1: the torque-converter automatic
// (tests/support/fixtures/vehicles/car_sedan_at.json, data/gearboxes/sedan_8at.json) and the dual-clutch box
// (car_sedan_dct.json, data/gearboxes/sedan_7dct.json) on the reference sedan body, driven through World like a game.
//
// Automatic: creep at idle in D, the converter stall speed against an independent bisection of the engine's WOT curve
// against the pump law (n/K(0))^2, the upshift sequence from rest, lock-up at a steady cruise. DualClutch: a launch on
// clutch slip from rest, a fast upshift whose output torque never drops below the handover (no gap like a manual's
// clutch-open phase), creep. Both: the per-tick energy ledger closes (converter_loss included) and 1 vs 10 workers
// give the same state hash every tick. Measured numbers are printed by every case.

#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kMass = 1500.0;
const Vec3 kHalfExtents{2.0, 0.4, 0.15};
constexpr real kOriginHeight = 0.6;
constexpr real kD = 99.0; // gear_request: plain D (clamped to the top gear)

std::string fixture(const char* name) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/" + name; }

std::uint64_t ticks_for(real seconds, real dt) { return static_cast<std::uint64_t>(seconds / dt + 0.5); }

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) +
           std::abs(e.gear_loss) + std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) +
           std::abs(e.engine_friction_loss) + std::abs(e.engagement_loss) + std::abs(e.starter_in) +
           std::abs(e.external_reset) + std::abs(e.diff_loss) + std::abs(e.converter_loss);
}

struct Run {
    ps_test::VehicleTestRig rig;
    real max_balance_ratio = 0.0;
    bool stalled = false;
    explicit Run(const char* file, int workers = 4)
        : rig(ps_test::build_vehicle_rig(fixture(file), kMass, kHalfExtents, kOriginHeight, 0.0, workers)) {}
    World& w() { return *rig.world; }
    const drivetrain::PowertrainSnapshot& pt() { return w().powertrain_state(rig.vehicle); }
    real speed() { return w().get_motion(rig.chassis).linear.length(); }
    real fx_driven() { return w().wheel_telemetry(rig.vehicle, 0).fx + w().wheel_telemetry(rig.vehicle, 1).fx; }
    void step() {
        w().step();
        const drivetrain::PowertrainEnergy& e = w().powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) max_balance_ratio = std::max(max_balance_ratio, std::abs(e.balance_error) / denom);
        if (pt().engine_state != drivetrain::EngineState::Running) stalled = true;
    }
    void run(real seconds) {
        const std::uint64_t n = ticks_for(seconds, w().dt());
        for (std::uint64_t i = 0; i < n; ++i) step();
    }
};


} // namespace

TEST_CASE("Automatic A1: creep at idle in D, standing in D on the brake", "[validation][vehicle][drivetrain][automatic]") {
    Run r("car_sedan_at.json");
    ps_test::settle(r.w(), 1.0);
    REQUIRE(r.pt().transmission_kind == 1);
    r.w().set_control("brake", 1.0);
    r.w().set_control("gear_request", kD);
    r.run(3.0);
    const real rpm_braked = r.pt().engine_rpm;
    const real speed_braked = r.speed();
    CHECK(r.pt().gear == 1);
    CHECK(r.pt().transmission_selector == 8);
    CHECK_FALSE(r.stalled);
    r.w().set_control("brake", 0.0);
    r.run(8.0);
    const real speed_creep = r.speed();
    const real rpm_creep = r.pt().engine_rpm;
    std::printf("[automatic creep] braked rpm=%.1f speed=%.4f | creep after 8 s speed=%.3f m/s rpm=%.1f gear=%d SR=%.3f balance=%.2e\n",
                static_cast<double>(rpm_braked), static_cast<double>(speed_braked), static_cast<double>(speed_creep),
                static_cast<double>(rpm_creep), r.pt().gear, static_cast<double>(r.pt().converter_speed_ratio),
                static_cast<double>(r.max_balance_ratio));
    CHECK(speed_braked < 0.05);
    CHECK(rpm_braked > 650.0);
    CHECK(speed_creep > 1.0);  // idle creep through the converter alone, no throttle
    CHECK(speed_creep < 4.0);
    CHECK(std::abs(rpm_creep - 800.0) < 120.0);
    CHECK_FALSE(r.stalled);
    CHECK(r.max_balance_ratio <= 5e-8);
}

TEST_CASE("Automatic A1: converter stall speed vs an independent pump-law bisection", "[validation][vehicle][drivetrain][automatic]") {
    Run r("car_sedan_at.json");
    ps_test::settle(r.w(), 1.0);
    r.w().set_control("brake", 1.0);
    r.w().set_control("handbrake", 1.0);
    r.w().set_control("gear_request", kD);
    r.run(1.0);
    r.w().set_control("throttle", 1.0);
    r.run(2.5);
    real rpm_sum = 0.0;
    const std::uint64_t n = ticks_for(0.5, r.w().dt());
    for (std::uint64_t i = 0; i < n; ++i) {
        r.step();
        rpm_sum += r.pt().engine_rpm;
    }
    const real rpm = rpm_sum / static_cast<real>(n);
    // Independent reference: the loaded descs' WOT curve and K(0) (the turbine held at SR 0 by the brakes):
    // T_wot(n) = (n / K0)^2, bisected.
    const vehicle::VehicleDesc desc = io::load_vehicle_json(fixture("car_sedan_at.json"));
    const drivetrain::TorqueMapEngineDesc* eng = nullptr;
    const drivetrain::GearboxDesc* gb = nullptr;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* e = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) eng = e;
        if (const auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) gb = g;
    }
    REQUIRE(eng != nullptr);
    REQUIRE(gb != nullptr);
    REQUIRE(gb->converter.has_value());
    const real k0 = gb->converter->k_factor.y.front();
    real lo = 1000.0, hi = 6000.0;
    for (int it = 0; it < 100; ++it) {
        const real mid = real(0.5) * (lo + hi);
        const real f = eng->wot_torque_nm_vs_rpm.evaluate(mid) - (mid / k0) * (mid / k0);
        (f > 0.0 ? lo : hi) = mid;
    }
    const real ref = real(0.5) * (lo + hi);
    std::printf("[automatic stall] sim %.1f rpm, reference %.1f rpm (%.2f %%), speed %.4f m/s, turbine SR %.4f\n",
                static_cast<double>(rpm), static_cast<double>(ref), static_cast<double>(100.0 * (rpm / ref - 1.0)),
                static_cast<double>(r.speed()), static_cast<double>(r.pt().converter_speed_ratio));
    CHECK(std::abs(rpm / ref - 1.0) < 0.04);
    CHECK(r.speed() < 0.1);
    CHECK_FALSE(r.stalled);
}

TEST_CASE("Automatic A1: upshift sequence from rest, lock-up at a steady cruise, energy balance",
          "[validation][vehicle][drivetrain][automatic]") {
    for (const real throttle : {real(0.4), real(1.0)}) {
        INFO("throttle " << throttle);
        Run r("car_sedan_at.json");
        ps_test::settle(r.w(), 1.0);
        r.w().set_control("gear_request", kD);
        r.w().set_control("throttle", throttle);
        std::vector<int> gears;
        std::vector<real> shift_t;
        const std::uint64_t n = ticks_for(25.0, r.w().dt());
        bool went_down = false;
        for (std::uint64_t i = 0; i < n; ++i) {
            r.step();
            const int g = r.pt().gear;
            if (gears.empty() || g != gears.back()) {
                if (!gears.empty() && g < gears.back() && g != 0) went_down = true;
                gears.push_back(g);
                shift_t.push_back(static_cast<real>(i) * r.w().dt());
            }
        }
        std::printf("[automatic upshifts] throttle %.1f: speed %.1f m/s after 25 s, gears:", static_cast<double>(throttle),
                    static_cast<double>(r.speed()));
        for (std::size_t k = 0; k < gears.size(); ++k) std::printf(" %d@%.2fs", gears[k], static_cast<double>(shift_t[k]));
        std::printf(" | lock-up %.2f balance %.2e\n", static_cast<double>(r.pt().lockup_engagement), static_cast<double>(r.max_balance_ratio));
        CHECK_FALSE(went_down);
        CHECK(gears.back() >= 5);
        for (std::size_t k = 1; k < gears.size(); ++k) CHECK(gears[k] == gears[k - 1] + 1); // one gear at a time from rest
        CHECK_FALSE(r.stalled);
        CHECK(r.max_balance_ratio <= 5e-8);
    }

    // Lock-up: settle a cruise at part throttle, the converter locks (SR -> 1, engine == turbine).
    Run r("car_sedan_at.json");
    ps_test::settle(r.w(), 1.0);
    r.w().set_control("gear_request", kD);
    r.w().set_control("throttle", 0.5);
    r.run(12.0);
    r.w().set_control("throttle", 0.25);
    r.run(15.0);
    const auto& s = r.pt();
    std::printf("[automatic lock-up] gear %d speed %.1f m/s rpm %.0f lock-up %.3f SR %.5f\n", s.gear, static_cast<double>(r.speed()),
                static_cast<double>(s.engine_rpm), static_cast<double>(s.lockup_engagement), static_cast<double>(s.converter_speed_ratio));
    CHECK(s.gear >= 3);
    CHECK(s.lockup_engagement == 1.0);
    CHECK(std::abs(s.converter_speed_ratio - 1.0) < 1e-3);
    // Kickdown from the cruise: full throttle drops at least one gear and releases the lock-up.
    const int g0 = s.gear;
    r.w().set_control("throttle", 1.0);
    r.run(0.6);
    std::printf("[automatic kickdown] %d -> %d, lock-up %.3f\n", g0, r.pt().gear, static_cast<double>(r.pt().lockup_engagement));
    CHECK(r.pt().gear < g0);
    CHECK(r.pt().lockup_engagement < 0.5);
    CHECK_FALSE(r.stalled);
    CHECK(r.max_balance_ratio <= 5e-8);
}

TEST_CASE("Automatic A1: P R N D through gear_request and park_lock, manual mode via the shift counters",
          "[validation][vehicle][drivetrain][automatic]") {
    Run r("car_sedan_at.json");
    ps_test::settle(r.w(), 1.0);
    // P: N + park lock; throttle does not move the car.
    r.w().set_control("gear_request", 0.0);
    r.w().set_control("park_lock", 1.0);
    r.run(0.5);
    CHECK(r.pt().gear == 0);
    CHECK(r.pt().park_lock_engaged == 1.0);
    // R: rolls backwards on light throttle (release the pawl first).
    r.w().set_control("park_lock", 0.0);
    r.w().set_control("gear_request", -1.0);
    r.w().set_control("throttle", 0.3);
    r.run(3.0);
    const real vx_rev = r.w().get_motion(r.rig.chassis).linear.x;
    CHECK(r.pt().gear == -1);
    CHECK(vx_rev < -0.5);
    // N then D.
    r.w().set_control("throttle", 0.0);
    r.w().set_control("brake", 1.0);
    r.w().set_control("gear_request", 0.0);
    r.run(2.0);
    CHECK(r.pt().gear == 0);
    r.w().set_control("brake", 0.0);
    r.w().set_control("gear_request", kD);
    r.w().set_control("throttle", 0.6);
    r.run(6.0);
    const int g_auto = r.pt().gear;
    CHECK(g_auto >= 2);
    // Manual mode: two downshift presses step down two gears and hold them (no timeout set in the fixture).
    r.w().set_control("shift_down_count", 1.0);
    r.run(1.0);
    r.w().set_control("shift_down_count", 2.0);
    r.run(1.5);
    std::printf("[automatic PRND] reverse vx %.2f, D gear %d, manual after 2 down presses %d (manual mode %d)\n",
                static_cast<double>(vx_rev), g_auto, r.pt().gear, r.pt().transmission_manual_mode ? 1 : 0);
    CHECK(r.pt().transmission_manual_mode);
    CHECK(r.pt().gear <= std::max(1, g_auto - 1));
    CHECK_FALSE(r.stalled);
}

TEST_CASE("DualClutch A1: launch on clutch slip from rest, fast upshift without a torque gap, creep",
          "[validation][vehicle][drivetrain][automatic]") {
    Run r("car_sedan_dct.json");
    ps_test::settle(r.w(), 1.0);
    REQUIRE(r.pt().transmission_kind == 2);
    r.w().set_control("gear_request", kD);
    r.run(0.5); // N -> 1 engaged, creep clutch only
    r.w().set_control("throttle", 1.0);
    const real dt = r.w().dt();
    real max_launch_rpm = 0.0, t_lock = -1.0;
    real fx_before = 0.0, fx_min_shift = 1e9, t_shift_start = -1.0, t_shift_end = -1.0;
    int prev_gear = r.pt().gear;
    bool in_shift = false;
    std::vector<real> fx_hist;
    const std::uint64_t n = ticks_for(6.0, dt);
    for (std::uint64_t i = 0; i < n; ++i) {
        r.step();
        const auto& s = r.pt();
        const real t = static_cast<real>(i + 1) * dt;
        if (t_lock < 0.0) {
            max_launch_rpm = std::max(max_launch_rpm, s.engine_rpm);
            if (s.clutch_engagement >= 1.0 && s.gear == 1) t_lock = t;
        }
        fx_hist.push_back(r.fx_driven());
        // The first 1->2 shift: from the tick the shift machine leaves Idle until it is Idle again in 2nd.
        if (t_shift_start < 0.0 && prev_gear == 1 && s.shift_phase != drivetrain::ShiftPhase::Idle) {
            t_shift_start = t;
            in_shift = true;
            // mean driven Fx over the 0.1 s before the shift
            const std::size_t m = std::min<std::size_t>(fx_hist.size() - 1, ticks_for(0.1, dt));
            real sum = 0.0;
            for (std::size_t k = fx_hist.size() - 1 - m; k + 1 < fx_hist.size(); ++k) sum += fx_hist[k];
            fx_before = sum / static_cast<real>(m);
        }
        if (in_shift) {
            fx_min_shift = std::min(fx_min_shift, r.fx_driven());
            if (s.gear == 2 && s.shift_phase == drivetrain::ShiftPhase::ClutchClosing && t_shift_end < 0.0) t_shift_end = t;
            if (s.gear == 2 && s.shift_phase == drivetrain::ShiftPhase::Idle) in_shift = false;
        }
        if (s.gear != 0) prev_gear = s.gear;
    }
    const real handover = t_shift_end - t_shift_start;
    std::printf("[dct launch] max launch rpm %.0f, locked at %.3f s, speed@6s %.2f m/s, gear %d | 1->2: start %.3f s, "
                "ratio change after %.3f s, driven Fx before %.0f N, min during %.0f N (%.0f %%) | balance %.2e\n",
                static_cast<double>(max_launch_rpm), static_cast<double>(t_lock), static_cast<double>(r.speed()), r.pt().gear,
                static_cast<double>(t_shift_start), static_cast<double>(handover), static_cast<double>(fx_before),
                static_cast<double>(fx_min_shift), static_cast<double>(100.0 * fx_min_shift / fx_before),
                static_cast<double>(r.max_balance_ratio));
    CHECK_FALSE(r.stalled);
    CHECK(t_lock > 0.0);
    CHECK(t_lock < 2.5);
    // WOT launch: the clutch slips at e(rpm) = (rpm - bite)/(full - bite) until C*e meets the engine torque (350 N*m box,
    // ~180 N*m engine -> ~2300 rpm), well above the bite point and below the limiter.
    CHECK(max_launch_rpm > 1800.0);
    CHECK(max_launch_rpm < 4500.0);
    CHECK(r.speed() > 12.0);
    CHECK(r.pt().gear >= 2);
    REQUIRE(t_shift_start > 0.0);
    REQUIRE(t_shift_end > 0.0);
    CHECK(handover < 0.25);           // torque phase 0.06 s + inertia phase: tens of ms, not a manual's clutch cycle
    CHECK(fx_min_shift > 0.4 * fx_before); // no gap: the on-coming clutch carries the drive through the handover
    CHECK(r.max_balance_ratio <= 5e-8);

    // Creep: a fresh car, D, no throttle.
    Run c("car_sedan_dct.json");
    ps_test::settle(c.w(), 1.0);
    c.w().set_control("gear_request", kD);
    c.run(8.0);
    std::printf("[dct creep] speed %.3f m/s rpm %.0f e %.3f\n", static_cast<double>(c.speed()), static_cast<double>(c.pt().engine_rpm),
                static_cast<double>(c.pt().clutch_engagement));
    CHECK(c.speed() > 0.5);
    CHECK_FALSE(c.stalled);
}

TEST_CASE("Automatic/DualClutch A1: 1 vs 10 workers identical state hash every tick", "[validation][vehicle][drivetrain][automatic][determinism]") {
    for (const char* file : {"car_sedan_at.json", "car_sedan_dct.json"}) {
        INFO(file);
        Run a(file, 1), b(file, 10);
        bool same = true;
        auto drive = [&](real seconds) {
            const std::uint64_t n = ticks_for(seconds, a.w().dt());
            for (std::uint64_t i = 0; i < n; ++i) {
                a.step();
                b.step();
                if (a.w().state_hash() != b.w().state_hash()) same = false;
            }
        };
        for (World* w : {&a.w(), &b.w()}) w->set_control("gear_request", kD);
        drive(1.0);
        for (World* w : {&a.w(), &b.w()}) w->set_control("throttle", 1.0);
        drive(6.0);
        for (World* w : {&a.w(), &b.w()}) w->set_control("throttle", 0.2);
        drive(3.0);
        std::printf("[automatic determinism] %s gear %d hash 0x%016llx\n", file, a.pt().gear,
                    static_cast<unsigned long long>(a.w().state_hash()));
        CHECK(same);
    }
}
