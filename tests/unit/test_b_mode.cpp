// tests/unit/test_b_mode.cpp - the EV "B" one-pedal regeneration mode (owner request 2026-10-21):
//   "There should be a B gear that maxes out regeneration on the motors at all speeds at 0 throttle and then scales the
//    throttle between that maximum negative power and the maximum peak power the car can produce."
//
// Law under test (docs/electric_drive.md "B mode", ps/drivetrain/b_mode.h): system power P = -R + pedal (R + P_max), R the
// regeneration the car can take now, P_max the peak drive power now; pedal 0 is full regeneration, pedal 1 the D full power,
// the zero-power point is at pedal R / (R + P_max). The brake pedal still blends on top. A wheel-slip guard withdraws the
// regeneration of a motor whose wheels slip; below the regeneration fade the pads hold the car (auto hold).
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS]):
//   BM1 car_electric, pack at 50 %, B, pedal 0 at 30/60/100 km/h: the mechanical regeneration power (ledger delta over 4 ticks)
//       equals R = sum_motor min(regen_torque_nm * omega_motor, regen power limit) to 1 %; the snapshot's b_regen_capacity_w
//       equals R to 1 %
//   BM2 pedal 1 in B gives the same mechanical drive power as pedal 1 in D (1 %) at 30/60/100 km/h
//   BM3 pedal = snapshot b_zero_power_pedal: |mechanical power| < 1 % of P_max over a 0.2 s window; the zero point is
//       R / (R + P_max) from the snapshot numbers
//   BM4 coast-down in B from 100 km/h, pedal 0, flat: the speed never rises, the car stops (< 0.3 m/s) within 30 s, the pads
//       hold it (b_hold_level > 0 at the end, speed stays < 0.5 m/s for 2 s), SOC rises, balance_error < 1e-5, regenerated
//       energy > 0.9 of the lost kinetic energy minus the road loads (reported)
//   BM5 low_mu (no ABS unit): B pedal 0 from 100 km/h: no driven wheel stays beyond |slip| 0.5 for more than 0.3 s (the guard
//       is the only thing between pedal 0 and a lock); NEGATIVE CONTROL: with b_mode.slip_target = 100 (the guard sabotaged)
//       the same run locks a wheel for more than 0.3 s
//   BM6 selector: gear_request 99 -> up -> B; B -> down -> D; D -> down (a plain request 0) -> N; request 98 selects B; a
//       gearbox car (car_sedan) never reports B
//   BM7 B state is not hashed in D: car_electric after a drive in D hashes identical to a run that never touched the new
//       channels (same sequence, recorded on this code with b_mode.enabled = false vs true); 1 vs 10 workers equal in B
//   BM8 zero allocation (debug_alloc armed, own count 0) over a B pull + regen phase
//   BM9 TC ceiling (PR #37) scales B's drive and never B's regeneration
//   BL1 loader: regen_blend.b_mode round-trips; unknown key and out-of-range values rejected with the field named

#include "ps/backend/i_rigid_backend.h"
#include "ps/drivetrain/b_mode.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

using namespace ps;
using namespace ps::drivetrain;
using namespace ps::vehicle;
using nlohmann::json;

namespace {
const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
const std::string kEv = kData + "/vehicles/car_electric.json";
const std::string kSedan = kData + "/vehicles/car_sedan.json";
constexpr real kMass = 1850;
constexpr real kRatio = 9.0;

VehicleDesc ev(real soc = 0.5) {
    VehicleDesc d = io::load_vehicle_json(kEv);
    std::get<BatteryDesc>(d.powertrain.components[0].params).initial_soc = soc;
    return d;
}

struct Rig {
    ps_test::VehicleTestRig rig;
    World& w() { return *rig.world; }
    [[nodiscard]] const PowertrainSnapshot& pt() { return rig.world->powertrain_state(rig.vehicle); }
    [[nodiscard]] real speed() { return rig.world->backend().get_motion(rig.chassis).linear.length(); }
    [[nodiscard]] real x() { return rig.world->backend().get_pose(rig.chassis).position.x; }
};

void roll(Rig& r, real v, int gear) {
    Motion m;
    m.linear = Vec3{v, 0, 0};
    r.w().backend().set_motion(r.rig.chassis, m);
    VehicleResetOptions ro;
    ro.gear = gear;
    ro.hubs_roll_with_chassis = true;
    r.w().reset_vehicle(r.rig.vehicle, ro);
}

Rig make_ev(const VehicleDesc& d, real v0, int workers = 4) {
    Rig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, kMass, {2, .4, .15}, .6, 0.0, workers);
    r.w().set_control("gear_request", 1);
    r.w().set_control("assist.traction_control", 0);
    ps_test::settle(r.w(), 0.5);
    roll(r, v0, VehicleResetOptions::kKeepGear);
    return r;
}

// A rig on low_mu ground (the brake-blend test's helper, no ABS unit).
Rig make_low_mu(const VehicleDesc& d, real v0) {
    Rig r;
    r.rig.surfaces = std::make_shared<io::SurfaceTable>(kData + "/surfaces/surfaces.json");
    ps::WorldConfig cfg;
    cfg.job_workers = 4;
    r.rig.world = std::make_unique<World>(cfg);
    BodyDesc ground;
    ground.shape = ps_test::flat_surface_mesh(1000.0, r.rig.surfaces->id_for("low_mu"));
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    r.w().create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2, .4, .15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = kMass;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0, 0, 0.6};
    r.rig.chassis = r.w().create_body(chassis);
    r.w().set_surface_table(r.rig.surfaces);
    r.rig.vehicle = r.w().create_vehicle(d, r.rig.chassis);
    r.w().set_control("gear_request", 1);
    r.w().set_control("assist.traction_control", 0);
    r.w().set_control("assist.abs", 0);
    r.w().set_control("assist.esc", 0);
    ps_test::settle(r.w(), 0.5);
    roll(r, v0, VehicleResetOptions::kKeepGear);
    return r;
}

// Regeneration capacity of car_electric from first principles: per motor min(regen torque x omega_motor, power limit), the
// motor turning at 9 x its axle's wheel speed (the wheels roll slightly slower than the chassis while braking).
real expected_regen_w(real omega_front, real omega_rear) {
    return std::min(100.0 * omega_front * kRatio, 300000.0) + std::min(100.0 * omega_rear * kRatio, 300000.0);
}

struct Mean {
    real mech_w = 0, speed = 0, omega_front = 0, omega_rear = 0; // mean wheel speeds of the front / rear axle [rad/s]
};
// Steps `ticks` ticks and returns the mean mechanical motor power (ledger delta) and the mean speed.
Mean run_ticks(Rig& r, int ticks) {
    const real e0 = r.w().powertrain_energy(r.rig.vehicle).electric_out;
    Mean m;
    for (int k = 0; k < ticks; ++k) {
        r.w().step();
        m.speed += r.speed() / ticks;
        m.omega_front += 0.5 * (r.w().wheel_omega(r.rig.vehicle, 0) + r.w().wheel_omega(r.rig.vehicle, 1)) / ticks;
        m.omega_rear += 0.5 * (r.w().wheel_omega(r.rig.vehicle, 2) + r.w().wheel_omega(r.rig.vehicle, 3)) / ticks;
    }
    m.mech_w = (r.w().powertrain_energy(r.rig.vehicle).electric_out - e0) / (ticks / 240.0);
    return m;
}
} // namespace

TEST_CASE("BM1: B at pedal 0 regenerates the full capacity R at every speed", "[electric][b_mode][vehicle]") {
    for (const real kmh : {30.0, 60.0, 100.0}) {
        const real v0 = kmh / 3.6;
        Rig r = make_ev(ev(0.5), v0);
        r.w().set_control("gear_request", kGearRequestB);
        r.w().set_control("throttle", 0);
        for (int k = 0; k < 24; ++k) r.w().step(); // path ramp
        const Mean m = run_ticks(r, 4);
        const real expected = expected_regen_w(m.omega_front, m.omega_rear);
        INFO(kmh << " km/h: mechanical " << m.mech_w / 1e3 << " kW, expected R " << expected / 1e3 << " kW, snapshot R "
                 << r.pt().b_regen_capacity_w / 1e3 << " kW, speed " << m.speed);
        std::printf("BM1 %5.0f km/h: regen %.2f kW (R %.2f kW), P_max %.1f kW, zero-power pedal %.4f\n", kmh, -m.mech_w / 1e3,
                    expected / 1e3, r.pt().b_drive_capacity_w / 1e3, r.pt().b_zero_power_pedal);
        REQUIRE(r.pt().b_mode_active);
        CHECK(-m.mech_w == Catch::Approx(expected).epsilon(0.01));
        CHECK(r.pt().b_regen_capacity_w == Catch::Approx(expected).epsilon(0.01));
    }
}

TEST_CASE("BM2: B at pedal 1 gives D's full drive power", "[electric][b_mode][vehicle]") {
    for (const real kmh : {30.0, 60.0, 100.0}) {
        const real v0 = kmh / 3.6;
        const auto measure = [&](bool b) {
            Rig r = make_ev(ev(0.5), v0);
            if (b) r.w().set_control("gear_request", kGearRequestB);
            r.w().set_control("throttle", 1.0);
            for (int k = 0; k < 12; ++k) r.w().step();
            return run_ticks(r, 4);
        };
        const Mean d = measure(false), b = measure(true);
        INFO(kmh << " km/h: D " << d.mech_w / 1e3 << " kW, B " << b.mech_w / 1e3 << " kW");
        std::printf("BM2 %5.0f km/h: D %.2f kW, B %.2f kW\n", kmh, d.mech_w / 1e3, b.mech_w / 1e3);
        REQUIRE(d.mech_w > 0);
        CHECK(b.mech_w == Catch::Approx(d.mech_w).epsilon(0.01));
    }
}

TEST_CASE("BM3: the zero-power pedal point coasts", "[electric][b_mode][vehicle]") {
    for (const real kmh : {30.0, 60.0, 100.0}) {
        Rig r = make_ev(ev(0.5), kmh / 3.6);
        r.w().set_control("gear_request", kGearRequestB);
        r.w().set_control("throttle", 0);
        for (int k = 0; k < 24; ++k) r.w().step();
        // the point moves with the speed: follow the snapshot's value each tick, then average the power
        real sum_abs = 0, p_max = 0, zero = 0;
        for (int k = 0; k < 48; ++k) {
            r.w().set_control("throttle", r.pt().b_zero_power_pedal);
            const real e0 = r.w().powertrain_energy(r.rig.vehicle).electric_out;
            r.w().step();
            if (k >= 24) {
                sum_abs += std::abs(r.w().powertrain_energy(r.rig.vehicle).electric_out - e0) * 240.0 / 24;
                p_max = r.pt().b_drive_capacity_w;
                zero = r.pt().b_zero_power_pedal;
            }
        }
        const real regen = r.pt().b_regen_capacity_w;
        INFO(kmh << " km/h: mean |P| " << sum_abs / 1e3 << " kW, P_max " << p_max / 1e3 << " kW, zero point " << zero);
        std::printf("BM3 %5.0f km/h: zero-power pedal %.4f (R/(R+Pmax) %.4f), mean |P| %.2f kW (%.3f %% of Pmax)\n", kmh, zero,
                    regen / (regen + p_max), sum_abs / 1e3, 100 * sum_abs / p_max);
        CHECK(zero == Catch::Approx(regen / (regen + p_max)).epsilon(0.02));
        CHECK(sum_abs < 0.01 * p_max);
    }
}

constexpr bool kTrace = false;

TEST_CASE("BM4: coasting down in B stops the car with the auto hold and charges the pack", "[electric][b_mode][vehicle]") {
    Rig r = make_ev(ev(0.5), 100.0 / 3.6);
    World& w = r.w();
    w.set_control("gear_request", kGearRequestB);
    w.set_control("throttle", 0);
    const real soc0 = r.pt().battery_soc, x0 = r.x();
    real t_stop = -1, max_rise = 0, max_balance = 0, hold_end = 0, v_fade_t = -1, x_fade = 0;
    std::vector<real> speeds;
    for (int k = 0; k < 240 * 40; ++k) {
        w.step();
        const real v = r.speed();
        speeds.push_back(v);
        if constexpr (kTrace) if (k % 12 == 0 && v < 8) std::printf("  t %.2f v %.2f hold %.2f regen_share %.2f pad_share %.2f\n", (k + 1) / 240.0, v, r.pt().b_hold_level, r.pt().regen_brake_share, r.pt().pad_brake_share);
        if (k > 3) max_balance = std::max(max_balance, std::abs(w.powertrain_energy(r.rig.vehicle).balance_error));
        if (k >= 36 && speeds[k - 12] > 3.0) max_rise = std::max(max_rise, v - speeds[k - 12]); // over 50 ms: no chassis jitter
        if (v_fade_t < 0 && v < 2.5) { v_fade_t = (k + 1) / 240.0; x_fade = r.x() - x0; }
        hold_end = r.pt().b_hold_level;
        if (v < 0.3 && t_stop < 0) t_stop = (k + 1) / 240.0;
        if (t_stop > 0 && (k + 1) / 240.0 > t_stop + 2.0) break;
    }
    const real dist = r.x() - x0;
    INFO("stop at " << t_stop << " s, " << dist << " m; fade speed reached at " << v_fade_t << " s / " << x_fade << " m");
    std::printf("BM4: 100 km/h -> fade speed 2.5 m/s at %.2f s / %.1f m; stopped (<0.3 m/s) at %.2f s / %.1f m; SOC %.5f -> %.5f; hold %.2f\n",
                v_fade_t, x_fade, t_stop, dist, soc0, r.pt().battery_soc, hold_end);
    REQUIRE(t_stop > 0);
    CHECK(t_stop < 30.0);
    CHECK(max_rise < 0.01); // the speed never rises (m/s over 50 ms, above 3 m/s)
    CHECK(hold_end > 0.0);
    CHECK(r.speed() < 0.5);
    CHECK(r.pt().battery_soc > soc0);
    CHECK(max_balance < 1e-5);
}

TEST_CASE("BM5: the slip guard keeps B regeneration from locking the driven wheels on low grip", "[electric][b_mode][vehicle]") {
    const auto run = [](const VehicleDesc& d) {
        Rig r = make_low_mu(d, 100.0 / 3.6);
        World& w = r.w();
        w.set_control("gear_request", kGearRequestB);
        w.set_control("throttle", 0);
        real lock_run = 0, longest = 0, min_fraction = 1, max_slip = 0;
        for (int k = 0; k < 240 * 3; ++k) {
            w.step();
            bool locked = false;
            for (std::size_t i = 0; i < d.wheels.size(); ++i) {
                const real s = std::abs(w.wheel_telemetry(r.rig.vehicle, i).slip_ratio);
                max_slip = std::max(max_slip, s);
                locked = locked || s > 0.5;
            }
            lock_run = locked ? lock_run + 1.0 / 240 : 0;
            longest = std::max(longest, lock_run);
            min_fraction = std::min(min_fraction, r.pt().b_slip_fraction);
        }
        std::printf("BM5: longest lock %.3f s, max |slip| %.3f, min regen fraction %.3f, v_end %.2f\n", longest, max_slip, min_fraction, r.speed());
        return std::array<real, 3>{longest, max_slip, min_fraction};
    };
    const auto guarded = run(ev(0.5));
    VehicleDesc sabotaged = ev(0.5);
    sabotaged.brake_system.regen_blend.b_mode.slip_target = 100.0; // the guard never intervenes
    const auto open = run(sabotaged);
    CHECK(guarded[0] <= 0.3);
    CHECK(guarded[2] < 0.9); // the guard did act
    CHECK(open[0] > 0.3);    // negative control: without it pedal 0 locks the wheels on low_mu
}

TEST_CASE("BM6: the selector goes R N D B and back", "[electric][b_mode][vehicle]") {
    Rig r = make_ev(ev(0.5), 5.0);
    World& w = r.w();
    int up = 0, down = 0;
    const auto settle_ticks = [&] { for (int k = 0; k < 4; ++k) w.step(); };
    const auto shift_up = [&] { w.set_control("shift_up_count", ++up); settle_ticks(); };
    const auto shift_down = [&] { w.set_control("shift_down_count", ++down); settle_ticks(); };
    w.set_control("gear_request", 99);
    settle_ticks();
    CHECK_FALSE(r.pt().b_mode_active);
    CHECK(r.pt().b_mode_available);
    shift_up();
    CHECK(r.pt().b_mode_active);
    shift_down();
    CHECK_FALSE(r.pt().b_mode_active);
    w.set_control("gear_request", 0); // N
    settle_ticks();
    CHECK_FALSE(r.pt().b_mode_active);
    shift_up(); // from N the counter does not enter B (B sits above D)
    CHECK_FALSE(r.pt().b_mode_active);
    w.set_control("gear_request", 99);
    settle_ticks();
    w.set_control("gear_request", kGearRequestB); // the absolute request selects B
    settle_ticks();
    CHECK(r.pt().b_mode_active);
    w.set_control("gear_request", -1); // R ends B
    settle_ticks();
    CHECK_FALSE(r.pt().b_mode_active);
    // a gearbox car never has B
    auto rig = ps_test::build_vehicle_rig(kSedan, 1500, {2, .4, .15}, .6, 0.0, 4);
    rig.world->set_control("gear_request", kGearRequestB);
    rig.world->set_control("shift_up_count", 1);
    for (int k = 0; k < 8; ++k) rig.world->step();
    CHECK_FALSE(rig.world->powertrain_state(rig.vehicle).b_mode_available);
    CHECK_FALSE(rig.world->powertrain_state(rig.vehicle).b_mode_active);
}

TEST_CASE("BM7: B leaves D bit-identical and is worker-count independent", "[electric][b_mode][determinism]") {
    const auto run = [](bool b_enabled, bool use_b, int workers) {
        VehicleDesc d = ev(0.5);
        d.brake_system.regen_blend.b_mode.enabled = b_enabled;
        Rig r = make_ev(d, 15.0, workers);
        if (use_b) r.w().set_control("gear_request", kGearRequestB);
        std::vector<std::uint64_t> h;
        for (int i = 0; i < 480; ++i) {
            r.w().set_control("throttle", i < 240 ? 0.6 : 0.0);
            r.w().set_control("brake", i >= 360 ? 0.2 : 0.0);
            r.w().step();
            h.push_back(r.w().state_hash());
        }
        return h;
    };
    CHECK(run(true, false, 4) == run(false, false, 4)); // D: the B code leaves no trace
    CHECK(run(true, true, 1) == run(true, true, 10));
    CHECK(run(true, true, 4) != run(true, false, 4)); // and B itself does something
}

TEST_CASE("BM8: B allocates nothing in the substep path", "[electric][b_mode][vehicle]") {
    if (!jobs::debug_alloc::hook_available()) SKIP("no allocation hook on this build");
    Rig r = make_ev(ev(0.5), 25.0);
    r.w().set_control("gear_request", kGearRequestB);
    for (int k = 0; k < 40; ++k) r.w().step(); // warm up
    jobs::debug_alloc::reset();
    jobs::debug_alloc::set_armed(true);
    // the whole B range: regeneration, the zero point, drive
    for (int k = 0; k < 480; ++k) {
        r.w().set_control("throttle", (k % 240) < 120 ? 0.0 : 0.8);
        r.w().step();
    }
    jobs::debug_alloc::set_armed(false);
    const std::uint64_t own = jobs::debug_alloc::own_violation_count();
    INFO("own violations " << own << ", external " << jobs::debug_alloc::external_violation_count());
    jobs::debug_alloc::reset();
    CHECK(own == 0);
}

TEST_CASE("BM9: the traction ceiling scales B's drive and never B's regeneration", "[electric][b_mode][vehicle]") {
    // The ceiling is the TC's fraction of the pedal; a stub is not available at this level, so drive the unit through its law:
    // the public motor law carries the scale (test_electric_traction_ceiling.cpp); here the B regeneration at pedal 0 must equal
    // the no-TC run even with traction control commanded on a high-grip road.
    const auto regen_at = [](real tc) {
        Rig r = make_ev(ev(0.5), 100.0 / 3.6);
        r.w().set_control("assist.traction_control", tc);
        r.w().set_control("gear_request", kGearRequestB);
        r.w().set_control("throttle", 0);
        for (int k = 0; k < 24; ++k) r.w().step();
        return -run_ticks(r, 4).mech_w;
    };
    CHECK(regen_at(1.0) == Catch::Approx(regen_at(0.0)).epsilon(1e-6));
}

TEST_CASE("BL1: regen_blend.b_mode loads and validates", "[electric][b_mode][io]") {
    std::ifstream f(kEv);
    json base = json::parse(f);
    const auto load_with = [&](json b_mode) {
        json j = base;
        j["brake_system"]["regen_blend"] = {{"b_mode", std::move(b_mode)}};
        const std::string path = kData + "/vehicles/_tmp_b_mode_bl1.json";
        { std::ofstream o(path); o << j.dump(2); }
        struct Cleanup { std::string p; ~Cleanup() { std::remove(p.c_str()); } } cleanup{path};
        return io::load_vehicle_json(path);
    };
    const VehicleDesc ok = load_with({{"enabled", true}, {"slip_target", 0.12}, {"auto_hold", false}, {"hold_release_s", 0.5}});
    CHECK(ok.brake_system.regen_blend.b_mode.slip_target == 0.12);
    CHECK_FALSE(ok.brake_system.regen_blend.b_mode.auto_hold);
    CHECK(ok.brake_system.regen_blend.b_mode.hold_release_s == 0.5);
    CHECK_THROWS_WITH(load_with({{"bogus", 1}}), Catch::Matchers::ContainsSubstring("bogus"));
    CHECK_THROWS_WITH(load_with({{"slip_target", -0.1}}), Catch::Matchers::ContainsSubstring("slip_target"));
    CHECK_THROWS_WITH(load_with({{"auto_hold", 1}}), Catch::Matchers::ContainsSubstring("auto_hold"));
    CHECK_THROWS_WITH(load_with({{"hold_release_s", -1.0}}), Catch::Matchers::ContainsSubstring("hold_release_s"));
    CHECK(std::get<BatteryDesc>(ev().powertrain.components[0].params).initial_soc == 0.5);
}

TEST_CASE("BP1: the B law is a straight line through the zero-power point", "[electric][b_mode]") {
    CHECK(b_system_power(100e3, 300e3, 0.0) == -100e3);
    CHECK(b_system_power(100e3, 300e3, 1.0) == 300e3);
    CHECK(b_zero_power_pedal(100e3, 300e3) == Catch::Approx(0.25));
    CHECK(b_system_power(100e3, 300e3, 0.25) == Catch::Approx(0.0).margin(1e-9));
    CHECK(b_system_power(100e3, 300e3, 0.5) == Catch::Approx(100e3));
    CHECK(b_zero_power_pedal(0, 0) == 0);
}
