// tests/validation/test_vehicle_standstill_wiggle.cpp — Validation: a car
// arriving at near-standstill WITH residual yaw rate and/or steering input
// settles instead of wiggling (owner-reported bug, live Godot demo: "the car
// wiggles around while on the skid pad after driving onto it and almost
// standing still").
//
// Root cause (measured, not just derived - see CLAUDE.md's vehicle-system
// section for the full writeup): step_slip_relaxation's own decay rate
// (a = vx_mag/sigma, pacejka_tyre_model.cpp) is shared by the longitudinal
// and lateral slip channels and vanishes as the wheel's ROLLING speed -> 0.
// The longitudinal channel is saved from the consequences by rolling
// resistance (a genuine Coulomb-type drag along the SAME axis as kappa's
// restoring force, still nonzero at vanishing speed - why
// test_vehicle_parking_slope.cpp already passed); the lateral channel has no
// equivalent, making the alpha_eq/chassis-yaw-rate loop a marginally-stable
// (zeta -> 0) continuous oscillator. Measured via a clean isolation (a pure
// yaw-rate kick at true standstill, no steering, no forward motion): at the
// normal 240 Hz tick rate this GROWS into a saturated ~5.4 Hz / ~0.92 rad/s
// peak-to-peak limit cycle within ~8 s, but plainly DECAYS under the
// identical initial condition at 4x finer tick rate - confirming the growth
// is a NUMERICAL instability (VehicleInstance::step_tick's one-tick-delayed
// force application turning zero continuous damping margin negative), not a
// physical one. The centred-steering case below grows identically to the
// steered case, ruling out steering as the trigger (only residual yaw rate
// is needed).
//
// Fix: wheel_instance.cpp's lateral_standstill_damping_force adds a
// low-speed-only viscous lateral force, giving the continuous system real
// damping headroom so the discretization's delay can no longer tip it
// negative - see that function's own comment for the full derivation
// (including a correction found via this file's own diagnostics: naively
// increasing the damping coefficient made things catastrophically WORSE,
// because all four wheels' damping torques sum onto the single shared yaw
// DOF, and tying the coefficient to the LIVE instantaneous wheel load
// created a positive-feedback loop during hard-acceleration weight-transfer
// transients - both found and fixed before landing here).

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ps;

namespace {
real amplitude(const std::vector<real>& v) {
    if (v.empty()) return 0.0;
    real mn = v.front(), mx = v.front();
    for (real x : v) { mn = std::min(mn, x); mx = std::max(mx, x); }
    return mx - mn;
}

// Drives the car up to speed, applies a real steer angle, then brings it to
// near-standstill by RAMPING the brake on over 1.5 s (not a teleport - the
// owner drove there) while yaw rate and lateral slip are still building.
// hold_steer is the steering command left in place once the brake releases:
// 0.3 reproduces the exact residual-steering case from the bug report; 0.0
// isolates whether steering is even necessary (it is not - see header).
void run_case(real hold_steer, real amp_bound) {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 0.5);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{10.0, 0.0, 0.0}, Vec3::zero()});
    rig.world->set_control("steer", 0.3);
    ps_test::settle(*rig.world, 0.4); // build up yaw rate / lateral slip while still moving

    rig.world->set_control("brake", 0.6);
    const real dt = rig.world->dt();
    const std::uint64_t brake_ticks = static_cast<std::uint64_t>(1.5 / dt + 0.5);
    for (std::uint64_t i = 0; i < brake_ticks; ++i) rig.world->step();

    rig.world->set_control("brake", 0.0);
    rig.world->set_control("steer", hold_steer);

    std::vector<real> window_early, window_late;
    const std::uint64_t n = static_cast<std::uint64_t>(6.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const real yaw_rate = rig.world->get_motion(rig.chassis).angular.z;
        REQUIRE(std::isfinite(yaw_rate));
        REQUIRE(std::abs(yaw_rate) < 5.0); // sanity bound - a divergent instability would blow past this quickly
        if (t >= 0.2 && t < 1.0) window_early.push_back(yaw_rate);
        if (t >= 4.0 && t < 5.0) window_late.push_back(yaw_rate);
    }

    const real amp_early = amplitude(window_early);
    const real amp_late = amplitude(window_late);
    INFO("amp_early=" << amp_early << " rad/s, amp_late=" << amp_late << " rad/s");
    CHECK(amp_late <= amp_early); // must not GROW - the actual failure mode this bug exhibited
    CHECK(amp_late < amp_bound);  // and must actually be small (settled), not just bounded
}
} // namespace

TEST_CASE("Standstill with residual yaw rate and steering settles, does not wiggle", "[validation][vehicle][long]") {
    run_case(0.3, 0.15); // steering left at the value it had when the brake released
}

TEST_CASE("Standstill with residual yaw rate, steering centred, settles", "[validation][vehicle][long]") {
    run_case(0.0, 0.05); // isolates yaw-rate-only excitation - no steering trigger needed
}

// The cleanest possible isolation of the core mechanism: no driving, no
// braking transient, no steering ever commanded - just a pure yaw-rate kick
// applied directly at true rest (vx = vy = 0 exactly). This is what actually
// caught the root cause during this bug's investigation (see header) and
// remains the most direct regression coverage for it.
TEST_CASE("Pure yaw-rate kick at true standstill decays", "[validation][vehicle][long]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0); // true rest, no steer ever commanded
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3::zero(), Vec3{0.0, 0.0, 0.3}});

    std::vector<real> window_early, window_late;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(6.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const real yaw_rate = rig.world->get_motion(rig.chassis).angular.z;
        REQUIRE(std::isfinite(yaw_rate));
        if (t >= 0.0 && t < 0.5) window_early.push_back(yaw_rate);
        if (t >= 3.0 && t < 4.0) window_late.push_back(yaw_rate);
    }

    const real amp_early = amplitude(window_early);
    const real amp_late = amplitude(window_late);
    INFO("amp_early=" << amp_early << " rad/s, amp_late=" << amp_late << " rad/s");
    CHECK(amp_late <= amp_early);
    CHECK(amp_late < 0.02); // measured ~0.0003-0.0008 with the fix; original bug saturated at ~0.92
}

// vehicle_step_steer.json (the shared cross-platform hash-compare scenario)
// runs at 20 m/s throughout, always well above kFadeSpeedMps - the new
// lateral_standstill_damping_force term never engages there and that
// scenario's hash is confirmed unchanged. Exercise the NEW code path's own
// worker-count determinism directly instead, mirroring
// tests/determinism/test_worker_count_hash.cpp's pattern: identical
// State::state_hash() at 1 vs 8 workers for the same near-standstill
// yaw-rate-kick scenario the fix targets.
TEST_CASE("Pure yaw-rate kick at standstill: state_hash matches across worker counts", "[determinism][vehicle]") {
    auto run_with_workers = [](int workers) -> std::uint64_t {
        WorldConfig cfg;
        cfg.tick_rate_hz = 240.0;
        cfg.substep_rate_hz = 960.0;
        cfg.job_workers = workers;
        World world(cfg);

        BodyDesc ground;
        ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
        ground.motion = BodyMotionType::Static;
        ground.friction = 1.0;
        ground.pose.position = Vec3{0.0, 0.0, -0.5};
        world.create_body(ground);

        BodyDesc chassis;
        chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
        chassis.motion = BodyMotionType::Dynamic;
        chassis.mass = 1500.0;
        chassis.friction = 0.5;
        chassis.allow_sleep = false;
        chassis.pose.position = Vec3{0.0, 0.0, 0.6};
        const BodyId chassis_id = world.create_body(chassis);

        auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
        world.set_surface_table(surfaces);
        const vehicle::VehicleDesc desc =
            io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
        world.create_vehicle(desc, chassis_id);

        ps_test::settle(world, 1.0);
        world.backend().set_motion(chassis_id, Motion{Vec3::zero(), Vec3{0.0, 0.0, 0.3}});
        const real dt = world.dt();
        const std::uint64_t n = static_cast<std::uint64_t>(3.0 / dt + 0.5);
        for (std::uint64_t i = 0; i < n; ++i) world.step();
        return world.state_hash();
    };

    const std::uint64_t h1 = run_with_workers(1);
    const std::uint64_t h8 = run_with_workers(8);
    INFO("hash(1 worker)=" << h1 << " hash(8 workers)=" << h8);
    CHECK(h1 == h8);
}

// --- PHYS-033 I1 plan section 4.4: sizing low_speed_damping_ratio by
// measurement, now that W1 (commit D) has removed the numerical delay this
// term used to compensate for. [.][diag]: hidden from default runs, kept as
// a re-tunable diagnostic (run by hand:
// ps_validation_tests.exe "zeta sweep*" -s) rather than deleted, in case a
// future change (e.g. the F5/F16 findings in the I1 plan's section 9) moves
// these numbers enough to need a re-sweep.
//
// OUTCOME (see tyre_desc.h's low_speed_damping_ratio comment for the full
// writeup): no yaw-mode regrowth and a fast lateral-kick decay hold at
// EVERY ratio tried, 0 through 1.5 - W1 alone already fixes the underlying
// instability this term was built to compensate for. But two existing
// acceptance tests (the full-throttle asphalt standing start's peak slip,
// and the parking-slope creep) turned out to be non-monotonic/near-chaotic
// in this ratio, each passing at some values and failing at others with no
// visible trend across [0, 1.5]. 1.5 (the commit B/pre-W1 value) was the
// only ratio tried that kept both simultaneously green, so it is RETAINED,
// not lowered - the plan's own a-priori expectation of ~0.3 does not
// survive contact with these two tests' measured behaviour.
namespace {

vehicle::VehicleDesc car_desc_with_zeta(real zeta, bool zero_rolling_resistance) {
    vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    for (auto& w : desc.wheels) {
        w.tyre.low_speed_damping_ratio = zeta;
        if (zero_rolling_resistance) { w.tyre.qsy1 = 0.0; w.tyre.qsy2 = 0.0; w.tyre.qsy4 = 0.0; }
    }
    return desc;
}

// Case 1: yaw kick with qsy=0 on every tyre (isolates the yaw mode from
// rolling-resistance decay, I1 plan 5.2b(1)). Returns each second's
// envelope E_k = max|yaw_rate| over [k, k+1) s, k = 0..23.
std::vector<real> yaw_kick_envelopes(real zeta) {
    auto rig = ps_test::build_vehicle_rig_from_desc(car_desc_with_zeta(zeta, /*zero_rolling_resistance=*/true), 1500.0,
                                                     Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3::zero(), Vec3{0.0, 0.0, 0.3}});
    std::vector<real> envelope(24, 0.0);
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(24.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const std::size_t k = std::min<std::size_t>(23, static_cast<std::size_t>(t));
        envelope[k] = std::max(envelope[k], std::abs(rig.world->get_motion(rig.chassis).angular.z));
    }
    return envelope;
}

// Case 5 (added while investigating an I1 regression, not in the original
// plan's sweep list): the EXISTING parking-slope acceptance test's own
// drift number at this zeta - the low-speed lateral damper is active in
// its near-standstill fade band too, and turned out to matter there.
real parking_drift_at(real zeta) {
    auto rig = ps_test::build_vehicle_rig_from_desc(car_desc_with_zeta(zeta, /*zero_rolling_resistance=*/false), 1500.0,
                                                     Vec3{2.0, 0.4, 0.15}, 0.6, 0.15);
    rig.world->set_control("handbrake", 1.0);
    ps_test::settle(*rig.world, 2.0);
    const Vec3 p0 = rig.world->get_pose(rig.chassis).position;
    ps_test::settle(*rig.world, 30.0);
    const Vec3 p1 = rig.world->get_pose(rig.chassis).position;
    return distance(p0, p1);
}

// Case 2: lateral kick at rest; time for |vy| to fall below 5% of initial.
real lateral_kick_decay_time(real zeta) {
    auto rig = ps_test::build_vehicle_rig_from_desc(car_desc_with_zeta(zeta, /*zero_rolling_resistance=*/false), 1500.0,
                                                     Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    const real v0 = 0.3;
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{0.0, v0, 0.0}, Vec3::zero()});
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(10.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        if (std::abs(rig.world->get_motion(rig.chassis).linear.y) < 0.05 * v0) {
            return static_cast<real>(i + 1) * dt;
        }
    }
    return -1.0; // did not decay within the window
}

// Case 3: the existing driven-then-braked wiggle cases, at a given zeta.
struct WiggleAmp { real amp_early, amp_late; };
WiggleAmp wiggle_case_amps(real zeta, real hold_steer) {
    auto rig = ps_test::build_vehicle_rig_from_desc(car_desc_with_zeta(zeta, /*zero_rolling_resistance=*/false), 1500.0,
                                                     Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 0.5);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{10.0, 0.0, 0.0}, Vec3::zero()});
    rig.world->set_control("steer", 0.3);
    ps_test::settle(*rig.world, 0.4);
    rig.world->set_control("brake", 0.6);
    const real dt = rig.world->dt();
    const std::uint64_t brake_ticks = static_cast<std::uint64_t>(1.5 / dt + 0.5);
    for (std::uint64_t i = 0; i < brake_ticks; ++i) rig.world->step();
    rig.world->set_control("brake", 0.0);
    rig.world->set_control("steer", hold_steer);
    std::vector<real> window_early, window_late;
    const std::uint64_t n = static_cast<std::uint64_t>(6.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const real yaw_rate = rig.world->get_motion(rig.chassis).angular.z;
        if (t >= 0.2 && t < 1.0) window_early.push_back(yaw_rate);
        if (t >= 4.0 && t < 5.0) window_late.push_back(yaw_rate);
    }
    return WiggleAmp{amplitude(window_early), amplitude(window_late)};
}

// Case 4: settled full-throttle asphalt standing start (duplicated point-
// mass estimate, same as test_vehicle_accel_standing_start.cpp - see that
// file's header comment for why this is deliberately kept in sync by hand).
real curve_eval_local(const std::vector<real>& x, const std::vector<real>& y, real xv) {
    if (xv <= x.front()) return y.front();
    if (xv >= x.back()) return y.back();
    for (std::size_t i = 1; i < x.size(); ++i) {
        if (xv <= x[i]) {
            const real t = (xv - x[i - 1]) / (x[i] - x[i - 1]);
            return y[i - 1] + t * (y[i] - y[i - 1]);
        }
    }
    return y.back();
}
struct AccelResult { real avg_accel_measured, avg_accel_estimate, max_abs_slip; };
AccelResult asphalt_accel_case(real zeta) {
    const real mass = 1500.0;
    auto rig = ps_test::build_vehicle_rig_from_desc(car_desc_with_zeta(zeta, /*zero_rolling_resistance=*/false), mass,
                                                     Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("throttle", 1.0);
    const real target_speed = 50.0 / 3.6;
    real time_to_target = -1.0, max_abs_slip = 0.0;
    const std::uint64_t max_ticks = static_cast<std::uint64_t>(15.0 / rig.world->dt() + 0.5);
    for (std::uint64_t i = 0; i < max_ticks; ++i) {
        rig.world->step();
        const auto& w0 = rig.world->wheel_telemetry(rig.vehicle, 0);
        const auto& w1 = rig.world->wheel_telemetry(rig.vehicle, 1);
        max_abs_slip = std::max({max_abs_slip, std::abs(w0.slip_ratio), std::abs(w1.slip_ratio)});
        const real speed = rig.world->get_motion(rig.chassis).linear.length();
        if (time_to_target < 0.0 && speed >= target_speed) { time_to_target = static_cast<real>(i + 1) * rig.world->dt(); break; }
    }
    if (time_to_target < 0.0) return AccelResult{-1.0, -1.0, max_abs_slip};
    const real avg_accel_measured = target_speed / time_to_target;
    const std::vector<real> rpm_x = {900, 2000, 4000, 6000, 6500};
    const std::vector<real> torque_y = {120, 220, 250, 200, 150};
    real v = 0.0, t = 0.0;
    const real froll_total = 0.023 * mass * 9.81;
    while (v < target_speed && t < 60.0) {
        const real wheel_rpm = std::abs(v) / 0.317 * real(60.0) / (real(2.0) * real(3.14159265358979323846));
        const real engine_rpm_raw = wheel_rpm * 10.0;
        const real engine_rpm = std::clamp(engine_rpm_raw, real(900.0), real(6500.0));
        real torque = curve_eval_local(rpm_x, torque_y, engine_rpm);
        if (engine_rpm_raw > 6500.0) torque = 0.0;
        const real wheel_force_total = (torque * 10.0 * 0.9) / 0.317;
        const real a = (wheel_force_total - froll_total) / mass;
        v += a * rig.world->dt();
        t += rig.world->dt();
    }
    const real avg_accel_estimate = target_speed / t;
    return AccelResult{avg_accel_measured, avg_accel_estimate, max_abs_slip};
}

} // namespace

TEST_CASE("zeta sweep diagnostic", "[.][diag][vehicle]") {
    const std::vector<real> zetas = {0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
    for (real zeta : zetas) {
        const std::vector<real> env = yaw_kick_envelopes(zeta);
        bool regrowth = false;
        for (std::size_t k = 0; k + 1 < 9 && k + 1 < env.size(); ++k) {
            if (env[k + 1] > 1.001 * env[k] + 1e-6) regrowth = true;
        }
        const real lat_decay_t = lateral_kick_decay_time(zeta);
        const WiggleAmp w1a = wiggle_case_amps(zeta, 0.3);
        const WiggleAmp w2a = wiggle_case_amps(zeta, 0.0);
        const AccelResult acc = asphalt_accel_case(zeta);
        const real park_drift = parking_drift_at(zeta);

        std::printf("zeta=%.1f yaw_regrowth=%d E0..8=[%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f] lat_decay_t=%.3f\n",
                    static_cast<double>(zeta), regrowth ? 1 : 0, static_cast<double>(env[0]), static_cast<double>(env[1]),
                    static_cast<double>(env[2]), static_cast<double>(env[3]), static_cast<double>(env[4]),
                    static_cast<double>(env[5]), static_cast<double>(env[6]), static_cast<double>(env[7]),
                    static_cast<double>(env[8]), static_cast<double>(lat_decay_t));
        std::printf("        wiggle(steer=0.3) amp_early=%.5f amp_late=%.5f  wiggle(steer=0) amp_early=%.5f amp_late=%.5f\n",
                    static_cast<double>(w1a.amp_early), static_cast<double>(w1a.amp_late),
                    static_cast<double>(w2a.amp_early), static_cast<double>(w2a.amp_late));
        std::printf("        accel: measured=%.4f estimate=%.4f (%.2f%% diff) max_abs_slip=%.4f  park_drift=%.6f m\n",
                    static_cast<double>(acc.avg_accel_measured), static_cast<double>(acc.avg_accel_estimate),
                    static_cast<double>(100.0 * (acc.avg_accel_measured - acc.avg_accel_estimate) / acc.avg_accel_estimate),
                    static_cast<double>(acc.max_abs_slip), static_cast<double>(park_drift));
    }
}
