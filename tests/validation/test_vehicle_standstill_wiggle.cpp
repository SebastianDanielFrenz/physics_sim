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
