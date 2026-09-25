// tests/validation/test_powertrain_multi_diff_awd.cpp — PHYS-033 P4 I4 C4
// (plan sections 2.5/2.7): a whole-vehicle AWD built from car_sedan.json's
// VehicleDesc with its powertrain rewired IN THE TEST ONLY (tests/support/
// powertrain_multi_diff_desc.h: open centre diff ratio 1 -> open front/rear
// axle diffs 4.06, efficiency 0.97). Checks that WOT from rest drives all
// four wheels, that the energy ledger still balances (same bounds as
// test_powertrain_energy_balance.cpp), that the speedo path through the
// centre diff reads ground speed at a low-slip cruise, that an open centre
// splits torque ~50/50 on equal grip, and that the state hash is
// worker-count invariant with the nested (non-leaf) reconstruction live.

#include "powertrain_multi_diff_desc.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

using namespace ps;

namespace {

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) +
           std::abs(e.gear_loss) + std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) +
           std::abs(e.engine_friction_loss) + std::abs(e.engagement_loss) + std::abs(e.starter_in) +
           std::abs(e.external_reset) + std::abs(e.diff_loss); // I4 C6: diff_loss added, 0 (every diff here is open)
}

ps_test::VehicleTestRig build_awd_rig(real centre_eff, int workers) {
    return ps_test::build_vehicle_rig_from_desc(ps_test::make_awd_desc(centre_eff), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers,
                                                 Vec3::unit_y(), 240.0, 960.0, /*ground_half_extent=*/2000.0);
}

std::uint64_t ticks_for(real seconds, real dt) { return static_cast<std::uint64_t>(seconds / dt + 0.5); }

} // namespace

TEST_CASE("AWD (open centre -> open axle diffs): WOT drives all four wheels, ledger balances, speedo reads ground speed, 50/50 split",
          "[validation][vehicle][powertrain][drivetrain]") {
    for (const real centre_eff : {real(1.0), real(0.99)}) {
        INFO("centre efficiency " << centre_eff);
        auto rig = build_awd_rig(centre_eff, 4);
        World& w = *rig.world;
        REQUIRE(w.vehicle_wheel_count(rig.vehicle) == 4);
        for (std::size_t i = 0; i < 4; ++i) CHECK(w.wheel_is_driven(rig.vehicle, i));
        REQUIRE(w.wheel_name(rig.vehicle, 0) == "FL");
        REQUIRE(w.wheel_name(rig.vehicle, 1) == "FR");
        REQUIRE(w.wheel_name(rig.vehicle, 2) == "RL");
        REQUIRE(w.wheel_name(rig.vehicle, 3) == "RR");

        ps_test::settle(w, 1.0);
        const real dt = w.dt();

        real max_ratio = 0.0;
        auto check_tick = [&]() {
            const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
            const real denom = sum_abs_terms(e);
            if (denom > 1e-9) {
                max_ratio = std::max(max_ratio, std::abs(e.balance_error) / denom);
                CHECK(std::abs(e.balance_error) <= 5e-8 * denom);
            }
        };

        // 1. WOT from rest in 1st (auto-clutch launch).
        w.set_control("gear_request", 1.0);
        w.set_control("throttle", 1.0);
        for (std::uint64_t i = 0; i < ticks_for(0.3, dt); ++i) {
            w.step();
            check_tick();
        }
        REQUIRE(w.powertrain_state(rig.vehicle).gear == 1);
        w.set_control("gear_request", drivetrain::kGearRequestNone);

        // 2. Keep WOT for 2 s; from 0.5 s on (clutch locked) accumulate the
        // front vs rear axle torque (each axle diff's hub shaft torque).
        real front_sum = 0.0, rear_sum = 0.0;
        real max_abs_kappa = 0.0;
        const std::uint64_t wot_ticks = ticks_for(2.0, dt);
        for (std::uint64_t i = 0; i < wot_ticks; ++i) {
            w.step();
            check_tick();
            if (static_cast<real>(i) * dt >= 0.5) {
                const drivetrain::PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
                front_sum += dbg.hub_shaft_torque[0] + dbg.hub_shaft_torque[1];
                rear_sum += dbg.hub_shaft_torque[2] + dbg.hub_shaft_torque[3];
            }
            for (std::size_t k = 0; k < 4; ++k) max_abs_kappa = std::max(max_abs_kappa, std::abs(w.wheel_telemetry(rig.vehicle, k).slip_ratio));
        }
        const real speed_after_wot = w.get_motion(rig.chassis).linear.length();
        const real front_share = front_sum / (front_sum + rear_sum);
        INFO("speed_after_wot=" << speed_after_wot << " front_share=" << front_share << " front_sum=" << front_sum
                                << " rear_sum=" << rear_sum << " max_abs_kappa=" << max_abs_kappa);
        CHECK(speed_after_wot > 5.0); // drives away
        CHECK(front_sum > 0.0);       // both axles carry drive torque
        CHECK(rear_sum > 0.0);
        CHECK(std::abs(front_share - 0.5) < 0.02); // open centre, equal grip: ~50/50
        CHECK(max_abs_kappa < 0.5);                // four driven tyres share the torque - no runaway spin

        // 3. Low-slip cruise: light throttle for 2 s, then speedo vs chassis
        // ground speed (the speedo path runs through the centre diff).
        w.set_control("throttle", 0.15);
        for (std::uint64_t i = 0; i < ticks_for(2.0, dt); ++i) {
            w.step();
            check_tick();
        }
        const real ground = w.get_motion(rig.chassis).linear.length();
        const real speedo = w.powertrain_state(rig.vehicle).speedo_mps;
        INFO("ground=" << ground << " speedo=" << speedo << " max per-tick balance ratio=" << max_ratio);
        REQUIRE(ground > 3.0);
        CHECK(std::abs(speedo - ground) <= 0.03 * ground);

        // 4. End-of-run ledger bound (test_powertrain_energy_balance.cpp's).
        const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        CHECK(std::abs(e.balance_error) <= 0.01 * (std::abs(e.engine_out) + std::abs(e.starter_in)));
        CHECK(e.gear_loss > 0.0);
    }
}

TEST_CASE("AWD state hash is worker-count invariant (1 vs 10 workers)", "[validation][vehicle][powertrain][drivetrain][determinism]") {
    auto run = [](int workers) {
        auto rig = build_awd_rig(0.99, workers); // lossy centre: the nested reconstruction feeds hashed state
        World& w = *rig.world;
        ps_test::settle(w, 0.5);
        const real dt = w.dt();
        w.set_control("gear_request", 1.0);
        w.set_control("throttle", 1.0);
        for (std::uint64_t i = 0; i < ticks_for(1.5, dt); ++i) w.step();
        w.set_control("steer", 0.3);
        for (std::uint64_t i = 0; i < ticks_for(0.5, dt); ++i) w.step();
        return w.state_hash();
    };
    const std::uint64_t h1 = run(1);
    const std::uint64_t h10 = run(10);
    CHECK(h1 == h10);
}
