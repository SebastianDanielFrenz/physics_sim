// tests/validation/test_reference_cars.cpp — owner request 2026-09-25:
// "nothing overly complicated, just so the implementation can be used as a
// baseline". Covers the three reference sedan variants as a single simple
// pass/fail smoke:
//   - data/vehicles/car_sedan.json     (FWD: driven FL, FR; 60/40 static split)
//   - data/vehicles/car_sedan_rwd.json (RWD: driven RL, RR; 50/50 static split)
//   - data/vehicles/car_sedan_awd.json (AWD: open centre diff -> open front +
//     rear diffs, all four driven; 50/50 static split, same body as the RWD
//     reference)
//
// Per car: World::wheel_is_driven matches the expected driven set; settled
// static corner loads sum to mass*g within 2% with every wheel in contact;
// a wide-open-throttle launch in 1st from settled rest for ~3 s makes real
// forward progress with bounded slip on both driven and undriven wheels,
// never stalls the engine, and keeps the per-tick energy-ledger balance_error
// small (test_powertrain_energy_balance.cpp's own 5e-8-relative criterion);
// and the same 3 s launch gives an identical World::state_hash() at 1 and 10
// job workers (a separate TEST_CASE, [determinism] tag).
//
// Measured this session (Windows release build, out/build/release,
// job_workers=4 for the non-determinism TEST_CASE; the determinism
// TEST_CASE's own 1-worker/10-worker hashes matched exactly for all three
// cars):
//
//   Car | speed@3s (m/s) | max driven |kappa| | max undriven |kappa| | max balance ratio | hash (1w == 10w)
//   ----|-----------------|-----------------------|-------------------------|--------------------|-----------------
//   FWD | 12.4933         | 2.0154                 | 0.0010                   | 5.6e-16             | 0x3327aa33e997519f
//   RWD | 12.4414         | 0.1032                 | 0.0011                   | 5.5e-16             | 0xb02ada154e97e7e8
//   AWD | 12.4359         | 1.3615                 | n/a (all driven)         | 7.2e-16             | 0x9e7cf59ee598a16f
//
// SURPRISE (reported, not hidden - task's own instruction): at TRUE wide-open
// throttle (1.0, not the 0.7 test_vehicle_accel_standing_start.cpp uses to
// stay inside the front tyres' grip) from a settled rest, FWD and AWD both
// show real, sustained wheelspin (kappa well past the tyre's own ~0.14 peak
// slip, though nowhere near the tyre model's +-3.0 safety clamp) - this is
// the SAME documented, structural "no traction control until I5" limitation
// test_vehicle_accel_standing_start.cpp's own header comment and its
// "Diagnostic: ... WOT wheelspin trace" [.][diag] case already describe for
// car_sedan.json, just asserted here instead of only observed. RWD stays
// low-slip (0.10) because accelerating load-transfers weight ONTO its driven
// rear axle instead of off its driven front axle. AWD's open centre diff
// still sends the front axle half the drive torque regardless of grip, and
// car_sedan_awd.json's 50/50 static split (same body as the RWD reference,
// see its own "source" field) gives that front axle noticeably less static
// load than car_sedan.json's FWD-biased 60/40 split - so AWD's front axle
// spins nearly as much as FWD's does (1.36 vs. 2.02), unlike
// test_powertrain_multi_diff_awd.cpp's synthetic AWD variant (built by
// rewiring car_sedan.json's own 60/40-split VehicleDesc, not this file's
// 50/50 car_sedan_awd.json), which reports <0.5 kappa at the same throttle -
// a genuine, data-driven difference between the two AWD configurations, not
// a bug in either. The per-car kappa bounds below are set with margin ABOVE
// each car's own measured value (same "measure first, bound with margin"
// pattern test_vehicle_accel_standing_start.cpp's own unsettled-start case
// uses), not tuned down to hide this - a regression toward the tyre's
// +-3.0 clamp (deep post-peak, not just past-peak, slip) would still fail.

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

using namespace ps;

namespace {

struct CarConfig {
    const char* json_path;
    const char* label;
    std::array<bool, 4> driven; // FL, FR, RL, RR
    // Bounds set WITH MARGIN ABOVE this file's own measured value per car
    // (see the header comment's table/"SURPRISE" note) - each car's WOT
    // launch behaves differently (FWD/AWD show real, structural wheelspin
    // without traction control; RWD does not), so a single shared bound
    // would either be too loose for RWD or fail FWD/AWD for a real physical
    // reason, not a bug.
    real max_driven_kappa_bound;
    real max_undriven_kappa_bound; // ignored if the car has no undriven wheel
    real min_speed_3s;
};

const std::array<CarConfig, 3> kCars = {{
    // FWD: measured max_driven_kappa=2.0154 (WOT wheelspin, see header
    // comment), max_undriven_kappa=0.0010, speed@3s=12.4933.
    {"/data/vehicles/car_sedan.json", "FWD (car_sedan)", {true, true, false, false}, 2.5, 0.05, 7.0},
    // RWD: measured max_driven_kappa=0.1032 (accel load-transfers weight
    // onto the driven axle - low slip), max_undriven_kappa=0.0011,
    // speed@3s=12.4414.
    {"/data/vehicles/car_sedan_rwd.json", "RWD (car_sedan_rwd)", {false, false, true, true}, 0.3, 0.05, 7.0},
    // AWD: measured max_driven_kappa=1.3615 (open centre diff still spins
    // the less-loaded front axle, see header comment), no undriven wheels,
    // speed@3s=12.4359.
    {"/data/vehicles/car_sedan_awd.json", "AWD (car_sedan_awd)", {true, true, true, true}, 1.8, 0.0, 7.0},
}};

constexpr real kMass = 1500.0;
const Vec3 kHalfExtents{2.0, 0.4, 0.15};
constexpr real kOriginHeight = 0.6;
constexpr real kG = 9.81;

std::uint64_t ticks_for(real seconds, real dt) { return static_cast<std::uint64_t>(seconds / dt + 0.5); }

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) +
           std::abs(e.gear_loss) + std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) +
           std::abs(e.engine_friction_loss) + std::abs(e.engagement_loss) + std::abs(e.starter_in) +
           std::abs(e.external_reset) + std::abs(e.diff_loss); // I4 C6: diff_loss added, 0 for every reference car today
}

ps_test::VehicleTestRig build_rig(const CarConfig& car, int job_workers) {
    return ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + car.json_path, kMass, kHalfExtents, kOriginHeight,
                                       /*slope_grade=*/0.0, job_workers);
}

} // namespace

TEST_CASE("Reference sedan variants (FWD/RWD/AWD): driven wheels, static loads, WOT launch, energy balance",
          "[validation][vehicle][drivetrain]") {
    for (const CarConfig& car : kCars) {
        INFO("car: " << car.label);
        auto rig = build_rig(car, /*job_workers=*/4);
        World& w = *rig.world;

        // 1. Driven-wheel set matches exactly.
        REQUIRE(w.vehicle_wheel_count(rig.vehicle) == 4);
        for (std::size_t i = 0; i < 4; ++i) CHECK(w.wheel_is_driven(rig.vehicle, i) == car.driven[i]);

        // 2. Settle, then static corner loads sum to mass*g within 2%, every
        // wheel in contact (fz > 0).
        ps_test::settle(w, 1.0);
        const real dt = w.dt();
        real fz_sum = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
            const real fz = w.wheel_telemetry(rig.vehicle, i).fz;
            CHECK(fz > 0.0);
            fz_sum += fz;
        }
        const real expected_weight = kMass * kG;
        INFO("fz_sum=" << fz_sum << " expected=" << expected_weight);
        CHECK(std::abs(fz_sum - expected_weight) <= 0.02 * expected_weight);

        // 3. WOT launch in 1st from settled rest, ~3 s.
        real max_driven_kappa = 0.0;
        real max_undriven_kappa = 0.0;
        bool any_undriven = false;
        real max_balance_ratio = 0.0;
        bool ever_stalled = false;
        w.set_control("gear_request", 1.0);
        w.set_control("throttle", 1.0);
        const std::uint64_t launch_ticks = ticks_for(3.0, dt);
        for (std::uint64_t i = 0; i < launch_ticks; ++i) {
            w.step();
            if (w.powertrain_state(rig.vehicle).engine_state == drivetrain::EngineState::Stalled) ever_stalled = true;
            for (std::size_t k = 0; k < 4; ++k) {
                const real kappa = std::abs(w.wheel_telemetry(rig.vehicle, k).slip_ratio);
                if (car.driven[k]) {
                    max_driven_kappa = std::max(max_driven_kappa, kappa);
                } else {
                    any_undriven = true;
                    max_undriven_kappa = std::max(max_undriven_kappa, kappa);
                }
            }
            const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
            const real denom = sum_abs_terms(e);
            if (denom > 1e-9) {
                const real ratio = std::abs(e.balance_error) / denom;
                max_balance_ratio = std::max(max_balance_ratio, ratio);
                CHECK(ratio <= 5e-8);
            }
        }
        const real speed_3s = w.get_motion(rig.chassis).linear.length();

        std::printf("[reference cars] %-20s speed@3s=%.4f m/s  max_driven_kappa=%.4f  max_undriven_kappa=%.4f  "
                    "max_balance_ratio=%.3e  ever_stalled=%d\n",
                    car.label, static_cast<double>(speed_3s), static_cast<double>(max_driven_kappa),
                    static_cast<double>(any_undriven ? max_undriven_kappa : real(-1.0)),
                    static_cast<double>(max_balance_ratio), ever_stalled ? 1 : 0);
        INFO("speed_3s=" << speed_3s << " max_driven_kappa=" << max_driven_kappa
                          << " max_undriven_kappa=" << max_undriven_kappa << " max_balance_ratio=" << max_balance_ratio);

        // Forward progress: at least ~55% of this file's own measured value
        // per car (see the header comment table), a floor that still catches
        // a gross regression (e.g. the car not moving at all, or stalling).
        CHECK(speed_3s > car.min_speed_3s);
        CHECK_FALSE(ever_stalled);
        // Slip bound WITH MARGIN ABOVE this car's own measured value (header
        // comment/CarConfig comments) - not a common "small slip" bound,
        // since FWD/AWD genuinely spin at true WOT (a real, documented,
        // structural finding, not a bug - see the header comment). Still
        // well inside the tyre's own +-3.0 post-peak safety clamp, so a
        // regression toward deep post-peak slip would fail this.
        CHECK(max_driven_kappa < car.max_driven_kappa_bound);
        if (any_undriven) CHECK(max_undriven_kappa < car.max_undriven_kappa_bound);
    }
}

TEST_CASE("Reference sedan variants (FWD/RWD/AWD): state hash is worker-count invariant (1 vs 10 workers)",
          "[validation][vehicle][drivetrain][determinism]") {
    for (const CarConfig& car : kCars) {
        INFO("car: " << car.label);
        auto run = [&](int workers) {
            auto rig = build_rig(car, workers);
            World& w = *rig.world;
            ps_test::settle(w, 1.0);
            const real dt = w.dt();
            w.set_control("gear_request", 1.0);
            w.set_control("throttle", 1.0);
            for (std::uint64_t i = 0; i < ticks_for(3.0, dt); ++i) w.step();
            return w.state_hash();
        };
        const std::uint64_t h1 = run(1);
        const std::uint64_t h10 = run(10);
        std::printf("[reference cars] %-20s hash(1w)=0x%016llx hash(10w)=0x%016llx\n", car.label,
                    static_cast<unsigned long long>(h1), static_cast<unsigned long long>(h10));
        CHECK(h1 == h10);
    }
}
