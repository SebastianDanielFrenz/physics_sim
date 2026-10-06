// tests/validation/test_car_sedan_gen_n2o.cpp -- N2O plan N2 smoke test of the shipped
// nitrous car data/vehicles/car_sedan_gen_n2o.json (car_sedan_gen + the 10 lb 950 psi
// bottle + the Edelbrock 70011 50 hp kit + edelbrock_basic controller, arm channel
// "nitrous_arm"). A WOT pull: launch in 1st, upshift at 5500 rpm to 2nd and 3rd, arm the
// kit once 3rd is in and the auto-clutch has locked, pull to 6000 rpm. The same script
// without arming is the baseline.
//
// Why 3rd: this FWD sedan spins its front tyres at WOT in 1st and 2nd (test_reference_cars:
// FWD driven kappa ~2), so the 2nd-gear clutch locks only near 5100 rpm and the post-shift
// close (upshift ignition cut, slipping clutch) covers most of the rpm range. In 3rd the
// clutch locks low enough to leave 4500-5500 rpm for the comparison.
//
// Asserted: no stall, the kit sprays, balance_error <= 5e-8 relative every tick, bottle
// closure (initial - m_bottle = n2o_mass + n2o_purged - n2o_external_added) <= 1e-12 of the
// fill every tick, the N2O energy closure (fuel + N2O energy in = indicated + engine_heat),
// and a positive crank-power gain at 4500/5000/5500 rpm. The gain itself is REPORTED
// only: the kit/whole-car validation against the Edelbrock rating is N5 (plan 9.2/9.4,
// planner's estimate about +37 kW).
#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kMass = 1500.0;
const Vec3 kHalfExtents{2.0, 0.4, 0.15};
constexpr real kOriginHeight = 0.6;
constexpr int kPullGear = 3;
constexpr real kShiftRpm = 5500.0;

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
}

struct Sample {
    real rpm, dt, crank_work; // crank work this tick: engine_out - engine_friction_loss (the Coulomb row's share)
};

struct PullResult {
    std::vector<Sample> pull; // ticks in kPullGear with the clutch locked, from the arm point (or where it would be)
    bool stalled = false;
    real worst_balance = 0.0, worst_bottle = 0.0, worst_closure = 0.0;
    real n2o_kg = 0.0, n2o_energy = 0.0, kit_fuel_kg = 0.0, fuel_kg = 0.0, unburnt_kg = 0.0;
    int spray_ticks = 0;
    real arm_rpm = 0.0, t_arm = -1.0, t_end = 0.0, bottle0 = 0.0;
};

PullResult pull(const vehicle::VehicleDesc& desc, bool arm) {
    PullResult res;
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kMass, kHalfExtents, kOriginHeight, 0.0, 4);
    World& w = *rig.world;
    const real dt = w.dt();
    ps_test::settle(w, 2.0);
    res.bottle0 = w.n2o_bottle_mass(rig.vehicle);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    bool armed = false;
    real prev_work = w.powertrain_energy(rig.vehicle).engine_out - w.powertrain_energy(rig.vehicle).engine_friction_loss;
    const int max_ticks = static_cast<int>(20.0 / dt);
    for (int t = 0; t < max_ticks; ++t) {
        w.step();
        const drivetrain::PowertrainSnapshot& st = w.powertrain_state(rig.vehicle);
        const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        if (st.engine_state != drivetrain::EngineState::Running) res.stalled = true;
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) res.worst_balance = std::max(res.worst_balance, std::abs(e.balance_error) / denom);
        res.worst_bottle = std::max(res.worst_bottle,
                                    std::abs((res.bottle0 - w.n2o_bottle_mass(rig.vehicle)) - (e.n2o_mass + e.n2o_purged - e.n2o_external_added)));
        const real in = e.fuel_energy_in + e.n2o_energy_in;
        if (in > 0.0) res.worst_closure = std::max(res.worst_closure, std::abs(in - e.indicated_work - e.engine_heat) / in);
        if (st.engines.at(0).n2o_active) ++res.spray_ticks;
        const real work = e.engine_out - e.engine_friction_loss;
        const bool idle = st.shift_phase == drivetrain::ShiftPhase::Idle;
        if (idle && st.gear >= 1 && st.gear < kPullGear && st.engine_rpm >= kShiftRpm)
            w.set_control("gear_request", static_cast<real>(st.gear + 1));
        const bool in_pull = st.gear == kPullGear && idle && st.clutch_engagement >= 0.999;
        if (in_pull && !armed) {
            armed = true;
            res.arm_rpm = st.engine_rpm;
            res.t_arm = t * dt;
            if (arm) w.set_control("nitrous_arm", 1.0);
        }
        if (armed && in_pull) res.pull.push_back(Sample{st.engine_rpm, dt, work - prev_work});
        prev_work = work;
        if (armed && st.engine_rpm >= 6000.0) {
            res.t_end = t * dt;
            break;
        }
    }
    const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
    res.n2o_kg = e.n2o_mass;
    res.n2o_energy = e.n2o_energy_in;
    res.kit_fuel_kg = e.kit_fuel_mass;
    res.fuel_kg = e.fuel_mass;
    res.unburnt_kg = e.n2o_unburnt;
    return res;
}

// Mean crank power (W) over the pull-gear ticks with rpm in [rpm - 150, rpm + 150].
real power_at(const PullResult& r, real rpm) {
    real work = 0.0, time = 0.0;
    for (const Sample& s : r.pull) {
        if (std::abs(s.rpm - rpm) <= 150.0) {
            work += s.crank_work;
            time += s.dt;
        }
    }
    return time > 0.0 ? work / time : std::nan("");
}

} // namespace

TEST_CASE("car_sedan_gen_n2o: WOT 3rd-gear pull with the 50 hp kit armed once the clutch locks", "[validation][n2o][vehicle][drivetrain]") {
    vehicle::VehicleDesc desc;
    {
        jobs::JobSystem pool(10);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_gen_n2o.json", opt);
    }
    const PullResult with = pull(desc, true);
    const PullResult base = pull(desc, false);

    std::printf("[N2 smoke] gear %d locked/armed at %.0f rpm (t %.2f s; baseline %.0f rpm), 6000 rpm at t %.2f s (baseline %.2f s); sprayed %d "
                "ticks, N2O %.2f g (unburnt %.3f g), decomposition %.1f kJ, kit fuel %.2f g of %.2f g fuel\n",
                kPullGear, with.arm_rpm, with.t_arm, base.arm_rpm, with.t_end, base.t_end, with.spray_ticks, with.n2o_kg * 1e3,
                with.unburnt_kg * 1e3, with.n2o_energy * 1e-3, with.kit_fuel_kg * 1e3, with.fuel_kg * 1e3);
    std::printf("[N2 smoke] worst per tick: balance %.2e, bottle closure %.2e kg, N2O energy closure %.2e\n", with.worst_balance,
                with.worst_bottle, with.worst_closure);
    for (real rpm : {4000.0, 4500.0, 5000.0, 5500.0}) {
        const real pw = power_at(with, rpm), pb = power_at(base, rpm);
        std::printf("[N2 smoke] %.0f rpm: crank power %.1f kW with N2O, %.1f kW without, gain %+.1f kW (%+.1f %%)\n", rpm, pw * 1e-3,
                    pb * 1e-3, (pw - pb) * 1e-3, 100.0 * (pw / pb - 1.0));
    }

    CHECK_FALSE(with.stalled);
    CHECK_FALSE(base.stalled);
    REQUIRE(with.t_arm > 0.0);
    CHECK(with.arm_rpm <= 4300.0); // leaves 4500-5500 rpm locked in the pull gear
    CHECK(with.spray_ticks > 0);
    CHECK(with.n2o_energy > 0.0);
    CHECK(base.n2o_kg == 0.0);
    CHECK(with.worst_balance <= 5e-8);
    CHECK(base.worst_balance <= 5e-8);
    CHECK(with.worst_bottle <= 1e-12 * with.bottle0); // relative to the initial fill, like N1 S2
    CHECK(with.worst_closure <= 1e-12);
    for (real rpm : {4500.0, 5000.0, 5500.0}) CHECK(power_at(with, rpm) > power_at(base, rpm));
    CHECK(with.t_end - with.t_arm < base.t_end - base.t_arm);
}
