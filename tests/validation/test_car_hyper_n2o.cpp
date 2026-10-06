// tests/validation/test_car_hyper_n2o.cpp -- N2O plan N7 (owner "50hp is negligable, make it 800"):
// the hypercar (data/vehicles/car_hyper.json, a twin-turbo 5.07 L V8, engines/hyper_v8_one1_audio.json)
// with an ~800 hp (597 kW) [AS] wet shot, data/vehicles/car_hyper_n2o.json.
//
// The numbers below are the POST-N3 ones (turbo exhaust balance with the N2O decomposition heat, merged e4756a8); the pre-N3 table is kept
// in the comment block above the asserting tests. The diag cases (tag [.][diag][n2o]) print the dyno table, a kit-size exploration and a pull
// report and are rerunnable (env vars PS_JET, PS_FJET, PS_RPM0/1, PS_DRPM, PS_PULL_GEAR; see the explore case).
#include "drivetrain/engine_simulated.h"
#include "drivetrain/nitrous_flow.h"
#include "vehicle_test_helpers.h"

#include "ps/io/nitrous_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

using namespace ps;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kHpW = 745.699872; // [D] mechanical horsepower in W
constexpr real kIsaP = 101325.0;

std::string data_path(const std::string& r) { return std::string(PS_SOURCE_DIR) + "/data/" + r; }

vehicle::VehicleDesc load_car(const std::string& file, drivetrain::MapQuality q = drivetrain::MapQuality::Full) {
    static jobs::JobSystem pool(10);
    io::EngineMapOptions opt;
    opt.quality = q;
    opt.pool = &pool;
    opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_n2o_800hp_engine_cache").string();
    return io::load_vehicle_json(data_path("vehicles/" + file), opt);
}

template <typename T>
const T* find_component(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* p = std::get_if<T>(&c.params)) return p;
    return nullptr;
}

// What the dyno-held harness needs of a kit: jets, fuel, bottle pressure, retard.
struct KitSpec {
    real n_area_m2 = 0, cd_n = 0, f_area_m2 = 0, cd_f = 0, fuel_gauge_pa = 0;
    bool fuel_manifold_ref = false;
    bool dyer = false;
    real retard_deg = 0, bottle_abs_pa = 0;
};

[[maybe_unused]] KitSpec spec_from(const drivetrain::NitrousKitDesc& k, const drivetrain::N2oBottleDesc& b) {
    KitSpec s;
    const real n = real(k.nozzle_count);
    s.n_area_m2 = n * kPi / 4.0 * k.nitrous_jet_diameter_mm * k.nitrous_jet_diameter_mm * 1e-6;
    s.f_area_m2 = n * kPi / 4.0 * k.fuel_jet_diameter_mm * k.fuel_jet_diameter_mm * 1e-6;
    s.cd_n = k.nitrous_jet_cd;
    s.cd_f = k.fuel_jet_cd;
    s.fuel_gauge_pa = k.fuel_pressure_gauge_bar * 1e5;
    s.fuel_manifold_ref = k.fuel_reference == drivetrain::NitrousFuelReference::Manifold;
    s.dyer = k.flow_model == drivetrain::NitrousFlowModel::Dyer;
    s.retard_deg = k.controller.retard_is_curve ? k.controller.retard_vs_fraction.evaluate(1.0) : k.controller.spark_retard_deg;
    // The shipped bottle files are constant pressure; the harness holds that pressure.
    s.bottle_abs_pa = (b.pressure.gauge_bar + 1.01325) * 1e5;
    return s;
}

struct ShotRes {
    real brake_nm = 0, kw = 0, p_im_kpa = 0, boost_bar = 0, lambda_c = 0, lambda_ecu = 0, t_mix = 0, t_charge = 0, mdot_n = 0, mdot_kf = 0,
         spark = 0, turbo_rpm = 0, brake_min = 1e300, brake_max = -1e300;
    std::uint64_t overflow = 0;
};

// Dyno-held engine: omega fixed, the engine's own management running, the kit's flows computed from the kit
// description exactly like NitrousSystem::step (HEM/Dyer flux x Cd x A; the fuel jet incompressible).
ShotRes dyno_shot(const drivetrain::SimulatedEngineDesc& d, const std::shared_ptr<const drivetrain::SimulatedEngineMaps>& maps,
                  const drivetrain::OxidizerTables& tabs, real rpm, const KitSpec* kit, real settle_s = 10.0, real avg_s = 0.5) {
    constexpr real h = 1.0 / 960.0;
    drivetrain::SimulatedEngine e(d, maps);
    const real omega = rpm * kPi / 30.0;
    const int n_settle = static_cast<int>(settle_s / h + 0.5), n_avg = static_cast<int>(avg_s / h + 0.5);
    real hin = 0, g = 0;
    if (kit) {
        hin = tabs.liquid_at_pressure(kit->bottle_abs_pa).h_l_j_kg;
        g = tabs.g_hem(kit->bottle_abs_pa);
    }
    const real rho_f = d.fuels[0].density_kg_m3;
    ShotRes r;
    real p_im = 0.9e5;
    std::uint64_t ov0 = 0;
    for (int i = 0; i < n_settle + n_avg; ++i) {
        if (i == n_settle) ov0 = e.n2o_overflow_substeps();
        drivetrain::Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.clutch_engaged_fraction = 1.0;
        s.gearbox_in_neutral = false;
        real mdn = 0, mdk = 0;
        if (kit) {
            const real gg = kit->dyer ? tabs.g_dyer(kit->bottle_abs_pa, p_im) : g;
            mdn = kit->cd_n * kit->n_area_m2 * gg;
            const real p_fuel = (kit->fuel_manifold_ref ? p_im : kIsaP) + kit->fuel_gauge_pa;
            mdk = kit->cd_f * kit->f_area_m2 * std::sqrt(2.0 * rho_f * std::max(0.0, p_fuel - p_im));
            s.nitrous.mdot_n_kg_s = mdn;
            s.nitrous.mdot_kf_kg_s = mdk;
            s.nitrous.h_in_j_kg = hin;
            s.nitrous.retard_deg = kit->retard_deg;
            s.nitrous.active = true;
            s.nitrous.oxidizer = &tabs;
        }
        drivetrain::ThrottleCommand tc;
        tc.command = 1.0;
        const drivetrain::PreSolveResult pre = e.pre_solve(s, tc, drivetrain::TcuRequest{}, h);
        (void)e.post_solve(omega, h);
        p_im = pre.p_intake_pa;
        if (i >= n_settle) {
            const real b = pre.tau0 - pre.friction_cap_kinetic;
            r.brake_nm += b;
            r.brake_min = std::min(r.brake_min, b);
            r.brake_max = std::max(r.brake_max, b);
            r.p_im_kpa += pre.p_intake_pa * 1e-3;
            r.boost_bar += pre.boost_bar;
            r.lambda_c += pre.n2o_lambda_combined;
            r.lambda_ecu += pre.lambda;
            r.t_mix += pre.charge_temperature_k;
            r.t_charge += pre.t_charge_k;
            r.mdot_n += mdn;
            r.mdot_kf += mdk;
            r.spark += pre.spark_advance_deg;
            r.turbo_rpm += pre.turbo_rpm;
        }
    }
    const real inv = 1.0 / n_avg;
    for (real* x : {&r.brake_nm, &r.p_im_kpa, &r.boost_bar, &r.lambda_c, &r.lambda_ecu, &r.t_mix, &r.t_charge, &r.mdot_n, &r.mdot_kf, &r.spark,
                    &r.turbo_rpm})
        *x *= inv;
    r.kw = r.brake_nm * omega * 1e-3;
    r.overflow = e.n2o_overflow_substeps() - ov0;
    return r;
}

std::string envs(const char* n, const std::string& def = "") {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, n) != 0 || buf == nullptr) return def;
    std::string v(buf);
    std::free(buf);
    return v;
#else
    const char* v = std::getenv(n);
    return v ? std::string(v) : def;
#endif
}

real envd(const char* n, real def) {
    const std::string v = envs(n);
    return v.empty() ? def : std::atof(v.c_str());
}

void print_row(real rpm, const ShotRes& b, const ShotRes& w) {
    std::printf("[N7 dyno] %5.0f rpm: base %7.1f Nm %6.1f kW (boost %.2f bar, pim %.0f kPa) | shot %7.1f Nm %6.1f kW (boost %.2f, pim %.0f)  gain %+6.1f kW %+6.1f hp"
                " | lam_c %.3f (ecu %.3f) T_mix %.1f K (T_charge %.1f) N2O %.1f g/s kit fuel %.1f g/s spark %.1f turbo %.0f/%.0f rpm ov %llu spread %.1f Nm\n",
                rpm, b.brake_nm, b.kw, b.boost_bar, b.p_im_kpa, w.brake_nm, w.kw, w.boost_bar, w.p_im_kpa, w.kw - b.kw, (w.kw - b.kw) * 1e3 / kHpW,
                w.lambda_c, w.lambda_ecu, w.t_mix, w.t_charge, w.mdot_n * 1e3, w.mdot_kf * 1e3, w.spark, b.turbo_rpm, w.turbo_rpm,
                static_cast<unsigned long long>(w.overflow), w.brake_max - w.brake_min);
}

} // namespace

// Exploration of the kit size (environment variables), on car_hyper.json's own engine.
TEST_CASE("N7 explore: nitrous jets on the hyper V8", "[.][diag][n2o]") {
    const vehicle::VehicleDesc car = load_car(envs("PS_CAR", "car_hyper.json"));
    const auto* ed = find_component<drivetrain::SimulatedEngineDesc>(car);
    REQUIRE(ed != nullptr);
    const auto maps = drivetrain::SimulatedEngine::build_maps(*ed);
    const drivetrain::OxidizerDesc ox = io::load_oxidizer_json(data_path("oxidizers/nitrous_oxide.json"));
    const drivetrain::OxidizerTables tabs(ox);
    KitSpec k;
    const real dn = envd("PS_JET", 3.0), df = envd("PS_FJET", 1.0);
    k.n_area_m2 = kPi / 4.0 * dn * dn * 1e-6;
    k.f_area_m2 = kPi / 4.0 * df * df * 1e-6;
    k.cd_n = envd("PS_CDN", 0.66);
    k.cd_f = envd("PS_CDF", 0.7);
    k.fuel_gauge_pa = envd("PS_FBAR", 4.0) * 1e5;
    k.fuel_manifold_ref = envd("PS_FMAN", 1.0) > 0.5;
    k.retard_deg = envd("PS_RETARD", 6.0);
    k.bottle_abs_pa = (envd("PS_BAR", 65.5) + 1.01325) * 1e5;
    std::printf("[N7 explore] N2O jet %.3f mm, fuel jet %.3f mm, fuel %.1f bar (%s), retard %.1f, bottle %.1f bar abs\n", dn, df, k.fuel_gauge_pa * 1e-5,
                k.fuel_manifold_ref ? "manifold" : "atmosphere", k.retard_deg, k.bottle_abs_pa * 1e-5);
    const real r0 = envd("PS_RPM0", 4000), r1 = envd("PS_RPM1", 6000), dr = envd("PS_DRPM", 2000);
    for (real rpm = r0; rpm <= r1 + 1; rpm += dr) {
        const ShotRes b = dyno_shot(*ed, maps, tabs, rpm, nullptr);
        const ShotRes w = dyno_shot(*ed, maps, tabs, rpm, &k);
        print_row(rpm, b, w);
    }
}

// ===========================================================================
// The shipped car: data/vehicles/car_hyper_n2o.json
// ===========================================================================
namespace {

constexpr int kPullGear = 3;
constexpr real kShiftRpm = 7000.0;
constexpr real kEndRpm = 7800.0;
constexpr real kMass = 1500.0;
const Vec3 kHalfExtents{2.0, 0.4, 0.15};
constexpr real kOriginHeight = 0.5;

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
}

struct Sample {
    real t, rpm, dt, crank_work, flow_g_s, kappa, clutch_slip_rpm, boost_bar;
    bool stuck;
};

struct PullResult {
    std::vector<Sample> pull; // ticks in kPullGear with the clutch locked, from the arm point (or where it would be)
    bool stalled = false;
    real worst_balance = 0.0, worst_bottle = 0.0, worst_closure = 0.0, worst_tank = 0.0;
    real n2o_kg = 0.0, kit_fuel_kg = 0.0, unburnt_kg = 0.0;
    int spray_ticks = 0, locked_ticks = 0, unlocked_ticks = 0;
    real arm_rpm = 0.0, t_arm = -1.0, t_end = 0.0, bottle0 = 0.0, max_kappa = 0.0, max_slip_rpm = 0.0, v_end = 0.0;
};

PullResult pull(const vehicle::VehicleDesc& desc, bool arm, int pull_gear = kPullGear, real end_rpm = kEndRpm) {
    PullResult res;
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kMass, kHalfExtents, kOriginHeight, 0.0, 4);
    World& w = *rig.world;
    const real dt = w.dt();
    ps_test::settle(w, 2.0);
    res.bottle0 = w.n2o_bottle_mass(rig.vehicle);
    const real tank0 = w.fuel_tank_mass(rig.vehicle);
    // The fuel ledger is cumulative since t = 0 (the 2 s settle burns fuel at idle): close the tank against the DELTA since here.
    const drivetrain::PowertrainEnergy e0 = w.powertrain_energy(rig.vehicle);
    const real fuel0 = e0.fuel_mass - e0.fuel_external_added;
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    bool armed = false;
    auto work_of = [&] {
        const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        return e.engine_out - e.engine_friction_loss;
    };
    real prev_work = work_of();
    const int max_ticks = static_cast<int>(30.0 / dt);
    for (int t = 0; t < max_ticks; ++t) {
        w.step();
        const drivetrain::PowertrainSnapshot& st = w.powertrain_state(rig.vehicle);
        const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        if (st.engine_state != drivetrain::EngineState::Running) res.stalled = true;
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) res.worst_balance = std::max(res.worst_balance, std::abs(e.balance_error) / denom);
        res.worst_bottle = std::max(res.worst_bottle, std::abs((res.bottle0 - w.n2o_bottle_mass(rig.vehicle)) - (e.n2o_mass + e.n2o_purged - e.n2o_external_added)));
        res.worst_tank = std::max(res.worst_tank, std::abs((tank0 - w.fuel_tank_mass(rig.vehicle)) - ((e.fuel_mass - e.fuel_external_added) - fuel0)));
        const real in = e.fuel_energy_in + e.n2o_energy_in;
        if (in > 0.0) res.worst_closure = std::max(res.worst_closure, std::abs(in - e.indicated_work - e.engine_heat) / in);
        const auto& es = st.engines.at(0);
        if (es.n2o_active) ++res.spray_ticks;
        const real work = work_of();
        const bool idle = st.shift_phase == drivetrain::ShiftPhase::Idle;
        if (idle && st.gear >= 1 && st.gear < pull_gear && st.engine_rpm >= kShiftRpm) w.set_control("gear_request", static_cast<real>(st.gear + 1));
        const bool in_pull = st.gear == pull_gear && idle && st.clutch_engagement >= 0.999;
        if (in_pull && !armed) {
            armed = true;
            res.arm_rpm = st.engine_rpm;
            res.t_arm = t * dt;
            if (arm) w.set_control("nitrous_arm", 1.0);
        }
        if (armed && in_pull) {
            const drivetrain::PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
            const real kappa = std::max(w.wheel_state(rig.vehicle, 2).slip_ratio, w.wheel_state(rig.vehicle, 3).slip_ratio);
            res.max_kappa = std::max(res.max_kappa, kappa);
            const real slip_rpm = std::abs(dbg.clutch_slip) * 60.0 / (2.0 * kPi);
            res.max_slip_rpm = std::max(res.max_slip_rpm, slip_rpm);
            if (dbg.clutch_stuck) ++res.locked_ticks;
            else ++res.unlocked_ticks;
            res.pull.push_back(Sample{t * dt - res.t_arm, st.engine_rpm, dt, work - prev_work, es.n2o_flow_g_s, kappa, slip_rpm, es.boost_bar, dbg.clutch_stuck});
        }
        prev_work = work;
        if (armed && st.engine_rpm >= end_rpm) {
            res.t_end = t * dt;
            break;
        }
    }
    const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
    res.n2o_kg = e.n2o_mass;
    res.kit_fuel_kg = e.kit_fuel_mass;
    res.unburnt_kg = e.n2o_unburnt;
    const auto m = w.get_motion(rig.chassis);
    res.v_end = m.linear.x;
    return res;
}

// Mean crank power (W) over the pull-gear ticks with rpm in [rpm - 250, rpm + 250] and t_since_arm >= t_min.
real power_at(const PullResult& r, real rpm, real t_min = 0.0) {
    real work = 0.0, time = 0.0;
    for (const Sample& s : r.pull) {
        if (std::abs(s.rpm - rpm) <= 250.0 && s.t >= t_min) {
            work += s.crank_work;
            time += s.dt;
        }
    }
    return time > 0.0 ? work / time : std::nan("");
}

} // namespace

TEST_CASE("N7 dyno: car_hyper_n2o table", "[.][diag][n2o]") {
    const vehicle::VehicleDesc car = load_car("car_hyper_n2o.json");
    const auto* ed = find_component<drivetrain::SimulatedEngineDesc>(car);
    const auto* kit = find_component<drivetrain::NitrousKitDesc>(car);
    const auto* bottle = find_component<drivetrain::N2oBottleDesc>(car);
    REQUIRE(ed != nullptr);
    REQUIRE(kit != nullptr);
    REQUIRE(bottle != nullptr);
    const auto maps = drivetrain::SimulatedEngine::build_maps(*ed);
    const drivetrain::OxidizerDesc ox = bottle->oxidizer;
    const drivetrain::OxidizerTables tabs(ox);
    const KitSpec spec = spec_from(*kit, *bottle);
    for (real rpm : {3000.0, 4000.0, 5000.0, 6000.0, 7000.0}) print_row(rpm, dyno_shot(*ed, maps, tabs, rpm, nullptr), dyno_shot(*ed, maps, tabs, rpm, &spec));
    const real mdot = spec.cd_n * spec.n_area_m2 * tabs.g_hem(spec.bottle_abs_pa);
    std::printf("[N7 dyno] N2O flow %.1f g/s, bottle %.3f kg lasts %.1f s at WOT\n", mdot * 1e3, bottle->capacity_kg, bottle->capacity_kg / mdot);
}

TEST_CASE("N7 whole car: car_hyper_n2o WOT pull report", "[.][diag][n2o]") {
    const vehicle::VehicleDesc desc = load_car("car_hyper_n2o.json");
    const int pg = static_cast<int>(envd("PS_PULL_GEAR", kPullGear));
    const PullResult with = pull(desc, true, pg);
    const PullResult base = pull(desc, false, pg);
    std::printf("[N7 pull] gear %d armed at %.0f rpm (t %.2f s; baseline %.0f rpm), %.0f rpm at t %.2f s (baseline %.2f s); sprayed %d ticks, N2O %.1f g (unburnt %.3f g), kit fuel %.1f g\n",
                pg, with.arm_rpm, with.t_arm, base.arm_rpm, kEndRpm, with.t_end, base.t_end, with.spray_ticks, with.n2o_kg * 1e3, with.unburnt_kg * 1e3, with.kit_fuel_kg * 1e3);
    std::printf("[N7 pull] worst per tick: balance %.2e, bottle %.2e kg, tank %.2e kg, N2O energy closure %.2e; clutch stuck ticks %d unstuck %d, max slip %.1f rpm, max rear kappa %.2f (baseline %.2f), v_end %.1f (baseline %.1f) m/s\n",
                with.worst_balance, with.worst_bottle, with.worst_tank, with.worst_closure, with.locked_ticks, with.unlocked_ticks, with.max_slip_rpm, with.max_kappa, base.max_kappa, with.v_end, base.v_end);
    int n = 0;
    for (const Sample& s : with.pull) {
        if (n++ % 60 == 0)
            std::printf("[N7 pull]   t %.2f rpm %.0f work-rate %.1f kW n2o %.1f g/s kappa %.2f slip %.1f rpm boost %.2f%s\n", s.t, s.rpm, s.crank_work / s.dt * 1e-3, s.flow_g_s, s.kappa, s.clutch_slip_rpm, s.boost_bar, s.stuck ? " stuck" : "");
    }
    for (real rpm : {5000.0, 6000.0, 7000.0}) {
        const real pw = power_at(with, rpm), pb = power_at(base, rpm);
        std::printf("[N7 pull] %.0f rpm: crank power %.1f kW with N2O (progressive flow), %.1f kW without, gain %+.1f kW\n", rpm, pw * 1e-3, pb * 1e-3, (pw - pb) * 1e-3);
    }
}

// ===========================================================================
// Asserting tests. Bounds are the POST-N3 measurement (release, Windows, 2026-10-06) with margin.
//
// Dyno-held shot, data/vehicles/car_hyper_n2o.json (kit 3.6 mm / 1.45 mm, bottle 950 psig, 6 deg retard; settle 10 s, mean of 0.5 s):
//   rpm   base kW   shot kW   gain kW (hp)   boost base/shot   lambda_c  T_mix K  N2O g/s  kit fuel g/s  spark  turbo rpm shot
//   3000   387.0     925.8    +538.7 (722)   1.78/1.79         0.839     206.7    220.0    28.3          12.0    79100
//   4000   534.9    1080.3    +545.4 (731)   1.78/1.78         0.837     225.1    220.0    28.3          12.0    82607
//   5000   723.5    1343.1    +619.6 (831)   1.76/2.12         0.832     246.6    220.0    28.3           8.5   107081
//   6000   867.0    1549.8    +682.8 (916)   1.73/2.37         0.829     258.6    220.0    28.3           4.8   180000 (model cap)
//   7000   977.1    1599.1    +622.0 (834)   1.71/2.02         0.829     258.7    220.0    28.3          10.0   180000 (model cap)
//   mean gain +601.7 kW = 807 hp (target 800 hp = 596.6 kW). No overflow (N2O alone never fills the cylinder). Bottle 6.804 kg lasts 30.9 s.
// The shot is limited by the jet/bottle flow, not by the charge model (the overflow guard never fires on this 5.07 L boosted V8); after N3
// the shot also lifts the charge pressure (2.0-2.4 bar against the 1.8 bar target) until the turbine reaches the model's 180000 rpm cap.
// PRE-N3 for comparison (3.8/1.5 mm, same files otherwise): +598/+605/+585/+556/+594 kW at 3000..7000 rpm, boost unchanged (1.71-1.78).
//
// Whole-car pull (3rd gear, armed at the 3rd-gear clutch lock, 3180 rpm, progressive 0.4 -> 1.0 over 1 s): the clutch stays locked
// (88 of 88 ticks, 0.0 rpm slip), rear wheelspin max kappa 1.30 (baseline 0.80; physics, not fixed), the 7800 rpm mark is reached at t 5.22 s
// against 6.16 s without spray; balance_error 4.0e-16, bottle closure 8e-16 kg, N2O energy closure 2.2e-15.
// ===========================================================================
namespace {

struct Loaded {
    vehicle::VehicleDesc desc;
    const drivetrain::SimulatedEngineDesc* engine = nullptr;
    const drivetrain::NitrousKitDesc* kit = nullptr;
    const drivetrain::N2oBottleDesc* bottle = nullptr;
    const drivetrain::ClutchDesc* clutch = nullptr;
};

void bind(Loaded& l) {
    l.engine = find_component<drivetrain::SimulatedEngineDesc>(l.desc);
    l.kit = find_component<drivetrain::NitrousKitDesc>(l.desc);
    l.bottle = find_component<drivetrain::N2oBottleDesc>(l.desc);
    l.clutch = find_component<drivetrain::ClutchDesc>(l.desc);
}

real jet_flow(const drivetrain::NitrousKitDesc& k, const drivetrain::OxidizerTables& tabs, real bottle_abs_pa) {
    return k.nitrous_jet_cd * kPi / 4.0 * k.nitrous_jet_diameter_mm * k.nitrous_jet_diameter_mm * 1e-6 * tabs.g_hem(bottle_abs_pa);
}

} // namespace

// Cheap parts check with Preview-quality maps (no [long]): the data files and the flow arithmetic, plus one coarse dyno point.
TEST_CASE("N7 car_hyper_n2o: parts and flow arithmetic and a coarse shot", "[n2o][vehicle][drivetrain][validation]") {
    Loaded a, stock;
    a.desc = load_car("car_hyper_n2o.json", drivetrain::MapQuality::Preview);
    stock.desc = load_car("car_hyper.json", drivetrain::MapQuality::Preview);
    bind(a);
    bind(stock);
    REQUIRE(a.engine != nullptr);
    REQUIRE(a.kit != nullptr);
    REQUIRE(a.bottle != nullptr);
    REQUIRE(a.clutch != nullptr);
    REQUIRE(stock.clutch != nullptr);
    CHECK(stock.kit == nullptr);
    CHECK(stock.bottle == nullptr);
    // car_hyper.json stays stock; car_hyper_n2o.json adds the two parts and two fluid links, and raises ONLY the clutch.
    CHECK(a.desc.powertrain.components.size() == stock.desc.powertrain.components.size() + 2);
    CHECK(a.desc.powertrain.fluid.size() == stock.desc.powertrain.fluid.size() + 2);
    CHECK(stock.clutch->capacity_nm == 1650.0);
    CHECK(a.clutch->capacity_nm == 3400.0);
    // The kit: every number as shipped (data/nitrous_kits/custom_wet_plate_800.json).
    CHECK(a.kit->type == drivetrain::NitrousKitType::Wet);
    CHECK(a.kit->nozzle_count == 1);
    CHECK(a.kit->nitrous_jet_diameter_mm == 3.6);
    CHECK(a.kit->nitrous_jet_cd == 0.66);
    CHECK(a.kit->flow_model == drivetrain::NitrousFlowModel::Hem);
    CHECK(a.kit->fuel_jet_diameter_mm == 1.45);
    CHECK(a.kit->fuel_jet_cd == 0.7);
    CHECK(a.kit->fuel_pressure_gauge_bar == 4.0);
    CHECK(a.kit->fuel_reference == drivetrain::NitrousFuelReference::Manifold);
    CHECK(a.kit->rating.power_kw == Catch::Approx(800.0 * kHpW * 1e-3).epsilon(1e-6));
    CHECK(a.kit->controller.arm_channel == "nitrous_arm");
    CHECK(a.kit->controller.arm_default == 0.0);
    CHECK(a.kit->controller.min_gear == 2);
    CHECK(a.kit->controller.min_rpm == 3000.0);
    CHECK(a.kit->controller.max_rpm == 8100.0);
    CHECK(a.kit->controller.progressive.start_fraction == 0.4);
    CHECK(a.kit->controller.progressive.ramp_s == 1.0);
    CHECK(a.kit->controller.spark_retard_deg == 6.0);
    CHECK(a.kit->controller.safety.max_active_s == 35.0);
    CHECK(a.bottle->capacity_kg == Catch::Approx(15.0 * 0.45359237).epsilon(1e-4)); // 15 lb
    CHECK(a.bottle->initial_kg == a.bottle->capacity_kg);
    CHECK(a.bottle->pressure.gauge_bar == Catch::Approx(950.0 * 0.0689476).epsilon(1e-4)); // 950 psig
    // Flow = Cd * A * G_HEM(p_bottle); the bottle lasts 30.9 s at full flow, less than the controller's own 35 s cut.
    const drivetrain::OxidizerTables tabs(a.bottle->oxidizer);
    const real p_abs = (a.bottle->pressure.gauge_bar + 1.01325) * 1e5;
    const real mdot = jet_flow(*a.kit, tabs, p_abs);
    CHECK(mdot * 1e3 == Catch::Approx(220.0).margin(0.5));
    const real duration = a.bottle->capacity_kg / mdot;
    CHECK(duration > 30.5);
    CHECK(duration < 31.3);
    CHECK(duration < a.kit->controller.safety.max_active_s);
    // 800 hp sizing: the shot's expected crank work at 2.46 MJ per kg N2O (out/n2o_plan.md section 9) is within -10 %/+15 % of 597 kW.
    CHECK(mdot * 2.46e6 * 1e-3 > 0.9 * a.kit->rating.power_kw);
    CHECK(mdot * 2.46e6 * 1e-3 < 1.15 * a.kit->rating.power_kw);

    // One coarse dyno point (Preview maps: the numbers are coarser than the Full-quality [long] table, the bounds wide).
    const auto maps = drivetrain::SimulatedEngine::build_maps(*a.engine);
    const KitSpec spec = spec_from(*a.kit, *a.bottle);
    const ShotRes b = dyno_shot(*a.engine, maps, tabs, 4000.0, nullptr, 3.0, 0.25);
    const ShotRes w = dyno_shot(*a.engine, maps, tabs, 4000.0, &spec, 3.0, 0.25);
    print_row(4000.0, b, w);
    CHECK(w.overflow == 0);
    CHECK(w.mdot_n == Catch::Approx(mdot).epsilon(1e-12));
    CHECK(w.kw - b.kw > 450.0); // measured +550.5 kW (Preview maps; the Full-quality table gives +545.4 kW at this rpm)
    CHECK(w.kw - b.kw < 650.0);
    CHECK(w.lambda_c > 0.75);
    CHECK(w.lambda_c < 0.95);
    CHECK(w.t_mix < b.t_charge - 20.0);
}

TEST_CASE("N7 car_hyper_n2o: dyno-held shot at 3000-7000 rpm", "[n2o][vehicle][drivetrain][validation][long]") {
    Loaded a;
    a.desc = load_car("car_hyper_n2o.json");
    bind(a);
    REQUIRE(a.engine != nullptr);
    REQUIRE(a.kit != nullptr);
    REQUIRE(a.bottle != nullptr);
    REQUIRE(a.clutch != nullptr);
    const auto maps = drivetrain::SimulatedEngine::build_maps(*a.engine);
    const drivetrain::OxidizerTables tabs(a.bottle->oxidizer);
    const KitSpec spec = spec_from(*a.kit, *a.bottle);
    const real mdot = jet_flow(*a.kit, tabs, spec.bottle_abs_pa);
    struct Band {
        real rpm, gain_lo, gain_hi;
    };
    const Band bands[] = {{3000, 495, 585}, {4000, 500, 590}, {5000, 570, 670}, {6000, 630, 735}, {7000, 575, 670}};
    real gain_sum = 0.0, peak_torque = 0.0;
    for (const Band& bd : bands) {
        const ShotRes b = dyno_shot(*a.engine, maps, tabs, bd.rpm, nullptr);
        const ShotRes w = dyno_shot(*a.engine, maps, tabs, bd.rpm, &spec);
        print_row(bd.rpm, b, w);
        const real gain = w.kw - b.kw;
        gain_sum += gain;
        peak_torque = std::max(peak_torque, w.brake_nm);
        INFO("rpm " << bd.rpm);
        CHECK(gain > bd.gain_lo);
        CHECK(gain < bd.gain_hi);
        CHECK(w.overflow == 0); // the charge model does not limit this shot
        CHECK(w.lambda_c > 0.80);
        CHECK(w.lambda_c < 0.87);
        CHECK(w.t_mix > 190.0); // evaporative cooling of the N2O: ~90 K below the 295 K charge
        CHECK(w.t_mix < 275.0);
        CHECK(w.mdot_n == Catch::Approx(mdot).epsilon(1e-12));
        CHECK(w.mdot_n * 1e3 == Catch::Approx(220.0).margin(0.5));
        CHECK(w.mdot_kf * 1e3 > 26.5);
        CHECK(w.mdot_kf * 1e3 < 30.5);
        CHECK(w.brake_max - w.brake_min < 80.0);  // settled, not hunting
        CHECK(w.boost_bar >= b.boost_bar - 0.02); // the shot never LOWERS boost after N3
        CHECK(w.turbo_rpm <= 180000.0 + 1.0);
    }
    const real mean_gain = gain_sum / 5.0;
    std::printf("[N7 dyno] mean gain %.1f kW = %.0f hp (target 596.6 kW = 800 hp), peak shot crank torque %.0f Nm vs clutch %.0f Nm\n", mean_gain,
                mean_gain * 1e3 / kHpW, peak_torque, a.clutch->capacity_nm);
    CHECK(mean_gain > 560.0);
    CHECK(mean_gain < 650.0);
    // The clutch carries the shot with a >= 10 % margin on the peak crank torque (3000 rpm, the controller's lowest rpm).
    CHECK(1.1 * peak_torque <= a.clutch->capacity_nm);
    // No-spray reference of the stock engine (unchanged by the kit).
    const ShotRes b4 = dyno_shot(*a.engine, maps, tabs, 4000.0, nullptr);
    const ShotRes b6 = dyno_shot(*a.engine, maps, tabs, 6000.0, nullptr);
    CHECK(b4.kw > 510.0);
    CHECK(b4.kw < 560.0);
    CHECK(b6.kw > 830.0);
    CHECK(b6.kw < 905.0);
}

TEST_CASE("N7 car_hyper_n2o: whole-car WOT pull with the shot", "[n2o][vehicle][drivetrain][validation][long]") {
    const vehicle::VehicleDesc desc = load_car("car_hyper_n2o.json");
    const PullResult with = pull(desc, true);
    const PullResult base = pull(desc, false);
    std::printf("[N7 pull] armed %.0f rpm, %.0f rpm at %.2f s (baseline %.2f s), sprayed %d ticks, N2O %.1f g, balance %.2e, bottle %.2e, tank %.2e, closure %.2e, locked %d unlocked %d, max slip %.2f rpm, kappa %.2f (baseline %.2f)\n",
                with.arm_rpm, kEndRpm, with.t_end, base.t_end, with.spray_ticks, with.n2o_kg * 1e3, with.worst_balance, with.worst_bottle, with.worst_tank, with.worst_closure,
                with.locked_ticks, with.unlocked_ticks, with.max_slip_rpm, with.max_kappa, base.max_kappa);
    // No stall, the clutch locked on every pull tick (the 3400 N*m clutch carries the shot), nothing else in the driveline slips.
    CHECK_FALSE(with.stalled);
    CHECK_FALSE(base.stalled);
    CHECK(with.locked_ticks >= 50);
    CHECK(with.unlocked_ticks == 0);
    CHECK(with.max_slip_rpm < 1.0);
    CHECK(with.spray_ticks >= 50);
    // The shot sprayed (progressive flow, 0.4 -> 1.0 over 1 s, so about 40 g in the 0.36 s pull); the bottle drops by exactly what flowed.
    CHECK(with.n2o_kg * 1e3 > 30.0);
    CHECK(with.n2o_kg * 1e3 < 50.0);
    CHECK(with.unburnt_kg < 1e-9); // the whole N2O is accounted as burnt charge (nothing overflows)
    // Ledgers: the powertrain energy balance per tick (PLAN.md bound 5e-8), the N2O bottle, the fuel tank, the N2O/fuel energy closure.
    CHECK(with.worst_balance <= 5e-8);
    CHECK(base.worst_balance <= 5e-8);
    CHECK(with.worst_bottle <= 1e-12);
    CHECK(with.worst_tank <= 2e-12); // measured 1.3e-13 kg against a ~60 kg tank (double rounding of the running sums)
    CHECK(with.worst_closure <= 1e-9);
    // The spray gets to the end rpm sooner; the crank power with the shot is higher at every sampled rpm. The wheelspin it causes is physics:
    // reported, only bounded here against a runaway (the tyre model clamps the slip ratio at +-3).
    CHECK(with.t_end < base.t_end - 0.5);
    for (real rpm : {5000.0, 6000.0, 7000.0}) {
        const real gain_kw = (power_at(with, rpm) - power_at(base, rpm)) * 1e-3;
        std::printf("[N7 pull] %.0f rpm: gain %+.1f kW (partial flow)\n", rpm, gain_kw);
        INFO("rpm " << rpm);
        CHECK(gain_kw > 50.0);
    }
    CHECK(with.max_kappa < 2.0);
    CHECK(with.max_kappa >= base.max_kappa);
}

// The 3rd-gear pull above is short (0.36 s, rear wheelspin). In 5th the tyres hold (max kappa 0.24), the progressive flow reaches its full
// 220 g/s at t = 1 s and the pull lasts 1.2 s with the shot against 4.8 s without (5646 -> 7800 rpm); measured post-N3: clutch locked 296/296,
// 204 g N2O, crank power gain +366 kW at 6000 rpm and +460 kW at 7000 rpm over the baseline pull, v_end 67.4 m/s (baseline 80.2 m/s, the
// baseline pull simply lasts 3.6 s longer: not a speed comparison), balance 4.0e-16, bottle 3.2e-14 kg, tank 2.1e-13 kg, closure 5.2e-15.
TEST_CASE("N7 car_hyper_n2o: whole-car WOT pull in 5th gear reaches full flow", "[n2o][vehicle][drivetrain][validation][long]") {
    const vehicle::VehicleDesc desc = load_car("car_hyper_n2o.json");
    constexpr int kGear = 5;
    const PullResult with = pull(desc, true, kGear);
    const PullResult base = pull(desc, false, kGear);
    real max_flow = 0.0;
    for (const Sample& s : with.pull) max_flow = std::max(max_flow, s.flow_g_s);
    std::printf("[N7 pull5] armed %.0f rpm, %.0f rpm after %.2f s (baseline %.2f s), sprayed %d ticks, N2O %.1f g, max flow %.1f g/s, balance %.2e, bottle %.2e, tank %.2e, closure %.2e, locked %d unlocked %d, max slip %.2f rpm, kappa %.2f (baseline %.2f), v_end %.1f (baseline %.1f)\n",
                with.arm_rpm, kEndRpm, with.t_end - with.t_arm, base.t_end - base.t_arm, with.spray_ticks, with.n2o_kg * 1e3, max_flow, with.worst_balance, with.worst_bottle,
                with.worst_tank, with.worst_closure, with.locked_ticks, with.unlocked_ticks, with.max_slip_rpm, with.max_kappa, base.max_kappa, with.v_end, base.v_end);
    CHECK_FALSE(with.stalled);
    CHECK_FALSE(base.stalled);
    CHECK(with.locked_ticks >= 200);
    CHECK(with.unlocked_ticks == 0);
    CHECK(with.max_slip_rpm < 1.0);
    CHECK(with.spray_ticks >= 200);
    CHECK(with.max_kappa < 0.6); // the tyres hold in 5th: no wheelspin runaway
    // Full flow (Cd*A*G_HEM = 220 g/s) is reached once the 1 s progressive ramp ends, never exceeded.
    CHECK(max_flow > 215.0);
    CHECK(max_flow < 225.0);
    CHECK(with.n2o_kg * 1e3 > 170.0);
    CHECK(with.n2o_kg * 1e3 < 240.0);
    CHECK(with.unburnt_kg < 1e-9);
    CHECK(with.worst_balance <= 5e-8);
    CHECK(with.worst_bottle <= 1e-12);
    CHECK(with.worst_tank <= 2e-12);
    CHECK(with.worst_closure <= 1e-9);
    // The shot cuts the time to 7800 rpm to a quarter; crank power gain at 6000/7000 rpm (the mean over the 500 rpm bins).
    CHECK(with.t_end - with.t_arm < 1.6);
    CHECK(base.t_end - base.t_arm > 3.5);
    const real g6 = (power_at(with, 6000.0) - power_at(base, 6000.0)) * 1e-3;
    const real g7 = (power_at(with, 7000.0) - power_at(base, 7000.0)) * 1e-3;
    std::printf("[N7 pull5] crank power gain %+.1f kW at 6000 rpm, %+.1f kW at 7000 rpm\n", g6, g7);
    CHECK(g6 > 300.0);
    CHECK(g6 < 430.0);
    CHECK(g7 > 380.0);
    CHECK(g7 < 540.0);
}
