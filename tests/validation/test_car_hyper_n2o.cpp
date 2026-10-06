// tests/validation/test_car_hyper_n2o.cpp -- N2O plan N7 (owner "50hp is negligable, make it 800"):
// the hypercar (data/vehicles/car_hyper.json, a twin-turbo 5.07 L V8, engines/hyper_v8_one1_audio.json)
// with an ~800 hp (597 kW) [AS] wet shot, data/vehicles/car_hyper_n2o.json.
//
// NUMBERS ARE "pre-N3": the N2O exhaust heat to the turbine, the retard heat and the N2O exhaust mass flow
// are not in the turbo model yet (N3). The harness below is written to be rerun unchanged after N3 merges.
#include "drivetrain/engine_simulated.h"
#include "drivetrain/nitrous_flow.h"
#include "vehicle_test_helpers.h"

#include "ps/io/nitrous_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

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

vehicle::VehicleDesc load_car(const std::string& file) {
    static jobs::JobSystem pool(10);
    io::EngineMapOptions opt;
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

PullResult pull(const vehicle::VehicleDesc& desc, bool arm) {
    PullResult res;
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kMass, kHalfExtents, kOriginHeight, 0.0, 4);
    World& w = *rig.world;
    const real dt = w.dt();
    ps_test::settle(w, 2.0);
    res.bottle0 = w.n2o_bottle_mass(rig.vehicle);
    const real tank0 = w.fuel_tank_mass(rig.vehicle);
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
        res.worst_tank = std::max(res.worst_tank, std::abs((tank0 - w.fuel_tank_mass(rig.vehicle)) - (e.fuel_mass - e.fuel_external_added)));
        const real in = e.fuel_energy_in + e.n2o_energy_in;
        if (in > 0.0) res.worst_closure = std::max(res.worst_closure, std::abs(in - e.indicated_work - e.engine_heat) / in);
        const auto& es = st.engines.at(0);
        if (es.n2o_active) ++res.spray_ticks;
        const real work = work_of();
        const bool idle = st.shift_phase == drivetrain::ShiftPhase::Idle;
        if (idle && st.gear >= 1 && st.gear < kPullGear && st.engine_rpm >= kShiftRpm) w.set_control("gear_request", static_cast<real>(st.gear + 1));
        const bool in_pull = st.gear == kPullGear && idle && st.clutch_engagement >= 0.999;
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
        if (armed && st.engine_rpm >= kEndRpm) {
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
    const PullResult with = pull(desc, true);
    const PullResult base = pull(desc, false);
    std::printf("[N7 pull] gear %d armed at %.0f rpm (t %.2f s; baseline %.0f rpm), %.0f rpm at t %.2f s (baseline %.2f s); sprayed %d ticks, N2O %.1f g (unburnt %.3f g), kit fuel %.1f g\n",
                kPullGear, with.arm_rpm, with.t_arm, base.arm_rpm, kEndRpm, with.t_end, base.t_end, with.spray_ticks, with.n2o_kg * 1e3, with.unburnt_kg * 1e3, with.kit_fuel_kg * 1e3);
    std::printf("[N7 pull] worst per tick: balance %.2e, bottle %.2e kg, tank %.2e kg, N2O energy closure %.2e; clutch stuck ticks %d unstuck %d, max slip %.1f rpm, max rear kappa %.2f (baseline %.2f), v_end %.1f (baseline %.1f) m/s\n",
                with.worst_balance, with.worst_bottle, with.worst_tank, with.worst_closure, with.locked_ticks, with.unlocked_ticks, with.max_slip_rpm, with.max_kappa, base.max_kappa, with.v_end, base.v_end);
    int n = 0;
    for (const Sample& s : with.pull) {
        if (n++ % 60 == 0)
            std::printf("[N7 pull]   t %.2f rpm %.0f work-rate %.1f kW n2o %.1f g/s kappa %.2f slip %.1f rpm boost %.2f%s\n", s.t, s.rpm, s.crank_work / s.dt * 1e-3, s.flow_g_s, s.kappa, s.clutch_slip_rpm, s.boost_bar, s.stuck ? " stuck" : "");
    }
    for (real rpm : {5000.0, 6000.0, 7000.0}) {
        const real pw = power_at(with, rpm, 1.0), pb = power_at(base, rpm);
        std::printf("[N7 pull] %.0f rpm: crank power %.1f kW with N2O (>=1 s after arm), %.1f kW without, gain %+.1f kW\n", rpm, pw * 1e-3, pb * 1e-3, (pw - pb) * 1e-3);
    }
}
