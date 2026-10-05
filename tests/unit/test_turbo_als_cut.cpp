// tests/unit/test_turbo_als_cut.cpp -- I7d D5 extension: a rally-style anti-lag
// (ALS) - cylinder cut with the fuel still injected, a deep absolute spark retard,
// enrichment and a held boost target - on top of the exhaust energy balance
// (test_turbo_exhaust_thermal.cpp). docs/data_formats/turbo.md "anti_lag".
//
// What is modelled (engine_simulated.cpp, evaluate_torque_impl and the exhaust
// split): while anti-lag is active
//   lambda    = anti_lag.lambda when > 0 (else the ECU target): fuel per charge
//               m_air/(lambda*AFR) and f_lambda follow it;
//   spark     = spark_advance_deg (absolute) or scheduled - retard_deg; below the
//               eta_spark table's most retarded node (40 deg after MBT) eta_spark is
//               extrapolated along the first segment, floored at 0 (PROVISIONAL);
//   cut       = cylinder_cut_fraction of the injected charges are not fired:
//               tau_comb = (1 - cut) * eta_i f_lambda eta_spark LHV/2pi m_fuel_rev,
//               the booked fuel is the whole injected m_fuel_rev;
//   exhaust   = released R = min(1, lambda) Q_f; the cut share cut*R passes the
//               cylinder unburnt and manifold_burn_fraction b of it is released in
//               the hot manifold (b*cut*R, part of the sensible enthalpy), the rest
//               stays chemical; in-cylinder wall heat f*(1 - cut)*R;
//   wastegate = the boost PID's plate input is max(plate, boost_target_fraction).
// Not modelled: a manifold ignition threshold (the cut charge burns whatever the
// wall temperature), knock/misfire limits, a lambda- or temperature-dependent
// burn fraction, ECU material protection.
//
// Tests
//   AC1  evaluate_torque against literal reconstructions (cut, absolute spark with
//        extrapolation, enrichment, tau_comb_unretarded); nullptr = no change.
//   AC2  exhaust split with a cut: closure and literal manifold/chemical/wall terms.
//   AC3  boost target: with a plate floor below 10 % the wastegate is commanded
//        open without it and held with it.
//   AC4  hashing of the extension fields; an un-extended block keeps its layout.
//   AC5  loader values and rejections.
//   AC6  hyper V8 at 6000 rpm (measured, tests/support/turbo_fixtures.h
//        hyper_like_als_cut): ALS air 10-20 % of WOT air, crank torque in [-80, 0]
//        N*m, rotor far above the fuel-cut coast at +0.25 s and +1 s (and above it at
//        +0.5 s), at least the D5 fixture's at +1 s, fuel below 0.6 x the D5 fixture's.
//        Measured after the review fix (parallel wastegate, compressor bound; shipped
//        2e-5 rotor; mean +0.25..+1 s): air 0.1211 kg/s = 14.8 % of WOT (0.8187),
//        fuel 10.03 g/s = 14.5 % of WOT (69.10 g/s), net -63.9 N*m, boost -0.552 bar,
//        wastegate 0.094, manifold burn 146.9 kW, port 2279 K, turbine inlet 1383 K,
//        rotor 102248 / 60888 / 37436 / 58417 rpm at +0 / +0.25 / +0.5 / +1 s - the
//        boost-target PID overshoots at the lift (rotor 174k rpm at +80 ms, gate to
//        0.92), the small rotor dips to 37k at +0.47 s and recovers. D5 fixture: air
//        59.7 %, fuel 36.64 g/s, net +609.6 N*m, rotor 62009 / 54879 / 57528. No
//        anti-lag: 23102 / 19809 / 14646 (was 0 from +0.25 s with the 100 % bypass).
//        Before the fix: air 18.4 %, fuel 12.39 g/s, net -54.3 N*m, rotor 51699 /
//        59820 / 70012.
//   AC3  measured: no target wastegate 0.9999, rotor 24474; target 0.5: 0.0008,
//        25880 rpm (+1 s, 5000 rpm, fixture engine).
//
// Sabotages (each built and run on AC1-AC6, 2026-10-05; failing cases):
//   eta_spark extrapolation removed (table clamps)        AC1, AC6 (rotor +0.5 s 53097 < 55631)
//   cut ignored in tau_comb                                AC1, AC6 (net -14.7 -> rotor 52271 < 55631)
//   cut also removes the booked fuel                       AC1 (m_fuel_rev), AC6
//   lambda override ignored                                AC1
//   absolute spark ignored (scheduled - 30 deg instead)    AC1, AC6
//   manifold burn not added to the sensible enthalpy       AC2, AC6 (rotor +1 s 19362)
//   unburnt cut share not booked as chemical               AC2 (closure)
//   wall heat on the whole released energy                 AC2
//   boost target not passed to the wastegate PID           AC3 (wastegate 0.9999), AC6 (air 8.9 %)
//   extension fields not hashed                            AC4
//   loader retard_deg/spark_advance_deg exclusivity gone   AC5

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps::drivetrain;
using ps::real;
using json = nlohmann::json;
using Catch::Matchers::ContainsSubstring;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

constexpr real kH = 1.0 / 960.0;
constexpr real kPi = 3.14159265358979323846;
constexpr real kFwall = 0.20; // the [AS] default in_cylinder_wall_heat_fraction, typed in

real omega_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

SimulatedEngineDesc load_hyper() {
    static ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    return std::get<SimulatedEngineDesc>(ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
}

// Lift at a held crank: WOT for n_wot substeps, then command 0 with the channel on.
struct LiftStats {
    real wot_air = 0, wot_fuel = 0, wot_net = 0;                 // last 0.25 s of WOT (kg/s, g/s, N*m)
    real als_air = 0, als_fuel = 0, als_net = 0, als_boost = 0;  // mean over +0.25..+1.0 s
    real als_port = 0, als_turbine = 0, als_manifold_w = 0, als_wastegate = 0, als_plate = 0;
    real rotor[4] = {0, 0, 0, 0};                                // +0 / +0.25 / +0.5 / +1 s
    int active = 0;
};
LiftStats run_lift(const SimulatedEngineDesc& d, const std::shared_ptr<const SimulatedEngineMaps>& maps, real rpm, real channel) {
    SimulatedEngine e(d, maps);
    const real omega = omega_of(rpm);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = channel;
    const int n_wot = 2 * 960, n_lift = 960;
    LiftStats out;
    int n_wot_avg = 0, n_als_avg = 0;
    for (int i = 0; i < n_wot + n_lift; ++i) {
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < n_wot ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        const real air = r.exhaust_mass_flow_kg_s - r.fuel_flow_g_s * 1e-3;
        const real net = r.tau0 - maps->c_k;
        if (i >= n_wot - 240 && i < n_wot) { out.wot_air += air; out.wot_fuel += r.fuel_flow_g_s; out.wot_net += net; ++n_wot_avg; }
        const int m = i - (n_wot - 1);
        if (m == 0) out.rotor[0] = r.turbo_rpm;
        if (m == 240) out.rotor[1] = r.turbo_rpm;
        if (m == 480) out.rotor[2] = r.turbo_rpm;
        if (m == 960) out.rotor[3] = r.turbo_rpm;
        if (r.anti_lag_active) ++out.active;
        if (m >= 240 && m <= 960) {
            out.als_air += air; out.als_fuel += r.fuel_flow_g_s; out.als_net += net; out.als_boost += r.boost_bar;
            out.als_port += r.exhaust_port_k; out.als_turbine += r.turbine_inlet_k; out.als_manifold_w += r.anti_lag_manifold_burn_w;
            out.als_wastegate += r.wastegate_opening; out.als_plate += r.throttle_position;
            ++n_als_avg;
        }
    }
    out.wot_air /= n_wot_avg; out.wot_fuel /= n_wot_avg; out.wot_net /= n_wot_avg;
    for (real* v : {&out.als_air, &out.als_fuel, &out.als_net, &out.als_boost, &out.als_port, &out.als_turbine, &out.als_manifold_w,
                    &out.als_wastegate, &out.als_plate})
        *v /= n_als_avg;
    return out;
}

void print(const char* tag, const LiftStats& x) {
    std::printf("[%s] WOT air %.4f kg/s fuel %.2f g/s net %+.1f N*m | ALS air %.4f kg/s (%.1f %% of WOT) fuel %.2f g/s (%.1f %% of WOT) "
                "net %+.1f N*m boost %+.3f bar plate %.3f wg %.3f port %.0f K turbine %.0f K manifold burn %.0f W | rotor %.0f / %.0f / %.0f / %.0f rpm, active %d\n",
                tag, x.wot_air, x.wot_fuel, x.wot_net, x.als_air, 100.0 * x.als_air / x.wot_air, x.als_fuel, 100.0 * x.als_fuel / x.wot_fuel,
                x.als_net, x.als_boost, x.als_plate, x.als_wastegate, x.als_port, x.als_turbine, x.als_manifold_w, x.rotor[0], x.rotor[1], x.rotor[2],
                x.rotor[3], x.active);
}

json base_config() {
    return json::parse(R"({
      "format": "physics_sim.turbo_configuration/1", "name": "als_cut_test", "turbo": "HARDWARE",
      "count": 2, "target_boost_bar": 1.8, "intercooler_effectiveness": 0.95
    })");
}
json valid_als() {
    return json::parse(R"({
      "arm_below_command": 0.1, "min_rpm": 3000, "max_rpm": 7500, "spark_advance_deg": -35,
      "plate_floor": 0.12, "max_active_s": 1.5, "cylinder_cut_fraction": 0.25, "manifold_burn_fraction": 0.8,
      "lambda": 0.85, "boost_target_fraction": 0.5, "source": "[AS] io test"
    })");
}
TurboPairDesc load(json cfg, const std::string& tag) {
    cfg["turbo"] = std::string(PS_SOURCE_DIR) + "/data/turbos/hyper_turbo_approx.json";
    const auto dir = std::filesystem::temp_directory_path() / "ps_turbo_als_cut_io_test";
    std::filesystem::create_directories(dir);
    const auto path = dir / (tag + ".json");
    std::ofstream(path) << cfg.dump(2);
    return ps::io::load_turbo_configuration_json(path.string());
}

} // namespace

TEST_CASE("AC1: anti-lag cut, absolute spark and enrichment in evaluate_torque match literal reconstructions",
          "[turbo][drivetrain][anti_lag][sim_engine]") {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    const auto maps = SimulatedEngine::build_maps(d);
    SimulatedEngine e(d, maps);
    const auto& es = maps->maps.eta_spark;
    REQUIRE(es.x.size() >= 2);
    for (const real rpm : {3000.0, 5000.0}) {
        for (const real p : {60000.0, 150000.0}) {
            const real w = omega_of(rpm);
            const auto ref = e.evaluate_torque(w, p, true, 1.0, false, nullptr);
            for (const real spark_abs : {10.0, -15.0, -35.0, -45.0}) {
                AntiLagDesc a = tb::hyper_like_als_cut();
                a.spark_advance_deg = spark_abs;
                a.cylinder_cut_fraction = 0.3;
                a.lambda = 0.8;
                const auto ev = e.evaluate_torque(w, p, true, 1.0, false, &a);
                // literal eta_spark incl. the extrapolation below the first node
                const real x = spark_abs - ref.theta_mbt_deg;
                real eta_s = es.evaluate(x);
                if (x < es.x.front()) eta_s = std::max(0.0, es.y[0] + (es.y[1] - es.y[0]) / (es.x[1] - es.x[0]) * (x - es.x[0]));
                const real f_l = maps->maps.f_lambda.evaluate(0.8);
                const real m_fuel = ref.m_air_rev / (0.8 * maps->afr_stoich);
                const real lhv_2pi = maps->lhv_j_kg / (2.0 * kPi);
                CHECK(ev.lambda == 0.8);
                CHECK(ev.spark_deg == spark_abs);
                CHECK(ev.cut_fraction == 0.3);
                CHECK(ev.eta_spark == Catch::Approx(eta_s).epsilon(1e-14).margin(1e-15));
                CHECK(ev.m_fuel_rev == Catch::Approx(m_fuel).epsilon(1e-14));
                CHECK(ev.tau_comb == Catch::Approx(0.7 * ref.eta_i * f_l * eta_s * lhv_2pi * m_fuel).epsilon(1e-13).margin(1e-12));
                const real eta_sched = es.evaluate(ref.spark_deg - ref.theta_mbt_deg);
                CHECK(ev.tau_comb_unretarded == Catch::Approx(ref.eta_i * f_l * eta_sched * lhv_2pi * m_fuel).epsilon(1e-13));
                if (spark_abs <= -35.0) CHECK(x < es.x.front()); // the deep settings exercise the extrapolation
            }
            // an un-extended D5 block (retard only) and nullptr: unchanged paths
            const auto ev0 = e.evaluate_torque(w, p, true, 1.0, false, nullptr);
            CHECK(ev0.tau == ref.tau);
            CHECK(ev0.cut_fraction == 0.0);
        }
    }
}

TEST_CASE("AC2: exhaust split with a cylinder cut - closure and literal manifold, chemical and wall terms",
          "[turbo][drivetrain][anti_lag][sim_engine][exhaust_thermal]") {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    AntiLagDesc a = tb::hyper_like_als_cut();
    a.manifold_burn_fraction = 0.7;
    d.turbo_pair->anti_lag = a;
    SimulatedEngine e(d);
    const real omega = omega_of(5000.0);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = 1.0;
    int active = 0;
    real worst = 0.0, max_port = 0.0, manifold_seen = 0.0;
    for (int i = 0; i < 3 * 960; ++i) {
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < 1920 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        if (!r.anti_lag_active) continue;
        ++active;
        const auto& x = e.exhaust_split();
        const real eta_c = std::min(1.0, r.lambda);
        const real released = eta_c * x.fuel_w;
        const real c = a.cylinder_cut_fraction;
        const real man = 0.7 * c * released;
        const real chem = (1.0 - eta_c) * x.fuel_w + 0.3 * c * released;
        const real wall = kFwall * (1.0 - c) * released;
        const real sens = (1.0 - c) * released - x.indicated_w - wall + man;
        const real scale = std::max(1.0, x.fuel_w);
        for (const real err : {std::abs(x.manifold_w - man), std::abs(x.chemical_w - chem), std::abs(x.wall_w - wall),
                               std::abs(x.sensible_w - sens), std::abs(x.indicated_w + x.wall_w + x.sensible_w + x.chemical_w - x.clamp_w - x.fuel_w)})
            worst = std::max(worst, err / scale);
        CHECK(r.anti_lag_manifold_burn_w == x.manifold_w);
        max_port = std::max(max_port, r.exhaust_port_k);
        manifold_seen = std::max(manifold_seen, x.manifold_w);
        CHECK(x.clamp_w == 0.0);
    }
    std::printf("[AC2] %d active substeps, worst literal error %.2e of the fuel power, max manifold burn %.0f W, max port %.1f K\n", active,
                worst, manifold_seen, max_port);
    CHECK(active == 960);
    CHECK(worst <= 1e-13);
    CHECK(manifold_seen > 1000.0);
}

TEST_CASE("AC3: the anti-lag boost target holds the wastegate under a plate floor below 10 %", "[turbo][drivetrain][anti_lag][sim_engine]") {
    const auto run = [](real target) {
        auto d = fx::electronic_engine();
        d.turbo_pair = tb::hyper_like_pair();
        AntiLagDesc a = tb::hyper_like_als_cut();
        a.plate_floor = 0.08;
        a.boost_target_fraction = target;
        d.turbo_pair->anti_lag = a;
        SimulatedEngine e(d);
        const real omega = omega_of(5000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.anti_lag_channel = 1.0;
        PreSolveResult r;
        for (int i = 0; i < 1920 + 960; ++i) {
            r = e.pre_solve(s, ThrottleCommand{i < 1920 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
        }
        return r;
    };
    const PreSolveResult none = run(0.0), held = run(0.5);
    std::printf("[AC3] +1.0 s, plate %.3f: no target wastegate %.4f charge %.0f Pa rotor %.0f; target 0.5 wastegate %.4f charge %.0f Pa rotor %.0f\n",
                held.throttle_position, none.wastegate_opening, none.p_charge_pa, none.turbo_rpm, held.wastegate_opening, held.p_charge_pa,
                held.turbo_rpm);
    REQUIRE(none.anti_lag_active);
    REQUIRE(held.anti_lag_active);
    CHECK(none.wastegate_opening > 0.99); // commanded open below a 10 % plate
    CHECK(held.wastegate_opening < 0.9);
    CHECK(held.turbo_rpm > none.turbo_rpm);
}

TEST_CASE("AC4: anti-lag extension fields are hashed, an un-extended block keeps its layout", "[turbo][drivetrain][anti_lag][sim_engine]") {
    const auto hash_of = [](const AntiLagDesc& a) {
        auto d = fx::electronic_engine();
        d.turbo_pair = tb::hyper_like_pair();
        d.turbo_pair->anti_lag = a;
        SimulatedEngine e(d);
        return e.fold_state_hash(0);
    };
    const AntiLagDesc d5 = tb::hyper_like_anti_lag();
    AntiLagDesc same = d5;
    same.manifold_burn_fraction = 1.0; // the default: still no extension
    CHECK_FALSE(same.has_extension());
    CHECK(hash_of(same) == hash_of(d5));
    const std::uint64_t h0 = hash_of(d5);
    AntiLagDesc a = d5; a.spark_advance_deg = -35.0; CHECK(hash_of(a) != h0);
    AntiLagDesc b = d5; b.cylinder_cut_fraction = 0.25; CHECK(hash_of(b) != h0);
    AntiLagDesc c = d5; c.manifold_burn_fraction = 0.9; CHECK(hash_of(c) != h0);
    AntiLagDesc l = d5; l.lambda = 0.85; CHECK(hash_of(l) != h0);
    AntiLagDesc t = d5; t.boost_target_fraction = 0.5; CHECK(hash_of(t) != h0);
    AntiLagDesc s1 = d5; s1.spark_advance_deg = -35.0;
    AntiLagDesc s2 = d5; s2.spark_advance_deg = -36.0;
    CHECK(hash_of(s1) != hash_of(s2));
}

TEST_CASE("AC5: anti_lag extension loader - values and rejections", "[turbo][io][anti_lag]") {
    {
        json c = base_config();
        c["anti_lag"] = valid_als();
        const TurboPairDesc d = load(c, "valid");
        REQUIRE(d.anti_lag.has_value());
        REQUIRE(d.anti_lag->spark_advance_deg.has_value());
        CHECK(*d.anti_lag->spark_advance_deg == -35.0);
        CHECK(d.anti_lag->retard_deg == 0.0);
        CHECK(d.anti_lag->cylinder_cut_fraction == 0.25);
        CHECK(d.anti_lag->manifold_burn_fraction == 0.8);
        CHECK(d.anti_lag->lambda == 0.85);
        CHECK(d.anti_lag->boost_target_fraction == 0.5);
        json c2 = base_config();
        json a2 = valid_als();
        for (const char* k : {"spark_advance_deg", "cylinder_cut_fraction", "manifold_burn_fraction", "lambda", "boost_target_fraction"}) a2.erase(k);
        a2["retard_deg"] = 30;
        c2["anti_lag"] = a2;
        const TurboPairDesc d2 = load(c2, "plain");
        CHECK_FALSE(d2.anti_lag->has_extension());
        CHECK(d2.anti_lag->retard_deg == 30.0);
    }
    const auto reject = [](const char* key, const json& value, const char* message) {
        json c = base_config();
        json a = valid_als();
        if (value.is_null()) a.erase(key);
        else a[key] = value;
        c["anti_lag"] = a;
        CHECK_THROWS_WITH(load(c, std::string("bad_") + key), ContainsSubstring(message));
    };
    reject("retard_deg", 30.0, "give either 'retard_deg' or 'spark_advance_deg', not both");
    reject("spark_advance_deg", nullptr, "'retard_deg' is required");
    reject("spark_advance_deg", -95.0, "'spark_advance_deg' must be in [-90, 60]");
    reject("cylinder_cut_fraction", 1.0, "'cylinder_cut_fraction' must be in [0, 1)");
    reject("cylinder_cut_fraction", -0.1, "'cylinder_cut_fraction' must be in [0, 1)");
    reject("manifold_burn_fraction", 1.1, "'manifold_burn_fraction' must be in [0, 1]");
    reject("lambda", 0.3, "'lambda' must be in [0.5, 1.5]");
    reject("boost_target_fraction", 1.5, "'boost_target_fraction' must be in [0, 1]");
    reject("cut_fraction", 0.2, "unknown key 'cut_fraction'");
}

TEST_CASE("AC6: hyper V8 rally ALS at 6000 rpm - realistic bypass air, crank torque near zero (measured)",
          "[turbo][drivetrain][anti_lag][sim_engine][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    const auto maps = SimulatedEngine::build_maps(base);
    SimulatedEngineDesc d5 = base, als = base;
    d5.turbo_pair->anti_lag = tb::hyper_like_anti_lag();
    als.turbo_pair->anti_lag = tb::hyper_like_als_cut();
    const LiftStats off = run_lift(d5, maps, 6000.0, 0.0);
    const LiftStats x5 = run_lift(d5, maps, 6000.0, 1.0);
    const LiftStats x = run_lift(als, maps, 6000.0, 1.0);
    print("AC6 off", off);
    print("AC6 D5 fixture", x5);
    print("AC6 rally ALS", x);
    REQUIRE(x.active == 960);
    const real air_frac = x.als_air / x.wot_air;
    const real fuel_frac = x.als_fuel / x.wot_fuel;
    CHECK(air_frac >= 0.10);
    CHECK(air_frac <= 0.20);
    // crank torque near zero or slightly negative (a car does not pull on the lift)
    CHECK(x.als_net >= -80.0);
    CHECK(x.als_net <= 0.0);
    // the turbine keeps spinning: far above the fuel-cut coast (above it through the +0.5 s dip after the
    // boost-target overshoot), at least the D5 fixture at +1 s
    CHECK(x.rotor[1] > off.rotor[1] + 20000.0);
    CHECK(x.rotor[2] > off.rotor[2] + 10000.0);
    CHECK(x.rotor[3] > off.rotor[3] + 20000.0);
    CHECK(x.rotor[3] >= x5.rotor[3]);
    // fuel: reported in g/s and against WOT; well below the D5 fixture's 50 % plate
    std::printf("[AC6] ALS fuel %.2f g/s = %.1f %% of WOT (%.2f g/s); air %.1f %% of WOT; D5 fixture fuel %.2f g/s\n", x.als_fuel,
                100.0 * fuel_frac, x.wot_fuel, 100.0 * air_frac, x5.als_fuel);
    CHECK(x.als_fuel < 0.6 * x5.als_fuel);
    CHECK(x.als_manifold_w > 0.0);
}

TEST_CASE("AC diag: hyper V8 ALS sweep at 6000 rpm", "[.][diag][anti_lag][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    const auto maps = SimulatedEngine::build_maps(base);
    {
        SimulatedEngineDesc d = base;
        d.turbo_pair->anti_lag = tb::hyper_like_anti_lag();
        print("AC diag D5 fixture", run_lift(d, maps, 6000.0, 1.0));
        print("AC diag off", run_lift(d, maps, 6000.0, 0.0));
    }
    for (const real floor : {0.12, 0.15, 0.17, 0.20, 0.25}) {
        for (const real spark : {-35.0, -40.0, -45.0}) {
            for (const real cut : {0.0, 0.4}) {
                for (const real target : {0.0, 1.0}) {
                    SimulatedEngineDesc d = base;
                    AntiLagDesc a = tb::hyper_like_als_cut();
                    a.plate_floor = floor;
                    a.spark_advance_deg = spark;
                    a.cylinder_cut_fraction = cut;
                    a.boost_target_fraction = target;
                    d.turbo_pair->anti_lag = a;
                    char tag[96];
                    std::snprintf(tag, sizeof tag, "AC diag floor %.2f spark %+.0f cut %.2f target %.1f", floor, spark, cut, target);
                    print(tag, run_lift(d, maps, 6000.0, 1.0));
                }
            }
        }
    }
}
