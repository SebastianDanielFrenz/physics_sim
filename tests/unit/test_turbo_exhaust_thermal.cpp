// tests/unit/test_turbo_exhaust_thermal.cpp -- exhaust energy balance and exhaust
// thermal mass of turbo engines (turbo configuration block "exhaust_thermal",
// docs/data_formats/turbo.md). Replaces the 25 % fuel-heat fraction, the 0.5 s
// peak-hold of the fuel watts and the 1200 K turbine-inlet cap.
//
// Model (engine_simulated.cpp, turbo_pair.h):
//   engine, per substep: Q_f = booked fuel x LHV / h; eta_c = min(1, lambda);
//     released = eta_c Q_f; W_i = omega_old tau_comb; Q_wall = f_wall released;
//     H_sens = released - W_i - Q_wall (>= 0 guard, clamp_w); H_chem = (1 - eta_c) Q_f;
//     T_port = T_charge + H_sens / (cp mdot_exh)                       (cp 1100)
//   pair, per step: G = cp mdot, Keff = K G / (K + G);
//     T_w' = (C T_w + h (Keff T_port + Ka T_amb)) / (C + h (Keff + Ka));
//     T_turbine = (G T_port + K T_w') / (G + K)
// Defaults [AS]: f_wall 0.20, C 9200 J/K, K 120 W/K, Ka 30 W/K, cold start.
//
// Sabotages (each made the named checks fail, then reverted):
//   forward-Euler wall step                   -> ET1 literal + closure (5.6e-10), ET2 discrete, ET3, ET4
//   eta_c = 1 (lambda ignored)                -> ET5 chemical/wall/sensible literals (9.2e-2 off)
//   port temperature over the AIR flow only   -> ET5 port literal (5.9e-2 off)
//   in-cylinder wall heat dropped             -> ET5 wall/sensible literals (0.196 off)
//   1200 K cap put back on the turbine inlet  -> ET3 (1200.000 K)
//   no zero-flow guard on the port temperature-> ET6 (non-finite port temperature)
//   no sub-ambient guard on the port gas      -> ET4
//   wall temperature not folded in the hash   -> ET7

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
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

constexpr real kH = 1.0 / 960.0;
constexpr real kPamb = 101325.0;
constexpr real kTamb = 288.15;
constexpr real kPi = 3.14159265358979323846;
// The [AS] defaults typed in (TurboPairDesc), not read from the code under test.
constexpr real kC = 9200.0, kK = 120.0, kKa = 30.0, kFwall = 0.20, kCp = 1100.0;

real omega_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

real rel(real a, real b) { return std::abs(a - b) / std::max({std::abs(a), std::abs(b), 1e-300}); }

} // namespace

// ---------------------------------------------------------------------------
// Pair level: the lumped exhaust wall.
// ---------------------------------------------------------------------------
TEST_CASE("ET1: exhaust wall - one step against literal values, energy closed to round-off", "[turbo][drivetrain][exhaust_thermal]") {
    // (a) literal step: wall 900 K, port 1200 K, 0.5 kg/s exhaust.
    {
        TurboPairDesc d = tb::hyper_like_pair();
        d.exhaust_wall_initial_k = 900.0;
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        REQUIRE(p.wall_k == 900.0);
        p.step(kH, kPamb, kTamb, 0.4, 0.5, 1200.0, 1.0);
        const real g = kCp * 0.5;
        const real keff = kK * g / (kK + g);
        const real tw = (kC * 900.0 + kH * (keff * 1200.0 + kKa * kTamb)) / (kC + kH * (keff + kKa));
        CHECK(p.wall_k == Catch::Approx(tw).epsilon(1e-14));
        CHECK(p.exhaust_k == Catch::Approx((g * 1200.0 + kK * tw) / (g + kK)).epsilon(1e-14));
        CHECK(p.gas_to_wall_w == Catch::Approx(keff * (1200.0 - tw)).epsilon(1e-13));
        CHECK(p.wall_to_ambient_w == Catch::Approx(kKa * (tw - kTamb)).epsilon(1e-13));
        CHECK(p.port_k == 1200.0);
    }
    // (b) closure over the guard script (spool, lift, re-apply): wall C dT = h (gas->wall - wall->ambient),
    //     gas G (T_port - T_turbine) = gas->wall, every step.
    {
        TurboPair p(tb::hyper_like_pair());
        p.reset(kPamb, kTamb);
        real worst_wall = 0.0, worst_gas = 0.0, max_wall = 0.0;
        for (int i = 0; i < tb::kScriptSteps; ++i) {
            const tb::Phase ph = tb::script_phase(i);
            const real tw0 = p.wall_k;
            p.step(kH, kPamb, kTamb, ph.air, ph.exhaust, ph.port_k, ph.plate);
            const real lhs = kC * (p.wall_k - tw0);
            const real rhs = kH * (p.gas_to_wall_w - p.wall_to_ambient_w);
            // relative to the wall's stored energy C*T_w: C*(T_w' - T_w) cancels to ~C*T_w*eps
            worst_wall = std::max(worst_wall, std::abs(lhs - rhs) / (kC * p.wall_k));
            const real gas = kCp * ph.exhaust * (ph.port_k - p.exhaust_k);
            worst_gas = std::max(worst_gas, std::abs(gas - p.gas_to_wall_w) / std::max(1.0, std::abs(gas)));
            max_wall = std::max(max_wall, p.wall_k);
        }
        std::printf("[ET1] guard script: wall closure worst %.3e, gas closure worst %.3e, wall max %.2f K\n", worst_wall, worst_gas, max_wall);
        CHECK(worst_wall <= 1e-14);
        CHECK(worst_gas <= 1e-12);
        CHECK(max_wall > kTamb + 5.0); // the script did heat the wall
    }
}

TEST_CASE("ET2: exhaust wall decay matches its analytic time constant", "[turbo][drivetrain][exhaust_thermal]") {
    // (a) no flow: T_w - T_amb decays with tau = C/Ka = 306.67 s. Backward Euler's own recurrence
    //     exactly, and the continuous exponential within its O(h/tau) discretisation error.
    {
        TurboPairDesc d = tb::hyper_like_pair();
        d.exhaust_wall_initial_k = 1000.0;
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        const int n = 60 * 960;
        for (int i = 0; i < n; ++i) p.step(kH, kPamb, kTamb, 0.0, 0.0, 0.0, 0.0);
        const real tau = kC / kKa;
        const real discrete = kTamb + (1000.0 - kTamb) * std::pow(kC / (kC + kH * kKa), n);
        const real analytic = kTamb + (1000.0 - kTamb) * std::exp(-60.0 / tau);
        std::printf("[ET2] no flow 60 s: wall %.9f K, discrete %.9f, analytic %.9f (tau %.2f s)\n", p.wall_k, discrete, analytic, tau);
        CHECK(p.wall_k == Catch::Approx(discrete).epsilon(1e-11));
        CHECK(rel(p.wall_k - kTamb, analytic - kTamb) < 1e-5);
        CHECK(p.exhaust_k == p.wall_k); // no gas: the turbine-inlet reading is the wall
    }
    // (b) constant flow and port temperature from cold: the wall approaches
    //     T_ss = (Keff T_port + Ka T_amb)/(Keff + Ka) with tau = C/(Keff + Ka).
    {
        TurboPair p(tb::hyper_like_pair());
        p.reset(kPamb, kTamb);
        const real mdot = 0.3, tp = 1150.0;
        const real g = kCp * mdot, keff = kK * g / (kK + g);
        const real tss = (keff * tp + kKa * kTamb) / (keff + kKa);
        const real tau = kC / (keff + kKa);
        const int n = int(std::round(tau * 960.0));
        for (int i = 0; i < n; ++i) p.step(kH, kPamb, kTamb, 0.0, mdot, tp, 0.0);
        const real frac = (p.wall_k - tss) / (kTamb - tss);
        std::printf("[ET2] flow 0.3 kg/s, port 1150 K: tau %.2f s, T_ss %.2f K, after tau wall %.3f K -> remaining fraction %.6f (e^-1 %.6f)\n",
                    tau, tss, p.wall_k, frac, std::exp(-1.0));
        CHECK(std::abs(frac - std::exp(-n * kH / tau)) < 1e-4);
    }
}

TEST_CASE("ET3: no temperature cap - a wall and gas above 1200 K reach the turbine", "[turbo][drivetrain][exhaust_thermal]") {
    TurboPairDesc d = tb::hyper_like_pair();
    d.exhaust_wall_initial_k = 1500.0;
    TurboPair p(d);
    p.reset(kPamb, kTamb);
    p.step(kH, kPamb, kTamb, 0.2, 0.3, 1600.0, 1.0);
    const real g = kCp * 0.3, keff = kK * g / (kK + g);
    const real tw = (kC * 1500.0 + kH * (keff * 1600.0 + kKa * kTamb)) / (kC + kH * (keff + kKa));
    std::printf("[ET3] turbine inlet %.3f K (wall %.3f K)\n", p.exhaust_k, p.wall_k);
    CHECK(p.exhaust_k > 1500.0);
    CHECK(p.exhaust_k == Catch::Approx((g * 1600.0 + kK * tw) / (g + kK)).epsilon(1e-14));
    // the turbine power uses it (no clamp anywhere downstream)
    const real pr = p.exhaust_pa / kPamb;
    REQUIRE(pr > 1.0);
}

TEST_CASE("ET4: zero, negative flow and a port below ambient are numerically guarded", "[turbo][drivetrain][exhaust_thermal]") {
    TurboPairDesc d = tb::hyper_like_pair();
    d.exhaust_wall_initial_k = 800.0;
    for (const real mdot : {0.0, -0.1}) {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.step(kH, kPamb, kTamb, 0.0, mdot, 5000.0, 0.0);
        CHECK(std::isfinite(p.exhaust_k));
        CHECK(p.gas_to_wall_w == 0.0);
        CHECK(p.exhaust_k == p.wall_k);
        CHECK(p.wall_k == Catch::Approx((kC * 800.0 + kH * kKa * kTamb) / (kC + kH * kKa)).epsilon(1e-14));
    }
    TurboPair q(d);
    q.reset(kPamb, kTamb);
    q.step(kH, kPamb, kTamb, 0.1, 0.1, 0.0, 0.0); // port 0 K (no gas reading yet) counts as ambient
    CHECK(q.port_k == kTamb);
    CHECK(q.gas_to_wall_w < 0.0); // the 800 K wall heats the ambient-temperature gas
}

// ---------------------------------------------------------------------------
// Engine level: the exhaust energy balance.
// ---------------------------------------------------------------------------
namespace {
SimulatedEngineDesc turbo_engine(const std::optional<AntiLagDesc>& al) {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    d.turbo_pair->anti_lag = al;
    return d;
}
} // namespace

TEST_CASE("ET5: engine exhaust split closes to round-off and matches literal terms (WOT, anti-lag, DFCO)", "[turbo][drivetrain][exhaust_thermal][sim_engine]") {
    SimulatedEngine e(turbo_engine(tb::hyper_like_anti_lag()));
    const real omega = omega_of(5000.0);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = 1.0;
    real worst_close = 0.0, worst_fuel = 0.0, worst_ind = 0.0, worst_chem = 0.0, worst_wall = 0.0, worst_port = 0.0, worst_als = 0.0;
    int rich = 0, als = 0, dfco = 0, clamped = 0;
    real port_wot = 0.0, port_als = 0.0, turbine_wot = 0.0;
    for (int i = 0; i < 2 * 960 + 1440 + 960; ++i) {
        const bool wot = i < 2 * 960;
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{wot ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        const PostSolveResult p = e.post_solve(omega, kH);
        const auto& x = e.exhaust_split();
        // closure
        worst_close = std::max(worst_close, std::abs(x.fuel_w - (x.indicated_w + x.wall_w + x.sensible_w + x.chemical_w - x.clamp_w)) /
                                                std::max(1.0, x.fuel_w));
        // the ledger's own fuel energy and indicated work
        worst_fuel = std::max(worst_fuel, std::abs(x.fuel_w * kH - p.fuel_energy_in_delta) / std::max(1e-9, p.fuel_energy_in_delta));
        worst_ind = std::max(worst_ind, std::abs(x.indicated_w * kH - p.indicated_work_delta) / std::max(1e-9, std::abs(p.indicated_work_delta)));
        // literal terms: eta_c = min(1, lambda), wall 0.20 of the released energy, port = T_charge + H/(1100 mdot)
        const real eta_c = std::min(1.0, r.lambda);
        worst_chem = std::max(worst_chem, std::abs(x.chemical_w - (1.0 - eta_c) * x.fuel_w) / std::max(1.0, x.fuel_w));
        worst_wall = std::max(worst_wall, std::abs(x.wall_w - kFwall * eta_c * x.fuel_w) / std::max(1.0, x.fuel_w));
        const real port = r.exhaust_mass_flow_kg_s > 0.0 ? r.t_charge_k + x.sensible_w / (kCp * r.exhaust_mass_flow_kg_s) : r.t_charge_k;
        worst_port = std::max(worst_port, rel(port, r.exhaust_port_k));
        CHECK(x.port_k == r.exhaust_port_k);
        if (x.clamp_w != 0.0) ++clamped;
        if (x.chemical_w > 1000.0) ++rich;
        if (r.dfco_active) {
            ++dfco;
            CHECK(x.fuel_w == 0.0);
            CHECK(r.exhaust_port_k == r.t_charge_k);
        }
        if (r.anti_lag_active) {
            ++als;
            // the work the retard removes stays in the gas: sensible + indicated equals what the unretarded
            // spark would leave, evaluated independently at the same state
            const auto ev0 = e.evaluate_torque(omega, e.p_im(), true, 1.0, false, nullptr);
            const real expect_sensible = (1.0 - kFwall) * eta_c * x.fuel_w - omega * ev0.tau_comb + r.anti_lag_extra_exhaust_w;
            worst_als = std::max(worst_als, std::abs(x.sensible_w - expect_sensible) / std::max(1.0, x.sensible_w));
            CHECK(r.anti_lag_extra_exhaust_w == Catch::Approx(omega * (ev0.tau_comb - x.indicated_w / omega)).epsilon(1e-9));
            if (i == 2 * 960 + 480) port_als = r.exhaust_port_k;
        }
        if (i == 2 * 960 - 1) { port_wot = r.exhaust_port_k; turbine_wot = r.turbine_inlet_k; }
    }
    std::printf("[ET5] closure %.2e, fuel %.2e, indicated %.2e, chemical %.2e, wall %.2e, port %.2e, anti-lag sensible %.2e; "
                "rich %d, anti-lag %d, dfco %d substeps; port WOT %.1f K (turbine %.1f K), port under anti-lag %.1f K\n",
                worst_close, worst_fuel, worst_ind, worst_chem, worst_wall, worst_port, worst_als, rich, als, dfco, port_wot, turbine_wot, port_als);
    CHECK(worst_close <= 1e-12);
    CHECK(worst_fuel <= 1e-12);
    CHECK(worst_ind <= 1e-12);
    CHECK(worst_chem <= 1e-12);
    CHECK(worst_wall <= 1e-12);
    CHECK(worst_port <= 1e-12);
    CHECK(worst_als <= 1e-9);
    CHECK(clamped == 0);
    CHECK(rich > 1000);   // the fixture's lambda < 1 at WOT: chemical term exercised
    CHECK(als == 1440);
    CHECK(dfco > 900);
    CHECK(port_als > 0.0);
}

TEST_CASE("ET6: port temperature bounded - engine stopped (no flow) and an extreme anti-lag", "[turbo][drivetrain][exhaust_thermal][sim_engine]") {
    // (a) crank at rest, ignition on: no exhaust flow, no division - the port reads the charge temperature.
    {
        SimulatedEngine e(turbo_engine(std::nullopt));
        Sensors s;
        s.omega = 0.0;
        s.ignition = 1.0;
        for (int i = 0; i < 96; ++i) {
            const PreSolveResult r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(0.0, kH);
            REQUIRE(r.exhaust_mass_flow_kg_s == 0.0);
            CHECK(r.exhaust_port_k == r.t_charge_k);
            CHECK(std::isfinite(r.turbine_inlet_k));
        }
    }
    // (b) anti-lag with a 60 deg retard (eta_spark at its 0.55 floor) and a full-open plate floor: the
    //     port never exceeds the oxygen-limited adiabatic bound T_charge + min(1,lambda) LHV/(cp (lambda AFR + 1)).
    {
        AntiLagDesc al = tb::hyper_like_anti_lag();
        al.retard_deg = 60.0;
        al.plate_floor = 1.0;
        al.max_active_s = 3.0;
        SimulatedEngine e(turbo_engine(al));
        const real omega = omega_of(6000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.anti_lag_channel = 1.0;
        real worst_margin = 1e300, port_max = 0.0;
        for (int i = 0; i < 960 + 2880; ++i) {
            const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < 960 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
            const real bound = r.t_charge_k + std::min(1.0, r.lambda) * 44.0e6 / (kCp * (r.lambda * 14.5 + 1.0));
            worst_margin = std::min(worst_margin, bound - r.exhaust_port_k);
            port_max = std::max(port_max, r.exhaust_port_k);
        }
        std::printf("[ET6] extreme anti-lag: port max %.1f K, min margin to the adiabatic bound %.1f K\n", port_max, worst_margin);
        CHECK(worst_margin > 0.0);
        CHECK(std::isfinite(port_max));
    }
}

TEST_CASE("ET7: wall temperature and the exhaust_thermal parameters are hashed", "[turbo][drivetrain][exhaust_thermal][sim_engine]") {
    const auto hash_of = [](const TurboPairDesc& pair) {
        auto d = fx::electronic_engine();
        d.turbo_pair = pair;
        SimulatedEngine e(d);
        return e.fold_state_hash(0);
    };
    const TurboPairDesc base = tb::hyper_like_pair();
    const std::uint64_t h0 = hash_of(base);
    TurboPairDesc warm = base;
    warm.exhaust_wall_initial_k = 900.0; // not folded itself: only the wall state differs
    CHECK(hash_of(warm) != h0);
    TurboPairDesc b = base;
    b.in_cylinder_wall_heat_fraction = 0.25; CHECK(hash_of(b) != h0);
    b = base; b.exhaust_wall_heat_capacity_j_k = 5000.0; CHECK(hash_of(b) != h0);
    b = base; b.exhaust_wall_gas_conductance_w_k = 60.0; CHECK(hash_of(b) != h0);
    b = base; b.exhaust_wall_ambient_conductance_w_k = 10.0; CHECK(hash_of(b) != h0);
}

// ---------------------------------------------------------------------------
// Loader.
// ---------------------------------------------------------------------------
namespace {
using json = nlohmann::json;
using Catch::Matchers::ContainsSubstring;
TurboPairDesc load_cfg(json cfg, const std::string& tag) {
    cfg["turbo"] = std::string(PS_SOURCE_DIR) + "/data/turbos/hyper_turbo_approx.json";
    const auto dir = std::filesystem::temp_directory_path() / "ps_turbo_exhaust_thermal_io_test";
    std::filesystem::create_directories(dir);
    const auto path = dir / (tag + ".json");
    std::ofstream(path) << cfg.dump(2);
    return ps::io::load_turbo_configuration_json(path.string());
}
json base_cfg() {
    return json::parse(R"({"format": "physics_sim.turbo_configuration/1", "name": "et", "turbo": "X", "count": 2,
                           "target_boost_bar": 1.8, "intercooler_effectiveness": 0.95})");
}
json valid_block() {
    return json::parse(R"({"in_cylinder_wall_heat_fraction": 0.18, "wall_heat_capacity_j_k": 12000, "wall_gas_conductance_w_k": 90,
                           "wall_ambient_conductance_w_k": 25, "initial_wall_temperature_k": 700, "source": "[AS] io test"})");
}
} // namespace

TEST_CASE("ET8: exhaust_thermal loader - defaults, values and rejections", "[turbo][io][exhaust_thermal]") {
    const TurboPairDesc none = load_cfg(base_cfg(), "none");
    CHECK(none.in_cylinder_wall_heat_fraction == kFwall);
    CHECK(none.exhaust_wall_heat_capacity_j_k == kC);
    CHECK(none.exhaust_wall_gas_conductance_w_k == kK);
    CHECK(none.exhaust_wall_ambient_conductance_w_k == kKa);
    CHECK(none.exhaust_wall_initial_k == 0.0);
    json c = base_cfg();
    c["exhaust_thermal"] = valid_block();
    const TurboPairDesc d = load_cfg(c, "valid");
    CHECK(d.in_cylinder_wall_heat_fraction == 0.18);
    CHECK(d.exhaust_wall_heat_capacity_j_k == 12000.0); // installation totals: not scaled by count 2
    CHECK(d.exhaust_wall_gas_conductance_w_k == 90.0);
    CHECK(d.exhaust_wall_ambient_conductance_w_k == 25.0);
    CHECK(d.exhaust_wall_initial_k == 700.0);
    const auto reject = [](const char* key, const json& value, const char* message) {
        json cfg = base_cfg();
        json b = valid_block();
        if (value.is_null()) b.erase(key);
        else b[key] = value;
        cfg["exhaust_thermal"] = b;
        CHECK_THROWS_WITH(load_cfg(cfg, std::string("bad_") + key), ContainsSubstring(message));
    };
    reject("in_cylinder_wall_heat_fraction", 1.0, "'in_cylinder_wall_heat_fraction' must be in [0, 1)");
    reject("in_cylinder_wall_heat_fraction", -0.1, "'in_cylinder_wall_heat_fraction' must be in [0, 1)");
    reject("wall_heat_capacity_j_k", 0.0, "wall_heat_capacity_j_k");
    reject("wall_gas_conductance_w_k", -1.0, "'wall_gas_conductance_w_k' must be >= 0");
    reject("wall_ambient_conductance_w_k", -1.0, "'wall_ambient_conductance_w_k' must be >= 0");
    reject("initial_wall_temperature_k", 0.0, "initial_wall_temperature_k");
    reject("source", nullptr, "'source' is required");
    reject("max_temperature_k", 1300.0, "unknown key 'max_temperature_k'");
    json n = base_cfg();
    n["exhaust_thermal"] = 3;
    CHECK_THROWS_WITH(load_cfg(n, "not_object"), ContainsSubstring("'exhaust_thermal' must be an object"));
}

// ---------------------------------------------------------------------------
// Measured report on the shipped hyper V8 (not run by default): steady WOT at
// 3000/6000/7500 rpm (15 s from a cold wall, like bench_hyper_calibration, and
// from a pre-warmed wall), lift-off rotor speed with and without the D5 fixture
// anti-lag.
// ---------------------------------------------------------------------------
namespace {
SimulatedEngineDesc load_hyper() {
    static ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    return std::get<SimulatedEngineDesc>(ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
}
} // namespace

TEST_CASE("ET diag: hyper V8 exhaust temperatures, calibration points and lift-off", "[.][diag][exhaust_thermal][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    const auto maps = SimulatedEngine::build_maps(base);
    for (const real wall0 : {0.0, 1000.0}) {
        for (const real rpm : {3000.0, 6000.0, 7500.0}) {
            SimulatedEngineDesc d = base;
            d.turbo_pair->exhaust_wall_initial_k = wall0;
            SimulatedEngine e(d, maps);
            const real omega = omega_of(rpm);
            Sensors s;
            s.omega = omega;
            s.ignition = 1.0;
            PreSolveResult r;
            real sum = 0, fuel = 0;
            for (int i = 0; i < 14400; ++i) {
                r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
                (void)e.post_solve(omega, kH);
                if (i >= 13440) { sum += (r.tau0 - maps->c_k) / 960.0; fuel += r.fuel_flow_g_s / 960.0; }
            }
            const auto& x = e.exhaust_split();
            std::printf("[ET hyper WOT] wall0 %4.0f rpm %4.0f: net %.1f N*m %.1f kW boost %.3f bar exhaust %.0f Pa fuel %.2f g/s lambda %.3f | "
                        "port %.1f K (%.1f C) turbine inlet %.1f K (%.1f C) wall %.1f K | split of fuel: indicated %.3f wall %.3f sensible %.3f chemical %.3f\n",
                        wall0, rpm, sum, sum * omega / 1000.0, r.boost_bar, r.p_exhaust_pa, fuel, r.lambda, r.exhaust_port_k, r.exhaust_port_k - 273.15,
                        r.turbine_inlet_k, r.turbine_inlet_k - 273.15, r.exhaust_wall_k, x.indicated_w / x.fuel_w, x.wall_w / x.fuel_w,
                        x.sensible_w / x.fuel_w, x.chemical_w / x.fuel_w);
        }
    }
    // lift-off at 6000 rpm after 2 s WOT, ALS off / on (D5 fixture block), cold and warm wall
    for (const real wall0 : {0.0, 1000.0}) {
        for (const real channel : {0.0, 1.0}) {
            SimulatedEngineDesc d = base;
            d.turbo_pair->exhaust_wall_initial_k = wall0;
            d.turbo_pair->anti_lag = tb::hyper_like_anti_lag();
            SimulatedEngine e(d, maps);
            const real omega = omega_of(6000.0);
            Sensors s;
            s.omega = omega;
            s.ignition = 1.0;
            s.anti_lag_channel = channel;
            const int n_wot = 2 * 960;
            const int marks[] = {0, 240, 480, 960, 1920};
            std::vector<PreSolveResult> rs;
            for (int i = 0; i < n_wot + 3 * 960; ++i) {
                rs.push_back(e.pre_solve(s, ThrottleCommand{i < n_wot ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH));
                (void)e.post_solve(omega, kH);
            }
            std::printf("[ET hyper lift 6000] wall0 %4.0f ALS %s", wall0, channel > 0.5 ? "on " : "off");
            for (int m : marks) {
                const PreSolveResult& r = rs[std::size_t(n_wot - 1 + m)];
                std::printf(" | +%.2f s rotor %6.0f boost %+.3f port %4.0f turb %4.0f wall %4.0f tau0 %+7.1f fuel %.1f g/s", m / 960.0, r.turbo_rpm,
                            r.boost_bar, r.exhaust_port_k, r.turbine_inlet_k, r.exhaust_wall_k, r.tau0, r.fuel_flow_g_s);
            }
            std::printf("\n");
        }
    }
    // Lift-off without anti-lag from a warm wall: what drains the rotor. Sensitivity to the [AS] rotor
    // inertia and to the gas-side wall conductance (test-only desc mutations, a diagnosis, not data).
    for (const real j_scale : {1.0, 5.0, 10.0}) {
        for (const real k_scale : {1.0, 4.0}) {
            SimulatedEngineDesc d = base;
            d.turbo_pair->exhaust_wall_initial_k = 1000.0;
            d.turbo_pair->rotor_inertia_kgm2 *= j_scale;
            d.turbo_pair->exhaust_wall_gas_conductance_w_k *= k_scale;
            SimulatedEngine e(d, maps);
            const real omega = omega_of(6000.0);
            Sensors s;
            s.omega = omega;
            s.ignition = 1.0;
            const int n_wot = 4 * 960;
            std::printf("[ET lift sens] J x%2.0f K x%.0f", j_scale, k_scale);
            for (int i = 0; i < n_wot + 2 * 960; ++i) {
                const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < n_wot ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
                (void)e.post_solve(omega, kH);
                const int m = i - (n_wot - 1);
                if (m == 0 || m == 10 || m == 24 || m == 48 || m == 240 || m == 480 || m == 960 || m == 1920) {
                    const TurboPair& t = *e.turbo();
                    std::printf(" | +%4.0f ms rotor %6.0f turb %4.0f K P_t %5.1f P_c %5.1f kW", m * 1000.0 / 960.0, r.turbo_rpm, t.exhaust_k,
                                t.turbine_w / 1000.0, t.compressor_w / 1000.0);
                }
            }
            std::printf("\n");
        }
    }
}
