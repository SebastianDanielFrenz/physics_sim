// tests/unit/test_turbo_wastegate_split.cpp -- review fix of the exhaust thermal / ALS work:
// the wastegate is a bypass orifice IN PARALLEL with the turbine, and the compressor power is
// bounded by the head the current rotor speed makes.
//
// Defect (coordinator review of 9fd087c + 99f1dbc): turbo_pair.h sent exhaust*(1 - wastegate)
// through the turbine and the rest around it. Below 10 % plate the wastegate is commanded fully
// open, so the turbine got ZERO flow and the shipped hyper rotor sat at exactly 0 rpm while the
// engine ran (every lift, every gear change). A real wastegate is a port beside the turbine:
// both see the same manifold pressure and the exhaust splits by effective area. TurboPair::step
// now uses the incompressible orifice form it already used for the manifold pressure:
//   A = A_t + A_wg*opening, turbine flow = m*A_t/A, p_exh = p_amb + m^2/(2 rho A^2).
// The wastegate port area is the optional hardware key wastegate_area_m2 (per turbo); absent =
// 2.4 x turbine_area_m2 [AS]. The shipped hyper hardware sets 1.18e-3 m2 (turbo.md: a 50 mm poppet
// valve, Cd 0.6 [AS], against its 500 mm2 throat). Sizing check ("WG diag area"): with 1.5 x the
// throat the open gate could not hold 1.8 bar above ~6500 rpm (boost creep, rotor on its 1.5x cap
// at 7000-8000 rpm); at 2.5 x the steady WOT gate opening peaks at 0.67 (8000 rpm) and tau0 / boost /
// rotor match the pre-fix steady values (the split leaves an unsaturated equilibrium unchanged:
// p_exh - p_amb = m_turbine^2/(2 rho A_t^2) in both forms).
//
// Second change: compressor_w used the pressure ratio of charge_pa, which lags the head by 25 ms.
// After a lift the slowing wheel kept paying for a ratio it no longer makes. The power now uses
// min(pr, 1 + head(current speed, flow)/p_amb).
//
// Tests
//   WG1  One TurboPair step with the gate fully open and half open: manifold pressure and turbine
//        power equal a literal re-derivation of the parallel split (1e-12); the open gate still
//        drives the turbine (share A_t/(A_t + A_wg) = 1/3.4 with the default ratio). Sabotage: the
//        old 100 % bypass (turbine flow m*(1 - wastegate)) - turbine power 0 at an open gate.
//   WG2  Compressor power with the charge pressure above the wheel's head equals the literal
//        bounded form, and with the head above the charge the unbounded form. Sabotage: the bound
//        removed.
//   WG3  Shipped hyper V8, held 6000 rpm, WOT 2 s, then throttle 0 (DFCO, no ALS) for 2 s, then a
//        held idle (950 rpm) for 8 s: the rotor never reaches 0 rpm and keeps measured-with-margin
//        floors. Sabotage: the old 100 % bypass fails it on the 2e-5 rotor (min 0 rpm); on the derived
//        2.3e-4 rotor it does NOT (90.1k / 84.5k / 77.9k at +0.25 / +1 / +2 s, idle 45.9k - the rotor
//        coasts through the window), so WG1 is the discriminating guard for the split. The data
//        sabotage (2e-5 restored) fails every floor.
//        Measured (shipped derived 2.3e-4 rotor, 2026-10-05): rotor 106137 at the lift, 92158 / 90148 /
//        86357 / 79500 rpm at +0.25 / +0.5 / +1 / +2 s; min 45177; idle (last 1 s of 8 s) 46616 rpm,
//        still coasting (60 s idle: 3.1k rpm). Floors 80k / 75k / 65k / idle 30k. The old 2e-5 [AS] rotor
//        restored in the data file fails every floor. With the 2e-5 rotor (commit 258f5c2, floors then
//        18k / 11k / 6k / 1): 102248 at the lift, 23102 / 19809 / 14646 / 8084 rpm; min 4 rpm; idle mean
//        4 rpm. Without the compressor bound (2e-5): 0 rpm at +0.25 s. KNOWN LIMIT: the steady idle rotor speed is only ~10 rpm for any
//        inertia (WG diag, 60 s idle). At idle exhaust flow (~0.01 kg/s through the open gate) the
//        mean-value orifice turbine sees a few Pa of pressure drop, i.e. mW of power; the equilibrium
//        is set against the compressor's power at the bilinear head table's lowest speed cell (head
//        linear in speed below 1/40 of reference). Real turbos idle far faster on pulse energy, which
//        a mean-value turbine does not carry - see the progress file for options.
//   WG4  Loader: wastegate_area_m2 x count, absent = 2.4 x the turbine area, rejections; the area
//        is hashed.
//   WG diag  The same lift and a car_hyper 3rd-gear WOT spool-up from ~3000 rpm, for a list of
//        rotor inertias (the review's old/new comparison).

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"
#include "vehicle_test_helpers.h"

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
constexpr real kAmbPa = 101325.0, kAmbK = 288.15;
// WG3 measured-with-margin floors (shipped data; values in the progress file).
constexpr real WG3_FLOOR_025 = 80000.0, WG3_FLOOR_1 = 75000.0, WG3_FLOOR_2 = 65000.0, WG3_FLOOR_IDLE = 30000.0;

real omega_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

SimulatedEngineDesc load_hyper() {
    static ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    return std::get<SimulatedEngineDesc>(
        ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
}

// Literal parallel-orifice split (the same incompressible form as the manifold pressure).
struct Split { real p_exh, turbine_w, share; };
Split literal_split(const TurboPairDesc& d, real m, real w, real t_gas) {
    const real at = d.turbine_area_m2, aw = d.wastegate_area();
    const real a = at + aw * w;
    const real rho = kAmbPa / (287.05 * t_gas);
    Split s;
    s.share = at / a;
    s.p_exh = kAmbPa + m * m / (2.0 * rho * a * a);
    s.turbine_w = m * s.share * 1100.0 * t_gas * d.turbine_efficiency * (1.0 - std::pow(kAmbPa / s.p_exh, 0.24812030075188));
    return s;
}

// Held-crank lift: WOT n_wot, then command 0 at the same speed for n_lift, then a held idle.
struct LiftTrace { std::vector<real> rotor, boost, wg, exh_pa, exh_k; int n_wot = 0, n_lift = 0; real min_rotor_after_lift = 1e30; };
LiftTrace run_lift(const SimulatedEngineDesc& d, real rpm, real idle_rpm, int n_wot, int n_lift, int n_idle) {
    SimulatedEngine e(d);
    Sensors s;
    s.ignition = 1.0;
    LiftTrace out;
    out.n_wot = n_wot; out.n_lift = n_lift;
    for (int i = 0; i < n_wot + n_lift + n_idle; ++i) {
        s.omega = omega_of(i < n_wot + n_lift ? rpm : idle_rpm);
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < n_wot ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(s.omega, kH);
        out.rotor.push_back(r.turbo_rpm); out.boost.push_back(r.boost_bar); out.wg.push_back(r.wastegate_opening);
        out.exh_pa.push_back(r.p_exhaust_pa); out.exh_k.push_back(r.turbine_inlet_k);
        if (i >= n_wot) out.min_rotor_after_lift = std::min(out.min_rotor_after_lift, r.turbo_rpm);
    }
    return out;
}

real at(const LiftTrace& t, real seconds_after_lift) {
    const int k = t.n_wot - 1 + int(std::lround(seconds_after_lift * 960.0));
    return t.rotor[std::size_t(std::min<int>(k, int(t.rotor.size()) - 1))];
}

real idle_mean(const LiftTrace& t) {
    real sum = 0.0; int n = 0;
    for (std::size_t k = t.rotor.size() - 960; k < t.rotor.size(); ++k) { sum += t.rotor[k]; ++n; }
    return sum / n;
}

void print_lift(const char* tag, const LiftTrace& t) {
    std::printf("[%s] rotor +0 %.0f | +0.25 %.0f | +0.5 %.0f | +1 %.0f | +2 %.0f rpm | min after lift %.0f rpm | idle (last 1 s of 8 s) %.0f rpm,"
                " wg %.3f, exh %.0f Pa %.0f K\n",
                tag, at(t, 0), at(t, 0.25), at(t, 0.5), at(t, 1.0), at(t, 2.0), t.min_rotor_after_lift, idle_mean(t), t.wg.back(),
                t.exh_pa.back(), t.exh_k.back());
}

json base_config() {
    return json::parse(R"({
      "format": "physics_sim.turbo_configuration/1", "name": "wg_split_test", "turbo": "HARDWARE",
      "count": 2, "target_boost_bar": 1.8, "intercooler_effectiveness": 0.95
    })");
}
TurboPairDesc load(const json& hw_patch, const std::string& tag) {
    const auto dir = std::filesystem::temp_directory_path() / "ps_turbo_wg_split_io_test";
    std::filesystem::create_directories(dir);
    std::ifstream in(std::string(PS_SOURCE_DIR) + "/data/turbos/hyper_turbo_approx.json");
    json hw = json::parse(in);
    hw.merge_patch(hw_patch);
    const auto hw_path = dir / (tag + "_hw.json");
    std::ofstream(hw_path) << hw.dump(2);
    json cfg = base_config();
    cfg["turbo"] = hw_path.string();
    const auto path = dir / (tag + ".json");
    std::ofstream(path) << cfg.dump(2);
    return ps::io::load_turbo_configuration_json(path.string());
}

} // namespace

TEST_CASE("WG1: wastegate in parallel with the turbine - literal split, an open gate still drives the turbine",
          "[turbo][drivetrain][wastegate]") {
    const TurboPairDesc d = tb::hyper_like_pair();
    REQUIRE(d.wastegate_area() == 2.4 * d.turbine_area_m2);
    const real m = 0.35, t_port = 1150.0;
    for (const real w0 : {1.0, 0.5, 0.0}) {
        TurboPair p(d);
        p.reset(kAmbPa, kAmbK);
        p.wall_k = 1050.0;
        p.wastegate = w0;
        p.energy_j = 0.5 * d.rotor_inertia_kgm2 * omega_of(80000.0) * omega_of(80000.0);
        // plate 0: commanded open; w0 == 1 stays exactly 1, w0 0.5 moves toward 1 by the actuator lag.
        p.step(kH, kAmbPa, kAmbK, 0.1, m, t_port, w0 == 0.0 ? 1.0 : 0.0);
        if (w0 == 0.0) { p.wastegate = 0.0; p.step(kH, kAmbPa, kAmbK, 0.1, m, t_port, 1.0); }
        const Split lit = literal_split(d, m, p.wastegate, p.exhaust_k);
        INFO("w0 " << w0 << " wastegate " << p.wastegate << " exhaust_k " << p.exhaust_k);
        CHECK(p.exhaust_pa == Catch::Approx(lit.p_exh).epsilon(1e-12));
        CHECK(p.turbine_w == Catch::Approx(lit.turbine_w).epsilon(1e-12));
        std::printf("[WG1] gate %.4f: turbine share %.4f, manifold %.0f Pa, turbine %.0f W (exhaust %.4f kg/s at %.0f K)\n", p.wastegate,
                    lit.share, p.exhaust_pa, p.turbine_w, m, p.exhaust_k);
        if (w0 == 1.0) {
            CHECK(p.wastegate == 1.0);
            CHECK(lit.share == Catch::Approx(1.0 / 3.4).epsilon(1e-12));
            CHECK(p.turbine_w > 1000.0); // the old 100 % bypass gave exactly 0 here
        }
    }
}

TEST_CASE("WG2: compressor power is bounded by the head of the current rotor speed", "[turbo][drivetrain][wastegate]") {
    const TurboPairDesc d = tb::hyper_like_pair();
    const real wr = omega_of(d.reference_rpm);
    const auto literal_cw = [&](const TurboPair& p, real speed, real air, bool bounded) {
        const real flow = air / d.max_mass_flow_kg_s;
        const real head = d.maps->head_pa.value(speed, flow);
        const real pr = std::max(1.0, p.charge_pa / kAmbPa); // no intercooler drop in the fixture
        const real pr_c = bounded ? std::min(pr, 1.0 + std::max(0.0, head) / kAmbPa) : pr;
        return std::pair<real, real>{air * 1005.0 * kAmbK * (std::pow(pr_c, 2.0 / 7.0) - 1.0) / d.compressor_efficiency,
                                     1.0 + std::max(0.0, head) / kAmbPa};
    };
    SECTION("slow wheel under a high charge pressure: bounded") {
        TurboPair p(d);
        p.reset(kAmbPa, kAmbK);
        p.charge_pa = 2.8e5;
        const real speed = 0.25;
        p.energy_j = 0.5 * d.rotor_inertia_kgm2 * (speed * wr) * (speed * wr);
        const real sp = std::sqrt(2.0 * p.energy_j / (d.rotor_inertia_kgm2 * wr * wr));
        p.step(kH, kAmbPa, kAmbK, 0.3, 0.0, kAmbK, 0.0);
        const auto [cw, pr_wheel] = literal_cw(p, sp, 0.3, true);
        const auto [cw_unbounded, unused] = literal_cw(p, sp, 0.3, false);
        (void)unused;
        std::printf("[WG2] slow wheel: charge PR %.3f, wheel PR %.3f, compressor %.0f W (unbounded %.0f W)\n", p.compressor_pr, pr_wheel,
                    p.compressor_w, cw_unbounded);
        REQUIRE(p.compressor_pr > pr_wheel + 0.5); // the bound is active
        CHECK(p.compressor_w == Catch::Approx(cw).epsilon(1e-12));
        CHECK(p.compressor_w < 0.5 * cw_unbounded);
    }
    SECTION("fast wheel under a low charge pressure: unbounded") {
        TurboPair p(d);
        p.reset(kAmbPa, kAmbK);
        p.charge_pa = 1.6e5;
        const real speed = 1.0;
        p.energy_j = 0.5 * d.rotor_inertia_kgm2 * (speed * wr) * (speed * wr);
        const real sp = std::sqrt(2.0 * p.energy_j / (d.rotor_inertia_kgm2 * wr * wr));
        p.step(kH, kAmbPa, kAmbK, 0.3, 0.0, kAmbK, 1.0);
        const auto [cw, pr_wheel] = literal_cw(p, sp, 0.3, false);
        REQUIRE(p.compressor_pr < pr_wheel);
        CHECK(p.compressor_w == Catch::Approx(cw).epsilon(1e-12));
    }
}

TEST_CASE("WG3: shipped hyper V8 lift without anti-lag - the rotor never reaches 0 rpm (measured)",
          "[turbo][sim_engine][hyper][wastegate]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    REQUIRE_FALSE(base.turbo_pair->anti_lag.has_value());
    const LiftTrace t = run_lift(base, 6000.0, base.idle_rpm, 2 * 960, 2 * 960, 8 * 960);
    print_lift("WG3 shipped", t);
    CHECK(t.min_rotor_after_lift > 0.0);
    // Measured-with-margin floors (values in the commit message / progress file).
    CHECK(at(t, 0.25) >= WG3_FLOOR_025);
    CHECK(at(t, 1.0) >= WG3_FLOOR_1);
    CHECK(at(t, 2.0) >= WG3_FLOOR_2);
    CHECK(idle_mean(t) >= WG3_FLOOR_IDLE);
}

TEST_CASE("WG4: wastegate_area_m2 loader - per turbo x count, default ratio, rejections, hashed", "[turbo][io][wastegate]") {
    const TurboPairDesc shipped = load(json::object(), "shipped");
    REQUIRE(shipped.wastegate_area_m2.has_value());
    CHECK(*shipped.wastegate_area_m2 == 1.18e-3 * 2);
    const TurboPairDesc def = load(json::parse(R"({"wastegate_area_m2": null})"), "default"); // merge patch: key removed
    CHECK_FALSE(def.wastegate_area_m2.has_value());
    CHECK(def.wastegate_area() == 2.4 * def.turbine_area_m2);
    const TurboPairDesc set = load(json::parse(R"({"wastegate_area_m2": 0.0006})"), "set");
    REQUIRE(set.wastegate_area_m2.has_value());
    CHECK(*set.wastegate_area_m2 == 0.0006 * 2);
    CHECK_THROWS_WITH(load(json::parse(R"({"wastegate_area_m2": 0})"), "zero"), ContainsSubstring("wastegate_area_m2"));
    CHECK_THROWS_WITH(load(json::parse(R"({"wastegate_area_m2": -1e-4})"), "neg"), ContainsSubstring("wastegate_area_m2"));
    CHECK_THROWS_WITH(load(json::parse(R"({"wastegate_area_m2": "big"})"), "str"), ContainsSubstring("wastegate_area_m2"));

    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    auto d2 = d;
    d2.turbo_pair->wastegate_area_m2 = d.turbo_pair->wastegate_area() * 1.01;
    SimulatedEngine a(d), b(d2);
    CHECK(a.fold_state_hash(0) != b.fold_state_hash(0));
}

TEST_CASE("WG diag: hyper lift and 3rd-gear spool-up by rotor inertia", "[.][diag][wastegate][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions opt;
    opt.pool = &pool;
    opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    const auto car = ps::io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    const real shipped_pair = base.turbo_pair->rotor_inertia_kgm2;
    std::printf("[WG diag] shipped pair inertia %.3e kg*m^2 (per turbo %.3e)\n", shipped_pair, shipped_pair / 2.0);
    for (const real per_turbo : {2e-5, 1e-4, 2.3e-4}) {
        SimulatedEngineDesc d = base;
        d.turbo_pair->rotor_inertia_kgm2 = 2.0 * per_turbo;
        const LiftTrace t = run_lift(d, 6000.0, d.idle_rpm, 2 * 960, 2 * 960, 60 * 960);
        std::printf("[WG diag idle J %.1e] rotor at idle +2 %.0f | +5 %.0f | +10 %.0f | +20 %.0f | +40 %.0f | +60 s %.0f rpm\n", per_turbo,
                    at(t, 4.0), at(t, 7.0), at(t, 12.0), at(t, 22.0), at(t, 42.0), at(t, 62.0));
        char tag[64];
        std::snprintf(tag, sizeof tag, "WG diag lift J %.1e", per_turbo);
        print_lift(tag, t);

        // car_hyper in 3rd: rolling at the speed for 3000 rpm, light throttle 1 s, then WOT.
        auto desc = car;
        for (auto& c : desc.powertrain.components)
            if (auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) e->turbo_pair->rotor_inertia_kgm2 = 2.0 * per_turbo;
        auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0.0, 4, ps::Vec3::unit_y(), 240.0, 960.0, 3000.0);
        ps_test::settle(*rig.world, .5);
        const real v3000 = omega_of(3000.0) / (1.65 * 3.35) * 0.364;
        auto motion = rig.world->get_motion(rig.chassis);
        motion.linear = {v3000, 0, 0};
        rig.world->backend().set_motion(rig.chassis, motion);
        ps::vehicle::VehicleResetOptions reset;
        reset.gear = 3;
        reset.clutch_locked = true;
        rig.world->reset_vehicle(rig.vehicle, reset);
        rig.world->set_control("gear_request", 3);
        rig.world->set_control("throttle", 0.1);
        ps_test::settle(*rig.world, 1.0);
        auto ps0 = rig.world->powertrain_state(rig.vehicle);
        const real rpm0 = ps0.engine_rpm, rotor0 = ps0.engines.empty() ? 0.0 : ps0.engines[0].turbo_rpm,
                   boost0 = ps0.engines.empty() ? 0.0 : ps0.engines[0].boost_bar;
        rig.world->set_control("throttle", 1.0);
        real t90 = -1, t98 = -1, rpm90 = 0, rpm98 = 0, peak = 0;
        for (int tick = 1; tick <= 240 * 6; ++tick) {
            rig.world->step();
            const auto st = rig.world->powertrain_state(rig.vehicle);
            if (st.engines.empty()) break;
            const real b = st.engines[0].boost_bar;
            peak = std::max(peak, b);
            if (t90 < 0 && b >= 0.9 * 1.8) { t90 = tick / 240.0; rpm90 = st.engine_rpm; }
            if (t98 < 0 && b >= 0.98 * 1.8) { t98 = tick / 240.0; rpm98 = st.engine_rpm; }
            if (st.gear != 3 || st.engine_rpm > 8100) break;
        }
        std::printf("[WG diag spool J %.1e] 3rd gear WOT from %.0f rpm (rotor %.0f rpm, boost %+.3f bar): 90 %% of 1.8 bar at +%.3f s (%.0f rpm),"
                    " 98 %% at +%.3f s (%.0f rpm), peak %.3f bar\n",
                    per_turbo, rpm0, rotor0, boost0, t90, rpm90, t98, rpm98, peak);
    }
}

TEST_CASE("WG diag area: steady WOT wastegate authority by area ratio", "[.][diag][wastegate][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    for (const real ratio : {1.5, 2.5, 3.5, 4.5, 6.0}) {
        for (const real rpm : {3000.0, 6000.0, 7000.0, 8000.0, 8250.0}) {
            SimulatedEngineDesc d = base;
            d.turbo_pair->wastegate_area_m2 = ratio * d.turbo_pair->turbine_area_m2;
            SimulatedEngine e(d);
            Sensors s;
            s.omega = omega_of(rpm);
            s.ignition = 1.0;
            PreSolveResult r;
            real boost = 0.0, wg = 0.0, tau = 0.0;
            for (int i = 0; i < 6 * 960; ++i) {
                r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
                (void)e.post_solve(s.omega, kH);
                if (i >= 5 * 960) { boost += r.boost_bar / 960.0; wg += r.wastegate_opening / 960.0; tau += r.tau0 / 960.0; }
            }
            std::printf("[WG diag area] ratio %.1f %5.0f rpm: boost %.4f bar, wastegate %.4f, rotor %.0f rpm, tau0 %.1f N*m, exhaust %.0f Pa\n", ratio, rpm,
                        boost, wg, r.turbo_rpm, tau, r.p_exhaust_pa);
        }
    }
}

TEST_CASE("WG diag als trace: rally ALS fixture at 6000 rpm, 20 ms samples", "[.][diag][wastegate][hyper]") {
  const SimulatedEngineDesc base = load_hyper();
  for (const real per_turbo : {2e-5, 2.3e-4}) {
    SimulatedEngineDesc d = base;
    d.turbo_pair->rotor_inertia_kgm2 = 2.0 * per_turbo;
    std::printf("[WG diag als] rotor inertia per turbo %.1e\n", per_turbo);
    d.turbo_pair->anti_lag = tb::hyper_like_als_cut();
    SimulatedEngine e(d);
    Sensors s;
    s.omega = omega_of(6000.0);
    s.ignition = 1.0;
    s.anti_lag_channel = 1.0;
    for (int i = 0; i < 3 * 960; ++i) {
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < 1920 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(s.omega, kH);
        if (i >= 1900 && (i - 1900) % 19 == 0)
            std::printf("[WG diag als] t %+.3f s rotor %6.0f boost %+.3f wg %.3f plate %.3f exh %.0f Pa T %.0f K\n", (i - 1919) / 960.0, r.turbo_rpm,
                        r.boost_bar, r.wastegate_opening, r.throttle_position, r.p_exhaust_pa, r.turbine_inlet_k);
    }
  }
}
