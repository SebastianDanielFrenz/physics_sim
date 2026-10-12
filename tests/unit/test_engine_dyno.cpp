// tests/unit/test_engine_dyno.cpp - engine garage P3 (A9): run_dyno_sweep and EngineRig's desc constructor.
//
// D1 torque map: every point is the WOT curve (==), the default grid is idle..limiter step 250 limiter included, and the
//    definition matches the runtime EngineTorqueMap (tau0 - c_k, fuel_flow_g_s) at part throttle too.
// D2 NA simulated: every point is the steady accessor exactly at the steady tables' nodes, within 0.25 % of a stepped
//    runtime engine held at the point (this pins steady_point against the real pre_solve/post_solve loop).
// D3 hyper (Sketch maps; Full is [long]): within 0.5 % of an independent copy of bench_hyper_calibration's held-crank loop.
// D4 early exit vs a forced max_settle_s run: within 0.5 % at every converged point.
// D5 cancel: set before the call -> throws before any point; set from another thread mid-sweep -> throws, no result.
// D6 1 vs 10 workers vs no pool: bit-identical in every field.
// D7 EngineRig from a desc equals EngineRig from its path (ledger and state over a free rev).
// D8 a maps_pending desc is rejected with the engine's message; pending desc + generated maps equals the loaded desc.
//
// Sabotages (recorded in out/garage_p3_progress.md, each reverted): D1 drop the (1 - throttle) term / use w - m; D2
// fired = false in steady_point; D3 drop "- c_k"; D4 always converged after two windows; D5 no cancel check;
// D6 one engine reused across points; D7 desc ctor alters the desc; D8 with_engine_maps keeps maps_pending.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "drivetrain/engine_simulated.h"
#include "drivetrain/engine_torque_map.h"
#include "drivetrain/powertrain.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_rig.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {

const std::string kEngines = std::string(PS_SOURCE_DIR) + "/data/engines/";

std::string cache_dir() {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "ps_garage_p3_maps";
    std::filesystem::create_directories(p);
    return p.string();
}

jobs::JobSystem& map_pool() {
    static jobs::JobSystem pool(10);
    return pool;
}

ComponentParams load(const std::string& file, MapQuality q) {
    io::EngineMapOptions o;
    o.quality = q;
    o.pool = &map_pool();
    o.cache_dir = cache_dir();
    return io::load_engine_json(kEngines + file, o);
}

const ComponentParams& torque_map_engine() {
    static const ComponentParams e = io::load_engine_json(kEngines + "i4_2l_na.json");
    return e;
}
const ComponentParams& fuel_map_engine() {
    static const ComponentParams e = io::load_engine_json(kEngines + "toyota_a25a_fks.json");
    return e;
}
const ComponentParams& na_engine() {
    static const ComponentParams e = io::load_engine_json(kEngines + "toyota_a25a_fks_sim.json");
    return e;
}
const ComponentParams& hyper_sketch() {
    static const ComponentParams e = load("hyper_v8_one1_audio.json", MapQuality::Sketch);
    return e;
}

bool same_bits(real a, real b) { return std::memcmp(&a, &b, sizeof(real)) == 0; }
bool same_point(const DynoPoint& a, const DynoPoint& b) {
    return same_bits(a.rpm, b.rpm) && same_bits(a.torque_nm, b.torque_nm) && same_bits(a.power_kw, b.power_kw) &&
           same_bits(a.boost_bar, b.boost_bar) && same_bits(a.turbo_rpm, b.turbo_rpm) && same_bits(a.fuel_g_s, b.fuel_g_s) &&
           same_bits(a.bsfc_g_kwh, b.bsfc_g_kwh) && same_bits(a.lambda, b.lambda) && same_bits(a.spark_deg, b.spark_deg) &&
           same_bits(a.p_intake_kpa, b.p_intake_kpa) && same_bits(a.exhaust_k, b.exhaust_k) &&
           same_bits(a.charge_boost_bar, b.charge_boost_bar) && a.converged == b.converged && same_bits(a.settle_s, b.settle_s);
}
bool same_result(const DynoResult& a, const DynoResult& b) {
    if (a.points.size() != b.points.size()) return false;
    for (std::size_t i = 0; i < a.points.size(); ++i) {
        if (!same_point(a.points[i], b.points[i])) return false;
    }
    return true;
}
real rel(real a, real b) { return std::abs(a - b) / std::max(std::abs(b), real(1e-12)); }

// An independent copy of tests/bench/bench_hyper_calibration.cpp's held-crank loop (NOT a call into the dyno): 15 s at
// 960 Hz, the net torque (tau0 - c_k) averaged over the last second.
real bench_reference_torque(const SimulatedEngineDesc& d, const std::shared_ptr<const SimulatedEngineMaps>& maps, real rpm,
                            int steps = 14400, int mean_steps = 960) {
    SimulatedEngine e(d, maps);
    real sum = 0;
    const real omega = rpm * 6.283185307179586 / 60, h = 1. / 960;
    for (int i = 0; i < steps; ++i) {
        Sensors s;
        s.omega = omega;
        s.ignition = 1;
        const PreSolveResult r = e.pre_solve(s, {1, 0, 1}, {}, h);
        (void)e.post_solve(omega, h);
        if (i >= steps - mean_steps) sum += (r.tau0 - maps->c_k) / mean_steps;
    }
    return sum;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno D1: a torque-map engine's sweep is its WOT curve, exactly", "[drivetrain][dyno]") {
    const TorqueMapEngineDesc& d = std::get<TorqueMapEngineDesc>(torque_map_engine());
    DynoSweepSpec spec;
    const DynoResult r = run_dyno_sweep(torque_map_engine(), nullptr, spec);

    SECTION("default grid: idle .. limiter in 250 rpm steps, limiter included, no duplicate") {
        REQUIRE(d.idle_rpm == 800.0);
        REQUIRE(d.limiter.rpm == 6800.0);
        REQUIRE(r.points.size() == 25);
        for (std::size_t i = 0; i < r.points.size(); ++i) CHECK(r.points[i].rpm == 800.0 + 250.0 * double(i));
        CHECK(r.points.back().rpm == d.limiter.rpm);
    }
    SECTION("default grid when the limiter is not on a step: it is appended") {
        TorqueMapEngineDesc e = d;
        e.limiter.rpm = 6900.0;
        const DynoResult q = run_dyno_sweep(ComponentParams{e}, nullptr, spec);
        REQUIRE(q.points.size() == 26);
        CHECK(q.points[24].rpm == 6800.0);
        CHECK(q.points[25].rpm == 6900.0);
    }
    SECTION("torque == the WOT curve (==), power = torque * omega, instant, converged") {
        for (const DynoPoint& p : r.points) {
            CHECK(p.torque_nm == d.wot_torque_nm_vs_rpm.evaluate(p.rpm));
            CHECK(p.power_kw == p.torque_nm * (p.rpm * 6.283185307179586 / 60.0) / 1000.0);
            CHECK(p.converged);
            CHECK(p.settle_s == 0.0);
            CHECK(p.boost_bar == 0.0);
            CHECK(p.turbo_rpm == 0.0);
            CHECK(std::isnan(p.lambda));
            CHECK(std::isnan(p.spark_deg));
            CHECK(std::isnan(p.p_intake_kpa));
            CHECK(std::isnan(p.exhaust_k));
        }
    }
    SECTION("explicit rpm list keeps its order") {
        spec.rpm = {5000.0, 2000.0, 3000.5};
        const DynoResult q = run_dyno_sweep(torque_map_engine(), nullptr, spec);
        REQUIRE(q.points.size() == 3);
        CHECK(q.points[0].rpm == 5000.0);
        CHECK(q.points[1].torque_nm == d.wot_torque_nm_vs_rpm.evaluate(2000.0));
        CHECK(q.points[2].torque_nm == d.wot_torque_nm_vs_rpm.evaluate(3000.5));
    }
    SECTION("the definition equals the runtime EngineTorqueMap (tau0 - c_k) at WOT and at part throttle, crude fuel flow too") {
        for (real throttle : {1.0, 0.6}) {
            for (real rpm : {2000.0, 3500.0, 5200.0}) {
                EngineTorqueMap::Desc ed = to_engine_desc(d);
                EngineTorqueMap engine(ed);
                Sensors s;
                s.omega = rpm * 6.283185307179586 / 60.0;
                s.ignition = 1.0;
                const PreSolveResult pr = engine.pre_solve(s, ThrottleCommand{throttle, 0.0, 1.0}, TcuRequest{}, 1.0 / 960.0);
                const real c_k = -ed.motoring_torque_nm_vs_rpm.evaluate(0.0);
                DynoSweepSpec sp;
                sp.rpm = {rpm};
                sp.throttle = throttle;
                const DynoPoint p = run_dyno_sweep(torque_map_engine(), nullptr, sp).points[0];
                INFO("throttle " << throttle << " rpm " << rpm);
                CHECK(rel(p.torque_nm, pr.tau0 - c_k) < 1e-12);
                CHECK(rel(p.fuel_g_s, pr.fuel_flow_g_s) < 1e-12);
            }
        }
    }
    SECTION("a fuel-map engine reports the map's flow at the WOT row (the runtime's own lookup)") {
        const TorqueMapEngineDesc& f = std::get<TorqueMapEngineDesc>(fuel_map_engine());
        REQUIRE(!f.fuel_map.values.empty());
        DynoSweepSpec sp;
        sp.rpm = {2500.0, 4000.0};
        const DynoResult q = run_dyno_sweep(fuel_map_engine(), nullptr, sp);
        for (const DynoPoint& p : q.points) {
            CHECK(p.fuel_g_s == f.fuel_map.value(p.rpm, 1.0));
            CHECK(p.bsfc_g_kwh == p.fuel_g_s * 3600.0 / p.power_kw);
        }
    }
}

TEST_CASE("Dyno: bad specs and non-engine components are rejected", "[drivetrain][dyno]") {
    DynoSweepSpec s;
    s.rpm = {0.0};
    CHECK_THROWS_AS(run_dyno_sweep(torque_map_engine(), nullptr, s), std::invalid_argument);
    s = {};
    s.throttle = 1.5;
    CHECK_THROWS_AS(run_dyno_sweep(torque_map_engine(), nullptr, s), std::invalid_argument);
    s = {};
    s.settle_window_s = 0.0;
    CHECK_THROWS_AS(run_dyno_sweep(torque_map_engine(), nullptr, s), std::invalid_argument);
    s = {};
    CHECK_THROWS_AS(run_dyno_sweep(ComponentParams{ClutchDesc{}}, nullptr, s), std::invalid_argument);
    // explicit maps on a torque-map engine
    CHECK_THROWS_WITH(run_dyno_sweep(torque_map_engine(), std::make_shared<EngineMaps>(), s),
                      Catch::Matchers::ContainsSubstring("not a simulated engine"));
}

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno D2: an NA simulated engine's sweep is the steady accessor, and matches the stepped runtime", "[drivetrain][dyno]") {
    const SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(na_engine());
    REQUIRE_FALSE(d.turbo_pair.has_value());
    const std::shared_ptr<const SimulatedEngineMaps> full = SimulatedEngine::build_maps(d);

    SECTION("at the steady tables' nodes the point equals the table value exactly") {
        DynoSweepSpec spec;
        spec.rpm = {1000.0, 2500.0, 4000.0, 5500.0, 6500.0};
        const DynoResult r = run_dyno_sweep(na_engine(), nullptr, spec);
        REQUIRE(r.points.size() == 5);
        for (const DynoPoint& p : r.points) {
            INFO("rpm " << p.rpm);
            CHECK(p.torque_nm == full->steady_wot_nm.evaluate(p.rpm));
            CHECK(p.torque_nm == full->steady_torque_nm.value(p.rpm, 1.0));
            CHECK(p.converged);
            CHECK(p.settle_s == 0.0);
            CHECK(p.boost_bar == 0.0);
            CHECK(std::isnan(p.exhaust_k));
            CHECK(p.p_intake_kpa > 0.0);
            CHECK(p.p_intake_kpa < 101.325);
            CHECK(p.lambda > 0.0);
            CHECK(p.fuel_g_s > 0.0);
            CHECK(p.bsfc_g_kwh > 150.0);
            CHECK(p.bsfc_g_kwh < 600.0);
        }
    }
    SECTION("between nodes it is the exact solve, within the table's interpolation error") {
        DynoSweepSpec spec;
        spec.rpm = {1111.0, 3333.0, 5071.0};
        const DynoResult r = run_dyno_sweep(na_engine(), nullptr, spec);
        for (const DynoPoint& p : r.points) {
            INFO("rpm " << p.rpm);
            CHECK(rel(p.torque_nm, full->steady_wot_nm.evaluate(p.rpm)) < 0.03); // measured 1.4 % at 1111 rpm (table is linear between nodes)
        }
    }
    SECTION("the default grid ends at the limiter") {
        const DynoResult r = run_dyno_sweep(na_engine(), nullptr, DynoSweepSpec{});
        REQUIRE(r.points.size() > 2);
        CHECK(r.points.front().rpm == d.idle_rpm);
        CHECK(r.points.back().rpm == d.limiter.rpm);
    }
    SECTION("within 0.25 % of the real engine stepped with the crank held (what the car would deliver)") {
        for (real rpm : {2000.0, 4000.0, 6000.0}) {
            DynoSweepSpec spec;
            spec.rpm = {rpm};
            const real dyno = run_dyno_sweep(na_engine(), nullptr, spec).points[0].torque_nm;
            const real stepped = bench_reference_torque(d, full, rpm, 4 * 960, 480);
            INFO("rpm " << rpm << " dyno " << dyno << " stepped " << stepped);
            CHECK(rel(dyno, stepped) < 0.0025);
        }
    }
    SECTION("ambient: thinner air gives less torque, and the ISA default is the ISA ambient") {
        DynoSweepSpec isa, high;
        isa.rpm = high.rpm = {4000.0};
        AmbientSample alt;
        alt.pressure_pa = 79500.0; // ~2000 m ISA
        alt.temperature_k = 275.15;
        high.ambient = alt;
        DynoSweepSpec isa2 = isa;
        isa2.ambient = AmbientSample{101325.0, 288.15, 0.0};
        const real t_isa = run_dyno_sweep(na_engine(), nullptr, isa).points[0].torque_nm;
        const real t_high = run_dyno_sweep(na_engine(), nullptr, high).points[0].torque_nm;
        CHECK(t_high < 0.9 * t_isa);
        CHECK(t_high > 0.6 * t_isa);
        CHECK(run_dyno_sweep(na_engine(), nullptr, isa2).points[0].torque_nm == t_isa);
    }
    SECTION("part throttle (electronic torque request) lies between motoring and WOT") {
        DynoSweepSpec half;
        half.rpm = {4000.0};
        half.throttle = 0.5;
        const real wot = full->steady_wot_nm.evaluate(4000.0), mot = full->steady_motoring_nm.evaluate(4000.0);
        const real t = run_dyno_sweep(na_engine(), nullptr, half).points[0].torque_nm;
        CHECK(rel(t, mot + 0.5 * (wot - mot)) < 0.01);
    }
}

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno D3: the hyper's sweep matches the bench's held-crank loop (Sketch maps)", "[drivetrain][dyno][turbo]") {
    const SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(hyper_sketch());
    REQUIRE(d.turbo_pair.has_value());
    const auto maps = SimulatedEngine::build_maps(d);
    DynoSweepSpec spec;
    spec.rpm = {3000.0, 4500.0, 6000.0, 7500.0, 8000.0};
    jobs::JobSystem pool(5);
    const DynoResult r = run_dyno_sweep(hyper_sketch(), nullptr, spec, &pool);
    REQUIRE(r.points.size() == 5);
    for (const DynoPoint& p : r.points) {
        const real ref = bench_reference_torque(d, maps, p.rpm);
        INFO("rpm " << p.rpm << " dyno " << p.torque_nm << " ref " << ref << " settle_s " << p.settle_s << " converged " << p.converged);
        CHECK(rel(p.torque_nm, ref) < 0.005);
        CHECK(p.boost_bar > 0.1);
        CHECK(p.turbo_rpm > 10000.0);
        CHECK(p.power_kw == p.torque_nm * (p.rpm * 6.283185307179586 / 60.0) / 1000.0);
        CHECK(std::isfinite(p.exhaust_k));
        CHECK(p.exhaust_k > 600.0);
        CHECK(p.charge_boost_bar > p.boost_bar);
    }
}

TEST_CASE("Dyno D3 [long]: the hyper's sweep with Full maps matches the bench's held-crank loop", "[drivetrain][dyno][turbo][long]") {
    const ComponentParams full_engine = load("hyper_v8_one1_audio.json", MapQuality::Full);
    const SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(full_engine);
    const auto maps = SimulatedEngine::build_maps(d);
    DynoSweepSpec spec;
    spec.rpm = {3000.0, 4000.0, 4500.0, 5000.0, 6000.0, 7000.0, 7500.0, 8000.0}; // bench_hyper_calibration's points
    jobs::JobSystem pool(10);
    const DynoResult r = run_dyno_sweep(full_engine, nullptr, spec, &pool);
    for (const DynoPoint& p : r.points) {
        const real ref = bench_reference_torque(d, maps, p.rpm);
        INFO("rpm " << p.rpm << " dyno " << p.torque_nm << " ref " << ref << " settle_s " << p.settle_s);
        CHECK(rel(p.torque_nm, ref) < 0.005);
    }
}

TEST_CASE("Dyno D4: early exit agrees with a forced full-length run where it converged", "[drivetrain][dyno][turbo]") {
    DynoSweepSpec early;
    early.rpm = {2000.0, 3000.0, 4500.0, 6000.0, 7500.0};
    DynoSweepSpec forced = early;
    forced.settle_boost_bar_per_s = 0.0; // never converges: runs max_settle_s
    jobs::JobSystem pool(5);
    const DynoResult e = run_dyno_sweep(hyper_sketch(), nullptr, early, &pool);
    const DynoResult f = run_dyno_sweep(hyper_sketch(), nullptr, forced, &pool);
    REQUIRE(e.points.size() == f.points.size());
    int converged = 0;
    for (std::size_t i = 0; i < e.points.size(); ++i) {
        const DynoPoint& a = e.points[i];
        const DynoPoint& b = f.points[i];
        INFO("rpm " << a.rpm << " early " << a.torque_nm << " @ " << a.settle_s << " s (converged " << a.converged << ") forced "
                    << b.torque_nm << " @ " << b.settle_s << " s (converged " << b.converged << ")");
        CHECK_FALSE(b.converged);
        CHECK(b.settle_s == 15.0);
        CHECK(a.settle_s <= b.settle_s);
        if (a.converged) {
            ++converged;
            CHECK(a.settle_s >= 1.5); // three windows of 0.5 s at least
            CHECK(rel(a.torque_nm, b.torque_nm) < 0.005);
            CHECK(rel(a.boost_bar, b.boost_bar) < 0.01);
        }
    }
    CHECK(converged >= 2);
}

TEST_CASE("Dyno: max_settle_s caps a turbo point and marks it not converged", "[drivetrain][dyno][turbo]") {
    DynoSweepSpec s;
    s.rpm = {5000.0};
    s.max_settle_s = 0.25; // shorter than one window: a single (shortened) window
    const DynoPoint p = run_dyno_sweep(hyper_sketch(), nullptr, s).points[0];
    CHECK_FALSE(p.converged);
    CHECK(p.settle_s <= 0.5);
    s.max_settle_s = 0.9; // one whole window only: nothing to compare with
    CHECK_FALSE(run_dyno_sweep(hyper_sketch(), nullptr, s).points[0].converged);
}

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno D5: cancel", "[drivetrain][dyno][turbo][walltime]") {
    // [walltime]: the mid-sweep section bounds the cancelled sweep by a multiple of one point's measured wall time; under
    // the parallel phase-A ctest load (24 jobs on linux-release) that bound was missed once (2026-10-12, passes 10/10 alone),
    // so the case runs in tools/ci.ps1's serial phase B like the timing-asserting benches.
    DynoSweepSpec spec;
    spec.rpm.clear();
    for (int i = 0; i < 40; ++i) spec.rpm.push_back(3000.0 + 100.0 * i);
    spec.settle_boost_bar_per_s = 0.0; // every point runs the full 15 s: long enough to be cancelled mid-sweep

    SECTION("set before the call: throws before any point, for every engine kind") {
        std::atomic<bool> c{true};
        CHECK_THROWS_AS(run_dyno_sweep(hyper_sketch(), nullptr, spec, nullptr, &c), GenerationCancelled);
        CHECK_THROWS_AS(run_dyno_sweep(na_engine(), nullptr, spec, nullptr, &c), GenerationCancelled);
        CHECK_THROWS_AS(run_dyno_sweep(torque_map_engine(), nullptr, spec, nullptr, &c), GenerationCancelled);
    }
    SECTION("the result is returned normally when the flag stays clear") {
        std::atomic<bool> c{false};
        DynoSweepSpec s2;
        s2.rpm = {4000.0};
        CHECK_NOTHROW(run_dyno_sweep(hyper_sketch(), nullptr, s2, nullptr, &c));
    }
    SECTION("set from another thread mid-sweep: serial and pooled runs throw early, no result") {
        DynoSweepSpec one;
        one.rpm = {5000.0};
        one.settle_boost_bar_per_s = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        (void)run_dyno_sweep(hyper_sketch(), nullptr, one);
        const double one_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        for (int workers : {0, 3}) {
            std::atomic<bool> c{false};
            std::unique_ptr<jobs::JobSystem> pool;
            if (workers > 0) pool = std::make_unique<jobs::JobSystem>(workers);
            std::thread canceller([&] {
                std::this_thread::sleep_for(std::chrono::duration<double>(one_s * 1.5));
                c.store(true);
            });
            const auto t1 = std::chrono::steady_clock::now();
            bool threw = false;
            try {
                (void)run_dyno_sweep(hyper_sketch(), nullptr, spec, pool.get(), &c);
            } catch (const GenerationCancelled&) {
                threw = true;
            }
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
            canceller.join();
            INFO("workers " << workers << " one point " << one_s << " s, elapsed " << elapsed << " s");
            CHECK(threw);
            // 40 points would take 40 * one_s serially (13 * one_s on 3 workers); cancelled it stops within a few points.
            CHECK(elapsed < 12.0 * one_s);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno D6: 1 vs 10 workers vs no pool give identical results in every field", "[drivetrain][dyno][turbo]") {
    DynoSweepSpec spec;
    spec.rpm = {1500.0, 2200.0, 3000.0, 3800.0, 4500.0, 5200.0, 6000.0, 6800.0, 7500.0, 8000.0, 8250.0};
    spec.max_settle_s = 6.0; // some points stop on the cap, some converge: both paths exercised
    const DynoResult serial = run_dyno_sweep(hyper_sketch(), nullptr, spec);
    jobs::JobSystem p1(1), p10(10);
    const DynoResult r1 = run_dyno_sweep(hyper_sketch(), nullptr, spec, &p1);
    const DynoResult r10 = run_dyno_sweep(hyper_sketch(), nullptr, spec, &p10);
    CHECK(same_result(serial, r1));
    CHECK(same_result(serial, r10));
    CHECK(same_result(r1, r10));
    // the instant kinds too
    DynoSweepSpec na_spec;
    CHECK(same_result(run_dyno_sweep(na_engine(), nullptr, na_spec), run_dyno_sweep(na_engine(), nullptr, na_spec, &p10)));
    // and a repeat on a fresh pool
    jobs::JobSystem p7(7);
    CHECK(same_result(serial, run_dyno_sweep(hyper_sketch(), nullptr, spec, &p7)));
}

// ---------------------------------------------------------------------------------------------------------------
namespace {
struct RigTrace {
    std::vector<real> v;
    void add(const EngineRig& r) {
        const PowertrainEnergy& l = r.ledger();
        v.insert(v.end(), {r.sim_time_s(), r.engine_omega_rad_s(), real(static_cast<int>(r.engine_state())), l.engine_out,
                           l.starter_in, l.indicated_work, l.fuel_mass, l.fuel_energy_in, l.engine_heat, l.balance_error,
                           r.engine_sound_state().p_intake_pa, r.engine_sound_state().fuel_flow_g_s});
    }
};
RigTrace drive(EngineRig& rig, int ticks) {
    RigTrace t;
    rig.set_ignition(1.0);
    for (int i = 0; i < ticks; ++i) {
        rig.set_starter(i < 120 ? 1.0 : 0.0);                    // crank, then run
        rig.set_throttle(i < 160 ? 0.0 : (i < 300 ? 1.0 : 0.0)); // idle, WOT free rev, lift
        rig.step();
        t.add(rig);
    }
    return t;
}
bool same_trace(const RigTrace& a, const RigTrace& b) {
    if (a.v.size() != b.v.size()) return false;
    for (std::size_t i = 0; i < a.v.size(); ++i) {
        if (!same_bits(a.v[i], b.v[i])) return false;
    }
    return true;
}
} // namespace

TEST_CASE("Dyno D7: EngineRig from a descriptor equals EngineRig from its path", "[drivetrain][dyno]") {
    EngineRig::Config cfg;
    cfg.initial_engine_state = EngineState::Off;
    for (const char* file : {"toyota_a25a_fks_sim.json", "i4_2l_na.json"}) {
        INFO(file);
        EngineRig from_path(kEngines + file, io::EngineMapOptions{}, nullptr, cfg);
        const ComponentParams desc = io::load_engine_json(kEngines + file);
        EngineRig from_desc(desc, nullptr, cfg);
        const RigTrace a = drive(from_path, 400);
        const RigTrace b = drive(from_desc, 400);
        CHECK(same_trace(a, b));
        CHECK(a.v.size() == 400 * 12);
        // the engine actually ran (not a vacuous equality)
        CHECK(from_path.engine_state() == EngineState::Running);
        CHECK(from_desc.engine_state() == EngineState::Running);
        CHECK(from_path.ledger().fuel_mass > 0.0);
    }
    // default-Config overload
    EngineRig d1(io::load_engine_json(kEngines + "i4_2l_na.json"), nullptr);
    EngineRig d2(kEngines + "i4_2l_na.json");
    d1.set_ignition(1.0);
    d2.set_ignition(1.0);
    for (int i = 0; i < 50; ++i) {
        d1.step();
        d2.step();
    }
    CHECK(same_bits(d1.engine_omega_rad_s(), d2.engine_omega_rad_s()));
    // a cooling system without a path-derived thermal file needs an explicit one
    EngineRig::Config thermal;
    thermal.cooling_system_path = std::string(PS_SOURCE_DIR) + "/data/cooling_systems/sedan.json";
    CHECK_THROWS_WITH(EngineRig(torque_map_engine(), nullptr, thermal), Catch::Matchers::ContainsSubstring("engine_thermal_path"));
    // explicit maps on a torque-map engine
    CHECK_THROWS_WITH(EngineRig(torque_map_engine(), std::make_shared<EngineMaps>()),
                      Catch::Matchers::ContainsSubstring("not a simulated engine"));
}

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno D8: a maps_pending descriptor is rejected; with generated maps it runs like the loaded engine", "[drivetrain][dyno][turbo]") {
    io::EngineMapOptions dry;
    dry.generate = false;
    const ComponentParams pending = io::load_engine_json(kEngines + "hyper_v8_one1_audio.json", dry);
    const SimulatedEngineDesc& pd = std::get<SimulatedEngineDesc>(pending);
    REQUIRE(pd.maps_pending);
    DynoSweepSpec spec;
    spec.rpm = {5000.0, 7000.0};

    CHECK_THROWS_WITH(run_dyno_sweep(pending, nullptr, spec), Catch::Matchers::ContainsSubstring("maps are pending"));
    CHECK_THROWS_WITH(EngineRig(pending, nullptr), Catch::Matchers::ContainsSubstring("maps are pending"));
    // pending NA engine: same rejection on the instant path
    SimulatedEngineDesc na_pending = std::get<SimulatedEngineDesc>(na_engine());
    na_pending.maps_pending = true;
    CHECK_THROWS_WITH(run_dyno_sweep(ComponentParams{na_pending}, nullptr, spec), Catch::Matchers::ContainsSubstring("maps are pending"));

    // The garage flow: validate-only load, generate the maps, run with them. Equals the engine loaded complete.
    REQUIRE(pd.cycle.has_value());
    const EngineMapsPtr maps = generate_engine_maps(*pd.cycle, pd.fuels.front(), MapQuality::Sketch, &map_pool());
    const DynoResult with_maps = run_dyno_sweep(pending, maps, spec);
    const DynoResult loaded = run_dyno_sweep(hyper_sketch(), nullptr, spec);
    CHECK(same_result(with_maps, loaded));
    // and the desc's own maps given explicitly change nothing
    CHECK(same_result(with_maps, run_dyno_sweep(hyper_sketch(), maps, spec)));
}

// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("Dyno settle trace", "[.][diag]") {
    // boost / torque / turbo rpm of the held-crank engine after t seconds, for choosing the convergence criterion
    const SimulatedEngineDesc& d = std::get<SimulatedEngineDesc>(hyper_sketch());
    const auto maps = SimulatedEngine::build_maps(d);
    for (real rpm : {2000.0, 3000.0, 4500.0, 6000.0, 7500.0}) {
        SimulatedEngine e(d, maps);
        e.reset(EngineState::Running);
        const real omega = rpm * 6.283185307179586 / 60, h = 1. / 960;
        std::printf("rpm %.0f: window(0.5 s) means of boost bar / torque / turbo rpm\n", rpm);
        real sb = 0, st = 0, sr = 0;
        for (int i = 0; i < 14400; ++i) {
            Sensors s;
            s.omega = omega;
            s.ignition = 1;
            const PreSolveResult r = e.pre_solve(s, {1, 0, 1}, {}, h);
            (void)e.post_solve(omega, h);
            sb += r.boost_bar;
            st += r.tau0 - maps->c_k;
            sr += r.turbo_rpm;
            if ((i + 1) % 480 == 0) {
                std::printf("  t=%5.1f  boost %.4f  torque %8.2f  turbo %8.0f\n", (i + 1) / 960.0, sb / 480, st / 480, sr / 480);
                sb = st = sr = 0;
            }
        }
    }
}

TEST_CASE("Dyno timing report", "[.][diag]") {
    using clk = std::chrono::steady_clock;
    const auto secs = [](clk::time_point a) { return std::chrono::duration<double>(clk::now() - a).count(); };
    const auto report = [&](const char* name, const ComponentParams& e, jobs::JobSystem* pool) {
        const auto t0 = clk::now();
        const DynoResult r = run_dyno_sweep(e, nullptr, DynoSweepSpec{}, pool);
        const double dt = secs(t0);
        std::vector<real> settle;
        int conv = 0;
        for (const DynoPoint& p : r.points) {
            conv += p.converged ? 1 : 0;
            settle.push_back(p.settle_s);
        }
        std::sort(settle.begin(), settle.end());
        std::printf("%-34s workers %2d: %3zu points, %6.1f ms, converged %d/%zu, median settle %.2f s\n", name,
                    pool ? pool->worker_count() : 0, r.points.size(), dt * 1000.0, conv, r.points.size(), settle[settle.size() / 2]);
        for (const DynoPoint& p : r.points) {
            if (!p.converged) std::printf("    not converged: %.0f rpm, %.1f N*m, boost %.3f bar\n", p.rpm, p.torque_nm, p.boost_bar);
        }
    };
    jobs::JobSystem p10(10);
    report("torque map i4_2l_na", torque_map_engine(), nullptr);
    report("NA simulated a25a_sim", na_engine(), nullptr);
    report("hyper Sketch", hyper_sketch(), nullptr);
    report("hyper Sketch", hyper_sketch(), &p10);
    const ComponentParams full = load("hyper_v8_one1_audio.json", MapQuality::Full);
    report("hyper Full", full, nullptr);
    report("hyper Full", full, &p10);
    const auto t0 = clk::now();
    DynoSweepSpec s;
    s.settle_boost_bar_per_s = 0.0;
    (void)run_dyno_sweep(full, nullptr, s, &p10);
    std::printf("hyper Full forced %.0f s on 10 workers: %.1f ms\n", s.max_settle_s, secs(t0) * 1000.0);
}
