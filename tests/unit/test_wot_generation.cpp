// tests/unit/test_wot_generation.cpp - engine garage P7d: the pipe-mode map generator at wide-open throttle and on
// long-valve-overlap engines (the P7c Mazda pipe experiment, tests/support/fixtures/mazda_skyactiv_g20/
// engine_pipes_p7c.json, could not generate Full maps: VE 0.0009 at 1500 rpm / 70 kPa).
//
//   diag "P7d measure"  every grid node of a pipe engine as the generator runs it (trimmed throttle, ISA ambient):
//                       trim error, throttle Cd*A / max, saturation, extra trim cycles, VE, eta_i, residual at IVC,
//                       plenum burned fraction - and per grid rpm the wide-open-throttle state (throttle at its
//                       maximum Cd*A, plenum free). Written to out/garage_p7d_progress.md section 1.
//
// ---------------------------------------------------------------------------------------------------------------------
// P7d VALIDATION WINDOWS - declared and committed BEFORE the first run of the fixed generator (the commit is the proof).
// Fix (progress file D1-D3): ambient-fed pipe engines get one wide-open-throttle row per rpm as the top row of their
// p_im-axis tables, read through a per-rpm warp (map_pressure_index); non-breathing held nodes are filled from the nearest
// breathing node above. Hard windows (asserted):
//   W1  the Mazda pipe experiment (engine_pipes_p7c.json) generates Sketch, Preview and Full maps without throwing.
//   W5h the hyper (compressor_supply) map content hash and cache key are unchanged at every quality (pins recorded by
//       "P7d diag: before/after report" on the unchanged generator, kHyper* below once recorded).
//   W6  test_car_sedan_pipes (idle/limiter/launch checks) passes in its existing bounds (run as is).
//   W7h hyper Sketch generation on 10 workers < 1.5 s (P1 budget; reported with every other time).
//   W8  guards, each with a recorded sabotage: (a) the runtime lookup reproduces every kept node and every WOT node
//       (VE value at the node, air = VE x air_per_ve(p) at p_wot) to <= 1e-15 relative, at ISA; (b) 1 vs 10 workers
//       identical content hash (Mazda pipe Preview); (c) zero allocation inside generation jobs; (d) P1 progress 1..total
//       on the calling thread with total = point_count incl. the WOT points, cancel throws GenerationCancelled and
//       writes no cache file; (e) lumped engines (toyota_a25a_fks_gen, the lumped Mazda KV2 fixture) keep their content
//       hash and key at every quality (pins recorded on the unchanged generator); (f) the warp is the identity without a
//       WOT curve, continuous at the last author row, its dy/dp and dy/drpm match central differences (1e-6 rel);
//       (g) the loader rejects an authored p_im-axis table mixed with generated WOT rows; (h) the .psmaps round trip
//       keeps the WOT curve, a file without one is byte-identical to the old format.
//   W9  every tools/ci.ps1 $script:HashCheckScenarios scenario hashes as before at 1 and 10 workers (none loads a pipe
//       engine).
// Reported (not asserted; held out or before/after):
//   W2  Mazda WOT trapped air at the EPA rows M78..M133 through the runtime's own lookup (map_pressure_index at the row's
//       cell pressure, VE applied at the cell temperature): R_air = measured / model against P7c A2's band 0.94..1.06.
//   W3  Mazda WOT-state IVC temperature (the wide-open-throttle cycle at each row's rpm, generator conventions) against
//       P7c A3's band 359.8..423.0 K.
//   W4  KV2 (test_knock_calibration.cpp) on the pipe experiment's generated Full knock table at the rows, through the
//       runtime's lookup, against KV2's band [-5, +10] deg - report only, the KV2 fixture stays lumped (a switch is only
//       proposed if every row is inside the band and the rms is clearly lower than the lumped fixture's).
//   W5  demo_i4_pipes and hyper WOT dyno (steady/settled) torque, p_intake and fuel flow at 1000/2000/3000/4000/5000/6000
//       rpm (hyper 2000/4000/6000), before and after.
//   W7  generation time Sketch/Preview/Full on 10 workers (no cache) for demo_i4_pipes, the hyper and the Mazda pipe
//       experiment, before and after.
// Pins recorded on the unchanged generator (commit before the fix; "P7d diag: before/after report"), {key, content hash}
// for Sketch, Preview, Full - W5h (hyper) and W8e (lumped) assert them unchanged after the fix:
//   hyper_v8_one1_audio          1ad2c691e62cb8c1/91e6da1cfbac2a2f  5ac81ea2929831e3/c7990547a1831263  bacd729a3c627552/dbdeea7f720c19bd
//   toyota_a25a_fks_gen (lumped) a8a8c52be9ff3c39/33733fe864e42cfa  e89e1d3c966ab55b/fe26a4fdc04d367e  48a371344034f8ca/4d5bed459c60c684
//   Mazda KV2 fixture (lumped)   b4443bd5f36341a3/18777e7a1e565ce8  744ee3c546f7c881/7d4f1c9f430b93b3  d45437bcf0c20bf0/d543d9180958b80e
// Before the fix the Mazda pipe experiment throws at Sketch, Preview and Full.
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;

namespace {

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);
const std::filesystem::path kMazdaDir = kSrc / "tests" / "support" / "fixtures" / "mazda_skyactiv_g20";
const std::filesystem::path kEngines = kSrc / "data" / "engines";

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

SimulatedEngineDesc load_desc(const char* file, const std::filesystem::path& dir) {
    io::EngineMapOptions o;
    o.quality = MapQuality::Full;
    o.pool = &pool();
    o.generate = false;
    const ComponentParams p = io::load_engine_json_from_string(json{{"extends", file}}.dump(), dir, o);
    const auto* e = std::get_if<SimulatedEngineDesc>(&p);
    REQUIRE(e != nullptr);
    REQUIRE(e->cycle.has_value());
    return *e;
}

struct NodeOut {
    real rpm = 0.0, p_target = 0.0;
    real p_mean = 0.0, err = 0.0, cda_rel = 0.0;
    bool sat = false;
    int extra = 0;
    real ve = 0.0, eta = 0.0, resid = 0.0, x_plenum = 0.0, t_ivc = 0.0, air_change = 0.0;
};

real mbt_atdc(const CycleModelDesc& d) {
    return ec::Wiebe::theta0_for_ca50(d.wiebe_a, d.wiebe_m, d.burn_duration_deg, d.ca50_mbt_deg_atdc);
}

ec::OperatingPoint gen_op(const CycleModelDesc& d, real rpm, real p) {
    ec::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p;
    op.t_im_k = kIsaAirTemperatureK;
    op.p_amb_pa = kIsaAirPressurePa;
    if (d.intake->compressor_supply) op.p_supply_pa = std::max(kIsaAirPressurePa, p * 1.05);
    op.lambda = 1.0;
    const bool scheduled = !d.spark_advance_deg.x.empty();
    op.spark_deg_atdc = scheduled ? -d.spark_advance_deg.value(rpm, p) + d.ignition_delay_deg : mbt_atdc(d);
    op.intake_advance_deg = d.cam.intake_advance_deg.x.empty() ? 0.0 : d.cam.intake_advance_deg.value(rpm, p);
    op.exhaust_retard_deg = d.cam.exhaust_retard_deg.x.empty() ? 0.0 : d.cam.exhaust_retard_deg.value(rpm, p);
    return op;
}

NodeOut run_one(const CycleModelDesc& d, const FuelDesc& f, const ec::OperatingPoint& op, bool wot, bool full) {
    ec::CycleEngineSettings set;
    set.h_s = full ? ec::kStepFull : ec::kStepPreview;
    set.settling_cycles = full ? ec::kEngineSettlingCyclesFull : ec::kEngineSettlingCyclesPreview;
    set.wide_open_throttle = wot;
    ec::CycleEngine eng(d, f.lhv_j_kg, f.afr_stoich, op, set);
    const ec::CycleEngineResult r = eng.run();
    NodeOut o;
    o.rpm = op.rpm;
    o.p_target = op.p_im_pa;
    o.p_mean = r.p_plenum_mean_pa;
    o.err = r.plenum_error_rel;
    o.cda_rel = r.throttle_cd_area_m2 / r.throttle_cd_area_max_m2;
    o.sat = r.throttle_saturated;
    o.extra = r.extra_trim_cycles;
    const real v_disp = real(d.cylinders) * 3.14159265358979323846 / 4.0 * d.bore_m * d.bore_m * d.stroke_m;
    const real p_ref = wot ? r.p_plenum_mean_pa : op.p_im_pa;
    const real air_per_ve = p_ref / (kIsaAirGasConstantJKgK * kIsaAirTemperatureK) * v_disp * 0.5;
    o.ve = real(d.cylinders) * r.mean.m_air_j_cyc * 0.5 / air_per_ve;
    o.eta = r.mean.m_air_j_cyc > 0.0 ? r.mean.w_gross_j * f.afr_stoich / (f.lhv_j_kg * r.mean.m_air_j_cyc) : 0.0;
    o.resid = r.mean.residual_frac;
    o.x_plenum = r.plenum_burned_mean;
    const ec::CycleModel& m = eng.cylinder(0).model();
    const real v_ivc = m.geom.eval(r.mean.ivc.crank_deg).v;
    o.t_ivc = ec::cv_state(m.gas, r.mean.ivc.m, r.mean.ivc.u, r.mean.ivc.mb, v_ivc).t;
    o.air_change = r.air_change_rel;
    return o;
}

void print(const char* tag, const NodeOut& o) {
    std::printf("%s %6.0f rpm p_t %6.1f p_mean %6.2f err %+7.4f cda %5.3f%s extra %2d | VE %6.4f eta %6.4f resid %5.3f "
                "x_pl %5.3f T_ivc %5.1f dAir %.1e\n",
                tag, o.rpm, o.p_target / 1000.0, o.p_mean / 1000.0, o.err, o.cda_rel, o.sat ? " SAT" : "    ", o.extra, o.ve,
                o.eta, o.resid, o.x_plenum, o.t_ivc, o.air_change);
}

void measure(const char* name, const SimulatedEngineDesc& sd, bool full) {
    const CycleModelDesc& d = *sd.cycle;
    const FuelDesc& f = sd.fuels[0];
    std::vector<ec::OperatingPoint> ops;
    std::vector<int> wot;
    for (real rpm : d.grid_rpm) {
        for (real p : d.grid_p_im) {
            ops.push_back(gen_op(d, rpm, p));
            wot.push_back(0);
        }
        ec::OperatingPoint w = gen_op(d, rpm, kIsaAirPressurePa);
        ops.push_back(w);
        wot.push_back(1);
    }
    std::vector<NodeOut> out(ops.size());
    std::vector<std::string> fail(ops.size());
    pool().parallel_for(ops.size(), 1, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            try {
                out[i] = run_one(d, f, ops[i], wot[i] != 0, full);
            } catch (...) {
                fail[i] = "threw";
            }
        }
    });
    std::printf("=== P7d measure: %s (%s settings) ===\n", name, full ? "Full" : "Preview");
    for (std::size_t i = 0; i < ops.size(); ++i) {
        if (!fail[i].empty()) {
            std::printf("%s %6.0f rpm p_t %6.1f THREW\n", wot[i] ? "WOT " : "node", ops[i].rpm, ops[i].p_im_pa / 1000.0);
            continue;
        }
        print(wot[i] ? "WOT " : "node", out[i]);
        if (!wot[i] && !out[i].sat && out[i].err > 0.02) {
            ec::OperatingPoint held = ops[i];
            held.p_supply_pa = held.p_im_pa;
            try {
                print("held", run_one(d, f, held, false, full));
            } catch (...) {
                std::printf("held THREW\n");
            }
        }
    }
}

} // namespace

TEST_CASE("P7d diag: measure the pipe generator's grid nodes and the WOT state", "[.][diag][cycle][pipe]") {
    measure("Mazda P7c pipe experiment", load_desc("engine_pipes_p7c.json", kMazdaDir), true);
}

TEST_CASE("P7d diag: measure demo_i4_pipes and the hyper", "[.][diag][cycle][pipe]") {
    measure("demo_i4_pipes", load_desc("demo_i4_pipes.json", kEngines), true);
    measure("hyper_v8_one1_audio", load_desc("hyper_v8_one1_audio.json", kEngines), true);
}

namespace {

struct Gen {
    EngineMapsPtr maps;
    std::uint64_t key = 0;
    double seconds = 0.0;
    bool threw = false;
    std::string what;
};

Gen generate(const SimulatedEngineDesc& sd, MapQuality q) {
    Gen g;
    g.key = engine_map_cache_key(*sd.cycle, sd.fuels[0], q);
    const auto t0 = std::chrono::steady_clock::now();
    try {
        g.maps = generate_engine_maps(*sd.cycle, sd.fuels[0], q, &pool());
    } catch (const std::exception& e) { // diag only (release); the asan leg never runs [.] cases
        g.threw = true;
        g.what = e.what();
    }
    g.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return g;
}

const char* qname(MapQuality q) { return q == MapQuality::Full ? "Full" : q == MapQuality::Preview ? "Preview" : "Sketch"; }

void report(const char* name, const SimulatedEngineDesc& sd, const std::vector<real>& dyno_rpm) {
    EngineMapsPtr full;
    for (MapQuality q : {MapQuality::Sketch, MapQuality::Preview, MapQuality::Full}) {
        const Gen g = generate(sd, q);
        if (g.threw) {
            std::printf("W7 %-28s %-7s key %016llx THREW after %.3f s: %s\n", name, qname(q), static_cast<unsigned long long>(g.key),
                        g.seconds, g.what.c_str());
            continue;
        }
        std::printf("W7 %-28s %-7s key %016llx hash %016llx v%u held %d %.3f s\n", name, qname(q),
                    static_cast<unsigned long long>(g.key), static_cast<unsigned long long>(g.maps->content_hash),
                    g.maps->generator_version, g.maps->held_plenum_points, g.seconds);
        if (q == MapQuality::Full) full = g.maps;
    }
    if (!full || dyno_rpm.empty()) return;
    DynoSweepSpec spec;
    spec.rpm = dyno_rpm;
    DynoResult r;
    try {
        r = run_dyno_sweep(ComponentParams{sd}, full, spec, &pool());
    } catch (const std::exception& e) { // diag only (release)
        std::printf("W5 %-28s dyno THREW: %s\n", name, e.what());
        return;
    }
    for (const DynoPoint& p : r.points) {
        std::printf("W5 %-28s %6.0f rpm torque %8.3f N*m  p_intake %7.3f kPa  fuel %7.4f g/s  boost %6.3f bar%s\n", name, p.rpm,
                    p.torque_nm, p.p_intake_kpa, p.fuel_g_s, p.boost_bar, p.converged ? "" : " (not converged)");
    }
}

} // namespace

// W5/W7 and the W5h/W8e pins: run once on the unchanged generator ("before") and once on the fixed one ("after").
TEST_CASE("P7d diag: before/after report (keys, hashes, times, WOT dyno)", "[.][diag][cycle][pipe]") {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    const std::vector<real> demo_rpm{1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0};
    report("Mazda P7c pipe experiment", load_desc("engine_pipes_p7c.json", kMazdaDir), {1500.0, 1750.0, 2000.0, 2500.0, 3000.0, 4000.0});
    report("demo_i4_pipes", load_desc("demo_i4_pipes.json", kEngines), demo_rpm);
    report("hyper_v8_one1_audio", load_desc("hyper_v8_one1_audio.json", kEngines), {2000.0, 4000.0, 6000.0});
    report("toyota_a25a_fks_gen (lumped)", load_desc("toyota_a25a_fks_gen.json", kEngines), {});
    report("Mazda KV2 fixture (lumped)", load_desc("engine.json", kMazdaDir), {});
}
