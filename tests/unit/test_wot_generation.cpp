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
#include "engine_cycle/map_cache.h"
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/map_pressure_index.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/cycle_engine.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;
namespace da = ps::jobs::debug_alloc;

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
    if (d.burn_law == CycleModelDesc::BurnLawMode::Standard) {
        // Burn law (2026-10-19, vault PHYS-079): the generator's node_op - ECU spark in Spark mode (the law adds its own
        // delay), MBT tracking relative to the cycle's own MBT.
        op.spark_mode = scheduled ? ec::OperatingPoint::SparkMode::Spark : ec::OperatingPoint::SparkMode::MbtOffset;
        op.spark_deg_atdc = scheduled ? -d.spark_advance_deg.value(rpm, p) : 0.0;
    } else {
        op.spark_deg_atdc = scheduled ? -d.spark_advance_deg.value(rpm, p) + d.ignition_delay_deg : mbt_atdc(d);
    }
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

// =====================================================================================================================
// P7d guards (the hard windows declared above). Each guard's recorded sabotage is in out/garage_p7d_progress.md section 5.
// =====================================================================================================================
namespace {

const SimulatedEngineDesc& mazda_pipes_desc() {
    static const SimulatedEngineDesc d = load_desc("engine_pipes_p7c.json", kMazdaDir);
    return d;
}

EngineMapsPtr mazda_pipes_sketch() {
    static const EngineMapsPtr m =
        generate_engine_maps(*mazda_pipes_desc().cycle, mazda_pipes_desc().fuels[0], MapQuality::Sketch, &pool());
    return m;
}

// W8a: the runtime's own lookup (map_pressure_index + Table2D::value, as SimulatedEngine reads ve/pmep_excess/theta_kl)
// returns every kept node at its own pressure and every WOT node at its own plenum pressure at ISA: the table coordinate
// exactly, the value to <= 1e-15 relative to the largest neighbouring knot (Table2D's bilinear form v0 + t (v1 - v0) at
// t = 1 is 1-2 ulp of the larger operand off an interior knot - for every author node too, before P7d).
void check_nodes_exact(const EngineMaps& em) {
    const SimulatedEngineMapsDesc& m = em.maps;
    REQUIRE(has_wot_rows(m));
    REQUIRE(m.wot_p_im_pa.x == m.ve.x);
    REQUIRE(m.ve.y.back() == kIsaAirPressurePa);
    const std::size_t nr = m.ve.x.size(), np = m.ve.y.size();
    REQUIRE(np >= 2);
    const std::vector<const Table2D*> tables = {&m.ve, &m.pmep_excess_pa, &m.theta_kl_deg};
    std::size_t checked = 0;
    for (std::size_t ir = 0; ir < nr; ++ir) {
        const real rpm = m.ve.x[ir];
        for (std::size_t ip = 0; ip < np; ++ip) {
            const bool wot = ip + 1 == np;
            const real p = wot ? m.wot_p_im_pa.y[ir] : m.ve.y[ip];
            const MapPressureIndex ix = map_pressure_index(m, rpm, p, kIsaAirPressurePa);
            INFO("rpm " << rpm << " p " << p << (wot ? " (WOT)" : ""));
            CHECK(ix.y == m.ve.y[ip]);
            for (const Table2D* t : tables) {
                if (t->values.empty()) continue;
                const real want = t->values[ir * np + ip];
                // 2026-10-20 (burn law phase 3): v0 + t (v1 - v0) at t = 1 rounds to a few ulp of the larger OPERAND, not
                // of the result, so the bound is relative to the largest knot the bilinear lookup can read (the 3 x 3
                // neighbourhood). The 4-row Sketch grid of the Mazda pipe experiment has theta_kl 53.43 / 1.71 / 16.10 deg
                // along rpm 2000 / 2500 / 3000 at 80 kPa: 1.71 comes back 3.55e-15 off (half an ulp of 53.4) against a
                // 1.71e-15 result-relative bound. The value is still the node's own to rounding.
                real scale = std::abs(want);
                for (std::size_t jr = (ir > 0 ? ir - 1 : 0); jr < std::min(nr, ir + 2); ++jr) {
                    for (std::size_t jp = (ip > 0 ? ip - 1 : 0); jp < std::min(np, ip + 2); ++jp) {
                        scale = std::max(scale, std::abs(t->values[jr * np + jp]));
                    }
                }
                INFO("operand scale " << scale);
                CHECK(std::abs(t->value(rpm, ix.y) - want) <= 1e-15 * scale);
                ++checked;
            }
        }
    }
    CHECK(checked >= 2 * nr * np);
}

// Whether a file holds the .psmaps WOT block tag (0x31765424F5744F37, little-endian).
bool file_has_wot_tag(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    const std::vector<unsigned char> b((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    REQUIRE(!b.empty());
    constexpr unsigned char tag[8] = {0x37, 0x4F, 0x74, 0xF5, 0x24, 0x54, 0x76, 0x31};
    return std::search(b.begin(), b.end(), std::begin(tag), std::end(tag)) != b.end();
}

struct TmpDir {
    std::filesystem::path path;
    explicit TmpDir(const char* name) {
        path = std::filesystem::temp_directory_path() /
               (std::string("ps_test_wot_") + name + "_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }
    ~TmpDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
    [[nodiscard]] std::size_t files() const {
        std::size_t n = 0;
        for (const auto& e : std::filesystem::recursive_directory_iterator(path)) n += e.is_regular_file() ? 1 : 0;
        return n;
    }
};

struct Pin {
    MapQuality q;
    std::uint64_t key, hash;
};

void check_pins(const char* file, const std::filesystem::path& dir, const std::vector<Pin>& pins, bool generate) {
    const SimulatedEngineDesc sd = load_desc(file, dir);
    for (const Pin& p : pins) {
        INFO(file << " " << qname(p.q));
        const std::uint64_t key = engine_map_cache_key(*sd.cycle, sd.fuels[0], p.q);
        std::printf("P7d pins %s %s: key 0x%016llx", file, qname(p.q), static_cast<unsigned long long>(key));
        CHECK(key == p.key);
        if (!generate) {
            std::printf("\n");
            continue;
        }
        const EngineMapsPtr m = generate_engine_maps(*sd.cycle, sd.fuels[0], p.q, &pool());
        std::printf(", content_hash 0x%016llx\n", static_cast<unsigned long long>(m->content_hash));
        CHECK(m->content_hash == p.hash);
        CHECK(!has_wot_rows(m->maps));
    }
}

} // namespace

// W1 (Sketch; Preview and Full in the [long] case below) and W8a on the generated tables.
TEST_CASE("P7d W1: the Mazda pipe experiment generates Sketch maps with a WOT top row", "[cycle][pipe][drivetrain]") {
    const EngineMapsPtr m = mazda_pipes_sketch();
    REQUIRE(m != nullptr);
    std::printf("P7d W1 Mazda pipe Sketch: content_hash 0x%016llx, rows", static_cast<unsigned long long>(m->content_hash));
    for (real y : m->maps.ve.y) std::printf(" %.0f", y);
    std::printf(", non-breathing filled %d, dropped rows %d, held %d\n", m->nonbreathing_points, m->wot_dropped_rows,
                m->held_plenum_points);
    CHECK(m->generator_version == kLawPipeWotGeneratorVersion); // burn law 2026-10-19 (was kPipeWotGeneratorVersion 4)
    CHECK(m->nonbreathing_points >= 1); // the long-overlap nodes the fix exists for
    for (real v : m->maps.ve.values) CHECK((std::isfinite(v) && v > 0.0));
    check_nodes_exact(*m);
}

TEST_CASE("P7d W1: the Mazda pipe experiment generates Preview and Full maps", "[cycle][pipe][drivetrain][long]") {
    const SimulatedEngineDesc& sd = mazda_pipes_desc();
    for (MapQuality q : {MapQuality::Preview, MapQuality::Full}) {
        INFO(qname(q));
        const EngineMapsPtr m = generate_engine_maps(*sd.cycle, sd.fuels[0], q, &pool());
        REQUIRE(m != nullptr);
        std::printf("P7d W1 Mazda pipe %s: content_hash 0x%016llx, non-breathing %d, dropped %d, held %d\n", qname(q),
                    static_cast<unsigned long long>(m->content_hash), m->nonbreathing_points, m->wot_dropped_rows,
                    m->held_plenum_points);
        for (real v : m->maps.ve.values) CHECK((std::isfinite(v) && v > 0.0));
        check_nodes_exact(*m);
    }
}

// W8a, second half: the WOT node is the wide-open-throttle cycle itself - its plenum pressure is the curve value bit for
// bit, and the runtime's VE there (air = VE x p_wot / (R T) x V_d / 2) is that cycle's trapped air.
TEST_CASE("P7d W8a: the WOT row is the wide-open-throttle cycle at its own plenum pressure", "[cycle][pipe][drivetrain]") {
    const SimulatedEngineDesc& sd = mazda_pipes_desc();
    const EngineMapsPtr m = mazda_pipes_sketch();
    const SimulatedEngineMapsDesc& mm = m->maps;
    REQUIRE(has_wot_rows(mm));
    const std::size_t np = mm.ve.y.size();
    for (std::size_t ir : {std::size_t(0), mm.ve.x.size() / 2, mm.ve.x.size() - 1}) {
        const real rpm = mm.ve.x[ir];
        const NodeOut o = run_one(*sd.cycle, sd.fuels[0], gen_op(*sd.cycle, rpm, kIsaAirPressurePa), true, false);
        const real ve_rt = mm.ve.value(rpm, map_pressure_index(mm, rpm, o.p_mean, kIsaAirPressurePa).y);
        std::printf("P7d W8a %6.0f rpm: p_wot %.6f Pa (curve %.6f), VE cycle %.15f runtime %.15f\n", rpm, o.p_mean,
                    mm.wot_p_im_pa.y[ir], o.ve, ve_rt);
        CHECK(o.p_mean == mm.wot_p_im_pa.y[ir]);
        CHECK(std::abs(ve_rt - mm.ve.values[ir * np + np - 1]) <= 1e-15 * ve_rt);
        CHECK(std::abs(ve_rt / o.ve - 1.0) <= 1e-12);
    }
}

// W8b.
TEST_CASE("P7d W8b: the Mazda pipe experiment's Preview maps are identical on 1 and 10 workers",
          "[cycle][pipe][drivetrain][long]") {
    const SimulatedEngineDesc& sd = mazda_pipes_desc();
    jobs::JobSystem one(1);
    const EngineMapsPtr a = generate_engine_maps(*sd.cycle, sd.fuels[0], MapQuality::Preview, &one);
    const EngineMapsPtr b = generate_engine_maps(*sd.cycle, sd.fuels[0], MapQuality::Preview, &pool());
    CHECK(a->content_hash == b->content_hash);
    CHECK(a->maps.ve.values == b->maps.ve.values);
    CHECK(a->maps.wot_p_im_pa.y == b->maps.wot_p_im_pa.y);
    CHECK(a->maps.theta_kl_deg.values == b->maps.theta_kl_deg.values);
}

// W8c + W8d: the WOT cycle run allocates nothing inside a job; progress once per point incl. the WOT points, on the
// calling thread; cancel throws GenerationCancelled and a cancelled load writes no cache file. A pipe engine's
// generation job also CONSTRUCTS its CycleEngine (pipe network, steppers), which allocates - on every pipe engine since
// A5 ("run() single-use and allocation-free"), so the whole-generation count is reported for the v3 and v4 paths, and the
// assertion is on the run (progress file section 1, deviation from the declared "zero allocation inside generation jobs").
TEST_CASE("P7d W8c/W8d: WOT points keep the P1 allocation, progress and cancel contract", "[cycle][pipe][drivetrain][long]") {
    const SimulatedEngineDesc& sd = mazda_pipes_desc();
    const CycleModelDesc& d = *sd.cycle;
    CycleModelDesc no_wot = d;
    no_wot.intake->compressor_supply = true; // the v3 path: same grid and sweeps, no WOT points
    const std::size_t total = EngineMapGenTask(d, sd.fuels[0], MapQuality::Sketch).point_count();
    const std::size_t total_v3 = EngineMapGenTask(no_wot, sd.fuels[0], MapQuality::Sketch).point_count();
    const std::size_t rpm_n = mazda_pipes_sketch()->maps.ve.x.size();
    CHECK(total == total_v3 + rpm_n);

    if (!da::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    {
        ec::CycleEngineSettings set;
        set.h_s = ec::kStepPreview;
        set.settling_cycles = ec::kEngineSettlingCyclesPreview;
        set.wide_open_throttle = true;
        ec::CycleEngine eng(d, sd.fuels[0].lhv_j_kg, sd.fuels[0].afr_stoich, gen_op(d, 2500.0, kIsaAirPressurePa), set);
        ec::CycleEngineResult r{};
        da::reset();
        da::set_armed(true);
        pool().parallel_for(1, 1, [&](std::size_t, std::size_t) { r = eng.run(); });
        da::set_armed(false);
        std::printf("P7d W8c WOT run in a job: own violations %llu (p_wot %.1f Pa)\n",
                    static_cast<unsigned long long>(da::own_violation_count()), r.p_plenum_mean_pa);
        CHECK(da::own_violation_count() == 0);
        CHECK(r.p_plenum_mean_pa > 0.0);
    }
    jobs::JobSystem four(4);
    da::reset();
    da::set_armed(true);
    (void)generate_engine_maps(no_wot, sd.fuels[0], MapQuality::Sketch, &four);
    da::set_armed(false);
    const unsigned long long v3_violations = da::own_violation_count();
    std::vector<std::size_t> done;
    std::vector<std::thread::id> threads;
    done.reserve(total);
    threads.reserve(total);
    std::atomic<bool> never{false};
    MapGenerationControl c;
    c.cancel = &never;
    c.on_progress = [&](std::size_t n, std::size_t tot) {
        done.push_back(tot == total ? n : 0);
        threads.push_back(std::this_thread::get_id());
    };
    da::reset();
    da::set_armed(true);
    const EngineMapsPtr m = generate_engine_maps(d, sd.fuels[0], MapQuality::Sketch, &four, &c);
    da::set_armed(false);
    std::printf("P7d W8c whole Sketch generation (reported): own violations v4 %llu over %zu points, v3 path %llu over %zu\n",
                static_cast<unsigned long long>(da::own_violation_count()), total, v3_violations, total_v3);
    REQUIRE(done.size() == total);
    for (std::size_t i = 0; i < total; ++i) {
        CHECK(done[i] == i + 1);
        CHECK(threads[i] == std::this_thread::get_id());
    }
    CHECK(m->content_hash == mazda_pipes_sketch()->content_hash);

    // cancel after the third point through the loader with a cache directory: GenerationCancelled, no file written
    TmpDir td("cancel");
    std::atomic<bool> cancel{false};
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &four;
    o.cache_dir = td.path.string();
    o.cancel = &cancel;
    o.on_progress = [&](std::size_t n, std::size_t) {
        if (n == 3) cancel.store(true);
    };
    CHECK_THROWS_AS(io::load_engine_json_from_string(json{{"extends", "engine_pipes_p7c.json"}}.dump(), kMazdaDir, o),
                    GenerationCancelled);
    CHECK(td.files() == 0);
}

// 2026-10-18: pins re-recorded for the knock opt-in of the A25A gen and the hyper (cause: knock opt-in; octane + margin join the key and hash; the Mazda fixture is not opted in and kept its pins).
// 2026-10-18 (2): hyper pins re-recorded again for the E85 fuel switch (cause: fuel E5 -> E85, knock RON 104).
// W5h + W8e: compressor-fed pipe engines (the hyper) and lumped engines keep their key and content hash at every quality
// (pins recorded on the unchanged generator, header). Keys always; the fast hashes here, the hyper's slow ones [long].
TEST_CASE("P7d W5h/W8e: the hyper and lumped engines keep their keys and map hashes", "[cycle][pipe][drivetrain]") {
    // 2026-10-19 burn law F + knock model v5 (vault PHYS-079) moved the A25A gen and Mazda keys and hashes; was A25A
    // 0x7d4f5df61b83fd3d/0x75f0658d60c662b8, 0xbd44b606c7ef765f/0x4e108f53ec6e4541, 0x1d4a09fe71b9b9ce/0x470361fea395f4f3,
    // Mazda 0xd64488ea1fff9442/0x1bb312f8a650b0f4, 0x964f30d973941b20/0xbafd38470361fbab, 0x3649dce1c9c9d7b1/0x94534cc297971f9e.
    // 2026-10-20 burn law v3 + knock model v6 (PHYS-079 phase 3) moved them again, and the Sketch rows again for the
    // 4-row Sketch grid (kSketchGridVersion 2 in the Sketch key); was A25A 0x2fe4dae6fb085a53/0x9fa4fb02b889c7ab,
    // 0xefef82d64e9ce131/0x2e71479db8c0b848, 0x4ff4d6cdf86724a0/0x2eaa40899574544b, Mazda 0x8140a538e6448b19/
    // 0x23a4d1f462b9a556, 0xc135fd4992b0043b/0x117bd3ef7ecc0cf8, 0x213b51413c7a47aa/0x7cec2a203265eb47.
    // Hyper re-pinned 2026-10-20 (burn v3 / knock v6, owner 2026-10-20: hyper is a guess, not tuned; Sketch also
    // kSketchGridVersion 2): was Sketch 0x98217b790e718eb9/0xa09334bc88debc6c, Preview 0xd816d389badd07db/
    // 0xc962e26f4009c30b, Full 0x381c278164a74b4a/0xd1dee97c3fdcaa0c.
    check_pins("toyota_a25a_fks_gen.json", kEngines,
               {{MapQuality::Sketch, 0x253bbfd0ceb28042ull, 0x20f029ffc4fae10full},
                {MapQuality::Preview, 0x7e9d6c1b70121e42ull, 0xc46a220a9a92063bull},
                {MapQuality::Full, 0x1e981823c647dad3ull, 0xaf2fe424cda51481ull}},
               true);
    check_pins("engine.json", kMazdaDir,
               {{MapQuality::Sketch, 0x3f02abcdb78fb872ull, 0xa14edf744d9401d1ull},
                {MapQuality::Preview, 0x029974d1e620ab72ull, 0x72cd7ab175cfb86bull},
                {MapQuality::Full, 0xa29420da3c566803ull, 0xfd7d1733c21dabd5ull}},
               true);
    check_pins("hyper_v8_one1_audio.json", kEngines, {{MapQuality::Sketch, 0x0a5d355fdc243a17ull, 0x6a233d6475a57032ull}}, true);
    check_pins("hyper_v8_one1_audio.json", kEngines,
               {{MapQuality::Preview, 0x8bb600415357f947ull, 0}, {MapQuality::Full, 0xebbb5438fd223cb6ull, 0}}, false);
}

TEST_CASE("P7d W5h: the hyper keeps its Preview and Full map hashes", "[cycle][pipe][drivetrain][turbo][long]") {
    check_pins("hyper_v8_one1_audio.json", kEngines,
               {{MapQuality::Preview, 0x8bb600415357f947ull, 0xfc3d03b81646d8a1ull},
                {MapQuality::Full, 0xebbb5438fd223cb6ull, 0xbc895c1fc6ad4d58ull}},
               true);
}

// W8f: the warp on a synthetic map (no generator).
TEST_CASE("P7d W8f: map_pressure_index is the identity without a WOT curve, continuous, with exact slopes",
          "[cycle][pipe][drivetrain]") {
    SimulatedEngineMapsDesc m;
    m.ve.x = {1000.0, 3000.0, 5000.0};
    m.ve.y = {50000.0, 90000.0, 101325.0};
    m.ve.values.assign(9, 0.8);
    for (real p : {20000.0, 90000.0, 99000.0, 101325.0, 130000.0}) { // identity without a curve, at any ambient
        const MapPressureIndex ix = map_pressure_index(m, 2000.0, p, 95000.0);
        CHECK(ix.y == p);
        CHECK(ix.dy_dp == 1.0);
        CHECK(ix.dy_drpm == 0.0);
    }
    m.wot_p_im_pa.x = {1000.0, 3000.0, 5000.0};
    m.wot_p_im_pa.y = {100500.0, 101100.0, 99800.0};
    for (std::size_t i = 0; i < 3; ++i) { // exact at the author rows and the WOT nodes at ISA
        const real rpm = m.wot_p_im_pa.x[i];
        CHECK(map_pressure_index(m, rpm, 50000.0, kIsaAirPressurePa).y == 50000.0);
        CHECK(map_pressure_index(m, rpm, 90000.0, kIsaAirPressurePa).y == 90000.0);
        CHECK(map_pressure_index(m, rpm, m.wot_p_im_pa.y[i], kIsaAirPressurePa).y == 101325.0);
        const real amb = 90000.0; // the WOT plenum referred to another ambient reads the WOT row too
        CHECK(std::abs(map_pressure_index(m, rpm, m.wot_p_im_pa.y[i] * amb / kIsaAirPressurePa, amb).y - 101325.0) <= 1e-9);
    }
    const real above = map_pressure_index(m, 2000.0, 90000.0 * (1.0 + 1e-12), kIsaAirPressurePa).y; // continuous
    CHECK(std::abs(above - 90000.0) <= 1e-6);
    for (real rpm : {1500.0, 2500.0, 4200.0}) { // slopes vs central differences, away from knots
        for (real amb : {kIsaAirPressurePa, 92000.0}) {
            for (real p : {60000.0, 95000.0, 100000.0}) {
                INFO("rpm " << rpm << " amb " << amb << " p " << p);
                const MapPressureIndex ix = map_pressure_index(m, rpm, p, amb);
                const real hp = 1e-3, hr = 1e-3;
                const real dp =
                    (map_pressure_index(m, rpm, p + hp, amb).y - map_pressure_index(m, rpm, p - hp, amb).y) / (2.0 * hp);
                const real dr =
                    (map_pressure_index(m, rpm + hr, p, amb).y - map_pressure_index(m, rpm - hr, p, amb).y) / (2.0 * hr);
                CHECK(std::abs(ix.dy_dp - dp) <= 1e-6 * std::abs(dp));
                CHECK(std::abs(ix.dy_drpm - dr) <= 1e-6 * std::max(std::abs(dr), 1e-3));
            }
        }
    }
}

// W8g: the loader rejects an authored p_im-axis table next to generated WOT rows; without one it carries the curve.
TEST_CASE("P7d W8g: the loader carries the WOT curve and rejects an authored ve beside it", "[cycle][pipe][drivetrain][io]") {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool();
    const ComponentParams ok =
        io::load_engine_json_from_string(json{{"extends", "engine_pipes_p7c.json"}}.dump(), kMazdaDir, o);
    const auto* e = std::get_if<SimulatedEngineDesc>(&ok);
    REQUIRE(e != nullptr);
    CHECK(!e->maps_pending);
    CHECK(e->maps.wot_p_im_pa.x == e->maps.ve.x);
    CHECK(e->maps.wot_p_im_pa.y == mazda_pipes_sketch()->maps.wot_p_im_pa.y);
    json j{{"extends", "engine_pipes_p7c.json"}};
    j["maps"]["source"] = "P7d W8g test table";
    j["maps"]["ve"] = json{{"rpm", {1000, 6500}}, {"p_im_kpa", {20, 100}}, {"values", {{0.5, 0.9}, {0.5, 0.9}}}};
    REQUIRE_THROWS_WITH(io::load_engine_json_from_string(j.dump(), kMazdaDir, o),
                        Catch::Matchers::ContainsSubstring("carry a wide-open-throttle row"));
}

// W8h: the .psmaps round trip keeps the WOT curve; a file without one carries no WOT block (the old format).
TEST_CASE("P7d W8h: the map cache keeps the WOT curve and writes the old format without one", "[cycle][pipe][drivetrain][io]") {
    TmpDir td("cache");
    const std::string dir = td.path.generic_string();
    const EngineMapsPtr m = mazda_pipes_sketch();
    store_cached_engine_maps(dir, 1, *m);
    const EngineMapsPtr hit = load_cached_engine_maps(dir, 1, m->generator_version); // the maps' own stamp (burn law: v7, 2026-10-19)
    REQUIRE(hit != nullptr);
    CHECK(hit->content_hash == m->content_hash);
    CHECK(hit->maps.wot_p_im_pa.x == m->maps.wot_p_im_pa.x);
    CHECK(hit->maps.wot_p_im_pa.y == m->maps.wot_p_im_pa.y);
    CHECK(hit->maps.theta_kl_deg.values == m->maps.theta_kl_deg.values);
    CHECK(file_has_wot_tag(td.path / "0000000000000001.psmaps"));
    const SimulatedEngineDesc lumped = load_desc("engine.json", kMazdaDir); // lumped: no WOT block in the file
    const EngineMapsPtr l = generate_engine_maps(*lumped.cycle, lumped.fuels[0], MapQuality::Sketch, &pool());
    store_cached_engine_maps(dir, 2, *l);
    CHECK(!file_has_wot_tag(td.path / "0000000000000002.psmaps"));
    const EngineMapsPtr lhit = load_cached_engine_maps(dir, 2, l->generator_version); // burn law: v5 (2026-10-19)
    REQUIRE(lhit != nullptr);
    CHECK(lhit->content_hash == l->content_hash);
    CHECK(!has_wot_rows(lhit->maps));
}

// W2/W3/W4 (reported, held out): the Mazda pipe experiment's generated Full maps against the EPA WOT rows.
namespace {
struct EpaRow {
    const char* tag;
    real rpm, map_kpa, spark_btdc, air_g_s, t_cell_c, p_cell_kpa;
};
// EPA NVFEL Tier 2 workbook "Steady State" rows 78..133 [M] - the fields of test_gas_exchange.cpp's kMazda.
constexpr EpaRow kMazdaRows[] = {
    {"M78", 1746.96, 98.2753, 5.64618, 28.1162, 26.1418, 97.5712},
    {"M95", 1996.41, 98.1695, 5.56535, 35.7123, 27.5825, 97.5955},
    {"M107", 2496.5, 98.0487, 7.31974, 43.7619, 26.6479, 97.5921},
    {"M115", 2995.57, 97.0745, 10.2162, 56.0714, 26.1692, 97.5861},
    {"M124", 3495.0, 97.2984, 11.374, 63.0635, 27.2079, 97.5663},
    {"M133", 3994.42, 97.1587, 11.6698, 77.9752, 25.6738, 97.5947},
};
} // namespace

TEST_CASE("P7d W2/W3/W4 report: Mazda pipe experiment Full maps at the EPA WOT rows", "[.][diag][cycle][pipe]") {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    io::EngineMapOptions o;
    o.quality = MapQuality::Full;
    o.pool = &pool();
    const ComponentParams cp =
        io::load_engine_json_from_string(json{{"extends", "engine_pipes_p7c.json"}}.dump(), kMazdaDir, o);
    const SimulatedEngineDesc& e = std::get<SimulatedEngineDesc>(cp);
    const SimulatedEngineMapsDesc& m = e.maps;
    const CycleModelDesc& c = *e.cycle;
    const real v_d = real(c.cylinders) * 3.14159265358979323846 / 4.0 * c.bore_m * c.bore_m * c.stroke_m;
    std::vector<NodeOut> wot(std::size(kMazdaRows));
    pool().parallel_for(wot.size(), 1, [&](std::size_t b, std::size_t en) {
        for (std::size_t i = b; i < en; ++i) {
            wot[i] = run_one(c, e.fuels[0], gen_op(c, kMazdaRows[i].rpm, kIsaAirPressurePa), true, true);
        }
    });
    std::printf("W2 row     rpm    MAP  p_cell  y_table  meas_g_s model_g_s  R_air (band 0.94..1.06)\n");
    for (const EpaRow& r : kMazdaRows) {
        const real p = r.map_kpa * 1000.0;
        const real y = map_pressure_index(m, r.rpm, p, r.p_cell_kpa * 1000.0).y;
        const real model =
            m.ve.value(r.rpm, y) * p / (kIsaAirGasConstantJKgK * (r.t_cell_c + 273.15)) * v_d * r.rpm / 120.0 * 1000.0;
        std::printf("W2 %5s %6.0f %6.2f %6.2f %8.0f %8.3f %9.3f %7.3f\n", r.tag, r.rpm, r.map_kpa, r.p_cell_kpa, y, r.air_g_s,
                    model, r.air_g_s / model);
    }
    for (std::size_t i = 0; i < std::size(kMazdaRows); ++i) {
        std::printf("W3 %5s %6.0f rpm: WOT T_ivc %6.1f K (band 359.8..423.0), p_wot %.2f kPa, VE %.4f, residual %.3f\n",
                    kMazdaRows[i].tag, kMazdaRows[i].rpm, wot[i].t_ivc, wot[i].p_mean / 1000.0, wot[i].ve, wot[i].resid);
    }
    REQUIRE(m.has_theta_kl);
    real ss = 0.0;
    for (const EpaRow& r : kMazdaRows) {
        const real y = map_pressure_index(m, r.rpm, r.map_kpa * 1000.0, r.p_cell_kpa * 1000.0).y;
        const real t = m.theta_kl_deg.value(r.rpm, y);
        ss += (t - r.spark_btdc) * (t - r.spark_btdc);
        std::printf("W4 %5s %6.0f rpm: theta_kl %7.2f measured %6.2f diff %+7.2f (band -5..+10)\n", r.tag, r.rpm, t,
                    r.spark_btdc, t - r.spark_btdc);
    }
    std::printf("W4 rms %.2f deg\n", std::sqrt(ss / real(std::size(kMazdaRows))));
}
