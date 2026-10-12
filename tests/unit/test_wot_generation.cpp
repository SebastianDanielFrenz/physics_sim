// tests/unit/test_wot_generation.cpp - engine garage P7d: the pipe-mode map generator at wide-open throttle and on
// long-valve-overlap engines (the P7c Mazda pipe experiment, tests/support/fixtures/mazda_skyactiv_g20/
// engine_pipes_p7c.json, could not generate Full maps: VE 0.0009 at 1500 rpm / 70 kPa).
//
//   diag "P7d measure"  every grid node of a pipe engine as the generator runs it (trimmed throttle, ISA ambient):
//                       trim error, throttle Cd*A / max, saturation, extra trim cycles, VE, eta_i, residual at IVC,
//                       plenum burned fraction - and per grid rpm the wide-open-throttle state (throttle at its
//                       maximum Cd*A, plenum free). Written to out/garage_p7d_progress.md section 1.
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
