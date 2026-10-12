// tests/unit/test_gas_exchange.cpp - engine garage P7c: the generator's gas exchange (trapped air, intake/IVC
// temperature, residual) against the EPA NVFEL WOT rows of the Toyota A25A-FKS (development engine) and the held-out
// Mazda 2.0 SKYACTIV-G.
//
// Rows: the same literal xlsx readings as tests/unit/test_knock_calibration.cpp (kA25aRows / kMazdaWot), plus the
// workbooks' cell temperature/pressure, catalyst inlet pressure and exhaust port temperature of the same rows
// (<scratchpad>/p7c/rows_extra.py, literal readings [M]).
//
//   diag "P7c budget"  the per-term gap of the lumped cycle model at every row (generator conventions -> measured
//                      boundary conditions, one lever at a time), written to out/garage_p7c_progress.md.
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
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
#include <functional>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;

namespace {

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

SimulatedEngineDesc load_sim(const json& j, const std::filesystem::path& dir, MapQuality q, bool generate) {
    io::EngineMapOptions o;
    o.quality = q;
    o.pool = &pool();
    o.generate = generate;
    const ComponentParams p = io::load_engine_json_from_string(j.dump(), dir, o);
    const auto* e = std::get_if<SimulatedEngineDesc>(&p);
    REQUIRE(e != nullptr);
    return *e;
}

// One measured WOT operating point (literal workbook readings [M]).
struct Row {
    const char* tag;
    real rpm, map_kpa, imt_c, lambda, icam_adv, ecam_ret, spark_btdc;
    real air_g_s;  // A25A: fuel meter x lambda x AFR (no air column); Mazda: the measured inlet air flow
    real fuel_g_s; // fuel meter
    real t_cell_c, p_cell_kpa, p_cat_kpa, t_exh_port_c;
};

// A25A (EPA NVFEL Report 26781, Tier 3 workbook): "High Load Final" rows 23..39 and the near-WOT "Steady State" rows of
// test_knock_calibration.cpp's kA25aRows. air_g_s 0 = fuel x lambda x AFR of the engine's fuel (filled at run time).
constexpr Row kA25a[] = {
    {"H23", 2017.13, 97.2897, 32.1949, 0.79747, 55, 0, -0.8834489, 0, 3.81202, 26.8997, 98.267, 111.281, 608.872},
    {"H26", 2534.94, 97.3509, 32.1453, 0.867008, 55, 0, -0.1004669, 0, 4.46601, 27.3, 98.2473, 115.79, 671.844},
    {"H29", 2995.63, 97.2884, 32.7409, 0.896868, 55, 5, 3.648455, 0, 4.83102, 27.7, 98.2005, 118.418, 689.278},
    {"H32", 3571.24, 97.0007, 32.744, 0.879773, 55, 10, 5.285163, 0, 6.09503, 27.5308, 98.1844, 123.984, 698.565},
    {"H34", 4032.22, 96.8321, 32.6323, 0.859654, 55, 10, 3.459633, 0, 7.28591, 27.1, 98.2054, 131.943, 755.366},
    {"H37", 4493.11, 96.6618, 33.4748, 0.87221, 55, 10, 3.372417, 0, 8.15611, 27.2, 98.2557, 138.927, 785.488},
    {"H39", 4954.03, 96.4381, 34.1517, 0.870633, 52.999, 11.922, 4.398437, 0, 9.08713, 27.4004, 98.2724, 145.658, 803.91},
    {"S27", 1003, 95.87, 32.9, 1, 23.087, 5, -1.194, 0, 1.068, 25.6, 98.37, 102.67, 568.9},
    {"S40", 1153, 95.59, 31.8, 1, 38.175, 0, -3.897, 0, 1.443, 25.9, 98.29, 104.69, 630.1},
    {"S53", 1326, 95.56, 31.4, 1, 50, 0, -6.514, 0, 1.758, 26.2, 98.36, 107.26, 674.6},
    {"S67", 1499, 96.53, 31.7, 1.01, 50, 0, -4.466, 0, 1.978, 26.3, 98.33, 109.33, 690.7},
    {"S82", 1729, 95.34, 31.6, 1, 50, 0, -5.441, 0, 2.304, 26.5, 98.34, 109.74, 720.6},
    {"S94", 2017, 95.51, 31.9, 1, 50, 0, -2.47, 0, 2.697, 27.4, 98.41, 111.81, 739},
    {"S109", 2535, 95.27, 32.9, 1, 50, 0, -1.747, 0, 3.532, 27.8, 98.37, 116.57, 775.2},
    {"S122", 2996, 95.84, 33.4, 1, 50.929, 0, 1.477, 0, 4.061, 29.2, 98.34, 119.24, 775},
    {"S135", 3572, 95.3, 32.2, 1, 49.969, 5, 2.345, 0, 5.074, 26.1, 98.4, 126.3, 811.3},
    {"S143", 4032, 87.7, 32.8, 0.99, 40.837, 0.163, 6.173, 0, 5.144, 26, 98.4, 125.7, 787.1},
};
constexpr std::size_t kNA25a = std::size(kA25a);

// Mazda 2.0 SKYACTIV-G (EPA NVFEL Tier 2 workbook, "Steady State" rows 78..133) - test_knock_calibration.cpp's
// kMazdaWot. Exhaust port temperature = the mean of the four cylinders' readings.
constexpr Row kMazda[] = {
    {"M78", 1746.96, 98.2753, 29.1745, 0.845404, 67.0094, 29.6558, 5.64618, 28.1162, 2.33203, 26.1418, 97.5712, 101.374, 613.334},
    {"M95", 1996.41, 98.1695, 30.3047, 0.848898, 63.86, 20.1108, 5.56535, 35.7123, 2.89866, 27.5825, 97.5955, 101.936, 653.765},
    {"M107", 2496.5, 98.0487, 29.1286, 0.847505, 63.8005, 10.1935, 7.31974, 43.7619, 3.50104, 26.6479, 97.5921, 103.573, 692.46},
    {"M115", 2995.57, 97.0745, 28.0111, 0.84472, 74.565, 24.834, 10.2162, 56.0714, 4.49104, 26.1692, 97.5861, 107.822, 696.121},
    {"M124", 3495.0, 97.2984, 29.0828, 0.845017, 67.0103, 15.3763, 11.374, 63.0635, 5.17707, 27.2079, 97.5663, 111.161, 733.8},
    {"M133", 3994.42, 97.1587, 26.6944, 0.844589, 67.134, 10.1081, 11.6698, 77.9752, 6.16192, 25.6738, 97.5947, 113.225, 763.993},
};
constexpr std::size_t kNMazda = std::size(kMazda);

struct Engine {
    SimulatedEngineDesc desc;
    const CycleModelDesc* c = nullptr;
    const FuelDesc* f = nullptr;
    std::unique_ptr<ec::CycleStepper> proto;
    explicit Engine(SimulatedEngineDesc d) : desc(std::move(d)) {
        REQUIRE(desc.cycle.has_value());
        c = &*desc.cycle;
        f = &desc.fuels[0];
        proto = std::make_unique<ec::CycleStepper>(*c, f->lhv_j_kg, f->afr_stoich);
    }
};

const Engine& a25a() {
    static const Engine e(load_sim(json{{"extends", "toyota_a25a_fks_gen.json"}}, kSrc / "data" / "engines", MapQuality::Sketch, false));
    return e;
}

const std::filesystem::path kMazdaDir = kSrc / "tests" / "support" / "fixtures" / "mazda_skyactiv_g20";

const Engine& mazda() {
    static const Engine e(load_sim(json{{"extends", "engine.json"}}, kMazdaDir, MapQuality::Sketch, false));
    return e;
}

real measured_air_g_s(const Engine& e, const Row& r) {
    return r.air_g_s > 0.0 ? r.air_g_s : r.fuel_g_s * r.lambda * e.f->afr_stoich;
}

// Boundary-condition levers of one cycle-point run (generator conventions = every flag off).
struct Lever {
    bool imt = false;      // measured intake manifold temperature (else ISA 288.15 K)
    bool cam = false;      // measured cam phases (else the engine's cam tables at the node)
    bool fire = false;     // measured lambda and spark (else lambda 1 and the generator's node spark)
    int exhaust = 0;       // exhaust reservoir pressure: 0 ISA 101.325 kPa, 1 cell pressure, 2 catalyst inlet pressure
    bool backflow_t = false; // reservoir backflow gas at the measured exhaust port temperature (else the desc's 800 K)
    real runner_scale = 1.0; // runner length x (sensitivity only)
};

struct GxResult {
    real air_g_s = 0.0;       // net fresh air through the intake valves, engine total
    real trapped_g_s = 0.0;   // fresh mass in the cylinder at IVC, engine total (per-second equivalent)
    real t_ivc = 0.0, x_ivc = 0.0, p_ivc_kpa = 0.0;
    real t_iv_in = 0.0;       // mass-averaged temperature of the forward intake-valve flow
    real backflow_iv_frac = 0.0; // intake-valve reverse mass / forward mass
    real backflow_ev_frac = 0.0; // exhaust-valve reverse mass (into the cylinder) / outflow
    real t_exh = 0.0;         // mass-averaged exhaust-valve outflow temperature
    real imep_gross_kpa = 0.0, pmep_kpa = 0.0;
};

real gen_spark_atdc(const CycleModelDesc& c, real rpm, real p_pa) {
    const bool scheduled = !c.spark_advance_deg.x.empty();
    if (scheduled) return -c.spark_advance_deg.value(rpm, p_pa) + c.ignition_delay_deg;
    return ec::Wiebe::theta0_for_ca50(c.wiebe_a, c.wiebe_m, c.burn_duration_deg, c.ca50_mbt_deg_atdc);
}

GxResult run_row(const Engine& e, const Row& r, const Lever& lv) {
    const CycleModelDesc& c0 = *e.c;
    const ec::CycleStepper* proto = e.proto.get();
    std::unique_ptr<ec::CycleStepper> own;
    if (lv.runner_scale != 1.0 || lv.backflow_t) {
        CycleModelDesc c = c0;
        c.runner_length_m *= lv.runner_scale;
        if (lv.backflow_t) c.exhaust_backflow_temperature_k = r.t_exh_port_c + 273.15;
        own = std::make_unique<ec::CycleStepper>(c, e.f->lhv_j_kg, e.f->afr_stoich);
        proto = own.get();
    }
    ec::OperatingPoint op;
    op.rpm = r.rpm;
    op.p_im_pa = r.map_kpa * 1000.0;
    op.t_im_k = lv.imt ? r.imt_c + 273.15 : 288.15;
    op.p_amb_pa = lv.exhaust == 0 ? 101325.0 : (lv.exhaust == 1 ? r.p_cell_kpa * 1000.0 : r.p_cat_kpa * 1000.0);
    op.lambda = lv.fire ? r.lambda : 1.0;
    op.spark_deg_atdc = lv.fire ? -r.spark_btdc + c0.ignition_delay_deg : gen_spark_atdc(c0, r.rpm, op.p_im_pa);
    if (lv.cam) {
        op.intake_advance_deg = r.icam_adv;
        op.exhaust_retard_deg = r.ecam_ret;
    } else {
        op.intake_advance_deg = c0.cam.intake_advance_deg.x.empty() ? 0.0 : c0.cam.intake_advance_deg.value(op.rpm, op.p_im_pa);
        op.exhaust_retard_deg = c0.cam.exhaust_retard_deg.x.empty() ? 0.0 : c0.cam.exhaust_retard_deg.value(op.rpm, op.p_im_pa);
    }
    ec::CycleStepper s = *proto;
    s.reset(op);
    const ec::StepperSettings set{ec::kStepFull, ec::kCyclesFull};
    const std::uint64_t n = ec::steps_per_cycle(op.rpm, set.h_s);
    const std::uint64_t total = n * std::uint64_t(set.cycles);
    const std::uint64_t last = n * std::uint64_t(set.cycles - 1);
    const real h = 120.0 / (op.rpm * real(n));
    real iv_fwd = 0.0, iv_fwd_t = 0.0, iv_rev = 0.0, ev_out = 0.0, ev_rev = 0.0;
    for (std::uint64_t k = 0; k < total; ++k) {
        if (k == last) {
            s.reset_accumulators();
            s.reset_peak_pressure();
        }
        if (k >= last) {
            const ec::CycleInputs in = s.evaluate(720.0 * real(k % n) / real(n), s.state());
            if (in.mdot_iv > 0.0) {
                iv_fwd += in.mdot_iv * h;
                iv_fwd_t += in.mdot_iv * h * in.ip.t;
            } else {
                iv_rev -= in.mdot_iv * h;
            }
            if (in.mdot_ev > 0.0) ev_out += in.mdot_ev * h; else ev_rev -= in.mdot_ev * h;
        }
        s.step_aligned(k, n);
    }
    const real* y = s.state().y;
    const real cyl = real(c0.cylinders);
    const real per_s = cyl * r.rpm / 120.0 * 1000.0;
    GxResult o;
    o.air_g_s = y[ec::kAccIvFresh] * per_s;
    o.trapped_g_s = s.m_air_trapped() * per_s;
    const ec::CycleModel& m = s.model();
    const real v_ivc = m.geom.eval(s.ivc_crank_deg()).v;
    const ec::CvState iv = ec::cv_state(m.gas, s.ivc_cyl_m(), s.ivc_cyl_u(), s.ivc_cyl_mb(), v_ivc);
    o.t_ivc = iv.t;
    o.x_ivc = iv.x;
    o.p_ivc_kpa = iv.p / 1000.0;
    o.t_iv_in = iv_fwd > 0.0 ? iv_fwd_t / iv_fwd : 0.0;
    o.backflow_iv_frac = iv_fwd > 0.0 ? iv_rev / iv_fwd : 0.0;
    o.backflow_ev_frac = ev_out > 0.0 ? ev_rev / ev_out : 0.0;
    o.t_exh = y[ec::kAccEvOutMass] > 0.0 ? y[ec::kAccEvOutMassT] / y[ec::kAccEvOutMass] : 0.0;
    const real vd = m.geom.v_d;
    o.imep_gross_kpa = y[ec::kAccWorkGross] / vd / 1000.0;
    o.pmep_kpa = -(y[ec::kAccWork] - y[ec::kAccWorkGross]) / vd / 1000.0;
    return o;
}

std::vector<GxResult> run_all(const Engine& e, const Row* rows, std::size_t n, const Lever& lv) {
    std::vector<GxResult> out(n);
    pool().parallel_for(n, 1, [&](std::size_t b, std::size_t en) {
        for (std::size_t i = b; i < en; ++i) out[i] = run_row(e, rows[i], lv);
    });
    return out;
}

void budget(const char* name, const Engine& e, const Row* rows, std::size_t n) {
    struct Term {
        const char* name;
        Lever lv;
    };
    std::vector<Term> terms;
    const Lever g0;
    terms.push_back({"G0 generator conventions (ISA T_im, exhaust 101.3 kPa, cam table, lambda 1, node spark)", g0});
    auto with = [&](const char* nm, auto&& f) {
        Lever l = g0;
        f(l);
        terms.push_back({nm, l});
    };
    with("+ measured IMT", [](Lever& l) { l.imt = true; });
    with("+ measured cam", [](Lever& l) { l.cam = true; });
    with("+ measured lambda + spark", [](Lever& l) { l.fire = true; });
    with("+ exhaust at cell pressure", [](Lever& l) { l.exhaust = 1; });
    with("+ exhaust at catalyst inlet pressure", [](Lever& l) { l.exhaust = 2; });
    with("+ backflow gas at measured port T", [](Lever& l) { l.backflow_t = true; });
    with("BC0 IMT+cam+fire, exhaust cell", [](Lever& l) { l.imt = l.cam = l.fire = true; l.exhaust = 1; });
    with("BC1 IMT+cam+fire, exhaust catalyst inlet", [](Lever& l) { l.imt = l.cam = l.fire = true; l.exhaust = 2; });
    with("BC1 + backflow T measured", [](Lever& l) { l.imt = l.cam = l.fire = l.backflow_t = true; l.exhaust = 2; });
    with("BC1, runner x0.5 (sensitivity)", [](Lever& l) { l.imt = l.cam = l.fire = true; l.exhaust = 2; l.runner_scale = 0.5; });
    with("BC1, runner x0.1 (no ram, sensitivity)", [](Lever& l) { l.imt = l.cam = l.fire = true; l.exhaust = 2; l.runner_scale = 0.1; });
    std::vector<std::vector<GxResult>> res;
    for (const Term& t : terms) res.push_back(run_all(e, rows, n, t.lv));
    std::printf("\n=== %s: measured / model air (inducted) per row\n%-72s", name, "lever");
    for (std::size_t i = 0; i < n; ++i) std::printf(" %6s", rows[i].tag);
    std::printf("\n");
    for (std::size_t k = 0; k < terms.size(); ++k) {
        std::printf("%-72s", terms[k].name);
        for (std::size_t i = 0; i < n; ++i) std::printf(" %6.3f", measured_air_g_s(e, rows[i]) / res[k][i].air_g_s);
        std::printf("\n");
    }
    auto table = [&](const char* what, auto&& get, const char* fmt) {
        std::printf("--- %s\n", what);
        for (std::size_t k = 0; k < terms.size(); ++k) {
            std::printf("%-72s", terms[k].name);
            for (std::size_t i = 0; i < n; ++i) std::printf(fmt, get(res[k][i]));
            std::printf("\n");
        }
    };
    table("trapped / inducted air", [](const GxResult& g) { return g.trapped_g_s / g.air_g_s; }, " %6.3f");
    table("T_ivc [K]", [](const GxResult& g) { return g.t_ivc; }, " %6.1f");
    table("x_ivc (residual)", [](const GxResult& g) { return g.x_ivc; }, " %6.3f");
    table("p_ivc [kPa]", [](const GxResult& g) { return g.p_ivc_kpa; }, " %6.1f");
    table("T intake-valve inflow [K]", [](const GxResult& g) { return g.t_iv_in; }, " %6.1f");
    table("intake-valve backflow / inflow", [](const GxResult& g) { return g.backflow_iv_frac; }, " %6.3f");
    table("exhaust-valve backflow / outflow", [](const GxResult& g) { return g.backflow_ev_frac; }, " %6.3f");
    table("T exhaust-valve outflow [K]", [](const GxResult& g) { return g.t_exh; }, " %6.1f");
    table("PMEP [kPa]", [](const GxResult& g) { return g.pmep_kpa; }, " %6.1f");
    std::printf("measured: ");
    for (std::size_t i = 0; i < n; ++i) {
        std::printf("%s air %.2f g/s IMT %.1f K p_cat %.1f kPa T_port %.0f K | ", rows[i].tag, measured_air_g_s(e, rows[i]),
                    rows[i].imt_c + 273.15, rows[i].p_cat_kpa, rows[i].t_exh_port_c + 273.15);
    }
    std::printf("\n");
}

} // namespace

TEST_CASE("P7c diag: gas-exchange budget per term at the A25A and Mazda WOT rows", "[.][diag][gas_exchange][cycle]") {
    budget("A25A", a25a(), kA25a, kNA25a);
    budget("Mazda (held out; existing fixture, no geometry change)", mazda(), kMazda, kNMazda);
}
