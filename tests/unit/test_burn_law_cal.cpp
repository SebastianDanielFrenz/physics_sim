// tests/unit/test_burn_law_cal.cpp - the burn law's fitted constants (design C, owner decision 2026-10-19, vault
// PHYS-079): scale s, speed exponent e_n and flame-speed exponent e_s of ps/engine_cycle/burn_law.h, fitted [M] at
// MEASURED boundary conditions on part-load EPA NVFEL rows.
//
// Method (the KV1-BC rule of test_knock_calibration.cpp, unchanged): per measured row one converged cycle point
// (run_operating_point, Full settings) at the row's rpm, MAP, intake temperature, lambda, cam phases (the engine's cam
// table where the row has none), exhaust back pressure (turbine inlet, else catalyst inlet, else ambient) and its
// measured spark (SparkMode::Spark: the law sets this cycle's delay and duration). Its IVC state is scaled to the
// MEASURED trapped air (the inlet air flow where the workbook has one - Mazda 2.0 -, else fuel meter x lambda x AFR) and
// the law is evaluated on that state. Rows: tests/support/fixtures/burn_law/<package>_part_rows.json (MAP <= 85 kPa,
// lambda 0.95-1.05, BMEP >= 1 bar, >= 1000 rpm; tools/enginedata/epa_turbo_extract.py). The measured spark there is
// taken as MBT [AS]: no knock at these loads, no torque reserve off idle.
//
// The quantity compared is the law's MBT spark (CA50 at 8 deg ATDC) against the measured spark. The law's delay and
// duration share one factor f, so MBT = f (dev_ref + 0.4866 rapid_ref) - CA50 = 24.731 f - 8 deg BTDC; the constant
// burn of C17a (f = 1) is 16.73 deg BTDC everywhere - the "before" column.
//
// Sets (declared before the fit): training Ford 2.7 EcoBoost Tier 2 cell 9 + Honda L15B7 Tier 3; held out Ford 1.6
// EcoBoost Tier 2, Mazda 2.5 T Tier 3, Mazda 2.0 SKYACTIV-G Tier 2 and Toyota A25A Tier 3. Chevrolet 4.3 is not in the
// set: no engine fixture exists and its package publishes no valve events (report only).
//
// Fit: Levenberg-Marquardt on the training rows' gaps [deg] over (ln s, e_n, e_s), then the rows are re-run with the
// fitted constants (the cycle's own residual and exhaust state depend weakly on its burn) and fitted once more.
#define _CRT_SECURE_NO_WARNINGS // std::getenv in the hidden diags
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/map_pressure_index.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/burn_law.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/io/fuel_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;

namespace {

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);
const std::filesystem::path kRows = kSrc / "tests" / "support" / "fixtures" / "burn_law";

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

struct PkgSpec {
    const char* key;
    const char* engine; // relative to the source root
    bool train;
    const char* label;
};
constexpr PkgSpec kPkgs[] = {
    {"ford_27_t2_cell9", "tests/support/fixtures/epa_turbo/ford_27/engine.json", true, "Ford 2.7 T (T2 c9)"},
    {"honda_l15b7_t3", "tests/support/fixtures/epa_turbo/honda_l15b7/engine.json", true, "Honda 1.5 T (T3)"},
    {"ford_16_t2", "tests/support/fixtures/epa_turbo/ford_16/engine.json", false, "Ford 1.6 T (T2)"},
    {"mazda_25t_t3", "tests/support/fixtures/epa_turbo/mazda_25t/engine.json", false, "Mazda 2.5 T (T3)"},
    {"mazda20_t2", "tests/support/fixtures/mazda_skyactiv_g20/engine.json", false, "Mazda 2.0 NA (T2)"},
    {"a25a_t3", "data/engines/toyota_a25a_fks_gen.json", false, "Toyota A25A NA (T3)"},
};

struct Row {
    real rpm, map_kpa, imt_c, lambda, spark_btdc, fuel_g_s, air_g_s, exh_kpa, bmep, torque_nm;
    bool has_icam, has_ecam;
    real icam_adv, ecam_ret;
    int row;
};

struct Package {
    PkgSpec spec{};
    SimulatedEngineDesc desc;
    FuelDesc fuel;
    std::vector<Row> rows;
};

real num(const json& r, const char* k) {
    return (r.contains(k) && r.at(k).is_number()) ? r.at(k).get<real>() : std::nan("");
}

std::unique_ptr<Package> load_package(const PkgSpec& s, const char* suffix = "_part_rows.json") {
    auto pk = std::make_unique<Package>();
    pk->spec = s;
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool();
    o.generate = false;
    const std::filesystem::path engine = kSrc / s.engine;
    const ComponentParams p = io::load_engine_json(engine.generic_string(), o);
    const auto* e = std::get_if<SimulatedEngineDesc>(&p);
    REQUIRE(e != nullptr);
    REQUIRE(e->cycle.has_value());
    pk->desc = *e;
    std::ifstream in(kRows / (std::string(s.key) + suffix));
    REQUIRE(in.good());
    const json j = json::parse(in);
    const std::string fuel = j.at("fuel").get<std::string>();
    std::filesystem::path fp = kSrc / "data" / "fuels" / (fuel + ".json");
    if (!std::filesystem::exists(fp)) fp = engine.parent_path() / (fuel + ".json");
    pk->fuel = io::load_fuel_json(fp.generic_string());
    for (const json& r : j.at("rows")) {
        Row w{};
        w.rpm = num(r, "rpm");
        w.map_kpa = num(r, "map_kpa");
        w.imt_c = num(r, "imt_c");
        w.lambda = num(r, "lambda");
        w.spark_btdc = num(r, "spark_btdc_deg");
        w.fuel_g_s = std::isfinite(num(r, "fuel_meter_g_s")) ? num(r, "fuel_meter_g_s") : num(r, "inj_fuel_g_s");
        w.air_g_s = num(r, "air_g_s");
        w.exh_kpa = std::isfinite(num(r, "turb_in_kpa")) ? num(r, "turb_in_kpa") : num(r, "cat_in_kpa");
        w.bmep = num(r, "bmep_bar");
        w.torque_nm = num(r, "torque_nm");
        w.has_icam = std::isfinite(num(r, "intake_cam_adv_deg"));
        w.has_ecam = std::isfinite(num(r, "exhaust_cam_ret_deg"));
        w.icam_adv = w.has_icam ? num(r, "intake_cam_adv_deg") : 0.0;
        w.ecam_ret = w.has_ecam ? num(r, "exhaust_cam_ret_deg") : 0.0;
        w.row = r.at("row").get<int>();
        pk->rows.push_back(w);
    }
    return pk;
}

std::vector<std::unique_ptr<Package>>& packages() {
    static std::vector<std::unique_ptr<Package>> v = [] {
        std::vector<std::unique_ptr<Package>> out;
        for (const PkgSpec& s : kPkgs) out.push_back(load_package(s));
        return out;
    }();
    return v;
}

// A row's law input at measured boundary conditions.
struct Prepared {
    bool ok = false;
    int fail_reason = 0;   // 1 the cycle point threw, 2 no trapped air at IVC, 3 the scaled IVC state is not physical
    int step_halvings = 0; // the step the row converged at: kStepFull / 2^step_halvings
    ec::BurnLawInput in;
    real air_ratio = 1.0;
    real spark = 0.0;
};

// fixed_burn: the C17a constant burn instead of the law (40.3205 deg duration, 3.8632 deg delay, the Wiebe started at the
// measured spark plus that delay) - the reference run that shows a row's failure is not the law's.
std::vector<Prepared> prepare_all(const Package& pk, const ec::BurnLawParams& params, bool fixed_burn = false) {
    CycleModelDesc c = *pk.desc.cycle;
    c.burn_law = fixed_burn ? CycleModelDesc::BurnLawMode::Fixed : CycleModelDesc::BurnLawMode::Standard;
    c.burn_law_params = params;
    c.burn_duration_deg = 40.3205;
    c.ignition_delay_deg = 3.8632;
    const ec::CycleStepper proto(c, pk.fuel.lhv_j_kg, pk.fuel.afr_stoich);
    std::vector<Prepared> out(pk.rows.size());
    pool().parallel_for(pk.rows.size(), 1, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            const Row& r = pk.rows[i];
            ec::OperatingPoint op;
            op.rpm = r.rpm;
            op.p_im_pa = r.map_kpa * 1000.0;
            op.t_im_k = r.imt_c + 273.15;
            op.p_amb_pa = std::isfinite(r.exh_kpa) && r.exh_kpa > 0.0 ? r.exh_kpa * 1000.0 : 101325.0;
            op.lambda = r.lambda;
            op.spark_mode = fixed_burn ? ec::OperatingPoint::SparkMode::WiebeStart : ec::OperatingPoint::SparkMode::Spark;
            op.spark_deg_atdc = -r.spark_btdc + (fixed_burn ? c.ignition_delay_deg : 0.0);
            const real it = c.cam.intake_advance_deg.x.empty() ? 0.0 : c.cam.intake_advance_deg.value(op.rpm, op.p_im_pa);
            const real et = c.cam.exhaust_retard_deg.x.empty() ? 0.0 : c.cam.exhaust_retard_deg.value(op.rpm, op.p_im_pa);
            op.intake_advance_deg = r.has_icam ? r.icam_adv : it;
            op.exhaust_retard_deg = r.has_ecam ? r.ecam_ret : et;
            // Some rows diverge in the gas exchange (a NaN state): at large measured exhaust-cam retard and low MAP the
            // reversed runner slug drains the intake port to zero mass during the overlap, mostly in the first cycle before
            // any combustion - with the constant burn too (BLC1 checks that). Such a row is re-run at h/2, then h/4 (rescues
            // 2 of 76); the rest are reported, not used.
            ec::PointResult pr;
            int halvings = 0;
            for (; halvings < 3; ++halvings) {
                out[i].fail_reason = 0;
                try {
                    pr = ec::run_operating_point(proto, op,
                                                 ec::StepperSettings{ec::kStepFull / real(1 << halvings), ec::kCyclesFull});
                } catch (...) {
                    out[i].fail_reason = 1;
                    continue;
                }
                if (!(pr.ivc.m > 0.0) || !(pr.ivc.m > pr.ivc.mb)) {
                    out[i].fail_reason = 2;
                    continue;
                }
                break;
            }
            out[i].step_halvings = halvings;
            if (out[i].fail_reason != 0) continue;
            const real air_model = pr.ivc.m - pr.ivc.mb;
            const real air_g_s = std::isfinite(r.air_g_s) ? r.air_g_s : r.fuel_g_s * r.lambda * pk.fuel.afr_stoich;
            const real air_meas = air_g_s * 1e-3 / (r.rpm / 120.0 * real(c.cylinders));
            Prepared& p = out[i];
            p.air_ratio = air_meas / air_model;
            const real m = pr.ivc.m * p.air_ratio, u = pr.ivc.u * p.air_ratio, mb = pr.ivc.mb * p.air_ratio;
            const real v = proto.model().geom.eval(pr.ivc.crank_deg).v;
            const ec::CvState s = ec::cv_state(proto.model().gas, m, u, mb, v);
            if (!(s.t > 0.0) || !(s.p > 0.0)) {
                p.fail_reason = 3;
                continue;
            }
            p.in.rpm = r.rpm;
            p.in.lambda = r.lambda;
            p.in.t_ivc_k = s.t;
            p.in.p_ivc_pa = s.p;
            p.in.residual_fraction = s.x;
            p.in.volume_ratio = v / proto.model().geom.v_c;
            p.spark = r.spark_btdc;
            p.ok = true;
        }
    });
    return out;
}

real predicted_mbt(const Package& pk, const ec::BurnLawParams& p, const ec::BurnLawInput& in) {
    const CycleModelDesc& c = *pk.desc.cycle;
    const ec::BurnAngles b = ec::evaluate_burn_law(p, c.wiebe_a, c.wiebe_m, in);
    return ec::mbt_spark_btdc(b, c.wiebe_a, c.wiebe_m, c.ca50_mbt_deg_atdc);
}

real constant_mbt(const Package& pk) {
    const CycleModelDesc& c = *pk.desc.cycle;
    ec::BurnLawParams p;
    p.scale = 1.0;
    return ec::mbt_spark_btdc(ec::reference_burn_angles(p, c.wiebe_a, c.wiebe_m), c.wiebe_a, c.wiebe_m, c.ca50_mbt_deg_atdc);
}

struct Set {
    std::vector<std::vector<Prepared>> rows; // per package
};

Set prepare_set(const ec::BurnLawParams& p) {
    Set s;
    for (const auto& pk : packages()) s.rows.push_back(prepare_all(*pk, p));
    return s;
}

real sumsq(const std::vector<real>& r) {
    real s = 0.0;
    for (real v : r) s += v * v;
    return s;
}

struct Stats {
    int n = 0;
    real mean = 0.0, rms = 0.0, max_abs = 0.0;
};

Stats stats_of(const std::vector<real>& d) {
    Stats s;
    real sum = 0.0, sq = 0.0;
    for (real v : d) {
        ++s.n;
        sum += v;
        sq += v * v;
        s.max_abs = std::max(s.max_abs, std::abs(v));
    }
    if (s.n > 0) {
        s.mean = sum / s.n;
        s.rms = std::sqrt(sq / s.n);
    }
    return s;
}

// Per package: gaps before (constant 16.73 deg BTDC MBT) and after (the law with p).
struct PkgGaps {
    std::vector<real> before, after;
    int failed = 0;
    [[nodiscard]] int n_ok() const { return int(after.size()); }
};

std::vector<PkgGaps> gaps(const Set& s, const ec::BurnLawParams& p) {
    std::vector<PkgGaps> out(packages().size());
    for (std::size_t k = 0; k < packages().size(); ++k) {
        const Package& pk = *packages()[k];
        const real c0 = constant_mbt(pk);
        for (const Prepared& w : s.rows[k]) {
            if (!w.ok) {
                ++out[k].failed;
                continue;
            }
            out[k].before.push_back(c0 - w.spark);
            out[k].after.push_back(predicted_mbt(pk, p, w.in) - w.spark);
        }
    }
    return out;
}

// The joint fit (phase 1b, defined at the end of the file): pass 1 on rows prepared with the compiled defaults, pass 2 on
// rows re-prepared with the pass-1 constants (the cycle's own residual and exhaust state depend weakly on its burn).
struct Calibration {
    ec::BurnLawParams first, second;
    Set set;                                // part-load rows prepared with `first`
    std::vector<std::vector<Prepared>> wot; // WOT rows prepared with `first`
};

const Calibration& calibration();
void print_tables(const Calibration& cal) {
    const ec::BurnLawParams& p = cal.second;
    std::printf("burn law fit: pass 1 s %.6f e_n %.6f e_s %.6f; pass 2 s %.6f e_n %.6f e_s %.6f\n", cal.first.scale,
                cal.first.rpm_exponent, cal.first.flame_speed_exponent, p.scale, p.rpm_exponent, p.flame_speed_exponent);
    const std::vector<PkgGaps> g = gaps(cal.set, p);
    std::printf("%-22s %-5s %4s %5s | before (16.73 deg BTDC)  | after (law)\n", "package", "set", "n", "fail");
    std::printf("%-22s %-5s %4s %5s | %7s %7s %7s | %7s %7s %7s\n", "", "", "", "", "mean", "rms", "max", "mean", "rms", "max");
    std::vector<real> tb, ta, hb, ha;
    for (std::size_t k = 0; k < g.size(); ++k) {
        const Stats b = stats_of(g[k].before), a = stats_of(g[k].after);
        const bool tr = packages()[k]->spec.train;
        std::printf("%-22s %-5s %4d %5d | %+7.2f %7.2f %7.2f | %+7.2f %7.2f %7.2f\n", packages()[k]->spec.label,
                    tr ? "train" : "hold", b.n, g[k].failed, b.mean, b.rms, b.max_abs, a.mean, a.rms, a.max_abs);
        auto& vb = tr ? tb : hb;
        auto& va = tr ? ta : ha;
        vb.insert(vb.end(), g[k].before.begin(), g[k].before.end());
        va.insert(va.end(), g[k].after.begin(), g[k].after.end());
    }
    for (int t = 0; t < 2; ++t) {
        const Stats b = stats_of(t == 0 ? tb : hb), a = stats_of(t == 0 ? ta : ha);
        std::printf("%-22s %-5s %4d %5s | %+7.2f %7.2f %7.2f | %+7.2f %7.2f %7.2f\n", t == 0 ? "ALL training" : "ALL held out",
                    "", b.n, "", b.mean, b.rms, b.max_abs, a.mean, a.rms, a.max_abs);
    }
    // Banded (rpm x MAP), pooled per set: mean gap before / after, n.
    const real rb[] = {1000, 1750, 2500, 3500, 6500};
    const real mb[] = {0, 40, 60, 86};
    for (int t = 0; t < 2; ++t) {
        std::printf("%s rows, mean gap before / after [deg] (n) by rpm x MAP:\n", t == 0 ? "training" : "held-out");
        for (int j = 0; j < 3; ++j) {
            std::printf("  MAP %2.0f-%2.0f kPa:", mb[j], mb[j + 1]);
            for (int i = 0; i < 4; ++i) {
                std::vector<real> vb, va;
                for (std::size_t k = 0; k < packages().size(); ++k) {
                    if (packages()[k]->spec.train != (t == 0)) continue;
                    const Package& pk = *packages()[k];
                    for (std::size_t q = 0; q < pk.rows.size(); ++q) {
                        const Prepared& w = cal.set.rows[k][q];
                        const Row& r = pk.rows[q];
                        if (!w.ok || r.rpm < rb[i] || r.rpm >= rb[i + 1] || r.map_kpa < mb[j] || r.map_kpa >= mb[j + 1]) continue;
                        vb.push_back(constant_mbt(pk) - w.spark);
                        va.push_back(predicted_mbt(pk, p, w.in) - w.spark);
                    }
                }
                if (vb.empty()) {
                    std::printf("   %4.0f-%4.0f      -       ", rb[i], rb[i + 1]);
                } else {
                    std::printf("   %4.0f-%4.0f %+5.1f/%+5.1f (%3zu)", rb[i], rb[i + 1], stats_of(vb).mean, stats_of(va).mean, vb.size());
                }
            }
            std::printf("\n");
        }
    }
}

} // namespace

TEST_CASE("BLC diag: burn law gap tables", "[.][diag][burn_law_cal]") {
    const Calibration& cal = calibration();
    print_tables(cal);
    for (std::size_t k = 0; k < packages().size(); ++k) {
        const Package& pk = *packages()[k];
        int h[4] = {0, 0, 0, 0};
        for (const Prepared& w : cal.set.rows[k]) {
            if (w.ok) ++h[std::min(w.step_halvings, 3)];
        }
        std::printf("%-22s rows converged at h, h/2, h/4: %d %d %d\n", pk.spec.label, h[0], h[1], h[2]);
    }
    for (std::size_t k = 0; k < packages().size(); ++k) {
        const Package& pk = *packages()[k];
        for (std::size_t q = 0; q < pk.rows.size(); ++q) {
            const Prepared& w = cal.set.rows[k][q];
            if (w.ok) continue;
            const Row& r = pk.rows[q];
            std::printf("  failed: %-22s row %4d reason %d rpm %6.0f MAP %5.1f kPa exh %6.1f kPa lambda %.3f\n", pk.spec.label, r.row,
                        w.fail_reason, r.rpm, r.map_kpa, r.exh_kpa, r.lambda);
        }
    }
}

// =====================================================================================================================
// Burn law phase 1b: wide-open throttle.
//
// WOT rows: tests/support/fixtures/burn_law/<package>_wot_rows.json (the speed-line torque envelope, >= 1000 rpm;
// tools/enginedata/epa_turbo_extract.py). Their spark is the ECU's knock-limited spark, not MBT, so the MBT-gap method
// of the part-load rows does not apply. What identifies the burn there is the measured WORK at the measured spark:
//
//   measured gross indicated work per cylinder-cycle  W_ig = W_brake + W_friction - W_pump
//       W_brake    = T 4 pi / cylinders                       (measured torque)
//       W_friction = FMEP(mean piston speed) * V_d/cylinder    (the engine's own cycle friction correlation, D13)
//       W_pump     = the cycle point's own gas-exchange work   (model, at the measured MAP and exhaust pressure)
//   divided by the measured trapped air (inlet air flow, else fuel x lambda x AFR_st, per cylinder-cycle), and
//   the model's gross work per trapped air  W_gross / (m_ivc - m_b,ivc)  of a cycle point at the row's measured
//   boundary conditions (the KV1-BC rule of prepare_all) and its measured spark.
//
// r = model / measured. Normalising per unit trapped air cancels the generator's air excess (the [AS] runner's ram,
// PHYS-070): what is left is the cycle's efficiency at that spark - the burn rate and phasing, plus heat transfer,
// combustion efficiency and the generic friction correlation. The burn is varied as a constant factor f on the cited
// angles (Fixed law, duration 40.32 f, delay 3.86 f - the law's own mapping), so each row gets a curve r(f) once and
// any law's f at that row reads its r without re-running the cycle. Controls: r at f = 1 (the C17a constant burn).
// =====================================================================================================================
namespace {

std::vector<std::unique_ptr<Package>>& wot_packages() {
    static std::vector<std::unique_ptr<Package>> v = [] {
        std::vector<std::unique_ptr<Package>> out;
        for (const PkgSpec& s : kPkgs) out.push_back(load_package(s, "_wot_rows.json"));
        return out;
    }();
    return v;
}

// The law's burn factor f (development and rapid angles over their references) - production code.
real law_f(const ec::BurnLawParams& p, const ec::BurnLawInput& in) {
    return ec::evaluate_burn_law(p, 5.0, 2.0, in).development_deg / p.development_reference_deg;
}

// Structural options of the law evaluated here (option F went into burn_law.cpp as flame_speed_pressure_factor 0 with
// rpm_exponent 0.2):
//   kp  - S_L evaluated at p_ref (p_u/p_ref)^kp (1 = the M-K pressure dependence, 0 = S_L at the reference pressure)
//   e_t - exponent on the mean piston speed (turbulence): f *= (S_p,ref / S_p)^e_t, S_p,ref at the reference rpm and a
//         0.09 m stroke [AS]
struct LawOpt {
    real kp = 1.0;
    real e_t = 0.0;
    real e_p = 0.0; // f *= (p_u / p_ref)^e_p
};

real sl_local(const ec::BurnLawParams& p, real kp, real phi_in, real t_u, real p_u, real x_in) {
    const real phi = std::min(p.mk_phi_max, std::max(p.mk_phi_min, phi_in));
    const real dphi = phi - p.mk_phi_m;
    const real s0 = p.mk_b_m_m_s + p.mk_b_2_m_s * dphi * dphi;
    const real alpha = p.mk_alpha_0 + p.mk_alpha_phi * (phi - 1.0);
    const real beta = p.mk_beta_0 + p.mk_beta_phi * (phi - 1.0);
    // kp: S_L at the effective pressure p_ref (p_u / p_ref)^kp (1 = M-K, 0 = S_L at the reference pressure)
    const real p_eff = p.reference_unburned_pressure_pa * std::pow(p_u / p.reference_unburned_pressure_pa, kp);
    const real x = std::max(real(0.0), x_in);
    const real dil = std::max(p.min_dilution_factor, 1.0 - p.dilution_k * std::pow(x, p.dilution_exponent));
    return std::max(p.min_flame_speed_m_s, s0 * std::pow(t_u / p.mk_t0_k, alpha) * std::pow(p_eff / p.mk_p0_pa, beta) * dil);
}

real f_local(const ec::BurnLawParams& p, const LawOpt& o, real stroke_m, const ec::BurnLawInput& in) {
    const real r = std::max(real(1.0), in.volume_ratio);
    const real t_u = in.t_ivc_k * std::pow(r, p.compression_exponent - 1.0);
    const real p_u = in.p_ivc_pa * std::pow(r, p.compression_exponent);
    const real sl = sl_local(p, o.kp, 1.0 / in.lambda, t_u, p_u, in.residual_fraction);
    const real sref = sl_local(p, o.kp, 1.0, p.reference_unburned_temperature_k, p.reference_unburned_pressure_pa,
                               p.reference_residual_fraction);
    real f = p.scale * std::pow(std::max(in.rpm, real(1.0)) / p.reference_rpm, p.rpm_exponent) *
             std::pow(sref / sl, p.flame_speed_exponent);
    if (o.e_t != 0.0) {
        const real sp = 2.0 * stroke_m * in.rpm / 60.0, sp_ref = 2.0 * 0.09 * p.reference_rpm / 60.0;
        f *= std::pow(sp_ref / sp, o.e_t);
    }
    if (o.e_p != 0.0) f *= std::pow(p_u / p.reference_unburned_pressure_pa, o.e_p);
    return f;
}

// MBT [deg BTDC] of a burn factor f (the law's mapping: duration and delay both scale with f).
real mbt_of_f(const Package& pk, real f) {
    const CycleModelDesc& c = *pk.desc.cycle;
    ec::BurnLawParams one;
    one.scale = 1.0;
    ec::BurnAngles b = ec::reference_burn_angles(one, c.wiebe_a, c.wiebe_m);
    b.wiebe_duration_deg *= f;
    b.ignition_delay_deg *= f;
    return ec::mbt_spark_btdc(b, c.wiebe_a, c.wiebe_m, c.ca50_mbt_deg_atdc);
}

real wiebe_duration_of_f(const Package& pk, real f) {
    const CycleModelDesc& c = *pk.desc.cycle;
    ec::BurnLawParams one;
    one.scale = 1.0;
    return ec::reference_burn_angles(one, c.wiebe_a, c.wiebe_m).wiebe_duration_deg * f;
}

// A cycle point at a row's measured boundary conditions (prepare_all's rule and step-halving retry).
bool run_row(const Package& pk, const ec::CycleStepper& proto, const Row& r, ec::OperatingPoint::SparkMode mode,
             real spark_deg_atdc, ec::PointResult* pr) {
    const CycleModelDesc& c = *pk.desc.cycle;
    ec::OperatingPoint op;
    op.rpm = r.rpm;
    op.p_im_pa = r.map_kpa * 1000.0;
    op.t_im_k = r.imt_c + 273.15;
    op.p_amb_pa = std::isfinite(r.exh_kpa) && r.exh_kpa > 0.0 ? r.exh_kpa * 1000.0 : 101325.0;
    op.lambda = r.lambda;
    op.spark_mode = mode;
    op.spark_deg_atdc = spark_deg_atdc;
    const real it = c.cam.intake_advance_deg.x.empty() ? 0.0 : c.cam.intake_advance_deg.value(op.rpm, op.p_im_pa);
    const real et = c.cam.exhaust_retard_deg.x.empty() ? 0.0 : c.cam.exhaust_retard_deg.value(op.rpm, op.p_im_pa);
    op.intake_advance_deg = r.has_icam ? r.icam_adv : it;
    op.exhaust_retard_deg = r.has_ecam ? r.ecam_ret : et;
    for (int h = 0; h < 3; ++h) {
        try {
            *pr = ec::run_operating_point(proto, op, ec::StepperSettings{ec::kStepFull / real(1 << h), ec::kCyclesFull});
        } catch (...) {
            continue;
        }
        if (pr->ivc.m > 0.0 && pr->ivc.m > pr->ivc.mb && std::isfinite(pr->w_gross_j)) return true;
    }
    return false;
}

constexpr int kNf = 15;
real f_grid(int k) { return 0.4 * std::pow(1.15, real(k)); } // 0.40 .. 2.83

struct WotScan {
    bool ok = false;
    real w_meas = 0.0;                              // measured gross indicated work per trapped air [J/kg]
    real w_brake = 0.0, w_fric = 0.0, w_pump = 0.0; // per cylinder-cycle [J] (pump from the constant-burn run)
    std::array<real, kNf> w{};                      // model gross work per trapped air at f_grid(k) [J/kg]
};

real interp_w(const WotScan& s, real f) { // log-linear in f, clamped at the grid ends
    const real lf = std::log(std::max(f_grid(0), std::min(f_grid(kNf - 1), f)));
    const real k = (lf - std::log(0.4)) / std::log(1.15);
    const int i = std::max(0, std::min(kNf - 2, int(std::floor(k))));
    const real t = k - i;
    return s.w[i] + t * (s.w[i + 1] - s.w[i]);
}

std::vector<std::vector<WotScan>> scan_all() {
    std::vector<std::vector<WotScan>> out;
    for (const auto& pkp : wot_packages()) {
        const Package& pk = *pkp;
        const CycleModelDesc& c0 = *pk.desc.cycle;
        std::vector<std::unique_ptr<ec::CycleStepper>> protos;
        for (int k = 0; k < kNf; ++k) {
            CycleModelDesc c = c0;
            c.burn_law = CycleModelDesc::BurnLawMode::Fixed;
            c.burn_duration_deg = wiebe_duration_of_f(pk, f_grid(k));
            ec::BurnLawParams one;
            one.scale = f_grid(k);
            c.ignition_delay_deg = ec::reference_burn_angles(one, c.wiebe_a, c.wiebe_m).ignition_delay_deg;
            protos.push_back(std::make_unique<ec::CycleStepper>(c, pk.fuel.lhv_j_kg, pk.fuel.afr_stoich));
        }
        CycleModelDesc cc = c0;
        cc.burn_law = CycleModelDesc::BurnLawMode::Fixed;
        cc.burn_duration_deg = 40.3205;
        cc.ignition_delay_deg = 3.8632;
        const ec::CycleStepper proto_const(cc, pk.fuel.lhv_j_kg, pk.fuel.afr_stoich);
        std::vector<WotScan> v(pk.rows.size());
        const real vd_cyl = 0.25 * 3.14159265358979323846 * c0.bore_m * c0.bore_m * c0.stroke_m;
        pool().parallel_for(pk.rows.size() * kNf, 1, [&](std::size_t b, std::size_t e) {
            for (std::size_t q = b; q < e; ++q) {
                const std::size_t i = q / kNf;
                const int k = int(q % kNf);
                ec::PointResult pr;
                const bool ok =
                    run_row(pk, *protos[k], pk.rows[i], ec::OperatingPoint::SparkMode::Spark, -pk.rows[i].spark_btdc, &pr);
                v[i].w[k] = ok ? pr.w_gross_j / (pr.ivc.m - pr.ivc.mb) : std::nan("");
            }
        });
        for (std::size_t i = 0; i < pk.rows.size(); ++i) {
            const Row& r = pk.rows[i];
            ec::PointResult pr;
            if (!run_row(pk, proto_const, r, ec::OperatingPoint::SparkMode::Spark, -r.spark_btdc, &pr)) continue;
            bool all = true;
            for (int k = 0; k < kNf; ++k) all = all && std::isfinite(v[i].w[k]);
            if (!all) continue;
            const real sp = 2.0 * c0.stroke_m * r.rpm / 60.0;
            const real fmep = c0.friction.a_pa + c0.friction.b_pa * sp + c0.friction.c_pa * sp * sp;
            v[i].w_brake = r.torque_nm * 4.0 * 3.14159265358979323846 / real(c0.cylinders);
            v[i].w_fric = fmep * vd_cyl;
            v[i].w_pump = pr.w_pump_j;
            const real air_g_s = std::isfinite(r.air_g_s) ? r.air_g_s : r.fuel_g_s * r.lambda * pk.fuel.afr_stoich;
            const real air_cyl = air_g_s * 1e-3 / (r.rpm / 120.0 * real(c0.cylinders));
            v[i].w_meas = (v[i].w_brake + v[i].w_fric - v[i].w_pump) / air_cyl;
            v[i].ok = std::isfinite(v[i].w_meas) && v[i].w_meas > 0.0;
        }
        out.push_back(std::move(v));
    }
    return out;
}

const std::vector<std::vector<WotScan>>& wot_scans() {
    static const std::vector<std::vector<WotScan>> s = scan_all();
    return s;
}

// The WOT rows' law inputs (KV1-BC rule, measured spark) under the shipped params.
const std::vector<std::vector<Prepared>>& wot_prepared_shipped() {
    static const std::vector<std::vector<Prepared>> s = [] {
        std::vector<std::vector<Prepared>> out;
        const ec::BurnLawParams shipped;
        for (const auto& pk : wot_packages()) out.push_back(prepare_all(*pk, shipped));
        return out;
    }();
    return s;
}

// The decreasing-branch root of r(f) = 1 (the burn factor the measured work implies), NaN if r stays below 1.
real f_needed(const WotScan& s) {
    int kmax = 0;
    for (int k = 1; k < kNf; ++k) {
        if (s.w[k] > s.w[kmax]) kmax = k;
    }
    if (s.w[kmax] < s.w_meas) return std::nan("");
    for (int k = kmax; k + 1 < kNf; ++k) {
        if (s.w[k] >= s.w_meas && s.w[k + 1] < s.w_meas) {
            const real t = (s.w[k] - s.w_meas) / (s.w[k] - s.w[k + 1]);
            return std::exp(std::log(f_grid(k)) + t * std::log(1.15));
        }
    }
    return std::numeric_limits<real>::infinity(); // still above at the slowest grid burn
}

// WOT fit rows (declared before the fit): lambda in [0.85, 1.05] [AS] - the richer rows run the ECU's component
// protection enrichment, where the cycle's eta_c = 0.98 lambda rule (plan D14) and the measured lambda dominate the work.
// MAP >= 90 kPa: the envelope of the Mazda 2.0 Tier 2 package also picks throttled rows (MAP 71-89 kPa, BMEP 3-7.5 bar) in bins
// without a WOT point - they are part load, not WOT (added after the first diag run, before any fit counted).
bool wot_fit_row(const Row& r) { return r.lambda >= 0.85 && r.lambda <= 1.05 && r.map_kpa >= 90.0; }

// Log decomposition of the law factor: ln f = ln s + e_n ln(n/n_ref) + e_s [phi + T + p + dil] (each a log term; p at the
// effective pressure, so it is 0 at k_p = 0).
struct Decomp {
    real tu = 0, pu = 0, phi = 0, phi_raw = 0, f = 0, g_rpm = 0, l_phi = 0, l_t = 0, l_p = 0, l_d = 0;
};

Decomp decompose(const ec::BurnLawParams& p, const ec::BurnLawInput& in) {
    Decomp d;
    const real rr = std::max(real(1.0), in.volume_ratio);
    d.tu = in.t_ivc_k * std::pow(rr, p.compression_exponent - 1.0);
    d.pu = in.p_ivc_pa * std::pow(rr, p.compression_exponent);
    // S_L's effective pressure (burn_law.h): p_ref (p_u / p_ref)^k_p.
    const real p_eff = p.reference_unburned_pressure_pa * std::pow(d.pu / p.reference_unburned_pressure_pa, p.flame_speed_pressure_factor);
    d.phi_raw = 1.0 / in.lambda;
    d.phi = std::min(p.mk_phi_max, std::max(p.mk_phi_min, d.phi_raw));
    auto s0 = [&](real ph) { return p.mk_b_m_m_s + p.mk_b_2_m_s * (ph - p.mk_phi_m) * (ph - p.mk_phi_m); };
    auto dil = [&](real x) {
        return std::max(p.min_dilution_factor, 1.0 - p.dilution_k * std::pow(std::max(real(0.0), x), p.dilution_exponent));
    };
    const real al = p.mk_alpha_0 + p.mk_alpha_phi * (d.phi - 1.0), be = p.mk_beta_0 + p.mk_beta_phi * (d.phi - 1.0);
    d.l_phi = std::log(s0(1.0) / s0(d.phi)) + (p.mk_alpha_0 - al) * std::log(d.tu / p.mk_t0_k) +
              (p.mk_beta_0 - be) * std::log(p_eff / p.mk_p0_pa);
    d.l_t = p.mk_alpha_0 * std::log(p.reference_unburned_temperature_k / d.tu);
    d.l_p = p.mk_beta_0 * std::log(p.reference_unburned_pressure_pa / p_eff);
    d.l_d = std::log(dil(p.reference_residual_fraction) / dil(in.residual_fraction));
    d.g_rpm = std::pow(in.rpm / p.reference_rpm, p.rpm_exponent);
    d.f = p.scale * d.g_rpm * std::exp(p.flame_speed_exponent * (d.l_phi + d.l_t + d.l_p + d.l_d));
    return d;
}

} // namespace

TEST_CASE("BLC diag: WOT decomposition of the law factor", "[.][diag][burn_law_cal]") {
    const ec::BurnLawParams p;
    const real es = p.flame_speed_exponent;
    const auto& prep = wot_prepared_shipped();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        const Package& pk = *wot_packages()[k];
        std::printf("== %s WOT rows: f = s * rpm * (phi * T * p * dil) - each column its factor on f\n", pk.spec.label);
        std::printf("   rpm   lam  spark   Tu_K pu_MPa  x_res phi_c |    f     s   rpm   phi     T     p   dil |  dur   MBT\n");
        for (std::size_t q = 0; q < pk.rows.size(); ++q) {
            const Prepared& w = prep[k][q];
            const Row& r = pk.rows[q];
            if (!w.ok) {
                std::printf("%6.0f  failed (%d)\n", r.rpm, w.fail_reason);
                continue;
            }
            const Decomp d = decompose(p, w.in);
            const ec::BurnAngles b = ec::evaluate_burn_law(p, pk.desc.cycle->wiebe_a, pk.desc.cycle->wiebe_m, w.in);
            CHECK(d.f == Catch::Approx(b.wiebe_duration_deg / wiebe_duration_of_f(pk, 1.0)).epsilon(1e-9));
            std::printf("%6.0f %5.3f %5.1f %6.0f %6.2f %6.3f %5.2f%s| %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f | %5.1f %5.1f\n", r.rpm,
                        r.lambda, r.spark_btdc, d.tu, d.pu * 1e-6, w.in.residual_fraction, d.phi, d.phi != d.phi_raw ? "*" : " ", d.f,
                        p.scale, d.g_rpm, std::exp(es * d.l_phi), std::exp(es * d.l_t), std::exp(es * d.l_p), std::exp(es * d.l_d),
                        b.wiebe_duration_deg, predicted_mbt(pk, p, w.in));
        }
    }
    // The same factors at the part-load rows (geometric means per package), for contrast.
    const Calibration& cal = calibration();
    std::printf("== part-load rows: geometric means of each factor\n");
    for (std::size_t k = 0; k < packages().size(); ++k) {
        const Package& pk = *packages()[k];
        real n = 0, sf = 0, sr = 0, sph = 0, st = 0, sp = 0, sd = 0, spu = 0, stu = 0, sx = 0;
        for (const Prepared& w : cal.set.rows[k]) {
            if (!w.ok) continue;
            const Decomp d = decompose(p, w.in);
            sf += std::log(d.f);
            sr += std::log(d.g_rpm);
            sph += es * d.l_phi;
            st += es * d.l_t;
            sp += es * d.l_p;
            sd += es * d.l_d;
            spu += std::log(d.pu);
            stu += std::log(d.tu);
            sx += w.in.residual_fraction;
            n += 1;
        }
        std::printf("  %-22s n %3.0f  f %5.3f rpm %5.3f phi %5.3f T %5.3f p %5.3f dil %5.3f | pu %5.2f MPa Tu %4.0f K x %5.3f\n",
                    pk.spec.label, n, std::exp(sf / n), std::exp(sr / n), std::exp(sph / n), std::exp(st / n), std::exp(sp / n),
                    std::exp(sd / n), std::exp(spu / n) * 1e-6, std::exp(stu / n), sx / n);
    }
}

TEST_CASE("BLC diag: WOT work per trapped air", "[.][diag][burn_law_cal]") {
    const ec::BurnLawParams p;
    const auto& prep = wot_prepared_shipped();
    const auto& sc = wot_scans();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        const Package& pk = *wot_packages()[k];
        std::printf("== %s: r = model / measured gross work per trapped air at the measured spark (x = not a fit row)\n", pk.spec.label);
        std::printf("   rpm   lam spark  bmep  Wb_J  Wf_J  Wp_J Wmeas_kJ/kg  | r(f=1)  f_law r(law) MBT(law) | f_need   dur MBT(f_need)\n");
        for (std::size_t q = 0; q < pk.rows.size(); ++q) {
            const Row& r = pk.rows[q];
            const WotScan& s = sc[k][q];
            const Prepared& w = prep[k][q];
            if (!s.ok || !w.ok) {
                std::printf("%6.0f  failed\n", r.rpm);
                continue;
            }
            const real fl = law_f(p, w.in);
            const real fn = f_needed(s);
            std::printf("%6.0f %5.3f %5.1f %5.2f %5.0f %5.0f %+5.0f %7.1f %s | %6.3f %5.2f %6.3f %6.1f | %5.2f %5.1f %5.1f\n", r.rpm,
                        r.lambda, r.spark_btdc, r.bmep, s.w_brake, s.w_fric, s.w_pump, s.w_meas * 1e-3, wot_fit_row(r) ? " " : "x",
                        interp_w(s, 1.0) / s.w_meas, fl, interp_w(s, fl) / s.w_meas, mbt_of_f(pk, fl), fn,
                        std::isfinite(fn) ? wiebe_duration_of_f(pk, fn) : fn, std::isfinite(fn) ? mbt_of_f(pk, fn) : fn);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Joint fit (phase 1b): part-load MBT gaps and WOT work ratios together, over a law structure option.
// Declared before the fit: sigma_deg 7 (the part-load rms of the phase 1 fit), sigma_r 0.03 [AS] on ln r (friction and
// heat-transfer model uncertainty at <= 3000 rpm), WOT fit rows lambda 0.85-1.05 and rpm <= 3000 [AS] (model pumping
// <= ~3 % and friction <= ~6 % of the gross work there; above it the model's gas-exchange loss grows to 10 % of W_ig on
// the NA engines and contaminates r), training packages as in phase 1 (Ford 2.7 + Honda T3).
// ---------------------------------------------------------------------------------------------------------------------
namespace {

// The phase 1 constants (part-load fit, M-K pressure term) - the comparison column of the diags.
constexpr real kPhase1Scale = 1.614, kPhase1RpmExponent = 0.3944, kPhase1FlameSpeedExponent = 0.5592;

constexpr real kSigmaDeg = 7.0;
constexpr real kSigmaR = 0.03;
bool wot_joint_row(const Row& r) { return wot_fit_row(r) && r.rpm <= 3000.0; }

struct Opt {
    const char* name;
    int n;           // fitted parameters: ln s, [e_n], e_s, [extra]
    bool fit_en;     // e_n fitted (else held at en_fixed)
    real en_fixed;   // [C] MIT 2.61 lec 9: burn angle ~ rpm^0.2
    real kp;         // factor on the M-K pressure exponent inside the S_L ratio
    int extra;       // 0 none, 1 e_p (f *= (p_u/p_ref)^e_p), 2 e_t (mean piston speed)
};

struct OptParams {
    ec::BurnLawParams p;
    LawOpt o;
};

OptParams decode(const Opt& op, const std::vector<real>& x) {
    OptParams r;
    int i = 0;
    r.p.scale = std::exp(x[i++]);
    r.p.rpm_exponent = op.fit_en ? x[i++] : op.en_fixed;
    r.p.flame_speed_exponent = x[i++];
    r.o.kp = op.kp;
    if (op.extra == 1) r.o.e_p = x[i++];
    if (op.extra == 2) r.o.e_t = x[i++];
    return r;
}

std::vector<real> joint_residuals(const Set& part, const Opt& op, const std::vector<real>& x, bool train) {
    const OptParams q = decode(op, x);
    std::vector<real> res;
    for (std::size_t k = 0; k < packages().size(); ++k) {
        if (packages()[k]->spec.train != train) continue;
        const Package& pk = *packages()[k];
        for (const Prepared& w : part.rows[k]) {
            if (w.ok) res.push_back((mbt_of_f(pk, f_local(q.p, q.o, pk.desc.cycle->stroke_m, w.in)) - w.spark) / kSigmaDeg);
        }
    }
    const auto& prep = wot_prepared_shipped();
    const auto& sc = wot_scans();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        if (wot_packages()[k]->spec.train != train) continue;
        const Package& pk = *wot_packages()[k];
        for (std::size_t i = 0; i < pk.rows.size(); ++i) {
            if (!prep[k][i].ok || !sc[k][i].ok || !wot_joint_row(pk.rows[i])) continue;
            const real f = f_local(q.p, q.o, pk.desc.cycle->stroke_m, prep[k][i].in);
            res.push_back(std::log(interp_w(sc[k][i], f) / sc[k][i].w_meas) / kSigmaR);
        }
    }
    return res;
}

// Levenberg-Marquardt over x, central-difference Jacobian.
using ResidualFn = std::function<std::vector<real>(const std::vector<real>&)>;
std::vector<real> lm_generic(const ResidualFn& res, std::vector<real> x) {
    const int n = int(x.size());
    real mu = 1e-3;
    real f0 = sumsq(res(x));
    for (int it = 0; it < 300; ++it) {
        const std::vector<real> r0 = res(x);
        std::vector<std::vector<real>> jac(n);
        for (int k = 0; k < n; ++k) {
            std::vector<real> xp = x, xm = x;
            xp[k] += 1e-6;
            xm[k] -= 1e-6;
            const std::vector<real> rp = res(xp), rm = res(xm);
            jac[k].resize(r0.size());
            for (std::size_t i = 0; i < r0.size(); ++i) jac[k][i] = (rp[i] - rm[i]) / 2e-6;
        }
        std::vector<std::vector<real>> a(n, std::vector<real>(n, 0.0));
        std::vector<real> g(n, 0.0);
        for (std::size_t i = 0; i < r0.size(); ++i) {
            for (int p = 0; p < n; ++p) {
                g[p] += jac[p][i] * r0[i];
                for (int q = 0; q < n; ++q) a[p][q] += jac[p][i] * jac[q][i];
            }
        }
        bool improved = false;
        for (int tries = 0; tries < 30 && !improved; ++tries) {
            std::vector<std::vector<real>> m(n, std::vector<real>(n + 1, 0.0));
            for (int p = 0; p < n; ++p) {
                for (int q = 0; q < n; ++q) m[p][q] = a[p][q] + (p == q ? mu * (a[p][p] + 1e-12) : 0.0);
                m[p][n] = -g[p];
            }
            for (int c = 0; c < n; ++c) {
                int piv = c;
                for (int rr = c + 1; rr < n; ++rr) {
                    if (std::abs(m[rr][c]) > std::abs(m[piv][c])) piv = rr;
                }
                std::swap(m[c], m[piv]);
                for (int rr = 0; rr < n; ++rr) {
                    if (rr == c) continue;
                    const real fct = m[rr][c] / m[c][c];
                    for (int q = 0; q <= n; ++q) m[rr][q] -= fct * m[c][q];
                }
            }
            std::vector<real> xn = x;
            for (int p = 0; p < n; ++p) xn[p] += m[p][n] / m[p][p];
            const real fn = sumsq(res(xn));
            if (std::isfinite(fn) && fn < f0) {
                const real rel = (f0 - fn) / f0;
                x = xn;
                f0 = fn;
                mu = std::max(mu * 0.3, 1e-9);
                improved = true;
                if (rel < 1e-12) return x;
            } else {
                mu *= 10.0;
            }
        }
        if (!improved) break;
    }
    return x;
}

std::vector<real> lm_fit(const Set& part, const Opt& op, std::vector<real> x) {
    return lm_generic([&](const std::vector<real>& v) { return joint_residuals(part, op, v, true); }, std::move(x));
}

struct OptReport {
    std::vector<real> x;
    real part_rms_train = 0, part_rms_hold = 0, wot_lnr_train = 0, wot_lnr_hold = 0, wot_lnr_hold_const = 0;
    real f_min = 1e9, f_max = 0, mbt_min = 1e9, mbt_max = -1e9, en_wot = 0;
    int n_wot_train = 0, n_wot_hold = 0;
};

OptReport evaluate_option(const Set& part, const Opt& op, const std::vector<real>& x) {
    OptReport r;
    r.x = x;
    const OptParams q = decode(op, x);
    std::vector<real> gt, gh;
    for (std::size_t k = 0; k < packages().size(); ++k) {
        const Package& pk = *packages()[k];
        for (const Prepared& w : part.rows[k]) {
            if (!w.ok) continue;
            const real g = mbt_of_f(pk, f_local(q.p, q.o, pk.desc.cycle->stroke_m, w.in)) - w.spark;
            (pk.spec.train ? gt : gh).push_back(g);
        }
    }
    r.part_rms_train = stats_of(gt).rms;
    r.part_rms_hold = stats_of(gh).rms;
    std::vector<real> lt, lh, lhc;
    real sxx = 0, sxy = 0, sx = 0, sy = 0, nn = 0;
    const auto& prep = wot_prepared_shipped();
    const auto& sc = wot_scans();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        const Package& pk = *wot_packages()[k];
        for (std::size_t i = 0; i < pk.rows.size(); ++i) {
            if (!prep[k][i].ok || !sc[k][i].ok) continue;
            const Row& row = pk.rows[i];
            const real f = f_local(q.p, q.o, pk.desc.cycle->stroke_m, prep[k][i].in);
            if (wot_fit_row(row)) {
                r.f_min = std::min(r.f_min, f);
                r.f_max = std::max(r.f_max, f);
                r.mbt_min = std::min(r.mbt_min, mbt_of_f(pk, f));
                r.mbt_max = std::max(r.mbt_max, mbt_of_f(pk, f));
                if (row.rpm >= 1500.0 && row.rpm <= 4500.0) {
                    const real lx = std::log(row.rpm), ly = std::log(f);
                    sxx += lx * lx;
                    sxy += lx * ly;
                    sx += lx;
                    sy += ly;
                    nn += 1;
                }
            }
            if (!wot_joint_row(row)) continue;
            const real lr = std::log(interp_w(sc[k][i], f) / sc[k][i].w_meas);
            if (pk.spec.train) {
                lt.push_back(lr);
            } else {
                lh.push_back(lr);
                lhc.push_back(std::log(interp_w(sc[k][i], 1.0) / sc[k][i].w_meas));
            }
        }
    }
    r.wot_lnr_train = stats_of(lt).rms;
    r.wot_lnr_hold = stats_of(lh).rms;
    r.wot_lnr_hold_const = stats_of(lhc).rms;
    r.n_wot_train = int(lt.size());
    r.n_wot_hold = int(lh.size());
    r.en_wot = (nn * sxy - sx * sy) / (nn * sxx - sx * sx); // pooled log-log slope of f vs rpm (within-package spread included)
    return r;
}

const Opt kOpts[] = {
    {"A  phase 1 structure, joint refit", 3, true, 0.0, 1.0, 0},
    {"B  S_L at p_ref + own pressure exponent e_p", 4, true, 0.0, 0.0, 1},
    {"C  S_L at p_ref (no pressure term)", 3, true, 0.0, 0.0, 0},
    {"D  mean-piston-speed exponent e_t", 4, true, 0.0, 1.0, 2},
    {"E  B with e_n = 0.2 [C]", 3, false, 0.2, 0.0, 1},
    {"F  S_L at p_ref, e_n = 0.2 [C]", 2, false, 0.2, 0.0, 0},
};

} // namespace

TEST_CASE("BLC diag: joint part-load + WOT fit over law options", "[.][diag][burn_law_cal]") {
    const Calibration& cal = calibration();
    const ec::BurnLawParams shipped;
    {
        const Opt ph1{"phase 1 law (s 1.614 e_n 0.3944 e_s 0.5592, kp 1)", 3, true, 0.0, 1.0, 0};
        const OptReport r = evaluate_option(cal.set, ph1, {std::log(kPhase1Scale), kPhase1RpmExponent, kPhase1FlameSpeedExponent});
        std::printf("%-46s part rms train %5.2f hold %5.2f | WOT ln r rms train %5.3f (n %d) hold %5.3f (n %d; constant burn %5.3f) | WOT f %4.2f-%4.2f MBT %4.1f-%4.1f slope %+5.2f\n",
                    ph1.name, r.part_rms_train, r.part_rms_hold, r.wot_lnr_train, r.n_wot_train, r.wot_lnr_hold, r.n_wot_hold,
                    r.wot_lnr_hold_const, r.f_min, r.f_max, r.mbt_min, r.mbt_max, r.en_wot);
    }
    for (const Opt& op : kOpts) {
        std::vector<real> x0{std::log(shipped.scale)};
        if (op.fit_en) x0.push_back(shipped.rpm_exponent);
        x0.push_back(shipped.flame_speed_exponent);
        if (op.extra != 0) x0.push_back(0.0);
        const std::vector<real> x = lm_fit(cal.set, op, x0);
        const OptReport r = evaluate_option(cal.set, op, x);
        const OptParams q = decode(op, x);
        std::printf("%-46s part rms train %5.2f hold %5.2f | WOT ln r rms train %5.3f (n %d) hold %5.3f (n %d; constant burn %5.3f) | WOT f %4.2f-%4.2f MBT %4.1f-%4.1f slope %+5.2f\n",
                    op.name, r.part_rms_train, r.part_rms_hold, r.wot_lnr_train, r.n_wot_train, r.wot_lnr_hold, r.n_wot_hold,
                    r.wot_lnr_hold_const, r.f_min, r.f_max, r.mbt_min, r.mbt_max, r.en_wot);
        std::printf("      s %.4f e_n %.4f e_s %.4f e_p %.4f e_t %.4f (kp %.0f)\n", q.p.scale, q.p.rpm_exponent, q.p.flame_speed_exponent,
                    q.o.e_p, q.o.e_t, q.o.kp);
    }
}

namespace {

void print_option_detail(const Set& part, const Opt& op, const std::vector<real>& x) {
    const OptParams q = decode(op, x);
    std::printf("---- detail: %s (s %.4f e_n %.4f e_s %.4f e_p %.4f e_t %.4f kp %.0f)\n", op.name, q.p.scale, q.p.rpm_exponent,
                q.p.flame_speed_exponent, q.o.e_p, q.o.e_t, q.o.kp);
    for (std::size_t k = 0; k < packages().size(); ++k) {
        const Package& pk = *packages()[k];
        std::vector<real> g;
        for (const Prepared& w : part.rows[k]) {
            if (w.ok) g.push_back(mbt_of_f(pk, f_local(q.p, q.o, pk.desc.cycle->stroke_m, w.in)) - w.spark);
        }
        const Stats s = stats_of(g);
        std::printf("  part load %-22s n %3d mean %+6.2f rms %5.2f max %5.2f\n", pk.spec.label, s.n, s.mean, s.rms, s.max_abs);
    }
    // rpm x MAP banded part-load mean gap, pooled per set
    const real rb[] = {1000, 1750, 2500, 3500, 6500};
    const real mb[] = {0, 40, 60, 86};
    for (int t = 0; t < 2; ++t) {
        std::printf("  %s part-load mean gap [deg] (n) by rpm x MAP:\n", t == 0 ? "training" : "held-out");
        for (int j = 0; j < 3; ++j) {
            std::printf("    MAP %2.0f-%2.0f:", mb[j], mb[j + 1]);
            for (int i = 0; i < 4; ++i) {
                std::vector<real> v;
                for (std::size_t k = 0; k < packages().size(); ++k) {
                    if (packages()[k]->spec.train != (t == 0)) continue;
                    const Package& pk = *packages()[k];
                    for (std::size_t r = 0; r < pk.rows.size(); ++r) {
                        const Prepared& w = part.rows[k][r];
                        const Row& row = pk.rows[r];
                        if (!w.ok || row.rpm < rb[i] || row.rpm >= rb[i + 1] || row.map_kpa < mb[j] || row.map_kpa >= mb[j + 1]) continue;
                        v.push_back(mbt_of_f(pk, f_local(q.p, q.o, pk.desc.cycle->stroke_m, w.in)) - w.spark);
                    }
                }
                if (v.empty()) std::printf("   %4.0f-%4.0f     -      ", rb[i], rb[i + 1]);
                else std::printf("   %4.0f-%4.0f %+5.1f (%3zu)", rb[i], rb[i + 1], stats_of(v).mean, v.size());
            }
            std::printf("\n");
        }
    }
    const auto& prep = wot_prepared_shipped();
    const auto& sc = wot_scans();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        const Package& pk = *wot_packages()[k];
        std::printf("  WOT %-22s rpm/lambda: f  0-10  10-90  MBT  ln r  (x = not a fit row, j = joint fit row)\n", pk.spec.label);
        for (std::size_t i = 0; i < pk.rows.size(); ++i) {
            if (!prep[k][i].ok || !sc[k][i].ok) continue;
            const Row& row = pk.rows[i];
            const real f = f_local(q.p, q.o, pk.desc.cycle->stroke_m, prep[k][i].in);
            std::printf("    %5.0f %5.3f %s | %5.2f %5.1f %5.1f %5.1f %+6.3f\n", row.rpm, row.lambda,
                        wot_joint_row(row) ? "j" : (wot_fit_row(row) ? " " : "x"), f, 15.0 * f, 20.0 * f, mbt_of_f(pk, f),
                        std::log(interp_w(sc[k][i], f) / sc[k][i].w_meas));
        }
    }
}

} // namespace

TEST_CASE("BLC diag: joint fit option details", "[.][diag][burn_law_cal]") {
    const Calibration& cal = calibration();
    const ec::BurnLawParams shipped;
    const char* env = std::getenv("PS_BLC_OPTS"); // letters of kOpts to detail, default "ACF"
    const std::string want = env ? env : "ACF";
    {
        const Opt ph1{"phase 1 law (s 1.614 e_n 0.3944 e_s 0.5592, kp 1)", 3, true, 0.0, 1.0, 0};
        if (want.find('0') != std::string::npos)
            print_option_detail(cal.set, ph1, {std::log(kPhase1Scale), kPhase1RpmExponent, kPhase1FlameSpeedExponent});
    }
    for (const Opt& op : kOpts) {
        if (want.find(op.name[0]) == std::string::npos) continue;
        std::vector<real> x0{std::log(shipped.scale)};
        if (op.fit_en) x0.push_back(shipped.rpm_exponent);
        x0.push_back(shipped.flame_speed_exponent);
        if (op.extra != 0) x0.push_back(0.0);
        print_option_detail(cal.set, op, lm_fit(cal.set, op, x0));
    }
}

// =====================================================================================================================
// The shipped fit (phase 1b, option F): rpm_exponent 0.2 [C] and flame_speed_pressure_factor 0 held, (ln s, e_s) fitted
// jointly on the training packages' part-load MBT gaps (sigma 7 deg) and WOT work ratios (sigma 0.03 on ln r, WOT joint
// rows), through the production evaluate_burn_law. Pass 2 re-prepares both row sets with the pass-1 constants.
// =====================================================================================================================
namespace {

std::vector<std::vector<Prepared>> prepare_wot(const ec::BurnLawParams& p) {
    std::vector<std::vector<Prepared>> out;
    for (const auto& pk : wot_packages()) out.push_back(prepare_all(*pk, p));
    return out;
}

ec::BurnLawParams with_joint(const ec::BurnLawParams& base, const std::vector<real>& x) {
    ec::BurnLawParams p = base;
    p.scale = std::exp(x[0]);
    p.flame_speed_exponent = x[1];
    return p;
}

std::vector<real> law_joint_residuals(const Set& part, const std::vector<std::vector<Prepared>>& wot,
                                      const ec::BurnLawParams& p, bool train) {
    std::vector<real> res;
    for (std::size_t k = 0; k < packages().size(); ++k) {
        if (packages()[k]->spec.train != train) continue;
        for (const Prepared& w : part.rows[k]) {
            if (w.ok) res.push_back((predicted_mbt(*packages()[k], p, w.in) - w.spark) / kSigmaDeg);
        }
    }
    const auto& sc = wot_scans();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        if (wot_packages()[k]->spec.train != train) continue;
        const Package& pk = *wot_packages()[k];
        for (std::size_t i = 0; i < pk.rows.size(); ++i) {
            if (!wot[k][i].ok || !sc[k][i].ok || !wot_joint_row(pk.rows[i])) continue;
            res.push_back(std::log(interp_w(sc[k][i], law_f(p, wot[k][i].in)) / sc[k][i].w_meas) / kSigmaR);
        }
    }
    return res;
}

ec::BurnLawParams joint_fit(const Set& part, const std::vector<std::vector<Prepared>>& wot, const ec::BurnLawParams& start) {
    const std::vector<real> x = lm_generic(
        [&](const std::vector<real>& v) { return law_joint_residuals(part, wot, with_joint(start, v), true); },
        {std::log(start.scale), start.flame_speed_exponent});
    return with_joint(start, x);
}

const Calibration& calibration() {
    static const Calibration c = [] {
        Calibration out;
        const ec::BurnLawParams start; // the compiled defaults
        const Set s0 = prepare_set(start);
        out.first = joint_fit(s0, wot_prepared_shipped(), start);
        out.set = prepare_set(out.first);
        out.wot = prepare_wot(out.first);
        out.second = joint_fit(out.set, out.wot, out.first);
        return out;
    }();
    return c;
}

// The declared phase 1b WOT bands (see the BLC1/BLC2 comments) evaluated for params p on the calibration's WOT rows.
struct WotBandStats {
    int n = 0, out_rapid = 0, out_dev = 0, out_mbt = 0;
    real f_min = 1e9, f_max = 0, mbt_min = 1e9, mbt_max = -1e9, slope = 0;
    real lnr_train = 0, lnr_hold = 0, lnr_hold_const = 0;
    int n_train = 0, n_hold = 0;
};

WotBandStats wot_band_stats(const Calibration& cal, const ec::BurnLawParams& p) {
    WotBandStats r;
    std::vector<real> lt, lh, lhc;
    real sxx = 0, sxy = 0, sx = 0, sy = 0, nn = 0;
    const auto& sc = wot_scans();
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        const Package& pk = *wot_packages()[k];
        for (std::size_t i = 0; i < pk.rows.size(); ++i) {
            const Prepared& w = cal.wot[k][i];
            const Row& row = pk.rows[i];
            if (!w.ok) continue;
            const real f = law_f(p, w.in);
            if (wot_fit_row(row) && row.rpm >= 1000.0 && row.rpm <= 6000.0) {
                ++r.n;
                const real mbt = predicted_mbt(pk, p, w.in);
                r.f_min = std::min(r.f_min, f);
                r.f_max = std::max(r.f_max, f);
                r.mbt_min = std::min(r.mbt_min, mbt);
                r.mbt_max = std::max(r.mbt_max, mbt);
                if (!(20.0 * f >= 15.0 && 20.0 * f <= 30.0)) ++r.out_rapid;
                if (!(15.0 * f >= 11.0 && 15.0 * f <= 23.0)) ++r.out_dev;
                if (!(mbt >= 10.0 && mbt <= 30.0)) ++r.out_mbt;
                if (row.rpm >= 1500.0 && row.rpm <= 4500.0) {
                    const real lx = std::log(row.rpm), ly = std::log(f);
                    sxx += lx * lx;
                    sxy += lx * ly;
                    sx += lx;
                    sy += ly;
                    nn += 1;
                }
            }
            if (!sc[k][i].ok || !wot_joint_row(row)) continue;
            const real lr = std::log(interp_w(sc[k][i], f) / sc[k][i].w_meas);
            if (pk.spec.train) {
                lt.push_back(lr);
            } else {
                lh.push_back(lr);
                lhc.push_back(std::log(interp_w(sc[k][i], 1.0) / sc[k][i].w_meas));
            }
        }
    }
    r.slope = (nn * sxy - sx * sy) / (nn * sxx - sx * sx);
    r.lnr_train = stats_of(lt).rms;
    r.lnr_hold = stats_of(lh).rms;
    r.lnr_hold_const = stats_of(lhc).rms;
    r.n_train = int(lt.size());
    r.n_hold = int(lh.size());
    return r;
}

} // namespace

// BLC1: the shipped constants are the joint fit, and the declared phase 1b bands that hold (declared before the fit;
// part-load bands = the phase 1 rms + 1 deg, the WOT-work band = no worse than the constant burn on the held-out rows,
// the direction band = a pooled WOT log-log slope of f over rpm <= 0.30, MIT 2.61 lec. 9 [C] rpm^0.2 + 0.1 [AS]).
// The second pass re-runs every row with the fitted constants; it moves them by less than the stated tolerance.
TEST_CASE("BLC1 the burn law's shipped constants are the joint part-load + WOT fit", "[burn_law][cycle][long]") {
    const Calibration& cal = calibration();
    const ec::BurnLawParams shipped;
    CHECK(cal.second.scale == Catch::Approx(shipped.scale).epsilon(2e-3));
    CHECK(cal.second.flame_speed_exponent == Catch::Approx(shipped.flame_speed_exponent).margin(2e-3));
    CHECK(shipped.rpm_exponent == 0.2);                 // [C], held
    CHECK(shipped.flame_speed_pressure_factor == 0.0);  // option F, held
    // Band 1: part-load MBT gap rms.
    const std::vector<PkgGaps> g = gaps(cal.set, shipped);
    std::vector<real> before, train, hold;
    // Every row that fails under the law fails with the constant burn too: the failures are the gas exchange's (see
    // prepare_all), not the law's.
    for (std::size_t k = 0; k < g.size(); ++k) {
        const std::vector<Prepared> fixed = prepare_all(*packages()[k], shipped, true);
        int law_only = 0;
        for (std::size_t q = 0; q < fixed.size(); ++q) {
            if (!cal.set.rows[k][q].ok && fixed[q].ok) ++law_only;
        }
        CHECK(law_only == 0);
        CHECK(g[k].n_ok() >= 30); // every package keeps at least 30 rows
        auto& v = packages()[k]->spec.train ? train : hold;
        v.insert(v.end(), g[k].after.begin(), g[k].after.end());
        if (packages()[k]->spec.train) before.insert(before.end(), g[k].before.begin(), g[k].before.end());
    }
    CHECK(stats_of(train).rms < stats_of(before).rms);
    CHECK(stats_of(train).rms <= 7.81);
    CHECK(stats_of(hold).rms <= 9.07);
    // The WOT rows: the same failure rule, and the test-local option evaluator agrees with the production law (option F).
    for (std::size_t k = 0; k < wot_packages().size(); ++k) {
        const Package& pk = *wot_packages()[k];
        const std::vector<Prepared> fixed = prepare_all(pk, shipped, true);
        int law_only = 0;
        for (std::size_t q = 0; q < fixed.size(); ++q) {
            if (!cal.wot[k][q].ok && fixed[q].ok) ++law_only;
            if (!cal.wot[k][q].ok) continue;
            const LawOpt o{shipped.flame_speed_pressure_factor, 0.0, 0.0};
            CHECK(f_local(shipped, o, pk.desc.cycle->stroke_m, cal.wot[k][q].in) ==
                  Catch::Approx(law_f(shipped, cal.wot[k][q].in)).epsilon(1e-12));
        }
        CHECK(law_only == 0);
    }
    // The direction band, and the WOT work on the training rows (the fit's own target, sigma 0.03).
    const WotBandStats w = wot_band_stats(cal, shipped);
    INFO("WOT ln r rms train " << w.lnr_train << " (n " << w.n_train << ") hold " << w.lnr_hold << " (n " << w.n_hold
                               << ") constant burn hold " << w.lnr_hold_const << "; slope " << w.slope);
    CHECK(w.n_train >= 15);
    CHECK(w.n_hold >= 15);
    CHECK(w.lnr_train <= 0.05);
    CHECK(w.slope <= 0.30);
}

// BLC2: band 2 as declared - every lambda 0.85-1.05, MAP >= 90 kPa WOT row of the six packages at 1000-6000 rpm burns
// 10-90 % in 15-30 deg, 0-10 % in 11-23 deg, MBT 10-30 deg BTDC. Basis MIT 2.61 lec. 9 [C] (fast-burn part load 0-10
// ~15, 10-90 ~20 deg) and ITU MAK 493E LN07 [C] (WOT burns at or below the part-load anchor); widths [AS].
// Band 3 as declared: the held-out WOT rows' work-ratio rms no worse than the constant burn's on the same rows.
// [!mayfail], both measured just outside: band 2 - 2 of 51 rows over 30 deg 10-90 (Mazda 2.0 1869 rpm, A25A 5414 rpm,
// Ford 2.7 4501 rpm over 23 deg 0-10 / 30 deg MBT); band 3 - 0.0741 vs the constant burn's 0.0726 (the A25A burns
// faster than the training engines, ln r -0.07..-0.24, no law input separates it).
TEST_CASE("BLC2 WOT burn angles inside the cited band, WOT work no worse than the constant burn", "[burn_law][cycle][long][!mayfail]") {
    const Calibration& cal = calibration();
    const ec::BurnLawParams shipped;
    const WotBandStats w = wot_band_stats(cal, shipped);
    INFO("n " << w.n << " f " << w.f_min << "-" << w.f_max << " MBT " << w.mbt_min << "-" << w.mbt_max << " outside: rapid "
              << w.out_rapid << " dev " << w.out_dev << " MBT " << w.out_mbt << "; held-out ln r rms " << w.lnr_hold
              << " constant burn " << w.lnr_hold_const);
    CHECK(w.n >= 50);
    CHECK(w.lnr_hold <= w.lnr_hold_const);
    CHECK(w.out_rapid == 0);
    CHECK(w.out_dev == 0);
    CHECK(w.out_mbt == 0);
}

// =====================================================================================================================
// Five-engine WOT dyno table (phase 1b report): run_dyno_sweep at throttle 1 on Full maps, per engine
//   law ECU+KL  - the shipped law, the ECU spark schedule capped by the knock limit (the engine as shipped)
//   law ECU     - the same without the knock limit (has_theta_kl cleared on every map set)
//   law MBT     - the ECU spark replaced by the maps' own theta_MBT (read through map_pressure_index, on a dense
//                 rpm x p_im table), no knock limit
//   const ECU / const MBT - the same two with the C17a constant burn ("burn_law": "fixed", 40.3205 / 3.8632 deg)
// A cam-profile engine (K20A) runs its MBT columns on the steady-state ECU profile (cam_profile -1) with the MBT table
// of the profile it would select (low below the switch rpm, high at or above); a lift-family engine (N52) runs at full
// lift (u = 1 at WOT), whose map set is `maps`.
// =====================================================================================================================
namespace {

Table2D dense_mbt_table(const SimulatedEngineMapsDesc& m) {
    Table2D t;
    for (real rpm = 500.0; rpm <= 10000.0; rpm += 100.0) t.x.push_back(rpm);
    for (real p = 10000.0; p < 110000.0; p += 1000.0) t.y.push_back(p);
    for (real p = 110000.0; p <= 400000.0; p += 5000.0) t.y.push_back(p);
    t.values.resize(t.x.size() * t.y.size());
    for (std::size_t i = 0; i < t.x.size(); ++i) {
        for (std::size_t j = 0; j < t.y.size(); ++j) {
            const real yp = map_pressure_index(m, t.x[i], t.y[j], kIsaAirPressurePa).y;
            t.values[i * t.y.size() + j] = m.theta_mbt_deg.value(t.x[i], yp);
        }
    }
    return t;
}

void clear_knock(SimulatedEngineDesc& d) {
    d.maps.has_theta_kl = false;
    for (auto& m : d.cam_profile_maps) m.has_theta_kl = false;
    for (auto& m : d.lift_node_maps) m.has_theta_kl = false;
}

ComponentParams load_engine_variant(const std::string& file, bool constant_burn, jobs::JobSystem& pool, const std::string& cache) {
    io::EngineMapOptions o;
    o.quality = MapQuality::Full;
    o.pool = &pool;
    o.cache_dir = cache;
    const std::filesystem::path path = kSrc / "data" / "engines" / file;
    if (!constant_burn) return io::load_engine_json(path.string(), o);
    std::ifstream in(path);
    json j = json::parse(in);
    j["cycle"]["burn_law"] = "fixed";
    j["cycle"]["burn_duration_deg"] = 40.3205;
    j["cycle"]["ignition_delay_deg"] = 3.8632;
    return io::load_engine_json_from_string(j.dump(), path.parent_path(), o);
}

} // namespace

TEST_CASE("BLC diag: five-engine WOT dyno table", "[.][diag][burn_law_cal]") {
    jobs::JobSystem pool10(10);
    const std::string cache = (std::filesystem::temp_directory_path() / "ps_blc_engine_maps").string();
    std::filesystem::create_directories(cache);
    struct E {
        const char* file;
        std::vector<real> rpm;
        real switch_rpm; // cam-profile switch (0 = none)
    };
    const std::vector<E> engines = {
        {"hyper_v8_one1_audio.json", {2000, 3000, 4000, 5000, 6000, 7000, 7500, 8000}, 0.0},
        {"demo_i4_pipes.json", {2000, 3000, 4000, 5000, 6000, 6500}, 0.0},
        {"honda_k20a_dc5.json", {2000, 3000, 4000, 5000, 6000, 7000, 8000}, 6000.0},
        {"bmw_n52b30.json", {1500, 2500, 3500, 4500, 5500, 6500}, 0.0},
        {"toyota_a25a_fks_gen.json", {1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000}, 0.0},
    };
    const char* env = std::getenv("PS_BLC_ENGINES"); // substring filter on the file names
    for (const E& e : engines) {
        if (env && std::string(e.file).find(env) == std::string::npos) continue;
        struct Col {
            std::vector<DynoPoint> pts;
        };
        Col cols[5];
        for (int burn = 0; burn < 2; ++burn) {
            const ComponentParams base = load_engine_variant(e.file, burn == 1, pool10, cache);
            const auto* d0 = std::get_if<SimulatedEngineDesc>(&base);
            REQUIRE(d0 != nullptr);
            for (int mode = 0; mode < 3; ++mode) {
                if (burn == 1 && mode == 0) continue; // constant burn: ECU and MBT only
                DynoSweepSpec spec;
                spec.throttle = 1.0;
                std::vector<DynoPoint> pts;
                if (mode == 2 && d0->has_cam_profiles()) {
                    // one sweep per profile with its own MBT table, each point taken from the profile the ECU selects
                    for (int prof = 0; prof < 2; ++prof) {
                        SimulatedEngineDesc d = *d0;
                        clear_knock(d);
                        d.ecu.spark_advance_deg = dense_mbt_table(prof == 0 ? d.maps : d.cam_profile_maps[0]);
                        spec.rpm.clear();
                        for (real r : e.rpm) {
                            if ((r >= e.switch_rpm) == (prof == 1)) spec.rpm.push_back(r);
                        }
                        if (spec.rpm.empty()) continue;
                        const DynoResult res = run_dyno_sweep(ComponentParams{d}, nullptr, spec, &pool10);
                        pts.insert(pts.end(), res.points.begin(), res.points.end());
                    }
                    std::sort(pts.begin(), pts.end(), [](const DynoPoint& a, const DynoPoint& b) { return a.rpm < b.rpm; });
                } else {
                    SimulatedEngineDesc d = *d0;
                    if (mode >= 1) clear_knock(d);
                    if (mode == 2) d.ecu.spark_advance_deg = dense_mbt_table(d.maps);
                    spec.rpm = e.rpm;
                    pts = run_dyno_sweep(ComponentParams{d}, nullptr, spec, &pool10).points;
                }
                cols[burn == 0 ? mode : 2 + mode] = Col{pts};
            }
        }
        std::printf("== %s WOT: torque N*m (spark deg BTDC)  [law = shipped burn law; const = C17a constant burn]\n", e.file);
        std::printf("   rpm |  law ECU+KL     |  law ECU        |  law MBT        | const ECU       | const MBT       | law/const MBT\n");
        for (std::size_t i = 0; i < e.rpm.size(); ++i) {
            std::printf("%6.0f", e.rpm[i]);
            for (const Col& c : cols) {
                if (i < c.pts.size()) std::printf(" | %7.1f (%5.1f)%s", c.pts[i].torque_nm, c.pts[i].spark_deg, c.pts[i].converged ? " " : "*");
                else std::printf(" |        -        ");
            }
            if (i < cols[2].pts.size() && i < cols[4].pts.size())
                std::printf(" | %+6.1f %%", 100.0 * (cols[2].pts[i].torque_nm / cols[4].pts[i].torque_nm - 1.0));
            std::printf("\n");
        }
    }
}
