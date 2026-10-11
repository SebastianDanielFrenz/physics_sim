// tests/unit/test_knock_calibration.cpp - engine garage P7b: the knock model's calibration (Toyota A25A, training) and
// its held-out validation (Mazda 2.0 SKYACTIV-G).
//
// The bands are declared in tests/unit/test_knock_limit.cpp (kKvBandLow/High, kKvBcBand; commit "declare the KV1/KV1-BC/
// KV2 bands before any calibrated or Mazda run") and repeated here by value.
//
//   KV1-BC  the boundary-condition replay the two constants are fitted on: every A25A training row - measured rpm, MAP,
//           intake temperature, cam phases, lambda and trapped air, the measured spark - within +-5 deg.
//   diags   the per-term budget of the P7 gap, the fit itself, the sensitivity report (C17b).
//
// Training rows (EPA NVFEL Report 26781 Tier 3 test-data workbook, literal readings, transcribed by
// <scratchpad>/p7b/a25a_training.py - see out/garage_p7b_progress.md "Transcription"):
//   * "High Load Final" rows 23/26/29/32/34/37/39 - the P7 KV1 WOT rows (lambda 0.80-0.90, DI only);
//   * "Steady State" knock-limited lambda-1 rows [D criterion, model independent]: lambda in [0.99, 1.01] and spark at
//     least 10 deg below the spark of the next-lower-MAP row at the same speed (or within 3 deg above the first such row
//     at a higher MAP); per speed the qualifying row with the highest pedal. A 15-25 deg spark collapse over 2 kPa of
//     MAP is the ECU's knock retard, not MBT behaviour.
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/knock.h"
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

// A user engine extending a shipped one with the knock opt-in, margin 0, the cycle's measured spark schedule removed
// (MBT tracking: the knock limit is the only spark limit) - test_knock_limit.cpp's knock_engine(base, 0, true).
json knock_engine(const char* base) {
    json j;
    j["extends"] = base;
    j["ecu"] = {{"ignition_strategy", "knock_limited"}, {"knock_margin_deg", 0.0}};
    j["cycle"] = {{"spark_advance_deg", nullptr}, {"spark_advance_deg_source", nullptr}};
    return j;
}

// One measured operating point. air_g_s 0: the trapped air is derived from fuel x lambda x AFR (the A25A workbook has no
// air-flow column); > 0: the measured inlet air flow.
struct TrainRow {
    int high_load; // 1: "High Load Final", 0: "Steady State"
    int row;
    real rpm, map_kpa, imt_c, lambda, icam_adv, ecam_ret, spark_btdc, fuel_g_s, pfi_pct, torque_nm;
    real air_g_s = 0.0;
};

// A25A training rows (sheet flag, sheet row, rpm, MAP kPa, IMT degC, lambda, intake cam adv, exhaust cam ret, spark deg
// BTDC, fuel meter g/s, PFI share %, torque N*m) - literal readings.
constexpr TrainRow kA25aRows[] = {
    {1, 23, 2017.13, 97.2897, 32.1949, 0.79747, 55, 0, -0.8834489, 3.81202, 0, 216.321},
    {1, 26, 2534.94, 97.3509, 32.1453, 0.867008, 55, 0, -0.1004669, 4.46601, 0, 225.125},
    {1, 29, 2995.63, 97.2884, 32.7409, 0.896868, 55, 5, 3.648455, 4.83102, 0, 217.813},
    {1, 32, 3571.24, 97.0007, 32.744, 0.879773, 55, 10, 5.285163, 6.09503, 0, 228.4},
    {1, 34, 4032.22, 96.8321, 32.6323, 0.859654, 55, 10, 3.459633, 7.28591, 0, 228.347},
    {1, 37, 4493.11, 96.6618, 33.4748, 0.87221, 55, 10, 3.372417, 8.15611, 0, 232.038},
    {1, 39, 4954.03, 96.4381, 34.1517, 0.870633, 52.999, 11.922, 4.398437, 9.08713, 0, 231.437},
    {0, 27, 1003, 95.87, 32.9, 1, 23.087, 5, -1.194, 1.068, 30.6262, 148.9},
    {0, 40, 1153, 95.59, 31.8, 1, 38.175, 0, -3.897, 1.443, 0, 168.8},
    {0, 53, 1326, 95.56, 31.4, 1, 50, 0, -6.514, 1.758, 0, 172.6},
    {0, 67, 1499, 96.53, 31.7, 1.01, 50, 0, -4.466, 1.978, 0, 176.3},
    {0, 82, 1729, 95.34, 31.6, 1, 50, 0, -5.441, 2.304, 29.2483, 177.6},
    {0, 94, 2017, 95.51, 31.9, 1, 50, 0, -2.47, 2.697, 29.8795, 185.8},
    {0, 109, 2535, 95.27, 32.9, 1, 50, 0, -1.747, 3.532, 29.8261, 196.7},
    {0, 122, 2996, 95.84, 33.4, 1, 50.929, 0, 1.477, 4.061, 30.4463, 196.7},
    {0, 135, 3572, 95.3, 32.2, 1, 49.969, 5, 2.345, 5.074, 30.1707, 207.8},
    {0, 143, 4032, 87.7, 32.8, 0.99, 40.837, 0.163, 6.173, 5.144, 30.6113, 184.6},
};

// DI charge-cooling estimate (diagnostic lever only, not in the generator): the whole latent heat of the DI share drawn
// from the charge at constant pressure. h_fg 305 kJ/kg [AS: gasoline, Heywood 1988 App. D recalled, not verified
// locally]; cp 1005 J/(kg K) [AS: air near 300 K]. No local source gives the share drawn from the gas (vs the walls).
constexpr real kHfgGasoline = 305e3;
constexpr real kCpCharge = 1005.0;

struct Bc {
    bool imt = true;    // measured intake temperature (else ISA 288.15 K, the generator's convention)
    bool cam = true;    // measured cam phases (else the engine's cam tables at the node)
    bool lambda = true; // measured lambda (else 1)
    bool air = true;    // trapped air scaled to the measured air (else the model's own)
    bool di_cooling = false;
    bool wall = false;
    real end_fraction = ec::kKnockEndBurnFraction;
    real prefactor_scale = ec::kKnockCalPrefactorScale;
    real activation_k = ec::kKnockCalActivationK;
    real margin_deg = 0.0;
    real residual_scale = 1.0;   // sensitivity: burned mass at IVC scaled at the IVC temperature (fresh air held)
    real vapour_cv_scale = 1.0;  // sensitivity: the fuel-vapour cv fit scaled
};

struct BcResult {
    real theta_kl = 0.0;
    real mbt_btdc = 0.0;
    real integral = 0.0; // Livengood-Wu integral at the measured spark
    real air_ratio = 1.0;
    real t_ivc = 0.0;
    real x_ivc = 0.0;
    bool limited = false, saturated = false;
};

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

real mbt_btdc_of(const CycleModelDesc& c) {
    const real mbt_atdc = ec::Wiebe::theta0_for_ca50(c.wiebe_a, c.wiebe_m, c.burn_duration_deg, c.ca50_mbt_deg_atdc);
    return -(mbt_atdc - c.ignition_delay_deg);
}

BcResult bc_replay(const Engine& e, const TrainRow& r, const Bc& o) {
    const CycleModelDesc& c = *e.c;
    const FuelDesc& f = *e.f;
    ec::OperatingPoint op;
    op.rpm = r.rpm;
    op.p_im_pa = r.map_kpa * 1000.0;
    op.t_im_k = o.imt ? r.imt_c + 273.15 : 288.15;
    op.p_amb_pa = 101325.0;
    op.lambda = o.lambda ? r.lambda : 1.0;
    op.spark_deg_atdc = -r.spark_btdc + c.ignition_delay_deg; // the IVC state of the measured spark
    if (o.cam) {
        op.intake_advance_deg = r.icam_adv;
        op.exhaust_retard_deg = r.ecam_ret;
    } else {
        op.intake_advance_deg = c.cam.intake_advance_deg.x.empty() ? 0.0 : c.cam.intake_advance_deg.value(op.rpm, op.p_im_pa);
        op.exhaust_retard_deg = c.cam.exhaust_retard_deg.x.empty() ? 0.0 : c.cam.exhaust_retard_deg.value(op.rpm, op.p_im_pa);
    }
    const ec::PointResult pr = ec::run_operating_point(*e.proto, op, ec::StepperSettings{ec::kStepFull, ec::kCyclesFull});
    ec::KnockReplayInput kin;
    kin.proto = e.proto.get();
    kin.op = op;
    kin.ivc = pr.ivc;
    kin.ignition_delay_deg = c.ignition_delay_deg;
    kin.burn_duration_deg = c.burn_duration_deg;
    kin.end_burn_fraction = o.end_fraction;
    kin.activation_k = o.activation_k;
    kin.wall_heat_end_gas = o.wall;
    kin.inv_on_term = ec::douaud_eyzat_inv_on_term(ec::octane_index(f.knock.ron, f.knock.mon), ec::kDouaudEyzatA_s * o.prefactor_scale);
    const real ff = 1.0 / (1.0 + op.lambda * f.afr_stoich);
    kin.fuel_mass_fraction = ff;
    kin.vapour_cv_scale = o.vapour_cv_scale;
    BcResult out;
    const real air_model = pr.ivc.m - pr.ivc.mb;
    if (o.air) {
        const real air_meas_g_s = r.air_g_s > 0.0 ? r.air_g_s : r.fuel_g_s * r.lambda * f.afr_stoich;
        const real air_meas = air_meas_g_s * 1e-3 / (r.rpm / 120.0 * real(c.cylinders));
        out.air_ratio = air_meas / air_model;
        kin.ivc.m *= out.air_ratio;
        kin.ivc.u *= out.air_ratio;
        kin.ivc.mb *= out.air_ratio;
    } else {
        out.air_ratio = 1.0;
    }
    const real v_ivc = e.proto->model().geom.eval(kin.ivc.crank_deg).v;
    if (o.residual_scale != 1.0) {
        // Burned mass scaled at the IVC temperature, fresh air held.
        const ec::CvState s0 = ec::cv_state(e.proto->model().gas, kin.ivc.m, kin.ivc.u, kin.ivc.mb, v_ivc);
        const real mb_new = kin.ivc.mb * o.residual_scale;
        const real m_new = kin.ivc.m - kin.ivc.mb + mb_new;
        kin.ivc.mb = mb_new;
        kin.ivc.m = m_new;
        kin.ivc.u = ec::energy_from_temperature(e.proto->model().gas, m_new, mb_new, s0.t);
    }
    const ec::CvState c0 = ec::cv_state(e.proto->model().gas, kin.ivc.m, kin.ivc.u, kin.ivc.mb, v_ivc);
    out.t_ivc = c0.t;
    out.x_ivc = c0.x;
    if (o.di_cooling) kin.end_gas_t0_offset_k -= (1.0 - r.pfi_pct / 100.0) * ff * kHfgGasoline / kCpCharge;
    out.mbt_btdc = mbt_btdc_of(c);
    const ec::KnockLimitResult kl = ec::knock_limited_spark(kin, out.mbt_btdc, o.margin_deg);
    out.theta_kl = kl.theta_kl_btdc;
    out.limited = kl.limited;
    out.saturated = kl.saturated;
    ec::KnockReplayInput full = kin;
    full.stop_at_knock = false;
    out.integral = ec::knock_replay(full, r.spark_btdc).integral;
    return out;
}

// theta_kl - spark for every row, the rows in parallel on the pool (each job writes its own slot).
std::vector<BcResult> bc_all(const Engine& e, const TrainRow* rows, std::size_t n, const Bc& o) {
    std::vector<BcResult> out(n);
    pool().parallel_for(n, 1, [&](std::size_t b, std::size_t en) {
        for (std::size_t i = b; i < en; ++i) out[i] = bc_replay(e, rows[i], o);
    });
    return out;
}

real rms_residual(const std::vector<BcResult>& r, const TrainRow* rows) {
    real s = 0.0;
    for (std::size_t i = 0; i < r.size(); ++i) s += (r[i].theta_kl - rows[i].spark_btdc) * (r[i].theta_kl - rows[i].spark_btdc);
    return std::sqrt(s / real(r.size()));
}

const Engine& a25a() {
    static const Engine e(load_sim(knock_engine("toyota_a25a_fks_gen.json"), kSrc / "data" / "engines", MapQuality::Sketch, false));
    return e;
}

constexpr std::size_t kNA25a = std::size(kA25aRows);

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// KV1-BC: the calibrated model on the rows it was fitted on, at their measured boundary conditions. Guards the fitted
// constant itself: each row within the declared +-5 deg, and the shipped pre-factor scale a minimum of the objective
// (x0.9 and x1.1 both worse - a drifted constant or a changed replay that moves the optimum fails here).
TEST_CASE("P7b KV1-BC: calibrated knock limit at the A25A training rows' measured boundary conditions", "[knock][drivetrain][cycle]") {
    constexpr real kKvBcBand = 5.0; // declared in test_knock_limit.cpp before the first calibrated run
    const Engine& e = a25a();
    const Bc o; // measured IMT, cam, lambda, trapped air; generator constants
    const std::vector<BcResult> r = bc_all(e, kA25aRows, kNA25a, o);
    real mean = 0.0;
    for (std::size_t i = 0; i < kNA25a; ++i) {
        const real d = r[i].theta_kl - kA25aRows[i].spark_btdc;
        mean += d / real(kNA25a);
        std::printf("KV1-BC %c%-3d %6.0f rpm: theta_kl %6.2f spark %6.2f diff %6.2f air_ratio %.3f\n", kA25aRows[i].high_load ? 'H' : 'S',
                    kA25aRows[i].row, kA25aRows[i].rpm, r[i].theta_kl, kA25aRows[i].spark_btdc, d, r[i].air_ratio);
        INFO("row " << kA25aRows[i].row << " rpm " << kA25aRows[i].rpm);
        CHECK(r[i].limited);
        CHECK_FALSE(r[i].saturated);
        CHECK(std::abs(d) <= kKvBcBand);
    }
    const real rms = rms_residual(r, kA25aRows);
    std::printf("KV1-BC rms %.3f mean %+.3f\n", rms, mean);
    Bc lo = o, hi = o;
    lo.prefactor_scale = 0.9 * ec::kKnockCalPrefactorScale;
    hi.prefactor_scale = 1.1 * ec::kKnockCalPrefactorScale;
    const real rms_lo = rms_residual(bc_all(e, kA25aRows, kNA25a, lo), kA25aRows);
    const real rms_hi = rms_residual(bc_all(e, kA25aRows, kNA25a, hi), kA25aRows);
    std::printf("KV1-BC rms at scale x0.9 %.3f, x1.1 %.3f\n", rms_lo, rms_hi);
    CHECK(rms < rms_lo);
    CHECK(rms < rms_hi);
    CHECK(std::abs(mean) < 0.5);
}

// ---------------------------------------------------------------------------------------------------------------------
// Diagnostics (hidden): the per-term budget of the P7 gap.
TEST_CASE("P7b diag: knock budget per term at the A25A training rows", "[.][diag][knock][drivetrain][cycle]") {
    const Engine& e = a25a();
    std::printf("A25A fuel %s RON %.1f MON %.1f, MBT %.2f BTDC, constants: scale %.4g B %.1f K\n", e.f->name.c_str(),
                e.f->knock.ron, e.f->knock.mon, mbt_btdc_of(*e.c), ec::kKnockCalPrefactorScale, ec::kKnockCalActivationK);
    Bc v0;
    v0.imt = v0.cam = v0.lambda = v0.air = false;
    v0.prefactor_scale = 1.0;
    v0.activation_k = ec::kDouaudEyzatB_K;
    struct Term {
        const char* name;
        Bc o;
    };
    std::vector<Term> terms;
    terms.push_back({"V0 (P7: ISA, cam table, lambda 1, model air, literature)", v0});
    auto with = [&](const char* name, auto fn) {
        Bc o = v0;
        fn(o);
        terms.push_back({name, o});
    };
    with("+ measured IMT", [](Bc& o) { o.imt = true; });
    with("+ measured cam", [](Bc& o) { o.cam = true; });
    with("+ measured lambda (vapour)", [](Bc& o) { o.lambda = true; });
    with("+ measured trapped air", [](Bc& o) { o.air = true; });
    with("+ DI charge cooling [AS upper bound]", [](Bc& o) { o.di_cooling = true; });
    with("+ wall heat in the end gas (bulk until burn)", [](Bc& o) { o.wall = true; });
    with("end fraction 0.80", [](Bc& o) { o.end_fraction = 0.80; });
    with("end fraction 0.95", [](Bc& o) { o.end_fraction = 0.95; });
    with("pre-factor x2", [](Bc& o) { o.prefactor_scale = 2.0; });
    with("B -10 %", [](Bc& o) { o.activation_k = 0.9 * ec::kDouaudEyzatB_K; });
    with("BC (IMT+cam+lambda+air), literature", [](Bc& o) { o.imt = o.cam = o.lambda = o.air = true; });
    with("BC + DI cooling, literature", [](Bc& o) { o.imt = o.cam = o.lambda = o.air = o.di_cooling = true; });
    with("BC + wall heat, literature", [](Bc& o) { o.imt = o.cam = o.lambda = o.air = o.wall = true; });
    {
        Bc o;
        terms.push_back({"BC, calibrated (generator constants)", o});
    }
    std::vector<std::vector<BcResult>> res;
    for (const Term& t : terms) res.push_back(bc_all(e, kA25aRows, kNA25a, t.o));
    std::printf("\nrows: theta_kl - spark_meas [deg]; first block = V0 absolute, the rest = shift vs V0 (theta_kl term - V0)\n");
    std::printf("%-58s", "row (sheet/row rpm map)");
    for (std::size_t i = 0; i < kNA25a; ++i) std::printf(" %c%-3d", kA25aRows[i].high_load ? 'H' : 'S', kA25aRows[i].row);
    std::printf("   mean   rms\n");
    for (std::size_t t = 0; t < terms.size(); ++t) {
        std::printf("%-58s", terms[t].name);
        real sum = 0.0, sq = 0.0;
        for (std::size_t i = 0; i < kNA25a; ++i) {
            const real d = res[t][i].theta_kl - kA25aRows[i].spark_btdc;
            const real shown = (t == 0 || t >= terms.size() - 4) ? d : res[t][i].theta_kl - res[0][i].theta_kl;
            std::printf(" %5.1f", shown);
            sum += shown;
            sq += shown * shown;
        }
        std::printf("  %5.2f %5.2f\n", sum / real(kNA25a), std::sqrt(sq / real(kNA25a)));
    }
    std::printf("\nper row: rpm map imt lambda spark | V0: T_ivc x_ivc air_ratio(meas/model) I(spark) | BC lit: I(spark) T_ivc\n");
    for (std::size_t i = 0; i < kNA25a; ++i) {
        const TrainRow& r = kA25aRows[i];
        const BcResult& a = res[0][i];
        const BcResult& b = res[terms.size() - 4][i];
        Bc probe = v0;
        probe.air = true; // only to report the ratio
        const BcResult pa = bc_replay(e, r, probe);
        std::printf("%c%-3d %6.0f %6.2f %5.1f %5.3f %6.2f | %6.1f %.3f %.3f %6.2f | %6.2f %6.1f%s%s\n", r.high_load ? 'H' : 'S', r.row,
                    r.rpm, r.map_kpa, r.imt_c, r.lambda, r.spark_btdc, a.t_ivc, a.x_ivc, pa.air_ratio, a.integral, b.integral, b.t_ivc,
                    a.saturated ? " V0-saturated" : "", b.saturated ? " BC-saturated" : "");
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The fit (hidden diag): two constants on the BC replay of the A25A training rows, least squares of theta_kl - spark.
namespace {

struct FitPoint {
    real a = 0.0, b = 0.0, rms = 0.0, mean = 0.0;
};

// Objective for (log10 pre-factor scale, second parameter); second = B [K] or the end fraction.
FitPoint fit_eval(const Engine& e, real log_scale, real second, bool second_is_end) {
    Bc o;
    o.prefactor_scale = std::pow(10.0, log_scale);
    if (second_is_end) {
        o.end_fraction = second;
        o.activation_k = ec::kDouaudEyzatB_K;
    } else {
        o.activation_k = second;
    }
    const std::vector<BcResult> r = bc_all(e, kA25aRows, kNA25a, o);
    FitPoint p;
    p.a = log_scale;
    p.b = second;
    p.rms = rms_residual(r, kA25aRows);
    for (std::size_t i = 0; i < r.size(); ++i) p.mean += (r[i].theta_kl - kA25aRows[i].spark_btdc) / real(r.size());
    return p;
}

FitPoint golden_scale(const Engine& e, real second, bool second_is_end, real lo, real hi) {
    // The objective is flat where every row saturates (both search ends), so a plain golden section can lock onto a
    // plateau: scan log10(scale) in 0.1 steps first, then refine inside the best step's neighbours.
    {
        real best_x = lo, best_rms = 1e300;
        for (real x = lo; x <= hi + 1e-9; x += 0.1) {
            const FitPoint p = fit_eval(e, x, second, second_is_end);
            if (p.rms < best_rms) {
                best_rms = p.rms;
                best_x = x;
            }
        }
        lo = best_x - 0.1;
        hi = best_x + 0.1;
    }
    const real g = 0.5 * (std::sqrt(5.0) - 1.0);
    real x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
    FitPoint f1 = fit_eval(e, x1, second, second_is_end), f2 = fit_eval(e, x2, second, second_is_end);
    while (hi - lo > 1e-3) {
        if (f1.rms <= f2.rms) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - g * (hi - lo);
            f1 = fit_eval(e, x1, second, second_is_end);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + g * (hi - lo);
            f2 = fit_eval(e, x2, second, second_is_end);
        }
    }
    return f1.rms <= f2.rms ? f1 : f2;
}

} // namespace

TEST_CASE("P7b diag: knock calibration fit on the A25A training rows", "[.][diag][knock][drivetrain][cycle]") {
    const Engine& e = a25a();
    // 1) one parameter (pre-factor only, literature B) for reference.
    {
        const FitPoint p = golden_scale(e, ec::kDouaudEyzatB_K, false, -1.0, 1.0);
        std::printf("FIT1 pre-factor only: scale %.6f (log10 %.6f), B %.1f: rms %.3f mean %.3f\n", std::pow(10.0, p.a), p.a, p.b, p.rms, p.mean);
        Bc o;
        o.prefactor_scale = std::pow(10.0, p.a);
        o.activation_k = p.b;
        const std::vector<BcResult> r = bc_all(e, kA25aRows, kNA25a, o);
        for (std::size_t i = 0; i < kNA25a; ++i) {
            std::printf("FIT1 row %c%-3d %6.0f rpm: theta_kl %6.2f spark %6.2f residual %6.2f%s\n", kA25aRows[i].high_load ? 'H' : 'S',
                        kA25aRows[i].row, kA25aRows[i].rpm, r[i].theta_kl, kA25aRows[i].spark_btdc, r[i].theta_kl - kA25aRows[i].spark_btdc,
                        r[i].saturated ? " saturated" : (r[i].limited ? "" : " not limited"));
        }
        for (real ds : {-0.1, 0.1}) {
            const FitPoint q = fit_eval(e, p.a + std::log10(1.0 + ds), p.b, false);
            std::printf("FIT1 sens: scale x%.2f -> rms %.3f mean %+.3f\n", 1.0 + ds, q.rms, q.mean);
        }
    }
    // 2) (pre-factor, B): coarse scan over B, golden over log10(scale) per B, then golden over B.
    FitPoint best;
    best.rms = 1e300;
    for (real b = 2000.0; b <= 7001.0; b += 500.0) {
        const FitPoint p = golden_scale(e, b, false, -2.0, 1.5);
        std::printf("FIT2 B %.0f: scale %.4f rms %.3f mean %.3f\n", b, std::pow(10.0, p.a), p.rms, p.mean);
        if (p.rms < best.rms) best = p;
    }
    {
        real lo = best.b - 500.0, hi = best.b + 500.0;
        const real g = 0.5 * (std::sqrt(5.0) - 1.0);
        real x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
        FitPoint f1 = golden_scale(e, x1, false, -2.0, 1.5), f2 = golden_scale(e, x2, false, -2.0, 1.5);
        while (hi - lo > 5.0) {
            if (f1.rms <= f2.rms) {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - g * (hi - lo);
                f1 = golden_scale(e, x1, false, -2.0, 1.5);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + g * (hi - lo);
                f2 = golden_scale(e, x2, false, -2.0, 1.5);
            }
        }
        const FitPoint fb = f1.rms <= f2.rms ? f1 : f2;
        if (fb.rms < best.rms) best = fb;
    }
    std::printf("FIT2 best: scale %.6f (log10 %.6f) B %.2f K: rms %.3f mean %.3f\n", std::pow(10.0, best.a), best.a, best.b, best.rms, best.mean);
    {
        Bc o;
        o.prefactor_scale = std::pow(10.0, best.a);
        o.activation_k = best.b;
        const std::vector<BcResult> r = bc_all(e, kA25aRows, kNA25a, o);
        for (std::size_t i = 0; i < kNA25a; ++i) {
            std::printf("FIT2 row %c%-3d %6.0f rpm: theta_kl %6.2f spark %6.2f residual %6.2f%s\n", kA25aRows[i].high_load ? 'H' : 'S',
                        kA25aRows[i].row, kA25aRows[i].rpm, r[i].theta_kl, kA25aRows[i].spark_btdc, r[i].theta_kl - kA25aRows[i].spark_btdc,
                        r[i].saturated ? " saturated" : (r[i].limited ? "" : " not limited"));
        }
        for (real ds : {-0.1, 0.1}) {
            const FitPoint p = fit_eval(e, best.a + std::log10(1.0 + ds), best.b, false);
            std::printf("FIT2 sens: scale x%.2f -> rms %.3f mean %+.3f\n", 1.0 + ds, p.rms, p.mean);
        }
        for (real db : {-0.1, 0.1}) {
            const FitPoint p = fit_eval(e, best.a, best.b * (1.0 + db), false);
            std::printf("FIT2 sens: B x%.2f -> rms %.3f mean %+.3f\n", 1.0 + db, p.rms, p.mean);
        }
    }
    // 3) alternative pair (pre-factor, end fraction), literature B.
    FitPoint alt;
    alt.rms = 1e300;
    for (real xe : {0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 0.98}) {
        const FitPoint p = golden_scale(e, xe, true, -2.0, 1.5);
        std::printf("FIT3 end %.2f: scale %.4f rms %.3f mean %.3f\n", xe, std::pow(10.0, p.a), p.rms, p.mean);
        if (p.rms < alt.rms) alt = p;
    }
    std::printf("FIT3 best (grid): scale %.4f end %.2f rms %.3f\n", std::pow(10.0, alt.a), alt.b, alt.rms);
}
