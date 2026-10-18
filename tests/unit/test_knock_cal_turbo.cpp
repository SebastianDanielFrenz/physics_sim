// tests/unit/test_knock_cal_turbo.cpp - knock calibration phase B: the knock model at MEASURED boundary conditions on the
// EPA NVFEL turbo engines (phase A fixtures tests/support/fixtures/epa_turbo/) plus the A25A KV1-BC rows.
//
// Method (the KV1-BC method of test_knock_calibration.cpp, unchanged): per measured row one converged cycle point at the
// row's rpm, MAP, intake temperature, lambda, cam phases (the fixture's cam table where the row has no cam channel) and
// measured spark; its IVC state is scaled to the measured trapped air (fuel meter x lambda x AFR), then the closed-cycle
// knock search gives theta_kl with margin 0, compared with the MEASURED spark. The measured ECU spark is the OEM's knock
// limit minus its own (unknown) margin, so - exactly as KV1-BC treated the A25A - the fitted constants absorb the OEM margin:
// theta_kl(margin 0) is "the spark an OEM calibration would run", and an extra ECU margin on top of it is conservative.
// Exhaust back pressure = the measured turbine-inlet pressure (a measured boundary condition; the generator uses ambient,
// reported as a sensitivity).
//
// Row sets (declared before the fit, see the commit message): "envelope" = the highest-torque row per 50 rpm speed bin
// (phase A's WOT definition); the fit uses the Honda L15B7 Tier 3 envelope rows plus the 17 A25A KV1-BC rows.
#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/knock.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/io/fuel_io.h"
#include "ps/io/knock_model_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;

namespace {

// Knock model v2 (P7b) pre-factor scale at n 1.7 / B 3800 K: the diag grids below centre their scale search on it.
constexpr real kV2Scale = 1.26;

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);
const std::filesystem::path kFix = kSrc / "tests" / "support" / "fixtures" / "epa_turbo";

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

struct Row {
    real rpm, map_kpa, imt_c, lambda, spark_btdc, fuel_g_s, exh_kpa;
    bool has_icam, has_ecam;
    real icam_adv, ecam_ret;
    bool envelope;
    int row;
};

// The A25A KV1-BC training rows - the same literals as test_knock_calibration.cpp kA25aRows (sheet row, rpm, MAP,
// IMT, lambda, intake adv, exhaust ret, spark, fuel g/s); NA, exhaust at ambient.
struct A25a {
    int row;
    real rpm, map, imt, lambda, icam, ecam, spark, fuel;
};
constexpr A25a kA25a[] = {
    {23, 2017.13, 97.2897, 32.1949, 0.79747, 55, 0, -0.8834489, 3.81202}, {26, 2534.94, 97.3509, 32.1453, 0.867008, 55, 0, -0.1004669, 4.46601},
    {29, 2995.63, 97.2884, 32.7409, 0.896868, 55, 5, 3.648455, 4.83102}, {32, 3571.24, 97.0007, 32.744, 0.879773, 55, 10, 5.285163, 6.09503},
    {34, 4032.22, 96.8321, 32.6323, 0.859654, 55, 10, 3.459633, 7.28591}, {37, 4493.11, 96.6618, 33.4748, 0.87221, 55, 10, 3.372417, 8.15611},
    {39, 4954.03, 96.4381, 34.1517, 0.870633, 52.999, 11.922, 4.398437, 9.08713}, {27, 1003, 95.87, 32.9, 1, 23.087, 5, -1.194, 1.068},
    {40, 1153, 95.59, 31.8, 1, 38.175, 0, -3.897, 1.443}, {53, 1326, 95.56, 31.4, 1, 50, 0, -6.514, 1.758},
    {67, 1499, 96.53, 31.7, 1.01, 50, 0, -4.466, 1.978}, {82, 1729, 95.34, 31.6, 1, 50, 0, -5.441, 2.304},
    {94, 2017, 95.51, 31.9, 1, 50, 0, -2.47, 2.697}, {109, 2535, 95.27, 32.9, 1, 50, 0, -1.747, 3.532},
    {122, 2996, 95.84, 33.4, 1, 50.929, 0, 1.477, 4.061}, {135, 3572, 95.3, 32.2, 1, 49.969, 5, 2.345, 5.074},
    {143, 4032, 87.7, 32.8, 0.99, 40.837, 0.163, 6.173, 5.144},
};

struct Package {
    std::string name;
    SimulatedEngineDesc desc;
    FuelDesc fuel;
    std::unique_ptr<ec::CycleStepper> proto;
    std::vector<Row> rows;
};

SimulatedEngineDesc load_desc(const std::filesystem::path& file) {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool();
    o.generate = false;
    const ComponentParams p = io::load_engine_json(file.generic_string(), o);
    const auto* e = std::get_if<SimulatedEngineDesc>(&p);
    REQUIRE(e != nullptr);
    REQUIRE(e->cycle.has_value());
    return *e;
}

std::unique_ptr<Package> load_package(const std::string& rows_name, const std::string& engine_dir) {
    auto pk = std::make_unique<Package>();
    pk->name = rows_name;
    std::ifstream in(kFix / (rows_name + "_rows.json"));
    REQUIRE(in.good());
    const json j = json::parse(in);
    pk->desc = load_desc(kFix / engine_dir / "engine.json");
    pk->fuel = io::load_fuel_json((kSrc / "data" / "fuels" / (j.at("fuel").get<std::string>() + ".json")).generic_string());
    REQUIRE(pk->fuel.has_knock);
    pk->proto = std::make_unique<ec::CycleStepper>(*pk->desc.cycle, pk->fuel.lhv_j_kg, pk->fuel.afr_stoich);
    for (const json& r : j.at("rows")) {
        if (!r.at("high_load").get<bool>()) continue;
        auto num = [&](const char* k) { return (r.contains(k) && r.at(k).is_number()) ? r.at(k).get<real>() : std::nan(""); };
        Row w{};
        w.rpm = num("rpm");
        w.map_kpa = num("map_kpa");
        w.imt_c = num("imt_c");
        w.lambda = num("lambda");
        w.spark_btdc = num("spark_btdc_deg");
        w.fuel_g_s = std::isfinite(num("fuel_meter_g_s")) ? num("fuel_meter_g_s") : num("inj_fuel_g_s");
        w.exh_kpa = std::isfinite(num("turb_in_kpa")) ? num("turb_in_kpa") : 0.0;
        w.has_icam = std::isfinite(num("intake_cam_adv_deg"));
        w.has_ecam = std::isfinite(num("exhaust_cam_ret_deg"));
        w.icam_adv = w.has_icam ? num("intake_cam_adv_deg") : 0.0;
        w.ecam_ret = w.has_ecam ? num("exhaust_cam_ret_deg") : 0.0;
        w.envelope = r.at("envelope");
        w.row = r.at("row");
        if (!(std::isfinite(w.rpm) && std::isfinite(w.map_kpa) && std::isfinite(w.imt_c) && std::isfinite(w.lambda) &&
              std::isfinite(w.spark_btdc) && std::isfinite(w.fuel_g_s))) {
            continue;
        }
        pk->rows.push_back(w);
    }
    return pk;
}

std::unique_ptr<Package> load_a25a() {
    auto pk = std::make_unique<Package>();
    pk->name = "a25a_kv1bc";
    json j;
    j["extends"] = "toyota_a25a_fks_gen.json";
    j["ecu"] = {{"ignition_strategy", "knock_limited"}, {"knock_margin_deg", 0.0}};
    j["cycle"] = {{"spark_advance_deg", nullptr}, {"spark_advance_deg_source", nullptr}};
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool();
    o.generate = false;
    const ComponentParams p = io::load_engine_json_from_string(j.dump(), kSrc / "data" / "engines", o);
    pk->desc = std::get<SimulatedEngineDesc>(p);
    pk->fuel = pk->desc.fuels[0];
    pk->proto = std::make_unique<ec::CycleStepper>(*pk->desc.cycle, pk->fuel.lhv_j_kg, pk->fuel.afr_stoich);
    for (const A25a& a : kA25a) {
        Row w{};
        w.rpm = a.rpm;
        w.map_kpa = a.map;
        w.imt_c = a.imt;
        w.lambda = a.lambda;
        w.spark_btdc = a.spark;
        w.fuel_g_s = a.fuel;
        w.exh_kpa = 0.0;
        w.has_icam = w.has_ecam = true;
        w.icam_adv = a.icam;
        w.ecam_ret = a.ecam;
        w.envelope = true;
        w.row = a.row;
        pk->rows.push_back(w);
    }
    return pk;
}

// Boundary-condition switches for the per-row cycle point (sensitivities).
struct BcOpt {
    bool exh_measured = true; // exhaust reservoir = measured turbine-inlet pressure (else ambient, the generator's choice)
    real icam_shift = 0.0;    // added to the intake cam advance (IVC sensitivity: +10 = IVC 10 deg earlier)
    // true: the measured air is the DELIVERED air (it includes what blows through during valve overlap under boost):
    //   trapped = measured x the model's own trapping efficiency (trapped / net fresh air through the intake valve), and the
    //   trapped lambda = measured exhaust lambda x that efficiency (DI: all fuel is trapped). false: KV1-BC's rule,
    //   measured air = trapped air (exact for an NA engine without blow-through).
    bool delivered_air = true;
    bool envelope_only = false; // prepare only the envelope rows (the asserted tests; the others stay ok = false)
};

// The boundary conditions of the knock model v3 fit (KV1-BC's rule): trapped = measured air, exhaust at ambient.
BcOpt fit_bc() {
    BcOpt o;
    o.exh_measured = false;
    o.delivered_air = false;
    o.envelope_only = true;
    return o;
}

// A prepared row: the knock search input at measured boundary conditions, octane set later.
struct Prepared {
    bool ok = false;
    ec::KnockReplayInput kin;
    real mbt_btdc = 0.0;
    real spark = 0.0;
    real air_ratio = 1.0, t_ivc = 0.0, x_ivc = 0.0, trap_eff = 1.0;
};

real mbt_btdc_of(const CycleModelDesc& c) {
    const real mbt_atdc = ec::Wiebe::theta0_for_ca50(c.wiebe_a, c.wiebe_m, c.burn_duration_deg, c.ca50_mbt_deg_atdc);
    return -(mbt_atdc - c.ignition_delay_deg);
}

Prepared prepare(const Package& pk, const Row& r, const BcOpt& o) {
    const CycleModelDesc& c = *pk.desc.cycle;
    Prepared out;
    ec::OperatingPoint op;
    op.rpm = r.rpm;
    op.p_im_pa = r.map_kpa * 1000.0;
    op.t_im_k = r.imt_c + 273.15;
    op.p_amb_pa = (o.exh_measured && r.exh_kpa > 0.0) ? r.exh_kpa * 1000.0 : 101325.0;
    op.lambda = r.lambda;
    op.spark_deg_atdc = -r.spark_btdc + c.ignition_delay_deg;
    const real it = c.cam.intake_advance_deg.x.empty() ? 0.0 : c.cam.intake_advance_deg.value(op.rpm, op.p_im_pa);
    const real et = c.cam.exhaust_retard_deg.x.empty() ? 0.0 : c.cam.exhaust_retard_deg.value(op.rpm, op.p_im_pa);
    op.intake_advance_deg = (r.has_icam ? r.icam_adv : it) + o.icam_shift;
    op.exhaust_retard_deg = r.has_ecam ? r.ecam_ret : et;
    ec::PointResult pr;
    try {
        pr = ec::run_operating_point(*pk.proto, op, ec::StepperSettings{ec::kStepFull, ec::kCyclesFull});
    } catch (...) {
        return out;
    }
    if (!(pr.ivc.m > 0.0)) return out;
    ec::KnockReplayInput& kin = out.kin;
    kin.proto = pk.proto.get();
    kin.op = op;
    kin.ivc = pr.ivc;
    kin.ignition_delay_deg = c.ignition_delay_deg;
    kin.burn_duration_deg = c.burn_duration_deg;
    const real air_model = pr.ivc.m - pr.ivc.mb;
    const real air_meas = r.fuel_g_s * r.lambda * pk.fuel.afr_stoich * 1e-3 / (r.rpm / 120.0 * real(c.cylinders));
    real te = 1.0;
    if (o.delivered_air && pr.m_air_j_cyc > 0.0) te = std::min(real(1.0), air_model / pr.m_air_j_cyc);
    out.trap_eff = te;
    kin.op.lambda = r.lambda * te;
    kin.fuel_mass_fraction = 1.0 / (1.0 + kin.op.lambda * pk.fuel.afr_stoich);
    out.air_ratio = air_meas * te / air_model;
    kin.ivc.m *= out.air_ratio;
    kin.ivc.u *= out.air_ratio;
    kin.ivc.mb *= out.air_ratio;
    const real v = pk.proto->model().geom.eval(kin.ivc.crank_deg).v;
    const ec::CvState c0 = ec::cv_state(pk.proto->model().gas, kin.ivc.m, kin.ivc.u, kin.ivc.mb, v);
    out.t_ivc = c0.t;
    out.x_ivc = c0.x;
    out.mbt_btdc = mbt_btdc_of(c);
    out.spark = r.spark_btdc;
    out.ok = true;
    return out;
}

std::vector<Prepared> prepare_all(const Package& pk, const BcOpt& o) {
    std::vector<Prepared> out(pk.rows.size());
    pool().parallel_for(pk.rows.size(), 1, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            if (o.envelope_only && !pk.rows[i].envelope) continue;
            out[i] = prepare(pk, pk.rows[i], o);
        }
    });
    return out;
}

// The model parameters under test (defaults = the shipped model).
struct Model {
    real scale = ec::kKnockCalPrefactorScale;
    real p_exp = ec::kKnockCalPressureExponent;
    real b_k = ec::kKnockCalActivationK;
    real k_oi = ec::kOctaneIndexK;
};

struct Res {
    bool ok = false;
    real theta_kl = 0.0, diff = 0.0;
    bool limited = false, saturated = false;
};

std::vector<Res> eval_all(const Package& pk, const std::vector<Prepared>& pr, const Model& m) {
    std::vector<Res> out(pr.size());
    const real oi = pk.fuel.knock.ron - m.k_oi * (pk.fuel.knock.ron - pk.fuel.knock.mon);
    const real inv_on = ec::douaud_eyzat_inv_on_term(oi, ec::kDouaudEyzatA_s * m.scale);
    pool().parallel_for(pr.size(), 1, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            if (!pr[i].ok) continue;
            ec::KnockReplayInput kin = pr[i].kin;
            kin.inv_on_term = inv_on;
            kin.p_exponent = m.p_exp;
            kin.activation_k = m.b_k;
            const ec::KnockLimitResult kl = ec::knock_limited_spark(kin, pr[i].mbt_btdc, 0.0);
            out[i].ok = true;
            out[i].theta_kl = kl.theta_kl_btdc;
            out[i].diff = kl.theta_kl_btdc - pr[i].spark;
            out[i].limited = kl.limited;
            out[i].saturated = kl.saturated;
        }
    });
    return out;
}

struct Stats {
    int n = 0;
    real mean = 0.0, rms = 0.0, max_abs = 0.0;
    int outside5 = 0;
};

Stats stats(const Package& pk, const std::vector<Res>& r, bool envelope_only) {
    Stats s;
    real sum = 0.0, sq = 0.0;
    for (std::size_t i = 0; i < r.size(); ++i) {
        if (!r[i].ok || (envelope_only && !pk.rows[i].envelope)) continue;
        ++s.n;
        sum += r[i].diff;
        sq += r[i].diff * r[i].diff;
        s.max_abs = std::max(s.max_abs, std::abs(r[i].diff));
        if (std::abs(r[i].diff) > 5.0) ++s.outside5;
    }
    if (s.n > 0) {
        s.mean = sum / s.n;
        s.rms = std::sqrt(sq / s.n);
    }
    return s;
}

struct Set {
    std::unique_ptr<Package> pk;
    std::vector<Prepared> pr;
};

std::vector<Set>& sets() {
    static std::vector<Set> s = [] {
        std::vector<Set> v;
        auto add = [&](std::unique_ptr<Package> p) {
            Set e;
            e.pk = std::move(p);
            v.push_back(std::move(e));
        };
        add(load_a25a());
        add(load_package("honda_l15b7_t3", "honda_l15b7"));
        add(load_package("honda_l15b7_t2", "honda_l15b7"));
        add(load_package("ford_27_t2_cell9", "ford_27"));
        add(load_package("ford_27_levIII_cell9", "ford_27"));
        add(load_package("ford_27_t2_hd1", "ford_27"));
        add(load_package("ford_27_t3_hd", "ford_27"));
        add(load_package("ford_16_t2", "ford_16"));
        add(load_package("ford_16_levIII", "ford_16"));
        add(load_package("mazda_25t_t2", "mazda_25t"));
        add(load_package("mazda_25t_t3", "mazda_25t"));
        return v;
    }();
    return s;
}

void prepare_sets(const BcOpt& o) {
    for (Set& s : sets()) s.pr = prepare_all(*s.pk, o);
}

void print_table(const char* title, const Model& m) {
    std::printf("\n== %s: scale %.4g p_exp %.3f B %.1f K_oi %.3f\n", title, m.scale, m.p_exp, m.b_k, m.k_oi);
    std::printf("%-22s %4s %7s %6s %6s %4s | %4s %7s %6s %6s %4s\n", "package", "nEnv", "mean", "rms", "max", ">5", "nAll", "mean", "rms", "max", ">5");
    for (const Set& s : sets()) {
        const std::vector<Res> r = eval_all(*s.pk, s.pr, m);
        const Stats a = stats(*s.pk, r, true), b = stats(*s.pk, r, false);
        std::printf("%-22s %4d %+7.2f %6.2f %6.2f %4d | %4d %+7.2f %6.2f %6.2f %4d\n", s.pk->name.c_str(), a.n, a.mean, a.rms, a.max_abs,
                    a.outside5, b.n, b.mean, b.rms, b.max_abs, b.outside5);
    }
}

} // namespace

TEST_CASE("KCB diag: knock model at measured BCs on the EPA turbo rows", "[.][diag][knock][knock_cal]") {
    const Model shipped;
    BcOpt o;
    o.delivered_air = false;
    o.exh_measured = false;
    prepare_sets(o);
    print_table("shipped model, BC: trapped=measured air, exhaust ambient", shipped);
    o.exh_measured = true;
    prepare_sets(o);
    print_table("shipped model, BC: trapped=measured air, exhaust measured", shipped);
    o.delivered_air = true;
    o.exh_measured = false;
    prepare_sets(o);
    print_table("shipped model, BC: delivered air, exhaust ambient", shipped);
    o.exh_measured = true;
    prepare_sets(o);
    print_table("shipped model, BC: delivered air, exhaust measured", shipped);
    // per-row dump for the shipped model: diff under the four BC variants, then the delivered/ambient state.
    std::vector<std::vector<std::vector<Res>>> var;
    std::vector<std::vector<Prepared>> keep;
    const BcOpt vs[4] = {{false, 0.0, false}, {true, 0.0, false}, {false, 0.0, true}, {true, 0.0, true}};
    for (const BcOpt& v : vs) {
        prepare_sets(v);
        std::vector<std::vector<Res>> per;
        for (const Set& s : sets()) per.push_back(eval_all(*s.pk, s.pr, shipped));
        var.push_back(std::move(per));
    }
    prepare_sets(vs[2]);
    std::printf("\nper envelope row: diff = theta_kl - spark for [trap/amb, trap/meas, deliv/amb, deliv/meas]; state of deliv/amb\n");
    for (std::size_t k = 0; k < sets().size(); ++k) {
        const Set& s = sets()[k];
        std::printf("\n-- %s (fuel %s RON %.1f MON %.1f)\n", s.pk->name.c_str(), s.pk->fuel.name.c_str(), s.pk->fuel.knock.ron, s.pk->fuel.knock.mon);
        for (std::size_t i = 0; i < s.pk->rows.size(); ++i) {
            const Row& w = s.pk->rows[i];
            if (!w.envelope) continue;
            std::printf("  r%-4d %5.0f map %6.1f exh %6.1f imt %5.1f lam %.3f spk %6.2f | %+6.1f %+6.1f %+6.1f %+6.1f | air %.3f te %.3f Tivc %5.1f x %.3f%s\n",
                        w.row, w.rpm, w.map_kpa, w.exh_kpa, w.imt_c, w.lambda, w.spark_btdc, var[0][k][i].diff, var[1][k][i].diff, var[2][k][i].diff,
                        var[3][k][i].diff, s.pr[i].air_ratio, s.pr[i].trap_eff, s.pr[i].t_ivc, s.pr[i].x_ivc, var[2][k][i].saturated ? " SAT" : "");
        }
    }
}

namespace {

// Combined rms of theta_kl - spark over the fit set: the A25A KV1-BC rows and the Honda L15B7 Tier 3 envelope rows.
// g_kv1bc_penalty: the fit is constrained to keep every A25A row inside the KV1-BC band (+-5 deg, minus 0.05 deg slack):
// a quadratic penalty on the excess is added to the objective (the reported rms is the plain rms).
bool g_kv1bc_penalty = false;
real fit_rms_obj(const Model& m);
real fit_rms(const Model& m, real* a25a_rms = nullptr, real* honda_rms = nullptr) {
    real sq = 0.0;
    int n = 0;
    for (const Set& s : sets()) {
        const bool a25a = s.pk->name == "a25a_kv1bc", honda = s.pk->name == "honda_l15b7_t3";
        if (!a25a && !honda) continue;
        const Stats st = stats(*s.pk, eval_all(*s.pk, s.pr, m), true);
        sq += st.rms * st.rms * st.n;
        n += st.n;
        if (a25a && a25a_rms) *a25a_rms = st.rms;
        if (honda && honda_rms) *honda_rms = st.rms;
    }
    return std::sqrt(sq / n);
}

real fit_rms_obj(const Model& m) {
    real pen = 0.0;
    for (const Set& s : sets()) {
        if (s.pk->name != "a25a_kv1bc") continue;
        const Stats st = stats(*s.pk, eval_all(*s.pk, s.pr, m), true);
        if (st.max_abs > 4.95) pen = 10.0 * (st.max_abs - 4.95) * (st.max_abs - 4.95) + 0.5 * (st.max_abs - 4.95);
    }
    return fit_rms(m) + pen;
}

// Golden-section search of log10(scale) for a fixed pressure exponent.
Model fit_scale(Model m, real lo, real hi) {
    const real g = 0.5 * (std::sqrt(5.0) - 1.0);
    real a = lo, b = hi;
    real c = b - g * (b - a), d = a + g * (b - a);
    auto f = [&](real x) {
        Model t = m;
        t.scale = std::pow(10.0, x);
        return g_kv1bc_penalty ? fit_rms_obj(t) : fit_rms(t);
    };
    real fc = f(c), fd = f(d);
    while (b - a > 0.002) {
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - g * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + g * (b - a);
            fd = f(d);
        }
    }
    m.scale = std::pow(10.0, 0.5 * (a + b));
    return m;
}

} // namespace

namespace {
void run_fit(bool exh_measured) {
    BcOpt o;
    o.delivered_air = true;
    o.exh_measured = exh_measured;
    prepare_sets(o);
    std::printf("BC: delivered air, exhaust %s\n", o.exh_measured ? "measured" : "ambient");
    {
        Model m;
        m = fit_scale(m, -1.0, 1.5);
        real ra = 0, rh = 0;
        const real r = fit_rms(m, &ra, &rh);
        std::printf("scale only (n 1.70): scale %.4f rms %.3f (A25A %.3f Honda %.3f)\n", m.scale, r, ra, rh);
    }
    Model best;
    real best_rms = 1e9;
    for (real n = 0.6; n <= 1.801; n += 0.1) {
        Model m;
        m.p_exp = n;
        // centre the scale search on the A25A pressure level: rate ~ p^n, p ~ 50 atm
        const real centre = std::log10(kV2Scale) + (n - ec::kDouaudEyzatPExponent) * std::log10(50.0);
        m = fit_scale(m, centre - 1.0, centre + 1.0);
        real ra = 0, rh = 0;
        const real r = fit_rms(m, &ra, &rh);
        std::printf("n %.2f: scale %.4f rms %.3f (A25A %.3f Honda %.3f)\n", n, m.scale, r, ra, rh);
        if (r < best_rms) {
            best_rms = r;
            best = m;
        }
    }
    print_table("best (scale, n)", best);
}

} // namespace

TEST_CASE("KCB diag: fit of the pre-factor and the pressure exponent", "[.][diag][knock][knock_cal]") { run_fit(false); }
TEST_CASE("KCB diag: fit with the measured exhaust pressure", "[.][diag][knock][knock_cal]") { run_fit(true); }

TEST_CASE("KCB diag: per-row scale scan", "[.][diag][knock][knock_cal]") {
    BcOpt o;
    prepare_sets(o);
    const real scales[] = {1.0, 1.26, 1.4, 1.58, 2.0, 3.0};
    for (const Set& s : sets()) {
        if (s.pk->name != "honda_l15b7_t3" && s.pk->name != "a25a_kv1bc") continue;
        std::vector<std::vector<Res>> rr;
        for (real sc : scales) {
            Model m;
            m.scale = sc;
            rr.push_back(eval_all(*s.pk, s.pr, m));
        }
        std::printf("-- %s\n", s.pk->name.c_str());
        for (std::size_t i = 0; i < s.pk->rows.size(); ++i) {
            if (!s.pk->rows[i].envelope) continue;
            std::printf("  r%-4d %5.0f map %6.1f spk %6.2f :", s.pk->rows[i].row, s.pk->rows[i].rpm, s.pk->rows[i].map_kpa, s.pk->rows[i].spark_btdc);
            for (std::size_t k = 0; k < rr.size(); ++k) std::printf(" %+6.1f%s", rr[k][i].diff, rr[k][i].saturated ? "S" : rr[k][i].limited ? " " : "U");
            std::printf("\n");
        }
    }
}

TEST_CASE("KCB diag: per-row residuals of a model", "[.][diag][knock][knock_cal]") {
    BcOpt o;
    o.exh_measured = false;
    prepare_sets(o);
    Model m;
    m.p_exp = 1.15;
    m.b_k = 5000.0;
    {
        const real centre = std::log10(kV2Scale) + (m.p_exp - ec::kDouaudEyzatPExponent) * std::log10(50.0) -
                            (m.b_k - 3800.0) / 850.0 * 0.4343;
        m = fit_scale(m, centre - 1.5, centre + 1.5);
    }
    print_table("per-row model", m);
    for (const Set& s : sets()) {
        const std::vector<Res> r = eval_all(*s.pk, s.pr, m);
        std::printf("-- %s\n", s.pk->name.c_str());
        for (std::size_t i = 0; i < s.pk->rows.size(); ++i) {
            const Row& w = s.pk->rows[i];
            if (!w.envelope) continue;
            std::printf("  %5.0f map %6.1f lam %.3f lamT %.3f Tivc %5.1f x %.3f spk %6.2f diff %+6.1f%s\n", w.rpm, w.map_kpa, w.lambda,
                        s.pr[i].kin.op.lambda, s.pr[i].t_ivc, s.pr[i].x_ivc, w.spark_btdc, r[i].diff, r[i].saturated ? " SAT" : r[i].limited ? "" : " UNL");
        }
    }
}

namespace {
void grid_nb(bool exh_measured) {
    BcOpt o;
    o.exh_measured = exh_measured;
    prepare_sets(o);
    std::printf("BC: delivered air, exhaust %s\n", exh_measured ? "measured" : "ambient");
    const real ns[] = {0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
    const real bs[] = {4200, 4600, 5000, 5400, 5800, 6400, 7000};
    std::printf("%5s %6s %8s %6s %6s %6s %5s %5s | holdout env rms:", "n", "B", "scale", "rms", "A25A", "Honda", "maxA", "maxH");
    for (const Set& s : sets()) std::printf(" %s", s.pk->name.c_str());
    std::printf("\n");
    for (real b : bs) {
        for (real n : ns) {
            Model m;
            m.p_exp = n;
            m.b_k = b;
            const real centre = std::log10(kV2Scale) + (n - ec::kDouaudEyzatPExponent) * std::log10(50.0) +
                                -(b - 3800.0) / 850.0 * 0.4343;
            m = fit_scale(m, centre - 1.5, centre + 1.5);
            real ra = 0, rh = 0;
            const real r = fit_rms(m, &ra, &rh);
            real maxa = 0, maxh = 0;
            std::printf("%5.2f %6.0f %8.4g %6.3f %6.3f %6.3f", n, b, m.scale, r, ra, rh);
            std::string tail;
            for (const Set& s : sets()) {
                const Stats st = stats(*s.pk, eval_all(*s.pk, s.pr, m), true);
                if (s.pk->name == "a25a_kv1bc") maxa = st.max_abs;
                if (s.pk->name == "honda_l15b7_t3") maxh = st.max_abs;
                char buf[32];
                std::snprintf(buf, sizeof buf, " %5.2f/%+5.1f", st.rms, st.mean);
                tail += buf;
            }
            std::printf(" %5.1f %5.1f |%s\n", maxa, maxh, tail.c_str());
        }
    }
}
} // namespace

TEST_CASE("KCB diag: grid over pressure exponent and activation temperature", "[.][diag][knock][knock_cal]") { grid_nb(false); }
TEST_CASE("KCB diag: grid over pressure exponent and activation temperature with the measured exhaust", "[.][diag][knock][knock_cal]") { grid_nb(true); }

namespace {
void fine_grid(bool delivered) {
    g_kv1bc_penalty = true;
    BcOpt o;
    o.exh_measured = false;
    o.delivered_air = delivered;
    prepare_sets(o);
    std::printf("%5s %6s %8s %6s %6s %6s %5s %5s %6s %6s\n", "n", "B", "scale", "rms", "A25A", "Honda", "maxA", "maxH", "meanA", "meanH");
    for (real b = 4000.0; b <= 6001.0; b += 200.0) {
        for (real n = 0.95; n <= 1.401; n += 0.05) {
            Model m;
            m.p_exp = n;
            m.b_k = b;
            const real centre = std::log10(kV2Scale) + (n - ec::kDouaudEyzatPExponent) * std::log10(50.0) -
                                (b - 3800.0) / 850.0 * 0.4343;
            m = fit_scale(m, centre - 1.5, centre + 1.5);
            real ra = 0, rh = 0;
            const real r = fit_rms(m, &ra, &rh);
            Stats sa, sh;
            for (const Set& s : sets()) {
                if (s.pk->name == "a25a_kv1bc") sa = stats(*s.pk, eval_all(*s.pk, s.pr, m), true);
                if (s.pk->name == "honda_l15b7_t3") sh = stats(*s.pk, eval_all(*s.pk, s.pr, m), true);
            }
            std::printf("%5.2f %6.0f %8.4g %6.3f %6.3f %6.3f %5.2f %5.2f %+6.2f %+6.2f\n", n, b, m.scale, r, ra, rh, sa.max_abs, sh.max_abs, sa.mean, sh.mean);
        }
    }
}
} // namespace

TEST_CASE("KCB diag: fine grid", "[.][diag][knock][knock_cal]") { fine_grid(true); }
TEST_CASE("KCB diag: fine grid with the trapped-air rule", "[.][diag][knock][knock_cal]") { fine_grid(false); }

TEST_CASE("KCB diag: chosen model and its sensitivities", "[.][diag][knock][knock_cal]") {
    Model m;
    m.p_exp = 1.05;
    m.b_k = 5800.0;
    m.scale = 0.012;
    struct V {
        const char* name;
        BcOpt o;
    };
    V vs[5];
    vs[0] = {"fit BC (trapped = measured air, exhaust ambient)", BcOpt{}};
    vs[0].o.exh_measured = false;
    vs[0].o.delivered_air = false;
    vs[1] = vs[0];
    vs[1].name = "exhaust = measured turbine inlet";
    vs[1].o.exh_measured = true;
    vs[2] = vs[0];
    vs[2].name = "delivered air (model trapping efficiency)";
    vs[2].o.delivered_air = true;
    vs[3] = vs[0];
    vs[3].name = "intake cam +10 deg (IVC 10 deg earlier)";
    vs[3].o.icam_shift = 10.0;
    vs[4] = vs[0];
    vs[4].name = "intake cam -10 deg (IVC 10 deg later)";
    vs[4].o.icam_shift = -10.0;
    for (const V& v : vs) {
        prepare_sets(v.o);
        print_table(v.name, m);
    }
    prepare_sets(vs[0].o);
    Model lit;
    lit.scale = 1.0;
    print_table("literature Douaud-Eyzat (scale 1, n 1.7, B 3800)", lit);
}

// Step 3 table: WOT dyno rows of every shipped knock-limited engine with the CURRENT compiled knock model (run before
// and after a model change). Full maps, own cache dir; the turbo hyper is the stepped dyno (15 s cap per point).
TEST_CASE("KCB diag: engine WOT table", "[.][diag][knock][knock_cal]") {
    jobs::JobSystem pool(10);
    const std::filesystem::path cache = std::filesystem::temp_directory_path() / "ps_kcb_engine_maps";
    std::filesystem::create_directories(cache);
    struct E { const char* file; std::vector<real> rpm; };
    const std::vector<E> engines = {
        {"hyper_v8_one1_audio.json", {2000, 3000, 4000, 5000, 6000, 7000, 7500, 8000}},
        {"demo_i4_pipes.json", {2000, 3000, 4000, 5000, 6000, 6500}},
        {"honda_k20a_dc5.json", {2000, 3000, 4000, 5000, 6000, 7000, 8000}},
        {"bmw_n52b30.json", {1500, 2500, 3500, 4500, 5500, 6500}},
        {"toyota_a25a_fks_gen.json", {2000, 3000, 4000, 5000, 6000}},
    };
    for (const E& e : engines) {
        io::EngineMapOptions o;
        o.quality = MapQuality::Full;
        o.pool = &pool;
        o.cache_dir = cache.string();
        const ComponentParams eng = io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/" + e.file, o);
        DynoSweepSpec spec;
        spec.rpm = e.rpm;
        const DynoResult r = run_dyno_sweep(eng, nullptr, spec, &pool);
        std::printf("== %s\n", e.file);
        std::printf("   rpm   torque_Nm  power_kW  knock_retard  spark  boost_bar  conv" "\n");
        for (const DynoPoint& p : r.points)
            std::printf("%6.0f  %9.1f  %8.1f  %12.2f  %5.1f  %9.3f  %d" "\n", p.rpm, p.torque_nm, p.power_kw, p.knock_retard_deg,
                        p.spark_deg, p.boost_bar, p.converged ? 1 : 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Knock model v3 (knock calibration phase B): asserted checks at MEASURED boundary conditions (fit_bc: trapped =
// measured air, measured IMT/lambda/cam, exhaust at ambient). Like KV1-BC the measured ECU spark is taken as the knock
// limit with margin 0 (it is the OEM's knock limit minus its own margin, so the calibrated model absorbs that margin).
// Declared band (before the asserted run): fitted engines - every A25A row within +-5 deg (KV1-BC), Honda L15B7 Tier 3
// envelope rms <= 3.0 deg and every row within +-6.5 deg; held out - envelope rms <= 3.5 deg and every row within
// +-6.5 deg per package.
namespace {
constexpr real kFitRmsHonda = 3.0;
constexpr real kBandMax = 6.5;
constexpr real kHoldRms = 3.5;

const Set* find_set(const std::string& name) {
    for (const Set& s : sets())
        if (s.pk->name == name) return &s;
    return nullptr;
}

void prepare_named(const std::vector<std::string>& names) {
    const BcOpt o = fit_bc();
    for (Set& s : sets())
        for (const std::string& n : names)
            if (s.pk->name == n) s.pr = prepare_all(*s.pk, o);
}

void check_package(const std::string& name, real rms_bound, real max_bound) {
    const Set* s = find_set(name);
    REQUIRE(s != nullptr);
    const Model m; // the compiled knock model v3
    const std::vector<Res> r = eval_all(*s->pk, s->pr, m);
    const Stats st = stats(*s->pk, r, true);
    std::printf("KCB %-22s n %2d mean %+6.2f rms %5.2f max %5.2f\n", name.c_str(), st.n, st.mean, st.rms, st.max_abs);
    INFO(name);
    CHECK(st.n >= 10);
    CHECK(st.rms <= rms_bound);
    CHECK(st.max_abs <= max_bound);
    for (std::size_t i = 0; i < r.size(); ++i) {
        if (!r[i].ok) continue;
        INFO("row " << s->pk->rows[i].row << " rpm " << s->pk->rows[i].rpm);
        CHECK_FALSE(r[i].saturated);
    }
}
} // namespace

TEST_CASE("KCB knock model file equals the compiled knock model v3", "[knock][knock_cal][io]") {
    const std::string path = std::string(PS_SOURCE_DIR) + "/data/knock_models/douaud_eyzat_v3.json";
    const io::KnockModelFile f = io::load_knock_model_json(path);
    const ec::KnockModelParams d;
    CHECK(f.version == static_cast<int>(ec::kKnockModelVersion));
    CHECK(f.prefactor_s == ec::kDouaudEyzatA_s);
    CHECK(f.octane_exponent == ec::kDouaudEyzatOnExponent);
    CHECK(f.params.octane_index_k == d.octane_index_k);
    CHECK(f.params.prefactor_scale == d.prefactor_scale);
    CHECK(f.params.pressure_exponent == d.pressure_exponent);
    CHECK(f.params.activation_k == d.activation_k);
    // The desc default is the same set (what the loader leaves in CycleModelDesc::knock.model).
    const CycleModelDesc c;
    CHECK(c.knock.model.prefactor_scale == d.prefactor_scale);
    CHECK(c.knock.model.pressure_exponent == d.pressure_exponent);
    CHECK(c.knock.model.activation_k == d.activation_k);
    CHECK(c.knock.model.octane_index_k == d.octane_index_k);
    // Rejections (matched by type only - TOOL-019 on the asan leg).
    std::ifstream in(path);
    json j;
    in >> j;
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "ps_kcb_knock_model_file";
    std::filesystem::create_directories(dir);
    auto rejects = [&](const char* what, const json& bad) {
        const std::string p = (dir / (std::string(what) + ".json")).string();
        {
            std::ofstream out(p);
            out << bad.dump();
        }
        INFO(what);
        CHECK_THROWS_AS(io::load_knock_model_json(p), std::runtime_error);
    };
    json a = j;
    a["extra"] = 1;
    rejects("unknown_key", a);
    json b = j;
    b["activation_k"].erase("source");
    rejects("no_source", b);
    json c2 = j;
    c2["version"] = 2;
    rejects("old_version", c2);
    json e = j;
    e["pressure_exponent"]["value"] = -1.0;
    rejects("negative", e);
    json g = j;
    g.erase("prefactor_scale");
    rejects("missing", g);
}

TEST_CASE("KCB fit: knock model v3 on the A25A and Honda L15B7 Tier 3 rows at measured boundary conditions", "[knock][knock_cal]") {
    prepare_named({"a25a_kv1bc", "honda_l15b7_t3"});
    // A25A: the KV1-BC band row by row (also the v3 fit's constraint).
    {
        const Set* s = find_set("a25a_kv1bc");
        REQUIRE(s != nullptr);
        const std::vector<Res> r = eval_all(*s->pk, s->pr, Model{});
        const Stats st = stats(*s->pk, r, true);
        std::printf("KCB %-22s n %2d mean %+6.2f rms %5.2f max %5.2f\n", "a25a_kv1bc", st.n, st.mean, st.rms, st.max_abs);
        CHECK(st.n == 17);
        for (std::size_t i = 0; i < r.size(); ++i) {
            INFO("A25A row " << s->pk->rows[i].row << " rpm " << s->pk->rows[i].rpm);
            CHECK(r[i].limited);
            CHECK_FALSE(r[i].saturated);
            CHECK(std::abs(r[i].diff) <= 5.0);
        }
    }
    check_package("honda_l15b7_t3", kFitRmsHonda, kBandMax);
    // The compiled set is the minimum of the constrained fit objective (rms over both sets plus the KV1-BC penalty):
    // a drifted constant or a changed replay that moves the optimum fails here.
    g_kv1bc_penalty = true;
    const Model m0;
    const real j0 = fit_rms_obj(m0);
    auto worse = [&](const char* what, const Model& m) {
        const real jm = fit_rms_obj(m);
        std::printf("KCB objective %.4f vs %s %.4f\n", j0, what, jm);
        INFO(what);
        CHECK(j0 < jm);
    };
    Model a = m0, b = m0, c = m0, d = m0, e = m0, f = m0;
    a.scale *= 0.9;
    b.scale *= 1.1;
    c.p_exp -= 0.05;
    d.p_exp += 0.05;
    e.b_k -= 200.0;
    f.b_k += 200.0;
    // Each neighbour gets its own best pre-factor (the scale and the shape parameters trade off).
    for (Model* m : {&c, &d, &e, &f}) m->scale = fit_scale(*m, std::log10(m0.scale) - 0.5, std::log10(m0.scale) + 0.5).scale;
    worse("scale x0.9", a);
    worse("scale x1.1", b);
    worse("n -0.05 (own best scale)", c);
    worse("n +0.05 (own best scale)", d);
    worse("B -200 K (own best scale)", e);
    worse("B +200 K (own best scale)", f);
    g_kv1bc_penalty = false;
}

TEST_CASE("KCB holdout: Honda L15B7 Tier 2 and Ford 2.7 EcoBoost at measured boundary conditions", "[knock][knock_cal]") {
    const std::vector<std::string> names = {"honda_l15b7_t2", "ford_27_t2_cell9", "ford_27_levIII_cell9", "ford_27_t2_hd1", "ford_27_t3_hd"};
    prepare_named(names);
    for (const std::string& n : names) check_package(n, kHoldRms, kBandMax);
}

TEST_CASE("KCB report: Ford 1.6 EcoBoost and Mazda 2.5 T at measured boundary conditions", "[knock][knock_cal][!mayfail]") {
    // Not fitted, no claim: Ford 1.6 (bore/stroke [AS]) and Mazda 2.5 T (cooled EGR not modelled) read 2.6-5.2 deg too
    // knock-resistant on average with knock model v3 - outside the held-out band.
    const std::vector<std::string> names = {"ford_16_t2", "ford_16_levIII", "mazda_25t_t2", "mazda_25t_t3"};
    prepare_named(names);
    for (const std::string& n : names) check_package(n, kHoldRms, kBandMax);
}
