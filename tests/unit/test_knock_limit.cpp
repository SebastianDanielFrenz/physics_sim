// tests/unit/test_knock_limit.cpp - engine garage P7 (C17b part 1): the knock limit theta_kl(rpm, p_im).
//
// What is checked (tags [knock][drivetrain][cycle]; windows declared in out/garage_p7_progress.md before the runs):
//   K1  physics_sim.fuel/1 "knock" block: the shipped values, the E25 molar blend rule recomputed, every rejection.
//   K2  the ECU opt-in (ignition_strategy / knock_margin_deg): rejections, a file theta_kl wins, MBT tracking, an engine
//       that does not opt in loads without any knock table.
//   K3  the generated tables (A25A, Full): non-increasing in p_im, higher octane never lower, E85 above E10, unlimited at
//       the lowest p_im row, the margin moves exactly the limited nodes.
//   K4  the closed-cycle replay: its own p_max against the cycle point, the integral rises with advance, theta_kl at
//       0.5 deg vs 0.125 deg replay steps.
//   K5  the Powertrain burns the tank fuel's table.
//   K6  the generator contract with the knock limit on: 1 vs 10 workers, zero allocation inside the jobs, progress and
//       cancel unchanged (no extra points).
//   K7  classify_engine_change: margin / octane / fuel order on an opted-in engine cost Maps, an octane edit on an engine
//       that does not opt in only Runtime.
//   K8  the dyno on a P5 turbo conversion (A25A CR 10 + single_medium): Knock findings, never more torque than knock off.
//   KV1 the A25A's knock limit against its measured WOT spark (EPA NVFEL Report 26781) - [!mayfail], see the case.
//
// Recorded sabotages: see out/garage_p7_progress.md "Sabotages" (each run once by hand, then reverted).
#include "drivetrain/engine_simulated.h"
#include "drivetrain/powertrain.h"

#include "cycle_fixtures.h"
#include "mvem_fixtures.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/knock.h"
#include "ps/engine_cycle/wiebe.h"
#include "ps/io/fuel_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_desc.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;
namespace da = ps::jobs::debug_alloc;

namespace {

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);
std::filesystem::path engines_dir() { return kSrc / "data" / "engines"; }
std::string fuel_path(const char* name) { return (kSrc / "data" / "fuels" / (std::string(name) + ".json")).generic_string(); }

jobs::JobSystem& pool() {
    static jobs::JobSystem p(10);
    return p;
}

struct TempDir {
    std::filesystem::path dir;
    explicit TempDir(const char* tag) {
        const auto t = std::chrono::steady_clock::now().time_since_epoch().count();
        dir = std::filesystem::temp_directory_path() / (std::string("ps_test_knock_") + tag + "_" + std::to_string(t));
        std::filesystem::create_directories(dir);
    }
    ~TempDir() {
        std::error_code e;
        std::filesystem::remove_all(dir, e);
    }
    std::string write(const std::string& name, const json& j) const {
        const auto p = dir / name;
        std::ofstream out(p);
        out << j.dump(2);
        return p.generic_string();
    }
};

// A user engine extending a shipped one (data/engines/<base>) with the knock opt-in. mbt: the cycle block's measured
// spark schedule removed, so the ECU tracks MBT (the knock limit is then the only spark limit).
json knock_engine(const char* base, real margin, bool mbt) {
    json j;
    j["extends"] = base;
    j["ecu"] = {{"ignition_strategy", "knock_limited"}, {"knock_margin_deg", margin}};
    if (mbt) j["cycle"] = {{"spark_advance_deg", nullptr}, {"spark_advance_deg_source", nullptr}};
    return j;
}

ComponentParams load_params(const json& j, MapQuality q = MapQuality::Full, bool generate = true) {
    io::EngineMapOptions o;
    o.quality = q;
    o.pool = &pool();
    o.generate = generate;
    return io::load_engine_json_from_string(j.dump(), engines_dir(), o);
}

SimulatedEngineDesc load_sim(const json& j, MapQuality q = MapQuality::Full, bool generate = true) {
    const ComponentParams p = load_params(j, q, generate);
    const auto* e = std::get_if<SimulatedEngineDesc>(&p);
    REQUIRE(e != nullptr);
    return *e;
}

std::string load_error(const json& j) {
    try {
        (void)load_params(j, MapQuality::Sketch, false);
    } catch (const std::exception& e) {
        return e.what();
    }
    return "<no exception>";
}

real kl_at(const Table2D& t, std::size_t ix, std::size_t iy) { return t.values[ix * t.y.size() + iy]; }

struct SparkRow {
    real rpm, map_kpa, spark_btdc, fuel_g_s, lambda, torque_nm;
};
// EPA NVFEL Report 26781 Tier 3 test-data workbook, sheet "High Load Final", rows 23-39 (literal readings; rpm, MAP,
// fuel flow, lambda and torque are the rows tests/validation/test_gen_engine_dyno.cpp pins, the spark advance the one its
// review diag reads). Spot-checked against the xlsx by a script (out/garage_p7_progress.md "KV1 transcription").
constexpr SparkRow kWotSpark[] = {
    {2017.13, 97.2897, -0.8834489, 3.81202, 0.79747, 216.321},  {2534.94, 97.3509, -0.1004669, 4.46601, 0.867008, 225.125},
    {2995.63, 97.2884, 3.6484548, 4.83102, 0.896868, 217.813},  {3571.24, 97.0007, 5.2851633, 6.09503, 0.879773, 228.4},
    {4032.22, 96.8321, 3.4596332, 7.28591, 0.859654, 228.347},  {4493.11, 96.6618, 3.3724168, 8.15611, 0.87221, 232.038},
    {4954.03, 96.4381, 4.3984367, 9.08713, 0.870633, 231.437},
};
constexpr std::size_t kKv1Rows = 4; // 2017-3571 rpm: the asserted band

// The generator's operating point at a node (map_generator.cpp make_op: ISA intake/ambient, lambda 1, the cam schedule).
ec::OperatingPoint node_op(const CycleModelDesc& c, real rpm, real p_pa, real spark_atdc) {
    ec::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p_pa;
    op.t_im_k = 288.15;
    op.p_amb_pa = 101325.0;
    op.lambda = 1.0;
    op.spark_deg_atdc = spark_atdc;
    op.intake_advance_deg = c.cam.intake_advance_deg.x.empty() ? 0.0 : c.cam.intake_advance_deg.value(rpm, p_pa);
    op.exhaust_retard_deg = c.cam.exhaust_retard_deg.x.empty() ? 0.0 : c.cam.exhaust_retard_deg.value(rpm, p_pa);
    return op;
}

real mbt_atdc_of(const CycleModelDesc& c) {
    return ec::Wiebe::theta0_for_ca50(c.wiebe_a, c.wiebe_m, c.burn_duration_deg, c.ca50_mbt_deg_atdc);
}

// test_engine_map_control.cpp's small generic fixture (4 x 3 grid) opted in with two knock fuels of low and high octane.
CycleModelDesc small_knock_desc() {
    CycleModelDesc d = ps::test::make_generic_i4_cycle_desc();
    d.grid_rpm = {1000.0, 2500.0, 4000.0, 5500.0};
    d.grid_p_im = {30000.0, 60000.0, 95000.0};
    d.friction.model = FrictionModelDesc::Model::PolynomialPistonSpeed;
    d.friction.a_pa = 66619.521802247167;
    d.friction.b_pa = 91.085076199249656;
    d.friction.c_pa = 346.80954153442133;
    d.knock.enabled = true;
    d.knock.margin_deg = 2.0;
    d.knock.fuels = {CycleModelDesc::KnockFuel{"low", 80.0, 72.0}, CycleModelDesc::KnockFuel{"high", 98.0, 88.0}};
    return d;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K1: fuel knock block - shipped values, the E25 blend rule, every rejection", "[knock][drivetrain][cycle][io]") {
    struct Shipped {
        const char* name;
        real ron, mon;
    };
    const Shipped shipped[] = {{"gasoline", 95.0, 85.0}, {"e5", 95.0, 85.0},     {"e10", 95.0, 85.0},
                               {"e25", 100.91, 87.04},  {"e85", 104.0, 88.0},    {"ethanol", 108.6, 89.7},
                               {"tier3_e10_cert", 92.0, 84.0}};
    for (const Shipped& s : shipped) {
        INFO(s.name);
        const FuelDesc f = io::load_fuel_json(fuel_path(s.name));
        REQUIRE(f.has_knock);
        CHECK(f.knock.model == FuelKnockModel::DouaudEyzat);
        CHECK(f.knock.ron == s.ron);
        CHECK(f.knock.mon == s.mon);
        CHECK_FALSE(f.knock.source.empty());
    }
    CHECK_FALSE(io::load_fuel_json(fuel_path("diesel")).has_knock); // a compression-ignition fuel has no knock block

    SECTION("E25 = the molar-linear rule over gasoline.json and ethanol.json (docs/data_formats/fuel.md)") {
        const FuelDesc g = io::load_fuel_json(fuel_path("gasoline"));
        const FuelDesc e = io::load_fuel_json(fuel_path("ethanol"));
        const real v = 0.25, m_e = 46.07, m_g = 100.0;
        const real ne = v * e.density_kg_m3 / m_e, ng = (1.0 - v) * g.density_kg_m3 / m_g;
        const real x = ne / (ne + ng);
        const FuelDesc b = io::load_fuel_json(fuel_path("e25"));
        CHECK(std::abs(b.knock.ron - (g.knock.ron + x * (e.knock.ron - g.knock.ron))) < 0.006); // the file rounds to 0.01
        CHECK(std::abs(b.knock.mon - (g.knock.mon + x * (e.knock.mon - g.knock.mon))) < 0.006);
    }
    SECTION("rejections") {
        TempDir td("k1");
        json base = json::parse(std::ifstream(fuel_path("gasoline")));
        auto reject = [&](const json& knock, const char* expected) {
            json j = base;
            j["knock"] = knock;
            const std::string p = td.write("f.json", j);
            CHECK_THROWS_WITH(io::load_fuel_json(p), ContainsSubstring(expected));
        };
        const json ok = base["knock"];
        reject(json::array(), "must be an object");
        json k = ok;
        k["octane"] = 95;
        reject(k, "unknown key 'octane'");
        k = ok;
        k["model"] = "kalghatgi";
        reject(k, "is not supported");
        k = ok;
        k.erase("model");
        reject(k, "'model' is required");
        k = ok;
        k.erase("ron");
        reject(k, "'ron' is required");
        k = ok;
        k["mon"] = "85";
        reject(k, "'mon' is required and must be a number");
        k = ok;
        k["ron"] = 131.0;
        reject(k, "must be in [40, 130]");
        k = ok;
        k["mon"] = 39.0;
        reject(k, "must be in [40, 130]");
        k = ok;
        k["mon"] = 96.0;
        reject(k, "must not exceed ron");
        k = ok;
        k.erase("source");
        reject(k, "'source' is required");
    }
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K2: the ECU opt-in - rejections, a file theta_kl wins, MBT tracking, no opt-in no table", "[knock][drivetrain][cycle][io]") {
    SECTION("an engine that does not opt in has no knock table and no knock in its cycle desc") {
        const SimulatedEngineDesc e = load_sim(json{{"extends", "toyota_a25a_fks_gen.json"}}, MapQuality::Sketch);
        CHECK_FALSE(e.maps.has_theta_kl);
        CHECK(e.maps.theta_kl_by_fuel.empty());
        CHECK_FALSE(e.cycle->knock.enabled);
        CHECK(e.cycle->knock.fuels.empty());
    }
    SECTION("rejections") {
        json j = knock_engine("toyota_a25a_fks_gen.json", 2.0, false);
        j["ecu"]["ignition_strategy"] = "adaptive";
        CHECK_THAT(load_error(j), ContainsSubstring("ignition_strategy must be \"schedule\" or \"knock_limited\""));
        j = json{{"extends", "toyota_a25a_fks_gen.json"}, {"ecu", {{"knock_margin_deg", 2.0}}}};
        CHECK_THAT(load_error(j), ContainsSubstring("'knock_margin_deg' needs ignition_strategy"));
        j = knock_engine("toyota_a25a_fks_gen.json", 2.0, false);
        j["ecu"].erase("knock_margin_deg");
        CHECK_THAT(load_error(j), ContainsSubstring("needs 'knock_margin_deg'"));
        j = knock_engine("toyota_a25a_fks_gen.json", 10.5, false);
        CHECK_THAT(load_error(j), ContainsSubstring("must be in [0, 10]"));
        j = knock_engine("toyota_a25a_fks_gen.json", -0.5, false);
        CHECK_THAT(load_error(j), ContainsSubstring("must be in [0, 10]"));
        j = knock_engine("toyota_a25a_fks_gen.json", 2.0, false);
        j["fuels"] = json::array({fuel_path("e10"), fuel_path("diesel")});
        CHECK_THAT(load_error(j), ContainsSubstring("fuel 'diesel' has none"));
        j = knock_engine("toyota_a25a_fks_sim.json", 2.0, false); // complete maps block, no cycle block
        CHECK_THAT(load_error(j), ContainsSubstring("needs a cycle block"));
        j = knock_engine("toyota_a25a_fks_sim.json", 2.0, false);
        j["ecu"]["spark_advance_deg"] = nullptr; // MBT tracking off the file's theta_MBT: still nothing to limit with
        CHECK_THAT(load_error(j), ContainsSubstring("needs a cycle block"));
    }
    SECTION("a file maps.theta_kl_deg wins: nothing is generated for knock, no per-fuel tables") {
        const json table = {{"rpm", {1000.0, 6000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 5.0}, {30.0, 8.0}}}};
        json j = knock_engine("toyota_a25a_fks_sim.json", 2.0, false);
        j["maps"] = {{"source", "[AS] test table"}, {"theta_kl_deg", table}};
        const SimulatedEngineDesc s = load_sim(j);
        REQUIRE(s.maps.has_theta_kl);
        CHECK(s.maps.theta_kl_by_fuel.empty());
        CHECK(s.maps.theta_kl_deg.values == std::vector<real>({30.0, 5.0, 30.0, 8.0}));
        j = knock_engine("toyota_a25a_fks_gen.json", 2.0, false);
        j["maps"] = {{"source", "[AS] test table"}, {"theta_kl_deg", table}};
        const SimulatedEngineDesc g = load_sim(j, MapQuality::Sketch);
        REQUIRE(g.maps.has_theta_kl);
        CHECK(g.maps.theta_kl_by_fuel.empty());
        CHECK(g.maps.theta_kl_deg.values == std::vector<real>({30.0, 5.0, 30.0, 8.0}));
        CHECK_FALSE(g.cycle->knock.enabled); // not a generator input, so not in the map key
    }
    SECTION("knock_limited without an ECU spark table: the ECU tracks the theta_MBT map; one generated table per fuel") {
        const SimulatedEngineDesc e = load_sim(knock_engine("toyota_a25a_fks_gen.json", 2.0, true), MapQuality::Sketch);
        REQUIRE(e.cycle->knock.enabled);
        CHECK(e.cycle->knock.margin_deg == 2.0);
        REQUIRE(e.cycle->knock.fuels.size() == 2);
        CHECK(e.cycle->knock.fuels[0].name == "tier3_e10_cert");
        CHECK(e.cycle->knock.fuels[1].ron == 95.0);
        CHECK(e.ecu.spark_advance_deg.values == e.maps.theta_mbt_deg.values);
        CHECK(e.ecu.spark_advance_deg.x == e.maps.theta_mbt_deg.x);
        REQUIRE(e.maps.has_theta_kl);
        REQUIRE(e.maps.theta_kl_by_fuel.size() == 2);
        CHECK(e.maps.theta_kl_deg.values == e.maps.theta_kl_by_fuel[0].values);
        CHECK(e.maps.theta_kl_by_fuel[0].x == e.maps.ve.x); // the generated grid (Sketch: reduced)
        CHECK(e.maps.theta_kl_by_fuel[0].y == e.maps.ve.y);
    }
    SECTION("knock_limited with the measured schedule: the ECU keeps the schedule (spark = min(schedule, theta_kl) at runtime)") {
        const SimulatedEngineDesc shipped = load_sim(json{{"extends", "toyota_a25a_fks_gen.json"}}, MapQuality::Sketch, false);
        const SimulatedEngineDesc e = load_sim(knock_engine("toyota_a25a_fks_gen.json", 2.0, false), MapQuality::Sketch, false);
        CHECK(e.ecu.spark_advance_deg.values == shipped.ecu.spark_advance_deg.values);
        CHECK(e.maps_pending);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K3: generated A25A tables - monotone in p_im, octane order, unlimited at low load, the margin", "[knock][drivetrain][cycle]") {
    json j = knock_engine("toyota_a25a_fks_gen.json", 0.0, true);
    j["fuels"] = json::array({fuel_path("tier3_e10_cert"), fuel_path("e10"), fuel_path("e85")});
    const SimulatedEngineDesc e = load_sim(j);
    REQUIRE(e.maps.theta_kl_by_fuel.size() == 3);
    const Table2D& t3 = e.maps.theta_kl_by_fuel[0];
    const Table2D& e10 = e.maps.theta_kl_by_fuel[1];
    const Table2D& e85 = e.maps.theta_kl_by_fuel[2];
    constexpr real kTol = 0.1; // two independent 0.05 deg bisections
    std::size_t limited = 0, e85_gain = 0;
    for (std::size_t ix = 0; ix < t3.x.size(); ++ix) {
        for (std::size_t iy = 0; iy < t3.y.size(); ++iy) {
            INFO("rpm " << t3.x[ix] << " p_im " << t3.y[iy]);
            if (iy > 0) {
                CHECK(kl_at(t3, ix, iy) <= kl_at(t3, ix, iy - 1) + kTol);
                CHECK(kl_at(e10, ix, iy) <= kl_at(e10, ix, iy - 1) + kTol);
                CHECK(kl_at(e85, ix, iy) <= kl_at(e85, ix, iy - 1) + kTol);
            }
            CHECK(kl_at(e10, ix, iy) >= kl_at(t3, ix, iy) - kTol);
            CHECK(kl_at(e85, ix, iy) >= kl_at(e10, ix, iy) - kTol);
            const real mbt = e.maps.theta_mbt_deg.value(t3.x[ix], t3.y[iy]);
            if (kl_at(t3, ix, iy) < mbt + ec::kKnockSearchAboveMbtDeg) ++limited;
            if (kl_at(e85, ix, iy) > kl_at(e10, ix, iy) + 1.0) ++e85_gain;
        }
        CHECK(kl_at(t3, ix, 0) >= e.maps.theta_mbt_deg.value(t3.x[ix], t3.y[0])); // 20 kPa: never knock-limited
    }
    std::printf("K3: %zu of %zu tier3 nodes limited, E85 > E10 + 1 deg at %zu nodes\n", limited, t3.values.size(), e85_gain);
    CHECK(limited > 0);
    CHECK(e85_gain > 0);

    SECTION("the margin lowers exactly the limited nodes by itself (Sketch)") {
        const SimulatedEngineDesc a = load_sim(knock_engine("toyota_a25a_fks_gen.json", 0.0, true), MapQuality::Sketch);
        const SimulatedEngineDesc b = load_sim(knock_engine("toyota_a25a_fks_gen.json", 3.0, true), MapQuality::Sketch);
        const Table2D& ta = a.maps.theta_kl_deg;
        const Table2D& tb = b.maps.theta_kl_deg;
        REQUIRE(ta.values.size() == tb.values.size());
        for (std::size_t k = 0; k < ta.values.size(); ++k) {
            if (ta.values[k] > real(-30.0) + 3.0) { // margin 3 does not reach the -30 deg clamp
                const bool unlimited = ta.values[k] == tb.values[k];
                CHECK((unlimited || std::abs(ta.values[k] - tb.values[k] - 3.0) < 1e-12));
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K4: the closed-cycle replay - p_max of the cycle, the integral rises with advance, step convergence", "[knock][drivetrain][cycle]") {
    const SimulatedEngineDesc e = load_sim(knock_engine("toyota_a25a_fks_gen.json", 0.0, true), MapQuality::Sketch, false);
    const CycleModelDesc& c = *e.cycle;
    const FuelDesc& f = e.fuels[0];
    const ec::CycleStepper proto(c, f.lhv_j_kg, f.afr_stoich);
    const ec::StepperSettings st{ec::kStepFull, ec::kCyclesFull};
    const real mbt_atdc = mbt_atdc_of(c);
    const real mbt_btdc = -(mbt_atdc - c.ignition_delay_deg);
    for (std::size_t i = 0; i < kKv1Rows; ++i) {
        const SparkRow& r = kWotSpark[i];
        INFO("rpm " << r.rpm);
        const ec::OperatingPoint op = node_op(c, r.rpm, r.map_kpa * 1000.0, mbt_atdc);
        const ec::PointResult pr = ec::run_operating_point(proto, op, st);
        ec::KnockReplayInput in;
        in.proto = &proto;
        in.op = op;
        in.ivc = pr.ivc;
        in.ignition_delay_deg = c.ignition_delay_deg;
        in.burn_duration_deg = c.burn_duration_deg;
        in.inv_on_term = ec::douaud_eyzat_inv_on_term(ec::octane_index(f.knock.ron, f.knock.mon));
        in.fuel_mass_fraction = 1.0 / (1.0 + f.afr_stoich);
        in.stop_at_knock = false;
        in.end_burn_fraction = 0.999; // past the pressure peak, for the p_max comparison
        in.step_deg = 0.125;
        const ec::KnockReplayResult own = ec::knock_replay(in, mbt_btdc);
        std::printf("K4 %6.0f rpm: replay p_max %.4f bar vs cycle %.4f bar\n", r.rpm, own.p_max_pa / 1e5, pr.p_max_pa / 1e5);
        CHECK(std::abs(own.p_max_pa / pr.p_max_pa - 1.0) < 0.015);

        in.end_burn_fraction = ec::kKnockEndBurnFraction;
        in.step_deg = ec::kKnockReplayStepDeg;

        // The end gas: an independent RK4 integration of dT/dln(p) = R_m*T/cp_m(T) for air + fuel vapour + residual
        // (the GasPropertiesDesc defaults, iso-octane vapour cp linear through 188.9 J/(mol K) at 298.15 K and
        // 425 J/(mol K) at 900 K, M 114.23 g/mol) from the IVC state to the replay's end pressure.
        {
            const ec::KnockReplayResult end = ec::knock_replay(in, mbt_btdc);
            const real v0 = proto.model().geom.eval(pr.ivc.crank_deg).v;
            const ec::CvState s0 = ec::cv_state(proto.model().gas, pr.ivc.m, pr.ivc.u, pr.ivc.mb, v0);
            const GasPropertiesDesc g{};
            const real rv = 8314.462618 / 114.23;
            const real cp298 = 188.9 / 0.11423, cp900 = 425.0 / 0.11423;
            const real ff = in.fuel_mass_fraction, x = s0.x;
            auto cv = [&](real t) {
                const real air = g.fresh.cv_a_j_kgk + g.fresh.cv_b_j_kgk2 * t;
                const real vap = cp298 + (cp900 - cp298) * (t - 298.15) / (900.0 - 298.15) - rv;
                const real brn = g.burned.cv_a_j_kgk + g.burned.cv_b_j_kgk2 * t;
                return (1.0 - x) * ((1.0 - ff) * air + ff * vap) + x * brn;
            };
            const real rm = (1.0 - x) * ((1.0 - ff) * g.fresh.r_j_kgk + ff * rv) + x * g.burned.r_j_kgk;
            auto dtdl = [&](real t) { return rm * t / (cv(t) + rm); };
            const int n = 4000;
            const real dl = std::log(end.p_end_pa / s0.p) / n;
            real t = s0.t;
            for (int k = 0; k < n; ++k) {
                const real k1 = dtdl(t), k2 = dtdl(t + 0.5 * dl * k1), k3 = dtdl(t + 0.5 * dl * k2), k4 = dtdl(t + dl * k3);
                t += dl * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
            }
            ec::KnockReplayInput air_only = in;
            air_only.fuel_mass_fraction = 0.0;
            const real t_air = ec::knock_replay(air_only, mbt_btdc).t_end_gas_k;
            std::printf("K4 %6.0f rpm: end gas %.2f K at %.1f bar (independent %.2f K; air only %.2f K)\n", r.rpm, end.t_end_gas_k,
                        end.p_end_pa / 1e5, t, t_air);
            CHECK(std::abs(end.t_end_gas_k - t) < 0.5);
            CHECK(t_air - end.t_end_gas_k > 50.0);
        }
        real prev = -1.0;
        for (real sp = -10.0; sp <= 30.0; sp += 5.0) {
            const real integral = ec::knock_replay(in, sp).integral;
            CHECK(integral > prev);
            prev = integral;
        }
        in.stop_at_knock = true;
        const ec::KnockLimitResult coarse = ec::knock_limited_spark(in, mbt_btdc, 0.0);
        in.step_deg = 0.125;
        const ec::KnockLimitResult fine = ec::knock_limited_spark(in, mbt_btdc, 0.0);
        std::printf("K4 %6.0f rpm: theta_kl %.3f (0.5 deg) vs %.3f (0.125 deg), %d / %d replays\n", r.rpm, coarse.theta_kl_btdc,
                    fine.theta_kl_btdc, coarse.replays, fine.replays);
        CHECK(std::abs(coarse.theta_kl_btdc - fine.theta_kl_btdc) < 0.5);
        CHECK(coarse.limited);
        CHECK_FALSE(coarse.saturated);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K5: the Powertrain burns the tank fuel's knock table", "[knock][drivetrain][cycle]") {
    TempDir td("k5");
    json eng = knock_engine((engines_dir() / "toyota_a25a_fks_gen.json").generic_string().c_str(), 2.0, true);
    eng["fuels"] = json::array({fuel_path("e10"), fuel_path("e85")});
    const std::string eng_path = td.write("engine.json", eng);
    // car_sedan_gen.json with every relative path made absolute, the engine above and (optionally) a fuel tank.
    const std::filesystem::path vdir = kSrc / "data" / "vehicles";
    json v = json::parse(std::ifstream(vdir / "car_sedan_gen.json"));
    std::function<void(json&)> absolutise = [&](json& n) {
        if (n.is_object() || n.is_array()) {
            for (auto& c : n) absolutise(c);
        } else if (n.is_string()) {
            const std::string s = n.get<std::string>();
            if (s.rfind("../", 0) == 0) n = (vdir / s).lexically_normal().generic_string();
        }
    };
    absolutise(v);
    for (auto& c : v["powertrain"]["components"]) {
        if (c.value("id", "") == "engine") c["ref"] = eng_path;
    }
    auto build = [&](const char* tank_fuel) {
        json vv = v;
        if (tank_fuel != nullptr) {
            vv["powertrain"]["components"].push_back(
                {{"id", "tank"}, {"type", "fuel_tank"}, {"fuel", fuel_path(tank_fuel)}, {"capacity_kg", 40.0}, {"initial_kg", 30.0}});
            vv["powertrain"]["fluid"] = json::array({json::array({"tank.out", "engine.fuel"})});
        }
        io::VehicleLoadOptions o;
        o.maps.quality = MapQuality::Sketch;
        o.maps.pool = &pool();
        return io::load_vehicle_json(td.write("vehicle.json", vv), o);
    };
    for (const char* tank : {static_cast<const char*>(nullptr), "e10", "e85"}) {
        INFO("tank " << (tank ? tank : "none"));
        const vehicle::VehicleDesc d = build(tank);
        const auto* sim = std::get_if<SimulatedEngineDesc>(&d.powertrain.components[0].params);
        REQUIRE(sim != nullptr);
        REQUIRE(sim->maps.theta_kl_by_fuel.size() == 2);
        CHECK(sim->maps.theta_kl_by_fuel[0].values != sim->maps.theta_kl_by_fuel[1].values);
        const Powertrain pt(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name);
        const std::size_t expect = (tank != nullptr && std::string(tank) == "e85") ? 1 : 0;
        CHECK(pt.burnt_fuel_index() == expect);
        REQUIRE(pt.simulated_engine() != nullptr);
        CHECK(pt.simulated_engine()->maps().maps.theta_kl_deg.values == sim->maps.theta_kl_by_fuel[expect].values);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K6: the generator contract with the knock limit - workers, allocation, progress, cancel", "[knock][drivetrain][cycle]") {
    const CycleModelDesc d = small_knock_desc();
    const FuelDesc fuel = ps::test::mvem::gasoline();
    CycleModelDesc plain = d;
    plain.knock = {};
    jobs::JobSystem one(1);
    for (MapQuality q : {MapQuality::Sketch, MapQuality::Preview}) {
        INFO("quality " << int(q));
        const EngineMapsPtr a = generate_engine_maps(d, fuel, q, &one);
        const EngineMapsPtr b = generate_engine_maps(d, fuel, q, &pool());
        CHECK(a->content_hash == b->content_hash);
        CHECK(a->maps.theta_kl_by_fuel[0].values == b->maps.theta_kl_by_fuel[0].values);
        CHECK(a->maps.theta_kl_by_fuel[1].values == b->maps.theta_kl_by_fuel[1].values);
        CHECK(a->knock_limited_points == b->knock_limited_points);
        CHECK(a->knock_limited_points > 0);
        std::printf("K6 quality %d: limited %d saturated %d replays %ld of %zu node-fuels\n", int(q), a->knock_limited_points,
                    a->knock_saturated_points, a->knock_replays, a->maps.theta_kl_deg.values.size() * 2);
        // the knock limit changes nothing else
        const EngineMapsPtr p = generate_engine_maps(plain, fuel, q, &pool());
        CHECK(p->maps.ve.values == b->maps.ve.values);
        CHECK(p->maps.eta_i.values == b->maps.eta_i.values);
        CHECK(p->maps.theta_mbt_deg.values == b->maps.theta_mbt_deg.values);
        CHECK(p->content_hash != b->content_hash);
        CHECK_FALSE(p->maps.has_theta_kl);
    }
    SECTION("progress: the same points as without the knock limit; cancel throws") {
        std::size_t with = 0, without = 0;
        MapGenerationControl c;
        c.on_progress = [&](std::size_t, std::size_t) { ++with; };
        (void)generate_engine_maps(d, fuel, MapQuality::Preview, &pool(), &c);
        c.on_progress = [&](std::size_t, std::size_t) { ++without; };
        (void)generate_engine_maps(plain, fuel, MapQuality::Preview, &pool(), &c);
        CHECK(with == without);
        CHECK(with > 0);
        std::atomic<bool> cancel{true};
        MapGenerationControl k;
        k.cancel = &cancel;
        CHECK_THROWS_AS(generate_engine_maps(d, fuel, MapQuality::Preview, &pool(), &k), GenerationCancelled);
    }
    SECTION("zero allocation inside the generation jobs (the replays copy the stepper on the stack)") {
        if (!da::hook_available()) {
            WARN("debug_alloc hook unavailable in this build - allocation check skipped");
        } else {
            jobs::JobSystem four(4);
            da::reset();
            da::set_armed(true);
            const EngineMapsPtr m = generate_engine_maps(d, fuel, MapQuality::Preview, &four);
            da::set_armed(false);
            std::printf("K6 own violations with the knock limit: %llu\n", static_cast<unsigned long long>(da::own_violation_count()));
            CHECK(da::own_violation_count() == 0);
            CHECK(m->knock_replays > 0);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K7: classify_engine_change on a knock-limited engine", "[knock][drivetrain][cycle]") {
    const SimulatedEngineDesc a = load_sim(knock_engine("toyota_a25a_fks_gen.json", 2.0, false), MapQuality::Full, false);
    SimulatedEngineDesc b = a;
    b.cycle->knock.margin_deg = 3.0;
    CHECK(classify_engine_change(a, b) == EngineChange::Maps);
    b = a;
    b.fuels[1].knock.ron += 1.0; // the second listed fuel's octane: its table is generated too
    CHECK(classify_engine_change(a, b) == EngineChange::Maps);
    b = a;
    std::swap(b.fuels[0], b.fuels[1]);
    CHECK(classify_engine_change(a, b) == EngineChange::Maps);

    const SimulatedEngineDesc n = load_sim(json{{"extends", "toyota_a25a_fks_gen.json"}}, MapQuality::Full, false);
    SimulatedEngineDesc m = n;
    m.fuels[1].knock.ron += 1.0; // no opt-in: the octane is no generator input
    CHECK(classify_engine_change(n, m) == EngineChange::Runtime);
    m = n;
    m.fuels[0].has_knock = false;
    CHECK(classify_engine_change(n, m) == EngineChange::Runtime);
}

// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("P7 K8: the dyno on a P5 turbo conversion - Knock findings, never more torque than knock off", "[knock][drivetrain][cycle][turbo]") {
    // A25A CR 10 + single_medium (FI1's conversion) with the ECU tracking MBT under a 2 deg knock margin.
    const SimulatedEngineDesc stock = load_sim(json{{"extends", "toyota_a25a_fks_gen.json"}}, MapQuality::Preview, false);
    const TurboPairDesc tp = io::load_turbo_configuration_json((kSrc / "data" / "turbo_configurations" / "single_medium.json").generic_string());
    const std::vector<real> grid = boost_grid_p_im(*stock.cycle, tp.target_boost_pa / 1e5);
    json j = knock_engine("toyota_a25a_fks_gen.json", 2.0, true);
    std::vector<real> grid_kpa;
    for (real p : grid) grid_kpa.push_back(p / 1000.0);
    j["cycle"]["grid_p_im_kpa"] = grid_kpa;
    j["cycle"]["compression_ratio"] = 10.0;
    io::EngineMapOptions o;
    o.quality = MapQuality::Preview;
    o.pool = &pool();
    io::EngineSlotOverrides slots;
    slots["forced_induction"] = (kSrc / "data" / "turbo_configurations" / "single_medium.json").generic_string();
    const ComponentParams on = io::load_engine_json_from_string(j.dump(), engines_dir(), o, slots);
    const SimulatedEngineDesc& e = std::get<SimulatedEngineDesc>(on);
    REQUIRE(e.turbo_pair.has_value());
    REQUIRE(e.maps.has_theta_kl);
    ComponentParams off = on;
    std::get<SimulatedEngineDesc>(off).maps.has_theta_kl = false;

    DynoSweepSpec spec;
    spec.rpm = {2500.0, 4000.0, 5500.0};
    spec.part_files.fuel = fuel_path("tier3_e10_cert");
    const DynoResult r_on = run_dyno_sweep(on, nullptr, spec, &pool());
    const DynoResult r_off = run_dyno_sweep(off, nullptr, spec, &pool());
    REQUIRE(r_on.points.size() == r_off.points.size());
    for (std::size_t i = 0; i < r_on.points.size(); ++i) {
        const DynoPoint& a = r_on.points[i];
        const DynoPoint& b = r_off.points[i];
        std::printf("K8 %6.0f rpm: knock on %7.1f N*m (spark %6.2f, retard %5.2f, boost %.3f bar) | off %7.1f N*m (spark %6.2f, boost %.3f)\n",
                    a.rpm, a.torque_nm, a.spark_deg, a.knock_retard_deg, a.boost_bar, b.torque_nm, b.spark_deg, b.boost_bar);
        CHECK(a.torque_nm <= b.torque_nm * 1.005);
        CHECK(b.knock_retard_deg == 0.0);
    }
    std::size_t knock = 0;
    for (const LimitFinding& f : r_on.limits) {
        if (f.kind != LimitKind::Knock) continue;
        ++knock;
        std::printf("K8 Knock %s @%.0f: retard %.2f deg (limit %.1f), part %s %s\n", to_string(f.severity), f.rpm, f.value, f.limit,
                    f.part.file.c_str(), f.part.field.c_str());
        CHECK(f.unit == LimitUnit::CrankDegree);
        CHECK(f.part.slot == "fuel");
        CHECK(f.part.file == spec.part_files.fuel);
        CHECK(f.part.field == "/knock/ron");
        REQUIRE(f.remedy.has_value());
        CHECK(f.remedy->cost == RemedyCost::Simulated);
        CHECK_FALSE(f.remedy->solved);
        CHECK(f.remedy->current == 92.0);
    }
    CHECK(knock >= 1);
    for (const LimitFinding& f : r_off.limits) CHECK(f.kind != LimitKind::Knock);
}

// ---------------------------------------------------------------------------------------------------------------------
// KV1 - the generated A25A's knock limit (tier3_e10_cert, margin 0, MBT tracking) against the spark its ECU actually ran
// at WOT. Band declared before the asserted run: theta_kl - spark_meas in [-5, +10] deg at 2017-3571 rpm.
//
// [!mayfail]: the model is MORE knock-prone than the engine - theta_kl lies 4-13 deg below the measured spark at WOT
// (out/garage_p7_progress.md). Discriminating measurement ("P7 diag: knock replay internals"): replayed from an IVC
// state scaled to the MEASURED trapped air (the generated A25A over-traps 11-15 % at 3000-4500 rpm, the [AS] runner),
// the Livengood-Wu integral at the measured spark is still 1.19-1.40 (> 1, knock) at every WOT row - so the trapped-air
// excess explains part of the 3000-3571 rpm gap, the rest is the uncalibrated Douaud-Eyzat correlation / single-zone
// end gas, a model limit (no constant was tuned to this data, D1).
TEST_CASE("P7 KV1: A25A knock limit vs measured WOT spark (2017-3571 rpm)", "[knock][drivetrain][cycle][!mayfail]") {
    const SimulatedEngineDesc e = load_sim(knock_engine("toyota_a25a_fks_gen.json", 0.0, true));
    REQUIRE(e.fuels[0].name == "tier3_e10_cert");
    const Table2D& kl = e.maps.theta_kl_by_fuel[0];
    SimulatedEngineDesc off = e;
    off.maps.has_theta_kl = false;
    const SimulatedEngineDesc shipped = load_sim(json{{"extends", "toyota_a25a_fks_gen.json"}});
    const auto m_on = SimulatedEngine::build_model_maps(e);
    const auto m_off = SimulatedEngine::build_model_maps(off);
    const auto m_shipped = SimulatedEngine::build_model_maps(shipped);
    const AmbientSample isa{};
    std::printf("KV1   rpm  spark_meas theta_kl  diff | WOT torque: measured  knock-on  knock-off(MBT)  shipped(schedule)\n");
    for (std::size_t i = 0; i < std::size(kWotSpark); ++i) {
        const SparkRow& r = kWotSpark[i];
        const real t = kl.value(r.rpm, r.map_kpa * 1000.0);
        const auto on = SimulatedEngine::steady_point(*m_on, r.rpm, 1.0, isa);
        const auto of = SimulatedEngine::steady_point(*m_off, r.rpm, 1.0, isa);
        const auto sh = SimulatedEngine::steady_point(*m_shipped, r.rpm, 1.0, isa);
        std::printf("KV1 %6.0f %8.2f %8.2f %6.2f | %8.1f %9.1f %9.1f %12.1f\n", r.rpm, r.spark_btdc, t, t - r.spark_btdc, r.torque_nm,
                    on.net_torque_nm, of.net_torque_nm, sh.net_torque_nm);
        CHECK(on.net_torque_nm <= of.net_torque_nm + 1e-9);
        if (i < kKv1Rows) {
            INFO("rpm " << r.rpm << " theta_kl " << t << " measured " << r.spark_btdc);
            CHECK(t - r.spark_btdc >= -5.0);
            CHECK(t - r.spark_btdc <= 10.0);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Diagnostics (hidden).
TEST_CASE("P7 diag: generated A25A knock limit vs measured WOT spark", "[.][diag][knock][drivetrain][cycle]") {
    const json j = knock_engine("toyota_a25a_fks_gen.json", 0.0, true);
    const SimulatedEngineDesc e = load_sim(j);
    for (std::size_t f = 0; f < e.maps.theta_kl_by_fuel.size(); ++f) {
        const Table2D& t = e.maps.theta_kl_by_fuel[f];
        std::printf("fuel %s (RON %.1f MON %.1f)\n", e.fuels[f].name.c_str(), e.fuels[f].knock.ron, e.fuels[f].knock.mon);
        std::printf("  rpm \\ p_im:");
        for (real p : t.y) std::printf(" %6.1f", p / 1000.0);
        std::printf("\n");
        for (std::size_t ix = 0; ix < t.x.size(); ++ix) {
            std::printf("  %6.0f    :", t.x[ix]);
            for (std::size_t iy = 0; iy < t.y.size(); ++iy) std::printf(" %6.2f", kl_at(t, ix, iy));
            std::printf("\n");
        }
        std::printf("  WOT rows: rpm map spark_meas theta_kl theta_mbt\n");
        for (const SparkRow& r : kWotSpark) {
            std::printf("  %7.1f %6.2f %7.2f %7.2f %7.2f\n", r.rpm, r.map_kpa, r.spark_btdc, t.value(r.rpm, r.map_kpa * 1000.0),
                        e.maps.theta_mbt_deg.value(r.rpm, r.map_kpa * 1000.0));
        }
    }
}

TEST_CASE("P7 diag: knock replay internals at the A25A WOT anchors", "[.][diag][knock][drivetrain][cycle]") {
    const SimulatedEngineDesc e = load_sim(knock_engine("toyota_a25a_fks_gen.json", 0.0, true), MapQuality::Sketch, false);
    const CycleModelDesc& c = *e.cycle;
    const FuelDesc& f = e.fuels[0];
    const ec::CycleStepper proto(c, f.lhv_j_kg, f.afr_stoich);
    const ec::StepperSettings st{ec::kStepFull, ec::kCyclesFull};
    const real mbt_atdc = mbt_atdc_of(c);
    const real mbt_btdc = -(mbt_atdc - c.ignition_delay_deg);
    const real inv_on = ec::douaud_eyzat_inv_on_term(ec::octane_index(f.knock.ron, f.knock.mon));
    std::printf("MBT %.2f BTDC, fuel %s OI %.1f\n", mbt_btdc, f.name.c_str(), ec::octane_index(f.knock.ron, f.knock.mon));
    std::printf("  rpm   lambda | ivc_deg T_ivc p_ivc[kPa] x_ivc | spark: integral T_eg_end p_max[bar]\n");
    for (const SparkRow& r : kWotSpark) {
        for (real lam : {1.0, 0.8}) {
            ec::OperatingPoint op = node_op(c, r.rpm, r.map_kpa * 1000.0, mbt_atdc);
            op.lambda = lam;
            const ec::PointResult pr = ec::run_operating_point(proto, op, st);
            ec::KnockReplayInput kin;
            kin.proto = &proto;
            kin.op = op;
            kin.ivc = pr.ivc;
            kin.ignition_delay_deg = c.ignition_delay_deg;
            kin.burn_duration_deg = c.burn_duration_deg;
            kin.inv_on_term = inv_on;
            kin.stop_at_knock = false;
            kin.fuel_mass_fraction = 1.0 / (1.0 + lam * f.afr_stoich);
            ec::KnockReplayInput air = kin;
            air.fuel_mass_fraction = 0.0;
            const real v = proto.model().geom.eval(pr.ivc.crank_deg).v;
            const ec::CvState c0 = ec::cv_state(proto.model().gas, pr.ivc.m, pr.ivc.u, pr.ivc.mb, v);
            std::printf("%6.0f %5.2f | %6.1f %6.1f %7.1f %.3f |", r.rpm, lam, pr.ivc.crank_deg, c0.t, c0.p / 1000.0, c0.x);
            for (real sp : {r.spark_btdc, real(5.0), mbt_btdc}) {
                const ec::KnockReplayResult kr = ec::knock_replay(kin, sp);
                std::printf("  %5.1f: %6.2f %5.0f %5.1f", sp, kr.integral, kr.t_end_gas_k, kr.p_max_pa / 1e5);
            }
            std::printf("\n        air-only end gas: integral at the measured spark %.2f\n", ec::knock_replay(air, r.spark_btdc).integral);
            if (lam != 1.0) continue;
            // Discriminating measurement: the replay from an IVC state scaled to the MEASURED air per cylinder-cycle (fuel
            // flow x lambda x AFR) at the same temperature and composition.
            const real afr = f.afr_stoich;
            const real air_model = (pr.ivc.m - pr.ivc.mb) * afr / (afr + 1.0);
            const real air_meas = r.fuel_g_s * 1e-3 * r.lambda * afr / (r.rpm / 120.0 * real(c.cylinders));
            const real ratio = air_meas / air_model;
            ec::KnockReplayInput ks = kin;
            ks.ivc.m *= ratio;
            ks.ivc.u *= ratio;
            ks.ivc.mb *= ratio;
            std::printf("        air model %.4f g, measured %.4f g (x%.3f) -> integral at the measured spark %.2f\n", air_model * 1e3,
                        air_meas * 1e3, ratio, ec::knock_replay(ks, r.spark_btdc).integral);
            for (real xb : {0.8, 0.95}) {
                ec::KnockReplayInput kx = kin;
                kx.end_burn_fraction = xb;
                kx.stop_at_knock = true;
                std::printf("        end fraction %.2f: theta_kl %.2f\n", xb, ec::knock_limited_spark(kx, mbt_btdc, 0.0).theta_kl_btdc);
            }
            kin.stop_at_knock = true;
            std::printf("        end fraction 0.90: theta_kl %.2f\n", ec::knock_limited_spark(kin, mbt_btdc, 0.0).theta_kl_btdc);
        }
    }
}

TEST_CASE("P7 diag: generation time with the knock limit on and off", "[.][diag][knock][drivetrain][cycle]") {
    struct Case {
        const char* label;
        json on, off;
        io::EngineSlotOverrides slots_on;
    };
    TempDir td("timing");
    json hyper_ecu = {{"format", "physics_sim.ecu/1"},
                      {"extends", (kSrc / "data" / "ecus" / "hyper_v8_one1.json").generic_string()},
                      {"ecu", {{"ignition_strategy", "knock_limited"}, {"knock_margin_deg", 2.0}}}};
    io::EngineSlotOverrides hyper_slots;
    hyper_slots["ecu"] = td.write("hyper_ecu.json", hyper_ecu);
    std::vector<Case> cases;
    cases.push_back({"a25a_gen", knock_engine("toyota_a25a_fks_gen.json", 2.0, false), json{{"extends", "toyota_a25a_fks_gen.json"}}, {}});
    cases.push_back({"demo_i4_pipes", knock_engine("demo_i4_pipes.json", 2.0, false), json{{"extends", "demo_i4_pipes.json"}}, {}});
    cases.push_back({"hyper_v8_one1_audio", json{{"extends", "hyper_v8_one1_audio.json"}}, json{{"extends", "hyper_v8_one1_audio.json"}}, hyper_slots});
    for (const Case& c : cases) {
        for (MapQuality q : {MapQuality::Sketch, MapQuality::Preview, MapQuality::Full}) {
            for (int knock = 0; knock < 2; ++knock) {
                io::EngineMapOptions o;
                o.quality = q;
                o.pool = &pool();
                const auto t0 = std::chrono::steady_clock::now();
                const ComponentParams p = io::load_engine_json_from_string((knock ? c.on : c.off).dump(), engines_dir(), o,
                                                                            knock ? c.slots_on : io::EngineSlotOverrides{});
                const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                const auto& e = std::get<SimulatedEngineDesc>(p);
                std::printf("TIMING %-20s quality %d knock %d: %.3f s (theta_kl tables %zu)\n", c.label, int(q), knock, s,
                            e.maps.theta_kl_by_fuel.size());
            }
        }
    }
}
