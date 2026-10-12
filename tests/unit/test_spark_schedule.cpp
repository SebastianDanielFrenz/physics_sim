// tests/unit/test_spark_schedule.cpp - PHYS-033 P4 I7c C17a follow-up: the
// Tier 1b ECU spark schedule and the Tier 2 ignition (flame-development)
// delay in a simulated engine's cycle block (ps/drivetrain/engine_cycle_desc.h
// CycleModelDesc::spark_advance_deg / ignition_delay_deg).
//
// Design (A2, out/c17a2_progress.md): the generator runs every grid node with
// its Wiebe start at -(schedule) + delay, stores theta_mbt = -(MBT Wiebe start
// - delay) and the MBT-EQUIVALENT eta_i = eta_gross / eta_spark(schedule -
// theta_mbt), so the unchanged runtime (eta_i * f_lambda * eta_spark(spark -
// theta_mbt), spark = the ECU table the loader copies from the cycle block)
// reproduces the cycle at the scheduled spark at every node. Without a
// schedule and with delay 0 nothing changes.
//
// Sabotages (each run once by hand on a modified build, 2026-09-28):
//   X1  the generator's eta_spark division running for an engine WITHOUT a
//       schedule, the empty table read as 0 deg BTDC -> all three pinned
//       hashes differ. The literals were recorded on the pre-follow-up build
//       (50b01b5) before any code change.
//   X2  skipping the eta_spark division (eta_i stored at the scheduled spark,
//       not MBT-equivalent) -> worst node rel 0.142; the Wiebe start ignoring
//       the delay (-schedule only) -> worst node rel 0.0469; theta_mbt
//       ignoring the delay -> the nodes stay consistent (8e-16: generator and
//       runtime share theta_mbt by construction) and only the explicit
//       theta_mbt == -(MBT start - delay) check fails - which is why it is
//       there.
//   X3  removing the "both given" rejection -> that SECTION fails (no throw:
//       the ecu table would be silently replaced).
//   H3  (test_engine_map_cache.cpp) dropping the delay from the cache key ->
//       its "with a spark schedule present" SECTION fails on
//       ignition_delay_deg.
#include "drivetrain/engine_simulated.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/wiebe.h"

#include "cycle_fixtures.h"
#include "mvem_fixtures.h"

#include "ps/drivetrain/engine_maps.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;

namespace {

namespace ec = ps::engine_cycle;
namespace fx = ps::test::mvem;

constexpr real kPi = 3.14159265358979323846;
real w_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

// test_engine_map_generator.cpp's fixture (the generic ASSUMED 2 L I4 on a
// 4 x 3 grid, F4 friction).
CycleModelDesc fixture_desc() {
    CycleModelDesc d = ps::test::make_generic_i4_cycle_desc();
    d.grid_rpm = {1000.0, 2500.0, 4000.0, 5500.0};
    d.grid_p_im = {30000.0, 60000.0, 95000.0};
    d.friction.model = FrictionModelDesc::Model::PolynomialPistonSpeed;
    d.friction.a_pa = 66619.521802247167;
    d.friction.b_pa = 91.085076199249656;
    d.friction.c_pa = 346.80954153442133;
    return d;
}

// A deliberately non-uniform schedule (deg BTDC over rpm x Pa), well inside
// the eta_spark sweep range for the fixture's MBT start of -23.05 deg ATDC.
Table2D test_schedule() {
    Table2D t;
    t.x = {1000.0, 3000.0, 5500.0};
    t.y = {30000.0, 95000.0};
    t.values = {12.0, 4.0, 26.0, 15.0, 31.0, 22.0};
    return t;
}
constexpr real kTestDelayDeg = 5.0;

real mbt_atdc(const CycleModelDesc& d) {
    return ec::Wiebe::theta0_for_ca50(d.wiebe_a, d.wiebe_m, d.burn_duration_deg, d.ca50_mbt_deg_atdc);
}

Table2D const_table(real v) {
    Table2D t;
    t.x = {0.0, 9000.0};
    t.y = {0.0, 150000.0};
    t.values.assign(4, v);
    return t;
}

SimulatedEngineDesc engine_on(const EngineMaps& em, const Table2D& spark) {
    SimulatedEngineDesc e = fx::base_engine();
    e.maps = em.maps;
    e.ecu.lambda_target = const_table(1.0);
    e.ecu.spark_advance_deg = spark;
    e.breakaway_torque_nm = em.maps.friction_torque_nm.y.front() + 5.0;
    return e;
}

// The cycle at one node, lambda 1, Wiebe start = -(spark BTDC) + delay.
real reference_net_torque(const ec::CycleStepper& proto, const CycleModelDesc& d, real rpm, real p, real wiebe_start_atdc) {
    ec::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p;
    op.t_im_k = SimulatedEngine::kIsaTemperatureK;
    op.p_amb_pa = SimulatedEngine::kIsaPressurePa;
    op.lambda = 1.0;
    op.spark_deg_atdc = wiebe_start_atdc;
    ec::StepperSettings st;
    st.h_s = ec::kStepPreview;
    st.cycles = ec::kCyclesPreview;
    const ec::PointResult r = ec::run_operating_point(proto, op, st);
    return static_cast<real>(d.cylinders) * (r.w_gross_j + r.w_pump_j) / (4.0 * kPi) - cycle_friction_torque_nm(d, rpm);
}

bool same_bits(const std::vector<real>& a, const std::vector<real>& b) {
    return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(real)) == 0);
}

real max_rel(const std::vector<real>& a, const std::vector<real>& b) {
    REQUIRE(a.size() == b.size());
    real w = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) w = std::max(w, std::abs(a[i] - b[i]) / std::max(std::abs(b[i]), real(1e-12)));
    return w;
}

// ---- loader helpers (test_engine_map_generator.cpp's patterns) ----
struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_spark_schedule_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::create_directories(dir);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    std::filesystem::path write(const std::string& name, const json& j) const {
        const auto p = dir / name;
        std::ofstream out(p);
        out << j.dump(2);
        return p;
    }
};

json curve_json(const vehicle::Curve1D& c, real scale_y) {
    json x = json::array(), y = json::array();
    for (real v : c.x) x.push_back(v);
    for (real v : c.y) y.push_back(v * scale_y);
    return {{"x", x}, {"y", y}};
}

json cycle_json(const CycleModelDesc& d) {
    json c;
    c["source"] = "generic 2 L I4 cycle fixture (tests/support/cycle_fixtures.h), every value ASSUMED, not physical data";
    c["rod_mm"] = d.rod_m * 1e3;
    c["compression_ratio"] = d.compression_ratio;
    c["intake_valves"] = d.intake_valves;
    c["exhaust_valves"] = d.exhaust_valves;
    c["intake_valve_d_mm"] = d.intake_valve_d_m * 1e3;
    c["exhaust_valve_d_mm"] = d.exhaust_valve_d_m * 1e3;
    c["intake_lift"] = {{"lift_mm_vs_deg", curve_json(d.intake_lift.lift_m_vs_deg, 1e3)},
                        {"phase_ref_deg", d.intake_lift.phase_ref_deg}};
    c["exhaust_lift"] = {{"lift_mm_vs_deg", curve_json(d.exhaust_lift.lift_m_vs_deg, 1e3)},
                         {"phase_ref_deg", d.exhaust_lift.phase_ref_deg}};
    c["runner_length_mm"] = d.runner_length_m * 1e3;
    c["runner_diameter_mm"] = d.runner_diameter_m * 1e3;
    c["intake_port_cc"] = d.intake_port_m3 * 1e6;
    c["exhaust_port_cc"] = d.exhaust_port_m3 * 1e6;
    c["cd_vs_l_over_d"] = curve_json(d.cd_vs_l_over_d, 1.0);
    c["friction"] = {{"model", "polynomial_piston_speed"},
                     {"a_kpa", d.friction.a_pa / 1e3},
                     {"b_kpa", d.friction.b_pa / 1e3},
                     {"c_kpa", d.friction.c_pa / 1e3}};
    json g = json::array(), p = json::array();
    for (real v : d.grid_rpm) g.push_back(v);
    for (real v : d.grid_p_im) p.push_back(v / 1e3);
    c["grid_rpm"] = g;
    c["grid_p_im_kpa"] = p;
    return c;
}

json schedule_json() {
    // test_schedule() in the file's units (kPa).
    return {{"rpm", {1000.0, 3000.0, 5500.0}}, {"p_im_kpa", {30.0, 95.0}}, {"values", {{12.0, 4.0}, {26.0, 15.0}, {31.0, 22.0}}}};
}

// test_engine_map_generator.cpp's base_engine_json() without "maps" (every
// map from the cycle block) and with the cycle's spark schedule instead of an
// ecu one.
json scheduled_engine_json() {
    json j;
    j["id"] = "engine";
    j["type"] = "engine";
    j["kind"] = "simulated";
    j["architecture"] = "piston_4stroke";
    j["bore_mm"] = 86.0;
    j["stroke_mm"] = 86.0;
    j["cylinders"] = 4;
    j["intake_manifold_volume_l"] = 3.0;
    j["throttle_body"] = {{"diameter_mm", 60.0},
                          {"count", 1},
                          {"discharge_coefficient", 0.8},
                          {"area_fraction_vs_position", {{"x", {0.0, 0.5, 1.0}}, {"y", {0.01, 0.4, 1.0}}}}};
    j["throttle_input"] = "electronic";
    j["throttle_actuator_tau_s"] = 0.04;
    j["idle_control"] = {{"kp_per_rpm", 0.001}, {"ki_per_rpm_s", 0.004}, {"max_throttle", 0.2}};
    j["ecu"] = {{"lambda_target", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{1.0, 1.0}, {1.0, 1.0}}}}}};
    j["cycle"] = cycle_json(fixture_desc());
    j["cycle"]["spark_advance_deg"] = schedule_json();
    j["cycle"]["ignition_delay_deg"] = kTestDelayDeg;
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 40.0;
    j["fuel"] = std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string();
    j["ignition"] = "spark";
    return j;
}

vehicle::VehicleDesc load_engine(const json& engine) {
    json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_spark_schedule.cpp)";
    v["wheels"] = json::array({{{"name", "FL"}, {"tyre", {{"source", "test fixture"}}}}});
    v["powertrain"] = {{"components", json::array({engine})}};
    TempDir d;
    const auto p = d.write("vehicle.json", v);
    io::EngineMapOptions o;
    o.quality = MapQuality::Preview;
    return io::load_vehicle_json(p.string(), o);
}

const SimulatedEngineDesc& only_sim_engine(const vehicle::VehicleDesc& d) {
    REQUIRE(d.powertrain.components.size() == 1);
    const auto* e = std::get_if<SimulatedEngineDesc>(&d.powertrain.components[0].params);
    REQUIRE(e != nullptr);
    return *e;
}

} // namespace

// Literals recorded on the pre-follow-up build (50b01b5, out/c17a2_progress.md)
// before any code change: an engine without a schedule (and delay 0) must keep
// its generated maps bit for bit, so its state hash and cache content too.
TEST_CASE("C17a2 X1: an engine without a spark schedule generates bit-identical maps", "[drivetrain][cycle]") {
    const CycleModelDesc d = fixture_desc();
    REQUIRE(d.spark_advance_deg.x.empty());
    REQUIRE(d.ignition_delay_deg == 0.0);
    const auto fuel = fx::gasoline();
    CHECK(generate_engine_maps(d, fuel, MapQuality::Preview)->content_hash == 0xa05071cba1c4edb6ULL);
    CHECK(generate_engine_maps(d, fuel, MapQuality::Full)->content_hash == 0xa5151993c360b909ULL);
    CycleModelDesc cam = d;
    cam.cam.intake_advance_deg.x = {1000.0, 5500.0};
    cam.cam.intake_advance_deg.y = {30000.0, 95000.0};
    cam.cam.intake_advance_deg.values = {0.0, 10.0, 8.0, 5.0};
    CHECK(generate_engine_maps(cam, fuel, MapQuality::Preview)->content_hash == 0x470d850d99e07328ULL);
}

TEST_CASE("C17a2 X2: at every node the runtime reproduces the cycle at the scheduled spark plus delay",
          "[drivetrain][cycle]") {
    CycleModelDesc d = fixture_desc();
    d.spark_advance_deg = test_schedule();
    d.ignition_delay_deg = kTestDelayDeg;
    const EngineMapsPtr em = generate_engine_maps(d, fx::gasoline(), MapQuality::Preview);
    const SimulatedEngine eng(engine_on(*em, d.spark_advance_deg));
    const ec::CycleStepper proto(d, fx::gasoline().lhv_j_kg, fx::gasoline().afr_stoich);
    const real c_k = em->maps.friction_torque_nm.y.front();

    // theta_mbt is the spark whose Wiebe start is the MBT start.
    const real mbt = mbt_atdc(d);
    for (real v : em->maps.theta_mbt_deg.values) CHECK(v == -(mbt - kTestDelayDeg));

    real worst = 0.0;
    real spark_lo = 1e9, spark_hi = -1e9, eta_lo = 1e9;
    for (real rpm : d.grid_rpm) {
        for (real p : d.grid_p_im) {
            const real spark = d.spark_advance_deg.value(rpm, p);
            const real ref = reference_net_torque(proto, d, rpm, p, -spark + kTestDelayDeg);
            const real sim = eng.evaluate_torque(w_of(rpm), p, true, 1.0, false).tau - c_k;
            const real rel = std::abs(sim - ref) / std::max(std::abs(ref), real(1.0));
            const real es = em->maps.eta_spark.evaluate(spark - em->maps.theta_mbt_deg.value(rpm, p));
            std::printf("X2 node %6.0f rpm %6.0f Pa  spark %6.2f BTDC  eta_spark %.4f  ref %12.6f  sim %12.6f  rel %.3g\n",
                        rpm, p, spark, es, ref, sim, rel);
            worst = std::max(worst, rel);
            spark_lo = std::min(spark_lo, spark);
            spark_hi = std::max(spark_hi, spark);
            eta_lo = std::min(eta_lo, es);
        }
    }
    std::printf("X2 worst node rel %.3g (spark %.2f..%.2f BTDC, eta_spark >= %.4f)\n", worst, spark_lo, spark_hi, eta_lo);
    // The schedule must actually exercise retard: otherwise a skipped
    // eta_spark division could pass unnoticed.
    CHECK(eta_lo < 0.95);
    CHECK(worst <= 1e-9);
}

TEST_CASE("C17a2 X2b: a schedule equal to the MBT spark reproduces the mbt_tracking maps", "[drivetrain][cycle]") {
    const CycleModelDesc base = fixture_desc();
    CycleModelDesc d = base;
    d.spark_advance_deg = const_table(-mbt_atdc(base));
    const EngineMapsPtr a = generate_engine_maps(base, fx::gasoline(), MapQuality::Preview);
    const EngineMapsPtr b = generate_engine_maps(d, fx::gasoline(), MapQuality::Preview);
    // Same cycle points to rounding of -(-mbt): the maps agree to rounding.
    const real e_eta = max_rel(b->maps.eta_i.values, a->maps.eta_i.values);
    const real e_ve = max_rel(b->maps.ve.values, a->maps.ve.values);
    std::printf("X2b eta_i %.3g ve %.3g\n", e_eta, e_ve);
    CHECK(e_eta <= 1e-12);
    CHECK(e_ve <= 1e-12);
    CHECK(same_bits(b->maps.theta_mbt_deg.values, a->maps.theta_mbt_deg.values));
    CHECK(same_bits(b->maps.eta_spark.y, a->maps.eta_spark.y));
}

TEST_CASE("C17a2 X3: cycle spark schedule loader, ecu copy and rejections", "[drivetrain][cycle][io]") {
    SECTION("accepted: the ecu schedule is the cycle's, the delay reaches the desc") {
        const vehicle::VehicleDesc v = load_engine(scheduled_engine_json());
        const SimulatedEngineDesc& e = only_sim_engine(v);
        REQUIRE(e.cycle.has_value());
        const Table2D want = test_schedule();
        CHECK(e.cycle->spark_advance_deg.x == want.x);
        CHECK(e.cycle->spark_advance_deg.y == want.y);
        CHECK(e.cycle->spark_advance_deg.values == want.values);
        CHECK(e.ecu.spark_advance_deg.x == want.x);
        CHECK(e.ecu.spark_advance_deg.y == want.y);
        CHECK(e.ecu.spark_advance_deg.values == want.values);
        CHECK(e.cycle->ignition_delay_deg == kTestDelayDeg);
        CHECK(e.maps.theta_mbt_deg.values.front() == -(mbt_atdc(*e.cycle) - kTestDelayDeg));
    }
    SECTION("rejected: both the ecu and the cycle give a schedule") {
        json j = scheduled_engine_json();
        j["ecu"]["spark_advance_deg"] = schedule_json();
        CHECK_THROWS_WITH(load_engine(j),
                          ContainsSubstring("'spark_advance_deg' must be absent when cycle.spark_advance_deg is given"));
    }
    SECTION("rejected: neither gives a schedule") {
        json j = scheduled_engine_json();
        j["cycle"].erase("spark_advance_deg");
        CHECK_THROWS_WITH(load_engine(j), ContainsSubstring("'spark_advance_deg' is required"));
    }
    SECTION("rejected: delay out of [0, 60]") {
        for (real bad : {-0.5, 60.5}) {
            json j = scheduled_engine_json();
            j["cycle"]["ignition_delay_deg"] = bad;
            CHECK_THROWS_WITH(load_engine(j), ContainsSubstring("'ignition_delay_deg' must be in [0, 60], got"));
        }
    }
    SECTION("rejected: a spark value outside [-30, 70]") {
        json j = scheduled_engine_json();
        j["cycle"]["spark_advance_deg"]["values"][1][0] = 70.5;
        CHECK_THROWS_WITH(load_engine(j), ContainsSubstring(".spark_advance_deg: every value must be in [-30.000000, 70.000000]"));
    }
    SECTION("rejected: a non-positive pressure axis") {
        json j = scheduled_engine_json();
        j["cycle"]["spark_advance_deg"]["p_im_kpa"] = {0.0, 95.0};
        CHECK_THROWS_WITH(load_engine(j), ContainsSubstring(".spark_advance_deg: 'p_im_kpa' must be > 0"));
    }
    SECTION("generator-side rejections (a desc built in C++)") {
        CycleModelDesc d = fixture_desc();
        d.ignition_delay_deg = -1.0;
        CHECK_THROWS_WITH(generate_engine_maps(d, fx::gasoline(), MapQuality::Preview),
                          ContainsSubstring("ignition_delay_deg must be in [0, 60]"));
        d.ignition_delay_deg = 0.0;
        d.spark_advance_deg = test_schedule();
        d.spark_advance_deg.values.pop_back();
        CHECK_THROWS_WITH(generate_engine_maps(d, fx::gasoline(), MapQuality::Preview),
                          ContainsSubstring("engine map generator: spark_advance_deg: "));
    }
}
