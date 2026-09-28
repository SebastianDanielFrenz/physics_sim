// PHYS-033 P4 I7c C16a: map extraction from the 0D cycle model
// (ps::engine_cycle), loader-time generation and worker invariance
// (out/i7c_plan.md C16a, tests M1-M8, plus the F4 friction fit).
//
// Test inputs: tests/support/cycle_fixtures.h's generic 2 L I4 (every value
// ASSUMED, not physical data) on a small 4 x 3 grid, generated at Preview
// quality (the 4 x 3 grid is below the Preview subsample, so Preview uses it
// unchanged). The A25A is NOT a generator test input (plan D1: its comparison
// belongs to C17a); no Mazda data is used.
//
// F4 friction: the fixture's friction polynomial is fitted here from the
// MEASURED motored-friction sweep of the 2013 GM 2.5 L Ecotec LCV (EPA NVFEL
// test package, out/sources/chevrolet_ecotec_lcv_25/, sheet "Motored
// Friction"), with pumping and closed-cycle heat loss removed by our own
// cycle model on an ASSUMED Ecotec valvetrain (the generic fixture's,
// scaled to 88 mm bore). See the "C16a F4" case for what is measured,
// cited, derived and assumed.

#include "drivetrain/engine.h"
#include "drivetrain/engine_simulated.h"
#include "drivetrain/powertrain.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/engine_cycle/wiebe.h"

#include "cycle_fixtures.h"
#include "mvem_fixtures.h"
#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/drivetrain/controller_desc.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;
namespace fx = ps::test::mvem;
namespace da = ps::jobs::debug_alloc;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;
real w_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

// F4 fit (C16a F4 case below): FMEP = a + b*Sp + c*Sp^2, Sp = mean piston
// speed [m/s]. DERIVED from the measured Ecotec sweep; pinned by that case.
constexpr real kF4A = 66619.521802247167;
constexpr real kF4B = 91.085076199249656;
constexpr real kF4C = 346.80954153442133;

CycleModelDesc fixture_desc() {
    CycleModelDesc d = ps::test::make_generic_i4_cycle_desc();
    d.grid_rpm = {1000.0, 2500.0, 4000.0, 5500.0};
    d.grid_p_im = {30000.0, 60000.0, 95000.0};
    d.friction.model = FrictionModelDesc::Model::PolynomialPistonSpeed;
    d.friction.a_pa = kF4A;
    d.friction.b_pa = kF4B;
    d.friction.c_pa = kF4C;
    return d;
}

const EngineMaps& fixture_maps() {
    static const EngineMapsPtr p = generate_engine_maps(fixture_desc(), fx::gasoline(), MapQuality::Preview);
    return *p;
}

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

// mvem_fixtures' base engine (86/86 mm I4 like the cycle fixture) on the
// generated maps; ECU lambda 1 and spark = the maps' theta_MBT everywhere,
// so f_lambda = eta_spark = 1 and the runtime sees exactly the node values.
SimulatedEngineDesc engine_on(const EngineMaps& em) {
    SimulatedEngineDesc e = fx::base_engine();
    e.maps = em.maps;
    e.ecu.lambda_target = const_table(1.0);
    e.ecu.spark_advance_deg = const_table(em.maps.theta_mbt_deg.values.front());
    e.breakaway_torque_nm = em.maps.friction_torque_nm.y.front() + 5.0;
    return e;
}

ec::PointResult reference_point(const ec::CycleStepper& proto, const CycleModelDesc& d, real rpm, real p) {
    ec::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p;
    op.t_im_k = SimulatedEngine::kIsaTemperatureK;
    op.p_amb_pa = SimulatedEngine::kIsaPressurePa;
    op.lambda = 1.0;
    op.spark_deg_atdc = mbt_atdc(d);
    ec::StepperSettings st;
    st.h_s = ec::kStepPreview;
    st.cycles = ec::kCyclesPreview;
    return ec::run_operating_point(proto, op, st);
}

// Net brake torque the cycle model plus the friction correlation predict.
real reference_net_torque(const CycleModelDesc& d, const ec::PointResult& r, real rpm) {
    return static_cast<real>(d.cylinders) * (r.w_gross_j + r.w_pump_j) / (4.0 * kPi) - cycle_friction_torque_nm(d, rpm);
}

bool same_bits(const std::vector<real>& a, const std::vector<real>& b) {
    return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(real)) == 0);
}
bool same_bits(const Table2D& a, const Table2D& b) {
    return same_bits(a.x, b.x) && same_bits(a.y, b.y) && same_bits(a.values, b.values);
}
bool same_bits(const vehicle::Curve1D& a, const vehicle::Curve1D& b) { return same_bits(a.x, b.x) && same_bits(a.y, b.y); }
bool same_bits(real a, real b) { return std::memcmp(&a, &b, sizeof(real)) == 0; }

bool same_maps(const SimulatedEngineMapsDesc& a, const SimulatedEngineMapsDesc& b) {
    return same_bits(a.ve, b.ve) && same_bits(a.eta_i, b.eta_i) && same_bits(a.f_lambda, b.f_lambda) &&
           same_bits(a.eta_spark, b.eta_spark) && same_bits(a.theta_mbt_deg, b.theta_mbt_deg) &&
           same_bits(a.friction_torque_nm, b.friction_torque_nm) && a.has_pmep_excess == b.has_pmep_excess &&
           same_bits(a.pmep_excess_pa, b.pmep_excess_pa) && a.has_theta_kl == b.has_theta_kl &&
           same_bits(a.theta_kl_deg, b.theta_kl_deg);
}

bool same_engine_maps(const EngineMaps& a, const EngineMaps& b) {
    return same_maps(a.maps, b.maps) && a.maps.source == b.maps.source && same_bits(a.t_exh_k, b.t_exh_k) &&
           same_bits(a.h_exh_w, b.h_exh_w) && same_bits(a.eta_spark_spread, b.eta_spark_spread) &&
           a.generator_version == b.generator_version && a.quality == b.quality && a.content_hash == b.content_hash;
}

// ---- loader helpers (test_simulated_engine_io.cpp's patterns) ----

struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_engine_map_gen_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
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

std::string gasoline_path() {
    return std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string();
}

json curve_json(const vehicle::Curve1D& c, real scale_y) {
    json x = json::array(), y = json::array();
    for (real v : c.x) x.push_back(v);
    for (real v : c.y) y.push_back(v * scale_y);
    return {{"x", x}, {"y", y}};
}

// The fixture desc as a "cycle" block (mm, cc, kPa). Engine geometry
// (bore/stroke/cylinders) comes from the engine's own piston fields.
json cycle_json(const CycleModelDesc& d, bool with_friction) {
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
    if (with_friction) {
        c["friction"] = {{"model", "polynomial_piston_speed"},
                         {"a_kpa", d.friction.a_pa / 1e3},
                         {"b_kpa", d.friction.b_pa / 1e3},
                         {"c_kpa", d.friction.c_pa / 1e3}};
    }
    json g = json::array(), p = json::array();
    for (real v : d.grid_rpm) g.push_back(v);
    for (real v : d.grid_p_im) p.push_back(v / 1e3);
    c["grid_rpm"] = g;
    c["grid_p_im_kpa"] = p;
    return c;
}

// test_simulated_engine_io.cpp's base_simulated_engine() (86/86 mm I4,
// synthetic test maps, not physical data).
json base_engine_json() {
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
    j["ecu"] = {{"lambda_target", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{1.0, 1.0}, {1.0, 0.9}}}}},
                {"spark_advance_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}}};
    j["maps"] = {
        {"source", "synthetic test fixture (test_engine_map_generator.cpp), not physical data"},
        {"ve", {{"rpm", {0.0, 3000.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{0.80, 0.85}, {0.88, 0.95}, {0.85, 0.90}}}}},
        {"eta_i", {{"rpm", {0.0, 7000.0}}, {"air_mass_per_rev_g", {0.0, 1.5}}, {"values", {{0.34, 0.37}, {0.36, 0.38}}}}},
        {"f_lambda", {{"x", {0.7, 1.0, 1.5}}, {"y", {0.85, 1.0, 0.9}}}},
        {"eta_spark", {{"x", {-40.0, 0.0, 10.0}}, {"y", {0.5, 1.0, 0.95}}}},
        {"theta_mbt_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}},
        {"friction_torque_nm", {{"x", {0.0, 1000.0, 7000.0}}, {"y", {15.0, 18.0, 40.0}}}}};
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 20.0;
    j["fuel"] = gasoline_path();
    j["ignition"] = "spark";
    return j;
}

// An engine whose maps come from its cycle block only.
json cycle_only_engine() {
    json j = base_engine_json();
    j.erase("maps");
    j["cycle"] = cycle_json(fixture_desc(), true);
    j["breakaway_torque_nm"] = 40.0;
    return j;
}

json vehicle_with_engine(const json& engine) {
    json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_engine_map_generator.cpp)";
    v["wheels"] = json::array({{{"name", "FL"}}});
    v["powertrain"] = {{"components", json::array({engine})}};
    return v;
}

io::EngineMapOptions preview_options(std::size_t* count) {
    io::EngineMapOptions o;
    o.quality = MapQuality::Preview;
    o.generation_count_out = count;
    return o;
}

vehicle::VehicleDesc load_engine(const json& engine, std::size_t* count) {
    TempDir d;
    const auto p = d.write("vehicle.json", vehicle_with_engine(engine));
    return io::load_vehicle_json(p.string(), preview_options(count));
}

const SimulatedEngineDesc& only_sim_engine(const vehicle::VehicleDesc& d) {
    REQUIRE(d.powertrain.components.size() == 1);
    const auto* e = std::get_if<SimulatedEngineDesc>(&d.powertrain.components[0].params);
    REQUIRE(e != nullptr);
    return *e;
}

// ---- World helpers (test_simulated_engine_powertrain.cpp's patterns) ----

vehicle::VehicleDesc sim_sedan(SimulatedEngineDesc eng) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    bool swapped = false;
    for (auto& c : d.powertrain.components) {
        if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            eng.crank_axis_local = tm->crank_axis_local;
            c.params = eng;
            swapped = true;
            break;
        }
    }
    REQUIRE(swapped);
    return d;
}

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
};

Rig make_rig(const vehicle::VehicleDesc& desc, int workers) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    rig.world = std::make_unique<World>(cfg);

    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);

    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId ch = rig.world->create_body(chassis);

    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    rig.vehicle = rig.world->create_vehicle(desc, ch);
    rig.world->set_control("assist.auto_clutch", 0.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = true;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

Sensors running(real omega) {
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    return s;
}

real crank_step(SimulatedEngine& e, real omega, real c, real J, real h) {
    const PreSolveResult r = e.pre_solve(running(omega), ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, h);
    real w = omega + h * (r.tau0 - r.friction_cap_kinetic) / (J + h * r.d);
    if (w < 0.0) w = 0.0;
    (void)e.post_solve(w, h);
    return w;
}

} // namespace

// ---------------------------------------------------------------------------
// F4: motoring FMEP of the 2013 GM 2.5 L Ecotec LCV.
//   MEASURED (EPA NVFEL package, "Motored Friction" sheet): speed, motored
//     brake torque, BMEP and ambient pressure at 5 speeds, oil 91.9-98.4 C.
//   CITED (report): 11.3:1 compression ratio.
//   DERIVED: V_d = 4*pi*T/BMEP = 2.4568 L from the xlsx.
//   ASSUMED: bore 88 / stroke 101 mm (GM spec, not in the package; gives
//     2.4573 L, 0.02 % from the derived V_d); throttle wide open while
//     motoring (p_im = p_amb); T_im 298.15 K; the valvetrain, ports and
//     runner of the generic fixture scaled by 88/86 (valve diameters) and the
//     rod by 101/86.
// FMEP = IMEP_net(cycle model, motored: pumping + closed-cycle heat loss) -
// BMEP(measured); a + b*Sp + c*Sp^2 least-squares over the 5 points.
TEST_CASE("C16a F4: Ecotec motored-friction fit over mean piston speed", "[drivetrain][cycle]") {
    struct Row { real rpm, bmep_bar, p_amb_bar; };
    const Row rows[] = {{999.70, -0.88419, 0.9705}, {1991.42, -1.08094, 0.9710}, {3000.06, -1.43171, 0.9731},
                        {3986.00, -1.94473, 0.972}, {4997.85, -2.49549, 0.9690}};
    CycleModelDesc d = ps::test::make_generic_i4_cycle_desc();
    const real b0 = d.bore_m;
    d.bore_m = 0.088;
    d.stroke_m = 0.101;
    d.rod_m = d.rod_m * 0.101 / 0.086;
    d.intake_valve_d_m = d.intake_valve_d_m * 0.088 / b0;
    d.exhaust_valve_d_m = d.exhaust_valve_d_m * 0.088 / b0;
    d.compression_ratio = 11.3;
    const real v_d = kPi / 4.0 * d.bore_m * d.bore_m * d.stroke_m * static_cast<real>(d.cylinders);
    ec::CycleStepper proto(d, ps::test::kCycleFixtureLhv, ps::test::kCycleFixtureAfr);
    proto.set_switches(ec::CycleSwitches{true, false, true});
    ec::StepperSettings st;
    st.h_s = ec::kStepFull;
    st.cycles = ec::kCyclesFull;

    real sp[5], fmep[5];
    for (int i = 0; i < 5; ++i) {
        ec::OperatingPoint op;
        op.rpm = rows[i].rpm;
        op.p_im_pa = rows[i].p_amb_bar * 1e5;
        op.p_amb_pa = rows[i].p_amb_bar * 1e5;
        op.t_im_k = 298.15;
        op.lambda = 1.0;
        const ec::PointResult r = ec::run_operating_point(proto, op, st);
        const real imep = static_cast<real>(d.cylinders) * (r.w_gross_j + r.w_pump_j) / v_d;
        sp[i] = 2.0 * d.stroke_m * rows[i].rpm / 60.0;
        fmep[i] = imep - rows[i].bmep_bar * 1e5;
        std::printf("F4 rpm %8.2f Sp %7.4f m/s  imep_net %10.2f Pa (gross %10.2f pump %10.2f)  fmep %10.2f Pa\n", rows[i].rpm,
                    sp[i], imep, static_cast<real>(d.cylinders) * r.w_gross_j / v_d,
                    static_cast<real>(d.cylinders) * r.w_pump_j / v_d, fmep[i]);
    }
    // normal equations for [a b c]
    real m[3][4] = {};
    for (int i = 0; i < 5; ++i) {
        const real phi[3] = {1.0, sp[i], sp[i] * sp[i]};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) m[r][c] += phi[r] * phi[c];
            m[r][3] += phi[r] * fmep[i];
        }
    }
    for (int k = 0; k < 3; ++k) {
        for (int r = k + 1; r < 3; ++r) {
            const real f = m[r][k] / m[k][k];
            for (int c = k; c < 4; ++c) m[r][c] -= f * m[k][c];
        }
    }
    real coef[3];
    for (int k = 2; k >= 0; --k) {
        real s = m[k][3];
        for (int c = k + 1; c < 3; ++c) s -= m[k][c] * coef[c];
        coef[k] = s / m[k][k];
    }
    real worst = 0.0;
    for (int i = 0; i < 5; ++i) {
        const real fit = coef[0] + coef[1] * sp[i] + coef[2] * sp[i] * sp[i];
        worst = std::max(worst, std::abs(fit - fmep[i]));
    }
    std::printf("F4 fit a %.17g Pa, b %.17g Pa/(m/s), c %.17g Pa/(m/s)^2, worst residual %.4g Pa\n", coef[0], coef[1], coef[2], worst);
    CHECK(coef[0] >= 0.0);
    CHECK(std::abs(coef[0] - kF4A) <= 1e-8 * std::abs(kF4A));
    CHECK(std::abs(coef[1] - kF4B) <= 1e-8 * std::abs(kF4B));
    CHECK(std::abs(coef[2] - kF4C) <= 1e-8 * std::abs(kF4C));
}

// ---------------------------------------------------------------------------
namespace {

struct OffNodeStats {
    int n = 0;
    real net = 0.0, net_mean = 0.0, net_rpm = 0.0, net_p = 0.0; // |sim - ref| / gross torque
    real eta = 0.0;   // |eta_sim / eta_ref - 1|
    real pump = 0.0;  // |pump_sim - pump_ref| / gross torque
    real fric = 0.0;  // |fric_sim - fric_ref| / gross torque
    real air = 0.0, air_mean = 0.0; // |m_air_sim / m_air_ref - 1|
};

// M8's production-spacing grid: 500 rpm x 5 kPa, 12 x 20.
CycleModelDesc production_grid_desc(const CycleModelDesc& d) {
    CycleModelDesc dd = d;
    dd.grid_rpm.clear();
    dd.grid_p_im.clear();
    for (int i = 0; i < 12; ++i) dd.grid_rpm.push_back(1000.0 + 500.0 * i);
    for (int i = 0; i < 20; ++i) dd.grid_p_im.push_back(20000.0 + 5000.0 * i);
    return dd;
}

// Every cell centre of dd's grid: the runtime on Preview maps generated
// from dd vs. the cycle model run at that point, the net torque and each
// term the runtime interpolates separately.
OffNodeStats off_node_error(const ec::CycleStepper& proto, const CycleModelDesc& dd) {
    jobs::JobSystem pool(10);
    const EngineMapsPtr dm = generate_engine_maps(dd, fx::gasoline(), MapQuality::Preview, &pool);
    const SimulatedEngine deng(engine_on(*dm));
    const real dc_k = dm->maps.friction_torque_nm.y.front();
    const real cyl = static_cast<real>(dd.cylinders);
    OffNodeStats st;
    real sum_net = 0.0, sum_air = 0.0;
    for (std::size_t i = 0; i + 1 < dd.grid_rpm.size(); ++i) {
        for (std::size_t j = 0; j + 1 < dd.grid_p_im.size(); ++j) {
            const real rpm = 0.5 * (dd.grid_rpm[i] + dd.grid_rpm[i + 1]);
            const real p = 0.5 * (dd.grid_p_im[j] + dd.grid_p_im[j + 1]);
            const ec::PointResult r = reference_point(proto, dd, rpm, p);
            const real ref = reference_net_torque(dd, r, rpm);
            const real gross = cyl * r.w_gross_j / (4.0 * kPi);
            const auto te = deng.evaluate_torque(w_of(rpm), p, true, 1.0, false);
            const real sim = te.tau - dc_k;
            const real net = std::abs(sim - ref) / gross;
            if (net > st.net) {
                st.net = net;
                st.net_rpm = rpm;
                st.net_p = p;
            }
            sum_net += net;
            const real eta_ref = r.w_gross_j * fx::gasoline().afr_stoich / (fx::gasoline().lhv_j_kg * r.m_air_j_cyc);
            st.eta = std::max(st.eta, std::abs(te.eta_i / eta_ref - 1.0));
            const real pump_ref = cyl * r.w_pump_j / (4.0 * kPi);
            st.pump = std::max(st.pump, std::abs(te.tau_pump - pump_ref) / gross);
            // Friction as the runtime composes it: whatever of the net torque
            // is neither combustion nor pumping.
            const real fric_sim = te.tau_comb + te.tau_pump - sim;
            st.fric = std::max(st.fric, std::abs(fric_sim - cycle_friction_torque_nm(dd, rpm)) / gross);
            const real air = std::abs(te.m_air_rev / (cyl * r.m_air_j_cyc / 2.0) - 1.0);
            st.air = std::max(st.air, air);
            sum_air += air;
            ++st.n;
        }
    }
    st.net_mean = sum_net / st.n;
    st.air_mean = sum_air / st.n;
    return st;
}

}  // namespace

TEST_CASE("C16a M1: the runtime reproduces the cycle model's net torque at grid nodes", "[drivetrain][cycle]") {
    const CycleModelDesc d = fixture_desc();
    const EngineMaps& em = fixture_maps();
    const SimulatedEngine eng(engine_on(em));
    const ec::CycleStepper proto(d, fx::gasoline().lhv_j_kg, fx::gasoline().afr_stoich);
    const real c_k = em.maps.friction_torque_nm.y.front();
    REQUIRE(em.maps.ve.x == d.grid_rpm);
    REQUIRE(em.maps.ve.y == d.grid_p_im);

    real worst_node = 0.0;
    for (real rpm : d.grid_rpm) {
        for (real p : d.grid_p_im) {
            const real ref = reference_net_torque(d, reference_point(proto, d, rpm, p), rpm);
            const real sim = eng.evaluate_torque(w_of(rpm), p, true, 1.0, false).tau - c_k;
            const real rel = std::abs(sim - ref) / std::max(std::abs(ref), real(1.0));
            std::printf("M1 node %6.0f rpm %6.0f Pa  ref %12.6f  sim %12.6f  rel %.3g\n", rpm, p, ref, sim, rel);
            worst_node = std::max(worst_node, rel);
        }
    }
    std::printf("M1 worst node rel %.3g\n", worst_node);
    CHECK(worst_node <= 1e-9);

    // Off-node (plan: <= 1 %, interpolation), PROVISIONAL P-O: cell centres
    // of the production-spacing grid. Every term the runtime interpolates on
    // a smooth surface - eta_i, pumping, friction - is held to the plan's
    // 1 % (eta relative, torques relative to the indicated gross torque,
    // since the net torque crosses zero at light load). Trapped air is not:
    // the fixture's lumped runner (a Helmholtz resonator, ~250 Hz) rings at
    // harmonics of the intake frequency, a step- and cycle-converged ripple
    // of ~100 rpm period and up to +-2.5 % at 1700 rpm / 22.5 kPa (the "M1
    // diag" case) that no practical rpm grid resolves; its error is pinned
    // at the measured value with margin instead, and the net-torque error
    // it causes is reported.
    const CycleModelDesc dd = production_grid_desc(d);
    const OffNodeStats st = off_node_error(proto, dd);
    std::printf("M1 off-node %d cell centres: eta %.4g  pump %.4g  fric %.4g  air worst %.4g mean %.4g  "
                "net(gross) worst %.4g at %.0f rpm / %.0f Pa, mean %.4g\n",
                st.n, st.eta, st.pump, st.fric, st.air, st.air_mean, st.net, st.net_rpm, st.net_p, st.net_mean);
    CHECK(st.eta <= 0.01);
    CHECK(st.pump <= 0.01);
    CHECK(st.fric <= 0.01);
    CHECK(st.air <= 0.09);  // measured 0.0766
    CHECK(st.air_mean <= 0.02);  // measured 0.0158
}

TEST_CASE("C16a M2: eta_i resampling is exact at every node", "[drivetrain][cycle]") {
    const CycleModelDesc d = fixture_desc();
    const EngineMaps& em = fixture_maps();
    const SimulatedEngine eng(engine_on(em));
    const ec::CycleStepper proto(d, fx::gasoline().lhv_j_kg, fx::gasoline().afr_stoich);
    real worst = 0.0;
    for (real rpm : d.grid_rpm) {
        for (real p : d.grid_p_im) {
            const ec::PointResult r = reference_point(proto, d, rpm, p);
            const real eta_ref = r.w_gross_j * fx::gasoline().afr_stoich / (fx::gasoline().lhv_j_kg * r.m_air_j_cyc);
            const real m_air_rev = eng.evaluate_torque(w_of(rpm), p, true, 1.0, false).m_air_rev;
            const real eta = em.maps.eta_i.value(rpm, m_air_rev);
            worst = std::max(worst, std::abs(eta - eta_ref) / eta_ref);
        }
    }
    std::printf("M2 worst eta_i node rel %.3g (axis %zu air values)\n", worst, em.maps.eta_i.y.size());
    CHECK(worst <= 1e-12);
}

TEST_CASE("C16a M3: generation is invariant to workers and point order", "[drivetrain][cycle]") {
    const CycleModelDesc d = fixture_desc();
    const EngineMaps& ref = fixture_maps();
    for (int workers : {1, 4, 10}) {
        jobs::JobSystem pool(workers);
        const EngineMapsPtr m = generate_engine_maps(d, fx::gasoline(), MapQuality::Preview, &pool);
        INFO("workers " << workers);
        CHECK(m->content_hash == ref.content_hash);
        CHECK(same_engine_maps(*m, ref));
    }
    EngineMapGenTask task(d, fx::gasoline(), MapQuality::Preview);
    CHECK(task.point_count() == 51);
    CHECK(task.grid_point_count() == 12);
    for (std::size_t i = task.point_count(); i-- > 0;) task.run_point(i);
    const EngineMapsPtr rev = task.finish();
    CHECK(rev->content_hash == ref.content_hash);
    CHECK(same_engine_maps(*rev, ref));
    CHECK_THROWS(task.finish());
    EngineMapGenTask partial(d, fx::gasoline(), MapQuality::Preview);
    partial.run_point(0);
    CHECK_THROWS(partial.finish());
}

TEST_CASE("C16a M4: the content hash covers every field and nothing else", "[drivetrain][cycle]") {
    const EngineMaps& ref = fixture_maps();
    CHECK(engine_maps_content_hash(ref) == ref.content_hash);
    auto bump = [](real& v) { v = std::nextafter(v, real(1e300)); };
    const std::vector<std::pair<const char*, std::function<void(EngineMaps&)>>> edits = {
        {"ve value", [&](EngineMaps& m) { bump(m.maps.ve.values[5]); }},
        {"ve axis", [&](EngineMaps& m) { bump(m.maps.ve.y[1]); }},
        {"eta_i value", [&](EngineMaps& m) { bump(m.maps.eta_i.values[3]); }},
        {"eta_i air axis", [&](EngineMaps& m) { bump(m.maps.eta_i.y[2]); }},
        {"f_lambda", [&](EngineMaps& m) { bump(m.maps.f_lambda.y[0]); }},
        {"eta_spark", [&](EngineMaps& m) { bump(m.maps.eta_spark.y[0]); }},
        {"theta_mbt", [&](EngineMaps& m) { bump(m.maps.theta_mbt_deg.values[0]); }},
        {"friction", [&](EngineMaps& m) { bump(m.maps.friction_torque_nm.y[2]); }},
        {"pmep_excess", [&](EngineMaps& m) { bump(m.maps.pmep_excess_pa.values[4]); }},
        {"has_pmep_excess", [&](EngineMaps& m) { m.maps.has_pmep_excess = false; }},
        {"t_exh", [&](EngineMaps& m) { bump(m.t_exh_k.values[1]); }},
        {"h_exh", [&](EngineMaps& m) { bump(m.h_exh_w.values[1]); }},
        {"eta_spark_spread", [&](EngineMaps& m) { bump(m.eta_spark_spread); }},
        {"quality", [&](EngineMaps& m) { m.quality = MapQuality::Full; }},
        {"generator_version", [&](EngineMaps& m) { ++m.generator_version; }},
    };
    for (const auto& [name, fn] : edits) {
        EngineMaps m = ref;
        fn(m);
        INFO(name);
        CHECK(engine_maps_content_hash(m) != ref.content_hash);
    }
    EngineMaps m = ref;
    m.maps.source = "a different source text";
    CHECK(engine_maps_content_hash(m) == ref.content_hash);
}

TEST_CASE("C16a M5: loader-time generation, override precedence and cycle-block rejections", "[drivetrain][cycle][io]") {
    SECTION("cycle only: generated once, equal to a direct generate") {
        std::size_t n = 0;
        const vehicle::VehicleDesc v = load_engine(cycle_only_engine(), &n);
        CHECK(n == 1);
        const SimulatedEngineDesc& e = only_sim_engine(v);
        REQUIRE(e.cycle.has_value());
        const CycleModelDesc f = fixture_desc();
        CHECK(std::abs(e.cycle->rod_m - f.rod_m) <= 1e-12);
        CHECK(std::abs(e.cycle->intake_port_m3 - f.intake_port_m3) <= 1e-15);
        CHECK(std::abs(e.cycle->friction.b_pa - f.friction.b_pa) <= 1e-9 * std::max(real(1.0), std::abs(f.friction.b_pa)));
        CHECK(e.cycle->grid_p_im == f.grid_p_im);
        const EngineMapsPtr g = generate_engine_maps(*e.cycle, e.fuels.front(), MapQuality::Preview);
        CHECK(same_maps(e.maps, g->maps));
        CHECK(e.maps.source == g->maps.source);
    }
    SECTION("complete override: the generator is not called, the file's maps are used") {
        json j = base_engine_json();
        j["maps"]["pmep_excess_kpa"] = {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{2.0, 1.0}, {6.0, 4.0}}}};
        json plain = j;
        j["cycle"] = cycle_json(fixture_desc(), true);
        std::size_t n = 0, n_plain = 0;
        const vehicle::VehicleDesc v = load_engine(j, &n);
        const vehicle::VehicleDesc vp = load_engine(plain, &n_plain);
        CHECK(n == 0);
        CHECK(n_plain == 0);
        CHECK(only_sim_engine(v).cycle.has_value());
        CHECK_FALSE(only_sim_engine(vp).cycle.has_value());
        CHECK(same_maps(only_sim_engine(v).maps, only_sim_engine(vp).maps));
        CHECK(only_sim_engine(v).maps.source == only_sim_engine(vp).maps.source);
    }
    SECTION("partial override: the missing maps are generated, the given ones kept") {
        json j = base_engine_json();
        json maps = {{"source", j["maps"]["source"]}, {"ve", j["maps"]["ve"]}, {"friction_torque_nm", j["maps"]["friction_torque_nm"]}};
        j["maps"] = maps;
        j["cycle"] = cycle_json(fixture_desc(), false);
        std::size_t n = 0;
        const vehicle::VehicleDesc v = load_engine(j, &n);
        CHECK(n == 1);
        const SimulatedEngineDesc& e = only_sim_engine(v);
        const vehicle::VehicleDesc vp = load_engine(base_engine_json(), nullptr);
        const SimulatedEngineDesc& ep = only_sim_engine(vp);
        CHECK(same_bits(e.maps.ve, ep.maps.ve));
        CHECK(same_bits(e.maps.friction_torque_nm, ep.maps.friction_torque_nm));
        const EngineMapsPtr g = generate_engine_maps(*e.cycle, e.fuels.front(), MapQuality::Preview);
        CHECK(same_bits(e.maps.eta_i, g->maps.eta_i));
        CHECK(same_bits(e.maps.f_lambda, g->maps.f_lambda));
        CHECK(same_bits(e.maps.eta_spark, g->maps.eta_spark));
        CHECK(same_bits(e.maps.theta_mbt_deg, g->maps.theta_mbt_deg));
        CHECK(e.maps.has_pmep_excess);
        CHECK(same_bits(e.maps.pmep_excess_pa, g->maps.pmep_excess_pa));
        CHECK_THAT(e.maps.source, ContainsSubstring("synthetic test fixture"));
        CHECK_THAT(e.maps.source,
                   ContainsSubstring("generated from the cycle block (eta_i, f_lambda, eta_spark, theta_mbt_deg, pmep_excess_kpa)"));
    }
    SECTION("the shipped car_sedan_sim.json never runs the generator") {
        std::size_t n = 0;
        (void)io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_sim.json", preview_options(&n));
        CHECK(n == 0);
    }
    SECTION("malformed cycle blocks are rejected with their messages") {
        const std::vector<std::pair<std::function<void(json&)>, std::string>> cases = {
            {[](json& c) { c["bore_mm"] = 86.0; }, "'bore_mm' is not allowed here"},
            {[](json& c) { c["cylinders"] = 4; }, "'cylinders' is not allowed here"},
            {[](json& c) { c["back_pressure_kpa_vs_exh_kg_s"] = {{"x", {0.0, 0.1}}, {"y", {0.0, 5.0}}}; },
             "'back_pressure_kpa_vs_exh_kg_s' is not supported yet"},
            {[](json& c) { c["runner_volume_cc"] = 1.0; }, "unknown key 'runner_volume_cc'"},
            {[](json& c) { c.erase("source"); }, "'source' is required"},
            {[](json& c) { c["rod_mm"] = 40.0; }, "'rod_mm' must exceed stroke_mm/2 + |pin_offset_mm|"},
            {[](json& c) { c["compression_ratio"] = 1.0; }, "'compression_ratio' must be > 1"},
            {[](json& c) { c.erase("cd_vs_l_over_d"); }, "'cd_vs_l_over_d' is required"},
            {[](json& c) { c["cd_vs_l_over_d"]["y"][0] = 1.2; }, "every Cd must be in (0, 1]"},
            {[](json& c) { c["cam"] = {{"intake_open_deg", 1.0}}; }, ".cam: unknown key 'intake_open_deg'"},
            {[](json& c) { c["wiebe_m"] = -1.0; }, "'wiebe_m' must be >= 0"},
            {[](json& c) { c["combustion_eff_lean"] = 1.5; }, "'combustion_eff_lean' must be in (0, 1]"},
            {[](json& c) { c["friction"]["model"] = "barnes_moss"; },
             "'model' must be \"polynomial_rpm\" or \"polynomial_piston_speed\""},
            {[](json& c) { c["friction"]["a_kpa"] = -1.0; }, "'a_kpa' must be >= 0 (the Coulomb part)"},
            {[](json& c) { c.erase("friction"); }, "'friction' is required unless maps.friction_torque_nm is given"},
            {[](json& c) { c.erase("grid_rpm"); }, "'grid_rpm' array is required"},
            {[](json& c) { c["grid_rpm"] = {"a", 1000.0}; }, "'grid_rpm' entries must be numbers"},
            {[](json& c) { c["grid_rpm"] = {1000.0}; }, "'grid_rpm' needs at least 2 values"},
            {[](json& c) { c["grid_p_im_kpa"] = {0.0, 50.0}; }, "'grid_p_im_kpa' values must be > 0 and finite"},
            {[](json& c) { c["grid_rpm"] = {2000.0, 1000.0}; }, "'grid_rpm' must be strictly ascending"},
            {[](json& c) { c["intake_lift"].erase("lift_mm_vs_deg"); }, "'lift_mm_vs_deg' is required"},
            {[](json& c) { c["exhaust_lift"]["lift_mm_vs_deg"] = {{"x", {0.0}}, {"y", {0.0}}}; }, "needs at least 2 points"},
            {[](json& c) { c["intake_lift"]["lift_mm_vs_deg"] = {{"x", {0.0, 360.0, 720.0}}, {"y", {0.0, 9.0, 0.0}}}; },
             "the span (last x - first x) must be < 720 deg"},
            {[](json& c) { c["intake_lift"]["lift_mm_vs_deg"]["y"][0] = 0.5; }, "the first and last lift must be exactly 0"},
            {[](json& c) { c["intake_lift"]["lift_mm_vs_deg"]["y"][3] = -0.5; }, "every lift must be >= 0 and finite"},
            // ps_audio A1 (out/audio_plan.md section 1/3): firing_order/
            // firing_angles_deg/banks/bank_angle_deg. cycle_only_engine()'s
            // engine is 4-cylinder, so firing_order defaults to 1-3-4-2 and
            // is not itself under test here - see test_firing_order.cpp for
            // the pure-function coverage (including the non-4-cylinder "no
            // default" case, exercised below outside this table since it
            // needs a different cylinder count, not just a cycle-block key).
            {[](json& c) { c["firing_order"] = {1, 3, 4}; }, "firing_order: expected 4 entries, got 3"},
            {[](json& c) { c["firing_order"] = {1, 3, 4, 5}; }, "cylinder number 5 is out of range [1, 4]"},
            {[](json& c) { c["firing_order"] = {1, 3, 4, 3}; }, "cylinder 3 appears more than once"},
            {[](json& c) { c["firing_order"] = {1, "x", 4, 2}; }, "'firing_order' entries must be integers"},
            {[](json& c) { c["firing_angles_deg"] = {0.0, 180.0}; },
             "firing_angles_deg: expected 4 entries (one per cylinder) or none, got 2"},
            {[](json& c) { c["firing_angles_deg"] = {0.0, 180.0, 360.0, 720.0}; }, "must be finite and in [0, 720)"},
            {[](json& c) { c["firing_angles_deg"] = {0.0, 180.0, 180.0, 540.0}; }, "share the same firing angle"},
            {[](json& c) { c["banks"] = json::array(); }, "'banks' must be a non-empty array of cylinder-number arrays"},
            {[](json& c) { c["banks"] = {{1, 2, 3, 5}}; }, "'banks' cylinder number 5 is out of range [1, 4]"},
            {[](json& c) { c["banks"] = {{1, 2}, {2, 3, 4}}; }, "'banks' cylinder 2 is assigned to more than one bank"},
            {[](json& c) { c["banks"] = {{1, 2, 3}}; }, "'banks' must cover every cylinder exactly once (covered 3 of 4)"},
            {[](json& c) { c["bank_angle_deg"] = "x"; }, "'bank_angle_deg' must be a number"},
        };
        for (const auto& [mutate, msg] : cases) {
            json j = cycle_only_engine();
            mutate(j["cycle"]);
            INFO(msg);
            TempDir td;
            const auto p = td.write("vehicle.json", vehicle_with_engine(j));
            CHECK_THROWS_WITH(io::load_vehicle_json(p.string(), preview_options(nullptr)), ContainsSubstring(msg));
        }
        // a rotary engine cannot carry a piston cycle block
        json r = cycle_only_engine();
        r["architecture"] = "rotary";
        r.erase("bore_mm");
        r.erase("stroke_mm");
        r.erase("cylinders");
        r["rotor_radius_mm"] = 105.0;
        r["eccentricity_mm"] = 15.0;
        r["rotor_width_mm"] = 80.0;
        r["rotors"] = 2;
        {
            TempDir td;
            const auto p = td.write("vehicle.json", vehicle_with_engine(r));
            CHECK_THROWS_WITH(io::load_vehicle_json(p.string(), preview_options(nullptr)),
                              ContainsSubstring("a cycle block needs architecture \"piston_4stroke\""));
        }
        // breakaway below the generated Coulomb part
        json b = cycle_only_engine();
        b["breakaway_torque_nm"] = 0.5 * fixture_maps().maps.friction_torque_nm.y.front();
        {
            TempDir td;
            const auto p = td.write("vehicle.json", vehicle_with_engine(b));
            CHECK_THROWS_WITH(io::load_vehicle_json(p.string(), preview_options(nullptr)),
                              ContainsSubstring("breakaway_torque_nm must be >= maps.friction_torque_nm(0)"));
        }
        // firing_order has no default for any cylinder count other than 4
        // (ps_audio A1, out/audio_plan.md section 3) - a cross-plane and a
        // flat-plane V8 differ, so a 6-cylinder engine must say explicitly.
        json v6 = cycle_only_engine();
        v6["cylinders"] = 6;
        {
            TempDir td;
            const auto p = td.write("vehicle.json", vehicle_with_engine(v6));
            CHECK_THROWS_WITH(io::load_vehicle_json(p.string(), preview_options(nullptr)),
                              ContainsSubstring("'firing_order' is required for 6 cylinders"));
        }
    }
    SECTION("firing_order/firing_angles_deg/banks/bank_angle_deg load and default correctly") {
        // Defaults for the 4-cylinder fixture: firing_order {1,3,4,2}, empty
        // firing_angles_deg (derive), one bank with every cylinder, and
        // bank_angle_deg 0.
        {
            const vehicle::VehicleDesc v = load_engine(cycle_only_engine(), nullptr);
            const auto& cyc = *only_sim_engine(v).cycle;
            CHECK(cyc.firing_order == std::vector<int>{1, 3, 4, 2});
            CHECK(cyc.firing_angles_deg.empty());
            REQUIRE(cyc.banks.size() == 1);
            CHECK(cyc.banks[0] == std::vector<int>{1, 2, 3, 4});
            CHECK(cyc.bank_angle_deg == 0.0);
        }
        // Explicit values round-trip through the loader unchanged.
        {
            json j = cycle_only_engine();
            j["cycle"]["firing_order"] = {1, 3, 4, 2};
            j["cycle"]["firing_angles_deg"] = {0.0, 540.0, 180.0, 360.0};
            j["cycle"]["banks"] = {{1, 3}, {2, 4}};
            j["cycle"]["bank_angle_deg"] = 90.0;
            const vehicle::VehicleDesc v = load_engine(j, nullptr);
            const auto& cyc = *only_sim_engine(v).cycle;
            CHECK(cyc.firing_order == std::vector<int>{1, 3, 4, 2});
            CHECK(cyc.firing_angles_deg == std::vector<real>{0.0, 540.0, 180.0, 360.0});
            REQUIRE(cyc.banks.size() == 2);
            CHECK(cyc.banks[0] == std::vector<int>{1, 3});
            CHECK(cyc.banks[1] == std::vector<int>{2, 4});
            CHECK(cyc.bank_angle_deg == 90.0);
        }
    }
}

TEST_CASE("C16a M6: the maps' content hash seeds state_hash; worker invariance in World", "[drivetrain][cycle]") {
    CycleModelDesc d2 = fixture_desc();
    d2.runner_length_m = 0.30;
    const EngineMapsPtr m2 = generate_engine_maps(d2, fx::gasoline(), MapQuality::Preview);
    REQUIRE(m2->content_hash != fixture_maps().content_hash);

    // Discriminating: one VE node at the far corner (max rpm, max p_im)
    // changes, which no initial state reads (idle sits in the first rpm
    // cells), so two fresh engines differ ONLY through the folded maps hash.
    // Without the fold this CHECK sees equal hashes (sabotage M6).
    {
        const SimulatedEngine ea(engine_on(fixture_maps()));
        SimulatedEngineDesc eb_desc = engine_on(fixture_maps());
        eb_desc.maps.ve.values.back() *= 1.01;
        const SimulatedEngine eb(eb_desc);
        REQUIRE(ea.idle_rpm() == eb.idle_rpm());
        CHECK(ea.fold_state_hash(0x9e3779b97f4a7c15ULL) != eb.fold_state_hash(0x9e3779b97f4a7c15ULL));
    }

    Rig a = make_rig(sim_sedan(engine_on(fixture_maps())), 1);
    Rig b = make_rig(sim_sedan(engine_on(*m2)), 1);
    const std::uint64_t a0 = a.world->state_hash(), b0 = b.world->state_hash();
    CHECK(a0 != b0); // before any step: only the folded content hash differs
    a.world->step();
    b.world->step();
    CHECK(a.world->state_hash() != b.world->state_hash());

    Rig c = make_rig(sim_sedan(engine_on(fixture_maps())), 10);
    CHECK(c.world->state_hash() == a0);
    c.world->step();
    CHECK(c.world->state_hash() == a.world->state_hash());
    for (int t = 1; t < 480; ++t) {
        const real thr = t < 240 ? 0.0 : 0.6;
        a.world->set_control("throttle", thr);
        c.world->set_control("throttle", thr);
        a.world->step();
        c.world->step();
    }
    std::printf("M6 hash after 2 s: 1 worker %016llx, 10 workers %016llx, rpm %.2f\n",
                static_cast<unsigned long long>(a.world->state_hash()), static_cast<unsigned long long>(c.world->state_hash()),
                a.world->powertrain_state(a.vehicle).engine_rpm);
    CHECK(c.world->state_hash() == a.world->state_hash());
    CHECK(a.world->powertrain_state(a.vehicle).engine_state == EngineState::Running);
}

TEST_CASE("C16a M7: stepping on generated maps allocates nothing", "[drivetrain][cycle]") {
    if (!da::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    SECTION("engine alone") {
        SimulatedEngine e(engine_on(fixture_maps()));
        real w = w_of(800.0);
        da::reset();
        da::set_armed(true);
        da::enter_job();
        for (int i = 0; i < 2000; ++i) {
            const real c = (i / 400) % 2 ? 1.0 : 0.0;
            w = crank_step(e, w, c, 0.15, kH);
            (void)e.fold_state_hash(1);
        }
        da::exit_job();
        da::set_armed(false);
        CHECK(da::own_violation_count() == 0);
        CHECK(w > 0.0);
    }
    SECTION("World with the vehicle") {
        Rig r = make_rig(sim_sedan(engine_on(fixture_maps())), 4);
        for (int t = 0; t < 240; ++t) r.world->step();
        da::reset();
        da::set_armed(true);
        for (int t = 0; t < 240; ++t) {
            r.world->set_control("throttle", (t / 60) % 2 ? 0.8 : 0.1);
            r.world->step();
        }
        da::set_armed(false);
        CHECK(da::own_violation_count() == 0);
    }
}

TEST_CASE("C16a M8: generation time report", "[.][diag][drivetrain][cycle]") {
    CycleModelDesc d = fixture_desc();
    d.grid_rpm.clear();
    d.grid_p_im.clear();
    for (int i = 0; i < 12; ++i) d.grid_rpm.push_back(1000.0 + 500.0 * i);
    for (int i = 0; i < 20; ++i) d.grid_p_im.push_back(20000.0 + 5000.0 * i);
    jobs::JobSystem pool(10);
    for (MapQuality q : {MapQuality::Preview, MapQuality::Full}) {
        for (jobs::JobSystem* p : {static_cast<jobs::JobSystem*>(nullptr), &pool}) {
            const auto t0 = std::chrono::steady_clock::now();
            const EngineMapsPtr m = generate_engine_maps(d, fx::gasoline(), q, p);
            const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("M8 %s %s: %.3f s, hash %016llx\n", q == MapQuality::Full ? "Full   " : "Preview",
                        p ? "10 workers" : "1 thread  ", s, static_cast<unsigned long long>(m->content_hash));
            real pmin = 1e300, pmax = -1e300;
            for (real v : m->maps.pmep_excess_pa.values) {
                pmin = std::min(pmin, v);
                pmax = std::max(pmax, v);
            }
            std::printf("    pmep_excess [%.1f, %.1f] Pa, eta_spark_spread %.4g, f_lambda(0.8) %.4f, c_k %.3f N*m\n", pmin, pmax,
                        m->eta_spark_spread, m->maps.f_lambda.evaluate(0.8), m->maps.friction_torque_nm.y.front());
        }
    }
}

TEST_CASE("C16a M1 diag: trapped air vs rpm, step and cycle count", "[.][diag][drivetrain][cycle]") {
    const CycleModelDesc d = fixture_desc();
    const ec::CycleStepper proto(d, fx::gasoline().lhv_j_kg, fx::gasoline().afr_stoich);
    auto air = [&](real rpm, real p, real h, int n) {
        ec::OperatingPoint op;
        op.rpm = rpm;
        op.p_im_pa = p;
        op.t_im_k = SimulatedEngine::kIsaTemperatureK;
        op.p_amb_pa = SimulatedEngine::kIsaPressurePa;
        op.lambda = 1.0;
        op.spark_deg_atdc = mbt_atdc(d);
        ec::StepperSettings st;
        st.h_s = h;
        st.cycles = n;
        return ec::run_operating_point(proto, op, st).m_air_j_cyc;
    };
    for (real p : {22500.0, 30000.0, 90000.0}) {
        for (real rpm = 1600.0; rpm <= 1800.0; rpm += 10.0) {
            std::printf("M1diag p %6.0f rpm %5.0f  prev %.6e  full %.6e  full/2 %.6e  full/4 %.6e  prev-n8 %.6e\n", p, rpm,
                        air(rpm, p, ec::kStepPreview, ec::kCyclesPreview), air(rpm, p, ec::kStepFull, ec::kCyclesFull),
                        air(rpm, p, 0.5 * ec::kStepFull, ec::kCyclesFull), air(rpm, p, 0.25 * ec::kStepFull, ec::kCyclesFull),
                        air(rpm, p, ec::kStepPreview, 8));
        }
    }
}

// ---------------------------------------------------------------------------
// C16a review diagnostics (2026-09-28): one discriminating measurement for
// each of the C16a report's two surprising numbers. Print-only, [.][diag].
namespace {

// F4's Ecotec cycle desc (the same edits as the F4 case).
CycleModelDesc review_ecotec_desc() {
    CycleModelDesc d = ps::test::make_generic_i4_cycle_desc();
    const real b0 = d.bore_m;
    d.bore_m = 0.088;
    d.stroke_m = 0.101;
    d.rod_m = d.rod_m * 0.101 / 0.086;
    d.intake_valve_d_m = d.intake_valve_d_m * 0.088 / b0;
    d.exhaust_valve_d_m = d.exhaust_valve_d_m * 0.088 / b0;
    d.compression_ratio = 11.3;
    return d;
}

struct ReviewPumpSplit {
    real w_pump_acc = 0.0;                // the stepper's own W_total - W_gross [J]
    real w_pump_trap = 0.0;               // trapezoid of p dV over the same window [J]
    real in_valve = 0.0, in_runner = 0.0; // intake stroke: (p_cyl - p_ip) dV, (p_ip - p_im) dV
    real ex_valve = 0.0, ex_port = 0.0;   // exhaust stroke: (p_cyl - p_ep) dV, (p_ep - p_amb) dV
    real ref = 0.0;                       // p_im dV + p_amb dV (0 for a closed stroke pair)
    real qs_in = 0.0, qs_ex = 0.0;        // quasi-steady orifice estimate [J]
};

// One motored point (valves on, combustion off, wall heat on), last cycle
// split into gas-exchange terms, plus an independent quasi-steady estimate:
// incompressible orifice flow of the piston's volume rate through the desc's
// own curtain area n*pi*D*L(theta)*Cd(L/D), upstream density (intake port on
// the intake stroke, cylinder on the exhaust stroke), Delta p capped at
// 0.47 of the upstream pressure (choking). The QS estimate ignores wave
// dynamics and the runner/port volumes entirely.
ReviewPumpSplit review_motored_pump_split(const CycleModelDesc& d, real rpm, real p_amb, real t_im) {
    ec::CycleStepper s(d, ps::test::kCycleFixtureLhv, ps::test::kCycleFixtureAfr);
    s.set_switches(ec::CycleSwitches{true, false, true});
    ec::OperatingPoint op;
    op.rpm = rpm;
    op.p_im_pa = p_amb;
    op.p_amb_pa = p_amb;
    op.t_im_k = t_im;
    op.lambda = 1.0;
    s.reset(op);
    const std::uint64_t n = ec::steps_per_cycle(rpm, ec::kStepFull);
    const std::uint64_t total = n * static_cast<std::uint64_t>(ec::kCyclesFull);
    const std::uint64_t last = total - n;
    const real bdc = s.model().geom.theta_bdc_deg;
    auto qs_dp = [&](real vdot, real lift, real dval, int nv, real rho, real p_up) {
        if (lift <= 0.0) return 0.47 * p_up;
        const real cd = d.cd_vs_l_over_d.evaluate(lift / dval);
        const real a = static_cast<real>(nv) * kPi * dval * lift * cd;
        const real u = vdot / a;
        return std::min(0.5 * rho * u * u, 0.47 * p_up);
    };
    ReviewPumpSplit r;
    for (std::uint64_t k = 0; k < total; ++k) {
        if (k == last) s.reset_accumulators();
        const ec::CycleInputs a = s.evaluate(s.crank_deg(), s.state());
        s.step_aligned(k, n);
        if (k < last) continue;
        const ec::CycleInputs b = s.evaluate(s.crank_deg(), s.state());
        const real th = ec::wrap720(a.theta_deg + 0.5 * 720.0 / static_cast<real>(n));
        if (ec::wrap720(th - bdc) < 360.0) continue; // gross window
        const real dv = b.v_cyl - a.v_cyl;
        const real pc = 0.5 * (a.cyl.p + b.cyl.p);
        r.w_pump_trap += pc * dv;
        const real vdot = std::abs(0.5 * (a.dv_dt + b.dv_dt));
        r.ref += p_amb * dv;
        if (th < 360.0) { // intake stroke, TDC -> BDC
            const real pip = 0.5 * (a.ip.p + b.ip.p);
            r.in_valve += (pc - pip) * dv;
            r.in_runner += (pip - p_amb) * dv;
            const real rho = a.ip.p / (a.ip.r * a.ip.t);
            const real lift = 0.5 * (a.lift_in_m + b.lift_in_m);
            r.qs_in -= qs_dp(vdot, lift, d.intake_valve_d_m, d.intake_valves, rho, a.ip.p) * std::abs(dv);
        } else { // exhaust stroke, BDC -> TDC
            const real pep = 0.5 * (a.ep.p + b.ep.p);
            r.ex_valve += (pc - pep) * dv;
            r.ex_port += (pep - p_amb) * dv;
            const real rho = a.cyl.p / (a.cyl.r * a.cyl.t);
            const real lift = 0.5 * (a.lift_ex_m + b.lift_ex_m);
            r.qs_ex -= qs_dp(vdot, lift, d.exhaust_valve_d_m, d.exhaust_valves, rho, a.cyl.p) * std::abs(dv);
        }
    }
    r.w_pump_acc = s.state().y[ec::kAccWork] - s.state().y[ec::kAccWorkGross];
    return r;
}

} // namespace

TEST_CASE("C16a review diag (b): Ecotec motored pumping split vs a quasi-steady estimate", "[.][diag][drivetrain][cycle]") {
    struct Row { real rpm, bmep_bar, p_amb_bar; };
    const Row rows[] = {{999.70, -0.88419, 0.9705}, {1991.42, -1.08094, 0.9710}, {3000.06, -1.43171, 0.9731},
                        {3986.00, -1.94473, 0.972}, {4997.85, -2.49549, 0.9690}};
    for (real lscale : {1.0, 1e-3}) {
        CycleModelDesc d = review_ecotec_desc();
        d.runner_length_m *= lscale;
        const real v_c = kPi / 4.0 * d.bore_m * d.bore_m * d.stroke_m; // one cylinder
        real sp[5], fmep_f4[5], fmep_qs[5];
        std::printf("RB runner L %.4f m\n", d.runner_length_m);
        for (int i = 0; i < 5; ++i) {
            const ReviewPumpSplit r = review_motored_pump_split(d, rows[i].rpm, rows[i].p_amb_bar * 1e5, 298.15);
            auto kpa = [&](real w) { return w / v_c / 1e3; };
            sp[i] = 2.0 * d.stroke_m * rows[i].rpm / 60.0;
            std::printf("RB rpm %7.1f Sp %5.2f pmep acc %7.2f trap %7.2f kPa | in valve %6.2f runner %6.2f | ex valve %6.2f "
                        "port %6.2f | ref %6.3f | QS in %6.2f ex %6.2f sum %6.2f kPa\n",
                        rows[i].rpm, sp[i], kpa(r.w_pump_acc), kpa(r.w_pump_trap), kpa(r.in_valve), kpa(r.in_runner),
                        kpa(r.ex_valve), kpa(r.ex_port), kpa(r.ref), kpa(r.qs_in), kpa(r.qs_ex), kpa(r.qs_in + r.qs_ex));
            // F4: FMEP = IMEP_net(model) - BMEP. Swapping the model's pumping
            // for the QS pumping shifts FMEP by (W_qs - W_model)/V_c.
            fmep_f4[i] = kF4A + kF4B * sp[i] + kF4C * sp[i] * sp[i];
            fmep_qs[i] = fmep_f4[i] + (r.qs_in + r.qs_ex - r.w_pump_acc) / v_c;
        }
        if (lscale != 1.0) continue;
        real m[3][4] = {};
        for (int i = 0; i < 5; ++i) {
            const real phi[3] = {1.0, sp[i], sp[i] * sp[i]};
            for (int rr = 0; rr < 3; ++rr) {
                for (int c = 0; c < 3; ++c) m[rr][c] += phi[rr] * phi[c];
                m[rr][3] += phi[rr] * fmep_qs[i];
            }
        }
        for (int k = 0; k < 3; ++k)
            for (int rr = k + 1; rr < 3; ++rr) {
                const real f = m[rr][k] / m[k][k];
                for (int c = k; c < 4; ++c) m[rr][c] -= f * m[k][c];
            }
        real coef[3];
        for (int k = 2; k >= 0; --k) {
            real s = m[k][3];
            for (int c = k + 1; c < 3; ++c) s -= m[k][c] * coef[c];
            coef[k] = s / m[k][k];
        }
        for (int i = 0; i < 5; ++i)
            std::printf("RB fmep rpm %7.1f  F4 %7.2f kPa  with QS pumping %7.2f kPa  diff %+6.2f kPa (%+.1f %%)\n", rows[i].rpm,
                        fmep_f4[i] / 1e3, fmep_qs[i] / 1e3, (fmep_qs[i] - fmep_f4[i]) / 1e3,
                        100.0 * (fmep_qs[i] / fmep_f4[i] - 1.0));
        std::printf("RB QS refit a %.1f b %.3f c %.3f  (F4 a %.1f b %.3f c %.3f)\n", coef[0], coef[1], coef[2], kF4A, kF4B, kF4C);
    }
}

TEST_CASE("C16a review diag (a): air ripple at 22.5 kPa vs runner length", "[.][diag][drivetrain][cycle]") {
    const CycleModelDesc base = fixture_desc();
    for (real lscale : {1.0, 2.0, 1e-3}) {
        CycleModelDesc d = base;
        d.runner_length_m *= lscale;
        const ec::CycleStepper proto(d, fx::gasoline().lhv_j_kg, fx::gasoline().afr_stoich);
        std::vector<real> rpm, air;
        for (real r = 1450.0; r <= 2000.0 + 1e-9; r += 5.0) {
            rpm.push_back(r);
            air.push_back(reference_point(proto, d, r, 22500.0).m_air_j_cyc);
        }
        const real a_r = kPi / 4.0 * d.runner_diameter_m * d.runner_diameter_m;
        const real c = std::sqrt(1.4 * 287.05 * SimulatedEngine::kIsaTemperatureK);
        const real f_h = c / (2.0 * kPi) * std::sqrt(a_r / (d.runner_length_m * d.intake_port_m3));
        std::vector<real> peaks;
        real amin = 1e300, amax = -1e300;
        for (std::size_t i = 0; i < air.size(); ++i) {
            amin = std::min(amin, air[i]);
            amax = std::max(amax, air[i]);
            if (i == 0 || i + 1 == air.size()) continue;
            if (air[i] > air[i - 1] && air[i] >= air[i + 1]) {
                const real y0 = air[i - 1], y1 = air[i], y2 = air[i + 1];
                const real den = y0 - 2.0 * y1 + y2;
                const real off = den != 0.0 ? 0.5 * (y0 - y2) / den : 0.0;
                peaks.push_back(rpm[i] + 5.0 * off);
            }
        }
        std::printf("RA L %.5f m: Helmholtz (runner into port volume, valve shut) %.1f Hz; air [%.5e, %.5e] +-%.3f %%; %zu peaks:",
                    d.runner_length_m, f_h, amin, amax, 100.0 * (amax - amin) / (amax + amin), peaks.size());
        for (real p : peaks) std::printf(" %.1f", p);
        std::printf("\n");
        for (std::size_t i = 1; i < peaks.size(); ++i) {
            const real f = 1.0 / (120.0 * (1.0 / peaks[i - 1] - 1.0 / peaks[i]));
            std::printf("RA   peaks %.1f/%.1f rpm -> f = %.1f Hz (order k = 120 f/rpm = %.2f at %.1f rpm)\n", peaks[i - 1], peaks[i], f,
                        120.0 * f / peaks[i], peaks[i]);
        }
    }
}
