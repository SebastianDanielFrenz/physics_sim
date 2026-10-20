// tests/unit/test_burn_law.cpp - the burn law (design C, owner decision 2026-10-19, vault PHYS-079):
// ps/engine_cycle/burn_law.h, its data file data/burn_laws/default.json (ps/io/burn_law_io.h) and the engine loader's
// "burn_law" switch. The fitted constants themselves are checked by test_burn_law_cal.cpp.
//
//   BL1 the data file equals the compiled defaults, field by field, exactly
//   BL2 every loader rejection names the file and the key
//   BL3 at the reference state the law gives the C17a angles (40.3205 / 3.8632 deg) times the scale
//   BL4 the law's directions: hotter/denser-fuel faster, more residual or (k_p = 1) pressure slower, the effective pressure, the speed exponent
//   BL5 the engine loader: constant burn keys rejected on a Standard engine, accepted with "burn_law": "fixed"
//   BL6 the stepper hook: a Standard model's cycle runs the law's angles, a Fixed model the constants
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/engine_cycle/burn_law.h"
#include "ps/engine_cycle/cycle_point.h"
#include "ps/engine_cycle/cycle_stepper.h"
#include "ps/io/burn_law_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
namespace ec = ps::engine_cycle;

namespace {

const std::filesystem::path kSrc = std::filesystem::path(PS_SOURCE_DIR);
const std::filesystem::path kDefault = kSrc / "data" / "burn_laws" / "default.json";

json default_json() {
    std::ifstream in(kDefault);
    REQUIRE(in.good());
    return json::parse(in);
}

// A per-process scratch directory (tests may run in parallel processes).
std::filesystem::path scratch_dir() {
    static const std::filesystem::path d = [] {
        const auto t = std::chrono::steady_clock::now().time_since_epoch().count();
        std::filesystem::path p = std::filesystem::temp_directory_path() / ("ps_burn_law_" + std::to_string(t));
        std::filesystem::create_directories(p);
        return p;
    }();
    return d;
}

std::string write_tmp(const std::string& name, const std::string& text) {
    const std::filesystem::path p = scratch_dir() / name;
    std::ofstream(p) << text;
    return p.generic_string();
}

// The input whose TDC unburned state is the law's reference state (r = 10, phi 1, x_ref, n_ref).
ec::BurnLawInput reference_input(const ec::BurnLawParams& p) {
    ec::BurnLawInput in;
    const real r = 10.0;
    in.rpm = p.reference_rpm;
    in.lambda = 1.0;
    in.volume_ratio = r;
    in.t_ivc_k = p.reference_unburned_temperature_k / std::pow(r, p.compression_exponent - 1.0);
    in.p_ivc_pa = p.reference_unburned_pressure_pa / std::pow(r, p.compression_exponent);
    in.residual_fraction = p.reference_residual_fraction;
    return in;
}

jobs::JobSystem& pool() {
    static jobs::JobSystem p(4);
    return p;
}

ComponentParams load_engine(const json& j) {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &pool();
    o.generate = false;
    return io::load_engine_json_from_string(j.dump(), kSrc / "data" / "engines", o);
}

} // namespace

TEST_CASE("BL1 data/burn_laws/default.json equals the compiled burn law defaults", "[burn_law][cycle][io]") {
    const io::BurnLawFile f = io::load_burn_law_json(kDefault.generic_string());
    const ec::BurnLawParams c; // compiled
    const ec::BurnLawParams& d = f.params;
    CHECK(f.version == int(ec::kBurnLawVersion));
    CHECK(d.reference_rpm == c.reference_rpm);
    CHECK(d.reference_unburned_temperature_k == c.reference_unburned_temperature_k);
    CHECK(d.reference_unburned_pressure_pa == c.reference_unburned_pressure_pa);
    CHECK(d.reference_residual_fraction == c.reference_residual_fraction);
    CHECK(d.development_reference_deg == c.development_reference_deg);
    CHECK(d.rapid_reference_deg == c.rapid_reference_deg);
    CHECK(d.scale == c.scale);
    CHECK(d.rpm_exponent == c.rpm_exponent);
    CHECK(d.flame_speed_exponent == c.flame_speed_exponent);
    CHECK(d.flame_speed_pressure_factor == c.flame_speed_pressure_factor);
    CHECK(d.mk_phi_m == c.mk_phi_m);
    CHECK(d.mk_b_m_m_s == c.mk_b_m_m_s);
    CHECK(d.mk_b_2_m_s == c.mk_b_2_m_s);
    CHECK(d.mk_alpha_0 == c.mk_alpha_0);
    CHECK(d.mk_alpha_phi == c.mk_alpha_phi);
    CHECK(d.mk_beta_0 == c.mk_beta_0);
    CHECK(d.mk_beta_phi == c.mk_beta_phi);
    CHECK(d.mk_t0_k == c.mk_t0_k);
    CHECK(d.mk_p0_pa == c.mk_p0_pa);
    CHECK(d.mk_phi_min == c.mk_phi_min);
    CHECK(d.mk_phi_max == c.mk_phi_max);
    CHECK(d.dilution_k == c.dilution_k);
    CHECK(d.dilution_exponent == c.dilution_exponent);
    CHECK(d.min_dilution_factor == c.min_dilution_factor);
    CHECK(d.min_flame_speed_m_s == c.min_flame_speed_m_s);
    CHECK(d.compression_exponent == c.compression_exponent);
    CHECK(d.pressure_exponent == c.pressure_exponent);
    // 27 parameters + format/model/version/source: a key added to the struct must also appear in the file.
    static_assert(sizeof(ec::BurnLawParams) == 27 * sizeof(real), "a new BurnLawParams field needs a row here and in the file");
    CHECK(default_json().size() == 31);
}

TEST_CASE("BL2 the burn law loader rejects malformed files", "[burn_law][cycle][io]") {
    const json base = default_json();
    struct Case {
        const char* name;
        void (*edit)(json&);
        const char* message;
    };
    const Case cases[] = {
        {"unknown top-level key", [](json& j) { j["scale_typo"] = 1.0; }, "unknown key 'scale_typo'"},
        {"unknown parameter key", [](json& j) { j["scale"]["unit"] = "-"; }, "scale: unknown key 'unit'"},
        {"missing parameter", [](json& j) { j.erase("rpm_exponent"); }, "'rpm_exponent' must be {"},
        {"bare number", [](json& j) { j["mk_t0_k"] = 298.0; }, "'mk_t0_k' must be {"},
        {"value not a number", [](json& j) { j["scale"]["value"] = "1"; }, "scale.value must be a number"},
        {"source without a tag", [](json& j) { j["dilution_k"]["source"] = "Rhodes & Keck"; }, "dilution_k.source is required"},
        {"missing source", [](json& j) { j["dilution_k"].erase("source"); }, "dilution_k.source is required"},
        {"wrong format", [](json& j) { j["format"] = "physics_sim.burn_law/2"; }, "format must be"},
        {"wrong model", [](json& j) { j["model"] = "wiebe_constant"; }, "model must be"},
        {"wrong version", [](json& j) { j["version"] = 99; }, "is not the compiled burn law version"},
        {"missing file source", [](json& j) { j.erase("source"); }, "'source' is required"},
        {"invalid value", [](json& j) { j["compression_exponent"]["value"] = 2.0; }, "compression_exponent must be in [1, 1.67]"},
        {"non-positive scale", [](json& j) { j["scale"]["value"] = 0.0; }, "scale must be > 0"},
        {"inverted phi range", [](json& j) { j["mk_phi_max"]["value"] = 0.7; }, "mk_phi_max must be > mk_phi_min"},
        {"pressure factor above 1", [](json& j) { j["flame_speed_pressure_factor"]["value"] = 1.5; }, "flame_speed_pressure_factor must be <= 1"},
        {"negative pressure factor", [](json& j) { j["flame_speed_pressure_factor"]["value"] = -0.1; }, "flame_speed_pressure_factor must be >= 0"},
        {"pressure exponent below -1", [](json& j) { j["pressure_exponent"]["value"] = -1.5; }, "pressure_exponent must be in [-1, 1]"},
    };
    for (const Case& c : cases) {
        json j = base;
        c.edit(j);
        const std::string path = write_tmp(std::string("bl2_") + std::to_string(&c - cases) + ".json", j.dump(2));
        INFO(c.name);
        CHECK_THROWS_WITH(io::load_burn_law_json(path), ContainsSubstring(c.message) && ContainsSubstring(path));
    }
    CHECK_THROWS_WITH(io::load_burn_law_json(write_tmp("bl2_bad.json", "{ not json")), ContainsSubstring("is not valid JSON"));
    CHECK_THROWS_WITH(io::load_burn_law_json((scratch_dir() / "missing.json").generic_string()), ContainsSubstring("cannot open"));
    // The unmodified file loads.
    CHECK_NOTHROW(io::load_burn_law_json(write_tmp("bl2_ok.json", base.dump(2))));
}

TEST_CASE("BL3 the burn law gives the C17a constants at its reference state, times the scale", "[burn_law][cycle]") {
    const real a = 5.0, m = 2.0;
    for (real scale : {1.0, 0.8, 1.5}) {
        ec::BurnLawParams p;
        p.scale = scale;
        INFO("scale " << scale);
        const ec::BurnAngles ref = ec::reference_burn_angles(p, a, m);
        // C17a (vault PHYS-055): rapid 20 / (q90 - q10) = 40.3205 deg, delay 15 - q10 * 40.3205 = 3.8632 deg.
        CHECK(ref.wiebe_duration_deg / scale == Catch::Approx(40.3205).margin(5e-5));
        CHECK(ref.ignition_delay_deg / scale == Catch::Approx(3.8632).margin(5e-5));
        CHECK(ref.development_deg == Catch::Approx(15.0 * scale).epsilon(1e-14));
        CHECK(ref.rapid_deg == Catch::Approx(20.0 * scale).epsilon(1e-14));
        CHECK_FALSE(ref.delay_clamped);
        // MBT = f (15 + 0.4866 * 20) - 8 deg BTDC.
        CHECK(ec::mbt_spark_btdc(ref, a, m, 8.0) == Catch::Approx(24.7310 * scale - 8.0).margin(2e-3));
        // The law evaluated at an input whose TDC state is the reference state is the reference.
        const ec::BurnAngles at = ec::evaluate_burn_law(p, a, m, reference_input(p));
        CHECK(at.wiebe_duration_deg == Catch::Approx(ref.wiebe_duration_deg).epsilon(1e-12));
        CHECK(at.ignition_delay_deg == Catch::Approx(ref.ignition_delay_deg).epsilon(1e-12));
        CHECK(at.t_unburned_k == Catch::Approx(p.reference_unburned_temperature_k).epsilon(1e-12));
        CHECK(at.p_unburned_pa == Catch::Approx(p.reference_unburned_pressure_pa).epsilon(1e-12));
    }
}

TEST_CASE("BL4 the burn law's directions", "[burn_law][cycle]") {
    const real a = 5.0, m = 2.0;
    ec::BurnLawParams p;
    p.rpm_exponent = 0.3;
    p.flame_speed_exponent = 0.6;
    p.flame_speed_pressure_factor = 1.0; // the Metghalchi-Keck pressure dependence as measured (shipped: 0, below)
    // Burn law v3 (2026-10-20): the burn's own pressure term is off here, so the pressure directions below are the
    // flame speed's alone; its own direction is checked at the end. (The v3 default -0.2485 failed lines "p_ivc x 1.5
    // slower" and "k_p = 0 is pressure neutral" - by design, not a defect: 2^-0.2485 = 53.06 / 63.03 deg.)
    p.pressure_exponent = 0.0;
    const ec::BurnLawInput ref = reference_input(p);
    const real d0 = ec::evaluate_burn_law(p, a, m, ref).wiebe_duration_deg;
    auto dur = [&](auto edit) {
        ec::BurnLawInput in = ref;
        edit(in);
        return ec::evaluate_burn_law(p, a, m, in).wiebe_duration_deg;
    };
    CHECK(dur([](ec::BurnLawInput& in) { in.t_ivc_k *= 1.1; }) < d0);           // hotter charge: faster flame
    CHECK(dur([](ec::BurnLawInput& in) { in.p_ivc_pa *= 1.5; }) > d0);          // beta < 0: denser, slower laminar flame
    CHECK(dur([](ec::BurnLawInput& in) { in.residual_fraction = 0.25; }) > d0); // dilution slows it
    CHECK(dur([](ec::BurnLawInput& in) { in.residual_fraction = 0.0; }) < d0);
    CHECK(dur([](ec::BurnLawInput& in) { in.lambda = 1.0 / 1.13; }) < d0);      // the Metghalchi-Keck peak phi 1.13
    CHECK(dur([](ec::BurnLawInput& in) { in.lambda = 1.3; }) > d0);             // lean: slower
    CHECK(dur([](ec::BurnLawInput& in) { in.rpm = 4000.0; }) == Catch::Approx(d0 * std::pow(2.0, 0.3)).epsilon(1e-12));
    // phi is clamped into the correlation's range [0.8, 1.5]: a rich WOT lambda 0.6 burns like phi 1.5, a lean 1.4 like 0.8.
    CHECK(dur([](ec::BurnLawInput& in) { in.lambda = 0.6; }) == Catch::Approx(dur([](ec::BurnLawInput& in) { in.lambda = 1.0 / 1.5; })).epsilon(1e-12));
    CHECK(dur([](ec::BurnLawInput& in) { in.lambda = 1.4; }) == Catch::Approx(dur([](ec::BurnLawInput& in) { in.lambda = 1.0 / 0.8; })).epsilon(1e-12));
    CHECK(dur([](ec::BurnLawInput& in) { in.lambda = 0.7; }) > dur([](ec::BurnLawInput& in) { in.lambda = 0.8; }));
    // The flame-speed exponent: f proportional to (S_ref / S)^e_s.
    {
        ec::BurnLawInput hot = ref;
        hot.t_ivc_k *= 1.2;
        const ec::BurnAngles b = ec::evaluate_burn_law(p, a, m, hot);
        const real s_ref = ec::evaluate_burn_law(p, a, m, ref).flame_speed_m_s;
        CHECK(b.wiebe_duration_deg == Catch::Approx(d0 * std::pow(s_ref / b.flame_speed_m_s, 0.6)).epsilon(1e-12));
    }
    // Guards: a fully diluted charge stays finite (dilution and flame speed floors).
    const ec::BurnAngles dil = ec::evaluate_burn_law(p, a, m, [&] {
        ec::BurnLawInput in = ref;
        in.residual_fraction = 0.9;
        return in;
    }());
    CHECK(std::isfinite(dil.wiebe_duration_deg));
    // The effective pressure p_eff = p_ref (p_u / p_ref)^k_p: k_p = 0 (shipped) makes the pressure neutral, k_p = 0.5 halves
    // the log-effect of k_p = 1 on S_L; the reported unburned pressure stays the actual one.
    {
        ec::BurnLawInput dense = ref;
        dense.p_ivc_pa *= 2.0;
        ec::BurnLawParams pk = p;
        pk.flame_speed_pressure_factor = 0.0;
        const ec::BurnAngles b0 = ec::evaluate_burn_law(pk, a, m, dense);
        CHECK(b0.wiebe_duration_deg == Catch::Approx(ec::evaluate_burn_law(pk, a, m, ref).wiebe_duration_deg).epsilon(1e-14));
        CHECK(b0.p_unburned_pa == Catch::Approx(2.0 * pk.reference_unburned_pressure_pa).epsilon(1e-12));
        pk.flame_speed_pressure_factor = 0.5;
        const real s_half = ec::evaluate_burn_law(pk, a, m, dense).flame_speed_m_s;
        const real s_full = ec::evaluate_burn_law(p, a, m, dense).flame_speed_m_s;
        CHECK(std::log(s_half / b0.flame_speed_m_s) == Catch::Approx(0.5 * std::log(s_full / b0.flame_speed_m_s)).epsilon(1e-10));
        CHECK(s_full < s_half);
        CHECK(s_half < b0.flame_speed_m_s); // beta < 0 at phi 1
    }
    CHECK(dil.flame_speed_m_s >= p.min_flame_speed_m_s);
    // With e_s = 0 the flame speed has no influence.
    ec::BurnLawParams p0 = p;
    p0.flame_speed_exponent = 0.0;
    ec::BurnLawInput hot = ref;
    hot.t_ivc_k *= 1.3;
    CHECK(ec::evaluate_burn_law(p0, a, m, hot).wiebe_duration_deg == Catch::Approx(ec::evaluate_burn_law(p0, a, m, ref).wiebe_duration_deg).epsilon(1e-14));
    // Burn law v3: the burn's own pressure term f *= (p_u / p_ref)^e_p. With the flame speed's pressure dependence off,
    // doubling p_ivc (a factor 2 on p_u along the isentrope) scales the duration by exactly 2^e_p; the shipped e_p < 0
    // burns a denser charge faster. Recorded sabotage (2026-10-20): the factor's exponent sign flipped in burn_law.cpp
    // fails this check (63.03 * 2^+0.2485 = 74.88 vs 53.06 deg).
    {
        ec::BurnLawParams pe = p;
        pe.flame_speed_pressure_factor = 0.0;
        pe.pressure_exponent = ec::BurnLawParams{}.pressure_exponent;
        REQUIRE(pe.pressure_exponent < 0.0);
        ec::BurnLawInput dense = ref;
        dense.p_ivc_pa *= 2.0;
        const real d_ref = ec::evaluate_burn_law(pe, a, m, ref).wiebe_duration_deg;
        CHECK(ec::evaluate_burn_law(pe, a, m, dense).wiebe_duration_deg == Catch::Approx(d_ref * std::pow(2.0, pe.pressure_exponent)).epsilon(1e-12));
    }
}

TEST_CASE("BL5 the engine loader: constant burn keys need \"burn_law\": \"fixed\"", "[burn_law][cycle][io]") {
    json j;
    j["extends"] = "toyota_a25a_fks_gen.json";
    SECTION("the shipped engine runs the law") {
        const auto e = std::get<SimulatedEngineDesc>(load_engine(j));
        REQUIRE(e.cycle.has_value());
        CHECK(e.cycle->burn_law == CycleModelDesc::BurnLawMode::Standard);
        CHECK(e.cycle->burn_law_params.scale == ec::BurnLawParams{}.scale);
        CHECK(engine_map_generator_version(*e.cycle) == kLawLumpedGeneratorVersion);
    }
    SECTION("burn_duration_deg on a Standard engine is rejected") {
        j["cycle"]["burn_duration_deg"] = 40.0;
        CHECK_THROWS_WITH(load_engine(j), ContainsSubstring("burn_duration_deg") && ContainsSubstring("\"burn_law\": \"fixed\""));
    }
    SECTION("ignition_delay_deg on a Standard engine is rejected") {
        j["cycle"]["ignition_delay_deg"] = 3.0;
        CHECK_THROWS_WITH(load_engine(j), ContainsSubstring("ignition_delay_deg") && ContainsSubstring("\"burn_law\": \"fixed\""));
    }
    SECTION("an explicit \"standard\" is the default") {
        j["cycle"]["burn_law"] = "standard";
        CHECK(std::get<SimulatedEngineDesc>(load_engine(j)).cycle->burn_law == CycleModelDesc::BurnLawMode::Standard);
    }
    SECTION("\"fixed\" takes the constants and the pre-law generator version") {
        j["cycle"]["burn_law"] = "fixed";
        j["cycle"]["burn_duration_deg"] = 40.3205;
        j["cycle"]["ignition_delay_deg"] = 3.8632;
        const auto e = std::get<SimulatedEngineDesc>(load_engine(j));
        CHECK(e.cycle->burn_law == CycleModelDesc::BurnLawMode::Fixed);
        CHECK(e.cycle->burn_duration_deg == 40.3205);
        CHECK(e.cycle->ignition_delay_deg == 3.8632);
        CHECK(engine_map_generator_version(*e.cycle) == kLumpedGeneratorVersion);
    }
    SECTION("an unknown mode is rejected") {
        j["cycle"]["burn_law"] = "constant";
        CHECK_THROWS_WITH(load_engine(j), ContainsSubstring("burn_law"));
    }
}

TEST_CASE("BL6 the stepper runs the law's angles on a Standard model and the constants on a Fixed one", "[burn_law][cycle]") {
    json j;
    j["extends"] = "toyota_a25a_fks_gen.json";
    const auto e = std::get<SimulatedEngineDesc>(load_engine(j));
    CycleModelDesc std_desc = *e.cycle;
    CycleModelDesc fix_desc = std_desc;
    fix_desc.burn_law = CycleModelDesc::BurnLawMode::Fixed;
    fix_desc.burn_duration_deg = 40.3205;
    fix_desc.ignition_delay_deg = 3.8632;
    const FuelDesc& fuel = e.fuels[0];
    const ec::CycleStepper ps_std(std_desc, fuel.lhv_j_kg, fuel.afr_stoich);
    const ec::CycleStepper ps_fix(fix_desc, fuel.lhv_j_kg, fuel.afr_stoich);
    ec::OperatingPoint op;
    op.rpm = 2500.0;
    op.p_im_pa = 50000.0;
    op.t_im_k = 300.0;
    op.lambda = 1.0;
    op.spark_mode = ec::OperatingPoint::SparkMode::Spark;
    op.spark_deg_atdc = -25.0;
    const ec::StepperSettings st{ec::kStepPreview, ec::kCyclesPreview};
    const ec::PointResult rs = ec::run_operating_point(ps_std, op, st);
    const ec::PointResult rf = ec::run_operating_point(ps_fix, op, st);
    // Fixed: the constants, spark + the constant delay.
    CHECK(rf.burn.wiebe_duration_deg == 40.3205);
    CHECK(rf.burn.ignition_delay_deg == 3.8632);
    // Standard: the law at this cycle's own IVC state (recomputed here from the captured state).
    const ec::CvState c = ec::cv_state(ps_std.model().gas, rs.ivc.m, rs.ivc.u, rs.ivc.mb, ps_std.model().geom.eval(rs.ivc.crank_deg).v);
    ec::BurnLawInput in;
    in.rpm = op.rpm;
    in.lambda = op.lambda;
    in.t_ivc_k = c.t;
    in.p_ivc_pa = c.p;
    in.residual_fraction = c.x;
    in.volume_ratio = ps_std.model().geom.eval(rs.ivc.crank_deg).v / ps_std.model().geom.v_c;
    const ec::BurnAngles law = ec::evaluate_burn_law(std_desc.burn_law_params, std_desc.wiebe_a, std_desc.wiebe_m, in);
    CHECK(rs.burn.wiebe_duration_deg == law.wiebe_duration_deg);
    CHECK(rs.burn.ignition_delay_deg == law.ignition_delay_deg);
    CHECK(rs.burn.wiebe_duration_deg != rf.burn.wiebe_duration_deg);
    // Spark mode starts the Wiebe at the spark plus the cycle's delay (law or constant): the start implied by the cycle's
    // own CA50 and duration.
    const real a = std_desc.wiebe_a, m = std_desc.wiebe_m;
    CHECK(ec::mbt_wiebe_start_atdc(rs.burn, a, m, rs.ca50_deg) == Catch::Approx(-25.0 + rs.burn.ignition_delay_deg).margin(1e-9));
    CHECK(ec::mbt_wiebe_start_atdc(rf.burn, a, m, rf.ca50_deg) == Catch::Approx(-25.0 + 3.8632).margin(1e-9));
    // MbtOffset 0 puts CA50 at the MBT rule's 8 deg ATDC with the law's own duration.
    op.spark_mode = ec::OperatingPoint::SparkMode::MbtOffset;
    op.spark_deg_atdc = 0.0;
    const ec::PointResult rm = ec::run_operating_point(ps_std, op, st);
    CHECK(rm.ca50_deg == Catch::Approx(std_desc.ca50_mbt_deg_atdc).margin(1e-9));
    CHECK(rm.mbt_spark_btdc == Catch::Approx(ec::mbt_spark_btdc(rm.burn, std_desc.wiebe_a, std_desc.wiebe_m, std_desc.ca50_mbt_deg_atdc)).epsilon(1e-14));
}
