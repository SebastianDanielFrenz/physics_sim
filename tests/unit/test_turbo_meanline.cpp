// test_turbo_meanline.cpp - the C++ stage-map generator (ps/drivetrain/turbo_meanline.h) against its Python reference
// (tools/turbo_sim/meanline.py; fixtures written by tests/fixtures/turbo_meanline/make_reference.py), plus determinism,
// JSON round trip, the loader path and a widened sampling range.
//
// Acceptance bands (written before the first run):
//  * every point's status equals the Python's (0 mismatches over 4 cases, ~1200 compressor points);
//  * every compared number within 1e-9 relative (floors below), the ONLY expected difference being ps::math vs libm
//    transcendentals (<= 1 ulp per call) through 55-step bisections that resolve 2^-55: [D] ~1e-14 expected;
//    continuity residuals (noise around 1e-12 kg/s) within 1e-9 kg/s absolute;
//  * an adaptive map's refined axes BITWISE equal to the Python's (they are midpoints of exact knots) and its sampling
//    block (passes, budget_limited, audit counts) equal; audit maximum errors within 1e-9 relative;
//  * inline vs 4 workers bitwise identical; JSON round trip bitwise identical to the in-memory maps.
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/turbo_meanline.h"
#include "ps/drivetrain/turbo_stage_maps.h"
#include "ps/io/turbo_stage_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {
using json = nlohmann::json;
namespace ml = ps::drivetrain::meanline;
using ps::real;

std::string read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}
std::filesystem::path fixture_dir() { return std::filesystem::path(PS_SOURCE_DIR) / "tests/fixtures/turbo_meanline"; }

ml::StageGeometry geometry_of(const json& profile, const std::string& label) {
    return ps::io::parse_turbo_geometry_json(profile.dump(), label);
}

bool close(real a, real b, real floor) { return std::abs(a - b) <= real(1e-9) * std::max(std::abs(b), floor); }

const json& cases() {
    static const json j = json::parse(read_file(fixture_dir() / "reference_cases.json"));
    return j.at("cases");
}

ml::StageMapResult run_case(const json& c, ps::jobs::JobSystem* pool) {
    const ml::StageGeometry g = geometry_of(c.at("geometry"), c.at("name").get<std::string>());
    const json& s = c.at("settings");
    if (s.at("kind") == "uniform") {
        const auto a = s.at("axes");
        return ml::generate(g, ml::uniform_sampling(g, a[0].get<real>(), a[1].get<int>(), a[2].get<real>(), a[3].get<int>(),
                                                    a[4].get<real>(), a[5].get<int>()), pool);
    }
    ml::Sampling init{s.at("speeds").get<std::vector<real>>(), s.at("flows").get<std::vector<real>>(),
                      s.at("expansions").get<std::vector<real>>()};
    ml::AdaptiveOptions o;
    o.tolerance = s.at("tolerance").get<real>();
    o.max_passes = s.at("max_passes").get<int>();
    o.max_speed_lines = s.at("max_speed_lines").get<int>();
    o.max_flow_points = s.at("max_flow_points").get<int>();
    o.max_expansion_points = s.at("max_expansion_points").get<int>();
    return ml::adaptive_generate(g, init, o, pool);
}

real comp_field(const ml::CompressorPoint& p, const std::string& f) {
    if (f == "pressure_ratio") return p.pressure_ratio;
    if (f == "specific_work_j_kg") return p.specific_work_j_kg;
    if (f == "efficiency") return p.efficiency;
    if (f == "outlet_total_k") return p.outlet_total_k;
    if (f == "inlet_mach") return p.inlet_mach;
    if (f == "inlet_relative_mach") return p.inlet_relative_mach;
    if (f == "exit_relative_mach") return p.exit_relative_mach;
    if (f == "shock_pressure_retention") return p.shock_pressure_retention;
    if (f == "continuity_residual_kg_s") return p.continuity_residual_kg_s;
    FAIL("unknown field " << f);
    return 0;
}
real comp_floor(const std::string& f) {
    if (f == "specific_work_j_kg") return 1000;
    if (f == "outlet_total_k") return 1;
    return 1; // ratios, efficiencies, Mach numbers
}
real turb_field(const ml::TurbinePoint& p, const std::string& f) {
    if (f == "mass_flow_kg_s") return p.mass_flow_kg_s;
    if (f == "specific_work_j_kg") return p.specific_work_j_kg;
    if (f == "specific_torque_nm_per_kg_s") return p.specific_torque_nm_per_kg_s;
    if (f == "efficiency") return p.efficiency;
    if (f == "outlet_total_k") return p.outlet_total_k;
    FAIL("unknown field " << f);
    return 0;
}
real turb_floor(const std::string& f) {
    if (f == "mass_flow_kg_s") return 1e-3;
    if (f == "specific_work_j_kg") return 1000;
    return 1;
}

bool same_maps(const ps::drivetrain::TurboStageMaps& a, const ps::drivetrain::TurboStageMaps& b) {
    const auto eq = [](const ps::drivetrain::Table2D& x, const ps::drivetrain::Table2D& y) {
        return x.x == y.x && x.y == y.y && x.values == y.values;
    };
    return eq(a.compressor_pr, b.compressor_pr) && eq(a.compressor_work_j_kg, b.compressor_work_j_kg) &&
           eq(a.turbine_flow_kg_s, b.turbine_flow_kg_s) && eq(a.turbine_work_j_kg, b.turbine_work_j_kg) &&
           eq(a.turbine_torque_nm_per_kg_s, b.turbine_torque_nm_per_kg_s) && a.compressor_valid == b.compressor_valid &&
           a.reference_pa == b.reference_pa && a.compressor_cp == b.compressor_cp && a.air_kappa == b.air_kappa &&
           a.exhaust_gamma == b.exhaust_gamma && a.exhaust_r == b.exhaust_r && a.units == b.units && a.source == b.source &&
           a.solver == b.solver;
}
} // namespace

TEST_CASE("meanline C++ port matches the Python reference point by point", "[turbo_meanline][turbo_stage][drivetrain]") {
    REQUIRE(cases().size() == 4);
    for (const json& c : cases()) {
        const std::string name = c.at("name").get<std::string>();
        INFO(name);
        const ml::StageMapResult r = run_case(c, nullptr);
        if (c.at("settings").at("kind") == "adaptive") {
            // Refined axes are midpoints of exact knots: bitwise equal, or the refinement decisions differ.
            CHECK(r.axes.corrected_rpm == c.at("corrected_rpm").get<std::vector<real>>());
            CHECK(r.axes.flow_kg_s == c.at("compressor_corrected_flow_kg_s").get<std::vector<real>>());
            CHECK(r.axes.expansion_ratio == c.at("turbine_expansion_ratio").get<std::vector<real>>());
            const json& s = c.at("sampling");
            CHECK(r.sampling.passes == s.at("passes").get<int>());
            CHECK(r.sampling.budget_limited == s.at("budget_limited").get<bool>());
            for (const char* st : {"compressor", "turbine"}) {
                const ml::StageAudit& a = std::string(st) == "compressor" ? r.sampling.compressor : r.sampling.turbine;
                const json& pa = s.at("audit").at(st);
                CHECK(a.cell_centers_checked == pa.at("cell_centers_checked").get<int>());
                CHECK(a.coverage_boundary_cells == pa.at("coverage_boundary_cells").get<int>());
                CHECK(close(a.maximum_relative_error, pa.at("maximum_relative_error").get<real>(), 1e-6));
            }
        } else {
            // Uniform axes come from the same formulas (one pow per expansion knot): within the band, not bitwise.
            const auto py_er = c.at("turbine_expansion_ratio").get<std::vector<real>>();
            REQUIRE(r.axes.expansion_ratio.size() == py_er.size());
            for (std::size_t i = 0; i < py_er.size(); ++i) CHECK(close(r.axes.expansion_ratio[i], py_er[i], 1));
            CHECK(r.axes.corrected_rpm == c.at("corrected_rpm").get<std::vector<real>>());
            CHECK(r.axes.flow_kg_s == c.at("compressor_corrected_flow_kg_s").get<std::vector<real>>());
        }
        const auto fields = c.at("compressor_fields").get<std::vector<std::string>>();
        const json& pc = c.at("compressor");
        REQUIRE(r.compressor.size() == pc.size());
        int status_mismatch = 0, value_mismatch = 0;
        real worst = 0; // over the band-checked fields, relative to max(|python|, floor)
        for (std::size_t i = 0; i < pc.size(); ++i) {
            const ml::CompressorPoint& p = r.compressor[i];
            if (pc[i][0].get<std::string>() != ml::to_string(p.status)) {
                ++status_mismatch;
                continue;
            }
            for (std::size_t k = 1; k < fields.size(); ++k) {
                if (pc[i][k].is_null()) continue; // not reported for this status
                const real py = pc[i][k].get<real>(), cc = comp_field(p, fields[k]);
                const bool resid = fields[k] == "continuity_residual_kg_s";
                const bool ok = resid ? std::abs(cc - py) <= 1e-9 : close(cc, py, comp_floor(fields[k]));
                if (!resid) worst = std::max(worst, std::abs(cc - py) / std::max(std::abs(py), comp_floor(fields[k])));
                if (!ok) {
                    if (value_mismatch < 5) UNSCOPED_INFO("point " << i << " " << fields[k] << " c++ " << cc << " python " << py);
                    ++value_mismatch;
                }
            }
        }
        const auto tfields = c.at("turbine_fields").get<std::vector<std::string>>();
        const json& pt = c.at("turbine");
        REQUIRE(r.turbine.size() == pt.size());
        for (std::size_t i = 0; i < pt.size(); ++i) {
            for (std::size_t k = 0; k < tfields.size(); ++k) {
                const real py = pt[i][k].get<real>(), cc = turb_field(r.turbine[i], tfields[k]);
                worst = std::max(worst, std::abs(cc - py) / std::max(std::abs(py), turb_floor(tfields[k])));
                if (!close(cc, py, turb_floor(tfields[k]))) ++value_mismatch;
            }
        }
        std::printf("meanline cross-check %s: %zu compressor + %zu turbine points, worst relative difference %.3g\n", name.c_str(),
                    r.compressor.size(), r.turbine.size(), worst);
        CHECK(status_mismatch == 0);
        CHECK(value_mismatch == 0);
    }
}

TEST_CASE("meanline: the Python's map file and the C++ maps load to the same TurboStageMaps", "[turbo_meanline][turbo_stage][drivetrain]") {
    const std::string python_text = read_file(fixture_dir() / "standard_uniform.json");
    const auto from_python = ps::io::turbo_stage_maps_from_json(python_text, 2, "standard_uniform.json");
    const ml::StageGeometry g = ps::io::parse_turbo_geometry_json(python_text, "standard_uniform.json"); // the embedded geometry
    const ml::StageMapResult r = ml::generate(g, ml::uniform_sampling(g, 240000, 9, 0.9, 25, 40, 9));
    const auto from_cpp = ml::to_stage_maps(r, 2);
    CHECK(from_cpp->compressor_valid == from_python->compressor_valid);
    CHECK(from_cpp->compressor_pr.x == from_python->compressor_pr.x);
    CHECK(from_cpp->compressor_pr.y == from_python->compressor_pr.y);
    int bad = 0;
    for (std::size_t i = 0; i < from_cpp->compressor_pr.values.size(); ++i) {
        bad += !close(from_cpp->compressor_pr.values[i], from_python->compressor_pr.values[i], 1);
        bad += !close(from_cpp->compressor_work_j_kg.values[i], from_python->compressor_work_j_kg.values[i], 1000);
    }
    for (std::size_t i = 0; i < from_cpp->turbine_flow_kg_s.values.size(); ++i) {
        bad += !close(from_cpp->turbine_flow_kg_s.values[i], from_python->turbine_flow_kg_s.values[i], 1e-3);
        bad += !close(from_cpp->turbine_torque_nm_per_kg_s.values[i], from_python->turbine_torque_nm_per_kg_s.values[i], 1);
    }
    CHECK(bad == 0);
    CHECK(from_cpp->source == from_python->source);
    CHECK(from_cpp->solver == from_python->solver);
    CHECK(from_cpp->compressor_cp == from_python->compressor_cp);
    CHECK(from_cpp->air_kappa == from_python->air_kappa);

    // C++ JSON -> the loader's parse: bitwise the in-memory maps (nlohmann round-trips doubles exactly).
    const std::uint64_t key = ml::cache_key(g, r.axes, nullptr);
    const std::string text = ps::io::turbo_stage_maps_to_json(r, key);
    CHECK(same_maps(*ps::io::turbo_stage_maps_from_json(text, 2, "generated"), *from_cpp));
    // Its embedded geometry regenerates the same map.
    const ml::StageGeometry again = ps::io::parse_turbo_geometry_json(text, "generated");
    CHECK(again.profile_json == g.profile_json);
    CHECK(ml::cache_key(again, r.axes, nullptr) == key);
}

TEST_CASE("meanline: worker count, cache key and input validation", "[turbo_meanline][turbo_stage][drivetrain]") {
    const json& adaptive = cases()[3];
    REQUIRE(adaptive.at("settings").at("kind") == "adaptive");
    const ml::StageMapResult inline_r = run_case(adaptive, nullptr);
    ps::jobs::JobSystem pool(4);
    const ml::StageMapResult pooled = run_case(adaptive, &pool);
    CHECK(same_maps(*ml::to_stage_maps(inline_r, 1), *ml::to_stage_maps(pooled, 1)));
    CHECK(ps::io::turbo_stage_maps_to_json(inline_r, 1) == ps::io::turbo_stage_maps_to_json(pooled, 1)); // every field

    const ml::StageGeometry g = geometry_of(adaptive.at("geometry"), "large");
    const ml::Sampling s = ml::uniform_sampling(g, 200000, 9, 1.3, 13, 40, 9);
    ml::Sampling wider = s;
    wider.flow_kg_s.push_back(1.5);
    ml::AdaptiveOptions o;
    CHECK(ml::cache_key(g, s, nullptr) == ml::cache_key(g, s, nullptr));
    CHECK(ml::cache_key(g, s, nullptr) != ml::cache_key(g, wider, nullptr));
    CHECK(ml::cache_key(g, s, nullptr) != ml::cache_key(g, s, &o));
    ml::AdaptiveOptions o2 = o;
    o2.max_passes = 4;
    CHECK(ml::cache_key(g, s, &o) != ml::cache_key(g, s, &o2));
    ml::StageGeometry g2 = ps::io::parse_turbo_geometry_json(
        [&] { json p = adaptive.at("geometry"); p["compressor"]["exit_angle_deg"] = 61; return p.dump(); }(), "changed");
    CHECK(ml::cache_key(g, s, nullptr) != ml::cache_key(g2, s, nullptr));

    ml::Sampling bad = s;
    std::swap(bad.flow_kg_s[1], bad.flow_kg_s[2]);
    CHECK_THROWS_AS(ml::generate(g, bad), std::invalid_argument);
    json broken = adaptive.at("geometry");
    broken["compressor"]["hub_mm"] = 500;
    CHECK_THROWS_AS(ps::io::parse_turbo_geometry_json(broken.dump(), "broken"), std::invalid_argument);
    broken = adaptive.at("geometry");
    broken["compressor"]["blade_count"] = 3;
    CHECK_THROWS_AS(ps::io::parse_turbo_geometry_json(broken.dump(), "broken"), std::runtime_error);
}

TEST_CASE("meanline: a widened sampling range keeps every shared point and extends coverage", "[turbo_meanline][turbo_stage][drivetrain]") {
    const ml::StageGeometry g = ps::io::load_turbo_geometry_json((std::filesystem::path(PS_SOURCE_DIR) / "tools/turbo_sim/standard_geometry.json").string());
    // 30k rpm steps to 240k and 0.0375 kg/s steps to 0.6, then the same knots extended to 300k and 0.9.
    const auto axes = [&](int lines, int flows) {
        ml::Sampling s = ml::uniform_sampling(g, 1, 2, 1, 2, 40, 5);
        s.corrected_rpm.clear(); s.flow_kg_s.clear();
        for (int i = 0; i < lines; ++i) s.corrected_rpm.push_back(30000.0 * i);
        for (int j = 0; j < flows; ++j) s.flow_kg_s.push_back(0.0375 * j);
        return s;
    };
    const ml::Sampling narrow = axes(9, 17), wide = axes(11, 25);
    const ml::StageMapResult a = ml::generate(g, narrow), b = ml::generate(g, wide);
    // Shared knots: bitwise identical points (each point is solved on its own).
    int shared = 0, differ = 0;
    for (std::size_t i = 0; i < narrow.corrected_rpm.size(); ++i) {
        REQUIRE(narrow.corrected_rpm[i] == wide.corrected_rpm[i]);
        for (std::size_t j = 0; j < narrow.flow_kg_s.size(); ++j) {
            REQUIRE(narrow.flow_kg_s[j] == wide.flow_kg_s[j]);
            const auto& p = a.compressor[i * narrow.flow_kg_s.size() + j];
            const auto& q = b.compressor[i * wide.flow_kg_s.size() + j];
            ++shared;
            differ += int(p.status != q.status || p.pressure_ratio != q.pressure_ratio || p.specific_work_j_kg != q.specific_work_j_kg);
        }
    }
    CHECK(shared == 9 * 17);
    CHECK(differ == 0);
    const auto ma = ml::to_stage_maps(a, 1), mb = ml::to_stage_maps(b, 1);
    // Everything the narrow map certifies, the wide one certifies too, with the same values.
    int covered = 0;
    for (real n = 0; n <= 240000; n += 7500) {
        for (real w = 0; w <= 0.6; w += 0.0125) {
            if (!ma->compressor_covered(n, w)) continue;
            ++covered;
            CHECK(mb->compressor_covered(n, w));
            CHECK(ma->compressor_sample(n, w).pressure_ratio == mb->compressor_sample(n, w).pressure_ratio);
        }
    }
    CHECK(covered > 100);
    // New coverage: a speed above the narrow map's top line, and a flow beyond its flow axis (inlet choke is at
    // ~0.66 kg/s for this inducer, [D] 2737 mm2 x 241 kg/(m2 s)).
    CHECK_FALSE(ma->compressor_covered(270000, 0.3));
    CHECK(mb->compressor_covered(270000, 0.3));
    CHECK_FALSE(ma->compressor_covered(200000, 0.62));
    CHECK(mb->compressor_covered(200000, 0.62));
    // The choke edge itself does not move with sampling: at 105k rpm the line stops near 0.43 kg/s in both.
    CHECK_FALSE(mb->compressor_covered(105000, 0.5));
}

TEST_CASE("meanline: the shipped standard recipe refines to the Python's axes (and its generation time)", "[turbo_meanline][turbo_stage][drivetrain]") {
    // tools/turbo_sim/build_examples.py's "standard" call (9 lines + the turbine terminal line, 13 flows, 9 expansion
    // ratios, 5 passes, budgets 129/257/65).
    const ml::StageGeometry g = ps::io::load_turbo_geometry_json((std::filesystem::path(PS_SOURCE_DIR) / "tools/turbo_sim/standard_geometry.json").string());
    ml::Sampling s;
    for (int i = 0; i < 9; ++i) s.corrected_rpm.push_back(240000.0 * i / 8);
    const real jet_max = std::sqrt(2 * g.exhaust.cp() * 1000 * g.turbine.nozzle_efficiency);
    const real terminal = 2 * jet_max / (g.turbine.diameter_mm * .0005) * 60 / 6.283185307179586;
    if (terminal > s.corrected_rpm.back()) s.corrected_rpm.push_back(terminal);
    for (int i = 0; i < 13; ++i) s.flow_kg_s.push_back(.9 * i / 12);
    const ml::Sampling e = ml::uniform_sampling(g, 1, 2, 1, 2, 40, 9);
    s.expansion_ratio = e.expansion_ratio;
    ml::AdaptiveOptions o;
    o.max_passes = 5;
    o.max_speed_lines = 129;
    o.max_flow_points = 257;
    for (int workers : {0, 4}) {
        std::unique_ptr<ps::jobs::JobSystem> pool;
        if (workers > 0) pool = std::make_unique<ps::jobs::JobSystem>(workers);
        const auto t0 = std::chrono::steady_clock::now();
        const ml::StageMapResult r = ml::adaptive_generate(g, s, o, pool.get());
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("meanline standard recipe: workers=%d %.1f ms, grid %zux%zux%zu, passes %d, budget_limited %d\n", workers, ms,
                    r.axes.corrected_rpm.size(), r.axes.flow_kg_s.size(), r.axes.expansion_ratio.size(), r.sampling.passes,
                    int(r.sampling.budget_limited));
        CHECK_FALSE(r.sampling.budget_limited);
        const json all = json::parse(read_file(fixture_dir() / "reference_cases.json"));
        const json& py = all.at("standard_recipe");
        CHECK(s.corrected_rpm == py.at("initial").at("speeds").get<std::vector<real>>());
        CHECK(r.axes.corrected_rpm == py.at("corrected_rpm").get<std::vector<real>>());
        CHECK(r.axes.flow_kg_s == py.at("compressor_corrected_flow_kg_s").get<std::vector<real>>());
        CHECK(r.axes.expansion_ratio.size() == py.at("turbine_expansion_ratio").size());
        CHECK(r.sampling.passes == py.at("sampling").at("passes").get<int>());
        CHECK(r.sampling.compressor.cell_centers_checked == py.at("sampling").at("audit").at("compressor").at("cell_centers_checked").get<int>());
        CHECK(r.sampling.compressor.coverage_boundary_cells == py.at("sampling").at("audit").at("compressor").at("coverage_boundary_cells").get<int>());
    }
}

TEST_CASE("meanline: a demand-driven extension covers what the solver can compute, not the choked demand", "[turbo_meanline][turbo_stage][drivetrain]") {
    // [M] racing_game car_hyper_n2o on its standard twin map at the 1.8 bar target (D1, 2026-10-14): pressure ratio
    // 2.7765, the last four of turbo_stage_demand's 16 steps (per-turbo corrected flow, rotor speed on the continuation).
    // The first three were uncovered on the shipped map (15k rpm line spacing around 105-120k); the fourth (8250 rpm,
    // 0.5336 kg/s) needs more flow than the compressor passes at that pressure ratio (choke flow 0.5186 kg/s [M] with the
    // Python solver), so no sampling can certify it.
    constexpr real kPr = 2.7765;
    const std::vector<ml::DemandPoint> demand = {{0.4336, 111746}, {0.4669, 114410}, {0.5003, 118503}, {0.5336, 123418}};
    const ml::StageGeometry g = ps::io::load_turbo_geometry_json((std::filesystem::path(PS_SOURCE_DIR) / "tools/turbo_sim/standard_geometry.json").string());
    ml::Sampling base = ml::uniform_sampling(g, 240000, 17, 0.9, 193, 40, 5); // 15k rpm lines like the shipped map's band
    const ml::StageMapResult before = ml::generate(g, base);
    const auto mb = ml::to_stage_maps(before, 2);
    for (std::size_t i = 0; i < 3; ++i) CHECK_FALSE(mb->compressor_covered(demand[i].rotor_rpm, demand[i].flow_kg_s));

    const ml::ExtensionPolicy policy; // defaults: 500 rpm lines, 0.01 kg/s flow knots, 5 % / 10 % margins
    const ml::Sampling wide = ml::extend_sampling_for_demand(base, demand, policy);
    CHECK(wide.corrected_rpm.size() <= std::size_t(policy.max_speed_lines));
    CHECK(wide.flow_kg_s.size() <= std::size_t(policy.max_flow_points));
    for (real n : base.corrected_rpm) CHECK(std::find(wide.corrected_rpm.begin(), wide.corrected_rpm.end(), n) != wide.corrected_rpm.end());
    for (real w : base.flow_kg_s) CHECK(std::find(wide.flow_kg_s.begin(), wide.flow_kg_s.end(), w) != wide.flow_kg_s.end());
    const auto t0 = std::chrono::steady_clock::now();
    const ml::StageMapResult after = ml::generate(g, wide);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("meanline extension: %zu x %zu (from %zu x %zu) generated in %.1f ms\n", wide.corrected_rpm.size(), wide.flow_kg_s.size(),
                base.corrected_rpm.size(), base.flow_kg_s.size(), ms);
    const auto ma = ml::to_stage_maps(after, 2);
    for (std::size_t i = 0; i < 3; ++i) {
        INFO("demand " << i);
        CHECK(ma->compressor_covered(demand[i].rotor_rpm, demand[i].flow_kg_s));
    }
    CHECK_FALSE(ma->compressor_covered(demand[3].rotor_rpm, demand[3].flow_kg_s));
    // Physical: at 0.5336 kg/s every computable point already exceeds the target pressure ratio (its lowest-speed
    // valid point, the choke edge, is above it), so the target lies in the choke region the solver rejects.
    real min_valid_pr = 1e9;
    for (real n = 100000; n <= 200000; n += 250) {
        const ml::CompressorPoint p = ml::compressor_point(g, n, demand[3].flow_kg_s);
        if (p.status == ml::CompressorStatus::Ok) min_valid_pr = std::min(min_valid_pr, p.pressure_ratio);
    }
    CHECK(min_valid_pr > kPr);
    CHECK(min_valid_pr < 3.0); // [M] 2.913 at the edge, ~123.6k rpm
}

TEST_CASE("meanline: demand points from turbo_stage_demand, affinity estimate above the map", "[turbo_meanline][turbo_stage][drivetrain]") {
    ps::drivetrain::TurboStageDemand d;
    CHECK(ml::demand_points(d, 2.0 / 7.0).empty()); // not applicable: nothing
    d.applies = true;
    d.pressure_ratio = 2.5;
    d.top_speed_line_rpm = 200000;
    ps::drivetrain::TurboStageDemand::Step a;
    a.flow_kg_s = 0.3; a.rotor_rpm = 110000; a.reached = true; a.covered = false;
    ps::drivetrain::TurboStageDemand::Step b;
    b.flow_kg_s = 0.6; b.rotor_rpm = 200000; b.reached = false; b.top_line_pressure_ratio = 2.0;
    d.steps = {a, b};
    const auto pts = ml::demand_points(d, 2.0 / 7.0);
    REQUIRE(pts.size() == 2);
    CHECK(pts[0].flow_kg_s == 0.3);
    CHECK(pts[0].rotor_rpm == 110000);
    // [D] 200000 x sqrt((2.5^(2/7) - 1) / (2^(2/7) - 1)) = 200000 x sqrt(0.29938 / 0.21901) = 233.79k rpm
    CHECK(std::abs(pts[1].rotor_rpm - 233787) < 5);
}
