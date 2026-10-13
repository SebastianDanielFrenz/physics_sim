// tests/unit/test_gearbox_rating.cpp - transmission plan T0 (out/transmission_pedal_plan.md sections 6.3, 7.1, 7.3, 8):
// the gearbox data foundation. Rating (rated_input_torque_nm -> BuildRule::GearboxRating, LimitKind::GearboxInputTorque),
// gear_type / tooth counts / mesh_order -> DrivelineSoundState, gearbox "extends", classify_gearbox_change.
//
// BANDS (written before the first run; never widened; each guard has a recorded sabotage in the case comment):
//   G1  loader: every new field round-trips EXACTLY (== on doubles/ints); an extends chain inherits arrays and overrides a
//       scalar; defaults of a gearbox without the new keys are rating 0, helical, no teeth, pair {0,0}, mesh_order 0.
//   G2  loader rejections: each bad value throws, the message names the key (substrings below).
//   G3  grading with a 1000 N*m rating: peak 1000 -> none; 1049.9 Near; 1050.1 Limiting; 1149.9 Limiting; 1150.1 Exceeded;
//       no rating -> none; only Exceeded blocks (has_blocking_issue).
//   G4  hyper (Sketch maps, dyno peak ~1378 N*m): rated 900 -> Exceeded in BOTH check_gearbox_rating and
//       check_driveline_limits (GearboxInputTorque, rpm = the peak point's rpm, required_min == peak, required_margin ==
//       1.1 * peak to 1e-12); rated 1500 -> no finding; rated 1250 (extends chain) -> Limiting (10 %); the finding sorts after
//       a ClutchCapacity finding.
//   G5  torque map (i4): peak_net_torque_nm == 200.0 (curve knot max; the dyno's sampled peak 199.8 is within 0.2 % and never above):
//       rating 199 -> Near, 185 -> Limiting, 170 -> Exceeded, 200 -> none.
//   G6  classify_gearbox_change: the table in the case (None / Runtime / Respawn), symmetric, worst field wins.
//   G7  sound fields (World, car_sedan with a straight-cut gearbox with tooth counts): mesh_frequency_hz[0] ==
//       |w_in| * z_a / (2 pi) and [1] == |w_in| * (z_a / z_b) * z_drive(gear) / (2 pi), relative 1e-12 on EVERY tick of a
//       launch in 1st and 2nd; fallback |w_in| * mesh_order / (2 pi) with no teeth; [1] and the load 0 in neutral;
//       gear_type 1 straight-cut / 0 helical.
//   G8  mesh_load_nm vs the clutch torque of a locked-clutch pull: |mesh_load - |T_clutch|| <= 3 % * |T_clutch| +
//       2 * J_in * |alpha| + 1 N*m (J_in = clutch out + gearbox input inertia, alpha from the tick-to-tick input speed) on every
//       locked tick, for the lossy sedan box AND the same box with all efficiencies 1.0; peak load > 100 N*m.
//   G9  hash neutrality: the same launch with and without rating / gear_type / teeth / mesh_order gives an identical
//       World::state_hash() at 1 and 10 workers.
//   G10 zero own allocation per tick with the sound data present (debug_alloc armed, 200 ticks of a launch).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "drivetrain/engine_rules_table.h"
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/engine_dyno.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;

namespace {

const std::string kRoot = PS_SOURCE_DIR;
const std::string kVehicles = kRoot + "/data/vehicles/";
const std::string kEngines = kRoot + "/data/engines/";
const std::string kFixtures = kRoot + "/tests/support/fixtures/trans_t0/";
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

std::string cache_dir() {
    const std::filesystem::path p = std::filesystem::temp_directory_path() / "ps_garage_p3b1_maps"; // shared with test_limit_findings
    std::filesystem::create_directories(p);
    return p.string();
}
jobs::JobSystem& map_pool() {
    static jobs::JobSystem pool(10);
    return pool;
}
io::EngineMapOptions sketch_options() {
    io::EngineMapOptions o;
    o.quality = MapQuality::Sketch;
    o.pool = &map_pool();
    o.cache_dir = cache_dir();
    return o;
}

// A per-process directory for generated gearbox files (parallel ctest processes must not share files).
const std::filesystem::path& temp_dir() {
    static const std::filesystem::path p = [] {
        std::random_device rd;
        const std::filesystem::path d = std::filesystem::temp_directory_path() / ("ps_trans_t0_" + std::to_string(rd()) + std::to_string(rd()));
        std::filesystem::create_directories(d);
        return d;
    }();
    return p;
}
std::string write_gearbox(const std::string& name, const std::string& body) {
    const std::filesystem::path f = temp_dir() / (name + ".json");
    std::ofstream o(f);
    o << body;
    return f.string();
}
// A gearbox part file that extends sedan_6mt with `patch` (the inside of a JSON object, without braces).
std::string write_patch(const std::string& name, const std::string& patch) {
    std::string base = kRoot + "/data/gearboxes/sedan_6mt.json";
    return write_gearbox(name, "{ \"extends\": \"" + base + "\", " + patch + " }");
}

// car_sedan.json with its gearbox component replaced by `gearbox_path` (the user layer's part override).
vehicle::VehicleDesc sedan_with(const std::string& gearbox_path) {
    io::VehicleLoadOptions o;
    if (!gearbox_path.empty()) o.part_overrides["gearbox"].ref = gearbox_path;
    return io::load_vehicle_json(kVehicles + "car_sedan.json", o);
}
GearboxDesc& gearbox_in(vehicle::VehicleDesc& v) {
    for (ComponentDesc& c : v.powertrain.components)
        if (auto* g = std::get_if<GearboxDesc>(&c.params)) return *g;
    FAIL("no gearbox component");
    throw 0;
}
const GearboxDesc& gearbox_in(const vehicle::VehicleDesc& v) {
    for (const ComponentDesc& c : v.powertrain.components)
        if (const auto* g = std::get_if<GearboxDesc>(&c.params)) return *g;
    FAIL("no gearbox component");
    throw 0;
}
const ClutchDesc& clutch_in(const vehicle::VehicleDesc& v) {
    for (const ComponentDesc& c : v.powertrain.components)
        if (const auto* g = std::get_if<ClutchDesc>(&c.params)) return *g;
    FAIL("no clutch component");
    throw 0;
}

const ComponentParams& hyper() {
    static const ComponentParams c = io::load_engine_json(kEngines + "hyper_v8_one1_audio.json", sketch_options());
    return c;
}
const ComponentParams& i4_torque_map() {
    static const ComponentParams c = io::load_engine_json(kEngines + "i4_2l_na.json");
    return c;
}
DynoSweepSpec hyper_spec() {
    DynoSweepSpec s;
    s.rpm = {2500.0, 5000.0, 6000.0, 7000.0}; // the first point (low boost) must NOT be the peak: it guards "peak point" vs "first point"
    return s;
}
const DynoResult& hyper_dyno() {
    static jobs::JobSystem pool(10);
    static const DynoResult r = run_dyno_sweep(hyper(), nullptr, hyper_spec(), &pool);
    return r;
}
const DynoPoint& peak_point(const DynoResult& r) {
    const DynoPoint* best = &r.points.front();
    for (const DynoPoint& p : r.points)
        if (p.torque_nm > best->torque_nm) best = &p;
    return *best;
}

// A gearbox with a rating only.
GearboxDesc rated(double nm) {
    GearboxDesc g;
    g.forward_ratios = {3.0, 2.0, 1.0};
    g.forward_efficiency = {1.0, 1.0, 1.0};
    g.rated_input_torque_nm = nm;
    return g;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// G1/G2: loader

TEST_CASE("G1 gearbox loader: rating, gear_type, teeth, mesh_order round-trip; extends chain; defaults", "[drivetrain][gearbox][io]") {
    // Recorded sabotage: reading `teeth.forward` into reverse_teeth (and vice versa) fails the tooth-count checks; dropping the
    // gearbox branch of resolve_component_ref's load_part (plain read_json_file) fails with "unknown key extends"-less merge:
    // the fixture then has no forward_ratios and the load throws.
    const vehicle::VehicleDesc plain = sedan_with("");
    const GearboxDesc& d0 = gearbox_in(plain);
    CHECK(d0.rated_input_torque_nm == 0.0);
    CHECK(d0.gear_type == GearType::Helical);
    CHECK(d0.forward_teeth.empty());
    CHECK(d0.reverse_teeth.empty());
    CHECK(d0.input_pair_teeth.drive == 0);
    CHECK(d0.input_pair_teeth.driven == 0);
    CHECK(d0.mesh_order == 0.0);

    const vehicle::VehicleDesc v900 = sedan_with(kFixtures + "gearbox_rated_900.json");
    const GearboxDesc& g = gearbox_in(v900);
    CHECK(g.type == GearboxType::Manual);
    CHECK(g.rated_input_torque_nm == 900.0);
    CHECK(g.gear_type == GearType::StraightCut);
    REQUIRE(g.forward_ratios.size() == 7); // inherited from hyper_7mt
    CHECK(g.forward_ratios[0] == 3.1);
    CHECK(g.input_inertia_kgm2 == 0.01);
    REQUIRE(g.forward_teeth.size() == 7);
    CHECK(g.forward_teeth[0].drive == 13);
    CHECK(g.forward_teeth[0].driven == 29);
    CHECK(g.forward_teeth[6].drive == 31);
    CHECK(g.forward_teeth[6].driven == 24);
    REQUIRE(g.reverse_teeth.size() == 1);
    CHECK(g.reverse_teeth[0].drive == 12);
    CHECK(g.reverse_teeth[0].driven == 31);
    CHECK(g.input_pair_teeth.drive == 23);
    CHECK(g.input_pair_teeth.driven == 31);

    // Two-level chain: hyper_7mt <- rated_900 <- chain. The scalar override wins, the arrays are inherited.
    const vehicle::VehicleDesc vchain = sedan_with(kFixtures + "gearbox_chain.json");
    const GearboxDesc& c = gearbox_in(vchain);
    CHECK(c.rated_input_torque_nm == 1250.0);
    CHECK(c.gear_type == GearType::Helical);
    CHECK(c.forward_teeth.size() == 7);
    CHECK(c.input_pair_teeth.drive == 23);
    CHECK(c.forward_ratios.size() == 7);

    // mesh_order alone, and a gearbox with only a rating.
    const GearboxDesc& o = gearbox_in(sedan_with(write_patch("g1_order", "\"mesh_order\": 3.5, \"rated_input_torque_nm\": 410")));
    CHECK(o.mesh_order == 3.5);
    CHECK(o.rated_input_torque_nm == 410.0);
    CHECK(o.forward_teeth.empty());
}

TEST_CASE("G2 gearbox loader rejections name the key", "[drivetrain][gearbox][io]") {
    // Recorded sabotage: removing the `> 0` test of rated_input_torque_nm fails the first three rows; accepting a fractional
    // tooth count fails the "integers" row.
    struct Row {
        const char* name;
        const char* patch;
        const char* message;
    };
    const Row rows[] = {
        {"rate0", "\"rated_input_torque_nm\": 0", "rated_input_torque_nm must be a number > 0"},
        {"rateneg", "\"rated_input_torque_nm\": -5", "rated_input_torque_nm must be a number > 0"},
        {"ratestr", "\"rated_input_torque_nm\": \"big\"", "rated_input_torque_nm must be a number > 0"},
        {"gtype", "\"gear_type\": \"spur\"", "gear_type must be"},
        {"gtypenum", "\"gear_type\": 1", "gear_type must be"},
        {"order0", "\"mesh_order\": 0", "mesh_order must be a number > 0"},
        {"teethobj", "\"teeth\": 3", "teeth must be an object"},
        {"teethkey", "\"teeth\": { \"fwd\": [] }", "teeth: unknown key 'fwd'"},
        {"teethlen", "\"teeth\": { \"forward\": [ {\"drive\": 14, \"driven\": 38} ] }", "teeth.forward must have the same length as forward_ratios"},
        {"teethrev", "\"teeth\": { \"reverse\": [] }", "teeth.reverse must have the same length as reverse_ratios"},
        {"teethzero", "\"teeth\": { \"input_pair\": {\"drive\": 0, \"driven\": 30} }", "tooth counts must be integers >= 1"},
        {"teethfrac", "\"teeth\": { \"input_pair\": {\"drive\": 14.5, \"driven\": 30} }", "tooth counts must be integers >= 1"},
        {"teethmiss", "\"teeth\": { \"input_pair\": {\"drive\": 14} }", "a tooth pair needs"},
        {"teethpair", "\"teeth\": { \"input_pair\": {\"drive\": 14, \"driven\": 30, \"x\": 1} }", "unknown key 'x'"},
    };
    for (const Row& r : rows) {
        INFO(r.name);
        const std::string path = write_patch(std::string("g2_") + r.name, r.patch);
        CHECK_THROWS_WITH(sedan_with(path), ContainsSubstring(r.message));
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// G3: grading

TEST_CASE("G3 check_gearbox_rating grades over the rating: Near < 5 %, Limiting < 15 %, Exceeded beyond", "[drivetrain][gearbox]") {
    // Recorded sabotage: swapping kGearboxOverLimiting and kGearboxOverExceeded in the grading (Exceeded at 5 %) fails the
    // 1049.9 / 1050.1 rows; grading UNDER the rating (the clutch's 1/1.3 rule) reports 1000 itself.
    const GearboxDesc g = rated(1000.0);
    CHECK(check_gearbox_rating(g, 1000.0).empty());
    CHECK(check_gearbox_rating(g, 500.0).empty());
    CHECK(check_gearbox_rating(rated(0.0), 5000.0).empty());
    struct Row {
        double peak;
        LimitSeverity sev;
    };
    const Row rows[] = {{1000.0001, LimitSeverity::Near},   {1049.9, LimitSeverity::Near},      {1050.1, LimitSeverity::Limiting},
                        {1149.9, LimitSeverity::Limiting}, {1150.1, LimitSeverity::Exceeded}, {2000.0, LimitSeverity::Exceeded}};
    for (const Row& r : rows) {
        INFO("peak " << r.peak);
        const auto issues = check_gearbox_rating(g, r.peak);
        REQUIRE(issues.size() == 1);
        const BuildIssue& b = issues[0];
        CHECK(b.rule == BuildRule::GearboxRating);
        CHECK(b.severity == r.sev);
        CHECK(b.slot == "gearbox");
        CHECK(b.field_pointer == "/rated_input_torque_nm");
        CHECK(b.value == r.peak);
        CHECK(b.limit == 1000.0);
        CHECK(b.unit == LimitUnit::NewtonMetre);
        CHECK(b.source_label.rfind("[AS]", 0) == 0);
        CHECK_FALSE(b.message.empty());
        CHECK(has_blocking_issue(issues) == (r.sev == LimitSeverity::Exceeded));
    }
    // The message carries both numbers.
    const auto issues = check_gearbox_rating(g, 1300.0);
    REQUIRE(issues.size() == 1);
    CHECK_THAT(issues[0].message, ContainsSubstring("1300"));
    CHECK_THAT(issues[0].message, ContainsSubstring("1000"));
    // The powertrain overload checks each gearbox component.
    vehicle::VehicleDesc v = sedan_with("");
    gearbox_in(v).rated_input_torque_nm = 200.0;
    const auto pi = check_gearbox_rating(v.powertrain, 240.0); // 20 % over
    REQUIRE(pi.size() == 1);
    CHECK(pi[0].severity == LimitSeverity::Exceeded);
    CHECK(check_gearbox_rating(v.powertrain, 190.0).empty());

    CHECK(static_cast<int>(BuildRule::GearboxRating) == 16);
    CHECK(std::string(to_string(BuildRule::GearboxRating)) == "GearboxRating");
    CHECK(rules::kGearboxOverLimiting.value == 0.05);
    CHECK(rules::kGearboxOverExceeded.value == 0.15);
    CHECK(rules::kGearboxRemedyMargin.value == 1.10);
    bool listed = false;
    for (std::size_t i = 0; i < rules::kAllEngineRuleThresholdCount; ++i)
        if (std::string(rules::kAllEngineRuleThresholds[i].name) == "gearbox_rating_remedy_margin") listed = true;
    CHECK(listed);
}

// ---------------------------------------------------------------------------------------------------------------------
// G4/G5: against real engines

TEST_CASE("G4 hyper: a 900 N*m box is Exceeded (rule and finding), 1500 is clear, 1250 is Limiting", "[drivetrain][gearbox][dyno][io]") {
    // The peak is the BOOSTED dyno peak (Sketch maps ~1378 N*m at 6000 rpm; 1371 calibrated). Recorded sabotages:
    //  - check_driveline_limits using dyno.points.front() (5000 rpm) instead of the peak point fails the rpm / value rows;
    //  - rating grade computed against the engine's NA torque (the hyper without its turbo makes ~900-1000) reports no finding for
    //    the 1250 box: the Limiting row fails;
    //  - dropping `/ margin` -> required_margin = peak fails the 1.1 row.
    const DynoResult& dyno = hyper_dyno();
    const DynoPoint& pk = peak_point(dyno);
    REQUIRE(pk.torque_nm > 1300.0);
    REQUIRE(pk.rpm != dyno.points.front().rpm);

    const auto load = [&](const char* file) {
        io::VehicleLoadOptions o;
        o.maps = sketch_options();
        o.part_overrides["gearbox"].ref = kFixtures + file;
        return io::load_vehicle_json(kVehicles + "car_hyper.json", o);
    };
    {
        const vehicle::VehicleDesc v = load("gearbox_rated_900.json");
        const auto issues = check_gearbox_rating(v.powertrain, pk.torque_nm);
        REQUIRE(issues.size() == 1);
        CHECK(issues[0].severity == LimitSeverity::Exceeded);
        CHECK(has_blocking_issue(issues));
        CHECK(issues[0].value == pk.torque_nm);
        CHECK(issues[0].limit == 900.0);

        const auto f = check_driveline_limits(dyno, v.powertrain, "clutch.json", "gearbox.json");
        REQUIRE(f.size() == 1); // the hyper's 1650 N*m clutch holds the peak with the 1.3 margin (test_limit_findings L19)
        CHECK(f[0].kind == LimitKind::GearboxInputTorque);
        CHECK(f[0].severity == LimitSeverity::Exceeded);
        CHECK(f[0].rpm == pk.rpm);
        CHECK(f[0].value == pk.torque_nm);
        CHECK(f[0].limit == 900.0);
        CHECK(f[0].unit == LimitUnit::NewtonMetre);
        CHECK(f[0].part.slot == "gearbox");
        CHECK(f[0].part.file == "gearbox.json");
        CHECK(f[0].part.field == "/rated_input_torque_nm");
        REQUIRE(f[0].remedy.has_value());
        const Remedy& r = *f[0].remedy;
        CHECK(r.cost == RemedyCost::Closed);
        CHECK(r.solved);
        CHECK(r.unit == LimitUnit::NewtonMetre);
        CHECK(r.current == 900.0);
        CHECK(r.required_min == pk.torque_nm);
        CHECK(r.required_margin == Catch::Approx(1.1 * pk.torque_nm).epsilon(1e-12));
        CHECK(r.part.field == "/rated_input_torque_nm");
        // The peak is also over the 900 box in the 1.15 band: well beyond (53 %).
        CHECK(pk.torque_nm / 900.0 > 1.4);
    }
    {
        const vehicle::VehicleDesc v = load("gearbox_rated_1500.json");
        CHECK(check_gearbox_rating(v.powertrain, pk.torque_nm).empty());
        CHECK(check_driveline_limits(dyno, v.powertrain, "clutch.json", "gearbox.json").empty());
    }
    {
        const vehicle::VehicleDesc v = load("gearbox_chain.json"); // 1250
        const auto issues = check_gearbox_rating(v.powertrain, pk.torque_nm);
        REQUIRE(issues.size() == 1);
        CHECK(issues[0].severity == LimitSeverity::Limiting);
        CHECK_FALSE(has_blocking_issue(issues));
        const auto f = check_driveline_limits(dyno, v.powertrain);
        REQUIRE(f.size() == 1);
        CHECK(f[0].kind == LimitKind::GearboxInputTorque);
        CHECK(f[0].severity == LimitSeverity::Limiting);
        CHECK(f[0].part.file.empty());
    }
    {
        // Both a too-small clutch and a too-small gearbox: two findings in LimitKind order.
        vehicle::VehicleDesc v = load("gearbox_rated_900.json");
        for (ComponentDesc& c : v.powertrain.components)
            if (auto* cl = std::get_if<ClutchDesc>(&c.params)) cl->capacity_nm = 1000.0;
        const auto f = check_driveline_limits(dyno, v.powertrain, "c.json", "g.json");
        REQUIRE(f.size() == 2);
        CHECK(f[0].kind == LimitKind::ClutchCapacity);
        CHECK(f[1].kind == LimitKind::GearboxInputTorque);
    }
}

TEST_CASE("G5 torque-map engine: peak_net_torque_nm and the rating grades", "[drivetrain][gearbox][dyno][io]") {
    // i4_2l_na: WOT peak 199.8 N*m (test_limit_findings L19). Rating 185 -> 8 % over, Limiting; 170 -> 17.5 %, Exceeded; 200 -> clear.
    // Recorded sabotage: taking the curve's LAST point (4550+ rpm, lower than the peak) instead of the maximum reports 185 as Near.
    const auto& tm = std::get<TorqueMapEngineDesc>(i4_torque_map());
    const double peak = peak_net_torque_nm(tm);
    CHECK(peak == 200.0); // the curve's knot maximum (4500 rpm); the dyno's sampled net peak is 199.8
    CHECK(static_cast<int>(check_gearbox_rating(rated(199.0), peak)[0].severity) == static_cast<int>(LimitSeverity::Near));
    const auto sev = [&](double rating) {
        const auto issues = check_gearbox_rating(rated(rating), peak);
        return issues.empty() ? -1 : static_cast<int>(issues[0].severity);
    };
    CHECK(sev(185.0) == static_cast<int>(LimitSeverity::Limiting));
    CHECK(sev(170.0) == static_cast<int>(LimitSeverity::Exceeded));
    CHECK(sev(200.0) == -1);
    // The dyno of the torque map agrees with the closed form within its rpm sampling (199.8 vs 200, never above).
    const DynoResult dyno = run_dyno_sweep(i4_torque_map(), nullptr, DynoSweepSpec{}, nullptr);
    CHECK(peak_point(dyno).torque_nm <= peak);
    CHECK(peak_point(dyno).torque_nm == Catch::Approx(peak).epsilon(0.002));
}

// ---------------------------------------------------------------------------------------------------------------------
// G6: classify_gearbox_change

TEST_CASE("G6 classify_gearbox_change: sound data None, controller/strength data Runtime, geometry Respawn", "[drivetrain][gearbox]") {
    // Recorded sabotage: classifying a ratio edit as Runtime (moving forward_ratios into the Runtime group) fails the Respawn
    // rows; dropping rated_input_torque_nm from the Runtime group makes the rating row read None.
    const vehicle::VehicleDesc v = sedan_with("");
    const GearboxDesc base = gearbox_in(v);
    CHECK(classify_gearbox_change(base, base) == GearboxChange::None);
    CHECK(static_cast<int>(GearboxChange::None) == 0);
    CHECK(static_cast<int>(GearboxChange::Runtime) == 1);
    CHECK(static_cast<int>(GearboxChange::Respawn) == 2);
    CHECK(std::string(to_string(GearboxChange::Respawn)) == "Respawn");

    struct Row {
        const char* name;
        GearboxChange expected;
        void (*mutate)(GearboxDesc&);
    };
    const Row rows[] = {
        {"gear_type", GearboxChange::None, [](GearboxDesc& g) { g.gear_type = GearType::StraightCut; }},
        {"forward_teeth", GearboxChange::None, [](GearboxDesc& g) { g.forward_teeth.assign(g.forward_ratios.size(), GearTeeth{14, 30}); }},
        {"reverse_teeth", GearboxChange::None, [](GearboxDesc& g) { g.reverse_teeth.assign(g.reverse_ratios.size(), GearTeeth{14, 30}); }},
        {"input_pair_teeth", GearboxChange::None, [](GearboxDesc& g) { g.input_pair_teeth = GearTeeth{19, 41}; }},
        {"mesh_order", GearboxChange::None, [](GearboxDesc& g) { g.mesh_order = 4.0; }},
        {"rated_input_torque_nm", GearboxChange::Runtime, [](GearboxDesc& g) { g.rated_input_torque_nm = 250.0; }},
        {"forward_efficiency", GearboxChange::Runtime, [](GearboxDesc& g) { g.forward_efficiency[2] = 0.9; }},
        {"reverse_efficiency", GearboxChange::Runtime, [](GearboxDesc& g) { g.reverse_efficiency[0] = 0.9; }},
        {"synchro.forward_capacity_nm", GearboxChange::Runtime, [](GearboxDesc& g) { g.synchro.forward_capacity_nm[0] += 1.0; }},
        {"synchro.reverse_capacity_nm", GearboxChange::Runtime, [](GearboxDesc& g) { g.synchro.reverse_capacity_nm[0] += 1.0; }},
        {"synchro.static_factor", GearboxChange::Runtime, [](GearboxDesc& g) { g.synchro.static_factor += 0.1; }},
        {"synchro.max_dog_mismatch_rad_s", GearboxChange::Runtime, [](GearboxDesc& g) { g.synchro.max_dog_mismatch_rad_s += 1.0; }},
        {"neutral_phase_s", GearboxChange::Runtime, [](GearboxDesc& g) { g.neutral_phase_s += 0.01; }},
        {"dog_release_torque_nm", GearboxChange::Runtime, [](GearboxDesc& g) { g.dog_release_torque_nm += 1.0; }},
        {"grind_after_s", GearboxChange::Runtime, [](GearboxDesc& g) { g.grind_after_s += 0.01; }},
        {"abort_after_s", GearboxChange::Runtime, [](GearboxDesc& g) { g.abort_after_s += 0.01; }},
        {"initial_gear", GearboxChange::Runtime, [](GearboxDesc& g) { g.initial_gear = 1; }},
        {"type", GearboxChange::Respawn, [](GearboxDesc& g) { g.type = GearboxType::Sequential; }},
        {"forward_ratios value", GearboxChange::Respawn, [](GearboxDesc& g) { g.forward_ratios[1] -= 0.01; }},
        {"reverse_ratios value", GearboxChange::Respawn, [](GearboxDesc& g) { g.reverse_ratios[0] -= 0.01; }},
        {"forward gear count", GearboxChange::Respawn,
         [](GearboxDesc& g) {
             g.forward_ratios.pop_back();
             g.forward_efficiency.pop_back();
             g.synchro.forward_capacity_nm.pop_back();
         }},
        {"input_inertia_kgm2", GearboxChange::Respawn, [](GearboxDesc& g) { g.input_inertia_kgm2 += 0.001; }},
        {"output_inertia_kgm2", GearboxChange::Respawn, [](GearboxDesc& g) { g.output_inertia_kgm2 += 0.001; }},
        {"input_axis_local", GearboxChange::Respawn, [](GearboxDesc& g) { g.input_axis_local = Vec3{0.0, -1.0, 0.0}; }},
        {"output_axis_local", GearboxChange::Respawn, [](GearboxDesc& g) { g.output_axis_local = Vec3{0.0, 1.0, 0.0}; }},
    };
    for (const Row& r : rows) {
        INFO(r.name);
        GearboxDesc m = base;
        r.mutate(m);
        CHECK(classify_gearbox_change(base, m) == r.expected);
        CHECK(classify_gearbox_change(m, base) == r.expected); // symmetric
    }
    // The most expensive changed field wins.
    GearboxDesc m = base;
    m.gear_type = GearType::StraightCut;
    m.rated_input_torque_nm = 300.0;
    CHECK(classify_gearbox_change(base, m) == GearboxChange::Runtime);
    m.forward_ratios[0] += 0.1;
    CHECK(classify_gearbox_change(base, m) == GearboxChange::Respawn);
}

// ---------------------------------------------------------------------------------------------------------------------
// G7-G10: sound fields in a running vehicle

namespace {

constexpr double kMass = 1500.0;
const Vec3 kHalf{2.0, 0.4, 0.15};
constexpr double kOriginHeight = 0.6;

ps_test::VehicleTestRig sedan_rig(const vehicle::VehicleDesc& desc, int workers = 4) {
    return ps_test::build_vehicle_rig_from_desc(desc, kMass, kHalf, kOriginHeight, 0.0, workers);
}
void expect_close(double got, double want) { CHECK(got == Catch::Approx(want).epsilon(1e-12)); }

} // namespace

TEST_CASE("G7 mesh frequencies follow the tooth counts of the engaged gear (1st and 2nd), neutral and fallback", "[drivetrain][gearbox][vehicle]") {
    // Recorded sabotages: teeth indexed by `pos` instead of `pos - 1` (gear 1 reads the 2nd gear's teeth) fails the 1st-gear
    // rows; drive/driven swapped fails every row; the layshaft speed factor z_a/z_b dropped fails [1].
    const vehicle::VehicleDesc desc = sedan_with(kFixtures + "gearbox_sedan_teeth.json");
    auto rig = sedan_rig(desc);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    {
        // Neutral: [1] and the load are 0, [0] is the pair at the input speed, the type is straight-cut.
        w.step();
        const auto& dl = w.powertrain_state(rig.vehicle).driveline;
        CHECK(dl.gear == 0);
        CHECK(dl.gear_type == 1);
        CHECK(dl.mesh_frequency_hz[1] == 0.0);
        CHECK(dl.mesh_load_nm == 0.0);
        expect_close(dl.mesh_frequency_hz[0], std::abs(dl.gearbox_in_omega) * 19.0 / kTwoPi);
    }
    const double zdrive[] = {0.0, 14.0, 18.0};
    int checked[3] = {0, 0, 0};
    w.set_control("throttle", 1.0);
    w.set_control("gear_request", 1.0);
    for (int i = 0; i < 960; ++i) { // 4 s: 1st, then 2nd from t = 2 s
        if (i == 480) w.set_control("gear_request", 2.0);
        w.step();
        const auto& dl = w.powertrain_state(rig.vehicle).driveline;
        if (dl.gear != 1 && dl.gear != 2) continue;
        if (std::abs(dl.gearbox_in_omega) < 20.0) continue;
        ++checked[dl.gear];
        CHECK(dl.gear_type == 1);
        expect_close(dl.mesh_frequency_hz[0], std::abs(dl.gearbox_in_omega) * 19.0 / kTwoPi);
        expect_close(dl.mesh_frequency_hz[1], std::abs(dl.gearbox_in_omega) * (19.0 / 41.0) * zdrive[dl.gear] / kTwoPi);
        CHECK(dl.mesh_load_nm >= 0.0);
    }
    CHECK(checked[1] > 100);
    CHECK(checked[2] > 100);
}

TEST_CASE("G7b fallback: mesh_order without teeth, a pair without gear teeth, nothing known", "[drivetrain][gearbox][vehicle]") {
    // Recorded sabotage: the fallback applied to [0] only fails the order rows' [1] check.
    const auto run = [&](const auto& edit, auto&& check) {
        vehicle::VehicleDesc desc = sedan_with("");
        edit(gearbox_in(desc));
        auto rig = sedan_rig(desc);
        World& w = *rig.world;
        ps_test::settle(w, 1.0);
        w.set_control("throttle", 1.0);
        w.set_control("gear_request", 1.0);
        int n = 0;
        for (int i = 0; i < 480; ++i) {
            w.step();
            const auto& dl = w.powertrain_state(rig.vehicle).driveline;
            if (dl.gear != 1 || std::abs(dl.gearbox_in_omega) < 20.0) continue;
            ++n;
            check(dl);
        }
        CHECK(n > 100);
    };
    // mesh_order 4, no teeth: both entries are |w_in| * 4 / 2 pi.
    run([](GearboxDesc& g) { g.mesh_order = 4.0; }, [&](const DrivelineSoundState& dl) {
        CHECK(dl.gear_type == 0);
        expect_close(dl.mesh_frequency_hz[0], std::abs(dl.gearbox_in_omega) * 4.0 / kTwoPi);
        expect_close(dl.mesh_frequency_hz[1], std::abs(dl.gearbox_in_omega) * 4.0 / kTwoPi);
    });
    // A pair, no gear teeth, mesh_order 5: [0] from the pair, [1] from the fallback order.
    run([](GearboxDesc& g) { g.input_pair_teeth = GearTeeth{20, 40}; g.mesh_order = 5.0; }, [&](const DrivelineSoundState& dl) {
        expect_close(dl.mesh_frequency_hz[0], std::abs(dl.gearbox_in_omega) * 20.0 / kTwoPi);
        expect_close(dl.mesh_frequency_hz[1], std::abs(dl.gearbox_in_omega) * 5.0 / kTwoPi);
    });
    // Nothing known: both 0.
    run([](GearboxDesc&) {}, [&](const DrivelineSoundState& dl) {
        CHECK(dl.mesh_frequency_hz[0] == 0.0);
        CHECK(dl.mesh_frequency_hz[1] == 0.0);
    });
    // Gear teeth without an input pair: the layshaft is the input shaft.
    run([](GearboxDesc& g) { g.forward_teeth.assign(g.forward_ratios.size(), GearTeeth{15, 30}); }, [&](const DrivelineSoundState& dl) {
        CHECK(dl.mesh_frequency_hz[0] == 0.0);
        expect_close(dl.mesh_frequency_hz[1], std::abs(dl.gearbox_in_omega) * 15.0 / kTwoPi);
    });
}

TEST_CASE("G8 mesh_load_nm is the torque through the engaged gear (clutch torque less the input inertia), lossy and lossless", "[drivetrain][gearbox][vehicle]") {
    // Recorded sabotages: publishing the lagged hashed variable (gearbox_lambda_mesh_prev_) leaves the lossless box at 0 and
    // fails its rows; publishing the torque at the OUTPUT (times the ratio) fails the 3 % band by a factor of 3.5.
    for (const bool lossless : {false, true}) {
        INFO((lossless ? "lossless box" : "lossy box"));
        vehicle::VehicleDesc desc = sedan_with("");
        if (lossless) {
            GearboxDesc& g = gearbox_in(desc);
            std::fill(g.forward_efficiency.begin(), g.forward_efficiency.end(), 1.0);
            std::fill(g.reverse_efficiency.begin(), g.reverse_efficiency.end(), 1.0);
        }
        const double j_in = gearbox_in(desc).input_inertia_kgm2 + clutch_in(desc).out_inertia_kgm2;
        auto rig = sedan_rig(desc);
        World& w = *rig.world;
        ps_test::settle(w, 1.0);
        w.set_control("throttle", 1.0);
        w.set_control("gear_request", 1.0);
        struct Tick {
            double w_in, load, clutch, engagement;
            bool stuck;
        };
        std::vector<Tick> t;
        for (int i = 0; i < 720; ++i) {
            w.step();
            const auto& ps = w.powertrain_state(rig.vehicle);
            const auto dbg = w.powertrain_debug(rig.vehicle);
            t.push_back({ps.driveline.gearbox_in_omega, ps.driveline.mesh_load_nm, dbg.clutch_torque, ps.clutch_engagement, dbg.clutch_stuck});
        }
        const double dt = w.dt();
        int locked = 0;
        double peak = 0.0;
        for (std::size_t i = 1; i + 1 < t.size(); ++i) {
            if (!(t[i].engagement > 0.99) || !t[i].stuck) continue;
            ++locked;
            const double alpha = (t[i + 1].w_in - t[i - 1].w_in) / (2.0 * dt);
            const double tc = std::abs(t[i].clutch);
            const double band = 0.03 * tc + 2.0 * j_in * std::abs(alpha) + 1.0;
            INFO("tick " << i << " load " << t[i].load << " clutch " << tc << " band " << band);
            CHECK(std::abs(t[i].load - tc) <= band);
            peak = std::max(peak, t[i].load);
        }
        CHECK(locked > 100);
        CHECK(peak > 100.0);
    }
}

TEST_CASE("G9 the sound/rating data is hash-neutral (1 and 10 workers)", "[drivetrain][gearbox][vehicle][determinism]") {
    // Recorded sabotage: folding gearbox_mesh_load_nm_ into Powertrain::fold_state_hash moves the hash of the second run.
    const auto run = [&](const vehicle::VehicleDesc& desc, int workers) {
        auto rig = sedan_rig(desc, workers);
        World& w = *rig.world;
        ps_test::settle(w, 0.5);
        w.set_control("throttle", 1.0);
        w.set_control("gear_request", 1.0);
        for (int i = 0; i < 600; ++i) {
            if (i == 360) w.set_control("gear_request", 2.0);
            w.step();
        }
        return w.state_hash();
    };
    const vehicle::VehicleDesc plain = sedan_with("");
    vehicle::VehicleDesc dressed = sedan_with(kFixtures + "gearbox_sedan_teeth.json"); // rating, straight-cut, teeth
    // The fixture's ratios and efficiencies are the plain box's (extends sedan_6mt): only the new data differs.
    gearbox_in(dressed).mesh_order = 3.0;
    const std::uint64_t h1 = run(plain, 1);
    CHECK(h1 == run(plain, 10));
    CHECK(h1 == run(dressed, 1));
    CHECK(h1 == run(dressed, 10));
}

TEST_CASE("G10 stepping with the sound data allocates nothing in our own code", "[drivetrain][gearbox][vehicle][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    const vehicle::VehicleDesc desc = sedan_with(kFixtures + "gearbox_sedan_teeth.json");
    auto rig = sedan_rig(desc, 4);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);
    w.set_control("throttle", 0.8);
    w.set_control("gear_request", 1.0);
    for (int i = 0; i < 100; ++i) w.step(); // warm-up
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 200; ++i) w.step();
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    ps::jobs::debug_alloc::reset();
    CHECK(own == 0);
    CHECK(w.powertrain_state(rig.vehicle).driveline.mesh_frequency_hz[1] > 0.0);
}
