// PHYS-033 P4 I7c C16b (D8): the .psmaps on-disk cache in
// core/src/engine_cycle/map_cache.h/.cpp. Covers the C16b test table's
// H1-H4 (out/i7c_plan.md):
//
// H1 a cache hit reproduces a fresh generation bit-for-bit, both at the
//    map_cache.h level directly and through load_vehicle_json's
//    generation_count_out wiring (vehicle_io.cpp: miss -> generate -> store,
//    hit -> generation_count_out untouched).
// H2 a corrupt or mismatched .psmaps file is silently discarded (never
//    thrown), and the very next store_cached_engine_maps call overwrites it
//    (self-heals) so a subsequent load hits again.
// H3 engine_map_cache_key changes with every individual CycleModelDesc/
//    FuelDesc field, with kGeneratorVersion, and with MapQuality - but
//    (by construction: the key is computed from the RESOLVED structs, never
//    from JSON text - map_cache.h's own PROVISIONAL DECISION P-F) is
//    invariant to how the source JSON was formatted, since no JSON text
//    ever enters the key computation.
// H4 the write is atomic: a foreign/stale temp file left in cache_dir is
//    never picked up as a real cache entry (its name never matches
//    "<key:016x>.psmaps"), and a genuine store leaves exactly one file
//    behind with that exact name.

#include "engine_cycle/map_cache.h"

#include "cycle_fixtures.h"
#include "mvem_fixtures.h"

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/fuel_desc.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
namespace fx = ps::test::mvem;

namespace {

// Same fixture geometry as test_engine_map_generator.cpp's own (anonymous-
// namespace, not shared across translation units) fixture_desc() - a small
// generic I4 cycle block with a real friction polynomial, cheap enough
// (Preview quality, a 4x3 grid) for a fast unit test.
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

bool same_bits(real a, real b) { return std::memcmp(&a, &b, sizeof(real)) == 0; }
bool same_bits(const std::vector<real>& a, const std::vector<real>& b) {
    return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(real)) == 0);
}
bool same_bits(const Table2D& a, const Table2D& b) { return same_bits(a.x, b.x) && same_bits(a.y, b.y) && same_bits(a.values, b.values); }
bool same_bits(const vehicle::Curve1D& a, const vehicle::Curve1D& b) { return same_bits(a.x, b.x) && same_bits(a.y, b.y); }

bool same_engine_maps(const EngineMaps& a, const EngineMaps& b) {
    return same_bits(a.maps.ve, b.maps.ve) && same_bits(a.maps.eta_i, b.maps.eta_i) &&
           same_bits(a.maps.f_lambda, b.maps.f_lambda) && same_bits(a.maps.eta_spark, b.maps.eta_spark) &&
           same_bits(a.maps.theta_mbt_deg, b.maps.theta_mbt_deg) &&
           same_bits(a.maps.friction_torque_nm, b.maps.friction_torque_nm) &&
           a.maps.has_pmep_excess == b.maps.has_pmep_excess && same_bits(a.maps.pmep_excess_pa, b.maps.pmep_excess_pa) &&
           a.maps.has_theta_kl == b.maps.has_theta_kl && same_bits(a.maps.theta_kl_deg, b.maps.theta_kl_deg) &&
           a.maps.source == b.maps.source && same_bits(a.t_exh_k, b.t_exh_k) && same_bits(a.h_exh_w, b.h_exh_w) &&
           same_bits(a.eta_spark_spread, b.eta_spark_spread) && a.generator_version == b.generator_version &&
           a.quality == b.quality && a.content_hash == b.content_hash;
}

struct TempCacheDir {
    std::filesystem::path dir;
    TempCacheDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_engine_map_cache_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::create_directories(dir);
    }
    ~TempCacheDir() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    std::string path() const { return dir.string(); }
    std::vector<std::string> entries() const {
        std::vector<std::string> names;
        for (const auto& e : std::filesystem::directory_iterator(dir)) names.push_back(e.path().filename().string());
        return names;
    }
};

std::string cache_file_name(std::uint64_t key) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%016llx.psmaps", static_cast<unsigned long long>(key));
    return buf;
}

std::filesystem::path cache_file_path(const TempCacheDir& c, std::uint64_t key) { return c.dir / cache_file_name(key); }

std::vector<char> read_all(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    return std::vector<char>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
void write_all(const std::filesystem::path& p, const std::vector<char>& bytes) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

// --- H1's loader-level (vehicle_io.cpp) helpers. Copied verbatim in
// structure from test_engine_map_generator.cpp's own known-good
// cycle_json/base_engine_json/cycle_only_engine/vehicle_with_engine
// (anonymous-namespace there, not shared across translation units) -
// that file's helpers are the verified-correct schema for
// read_cycle_block()/load_vehicle_json(); do not hand-roll this schema
// from memory (mm/cc/kPa-suffixed cycle-block keys, bore/stroke/cylinders
// on the ENGINE not the cycle block, "lift_mm_vs_deg", a required cycle
// "source", components[] entries ARE the engine json directly - no
// "params" wrapper). ---

std::string gasoline_path() { return std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string(); }

nlohmann::json curve_json(const vehicle::Curve1D& c, real scale_y) {
    nlohmann::json x = nlohmann::json::array(), y = nlohmann::json::array();
    for (real v : c.x) x.push_back(v);
    for (real v : c.y) y.push_back(v * scale_y);
    return {{"x", x}, {"y", y}};
}

// The fixture desc as a "cycle" block (mm, cc, kPa). Engine geometry
// (bore/stroke/cylinders) comes from the engine's own piston fields, not
// the cycle block - read_cycle_block() explicitly rejects bore/stroke/
// cylinders/cycle keys there.
nlohmann::json cycle_json(const CycleModelDesc& d, bool with_friction) {
    nlohmann::json c;
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
    nlohmann::json g = nlohmann::json::array(), p = nlohmann::json::array();
    for (real v : d.grid_rpm) g.push_back(v);
    for (real v : d.grid_p_im) p.push_back(v / 1e3);
    c["grid_rpm"] = g;
    c["grid_p_im_kpa"] = p;
    return c;
}

// test_simulated_engine_io.cpp's base_simulated_engine() (86/86 mm I4,
// synthetic test maps, not physical data).
nlohmann::json base_engine_json() {
    nlohmann::json j;
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
        {"source", "synthetic test fixture (test_engine_map_cache.cpp), not physical data"},
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
nlohmann::json cycle_only_engine_json() {
    nlohmann::json j = base_engine_json();
    j.erase("maps");
    j["cycle"] = cycle_json(fixture_desc(), true);
    j["breakaway_torque_nm"] = 40.0;
    return j;
}

nlohmann::json vehicle_with_engine_json(const nlohmann::json& engine) {
    nlohmann::json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_engine_map_cache.cpp)";
    v["wheels"] = nlohmann::json::array({{{"name", "FL"}}});
    v["powertrain"] = {{"components", nlohmann::json::array({engine})}};
    return v;
}

struct TempJsonDir {
    std::filesystem::path dir;
    TempJsonDir() {
        dir = std::filesystem::temp_directory_path() / ("ps_test_engine_map_cache_veh_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::filesystem::create_directories(dir);
    }
    ~TempJsonDir() {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }
    std::filesystem::path write_vehicle(const nlohmann::json& veh) const {
        const auto p = dir / "vehicle.json";
        std::ofstream(p) << veh.dump(2);
        return p;
    }
};

vehicle::VehicleDesc load_with_cache(const std::string& cache_dir, std::size_t* count) {
    TempJsonDir d;
    const auto veh = vehicle_with_engine_json(cycle_only_engine_json());
    const auto path = d.write_vehicle(veh);
    io::EngineMapOptions opt;
    opt.quality = MapQuality::Preview;
    opt.generation_count_out = count;
    opt.cache_dir = cache_dir;
    return io::load_vehicle_json(path.string(), opt);
}

const SimulatedEngineMapsDesc& only_sim_maps(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components) {
        if (const auto* s = std::get_if<SimulatedEngineDesc>(&c.params)) return s->maps;
    }
    FAIL("no SimulatedEngineDesc in vehicle");
    static SimulatedEngineMapsDesc dummy;
    return dummy;
}

} // namespace

TEST_CASE("C16b H1: a cache hit is bit-for-bit identical to a fresh generation", "[drivetrain][cycle][io]") {
    const CycleModelDesc desc = fixture_desc();
    const FuelDesc fuel = fx::gasoline();

    SECTION("map_cache.h directly") {
        TempCacheDir cache;
        const std::uint64_t key = engine_map_cache_key(desc, fuel, MapQuality::Preview);
        REQUIRE(load_cached_engine_maps(cache.path(), key) == nullptr); // empty dir: miss

        const EngineMapsPtr fresh = generate_engine_maps(desc, fuel, MapQuality::Preview);
        store_cached_engine_maps(cache.path(), key, *fresh);

        const EngineMapsPtr loaded = load_cached_engine_maps(cache.path(), key);
        REQUIRE(loaded != nullptr);
        CHECK(same_engine_maps(*fresh, *loaded));
        CHECK(loaded->content_hash == fresh->content_hash);

        // The generator itself is deterministic (C16a M3): a second
        // independent fresh generation must match the cached copy too.
        const EngineMapsPtr fresh2 = generate_engine_maps(desc, fuel, MapQuality::Preview);
        CHECK(same_engine_maps(*fresh2, *loaded));

        const auto entries = cache.entries();
        REQUIRE(entries.size() == 1);
        CHECK(entries.front() == cache_file_name(key));
    }

    SECTION("through load_vehicle_json's cache_dir + generation_count_out wiring") {
        TempCacheDir cache;
        std::size_t n1 = 0, n2 = 0;
        const vehicle::VehicleDesc v1 = load_with_cache(cache.path(), &n1);
        CHECK(n1 == 1); // empty cache: the generator ran once

        const vehicle::VehicleDesc v2 = load_with_cache(cache.path(), &n2);
        CHECK(n2 == 0); // same cache_dir, same engine/fuel/quality: a hit, generator not run again

        const SimulatedEngineMapsDesc& m1 = only_sim_maps(v1);
        const SimulatedEngineMapsDesc& m2 = only_sim_maps(v2);
        CHECK(same_bits(m1.ve, m2.ve));
        CHECK(same_bits(m1.eta_i, m2.eta_i));
        CHECK(same_bits(m1.friction_torque_nm, m2.friction_torque_nm));
        CHECK(m1.source == m2.source);
    }
}

// C16b review R4 (item 2a): the generated maps' provenance string embeds the
// cycle block's source (map_generator.cpp), so editing only that citation
// must not be served the old string from the cache.
TEST_CASE("C16b review R4: a cycle-source edit misses the cache and the loaded maps carry the new source", "[drivetrain][cycle][io]") {
    TempCacheDir cache;
    const auto load = [&](const std::string& cycle_source, std::size_t* count) {
        TempJsonDir d;
        nlohmann::json engine = cycle_only_engine_json();
        engine["cycle"]["source"] = cycle_source;
        const auto path = d.write_vehicle(vehicle_with_engine_json(engine));
        io::EngineMapOptions opt;
        opt.quality = MapQuality::Preview;
        opt.generation_count_out = count;
        opt.cache_dir = cache.path();
        return io::load_vehicle_json(path.string(), opt);
    };
    std::size_t n1 = 0, n2 = 0, n3 = 0;
    const vehicle::VehicleDesc v1 = load("citation A", &n1);
    const vehicle::VehicleDesc v2 = load("citation B", &n2);
    const vehicle::VehicleDesc v3 = load("citation B", &n3);
    CHECK(n1 == 1);
    CHECK(n2 == 1); // a different citation is a different key: generated, not served A's maps
    CHECK(n3 == 0); // and B's own entry is then a hit
    CHECK(only_sim_maps(v1).source.find("(cycle source: citation A)") != std::string::npos);
    CHECK(only_sim_maps(v2).source.find("(cycle source: citation B)") != std::string::npos);
    CHECK(only_sim_maps(v3).source == only_sim_maps(v2).source);
    CHECK(same_bits(only_sim_maps(v1).ve, only_sim_maps(v2).ve)); // physics unchanged
}

TEST_CASE("C16b H2: a corrupt or mismatched cache file is silently discarded, never thrown, and self-heals",
          "[drivetrain][cycle][io]") {
    TempCacheDir cache;
    const CycleModelDesc desc = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    const std::uint64_t key = engine_map_cache_key(desc, fuel, MapQuality::Preview);
    const auto file = cache_file_path(cache, key);

    const auto make_valid_file = [&]() {
        const EngineMapsPtr m = generate_engine_maps(desc, fuel, MapQuality::Preview);
        store_cached_engine_maps(cache.path(), key, *m);
        REQUIRE(std::filesystem::exists(file));
    };

    SECTION("truncated: cuts off mid-header") {
        make_valid_file();
        auto bytes = read_all(file);
        REQUIRE(bytes.size() > 40);
        bytes.resize(20);
        write_all(file, bytes);
        CHECK_NOTHROW(load_cached_engine_maps(cache.path(), key));
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }
    SECTION("truncated: header intact, payload short") {
        make_valid_file();
        auto bytes = read_all(file);
        REQUIRE(bytes.size() > 50);
        bytes.resize(bytes.size() - 8);
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }
    SECTION("wrong magic") {
        make_valid_file();
        auto bytes = read_all(file);
        bytes[0] = 'X';
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }
    SECTION("wrong format_version") {
        make_valid_file();
        auto bytes = read_all(file);
        std::uint32_t bad = kPsMapsFormatVersion + 1;
        std::memcpy(bytes.data() + 8, &bad, sizeof(bad));
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }
    SECTION("wrong key recorded in the header") {
        make_valid_file();
        auto bytes = read_all(file);
        std::uint64_t bad = key ^ 0x1;
        std::memcpy(bytes.data() + 12, &bad, sizeof(bad));
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }
    SECTION("corrupted payload: content_hash mismatch") {
        make_valid_file();
        auto bytes = read_all(file);
        REQUIRE(bytes.size() > 60);
        bytes[55] = static_cast<char>(bytes[55] ^ 0xFF); // flip a byte well inside the payload
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }
    SECTION("nonexistent cache_dir") {
        std::filesystem::remove_all(cache.dir);
        CHECK_NOTHROW(load_cached_engine_maps(cache.path(), key));
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr);
    }

    // Every corruption case above must self-heal: exactly the sequence
    // vehicle_io.cpp's C16b wiring runs on a miss (generate, then store)
    // leaves a GOOD file behind, and the next load hits it.
    const EngineMapsPtr regenerated = generate_engine_maps(desc, fuel, MapQuality::Preview);
    CHECK_NOTHROW(store_cached_engine_maps(cache.path(), key, *regenerated));
    const EngineMapsPtr healed = load_cached_engine_maps(cache.path(), key);
    REQUIRE(healed != nullptr);
    CHECK(same_engine_maps(*regenerated, *healed));
}

TEST_CASE("C16b H3: the cache key changes with every field, the generator version and quality - never with JSON formatting",
          "[drivetrain][cycle][io]") {
    const CycleModelDesc ref_desc = fixture_desc();
    const FuelDesc ref_fuel = fx::gasoline();
    const std::uint64_t ref_key = engine_map_cache_key(ref_desc, ref_fuel, MapQuality::Preview);

    SECTION("quality changes the key") { CHECK(engine_map_cache_key(ref_desc, ref_fuel, MapQuality::Full) != ref_key); }

    SECTION("the fuel's source citation does NOT change the key (the generator never reads it)") {
        FuelDesc fuel2 = ref_fuel;
        fuel2.source = fuel2.source + " (mutated citation only)";
        CHECK(engine_map_cache_key(ref_desc, fuel2, MapQuality::Preview) == ref_key);
    }

    SECTION("the cycle block's source DOES change the key (C16b review: it ends up in EngineMaps::source)") {
        CycleModelDesc d = ref_desc;
        d.source += " (mutated citation only)";
        CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != ref_key);
    }

    SECTION("every CycleModelDesc field bumps the key") {
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };
        std::vector<std::pair<const char*, std::function<void(CycleModelDesc&)>>> edits;
        edits.emplace_back("bore_m", [&](CycleModelDesc& d) { d.bore_m = bump(d.bore_m); });
        edits.emplace_back("stroke_m", [&](CycleModelDesc& d) { d.stroke_m = bump(d.stroke_m); });
        edits.emplace_back("rod_m", [&](CycleModelDesc& d) { d.rod_m = bump(d.rod_m); });
        edits.emplace_back("pin_offset_m", [&](CycleModelDesc& d) { d.pin_offset_m = bump(d.pin_offset_m + 1e-6); });
        edits.emplace_back("compression_ratio", [&](CycleModelDesc& d) { d.compression_ratio = bump(d.compression_ratio); });
        edits.emplace_back("cylinders", [&](CycleModelDesc& d) { d.cylinders += 2; });
        edits.emplace_back("cycle", [&](CycleModelDesc& d) { d.cycle = (d.cycle == 4) ? 2 : 4; });
        edits.emplace_back("intake_lift.phase_ref_deg", [&](CycleModelDesc& d) { d.intake_lift.phase_ref_deg = bump(d.intake_lift.phase_ref_deg); });
        edits.emplace_back("exhaust_lift.phase_ref_deg", [&](CycleModelDesc& d) { d.exhaust_lift.phase_ref_deg = bump(d.exhaust_lift.phase_ref_deg); });
        edits.emplace_back("intake_lift curve y", [&](CycleModelDesc& d) { d.intake_lift.lift_m_vs_deg.y.front() = bump(d.intake_lift.lift_m_vs_deg.y.front()); });
        edits.emplace_back("intake_valves", [&](CycleModelDesc& d) { d.intake_valves += 1; });
        edits.emplace_back("exhaust_valves", [&](CycleModelDesc& d) { d.exhaust_valves += 1; });
        edits.emplace_back("intake_valve_d_m", [&](CycleModelDesc& d) { d.intake_valve_d_m = bump(d.intake_valve_d_m); });
        edits.emplace_back("exhaust_valve_d_m", [&](CycleModelDesc& d) { d.exhaust_valve_d_m = bump(d.exhaust_valve_d_m); });
        edits.emplace_back("runner_length_m", [&](CycleModelDesc& d) { d.runner_length_m = bump(d.runner_length_m); });
        edits.emplace_back("runner_diameter_m", [&](CycleModelDesc& d) { d.runner_diameter_m = bump(d.runner_diameter_m); });
        edits.emplace_back("intake_port_m3", [&](CycleModelDesc& d) { d.intake_port_m3 = bump(d.intake_port_m3); });
        edits.emplace_back("exhaust_port_m3", [&](CycleModelDesc& d) { d.exhaust_port_m3 = bump(d.exhaust_port_m3); });
        edits.emplace_back("cd_vs_l_over_d y", [&](CycleModelDesc& d) { d.cd_vs_l_over_d.y.front() = bump(d.cd_vs_l_over_d.y.front()); });
        edits.emplace_back("wiebe_a", [&](CycleModelDesc& d) { d.wiebe_a = bump(d.wiebe_a + 1.0); });
        edits.emplace_back("wiebe_m", [&](CycleModelDesc& d) { d.wiebe_m = bump(d.wiebe_m + 1.0); });
        edits.emplace_back("burn_duration_deg", [&](CycleModelDesc& d) { d.burn_duration_deg = bump(d.burn_duration_deg + 1.0); });
        edits.emplace_back("ca50_mbt_deg_atdc", [&](CycleModelDesc& d) { d.ca50_mbt_deg_atdc = bump(d.ca50_mbt_deg_atdc + 1.0); });
        edits.emplace_back("woschni_c1_gx", [&](CycleModelDesc& d) { d.woschni_c1_gx = bump(d.woschni_c1_gx + 1.0); });
        edits.emplace_back("woschni_c1_cc", [&](CycleModelDesc& d) { d.woschni_c1_cc = bump(d.woschni_c1_cc + 1.0); });
        edits.emplace_back("woschni_c2", [&](CycleModelDesc& d) { d.woschni_c2 = bump(d.woschni_c2 + 1.0); });
        edits.emplace_back("wall_temperature_k", [&](CycleModelDesc& d) { d.wall_temperature_k = bump(d.wall_temperature_k); });
        edits.emplace_back("head_area_factor", [&](CycleModelDesc& d) { d.head_area_factor = bump(d.head_area_factor + 1.0); });
        edits.emplace_back("combustion_eff_lean", [&](CycleModelDesc& d) { d.combustion_eff_lean = bump(d.combustion_eff_lean + 1.0); });
        edits.emplace_back("gas.fresh.cv_a_j_kgk", [&](CycleModelDesc& d) { d.gas.fresh.cv_a_j_kgk = bump(d.gas.fresh.cv_a_j_kgk); });
        edits.emplace_back("gas.burned.r_j_kgk", [&](CycleModelDesc& d) { d.gas.burned.r_j_kgk = bump(d.gas.burned.r_j_kgk); });
        edits.emplace_back("friction.model", [&](CycleModelDesc& d) { d.friction.model = FrictionModelDesc::Model::PolynomialRpm; });
        edits.emplace_back("friction.a_pa", [&](CycleModelDesc& d) { d.friction.a_pa = bump(d.friction.a_pa); });
        edits.emplace_back("friction.b_pa", [&](CycleModelDesc& d) { d.friction.b_pa = bump(d.friction.b_pa); });
        edits.emplace_back("friction.c_pa", [&](CycleModelDesc& d) { d.friction.c_pa = bump(d.friction.c_pa); });
        edits.emplace_back("exhaust_backflow_temperature_k", [&](CycleModelDesc& d) { d.exhaust_backflow_temperature_k = bump(d.exhaust_backflow_temperature_k + 1.0); });
        edits.emplace_back("grid_rpm", [&](CycleModelDesc& d) { d.grid_rpm.back() = bump(d.grid_rpm.back()); });
        edits.emplace_back("grid_p_im", [&](CycleModelDesc& d) { d.grid_p_im.back() = bump(d.grid_p_im.back()); });
        // C16b review: the fields the original table left out. A pushed
        // element works whether the fixture's table/curve is empty or not.
        edits.emplace_back("intake_lift curve x", [&](CycleModelDesc& d) { d.intake_lift.lift_m_vs_deg.x.front() = bump(d.intake_lift.lift_m_vs_deg.x.front()); });
        edits.emplace_back("exhaust_lift curve x", [&](CycleModelDesc& d) { d.exhaust_lift.lift_m_vs_deg.x.front() = bump(d.exhaust_lift.lift_m_vs_deg.x.front()); });
        edits.emplace_back("exhaust_lift curve y", [&](CycleModelDesc& d) { d.exhaust_lift.lift_m_vs_deg.y.front() = bump(d.exhaust_lift.lift_m_vs_deg.y.front()); });
        edits.emplace_back("cd_vs_l_over_d x", [&](CycleModelDesc& d) { d.cd_vs_l_over_d.x.back() = bump(d.cd_vs_l_over_d.x.back()); });
        edits.emplace_back("cam.intake_advance_deg x", [&](CycleModelDesc& d) { d.cam.intake_advance_deg.x.push_back(1e6); });
        edits.emplace_back("cam.intake_advance_deg y", [&](CycleModelDesc& d) { d.cam.intake_advance_deg.y.push_back(1e6); });
        edits.emplace_back("cam.intake_advance_deg values", [&](CycleModelDesc& d) { d.cam.intake_advance_deg.values.push_back(1.0); });
        edits.emplace_back("cam.exhaust_retard_deg x", [&](CycleModelDesc& d) { d.cam.exhaust_retard_deg.x.push_back(1e6); });
        edits.emplace_back("cam.exhaust_retard_deg y", [&](CycleModelDesc& d) { d.cam.exhaust_retard_deg.y.push_back(1e6); });
        edits.emplace_back("cam.exhaust_retard_deg values", [&](CycleModelDesc& d) { d.cam.exhaust_retard_deg.values.push_back(1.0); });
        edits.emplace_back("back_pressure x", [&](CycleModelDesc& d) { d.back_pressure_pa_vs_exh_kg_s.x.push_back(1e6); });
        edits.emplace_back("back_pressure y", [&](CycleModelDesc& d) { d.back_pressure_pa_vs_exh_kg_s.y.push_back(1e6); });
        edits.emplace_back("gas.fresh.cv_b_j_kgk2", [&](CycleModelDesc& d) { d.gas.fresh.cv_b_j_kgk2 = bump(d.gas.fresh.cv_b_j_kgk2); });
        edits.emplace_back("gas.fresh.r_j_kgk", [&](CycleModelDesc& d) { d.gas.fresh.r_j_kgk = bump(d.gas.fresh.r_j_kgk); });
        edits.emplace_back("gas.fresh.gamma_flow", [&](CycleModelDesc& d) { d.gas.fresh.gamma_flow = bump(d.gas.fresh.gamma_flow); });
        edits.emplace_back("gas.burned.cv_a_j_kgk", [&](CycleModelDesc& d) { d.gas.burned.cv_a_j_kgk = bump(d.gas.burned.cv_a_j_kgk); });
        edits.emplace_back("gas.burned.cv_b_j_kgk2", [&](CycleModelDesc& d) { d.gas.burned.cv_b_j_kgk2 = bump(d.gas.burned.cv_b_j_kgk2); });
        edits.emplace_back("gas.burned.gamma_flow", [&](CycleModelDesc& d) { d.gas.burned.gamma_flow = bump(d.gas.burned.gamma_flow); });
        edits.emplace_back("source", [&](CycleModelDesc& d) { d.source += "x"; });

        for (const auto& [name, edit] : edits) {
            CycleModelDesc d = ref_desc;
            edit(d);
            INFO("field: " << name);
            CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != ref_key);
        }
    }

    SECTION("every FuelDesc physics field bumps the key (name/phase/lhv/afr/density)") {
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };
        {
            FuelDesc f = ref_fuel;
            f.name += "x";
            CHECK(engine_map_cache_key(ref_desc, f, MapQuality::Preview) != ref_key);
        }
        {
            FuelDesc f = ref_fuel;
            f.lhv_j_kg = bump(f.lhv_j_kg);
            CHECK(engine_map_cache_key(ref_desc, f, MapQuality::Preview) != ref_key);
        }
        {
            FuelDesc f = ref_fuel;
            f.afr_stoich = bump(f.afr_stoich);
            CHECK(engine_map_cache_key(ref_desc, f, MapQuality::Preview) != ref_key);
        }
        {
            FuelDesc f = ref_fuel;
            f.density_kg_m3 = bump(f.density_kg_m3);
            CHECK(engine_map_cache_key(ref_desc, f, MapQuality::Preview) != ref_key);
        }
    }

    SECTION("the key is invariant to how the desc was built (JSON key order/whitespace never enters the key at all)") {
        // P-F design: engine_map_cache_key hashes the resolved CycleModelDesc
        // struct directly, never JSON text - two independently-constructed
        // but field-identical descs always hash the same, which is the only
        // sense in which "JSON formatting" can matter here.
        CycleModelDesc rebuilt = fixture_desc();
        CHECK(engine_map_cache_key(rebuilt, ref_fuel, MapQuality::Preview) == ref_key);
    }
}

TEST_CASE("C16b H4: store_cached_engine_maps writes atomically; stray temp files are never picked up", "[drivetrain][cycle][io]") {
    TempCacheDir cache;
    const CycleModelDesc desc = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    const std::uint64_t key = engine_map_cache_key(desc, fuel, MapQuality::Preview);
    const EngineMapsPtr maps = generate_engine_maps(desc, fuel, MapQuality::Preview);

    SECTION("a plausible stray .tmp file left behind never satisfies a load") {
        // Simulate debris from an interrupted/concurrent write: a temp file
        // whose name embeds the real key but is not the canonical
        // "<key:016x>.psmaps" name map_cache.cpp's rename target uses.
        const auto stray = cache_file_path(cache, key);
        write_all(std::filesystem::path(stray.string() + ".1234.5678.tmp"), {'g', 'a', 'r', 'b', 'a', 'g', 'e'});
        CHECK(load_cached_engine_maps(cache.path(), key) == nullptr); // no real file yet - the stray temp is not one
        const auto entries_before = cache.entries();
        REQUIRE(entries_before.size() == 1); // just the stray temp

        store_cached_engine_maps(cache.path(), key, *maps);

        // The real file now exists and loads; the stray temp is untouched
        // debris (store never has to see or clean up someone else's temp
        // file - only its own, on a failed write, which this is not).
        const EngineMapsPtr loaded = load_cached_engine_maps(cache.path(), key);
        REQUIRE(loaded != nullptr);
        CHECK(same_engine_maps(*maps, *loaded));

        bool saw_real_file = false;
        for (const auto& name : cache.entries()) {
            if (name == cache_file_name(key)) saw_real_file = true;
        }
        CHECK(saw_real_file);
    }

    SECTION("a completed write leaves exactly one canonically-named file, never a leftover .tmp") {
        store_cached_engine_maps(cache.path(), key, *maps);
        const auto entries = cache.entries();
        REQUIRE(entries.size() == 1);
        CHECK(entries.front() == cache_file_name(key));
        for (const auto& name : entries) CHECK_THAT(name, !ContainsSubstring(".tmp"));
    }

    SECTION("two stores of the same key (e.g. two independent regenerations) both succeed and the file stays loadable") {
        store_cached_engine_maps(cache.path(), key, *maps);
        const EngineMapsPtr maps2 = generate_engine_maps(desc, fuel, MapQuality::Preview);
        store_cached_engine_maps(cache.path(), key, *maps2); // overwrite via a fresh temp+rename, not an in-place write
        const EngineMapsPtr loaded = load_cached_engine_maps(cache.path(), key);
        REQUIRE(loaded != nullptr);
        CHECK(same_engine_maps(*maps, *loaded));
        const auto entries = cache.entries();
        REQUIRE(entries.size() == 1); // no leftover temp from either store
        CHECK(entries.front() == cache_file_name(key));
    }
}
