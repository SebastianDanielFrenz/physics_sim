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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
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
    v["wheels"] = nlohmann::json::array({{{"name", "FL"}, {"tyre", {{"source", "test fixture"}}}}});
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
        REQUIRE(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr); // empty dir: miss

        const EngineMapsPtr fresh = generate_engine_maps(desc, fuel, MapQuality::Preview);
        store_cached_engine_maps(cache.path(), key, *fresh);

        const EngineMapsPtr loaded = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc));
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
        CHECK_NOTHROW(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)));
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }
    SECTION("truncated: header intact, payload short") {
        make_valid_file();
        auto bytes = read_all(file);
        REQUIRE(bytes.size() > 50);
        bytes.resize(bytes.size() - 8);
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }
    SECTION("wrong magic") {
        make_valid_file();
        auto bytes = read_all(file);
        bytes[0] = 'X';
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }
    SECTION("wrong format_version") {
        make_valid_file();
        auto bytes = read_all(file);
        std::uint32_t bad = kPsMapsFormatVersion + 1;
        std::memcpy(bytes.data() + 8, &bad, sizeof(bad));
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }
    SECTION("wrong key recorded in the header") {
        make_valid_file();
        auto bytes = read_all(file);
        std::uint64_t bad = key ^ 0x1;
        std::memcpy(bytes.data() + 12, &bad, sizeof(bad));
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }
    SECTION("corrupted payload: content_hash mismatch") {
        make_valid_file();
        auto bytes = read_all(file);
        REQUIRE(bytes.size() > 60);
        bytes[55] = static_cast<char>(bytes[55] ^ 0xFF); // flip a byte well inside the payload
        write_all(file, bytes);
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }
    SECTION("nonexistent cache_dir") {
        std::filesystem::remove_all(cache.dir);
        CHECK_NOTHROW(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)));
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr);
    }

    // Every corruption case above must self-heal: exactly the sequence
    // vehicle_io.cpp's C16b wiring runs on a miss (generate, then store)
    // leaves a GOOD file behind, and the next load hits it.
    const EngineMapsPtr regenerated = generate_engine_maps(desc, fuel, MapQuality::Preview);
    CHECK_NOTHROW(store_cached_engine_maps(cache.path(), key, *regenerated));
    const EngineMapsPtr healed = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc));
    REQUIRE(healed != nullptr);
    CHECK(same_engine_maps(*regenerated, *healed));
}

// C16b review R5 (H2, "the file is rewritten"): the H2 case above heals by
// calling generate + store itself; this one goes through the production
// loader, so dropping the store-on-miss in vehicle_io.cpp fails it.
TEST_CASE("C16b review R5: load_vehicle_json regenerates a corrupt cache entry and rewrites it", "[drivetrain][cycle][io]") {
    TempCacheDir cache;
    std::size_t n1 = 0, n2 = 0, n3 = 0;
    const vehicle::VehicleDesc v1 = load_with_cache(cache.path(), &n1);
    REQUIRE(n1 == 1);

    std::filesystem::path file;
    int entries = 0;
    for (const auto& de : std::filesystem::directory_iterator(cache.dir)) {
        if (de.path().extension() == ".psmaps") {
            file = de.path();
            ++entries;
        }
    }
    REQUIRE(entries == 1);
    auto bytes = read_all(file);
    REQUIRE(bytes.size() > 60);
    bytes[55] = static_cast<char>(bytes[55] ^ 0xFF); // payload byte: content-hash mismatch
    write_all(file, bytes);

    vehicle::VehicleDesc v2;
    CHECK_NOTHROW(v2 = load_with_cache(cache.path(), &n2));
    CHECK(n2 == 1); // the corrupt entry is a miss: regenerated
    CHECK(read_all(file) != bytes); // and rewritten

    const vehicle::VehicleDesc v3 = load_with_cache(cache.path(), &n3);
    CHECK(n3 == 0); // the rewritten entry is a hit
    CHECK(same_bits(only_sim_maps(v1).ve, only_sim_maps(v3).ve));
    CHECK(same_bits(only_sim_maps(v1).eta_i, only_sim_maps(v3).eta_i));
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
        // C17a follow-up: the Tier 1b spark schedule and the Tier 2 ignition
        // delay (folded only when present/non-zero, so an engine without them
        // keeps its pre-follow-up key).
        edits.emplace_back("spark_advance_deg x", [&](CycleModelDesc& d) { d.spark_advance_deg.x.push_back(1e6); });
        edits.emplace_back("spark_advance_deg y", [&](CycleModelDesc& d) { d.spark_advance_deg.y.push_back(1e6); });
        edits.emplace_back("spark_advance_deg values", [&](CycleModelDesc& d) { d.spark_advance_deg.values.push_back(1.0); });
        edits.emplace_back("ignition_delay_deg", [&](CycleModelDesc& d) { d.ignition_delay_deg = bump(d.ignition_delay_deg); });
        edits.emplace_back("gas.fresh.cv_b_j_kgk2", [&](CycleModelDesc& d) { d.gas.fresh.cv_b_j_kgk2 = bump(d.gas.fresh.cv_b_j_kgk2); });
        edits.emplace_back("gas.fresh.r_j_kgk", [&](CycleModelDesc& d) { d.gas.fresh.r_j_kgk = bump(d.gas.fresh.r_j_kgk); });
        edits.emplace_back("gas.fresh.gamma_flow", [&](CycleModelDesc& d) { d.gas.fresh.gamma_flow = bump(d.gas.fresh.gamma_flow); });
        edits.emplace_back("gas.burned.cv_a_j_kgk", [&](CycleModelDesc& d) { d.gas.burned.cv_a_j_kgk = bump(d.gas.burned.cv_a_j_kgk); });
        edits.emplace_back("gas.burned.cv_b_j_kgk2", [&](CycleModelDesc& d) { d.gas.burned.cv_b_j_kgk2 = bump(d.gas.burned.cv_b_j_kgk2); });
        edits.emplace_back("gas.burned.gamma_flow", [&](CycleModelDesc& d) { d.gas.burned.gamma_flow = bump(d.gas.burned.gamma_flow); });
        edits.emplace_back("source", [&](CycleModelDesc& d) { d.source += "x"; });
        // Engine garage P7: the knock opt-in flag alone moves the key (the fixture has no knock limit).
        edits.emplace_back("knock.enabled", [&](CycleModelDesc& d) { d.knock.enabled = true; });

        for (const auto& [name, edit] : edits) {
            CycleModelDesc d = ref_desc;
            edit(d);
            INFO("field: " << name);
            CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != ref_key);
        }
    }

    SECTION("burn law (vault PHYS-079): the mode and every law parameter bump the key") {
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };
        REQUIRE(ref_desc.burn_law == CycleModelDesc::BurnLawMode::Standard);
        std::vector<std::pair<const char*, std::function<void(CycleModelDesc&)>>> edits;
        edits.emplace_back("burn_law mode", [&](CycleModelDesc& d) { d.burn_law = CycleModelDesc::BurnLawMode::Fixed; });
        using P = engine_cycle::BurnLawParams;
        const std::pair<const char*, real P::*> params[] = {
            {"reference_rpm", &P::reference_rpm},
            {"reference_unburned_temperature_k", &P::reference_unburned_temperature_k},
            {"reference_unburned_pressure_pa", &P::reference_unburned_pressure_pa},
            {"reference_residual_fraction", &P::reference_residual_fraction},
            {"development_reference_deg", &P::development_reference_deg},
            {"rapid_reference_deg", &P::rapid_reference_deg},
            {"scale", &P::scale},
            {"rpm_exponent", &P::rpm_exponent},
            {"flame_speed_exponent", &P::flame_speed_exponent},
            {"flame_speed_pressure_factor", &P::flame_speed_pressure_factor},
            {"mk_phi_m", &P::mk_phi_m},
            {"mk_b_m_m_s", &P::mk_b_m_m_s},
            {"mk_b_2_m_s", &P::mk_b_2_m_s},
            {"mk_alpha_0", &P::mk_alpha_0},
            {"mk_alpha_phi", &P::mk_alpha_phi},
            {"mk_beta_0", &P::mk_beta_0},
            {"mk_beta_phi", &P::mk_beta_phi},
            {"mk_t0_k", &P::mk_t0_k},
            {"mk_p0_pa", &P::mk_p0_pa},
            {"mk_phi_min", &P::mk_phi_min},
            {"mk_phi_max", &P::mk_phi_max},
            {"dilution_k", &P::dilution_k},
            {"dilution_exponent", &P::dilution_exponent},
            {"min_dilution_factor", &P::min_dilution_factor},
            {"min_flame_speed_m_s", &P::min_flame_speed_m_s},
            {"compression_exponent", &P::compression_exponent},
            {"pressure_exponent", &P::pressure_exponent},
        };
        static_assert(sizeof(params) / sizeof(params[0]) == 27, "every BurnLawParams field has a row");
        static_assert(sizeof(P) == 27 * sizeof(real), "a new BurnLawParams field needs a row here");
        for (const auto& [name, field] : params) {
            edits.emplace_back(name, [field = field, &bump](CycleModelDesc& d) {
                d.burn_law_params.*field = bump(d.burn_law_params.*field);
            });
        }
        for (const auto& [name, edit] : edits) {
            CycleModelDesc d = ref_desc;
            edit(d);
            INFO("burn law field: " << name);
            CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != ref_key);
        }
        // A Fixed desc does not read the law, so its parameters are outside its key.
        CycleModelDesc fixed = ref_desc;
        fixed.burn_law = CycleModelDesc::BurnLawMode::Fixed;
        CycleModelDesc fixed2 = fixed;
        fixed2.burn_law_params.scale = 2.0;
        CHECK(engine_map_cache_key(fixed, ref_fuel, MapQuality::Preview) ==
              engine_map_cache_key(fixed2, ref_fuel, MapQuality::Preview));
    }

    SECTION("with a spark schedule present, every schedule value and the delay still bump the key (C17a follow-up)") {
        // The fields above start from an engine WITHOUT a schedule, where the
        // presence condition alone would change the key; this base has one.
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };
        CycleModelDesc base = ref_desc;
        base.spark_advance_deg.x = {1000.0, 5500.0};
        base.spark_advance_deg.y = {30000.0, 95000.0};
        base.spark_advance_deg.values = {20.0, 10.0, 25.0, 15.0};
        base.ignition_delay_deg = 3.0;
        const std::uint64_t base_key = engine_map_cache_key(base, ref_fuel, MapQuality::Preview);
        CHECK(base_key != ref_key);
        std::vector<std::pair<const char*, std::function<void(CycleModelDesc&)>>> edits;
        edits.emplace_back("spark x", [&](CycleModelDesc& d) { d.spark_advance_deg.x.back() = bump(d.spark_advance_deg.x.back()); });
        edits.emplace_back("spark y", [&](CycleModelDesc& d) { d.spark_advance_deg.y.back() = bump(d.spark_advance_deg.y.back()); });
        edits.emplace_back("spark values", [&](CycleModelDesc& d) { d.spark_advance_deg.values[2] = bump(d.spark_advance_deg.values[2]); });
        edits.emplace_back("ignition_delay_deg", [&](CycleModelDesc& d) { d.ignition_delay_deg = bump(d.ignition_delay_deg); });
        for (const auto& [name, edit] : edits) {
            CycleModelDesc d = base;
            edit(d);
            INFO(name);
            CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != base_key);
        }
    }

    SECTION("engine garage P7: with a knock limit, every knock field bumps the key; a fuel's own octane never does") {
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };
        CycleModelDesc base = ref_desc;
        base.knock.enabled = true;
        base.knock.margin_deg = 2.0;
        base.knock.fuels = {{"e10", 95.0, 85.0}, {"e85", 104.0, 88.0}};
        // P7b: the replay's lambda target (the loader copies ecu.lambda_target) is a generator input.
        base.knock.lambda_target.x = {1000.0, 6000.0};
        base.knock.lambda_target.y = {30000.0, 100000.0};
        base.knock.lambda_target.values = {1.0, 1.0, 1.0, 0.85};
        const std::uint64_t base_key = engine_map_cache_key(base, ref_fuel, MapQuality::Preview);
        CHECK(base_key != ref_key);
        std::vector<std::pair<const char*, std::function<void(CycleModelDesc&)>>> edits;
        edits.emplace_back("knock.margin_deg", [&](CycleModelDesc& d) { d.knock.margin_deg = bump(d.knock.margin_deg); });
        edits.emplace_back("knock.fuels[0].ron", [&](CycleModelDesc& d) { d.knock.fuels[0].ron = bump(d.knock.fuels[0].ron); });
        edits.emplace_back("knock.fuels[1].mon", [&](CycleModelDesc& d) { d.knock.fuels[1].mon = bump(d.knock.fuels[1].mon); });
        edits.emplace_back("knock.fuels[1].name", [&](CycleModelDesc& d) { d.knock.fuels[1].name += "x"; });
        edits.emplace_back("knock.fuels order", [&](CycleModelDesc& d) { std::swap(d.knock.fuels[0], d.knock.fuels[1]); });
        edits.emplace_back("knock.fuels count", [&](CycleModelDesc& d) { d.knock.fuels.pop_back(); });
        edits.emplace_back("knock.lambda_target value",
                           [&](CycleModelDesc& d) { d.knock.lambda_target.values[3] = bump(d.knock.lambda_target.values[3]); });
        edits.emplace_back("knock.lambda_target x axis",
                           [&](CycleModelDesc& d) { d.knock.lambda_target.x[1] = bump(d.knock.lambda_target.x[1]); });
        edits.emplace_back("knock.lambda_target y axis",
                           [&](CycleModelDesc& d) { d.knock.lambda_target.y[0] = bump(d.knock.lambda_target.y[0]); });
        // The knock model's parameter set (v3-v5), one row per field.
        edits.emplace_back("knock.model.prefactor_scale", [&](CycleModelDesc& d) { d.knock.model.prefactor_scale = bump(d.knock.model.prefactor_scale); });
        edits.emplace_back("knock.model.pressure_exponent", [&](CycleModelDesc& d) { d.knock.model.pressure_exponent = bump(d.knock.model.pressure_exponent); });
        edits.emplace_back("knock.model.activation_k", [&](CycleModelDesc& d) { d.knock.model.activation_k = bump(d.knock.model.activation_k); });
        edits.emplace_back("knock.model.octane_index_k", [&](CycleModelDesc& d) { d.knock.model.octane_index_k = bump(d.knock.model.octane_index_k); });
        edits.emplace_back("knock.model.dilution_exponent", [&](CycleModelDesc& d) { d.knock.model.dilution_exponent = bump(d.knock.model.dilution_exponent); });
        edits.emplace_back("knock.model.end_burn_fraction", [&](CycleModelDesc& d) { d.knock.model.end_burn_fraction = bump(d.knock.model.end_burn_fraction); });
        for (const auto& [name, edit] : edits) {
            CycleModelDesc d = base;
            edit(d);
            INFO(name);
            CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != base_key);
        }
        // The generator reads octane from desc.knock (filled by the loader), never from the FuelDesc it is handed:
        // a fuel's knock block is outside the key, for opted-in engines too.
        FuelDesc f = ref_fuel;
        f.has_knock = true;
        f.knock.ron = 99.0;
        f.knock.mon = 88.0;
        CHECK(engine_map_cache_key(base, f, MapQuality::Preview) == base_key);
        CHECK(engine_map_cache_key(ref_desc, f, MapQuality::Preview) == ref_key);
        // Not opted in: a knock block filled but disabled leaves the key untouched (only `enabled` gates it).
        CycleModelDesc off = base;
        off.knock.enabled = false;
        CHECK(engine_map_cache_key(off, ref_fuel, MapQuality::Preview) == ref_key);
    }

    // ps_audio A4 (out/audio_plan.md section 3.1/3.2/3.3): the intake/exhaust
    // pipe pair, the "resolution" cell-size block and a resolved
    // InstallationDesc. Every ACOUSTIC field must bump the key; positions
    // (inlet_position_m, an exhaust element's position_m) and every
    // provenance-only string (intake/exhaust/installation `source`,
    // default_installation_path, installation_path) must NOT.
    SECTION("with intake/exhaust pipe blocks and a resolved installation present, every acoustic field bumps the key; "
            "positions and source/path strings do not") {
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };

        CycleModelDesc base = ref_desc;
        IntakeCycleDesc intake;
        intake.throttle_bore_m = 0.060;
        intake.plenum_m3 = 0.0025;
        RunnerPipeDesc runner;
        runner.length_m = 0.35;
        runner.d_in_m = 0.045;
        runner.d_out_m = 0.040;
        intake.runners = {runner}; // empty cylinders = "all"
        intake.interface = "throttle_inlet";
        intake.source = "test fixture, not physical data";
        base.intake = intake;

        ExhaustCycleDesc exhaust;
        PrimaryPipeDesc primary;
        primary.length_m = 0.40;
        primary.d_m = 0.036;
        exhaust.primaries = {primary};
        MergeCycleDesc merge;
        MergeSourceRef c1;
        c1.is_cylinder = true;
        c1.cylinder = 1;
        MergeSourceRef c2;
        c2.is_cylinder = true;
        c2.cylinder = 2;
        merge.from = {c1, c2};
        merge.into = "flange";
        merge.length_m = 0.10;
        merge.d_m = 0.050;
        exhaust.merges = {merge};
        exhaust.interface = "flange";
        exhaust.source = "test fixture, not physical data";
        base.exhaust = exhaust;

        base.resolution = ResolutionCycleDesc{};
        base.default_installation_path = "/fixture/default_installation.json";

        InstallationDesc inst;
        inst.intake.airbox_m3 = 0.010;
        inst.intake.has_snorkel = true;
        inst.intake.snorkel.length_m = 0.4;
        inst.intake.snorkel.d_m = 0.07;
        inst.intake.inlet_position_m = Vec3{0.9, 0.3, 0.5};
        ExhaustElementDesc cat;
        cat.type = ExhaustElementType::Catalyst;
        cat.length_m = 0.12;
        cat.d_m = 0.11;
        cat.k_loss = 4.0;
        ExhaustElementDesc pipe1;
        pipe1.type = ExhaustElementType::Pipe;
        pipe1.length_m = 1.6;
        pipe1.d_m = 0.05;
        ExhaustElementDesc outlet;
        outlet.type = ExhaustElementType::Outlet;
        outlet.position_m = Vec3{-2.2, -0.45, 0.25};
        inst.exhaust.elements = {cat, pipe1, outlet};
        inst.source = "test fixture, not physical data";
        base.installation = inst;
        base.installation_path = "/fixture/resolved_installation.json";

        const std::uint64_t base_key = engine_map_cache_key(base, ref_fuel, MapQuality::Preview);
        CHECK(base_key != ref_key); // presence alone (vs. ref_desc's no pipes) must bump the key

        std::vector<std::pair<const char*, std::function<void(CycleModelDesc&)>>> edits;
        edits.emplace_back("intake.throttle_bore_m", [&](CycleModelDesc& d) { d.intake->throttle_bore_m = bump(d.intake->throttle_bore_m); });
        edits.emplace_back("intake.plenum_m3", [&](CycleModelDesc& d) { d.intake->plenum_m3 = bump(d.intake->plenum_m3); });
        edits.emplace_back("intake.compressor_supply", [&](CycleModelDesc& d) { d.intake->compressor_supply = true; });
        edits.emplace_back("intake.runners[0].cylinders", [&](CycleModelDesc& d) { d.intake->runners[0].cylinders = {1}; });
        edits.emplace_back("intake.runners[0].length_m", [&](CycleModelDesc& d) { d.intake->runners[0].length_m = bump(d.intake->runners[0].length_m); });
        edits.emplace_back("intake.runners[0].d_in_m", [&](CycleModelDesc& d) { d.intake->runners[0].d_in_m = bump(d.intake->runners[0].d_in_m); });
        edits.emplace_back("intake.runners[0].d_out_m", [&](CycleModelDesc& d) { d.intake->runners[0].d_out_m = bump(d.intake->runners[0].d_out_m); });
        edits.emplace_back("intake.runners count", [&](CycleModelDesc& d) { d.intake->runners.push_back(d.intake->runners[0]); });
        edits.emplace_back("intake.interface", [&](CycleModelDesc& d) { d.intake->interface += "x"; });
        edits.emplace_back("exhaust.primaries[0].cylinders", [&](CycleModelDesc& d) { d.exhaust->primaries[0].cylinders = {1}; });
        edits.emplace_back("exhaust.primaries[0].length_m", [&](CycleModelDesc& d) { d.exhaust->primaries[0].length_m = bump(d.exhaust->primaries[0].length_m); });
        edits.emplace_back("exhaust.primaries[0].d_m", [&](CycleModelDesc& d) { d.exhaust->primaries[0].d_m = bump(d.exhaust->primaries[0].d_m); });
        edits.emplace_back("exhaust.merges[0].from[0].cylinder", [&](CycleModelDesc& d) { d.exhaust->merges[0].from[0].cylinder = 3; });
        edits.emplace_back("exhaust.merges[0].from[0].is_cylinder", [&](CycleModelDesc& d) {
            d.exhaust->merges[0].from[0].is_cylinder = false;
            d.exhaust->merges[0].from[0].name = "sec_a";
        });
        edits.emplace_back("exhaust.merges[0].into", [&](CycleModelDesc& d) {
            d.exhaust->merges[0].into += "x";
            d.exhaust->interface += "x"; // keep interface == last merge's into, not that the key cares
        });
        edits.emplace_back("exhaust.merges[0].length_m", [&](CycleModelDesc& d) { d.exhaust->merges[0].length_m = bump(d.exhaust->merges[0].length_m); });
        edits.emplace_back("exhaust.merges[0].d_m", [&](CycleModelDesc& d) { d.exhaust->merges[0].d_m = bump(d.exhaust->merges[0].d_m); });
        edits.emplace_back("exhaust.merges count", [&](CycleModelDesc& d) { d.exhaust->merges.push_back(d.exhaust->merges[0]); });
        edits.emplace_back("exhaust.interface", [&](CycleModelDesc& d) { d.exhaust->interface += "x"; });
        edits.emplace_back("resolution.power_dx_m", [&](CycleModelDesc& d) { d.resolution.power_dx_m = bump(d.resolution.power_dx_m); });
        edits.emplace_back("resolution.audio_dx_exhaust_m", [&](CycleModelDesc& d) { d.resolution.audio_dx_exhaust_m = bump(d.resolution.audio_dx_exhaust_m); });
        edits.emplace_back("resolution.audio_dx_intake_m", [&](CycleModelDesc& d) { d.resolution.audio_dx_intake_m = bump(d.resolution.audio_dx_intake_m); });
        edits.emplace_back("installation presence", [&](CycleModelDesc& d) { d.installation.reset(); });
        edits.emplace_back("installation.intake.airbox_m3", [&](CycleModelDesc& d) { d.installation->intake.airbox_m3 = bump(d.installation->intake.airbox_m3); });
        edits.emplace_back("installation.intake.has_snorkel", [&](CycleModelDesc& d) { d.installation->intake.has_snorkel = false; });
        edits.emplace_back("installation.intake.snorkel.length_m", [&](CycleModelDesc& d) { d.installation->intake.snorkel.length_m = bump(d.installation->intake.snorkel.length_m); });
        edits.emplace_back("installation.intake.snorkel.d_m", [&](CycleModelDesc& d) { d.installation->intake.snorkel.d_m = bump(d.installation->intake.snorkel.d_m); });
        edits.emplace_back("installation.exhaust.elements[0].type", [&](CycleModelDesc& d) { d.installation->exhaust.elements[0].type = ExhaustElementType::Pipe; });
        edits.emplace_back("installation.exhaust.elements[0].length_m", [&](CycleModelDesc& d) { d.installation->exhaust.elements[0].length_m = bump(d.installation->exhaust.elements[0].length_m); });
        edits.emplace_back("installation.exhaust.elements[0].d_m", [&](CycleModelDesc& d) { d.installation->exhaust.elements[0].d_m = bump(d.installation->exhaust.elements[0].d_m); });
        edits.emplace_back("installation.exhaust.elements[0].k_loss", [&](CycleModelDesc& d) { d.installation->exhaust.elements[0].k_loss = bump(d.installation->exhaust.elements[0].k_loss); });
        edits.emplace_back("installation.exhaust.elements count", [&](CycleModelDesc& d) { d.installation->exhaust.elements.push_back(d.installation->exhaust.elements[0]); });
        edits.emplace_back("installation.exhaust branch element (structure)", [&](CycleModelDesc& d) {
            ExhaustElementDesc branch;
            branch.type = ExhaustElementType::Branch;
            ExhaustElementDesc leaf_outlet;
            leaf_outlet.type = ExhaustElementType::Outlet;
            branch.branches = {{leaf_outlet}, {leaf_outlet}};
            d.installation->exhaust.elements.back() = branch; // replace the outlet with a branch
        });
        // ps_audio A5: the generator v2 phases the cylinders by these.
        edits.emplace_back("firing_order", [&](CycleModelDesc& d) { d.firing_order = {1, 2, 4, 3}; });
        edits.emplace_back("firing_angles_deg", [&](CycleModelDesc& d) { d.firing_angles_deg = {0.0, 180.0, 360.0, 540.0}; });

        for (const auto& [name, edit] : edits) {
            CycleModelDesc d = base;
            edit(d);
            INFO("field: " << name);
            CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) != base_key);
        }

        SECTION("positions and source/path strings do NOT change the key") {
            std::vector<std::pair<const char*, std::function<void(CycleModelDesc&)>>> excluded;
            excluded.emplace_back("intake.source", [](CycleModelDesc& d) { d.intake->source += "x"; });
            excluded.emplace_back("exhaust.source", [](CycleModelDesc& d) { d.exhaust->source += "x"; });
            excluded.emplace_back("installation.source", [](CycleModelDesc& d) { d.installation->source += "x"; });
            excluded.emplace_back("default_installation_path", [](CycleModelDesc& d) { d.default_installation_path += "x"; });
            excluded.emplace_back("installation_path", [](CycleModelDesc& d) { d.installation_path += "x"; });
            excluded.emplace_back("installation.intake.inlet_position_m",
                                   [](CycleModelDesc& d) { d.installation->intake.inlet_position_m.x += real(1.0); });
            excluded.emplace_back("installation exhaust outlet position_m", [](CycleModelDesc& d) {
                d.installation->exhaust.elements.back().position_m.z += real(1.0);
            });
            // ps_audio A5: bank geometry is not a generator input.
            excluded.emplace_back("banks", [](CycleModelDesc& d) { d.banks = {{1, 2, 3, 4}}; });
            excluded.emplace_back("bank_angle_deg", [](CycleModelDesc& d) { d.bank_angle_deg += real(60.0); });
            for (const auto& [name, edit] : excluded) {
                CycleModelDesc d = base;
                edit(d);
                INFO("excluded field: " << name);
                CHECK(engine_map_cache_key(d, ref_fuel, MapQuality::Preview) == base_key);
            }
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

    SECTION("garage P9: one key per cam profile - each the single-lift key of that profile's lifts, ids never enter") {
        const auto bump = [](real v) { return std::nextafter(v, real(1e300)); };
        // A profile-free desc keys exactly as before (the one engine_map_cache_key).
        const std::vector<std::uint64_t> plain = engine_map_profile_cache_keys(ref_desc, ref_fuel, MapQuality::Preview);
        REQUIRE(plain.size() == 1);
        CHECK(plain[0] == ref_key);

        // Profile 0 = the fixture's own lifts, profile 1 = the same with a raised intake curve.
        CycleModelDesc hi_single = ref_desc;
        for (real& y : hi_single.intake_lift.lift_m_vs_deg.y) y = y * real(1.25);
        const std::uint64_t hi_key = engine_map_cache_key(hi_single, ref_fuel, MapQuality::Preview);
        REQUIRE(hi_key != ref_key);
        CycleModelDesc base = ref_desc;
        base.cam_profiles.push_back({"low", ref_desc.intake_lift, ref_desc.exhaust_lift});
        base.cam_profiles.push_back({"high", hi_single.intake_lift, hi_single.exhaust_lift});
        const std::vector<std::uint64_t> keys = engine_map_profile_cache_keys(base, ref_fuel, MapQuality::Preview);
        REQUIRE(keys.size() == 2);
        CHECK(keys[0] == ref_key);
        CHECK(keys[1] == hi_key);
        // The single-set key refuses a profile desc (it would silently key profile 0 only).
        CHECK_THROWS(engine_map_cache_key(base, ref_fuel, MapQuality::Preview));
        // Quality moves every profile's key.
        const std::vector<std::uint64_t> full = engine_map_profile_cache_keys(base, ref_fuel, MapQuality::Full);
        CHECK(full[0] != keys[0]);
        CHECK(full[1] != keys[1]);

        // Every lift field of a profile moves that profile's key and no other.
        std::vector<std::pair<const char*, std::function<void(CycleModelDesc::CamProfile&)>>> edits;
        edits.emplace_back("intake_lift.phase_ref_deg", [&](CycleModelDesc::CamProfile& p) { p.intake_lift.phase_ref_deg = bump(p.intake_lift.phase_ref_deg); });
        edits.emplace_back("exhaust_lift.phase_ref_deg", [&](CycleModelDesc::CamProfile& p) { p.exhaust_lift.phase_ref_deg = bump(p.exhaust_lift.phase_ref_deg); });
        edits.emplace_back("intake_lift curve x", [&](CycleModelDesc::CamProfile& p) { p.intake_lift.lift_m_vs_deg.x.front() = bump(p.intake_lift.lift_m_vs_deg.x.front()); });
        edits.emplace_back("intake_lift curve y", [&](CycleModelDesc::CamProfile& p) { p.intake_lift.lift_m_vs_deg.y.front() = bump(p.intake_lift.lift_m_vs_deg.y.front()); });
        edits.emplace_back("exhaust_lift curve x", [&](CycleModelDesc::CamProfile& p) { p.exhaust_lift.lift_m_vs_deg.x.front() = bump(p.exhaust_lift.lift_m_vs_deg.x.front()); });
        edits.emplace_back("exhaust_lift curve y", [&](CycleModelDesc::CamProfile& p) { p.exhaust_lift.lift_m_vs_deg.y.front() = bump(p.exhaust_lift.lift_m_vs_deg.y.front()); });
        for (std::size_t k = 0; k < 2; ++k) {
            for (const auto& [name, edit] : edits) {
                CycleModelDesc d = base;
                edit(d.cam_profiles[k]);
                const std::vector<std::uint64_t> e = engine_map_profile_cache_keys(d, ref_fuel, MapQuality::Preview);
                INFO("profile " << k << " field " << name);
                CHECK(e[k] != keys[k]);
                CHECK(e[1 - k] == keys[1 - k]);
            }
        }

        // A non-lift desc field moves every profile's key (each per-profile desc carries it).
        {
            CycleModelDesc d = base;
            d.compression_ratio = bump(d.compression_ratio);
            const std::vector<std::uint64_t> e = engine_map_profile_cache_keys(d, ref_fuel, MapQuality::Preview);
            CHECK(e[0] != keys[0]);
            CHECK(e[1] != keys[1]);
        }
        // Renaming a profile moves no key (a label; the generator never reads it).
        {
            CycleModelDesc d = base;
            d.cam_profiles[0].id = "low_renamed";
            d.cam_profiles[1].id = "vtec";
            CHECK(engine_map_profile_cache_keys(d, ref_fuel, MapQuality::Preview) == keys);
        }
        // Swapping the profile order swaps the keys (the order is the cam order the ECU switches along).
        {
            CycleModelDesc d = base;
            std::swap(d.cam_profiles[0], d.cam_profiles[1]);
            const std::vector<std::uint64_t> e = engine_map_profile_cache_keys(d, ref_fuel, MapQuality::Preview);
            CHECK(e[0] == keys[1]);
            CHECK(e[1] == keys[0]);
        }
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
        CHECK(load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc)) == nullptr); // no real file yet - the stray temp is not one
        const auto entries_before = cache.entries();
        REQUIRE(entries_before.size() == 1); // just the stray temp

        store_cached_engine_maps(cache.path(), key, *maps);

        // The real file now exists and loads; the stray temp is untouched
        // debris (store never has to see or clean up someone else's temp
        // file - only its own, on a failed write, which this is not).
        const EngineMapsPtr loaded = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc));
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
        const EngineMapsPtr loaded = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc));
        REQUIRE(loaded != nullptr);
        CHECK(same_engine_maps(*maps, *loaded));
        const auto entries = cache.entries();
        REQUIRE(entries.size() == 1); // no leftover temp from either store
        CHECK(entries.front() == cache_file_name(key));
    }

    // C16b review (item 2d): concurrent writers of one key and a concurrent
    // reader. Every successful load is the whole file (never a torn one),
    // every writer's temp file is gone afterwards, and the final file loads.
    // A characterisation of the reader-side safety (rename + content-hash
    // recheck), not a guard for the temp NAME: on Windows it also passes
    // with one shared temp name (measured, 5 runs) - the per-process part
    // of the name (the pid) matters across processes, which an in-process
    // test cannot exercise.
    SECTION("concurrent writers of one key never expose a partial file and leave no temp behind") {
        constexpr int kWriters = 6;
        constexpr int kStoresEach = 8;
        std::atomic<int> writers_done{0};
        std::atomic<int> loads_ok{0};
        std::atomic<int> loads_bad{0}; // a non-null load whose maps differ
        std::vector<std::thread> threads;
        for (int t = 0; t < kWriters; ++t) {
            threads.emplace_back([&] {
                for (int i = 0; i < kStoresEach; ++i) store_cached_engine_maps(cache.path(), key, *maps);
                writers_done.fetch_add(1);
            });
        }
        threads.emplace_back([&] {
            while (writers_done.load() < kWriters) {
                const EngineMapsPtr l = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc));
                if (!l) continue;
                if (same_engine_maps(*maps, *l) && l->content_hash == maps->content_hash) {
                    loads_ok.fetch_add(1);
                } else {
                    loads_bad.fetch_add(1);
                }
            }
        });
        for (auto& th : threads) th.join();
        std::printf("2d concurrent: %d writers x %d stores, reader saw %d whole loads, %d bad\n", kWriters, kStoresEach,
                    loads_ok.load(), loads_bad.load());
        CHECK(loads_bad.load() == 0);
        const EngineMapsPtr final_load = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(desc));
        REQUIRE(final_load != nullptr);
        CHECK(same_engine_maps(*maps, *final_load));
        const auto entries = cache.entries();
        CHECK(entries.size() == 1);
        for (const auto& name : entries) CHECK_THAT(name, !ContainsSubstring(".tmp"));
    }
}

// C16b review task 4 (report only): a cache hit vs a fresh generation,
// single thread (no pool), release, the cycle_fixtures engine - both the
// 4x3 unit-test grid and the 12x20 grid of test_engine_map_generator.cpp's
// M8 report - Full and Preview, median of 5 each.
// Review item 2g (report only): the runtime's total pumping mean pressure
// (p_amb - p + pmep_excess, engine_simulated.cpp) at every node equals the
// cycle's own -cyl*W_pump/V_disp; printed with its sign, plus the clamped
// value above the grid's highest p_im.
TEST_CASE("C16b review: cache hit vs fresh generation timing, pumping sign", "[.][diag][drivetrain][cycle]") {
    CycleModelDesc big = fixture_desc();
    big.grid_rpm.clear();
    big.grid_p_im.clear();
    for (int i = 0; i < 12; ++i) big.grid_rpm.push_back(1000.0 + 500.0 * i);
    for (int i = 0; i < 20; ++i) big.grid_p_im.push_back(20000.0 + 5000.0 * i);
    const FuelDesc fuel = fx::gasoline();
    constexpr real kPamb = 101325.0; // SimulatedEngine::kIsaPressurePa, the generator's p_amb
    const auto median5 = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[2];
    };
    struct Grid {
        const char* name;
        CycleModelDesc desc;
    };
    for (const Grid& g : {Grid{"4x3  ", fixture_desc()}, Grid{"12x20", big}}) {
        for (MapQuality q : {MapQuality::Preview, MapQuality::Full}) {
            TempCacheDir cache;
            const std::uint64_t key = engine_map_cache_key(g.desc, fuel, q);
            std::vector<double> gen, hit;
            EngineMapsPtr fresh;
            for (int r = 0; r < 5; ++r) {
                const auto t0 = std::chrono::steady_clock::now();
                fresh = generate_engine_maps(g.desc, fuel, q, nullptr);
                gen.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            }
            store_cached_engine_maps(cache.path(), key, *fresh);
            EngineMapsPtr loaded;
            for (int r = 0; r < 5; ++r) {
                const auto t0 = std::chrono::steady_clock::now();
                loaded = load_cached_engine_maps(cache.path(), key, engine_map_generator_version(g.desc));
                hit.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            }
            REQUIRE(loaded);
            CHECK(loaded->content_hash == fresh->content_hash);
            const double mg = median5(gen), mh = median5(hit);
            std::printf("TIMING %s %s: generate %.4f s, cache hit %.6f s, ratio %.0fx, file %llu bytes\n", g.name,
                        q == MapQuality::Full ? "Full   " : "Preview", mg, mh, mg / mh,
                        static_cast<unsigned long long>(std::filesystem::file_size(cache.dir / cache.entries().front())));

            const Table2D& t = fresh->maps.pmep_excess_pa;
            real tmin = 1e300, tmax = -1e300;
            int negative = 0;
            real neg_p_min = 1e300;
            for (std::size_t ix = 0; ix < t.x.size(); ++ix) {
                for (std::size_t iy = 0; iy < t.y.size(); ++iy) {
                    const real total = kPamb - t.y[iy] + t.at(ix, iy);
                    tmin = std::min(tmin, total);
                    tmax = std::max(tmax, total);
                    if (total < 0) {
                        ++negative;
                        neg_p_min = std::min(neg_p_min, t.y[iy]);
                    }
                }
            }
            const real p_top = t.y.back();
            real above_min = 1e300;
            for (real rpm : t.x) {
                above_min = std::min(above_min, kPamb - 101325.0 + t.value(rpm, 101325.0));
            }
            std::printf("    PUMP total pmep at nodes [%.1f, %.1f] Pa, %d of %zu negative (lowest such p_im %.0f Pa); top p_im %.0f Pa; min total at p 101325 "
                        "(clamped excess) %.1f Pa\n",
                        tmin, tmax, negative, t.values.size(), negative ? neg_p_min : 0.0, p_top, above_min);
        }
    }
}

// Engine garage P2 (design A8, A5): the engine mass metadata and the part-slot paths are not generator inputs, so
// they never enter the .psmaps key. Measured through the loader (validate-only, instant): the same demo engine with
// mass_kg/mass_source added, with its whole ECU and cam moved into part files named by default_ecu/default_cam, and
// with its installation given as a slot by another spelling of the same path, all resolve to the same key.
// mass_kg/mass_source live on SimulatedEngineDesc, which the key never sees; the slot paths are not stored in the
// CycleModelDesc at all (installation_path is, and H3 above already excludes it). The last check is the
// discriminating one: the same part-file route with a cam that really differs moves the key.
TEST_CASE("garage P2 H3: mass_kg, mass_source and slot paths never enter the key", "[drivetrain][cycle][io]") {
    namespace fs = std::filesystem;
    using json = nlohmann::json;
    const fs::path engines = fs::path(std::string(PS_SOURCE_DIR)) / "data" / "engines";
    std::ifstream in(engines / "demo_i4_pipes.json");
    const json base = json::parse(in);
    io::EngineMapOptions o;
    o.generate = false;
    auto key = [&](const json& j, const io::EngineSlotOverrides& slots = {}) {
        const auto p = io::load_engine_json_from_string(j.dump(), engines, o, slots);
        const auto& e = std::get<SimulatedEngineDesc>(p);
        return engine_map_cache_key(*e.cycle, e.fuels.front(), MapQuality::Full);
    };
    const std::uint64_t ref = key(base);

    json m = base;
    m["mass_kg"] = 141.5;
    m["mass_source"] = "[AS] test";
    CHECK(key(m) == ref);

    // ECU and cam as part files in a scratch directory.
    const fs::path dir = fs::temp_directory_path() / ("ps_h3_parts_" + std::to_string(reinterpret_cast<std::uintptr_t>(&m)));
    fs::create_directories(dir);
    json s = base;
    json ecu = {{"format", "physics_sim.ecu/1"}, {"name", "h3"}};
    for (const char* k : {"idle_rpm", "stall_rpm", "fire_rpm", "start_complete_rpm", "dfco_enable_rpm", "dfco_reenable_rpm",
                          "idle_control", "limiter", "throttle_actuator_tau_s", "ecu"}) {
        if (s.contains(k)) {
            ecu[k] = s[k];
            s.erase(k);
        }
    }
    json cam = {{"format", "physics_sim.cam/1"}, {"name", "h3"}, {"intake_lift", s["cycle"]["intake_lift"]},
                {"exhaust_lift", s["cycle"]["exhaust_lift"]}};
    s["cycle"].erase("intake_lift");
    s["cycle"].erase("exhaust_lift");
    std::ofstream(dir / "ecu.json") << ecu.dump(2);
    std::ofstream(dir / "cam.json") << cam.dump(2);
    s["default_ecu"] = (dir / "ecu.json").generic_string();
    s["default_cam"] = (dir / "cam.json").generic_string();
    CHECK(key(s) == ref);

    // The installation slot, the same file spelled through a detour.
    const std::string detour = (engines / ".." / "engines" / ".." / "installations" / "demo_i4_exhaust.json").string();
    CHECK(key(base, {{"installation", detour}}) == ref);

    // Discriminating: a cam that really differs moves the key.
    cam["intake_lift"]["phase_ref_deg"] = cam["intake_lift"]["phase_ref_deg"].get<double>() + 1.0;
    std::ofstream(dir / "cam2.json") << cam.dump(2);
    s["default_cam"] = (dir / "cam2.json").generic_string();
    CHECK(key(s) != ref);
    std::error_code ec;
    fs::remove_all(dir, ec);
}
