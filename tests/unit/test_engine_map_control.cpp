// Engine garage P1 (A1-A3): control of the engine-map generation - cancel,
// progress, validate-only loads, MapQuality::Sketch, the public cache key and
// a background-priority JobSystem (out/engine_garage_design.md section 9.2).
//
// Test inputs: tests/support/cycle_fixtures.h's generic 2 L I4 (every value
// ASSUMED) and cycle_pipe_fixtures.h's pipe variant of it. Lumped engines are
// cheap (milliseconds per point); the one pipe Sketch case is [long].
//
// Every guard below has a recorded sabotage in out/garage_p1_progress.md.

#include "drivetrain/engine_simulated.h"

#include "cycle_fixtures.h"
#include "cycle_pipe_fixtures.h"
#include "mvem_fixtures.h"
#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/interfaces/catch_interfaces_registry_hub.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__linux__)
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;
using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
namespace fx = ps::test::mvem;
namespace da = ps::jobs::debug_alloc;

namespace {

constexpr real kF4A = 66619.521802247167; // the generator test's fixture friction (DERIVED there)
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

// The fixture on a 12 x 20 grid (Full 240 grid points; Preview 10 x 6; Sketch 8 x 3).
CycleModelDesc big_grid_desc() {
    CycleModelDesc d = fixture_desc();
    d.grid_rpm.clear();
    d.grid_p_im.clear();
    for (int i = 0; i < 12; ++i) d.grid_rpm.push_back(1000.0 + 500.0 * i);
    for (int i = 0; i < 20; ++i) d.grid_p_im.push_back(20000.0 + 5000.0 * i);
    return d;
}

bool same_bits(const std::vector<real>& a, const std::vector<real>& b) {
    return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(real)) == 0);
}
bool same_bits(const Table2D& a, const Table2D& b) {
    return same_bits(a.x, b.x) && same_bits(a.y, b.y) && same_bits(a.values, b.values);
}
bool same_bits(const vehicle::Curve1D& a, const vehicle::Curve1D& b) { return same_bits(a.x, b.x) && same_bits(a.y, b.y); }

bool same_maps(const SimulatedEngineMapsDesc& a, const SimulatedEngineMapsDesc& b) {
    return same_bits(a.ve, b.ve) && same_bits(a.eta_i, b.eta_i) && same_bits(a.f_lambda, b.f_lambda) &&
           same_bits(a.eta_spark, b.eta_spark) && same_bits(a.theta_mbt_deg, b.theta_mbt_deg) &&
           same_bits(a.friction_torque_nm, b.friction_torque_nm) && a.has_pmep_excess == b.has_pmep_excess &&
           same_bits(a.pmep_excess_pa, b.pmep_excess_pa) && a.has_theta_kl == b.has_theta_kl &&
           same_bits(a.theta_kl_deg, b.theta_kl_deg) && same_bits(a.wot_p_im_pa, b.wot_p_im_pa); // P7d: + WOT curve, knock
}

bool same_engine_maps(const EngineMaps& a, const EngineMaps& b) {
    return same_maps(a.maps, b.maps) && a.maps.source == b.maps.source && same_bits(a.t_exh_k, b.t_exh_k) &&
           same_bits(a.h_exh_w, b.h_exh_w) && a.generator_version == b.generator_version && a.quality == b.quality &&
           a.content_hash == b.content_hash;
}

// What the active exception says (ASan-safe: no reference to the caught object, TOOL-019).
template <class F>
std::string message_of(F&& f) {
    try {
        f();
    } catch (...) {
        return Catch::translateActiveException();
    }
    return "<no exception>";
}

struct TempDir {
    std::filesystem::path dir;
    TempDir() {
        dir = std::filesystem::temp_directory_path() /
              ("ps_test_engine_map_control_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
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
    std::filesystem::path cache() const { return dir / "cache"; }
};

std::size_t count_psmaps(const std::filesystem::path& dir) {
    std::size_t n = 0;
    std::error_code ec;
    if (!std::filesystem::exists(dir, ec)) return 0;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        if (e.path().extension() == ".psmaps") ++n;
    }
    return n;
}

std::filesystem::path key_file(const std::filesystem::path& dir, std::uint64_t key) {
    char name[40];
    std::snprintf(name, sizeof(name), "%016llx.psmaps", static_cast<unsigned long long>(key));
    return dir / name;
}

std::string gasoline_path() {
    return std::filesystem::path(std::string(PS_SOURCE_DIR) + "/data/fuels/gasoline.json").generic_string();
}

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

// An 86/86 mm I4 simulated engine whose maps come from its cycle block only
// (test_engine_map_generator.cpp's cycle_only_engine()).
json cycle_only_engine(const CycleModelDesc& d = fixture_desc()) {
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
    j["inertia_kgm2"] = 0.15;
    j["breakaway_torque_nm"] = 40.0;
    j["fuel"] = gasoline_path();
    j["ignition"] = "spark";
    j["cycle"] = cycle_json(d);
    return j;
}

json vehicle_with_engine(const json& engine) {
    json v;
    v["format"] = "physics_sim.vehicle/2";
    v["source"] = "test fixture, not a real vehicle (test_engine_map_control.cpp)";
    v["wheels"] = json::array({{{"name", "FL"}, {"tyre", {{"source", "test fixture"}}}}});
    v["powertrain"] = {{"components", json::array({engine})}};
    return v;
}

vehicle::VehicleDesc load(const TempDir& td, const json& engine, const io::EngineMapOptions& o) {
    const auto p = td.write("vehicle.json", vehicle_with_engine(engine));
    return io::load_vehicle_json(p.string(), o);
}

const SimulatedEngineDesc& only_sim_engine(const vehicle::VehicleDesc& d) {
    REQUIRE(d.powertrain.components.size() == 1);
    const auto* e = std::get_if<SimulatedEngineDesc>(&d.powertrain.components[0].params);
    REQUIRE(e != nullptr);
    return *e;
}

SimulatedEngineDesc engine_on(const EngineMaps& em) {
    SimulatedEngineDesc e = fx::base_engine();
    e.maps = em.maps;
    e.maps_pending = false;
    e.ecu.lambda_target = Table2D{};
    e.ecu.lambda_target.x = {0.0, 9000.0};
    e.ecu.lambda_target.y = {0.0, 150000.0};
    e.ecu.lambda_target.values.assign(4, 1.0);
    e.ecu.spark_advance_deg = e.ecu.lambda_target;
    e.ecu.spark_advance_deg.values.assign(4, em.maps.theta_mbt_deg.values.front());
    e.breakaway_torque_nm = em.maps.friction_torque_nm.y.front() + 5.0;
    return e;
}

struct Recorded {
    std::vector<std::pair<std::size_t, std::size_t>> calls;
    std::vector<std::thread::id> threads;
    MapGenerationControl control() {
        MapGenerationControl c;
        c.on_progress = [this](std::size_t done, std::size_t total) {
            calls.emplace_back(done, total);
            threads.push_back(std::this_thread::get_id());
        };
        return c;
    }
};

std::size_t points_of(const CycleModelDesc& d, MapQuality q) {
    EngineMapGenTask t(d, fx::gasoline(), q);
    return t.point_count();
}

} // namespace

// ---------------------------------------------------------------------------
// 1. Cancellation.
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 T1: cancel stops generation, throws GenerationCancelled and writes nothing", "[drivetrain][cycle][io]") {
    const CycleModelDesc d = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    const std::size_t total = points_of(d, MapQuality::Preview);
    REQUIRE(total > 10);

    SECTION("a flag already set on entry: no point runs") {
        std::atomic<bool> cancel{true};
        Recorded rec;
        MapGenerationControl c = rec.control();
        c.cancel = &cancel;
        CHECK_THROWS_AS(generate_engine_maps(d, fuel, MapQuality::Preview, nullptr, &c), GenerationCancelled);
        CHECK(rec.calls.empty());
    }
    SECTION("raised from the progress callback, calling thread: exactly the points before the flag ran") {
        std::atomic<bool> cancel{false};
        std::size_t calls = 0;
        MapGenerationControl c;
        c.cancel = &cancel;
        c.on_progress = [&](std::size_t done, std::size_t) {
            ++calls;
            if (done == 5) cancel.store(true);
        };
        CHECK_THROWS_AS(generate_engine_maps(d, fuel, MapQuality::Preview, nullptr, &c), GenerationCancelled);
        CHECK(calls == 5); // the sixth point saw the flag before it started
    }
    SECTION("raised on the pool: stops early, latency bounded by the worker count") {
        const CycleModelDesc big = big_grid_desc();
        const std::size_t big_total = points_of(big, MapQuality::Full); // 279 points, ~10 ms each
        jobs::JobSystem pool(2);
        std::atomic<bool> cancel{false};
        std::size_t calls = 0;
        MapGenerationControl c;
        c.cancel = &cancel;
        c.on_progress = [&](std::size_t done, std::size_t) {
            ++calls;
            if (done == 2) cancel.store(true);
        };
        CHECK_THROWS_AS(generate_engine_maps(big, fuel, MapQuality::Full, &pool, &c), GenerationCancelled);
        // 2 points finished at the raise, at most 2 were running (2 workers), a poll delivers a few more.
        CHECK(calls >= 2);
        CHECK(calls <= 12);
        CHECK(calls < big_total / 4);
    }
    SECTION("raised only after the last point started: the run completes normally") {
        std::atomic<bool> cancel{false};
        MapGenerationControl c;
        c.cancel = &cancel;
        c.on_progress = [&](std::size_t done, std::size_t tot) {
            if (done == tot) cancel.store(true);
        };
        EngineMapsPtr m;
        REQUIRE_NOTHROW(m = generate_engine_maps(d, fuel, MapQuality::Preview, nullptr, &c));
        CHECK(same_engine_maps(*m, *generate_engine_maps(d, fuel, MapQuality::Preview)));
    }
    SECTION("loader: a cancelled load writes no cache file, an uncancelled one does") {
        TempDir td;
        std::atomic<bool> cancel{false};
        std::size_t generations = 0;
        std::size_t calls = 0;
        io::EngineMapOptions o;
        o.quality = MapQuality::Preview;
        o.cache_dir = td.cache().string();
        o.cancel = &cancel;
        o.generation_count_out = &generations;
        o.on_progress = [&](std::size_t done, std::size_t) {
            ++calls;
            if (done == 3) cancel.store(true);
        };
        CHECK_THROWS_AS(load(td, cycle_only_engine(), o), io::GenerationCancelled);
        CHECK_THROWS_AS(load(td, cycle_only_engine(), o), std::runtime_error); // GenerationCancelled is a runtime_error
        CHECK(count_psmaps(td.cache()) == 0);

        cancel.store(false);
        o.on_progress = nullptr;
        generations = 0;
        const vehicle::VehicleDesc v = load(td, cycle_only_engine(), o);
        CHECK(generations == 1);
        CHECK(count_psmaps(td.cache()) == 1);
        CHECK_FALSE(only_sim_engine(v).maps_pending);

        // documented: the flag is read before the cache lookup, so a set flag throws even for a hit
        cancel.store(true);
        CHECK_THROWS_AS(load(td, cycle_only_engine(), o), io::GenerationCancelled);
        // a load that generates nothing never reads it (complete override -> no cycle generation)
        cancel.store(false);
    }
    SECTION("loader: a flag never set changes nothing") {
        TempDir td;
        std::atomic<bool> never{false};
        io::EngineMapOptions a, b;
        a.quality = b.quality = MapQuality::Preview;
        b.cancel = &never;
        const vehicle::VehicleDesc va = load(td, cycle_only_engine(), a);
        const vehicle::VehicleDesc vb = load(td, cycle_only_engine(), b);
        CHECK(same_maps(only_sim_engine(va).maps, only_sim_engine(vb).maps));
    }
}

// ---------------------------------------------------------------------------
// 2. Progress.
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 T2: progress is once per point, rising by 1, on the calling thread, at 1 and 10 workers",
          "[drivetrain][cycle][io]") {
    const CycleModelDesc d = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    for (MapQuality q : {MapQuality::Preview, MapQuality::Sketch, MapQuality::Full}) {
        const std::size_t total = points_of(d, q);
        for (int workers : {0, 1, 10}) { // 0 = no pool, calling thread
            INFO("quality " << static_cast<int>(q) << " workers " << workers);
            std::unique_ptr<jobs::JobSystem> pool;
            if (workers > 0) pool = std::make_unique<jobs::JobSystem>(workers);
            Recorded rec;
            MapGenerationControl c = rec.control();
            const EngineMapsPtr m = generate_engine_maps(d, fuel, q, pool.get(), &c);
            REQUIRE(rec.calls.size() == total);
            for (std::size_t i = 0; i < total; ++i) {
                CHECK(rec.calls[i].first == i + 1);
                CHECK(rec.calls[i].second == total);
                CHECK(rec.threads[i] == std::this_thread::get_id());
            }
            // the maps do not depend on being observed
            const EngineMapsPtr plain = generate_engine_maps(d, fuel, q);
            CHECK(same_engine_maps(*m, *plain));
        }
    }
    SECTION("loader: one run per generating engine, nothing for a cache hit") {
        TempDir td;
        std::size_t calls = 0, last_done = 0, last_total = 0;
        io::EngineMapOptions o;
        o.quality = MapQuality::Preview;
        o.cache_dir = td.cache().string();
        o.on_progress = [&](std::size_t done, std::size_t tot) {
            ++calls;
            last_done = done;
            last_total = tot;
        };
        (void)load(td, cycle_only_engine(), o);
        CHECK(calls == points_of(d, MapQuality::Preview));
        CHECK(last_done == last_total);
        calls = 0;
        (void)load(td, cycle_only_engine(), o); // cache hit
        CHECK(calls == 0);
    }
}

// ---------------------------------------------------------------------------
// 3. Validate-only loads.
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 T3: generate = false rejects exactly what a full load rejects and generates nothing",
          "[drivetrain][cycle][io]") {
    SECTION("table: the same message as a full load") {
        const std::vector<std::pair<std::function<void(json&)>, std::string>> cases = {
            {[](json& c) { c["bore_mm"] = 86.0; }, "'bore_mm' is not allowed here"},
            {[](json& c) { c["runner_volume_cc"] = 1.0; }, "unknown key 'runner_volume_cc'"},
            {[](json& c) { c.erase("source"); }, "'source' is required"},
            {[](json& c) { c["rod_mm"] = 40.0; }, "'rod_mm' must exceed stroke_mm/2 + |pin_offset_mm|"},
            {[](json& c) { c["compression_ratio"] = 1.0; }, "'compression_ratio' must be > 1"},
            {[](json& c) { c.erase("cd_vs_l_over_d"); }, "'cd_vs_l_over_d' is required"},
            {[](json& c) { c["wiebe_m"] = -1.0; }, "'wiebe_m' must be >= 0"},
            {[](json& c) { c["friction"]["model"] = "barnes_moss"; },
             "'model' must be \"polynomial_rpm\" or \"polynomial_piston_speed\""},
            {[](json& c) { c.erase("grid_rpm"); }, "'grid_rpm' array is required"},
            {[](json& c) { c["grid_p_im_kpa"] = {0.0, 50.0}; }, "'grid_p_im_kpa' values must be > 0 and finite"},
            {[](json& c) { c["grid_rpm"] = {2000.0, 1000.0}; }, "'grid_rpm' must be strictly ascending"},
            {[](json& c) { c["intake_lift"]["lift_mm_vs_deg"]["y"][0] = 0.5; }, "the first and last lift must be exactly 0"},
            {[](json& c) { c["firing_order"] = {1, 3, 4, 3}; }, "cylinder 3 appears more than once"},
            {[](json& c) { c["firing_angles_deg"] = {0.0, 180.0, 180.0, 540.0}; }, "share the same firing angle"},
            {[](json& c) { c["banks"] = {{1, 2, 3}}; }, "'banks' must cover every cylinder exactly once (covered 3 of 4)"},
            // raised by the generator's own build-time check (validate_engine_map_generation), not by the file reader
            {[](json& c) { c["friction"]["c_kpa"] = -50.0; }, "the friction correlation is negative at"},
        };
        REQUIRE(cases.size() >= 10);
        for (const auto& [mutate, msg] : cases) {
            json j = cycle_only_engine();
            mutate(j["cycle"]);
            INFO(msg);
            TempDir td;
            io::EngineMapOptions full, dry;
            full.quality = dry.quality = MapQuality::Preview;
            dry.generate = false;
            std::size_t n_dry = 0;
            dry.generation_count_out = &n_dry;
            const std::string m_full = message_of([&] { (void)load(td, j, full); });
            const std::string m_dry = message_of([&] { (void)load(td, j, dry); });
            CHECK_THAT(m_full, ContainsSubstring(msg));
            CHECK(m_dry == m_full);
            CHECK(n_dry == 0);
        }
    }
    SECTION("desc-level generator checks: validate_engine_map_generation throws what generate_engine_maps throws") {
        const FuelDesc fuel = fx::gasoline();
        struct Case {
            const char* name;
            std::function<CycleModelDesc()> make;
        };
        const std::vector<Case> cases = {
            {"grid_rpm descending", [] { CycleModelDesc d = fixture_desc(); d.grid_rpm = {4000.0, 1000.0}; return d; }},
            {"grid_p_im missing", [] { CycleModelDesc d = fixture_desc(); d.grid_p_im.clear(); return d; }},
            {"pipe engine without merges",
             [] { CycleModelDesc d = ps::test::make_pipe_i4_cycle_desc(); d.exhaust->merges.clear(); return d; }},
            {"pipe engine, last merge not into the interface",
             [] { CycleModelDesc d = ps::test::make_pipe_i4_cycle_desc(); d.exhaust->interface = "elsewhere"; return d; }},
            {"pipe engine, negative plenum volume",
             [] { CycleModelDesc d = ps::test::make_pipe_i4_cycle_desc(); d.intake->plenum_m3 = -1.0; return d; }},
        };
        for (const Case& c : cases) {
            for (MapQuality q : {MapQuality::Preview, MapQuality::Sketch}) {
                INFO(c.name << " quality " << static_cast<int>(q));
                const CycleModelDesc d = c.make();
                const std::string m_gen = message_of([&] { (void)generate_engine_maps(d, fuel, q); });
                const std::string m_val = message_of([&] { validate_engine_map_generation(d, fuel, q); });
                CHECK(m_gen != "<no exception>");
                CHECK(m_val == m_gen);
            }
        }
        // a valid desc validates quietly
        for (CycleModelDesc d : {fixture_desc(), ps::test::make_pipe_i4_cycle_desc()}) {
            CHECK_NOTHROW(validate_engine_map_generation(d, fuel, MapQuality::Full));
        }
    }
    SECTION("a valid engine: instant, nothing generated, nothing written, pending") {
        TempDir td;
        std::size_t n = 0;
        io::EngineMapOptions dry;
        dry.quality = MapQuality::Full;
        dry.generate = false;
        dry.cache_dir = td.cache().string();
        dry.generation_count_out = &n;
        bool progressed = false;
        dry.on_progress = [&](std::size_t, std::size_t) { progressed = true; };
        const vehicle::VehicleDesc v = load(td, cycle_only_engine(), dry);
        CHECK(n == 0);
        CHECK_FALSE(progressed);
        CHECK(count_psmaps(td.cache()) == 0);
        CHECK_FALSE(std::filesystem::exists(td.cache())); // not even the directory
        const SimulatedEngineDesc& e = only_sim_engine(v);
        CHECK(e.maps_pending);
        REQUIRE(e.cycle.has_value());
        CHECK(e.maps.ve.values.empty());

        // a pending engine cannot reach a SimulatedEngine, a Powertrain or a World
        CHECK_THROWS_WITH(SimulatedEngine(e), ContainsSubstring("pending"));
        vehicle::VehicleDesc car = ps_test::load_sedan_desc();
        bool swapped = false;
        for (auto& c : car.powertrain.components) {
            if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&c.params)) {
                SimulatedEngineDesc s = e;
                s.crank_axis_local = tm->crank_axis_local;
                c.params = s;
                swapped = true;
                break;
            }
        }
        REQUIRE(swapped);
        WorldConfig cfg;
        cfg.job_workers = 1;
        World w(cfg);
        BodyDesc chassis;
        chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
        chassis.motion = BodyMotionType::Dynamic;
        chassis.mass = 1500.0;
        const BodyId ch = w.create_body(chassis);
        CHECK_THROWS_WITH(w.create_vehicle(car, ch), ContainsSubstring("pending"));

        // the same file loaded for real is not pending
        io::EngineMapOptions full = dry;
        full.generate = true;
        full.quality = MapQuality::Preview;
        const vehicle::VehicleDesc vf = load(td, cycle_only_engine(), full);
        CHECK_FALSE(only_sim_engine(vf).maps_pending);
        CHECK(n == 1);
    }
    SECTION("an engine that needs no generation is untouched by generate = false") {
        // a complete maps override: same desc either way
        TempDir td;
        json j = cycle_only_engine();
        j["maps"] = {
            {"source", "synthetic test fixture, not physical data"},
            {"ve", {{"rpm", {0.0, 3000.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{0.80, 0.85}, {0.88, 0.95}, {0.85, 0.90}}}}},
            {"eta_i", {{"rpm", {0.0, 7000.0}}, {"air_mass_per_rev_g", {0.0, 1.5}}, {"values", {{0.34, 0.37}, {0.36, 0.38}}}}},
            {"f_lambda", {{"x", {0.7, 1.0, 1.5}}, {"y", {0.85, 1.0, 0.9}}}},
            {"eta_spark", {{"x", {-40.0, 0.0, 10.0}}, {"y", {0.5, 1.0, 0.95}}}},
            {"theta_mbt_deg", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{30.0, 20.0}, {40.0, 30.0}}}}},
            {"friction_torque_nm", {{"x", {0.0, 1000.0, 7000.0}}, {"y", {15.0, 18.0, 40.0}}}},
            {"pmep_excess_kpa", {{"rpm", {0.0, 7000.0}}, {"p_im_kpa", {20.0, 100.0}}, {"values", {{2.0, 1.0}, {6.0, 4.0}}}}}};
        io::EngineMapOptions a, b;
        b.generate = false;
        const vehicle::VehicleDesc va = load(td, j, a);
        const vehicle::VehicleDesc vb = load(td, j, b);
        CHECK_FALSE(only_sim_engine(vb).maps_pending);
        CHECK(same_maps(only_sim_engine(va).maps, only_sim_engine(vb).maps));
    }
}

// ---------------------------------------------------------------------------
// 4. MapQuality::Sketch.
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 T4: Sketch is a complete, deterministic, separately keyed map set on a reduced grid",
          "[drivetrain][cycle][io]") {
    const FuelDesc fuel = fx::gasoline();
    SECTION("numbering: Preview and Full keep their values") {
        CHECK(static_cast<int>(MapQuality::Preview) == 0);
        CHECK(static_cast<int>(MapQuality::Full) == 1);
        CHECK(static_cast<int>(MapQuality::Sketch) == 2);
    }
    SECTION("point set and grid") {
        const CycleModelDesc big = big_grid_desc();
        CHECK(points_of(big, MapQuality::Full) == 12 * 20 + 30 + 9);
        CHECK(points_of(big, MapQuality::Preview) == 10 * 6 + 30 + 9);
        // 4 p_im rows since burn law phase 3 (2026-10-20; 8 * 3 + 6 + 5 = 35 points before).
        CHECK(points_of(big, MapQuality::Sketch) == 8 * 4 + 6 + 5);
        const EngineMapsPtr m = generate_engine_maps(big, fuel, MapQuality::Sketch);
        CHECK(m->quality == MapQuality::Sketch);
        CHECK(m->maps.ve.x.size() <= 8);
        CHECK(m->maps.ve.y.size() == 4);
        CHECK(m->maps.ve.x.size() == 8);
        CHECK(m->maps.ve.x.front() == big.grid_rpm.front());
        CHECK(m->maps.ve.x.back() == big.grid_rpm.back());
        // the rows: the lowest node, the part-load node halfway by index to the node nearest ambient, that node, the top
        // node (grid 20..115 kPa in 5 kPa steps: 20, 60, 100, 115 kPa)
        CHECK(m->maps.ve.y.front() == big.grid_p_im.front());
        CHECK(m->maps.ve.y.back() == big.grid_p_im.back());
        CHECK(m->maps.ve.y[1] == big.grid_p_im[8]);
        CHECK(std::abs(m->maps.ve.y[2] - 101325.0) <= 2500.0 + 1e-9);
        CHECK_THAT(m->maps.source, ContainsSubstring("Sketch"));
    }
    SECTION("rows on a grid whose top node is the one nearest ambient (toyota_a25a_fks_gen's p_im axis)") {
        // Burn law phase 3 (2026-10-20): the near-ambient row is the highest node at or below ambient (97.273 kPa), not
        // the node nearest it (the 105 kPa top, which made version 1 fall back to the middle node, 58.636 kPa, and left
        // 58.6 -> 105 kPa as one span). Recorded sabotage: the version 1 'nearest ambient' choice -> rows 20000 35455
        // 58636 105000, both row checks fail.
        CycleModelDesc d = fixture_desc();
        d.grid_p_im = {20000.0, 27727.0, 35455.0, 43182.0, 50909.0, 58636.0, 66364.0, 74091.0, 81818.0, 89545.0, 97273.0, 105000.0};
        const EngineMapsPtr m = generate_engine_maps(d, fuel, MapQuality::Sketch);
        REQUIRE(m->maps.ve.y.size() == 4);
        CHECK(m->maps.ve.y[0] == 20000.0);
        CHECK(m->maps.ve.y[1] == 58636.0);
        CHECK(m->maps.ve.y[2] == 97273.0);
        CHECK(m->maps.ve.y[3] == 105000.0);
    }
    SECTION("complete: every table filled and finite, loads into a SimulatedEngine") {
        const CycleModelDesc d = fixture_desc();
        const EngineMapsPtr m = generate_engine_maps(d, fuel, MapQuality::Sketch);
        const SimulatedEngineMapsDesc& s = m->maps;
        for (const Table2D* t : {&s.ve, &s.eta_i, &s.theta_mbt_deg, &s.pmep_excess_pa}) {
            REQUIRE_FALSE(t->values.empty());
            for (real v : t->values) CHECK(std::isfinite(v));
        }
        REQUIRE_FALSE(s.f_lambda.x.empty());
        REQUIRE_FALSE(s.eta_spark.x.empty());
        REQUIRE_FALSE(s.friction_torque_nm.x.empty());
        CHECK(s.has_pmep_excess);
        CHECK(s.f_lambda.x.size() == 5);
        for (real v : s.f_lambda.y) CHECK((std::isfinite(v) && v > 0.0));
        for (real v : s.eta_spark.y) CHECK((std::isfinite(v) && v > 0.0));
        CHECK_NOTHROW(SimulatedEngine(engine_on(*m)));
    }
    SECTION("1 vs 10 workers: hash-identical") {
        for (const CycleModelDesc& d : {fixture_desc(), big_grid_desc()}) {
            jobs::JobSystem ten(10);
            const EngineMapsPtr a = generate_engine_maps(d, fuel, MapQuality::Sketch);
            const EngineMapsPtr b = generate_engine_maps(d, fuel, MapQuality::Sketch, &ten);
            CHECK(a->content_hash == b->content_hash);
            CHECK(same_engine_maps(*a, *b));
        }
    }
    SECTION("the key and the content differ from Preview and Full") {
        const CycleModelDesc d = fixture_desc();
        const std::uint64_t ks = engine_map_cache_key(d, fuel, MapQuality::Sketch);
        const std::uint64_t kp = engine_map_cache_key(d, fuel, MapQuality::Preview);
        const std::uint64_t kf = engine_map_cache_key(d, fuel, MapQuality::Full);
        CHECK(ks != kp);
        CHECK(ks != kf);
        CHECK(kp != kf);
        const EngineMapsPtr s = generate_engine_maps(d, fuel, MapQuality::Sketch);
        const EngineMapsPtr p = generate_engine_maps(d, fuel, MapQuality::Preview);
        CHECK(s->content_hash != p->content_hash);
    }
    SECTION("cache: a hit is bit-identical, a bad quality byte is a miss") {
        TempDir td;
        std::size_t n = 0;
        io::EngineMapOptions o;
        o.quality = MapQuality::Sketch;
        o.cache_dir = td.cache().string();
        o.generation_count_out = &n;
        const vehicle::VehicleDesc v1 = load(td, cycle_only_engine(), o);
        CHECK(n == 1);
        CHECK(count_psmaps(td.cache()) == 1);
        const SimulatedEngineDesc& e1 = only_sim_engine(v1);
        // A2: the public key names the file the load wrote
        const std::uint64_t key = engine_map_cache_key(*e1.cycle, e1.fuels.front(), MapQuality::Sketch);
        const auto file = key_file(td.cache(), key);
        REQUIRE(std::filesystem::exists(file));
        const vehicle::VehicleDesc v2 = load(td, cycle_only_engine(), o);
        CHECK(n == 1); // hit
        CHECK(same_maps(e1.maps, only_sim_engine(v2).maps));
        CHECK(e1.maps.source == only_sim_engine(v2).maps.source);

        // header: magic[8] + version u32 + key u64 + content_hash u64 + quality u32 (offset 28)
        std::vector<char> bytes;
        {
            std::ifstream in(file, std::ios::binary);
            bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        }
        REQUIRE(bytes.size() > 40);
        const std::uint32_t bad = 7;
        std::memcpy(bytes.data() + 28, &bad, sizeof(bad));
        {
            std::ofstream out(file, std::ios::binary | std::ios::trunc);
            out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        }
        const vehicle::VehicleDesc v3 = load(td, cycle_only_engine(), o);
        CHECK(n == 2); // regenerated
        CHECK(same_maps(e1.maps, only_sim_engine(v3).maps));
    }
}

TEST_CASE("Garage P1 T4b: Sketch of a pipe engine is complete and worker-count invariant", "[drivetrain][cycle][io][pipe]") {
    const CycleModelDesc d = ps::test::make_pipe_i4_cycle_desc();
    const FuelDesc fuel = fx::gasoline();
    jobs::JobSystem ten(10);
    const EngineMapsPtr a = generate_engine_maps(d, fuel, MapQuality::Sketch, &ten);
    const EngineMapsPtr b = generate_engine_maps(d, fuel, MapQuality::Sketch);
    CHECK(a->quality == MapQuality::Sketch);
    // engine garage P7d: ambient-fed pipe engine. 2026-10-19 (burn law F, vault PHYS-079): the fixture runs the
    // standard burn law, so it generates as the law's pipe-WOT version 7 (was kPipeWotGeneratorVersion 4).
    CHECK(a->generator_version == kLawPipeWotGeneratorVersion);
    CHECK(same_engine_maps(*a, *b));
    // (Loading pipe-engine maps into a SimulatedEngine is covered through the loader by the A5/A6 tests; the
    // generic mvem_fixtures base engine used by engine_on() is built for the lumped fixture's grid.)
    for (const real v : a->maps.ve.values) CHECK((std::isfinite(v) && v > 0.0));
    CHECK(a->maps.ve.x.size() == 4);
    // Engine garage P7d: the Sketch rows (lowest, nearest ambient, top) below every WOT plenum pressure, plus the WOT
    // row at 101325 Pa (was 3 author rows).
    std::printf("T4b Sketch p rows:");
    for (const real y : a->maps.ve.y) std::printf(" %.0f", y);
    std::printf(" (dropped %d)\n", a->wot_dropped_rows);
    CHECK(a->maps.ve.y.size() == 4); // 30, 60, 90 kPa + WOT (dropped 0)
    CHECK(a->maps.ve.y.back() == 101325.0);
    CHECK(a->maps.wot_p_im_pa.x == a->maps.ve.x);
    CHECK(engine_map_cache_key(d, fuel, MapQuality::Sketch) != engine_map_cache_key(d, fuel, MapQuality::Preview));
}

// ---------------------------------------------------------------------------
// 5. Observation does not change results (Full).
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 T5: Full with progress and a never-set cancel flag is hash-identical to Full without", "[drivetrain][cycle][io]") {
    const CycleModelDesc d = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    jobs::JobSystem four(4);
    const EngineMapsPtr plain = generate_engine_maps(d, fuel, MapQuality::Full, &four);
    std::atomic<bool> never{false};
    Recorded rec;
    MapGenerationControl c = rec.control();
    c.cancel = &never;
    const EngineMapsPtr observed = generate_engine_maps(d, fuel, MapQuality::Full, &four, &c);
    CHECK(plain->content_hash == observed->content_hash);
    CHECK(same_engine_maps(*plain, *observed));
    CHECK(rec.calls.size() == points_of(d, MapQuality::Full));
    // and the single-thread run with both
    Recorded rec1;
    MapGenerationControl c1 = rec1.control();
    c1.cancel = &never;
    CHECK(same_engine_maps(*plain, *generate_engine_maps(d, fuel, MapQuality::Full, nullptr, &c1)));
}

// ---------------------------------------------------------------------------
// 6. Zero allocation inside the jobs.
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 T6: progress and cancel add no allocation inside the generation jobs", "[drivetrain][cycle]") {
    if (!da::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const CycleModelDesc d = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    jobs::JobSystem pool(4);
    auto run = [&](const MapGenerationControl* control) {
        da::reset();
        da::set_armed(true);
        const EngineMapsPtr m = generate_engine_maps(d, fuel, MapQuality::Preview, &pool, control);
        da::set_armed(false);
        return std::make_pair(da::own_violation_count(), m);
    };
    const auto [plain_violations, plain_maps] = run(nullptr);
    std::atomic<bool> never{false};
    std::vector<std::size_t> seen; // the callback runs on this thread and may allocate; a job must not
    MapGenerationControl c;
    c.cancel = &never;
    c.on_progress = [&](std::size_t done, std::size_t) { seen.push_back(done); };
    const auto [observed_violations, observed_maps] = run(&c);
    std::printf("T6 own violations: plain %llu, with progress+cancel %llu\n", static_cast<unsigned long long>(plain_violations),
                static_cast<unsigned long long>(observed_violations));
    CHECK(observed_violations == plain_violations);
    CHECK(observed_violations == 0);
    CHECK(seen.size() == points_of(d, MapQuality::Preview));
    CHECK(same_engine_maps(*plain_maps, *observed_maps));
}

// ---------------------------------------------------------------------------
// 7. Background priority.
// ---------------------------------------------------------------------------
namespace {
// The OS priority of the thread running a job of `pool`; INT_MIN if it cannot be read.
int worker_priority(jobs::JobSystem& pool) {
    std::atomic<int> result{-12345};
    jobs::JobSystem::WaitGroup g;
    pool.submit(jobs::JobFn([&result]() {
#if defined(_WIN32)
                    result.store(GetThreadPriority(GetCurrentThread()));
#elif defined(__linux__)
                    result.store(getpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid))));
#endif
                }),
                &g);
    pool.wait(g);
    return result.load();
}
} // namespace

TEST_CASE("Garage P1 T7: a background-priority pool lowers its threads and gives the same maps", "[drivetrain][cycle]") {
    const CycleModelDesc d = fixture_desc();
    const FuelDesc fuel = fx::gasoline();
    jobs::JobSystemOptions opt;
    opt.background_priority = true;
    jobs::JobSystem low(4, opt);
    jobs::JobSystem normal(4);
    const EngineMapsPtr a = generate_engine_maps(d, fuel, MapQuality::Full, &low);
    const EngineMapsPtr b = generate_engine_maps(d, fuel, MapQuality::Full, &normal);
    CHECK(same_engine_maps(*a, *b));
    CHECK(a->content_hash == b->content_hash);
#if defined(_WIN32)
    CHECK(worker_priority(low) == THREAD_PRIORITY_BELOW_NORMAL);
    CHECK(worker_priority(normal) == THREAD_PRIORITY_NORMAL);
#elif defined(__linux__)
    CHECK(worker_priority(low) > worker_priority(normal));
#endif
    // the old constructor shape stays source-compatible
    jobs::JobSystem legacy(2, std::size_t{64});
    CHECK(legacy.worker_count() == 2);
}

// ---------------------------------------------------------------------------
// Timing table (engine garage P1 measurement): Sketch and Full, cold (no cache), on 1/10/22 workers.
// Run by hand: ps_unit_tests "Garage P1 timing*" (hidden, [.][diag]).
// ---------------------------------------------------------------------------
TEST_CASE("Garage P1 timing: Sketch and Full map generation on 1/10/22 workers", "[.][diag][drivetrain][cycle]") {
    struct Engine {
        std::string name;
        CycleModelDesc desc;
        FuelDesc fuel;
    };
    std::vector<Engine> engines;
    auto from_vehicle = [&](const char* name, const char* file) {
        io::EngineMapOptions o;
        o.generate = false; // validate-only: instant, gives the resolved desc
        const vehicle::VehicleDesc v = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/" + file, o);
        for (const auto& c : v.powertrain.components) {
            if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) {
                if (e->cycle.has_value()) engines.push_back({name, *e->cycle, e->fuels.front()});
            }
        }
    };
    from_vehicle("hyper_v8_one1_audio", "data/vehicles/car_hyper.json");
    from_vehicle("demo_i4_pipes", "tests/support/fixtures/vehicles/car_sedan_pipes.json");
    from_vehicle("toyota_a25a_fks_gen", "tests/support/fixtures/vehicles/car_sedan_gen.json");
    // boosted demo_i4_pipes: a test-local copy with a compressor and two grid rows above ambient
    for (const Engine& e : std::vector<Engine>(engines)) {
        if (e.name != "demo_i4_pipes") continue;
        Engine b = e;
        b.name = "demo_i4_pipes boosted (test-local)";
        b.desc.intake->compressor_supply = true;
        b.desc.grid_p_im.push_back(130000.0);
        b.desc.grid_p_im.push_back(160000.0);
        engines.push_back(b);
    }
    REQUIRE(engines.size() == 4);
    const bool full_one = true;
    std::printf("TIMING hardware threads %u\n", std::thread::hardware_concurrency());
    for (const Engine& e : engines) {
        for (MapQuality q : {MapQuality::Sketch, MapQuality::Full}) {
            const std::size_t pts = points_of(e.desc, q);
            for (int workers : {1, 10, 22}) {
                if (q == MapQuality::Full && workers == 1 && !full_one) continue;
                jobs::JobSystem pool(workers);
                const auto t0 = std::chrono::steady_clock::now();
                std::string status = "ok";
                std::uint64_t hash = 0;
                try {
                    const EngineMapsPtr m = generate_engine_maps(e.desc, e.fuel, q, &pool);
                    hash = m->content_hash;
                } catch (...) {
                    status = Catch::translateActiveException();
                }
                const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                std::printf("TIMING %-36s %-6s %3d workers %4zu points %8.3f s  hash %016llx  %s\n", e.name.c_str(),
                            q == MapQuality::Sketch ? "Sketch" : "Full", workers, pts, s, static_cast<unsigned long long>(hash),
                            status.c_str());
                std::fflush(stdout);
            }
        }
    }
}
