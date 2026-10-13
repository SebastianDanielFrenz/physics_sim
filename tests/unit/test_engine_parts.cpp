// Engine garage P2 (out/engine_garage_design.md sections 4.4-7, A4-A8): part
// slots (default_ecu / default_cam / default_forced_induction on the engine;
// ecu / cam / forced_induction / installation / thermal on a vehicle's engine
// component; the user layer's PartOverride), "extends" in every engine-family
// part, the forced-induction rename, load_engine_json_from_string and the
// engine mass metadata.
//
// Every guard below was shown to fail on a sabotaged loader (the sabotage and
// the observed failure are recorded next to each case, then reverted).
//
// Scratch files live in a per-test temp directory; generated maps use a disk
// cache shared with test_turbo_io.cpp (ps_turbo_vehicle_override_cache), so a
// warm run loads them in milliseconds.

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/engine_rig.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/turbo_maps.h"
#include "ps/io/nitrous_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <variant>

using Catch::Matchers::ContainsSubstring;
using json = nlohmann::json;
using ps::drivetrain::MapQuality;
using ps::drivetrain::SimulatedEngineDesc;

namespace {

namespace fs = std::filesystem;

std::string data_dir() { return std::string(PS_SOURCE_DIR) + "/data"; }
std::string data_path(const std::string& rel) { return fs::weakly_canonical(fs::path(data_dir()) / rel).generic_string(); }

json read_json(const std::string& path) {
    std::ifstream in(path);
    REQUIRE(in);
    return json::parse(in);
}

std::string read_text(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

// "../x" -> "<abs base>/x" for every string value starting with "../".
void absolutize(json& j, const fs::path& base) {
    if (j.is_string()) {
        const std::string v = j.get<std::string>();
        if (v.rfind("../", 0) == 0) j = fs::weakly_canonical(base / v).generic_string();
    } else if (j.is_object() || j.is_array()) {
        for (auto& e : j) absolutize(e, base);
    }
}

struct TempDir {
    fs::path dir;
    TempDir() {
        static int counter = 0;
        dir = fs::temp_directory_path() / ("ps_engine_parts_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + "_" +
                                           std::to_string(++counter));
        fs::remove_all(dir);
        fs::create_directories(dir);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(dir, ec);
    }
    std::string write(const std::string& rel, const json& j) const {
        const fs::path p = dir / rel;
        fs::create_directories(p.parent_path());
        std::ofstream(p) << j.dump(2);
        return p.generic_string();
    }
};

ps::io::EngineMapOptions validate_only() {
    ps::io::EngineMapOptions o;
    o.generate = false;
    return o;
}

ps::io::EngineMapOptions preview() {
    static ps::jobs::JobSystem pool(4);
    ps::io::EngineMapOptions o;
    o.quality = MapQuality::Preview;
    o.pool = &pool;
    o.cache_dir = (fs::temp_directory_path() / "ps_turbo_vehicle_override_cache").string();
    return o;
}

const SimulatedEngineDesc& sim(const ps::drivetrain::ComponentParams& p) { return std::get<SimulatedEngineDesc>(p); }

const SimulatedEngineDesc& sim_engine(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) return *e;
    throw std::runtime_error("no simulated engine");
}

json& component(json& v, const std::string& id) {
    for (auto& c : v["powertrain"]["components"])
        if (c.value("id", "") == id) return c;
    throw std::runtime_error("no component " + id);
}

std::uint64_t key_of(const SimulatedEngineDesc& e, MapQuality q = MapQuality::Preview) {
    REQUIRE(e.cycle.has_value());
    return ps::drivetrain::engine_map_cache_key(*e.cycle, e.fuels.front(), q);
}

std::uint64_t hash_after(const ps::vehicle::VehicleDesc& d, int ticks, int workers) {
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500.0, ps::Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, workers);
    rig.world->set_control("throttle", 0.6);
    for (int i = 0; i < ticks; ++i) rig.world->step();
    return rig.world->state_hash();
}

// A free-rev trajectory of an engine on EngineRig, every value bit-exact (the rig has no state hash of its own).
std::string rig_trace(const ps::drivetrain::ComponentParams& engine, int ticks) {
    ps::EngineRig::Config cfg;
    cfg.initial_engine_state = ps::drivetrain::EngineState::Running;
    ps::EngineRig rig(engine, nullptr, cfg);
    std::string out;
    for (int i = 0; i < ticks; ++i) {
        rig.set_throttle(i < ticks / 2 ? 0.7 : 0.0);
        rig.step();
        const double v[3] = {rig.engine_omega_rad_s(), rig.engine_sound_state().p_intake_pa, rig.ledger().engine_out};
        out.append(reinterpret_cast<const char*>(v), sizeof(v));
    }
    return out;
}

json ecu_part(const json& fields) {
    json j = fields;
    j["format"] = "physics_sim.ecu/1";
    j["name"] = "test_ecu";
    return j;
}

} // namespace

// ---------------------------------------------------------------------------
// The shipped split: engines/hyper_v8_one1_audio.json names ecus/ and cams/
// part files; the fixture is the pre-split (master cb1f475) engine with its
// paths rebased. Same resolved desc, same .psmaps key, same turbo tables and
// the same state hash.
// Sabotage (recorded 2026-10-11): the loader not merging the cam patch into the
// cycle block - the split engine is rejected ("cycle: 'intake_lift' object is
// required").
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: the split hyper engine equals its inline pre-split form", "[io][drivetrain][turbo]") {
    const std::string split = data_dir() + "/engines/hyper_v8_one1_audio.json";
    const std::string inl = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/garage_p2/hyper_inline.json";
    REQUIRE(read_json(split).contains("default_ecu"));
    REQUIRE(read_json(split).contains("default_cam"));
    REQUIRE_FALSE(read_json(inl).contains("default_ecu"));

    const auto a = ps::io::load_engine_json(split, validate_only());
    const auto b = ps::io::load_engine_json(inl, validate_only());
    const auto& ea = sim(a);
    const auto& eb = sim(b);
    CHECK(ea.idle_rpm == eb.idle_rpm);
    CHECK(ea.limiter.rpm == eb.limiter.rpm);
    CHECK(ea.idle.kp_per_rpm == eb.idle.kp_per_rpm);
    CHECK(ea.throttle_actuator_tau_s == eb.throttle_actuator_tau_s);
    CHECK(ea.cycle->intake_lift.lift_m_vs_deg.y == eb.cycle->intake_lift.lift_m_vs_deg.y);
    CHECK(ea.cycle->exhaust_lift.phase_ref_deg == eb.cycle->exhaust_lift.phase_ref_deg);
    CHECK(key_of(ea, MapQuality::Preview) == key_of(eb, MapQuality::Preview));
    CHECK(key_of(ea, MapQuality::Full) == key_of(eb, MapQuality::Full));

    // Full loads through a vehicle: same turbo tables, same state hash at 1 and 10 workers.
    TempDir td;
    json v = read_json(data_dir() + "/vehicles/car_hyper.json");
    absolutize(v, fs::path(data_dir()) / "vehicles");
    const auto da = ps::io::load_vehicle_json(td.write("split.json", v), preview());
    component(v, "engine")["ref"] = fs::path(inl).generic_string();
    const auto db = ps::io::load_vehicle_json(td.write("inline.json", v), preview());
    REQUIRE(sim_engine(da).turbo_pair.has_value());
    REQUIRE(sim_engine(db).turbo_pair.has_value());
    REQUIRE(sim_engine(da).turbo_pair->maps != nullptr);
    CHECK(sim_engine(da).turbo_pair->maps->content_hash == sim_engine(db).turbo_pair->maps->content_hash);
    const std::uint64_t h = hash_after(da, 20, 1);
    CHECK(h == hash_after(db, 20, 1));
    CHECK(h == hash_after(da, 20, 10));
}

// ---------------------------------------------------------------------------
// "extends": a child engine in another directory extending a shipped engine,
// the parent's relative paths ("../fuels/e5.json", the installation) resolving
// against the PARENT's directory, equals an inline file with the same content.
// Sabotage (recorded 2026-10-11): parts::load_part without the rebase callback
// (ancestor paths left relative) - "cannot open file: <temp>/child\../fuels/
// gasoline.json".
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: an extends chain equals the inline file (key, desc, state hash)", "[io][drivetrain]") {
    TempDir td;
    const std::string parent = data_path("engines/demo_i4_pipes.json");
    // Chain of two: child -> mid (another directory) -> shipped parent.
    td.write("mid/mid.json", json{{"extends", parent}, {"idle_rpm", 820.0}});
    const std::string child = td.write("child/child.json", json{{"extends", "../mid/mid.json"}, {"mass_kg", 140.0}});
    json flat = read_json(parent);
    absolutize(flat, fs::path(data_dir()) / "engines");
    flat["idle_rpm"] = 820.0;
    flat["mass_kg"] = 140.0;
    const std::string inline_path = td.write("inline/inline.json", flat);

    const auto a = ps::io::load_engine_json(child, preview());
    const auto b = ps::io::load_engine_json(inline_path, preview());
    CHECK(sim(a).idle_rpm == 820.0);
    CHECK(sim(a).mass_kg == 140.0);
    CHECK(key_of(sim(a)) == key_of(sim(b)));
    CHECK(sim(a).fuels.front().name == sim(b).fuels.front().name);
    CHECK(rig_trace(a, 240) == rig_trace(b, 240));

    // The same through a vehicle: identical state hash.
    json v = read_json(fixture_vehicles_dir() + "/car_sedan_pipes.json");
    absolutize(v, fs::path(fixture_vehicles_dir()));
    component(v, "engine")["ref"] = child;
    const auto va = ps::io::load_vehicle_json(td.write("va.json", v), preview());
    component(v, "engine")["ref"] = inline_path;
    const auto vb = ps::io::load_vehicle_json(td.write("vb.json", v), preview());
    CHECK(hash_after(va, 20, 1) == hash_after(vb, 20, 1));
}

TEST_CASE("garage P2: extends in a turbo configuration and its hardware equals the inline file", "[io][drivetrain][turbo]") {
    TempDir td;
    // The child names only what changes; the parent's "turbo": "../turbos/..." resolves against the parent.
    const std::string child = td.write("cfg/child.json",
                                       json{{"extends", data_path("turbo_configurations/hyper_twin_default.json")}, {"target_boost_bar", 1.6}});
    json flat = read_json(data_dir() + "/turbo_configurations/hyper_twin_default.json");
    absolutize(flat, fs::path(data_dir()) / "turbo_configurations");
    flat["target_boost_bar"] = 1.6;
    const std::string inl = td.write("flat.json", flat);
    // A hardware child two directories away from its parent.
    const std::string hw = td.write("a/b/hw.json", json{{"extends", data_path("turbos/hyper_turbo_approx.json")}});
    json cfg_hw = flat;
    cfg_hw["turbo"] = hw;
    const std::string inl_hw = td.write("flat_hw.json", cfg_hw);

    const auto a = ps::io::load_turbo_configuration_json(child);
    const auto b = ps::io::load_turbo_configuration_json(inl);
    const auto c = ps::io::load_turbo_configuration_json(inl_hw);
    CHECK(a.target_boost_pa == Catch::Approx(1.6e5).epsilon(1e-12));
    REQUIRE(a.maps != nullptr);
    REQUIRE(b.maps != nullptr);
    REQUIRE(c.maps != nullptr);
    CHECK(a.maps->content_hash == b.maps->content_hash);
    CHECK(b.maps->content_hash == c.maps->content_hash);
    CHECK(a.wastegate_area() == b.wastegate_area());
}

// Sabotage (recorded 2026-10-11): max_depth 8 -> 100 in parts::load_part - "depth 9" throws nothing.
TEST_CASE("garage P2: extends cycles and chains deeper than 8 are rejected", "[io][drivetrain]") {
    TempDir td;
    SECTION("a cycle") {
        td.write("a.json", json{{"extends", "b.json"}, {"idle_rpm", 900.0}});
        td.write("b.json", json{{"extends", "a.json"}});
        CHECK_THROWS_WITH(ps::io::load_engine_json((td.dir / "a.json").string(), validate_only()), ContainsSubstring("extends cycle"));
    }
    SECTION("depth 9") {
        std::string prev = data_path("engines/demo_i4_pipes.json");
        for (int i = 0; i < 9; ++i) prev = td.write("d" + std::to_string(i) + ".json", json{{"extends", prev}});
        CHECK_THROWS_WITH(ps::io::load_engine_json(prev, validate_only()), ContainsSubstring("extends chain deeper than 8"));
    }
    SECTION("depth 8 loads") {
        std::string prev = data_path("engines/demo_i4_pipes.json");
        for (int i = 0; i < 8; ++i) prev = td.write("d" + std::to_string(i) + ".json", json{{"extends", prev}});
        CHECK_NOTHROW(ps::io::load_engine_json(prev, validate_only()));
    }
    SECTION("an n2o bottle and a nitrous kit extend their parents (relative paths per file)") {
        const std::string bottle = td.write("n/bottle.json", json{{"extends", data_path("n2o_bottles/bottle_10lb_950psi.json")}});
        CHECK_NOTHROW(ps::io::load_n2o_bottle_json(bottle));
        const std::string kit = td.write("n/kit.json", json{{"extends", data_path("nitrous_kits/edelbrock_70011_square_50.json")}});
        CHECK_NOTHROW(ps::io::load_nitrous_kit_json(kit));
        const std::string inst = td.write("n/inst.json", json{{"extends", data_path("installations/demo_i4_exhaust.json")}});
        CHECK_NOTHROW(ps::io::load_installation_json(inst));
    }
}

// ---------------------------------------------------------------------------
// Slots: the user layer beats the vehicle, the vehicle beats the engine file,
// the engine file beats the inline content (bottom-up, the user last).
// Sabotage (recorded 2026-10-11): pick() preferring the engine default over an
// override - "vehicle level" reads idle 900.0 / phase -10.0 instead of 1000 / -12.
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: each slot at engine, vehicle and user level wins bottom-up", "[io][drivetrain][turbo]") {
    TempDir td;
    // The engine's whole ECU and cam content moves into part files (inline + slot is rejected); the vehicle and
    // user parts are complete copies differing in idle_rpm / the intake cam phase.
    json eng = read_json(data_dir() + "/engines/demo_i4_pipes.json");
    absolutize(eng, fs::path(data_dir()) / "engines");
    json ecu = json::object();
    for (const char* k : {"idle_rpm", "stall_rpm", "fire_rpm", "start_complete_rpm", "dfco_enable_rpm", "dfco_reenable_rpm",
                          "idle_control", "limiter", "throttle_actuator_tau_s", "ecu"}) {
        if (eng.contains(k)) {
            ecu[k] = eng[k];
            eng.erase(k);
        }
    }
    auto ecu_with_idle = [&](double idle) {
        json e = ecu_part(ecu);
        e["idle_rpm"] = idle;
        return e;
    };
    td.write("ecus/e.json", ecu_with_idle(900.0));
    td.write("ecus/v.json", ecu_with_idle(1000.0));
    const std::string ecu_u = td.write("ecus/u.json", ecu_with_idle(1100.0));
    json cam = {{"format", "physics_sim.cam/1"}, {"name", "c"}, {"intake_lift", eng["cycle"]["intake_lift"]},
                {"exhaust_lift", eng["cycle"]["exhaust_lift"]}};
    eng["cycle"].erase("intake_lift");
    eng["cycle"].erase("exhaust_lift");
    auto cam_with_phase = [&](double phase) {
        json c = cam;
        c["intake_lift"]["phase_ref_deg"] = phase;
        return c;
    };
    td.write("cams/e.json", cam_with_phase(-10.0));
    const std::string cam_v = td.write("cams/v.json", cam_with_phase(-12.0));
    td.write("cams/u.json", cam_with_phase(-14.0));
    eng["default_ecu"] = "../ecus/e.json";
    eng["default_cam"] = "../cams/e.json";
    const std::string eng_path = td.write("engines/eng.json", eng);
    json v = read_json(fixture_vehicles_dir() + "/car_sedan_pipes.json");
    absolutize(v, fs::path(fixture_vehicles_dir()));
    component(v, "engine")["ref"] = eng_path;
    const std::string inst_v = data_path("installations/demo_i4_exhaust.json");

    auto load = [&](const json& vehicle, const ps::io::VehicleLoadOptions& o) {
        return ps::io::load_vehicle_json(td.write("vehicles/v.json", vehicle), o);
    };
    ps::io::VehicleLoadOptions none;
    none.maps = validate_only();
    SECTION("engine level") {
        const auto d = load(v, none);
        const auto& e = sim_engine(d);
        CHECK(e.idle_rpm == 900.0);
        CHECK(e.cycle->intake_lift.phase_ref_deg == -10.0);
    }
    json vv = v;
    component(vv, "engine")["ecu"] = "../ecus/v.json";
    component(vv, "engine")["cam"] = cam_v;
    SECTION("vehicle level") {
        const auto d = load(vv, none);
        const auto& e = sim_engine(d);
        CHECK(e.idle_rpm == 1000.0);
        CHECK(e.cycle->intake_lift.phase_ref_deg == -12.0);
    }
    SECTION("user level") {
        ps::io::VehicleLoadOptions o = none;
        o.part_overrides["engine"].slots = {{"ecu", ecu_u}, {"cam", "../cams/u.json"}};
        const auto d = load(vv, o);
        const auto& e = sim_engine(d);
        CHECK(e.idle_rpm == 1100.0);
        CHECK(e.cycle->intake_lift.phase_ref_deg == -14.0);
    }
    SECTION("installation: vehicle and user slot override the engine default") {
        json vi = v;
        component(vi, "engine")["installation"] = inst_v;
        const auto d = load(vi, none);
        const auto& e = sim_engine(d);
        CHECK(e.cycle->installation_path == inst_v);
        ps::io::VehicleLoadOptions o = none;
        o.part_overrides["engine"].slots = {{"installation", "../installations/missing.json"}};
        CHECK_THROWS_WITH(load(vi, o), ContainsSubstring("missing.json"));
    }
    SECTION("forced induction: engine default < vehicle < user (and \"none\" at user level)") {
        json h = read_json(data_dir() + "/vehicles/car_hyper.json");
        absolutize(h, fs::path(data_dir()) / "vehicles");
        json c16 = read_json(data_dir() + "/turbo_configurations/hyper_twin_default.json");
        absolutize(c16, fs::path(data_dir()) / "turbo_configurations");
        c16["target_boost_bar"] = 1.6;
        json c17 = c16;
        c17["target_boost_bar"] = 1.7;
        const std::string p16 = td.write("tc/c16.json", c16);
        const std::string p17 = td.write("tc/c17.json", c17);
        auto boost = [&](const json& vehicle, const ps::io::VehicleLoadOptions& o) {
            const auto d = load(vehicle, o);
            const auto& e = sim_engine(d);
            REQUIRE(e.turbo_pair.has_value());
            return e.turbo_pair->target_boost_pa;
        };
        CHECK(boost(h, none) == Catch::Approx(1.8e5).epsilon(1e-12));
        component(h, "engine")["forced_induction"] = p16;
        CHECK(boost(h, none) == Catch::Approx(1.6e5).epsilon(1e-12));
        ps::io::VehicleLoadOptions o = none;
        o.part_overrides["engine"].slots = {{"forced_induction", p17}};
        CHECK(boost(h, o) == Catch::Approx(1.7e5).epsilon(1e-12));
        o.part_overrides["engine"].slots = {{"forced_induction", "none"}};
        CHECK_THROWS_WITH(load(h, o), ContainsSubstring("above ambient"));
    }
    SECTION("thermal: engine default < vehicle < user") {
        json t = read_json(fixture_vehicles_dir() + "/car_sedan_thermal.json");
        absolutize(t, fs::path(fixture_vehicles_dir()));
        auto src = [&](const json& vehicle, const ps::io::VehicleLoadOptions& o) {
            const auto d = load(vehicle, o);
            REQUIRE(d.powertrain.thermal.has_value());
            return d.powertrain.thermal->engine.source;
        };
        const std::string s_engine = src(t, none);
        component(t, "engine")["thermal"] = data_path("engine_thermal/toyota_a25a_fks.json");
        const std::string s_vehicle = src(t, none);
        // The user's file extends the vehicle's choice and changes only its note.
        const std::string user = td.write("thermal/u.json", json{{"extends", data_path("engine_thermal/toyota_a25a_fks.json")},
                                                                  {"source", "user thermal"}});
        ps::io::VehicleLoadOptions o = none;
        o.part_overrides["engine"].slots = {{"thermal", user}};
        CHECK(s_engine != s_vehicle);
        CHECK(src(t, o) == "user thermal");
    }
}

// ---------------------------------------------------------------------------
// The user layer replaces part files ("ref") and the nitrous kit's controller.
// Sabotage (recorded 2026-10-11): apply_part_override ignoring ov.ref - "a
// gearbox ref" reads 6 == 7 forward ratios, "an engine ref" keeps the pipe engine.
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: the user layer replaces part files and rejects what it cannot apply", "[io][drivetrain]") {
    const std::string car = fixture_vehicles_dir() + "/car_sedan_pipes.json";
    ps::io::VehicleLoadOptions o;
    o.maps = validate_only();
    SECTION("no overrides = the plain load") {
        const auto a = ps::io::load_vehicle_json(car, o);
        const auto b = ps::io::load_vehicle_json(car, validate_only());
        CHECK(key_of(sim_engine(a)) == key_of(sim_engine(b)));
    }
    SECTION("an engine ref (relative to the vehicle file)") {
        o.part_overrides["engine"].ref = "../../../../data/engines/toyota_a25a_fks_gen.json";
        const auto d = ps::io::load_vehicle_json(car, o);
        const auto& e = sim_engine(d);
        REQUIRE(e.cycle.has_value());
        CHECK_FALSE(e.cycle->intake.has_value()); // the A25A has no pipe blocks
    }
    SECTION("a gearbox ref") {
        o.part_overrides["gearbox"].ref = "../../../../data/gearboxes/hyper_7mt.json";
        const auto d = ps::io::load_vehicle_json(car, o);
        bool found = false;
        for (const auto& c : d.powertrain.components)
            if (const auto* g = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) {
                CHECK(g->forward_ratios.size() == 7);
                found = true;
            }
        CHECK(found);
    }
    SECTION("an unknown component id") {
        o.part_overrides["turbo"].ref = "x.json";
        CHECK_THROWS_WITH(ps::io::load_vehicle_json(car, o), ContainsSubstring("part_overrides: no powertrain component with id 'turbo'"));
    }
    SECTION("an unknown slot") {
        o.part_overrides["engine"].slots = {{"supercharger", "x.json"}};
        CHECK_THROWS_WITH(ps::io::load_vehicle_json(car, o), ContainsSubstring("unknown slot 'supercharger'"));
    }
    SECTION("a component without a part file") {
        o.part_overrides["clutch"].ref = "x.json";
        CHECK_THROWS_WITH(ps::io::load_vehicle_json(car, o), ContainsSubstring("clutch"));
    }
    SECTION("the nitrous kit's controller") {
        const std::string n2o = data_dir() + "/vehicles/car_hyper_n2o.json";
        o.part_overrides["n2o_kit"].slots = {{"controller", "../nitrous_controllers/edelbrock_basic.json"}};
        CHECK_NOTHROW(ps::io::load_vehicle_json(n2o, o));
        o.part_overrides["n2o_kit"].slots = {{"ecu", "x.json"}};
        CHECK_THROWS_WITH(ps::io::load_vehicle_json(n2o, o), ContainsSubstring("ecu"));
    }
}

// ---------------------------------------------------------------------------
// Rejections by message.
// Sabotage (recorded 2026-10-11): inline_ecu_field never reporting a field -
// "inline ECU content and default_ecu" throws nothing.
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: renamed keys, inline + slot, unknown slots and bad parts are rejected", "[io][drivetrain]") {
    TempDir td;
    json eng = read_json(data_dir() + "/engines/demo_i4_pipes.json");
    absolutize(eng, fs::path(data_dir()) / "engines");
    auto load = [&](const json& e) { return ps::io::load_engine_json(td.write("e.json", e), validate_only()); };
    auto strip_ecu = [](json e) {
        for (const char* k : {"idle_rpm", "stall_rpm", "fire_rpm", "start_complete_rpm", "dfco_enable_rpm", "dfco_reenable_rpm",
                              "idle_control", "limiter", "throttle_actuator_tau_s", "ecu"})
            e.erase(k);
        return e;
    };
    SECTION("default_turbo_configuration") {
        json e = eng;
        e["default_turbo_configuration"] = data_path("turbo_configurations/hyper_twin_default.json");
        CHECK_THROWS_WITH(load(e), ContainsSubstring("'default_turbo_configuration' was renamed 'default_forced_induction'"));
    }
    SECTION("inline ECU content and default_ecu") {
        json e = eng;
        e["default_ecu"] = td.write("ecu.json", ecu_part(json{{"idle_rpm", 900.0}}));
        CHECK_THROWS_WITH(load(e), ContainsSubstring("'default_ecu' and the inline ECU field 'idle_control'"));
    }
    SECTION("inline cam content and default_cam") {
        json e = eng;
        e["default_cam"] = td.write("cam.json", json{{"format", "physics_sim.cam/1"}, {"name", "c"}, {"intake_lift", eng["cycle"]["intake_lift"]}});
        CHECK_THROWS_WITH(load(e), ContainsSubstring("'default_cam' and the inline cam field 'cycle.intake_lift'"));
    }
    SECTION("an unknown engine slot") {
        json e = eng;
        e["default_supercharger"] = "x.json";
        CHECK_THROWS_WITH(load(e), ContainsSubstring("unknown slot 'default_supercharger'"));
    }
    SECTION("\"none\" on a slot other than forced induction") {
        json e = strip_ecu(eng);
        e["default_ecu"] = "none";
        CHECK_THROWS_WITH(load(e), ContainsSubstring("only allowed for forced induction"));
    }
    SECTION("an ECU part with a non-ECU field") {
        json e = strip_ecu(eng);
        e["default_ecu"] = td.write("ecu.json", ecu_part(json{{"idle_rpm", 900.0}, {"bore_mm", 80.0}}));
        CHECK_THROWS_WITH(load(e), ContainsSubstring("'bore_mm' is not an ECU field"));
    }
    SECTION("an ECU part with the wrong format") {
        json e = strip_ecu(eng);
        json p = ecu_part(json{{"idle_rpm", 900.0}});
        p["format"] = "physics_sim.cam/1";
        e["default_ecu"] = td.write("ecu.json", p);
        CHECK_THROWS_WITH(load(e), ContainsSubstring("physics_sim.ecu/1"));
    }
    SECTION("cam fits: valve count, valve diameter, unknown key") {
        json e = eng;
        json c = {{"format", "physics_sim.cam/1"}, {"name", "c"}, {"intake_lift", e["cycle"]["intake_lift"]},
                  {"exhaust_lift", e["cycle"]["exhaust_lift"]}};
        e["cycle"].erase("intake_lift");
        e["cycle"].erase("exhaust_lift");
        const int iv = eng["cycle"]["intake_valves"].get<int>();
        const double dv = eng["cycle"]["intake_valve_d_mm"].get<double>();
        c["fits"] = {{"intake_valves", iv}, {"intake_valve_d_mm", {dv - 1.0, dv + 1.0}}};
        e["default_cam"] = td.write("cam_ok.json", c);
        CHECK_NOTHROW(load(e));
        c["fits"] = {{"intake_valves", iv + 1}};
        e["default_cam"] = td.write("cam_n.json", c);
        CHECK_THROWS_WITH(load(e), ContainsSubstring("the cam does not fit this engine"));
        c["fits"] = {{"intake_valve_d_mm", {dv + 1.0, dv + 2.0}}};
        e["default_cam"] = td.write("cam_d.json", c);
        CHECK_THROWS_WITH(load(e), ContainsSubstring("outside fits.intake_valve_d_mm"));
        c["fits"] = {{"lobe_separation_deg", 110}};
        e["default_cam"] = td.write("cam_k.json", c);
        CHECK_THROWS_WITH(load(e), ContainsSubstring("fits: unknown key 'lobe_separation_deg'"));
        c.erase("fits");
        c["rod_mm"] = 140.0;
        e["default_cam"] = td.write("cam_r.json", c);
        CHECK_THROWS_WITH(load(e), ContainsSubstring("'rod_mm' is not a cam field"));
    }
    SECTION("mass_kg / mass_source") {
        json e = eng;
        e["mass_kg"] = -1.0;
        CHECK_THROWS_WITH(load(e), ContainsSubstring("mass_kg"));
        e.erase("mass_kg");
        e["mass_source"] = "[AS]";
        CHECK_THROWS_WITH(load(e), ContainsSubstring("mass_source"));
        e["mass_kg"] = 135.0;
        const auto ok = load(e);
        CHECK(sim(ok).mass_kg == 135.0);
        CHECK(sim(ok).mass_source == "[AS]");
    }
    SECTION("a torque-map engine with forced induction") {
        json t = read_json(data_dir() + "/engines/i4_2l_na.json");
        absolutize(t, fs::path(data_dir()) / "engines");
        t["default_forced_induction"] = data_path("turbo_configurations/hyper_twin_default.json");
        CHECK_THROWS_WITH(load(t), ContainsSubstring("is only allowed on a simulated engine"));
    }
}

// ---------------------------------------------------------------------------
// forced_induction "none" on the turbo hyper: rejected as shipped (its grid
// reaches 250 kPa), loads naturally aspirated with a grid at or below ambient.
// Sabotage (recorded 2026-10-11): the grid check disabled - both CHECK_THROWS
// get no exception at validate-only (the generator would fail later, G3).
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: forced induction \"none\" on the hyper", "[io][drivetrain][turbo]") {
    const std::string hyper = data_dir() + "/engines/hyper_v8_one1_audio.json";
    const ps::io::EngineSlotOverrides none = {{"forced_induction", "none"}};
    CHECK_THROWS_WITH(ps::io::load_engine_json(hyper, validate_only(), nullptr, none),
                      ContainsSubstring("forced induction is \"none\""));
    CHECK_THROWS_WITH(ps::io::load_engine_json(hyper, validate_only(), nullptr, none), ContainsSubstring("Fix: give the engine a grid_p_im_kpa"));

    // The fix the message names: an engine that extends the hyper and replaces the grid. Top node 95 kPa [AS]:
    // the NA hyper's wide-open plenum reaches 97.87 kPa at 8000 rpm [M] (a 100 kPa node fails the generator with
    // "plenum target 100000 Pa unreachable at 8000 rpm").
    const json grid = {25.0, 50.0, 75.0, 95.0};
    json child = {{"extends", "hyper_v8_one1_audio.json"}, {"cycle", {{"grid_p_im_kpa", grid}}}};
    const auto na = ps::io::load_engine_json_from_string(child.dump(), fs::path(data_dir()) / "engines", validate_only(), none);
    const auto& e = sim(na);
    CHECK_FALSE(e.turbo_pair.has_value());
    REQUIRE(e.cycle->intake.has_value());
    CHECK_FALSE(e.cycle->intake->compressor_supply);
    CHECK(e.cycle->grid_p_im.back() <= ps::drivetrain::kIsaAirPressurePa);
    // And it generates (Sketch, the garage preview grid).
    ps::io::EngineMapOptions sketch = preview();
    sketch.quality = MapQuality::Sketch;
    const auto gen = ps::io::load_engine_json_from_string(child.dump(), fs::path(data_dir()) / "engines", sketch, none);
    CHECK_FALSE(sim(gen).maps_pending);
}

// ---------------------------------------------------------------------------
// load_engine_json_from_string == load_engine_json of the same bytes.
// Sabotage (recorded 2026-10-11): the in-memory engine's virtual path without
// base_dir (its "extends" resolved against the current directory) -
// "demo_i4_pipes.json: cannot open file".
// ---------------------------------------------------------------------------
TEST_CASE("garage P2: load_engine_json_from_string equals the file load", "[io][drivetrain]") {
    const std::string path = data_dir() + "/engines/demo_i4_pipes.json";
    const std::string text = read_text(path);
    const auto a = ps::io::load_engine_json(path, preview());
    const auto b = ps::io::load_engine_json_from_string(text, fs::path(data_dir()) / "engines", preview());
    CHECK(key_of(sim(a)) == key_of(sim(b)));
    CHECK(fs::equivalent(sim(a).cycle->installation_path, sim(b).cycle->installation_path));
    CHECK(rig_trace(a, 240) == rig_trace(b, 240));
    // An extends chain from memory resolves against base_dir too.
    const auto c = ps::io::load_engine_json_from_string(R"({"extends": "demo_i4_pipes.json"})", fs::path(data_dir()) / "engines", preview());
    CHECK(key_of(sim(c)) == key_of(sim(a)));
    CHECK_THROWS_WITH(ps::io::load_engine_json_from_string("{ not json", fs::path(data_dir()) / "engines"), ContainsSubstring("invalid JSON"));
}
