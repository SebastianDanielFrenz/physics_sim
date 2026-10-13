// tests/unit/test_assist_channel_writeback.cpp - assists plan I7c (vault PHYS-074): a drive-mode change WRITES its
// traction_control_default / launch_control_default into the control channel itself (no override state any more).
//
// Problem (owner, 2026-10-13): the I7/I7b override was released only when the channel VALUE changed, but the channel kept
// the old value, so a UI toggle needed two presses (writing the value the channel already holds is invisible). Fix: the
// vehicle queues a pending channel write in step_tick (a job cannot touch World's map); World::step applies the writes
// sequentially after the pipeline in vehicle id order. The effective value in the tick of the change already uses the default.
//
//   W1  one press, launch control: Normal -> Sport turns LC on (bit4) AND get_control reads 1 right after the step; ONE write
//       of 0 turns it off on the next tick and it stays off (20 ticks); one write of 1 turns it back on; Sport -> Normal
//       writes 0.
//   W2  one press, traction control: Normal -> Sport: get_control == 2 (Relaxed), effective 2; one write of 3 takes effect
//       next tick and persists 20 ticks; one write of 2 (the value the physics wrote earlier) takes effect; one write of 0
//       gives Off. The driver value is never touched while the mode does not change (100 ticks in Sport).
//   W3  same tick: a mode change and a driver write in ONE tick (TC 4, LC 0): the mode default wins for that tick (effective
//       TC 2, LC on) and is written over the driver's value (channels read 2 / 1); the NEXT tick's driver write wins.
//   W4  a mode without the default leaves the channel alone: Sport with the TC default removed keeps the driver's TC 3 in the
//       channel and in effect; spawn writes nothing (channels equal the part's defaults).
//   W5  LC not fitted (Street unit): the LC default is ignored without a crash (no channel, get_control 0); the TC code is
//       still written raw (Relaxed 2) while the effective mode resolves to On (1, I1 fallback).
//   W6  two vehicles: vehicle 1's mode change writes its own v1.* channels only; vehicle 0's writes stay on the bare names;
//       the writes are applied in vehicle id order (vehicle 0 first).
//   W7  determinism: 1 vs 10 workers equal state hash every tick with mode changes and TC / LC writes, zero own allocation per
//       tick (debug_alloc armed) with pending writes occurring; non-vacuity: the channel written by the physics changes at
//       least 4 times.
//   W8  adapter mirror helper (adapters/godot/src/channel_mirror.h, Godot-free): a World-written channel is adopted into the
//       adapter's atomic map; a user write made between the push and the adoption wins; an unchanged channel is not touched;
//       the scratch keeps its storage after the first tick.
//   W9  adapter loop end to end on a real World (push -> user writes -> step -> adopt, exactly the sim thread's order): the
//       adapter map reads LC 1 / TC 2 right after Sport; the car keeps LC on over 30 ticks (the push does not revert the
//       physics' write); ONE user write of 0 turns it off for good. Without the adoption step the same loop turns LC off
//       again the tick after the change (recorded sabotage).
//
// BANDS COMMITTED BEFORE THE FIRST RUN (assists I7c brief; all exact equalities, no numeric tolerance):
//   W1-W6 channel values and effective modes exactly as listed (bit4 of assist_active_bits = the effective LC enable, read one
//   tick after a write); W7 first differing hash index == size, own allocation == 0, >= 4 physics writes seen; W8 adopted
//   count and values exactly as listed, vector data pointer unchanged; W9 the stated booleans for all 30 + 2 ticks.
//
// RECORDED SABOTAGES (temporary env-switched edits in core/src and channel_mirror.h, reverted; run = [assist][world] + LD1,
// LD6, T1, P1):
//   1 World::step drops the pending writes                      -> LD1, LD6, P1, T1, W1, W2, W3, W5, W6, W7, W9 fail (23 assertions)
//   2 pending-write count never cleared (stale writes re-applied) -> W1, W2 ... fail (22 assertions)
//   3 mirror stores unconditionally instead of compare_exchange  -> W8 fails (3)
//   6 the LC write goes to the TC channel                        -> LD1, LD6, P1, T1, W1, W2, W3, W6, W7, W9 fail (23)
//   7 a surviving heap allocation per queued write               -> W7 fails (own alloc 4 / 4 instead of 0 / 0; 2 assertions)
//   (the id-order rule cannot be observed black-box: vehicles use disjoint channel names, see electronics.md rule 4c.)

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/electronics_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/drive_modes_desc.h"
#include "vehicle_test_helpers.h"
#include "../../adapters/godot/src/channel_mirror.h"

#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps;
using namespace ps::vehicle;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
const std::string kPart = kData + "/electronics/drive_modes_car_hyper_n2o.json";
const std::string kStreet = kData + "/electronics/assist_unit_street.json";
const std::string kHyperN2o = kData + "/vehicles/car_hyper_n2o.json";

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_assist_writeback_" + tag);
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    [[nodiscard]] std::string file(const std::string& name) const { return (path / name).generic_string(); }
};

std::string cache_dir() {
    const std::string d = std::string(PS_SOURCE_DIR) + "/out/assists_i7c_map_cache";
    fs::create_directories(d);
    return d;
}

std::string part_with(const TempDir& td, const std::string& name, const std::function<void(json&)>& edit) {
    std::ifstream f(kPart);
    json j = json::parse(f);
    edit(j);
    std::ofstream o(td.file(name), std::ios::binary);
    o << j.dump(1);
    return td.file(name);
}

VehicleDesc load_car(io::VehicleLoadOptions o = {}) {
    static jobs::JobSystem pool(10);
    o.maps.pool = &pool;
    o.maps.cache_dir = cache_dir();
    return io::load_vehicle_json(kHyperN2o, o);
}

const VehicleDesc& n2o() {
    static const VehicleDesc d = load_car();
    return d;
}

io::VehicleLoadOptions with_modes(const std::string& path) {
    io::VehicleLoadOptions o;
    o.electronics["drive_modes"] = path;
    return o;
}

struct Rig {
    ps_test::VehicleTestRig rig;
    World& w() { return *rig.world; }
    [[nodiscard]] const drivetrain::PowertrainSnapshot& pt() { return rig.world->powertrain_state(rig.vehicle); }
};

Rig make_rig(const VehicleDesc& d, int workers = 1) {
    Rig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0.0, workers);
    return r;
}

// Normal mode, TC at the Normal default (On), wing automation off, LC channel at its default.
void base_controls(World& w) {
    w.set_control("assist.drive_mode", 1.0);
    w.set_control("assist.traction_control", 1.0);
    w.set_control("assist.aero_control", 0.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
}

bool lc_on(Rig& r) { return (r.pt().assist_active_bits & 16u) != 0; }
int tc_eff(Rig& r) { return static_cast<int>(r.pt().traction_mode_effective); }
void mode(Rig& r, int m) {
    r.w().set_control("assist.drive_mode", static_cast<real>(m));
    r.w().step();
}
void write(Rig& r, const char* ch, real v) {
    r.w().set_control(ch, v);
    r.w().step();
}

} // namespace

TEST_CASE("W1: one press - a mode change writes LC into the channel; a single write of 0 turns it off, a single write of 1 on", "[assist][drive_modes][vehicle][world]") {
    Rig r = make_rig(n2o());
    World& w = r.w();
    base_controls(w);
    w.step();
    CHECK_FALSE(lc_on(r));
    CHECK(w.get_control("assist.launch_control") == 0.0);
    mode(r, 2); // Normal -> Sport
    CHECK(lc_on(r));
    CHECK(w.get_control("assist.launch_control") == 1.0);
    // ONE write of 0: off on the next tick and it stays off
    write(r, "assist.launch_control", 0.0);
    CHECK_FALSE(lc_on(r));
    for (int k = 0; k < 20; ++k) {
        w.step();
        INFO("tick " << k);
        CHECK_FALSE(lc_on(r));
        CHECK(w.get_control("assist.launch_control") == 0.0);
    }
    // ONE write of 1: on again
    write(r, "assist.launch_control", 1.0);
    CHECK(lc_on(r));
    // Sport -> Normal writes 0
    mode(r, 1);
    CHECK_FALSE(lc_on(r));
    CHECK(w.get_control("assist.launch_control") == 0.0);
}

TEST_CASE("W2: one press - a mode change writes the TC code into the channel; a single write of any value takes effect", "[assist][drive_modes][vehicle][world]") {
    Rig r = make_rig(n2o());
    World& w = r.w();
    base_controls(w);
    w.step();
    CHECK(tc_eff(r) == 1);
    mode(r, 2);
    CHECK(tc_eff(r) == 2);
    CHECK(w.get_control("assist.traction_control") == 2.0);
    write(r, "assist.traction_control", 3.0);
    CHECK(tc_eff(r) == 3);
    for (int k = 0; k < 20; ++k) {
        w.step();
        CHECK(tc_eff(r) == 3);
    }
    write(r, "assist.traction_control", 2.0); // the value the physics wrote earlier
    CHECK(tc_eff(r) == 2);
    write(r, "assist.traction_control", 0.0);
    CHECK(tc_eff(r) == 0);
    // 100 ticks in Sport: the physics never touches the channel again
    for (int k = 0; k < 100; ++k) w.step();
    CHECK(w.get_control("assist.traction_control") == 0.0);
    CHECK(tc_eff(r) == 0);
}

TEST_CASE("W3: a mode change and a driver write in the same tick - the mode default wins that tick, the next tick's write wins", "[assist][drive_modes][vehicle][world]") {
    Rig r = make_rig(n2o());
    World& w = r.w();
    base_controls(w);
    for (int k = 0; k < 3; ++k) w.step();
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("assist.traction_control", 4.0);
    w.set_control("assist.launch_control", 0.0);
    w.step();
    CHECK(tc_eff(r) == 2);
    CHECK(lc_on(r));
    CHECK(w.get_control("assist.traction_control") == 2.0);
    CHECK(w.get_control("assist.launch_control") == 1.0);
    w.set_control("assist.traction_control", 4.0);
    w.set_control("assist.launch_control", 0.0);
    w.step();
    CHECK(tc_eff(r) == 4);
    CHECK_FALSE(lc_on(r));
}

TEST_CASE("W4: a mode without a default leaves the channel alone; spawn writes nothing", "[assist][drive_modes][vehicle][world][io]") {
    TempDir td("w4");
    const std::string p = part_with(td, "no_tc_default.json", [](json& j) {
        j["modes"]["sport"].erase("traction_control_default");
        j["modes"]["sport"].erase("launch_control_default");
    });
    Rig r = make_rig(load_car(with_modes(p)));
    World& w = r.w();
    CHECK(w.get_control("assist.traction_control") == 1.0); // part defaults
    CHECK(w.get_control("assist.launch_control") == 0.0);
    base_controls(w);
    w.set_control("assist.traction_control", 3.0);
    w.step();
    w.step();
    CHECK(w.get_control("assist.traction_control") == 3.0); // spawn: nothing written
    mode(r, 2);
    CHECK(w.get_control("assist.traction_control") == 3.0);
    CHECK(tc_eff(r) == 3);
    CHECK_FALSE(lc_on(r));
    CHECK(w.get_control("assist.launch_control") == 0.0);
    // a part whose default mode is sport: channel defaults are Sport's, and spawn writes nothing over a driver value
    const std::string q = part_with(td, "sport_default.json", [](json& j) { j["default_mode"] = "sport"; });
    Rig s = make_rig(load_car(with_modes(q)));
    CHECK(s.w().get_control("assist.traction_control") == 2.0);
    CHECK(s.w().get_control("assist.launch_control") == 1.0);
    s.w().set_control("assist.launch_control", 0.0);
    s.w().step();
    s.w().step();
    CHECK(s.w().get_control("assist.launch_control") == 0.0);
    CHECK_FALSE(lc_on(s));
}

TEST_CASE("W5: LC not fitted (Street unit) - the LC default is ignored, the TC code is still written raw", "[assist][drive_modes][vehicle][world][io]") {
    io::VehicleLoadOptions o = with_modes(kPart);
    o.electronics["assists"] = kStreet;
    Rig r = make_rig(load_car(o));
    World& w = r.w();
    base_controls(w);
    w.step();
    mode(r, 2);
    CHECK(w.get_control("assist.launch_control") == 0.0); // no such channel
    CHECK_FALSE(lc_on(r));
    CHECK(w.get_control("assist.traction_control") == 2.0); // raw request code
    CHECK(r.pt().traction_mode_requested == 2);
    CHECK(tc_eff(r) == 1); // Street offers Off/On only: Relaxed resolves to On
}

TEST_CASE("W6: two vehicles - each writes its own channels, in vehicle id order", "[assist][drive_modes][vehicle][world]") {
    Rig r = make_rig(n2o());
    World& w = r.w();
    BodyDesc c;
    c.shape = BoxShape{{2, .4, .15}};
    c.motion = BodyMotionType::Dynamic;
    c.mass = 1500;
    c.friction = .5;
    c.allow_sleep = false;
    c.pose.position = Vec3{0, 30, .5};
    const BodyId b2 = w.create_body(c);
    const VehicleId v2 = w.create_vehicle(n2o(), b2);
    base_controls(w);
    w.set_control("v1.assist.drive_mode", 1.0);
    w.set_control("v1.assist.traction_control", 1.0);
    w.set_control("v1.assist.aero_control", 0.0);
    w.step();
    // vehicle 1 only
    w.set_control("v1.assist.drive_mode", 2.0);
    w.step();
    CHECK(w.get_control("v1.assist.traction_control") == 2.0);
    CHECK(w.get_control("v1.assist.launch_control") == 1.0);
    CHECK(w.get_control("assist.traction_control") == 1.0);
    CHECK(w.get_control("assist.launch_control") == 0.0);
    CHECK(w.powertrain_state(v2).traction_mode_effective == 2);
    CHECK(tc_eff(r) == 1);
    // vehicle 0 only
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("v1.assist.drive_mode", 1.0);
    w.step();
    CHECK(w.get_control("assist.traction_control") == 2.0);
    CHECK(w.get_control("assist.launch_control") == 1.0);
    CHECK(w.get_control("v1.assist.traction_control") == 1.0);
    CHECK(w.get_control("v1.assist.launch_control") == 0.0);
}

namespace {
struct DetRun {
    std::vector<std::uint64_t> hashes;
    std::uint64_t own_alloc = 0;
    int physics_writes = 0; // ticks on which the TC or LC channel changed without a set_control in that tick
};

DetRun det_run(int workers, bool alloc) {
    Rig r = make_rig(n2o(), workers);
    World& w = r.w();
    base_controls(w);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 0.5);
    w.set_control("handbrake", 0.0);
    w.set_control("throttle", 1.0);
    w.set_control("brake", 1.0);
    namespace da = jobs::debug_alloc;
    const bool use_alloc = alloc && da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    DetRun out;
    for (int k = 0; k < 4 * 240; ++k) {
        if (k == 60) w.set_control("assist.drive_mode", 2.0);
        if (k == 200) w.set_control("assist.traction_control", 4.0);
        if (k == 300) w.set_control("assist.launch_control", 0.0);
        if (k == 420) w.set_control("assist.drive_mode", 1.0);
        if (k == 600) w.set_control("assist.launch_control", 1.0);
        if (k == 720) w.set_control("assist.drive_mode", 2.0);
        if (k == 840) w.set_control("assist.drive_mode", 0.0);
        const real tc0 = w.get_control("assist.traction_control"), lc0 = w.get_control("assist.launch_control");
        const bool user_write = k == 200 || k == 300 || k == 600;
        w.step();
        if (!user_write && (w.get_control("assist.traction_control") != tc0 || w.get_control("assist.launch_control") != lc0)) ++out.physics_writes;
        out.hashes.push_back(w.state_hash());
    }
    if (use_alloc) {
        da::set_armed(false);
        out.own_alloc = da::own_violation_count();
    }
    return out;
}
} // namespace

TEST_CASE("W7: 1 vs 10 workers equal hash every tick with pending channel writes; zero own allocation per tick", "[assist][drive_modes][vehicle][world][determinism]") {
    const DetRun one = det_run(1, true);
    const DetRun ten = det_run(10, true);
    REQUIRE(one.hashes.size() == ten.hashes.size());
    std::size_t first_diff = one.hashes.size();
    for (std::size_t i = 0; i < one.hashes.size(); ++i)
        if (one.hashes[i] != ten.hashes[i]) {
            first_diff = i;
            break;
        }
    std::printf("[W7] physics channel writes seen: %d / %d, own alloc %llu / %llu\n", one.physics_writes, ten.physics_writes,
                static_cast<unsigned long long>(one.own_alloc), static_cast<unsigned long long>(ten.own_alloc));
    CHECK(first_diff == one.hashes.size());
    CHECK(one.physics_writes >= 4);
    CHECK(ten.physics_writes == one.physics_writes);
    if (jobs::debug_alloc::hook_available()) {
        CHECK(one.own_alloc == 0);
        CHECK(ten.own_alloc == 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// W8 / W9: the adapter-side mirror (Godot-free helper).
// ---------------------------------------------------------------------------------------------------------------
namespace {
using Channels = std::unordered_map<std::string, std::atomic<float>>;

Channels make_channels(std::initializer_list<std::pair<const char*, float>> init) {
    Channels c;
    for (const auto& [k, v] : init) c.emplace(k, v);
    return c;
}
} // namespace

TEST_CASE("W8: channel_mirror - a World write is adopted; a user write made meanwhile wins; unchanged channels are untouched", "[assist][world][adapter]") {
    Channels ch = make_channels({{"a", 0.0f}, {"b", 2.0f}, {"c", 7.0f}});
    std::unordered_map<std::string, double> world;
    std::vector<float> pushed;
    const auto set_world = [&](const std::string& n, float v) { world[n] = v; };
    const auto get_world = [&](const std::string& n) { return world.at(n); };
    ps_godot::push_channels(ch, pushed, set_world);
    REQUIRE(pushed.size() == 3);
    const float* storage = pushed.data();
    CHECK(world["a"] == 0.0);
    CHECK(world["c"] == 7.0);
    // nothing changed
    CHECK(ps_godot::adopt_world_channel_writes(ch, pushed, get_world) == 0);
    // the "physics" writes a (0 -> 1) and b (2 -> 3); the user wrote b = 5 in the meantime (and c = 8, which the world did not touch)
    world["a"] = 1.0;
    world["b"] = 3.0;
    ch.at("b").store(5.0f);
    ch.at("c").store(8.0f);
    CHECK(ps_godot::adopt_world_channel_writes(ch, pushed, get_world) == 1);
    CHECK(ch.at("a").load() == 1.0f); // adopted
    CHECK(ch.at("b").load() == 5.0f); // the user's value wins
    CHECK(ch.at("c").load() == 8.0f); // untouched
    // the next push sends the adopted / user values
    ps_godot::push_channels(ch, pushed, set_world);
    CHECK(pushed.data() == storage); // no reallocation after the first tick
    CHECK(world["a"] == 1.0);
    CHECK(world["b"] == 5.0);
    CHECK(world["c"] == 8.0);
}

namespace {
// The sim thread's order for one tick, with the adapter's atomic map as the source of truth: push, (user writes happen
// between), step, adopt. `adopt` false = the pre-I7c adapter.
void adapter_tick(Rig& r, Channels& ch, std::vector<float>& pushed, bool adopt) {
    World& w = r.w();
    ps_godot::push_channels(ch, pushed, [&](const std::string& n, float v) { w.set_control(n, static_cast<real>(v)); });
    w.step();
    if (adopt) ps_godot::adopt_world_channel_writes(ch, pushed, [&](const std::string& n) { return w.get_control(n); });
}

struct AdapterLoop {
    bool lc_stays_on = true;
    float lc_channel_after_change = -1, tc_channel_after_change = -1;
    bool lc_off_after_one_write = false;
    bool lc_stays_off = true;
};

AdapterLoop run_adapter_loop(bool adopt) {
    Rig r = make_rig(n2o());
    World& w = r.w();
    base_controls(w);
    Channels ch = make_channels({{"assist.drive_mode", 1.0f}, {"assist.traction_control", 1.0f}, {"assist.launch_control", 0.0f}});
    std::vector<float> pushed;
    for (int k = 0; k < 3; ++k) adapter_tick(r, ch, pushed, adopt);
    ch.at("assist.drive_mode").store(2.0f); // the user selects Sport
    adapter_tick(r, ch, pushed, adopt);
    AdapterLoop out;
    out.lc_channel_after_change = ch.at("assist.launch_control").load();
    out.tc_channel_after_change = ch.at("assist.traction_control").load();
    for (int k = 0; k < 30; ++k) {
        adapter_tick(r, ch, pushed, adopt);
        if (!lc_on(r)) out.lc_stays_on = false;
    }
    ch.at("assist.launch_control").store(0.0f); // ONE user press
    adapter_tick(r, ch, pushed, adopt);
    out.lc_off_after_one_write = !lc_on(r);
    for (int k = 0; k < 30; ++k) {
        adapter_tick(r, ch, pushed, adopt);
        if (lc_on(r)) out.lc_stays_off = false;
    }
    return out;
}
} // namespace

TEST_CASE("W9: the adapter loop on a real World - the physics' write survives the next push; one user press turns LC off for good", "[assist][drive_modes][vehicle][world][adapter]") {
    const AdapterLoop with = run_adapter_loop(true);
    CHECK(with.lc_channel_after_change == 1.0f);
    CHECK(with.tc_channel_after_change == 2.0f);
    CHECK(with.lc_stays_on);
    CHECK(with.lc_off_after_one_write);
    CHECK(with.lc_stays_off);
    // the pre-I7c adapter (no adoption): the push puts the stale 0 over the physics' write and LC drops out again
    const AdapterLoop without = run_adapter_loop(false);
    std::printf("[W9] with adoption: lc channel %.0f tc channel %.0f stays_on %d off_after_press %d; without: lc channel %.0f stays_on %d\n",
                with.lc_channel_after_change, with.tc_channel_after_change, with.lc_stays_on, with.lc_off_after_one_write,
                without.lc_channel_after_change, without.lc_stays_on);
    CHECK_FALSE(without.lc_stays_on);
}
