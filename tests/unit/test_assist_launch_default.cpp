// tests/unit/test_assist_launch_default.cpp - assists plan I7b (vault PHYS-074): the per-drive-mode launch-control DEFAULT.
// A drive mode may set `launch_control_default` (bool). Like the traction-control default (I7): when the EFFECTIVE drive
// mode changes to a mode that sets one, the effective launch-control enable becomes that value and stays until the driver
// changes the assist.launch_control channel VALUE (a rewrite of the same value is not seen); launch_control_allowed still
// gates on top. Owner decision 2026-10-13: "turn LC on when Sport mode is activated".
//
//   LD1  state machine through a World on car_hyper_n2o (assist_active_bits bit4 = the effective enable, read one tick after a
//        write): spawn in Normal -> bit4 clear, LC channel 0; Normal -> Sport -> bit4 set with the channel untouched (== 0);
//        Sport -> Normal -> clear; Normal -> Sport -> Eco -> Sport sets it again; a driver write of a DIFFERENT value wins
//        until the next mode change (Sport, override on, last seen 0: write 1 releases and keeps it on, write 0 then turns
//        it OFF while still in Sport); a same-value rewrite (0, 0, 0 ... for 20 ticks) does not release the override.
//   LD2  spawn: default_mode sport (part edited) -> the LC channel default reads 1 and bit4 is set on the first tick, no
//        override; a default mode that sets no launch_control_default keeps the channel default 0; a car without the part
//        (and car_hyper) behave as before: channel 0, writing 1 switches it on, mode channel does nothing.
//   LD3  arming: from Normal, switch to Sport with NO assist.launch_control write, throttle 1 + brake 1 from rest in 1st gear
//        -> launch_phase Armed (1) seen within 1.5 s (360 ticks); the same in Normal (allowed edited to true so only the
//        DEFAULT keeps it off) never arms, and neither does Eco.
//   LD4  loader: launch_control_default true next to launch_control_allowed false in one mode is rejected with a message
//        naming both keys; a non-boolean is rejected; an unknown key is still rejected; a default on a car whose LC is
//        not fitted loads (ignored, mirrors traction_control_default).
//   LD5  determinism: 1 vs 10 workers, equal state hash on every tick with mode changes and LC writes mid-run; zero own
//        allocation per tick (debug_alloc).
//   LD6  hash coverage: two runs whose physics is identical (effective LC on in both), one with the override still active
//        and one released by a driver write of a different value, have DIFFERENT state hashes; with the launch_control_default
//        keys stripped from the part, the Normal -> Sport change leaves LC off (no default, nothing folded).
//
// BANDS COMMITTED BEFORE THE FIRST RUN (assists plan I7b, brief assists_i7b_lc_default; all [AS]):
//   LD1/LD2 exact equalities on bit4 and the channel value as listed; LD3 armed seen in Sport (>= 1 tick) and 0 ticks in
//   Normal/Eco within 360 ticks; LD5 first differing hash index == size, own allocation == 0; LD6 hash inequality /
//   LC-off equality as listed. No numeric tolerance is involved.

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/electronics_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/drive_modes_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps;
using namespace ps::vehicle;
using Catch::Matchers::ContainsSubstring;
namespace fs = std::filesystem;
using json = nlohmann::json;

const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
const std::string kPart = kData + "/electronics/drive_modes_car_hyper_n2o.json";
const std::string kHyperN2o = kData + "/vehicles/car_hyper_n2o.json";
const std::string kHyper = kData + "/vehicles/car_hyper.json";

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_assist_lc_default_" + tag);
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    [[nodiscard]] std::string file(const std::string& name) const { return (path / name).generic_string(); }
};

void write_text(const std::string& p, const std::string& text) {
    std::ofstream f(p, std::ios::binary);
    f << text;
}

std::string cache_dir() {
    const std::string d = std::string(PS_SOURCE_DIR) + "/out/assists_i7b_map_cache";
    fs::create_directories(d);
    return d;
}

json shipped_json() {
    std::ifstream f(kPart);
    return json::parse(f);
}

std::string part_with(const TempDir& td, const std::string& name, const std::function<void(json&)>& edit) {
    json j = shipped_json();
    edit(j);
    write_text(td.file(name), j.dump(1));
    return td.file(name);
}

VehicleDesc load_car(const std::string& vehicle, io::VehicleLoadOptions o = {}) {
    static jobs::JobSystem pool(10);
    o.maps.pool = &pool;
    o.maps.cache_dir = cache_dir();
    return io::load_vehicle_json(vehicle, o);
}

const VehicleDesc& n2o() {
    static const VehicleDesc d = load_car(kHyperN2o);
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
};

Rig make_rig(const VehicleDesc& d, int workers = 1) {
    Rig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0.0, workers);
    return r;
}

void base_controls(World& w, int mode) {
    w.set_control("assist.drive_mode", static_cast<real>(mode));
    w.set_control("assist.traction_control", 0.0);
    w.set_control("assist.aero_control", 0.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
}

bool lc_on(Rig& r) { return (r.w().powertrain_state(r.rig.vehicle).assist_active_bits & 16u) != 0; }
void drive_mode(Rig& r, int m) {
    r.w().set_control("assist.drive_mode", static_cast<real>(m));
    r.w().step();
}
void write_lc(Rig& r, real v) {
    r.w().set_control("assist.launch_control", v);
    r.w().step();
}

} // namespace

TEST_CASE("LD1: a mode change sets the effective LC enable; a changed channel value wins; a same-value rewrite does not", "[assist][drive_modes][vehicle][drivetrain]") {
    Rig r = make_rig(n2o());
    World& w = r.w();
    base_controls(w, 1);
    CHECK(w.get_control("assist.launch_control") == 0.0);
    w.step();
    CHECK_FALSE(lc_on(r));
    // Normal -> Sport: on, channel untouched
    drive_mode(r, 2);
    CHECK(lc_on(r));
    CHECK(w.get_control("assist.launch_control") == 0.0);
    // Sport -> Normal: off
    drive_mode(r, 1);
    CHECK_FALSE(lc_on(r));
    // Normal -> Sport -> Eco -> Sport: on, off, on
    drive_mode(r, 2);
    CHECK(lc_on(r));
    drive_mode(r, 0);
    CHECK_FALSE(lc_on(r));
    drive_mode(r, 2);
    CHECK(lc_on(r));
    // a same-value rewrite (the channel stays 0) does not release the override
    for (int k = 0; k < 20; ++k) {
        write_lc(r, 0.0);
        INFO("tick " << k);
        CHECK(lc_on(r));
    }
    // a different value releases it: 1 keeps LC on (the channel now says 1) ...
    write_lc(r, 1.0);
    CHECK(lc_on(r));
    // ... and 0 then turns it OFF while still in Sport (the channel wins)
    write_lc(r, 0.0);
    CHECK_FALSE(lc_on(r));
    w.step();
    CHECK_FALSE(lc_on(r));
    // the next mode change applies the default again, over the driver's 0
    drive_mode(r, 1);
    CHECK_FALSE(lc_on(r));
    drive_mode(r, 2);
    CHECK(lc_on(r));
    // Sport -> Normal with the channel at 1 (driver on): the Normal default (off) wins over the channel
    write_lc(r, 1.0);
    CHECK(lc_on(r));
    drive_mode(r, 1);
    CHECK_FALSE(lc_on(r));
    // ... and afterwards the driver's channel wins over the default, but Normal does not ALLOW LC (shipped part), so the
    // allowed gate keeps it off whatever the channel says (set-up fault of the first run: this was written as CHECK(on))
    write_lc(r, 0.0);
    CHECK_FALSE(lc_on(r));
    write_lc(r, 1.0);
    CHECK_FALSE(lc_on(r));
    drive_mode(r, 2); // Sport allows it again: the default (on) re-applies
    CHECK(lc_on(r));
}

TEST_CASE("LD2: spawn default, a default mode without a default, cars without the part", "[assist][drive_modes][vehicle][drivetrain][io]") {
    TempDir td("ld2");
    {
        const std::string p = part_with(td, "sport_default.json", [](json& j) { j["default_mode"] = "sport"; });
        Rig r = make_rig(load_car(kHyperN2o, with_modes(p)));
        World& w = r.w();
        CHECK(w.get_control("assist.launch_control") == 1.0);
        w.step();
        CHECK(lc_on(r));
        // no override at spawn: a driver write of 0 (a change from the default 1) turns it off at once
        write_lc(r, 0.0);
        CHECK_FALSE(lc_on(r));
    }
    {
        // the default mode (normal) sets no launch_control_default: the channel default stays 0, and Sport still switches it on
        const std::string p = part_with(td, "normal_none.json", [](json& j) { j["modes"]["normal"].erase("launch_control_default"); });
        Rig r = make_rig(load_car(kHyperN2o, with_modes(p)));
        World& w = r.w();
        base_controls(w, 1);
        CHECK(w.get_control("assist.launch_control") == 0.0);
        w.step();
        CHECK_FALSE(lc_on(r));
        drive_mode(r, 2);
        CHECK(lc_on(r));
    }
    {
        // car_hyper_n2o without the part: the mode channel does not exist, LC only follows its own channel
        io::VehicleLoadOptions none;
        none.electronics["drive_modes"] = "none";
        Rig r = make_rig(load_car(kHyperN2o, none));
        World& w = r.w();
        w.step();
        CHECK_FALSE(lc_on(r));
        write_lc(r, 1.0);
        CHECK(lc_on(r));
        write_lc(r, 0.0);
        CHECK_FALSE(lc_on(r));
    }
    {
        // car_hyper: no drive modes part at all, the same
        Rig r = make_rig(load_car(kHyper));
        World& w = r.w();
        w.step();
        CHECK_FALSE(lc_on(r));
        CHECK(w.powertrain_state(r.rig.vehicle).drive_modes_available_bits == 0);
        write_lc(r, 1.0);
        CHECK(lc_on(r));
    }
}

namespace {
// Standstill in 1st gear, throttle 1 + brake 1 for 360 ticks: how many ticks was the launch phase Armed (1)?
int armed_ticks(const VehicleDesc& d, int start_mode, int run_mode) {
    Rig r = make_rig(d, 4);
    World& w = r.w();
    base_controls(w, start_mode);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 1.0);
    w.set_control("handbrake", 0.0);
    w.set_control("assist.drive_mode", static_cast<real>(run_mode)); // NO assist.launch_control write
    w.set_control("throttle", 1.0);
    w.set_control("brake", 1.0);
    int armed = 0;
    for (int k = 0; k < 360; ++k) {
        w.step();
        if (w.powertrain_state(r.rig.vehicle).launch_phase == 1) ++armed;
    }
    return armed;
}
} // namespace

TEST_CASE("LD3: Sport arms launch control through the default alone; Normal (even when allowed) and Eco never arm", "[assist][drive_modes][vehicle][drivetrain]") {
    TempDir td("ld3");
    CHECK(armed_ticks(n2o(), 1, 2) >= 1);
    CHECK(armed_ticks(n2o(), 1, 1) == 0);
    CHECK(armed_ticks(n2o(), 1, 0) == 0);
    // Normal allows LC here, so only its default (off) keeps it from arming
    const std::string p = part_with(td, "normal_allowed.json", [](json& j) { j["modes"]["normal"]["launch_control_allowed"] = true; });
    const VehicleDesc d = load_car(kHyperN2o, with_modes(p));
    CHECK(armed_ticks(d, 1, 1) == 0);
    CHECK(armed_ticks(d, 1, 2) >= 1);
}

TEST_CASE("LD4: loader - default true contradicting allowed false, non-boolean, unknown keys, an unfitted LC", "[assist][drive_modes][io]") {
    TempDir td("ld4");
    {
        const std::string p = part_with(td, "contradict.json", [](json& j) { j["modes"]["eco"]["launch_control_default"] = true; });
        CHECK_THROWS_WITH(load_car(kHyperN2o, with_modes(p)), ContainsSubstring("launch_control_default") && ContainsSubstring("launch_control_allowed"));
    }
    {
        const std::string p = part_with(td, "notbool.json", [](json& j) { j["modes"]["sport"]["launch_control_default"] = "yes"; });
        CHECK_THROWS_WITH(load_car(kHyperN2o, with_modes(p)), ContainsSubstring("launch_control_default") && ContainsSubstring("boolean"));
    }
    {
        const std::string p = part_with(td, "unknown.json", [](json& j) { j["modes"]["sport"]["launch_control_defualt"] = true; });
        CHECK_THROWS_WITH(load_car(kHyperN2o, with_modes(p)), ContainsSubstring("unknown field"));
    }
    {
        // true with allowed true loads; the shipped values round-trip
        const VehicleDesc& d = n2o();
        REQUIRE(d.drive_modes.has_value());
        CHECK(d.drive_modes->params[0].launch_control_default == std::optional<bool>(false));
        CHECK(d.drive_modes->params[1].launch_control_default == std::optional<bool>(false));
        CHECK(d.drive_modes->params[2].launch_control_default == std::optional<bool>(true));
        CHECK(any_launch_control_default(*d.drive_modes));
    }
    {
        // a car with the assist unit's LC removed: the default loads and is ignored (no channel, nothing armed)
        io::VehicleLoadOptions o = with_modes(kPart);
        o.electronics["assists"] = kData + "/electronics/assist_unit_street.json";
        const VehicleDesc d = load_car(kHyperN2o, o);
        Rig r = make_rig(d);
        r.w().step();
        drive_mode(r, 2);
        CHECK_FALSE(lc_on(r));
    }
}

namespace {
struct DetRun {
    std::vector<std::uint64_t> hashes;
    std::uint64_t own_alloc = 0;
};

DetRun det_run(int workers, bool alloc) {
    Rig r = make_rig(n2o(), workers);
    World& w = r.w();
    base_controls(w, 1);
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
        if (k == 300) w.set_control("assist.launch_control", 1.0);
        if (k == 420) w.set_control("assist.drive_mode", 1.0);
        if (k == 600) w.set_control("assist.launch_control", 0.0);
        if (k == 720) w.set_control("assist.drive_mode", 2.0);
        w.step();
        out.hashes.push_back(w.state_hash());
    }
    if (use_alloc) {
        da::set_armed(false);
        out.own_alloc = da::own_violation_count();
    }
    return out;
}
} // namespace

TEST_CASE("LD5: 1 vs 10 workers equal hash every tick with mode changes and LC writes; zero own allocation per tick", "[assist][drive_modes][vehicle][drivetrain][determinism]") {
    const DetRun one = det_run(1, true);
    const DetRun ten = det_run(10, true);
    REQUIRE(one.hashes.size() == ten.hashes.size());
    std::size_t first_diff = one.hashes.size();
    for (std::size_t i = 0; i < one.hashes.size(); ++i)
        if (one.hashes[i] != ten.hashes[i]) {
            first_diff = i;
            break;
        }
    CHECK(first_diff == one.hashes.size());
    if (jobs::debug_alloc::hook_available()) {
        CHECK(one.own_alloc == 0);
        CHECK(ten.own_alloc == 0);
    }
}

namespace {
// Normal -> Sport at tick 5; then optionally the driver writes LC 1 (a change from the seen 0: releases the override while
// the effective enable stays on). Returns the hash after 10 ticks.
std::uint64_t hash_after(const VehicleDesc& d, bool release) {
    Rig r = make_rig(d, 1);
    World& w = r.w();
    base_controls(w, 1);
    for (int k = 0; k < 5; ++k) w.step();
    w.set_control("assist.drive_mode", 2.0);
    w.step();
    if (release) w.set_control("assist.launch_control", 1.0);
    for (int k = 0; k < 4; ++k) w.step();
    REQUIRE(lc_on(r));
    return w.state_hash();
}
} // namespace

TEST_CASE("LD6: the override state is hashed; a part without a launch_control_default folds and enables nothing", "[assist][drive_modes][vehicle][drivetrain]") {
    TempDir td("ld6");
    CHECK(hash_after(n2o(), false) != hash_after(n2o(), true));
    // stripped of every launch_control_default: Normal -> Sport leaves LC off
    const std::string p = part_with(td, "stripped.json", [](json& j) {
        for (const char* m : {"eco", "normal", "sport"}) j["modes"][m].erase("launch_control_default");
    });
    const VehicleDesc d = load_car(kHyperN2o, with_modes(p));
    CHECK_FALSE(any_launch_control_default(*d.drive_modes));
    Rig r = make_rig(d);
    base_controls(r.w(), 1);
    r.w().step();
    drive_mode(r, 2);
    CHECK_FALSE(lc_on(r));
    CHECK(r.w().get_control("assist.launch_control") == 0.0);
}
