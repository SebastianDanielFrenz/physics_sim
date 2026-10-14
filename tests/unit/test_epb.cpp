// tests/unit/test_epb.cpp - queue item Q3 / assists I8 (docs/next_steps_brief.md 2.1): the Racing drive mode (DriveMode
// Racing = 3, appended) and the electronic parking brake (electronics kind `epb`, channel `epb`, per-drive-mode `epb_mode`
// park_only | handbrake).
//
// Owner rule, verbatim (relayed by the racing_game cockpit plan): "No, unless special modes are used (racing, drift), the
// parking brake should refuse to engage while speed is significant."
//
// Law under test (VehicleInstance::step_tick, docs/data_formats/electronics.md "epb"):
//   level = channel epb >= 0.5; speed = |chassis velocity| at the tick start; mode = the effective drive mode's epb_mode,
//   else the part's epb_mode.
//   - level 0: released (state 0), refusal cleared.
//   - a refused request stays dropped while the level is held (in every mode): it engages only on a NEW rising edge.
//   - handbrake mode: state 2 = the rear handbrake rows driven at full (handbrake = max(channel, 1)) at any speed.
//   - park_only: a rising edge below max_engage_speed_mps latches (state 1, the same rows at full); at or above it the request
//     is refused (state 0, brakes untouched, snapshot epb_refused). A latched brake stays latched. A handbrake-mode
//     application that meets park_only by a mode switch is a new request: latched below the threshold, else refused.
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS] unless marked):
//   L1  the shipped part data/electronics/epb_generic.json: kind Epb, max_engage_speed_mps 2.0, epb_mode park_only;
//       rejections (unknown field, missing / non-positive / non-finite max_engage_speed_mps, unknown epb_mode) name the field;
//       a drive_modes epb_mode other than park_only / handbrake is rejected
//   L2  car_hyper_n2o's part: modes eco/normal/sport/racing (bits 0b1111); Racing: TC default sport, LC default true and
//       allowed, nitrous allowed, auto-shift up 4500 / 8100, down 2500 / 5500 rpm, close 0.04 s, epb_mode handbrake; the
//       other three modes epb_mode park_only. car_hyper's new part: the same four modes, Normal without any override.
//       A car with an epb part but no handbrake rows, and a car with two epb parts, fail to load
//   C1  channel `epb` registered (default 0) only on a car with the part (car_hyper, car_hyper_n2o); car_sedan has none;
//       snapshot assist_fitted_bits bit 7 = EPB fitted (0 on the sedan), assist_active_bits bit 7 = applied
//   E1  park_only (Normal) refuse / drop / engage on car_hyper_n2o in neutral on flat asphalt:
//       a rising edge at 2.5 m/s is refused in that tick (epb_refused, state 0); at 1.5 m/s it latches in that tick (state 1).
//       Refused at 10 m/s: the chassis position and velocity are bit-identical, every tick for 1 s, to the same run without
//       the epb write (brakes untouched). Level held while the car is braked to rest (service brake 1): state stays 0 every
//       tick (dropped, not pended). Level 0 clears the refusal; a new rising edge at rest latches (state 1) in that tick
//   E2  handbrake mode (Racing) at 20 m/s: state 2 and no refusal in the tick of the rising edge; the chassis position and
//       velocity are bit-identical every tick for 2 s to the run with the handbrake channel at 1 instead; speed after 2 s at
//       least 1 m/s below the free-coast run (non-vacuous)
//   E3  mode switch with the level held: Racing state 2 at 20 m/s -> Normal: state 0 and refused in that tick; at rest
//       Normal 1 -> Racing 2 -> Normal 1 (each in the tick of the switch); refused in Normal at 10 m/s -> Racing: stays 0
//       (dropped) until the level falls and rises again, then 2
//   E4  1 vs 10 workers: equal state hash every tick through a run with EPB requests, a refusal and mode switches; zero own
//       allocation per tick (debug_alloc)
//   E5  EPB state is hashed on a car with the part: two runs that differ only in a latched-at-rest EPB have different hashes
//       from that tick on (and equal before). [Amended before the first run: the comparison run pulls the HANDBRAKE channel at
//       the same tick instead of nothing, so both trajectories are identical (E2) and only the EPB fold can tell them apart -
//       against "nothing" the handbrake row's own stick state would move the hash and sabotage 5 could not fail E5.]
//   R1  car_hyper with its new part in Normal vs the part removed ("none"): chassis pose and velocity bit-identical every tick
//       of a 6 s WOT (amended from 3 s after the first run: the hyper did not leave 1st in 3 s) auto-shift launch (Normal overrides nothing); Racing selected: the TC channel is written to 3 (sport)
//       and assist.launch_control to 1 by the mode change
//
// SABOTAGES (temporary edits in core/src/vehicle/vehicle_instance.cpp, each reverted; results of the runs 2026-10-14):
//   1 park_only speed gate ignored (limit + 1e9)          -> E1 and E3 failed
//   2 a refused request latches once below the limit      -> E1 failed
//   3 rows applied only in state 1 (not in Handbrake)     -> E2 failed
//   4 a Handbrake -> park_only switch is not a new request -> E3 failed
//   5 EPB state (last_level, refused, state) not folded    -> E5 failed
//   Every other [epb] case passed under each sabotage; all 9 pass with the source restored.

#include "ps/backend/i_rigid_backend.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/electronics_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/drive_modes_desc.h"
#include "ps/vehicle/epb_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
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
const std::string kEpbPart = kData + "/electronics/epb_generic.json";
const std::string kN2oModes = kData + "/electronics/drive_modes_car_hyper_n2o.json";
const std::string kHyperModes = kData + "/electronics/drive_modes_car_hyper.json";
const std::string kHyper = kData + "/vehicles/car_hyper.json";
const std::string kHyperN2o = kData + "/vehicles/car_hyper_n2o.json";
const std::string kSedan = kData + "/vehicles/car_sedan.json";

// Bands (committed before the first run, see the header).
constexpr real kRefuseSpeed = 2.5;   // E1: above max_engage_speed_mps 2.0
constexpr real kEngageSpeed = 1.5;   // E1: below it
constexpr real kRefusedRunSpeed = 10.0;
constexpr real kHandbrakeSpeed = 20.0; // E2 / E3
constexpr real kMinHandbrakeEffect = 1.0; // E2: m/s below the free coast after 2 s

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_epb_" + tag);
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
    const std::string d = std::string(PS_SOURCE_DIR) + "/out/assists_i7_map_cache";
    fs::create_directories(d);
    return d;
}

VehicleDesc load_hyper(const std::string& path, io::VehicleLoadOptions o = {}) {
    static jobs::JobSystem pool(10);
    o.maps.pool = &pool;
    o.maps.cache_dir = cache_dir();
    return io::load_vehicle_json(path, o);
}

const VehicleDesc& n2o() {
    static const VehicleDesc d = load_hyper(kHyperN2o);
    return d;
}

std::string epb_part_with(const TempDir& td, const std::string& name, const std::function<void(json&)>& edit) {
    std::ifstream f(kEpbPart);
    json j = json::parse(f);
    edit(j);
    write_text(td.file(name), j.dump(1));
    return td.file(name);
}

struct Rig {
    ps_test::VehicleTestRig rig;
    World& w() { return *rig.world; }
    [[nodiscard]] const drivetrain::PowertrainSnapshot& pt() { return rig.world->powertrain_state(rig.vehicle); }
};

Rig make_rig(const VehicleDesc& d, int workers = 4) {
    Rig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0.0, workers);
    return r;
}

// TC / wing automation off, assists on, auto-shift off, the drive mode written before the first tick (no mode change at spawn).
void base_controls(World& w, int mode) {
    w.set_control("assist.drive_mode", static_cast<real>(mode));
    w.set_control("assist.traction_control", 0.0);
    w.set_control("assist.aero_control", 0.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("throttle", 0.0);
    w.set_control("brake", 0.0);
    w.set_control("handbrake", 0.0);
    w.set_control("epb", 0.0);
}

// Neutral, rolling straight at v (hubs rolling, engine running): the EPB is the only brake.
void roll_neutral(Rig& r, real v) {
    World& w = r.w();
    Motion m;
    m.linear = Vec3{v, 0, 0};
    w.backend().set_motion(r.rig.chassis, m);
    VehicleResetOptions ro;
    ro.gear = 0;
    ro.hubs_roll_with_chassis = true;
    w.reset_vehicle(r.rig.vehicle, ro);
}

Rig settled_rig(const VehicleDesc& d, int mode, real v, int workers = 4) {
    Rig r = make_rig(d, workers);
    base_controls(r.w(), mode);
    roll_neutral(r, 0.0);
    ps_test::settle(r.w(), 0.5); // drop onto the ground
    roll_neutral(r, v);
    return r;
}

struct Kin {
    Vec3 p, v, wv;
};
Kin kin(Rig& r) {
    return {r.w().backend().get_pose(r.rig.chassis).position, r.w().backend().get_motion(r.rig.chassis).linear,
            r.w().backend().get_motion(r.rig.chassis).angular};
}
bool same(const Kin& a, const Kin& b) {
    return a.p.x == b.p.x && a.p.y == b.p.y && a.p.z == b.p.z && a.v.x == b.v.x && a.v.y == b.v.y && a.v.z == b.v.z && a.wv.x == b.wv.x &&
           a.wv.y == b.wv.y && a.wv.z == b.wv.z;
}
real speed(Rig& r) { return r.w().backend().get_motion(r.rig.chassis).linear.length(); }

// Every path-valued string of a shipped vehicle file made absolute, so the edited copy loads from a temp directory.
void absolutise(json& j, const fs::path& base) {
    if (j.is_object() || j.is_array()) {
        for (auto& v : j) absolutise(v, base);
    } else if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s.size() > 5 && s.compare(s.size() - 5, 5, ".json") == 0 && s.rfind("../", 0) == 0) j = (base / s).lexically_normal().generic_string();
    }
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// L: loader.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("L1: the shipped epb part and the loader rejections", "[epb][assist][io]") {
    const io::ElectronicsPart p = io::load_electronics_json(kEpbPart);
    REQUIRE(p.kind == io::ElectronicsKind::Epb);
    CHECK(p.epb.max_engage_speed_mps == 2.0);
    CHECK(p.epb.epb_mode == EpbMode::ParkOnly);
    TempDir td("l1");
    const auto rejects = [&](const std::string& name, const std::function<void(json&)>& edit, const std::string& what) {
        const std::string f = epb_part_with(td, name, edit);
        CHECK_THROWS_WITH(io::load_electronics_json(f), ContainsSubstring(what));
    };
    rejects("unknown.json", [](json& j) { j["max_speed_mps"] = 2.0; }, "unknown electronics field 'max_speed_mps'");
    rejects("missing.json", [](json& j) { j.erase("max_engage_speed_mps"); }, "'max_engage_speed_mps' is required");
    rejects("zero.json", [](json& j) { j["max_engage_speed_mps"] = 0.0; }, "'max_engage_speed_mps' must be finite and > 0");
    rejects("neg.json", [](json& j) { j["max_engage_speed_mps"] = -1.0; }, "'max_engage_speed_mps' must be finite and > 0");
    rejects("str.json", [](json& j) { j["max_engage_speed_mps"] = "2"; }, "'max_engage_speed_mps' is required and must be a number");
    rejects("mode.json", [](json& j) { j["epb_mode"] = "drift"; }, "'epb_mode' must be \"park_only\" or \"handbrake\"");
    const std::string hb = epb_part_with(td, "hb.json", [](json& j) { j["epb_mode"] = "handbrake"; });
    CHECK(io::load_electronics_json(hb).epb.epb_mode == EpbMode::Handbrake);
    // non-finite through the code-built path (JSON cannot carry it)
    EpbDesc nan;
    nan.max_engage_speed_mps = std::numeric_limits<real>::quiet_NaN();
    CHECK_THROWS_WITH(validate_epb(nan), ContainsSubstring("max_engage_speed_mps must be finite and > 0"));
    // a drive_modes epb_mode other than park_only / handbrake
    std::ifstream f(kN2oModes);
    json dm = json::parse(f);
    dm["modes"]["sport"]["epb_mode"] = "always";
    write_text(td.file("dm.json"), dm.dump(1));
    CHECK_THROWS_WITH(io::load_electronics_json(td.file("dm.json")), ContainsSubstring("'epb_mode' must be \"park_only\" or \"handbrake\""));
}

TEST_CASE("L2: Racing and epb_mode in the shipped drive modes; attach rules", "[epb][assist][drive_modes][io][vehicle]") {
    const DriveModesDesc n = io::load_electronics_json(kN2oModes).drive_modes;
    CHECK(n.modes_bits == 0b1111);
    CHECK(n.default_mode == DriveMode::Normal);
    const DriveModeParams& rc = n.params[static_cast<std::size_t>(DriveMode::Racing)];
    CHECK(rc.traction_control_default == TractionMode::Sport);
    CHECK(rc.launch_control_default == true);
    CHECK(rc.launch_control_allowed);
    CHECK(rc.nitrous_arm_allowed);
    CHECK(*rc.up_rpm_at_zero_throttle == 4500.0);
    CHECK(*rc.up_rpm_at_full_throttle == 8100.0);
    CHECK(*rc.down_rpm_at_zero_throttle == 2500.0);
    CHECK(*rc.down_rpm_at_full_throttle == 5500.0);
    CHECK(*rc.clutch_close_time_s == 0.04);
    CHECK(rc.epb_mode == EpbMode::Handbrake);
    for (DriveMode m : {DriveMode::Eco, DriveMode::Normal, DriveMode::Sport}) CHECK(n.params[static_cast<std::size_t>(m)].epb_mode == EpbMode::ParkOnly);

    const DriveModesDesc h = io::load_electronics_json(kHyperModes).drive_modes;
    CHECK(h.modes_bits == 0b1111);
    CHECK(h.default_mode == DriveMode::Normal);
    const DriveModeParams& hn = h.params[static_cast<std::size_t>(DriveMode::Normal)];
    CHECK_FALSE(hn.pedal_bend.has_value());
    CHECK_FALSE(hn.up_rpm_at_zero_throttle.has_value());
    CHECK_FALSE(hn.up_rpm_at_full_throttle.has_value());
    CHECK_FALSE(hn.down_rpm_at_zero_throttle.has_value());
    CHECK_FALSE(hn.down_rpm_at_full_throttle.has_value());
    CHECK_FALSE(hn.clutch_close_time_s.has_value());
    CHECK_FALSE(hn.traction_control_default.has_value());
    CHECK_FALSE(hn.launch_control_default.has_value());
    CHECK(hn.launch_control_allowed);
    CHECK(hn.epb_mode == EpbMode::ParkOnly);
    CHECK(h.params[static_cast<std::size_t>(DriveMode::Racing)].epb_mode == EpbMode::Handbrake);
    for (DriveMode m : {DriveMode::Eco, DriveMode::Sport, DriveMode::Racing}) {
        const DriveModeParams& a = h.params[static_cast<std::size_t>(m)];
        const DriveModeParams& b = n.params[static_cast<std::size_t>(m)];
        CHECK(a.up_rpm_at_full_throttle == b.up_rpm_at_full_throttle);
        CHECK(a.traction_control_default == b.traction_control_default);
        CHECK(a.clutch_close_time_s == b.clutch_close_time_s);
    }

    // both hyper cars carry the part
    CHECK(n2o().epb.has_value());
    CHECK(load_hyper(kHyper).epb.has_value());
    CHECK(load_hyper(kHyper).drive_modes.has_value());
    // two epb parts
    io::VehicleLoadOptions two;
    two.electronics["epb_a"] = kEpbPart;
    two.electronics["epb_b"] = kEpbPart;
    CHECK_THROWS_WITH(io::load_vehicle_json(kSedan, two), ContainsSubstring("at most one epb per vehicle"));
    // a car without handbrake rows
    TempDir td("l2");
    std::ifstream f(kSedan);
    json car = json::parse(f);
    absolutise(car, fs::path(kSedan).parent_path());
    car["brake_system"].erase("handbrake");
    write_text(td.file("nohb.json"), car.dump(1));
    REQUIRE_NOTHROW(io::load_vehicle_json(td.file("nohb.json")));
    io::VehicleLoadOptions one;
    one.electronics["epb"] = kEpbPart;
    CHECK_THROWS_WITH(io::load_vehicle_json(td.file("nohb.json"), one), ContainsSubstring("epb needs brake_system.handbrake rows"));
    // and the VehicleInstance backstop for a code-built desc
    VehicleDesc coded = io::load_vehicle_json(kSedan);
    coded.epb = EpbDesc{};
    coded.brake_system.handbrake.wheels.clear();
    CHECK_THROWS_WITH(make_rig(coded, 1), ContainsSubstring("epb needs brake_system.handbrake rows"));
}

// ---------------------------------------------------------------------------------------------------------------
// C: channel and snapshot bits.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("C1: the epb channel and the fitted / active bits", "[epb][assist][vehicle][world]") {
    const auto has_channel = [](World& w) {
        const auto names = w.control_channel_names();
        return std::find(names.begin(), names.end(), "epb") != names.end();
    };
    {
        Rig r = make_rig(n2o(), 1);
        CHECK(has_channel(r.w()));
        CHECK(r.w().get_control("epb") == 0.0);
        r.w().step();
        CHECK((r.pt().assist_fitted_bits & 128u) != 0);
        CHECK((r.pt().assist_active_bits & 128u) == 0);
        CHECK(r.pt().epb_state == 0);
        r.w().set_control("epb", 1.0);
        r.w().step(); // at rest: latches
        CHECK(r.pt().epb_state == 1);
        CHECK((r.pt().assist_active_bits & 128u) != 0);
    }
    {
        Rig r = make_rig(load_hyper(kHyper), 1);
        CHECK(has_channel(r.w()));
    }
    {
        Rig r = make_rig(io::load_vehicle_json(kSedan), 1);
        CHECK_FALSE(has_channel(r.w()));
        r.w().step();
        CHECK((r.pt().assist_fitted_bits & 128u) == 0);
        r.w().set_control("epb", 1.0); // creates an unread channel: no effect on a car without the part
        r.w().step();
        CHECK(r.pt().epb_state == 0);
        CHECK((r.pt().assist_active_bits & 128u) == 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// E: the law.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("E1: park_only - refuse at speed, drop the refused request, engage on a new edge at rest", "[epb][assist][vehicle]") {
    {
        Rig r = settled_rig(n2o(), 1, kRefuseSpeed);
        REQUIRE(speed(r) > 2.0);
        r.w().set_control("epb", 1.0);
        r.w().step();
        CHECK(r.pt().epb_state == 0);
        CHECK(r.pt().epb_refused);
    }
    {
        Rig r = settled_rig(n2o(), 1, kEngageSpeed);
        r.w().set_control("epb", 1.0);
        r.w().step();
        CHECK(r.pt().epb_state == 1);
        CHECK_FALSE(r.pt().epb_refused);
    }
    // refused at 10 m/s: brakes untouched (bit-identical to the run without the request) for 1 s
    Rig a = settled_rig(n2o(), 1, kRefusedRunSpeed);
    Rig b = settled_rig(n2o(), 1, kRefusedRunSpeed);
    REQUIRE(same(kin(a), kin(b)));
    a.w().set_control("epb", 1.0);
    int diverged = 0, not_refused = 0;
    for (int k = 0; k < 240; ++k) {
        a.w().step();
        b.w().step();
        if (!same(kin(a), kin(b))) ++diverged;
        if (!(a.pt().epb_refused && a.pt().epb_state == 0)) ++not_refused;
    }
    CHECK(diverged == 0);
    CHECK(not_refused == 0);
    CHECK(speed(a) > 8.0); // non-vacuous: still rolling
    // the level held while the service brake stops the car: dropped, never pended
    a.w().set_control("brake", 1.0);
    int engaged = 0;
    for (int k = 0; k < 6 * 240; ++k) {
        a.w().step();
        if (a.pt().epb_state != 0) ++engaged;
    }
    CHECK(engaged == 0);
    CHECK(a.pt().epb_refused);
    CHECK(speed(a) < 0.1);
    // level 0 clears the refusal; a new rising edge at rest latches in that tick
    a.w().set_control("epb", 0.0);
    a.w().step();
    CHECK_FALSE(a.pt().epb_refused);
    CHECK(a.pt().epb_state == 0);
    a.w().set_control("epb", 1.0);
    a.w().step();
    CHECK(a.pt().epb_state == 1);
}

TEST_CASE("E2: handbrake mode (Racing) applies the rear handbrake rows at speed", "[epb][assist][vehicle]") {
    Rig a = settled_rig(n2o(), 3, kHandbrakeSpeed); // EPB
    Rig b = settled_rig(n2o(), 3, kHandbrakeSpeed); // the handbrake channel
    Rig c = settled_rig(n2o(), 3, kHandbrakeSpeed); // free coast
    REQUIRE(a.pt().drive_mode_effective == 3);
    a.w().set_control("epb", 1.0);
    b.w().set_control("handbrake", 1.0);
    a.w().step();
    b.w().step();
    c.w().step();
    CHECK(a.pt().epb_state == 2);
    CHECK_FALSE(a.pt().epb_refused);
    int diverged = same(kin(a), kin(b)) ? 0 : 1, not_applied = 0;
    for (int k = 1; k < 480; ++k) {
        a.w().step();
        b.w().step();
        c.w().step();
        if (!same(kin(a), kin(b))) ++diverged;
        if (a.pt().epb_state != 2) ++not_applied;
    }
    CHECK(diverged == 0);
    CHECK(not_applied == 0);
    INFO("EPB " << speed(a) << " coast " << speed(c));
    CHECK(speed(a) < speed(c) - kMinHandbrakeEffect);
}

TEST_CASE("E3: drive-mode switches with the level held", "[epb][assist][vehicle][drive_modes]") {
    {
        // Racing state 2 at speed -> Normal: a new park_only request at speed, refused
        Rig r = settled_rig(n2o(), 3, kHandbrakeSpeed);
        r.w().set_control("epb", 1.0);
        r.w().step();
        REQUIRE(r.pt().epb_state == 2);
        r.w().set_control("assist.drive_mode", 1.0);
        r.w().step();
        CHECK(r.pt().drive_mode_effective == 1);
        CHECK(r.pt().epb_state == 0);
        CHECK(r.pt().epb_refused);
    }
    {
        // at rest: Normal 1 -> Racing 2 -> Normal 1, each in the tick of the switch
        Rig r = settled_rig(n2o(), 1, 0.0);
        r.w().set_control("epb", 1.0);
        r.w().step();
        CHECK(r.pt().epb_state == 1);
        r.w().set_control("assist.drive_mode", 3.0);
        r.w().step();
        CHECK(r.pt().epb_state == 2);
        r.w().set_control("assist.drive_mode", 1.0);
        r.w().step();
        CHECK(r.pt().epb_state == 1);
        CHECK_FALSE(r.pt().epb_refused);
    }
    {
        // refused in Normal at 10 m/s -> Racing: stays dropped until the level falls and rises again
        Rig r = settled_rig(n2o(), 1, kRefusedRunSpeed);
        r.w().set_control("epb", 1.0);
        r.w().step();
        REQUIRE(r.pt().epb_refused);
        r.w().set_control("assist.drive_mode", 3.0);
        int applied = 0;
        for (int k = 0; k < 24; ++k) {
            r.w().step();
            if (r.pt().epb_state != 0) ++applied;
        }
        CHECK(r.pt().drive_mode_effective == 3);
        CHECK(applied == 0);
        r.w().set_control("epb", 0.0);
        r.w().step();
        r.w().set_control("epb", 1.0);
        r.w().step();
        CHECK(r.pt().epb_state == 2);
    }
}

namespace {
struct DetRun {
    std::vector<std::uint64_t> hashes;
    std::uint64_t own_alloc = 0;
};

DetRun det_run(int workers) {
    Rig r = settled_rig(n2o(), 1, kRefusedRunSpeed, workers);
    World& w = r.w();
    namespace da = jobs::debug_alloc;
    const bool use_alloc = da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    DetRun out;
    for (int k = 0; k < 6 * 240; ++k) {
        if (k == 10) w.set_control("epb", 1.0);              // refused at speed
        if (k == 120) w.set_control("assist.drive_mode", 3.0); // still dropped
        if (k == 200) w.set_control("epb", 0.0);
        if (k == 220) w.set_control("epb", 1.0);              // handbrake mode at speed
        if (k == 600) w.set_control("assist.drive_mode", 1.0); // -> park_only
        if (k == 640) w.set_control("brake", 1.0);
        if (k == 1100) w.set_control("epb", 0.0);
        if (k == 1120) w.set_control("epb", 1.0);             // latched at rest
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

TEST_CASE("E4: 1 vs 10 workers - equal hash every tick through EPB requests, a refusal and mode switches; zero own allocation", "[epb][assist][vehicle][determinism]") {
    const DetRun one = det_run(1);
    const DetRun ten = det_run(10);
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

TEST_CASE("E5: the EPB state is hashed on a car with the part", "[epb][assist][vehicle][determinism][long]") {
    Rig a = settled_rig(n2o(), 1, 0.0);
    Rig b = settled_rig(n2o(), 1, 0.0);
    constexpr int kLatch = 60;
    int equal_before = 0, differ_after = 0, same_kin = 0;
    for (int k = 0; k < 240; ++k) {
        if (k == kLatch) {
            a.w().set_control("epb", 1.0);       // latched at rest
            b.w().set_control("handbrake", 1.0); // the same rows, no EPB state
        }
        a.w().step();
        b.w().step();
        if (same(kin(a), kin(b))) ++same_kin;
        if (k < kLatch && a.w().state_hash() == b.w().state_hash()) ++equal_before;
        if (k >= kLatch && a.w().state_hash() != b.w().state_hash()) ++differ_after;
    }
    CHECK(a.pt().epb_state == 1);
    CHECK(same_kin == 240); // only the EPB fold tells them apart
    CHECK(equal_before == kLatch);
    CHECK(differ_after == 240 - kLatch);
}

// ---------------------------------------------------------------------------------------------------------------
// R: car_hyper's new part.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("R1: car_hyper - Normal is inert; Racing writes the TC and LC channels", "[epb][assist][drive_modes][vehicle]") {
    io::VehicleLoadOptions none;
    none.electronics["drive_modes"] = "none";
    none.electronics["epb"] = "none";
    const VehicleDesc with = load_hyper(kHyper);
    const VehicleDesc without = load_hyper(kHyper, none);
    REQUIRE_FALSE(without.drive_modes.has_value());
    REQUIRE_FALSE(without.epb.has_value());
    const auto launch = [](const VehicleDesc& d) {
        Rig r = make_rig(d, 4);
        World& w = r.w();
        w.set_control("assist.traction_control", 1.0);
        w.set_control("assist.aero_control", 0.0);
        w.set_control("handbrake", 1.0);
        w.set_control("gear_request", 1.0);
        ps_test::settle(w, 0.5);
        w.set_control("handbrake", 0.0);
        w.set_control("assist.auto_shift", 1.0);
        w.set_control("gear_request", drivetrain::kGearRequestNone);
        w.set_control("throttle", 1.0);
        std::vector<Kin> out;
        int max_gear = 0;
        for (int k = 0; k < 6 * 240; ++k) { // 6 s: in 3 s the hyper stays in 1st (amended after the first run)
            w.step();
            out.push_back(kin(r));
            max_gear = std::max(max_gear, static_cast<int>(r.pt().gear));
        }
        return std::make_pair(out, max_gear);
    };
    const auto [ka, ga] = launch(with);
    const auto [kb, gb] = launch(without);
    REQUIRE(ka.size() == kb.size());
    int diverged = 0;
    for (std::size_t i = 0; i < ka.size(); ++i)
        if (!same(ka[i], kb[i])) ++diverged;
    CHECK(diverged == 0);
    CHECK(ga >= 2); // non-vacuous: the launch really shifted
    CHECK(ka.back().v.length() > 15.0);

    Rig r = make_rig(with, 1);
    World& w = r.w();
    w.step();
    CHECK(w.get_control("assist.traction_control") == 1.0);
    CHECK(w.get_control("assist.launch_control") == 0.0);
    w.set_control("assist.drive_mode", 3.0);
    w.step();
    CHECK(r.pt().drive_mode_effective == 3);
    CHECK(w.get_control("assist.traction_control") == 3.0);
    CHECK(w.get_control("assist.launch_control") == 1.0);
}
