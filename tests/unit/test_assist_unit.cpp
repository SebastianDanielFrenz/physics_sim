// tests/unit/test_assist_unit.cpp - assists plan I1 (vault PHYS-074): the assist_unit electronics part, graded
// Street / Sport / Racing, with the traction-control mode table (channel assist.traction_control: 0 Off, 1 On,
// 2 Relaxed, 3 Sport, 4 Drift).
//
//   U*  pure logic: channel value -> requested code, the unavailable-mode fallback (V7), mode params resolution
//   P*  the step function's two new per-mode knobs (min_fraction floor, ignition_cut) and the loader's shapes
//   R*  loader rejections, each asserting its own message
//   A*  attachment: vehicle-file entry, user layer add/replace/none, extends, fitted=false wins over everything
//   B*  snapshot fields, channel default, fallback through the World (V7)
//   H*  V0 bit-identity: a car with a Street unit hashes like a car without one; mode 1 of a Sport unit is physically
//       identical to a car without a unit; no unit = today
//   D*  1 vs 10 workers identical hash with a unit, zero own allocation per tick
//   V6  [long] mode ordering on car_hyper with a Racing unit attached in the test
//
// BANDS COMMITTED BEFORE THE FIRST RUN (assists plan B6, brief assists_i1):
//   V0  Street unit vs no unit: state hash equal on every tick; Sport unit at mode 1 vs no unit: chassis pose, motion
//       and every wheel omega bit-equal on every tick; ignition_cut=false differs from the mode that cuts.
//   V6  car_hyper + Racing unit, grass_rg (lambda_mu 0.63, crr 0.08, tests/support/fixtures/assists), 5 % slope,
//       WOT in 1st gear. Mean rear |kappa| over t in [0.3, 3.0] s after the throttle step must satisfy
//       On < Relaxed < Sport < Drift, each step larger by at least 0.01 (absolute kappa). The known crawl bug
//       (PHYS-074: the auto-clutch locks onto spinning wheels below ~3.1 m/s in 1st, then TC fights the lock) is
//       avoided by starting ROLLING at 8 m/s with the clutch locked (VehicleResetOptions::clutch_locked): the rim
//       speed never goes below the 3.1 m/s anti-stall equivalent, so no lock-on-spin cycle can start.
//       The plan's Drift yaw-rate step-steer check (> 1.3x On) is NOT part of I1 (open point).
//   V7  Street unit codes 2-4 -> effective On; Off stays Off; a unit without Off maps 0 to its least-intervening mode;
//       available bits per grade: street 0b00011, sport 0b01111, racing 0b11111; not fitted 0.
//
// Every guard has a recorded sabotage (out/assists_i1_progress.md in the worktree).

#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/electronics_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"
#include "ps/vehicle/assist_unit_desc.h"
#include "ps/vehicle/traction_control.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <variant>
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
const std::string kStreet = kData + "/electronics/assist_unit_street.json";
const std::string kSport = kData + "/electronics/assist_unit_sport.json";
const std::string kRacing = kData + "/electronics/assist_unit_racing.json";

struct TempDir {
    fs::path path;
    explicit TempDir(const std::string& tag) {
        path = fs::temp_directory_path() / ("ps_assist_unit_" + tag);
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

void absolutise(json& j) {
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        if (s.rfind("../", 0) == 0) j = kData + "/" + s.substr(3);
    } else if (j.is_array() || j.is_object()) {
        for (auto& e : j) absolutise(e);
    }
}

json vehicle_json(const std::string& name) {
    std::ifstream f(kData + "/vehicles/" + name);
    REQUIRE(f.good());
    json j = json::parse(f);
    absolutise(j);
    return j;
}

std::string write_vehicle(const TempDir& d, const std::string& fname, const json& j) {
    write_text(d.file(fname), j.dump(1));
    return d.file(fname);
}

VehicleDesc sedan(const io::VehicleLoadOptions& o = {}) { return io::load_vehicle_json(kData + "/vehicles/car_sedan.json", o); }

io::VehicleLoadOptions with_unit(const std::string& path) {
    io::VehicleLoadOptions o;
    o.electronics["assist"] = path;
    return o;
}

const std::string kUnitHead = R"({"format":"physics_sim.electronics/1","kind":"assist_unit",)";

// A low-grip sedan so TC actually intervenes in a short WOT launch (same device as test_traction_control.cpp).
VehicleDesc slippery(VehicleDesc d) {
    for (auto& w : d.wheels) w.tyre.lambda_mux *= 0.25;
    return d;
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------
// U: pure logic.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("U1: channel value to requested mode code", "[assist][vehicle]") {
    CHECK(traction_mode_requested(0.0) == TractionMode::Off);
    CHECK(traction_mode_requested(0.5) == TractionMode::Off); // today's rule: value > 0.5 is on
    CHECK(traction_mode_requested(0.51) == TractionMode::On);
    CHECK(traction_mode_requested(1.0) == TractionMode::On);
    CHECK(traction_mode_requested(1.49) == TractionMode::On);
    CHECK(traction_mode_requested(1.51) == TractionMode::Relaxed);
    CHECK(traction_mode_requested(3.0) == TractionMode::Sport);
    CHECK(traction_mode_requested(4.0) == TractionMode::Drift);
    CHECK(traction_mode_requested(9.0) == TractionMode::Drift);
    CHECK(traction_mode_requested(-3.0) == TractionMode::Off);
    CHECK(traction_mode_requested(std::numeric_limits<real>::quiet_NaN()) == TractionMode::Off);
    CHECK(traction_mode_requested(std::numeric_limits<real>::infinity()) == TractionMode::Off);
}

TEST_CASE("U2: unavailable-mode fallback is the nearest MORE intervening mode, Off never substituted", "[assist][vehicle]") {
    const auto bits = [](std::initializer_list<TractionMode> l) {
        std::uint8_t b = 0;
        for (TractionMode m : l) b = static_cast<std::uint8_t>(b | traction_mode_bit(m));
        return b;
    };
    using M = TractionMode;
    // rank order: On 4 > Relaxed 3 > Sport 2 > Drift 1 > Off 0
    CHECK(traction_mode_rank(M::On) > traction_mode_rank(M::Relaxed));
    CHECK(traction_mode_rank(M::Relaxed) > traction_mode_rank(M::Sport));
    CHECK(traction_mode_rank(M::Sport) > traction_mode_rank(M::Drift));
    CHECK(traction_mode_rank(M::Drift) > traction_mode_rank(M::Off));

    const std::uint8_t street = bits({M::Off, M::On});
    for (M r : {M::Relaxed, M::Sport, M::Drift}) CHECK(resolve_traction_mode(street, r) == M::On);
    CHECK(resolve_traction_mode(street, M::Off) == M::Off);
    CHECK(resolve_traction_mode(street, M::On) == M::On);

    const std::uint8_t sport = bits({M::Off, M::On, M::Relaxed, M::Sport});
    CHECK(resolve_traction_mode(sport, M::Drift) == M::Sport); // nearest more intervening, not On
    CHECK(resolve_traction_mode(sport, M::Sport) == M::Sport);

    // {Off, On, Sport}: Relaxed -> On (more intervening), Drift -> Sport
    const std::uint8_t gap = bits({M::Off, M::On, M::Sport});
    CHECK(resolve_traction_mode(gap, M::Relaxed) == M::On);
    CHECK(resolve_traction_mode(gap, M::Drift) == M::Sport);

    // {Off, Relaxed, Drift}: Sport -> Relaxed; On -> nothing more intervening, so the MOST intervening available
    const std::uint8_t odd = bits({M::Off, M::Relaxed, M::Drift});
    CHECK(resolve_traction_mode(odd, M::Sport) == M::Relaxed);
    CHECK(resolve_traction_mode(odd, M::On) == M::Relaxed);
    CHECK(resolve_traction_mode(bits({M::Off, M::Drift}), M::On) == M::Drift); // never Off

    // a unit without Off: Off maps to the LEAST intervening available mode
    CHECK(resolve_traction_mode(bits({M::On, M::Sport}), M::Off) == M::Sport);
    CHECK(resolve_traction_mode(bits({M::On, M::Relaxed, M::Sport, M::Drift}), M::Off) == M::Drift);
    CHECK(resolve_traction_mode(bits({M::On}), M::Off) == M::On);
    CHECK(resolve_traction_mode(bits({M::On, M::Sport}), M::Relaxed) == M::On);

    // not fitted: nothing available, always Off
    for (int r = 0; r < kTractionModeCount; ++r) CHECK(resolve_traction_mode(0, static_cast<M>(r)) == M::Off);
}

TEST_CASE("U3: mode descs are the car's block plus overrides; On and Off are the car's own", "[assist][vehicle]") {
    TractionControlDesc base;
    base.target_slip = 0.07;
    base.kp = 3.0;
    AssistUnitDesc u;
    u.grade = AssistGrade::Racing;
    u.has_traction_control = true;
    u.traction_control.modes_bits = 0b11111;
    u.traction_control.params[3].target_slip = 0.2;
    u.traction_control.params[3].min_fraction = 0.25;
    u.traction_control.params[3].ignition_cut = false;
    const auto descs = resolve_traction_mode_descs(base, u);
    // On (1) and Off (0): untouched, same bytes as the base
    for (int m : {0, 1}) {
        CHECK(descs[m].target_slip == base.target_slip);
        CHECK(descs[m].kp == base.kp);
        CHECK(descs[m].min_fraction == 0.0);
        CHECK(descs[m].ignition_cut);
    }
    // Sport (3): overridden fields, the rest inherits the car's value
    CHECK(descs[3].target_slip == 0.2);
    CHECK(descs[3].kp == 3.0);
    CHECK(descs[3].min_fraction == 0.25);
    CHECK_FALSE(descs[3].ignition_cut);
    // an offered mode that resolves to an invalid controller is rejected naming the mode
    u.traction_control.params[4].kp = -1.0;
    CHECK_THROWS_WITH(resolve_traction_mode_descs(base, u), ContainsSubstring("traction mode 'drift'"));
    // an unoffered mode is not validated
    u.traction_control.modes_bits = 0b01111;
    CHECK_NOTHROW(resolve_traction_mode_descs(base, u));
    // no unit: all five entries are the base
    const auto none = resolve_traction_mode_descs(base, std::nullopt);
    for (int m = 0; m < kTractionModeCount; ++m) CHECK(none[m].target_slip == base.target_slip);
}

// ---------------------------------------------------------------------------------------------------------------
// P: the step function's per-mode knobs.
// ---------------------------------------------------------------------------------------------------------------
namespace {
std::array<TractionWheelState, 4> spin_sensors(real speed, real driven_speed) {
    return {{{speed,speed,3500,false,true}, {speed,speed,3500,false,true}, {driven_speed,speed,3500,true,true}, {driven_speed,speed,3500,true,true}}};
}
real run_spin(const TractionControlDesc& d, real* out_min = nullptr) {
    TractionControlState s;
    auto w = spin_sensors(10, 10);
    step_traction_control(d, s, w, 10, 1, 0, true, .01);
    real lo = 1;
    w = spin_sensors(10, 16); // 60 % common wheelspin on the driven axle
    for (int i = 0; i < 300; ++i) {
        step_traction_control(d, s, w, 10, 1, 0, true, .01);
        lo = std::min(lo, s.torque_fraction);
    }
    if (out_min) *out_min = lo;
    return s.torque_fraction;
}
} // namespace

TEST_CASE("P1: min_fraction is a torque floor; the default 0 is today's controller", "[assist][vehicle]") {
    TractionControlDesc d;
    real lo0 = 1;
    run_spin(d, &lo0);
    CHECK(lo0 < 0.01); // today: the fraction goes to ~0 under heavy spin

    d.min_fraction = 0.35;
    real lo1 = 1;
    const real fin = run_spin(d, &lo1);
    CHECK(lo1 >= 0.35 - 1e-12);
    CHECK(fin == 0.35); // sits exactly on the floor under persistent spin

    // the default desc and an explicit 0 floor are the same controller, bit for bit
    TractionControlDesc d0, dz;
    dz.min_fraction = 0.0;
    CHECK(run_spin(d0) == run_spin(dz));
    d.min_fraction = 1.0;
    CHECK_THROWS(validate_traction_control(d));
    d.min_fraction = -0.1;
    CHECK_THROWS(validate_traction_control(d));
}

// ---------------------------------------------------------------------------------------------------------------
// R: loader shapes and rejections.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("R0: the shipped grade files load with the approved grade mapping", "[assist][io]") {
    const io::ElectronicsPart st = io::load_electronics_json(kStreet);
    const io::ElectronicsPart sp = io::load_electronics_json(kSport);
    const io::ElectronicsPart ra = io::load_electronics_json(kRacing);
    CHECK(st.kind == io::ElectronicsKind::AssistUnit);
    CHECK(st.unit.grade == AssistGrade::Street);
    CHECK(sp.unit.grade == AssistGrade::Sport);
    CHECK(ra.unit.grade == AssistGrade::Racing);
    CHECK(st.unit.traction_control.modes_bits == 0b00011);
    CHECK(sp.unit.traction_control.modes_bits == 0b01111);
    CHECK(ra.unit.traction_control.modes_bits == 0b11111);
    CHECK(st.unit.traction_control.default_mode == TractionMode::On);
    CHECK(*ra.unit.traction_control.params[4].target_slip == 0.35);
    CHECK(*ra.unit.traction_control.params[3].min_fraction == 0.15);
    CHECK(*ra.unit.traction_control.params[3].ignition_cut == true); // racing_game PR #5: Sport retards at the boost
    CHECK(*ra.unit.traction_control.params[4].ignition_cut == false);
    CHECK(ra.name == "Racing Assists");
    // the grade table: which blocks a grade may carry
    CHECK_FALSE(assist_grade_info(AssistGrade::Street).allows_launch_control);
    CHECK_FALSE(assist_grade_info(AssistGrade::Street).allows_aero_control);
    CHECK(assist_grade_info(AssistGrade::Sport).allows_launch_control);
    CHECK_FALSE(assist_grade_info(AssistGrade::Sport).allows_aero_control);
    CHECK(assist_grade_info(AssistGrade::Racing).allows_launch_control);
    CHECK(assist_grade_info(AssistGrade::Racing).allows_aero_control);
}

TEST_CASE("R1: assist_unit rejections", "[assist][io]") {
    TempDir td("r1");
    const auto load = [&](const std::string& body) {
        write_text(td.file("p.json"), kUnitHead + body);
        return io::load_electronics_json(td.file("p.json"));
    };
    const std::string tc_ok = R"("traction_control":{"modes":["off","on"],"default_mode":"on"})";
    CHECK_THROWS_WITH(load(R"("name":"x"})"), ContainsSubstring("'grade' is required"));
    CHECK_THROWS_WITH(load(R"("grade":"gold"})"), ContainsSubstring("unknown grade \"gold\""));
    CHECK_THROWS_WITH(load(R"("grade":"street","bogus":1})"), ContainsSubstring("unknown electronics field 'bogus'"));
    // grade table: blocks a grade does not allow / allows but are not loaded yet
    CHECK_THROWS_WITH(load(R"("grade":"street","launch_control":{}})"), ContainsSubstring("grade 'street' does not allow 'launch_control'"));
    CHECK_THROWS_WITH(load(R"("grade":"sport","aero_control":{}})"), ContainsSubstring("grade 'sport' does not allow 'aero_control'"));
    CHECK_THROWS_WITH(load(R"("grade":"racing","launch_control":{}})"), ContainsSubstring("exactly one of 'launch_rpm'"));
    CHECK(load(R"("grade":"racing","launch_control":{"launch_rpm":4500}})").unit.launch_control.has_value()); // assists I4 loads it (tests/unit/test_assist_launch.cpp)
    CHECK(load(R"("grade":"racing","aero_control":{}})").unit.aero_control.has_value()); // assists I5 loads it (tests/unit/test_assist_aero.cpp)
    // traction_control block
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":[],"default_mode":"on"}})"), ContainsSubstring("'modes' is required"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"default_mode":"on"}})"), ContainsSubstring("'modes' is required"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":["off","turbo"],"default_mode":"on"}})"),
                      ContainsSubstring("'modes' entries must be one of off, on, relaxed, sport, drift"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":["on","on"],"default_mode":"on"}})"), ContainsSubstring("duplicate mode 'on'"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":["off"],"default_mode":"off"}})"),
                      ContainsSubstring("at least one mode other than 'off'"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":["off","on"]}})"), ContainsSubstring("'default_mode' is required"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":["off","on"],"default_mode":"sport"}})"),
                      ContainsSubstring("'default_mode' \"sport\" is not one of the listed 'modes'"));
    CHECK_THROWS_WITH(load(R"("grade":"street","traction_control":{"modes":["off","on"],"default_mode":"on","extra":1}})"),
                      ContainsSubstring("traction_control: unknown field 'extra'"));
    const std::string tcs = R"("grade":"racing","traction_control":{"modes":["off","on","relaxed","sport","drift"],"default_mode":"on",)";
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"on":{"kp":1}}}})"), ContainsSubstring("'on' takes no parameters"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"off":{}}}})"), ContainsSubstring("'off' takes no parameters"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"turbo":{}}}})"), ContainsSubstring("unknown mode 'turbo'"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"sport":{"kp":0}}}})"), ContainsSubstring("'kp' must be finite and > 0"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"sport":{"target_slip":1.2}}}})"), ContainsSubstring("'target_slip' must be below 1"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"sport":{"min_fraction":1.0}}}})"), ContainsSubstring("'min_fraction' must be in [0, 1)"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"sport":{"ignition_cut":1}}}})"), ContainsSubstring("'ignition_cut' must be a boolean"));
    CHECK_THROWS_WITH(load(tcs + R"("mode_params":{"sport":{"slip_speed_floor_m_s":1}}}})"), ContainsSubstring("mode_params.sport: unknown field 'slip_speed_floor_m_s'"));
    // params for a mode the unit does not list
    CHECK_THROWS_WITH(load(R"("grade":"sport","traction_control":{"modes":["off","on"],"default_mode":"on","mode_params":{"sport":{"kp":1}}}})"),
                      ContainsSubstring("mode_params for 'sport' but 'sport' is not in 'modes'"));
    // a unit without a TC block is valid (TC stays the car's)
    const io::ElectronicsPart bare = load(R"("grade":"street"})");
    CHECK_FALSE(bare.unit.has_traction_control);
    (void)tc_ok;
}

TEST_CASE("R2: extends works for assist units", "[assist][io]") {
    TempDir td("r2");
    write_text(td.file("child.json"), R"({"extends":")" + kSport + R"(","traction_control":{"mode_params":{"sport":{"target_slip":0.2}}}})");
    const io::ElectronicsPart p = io::load_electronics_json(td.file("child.json"));
    CHECK(*p.unit.traction_control.params[3].target_slip == 0.2); // patched
    CHECK(*p.unit.traction_control.params[3].kp == 1.5);          // inherited
    CHECK(p.unit.traction_control.modes_bits == 0b01111);
}

// ---------------------------------------------------------------------------------------------------------------
// A: attachment and fitment.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("A1: attachment - user layer add, replace and remove; vehicle-file entry equals the user layer", "[assist][io][vehicle]") {
    TempDir td("a1");
    const VehicleDesc plain = sedan();
    CHECK_FALSE(plain.assist_unit.has_value());

    // user layer ADDS an entry
    const VehicleDesc added = sedan(with_unit(kSport));
    REQUIRE(added.assist_unit.has_value());
    CHECK(added.assist_unit->grade == AssistGrade::Sport);
    CHECK(added.controllers.size() == plain.controllers.size()); // a unit adds no controller

    // the same part written into the vehicle file
    json j = vehicle_json("car_sedan.json");
    j["controllers"].push_back({{"id", "assist"}, {"type", "electronics"}, {"ref", kSport}});
    const std::string vf = write_vehicle(td, "v.json", j);
    const VehicleDesc filed = io::load_vehicle_json(vf);
    REQUIRE(filed.assist_unit.has_value());
    CHECK(filed.assist_unit->traction_control.modes_bits == added.assist_unit->traction_control.modes_bits);
    CHECK(*filed.assist_unit->traction_control.params[3].kp == *added.assist_unit->traction_control.params[3].kp);

    // user layer REPLACES the file's entry
    io::VehicleLoadOptions rep;
    rep.electronics["assist"] = kRacing;
    const VehicleDesc replaced = io::load_vehicle_json(vf, rep);
    CHECK(replaced.assist_unit->grade == AssistGrade::Racing);

    // "none" REMOVES it
    io::VehicleLoadOptions rem;
    rem.electronics["assist"] = "none";
    CHECK_FALSE(io::load_vehicle_json(vf, rem).assist_unit.has_value());
    io::VehicleLoadOptions rem2;
    rem2.electronics["nothing"] = "none";
    CHECK_THROWS_WITH(sedan(rem2), ContainsSubstring("no electronics entry with id 'nothing' to remove"));

    // at most one assist_unit
    io::VehicleLoadOptions two = with_unit(kSport);
    two.electronics["assist2"] = kRacing;
    CHECK_THROWS_WITH(sedan(two), ContainsSubstring("at most one assist_unit per vehicle"));
}

TEST_CASE("A2: the car's traction_control block is mode On; fitted=false wins over everything", "[assist][io][vehicle]") {
    TempDir td("a2");
    json j = vehicle_json("car_sedan.json");
    j["traction_control"] = {{"target_slip", 0.09}, {"fitted", false}};
    const std::string vf = write_vehicle(td, "v.json", j);
    const VehicleDesc d = io::load_vehicle_json(vf, with_unit(kRacing));
    REQUIRE(d.assist_unit.has_value());
    CHECK_FALSE(d.traction_control.fitted);
    CHECK(traction_modes_available_bits(d.assist_unit, d.traction_control.fitted) == 0);
    CHECK(traction_default_channel_value(d.traction_control, d.assist_unit) == 0.0);
    // modes are resolved against the car's own block: Relaxed inherits kp/target of the car where unset
    const auto descs = resolve_traction_mode_descs(d.traction_control, d.assist_unit);
    CHECK(descs[1].target_slip == 0.09);

    // through the World: whatever the channel says, the effective mode is Off and the snapshot offers nothing
    auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500, {2, .4, .15}, .5, 0, 1);
    for (real code : {1.0, 2.0, 4.0}) {
        rig.world->set_control("assist.traction_control", code);
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        CHECK(pt.traction_mode_effective == 0);
        CHECK(pt.traction_modes_available_bits == 0);
        CHECK(pt.traction_mode_requested == static_cast<std::uint8_t>(code));
        CHECK((pt.assist_active_bits & 8u) == 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// B: snapshot, default and fallback through the World (V7).
// ---------------------------------------------------------------------------------------------------------------
namespace {
struct ModeProbe {
    std::uint8_t requested, effective, bits;
    bool active;
};
ModeProbe probe_mode(World& w, VehicleId v, std::optional<real> code) {
    if (code) w.set_control("assist.traction_control", *code);
    w.step();
    const auto& pt = w.powertrain_state(v);
    return {pt.traction_mode_requested, pt.traction_mode_effective, pt.traction_modes_available_bits, (pt.assist_active_bits & 8u) != 0};
}
} // namespace

TEST_CASE("B1: snapshot fields, channel default and the V7 fallback per grade", "[assist][vehicle]") {
    struct Row { const char* path; std::uint8_t bits; };
    for (const Row row : {Row{"street", 0b00011}, Row{"sport", 0b01111}, Row{"racing", 0b11111}}) {
        CAPTURE(row.path);
        const std::string path = kData + "/electronics/assist_unit_" + row.path + ".json";
        auto rig = ps_test::build_vehicle_rig_from_desc(sedan(with_unit(path)), 1500, {2, .4, .15}, .5, 0, 1);
        World& w = *rig.world;
        // the channel's default is the unit's default_mode (On): nobody touched the channel
        const ModeProbe d = probe_mode(w, rig.vehicle, std::nullopt);
        CHECK(d.bits == row.bits);
        CHECK(d.requested == 1);
        CHECK(d.effective == 1);
        CHECK(d.active);
        CHECK(w.get_control("assist.traction_control") == 1.0);
        // V7: Off stays Off, every offered code is honoured, an unoffered one falls back to a MORE intervening mode
        CHECK(probe_mode(w, rig.vehicle, 0.0).effective == 0);
        CHECK_FALSE(probe_mode(w, rig.vehicle, 0.0).active);
        for (int code = 1; code <= 4; ++code) {
            const ModeProbe p = probe_mode(w, rig.vehicle, static_cast<real>(code));
            CHECK(p.requested == code);
            const bool offered = (row.bits >> code) & 1;
            if (offered) {
                CHECK(p.effective == code);
            } else if (std::string(row.path) == "street") {
                CHECK(p.effective == 1); // codes 2-4 -> On
            } else {
                CHECK(p.effective == 3); // sport grade, Drift -> Sport (the nearest more intervening)
            }
            CHECK(p.active);
        }
    }
}

TEST_CASE("B2: a car without a unit offers Off and On; codes 2-4 are On; defaults unchanged", "[assist][vehicle]") {
    auto rig = ps_test::build_vehicle_rig_from_desc(sedan(), 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    const ModeProbe d = probe_mode(w, rig.vehicle, std::nullopt);
    CHECK(d.bits == 0b00011);
    CHECK(d.effective == 0); // car_sedan's TC block defaults to disabled
    CHECK(w.get_control("assist.traction_control") == 0.0);
    for (int code = 2; code <= 4; ++code) {
        const ModeProbe p = probe_mode(w, rig.vehicle, static_cast<real>(code));
        CHECK(p.effective == 1);
        CHECK(p.requested == code);
    }
    CHECK(probe_mode(w, rig.vehicle, 0.5).effective == 0); // today's rule: > 0.5
    CHECK(probe_mode(w, rig.vehicle, 0.6).effective == 1);
}

TEST_CASE("B3: a unit without Off maps Off to its least intervening mode; default mode is the channel default", "[assist][vehicle]") {
    TempDir td("b3");
    write_text(td.file("u.json"), kUnitHead + R"("grade":"sport","traction_control":{"modes":["on","sport"],"default_mode":"sport",
              "mode_params":{"sport":{"target_slip":0.18}}}})");
    auto rig = ps_test::build_vehicle_rig_from_desc(sedan(with_unit(td.file("u.json"))), 1500, {2, .4, .15}, .5, 0, 1);
    World& w = *rig.world;
    const ModeProbe d = probe_mode(w, rig.vehicle, std::nullopt);
    CHECK(d.bits == 0b01010);
    CHECK(d.requested == 3);
    CHECK(d.effective == 3);
    CHECK(w.get_control("assist.traction_control") == 3.0);
    const ModeProbe off = probe_mode(w, rig.vehicle, 0.0);
    CHECK(off.requested == 0);
    CHECK(off.effective == 3); // TC cannot be disabled on this unit: Off -> the least intervening mode
    CHECK(probe_mode(w, rig.vehicle, 2.0).effective == 1);
    CHECK(probe_mode(w, rig.vehicle, 4.0).effective == 3);
}

// ---------------------------------------------------------------------------------------------------------------
// H: V0 bit-identity.
// ---------------------------------------------------------------------------------------------------------------
namespace {
struct Trace {
    std::vector<std::uint64_t> hashes;
    std::vector<Vec3> pos, vel;
    std::vector<real> omega_sum, fraction;
    std::uint64_t own_violations = 0;
    real mean_slip = 0;
    std::uint8_t effective = 0;
    real min_fraction = 1;
};

Trace run_sedan(const VehicleDesc& desc, std::optional<real> code, int workers = 1, int ticks = 600, bool alloc = false) {
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0, workers);
    World& w = *rig.world;
    if (code) w.set_control("assist.traction_control", *code); // before the settle: the TC state history is part of the comparison
    ps_test::settle(w, 0.5);
    w.set_control("gear_request", 1);
    w.set_control("throttle", 1);
    namespace da = jobs::debug_alloc;
    const bool use_alloc = alloc && da::hook_available();
    if (use_alloc) {
        da::reset();
        da::set_armed(true);
    }
    Trace t;
    for (int i = 0; i < ticks; ++i) {
        w.step();
        t.hashes.push_back(w.state_hash());
        t.pos.push_back(w.get_pose(rig.chassis).position);
        t.vel.push_back(w.get_motion(rig.chassis).linear);
        real om = 0;
        for (std::size_t k = 0; k < desc.wheels.size(); ++k) om += w.wheel_omega(rig.vehicle, k) * static_cast<real>(k + 1);
        t.omega_sum.push_back(om);
        const auto& pt = w.powertrain_state(rig.vehicle);
        t.fraction.push_back(pt.traction_torque_fraction);
        t.min_fraction = std::min(t.min_fraction, pt.traction_torque_fraction);
        t.effective = pt.traction_mode_effective;
        for (std::size_t k : {std::size_t(0), std::size_t(1)}) t.mean_slip += std::abs(w.wheel_telemetry(rig.vehicle, k).slip_ratio) / (2.0 * ticks);
    }
    if (use_alloc) {
        da::set_armed(false);
        t.own_violations = da::own_violation_count();
    }
    return t;
}

bool same_physics(const Trace& a, const Trace& b) {
    if (a.pos.size() != b.pos.size()) return false;
    for (std::size_t i = 0; i < a.pos.size(); ++i) {
        if (a.pos[i].x != b.pos[i].x || a.pos[i].y != b.pos[i].y || a.pos[i].z != b.pos[i].z) return false;
        if (a.vel[i].x != b.vel[i].x || a.vel[i].y != b.vel[i].y || a.vel[i].z != b.vel[i].z) return false;
        if (a.omega_sum[i] != b.omega_sum[i] || a.fraction[i] != b.fraction[i]) return false;
    }
    return true;
}

} // namespace

TEST_CASE("H1: V0 - mode On is today's TC bit for bit; a Street unit hashes like no unit", "[assist][vehicle][determinism]") {
    // "Today": a car without a unit, TC switched on through the channel.
    const Trace today = run_sedan(slippery(sedan()), 1.0);
    REQUIRE(today.min_fraction < 0.5); // the scenario really makes TC intervene (else the comparison proves nothing)
    // ... which is the same motion as the TC.enabled-in-the-desc way (that desc hashes differently: `enabled` is folded)
    VehicleDesc enabled = slippery(sedan());
    enabled.traction_control.enabled = true;
    CHECK(same_physics(run_sedan(enabled, std::nullopt), today));

    // Street unit, default mode On (nobody touches the channel): the state hash equals today's on every tick
    const Trace s = run_sedan(slippery(sedan(with_unit(kStreet))), std::nullopt);
    REQUIRE(s.hashes.size() == today.hashes.size());
    CHECK(s.hashes == today.hashes);
    CHECK(same_physics(s, today));
    CHECK(s.effective == 1);

    // Sport and Racing units at mode 1: physically identical (the mode table changes the hash, not the dynamics)
    for (const std::string& path : {kSport, kRacing}) {
        CAPTURE(path);
        const Trace u = run_sedan(slippery(sedan(with_unit(path))), 1.0);
        CHECK(same_physics(u, today));
        CHECK(u.hashes != today.hashes); // the mode table is folded: a unit with modes beyond Off/On is a different car
    }

    // codes 2-4 on a car without a unit are On
    const Trace c3 = run_sedan(slippery(sedan()), 3.0);
    CHECK(c3.hashes == today.hashes);
    // a Street unit given code 4 runs On too
    CHECK(run_sedan(slippery(sedan(with_unit(kStreet))), 4.0).hashes == today.hashes);
}

TEST_CASE("H2: the other modes really run other parameters; ignition_cut is honoured", "[assist][vehicle]") {
    const Trace on = run_sedan(slippery(sedan(with_unit(kSport))), 1.0);
    const Trace relaxed = run_sedan(slippery(sedan(with_unit(kSport))), 2.0);
    const Trace sport = run_sedan(slippery(sedan(with_unit(kSport))), 3.0);
    std::printf("[H2] mean front slip: on %.4f relaxed %.4f sport %.4f; min fraction on %.3f relaxed %.3f sport %.3f\n", on.mean_slip, relaxed.mean_slip,
                sport.mean_slip, on.min_fraction, relaxed.min_fraction, sport.min_fraction);
    CHECK(relaxed.effective == 2);
    CHECK(sport.effective == 3);
    CHECK(relaxed.hashes != on.hashes);
    CHECK(sport.hashes != on.hashes);
    CHECK(sport.min_fraction >= 0.15 - 1e-12); // the Sport torque floor holds in a real run
    CHECK(on.min_fraction < 0.15);              // ... while mode On goes lower here (discriminating)

    // A code-built unit whose Relaxed row copies On's numbers: ignition_cut true = On bit for bit, false = differs
    // (the cut fires in this scenario, else the flag could not matter).
    VehicleDesc d = slippery(sedan());
    AssistUnitDesc u;
    u.grade = AssistGrade::Sport;
    u.has_traction_control = true;
    u.traction_control.modes_bits = 0b00111;
    u.traction_control.params[2].ignition_cut = true;
    d.assist_unit = u;
    const Trace clone_cut = run_sedan(d, 2.0);
    const Trace today = run_sedan(slippery(sedan(with_unit(kStreet))), 1.0);
    CHECK(same_physics(clone_cut, today));
    d.assist_unit->traction_control.params[2].ignition_cut = false;
    const Trace clone_nocut = run_sedan(d, 2.0);
    CHECK_FALSE(same_physics(clone_nocut, today));
}

// ---------------------------------------------------------------------------------------------------------------
// D: determinism and allocation.
// ---------------------------------------------------------------------------------------------------------------
TEST_CASE("D1: with a unit - 1 vs 10 workers identical, zero own allocation per tick", "[assist][vehicle][determinism]") {
    if (!jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    for (real code : {1.0, 3.0}) {
        CAPTURE(code);
        const VehicleDesc d = slippery(sedan(with_unit(kRacing)));
        const Trace w1 = run_sedan(d, code, 1, 480, true);
        const Trace w10 = run_sedan(d, code, 10, 480, true);
        CHECK(w1.hashes == w10.hashes);
        CHECK(w1.own_violations == 0);
        CHECK(w10.own_violations == 0);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// V6: mode ordering on car_hyper with a Racing unit attached in the test.
// ---------------------------------------------------------------------------------------------------------------
namespace {
const VehicleDesc& hyper_racing() {
    static const VehicleDesc desc = [] {
        jobs::JobSystem pool(10);
        io::VehicleLoadOptions o;
        o.maps.pool = &pool;
        o.maps.cache_dir = (fs::temp_directory_path() / "ps_assists_diag_cache").string();
        fs::create_directories(o.maps.cache_dir);
        o.electronics["assists"] = kRacing; // Assists I2: car_hyper ships this unit under id "assists"; same id = replace, not a second unit
        return io::load_vehicle_json(kData + "/vehicles/car_hyper.json", o);
    }();
    return desc;
}

// Mean rear |kappa| over [0.3, 3.0] s after the throttle step, WOT in 1st on a 5 % grass_rg slope, rolling start 8 m/s.
real hyper_mean_kappa(real code, int workers = 4, real* v_end = nullptr) {
    const VehicleDesc& desc = hyper_racing();
    ps_test::VehicleTestRig rig;
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = workers;
    rig.world = std::make_unique<World>(cfg);
    rig.surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/assists/surfaces_racing_game.json");
    const SurfaceId sid = rig.surfaces->id_for("grass_rg");
    const real grade = 0.05;
    const Quat tilt = Quat::from_axis_angle(Vec3::unit_y(), math::atan(-grade)); // the plane RISES in +X
    BodyDesc ground;
    ground.shape = ps_test::flat_surface_mesh(300.0, sid);
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.orientation = tilt;
    rig.world->create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.orientation = tilt;
    chassis.pose.position = tilt.rotate(Vec3{0.0, 0.0, 0.5});
    rig.chassis = rig.world->create_body(chassis);
    rig.world->set_surface_table(rig.surfaces);
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);
    World& w = *rig.world;
    const Vec3 dir = tilt.rotate(Vec3::unit_x());

    w.set_control("assist.traction_control", code);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", 1.0);
    ps_test::settle(w, 1.5);
    // Rolling start with the clutch LOCKED in 1st: the rim never drops below the 3.1 m/s anti-stall equivalent, so
    // the known crawl bug (lock-on-spinning-wheels at crawl speed, PHYS-074) cannot start.
    Motion mo;
    mo.linear = dir * 8.0;
    w.backend().set_motion(rig.chassis, mo);
    vehicle::VehicleResetOptions ro;
    ro.gear = 1;
    ro.hubs_roll_with_chassis = true;
    ro.clutch_locked = true;
    w.reset_vehicle(rig.vehicle, ro);
    w.set_control("handbrake", 0.0);
    w.set_control("throttle", 1.0);
    const int n = static_cast<int>(3.0 * 240.0 + 0.5);
    real sum = 0;
    int cnt = 0;
    for (int k = 0; k < n; ++k) {
        w.step();
        if (k >= static_cast<int>(0.3 * 240.0)) {
            sum += 0.5 * (std::abs(w.wheel_telemetry(rig.vehicle, 2).slip_ratio) + std::abs(w.wheel_telemetry(rig.vehicle, 3).slip_ratio));
            ++cnt;
        }
        REQUIRE(w.powertrain_state(rig.vehicle).engine_state != drivetrain::EngineState::Stalled);
    }
    if (v_end) *v_end = dot(w.get_motion(rig.chassis).linear, dir);
    return sum / cnt;
}
} // namespace

TEST_CASE("V6: hyper mode ordering - mean rear kappa On < Relaxed < Sport < Drift", "[assist][vehicle][long]") {
    real v[5] = {};
    real k[5] = {};
    for (int code : {0, 1, 2, 3, 4}) k[code] = hyper_mean_kappa(static_cast<real>(code), 4, &v[code]);
    std::printf("[V6] mean rear kappa: off %.4f on %.4f relaxed %.4f sport %.4f drift %.4f | v_end %.2f %.2f %.2f %.2f %.2f\n", k[0], k[1], k[2], k[3], k[4],
                v[0], v[1], v[2], v[3], v[4]);
    CHECK(k[1] + 0.01 < k[2]);
    CHECK(k[2] + 0.01 < k[3]);
    CHECK(k[3] + 0.01 < k[4]);
    CHECK(k[0] > k[1]); // TC off spins more than TC on (the ordering is not an artefact of the rig)
    // the mode run is the mode asked for
    CHECK(hyper_mean_kappa(3.0, 1) == k[3]); // worker count does not change the result
}
