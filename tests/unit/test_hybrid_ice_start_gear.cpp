// Hybrid supervisor: ICE start while the car is moving fast on electric power (owner report 2026-10-19, racing_game: "on a
// hybrid, when the ICE starts while the vehicle moves at 150 km/h+, the ICE seems to shift through all gears from 1st").
// Diagnostic reproduction (hidden [.] cases, they print): ./ps_unit_tests "[hybrid_ice_start]"
#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <map>
#include <variant>
#include <string>
#include <vector>

using namespace ps;

namespace {

vehicle::VehicleDesc load_fixture(const std::string& rel) {
    static jobs::JobSystem pool(10);
    io::EngineMapOptions opt;
    opt.quality = drivetrain::MapQuality::Preview;
    opt.pool = &pool;
    opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_hybrid_ice_start_cache").string();
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + rel, opt);
}

struct Scenario {
    double kmh = 150.0;
    double mode = 1.0;                       // assist.drive_mode: 1 Eco (ICE only when the request exceeds the motors), 2/3 ICE always
    std::function<double(double)> pedal;     // pedal over time
    double seconds = 12.0;
    bool verbose = false;
    double ice_off_at = -1.0;                // > 0: drop mode to 0 (EV only) at this time (ICE stops after stop_delay_s)
    double ice_on_again_at = -1.0;           // > 0: back to `mode` at this time
    std::function<void(vehicle::VehicleDesc&)> mutate;  // test-only desc edit before the rig is built
    int proxy_fit_gear = 0;                  // > 0: PROXY for the proposed fix - the tick the ICE becomes Running, reset_vehicle into this gear (ICE at idle, clutch not locked)
};

struct Trace {
    double ice_start_t = -1.0;
    double connect_t = -1.0;
    double max_rpm = 0.0;
    double rpm_at_connect = 0.0;
    double speed_at_connect_kmh = 0.0;
    int first_gear = 0;
    int final_gear = 0;
    double final_rpm = 0.0;
    std::vector<int> gears;                  // distinct gears (>= 0) after the first connection, in order
    std::map<int, double> rpm_per_kmh;       // locked rpm per km/h of each gear seen (clutch >= 0.99)
    int ice_stops = 0;
    double rpm_at_stop_gear = 0.0;
    std::vector<int> gears_before_stop;
    double v_ice_start_kmh = 0.0;
    double min_v_after_kmh = 1e9;            // lowest speed in the 4 s after the ICE reports Running
    std::vector<int> seq;                    // engaged gears (>= 1) after the first connection, distinct, in order
    double peak_rpm_to_upshift = 0.0;        // peak engine rpm from ICE Running to the first change of gear
    double lock_t = -1.0;                    // first time after the connection with the clutch >= 0.99
    std::uint64_t state_hash = 0;
};

Trace run(const std::string& fixture, const Scenario& sc) {
    vehicle::VehicleDesc desc = load_fixture(fixture);
    if (sc.mutate) sc.mutate(desc);
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1800.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, 4, Vec3::unit_y(), 240.0, 960.0, 6000.0);
    World& w = *rig.world;
    w.set_control("assist.drive_mode", sc.mode);
    w.set_control("assist.traction_control", 0.0);
    w.set_control("assist.aero_control", 0.0);
    w.set_control("electric.ready", 1.0);
    w.set_control("gear_request", 99.0);
    w.set_control("throttle", 0.0);
    ps_test::settle(w, 1.0);
    Motion m;
    m.linear = Vec3{sc.kmh / 3.6, 0, 0};
    w.backend().set_motion(rig.chassis, m);
    vehicle::VehicleResetOptions ro;
    ro.gear = 0;
    ro.engine_state = drivetrain::EngineState::Off;
    ro.hubs_roll_with_chassis = true;
    w.reset_vehicle(rig.vehicle, ro);
    Trace t;
    const int hz = 240;
    const int total = static_cast<int>(sc.seconds * hz);
    int last_gear = -99, last_phase = -1, last_state = -1;
    bool was_running = false;
    for (int i = 0; i < total; ++i) {
        const double sim_t = i / double(hz);
        w.set_control("throttle", sc.pedal(sim_t));
        if (sc.ice_off_at > 0 && sim_t >= sc.ice_off_at && !(sc.ice_on_again_at > 0 && sim_t >= sc.ice_on_again_at))
            w.set_control("assist.drive_mode", 0.0);
        else
            w.set_control("assist.drive_mode", sc.mode);
        w.step();
        const auto& st = w.powertrain_state(rig.vehicle);
        const double speed_kmh = w.backend().get_motion(rig.chassis).linear.length() * 3.6;
        const int es = static_cast<int>(st.engine_state);
        const int ph = static_cast<int>(st.shift_phase);
        const bool running = st.engine_state == drivetrain::EngineState::Running;
        if (running && t.ice_start_t < 0) {
            t.ice_start_t = sim_t;
            t.v_ice_start_kmh = speed_kmh;
            if (sc.proxy_fit_gear > 0) {
                vehicle::VehicleResetOptions pro;
                pro.gear = sc.proxy_fit_gear;
                pro.engine_state = drivetrain::EngineState::Running;
                pro.hubs_roll_with_chassis = true;
                pro.clutch_locked = false;
                w.reset_vehicle(rig.vehicle, pro);
            }
        }
        if (was_running && !running && st.engine_state != drivetrain::EngineState::Running) {
            ++t.ice_stops;
            t.rpm_at_stop_gear = st.gear;
        }
        was_running = running;
        if (t.ice_start_t >= 0) t.max_rpm = std::max(t.max_rpm, double(st.engine_rpm));
        if (t.ice_start_t >= 0 && sim_t - t.ice_start_t <= 4.0) t.min_v_after_kmh = std::min(t.min_v_after_kmh, speed_kmh);
        if (st.gear >= 1 && t.connect_t < 0) {
            t.connect_t = sim_t;
            t.rpm_at_connect = st.engine_rpm;
            t.first_gear = st.gear;
            t.speed_at_connect_kmh = speed_kmh;
        }
        if (t.connect_t >= 0 && (t.gears.empty() || t.gears.back() != st.gear)) t.gears.push_back(st.gear);
        if (t.connect_t >= 0 && st.gear >= 1 && (t.seq.empty() || t.seq.back() != st.gear)) t.seq.push_back(st.gear);
        if (t.ice_start_t >= 0 && t.seq.size() < 2) t.peak_rpm_to_upshift = std::max(t.peak_rpm_to_upshift, double(st.engine_rpm));
        if (t.connect_t >= 0 && t.lock_t < 0 && st.gear >= 1 && st.clutch_engagement >= 0.99) t.lock_t = sim_t;
        if (st.gear >= 1 && st.clutch_engagement >= 0.99 && st.shift_phase == drivetrain::ShiftPhase::Idle && speed_kmh > 20.0 &&
            running)
            t.rpm_per_kmh[st.gear] = st.engine_rpm / speed_kmh;
        if (sc.verbose && (st.gear != last_gear || ph != last_phase || es != last_state || i % 24 == 0)) {
            std::printf("  t=%6.3f v=%6.1f km/h gear=%2d tgt=%2d phase=%d eng_state=%d rpm=%6.0f clutch=%.2f ice_req=%d\n", sim_t, speed_kmh,
                        st.gear, st.gear_target, ph, es, double(st.engine_rpm), double(st.clutch_engagement), st.hybrid_ice_requested ? 1 : 0);
        }
        last_gear = st.gear;
        last_phase = ph;
        last_state = es;
    }
    t.final_gear = w.powertrain_state(rig.vehicle).gear;
    t.final_rpm = w.powertrain_state(rig.vehicle).engine_rpm;
    t.state_hash = w.state_hash();
    return t;
}

void print(const std::string& label, const Trace& t) {
    std::printf("[hybrid ice start] %s: ICE running %.2f s, connected %.2f s at %.0f km/h in gear %d (engine %.0f rpm), peak rpm %.0f, final gear %d (%.0f rpm), gears:",
                label.c_str(), t.ice_start_t, t.connect_t, t.speed_at_connect_kmh, t.first_gear, t.rpm_at_connect, t.max_rpm, t.final_gear, t.final_rpm);
    for (int g : t.gears) std::printf(" %d", g);
    std::printf("\n");
}

// The gear the proposed fix would engage: lowest gear whose input rpm at the present speed does not exceed the schedule's
// upshift point at the driver's pedal.

const char* kDct = "/tests/support/fixtures/vehicles/car_hyper_n2o_dct_hybrid.json";
const char* kManual = "/data/vehicles/car_hybrid.json";

// Pedal 0.25 for 2 s (electric cruise in mode 1), then `p`.
std::function<double(double)> step_pedal(double p) {
    return [p](double t) { return t < 2.0 ? 0.25 : p; };
}

// rpm of the gearbox input per km/h of road speed, per forward gear, from the loaded desc (gear ratio x the first differential's
// ratio / the rear wheel radius).
std::vector<double> rpm_per_kmh_from_desc(const std::string& fixture) {
    const vehicle::VehicleDesc d = load_fixture(fixture);
    double diff = 1.0;
    std::vector<double> ratios;
    for (const auto& c : d.powertrain.components) {
        if (const auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) ratios = g->forward_ratios;
        if (const auto* df = std::get_if<drivetrain::DifferentialDesc>(&c.params)) diff = df->ratio;
    }
    double radius = 0.34;
    for (const auto& wh : d.wheels)
        if (wh.name == "RL") radius = wh.wheel_radius;
    std::vector<double> out;
    for (double r : ratios) out.push_back(r * diff * 60.0 / (2.0 * 3.14159265358979323846 * radius) / 3.6);
    return out;
}

// The schedule's upshift rpm at full pedal, read from the loaded controller descriptor (not from the production helper).
double upshift_full_rpm(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers) {
        if (const auto* a = std::get_if<drivetrain::AutomaticTcuDesc>(&c.params)) return a->schedule.up_rpm_at_full_throttle;
        if (const auto* m = std::get_if<drivetrain::ManualTcuDesc>(&c.params)) return m->auto_shift.up_rpm_at_full_throttle;
    }
    return 0.0;
}

// Independent fit gear: the lowest forward gear whose gearbox input rpm at `kmh` is at or below the upshift rpm, else the top gear.
int independent_fit_gear(const std::string& fixture, double kmh) {
    const std::vector<double> k = rpm_per_kmh_from_desc(fixture);
    const double up = upshift_full_rpm(load_fixture(fixture));
    for (std::size_t g = 0; g < k.size(); ++g)
        if (k[g] * kmh <= up) return static_cast<int>(g) + 1;
    return static_cast<int>(k.size());
}

// What every connect must satisfy (owner report 2026-10-19). Peak-rpm margin and speed-drop bound are set from the measured values
// of the fixed code (drop 0.0 km/h up to 200 km/h; the unfixed code engaged 1st: peak 8260 / 6800 rpm, 3-7 km/h).
constexpr double kPeakRpmMargin = 1.06;   // x max(upshift rpm, the fit gear's input rpm at connect)
constexpr double kMaxSpeedDropKmh = 1.0;      // measured 0.00 up to 200 km/h
constexpr double kMaxSpeedDropDragKmh = 2.0;  // 272 km/h: 1.5-1.6 km/h of drag deceleration while the 250 kW electric power is flat (measured)
constexpr double kConnectTimeoutS = 1.0;  // HybridControlDesc::connect_timeout_s default

void check_connect(const char* name, const std::string& fixture, double kmh) {
    Scenario sc;
    sc.kmh = kmh;
    sc.pedal = step_pedal(1.0);
    sc.seconds = 7.0;
    const Trace t = run(fixture, sc);
    INFO(name << " at " << kmh << " km/h: connected in gear " << t.first_gear << " at " << t.speed_at_connect_kmh << " km/h, peak rpm "
              << t.peak_rpm_to_upshift << ", drop " << (t.v_ice_start_kmh - t.min_v_after_kmh) << " km/h");
    std::printf("[hybrid ice start] %s %.0f km/h: gear %d at %.1f km/h, connect %.3f s, peak %.0f rpm, lock +%.3f s, drop %.2f km/h, gears", name, kmh, t.first_gear,
                t.speed_at_connect_kmh, t.connect_t - t.ice_start_t, t.peak_rpm_to_upshift, t.lock_t - t.connect_t, t.v_ice_start_kmh - t.min_v_after_kmh);
    for (int g : t.seq) std::printf(" %d", g);
    std::printf("\n");
    REQUIRE(t.connect_t >= 0.0);
    const int fit = independent_fit_gear(fixture, t.speed_at_connect_kmh);
    CHECK(t.first_gear == fit);
    for (int g : t.seq) CHECK(g >= fit);                         // no gear below the fit gear before the first upshift
    for (std::size_t i = 1; i < t.seq.size(); ++i) CHECK(t.seq[i] > t.seq[i - 1]); // and no downshift after the connect
    const std::vector<double> k = rpm_per_kmh_from_desc(fixture);
    const double up = upshift_full_rpm(load_fixture(fixture));
    const double fit_rpm = k[static_cast<std::size_t>(fit - 1)] * t.speed_at_connect_kmh;
    CHECK(t.peak_rpm_to_upshift <= kPeakRpmMargin * std::max(up, fit_rpm));
    CHECK(t.v_ice_start_kmh - t.min_v_after_kmh <= (kmh > 250.0 ? kMaxSpeedDropDragKmh : kMaxSpeedDropKmh));
    REQUIRE(t.lock_t >= 0.0);
    CHECK(t.lock_t - t.connect_t <= kConnectTimeoutS + 1.0);       // the clutch closes: rev-match window plus the clutch law
}

} // namespace

TEST_CASE("Hybrid ICE start at speed: DCT connects in the fit gear", "[hybrid][drivetrain]") {
    for (double kmh : {80.0, 150.0, 200.0, 272.0}) check_connect("DCT", kDct, kmh);
}

TEST_CASE("Hybrid ICE start at speed: manual TCU connects in the fit gear", "[hybrid][drivetrain]") {
    for (double kmh : {80.0, 150.0, 200.0, 272.0}) check_connect("manual", kManual, kmh);
}

TEST_CASE("Hybrid ICE start at standstill: the rev-match window never opens", "[hybrid][drivetrain]") {
    // At standstill the fit gear is 1 and its input speed is 0, so connect_timeout_s changes nothing: the world state hash after a
    // standing ICE start and launch is identical with the window disabled.
    for (const char* fx : {kDct, kManual}) {
        Scenario sc;
        sc.kmh = 0.0;
        sc.mode = 2.0;
        sc.pedal = [](double t) { return t < 1.0 ? 0.0 : 0.6; };
        sc.seconds = 5.0;
        const Trace a = run(fx, sc);
        sc.mutate = [](vehicle::VehicleDesc& d) {
            for (auto& c : d.powertrain.components)
                if (auto* h = std::get_if<drivetrain::HybridControlDesc>(&c.params)) h->connect_timeout_s = 0.0;
        };
        const Trace b = run(fx, sc);
        INFO(fx);
        REQUIRE(a.connect_t >= 0.0);
        CHECK(a.first_gear == 1);
        CHECK(a.state_hash == b.state_hash);
    }
}

namespace {

void speed_sweep(const char* name, const char* fixture, double up_full) {
    const std::vector<double> k = rpm_per_kmh_from_desc(fixture);
    std::printf("[hybrid ice start] %s: gearbox input rpm per km/h:", name);
    for (std::size_t g = 0; g < k.size(); ++g) std::printf(" g%zu=%.1f", g + 1, k[g]);
    std::printf("\n");
    for (double kmh : {5.0, 10.0, 20.0, 30.0, 40.0, 60.0, 80.0, 100.0, 120.0, 150.0, 200.0}) {
        Scenario sc;
        sc.kmh = kmh;
        sc.pedal = step_pedal(1.0);
        sc.seconds = 9.0;
        const Trace t = run(fixture, sc);
        int fit = static_cast<int>(k.size());
        for (std::size_t g = 0; g < k.size(); ++g)
            if (k[g] * t.speed_at_connect_kmh <= up_full) {
                fit = static_cast<int>(g) + 1;
                break;
            }
        char label[128];
        std::snprintf(label, sizeof label, "%s %.0f km/h pedal 1 (1st would be %.0f rpm; fit gear for up=%.0f: %d)", name, kmh,
                      k[0] * t.speed_at_connect_kmh, up_full, fit);
        print(label, t);
    }
}

} // namespace

TEST_CASE("Hybrid ICE start at speed: DCT sweep", "[.][diag][hybrid_ice_start]") {
    Scenario sc;
    sc.pedal = step_pedal(1.0);
    sc.verbose = true;
    print("DCT 150 km/h pedal 1 (trace)", run(kDct, sc));
    speed_sweep("DCT", kDct, 8000.0);
}

TEST_CASE("Hybrid ICE start at speed: manual TCU sweep", "[.][diag][hybrid_ice_start]") {
    Scenario sc;
    sc.pedal = step_pedal(1.0);
    sc.verbose = true;
    print("manual 150 km/h pedal 1 (trace)", run(kManual, sc));
    speed_sweep("manual", kManual, 6300.0);
}

TEST_CASE("Hybrid ICE start at speed: partial pedal, kickdown during the start, stop and restart", "[.][diag][hybrid_ice_start]") {
    for (const char* fx : {kDct, kManual}) {
        const std::string name = fx == kDct ? "DCT" : "manual";
        {   // Mode 2 (ICE always on): moderate pedal, then a kickdown to full pedal 0.1 s after the ICE is requested.
            Scenario sc;
            sc.mode = 2.0;
            sc.pedal = [](double t) { return t < 2.0 ? 0.25 : (t < 2.1 ? 0.5 : 1.0); };
            print(name + " 150 km/h mode 2, pedal 0.5 -> kickdown 1.0 at +0.1 s", run(fx, sc));
        }
        {   // Same, pedal 0.5 held.
            Scenario sc;
            sc.mode = 2.0;
            sc.pedal = [](double t) { return t < 2.0 ? 0.25 : 0.5; };
            print(name + " 150 km/h mode 2, pedal 0.5 held", run(fx, sc));
        }
        {   // ICE running (mode 2), then EV only (mode 0) at 6 s -> ICE stops after stop_delay_s, then back to mode 2 at 11 s.
            Scenario sc;
            sc.mode = 2.0;
            sc.pedal = [](double t) { return t < 2.0 ? 0.25 : 1.0; };
            sc.ice_off_at = 8.0;
            sc.ice_on_again_at = 13.0;
            sc.seconds = 18.0;
            sc.verbose = true;
            print(name + " 150 km/h mode 2 -> EV at 8 s -> mode 2 at 13 s (stop/restart)", run(fx, sc));
        }
    }
}

// PROXY for the proposed fix (no physics change): at the tick the ICE reports Running, put the gearbox straight into the gear the
// schedule fits (reset_vehicle: ICE at idle, clutch not forced locked) and let the supervisor/TCU continue from there.
TEST_CASE("Hybrid ICE start at 150 km/h with the fitting gear engaged directly (proxy for the fix)", "[.][diag][hybrid_ice_start]") {
    Scenario sc;
    sc.pedal = step_pedal(1.0);
    sc.proxy_fit_gear = 3;
    sc.verbose = true;
    print("DCT 150 km/h pedal 1, gear 3 at start", run(kDct, sc));
    sc.proxy_fit_gear = 4;
    print("manual 150 km/h pedal 1, gear 4 at start", run(kManual, sc));
}
