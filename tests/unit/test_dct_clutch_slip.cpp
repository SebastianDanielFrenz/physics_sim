// DCT shift clutch against the engine's torque (owner drive 2026-10-19, hyper ICE + battery hybrid, 7-speed DCT): in 6th and
// 7th the engine sat at the limiter, 1200-1500 rpm above the locked speed, while the HUD's clutch_engagement read 1.00. The
// mark's garage setup (big-bore V8, large twin turbo at 4.08 bar, stock hyper_7dct) makes ~2150-2400 N*m, more than the
// stock box's shift clutches hold (1900 N*m kinetic, 1.15 x that static): the clutch slips at its kinetic capacity, which
// the physics gets right. These tests pin that (the clutch_slip / clutch_torque fields now read the active shift clutch of
// a DCT, they were always 0), and that a clutch which covers the torque locks. Diagnostics: ./ps_unit_tests "[dct_slip]"
#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <tuple>
#include <stdexcept>
#include <variant>

using namespace ps;
using Catch::Approx;

namespace {

constexpr const char* kDctFixture = "/tests/support/fixtures/vehicles/car_hyper_n2o_dct_hybrid.json";
constexpr double kRadToRpm = 60.0 / (2.0 * 3.14159265358979323846);

// The owner's engine: the hyper V8 bored out by 3 mm with the head kept (the garage's rule CR' = 1 + (CR - 1) (b'/b)^2) and a
// twin-turbo gauge target (the hyper's own twin turbo configuration, a stand-in for the garage's large kit).
vehicle::VehicleDesc load_car(double boost_bar, double bore_mm, double clutch_scale) {
    // One directory per call and process: ctest runs the test cases as parallel processes.
    static int call_counter = 0;
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "ps_dct_clutch_slip" /
        (std::to_string(static_cast<long long>(std::chrono::steady_clock::now().time_since_epoch().count())) + "_" + std::to_string(call_counter++));
    std::filesystem::create_directories(dir);
    const double cr = 1.0 + 8.0 * (bore_mm / 92.0) * (bore_mm / 92.0);
    {
        std::ofstream e(dir / "engine.json");
        e << "{\"format\": \"physics_sim.engine/1\", \"extends\": \"" << PS_SOURCE_DIR << "/data/engines/hyper_v8_one1_audio.json\", "
          << "\"bore_mm\": " << bore_mm << ", \"cycle\": {\"compression_ratio\": " << cr << "}}\n";
        std::ofstream t(dir / "turbo.json");
        t << "{\"format\": \"physics_sim.turbo_configuration/1\", \"extends\": \"" << PS_SOURCE_DIR
          << "/data/turbo_configurations/hyper_twin_default.json\", \"target_boost_bar\": " << boost_bar << "}\n";
    }
    static jobs::JobSystem pool(10);
    io::VehicleLoadOptions lo;
    lo.maps.quality = drivetrain::MapQuality::Preview;
    lo.maps.pool = &pool;
    lo.maps.cache_dir = (std::filesystem::temp_directory_path() / "ps_hybrid_ice_start_cache").string();
    lo.part_overrides["engine"].ref = (dir / "engine.json").string();
    lo.part_overrides["engine"].slots["forced_induction"] = (dir / "turbo.json").string();
    vehicle::VehicleDesc desc = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + kDctFixture, lo);
    for (auto& c : desc.powertrain.components)
        if (auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) g->shift_clutch.capacity_nm *= clutch_scale;
    return desc;
}

const drivetrain::GearboxDesc& gearbox_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) return *g;
    throw std::runtime_error("no gearbox");
}

struct Pull {
    double top_kmh = 0.0;
    double peak_demand_nm = 0.0;                // largest engine torque demand with the gearbox in a gear and Idle
    double max_slip_rpm = 0.0;                  // largest |clutch_slip| in phase Idle, gear >= 5, after the post-shift settle
    double max_slip_error_rpm = 0.0;            // |clutch_slip - (engine rpm - ratio * output rpm)| (independent computation)
    double slip_time_s = 0.0;                   // time in phase Idle, gear >= 5, with |slip| above the limit
    double torque_while_slipping_nm = 0.0;      // mean clutch_torque in those ticks
    double peak_clutch_torque_nm = 0.0;         // largest |clutch_torque| in an Idle phase with a gear engaged
    int slipping_ticks = 0;
};

// WOT from rest in drive mode `mode`, `seconds` long; settle_s = how long after a gear change the clutch may still be closing.
Pull wot_pull(const vehicle::VehicleDesc& desc, double seconds, double slip_limit_rpm, double settle_s, bool verbose) {
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1800.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, 4, Vec3::unit_y(), 240.0, 960.0, 8000.0);
    World& w = *rig.world;
    const drivetrain::GearboxDesc& gb = gearbox_of(desc);
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("electric.ready", 1.0);
    w.set_control("gear_request", 99.0);
    w.set_control("throttle", 0.0);
    ps_test::settle(w, 1.0);
    w.set_control("throttle", 1.0);
    Pull r;
    int last_gear = -9;
    double since_gear_change = 0.0;
    double torque_sum = 0.0;
    for (int i = 0; i < static_cast<int>(240.0 * seconds); ++i) {
        w.step();
        const auto& st = w.powertrain_state(rig.vehicle);
        const auto& dl = st.driveline;
        const double kmh = w.backend().get_motion(rig.chassis).linear.length() * 3.6;
        r.top_kmh = std::max(r.top_kmh, kmh);
        if (st.gear != last_gear) {
            since_gear_change = 0.0;
            last_gear = st.gear;
        } else {
            since_gear_change += 1.0 / 240.0;
        }
        const double slip_rpm = dl.clutch_slip * kRadToRpm;
        if (st.shift_phase == drivetrain::ShiftPhase::Idle && st.gear >= 1 && st.gear <= static_cast<int>(gb.forward_ratios.size())) {
            r.peak_demand_nm = std::max(r.peak_demand_nm, double(w.powertrain_debug(rig.vehicle).engine_torque_demand));
            r.peak_clutch_torque_nm = std::max(r.peak_clutch_torque_nm, std::abs(double(dl.clutch_torque)));
            const double expected = st.engine_rpm - gb.forward_ratios[static_cast<std::size_t>(st.gear - 1)] * dl.gearbox_out_omega * kRadToRpm;
            if (since_gear_change > settle_s) {
                r.max_slip_error_rpm = std::max(r.max_slip_error_rpm, std::abs(slip_rpm - expected));
                if (st.gear >= 5) {
                    r.max_slip_rpm = std::max(r.max_slip_rpm, std::abs(slip_rpm));
                    if (std::abs(slip_rpm) > slip_limit_rpm) {
                        r.slip_time_s += 1.0 / 240.0;
                        ++r.slipping_ticks;
                        torque_sum += dl.clutch_torque;
                    }
                }
            }
        }
        if (verbose && (i % 120 == 0)) {
            std::printf("  t=%6.2f v=%5.0f gear=%d ph=%d eng=%6.0f slip=%6.0f Tclutch=%6.0f Tdem=%6.0f e=%.2f boost=%.2f\n", i / 240.0, kmh, st.gear,
                        static_cast<int>(st.shift_phase), double(st.engine_rpm), slip_rpm, double(dl.clutch_torque),
                        double(w.powertrain_debug(rig.vehicle).engine_torque_demand), double(st.clutch_engagement),
                        double(st.engines.empty() ? 0 : st.engines[0].boost_bar));
        }
    }
    if (r.slipping_ticks > 0) r.torque_while_slipping_nm = torque_sum / r.slipping_ticks;
    return r;
}

double lock_window_rpm(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.controllers)
        if (const auto* a = std::get_if<drivetrain::AutomaticTcuDesc>(&c.params)) return a->launch_lock_slip_rpm;
    throw std::runtime_error("no automatic TCU");
}

// The shift clutch's own timing bounds the time from a gear change to a closed clutch: torque phase + inertia phase + engage.
double settle_after_shift_s(const vehicle::VehicleDesc& d) {
    const auto& sc = gearbox_of(d).shift_clutch;
    return sc.torque_phase_s + sc.max_inertia_phase_s + sc.engage_time_s;
}

} // namespace

TEST_CASE("DCT: clutch_slip and clutch_torque read the active shift clutch and a covered engine keeps it locked", "[dct][drivetrain][vehicle]") {
    // The hyper's stock 1.8 bar twin turbo: peak demand ~1470 N*m against 1900 N*m kinetic / 2185 static.
    const vehicle::VehicleDesc desc = load_car(1.8, 92.0, 1.0);
    const Pull r = wot_pull(desc, 25.0, lock_window_rpm(desc), settle_after_shift_s(desc), false);
    INFO("top " << r.top_kmh << " km/h, peak demand " << r.peak_demand_nm << " N*m");
    CHECK(r.top_kmh > 240.0);                    // reached the gears the check is about (gear >= 5 from ~240 km/h)
    CHECK(r.peak_demand_nm < 1900.0 * 1.15);     // the clutch covers the engine
    CHECK(r.slip_time_s == 0.0);                 // never beyond the lock window in an Idle phase of gear >= 5
    CHECK(r.max_slip_error_rpm < 0.5);           // the field equals engine rpm - ratio * output rpm, computed from the desc
    CHECK(r.peak_clutch_torque_nm > 800.0);      // and it is the torque through the clutch, not a dead 0
}

TEST_CASE("DCT: an engine stronger than the shift clutches makes them slip at their kinetic capacity", "[dct][drivetrain][vehicle]") {
    // The owner's engine class: the hyper V8 bored +3 mm at a 3.8 bar target, ~2370 N*m against the stock 1900 / 2185 N*m.
    const vehicle::VehicleDesc desc = load_car(3.8, 95.0, 1.0);
    const auto& sc = gearbox_of(desc).shift_clutch;
    const double kinetic = sc.capacity_nm, stat = sc.capacity_nm * sc.static_factor;
    const Pull r = wot_pull(desc, 30.0, lock_window_rpm(desc), settle_after_shift_s(desc), false);
    INFO("peak demand " << r.peak_demand_nm << " N*m, max slip " << r.max_slip_rpm << " rpm, slipping " << r.slip_time_s << " s at "
                        << r.torque_while_slipping_nm << " N*m");
    CHECK(r.peak_demand_nm > stat);                        // precondition: the engine beats even the static capacity
    CHECK(r.slip_time_s > 3.0);                            // sustained slip in gear >= 5 (the owner saw it in 6th and 7th)
    CHECK(r.max_slip_rpm > 800.0);                         // ... of the order the owner measured (1200-1500 rpm)
    CHECK(std::abs(r.torque_while_slipping_nm) == Approx(kinetic).epsilon(0.01)); // a slipping clutch carries its kinetic capacity
    CHECK(r.max_slip_error_rpm < 0.5);
}

TEST_CASE("DCT: shift clutches whose static capacity covers the engine lock within the shift's own time", "[dct][drivetrain][vehicle]") {
    // The same engine with the garage's shift_clutch_capacity raised: 1.5 x 1900 x 1.15 = 3277 N*m against ~2570 N*m of demand.
    const vehicle::VehicleDesc desc = load_car(3.8, 95.0, 1.5);
    const auto& sc = gearbox_of(desc).shift_clutch;
    const Pull r = wot_pull(desc, 30.0, lock_window_rpm(desc), settle_after_shift_s(desc), false);
    INFO("peak demand " << r.peak_demand_nm << " N*m, max slip " << r.max_slip_rpm << " rpm");
    CHECK(r.top_kmh > 300.0);
    CHECK(r.peak_demand_nm > 1900.0 * 1.15);               // precondition: the stock clutch would not have held this
    CHECK(r.peak_demand_nm * 1.1 < sc.capacity_nm * sc.static_factor);
    CHECK(r.slip_time_s == 0.0);                           // locked in every Idle phase of gear >= 5, settle after each shift
    CHECK(r.max_slip_error_rpm < 0.5);
}

TEST_CASE("DCT slip calibration (diag)", "[.][diag][dct_slip]") {
    for (const auto& [boost, bore, scale] : {std::tuple{1.8, 92.0, 1.0}, std::tuple{3.8, 92.0, 1.0}, std::tuple{3.8, 95.0, 1.0},
                                              std::tuple{3.8, 95.0, 1.5}, std::tuple{4.5, 95.0, 1.5}}) {
        std::printf("[dct slip] boost %.1f bore %.0f clutch x%.1f\n", boost, bore, scale);
        const vehicle::VehicleDesc d = load_car(boost, bore, scale);
        const Pull r = wot_pull(d, 40.0, lock_window_rpm(d), settle_after_shift_s(d), true);
        std::printf("[dct slip] -> top %.0f km/h, peak demand %.0f N*m, max slip %.0f rpm, slipping %.2f s at %.0f N*m, field error %.1f rpm\n", r.top_kmh,
                    r.peak_demand_nm, r.max_slip_rpm, r.slip_time_s, r.torque_while_slipping_nm, r.max_slip_error_rpm);
    }
}
