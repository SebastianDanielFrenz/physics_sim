// Hybrid (ICE + axle motor) shift schedule: the automatic transmission schedules on the DRIVER's pedal command, not on the
// supervisor's ICE share or on requested/available power. Owner report 2026-10-18 (racing_game, car_hyper + hybrid
// conversion, Racing): "the hybrid is not shifting for optimal performance" - upshifts came at ~4100/4700 rpm instead of
// near the full-throttle point (~8000); the schedule read requested power / (motors + the engine's full-load power).
#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace ps;

namespace {

vehicle::VehicleDesc load_hybrid_dct() {
    static jobs::JobSystem pool(10);
    io::EngineMapOptions opt;
    opt.quality = drivetrain::MapQuality::Preview;
    opt.pool = &pool;
    opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_hybrid_shift_engine_cache").string();
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/car_hyper_n2o_dct_hybrid.json", opt);
}

struct Upshift {
    int from;
    double rpm;         // peak engine rpm in the gear before the shift (the schedule decides on the gearbox input rpm: the up point)
    double ice_command; // lowest engine throttle command in that gear, outside shifts and the launch
    double tc_fraction; // lowest traction-control torque fraction in that gear
};

struct Pull {
    std::vector<Upshift> upshifts;
    double min_request_w = 1e30, max_request_w = 0.0;
    double min_tcf = 1.0;
    bool tc_intervened = false;
};

// Pull from rest in drive on the hybrid DCT fixture; `mode` is the hybrid/drive-mode channel (3 = Racing). An upshift is
// recorded when the engaged gear increases, with the peak rpm reached in the gear it leaves.
Pull run_pull(double mode, double tc, double pedal, double seconds) {
    vehicle::VehicleDesc desc = load_hybrid_dct();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, 4, Vec3::unit_y(), 240.0, 960.0, 6000.0);
    World& w = *rig.world;
    w.set_control("assist.drive_mode", mode);
    w.set_control("assist.traction_control", tc);
    w.set_control("assist.aero_control", 0.0);
    ps_test::settle(w, 2.0);
    w.set_control("gear_request", 99.0);
    w.set_control("throttle", pedal);
    Pull out;
    int gear = 0;
    double peak_rpm = 0.0, min_cmd = 1e30, min_tcf_gear = 1.0;
    const int n = static_cast<int>(seconds * 240.0);
    for (int i = 0; i < n; ++i) {
        w.step();
        const drivetrain::PowertrainSnapshot& st = w.powertrain_state(rig.vehicle);
        if (st.gear > gear && gear >= 1 && st.gear == gear + 1) out.upshifts.push_back({gear, peak_rpm, min_cmd, min_tcf_gear});
        if (st.gear != gear) {
            gear = st.gear;
            peak_rpm = 0.0;
            min_cmd = 1e30;
            min_tcf_gear = 1.0;
        }
        peak_rpm = std::max(peak_rpm, st.engine_rpm);
        if (st.gear >= 1 && st.shift_phase == drivetrain::ShiftPhase::Idle && !st.engines.empty() && st.engine_rpm > 2500.0)
            min_cmd = std::min(min_cmd, st.engines.at(0).throttle_command);
        min_tcf_gear = std::min(min_tcf_gear, st.traction_torque_fraction);
        if (st.gear >= 1 && st.hybrid_ice_requested) {
            out.min_request_w = std::min(out.min_request_w, st.hybrid_requested_power_w);
            out.max_request_w = std::max(out.max_request_w, st.hybrid_requested_power_w);
        }
        out.min_tcf = std::min(out.min_tcf, st.traction_torque_fraction);
        out.tc_intervened |= st.traction_control_intervening;
    }
    return out;
}

constexpr double kUpFull = 8000.0; // the fixture's automatic_tcu schedule.up_rpm_at_full_throttle

void print_pull(const char* label, const Pull& p) {
    for (const Upshift& u : p.upshifts)
        std::printf("[hybrid shift] %s: %d->%d at %.0f rpm, min ICE command %.2f, min TC fraction %.2f\n", label, u.from, u.from + 1, u.rpm, u.ice_command, u.tc_fraction);
}

} // namespace

TEST_CASE("Hybrid DCT at full pedal in Racing upshifts near the full-throttle schedule point", "[hybrid][tcu][drivetrain][vehicle]") {
    const Pull p = run_pull(3.0, 0.0, 1.0, 5.0);
    print_pull("pedal 1, mode 3, TC off", p);
    REQUIRE(p.upshifts.size() >= 2);
    for (std::size_t i = 0; i < 2; ++i) {
        CHECK(p.upshifts[i].rpm >= kUpFull * 0.97);
        CHECK(p.upshifts[i].rpm <= kUpFull * 1.05);
        CHECK(p.upshifts[i].ice_command > 0.98);
    }
}

TEST_CASE("Hybrid DCT with traction control intervening keeps the full-throttle shift points and the full request", "[hybrid][tcu][drivetrain][vehicle]") {
    const Pull p = run_pull(3.0, 1.0, 1.0, 9.0);
    print_pull("pedal 1, mode 3, TC on", p);
    std::printf("[hybrid shift] TC on: min torque fraction %.2f, intervened %d, request %.0f..%.0f kW\n", p.min_tcf, p.tc_intervened ? 1 : 0,
                p.min_request_w / 1e3, p.max_request_w / 1e3);
    REQUIRE(p.tc_intervened); // otherwise the case proves nothing
    REQUIRE(p.upshifts.size() >= 2);
    for (std::size_t i = 0; i < 2; ++i) {
        CHECK(p.upshifts[i].rpm >= kUpFull * 0.97);
        CHECK(p.upshifts[i].tc_fraction < 0.95); // TC acted in that gear
        CHECK(p.upshifts[i].ice_command > 0.98); // ... and did not reach the ICE command
    }
    // The request is the driver's (250 kW at full pedal in the fixture); the ceiling is applied downstream, once.
    CHECK(p.min_request_w == Catch::Approx(250000.0).margin(1.0));
    CHECK(p.max_request_w == Catch::Approx(250000.0).margin(1.0));
}

TEST_CASE("Hybrid DCT schedules on the pedal: part throttle shifts earlier than full throttle", "[hybrid][tcu][drivetrain][vehicle]") {
    const Pull full = run_pull(3.0, 0.0, 1.0, 3.0);
    const Pull part = run_pull(3.0, 0.0, 0.5, 9.0);
    REQUIRE(full.upshifts.size() >= 1);
    REQUIRE(part.upshifts.size() >= 1);
    std::printf("[hybrid shift] pedal 0.5: 1->2 at %.0f rpm; pedal 1: 1->2 at %.0f rpm\n", part.upshifts[0].rpm, full.upshifts[0].rpm);
    // schedule: up = lerp(2500, 8000, pedal) = 5250 at 0.5
    CHECK(part.upshifts[0].rpm == Catch::Approx(5250.0).margin(5250.0 * 0.06));
    CHECK(part.upshifts[0].rpm < full.upshifts[0].rpm - 1500.0);
}
