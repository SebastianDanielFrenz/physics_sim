// Assists plan Part A (2026-10-12): car_hyper cannot crawl a slight grass slope from 1-2 km/h and stalled once
// (owner report from racing_game). Hidden [.][diag] cases only - they measure, they assert nothing about the
// behaviour (no production change in this step). The plan (out/assists_plan.md) turns the matrix below into an
// asserted crawl test once the fix lands.
//
// Rig: car_hyper.json (maps generated once, cached under the system temp dir) on a TILTED flat mesh ground tagged
// with one named surface (20 m cells - flat_surface_mesh), chassis box 1500 kg {2, .4, .15} at 0.5 m like
// test_traction_control.cpp. The ground rises in +X (the car faces uphill). Surfaces: physics_sim's own
// data/surfaces/surfaces.json (grass lambda_mu 0.42 at Part A, 0.6 since assists I2; crr 0.08) and the test fixture
// tests/support/fixtures/assists/surfaces_racing_game.json, which copies racing_game's game-owned grass
// (lambda_mu 0.63, crr 0.08 - the surface the owner actually drove on) under the name "grass_rg".
//
// Start: settle 1.5 s on the slope with the handbrake, 1st gear requested, auto-clutch on (the launch law holds
// the clutch open at idle). Then either release from rest (v0 = 0) or teleport the chassis to v0 along the slope
// (World::reset_vehicle with hubs rolling, clutch not locked) - the owner's "1-2 km/h" - and step the throttle.
//
// Environment overrides for the trace case (all optional): PS_CRAWL_SURFACE (grass|grass_rg|asphalt),
// PS_CRAWL_GRADE (0.08), PS_CRAWL_THROTTLE (0.4), PS_CRAWL_TC (0/1), PS_CRAWL_V0 (0.4), PS_CRAWL_ANTI_STALL (rpm),
// PS_CRAWL_BITE (rpm), PS_CRAWL_FULL_MIN (rpm), PS_CRAWL_SECONDS (10), PS_CRAWL_TRACE_EVERY (ticks, 12).
#include "vehicle_test_helpers.h"

#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/math/transcendental.h"
#include "ps/vehicle/vehicle_reset.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

using namespace ps;

namespace {

std::string surfaces_path_for(const std::string& surface) {
    if (surface == "grass_rg") return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/assists/surfaces_racing_game.json";
    return std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json";
}

const vehicle::VehicleDesc& hyper_desc() {
    static const vehicle::VehicleDesc desc = [] {
        jobs::JobSystem pool(10);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_assists_diag_cache").string();
        std::filesystem::create_directories(opt.cache_dir);
        return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    }();
    return desc;
}

drivetrain::AutoClutchDesc* auto_clutch_of(vehicle::VehicleDesc& d) {
    for (auto& c : d.controllers)
        if (auto* m = std::get_if<drivetrain::ManualTcuDesc>(&c.params)) return &m->auto_clutch;
    return nullptr;
}

struct CrawlCase {
    std::string surface = "grass";
    real grade = 0.08;    // rise/run, > 0 climbs in +X
    real throttle = 0.4;
    bool tc = true;
    real v0 = 0.4;        // m/s along the slope at the throttle step; 0 = release from rest
    real anti_stall = -1; // < 0: keep the data value
    real bite = -1, full_min = -1;
    real seconds = 10.0;
    int trace_every = 0;  // ticks between trace lines; 0 = no trace
    int workers = 4;
    bool auto_shift = false;
    int gear = 1;
    real lift_at = -1, lift_for = 0; // throttle 0 over [lift_at, lift_at + lift_for) after the step; lift_at < 0: never
};

struct CrawlResult {
    real v_end = 0, s_end = 0, v_min = 1e9, v_max = -1e9;
    bool stalled = false;
    real t_stall = -1;
    real rpm_min = 1e9, rpm_mean_last = 0;
    real e_min = 1e9, e_max = -1e9;
    int ticks_below_anti_stall = 0, anti_stall_dumps = 0;
    int tc_ticks = 0;
    real tc_min_fraction = 1, tc_max_slip = 0, tc_est_err_max = 0;
    real kappa_max = 0, kappa_mean = 0;
    real util_max = 0, util_mean = 0; // rear fx / (mu_peak * lambda_mu * fz)
    real boost_max = 0, clutch_torque_max = 0;
    real f_req = 0, f_avail = 0, fz_rear_settled = 0;
    int ticks = 0;
};

// The rear tyre's nominal peak longitudinal friction at its own fz0 (pdx1 * lambda_mux, MF pure slip Dx/Fz at
// dfz = 0, camber 0) - the "available mu" scale for util. DERIVED from the loaded desc, not a measurement.
real rear_mu_peak(const vehicle::VehicleDesc& d) { return d.wheels[2].tyre.pdx1 * d.wheels[2].tyre.lambda_mux; }

CrawlResult run_crawl(const CrawlCase& c) {
    vehicle::VehicleDesc desc = hyper_desc();
    if (auto* ac = auto_clutch_of(desc)) {
        if (c.anti_stall > 0) ac->anti_stall_rpm = c.anti_stall;
        if (c.bite > 0) ac->bite_rpm = c.bite;
        if (c.full_min > 0) ac->full_rpm_min = c.full_min;
    }
    const real anti_stall_rpm = auto_clutch_of(desc) ? auto_clutch_of(desc)->anti_stall_rpm : 0;

    ps_test::VehicleTestRig rig;
    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = c.workers;
    rig.world = std::make_unique<World>(cfg);
    rig.surfaces = std::make_shared<io::SurfaceTable>(surfaces_path_for(c.surface));
    const SurfaceId sid = rig.surfaces->id_for(c.surface);
    const real theta = math::atan(-c.grade); // negative tilt about +Y: the plane RISES in +X (helper convention)
    const Quat tilt = Quat::from_axis_angle(Vec3::unit_y(), theta);
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

    w.set_control("assist.traction_control", c.tc ? 1.0 : 0.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("handbrake", 1.0);
    w.set_control("gear_request", static_cast<real>(c.gear));
    w.set_control("assist.auto_shift", c.auto_shift ? 1.0 : 0.0);
    ps_test::settle(w, 1.5);

    CrawlResult r;
    r.fz_rear_settled = w.wheel_telemetry(rig.vehicle, 2).fz + w.wheel_telemetry(rig.vehicle, 3).fz;
    const real mu_lambda = rear_mu_peak(desc) * rig.surfaces->lambda_mu_for(sid);
    const real g = 9.81, m = 1500.0;
    const real cth = 1.0 / std::sqrt(1.0 + c.grade * c.grade), sth = c.grade * cth;
    // Required steady tractive force: grade + surface rolling resistance (surface crr * N). The tyre's own qsy1 term
    // adds to it; reported separately by the measured fx at steady crawl, so not double counted here.
    r.f_req = m * g * (sth + rig.surfaces->crr_for(sid) * cth);
    r.f_avail = mu_lambda * r.fz_rear_settled;

    if (c.v0 != 0) {
        Motion mo;
        mo.linear = dir * c.v0;
        w.backend().set_motion(rig.chassis, mo);
        vehicle::VehicleResetOptions ro;
        ro.hubs_roll_with_chassis = true;
        ro.clutch_locked = false;
        w.reset_vehicle(rig.vehicle, ro);
    }
    w.set_control("handbrake", 0.0);
    if (c.auto_shift) w.set_control("gear_request", drivetrain::kGearRequestNone); // hand the gear to auto-shift
    w.set_control("throttle", c.throttle);

    const int n = static_cast<int>(c.seconds * 240.0 + 0.5);
    real prev_e = -1, rpm_sum_last = 0;
    int last_count = 0;
    if (c.trace_every > 0)
        std::printf("   t     v      s    rpm  st   e    slipRpm Tcl   thrCmd  tcFr  tcSlip tcEst  kapRL  kapRR  fxR    fzR   util boost idleI\n");
    for (int k = 0; k < n; ++k) {
        if (c.lift_at >= 0) {
            const real t = k / 240.0;
            w.set_control("throttle", (t >= c.lift_at && t < c.lift_at + c.lift_for) ? 0.0 : c.throttle);
        }
        w.step();
        const auto& pt = w.powertrain_state(rig.vehicle);
        const auto dbg = w.powertrain_debug(rig.vehicle);
        const real v = dot(w.get_motion(rig.chassis).linear, dir);
        const real s = dot(w.get_pose(rig.chassis).position, dir);
        const auto rl = w.wheel_telemetry(rig.vehicle, 2), rr = w.wheel_telemetry(rig.vehicle, 3);
        const real fxr = rl.fx + rr.fx, fzr = rl.fz + rr.fz;
        const real util = fzr > 1 ? fxr / (mu_lambda * fzr) : 0;
        const real kap = std::max(std::abs(rl.slip_ratio), std::abs(rr.slip_ratio));
        const real boost = pt.engines.empty() ? 0 : pt.engines[0].boost_bar;
        const real thr_cmd = pt.engines.empty() ? 0 : pt.engines[0].throttle_command;
        ++r.ticks;
        r.v_min = std::min(r.v_min, v);
        r.v_max = std::max(r.v_max, v);
        r.rpm_min = std::min(r.rpm_min, pt.engine_rpm);
        r.e_min = std::min(r.e_min, pt.clutch_engagement);
        r.e_max = std::max(r.e_max, pt.clutch_engagement);
        if (pt.engine_rpm < anti_stall_rpm) ++r.ticks_below_anti_stall;
        if (prev_e >= 0 && pt.clutch_engagement < prev_e - 1e-9 && pt.engine_rpm < anti_stall_rpm) ++r.anti_stall_dumps;
        prev_e = pt.clutch_engagement;
        if (pt.traction_control_intervening) ++r.tc_ticks;
        r.tc_min_fraction = std::min(r.tc_min_fraction, pt.traction_torque_fraction);
        r.tc_max_slip = std::max(r.tc_max_slip, pt.traction_positive_slip);
        if (c.tc) r.tc_est_err_max = std::max(r.tc_est_err_max, std::abs(pt.traction_estimated_speed_m_s - w.get_pose(rig.chassis).orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x));
        r.kappa_max = std::max(r.kappa_max, kap);
        r.kappa_mean += kap;
        r.util_max = std::max(r.util_max, util);
        r.util_mean += util;
        r.boost_max = std::max(r.boost_max, boost);
        r.clutch_torque_max = std::max(r.clutch_torque_max, std::abs(dbg.clutch_torque));
        if (!r.stalled && pt.engine_state == drivetrain::EngineState::Stalled) {
            r.stalled = true;
            r.t_stall = (k + 1) / 240.0;
        }
        if (k >= n - 720) {
            rpm_sum_last += pt.engine_rpm;
            ++last_count;
        }
        if (c.trace_every > 0 && (k % c.trace_every == 0 || k == n - 1))
            std::printf("%5.2f %6.3f %6.2f %5.0f %d %5.3f %6.0f %6.1f %5.3f %5.3f %6.3f %6.3f %6.3f %6.3f %6.0f %6.0f %5.2f %5.2f %6.3f\n",
                        (k + 1) / 240.0, v, s, pt.engine_rpm, int(pt.engine_state), pt.clutch_engagement, dbg.clutch_slip * 60 / 6.283185307179586,
                        dbg.clutch_torque, thr_cmd, pt.traction_torque_fraction, pt.traction_positive_slip, pt.traction_estimated_speed_m_s,
                        rl.slip_ratio, rr.slip_ratio, fxr, fzr, util, boost, dbg.idle_integrator);
    }
    r.v_end = dot(w.get_motion(rig.chassis).linear, dir);
    r.s_end = dot(w.get_pose(rig.chassis).position, dir);
    r.kappa_mean /= std::max(1, r.ticks);
    r.util_mean /= std::max(1, r.ticks);
    r.rpm_mean_last = last_count ? rpm_sum_last / last_count : 0;
    return r;
}

void print_header() {
    std::printf("surface  grade thr  tc  v0   AS   | v_end  s_end  v_min  stall t_st  rpm_min rpm_last e_min e_max <AS dumps | tc%%   tcMin tcSlip estErr | kapMax kapMean utilMax utilMean | boost Tclmax | Freq  Favail\n");
}
void print_row(const CrawlCase& c, const CrawlResult& r) {
    std::printf("%-8s %5.3f %4.2f %d %4.2f %5.0f | %6.3f %6.2f %6.3f %d %5.2f %6.0f %6.0f %5.3f %5.3f %4d %4d | %5.1f %5.3f %6.3f %6.3f | %6.3f %6.3f %6.3f %6.3f | %5.2f %6.0f | %5.0f %5.0f\n",
                c.surface.c_str(), c.grade, c.throttle, int(c.tc), c.v0, c.anti_stall, r.v_end, r.s_end, r.v_min, int(r.stalled), r.t_stall,
                r.rpm_min, r.rpm_mean_last, r.e_min, r.e_max, r.ticks_below_anti_stall, r.anti_stall_dumps,
                100.0 * r.tc_ticks / std::max(1, r.ticks), r.tc_min_fraction, r.tc_max_slip, r.tc_est_err_max, r.kappa_max, r.kappa_mean,
                r.util_max, r.util_mean, r.boost_max, r.clutch_torque_max, r.f_req, r.f_avail);
}

std::string env_string(const char* name) {
#if defined(_MSC_VER) || defined(_WIN32)
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return {};
    std::string out(buf);
    std::free(buf);
    return out;
#else
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
#endif
}

real env_real(const char* name, real def) {
    const std::string v = env_string(name);
    return v.empty() ? def : std::atof(v.c_str());
}

} // namespace

TEST_CASE("Assists Part A: hyper grass-slope crawl matrix", "[.][diag][traction_control][vehicle]") {
    print_header();
    const real seconds = env_real("PS_CRAWL_SECONDS", 10.0);
    for (const char* surf : {"grass", "grass_rg", "asphalt"}) {
        for (real grade : {0.03, 0.05, 0.08, 0.12}) {
            for (real thr : {0.2, 0.4, 0.6, 1.0}) {
                for (bool tc : {false, true}) {
                    for (real v0 : {0.0, 0.4}) {
                        CrawlCase c;
                        c.surface = surf;
                        c.grade = grade;
                        c.throttle = thr;
                        c.tc = tc;
                        c.v0 = v0;
                        c.seconds = seconds;
                        print_row(c, run_crawl(c));
                    }
                }
            }
        }
    }
}

TEST_CASE("Assists Part A: hyper crawl A/B on the auto-clutch thresholds", "[.][diag][traction_control][vehicle]") {
    print_header();
    const real seconds = env_real("PS_CRAWL_SECONDS", 10.0);
    struct Ab { real as, bite, full; };
    for (const Ab ab : {Ab{-1, -1, -1}, Ab{600, -1, -1}, Ab{600, 1100, 1600}, Ab{850, 1100, 1600}}) {
        std::printf("-- anti_stall %.0f bite %.0f full_min %.0f\n", ab.as, ab.bite, ab.full);
        for (const char* surf : {"grass", "grass_rg"}) {
            for (real grade : {0.05, 0.08}) {
                for (real thr : {0.2, 0.4, 1.0}) {
                    for (bool tc : {false, true}) {
                        CrawlCase c;
                        c.surface = surf;
                        c.grade = grade;
                        c.throttle = thr;
                        c.tc = tc;
                        c.v0 = 0.4;
                        c.anti_stall = ab.as;
                        c.bite = ab.bite;
                        c.full_min = ab.full;
                        c.seconds = seconds;
                        print_row(c, run_crawl(c));
                    }
                }
            }
        }
    }
}

TEST_CASE("Assists Part A: hyper crawl trace", "[.][diag][traction_control][vehicle]") {
    CrawlCase c;
    if (const std::string s = env_string("PS_CRAWL_SURFACE"); !s.empty()) c.surface = s;
    c.grade = env_real("PS_CRAWL_GRADE", c.grade);
    c.throttle = env_real("PS_CRAWL_THROTTLE", c.throttle);
    c.tc = env_real("PS_CRAWL_TC", 1.0) > 0.5;
    c.v0 = env_real("PS_CRAWL_V0", c.v0);
    c.anti_stall = env_real("PS_CRAWL_ANTI_STALL", -1);
    c.bite = env_real("PS_CRAWL_BITE", -1);
    c.full_min = env_real("PS_CRAWL_FULL_MIN", -1);
    c.seconds = env_real("PS_CRAWL_SECONDS", 10.0);
    c.trace_every = static_cast<int>(env_real("PS_CRAWL_TRACE_EVERY", 12));
    c.auto_shift = env_real("PS_CRAWL_AUTO_SHIFT", 0) > 0.5;
    c.gear = static_cast<int>(env_real("PS_CRAWL_GEAR", 1));
    c.lift_at = env_real("PS_CRAWL_LIFT_AT", -1);
    c.lift_for = env_real("PS_CRAWL_LIFT_FOR", 0);
    const auto r = run_crawl(c);
    print_header();
    print_row(c, r);
}

// Stall hunt: the matrix above never stalls with a constant throttle. These are the driver patterns a crawl on a
// slope produces: a lift and re-apply (the car stops and rolls back with the clutch open, then the throttle comes
// back while it rolls backwards), a start while already rolling backwards, 2nd gear, and auto-shift.
TEST_CASE("Assists Part A: hyper crawl stall hunt", "[.][diag][traction_control][vehicle]") {
    print_header();
    const real seconds = env_real("PS_CRAWL_SECONDS", 12.0);
    for (const char* surf : {"grass", "grass_rg"}) {
        for (real grade : {0.05, 0.08, 0.12}) {
            for (real thr : {0.4, 0.7, 1.0}) {
                for (bool tc : {false, true}) {
                    for (int pattern = 0; pattern < 5; ++pattern) {
                        CrawlCase c;
                        c.surface = surf;
                        c.grade = grade;
                        c.throttle = thr;
                        c.tc = tc;
                        c.seconds = seconds;
                        c.v0 = 0.4;
                        if (pattern == 1) { c.lift_at = 2.0; c.lift_for = 1.5; } // lift, roll back, re-apply
                        if (pattern == 2) c.v0 = -0.8;                            // already rolling backwards
                        if (pattern == 3) c.gear = 2;                             // 2nd gear crawl
                        if (pattern == 4) c.auto_shift = true;
                        std::printf("p%d ", pattern);
                        print_row(c, run_crawl(c));
                    }
                }
            }
        }
    }
}

// Free rev in neutral: net crank torque demand (PowertrainDebug::engine_torque_demand, the engine's own
// limiter-free net demand) versus rpm at a fixed throttle - what the engine offers the launch law at part throttle.
TEST_CASE("Assists Part A: hyper part-throttle torque in neutral", "[.][diag][traction_control][vehicle]") {
    for (real thr : {0.1, 0.2, 0.3, 0.4, 0.6, 1.0}) {
        const vehicle::VehicleDesc desc = hyper_desc();
        auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0.0, 4);
        World& w = *rig.world;
        w.set_control("gear_request", 0.0);
        w.set_control("handbrake", 1.0);
        ps_test::settle(w, 1.0);
        w.set_control("throttle", thr);
        std::printf("throttle %.2f:", thr);
        real last_rpm = 0;
        for (int k = 0; k < 240 * 4; ++k) {
            w.step();
            const auto& pt = w.powertrain_state(rig.vehicle);
            const auto dbg = w.powertrain_debug(rig.vehicle);
            if (k % 24 == 0) std::printf(" [%.0f rpm %.0f Nm]", pt.engine_rpm, dbg.engine_torque_demand);
            last_rpm = pt.engine_rpm;
        }
        std::printf(" -> settled %.0f rpm\n", last_rpm);
    }
}
