// Traction control speed-estimate drift (owner, PR #8: "fix the drift of sensed values").
//
// The sensor-only TC keeps a ground-speed estimate from the virtual IMU and the wheel speed sensors. These cases drive
// the full hypercar (car_hyper.json, RWD, Racing assist unit, TC mode On) through the manoeuvres that used to leave a
// lasting offset in that estimate - braking with locked wheels, a handbrake slide, slopes, a kerb, switching TC off and
// on - and then crawl with the pedal down, the situation of the owner's log (cut at 5-6 km/h for 17 s). The error is
// estimate minus the chassis's true longitudinal velocity (chassis frame x, at contact-patch height), read every tick; it is only scored while
// the estimator runs (pedal down, after its first 0.25 s).
//
// Bands (written before the fix was run): during the crawl phases |error| <= 0.10 m/s (0.05 slip at the 2 m/s floor,
// below the 0.08 target), and over the whole run |error| <= 0.5 m/s outside the 0.5 s after a slide or a kerb hit.
// The torque fraction may not sit below 0.1 for more than 0.3 s while the driven wheels' true slip ratio is below the
// target. A kerb hit is detected from the chassis itself (|dvx/dt| > 4 m/s^2 at 20 % throttle, or a pitch rate
// above 0.15 rad/s while it bounces); the crawl band is not applied until 0.5 s after the last such tick either
// (first run: a 0.33 m/s transient while the wheels ride the kerb, gone within 0.2 s), the transient itself is held to
// the 0.5 m/s whole-run band (impact_max).
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/traction_control.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <string>
using namespace ps;
using namespace ps::vehicle;
namespace {
const VehicleDesc& hyper() {
    static const VehicleDesc d = [] {
        jobs::JobSystem pool(10);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_assists_diag_cache").string();
        std::filesystem::create_directories(opt.cache_dir);
        return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper.json", opt);
    }();
    return d;
}
struct Controls { real throttle = 0, brake = 0, handbrake = 0, steer = 0, tc = 1; bool crawl = false, settle = false; };
struct DriftResult {
    real crawl_max = 0;     // max |error| while a crawl phase is scored
    real run_max = 0;       // max |error| over the whole scored run (outside the settle windows)
    real false_cut_s = 0;   // longest time with fraction < 0.1 while the true driven slip is below the target
    real impact_max = 0;    // max |error| inside the settle windows (slides, kerb hits)
    real v_end = 0;
};
// script(t) gives the controls of the tick at time t. grade > 0 descends in +X (the rig's convention).
DriftResult drive(const char* name, real seconds, real grade, const std::function<Controls(real)>& script,
                  const std::function<void(World&)>& extra_setup = {}, bool impact_settle = false) {
    VehicleDesc desc = hyper();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, grade, 4);
    World& w = *rig.world;
    if (extra_setup) extra_setup(w);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.traction_control", 1.0);
    w.set_control("handbrake", 1.0);
    ps_test::settle(w, .5);
    w.set_control("gear_request", 1);
    DriftResult r;
    real scored_for = 0, settle_left = 0, cut_run = 0, vx_prev = 0;
    const real contact_z = desc.wheels[0].attachment_local.z - desc.wheels[0].wheel_radius;
    const bool trace = std::getenv("PS_TC_DRIFT_TRACE") != nullptr;
    const int ticks = int(seconds * 240);
    for (int k = 0; k < ticks; ++k) {
        const real t = k / 240.0;
        const Controls c = script(t);
        w.set_control("throttle", c.throttle);
        w.set_control("brake", c.brake);
        w.set_control("handbrake", c.handbrake);
        w.set_control("steer", c.steer);
        w.set_control("assist.traction_control", c.tc);
        w.step();
        const auto pt = w.powertrain_state(rig.vehicle);
        const Pose p = w.get_pose(rig.chassis);
        // Truth at contact-patch height under the chassis origin (what a rolling wheel sees): x of v + w x (0, 0, z_c).
        // Without the pitch term a kerb bounce reads as a 0.1 m/s estimate error that is only the reference point.
        const Vec3 omega_body = p.orientation.inverse().rotate(w.get_motion(rig.chassis).angular);
        const real vx_origin = p.orientation.inverse().rotate(w.get_motion(rig.chassis).linear).x;
        const real vx = vx_origin + omega_body.y * contact_z;
        const real err = pt.traction_estimated_speed_m_s - vx;
        const bool impact = impact_settle && k > 0 && (std::abs(vx_origin - vx_prev) * 240 > 4.0 || std::abs(omega_body.y) > 0.15);
        vx_prev = vx_origin;
        if (c.settle || impact) settle_left = .5;
        else settle_left = std::max(real(0), settle_left - real(1.0 / 240));
        const bool running = c.tc > .5 && c.throttle > .02 && c.brake <= .05 && c.handbrake <= .05;
        scored_for = running ? scored_for + 1.0 / 240 : 0;
        if (scored_for > .25) {
            if (settle_left <= 0) r.run_max = std::max(r.run_max, std::abs(err));
            else r.impact_max = std::max(r.impact_max, std::abs(err));
            if (c.crawl && settle_left <= 0) r.crawl_max = std::max(r.crawl_max, std::abs(err));
            real true_slip = 0;
            for (std::size_t i : {std::size_t(2), std::size_t(3)}) true_slip = std::max(true_slip, w.wheel_telemetry(rig.vehicle, i).slip_ratio);
            if (pt.traction_torque_fraction < .1 && true_slip < .08) { cut_run += 1.0 / 240; r.false_cut_s = std::max(r.false_cut_s, cut_run); }
            else cut_run = 0;
        } else cut_run = 0;
        if (trace && (k % 24 == 0 || (std::getenv("PS_TC_DRIFT_TRACE")[0] == '2' && std::abs(err) > .08 && settle_left <= 0)))
            std::printf("  %s t=%.2f vx=%.3f est=%.3f err=%+.3f frac=%.3f slip=%.3f thr=%.2f br=%.2f hb=%.2f st=%.2f sl=%.2f qz=%.3f fw=%.3f\n", name, t, vx,
                        pt.traction_estimated_speed_m_s, err, pt.traction_torque_fraction, pt.traction_positive_slip, c.throttle, c.brake,
                        c.handbrake, c.steer, settle_left, p.orientation.inverse().rotate(w.get_motion(rig.chassis).angular).y,
                        w.wheel_omega(rig.vehicle, 0) * desc.wheels[0].wheel_radius);
        r.v_end = vx;
    }
    std::printf("TC drift %-18s crawl_max=%.3f m/s run_max=%.3f m/s impact_max=%.3f m/s false_cut=%.2f s v_end=%.2f m/s\n", name,
                r.crawl_max, r.run_max, r.impact_max, r.false_cut_s, r.v_end);
    return r;
}
void check_bounded(const DriftResult& r) {
    CHECK(r.crawl_max <= 0.10);
    CHECK(r.run_max <= 0.5);
    CHECK(r.false_cut_s <= 0.3);
}
} // namespace

TEST_CASE("TC speed estimate stays bounded after hard braking with locked wheels", "[traction_control][vehicle][drift]") {
    // Launch, 3 s full throttle, brake hard (no ABS: the wheels lock) to a near stop, crawl on 15 % throttle.
    check_bounded(drive("brake_lock", 16, 0, [](real t) {
        Controls c;
        if (t < 3) c.throttle = 1;
        else if (t < 4.6) c.brake = 1;
        else { c.throttle = .15; c.crawl = t > 6; }
        return c;
    }));
}
TEST_CASE("TC speed estimate stays bounded after a handbrake slide", "[traction_control][vehicle][drift]") {
    check_bounded(drive("handbrake_slide", 16, 0, [](real t) {
        Controls c;
        if (t < 2.5) c.throttle = 1;
        else if (t < 4.0) { c.handbrake = 1; c.steer = .8; c.settle = true; }
        else if (t < 5.0) { c.steer = .3; c.settle = true; c.throttle = .1; }
        else { c.throttle = .15; c.crawl = t > 7; }
        return c;
    }));
}
TEST_CASE("TC speed estimate stays bounded through a power slide", "[traction_control][vehicle][drift]") {
    // Also the settle window right after TC comes back on is held to 0.25 m/s (band added after the first runs: with a
    // stale IMU sample from before TC was switched off the first tick read a 4.8 m/s jump here; the fix samples the
    // IMU every tick). Same in tc_toggle.
    const DriftResult r = drive("power_slide", 16, 0, [](real t) {
        Controls c;
        if (t < 2.5) c.throttle = 1;
        else if (t < 6.0) { c.throttle = 1; c.steer = .7; c.tc = 0; c.settle = true; } // TC off: a real slide
        else if (t < 7.5) { c.throttle = .2; c.settle = true; }
        else { c.throttle = .15; c.crawl = t > 9; }
        return c;
    });
    check_bounded(r);
    CHECK(r.impact_max <= 0.25);
}
TEST_CASE("TC speed estimate stays bounded on a long crawl up and down a slope", "[traction_control][vehicle][drift]") {
    // grade < 0 climbs in +X; pedal pumped to pitch the body.
    for (real grade : {-0.12, 0.12}) {
        check_bounded(drive(grade < 0 ? "slope_up" : "slope_down", 20, grade, [](real t) {
            Controls c;
            const int phase = int(t / 1.5) % 3;
            c.throttle = phase == 0 ? .35 : (phase == 1 ? .15 : .25);
            if (t > 9 && t < 9.6) { c.throttle = 0; c.brake = .6; }
            c.crawl = t > 2;
            return c;
        }));
    }
}
TEST_CASE("TC speed estimate stays bounded over a kerb", "[traction_control][vehicle][drift]") {
    const DriftResult r = drive("kerb", 14, 0, [](real t) {
        Controls c;
        c.throttle = .2;
        c.crawl = t > 2;
        return c;
    }, [](World& w) {
        for (real x : {3.0, 7.0}) {
            BodyDesc kerb;
            kerb.shape = BoxShape{Vec3{.15, 6, .06}};
            kerb.motion = BodyMotionType::Static;
            kerb.friction = 1.0;
            kerb.pose.position = Vec3{x, 0, .06};
            w.create_body(kerb);
        }
    }, true);
    check_bounded(r);
    CHECK(r.impact_max <= 0.5);
}
TEST_CASE("TC speed estimate stays bounded when TC is switched off and on", "[traction_control][vehicle][drift]") {
    const DriftResult r = drive("tc_toggle", 12, 0, [](real t) {
        Controls c;
        c.throttle = t < 4 ? .6 : .15;
        if (t > 1.5 && t < 3.0) c.tc = 0;
        if (t > 3.0 && t < 3.5) c.settle = true;
        c.crawl = t > 6;
        return c;
    });
    check_bounded(r);
    CHECK(r.impact_max <= 0.25);
}
