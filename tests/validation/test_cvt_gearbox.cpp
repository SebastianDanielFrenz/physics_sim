// tests/validation/test_cvt_gearbox.cpp - CVT plan C1: the push-belt CVT with a torque converter
// (tests/support/fixtures/vehicles/car_sedan_cvt.json, data/gearboxes/sedan_cvt.json) and with a wet starting clutch
// (car_sedan_cvt_clutch.json, data/gearboxes/sedan_cvt_clutch.json) on the reference sedan body, driven through World
// like a game; plus the dual-clutch box's launch control through the assist unit (car_sedan_dct.json + the shipped
// Racing unit) against the plain DCT launch.
//
// CVT: creep at idle in D, a WOT launch (no stall; the starting clutch slips, then locks, and the variator stays at its
// launch ratio until then), the ratio sweep of a WOT run (monotonic, inside [ratio_high, ratio_low], never faster than
// the rate limit), the engine held at the ratio law's WOT target while the ratio is between its limits, the stepped S
// mode and the paddles, the belt slipping (and booked into clutch_loss) when its clamping force is capped below the
// torque, and 1 vs 10 workers equal hash every tick. DCT launch control: armed (brake + throttle, revs at the launch
// rpm, car still), released on the brake, the driven-wheel slip regulated near the mode's target, 0-100 km/h against
// the plain launch. Every case closes the per-tick energy ledger and prints its measured numbers.

#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kMass = 1500.0;
const Vec3 kHalfExtents{2.0, 0.4, 0.15};
constexpr real kOriginHeight = 0.6;
constexpr real kD = 99.0; // gear_request: plain D

std::string fixture(const char* name) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/" + name; }
std::uint64_t ticks_for(real seconds, real dt) { return static_cast<std::uint64_t>(seconds / dt + 0.5); }

real sum_abs_terms(const drivetrain::PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) +
           std::abs(e.gear_loss) + std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) +
           std::abs(e.engine_friction_loss) + std::abs(e.engagement_loss) + std::abs(e.starter_in) +
           std::abs(e.external_reset) + std::abs(e.diff_loss) + std::abs(e.converter_loss);
}

const drivetrain::GearboxDesc& gearbox_of(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) return *g;
    throw std::runtime_error("no gearbox");
}
drivetrain::GearboxDesc& gearbox_of(vehicle::VehicleDesc& d) {
    for (auto& c : d.powertrain.components)
        if (auto* g = std::get_if<drivetrain::GearboxDesc>(&c.params)) return *g;
    throw std::runtime_error("no gearbox");
}

struct Run {
    ps_test::VehicleTestRig rig;
    real max_balance_ratio = 0.0;
    bool stalled = false;
    explicit Run(const vehicle::VehicleDesc& desc, int workers = 4)
        : rig(ps_test::build_vehicle_rig_from_desc(desc, kMass, kHalfExtents, kOriginHeight, 0.0, workers, Vec3::unit_y(), 240.0,
                                                   960.0, 3000.0)) {}
    explicit Run(const char* file, int workers = 4) : Run(io::load_vehicle_json(fixture(file)), workers) {}
    World& w() { return *rig.world; }
    const drivetrain::PowertrainSnapshot& pt() { return w().powertrain_state(rig.vehicle); }
    real speed() { return w().get_motion(rig.chassis).linear.length(); }
    real front_slip() { return real(0.5) * (w().wheel_telemetry(rig.vehicle, 0).slip_ratio + w().wheel_telemetry(rig.vehicle, 1).slip_ratio); }
    void step() {
        w().step();
        const drivetrain::PowertrainEnergy& e = w().powertrain_energy(rig.vehicle);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) max_balance_ratio = std::max(max_balance_ratio, std::abs(e.balance_error) / denom);
        if (pt().engine_state != drivetrain::EngineState::Running) stalled = true;
    }
    void run(real seconds) {
        const std::uint64_t n = ticks_for(seconds, w().dt());
        for (std::uint64_t i = 0; i < n; ++i) step();
    }
};

} // namespace

TEST_CASE("CVT C1: creep at idle in D (converter and starting clutch), standing on the brake", "[validation][vehicle][drivetrain][automatic][cvt]") {
    for (const char* file : {"car_sedan_cvt.json", "car_sedan_cvt_clutch.json"}) {
        INFO(file);
        Run r(file);
        ps_test::settle(r.w(), 1.0);
        REQUIRE(r.pt().transmission_kind == 3);
        r.w().set_control("brake", 1.0);
        r.w().set_control("gear_request", kD);
        r.run(3.0);
        const real speed_braked = r.speed();
        const real rpm_braked = r.pt().engine_rpm;
        CHECK(r.pt().gear == 1);
        r.w().set_control("brake", 0.0);
        r.run(8.0);
        const auto& s = r.pt();
        std::printf("[cvt creep] %s braked rpm %.0f speed %.4f | after 8 s speed %.3f m/s rpm %.0f ratio %.3f e %.3f balance %.2e\n", file,
                    static_cast<double>(rpm_braked), static_cast<double>(speed_braked), static_cast<double>(r.speed()),
                    static_cast<double>(s.engine_rpm), static_cast<double>(s.cvt_ratio), static_cast<double>(s.clutch_engagement),
                    static_cast<double>(r.max_balance_ratio));
        CHECK(speed_braked < 0.05);
        CHECK(r.speed() > 0.5);
        CHECK(r.speed() < 4.0);
        CHECK(std::abs(s.cvt_ratio - 2.63) < 0.05); // creep stays in the launch ratio
        CHECK_FALSE(r.stalled);
        CHECK(r.max_balance_ratio <= 5e-8);
    }
}

TEST_CASE("CVT C1: WOT launch, ratio sweep, the engine held at the WOT target rpm", "[validation][vehicle][drivetrain][automatic][cvt]") {
    for (const char* file : {"car_sedan_cvt.json", "car_sedan_cvt_clutch.json"}) {
        INFO(file);
        const vehicle::VehicleDesc desc = io::load_vehicle_json(fixture(file));
        const drivetrain::CvtDesc& cvt = *gearbox_of(desc).cvt;
        Run r(desc);
        ps_test::settle(r.w(), 1.0);
        r.w().set_control("gear_request", kD);
        r.run(0.5);
        r.w().set_control("throttle", 1.0);
        const real dt = r.w().dt();
        const real target = 5600.0; // the fixture's target_rpm_vs_throttle at full throttle
        real prev_ratio = r.pt().cvt_ratio;
        real max_rate = 0.0, min_ratio = 1e9, max_ratio = 0.0, max_rise = 0.0;
        real launch_peak_rpm = 0.0, t_lock = -1.0, ratio_at_lock = 0.0;
        real hold_sum = 0.0, hold_max_dev = 0.0, t_100 = -1.0;
        std::uint64_t hold_n = 0, belt_slip_ticks = 0;
        const std::uint64_t n = ticks_for(30.0, dt);
        for (std::uint64_t i = 0; i < n; ++i) {
            r.step();
            const auto& s = r.pt();
            const real t = static_cast<real>(i + 1) * dt;
            const real ratio = s.cvt_ratio;
            max_rate = std::max(max_rate, std::abs(std::log(ratio / prev_ratio)) / dt);
            max_rise = std::max(max_rise, ratio - prev_ratio);
            min_ratio = std::min(min_ratio, ratio);
            max_ratio = std::max(max_ratio, ratio);
            prev_ratio = ratio;
            if (t_lock < 0.0) {
                launch_peak_rpm = std::max(launch_peak_rpm, s.engine_rpm);
                const bool locked = s.clutch_engagement >= 1.0 && (s.lockup_engagement >= 1.0 || !gearbox_of(desc).converter);
                if (locked) {
                    t_lock = t;
                    ratio_at_lock = ratio;
                }
            }
            if (t_100 < 0.0 && r.speed() >= 100.0 / 3.6) t_100 = t;
            if (s.cvt_belt_slip_rad_s > 1.0) ++belt_slip_ticks;
            if (t_lock > 0.0 && t > t_lock + 1.0 && ratio < 0.97 * cvt.ratio_low && ratio > 1.03 * cvt.ratio_high) {
                hold_sum += s.engine_rpm;
                hold_max_dev = std::max(hold_max_dev, std::abs(s.engine_rpm - target));
                ++hold_n;
            }
        }
        const real hold_mean = hold_n > 0 ? hold_sum / static_cast<real>(hold_n) : 0.0;
        std::printf("[cvt wot] %s: launch peak rpm %.0f, locked at %.2f s (ratio %.3f), 0-100 km/h %.2f s, speed@30s %.1f m/s | "
                    "ratio %.3f..%.3f end %.3f, max |dln r/dt| %.3f /s (limit %.2f), max rise %.2e | hold over %.1f s: mean %.0f rpm, "
                    "max dev %.0f rpm | belt slip ticks %llu | balance %.2e\n",
                    file, static_cast<double>(launch_peak_rpm), static_cast<double>(t_lock), static_cast<double>(ratio_at_lock),
                    static_cast<double>(t_100), static_cast<double>(r.speed()), static_cast<double>(min_ratio),
                    static_cast<double>(max_ratio), static_cast<double>(r.pt().cvt_ratio), static_cast<double>(max_rate),
                    static_cast<double>(cvt.ratio_rate_per_s), static_cast<double>(max_rise), static_cast<double>(hold_n) * dt,
                    static_cast<double>(hold_mean), static_cast<double>(hold_max_dev), static_cast<unsigned long long>(belt_slip_ticks),
                    static_cast<double>(r.max_balance_ratio));
        CHECK_FALSE(r.stalled);
        CHECK(t_lock > 0.0);
        CHECK(t_lock < 6.0);
        CHECK(min_ratio >= cvt.ratio_high);
        CHECK(max_ratio <= cvt.ratio_low);
        CHECK(max_rate <= cvt.ratio_rate_per_s * 1.0001); // the rate limit (continuous mode)
        // The law follows the output speed, which rings slightly after the lock-up; the ratio never lengthens by more than that.
        CHECK(max_rise <= 0.01);
        CHECK(r.pt().cvt_ratio < 1.0);                    // swept well past 1:1 by 30 s
        CHECK(launch_peak_rpm > 1800.0);
        CHECK(launch_peak_rpm < 6600.0);
        REQUIRE(hold_n * dt > 5.0);
        CHECK(std::abs(hold_mean - target) < 100.0); // the variator holds the engine at the law's WOT target
        CHECK(hold_max_dev < 250.0);
        CHECK(t_100 > 0.0);
        CHECK(t_100 < 16.0);
        CHECK(belt_slip_ticks == 0); // the clamping force follows the torque: no belt slip in a normal run
        CHECK(r.max_balance_ratio <= 5e-8);
    }
}

TEST_CASE("CVT C1: part throttle ratio follows the law; stepped S mode and paddles", "[validation][vehicle][drivetrain][automatic][cvt]") {
    // Part throttle: the engine sits at the half-throttle target once the ratio is off its launch stop.
    {
        Run r("car_sedan_cvt.json");
        ps_test::settle(r.w(), 1.0);
        r.w().set_control("gear_request", kD);
        r.w().set_control("throttle", 0.5);
        r.run(15.0);
        const auto& s = r.pt();
        // target_rpm_vs_throttle(0.5) = 2600 rpm; min_rpm_vs_output_rpm stays below it.
        std::printf("[cvt part throttle] speed %.1f m/s rpm %.0f ratio %.3f lock-up %.2f\n", static_cast<double>(r.speed()),
                    static_cast<double>(s.engine_rpm), static_cast<double>(s.cvt_ratio), static_cast<double>(s.lockup_engagement));
        CHECK(std::abs(s.engine_rpm - 2600.0) < 50.0);
        CHECK(s.cvt_ratio < 2.0);
        CHECK(s.lockup_engagement == 1.0);
        CHECK_FALSE(r.stalled);
    }
    // S: the seven virtual steps on the sport schedule, one at a time, the ratio settling exactly on each step.
    {
        const vehicle::VehicleDesc desc = io::load_vehicle_json(fixture("car_sedan_cvt.json"));
        const std::vector<real>& steps = gearbox_of(desc).cvt->steps;
        Run r(desc);
        ps_test::settle(r.w(), 1.0);
        r.w().set_control("gear_request", kD);
        r.w().set_control("transmission.mode", 1.0);
        r.w().set_control("throttle", 1.0);
        std::vector<int> seq;
        std::vector<real> t_seq;
        bool settled_on_steps = true;
        real max_rpm = 0.0;
        const real dt = r.w().dt();
        const std::uint64_t n = ticks_for(25.0, dt);
        int since_change = 0;
        for (std::uint64_t i = 0; i < n; ++i) {
            r.step();
            const auto& s = r.pt();
            max_rpm = std::max(max_rpm, s.engine_rpm);
            if (seq.empty() || s.cvt_step != seq.back()) {
                seq.push_back(s.cvt_step);
                t_seq.push_back(static_cast<real>(i + 1) * dt);
                since_change = 0;
            } else if (++since_change == 60 && s.cvt_step > 0) { // 0.25 s on the step
                if (std::abs(s.cvt_ratio - steps[static_cast<std::size_t>(s.cvt_step - 1)]) > 1e-12) settled_on_steps = false;
            }
        }
        std::printf("[cvt stepped S] steps:");
        for (std::size_t k = 0; k < seq.size(); ++k) std::printf(" %d@%.2fs", seq[k], static_cast<double>(t_seq[k]));
        std::printf(" | speed %.1f m/s, max rpm %.0f, balance %.2e\n", static_cast<double>(r.speed()), static_cast<double>(max_rpm),
                    static_cast<double>(r.max_balance_ratio));
        REQUIRE(seq.size() >= 4);
        CHECK(seq.front() == 1); // S starts on the step nearest the launch ratio
        for (std::size_t k = 1; k < seq.size(); ++k) CHECK(seq[k] == seq[k - 1] + 1);
        CHECK(seq.back() >= 4);
        CHECK(settled_on_steps);
        CHECK(max_rpm > 6000.0); // shifts on the sport up line (6500 at full throttle), not the continuous target
        CHECK(r.pt().transmission_sport_mode);
        CHECK_FALSE(r.stalled);
        CHECK(r.max_balance_ratio <= 5e-8);
        // Paddles: two downshift presses in D (mode back to D) take two steps down and hold them (manual mode).
        r.w().set_control("transmission.mode", 0.0);
        r.w().set_control("throttle", 0.3);
        r.run(2.0);
        const int step0 = r.pt().cvt_step;
        CHECK(step0 == 0); // back to the continuous law in D
        r.w().set_control("shift_down_count", 1.0);
        r.run(0.5);
        const int step1 = r.pt().cvt_step;
        r.w().set_control("shift_down_count", 2.0);
        r.run(1.0);
        std::printf("[cvt paddles] continuous step %d, after 1 press %d, after 2 presses %d (manual %d) ratio %.3f\n", step0, step1,
                    r.pt().cvt_step, r.pt().transmission_manual_mode ? 1 : 0, static_cast<double>(r.pt().cvt_ratio));
        CHECK(r.pt().transmission_manual_mode);
        CHECK(step1 >= 1);
        CHECK(r.pt().cvt_step == step1 - 1);
        CHECK(std::abs(r.pt().cvt_ratio - steps[static_cast<std::size_t>(r.pt().cvt_step - 1)]) < 1e-12);
        CHECK_FALSE(r.stalled);
    }
}

TEST_CASE("CVT C1: a belt whose clamping force is capped below the torque slips, booked as clutch loss", "[validation][vehicle][drivetrain][automatic][cvt]") {
    vehicle::VehicleDesc desc = io::load_vehicle_json(fixture("car_sedan_cvt.json"));
    drivetrain::CvtBeltDesc& belt = gearbox_of(desc).cvt->belt;
    // Cap the clamping force so the belt carries ~100 N*m at the launch ratio (per N: 2 mu R_p / cos(beta)).
    const real rp = belt.pulley_radius_sum_m / (1.0 + gearbox_of(desc).cvt->ratio_low);
    const real per_n = 2.0 * belt.friction_coefficient * rp / std::cos(belt.sheave_half_angle_deg * 3.14159265358979323846 / 180.0);
    belt.clamp_force_max_n = 100.0 / per_n;
    belt.clamp_force_min_n = std::min(belt.clamp_force_min_n, belt.clamp_force_max_n);
    Run r(desc);
    ps_test::settle(r.w(), 1.0);
    r.w().set_control("gear_request", kD);
    r.run(0.5);
    const real clutch0 = r.w().powertrain_energy(r.rig.vehicle).clutch_loss;
    r.w().set_control("throttle", 1.0);
    std::uint64_t slip_ticks = 0;
    real max_slip = 0.0, cap_seen = 0.0;
    const std::uint64_t n = ticks_for(4.0, r.w().dt());
    for (std::uint64_t i = 0; i < n; ++i) {
        r.step();
        if (r.pt().cvt_belt_slip_rad_s > 1.0) ++slip_ticks;
        max_slip = std::max(max_slip, r.pt().cvt_belt_slip_rad_s);
        cap_seen = std::max(cap_seen, r.pt().cvt_belt_capacity_nm);
    }
    const real clutch_loss = r.w().powertrain_energy(r.rig.vehicle).clutch_loss - clutch0;
    std::printf("[cvt belt slip] capacity <= %.1f N*m, slipping %.2f s of 4, max slip %.0f rad/s, clutch_loss %.0f J, speed %.1f, balance %.2e\n",
                static_cast<double>(cap_seen), static_cast<double>(slip_ticks) * r.w().dt(), static_cast<double>(max_slip),
                static_cast<double>(clutch_loss), static_cast<double>(r.speed()), static_cast<double>(r.max_balance_ratio));
    CHECK(cap_seen <= 100.0 * 1.0001);
    CHECK(slip_ticks * r.w().dt() > 1.0);
    CHECK(clutch_loss > 1000.0);
    CHECK(r.max_balance_ratio <= 5e-8);
}

TEST_CASE("DualClutch launch control: armed on brake + throttle, slip-controlled release, against the plain DCT launch",
          "[validation][vehicle][drivetrain][automatic][launch]") {
    const std::string racing = std::string(PS_SOURCE_DIR) + "/data/electronics/assist_unit_racing.json";
    io::VehicleLoadOptions opt;
    opt.electronics["assists"] = racing;
    const vehicle::VehicleDesc lc_desc = io::load_vehicle_json(fixture("car_sedan_dct.json"), opt);
    const real launch_rpm = lc_desc.assist_unit->launch_control->launch_rpm;
    const real target = lc_desc.assist_unit->launch_control->slip_target.front() == 0.0 ? 0.10 : 0.10; // mode On = the car's TC

    struct Result {
        real t_100 = -1.0, slip_mean = 0.0, slip_max = 0.0, rpm_release = 0.0, armed_speed = 0.0, armed_rpm_min = 1e9, armed_rpm_max = 0.0;
        bool armed_seen = false, launch_seen = false, stalled = false;
        real balance = 0.0;
    };
    auto drive = [&](bool with_lc) {
        Result res;
        Run r(lc_desc);
        ps_test::settle(r.w(), 1.0);
        r.w().set_control("gear_request", kD);
        r.w().set_control("assist.launch_control", with_lc ? 1.0 : 0.0);
        r.run(0.5);
        const real dt = r.w().dt();
        if (with_lc) {
            r.w().set_control("brake", 1.0);
            r.w().set_control("throttle", 1.0);
            const std::uint64_t na = ticks_for(1.5, dt);
            for (std::uint64_t i = 0; i < na; ++i) {
                r.step();
                if (r.pt().launch_phase == 1) res.armed_seen = true;
                if (i + ticks_for(0.5, dt) >= na) { // the last 0.5 s of the hold
                    res.armed_rpm_min = std::min(res.armed_rpm_min, r.pt().engine_rpm);
                    res.armed_rpm_max = std::max(res.armed_rpm_max, r.pt().engine_rpm);
                }
            }
            res.armed_speed = r.speed();
            res.rpm_release = r.pt().engine_rpm;
            r.w().set_control("brake", 0.0);
        } else {
            r.w().set_control("throttle", 1.0);
        }
        real sum = 0.0;
        std::uint64_t cnt = 0;
        const std::uint64_t n = ticks_for(15.0, dt);
        for (std::uint64_t i = 0; i < n; ++i) {
            r.step();
            const real t = static_cast<real>(i + 1) * dt;
            if (r.pt().launch_phase == 2) res.launch_seen = true;
            const real k = r.front_slip();
            if (t >= 0.2 && t <= 1.0) {
                sum += k;
                ++cnt;
            }
            if (t <= 2.0) res.slip_max = std::max(res.slip_max, k);
            if (res.t_100 < 0.0 && r.speed() >= 100.0 / 3.6) res.t_100 = t;
        }
        res.slip_mean = cnt > 0 ? sum / static_cast<real>(cnt) : 0.0;
        res.stalled = r.stalled;
        res.balance = r.max_balance_ratio;
        return res;
    };
    const Result plain = drive(false);
    const Result lc = drive(true);
    std::printf("[dct launch control] launch rpm %.0f: armed rpm %.0f..%.0f (last 0.5 s), speed while armed %.3f m/s, rpm at release %.0f | "
                "LC: front slip mean(0.2-1.0 s) %.3f (target %.2f) max %.3f, 0-100 km/h %.2f s | plain DCT launch: slip mean %.3f max %.3f, "
                "0-100 km/h %.2f s | balance %.2e / %.2e\n",
                static_cast<double>(launch_rpm), static_cast<double>(lc.armed_rpm_min), static_cast<double>(lc.armed_rpm_max),
                static_cast<double>(lc.armed_speed), static_cast<double>(lc.rpm_release), static_cast<double>(lc.slip_mean),
                static_cast<double>(target), static_cast<double>(lc.slip_max), static_cast<double>(lc.t_100),
                static_cast<double>(plain.slip_mean), static_cast<double>(plain.slip_max), static_cast<double>(plain.t_100),
                static_cast<double>(lc.balance), static_cast<double>(plain.balance));
    CHECK(lc.armed_seen);
    CHECK(lc.launch_seen);
    CHECK_FALSE(plain.launch_seen);
    CHECK(lc.armed_speed < 0.05); // the launch clutch is held open on the brake
    CHECK(lc.armed_rpm_min > launch_rpm - 400.0);
    CHECK(lc.armed_rpm_max < launch_rpm + 100.0);
    CHECK(std::abs(lc.slip_mean - target) < 0.05); // the clutch regulates the driven-wheel slip at the mode's target
    CHECK(lc.t_100 > 0.0);
    CHECK(plain.t_100 > 0.0);
    CHECK(lc.t_100 <= plain.t_100); // the held-rpm launch is at least as quick as the plain clutch-slip launch
    CHECK_FALSE(lc.stalled);
    CHECK_FALSE(plain.stalled);
    CHECK(lc.balance <= 5e-8);
}

TEST_CASE("CVT C1 / DCT launch control: 1 vs 10 workers identical state hash every tick", "[validation][vehicle][drivetrain][automatic][cvt][determinism]") {
    const std::string racing = std::string(PS_SOURCE_DIR) + "/data/electronics/assist_unit_racing.json";
    io::VehicleLoadOptions opt;
    opt.electronics["assists"] = racing;
    struct Case {
        const char* name;
        vehicle::VehicleDesc desc;
        bool lc;
    };
    std::vector<Case> cases;
    cases.push_back({"car_sedan_cvt", io::load_vehicle_json(fixture("car_sedan_cvt.json")), false});
    cases.push_back({"car_sedan_cvt_clutch", io::load_vehicle_json(fixture("car_sedan_cvt_clutch.json")), false});
    cases.push_back({"car_sedan_dct + launch control", io::load_vehicle_json(fixture("car_sedan_dct.json"), opt), true});
    for (const Case& c : cases) {
        INFO(c.name);
        Run a(c.desc, 1), b(c.desc, 10);
        bool same = true;
        auto drive = [&](real seconds) {
            const std::uint64_t n = ticks_for(seconds, a.w().dt());
            for (std::uint64_t i = 0; i < n; ++i) {
                a.step();
                b.step();
                if (a.w().state_hash() != b.w().state_hash()) same = false;
            }
        };
        auto set = [&](const char* ch, real v) {
            a.w().set_control(ch, v);
            b.w().set_control(ch, v);
        };
        set("gear_request", kD);
        if (c.lc) set("assist.launch_control", 1.0);
        drive(1.0);
        if (c.lc) set("brake", 1.0);
        set("throttle", 1.0);
        drive(1.5);
        set("brake", 0.0);
        drive(5.0);
        set("transmission.mode", 1.0);
        drive(3.0);
        set("throttle", 0.2);
        set("transmission.mode", 0.0);
        drive(3.0);
        std::printf("[cvt determinism] %s gear %d ratio %.3f hash 0x%016llx\n", c.name, a.pt().gear, static_cast<double>(a.pt().cvt_ratio),
                    static_cast<unsigned long long>(a.w().state_hash()));
        CHECK(same);
    }
}
