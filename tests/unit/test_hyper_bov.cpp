// tests/unit/test_hyper_bov.cpp -- blow-off valve on the shipped hypercar turbo configurations (2026-10-13).
//
// Boost B0 holds the wastegate through a gearshift. car_hyper / car_hyper_n2o had no blow-off valve, so the closed-plate
// pressure spike during the shift cut rose from 4.75-5.36 to 5.6-6.1 bar gauge [M]. Owner decision 2026-10-13: add a
// valve to hyper_twin_default.json and hyper_twin_n2o.json (sized by the rule of tests/support/fixtures/
// turbo_config_hyper_bov.json, see "source" in the data).
//
// Bands committed in out/hyper_bov_progress.md before the first run (never widened):
//   H1 [long] stock car_hyper_n2o WOT from rest with auto-shift, 40 s: the charge peak from the shift start (the TCU's
//      phase leaves Idle) to 0.5 s after the new gear engages is <= 3.0 bar gauge for every upshift 1->2..6->7 [AS: the
//      1.8 bar target plus 1.2 bar for the wastegate overshoot; the valve-less spike is 5.6-6.1 bar, B0]; the valve
//      opens (bov_opening >= 0.5) in at least 4 of the 6 shifts
//   H2 [long] idle (3 s) and PI speed holds at 60 and 120 km/h (the last >= 8 s with no gear change): bov_opening <= 0.001
//   H3 engine level, both cars' engines, crank held at 3000/5000/7000 rpm, WOT 6 s: the mean net torque of the last 0.5 s
//      with the shipped valve is within 0.5 % of the same desc without it; bov_opening <= 0.01 throughout
// The recovery bands of test_boost_shift_hold V1 stay as committed.
//
// Round 3 (2026-10-14, out/hyper_bov_r3_progress.md): on the pedal-P1 master the round-1 valve (920 mm2) failed H1 (2->3
// peak 3.03 bar) and V1 (2->3 recovery 2.0 s); the valve-less spike on P1 is 4.7-8.2 bar gauge [M]. Diagnosis: TurboPair is
// already the quasi-steady plenum flow balance (compressor flow on the speed line = engine + vent, work = flow x isentropic
// head / eta at that point) - H5 checks the lift-off against an independent re-derivation of that balance. No model change;
// the shipped valve is resized to 1330 mm2 per turbo, tau 20 ms (see "source" in the data).

#include "vehicle_test_helpers.h"

#include "drivetrain/engine_simulated.h"

#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace {

using namespace ps::drivetrain;
using ps::real;

constexpr real kPamb = 101325.0;
constexpr real kPi = 3.14159265358979323846;

// Diagnostic knob: an environment variable as a number, `def` when unset (or NaN-free fallback).
real env_real(const char* name, real def) {
#if defined(_MSC_VER) || defined(_WIN32)
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return def;
    const real v = std::stod(buf);
    std::free(buf);
    return v;
#else
    const char* e = std::getenv(name);
    return e ? std::stod(e) : def;
#endif
}

ps::vehicle::VehicleDesc load_car(const char* file) {
    ps::jobs::JobSystem pool(10);
    ps::io::VehicleLoadOptions opt;
    opt.maps.pool = &pool;
    const std::string dir = std::string(PS_SOURCE_DIR) + "/out/hyper_bov_map_cache";
    std::filesystem::create_directories(dir);
    opt.maps.cache_dir = dir;
    return ps::io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/" + file, opt);
}

const SimulatedEngineDesc& sim_engine(const ps::vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* s = std::get_if<SimulatedEngineDesc>(&c.params)) return *s;
    throw std::runtime_error("no simulated engine");
}

// A straight strip along +x: [-200, 20000] x [-20, 20] m in 20 m cells (PHYS-047).
ps::MeshShape strip_mesh(ps::SurfaceId sid) {
    constexpr real kCell = 20.0, x0 = -200.0, x1 = 20000.0, y0 = -20.0, y1 = 20.0;
    const int nx = static_cast<int>((x1 - x0) / kCell), ny = static_cast<int>((y1 - y0) / kCell);
    ps::MeshShape m;
    for (int j = 0; j <= ny; ++j)
        for (int i = 0; i <= nx; ++i) m.vertices.push_back(ps::Vec3{x0 + i * kCell, y0 + j * kCell, 0.0});
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            const auto a = static_cast<std::uint32_t>(j * (nx + 1) + i);
            const std::uint32_t b = a + 1, c = a + static_cast<std::uint32_t>(nx + 1) + 1, dd = a + static_cast<std::uint32_t>(nx + 1);
            m.indices.insert(m.indices.end(), {a, b, c, a, c, dd});
            m.triangle_surface_ids.push_back(sid);
            m.triangle_surface_ids.push_back(sid);
        }
    return m;
}

struct ShiftPeak {
    int from = 0, to = 0;
    real peak_bar = 0.0;
    real max_bov = 0.0;
    // Diagnostics only (never asserted): time of the peak after the shift start, whether the gear had engaged by then, the
    // charge at the shift start, the rotor at the peak and the minimum rotor in the window.
    real t_peak = 0.0, charge_start_bar = 0.0, rotor_peak = 0.0, rotor_min = 1e9, t_engage = -1.0;
    bool peak_after_engage = false;
};

struct Drive {
    std::vector<ShiftPeak> shifts;
    real idle_max_bov = 0.0;  // first 3 s running at pedal 0
    real hold_max_bov = 0.0;  // the speed hold since its last gear change (+ the 1 s settle is inside the window start)
    real hold_window_s = 0.0; // length of that window
    real max_bov = 0.0;
};

// hold_kmh <= 0: WOT (0.6 up to 40 km/h) with auto-shift for `seconds`. hold_kmh > 0: idle 3 s, then WOT to the hold
// speed, then a PI speed hold on the pedal (racing_game's gains as in test_boost_shift_hold).
Drive drive(const ps::vehicle::VehicleDesc& desc, real seconds, real hold_kmh) {
    ps::WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = 4;
    ps::World w(cfg);
    auto surfaces = std::make_shared<ps::io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    ps::BodyDesc ground;
    ground.shape = strip_mesh(surfaces->id_for("asphalt"));
    ground.motion = ps::BodyMotionType::Static;
    ground.friction = 1.0;
    w.create_body(ground);
    ps::BodyDesc chassis;
    chassis.shape = ps::BoxShape{ps::Vec3{2.0, 0.4, 0.15}};
    chassis.motion = ps::BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = ps::Vec3{0.0, 0.0, 0.5};
    const ps::BodyId body = w.create_body(chassis);
    w.set_surface_table(surfaces);
    const ps::VehicleId vid = w.create_vehicle(desc, body);
    w.set_control("ignition", 1.0);
    w.set_control("starter", 1.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    ps_test::settle(w, 2.0);

    Drive res;
    const real dt = w.dt();
    const long steps = static_cast<long>(seconds / dt);
    real pedal = 0.0, integ = 0.0, run_t0 = -1.0, last_change_t = 0.0, window_max = 0.0, until = -1.0, shift_t0 = 0.0;
    bool started = false, holding = false, in_shift = false;
    int last_gear = w.powertrain_state(vid).gear;
    ShiftPeak cur;
    ShiftPhase prev_phase = ShiftPhase::Idle;
    for (long k = 0; k < steps; ++k) {
        const real t = k * dt;
        const real kmh0 = w.get_motion(body).linear.x * 3.6;
        if (!started && (w.powertrain_state(vid).engine_state == EngineState::Running || t > 6.0)) {
            w.set_control("starter", 0.0);
            started = true;
            run_t0 = t;
        }
        if (started) {
            if (hold_kmh <= 0.0) {
                pedal = kmh0 < 40.0 ? 0.6 : 1.0;
            } else if (t < run_t0 + 3.0) {
                pedal = 0.0;
            } else if (holding || kmh0 >= hold_kmh) {
                if (!holding) {
                    holding = true;
                    integ = 0.6;
                }
                const real e = (hold_kmh - kmh0) / 3.6;
                integ = std::clamp(integ + 0.05 * e * dt, 0.0, 1.0);
                pedal = std::clamp(0.08 * e + integ, 0.0, 1.0);
            } else {
                pedal = kmh0 < 40.0 ? 0.6 : 1.0;
            }
            w.set_control("throttle", pedal);
        }
        w.step();
        const auto& pt = w.powertrain_state(vid);
        real charge = 0.0, bov = 0.0, rotor = 0.0;
        if (!pt.engines.empty()) {
            const auto& e = pt.engines[0];
            if (e.p_charge_pa > 0.0) charge = (e.p_charge_pa - kPamb) / 1e5;
            bov = e.bov_opening;
            rotor = e.turbo_rpm;
        }
        res.max_bov = std::max(res.max_bov, bov);
        if (started && hold_kmh > 0.0 && t < run_t0 + 3.0) res.idle_max_bov = std::max(res.idle_max_bov, bov);
        // Shift window: phase leaving Idle .. 0.5 s after an upshift engages.
        if (pt.shift_phase != ShiftPhase::Idle && prev_phase == ShiftPhase::Idle && !in_shift) {
            in_shift = true;
            cur = ShiftPeak{};
            cur.from = last_gear;
            cur.charge_start_bar = charge;
            shift_t0 = t;
            until = -1.0;
        }
        prev_phase = pt.shift_phase;
        if (in_shift) {
            if (charge > cur.peak_bar) {
                cur.peak_bar = charge;
                cur.t_peak = t - shift_t0;
                cur.rotor_peak = rotor;
                cur.peak_after_engage = until >= 0.0;
            }
            cur.rotor_min = std::min(cur.rotor_min, rotor);
            cur.max_bov = std::max(cur.max_bov, bov);
        }
        if (pt.gear > 0 && pt.gear != last_gear) {
            if (last_gear > 0) {
                last_change_t = t;
                window_max = 0.0;
                if (in_shift && pt.gear > last_gear) {
                    cur.to = pt.gear;
                    cur.t_engage = t - shift_t0;
                    until = t + 0.5;
                }
            }
            last_gear = pt.gear;
        }
        if (in_shift && until >= 0.0 && t > until) {
            res.shifts.push_back(cur);
            in_shift = false;
        }
        // The steady window starts 1 s after the last gear change (the shift's own transient is H1's business).
        if (holding && t - last_change_t >= 1.0) {
            res.hold_window_s = t - last_change_t - 1.0;
            window_max = std::max(window_max, bov);
        }
        if (w.get_pose(body).position.x > 19500.0) break;
    }
    res.hold_max_bov = window_max;
    return res;
}

struct DynoPoint {
    real tau_mean = 0.0, max_bov = 0.0;
};

DynoPoint held_wot(const SimulatedEngineDesc& d, real rpm) {
    SimulatedEngine e(d);
    constexpr real kH = 1.0 / 960.0;
    const real omega = rpm * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    DynoPoint out;
    constexpr int kN = 6 * 960, kWin = 480;
    for (int i = 0; i < kN; ++i) {
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        out.max_bov = std::max(out.max_bov, r.bov_opening);
        if (i >= kN - kWin) out.tau_mean += (r.tau0 - e.maps().c_k) / kWin;
    }
    return out;
}

} // namespace

TEST_CASE("H1: the shipped blow-off valve caps the in-shift charge spike on car_hyper_n2o", "[turbo][drivetrain][vehicle][long]") {
    const Drive r = drive(load_car("car_hyper_n2o.json"), 40.0, 0.0);
    int opened = 0;
    bool seen[8] = {};
    for (const ShiftPeak& s : r.shifts) {
        std::printf("H1 %d->%d in-shift peak %.3f bar gauge, max bov %.3f\n", s.from, s.to, s.peak_bar, s.max_bov);
        if (s.from < 1 || s.from > 6 || s.to != s.from + 1) continue;
        seen[s.from] = true;
        INFO(s.from << "->" << s.to);
        CHECK(s.peak_bar <= 3.0);
        if (s.max_bov >= 0.5) ++opened;
    }
    for (int g = 1; g <= 6; ++g) {
        INFO("upshift from " << g);
        CHECK(seen[g]);
    }
    CHECK(opened >= 4);
}

TEST_CASE("H2: the valve stays shut at idle and at steady cruise", "[turbo][drivetrain][vehicle][long]") {
    const auto desc = load_car("car_hyper_n2o.json");
    for (real kmh : {60.0, 120.0}) {
        const Drive r = drive(desc, 40.0, kmh);
        std::printf("H2 hold %.0f km/h: idle max bov %.5f, steady window %.1f s, hold max bov %.5f, whole-run max %.3f\n", kmh,
                    r.idle_max_bov, r.hold_window_s, r.hold_max_bov, r.max_bov);
        INFO("hold " << kmh << " km/h");
        REQUIRE(r.hold_window_s >= 8.0); // precondition: a steady window without a gear change
        CHECK(r.idle_max_bov <= 0.001);
        CHECK(r.hold_max_bov <= 0.001);
    }
}

TEST_CASE("H3: the valve does not touch the WOT torque", "[turbo][drivetrain][long]") {
    for (const char* car : {"car_hyper.json", "car_hyper_n2o.json"}) {
        const auto desc = load_car(car);
        const SimulatedEngineDesc& with = sim_engine(desc);
        REQUIRE(with.turbo_pair.has_value());
        REQUIRE(with.turbo_pair->blow_off_valve.has_value()); // precondition: the shipped configuration carries the valve
        SimulatedEngineDesc without = with;
        without.turbo_pair->blow_off_valve.reset();
        for (real rpm : {3000.0, 5000.0, 7000.0}) {
            const DynoPoint a = held_wot(with, rpm);
            const DynoPoint b = held_wot(without, rpm);
            std::printf("H3 %s %4.0f rpm: tau with %.2f without %.2f (%+.4f %%), max bov %.4f\n", car, rpm, a.tau_mean, b.tau_mean,
                        100.0 * (a.tau_mean / b.tau_mean - 1.0), a.max_bov);
            INFO(car << " " << rpm << " rpm");
            REQUIRE(b.tau_mean > 100.0);
            CHECK(std::abs(a.tau_mean / b.tau_mean - 1.0) <= 0.005);
            CHECK(a.max_bov <= 0.01);
        }
    }
}

// H4: the lift-off with and without the shipped valve. Shipped car_hyper engine, crank held at 6000 rpm, WOT 2 s, plate shut.
// The valve keeps the compressor flowing (~0.5-0.9 kg/s at PR 1.4-2.5), so the rotor spins down FASTER with it - the physics
// of a recirculating valve on this speed line, checked against an independent flow balance in H5 (round 3, 2026-10-14). The
// valve-less control dead-heads the compressor (f ~ 0.01, the surge side the lumped model does not cycle) and keeps the rotor.
// Bands from the first measurement (release, 2026-10-13, [M], round-1 valve 920 mm2): with the valve rotor 36252 rpm at
// +0.5 s and 23539 rpm at +1.0 s, valve opening 0.998 at +0.25 s; without it 91216 / 89314 rpm. Margins: rotor +-15 % around
// the measured value, "without" at least 85000 / 80000 rpm. Round 3 (1330 mm2, tau 20 ms): 38944 / 22349 rpm - inside the
// round-1 bands, not re-pinned (H5 is the physics guard; H4 keeps the order of magnitude and the control).
TEST_CASE("H4: lift-off with the shipped valve - the spin-down it causes is pinned", "[turbo][drivetrain]") {
    const auto desc = load_car("car_hyper.json");
    struct Lift { real rotor_05 = 0, rotor_10 = 0, bov_025 = 0, bov_max = 0; };
    const auto run = [&](bool valve) {
        SimulatedEngineDesc d = sim_engine(desc);
        if (!valve) d.turbo_pair->blow_off_valve.reset();
        SimulatedEngine e(d);
        constexpr real kH = 1.0 / 960.0;
        const real omega = 6000.0 * 2.0 * kPi / 60.0;
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        Lift out;
        for (int i = 0; i < 4 * 960; ++i) {
            const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < 2 * 960 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
            const int j = i - 2 * 960;
            out.bov_max = std::max(out.bov_max, r.bov_opening);
            if (j == 240) out.bov_025 = r.bov_opening;
            if (j == 480) out.rotor_05 = r.turbo_rpm;
            if (j == 960) out.rotor_10 = r.turbo_rpm;
        }
        return out;
    };
    const Lift with = run(true), without = run(false);
    std::printf("H4 with valve: rotor %.0f (0.5 s) %.0f (1.0 s), bov %.3f at 0.25 s | without: rotor %.0f / %.0f, bov max %.3f\n", with.rotor_05,
                with.rotor_10, with.bov_025, without.rotor_05, without.rotor_10, without.bov_max);
    CHECK(without.bov_max == 0.0); // the control run really has no valve
    CHECK(without.rotor_05 >= 85000.0);
    CHECK(without.rotor_10 >= 80000.0);
    CHECK(with.bov_025 >= 0.9);
    CHECK(with.rotor_05 >= 30800.0);
    CHECK(with.rotor_05 <= 41700.0);
    CHECK(with.rotor_10 >= 20000.0);
    CHECK(with.rotor_10 <= 27100.0);
}

// H5 (BOV round 3, 2026-10-14): the lift-off spin-down with the shipped valve follows an INDEPENDENT flow balance. The
// reference below is written here, not taken from turbo_pair.h: rotor energy E, valve opening x and a quasi-steady charge
// pipe, i.e. the compressor's delivered flow on the hardware's speed line equals the engine's throttle-path flow plus the
// valve's orifice flow,
//     m_max * sqrt(1 - (p - p_amb) / (H_ref s^2)) = m_e + x * A * p * Psi(p_amb / p) / sqrt(R T_ch),   s = N / N_ref,
// solved by bisection every substep (Psi analytic, gamma 1.4, not the engine's table), the compressor power at THAT
// operating point with the same law as with the valve shut, P_c = (m_e + m_v) cp T_amb (PR^(2/7) - 1) / eta, and
// dE/dt = P_t - P_c - loss_ref s^2, the valve following clamp((p - p_im - crack)/(full - crack)) with its lag.
// Boundary inputs per substep from the simulation (not the compressor/valve/rotor physics under test): the engine's
// throttle-path flow m_e, the turbine power P_t, the manifold pressure p_im and the charge temperature T_ch.
// Bands (committed in out/hyper_bov_r3_progress.md before the first run, [D] tolerance for the simulation's 25 ms charge
// relaxation and its one-substep explicit vent hand-off against the quasi-steady reference): rotor within 10 % of the
// reference at +0.25 / +0.5 / +1.0 s and charge within 0.15 bar of it at the same instants; the valve is open (>= 0.9)
// at +0.25 s. The derived retention the reference gives is printed (the [D] statement in turbo.md).
struct LiftRef {
    real rotor_sim[3] = {}, rotor_ref[3] = {}, charge_sim_bar[3] = {}, charge_ref_bar[3] = {};
    real rotor_lift = 0.0, bov_025 = 0.0;
};

LiftRef lift_vs_reference(const SimulatedEngineDesc& d) {
    SimulatedEngine e(d);
    const TurboPairDesc& td = *d.turbo_pair;
    const BlowOffValveDesc& v = *td.blow_off_valve;
    constexpr real kH = 1.0 / 960.0, kR = 287.05, kCp = 1005.0, kGam = 1.4;
    const real omega = 6000.0 * 2.0 * kPi / 60.0;
    const real wref = td.reference_rpm * 2.0 * kPi / 60.0;
    const auto psi = [](real q) {
        if (q >= 1.0) return 0.0;
        const real qc = std::pow(2.0 / (kGam + 1.0), kGam / (kGam - 1.0));
        q = std::max(q, qc);
        return std::sqrt(2.0 * kGam / (kGam - 1.0) * (std::pow(q, 2.0 / kGam) - std::pow(q, (kGam + 1.0) / kGam)));
    };
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    LiftRef out;
    real E = 0.0, x = 0.0, p_ref = 0.0;
    for (int i = 0; i < 3 * 960; ++i) {
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{i < 2 * 960 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        const TurboPair& tp = *e.turbo();
        const int j = i - 2 * 960;
        if (j < 0) continue;
        if (j == 0) { // hand-over at the lift: the reference starts from the simulation's rotor and a shut valve
            E = tp.energy_j;
            x = tp.bov_opening;
            out.rotor_lift = tp.rpm();
        }
        const real pa = 101325.0, ta = 288.15;
        const real m_e = std::max(0.0, tp.compressor_flow_kg_s - tp.bov_flow_kg_s);
        const real t_ch = tp.charge_k;
        const real w = std::sqrt(2.0 * E / td.rotor_inertia_kgm2);
        const real sp = w / wref;
        const real hs = td.reference_head_pa * sp * sp;
        const auto m_c = [&](real p) { return td.max_mass_flow_kg_s * std::sqrt(std::max(0.0, 1.0 - (p - pa) / hs)); };
        const auto m_v = [&](real p) { return x * v.effective_area_m2 * p * psi(pa / p) / std::sqrt(kR * t_ch); };
        real lo = pa, hi = pa + hs;
        for (int it = 0; it < 80; ++it) {
            const real mid = 0.5 * (lo + hi);
            if (m_c(mid) - m_e - m_v(mid) > 0.0) lo = mid; else hi = mid;
        }
        p_ref = 0.5 * (lo + hi);
        const real pr = std::max(1.0, p_ref / pa);
        const real pc = (m_e + m_v(p_ref)) * kCp * ta * (std::pow(pr, 2.0 / 7.0) - 1.0) / td.compressor_efficiency;
        E = std::max(0.0, E + kH * (tp.turbine_w - pc - td.shaft_loss_w_at_reference * sp * sp));
        const real cmd = std::clamp((p_ref - r.p_intake_pa - v.crack_dp_pa) / (v.full_open_dp_pa - v.crack_dp_pa), 0.0, 1.0);
        x += kH / (v.tau_s + kH) * (cmd - x);
        if (j == 240) out.bov_025 = tp.bov_opening;
        const int slot = j == 240 ? 0 : j == 480 ? 1 : j == 960 - 1 ? 2 : -1;
        if (slot >= 0) {
            out.rotor_sim[slot] = tp.rpm();
            out.rotor_ref[slot] = std::sqrt(2.0 * E / td.rotor_inertia_kgm2) * 60.0 / (2.0 * kPi);
            out.charge_sim_bar[slot] = (tp.charge_pa - pa) / 1e5;
            out.charge_ref_bar[slot] = (p_ref - pa) / 1e5;
        }
    }
    return out;
}

TEST_CASE("H5: the lift-off spin-down with the shipped valve follows an independent compressor/valve flow balance", "[turbo][drivetrain]") {
    const auto desc = load_car("car_hyper.json");
    const SimulatedEngineDesc& d = sim_engine(desc);
    REQUIRE(d.turbo_pair.has_value());
    REQUIRE(d.turbo_pair->blow_off_valve.has_value());
    REQUIRE((!d.turbo_pair->maps || !d.turbo_pair->maps->stages));// precondition: the authored speed line the reference uses
    const LiftRef r = lift_vs_reference(d);
    const char* at[3] = {"+0.25 s", "+0.5 s", "+1.0 s"};
    for (int k = 0; k < 3; ++k) {
        std::printf("H5 %s: rotor sim %.0f ref %.0f rpm (%+.2f %%, retention sim %.1f %% ref %.1f %% of %.0f), charge sim %.3f ref %.3f bar g\n",
                    at[k], r.rotor_sim[k], r.rotor_ref[k], 100.0 * (r.rotor_sim[k] / r.rotor_ref[k] - 1.0),
                    100.0 * r.rotor_sim[k] / r.rotor_lift, 100.0 * r.rotor_ref[k] / r.rotor_lift, r.rotor_lift, r.charge_sim_bar[k],
                    r.charge_ref_bar[k]);
        INFO(at[k]);
        CHECK(std::abs(r.rotor_sim[k] / r.rotor_ref[k] - 1.0) <= 0.10);
        CHECK(std::abs(r.charge_sim_bar[k] - r.charge_ref_bar[k]) <= 0.15);
    }
    CHECK(r.bov_025 >= 0.9);
}

// Hidden diagnostic: H1's drive with the valve of car_hyper_n2o overridden from the environment (PS_BOV_NONE=1 removes it;
// PS_BOV_AREA_MM2 / PS_BOV_CRACK_BAR / PS_BOV_FULL_BAR / PS_BOV_TAU_S replace a field). Prints each upshift's peak, when it
// came (after the shift start; whether the new gear had engaged), the charge at the shift start and the rotor.
TEST_CASE("BOV H1 diag", "[.][diag][hyper_bov]") {
    auto desc = load_car("car_hyper_n2o.json");
    for (auto& c : desc.powertrain.components)
        if (auto* s = std::get_if<SimulatedEngineDesc>(&c.params)) {
            auto& v = *s->turbo_pair->blow_off_valve;
            v.effective_area_m2 = env_real("PS_BOV_AREA_MM2", v.effective_area_m2 * 1e6 / 2.0) * 1e-6 * 2.0;
            v.crack_dp_pa = env_real("PS_BOV_CRACK_BAR", v.crack_dp_pa / 1e5) * 1e5;
            v.full_open_dp_pa = env_real("PS_BOV_FULL_BAR", v.full_open_dp_pa / 1e5) * 1e5;
            v.tau_s = env_real("PS_BOV_TAU_S", v.tau_s);
            std::printf("valve pair area %.1f mm2, crack %.2f, full %.2f bar, tau %.3f s\n", v.effective_area_m2 * 1e6,
                        v.crack_dp_pa / 1e5, v.full_open_dp_pa / 1e5, v.tau_s);
            if (env_real("PS_BOV_NONE", 0.0) > 0.5) {
                s->turbo_pair->blow_off_valve.reset();
                std::printf("valve REMOVED\n");
            }
        }
    const Drive r = drive(desc, 40.0, 0.0);
    for (const ShiftPeak& s : r.shifts)
        std::printf("diag %d->%d peak %.3f bar at +%.3f s (engaged at +%.3f s, peak %s engagement), charge at start %.3f, "
                    "rotor at peak %.0f min %.0f, max bov %.3f\n",
                    s.from, s.to, s.peak_bar, s.t_peak, s.t_engage, s.peak_after_engage ? "after" : "before", s.charge_start_bar,
                    s.rotor_peak, s.rotor_min, s.max_bov);
}

// Hidden diagnostic (round 3 measurement): the lift-off of the shipped hyper V8 with and without the valve, crank held at
// PS_BOV_RPM (default 6000) rpm, WOT 2 s then plate shut. Optional PS_BOV_AREA_MM2 replaces the per-turbo valve area,
// PS_BOV_CRACK_BAR / PS_BOV_FULL_BAR the crack / full-open dp. Per sample: rotor, speed fraction s = N/N_ref, charge
// (post-cooler, Pa abs), compressor PR, total compressor flow and its fraction f of max, engine (throttle-path) flow, vent
// flow, valve opening, the head H(s, f) the authored speed line gives at that flow, compressor/turbine power, p_im, plate,
// rotor deceleration. Efficiency is the hardware's constant 0.8 (authored map, no stage map).
TEST_CASE("BOV lift diag", "[.][diag][hyper_bov]") {
    const auto desc = load_car("car_hyper.json");
    const real crank_rpm = env_real("PS_BOV_RPM", 6000.0);
    for (int variant = 0; variant < 2; ++variant) {
        SimulatedEngineDesc d = sim_engine(desc);
        if (variant == 1) d.turbo_pair->blow_off_valve.reset();
        if (variant == 0) {
            auto& v = *d.turbo_pair->blow_off_valve;
            v.effective_area_m2 = env_real("PS_BOV_AREA_MM2", v.effective_area_m2 * 1e6 / 2.0) * 1e-6 * 2.0;
            v.crack_dp_pa = env_real("PS_BOV_CRACK_BAR", v.crack_dp_pa / 1e5) * 1e5;
            v.full_open_dp_pa = env_real("PS_BOV_FULL_BAR", v.full_open_dp_pa / 1e5) * 1e5;
            std::printf("valve pair area %.1f mm2, crack %.2f bar, full %.2f bar, tau %.3f s\n", v.effective_area_m2 * 1e6,
                        v.crack_dp_pa / 1e5, v.full_open_dp_pa / 1e5, v.tau_s);
        }
        SimulatedEngine e(d);
        constexpr real kH = 1.0 / 960.0;
        const real omega = crank_rpm * 2.0 * kPi / 60.0;
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        std::printf("BOV diag %s, crank %.0f rpm\n", variant == 0 ? "with valve" : "no valve", crank_rpm);
        std::printf("   t_ms   rotor     s   charge     PR  comp_kg_s     f  eng_kg_s vent_kg_s   bov head_bar  Pc_kW  Pt_kW    p_im  plate exh_kg_s dN/dt_rpm_s\n");
        real rpm_prev = 0.0;
        for (int i = 0; i < 4 * 960; ++i) {
            const real thr = i < 2 * 960 ? 1.0 : 0.0;
            const PreSolveResult r = e.pre_solve(s, ThrottleCommand{thr, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
            const TurboPair* tp = e.turbo();
            const real rpm_now = tp->rpm();
            const real dndt = (rpm_now - rpm_prev) / kH;
            rpm_prev = rpm_now;
            const int j = i - 2 * 960;
            static const int marks[] = {-1, 0, 5, 10, 20, 30, 48, 72, 96, 144, 192, 240, 336, 480, 720, 960, 1440, 1919};
            bool print = false;
            for (int m : marks) print = print || (j == m);
            if (!print) continue;
            const real sp = rpm_now / tp->desc.reference_rpm;
            const real f = tp->compressor_flow_kg_s / tp->desc.max_mass_flow_kg_s;
            const real head = tp->desc.maps ? tp->desc.maps->head_pa.value(std::min(sp, tp->desc.maps->head_pa.x.back()), f) : 0.0;
            std::printf("  %5.0f %7.0f %5.3f %8.0f %6.3f %10.4f %5.3f %9.4f %9.4f %5.3f %8.3f %6.1f %6.1f %7.0f %6.3f %8.4f %11.0f\n",
                        j * 1000.0 / 960.0, rpm_now, sp, tp->charge_pa, tp->compressor_pr, tp->compressor_flow_kg_s, f,
                        tp->compressor_flow_kg_s - tp->bov_flow_kg_s, tp->bov_flow_kg_s, tp->bov_opening, head / 1e5,
                        tp->compressor_w / 1e3, tp->turbine_w / 1e3, r.p_intake_pa, r.throttle_position, r.exhaust_mass_flow_kg_s, dndt);
        }
    }
}
