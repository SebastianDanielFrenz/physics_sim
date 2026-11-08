// Hybrid crank runaway (owner drive 2026-11-06, racing_game release 20261104-1825 = physics_sim 603fc5f): a garage
// hypercar (quad-turbo kit at 5 bar, DCT, 50 kW crank generator on the crank shaft, hybrid_control) lifted at 357 km/h,
// the DCT went to neutral and the engine ran down to 0 rpm; with the pedal held at 1 the crank then spun up to ~60000 rpm
// and the boost reached tens of bar while the engine reported Off. Evidence: bug_logs/2026-11-06_racing_game_engine_failure.
// Fixture: tests/support/fixtures/hybrid_runaway/ (the owner's files, paths rewritten, tyre_spec -> shipped hyper tyres).
// Diagnostic trace (hidden, prints): ./ps_unit_tests "[hybrid_runaway][diag]"
// Status (2026-11-08): the car-level owner timeline (1 Hz pedal samples, also with 0.15/0.3/0.6 s pedal tapping) does NOT
// reproduce the runaway - the engine restarts and reconnects. Found and fixed: a TurboPair energy leak past choke (see the
// regression tests at the end) that turns a plate toggling on a crank held at speed into unbounded rotor/charge/crank energy
// with no fuel - the signature of the owner's log (tens of bar, crank still accelerating 29775 -> 60558 rpm after the pedal
// was released, no fuel above the 8100 rpm fuel cut). What held the owner's crank near speed with the DCT clutch open is
// not established.
#include "engine_map_test_cache.h"
#include "vehicle_test_helpers.h"

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"
#include "ps/drivetrain/engine_rig.h"
#include "ps/drivetrain/limiter_range.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace ps;

namespace {

const char* kFixture = "/tests/support/fixtures/hybrid_runaway/car_hyper_n2o_hybrid_quad.json";

vehicle::VehicleDesc load_runaway_car() {
    io::VehicleLoadOptions o;
    o.maps = ps_test::shared_map_options(drivetrain::MapQuality::Full);
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + kFixture, o);
}

struct RunawayResult {
    double max_rpm = 0.0;
    double max_rpm_off = 0.0;  // highest crank rpm while the engine reports Off or Stalled
    double max_boost_bar = 0.0;
    double max_turbo_rpm = 0.0;
    double max_rel_balance = 0.0;
    double t_max_rpm = 0.0;
};

// The owner's sequence: 230 km/h in neutral (the DCT's hybrid disconnect), engine Off at rest, drive mode Normal, then the
// pedal held at 1 for `hold_s` and released for `release_s`.
struct Segment {
    double duration_s, throttle, brake;
};

RunawayResult run(const std::vector<Segment>& pedal, double start_kmh, bool verbose) {
    vehicle::VehicleDesc desc = load_runaway_car();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, 1, Vec3::unit_y(), 240.0, 960.0,
                                                    6000.0);
    World& w = *rig.world;
    w.set_control("assist.drive_mode", 1.0);
    w.set_control("electric.ready", 1.0);
    w.set_control("gear_request", 99.0);
    w.set_control("throttle", 0.0);
    ps_test::settle(w, 0.5);
    Motion m;
    m.linear = Vec3{start_kmh / 3.6, 0, 0};
    w.backend().set_motion(rig.chassis, m);
    vehicle::VehicleResetOptions ro;
    ro.gear = 0;
    ro.engine_state = drivetrain::EngineState::Off;
    ro.hubs_roll_with_chassis = true;
    w.reset_vehicle(rig.vehicle, ro);
    ps_test::settle(w, 0.5);

    RunawayResult r;
    const int hz = 240;
    double total_s = 0.0;
    for (const auto& seg : pedal) total_s += seg.duration_s;
    const int total = static_cast<int>(total_s * hz);
    drivetrain::PowertrainEnergy e0 = w.powertrain_energy(rig.vehicle);
    for (int i = 0; i < total; ++i) {
        const double t = i / double(hz);
        double pedal_now = 0.0, brake_now = 0.0, edge = 0.0;
        for (const auto& seg : pedal) {
            edge += seg.duration_s;
            if (t < edge) {
                pedal_now = seg.throttle;
                brake_now = seg.brake;
                break;
            }
        }
        w.set_control("throttle", pedal_now);
        w.set_control("brake", brake_now);
        const drivetrain::PowertrainEnergy prev = w.powertrain_energy(rig.vehicle);
        w.step();
        const auto& st = w.powertrain_state(rig.vehicle);
        const drivetrain::PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        const double rpm = st.engine_rpm;
        const bool unfired_state = st.engine_state == drivetrain::EngineState::Off || st.engine_state == drivetrain::EngineState::Stalled;
        if (rpm > r.max_rpm) {
            r.max_rpm = rpm;
            r.t_max_rpm = t;
        }
        if (unfired_state) r.max_rpm_off = std::max(r.max_rpm_off, rpm);
        const drivetrain::EngineSoundState* es = st.engines.empty() ? nullptr : &st.engines[0];
        if (es) {
            r.max_boost_bar = std::max(r.max_boost_bar, double(es->boost_bar));
            r.max_turbo_rpm = std::max(r.max_turbo_rpm, double(es->turbo_rpm));
        }
        const double scale = std::max({std::abs(e.engine_out - prev.engine_out), std::abs(e.electric_out - prev.electric_out),
                                       std::abs(e.rotor_ke - prev.rotor_ke), 1.0});
        r.max_rel_balance = std::max(r.max_rel_balance, std::abs(double(e.balance_error)) / scale);
        if (verbose && (i % 24 == 0) && es) {
            const double dt_tick = 1.0 / hz;
            std::printf("t=%6.2f v=%5.1f g=%d st=%d rpm=%7.0f clutch=%.2f ice_req=%d lim=%d starter=%d fuel=%6.1f plate=%.2f p_im=%6.1f "
                        "p_ch=%6.1f p_exh=%6.1f boost=%6.2f trpm=%7.0f wg=%.2f T_ch=%5.0f T_port=%5.0f mdot_ex=%.3f "
                        "P_eng=%8.1f kW P_pump=%8.1f P_fric=%7.1f P_ind=%7.1f P_elec=%7.1f P_start=%6.1f motor_rpm=%6.0f bal=%.2e\n",
                        t, w.backend().get_motion(rig.chassis).linear.length() * 3.6, st.gear, int(st.engine_state), rpm,
                        double(st.clutch_engagement), st.hybrid_ice_requested ? 1 : 0, es->limiter_active ? 1 : 0, es->starter_active ? 1 : 0,
                        double(es->fuel_flow_g_s), double(es->throttle_position), es->p_intake_pa / 1000.0, es->p_charge_pa / 1000.0,
                        es->p_exhaust_pa / 1000.0, double(es->boost_bar), double(es->turbo_rpm), double(es->wastegate_opening),
                        double(es->t_charge_k), double(es->exhaust_port_k), double(es->exhaust_mass_flow_kg_s),
                        (e.engine_out - prev.engine_out) / dt_tick / 1000.0, (e.pumping_work - prev.pumping_work) / dt_tick / 1000.0,
                        (e.friction_work - prev.friction_work) / dt_tick / 1000.0,
                        (e.indicated_work - prev.indicated_work) / dt_tick / 1000.0,
                        (e.electric_out - prev.electric_out) / dt_tick / 1000.0, (e.starter_in - prev.starter_in) / dt_tick / 1000.0,
                        double(st.motor_rpm), double(e.balance_error));
        }
    }
    (void)e0;
    return r;
}

} // namespace

TEST_CASE("hybrid crank runaway trace", "[.][diag][hybrid_runaway]") {
    const RunawayResult r = run({{13.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {1.0, 0.04, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.03, 0.0},
                                {3.0, 0.12, 0.0}, {2.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {14.0, 1.0, 0.0}, {10.0, 0.0, 0.0}},
                               0.0, true);
    std::printf("max rpm %.0f at %.2f s (unfired max %.0f), max boost %.2f bar, max turbo %.0f rpm, max rel balance %.2e\n", r.max_rpm,
                r.t_max_rpm, r.max_rpm_off, r.max_boost_bar, r.max_turbo_rpm, r.max_rel_balance);
}

namespace {
struct RigSeg {
    double duration_s, throttle, ignition, starter;
};
void run_engine_rig(const std::vector<RigSeg>& segs, double limiter_rpm = 0.0, double band = -1.0) {
    vehicle::VehicleDesc desc = load_runaway_car();
    drivetrain::ComponentParams* eng = nullptr;
    for (auto& c : desc.powertrain.components)
        if (std::holds_alternative<drivetrain::SimulatedEngineDesc>(c.params)) eng = &c.params;
    REQUIRE(eng != nullptr);
    if (limiter_rpm > 0.0) std::get<drivetrain::SimulatedEngineDesc>(*eng).limiter.rpm = limiter_rpm;
    if (band >= 0.0) std::get<drivetrain::SimulatedEngineDesc>(*eng).rpm_rolloff_band_fraction = band;
    {
        const auto& sd = std::get<drivetrain::SimulatedEngineDesc>(*eng);
        const drivetrain::RpmRolloff ro = drivetrain::rpm_rolloff(sd);
        std::printf("limiter %.0f (type %d), rolloff armed %d start %.0f end %.0f, effective limiter %.0f\n", sd.limiter.rpm,
                    int(sd.limiter.type), ro.armed() ? 1 : 0, ro.start_rpm, ro.end_rpm, drivetrain::effective_limiter_rpm(sd));
    }
    EngineRig rig(*eng, nullptr);
    const int hz = 240;
    int i = 0;
    for (const auto& s : segs) {
        const int n = static_cast<int>(s.duration_s * hz);
        for (int k = 0; k < n; ++k, ++i) {
            rig.set_throttle(s.throttle);
            rig.set_ignition(s.ignition);
            rig.set_starter(s.starter);
            const drivetrain::PowertrainEnergy prev = rig.ledger();
            rig.step();
            const drivetrain::PowertrainEnergy& e = rig.ledger();
            const auto& es = rig.engine_sound_state();
            if (i % 24 == 0) {
                const double dt = 1.0 / hz;
                std::printf("t=%6.2f st=%d rpm=%7.0f thr=%.2f lim=%d fuel=%6.1f plate=%.2f p_im=%7.1f p_ch=%7.1f p_exh=%7.1f trpm=%7.0f "
                            "wg=%.2f T_ch=%5.0f T_port=%5.0f mdot=%.3f P_eng=%8.1f P_pump=%8.1f P_fric=%7.1f P_ind=%7.1f bal=%.2e\n",
                            rig.sim_time_s(), int(rig.engine_state()), rig.engine_rpm(), s.throttle, es.limiter_active ? 1 : 0,
                            double(es.fuel_flow_g_s), double(es.throttle_position), es.p_intake_pa / 1000.0, es.p_charge_pa / 1000.0,
                            es.p_exhaust_pa / 1000.0, double(es.turbo_rpm), double(es.wastegate_opening), double(es.t_charge_k),
                            double(es.exhaust_port_k), double(es.exhaust_mass_flow_kg_s), (e.engine_out - prev.engine_out) / dt / 1000.0,
                            (e.pumping_work - prev.pumping_work) / dt / 1000.0, (e.friction_work - prev.friction_work) / dt / 1000.0,
                            (e.indicated_work - prev.indicated_work) / dt / 1000.0, double(e.balance_error));
            }
        }
    }
}
} // namespace

TEST_CASE("hybrid crank runaway engine rig", "[.][diag][hybrid_runaway_rig]") {
    // start, WOT free rev into the limiter, then key off at WOT, then key off closed.
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {8.0, 1.0, 1.0, 0.0}, {10.0, 1.0, 0.0, 0.0}, {5.0, 0.0, 0.0, 0.0}});
}

TEST_CASE("hybrid crank runaway engine rig high limiter", "[.][diag][hybrid_runaway_rig2]") {
    // The limiter raised far past the map grid (8250 rpm): does the engine above the grid make power from nothing?
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {4.0, 1.0, 1.0, 0.0}, {6.0, 1.0, 0.0, 0.0}, {4.0, 0.0, 0.0, 0.0}},
                   60000.0);
}

TEST_CASE("hybrid crank runaway engine rig high limiter no rolloff", "[.][diag][hybrid_runaway_rig3]") {
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {4.0, 1.0, 1.0, 0.0}, {6.0, 1.0, 0.0, 0.0}, {4.0, 0.0, 0.0, 0.0}},
                   60000.0, 0.0);
}

TEST_CASE("hybrid crank runaway engine rig lift then reopen key off", "[.][diag][hybrid_runaway_rig4]") {
    // WOT into the limiter, a lift (charge left at the rotor's zero-flow head), then the plate reopened with the key off.
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {8.0, 1.0, 1.0, 0.0}, {0.6, 0.0, 1.0, 0.0}, {6.0, 1.0, 0.0, 0.0},
                    {4.0, 0.0, 0.0, 0.0}});
}

TEST_CASE("hybrid crank runaway engine rig restart on a spinning turbo", "[.][diag][hybrid_runaway_rig6]") {
    // WOT into the limiter, lift with the key off until the crank stops (the rotors keep spinning), then the owner's
    // restart: crank at pedal 0.12, then WOT.
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {8.0, 1.0, 1.0, 0.0}, {5.0, 0.0, 0.0, 0.0}, {1.5, 0.12, 1.0, 1.0},
                    {12.0, 1.0, 1.0, 0.0}, {4.0, 0.0, 1.0, 0.0}});
}

TEST_CASE("hybrid crank runaway engine rig unfired air motor", "[.][diag][hybrid_runaway_rig7]") {
    // As rig6, but the key stays off while the plate opens: the crank can only be turned by the charge.
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {8.0, 1.0, 1.0, 0.0}, {5.0, 0.0, 0.0, 0.0}, {1.5, 0.12, 0.0, 0.0},
                    {12.0, 1.0, 0.0, 0.0}, {4.0, 0.0, 0.0, 0.0}});
}

TEST_CASE("hybrid crank runaway engine rig lift then reopen key on", "[.][diag][hybrid_runaway_rig5]") {
    run_engine_rig({{1.0, 0.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 0.0}, {8.0, 1.0, 1.0, 0.0}, {0.6, 0.0, 1.0, 0.0}, {6.0, 1.0, 1.0, 0.0},
                    {4.0, 0.0, 0.0, 0.0}});
}

namespace {
// Held crank, key off: an unfired engine with its turbos at a held crank speed. After a fired WOT pre-spin the key goes
// off with the plate held open. The crank's net torque must not be positive at a steady state - nothing but the crank
// itself feeds the gas path, so positive torque would be work out of nothing.
struct HeldResult {
    double mean_net_nm = 0.0, max_charge_bar = 0.0, max_rotor_rpm = 0.0;
};
HeldResult held_keyoff(double pre_rpm, double rpm, double throttle, double hold_s, bool verbose, double toggle_s = 0.0) {
    vehicle::VehicleDesc desc = load_runaway_car();
    const drivetrain::SimulatedEngineDesc* sd = nullptr;
    for (auto& c : desc.powertrain.components)
        if (std::holds_alternative<drivetrain::SimulatedEngineDesc>(c.params)) sd = &std::get<drivetrain::SimulatedEngineDesc>(c.params);
    REQUIRE(sd != nullptr);
    drivetrain::SimulatedEngine e(*sd);
    e.reset(drivetrain::EngineState::Running);
    const double h = 1.0 / 960.0;
    drivetrain::Sensors s;
    s.ignition = 1.0;
    s.omega = pre_rpm * 2.0 * 3.14159265358979 / 60.0;
    for (int i = 0; i < 960 * 5; ++i) {
        (void)e.pre_solve(s, drivetrain::ThrottleCommand{1.0, 0.0, 1.0}, drivetrain::TcuRequest{}, h);
        (void)e.post_solve(s.omega, h);
    }
    s.ignition = 0.0;
    s.omega = rpm * 2.0 * 3.14159265358979 / 60.0;
    const double c_k = e.maps().c_k;
    double sum = 0.0;
    int n = 0;
    HeldResult out;
    const int steps = static_cast<int>(hold_s * 960.0);
    const int last = toggle_s > 0.0 ? steps / 2 : 960; // with toggling: the mean over the second half (whole periods)
    for (int i = 0; i < steps; ++i) {
        const double thr = toggle_s > 0.0 ? ((static_cast<int>(i * h / toggle_s) % 2) == 0 ? throttle : 0.0) : throttle;
        const drivetrain::PreSolveResult r = e.pre_solve(s, drivetrain::ThrottleCommand{thr, 0.0, 1.0}, drivetrain::TcuRequest{}, h);
        (void)e.post_solve(s.omega, h);
        const double net = r.tau0 - c_k;
        if (i >= steps - last) { sum += net; ++n; }
        out.max_charge_bar = std::max(out.max_charge_bar, (e.turbo()->charge_pa - 101325.0) / 1e5);
        out.max_rotor_rpm = std::max(out.max_rotor_rpm, double(e.turbo()->rpm()));
        if (verbose && i % 24 == 0) {
            const auto* t = e.turbo();
            std::printf("  t=%5.3f thr=%.0f net=%9.1f p_im=%9.1f p_ch=%9.1f p_exh=%9.1f trpm=%9.0f wg=%.2f comp_kW=%9.1f turb_kW=%9.1f "
                        "flow=%.3f exf=%.3f T_ch=%.0f T_ex=%.0f pr=%.2f\n",
                        i * h, thr, net, e.intake_pressure_pa() / 1000.0, t->charge_pa / 1000.0, t->exhaust_pa / 1000.0, t->rpm(),
                        t->wastegate, t->compressor_w / 1000.0, t->turbine_w / 1000.0, t->compressor_flow_kg_s,
                        double(r.exhaust_mass_flow_kg_s), t->charge_k, t->exhaust_k, t->compressor_pr);
        }
    }
    const auto* t = e.turbo();
    std::printf("held key-off pre %.0f rpm %.0f thr %.2f: last-1s net %.1f N*m (%.1f kW), charge %.1f bar, rotor %.0f rpm\n", pre_rpm, rpm,
                throttle, sum / n, sum / n * s.omega / 1000.0, (t->charge_pa - 101325.0) / 1e5, t->rpm());
    out.mean_net_nm = sum / n;
    return out;
}
} // namespace

namespace {
struct FreeResult {
    double max_rpm = 0.0, end_rpm = 0.0, max_charge_bar = 0.0, max_rotor_rpm = 0.0, crank_work_j = 0.0;
};
// Free crank, key off: after a fired WOT pre-spin at pre_rpm (rotors spun up), the key goes off and the crank (engine plus
// crank generator inertia) runs free from rpm0 with the plate held at 1 or toggled 1/0 every toggle_s. Only the stored
// rotor and crank energy can turn it; it must never end faster than it started.
FreeResult free_keyoff(double pre_rpm, double rpm0, double toggle_s, double run_s, bool verbose) {
    vehicle::VehicleDesc desc = load_runaway_car();
    const drivetrain::SimulatedEngineDesc* sd = nullptr;
    for (auto& c : desc.powertrain.components)
        if (std::holds_alternative<drivetrain::SimulatedEngineDesc>(c.params)) sd = &std::get<drivetrain::SimulatedEngineDesc>(c.params);
    REQUIRE(sd != nullptr);
    drivetrain::SimulatedEngine e(*sd);
    e.reset(drivetrain::EngineState::Running);
    const double h = 1.0 / 960.0, k = 2.0 * 3.14159265358979 / 60.0;
    drivetrain::Sensors s;
    s.ignition = 1.0;
    s.omega = pre_rpm * k;
    for (int i = 0; i < 960 * 5; ++i) {
        (void)e.pre_solve(s, drivetrain::ThrottleCommand{1.0, 0.0, 1.0}, drivetrain::TcuRequest{}, h);
        (void)e.post_solve(s.omega, h);
    }
    s.ignition = 0.0;
    s.omega = rpm0 * k;
    const double inertia = sd->inertia_kgm2 + 0.1; // the owner's crank generator rotor
    const double c_k = e.maps().c_k;
    FreeResult out;
    const int steps = static_cast<int>(run_s * 960.0);
    for (int i = 0; i < steps; ++i) {
        const double thr = toggle_s > 0.0 ? ((static_cast<int>(i * h / toggle_s) % 2) == 0 ? 1.0 : 0.0) : 1.0;
        const drivetrain::PreSolveResult r = e.pre_solve(s, drivetrain::ThrottleCommand{thr, 0.0, 1.0}, drivetrain::TcuRequest{}, h);
        const double coulomb = s.omega > 0.0 ? c_k : 0.0;
        const double omega_new = std::max(0.0, s.omega + h * (r.tau0 - coulomb) / (inertia + h * std::max(0.0, double(r.d))));
        out.crank_work_j += h * (r.tau0 - coulomb) * omega_new;
        s.omega = omega_new;
        (void)e.post_solve(s.omega, h);
        const auto* t = e.turbo();
        out.max_rpm = std::max(out.max_rpm, s.omega / k);
        out.max_charge_bar = std::max(out.max_charge_bar, (t->charge_pa - 101325.0) / 1e5);
        out.max_rotor_rpm = std::max(out.max_rotor_rpm, t->rpm());
        if (verbose && i % 96 == 0)
            std::printf("  t=%5.2f thr=%.0f rpm=%8.0f net=%8.1f p_im=%8.1f p_ch=%8.1f trpm=%9.0f comp_kW=%8.1f turb_kW=%8.1f flow=%.3f\n",
                        i * h, thr, s.omega / k, r.tau0 - coulomb, e.intake_pressure_pa() / 1000.0, t->charge_pa / 1000.0, t->rpm(),
                        t->compressor_w / 1000.0, t->turbine_w / 1000.0, t->compressor_flow_kg_s);
    }
    out.end_rpm = s.omega / k;
    return out;
}
} // namespace

TEST_CASE("hybrid crank runaway free crank sweep", "[.][diag][hybrid_runaway_free]") {
    for (double rpm0 : {300.0, 1100.0, 2000.0, 6000.0, 8200.0, 12000.0, 20000.0})
        for (double period : {0.0, 0.1, 0.3, 0.6}) {
            const FreeResult r = free_keyoff(8000.0, rpm0, period, 15.0, false);
            std::printf("free key-off from %.0f rpm, toggle %.2f s: max %.0f end %.0f rpm, max charge %.1f bar, max rotor %.0f rpm, "
                        "crank work %.1f kJ\n",
                        rpm0, period, r.max_rpm, r.end_rpm, r.max_charge_bar, r.max_rotor_rpm, r.crank_work_j / 1000.0);
        }
}

TEST_CASE("hybrid crank runaway owner timeline", "[.][diag][hybrid_runaway2]") {
    // The owner's 1 Hz RG_DRIVE pedal/brake samples from t=49 s (lift at 357 km/h), after a WOT run-up to that speed.
    const RunawayResult r = run({{12.4, 1.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {1.0, 0.04, 0.0}, {1.0, 0.0, 0.0},
                                 {1.0, 0.028, 0.0}, {1.0, 0.112, 0.0}, {1.0, 0.128, 0.0}, {1.0, 0.132, 0.0}, {2.0, 1.0, 0.0},
                                 {1.0, 0.0, 0.0}, {14.0, 1.0, 0.0}, {10.0, 0.0, 0.0}},
                                0.0, true);
    std::printf("max rpm %.0f at %.2f s (unfired max %.0f), max boost %.2f bar, max turbo %.0f rpm, max rel balance %.2e\n", r.max_rpm,
                r.t_max_rpm, r.max_rpm_off, r.max_boost_bar, r.max_turbo_rpm, r.max_rel_balance);
}

TEST_CASE("hybrid crank runaway owner timeline tapping", "[.][diag][hybrid_runaway_tap]") {
    for (double period : {0.15, 0.3, 0.6}) {
        std::vector<Segment> segs = {{12.4, 1.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, {1.0, 0.0, 0.0},  {1.0, 0.04, 0.0},
                                     {1.0, 0.0, 0.0},  {1.0, 0.028, 0.0}, {1.0, 0.112, 0.0}, {1.0, 0.128, 0.0}, {1.0, 0.132, 0.0}};
        for (double t = 0.0; t < 17.0; t += 2.0 * period) {
            segs.push_back({period, 1.0, 0.0});
            segs.push_back({period, 0.0, 0.0});
        }
        segs.push_back({10.0, 0.0, 0.0});
        const RunawayResult r = run(segs, 0.0, false);
        std::printf("tapping %.2f s: max rpm %.0f at %.2f s (unfired max %.0f), max boost %.2f bar, max turbo %.0f rpm, max rel balance %.2e\n",
                    period, r.max_rpm, r.t_max_rpm, r.max_rpm_off, r.max_boost_bar, r.max_turbo_rpm, r.max_rel_balance);
    }
}

TEST_CASE("hybrid crank runaway held key-off sweep", "[.][diag][hybrid_runaway_held]") {
    for (double rpm : {3000.0, 6000.0, 8000.0, 10000.0, 15000.0, 20000.0, 30000.0, 40000.0})
        held_keyoff(8000.0, rpm, 1.0, 10.0, false);
}
TEST_CASE("hybrid crank runaway held key-off trace", "[.][diag][hybrid_runaway_held_trace]") {
    held_keyoff(8000.0, 20000.0, 1.0, 10.0, true);
}
TEST_CASE("hybrid crank runaway held key-off toggling plate", "[.][diag][hybrid_runaway_toggle]") {
    for (double period : {0.1, 0.3, 0.6, 1.2})
        for (double rpm : {6000.0, 10000.0, 20000.0, 30000.0}) {
            std::printf("toggle %.2f s: ", period);
            held_keyoff(8000.0, rpm, 1.0, 9.6, false, period);
        }
}
TEST_CASE("hybrid crank runaway held key-off toggling trace", "[.][diag][hybrid_runaway_toggle_trace]") {
    held_keyoff(8000.0, 30000.0, 1.0, 4.8, true, 0.3);
}
// ---------------------------------------------------------------------------------------------------------------------------
// Regression tests [drivetrain][hybrid][turbo] (not hidden). The defect: TurboPair::step's semi-implicit overspeed term (the
// derivative of the analytic head parabola, applied to the charge lag) was also applied past choke, where the head is clamped
// to 0 and its derivative is 0. A charge left above the head then decayed with tau ~90 ms instead of 25 ms while the engine
// drew several times the choke flow and the compressor paid nothing for it (pr_wheel = 1), so a plate opened onto a deadheaded
// charge got free high-pressure air and the turbine free exhaust energy. Recorded sabotage (the past_choke gate removed,
// 2026-11-08): P1 decays 0.0120 of the gap per substep (bound >= 0.95 * alpha = 0.038); T1 (held 30000 rpm, plate toggled
// every 0.3 s) mean net crank torque +246920 N*m, max charge 415802 bar, max rotor 84.8M rpm - fixed: -230.2 N*m, charge and
// rotor bounded by the pre-spin deadhead (rotor <= 516k rpm). F1 and R1 are guards (they pass with the sabotage too: a FREE
// crank coasts down before the leak can build; R1 key-off max 7856 rpm, ledger 4.3e-11 relative from tick 1).
TEST_CASE("hybrid crank runaway P1: a charge above a choked overspeed compressor decays at the lag rate",
          "[drivetrain][hybrid][turbo]") {
    vehicle::VehicleDesc desc = load_runaway_car();
    const drivetrain::SimulatedEngineDesc* sd = nullptr;
    for (auto& c : desc.powertrain.components)
        if (std::holds_alternative<drivetrain::SimulatedEngineDesc>(c.params)) sd = &std::get<drivetrain::SimulatedEngineDesc>(c.params);
    REQUIRE(sd != nullptr);
    REQUIRE(sd->turbo_pair.has_value());
    const drivetrain::TurboPairDesc& td = *sd->turbo_pair;
    REQUIRE(td.maps != nullptr);
    REQUIRE(td.maps->stages == nullptr); // the analytic law (the garage kit has no stage maps)
    drivetrain::TurboPair t(td);
    const double amb = 101325.0, h = 1.0 / 960.0;
    t.reset(amb, 288.15);
    // Rotor at 2.6x the reference speed (the kit's rotors after a 5 bar WOT run: ~470k of 180k rpm), past the map's speed edge.
    const double wr = td.reference_rpm * 2.0 * 3.14159265358979323846 / 60.0, speed = 2.6;
    REQUIRE(speed > td.maps->head_pa.x.back());
    t.energy_j = 0.5 * td.rotor_inertia_kgm2 * wr * wr * speed * speed;
    t.charge_pa = 30.0e5; // a deadheaded charge left by a closed plate
    // The engine draws 9 kg/s through the reopened plate: corrected flow far past choke at this speed.
    const double air = 9.0;
    REQUIRE(air / td.max_mass_flow_kg_s / (speed / td.maps->head_pa.x.back()) > 1.0);
    const double before = t.charge_pa;
    t.step(h, amb, 288.15, air, air, 900.0, 1.0, 9.0e5);
    const double alpha = h / (0.025 + h);
    const double decayed = (before - t.charge_pa) / (before - amb); // desired = ambient (head 0, no cooler drop)
    CHECK(decayed >= 0.95 * alpha);
    CHECK(decayed <= 1.0001 * alpha);
    CHECK(t.compressor_w == 0.0); // past choke the wheel makes no head and does no work
}

TEST_CASE("hybrid crank runaway T1: key-off plate toggling on a held crank creates no energy", "[drivetrain][hybrid][turbo]") {
    // Held 30000 rpm, key off after a fired WOT pre-spin at 8000 rpm, plate toggled 1/0 every 0.3 s for 9.6 s (the
    // hybrid supervisor's rev-match / split throttle hand-over flips the plate like this). With no fuel the gas path can only
    // take work from the crank: the mean net crank torque over whole periods must be negative.
    const HeldResult r = held_keyoff(8000.0, 30000.0, 1.0, 9.6, false, 0.3);
    CHECK(r.mean_net_nm < 0.0);
    // The kit's deadhead head at its pre-spin rotor speed (~516k rpm, 2.87x the 180k reference at 4.5 bar) is ~37 bar;
    // the rotor itself must not gain energy from the toggling.
    CHECK(r.max_charge_bar < 45.0);
    CHECK(r.max_rotor_rpm < 600000.0);
}

TEST_CASE("hybrid crank runaway F1: key-off free crank on spinning rotors stays below the limiter", "[drivetrain][hybrid][turbo]") {
    // The owner's free crank (engine + crank generator inertia, key off) with rotors spun by a 5 s fired WOT run at 8000 rpm,
    // plate toggled every 0.3 s for 15 s from 2000 rpm: the stored rotor energy can turn the crank as an air motor, but only
    // the energy it holds - the crank stays below the 8100 rpm limiter plus a margin and coasts to rest. A guard: it passes
    // on the pre-fix code too (max 3882 rpm), the defect needs a crank held at speed (T1).
    const FreeResult r = free_keyoff(8000.0, 2000.0, 0.3, 15.0, false);
    CHECK(r.max_rpm < 8100.0 + 500.0);
    CHECK(r.end_rpm < 1.0);
    CHECK(r.max_charge_bar < 45.0);
    CHECK(r.max_rotor_rpm < 600000.0);
}

TEST_CASE("hybrid crank runaway R1: engine rig key-off plate toggling, limiter and ledger", "[drivetrain][hybrid][turbo]") {
    // The real Powertrain (EngineRig: crank + engine, 240/960 Hz): start, 8 s WOT into the 8100 rpm fuel cut (rotors to
    // ~515k rpm, 5 bar), then key off with the plate toggled 1/0 every 0.3 s for 12 s. The crank stays below the limiter
    // plus a margin, the boost is bounded by the rotor's deadhead head, and the PowertrainEnergy ledger closes every tick.
    vehicle::VehicleDesc desc = load_runaway_car();
    drivetrain::ComponentParams* eng = nullptr;
    for (auto& c : desc.powertrain.components)
        if (std::holds_alternative<drivetrain::SimulatedEngineDesc>(c.params)) eng = &c.params;
    REQUIRE(eng != nullptr);
    const double limiter = std::get<drivetrain::SimulatedEngineDesc>(*eng).limiter.rpm;
    EngineRig rig(*eng, nullptr);
    const int hz = 240;
    double max_rpm_keyoff = 0.0, max_boost = 0.0, max_rotor = 0.0, max_rel = 0.0;
    int ticks = 0;
    auto run_for = [&](double seconds, double ignition, double starter, double toggle_s, bool key_off) {
        const int n = static_cast<int>(seconds * hz);
        for (int k = 0; k < n; ++k) {
            const double thr = toggle_s > 0.0 ? ((static_cast<int>(k / (toggle_s * hz)) % 2) == 0 ? 1.0 : 0.0) : 1.0;
            rig.set_throttle(starter > 0.5 ? 0.0 : thr);
            rig.set_ignition(ignition);
            rig.set_starter(starter);
            const drivetrain::PowertrainEnergy prev = rig.ledger();
            rig.step();
            const drivetrain::PowertrainEnergy& e = rig.ledger();
            const auto& es = rig.engine_sound_state();
            if (key_off) max_rpm_keyoff = std::max(max_rpm_keyoff, rig.engine_rpm());
            max_boost = std::max(max_boost, double(es.boost_bar));
            max_rotor = std::max(max_rotor, double(es.turbo_rpm));
            const double scale = std::max({std::abs(e.engine_out - prev.engine_out), std::abs(e.rotor_ke - prev.rotor_ke), 1.0});
            // Tick 0 carries the unbooked seed of the initial crank speed (vault PHYS-049 F-T0); the ledger closes from tick 1.
            if (++ticks > 1) max_rel = std::max(max_rel, std::abs(double(e.balance_error)) / scale);
        }
    };
    run_for(1.0, 1.0, 1.0, 0.0, false);
    run_for(8.0, 1.0, 0.0, 0.0, false);
    run_for(12.0, 0.0, 0.0, 0.3, true);
    std::printf("R1: key-off max %.0f rpm (limiter %.0f), max boost %.2f bar, max rotor %.0f rpm, max rel balance %.2e\n", max_rpm_keyoff,
                limiter, max_boost, max_rotor, max_rel);
    CHECK(max_rpm_keyoff < limiter + 500.0);
    CHECK(max_boost < 45.0);
    CHECK(max_rotor < 600000.0);
    CHECK(max_rel < 5e-8);
}
