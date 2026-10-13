// Boost B0 (2026-10-13) hidden diagnostics [.][diag][boost_b0]: the hypercar boost-control bugs reported by the
// racing_game session (owner drive on car_hyper_n2o with the garage setup: hyper_twin_large_n2o, target 15.48 bar).
// Ported from racing_game tests/unit/test_hyper_power_jumps.cpp (read-only there).
//
// Cars (env PS_B0_CAR):
//   stock - data/vehicles/car_hyper_n2o.json (physics_sim's own: hyper_twin_n2o, no stage map)
//   owner - tests/support/fixtures/assists/v12/vehicle_v12.json (the owner's garage setup on car_hyper_n2o, stage map
//           turbo_map_large.json = racing_game data/turbo_maps/large.json) plus data/electronics/
//           drive_modes_car_hyper_n2o.json and car_hyper_n2o's current auto-shift min_time_in_gear_s 2.0 /
//           kickdown_throttle 0.95 (the fixture predates assists I7), so its auto-shift matches the shipped car.
//   PS_B0_FI = a forced_induction file (absolute path) replacing the engine's slot (e.g. racing_game's
//           hyper_twin_standard_n2o.json to reproduce item 3 on the standard stage map; read-only use).
//
// Case "B0 drive trace": a straight 20 km strip (20 m cells), auto-shift on, profile PS_B0_PROFILE:
//   wot   - PS_B0_LAUNCH_THROTTLE (0.6) until PS_B0_LAUNCH_KMH (40), then 1.0
//   hold  - wot up to PS_B0_HOLD_KMH, then a PI speed hold on the pedal (racing_game's gains); PS_B0_FREEZE_S > 0
//           freezes the pedal after that many seconds of hold
//   const - PS_B0_THROTTLE (0.3) all the time
// PS_B0_SECONDS (60), PS_B0_CSV (per-tick CSV), PS_B0_TC (assist.traction_control code, default the car's).
// Prints one line per second plus every gear change and, per upshift, the boost recovery time (new-gear engagement
// until the charge, after first falling below 90 % of its max over the 0.25 s before the shift began, is back at
// >= 90 %; 0 = never below in 3 s, -1 = not within 8 s).
// Case "B0 part-throttle dyno" (item 1): SimulatedEngine with the car's engine + turbo held at PS_B0_DYNO_RPM (5000)
// for PS_B0_DYNO_S (20) s per throttle command 0.1..1.0, from a cold rotor and from a rotor spun 3 s at command 1;
// prints the last-0.5 s mean power, boost, rotor speed, gate and plate (the bistable/cliff part-throttle map).
//
// Case "B0 frozen rotor sweep": white-box TurboPair (core/src) from the car's turbo desc; for each rotor speed the
// rotor energy is held, air/exhaust flow fixed (PS_B0_AIR_KG_S, PS_B0_EXH_KG_S, PS_B0_PORT_K), plate 1, and the
// charge pressure the pair settles at is printed with the stage-map coverage flags - the seam diagnostic (item 3).
#include "vehicle_test_helpers.h"

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

using namespace ps;

namespace {

std::string env_or(const char* name, const std::string& fallback) {
#if defined(_MSC_VER) || defined(_WIN32)
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return fallback;
    std::string out(buf);
    std::free(buf);
    return out.empty() ? fallback : out;
#else
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
#endif
}
real env_num(const char* name, real fallback) {
    const std::string s = env_or(name, "");
    return s.empty() ? fallback : std::stod(s);
}

std::string cache_dir() {
    const std::string d = std::string(PS_SOURCE_DIR) + "/out/boost_b0_map_cache";
    std::filesystem::create_directories(d);
    return d;
}

vehicle::VehicleDesc load_car(const std::string& car) {
    jobs::JobSystem pool(10);
    io::VehicleLoadOptions opt;
    opt.maps.pool = &pool;
    opt.maps.cache_dir = cache_dir();
    std::string path;
    if (car == "owner") {
        path = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/assists/v12/vehicle_v12.json";
        opt.electronics["drive_modes"] = std::string(PS_SOURCE_DIR) + "/data/electronics/drive_modes_car_hyper_n2o.json";
    } else {
        path = std::string(PS_SOURCE_DIR) + "/data/vehicles/car_hyper_n2o.json";
    }
    const std::string fi = env_or("PS_B0_FI", "");
    if (!fi.empty()) opt.part_overrides["engine"].slots["forced_induction"] = fi;
    vehicle::VehicleDesc d = io::load_vehicle_json(path, opt);
    if (car == "owner") {
        for (auto& c : d.controllers)
            if (auto* m = std::get_if<drivetrain::ManualTcuDesc>(&c.params)) {
                m->auto_shift.min_time_in_gear_s = 2.0;
                m->auto_shift.kickdown_throttle = 0.95;
            }
    }
    return d;
}

const drivetrain::SimulatedEngineDesc* sim_engine(const vehicle::VehicleDesc& d) {
    for (const auto& c : d.powertrain.components)
        if (const auto* s = std::get_if<drivetrain::SimulatedEngineDesc>(&c.params)) return s;
    return nullptr;
}

// A straight strip along +x: [-200, 20000] x [-20, 20] m in 20 m cells, one surface.
MeshShape strip_mesh(SurfaceId sid) {
    constexpr real kCell = 20.0, x0 = -200.0, x1 = 20000.0, y0 = -20.0, y1 = 20.0;
    const int nx = static_cast<int>((x1 - x0) / kCell), ny = static_cast<int>((y1 - y0) / kCell);
    MeshShape m;
    for (int j = 0; j <= ny; ++j)
        for (int i = 0; i <= nx; ++i) m.vertices.push_back(Vec3{x0 + i * kCell, y0 + j * kCell, 0.0});
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

} // namespace

TEST_CASE("B0 drive trace", "[.][diag][boost_b0]") {
    const std::string car = env_or("PS_B0_CAR", "stock");
    const vehicle::VehicleDesc desc = load_car(car);
    const auto* eng = sim_engine(desc);
    REQUIRE(eng != nullptr);
    REQUIRE(eng->turbo_pair.has_value());
    std::printf("B0 car=%s target_boost=%.3f bar stage_maps=%d ref_rpm=%.0f max_flow=%.3f wg_area=%.3e speed_edge=%.3f\n",
                car.c_str(), eng->turbo_pair->target_boost_pa / 1e5,
                eng->turbo_pair->maps && eng->turbo_pair->maps->stages ? 1 : 0, eng->turbo_pair->reference_rpm,
                eng->turbo_pair->max_mass_flow_kg_s, eng->turbo_pair->wastegate_area(),
                eng->turbo_pair->maps ? eng->turbo_pair->maps->head_pa.x.back() : 1.5);

    WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    cfg.job_workers = 4;
    World w(cfg);
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    BodyDesc ground;
    ground.shape = strip_mesh(surfaces->id_for("asphalt"));
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    w.create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.5};
    const BodyId body = w.create_body(chassis);
    w.set_surface_table(surfaces);
    const VehicleId vid = w.create_vehicle(desc, body);

    w.set_control("ignition", 1.0);
    w.set_control("starter", 1.0);
    w.set_control("assist.auto_shift", 1.0);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 1.0);
    if (const std::string tc = env_or("PS_B0_TC", ""); !tc.empty()) w.set_control("assist.traction_control", std::stod(tc));
    ps_test::settle(w, 2.0);

    const std::string profile = env_or("PS_B0_PROFILE", "wot");
    const real seconds = env_num("PS_B0_SECONDS", 60.0);
    const real launch_throttle = env_num("PS_B0_LAUNCH_THROTTLE", 0.6), launch_kmh = env_num("PS_B0_LAUNCH_KMH", 40.0);
    const real hold_kmh = env_num("PS_B0_HOLD_KMH", 200.0), const_throttle = env_num("PS_B0_THROTTLE", 0.3);
    const real freeze_s = env_num("PS_B0_FREEZE_S", 0.0);
    std::ofstream csv;
    if (const std::string p = env_or("PS_B0_CSV", ""); !p.empty()) {
        csv.open(p);
        csv << "t,kmh,gear,gear_target,shift_phase,rpm,pedal,throttle_cmd,plate,theta_c,ignition_cut,eng_kw,boost_bar,"
               "charge_bar,wastegate,turbo_rpm,comp_flow,comp_pr,comp_stage,turb_stage,fallback,p_exh_kpa,limiter,tc_frac\n";
    }
    const real dt = w.dt();
    const long steps = static_cast<long>(seconds / dt);
    real pedal = 0.0, integ = 0.0, hold_t = 0.0;
    bool started = false, holding = false, frozen = false;
    real prev_out = w.powertrain_energy(vid).engine_out;
    int prev_gear = w.powertrain_state(vid).gear;
    int upshifts = 0, downshifts = 0;
    // Upshift recovery bookkeeping: charge (gauge, bar) over the last 0.25 s before the shift began.
    std::vector<real> charge_hist;
    real pending_ref = -1.0, pending_t = 0.0, shift_ref = 0.0, min_charge = 1e9;
    bool dipped = false;
    int last_gear = prev_gear;
    drivetrain::ShiftPhase prev_phase = drivetrain::ShiftPhase::Idle;
    std::vector<real> recov;
    for (long k = 0; k < steps; ++k) {
        const real t = k * dt;
        const Motion mo0 = w.get_motion(body);
        const real kmh0 = mo0.linear.x * 3.6;
        const auto& pt0 = w.powertrain_state(vid);
        if (!started && (pt0.engine_state == drivetrain::EngineState::Running || t > 6.0)) {
            w.set_control("starter", 0.0);
            started = true;
        }
        if (started) {
            if (profile == "const") {
                pedal = const_throttle;
            } else if (profile == "hold" && (holding || kmh0 >= hold_kmh)) {
                if (!holding) { holding = true; integ = 0.6; }
                hold_t += dt;
                if (freeze_s > 0.0 && hold_t >= freeze_s && !frozen) {
                    frozen = true;
                    std::printf("B0 pedal frozen at %.4f (t=%.1f)\n", pedal, t);
                }
                if (!frozen) {
                    const real e = (hold_kmh - kmh0) / 3.6;
                    integ = std::clamp(integ + 0.05 * e * dt, 0.0, 1.0);
                    pedal = std::clamp(0.08 * e + integ, 0.0, 1.0);
                }
            } else {
                pedal = kmh0 < launch_kmh ? launch_throttle : 1.0;
            }
            w.set_control("throttle", pedal);
        }
        w.step();
        const auto& pt = w.powertrain_state(vid);
        const auto dbg = w.powertrain_debug(vid);
        const real out = w.powertrain_energy(vid).engine_out;
        const real eng_kw = (out - prev_out) / dt / 1000.0;
        prev_out = out;
        drivetrain::EngineSoundState es{};
        if (!pt.engines.empty()) es = pt.engines[0];
        const real kmh = w.get_motion(body).linear.x * 3.6;
        const real charge = es.p_charge_pa > 0 ? (es.p_charge_pa - 101325.0) / 1e5 : 0.0;
        // Recovery reference: the charge before the SHIFT began (phase leaving Idle; the cut's deadhead starts there).
        if (pt.shift_phase != drivetrain::ShiftPhase::Idle && prev_phase == drivetrain::ShiftPhase::Idle && !charge_hist.empty())
            shift_ref = *std::max_element(charge_hist.begin(), charge_hist.end());
        prev_phase = pt.shift_phase;
        if (pt.gear != prev_gear) {
            std::printf("B0 shift t=%6.2f %d->%d kmh=%6.1f rpm=%5.0f pedal=%.3f charge=%.2f wg=%.2f turbo=%6.0f\n", t, prev_gear,
                        int(pt.gear), kmh, pt.engine_rpm, pedal, charge, es.wastegate_opening, es.turbo_rpm);
            if (pt.gear > 0) {
                if (last_gear > 0 && pt.gear > last_gear) {
                    ++upshifts;
                    pending_ref = shift_ref;
                    pending_t = t;
                    dipped = false;
                    min_charge = 1e9;
                }
                if (last_gear > 0 && pt.gear < last_gear) { ++downshifts; pending_ref = -1.0; }
                last_gear = pt.gear;
            }
            prev_gear = pt.gear;
        }
        charge_hist.push_back(charge);
        if (charge_hist.size() > 60) charge_hist.erase(charge_hist.begin());
        // Recovery: from the new gear's engagement until the charge, after first falling below 90 % of the pre-shift
        // charge (the closed-plate deadhead spike during the cut is skipped that way), is back at >= 90 %.
        if (pending_ref >= 0 && pt.gear > 0) {
            min_charge = std::min(min_charge, charge);
            if (charge < 0.9 * pending_ref) dipped = true;
            if (pending_ref < 0.1) {
                pending_ref = -1.0;
            } else if (dipped && charge >= 0.9 * pending_ref) {
                recov.push_back(t - pending_t);
                std::printf("B0 recovery t=%6.2f ref=%.2f bar min=%.2f -> %.3f s\n", t, pending_ref, min_charge, recov.back());
                pending_ref = -1.0;
            } else if (!dipped && t - pending_t > 3.0) {
                recov.push_back(0.0);
                std::printf("B0 recovery t=%6.2f ref=%.2f bar -> never below 90 %% in 3 s\n", t, pending_ref);
                pending_ref = -1.0;
            } else if (t - pending_t > 8.0) {
                recov.push_back(-1.0);
                std::printf("B0 recovery t=%6.2f ref=%.2f bar min=%.2f -> not within 8 s (charge %.2f)\n", t, pending_ref, min_charge, charge);
                pending_ref = -1.0;
            }
        }
        if (csv.is_open())
            csv << t << ',' << kmh << ',' << int(pt.gear) << ',' << int(pt.gear_target) << ',' << int(pt.shift_phase) << ','
                << pt.engine_rpm << ',' << pedal << ',' << es.throttle_command << ',' << es.throttle_position << ',' << es.load
                << ',' << (dbg.ignition_cut ? 1 : 0) << ',' << eng_kw << ',' << es.boost_bar << ',' << charge << ','
                << es.wastegate_opening << ',' << es.turbo_rpm << ',' << es.compressor_flow_kg_s << ','
                << es.compressor_pressure_ratio << ',' << int(es.turbo_compressor_stage_used) << ','
                << int(es.turbo_turbine_stage_used) << ',' << int(es.turbo_stage_map_fallback) << ',' << es.p_exhaust_pa / 1000.0
                << ',' << int(es.limiter_active) << ',' << pt.traction_torque_fraction << '\n';
        if (k % 240 == 0)
            std::printf("B0 t=%5.1f kmh=%6.1f gear=%d rpm=%5.0f pedal=%.3f plate=%.3f eng_kw=%7.1f charge=%5.2f wg=%.2f turbo=%6.0f fb=%d\n",
                        t, kmh, int(pt.gear), pt.engine_rpm, pedal, es.throttle_position, eng_kw, charge, es.wastegate_opening,
                        es.turbo_rpm, int(es.turbo_stage_map_fallback));
        if (w.get_pose(body).position.x > 19500.0) {
            std::printf("B0 ran out of ground at t=%.1f\n", t);
            break;
        }
    }
    std::printf("B0 SUMMARY car=%s profile=%s upshifts=%d downshifts=%d recoveries:", car.c_str(), profile.c_str(), upshifts,
                downshifts);
    for (real r : recov) std::printf(" %.3f", r);
    std::printf("\n");
    SUCCEED();
}

TEST_CASE("B0 frozen rotor sweep", "[.][diag][boost_b0]") {
    const std::string car = env_or("PS_B0_CAR", "stock");
    const vehicle::VehicleDesc desc = load_car(car);
    const auto* eng = sim_engine(desc);
    REQUIRE(eng != nullptr);
    REQUIRE(eng->turbo_pair.has_value());
    const auto& td = *eng->turbo_pair;
    const real air = env_num("PS_B0_AIR_KG_S", 0.8), exh = env_num("PS_B0_EXH_KG_S", 0.85), port_k = env_num("PS_B0_PORT_K", 1150.0);
    const real rpm_lo = env_num("PS_B0_RPM_LO", 20000.0), rpm_hi = env_num("PS_B0_RPM_HI", 240000.0), rpm_step = env_num("PS_B0_RPM_STEP", 2500.0);
    const real h = 1.0 / 960.0;
    std::printf("B0 sweep car=%s air=%.3f exh=%.3f port_k=%.0f ref_rpm=%.0f speed_edge_rpm=%.0f\n", car.c_str(), air, exh, port_k,
                td.reference_rpm, (td.maps ? td.maps->head_pa.x.back() : 1.5) * td.reference_rpm);
    std::printf("B0 sweep rpm charge_bar(after 0.05s) charge_bar(settled) steps_to_99pct comp_w_kw turb_w_kw stage fallback\n");
    for (real rpm = rpm_lo; rpm <= rpm_hi + 1e-9; rpm += rpm_step) {
        drivetrain::TurboPair tp(td);
        tp.reset(101325.0, 288.15);
        const real omega = rpm * 6.2831853071795864769 / 60.0;
        const real e0 = 0.5 * td.rotor_inertia_kgm2 * omega * omega;
        real c50 = 0.0;
        std::vector<real> hist;
        for (int i = 0; i < 4800; ++i) {
            tp.energy_j = e0;
            tp.step(h, 101325.0, 288.15, air, exh, port_k, 1.0);
            hist.push_back(tp.charge_pa);
            if (i == 47) c50 = tp.charge_pa;
        }
        const real fin = hist.back();
        int n99 = -1;
        for (std::size_t i = 0; i < hist.size(); ++i)
            if (std::abs(hist[i] - fin) <= 0.01 * std::max(1.0, std::abs(fin - 101325.0))) { n99 = static_cast<int>(i); break; }
        std::printf("B0 sweep %7.0f %7.3f %7.3f %5d %8.1f %8.1f %d %d\n", rpm, (c50 - 101325.0) / 1e5, (fin - 101325.0) / 1e5, n99,
                    tp.compressor_w / 1000.0, tp.turbine_w / 1000.0, int(tp.compressor_stage_used), int(tp.stage_map_fallback));
    }
    SUCCEED();
}

// Case "B0 part-throttle dyno" (item 1, bistability): the car's SimulatedEngine held at PS_B0_DYNO_RPM (5000) as in
// run_dyno_sweep's stepped kind (960 Hz, Sensors::omega fixed, TcuRequest{}), at each throttle command 0.1..1.0 from two
// initial states: COLD (reset Running, rotor at rest) and SPUN (3 s at command 1 first). Each runs PS_B0_DYNO_S (20)
// simulated seconds at the command; the mean over the last 0.5 s of net power ((tau0 - c_k) * omega), charge boost, rotor
// rpm, gate and plate is printed. A spun/cold power ratio far from 1 at one steady command is the bistability.
TEST_CASE("B0 part-throttle dyno", "[.][diag][boost_b0]") {
    const std::string car = env_or("PS_B0_CAR", "stock");
    const vehicle::VehicleDesc desc = load_car(car);
    const auto* eng = sim_engine(desc);
    REQUIRE(eng != nullptr);
    drivetrain::SimulatedEngineDesc d = *eng;
    d.fuels.resize(1);
    const real rpm = env_num("PS_B0_DYNO_RPM", 5000.0), run_s = env_num("PS_B0_DYNO_S", 20.0);
    const real h = 1.0 / 960.0, omega = rpm * 6.2831853071795864769 / 60.0;
    std::printf("B0 dyno car=%s rpm=%.0f target_boost=%.3f bar\n", car.c_str(), rpm, d.turbo_pair->target_boost_pa / 1e5);
    std::printf("B0 dyno cmd | cold: kW boost_bar rotor_rpm gate plate | spun: kW boost_bar rotor_rpm gate plate\n");
    for (int ic = 1; ic <= 10; ++ic) {
        const real cmd = 0.1 * ic;
        real res[2][5] = {};
        for (int spun = 0; spun < 2; ++spun) {
            drivetrain::SimulatedEngine e(d);
            e.reset(drivetrain::EngineState::Running);
            drivetrain::Sensors s;
            s.omega = omega;
            s.ignition = 1.0;
            const real c_k = e.maps().c_k;
            const long pre = spun ? static_cast<long>(3.0 / h) : 0L, n = static_cast<long>(run_s / h), win = static_cast<long>(0.5 / h);
            real acc[5] = {};
            for (long i = 0; i < pre + n; ++i) {
                const drivetrain::ThrottleCommand th{i < pre ? 1.0 : cmd, 0.0, 1.0};
                const drivetrain::PreSolveResult r = e.pre_solve(s, th, drivetrain::TcuRequest{}, h);
                (void)e.post_solve(omega, h);
                if (i >= pre + n - win) {
                    acc[0] += (r.tau0 - c_k) * omega / 1000.0;
                    acc[1] += (r.p_charge_pa - 101325.0) / 1e5;
                    acc[2] += r.turbo_rpm;
                    acc[3] += r.wastegate_opening;
                    acc[4] += r.throttle_position;
                }
            }
            for (int j = 0; j < 5; ++j) res[spun][j] = acc[j] / static_cast<real>(win);
        }
        std::printf("B0 dyno %.1f | %7.1f %6.2f %7.0f %.2f %.3f | %7.1f %6.2f %7.0f %.2f %.3f\n", cmd, res[0][0], res[0][1], res[0][2],
                    res[0][3], res[0][4], res[1][0], res[1][1], res[1][2], res[1][3], res[1][4]);
    }
    SUCCEED();
}

// The N7 whole-car pull of tests/validation/test_car_hyper_n2o.cpp (same rig, controls and shift rule), traced around the
// shifts: PS_B0_GEAR the pull gear (default 5), PS_B0_ARM 1 arms the shot, PS_B0_EVERY ticks between lines (default 4).
TEST_CASE("B0 N2O pull trace", "[.][diag][boost_b0]") {
    const vehicle::VehicleDesc desc = load_car("stock");
    const int pull_gear = static_cast<int>(env_num("PS_B0_GEAR", 5.0));
    const bool arm = env_num("PS_B0_ARM", 0.0) > 0.5;
    const int every = static_cast<int>(env_num("PS_B0_EVERY", 4.0));
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, 4);
    World& w = *rig.world;
    const real dt = w.dt();
    w.set_control("assist.traction_control", 0.0);
    w.set_control("assist.drive_mode", 2.0);
    w.set_control("assist.aero_control", 0.0);
    ps_test::settle(w, 2.0);
    w.set_control("gear_request", 1.0);
    w.set_control("throttle", 1.0);
    bool armed = false;
    for (int t = 0; t < static_cast<int>(30.0 / dt); ++t) {
        w.step();
        const auto& st = w.powertrain_state(rig.vehicle);
        const auto dbg = w.powertrain_debug(rig.vehicle);
        const auto& es = st.engines.at(0);
        const bool idle = st.shift_phase == drivetrain::ShiftPhase::Idle;
        if (idle && st.gear >= 1 && st.gear < pull_gear && st.engine_rpm >= 7000.0) w.set_control("gear_request", static_cast<real>(st.gear + 1));
        const bool in_pull = st.gear == pull_gear && idle && st.clutch_engagement >= 0.999;
        if (in_pull && !armed) {
            armed = true;
            std::printf("B0N ARM t=%.4f rpm=%.0f\n", t * dt, st.engine_rpm);
            if (arm) w.set_control("nitrous_arm", 1.0);
        }
        const real kappa = std::max(w.wheel_state(rig.vehicle, 2).slip_ratio, w.wheel_state(rig.vehicle, 3).slip_ratio);
        if (t % every == 0 || (armed && st.engine_rpm >= 7800.0))
            std::printf("B0N t=%.4f gear=%d tgt=%d ph=%d rpm=%5.0f kmh=%6.1f eng=%.3f slip=%7.1f cut=%d plate=%.3f charge=%5.2f wg=%.3f turbo=%6.0f n2o=%5.1f kappa=%.3f\n",
                        t * dt, int(st.gear), int(st.gear_target), int(st.shift_phase), st.engine_rpm, w.get_motion(rig.chassis).linear.x * 3.6,
                        st.clutch_engagement, dbg.clutch_slip * 60.0 / (2.0 * 3.14159265358979), dbg.ignition_cut ? 1 : 0, es.throttle_position,
                        (es.p_charge_pa - 101325.0) / 1e5, es.wastegate_opening, es.turbo_rpm, es.n2o_flow_g_s, kappa);
        if (armed && st.engine_rpm >= 7800.0) break;
    }
    SUCCEED();
}
