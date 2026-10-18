// tests/validation/test_n2o_oscillation.cpp -- owner report (racing_game drive, car_hyper_n2o): "N2O causes cyclical rev ups and rev downs at
// more than 0.5 Hz". Diag case (hidden) writes a per-tick CSV of a full-throttle run; the asserting case lives below it.
#include "vehicle_test_helpers.h"

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
#include <vector>

using namespace ps;

namespace {

std::string envs_(const char* n, const std::string& def = "") {
#ifdef _WIN32
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, n) != 0 || buf == nullptr) return def;
    std::string v(buf);
    std::free(buf);
    return v;
#else
    const char* v = std::getenv(n);
    return v ? std::string(v) : def;
#endif
}
double envd_(const char* n, double def) {
    const std::string v = envs_(n);
    return v.empty() ? def : std::atof(v.c_str());
}

vehicle::VehicleDesc load_car_(const std::string& file) {
    static jobs::JobSystem pool(10);
    io::EngineMapOptions opt;
    opt.quality = drivetrain::MapQuality::Full;
    opt.pool = &pool;
    opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_n2o_800hp_engine_cache").string();
    return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + (file.find('/') != std::string::npos ? "/" : "/data/vehicles/") + file, opt);
}

struct Row {
    double t, v, rpm, clutch, clutch_slip, cmd, ceil, load, boost, flow_frac, kappa, tcf;
    int gear, target, phase, limiter, dfco, ign_cut, n2o, sol, cut, tc, tc_int;
};

struct RunSpec {
    std::string car = "car_hyper_n2o.json";
    bool arm = true;
    bool auto_shift = true;
    double tc = 1.0;       // assist.traction_control channel value
    double mode = 2.0;     // assist.drive_mode (2 Sport allows the N2O arm)
    double seconds = 25.0;
    double shift_rpm = 7800.0; // manual shifts
    int max_gear = 7;
    double arm_after_s = 0.0;
    double gear_request = 0.0; // 99 = D on an automatic_tcu
    double sport = 0.0;       // transmission.mode (automatic sport schedule)
    bool legacy_controller = false; // restore the pre-fix controller behaviour (no hysteresis, no taper, every closure restarts the ramp)
};

std::vector<Row> run_pull(const RunSpec& sp) {
    vehicle::VehicleDesc desc = load_car_(sp.car);
    if (sp.legacy_controller) {
        for (drivetrain::ComponentDesc& cd : desc.powertrain.components) {
            if (auto* kit = std::get_if<drivetrain::NitrousKitDesc>(&cd.params)) {
                kit->controller.throttle_hysteresis = 0.0;
                kit->controller.max_rpm_fade_rpm = 0.0;
                kit->controller.progressive.resume_within_s = 0.0;
            }
        }
    }
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.5, 0.0, 4, Vec3::unit_y(), 240.0, 960.0, 6000.0);
    World& w = *rig.world;
    const double dt = w.dt();
    w.set_control("assist.drive_mode", sp.mode);
    w.set_control("assist.traction_control", sp.tc);
    w.set_control("assist.aero_control", 0.0);
    ps_test::settle(w, 2.0);
    w.set_control("assist.auto_shift", sp.auto_shift ? 1.0 : 0.0);
    if (sp.gear_request > 0.0) w.set_control("gear_request", sp.gear_request);
    else if (!sp.auto_shift) w.set_control("gear_request", 1.0);
    w.set_control("transmission.mode", sp.sport);
    w.set_control("throttle", 1.0);
    if (sp.arm && sp.arm_after_s <= 0.0) w.set_control("nitrous_arm", 1.0);
    std::vector<Row> rows;
    const int n = static_cast<int>(sp.seconds / dt);
    for (int i = 0; i < n; ++i) {
        const double t = i * dt;
        if (sp.arm && sp.arm_after_s > 0.0 && t >= sp.arm_after_s) w.set_control("nitrous_arm", 1.0);
        w.step();
        const drivetrain::PowertrainSnapshot& st = w.powertrain_state(rig.vehicle);
        const auto& es = st.engines.at(0);
        const drivetrain::PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
        if (!sp.auto_shift && st.shift_phase == drivetrain::ShiftPhase::Idle && st.gear >= 1 && st.gear < sp.max_gear && st.engine_rpm >= sp.shift_rpm)
            w.set_control("gear_request", static_cast<real>(st.gear + 1));
        Row r{};
        r.t = t;
        r.v = w.get_motion(rig.chassis).linear.x;
        r.rpm = st.engine_rpm;
        r.clutch = st.clutch_engagement;
        r.clutch_slip = dbg.clutch_slip * 60.0 / (2.0 * 3.14159265358979);
        r.cmd = es.throttle_command;
        r.ceil = dbg.throttle_ceiling;
        r.load = es.load;
        r.boost = es.boost_bar;
        r.flow_frac = es.n2o_flow_fraction;
        r.kappa = std::max(w.wheel_state(rig.vehicle, 2).slip_ratio, w.wheel_state(rig.vehicle, 3).slip_ratio);
        r.tcf = st.traction_torque_fraction;
        r.gear = st.gear;
        r.target = st.gear_target;
        r.phase = static_cast<int>(st.shift_phase);
        r.limiter = es.limiter_active;
        r.dfco = es.dfco_active;
        r.ign_cut = dbg.ignition_cut;
        r.n2o = es.n2o_active;
        r.sol = es.n2o_solenoid_open;
        r.cut = es.n2o_safety_cut;
        r.tc = st.traction_mode_effective;
        r.tc_int = st.traction_control_intervening;
        rows.push_back(r);
    }
    return rows;
}

void write_csv(const std::vector<Row>& rows, const std::string& path) {
    std::ofstream f(path);
    if (!f) return;
    f << "t,v,rpm,gear,target,phase,clutch,slip_rpm,cmd,ceil,load,limiter,dfco,ign_cut,n2o,sol,cut,flow_frac,boost,kappa,tc_mode,tc_int,tcf\n";
    char buf[512];
    for (const Row& r : rows) {
        std::snprintf(buf, sizeof(buf), "%.4f,%.3f,%.1f,%d,%d,%d,%.3f,%.1f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%d,%d,%.3f\n", r.t, r.v, r.rpm, r.gear,
                      r.target, r.phase, r.clutch, r.clutch_slip, r.cmd, r.ceil, r.load, r.limiter, r.dfco, r.ign_cut, r.n2o, r.sol, r.cut, r.flow_frac, r.boost,
                      r.kappa, r.tc, r.tc_int, r.tcf);
        f << buf;
    }
}

// Zig-zag swing detector: counts reversals of at least `amp` rpm in the series (a full up-down or down-up swing each).
int count_swings(const std::vector<double>& x, double amp) {
    if (x.empty()) return 0;
    int swings = 0, mode = 0; // mode 0: direction unknown, 1: rising, -1: falling
    double hi = x.front(), lo = x.front(), ext = x.front();
    for (double v : x) {
        if (mode == 0) {
            hi = std::max(hi, v);
            lo = std::min(lo, v);
            if (v - lo >= amp) { mode = 1; ext = v; }
            else if (hi - v >= amp) { mode = -1; ext = v; }
        } else if (mode == 1) {
            if (v > ext) ext = v;
            else if (ext - v >= amp) { mode = -1; ext = v; ++swings; }
        } else {
            if (v < ext) ext = v;
            else if (v - ext >= amp) { mode = 1; ext = v; ++swings; }
        }
    }
    return swings;
}

struct OscMetrics {
    int closes = 0;      // solenoid 1 -> 0 transitions
    int short_restarts = 0; // closures under 0.25 s after which the progressive flow came back at least 0.2 lower (ramp restarted)
    int swings = 0;      // >= 200 rpm reversals in top gear
    double top_gear_s = 0.0;
};

OscMetrics measure(const std::vector<Row>& rows, int top_gear) {
    OscMetrics m;
    std::vector<double> rpm;
    for (std::size_t i = 1; i < rows.size(); ++i) {
        if (rows[i - 1].sol == 1 && rows[i].sol == 0) {
            ++m.closes;
            std::size_t j = i;
            while (j < rows.size() && rows[j].sol == 0) ++j;
            if (j < rows.size() && rows[j].t - rows[i].t < 0.25 && rows[j].flow_frac <= rows[i - 1].flow_frac - 0.2) ++m.short_restarts;
        }
        if (rows[i].gear == top_gear) rpm.push_back(rows[i].rpm);
    }
    m.top_gear_s = static_cast<double>(rpm.size()) / 240.0;
    m.swings = count_swings(rpm, 200.0);
    return m;
}

} // namespace

// Owner report (racing_game, car_hyper_n2o with the hybrid DCT drivetrain): "cyclical rev ups and rev downs at more than 0.5 Hz"
// with N2O on. Mechanism (measured, 7th gear, hybrid + DCT fixture): the rpm climbs to the top of the N2O window (max_rpm 8100);
// the solenoid closes for ONE tick, which restarted the progressive ramp at start_fraction 0.4 -> the shot torque dropped by 60 %,
// the rpm fell 500-1000 rpm in ~0.2 s, then the 1 s ramp brought it back to the window edge: a ~0.9 Hz, 500-1000 rpm cycle.
// Second contributor: the hybrid supervisor's engine command hovers at the 0.95 throttle gate (0.944-0.954), so the solenoid
// toggled ~26 times a second, each time restarting the ramp. Fix (NitrousControllerDesc): ramp hold across short closures,
// throttle hysteresis, linear flow taper below max_rpm.
TEST_CASE("N2O: no rev cycle or solenoid chatter in the hybrid DCT pull", "[n2o][long][vehicle]") {
    RunSpec sp;
    sp.car = "tests/support/fixtures/vehicles/car_hyper_n2o_dct_hybrid.json";
    sp.gear_request = 99.0;
    sp.mode = 3.0;
    sp.seconds = 22.0;

    {   // the shipped controller (data/nitrous_controllers/custom_wet_800.json) carries the fix values; the defaults stay the old behaviour
        const vehicle::VehicleDesc d = load_car_(sp.car);
        bool found = false;
        for (const drivetrain::ComponentDesc& cd : d.powertrain.components) {
            if (const auto* kit = std::get_if<drivetrain::NitrousKitDesc>(&cd.params)) {
                found = true;
                CHECK(kit->controller.throttle_hysteresis > 0.0);
                CHECK(kit->controller.max_rpm_fade_rpm > 0.0);
                CHECK(kit->controller.progressive.resume_within_s > 0.0);
            }
        }
        REQUIRE(found);
    }

    sp.legacy_controller = true;
    const OscMetrics before = measure(run_pull(sp), 7);
    sp.legacy_controller = false;
    const OscMetrics after = measure(run_pull(sp), 7);
    std::printf("[n2o osc] legacy: closes %d ramp-restarts-after-short-close %d swings>=200rpm %d (top gear %.1f s); fixed: closes %d restarts %d swings %d (top gear %.1f s)\n",
                before.closes, before.short_restarts, before.swings, before.top_gear_s, after.closes, after.short_restarts, after.swings, after.top_gear_s);

    // The control run must show the owner's symptom (otherwise this test would prove nothing).
    REQUIRE(before.top_gear_s > 3.0);
    // The rev swings of the legacy controller (4+ before the hybrid shift fix) came with the hybrid's ICE command hovering
    // below full pedal (the TC ceiling folded into the request); the supervisor now keeps it at 1, so the control run
    // shows the solenoid symptom only (ramp restarts after short closures).
    CHECK(before.closes > after.closes);
    CHECK(before.short_restarts >= 2);

    REQUIRE(after.top_gear_s > 3.0);
    CHECK(after.swings <= 1);
    CHECK(after.short_restarts == 0);
}

TEST_CASE("N2O oscillation diag: full-throttle run CSV", "[.][diag][n2o]") {
    RunSpec sp;
    sp.car = envs_("PS_CAR", sp.car);
    sp.arm = envd_("PS_ARM", 1) != 0;
    sp.auto_shift = envd_("PS_AUTO_SHIFT", 1) != 0;
    sp.tc = envd_("PS_TC", 1);
    sp.mode = envd_("PS_MODE", 2);
    sp.seconds = envd_("PS_SECONDS", 25);
    sp.shift_rpm = envd_("PS_SHIFT_RPM", 7800);
    sp.max_gear = static_cast<int>(envd_("PS_MAX_GEAR", 7));
    sp.arm_after_s = envd_("PS_ARM_AFTER", 0);
    sp.gear_request = envd_("PS_GEAR_REQ", 0);
    sp.sport = envd_("PS_SPORT", 0);
    sp.legacy_controller = envd_("PS_LEGACY", 0) != 0;
    const std::vector<Row> rows = run_pull(sp);
    write_csv(rows, envs_("PS_CSV", "n2o_osc.csv"));
    std::printf("[n2o osc] wrote %zu rows\n", rows.size());
    SUCCEED();
}
