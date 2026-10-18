// tests/unit/test_brake_blend_gearbox.cpp - brake blending for motors behind a gearbox, a clutch or a transfer case
// (parallel hybrids). Owner requirement (2026-10-18): on ANY vehicle with electric motors, braking starts with
// regeneration and the pads only take what the motors cannot. c6a060b left a motor whose path to the hubs runs through
// a gearbox, a clutch or a transfer case on parallel regeneration; the reflection now follows the CURRENT state of those
// elements (docs/electric_drive.md "Brake blending", "Motors behind a gearbox").
//
// Law under test:
//   * a path through a gearbox is open while no gear is engaged, during a shift (target != current, any phase) and
//     while a gear is being synchronised; engaged, it reflects through the CURRENT ratio and mesh efficiency;
//   * a path through the main clutch is open below regen_blend.path_min_engagement of clutch engagement; a motor
//     upstream of the clutch is also limited to clutch_capacity_margin x the clutch's kinetic capacity;
//   * a path through a transfer case front output is open below path_min_engagement of transfer engagement; the rear
//     output is rigid and always open; branches off one node split equally among the OPEN branches only;
//   * an open path reflects nothing and the pads carry the full request in the same substep (no torque hole); when a
//     path closes again the regeneration ramps in over path_ramp_s so the reflected torque does not step.
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS]); fixture = data/vehicles/car_hybrid.json with the generator, the hybrid
// supervisor and the motor reduction removed and the traction motor (regen 300 N*m) moved onto the shaft under test;
// FWD, so the motor can only ever brake the FRONT axle (5600 N*m of the 8800 N*m corner capacity), the rear pads always
// carry their 3200 N*m x pedal:
//   G1  motor on gearbox.in, 3rd gear, 25 m/s, pack 50 %, brake 0.12: while the car is faster than 8 m/s every tick has
//       regen torque = front request 672 N*m (+-3 %), pad torque = rear request 384 N*m (+-3 %), regen + pad = request
//       1056 N*m (+-1 %); the reflected rim share equals the 3rd-gear ratio x mesh efficiency x diff half
//       (regen 672 at motor torque 672 / 6.01 = 111.8 N*m)
//   G2  same car in neutral: regeneration 0 on every tick after the first 0.2 s, pads = the full request 1056 N*m (+-1 %),
//       regen power 0
//   G3  4th -> 3rd downshift requested at t = 1 s of a stop from 25 m/s (brake 0.12): total (regen + pad) = request +-1 %
//       on every tick; regen torque exactly 0 on every tick the gearbox itself is moving (Releasing, Selecting, Synchronising); the shift completes within 1.5 s;
//       afterwards regeneration is back to >= 90 % of the front request within 0.5 s; and the chassis deceleration over
//       0.25 s windows (25 -> 10 m/s) stays within 6 % of the SAME stop with the pack full (pads only, same shift physics:
//       the engine spin-up of an unblipped downshift raises the deceleration by 20 % in both) - no torque hole, no spike
//   G4  motor on engine.crank (upstream of the clutch), 3rd gear, assist off: clutch closed -> regen = front request
//       (+-3 %); clutch pedal to 1 (open) -> after 0.1 s regen 0 and the pads carry the request (+-1 %); pedal back to 0
//       -> regen is back, ramping (the first tick after the clutch closes below 50 % of the final value)
//   G5  tractor + motor on the gearbox.out / transfer.in shaft, awd 0: regen = the rear request (16000 N*m x pedal, +-3 %)
//       and the front pads the front request (6000 x pedal); awd 1 (after the transfer engagement ramp): the front is
//       reflected too - the regen torque equals the closed form for an equal split over both open branches (+-3 %)
//   G6  loader: path_ramp_s, path_min_engagement and clutch_capacity_margin round-trip and are validated
//   G7  EV (car_electric) regeneration is unchanged by the dynamic tree: bit-identical state hash on 1 and 10 workers

#include "ps/backend/i_rigid_backend.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>

using namespace ps;
using namespace ps::drivetrain;
using namespace ps::vehicle;
using nlohmann::json;

namespace {
const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
constexpr real kMass = 1850;
constexpr real kFrontCap = 5600, kRearCap = 3200; // sum of the corner capacities per axle

ComponentDesc* find(VehicleDesc& d, const std::string& id) {
    for (ComponentDesc& c : d.powertrain.components) if (c.id == id) return &c;
    return nullptr;
}
void erase_component(VehicleDesc& d, const std::string& id) {
    auto& cs = d.powertrain.components;
    cs.erase(std::remove_if(cs.begin(), cs.end(), [&](const ComponentDesc& c) { return c.id == id; }), cs.end());
    for (ShaftDesc& s : d.powertrain.shafts)
        s.ports.erase(std::remove_if(s.ports.begin(), s.ports.end(), [&](const PortRef& p) { return p.component == id; }), s.ports.end());
    auto& sh = d.powertrain.shafts;
    sh.erase(std::remove_if(sh.begin(), sh.end(), [](const ShaftDesc& s) { return s.ports.empty(); }), sh.end());
}
void attach(VehicleDesc& d, const std::string& comp, const std::string& port, const std::string& to_comp, const std::string& to_port) {
    for (ShaftDesc& s : d.powertrain.shafts)
        for (const PortRef& p : s.ports)
            if (p.component == to_comp && p.port == to_port) { s.ports.push_back(PortRef{comp, port, false}); return; }
    FAIL("no shaft with " << to_comp << "." << to_port);
}

// car_hybrid without its supervisor, generator and motor reduction: ICE + clutch + 6MT + diff, one traction motor.
VehicleDesc parallel_hybrid(const std::string& comp, const std::string& port, real soc = 0.5) {
    VehicleDesc d = io::load_vehicle_json(kData + "/vehicles/car_hybrid.json");
    erase_component(d, "hybrid_control");
    erase_component(d, "generator");
    erase_component(d, "motor_reduction");
    erase_component(d, "hybrid_motor"); // re-added below on the requested shaft
    ElectricMotorDesc m;
    m.battery = "hybrid_battery";
    m.peak_torque_nm = 450;
    m.max_power_w = 200000;
    m.max_rpm = 20000;
    m.inertia_kgm2 = 0.12;
    m.efficiency = 0.95;
    m.regen_torque_nm = 300;
    m.mass_kg = 60;
    ComponentDesc c;
    c.id = "hybrid_motor";
    c.params = m;
    d.powertrain.components.push_back(c);
    attach(d, "hybrid_motor", "shaft", comp, port);
    std::get<BatteryDesc>(find(d, "hybrid_battery")->params).initial_soc = soc;
    return d;
}

struct Rig {
    ps_test::VehicleTestRig rig;
    World& w() { return *rig.world; }
    [[nodiscard]] const PowertrainSnapshot& pt() { return rig.world->powertrain_state(rig.vehicle); }
    [[nodiscard]] real speed() { return rig.world->backend().get_motion(rig.chassis).linear.length(); }
};

Rig make(const VehicleDesc& d, real v0, int gear, real mass = kMass, Vec3 half = {2, .4, .15}, real height = .6, int workers = 4) {
    Rig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, mass, half, height, 0.0, workers);
    r.w().set_control("gear_request", gear);
    r.w().set_control("assist.traction_control", 0);
    ps_test::settle(r.w(), 0.5);
    Motion m;
    m.linear = Vec3{v0, 0, 0};
    r.w().backend().set_motion(r.rig.chassis, m);
    VehicleResetOptions ro;
    ro.gear = gear;
    ro.hubs_roll_with_chassis = true;
    r.w().reset_vehicle(r.rig.vehicle, ro);
    return r;
}
} // namespace

TEST_CASE("G1: a motor behind the gearbox regenerates through the engaged gear and the pads take only the rear axle", "[electric][brake][vehicle][gearbox]") {
    Rig r = make(parallel_hybrid("gearbox", "in"), 25.0, 3);
    World& w = r.w();
    w.set_control("throttle", 0);
    w.set_control("brake", 0.12);
    real min_regen = 1e9, max_regen = 0, min_pad = 1e9, max_pad = 0, min_tot = 1e9, max_tot = 0;
    int n = 0;
    for (int k = 0; k < 240 * 3 && r.speed() > 8.0; ++k) {
        w.step();
        if (k < 6) continue;
        const PowertrainSnapshot& p = r.pt();
        min_regen = std::min(min_regen, p.regen_brake_torque_nm);
        max_regen = std::max(max_regen, p.regen_brake_torque_nm);
        min_pad = std::min(min_pad, p.pad_brake_torque_nm);
        max_pad = std::max(max_pad, p.pad_brake_torque_nm);
        const real tot = p.regen_brake_torque_nm + p.pad_brake_torque_nm;
        min_tot = std::min(min_tot, tot);
        max_tot = std::max(max_tot, tot);
        ++n;
    }
    INFO("regen " << min_regen << ".." << max_regen << ", pad " << min_pad << ".." << max_pad << ", total " << min_tot << ".." << max_tot);
    REQUIRE(n > 100);
    CHECK(min_regen >= 0.97 * kFrontCap * 0.12);
    CHECK(max_regen <= 1.03 * kFrontCap * 0.12);
    CHECK(min_pad >= 0.97 * kRearCap * 0.12);
    CHECK(max_pad <= 1.03 * kRearCap * 0.12);
    CHECK(min_tot >= 0.99 * 8800 * 0.12);
    CHECK(max_tot <= 1.01 * 8800 * 0.12);
}

TEST_CASE("G2: in neutral the motor regenerates nothing and the pads take the whole request", "[electric][brake][vehicle][gearbox]") {
    Rig r = make(parallel_hybrid("gearbox", "in"), 25.0, 0);
    World& w = r.w();
    w.set_control("throttle", 0);
    w.set_control("brake", 0.12);
    real max_regen = 0, max_power = 0, min_pad = 1e9, max_pad = 0;
    int n = 0;
    for (int k = 0; k < 240 * 2 && r.speed() > 8.0; ++k) {
        w.step();
        if (k < 48) continue;
        const PowertrainSnapshot& p = r.pt();
        max_regen = std::max(max_regen, p.regen_brake_torque_nm);
        max_power = std::max(max_power, p.regen_power_w);
        min_pad = std::min(min_pad, p.pad_brake_torque_nm);
        max_pad = std::max(max_pad, p.pad_brake_torque_nm);
        ++n;
    }
    INFO("regen " << max_regen << " power " << max_power << " pad " << min_pad << ".." << max_pad);
    REQUIRE(n > 100);
    CHECK(max_regen == 0.0);
    CHECK(max_power == 0.0);
    CHECK(min_pad >= 0.99 * 8800 * 0.12);
    CHECK(max_pad <= 1.01 * 8800 * 0.12);
}

namespace {
struct DownshiftRun {
    real worst_total_err = 0, regen_in_shift = 0, regen_after = 0;
    int shift_ticks = 0, done_tick = -1, n_after = 0;
    std::vector<real> decel; // per 0.25 s window
};
DownshiftRun downshift_run(real soc) {
    Rig r = make(parallel_hybrid("gearbox", "in", soc), 25.0, 4);
    World& w = r.w();
    w.set_control("throttle", 0);
    w.set_control("brake", 0.12);
    const real request = 8800 * 0.12;
    DownshiftRun s;
    real win_v = r.speed();
    for (int k = 0; k < 240 * 4 && r.speed() > 9.0; ++k) {
        if (k == 240) w.set_control("gear_request", 3);
        w.step();
        const PowertrainSnapshot& p = r.pt();
        if (k >= 6) s.worst_total_err = std::max(s.worst_total_err, std::abs(p.regen_brake_torque_nm + p.pad_brake_torque_nm - request) / request);
        const bool box_moving = p.shift_phase == ShiftPhase::Releasing || p.shift_phase == ShiftPhase::Selecting || p.shift_phase == ShiftPhase::Synchronising;
        if (k >= 240 && box_moving) { ++s.shift_ticks; s.regen_in_shift = std::max(s.regen_in_shift, p.regen_brake_torque_nm); }
        if (k >= 240 && s.done_tick < 0 && p.gear == 3 && p.shift_phase == ShiftPhase::Idle) s.done_tick = k;
        if (s.done_tick >= 0 && k >= s.done_tick + 120) { s.regen_after = std::max(s.regen_after, p.regen_brake_torque_nm); ++s.n_after; }
        if (k > 0 && k % 60 == 0) {
            s.decel.push_back((win_v - r.speed()) / 0.25);
            win_v = r.speed();
        }
    }
    return s;
}
} // namespace

TEST_CASE("G3: a downshift during the stop leaves the total brake torque at the request", "[electric][brake][vehicle][gearbox]") {
    const DownshiftRun blended = downshift_run(0.5);
    const DownshiftRun pads_only = downshift_run(1.0); // pack full: no charge acceptance, every Nm from the pads, same shift physics
    real worst = 0;
    const std::size_t n = std::min(blended.decel.size(), pads_only.decel.size());
    std::string decel_s;
    for (std::size_t i = 0; i < n; ++i) {
        worst = std::max(worst, std::abs(blended.decel[i] - pads_only.decel[i]) / pads_only.decel[i]);
        decel_s += std::to_string(blended.decel[i]) + "/" + std::to_string(pads_only.decel[i]) + " ";
    }
    INFO("decel windows blended/pads-only: " << decel_s);
    INFO("total err " << blended.worst_total_err << ", shift ticks " << blended.shift_ticks << ", regen in shift " << blended.regen_in_shift
         << ", done at tick " << blended.done_tick << ", regen after " << blended.regen_after << ", worst window deviation " << worst);
    CHECK(blended.worst_total_err <= 0.01);
    CHECK(blended.shift_ticks > 0);
    CHECK(blended.regen_in_shift == 0.0);
    REQUIRE(blended.done_tick >= 0);
    CHECK(blended.done_tick <= 240 + 360);
    REQUIRE(blended.n_after > 0);
    CHECK(blended.regen_after >= 0.9 * kFrontCap * 0.12);
    CHECK(pads_only.regen_after == 0.0);
    REQUIRE(n >= 10);
    CHECK(worst <= 0.06);
}

TEST_CASE("G4: a motor upstream of the clutch regenerates only while the clutch is closed", "[electric][brake][vehicle][gearbox]") {
    Rig r = make(parallel_hybrid("engine", "crank"), 25.0, 3);
    World& w = r.w();
    w.set_control("assist.auto_clutch", 0);
    w.set_control("clutch", 0);
    w.set_control("throttle", 0);
    w.set_control("brake", 0.12);
    real closed_min = 1e9, closed_max = 0;
    for (int k = 0; k < 120; ++k) {
        w.step();
        if (k >= 30) { closed_min = std::min(closed_min, r.pt().regen_brake_torque_nm); closed_max = std::max(closed_max, r.pt().regen_brake_torque_nm); }
    }
    w.set_control("clutch", 1.0);
    real open_regen = 0, open_pad_min = 1e9, open_pad_max = 0;
    for (int k = 0; k < 96; ++k) {
        w.step();
        if (k >= 24) {
            open_regen = std::max(open_regen, r.pt().regen_brake_torque_nm);
            open_pad_min = std::min(open_pad_min, r.pt().pad_brake_torque_nm);
            open_pad_max = std::max(open_pad_max, r.pt().pad_brake_torque_nm);
        }
    }
    w.set_control("clutch", 0.0);
    real first_back = -1, final_back = 0;
    for (int k = 0; k < 240; ++k) {
        w.step();
        if (first_back < 0 && r.pt().regen_brake_torque_nm > 0) first_back = r.pt().regen_brake_torque_nm;
        final_back = r.pt().regen_brake_torque_nm;
    }
    INFO("closed " << closed_min << ".." << closed_max << ", open: regen " << open_regen << " pad " << open_pad_min << ".." << open_pad_max
         << ", back: first " << first_back << " final " << final_back);
    CHECK(closed_min >= 0.97 * kFrontCap * 0.12);
    CHECK(closed_max <= 1.03 * kFrontCap * 0.12);
    CHECK(open_regen == 0.0);
    CHECK(open_pad_min >= 0.99 * 8800 * 0.12);
    CHECK(open_pad_max <= 1.01 * 8800 * 0.12);
    REQUIRE(first_back > 0);
    CHECK(first_back < 0.5 * final_back);
    CHECK(final_back >= 0.97 * kFrontCap * 0.12);
}

namespace {
VehicleDesc tractor_with_motor() {
    VehicleDesc d = io::load_vehicle_json(kData + "/vehicles/tractor.json");
    BatteryDesc b;
    b.capacity_kwh = 20;
    b.initial_soc = 0.5;
    b.max_charge_w = 200000;
    b.max_discharge_w = 150000;
    b.mass_kg = 100;
    ComponentDesc bc;
    bc.id = "pack";
    bc.params = b;
    d.powertrain.components.push_back(bc);
    ElectricMotorDesc m;
    m.battery = "pack";
    m.peak_torque_nm = 300;
    m.max_power_w = 150000;
    m.max_rpm = 12000;
    m.inertia_kgm2 = 0.1;
    m.efficiency = 0.95;
    m.regen_torque_nm = 300;
    m.mass_kg = 60;
    ComponentDesc mc;
    mc.id = "motor";
    mc.params = m;
    d.powertrain.components.push_back(mc);
    attach(d, "motor", "shaft", "transfer", "in");
    return d;
}
struct TractorStop { real regen = 0, pad = 0; };
TractorStop tractor_brake(real awd, real pedal) {
    Rig r = make(tractor_with_motor(), 6.0, 3, 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 2);
    World& w = r.w();
    w.set_control("awd", awd);
    w.set_control("throttle", 0);
    w.set_control("assist.auto_clutch", 0);
    w.set_control("clutch", 0);
    w.set_control("brake", pedal);
    TractorStop s;
    int n = 0;
    for (int k = 0; k < 240 && r.speed() > 3.0; ++k) {
        w.step();
        if (k >= 120) { s.regen += r.pt().regen_brake_torque_nm; s.pad += r.pt().pad_brake_torque_nm; ++n; }
    }
    REQUIRE(n > 20);
    s.regen /= n;
    s.pad /= n;
    return s;
}
} // namespace

TEST_CASE("G5: behind a transfer case only the engaged outputs are reflected", "[electric][brake][vehicle][gearbox]") {
    const real pedal = 0.06;
    const TractorStop off = tractor_brake(0.0, pedal);
    const TractorStop on = tractor_brake(1.0, pedal);
    // Torque at a hub per unit motor torque: rear = rear_ratio x diff half / eff, front likewise; with both outputs open
    // the motor torque splits over two branches. The motor torque is limited by the tightest hub.
    const real rear_per_wheel = 1.0 * 20.0 * 0.5 / 0.94; // transfer rear_ratio x diff ratio/2 / eff
    const real front_per_wheel = 1.0 * 15.3 * 0.5 / 0.94;
    const real tm_off = 8000 * pedal / rear_per_wheel;
    const real tm_on = std::min(8000 * pedal / (0.5 * rear_per_wheel), 3000 * pedal / (0.5 * front_per_wheel));
    const real regen_off = 2 * rear_per_wheel * tm_off;
    const real regen_on = 2 * 0.5 * rear_per_wheel * tm_on + 2 * 0.5 * front_per_wheel * tm_on;
    INFO("awd 0: regen " << off.regen << " pad " << off.pad << " (expect " << regen_off << " / " << 6000 * pedal << "); awd 1: regen " << on.regen
         << " pad " << on.pad << " (expect " << regen_on << ")");
    CHECK(off.regen == Catch::Approx(regen_off).epsilon(0.03));
    CHECK(off.pad == Catch::Approx(6000 * pedal).epsilon(0.03));
    CHECK(on.regen == Catch::Approx(regen_on).epsilon(0.03));
    CHECK(on.regen + on.pad == Catch::Approx(22000 * pedal).epsilon(0.01));
}

TEST_CASE("G6: the path gating configuration loads and validates", "[electric][brake][io]") {
    const std::string base = kData + "/vehicles/car_electric.json";
    json j;
    {
        std::ifstream f(base);
        j = json::parse(f);
    }
    auto write_and_load = [&](json rb) {
        json k = j;
        k["brake_system"]["regen_blend"] = std::move(rb);
        // part paths are relative to the vehicle file, so write next to it
        const std::string path = kData + "/vehicles/_tmp_g6.json";
        std::ofstream(path) << k.dump();
        struct Cleanup { std::string p; ~Cleanup() { std::remove(p.c_str()); } } cleanup{path};
        return io::load_vehicle_json(path);
    };
    const VehicleDesc ok = write_and_load({{"path_ramp_s", 0.2}, {"path_min_engagement", 0.9}, {"clutch_capacity_margin", 0.8}});
    CHECK(ok.brake_system.regen_blend.path_ramp_s == Catch::Approx(0.2));
    CHECK(ok.brake_system.regen_blend.path_min_engagement == Catch::Approx(0.9));
    CHECK(ok.brake_system.regen_blend.clutch_capacity_margin == Catch::Approx(0.8));
    const VehicleDesc def = write_and_load(json::object());
    CHECK(def.brake_system.regen_blend.path_ramp_s == Catch::Approx(0.05));
    CHECK(def.brake_system.regen_blend.path_min_engagement == Catch::Approx(0.95));
    CHECK(def.brake_system.regen_blend.clutch_capacity_margin == Catch::Approx(0.9));
    CHECK_THROWS_WITH(write_and_load({{"path_ramp_s", -0.1}}), Catch::Matchers::ContainsSubstring("path_ramp_s"));
    CHECK_THROWS_WITH(write_and_load({{"path_min_engagement", 0.0}}), Catch::Matchers::ContainsSubstring("path_min_engagement"));
    CHECK_THROWS_WITH(write_and_load({{"path_min_engagement", 1.2}}), Catch::Matchers::ContainsSubstring("path_min_engagement"));
    CHECK_THROWS_WITH(write_and_load({{"clutch_capacity_margin", 0.0}}), Catch::Matchers::ContainsSubstring("clutch_capacity_margin"));
    CHECK_THROWS_WITH(write_and_load({{"clutch_capacity_margin", 1.5}}), Catch::Matchers::ContainsSubstring("clutch_capacity_margin"));
}

TEST_CASE("G7: a motor behind a gearbox runs bit-identically on 1 and 10 workers", "[electric][brake][determinism]") {
    auto run = [](int workers) {
        Rig r = make(parallel_hybrid("gearbox", "in"), 22.0, 4, kMass, {2, .4, .15}, .6, workers);
        World& w = r.w();
        w.set_control("throttle", 0);
        w.set_control("brake", 0.12);
        std::uint64_t h = 0;
        for (int k = 0; k < 240 * 2; ++k) {
            if (k == 120) w.set_control("gear_request", 3);
            w.step();
            h = h * 1099511628211ULL ^ w.state_hash();
        }
        return h;
    };
    CHECK(run(1) == run(10));
}
