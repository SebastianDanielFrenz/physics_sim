// tests/unit/test_brake_blend.cpp - brake blending, regeneration first (owner requirement 2026-10-18):
//   "When a vehicle that has electric motors on the powertrain brakes, the braking should start via recuperation and
//    only when either the battery is full or the requested braking power exceeds the engine's power generation capacity
//    should the brake pads kick in. The amount of recuperation currently in progress should be exposed to the dashboard."
//
// Law under test (docs/electric_drive.md "Brake blending"):
//   * the brake pedal is a request for the total service brake torque (pedal x corner capacities, as always);
//   * a motor takes it first, up to what it can regenerate this substep: its regen torque and power limits, the pack's
//     charge acceptance (full below taper_start_soc, zero at full_soc, max_charge_w), a low-speed fade;
//   * the pads of the wheels that motor drives supply only the rest, reflected through the motor-to-rim ratio; the other
//     axle keeps its full share; the total never has a hole;
//   * ABS acting on a wheel the motor drives withdraws that motor's regeneration (sensor-only, held abs_hold_s);
//   * snapshot (never hashed): regen_power_w, regen_brake_torque_nm, pad_brake_torque_nm, regen_brake_share, pad_brake_share.
//
// BANDS COMMITTED BEFORE THE FIRST RUN (all [AS]):
//   B1  car_electric, pack at 50 %, 100 km/h, brake 0.15 (request inside the regen limits of both axles): while the car is
//       faster than 5 m/s every tick has pad share <= 0.03 and regen share >= 0.97; SOC rises; the shaft ledger closes
//       (balance_error < 1e-5); battery gain = |electric_out| x efficiency to 1e-6 relative; brake_loss (pads, incl. the
//       low-speed hand-over) <= 8 % of the regenerated mechanical energy
//   B2  same with the pack at 100 % (above full_soc): pad share >= 0.999, regen power 0 on every tick; stop distance equals
//       the same car with regen_blend.enabled = false and a full pack (pads only) to 1e-6 relative, and the half-pack stop
//       distance is within 3 % of it (the source of the torque does not change the stop)
//   B3  demand above the regen capacity (brake 0.35 from 20 m/s, 0.4 s): both axles at their regen limit (100 N*m x 9 / 0.98
//       per axle): regen torque 1837 N*m +- 3 %, pad torque = request - regen
//       (+- 1 %), regen + pad = request 3080 N*m +- 1 %; mean deceleration within 6 % of request/(r m_eff), m_eff = chassis + rotating inertia
//   B4  per-axle reflection: rear motor regen_torque_nm = 0 (rear cannot regenerate): regen 900 N*m +- 3 % (the front
//       limit), the rear pads carry their full 1120 N*m share, regen + pad = request +- 1 %
//   B5  ABS on low_mu (car_electric + the street ABS/ESC unit, 20 m/s, brake 1): regeneration is on in the first ticks
//       (> 50 % of the front request) and withdrawn while ABS acts (mean regen torque share <= 5 % over the ticks 1.0-4.0 s
//       with abs_active); no wheel stays locked for more than 0.5 s; negative control: with ABS withdrawal sabotaged the
//       same run locks (not asserted in the test, recorded in the sabotage list)
//   B6  hand-over: a 0.25 kWh pack starting at 85 % crosses its taper mid-stop (brake 0.15 from 100 km/h): the pad share
//       starts below 0.2 and ends above 0.8 while the car is still above 6 m/s, and the deceleration (0.25 s windows,
//       27 -> 6 m/s) stays within 7 % of its mean - no torque hole when the regeneration is lost
//   B7  regen_blend.enabled = false: the old parallel behaviour (snapshot shares 0; the stop is >= 5 % shorter than the
//       blended one because pads and regeneration both act)
//   B8  ICE car (car_sedan): state hash after a throttle + brake run equals the value recorded on the unchanged code, at 1
//       and 10 workers
//   B9  EV: 1 vs 10 workers equal state hash every tick through a stop
//   L1  loader: regen_blend and regen_power_w round-trip; unknown key, taper_start >= full_soc, fade_end >= fade_start,
//       non-boolean enabled, negative abs_hold_s and a negative regen_power_w are rejected with the field named

#include "ps/backend/i_rigid_backend.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_reset.h"
#include "vehicle_test_helpers.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

using namespace ps;
using namespace ps::drivetrain;
using namespace ps::vehicle;
using nlohmann::json;

namespace {
const std::string kData = std::string(PS_SOURCE_DIR) + "/data";
const std::string kEv = kData + "/vehicles/car_electric.json";
const std::string kSedan = kData + "/vehicles/car_sedan.json";
const std::string kUnit = kData + "/electronics/assist_unit_abs_esc.json";
constexpr real kV100 = 100.0 / 3.6;
constexpr real kWheelR = 0.317;
constexpr real kMass = 1850;

VehicleDesc ev(real soc = 0.5) {
    VehicleDesc d = io::load_vehicle_json(kEv);
    std::get<BatteryDesc>(d.powertrain.components[0].params).initial_soc = soc;
    return d;
}
BatteryDesc& battery(VehicleDesc& d) { return std::get<BatteryDesc>(d.powertrain.components[0].params); }
ElectricMotorDesc& rear_motor(VehicleDesc& d) { return std::get<ElectricMotorDesc>(d.powertrain.components[3].params); }

// Chassis mass plus the rotating inertia the brakes must also slow: the wheels and each motor rotor through its 9:1 reduction.
real effective_mass(const VehicleDesc& d) {
    real j = 0;
    for (const WheelDesc& w : d.wheels) j += w.wheel_inertia;
    j += std::get<ElectricMotorDesc>(d.powertrain.components[1].params).inertia_kgm2 * 81;
    j += std::get<ElectricMotorDesc>(d.powertrain.components[3].params).inertia_kgm2 * 81;
    return kMass + j / (kWheelR * kWheelR);
}

struct Rig {
    ps_test::VehicleTestRig rig;
    World& w() { return *rig.world; }
    [[nodiscard]] const PowertrainSnapshot& pt() { return rig.world->powertrain_state(rig.vehicle); }
    [[nodiscard]] real speed() { return rig.world->backend().get_motion(rig.chassis).linear.length(); }
    [[nodiscard]] real x() { return rig.world->backend().get_pose(rig.chassis).position.x; }
};

void roll(Rig& r, real v, int gear) {
    Motion m;
    m.linear = Vec3{v, 0, 0};
    r.w().backend().set_motion(r.rig.chassis, m);
    VehicleResetOptions ro;
    ro.gear = gear;
    ro.hubs_roll_with_chassis = true;
    r.w().reset_vehicle(r.rig.vehicle, ro);
}

Rig make_ev(const VehicleDesc& d, real v0, int workers = 4) {
    Rig r;
    r.rig = ps_test::build_vehicle_rig_from_desc(d, kMass, {2, .4, .15}, .6, 0.0, workers);
    r.w().set_control("gear_request", 1);
    r.w().set_control("assist.traction_control", 0);
    ps_test::settle(r.w(), 0.5);
    roll(r, v0, VehicleResetOptions::kKeepGear);
    return r;
}

struct Stop {
    real distance = 0, t = 0, v_end = 0;
    real max_pad_share_fast = 0, min_regen_share_fast = 1; // while faster than 5 m/s, request > 0
    real max_regen_power = 0, max_blend_torque = 0;
    real soc0 = 0, soc1 = 0;
    PowertrainEnergy energy{};
    real max_balance = 0;
    std::vector<real> speed_t, pad_share_t; // per tick
};

Stop run_stop(const VehicleDesc& d, real v0, real brake, real max_s, int workers = 4) {
    Rig r = make_ev(d, v0, workers);
    Stop s;
    World& w = r.w();
    const real x0 = r.x();
    s.soc0 = r.pt().battery_soc;
    w.set_control("throttle", 0);
    w.set_control("brake", brake);
    const int n = static_cast<int>(max_s * 240);
    for (int k = 0; k < n; ++k) {
        w.step();
        const real v = r.speed();
        const PowertrainSnapshot& p = r.pt();
        s.speed_t.push_back(v);
        s.pad_share_t.push_back(p.pad_brake_share);
        s.max_regen_power = std::max(s.max_regen_power, p.regen_power_w);
        s.max_blend_torque = std::max(s.max_blend_torque, p.regen_brake_torque_nm + p.pad_brake_torque_nm);
        if (k >= 3) s.max_balance = std::max(s.max_balance, std::abs(w.powertrain_energy(r.rig.vehicle).balance_error)); // k < 3: the reset's own ledger term
        if (v > 5.0) {
            s.max_pad_share_fast = std::max(s.max_pad_share_fast, p.pad_brake_share);
            s.min_regen_share_fast = std::min(s.min_regen_share_fast, p.regen_brake_share);
        }
        s.t = (k + 1) / 240.0;
        if (v < 0.3) break;
    }
    s.distance = r.x() - x0;
    s.v_end = r.speed();
    s.soc1 = r.pt().battery_soc;
    s.energy = w.powertrain_energy(r.rig.vehicle);
    return s;
}
} // namespace

TEST_CASE("B1: braking from 100 km/h with a half-full pack is regeneration only above walking pace", "[electric][brake][vehicle]") {
    const Stop s = run_stop(ev(0.5), kV100, 0.15, 25.0);
    INFO("distance " << s.distance << " m in " << s.t << " s, v_end " << s.v_end << ", max pad share " << s.max_pad_share_fast
                     << ", min regen share " << s.min_regen_share_fast << ", brake_loss " << s.energy.brake_loss
                     << ", electric_out " << s.energy.electric_out << ", battery_draw " << s.energy.battery_draw);
    REQUIRE(s.v_end < 0.5);
    CHECK(s.max_pad_share_fast <= 0.03);
    CHECK(s.min_regen_share_fast >= 0.97);
    CHECK(s.max_regen_power > 20000);
    CHECK(s.soc1 > s.soc0);
    CHECK(s.max_balance < 1e-5);
    const real regen_mech = -s.energy.electric_out;
    REQUIRE(regen_mech > 0);
    CHECK(-s.energy.battery_draw == Catch::Approx(regen_mech * 0.95).epsilon(1e-6));
    CHECK(s.energy.brake_loss <= 0.08 * regen_mech);
}

TEST_CASE("B2: with a full pack the pads carry the whole request and the stop equals the pad-only stop", "[electric][brake][vehicle]") {
    const Stop full = run_stop(ev(1.0), kV100, 0.15, 25.0);
    VehicleDesc off = ev(1.0);
    off.brake_system.regen_blend.enabled = false;
    const Stop pads_only = run_stop(off, kV100, 0.15, 25.0);
    const Stop half = run_stop(ev(0.5), kV100, 0.15, 25.0);
    INFO("full " << full.distance << " m, pads-only " << pads_only.distance << " m, half pack " << half.distance << " m");
    REQUIRE(full.v_end < 0.5);
    CHECK(full.max_regen_power == 0);
    real min_pad = 1;
    for (std::size_t i = 0; i < full.speed_t.size(); ++i) if (full.speed_t[i] > 5.0) min_pad = std::min(min_pad, full.pad_share_t[i]);
    CHECK(min_pad >= 0.999);
    CHECK(full.distance == Catch::Approx(pads_only.distance).epsilon(1e-6));
    CHECK(half.distance == Catch::Approx(full.distance).epsilon(0.03));
}

namespace {
struct Window {
    real regen = 0, pad = 0, decel = 0;
};
// Applies `brake` at v0 and averages the snapshot over `window_s` after a 0.1 s settle.
Window measure_window(const VehicleDesc& d, real v0, real brake, real window_s) {
    Rig r = make_ev(d, v0);
    World& w = r.w();
    w.set_control("throttle", 0);
    w.set_control("brake", brake);
    for (int k = 0; k < 24; ++k) w.step();
    const real va = r.speed();
    Window out;
    const int n = static_cast<int>(window_s * 240);
    for (int k = 0; k < n; ++k) {
        w.step();
        out.regen += r.pt().regen_brake_torque_nm / n;
        out.pad += r.pt().pad_brake_torque_nm / n;
    }
    out.decel = (va - r.speed()) / window_s;
    return out;
}
} // namespace

TEST_CASE("B3: a request above the regen capacity is regeneration at its limit plus exactly the rest from the pads", "[electric][brake][vehicle]") {
    const real brake = 0.35;
    const real request = brake * (2 * 2800.0 + 2 * 1600.0); // 3080
    const Window m = measure_window(ev(0.5), 20.0, brake, 0.4);
    INFO("regen " << m.regen << " pad " << m.pad << " request " << request << " decel " << m.decel);
    CHECK(m.regen == Catch::Approx(2 * 100.0 * 9 / 0.98).epsilon(0.03)); // both axles at their 100 N*m motor limit, 9:1 and the 0.98 mesh
    CHECK(m.pad == Catch::Approx(request - m.regen).epsilon(0.01));
    CHECK(m.regen + m.pad == Catch::Approx(request).epsilon(0.01));
    CHECK(m.decel == Catch::Approx(request / kWheelR / effective_mass(ev(0.5))).epsilon(0.06));
}

TEST_CASE("B4: a motor reduces only the pads of its own axle", "[electric][brake][vehicle]") {
    const real brake = 0.35;
    VehicleDesc d = ev(0.5);
    rear_motor(d).regen_torque_nm = 0;
    const Window m = measure_window(d, 20.0, brake, 0.4);
    const real request = brake * (2 * 2800.0 + 2 * 1600.0);
    INFO("regen " << m.regen << " pad " << m.pad);
    CHECK(m.regen == Catch::Approx(900.0).epsilon(0.03));
    CHECK(m.pad >= 2 * brake * 1600.0 * 0.99); // the rear pads keep their whole share
    CHECK(m.regen + m.pad == Catch::Approx(request).epsilon(0.01));
}

TEST_CASE("B6: when the pack stops accepting charge mid-stop the pads take over without a torque hole", "[electric][brake][vehicle]") {
    VehicleDesc d = ev(0.85);
    battery(d).capacity_kwh = 0.25;
    Rig r = make_ev(d, kV100);
    World& w = r.w();
    w.set_control("throttle", 0);
    w.set_control("brake", 0.15);
    real pad_start = -1, pad_end = -1;
    std::vector<real> decels;
    real window_v0 = r.speed();
    int in_window = 0;
    for (int k = 0; k < 240 * 25 && r.speed() > 0.5; ++k) {
        w.step();
        const real v = r.speed();
        if (pad_start < 0 && k >= 5) pad_start = r.pt().pad_brake_share;
        if (v > 6.0) pad_end = r.pt().pad_brake_share;
        if (++in_window == 60) {
            if (v > 6.0 && window_v0 < 27.0) decels.push_back((window_v0 - v) / 0.25);
            window_v0 = v;
            in_window = 0;
        }
    }
    REQUIRE(decels.size() > 10);
    real mean = 0;
    for (real a : decels) mean += a / decels.size();
    real worst = 0;
    for (real a : decels) worst = std::max(worst, std::abs(a - mean) / mean);
    INFO("pad share start " << pad_start << " end " << pad_end << ", mean decel " << mean << ", worst deviation " << worst);
    CHECK(pad_start < 0.2);
    CHECK(pad_end > 0.8);
    CHECK(worst < 0.07);
}

TEST_CASE("B7: regen_blend disabled keeps the old parallel behaviour", "[electric][brake][vehicle]") {
    VehicleDesc off = ev(0.5);
    off.brake_system.regen_blend.enabled = false;
    const Stop a = run_stop(off, kV100, 0.15, 25.0);
    const Stop b = run_stop(ev(0.5), kV100, 0.15, 25.0);
    INFO("parallel " << a.distance << " m, blended " << b.distance << " m");
    CHECK(a.max_blend_torque == 0); // the blend snapshot stays 0
    CHECK(a.max_regen_power > 20000); // parallel regeneration still happens
    CHECK(a.distance < 0.95 * b.distance);
}

TEST_CASE("B9: an EV stop is bit-identical across worker counts", "[electric][brake][determinism]") {
    const auto run = [](int workers) {
        Rig r = make_ev(ev(0.5), kV100, workers);
        r.w().set_control("brake", 0.3);
        std::vector<std::uint64_t> h;
        for (int i = 0; i < 480; ++i) { r.w().step(); h.push_back(r.w().state_hash()); }
        return h;
    };
    CHECK(run(1) == run(10));
}

TEST_CASE("B8: an ICE car is unchanged", "[electric][brake][determinism]") {
    const auto run = [](int workers) {
        auto rig = ps_test::build_vehicle_rig(kSedan, 1500, {2, .4, .15}, .6, 0.0, workers);
        World& w = *rig.world;
        w.set_control("gear_request", 1);
        w.set_control("throttle", 0.6);
        for (int i = 0; i < 480; ++i) w.step();
        w.set_control("throttle", 0);
        w.set_control("brake", 0.5);
        for (int i = 0; i < 480; ++i) w.step();
        return w.state_hash();
    };
    const std::uint64_t h1 = run(1), h10 = run(10);
    std::printf("B8 hash %016llx\n", static_cast<unsigned long long>(h1));
    CHECK(h1 == h10);
    CHECK(h1 == 0xc6c5faed6f257f1fULL); // recorded on the unchanged code (before any brake-blending edit)
}

namespace {
Rig make_low_mu(const VehicleDesc& d, real v0) {
    Rig r;
    r.rig.surfaces = std::make_shared<io::SurfaceTable>(kData + "/surfaces/surfaces.json");
    ps::WorldConfig cfg;
    cfg.job_workers = 4;
    r.rig.world = std::make_unique<World>(cfg);
    BodyDesc ground;
    ground.shape = ps_test::flat_surface_mesh(1000.0, r.rig.surfaces->id_for("low_mu"));
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    r.w().create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2, .4, .15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = kMass;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0, 0, 0.6};
    r.rig.chassis = r.w().create_body(chassis);
    r.w().set_surface_table(r.rig.surfaces);
    r.rig.vehicle = r.w().create_vehicle(d, r.rig.chassis);
    r.w().set_control("gear_request", 1);
    r.w().set_control("assist.traction_control", 0);
    r.w().set_control("assist.abs", 1);
    r.w().set_control("assist.esc", 0);
    ps_test::settle(r.w(), 0.5);
    roll(r, v0, VehicleResetOptions::kKeepGear);
    return r;
}
} // namespace

TEST_CASE("B5: ABS withdraws the regeneration of the axle it acts on", "[electric][brake][abs][vehicle]") {
    io::VehicleLoadOptions o;
    o.electronics["assists"] = kUnit;
    VehicleDesc d = io::load_vehicle_json(kEv, o);
    std::get<BatteryDesc>(d.powertrain.components[0].params).initial_soc = 0.5;
    Rig r = make_low_mu(d, 20.0);
    World& w = r.w();
    w.set_control("throttle", 0);
    w.set_control("brake", 1.0);
    real early_regen = 0, sum_regen = 0, sum_total = 0, lock_run = 0, longest_lock = 0;
    for (int k = 0; k < 240 * 4 && r.speed() > 0.5; ++k) {
        w.step();
        const PowertrainSnapshot& p = r.pt();
        if (k < 3) early_regen = std::max(early_regen, p.regen_brake_torque_nm);
        const real t = (k + 1) / 240.0;
        if (t >= 1.0 && p.abs_active) {
            sum_regen += p.regen_brake_torque_nm;
            sum_total += p.regen_brake_torque_nm + p.pad_brake_torque_nm;
        }
        bool locked = false;
        for (std::size_t i = 0; i < d.wheels.size(); ++i) locked = locked || std::abs(w.wheel_telemetry(r.rig.vehicle, i).slip_ratio) > 0.5;
        if (t >= 1.0) { lock_run = locked ? lock_run + 1.0 / 240 : 0; longest_lock = std::max(longest_lock, lock_run); }
    }
    INFO("early regen " << early_regen << " N*m, regen share while ABS acts " << (sum_total > 0 ? sum_regen / sum_total : -1) << ", longest lock " << longest_lock);
    CHECK(early_regen > 0.5 * 2 * 900.0 * 0.5);
    REQUIRE(sum_total > 0);
    CHECK(sum_regen / sum_total <= 0.05);
    CHECK(longest_lock <= 0.5);
}

TEST_CASE("L1: regen_blend loads and validates", "[electric][brake][io]") {
    std::ifstream f(kEv);
    json base = json::parse(f);
    // the file's relative references (tyres) are resolved against the file's own directory: write next to the original
    const auto load_with = [&](json blend, json motor_patch = json()) {
        json j = base;
        j["brake_system"]["regen_blend"] = std::move(blend);
        if (!motor_patch.is_null()) for (auto& c : j["powertrain"]["components"]) if (c["id"] == "front_motor") c.update(motor_patch);
        const std::string path = kData + "/vehicles/_tmp_brake_blend_l1.json";
        { std::ofstream o(path); o << j.dump(2); }
        struct Cleanup { std::string p; ~Cleanup() { std::remove(p.c_str()); } } cleanup{path};
        return io::load_vehicle_json(path);
    };
    const VehicleDesc ok = load_with({{"enabled", true}, {"taper_start_soc", 0.8}, {"full_soc", 0.95}, {"fade_start_speed_mps", 4.0},
                                      {"fade_end_speed_mps", 1.0}, {"abs_hold_s", 0.5}},
                                     {{"regen_power_w", 120000.0}});
    CHECK(ok.brake_system.regen_blend.taper_start_soc == 0.8);
    CHECK(ok.brake_system.regen_blend.full_soc == 0.95);
    CHECK(ok.brake_system.regen_blend.fade_start_speed_mps == 4.0);
    CHECK(ok.brake_system.regen_blend.abs_hold_s == 0.5);
    CHECK(std::get<ElectricMotorDesc>(ok.powertrain.components[1].params).regen_power_w == 120000.0);
    CHECK_THROWS_WITH(load_with({{"bogus", 1}}), Catch::Matchers::ContainsSubstring("regen_blend: unknown key 'bogus'"));
    CHECK_THROWS_WITH(load_with({{"taper_start_soc", 0.99}}), Catch::Matchers::ContainsSubstring("taper_start_soc"));
    CHECK_THROWS_WITH(load_with({{"fade_end_speed_mps", 9.0}}), Catch::Matchers::ContainsSubstring("fade_end_speed_mps"));
    CHECK_THROWS_WITH(load_with({{"enabled", 1}}), Catch::Matchers::ContainsSubstring("regen_blend.enabled must be a boolean"));
    CHECK_THROWS_WITH(load_with({{"abs_hold_s", -1.0}}), Catch::Matchers::ContainsSubstring("abs_hold_s"));
    CHECK_THROWS_WITH(load_with(json::object(), {{"regen_power_w", -5.0}}), Catch::Matchers::ContainsSubstring("regen"));
}
