// tests/unit/test_machine_coupling.cpp — two vehicles coupled at a hitch: an
// implicit socket spring-damper solved per substep with both shadow chassis
// stepped together in one island (core/src/machine/coupling_solver.h,
// vehicle::System::step_island). Commit 3a27366 holds the comparison against
// Jolt constraints this design rests on.
//
// The hidden [.][diag] cases print measurement tables; every number is measured
// on Jolt's own poses at the tick boundary. The normal cases pin the behaviour.

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/hitch_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/machine/coupling.h"
#include "ps/machine/hitch_limits.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"
#include "vehicle_test_helpers.h"

#include "machine/coupling_solver.h"

#include <nlohmann/json.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real k100KmH = 100.0 / 3.6;

enum class Method { None, Spring };

// A class with every rotation free: the bare socket.
machine::HitchClassDesc free_hitch() {
    machine::HitchClassDesc d;
    d.name = "free test hitch";
    d.source = "test fixture";
    d.frequency_hz = 60.0;
    d.latch_time_s = 0.5;
    d.max_latch_error_m = 0.1;
    d.max_latch_angle_rad = 0.1;
    d.max_latch_speed_m_s = 0.5;
    return d;
}

machine::HitchClassDesc load_class(const char* name) {
    return io::load_hitch_class_json(std::string(PS_SOURCE_DIR) + "/data/hitch_classes/" + name + ".json");
}

struct Cfg {
    Method method = Method::Spring;
    machine::HitchClassDesc hitch = free_hitch();
    real frequency_hz = 60.0; // over the class's own value
    real damping_ratio = 1.0;
    int passes = 2;
    real tick_hz = 240.0, substep_hz = 960.0;
    int workers = 4;
    int collision_steps = 1;
    real trailer_mass = 1500.0;
    bool trailer_brakes = false;
    bool tractor = false;
    bool coast_shell = false; // both vehicles lossless: no powertrain, no rolling resistance, no drag, a box ground
    bool drawbar_overlap = false; // the trailer's drawbar runs under the towing car, an extra axle stands right below it
    std::optional<machine::TowBrakeDesc> tow_brake;
    std::string scope_a, scope_b; // channel scopes
    bool tipper = false;          // tractor and tipper from their files, coupled at the points they name
    bool trailer_first = false;   // create the trailer before the towing vehicle
};

// An engine-less wheeled trailer, built in code on the car's own tyre and wheel data. A 1500 kg single axle or a
// tandem above 3 t (tyre fz0 x4: a truck-size tyre of the same shape). Suspension sized for 60 mm static compression.
struct TrailerGeometry {
    vehicle::VehicleDesc desc;
    Vec3 half_extents;
    Vec3 socket;
    real origin_height = 0.75;
};

TrailerGeometry make_trailer(const vehicle::VehicleDesc& car, real mass, bool brakes) {
    const bool tandem = mass > 3000.0;
    TrailerGeometry g;
    g.half_extents = tandem ? Vec3{3.0, 0.5, 0.2} : Vec3{1.4, 0.4, 0.15};
    const real socket_x = tandem ? 4.5 : 3.0;
    const real axle_mean = tandem ? -0.15 : -0.25; // behind the centre of mass: a downward tongue load
    g.socket = Vec3{socket_x, 0.0, -0.37};

    const std::vector<real> axles = tandem ? std::vector<real>{-0.65, 0.35} : std::vector<real>{axle_mean};
    const real axle_load = mass * 9.81 * socket_x / (socket_x - axle_mean);
    const real wheel_load = axle_load / static_cast<real>(2 * axles.size());
    const real k = wheel_load / 0.06;
    const real c = 0.4 * std::sqrt(k * wheel_load / 9.81);

    g.desc.name = "trailer";
    g.desc.source = "spike A0 fixture";
    int axle_index = 0;
    for (const real x : axles) {
        for (const real side : {1.0, -1.0}) {
            vehicle::WheelDesc w = car.wheels[2];
            w.name = (side > 0 ? "L" : "R") + std::to_string(axle_index);
            w.attachment_local = Vec3{x, side * (tandem ? 0.95 : 0.85), -0.493};
            w.steered = false;
            w.is_front = false;
            w.arb_partner = -1;
            w.suspension.spring_rate = k;
            w.suspension.damper_bump_rate = c;
            w.suspension.damper_rebound_rate = 1.5 * c;
            w.suspension.arb_rate = 0.0;
            if (tandem) w.tyre.fz0 *= 4.0;
            g.desc.wheels.push_back(w);
            if (brakes) g.desc.brake_system.corners.push_back({w.name, tandem ? 3000.0 : 1500.0});
        }
        ++axle_index;
    }
    return g;
}

void make_lossless(vehicle::VehicleDesc& d) {
    d.powertrain = drivetrain::PowertrainDesc{};
    d.controllers.clear();
    d.aero.drag_area_m2 = 0.0;
    d.brake_system = drivetrain::BrakeSystemDesc{};
    for (auto& w : d.wheels) {
        w.tyre.qsy1 = w.tyre.qsy2 = w.tyre.qsy3 = w.tyre.qsy4 = 0.0;
    }
}

class SpeedHold final : public IForceElement {
public:
    SpeedHold(Frame frame, real target, real gain, real limit) : frame_(frame), target_(target), gain_(gain), limit_(limit) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        const Vec3 forward = ctx.frame_pose.transform_direction(Vec3::unit_x());
        const real speed = dot(ctx.frame_motion.linear, forward);
        const real f = std::clamp(gain_ * (target_ - speed), -limit_, limit_);
        return Wrench{forward * f, Vec3::zero()};
    }

private:
    Frame frame_;
    real target_, gain_, limit_;
};

class RearwardForce final : public IForceElement {
public:
    RearwardForce(Frame frame, real magnitude) : frame_(frame), magnitude_(magnitude) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        return Wrench{ctx.frame_pose.transform_direction(Vec3::unit_x()) * -magnitude_, Vec3::zero()};
    }

private:
    Frame frame_;
    real magnitude_;
};

struct Rig {
    std::unique_ptr<World> world;
    Cfg cfg;
    BodyId a, b;
    VehicleId va, vb;
    HitchId coupling{};
    Vec3 socket_a, socket_b;
    real mass_a = 0.0, mass_b = 0.0;
    std::vector<std::unique_ptr<IForceElement>> elements;
    // A second trailer behind the first.
    bool chain = false;
    BodyId c;
    VehicleId vc;
    HitchId coupling2{};
    Vec3 socket_b_rear, socket_c;
};

const vehicle::VehicleDesc& car_desc() {
    static const vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
    return d;
}

const vehicle::VehicleDesc& tractor_desc() {
    static const vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    return d;
}

const vehicle::VehicleDesc& tipper_desc() {
    static const vehicle::VehicleDesc d = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/trailer_tipper.json");
    return d;
}

machine::CouplingDesc link_desc(const Rig& r) {
    machine::CouplingDesc d;
    d.towing = r.va;
    d.towed = r.vb;
    d.socket_towing_local = r.socket_a;
    d.socket_towed_local = r.socket_b;
    d.hitch = r.cfg.hitch;
    d.hitch.frequency_hz = r.cfg.frequency_hz;
    d.hitch.damping_ratio = r.cfg.damping_ratio;
    return d;
}

void add_link(Rig& r) {
    if (r.cfg.method != Method::Spring) return;
    r.coupling = r.cfg.tipper ? r.world->attach(r.va, "socket_hitch", r.vb, "socket_hitch_eye") : r.world->attach(link_desc(r));
}

// Towing vehicle at (x0, 0) facing +x, trailer behind it with the two sockets level and coincident.
Rig build_rig(const Cfg& cfg, real x0 = 0.0) {
    Rig r;
    r.cfg = cfg;
    WorldConfig wc;
    wc.tick_rate_hz = cfg.tick_hz;
    wc.substep_rate_hz = cfg.substep_hz;
    wc.job_workers = cfg.workers;
    wc.collision_steps = cfg.collision_steps;
    wc.coupling_passes = cfg.passes;
    r.world = std::make_unique<World>(wc);

    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    r.world->set_surface_table(surfaces);

    BodyDesc ground;
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    if (cfg.coast_shell) {
        // A box ground reports no surface, so the table's rolling resistance stays out of the energy budget.
        ground.shape = BoxShape{Vec3{1500.0, 1500.0, 0.5}};
        ground.pose.position = Vec3{0.0, 0.0, -0.5};
        r.world->create_body(ground);
    } else {
        static const ShapeHandle mesh = r.world->create_shape(ps_test::flat_surface_mesh(1000.0, surfaces->id_for("asphalt")));
        r.world->create_body(ground, mesh);
    }

    vehicle::VehicleDesc towing = cfg.tractor ? tractor_desc() : car_desc();
    TrailerGeometry trailer = make_trailer(car_desc(), cfg.trailer_mass, cfg.trailer_brakes);
    if (cfg.coast_shell) {
        make_lossless(towing);
        make_lossless(trailer.desc);
    }
    if (cfg.drawbar_overlap) {
        // 1.2 m past the socket, into the car's box, and an axle 0.9 m past it under the car's own rear.
        trailer.half_extents.x = trailer.socket.x + 1.2;
        for (const real side : {1.0, -1.0}) {
            vehicle::WheelDesc w = trailer.desc.wheels[0];
            w.name = side > 0 ? "jockey_L" : "jockey_R";
            w.attachment_local = Vec3{trailer.socket.x + 0.9, side * 0.2, -0.493};
            trailer.desc.wheels.push_back(w);
        }
    }

    r.mass_a = cfg.tractor ? 6000.0 : 1500.0;
    r.mass_b = cfg.trailer_mass;
    r.socket_a = cfg.tractor ? Vec3{-2.2, 0.0, -0.78} : Vec3{-2.49, 0.0, -0.22};
    r.socket_b = trailer.socket;
    const real height_a = cfg.tractor ? 1.28 : 0.6;
    r.socket_b.z = height_a + r.socket_a.z - trailer.origin_height;
    if (cfg.tipper) {
        trailer.desc = tipper_desc();
        trailer.half_extents = Vec3{2.5, 0.5, 0.3};
        trailer.origin_height = 1.1;
        r.socket_a = machine::find_hitch_point(towing.hitch_points, "socket_hitch")->position_local;
        r.socket_b = machine::find_hitch_point(trailer.desc.hitch_points, "socket_hitch_eye")->position_local;
    }

    BodyDesc a;
    a.shape = BoxShape{cfg.tractor ? Vec3{1.2, 0.35, 0.9} : Vec3{2.0, 0.4, 0.15}};
    a.mass = r.mass_a;
    a.friction = 0.5;
    a.pose.position = Vec3{x0, 0.0, height_a};
    r.a = r.world->create_body(a);

    BodyDesc b;
    b.shape = BoxShape{trailer.half_extents};
    b.mass = r.mass_b;
    b.friction = 0.5;
    b.pose.position = Vec3{x0 + r.socket_a.x - r.socket_b.x, 0.0, trailer.origin_height};
    r.b = r.world->create_body(b);

    if (!cfg.tipper) trailer.desc.tow_brake = cfg.tow_brake;
    if (cfg.trailer_first) r.vb = r.world->create_vehicle(trailer.desc, r.b, {}, cfg.scope_b);
    r.va = r.world->create_vehicle(towing, r.a, {}, cfg.scope_a);
    if (!cfg.trailer_first) r.vb = r.world->create_vehicle(trailer.desc, r.b, {}, cfg.scope_b);
    add_link(r);

    for (const char* name : {"assist.auto_clutch", "assist.auto_blip", "assist.auto_shift"}) r.world->set_control(r.world->vehicle_channel(r.va, name), 0.0);
    if (!cfg.tractor) r.world->set_control(r.world->vehicle_channel(r.va, "clutch"), 1.0);
    return r;
}

Vec3 hitch_error(const Rig& r) {
    return r.world->get_pose(r.a).transform_point(r.socket_a) - r.world->get_pose(r.b).transform_point(r.socket_b);
}

Vec3 hitch_error2(const Rig& r) {
    return r.world->get_pose(r.b).transform_point(r.socket_b_rear) - r.world->get_pose(r.c).transform_point(r.socket_c);
}

// build_rig plus a second trailer of the same mass, hung 2 m behind the first one's centre of mass.
Rig build_chain(const Cfg& cfg, real x0 = 0.0) {
    Rig r = build_rig(cfg, x0);
    TrailerGeometry second = make_trailer(car_desc(), cfg.trailer_mass, cfg.trailer_brakes);
    second.desc.tow_brake = cfg.tow_brake;
    const real rear_x = -second.half_extents.x - 0.2;
    r.chain = true;
    r.socket_b_rear = Vec3{rear_x, 0.0, r.socket_b.z};
    r.socket_c = Vec3{second.socket.x, 0.0, r.socket_b.z};
    BodyDesc c;
    c.shape = BoxShape{second.half_extents};
    c.mass = cfg.trailer_mass;
    c.friction = 0.5;
    c.pose.position = r.world->get_pose(r.b).position + r.socket_b_rear - r.socket_c;
    r.c = r.world->create_body(c);
    r.vc = r.world->create_vehicle(second.desc, r.c);
    machine::CouplingDesc d;
    d.towing = r.vb;
    d.towed = r.vc;
    d.socket_towing_local = r.socket_b_rear;
    d.socket_towed_local = r.socket_c;
    d.hitch = cfg.hitch;
    d.hitch.frequency_hz = cfg.frequency_hz;
    d.hitch.damping_ratio = cfg.damping_ratio;
    r.coupling2 = r.world->attach(d);
    return r;
}

real articulation_deg(const Rig& r) {
    const Vec3 fa = r.world->get_pose(r.a).transform_direction(Vec3::unit_x());
    const Vec3 fb = r.world->get_pose(r.b).transform_direction(Vec3::unit_x());
    return std::atan2(fa.x * fb.y - fa.y * fb.x, fa.x * fb.x + fa.y * fb.y) * 180.0 / kPi;
}

struct Metrics {
    real err_max_mm = 0.0, err_mean_mm = 0.0, err_final_mm = 0.0;
    real err2_max_mm = 0.0;  // the second link of a chain
    real ring_err_mm = 0.0;  // period-2 amplitude of the hitch error, 1/4 |e[k] - 2 e[k-1] + e[k-2]|
    real ring_accel = 0.0;   // the same on the trailer's tick-to-tick acceleration, m/s^2
    real dv_a = 0.0, dv_b = 0.0; // worst shadow_dv_error
    real force_peak_n = 0.0;     // Spring only
    real artic_max_deg = 0.0;
    real artic_end_deg = 0.0;    // lane change: left at the end of the run (sway decays or grows)
    real angle_max_deg = 0.0;    // the largest Cardan angle the hitch reports
    real roll_max_deg = 0.0;
    real ay_peak = 0.0;          // towing vehicle's v*yaw rate
    real end_speed = 0.0;
    real seconds = 0.0;
    std::size_t n = 0;
    bool finite = true;
};

class Recorder {
public:
    explicit Recorder(const Rig& r) : r_(r), dt_(r.world->dt()) {}

    void sample(bool shadow_valid) {
        const Vec3 e = hitch_error(r_);
        const Motion ma = r_.world->get_motion(r_.a);
        const Motion mb = r_.world->get_motion(r_.b);
        const real err = e.length();
        if (!std::isfinite(err) || err > 50.0 || !std::isfinite(mb.linear.length())) {
            m_.finite = false;
            return;
        }
        m_.err_max_mm = std::max(m_.err_max_mm, err * 1e3);
        if (r_.chain) m_.err2_max_mm = std::max(m_.err2_max_mm, hitch_error2(r_).length() * 1e3);
        sum_ += err * 1e3;
        ++m_.n;
        m_.err_mean_mm = sum_ / static_cast<real>(m_.n);
        m_.err_final_mm = err * 1e3;
        const Vec3 acc = (mb.linear - v1_) / dt_;
        if (count_ >= 2) m_.ring_err_mm = std::max(m_.ring_err_mm, (e - e1_ * 2.0 + e2_).length() * 1e3 / 4.0);
        if (count_ >= 3) m_.ring_accel = std::max(m_.ring_accel, (acc - a1_ * 2.0 + a2_).length() / 4.0);
        e2_ = e1_; e1_ = e;
        a2_ = a1_; a1_ = acc;
        v1_ = mb.linear;
        ++count_;
        if (shadow_valid) {
            m_.dv_a = std::max(m_.dv_a, r_.world->vehicle_shadow_dv_error(r_.va));
            m_.dv_b = std::max(m_.dv_b, r_.world->vehicle_shadow_dv_error(r_.vb));
        }
        if (r_.cfg.method == Method::Spring) m_.force_peak_n = std::max(m_.force_peak_n, r_.world->coupling_state(r_.coupling).mean_force_on_towing_world.length());
        m_.artic_max_deg = std::max(m_.artic_max_deg, std::abs(articulation_deg(r_)));
        if (r_.cfg.method == Method::Spring) {
            const machine::CouplingState& s = r_.world->coupling_state(r_.coupling);
            m_.angle_max_deg = std::max({m_.angle_max_deg, std::abs(s.yaw_rad) * 180.0 / kPi, std::abs(s.pitch_rad) * 180.0 / kPi, std::abs(s.roll_rad) * 180.0 / kPi});
            m_.roll_max_deg = std::max(m_.roll_max_deg, std::abs(s.roll_rad) * 180.0 / kPi);
        }
        m_.ay_peak = std::max(m_.ay_peak, std::abs(ma.linear.length() * ma.angular.z));
        m_.end_speed = ma.linear.length();
    }
    [[nodiscard]] const Metrics& metrics() const { return m_; }
    void reset() {
        const bool finite = m_.finite;
        m_ = Metrics{};
        m_.finite = finite;
        sum_ = 0.0;
        count_ = 0;
    }

private:
    const Rig& r_;
    real dt_;
    Metrics m_;
    real sum_ = 0.0;
    Vec3 e1_{}, e2_{}, a1_{}, a2_{}, v1_{};
    int count_ = 0;
};

using Before = std::function<void(World&, real)>;

// Steps `seconds`, calling `before(world, t)` ahead of each tick, stops early once `stop` says so.
Metrics run(Rig& r, real seconds, const Before& before, const std::function<bool()>& stop = {}) {
    Recorder rec(r);
    const real dt = r.world->dt();
    const auto n = static_cast<std::size_t>(seconds / dt + 0.5);
    std::size_t i = 0;
    for (; i < n; ++i) {
        const real t = static_cast<real>(i) * dt;
        if (before) before(*r.world, t);
        r.world->step();
        rec.sample(i > 2);
        if (!rec.metrics().finite) break;
        if (stop && stop()) break;
    }
    Metrics m = rec.metrics();
    m.seconds = static_cast<real>(i) * dt;
    return m;
}

void settle(Rig& r, real seconds) { ps_test::settle(*r.world, seconds); }

void launch(Rig& r, real speed) {
    const Vec3 forward = r.world->get_pose(r.a).transform_direction(Vec3::unit_x());
    r.world->backend().set_motion(r.a, Motion{forward * speed, Vec3::zero()});
    r.world->backend().set_motion(r.b, Motion{forward * speed, Vec3::zero()});
    vehicle::VehicleResetOptions o;
    o.gear = 0;
    o.engine_state = drivetrain::EngineState::Off;
    o.clutch_locked = false;
    r.world->reset_vehicle(r.va, o);
    r.world->reset_vehicle(r.vb, o);
    if (r.chain) {
        r.world->backend().set_motion(r.c, Motion{forward * speed, Vec3::zero()});
        r.world->reset_vehicle(r.vc, o);
    }
}

Metrics scenario_rest(const Cfg& c) {
    Rig r = build_rig(c);
    settle(r, 4.0);
    return run(r, 2.0, {});
}

Metrics scenario_brake(const Cfg& c) {
    Rig r = build_rig(c, -700.0);
    settle(r, 2.0);
    launch(r, k100KmH);
    run(r, 1.0, {});
    return run(r, 15.0, [](World& w, real) {
        w.set_control("brake", 0.8);
        w.set_control("v1.brake", 0.8);
    }, [&r] { return r.world->get_motion(r.a).linear.length() < 0.5; });
}

// Only the towing vehicle's brake is pressed, and the steering wobbles a little: nothing is left symmetric for a heavy
// trailer to balance on.
Metrics scenario_brake_wobble(const Cfg& c, real brake = 1.0, real amplitude = 0.02, real period = 1.0) {
    Rig r = build_rig(c, -700.0);
    settle(r, 2.0);
    launch(r, k100KmH);
    run(r, 1.0, {});
    return run(r, 25.0, [=](World& w, real t) {
        w.set_control("brake", brake);
        w.set_control("steer", amplitude * std::sin(2.0 * kPi * t / period));
    }, [&r] { return r.world->get_motion(r.a).linear.length() < 0.5; });
}

Metrics scenario_lane_change(const Cfg& c, real speed = k100KmH, real amplitude = 0.03, real period = 3.0) {
    Rig r = build_rig(c, -700.0);
    settle(r, 2.0);
    launch(r, speed);
    run(r, 1.0, {});
    Metrics m = run(r, period + 4.0, [=](World& w, real t) {
        w.set_control("steer", t < period ? amplitude * std::sin(2.0 * kPi * t / period) : 0.0);
    });
    m.artic_end_deg = std::abs(articulation_deg(r));
    return m;
}

// A parked pair with the trailer pushed `offset` metres along x: how long until the hitch is back at its rest error?
struct RecoveryResult {
    real seconds_to_1mm = -1.0; // -1: not within the run
    real error_after_1s_mm = 0.0;
    real rest_mm = 0.0;
};

RecoveryResult scenario_recovery(const Cfg& c, real offset = 0.02) {
    Rig r = build_rig(c);
    settle(r, 4.0);
    RecoveryResult out;
    const Vec3 rest = hitch_error(r);
    out.rest_mm = rest.length() * 1e3;
    Pose pose = r.world->get_pose(r.b);
    pose.position.x += offset;
    r.world->backend().set_pose(r.b, pose);
    const real dt = r.world->dt();
    const auto n = static_cast<std::size_t>(4.0 / dt + 0.5);
    for (std::size_t i = 0; i < n; ++i) {
        r.world->step();
        const real t = static_cast<real>(i + 1) * dt;
        const real off = (hitch_error(r) - rest).length();
        if (out.seconds_to_1mm < 0.0 && off < 1e-3) out.seconds_to_1mm = t;
        if (off >= 1e-3) out.seconds_to_1mm = -1.0; // the last entry into the band counts
        if (i + 1 == static_cast<std::size_t>(1.0 / dt + 0.5)) out.error_after_1s_mm = off * 1e3;
    }
    return out;
}

Metrics scenario_manoeuvre(const Cfg& c) {
    Rig r = build_rig(c);
    settle(r, 3.0);
    r.elements.push_back(std::make_unique<SpeedHold>(Frame(r.a), 0.5, 6000.0, 4000.0));
    r.world->add_force_element(r.elements.back().get());
    run(r, 3.0, {});
    return run(r, 20.0, [](World& w, real t) { w.set_control("steer", 0.5 * std::sin(2.0 * kPi * 0.2 * t)); });
}

Metrics scenario_reverse_jackknife(const Cfg& c) {
    Rig r = build_rig(c);
    settle(r, 3.0);
    r.elements.push_back(std::make_unique<SpeedHold>(Frame(r.a), -1.5, 6000.0, 5000.0));
    r.world->add_force_element(r.elements.back().get());
    return run(r, 20.0, [](World& w, real) { w.set_control("steer", 0.5); });
}

Metrics scenario_brake_and_lock(const Cfg& c) {
    Rig r = build_rig(c, -700.0);
    settle(r, 2.0);
    launch(r, 20.0);
    run(r, 0.5, {});
    return run(r, 6.0, [](World& w, real) {
        w.set_control("brake", 1.0);
        w.set_control("steer", 1.0);
    });
}

// Three bodies through their own hitches: the first link carries the second's load.
Metrics scenario_chain_rest(const Cfg& c) {
    Rig r = build_chain(c);
    settle(r, 4.0);
    return run(r, 2.0, {});
}

Metrics scenario_chain_brake(const Cfg& c) {
    Rig r = build_chain(c, -700.0);
    settle(r, 2.0);
    launch(r, k100KmH);
    run(r, 1.0, {});
    return run(r, 15.0, [](World& w, real) {
        w.set_control("brake", 0.8);
        w.set_control("v1.brake", 0.8);
        w.set_control("v2.brake", 0.8);
    }, [&r] { return r.world->get_motion(r.a).linear.length() < 0.5; });
}

Metrics scenario_chain_lane_change(const Cfg& c, real speed) {
    Rig r = build_chain(c, -700.0);
    settle(r, 2.0);
    launch(r, speed);
    run(r, 1.0, {});
    return run(r, 7.0, [](World& w, real t) { w.set_control("steer", t < 3.0 ? 0.03 * std::sin(2.0 * kPi * t / 3.0) : 0.0); });
}

Metrics scenario_chain_manoeuvre(const Cfg& c) {
    Rig r = build_chain(c);
    settle(r, 3.0);
    r.elements.push_back(std::make_unique<SpeedHold>(Frame(r.a), 0.5, 6000.0, 4000.0));
    r.world->add_force_element(r.elements.back().get());
    run(r, 3.0, {});
    return run(r, 20.0, [](World& w, real t) { w.set_control("steer", 0.5 * std::sin(2.0 * kPi * 0.2 * t)); });
}

// `skip_s`: seconds after the draft force steps on that are stepped but not recorded.
Metrics scenario_tractor_draft(Cfg c, real draft_n, real skip_s = 0.0) {
    c.tractor = true;
    Rig r = build_rig(c);
    settle(r, 3.0);
    r.world->set_control("gear_request", 1.0);
    r.world->set_control("hand_throttle", 1.0);
    r.world->set_control("assist.auto_clutch", 1.0);
    run(r, 4.0, {});
    r.elements.push_back(std::make_unique<RearwardForce>(Frame(r.b), draft_n));
    r.world->add_force_element(r.elements.back().get());
    if (skip_s > 0.0) run(r, skip_s, {});
    return run(r, 10.0 - skip_s, {});
}

real kinetic_energy(const Rig& r) {
    const auto body = [&r](BodyId id, real mass) {
        const Motion m = r.world->get_motion(id);
        return 0.5 * mass * dot(m.linear, m.linear) + 0.5 * dot(m.angular, r.world->backend().get_inertia_world(id) * m.angular);
    };
    real e = body(r.a, r.mass_a) + body(r.b, r.mass_b);
    for (VehicleId v : {r.va, r.vb}) {
        for (std::size_t i = 0; i < r.world->vehicle_wheel_count(v); ++i) {
            const real w = r.world->wheel_omega(v, i);
            e += 0.5 * 0.905 * w * w;
        }
    }
    return e;
}

struct CoastResult {
    real loss_percent = 0.0;
    Metrics m;
};

// A straight lossless coast, both vehicles without powertrain, rolling resistance or drag.
CoastResult scenario_coast(const Cfg& c, real seconds = 30.0) {
    Cfg cc = c;
    cc.coast_shell = true;
    Rig r = build_rig(cc, -1200.0);
    settle(r, 2.0);
    launch(r, k100KmH);
    run(r, 0.5, {});
    const real e0 = kinetic_energy(r);
    CoastResult out;
    out.m = run(r, seconds, {});
    out.loss_percent = 100.0 * (e0 - kinetic_energy(r)) / e0;
    return out;
}

// A tractor with a wheel-less 800 kg implement hanging from its socket, centre of mass 1 m behind and 0.3 m above it:
// nothing but the hitch keeps it up. The 3.2 m of lever across tractor and implement is the hard case for the rows'
// coupling, a locked hitch's lever turns every linear error into a rotational one.
Rig build_implement(const Cfg& cfg) {
    Rig r;
    r.cfg = cfg;
    WorldConfig wc;
    wc.tick_rate_hz = cfg.tick_hz;
    wc.substep_rate_hz = cfg.substep_hz;
    wc.job_workers = cfg.workers;
    r.world = std::make_unique<World>(wc);
    auto surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    r.world->set_surface_table(surfaces);
    BodyDesc ground;
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    static const ShapeHandle mesh = r.world->create_shape(ps_test::flat_surface_mesh(1000.0, surfaces->id_for("asphalt")));
    r.world->create_body(ground, mesh);

    r.mass_a = 6000.0;
    r.mass_b = 800.0;
    r.socket_a = Vec3{-2.2, 0.0, -0.78};
    r.socket_b = Vec3{1.0, 0.0, -0.3};
    BodyDesc a;
    a.shape = BoxShape{Vec3{1.2, 0.35, 0.9}};
    a.mass = r.mass_a;
    a.friction = 0.5;
    a.pose.position = Vec3{0.0, 0.0, 1.28};
    r.a = r.world->create_body(a);
    BodyDesc b;
    b.shape = BoxShape{Vec3{0.5, 1.0, 0.25}};
    b.mass = r.mass_b;
    b.friction = 0.5;
    b.pose.position = a.pose.position + r.socket_a - r.socket_b;
    r.b = r.world->create_body(b);
    r.va = r.world->create_vehicle(tractor_desc(), r.a);
    vehicle::VehicleDesc shell;
    shell.name = "implement";
    shell.source = "test fixture";
    r.vb = r.world->create_vehicle(shell, r.b);
    add_link(r);
    for (const char* name : {"assist.auto_clutch", "assist.auto_blip", "assist.auto_shift"}) r.world->set_control(name, 0.0);
    return r;
}

Metrics scenario_implement_rest(const Cfg& c) {
    Rig r = build_implement(c);
    settle(r, 3.0);
    return run(r, 2.0, {});
}

// A rearward force on the implement's lower edge, the load a plough puts on a locked hitch, stepped on.
class EdgeDraft final : public IForceElement {
public:
    EdgeDraft(Frame frame, real magnitude) : frame_(frame), magnitude_(magnitude) {}
    [[nodiscard]] const Frame& frame() const override { return frame_; }
    [[nodiscard]] Wrench evaluate(const ForceElementContext& ctx) const override {
        const Vec3 force = ctx.frame_pose.transform_direction(Vec3::unit_x()) * -magnitude_;
        return Wrench{force, cross(ctx.frame_pose.transform_direction(Vec3{0.0, 0.0, -0.5}), force)};
    }

private:
    Frame frame_;
    real magnitude_;
};

Metrics scenario_implement_draft(const Cfg& c, real draft_n = 15000.0) {
    Rig r = build_implement(c);
    settle(r, 3.0);
    r.elements.push_back(std::make_unique<EdgeDraft>(Frame(r.b), draft_n));
    r.world->add_force_element(r.elements.back().get());
    return run(r, 2.0, {});
}

// Weaving at 3 m/s, the implement swung round with the tractor.
Metrics scenario_implement_turn(const Cfg& c) {
    Rig r = build_implement(c);
    settle(r, 3.0);
    launch(r, 3.0);
    return run(r, 3.0, [](World& w, real t) { w.set_control("steer", 0.5 * std::sin(2.0 * kPi * t / 1.5)); });
}

struct PendulumResult {
    real energy_change_percent = 0.0; // of the swing energy m g L, at the end of the run
    real worst_percent = 0.0;
    real err_max_mm = 0.0;
    bool finite = true;
};

// Two wheel-less vehicles: a 20 t anchor with gravity off and a 1500 kg bar released level from its socket (90 degrees
// from hanging), no tyres, no ground. The analogue of D10's hinge swing (-22 % in 60 s on the Jolt hinge).
PendulumResult scenario_pendulum(const Cfg& c, real seconds = 60.0) {
    Rig r;
    r.cfg = c;
    WorldConfig wc;
    wc.tick_rate_hz = c.tick_hz;
    wc.substep_rate_hz = c.substep_hz;
    wc.job_workers = c.workers;
    wc.collision_steps = c.collision_steps;
    r.world = std::make_unique<World>(wc);

    constexpr real kLength = 3.0;
    BodyDesc a;
    a.shape = BoxShape{Vec3{0.5, 0.5, 0.5}};
    a.mass = 20000.0;
    a.gravity_enabled = false;
    a.pose.position = Vec3{0.0, 0.0, 100.0};
    r.a = r.world->create_body(a);
    BodyDesc b;
    b.shape = BoxShape{Vec3{1.4, 0.4, 0.15}};
    b.mass = 1500.0;
    b.pose.position = Vec3{-kLength, 0.0, 100.0};
    r.b = r.world->create_body(b);
    r.mass_a = a.mass;
    r.mass_b = b.mass;
    r.socket_a = Vec3::zero();
    r.socket_b = Vec3{kLength, 0.0, 0.0};

    vehicle::VehicleDesc empty;
    empty.name = "shell";
    empty.source = "spike A0 fixture";
    r.va = r.world->create_vehicle(empty, r.a);
    r.vb = r.world->create_vehicle(empty, r.b);
    add_link(r);

    const real swing = r.mass_b * 9.81 * kLength;
    const auto energy = [&r] {
        const auto body = [&r](BodyId id, real mass) {
            const Motion m = r.world->get_motion(id);
            return 0.5 * mass * dot(m.linear, m.linear) + 0.5 * dot(m.angular, r.world->backend().get_inertia_world(id) * m.angular);
        };
        return body(r.a, r.mass_a) + body(r.b, r.mass_b) + r.mass_b * 9.81 * (r.world->get_pose(r.b).position.z - 100.0);
    };
    PendulumResult out;
    const real e0 = energy();
    const auto n = static_cast<std::size_t>(seconds / r.world->dt() + 0.5);
    real last = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        r.world->step();
        const real err = hitch_error(r).length();
        if (!std::isfinite(err) || err > 50.0) {
            out.finite = false;
            break;
        }
        out.err_max_mm = std::max(out.err_max_mm, err * 1e3);
        last = 100.0 * (energy() - e0) / swing;
        if (std::abs(last) > std::abs(out.worst_percent)) out.worst_percent = last;
    }
    out.energy_change_percent = last;
    return out;
}

void print_header() {
    std::printf("  %-34s %8s %8s %8s %9s %9s %9s %9s %8s %7s %7s\n", "case", "max mm", "mean mm", "end mm", "ring mm", "ring m/s2", "dv car", "dv trl", "F kN", "art deg", "ay/v");
}

void print_row(const std::string& label, const Metrics& m) {
    std::printf("  %-34s %8.3f %8.3f %8.3f %9.4f %9.3f %9.2e %9.2e %8.2f %7.1f %7.2f%s\n", label.c_str(), m.err_max_mm, m.err_mean_mm, m.err_final_mm, m.ring_err_mm,
                m.ring_accel, m.dv_a, m.dv_b, m.force_peak_n / 1e3, m.artic_max_deg, m.ay_peak, m.finite ? "" : "  DIVERGED");
}

// A lane-change speed the combination survives: the 7500 kg trailer behind a 1500 kg car is stable at 60 km/h and
// jackknifes at 80 km/h with any hitch, so above that the rows measure the jackknife, not the hitch.
real stable_speed(real trailer_mass) { return trailer_mass > 3000.0 ? 60.0 / 3.6 : k100KmH; }

// The hitch scenarios for one configuration, one row each.
void print_scenarios(const Cfg& c, const std::string& label) {
    print_row(label + " rest", scenario_rest(c));
    print_row(label + " brake", scenario_brake(c));
    print_row(label + " lane change", scenario_lane_change(c, stable_speed(c.trailer_mass)));
    print_row(label + " 0.5 m/s", scenario_manoeuvre(c));
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Pinned behaviour of the recommended method.
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("Coupling: a vehicle without engine, driven wheels or wheels at all builds and rolls", "[machine][vehicle]") {
    // The tandem stands on its own: its centre of mass lies between the axles.
    Cfg c;
    c.method = Method::None;
    c.trailer_mass = 7500.0;
    Rig r = build_rig(c, -700.0);
    settle(r, 2.0);
    REQUIRE(r.world->vehicle_wheel_count(r.vb) == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK_FALSE(r.world->wheel_is_driven(r.vb, i));
        CHECK(r.world->wheel_telemetry(r.vb, i).fz > 10000.0);
    }
    CHECK(r.world->powertrain_state(r.vb).engine_state == drivetrain::EngineState::Off);

    launch(r, 10.0);
    ps_test::settle(*r.world, 1.0);
    const real expected = r.world->get_motion(r.b).linear.x / 0.317;
    CHECK(r.world->wheel_omega(r.vb, 0) == Catch::Approx(expected).margin(0.5));

    // Zero wheels: a shell vehicle is a plain body with a shadow.
    const PendulumResult p = scenario_pendulum(Cfg{}, 0.5);
    CHECK(p.finite);
}

namespace {

// The velocities the solved wrench gives the bodies, applied directly, not through the solver's own A.
struct After {
    Vec3 va, wa, vb, wb;
};

After apply_solve(const machine::SocketBody& a, const machine::SocketBody& b, const machine::SocketSolve& s, real h) {
    return {a.linear + s.force_on_a * (h * a.inv_mass), a.angular + a.inv_inertia_world * s.torque_on_a * h, b.linear - s.force_on_a * (h * b.inv_mass), b.angular + b.inv_inertia_world * s.torque_on_b * h};
}

// yaw, pitch, roll of the towed frame in the towing one, written out again here from the matrix.
std::array<real, 3> test_angles(const Quat& fa, const Quat& fb) {
    const Mat3 r = (fa.conjugate() * fb).to_mat3();
    return {std::atan2(r(1, 0), r(0, 0)), std::asin(-r(2, 0)), std::atan2(r(2, 1), r(2, 2))};
}

Quat advance(const Quat& q, const Vec3& w, real dt) {
    const real len = w.length();
    return len > 0.0 ? (Quat::from_axis_angle(w / len, len * dt) * q).normalized() : q;
}

// Finite-difference angle rates of the frames under these angular velocities.
std::array<real, 3> test_angle_rates(const Quat& fa, const Quat& fb, const Vec3& wa, const Vec3& wb) {
    constexpr real dt = 1e-6;
    const auto plus = test_angles(advance(fa, wa, dt), advance(fb, wb, dt));
    const auto minus = test_angles(advance(fa, wa, -dt), advance(fb, wb, -dt));
    return {(plus[0] - minus[0]) / (2.0 * dt), (plus[1] - minus[1]) / (2.0 * dt), (plus[2] - minus[2]) / (2.0 * dt)};
}

void socket_test_bodies(machine::SocketBody& a, machine::SocketBody& b, real mass_b) {
    a.linear = Vec3{10.0, 0.5, -0.1};
    a.angular = Vec3{0.1, -0.2, 0.3};
    a.socket_offset = Vec3{-2.5, 0.2, -0.2};
    a.socket_position = Vec3{1.0, 2.0, 0.4};
    a.inv_mass = 1.0 / 1500.0;
    a.inv_inertia_world = Mat3::diagonal(1.0 / 500.0, 1.0 / 1800.0, 1.0 / 2000.0);
    a.hitch_frame = Quat::from_axis_angle(Vec3::unit_z(), 0.2);
    b = a;
    b.linear = Vec3{9.7, 0.2, 0.05};
    b.angular = Vec3{-0.05, 0.1, -0.2};
    b.socket_offset = Vec3{3.0, -0.1, -0.4};
    b.socket_position = Vec3{1.002, 2.001, 0.399};
    b.inv_mass = 1.0 / mass_b;
    b.inv_inertia_world = Mat3::diagonal(10.0 / (mass_b * 4.0), 10.0 / (mass_b * 9.0), 10.0 / (mass_b * 10.0));
    // yaw 0.5, pitch 0.1, roll 0.25 relative to the towing frame.
    b.hitch_frame = (a.hitch_frame * Quat::from_axis_angle(Vec3::unit_z(), 0.5) * Quat::from_axis_angle(Vec3::unit_y(), 0.1) * Quat::from_axis_angle(Vec3::unit_x(), 0.25)).normalized();
}

} // namespace

TEST_CASE("Coupling: the socket solve satisfies its implicit equation at any mass ratio", "[machine][vehicle]") {
    const real h = 1.0 / 960.0;
    const real omega = 2.0 * kPi * 60.0, g = h * omega * omega + 2.0 * omega;
    const auto expected_rate = [&](real position, real rate) { return rate - (position * (omega * omega) + rate * g) * (h / (1.0 + h * g)); };
    machine::HitchClassDesc all_free = free_hitch();

    for (const real mass_b : {1500.0, 7500.0, 40000.0}) {
        machine::SocketBody a, b;
        socket_test_bodies(a, b, mass_b);
        const Vec3 edot_free = (a.linear + cross(a.angular, a.socket_offset)) - (b.linear + cross(b.angular, b.socket_offset));

        const machine::SocketSolve s = machine::solve_socket_coupling(a, b, all_free, h);
        CHECK(s.rows == 3);
        const After v = apply_solve(a, b, s, h);
        const Vec3 edot = (v.va + cross(v.wa, a.socket_offset)) - (v.vb + cross(v.wb, b.socket_offset));
        for (int k = 0; k < 3; ++k) CHECK(edot[k] == Catch::Approx(expected_rate(s.error[k], edot_free[k])).margin(1e-9));

        // One locked axis: only that angle's rate follows the law, and the socket still does.
        const auto rates_free = test_angle_rates(a.hitch_frame, b.hitch_frame, a.angular, b.angular);
        const std::array<real, 3> angles = test_angles(a.hitch_frame, b.hitch_frame);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            machine::HitchClassDesc d = all_free;
            (axis == 0 ? d.yaw : (axis == 1 ? d.pitch : d.roll)).mode = machine::HitchAxisMode::Locked;
            const machine::SocketSolve locked = machine::solve_socket_coupling(a, b, d, h);
            CHECK(locked.rows == 4);
            CHECK(locked.angle[axis] == Catch::Approx(angles[axis]).margin(1e-12));
            const After w = apply_solve(a, b, locked, h);
            const auto rates = test_angle_rates(a.hitch_frame, b.hitch_frame, w.wa, w.wb);
            CHECK(rates[axis] == Catch::Approx(expected_rate(angles[axis], rates_free[axis])).margin(1e-5));
            const Vec3 edot_locked = (w.va + cross(w.wa, a.socket_offset)) - (w.vb + cross(w.wb, b.socket_offset));
            for (int k = 0; k < 3; ++k) CHECK(edot_locked[k] == Catch::Approx(expected_rate(locked.error[k], edot_free[k])).margin(1e-9));
        }

        // All six at once. The wrench pair is an equal and opposite force at the two sockets plus a pure couple, so
        // the system's moment is what those two points being e apart leaves.
        machine::HitchClassDesc rigid = all_free;
        rigid.yaw.mode = rigid.pitch.mode = rigid.roll.mode = machine::HitchAxisMode::Locked;
        const machine::SocketSolve six = machine::solve_socket_coupling(a, b, rigid, h);
        CHECK(six.rows == 6);
        const After w = apply_solve(a, b, six, h);
        const auto rates = test_angle_rates(a.hitch_frame, b.hitch_frame, w.wa, w.wb);
        for (std::size_t axis = 0; axis < 3; ++axis) CHECK(rates[axis] == Catch::Approx(expected_rate(angles[axis], rates_free[axis])).margin(1e-5));
        const Vec3 com_a = a.socket_position - a.socket_offset, com_b = b.socket_position - b.socket_offset;
        const Vec3 moment = cross(com_a, six.force_on_a) + six.torque_on_a + cross(com_b, -six.force_on_a) + six.torque_on_b;
        CHECK((moment - cross(six.error, six.force_on_a)).length() <= 1e-9 * std::max(real(1.0), six.force_on_a.length()));
    }
}

TEST_CASE("Coupling: a limit row only exists at the stop and only ever pushes back", "[machine][vehicle]") {
    const real h = 1.0 / 960.0;
    machine::HitchClassDesc d = free_hitch();
    d.yaw = machine::HitchAxis{machine::HitchAxisMode::Limited, -0.4, 0.4};
    d.pitch = machine::HitchAxis{machine::HitchAxisMode::Limited, -0.3, 0.3};
    machine::SocketBody a, b;
    socket_test_bodies(a, b, 1500.0); // yaw 0.5, past the stop
    const Vec3 up = a.hitch_frame.rotate(Vec3::unit_z());
    a.angular = b.angular = Vec3::zero();

    machine::SocketSolve s = machine::solve_socket_coupling(a, b, d, h);
    CHECK(s.rows == 4);
    CHECK(dot(s.torque_on_b, up) < 0.0); // the towed frame is pushed back towards the stop

    // Coming back from the stop fast enough: the row would pull, so it comes out.
    b.angular = up * -20.0;
    b.linear = a.linear - cross(b.angular, b.socket_offset); // the sockets stay together, only the angle moves
    s = machine::solve_socket_coupling(a, b, d, h);
    CHECK(s.rows == 3);

    // Inside the range and leaving it within this substep: the stop acts before the angle gets past it.
    b.hitch_frame = (a.hitch_frame * Quat::from_axis_angle(Vec3::unit_z(), 0.399)).normalized();
    b.angular = up * 2.0;
    b.linear = a.linear - cross(b.angular, b.socket_offset);
    s = machine::solve_socket_coupling(a, b, d, h);
    CHECK(s.rows == 4);
    CHECK(dot(s.torque_on_b, up) < 0.0);
    b.angular = Vec3::zero();
    b.linear = a.linear;
    s = machine::solve_socket_coupling(a, b, d, h);
    CHECK(s.rows == 3);
}

TEST_CASE("Coupling: a car holds a 5x trailer within millimetres, without tick ringing, and the shadows see the force", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;

    const Metrics rest = scenario_rest(c);
    CHECK(rest.finite);
    CHECK(rest.err_max_mm < 0.2);
    CHECK(rest.ring_err_mm < 0.001);

    const Metrics brake = scenario_brake(c);
    CHECK(brake.finite);
    CHECK(brake.end_speed < 0.6);
    CHECK(brake.err_max_mm < 0.5);
    CHECK(brake.ring_err_mm < 0.01);
    CHECK(brake.dv_a < 1e-5);
    CHECK(brake.dv_b < 1e-5);

    const Metrics lane = scenario_lane_change(c, stable_speed(c.trailer_mass));
    CHECK(lane.finite);
    CHECK(lane.err_max_mm < 0.4);
    CHECK(lane.ring_err_mm < 0.01);
    CHECK(lane.dv_a < 1e-5);
    CHECK(lane.dv_b < 1e-5);
    CHECK(lane.ay_peak > 2.0);
    CHECK(lane.artic_max_deg > 1.0);
    CHECK(lane.artic_max_deg < 30.0); // not a jackknife against box contact

    Cfg light;
    light.trailer_mass = 1500.0;
    const Metrics fast = scenario_lane_change(light);
    CHECK(fast.err_max_mm < 0.4);
    CHECK(fast.ring_err_mm < 0.01);
    CHECK(fast.dv_a < 1e-5);
    CHECK(fast.dv_b < 1e-5);
    CHECK(fast.ay_peak > 2.0);
    CHECK(fast.artic_max_deg < 30.0);

    const Metrics draft = scenario_tractor_draft(c, 15000.0, 2.0);
    CHECK(draft.finite);
    CHECK(draft.err_max_mm < 0.3);
    CHECK(draft.ring_err_mm < 0.01);
}

TEST_CASE("Coupling: a position error closes at the hitch frequency, however stiff", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    c.hitch = load_class("ball");
    const RecoveryResult good = scenario_recovery(c);
    CHECK(good.seconds_to_1mm > 0.0);
    CHECK(good.seconds_to_1mm < 0.05);
    // Without the shadow's end pose reaching the backend, the shadow closed this error inside a tick, ended it at rest
    // and Jolt's position hardly moved: 0.092 s at 480 Hz against 0.021 s at 60 Hz.
    c.frequency_hz = 480.0;
    const RecoveryResult stiff = scenario_recovery(c);
    CHECK(stiff.seconds_to_1mm > 0.0);
    CHECK(stiff.seconds_to_1mm <= good.seconds_to_1mm);
}

TEST_CASE("Coupling: a locked three-point hitch carries an implement as one body with the tractor", "[machine][vehicle]") {
    Cfg c;
    c.hitch = load_class("three_point_locked");
    const Metrics rest = scenario_implement_rest(c);
    CHECK(rest.finite);
    CHECK(rest.err_max_mm < 0.3);
    CHECK(rest.angle_max_deg < 0.05);
    CHECK(rest.ring_err_mm < 0.001);
    const Metrics draft = scenario_implement_draft(c);
    CHECK(draft.finite);
    CHECK(draft.err_max_mm < 2.0);
    CHECK(draft.angle_max_deg < 0.5);
    const Metrics turn = scenario_implement_turn(c);
    CHECK(turn.finite);
    CHECK(turn.err_max_mm < 1.0);
    CHECK(turn.angle_max_deg < 0.2);

    // The same implement on a free socket swings down and hangs from the tractor.
    Cfg free;
    const Metrics hanging = scenario_implement_rest(free);
    CHECK(hanging.finite);
    CHECK(hanging.artic_max_deg < 90.0);
}

TEST_CASE("Coupling: an articulation limit ends a jackknife at the limit", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    c.hitch.yaw = machine::HitchAxis{machine::HitchAxisMode::Limited, -45.0 * kPi / 180.0, 45.0 * kPi / 180.0};
    c.hitch.pitch = machine::HitchAxis{machine::HitchAxisMode::Limited, -20.0 * kPi / 180.0, 20.0 * kPi / 180.0};
    const Metrics reverse = scenario_reverse_jackknife(c);
    CHECK(reverse.finite);
    CHECK(reverse.artic_max_deg > 40.0);
    CHECK(reverse.artic_max_deg < 46.0);
    CHECK(reverse.err_max_mm < 1.0);

    // Locked wheels and full lock at 20 m/s: the trailer comes round, hits the class's stop and the hitch stays closed.
    Cfg hard;
    hard.hitch = load_class("drawbar_pin");
    const Metrics lock = scenario_brake_and_lock(hard);
    CHECK(lock.finite);
    CHECK(lock.artic_max_deg > 70.0);
    CHECK(lock.artic_max_deg < 76.0);
    CHECK(lock.err_max_mm < 2.0);
}

TEST_CASE("Coupling: the fifth wheel carries roll, the ball does not", "[machine][vehicle]") {
    // Roll the trailer about its own length: on a ball it stands on its wheels, on a fifth wheel the hitch resists.
    const auto roll_kick = [](const char* name) {
        Cfg c;
        c.trailer_mass = 7500.0;
        c.hitch = load_class(name);
        Rig r = build_rig(c);
        settle(r, 3.0);
        Motion m = r.world->get_motion(r.b);
        m.angular.x += 0.4;
        r.world->backend().set_motion(r.b, m);
        return run(r, 1.0, {});
    };
    const Metrics ball = roll_kick("ball");
    const Metrics fifth = roll_kick("fifth_wheel");
    CHECK(ball.finite);
    CHECK(fifth.finite);
    CHECK(ball.roll_max_deg > 0.1);
    CHECK(fifth.roll_max_deg < 0.2 * ball.roll_max_deg);
    CHECK(fifth.err_max_mm < 5.0);
}

TEST_CASE("Coupling: the shipped hitch classes load, and the loader rejects bad ones", "[machine][vehicle][io]") {
    using machine::HitchAxisMode;
    const machine::HitchClassDesc ball = load_class("ball");
    CHECK(ball.yaw.mode == HitchAxisMode::Limited);
    CHECK(ball.frequency_hz == 60.0);
    CHECK(ball.damping_ratio == 1.0);
    CHECK(load_class("drawbar_pin").yaw.mode == HitchAxisMode::Limited);
    const machine::HitchClassDesc fifth = load_class("fifth_wheel");
    CHECK(fifth.roll.mode == HitchAxisMode::Locked);
    CHECK(fifth.pitch.mode == HitchAxisMode::Limited);
    const machine::HitchClassDesc rigid = load_class("three_point_locked");
    CHECK(rigid.yaw.mode == HitchAxisMode::Locked);
    CHECK(rigid.pitch.mode == HitchAxisMode::Locked);
    CHECK(rigid.roll.mode == HitchAxisMode::Locked);

    const std::filesystem::path dir = std::filesystem::temp_directory_path() / ("ps_hitch_io_" + std::to_string(reinterpret_cast<std::uintptr_t>(&dir)));
    std::filesystem::create_directories(dir);
    const auto write = [&dir](const std::string& name, const std::string& text) {
        const std::filesystem::path p = dir / name;
        std::ofstream(p) << text;
        return p.string();
    };
    const std::string latch = R"("latch": {"time_s": 0.5, "max_error_m": 0.1, "max_angle_deg": 5, "max_speed_m_s": 0.5})";
    const std::string free_axes = R"("yaw": {"mode": "free"}, "pitch": {"mode": "free"}, "roll": {"mode": "free"}, )";
    const auto text = [&](const std::string& axes, const std::string& extra = "", const std::string& frequency = "60") {
        return R"({"format": "physics_sim.hitch_class/1", "name": "t", "source": "test", )" + axes + R"("frequency_hz": )" + frequency + R"(, "damping_ratio": 1, )" + extra + latch + "}";
    };
    const auto rejects = [&](const std::string& json, const char* needle) {
        const std::string path = write("bad.json", json);
        CHECK_THROWS_WITH(io::load_hitch_class_json(path), Catch::Matchers::ContainsSubstring(needle));
    };
    CHECK(io::load_hitch_class_json(write("good.json", text(free_axes))).name == "t");
    rejects(text(free_axes, R"("colour": 1, )"), "colour");
    rejects(R"({"format": "physics_sim.hitch_class/2", "name": "t", "source": "t"})", "format");
    rejects(text(R"("yaw": {"mode": "stiff"}, "pitch": {"mode": "free"}, "roll": {"mode": "free"}, )"), "mode");
    rejects(text(R"("yaw": {"mode": "free", "max_deg": 3}, "pitch": {"mode": "free"}, "roll": {"mode": "free"}, )"), "max_deg");
    rejects(text(R"("yaw": {"mode": "limited", "min_deg": 10, "max_deg": 20}, "pitch": {"mode": "limited", "min_deg": -5, "max_deg": 5}, "roll": {"mode": "free"}, )"), "min < 0 < max");
    rejects(text(R"("yaw": {"mode": "limited", "min_deg": -175, "max_deg": 175}, "pitch": {"mode": "limited", "min_deg": -5, "max_deg": 5}, "roll": {"mode": "free"}, )"), "170");
    rejects(text(R"("yaw": {"mode": "limited", "min_deg": -90, "max_deg": 90}, "pitch": {"mode": "free"}, "roll": {"mode": "free"}, )"), "pitch must be limited");
    rejects(text(R"("yaw": {"mode": "free"}, "pitch": {"mode": "limited", "min_deg": -80, "max_deg": 80}, "roll": {"mode": "free"}, )"), "75");
    rejects(text(free_axes, "", "0"), "frequency_hz");

    // A user part overrides one value of a shipped class.
    const std::string child = write("stiffer.json", R"({"extends": ")" + (std::filesystem::path(PS_SOURCE_DIR) / "data/hitch_classes/ball.json").generic_string() + R"(", "frequency_hz": 120})");
    const machine::HitchClassDesc stiffer = io::load_hitch_class_json(child);
    CHECK(stiffer.frequency_hz == 120.0);
    CHECK(stiffer.yaw.mode == HitchAxisMode::Limited);
    std::filesystem::remove_all(dir);
}

TEST_CASE("Coupling: bodies overlapping at the hitch neither collide nor stand on each other", "[machine][vehicle]") {
    Cfg c;
    c.drawbar_overlap = true;
    Rig r = build_rig(c);
    const auto check_tick = [&r](int& contacts) {
        for (const ContactEvent& e : r.world->last_contact_events()) {
            if ((e.a == r.a && e.b == r.b) || (e.a == r.b && e.b == r.a)) ++contacts;
        }
    };
    int contacts = 0;
    for (int i = 0; i < 960; ++i) {
        r.world->step();
        check_tick(contacts);
    }
    CHECK(contacts == 0);
    const Metrics rest = run(r, 2.0, {});
    CHECK(rest.finite);
    CHECK(rest.err_max_mm < 1.0);
    CHECK(rest.force_peak_n < 6000.0);
    // The extra axle carries a load of the same order as the main one, not a wheel forced into full bump.
    const real main_axle = r.world->wheel_telemetry(r.vb, 0).fz;
    const real extra_axle = r.world->wheel_telemetry(r.vb, 2).fz;
    CHECK(main_axle > 1000.0);
    CHECK(extra_axle > 0.0);
    CHECK(extra_axle < main_axle);

    // Swinging the trailer about the socket: the same bodies, now moving through each other.
    r.elements.push_back(std::make_unique<SpeedHold>(Frame(r.a), 2.0, 6000.0, 4000.0));
    r.world->add_force_element(r.elements.back().get());
    const Metrics drive = run(r, 6.0, [&contacts, &check_tick](World& w, real t) {
        w.set_control("steer", 0.3 * std::sin(2.0 * kPi * t / 3.0));
        check_tick(contacts);
    });
    CHECK(drive.finite);
    CHECK(drive.err_max_mm < 2.0);
    CHECK(contacts == 0);
}

TEST_CASE("Coupling: a disabled pair makes no contacts, every other pair does", "[machine][backend]") {
    World world{WorldConfig{}};
    BodyDesc d;
    d.shape = BoxShape{Vec3{1.0, 1.0, 1.0}};
    d.mass = 100.0;
    d.gravity_enabled = false;
    const BodyId a = world.create_body(d);
    d.pose.position.x = 0.5;
    const BodyId b = world.create_body(d);
    d.pose.position.x = 1.0;
    const BodyId c = world.create_body(d);
    const auto touching = [&world](BodyId x, BodyId y) {
        world.step();
        for (const ContactEvent& e : world.last_contact_events()) {
            if ((e.a == x && e.b == y) || (e.a == y && e.b == x)) return true;
        }
        return false;
    };
    const std::vector<std::pair<BodyId, BodyId>> ab = {{a, b}};
    world.backend().set_disabled_collision_pairs(ab);
    CHECK_FALSE(touching(a, b));
    CHECK(touching(b, c));
    CHECK(touching(a, c));
    // All three carry the filter now: a and c still meet, as a tractor meets its second trailer.
    const std::vector<std::pair<BodyId, BodyId>> chain = {{a, b}, {b, c}};
    world.backend().set_disabled_collision_pairs(chain);
    CHECK_FALSE(touching(a, b));
    CHECK_FALSE(touching(b, c));
    CHECK(touching(a, c));
    world.backend().set_disabled_collision_pairs({});
    CHECK(touching(a, b));
}

TEST_CASE("Coupling: a chain of two trailers shares one island", "[machine][vehicle]") {
    Cfg c;
    c.tractor = true;
    c.trailer_mass = 7500.0;
    c.trailer_brakes = true;
    c.hitch = load_class("drawbar_pin");
    const Metrics rest = scenario_chain_rest(c);
    CHECK(rest.finite);
    CHECK(rest.err_max_mm < 0.2);
    CHECK(rest.err2_max_mm < 0.2);
    const Metrics brake = scenario_chain_brake(c);
    CHECK(brake.finite);
    CHECK(brake.err_max_mm < 0.5);
    CHECK(brake.err2_max_mm < 0.5);
    CHECK(brake.end_speed < 1.0);
    const Metrics lane = scenario_chain_lane_change(c, 60.0 / 3.6);
    CHECK(lane.finite);
    CHECK(lane.err_max_mm < 0.5);
    CHECK(lane.err2_max_mm < 0.5);

    // One sweep would leave the first link at half this: it never sees the second link's force in the same substep.
    CHECK(rest.err_max_mm == Catch::Approx(0.014).margin(0.002));
    // Two sweeps have converged: the first link's error is the same at four.
    Cfg more = c;
    more.passes = 4;
    CHECK(scenario_chain_rest(more).err_max_mm == Catch::Approx(rest.err_max_mm).margin(1e-3));
    CHECK(scenario_chain_brake(more).err_max_mm == Catch::Approx(brake.err_max_mm).margin(2e-3));
}

namespace {

// Tractor and a tandem trailer standing apart on their own wheels: a single axle would tip without the hitch.
Cfg uncoupled_tandem() {
    Cfg c;
    c.method = Method::None;
    c.tractor = true;
    c.trailer_mass = 7500.0;
    c.hitch = load_class("drawbar_pin");
    return c;
}

// The trailer's nose stands where its wheels put it, a hand's width off the spawn height: its socket is chosen from
// where the two stand now, `error` from the towing one.
void place_socket(Rig& r, const Vec3& error) {
    r.socket_b = r.world->get_pose(r.b).inverse().transform_point(r.world->get_pose(r.a).transform_point(r.socket_a) - error);
}

} // namespace

TEST_CASE("Coupling: attach says what stands in the way, with the numbers", "[machine][vehicle]") {
    using Catch::Matchers::ContainsSubstring;
    Rig r = build_rig(uncoupled_tandem());
    settle(r, 0.5);
    place_socket(r, Vec3{0.25, 0.0, 0.0});
    machine::CouplingDesc d = link_desc(r);

    const machine::AttachCheck far = r.world->check_attach(d);
    CHECK_FALSE(far.ok);
    CHECK(far.error_m == Catch::Approx(0.25).margin(0.03));
    CHECK_THAT(far.message, ContainsSubstring("m apart") && ContainsSubstring("drawbar_pin") && ContainsSubstring("0.100"));
    CHECK_THROWS_WITH(r.world->attach(d), far.message);

    // In reach of the hitch, but moving.
    Pose pose = r.world->get_pose(r.b);
    pose.position.x += 0.25;
    r.world->backend().set_pose(r.b, pose);
    r.world->backend().set_motion(r.b, Motion{Vec3{1.0, 0.0, 0.0}, Vec3::zero()});
    const machine::AttachCheck fast = r.world->check_attach(d);
    CHECK_FALSE(fast.ok);
    CHECK(fast.speed_m_s == Catch::Approx(1.0).margin(0.01));
    CHECK_THAT(fast.message, ContainsSubstring("m/s"));
    r.world->backend().set_motion(r.b, Motion{});

    // Turned past the class's limit, the sockets coincident.
    const Pose towing = r.world->get_pose(r.a);
    const Vec3 hitch_point = towing.transform_point(r.socket_a);
    const real half = 0.5 * 90.0 * kPi / 180.0;
    pose.orientation = Quat{0.0, 0.0, std::sin(half), std::cos(half)} * towing.orientation;
    pose.position = hitch_point - pose.orientation.rotate(r.socket_b);
    r.world->backend().set_pose(r.b, pose);
    const machine::AttachCheck turned = r.world->check_attach(d);
    CHECK_FALSE(turned.ok);
    CHECK(turned.angle_rad[0] * 180.0 / kPi == Catch::Approx(90.0).margin(1.0));
    CHECK_THAT(turned.message, ContainsSubstring("yaw") && ContainsSubstring("-75.00..75.00"));

    // Who may tow whom.
    machine::CouplingDesc who = d;
    who.towed = VehicleId{99, 1};
    CHECK_THAT(r.world->check_attach(who).message, ContainsSubstring("unknown vehicle"));
    who.towed = who.towing;
    CHECK_THAT(r.world->check_attach(who).message, ContainsSubstring("tow itself"));

    machine::CouplingDesc bad = d;
    bad.hitch.frequency_hz = 0.0;
    CHECK_THROWS_AS(r.world->attach(bad), std::invalid_argument);

    pose = r.world->get_pose(r.a);
    pose.position += pose.orientation.rotate(r.socket_a - r.socket_b);
    r.world->backend().set_pose(r.b, pose);
    REQUIRE(r.world->check_attach(d).ok);
    r.coupling = r.world->attach(d);
    CHECK_THAT(r.world->check_attach(d).message, ContainsSubstring("already towed"));
    machine::CouplingDesc back = d;
    back.towing = d.towed;
    back.towed = d.towing;
    CHECK_THAT(r.world->check_attach(back).message, ContainsSubstring("loop"));
}

TEST_CASE("Coupling: a latch closes the real socket error without a jolt", "[machine][vehicle]") {
    Rig r = build_rig(uncoupled_tandem());
    settle(r, 1.0);
    place_socket(r, Vec3{0.08, 0.0, 0.0});
    r.cfg.method = Method::Spring;
    const real e0 = hitch_error(r).length();
    r.coupling = r.world->attach(link_desc(r));
    CHECK(r.world->coupling_state(r.coupling).latch_progress == 0.0);

    const real latch = r.cfg.hitch.latch_time_s;
    const real dt = r.world->dt();
    real first_force = -1.0, peak_force = 0.0, worst_track = 0.0;
    for (int i = 0; i < static_cast<int>(latch / dt); ++i) {
        r.world->step();
        const machine::CouplingState& s = r.world->coupling_state(r.coupling);
        const real force = s.mean_force_on_towing_world.length();
        if (first_force < 0.0) first_force = force;
        peak_force = std::max(peak_force, force);
        const real u = (static_cast<real>(i) + 1.0) * dt / latch;
        worst_track = std::max(worst_track, std::abs(hitch_error(r).length() - e0 * ((2.0 * u - 3.0) * u * u + 1.0)));
    }
    std::printf("latch from %.1f mm: first tick %.0f N, peak %.0f N, worst off the ramp %.3f mm\n", e0 * 1e3, first_force, peak_force, worst_track * 1e3);
    CHECK(r.world->coupling_state(r.coupling).latch_progress == Catch::Approx(1.0).margin(0.02));
    // The ramp starts at 6 e0 / T^2 = 1.9 m/s^2: 6400 N on the two vehicles' reduced mass.
    CHECK(first_force < 3000.0);
    CHECK(peak_force < 9000.0);
    CHECK(worst_track < 3e-3);
    const Metrics after = run(r, 1.0, {});
    CHECK(r.world->coupling_state(r.coupling).latch_progress == 1.0);
    CHECK(after.finite);
    CHECK(after.err_final_mm < 0.2);
}

TEST_CASE("Coupling: a latch closes an angle on a locked hitch", "[machine][vehicle]") {
    // Tilted about the socket the moment before it would drop, and hung up. Everything else is the same in both runs,
    // the tractor's own first ticks on its springs included.
    const auto hang = [](real tilt_deg) {
        Cfg c;
        c.method = Method::None;
        c.hitch = load_class("three_point_locked");
        Rig r = build_implement(c);
        const Pose towing = r.world->get_pose(r.a);
        const Vec3 hitch_point = towing.transform_point(r.socket_a);
        const real half = 0.5 * tilt_deg * kPi / 180.0;
        Pose pose = r.world->get_pose(r.b);
        pose.orientation = Quat{0.0, std::sin(half), 0.0, std::cos(half)} * pose.orientation;
        pose.position = hitch_point - pose.orientation.rotate(r.socket_b);
        r.world->backend().set_pose(r.b, pose);
        r.cfg.method = Method::Spring;
        r.coupling = r.world->attach(link_desc(r));
        return r;
    };
    Rig flat = hang(0.0);
    flat.world->step();
    const real flat_first = flat.world->coupling_state(flat.coupling).mean_force_on_towing_world.length();

    Rig r = hang(3.0);
    r.world->step();
    const real first_force = r.world->coupling_state(r.coupling).mean_force_on_towing_world.length();
    CHECK(std::abs(r.world->coupling_state(r.coupling).pitch_rad) * 180.0 / kPi == Catch::Approx(3.0).margin(0.1));
    const Metrics m = run(r, 2.0, {});
    std::printf("latch from 3 deg: first tick %.0f N (flat %.0f N), peak %.0f N, end %.3f mm %.3f deg\n", first_force, flat_first, m.force_peak_n, m.err_final_mm, r.world->coupling_state(r.coupling).pitch_rad * 180.0 / kPi);
    CHECK(m.finite);
    CHECK(first_force < 1.2 * flat_first);
    CHECK(m.force_peak_n < 40000.0);
    CHECK(m.err_final_mm < 0.3);
    CHECK(std::abs(r.world->coupling_state(r.coupling).pitch_rad) < 1e-3);
}

TEST_CASE("Coupling: detach frees both vehicles and retires the id", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    Rig r = build_rig(c, -700.0);
    settle(r, 0.5);
    launch(r, 8.0);
    ps_test::settle(*r.world, 0.25);
    const HitchId old = r.coupling;
    r.world->detach(old);
    CHECK_FALSE(r.world->hitch_alive(old));
    CHECK_THROWS_AS(r.world->coupling_state(old), std::invalid_argument);
    CHECK_THROWS_AS(r.world->detach(old), std::invalid_argument);

    r.elements.push_back(std::make_unique<SpeedHold>(Frame(r.a), 14.0, 8000.0, 8000.0));
    const ForceElementId pull = r.world->add_force_element(r.elements.back().get());
    for (int i = 0; i < 480; ++i) r.world->step();
    CHECK(hitch_error(r).length() > 1.0);

    // A reused slot does not bring the old id back.
    r.world->remove_force_element(pull);
    const Pose towing = r.world->get_pose(r.a);
    r.world->backend().set_motion(r.a, Motion{});
    r.world->backend().set_motion(r.b, Motion{});
    Pose pose = towing;
    pose.position += towing.orientation.rotate(r.socket_a - r.socket_b);
    r.world->backend().set_pose(r.b, pose);
    const HitchId fresh = r.world->attach(link_desc(r));
    CHECK(fresh.index == old.index);
    CHECK(fresh.generation != old.generation);
    CHECK(r.world->hitch_alive(fresh));
    CHECK_FALSE(r.world->hitch_alive(old));
}

TEST_CASE("Coupling: after detach the two bodies collide and the wheels see the partner again", "[machine][vehicle]") {
    Cfg c;
    c.drawbar_overlap = true;
    Rig r = build_rig(c);
    ps_test::settle(*r.world, 1.0);
    const real jockey_coupled = r.world->wheel_telemetry(r.vb, 2).fz;
    r.world->detach(r.coupling);
    bool touched = false;
    real jockey_free = 0.0;
    for (int i = 0; i < 4; ++i) {
        r.world->step();
        jockey_free = std::max(jockey_free, r.world->wheel_telemetry(r.vb, 2).fz);
        for (const ContactEvent& e : r.world->last_contact_events()) {
            touched = touched || (e.a == r.a && e.b == r.b) || (e.a == r.b && e.b == r.a);
        }
    }
    CHECK(touched);
    // A cast starting inside the car's box reports a hit at once, and the wheel reads full bump.
    CHECK(jockey_free > 2.0 * jockey_coupled);
}

TEST_CASE("Coupling: a vehicle can be destroyed with its hitches alive", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    c.tractor = true;
    c.hitch = load_class("drawbar_pin");
    Rig r = build_chain(c);
    settle(r, 0.5);
    CHECK_THROWS_AS(r.world->destroy_body(r.b), std::logic_error);

    r.world->destroy_vehicle(r.vb); // the one in the middle
    CHECK_FALSE(r.world->hitch_alive(r.coupling));
    CHECK_FALSE(r.world->hitch_alive(r.coupling2));
    CHECK_THROWS_AS(r.world->destroy_vehicle(r.vb), std::invalid_argument);
    CHECK_THROWS_AS(r.world->vehicle_air(r.vb), std::invalid_argument);
    machine::CouplingDesc d = link_desc(r);
    CHECK_FALSE(r.world->check_attach(d).ok);

    r.world->step();
    const WorldSnapshot& snap = r.world->snapshots().read();
    REQUIRE(snap.vehicles.size() == 3);
    CHECK(snap.vehicles[0].id == r.va);
    CHECK_FALSE(snap.vehicles[1].id.valid());
    CHECK(snap.vehicles[1].wheels.empty());
    CHECK(snap.vehicles[2].id == r.vc);

    r.world->destroy_body(r.b);
    const real z_before = r.world->get_pose(r.a).position.z;
    for (int i = 0; i < 120; ++i) r.world->step();
    CHECK(std::abs(r.world->get_pose(r.a).position.z - z_before) < 0.02);
    CHECK(r.world->wheel_telemetry(r.va, 0).fz > 1000.0);
    CHECK(r.world->wheel_telemetry(r.vc, 0).fz > 1000.0);
}

TEST_CASE("Coupling: attach, detach and destroy are refused from inside a step", "[machine][vehicle]") {
    Rig r = build_rig(uncoupled_tandem());
    settle(r, 1.0);
    place_socket(r, Vec3{});
    const machine::CouplingDesc d = link_desc(r);
    REQUIRE(r.world->check_attach(d).ok);
    int refused = 0;
    r.world->set_between_forces_and_backend_step_hook([&](World& w) {
        // Caught by type: a valid desc leaves the "between steps" rule as the only reason to throw.
        try { (void)w.attach(d); } catch (const std::logic_error&) { ++refused; }
        try { w.destroy_vehicle(r.vb); } catch (const std::logic_error&) { ++refused; }
        try { w.detach(HitchId{0, 1}); } catch (const std::logic_error&) { ++refused; }
    });
    r.world->step();
    r.world->set_between_forces_and_backend_step_hook({});
    CHECK(refused == 3);
    CHECK(r.world->check_attach(d).ok);
    CHECK_NOTHROW(r.world->wheel_telemetry(r.vb, 0));
}

TEST_CASE("Coupling: the state hash sees a hitch, and only a live one", "[machine][vehicle]") {
    Rig r = build_rig(uncoupled_tandem());
    settle(r, 1.0);
    place_socket(r, Vec3{0.05, 0.0, 0.0});
    const std::uint64_t plain = r.world->state_hash();
    const machine::CouplingDesc d = link_desc(r);
    const HitchId hitch = r.world->attach(d);
    const std::uint64_t latching = r.world->state_hash();
    CHECK(latching != plain);
    r.world->detach(hitch);
    CHECK(r.world->state_hash() == plain);

    // The latch steers everything that follows, so it is state.
    machine::CouplingDesc slower = d;
    slower.hitch.latch_time_s = 2.0 * d.hitch.latch_time_s;
    const HitchId other = r.world->attach(slower);
    CHECK(r.world->state_hash() != latching);
    r.world->detach(other);

    r.world->step();
    r.world->attach(d);
    const std::uint64_t before = r.world->state_hash();
    r.world->step();
    CHECK(r.world->state_hash() != before);
}

TEST_CASE("Coupling: runtime attach, detach and destroy are worker-count independent", "[machine][vehicle]") {
    const auto hashes = [](int workers) {
        Cfg c = uncoupled_tandem();
        c.workers = workers;
        Rig r = build_rig(c);
        settle(r, 0.5);
        place_socket(r, Vec3{0.05, 0.0, 0.0});
        std::vector<std::uint64_t> out;
        HitchId hitch{};
        for (int i = 0; i < 480; ++i) {
            if (i == 0 || i == 240) hitch = r.world->attach(link_desc(r));
            if (i == 200) r.world->detach(hitch);
            if (i == 400) r.world->destroy_vehicle(r.vb);
            r.world->set_control("steer", 0.05 * std::sin(2.0 * kPi * static_cast<real>(i) / 120.0));
            r.world->step();
            out.push_back(r.world->state_hash());
        }
        return out;
    };
    CHECK(hashes(1) == hashes(10));
}

namespace {

machine::TowBrakeDesc channel_link() {
    machine::TowBrakeDesc d;
    d.kind = machine::TowBrakeKind::Channel;
    return d;
}

machine::TowBrakeDesc overrun(real deadband_n = 300.0) {
    machine::TowBrakeDesc d;
    d.kind = machine::TowBrakeKind::Overrun;
    d.deadband_n = deadband_n;
    d.lag_s = 0.1;
    return d;
}

} // namespace

TEST_CASE("Coupling: braking with a heavy trailer", "[.][diag][machine]") {
    std::printf("towing brake, steer amplitude, period: art deg / peak kN / s  none, channel, overrun\n");
    for (const real brake : {0.8, 1.0}) {
        for (const real amplitude : {0.02, 0.05, 0.1}) {
            for (const real period : {1.0, 2.0, 4.0}) {
                Cfg c;
                c.trailer_mass = 7500.0;
                c.trailer_brakes = true;
                c.hitch = load_class("ball");
                const Metrics none = scenario_brake_wobble(c, brake, amplitude, period);
                c.tow_brake = channel_link();
                const Metrics linked = scenario_brake_wobble(c, brake, amplitude, period);
                c.tow_brake = overrun();
                const Metrics overran = scenario_brake_wobble(c, brake, amplitude, period);
                std::printf("%.1f %.2f %.0f: %.1f %.0f %.1f,  %.1f %.0f %.1f,  %.1f %.0f %.1f\n", brake, amplitude, period, none.artic_max_deg, none.force_peak_n / 1e3, none.seconds,
                            linked.artic_max_deg, linked.force_peak_n / 1e3, linked.seconds, overran.artic_max_deg, overran.force_peak_n / 1e3, overran.seconds);
            }
        }
    }
}

TEST_CASE("Coupling: a trailer braked from the towing vehicle stays behind it", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    c.trailer_brakes = true;
    c.hitch = load_class("ball");
    const Metrics unbraked = scenario_brake_wobble(c);
    c.tow_brake = channel_link();
    const Metrics linked = scenario_brake_wobble(c);
    c.tow_brake = overrun();
    const Metrics overran = scenario_brake_wobble(c);
    std::printf("brake + wobble, 7500 kg behind a car, art deg / force kN / s: none %.1f %.1f %.2f, channel %.1f %.1f %.2f, overrun %.1f %.1f %.2f\n",
                unbraked.artic_max_deg, unbraked.force_peak_n / 1e3, unbraked.seconds, linked.artic_max_deg, linked.force_peak_n / 1e3, linked.seconds,
                overran.artic_max_deg, overran.force_peak_n / 1e3, overran.seconds);
    CHECK(unbraked.artic_max_deg > 45.0);
    CHECK(linked.finite);
    CHECK(linked.artic_max_deg < 3.0);
    CHECK(linked.seconds < 0.7 * unbraked.seconds);
    CHECK(overran.finite);
    CHECK(overran.artic_max_deg < 10.0);
    CHECK(overran.seconds < unbraked.seconds);
}

TEST_CASE("Coupling: a channel link follows the named channel through its lag", "[machine][vehicle]") {
    Cfg c;
    c.tractor = true;
    c.trailer_mass = 7500.0;
    c.trailer_brakes = true;
    c.hitch = load_class("drawbar_pin");
    c.tow_brake = channel_link();
    // The trailer exists first and the tractor reads scoped channels: the name holds whatever the order was.
    c.trailer_first = true;
    c.scope_a = "tractor";
    c.scope_b = "trailer";
    Rig r = build_rig(c);
    CHECK(r.world->vehicle_channel(r.va, "brake") == "tractor.brake");
    CHECK(r.world->vehicle_channel(r.vb, "brake") == "trailer.brake");
    settle(r, 0.25);
    CHECK(r.world->coupling_state(r.coupling).tow_brake == 0.0);

    r.world->set_control("tractor.brake", 0.5);
    const real a = r.world->dt() / (c.tow_brake->lag_s + r.world->dt());
    for (int i = 1; i <= 48; ++i) {
        r.world->step();
        CHECK(r.world->coupling_state(r.coupling).tow_brake == Catch::Approx(0.5 * (1.0 - std::pow(1.0 - a, i))).epsilon(1e-9));
    }
    // A bare or differently scoped brake does not reach it.
    r.world->set_control("tractor.brake", 0.0);
    r.world->set_control("brake", 1.0);
    r.world->set_control("trailer.brake", 1.0);
    for (int i = 0; i < 720; ++i) r.world->step();
    CHECK(r.world->coupling_state(r.coupling).tow_brake < 1e-3);
}

TEST_CASE("Coupling: channel scopes are checked", "[machine][vehicle]") {
    using Catch::Matchers::ContainsSubstring;
    Cfg c;
    c.scope_a = "same";
    c.scope_b = "same";
    CHECK_THROWS_WITH(build_rig(c), ContainsSubstring("same."));
    c.scope_b = "a.b";
    CHECK_THROWS_WITH(build_rig(c), ContainsSubstring("no dots"));
    c.scope_a = "v1"; // the automatic name of the second vehicle
    c.scope_b.clear();
    CHECK_THROWS_WITH(build_rig(c), ContainsSubstring("v1."));
    World world{WorldConfig{}};
    CHECK_THROWS_AS(world.vehicle_channel(VehicleId{0, 1}, "brake"), std::invalid_argument);
}

TEST_CASE("Coupling: a chain brakes from the front", "[machine][vehicle]") {
    Cfg c;
    c.tractor = true;
    c.trailer_mass = 7500.0;
    c.trailer_brakes = true;
    c.hitch = load_class("drawbar_pin");
    c.tow_brake = channel_link();
    Rig r = build_chain(c);
    settle(r, 0.25);
    r.world->set_control("brake", 1.0);
    for (int i = 0; i < 240; ++i) r.world->step();
    const real first = r.world->coupling_state(r.coupling).tow_brake;
    const real second = r.world->coupling_state(r.coupling2).tow_brake;
    CHECK(first > 0.9);
    CHECK(second > 0.5);
    CHECK(second < first);

    // The front link goes: the demand it passed down the chain goes with it, and the rear link lets its brakes off.
    r.world->detach(r.coupling);
    r.world->set_control("brake", 0.0);
    for (int i = 0; i < 720; ++i) r.world->step();
    CHECK(r.world->coupling_state(r.coupling2).tow_brake < 1e-3);
}

TEST_CASE("Coupling: an overrun brake's hitch force is state", "[machine][vehicle]") {
    // Equal poses and velocities at the hash, a different tick before it: the trailer pulled the hitch taut, which an
    // overrun brake ignores, so only the remembered force differs. It decides the next tick's demand. A channel link
    // does not read it, and those two runs have to agree: the check that the restored state is all of the state.
    const auto hash_after = [](const machine::TowBrakeDesc& link, real pull) {
        Cfg c;
        c.trailer_brakes = true;
        c.tow_brake = link;
        Rig r = build_rig(c);
        Pose pose_a = r.world->get_pose(r.a), pose_b = r.world->get_pose(r.b);
        pose_a.position.z += 50.0;
        pose_b.position.z += 50.0;
        r.world->backend().set_pose(r.a, pose_a);
        r.world->backend().set_pose(r.b, pose_b);
        launch(r, 0.0);
        r.world->backend().set_motion(r.b, Motion{Vec3{-pull, 0.0, 0.0}, Vec3::zero()});
        r.world->step();
        REQUIRE(r.world->coupling_state(r.coupling).tow_brake == 0.0);
        r.world->backend().set_pose(r.a, pose_a);
        r.world->backend().set_pose(r.b, pose_b);
        r.world->backend().set_motion(r.a, Motion{});
        r.world->backend().set_motion(r.b, Motion{});
        return r.world->state_hash();
    };
    CHECK(hash_after(overrun(), 1.0) != hash_after(overrun(), 0.0));
    CHECK(hash_after(channel_link(), 1.0) == hash_after(channel_link(), 0.0));
}

TEST_CASE("Coupling: an overrun brake acts on hitch compression and lets go in reverse", "[machine][vehicle]") {
    Cfg c;
    c.trailer_brakes = true;
    c.tow_brake = overrun();
    Rig r = build_rig(c, -700.0);
    settle(r, 1.0);
    launch(r, 15.0);
    for (int i = 0; i < 240; ++i) r.world->step();
    CHECK(r.world->coupling_state(r.coupling).tow_brake == 0.0); // rolling, the hitch in tension
    real peak = 0.0;
    r.world->set_control("brake", 0.8);
    for (int i = 0; i < 240; ++i) {
        r.world->step();
        peak = std::max(peak, r.world->coupling_state(r.coupling).tow_brake);
    }
    CHECK(peak > 0.3);

    // Pushed backwards on a small deadband: with the reverse release it lets go, without it the trailer brakes.
    const auto reversing = [](real release) {
        Cfg c2;
        c2.trailer_brakes = true;
        c2.tow_brake = overrun(40.0);
        c2.tow_brake->reverse_release_speed_m_s = release;
        Rig r2 = build_rig(c2, -700.0);
        settle(r2, 1.0);
        launch(r2, -2.0);
        r2.elements.push_back(std::make_unique<SpeedHold>(Frame(r2.a), -2.0, 6000.0, 6000.0));
        r2.world->add_force_element(r2.elements.back().get());
        real most = 0.0;
        for (int i = 0; i < 240; ++i) {
            r2.world->step();
            most = std::max(most, r2.world->coupling_state(r2.coupling).tow_brake);
        }
        return most;
    };
    CHECK(reversing(0.5) < 1e-3); // what settling left, decaying
    CHECK(reversing(100.0) > 0.0);
}

TEST_CASE("Coupling: a tow brake's lag is state", "[machine][vehicle]") {
    // In the air with the wheels still, the brake moves nothing: the two runs differ in the lag's state alone. A line of
    // its own, so the towing vehicle's brakes stay out of it.
    const auto hash_after = [](real line) {
        Cfg c;
        c.trailer_brakes = true;
        c.tow_brake = channel_link();
        c.tow_brake->channel = "line";
        Rig r = build_rig(c);
        for (const BodyId body : {r.a, r.b}) {
            Pose pose = r.world->get_pose(body);
            pose.position.z += 50.0;
            r.world->backend().set_pose(body, pose);
        }
        launch(r, 0.0);
        r.world->set_control("line", 0.5);
        for (int i = 0; i < 4; ++i) r.world->step();
        r.world->set_control("line", line);
        r.world->step();
        return r.world->state_hash();
    };
    CHECK(hash_after(0.5) == hash_after(0.5));
    CHECK(hash_after(0.5) != hash_after(1.0));
}

TEST_CASE("Coupling: the brake lets go with the hitch", "[machine][vehicle]") {
    // A tandem: it stands on its own once it is loose, where a single axle would drop its nose on the road.
    Cfg c;
    c.trailer_mass = 7500.0;
    c.trailer_brakes = true;
    c.tow_brake = channel_link();
    Rig r = build_rig(c, -700.0);
    settle(r, 0.5);
    launch(r, 15.0);
    r.world->set_control("brake", 1.0);
    for (int i = 0; i < 120; ++i) r.world->step();
    REQUIRE(r.world->coupling_state(r.coupling).tow_brake > 0.5);
    r.world->detach(r.coupling);
    r.world->set_control("brake", 0.0);
    r.world->step();
    const real before = r.world->get_motion(r.b).linear.length();
    for (int i = 0; i < 120; ++i) r.world->step();
    // Rolling free: half a second of it, nothing like a braked trailer's 3 m/s or more.
    CHECK(before - r.world->get_motion(r.b).linear.length() < 0.25);
}

TEST_CASE("Coupling: a tow brake is worker-count independent", "[machine][vehicle]") {
    const auto hashes = [](int workers, const machine::TowBrakeDesc& link) {
        Cfg c;
        c.trailer_mass = 7500.0;
        c.trailer_brakes = true;
        c.hitch = load_class("ball");
        c.tow_brake = link;
        c.workers = workers;
        Rig r = build_rig(c, -700.0);
        settle(r, 0.5);
        launch(r, 15.0);
        std::vector<std::uint64_t> out;
        for (int i = 0; i < 240; ++i) {
            r.world->set_control("brake", i < 60 ? 0.0 : 0.6);
            r.world->set_control("steer", 0.03 * std::sin(2.0 * kPi * static_cast<real>(i) / 120.0));
            r.world->step();
            out.push_back(r.world->state_hash());
        }
        return out;
    };
    CHECK(hashes(1, channel_link()) == hashes(10, channel_link()));
    CHECK(hashes(1, overrun()) == hashes(10, overrun()));
}

TEST_CASE("Coupling: a tractor tows the tipper from the sockets their files name", "[machine][vehicle]") {
    Cfg c;
    c.tractor = true;
    c.tipper = true;
    c.trailer_mass = 14000.0;
    Rig r = build_rig(c, -700.0);
    settle(r, 3.0);
    const machine::HitchLoads rest = r.world->hitch_loads(r.coupling);
    // 0.45 m ahead of the bogie, 4.95 m from it to the eye, if the pin stood where the trailer's own wheels hold the eye.
    // It stands lower, the tractor's rear settles under the load, and the eye and two axles share it three ways: the lever
    // is an upper bound. What must hold exactly is the sum.
    const real lever_tongue = 14000.0 * 9.81 * 0.45 / 4.95;
    real wheels = 0.0;
    for (std::size_t i = 0; i < r.world->vehicle_wheel_count(r.vb); ++i) wheels += r.world->wheel_telemetry(r.vb, i).fz;
    std::printf("tipper at rest: %.0f N on the pin (%.0f N by the lever), %.0f N on the wheels, %.0f N weight, %.3f mm\n", rest.tongue_load_n, lever_tongue, wheels, 14000.0 * 9.81,
                hitch_error(r).length() * 1e3);
    CHECK(rest.towed_mass_kg == Catch::Approx(14000.0));
    CHECK(rest.tongue_load_n + wheels == Catch::Approx(14000.0 * 9.81).epsilon(0.005));
    CHECK(rest.tongue_load_n > 0.7 * lever_tongue);
    CHECK(rest.tongue_load_n < lever_tongue);
    CHECK(hitch_error(r).length() < 1e-3);

    // Weaving at 8 m/s, then a stop on the tractor's pedal alone: the file's air line brakes the trailer.
    launch(r, 8.0);
    const Metrics weave = run(r, 6.0, [](World& w, real t) { w.set_control("steer", 0.1 * std::sin(2.0 * kPi * t / 3.0)); });
    const real speed = r.world->get_motion(r.a).linear.length();
    r.world->set_control("steer", 0.0);
    r.world->set_control("brake", 0.6);
    const Metrics stop = run(r, 3.0, {});
    std::printf("tipper weave at %.2f m/s: %.3f mm, %.1f kN, %.1f deg; stop: %.2f m/s left, %.3f mm, %.1f kN, line %.2f\n", speed, weave.err_max_mm, weave.force_peak_n / 1e3,
                weave.artic_max_deg, r.world->get_motion(r.a).linear.length(), stop.err_max_mm, stop.force_peak_n / 1e3, r.world->coupling_state(r.coupling).tow_brake);
    CHECK(weave.finite);
    CHECK(speed > 5.0);
    CHECK(weave.artic_max_deg > 2.0);
    CHECK(weave.err_max_mm < 2.0);
    CHECK(stop.err_max_mm < 2.0);
    CHECK(r.world->coupling_state(r.coupling).tow_brake > 0.5);
    CHECK(r.world->get_motion(r.a).linear.length() < 0.5 * speed);
}

TEST_CASE("Coupling: hitch points mate by role and class, one hitch at a time", "[machine][vehicle]") {
    using Catch::Matchers::ContainsSubstring;
    Cfg c;
    c.tractor = true;
    c.tipper = true;
    c.trailer_mass = 14000.0;
    c.method = Method::None;
    Rig r = build_rig(c);

    const auto refused = [&r](VehicleId a, const char* pa, VehicleId b, const char* pb) { return r.world->check_attach(a, pa, b, pb).message; };
    CHECK_THAT(refused(r.va, "nothing", r.vb, "socket_hitch_eye"), ContainsSubstring("no hitch point 'nothing'"));
    CHECK_THAT(refused(r.va, "socket_hitch", r.vb, "nothing"), ContainsSubstring("no hitch point 'nothing'"));
    CHECK_THAT(refused(r.vb, "socket_hitch_eye", r.va, "socket_hitch"), ContainsSubstring("is not a towing point"));
    CHECK_THAT(refused(r.va, "socket_hitch", r.va, "socket_3pt"), ContainsSubstring("is not a towed point"));
    CHECK_THAT(refused(r.va, "socket_3pt", r.vb, "socket_hitch_eye"), ContainsSubstring("takes class 'three_point_locked'"));
    CHECK_THAT(refused(VehicleId{9, 1}, "socket_hitch", r.vb, "socket_hitch_eye"), ContainsSubstring("unknown vehicle"));
    CHECK_THROWS_WITH(r.world->attach(r.va, "socket_3pt", r.vb, "socket_hitch_eye"), ContainsSubstring("takes class"));

    REQUIRE(r.world->check_attach(r.va, "socket_hitch", r.vb, "socket_hitch_eye").ok);
    const HitchId id = r.world->attach(r.va, "socket_hitch", r.vb, "socket_hitch_eye");

    // A second trailer on the same pin: the point is taken whatever the socket error would say.
    TrailerGeometry second = make_trailer(car_desc(), 1500.0, false);
    machine::HitchPointDesc eye = tipper_desc().hitch_points[0];
    eye.position_local = second.socket;
    second.desc.hitch_points.push_back(eye);
    BodyDesc body;
    body.shape = BoxShape{second.half_extents};
    body.mass = 1500.0;
    body.pose.position = Vec3{-30.0, 0.0, 1.0};
    const VehicleId third = r.world->create_vehicle(second.desc, r.world->create_body(body));
    CHECK_THAT(r.world->check_attach(r.va, "socket_hitch", third, "socket_hitch_eye").message, ContainsSubstring("'socket_hitch' of vehicle 0 is in use"));

    // Free again once it is detached: what stands in the way then is only the distance.
    r.world->detach(id);
    CHECK_THAT(r.world->check_attach(r.va, "socket_hitch", third, "socket_hitch_eye").message, ContainsSubstring("apart"));
}

TEST_CASE("Coupling: hitch point data is read and checked", "[machine][vehicle]") {
    using Catch::Matchers::ContainsSubstring;
    const std::vector<machine::HitchPointDesc>& points = tractor_desc().hitch_points;
    const machine::HitchPointDesc* three = machine::find_hitch_point(points, "socket_3pt");
    REQUIRE(three != nullptr);
    CHECK(three->hitch.name == "three_point_locked");
    CHECK(three->role == machine::HitchRole::Towing);
    CHECK(three->rig_sockets == std::vector<std::string>{"socket_3pt_L", "socket_3pt_R", "socket_3pt_top"});
    CHECK(three->position_local.x == Catch::Approx((-2.12 - 2.04 - 2.04) / 3.0));
    CHECK(three->position_local.y == Catch::Approx(0.0).margin(1e-12));
    CHECK(three->position_local.z == Catch::Approx((-0.04 - 0.68 - 0.68) / 3.0));
    REQUIRE(tipper_desc().tow_brake.has_value());
    CHECK(tipper_desc().tow_brake->kind == machine::TowBrakeKind::Channel);
    CHECK(tipper_desc().tow_brake->lag_s == 0.4);

    // The file's positions are the rig's: chassis origin from the front wheel, whose static compression the file's
    // spring and 35 % of 6000 kg give.
    const auto rig = [](const char* model) {
        std::ifstream f(std::string(PS_SOURCE_DIR) + "/data/models/" + model);
        return nlohmann::json::parse(f);
    };
    const nlohmann::json tractor = rig("tractor/tractor.rig.json");
    const vehicle::WheelDesc& fl = tractor_desc().wheels[0];
    Vec3 pivot;
    for (const auto& n : tractor.at("nodes")) {
        if (n.at("name") == "wheel_FL") pivot = Vec3{n.at("pivot")[0].get<real>(), n.at("pivot")[1].get<real>(), n.at("pivot")[2].get<real>()};
    }
    const real compression = 0.35 * 6000.0 * 9.81 / 2.0 / fl.suspension.spring_rate;
    const Vec3 origin = pivot - fl.attachment_local - Vec3{0.0, 0.0, compression};
    const auto in_rig = [](const nlohmann::json& file, const std::string& name) {
        for (const auto& s : file.at("sockets")) {
            if (s.at("name") == name) return Vec3{s.at("position")[0].get<real>(), s.at("position")[1].get<real>(), s.at("position")[2].get<real>()};
        }
        FAIL("no socket " << name);
        return Vec3{};
    };
    const auto near = [](const Vec3& a, const Vec3& b) { return (a - b).length() < 2e-3; };
    const machine::HitchPointDesc* pin = machine::find_hitch_point(points, "socket_hitch");
    REQUIRE(pin != nullptr);
    CHECK(near(pin->position_local + origin, in_rig(tractor, "socket_hitch")));
    Vec3 mean{};
    for (const std::string& s : three->rig_sockets) mean += in_rig(tractor, s) * (1.0 / 3.0);
    CHECK(near(three->position_local + origin, mean));
    const nlohmann::json tipper = rig("trailer_tipper/trailer_tipper.rig.json");
    CHECK(near(tipper_desc().hitch_points[0].position_local + Vec3{0.45, 0.0, 1.1}, in_rig(tipper, "socket_hitch_eye")));

    // Rejections, on a copy of the fixture in its own folder so the relative paths still hold.
    const std::string dir = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/";
    std::ifstream source(dir + "trailer_tipper.json");
    const nlohmann::json good = nlohmann::json::parse(source);
    const std::string path = (std::filesystem::temp_directory_path() / ("ps_hitch_points_" + std::to_string(std::random_device{}()) + ".json")).string();
    const auto load_with = [&](const std::function<void(nlohmann::json&)>& change) {
        nlohmann::json j = good;
        for (auto& w : j["wheels"]) w["tyre"] = dir + w["tyre"].get<std::string>();
        for (auto& p : j["hitch_points"]) p["class"] = dir + p["class"].get<std::string>();
        change(j);
        std::ofstream(path) << j.dump();
        (void)io::load_vehicle_json(path);
    };
    const auto rejects = [&](const std::function<void(nlohmann::json&)>& change, const char* text) {
        CHECK_THROWS_WITH(load_with(change), ContainsSubstring(text));
    };
    CHECK_NOTHROW(load_with([](nlohmann::json&) {}));
    rejects([](nlohmann::json& j) { j["hitch_points"][0]["role"] = "pushed"; }, "'role' must be");
    rejects([](nlohmann::json& j) { j["hitch_points"][0].erase("position"); }, "exactly one");
    rejects([](nlohmann::json& j) { j["hitch_points"][0]["sockets"] = {{"a", {0, 0, 0}}, {"b", {0, 0, 0}}}; }, "exactly one");
    rejects([](nlohmann::json& j) { j["hitch_points"][0].erase("position"); j["hitch_points"][0]["sockets"] = {{"a", {0, 0, 0}}}; }, "at least two");
    rejects([](nlohmann::json& j) { j["hitch_points"][0]["max_towed_mass_kg"] = 100.0; }, "only a towing point");
    rejects([](nlohmann::json& j) { j["hitch_points"][0]["wrong"] = 1; }, "unknown field 'wrong'");
    rejects([](nlohmann::json& j) { j["hitch_points"].push_back(j["hitch_points"][0]); }, "listed twice");
    rejects([](nlohmann::json& j) { j["hitch_points"][0].erase("class"); }, "'class' must be");
    rejects([](nlohmann::json& j) { j["tow_brake"]["kind"] = "magnetic"; }, "'kind' must be");
    rejects([](nlohmann::json& j) { j["tow_brake"]["deadband_n"] = 1.0; }, "unknown field 'deadband_n'");
    rejects([](nlohmann::json& j) { j["tow_brake"] = {{"kind", "overrun"}, {"gain", 1.0}}; }, "unknown field 'gain'");
    rejects([](nlohmann::json& j) { j["tow_brake"]["gain"] = 0.0; }, "gain");
    rejects([](nlohmann::json& j) { j.erase("brake_system"); }, "brake_system");
    std::filesystem::remove(path);
}

TEST_CASE("Coupling: a towing point's ratings are limit findings", "[machine][vehicle]") {
    using drivetrain::LimitFinding;
    using drivetrain::LimitKind;
    using drivetrain::LimitSeverity;
    machine::HitchPointDesc point;
    point.name = "socket_hitch";
    point.max_towed_mass_kg = 10000.0;
    point.max_tongue_load_n = 20000.0;
    const auto grade = [&point](real mass, real tongue) { return machine::check_hitch_limits(point, machine::HitchLoads{mass, tongue}); };
    CHECK(grade(8999.0, 17999.0).empty());
    for (const auto& [share, severity] : std::vector<std::pair<real, LimitSeverity>>{{0.95, LimitSeverity::Near}, {1.1, LimitSeverity::Limiting}, {1.2, LimitSeverity::Exceeded}}) {
        const std::vector<LimitFinding> f = grade(share * 10000.0, share * 20000.0);
        REQUIRE(f.size() == 2);
        CHECK(f[0].kind == LimitKind::TowedMass);
        CHECK(f[0].severity == severity);
        CHECK(f[0].value == Catch::Approx(share * 10000.0));
        CHECK(f[0].limit == 10000.0);
        CHECK(f[0].unit == drivetrain::LimitUnit::Kilogram);
        CHECK(f[0].part.slot == "socket_hitch");
        CHECK(f[0].part.field == "max_towed_mass_kg");
        CHECK(f[1].kind == LimitKind::TongueLoad);
        CHECK(f[1].severity == severity);
        CHECK(f[1].unit == drivetrain::LimitUnit::Newton);
    }
    CHECK(std::string(drivetrain::to_string(LimitKind::TowedMass)) == "TowedMass");
    CHECK(std::string(drivetrain::to_string(LimitKind::TongueLoad)) == "TongueLoad");
    CHECK(std::string(drivetrain::to_string(drivetrain::LimitUnit::Newton)) == "N");
    // Not rated, not graded.
    point.max_towed_mass_kg = 0.0;
    point.max_tongue_load_n = 0.0;
    CHECK(grade(1e9, 1e9).empty());

    // Live: the file's 14 t against a 14 t tipper, then a hitch built by hand has no rating to read, then a chain.
    Cfg c;
    c.tractor = true;
    c.tipper = true;
    c.trailer_mass = 14000.0;
    Rig r = build_rig(c, -700.0);
    settle(r, 2.0);
    std::vector<LimitFinding> live = r.world->hitch_limits(r.coupling);
    REQUIRE(live.size() == 1);
    CHECK(live[0].kind == LimitKind::TowedMass);
    CHECK(live[0].severity == LimitSeverity::Near);
    CHECK(live[0].value == Catch::Approx(14000.0));
    drivetrain::LimitPolicy strict = drivetrain::default_limit_policy();
    strict.hitch_near_share = 0.1;
    strict.hitch_limiting_share = 0.2;
    strict.hitch_exceeded_share = 0.3;
    live = r.world->hitch_limits(r.coupling, &strict);
    REQUIRE(live.size() == 2);
    CHECK(live[0].severity == LimitSeverity::Exceeded);
    CHECK(live[1].kind == LimitKind::TongueLoad);
    CHECK(live[1].value == Catch::Approx(r.world->hitch_loads(r.coupling).tongue_load_n));

    // A hitch on the tipper's tail behind which hangs a car trailer counts on the tractor's pin too.
    TrailerGeometry second = make_trailer(car_desc(), 1500.0, false);
    BodyDesc body;
    body.shape = BoxShape{second.half_extents};
    body.mass = 1500.0;
    const Pose tipper = r.world->get_pose(r.b);
    body.pose.position = tipper.transform_point(Vec3{-3.0, 0.0, -0.6}) - second.socket;
    const VehicleId rear = r.world->create_vehicle(second.desc, r.world->create_body(body));
    machine::CouplingDesc tail;
    tail.towing = r.vb;
    tail.towed = rear;
    tail.socket_towing_local = Vec3{-3.0, 0.0, -0.6};
    tail.socket_towed_local = second.socket;
    tail.hitch = free_hitch();
    const HitchId tail_id = r.world->attach(tail);
    CHECK(r.world->hitch_loads(r.coupling).towed_mass_kg == Catch::Approx(15500.0));
    CHECK(r.world->hitch_loads(tail_id).towed_mass_kg == Catch::Approx(1500.0));
    CHECK(r.world->hitch_limits(tail_id).empty());
    r.world->detach(r.coupling);
    CHECK_THROWS_AS(r.world->hitch_loads(r.coupling), std::invalid_argument);
    CHECK_THROWS_AS(r.world->hitch_limits(r.coupling), std::invalid_argument);
}

TEST_CASE("Coupling: a free swing keeps its energy at the shipped rates", "[machine][vehicle]") {
    // The bar swings about the anchor for 10 s, a few full periods. -41.4 % of m g L in 60 s before the backend's step
    // was made to land on the shadow's end pose; a socket spring then fought the difference between the two paths.
    const PendulumResult p = scenario_pendulum(Cfg{}, 10.0);
    CHECK(p.finite);
    CHECK(std::abs(p.energy_change_percent) < 1.0);
    CHECK(std::abs(p.worst_percent) < 1.5);
    CHECK(p.err_max_mm < 0.5);
}

TEST_CASE("Coupling: a contact still acts on a coupled chassis", "[machine][vehicle]") {
    // The shadow knows no contact: it lets a wheel-less chassis fall. Starting the backend's step from where it lands
    // on the shadow must leave the contact's effect on v1 in place, or the pair sinks through the floor.
    Rig r;
    WorldConfig wc;
    wc.job_workers = 2;
    r.world = std::make_unique<World>(wc);
    BodyDesc ground;
    ground.motion = BodyMotionType::Static;
    ground.shape = BoxShape{Vec3{50.0, 50.0, 0.5}};
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    r.world->create_body(ground);
    BodyDesc a;
    a.shape = BoxShape{Vec3{1.0, 1.0, 0.25}};
    a.mass = 1500.0;
    a.pose.position = Vec3{0.0, 0.0, 0.5};
    r.a = r.world->create_body(a);
    a.pose.position = Vec3{-4.0, 0.0, 0.5};
    r.b = r.world->create_body(a);
    vehicle::VehicleDesc shell;
    shell.name = "shell";
    shell.source = "test fixture";
    r.va = r.world->create_vehicle(shell, r.a);
    r.vb = r.world->create_vehicle(shell, r.b);
    r.socket_a = Vec3{-2.0, 0.0, 0.0};
    r.socket_b = Vec3{2.0, 0.0, 0.0};
    add_link(r);
    for (int i = 0; i < 720; ++i) r.world->step();
    CHECK(r.world->get_pose(r.a).position.z == Catch::Approx(0.25).margin(0.02));
    CHECK(r.world->get_pose(r.b).position.z == Catch::Approx(0.25).margin(0.02));
    CHECK(hitch_error(r).length() < 0.002);
}

TEST_CASE("Coupling: a jackknife stays finite and a slow manoeuvre stays quiet", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    const Metrics slow = scenario_manoeuvre(c);
    CHECK(slow.finite);
    CHECK(slow.err_max_mm < 0.3);
    CHECK(slow.ring_err_mm < 0.01);

    const Metrics reverse = scenario_reverse_jackknife(c);
    CHECK(reverse.finite);
    CHECK(reverse.artic_max_deg > 30.0);
    CHECK(reverse.err_max_mm < 1.0);
    const Metrics hard = scenario_brake_and_lock(c);
    CHECK(hard.finite);
    CHECK(hard.artic_max_deg > 90.0); // really jackknifed, into the box contact
    CHECK(hard.err_max_mm < 100.0);
}

TEST_CASE("Coupling: a coupled pair is worker-count independent and allocates nothing in the tick", "[machine][vehicle]") {
    const auto hashes = [](int workers) {
        Cfg c;
        c.trailer_mass = 7500.0;
        c.workers = workers;
        Rig r = build_rig(c, -700.0);
        settle(r, 0.5);
        launch(r, k100KmH);
        std::vector<std::uint64_t> out;
        for (int i = 0; i < 480; ++i) {
            r.world->set_control("steer", 0.03 * std::sin(2.0 * kPi * static_cast<real>(i) / 480.0));
            r.world->step();
            out.push_back(r.world->state_hash());
        }
        return out;
    };
    CHECK(hashes(1) == hashes(10));

    if (!jobs::debug_alloc::hook_available()) return;
    Cfg c;
    Rig r = build_rig(c, -700.0);
    settle(r, 0.5);
    launch(r, k100KmH);
    ps_test::settle(*r.world, 0.5);
    // The ticks below run with a latch ramping in.
    r.world->detach(r.coupling);
    r.coupling = r.world->attach(link_desc(r));
    jobs::debug_alloc::reset();
    jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 240; ++i) {
        r.world->set_control("steer", 0.03 * std::sin(2.0 * kPi * static_cast<real>(i) / 240.0));
        r.world->step();
    }
    jobs::debug_alloc::set_armed(false);
    CHECK(jobs::debug_alloc::own_violation_count() == 0);
}

// ---------------------------------------------------------------------------------------------------------------------
// Measurements. Run with: ps_unit_tests "Coupling diag*"
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("Coupling diag rows", "[.][diag][machine][vehicle]") {
    print_header();
    Cfg c;
    c.hitch = load_class("three_point_locked");
    print_row("implement rest", scenario_implement_rest(c));
    print_row("implement draft", scenario_implement_draft(c));
    print_row("implement turn", scenario_implement_turn(c));
    machine::SocketBody a, b;
    socket_test_bodies(a, b, 7500.0);
    machine::HitchClassDesc rigid = free_hitch();
    rigid.yaw.mode = rigid.pitch.mode = rigid.roll.mode = machine::HitchAxisMode::Locked;
    machine::HitchClassDesc ball = load_class("ball");
    const auto cost = [&](const machine::HitchClassDesc& d, const char* label) {
        const int count = 2000000;
        real sink = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < count; ++i) {
            a.linear.x = static_cast<real>(i) * 1e-7;
            sink += machine::solve_socket_coupling(a, b, d, 1.0 / 960.0).force_on_a.x;
        }
        const std::chrono::duration<double, std::nano> ns = std::chrono::steady_clock::now() - t0;
        std::printf("  solve %-22s %.1f ns per call (sink %.3g)\n", label, ns.count() / count, sink);
    };
    cost(free_hitch(), "free, 3 rows");
    cost(ball, "ball, 3-5 rows");
    cost(rigid, "locked, 6 rows");
}

TEST_CASE("Coupling diag table", "[.][diag][machine][vehicle]") {
    print_header();
    for (const real mass : {1500.0, 7500.0}) {
        for (const real f : {60.0, 120.0}) {
            Cfg c;
            c.frequency_hz = f;
            c.trailer_mass = mass;
            const std::string label = std::to_string(static_cast<int>(mass)) + " f" + std::to_string(static_cast<int>(f));
            print_scenarios(c, label);
            c.trailer_brakes = true;
            print_row(label + " brake+trl brakes", scenario_brake(c));
            c.trailer_brakes = false;
            print_row(label + " reverse jackknife", scenario_reverse_jackknife(c));
            print_row(label + " brake+lock", scenario_brake_and_lock(c));
            print_row(label + " tractor 15 kN", scenario_tractor_draft(c, 15000.0));
            print_row(label + " tractor after 2 s", scenario_tractor_draft(c, 15000.0, 2.0));
        }
    }
}

TEST_CASE("Coupling diag hard", "[.][diag][machine][vehicle]") {
    // Stiffer always looks better in the lane change and braking rows; these are the cases that can say no.
    std::printf("  %-22s %12s %10s %10s %10s %10s %9s %10s\n", "case", "recover s", "1 s mm", "jk max mm", "lock mm", "lock ring", "F kN", "swing %");
    for (const real mass : {1500.0, 7500.0}) {
        for (const real f : {20.0, 40.0, 60.0, 120.0, 240.0, 480.0}) {
            for (const real zeta : {0.5, 0.7, 1.0, 2.0}) {
                Cfg c;
                c.trailer_mass = mass;
                c.frequency_hz = f;
                c.damping_ratio = zeta;
                const RecoveryResult rec = scenario_recovery(c);
                const Metrics jack = scenario_reverse_jackknife(c);
                const Metrics lock = scenario_brake_and_lock(c);
                char label[64];
                std::snprintf(label, sizeof(label), "m %.0f f %.0f z %.1f", mass, f, zeta);
                Cfg swing;
                swing.frequency_hz = f;
                swing.damping_ratio = zeta;
                const PendulumResult p = scenario_pendulum(swing);
                std::printf("  %-22s %12.3f %10.3f %10.3f %10.3f %10.3f %9.1f %+10.2f%s\n", label, rec.seconds_to_1mm, rec.error_after_1s_mm, jack.err_max_mm, lock.err_max_mm, lock.ring_err_mm,
                            lock.force_peak_n / 1e3, p.energy_change_percent, jack.finite && lock.finite && p.finite ? "" : "  DIVERGED");
            }
        }
    }
}

TEST_CASE("Coupling diag limits", "[.][diag][machine][vehicle]") {
    print_header();
    for (const real mass : {1500.0, 7500.0}) {
        const std::string m = std::to_string(static_cast<int>(mass));
        for (const char* name : {"ball", "drawbar_pin"}) {
            Cfg c;
            c.trailer_mass = mass;
            c.hitch = load_class(name);
            print_row(std::string(name) + " " + m + " reverse", scenario_reverse_jackknife(c));
            print_row(std::string(name) + " " + m + " brake+lock", scenario_brake_and_lock(c));
        }
        Cfg free_case;
        free_case.trailer_mass = mass;
        print_row("free " + m + " reverse", scenario_reverse_jackknife(free_case));
        print_row("free " + m + " brake+lock", scenario_brake_and_lock(free_case));
        Cfg overlap = free_case;
        overlap.drawbar_overlap = true;
        print_row("free overlap " + m + " brake+lock", scenario_brake_and_lock(overlap));
    }
}

TEST_CASE("Coupling diag chain", "[.][diag][machine][vehicle]") {
    std::printf("  %-34s %8s %8s %8s %8s %7s\n", "case", "max mm", "max2 mm", "ring mm", "F kN", "art deg");
    const auto row = [](const std::string& label, const Metrics& m) {
        std::printf("  %-34s %8.3f %8.3f %8.4f %8.2f %7.1f%s\n", label.c_str(), m.err_max_mm, m.err2_max_mm, m.ring_err_mm, m.force_peak_n / 1e3, m.artic_max_deg, m.finite ? "" : "  DIVERGED");
    };
    for (const real mass : {1500.0, 7500.0}) {
        for (const int passes : {1, 2, 4, 8}) {
            Cfg c;
            c.tractor = mass > 3000.0;
            c.trailer_mass = mass;
            c.trailer_brakes = true;
            c.hitch = load_class("drawbar_pin");
            c.passes = passes;
            const std::string label = std::to_string(static_cast<int>(mass)) + " x2 passes " + std::to_string(passes);
            row(label + " rest", scenario_chain_rest(c));
            row(label + " brake", scenario_chain_brake(c));
            row(label + " lane", scenario_chain_lane_change(c, 60.0 / 3.6));
            row(label + " 0.5 m/s", scenario_chain_manoeuvre(c));
        }
    }
}

TEST_CASE("Coupling diag energy", "[.][diag][machine][vehicle]") {
    for (const Method method : {Method::None, Method::Spring}) {
        for (const real mass : {1500.0, 7500.0}) {
            if (method == Method::None && mass < 3000.0) continue; // the single axle cannot stand alone
            Cfg c;
            c.method = method;
            c.trailer_mass = mass;
            const CoastResult r = scenario_coast(c);
            std::printf("  coast %-10s m %5.0f: lost %.4f %% in 30 s, hitch max %.3f mm, dv %.2e / %.2e%s\n", method == Method::None ? "none" : "hitch", mass, r.loss_percent, r.m.err_max_mm, r.m.dv_a,
                        r.m.dv_b, r.m.finite ? "" : "  DIVERGED");
        }
    }
    const auto pendulum = [](const char* label, const Cfg& c) {
        const PendulumResult p = scenario_pendulum(c);
        std::printf("  swing %-26s end %+8.3f %% worst %+8.3f %% of m g L, hitch max %.3f mm%s\n", label, p.energy_change_percent, p.worst_percent, p.err_max_mm, p.finite ? "" : "  DIVERGED");
    };
    for (const real f : {5.0, 10.0, 20.0, 30.0, 40.0, 60.0}) {
        for (const real zeta : {0.0, 0.3, 1.0}) {
            Cfg c;
            c.frequency_hz = f;
            c.damping_ratio = zeta;
            char label[64];
            std::snprintf(label, sizeof(label), "f %.0f z %.1f", f, zeta);
            pendulum(label, c);
        }
    }
    for (const auto& [tick, sub] : {std::pair<real, real>{240.0, 240.0}, {240.0, 960.0}, {240.0, 3840.0}, {480.0, 1920.0}, {960.0, 960.0}, {960.0, 3840.0}}) {
        Cfg c;
        c.tick_hz = tick;
        c.substep_hz = sub;
        char label[64];
        std::snprintf(label, sizeof(label), "%.0f/%.0f", tick, sub);
        pendulum(label, c);
    }
}
