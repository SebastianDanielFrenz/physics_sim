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
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"
#include "vehicle_test_helpers.h"

#include "machine/coupling_solver.h"

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

void add_link(Rig& r) {
    const Cfg& c = r.cfg;
    if (c.method != Method::Spring) return;
    machine::CouplingDesc d;
    d.towing = r.va;
    d.towed = r.vb;
    d.socket_towing_local = r.socket_a;
    d.socket_towed_local = r.socket_b;
    d.hitch = c.hitch;
    d.hitch.frequency_hz = c.frequency_hz;
    d.hitch.damping_ratio = c.damping_ratio;
    r.coupling = r.world->create_coupling(d);
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

    r.va = r.world->create_vehicle(towing, r.a);
    r.vb = r.world->create_vehicle(trailer.desc, r.b);
    add_link(r);

    for (const char* name : {"assist.auto_clutch", "assist.auto_blip", "assist.auto_shift"}) r.world->set_control(name, 0.0);
    if (!cfg.tractor) r.world->set_control("clutch", 1.0);
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
    r.coupling2 = r.world->create_coupling(d);
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
