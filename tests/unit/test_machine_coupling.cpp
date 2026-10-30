// tests/unit/test_machine_coupling.cpp — attachment spike A0: how two vehicles
// are coupled at a hitch.
//
// Candidate A (Method::Spring): an implicit socket spring-damper solved per
// substep with both shadow chassis stepped together in one island
// (core/src/machine/coupling_solver.h, vehicle::System::step_island).
// Candidate B (Method::JoltPoint, JoltHinge): a Jolt constraint between the two
// chassis bodies at the tick rate; the wheels of both vehicles never see it
// inside the tick.
//
// The hidden [.][diag] cases print the tables the decision rests on; every
// number is measured on Jolt's own poses at the tick boundary, the same way for
// every method. The normal cases pin the recommended method.

#include "ps/backend/i_rigid_backend.h"
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

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace ps;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real k100KmH = 100.0 / 3.6;

enum class Method { None, Spring, JoltPoint, JoltHinge };

const char* method_name(Method m) {
    switch (m) {
        case Method::None: return "none";
        case Method::Spring: return "A spring";
        case Method::JoltPoint: return "B point";
        case Method::JoltHinge: return "B hinge";
    }
    return "?";
}

struct Cfg {
    Method method = Method::Spring;
    real frequency_hz = 60.0; // the recommendation, see "A0 diag hard"
    real damping_ratio = 1.0;
    real stiffness = 0.0; // used when frequency_hz == 0
    real damping = 0.0;
    real tick_hz = 240.0, substep_hz = 960.0;
    int workers = 4;
    int collision_steps = 1;
    real trailer_mass = 1500.0;
    bool trailer_brakes = false;
    bool tractor = false;
    bool pose_write_back = false; // spike experiment: after each tick the chassis takes its shadow's end pose
    bool coast_shell = false; // both vehicles lossless: no powertrain, no rolling resistance, no drag, a box ground
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
    if (c.method == Method::Spring) {
        machine::CouplingDesc d;
        d.towing = r.va;
        d.towed = r.vb;
        d.socket_towing_local = r.socket_a;
        d.socket_towed_local = r.socket_b;
        d.frequency_hz = c.frequency_hz;
        d.damping_ratio = c.damping_ratio;
        d.stiffness_n_m = c.stiffness;
        d.damping_n_s_m = c.damping;
        r.coupling = r.world->create_coupling(d);
    } else if (c.method == Method::JoltPoint || c.method == Method::JoltHinge) {
        JointDesc j;
        j.type = c.method == Method::JoltPoint ? JointType::Point : JointType::Hinge;
        j.body_a = r.a;
        j.body_b = r.b;
        j.anchor_a_local = r.socket_a;
        j.anchor_b_local = r.socket_b;
        j.axis_a_local = Vec3::unit_z();
        j.axis_b_local = Vec3::unit_z();
        r.world->create_joint(j);
    }
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

real articulation_deg(const Rig& r) {
    const Vec3 fa = r.world->get_pose(r.a).transform_direction(Vec3::unit_x());
    const Vec3 fb = r.world->get_pose(r.b).transform_direction(Vec3::unit_x());
    return std::atan2(fa.x * fb.y - fa.y * fb.x, fa.x * fb.x + fa.y * fb.y) * 180.0 / kPi;
}

struct Metrics {
    real err_max_mm = 0.0, err_mean_mm = 0.0, err_final_mm = 0.0;
    real ring_err_mm = 0.0;  // period-2 amplitude of the hitch error, 1/4 |e[k] - 2 e[k-1] + e[k-2]|
    real ring_accel = 0.0;   // the same on the trailer's tick-to-tick acceleration, m/s^2
    real dv_a = 0.0, dv_b = 0.0; // worst shadow_dv_error
    real force_peak_n = 0.0;     // Spring only
    real artic_max_deg = 0.0;
    real artic_end_deg = 0.0;    // lane change: left at the end of the run (sway decays or grows)
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
        if (r.cfg.pose_write_back) {
            r.world->backend().set_pose(r.a, r.world->vehicle_shadow_end_pose(r.va));
            r.world->backend().set_pose(r.b, r.world->vehicle_shadow_end_pose(r.vb));
        }
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
// Above about 60 Hz the shadow closes the error inside a tick and ends the tick at rest, so Jolt's position, the only
// one that survives the tick, hardly moves. This is what a very stiff hitch costs.
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
    if (c.method == Method::JoltHinge) {
        JointDesc j;
        j.type = JointType::Hinge;
        j.body_a = r.a;
        j.body_b = r.b;
        j.anchor_a_local = r.socket_a;
        j.anchor_b_local = r.socket_b;
        j.axis_a_local = Vec3::unit_y();
        j.axis_b_local = Vec3::unit_y();
        r.world->create_joint(j);
    } else {
        add_link(r);
    }

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
        if (c.pose_write_back) {
            r.world->backend().set_pose(r.a, r.world->vehicle_shadow_end_pose(r.va));
            r.world->backend().set_pose(r.b, r.world->vehicle_shadow_end_pose(r.vb));
        }
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

real reduced_mass(real a, real b) { return a * b / (a + b); }

// A lane-change speed the combination survives. "A0 diag sway": the 7500 kg trailer behind a 1500 kg car is stable at
// 60 km/h and jackknifes at 80 km/h with every hitch method, so above that the rows measure box contact, not the hitch.
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

TEST_CASE("A0: a vehicle without engine, driven wheels or wheels at all builds and rolls", "[machine][vehicle]") {
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

TEST_CASE("A0: the socket solve satisfies its implicit equation at any mass ratio", "[machine][vehicle]") {
    machine::SocketBody a;
    a.linear = Vec3{10.0, 0.5, -0.1};
    a.angular = Vec3{0.1, -0.2, 0.3};
    a.socket_offset = Vec3{-2.5, 0.2, -0.2};
    a.socket_position = Vec3{1.0, 2.0, 0.4};
    a.inv_mass = 1.0 / 1500.0;
    a.inv_inertia_world = Mat3::diagonal(1.0 / 500.0, 1.0 / 1800.0, 1.0 / 2000.0);
    machine::SocketBody b = a;
    b.linear = Vec3{9.7, 0.2, 0.05};
    b.angular = Vec3{-0.05, 0.1, -0.2};
    b.socket_offset = Vec3{3.0, -0.1, -0.4};
    b.socket_position = Vec3{1.002, 2.001, 0.399};
    const real h = 1.0 / 960.0;

    for (const real mass_b : {1500.0, 7500.0, 40000.0}) {
        b.inv_mass = 1.0 / mass_b;
        b.inv_inertia_world = Mat3::diagonal(10.0 / (mass_b * 4.0), 10.0 / (mass_b * 9.0), 10.0 / (mass_b * 10.0));
        for (const real k : {1e6, 1e8, 1e10}) {
            machine::CouplingDesc d;
            d.stiffness_n_m = k;
            d.damping_n_s_m = 2.0 * std::sqrt(k * 1000.0);
            const machine::SocketSolve s = machine::solve_socket_coupling(a, b, d, h);
            // Applied to the bodies directly, not through the solver's own W.
            const Vec3 fa = s.force_on_a, fb = -s.force_on_a;
            const Vec3 va = a.linear + fa * (h * a.inv_mass);
            const Vec3 wa = a.angular + a.inv_inertia_world * cross(a.socket_offset, fa) * h;
            const Vec3 vb = b.linear + fb * (h * b.inv_mass);
            const Vec3 wb = b.angular + b.inv_inertia_world * cross(b.socket_offset, fb) * h;
            const Vec3 edot = (va + cross(wa, a.socket_offset)) - (vb + cross(wb, b.socket_offset));
            const Vec3 residual = fa + (s.error + edot * h) * d.stiffness_n_m + edot * d.damping_n_s_m;
            CHECK(residual.length() <= 1e-6 * std::max(real(1.0), fa.length()));
        }
        // The mass-normalised form: one frequency, the socket's error response does not depend on the masses.
        machine::CouplingDesc d;
        d.frequency_hz = 60.0;
        const machine::SocketSolve s = machine::solve_socket_coupling(a, b, d, h);
        const real omega = 2.0 * kPi * 60.0, g = h * omega * omega + 2.0 * omega;
        const Vec3 fa = s.force_on_a;
        const Vec3 va = a.linear + fa * (h * a.inv_mass);
        const Vec3 wa = a.angular + a.inv_inertia_world * cross(a.socket_offset, fa) * h;
        const Vec3 vb = b.linear - fa * (h * b.inv_mass);
        const Vec3 wb = b.angular - b.inv_inertia_world * cross(b.socket_offset, fa) * h;
        const Vec3 edot = (va + cross(wa, a.socket_offset)) - (vb + cross(wb, b.socket_offset));
        const Vec3 edot_free = (a.linear + cross(a.angular, a.socket_offset)) - (b.linear + cross(b.angular, b.socket_offset));
        const Vec3 expected = edot_free - (s.error * (omega * omega) + edot_free * g) * (h / (1.0 + h * g));
        CHECK((edot - expected).length() <= 1e-9);
    }
}

TEST_CASE("A0: a car holds a 5x trailer within millimetres, without tick ringing, and the shadows see the force", "[machine][vehicle]") {
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
    CHECK(brake.dv_a < 1e-5); // a Jolt constraint: 4e-2
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

TEST_CASE("A0: a position error closes within tenths of a second, the reason not to go stiffer", "[machine][vehicle]") {
    Cfg c;
    c.trailer_mass = 7500.0;
    const RecoveryResult good = scenario_recovery(c);
    CHECK(good.seconds_to_1mm > 0.0);
    CHECK(good.seconds_to_1mm < 0.05);
    // Far too stiff for the tick: the shadow closes the error inside a tick, ends it at rest, and Jolt's position, the
    // only one that survives the tick, hardly moves.
    c.frequency_hz = 480.0;
    const RecoveryResult stiff = scenario_recovery(c);
    CHECK(stiff.seconds_to_1mm > 2.0 * good.seconds_to_1mm);
}

TEST_CASE("A0: a jackknife stays finite and a slow manoeuvre stays quiet", "[machine][vehicle]") {
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

TEST_CASE("A0: a coupled pair is worker-count independent and allocates nothing in the tick", "[machine][vehicle]") {
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
// Measurements. Run with: ps_unit_tests "A0 diag*"
// ---------------------------------------------------------------------------------------------------------------------

TEST_CASE("A0 diag table", "[.][diag][machine][vehicle]") {
    print_header();
    for (const real mass : {1500.0, 7500.0}) {
        for (const Method method : {Method::Spring, Method::JoltPoint, Method::JoltHinge}) {
            Cfg c;
            c.method = method;
            c.trailer_mass = mass;
            print_scenarios(c, std::string(method_name(method)) + " " + std::to_string(static_cast<int>(mass)));
            c.trailer_brakes = true;
            print_row(std::string(method_name(method)) + " " + std::to_string(static_cast<int>(mass)) + " brake+trl brakes", scenario_brake(c));
            c.trailer_brakes = false;
            print_row(std::string(method_name(method)) + " " + std::to_string(static_cast<int>(mass)) + " reverse jackknife", scenario_reverse_jackknife(c));
            print_row(std::string(method_name(method)) + " " + std::to_string(static_cast<int>(mass)) + " brake+lock", scenario_brake_and_lock(c));
            print_row(std::string(method_name(method)) + " " + std::to_string(static_cast<int>(mass)) + " tractor 15 kN", scenario_tractor_draft(c, 15000.0));
            print_row(std::string(method_name(method)) + " " + std::to_string(static_cast<int>(mass)) + " tractor after 2 s", scenario_tractor_draft(c, 15000.0, 2.0));
        }
    }
}

TEST_CASE("A0 diag sway", "[.][diag][machine][vehicle]") {
    // Is the 5x trailer stable behind this car at all? The same with every method means it is the vehicle, not the hitch.
    for (const real mass : {1500.0, 3000.0, 7500.0}) {
        for (const real kmh : {40.0, 60.0, 80.0, 100.0, 120.0}) {
            for (const Method method : {Method::Spring, Method::JoltPoint}) {
                Cfg c;
                c.method = method;
                c.trailer_mass = mass;
                const Metrics m = scenario_lane_change(c, kmh / 3.6);
                std::printf("  %-9s m %5.0f %3.0f km/h: art max %6.1f end %6.1f deg, ay %6.2f, hitch max %7.3f mm, ring %.4f mm, dv %.2e\n", method_name(method), mass, kmh, m.artic_max_deg,
                            m.artic_end_deg, m.ay_peak, m.err_max_mm, m.ring_err_mm, m.dv_a);
            }
        }
    }
}

TEST_CASE("A0 diag stiffness", "[.][diag][machine][vehicle]") {
    print_header();
    for (const real mass : {1500.0, 7500.0}) {
        for (const real k : {1e5, 1e6, 1e7, 3e7, 1e8, 1e9, 1e10}) {
            Cfg c;
            c.frequency_hz = 0.0;
            c.trailer_mass = mass;
            c.stiffness = k;
            c.damping = 2.0 * std::sqrt(k * reduced_mass(1500.0, mass));
            char label[64];
            std::snprintf(label, sizeof(label), "m %.0f k %.0e", mass, k);
            print_row(std::string(label) + " brake", scenario_brake(c));
            print_row(std::string(label) + " lane", scenario_lane_change(c, stable_speed(mass)));
        }
        for (const real f : {2.0, 5.0, 10.0, 20.0, 30.0, 40.0, 60.0, 80.0, 120.0, 240.0, 480.0}) {
            for (const real zeta : {0.3, 1.0, 3.0}) {
                Cfg c;
                c.frequency_hz = f;
                c.damping_ratio = zeta;
                c.trailer_mass = mass;
                char label[64];
                std::snprintf(label, sizeof(label), "m %.0f f %.0f z %.1f", mass, f, zeta);
                print_row(std::string(label) + " brake", scenario_brake(c));
                print_row(std::string(label) + " lane", scenario_lane_change(c, stable_speed(mass)));
            }
        }
    }
}

TEST_CASE("A0 diag hard", "[.][diag][machine][vehicle]") {
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

TEST_CASE("A0 diag rates", "[.][diag][machine][vehicle]") {
    print_header();
    const std::pair<real, real> rates[] = {{240.0, 240.0}, {240.0, 960.0}, {240.0, 1920.0}, {480.0, 960.0}, {960.0, 960.0}, {960.0, 3840.0}};
    for (const Method method : {Method::Spring, Method::JoltPoint}) {
        for (const auto& [tick, sub] : rates) {
            Cfg c;
            c.method = method;
            c.trailer_mass = 7500.0;
            c.tick_hz = tick;
            c.substep_hz = sub;
            char label[64];
            std::snprintf(label, sizeof(label), "%s %.0f/%.0f", method_name(method), tick, sub);
            print_row(std::string(label) + " brake", scenario_brake(c));
            print_row(std::string(label) + " lane", scenario_lane_change(c, stable_speed(c.trailer_mass)));
        }
    }
    for (const int steps : {1, 2, 4}) {
        Cfg c;
        c.method = Method::JoltPoint;
        c.trailer_mass = 7500.0;
        c.collision_steps = steps;
        print_row("B point collision_steps " + std::to_string(steps) + " lane", scenario_lane_change(c, stable_speed(c.trailer_mass)));
    }
}

TEST_CASE("A0 diag energy", "[.][diag][machine][vehicle]") {
    for (const Method method : {Method::None, Method::Spring, Method::JoltPoint, Method::JoltHinge}) {
        for (const real mass : {1500.0, 7500.0}) {
            if (method == Method::None && mass < 3000.0) continue; // the single axle cannot stand alone
            Cfg c;
            c.method = method;
            c.trailer_mass = mass;
            const CoastResult r = scenario_coast(c);
            std::printf("  coast %-10s m %5.0f: lost %.4f %% in 30 s, hitch max %.3f mm, dv %.2e / %.2e%s\n", method_name(method), mass, r.loss_percent, r.m.err_max_mm, r.m.dv_a, r.m.dv_b,
                        r.m.finite ? "" : "  DIVERGED");
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
            std::snprintf(label, sizeof(label), "A f %.0f z %.1f", f, zeta);
            pendulum(label, c);
        }
    }
    for (const auto& [tick, sub] : {std::pair<real, real>{240.0, 240.0}, {240.0, 960.0}, {240.0, 3840.0}, {480.0, 1920.0}, {960.0, 960.0}, {960.0, 3840.0}}) {
        Cfg c;
        c.tick_hz = tick;
        c.substep_hz = sub;
        char label[64];
        std::snprintf(label, sizeof(label), "A %.0f/%.0f", tick, sub);
        pendulum(label, c);
    }
    Cfg wb;
    wb.pose_write_back = true;
    pendulum("A + pose write-back", wb);
    print_header();
    print_row("A write-back 1500 lane", scenario_lane_change(wb, stable_speed(wb.trailer_mass)));
    print_row("A write-back 1500 brake", scenario_brake(wb));
    wb.trailer_mass = 7500.0;
    print_row("A write-back 7500 lane", scenario_lane_change(wb, stable_speed(wb.trailer_mass)));
    print_row("A write-back 7500 brake", scenario_brake(wb));
    wb.trailer_mass = 1500.0;
    wb.substep_hz = 3840.0;
    pendulum("A + write-back 240/3840", wb);
    Cfg b;
    b.method = Method::JoltHinge;
    pendulum("B hinge", b);
    b.method = Method::JoltPoint;
    pendulum("B point", b);
    b.collision_steps = 4;
    pendulum("B point collision_steps 4", b);
    b.collision_steps = 1;
    b.tick_hz = 960.0;
    pendulum("B point 960 Hz tick", b);
}

TEST_CASE("A0 diag cost hash alloc", "[.][diag][machine][vehicle]") {
    const auto cost = [](Method method, int workers) {
        Cfg c;
        c.method = method;
        c.trailer_mass = 7500.0;
        c.workers = workers;
        Rig r = build_rig(c, -700.0);
        settle(r, 0.5);
        launch(r, stable_speed(c.trailer_mass));
        ps_test::settle(*r.world, 0.5);
        double best = 1e30;
        for (int run_index = 0; run_index < 8; ++run_index) {
            const auto t0 = std::chrono::steady_clock::now();
            const int ticks = 600;
            for (int i = 0; i < ticks; ++i) {
                r.world->set_control("steer", 0.02 * std::sin(2.0 * kPi * static_cast<real>(i) / 720.0));
                r.world->step();
            }
            const std::chrono::duration<double, std::micro> us = std::chrono::steady_clock::now() - t0;
            best = std::min(best, us.count() / ticks); // best of 8: the machine is shared
        }
        return best;
    };
    for (const Method method : {Method::None, Method::Spring, Method::JoltPoint, Method::JoltHinge}) {
        for (const int workers : {1, 4}) {
            std::printf("  cost %-10s %2d workers: %.1f us/tick\n", method_name(method), workers, cost(method, workers));
        }
    }
    {
        machine::SocketBody a, b;
        a.inv_mass = 1.0 / 1500.0;
        a.inv_inertia_world = Mat3::diagonal(1.0 / 500.0, 1.0 / 1800.0, 1.0 / 2000.0);
        a.socket_offset = Vec3{-2.5, 0.0, -0.2};
        b = a;
        b.inv_mass = 1.0 / 7500.0;
        machine::CouplingDesc d;
        d.frequency_hz = 60.0;
        const int count = 10000000;
        real sink = 0.0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < count; ++i) {
            a.linear.x = static_cast<real>(i) * 1e-7;
            sink += machine::solve_socket_coupling(a, b, d, 1.0 / 960.0).force_on_a.x;
        }
        const std::chrono::duration<double, std::nano> ns = std::chrono::steady_clock::now() - t0;
        std::printf("  solve_socket_coupling: %.1f ns per call (sink %.3g)\n", ns.count() / count, sink);
    }
    for (const Method method : {Method::None, Method::Spring, Method::JoltPoint, Method::JoltHinge}) {
        const auto hashes = [method](int workers) {
            Cfg c;
            c.method = method;
            c.trailer_mass = 7500.0;
            c.workers = workers;
            Rig r = build_rig(c, -700.0);
            settle(r, 0.5);
            launch(r, k100KmH);
            std::vector<std::uint64_t> out;
            for (int i = 0; i < 960; ++i) {
                r.world->set_control("steer", 0.03 * std::sin(2.0 * kPi * static_cast<real>(i) / 480.0));
                r.world->step();
                out.push_back(r.world->state_hash());
            }
            return out;
        };
        const auto one = hashes(1), ten = hashes(10);
        std::size_t first_diff = one.size();
        for (std::size_t i = 0; i < one.size(); ++i) {
            if (one[i] != ten[i]) {
                first_diff = i;
                break;
            }
        }
        std::printf("  hash %-10s 1 vs 10 workers: %s (end %016llx)\n", method_name(method), first_diff == one.size() ? "equal over 960 ticks" : ("differ at " + std::to_string(first_diff)).c_str(),
                    static_cast<unsigned long long>(one.back()));

        if (!jobs::debug_alloc::hook_available()) continue;
        Cfg c;
        c.method = method;
        c.trailer_mass = 7500.0; // the tandem stands without a hitch too, so Method::None is a fair baseline
        Rig r = build_rig(c, -700.0);
        settle(r, 0.5);
        launch(r, stable_speed(c.trailer_mass));
        ps_test::settle(*r.world, 0.5);
        jobs::debug_alloc::reset();
        jobs::debug_alloc::set_armed(true);
        for (int i = 0; i < 960; ++i) {
            r.world->set_control("steer", 0.03 * std::sin(2.0 * kPi * static_cast<real>(i) / 480.0));
            r.world->step();
        }
        jobs::debug_alloc::set_armed(false);
        std::printf("  alloc %-10s own %llu external %llu (external = Jolt's QuadTree gap)\n", method_name(method),
                    static_cast<unsigned long long>(jobs::debug_alloc::own_violation_count()), static_cast<unsigned long long>(jobs::debug_alloc::external_violation_count()));
    }
}
