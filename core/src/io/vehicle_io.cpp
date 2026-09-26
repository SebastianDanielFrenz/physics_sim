#include "ps/io/vehicle_io.h"

#include "../vehicle/tir_parser.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;
using ps::vehicle::Curve1D;
using ps::vehicle::TyreDesc;
using ps::vehicle::VehicleDesc;
using ps::vehicle::WheelDesc;
using ps::vehicle::SuspensionDesc;
using ps::vehicle::SteeringDesc;
using ps::drivetrain::BrakeSystemDesc;
using ps::drivetrain::PowertrainDesc;

json read_json_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    json j;
    try {
        in >> j;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error("malformed JSON in " + path + ": " + e.what());
    }
    return j;
}

Vec3 read_vec3(const json& j, const Vec3& def) {
    if (j.is_null()) return def;
    if (!j.is_array() || j.size() != 3) throw std::runtime_error("expected a 3-element array for a Vec3");
    return Vec3{j[0].get<real>(), j[1].get<real>(), j[2].get<real>()};
}

Curve1D read_curve(const json& j) {
    Curve1D c;
    if (j.is_null()) return c;
    if (!j.contains("x") || !j.contains("y")) throw std::runtime_error("curve object needs 'x' and 'y' arrays");
    for (const auto& v : j.at("x")) c.x.push_back(v.get<real>());
    for (const auto& v : j.at("y")) c.y.push_back(v.get<real>());
    if (c.x.size() != c.y.size()) throw std::runtime_error("curve 'x'/'y' arrays must be the same length");
    return c;
}

std::string resolve_path(const std::string& base_dir, const std::string& p) {
    std::filesystem::path fp(p);
    if (fp.is_absolute()) return p;
    return (std::filesystem::path(base_dir) / fp).string();
}

// PHYS-033 P4 I3 plan section 9.1 (owner ruling Q7: NO legacy support -
// missing, unknown or outdated is a hard error, never a silent default).
// `kind` is e.g. "vehicle", "tyre", "engine" or "gearbox"; `major` the only
// accepted major version. Every standalone JSON file this loader reads goes
// through this check exactly once, right after parsing.
void require_format(const json& j, const std::string& kind, int major, const std::string& path) {
    const std::string expected = "physics_sim." + kind + "/" + std::to_string(major);
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) {
        throw std::runtime_error(path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
    }
}

// PHYS-033 P4 I2 plan section 9.1: a /1 vehicle file must fail with a
// pointer to the /2 layout, not just the generic require_format message.
// Deliberately does NOT wrap require_format() in a try/catch that rethrows
// off e.what() (an earlier version did; it access-violated under the asan
// preset - std::runtime_error's storage is not guaranteed to survive a
// rethrow-with-appended-text the way that pattern assumed) - duplicates
// its small amount of logic instead.
void require_vehicle_format(const json& j, const std::string& path) {
    const std::string expected = "physics_sim.vehicle/2";
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) {
        std::string msg = path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")";
        if (actual == "physics_sim.vehicle/1") {
            msg += " (physics_sim.vehicle/1 is no longer supported - PHYS-033 Q7; "
                   "see data/vehicles/car_sedan.json for the /2 layout)";
        }
        throw std::runtime_error(msg);
    }
}

TyreDesc read_tyre_object(const json& j) {
    TyreDesc d;
    d.name = j.value("name", std::string());
    d.source = j.value("source", std::string());
    if (d.source.empty()) throw std::runtime_error("tyre '" + d.name + "': 'source' is required (PLAN.md section 8)");

    d.unloaded_radius = j.value("unloaded_radius", d.unloaded_radius);
    d.width = j.value("width", d.width);
    d.rim_radius = j.value("rim_radius", d.rim_radius);
    d.fz0 = j.value("fz0", d.fz0);
    d.vertical_stiffness = j.value("vertical_stiffness", d.vertical_stiffness);
    d.v0 = j.value("v0", d.v0);
    d.qsy1 = j.value("qsy1", d.qsy1);
    d.qsy2 = j.value("qsy2", d.qsy2);
    d.qsy4 = j.value("qsy4", d.qsy4);
    d.rolling_resistance_v_eps = j.value("rolling_resistance_v_eps", d.rolling_resistance_v_eps);
    if (d.rolling_resistance_v_eps <= real(0.0)) {
        throw std::runtime_error("tyre '" + d.name + "': rolling_resistance_v_eps must be > 0, got " +
                                  std::to_string(d.rolling_resistance_v_eps));
    }
    d.relaxation_length_x = j.value("relaxation_length_x", d.relaxation_length_x);
    d.relaxation_length_y = j.value("relaxation_length_y", d.relaxation_length_y);
    d.lambda_mux = j.value("lambda_mux", d.lambda_mux);
    d.lambda_muy = j.value("lambda_muy", d.lambda_muy);

    // PHYS-033 I1: additive fields, tyre/1 unchanged.
    d.low_speed_damping_ratio = j.value("low_speed_damping_ratio", d.low_speed_damping_ratio);
    d.low_speed_damping_vx = j.value("low_speed_damping_vx", d.low_speed_damping_vx);
    if (d.low_speed_damping_ratio < real(0.0) || d.low_speed_damping_ratio > real(2.0)) {
        throw std::runtime_error("tyre '" + d.name + "': low_speed_damping_ratio must be in [0, 2], got " +
                                  std::to_string(d.low_speed_damping_ratio));
    }
    if (d.low_speed_damping_vx <= real(0.0)) {
        throw std::runtime_error("tyre '" + d.name + "': low_speed_damping_vx must be > 0, got " +
                                  std::to_string(d.low_speed_damping_vx));
    }

    if (j.contains("longitudinal")) {
        const auto& g = j.at("longitudinal");
        d.pcx1 = g.value("pcx1", d.pcx1);
        d.pdx1 = g.value("pdx1", d.pdx1); d.pdx2 = g.value("pdx2", d.pdx2); d.pdx3 = g.value("pdx3", d.pdx3);
        d.pex1 = g.value("pex1", d.pex1); d.pex2 = g.value("pex2", d.pex2); d.pex3 = g.value("pex3", d.pex3); d.pex4 = g.value("pex4", d.pex4);
        d.pkx1 = g.value("pkx1", d.pkx1); d.pkx2 = g.value("pkx2", d.pkx2); d.pkx3 = g.value("pkx3", d.pkx3);
        d.phx1 = g.value("phx1", d.phx1); d.phx2 = g.value("phx2", d.phx2);
        d.pvx1 = g.value("pvx1", d.pvx1); d.pvx2 = g.value("pvx2", d.pvx2);
    }
    if (j.contains("lateral")) {
        const auto& g = j.at("lateral");
        d.pcy1 = g.value("pcy1", d.pcy1);
        d.pdy1 = g.value("pdy1", d.pdy1); d.pdy2 = g.value("pdy2", d.pdy2); d.pdy3 = g.value("pdy3", d.pdy3);
        d.pey1 = g.value("pey1", d.pey1); d.pey2 = g.value("pey2", d.pey2); d.pey3 = g.value("pey3", d.pey3); d.pey4 = g.value("pey4", d.pey4);
        d.pky1 = g.value("pky1", d.pky1); d.pky2 = g.value("pky2", d.pky2);
        d.phy1 = g.value("phy1", d.phy1); d.phy2 = g.value("phy2", d.phy2); d.phy3 = g.value("phy3", d.phy3);
        d.pvy1 = g.value("pvy1", d.pvy1); d.pvy2 = g.value("pvy2", d.pvy2); d.pvy3 = g.value("pvy3", d.pvy3); d.pvy4 = g.value("pvy4", d.pvy4);
    }
    if (j.contains("combined")) {
        const auto& g = j.at("combined");
        d.rbx1 = g.value("rbx1", d.rbx1); d.rbx2 = g.value("rbx2", d.rbx2); d.rcx1 = g.value("rcx1", d.rcx1); d.rhx1 = g.value("rhx1", d.rhx1);
        d.rby1 = g.value("rby1", d.rby1); d.rby2 = g.value("rby2", d.rby2); d.rby3 = g.value("rby3", d.rby3); d.rcy1 = g.value("rcy1", d.rcy1);
        d.rhy1 = g.value("rhy1", d.rhy1); d.rhy2 = g.value("rhy2", d.rhy2);
        d.rvy1 = g.value("rvy1", d.rvy1); d.rvy2 = g.value("rvy2", d.rvy2); d.rvy3 = g.value("rvy3", d.rvy3);
        d.rvy4 = g.value("rvy4", d.rvy4); d.rvy5 = g.value("rvy5", d.rvy5); d.rvy6 = g.value("rvy6", d.rvy6);
    }
    if (j.contains("aligning")) {
        const auto& g = j.at("aligning");
        d.qbz1 = g.value("qbz1", d.qbz1); d.qbz2 = g.value("qbz2", d.qbz2); d.qbz3 = g.value("qbz3", d.qbz3);
        d.qcz1 = g.value("qcz1", d.qcz1);
        d.qdz1 = g.value("qdz1", d.qdz1); d.qdz2 = g.value("qdz2", d.qdz2);
        d.qez1 = g.value("qez1", d.qez1); d.qez2 = g.value("qez2", d.qez2);
        d.qhz1 = g.value("qhz1", d.qhz1); d.qhz2 = g.value("qhz2", d.qhz2); d.qhz3 = g.value("qhz3", d.qhz3);
    }
    if (j.contains("overturning")) {
        const auto& g = j.at("overturning");
        d.qsx1 = g.value("qsx1", d.qsx1); d.qsx2 = g.value("qsx2", d.qsx2); d.qsx3 = g.value("qsx3", d.qsx3);
    }
    return d;
}

TyreDesc read_tyre_ref(const json& j, const std::string& base_dir) {
    if (j.is_string()) {
        const std::string path = resolve_path(base_dir, j.get<std::string>());
        if (path.size() >= 4 && path.substr(path.size() - 4) == ".tir") {
            return ps::vehicle::parse_tir_file(path, "Parsed from " + path);
        }
        // A standalone tyre JSON file - carries its own "format", unlike an
        // inline object (below), which has no independent version of its
        // own and inherits its enclosing vehicle file's format instead.
        const json tj = read_json_file(path);
        require_format(tj, "tyre", 1, path);
        return read_tyre_object(tj);
    }
    return read_tyre_object(j);
}

SuspensionDesc read_suspension(const json& j) {
    SuspensionDesc s;
    s.axis = read_vec3(j.value("axis", json()), s.axis);
    s.travel_max = j.value("travel_max", s.travel_max);
    s.spring_rate = j.value("spring_rate", s.spring_rate);
    if (j.contains("spring_curve")) s.spring_curve = read_curve(j.at("spring_curve"));
    s.spring_preload = j.value("spring_preload", s.spring_preload);
    s.damper_bump_rate = j.value("damper_bump_rate", s.damper_bump_rate);
    s.damper_rebound_rate = j.value("damper_rebound_rate", s.damper_rebound_rate);
    if (j.contains("damper_curve")) s.damper_curve = read_curve(j.at("damper_curve"));
    s.bump_stop_rate = j.value("bump_stop_rate", s.bump_stop_rate);
    if (j.contains("camber_vs_travel")) s.camber_vs_travel = read_curve(j.at("camber_vs_travel"));
    if (j.contains("toe_vs_travel")) s.toe_vs_travel = read_curve(j.at("toe_vs_travel"));
    s.arb_rate = j.value("arb_rate", s.arb_rate);
    return s;
}

WheelDesc read_wheel(const json& j, const std::string& base_dir) {
    WheelDesc w;
    w.name = j.value("name", std::string("wheel"));
    w.attachment_local = read_vec3(j.value("attachment_local", json()), w.attachment_local);
    w.static_camber = j.value("static_camber", w.static_camber);
    w.static_toe = j.value("static_toe", w.static_toe);
    w.wheel_radius = j.value("wheel_radius", w.wheel_radius);
    w.wheel_width = j.value("wheel_width", w.wheel_width);
    w.wheel_mass = j.value("wheel_mass", w.wheel_mass);
    w.wheel_inertia = j.value("wheel_inertia", w.wheel_inertia);
    if (j.contains("suspension")) w.suspension = read_suspension(j.at("suspension"));
    w.arb_partner = j.value("arb_partner", w.arb_partner);
    if (j.contains("tyre")) w.tyre = read_tyre_ref(j.at("tyre"), base_dir);
    w.steered = j.value("steered", w.steered);
    // PHYS-033 I2 (owner ruling Q7, plan section 9.1): "driven" is a /1
    // field. Which wheels are driven is now determined by the vehicle's
    // "powertrain" graph (which hubs a differential/gear reaches).
    if (j.contains("driven")) {
        throw std::runtime_error("wheel '" + w.name + "': 'driven' is no longer a wheel field "
                                  "(physics_sim.vehicle/2) - which wheels are driven is now determined "
                                  "by the 'powertrain' graph (PHYS-033 Q7)");
    }
    w.is_front = j.value("is_front", w.is_front);
    return w;
}

SteeringDesc read_steering(const json& j) {
    SteeringDesc s;
    if (j.is_null()) return s;
    s.max_wheel_angle = j.value("max_wheel_angle", s.max_wheel_angle);
    s.ackermann_fraction = j.value("ackermann_fraction", s.ackermann_fraction);
    // [-1, 1]: 1 = full geometric Ackermann, -1 = full anti-Ackermann (see
    // vehicle_desc.h's SteeringDesc::ackermann_fraction comment) - the two
    // "full effect" endpoints in either direction. Anything further out
    // overshoots the geometric correction the formula is built from by
    // more than the geometry itself justifies and is far more likely a
    // data-entry mistake (e.g. a stray extra digit, or degrees where
    // radians/a unit fraction was meant) than an intentional setting; no
    // vehicle file in this project needs more than +-1.
    if (s.ackermann_fraction < real(-1.0) || s.ackermann_fraction > real(1.0)) {
        throw std::runtime_error("steering.ackermann_fraction must be in [-1, 1] (1 = full geometric "
                                  "Ackermann, -1 = full anti-Ackermann, 0 = parallel steer), got " +
                                  std::to_string(s.ackermann_fraction));
    }
    s.rear_steer_fraction = j.value("rear_steer_fraction", s.rear_steer_fraction);
    s.steer_rate_limit = j.value("steer_rate_limit", s.steer_rate_limit);
    return s;
}

// PHYS-033 P4 I2 plan section 9.1. Optional (no brakes at all if absent).
BrakeSystemDesc read_brake_system(const json& j) {
    BrakeSystemDesc b;
    if (j.is_null()) return b;
    const std::string type = j.value("type", std::string("direct_torque"));
    if (type != "direct_torque") {
        throw std::runtime_error("brake_system.type must be \"direct_torque\" (I5 adds \"hydraulic\"), got \"" + type + "\"");
    }
    b.type = ps::drivetrain::BrakeSystemType::DirectTorque;
    if (j.contains("corners")) {
        std::vector<std::string> seen;
        for (const auto& cj : j.at("corners")) {
            ps::drivetrain::BrakeCornerDesc c;
            if (!cj.contains("wheel") || !cj.at("wheel").is_string() || cj.at("wheel").get<std::string>().empty()) {
                throw std::runtime_error("brake_system.corners[]: 'wheel' is required and must be a non-empty string");
            }
            c.wheel = cj.at("wheel").get<std::string>();
            if (std::find(seen.begin(), seen.end(), c.wheel) != seen.end()) {
                throw std::runtime_error("brake_system.corners[]: duplicate wheel '" + c.wheel + "'");
            }
            seen.push_back(c.wheel);
            c.max_torque_nm = cj.value("max_torque_nm", real(0.0));
            if (c.max_torque_nm < real(0.0)) {
                throw std::runtime_error("brake_system.corners['" + c.wheel + "'].max_torque_nm must be >= 0");
            }
            b.corners.push_back(c);
        }
    }
    if (j.contains("handbrake")) {
        const auto& hj = j.at("handbrake");
        if (hj.contains("wheels")) {
            for (const auto& wn : hj.at("wheels")) b.handbrake.wheels.push_back(wn.get<std::string>());
        }
        b.handbrake.max_torque_nm = hj.value("max_torque_nm", real(0.0));
        if (b.handbrake.max_torque_nm < real(0.0)) {
            throw std::runtime_error("brake_system.handbrake.max_torque_nm must be >= 0");
        }
    }
    b.static_factor = j.value("static_factor", real(1.0));
    if (b.static_factor < real(1.0)) {
        throw std::runtime_error("brake_system.static_factor must be >= 1, got " + std::to_string(b.static_factor));
    }
    return b;
}

// --- PHYS-033 P4 I3 (plan section 9.1) --------------------------------------

void require_ascending(const Curve1D& c, const std::string& what) {
    if (c.x.empty()) throw std::runtime_error(what + " must be non-empty");
    for (std::size_t i = 1; i < c.x.size(); ++i) {
        if (!(c.x[i] > c.x[i - 1])) throw std::runtime_error(what + ": x values must be strictly ascending");
    }
}

// PHYS-033 P4 I7a C3 review item 4: a pedal_map/linkage_curve must cover the
// WHOLE [0, 1] pedal domain exactly (no clamped extrapolation past an
// interior endpoint - Curve1D's own edge-clamp semantics would otherwise let
// x.front() > 0 or x.back() < 1 silently flatten part of the pedal travel)
// and must be non-decreasing (a throttle command must not go DOWN as the
// pedal goes further down) - each with its own message. Assumes
// require_ascending(c, ...) already ran (strictly ascending x, non-empty).
// y within [0, 1] is checked separately at each call site (pre-existing).
void require_unit_domain_covered_nondecreasing(const Curve1D& c, const std::string& what) {
    if (c.x.front() != real(0.0)) {
        throw std::runtime_error(what + ": x.front() must be exactly 0 (the curve must cover the whole pedal range)");
    }
    if (c.x.back() != real(1.0)) {
        throw std::runtime_error(what + ": x.back() must be exactly 1 (the curve must cover the whole pedal range)");
    }
    for (real y : c.y) {
        if (y < real(0.0) || y > real(1.0)) throw std::runtime_error(what + ": y must be within [0, 1]");
    }
    for (std::size_t i = 1; i < c.y.size(); ++i) {
        if (c.y[i] < c.y[i - 1]) throw std::runtime_error(what + ": y must be non-decreasing");
    }
}

// PHYS-033 P4 I7a C3 (plan section 3.4.5): an engine's effective throttle
// kind is either an explicit throttle_control component naming it (that
// component's own 'kind' wins), or - with none - the engine's own
// TorqueMapEngineDesc::throttle_input (the IMPLICIT default stage,
// core/src/drivetrain/powertrain.cpp's Phase K). engine_id is assumed
// already validated to reference an existing TorqueMapEngineDesc component.
ps::drivetrain::ThrottleInput effective_throttle_kind(const PowertrainDesc& p, const std::string& engine_id) {
    for (const auto& c : p.components) {
        if (const auto* tcd = std::get_if<ps::drivetrain::ThrottleControlDesc>(&c.params)) {
            if (tcd->engine == engine_id) return tcd->kind;
        }
    }
    for (const auto& c : p.components) {
        if (c.id == engine_id) {
            if (const auto* e = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) return e->throttle_input;
        }
    }
    return ps::drivetrain::ThrottleInput::Electronic; // unreachable for a validated engine reference
}

// Zero or unit length (+-1e-6) - plan section 9.1's axis validation.
void require_zero_or_unit(const Vec3& v, const std::string& what) {
    const real len = v.length();
    if (len <= real(1e-6)) return; // zero (== no rotor reaction for this port)
    if (std::abs(len - real(1.0)) > real(1e-6)) {
        throw std::runtime_error(what + " must be zero or a unit vector, got length " + std::to_string(len));
    }
}

ps::drivetrain::TorqueMapEngineDesc read_engine_fields(const json& j, const std::string& ctx) {
    ps::drivetrain::TorqueMapEngineDesc e;
    if (j.value("kind", std::string()) != "torque_map") {
        throw std::runtime_error(ctx + ": engine 'kind' must be \"torque_map\" (I3's only kind)");
    }
    if (!j.contains("wot_torque_nm_vs_rpm")) throw std::runtime_error(ctx + ": 'wot_torque_nm_vs_rpm' is required");
    e.wot_torque_nm_vs_rpm = read_curve(j.at("wot_torque_nm_vs_rpm"));
    require_ascending(e.wot_torque_nm_vs_rpm, ctx + ".wot_torque_nm_vs_rpm");
    if (e.wot_torque_nm_vs_rpm.x.front() < real(0.0)) {
        throw std::runtime_error(ctx + ".wot_torque_nm_vs_rpm: x[0] must be >= 0");
    }

    if (!j.contains("motoring_torque_nm_vs_rpm")) {
        throw std::runtime_error(ctx + ": 'motoring_torque_nm_vs_rpm' is required (PHYS-033 I3 - an I2-era engine "
                                  "block without it is no longer accepted)");
    }
    e.motoring_torque_nm_vs_rpm = read_curve(j.at("motoring_torque_nm_vs_rpm"));
    require_ascending(e.motoring_torque_nm_vs_rpm, ctx + ".motoring_torque_nm_vs_rpm");
    if (e.motoring_torque_nm_vs_rpm.x.front() != real(0.0)) {
        throw std::runtime_error(ctx + ".motoring_torque_nm_vs_rpm: x[0] must be exactly 0");
    }
    for (real y : e.motoring_torque_nm_vs_rpm.y) {
        if (y > real(0.0)) throw std::runtime_error(ctx + ".motoring_torque_nm_vs_rpm: every y must be <= 0");
    }

    e.inertia_kgm2 = j.value("inertia_kgm2", real(0.0));
    if (e.inertia_kgm2 <= real(0.0)) {
        throw std::runtime_error(ctx + ": inertia_kgm2 must be > 0 in I3 (PHYS-033 I3 - the crank node needs real "
                                  "inertia; an I2-era engine block with inertia_kgm2 <= 0 is no longer accepted)");
    }

    e.breakaway_torque_nm = j.value("breakaway_torque_nm", real(0.0));
    const real motoring_at_0 = e.motoring_torque_nm_vs_rpm.evaluate(real(0.0));
    if (e.breakaway_torque_nm < -motoring_at_0) {
        throw std::runtime_error(ctx + ": breakaway_torque_nm must be >= -motoring_torque_nm_vs_rpm(0)");
    }

    e.idle_rpm = j.value("idle_rpm", e.idle_rpm);
    e.stall_rpm = j.value("stall_rpm", e.stall_rpm);
    e.fire_rpm = j.value("fire_rpm", e.fire_rpm);
    e.start_complete_rpm = j.value("start_complete_rpm", e.start_complete_rpm);
    e.dfco_enable_rpm = j.value("dfco_enable_rpm", e.dfco_enable_rpm);
    e.dfco_reenable_rpm = j.value("dfco_reenable_rpm", e.dfco_reenable_rpm);

    if (j.contains("idle_control")) {
        const auto& ij = j.at("idle_control");
        e.idle.kp_per_rpm = ij.value("kp_per_rpm", e.idle.kp_per_rpm);
        e.idle.ki_per_rpm_s = ij.value("ki_per_rpm_s", e.idle.ki_per_rpm_s);
        e.idle.max_throttle = ij.value("max_throttle", e.idle.max_throttle);
    }

    if (j.contains("limiter")) {
        const auto& lj = j.at("limiter");
        const std::string type = lj.value("type", std::string("fuel_cut"));
        if (type == "fuel_cut") e.limiter.type = ps::drivetrain::LimiterType::FuelCut;
        else if (type == "soft_cut") e.limiter.type = ps::drivetrain::LimiterType::SoftCut;
        else if (type == "hard_cut") {
            throw std::runtime_error(ctx + ": limiter.type \"hard_cut\" was the I2 placeholder; use \"fuel_cut\" or "
                                      "\"soft_cut\" (PHYS-033 I3)");
        } else {
            throw std::runtime_error(ctx + ": limiter.type must be \"fuel_cut\" or \"soft_cut\", got \"" + type + "\"");
        }
        e.limiter.rpm = lj.value("rpm", e.limiter.rpm);
        e.limiter.hysteresis_rpm = lj.value("hysteresis_rpm", e.limiter.hysteresis_rpm);
        e.limiter.band_rpm = lj.value("band_rpm", e.limiter.band_rpm);
    }

    // PHYS-033 P4 I4 C12 (plan section 3.6): the optional all-speed diesel
    // governor with droop - see docs/data_formats/engine.md for the full
    // model. Self-contained field checks live here; the cross-field check
    // against a soft_cut limiter band (needs e.idle_rpm from above and
    // e.limiter from just above) lives with the other cross-field rpm-
    // threshold checks near the end of this function.
    if (j.contains("governor")) {
        const auto& gj = j.at("governor");
        const std::string type = gj.value("type", std::string("all_speed"));
        if (type != "all_speed") {
            throw std::runtime_error(ctx + ": governor.type must be \"all_speed\", got \"" + type + "\"");
        }
        e.has_governor = true;
        e.governor.high_idle_rpm = gj.value("high_idle_rpm", e.governor.high_idle_rpm);
        e.governor.rated_rpm = gj.value("rated_rpm", e.governor.rated_rpm);
        e.governor.droop = gj.value("droop", e.governor.droop);
        e.governor.channel = gj.value("channel", e.governor.channel);

        if (e.governor.high_idle_rpm <= e.idle_rpm) {
            throw std::runtime_error(ctx + ": governor.high_idle_rpm must be > idle_rpm, got " +
                                      std::to_string(e.governor.high_idle_rpm) + " <= " + std::to_string(e.idle_rpm));
        }
        if (!(e.governor.droop > real(0.0) && e.governor.droop <= real(0.3))) {
            throw std::runtime_error(ctx + ": governor.droop must be in (0, 0.3], got " + std::to_string(e.governor.droop));
        }
        if (!(e.governor.rated_rpm > e.idle_rpm && e.governor.rated_rpm <= e.governor.high_idle_rpm)) {
            throw std::runtime_error(ctx + ": governor.rated_rpm must be in (idle_rpm, high_idle_rpm], got " +
                                      std::to_string(e.governor.rated_rpm));
        }
        if (e.governor.channel.empty()) {
            throw std::runtime_error(ctx + ": governor.channel must not be empty");
        }
    }

    if (j.contains("starter")) {
        const auto& sj = j.at("starter");
        e.starter.stall_torque_nm = sj.value("stall_torque_nm", e.starter.stall_torque_nm);
        e.starter.free_speed_rpm = sj.value("free_speed_rpm", e.starter.free_speed_rpm);
        const std::string interlock = sj.value("interlock", std::string("clutch_or_neutral"));
        if (interlock == "clutch_or_neutral") e.starter.interlock = ps::drivetrain::StarterDesc::Interlock::ClutchOrNeutral;
        else if (interlock == "none") e.starter.interlock = ps::drivetrain::StarterDesc::Interlock::None;
        else throw std::runtime_error(ctx + ": starter.interlock must be \"clutch_or_neutral\" or \"none\", got \"" + interlock + "\"");
    }

    e.crank_axis_local = read_vec3(j.value("crank_axis_local", json()), Vec3::zero());
    require_zero_or_unit(e.crank_axis_local, ctx + ".crank_axis_local");

    const std::string initial_state = j.value("initial_state", std::string("running"));
    if (initial_state == "running") e.initial_state = ps::drivetrain::EngineInitialState::Running;
    else if (initial_state == "off") e.initial_state = ps::drivetrain::EngineInitialState::Off;
    else throw std::runtime_error(ctx + ": initial_state must be \"running\" or \"off\", got \"" + initial_state + "\"");

    // PHYS-033 P4 I7a C3 (plan section 3.4): optional for a torque_map
    // engine, default "electronic" - a simulated (MVEM) engine will require
    // it explicitly, but that kind is out of this commit's scope.
    const std::string throttle_input = j.value("throttle_input", std::string("electronic"));
    if (throttle_input == "electronic") e.throttle_input = ps::drivetrain::ThrottleInput::Electronic;
    else if (throttle_input == "mechanical") e.throttle_input = ps::drivetrain::ThrottleInput::Mechanical;
    else throw std::runtime_error(ctx + ": throttle_input must be \"electronic\" or \"mechanical\", got \"" + throttle_input + "\"");

    if (!(e.stall_rpm < e.start_complete_rpm && e.start_complete_rpm <= e.idle_rpm && e.idle_rpm < e.limiter.rpm)) {
        throw std::runtime_error(ctx + ": engine rpm thresholds must satisfy stall_rpm < start_complete_rpm <= "
                                  "idle_rpm < limiter.rpm");
    }
    if (!(e.dfco_reenable_rpm < e.dfco_enable_rpm)) {
        throw std::runtime_error(ctx + ": dfco_reenable_rpm must be < dfco_enable_rpm");
    }
    // PHYS-033 P4 I4 C12: a governor plus a soft_cut limiter would apply
    // droop twice if their bands overlap - a soft_cut limiter IS itself a
    // (fixed, no-droop-scaling) governor at full lever, so overlapping the
    // two double-counts the taper. Reject; point at fuel_cut instead (an
    // overspeed cutoff, not a second governor).
    if (e.has_governor && e.limiter.type == ps::drivetrain::LimiterType::SoftCut) {
        const real gov_lo = e.governor.high_idle_rpm - e.governor.droop * e.governor.rated_rpm;
        const real gov_hi = e.governor.high_idle_rpm;
        const real lim_lo = e.limiter.rpm - e.limiter.band_rpm;
        const real lim_hi = e.limiter.rpm;
        if (lim_lo <= gov_hi && gov_lo <= lim_hi) {
            throw std::runtime_error(ctx + ": governor combined with a soft_cut limiter whose band [" +
                                      std::to_string(lim_lo) + ", " + std::to_string(lim_hi) +
                                      "] overlaps the governor's own droop band [" + std::to_string(gov_lo) + ", " +
                                      std::to_string(gov_hi) + "] - use a fuel_cut overspeed limiter instead");
        }
    }
    return e;
}

ps::drivetrain::ClutchDesc read_clutch_fields(const json& cj, const std::string& ctx) {
    ps::drivetrain::ClutchDesc c;
    c.capacity_nm = cj.value("capacity_nm", c.capacity_nm);
    if (c.capacity_nm <= real(0.0)) throw std::runtime_error(ctx + ": clutch capacity_nm must be > 0");
    c.static_factor = cj.value("static_factor", c.static_factor);
    if (c.static_factor < real(1.0)) throw std::runtime_error(ctx + ": clutch static_factor must be >= 1");
    if (!cj.contains("engagement_vs_pedal")) throw std::runtime_error(ctx + ": clutch 'engagement_vs_pedal' is required");
    c.engagement_vs_pedal = read_curve(cj.at("engagement_vs_pedal"));
    require_ascending(c.engagement_vs_pedal, ctx + ".engagement_vs_pedal");
    if (c.engagement_vs_pedal.x.front() < real(0.0) || c.engagement_vs_pedal.x.back() > real(1.0)) {
        throw std::runtime_error(ctx + ".engagement_vs_pedal: x must be within [0, 1]");
    }
    for (std::size_t i = 1; i < c.engagement_vs_pedal.y.size(); ++i) {
        if (c.engagement_vs_pedal.y[i] > c.engagement_vs_pedal.y[i - 1]) {
            throw std::runtime_error(ctx + ".engagement_vs_pedal: y must be non-increasing");
        }
    }
    for (real y : c.engagement_vs_pedal.y) {
        if (y < real(0.0) || y > real(1.0)) throw std::runtime_error(ctx + ".engagement_vs_pedal: y must be within [0, 1]");
    }
    c.in_inertia_kgm2 = cj.value("in_inertia_kgm2", c.in_inertia_kgm2);
    c.out_inertia_kgm2 = cj.value("out_inertia_kgm2", c.out_inertia_kgm2);
    c.axis_local = read_vec3(cj.value("axis_local", json()), Vec3::zero());
    require_zero_or_unit(c.axis_local, ctx + ".axis_local");
    return c;
}

ps::drivetrain::GearboxDesc read_gearbox_fields(const json& j, const std::string& ctx) {
    ps::drivetrain::GearboxDesc g;
    const std::string type = j.value("type", std::string("manual"));
    if (type == "manual") g.type = ps::drivetrain::GearboxType::Manual;
    else if (type == "sequential") g.type = ps::drivetrain::GearboxType::Sequential;
    else throw std::runtime_error(ctx + ": gearbox 'type' must be \"manual\" or \"sequential\", got \"" + type + "\"");

    if (!j.contains("forward_ratios")) throw std::runtime_error(ctx + ": 'forward_ratios' is required");
    for (const auto& v : j.at("forward_ratios")) g.forward_ratios.push_back(v.get<real>());
    for (std::size_t i = 0; i < g.forward_ratios.size(); ++i) {
        if (g.forward_ratios[i] <= real(0.0)) throw std::runtime_error(ctx + ".forward_ratios: every ratio must be > 0");
        if (i > 0 && !(g.forward_ratios[i] < g.forward_ratios[i - 1])) {
            throw std::runtime_error(ctx + ".forward_ratios: must be strictly decreasing (1st first)");
        }
    }
    if (j.contains("reverse_ratios")) {
        for (const auto& v : j.at("reverse_ratios")) g.reverse_ratios.push_back(v.get<real>());
    }
    for (real r : g.reverse_ratios) {
        if (r >= real(0.0)) throw std::runtime_error(ctx + ".reverse_ratios: every ratio must be < 0");
    }

    if (!j.contains("forward_efficiency")) throw std::runtime_error(ctx + ": 'forward_efficiency' is required");
    for (const auto& v : j.at("forward_efficiency")) g.forward_efficiency.push_back(v.get<real>());
    if (j.contains("reverse_efficiency")) {
        for (const auto& v : j.at("reverse_efficiency")) g.reverse_efficiency.push_back(v.get<real>());
    }
    if (g.forward_efficiency.size() != g.forward_ratios.size()) {
        throw std::runtime_error(ctx + ": forward_efficiency must have the same length as forward_ratios");
    }
    if (g.reverse_efficiency.size() != g.reverse_ratios.size()) {
        throw std::runtime_error(ctx + ": reverse_efficiency must have the same length as reverse_ratios");
    }
    for (real e : g.forward_efficiency) {
        if (e <= real(0.0) || e > real(1.0)) throw std::runtime_error(ctx + ".forward_efficiency: every value must be in (0, 1]");
    }
    for (real e : g.reverse_efficiency) {
        if (e <= real(0.0) || e > real(1.0)) throw std::runtime_error(ctx + ".reverse_efficiency: every value must be in (0, 1]");
    }

    g.input_inertia_kgm2 = j.value("input_inertia_kgm2", g.input_inertia_kgm2);
    g.output_inertia_kgm2 = j.value("output_inertia_kgm2", g.output_inertia_kgm2);
    g.input_axis_local = read_vec3(j.value("input_axis_local", json()), Vec3::zero());
    require_zero_or_unit(g.input_axis_local, ctx + ".input_axis_local");
    g.output_axis_local = read_vec3(j.value("output_axis_local", json()), Vec3::zero());
    require_zero_or_unit(g.output_axis_local, ctx + ".output_axis_local");

    if (j.contains("synchro")) {
        const auto& sj = j.at("synchro");
        if (sj.contains("forward_capacity_nm")) {
            for (const auto& v : sj.at("forward_capacity_nm")) g.synchro.forward_capacity_nm.push_back(v.get<real>());
        }
        if (sj.contains("reverse_capacity_nm")) {
            for (const auto& v : sj.at("reverse_capacity_nm")) g.synchro.reverse_capacity_nm.push_back(v.get<real>());
        }
        g.synchro.static_factor = sj.value("static_factor", g.synchro.static_factor);
        g.synchro.max_dog_mismatch_rad_s = sj.value("max_dog_mismatch_rad_s", g.synchro.max_dog_mismatch_rad_s);
    }
    if (!g.synchro.forward_capacity_nm.empty() && g.synchro.forward_capacity_nm.size() != g.forward_ratios.size()) {
        throw std::runtime_error(ctx + ": synchro.forward_capacity_nm must have the same length as forward_ratios");
    }
    if (!g.synchro.reverse_capacity_nm.empty() && g.synchro.reverse_capacity_nm.size() != g.reverse_ratios.size()) {
        throw std::runtime_error(ctx + ": synchro.reverse_capacity_nm must have the same length as reverse_ratios");
    }
    // Gearbox::synchro_capacity_kinetic_for_position indexes these arrays
    // directly and expects them sized to match the ratio arrays exactly
    // (0 entries meaning "dog only" are fine, an EMPTY array is not, unless
    // this gearbox genuinely has zero forward/reverse gears in that
    // direction) - pad with zeros rather than require every file to spell
    // them out.
    if (g.synchro.forward_capacity_nm.empty()) g.synchro.forward_capacity_nm.assign(g.forward_ratios.size(), real(0.0));
    if (g.synchro.reverse_capacity_nm.empty()) g.synchro.reverse_capacity_nm.assign(g.reverse_ratios.size(), real(0.0));

    g.neutral_phase_s = j.value("neutral_phase_s", g.neutral_phase_s);
    g.dog_release_torque_nm = j.value("dog_release_torque_nm", g.dog_release_torque_nm);
    g.grind_after_s = j.value("grind_after_s", g.grind_after_s);
    g.abort_after_s = j.value("abort_after_s", g.abort_after_s);
    g.initial_gear = j.value("initial_gear", g.initial_gear);
    if (g.initial_gear < -static_cast<int>(g.reverse_ratios.size()) || g.initial_gear > static_cast<int>(g.forward_ratios.size())) {
        throw std::runtime_error(ctx + ": initial_gear is out of range");
    }
    return g;
}

// PHYS-033 I3 plan section 9.1: {"id", "type": "engine"|"gearbox", "ref":
// "<relative path>"} - a ref'd object may contain ONLY id/type/ref (no
// merging with inline fields). Returns the resolved json object to read
// fields from (either the external file's own top-level object, format-
// checked, or `cj` itself for an inline component).
json resolve_component_ref(const json& cj, const std::string& kind, const std::string& base_dir, const std::string& ctx) {
    if (!cj.contains("ref")) return cj;
    for (auto it = cj.begin(); it != cj.end(); ++it) {
        if (it.key() != "id" && it.key() != "type" && it.key() != "ref") {
            throw std::runtime_error(ctx + ": a component with 'ref' may only contain 'id', 'type' and 'ref' "
                                      "(found extra field '" + it.key() + "')");
        }
    }
    const std::string path = resolve_path(base_dir, cj.at("ref").get<std::string>());
    const json fj = read_json_file(path);
    require_format(fj, kind, 1, path);
    return fj;
}

} // namespace

vehicle::TyreDesc load_tyre_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "tyre", 1, path);
    return read_tyre_object(j);
}

vehicle::VehicleDesc load_vehicle_json(const std::string& path) {
    const json j = read_json_file(path);
    require_vehicle_format(j, path);
    const std::string base_dir = std::filesystem::path(path).parent_path().string();

    VehicleDesc v;
    v.name = j.value("name", std::string());
    v.source = j.value("source", std::string());
    if (v.source.empty()) throw std::runtime_error("vehicle '" + v.name + "': 'source' is required (PLAN.md section 8)");

    // PHYS-033 I2 (owner ruling Q7): the /1 top-level "drive"/"brakes"
    // blocks are gone - "powertrain"/"brake_system" replace them.
    if (j.contains("drive")) {
        throw std::runtime_error("vehicle " + path + ": 'drive' is no longer a vehicle field "
                                  "(physics_sim.vehicle/2) - see 'powertrain' (PHYS-033 Q7)");
    }
    if (j.contains("brakes")) {
        throw std::runtime_error("vehicle " + path + ": 'brakes' is no longer a vehicle field "
                                  "(physics_sim.vehicle/2) - see 'brake_system' (PHYS-033 Q7)");
    }

    if (!j.contains("wheels") || !j.at("wheels").is_array() || j.at("wheels").empty()) {
        throw std::runtime_error("vehicle " + path + " has no 'wheels' array");
    }
    for (const auto& wj : j.at("wheels")) v.wheels.push_back(read_wheel(wj, base_dir));

    v.steering = read_steering(j.value("steering", json()));
    v.brake_system = read_brake_system(j.value("brake_system", json()));

    // --- aero (plan section 4.4/9.1) ---------------------------------------
    if (j.contains("aero")) {
        const auto& aj = j.at("aero");
        v.aero.drag_area_m2 = aj.value("drag_area_m2", real(0.0));
        if (v.aero.drag_area_m2 < real(0.0)) throw std::runtime_error("vehicle '" + v.name + "': aero.drag_area_m2 must be >= 0");
    }

    // --- powertrain (plan section 9.1) --------------------------------------
    PowertrainDesc p;
    std::vector<std::string> component_ids;
    if (j.contains("powertrain")) {
        const auto& pj = j.at("powertrain");
        if (pj.contains("components")) {
            for (const auto& cj : pj.at("components")) {
                if (!cj.contains("id") || !cj.at("id").is_string() || cj.at("id").get<std::string>().empty()) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.components[]: 'id' is required and must be a non-empty string");
                }
                const std::string id = cj.at("id").get<std::string>();
                if (std::find(component_ids.begin(), component_ids.end(), id) != component_ids.end()) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.components[]: duplicate id '" + id + "'");
                }
                component_ids.push_back(id);
                if (!cj.contains("type") || !cj.at("type").is_string()) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.components['" + id + "']: 'type' is required");
                }
                const std::string type = cj.at("type").get<std::string>();
                const std::string ctx = "vehicle '" + v.name + "': powertrain.components['" + id + "']";

                if (type == "engine") {
                    const json ej = resolve_component_ref(cj, "engine", base_dir, ctx);
                    p.components.push_back({id, read_engine_fields(ej, ctx)});
                } else if (type == "clutch") {
                    if (cj.contains("ref")) throw std::runtime_error(ctx + ": 'clutch' components are always inline in I3 (no 'ref')");
                    p.components.push_back({id, read_clutch_fields(cj, ctx)});
                } else if (type == "gearbox") {
                    const json gj = resolve_component_ref(cj, "gearbox", base_dir, ctx);
                    p.components.push_back({id, read_gearbox_fields(gj, ctx)});
                } else if (type == "gear_pair") {
                    ps::drivetrain::GearPairDesc g;
                    g.ratio = cj.value("ratio", g.ratio);
                    if (g.ratio == real(0.0)) throw std::runtime_error(ctx + ": gear_pair 'ratio' must be != 0");
                    g.efficiency = cj.value("efficiency", g.efficiency);
                    if (g.efficiency <= real(0.0) || g.efficiency > real(1.0)) {
                        throw std::runtime_error(ctx + ": gear_pair 'efficiency' must be in (0, 1]");
                    }
                    g.inertia_in_kgm2 = cj.value("inertia_in_kgm2", g.inertia_in_kgm2);
                    g.inertia_out_kgm2 = cj.value("inertia_out_kgm2", g.inertia_out_kgm2);
                    g.in_axis_local = read_vec3(cj.value("in_axis_local", json()), Vec3::zero());
                    require_zero_or_unit(g.in_axis_local, ctx + ".in_axis_local");
                    g.out_axis_local = read_vec3(cj.value("out_axis_local", json()), Vec3::zero());
                    require_zero_or_unit(g.out_axis_local, ctx + ".out_axis_local");
                    p.components.push_back({id, g});
                } else if (type == "differential") {
                    ps::drivetrain::DifferentialDesc d;
                    // PHYS-033 P4 I4 C6/C7 (plan sections 3.1/3.2): "open"/
                    // "locked"/"viscous"/"clutch_lsd"/"torsen" are
                    // implemented; "active" is a named-but-rejected
                    // placeholder (a clear, kind-specific message instead of
                    // falling through to the generic "unknown kind" case) so
                    // a vehicle file written against a later increment's
                    // kind fails loudly here, not silently as "open".
                    const std::string kind_str = cj.value("kind", std::string());
                    if (kind_str == "open") {
                        d.kind = ps::drivetrain::DifferentialKind::Open;
                    } else if (kind_str == "locked") {
                        d.kind = ps::drivetrain::DifferentialKind::Locked;
                    } else if (kind_str == "viscous") {
                        d.kind = ps::drivetrain::DifferentialKind::Viscous;
                    } else if (kind_str == "clutch_lsd") {
                        d.kind = ps::drivetrain::DifferentialKind::ClutchLsd;
                    } else if (kind_str == "torsen") {
                        d.kind = ps::drivetrain::DifferentialKind::Torsen;
                    } else if (kind_str == "active") {
                        throw std::runtime_error(ctx + ": differential kind \"active\" is not supported until a "
                                                        "torque-vectoring controller exists");
                    } else {
                        throw std::runtime_error(ctx + ": differential 'kind' must be \"open\", \"locked\", \"viscous\", "
                                                        "\"clutch_lsd\" or \"torsen\"");
                    }
                    d.ratio = cj.value("ratio", d.ratio);
                    if (d.ratio == real(0.0)) throw std::runtime_error(ctx + ": differential 'ratio' must be != 0");
                    d.efficiency = cj.value("efficiency", d.efficiency);
                    if (d.efficiency <= real(0.0) || d.efficiency > real(1.0)) {
                        throw std::runtime_error(ctx + ": differential 'efficiency' must be in (0, 1]");
                    }
                    d.carrier_inertia_kgm2 = cj.value("carrier_inertia_kgm2", d.carrier_inertia_kgm2);
                    d.carrier_axis_local = read_vec3(cj.value("carrier_axis_local", json()), Vec3::zero());
                    require_zero_or_unit(d.carrier_axis_local, ctx + ".carrier_axis_local");
                    // PHYS-033 P4 I4 C6: viscous_nm_s_rad is only meaningful
                    // (and required > 0) for kind == "viscous" - the implicit
                    // L-R coupling coefficient (powertrain_desc.h's own field
                    // comment).
                    if (cj.contains("viscous_nm_s_rad")) {
                        if (d.kind != ps::drivetrain::DifferentialKind::Viscous) {
                            throw std::runtime_error(ctx +
                                                      ": differential 'viscous_nm_s_rad' is only valid for kind \"viscous\"");
                        }
                        d.viscous_nm_s_rad = cj.value("viscous_nm_s_rad", d.viscous_nm_s_rad);
                    }
                    if (d.kind == ps::drivetrain::DifferentialKind::Viscous && !(d.viscous_nm_s_rad > real(0.0))) {
                        throw std::runtime_error(ctx + ": differential kind \"viscous\" requires 'viscous_nm_s_rad' > 0");
                    }
                    // PHYS-033 P4 I4 C7 (plan section 3.2): clutch_lsd's
                    // preload_nm/ramp/static_factor and torsen's
                    // torque_bias_ratio - each only valid on its own kind
                    // (same "field of another kind is an error" rule as
                    // viscous_nm_s_rad above), so a typo'd kind cannot
                    // silently ignore the parameters the author wrote.
                    const bool is_lsd = d.kind == ps::drivetrain::DifferentialKind::ClutchLsd;
                    const bool is_torsen = d.kind == ps::drivetrain::DifferentialKind::Torsen;
                    for (const char* key : {"preload_nm", "ramp", "static_factor"}) {
                        if (cj.contains(key) && !is_lsd) {
                            throw std::runtime_error(ctx + ": differential '" + key + "' is only valid for kind \"clutch_lsd\"");
                        }
                    }
                    if (cj.contains("torque_bias_ratio") && !is_torsen) {
                        throw std::runtime_error(ctx + ": differential 'torque_bias_ratio' is only valid for kind \"torsen\"");
                    }
                    if (is_lsd) {
                        d.preload_nm = cj.value("preload_nm", d.preload_nm);
                        if (!(d.preload_nm >= real(0.0))) throw std::runtime_error(ctx + ": differential 'preload_nm' must be >= 0");
                        if (!cj.contains("ramp")) {
                            throw std::runtime_error(ctx + ": differential kind \"clutch_lsd\" requires a 'ramp' block "
                                                            "{\"drive_factor\", \"coast_factor\"}");
                        }
                        const auto& rj = cj.at("ramp");
                        if (!rj.is_object() || !rj.contains("drive_factor") || !rj.contains("coast_factor")) {
                            throw std::runtime_error(ctx + ": differential 'ramp' requires both 'drive_factor' and 'coast_factor'");
                        }
                        d.drive_factor = rj.at("drive_factor").get<real>();
                        d.coast_factor = rj.at("coast_factor").get<real>();
                        if (!(d.drive_factor >= real(0.0) && d.drive_factor < real(1.0))) {
                            throw std::runtime_error(ctx + ": differential ramp.drive_factor must be in [0, 1)");
                        }
                        if (!(d.coast_factor >= real(0.0) && d.coast_factor < real(1.0))) {
                            throw std::runtime_error(ctx + ": differential ramp.coast_factor must be in [0, 1)");
                        }
                        d.static_factor = cj.value("static_factor", d.static_factor);
                        if (!(d.static_factor >= real(1.0))) throw std::runtime_error(ctx + ": differential 'static_factor' must be >= 1");
                    }
                    if (is_torsen) {
                        if (!cj.contains("torque_bias_ratio")) {
                            throw std::runtime_error(ctx + ": differential kind \"torsen\" requires 'torque_bias_ratio' > 1");
                        }
                        d.torque_bias_ratio = cj.at("torque_bias_ratio").get<real>();
                        if (!(d.torque_bias_ratio > real(1.0))) {
                            throw std::runtime_error(ctx + ": differential 'torque_bias_ratio' must be > 1");
                        }
                    }
                    // PHYS-033 P4 I4 C5 (plan section 3.1): lockable + optional lock block.
                    d.lockable = cj.value("lockable", false);
                    // PHYS-033 P4 I4 C6: kind "locked" is already always
                    // locked (an unconditional row, powertrain.cpp's
                    // always_locked_diffs_) - "lockable" (togglable at
                    // runtime) is meaningless on it.
                    if (d.lockable && d.kind == ps::drivetrain::DifferentialKind::Locked) {
                        throw std::runtime_error(ctx + ": differential 'lockable' is redundant on kind \"locked\" "
                                                        "(already always locked)");
                    }
                    if (cj.contains("lock")) {
                        if (!d.lockable) throw std::runtime_error(ctx + ": differential 'lock' requires \"lockable\": true");
                        const auto& lj = cj.at("lock");
                        d.lock.max_engage_mismatch_rad_s = lj.value("max_engage_mismatch_rad_s", d.lock.max_engage_mismatch_rad_s);
                        if (!(d.lock.max_engage_mismatch_rad_s >= real(0.0))) {
                            throw std::runtime_error(ctx + ": differential lock.max_engage_mismatch_rad_s must be >= 0");
                        }
                        d.lock.channel = lj.value("channel", d.lock.channel);
                        if (d.lock.channel.empty()) throw std::runtime_error(ctx + ": differential lock.channel must be non-empty");
                    }
                    p.components.push_back({id, d});
                } else if (type == "transfer_case") {
                    // PHYS-033 P4 I4 C9 (plan section 3.3): the MFWD transfer
                    // case - a rigid `rear` row (omega_in = rear_ratio*omega_rear)
                    // plus a friction `front` row (the AWD clutch, ratio_b =
                    // front_ratio) - see components.h's TransferCase.
                    ps::drivetrain::TransferCaseDesc t;
                    t.rear_ratio = cj.value("rear_ratio", t.rear_ratio);
                    if (t.rear_ratio == real(0.0)) throw std::runtime_error(ctx + ": transfer_case 'rear_ratio' must be != 0");
                    t.front_ratio = cj.value("front_ratio", t.front_ratio);
                    if (t.front_ratio == real(0.0)) throw std::runtime_error(ctx + ": transfer_case 'front_ratio' must be != 0");
                    if (!cj.contains("clutch")) {
                        throw std::runtime_error(ctx + ": transfer_case requires a 'clutch' block "
                                                        "{\"capacity_nm\", \"static_factor\", \"engage_time_s\"}");
                    }
                    const auto& tcj = cj.at("clutch");
                    t.clutch.capacity_nm = tcj.value("capacity_nm", t.clutch.capacity_nm);
                    if (!(t.clutch.capacity_nm > real(0.0))) {
                        throw std::runtime_error(ctx + ": transfer_case clutch.capacity_nm must be > 0");
                    }
                    t.clutch.static_factor = tcj.value("static_factor", t.clutch.static_factor);
                    if (!(t.clutch.static_factor >= real(1.0))) {
                        throw std::runtime_error(ctx + ": transfer_case clutch.static_factor must be >= 1");
                    }
                    t.clutch.engage_time_s = tcj.value("engage_time_s", t.clutch.engage_time_s);
                    if (!(t.clutch.engage_time_s >= real(0.0))) {
                        throw std::runtime_error(ctx + ": transfer_case clutch.engage_time_s must be >= 0");
                    }
                    t.channel = cj.value("channel", t.channel);
                    if (t.channel.empty()) throw std::runtime_error(ctx + ": transfer_case 'channel' must be non-empty");
                    t.in_inertia_kgm2 = cj.value("in_inertia_kgm2", t.in_inertia_kgm2);
                    t.front_inertia_kgm2 = cj.value("front_inertia_kgm2", t.front_inertia_kgm2);
                    t.axis_local = read_vec3(cj.value("axis_local", json()), Vec3::zero());
                    require_zero_or_unit(t.axis_local, ctx + ".axis_local");
                    p.components.push_back({id, t});
                } else if (type == "park_lock") {
                    // PHYS-033 P4 I4 C10 (plan section 3.4, owner ruling): the
                    // park pawl - a single friction row from its `shaft` port
                    // to GROUND, see components.h's ParkLock. At most one per
                    // vehicle - checked here at parse time (unlike engine/
                    // clutch/gearbox/transfer_case, whose own "at most one"
                    // rule is only enforced later, at Powertrain graph-build
                    // time) so a duplicate is a plain load_vehicle_json error.
                    for (const auto& existing : p.components) {
                        if (std::get_if<ps::drivetrain::ParkLockDesc>(&existing.params) != nullptr) {
                            throw std::runtime_error(ctx + ": more than one park_lock component - at most one is supported");
                        }
                    }
                    ps::drivetrain::ParkLockDesc pl;
                    pl.capacity_nm = cj.value("capacity_nm", pl.capacity_nm);
                    if (!(pl.capacity_nm > real(0.0))) {
                        throw std::runtime_error(ctx + ": park_lock 'capacity_nm' must be > 0");
                    }
                    pl.static_factor = cj.value("static_factor", pl.static_factor);
                    if (!(pl.static_factor >= real(1.0))) {
                        throw std::runtime_error(ctx + ": park_lock 'static_factor' must be >= 1");
                    }
                    pl.max_engage_speed_rad_s = cj.value("max_engage_speed_rad_s", pl.max_engage_speed_rad_s);
                    if (!(pl.max_engage_speed_rad_s >= real(0.0))) {
                        throw std::runtime_error(ctx + ": park_lock 'max_engage_speed_rad_s' must be >= 0");
                    }
                    pl.channel = cj.value("channel", pl.channel);
                    if (pl.channel.empty()) throw std::runtime_error(ctx + ": park_lock 'channel' must be non-empty");
                    p.components.push_back({id, pl});
                } else if (type == "pto") {
                    // PHYS-033 P4 I4 C11 (plan section 3.5): the power take-off
                    // - a friction row (in = engine/crank-side, out = PTO-
                    // side) with a per-mode ratio, see components.h's Pto. At
                    // most one per vehicle - checked here at parse time, like
                    // park_lock above.
                    for (const auto& existing : p.components) {
                        if (std::get_if<ps::drivetrain::PtoDesc>(&existing.params) != nullptr) {
                            throw std::runtime_error(ctx + ": more than one pto component - at most one is supported");
                        }
                    }
                    ps::drivetrain::PtoDesc pto;
                    if (!cj.contains("ratios") || !cj.at("ratios").is_object() || cj.at("ratios").empty()) {
                        throw std::runtime_error(ctx + ": pto requires a non-empty 'ratios' object with key(s) \"540\" and/or \"1000\"");
                    }
                    for (const auto& [rkey, rval] : cj.at("ratios").items()) {
                        const real ratio = rval.get<real>();
                        if (!(ratio > real(0.0))) throw std::runtime_error(ctx + ": pto ratios['" + rkey + "'] must be > 0");
                        if (rkey == "540") {
                            pto.has_540 = true;
                            pto.ratio_540 = ratio;
                        } else if (rkey == "1000") {
                            pto.has_1000 = true;
                            pto.ratio_1000 = ratio;
                        } else {
                            throw std::runtime_error(ctx + ": pto ratios key '" + rkey + "' is not \"540\" or \"1000\"");
                        }
                    }
                    pto.capacity_nm = cj.value("capacity_nm", pto.capacity_nm);
                    if (!(pto.capacity_nm > real(0.0))) throw std::runtime_error(ctx + ": pto 'capacity_nm' must be > 0");
                    pto.static_factor = cj.value("static_factor", pto.static_factor);
                    if (!(pto.static_factor >= real(1.0))) throw std::runtime_error(ctx + ": pto 'static_factor' must be >= 1");
                    pto.engage_time_s = cj.value("engage_time_s", pto.engage_time_s);
                    if (!(pto.engage_time_s >= real(0.0))) throw std::runtime_error(ctx + ": pto 'engage_time_s' must be >= 0");
                    pto.efficiency = cj.value("efficiency", pto.efficiency);
                    if (!(pto.efficiency > real(0.0) && pto.efficiency <= real(1.0))) {
                        throw std::runtime_error(ctx + ": pto 'efficiency' must be in (0, 1]");
                    }
                    pto.out_inertia_kgm2 = cj.value("out_inertia_kgm2", pto.out_inertia_kgm2);
                    if (!(pto.out_inertia_kgm2 >= real(0.0))) throw std::runtime_error(ctx + ": pto 'out_inertia_kgm2' must be >= 0");
                    pto.disengaged_brake_time_s = cj.value("disengaged_brake_time_s", pto.disengaged_brake_time_s);
                    if (!(pto.disengaged_brake_time_s > real(0.0))) {
                        throw std::runtime_error(ctx + ": pto 'disengaged_brake_time_s' must be > 0");
                    }
                    pto.channel = cj.value("channel", pto.channel);
                    if (pto.channel.empty()) throw std::runtime_error(ctx + ": pto 'channel' must be non-empty");
                    pto.speed_channel = cj.value("speed_channel", pto.speed_channel);
                    if (pto.speed_channel.empty()) throw std::runtime_error(ctx + ": pto 'speed_channel' must be non-empty");
                    pto.axis_local = read_vec3(cj.value("axis_local", json()), Vec3::zero());
                    require_zero_or_unit(pto.axis_local, ctx + ".axis_local");
                    p.components.push_back({id, pto});
                } else if (type == "load_dyno") {
                    // PHYS-033 P4 I4 C11: test/bench only - the loader supports
                    // it, but no shipped vehicle uses it. See components.h's
                    // LoadDyno. Any number per vehicle (no "at most one" rule).
                    ps::drivetrain::LoadDynoDesc dyno;
                    const std::string mode_str = cj.value("mode", std::string("brake"));
                    if (mode_str == "brake") {
                        dyno.mode = ps::drivetrain::LoadDynoMode::Brake;
                    } else if (mode_str == "viscous") {
                        dyno.mode = ps::drivetrain::LoadDynoMode::Viscous;
                    } else if (mode_str == "quadratic") {
                        dyno.mode = ps::drivetrain::LoadDynoMode::Quadratic;
                    } else {
                        throw std::runtime_error(ctx + ": load_dyno 'mode' must be \"brake\", \"viscous\" or \"quadratic\"");
                    }
                    dyno.inertia_kgm2 = cj.value("inertia_kgm2", dyno.inertia_kgm2);
                    if (!(dyno.inertia_kgm2 >= real(0.0))) throw std::runtime_error(ctx + ": load_dyno 'inertia_kgm2' must be >= 0");
                    if (dyno.mode == ps::drivetrain::LoadDynoMode::Brake) {
                        dyno.torque_nm = cj.value("torque_nm", dyno.torque_nm);
                        if (!(dyno.torque_nm > real(0.0))) {
                            throw std::runtime_error(ctx + ": load_dyno 'torque_nm' must be > 0 for mode \"brake\"");
                        }
                    } else if (cj.contains("torque_nm")) {
                        throw std::runtime_error(ctx + ": load_dyno 'torque_nm' is only valid for mode \"brake\"");
                    }
                    if (dyno.mode == ps::drivetrain::LoadDynoMode::Viscous) {
                        dyno.viscous_nm_s_rad = cj.value("viscous_nm_s_rad", dyno.viscous_nm_s_rad);
                        if (!(dyno.viscous_nm_s_rad > real(0.0))) {
                            throw std::runtime_error(ctx + ": load_dyno 'viscous_nm_s_rad' must be > 0 for mode \"viscous\"");
                        }
                    } else if (cj.contains("viscous_nm_s_rad")) {
                        throw std::runtime_error(ctx + ": load_dyno 'viscous_nm_s_rad' is only valid for mode \"viscous\"");
                    }
                    if (dyno.mode == ps::drivetrain::LoadDynoMode::Quadratic) {
                        dyno.quadratic_nm_s2_rad2 = cj.value("quadratic_nm_s2_rad2", dyno.quadratic_nm_s2_rad2);
                        if (!(dyno.quadratic_nm_s2_rad2 > real(0.0))) {
                            throw std::runtime_error(ctx + ": load_dyno 'quadratic_nm_s2_rad2' must be > 0 for mode \"quadratic\"");
                        }
                    } else if (cj.contains("quadratic_nm_s2_rad2")) {
                        throw std::runtime_error(ctx + ": load_dyno 'quadratic_nm_s2_rad2' is only valid for mode \"quadratic\"");
                    }
                    dyno.scale_channel = cj.value("scale_channel", dyno.scale_channel);
                    dyno.axis_local = read_vec3(cj.value("axis_local", json()), Vec3::zero());
                    require_zero_or_unit(dyno.axis_local, ctx + ".axis_local");
                    p.components.push_back({id, dyno});
                } else if (type == "throttle_control") {
                    // PHYS-033 P4 I7a C3 (plan section 3.4): the
                    // throttle-command stage's own component descriptor - a
                    // pure control-signal seam (no shaft-network port of its
                    // own; Powertrain's Phase A dispatch treats it as a
                    // no-op for graph-building purposes and instead uses it
                    // to configure the typed ThrottleStage member). "At most
                    // one, and only when 'engine' names this vehicle's
                    // engine component" is enforced in Powertrain's
                    // constructor (it needs engine_component_index anyway);
                    // checked again here too so a duplicate is caught at
                    // the clearest possible location.
                    for (const auto& existing : p.components) {
                        if (std::get_if<ps::drivetrain::ThrottleControlDesc>(&existing.params) != nullptr) {
                            throw std::runtime_error(ctx + ": more than one throttle_control component - at most one is supported");
                        }
                    }
                    ps::drivetrain::ThrottleControlDesc tcd;
                    if (!cj.contains("engine") || !cj.at("engine").is_string() || cj.at("engine").get<std::string>().empty()) {
                        throw std::runtime_error(ctx + ": throttle_control 'engine' (a component id reference) is required");
                    }
                    tcd.engine = cj.at("engine").get<std::string>();
                    // Existence, type (must be a torque_map engine) and
                    // kind-vs-throttle_input agreement are all checked
                    // ORDER-INDEPENDENTLY after the full components array has
                    // been parsed (below, near 'v.powertrain = p;') - a
                    // throttle_control naming an engine declared LATER in the
                    // array must still load (PHYS-033 P4 I7a C3 review item 5).
                    // "electronic" | "mechanical_linkage" - deliberately NOT
                    // the same string set as TorqueMapEngineDesc::
                    // throttle_input's "electronic"|"mechanical" (see this
                    // file's read_engine_fields): the two fields name
                    // different things (an engine's OWN wiring vs. this
                    // component's kind) and the plan spec gives them
                    // different literal strings on purpose.
                    const std::string kind_str = cj.value("kind", std::string());
                    if (kind_str == "electronic") {
                        tcd.kind = ps::drivetrain::ThrottleInput::Electronic;
                    } else if (kind_str == "mechanical_linkage") {
                        tcd.kind = ps::drivetrain::ThrottleInput::Mechanical;
                    } else {
                        throw std::runtime_error(ctx + ": throttle_control 'kind' must be \"electronic\" or "
                                                        "\"mechanical_linkage\", got \"" + kind_str + "\"");
                    }
                    // pedal_map (electronic only) / linkage_curve (mechanical
                    // only) - plan section 3.4.5's own field-name split, so a
                    // vehicle file cannot silently apply the wrong kind's
                    // curve.
                    if (cj.contains("pedal_map")) {
                        if (tcd.kind != ps::drivetrain::ThrottleInput::Electronic) {
                            throw std::runtime_error(ctx + ": throttle_control 'pedal_map' is only valid for kind "
                                                            "\"electronic\" (a mechanical linkage uses 'linkage_curve' instead)");
                        }
                        tcd.has_curve = true;
                        tcd.curve = read_curve(cj.at("pedal_map"));
                        require_ascending(tcd.curve, ctx + ".pedal_map");
                        require_unit_domain_covered_nondecreasing(tcd.curve, ctx + ".pedal_map");
                    }
                    if (cj.contains("linkage_curve")) {
                        if (tcd.kind != ps::drivetrain::ThrottleInput::Mechanical) {
                            throw std::runtime_error(ctx + ": throttle_control 'linkage_curve' is only valid for kind "
                                                            "\"mechanical_linkage\" (electronic uses 'pedal_map' instead)");
                        }
                        tcd.has_curve = true;
                        tcd.curve = read_curve(cj.at("linkage_curve"));
                        require_ascending(tcd.curve, ctx + ".linkage_curve");
                        require_unit_domain_covered_nondecreasing(tcd.curve, ctx + ".linkage_curve");
                    }
                    tcd.hand_lever_channel = cj.value("hand_lever_channel", std::string());
                    p.components.push_back({id, tcd});
                } else {
                    throw std::runtime_error(ctx + ": unknown type \"" + type + "\"");
                }
            }
        }

        // PHYS-033 P4 I7a C3 review items 2/3(b)/5: order-independent
        // throttle_control cross-checks, run once every component has been
        // parsed (so a stage listed BEFORE its engine in the array still
        // loads - the ordering-dependent early check this replaced is gone,
        // see the throttle_control parser above).
        {
            const ps::drivetrain::ThrottleControlDesc* tcd_ptr = nullptr;
            std::string tcd_id;
            for (const auto& c : p.components) {
                if (const auto* t = std::get_if<ps::drivetrain::ThrottleControlDesc>(&c.params)) {
                    tcd_ptr = t;
                    tcd_id = c.id;
                }
            }
            if (tcd_ptr != nullptr) {
                const std::string tctx = "vehicle '" + v.name + "': powertrain.components['" + tcd_id + "']";
                const ps::drivetrain::TorqueMapEngineDesc* engine_ptr = nullptr;
                for (const auto& c : p.components) {
                    if (c.id == tcd_ptr->engine) {
                        engine_ptr = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params);
                        break;
                    }
                }
                // Item 5: the ref must name an actual torque_map engine
                // component - covers both "unknown id" and "a vehicle with
                // no engine component at all" (engine_ptr stays nullptr
                // either way), and "id exists but is some other type".
                if (engine_ptr == nullptr) {
                    throw std::runtime_error(tctx + ": 'engine' (\"" + tcd_ptr->engine +
                                              "\") must name a \"torque_map\" engine component in this vehicle's powertrain");
                }
                // Item 2: the stage's own 'kind' must agree with the engine's
                // 'throttle_input' - both directions (an implicit stage
                // always agrees by construction, since it just copies
                // throttle_input; only an EXPLICIT stage can disagree).
                if (tcd_ptr->kind != engine_ptr->throttle_input) {
                    const char* stage_kind = tcd_ptr->kind == ps::drivetrain::ThrottleInput::Electronic ? "electronic" : "mechanical_linkage";
                    const char* engine_kind = engine_ptr->throttle_input == ps::drivetrain::ThrottleInput::Electronic ? "electronic" : "mechanical";
                    throw std::runtime_error(tctx + ": 'kind' (\"" + std::string(stage_kind) + "\") does not match engine '" +
                                              tcd_ptr->engine + "'.throttle_input (\"" + std::string(engine_kind) +
                                              "\") - an explicit throttle_control component must agree with its engine, "
                                              "it no longer silently overrides it");
                }
                // Item 3(b): an explicit hand_lever_channel on a GOVERNED
                // engine must either be empty (falls back to
                // governor.channel, Powertrain Phase K) or equal
                // governor.channel itself - a different channel would mean
                // the governor and the throttle stage read two different
                // hand levers, which makes no physical sense on one engine.
                // An explicit hand_lever_channel on an UNGOVERNED engine is
                // always allowed (a hand lever on the same linkage/ECU is
                // physical) and combines by max() as usual.
                if (!tcd_ptr->hand_lever_channel.empty() && engine_ptr->has_governor &&
                    tcd_ptr->hand_lever_channel != engine_ptr->governor.channel) {
                    throw std::runtime_error(tctx + ": 'hand_lever_channel' (\"" + tcd_ptr->hand_lever_channel +
                                              "\") differs from engine '" + tcd_ptr->engine + "'.governor.channel (\"" +
                                              engine_ptr->governor.channel +
                                              "\") - a governed engine has one hand lever; leave hand_lever_channel "
                                              "unset to use the governor's own channel, or set it to that same channel");
                }
            }
        }

        if (pj.contains("shafts")) {
            std::size_t shaft_index = 0;
            for (const auto& sj : pj.at("shafts")) {
                const std::string sctx = "vehicle '" + v.name + "': powertrain.shafts[" + std::to_string(shaft_index++) + "]";
                ps::drivetrain::ShaftDesc s;
                if (!sj.contains("ports")) throw std::runtime_error("vehicle '" + v.name + "': powertrain.shafts[]: 'ports' is required");
                for (const auto& prj : sj.at("ports")) {
                    const std::string ref = prj.get<std::string>();
                    ps::drivetrain::PortRef pr;
                    if (ref.rfind("wheel:", 0) == 0) {
                        pr.is_wheel = true;
                        pr.component = ref.substr(6);
                        if (pr.component.empty()) throw std::runtime_error("vehicle '" + v.name + "': powertrain.shafts[]: bad port ref \"" + ref + "\"");
                    } else {
                        const auto dot = ref.find('.');
                        if (dot == std::string::npos || dot == 0 || dot + 1 == ref.size()) {
                            throw std::runtime_error("vehicle '" + v.name + "': powertrain.shafts[]: bad port ref \"" + ref +
                                                      "\" (expected \"<id>.<port>\" or \"wheel:<name>\")");
                        }
                        pr.component = ref.substr(0, dot);
                        pr.port = ref.substr(dot + 1);
                    }
                    s.ports.push_back(pr);
                }
                // PHYS-033 P4 I4 C13: stiffness/damping are validated here
                // whatever the compliance (a negative value is a data error
                // either way); under "rigid" they are then simply ignored.
                s.stiffness = sj.value("stiffness", real(0.0));
                s.damping = sj.value("damping", real(0.0));
                if (!(s.stiffness >= real(0.0))) throw std::runtime_error(sctx + ": 'stiffness' must be >= 0");
                if (!(s.damping >= real(0.0))) throw std::runtime_error(sctx + ": 'damping' must be >= 0");
                if (s.damping > real(0.0) && s.stiffness == real(0.0)) {
                    throw std::runtime_error(sctx + ": 'damping' > 0 requires 'stiffness' > 0 (a pure damper shaft is not supported)");
                }
                p.shafts.push_back(s);
            }
        }

        if (pj.contains("compliance")) {
            const std::string c = pj.at("compliance").get<std::string>();
            if (c == "rigid") {
                p.compliance = ps::drivetrain::Compliance::Rigid;
            } else if (c == "compliant") {
                p.compliance = ps::drivetrain::Compliance::Compliant;
            } else {
                throw std::runtime_error("vehicle '" + v.name + "': powertrain.compliance must be \"rigid\" or \"compliant\", got \"" + c + "\"");
            }
        }
        if (p.compliance == ps::drivetrain::Compliance::Compliant) {
            bool any_stiff = false;
            for (std::size_t si = 0; si < p.shafts.size(); ++si) {
                if (p.shafts[si].stiffness <= real(0.0)) continue;
                any_stiff = true;
                if (p.shafts[si].ports.size() != 2) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.shafts[" + std::to_string(si) +
                                             "]: a compliant shaft (stiffness > 0) must list exactly 2 ports, got " +
                                             std::to_string(p.shafts[si].ports.size()));
                }
            }
            if (!any_stiff) {
                throw std::runtime_error("vehicle '" + v.name + "': powertrain.compliance is \"compliant\" but no shaft has stiffness > 0");
            }
        }
        p.rotor_reaction = pj.value("rotor_reaction", true);
    }
    v.powertrain = p;

    // --- controllers (plan section 4.2/9.1) ---------------------------------
    if (j.contains("controllers")) {
        for (const auto& cj : j.at("controllers")) {
            if (!cj.contains("id") || !cj.at("id").is_string() || cj.at("id").get<std::string>().empty()) {
                throw std::runtime_error("vehicle '" + v.name + "': controllers[]: 'id' is required and must be a non-empty string");
            }
            const std::string id = cj.at("id").get<std::string>();
            if (!cj.contains("type") || !cj.at("type").is_string()) {
                throw std::runtime_error("vehicle '" + v.name + "': controllers['" + id + "']: 'type' is required");
            }
            const std::string type = cj.at("type").get<std::string>();
            const std::string ctx = "vehicle '" + v.name + "': controllers['" + id + "']";
            if (type != "manual_tcu") throw std::runtime_error(ctx + ": unknown type \"" + type + "\"");

            ps::drivetrain::ManualTcuDesc md;
            if (!cj.contains("gearbox") || !cj.contains("clutch") || !cj.contains("engine")) {
                throw std::runtime_error(ctx + ": 'gearbox', 'clutch' and 'engine' (component id references) are all required");
            }
            md.gearbox = cj.at("gearbox").get<std::string>();
            md.clutch = cj.at("clutch").get<std::string>();
            md.engine = cj.at("engine").get<std::string>();
            if (std::find(component_ids.begin(), component_ids.end(), md.gearbox) == component_ids.end()) {
                throw std::runtime_error(ctx + ": references unknown component '" + md.gearbox + "'");
            }
            if (std::find(component_ids.begin(), component_ids.end(), md.clutch) == component_ids.end()) {
                throw std::runtime_error(ctx + ": references unknown component '" + md.clutch + "'");
            }
            if (std::find(component_ids.begin(), component_ids.end(), md.engine) == component_ids.end()) {
                throw std::runtime_error(ctx + ": references unknown component '" + md.engine + "'");
            }

            if (cj.contains("auto_clutch")) {
                const auto& acj = cj.at("auto_clutch");
                md.auto_clutch.enabled = acj.value("enabled", md.auto_clutch.enabled);
                md.auto_clutch.bite_rpm = acj.value("bite_rpm", md.auto_clutch.bite_rpm);
                md.auto_clutch.full_rpm_min = acj.value("full_rpm_min", md.auto_clutch.full_rpm_min);
                md.auto_clutch.full_rpm_max = acj.value("full_rpm_max", md.auto_clutch.full_rpm_max);
                md.auto_clutch.lock_slip_rpm = acj.value("lock_slip_rpm", md.auto_clutch.lock_slip_rpm);
                md.auto_clutch.anti_stall_rpm = acj.value("anti_stall_rpm", md.auto_clutch.anti_stall_rpm);
                md.auto_clutch.engage_rate_per_s = acj.value("engage_rate_per_s", md.auto_clutch.engage_rate_per_s);
                md.auto_clutch.release_rate_per_s = acj.value("release_rate_per_s", md.auto_clutch.release_rate_per_s);
                md.auto_clutch.open_time_s = acj.value("open_time_s", md.auto_clutch.open_time_s);
                md.auto_clutch.close_time_s = acj.value("close_time_s", md.auto_clutch.close_time_s);
                md.auto_clutch.anti_stall_release_rate_per_s =
                    acj.value("anti_stall_release_rate_per_s", md.auto_clutch.anti_stall_release_rate_per_s);
                md.auto_clutch.rolling_reengage_rpm = acj.value("rolling_reengage_rpm", md.auto_clutch.rolling_reengage_rpm);
                if (!(md.auto_clutch.rolling_reengage_rpm > md.auto_clutch.anti_stall_rpm)) {
                    throw std::runtime_error(ctx + ".auto_clutch: rolling_reengage_rpm must be > anti_stall_rpm");
                }
                md.auto_clutch.sync_time_s = acj.value("sync_time_s", md.auto_clutch.sync_time_s);
                md.auto_clutch.max_sync_torque_nm = acj.value("max_sync_torque_nm", md.auto_clutch.max_sync_torque_nm);
                if (!(md.auto_clutch.sync_time_s > 0.0)) {
                    throw std::runtime_error(ctx + ".auto_clutch: sync_time_s must be > 0");
                }
                if (!(md.auto_clutch.max_sync_torque_nm > 0.0)) {
                    throw std::runtime_error(ctx + ".auto_clutch: max_sync_torque_nm must be > 0");
                }
                if (!(md.auto_clutch.bite_rpm < md.auto_clutch.full_rpm_min && md.auto_clutch.full_rpm_min <= md.auto_clutch.full_rpm_max)) {
                    throw std::runtime_error(ctx + ".auto_clutch: must satisfy bite_rpm < full_rpm_min <= full_rpm_max");
                }
                if (!(md.auto_clutch.anti_stall_rpm < md.auto_clutch.bite_rpm)) {
                    throw std::runtime_error(ctx + ".auto_clutch: anti_stall_rpm must be < bite_rpm");
                }
            }
            if (cj.contains("auto_blip")) {
                const auto& abj = cj.at("auto_blip");
                md.auto_blip.enabled = abj.value("enabled", md.auto_blip.enabled);
                md.auto_blip.kp_per_rpm = abj.value("kp_per_rpm", md.auto_blip.kp_per_rpm);
                md.auto_blip.max_throttle = abj.value("max_throttle", md.auto_blip.max_throttle);
                md.auto_blip.upshift_cut = abj.value("upshift_cut", md.auto_blip.upshift_cut);
                md.auto_blip.upshift_cut_release_slip_rpm = abj.value("upshift_cut_release_slip_rpm", md.auto_blip.upshift_cut_release_slip_rpm);
                md.auto_blip.upshift_cut_max_hold_s = abj.value("upshift_cut_max_hold_s", md.auto_blip.upshift_cut_max_hold_s);
                md.auto_blip.upshift_torque_ramp_s = abj.value("upshift_torque_ramp_s", md.auto_blip.upshift_torque_ramp_s);
                if (!(md.auto_blip.upshift_cut_release_slip_rpm > 0.0)) {
                    throw std::runtime_error(ctx + ".auto_blip: upshift_cut_release_slip_rpm must be > 0");
                }
                if (!(md.auto_blip.upshift_cut_max_hold_s > 0.0)) {
                    throw std::runtime_error(ctx + ".auto_blip: upshift_cut_max_hold_s must be > 0");
                }
                if (!(md.auto_blip.upshift_torque_ramp_s >= 0.0)) {
                    throw std::runtime_error(ctx + ".auto_blip: upshift_torque_ramp_s must be >= 0");
                }
            }
            // PHYS-033 P4 I7a C3 (plan section 3.4.5, "which TCU
            // intervention lives where"): a mechanical-linkage engine has no
            // wire back to the pedal, so ManualTcu's blip (kp_per_rpm) and
            // the post-upshift pedal-ceiling ramp/floor (max_throttle,
            // upshift_torque_ramp_s) - all three only ever reach the engine
            // as ctx.tcu_throttle_floor/tcu_throttle_ceiling, which
            // ThrottleStage::evaluate ignores outright for a Mechanical
            // stage (floor/ceiling are fixed at 0/1) - cannot be carried.
            // The check is on the EFFECTIVE VALUES (post-default, whether or
            // not an 'auto_blip' block was present at all - md.auto_blip is
            // already default-constructed with kp_per_rpm=0.002,
            // max_throttle=1.0, upshift_torque_ramp_s=0.10, see
            // controller_desc.h's AutoBlipDesc), NOT on 'enabled': a
            // mechanical vehicle with 'enabled: true' but all three zeroed
            // is perfectly valid (it does nothing, same as if the block were
            // absent on an electronic engine with those zeroed) and MUST
            // load. The upshift ignition cut (auto_blip.upshift_cut) is
            // UNAFFECTED - TcuRequest::ignition_cut reaches the engine
            // directly (never through ThrottleCommand) and the engine
            // applies it the same way regardless of throttle kind, so it
            // still works on both.
            if (effective_throttle_kind(p, md.engine) == ps::drivetrain::ThrottleInput::Mechanical &&
                (md.auto_blip.kp_per_rpm > real(0.0) || md.auto_blip.max_throttle > real(0.0) ||
                 md.auto_blip.upshift_torque_ramp_s > real(0.0))) {
                throw std::runtime_error(
                    ctx + ".auto_blip: engine '" + md.engine +
                    "' has a mechanical throttle - a mechanical throttle cannot open the plate (blip) or close it "
                    "against the driver (ceiling/ramp); the upshift ignition cut stays available (set those three to 0)");
            }
            if (cj.contains("auto_shift")) {
                const auto& asj = cj.at("auto_shift");
                md.auto_shift.enabled = asj.value("enabled", md.auto_shift.enabled);
                md.auto_shift.up_rpm_at_zero_throttle = asj.value("up_rpm_at_zero_throttle", md.auto_shift.up_rpm_at_zero_throttle);
                md.auto_shift.up_rpm_at_full_throttle = asj.value("up_rpm_at_full_throttle", md.auto_shift.up_rpm_at_full_throttle);
                md.auto_shift.down_rpm_at_zero_throttle = asj.value("down_rpm_at_zero_throttle", md.auto_shift.down_rpm_at_zero_throttle);
                md.auto_shift.down_rpm_at_full_throttle = asj.value("down_rpm_at_full_throttle", md.auto_shift.down_rpm_at_full_throttle);
                md.auto_shift.min_time_in_gear_s = asj.value("min_time_in_gear_s", md.auto_shift.min_time_in_gear_s);
            }
            md.max_pending_steps = cj.value("max_pending_steps", md.max_pending_steps);

            v.controllers.push_back({id, md});
        }
    }
    if (v.controllers.size() > 1) {
        throw std::runtime_error("vehicle '" + v.name + "': at most one controller (manual_tcu) is supported in I3");
    }

    return v;
}

} // namespace ps::io
