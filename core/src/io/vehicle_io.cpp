#include "ps/io/vehicle_io.h"

#include "../engine_cycle/map_cache.h"
#include "../vehicle/tir_parser.h"
#include "ps/drivetrain/table2d.h"
#include "ps/engine_cycle/firing_order.h"
#include "ps/io/fuel_io.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
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
            if (const auto* e = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) return e->throttle_input;
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

// PHYS-033 P4 I7a C5: renamed from read_engine_fields (I3-I7a C4's only
// engine kind) - read_engine_fields below is now a thin dispatcher on
// "kind" so a second engine kind ("simulated", I7b) can sit next to this
// one. Every torque_map-specific field/check is unchanged.
// PHYS-033 P4 I7a C6: `resolved_fuel_paths_out` (optional) receives every
// fuel file path this engine lists ("fuels" entries, or the one "fuel"
// shorthand), resolved exactly as they were loaded - load_vehicle_json's
// fluid-link check compares a tank's own fuel file against these with
// std::filesystem::equivalent (FuelDesc itself carries no path).
// PHYS-033 P4 I7b C10 (I7 plan section 2.1): keys that exist only on a
// "simulated" engine. A torque_map file carrying any of them is rejected -
// it almost certainly meant "kind": "simulated" and would otherwise load as
// a plain torque map with those fields silently ignored.
constexpr const char* kSimulatedOnlyEngineKeys[] = {
    "architecture",  "bore_mm",           "stroke_mm",         "cylinders",
    "rotor_radius_mm", "eccentricity_mm", "rotor_width_mm",    "rotors",
    "intake_manifold_volume_l", "throttle_body", "throttle_actuator_tau_s", "ecu", "maps", "cycle",
};

ps::drivetrain::TorqueMapEngineDesc read_torque_map_fields(const json& j, const std::string& ctx, const std::string& engine_dir,
                                                           std::vector<std::string>* resolved_fuel_paths_out = nullptr) {
    for (const char* key : kSimulatedOnlyEngineKeys) {
        if (j.contains(key)) {
            throw std::runtime_error(ctx + ": '" + std::string(key) +
                                      "' is a \"simulated\" engine field and is not allowed on a \"torque_map\" engine "
                                      "(set \"kind\": \"simulated\" or remove it)");
        }
    }
    if (j.contains("idle_control") && j.at("idle_control").is_object() && j.at("idle_control").contains("bypass_area_mm2")) {
        throw std::runtime_error(ctx + ": idle_control.bypass_area_mm2 is a \"simulated\" engine field (a torque_map "
                                  "engine has no throttle plate or bypass valve)");
    }
    ps::drivetrain::TorqueMapEngineDesc e;
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
        // R6: a negative gain flips the PI loop's sign (destabilising, not
        // merely a weaker response) - reject rather than silently misbehave.
        if (e.idle.kp_per_rpm < real(0.0)) {
            throw std::runtime_error(ctx + ": idle_control.kp_per_rpm must be >= 0, got " + std::to_string(e.idle.kp_per_rpm));
        }
        if (e.idle.ki_per_rpm_s < real(0.0)) {
            throw std::runtime_error(ctx + ": idle_control.ki_per_rpm_s must be >= 0, got " + std::to_string(e.idle.ki_per_rpm_s));
        }
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
        // R6: a negative hysteresis reverses the FuelCut latch's restore
        // direction (restores ABOVE the cut point instead of below it).
        if (e.limiter.hysteresis_rpm < real(0.0)) {
            throw std::runtime_error(ctx + ": limiter.hysteresis_rpm must be >= 0, got " + std::to_string(e.limiter.hysteresis_rpm));
        }
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
        // R6: EngineTorqueMap/SimulatedEngine both divide by free_speed_rpm
        // (converted to rad/s) to build the starter's speed-torque line -
        // zero or negative divides by zero/flips its sign at runtime.
        if (!(e.starter.free_speed_rpm > real(0.0))) {
            throw std::runtime_error(ctx + ": starter.free_speed_rpm must be > 0, got " + std::to_string(e.starter.free_speed_rpm));
        }
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

    // PHYS-033 P4 I7a C5d: "fuels" - a plain list of fuel path strings for
    // the crude no-map estimate (no per-entry blend weight; a hypothetical
    // C5e was cancelled/dropped per owner ruling 2026-09-27). Mutually
    // exclusive with "fuel"/"fuel_map": a calibrated map is keyed to exactly
    // the one fuel named by "fuel" above, while "fuels" is for an engine
    // with no map that simply lists what it can run on. fuels[0] is the
    // ACTIVE fuel (order matters, no blending) - resolved the same way as
    // "fuel" above (relative to engine_dir). Checked and loaded BEFORE the
    // "fuel"/"fuel_map" block below so its own mutual-exclusion messages
    // fire instead of "fuel_map" block's generic "'fuel_map' requires a
    // 'fuel'" rejection when both "fuels" and "fuel_map" are present with
    // no "fuel" shorthand.
    if (j.contains("fuels")) {
        if (j.contains("fuel")) {
            throw std::runtime_error(ctx + ": 'fuels' and 'fuel' are mutually exclusive (a calibrated fuel_map is "
                                      "keyed to one fuel via 'fuel'; 'fuels' is for an engine with no map)");
        }
        if (j.contains("fuel_map")) {
            throw std::runtime_error(ctx + ": 'fuels' and 'fuel_map' are mutually exclusive (a fuel map needs a "
                                      "single calibrated 'fuel', not a list)");
        }
        if (!j.at("fuels").is_array() || j.at("fuels").empty()) {
            throw std::runtime_error(ctx + ": 'fuels' must be a non-empty array of path strings");
        }
        // N12 review fix: duplicate detection now uses
        // std::filesystem::equivalent instead of a weakly_canonical-then-
        // string-compare pass - equivalent() resolves the filesystem's own
        // notion of "same file" (symlinks, "./"/"x/../"-style relative
        // detours that weakly_canonical's plain string form could miss if
        // two different textual forms normalise to strings that still
        // differ, e.g. across a mount point) rather than trusting a second
        // string comparison of an already-normalised path.
        std::vector<std::string> resolved_paths;
        for (const auto& v : j.at("fuels")) {
            if (!v.is_string() || v.get<std::string>().empty()) {
                throw std::runtime_error(ctx + ": 'fuels' entries must be non-empty path strings");
            }
            const std::string fuel_path = resolve_path(engine_dir, v.get<std::string>());
            for (const std::string& seen : resolved_paths) {
                bool same = false;
                try {
                    same = std::filesystem::equivalent(fuel_path, seen);
                } catch (const std::filesystem::filesystem_error&) {
                    same = (fuel_path == seen); // one or both don't exist yet; fall back to exact string match
                }
                if (same) {
                    throw std::runtime_error(ctx + ": 'fuels' contains a duplicate fuel reference (" +
                                              v.get<std::string>() + ")");
                }
            }
            resolved_paths.push_back(fuel_path);
            e.fuels.push_back(ps::io::load_fuel_json(fuel_path));
        }
        if (resolved_fuel_paths_out != nullptr) *resolved_fuel_paths_out = resolved_paths;
    }

    // PHYS-033 P4 I7a C5 (plan section 2.3): optional fuel map. "fuel" is a
    // path (relative to this engine JSON's own directory - engine_dir,
    // component_ref_dir's result above - the referenced engine file's own
    // directory when this engine block itself came from a "ref", else the
    // vehicle file's own directory) to a fuel JSON, loaded via
    // ps::io::load_fuel_json. "fuel_map" is {rpm, load, fuel_flow_g_s
    // (rows=rpm, columns=load), source}. Neither is required; has_fuel_map
    // stays false (default) unless a full, valid fuel_map is present.
    // PHYS-033 P4 I7a C5d review fix (B1): "fuel" is TRUE SHORTHAND for a
    // one-entry "fuels" list, not a no-op without a map - e.fuels = {e.fuel}
    // below runs unconditionally whenever "fuel" is present (map or not), so
    // an engine with "fuel" and no "fuel_map" gets the crude no-map estimate
    // exactly like one written with "fuels": ["<same path>"]. "fuel_map"
    // with no "fuel" is still rejected outright (a fuel map with no LHV
    // cannot be turned into energy).
    if (j.contains("fuel_map") && !j.contains("fuel")) {
        throw std::runtime_error(ctx + ": 'fuel_map' requires a 'fuel' reference (a fuel map with no LHV cannot be "
                                  "turned into energy)");
    }
    if (j.contains("fuel")) {
        if (!j.at("fuel").is_string() || j.at("fuel").get<std::string>().empty()) {
            throw std::runtime_error(ctx + ": 'fuel' must be a non-empty path string");
        }
        const std::string fuel_path = resolve_path(engine_dir, j.at("fuel").get<std::string>());
        e.fuel = ps::io::load_fuel_json(fuel_path);
        // B1: "fuels" and "fuel" are mutually exclusive at the JSON-key
        // level (checked above, before this block runs), so e.fuels is
        // still empty here - safe to desugar unconditionally.
        e.fuels = {e.fuel};
        if (resolved_fuel_paths_out != nullptr) *resolved_fuel_paths_out = {fuel_path};
    }
    if (j.contains("fuel_map")) {
        const auto& fmj = j.at("fuel_map");
        if (!fmj.contains("rpm") || !fmj.at("rpm").is_array()) {
            throw std::runtime_error(ctx + ".fuel_map: 'rpm' array is required");
        }
        if (!fmj.contains("load") || !fmj.at("load").is_array()) {
            throw std::runtime_error(ctx + ".fuel_map: 'load' array is required");
        }
        if (!fmj.contains("fuel_flow_g_s") || !fmj.at("fuel_flow_g_s").is_array()) {
            throw std::runtime_error(ctx + ".fuel_map: 'fuel_flow_g_s' array is required");
        }
        const std::string source = fmj.value("source", std::string());
        if (source.empty()) throw std::runtime_error(ctx + ".fuel_map: 'source' is required (PLAN.md section 8)");

        ps::drivetrain::Table2D t;
        for (const auto& v : fmj.at("rpm")) t.x.push_back(v.get<real>());
        for (const auto& v : fmj.at("load")) t.y.push_back(v.get<real>());
        if (t.x.size() < 2) throw std::runtime_error(ctx + ".fuel_map: 'rpm' needs at least 2 points");
        if (t.y.size() < 2) throw std::runtime_error(ctx + ".fuel_map: 'load' needs at least 2 points");
        for (std::size_t i = 1; i < t.x.size(); ++i) {
            if (!(t.x[i] > t.x[i - 1])) throw std::runtime_error(ctx + ".fuel_map: 'rpm' must be strictly ascending");
        }
        for (std::size_t i = 1; i < t.y.size(); ++i) {
            if (!(t.y[i] > t.y[i - 1])) throw std::runtime_error(ctx + ".fuel_map: 'load' must be strictly ascending");
        }
        if (t.y.front() != real(0.0)) throw std::runtime_error(ctx + ".fuel_map: 'load' must start at exactly 0.0");
        if (t.y.back() != real(1.0)) throw std::runtime_error(ctx + ".fuel_map: 'load' must end at exactly 1.0");

        const auto& rows = fmj.at("fuel_flow_g_s");
        if (rows.size() != t.x.size()) {
            throw std::runtime_error(ctx + ".fuel_map: 'fuel_flow_g_s' must have one row per 'rpm' point (" +
                                      std::to_string(t.x.size()) + "), got " + std::to_string(rows.size()));
        }
        t.values.reserve(t.x.size() * t.y.size());
        for (std::size_t ix = 0; ix < rows.size(); ++ix) {
            const auto& row = rows[ix];
            if (!row.is_array() || row.size() != t.y.size()) {
                throw std::runtime_error(ctx + ".fuel_map: 'fuel_flow_g_s' row " + std::to_string(ix) +
                                          " must have one value per 'load' point (" + std::to_string(t.y.size()) + ")");
            }
            for (std::size_t iy = 0; iy < row.size(); ++iy) {
                // PHYS-033 P4 I7a C5 review fix: check the JSON value's own
                // type before .get<real>() - nlohmann::json::type_error (e.g.
                // a string or null entry) would otherwise propagate uncaught,
                // unlike every other rejection in this loader which throws a
                // clean std::runtime_error with context.
                if (!row[iy].is_number()) {
                    throw std::runtime_error(ctx + ".fuel_map: fuel_flow_g_s[" + std::to_string(ix) + "][" +
                                              std::to_string(iy) + "] must be a number");
                }
                const real val = row[iy].get<real>();
                if (!(val >= real(0.0)) || !std::isfinite(val)) {
                    throw std::runtime_error(ctx + ".fuel_map: fuel_flow_g_s[" + std::to_string(ix) + "][" +
                                              std::to_string(iy) + "] must be >= 0 and finite, got " + std::to_string(val));
                }
                // load == 0 must burn exactly 0 fuel - DFCO/ignition-cut/
                // pedal-0 substeps need an EXACT zero, not merely near-zero
                // (F3's discrimination check reads it as an equality).
                if (iy == 0 && val != real(0.0)) {
                    throw std::runtime_error(ctx + ".fuel_map: fuel_flow_g_s[" + std::to_string(ix) +
                                              "][0] (load 0) must be exactly 0.0, got " + std::to_string(val));
                }
                t.values.push_back(val);
            }
        }
        const std::string table_err = t.validate();
        if (!table_err.empty()) throw std::runtime_error(ctx + ".fuel_map: " + table_err);

        // Efficiency check (plan section 2.3): theta*(w(rpm)-m(rpm))*omega
        // must never exceed mdot*LHV, or the map implies >100% thermal
        // efficiency. Sampled at every map node, every map-cell midpoint
        // (bilinear interpolation can under/overshoot a saturating curve
        // between nodes) and every WOT/motoring curve breakpoint crossed
        // with every map load node (a curve knot is where w(rpm)-m(rpm)'s
        // own slope can kink, the worst case for a fixed-load line).
        constexpr real kTwoPiOver60 = real(2.0) * real(3.14159265358979323846) / real(60.0);
        auto check_point = [&](real rpm, real load) {
            if (rpm <= real(0.0)) return; // omega == 0: zero mechanical power at any load, never a violation
            const real w = e.wot_torque_nm_vs_rpm.evaluate(rpm);
            const real m = e.motoring_torque_nm_vs_rpm.evaluate(rpm);
            const real omega = rpm * kTwoPiOver60;
            const real mech_power_w = load * (w - m) * omega;
            const real mdot_kg_s = t.value(rpm, load) / real(1000.0);
            const real fuel_power_w = mdot_kg_s * e.fuel.lhv_j_kg;
            if (mech_power_w > fuel_power_w) {
                throw std::runtime_error(ctx + ".fuel_map: implies >100% thermal efficiency at rpm=" +
                                          std::to_string(rpm) + ", load=" + std::to_string(load) + " (mechanical " +
                                          std::to_string(mech_power_w) + " W > fuel " + std::to_string(fuel_power_w) +
                                          " W)");
            }
        };
        for (real rpm_node : t.x) {
            for (real load_node : t.y) check_point(rpm_node, load_node);
        }
        for (std::size_t ix = 1; ix < t.x.size(); ++ix) {
            const real rpm_mid = (t.x[ix - 1] + t.x[ix]) * real(0.5);
            for (std::size_t iy = 1; iy < t.y.size(); ++iy) {
                check_point(rpm_mid, (t.y[iy - 1] + t.y[iy]) * real(0.5));
            }
        }
        for (real rpm_bp : e.wot_torque_nm_vs_rpm.x) {
            for (real load_node : t.y) check_point(rpm_bp, load_node);
        }
        for (real rpm_bp : e.motoring_torque_nm_vs_rpm.x) {
            for (real load_node : t.y) check_point(rpm_bp, load_node);
        }

        e.fuel_map = std::move(t);
        e.has_fuel_map = true;
    }

    // PHYS-033 P4 I7a C5d: "ignition" - spark vs. compression (an ENGINE
    // property, see ps::drivetrain::EngineIgnition's own comment). Required
    // whenever this engine has any fuel model (a map or a "fuels" list) -
    // the crude no-map estimate's default indicated efficiency depends on
    // it and a silent default would be a silent physics choice, not a loud
    // one (same "no silent default" convention as the fuel-file "ignition"
    // field C5c briefly added and then reversed onto this file's own
    // "ignition" instead). Stored (never required, never rejected) even
    // with no fuel model at all - harmlessly unused.
    if (j.contains("ignition")) {
        // N12 review fix: check the JSON value's own type before .get<string>()
        // - nlohmann::json::type_error would otherwise propagate uncaught,
        // unlike every other rejection in this loader which throws a clean
        // std::runtime_error with context (same pattern as the fuel_map
        // fuel_flow_g_s cell check above).
        if (!j.at("ignition").is_string()) {
            throw std::runtime_error(ctx + ": ignition must be a string (\"spark\" or \"compression\")");
        }
        const std::string ign = j.at("ignition").get<std::string>();
        if (ign == "spark") {
            e.ignition = ps::drivetrain::EngineIgnition::Spark;
        } else if (ign == "compression") {
            e.ignition = ps::drivetrain::EngineIgnition::Compression;
        } else {
            throw std::runtime_error(ctx + ": ignition must be \"spark\" or \"compression\", got \"" + ign + "\"");
        }
        e.has_ignition = true;
    }
    const bool engine_has_fuel_model = e.has_fuel_map || !e.fuels.empty();
    if (engine_has_fuel_model && !e.has_ignition) {
        throw std::runtime_error(ctx + ": an engine with a fuel ('fuel'+'fuel_map' or 'fuels') requires 'ignition' "
                                  "(\"spark\" or \"compression\")");
    }

    // PHYS-033 P4 I7a C5d: "indicated_efficiency" - override for the crude
    // no-map model's eta_i (see TorqueMapEngineDesc's own comment).
    // Meaningless without the crude no-map path (nothing reads it there), so
    // rejected rather than silently ignored: both when there is no fuel
    // model at all, AND (N9 review fix) when a "fuel_map" is present - a
    // calibrated map is never gated by eta_i, so setting this alongside one
    // would silently do nothing.
    if (j.contains("indicated_efficiency")) {
        if (!engine_has_fuel_model) {
            throw std::runtime_error(ctx + ": indicated_efficiency requires a fuel model ('fuel_map' or 'fuels')");
        }
        if (e.has_fuel_map) {
            throw std::runtime_error(ctx + ": indicated_efficiency is meaningless with a 'fuel_map' present (the "
                                      "map is never gated by eta_i) - remove one or the other");
        }
        if (!j.at("indicated_efficiency").is_number()) {
            throw std::runtime_error(ctx + ": indicated_efficiency must be a number");
        }
        e.indicated_efficiency = j.at("indicated_efficiency").get<real>();
        if (!std::isfinite(e.indicated_efficiency) || !(e.indicated_efficiency > real(0.0)) ||
            !(e.indicated_efficiency < real(1.0))) {
            throw std::runtime_error(ctx + ": indicated_efficiency must be in (0, 1), got " +
                                      std::to_string(e.indicated_efficiency));
        }
    }

    return e;
}

// --- PHYS-033 P4 I7b C10: "kind": "simulated" -------------------------------

// A finite JSON number at `key` (required), with a clean message otherwise.
real require_number(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key)) throw std::runtime_error(ctx + ": '" + std::string(key) + "' is required");
    if (!j.at(key).is_number()) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be a number");
    const real v = j.at(key).get<real>();
    if (!std::isfinite(v)) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be finite");
    return v;
}

real require_positive(const json& j, const char* key, const std::string& ctx) {
    const real v = require_number(j, key, ctx);
    if (!(v > real(0.0))) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be > 0, got " + std::to_string(v));
    return v;
}

int require_positive_int(const json& j, const char* key, const std::string& ctx) {
    if (!j.contains(key)) throw std::runtime_error(ctx + ": '" + std::string(key) + "' is required");
    if (!j.at(key).is_number_integer()) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be an integer");
    const long long v = j.at(key).get<long long>();
    if (v < 1 || v > 64) throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be in [1, 64], got " + std::to_string(v));
    return static_cast<int>(v);
}

// An engine-map table: {"rpm": [...], "<y_key>": [...], "values": [[...] per rpm]}.
// y and values are multiplied by y_scale / value_scale (unit conversion to
// SI). rpm >= 0, strictly ascending; every entry a finite number; at least
// 2 points per axis (Table2D::validate()).
ps::drivetrain::Table2D read_engine_table(const json& j, const std::string& what, const char* y_key, real y_scale,
                                          real value_scale) {
    if (!j.is_object()) throw std::runtime_error(what + " must be an object {rpm, " + std::string(y_key) + ", values}");
    if (!j.contains("rpm") || !j.at("rpm").is_array()) throw std::runtime_error(what + ": 'rpm' array is required");
    if (!j.contains(y_key) || !j.at(y_key).is_array()) {
        throw std::runtime_error(what + ": '" + std::string(y_key) + "' array is required");
    }
    if (!j.contains("values") || !j.at("values").is_array()) throw std::runtime_error(what + ": 'values' array is required");
    ps::drivetrain::Table2D t;
    for (const auto& v : j.at("rpm")) {
        if (!v.is_number()) throw std::runtime_error(what + ": 'rpm' entries must be numbers");
        t.x.push_back(v.get<real>());
    }
    for (const auto& v : j.at(y_key)) {
        if (!v.is_number()) throw std::runtime_error(what + ": '" + std::string(y_key) + "' entries must be numbers");
        t.y.push_back(v.get<real>() * y_scale);
    }
    const auto& rows = j.at("values");
    if (rows.size() != t.x.size()) {
        throw std::runtime_error(what + ": 'values' must have one row per 'rpm' point (" + std::to_string(t.x.size()) +
                                  "), got " + std::to_string(rows.size()));
    }
    for (std::size_t ix = 0; ix < rows.size(); ++ix) {
        const auto& row = rows[ix];
        if (!row.is_array() || row.size() != t.y.size()) {
            throw std::runtime_error(what + ": 'values' row " + std::to_string(ix) + " must have one value per '" +
                                      std::string(y_key) + "' point (" + std::to_string(t.y.size()) + ")");
        }
        for (std::size_t iy = 0; iy < row.size(); ++iy) {
            if (!row[iy].is_number()) {
                throw std::runtime_error(what + ": values[" + std::to_string(ix) + "][" + std::to_string(iy) + "] must be a number");
            }
            t.values.push_back(row[iy].get<real>() * value_scale);
        }
    }
    const std::string err = t.validate();
    if (!err.empty()) throw std::runtime_error(what + ": " + err);
    if (t.x.front() < real(0.0)) throw std::runtime_error(what + ": 'rpm' must be >= 0");
    return t;
}

void require_table_values(const ps::drivetrain::Table2D& t, const std::string& what, real lo, bool lo_inclusive, real hi,
                          bool hi_inclusive) {
    for (std::size_t i = 0; i < t.values.size(); ++i) {
        const real v = t.values[i];
        const bool lo_ok = lo_inclusive ? (v >= lo) : (v > lo);
        const bool hi_ok = hi_inclusive ? (v <= hi) : (v < hi);
        if (!lo_ok || !hi_ok) {
            throw std::runtime_error(what + ": every value must be in " + std::string(lo_inclusive ? "[" : "(") +
                                      std::to_string(lo) + ", " + std::to_string(hi) + std::string(hi_inclusive ? "]" : ")") +
                                      ", got " + std::to_string(v) + " at index " + std::to_string(i));
        }
    }
}

void require_positive_y_axis(const ps::drivetrain::Table2D& t, const std::string& what, const char* y_key) {
    if (!(t.y.front() > real(0.0))) throw std::runtime_error(what + ": '" + std::string(y_key) + "' must be > 0");
}

// PHYS-033 P4 I7c C16a: a simulated engine's optional "cycle" block - the
// 0D cycle model's inputs (ps/drivetrain/engine_cycle_desc.h) from which the
// loader generates every runtime map the "maps" block does not give
// (docs/data_formats/engine.md). Geometry (bore, stroke, cylinders) is the
// engine's own - never repeated here. Lengths in mm, port volumes in cc,
// pressures in kPa, angles in crank degrees; converted to SI.
constexpr const char* kCycleKeys[] = {
    "source",          "rod_mm",           "pin_offset_mm",       "compression_ratio",  "intake_valves",
    "exhaust_valves",  "intake_valve_d_mm", "exhaust_valve_d_mm", "intake_lift",        "exhaust_lift",
    "runner_length_mm", "runner_diameter_mm", "intake_port_cc",   "exhaust_port_cc",    "cd_vs_l_over_d",
    "cam",             "wiebe_a",          "wiebe_m",             "burn_duration_deg",  "ca50_mbt_deg_atdc",
    "woschni_c1_gx",   "woschni_c1_cc",    "woschni_c2",          "wall_temperature_k", "head_area_factor",
    "combustion_eff_lean", "exhaust_backflow_temperature_k", "friction", "grid_rpm", "grid_p_im_kpa",
    // ps_audio A1 (out/audio_plan.md section 1/3): firing phasing + bank
    // geometry. Descriptor-only for now - see CycleModelDesc's own field
    // comments (ps/drivetrain/engine_cycle_desc.h).
    "firing_order", "firing_angles_deg", "banks", "bank_angle_deg",
    // I7c C17a follow-up: Tier 1b ECU spark schedule (then the ecu block's
    // own spark_advance_deg must be absent - it is filled from this one) and
    // Tier 2 flame-development delay.
    "spark_advance_deg", "ignition_delay_deg",
    // ps_audio A4 (out/audio_plan.md section 3.1): the optional intake/
    // exhaust pipe pair, the engine's own default_installation and the
    // pipe cell-size "resolution" block. See read_intake_block/
    // read_exhaust_block/read_resolution_block below.
    "intake", "exhaust", "default_installation", "resolution",
};

std::vector<real> read_cycle_grid(const json& cj, const char* key, real scale, const std::string& cctx) {
    if (!cj.contains(key) || !cj.at(key).is_array()) throw std::runtime_error(cctx + ": '" + std::string(key) + "' array is required");
    std::vector<real> v;
    for (const auto& x : cj.at(key)) {
        if (!x.is_number()) throw std::runtime_error(cctx + ": '" + std::string(key) + "' entries must be numbers");
        v.push_back(x.get<real>() * scale);
    }
    if (v.size() < 2) throw std::runtime_error(cctx + ": '" + std::string(key) + "' needs at least 2 values");
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (!(v[i] > real(0.0)) || !std::isfinite(v[i])) {
            throw std::runtime_error(cctx + ": '" + std::string(key) + "' values must be > 0 and finite");
        }
        if (i > 0 && !(v[i] > v[i - 1])) throw std::runtime_error(cctx + ": '" + std::string(key) + "' must be strictly ascending");
    }
    return v;
}

ps::drivetrain::ValveLiftProfile read_cycle_lift(const json& cj, const char* key, const std::string& cctx) {
    const std::string lctx = cctx + "." + key;
    if (!cj.contains(key) || !cj.at(key).is_object()) throw std::runtime_error(cctx + ": '" + std::string(key) + "' object is required");
    const auto& lj = cj.at(key);
    if (!lj.contains("lift_mm_vs_deg")) throw std::runtime_error(lctx + ": 'lift_mm_vs_deg' is required");
    ps::drivetrain::ValveLiftProfile p;
    p.lift_m_vs_deg = read_curve(lj.at("lift_mm_vs_deg"));
    if (p.lift_m_vs_deg.x.size() < 2) throw std::runtime_error(lctx + ".lift_mm_vs_deg: needs at least 2 points");
    require_ascending(p.lift_m_vs_deg, lctx + ".lift_mm_vs_deg");
    if (!(p.lift_m_vs_deg.x.back() - p.lift_m_vs_deg.x.front() < real(720.0))) {
        throw std::runtime_error(lctx + ".lift_mm_vs_deg: the span (last x - first x) must be < 720 deg");
    }
    if (p.lift_m_vs_deg.y.front() != real(0.0) || p.lift_m_vs_deg.y.back() != real(0.0)) {
        throw std::runtime_error(lctx + ".lift_mm_vs_deg: the first and last lift must be exactly 0");
    }
    for (real& y : p.lift_m_vs_deg.y) {
        if (!(y >= real(0.0)) || !std::isfinite(y)) throw std::runtime_error(lctx + ".lift_mm_vs_deg: every lift must be >= 0 and finite");
        y *= real(1e-3);
    }
    p.phase_ref_deg = require_number(lj, "phase_ref_deg", lctx);
    return p;
}

// ---- ps_audio A4 (out/audio_plan.md section 3.1/3.2): the cycle block's
// optional intake/exhaust pipe pair and the installation file format. ----

// Reads a "cylinders" field that is either the string "all" (returned as an
// empty vector - CycleModelDesc's own convention, see RunnerPipeDesc's field
// comment) or a non-empty array of 1-based cylinder numbers in [1, cylinders].
std::vector<int> read_cylinder_group(const json& gj, const char* key, int cylinders, const std::string& ctx) {
    if (!gj.contains(key)) throw std::runtime_error(ctx + ": '" + std::string(key) + "' is required");
    const auto& v = gj.at(key);
    std::vector<int> out;
    if (v.is_string()) {
        if (v.get<std::string>() != "all") {
            throw std::runtime_error(ctx + ": '" + std::string(key) + "' string value must be \"all\"");
        }
        return out; // empty = all
    }
    if (!v.is_array() || v.empty()) {
        throw std::runtime_error(ctx + ": '" + std::string(key) + "' must be \"all\" or a non-empty array of cylinder numbers");
    }
    for (const auto& cv : v) {
        if (!cv.is_number_integer()) throw std::runtime_error(ctx + ": '" + std::string(key) + "' entries must be integers");
        const int c = cv.get<int>();
        if (c < 1 || c > cylinders) {
            throw std::runtime_error(ctx + ": '" + std::string(key) + "' cylinder " + std::to_string(c) + " is out of range [1, " +
                                      std::to_string(cylinders) + "]");
        }
        out.push_back(c);
    }
    return out;
}

// Checks that a list of runner/primary-like elements (each with a
// `.cylinders` member following read_cylinder_group's "all"/list convention)
// together covers every cylinder in [1, cylinders] EXACTLY once - mirrors
// CycleModelDesc's own `banks` coverage rule (read_cycle_block below).
template <typename Elem>
void require_cylinder_coverage(const std::vector<Elem>& elems, int cylinders, const std::string& ctx) {
    bool any_all = false;
    for (const auto& e : elems) any_all = any_all || e.cylinders.empty();
    if (any_all) {
        if (elems.size() != 1) throw std::runtime_error(ctx + ": \"all\" must be the only entry");
        return;
    }
    std::vector<bool> seen(static_cast<std::size_t>(cylinders) + 1, false);
    int covered = 0;
    for (const auto& e : elems) {
        for (int c : e.cylinders) {
            if (seen[static_cast<std::size_t>(c)]) {
                throw std::runtime_error(ctx + ": cylinder " + std::to_string(c) + " is assigned to more than one entry");
            }
            seen[static_cast<std::size_t>(c)] = true;
            ++covered;
        }
    }
    if (covered != cylinders) {
        throw std::runtime_error(ctx + ": must cover every cylinder exactly once (covered " + std::to_string(covered) + " of " +
                                  std::to_string(cylinders) + ")");
    }
}

ps::drivetrain::IntakeCycleDesc read_intake_block(const json& ij, const std::string& ictx, int cylinders) {
    using namespace ps::drivetrain;
    if (!ij.is_object()) throw std::runtime_error(ictx + " must be an object");
    constexpr const char* kKeys[] = {"throttle_bore_m", "plenum_m3", "runners", "interface", "source"};
    for (auto it = ij.begin(); it != ij.end(); ++it) {
        bool known = false;
        for (const char* k : kKeys) known = known || it.key() == k;
        if (!known) throw std::runtime_error(ictx + ": unknown key '" + it.key() + "'");
    }
    IntakeCycleDesc d;
    d.source = ij.value("source", std::string());
    if (d.source.empty()) throw std::runtime_error(ictx + ": 'source' is required (PLAN.md section 8)");
    d.throttle_bore_m = require_positive(ij, "throttle_bore_m", ictx);
    d.plenum_m3 = require_positive(ij, "plenum_m3", ictx);
    if (!ij.contains("runners") || !ij.at("runners").is_array() || ij.at("runners").empty()) {
        throw std::runtime_error(ictx + ": 'runners' must be a non-empty array");
    }
    const std::string rctx = ictx + ".runners";
    for (const auto& rj : ij.at("runners")) {
        if (!rj.is_object()) throw std::runtime_error(rctx + ": each entry must be an object");
        constexpr const char* kRunnerKeys[] = {"cylinders", "length_m", "d_in_m", "d_out_m"};
        for (auto it = rj.begin(); it != rj.end(); ++it) {
            bool known = false;
            for (const char* k : kRunnerKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(rctx + ": unknown key '" + it.key() + "'");
        }
        RunnerPipeDesc r;
        r.cylinders = read_cylinder_group(rj, "cylinders", cylinders, rctx);
        r.length_m = require_positive(rj, "length_m", rctx);
        r.d_in_m = require_positive(rj, "d_in_m", rctx);
        r.d_out_m = require_positive(rj, "d_out_m", rctx);
        d.runners.push_back(r);
    }
    require_cylinder_coverage(d.runners, cylinders, rctx);
    if (!ij.contains("interface") || !ij.at("interface").is_string() || ij.at("interface").get<std::string>().empty()) {
        throw std::runtime_error(ictx + ": 'interface' is required (a non-empty string)");
    }
    d.interface = ij.at("interface").get<std::string>();
    return d;
}

ps::drivetrain::ExhaustCycleDesc read_exhaust_block(const json& ej, const std::string& ectx, int cylinders) {
    using namespace ps::drivetrain;
    if (!ej.is_object()) throw std::runtime_error(ectx + " must be an object");
    constexpr const char* kKeys[] = {"primaries", "merges", "interface", "source"};
    for (auto it = ej.begin(); it != ej.end(); ++it) {
        bool known = false;
        for (const char* k : kKeys) known = known || it.key() == k;
        if (!known) throw std::runtime_error(ectx + ": unknown key '" + it.key() + "'");
    }
    ExhaustCycleDesc d;
    d.source = ej.value("source", std::string());
    if (d.source.empty()) throw std::runtime_error(ectx + ": 'source' is required (PLAN.md section 8)");

    if (!ej.contains("primaries") || !ej.at("primaries").is_array() || ej.at("primaries").empty()) {
        throw std::runtime_error(ectx + ": 'primaries' must be a non-empty array");
    }
    const std::string pctx = ectx + ".primaries";
    for (const auto& pj : ej.at("primaries")) {
        if (!pj.is_object()) throw std::runtime_error(pctx + ": each entry must be an object");
        constexpr const char* kPrimaryKeys[] = {"cylinders", "length_m", "d_m"};
        for (auto it = pj.begin(); it != pj.end(); ++it) {
            bool known = false;
            for (const char* k : kPrimaryKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(pctx + ": unknown key '" + it.key() + "'");
        }
        PrimaryPipeDesc pr;
        pr.cylinders = read_cylinder_group(pj, "cylinders", cylinders, pctx);
        pr.length_m = require_positive(pj, "length_m", pctx);
        pr.d_m = require_positive(pj, "d_m", pctx);
        d.primaries.push_back(pr);
    }
    require_cylinder_coverage(d.primaries, cylinders, pctx);

    if (!ej.contains("merges") || !ej.at("merges").is_array() || ej.at("merges").empty()) {
        throw std::runtime_error(ectx + ": 'merges' must be a non-empty array (a pipe-bearing exhaust needs at least one "
                                  "junction reaching the flange)");
    }
    const std::string mctx = ectx + ".merges";
    std::vector<std::string> defined_names;   // 'into' names defined so far, in file order
    std::vector<int> consumed_count;          // parallel to defined_names: times referenced by a LATER merge
    std::vector<bool> cyl_used(static_cast<std::size_t>(cylinders) + 1, false);
    int cyl_used_count = 0;
    for (const auto& mj : ej.at("merges")) {
        if (!mj.is_object()) throw std::runtime_error(mctx + ": each entry must be an object");
        constexpr const char* kMergeKeys[] = {"from", "into", "length_m", "d_m"};
        for (auto it = mj.begin(); it != mj.end(); ++it) {
            bool known = false;
            for (const char* k : kMergeKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(mctx + ": unknown key '" + it.key() + "'");
        }
        MergeCycleDesc m;
        if (!mj.contains("from") || !mj.at("from").is_array() || mj.at("from").size() < 2) {
            throw std::runtime_error(mctx + ": 'from' must be an array of at least 2 sources");
        }
        for (const auto& fv : mj.at("from")) {
            MergeSourceRef ref;
            if (fv.is_number_integer()) {
                ref.is_cylinder = true;
                ref.cylinder = fv.get<int>();
                if (ref.cylinder < 1 || ref.cylinder > cylinders) {
                    throw std::runtime_error(mctx + ": 'from' cylinder " + std::to_string(ref.cylinder) + " is out of range [1, " +
                                              std::to_string(cylinders) + "]");
                }
                if (cyl_used[static_cast<std::size_t>(ref.cylinder)]) {
                    throw std::runtime_error(mctx + ": cylinder " + std::to_string(ref.cylinder) +
                                              " is referenced by more than one merge");
                }
                cyl_used[static_cast<std::size_t>(ref.cylinder)] = true;
                ++cyl_used_count;
            } else if (fv.is_string()) {
                ref.is_cylinder = false;
                ref.name = fv.get<std::string>();
                const auto it = std::find(defined_names.begin(), defined_names.end(), ref.name);
                if (it == defined_names.end()) {
                    throw std::runtime_error(mctx + ": 'from' name \"" + ref.name + "\" is not an earlier merge's 'into' "
                                              "(forward/unknown references are rejected)");
                }
                const std::size_t idx = static_cast<std::size_t>(it - defined_names.begin());
                if (consumed_count[idx] > 0) {
                    throw std::runtime_error(mctx + ": \"" + ref.name + "\" is referenced by more than one merge");
                }
                consumed_count[idx] += 1;
            } else {
                throw std::runtime_error(mctx + ": 'from' entries must be a cylinder number or a string naming an earlier merge");
            }
            m.from.push_back(ref);
        }
        if (!mj.contains("into") || !mj.at("into").is_string() || mj.at("into").get<std::string>().empty()) {
            throw std::runtime_error(mctx + ": 'into' is required (a non-empty string)");
        }
        m.into = mj.at("into").get<std::string>();
        if (std::find(defined_names.begin(), defined_names.end(), m.into) != defined_names.end()) {
            throw std::runtime_error(mctx + ": duplicate 'into' name \"" + m.into + "\"");
        }
        m.length_m = require_positive(mj, "length_m", mctx);
        m.d_m = require_positive(mj, "d_m", mctx);
        d.merges.push_back(m);
        defined_names.push_back(m.into);
        consumed_count.push_back(0);
    }
    if (cyl_used_count != cylinders) {
        throw std::runtime_error(mctx + ": must cover every cylinder exactly once across every merge's 'from' (covered " +
                                  std::to_string(cyl_used_count) + " of " + std::to_string(cylinders) + ")");
    }
    // Every 'into' except the LAST must be consumed by exactly one later
    // merge; the last is the terminal flange (consumed zero times).
    for (std::size_t i = 0; i + 1 < defined_names.size(); ++i) {
        if (consumed_count[i] != 1) {
            throw std::runtime_error(mctx + ": \"" + defined_names[i] + "\" must be consumed by exactly one later merge (a "
                                      "dangling branch never reaches the flange)");
        }
    }
    if (!defined_names.empty() && consumed_count.back() != 0) {
        throw std::runtime_error(mctx + ": the last merge's 'into' (\"" + defined_names.back() +
                                  "\") is the terminal flange and must not be consumed by another merge");
    }

    if (!ej.contains("interface") || !ej.at("interface").is_string() || ej.at("interface").get<std::string>().empty()) {
        throw std::runtime_error(ectx + ": 'interface' is required (a non-empty string)");
    }
    d.interface = ej.at("interface").get<std::string>();
    if (!defined_names.empty() && d.interface != defined_names.back()) {
        throw std::runtime_error(ectx + ": 'interface' (\"" + d.interface + "\") must equal the last merge's 'into' (\"" +
                                  defined_names.back() + "\")");
    }
    return d;
}

// CFL floor for the "resolution" cell-size block (out/audio_plan.md section
// 3.1's own rule: "the loader checks the CFL ... If dx is too small it
// rejects with the minimum dx"). Worst-case wave speeds are the plan's own
// [A] figures (exhaust |u|+a <= 900 m/s, intake <= 500 m/s). The step h used
// for the check is the LARGEST (worst-case, most restrictive on dx) h this
// resolution could ever be stepped at, so a resolution that passes here is
// CFL-safe at every quality/sample-rate the plan lists:
//  - power_dx_m: checked against h_preview = 1/(960*24) s (CLAUDE.md's
//    "Engine cycle" section) - the generator's coarser MapQuality::Preview
//    step, larger than h_full = 1/(960*48) s.
//  - audio_dx_exhaust_m/audio_dx_intake_m: checked against 1/44100 s (D-7's
//    44.1 kHz live rate), larger than 1/48000 s.
// Both constants are duplicated here rather than including
// core/src/engine_cycle's internal cycle_point.h from this public IO loader
// (A2/A3 are actively extending that internal directory; this loader must
// not create a build-order dependency on it).
ps::drivetrain::ResolutionCycleDesc read_resolution_block(const json& cj, const std::string& cctx) {
    using namespace ps::drivetrain;
    ResolutionCycleDesc r; // defaults: 0.05 / 0.032 / 0.017 m
    const std::string kctx = cctx + ".resolution";
    if (cj.contains("resolution")) {
        const auto& rj = cj.at("resolution");
        if (!rj.is_object()) throw std::runtime_error(kctx + " must be an object");
        constexpr const char* kKeys[] = {"power_dx_m", "audio_dx_exhaust_m", "audio_dx_intake_m"};
        for (auto it = rj.begin(); it != rj.end(); ++it) {
            bool known = false;
            for (const char* k : kKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(kctx + ": unknown key '" + it.key() + "'");
        }
        if (rj.contains("power_dx_m")) r.power_dx_m = require_positive(rj, "power_dx_m", kctx);
        if (rj.contains("audio_dx_exhaust_m")) r.audio_dx_exhaust_m = require_positive(rj, "audio_dx_exhaust_m", kctx);
        if (rj.contains("audio_dx_intake_m")) r.audio_dx_intake_m = require_positive(rj, "audio_dx_intake_m", kctx);
    }
    constexpr real kHPreview = real(1.0) / (real(960.0) * real(24.0));
    constexpr real kHAudioWorst = real(1.0) / real(44100.0);
    constexpr real kExhaustWaveSpeed = real(900.0);
    constexpr real kIntakeWaveSpeed = real(500.0);
    const real min_power_dx = kExhaustWaveSpeed * kHPreview;
    const real min_audio_dx_exhaust = kExhaustWaveSpeed * kHAudioWorst;
    const real min_audio_dx_intake = kIntakeWaveSpeed * kHAudioWorst;
    if (!(r.power_dx_m >= min_power_dx)) {
        throw std::runtime_error(kctx + ": 'power_dx_m' " + std::to_string(r.power_dx_m) + " is below the CFL minimum " +
                                  std::to_string(min_power_dx) + " m (900 m/s worst-case wave speed at h=1/(960*24) s)");
    }
    if (!(r.audio_dx_exhaust_m >= min_audio_dx_exhaust)) {
        throw std::runtime_error(kctx + ": 'audio_dx_exhaust_m' " + std::to_string(r.audio_dx_exhaust_m) +
                                  " is below the CFL minimum " + std::to_string(min_audio_dx_exhaust) +
                                  " m (900 m/s worst-case wave speed at h=1/44100 s)");
    }
    if (!(r.audio_dx_intake_m >= min_audio_dx_intake)) {
        throw std::runtime_error(kctx + ": 'audio_dx_intake_m' " + std::to_string(r.audio_dx_intake_m) +
                                  " is below the CFL minimum " + std::to_string(min_audio_dx_intake) +
                                  " m (500 m/s worst-case wave speed at h=1/44100 s)");
    }
    return r;
}

ps::drivetrain::ExhaustElementDesc read_exhaust_element(const json& ej, const std::string& ctx);

std::vector<ps::drivetrain::ExhaustElementDesc> read_exhaust_elements(const json& arr, const std::string& ctx) {
    if (!arr.is_array() || arr.empty()) throw std::runtime_error(ctx + " must be a non-empty array of elements");
    std::vector<ps::drivetrain::ExhaustElementDesc> out;
    for (const auto& ej : arr) out.push_back(read_exhaust_element(ej, ctx));
    for (std::size_t i = 0; i < out.size(); ++i) {
        const bool terminal = out[i].type == ps::drivetrain::ExhaustElementType::Outlet ||
                               out[i].type == ps::drivetrain::ExhaustElementType::Branch;
        const bool last = (i + 1 == out.size());
        if (terminal && !last) throw std::runtime_error(ctx + ": an 'outlet'/'branch' element must be the last element of its chain");
        if (!terminal && last) throw std::runtime_error(ctx + ": an exhaust chain must end in an 'outlet' or 'branch' element");
    }
    return out;
}

ps::drivetrain::ExhaustElementDesc read_exhaust_element(const json& ej, const std::string& ctx) {
    using namespace ps::drivetrain;
    if (!ej.is_object()) throw std::runtime_error(ctx + ": each exhaust element must be an object");
    if (!ej.contains("type") || !ej.at("type").is_string()) throw std::runtime_error(ctx + ": exhaust element 'type' is required");
    const std::string type = ej.at("type").get<std::string>();
    ExhaustElementDesc d;
    if (type == "catalyst") {
        d.type = ExhaustElementType::Catalyst;
        constexpr const char* kKeys[] = {"type", "length_m", "d_m", "k_loss"};
        for (auto it = ej.begin(); it != ej.end(); ++it) {
            bool known = false;
            for (const char* k : kKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(ctx + ": catalyst element: unknown key '" + it.key() + "'");
        }
        d.length_m = require_positive(ej, "length_m", ctx + ".catalyst");
        d.d_m = require_positive(ej, "d_m", ctx + ".catalyst");
        d.k_loss = require_positive(ej, "k_loss", ctx + ".catalyst");
    } else if (type == "pipe" || type == "expansion_chamber") {
        d.type = type == "pipe" ? ExhaustElementType::Pipe : ExhaustElementType::ExpansionChamber;
        constexpr const char* kKeys[] = {"type", "length_m", "d_m"};
        for (auto it = ej.begin(); it != ej.end(); ++it) {
            bool known = false;
            for (const char* k : kKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(ctx + ": " + type + " element: unknown key '" + it.key() + "'");
        }
        d.length_m = require_positive(ej, "length_m", ctx + "." + type);
        d.d_m = require_positive(ej, "d_m", ctx + "." + type);
    } else if (type == "outlet") {
        d.type = ExhaustElementType::Outlet;
        constexpr const char* kKeys[] = {"type", "position_m"};
        for (auto it = ej.begin(); it != ej.end(); ++it) {
            bool known = false;
            for (const char* k : kKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(ctx + ": outlet element: unknown key '" + it.key() + "'");
        }
        if (!ej.contains("position_m")) throw std::runtime_error(ctx + ": outlet element: 'position_m' is required");
        d.position_m = read_vec3(ej.at("position_m"), Vec3::zero());
    } else if (type == "branch") {
        d.type = ExhaustElementType::Branch;
        constexpr const char* kKeys[] = {"type", "branches"};
        for (auto it = ej.begin(); it != ej.end(); ++it) {
            bool known = false;
            for (const char* k : kKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(ctx + ": branch element: unknown key '" + it.key() + "'");
        }
        if (!ej.contains("branches") || !ej.at("branches").is_array() || ej.at("branches").size() < 2) {
            throw std::runtime_error(ctx + ": branch element: 'branches' must be an array of at least 2 element chains");
        }
        for (const auto& bj : ej.at("branches")) d.branches.push_back(read_exhaust_elements(bj, ctx + ".branch"));
    } else {
        throw std::runtime_error(ctx + ": exhaust element 'type' must be \"catalyst\", \"pipe\", \"expansion_chamber\", "
                                  "\"outlet\" or \"branch\", got \"" + type + "\"");
    }
    return d;
}

ps::drivetrain::InstallationDesc read_installation_object(const json& j, const std::string& path) {
    using namespace ps::drivetrain;
    require_format(j, "installation", 1, path);
    const std::string ctx = path;
    constexpr const char* kKeys[] = {"format", "intake", "exhaust", "source"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        bool known = false;
        for (const char* k : kKeys) known = known || it.key() == k;
        if (!known) throw std::runtime_error(ctx + ": unknown key '" + it.key() + "'");
    }
    InstallationDesc d;
    d.source = j.value("source", std::string());
    if (d.source.empty()) throw std::runtime_error(ctx + ": 'source' is required (PLAN.md section 8)");

    if (!j.contains("intake") || !j.at("intake").is_object()) throw std::runtime_error(ctx + ": 'intake' object is required");
    {
        const auto& ij = j.at("intake");
        const std::string ictx = ctx + ".intake";
        constexpr const char* kIntakeKeys[] = {"airbox_m3", "snorkel", "inlet_position_m"};
        for (auto it = ij.begin(); it != ij.end(); ++it) {
            bool known = false;
            for (const char* k : kIntakeKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(ictx + ": unknown key '" + it.key() + "'");
        }
        d.intake.airbox_m3 = require_positive(ij, "airbox_m3", ictx);
        if (ij.contains("snorkel")) {
            const auto& sj = ij.at("snorkel");
            const std::string sctx = ictx + ".snorkel";
            if (!sj.is_object()) throw std::runtime_error(sctx + " must be an object");
            constexpr const char* kSnorkelKeys[] = {"length_m", "d_m"};
            for (auto it = sj.begin(); it != sj.end(); ++it) {
                bool known = false;
                for (const char* k : kSnorkelKeys) known = known || it.key() == k;
                if (!known) throw std::runtime_error(sctx + ": unknown key '" + it.key() + "'");
            }
            d.intake.has_snorkel = true;
            d.intake.snorkel.length_m = require_positive(sj, "length_m", sctx);
            d.intake.snorkel.d_m = require_positive(sj, "d_m", sctx);
        }
        if (!ij.contains("inlet_position_m")) throw std::runtime_error(ictx + ": 'inlet_position_m' is required");
        d.intake.inlet_position_m = read_vec3(ij.at("inlet_position_m"), Vec3::zero());
    }

    if (!j.contains("exhaust") || !j.at("exhaust").is_object()) throw std::runtime_error(ctx + ": 'exhaust' object is required");
    {
        const auto& ej = j.at("exhaust");
        const std::string ectx = ctx + ".exhaust";
        constexpr const char* kExhaustKeys[] = {"elements"};
        for (auto it = ej.begin(); it != ej.end(); ++it) {
            bool known = false;
            for (const char* k : kExhaustKeys) known = known || it.key() == k;
            if (!known) throw std::runtime_error(ectx + ": unknown key '" + it.key() + "'");
        }
        if (!ej.contains("elements")) throw std::runtime_error(ectx + ": 'elements' is required");
        d.exhaust.elements = read_exhaust_elements(ej.at("elements"), ectx + ".elements");
    }
    return d;
}

ps::drivetrain::CycleModelDesc read_cycle_block(const json& cj, const std::string& cctx, const std::string& engine_dir,
                                                const ps::drivetrain::PistonGeometryDesc& piston,
                                                bool friction_map_given) {
    using namespace ps::drivetrain;
    constexpr real kMm = real(1e-3);
    if (!cj.is_object()) throw std::runtime_error(cctx + " must be an object");
    for (const char* k : {"bore_mm", "stroke_mm", "cylinders"}) {
        if (cj.contains(k)) {
            throw std::runtime_error(cctx + ": '" + std::string(k) + "' is not allowed here - the cycle model takes the "
                                      "engine's own bore_mm/stroke_mm/cylinders");
        }
    }
    if (cj.contains("back_pressure_kpa_vs_exh_kg_s")) {
        throw std::runtime_error(cctx + ": 'back_pressure_kpa_vs_exh_kg_s' is not supported yet (the simulated engine's "
                                  "runtime has no exhaust back pressure until I7d)");
    }
    for (auto it = cj.begin(); it != cj.end(); ++it) {
        const std::string& k = it.key();
        bool known = false;
        for (const char* ok : kCycleKeys) known = known || (k == ok);
        const bool source_note = k.size() > 7 && k.compare(k.size() - 7, 7, "_source") == 0;
        if (!known && !source_note) throw std::runtime_error(cctx + ": unknown key '" + k + "'");
    }

    CycleModelDesc d;
    const std::string source = cj.value("source", std::string());
    if (source.empty()) throw std::runtime_error(cctx + ": 'source' is required (PLAN.md section 8)");
    d.source = source;

    d.bore_m = piston.bore_m;
    d.stroke_m = piston.stroke_m;
    d.cylinders = piston.cylinders;
    d.cycle = 4;
    d.rod_m = require_positive(cj, "rod_mm", cctx) * kMm;
    d.pin_offset_m = cj.contains("pin_offset_mm") ? require_number(cj, "pin_offset_mm", cctx) * kMm : real(0.0);
    if (!(d.rod_m > real(0.5) * d.stroke_m + std::abs(d.pin_offset_m))) {
        throw std::runtime_error(cctx + ": 'rod_mm' must exceed stroke_mm/2 + |pin_offset_mm|");
    }
    d.compression_ratio = require_number(cj, "compression_ratio", cctx);
    if (!(d.compression_ratio > real(1.0))) {
        throw std::runtime_error(cctx + ": 'compression_ratio' must be > 1, got " + std::to_string(d.compression_ratio));
    }
    d.intake_valves = require_positive_int(cj, "intake_valves", cctx);
    d.exhaust_valves = require_positive_int(cj, "exhaust_valves", cctx);
    d.intake_valve_d_m = require_positive(cj, "intake_valve_d_mm", cctx) * kMm;
    d.exhaust_valve_d_m = require_positive(cj, "exhaust_valve_d_mm", cctx) * kMm;
    d.intake_lift = read_cycle_lift(cj, "intake_lift", cctx);
    d.exhaust_lift = read_cycle_lift(cj, "exhaust_lift", cctx);
    // ps_audio A4: 'intake'/'exhaust' (the finite-volume pipe pair) must be
    // given together (a manifold with no exhaust plumbing modeled, or vice
    // versa, is not a supported configuration) or not at all. When absent,
    // the pre-A4 lumped runner_length_mm/runner_diameter_mm pair is
    // required as before (bit-identical to every pre-A4 engine file); when
    // present, that lumped pair is redundant with intake.runners and is
    // rejected instead.
    const bool has_intake_block = cj.contains("intake");
    const bool has_exhaust_block = cj.contains("exhaust");
    if (has_intake_block != has_exhaust_block) {
        throw std::runtime_error(cctx + ": 'intake' and 'exhaust' must be given together (a pipe-modeled engine needs "
                                  "both, a lumped one needs neither)");
    }
    const bool has_pipes = has_intake_block && has_exhaust_block;
    if (has_pipes) {
        if (cj.contains("runner_length_mm") || cj.contains("runner_diameter_mm")) {
            throw std::runtime_error(cctx + ": 'runner_length_mm'/'runner_diameter_mm' are not allowed together with "
                                      "'intake'/'exhaust' (the pipe blocks replace the lumped runner)");
        }
        d.runner_length_m = real(0.0);
        d.runner_diameter_m = real(0.0);
    } else {
        d.runner_length_m = require_positive(cj, "runner_length_mm", cctx) * kMm;
        d.runner_diameter_m = require_positive(cj, "runner_diameter_mm", cctx) * kMm;
    }
    d.intake_port_m3 = require_positive(cj, "intake_port_cc", cctx) * real(1e-6);
    d.exhaust_port_m3 = require_positive(cj, "exhaust_port_cc", cctx) * real(1e-6);

    if (!cj.contains("cd_vs_l_over_d")) throw std::runtime_error(cctx + ": 'cd_vs_l_over_d' is required");
    d.cd_vs_l_over_d = read_curve(cj.at("cd_vs_l_over_d"));
    if (d.cd_vs_l_over_d.x.size() < 2) throw std::runtime_error(cctx + ".cd_vs_l_over_d: needs at least 2 points");
    require_ascending(d.cd_vs_l_over_d, cctx + ".cd_vs_l_over_d");
    if (d.cd_vs_l_over_d.x.front() < real(0.0)) throw std::runtime_error(cctx + ".cd_vs_l_over_d: x (L/D) must be >= 0");
    for (real y : d.cd_vs_l_over_d.y) {
        if (!(y > real(0.0) && y <= real(1.0))) throw std::runtime_error(cctx + ".cd_vs_l_over_d: every Cd must be in (0, 1]");
    }

    if (cj.contains("cam")) {
        const auto& camj = cj.at("cam");
        const std::string kctx = cctx + ".cam";
        if (!camj.is_object()) throw std::runtime_error(kctx + " must be an object");
        for (auto it = camj.begin(); it != camj.end(); ++it) {
            if (it.key() != "intake_advance_deg" && it.key() != "exhaust_retard_deg") {
                throw std::runtime_error(kctx + ": unknown key '" + it.key() + "'");
            }
        }
        if (camj.contains("intake_advance_deg")) {
            d.cam.intake_advance_deg = read_engine_table(camj.at("intake_advance_deg"), kctx + ".intake_advance_deg", "p_im_kpa",
                                                         real(1000.0), real(1.0));
            require_positive_y_axis(d.cam.intake_advance_deg, kctx + ".intake_advance_deg", "p_im_kpa");
        }
        if (camj.contains("exhaust_retard_deg")) {
            d.cam.exhaust_retard_deg = read_engine_table(camj.at("exhaust_retard_deg"), kctx + ".exhaust_retard_deg", "p_im_kpa",
                                                         real(1000.0), real(1.0));
            require_positive_y_axis(d.cam.exhaust_retard_deg, kctx + ".exhaust_retard_deg", "p_im_kpa");
        }
    }

    // C17a follow-up: Tier 1b spark schedule (same axes, units and value
    // range as ecu.spark_advance_deg) and Tier 2 ignition delay.
    if (cj.contains("spark_advance_deg")) {
        const std::string sctx = cctx + ".spark_advance_deg";
        d.spark_advance_deg = read_engine_table(cj.at("spark_advance_deg"), sctx, "p_im_kpa", real(1000.0), real(1.0));
        require_positive_y_axis(d.spark_advance_deg, sctx, "p_im_kpa");
        require_table_values(d.spark_advance_deg, sctx, real(-30.0), true, real(70.0), true);
    }
    if (cj.contains("ignition_delay_deg")) {
        d.ignition_delay_deg = require_number(cj, "ignition_delay_deg", cctx);
        if (!(d.ignition_delay_deg >= real(0.0) && d.ignition_delay_deg <= real(60.0))) {
            throw std::runtime_error(cctx + ": 'ignition_delay_deg' must be in [0, 60], got " +
                                     std::to_string(d.ignition_delay_deg));
        }
    }

    // Tier 2: literature defaults (engine_cycle_desc.h) unless given.
    auto opt_positive = [&](const char* key, real& field) {
        if (cj.contains(key)) field = require_positive(cj, key, cctx);
    };
    opt_positive("wiebe_a", d.wiebe_a);
    if (cj.contains("wiebe_m")) {
        d.wiebe_m = require_number(cj, "wiebe_m", cctx);
        if (!(d.wiebe_m >= real(0.0))) throw std::runtime_error(cctx + ": 'wiebe_m' must be >= 0, got " + std::to_string(d.wiebe_m));
    }
    opt_positive("burn_duration_deg", d.burn_duration_deg);
    if (cj.contains("ca50_mbt_deg_atdc")) d.ca50_mbt_deg_atdc = require_number(cj, "ca50_mbt_deg_atdc", cctx);
    opt_positive("woschni_c1_gx", d.woschni_c1_gx);
    opt_positive("woschni_c1_cc", d.woschni_c1_cc);
    opt_positive("woschni_c2", d.woschni_c2);
    opt_positive("wall_temperature_k", d.wall_temperature_k);
    opt_positive("head_area_factor", d.head_area_factor);
    opt_positive("exhaust_backflow_temperature_k", d.exhaust_backflow_temperature_k);
    if (cj.contains("combustion_eff_lean")) {
        d.combustion_eff_lean = require_number(cj, "combustion_eff_lean", cctx);
        if (!(d.combustion_eff_lean > real(0.0) && d.combustion_eff_lean <= real(1.0))) {
            throw std::runtime_error(cctx + ": 'combustion_eff_lean' must be in (0, 1], got " + std::to_string(d.combustion_eff_lean));
        }
    }

    // D13 friction correlation: FMEP = a + b*s + c*s^2 [kPa], s = rpm/1000
    // (polynomial_rpm) or the mean piston speed [m/s] (polynomial_piston_speed).
    if (cj.contains("friction")) {
        const auto& fj = cj.at("friction");
        const std::string fctx = cctx + ".friction";
        if (!fj.is_object()) throw std::runtime_error(fctx + " must be an object");
        const std::string model = fj.value("model", std::string());
        if (model == "polynomial_rpm") d.friction.model = FrictionModelDesc::Model::PolynomialRpm;
        else if (model == "polynomial_piston_speed") d.friction.model = FrictionModelDesc::Model::PolynomialPistonSpeed;
        else throw std::runtime_error(fctx + ": 'model' must be \"polynomial_rpm\" or \"polynomial_piston_speed\", got \"" + model + "\"");
        d.friction.a_pa = require_number(fj, "a_kpa", fctx) * real(1000.0);
        d.friction.b_pa = require_number(fj, "b_kpa", fctx) * real(1000.0);
        d.friction.c_pa = require_number(fj, "c_kpa", fctx) * real(1000.0);
        if (!(d.friction.a_pa >= real(0.0))) throw std::runtime_error(fctx + ": 'a_kpa' must be >= 0 (the Coulomb part)");
    } else if (!friction_map_given) {
        throw std::runtime_error(cctx + ": 'friction' is required unless maps.friction_torque_nm is given");
    }

    // ---- ps_audio A1 (out/audio_plan.md section 1/3): firing phasing +
    // bank geometry. Descriptor only - see CycleModelDesc's own field
    // comments for why nothing downstream reads these yet.
    if (cj.contains("firing_order")) {
        if (!cj.at("firing_order").is_array()) throw std::runtime_error(cctx + ": 'firing_order' must be an array of integers");
        for (const auto& v : cj.at("firing_order")) {
            if (!v.is_number_integer()) throw std::runtime_error(cctx + ": 'firing_order' entries must be integers");
            d.firing_order.push_back(v.get<int>());
        }
    } else if (d.cylinders == 4) {
        d.firing_order = {1, 3, 4, 2}; // the common inline-4 order (literature, plan section 3)
    } else {
        throw std::runtime_error(cctx + ": 'firing_order' is required for " + std::to_string(d.cylinders) +
                                  " cylinders (no default exists - a cross-plane and a flat-plane V8 differ, for "
                                  "example)");
    }
    try {
        ps::engine_cycle::validate_firing_order(d.firing_order, d.cylinders);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(cctx + ": " + e.what());
    }

    if (cj.contains("firing_angles_deg")) {
        if (!cj.at("firing_angles_deg").is_array()) {
            throw std::runtime_error(cctx + ": 'firing_angles_deg' must be an array of numbers");
        }
        for (const auto& v : cj.at("firing_angles_deg")) {
            if (!v.is_number()) throw std::runtime_error(cctx + ": 'firing_angles_deg' entries must be numbers");
            d.firing_angles_deg.push_back(v.get<real>());
        }
    }
    try {
        ps::engine_cycle::validate_firing_angles(d.firing_angles_deg, d.cylinders);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error(cctx + ": " + e.what());
    }

    if (cj.contains("banks")) {
        if (!cj.at("banks").is_array() || cj.at("banks").empty()) {
            throw std::runtime_error(cctx + ": 'banks' must be a non-empty array of cylinder-number arrays");
        }
        for (const auto& bj : cj.at("banks")) {
            if (!bj.is_array() || bj.empty()) {
                throw std::runtime_error(cctx + ": each 'banks' entry must be a non-empty array of cylinder numbers");
            }
            std::vector<int> bank;
            for (const auto& v : bj) {
                if (!v.is_number_integer()) throw std::runtime_error(cctx + ": 'banks' entries must be integers");
                bank.push_back(v.get<int>());
            }
            d.banks.push_back(std::move(bank));
        }
    } else {
        d.banks.push_back({}); // default: one bank, every cylinder (plan D5)
        for (int c = 1; c <= d.cylinders; ++c) d.banks.back().push_back(c);
    }
    {
        std::vector<bool> seen(static_cast<std::size_t>(d.cylinders) + 1, false);
        int covered = 0;
        for (const auto& bank : d.banks) {
            for (int c : bank) {
                if (c < 1 || c > d.cylinders) {
                    throw std::runtime_error(cctx + ": 'banks' cylinder number " + std::to_string(c) + " is out of range [1, " +
                                              std::to_string(d.cylinders) + "]");
                }
                if (seen[static_cast<std::size_t>(c)]) {
                    throw std::runtime_error(cctx + ": 'banks' cylinder " + std::to_string(c) + " is assigned to more than one bank");
                }
                seen[static_cast<std::size_t>(c)] = true;
                ++covered;
            }
        }
        if (covered != d.cylinders) {
            throw std::runtime_error(cctx + ": 'banks' must cover every cylinder exactly once (covered " + std::to_string(covered) +
                                      " of " + std::to_string(d.cylinders) + ")");
        }
    }

    d.bank_angle_deg = cj.contains("bank_angle_deg") ? require_number(cj, "bank_angle_deg", cctx) : real(0.0);

    // ps_audio A4: the pipe pair, the engine's own default installation and
    // the pipe cell-size resolution block. All three are only meaningful
    // (and only allowed) when has_pipes - see the pair-check above.
    if (has_pipes) {
        d.intake = read_intake_block(cj.at("intake"), cctx + ".intake", d.cylinders);
        d.exhaust = read_exhaust_block(cj.at("exhaust"), cctx + ".exhaust", d.cylinders);
        if (!cj.contains("default_installation") || !cj.at("default_installation").is_string() ||
            cj.at("default_installation").get<std::string>().empty()) {
            throw std::runtime_error(cctx + ": 'default_installation' is required (a non-empty path string) when "
                                      "'intake'/'exhaust' pipe blocks are present");
        }
        d.default_installation_path = resolve_path(engine_dir, cj.at("default_installation").get<std::string>());
        d.resolution = read_resolution_block(cj, cctx);
    } else {
        if (cj.contains("default_installation")) {
            throw std::runtime_error(cctx + ": 'default_installation' is only allowed together with 'intake'/'exhaust' "
                                      "pipe blocks");
        }
        if (cj.contains("resolution")) {
            throw std::runtime_error(cctx + ": 'resolution' is only allowed together with 'intake'/'exhaust' pipe blocks");
        }
    }

    d.grid_rpm = read_cycle_grid(cj, "grid_rpm", real(1.0), cctx);
    d.grid_p_im = read_cycle_grid(cj, "grid_p_im_kpa", real(1000.0), cctx);
    return d;
}

// Efficiency sanity bound: the largest indicated fuel-conversion efficiency
// any combination of the maps can reach must stay below 1.
void require_maps_efficiency_below_one(const ps::drivetrain::SimulatedEngineMapsDesc& m, const std::string& mctx) {
    real eta_max = real(0.0), fl_max = real(0.0), es_max = real(0.0);
    for (real v : m.eta_i.values) eta_max = std::max(eta_max, v);
    for (real v : m.f_lambda.y) fl_max = std::max(fl_max, v);
    for (real v : m.eta_spark.y) es_max = std::max(es_max, v);
    if (!(eta_max * fl_max * es_max < real(1.0))) {
        throw std::runtime_error(mctx + ": max(eta_i)*max(f_lambda)*max(eta_spark) = " + std::to_string(eta_max * fl_max * es_max) +
                                  " implies >= 100% indicated efficiency");
    }
}

ps::drivetrain::SimulatedEngineDesc read_simulated_fields(const json& j, const std::string& ctx, const std::string& engine_dir,
                                                          std::vector<std::string>* resolved_fuel_paths_out = nullptr,
                                                          const EngineMapOptions* map_options = nullptr,
                                                          const std::string* installation_override_path = nullptr) {
    using namespace ps::drivetrain;
    constexpr real kMm = real(1e-3);
    constexpr real kKpa = real(1000.0);
    constexpr real kGram = real(1e-3);

    // Graph-engine-only keys (I7 plan section 2.1).
    if (j.contains("wot_torque_nm_vs_rpm") || j.contains("motoring_torque_nm_vs_rpm")) {
        throw std::runtime_error(ctx + ": wot_torque_nm_vs_rpm/motoring_torque_nm_vs_rpm are not allowed on a "
                                  "\"simulated\" engine - simulated engines derive those curves from their air path and maps");
    }
    if (j.contains("governor")) {
        throw std::runtime_error(ctx + ": 'governor' is \"torque_map\"-only in I7 (a governed simulated engine is a later "
                                  "increment)");
    }
    if (j.contains("fuel_map")) {
        throw std::runtime_error(ctx + ": 'fuel_map' is not allowed on a \"simulated\" engine - its fuel flow follows from "
                                  "the air path and the lambda target");
    }
    if (j.contains("indicated_efficiency")) {
        throw std::runtime_error(ctx + ": 'indicated_efficiency' is not allowed on a \"simulated\" engine - use maps.eta_i");
    }

    SimulatedEngineDesc e;

    // --- architecture + geometry -------------------------------------------
    if (!j.contains("architecture") || !j.at("architecture").is_string()) {
        throw std::runtime_error(ctx + ": 'architecture' is required (\"piston_4stroke\" or \"rotary\")");
    }
    const std::string arch = j.at("architecture").get<std::string>();
    constexpr const char* kPistonKeys[] = {"bore_mm", "stroke_mm", "cylinders"};
    constexpr const char* kRotaryKeys[] = {"rotor_radius_mm", "eccentricity_mm", "rotor_width_mm", "rotors"};
    if (arch == "piston_4stroke") {
        e.architecture = EngineArchitecture::Piston4Stroke;
        for (const char* k : kRotaryKeys) {
            if (j.contains(k)) throw std::runtime_error(ctx + ": '" + std::string(k) + "' is a rotary field, not allowed on a piston_4stroke engine");
        }
        e.piston.bore_m = require_positive(j, "bore_mm", ctx) * kMm;
        e.piston.stroke_m = require_positive(j, "stroke_mm", ctx) * kMm;
        e.piston.cylinders = require_positive_int(j, "cylinders", ctx);
    } else if (arch == "rotary") {
        e.architecture = EngineArchitecture::Rotary;
        for (const char* k : kPistonKeys) {
            if (j.contains(k)) throw std::runtime_error(ctx + ": '" + std::string(k) + "' is a piston field, not allowed on a rotary engine");
        }
        e.rotary.rotor_radius_m = require_positive(j, "rotor_radius_mm", ctx) * kMm;
        e.rotary.eccentricity_m = require_positive(j, "eccentricity_mm", ctx) * kMm;
        e.rotary.rotor_width_m = require_positive(j, "rotor_width_mm", ctx) * kMm;
        e.rotary.rotors = require_positive_int(j, "rotors", ctx);
        if (!(e.rotary.eccentricity_m < e.rotary.rotor_radius_m)) {
            throw std::runtime_error(ctx + ": eccentricity_mm must be < rotor_radius_mm");
        }
    } else {
        throw std::runtime_error(ctx + ": 'architecture' must be \"piston_4stroke\" or \"rotary\", got \"" + arch + "\"");
    }

    e.intake_manifold_volume_m3 = require_positive(j, "intake_manifold_volume_l", ctx) * real(1e-3);

    // --- throttle body -------------------------------------------------------
    if (!j.contains("throttle_body") || !j.at("throttle_body").is_object()) {
        throw std::runtime_error(ctx + ": 'throttle_body' object is required");
    }
    {
        const auto& tb = j.at("throttle_body");
        const std::string tctx = ctx + ".throttle_body";
        e.throttle_body.diameter_m = require_positive(tb, "diameter_mm", tctx) * kMm;
        e.throttle_body.count = tb.contains("count") ? require_positive_int(tb, "count", tctx) : 1;
        e.throttle_body.discharge_coefficient = require_positive(tb, "discharge_coefficient", tctx);
        if (e.throttle_body.discharge_coefficient > real(1.0)) {
            throw std::runtime_error(tctx + ": 'discharge_coefficient' must be in (0, 1], got " +
                                      std::to_string(e.throttle_body.discharge_coefficient));
        }
        if (!tb.contains("area_fraction_vs_position")) throw std::runtime_error(tctx + ": 'area_fraction_vs_position' is required");
        e.throttle_body.area_fraction_vs_position = read_curve(tb.at("area_fraction_vs_position"));
        const std::string cctx = tctx + ".area_fraction_vs_position";
        require_ascending(e.throttle_body.area_fraction_vs_position, cctx);
        require_unit_domain_covered_nondecreasing(e.throttle_body.area_fraction_vs_position, cctx);
        if (!(e.throttle_body.area_fraction_vs_position.y.front() > real(0.0))) {
            throw std::runtime_error(cctx + ": y at position 0 must be > 0 (a closed plate still leaks; the manifold fixed "
                                      "point needs a nonzero area)");
        }
    }

    // --- throttle input (required, plan section 3.4.3) and its input-type-
    // specific fields (plan section 3.4.5) -----------------------------------
    if (!j.contains("throttle_input") || !j.at("throttle_input").is_string()) {
        throw std::runtime_error(ctx + ": 'throttle_input' is required for a simulated engine (\"electronic\" or \"mechanical\")");
    }
    const std::string throttle_input = j.at("throttle_input").get<std::string>();
    if (throttle_input == "electronic") e.throttle_input = ThrottleInput::Electronic;
    else if (throttle_input == "mechanical") e.throttle_input = ThrottleInput::Mechanical;
    else throw std::runtime_error(ctx + ": throttle_input must be \"electronic\" or \"mechanical\", got \"" + throttle_input + "\"");

    const bool has_bypass = j.contains("idle_control") && j.at("idle_control").is_object() &&
                            j.at("idle_control").contains("bypass_area_mm2");
    if (e.throttle_input == ThrottleInput::Mechanical) {
        if (j.contains("throttle_actuator_tau_s")) {
            throw std::runtime_error(ctx + ": 'throttle_actuator_tau_s' is not allowed on a mechanical throttle (a cable has "
                                      "no actuator)");
        }
        if (!has_bypass) {
            throw std::runtime_error(ctx + ": a mechanical throttle requires idle_control.bypass_area_mm2 (idle air comes "
                                      "from the idle bypass valve)");
        }
        e.idle_bypass_area_m2 = require_positive(j.at("idle_control"), "bypass_area_mm2", ctx + ".idle_control") * real(1e-6);
    } else {
        if (has_bypass) {
            throw std::runtime_error(ctx + ": idle_control.bypass_area_mm2 is not allowed on an electronic throttle (idle "
                                      "air comes from the plate)");
        }
        e.throttle_actuator_tau_s = require_number(j, "throttle_actuator_tau_s", ctx);
        if (e.throttle_actuator_tau_s < real(0.0)) {
            throw std::runtime_error(ctx + ": 'throttle_actuator_tau_s' must be >= 0, got " + std::to_string(e.throttle_actuator_tau_s));
        }
    }

    // --- ECU targets -------------------------------------------------------------
    if (!j.contains("ecu") || !j.at("ecu").is_object()) throw std::runtime_error(ctx + ": 'ecu' object is required");
    {
        const auto& ej = j.at("ecu");
        const std::string ectx = ctx + ".ecu";
        if (!ej.contains("lambda_target")) throw std::runtime_error(ectx + ": 'lambda_target' is required");
        e.ecu.lambda_target = read_engine_table(ej.at("lambda_target"), ectx + ".lambda_target", "p_im_kpa", kKpa, real(1.0));
        require_positive_y_axis(e.ecu.lambda_target, ectx + ".lambda_target", "p_im_kpa");
        require_table_values(e.ecu.lambda_target, ectx + ".lambda_target", real(0.5), true, real(2.0), true);
        // C17a follow-up: a cycle block's spark_advance_deg is the one spark
        // schedule (the generator runs its nodes at it); the ecu table is then
        // filled from it after read_cycle_block below and must not be given.
        const bool cycle_spark = j.contains("cycle") && j.at("cycle").is_object() &&
                                 j.at("cycle").contains("spark_advance_deg");
        if (cycle_spark) {
            if (ej.contains("spark_advance_deg")) {
                throw std::runtime_error(ectx + ": 'spark_advance_deg' must be absent when cycle.spark_advance_deg is "
                                                "given (the ecu schedule is taken from the cycle block)");
            }
        } else {
            if (!ej.contains("spark_advance_deg")) throw std::runtime_error(ectx + ": 'spark_advance_deg' is required");
            e.ecu.spark_advance_deg =
                read_engine_table(ej.at("spark_advance_deg"), ectx + ".spark_advance_deg", "p_im_kpa", kKpa, real(1.0));
            require_positive_y_axis(e.ecu.spark_advance_deg, ectx + ".spark_advance_deg", "p_im_kpa");
            require_table_values(e.ecu.spark_advance_deg, ectx + ".spark_advance_deg", real(-30.0), true, real(70.0), true);
        }
    }

    // --- runtime maps ------------------------------------------------------------
    // I7c C16a: with a "cycle" block the maps block is optional and partial -
    // every map it gives overrides the generated one (docs/data_formats/engine.md).
    const bool has_cycle = j.contains("cycle");
    if (!has_cycle && (!j.contains("maps") || !j.at("maps").is_object())) {
        throw std::runtime_error(ctx + ": 'maps' object is required");
    }
    if (j.contains("maps") && !j.at("maps").is_object()) throw std::runtime_error(ctx + ": 'maps' must be an object");
    bool have_ve = false, have_eta_i = false, have_f_lambda = false, have_eta_spark = false, have_theta_mbt = false,
         have_friction = false;
    if (j.contains("maps")) {
        const auto& mj = j.at("maps");
        const std::string mctx = ctx + ".maps";
        e.maps.source = mj.value("source", std::string());
        if (e.maps.source.empty()) throw std::runtime_error(mctx + ": 'source' is required (PLAN.md section 8)");

        if (!has_cycle && !mj.contains("ve")) throw std::runtime_error(mctx + ": 've' is required");
        if (mj.contains("ve")) {
            have_ve = true;
            e.maps.ve = read_engine_table(mj.at("ve"), mctx + ".ve", "p_im_kpa", kKpa, real(1.0));
            require_positive_y_axis(e.maps.ve, mctx + ".ve", "p_im_kpa");
            require_table_values(e.maps.ve, mctx + ".ve", real(0.0), false, real(2.0), false);
        }

        if (!has_cycle && !mj.contains("eta_i")) throw std::runtime_error(mctx + ": 'eta_i' is required");
        if (mj.contains("eta_i")) {
            have_eta_i = true;
            e.maps.eta_i = read_engine_table(mj.at("eta_i"), mctx + ".eta_i", "air_mass_per_rev_g", kGram, real(1.0));
            if (e.maps.eta_i.y.front() < real(0.0)) throw std::runtime_error(mctx + ".eta_i: 'air_mass_per_rev_g' must be >= 0");
            require_table_values(e.maps.eta_i, mctx + ".eta_i", real(0.0), false, real(1.0), false);
        }

        if (!has_cycle && !mj.contains("f_lambda")) throw std::runtime_error(mctx + ": 'f_lambda' is required");
        if (mj.contains("f_lambda")) {
            have_f_lambda = true;
            e.maps.f_lambda = read_curve(mj.at("f_lambda"));
            require_ascending(e.maps.f_lambda, mctx + ".f_lambda");
            if (e.maps.f_lambda.x.size() < 2) throw std::runtime_error(mctx + ".f_lambda: needs at least 2 points");
            if (!(e.maps.f_lambda.x.front() > real(0.0))) throw std::runtime_error(mctx + ".f_lambda: x (lambda) must be > 0");
            for (real y : e.maps.f_lambda.y) {
                if (!(y >= real(0.0)) || !std::isfinite(y)) throw std::runtime_error(mctx + ".f_lambda: every y must be >= 0 and finite");
            }
            if (e.maps.f_lambda.evaluate(real(1.0)) != real(1.0)) {
                throw std::runtime_error(mctx + ".f_lambda: f_lambda(1) must be exactly 1 (eta_i is defined at lambda = 1), got " +
                                          std::to_string(e.maps.f_lambda.evaluate(real(1.0))));
            }
        }

        if (!has_cycle && !mj.contains("eta_spark")) throw std::runtime_error(mctx + ": 'eta_spark' is required");
        if (mj.contains("eta_spark")) {
            have_eta_spark = true;
            e.maps.eta_spark = read_curve(mj.at("eta_spark"));
            require_ascending(e.maps.eta_spark, mctx + ".eta_spark");
            if (e.maps.eta_spark.x.size() < 2) throw std::runtime_error(mctx + ".eta_spark: needs at least 2 points");
            for (real y : e.maps.eta_spark.y) {
                if (!(y >= real(0.0) && y <= real(1.0))) throw std::runtime_error(mctx + ".eta_spark: every y must be in [0, 1]");
            }
            if (e.maps.eta_spark.evaluate(real(0.0)) != real(1.0)) {
                throw std::runtime_error(mctx + ".eta_spark: eta_spark(0) must be exactly 1 (eta_i is defined at MBT spark), got " +
                                          std::to_string(e.maps.eta_spark.evaluate(real(0.0))));
            }
        }

        if (!has_cycle && !mj.contains("theta_mbt_deg")) throw std::runtime_error(mctx + ": 'theta_mbt_deg' is required");
        if (mj.contains("theta_mbt_deg")) {
            have_theta_mbt = true;
            e.maps.theta_mbt_deg = read_engine_table(mj.at("theta_mbt_deg"), mctx + ".theta_mbt_deg", "p_im_kpa", kKpa, real(1.0));
            require_positive_y_axis(e.maps.theta_mbt_deg, mctx + ".theta_mbt_deg", "p_im_kpa");
            require_table_values(e.maps.theta_mbt_deg, mctx + ".theta_mbt_deg", real(-30.0), true, real(70.0), true);
        }

        if (!has_cycle && !mj.contains("friction_torque_nm")) throw std::runtime_error(mctx + ": 'friction_torque_nm' is required");
        if (mj.contains("friction_torque_nm")) {
            have_friction = true;
            e.maps.friction_torque_nm = read_curve(mj.at("friction_torque_nm"));
            require_ascending(e.maps.friction_torque_nm, mctx + ".friction_torque_nm");
            if (e.maps.friction_torque_nm.x.front() != real(0.0)) {
                throw std::runtime_error(mctx + ".friction_torque_nm: x[0] must be exactly 0 (its y[0] is the Coulomb part)");
            }
            for (real y : e.maps.friction_torque_nm.y) {
                if (!(y >= real(0.0)) || !std::isfinite(y)) {
                    throw std::runtime_error(mctx + ".friction_torque_nm: every y must be >= 0 and finite (a friction torque magnitude)");
                }
            }
        }

        if (mj.contains("pmep_excess_kpa")) {
            e.maps.pmep_excess_pa = read_engine_table(mj.at("pmep_excess_kpa"), mctx + ".pmep_excess_kpa", "p_im_kpa", kKpa, kKpa);
            require_positive_y_axis(e.maps.pmep_excess_pa, mctx + ".pmep_excess_kpa", "p_im_kpa");
            // N1: the runtime (engine_simulated.cpp's tau_pump term) uses
            // pmep_excess_pa as a plain SIGNED additive term with no sign
            // special-casing, and the C16a generator already produces a
            // signed value (provisional decision P-E) - a file value must
            // follow the same rule. The bound is a sane-magnitude sanity
            // check, not a physical constraint (real excess values are a
            // few kPa); it also rejects non-finite values (a NaN/Inf fails
            // both comparisons in require_table_values).
            require_table_values(e.maps.pmep_excess_pa, mctx + ".pmep_excess_kpa", real(-1e6), true, real(1e6), true);
            e.maps.has_pmep_excess = true;
        }
        if (mj.contains("theta_kl_deg")) {
            e.maps.theta_kl_deg = read_engine_table(mj.at("theta_kl_deg"), mctx + ".theta_kl_deg", "p_im_kpa", kKpa, real(1.0));
            require_positive_y_axis(e.maps.theta_kl_deg, mctx + ".theta_kl_deg", "p_im_kpa");
            require_table_values(e.maps.theta_kl_deg, mctx + ".theta_kl_deg", real(-30.0), true, real(70.0), true);
            e.maps.has_theta_kl = true;
        }

        if (!has_cycle) require_maps_efficiency_below_one(e.maps, mctx);
    }

    // --- shared engine fields (same meaning/validation as torque_map) ------------
    e.inertia_kgm2 = require_positive(j, "inertia_kgm2", ctx);
    e.breakaway_torque_nm = j.value("breakaway_torque_nm", real(0.0));
    if (!has_cycle && e.breakaway_torque_nm < e.maps.friction_torque_nm.evaluate(real(0.0))) {
        throw std::runtime_error(ctx + ": breakaway_torque_nm must be >= maps.friction_torque_nm(0)");
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
        if (!(e.idle.max_throttle > real(0.0) && e.idle.max_throttle <= real(1.0))) {
            throw std::runtime_error(ctx + ": idle_control.max_throttle must be in (0, 1], got " + std::to_string(e.idle.max_throttle));
        }
        // R6: a negative gain flips the PI loop's sign (destabilising, not
        // merely a weaker response) - reject rather than silently misbehave.
        if (e.idle.kp_per_rpm < real(0.0)) {
            throw std::runtime_error(ctx + ": idle_control.kp_per_rpm must be >= 0, got " + std::to_string(e.idle.kp_per_rpm));
        }
        if (e.idle.ki_per_rpm_s < real(0.0)) {
            throw std::runtime_error(ctx + ": idle_control.ki_per_rpm_s must be >= 0, got " + std::to_string(e.idle.ki_per_rpm_s));
        }
    }
    if (j.contains("limiter")) {
        const auto& lj = j.at("limiter");
        const std::string type = lj.value("type", std::string("fuel_cut"));
        if (type == "fuel_cut") e.limiter.type = LimiterType::FuelCut;
        else if (type == "soft_cut") e.limiter.type = LimiterType::SoftCut;
        else throw std::runtime_error(ctx + ": limiter.type must be \"fuel_cut\" or \"soft_cut\", got \"" + type + "\"");
        e.limiter.rpm = lj.value("rpm", e.limiter.rpm);
        e.limiter.hysteresis_rpm = lj.value("hysteresis_rpm", e.limiter.hysteresis_rpm);
        e.limiter.band_rpm = lj.value("band_rpm", e.limiter.band_rpm);
        if (e.limiter.type == LimiterType::SoftCut && !(e.limiter.band_rpm > real(0.0))) {
            throw std::runtime_error(ctx + ": limiter.band_rpm must be > 0 for a soft_cut limiter");
        }
        // R6: a negative hysteresis reverses the FuelCut latch's restore
        // direction (restores ABOVE the cut point instead of below it).
        if (e.limiter.hysteresis_rpm < real(0.0)) {
            throw std::runtime_error(ctx + ": limiter.hysteresis_rpm must be >= 0, got " + std::to_string(e.limiter.hysteresis_rpm));
        }
    }
    if (j.contains("starter")) {
        const auto& sj = j.at("starter");
        e.starter.stall_torque_nm = sj.value("stall_torque_nm", e.starter.stall_torque_nm);
        e.starter.free_speed_rpm = sj.value("free_speed_rpm", e.starter.free_speed_rpm);
        // R6: EngineTorqueMap/SimulatedEngine both divide by free_speed_rpm
        // (converted to rad/s) to build the starter's speed-torque line -
        // zero or negative divides by zero/flips its sign at runtime.
        if (!(e.starter.free_speed_rpm > real(0.0))) {
            throw std::runtime_error(ctx + ": starter.free_speed_rpm must be > 0, got " + std::to_string(e.starter.free_speed_rpm));
        }
        const std::string interlock = sj.value("interlock", std::string("clutch_or_neutral"));
        if (interlock == "clutch_or_neutral") e.starter.interlock = StarterDesc::Interlock::ClutchOrNeutral;
        else if (interlock == "none") e.starter.interlock = StarterDesc::Interlock::None;
        else throw std::runtime_error(ctx + ": starter.interlock must be \"clutch_or_neutral\" or \"none\", got \"" + interlock + "\"");
    }
    e.crank_axis_local = read_vec3(j.value("crank_axis_local", json()), Vec3::zero());
    require_zero_or_unit(e.crank_axis_local, ctx + ".crank_axis_local");
    const std::string initial_state = j.value("initial_state", std::string("running"));
    if (initial_state == "running") e.initial_state = EngineInitialState::Running;
    else if (initial_state == "off") e.initial_state = EngineInitialState::Off;
    else throw std::runtime_error(ctx + ": initial_state must be \"running\" or \"off\", got \"" + initial_state + "\"");
    if (!(e.stall_rpm < e.start_complete_rpm && e.start_complete_rpm <= e.idle_rpm && e.idle_rpm < e.limiter.rpm)) {
        throw std::runtime_error(ctx + ": engine rpm thresholds must satisfy stall_rpm < start_complete_rpm <= "
                                  "idle_rpm < limiter.rpm");
    }
    if (!(e.dfco_reenable_rpm < e.dfco_enable_rpm)) {
        throw std::runtime_error(ctx + ": dfco_reenable_rpm must be < dfco_enable_rpm");
    }

    // --- fuel (required) + ignition (required, spark only in I7b) -----------------
    if (j.contains("fuels") && j.contains("fuel")) {
        throw std::runtime_error(ctx + ": 'fuels' and 'fuel' are mutually exclusive ('fuel' is shorthand for a one-entry 'fuels')");
    }
    std::vector<std::string> fuel_refs;
    if (j.contains("fuels")) {
        if (!j.at("fuels").is_array() || j.at("fuels").empty()) {
            throw std::runtime_error(ctx + ": 'fuels' must be a non-empty array of path strings");
        }
        for (const auto& v : j.at("fuels")) {
            if (!v.is_string() || v.get<std::string>().empty()) {
                throw std::runtime_error(ctx + ": 'fuels' entries must be non-empty path strings");
            }
            fuel_refs.push_back(v.get<std::string>());
        }
    } else if (j.contains("fuel")) {
        if (!j.at("fuel").is_string() || j.at("fuel").get<std::string>().empty()) {
            throw std::runtime_error(ctx + ": 'fuel' must be a non-empty path string");
        }
        fuel_refs.push_back(j.at("fuel").get<std::string>());
    } else {
        throw std::runtime_error(ctx + ": a simulated engine requires 'fuels' (or the one-entry shorthand 'fuel') - its "
                                  "torque comes from burnt fuel");
    }
    std::vector<std::string> resolved_paths;
    for (const std::string& ref : fuel_refs) {
        const std::string fuel_path = resolve_path(engine_dir, ref);
        for (const std::string& seen : resolved_paths) {
            bool same = false;
            try {
                same = std::filesystem::equivalent(fuel_path, seen);
            } catch (const std::filesystem::filesystem_error&) {
                same = (fuel_path == seen);
            }
            if (same) throw std::runtime_error(ctx + ": 'fuels' contains a duplicate fuel reference (" + ref + ")");
        }
        resolved_paths.push_back(fuel_path);
        e.fuels.push_back(ps::io::load_fuel_json(fuel_path));
    }
    if (resolved_fuel_paths_out != nullptr) *resolved_fuel_paths_out = resolved_paths;

    if (!j.contains("ignition") || !j.at("ignition").is_string()) {
        throw std::runtime_error(ctx + ": 'ignition' is required for a simulated engine (\"spark\")");
    }
    const std::string ign = j.at("ignition").get<std::string>();
    if (ign == "spark") {
        e.ignition = EngineIgnition::Spark;
    } else if (ign == "compression") {
        throw std::runtime_error(ctx + ": ignition \"compression\" is not supported by the simulated engine yet (the I7 "
                                  "MVEM is a spark-ignition model; a CI combustion model is later)");
    } else {
        throw std::runtime_error(ctx + ": ignition must be \"spark\" or \"compression\", got \"" + ign + "\"");
    }

    // --- I7c C16a: maps generated from the 0D cycle model --------------------
    if (has_cycle) {
        const std::string cctx = ctx + ".cycle";
        if (e.architecture != EngineArchitecture::Piston4Stroke) {
            throw std::runtime_error(cctx + ": a cycle block needs architecture \"piston_4stroke\" (the rotary cycle model is "
                                      "I7e)");
        }
        e.cycle = read_cycle_block(j.at("cycle"), cctx, engine_dir, e.piston, have_friction);
        // C17a follow-up: the cycle's spark schedule IS the ECU's (the ecu
        // block's own table was rejected above) - the runtime then evaluates
        // exactly the table the generator ran its grid nodes at.
        if (!e.cycle->spark_advance_deg.x.empty()) e.ecu.spark_advance_deg = e.cycle->spark_advance_deg;
        // ps_audio A5 (PROVISIONAL P12): resolve the installation BEFORE the
        // generator runs - it is a generator input (the exhaust waveguide) and
        // part of the cache key. The vehicle component's override (already
        // resolved to a path by the caller) wins over the engine file's own
        // default_installation. A4 resolved it only after this function
        // returned, so a pipe engine would have generated without it.
        if (e.cycle->intake.has_value() && e.cycle->exhaust.has_value()) {
            const std::string& ip =
                installation_override_path != nullptr ? *installation_override_path : e.cycle->default_installation_path;
            e.cycle->installation = load_installation_json(ip);
            e.cycle->installation_path = ip;
        }
        const bool complete = have_ve && have_eta_i && have_f_lambda && have_eta_spark && have_theta_mbt && have_friction &&
                              e.maps.has_pmep_excess;
        if (!complete) {
            // Generator errors propagate unwrapped (their own "engine map
            // generator: ..." / "CycleModelDesc: ..." messages) - no catch
            // by reference here (vault TOOL-019, the Windows ASan leg).
            const EngineMapOptions defaults{};
            const EngineMapOptions& opt = map_options != nullptr ? *map_options : defaults;
            // PHYS-033 P4 I7c C16b (D8): an opt-in on-disk cache in front of
            // the generator call below. cache_dir empty (the default) means
            // load_cached_engine_maps/store_cached_engine_maps touch the
            // filesystem not at all (see map_cache.h's own contract) - this
            // whole block is then just the one key computation plus an
            // always-null load, bit-identical to pre-C16b behaviour.
            const std::uint64_t cache_key = drivetrain::engine_map_cache_key(*e.cycle, e.fuels.front(), opt.quality);
            EngineMapsPtr gen = drivetrain::load_cached_engine_maps(opt.cache_dir, cache_key,
                                                                    drivetrain::engine_map_generator_version(*e.cycle));
            if (!gen) {
                if (opt.generation_count_out != nullptr) ++*opt.generation_count_out;
                gen = generate_engine_maps(*e.cycle, e.fuels.front(), opt.quality, opt.pool);
                drivetrain::store_cached_engine_maps(opt.cache_dir, cache_key, *gen);
            }
            const SimulatedEngineMapsDesc& g = gen->maps;
            std::string generated_keys;
            auto note = [&](const char* k) { generated_keys += generated_keys.empty() ? k : std::string(", ") + k; };
            if (!have_ve) { e.maps.ve = g.ve; note("ve"); }
            if (!have_eta_i) { e.maps.eta_i = g.eta_i; note("eta_i"); }
            if (!have_f_lambda) { e.maps.f_lambda = g.f_lambda; note("f_lambda"); }
            if (!have_eta_spark) { e.maps.eta_spark = g.eta_spark; note("eta_spark"); }
            if (!have_theta_mbt) { e.maps.theta_mbt_deg = g.theta_mbt_deg; note("theta_mbt_deg"); }
            if (!have_friction) { e.maps.friction_torque_nm = g.friction_torque_nm; note("friction_torque_nm"); }
            if (!e.maps.has_pmep_excess) {
                e.maps.pmep_excess_pa = g.pmep_excess_pa;
                e.maps.has_pmep_excess = g.has_pmep_excess;
                note("pmep_excess_kpa");
            }
            e.maps.source = j.contains("maps") ? e.maps.source + " | generated from the cycle block (" + generated_keys + "): " + g.source
                                               : g.source;
        }
        const std::string mctx = ctx + ".maps";
        if (e.maps.friction_torque_nm.x.empty() || e.maps.friction_torque_nm.x.front() != real(0.0)) {
            throw std::runtime_error(mctx + ".friction_torque_nm: x[0] must be exactly 0 (its y[0] is the Coulomb part)");
        }
        for (real y : e.maps.friction_torque_nm.y) {
            if (!(y >= real(0.0)) || !std::isfinite(y)) {
                throw std::runtime_error(mctx + ".friction_torque_nm: every y must be >= 0 and finite (a friction torque magnitude)");
            }
        }
        require_maps_efficiency_below_one(e.maps, mctx);
        if (e.breakaway_torque_nm < e.maps.friction_torque_nm.evaluate(real(0.0))) {
            throw std::runtime_error(ctx + ": breakaway_torque_nm must be >= maps.friction_torque_nm(0)");
        }
    }

    return e;
}

// PHYS-033 P4 I7a C5: thin dispatcher on "kind" - "torque_map" (or absent,
// the only kind before C5) and, since PHYS-033 P4 I7b C10, "simulated"
// (the MVEM engine, read_simulated_fields above).
ps::drivetrain::ComponentParams read_engine_fields(const json& j, const std::string& ctx, const std::string& engine_dir,
                                                   std::vector<std::string>* resolved_fuel_paths_out = nullptr,
                                                   const EngineMapOptions* map_options = nullptr,
                                                   const std::string* installation_override_path = nullptr) {
    // Default "torque_map" (rather than requiring the field) is deliberate,
    // not an oversight: every shipped data/engines/*.json file happens to
    // set "kind" explicitly today, but nothing REQUIRES a hand-authored or
    // future engine file to - "torque_map" is the only kind that predates
    // this field, so an absent "kind" means exactly that, and this default
    // keeps such a file loading rather than rejecting it - see F5's "kind
    // absent defaults to torque_map" pin test (the only place this default
    // path is actually exercised, since no shipped file omits it).
    const std::string kind = j.value("kind", std::string("torque_map"));
    if (kind == "torque_map") return read_torque_map_fields(j, ctx, engine_dir, resolved_fuel_paths_out);
    if (kind == "simulated") {
        return read_simulated_fields(j, ctx, engine_dir, resolved_fuel_paths_out, map_options, installation_override_path);
    }
    throw std::runtime_error(ctx + ": engine 'kind' must be \"torque_map\" or \"simulated\", got \"" + kind + "\"");
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

// PHYS-033 P4 I7a C5: the directory a component's OWN further relative paths
// (an engine's "fuel" field) resolve against - the referenced file's own
// directory when `cj` uses "ref", else `base_dir` (the vehicle file's own
// directory) for an inline component. Mirrors resolve_component_ref's own
// "ref" check without re-reading/re-parsing the file.
std::string component_ref_dir(const json& cj, const std::string& base_dir) {
    if (!cj.contains("ref")) return base_dir;
    const std::string path = resolve_path(base_dir, cj.at("ref").get<std::string>());
    return std::filesystem::path(path).parent_path().string();
}

} // namespace

vehicle::TyreDesc load_tyre_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "tyre", 1, path);
    return read_tyre_object(j);
}

drivetrain::InstallationDesc load_installation_json(const std::string& path) {
    const json j = read_json_file(path);
    return read_installation_object(j, path);
}

vehicle::VehicleDesc load_vehicle_json(const std::string& path) { return load_vehicle_json(path, EngineMapOptions{}); }

vehicle::VehicleDesc load_vehicle_json(const std::string& path, const EngineMapOptions& options) {
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
    // PHYS-033 P4 I7a C6: engine id -> its resolved fuel file paths, and
    // tank id -> {fuel path as written, resolved fuel path}, for the fluid
    // link's "tank fuel is in the engine's fuels list" check.
    std::vector<std::pair<std::string, std::vector<std::string>>> engine_fuel_paths;
    std::vector<std::pair<std::string, std::pair<std::string, std::string>>> tank_fuel_paths;
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
                    const std::string engine_dir = component_ref_dir(cj, base_dir);
                    std::vector<std::string> fuel_paths;
                    // ps_audio A5 (P12): a well-formed installation override
                    // reaches the engine reader, which resolves it before any
                    // map generation; a malformed one is rejected below with
                    // A4's messages, exactly as before.
                    std::string inst_override;
                    const bool override_ok = cj.contains("installation") && cj.at("installation").is_string() &&
                                             !cj.at("installation").get<std::string>().empty();
                    if (override_ok) inst_override = resolve_path(base_dir, cj.at("installation").get<std::string>());
                    p.components.push_back({id, read_engine_fields(ej, ctx, engine_dir, &fuel_paths, &options,
                                                                   override_ok ? &inst_override : nullptr)});
                    engine_fuel_paths.emplace_back(id, std::move(fuel_paths)); // PHYS-033 P4 I7a C6: for the fluid-link check below

                    // ps_audio A4 (out/audio_plan.md section 3.2/3.3): resolve
                    // this engine's installation - the vehicle component's own
                    // "installation" override if given, else the engine file's
                    // own "default_installation" (already validated/resolved
                    // into cycle->default_installation_path by read_cycle_block,
                    // required-with/rejected-without pipes there). Only visible
                    // here, once both the vehicle-component JSON (cj, which may
                    // carry the override) and the just-loaded SimulatedEngineDesc
                    // (whose cycle->intake/exhaust say whether pipes exist) are
                    // both in hand.
                    if (auto* sim = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&p.components.back().params)) {
                        const bool has_pipes = sim->cycle.has_value() && sim->cycle->intake.has_value() && sim->cycle->exhaust.has_value();
                        const bool has_override = cj.contains("installation");
                        if (has_override) {
                            if (!cj.at("installation").is_string() || cj.at("installation").get<std::string>().empty()) {
                                throw std::runtime_error(ctx + ": 'installation' must be a non-empty path string");
                            }
                            if (!has_pipes) {
                                throw std::runtime_error(ctx + ": 'installation' is only allowed for an engine whose cycle "
                                                          "block has 'intake'/'exhaust' pipe blocks");
                            }
                            // ps_audio A5 (P12): already loaded by
                            // read_simulated_fields (installation_path ==
                            // inst_override), before map generation.
                        }
                        // else: the engine file's default_installation, also
                        // resolved by read_simulated_fields when pipes exist.
                    } else if (cj.contains("installation")) {
                        throw std::runtime_error(ctx + ": 'installation' is only allowed on a simulated engine with "
                                                  "'intake'/'exhaust' pipe blocks");
                    }
                } else if (type == "fuel_tank") {
                    // PHYS-033 P4 I7a C6: always inline (no 'ref' - a tank is
                    // a per-vehicle fill state, not a reusable part file).
                    // "fuel" resolves relative to the VEHICLE file's own
                    // directory (base_dir), like every inline component's
                    // relative path.
                    if (cj.contains("ref")) throw std::runtime_error(ctx + ": 'fuel_tank' components are always inline (no 'ref')");
                    if (!cj.contains("fuel") || !cj.at("fuel").is_string() || cj.at("fuel").get<std::string>().empty()) {
                        throw std::runtime_error(ctx + ": fuel_tank 'fuel' is required and must be a non-empty path string");
                    }
                    ps::drivetrain::FuelTankDesc t;
                    auto read_mass = [&](const char* key) -> real {
                        if (!cj.contains(key) || !cj.at(key).is_number()) {
                            throw std::runtime_error(ctx + ": fuel_tank '" + key + "' is required and must be a number");
                        }
                        const real v = cj.at(key).get<real>();
                        if (!std::isfinite(v)) throw std::runtime_error(ctx + ": fuel_tank '" + key + "' must be finite");
                        if (v < real(0.0)) throw std::runtime_error(ctx + ": fuel_tank '" + key + "' must not be negative, got " + std::to_string(v));
                        return v;
                    };
                    t.capacity_kg = read_mass("capacity_kg");
                    t.initial_kg = read_mass("initial_kg");
                    if (!(t.capacity_kg > real(0.0))) throw std::runtime_error(ctx + ": fuel_tank 'capacity_kg' must be > 0");
                    if (t.initial_kg > t.capacity_kg) {
                        throw std::runtime_error(ctx + ": fuel_tank 'initial_kg' (" + std::to_string(t.initial_kg) +
                                                  ") must not exceed 'capacity_kg' (" + std::to_string(t.capacity_kg) + ")");
                    }
                    const std::string tank_fuel_path = resolve_path(base_dir, cj.at("fuel").get<std::string>());
                    t.fuel = ps::io::load_fuel_json(tank_fuel_path);
                    tank_fuel_paths.push_back({id, {cj.at("fuel").get<std::string>(), tank_fuel_path}});
                    p.components.push_back({id, t});
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
                // PHYS-033 P4 I7b C10: either engine kind. A simulated engine
                // has no governor, so the hand-lever check below never fires
                // for it.
                struct EngineView {
                    ps::drivetrain::ThrottleInput throttle_input;
                    bool has_governor;
                    std::string governor_channel;
                };
                auto engine_view = [](const ps::drivetrain::ComponentDesc& c) -> std::optional<EngineView> {
                    if (const auto* e = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) {
                        return EngineView{e->throttle_input, e->has_governor, e->governor.channel};
                    }
                    if (const auto* e = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
                        return EngineView{e->throttle_input, false, std::string()};
                    }
                    return std::nullopt;
                };
                std::optional<EngineView> engine_opt;
                bool vehicle_has_engine = false;
                for (const auto& c : p.components) {
                    if (engine_view(c).has_value()) vehicle_has_engine = true;
                }
                if (!vehicle_has_engine) {
                    throw std::runtime_error(tctx + ": a throttle_control component in a vehicle with no engine component");
                }
                for (const auto& c : p.components) {
                    if (c.id == tcd_ptr->engine) {
                        engine_opt = engine_view(c);
                        break;
                    }
                }
                // Item 5: the ref must name an actual torque_map engine
                // component - covers "unknown id" and "id exists but is some
                // other type" (a vehicle with no engine at all is rejected
                // just above with its own message).
                if (!engine_opt.has_value()) {
                    throw std::runtime_error(tctx + ": 'engine' (\"" + tcd_ptr->engine +
                                              "\") must name a \"torque_map\" engine component or a \"simulated\" engine component in this vehicle's powertrain");
                }
                const EngineView* engine_ptr = &*engine_opt;
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
                    tcd_ptr->hand_lever_channel != engine_ptr->governor_channel) {
                    throw std::runtime_error(tctx + ": 'hand_lever_channel' (\"" + tcd_ptr->hand_lever_channel +
                                              "\") differs from engine '" + tcd_ptr->engine + "'.governor.channel (\"" +
                                              engine_ptr->governor_channel +
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

        // PHYS-033 P4 I7a C6: optional "fluid" - [["<tank>.out", "<engine>.fuel"], ...].
        // Each link feeds one engine's fuel input from one fuel_tank; the tank's
        // fuel file must be one of that engine's own fuel files (compared with
        // std::filesystem::equivalent, like the engine's own duplicate check).
        if (pj.contains("fluid")) {
            const auto& fj = pj.at("fluid");
            const std::string malformed = "must be a [\"<tank>.out\", \"<engine>.fuel\"] pair of port strings";
            if (!fj.is_array()) {
                throw std::runtime_error("vehicle '" + v.name + "': powertrain.fluid must be an array of [\"<tank>.out\", \"<engine>.fuel\"] pairs");
            }
            auto find_component = [&p](const std::string& cid) -> const ps::drivetrain::ComponentDesc* {
                for (const auto& c : p.components) {
                    if (c.id == cid) return &c;
                }
                return nullptr;
            };
            std::size_t link_index = 0;
            for (const auto& lj : fj) {
                const std::string lctx = "vehicle '" + v.name + "': powertrain.fluid[" + std::to_string(link_index++) + "]";
                if (!lj.is_array() || lj.size() != 2 || !lj[0].is_string() || !lj[1].is_string()) {
                    throw std::runtime_error(lctx + ": " + malformed);
                }
                auto parse_ref = [&](const std::string& ref) {
                    const auto dot = ref.find('.');
                    if (dot == std::string::npos || dot == 0 || dot + 1 == ref.size()) {
                        throw std::runtime_error(lctx + ": bad port ref \"" + ref + "\" - " + malformed);
                    }
                    ps::drivetrain::PortRef pr;
                    pr.component = ref.substr(0, dot);
                    pr.port = ref.substr(dot + 1);
                    return pr;
                };
                const std::string from_str = lj[0].get<std::string>();
                const std::string to_str = lj[1].get<std::string>();
                const ps::drivetrain::PortRef from = parse_ref(from_str);
                const ps::drivetrain::PortRef to = parse_ref(to_str);

                const ps::drivetrain::ComponentDesc* from_c = find_component(from.component);
                if (from_c == nullptr) throw std::runtime_error(lctx + ": unknown component '" + from.component + "'");
                if (std::get_if<ps::drivetrain::FuelTankDesc>(&from_c->params) == nullptr) {
                    throw std::runtime_error(lctx + ": '" + from.component + "' is not a fuel_tank (a fluid link must start at \"<tank>.out\")");
                }
                if (from.port != "out") {
                    throw std::runtime_error(lctx + ": unknown port \"" + from_str + "\" (a fuel_tank's only fluid port is \"out\")");
                }
                const ps::drivetrain::ComponentDesc* to_c = find_component(to.component);
                if (to_c == nullptr) throw std::runtime_error(lctx + ": unknown component '" + to.component + "'");
                const auto* engine = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&to_c->params);
                const auto* sim_engine = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&to_c->params);
                if (engine == nullptr && sim_engine == nullptr) {
                    throw std::runtime_error(lctx + ": '" + to.component + "' is not an engine (a fluid link must end at \"<engine>.fuel\")");
                }
                if (to.port != "fuel") {
                    throw std::runtime_error(lctx + ": unknown port \"" + to_str + "\" (an engine's only fluid port is \"fuel\")");
                }
                if (!(engine != nullptr ? ps::drivetrain::has_fuel_model(*engine) : ps::drivetrain::has_fuel_model(*sim_engine))) {
                    throw std::runtime_error(lctx + ": engine '" + to.component +
                                              "' has no fuel model (no 'fuels', 'fuel' or 'fuel_map') - a fluid link cannot feed it");
                }
                for (const auto& existing : p.fluid) {
                    if (existing.to.component != to.component) continue;
                    if (existing.from.component == from.component) {
                        throw std::runtime_error(lctx + ": duplicate fluid link \"" + from_str + "\" -> \"" + to_str + "\"");
                    }
                    throw std::runtime_error(lctx + ": engine '" + to.component + "' fuel port is fed by two tanks ('" +
                                              existing.from.component + "' and '" + from.component + "')");
                }
                const std::pair<std::string, std::string>* tank_path = nullptr;
                for (const auto& tp : tank_fuel_paths) {
                    if (tp.first == from.component) tank_path = &tp.second;
                }
                const std::vector<std::string>* engine_paths = nullptr;
                for (const auto& ep : engine_fuel_paths) {
                    if (ep.first == to.component) engine_paths = &ep.second;
                }
                bool listed = false;
                if (tank_path != nullptr && engine_paths != nullptr) {
                    for (const std::string& ep : *engine_paths) {
                        bool same = false;
                        try {
                            same = std::filesystem::equivalent(tank_path->second, ep);
                        } catch (const std::filesystem::filesystem_error&) {
                            same = (tank_path->second == ep);
                        }
                        if (same) listed = true;
                    }
                }
                if (!listed) {
                    throw std::runtime_error(lctx + ": tank fuel \"" + (tank_path != nullptr ? tank_path->first : std::string("?")) +
                                              "\" is not in engine \"" + to.component + "\"'s fuels list");
                }
                p.fluid.push_back({from, to});
            }
        }
    }
    // PHYS-033 P4 I7a C6: every tank must feed something (whether or not a
    // "fluid" array exists at all).
    for (const auto& c : p.components) {
        if (std::get_if<ps::drivetrain::FuelTankDesc>(&c.params) == nullptr) continue;
        bool consumed = false;
        for (const auto& link : p.fluid) consumed = consumed || link.from.component == c.id;
        if (!consumed) {
            throw std::runtime_error("vehicle '" + v.name + "': powertrain.components['" + c.id +
                                      "']: fuel_tank has no consumer (no powertrain.fluid link from \"" + c.id + ".out\")");
        }
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
