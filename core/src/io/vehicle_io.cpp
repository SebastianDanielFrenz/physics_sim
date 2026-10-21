#include "ps/drivetrain/turbo_maps.h"
#include "ps/math/transcendental.h"
#include <cstring>
#include <sstream>
#include "ps/io/vehicle_io.h"
#include "aero_io.h"
#include "nitrous_components.h"
#include "engine_parts.h"
#include "turbo_stage_json.h"

#include "../engine_cycle/map_cache.h"
#include "../vehicle/tir_parser.h"
#include "ps/vehicle/tyre_model.h"
#include "ps/drivetrain/table2d.h"
#include "ps/engine_cycle/firing_order.h"
#include "ps/io/engine_thermal_io.h"
#include "ps/io/fuel_io.h"
#include "ps/io/electronics_io.h"
#include "ps/drivetrain/isa_air.h"
#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/limiter_range.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
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

// G1 load warnings: a number as the message shows it ("8250", "98.5").
std::string fmt_rpm(real v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", static_cast<double>(v));
    return b;
}
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

// The ONE post-parse validation of a TyreDesc: read_tyre_object (file / inline tyre) ends in it, and a desc injected
// through VehicleLoadOptions::tyres goes through it unchanged, so both paths accept exactly the same tyres.
void validate_tyre_desc(const TyreDesc& d) {
    if (d.source.empty()) throw std::runtime_error("tyre '" + d.name + "': 'source' is required (PLAN.md section 8)");
    if (d.rolling_resistance_v_eps <= real(0.0)) {
        throw std::runtime_error("tyre '" + d.name + "': rolling_resistance_v_eps must be > 0, got " +
                                  std::to_string(d.rolling_resistance_v_eps));
    }
    if (d.low_speed_damping_ratio < real(0.0) || d.low_speed_damping_ratio > real(2.0)) {
        throw std::runtime_error("tyre '" + d.name + "': low_speed_damping_ratio must be in [0, 2], got " +
                                  std::to_string(d.low_speed_damping_ratio));
    }
    if (d.low_speed_damping_vx <= real(0.0)) {
        throw std::runtime_error("tyre '" + d.name + "': low_speed_damping_vx must be > 0, got " +
                                  std::to_string(d.low_speed_damping_vx));
    }
}

TyreDesc read_tyre_object(const json& j) {
    TyreDesc d;
    d.name = j.value("name", std::string());
    d.source = j.value("source", std::string());

    d.unloaded_radius = j.value("unloaded_radius", d.unloaded_radius);
    d.width = j.value("width", d.width);
    d.rim_radius = j.value("rim_radius", d.rim_radius);
    d.fz0 = j.value("fz0", d.fz0);
    d.vertical_stiffness = j.value("vertical_stiffness", d.vertical_stiffness);
    d.v0 = j.value("v0", d.v0);
    d.qsy1 = j.value("qsy1", d.qsy1);
    d.qsy2 = j.value("qsy2", d.qsy2);
    d.qsy3 = j.value("qsy3", d.qsy3);
    d.qsy4 = j.value("qsy4", d.qsy4);
    if (j.contains("rolling_resistance")) {
        // Optional anchor block deriving qsy1/qsy3/qsy4 in closed form
        // (ps::vehicle::derive_rolling_resistance, docs/data_formats/tyre.md).
        const std::string ctx = "tyre '" + d.name + "': rolling_resistance";
        for (const char* key : {"qsy1", "qsy3", "qsy4"}) {
            if (j.contains(key)) {
                throw std::runtime_error(ctx + ": cannot be combined with an explicit '" + key +
                                          "' (the block derives qsy1, qsy3 and qsy4)");
            }
        }
        const json& rr = j.at("rolling_resistance");
        if (!rr.is_object()) throw std::runtime_error(ctx + ": must be an object");
        for (auto it = rr.begin(); it != rr.end(); ++it) {
            const std::string& k = it.key();
            if (k != "rated_speed_kmh" && k != "crr_base" && k != "rise_at_fraction" && k != "factor_at_rated") {
                throw std::runtime_error(ctx + ": unknown key '" + k + "'");
            }
        }
        const auto need_number = [&](const json& obj, const char* key, const std::string& where) {
            if (!obj.contains(key) || !obj.at(key).is_number()) {
                throw std::runtime_error(where + ": missing required number '" + key + "'");
            }
            return obj.at(key).get<double>();
        };
        ps::vehicle::RollingResistanceAnchors anchors;
        anchors.rated_speed_kmh = need_number(rr, "rated_speed_kmh", ctx);
        anchors.crr_base = need_number(rr, "crr_base", ctx);
        anchors.factor_at_rated = need_number(rr, "factor_at_rated", ctx);
        if (!rr.contains("rise_at_fraction") || !rr.at("rise_at_fraction").is_object()) {
            throw std::runtime_error(ctx + ": missing required object 'rise_at_fraction'");
        }
        const json& rise = rr.at("rise_at_fraction");
        for (auto it = rise.begin(); it != rise.end(); ++it) {
            if (it.key() != "fraction" && it.key() != "factor") {
                throw std::runtime_error(ctx + ".rise_at_fraction: unknown key '" + it.key() + "'");
            }
        }
        anchors.fraction = need_number(rise, "fraction", ctx + ".rise_at_fraction");
        anchors.factor = need_number(rise, "factor", ctx + ".rise_at_fraction");
        try {
            const auto c = ps::vehicle::derive_rolling_resistance(anchors, d.v0);
            d.qsy1 = c.qsy1;
            d.qsy3 = c.qsy3;
            d.qsy4 = c.qsy4;
            d.rolling_resistance_anchors = anchors;
        } catch (const std::invalid_argument& e) {
            throw std::runtime_error("tyre '" + d.name + "': " + e.what());
        }
    }
    if (j.contains("rolling_resistance_speed_law")) {
        if (!j.at("rolling_resistance_speed_law").is_string())
            throw std::runtime_error("rolling_resistance_speed_law must be a string");
        const auto law = j.at("rolling_resistance_speed_law").get<std::string>();
        if (law != "quadratic" && law != "fourth_power")
            throw std::runtime_error("rolling_resistance_speed_law must be quadratic or fourth_power");
        ps::vehicle::select_rolling_resistance_speed_law(d, law == "quadratic" ?
            ps::vehicle::RollingResistanceSpeedLaw::Quadratic : ps::vehicle::RollingResistanceSpeedLaw::FourthPower);
    }
    d.rolling_resistance_v_eps = j.value("rolling_resistance_v_eps", d.rolling_resistance_v_eps);
    d.relaxation_length_x = j.value("relaxation_length_x", d.relaxation_length_x);
    d.relaxation_length_y = j.value("relaxation_length_y", d.relaxation_length_y);
    d.lambda_mux = j.value("lambda_mux", d.lambda_mux);
    d.lambda_muy = j.value("lambda_muy", d.lambda_muy);

    // PHYS-033 I1: additive fields, tyre/1 unchanged.
    d.low_speed_damping_ratio = j.value("low_speed_damping_ratio", d.low_speed_damping_ratio);
    d.low_speed_damping_vx = j.value("low_speed_damping_vx", d.low_speed_damping_vx);

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
    validate_tyre_desc(d);
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

// `injected` (VehicleLoadOptions::tyres, keyed by wheel name; nullptr = none) REPLACES the wheel's file "tyre", which
// is then not read at all. The caller has already checked that a tyre exists one way or the other.
WheelDesc read_wheel(const json& j, const std::string& base_dir, const TyreDesc* injected) {
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
    if (injected != nullptr) {
        validate_tyre_desc(*injected);
        w.tyre = *injected;
    } else if (j.contains("tyre")) {
        w.tyre = read_tyre_ref(j.at("tyre"), base_dir);
    }
    // W6 (weather plan 7.5): a physics_sim.tyre_profile/1 path/name. Stored
    // only; W6 resolves the environment's default_tyre_profile (W7 per-wheel).
    w.tyre_profile = j.value("tyre_profile", w.tyre_profile);
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
    if (j.contains("regen_blend")) {
        const json& rj = j.at("regen_blend");
        if (!rj.is_object()) throw std::runtime_error("brake_system.regen_blend must be an object");
        for (auto it = rj.begin(); it != rj.end(); ++it) {
            const std::string& k = it.key();
            if (k != "enabled" && k != "taper_start_soc" && k != "full_soc" && k != "fade_start_speed_mps" &&
                k != "fade_end_speed_mps" && k != "abs_hold_s" && k != "path_min_engagement" && k != "clutch_capacity_margin" &&
                k != "path_ramp_s") {
                throw std::runtime_error("brake_system.regen_blend: unknown key '" + k + "'");
            }
            if (k == "enabled" ? !it.value().is_boolean() : !it.value().is_number()) {
                throw std::runtime_error("brake_system.regen_blend." + k + (k == "enabled" ? " must be a boolean" : " must be a number"));
            }
        }
        auto& r = b.regen_blend;
        r.enabled = rj.value("enabled", r.enabled);
        r.taper_start_soc = rj.value("taper_start_soc", r.taper_start_soc);
        r.full_soc = rj.value("full_soc", r.full_soc);
        r.fade_start_speed_mps = rj.value("fade_start_speed_mps", r.fade_start_speed_mps);
        r.fade_end_speed_mps = rj.value("fade_end_speed_mps", r.fade_end_speed_mps);
        r.abs_hold_s = rj.value("abs_hold_s", r.abs_hold_s);
        r.path_min_engagement = rj.value("path_min_engagement", r.path_min_engagement);
        r.clutch_capacity_margin = rj.value("clutch_capacity_margin", r.clutch_capacity_margin);
        r.path_ramp_s = rj.value("path_ramp_s", r.path_ramp_s);
        if (!(r.taper_start_soc >= real(0.0) && r.taper_start_soc < r.full_soc && r.full_soc <= real(1.0))) {
            throw std::runtime_error("brake_system.regen_blend: need 0 <= taper_start_soc < full_soc <= 1, got " +
                                     std::to_string(r.taper_start_soc) + " and " + std::to_string(r.full_soc));
        }
        if (!(r.fade_end_speed_mps >= real(0.0) && r.fade_end_speed_mps < r.fade_start_speed_mps)) {
            throw std::runtime_error("brake_system.regen_blend: need 0 <= fade_end_speed_mps < fade_start_speed_mps, got " +
                                     std::to_string(r.fade_end_speed_mps) + " and " + std::to_string(r.fade_start_speed_mps));
        }
        if (!(r.abs_hold_s >= real(0.0)) || !std::isfinite(r.abs_hold_s)) {
            throw std::runtime_error("brake_system.regen_blend.abs_hold_s must be >= 0");
        }
        if (!(r.path_ramp_s >= real(0.0)) || !std::isfinite(r.path_ramp_s)) {
            throw std::runtime_error("brake_system.regen_blend.path_ramp_s must be >= 0");
        }
        if (!(r.path_min_engagement > real(0.0) && r.path_min_engagement <= real(1.0))) {
            throw std::runtime_error("brake_system.regen_blend.path_min_engagement must be in (0, 1]");
        }
        if (!(r.clutch_capacity_margin > real(0.0) && r.clutch_capacity_margin <= real(1.0))) {
            throw std::runtime_error("brake_system.regen_blend.clutch_capacity_margin must be in (0, 1]");
        }
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
    "switching", // engine garage P9: cam-profile switching (needs a cycle block with profiles)
    "lift_control", // engine garage P10: continuous intake-lift strategy (needs a cycle block with intake_lift_family)
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
    "combustion_eff_lean", "exhaust_backflow_temperature_k", "friction", "grid_rpm", "grid_p_im_kpa", "pressure_sampling",
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
    // Engine garage P9: discrete cam profiles (VTEC-style), docs/data_formats/cam.md "profiles".
    "profiles",
    // Engine garage P10: continuous intake lift (Valvetronic-style), docs/data_formats/cam.md "intake_lift_family".
    "intake_lift_family",
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
    if (cj.contains("profiles")) {
        // Engine garage P9: VTEC-style discrete cam profiles. Each profile carries both lifts; the single lifts are then
        // profile 0's (generator and key per profile: cycle_desc_for_cam_profile).
        if (cj.contains("intake_lift") || cj.contains("exhaust_lift")) {
            throw std::runtime_error(cctx + ": 'profiles' and 'intake_lift'/'exhaust_lift' together - with cam profiles every lift "
                                      "lives inside its profile");
        }
        const json& pj = cj.at("profiles");
        if (!pj.is_array()) throw std::runtime_error(cctx + ": 'profiles' must be an array of {id, intake_lift, exhaust_lift}");
        if (pj.size() != CycleModelDesc::kMaxCamProfiles) {
            throw std::runtime_error(cctx + ": 'profiles' must hold exactly " + std::to_string(CycleModelDesc::kMaxCamProfiles) +
                                      " profiles (low, then high: one ECU switching rule selects between two), got " +
                                      std::to_string(pj.size()));
        }
        for (std::size_t k = 0; k < pj.size(); ++k) {
            const std::string pctx = cctx + ".profiles[" + std::to_string(k) + "]";
            const json& p = pj[k];
            if (!p.is_object()) throw std::runtime_error(pctx + " must be an object");
            for (auto it = p.begin(); it != p.end(); ++it) {
                const std::string& key = it.key();
                const bool note = key == "source" || (key.size() > 7 && key.compare(key.size() - 7, 7, "_source") == 0);
                if (key != "id" && key != "intake_lift" && key != "exhaust_lift" && !note) {
                    throw std::runtime_error(pctx + ": unknown key '" + key + "' (a profile has id, intake_lift, exhaust_lift)");
                }
            }
            if (!p.contains("id") || !p.at("id").is_string() || p.at("id").get<std::string>().empty()) {
                throw std::runtime_error(pctx + ": 'id' must be a non-empty string");
            }
            CycleModelDesc::CamProfile cp;
            cp.id = p.at("id").get<std::string>();
            for (const auto& prev : d.cam_profiles) {
                if (prev.id == cp.id) throw std::runtime_error(pctx + ": duplicate profile id '" + cp.id + "'");
            }
            cp.intake_lift = read_cycle_lift(p, "intake_lift", pctx);
            cp.exhaust_lift = read_cycle_lift(p, "exhaust_lift", pctx);
            d.cam_profiles.push_back(std::move(cp));
        }
        d.intake_lift = d.cam_profiles.front().intake_lift;
        d.exhaust_lift = d.cam_profiles.front().exhaust_lift;
        if (cj.contains("intake_lift_family")) {
            throw std::runtime_error(cctx + ": 'profiles' and 'intake_lift_family' together - discrete cam profiles (P9) and a "
                                      "continuous intake lift family (P10) are mutually exclusive");
        }
    } else if (cj.contains("intake_lift_family")) {
        // Engine garage P10: Valvetronic-style continuous intake lift. Nodes over the normalised eccentric-shaft position u
        // in strictly ascending order, the last exactly u = 1 (full lift); the exhaust lift stays fixed. intake_lift then
        // holds the u = 1 node's lift (the generator and key per node: cycle_desc_for_lift_node).
        if (cj.contains("intake_lift")) {
            throw std::runtime_error(cctx + ": 'intake_lift_family' and 'intake_lift' together - with a lift family every intake "
                                      "lift lives inside its node (the u = 1 node is the full lift)");
        }
        const json& fj = cj.at("intake_lift_family");
        if (!fj.is_array()) {
            throw std::runtime_error(cctx + ": 'intake_lift_family' must be an array of {u, intake_lift}");
        }
        if (fj.size() < CycleModelDesc::kMinLiftNodes || fj.size() > CycleModelDesc::kMaxLiftNodes) {
            throw std::runtime_error(cctx + ": 'intake_lift_family' must hold " + std::to_string(CycleModelDesc::kMinLiftNodes) +
                                      " to " + std::to_string(CycleModelDesc::kMaxLiftNodes) + " nodes, got " +
                                      std::to_string(fj.size()));
        }
        for (std::size_t k = 0; k < fj.size(); ++k) {
            const std::string nctx = cctx + ".intake_lift_family[" + std::to_string(k) + "]";
            const json& n = fj[k];
            if (!n.is_object()) throw std::runtime_error(nctx + " must be an object");
            for (auto it = n.begin(); it != n.end(); ++it) {
                const std::string& key = it.key();
                const bool note = key == "source" || (key.size() > 7 && key.compare(key.size() - 7, 7, "_source") == 0);
                if (key != "u" && key != "intake_lift" && !note) {
                    throw std::runtime_error(nctx + ": unknown key '" + key + "' (a lift node has u, intake_lift)");
                }
            }
            CycleModelDesc::LiftNode ln;
            ln.u = require_number(n, "u", nctx);
            if (!(ln.u >= real(0.0) && ln.u <= real(1.0))) {
                throw std::runtime_error(nctx + ": 'u' must lie in [0, 1], got " + std::to_string(ln.u));
            }
            if (!d.lift_family.empty() && !(ln.u > d.lift_family.back().u)) {
                throw std::runtime_error(nctx + ": the nodes' 'u' must be strictly ascending (" + std::to_string(ln.u) +
                                          " after " + std::to_string(d.lift_family.back().u) + ")");
            }
            ln.intake_lift = read_cycle_lift(n, "intake_lift", nctx);
            d.lift_family.push_back(std::move(ln));
        }
        if (d.lift_family.back().u != real(1.0)) {
            throw std::runtime_error(cctx + ": the last 'intake_lift_family' node must be u = 1 (full lift), got u = " +
                                      std::to_string(d.lift_family.back().u));
        }
        d.intake_lift = d.lift_family.back().intake_lift;
        d.exhaust_lift = read_cycle_lift(cj, "exhaust_lift", cctx);
    } else {
        d.intake_lift = read_cycle_lift(cj, "intake_lift", cctx);
        d.exhaust_lift = read_cycle_lift(cj, "exhaust_lift", cctx);
    }
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
    // Non-throwing form (firing_order_error), not try/catch + e.what():
    // catching an exception by reference and then reading it is the
    // clang-cl 19.1.5 Windows ASan crash pattern (vault TOOL-019, CLAUDE.md
    // "Windows ASan caveat") - core/ must never do that.
    if (const std::string err = ps::engine_cycle::firing_order_error(d.firing_order, d.cylinders); !err.empty()) {
        throw std::runtime_error(cctx + ": " + err);
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
    // Non-throwing form - see the firing_order_error comment just above.
    if (const std::string err = ps::engine_cycle::firing_angles_error(d.firing_angles_deg, d.cylinders); !err.empty()) {
        throw std::runtime_error(cctx + ": " + err);
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
    if (cj.contains("pressure_sampling")) {
        const auto& a = cj.at("pressure_sampling");
        if (!a.is_object()) throw std::runtime_error(cctx + ": pressure_sampling must be an object");
        for (auto it=a.begin(); it!=a.end(); ++it)
            if (it.key()!="enabled" && it.key()!="min_kpa" && it.key()!="max_kpa" && it.key()!="relative_error" && it.key()!="max_points" && it.key()!="max_passes" && it.key()!="allow_budget_limit")
                throw std::runtime_error(cctx + ": unknown pressure_sampling field " + it.key());
        auto& p = d.pressure_sampling;
        p.enabled = a.value("enabled", true);
        p.allow_budget_limit = a.value("allow_budget_limit", false);
        p.min_pa = a.value("min_kpa", real(0)) * real(1000);
        p.max_pa = a.value("max_kpa", real(0)) * real(1000);
        p.relative_error = a.value("relative_error", real(0.01));
        const int budget = a.value("max_points", 256);
        p.max_passes = a.value("max_passes", 8);
        if (budget<5 || budget>1024 || p.max_passes<1 || p.max_passes>12 || !(p.relative_error>0 && p.relative_error<=0.1) ||
            !std::isfinite(p.min_pa) || !std::isfinite(p.max_pa) || p.min_pa<0 || p.max_pa<0 ||
            (p.min_pa>0 && p.max_pa>0 && p.min_pa>=p.max_pa))
            throw std::runtime_error(cctx + ": invalid pressure_sampling bounds/tolerance/budget");
        p.max_points = static_cast<std::size_t>(budget);
    }
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
                                                          const std::string* installation_override_path = nullptr,
                                                          const parts::AssembledEngine* assembled = nullptr) {
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
    // Engine garage P2: the forced-induction slot (engine default_forced_induction, the vehicle component's or the
    // user layer's forced_induction) was resolved by parts::assemble_engine; the winning configuration is loaded
    // here, before the turbo tables are built, so the derived cache key follows the file actually used.
    if (assembled != nullptr && assembled->has_forced_induction) {
        // A validate-only load (generate = false) writes nothing, the derived turbo table cache included.
        e.turbo_pair = load_turbo_configuration_json(assembled->forced_induction_path,
                                                     (map_options && map_options->generate) ? map_options->cache_dir : std::string{});
    }

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
    // Engine garage P7: the ECU's ignition strategy. "schedule" (default, every engine before P7): the spark is the
    // ECU table. "knock_limited": spark = min(table, theta_kl) with theta_kl generated from the cycle block and the
    // fuels' octane (or given as maps.theta_kl_deg); the table may then be absent = MBT tracking (the ECU follows
    // maps.theta_mbt_deg, filled at the end of this function).
    bool knock_limited = false;
    real knock_margin_deg = real(0.0);
    bool knock_mbt_tracking = false;
    {
        const auto& ej = j.at("ecu");
        const std::string ectx = ctx + ".ecu";
        if (ej.contains("ignition_strategy")) {
            if (!ej.at("ignition_strategy").is_string()) throw std::runtime_error(ectx + ": 'ignition_strategy' must be a string");
            const std::string st = ej.at("ignition_strategy").get<std::string>();
            if (st == "knock_limited") {
                knock_limited = true;
            } else if (st != "schedule") {
                throw std::runtime_error(ectx + ": ignition_strategy must be \"schedule\" or \"knock_limited\", got \"" + st + "\"");
            }
        }
        if (ej.contains("knock_margin_deg")) {
            if (!knock_limited) {
                throw std::runtime_error(ectx + ": 'knock_margin_deg' needs ignition_strategy \"knock_limited\"");
            }
            if (!ej.at("knock_margin_deg").is_number()) throw std::runtime_error(ectx + ": 'knock_margin_deg' must be a number");
            knock_margin_deg = ej.at("knock_margin_deg").get<real>();
            if (!(knock_margin_deg >= real(0.0) && knock_margin_deg <= real(10.0))) {
                throw std::runtime_error(ectx + ": 'knock_margin_deg' must be in [0, 10] deg, got " + std::to_string(knock_margin_deg));
            }
        } else if (knock_limited) {
            throw std::runtime_error(ectx + ": ignition_strategy \"knock_limited\" needs 'knock_margin_deg' (deg crank the ECU "
                                            "keeps below the knock limit, in [0, 10])");
        }
        // Pedal plan P1 (2.5): the ECU's part-load boost strategy on a turbo engine - "efficient" (default: wastegate first,
        // the turbo spools only for torque the atmospheric engine cannot make) or "spooled" (the rotor is kept at the boost
        // cap and the plate throttles; sharper response, more fuel). Read on every engine, used only with forced induction.
        if (ej.contains("part_load_boost")) {
            if (!ej.at("part_load_boost").is_string()) throw std::runtime_error(ectx + ": 'part_load_boost' must be a string");
            const std::string pl = ej.at("part_load_boost").get<std::string>();
            if (pl == "spooled") {
                e.part_load_boost = PartLoadBoost::Spooled;
            } else if (pl != "efficient") {
                throw std::runtime_error(ectx + ": part_load_boost must be \"efficient\" or \"spooled\", got \"" + pl + "\"");
            }
        }
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
        } else if (knock_limited && !ej.contains("spark_advance_deg")) {
            knock_mbt_tracking = true; // P7: no table - the ECU tracks MBT, limited by theta_kl
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
    // P7: a knock-limited engine without a cycle block has nothing to generate theta_kl from.
    const bool have_theta_kl_file = e.maps.has_theta_kl;
    if (knock_limited && !has_cycle && !have_theta_kl_file) {
        throw std::runtime_error(ctx + ": ecu.ignition_strategy \"knock_limited\" needs a cycle block (to generate the knock "
                                       "limit) or maps.theta_kl_deg");
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
        e.idle.max_torque_nm = ij.value("max_torque_nm", e.idle.max_torque_nm);
        if (!(std::isfinite(e.idle.max_torque_nm) && e.idle.max_torque_nm >= real(0.0))) {
            throw std::runtime_error(ctx + ": idle_control.max_torque_nm must be >= 0, got " + std::to_string(e.idle.max_torque_nm));
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

    // --- engine garage P9: the ECU's cam-profile switching rule (docs/data_formats/ecu.md "switching") ------------------
    const bool has_switching = j.contains("switching");
    if (has_switching) {
        const json& sj = j.at("switching");
        const std::string sctx = ctx + ".switching";
        if (!sj.is_object()) throw std::runtime_error(sctx + " must be an object");
        for (auto it = sj.begin(); it != sj.end(); ++it) {
            const std::string& k = it.key();
            const bool note = k == "source" || (k.size() > 7 && k.compare(k.size() - 7, 7, "_source") == 0);
            if (k != "type" && k != "rpm_on" && k != "rpm_off" && k != "min_p_im_kpa" && k != "min_oil_c" && k != "transition_s" &&
                !note) {
                throw std::runtime_error(sctx + ": unknown key '" + k +
                                          "' (switching has type, rpm_on, rpm_off, min_p_im_kpa, min_oil_c, transition_s)");
            }
        }
        if (!sj.contains("type") || !sj.at("type").is_string() || sj.at("type").get<std::string>() != "discrete") {
            throw std::runtime_error(sctx + ": 'type' must be \"discrete\" (the only switching type; continuous lift is P10)");
        }
        CamSwitchingDesc& cs = e.cam_switching;
        cs.rpm_on = require_number(sj, "rpm_on", sctx);
        cs.rpm_off = require_number(sj, "rpm_off", sctx);
        cs.min_p_im_pa = require_number(sj, "min_p_im_kpa", sctx) * kKpa;
        cs.min_oil_c = require_number(sj, "min_oil_c", sctx);
        cs.transition_s = require_number(sj, "transition_s", sctx);
        for (real v : {cs.rpm_on, cs.rpm_off, cs.min_p_im_pa, cs.min_oil_c, cs.transition_s}) {
            if (!std::isfinite(v)) throw std::runtime_error(sctx + ": every value must be finite");
        }
        if (!(cs.rpm_off < cs.rpm_on)) {
            throw std::runtime_error(sctx + ": rpm_off must be < rpm_on (the hysteresis band), got rpm_on " + std::to_string(cs.rpm_on) +
                                      ", rpm_off " + std::to_string(cs.rpm_off));
        }
        if (!(cs.rpm_off > e.idle_rpm) || !(cs.rpm_on < e.limiter.rpm)) {
            throw std::runtime_error(sctx + ": the switching band must lie between idle_rpm and limiter.rpm (idle_rpm < rpm_off < "
                                      "rpm_on < limiter.rpm) - the idle controller runs on the low profile");
        }
        if (!(cs.min_p_im_pa >= real(0.0))) throw std::runtime_error(sctx + ": 'min_p_im_kpa' must be >= 0");
        if (!(cs.min_oil_c >= real(-40.0) && cs.min_oil_c <= real(150.0))) {
            throw std::runtime_error(sctx + ": 'min_oil_c' must be in [-40, 150] deg C");
        }
        if (!(cs.transition_s >= real(0.0) && cs.transition_s <= real(2.0))) {
            throw std::runtime_error(sctx + ": 'transition_s' must be in [0, 2] s");
        }
    }

    // --- engine garage P10: the ECU's continuous intake-lift strategy (docs/data_formats/ecu.md "lift_control") ---------
    // Node membership of min_u/idle_u/plate_open_above_u is checked against the cam's family below (cycle block).
    const bool has_lift_control = j.contains("lift_control");
    if (has_lift_control) {
        const json& lj = j.at("lift_control");
        const std::string lctx = ctx + ".lift_control";
        if (!lj.is_object()) throw std::runtime_error(lctx + " must be an object");
        for (auto it = lj.begin(); it != lj.end(); ++it) {
            const std::string& k = it.key();
            const bool note = k == "source" || (k.size() > 7 && k.compare(k.size() - 7, 7, "_source") == 0);
            if (k != "type" && k != "actuator_tau_s" && k != "max_rate_per_s" && k != "min_u" && k != "idle_u" &&
                k != "plate_open_above_u" && k != "vacuum_target_kpa" && !note) {
                throw std::runtime_error(lctx + ": unknown key '" + k +
                                          "' (lift_control has type, actuator_tau_s, max_rate_per_s, min_u, idle_u, "
                                          "plate_open_above_u, vacuum_target_kpa)");
            }
        }
        if (!lj.contains("type") || !lj.at("type").is_string() || lj.at("type").get<std::string>() != "continuous") {
            throw std::runtime_error(lctx + ": 'type' must be \"continuous\" (the only lift-control type)");
        }
        LiftControlDesc& lc = e.lift_control;
        lc.actuator_tau_s = require_number(lj, "actuator_tau_s", lctx);
        lc.max_rate_per_s = require_number(lj, "max_rate_per_s", lctx);
        lc.min_u = require_number(lj, "min_u", lctx);
        lc.idle_u = require_number(lj, "idle_u", lctx);
        lc.plate_open_u = require_number(lj, "plate_open_above_u", lctx);
        lc.vacuum_pa = require_number(lj, "vacuum_target_kpa", lctx) * kKpa;
        for (real v : {lc.actuator_tau_s, lc.max_rate_per_s, lc.min_u, lc.idle_u, lc.plate_open_u, lc.vacuum_pa}) {
            if (!std::isfinite(v)) throw std::runtime_error(lctx + ": every value must be finite");
        }
        if (!(lc.actuator_tau_s >= real(0.0) && lc.actuator_tau_s <= real(1.0))) {
            throw std::runtime_error(lctx + ": 'actuator_tau_s' must be in [0, 1] s");
        }
        if (!(lc.max_rate_per_s > real(0.0))) throw std::runtime_error(lctx + ": 'max_rate_per_s' must be > 0");
        if (!(lc.min_u >= real(0.0) && lc.min_u < real(1.0))) throw std::runtime_error(lctx + ": 'min_u' must be in [0, 1)");
        if (!(lc.idle_u >= lc.min_u && lc.idle_u < real(1.0))) {
            throw std::runtime_error(lctx + ": 'idle_u' must be in [min_u, 1)");
        }
        if (!(lc.plate_open_u > lc.min_u && lc.plate_open_u <= real(1.0))) {
            throw std::runtime_error(lctx + ": 'plate_open_above_u' must be in (min_u, 1]");
        }
        if (!(lc.vacuum_pa >= real(0.0) && lc.vacuum_pa <= real(30.0) * kKpa)) {
            throw std::runtime_error(lctx + ": 'vacuum_target_kpa' must be in [0, 30] kPa");
        }
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
    if (!has_cycle && has_lift_control) {
        throw std::runtime_error(ctx + ": the ECU has a 'lift_control' block but the engine has no cycle block - an intake lift "
                                  "family is generated from a cycle block's cam (docs/data_formats/cam.md)");
    }
    if (!has_cycle && has_switching) {
        throw std::runtime_error(ctx + ": the ECU has a 'switching' block but the engine has no cycle block - cam profiles are "
                                  "generated from a cycle block's cam (docs/data_formats/cam.md)");
    }
    if (has_cycle) {
        const std::string cctx = ctx + ".cycle";
        if (e.architecture != EngineArchitecture::Piston4Stroke) {
            throw std::runtime_error(cctx + ": a cycle block needs architecture \"piston_4stroke\" (the rotary cycle model is "
                                      "I7e)");
        }
        e.cycle = read_cycle_block(j.at("cycle"), cctx, engine_dir, e.piston, have_friction);
        // Engine garage P9: cam profiles and the ECU's switching rule come as a pair.
        if (!e.cycle->cam_profiles.empty()) {
            if (!has_switching) {
                throw std::runtime_error(ctx + ": the cam has profiles (cycle.profiles) but the ECU has no 'switching' block - give the "
                                          "ECU part a switching rule (docs/data_formats/ecu.md), or use a single-lift cam");
            }
            if (j.contains("maps")) {
                throw std::runtime_error(ctx + ": an engine with cam profiles generates every map from its cycle block, one set per "
                                          "profile - a 'maps' block is not supported with profiles");
            }
            if (knock_mbt_tracking) {
                throw std::runtime_error(ctx + ": a knock-limited ECU without a spark table (MBT tracking) is not supported with cam "
                                          "profiles yet - give the ECU a spark schedule");
            }
            for (const auto& p : e.cycle->cam_profiles) e.cam_profile_ids.push_back(p.id);
        } else if (has_switching) {
            throw std::runtime_error(ctx + ": the ECU has a 'switching' block but the cam has no profiles - give the engine a cam with "
                                      "'profiles' (docs/data_formats/cam.md), or remove the switching block");
        }
        // Engine garage P10: an intake lift family and the ECU's lift_control come as a pair.
        if (!e.cycle->lift_family.empty()) {
            if (!has_lift_control) {
                throw std::runtime_error(ctx + ": the cam has an intake lift family (cycle.intake_lift_family) but the ECU has no "
                                          "'lift_control' block - give the ECU part a lift strategy (docs/data_formats/ecu.md), or "
                                          "use a single-lift cam");
            }
            if (j.contains("maps")) {
                throw std::runtime_error(ctx + ": an engine with an intake lift family generates every map from its cycle block, one "
                                          "set per lift node - a 'maps' block is not supported with a lift family");
            }
            if (knock_mbt_tracking) {
                throw std::runtime_error(ctx + ": a knock-limited ECU without a spark table (MBT tracking) is not supported with an "
                                          "intake lift family yet - give the ECU a spark schedule");
            }
            if (e.turbo_pair.has_value()) {
                throw std::runtime_error(ctx + ": an intake lift family with forced induction is not supported (the boost controller "
                                          "works on the throttle plate, which the lift strategy holds open) - remove the forced "
                                          "induction or use a single-lift cam");
            }
            if (e.throttle_input != ThrottleInput::Electronic) {
                throw std::runtime_error(ctx + ": an intake lift family needs throttle_input \"electronic\" (the ECU sets the lift "
                                          "and the plate from the torque request)");
            }
            auto is_node = [&](real u) {
                for (const auto& n : e.cycle->lift_family) {
                    if (n.u == u) return true;
                }
                return false;
            };
            const LiftControlDesc& lc = e.lift_control;
            const std::pair<const char*, real> named[] = {{"min_u", lc.min_u}, {"idle_u", lc.idle_u},
                                                          {"plate_open_above_u", lc.plate_open_u}};
            for (const auto& nv : named) {
                if (!is_node(nv.second)) {
                    throw std::runtime_error(ctx + ".lift_control: '" + std::string(nv.first) + "' = " + std::to_string(nv.second) +
                                              " is not one of the cam's intake_lift_family node u values (the ECU's tables "
                                              "are exact at the nodes)");
                }
            }
            for (const auto& n : e.cycle->lift_family) e.lift_u.push_back(n.u);
        } else if (has_lift_control) {
            throw std::runtime_error(ctx + ": the ECU has a 'lift_control' block but the cam has no intake lift family - give the "
                                      "engine a cam with 'intake_lift_family' (docs/data_formats/cam.md), or remove lift_control");
        }
        // C17a follow-up: the cycle's spark schedule IS the ECU's (the ecu
        // block's own table was rejected above) - the runtime then evaluates
        // exactly the table the generator ran its grid nodes at.
        if (!e.cycle->spark_advance_deg.x.empty()) e.ecu.spark_advance_deg = e.cycle->spark_advance_deg;
        // Engine garage P7: the knock limit is a generator input only for an engine that opts in. Every listed fuel
        // gets its own table (fuels order); each must carry a "knock" block. A file theta_kl_deg overrides the
        // generated one (the complete-override rule) - nothing is generated for knock then.
        if (knock_limited && !have_theta_kl_file) {
            e.cycle->knock.enabled = true;
            e.cycle->knock.margin_deg = knock_margin_deg;
            e.cycle->knock.lambda_target = e.ecu.lambda_target; // P7b: the replay runs at the ECU's lambda
            for (const FuelDesc& f : e.fuels) {
                if (!f.has_knock) {
                    throw std::runtime_error(ctx + ": ecu.ignition_strategy \"knock_limited\" needs a 'knock' block (RON/MON) in "
                                                   "every listed fuel; fuel '" + f.name + "' has none");
                }
                e.cycle->knock.fuels.push_back(CycleModelDesc::KnockFuel{f.name, f.knock.ron, f.knock.mon});
            }
        }
        // Engine garage P2: forced induction "none" on a pipe engine needs a grid the ambient intake path can reach
        // (the generator fails a node above ambient without a compressor supply - test_engine_map_gen_v2 G3).
        // Checked at load (also validate-only) so the message names the fix. Lumped engines generate between fixed
        // reservoirs at the node pressure and are left to the generator (design 7.3).
        if (assembled != nullptr && assembled->forced_induction_none && e.cycle->intake.has_value() && e.cycle->exhaust.has_value()) {
            real top = e.cycle->grid_p_im.empty() ? real(0.0) : e.cycle->grid_p_im.back();
            if (e.cycle->pressure_sampling.enabled && e.cycle->pressure_sampling.max_pa > top) top = e.cycle->pressure_sampling.max_pa;
            if (top > kIsaAirPressurePa) {
                throw std::runtime_error(cctx + ": forced induction is \"none\" ('" + assembled->forced_induction_origin +
                                         "') but cycle.grid_p_im_kpa reaches " + std::to_string(top / kKpa) +
                                         " kPa, above ambient (101.325 kPa [C ISO 2533]) - without a compressor the intake "
                                         "cannot reach those nodes. Fix: give the engine a grid_p_im_kpa (and "
                                         "pressure_sampling.max_kpa) that a naturally aspirated intake reaches - below "
                                         "ambient by the wide-open throttle and runner losses, so the top node a few kPa "
                                         "under 101.325 - e.g. a user engine that 'extends' this one and replaces "
                                         "cycle.grid_p_im_kpa");
            }
        }
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
            // A turbo feeds the throttle from its compressor; the generator's
            // grid nodes are then supplied, not drawn through the airbox.
            e.cycle->intake->compressor_supply = e.turbo_pair.has_value();
        }
        const bool complete = have_ve && have_eta_i && have_f_lambda && have_eta_spark && have_theta_mbt && have_friction &&
                              e.maps.has_pmep_excess && (!knock_limited || have_theta_kl_file);
        const bool have_pmep_file = e.maps.has_pmep_excess; // P7d: before the generated table is copied in
        const std::string mctx_gen = ctx + ".maps";
        if (!e.cycle->cam_profiles.empty() || !e.cycle->lift_family.empty()) {
            // Engine garage P9 (P10: likewise one set per intake lift node, u = 1 first): one complete map set per cam profile, each with its own .psmaps key (the key of the same
            // engine written with that cam alone). Progress reports the sum over the profiles that are generated (cache
            // hits add nothing); the cancel flag stops the whole load; the cache files are written only after every set
            // exists, so a cancelled load leaves no trace (the P1 contract).
            const EngineMapOptions defaults{};
            const EngineMapOptions& opt = map_options != nullptr ? *map_options : defaults;
            const std::vector<CycleModelDesc> pdesc = drivetrain::engine_map_set_descs(*e.cycle);
            const std::size_t np = pdesc.size();
            if (!opt.generate) {
                for (const CycleModelDesc& d : pdesc) drivetrain::validate_engine_map_generation(d, e.fuels.front(), opt.quality);
                e.maps_pending = true;
            } else {
                if (opt.cancel != nullptr && opt.cancel->load(std::memory_order_relaxed)) throw drivetrain::GenerationCancelled();
                std::vector<std::uint64_t> keys(np, 0);
                std::vector<EngineMapsPtr> gens(np);
                std::vector<bool> generated(np, false);
                for (std::size_t k = 0; k < np; ++k) {
                    keys[k] = drivetrain::engine_map_cache_key(pdesc[k], e.fuels.front(), opt.quality);
                    gens[k] = drivetrain::load_cached_engine_maps(opt.cache_dir, keys[k], drivetrain::engine_map_generator_version(pdesc[k]));
                }
                std::vector<std::size_t> points(np, 0);
                std::size_t total = 0;
                if (opt.on_progress) {
                    for (std::size_t k = 0; k < np; ++k) {
                        if (!gens[k]) points[k] = drivetrain::engine_map_progress_total(pdesc[k], e.fuels.front(), opt.quality);
                        total += points[k];
                    }
                }
                std::size_t offset = 0;
                for (std::size_t k = 0; k < np; ++k) {
                    if (gens[k]) continue;
                    if (opt.generation_count_out != nullptr) ++*opt.generation_count_out;
                    drivetrain::MapGenerationControl control;
                    control.cancel = opt.cancel;
                    if (opt.on_progress) {
                        control.on_progress = [&opt, offset, total](std::size_t done, std::size_t) {
                            opt.on_progress(std::min(offset + done, total), total);
                        };
                    }
                    if (k == 0 && opt.on_partial_engine) {
                        // A preview of the low profile alone (its partial maps; the other profiles are not generated yet).
                        control.on_partial_maps = [&](EngineMapsPtr partial) {
                            SimulatedEngineDesc preview = e;
                            preview.maps = partial->maps;
                            preview.cam_profile_ids.clear();
                            preview.cam_profile_maps.clear();
                            preview.lift_u.clear(); // P10: the u = 1 set alone (the engine without the family)
                            preview.lift_node_maps.clear();
                            preview.maps_pending = false;
                            opt.on_partial_engine(preview);
                        };
                    }
                    gens[k] = generate_engine_maps(pdesc[k], e.fuels.front(), opt.quality, opt.pool, &control);
                    generated[k] = true;
                    offset += points[k];
                }
                for (std::size_t k = 0; k < np; ++k) {
                    if (generated[k]) drivetrain::store_cached_engine_maps(opt.cache_dir, keys[k], *gens[k]);
                }
                e.maps = gens[0]->maps;
                for (std::size_t k = 1; k < np; ++k) {
                    if (e.cycle->lift_family.empty()) e.cam_profile_maps.push_back(gens[k]->maps);
                    else e.lift_node_maps.push_back(gens[k]->maps);
                }
            }
        } else if (!complete) {
            // Generator errors propagate unwrapped (their own "engine map
            // generator: ..." / "CycleModelDesc: ..." messages) - no catch
            // by reference here (vault TOOL-019, the Windows ASan leg).
            const EngineMapOptions defaults{};
            const EngineMapOptions& opt = map_options != nullptr ? *map_options : defaults;
            if (!opt.generate) {
                // Engine garage P1: validate-only. Everything above already ran; add the generator's
                // own build-time checks, then leave the maps pending (no generator, no cache access).
                drivetrain::validate_engine_map_generation(*e.cycle, e.fuels.front(), opt.quality);
                e.maps_pending = true;
            } else {
            // Engine garage P1: a request to cancel that is already set stops the load before
            // any work, even where the cache would have hit.
            if (opt.cancel != nullptr && opt.cancel->load(std::memory_order_relaxed)) throw drivetrain::GenerationCancelled();
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
                drivetrain::MapGenerationControl control;
                control.cancel = opt.cancel;
                control.on_progress = opt.on_progress;
                if(opt.on_partial_engine)control.on_partial_maps=[&](EngineMapsPtr partial) {
                    auto preview=e;const auto& g=partial->maps;
                    if(!have_ve)preview.maps.ve=g.ve;
                    if(!have_eta_i)preview.maps.eta_i=g.eta_i;
                    if(!have_f_lambda)preview.maps.f_lambda=g.f_lambda;
                    if(!have_eta_spark)preview.maps.eta_spark=g.eta_spark;
                    if(!have_theta_mbt)preview.maps.theta_mbt_deg=g.theta_mbt_deg;
                    if(!have_friction)preview.maps.friction_torque_nm=g.friction_torque_nm;
                    if(!preview.maps.has_pmep_excess){preview.maps.pmep_excess_pa=g.pmep_excess_pa;preview.maps.has_pmep_excess=g.has_pmep_excess;}
                    if(!have_ve && !have_pmep_file && !have_theta_kl_file)preview.maps.wot_p_im_pa=g.wot_p_im_pa; // P7d
                    preview.maps.source=g.source;preview.maps_pending=false;
                    opt.on_partial_engine(preview);
                };
                // A cancelled generation throws here, before the cache store: no file is written.
                gen = generate_engine_maps(*e.cycle, e.fuels.front(), opt.quality, opt.pool, &control);
                drivetrain::store_cached_engine_maps(opt.cache_dir, cache_key, *gen);
            }
            const SimulatedEngineMapsDesc& g = gen->maps;
            if (!g.wot_p_im_pa.x.empty()) {
                // Engine garage P7d: generated ve/pmep_excess/theta_kl carry a wide-open-throttle top row read through
                // map_pressure_index; a file table on the plain p_im axis cannot share the engine with them.
                const bool gen_p_tables = !have_ve || !have_pmep_file || (knock_limited && !have_theta_kl_file);
                const bool file_p_tables = have_ve || have_pmep_file || have_theta_kl_file;
                if (gen_p_tables && file_p_tables) {
                    throw std::runtime_error(mctx_gen + ": the generated maps of this pipe engine carry a wide-open-throttle row "
                                             "(engine garage P7d); give all of ve, pmep_excess_kpa and theta_kl in the maps "
                                             "block, or none of them");
                }
                if (gen_p_tables) e.maps.wot_p_im_pa = g.wot_p_im_pa;
            }
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
            if (knock_limited && !have_theta_kl_file) {
                // P7: the generated per-fuel knock tables; fuels[0]'s is active (Powertrain re-selects the tank's).
                e.maps.theta_kl_deg = g.theta_kl_deg;
                e.maps.theta_kl_by_fuel = g.theta_kl_by_fuel;
                e.maps.has_theta_kl = g.has_theta_kl;
                note("theta_kl_deg");
            }
            e.maps.source = j.contains("maps") ? e.maps.source + " | generated from the cycle block (" + generated_keys + "): " + g.source
                                               : g.source;
            } // generate
        }
        const std::string mctx = ctx + ".maps";
        if (e.maps_pending) {
            // Pending (validate-only): the generated keys are missing. What does not depend on them still
            // runs. The friction curve is given by the file, or - when it is generated - it is exactly
            // the correlation the generator would tabulate, whose value at rpm 0 is the Coulomb part.
            real friction0 = real(0.0);
            if (have_friction) {
                friction0 = e.maps.friction_torque_nm.evaluate(real(0.0));
            } else {
                friction0 = drivetrain::cycle_friction_torque_nm(*e.cycle, real(0.0));
            }
            if (e.breakaway_torque_nm < friction0) {
                throw std::runtime_error(ctx + ": breakaway_torque_nm must be >= maps.friction_torque_nm(0)");
            }
        } else {
        if (e.maps.friction_torque_nm.x.empty() || e.maps.friction_torque_nm.x.front() != real(0.0)) {
            throw std::runtime_error(mctx + ".friction_torque_nm: x[0] must be exactly 0 (its y[0] is the Coulomb part)");
        }
        for (real y : e.maps.friction_torque_nm.y) {
            if (!(y >= real(0.0)) || !std::isfinite(y)) {
                throw std::runtime_error(mctx + ".friction_torque_nm: every y must be >= 0 and finite (a friction torque magnitude)");
            }
        }
        require_maps_efficiency_below_one(e.maps, mctx);
        for (std::size_t k = 0; k < e.cam_profile_maps.size(); ++k) {
            // Engine garage P9: every further cam profile's generated set passes the same efficiency check.
            require_maps_efficiency_below_one(e.cam_profile_maps[k], mctx + " (cam profile '" + e.cam_profile_ids.at(k + 1) + "')");
        }
        for (std::size_t k = 0; k < e.lift_node_maps.size(); ++k) {
            // Engine garage P10: every lift node's generated set passes the same efficiency check.
            require_maps_efficiency_below_one(e.lift_node_maps[k], mctx + " (intake lift node u = " + std::to_string(e.lift_u.at(k)) + ")");
        }
        if (e.breakaway_torque_nm < e.maps.friction_torque_nm.evaluate(real(0.0))) {
            throw std::runtime_error(ctx + ": breakaway_torque_nm must be >= maps.friction_torque_nm(0)");
        }
        }
    }

    // Engine garage P7: an MBT-tracking knock-limited ECU follows the (final) theta_MBT map. Left empty while the maps
    // are pending - a pending engine is refused by SimulatedEngine anyway, and the garage completes it with
    // with_engine_maps, which keeps the ECU (so a pending MBT-tracking engine must be reloaded, see engine.md).
    if (knock_mbt_tracking && !e.maps_pending) e.ecu.spark_advance_deg = e.maps.theta_mbt_deg;

    return e;
}

// Engine garage P2 (design A8): optional engine mass metadata - data only, read by no physics and part of no
// cache key or state hash (until the centre-of-mass request R5).
void read_engine_mass(const json& j, const std::string& ctx, real& mass_kg, std::string& mass_source) {
    if (j.contains("mass_kg")) {
        if (!j.at("mass_kg").is_number()) throw std::runtime_error(ctx + ": 'mass_kg' must be a number");
        mass_kg = j.at("mass_kg").get<real>();
        if (!(mass_kg > real(0.0)) || !std::isfinite(mass_kg)) {
            throw std::runtime_error(ctx + ": 'mass_kg' must be > 0 and finite, got " + std::to_string(mass_kg));
        }
    }
    if (j.contains("mass_source")) {
        if (!j.at("mass_source").is_string()) throw std::runtime_error(ctx + ": 'mass_source' must be a string");
        if (!j.contains("mass_kg")) throw std::runtime_error(ctx + ": 'mass_source' without 'mass_kg'");
        mass_source = j.at("mass_source").get<std::string>();
    }
}

// PHYS-033 P4 I7a C5: thin dispatcher on "kind" - "torque_map" (or absent,
// the only kind before C5) and, since PHYS-033 P4 I7b C10, "simulated"
// (the MVEM engine, read_simulated_fields above).

// G1 (ps/drivetrain/limiter_range.h): the engine file's optional "rpm_rolloff_band_fraction", in (0, 1].
real read_rolloff_band_fraction(const json& j, const std::string& ctx, real fallback) {
    if (!j.contains("rpm_rolloff_band_fraction")) return fallback;
    const json& v = j.at("rpm_rolloff_band_fraction");
    if (!v.is_number() || !std::isfinite(v.get<real>()) || !(v.get<real>() > real(0.0)) || v.get<real>() > real(1.0)) {
        throw std::runtime_error(ctx + ": 'rpm_rolloff_band_fraction' must be a number in (0, 1]");
    }
    return v.get<real>();
}

// G1 load warning: the limiter lies above the engine's data, so the fired torque rolls off past it.
template <class Engine>
void warn_rpm_rolloff(const Engine& e, const std::string& ctx, std::vector<std::string>& out) {
    const ps::drivetrain::RpmRolloff r = ps::drivetrain::rpm_rolloff(e);
    if (!r.armed()) return;
    out.push_back(ctx + ": Limiting: the " + fmt_rpm(ps::drivetrain::fired_top_rpm(e)) + " rpm limiter is above the engine's data (it ends at " +
                  fmt_rpm(r.start_rpm) + " rpm): the fired torque rolls off to zero at " + fmt_rpm(r.end_rpm) + " rpm");
}

ps::drivetrain::ComponentParams read_engine_fields(const json& j, const std::string& ctx, const std::string& engine_dir,
                                                   std::vector<std::string>* resolved_fuel_paths_out = nullptr,
                                                   const EngineMapOptions* map_options = nullptr,
                                                   const std::string* installation_override_path = nullptr,
                                                   const parts::AssembledEngine* assembled = nullptr) {
    // Default "torque_map" (rather than requiring the field) is deliberate,
    // not an oversight: every shipped data/engines/*.json file happens to
    // set "kind" explicitly today, but nothing REQUIRES a hand-authored or
    // future engine file to - "torque_map" is the only kind that predates
    // this field, so an absent "kind" means exactly that, and this default
    // keeps such a file loading rather than rejecting it - see F5's "kind
    // absent defaults to torque_map" pin test (the only place this default
    // path is actually exercised, since no shipped file omits it).
    const std::string kind = j.value("kind", std::string("torque_map"));
    if (kind == "torque_map") {
        ps::drivetrain::TorqueMapEngineDesc e = read_torque_map_fields(j, ctx, engine_dir, resolved_fuel_paths_out);
        read_engine_mass(j, ctx, e.mass_kg, e.mass_source);
        e.rpm_rolloff_band_fraction = read_rolloff_band_fraction(j, ctx, e.rpm_rolloff_band_fraction);
        if (map_options != nullptr && map_options->warnings_out != nullptr) warn_rpm_rolloff(e, ctx, *map_options->warnings_out);
        return e;
    }
    if (kind == "simulated") {
        ps::drivetrain::SimulatedEngineDesc e =
            read_simulated_fields(j, ctx, engine_dir, resolved_fuel_paths_out, map_options, installation_override_path, assembled);
        read_engine_mass(j, ctx, e.mass_kg, e.mass_source);
        e.rpm_rolloff_band_fraction = read_rolloff_band_fraction(j, ctx, e.rpm_rolloff_band_fraction);
        if (j.contains("plate_monotone_tolerance")) { // G1
            const json& v = j.at("plate_monotone_tolerance");
            if (!v.is_number() || !std::isfinite(v.get<real>()) || v.get<real>() < real(0.0) || v.get<real>() > real(0.05)) {
                throw std::runtime_error(ctx + ": 'plate_monotone_tolerance' must be a number in [0, 0.05]");
            }
            e.plate_monotone_tolerance = v.get<real>();
        }
        if (map_options != nullptr && map_options->warnings_out != nullptr) {
            // Stage seam S0: advisory only, nothing of the result depends on it.
            for (const auto& issue : ps::drivetrain::check_turbo_stage_coverage(e)) {
                map_options->warnings_out->push_back(ctx + ": " + ps::drivetrain::to_string(issue.severity) + ": " + issue.message);
            }
            warn_rpm_rolloff(e, ctx, *map_options->warnings_out); // G1
        }
        return e;
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

// CVT plan C1: the "cvt" block of a cvt gearbox (docs/data_formats/gearbox.md). Unknown keys rejected.
ps::drivetrain::CvtDesc read_cvt_block(const json& o, const std::string& ctx) {
    auto check_keys = [](const json& b, std::initializer_list<const char*> keys, const std::string& where) {
        if (!b.is_object()) throw std::runtime_error(where + ": must be an object");
        for (auto it = b.begin(); it != b.end(); ++it) {
            bool ok = false;
            for (const char* k : keys) ok = ok || it.key() == k;
            if (!ok) throw std::runtime_error(where + ": unknown key '" + it.key() + "'");
        }
    };
    auto num = [](const json& b, const char* key, real def, const std::string& where) {
        if (!b.contains(key)) return def;
        const json& v = b.at(key);
        if (!v.is_number() || !std::isfinite(v.get<real>())) throw std::runtime_error(where + ": '" + key + "' must be a finite number");
        return v.get<real>();
    };
    check_keys(o, {"ratio_low", "ratio_high", "ratio_rate_per_s", "step_rate_per_s", "steps", "efficiency_vs_ratio", "pump_drag_nm",
                   "pump_drag_nm_per_kn", "belt"},
               ctx);
    ps::drivetrain::CvtDesc c;
    for (const char* k : {"ratio_low", "ratio_high", "steps", "efficiency_vs_ratio", "belt"}) {
        if (!o.contains(k)) throw std::runtime_error(ctx + ": '" + k + "' is required");
    }
    c.ratio_low = num(o, "ratio_low", c.ratio_low, ctx);
    c.ratio_high = num(o, "ratio_high", c.ratio_high, ctx);
    c.ratio_rate_per_s = num(o, "ratio_rate_per_s", c.ratio_rate_per_s, ctx);
    c.step_rate_per_s = num(o, "step_rate_per_s", c.step_rate_per_s, ctx);
    c.pump_drag_nm = num(o, "pump_drag_nm", c.pump_drag_nm, ctx);
    c.pump_drag_nm_per_kn = num(o, "pump_drag_nm_per_kn", c.pump_drag_nm_per_kn, ctx);
    if (!(c.ratio_high > real(0.0)) || !(c.ratio_low > c.ratio_high)) throw std::runtime_error(ctx + ": need ratio_low > ratio_high > 0");
    if (!(c.ratio_rate_per_s > real(0.0)) || !(c.step_rate_per_s > real(0.0))) {
        throw std::runtime_error(ctx + ": ratio_rate_per_s and step_rate_per_s must be > 0");
    }
    if (c.pump_drag_nm < real(0.0) || c.pump_drag_nm_per_kn < real(0.0)) throw std::runtime_error(ctx + ": pump drag terms must be >= 0");
    if (!o.at("steps").is_array()) throw std::runtime_error(ctx + ".steps: must be an array");
    for (const auto& v : o.at("steps")) {
        if (!v.is_number() || !std::isfinite(v.get<real>())) throw std::runtime_error(ctx + ".steps: every step must be a finite number");
        c.steps.push_back(v.get<real>());
    }
    if (c.steps.size() < 2) throw std::runtime_error(ctx + ".steps: needs at least two virtual steps");
    for (std::size_t i = 0; i < c.steps.size(); ++i) {
        if (c.steps[i] > c.ratio_low || c.steps[i] < c.ratio_high) throw std::runtime_error(ctx + ".steps: every step must lie within [ratio_high, ratio_low]");
        if (i > 0 && !(c.steps[i] < c.steps[i - 1])) throw std::runtime_error(ctx + ".steps: must be strictly decreasing (1st first)");
    }
    c.efficiency_vs_ratio = read_curve(o.at("efficiency_vs_ratio"));
    if (c.efficiency_vs_ratio.x.empty()) throw std::runtime_error(ctx + ".efficiency_vs_ratio: needs at least one point");
    require_ascending(c.efficiency_vs_ratio, ctx + ".efficiency_vs_ratio");
    for (real e : c.efficiency_vs_ratio.y) {
        if (!(e > real(0.0)) || e > real(1.0)) throw std::runtime_error(ctx + ".efficiency_vs_ratio: every efficiency must be in (0, 1]");
    }
    const json& b = o.at("belt");
    const std::string bctx = ctx + ".belt";
    check_keys(b, {"friction_coefficient", "sheave_half_angle_deg", "pulley_radius_sum_m", "clamp_force_min_n", "clamp_force_max_n",
                   "clamp_safety_factor", "clamp_time_constant_s", "static_factor"},
               bctx);
    ps::drivetrain::CvtBeltDesc& bd = c.belt;
    bd.friction_coefficient = num(b, "friction_coefficient", bd.friction_coefficient, bctx);
    bd.sheave_half_angle_deg = num(b, "sheave_half_angle_deg", bd.sheave_half_angle_deg, bctx);
    bd.pulley_radius_sum_m = num(b, "pulley_radius_sum_m", bd.pulley_radius_sum_m, bctx);
    bd.clamp_force_min_n = num(b, "clamp_force_min_n", bd.clamp_force_min_n, bctx);
    bd.clamp_force_max_n = num(b, "clamp_force_max_n", bd.clamp_force_max_n, bctx);
    bd.clamp_safety_factor = num(b, "clamp_safety_factor", bd.clamp_safety_factor, bctx);
    bd.clamp_time_constant_s = num(b, "clamp_time_constant_s", bd.clamp_time_constant_s, bctx);
    bd.static_factor = num(b, "static_factor", bd.static_factor, bctx);
    if (!(bd.friction_coefficient > real(0.0)) || !(bd.pulley_radius_sum_m > real(0.0)) || !(bd.sheave_half_angle_deg > real(0.0)) ||
        !(bd.sheave_half_angle_deg < real(45.0))) {
        throw std::runtime_error(bctx + ": need friction_coefficient > 0, pulley_radius_sum_m > 0 and sheave_half_angle_deg in (0, 45)");
    }
    if (!(bd.clamp_force_min_n > real(0.0)) || !(bd.clamp_force_max_n >= bd.clamp_force_min_n)) {
        throw std::runtime_error(bctx + ": need 0 < clamp_force_min_n <= clamp_force_max_n");
    }
    if (bd.clamp_safety_factor < real(1.0) || bd.static_factor < real(1.0) || !(bd.clamp_time_constant_s > real(0.0))) {
        throw std::runtime_error(bctx + ": clamp_safety_factor and static_factor must be >= 1, clamp_time_constant_s > 0");
    }
    return c;
}

ps::drivetrain::GearboxDesc read_gearbox_fields(const json& j, const std::string& ctx) {
    ps::drivetrain::GearboxDesc g;
    const std::string type = j.value("type", std::string("manual"));
    if (type == "manual") g.type = ps::drivetrain::GearboxType::Manual;
    else if (type == "sequential") g.type = ps::drivetrain::GearboxType::Sequential;
    else if (type == "automatic") g.type = ps::drivetrain::GearboxType::Automatic;
    else if (type == "dual_clutch") g.type = ps::drivetrain::GearboxType::DualClutch;
    else if (type == "cvt") g.type = ps::drivetrain::GearboxType::Cvt;
    else {
        throw std::runtime_error(ctx + ": gearbox 'type' must be \"manual\", \"sequential\", \"automatic\", \"dual_clutch\" or \"cvt\", got \"" +
                                 type + "\"");
    }

    // CVT plan C1: the variator block. Its virtual steps become forward_ratios and their efficiency the variator's
    // efficiency at that ratio, so the file carries neither array.
    if (g.type == ps::drivetrain::GearboxType::Cvt) {
        if (j.contains("forward_ratios") || j.contains("forward_efficiency")) {
            throw std::runtime_error(ctx + ": a cvt gearbox has no 'forward_ratios'/'forward_efficiency' (the variator's cvt.steps and "
                                           "cvt.efficiency_vs_ratio replace them)");
        }
        if (!j.contains("cvt")) throw std::runtime_error(ctx + ": 'cvt' is required for a cvt gearbox");
        g.cvt = read_cvt_block(j.at("cvt"), ctx + ".cvt");
        g.forward_ratios = g.cvt->steps;
        for (real r : g.cvt->steps) g.forward_efficiency.push_back(g.cvt->efficiency_vs_ratio.evaluate(r));
    } else if (j.contains("cvt")) {
        throw std::runtime_error(ctx + ": 'cvt' is for \"cvt\" gearboxes only");
    }

    if (g.type != ps::drivetrain::GearboxType::Cvt && !j.contains("forward_ratios")) throw std::runtime_error(ctx + ": 'forward_ratios' is required");
    if (g.type != ps::drivetrain::GearboxType::Cvt) for (const auto& v : j.at("forward_ratios")) g.forward_ratios.push_back(v.get<real>());
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

    if (g.type != ps::drivetrain::GearboxType::Cvt) {
        if (!j.contains("forward_efficiency")) throw std::runtime_error(ctx + ": 'forward_efficiency' is required");
        for (const auto& v : j.at("forward_efficiency")) g.forward_efficiency.push_back(v.get<real>());
    }
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

    // Automatic gearbox plan A1: the shift clutches (Automatic, DualClutch) and the torque converter (Automatic).
    {
        const bool c2c = ps::drivetrain::is_clutch_to_clutch(g.type);
        if (!c2c && (j.contains("shift_clutch") || j.contains("converter"))) {
            throw std::runtime_error(ctx + ": 'shift_clutch' and 'converter' are for \"automatic\", \"dual_clutch\" and \"cvt\" gearboxes only");
        }
        if (c2c) {
            if (j.contains("synchro")) throw std::runtime_error(ctx + ": a clutch-to-clutch gearbox has no 'synchro' block");
            if (!j.contains("shift_clutch")) throw std::runtime_error(ctx + ": 'shift_clutch' is required for this gearbox type");
            const json& sj = j.at("shift_clutch");
            if (!sj.is_object()) throw std::runtime_error(ctx + ".shift_clutch: must be an object");
            static const char* const kShiftKeys[] = {"capacity_nm",   "static_factor", "torque_phase_s",  "max_inertia_phase_s",
                                                    "sync_slip_rad_s", "engage_time_s", "preselect_time_s"};
            for (auto it = sj.begin(); it != sj.end(); ++it) {
                if (std::find_if(std::begin(kShiftKeys), std::end(kShiftKeys), [&](const char* k) { return it.key() == k; }) ==
                    std::end(kShiftKeys)) {
                    throw std::runtime_error(ctx + ".shift_clutch: unknown key '" + it.key() + "'");
                }
            }
            auto num = [&](const json& o, const char* key, real def, const std::string& where) {
                if (!o.contains(key)) return def;
                const json& v = o.at(key);
                if (!v.is_number() || !std::isfinite(v.get<real>())) throw std::runtime_error(where + ": '" + key + "' must be a finite number");
                return v.get<real>();
            };
            ps::drivetrain::ShiftClutchDesc& sc = g.shift_clutch;
            const std::string sctx = ctx + ".shift_clutch";
            if (!sj.contains("capacity_nm")) throw std::runtime_error(sctx + ": 'capacity_nm' is required");
            sc.capacity_nm = num(sj, "capacity_nm", sc.capacity_nm, sctx);
            sc.static_factor = num(sj, "static_factor", sc.static_factor, sctx);
            sc.torque_phase_s = num(sj, "torque_phase_s", sc.torque_phase_s, sctx);
            sc.max_inertia_phase_s = num(sj, "max_inertia_phase_s", sc.max_inertia_phase_s, sctx);
            sc.sync_slip_rad_s = num(sj, "sync_slip_rad_s", sc.sync_slip_rad_s, sctx);
            sc.engage_time_s = num(sj, "engage_time_s", sc.engage_time_s, sctx);
            sc.preselect_time_s = num(sj, "preselect_time_s", sc.preselect_time_s, sctx);
            if (!(sc.capacity_nm > real(0.0))) throw std::runtime_error(sctx + ": capacity_nm must be > 0");
            if (sc.static_factor < real(1.0)) throw std::runtime_error(sctx + ": static_factor must be >= 1");
            if (!(sc.torque_phase_s > real(0.0)) || !(sc.max_inertia_phase_s > real(0.0)) || !(sc.sync_slip_rad_s > real(0.0)) ||
                !(sc.engage_time_s > real(0.0))) {
                throw std::runtime_error(sctx + ": torque_phase_s, max_inertia_phase_s, sync_slip_rad_s and engage_time_s must be > 0");
            }
            if (sc.preselect_time_s < real(0.0)) throw std::runtime_error(sctx + ": preselect_time_s must be >= 0");
            if (g.type == ps::drivetrain::GearboxType::DualClutch && j.contains("converter")) {
                throw std::runtime_error(ctx + ": a dual_clutch gearbox has no 'converter'");
            }
            const bool is_cvt = g.type == ps::drivetrain::GearboxType::Cvt;
            if (is_cvt) {
                if (sc.preselect_time_s != real(0.0)) throw std::runtime_error(sctx + ": preselect_time_s is for dual_clutch gearboxes only");
                if (g.reverse_ratios.size() != 1) throw std::runtime_error(ctx + ": a cvt gearbox has exactly one reverse ratio");
            }
            if (g.type == ps::drivetrain::GearboxType::Automatic || (is_cvt && j.contains("converter"))) {
                if (sc.preselect_time_s != real(0.0)) throw std::runtime_error(sctx + ": preselect_time_s is for dual_clutch gearboxes only");
                if (!j.contains("converter")) throw std::runtime_error(ctx + ": 'converter' is required for an automatic gearbox");
                const json& cj = j.at("converter");
                const std::string cctx = ctx + ".converter";
                if (!cj.is_object()) throw std::runtime_error(cctx + ": must be an object");
                static const char* const kConvKeys[] = {"k_factor",           "torque_ratio",         "pump_inertia_kgm2",
                                                       "turbine_inertia_kgm2", "lockup_capacity_nm",   "lockup_static_factor",
                                                       "lockup_engage_time_s", "lockup_release_time_s"};
                for (auto it = cj.begin(); it != cj.end(); ++it) {
                    if (std::find_if(std::begin(kConvKeys), std::end(kConvKeys), [&](const char* k) { return it.key() == k; }) ==
                        std::end(kConvKeys)) {
                        throw std::runtime_error(cctx + ": unknown key '" + it.key() + "'");
                    }
                }
                ps::drivetrain::TorqueConverterDesc tc;
                if (!cj.contains("k_factor") || !cj.contains("torque_ratio")) {
                    throw std::runtime_error(cctx + ": 'k_factor' and 'torque_ratio' are required");
                }
                tc.k_factor = read_curve(cj.at("k_factor"));
                tc.torque_ratio = read_curve(cj.at("torque_ratio"));
                require_ascending(tc.k_factor, cctx + ".k_factor");
                require_ascending(tc.torque_ratio, cctx + ".torque_ratio");
                if (tc.k_factor.x.front() != real(0.0) || !(tc.k_factor.x.back() < real(1.0))) {
                    throw std::runtime_error(cctx + ".k_factor: x must start at 0 and stay below 1");
                }
                for (real k : tc.k_factor.y) {
                    if (!(k > real(0.0))) throw std::runtime_error(cctx + ".k_factor: every K must be > 0");
                }
                if (tc.torque_ratio.x.front() != real(0.0) || tc.torque_ratio.x.back() > real(1.0)) {
                    throw std::runtime_error(cctx + ".torque_ratio: x must start at 0 and stay within [0, 1]");
                }
                for (std::size_t i = 0; i < tc.torque_ratio.x.size(); ++i) {
                    const real tr = tc.torque_ratio.y[i];
                    if (tr < real(1.0)) throw std::runtime_error(cctx + ".torque_ratio: every ratio must be >= 1");
                    if (tr * tc.torque_ratio.x[i] > real(1.0) + real(1e-12)) {
                        throw std::runtime_error(cctx + ".torque_ratio: TR(SR) * SR must be <= 1 (no efficiency above 1)");
                    }
                    if (i > 0 && tr > tc.torque_ratio.y[i - 1]) {
                        throw std::runtime_error(cctx + ".torque_ratio: must be non-increasing");
                    }
                }
                tc.pump_inertia_kgm2 = num(cj, "pump_inertia_kgm2", tc.pump_inertia_kgm2, cctx);
                tc.turbine_inertia_kgm2 = num(cj, "turbine_inertia_kgm2", tc.turbine_inertia_kgm2, cctx);
                tc.lockup_capacity_nm = num(cj, "lockup_capacity_nm", tc.lockup_capacity_nm, cctx);
                tc.lockup_static_factor = num(cj, "lockup_static_factor", tc.lockup_static_factor, cctx);
                tc.lockup_engage_time_s = num(cj, "lockup_engage_time_s", tc.lockup_engage_time_s, cctx);
                tc.lockup_release_time_s = num(cj, "lockup_release_time_s", tc.lockup_release_time_s, cctx);
                if (!(tc.pump_inertia_kgm2 > real(0.0)) || !(tc.turbine_inertia_kgm2 + g.input_inertia_kgm2 > real(0.0))) {
                    throw std::runtime_error(cctx + ": pump_inertia_kgm2 must be > 0 and turbine_inertia_kgm2 + input_inertia_kgm2 > 0");
                }
                if (tc.turbine_inertia_kgm2 < real(0.0)) throw std::runtime_error(cctx + ": turbine_inertia_kgm2 must be >= 0");
                if (!(tc.lockup_capacity_nm > real(0.0))) throw std::runtime_error(cctx + ": lockup_capacity_nm must be > 0");
                if (tc.lockup_static_factor < real(1.0)) throw std::runtime_error(cctx + ": lockup_static_factor must be >= 1");
                if (!(tc.lockup_engage_time_s > real(0.0)) || !(tc.lockup_release_time_s > real(0.0))) {
                    throw std::runtime_error(cctx + ": lockup_engage_time_s and lockup_release_time_s must be > 0");
                }
                g.converter = std::move(tc);
            }
            if (g.initial_gear != 0) throw std::runtime_error(ctx + ": a clutch-to-clutch gearbox starts in N (initial_gear 0)");
        }
    }

    // Transmission plan T0: rating and sound data (never hashed, in no key).
    if (j.contains("rated_input_torque_nm")) {
        const json& rj = j.at("rated_input_torque_nm");
        if (!rj.is_number() || !std::isfinite(rj.get<real>()) || !(rj.get<real>() > real(0.0))) {
            throw std::runtime_error(ctx + ": rated_input_torque_nm must be a number > 0");
        }
        g.rated_input_torque_nm = rj.get<real>();
    }
    if (j.contains("gear_type")) {
        const json& tj = j.at("gear_type");
        const std::string gt = tj.is_string() ? tj.get<std::string>() : std::string();
        if (gt == "helical") g.gear_type = ps::drivetrain::GearType::Helical;
        else if (gt == "straight_cut") g.gear_type = ps::drivetrain::GearType::StraightCut;
        else throw std::runtime_error(ctx + ": gear_type must be \"helical\" or \"straight_cut\"");
    }
    if (j.contains("mesh_order")) {
        const json& mj = j.at("mesh_order");
        if (!mj.is_number() || !std::isfinite(mj.get<real>()) || !(mj.get<real>() > real(0.0))) {
            throw std::runtime_error(ctx + ": mesh_order must be a number > 0");
        }
        g.mesh_order = mj.get<real>();
    }
    if (j.contains("teeth")) {
        const json& tj = j.at("teeth");
        if (!tj.is_object()) throw std::runtime_error(ctx + ": teeth must be an object {forward, reverse, input_pair}");
        for (auto it = tj.begin(); it != tj.end(); ++it) {
            if (it.key() != "forward" && it.key() != "reverse" && it.key() != "input_pair") {
                throw std::runtime_error(ctx + ".teeth: unknown key '" + it.key() + "' (expected forward, reverse, input_pair)");
            }
        }
        auto read_pair = [&](const json& pj, const std::string& where) {
            ps::drivetrain::GearTeeth t;
            if (!pj.is_object() || !pj.contains("drive") || !pj.contains("driven")) {
                throw std::runtime_error(ctx + "." + where + ": a tooth pair needs {\"drive\", \"driven\"}");
            }
            for (auto it = pj.begin(); it != pj.end(); ++it) {
                if (it.key() != "drive" && it.key() != "driven") {
                    throw std::runtime_error(ctx + "." + where + ": unknown key '" + it.key() + "' (expected drive, driven)");
                }
            }
            const json& dj = pj.at("drive");
            const json& nj = pj.at("driven");
            if (!dj.is_number_integer() || !nj.is_number_integer() || dj.get<long long>() < 1 || nj.get<long long>() < 1) {
                throw std::runtime_error(ctx + "." + where + ": tooth counts must be integers >= 1");
            }
            t.drive = dj.get<int>();
            t.driven = nj.get<int>();
            return t;
        };
        auto read_list = [&](const char* key, std::vector<ps::drivetrain::GearTeeth>& out, std::size_t expected) {
            if (!tj.contains(key)) return;
            const json& lj = tj.at(key);
            if (!lj.is_array()) throw std::runtime_error(ctx + ".teeth." + key + " must be an array");
            if (lj.size() != expected) {
                throw std::runtime_error(ctx + ": teeth." + key + " must have the same length as " + key + "_ratios");
            }
            for (std::size_t i = 0; i < lj.size(); ++i) {
                out.push_back(read_pair(lj[i], std::string("teeth.") + key + "[" + std::to_string(i) + "]"));
            }
        };
        read_list("forward", g.forward_teeth, g.forward_ratios.size());
        read_list("reverse", g.reverse_teeth, g.reverse_ratios.size());
        if (tj.contains("input_pair")) g.input_pair_teeth = read_pair(tj.at("input_pair"), "teeth.input_pair");
    }
    return g;
}

// PHYS-033 I3 plan section 9.1: {"id", "type": "engine"|"gearbox", "ref":
// "<relative path>"} - a ref'd object may contain ONLY id/type/ref (no
// merging with inline fields). Returns the resolved json object to read
// fields from (either the external file's own top-level object, format-
// checked, or `cj` itself for an inline component).
json resolve_component_ref(const json& cj, const std::string& kind, const std::string& base_dir, const std::string& ctx) {
    if (!cj.contains("ref")) {
        if (kind == "gearbox" && cj.contains("extends")) {
            throw std::runtime_error(ctx + ": 'extends' is only available in a gearbox part file (referenced with 'ref')");
        }
        return cj;
    }
    for (auto it = cj.begin(); it != cj.end(); ++it) {
        // Engine garage P2: an engine component may carry its slots (ecu, cam, forced_induction, installation,
        // thermal - the T1 "thermal" and N7b override, renamed forced_induction, among them) next to "ref".
        const bool engine_slot = kind == "engine" && parts::is_engine_slot(it.key());
        if (it.key() != "id" && it.key() != "type" && it.key() != "ref" && !engine_slot) {
            throw std::runtime_error(ctx + ": a component with 'ref' may only contain 'id', 'type' and 'ref'" +
                                     std::string(kind == "engine" ? " (and the engine slots ecu, cam, forced_induction, installation, thermal)"
                                                                  : "") +
                                     " (found extra field '" + it.key() + "')");
        }
    }
    const std::string path = resolve_path(base_dir, cj.at("ref").get<std::string>());
    // Engine garage P2 (design A4): an engine file may "extends" a parent engine; relative paths stay per file.
    // Transmission plan T0: a gearbox file may "extends" a parent gearbox (it has no path-valued key to rebase).
    const json fj = kind == "engine"    ? parts::load_part(path, parts::rebase_engine_paths)
                    : kind == "gearbox" ? parts::load_part(path)
                                        : read_json_file(path);
    require_format(fj, kind, 1, path);
    return fj;
}

// Engine garage P2 (design A6): applies a user-layer PartOverride to a component's JSON. "ref" replaces the
// referenced file (an inline engine/gearbox becomes {id, type, ref} plus the engine's string-valued slots); each
// slot sets the component's own slot key. Paths keep the vehicle-level rule (relative to the vehicle file unless
// absolute). Types: "ref" for engine, gearbox, n2o_bottle, nitrous_kit; slots for engine (ecu, cam,
// forced_induction, installation, thermal) and nitrous_kit (controller).
json apply_part_override(const json& cj, const std::string& type, const PartOverride& ov, const std::string& ctx) {
    const std::string octx = ctx + ": part_overrides";
    const bool ref_type = type == "engine" || type == "gearbox" || type == "n2o_bottle" || type == "nitrous_kit";
    if (!ref_type) throw std::runtime_error(octx + ": a '" + type + "' component has no part file to override");
    json out = cj;
    if (!ov.ref.empty()) {
        if (!cj.contains("ref")) {
            json replaced = json::object();
            replaced["id"] = cj.at("id");
            replaced["type"] = cj.at("type");
            if (type == "engine") {
                for (const char* s : parts::kEngineSlots) {
                    if (cj.contains(s) && cj.at(s).is_string()) replaced[s] = cj.at(s);
                }
            }
            out = std::move(replaced);
        }
        out["ref"] = ov.ref;
    }
    for (const auto& kv : ov.slots) {
        const bool ok = type == "engine" ? parts::is_engine_slot(kv.first) : (type == "nitrous_kit" && kv.first == "controller");
        if (!ok) {
            throw std::runtime_error(octx + ": unknown slot '" + kv.first + "' for a '" + type + "' component" +
                                     (type == "engine" ? std::string(" (engine slots: ecu, cam, forced_induction, installation, thermal)")
                                      : type == "nitrous_kit" ? std::string(" (nitrous kit slot: controller)")
                                                              : std::string(" (it has no slots)")));
        }
        if (kv.second.empty()) throw std::runtime_error(octx + ": slot '" + kv.first + "' must be a non-empty path");
        if (type == "engine" && !out.contains("ref") && kv.first == "ecu" && out.contains("ecu") && out.at("ecu").is_object()) {
            throw std::runtime_error(octx + ": slot 'ecu' on an inline engine component whose 'ecu' block is inline - give the "
                                     "user layer a 'ref' too, or move the engine to a file");
        }
        out[kv.first] = kv.second;
    }
    return out;
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

// Automatic gearbox plan A1: an "automatic_tcu" controller (docs/data_formats/vehicle.md). Unknown keys rejected.
ps::drivetrain::AutomaticTcuDesc read_automatic_tcu(const json& cj, const std::string& ctx, const std::vector<std::string>& component_ids) {
    static const char* const kKeys[] = {"id",          "type",           "gearbox",  "engine",         "schedule",       "sport_schedule",
                                        "min_time_in_gear_s", "kickdown_throttle", "kickdown_max_rpm", "manual_timeout_s", "manual_up_rpm",
                                        "upshift_torque_fraction", "lockup", "launch", "creep_torque_nm", "cvt"};
    for (auto it = cj.begin(); it != cj.end(); ++it) {
        if (std::find_if(std::begin(kKeys), std::end(kKeys), [&](const char* k) { return it.key() == k; }) == std::end(kKeys)) {
            throw std::runtime_error(ctx + ": unknown key '" + it.key() + "'");
        }
    }
    auto num = [](const json& o, const char* key, real def, const std::string& where) {
        if (!o.contains(key)) return def;
        const json& v = o.at(key);
        if (!v.is_number() || !std::isfinite(v.get<real>())) throw std::runtime_error(where + ": '" + key + "' must be a finite number");
        return v.get<real>();
    };
    auto check_keys = [](const json& o, std::initializer_list<const char*> keys, const std::string& where) {
        if (!o.is_object()) throw std::runtime_error(where + ": must be an object");
        for (auto it = o.begin(); it != o.end(); ++it) {
            bool ok = false;
            for (const char* k : keys) ok = ok || it.key() == k;
            if (!ok) throw std::runtime_error(where + ": unknown key '" + it.key() + "'");
        }
    };
    ps::drivetrain::AutomaticTcuDesc d;
    if (!cj.contains("gearbox") || !cj.contains("engine") || !cj.at("gearbox").is_string() || !cj.at("engine").is_string()) {
        throw std::runtime_error(ctx + ": 'gearbox' and 'engine' (component id references) are required");
    }
    d.gearbox = cj.at("gearbox").get<std::string>();
    d.engine = cj.at("engine").get<std::string>();
    for (const std::string* ref : {&d.gearbox, &d.engine}) {
        if (std::find(component_ids.begin(), component_ids.end(), *ref) == component_ids.end()) {
            throw std::runtime_error(ctx + ": references unknown component '" + *ref + "'");
        }
    }
    auto read_schedule = [&](const char* key, ps::drivetrain::ShiftScheduleDesc& sch) {
        if (!cj.contains(key)) return;
        const std::string w = ctx + "." + key;
        const json& o = cj.at(key);
        check_keys(o, {"up_rpm_at_zero_throttle", "up_rpm_at_full_throttle", "down_rpm_at_zero_throttle", "down_rpm_at_full_throttle"}, w);
        sch.up_rpm_at_zero_throttle = num(o, "up_rpm_at_zero_throttle", sch.up_rpm_at_zero_throttle, w);
        sch.up_rpm_at_full_throttle = num(o, "up_rpm_at_full_throttle", sch.up_rpm_at_full_throttle, w);
        sch.down_rpm_at_zero_throttle = num(o, "down_rpm_at_zero_throttle", sch.down_rpm_at_zero_throttle, w);
        sch.down_rpm_at_full_throttle = num(o, "down_rpm_at_full_throttle", sch.down_rpm_at_full_throttle, w);
        if (!(sch.down_rpm_at_zero_throttle > real(0.0)) || !(sch.down_rpm_at_zero_throttle < sch.up_rpm_at_zero_throttle) ||
            !(sch.down_rpm_at_full_throttle < sch.up_rpm_at_full_throttle) || !(sch.down_rpm_at_full_throttle > real(0.0))) {
            throw std::runtime_error(w + ": every downshift rpm must be > 0 and below the upshift rpm at the same throttle");
        }
    };
    read_schedule("schedule", d.schedule);
    if (cj.contains("sport_schedule")) {
        read_schedule("sport_schedule", d.sport_schedule);
    } else {
        d.sport_schedule = d.schedule;
    }
    d.min_time_in_gear_s = num(cj, "min_time_in_gear_s", d.min_time_in_gear_s, ctx);
    d.kickdown_throttle = num(cj, "kickdown_throttle", d.kickdown_throttle, ctx);
    d.kickdown_max_rpm = num(cj, "kickdown_max_rpm", d.kickdown_max_rpm, ctx);
    d.manual_timeout_s = num(cj, "manual_timeout_s", d.manual_timeout_s, ctx);
    d.manual_up_rpm = num(cj, "manual_up_rpm", d.manual_up_rpm, ctx);
    d.upshift_torque_fraction = num(cj, "upshift_torque_fraction", d.upshift_torque_fraction, ctx);
    d.creep_torque_nm = num(cj, "creep_torque_nm", d.creep_torque_nm, ctx);
    if (d.min_time_in_gear_s < real(0.0) || d.manual_timeout_s < real(0.0) || d.manual_up_rpm < real(0.0) || d.creep_torque_nm < real(0.0)) {
        throw std::runtime_error(ctx + ": min_time_in_gear_s, manual_timeout_s, manual_up_rpm and creep_torque_nm must be >= 0");
    }
    if (d.kickdown_throttle < real(0.0) || d.kickdown_throttle > real(1.0) || !(d.kickdown_max_rpm > real(0.0))) {
        throw std::runtime_error(ctx + ": kickdown_throttle must be in [0, 1] and kickdown_max_rpm > 0");
    }
    if (!(d.upshift_torque_fraction > real(0.0)) || d.upshift_torque_fraction > real(1.0)) {
        throw std::runtime_error(ctx + ": upshift_torque_fraction must be in (0, 1]");
    }
    if (cj.contains("lockup")) {
        const json& o = cj.at("lockup");
        const std::string w = ctx + ".lockup";
        check_keys(o, {"enabled", "min_gear", "engage_rpm", "release_rpm", "max_throttle"}, w);
        if (o.contains("enabled") && !o.at("enabled").is_boolean()) throw std::runtime_error(w + ": 'enabled' must be a boolean");
        d.lockup_enabled = o.value("enabled", d.lockup_enabled);
        if (o.contains("min_gear") && !o.at("min_gear").is_number_integer()) throw std::runtime_error(w + ": 'min_gear' must be an integer");
        d.lockup_min_gear = o.value("min_gear", d.lockup_min_gear);
        d.lockup_engage_rpm = num(o, "engage_rpm", d.lockup_engage_rpm, w);
        d.lockup_release_rpm = num(o, "release_rpm", d.lockup_release_rpm, w);
        d.lockup_max_throttle = num(o, "max_throttle", d.lockup_max_throttle, w);
        if (d.lockup_min_gear < 1 || !(d.lockup_release_rpm > real(0.0)) || !(d.lockup_release_rpm < d.lockup_engage_rpm) ||
            d.lockup_max_throttle < real(0.0) || d.lockup_max_throttle > real(1.0)) {
            throw std::runtime_error(w + ": min_gear >= 1, 0 < release_rpm < engage_rpm and max_throttle in [0, 1] required");
        }
    }
    if (cj.contains("launch")) {
        const json& o = cj.at("launch");
        const std::string w = ctx + ".launch";
        check_keys(o, {"bite_rpm", "full_rpm_min", "full_rpm_max", "lock_slip_rpm", "anti_stall_rpm"}, w);
        d.launch_bite_rpm = num(o, "bite_rpm", d.launch_bite_rpm, w);
        d.launch_full_rpm_min = num(o, "full_rpm_min", d.launch_full_rpm_min, w);
        d.launch_full_rpm_max = num(o, "full_rpm_max", d.launch_full_rpm_max, w);
        d.launch_lock_slip_rpm = num(o, "lock_slip_rpm", d.launch_lock_slip_rpm, w);
        d.launch_anti_stall_rpm = num(o, "anti_stall_rpm", d.launch_anti_stall_rpm, w);
        if (!(d.launch_anti_stall_rpm > real(0.0)) || !(d.launch_bite_rpm > real(0.0)) || !(d.launch_full_rpm_min > d.launch_bite_rpm) ||
            d.launch_full_rpm_max < d.launch_full_rpm_min || !(d.launch_lock_slip_rpm > real(0.0))) {
            throw std::runtime_error(w + ": need anti_stall_rpm > 0, 0 < bite_rpm < full_rpm_min <= full_rpm_max and lock_slip_rpm > 0");
        }
    }
    if (cj.contains("cvt")) {
        // CVT plan C1: the ratio law of a cvt gearbox (CvtTcuDesc). Whether the gearbox IS a cvt is checked when the
        // powertrain is built (the controller references the gearbox component).
        const json& o = cj.at("cvt");
        const std::string w = ctx + ".cvt";
        check_keys(o, {"target_rpm_vs_throttle", "sport_target_rpm_vs_throttle", "min_rpm_vs_output_rpm", "stepped_sport"}, w);
        ps::drivetrain::CvtTcuDesc c;
        if (!o.contains("target_rpm_vs_throttle")) throw std::runtime_error(w + ": 'target_rpm_vs_throttle' is required");
        auto rpm_curve = [&](const char* key, bool throttle_x) {
            Curve1D cv = read_curve(o.at(key));
            require_ascending(cv, w + "." + key);
            for (real y : cv.y) {
                if (!(y > real(0.0)) || !std::isfinite(y)) throw std::runtime_error(w + "." + key + ": every rpm must be > 0");
            }
            if (throttle_x && (cv.x.front() < real(0.0) || cv.x.back() > real(1.0))) {
                throw std::runtime_error(w + "." + key + ": throttle x must lie within [0, 1]");
            }
            if (!throttle_x && cv.x.front() < real(0.0)) throw std::runtime_error(w + "." + key + ": output rpm x must be >= 0");
            return cv;
        };
        c.target_rpm_vs_throttle = rpm_curve("target_rpm_vs_throttle", true);
        if (o.contains("sport_target_rpm_vs_throttle")) c.sport_target_rpm_vs_throttle = rpm_curve("sport_target_rpm_vs_throttle", true);
        if (o.contains("min_rpm_vs_output_rpm")) c.min_rpm_vs_output_rpm = rpm_curve("min_rpm_vs_output_rpm", false);
        if (o.contains("stepped_sport")) {
            if (!o.at("stepped_sport").is_boolean()) throw std::runtime_error(w + ": 'stepped_sport' must be a boolean");
            c.stepped_sport = o.at("stepped_sport").get<bool>();
        }
        d.cvt = std::move(c);
    }
    return d;
}

} // namespace

drivetrain::TurboPairDesc load_turbo_configuration_json(const std::string& path, const std::string& cache_dir) {
    // Engine garage P2 (design A4): a configuration and its hardware may "extends" a parent (relative paths per file).
    const json cfg=parts::load_part(path,parts::rebase_turbo_configuration_paths);
    require_format(cfg,"turbo_configuration",1,path);
    const int count=require_positive_int(cfg,"count",path);
    if (count>8) throw std::runtime_error(path+": turbo count must be <= 8");
    const auto hw_path=resolve_path(std::filesystem::path(path).parent_path().string(),cfg.at("turbo").get<std::string>());
    const auto hw=parts::load_part(hw_path,parts::rebase_turbo_hardware_paths);
    require_format(hw,"turbo",1,hw_path);
    drivetrain::TurboPairDesc t;
    if(cfg.contains("compressible_exhaust")) {
        if(!cfg.at("compressible_exhaust").is_boolean())
            throw std::runtime_error(path+": compressible_exhaust must be boolean");
        t.compressible_exhaust=cfg.at("compressible_exhaust").get<bool>();
    }
    t.rotor_inertia_kgm2=require_positive(hw,"rotor_inertia_kgm2",hw_path)*count;
    t.reference_rpm=require_positive(hw,"reference_rpm",hw_path);
    // Pressures in turbo files are bar (owner rule 2026-10-02: files and
    // readouts in bar); the desc stays SI. The old Pa keys are rejected by name.
    if(hw.contains("reference_head_pa")) throw std::runtime_error(hw_path+": 'reference_head_pa' was replaced by 'reference_head_bar' (bar)");
    t.reference_head_pa=require_positive(hw,"reference_head_bar",hw_path)*real(1e5);
    t.max_mass_flow_kg_s=require_positive(hw,"max_mass_flow_kg_s",hw_path)*count;
    t.compressor_efficiency=require_positive(hw,"compressor_efficiency",hw_path);
    t.turbine_efficiency=require_positive(hw,"turbine_efficiency",hw_path);
    t.turbine_area_m2=require_positive(hw,"turbine_area_m2",hw_path)*count;
    // Optional effective wastegate port area per turbo (absent = the desc's [AS] ratio to the
    // turbine area). Feeds no map: outside the cache key.
    if(hw.contains("wastegate_area_m2")) t.wastegate_area_m2=require_positive(hw,"wastegate_area_m2",hw_path)*count;
    t.shaft_loss_w_at_reference=require_positive(hw,"shaft_loss_w_at_reference",hw_path)*count;
    // I7d D3: optional surge line of the hardware file (flag only; feeds no map and
    // no dynamics, so it is outside the cache key and the state hash).
    if(hw.contains("surge_line")) {
        const json& sl=hw.at("surge_line");
        if(!sl.is_object()) throw std::runtime_error(hw_path+": 'surge_line' must be an object");
        const std::string sctx=hw_path+": surge_line";
        const auto number_array=[&](const char* key) {
            if(!sl.contains(key) || !sl.at(key).is_array()) throw std::runtime_error(sctx+": '"+std::string(key)+"' must be an array of numbers");
            std::vector<real> v;
            for(const auto& e:sl.at(key)) {
                if(!e.is_number()) throw std::runtime_error(sctx+": '"+std::string(key)+"' must be an array of numbers");
                v.push_back(e.get<real>());
            }
            return v;
        };
        drivetrain::SurgeLineDesc d;
        d.flow_fraction_vs_pr.x=number_array("pressure_ratio");
        d.flow_fraction_vs_pr.y=number_array("flow_fraction");
        if(d.flow_fraction_vs_pr.x.size()<2) throw std::runtime_error(sctx+": needs at least 2 points");
        if(d.flow_fraction_vs_pr.x.size()!=d.flow_fraction_vs_pr.y.size())
            throw std::runtime_error(sctx+": 'pressure_ratio' and 'flow_fraction' must have the same length");
        if(!(d.flow_fraction_vs_pr.x.front()>=1.0)) throw std::runtime_error(sctx+": 'pressure_ratio' must start at >= 1");
        for(std::size_t i=1;i<d.flow_fraction_vs_pr.x.size();++i)
            if(!(d.flow_fraction_vs_pr.x[i]>d.flow_fraction_vs_pr.x[i-1])) throw std::runtime_error(sctx+": 'pressure_ratio' must be strictly increasing");
        for(real f:d.flow_fraction_vs_pr.y)
            if(!(f>=0.0 && f<1.0)) throw std::runtime_error(sctx+": 'flow_fraction' must be in [0, 1)");
        if(!sl.contains("source") || !sl.at("source").is_string() || sl.at("source").get<std::string>().empty())
            throw std::runtime_error(sctx+": 'source' is required (PLAN.md section 8)");
        d.source=sl.at("source").get<std::string>();
        t.surge_line=std::move(d);
    }
    if(cfg.contains("target_boost_pa")) throw std::runtime_error(path+": 'target_boost_pa' was replaced by 'target_boost_bar' (gauge, bar)");
    t.target_boost_pa=require_positive(cfg,"target_boost_bar",path)*real(1e5);
    t.intercooler_effectiveness=require_positive(cfg,"intercooler_effectiveness",path);
    if(t.compressor_efficiency>1 || t.turbine_efficiency>1 || t.intercooler_effectiveness>1)
        throw std::runtime_error(path+": turbo efficiencies must be <= 1");
    // I7d D4: optional intercooler pressure drop at the pair's max flow (bar in the file; intensive,
    // not scaled by count). Not part of the map cache key (it feeds no map).
    if(cfg.contains("intercooler_pressure_drop_bar_at_max_flow")) {
        const real dp_bar=require_number(cfg,"intercooler_pressure_drop_bar_at_max_flow",path);
        if(dp_bar<0) throw std::runtime_error(path+": 'intercooler_pressure_drop_bar_at_max_flow' must be >= 0");
        if(!(dp_bar*real(1e5)<t.reference_head_pa)) throw std::runtime_error(path+": 'intercooler_pressure_drop_bar_at_max_flow' must be < the hardware 'reference_head_bar'");
        t.intercooler_dp_pa_at_max_flow=dp_bar*real(1e5);
    }
    if(cfg.contains("boost_control")) {
        const json& bc=cfg.at("boost_control");
        if(!bc.is_object()) throw std::runtime_error(path+": 'boost_control' must be an object");
        const std::string bctx=path+": boost_control";
        if(bc.contains("kp")) t.wastegate_kp=require_positive(bc,"kp",bctx);
        if(bc.contains("ki_per_s")) t.wastegate_ki_per_s=require_positive(bc,"ki_per_s",bctx);
        if(bc.contains("kd_s")) { t.wastegate_kd_s=require_number(bc,"kd_s",bctx); if(t.wastegate_kd_s<0) throw std::runtime_error(bctx+": 'kd_s' must be >= 0"); }
        if(bc.contains("actuator_tau_s")) t.wastegate_actuator_tau_s=require_positive(bc,"actuator_tau_s",bctx);
    }
    // I7d D2: optional compressor bypass (blow-off) valve. Pressures in bar,
    // the per-turbo area is multiplied by the turbo count like the hardware's
    // extensive keys. Not part of the map cache key (it feeds no map).
    if(cfg.contains("blow_off_valve")) {
        const json& bv=cfg.at("blow_off_valve");
        if(!bv.is_object()) throw std::runtime_error(path+": 'blow_off_valve' must be an object");
        const std::string vctx=path+": blow_off_valve";
        if(bv.contains("crack_dp_pa")) throw std::runtime_error(vctx+": 'crack_dp_pa' was replaced by 'crack_dp_bar' (bar)");
        if(bv.contains("full_open_dp_pa")) throw std::runtime_error(vctx+": 'full_open_dp_pa' was replaced by 'full_open_dp_bar' (bar)");
        drivetrain::BlowOffValveDesc v;
        if(!bv.contains("vent") || !bv.at("vent").is_string()) throw std::runtime_error(vctx+": 'vent' must be \"atmosphere\" or \"recirculate\"");
        const std::string vent=bv.at("vent").get<std::string>();
        if(vent=="atmosphere") v.vent=drivetrain::BlowOffValveDesc::Vent::Atmosphere;
        else if(vent=="recirculate") v.vent=drivetrain::BlowOffValveDesc::Vent::Recirculate;
        else throw std::runtime_error(vctx+": 'vent' must be \"atmosphere\" or \"recirculate\", got \""+vent+"\"");
        const real crack_bar=require_number(bv,"crack_dp_bar",vctx);
        if(crack_bar<0) throw std::runtime_error(vctx+": 'crack_dp_bar' must be >= 0");
        const real full_bar=require_positive(bv,"full_open_dp_bar",vctx);
        if(!(full_bar>crack_bar)) throw std::runtime_error(vctx+": 'full_open_dp_bar' must be > 'crack_dp_bar'");
        v.crack_dp_pa=crack_bar*real(1e5);
        v.full_open_dp_pa=full_bar*real(1e5);
        // effective_area_mm2 is PER TURBO (Cd included); the pair is `count` valves.
        v.effective_area_m2=require_positive(bv,"effective_area_mm2",vctx)*real(1e-6)*count;
        v.tau_s=require_number(bv,"tau_s",vctx);
        if(v.tau_s<0) throw std::runtime_error(vctx+": 'tau_s' must be >= 0");
        if(!bv.contains("source") || !bv.at("source").is_string() || bv.at("source").get<std::string>().empty())
            throw std::runtime_error(vctx+": 'source' is required (PLAN.md section 8)");
        v.source=bv.at("source").get<std::string>();
        t.blow_off_valve=std::move(v);
    }
    // I7d D5: optional anti-lag (the ECU strategy serving the turbo). Not part of the map cache key (it feeds
    // no map). Unknown keys inside the block are rejected (a misspelt key would silently disable a limit).
    if(cfg.contains("anti_lag")) {
        const json& al=cfg.at("anti_lag");
        if(!al.is_object()) throw std::runtime_error(path+": 'anti_lag' must be an object");
        const std::string actx=path+": anti_lag";
        static const char* const kKeys[]={"channel","default_enabled","arm_below_command","min_rpm","max_rpm",
                                          "retard_deg","plate_floor","max_active_s","source","spark_advance_deg",
                                          "cylinder_cut_fraction","manifold_burn_fraction","lambda","boost_target_fraction"};
        for(auto it=al.begin();it!=al.end();++it)
            if(std::none_of(std::begin(kKeys),std::end(kKeys),[&](const char* k){ return it.key()==k; }))
                throw std::runtime_error(actx+": unknown key '"+it.key()+"'");
        drivetrain::AntiLagDesc a;
        if(al.contains("channel")) {
            if(!al.at("channel").is_string() || al.at("channel").get<std::string>().empty())
                throw std::runtime_error(actx+": 'channel' must be a non-empty string");
            a.channel=al.at("channel").get<std::string>();
        }
        if(al.contains("default_enabled")) {
            a.default_enabled=require_number(al,"default_enabled",actx);
            if(a.default_enabled!=real(0.0) && a.default_enabled!=real(1.0)) throw std::runtime_error(actx+": 'default_enabled' must be 0 or 1");
        }
        a.arm_below_command=require_number(al,"arm_below_command",actx);
        if(!(a.arm_below_command>0 && a.arm_below_command<=1)) throw std::runtime_error(actx+": 'arm_below_command' must be in (0, 1]");
        a.min_rpm=require_number(al,"min_rpm",actx);
        if(!(a.min_rpm>=0)) throw std::runtime_error(actx+": 'min_rpm' must be >= 0");
        a.max_rpm=require_number(al,"max_rpm",actx);
        if(!(a.max_rpm>a.min_rpm)) throw std::runtime_error(actx+": 'max_rpm' must be > 'min_rpm'");
        // Exactly one of retard_deg (from the scheduled advance) and spark_advance_deg (absolute).
        if(al.contains("retard_deg") && al.contains("spark_advance_deg"))
            throw std::runtime_error(actx+": give either 'retard_deg' or 'spark_advance_deg', not both");
        if(al.contains("spark_advance_deg")) {
            const real sa=require_number(al,"spark_advance_deg",actx);
            if(!(sa>=-90 && sa<=60)) throw std::runtime_error(actx+": 'spark_advance_deg' must be in [-90, 60]");
            a.spark_advance_deg=sa;
        } else {
            a.retard_deg=require_number(al,"retard_deg",actx);
            if(!(a.retard_deg>=0)) throw std::runtime_error(actx+": 'retard_deg' must be >= 0");
        }
        a.plate_floor=require_number(al,"plate_floor",actx);
        if(!(a.plate_floor>=0 && a.plate_floor<=1)) throw std::runtime_error(actx+": 'plate_floor' must be in [0, 1]");
        a.max_active_s=require_positive(al,"max_active_s",actx);
        if(al.contains("cylinder_cut_fraction")) {
            a.cylinder_cut_fraction=require_number(al,"cylinder_cut_fraction",actx);
            if(!(a.cylinder_cut_fraction>=0 && a.cylinder_cut_fraction<1)) throw std::runtime_error(actx+": 'cylinder_cut_fraction' must be in [0, 1)");
        }
        if(al.contains("manifold_burn_fraction")) {
            a.manifold_burn_fraction=require_number(al,"manifold_burn_fraction",actx);
            if(!(a.manifold_burn_fraction>=0 && a.manifold_burn_fraction<=1)) throw std::runtime_error(actx+": 'manifold_burn_fraction' must be in [0, 1]");
        }
        if(al.contains("lambda")) {
            a.lambda=require_number(al,"lambda",actx);
            if(!(a.lambda>=0.5 && a.lambda<=1.5)) throw std::runtime_error(actx+": 'lambda' must be in [0.5, 1.5]");
        }
        if(al.contains("boost_target_fraction")) {
            a.boost_target_fraction=require_number(al,"boost_target_fraction",actx);
            if(!(a.boost_target_fraction>=0 && a.boost_target_fraction<=1)) throw std::runtime_error(actx+": 'boost_target_fraction' must be in [0, 1]");
        }
        if(!al.contains("source") || !al.at("source").is_string() || al.at("source").get<std::string>().empty())
            throw std::runtime_error(actx+": 'source' is required (PLAN.md section 8)");
        a.source=al.at("source").get<std::string>();
        t.anti_lag=std::move(a);
    }
    // Exhaust energy balance and thermal mass (installation totals, not scaled by count). Optional:
    // absent keeps TurboPairDesc's [AS] defaults. Not part of the map cache key (it feeds no map).
    if(cfg.contains("exhaust_thermal")) {
        const json& et=cfg.at("exhaust_thermal");
        if(!et.is_object()) throw std::runtime_error(path+": 'exhaust_thermal' must be an object");
        const std::string ectx=path+": exhaust_thermal";
        static const char* const kKeys[]={"in_cylinder_wall_heat_fraction","wall_heat_capacity_j_k","wall_gas_conductance_w_k",
                                          "wall_ambient_conductance_w_k","initial_wall_temperature_k","source"};
        for(auto it=et.begin();it!=et.end();++it)
            if(std::none_of(std::begin(kKeys),std::end(kKeys),[&](const char* k){ return it.key()==k; }))
                throw std::runtime_error(ectx+": unknown key '"+it.key()+"'");
        if(et.contains("in_cylinder_wall_heat_fraction")) {
            t.in_cylinder_wall_heat_fraction=require_number(et,"in_cylinder_wall_heat_fraction",ectx);
            if(!(t.in_cylinder_wall_heat_fraction>=0 && t.in_cylinder_wall_heat_fraction<1))
                throw std::runtime_error(ectx+": 'in_cylinder_wall_heat_fraction' must be in [0, 1)");
        }
        if(et.contains("wall_heat_capacity_j_k")) t.exhaust_wall_heat_capacity_j_k=require_positive(et,"wall_heat_capacity_j_k",ectx);
        if(et.contains("wall_gas_conductance_w_k")) {
            t.exhaust_wall_gas_conductance_w_k=require_number(et,"wall_gas_conductance_w_k",ectx);
            if(!(t.exhaust_wall_gas_conductance_w_k>=0)) throw std::runtime_error(ectx+": 'wall_gas_conductance_w_k' must be >= 0");
        }
        if(et.contains("wall_ambient_conductance_w_k")) {
            t.exhaust_wall_ambient_conductance_w_k=require_number(et,"wall_ambient_conductance_w_k",ectx);
            if(!(t.exhaust_wall_ambient_conductance_w_k>=0)) throw std::runtime_error(ectx+": 'wall_ambient_conductance_w_k' must be >= 0");
        }
        if(et.contains("initial_wall_temperature_k")) t.exhaust_wall_initial_k=require_positive(et,"initial_wall_temperature_k",ectx);
        if(!et.contains("source") || !et.at("source").is_string() || et.at("source").get<std::string>().empty())
            throw std::runtime_error(ectx+": 'source' is required (PLAN.md section 8)");
    }
    // Versioned parameter key; cache is derived data, never an engine input.
    std::uint64_t key=14695981039346656037ULL;
    const real parameters[]={real(1.0),t.reference_head_pa,t.rotor_inertia_kgm2,t.reference_rpm,t.max_mass_flow_kg_s,
        t.compressor_efficiency,t.turbine_efficiency,t.turbine_area_m2,t.target_boost_pa,t.intercooler_effectiveness,t.shaft_loss_w_at_reference,
        t.wastegate_kp,t.wastegate_ki_per_s,t.wastegate_kd_s,t.wastegate_actuator_tau_s};
    for(real v:parameters) { unsigned char bytes[sizeof(v)]; std::memcpy(bytes,&v,sizeof(v));
        for(auto b:bytes) { key^=b; key*=1099511628211ULL; } }
    auto maps=std::make_shared<drivetrain::TurboPerformanceMaps>();
    std::filesystem::path cached;
    if(!cache_dir.empty()) { std::ostringstream hex; hex<<std::hex<<key;
        cached=std::filesystem::path(cache_dir)/"turbo"/(hex.str()+".json"); }
    bool hit=false;
    if(!cached.empty()) {
        try {
            const auto cj=read_json_file(cached.string());
            if(cj.at("key").get<std::uint64_t>()==key && cj.at("version").get<int>()==1) {
                maps->head_pa.x=cj.at("speed").get<std::vector<real>>();
                maps->head_pa.y=cj.at("flow").get<std::vector<real>>();
                maps->head_pa.values=cj.at("head").get<std::vector<real>>();
                maps->compressor_temperature_ratio.x=cj.at("pr").get<std::vector<real>>();
                maps->compressor_temperature_ratio.y=cj.at("temperature_ratio").get<std::vector<real>>();
                // Compare to the authored generator below before accepting.
                hit=maps->head_pa.x.size()==61 && maps->head_pa.y.size()==49 && maps->head_pa.values.size()==61*49 &&
                    maps->compressor_temperature_ratio.x.size()==91 && maps->compressor_temperature_ratio.y.size()==91;
            }
        } catch(...) { hit=false; }
    }
    drivetrain::TurboPerformanceMaps generated;
    for(int i=0;i<=60;++i) generated.head_pa.x.push_back(real(i)/real(40.0));
    for(int i=0;i<=48;++i) generated.head_pa.y.push_back(real(i)/real(40.0));
    for(real speed:generated.head_pa.x) for(real flow:generated.head_pa.y)
        generated.head_pa.values.push_back(t.reference_head_pa*speed*speed*std::max(real(0.0),real(1.0)-flow*flow));
    for(int i=0;i<=90;++i) { const real pr=real(1.0)+real(i)/real(10.0);
        generated.compressor_temperature_ratio.x.push_back(pr);
        generated.compressor_temperature_ratio.y.push_back(real(1.0)+(ps::math::pow(pr,real(2.0/7.0))-real(1.0))/t.compressor_efficiency); }
    hit=hit && maps->head_pa.x==generated.head_pa.x && maps->head_pa.y==generated.head_pa.y && maps->head_pa.values==generated.head_pa.values &&
        maps->compressor_temperature_ratio.x==generated.compressor_temperature_ratio.x && maps->compressor_temperature_ratio.y==generated.compressor_temperature_ratio.y;
    if(!hit) {
        *maps=std::move(generated);
        if(!cached.empty()) { try {
            std::filesystem::create_directories(cached.parent_path());
            json cj={{"version",1},{"key",key},{"speed",maps->head_pa.x},{"flow",maps->head_pa.y},{"head",maps->head_pa.values},
                {"pr",maps->compressor_temperature_ratio.x},{"temperature_ratio",maps->compressor_temperature_ratio.y}};
            std::ofstream out(cached); out<<cj.dump();
        } catch(...) { /* An unwritable cache does not prevent driving. */ } }
    }
    maps->content_hash=key;

    // Explicit opt-in: authored maps stay distinguishable from preliminary
    // offline results. Geometry generation NEVER runs during vehicle loading.
    if(hw.contains("performance_map")) {
        const auto stage_path=resolve_path(std::filesystem::path(hw_path).parent_path().string(),hw.at("performance_map").get<std::string>());
        const auto sj=read_json_file(stage_path);
        require_format(sj,"turbo_stage_maps",1,stage_path);
        auto sm=detail::stage_maps_from_json(sj,count,stage_path); // shared with ps::io::turbo_stage_maps_from_json
        // Hash the actual consumed file, including provenance; never reuse a
        // key from its legacy authored curve or trust a supplied hash alone.
        for(unsigned char b:sj.dump()) { key^=b; key*=1099511628211ULL; }
        maps->content_hash=key; maps->stages=std::move(sm);
    }
    t.maps=std::move(maps);
    return t;
}

vehicle::TyreDesc load_tyre_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "tyre", 1, path);
    return read_tyre_object(j);
}

drivetrain::InstallationDesc load_installation_json(const std::string& path) {
    const json j = parts::load_part(path); // engine garage P2: "extends" (an installation has no path keys)
    return read_installation_object(j, path);
}

// ps_audio A10: a standalone engine file, no vehicle wrapping it. Same
// dispatcher (read_engine_fields, in the anonymous namespace above) a
// vehicle's "ref"'d engine component reaches, given the engine file's own
// directory as the base for its further relative paths (fuel/installation),
// exactly component_ref_dir's rule for a "ref"'d (non-inline) component.
namespace {

// Engine garage P2: the shared tail of load_engine_json / load_engine_json_from_string - an engine object whose
// "extends" chain is resolved, its directory, the standalone slot overrides. Same assembly and reader a vehicle's
// engine component reaches.
drivetrain::ComponentParams load_engine_object(json j, const std::string& label, const std::string& engine_dir,
                                               const EngineMapOptions& options, const std::string* installation_override_path,
                                               const EngineSlotOverrides& slots) {
    require_format(j, "engine", 1, label);
    const std::string ctx = "engine '" + label + "'";
    parts::SlotMap resolved;
    if (installation_override_path != nullptr) resolved["installation"] = *installation_override_path;
    for (const auto& kv : slots) {
        if (!parts::is_engine_slot(kv.first)) {
            throw std::runtime_error(ctx + ": unknown slot '" + kv.first + "' (engine slots: ecu, cam, forced_induction, installation, thermal)");
        }
        if (kv.second.empty()) throw std::runtime_error(ctx + ": slot '" + kv.first + "' must be a non-empty path");
        if (kv.second == "none" && kv.first != "forced_induction") {
            throw std::runtime_error(ctx + ": slot '" + kv.first + "': \"none\" is only allowed for 'forced_induction'");
        }
        resolved[kv.first] = kv.second == "none" ? kv.second : resolve_path(engine_dir, kv.second);
    }
    const parts::AssembledEngine assembled = parts::assemble_engine(std::move(j), engine_dir, ctx, resolved);
    drivetrain::ComponentParams params =
        read_engine_fields(assembled.j, ctx, engine_dir, /*resolved_fuel_paths_out=*/nullptr, &options,
                           assembled.installation_path.empty() ? nullptr : &assembled.installation_path, &assembled);
    if (slots.count("installation") != 0) {
        const auto* sim = std::get_if<drivetrain::SimulatedEngineDesc>(&params);
        if (sim == nullptr || !sim->cycle.has_value() || !sim->cycle->intake.has_value()) {
            throw std::runtime_error(ctx + ": slot 'installation' is only allowed on a simulated engine with 'intake'/'exhaust' "
                                     "pipe blocks");
        }
    }
    return params;
}

} // namespace

// ps_audio A10: a standalone engine file, no vehicle wrapping it. Same
// dispatcher (read_engine_fields, in the anonymous namespace above) a
// vehicle's "ref"'d engine component reaches, given the engine file's own
// directory as the base for its further relative paths (fuel/installation),
// exactly component_ref_dir's rule for a "ref"'d (non-inline) component.
drivetrain::ComponentParams load_engine_json(const std::string& path, const EngineMapOptions& options,
                                              const std::string* installation_override_path, const EngineSlotOverrides& slots) {
    json j = parts::load_part(path, parts::rebase_engine_paths);
    const std::string engine_dir = std::filesystem::path(path).parent_path().string();
    return load_engine_object(std::move(j), path, engine_dir, options, installation_override_path, slots);
}

// Engine garage P2 (design A7): the same load from bytes in memory, as if they were a file in base_dir.
drivetrain::ComponentParams load_engine_json_from_string(std::string_view text, const std::filesystem::path& base_dir,
                                                          const EngineMapOptions& options, const EngineSlotOverrides& slots) {
    // allow_exceptions = false: no parse exception object is ever caught (CLAUDE.md "Windows ASan caveat").
    json root = json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
    const std::filesystem::path virtual_path = base_dir / "<engine string>";
    if (root.is_discarded()) throw std::runtime_error(virtual_path.string() + ": invalid JSON");
    json j = resolve_json_extends(std::move(root), virtual_path, 8, parts::rebase_engine_paths);
    return load_engine_object(std::move(j), virtual_path.string(), base_dir.string(), options, nullptr, slots);
}

vehicle::VehicleDesc load_vehicle_json(const std::string& path) { return load_vehicle_json(path, EngineMapOptions{}); }

vehicle::VehicleDesc load_vehicle_json(const std::string& path, const EngineMapOptions& options) {
    VehicleLoadOptions load_options;
    load_options.maps = options;
    return load_vehicle_json(path, load_options);
}

vehicle::VehicleDesc load_vehicle_json(const std::string& path, const VehicleLoadOptions& load_options) {
    const EngineMapOptions& options = load_options.maps;
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
    // Injected tyres (VehicleLoadOptions::tyres): every key must name a wheel of this file (typo guard); a wheel with
    // neither an injected desc nor a "tyre" key is a load error (it used to get a default-constructed TyreDesc).
    {
        std::set<std::string> wheel_names;
        for (const auto& wj : j.at("wheels")) wheel_names.insert(wj.value("name", std::string("wheel")));
        for (const auto& kv : load_options.tyres) {
            if (wheel_names.count(kv.first) == 0) {
                throw std::runtime_error("vehicle " + path + ": VehicleLoadOptions::tyres: no wheel named '" + kv.first + "'");
            }
        }
    }
    for (const auto& wj : j.at("wheels")) {
        const std::string wheel_name = wj.value("name", std::string("wheel"));
        const auto inj = load_options.tyres.find(wheel_name);
        const TyreDesc* injected = inj != load_options.tyres.end() ? &inj->second : nullptr;
        if (injected == nullptr && !wj.contains("tyre")) {
            throw std::runtime_error("vehicle " + path + ": wheel '" + wheel_name +
                                      "': no tyre (give a \"tyre\" path/object or VehicleLoadOptions::tyres)");
        }
        v.wheels.push_back(read_wheel(wj, base_dir, injected));
    }

    v.steering = read_steering(j.value("steering", json()));
    v.brake_system = read_brake_system(j.value("brake_system", json()));

    // --- aero (plan section 4.4/9.1) ---------------------------------------
    if(j.contains("traction_control")) {
        const auto& tc=j.at("traction_control");if(!tc.is_object())throw std::runtime_error("traction_control must be an object");
        for(auto it=tc.begin();it!=tc.end();++it)if(it.key()!="enabled"&&it.key()!="fitted"&&it.key()!="target_slip"&&it.key()!="reference_floor_m_s"&&it.key()!="kp"&&it.key()!="ki_per_s"&&it.key()!="attack_rate_per_s"&&it.key()!="recovery_rate_per_s")throw std::runtime_error("unknown traction_control field: "+it.key());
        auto& d=v.traction_control;d.enabled=tc.value("enabled",d.enabled);
        d.target_slip=tc.value("target_slip",d.target_slip);d.reference_floor_m_s=tc.value("reference_floor_m_s",d.reference_floor_m_s);
        d.kp=tc.value("kp",d.kp);d.ki_per_s=tc.value("ki_per_s",d.ki_per_s);
        d.attack_rate_per_s=tc.value("attack_rate_per_s",d.attack_rate_per_s);d.recovery_rate_per_s=tc.value("recovery_rate_per_s",d.recovery_rate_per_s);
        d.fitted=tc.value("fitted",d.fitted); // engine garage P8 (A15)
        ps::vehicle::validate_traction_control(d);
    }
    if (j.contains("aero")) {
        v.aero=read_aero(j.at("aero"),base_dir);
    }
    if (j.contains("contents_mass")) {
        const auto& cm = j.at("contents_mass");
        if (!cm.is_object()) throw std::runtime_error("vehicle " + path + ": contents_mass must be an object");
        for (auto it = cm.begin(); it != cm.end(); ++it) {
            if (it.key() != "enabled" && it.key() != "update_threshold_kg" && it.key() != "source")
                throw std::runtime_error("vehicle " + path + ": contents_mass: unknown key '" + it.key() + "'");
        }
        v.contents_mass.enabled = cm.value("enabled", v.contents_mass.enabled);
        v.contents_mass.update_threshold_kg = cm.value("update_threshold_kg", v.contents_mass.update_threshold_kg);
        ps::vehicle::validate_contents_mass(v.contents_mass);
    }

    // --- powertrain (plan section 9.1) --------------------------------------
    PowertrainDesc p;
    std::vector<std::string> component_ids;
    // PHYS-033 P4 I7a C6: engine id -> its resolved fuel file paths, and
    // tank id -> {fuel path as written, resolved fuel path}, for the fluid
    // link's "tank fuel is in the engine's fuels list" check.
    std::vector<std::pair<std::string, std::vector<std::string>>> engine_fuel_paths;
    std::vector<std::pair<std::string, std::pair<std::string, std::string>>> tank_fuel_paths;
    // Engine thermal plan T1: the first engine component's resolved thermal file ("" = it has none).
    std::string thermal_engine_id, thermal_engine_path;
    if (j.contains("powertrain")) {
        const auto& pj = j.at("powertrain");
        if (pj.contains("components")) {
            // Engine garage P2 (design A6): every user-layer override must name a component of this vehicle.
            for (const auto& ov : load_options.part_overrides) {
                bool found = false;
                for (const auto& c : pj.at("components")) {
                    found = found || (c.contains("id") && c.at("id").is_string() && c.at("id").get<std::string>() == ov.first);
                }
                if (!found) {
                    throw std::runtime_error("vehicle '" + v.name + "': part_overrides: no powertrain component with id '" + ov.first + "'");
                }
            }
            for (const auto& cj_file : pj.at("components")) {
                if (!cj_file.contains("id") || !cj_file.at("id").is_string() || cj_file.at("id").get<std::string>().empty()) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.components[]: 'id' is required and must be a non-empty string");
                }
                const std::string id = cj_file.at("id").get<std::string>();
                if (std::find(component_ids.begin(), component_ids.end(), id) != component_ids.end()) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.components[]: duplicate id '" + id + "'");
                }
                component_ids.push_back(id);
                if (!cj_file.contains("type") || !cj_file.at("type").is_string()) {
                    throw std::runtime_error("vehicle '" + v.name + "': powertrain.components['" + id + "']: 'type' is required");
                }
                const std::string type = cj_file.at("type").get<std::string>();
                const std::string ctx = "vehicle '" + v.name + "': powertrain.components['" + id + "']";
                // Engine garage P2 (design A6): the user layer rewrites this component's "ref" and slot keys before it
                // is read - the vehicle file as if it said this, so the user wins every slot it names.
                json cj_user;
                const auto override_it = load_options.part_overrides.find(id);
                const bool has_user_layer = override_it != load_options.part_overrides.end();
                if (has_user_layer) cj_user = apply_part_override(cj_file, type, override_it->second, ctx);
                const json& cj = has_user_layer ? cj_user : cj_file;

                const auto electric_keys=[&](const json& obj,std::initializer_list<const char*> allowed,const std::string& where) {
                    for(auto it=obj.begin();it!=obj.end();++it) {
                        bool found=false;for(const char* key:allowed)if(it.key()==key){found=true;break;}
                        if(!found)throw std::runtime_error(where+": unknown electric component field '"+it.key()+"'");
                    }
                };
                if (type == "battery") {
                    electric_keys(cj,{"id","type","capacity_kwh","initial_soc","max_discharge_w","max_charge_w","mass_kg"},ctx);
                    ps::drivetrain::BatteryDesc b;
                    b.capacity_kwh=cj.value("capacity_kwh",b.capacity_kwh);b.initial_soc=cj.value("initial_soc",b.initial_soc);
                    b.max_discharge_w=cj.value("max_discharge_w",b.max_discharge_w);b.max_charge_w=cj.value("max_charge_w",b.max_charge_w);
                    b.mass_kg=cj.value("mass_kg",b.mass_kg);
                    p.components.push_back({id,b});
                } else if(type == "electric_motor") {
                    electric_keys(cj,{"id","type","battery","peak_torque_nm","max_power_w","max_rpm","inertia_kgm2","efficiency",
                        "regen_torque_nm","regen_power_w","mass_kg","axis_local","ready_channel","throttle_channel","regen_channel","direction_channel","crank_assist"},ctx);
                    ps::drivetrain::ElectricMotorDesc m;
                    m.battery=cj.at("battery").get<std::string>();
                    m.peak_torque_nm=cj.value("peak_torque_nm",m.peak_torque_nm);m.max_power_w=cj.value("max_power_w",m.max_power_w);
                    m.max_rpm=cj.value("max_rpm",m.max_rpm);m.inertia_kgm2=cj.value("inertia_kgm2",m.inertia_kgm2);
                    m.efficiency=cj.value("efficiency",m.efficiency);m.regen_torque_nm=cj.value("regen_torque_nm",m.regen_torque_nm);
                    m.regen_power_w=cj.value("regen_power_w",m.regen_power_w);
                    if(!std::isfinite(m.regen_power_w)||m.regen_power_w<0)throw std::runtime_error("electric_motor '"+id+"': regen_power_w must be finite and >= 0");
                    m.mass_kg=cj.value("mass_kg",m.mass_kg);m.axis_local=read_vec3(cj.value("axis_local",json()),m.axis_local);
                    m.ready_channel=cj.value("ready_channel",m.ready_channel);
                    m.throttle_channel=cj.value("throttle_channel",std::string{});m.regen_channel=cj.value("regen_channel",std::string{});
                    m.direction_channel=cj.value("direction_channel",std::string{});
                    m.crank_assist=cj.value("crank_assist",false);
                    p.components.push_back({id,m});
                } else if(type=="hybrid_control") {
                    electric_keys(cj,{"id","type","requested_power_w","low_soc","charge_target_soc","charge_rpm","generator_fraction","stop_delay_s","crank_timeout_s","connect_timeout_s"},ctx);
                    ps::drivetrain::HybridControlDesc h;
                    h.requested_power_w=cj.value("requested_power_w",h.requested_power_w);h.low_soc=cj.value("low_soc",h.low_soc);h.charge_target_soc=cj.value("charge_target_soc",h.charge_target_soc);
                    h.charge_rpm=cj.value("charge_rpm",h.charge_rpm);h.generator_fraction=cj.value("generator_fraction",h.generator_fraction);
                    h.stop_delay_s=cj.value("stop_delay_s",h.stop_delay_s);h.crank_timeout_s=cj.value("crank_timeout_s",h.crank_timeout_s);
                    h.connect_timeout_s=cj.value("connect_timeout_s",h.connect_timeout_s);
                    if(!(h.connect_timeout_s>=0)||!std::isfinite(h.connect_timeout_s))throw std::runtime_error(ctx+": hybrid_control connect_timeout_s must be >= 0");
                    p.components.push_back({id,h});
                } else if (type == "fuel_cell") {
                    electric_keys(cj,{"id","type","battery","max_power_w","efficiency","target_soc","hydrogen_capacity_kg", "initial_hydrogen_kg","specific_energy_j_kg","mass_kg","ready_channel"},ctx);
                    ps::drivetrain::FuelCellDesc f;f.battery=cj.at("battery").get<std::string>();
                    f.max_power_w=cj.value("max_power_w",f.max_power_w);f.efficiency=cj.value("efficiency",f.efficiency);
                    f.target_soc=cj.value("target_soc",f.target_soc);f.hydrogen_capacity_kg=cj.value("hydrogen_capacity_kg",f.hydrogen_capacity_kg);
                    f.initial_hydrogen_kg=cj.value("initial_hydrogen_kg",f.initial_hydrogen_kg);f.specific_energy_j_kg=cj.value("specific_energy_j_kg",f.specific_energy_j_kg);
                    f.mass_kg=cj.value("mass_kg",f.mass_kg);f.ready_channel=cj.value("ready_channel",f.ready_channel);
                    p.components.push_back({id,f});
                } else if (type == "engine") {
                    // Engine garage P2: the component's slots (vehicle level - the user layer, applied above, already
                    // rewrote them), then the engine file with its "extends" chain, then the slot assembly (ECU and cam
                    // parts merged; forced induction, installation and thermal resolved to paths). See engine_parts.h.
                    const bool inline_engine = !cj.contains("ref");
                    const parts::SlotMap slots = parts::read_component_slots(cj, base_dir, ctx, inline_engine);
                    json ej = resolve_component_ref(cj, "engine", base_dir, ctx);
                    if (inline_engine) parts::erase_component_slots(ej);
                    const std::string engine_dir = component_ref_dir(cj, base_dir);
                    const parts::AssembledEngine assembled = parts::assemble_engine(std::move(ej), engine_dir, ctx, slots);
                    std::vector<std::string> fuel_paths;
                    // ps_audio A5 (P12): the installation slot reaches the engine reader, which resolves it before any
                    // map generation.
                    p.components.push_back({id, read_engine_fields(assembled.j, ctx, engine_dir, &fuel_paths, &options,
                                                                   assembled.installation_path.empty() ? nullptr
                                                                                                       : &assembled.installation_path,
                                                                   &assembled)});
                    engine_fuel_paths.emplace_back(id, std::move(fuel_paths)); // PHYS-033 P4 I7a C6: for the fluid-link check below

                    // Engine thermal plan T1 (section 9.1): the engine file's "default_thermal" (relative to the engine
                    // file), overridable by this component's own "thermal" (relative to the vehicle file) - now the
                    // "thermal" slot. Only used when the vehicle names a cooling_system (opt-in) - resolved after the
                    // component loop.
                    if (thermal_engine_id.empty()) {
                        thermal_engine_path = assembled.thermal_path;
                        thermal_engine_id = id;
                    }

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
                } else if (type == "n2o_bottle") {
                    // N2O plan N1: a part file ("ref") plus the component's own bottom-up overrides.
                    p.components.push_back({id, ps::io::detail::build_n2o_bottle_component(cj, base_dir, ctx)});
                } else if (type == "nitrous_kit") {
                    p.components.push_back({id, ps::io::detail::build_nitrous_kit_component(cj, base_dir, ctx)});
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
                // N2O plan N1: "<bottle>.out" -> "<kit>.n2o" and "<kit>.out" -> "<engine>.nitrous".
                if (std::get_if<ps::drivetrain::N2oBottleDesc>(&from_c->params) != nullptr) {
                    if (from.port != "out") {
                        throw std::runtime_error(lctx + ": unknown port \"" + from_str + "\" (an n2o_bottle's only fluid port is \"out\")");
                    }
                    const ps::drivetrain::ComponentDesc* kit_c = find_component(to.component);
                    if (kit_c == nullptr) throw std::runtime_error(lctx + ": unknown component '" + to.component + "'");
                    if (std::get_if<ps::drivetrain::NitrousKitDesc>(&kit_c->params) == nullptr) {
                        throw std::runtime_error(lctx + ": '" + to.component + "' is not a nitrous_kit (an n2o_bottle's link must end at \"<kit>.n2o\")");
                    }
                    if (to.port != "n2o") {
                        throw std::runtime_error(lctx + ": unknown port \"" + to_str + "\" (a nitrous_kit's bottle input is \"n2o\")");
                    }
                    // N2O plan N4: a heater channel shared with the controller's arm or purge channel (a vehicle-level controller
                    // override can introduce the clash; declared channels merge by name, so the two switches would be one).
                    if (const auto* bo = std::get_if<ps::drivetrain::N2oBottleDesc>(&from_c->params);
                        bo != nullptr && bo->has_heater) {
                        const auto& kc = std::get<ps::drivetrain::NitrousKitDesc>(kit_c->params).controller;
                        if (bo->heater.channel == kc.arm_channel) {
                            throw std::runtime_error(lctx + ": the bottle heater channel '" + bo->heater.channel + "' is also the arm channel of nitrous_kit '" +
                                                      to.component + "' controller");
                        }
                        if (kc.has_purge && bo->heater.channel == kc.purge.channel) {
                            throw std::runtime_error(lctx + ": the bottle heater channel '" + bo->heater.channel + "' is also the purge channel of nitrous_kit '" +
                                                      to.component + "' controller");
                        }
                    }
                    for (const auto& existing : p.fluid) {
                        if (existing.to.component == to.component && existing.to.port == "n2o") {
                            throw std::runtime_error(lctx + ": nitrous_kit '" + to.component + "' n2o port is fed by two bottles ('" +
                                                      existing.from.component + "' and '" + from.component + "')");
                        }
                        if (existing.from.component == from.component && existing.from.port == "out" && existing.to.port == "n2o") {
                            throw std::runtime_error(lctx + ": n2o_bottle '" + from.component + "' already feeds a kit");
                        }
                    }
                    p.fluid.push_back({from, to});
                    continue;
                }
                if (std::get_if<ps::drivetrain::NitrousKitDesc>(&from_c->params) != nullptr) {
                    if (from.port != "out") {
                        throw std::runtime_error(lctx + ": unknown port \"" + from_str + "\" (a nitrous_kit's only output port is \"out\")");
                    }
                    const ps::drivetrain::ComponentDesc* eng_c = find_component(to.component);
                    if (eng_c == nullptr) throw std::runtime_error(lctx + ": unknown component '" + to.component + "'");
                    const auto* eng = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&eng_c->params);
                    const auto* sim_eng = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&eng_c->params);
                    if (eng == nullptr && sim_eng == nullptr) {
                        throw std::runtime_error(lctx + ": '" + to.component + "' is not an engine (a kit's link must end at \"<engine>.nitrous\")");
                    }
                    if (to.port != "nitrous") {
                        throw std::runtime_error(lctx + ": unknown port \"" + to_str + "\" (an engine's nitrous input is \"nitrous\")");
                    }
                    if (!(eng != nullptr ? ps::drivetrain::has_fuel_model(*eng) : ps::drivetrain::has_fuel_model(*sim_eng))) {
                        throw std::runtime_error(lctx + ": engine '" + to.component +
                                                  "' has no fuel model (no 'fuels', 'fuel' or 'fuel_map') - a nitrous_kit's fuel jet needs its fuel");
                    }
                    for (const auto& existing : p.fluid) {
                        if (existing.to.component == to.component && existing.to.port == "nitrous") {
                            throw std::runtime_error(lctx + ": engine '" + to.component + "' nitrous port is fed by two kits ('" +
                                                      existing.from.component + "' and '" + from.component + "')");
                        }
                        if (existing.from.component == from.component && existing.from.port == "out" && existing.to.port == "nitrous") {
                            throw std::runtime_error(lctx + ": nitrous_kit '" + from.component + "' already feeds an engine");
                        }
                    }
                    p.fluid.push_back({from, to});
                    continue;
                }
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
                    if (existing.to.component != to.component || existing.to.port != "fuel") continue;
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
    // N2O plan N1: every bottle needs a kit and every kit a bottle and an engine, so a half-wired nitrous
    // system is rejected at load (Powertrain repeats the check for a code-built desc).
    for (const auto& c : p.components) {
        const bool is_bottle = std::get_if<ps::drivetrain::N2oBottleDesc>(&c.params) != nullptr;
        const bool is_kit = std::get_if<ps::drivetrain::NitrousKitDesc>(&c.params) != nullptr;
        if (!is_bottle && !is_kit) continue;
        bool as_source = false, as_target = false;
        for (const auto& link : p.fluid) {
            as_source = as_source || (link.from.component == c.id && link.from.port == "out");
            as_target = as_target || (link.to.component == c.id && link.to.port == "n2o");
        }
        const std::string cctx = "vehicle '" + v.name + "': powertrain.components['" + c.id + "']";
        if (is_bottle && !as_source) {
            throw std::runtime_error(cctx + ": n2o_bottle has no consumer (no powertrain.fluid link from \"" + c.id + ".out\")");
        }
        if (is_kit && !as_target) {
            throw std::runtime_error(cctx + ": nitrous_kit has no bottle (no powertrain.fluid link to \"" + c.id + ".n2o\")");
        }
        if (is_kit && !as_source) {
            throw std::runtime_error(cctx + ": nitrous_kit has no consumer (no powertrain.fluid link from \"" + c.id + ".out\")");
        }
    }
    // --- engine thermal (engine thermal plan T1 section 9.1): opt-in by naming a cooling system -------------
    if (j.contains("engine_thermal_start") && !j.contains("cooling_system")) {
        throw std::runtime_error("vehicle '" + v.name + "': 'engine_thermal_start' needs a 'cooling_system' (the vehicle opts in to engine thermal by naming one)");
    }
    if (j.contains("cooling_system")) {
        const std::string vctx = "vehicle '" + v.name + "'";
        if (!j.at("cooling_system").is_string() || j.at("cooling_system").get<std::string>().empty()) {
            throw std::runtime_error(vctx + ": 'cooling_system' must be a non-empty path string");
        }
        if (thermal_engine_id.empty()) {
            throw std::runtime_error(vctx + ": 'cooling_system' needs an engine component");
        }
        if (thermal_engine_path.empty()) {
            throw std::runtime_error(vctx + ": 'cooling_system' needs an engine thermal file, but engine '" + thermal_engine_id +
                                      "' has neither a 'default_thermal' nor a component 'thermal' override");
        }
        ps::drivetrain::EngineThermalConfig tc;
        tc.engine = ps::io::load_engine_thermal_json(thermal_engine_path);
        tc.cooling = ps::io::load_cooling_system_json(resolve_path(base_dir, j.at("cooling_system").get<std::string>()));
        if (j.contains("engine_thermal_start")) {
            if (!j.at("engine_thermal_start").is_string()) {
                throw std::runtime_error(vctx + ": 'engine_thermal_start' must be \"warm\" or \"ambient\"");
            }
            tc.start = ps::io::parse_thermal_start(j.at("engine_thermal_start").get<std::string>(), vctx);
        }
        // One energy split must never use two fractions (plan section 4.3): a turbo engine's exhaust_thermal
        // wall-heat fraction has to equal the thermal file's until T3 moves the owner.
        for (const auto& c : p.components) {
            if (c.id != thermal_engine_id) continue;
            if (const auto* sim = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
                if (sim->turbo_pair.has_value() &&
                    sim->turbo_pair->in_cylinder_wall_heat_fraction != tc.engine.wall_heat_fraction) {
                    throw std::runtime_error(vctx + ": engine '" + thermal_engine_id + "': the turbo configuration's exhaust_thermal "
                                              "in_cylinder_wall_heat_fraction (" + std::to_string(sim->turbo_pair->in_cylinder_wall_heat_fraction) +
                                              ") must equal the engine thermal file's wall_heat_fraction (" +
                                              std::to_string(tc.engine.wall_heat_fraction) + ") - one energy split, one fraction");
                }
            }
        }
        p.thermal = std::move(tc);
    }
    v.powertrain = p;

    // Engine garage P8 (A15): the car-side anti-lag switch. Absent = fitted (every pre-P8 file unchanged).
    if (j.contains("anti_lag_switch")) {
        const auto& asw = j.at("anti_lag_switch");
        if (!asw.is_object()) throw std::runtime_error("vehicle '" + v.name + "': anti_lag_switch must be an object");
        for (auto it = asw.begin(); it != asw.end(); ++it) {
            if (it.key() != "fitted") throw std::runtime_error("vehicle '" + v.name + "': unknown anti_lag_switch field: " + it.key());
        }
        if (asw.contains("fitted") && !asw.at("fitted").is_boolean()) {
            throw std::runtime_error("vehicle '" + v.name + "': anti_lag_switch.fitted must be a boolean");
        }
        v.powertrain.anti_lag_fitted = asw.value("fitted", true);
    }

    // --- controllers (plan section 4.2/9.1) ---------------------------------
    // Engine garage P8: "electronics" entries ({"id", "type": "electronics", "ref"}) are collected here and applied
    // after the manual_tcu (they may mark its assists fitted).
    std::vector<std::pair<std::string, std::string>> electronics_entries; // id -> resolved part path, file order
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
            if (type == "electronics") {
                if (!cj.contains("ref") || !cj.at("ref").is_string() || cj.at("ref").get<std::string>().empty()) {
                    throw std::runtime_error(ctx + ": an 'electronics' entry needs a 'ref' (path to a physics_sim.electronics/1 file)");
                }
                for (const auto& e : electronics_entries) {
                    if (e.first == id) throw std::runtime_error(ctx + ": duplicate controller id");
                }
                electronics_entries.emplace_back(id, resolve_path(base_dir, cj.at("ref").get<std::string>()));
                continue;
            }
            if (type == "automatic_tcu") {
                v.controllers.push_back({id, read_automatic_tcu(cj, ctx, component_ids)});
                continue;
            }
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
                md.auto_clutch.fitted = acj.value("fitted", md.auto_clutch.fitted); // engine garage P8 (A15)
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
                md.auto_blip.fitted = abj.value("fitted", md.auto_blip.fitted); // engine garage P8 (A15)
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
                md.auto_shift.fitted = asj.value("fitted", md.auto_shift.fitted); // engine garage P8 (A15)
                md.auto_shift.up_rpm_at_zero_throttle = asj.value("up_rpm_at_zero_throttle", md.auto_shift.up_rpm_at_zero_throttle);
                md.auto_shift.up_rpm_at_full_throttle = asj.value("up_rpm_at_full_throttle", md.auto_shift.up_rpm_at_full_throttle);
                md.auto_shift.down_rpm_at_zero_throttle = asj.value("down_rpm_at_zero_throttle", md.auto_shift.down_rpm_at_zero_throttle);
                md.auto_shift.down_rpm_at_full_throttle = asj.value("down_rpm_at_full_throttle", md.auto_shift.down_rpm_at_full_throttle);
                md.auto_shift.min_time_in_gear_s = asj.value("min_time_in_gear_s", md.auto_shift.min_time_in_gear_s);
                md.auto_shift.kickdown_throttle = asj.value("kickdown_throttle", md.auto_shift.kickdown_throttle); // assists I7
                md.auto_shift.max_limiter_fraction = asj.value("max_limiter_fraction", md.auto_shift.max_limiter_fraction); // G1
                if (!std::isfinite(md.auto_shift.max_limiter_fraction) || !(md.auto_shift.max_limiter_fraction > real(0.0)) ||
                    md.auto_shift.max_limiter_fraction > real(1.0)) {
                    throw std::runtime_error(ctx + ".auto_shift: max_limiter_fraction must be in (0, 1]");
                }
                if (!std::isfinite(md.auto_shift.kickdown_throttle) || md.auto_shift.kickdown_throttle < real(0.0) || md.auto_shift.kickdown_throttle > real(1.0)) {
                    throw std::runtime_error(ctx + ".auto_shift: kickdown_throttle must be in [0, 1] (0 = no bypass)");
                }
            }
            md.max_pending_steps = cj.value("max_pending_steps", md.max_pending_steps);

            v.controllers.push_back({id, md});
        }
    }
    if (v.controllers.size() > 1) {
        throw std::runtime_error("vehicle '" + v.name + "': at most one controller (manual_tcu or automatic_tcu) is supported");
    }

    // --- electronics (engine garage P8, A15/A16, docs/data_formats/electronics.md) -----------------------------
    // The user layer (load_options.electronics) is applied over the file's entries: a path replaces or adds, "none"
    // removes. Then every part is loaded in order: a launch_control becomes a controller, an assist_fitment marks
    // assists fitted.
    {
        for (const auto& kv : load_options.electronics) {
            const std::string& eid = kv.first;
            for (const auto& c : v.controllers) {
                if (c.id == eid) {
                    throw std::runtime_error("vehicle '" + v.name + "': electronics: id '" + eid + "' is the manual_tcu controller, not an electronics entry");
                }
            }
            const auto it = std::find_if(electronics_entries.begin(), electronics_entries.end(),
                                         [&](const auto& e) { return e.first == eid; });
            if (kv.second.empty()) {
                throw std::runtime_error("vehicle '" + v.name + "': electronics['" + eid + "']: the path must be non-empty (or \"none\" to remove)");
            }
            if (kv.second == "none") {
                if (it == electronics_entries.end()) {
                    throw std::runtime_error("vehicle '" + v.name + "': electronics: no electronics entry with id '" + eid + "' to remove");
                }
                electronics_entries.erase(it);
            } else if (it != electronics_entries.end()) {
                it->second = resolve_path(base_dir, kv.second);
            } else {
                electronics_entries.emplace_back(eid, resolve_path(base_dir, kv.second));
            }
        }
        v.controllers.reserve(v.controllers.size() + electronics_entries.size()); // keeps `tcu` below valid
        ps::drivetrain::ManualTcuDesc* tcu = nullptr;
        ps::drivetrain::AutomaticTcuDesc* automatic_tcu = nullptr;
        for (auto& c : v.controllers) {
            if (auto* t = std::get_if<ps::drivetrain::ManualTcuDesc>(&c.params)) tcu = t;
            if (auto* t = std::get_if<ps::drivetrain::AutomaticTcuDesc>(&c.params)) automatic_tcu = t;
        }
        bool have_launch = false;
        for (const auto& e : electronics_entries) {
            const std::string ectx = "vehicle '" + v.name + "': electronics['" + e.first + "']";
            const ElectronicsPart part = load_electronics_json(e.second);
            if (part.kind == ElectronicsKind::LaunchControl) {
                if (have_launch) throw std::runtime_error(ectx + ": at most one launch_control per vehicle");
                have_launch = true;
                // The second limit must lie strictly between the (first) engine's idle speed and its own limiter.
                real idle_rpm = 0.0, limiter_rpm = 0.0;
                bool have_engine = false;
                for (const auto& comp : v.powertrain.components) {
                    if (const auto* te = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&comp.params)) {
                        idle_rpm = te->idle_rpm; limiter_rpm = te->limiter.rpm; have_engine = true; break;
                    }
                    if (const auto* se = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&comp.params)) {
                        idle_rpm = se->idle_rpm; limiter_rpm = se->limiter.rpm; have_engine = true; break;
                    }
                }
                if (!have_engine) throw std::runtime_error(ectx + ": launch_control needs an engine component");
                if (!(part.launch.rpm > idle_rpm)) {
                    throw std::runtime_error(ectx + ": launch rpm " + std::to_string(part.launch.rpm) + " must be above the engine's idle rpm " +
                                             std::to_string(idle_rpm));
                }
                // G1: a launch rpm at or above max_limiter_fraction of the limiter (a low garage limiter) is clamped there by
                // the Powertrain; the load warns instead of failing.
                const ps::drivetrain::LaunchRpmFit fit =
                    ps::drivetrain::fit_launch_rpm(part.launch.rpm, idle_rpm, limiter_rpm, part.launch.max_limiter_fraction);
                if (!(limiter_rpm > idle_rpm)) {
                    throw std::runtime_error(ectx + ": the engine's limiter rpm " + std::to_string(limiter_rpm) + " must be above its idle rpm " +
                                             std::to_string(idle_rpm));
                }
                if (fit.clamped && options.warnings_out != nullptr) {
                    options.warnings_out->push_back(ectx + ": Limiting: launch rpm " + fmt_rpm(part.launch.rpm) + " is at or above " +
                                                    fmt_rpm(fit.cap) + " (" + fmt_rpm(part.launch.max_limiter_fraction * 100.0) +
                                                    " % of the " + fmt_rpm(limiter_rpm) + " rpm limiter): held at " + fmt_rpm(fit.rpm) + " rpm");
                }
                v.controllers.push_back({e.first, part.launch});
            } else if (part.kind == ElectronicsKind::AssistUnit) {
                if (v.assist_unit) throw std::runtime_error(ectx + ": at most one assist_unit per vehicle");
                // Every offered mode must resolve to a valid controller against the car's own traction_control block.
                try {
                    (void)ps::vehicle::resolve_traction_mode_descs(v.traction_control, std::optional<ps::vehicle::AssistUnitDesc>(part.unit));
                } catch (const std::invalid_argument& ex) {
                    throw std::runtime_error(ectx + ": " + ex.what());
                }
                if (part.unit.launch_control) {
                    // Assists I4: the launch rpm must lie strictly between the engine's idle and its own limiter (the
                    // P8 rule), resolved against this car's engine for a launch_rpm_fraction unit.
                    real idle_rpm = 0.0, limiter_rpm = 0.0;
                    bool have_engine = false;
                    for (const auto& comp : v.powertrain.components) {
                        if (const auto* te = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&comp.params)) {
                            idle_rpm = te->idle_rpm; limiter_rpm = te->limiter.rpm; have_engine = true; break;
                        }
                        if (const auto* se = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&comp.params)) {
                            idle_rpm = se->idle_rpm; limiter_rpm = se->limiter.rpm; have_engine = true; break;
                        }
                    }
                    if (!have_engine) throw std::runtime_error(ectx + ": launch_control needs an engine component");
                    const ps::vehicle::LaunchAssistDesc& la = *part.unit.launch_control;
                    const real rpm = ps::vehicle::launch_assist_rpm(la, limiter_rpm);
                    // G1: an rpm at or above max_limiter_fraction of the limiter (a low garage limiter) is clamped there by
                    // VehicleInstance; the load warns instead of failing. Below idle stays an error (no limiter causes it) -
                    // except where the clamp itself brings it above idle (an absolute launch_rpm over a low limiter).
                    const ps::drivetrain::LaunchRpmFit fit = ps::drivetrain::fit_launch_rpm(rpm, idle_rpm, limiter_rpm, la.max_limiter_fraction);
                    if (!(limiter_rpm > idle_rpm) || !(fit.rpm > idle_rpm) || !(fit.rpm < limiter_rpm)) {
                        throw std::runtime_error(ectx + ": launch rpm " + std::to_string(rpm) + " must lie between the engine's idle rpm " +
                                                 std::to_string(idle_rpm) + " and its own limiter rpm " + std::to_string(limiter_rpm));
                    }
                    if (fit.clamped && options.warnings_out != nullptr) {
                        options.warnings_out->push_back(ectx + ": Limiting: launch rpm " + fmt_rpm(rpm) + " is at or above " + fmt_rpm(fit.cap) +
                                                        " (" + fmt_rpm(la.max_limiter_fraction * 100.0) + " % of the " + fmt_rpm(limiter_rpm) +
                                                        " rpm limiter): held at " + fmt_rpm(fit.rpm) + " rpm");
                    }
                }
                // ABS / ESC act through the car's service-brake corners, so the car must have them.
                if (part.unit.abs) {
                    bool any_corner = false;
                    for (const auto& c : v.brake_system.corners) any_corner = any_corner || c.max_torque_nm > real(0);
                    if (!any_corner) throw std::runtime_error(ectx + ": abs needs brake_system.corners with max_torque_nm > 0");
                }
                v.assist_unit = part.unit;
            } else if (part.kind == ElectronicsKind::DriveModes) {
                // Drive modes support manual and automatic TCUs; the clutch-close override controls the shift close ramp.
                // Each offered mode's merged thresholds must keep a hysteresis (down below up at zero and at full throttle).
                if (v.drive_modes) throw std::runtime_error(ectx + ": at most one drive_modes per vehicle");
                for (int m = 0; m < ps::vehicle::kDriveModeCount; ++m) {
                    if (!(part.drive_modes.modes_bits & ps::vehicle::drive_mode_bit(static_cast<ps::vehicle::DriveMode>(m)))) continue;
                    const ps::vehicle::DriveModeParams& dmp = part.drive_modes.params[static_cast<std::size_t>(m)];
                    const bool shift = dmp.up_rpm_at_zero_throttle || dmp.up_rpm_at_full_throttle || dmp.down_rpm_at_zero_throttle || dmp.down_rpm_at_full_throttle;
                    if ((shift || dmp.clutch_close_time_s) && tcu == nullptr && automatic_tcu == nullptr) {
                        throw std::runtime_error(ectx + ": mode '" + ps::vehicle::kDriveModeNames[m] + "' overrides auto_shift / auto_clutch but the vehicle has no transmission controller");
                    }
                    if (shift) {
                        const auto check_schedule = [&](real base_uz, real base_uf, real base_dz, real base_df) {
                            const real uz = dmp.up_rpm_at_zero_throttle.value_or(base_uz);
                            const real uf = dmp.up_rpm_at_full_throttle.value_or(base_uf);
                            const real dz = dmp.down_rpm_at_zero_throttle.value_or(base_dz);
                            const real df = dmp.down_rpm_at_full_throttle.value_or(base_df);
                            if (!(dz < uz) || !(df < uf)) {
                                throw std::runtime_error(ectx + ": mode '" + ps::vehicle::kDriveModeNames[m] +
                                                         "': the merged auto-shift downshift rpm must stay below the upshift rpm at zero and at full throttle");
                            }
                        };
                        if (tcu) {
                            const auto& a = tcu->auto_shift;
                            check_schedule(a.up_rpm_at_zero_throttle, a.up_rpm_at_full_throttle,
                                           a.down_rpm_at_zero_throttle, a.down_rpm_at_full_throttle);
                        } else {
                            for (const auto& a : {automatic_tcu->schedule, automatic_tcu->sport_schedule})
                                check_schedule(a.up_rpm_at_zero_throttle, a.up_rpm_at_full_throttle,
                                               a.down_rpm_at_zero_throttle, a.down_rpm_at_full_throttle);
                        }
                    }
                }
                v.drive_modes = part.drive_modes;
            } else if (part.kind == ElectronicsKind::Epb) {
                // Q3 / I8: at most one, and it drives the car's handbrake rows, so the car must have them.
                if (v.epb) throw std::runtime_error(ectx + ": at most one epb per vehicle");
                if (v.brake_system.handbrake.wheels.empty() || !(v.brake_system.handbrake.max_torque_nm > real(0))) {
                    throw std::runtime_error(ectx + ": epb needs brake_system.handbrake rows (wheels and max_torque_nm > 0)");
                }
                v.epb = part.epb;
            } else {
                for (const std::string& f : part.fits) {
                    if (f == "traction_control") { v.traction_control.fitted = true; continue; }
                    if (f == "anti_lag") { v.powertrain.anti_lag_fitted = true; continue; }
                    if (tcu == nullptr) throw std::runtime_error(ectx + ": fits '" + f + "' but the vehicle has no manual_tcu");
                    if (f == "auto_clutch") tcu->auto_clutch.fitted = true;
                    else if (f == "auto_blip") tcu->auto_blip.fitted = true;
                    else tcu->auto_shift.fitted = true;
                }
            }
        }
        // Assists I4 (owner Q5): a car has EITHER a P8 two-step (a launch_control electronics part) OR an assist-unit
        // launch control, never both - checked after the loop so the order of the two entries does not matter.
        if (have_launch && v.assist_unit && v.assist_unit->launch_control) {
            throw std::runtime_error("vehicle '" + v.name +
                                     "': launch control: a car has either a two-step (launch_control) or an assist-unit launch control, not both");
        }
    }

    // G1: auto-shift points above the engine's effective limiter are scaled down at run time (ManualTcu, limiter_range.h
    // shift_point_scale); tell the caller, for the car's own points and for every drive mode that overrides them.
    if (options.warnings_out != nullptr) {
        for (const auto& c : v.controllers) {
            const auto* tcu = std::get_if<ps::drivetrain::ManualTcuDesc>(&c.params);
            if (tcu == nullptr) continue;
            real cap = real(0.0);
            for (const auto& comp : v.powertrain.components) {
                if (comp.id == tcu->engine) cap = tcu->auto_shift.max_limiter_fraction * ps::drivetrain::effective_limiter_rpm(comp.params);
            }
            const auto check = [&](const std::string& where, real uz, real uf, real dz, real df) {
                const real k = ps::drivetrain::shift_point_scale(uz, uf, cap);
                if (k == real(1.0)) return;
                options.warnings_out->push_back("vehicle '" + v.name + "': manual_tcu '" + c.id + "' auto_shift" + where + ": Limiting: the upshift at " +
                                                fmt_rpm(std::max(uz, uf)) + " rpm is above " + fmt_rpm(cap) + " rpm (" +
                                                fmt_rpm(tcu->auto_shift.max_limiter_fraction * real(100.0)) +
                                                " % of the effective limiter): the shift points are scaled by " + fmt_rpm(k) + " (up " +
                                                fmt_rpm(uz * k) + "/" + fmt_rpm(uf * k) + ", down " + fmt_rpm(dz * k) + "/" + fmt_rpm(df * k) + " rpm)");
            };
            const ps::drivetrain::AutoShiftDesc& a = tcu->auto_shift;
            check("", a.up_rpm_at_zero_throttle, a.up_rpm_at_full_throttle, a.down_rpm_at_zero_throttle, a.down_rpm_at_full_throttle);
            if (v.drive_modes) {
                for (int m = 0; m < ps::vehicle::kDriveModeCount; ++m) {
                    if (!(v.drive_modes->modes_bits & ps::vehicle::drive_mode_bit(static_cast<ps::vehicle::DriveMode>(m)))) continue;
                    const ps::vehicle::DriveModeParams& dmp = v.drive_modes->params[static_cast<std::size_t>(m)];
                    if (!(dmp.up_rpm_at_zero_throttle || dmp.up_rpm_at_full_throttle || dmp.down_rpm_at_zero_throttle || dmp.down_rpm_at_full_throttle)) continue;
                    check(std::string(" (drive mode '") + ps::vehicle::kDriveModeNames[m] + "')", dmp.up_rpm_at_zero_throttle.value_or(a.up_rpm_at_zero_throttle),
                          dmp.up_rpm_at_full_throttle.value_or(a.up_rpm_at_full_throttle), dmp.down_rpm_at_zero_throttle.value_or(a.down_rpm_at_zero_throttle),
                          dmp.down_rpm_at_full_throttle.value_or(a.down_rpm_at_full_throttle));
                }
            }
        }
    }

    return v;
}

} // namespace ps::io
