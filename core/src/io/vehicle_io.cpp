#include "ps/io/vehicle_io.h"

#include "../vehicle/tir_parser.h"

#include <nlohmann/json.hpp>

#include <algorithm>
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

// PLAN.md P4 I0 (PHYS-033 section 9 / owner ruling Q7: NO legacy support -
// missing, unknown or outdated is a hard error, never a silent default).
// `kind` is e.g. "vehicle" or "tyre"; `major` the only accepted major
// version. Every standalone JSON file this loader reads goes through this
// check exactly once, right after parsing.
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

// PHYS-033 P4 I2 plan section 9.1. Optional; absent means every wheel is
// free (no powertrain at all). Graph validation (unknown ports, double-
// connected ports, a zero-inertia unconnected hub, ...) is NOT done here -
// it happens in drivetrain::Powertrain's constructor (plan section 4),
// which does not exist yet in I2 commit 5a (temporary P3 shim only,
// vehicle_instance.cpp) - added in commit 5b.
PowertrainDesc read_powertrain(const json& j) {
    PowertrainDesc p;
    if (j.is_null()) return p;

    if (j.contains("components")) {
        std::vector<std::string> ids;
        for (const auto& cj : j.at("components")) {
            if (!cj.contains("id") || !cj.at("id").is_string() || cj.at("id").get<std::string>().empty()) {
                throw std::runtime_error("powertrain.components[]: 'id' is required and must be a non-empty string");
            }
            const std::string id = cj.at("id").get<std::string>();
            if (std::find(ids.begin(), ids.end(), id) != ids.end()) {
                throw std::runtime_error("powertrain.components[]: duplicate id '" + id + "'");
            }
            ids.push_back(id);
            if (!cj.contains("type") || !cj.at("type").is_string()) {
                throw std::runtime_error("powertrain.components['" + id + "']: 'type' is required");
            }
            const std::string type = cj.at("type").get<std::string>();

            if (type == "engine") {
                if (cj.value("kind", std::string()) != "torque_map") {
                    throw std::runtime_error("powertrain.components['" + id + "']: engine 'kind' must be \"torque_map\" (I2's only kind)");
                }
                ps::drivetrain::TorqueMapEngineDesc e;
                if (!cj.contains("wot_torque_nm_vs_rpm")) {
                    throw std::runtime_error("powertrain.components['" + id + "']: 'wot_torque_nm_vs_rpm' is required");
                }
                e.wot_torque_nm_vs_rpm = read_curve(cj.at("wot_torque_nm_vs_rpm"));
                if (e.wot_torque_nm_vs_rpm.x.empty()) {
                    throw std::runtime_error("powertrain.components['" + id + "']: 'wot_torque_nm_vs_rpm' must be non-empty");
                }
                e.inertia_kgm2 = cj.value("inertia_kgm2", e.inertia_kgm2);
                if (e.inertia_kgm2 < real(0.0)) {
                    throw std::runtime_error("powertrain.components['" + id + "']: inertia_kgm2 must be >= 0");
                }
                e.idle_rpm = cj.value("idle_rpm", e.idle_rpm);
                if (e.idle_rpm <= real(0.0)) {
                    throw std::runtime_error("powertrain.components['" + id + "']: idle_rpm must be > 0");
                }
                if (cj.contains("limiter")) {
                    const auto& lj = cj.at("limiter");
                    e.limiter.type = lj.value("type", std::string("hard_cut"));
                    if (e.limiter.type != "hard_cut") {
                        throw std::runtime_error("powertrain.components['" + id + "']: limiter.type must be \"hard_cut\" (I2's only type)");
                    }
                    e.limiter.rpm = lj.value("rpm", e.limiter.rpm);
                }
                if (e.limiter.rpm <= e.idle_rpm) {
                    throw std::runtime_error("powertrain.components['" + id + "']: limiter.rpm must be > idle_rpm");
                }
                p.components.push_back({id, e});
            } else if (type == "gear_pair") {
                ps::drivetrain::GearPairDesc g;
                g.ratio = cj.value("ratio", g.ratio);
                if (g.ratio == real(0.0)) {
                    throw std::runtime_error("powertrain.components['" + id + "']: gear_pair 'ratio' must be != 0");
                }
                g.efficiency = cj.value("efficiency", g.efficiency);
                if (g.efficiency <= real(0.0) || g.efficiency > real(1.0)) {
                    throw std::runtime_error("powertrain.components['" + id + "']: gear_pair 'efficiency' must be in (0, 1]");
                }
                g.inertia_in_kgm2 = cj.value("inertia_in_kgm2", g.inertia_in_kgm2);
                g.inertia_out_kgm2 = cj.value("inertia_out_kgm2", g.inertia_out_kgm2);
                p.components.push_back({id, g});
            } else if (type == "differential") {
                ps::drivetrain::DifferentialDesc d;
                if (cj.value("kind", std::string()) != "open") {
                    throw std::runtime_error("powertrain.components['" + id + "']: differential 'kind' must be \"open\" (I4 adds others)");
                }
                d.kind = ps::drivetrain::DifferentialKind::Open;
                d.ratio = cj.value("ratio", d.ratio);
                if (d.ratio == real(0.0)) {
                    throw std::runtime_error("powertrain.components['" + id + "']: differential 'ratio' must be != 0");
                }
                d.efficiency = cj.value("efficiency", d.efficiency);
                if (d.efficiency != real(1.0)) {
                    throw std::runtime_error("powertrain.components['" + id + "']: differential 'efficiency' must be exactly 1 in I2 (I4 adds losses)");
                }
                p.components.push_back({id, d});
            } else {
                throw std::runtime_error("powertrain.components['" + id + "']: unknown type \"" + type + "\"");
            }
        }
    }

    if (j.contains("shafts")) {
        for (const auto& sj : j.at("shafts")) {
            ps::drivetrain::ShaftDesc s;
            if (!sj.contains("ports")) throw std::runtime_error("powertrain.shafts[]: 'ports' is required");
            for (const auto& pj : sj.at("ports")) {
                const std::string ref = pj.get<std::string>();
                ps::drivetrain::PortRef pr;
                if (ref.rfind("wheel:", 0) == 0) {
                    pr.is_wheel = true;
                    pr.component = ref.substr(6);
                    if (pr.component.empty()) throw std::runtime_error("powertrain.shafts[]: bad port ref \"" + ref + "\"");
                } else {
                    const auto dot = ref.find('.');
                    if (dot == std::string::npos || dot == 0 || dot + 1 == ref.size()) {
                        throw std::runtime_error("powertrain.shafts[]: bad port ref \"" + ref +
                                                  "\" (expected \"<id>.<port>\" or \"wheel:<name>\")");
                    }
                    pr.component = ref.substr(0, dot);
                    pr.port = ref.substr(dot + 1);
                }
                s.ports.push_back(pr);
            }
            s.stiffness = sj.value("stiffness", real(0.0));
            if (s.stiffness != real(0.0)) {
                throw std::runtime_error("powertrain.shafts[]: 'stiffness' must be 0 in I2 (I4 adds compliant shafts)");
            }
            s.damping = sj.value("damping", real(0.0));
            p.shafts.push_back(s);
        }
    }

    if (j.contains("compliance")) {
        const std::string c = j.at("compliance").get<std::string>();
        if (c != "rigid") throw std::runtime_error("powertrain.compliance must be \"rigid\" in I2 (I4 adds \"compliant\"), got \"" + c + "\"");
        p.compliance = ps::drivetrain::Compliance::Rigid;
    }
    return p;
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
    v.powertrain = read_powertrain(j.value("powertrain", json()));
    return v;
}

} // namespace ps::io
