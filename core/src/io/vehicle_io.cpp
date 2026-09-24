#include "ps/io/vehicle_io.h"

#include "../vehicle/tir_parser.h"

#include <nlohmann/json.hpp>

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
using ps::vehicle::BrakeDesc;
using ps::vehicle::DriveDesc;

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
    w.driven = j.value("driven", w.driven);
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

BrakeDesc read_brakes(const json& j) {
    BrakeDesc b;
    if (j.is_null()) return b;
    b.max_torque_front = j.value("max_torque_front", b.max_torque_front);
    b.max_torque_rear = j.value("max_torque_rear", b.max_torque_rear);
    b.handbrake_rear_fraction = j.value("handbrake_rear_fraction", b.handbrake_rear_fraction);
    return b;
}

DriveDesc read_drive(const json& j) {
    DriveDesc d;
    if (j.is_null()) return d;
    if (j.contains("torque_curve_nm_vs_rpm")) d.torque_curve_nm_vs_rpm = read_curve(j.at("torque_curve_nm_vs_rpm"));
    d.final_drive_ratio = j.value("final_drive_ratio", d.final_drive_ratio);
    d.rev_limit_rpm = j.value("rev_limit_rpm", d.rev_limit_rpm);
    d.idle_rpm = j.value("idle_rpm", d.idle_rpm);
    d.drivetrain_efficiency = j.value("drivetrain_efficiency", d.drivetrain_efficiency);
    return d;
}

} // namespace

vehicle::TyreDesc load_tyre_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "tyre", 1, path);
    return read_tyre_object(j);
}

vehicle::VehicleDesc load_vehicle_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "vehicle", 1, path);
    const std::string base_dir = std::filesystem::path(path).parent_path().string();

    VehicleDesc v;
    v.name = j.value("name", std::string());
    v.source = j.value("source", std::string());
    if (v.source.empty()) throw std::runtime_error("vehicle '" + v.name + "': 'source' is required (PLAN.md section 8)");

    if (!j.contains("wheels") || !j.at("wheels").is_array() || j.at("wheels").empty()) {
        throw std::runtime_error("vehicle " + path + " has no 'wheels' array");
    }
    for (const auto& wj : j.at("wheels")) v.wheels.push_back(read_wheel(wj, base_dir));

    v.steering = read_steering(j.value("steering", json()));
    v.brakes = read_brakes(j.value("brakes", json()));
    v.drive = read_drive(j.value("drive", json()));
    if (v.drive.torque_curve_nm_vs_rpm.x.empty()) {
        throw std::runtime_error("vehicle " + path + ": drive.torque_curve_nm_vs_rpm is required (PLAN.md M1 placeholder drive)");
    }
    return v;
}

} // namespace ps::io
