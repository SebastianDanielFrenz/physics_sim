#include "ps/io/scenario.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

Vec3 read_vec3(const json& j, const Vec3& def = Vec3::zero()) {
    if (j.is_null()) return def;
    if (!j.is_array() || j.size() != 3) throw std::runtime_error("expected a 3-element array for a Vec3");
    return Vec3{j[0].get<real>(), j[1].get<real>(), j[2].get<real>()};
}

Quat read_quat(const json& j) {
    if (j.is_null()) return Quat::identity();
    if (!j.is_array() || j.size() != 4) throw std::runtime_error("expected a 4-element [x,y,z,w] array for a Quat");
    return Quat{j[0].get<real>(), j[1].get<real>(), j[2].get<real>(), j[3].get<real>()};
}

BodyMotionType read_motion(const std::string& s) {
    if (s == "static") return BodyMotionType::Static;
    if (s == "kinematic") return BodyMotionType::Kinematic;
    if (s == "dynamic" || s.empty()) return BodyMotionType::Dynamic;
    throw std::runtime_error("unknown body motion type '" + s + "' (expected static|kinematic|dynamic)");
}

ShapeDesc read_shape(const json& j) {
    const std::string type = j.at("type").get<std::string>();
    if (type == "box") {
        return BoxShape{read_vec3(j.value("half_extents", json()), Vec3{0.5, 0.5, 0.5})};
    }
    if (type == "sphere") {
        return SphereShape{j.value("radius", real(0.5))};
    }
    if (type == "capsule") {
        return CapsuleShape{j.value("half_height", real(0.5)), j.value("radius", real(0.25))};
    }
    throw std::runtime_error("unknown shape type '" + type + "' (expected box|sphere|capsule)");
}

ScenarioBody read_body(const json& j) {
    ScenarioBody b;
    b.name = j.at("name").get<std::string>();
    b.desc.shape = read_shape(j.at("shape"));
    b.desc.motion = read_motion(j.value("motion", std::string("dynamic")));
    b.desc.mass = j.value("mass", real(1.0));
    b.desc.pose.position = read_vec3(j.value("position", json()));
    b.desc.pose.orientation = read_quat(j.value("orientation", json()));
    b.desc.velocity.linear = read_vec3(j.value("linear_velocity", json()));
    b.desc.velocity.angular = read_vec3(j.value("angular_velocity", json()));
    b.desc.friction = j.value("friction", real(0.5));
    b.desc.restitution = j.value("restitution", real(0.0));
    b.desc.linear_damping = j.value("linear_damping", real(0.0));
    b.desc.angular_damping = j.value("angular_damping", real(0.0));
    b.desc.gravity_enabled = j.value("gravity_enabled", true);
    b.desc.allow_sleep = j.value("allow_sleep", false);
    return b;
}

ScenarioJoint read_joint(const json& j) {
    ScenarioJoint sj;
    sj.name = j.at("name").get<std::string>();
    const std::string type = j.value("type", std::string("fixed"));
    sj.desc.type = (type == "hinge") ? JointType::Hinge : JointType::Fixed;
    sj.body_a_name = j.at("body_a").get<std::string>();
    sj.body_b_name = j.value("body_b", std::string());
    sj.desc.anchor_a_local = read_vec3(j.value("anchor_a", json()));
    sj.desc.anchor_b_local = read_vec3(j.value("anchor_b", json()));
    sj.desc.axis_a_local = read_vec3(j.value("axis_a", json::array({0, 0, 1})));
    sj.desc.axis_b_local = read_vec3(j.value("axis_b", json::array({0, 0, 1})));
    sj.desc.friction_torque = j.value("friction_torque", real(0.0));
    sj.desc.limits_enabled = j.contains("limit_min") || j.contains("limit_max");
    sj.desc.limit_min_rad = j.value("limit_min", real(0.0));
    sj.desc.limit_max_rad = j.value("limit_max", real(0.0));
    return sj;
}

} // namespace

Scenario load_scenario(const std::string& json_path) {
    std::ifstream in(json_path);
    if (!in) throw std::runtime_error("cannot open scenario file: " + json_path);

    json j;
    try {
        in >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("malformed JSON in " + json_path + ": " + e.what());
    }

    Scenario s;
    try {
        s.name = j.value("name", std::string("scenario"));
        s.duration_s = j.value("duration_s", real(1.0));
        s.world_config.tick_rate_hz = j.value("tick_rate_hz", real(240.0));
        s.world_config.substep_rate_hz = j.value("substep_rate_hz", real(960.0));
        s.world_config.gravity = read_vec3(j.value("gravity", json::array({0, 0, -9.81})));
        s.world_config.job_workers = j.value("job_workers", 0);
        s.world_config.max_bodies = j.value("max_bodies", std::size_t(4096));
        s.telemetry_csv = j.value("telemetry_csv", std::string());

        if (j.contains("bodies")) {
            for (const auto& bj : j.at("bodies")) s.bodies.push_back(read_body(bj));
        }
        if (j.contains("joints")) {
            for (const auto& jj : j.at("joints")) s.joints.push_back(read_joint(jj));
        }
    } catch (const json::exception& e) {
        throw std::runtime_error("invalid scenario " + json_path + ": " + e.what());
    }
    return s;
}

BuiltScenario build_scenario(const Scenario& scenario) {
    BuiltScenario built;
    built.world = std::make_unique<World>(scenario.world_config);

    for (const auto& b : scenario.bodies) {
        const BodyId id = built.world->create_body(b.desc);
        built.bodies_by_name[b.name] = id;
    }
    for (const auto& sj : scenario.joints) {
        JointDesc desc = sj.desc;
        auto it_a = built.bodies_by_name.find(sj.body_a_name);
        if (it_a == built.bodies_by_name.end()) throw std::runtime_error("joint '" + sj.name + "' references unknown body_a '" + sj.body_a_name + "'");
        desc.body_a = it_a->second;
        if (!sj.body_b_name.empty()) {
            auto it_b = built.bodies_by_name.find(sj.body_b_name);
            if (it_b == built.bodies_by_name.end()) throw std::runtime_error("joint '" + sj.name + "' references unknown body_b '" + sj.body_b_name + "'");
            desc.body_b = it_b->second;
        }
        built.world->create_joint(desc);
    }
    return built;
}

} // namespace ps::io
