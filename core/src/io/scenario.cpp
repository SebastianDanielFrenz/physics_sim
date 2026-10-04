#include "ps/io/scenario.h"

#include "ps/environment/climate_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/terrain/procedural_terrain.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <filesystem>
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
    if (type == "cylinder") {
        return CylinderShape{j.value("half_height", real(0.5)), j.value("radius", real(0.3))};
    }
    throw std::runtime_error("unknown shape type '" + type + "' (expected box|sphere|capsule|cylinder)");
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

// --- PLAN.md D10/P1.5a: articulations ---------------------------------

Pose read_pose(const json& j) {
    Pose p;
    if (j.is_null()) return p;
    p.position = read_vec3(j.value("position", json()));
    p.orientation = read_quat(j.value("orientation", json()));
    return p;
}

Mat3 read_inertia_diag(const json& j) {
    const Vec3 d = read_vec3(j, Vec3::zero());
    return Mat3::diagonal(d.x, d.y, d.z);
}

articulation::JointType read_articulation_joint_type(const std::string& s) {
    if (s == "revolute" || s.empty()) return articulation::JointType::Revolute;
    if (s == "prismatic") return articulation::JointType::Prismatic;
    if (s == "fixed") return articulation::JointType::Fixed;
    throw std::runtime_error("unknown articulation joint type '" + s + "' (expected revolute|prismatic|fixed)");
}

// PLAN.md P1.5b: per-link contact opt-in (ps::articulation::LinkContactDesc).
// Absent "contact" object == disabled, matching LinkContactDesc's own
// default (every pre-P1.5b scenario file is unaffected).
articulation::LinkContactDesc read_link_contact(const json& j) {
    articulation::LinkContactDesc lc;
    lc.enabled = true; // presence of the "contact" object itself means enabled
    if (j.contains("shape")) lc.shape = read_shape(j.at("shape"));
    lc.shape_offset = read_pose(j.value("shape_offset", json()));
    if (j.contains("material")) {
        const json& m = j.at("material");
        lc.material.stiffness = m.value("stiffness", lc.material.stiffness);
        lc.material.stiffness_exponent = m.value("stiffness_exponent", lc.material.stiffness_exponent);
        lc.material.damping = m.value("damping", lc.material.damping);
        lc.material.friction = m.value("friction", lc.material.friction);
        lc.material.friction_regularization_speed = m.value("friction_regularization_speed", lc.material.friction_regularization_speed);
        lc.material.proxy_mass_scale = m.value("proxy_mass_scale", lc.material.proxy_mass_scale);
    }
    return lc;
}

articulation::LinkDesc read_link(const json& j) {
    articulation::LinkDesc ld;
    ld.parent = j.value("parent", -1);
    ld.joint = read_articulation_joint_type(j.value("joint", std::string("revolute")));
    ld.joint_frame_in_parent = read_pose(j.value("joint_frame_in_parent", json()));
    ld.axis = read_vec3(j.value("axis", json::array({0, 0, 1})));
    ld.mass = j.value("mass", real(1.0));
    ld.com = read_vec3(j.value("com", json()));
    ld.inertia = read_inertia_diag(j.value("inertia_diag", json::array({0, 0, 0})));
    ld.joint_damping = j.value("joint_damping", real(0.0));
    if (j.contains("limit_min") || j.contains("limit_max")) {
        ld.limits.enabled = true;
        ld.limits.min = j.value("limit_min", real(0.0));
        ld.limits.max = j.value("limit_max", real(0.0));
    }
    if (j.contains("velocity_actuator")) {
        const json& va = j.at("velocity_actuator");
        ld.velocity_actuator.enabled = true;
        ld.velocity_actuator.target_speed = va.value("target_speed", real(0.0));
        ld.velocity_actuator.max_force = va.value("max_force", real(0.0));
    }
    ld.q0 = j.value("q0", real(0.0));
    ld.qd0 = j.value("qd0", real(0.0));
    if (j.contains("contact")) ld.contact = read_link_contact(j.at("contact"));
    return ld;
}

// PLAN.md P1.5b: ArticulationDesc::contact_strategy ("none" default, matches
// every pre-P1.5b scenario file's behaviour unchanged).
articulation::ContactStrategy read_contact_strategy(const std::string& s) {
    if (s == "none" || s.empty()) return articulation::ContactStrategy::None;
    if (s == "compliant") return articulation::ContactStrategy::Compliant;
    if (s == "proxy_body") return articulation::ContactStrategy::ProxyBody;
    throw std::runtime_error("unknown contact_strategy '" + s + "' (expected none|compliant|proxy_body)");
}

articulation::LoopClosureDesc read_loop_closure(const json& j) {
    articulation::LoopClosureDesc lc;
    const std::string type = j.value("type", std::string("point"));
    lc.type = (type == "hinge") ? articulation::LoopClosureType::Hinge : articulation::LoopClosureType::Point;
    lc.link_a = j.value("link_a", -1);
    lc.link_b = j.value("link_b", -1);
    lc.anchor_a = read_vec3(j.value("anchor_a", json()));
    lc.anchor_b = read_vec3(j.value("anchor_b", json()));
    lc.axis_a = read_vec3(j.value("axis_a", json::array({0, 0, 1})));
    lc.axis_b = read_vec3(j.value("axis_b", json::array({0, 0, 1})));
    return lc;
}

ScenarioArticulation read_articulation(const json& j) {
    ScenarioArticulation sa;
    sa.name = j.at("name").get<std::string>();
    const json& root = j.value("root", json::object());
    sa.desc.root.floating = root.value("floating", false);
    sa.desc.root.pose = read_pose(root.value("pose", json()));
    sa.desc.root.velocity.linear = read_vec3(root.value("linear_velocity", json()));
    sa.desc.root.velocity.angular = read_vec3(root.value("angular_velocity", json()));
    sa.desc.root.mass = root.value("mass", real(1.0));
    sa.desc.root.com = read_vec3(root.value("com", json()));
    sa.desc.root.inertia = read_inertia_diag(root.value("inertia_diag", json::array({0, 0, 0})));

    if (j.contains("links")) {
        for (const auto& lj : j.at("links")) sa.desc.links.push_back(read_link(lj));
    }
    if (j.contains("loop_closures")) {
        for (const auto& cj : j.at("loop_closures")) sa.desc.loop_closures.push_back(read_loop_closure(cj));
    }
    sa.desc.contact_strategy = read_contact_strategy(j.value("contact_strategy", std::string("none")));
    return sa;
}

// PLAN.md M1: terrain source + a fixed set of interest points.
// scenario_dir is the scenario file's own directory, for resolving a
// relative spec_path (see ScenarioTerrain's doc comment).
ScenarioTerrain read_terrain(const json& j, const std::filesystem::path& scenario_dir) {
    ScenarioTerrain t;
    std::filesystem::path spec_path = j.at("spec_path").get<std::string>();
    if (spec_path.is_relative()) spec_path = scenario_dir / spec_path;
    t.spec_path = spec_path.lexically_normal().string();

    t.samples_per_tile = j.value("samples_per_tile", t.samples_per_tile);
    t.config.max_resident_tiles = j.value("max_resident_tiles", t.config.max_resident_tiles);
    t.config.max_tile_fills_per_tick = j.value("max_tile_fills_per_tick", t.config.max_tile_fills_per_tick);
    t.config.suppress_terrain_under_static_geometry =
        j.value("suppress_terrain_under_static_geometry", t.config.suppress_terrain_under_static_geometry);
    // Terrain prefetch ring (both default 0 == off, TerrainConfig's own
    // doc comment) - additive, no behaviour change for any scenario file
    // that does not set them.
    t.config.prefetch_margin_tiles = j.value("prefetch_margin_tiles", t.config.prefetch_margin_tiles);
    t.config.prefetch_max_tiles = j.value("prefetch_max_tiles", t.config.prefetch_max_tiles);

    if (j.contains("interest_points")) {
        for (const auto& ipj : j.at("interest_points")) {
            ScenarioTerrainInterestPoint ip;
            ip.id = ipj.at("id").get<std::uint64_t>();
            const auto xy = ipj.at("world_xy");
            ip.world_xy = Vec3{xy.at(0).get<real>(), xy.at(1).get<real>(), 0.0};
            ip.radius_m = ipj.at("radius_m").get<real>();
            t.interest_points.push_back(ip);
        }
    }
    return t;
}

// PLAN.md 5.1/P3: vehicle_json_path resolved relative to the scenario
// file's own directory, same convention as ScenarioTerrain::spec_path.
ScenarioVehicle read_vehicle(const json& j, const std::filesystem::path& scenario_dir, const EngineMapOptions& map_options) {
    ScenarioVehicle sv;
    sv.name = j.at("name").get<std::string>();
    sv.chassis_body_name = j.at("chassis_body").get<std::string>();
    sv.vehicle_json_path = j.at("vehicle_json_path").get<std::string>();
    std::filesystem::path p = sv.vehicle_json_path;
    if (p.is_relative()) p = scenario_dir / p;
    sv.vehicle_json_path = p.lexically_normal().string();
    sv.desc = load_vehicle_json(sv.vehicle_json_path, map_options);
    return sv;
}

// Weather plan W2: the scenario "environment" block (docs/data_formats/
// scenario.md). Rejects unknown keys and the keys of later increments (weather,
// surface_conditions, default_tyre_profile) so a scenario written for them
// never silently loads with half its meaning dropped.
ScenarioEnvironment read_environment(const json& j, const std::filesystem::path& scenario_dir) {
    const auto fail = [](const std::string& msg) { throw std::runtime_error("environment: " + msg); };
    if (!j.is_object()) fail("expected an object");
    for (const auto& [key, v] : j.items()) {
        (void)v;
        if (key == "weather" || key == "surface_conditions" || key == "default_tyre_profile")
            fail("key \"" + key + "\" is not supported yet (weather plan W3+)");
        if (key != "climate" && key != "seed" && key != "time" && key != "season" && key != "fixed_day_of_year" &&
            key != "north_yaw_deg" && key != "events")
            fail("unknown key \"" + key + "\"");
    }
    ScenarioEnvironment e;
    if (!j.contains("climate") || !j.at("climate").is_string()) fail("missing required string \"climate\"");
    std::filesystem::path cp = j.at("climate").get<std::string>();
    if (cp.is_relative()) cp = scenario_dir / cp;
    e.climate_path = cp.lexically_normal().string();
    e.climate = load_climate_json(e.climate_path);
    if (j.contains("seed")) e.seed = j.at("seed").get<std::uint64_t>();

    if (j.contains("time")) {
        const json& t = j.at("time");
        if (!t.is_object()) fail("\"time\" must be an object");
        for (const auto& [key, v] : t.items()) {
            (void)v;
            if (key != "start" && key != "scale") fail("unknown key \"time." + key + "\"");
        }
        if (t.contains("scale")) e.config.time_scale = t.at("scale").get<real>();
        if (t.contains("start")) {
            const json& st = t.at("start");
            if (!st.is_object()) fail("\"time.start\" must be an object");
            for (const auto& [key, v] : st.items()) {
                (void)v;
                if (key != "year" && key != "season" && key != "day" && key != "time_of_day_s")
                    fail("unknown key \"time.start." + key + "\"");
            }
            environment::EnvironmentStart start;
            start.year = st.value("year", std::int64_t(0));
            start.season = st.value("season", std::string());
            start.day = st.value("day", 0);
            start.time_of_day_s = st.value("time_of_day_s", real(0.0));
            e.config.start = start;
        }
    }
    if (j.contains("season")) {
        const std::string s = j.at("season").get<std::string>();
        if (s != "dynamic") e.config.fixed_season = s;
    }
    if (j.contains("fixed_day_of_year")) e.config.fixed_day_of_year = j.at("fixed_day_of_year").get<int>();
    if (j.contains("north_yaw_deg")) e.config.north_yaw_deg = j.at("north_yaw_deg").get<real>();

    if (j.contains("events")) {
        for (const auto& ej : j.at("events")) {
            if (!ej.is_object() || !ej.contains("t")) fail("each event needs an object with a \"t\"");
            ScenarioEnvironmentEvent ev;
            ev.time_s = ej.at("t").get<real>();
            int kinds = 0;
            for (const auto& [key, v] : ej.items()) {
                if (key == "t") continue;
                if (key == "jump_time_s") { ev.kind = ScenarioEnvironmentEvent::Kind::JumpTime; ev.value = v.get<real>(); ++kinds; }
                else if (key == "set_time_scale") { ev.kind = ScenarioEnvironmentEvent::Kind::SetTimeScale; ev.value = v.get<real>(); ++kinds; }
                else if (key == "set_weather") fail("event key \"set_weather\" is not supported yet (weather plan W3+)");
                else fail("unknown event key \"" + key + "\"");
            }
            if (kinds != 1) fail("each event needs exactly one of jump_time_s, set_time_scale");
            if (ev.kind == ScenarioEnvironmentEvent::Kind::SetTimeScale && !(ev.value >= 0.0 && std::isfinite(ev.value)))
                fail("set_time_scale must be >= 0 and finite");
            e.events.push_back(ev);
        }
        std::stable_sort(e.events.begin(), e.events.end(),
                         [](const ScenarioEnvironmentEvent& a, const ScenarioEnvironmentEvent& b) { return a.time_s < b.time_s; });
    }
    // Build the Environment once here so a bad config (unknown season, bad
    // scale, a start outside its day, ...) is a load-time error naming the
    // scenario, not a failure in build_scenario.
    try {
        environment::Environment probe(e.climate, e.config);
        (void)probe;
    } catch (const std::invalid_argument& ex) {
        fail(ex.what());
    }
    return e;
}

} // namespace

Scenario load_scenario(const std::string& json_path) { return load_scenario(json_path, EngineMapOptions{}); }

Scenario load_scenario(const std::string& json_path, const EngineMapOptions& map_options) {
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
        // PLAN.md P4 I0 (PHYS-033 section 9 / owner ruling Q7: strict
        // check, no legacy support). Every data/scenarios/*.json file (and
        // adapters/godot/demo/scenarios/m1_world.json) was given this field
        // when the check was added; there is no prior "physics_sim.scenario"
        // major to stay compatible with.
        {
            const std::string expected = "physics_sim.scenario/1";
            if (!j.contains("format") || !j.at("format").is_string()) {
                throw std::runtime_error(json_path + ": missing required 'format' field (expected \"" + expected + "\")");
            }
            const std::string actual = j.at("format").get<std::string>();
            if (actual != expected) {
                throw std::runtime_error(json_path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
            }
        }

        s.name = j.value("name", std::string("scenario"));
        s.duration_s = j.value("duration_s", real(1.0));
        s.world_config.tick_rate_hz = j.value("tick_rate_hz", real(240.0));
        s.world_config.substep_rate_hz = j.value("substep_rate_hz", real(960.0));
        s.world_config.gravity = read_vec3(j.value("gravity", json::array({0, 0, -9.81})));
        s.world_config.job_workers = j.value("job_workers", 0);
        s.world_config.max_bodies = j.value("max_bodies", std::size_t(4096));
        s.world_config.air_density = j.value("air_density", real(1.225));
        s.telemetry_csv = j.value("telemetry_csv", std::string());

        if (j.contains("bodies")) {
            for (const auto& bj : j.at("bodies")) s.bodies.push_back(read_body(bj));
        }
        if (j.contains("joints")) {
            for (const auto& jj : j.at("joints")) s.joints.push_back(read_joint(jj));
        }
        if (j.contains("articulations")) {
            for (const auto& aj : j.at("articulations")) s.articulations.push_back(read_articulation(aj));
        }
        if (j.contains("terrain")) {
            const std::filesystem::path scenario_dir = std::filesystem::path(json_path).parent_path();
            s.terrain = read_terrain(j.at("terrain"), scenario_dir);
        }
        if (j.contains("vehicles")) {
            const std::filesystem::path scenario_dir = std::filesystem::path(json_path).parent_path();
            for (const auto& vj : j.at("vehicles")) s.vehicles.push_back(read_vehicle(vj, scenario_dir, map_options));
        }
        if (j.contains("control_events")) {
            for (const auto& cj : j.at("control_events")) {
                ScenarioControlEvent ev;
                ev.time_s = cj.at("t").get<real>();
                ev.channel = cj.at("channel").get<std::string>();
                ev.value = cj.at("value").get<real>();
                s.control_events.push_back(ev);
            }
            std::sort(s.control_events.begin(), s.control_events.end(),
                      [](const ScenarioControlEvent& a, const ScenarioControlEvent& b) { return a.time_s < b.time_s; });
        }
        if (j.contains("environment")) {
            s.environment = read_environment(j.at("environment"), std::filesystem::path(json_path).parent_path());
        }
        if (j.contains("surface_table_path")) {
            std::filesystem::path p = j.at("surface_table_path").get<std::string>();
            if (p.is_relative()) p = std::filesystem::path(json_path).parent_path() / p;
            s.surface_table_path = p.lexically_normal().string();
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
    for (const auto& sa : scenario.articulations) {
        const ArticulationId id = built.world->create_articulation(sa.desc);
        built.articulations_by_name[sa.name] = id;
    }
    if (scenario.terrain) {
        auto source = terrain::load_procedural_terrain_source(scenario.terrain->spec_path, scenario.terrain->samples_per_tile);
        built.world->set_terrain_source(source, scenario.terrain->config);
        for (const auto& ip : scenario.terrain->interest_points) {
            built.world->set_terrain_interest_point(ip.id, ip.world_xy, ip.radius_m);
        }
    }
    if (!scenario.surface_table_path.empty()) {
        built.world->set_surface_table(std::make_shared<SurfaceTable>(scenario.surface_table_path));
    }
    if (scenario.environment) {
        built.world->install_environment(
            std::make_shared<environment::Environment>(scenario.environment->climate, scenario.environment->config));
    }
    for (const auto& sv : scenario.vehicles) {
        auto it = built.bodies_by_name.find(sv.chassis_body_name);
        if (it == built.bodies_by_name.end()) throw std::runtime_error("vehicle '" + sv.name + "' references unknown chassis_body '" + sv.chassis_body_name + "'");
        const VehicleId id = built.world->create_vehicle(sv.desc, it->second);
        built.vehicles_by_name[sv.name] = id;
    }
    return built;
}

std::size_t apply_environment_events(World& world, const Scenario& scenario, std::size_t& next_event) {
    if (!scenario.environment) return 0;
    const auto& events = scenario.environment->events;
    std::size_t applied = 0;
    while (next_event < events.size() && events[next_event].time_s <= world.sim_time()) {
        const ScenarioEnvironmentEvent& ev = events[next_event];
        switch (ev.kind) {
            case ScenarioEnvironmentEvent::Kind::JumpTime: {
                // The env time the NEXT step() will run at, plus the jump.
                const real now = world.environment()->clock().env_time(static_cast<std::int64_t>(world.tick()));
                world.set_environment_time(now + ev.value);
                break;
            }
            case ScenarioEnvironmentEvent::Kind::SetTimeScale:
                world.set_environment_time_scale(ev.value);
                break;
        }
        ++next_event;
        ++applied;
    }
    return applied;
}

} // namespace ps::io
