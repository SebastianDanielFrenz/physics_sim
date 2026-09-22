// ps/io/scenario.h — JSON scenario files (PLAN.md D6: "test scenarios are
// files, not code").
//
// nlohmann::json stays confined to scenario.cpp; this header only exposes
// plain structs, so parsing a scenario never forces a third-party type into
// a public signature elsewhere.

#pragma once

#include "ps/articulation/articulation_desc.h"
#include "ps/backend/i_rigid_backend.h"
#include "ps/world/world.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ps::io {

struct ScenarioBody {
    std::string name;
    BodyDesc desc;
};

struct ScenarioJoint {
    std::string name;
    JointDesc desc;
    std::string body_a_name;
    std::string body_b_name; // empty == anchored to the world
};

// Additive (PLAN.md D10/P1.5a): a scenario may describe one or more
// articulations the same way it describes bodies/joints.
struct ScenarioArticulation {
    std::string name;
    articulation::ArticulationDesc desc;
};

struct Scenario {
    std::string name;
    real duration_s = 1.0;
    WorldConfig world_config;
    std::vector<ScenarioBody> bodies;
    std::vector<ScenarioJoint> joints;
    std::vector<ScenarioArticulation> articulations;
    std::string telemetry_csv; // empty == "<name>.csv" next to the scenario file's caller (ps_run decides)
};

// Throws std::runtime_error with a descriptive message on a malformed or
// unreadable scenario file.
Scenario load_scenario(const std::string& json_path);

struct BuiltScenario {
    std::unique_ptr<World> world;
    std::unordered_map<std::string, BodyId> bodies_by_name;
    std::unordered_map<std::string, ArticulationId> articulations_by_name; // additive, PLAN.md D10/P1.5a
};

// Builds bodies/joints from a Scenario into a fresh World.
BuiltScenario build_scenario(const Scenario& scenario);

} // namespace ps::io
