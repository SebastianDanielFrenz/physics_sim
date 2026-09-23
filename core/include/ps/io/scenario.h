// ps/io/scenario.h — JSON scenario files (PLAN.md D6: "test scenarios are
// files, not code").
//
// nlohmann::json stays confined to scenario.cpp; this header only exposes
// plain structs, so parsing a scenario never forces a third-party type into
// a public signature elsewhere.

#pragma once

#include "ps/articulation/articulation_desc.h"
#include "ps/backend/i_rigid_backend.h"
#include "ps/terrain/terrain_config.h"
#include "ps/world/world.h"

#include <cstdint>
#include <memory>
#include <optional>
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

// Additive (PLAN.md M1): a scenario may install a procedural terrain
// source plus a fixed set of interest points (a static stand-in for
// "vehicles"/camera - a real one is a runtime concept, not scenario data,
// but the determinism/hash-compare scenarios this backs only need a fixed
// set of streamed tiles to exercise the tile system, not moving ones).
struct ScenarioTerrainInterestPoint {
    std::uint64_t id = 0;
    Vec3 world_xy;
    real radius_m = 0.0;
};

struct ScenarioTerrain {
    // Resolved to an absolute path at load time, relative to the scenario
    // file's own directory if not already absolute (so a scenario is
    // relocatable and never depends on the caller's current directory -
    // same reasoning as test_scenario_articulation.cpp's PS_SOURCE_DIR use,
    // but resolved here instead since ps_run/tools/sweep have no such
    // compile-time constant).
    std::string spec_path;
    // Grid resolution for the source this scenario builds - kept separate
    // from TerrainConfig (see ps/terrain/terrain_config.h's header comment):
    // it is a property of the ITerrainSource, not of TileManager's pool
    // sizing, and build_scenario passes it straight into
    // load_procedural_terrain_source so the two can never disagree.
    std::uint32_t samples_per_tile = 64;
    terrain::TerrainConfig config;
    std::vector<ScenarioTerrainInterestPoint> interest_points;
};

struct Scenario {
    std::string name;
    real duration_s = 1.0;
    WorldConfig world_config;
    std::vector<ScenarioBody> bodies;
    std::vector<ScenarioJoint> joints;
    std::vector<ScenarioArticulation> articulations;
    std::optional<ScenarioTerrain> terrain;
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
