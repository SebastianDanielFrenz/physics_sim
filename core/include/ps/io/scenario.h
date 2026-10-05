// ps/io/scenario.h — JSON scenario files (PLAN.md D6: "test scenarios are
// files, not code").
//
// nlohmann::json stays confined to scenario.cpp; this header only exposes
// plain structs, so parsing a scenario never forces a third-party type into
// a public signature elsewhere.

#pragma once

#include "ps/articulation/articulation_desc.h"
#include "ps/backend/i_rigid_backend.h"
#include "ps/environment/climate_desc.h"
#include "ps/environment/environment.h"
#include "ps/terrain/terrain_config.h"
#include "ps/vehicle/vehicle_desc.h"
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

// Additive (PLAN.md 5.1/P3): a scenario may describe one or more vehicles
// on top of an already-described body (the chassis). vehicle_json_path
// (resolved relative to the scenario file's own directory, same convention
// as ScenarioTerrain::spec_path) is loaded via ps::io::load_vehicle_json;
// leave it empty and fill `desc` directly when building a scenario in
// code (e.g. tests) instead of from a vehicle JSON file.
struct ScenarioVehicle {
    std::string name;
    std::string chassis_body_name;
    std::string vehicle_json_path;
    vehicle::VehicleDesc desc;
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

// Additive (PLAN.md P3 scope item 7: "control inputs from the scenario
// file: e.g. a step-steer or a braking test"): World::set_control(channel,
// value) applied once sim_time reaches time_s. ps_run (and any other
// scenario-driving tool) applies these in time order as it steps; a
// scenario with no control_events behaves exactly as before this field
// existed (every control channel reads as its World default, 0).
struct ScenarioControlEvent {
    real time_s = 0.0;
    std::string channel;
    real value = 0.0;
};

// Weather plan W2 (docs/data_formats/scenario.md "environment" block): an
// environment (clock + calendar + sun for now) installed into the World by
// build_scenario. The climate file is loaded - and any error surfaced - by
// load_scenario. A scenario without the block installs none and behaves
// exactly as before this field existed.
struct ScenarioEnvironmentEvent {
    enum class Kind : std::uint8_t {
        JumpTime,      // advance environment time by `value` env seconds (negative = back) at the event time
        SetTimeScale,  // set the time scale to `value` (>= 0) without a time jump
        // W3b (owner answer Q12), all need the environment's "weather" key:
        SetWeather,    // preset override from now to the end of the current season (`preset`, ramp `value` s)
        EndWeather,    // soft end of every override (each ramps out over its own ramp)
        ClearWeather,  // immediate removal of every override
        SetSeason      // season lock: `text` = a season name, or "dynamic" to unlock
    };
    real time_s = 0.0;  // SIM time at which the event applies
    Kind kind = Kind::JumpTime;
    real value = 0.0;
    environment::WeatherPresetDesc preset;  // SetWeather
    std::string text;                       // SetSeason
};

struct ScenarioEnvironment {
    std::string climate_path;                    // resolved, informational
    std::string surface_conditions_path;         // W5: resolved, informational (empty = none)
    environment::ClimateDesc climate;
    environment::EnvironmentConfig config;
    std::uint64_t seed = 0;                      // accepted now, used by the weather timeline (W3)
    std::vector<ScenarioEnvironmentEvent> events; // kept sorted by time_s (stable) by load_scenario
};

struct Scenario {
    std::string name;
    real duration_s = 1.0;
    WorldConfig world_config;
    std::vector<ScenarioBody> bodies;
    std::vector<ScenarioJoint> joints;
    std::vector<ScenarioArticulation> articulations;
    std::vector<ScenarioControlEvent> control_events; // kept sorted by time_s by load_scenario
    std::vector<ScenarioVehicle> vehicles;
    std::optional<ScenarioTerrain> terrain;
    // PLAN.md section 6/P3: shared surface-parameter lookup (lambda_mu,
    // crr) for every vehicle's tyre model - resolved relative to the
    // scenario file's own directory if not already absolute, same
    // convention as ScenarioTerrain::spec_path. Empty == no SurfaceTable
    // installed (every surface reads as lambda_mu=1, crr=0).
    std::string surface_table_path;
    std::optional<ScenarioEnvironment> environment; // weather plan W2
    std::string telemetry_csv; // empty == "<name>.csv" next to the scenario file's caller (ps_run decides)
};

// Throws std::runtime_error with a descriptive message on a malformed or
// unreadable scenario file.
Scenario load_scenario(const std::string& json_path);
// PHYS-033 P4 I7c C16a: the same, with every vehicle loaded through
// load_vehicle_json(path, map_options) (engine-map generation quality/pool,
// ps/io/vehicle_io.h). The overload above uses the defaults.
struct EngineMapOptions;
Scenario load_scenario(const std::string& json_path, const EngineMapOptions& map_options);

struct BuiltScenario {
    std::unique_ptr<World> world;
    std::unordered_map<std::string, BodyId> bodies_by_name;
    std::unordered_map<std::string, ArticulationId> articulations_by_name; // additive, PLAN.md D10/P1.5a
    std::unordered_map<std::string, VehicleId> vehicles_by_name;           // additive, PLAN.md 5.1/P3
};

// Builds bodies/joints from a Scenario into a fresh World.
BuiltScenario build_scenario(const Scenario& scenario);

// Applies every scenario environment event whose time_s <= world.sim_time(), in
// order, starting at `next_event` (advanced past the applied ones); returns how
// many were applied. The scenario-driving tools (ps_run, the determinism
// tests) call this each tick right before World::step(), next to their
// control_events loop. A scenario without an environment applies nothing.
std::size_t apply_environment_events(World& world, const Scenario& scenario, std::size_t& next_event);

} // namespace ps::io
