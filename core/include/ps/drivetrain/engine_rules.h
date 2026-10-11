// ps/drivetrain/engine_rules.h - engine garage P4 (racing_game out/engine_garage_design.md A10-A12):
//   A10 check_buildable      static buildability rules: which engines the garage refuses BEFORE a map job starts, with a
//                            reason, a source label and a JSON pointer to the field to change;
//   A11 classify_engine_change  what an edit costs: nothing, a runtime rebuild, turbo tables, map generation, a respawn;
//   A12 grid helpers         the rpm / manifold-pressure axes after the limiter is raised, a turbo is added or one is removed.
//
// Pure functions of descriptors; no I/O, no allocation guarantees (garage use, never inside World::step).
// docs/data_formats/engine.md "Buildability rules" lists every number and its source.
#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::drivetrain {

// ---------------------------------------------------------------------------------------------------------------------------
// A10: check_buildable
// ---------------------------------------------------------------------------------------------------------------------------

// Stable, append-only values (a game may switch on them or store them): never renumber, never reuse.
enum class BuildRule : std::uint16_t {
    ValveLayout = 0,           // intake x exhaust valves per cylinder is not a production layout
    ValveDiameterBand = 1,     // intake valve head diameter over bore outside the band of its layout
    ValvePackingRow = 2,       // the valves of one side do not fit across the bore
    ValvePackingArea = 3,      // all valve heads together exceed the densest packing in the bore circle
    ExhaustIntakeRatio = 4,    // exhaust / intake valve diameter outside 0.78-0.90
    RodRatio = 5,              // rod length / crank radius outside 3.0-4.2
    MeanPistonSpeed = 6,       // mean piston speed at the limiter
    CompressionRatio = 7,      // outside the band of its aspiration (forced induction <= 10.5)
    LimiterAboveRpmGrid = 8,   // the limiter is above the last rpm node of the generated maps
    BoostAbovePressureGrid = 9, // ambient + boost target is above the last manifold-pressure node
    LimiterAboveCurve = 10,    // torque-map engine: the limiter is above the last rpm of its torque curves
};

// A violation of a static rule. severity reuses P3b's LimitSeverity so one advisor handles both record kinds:
//   Exceeded - the engine cannot be built as it stands (the garage refuses to save or drive it);
//   Limiting - it can be built, but this rule caps the result (e.g. held map rows well past a grid edge);
//   Near     - worth telling the player (unusual for a road engine, a small clamp), never blocks.
struct BuildIssue {
    BuildRule rule = BuildRule::ValveLayout;
    LimitSeverity severity = LimitSeverity::Near;
    // Which part file the field lives in, in physics_sim's slot vocabulary ("engine" or "turbo_configuration").
    std::string slot;
    // RFC 6901 JSON pointer into THAT file ("/cycle/intake_valve_d_mm", "/limiter/rpm", "/target_boost_bar").
    std::string field_pointer;
    // Player-facing sentence with the numbers ("intake valves 41 mm do not fit an 86 mm bore with 4 valves: max 37.2 mm").
    std::string message;
    // The rule's source label as in engine_rules_table.h: "[AS] ...", "[D] ...", "[C] ..." (never empty).
    std::string source_label;
    real value = 0.0; // the offending number, in `unit`
    real limit = 0.0; // the bound it is held against
    LimitUnit unit = LimitUnit::Dimensionless;
};

// All issues, ordered by rule (BuildRule value), then by field pointer. An empty vector = nothing found.
// Which rules apply to which engine kind:
//   simulated + cycle block : every rule (valves, packaging, rod, CR, piston speed, grids, boost vs grid);
//   simulated, no cycle     : piston speed (bore/stroke are in the desc) only - its maps are measured data, whose axes the
//                             player cannot edit, so grid coverage is not a build rule there (toyota_a25a_fks_sim's VE axis ends
//                             at 5761 rpm below its 6500 rpm limiter and 97.5 kPa below ambient; P3b reports it per dyno point);
//   torque map              : the limiter against the last rpm of its torque curves (there is no geometry to check).
// Other component kinds: nothing applies, an empty vector.
[[nodiscard]] std::vector<BuildIssue> check_buildable(const ComponentParams& engine);
[[nodiscard]] std::vector<BuildIssue> check_buildable(const SimulatedEngineDesc& engine);
[[nodiscard]] std::vector<BuildIssue> check_buildable(const TorqueMapEngineDesc& engine);

// True iff any issue is Exceeded (the garage must refuse).
[[nodiscard]] bool has_blocking_issue(const std::vector<BuildIssue>& issues);

[[nodiscard]] const char* to_string(BuildRule r);

// ---------------------------------------------------------------------------------------------------------------------------
// A11: classify_engine_change
// ---------------------------------------------------------------------------------------------------------------------------

// What an edit costs, ascending (the most expensive class of all changed fields wins; compare with <).
//   None        - no difference that any consumer reads (identical descriptors, or only provenance text).
//   Runtime     - applies by constructing the engine again from the descriptor (steady tables ~68 ms, no map generation).
//   TurboTables - the turbo's derived tables are keyed differently (a few ms, cached); the engine maps are untouched.
//   Maps        - engine_map_cache_key moves: the .psmaps must be regenerated (seconds to minutes; the only class that starts
//                 background generation).
//   Respawn     - a different engine: kind, architecture, ignition type or cylinder count that no map key covers. Nothing can
//                 be rebuilt from the old one.
enum class EngineChange : std::uint8_t { None = 0, Runtime = 1, TurboTables = 2, Maps = 3, Respawn = 4 };

// One changed field group: `pointer` is an RFC 6901 JSON pointer into the part file named by `slot`.
struct EngineFieldChange {
    std::string slot;    // "engine" or "turbo_configuration"
    std::string pointer; // e.g. "/limiter/rpm"; "/cycle" for a change that moves the map key
    EngineChange change = EngineChange::Runtime;
};

// Every changed field group, in table order. The cycle block is reported as ONE "/cycle" entry when
// engine_map_cache_key differs (so the classification cannot drift from the cache) and the turbo as one
// "/target_boost_bar..." style entry per changed group; see engine_rules.cpp's table.
[[nodiscard]] std::vector<EngineFieldChange> list_engine_changes(const ComponentParams& before, const ComponentParams& after);

// The maximum over list_engine_changes (None when the list is empty). Derivation:
//   Maps        := engine_map_cache_key(effective cycle, fuels[0], Full) differs (or a cycle block appears/disappears), where
//                  the effective cycle has intake->compressor_supply = turbo_pair.has_value() exactly as the loader sets it;
//   TurboTables := the turbo's table key differs (turbo_table_key below), or a turbo appears/disappears;
//   Respawn     := the component alternative, architecture or ignition differs, or the cylinder count differs where no cycle
//                  block carries it in the map key;
//   Runtime     := any other difference; None := no difference.
// Both arguments must be an engine (TorqueMapEngineDesc or SimulatedEngineDesc); another alternative throws std::invalid_argument.
[[nodiscard]] EngineChange classify_engine_change(const ComponentParams& before, const ComponentParams& after);
[[nodiscard]] EngineChange classify_engine_change(const SimulatedEngineDesc& before, const SimulatedEngineDesc& after);
[[nodiscard]] EngineChange classify_engine_change(const TorqueMapEngineDesc& before, const TorqueMapEngineDesc& after);

[[nodiscard]] const char* to_string(EngineChange c);

// The key of a turbo's derived tables, the same FNV-1a over the same parameter list as ps::io::load_turbo_configuration_json
// (core/src/io/vehicle_io.cpp), computed from the descriptor: for a descriptor straight from that loader (no performance_map)
// it equals desc.maps->content_hash (tests/unit/test_engine_rules.cpp guards this, so a new key field there fails a test here).
// A performance_map's file hash is part of the loader's key; here it is compared separately through maps->content_hash.
[[nodiscard]] std::uint64_t turbo_table_key(const TurboPairDesc& desc);

// ---------------------------------------------------------------------------------------------------------------------------
// A12: grid helpers
// ---------------------------------------------------------------------------------------------------------------------------

// Manifold-pressure axis [Pa] for an engine that gets a turbo with the given gauge target (bar): the desc's nodes at or below
// ambient (ISA sea level, isa_air.h), then rows above ambient spaced 60 kPa (the hyper's pitch) up to (1 + target + 0.2) bar
// absolute, the last step 30..90 kPa. The hyper's 100 kPa node followed by 160 / 220 / 300 kPa is exactly boost_grid_p_im for
// a 1.8 bar target on its own stock part. Throws std::invalid_argument for target <= 0 or fewer than two nodes at/below ambient.
[[nodiscard]] std::vector<real> boost_grid_p_im(const CycleModelDesc& desc, real target_boost_bar);

// The axis without nodes above ambient (a node above ambient fails the load without a turbo). The highest node at or below
// ambient is kept, so wide-open-throttle still has a row. Throws std::invalid_argument if fewer than two nodes remain.
[[nodiscard]] std::vector<real> ambient_grid_p_im(const CycleModelDesc& desc);

// The rpm axis extended so its last node reaches limiter_rpm: nodes continue at the last interval until within half an interval
// of the limiter, then the limiter itself is the last node. Unchanged if the axis already covers the limiter. Throws
// std::invalid_argument for fewer than two nodes.
[[nodiscard]] std::vector<real> extend_grid_rpm(const CycleModelDesc& desc, real limiter_rpm);

} // namespace ps::drivetrain
