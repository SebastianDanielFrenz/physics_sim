// core/src/io/engine_parts.h - INTERNAL (engine garage P2, out/garage_p2_progress.md):
// part files with "extends", engine slots and their assembly into one engine
// JSON object. Used by vehicle_io.cpp and nitrous_io.cpp only; nlohmann::json
// stays confined to core/src/io.
//
// Resolution order of an engine (docs/data_formats/engine.md "Part slots"):
//   1. the engine file's "extends" chain (deepest parent first, the file last);
//   2. the engine's inline content;
//   3. the engine's default slots (default_ecu, default_cam,
//      default_forced_induction, cycle.default_installation, default_thermal),
//      relative to the file that wrote them;
//   4. the override layer: a vehicle engine component's slots (ecu, cam,
//      forced_induction, installation, thermal; relative to the vehicle file),
//      which the user layer (VehicleLoadOptions::part_overrides) rewrites
//      before the component is read, so the user wins.
// The highest level that names a slot picks THE file for that slot. An ECU
// or cam file is then merge-patched (RFC 7396) over the engine's own content
// of that part, so the existing engine reader, every rejection and every cache
// key see what an inline engine would give.

#pragma once

#include "ps/environment/json_extends.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>
#include <string>

namespace ps::io::parts {

using json = nlohmann::json;

// Path-valued keys per part format (relative paths per file, D3). A new
// path-valued key in one of these formats must be added to its function.
void rebase_engine_paths(json& j, const std::filesystem::path& from_dir, const std::filesystem::path& to_dir);
void rebase_turbo_configuration_paths(json& j, const std::filesystem::path& from_dir, const std::filesystem::path& to_dir);
void rebase_turbo_hardware_paths(json& j, const std::filesystem::path& from_dir, const std::filesystem::path& to_dir);
void rebase_n2o_bottle_paths(json& j, const std::filesystem::path& from_dir, const std::filesystem::path& to_dir);
void rebase_nitrous_kit_paths(json& j, const std::filesystem::path& from_dir, const std::filesystem::path& to_dir);

// Reads a part file and resolves its "extends" chain (depth 8, cycles
// rejected); `rebase` = the format's path rebase (empty for formats without
// path keys).
json load_part(const std::string& path, const ExtendsRebase& rebase = {});

// The engine slot names, in documentation order.
inline constexpr const char* kEngineSlots[] = {"ecu", "cam", "forced_induction", "installation", "thermal"};
bool is_engine_slot(const std::string& name);

// Slot -> value: a resolved path, or "none" (forced_induction only).
using SlotMap = std::map<std::string, std::string>;

// Reads a vehicle engine component's slot keys (resolved against base_dir).
// `inline_engine`: the component carries the engine itself (no "ref"); its
// "ecu" OBJECT is then the engine's inline ECU block, not a slot. Rejects the
// renamed "turbo_configuration" by name and a non-string / empty slot value.
SlotMap read_component_slots(const json& cj, const std::string& base_dir, const std::string& ctx, bool inline_engine);

// Removes the slot keys read_component_slots read from an inline engine
// component (so the engine reader never sees them).
void erase_component_slots(json& cj);

struct AssembledEngine {
    json j;                               // engine object, ECU/cam parts merged, slot keys removed
    bool has_forced_induction = false;    // a turbo configuration path won
    std::string forced_induction_path;    // the winning configuration (has_forced_induction)
    bool forced_induction_none = false;   // an explicit "none" won
    std::string forced_induction_origin;  // for messages: "default_forced_induction" / "forced_induction"
    std::string installation_path;        // "" = the engine's own cycle.default_installation
    std::string thermal_path;             // "" = no thermal part
};

// Steps 3-4 above on an engine object whose "extends" chain is already
// resolved. `overrides` = the vehicle/user layer (read_component_slots), or
// the standalone engine loader's slots.
AssembledEngine assemble_engine(json engine, const std::string& engine_dir, const std::string& ctx, const SlotMap& overrides);

} // namespace ps::io::parts
