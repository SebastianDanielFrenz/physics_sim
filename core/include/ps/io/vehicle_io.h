// ps/io/vehicle_io.h — JSON loading for TyreDesc/VehicleDesc (PLAN.md D6:
// "vehicles ... are files, not code"). nlohmann::json stays confined to
// vehicle_io.cpp.

#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/vehicle/tyre_desc.h"
#include "ps/vehicle/vehicle_desc.h"

#include <cstddef>
#include <string>

namespace ps::io {

// Loads a tyre JSON file (data/tyres/*.json). Throws std::runtime_error on
// a malformed/unreadable file or a missing required field (name/source -
// PLAN.md section 8).
vehicle::TyreDesc load_tyre_json(const std::string& path);

// Loads a vehicle JSON file (data/vehicles/*.json). Each wheel's "tyre"
// field is either an inline object (same schema load_tyre_json reads) or a
// string path to a separate tyre JSON/.tir file, resolved relative to the
// vehicle file's own directory if not already absolute.
vehicle::VehicleDesc load_vehicle_json(const std::string& path);

// PHYS-033 P4 I7c C16a: how a simulated engine's "cycle" block is turned
// into runtime maps at load time (docs/data_formats/engine.md). Only an
// engine with a "cycle" block and an incomplete "maps" override runs the
// generator; every other file loads exactly as without options.
struct EngineMapOptions {
    drivetrain::MapQuality quality = drivetrain::MapQuality::Full;
    jobs::JobSystem* pool = nullptr; // nullptr = generate on the calling thread
    std::string cache_dir;           // unused until C16b (.psmaps cache)
    // Test hook: incremented once per generator run (generate_engine_maps
    // call) this load makes. nullptr = not counted.
    std::size_t* generation_count_out = nullptr;
};

vehicle::VehicleDesc load_vehicle_json(const std::string& path, const EngineMapOptions& options);

} // namespace ps::io
