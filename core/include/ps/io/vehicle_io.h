// ps/io/vehicle_io.h — JSON loading for TyreDesc/VehicleDesc (PLAN.md D6:
// "vehicles ... are files, not code"). nlohmann::json stays confined to
// vehicle_io.cpp.

#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/installation_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/vehicle/tyre_desc.h"
#include "ps/vehicle/vehicle_desc.h"

#include <cstddef>
#include <string>

namespace ps::io {

// Loads a tyre JSON file (data/tyres/*.json). Throws std::runtime_error on
// a malformed/unreadable file or a missing required field (name/source -
// PLAN.md section 8).
vehicle::TyreDesc load_tyre_json(const std::string& path);

// ps_audio A4 (out/audio_plan.md section 3.2): loads an installation JSON
// file (data/installations/*.json, format "physics_sim.installation/1").
// Throws std::runtime_error on a malformed/unreadable file, a missing
// required field, or an inconsistent exhaust chain (see
// core/src/io/vehicle_io.cpp's read_installation_object for the exact
// rules). Called by load_vehicle_json to resolve an engine's
// default_installation / a vehicle component's "installation" override -
// not normally called directly outside tests.
drivetrain::InstallationDesc load_installation_json(const std::string& path);

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

// ps_audio A10 (out/audio_plan.md section 6, EngineRig): loads a standalone
// engine JSON file (format "physics_sim.engine/1", the same schema as a
// vehicle's powertrain.components[] "ref"'d engine file) with no vehicle
// wrapping it. Reuses the exact "kind" dispatch (torque_map/simulated),
// fuel resolution, and (for a simulated engine with pipe blocks) map
// generation and installation resolution that load_vehicle_json's engine
// component reader uses - a standalone engine loads identically to the same
// file referenced from a vehicle.
//
// `installation_override_path`: nullptr means "use the engine file's own
// default_installation" (required whenever the engine's cycle block has
// pipe blocks, rejected otherwise) - the same rule load_vehicle_json applies
// when a vehicle component gives no "installation" override. A non-null
// pointer is only accepted for a pipe-bearing simulated engine (same
// rejection as the vehicle loader's per-component override).
//
// Throws std::runtime_error on a malformed/unreadable file, exactly the
// messages read_engine_fields (vehicle_io.cpp) already gives for the same
// content reached through a vehicle file.
drivetrain::ComponentParams load_engine_json(const std::string& path, const EngineMapOptions& options = EngineMapOptions{},
                                              const std::string* installation_override_path = nullptr);

} // namespace ps::io
