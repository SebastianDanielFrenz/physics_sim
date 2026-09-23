// ps/io/vehicle_io.h — JSON loading for TyreDesc/VehicleDesc (PLAN.md D6:
// "vehicles ... are files, not code"). nlohmann::json stays confined to
// vehicle_io.cpp.

#pragma once

#include "ps/vehicle/tyre_desc.h"
#include "ps/vehicle/vehicle_desc.h"

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

} // namespace ps::io
