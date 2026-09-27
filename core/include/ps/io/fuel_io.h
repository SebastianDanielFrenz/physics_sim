// ps/io/fuel_io.h — JSON loading for FuelDesc (PLAN.md D6: data files, not
// code). nlohmann::json stays confined to fuel_io.cpp.

#pragma once

#include "ps/drivetrain/fuel_desc.h"

#include <string>

namespace ps::io {

// Loads a fuel JSON file (data/fuels/*.json, "format": "physics_sim.fuel/1").
// Throws std::runtime_error on a malformed/unreadable file, a missing/wrong
// "format", or a missing/invalid required field (name/phase/ignition/
// lhv_j_kg/afr_stoich/density_kg_m3/source).
drivetrain::FuelDesc load_fuel_json(const std::string& path);

} // namespace ps::io
