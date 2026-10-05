// ps/io/engine_thermal_io.h - loaders of the two engine thermal part files
// (engine thermal plan T1, vault PHYS-063): physics_sim.engine_thermal/1
// (engine side, docs/data_formats/engine_thermal.md) and
// physics_sim.cooling_system/1 (vehicle side, docs/data_formats/
// cooling_system.md). Both resolve an "extends" chain through
// ps::io::load_json_with_extends (RFC 7396 merge, bottom-up, user file last)
// before validating, so a class-default file can be overridden field by field.
// Per-litre forms are expanded against displacement_l here; temperatures in C
// in the files become K in the descriptors. Unknown keys throw.
//
// Not map-generator inputs: nothing here may enter the C16b cache key.

#pragma once

#include "ps/drivetrain/engine_thermal_desc.h"

#include <string>

namespace ps::io {

drivetrain::EngineThermalDesc load_engine_thermal_json(const std::string& path);
drivetrain::CoolingSystemDesc load_cooling_system_json(const std::string& path);

// Parses the vehicle-level "engine_thermal_start" string ("warm"/"ambient").
// Throws std::runtime_error naming `context` on any other value.
drivetrain::ThermalStartKind parse_thermal_start(const std::string& value, const std::string& context);

// The engine file's "default_thermal" resolved against the engine file's directory (what the vehicle loader
// does), or "" when the engine file has none. Throws on an unreadable file or a non-string/empty value.
std::string engine_default_thermal_path(const std::string& engine_json_path);

} // namespace ps::io
