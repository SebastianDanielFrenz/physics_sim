#pragma once
// JSON side of the turbo stage-map generator (ps/drivetrain/turbo_meanline.h): physics_sim.turbo_geometry/1 in,
// physics_sim.turbo_stage_maps/1 out, and the one parser of stage-map JSON that the vehicle loader also uses (so a
// generated map, written or kept in memory, is read exactly like a shipped file). Bad input throws std::runtime_error
// (the loader convention); generation errors throw std::invalid_argument (turbo_meanline.h).
#include "ps/drivetrain/turbo_meanline.h"

#include <cstdint>
#include <memory>
#include <string>

namespace ps::drivetrain { struct TurboStageMaps; }

namespace ps::io {

// A physics_sim.turbo_geometry/1 profile, or a physics_sim.turbo_stage_maps/1 file (its embedded "geometry" is used:
// regenerating a shipped map needs nothing but the map). label names the input in error messages.
drivetrain::meanline::StageGeometry parse_turbo_geometry_json(const std::string& text, const std::string& label);
drivetrain::meanline::StageGeometry load_turbo_geometry_json(const std::string& path);

// The three axes of a physics_sim.turbo_stage_maps/1 file (the start of a runtime extension: extend_sampling_for_demand
// keeps them bitwise).
drivetrain::meanline::Sampling parse_turbo_stage_sampling_json(const std::string& text, const std::string& label);

// physics_sim.turbo_stage_maps/1 text of a generated map, the same layout tools/turbo_sim/meanline.py writes (sorted
// keys; "sampling" only for an adaptive map; "content_hash" = "fnv1a64:<hex>" over the body without it, the Python
// writes a SHA-256 there - no reader trusts either). A "generator" block records the C++ port and its cache key.
std::string turbo_stage_maps_to_json(const drivetrain::meanline::StageMapResult& r, std::uint64_t cache_key);
// Writes it atomically (temp file in the same directory, then rename). Throws std::runtime_error on an IO failure.
void write_turbo_stage_maps_json(const std::string& path, const drivetrain::meanline::StageMapResult& r, std::uint64_t cache_key);

// The loader's parse of stage-map JSON text into the maps TurboPair reads (units = turbo count). Throws on invalid data
// with the loader's messages.
std::shared_ptr<drivetrain::TurboStageMaps> turbo_stage_maps_from_json(const std::string& text, int units, const std::string& label);

} // namespace ps::io
