// core/src/engine_cycle/map_cache.h — PHYS-033 P4 I7c C16b (out/i7c_plan.md
// D8): an opt-in on-disk cache for generate_engine_maps' output, so a
// vehicle load that would otherwise re-run the (measurably expensive, see
// this file's own progress notes) 0D cycle generator every time can instead
// read a previous run's result back in milliseconds. Entirely internal
// (ps::drivetrain, core/src/engine_cycle - mirrors map_generator.cpp's own
// namespace choice, PHYS-033 P4 I7c C16a); the only public-facing surface
// this enables is ps::io::EngineMapOptions::cache_dir (core/include/ps/io/
// vehicle_io.h, already present since C16a as "unused until C16b").
//
// File format ("<cache_dir>/<key:016x>.psmaps", little-endian - this
// project's only build targets, Windows/WSL, are both x86_64, same
// assumption core/src/io/test_ground_loader.cpp's .bin reader already
// makes):
//   char     magic[8]        = "PSMAPS01"
//   uint32_t format_version  = kPsMapsFormatVersion (THIS file's own binary
//                              layout version - distinct from
//                              ps::drivetrain::kGeneratorVersion, which is
//                              already folded into the cache KEY below, so a
//                              generator-version bump changes the file NAME
//                              rather than needing a format bump too)
//   uint64_t key             = engine_map_cache_key(...) - re-checked
//                              against the requested key on load (a
//                              same-named-but-wrong-key file is exactly as
//                              corrupt as a truncated one, H2)
//   uint64_t content_hash    = engine_maps_content_hash(maps) at write time
//                              - re-computed and compared on every load
//   uint32_t quality         = static_cast<uint32_t>(MapQuality)
//   uint64_t payload_size    = the payload's exact byte length
//   <payload_size> bytes of payload, PLAIN byte layout, EngineMaps field
//   order (see map_cache.cpp's write_payload/read_payload for the exact
//   sequence): every Table2D as {size(x), x[], size(y), y[], values[]}
//   (values.size() == x.size()*y.size(), not stored again), every Curve1D
//   as {size(x), x[], y[]} (y.size() == x.size()), a length-prefixed UTF-8
//   string for maps.source, has_pmep_excess/has_theta_kl as a single byte
//   each, eta_spark_spread as one double. generator_version/quality/
//   content_hash themselves are NOT repeated in the payload (quality and
//   content_hash already live in the header above; generator_version is set
//   to the CURRENT ps::drivetrain::kGeneratorVersion on every load, since a
//   cache built by a different generator version would have a different KEY
//   and thus a different filename - it could never reach here as a "hit").
//
// Any mismatch (missing file, short read, wrong magic, wrong format
// version, wrong key, a payload whose re-computed content_hash disagrees
// with the header's) is ALWAYS treated as a plain cache miss - never an
// exception, the file is simply left alone and the caller regenerates (H2).
// A write goes to a fresh, process+thread-unique temp file in the same
// directory, then std::filesystem::rename (atomic on both this project's
// target platforms' filesystems for a same-directory rename) - a
// pre-existing temp file from an earlier crashed/killed run is never read
// by load_cached_engine_maps (it doesn't match "<key:016x>.psmaps") and is
// simply overwritten the next time THIS process picks the same temp name,
// or left as harmless debris otherwise (H4).

#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/fuel_desc.h"

#include <cstdint>
#include <string>

namespace ps::drivetrain {

// This file's own binary layout version (see the header comment above for
// why this is distinct from kGeneratorVersion). Bump on any payload-layout
// change; a reader that sees a different value treats the file as corrupt
// (H2), never attempts a partial/compatible parse.
inline constexpr std::uint32_t kPsMapsFormatVersion = 1;

// PROVISIONAL DECISION (P-F, this commit): D8's own recommendation keys the
// cache on the canonical ENGINE JSON TEXT (parsed, re-serialised with
// sorted keys, shortest-round-trip numbers) plus the fuel JSON text. The
// only seam that has EngineMapOptions::cache_dir (core/src/io/
// vehicle_io.cpp's read_simulated_fields) never sees raw JSON text - by the
// time it decides whether to call generate_engine_maps, the engine file has
// already been fully parsed into a CycleModelDesc and the fuel file into a
// FuelDesc. Hashing those RESOLVED structs directly instead is a strictly
// cleaner seam here: H3's "does not change with JSON key order or
// whitespace" requirement is satisfied automatically (there is no text left
// to vary), and it also sidesteps any floating-point round-trip risk a
// re-serialise-then-hash-text approach would carry. source strings on both
// structs are citations, not physics values, and are excluded - mirrors
// engine_maps_content_hash's own "source string excluded - not model data"
// convention exactly.
[[nodiscard]] std::uint64_t engine_map_cache_key(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality);

// Attempts a cache load. Returns nullptr on ANY problem - see this file's
// header comment for the exact list, all silently treated as a miss (H2).
// cache_dir empty -> nullptr immediately, no filesystem access at all (the
// opt-in default - see ps::io::EngineMapOptions::cache_dir).
[[nodiscard]] EngineMapsPtr load_cached_engine_maps(const std::string& cache_dir, std::uint64_t key);

// Writes maps to the cache via a temp-file-then-rename (H4). Never throws:
// an empty cache_dir, a directory that cannot be created, or a write/rename
// failure are all silently ignored - caching is a pure optimisation, never
// a correctness requirement, and a failed write must not fail the vehicle
// load that triggered it.
void store_cached_engine_maps(const std::string& cache_dir, std::uint64_t key, const EngineMaps& maps);

} // namespace ps::drivetrain
