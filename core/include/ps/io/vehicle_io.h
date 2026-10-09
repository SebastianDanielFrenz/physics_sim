// ps/io/vehicle_io.h — JSON loading for TyreDesc/VehicleDesc (PLAN.md D6:
// "vehicles ... are files, not code"). nlohmann::json stays confined to
// vehicle_io.cpp.

#pragma once

#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/installation_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/vehicle/tyre_desc.h"
#include "ps/vehicle/vehicle_desc.h"

#include <atomic>
#include <cstddef>
#include <functional>
#include <string>

namespace ps::io {

// Thrown by a load whose EngineMapOptions::cancel flag was set while it
// generated engine maps (see EngineMapOptions::cancel). Derived from
// std::runtime_error; the same type as ps::drivetrain::GenerationCancelled.
using GenerationCancelled = drivetrain::GenerationCancelled;

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
//
// Engine garage P1 (out/engine_garage_design.md section 9.2) added cancel,
// on_progress and generate, and MapQuality::Sketch.
struct EngineMapOptions {
    // Full, Preview or Sketch (a complete map set on a reduced grid for a dyno
    // preview, never to drive a car - see ps/drivetrain/engine_maps.h). Each
    // quality has its own cache key.
    drivetrain::MapQuality quality = drivetrain::MapQuality::Full;
    jobs::JobSystem* pool = nullptr; // nullptr = generate on the calling thread
    std::string cache_dir;           // .psmaps cache directory; empty = no disk access (C16b)
    // Test hook: incremented once per generator run (generate_engine_maps
    // call) this load makes. nullptr = not counted.
    std::size_t* generation_count_out = nullptr;

    // Cancellation of a load that generates engine maps: the flag is read (relaxed)
    // before each point starts, on the thread that would run it; once set, no further
    // point starts and the load throws GenerationCancelled after the running points
    // finished - latency at most one point per worker (0.1-0.5 s on a pipe engine at
    // Full). Also checked once before the .psmaps cache lookup, so a load that reaches
    // map generation with the flag already set throws without running a point (even
    // when the cache would have hit). A cancelled load writes no cache file. If every
    // point had already started when the flag is set, the load finishes normally and
    // writes its cache file. A load that never generates (a torque-map engine, an
    // engine with a complete "maps" block, generate = false) never reads the flag.
    // nullptr = not cancellable.
    const std::atomic<bool>* cancel = nullptr;

    // Progress of the map generation, may be empty. Called once per finished point
    // with done = 1, 2, ..., total (strictly rising by 1, ending at done == total for
    // a load that is not cancelled and has no failing point), always on the thread that
    // called load_vehicle_json/load_engine_json - never on a pool worker - so the
    // callback needs no locking and may allocate; the workers only bump an atomic
    // counter, which keeps the zero-allocation-inside-jobs guarantee. A burst of points
    // finishing between two polls arrives as consecutive calls. NOT called for a cache
    // hit or a load that generates nothing; one load with several generating engines
    // reports each engine's run from 1 to its own total. The callback must not throw (an
    // exception is rethrown after the running points finished). If the loading thread
    // is itself a worker of `pool`, the calls arrive after the points finished.
    std::function<void(std::size_t done, std::size_t total)> on_progress;
    // Coarse engine snapshots during adaptive refinement; display only.
    std::function<void(const drivetrain::SimulatedEngineDesc&)> on_partial_engine;

    // false = validate-only: resolve and validate everything a full load does
    // (every loader rejection fires with the same message, plus the generator's own
    // build-time checks - drivetrain::validate_engine_map_generation) but run no
    // generator, read no cache and write no file (the turbo-table cache included). The
    // returned engine's maps are PENDING: an engine whose "cycle" block would generate
    // maps comes back with SimulatedEngineDesc::maps_pending = true and only the keys of
    // its own "maps" block filled; building a SimulatedEngine (a World vehicle, an
    // EngineRig, a Powertrain) from it throws. generation_count_out stays 0. Rejections
    // that depend on generated numbers (an unreachable plenum node, VE or eta_i out of
    // range) fire only when generating. Instant: milliseconds.
    bool generate = true;
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
drivetrain::TurboPairDesc load_turbo_configuration_json(const std::string& path, const std::string& cache_dir = {});

drivetrain::ComponentParams load_engine_json(const std::string& path, const EngineMapOptions& options = EngineMapOptions{},
                                              const std::string* installation_override_path = nullptr);

} // namespace ps::io
