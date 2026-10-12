#pragma once

// PHYS-033 P4 I7c C16a: map extraction from the 0D cycle model
// (ps::engine_cycle) into the simulated engine's runtime maps
// (SimulatedEngineMapsDesc, the C11 layout). Public API per PHYS-033
// section 5.4; the algorithm lives in core/src/engine_cycle/map_generator.cpp.
//
// Point set (fixed per quality, index order = assembly order):
//   [0, G)        the rpm x p_im grid (Full: CycleModelDesc::grid_rpm x
//                 grid_p_im; Preview: at most 10 x 6 of them, picked by index
//                 rounding; Sketch: at most 8 x 3, see below), lambda 1, MBT
//                 spark, cam schedule at the point;
//   then          spark sweep: retard {-10,-5,0,2,5,10,15,20,30,40} deg from
//                 MBT at 2000 rpm/50 kPa, 3000/80, 5000/100 (sweep-major)
//                 = 30 points (Sketch: retard {-10,0,5,10,20,40} at 3000/80
//                 only = 6 points);
//   then          lambda sweep at 3000 rpm/80 kPa, lambda {0.6 .. 1.5} = 9
//                 points (Sketch: {0.6,0.8,1.0,1.2,1.5} = 5 points).
// Sketch (engine garage P1, A1) is a COMPLETE map set on a reduced grid for a
// dyno preview while the player edits an engine; it runs Preview's step and
// cycle settings, is keyed apart from Preview and Full, and is never meant to
// drive a car. Its rows: the lowest p_im node, the node nearest ambient (the
// middle node when that is the lowest or the top one), and the top node -
// on a boosted grid that brackets ambient and the top. Its rpm axis is the
// grid subsampled to at most 8 values (index rounding, both ends kept).
// Every point runs at the ISA intake state (288.15 K, 101325 Pa exhaust),
// the reference the runtime's default ambient uses, so the runtime
// reproduces each grid node exactly (test M1).
//
// Determinism: run_point(i) writes only slot i (no shared scratch; each
// call copies the prototype stepper), finish() assembles sequentially in
// index order, so the result is bit-identical for any worker count or
// run_point order (test M3). ps::math only, no libm transcendental.

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/drivetrain/fuel_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/table2d.h"
#include "ps/types.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

namespace ps::jobs {
class JobSystem;
}

namespace ps::drivetrain {

// Bumped whenever the generator's output for the same inputs changes
// (C16b keys its cache on it). ps_audio A5: the version is PER PATH -
// a lumped engine (no intake/exhaust pipe blocks) still generates through
// the unchanged single-cylinder path and is stamped (and keyed) 1, so its
// maps, content hash and state hash stay bit-identical to v1; a pipe-bearing
// engine generates through CycleEngine and is stamped 2; ps_audio A8b: 3,
// the catalyst gained its laminar substrate resistance (cycle_engine.cpp), so
// every pipe engine's maps moved and its v2 cache files must not be reused.
// engine_map_generator_version() picks the one a desc uses.
// Engine garage P7d: 4, an AMBIENT-FED pipe engine (no compressor_supply, no
// adaptive pressure sampling) gains one wide-open-throttle row per rpm as the
// top row of its p_im-axis tables (SimulatedEngineMapsDesc::wot_p_im_pa, read
// through map_pressure_index); a compressor-fed pipe engine and an adaptive
// one keep 3 (their maps are unchanged).
inline constexpr std::uint32_t kLumpedGeneratorVersion = 1;
inline constexpr std::uint32_t kPipeGeneratorVersion = 3;
inline constexpr std::uint32_t kPipeWotGeneratorVersion = 4;
inline constexpr std::uint32_t kGeneratorVersion = kPipeWotGeneratorVersion; // the newest path

// Engine garage P7d: true when the generator adds the wide-open-throttle rows
// (pipe engine fed from ambient through its throttle, fixed p_im grid).
[[nodiscard]] inline bool engine_map_generates_wot_rows(const CycleModelDesc& desc) {
    return desc.intake.has_value() && !desc.intake->compressor_supply && !desc.pressure_sampling.enabled;
}

[[nodiscard]] inline std::uint32_t engine_map_generator_version(const CycleModelDesc& desc) {
    const std::uint32_t base = !desc.intake.has_value()            ? kLumpedGeneratorVersion
                               : engine_map_generates_wot_rows(desc) ? kPipeWotGeneratorVersion
                                                                     : kPipeGeneratorVersion;
    return base + (desc.pressure_sampling.enabled ? 259 : 0);
}

// The numeric values are folded into the .psmaps cache key, the content hash
// and the .psmaps header: never renumber, only append (Sketch = 2 was added by
// engine garage P1 without moving any existing key or pin).
enum class MapQuality { Preview = 0, Full = 1, Sketch = 2 };

// Thrown by a map generation that observed its cancel flag (see
// MapGenerationControl, ps::io::EngineMapOptions::cancel). ps::io re-exports
// it as ps::io::GenerationCancelled (same type).
class GenerationCancelled : public std::runtime_error {
public:
    GenerationCancelled() : std::runtime_error("engine map generation cancelled") {}
};

// Optional control of one generation run (generate_engine_maps). Both members
// are optional; a default-constructed control changes nothing.
//
// cancel: polled with a relaxed load immediately before each point starts, on
//   the thread that is about to run it (a pool worker or the calling thread).
//   Once a point observes it set, that point and every point not yet started
//   are skipped, and the call throws GenerationCancelled after every running
//   point has finished (latency: at most one point per worker). If every point
//   had already started when the flag was set, the run completes normally and
//   returns the maps - the flag is a request, not a guarantee. generate_engine_
//   maps also throws GenerationCancelled up front when the flag is already set
//   on entry (0 points run). Nothing is written anywhere by this function (the
//   .psmaps cache is the caller's), so a cancelled run leaves no trace.
// on_progress: called once per FINISHED point with done = 1, 2, ..., total
//   (strictly rising by 1, ending at total unless the run was cancelled or a
//   point failed - a failed point counts as finished, the call then throws).
//   ALWAYS on the thread that called generate_engine_maps, never on a worker:
//   workers only bump an atomic counter (allocation-free, so the jobs keep
//   debug_alloc's zero-allocation guarantee), the calling thread polls it
//   while it waits and delivers one call per finished point, so a burst of
//   points that finish between two polls arrives as consecutive calls with
//   consecutive values. The callback may allocate and may be slow (it delays
//   only the reporting, not the workers); it must not throw - an exception
//   escaping it is held until the running points have finished, then rethrown.
//   If the calling thread is itself a pool worker (nested use) it cannot poll
//   and all calls are delivered after the points have finished.
struct EngineMaps;
struct MapGenerationControl {
    const std::atomic<bool>* cancel = nullptr;
    std::function<void(std::size_t done, std::size_t total)> on_progress;
    // Immutable coarse map snapshots, on the calling thread after each stage.
    // Display only: not final/certified and never stored in the map cache.
    std::function<void(std::shared_ptr<const EngineMaps>)> on_partial_maps;
};

struct EngineMaps {
    // The runtime maps, C11 layout: ve, eta_i, f_lambda, eta_spark,
    // theta_mbt_deg, friction_torque_nm (empty when the cycle desc has no
    // friction model), pmep_excess_pa (has_pmep_excess = true; values may be
    // negative - the runtime takes them signed). theta_kl only for a
    // knock-limited engine (CycleModelDesc::knock.enabled, engine garage P7):
    // one table per knock fuel in theta_kl_by_fuel, theta_kl_deg = the first.
    // source names the generator and its version.
    SimulatedEngineMapsDesc maps;
    // Diagnostics for I7d, not consumed by the runtime (rpm x p_im grid).
    Table2D t_exh_k; // mass-averaged exhaust-valve outflow temperature [K]
    Table2D h_exh_w; // net exhaust enthalpy flow, whole engine [W]
    // Spark sweep: largest |normalised curve - mean| over the three sweep
    // points (the eta_spark map is their mean).
    real eta_spark_spread = 0.0;
    // Grid nodes below the engine's closed-throttle manifold floor, generated
    // with the plenum held at the node pressure (diagnostic, not hashed).
    int held_plenum_points = 0;
    // Engine garage P7d (diagnostics, not hashed, not cached - 0 after a cache
    // hit): held grid nodes that do not breathe (no net fresh air, VE <= 0 or
    // eta_i outside (0, 1)) and were filled from the nearest breathing node
    // above on their rpm line; author p_im rows dropped because they lie at or
    // above the lowest wide-open-throttle plenum pressure (or ambient).
    int nonbreathing_points = 0;
    int wot_dropped_rows = 0;
    // Engine garage P7 (knock-limited engines only; diagnostics, not hashed,
    // not cached - 0 after a cache hit): grid nodes x fuels whose knock-safe
    // advance lies below the search's upper bound (theta_MBT + 10), of which
    // saturated at its lower bound (theta_MBT - 40), and the closed-cycle
    // replays run.
    int knock_limited_points = 0;
    int knock_saturated_points = 0;
    long knock_replays = 0;
    std::uint32_t generator_version = kLumpedGeneratorVersion; // set by finish() / the cache loader
    MapQuality quality = MapQuality::Full;
    std::uint64_t content_hash = 0; // engine_maps_content_hash(*this), filled by finish()
};
using EngineMapsPtr = std::shared_ptr<const EngineMaps>;

// One generation job split into independent points. Construct, call
// run_point(i) exactly once for every i in [0, point_count()) - from any
// threads, in any order - then finish() once.
class EngineMapGenTask {
public:
    // Throws std::runtime_error on an unusable desc (grid missing or not
    // strictly ascending, geometry <= 0, a non-4-stroke cycle).
    // ps_audio A5: a desc with intake/exhaust pipe blocks generates every
    // point through ps::engine_cycle::CycleEngine (all cylinders, the pipe
    // network and the resolved installation, throttle trimmed to the point's
    // p_im); a grid node whose p_im the fully open throttle cannot reach (or
    // the trim does not settle to within its tolerance) fails the task with
    // "engine map generator: plenum target ..." (PROVISIONAL P6). A node
    // BELOW the closed-throttle floor (overlap backflow) is generated with
    // the plenum held at the node pressure instead (held_plenum_points). Sweep
    // points only feed normalised ratios and are not held to the target.
    EngineMapGenTask(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality);
    ~EngineMapGenTask();
    EngineMapGenTask(const EngineMapGenTask&) = delete;
    EngineMapGenTask& operator=(const EngineMapGenTask&) = delete;

    [[nodiscard]] std::size_t point_count() const;
    [[nodiscard]] std::size_t grid_point_count() const;
    void run_point(std::size_t i);
    // Before dispatch: reuse successful points from an identical engine/fuel/
    // quality task. Grid coordinates may differ. Returns the reused slot count.
    std::size_t reuse_points_from(const EngineMapGenTask& previous);
    // Engine garage P1 (validate-only loads): builds, without running anything,
    // what run_point(0) would build - for a pipe-bearing desc the CycleEngine
    // (its pipe network, installation and CFL checks), for a lumped one nothing
    // beyond the constructor - so a desc the generator would reject at build
    // time throws here with the same message finish() would rethrow. Does not
    // consume the task.
    void build_check() const;
    // Throws if a point was not run, or a speed line's air mass is not
    // strictly increasing in p_im (the eta_i resampling needs it).
    [[nodiscard]] EngineMapsPtr finish();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Convenience: all points on the pool (nullptr = on the calling thread,
// index order), then finish(). `control` (nullptr = none) adds cancellation and
// progress reporting, see MapGenerationControl; the result is bit-identical
// with or without it.
[[nodiscard]] EngineMapsPtr generate_engine_maps(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality,
                                                 jobs::JobSystem* pool = nullptr, const MapGenerationControl* control = nullptr);

// Engine garage P1 (validate-only loads, ps::io::EngineMapOptions::generate =
// false): every check generate_engine_maps makes BEFORE it needs a generated
// number - the task constructor's desc/fuel/grid/spark-schedule validation, the
// build-time part of the first point (build_check) and the friction
// correlation's sign over the friction axis - without running a point. The
// messages are exactly those a full generation would throw, in the same order.
// What it cannot see: rejections that depend on generated values (an
// unreachable plenum target at a node, VE/eta_i out of range, a non-increasing
// air mass); those only fire when generating.
void validate_engine_map_generation(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality);

// Engine garage P1 (A2): the .psmaps cache key a load would use for this
// resolved cycle desc + fuel + quality (FNV-1a over every generator input, the
// generator version of the desc's path included; see core/src/engine_cycle/
// map_cache.h for what is and is not folded). The file a cache_dir load writes
// is "<cache_dir>/<key as 16 lower-case hex digits>.psmaps". Implemented in
// ps_core (map_cache.cpp).
[[nodiscard]] std::uint64_t engine_map_cache_key(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality);

// FNV-1a over every field of `maps` except content_hash itself, in a fixed
// order (source string excluded - not model data).
[[nodiscard]] std::uint64_t engine_maps_content_hash(const EngineMaps& maps);

// Friction torque [N*m] of the D13 correlation at `rpm` for the engine's
// displacement: FMEP * V_disp / (4 pi). 0 for Model::None.
[[nodiscard]] real cycle_friction_torque_nm(const CycleModelDesc& desc, real rpm);

} // namespace ps::drivetrain
