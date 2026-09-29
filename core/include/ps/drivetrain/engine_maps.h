#pragma once

// PHYS-033 P4 I7c C16a: map extraction from the 0D cycle model
// (ps::engine_cycle) into the simulated engine's runtime maps
// (SimulatedEngineMapsDesc, the C11 layout). Public API per PHYS-033
// section 5.4; the algorithm lives in core/src/engine_cycle/map_generator.cpp.
//
// Point set (fixed per quality, index order = assembly order):
//   [0, G)        the rpm x p_im grid (Full: CycleModelDesc::grid_rpm x
//                 grid_p_im; Preview: at most 10 x 6 of them, picked by index
//                 rounding), lambda 1, MBT spark, cam schedule at the point;
//   [G, G + 30)   spark sweep: retard {-10,-5,0,2,5,10,15,20,30,40} deg from
//                 MBT at 2000 rpm/50 kPa, 3000/80, 5000/100 (sweep-major);
//   [G + 30, +9)  lambda sweep at 3000 rpm/80 kPa, lambda {0.6 .. 1.5}.
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

#include <cstddef>
#include <cstdint>
#include <memory>

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
inline constexpr std::uint32_t kLumpedGeneratorVersion = 1;
inline constexpr std::uint32_t kPipeGeneratorVersion = 3;
inline constexpr std::uint32_t kGeneratorVersion = kPipeGeneratorVersion; // the newest path

[[nodiscard]] inline std::uint32_t engine_map_generator_version(const CycleModelDesc& desc) {
    return desc.intake.has_value() ? kPipeGeneratorVersion : kLumpedGeneratorVersion;
}

enum class MapQuality { Preview, Full };

struct EngineMaps {
    // The runtime maps, C11 layout: ve, eta_i, f_lambda, eta_spark,
    // theta_mbt_deg, friction_torque_nm (empty when the cycle desc has no
    // friction model), pmep_excess_pa (has_pmep_excess = true; values may be
    // negative - the runtime takes them signed), no theta_kl (C17b). source
    // names the generator and its version.
    SimulatedEngineMapsDesc maps;
    // Diagnostics for I7d, not consumed by the runtime (rpm x p_im grid).
    Table2D t_exh_k; // mass-averaged exhaust-valve outflow temperature [K]
    Table2D h_exh_w; // net exhaust enthalpy flow, whole engine [W]
    // Spark sweep: largest |normalised curve - mean| over the three sweep
    // points (the eta_spark map is their mean).
    real eta_spark_spread = 0.0;
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
    // "engine map generator: plenum target ..." (PROVISIONAL P6). Sweep
    // points only feed normalised ratios and are not held to the target.
    EngineMapGenTask(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality);
    ~EngineMapGenTask();
    EngineMapGenTask(const EngineMapGenTask&) = delete;
    EngineMapGenTask& operator=(const EngineMapGenTask&) = delete;

    [[nodiscard]] std::size_t point_count() const;
    [[nodiscard]] std::size_t grid_point_count() const;
    void run_point(std::size_t i);
    // Throws if a point was not run, or a speed line's air mass is not
    // strictly increasing in p_im (the eta_i resampling needs it).
    [[nodiscard]] EngineMapsPtr finish();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Convenience: all points on the pool (nullptr = on the calling thread,
// index order), then finish().
[[nodiscard]] EngineMapsPtr generate_engine_maps(const CycleModelDesc& desc, const FuelDesc& fuel, MapQuality quality,
                                                 jobs::JobSystem* pool = nullptr);

// FNV-1a over every field of `maps` except content_hash itself, in a fixed
// order (source string excluded - not model data).
[[nodiscard]] std::uint64_t engine_maps_content_hash(const EngineMaps& maps);

// Friction torque [N*m] of the D13 correlation at `rpm` for the engine's
// displacement: FMEP * V_disp / (4 pi). 0 for Model::None.
[[nodiscard]] real cycle_friction_torque_nm(const CycleModelDesc& desc, real rpm);

} // namespace ps::drivetrain
