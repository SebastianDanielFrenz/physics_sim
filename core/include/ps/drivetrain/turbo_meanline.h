#pragma once
// C++ port of tools/turbo_sim/meanline.py (solver "meanline-4"): physics_sim.turbo_geometry/1 -> the stage maps
// physics_sim.turbo_stage_maps/1 that TurboPair reads (ps/drivetrain/turbo_stage_maps.h). The Python tool stays the
// reference implementation; tests/unit/test_turbo_meanline.cpp cross-checks this port against its committed output.
//
// Same equations, same iteration counts, same adaptive refinement rule as the Python. Deterministic: transcendentals go
// through ps::math (std::sqrt is correctly rounded), every point is independent and written to its own slot, so the
// result is bit-identical for any worker count and platform. It is NOT bit-identical to the Python (libm vs ps::math,
// hypot vs sqrt): the cross-check states its tolerance.
//
// Errors: invalid geometry or sampling throws std::invalid_argument (the loader convention: bad input data throws);
// the point solvers never throw for a validated geometry and an operating point inside the axes' domain.
#include "ps/types.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ps::jobs { class JobSystem; }

namespace ps::drivetrain {
struct TurboStageMaps;
struct TurboStageDemand; // ps/drivetrain/engine_rules.h

namespace meanline {

inline constexpr const char* kSolverVersion = "meanline-4";
inline constexpr real kReferencePa = 101325.0;
inline constexpr real kReferenceK = 288.15;          // compressor reference inlet temperature
inline constexpr real kTurbineReferenceK = 1000.0;   // turbine reference inlet temperature

struct Gas {
    real r = 287.05, gamma = 1.4;
    real cp() const { return r * gamma / (gamma - real(1)); }
};

enum class ShockModel { Unsupported, InletNormalEntropy };

struct CompressorGeometry {
    real inducer_mm = 0, exducer_mm = 0, hub_mm = 0, exit_width_mm = 0;
    int blades = 0;
    real exit_angle_deg = 0;  // relative blade angle, from tangent
    real inlet_angle_deg = 0; // at the RMS eye radius, from tangent
    real blockage = 1, passage_loss = 0, incidence_loss = 0, diffuser_loss = 0;
    ShockModel shock_model = ShockModel::Unsupported;
};

struct TurbineGeometry {
    real diameter_mm = 0, effective_throat_mm2 = 0;
    real nozzle_angle_deg = 0; // absolute jet angle from tangent
    real nozzle_efficiency = 1, rotor_velocity_retention = 0;
};

// One physics_sim.turbo_geometry/1 profile. profile_json is the profile's canonical JSON text (sorted keys, as parsed):
// it is embedded verbatim as the map's "geometry" and is part of the cache key. Build it with
// ps::io::parse_turbo_geometry_json (ps/io/turbo_stage_io.h).
struct StageGeometry {
    CompressorGeometry compressor;
    TurbineGeometry turbine;
    Gas air{287.05, 1.4}, exhaust{287.05, 1.33};
    std::string source;       // profile "source" (required, non-empty)
    std::string profile_json; // the whole profile, canonical JSON
};

// Throws std::invalid_argument with the Python's message for an invalid geometry.
void validate(const StageGeometry& g);

enum class CompressorStatus { Ok, InletChoke, ExitChokeOrNoPositiveHead, RelativeChoke, TransonicUnmodelled, NoPositivePressureRise };
const char* to_string(CompressorStatus s); // the Python's status strings ("ok", "inlet_choke", ...)

struct CompressorPoint {
    CompressorStatus status = CompressorStatus::InletChoke;
    // Valid for every status except InletChoke (which reports nothing) and ExitChokeOrNoPositiveHead (inlet_mach only).
    real pressure_ratio = 0, outlet_total_k = 0, specific_work_j_kg = 0, shaft_power_w = 0, efficiency = 0;
    real shock_pressure_retention = 1, shock_entropy_j_kg_k = 0;
    real inlet_mach = 0, inlet_relative_mach = 0, exit_relative_mach = 0, incidence_risk = 0;
    real continuity_residual_kg_s = 0, energy_residual_w = 0;
};

struct TurbinePoint {
    real mass_flow_kg_s = 0, specific_work_j_kg = 0, shaft_power_w = 0, specific_torque_nm_per_kg_s = 0;
    real outlet_total_k = 0, efficiency = 0, energy_residual_w = 0;
    bool choked = false;
};

// meanline.py nozzle_flux: kg/(m2 s), isentropic convergent nozzle incl. sonic choking (upstream total, downstream static).
real nozzle_flux(real upstream_pa, real downstream_pa, real total_k, const Gas& gas);
// meanline.py normal_shock_pressure_retention (1 on the subsonic branch).
real normal_shock_pressure_retention(real mach, const Gas& gas);
CompressorPoint compressor_point(const StageGeometry& g, real rpm, real mdot_kg_s, real pt = kReferencePa, real tt = kReferenceK);
TurbinePoint turbine_point(const StageGeometry& g, real rpm, real expansion_ratio, real pt = kReferencePa, real tt = kTurbineReferenceK);

// The three axes of a map: corrected rpm, per-unit corrected compressor flow (kg/s), turbine expansion ratio.
// Finite, strictly increasing, >= 2 points each, rpm >= 0, flow >= 0, expansion >= 1 (the loader also wants the
// turbine axis to START at 1).
struct Sampling {
    std::vector<real> corrected_rpm, flow_kg_s, expansion_ratio;
};
// meanline.py main()'s uniform axes: rpm_max*i/(n-1), flow_max*i/(n-1), expansions uniform in the isentropic spouting
// velocity sqrt(1 - ER^-(g-1)/g) up to expansion_max (exhaust gamma of the geometry).
Sampling uniform_sampling(const StageGeometry& g, real rpm_max, int speed_lines, real flow_max, int flow_points,
                          real expansion_max, int expansion_points);

struct AdaptiveOptions {
    real tolerance = 0.03; // relative interpolation error at cell centres
    int max_passes = 3;
    int max_speed_lines = 65, max_flow_points = 129, max_expansion_points = 65;
};
struct StageAudit {
    int cell_centers_checked = 0, coverage_boundary_cells = 0;
    real maximum_relative_error = 0;
};
struct SamplingReport { // the map's "sampling" block (adaptive generation only)
    real target_relative_error = 0;
    int passes = 0;
    bool budget_limited = false;
    StageAudit compressor, turbine;
};

struct StageMapResult {
    Sampling axes;                          // the final axes (refined, for an adaptive map)
    std::vector<CompressorPoint> compressor; // row-major [rpm][flow]
    std::vector<TurbinePoint> turbine;       // row-major [rpm][expansion]
    bool adaptive = false;
    SamplingReport sampling;                // meaningful only when adaptive
    StageGeometry geometry;
};

// Every point on the given axes (meanline.py Generator.generate). pool: optional, points spread over its workers.
StageMapResult generate(const StageGeometry& g, const Sampling& s, ps::jobs::JobSystem* pool = nullptr);
// meanline.py Generator.adaptive_generate: generate, audit the bilinear error at every fully valid cell centre against
// an independently solved point, add the failing cells' centre lines (largest error first, within the budgets), repeat.
StageMapResult adaptive_generate(const StageGeometry& g, const Sampling& initial, const AdaptiveOptions& o,
                                 ps::jobs::JobSystem* pool = nullptr);

// Cache identity of a generation request: FNV-1a 64 over the solver version, the canonical profile JSON, the initial
// axes and (adaptive) the options. The grid of an adaptive result depends only on these, so equal keys = equal maps.
std::uint64_t cache_key(const StageGeometry& g, const Sampling& initial, const AdaptiveOptions* adaptive);

// ---- Runtime extension for a demand (racing_game: generate a wider map when check_turbo_stage_coverage is Near/Limiting)
// A demanded operating point: per-turbo corrected flow and the corrected rotor speed that makes the target pressure ratio.
struct DemandPoint { real flow_kg_s = 0, rotor_rpm = 0; };
struct ExtensionPolicy {        // every number is the caller's config ([AS] defaults below)
    real flow_margin = 0.10;    // the flow axis reaches the largest demanded flow x (1 + flow_margin)
    real speed_margin = 0.05;   // dense speed lines up to the largest demanded speed x (1 + speed_margin)
    real speed_step_rpm = 500;  // dense line spacing over the demand band (multiples of it, so keys are stable); [D] the
                                // car_hyper_n2o demand at 1.8 bar sits 376 rpm above the choke edge (D1, 2026-10-14)
    real flow_step_kg_s = 0.01; // flow knot spacing added over the demand band (multiples of it)
    int max_speed_lines = 129, max_flow_points = 257; // the generator's adaptive budgets; steps double until they fit
};
// The current axes plus dense knots over the demand: speed lines at multiples of speed_step_rpm from one step below the
// slowest demanded speed to the fastest x (1 + speed_margin); flow knots at multiples of flow_step_kg_s from the smallest
// demanded flow to the largest x (1 + flow_margin). The current knots are kept bitwise, so every point the current map
// has is regenerated identically. The coverage boundary of a bilinear map lags the true choke line by one speed-line
// spacing (a cell needs four valid corners); dense lines move it onto the choke line. A demand beyond the CHOKE line
// stays uncovered whatever the sampling: the solver itself finds no valid point there.
// The demand of turbo_stage_demand(engine) as points: every step's flow at its rotor speed; a step the map does not
// reach below its top line gets the affinity estimate n_top x sqrt((PR^k - 1) / (PR_top^k - 1)), k = air_kappa (the
// map's (gamma-1)/gamma), from the continuation's pressure ratio PR_top on the top line at that flow.
std::vector<DemandPoint> demand_points(const TurboStageDemand& d, real air_kappa);
Sampling extend_sampling_for_demand(const Sampling& current, const std::vector<DemandPoint>& demand, const ExtensionPolicy& p);

// The in-memory maps TurboPair reads, exactly as ps::io would build them from to_json(result): same fields, same
// validity mask. units = turbo count of the installation.
std::shared_ptr<TurboStageMaps> to_stage_maps(const StageMapResult& r, int units);

} // namespace meanline
} // namespace ps::drivetrain
