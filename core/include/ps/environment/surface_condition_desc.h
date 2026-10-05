// ps/environment/surface_condition_desc.h - descriptors of the surface
// condition model (weather plan 4.1, 4.4, 7.4; W5/W6): one entry per surface
// CLASS (keyed by the same name as data/surfaces/surfaces.json), loaded from
// physics_sim.surface_conditions/1 (docs/data_formats/surface_conditions.md,
// ps/environment/surface_conditions_io.h), plus the climate's grid settings
// (the climate file's optional "condition" block).
//
// W5 implements the thermal part: a 1D conduction column under the surface
// (top layer + deep layer), driven by the surface energy balance. W6 adds the
// water part: MTD/texture storage, drainage, evaporation/dew, soil
// infiltration/ponding and a vegetated canopy store, plus the dry/damp/wet/
// wet_grass friction levels. Snow, ice, frost, mud and the condition rolling
// resistance (W8-W10) still have zero-valued placeholders in the state; the
// loader keeps their keys reserved.

#pragma once

#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::environment {

inline constexpr int kMaxSurfaceClasses = 32;   // plan 8.1: per-class snapshot capacity

// Plan 4.1. Static classes carry no state (condition factor 1 later).
enum class SurfaceKind : std::uint8_t {
    Static = 0,
    Sealed = 1,     // asphalt, concrete: no infiltration (W6+)
    Soil = 2,       // dirt, field: infiltration and mud (W9)
    Vegetated = 3   // grass: as soil plus canopy water (W6)
};

const char* surface_kind_name(SurfaceKind k);

// One homogeneous slab of the column. `depth_m` is the slab's THICKNESS.
struct ThermalLayerDesc {
    real depth_m = 0.0;        // > 0
    real k_w_mk = 0.0;         // thermal conductivity, > 0
    real rho_c_j_m3k = 0.0;    // volumetric heat capacity, > 0
};

// W6: the friction LEVELS of one surface class per condition, relative to the
// class's OWN dry level (the SurfaceTable lambda_mu carries the absolute dry
// level; plan 5.2 step 6). Only dry/damp/wet/wet_grass are modelled in W6;
// snow/ice/mud stay at their identity value until W8/W9.
struct SurfaceFrictionDesc {
    real damp = 0.85;        // [AS] midpoint of dry and wet [D]
    real wet = 0.71;         // [D] Wong wet/dry peak (0.6 / 0.85)
    real wet_grass = 0.67;   // [D] Cenek wet/dry mean (RACE-002)
};

// W6: the per-class soil hydraulic parameters (plan 4.4). Loaded from the
// class's "soil" block; absent for sealed/static classes.
struct SoilDesc {
    real ksat_mm_h = 0.0;    // saturated hydraulic conductivity, > 0
    real theta_fc = 0.0;     // field capacity (volumetric), (0, theta_sat)
    real theta_sat = 0.0;    // saturation, (theta_fc, 1]
};

struct SurfaceClassDesc {
    std::string name;
    SurfaceKind kind = SurfaceKind::Static;
    // Thermal fields: required for every non-static kind, absent for static.
    real albedo = 0.0;         // shortwave reflectance, [0, 1)
    real emissivity = 0.0;     // longwave, (0, 1]
    ThermalLayerDesc layer;    // the top layer (pavement / topsoil)
    ThermalLayerDesc deep;     // below it, down to the column bottom (Dirichlet at the climate's ground_temp_c)

    // --- W6 water/hydraulic fields ---
    // Sealed: mean texture depth (mm, > 0) and the drainage path/slope. The
    // texture storage W_tex = texture_storage_mm when given (> 0), else
    // mtd_mm * kTexturePorosity [AS].
    real mtd_mm = 0.0;
    real texture_storage_mm = -1.0;
    real drainage_path_m = 0.0;    // > 0
    real drainage_slope_m_m = 0.0; // > 0
    // Soil/vegetated: the top-layer hydraulic parameters (required).
    SoilDesc soil;
    // Vegetated: canopy interception capacity (mm, > 0; required).
    real canopy_capacity_mm = 0.0;
    // Friction levels (defaults are the plan's reference-tyre values).
    SurfaceFrictionDesc friction;

    std::string source;
};

struct SurfaceConditionsDesc {
    // Sorted by name (the loader sorts; validation requires it), unique names.
    std::vector<SurfaceClassDesc> classes;
    std::string source;

    const SurfaceClassDesc* find(const std::string& name) const;
};

// The climate's "condition" block (weather plan 4.2/4.7, 7.1). Environment
// seconds, not game days (W5 decision D6: the relaxation is physical, a game
// day's length is configurable). Defaults are the plan's [AS] values.
struct ConditionGridDesc {
    real dt_env_s = 60.0;                    // grid step, (0, 3600]
    real spinup_s = 7.0 * 86400.0;           // spin-up length before an install / large jump, >= dt
    real max_fast_forward_s = 30.0 * 86400.0; // forward jumps up to this are stepped, larger ones spin up; >= dt
};

// W6: texture storage W_tex = mtd_mm * kTexturePorosity when the class does
// not give an explicit texture_storage_mm [AS].
inline constexpr real kTexturePorosity = 0.5;

// Column discretisation constants [AS] (W5 decision D1): first cell at the
// surface kTopCellM thick, each next cell kCellGrowth times thicker, the
// cells of each slab rescaled so the slab boundary is a node.
inline constexpr real kColumnTopCellM = 0.005;
inline constexpr real kColumnCellGrowth = 1.3;
inline constexpr int kMaxColumnNodes = 32;
inline constexpr real kMaxColumnDepthM = 10.0;

// First problem ("surface_conditions: <field> ...") or empty / throws std::invalid_argument.
std::string surface_conditions_validation_error(const SurfaceConditionsDesc& d);
void validate_surface_conditions(const SurfaceConditionsDesc& d);
std::string condition_grid_validation_error(const ConditionGridDesc& g);

} // namespace ps::environment
