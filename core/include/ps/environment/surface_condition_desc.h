// ps/environment/surface_condition_desc.h - descriptors of the surface
// condition model (weather plan 4.1, 7.4; W5): one entry per surface CLASS
// (keyed by the same name as data/surfaces/surfaces.json), loaded from
// physics_sim.surface_conditions/1 (docs/data_formats/surface_conditions.md,
// ps/environment/surface_conditions_io.h), plus the climate's grid settings
// (the climate file's optional "condition" block).
//
// W5 implements the thermal part only: a 1D conduction column under the
// surface (top layer + deep layer), driven by the surface energy balance.
// Water, snow, ice, frost, soil moisture and canopy water (W6-W9) have
// zero-valued placeholders in the state but no model and no file fields yet;
// the loader rejects their keys as reserved.

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

struct SurfaceClassDesc {
    std::string name;
    SurfaceKind kind = SurfaceKind::Static;
    // Thermal fields: required for every non-static kind, absent for static.
    real albedo = 0.0;         // shortwave reflectance, [0, 1)
    real emissivity = 0.0;     // longwave, (0, 1]
    ThermalLayerDesc layer;    // the top layer (pavement / topsoil)
    ThermalLayerDesc deep;     // below it, down to the column bottom (Dirichlet at the climate's ground_temp_c)
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
