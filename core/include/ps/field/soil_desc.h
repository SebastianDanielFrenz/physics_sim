// ps/field/soil_desc.h - soil descriptor (physics_sim.soil/1, F1a).
// Descriptors only; the loader is ps/field/field_io.h. Every number in a data
// file carries a source tag ([C] cited, [D] derived, [M] measured, [AS]
// assumed); the loader refuses a file with a parameter that has none.

#pragma once

#include "ps/field/management_desc.h"
#include "ps/field/nutrient_desc.h"
#include "ps/types.h"

#include <string>

namespace ps::field {

struct SoilDesc {
    std::string name;
    std::string texture;               // informational ("silt loam")
    real theta_fc = 0.0;               // volumetric water content at field capacity, m3/m3
    real theta_wp = 0.0;               // ... at permanent wilting point
    real depth_m = 0.0;                // bottom of the water bucket (root zone cannot go deeper)
    real fallow_root_depth_m = 0.0;    // depth of the "root zone" bucket without a crop
    real evaporation_depth_m = 0.0;    // Ze, depth of the surface layer that dries by evaporation
    real rew_mm = 0.0;                 // readily evaporable water (stage 1 limit)
    // Surface runoff, SCS curve number method (NRCS NEH-4).
    real curve_number_ii = 0.0;        // CN for average antecedent moisture
    real cn_normal_wetness = 0.5;      // relative root-zone wetness at which CN_II applies
    real ia_ratio = 0.2;               // initial abstraction as a fraction of the retention S
    // Snow store: degree-day melt.
    real snow_melt_mm_per_k_day = 0.0;
    real snow_melt_base_c = 0.0;

    // Optional nutrient block (F1b); required when the field runs with nutrients.
    bool has_nutrients = false;
    SoilNutrientDesc nutrients;

    // Optional management block (F1c); required when the field runs with management_enabled.
    bool has_management = false;
    SoilManagementDesc management;

    // Derived (not in the file).
    real taw_mm_per_m() const { return 1000.0 * (theta_fc - theta_wp); }
    // Total evaporable water, FAO-56 eq 73.
    real tew_mm() const { return 1000.0 * (theta_fc - 0.5 * theta_wp) * evaporation_depth_m; }
    real total_taw_mm() const { return taw_mm_per_m() * depth_m; }
};

// First problem with the descriptor ("soil <name>: <field> ..."), or empty.
std::string soil_validation_error(const SoilDesc& soil);

} // namespace ps::field
