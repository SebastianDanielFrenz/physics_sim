// ps/field/crop_desc.h - crop descriptor (physics_sim.crop/1, F1a): an
// AquaCrop-style daily crop (thermal time to BBCH stages, vernalisation,
// canopy cover, transpiration x water productivity -> biomass, harvest index
// -> yield). Descriptors only; the loader is ps/field/field_io.h. Every number
// in a data file carries a source tag, see soil_desc.h.

#pragma once

#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::field {

struct CropStageDesc {
    std::string name;
    real bbch = 0.0;     // BBCH scale code the stage starts at (reporting only)
    real gdd = 0.0;      // development units from sowing at which the stage starts (ascending)
};

struct CropDesc {
    std::string name;
    std::string family;          // crop family; the pathogen pressure of F1d is per family

    // Thermal time (daily GDD from Tmin/Tmax, both clamped to [t_base, t_upper]).
    real t_base_c = 0.0;
    real t_upper_c = 0.0;

    // Phenology: ordered stages; the named ones the model needs are looked up
    // at load time into the indices below.
    std::vector<CropStageDesc> stages;
    int stage_emergence = -1;
    int stage_flowering = -1;
    int stage_senescence = -1;
    int stage_maturity = -1;
    int stage_vernalisation_end = -1;   // development is slowed by vernalisation before this stage

    // Vernalisation (Wang & Engel 1998 response curve, days per day).
    bool vernalises = false;
    real vern_t_min_c = 0.0, vern_t_opt_c = 0.0, vern_t_max_c = 0.0;
    real vern_days_saturation = 0.0;     // effective vernalisation days for full development rate
    real vern_min_rate_factor = 1.0;     // development rate factor with no vernalisation

    // Canopy cover (fractions of ground cover).
    real cc0_cm2_per_plant = 0.0;        // canopy per seedling at emergence
    real plant_density_per_ha = 0.0;     // established plants at the reference sowing
    real ccx = 0.0;                      // maximum canopy cover
    real cgc_per_gdd = 0.0;              // canopy growth coefficient
    real cdc_per_gdd = 0.0;              // canopy decline coefficient
    real senescence_acceleration = 0.0;  // extra senescence progress per unit (1 - Ks_sen)

    // Transpiration and water stress.
    real kc_tr_x = 0.0;                  // crop transpiration coefficient at full canopy
    real height_max_m = 0.0;             // for the Kc,max wind/humidity term
    real p_depletion = 0.0;              // FAO-56 Table 22 depletion fraction at ETc = 5 mm/day
    bool p_adjust_for_etc = true;        // FAO-56 note to Table 22: p = p_table + 0.04 (5 - ETc)
    real exp_p_upper = 0.0, exp_p_lower = 0.0, exp_shape = 0.0;   // canopy expansion stress
    real sen_p_upper = 0.0, sen_shape = 0.0;                      // early senescence stress

    // Biomass.
    real wp_star_g_m2 = 0.0;             // normalised water productivity, g biomass per m2 per (Tr/ET0 mm/mm)
    real biomass_gdd_full = 0.0;         // GDD/day for unrestricted biomass; below it growth is proportional (0 = off)

    // Harvest index.
    real hi0 = 0.0;
    real hi_build_gdd = 0.0;             // development units after flowering over which HI builds up
    real pollination_window_gdd = 0.0;   // development units after flowering that count for the stress mean
    real hi_water_stress_k = 0.0;        // HI loss per unit mean (1 - Ks_sto) in the window
    real heat_pollination_lo_c = 0.0, heat_pollination_up_c = 0.0;  // Tmax at which pollination starts / fully fails

    // Roots (linear in development units from emergence).
    real zr_min_m = 0.0, zr_max_m = 0.0;
    real root_gdd_full = 0.0;            // development units from sowing at which zr_max is reached

    // Frost kill of the established stand.
    real frost_kill_onset_c = 0.0, frost_kill_full_c = 0.0;
    real snow_insulation_swe_mm = 0.0;   // snow store above which the stand sees frost_snow_floor_c at the coldest
    real frost_snow_floor_c = 0.0;

    // After maturity: grain stays on the plant for a while, then sheds.
    real harvest_window_days = 0.0;      // days after maturity before shedding starts
    real shed_rate_per_day = 0.0;        // fraction of the remaining grain lost per day afterwards

    int stage_count() const { return static_cast<int>(stages.size()); }
    // Canopy cover at emergence for a given density scale.
    real cc0(real density_scale) const { return cc0_cm2_per_plant * 1.0e-4 * plant_density_per_ha * density_scale / 1.0e4; }
};

// First problem with the descriptor ("crop <name>: <field> ..."), or empty.
std::string crop_validation_error(const CropDesc& crop);

} // namespace ps::field
