// ps/field/crop_desc.h - crop descriptor (physics_sim.crop/1, F1a): an
// AquaCrop-style daily crop (thermal time to BBCH stages, vernalisation,
// canopy cover, transpiration x water productivity -> biomass, harvest index
// -> yield). Descriptors only; the loader is ps/field/field_io.h. Every number
// in a data file carries a source tag, see soil_desc.h.

#pragma once

#include "ps/field/management_desc.h"
#include "ps/field/nutrient_desc.h"
#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::field {

struct CropStageDesc {
    std::string name;
    real bbch = 0.0;     // BBCH scale code the stage starts at (reporting only)
    real gdd = 0.0;      // development units from sowing at which the stage starts (ascending)
};

// One point of the AquaCrop WP* adjustment for atmospheric CO2 (Eq. 3.11c, Table 3.11c/1: the multiplier of a C3 crop).
struct Co2Point {
    real ppm = 0.0;
    real factor = 1.0;
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
    real biomass_rue_g_mj = 0.0;         // light-limited potential of the daily biomass gain: rue x f_T x Rs x canopy cover g/m2/day (0 = off); the stress factors multiply the smaller of it and the water-limited gain
    real biomass_rue_t_low_c = 0.0;      // f_T: linear in the daily mean air temperature, 0 at and below this ...
    real biomass_rue_t_full_c = 0.0;     // ... and 1 at and above this (both 0 = no temperature response)
    bool biomass_light_primary = false;  // true: the daily gain is the light potential x Tr/Tr_potential (WP* unused); false: the smaller of it and WP* x Tr/ET0
    std::vector<Co2Point> co2_response;  // WP* (and rue) multiplier of a C3 crop over the CO2 concentration, ascending ppm; empty = no adjustment

    // WP* adjustment for the CO2 concentration (AquaCrop Eq. 3.11c/d): 1 + f_type (f_CO2 - 1), f_CO2 interpolated from
    // co2_response, f_type = clamp((40 - WP*)/20, 0, 1). 1 when the table is empty.
    real co2_factor(real ppm) const;
    real biomass_rs_ref_mj_m2 = 0.0;     // daily global radiation (MJ/m2) at and above which biomass growth is unrestricted; below it proportional (0 = off)

    // Harvest index (AquaCrop, Raes et al. reference manual ch. 3.12). HI at the end of
    // yield formation = hi0 x f_pollination x f_stomata x build-up, where
    //   f_pollination = min(1, (1 + excess_fruits/100) x sum_j F_j Ks_pol,j)   (Eq. 3.12i/j: F_j is the
    //                   share of flowers opening on flowering day j, Ks_pol the min of the water,
    //                   cold and heat pollination coefficients; the share not yet open counts as pollinated)
    //   f_stomata     = mean over the yield-formation days of 1 - (1 - Ks_sto^(1/10)) / b   (Eq. 3.12m/n).
    real hi0 = 0.0;
    real hi_build_gdd = 0.0;             // development units after flowering over which HI builds up (also the yield-formation length)
    real straw_harvest_fraction = 1.0;   // share of the non-grain aboveground biomass a harvest can carry off (cutting height, chaff, leaf litter stay as stubble)
    bool pollination = true;             // false: a root crop (yield formation from the "flowering" stage on, no pollination; the pollination keys are then required but unused)
    real flowering_length_gdd = 0.0;     // length of the flowering stage in development units
    real excess_fruits_pct = 0.0;        // excess of potential fruits, %
    real ppol = 0.0;                     // depletion fraction of TAW at which pollination starts to fail (Ks_pol,w linear to 0 at TAW)
    real hi_sto_b = 0.0;                 // coefficient b of the negative effect of stomatal closure, 1 (strong) .. 20 (small)
    real pol_cold_t_c = 0.0, pol_cold_width_k = 0.0;   // Tmin below which pollination fails, and the width to full failure
    real pol_heat_t_c = 0.0, pol_heat_width_k = 0.0;   // Tmax above which pollination fails, and the width to full failure
    real pol_logistic_steepness = 0.0;   // steepness of the logistic Ks curve of the temperature stresses

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

    // Optional nutrient block (F1b); required when the field runs with nutrients.
    bool has_nutrients = false;
    CropNutrientDesc nutrients;

    // Optional management block (F1c); required when the field runs with management_enabled.
    bool has_management = false;
    CropManagementDesc management;

    int stage_count() const { return static_cast<int>(stages.size()); }
    // Canopy cover at emergence for a given density scale.
    real cc0(real density_scale) const { return cc0_cm2_per_plant * 1.0e-4 * plant_density_per_ha * density_scale / 1.0e4; }
};

// First problem with the descriptor ("crop <name>: <field> ..."), or empty.
std::string crop_validation_error(const CropDesc& crop);

} // namespace ps::field
