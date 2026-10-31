// ps/field/nutrient_desc.h - nutrient, material and operation descriptors of the
// field model (farming F1b). Descriptors only; loaders are in ps/field/field_io.h.
// Every number in a data file carries a [C]/[D]/[M]/[AS] source tag (see soil_desc.h).
//
// Units: kg/ha for pools and ledgers, kg/t fresh matter for material contents,
// t/ha fresh matter for rates, mg/100 g for soil-test values (VDLUFA CAL), kg CaO/ha
// for the calcium-equivalent pool (CaO equivalent of the neutralising value).

#pragma once

#include "ps/types.h"

#include <string>

namespace ps::field {

// The nutrient block of a soil (physics_sim.soil/1, optional "nutrients" block).
struct SoilNutrientDesc {
    // --- nitrogen ---
    real no3_init_kg_ha = 0.0;           // initial nitrate N of the whole profile (split by root depth)
    real nh4_init_kg_ha = 0.0;           // initial ammonium N (top layer)
    real org_n_kg_ha = 0.0;              // slow organic N pool (top layer, humus)
    real min_rate_per_day_20c = 0.0;     // first-order mineralisation of the slow pool at 20 degC, optimum moisture
    real min_q10 = 2.0;                  // temperature response (also used for the fresh pool, nitrification, denitrification)
    real min_moisture_floor = 0.3;       // moisture factor at a dry root zone; rises linearly to 1 at field capacity
    real fresh_rate_per_day_20c = 0.0;   // mineralisation of the fresh organic N pool (manure, residues)
    real nitrif_rate_per_day_20c = 0.0;  // NH4 -> NO3
    real denit_rate_per_day_20c = 0.0;   // NO3 loss of the root layer when wet
    real denit_wetness = 0.9;            // relative root-zone wetness above which denitrification runs (linear to 1)
    real soil_t_lag_days = 10.0;         // soil temperature = air temperature low-passed with this time constant (>= 1)
    real soil_t_init_c = 10.0;           // soil temperature at construction
    real soil_t_ramp_c = 5.0;            // biological rates ramp linearly from 0 at 0 degC to full at this soil temperature
    real n_deposition_kg_ha_day = 0.0;   // atmospheric N deposition, booked as ammonium
    real nh3_temp_q10 = 2.0;             // volatilisation rate factor per 10 K around 15 degC
    // --- phosphorus and potassium (plough layer, soil-test based) ---
    real soil_test_kg_per_mg100g = 42.0; // kg nutrient/ha in the plough layer per mg/100 g soil-test value (depth x bulk density)
    real p_test_init_mg100g = 0.0;       // P2O5 (CAL), initial
    real p_reserve_kg_ha = 0.0;          // slowly available P2O5 reserve
    real p_eq_mg100g = 0.0;              // test value the reserve replenishes to / fixes down to
    real p_release_per_day = 0.0;        // reserve -> available, fraction of the gap to equilibrium per day
    real p_fix_per_day = 0.0;            // available above equilibrium -> reserve, fraction of the excess per day
    real p_rel_b = 0.0;                  // Mitscherlich coefficient per mg/100 g of the relative-yield curve
    real p_target_mg100g = 0.0;          // test value at which the relative yield is 1 (lower bound of VDLUFA class C)
    real k_test_init_mg100g = 0.0;
    real k_reserve_kg_ha = 0.0;
    real k_eq_mg100g = 0.0;
    real k_release_per_day = 0.0;
    real k_fix_per_day = 0.0;
    real k_rel_b = 0.0;
    real k_target_mg100g = 0.0;
    // --- reaction (pH in CaCl2) and calcium-equivalent pool ---
    real ph_init = 0.0;
    real buffer_cao_kg_ha_per_ph = 0.0;  // kg CaO/ha that move the pH by one unit
    real cao_loss_kg_ha_per_mm_drain = 0.0;   // unavoidable CaO loss (leaching + neutralisation) per mm of water leaving the root zone
    real lime_react_per_day = 0.0;       // reaction rate of lime pending in the plough layer (incorporated)
    real lime_surface_factor = 0.4;      // relative reaction rate of lime left on the surface
    real ph_avail_full = 6.0;            // pH at and above which mineralisation and P/K availability are unrestricted
    real ph_avail_floor = 4.5;           // pH at and below which they are at avail_floor_factor
    real avail_floor_factor = 0.6;
};

// The nutrient block of a crop (physics_sim.crop/1, optional "nutrients" block).
struct CropNutrientDesc {
    // Critical N dilution curve (Justes et al. 1994): Nc[%] = a W^-b for W >= w_min (t/ha dry matter), constant below.
    real nc_a = 0.0, nc_b = 0.0, nc_w_min_t_ha = 0.0, nc_const_pct = 0.0;
    real n_min_conc_pct = 0.0;           // N concentration at which growth stops (dilution limit)
    real n_stress_start_t_ha = 0.0;      // the N factor applies once the aboveground dry matter exceeds this
    real n_uptake_max_kg_ha_day = 0.0;
    real n_uptake_frac_per_day = 0.0;    // share of the available soil mineral N the crop can take per day
    real n_residual_kg_ha = 0.0;         // soil mineral N the roots cannot take
    real n_root_fraction = 0.0;          // root N as a fraction of aboveground N
    real n_grain_share = 0.0;            // share of aboveground N in the grain at harvest
    real n_fixation_fraction = 0.0;      // share of the N demand fixed from the air (legumes)
    real p2o5_conc_pct_dm = 0.0, p2o5_grain_share = 0.0;
    real k2o_conc_pct_dm = 0.0, k2o_grain_share = 0.0;
    // Yield response to the soil reaction (pH CaCl2): 1 inside [ph_opt_low, ph_opt_high], linear down to ph_floor_factor at ph_floor.
    real ph_opt_low = 0.0, ph_opt_high = 0.0, ph_floor = 0.0, ph_floor_factor = 1.0;
    // Lodging from surplus mineral N during stem elongation: loss = max_loss x clamp((Nmin - start)/(full - start)).
    real lodging_nmin_start_kg_ha = 0.0, lodging_nmin_full_kg_ha = 0.0, lodging_max_loss = 0.0;
    real lodging_bbch_from = 0.0, lodging_bbch_to = 0.0;   // window in which the surplus mineral N counts
};

// A fertiliser, manure or liming material (physics_sim.field_material/1).
struct MaterialDesc {
    std::string name;
    std::string kind;                    // mineral | liquid | slurry | manure | digestate | lime
    real dm_frac = 1.0;                  // dry matter, fraction of fresh matter
    real density_kg_m3 = 1000.0;         // converts m3/ha to t/ha
    // Contents per tonne fresh matter.
    real n_total_kg_t = 0.0;
    real nh4_n_kg_t = 0.0, urea_n_kg_t = 0.0, no3_n_kg_t = 0.0;    // organic N = total - these
    real p2o5_kg_t = 0.0, k2o_kg_t = 0.0;
    real cao_kg_t = 0.0;                 // neutralising value as CaO: > 0 liming (pending pool), < 0 acidifying (booked at once)
    // Ammonia behaviour of the ammoniacal + urea N left on the surface (first-order rates per day at 15 degC and no rain).
    real k_vol_per_day = 0.0;
    real k_inf_per_day = 0.0;
    real k_inf_per_mm_rain = 0.0;        // infiltration rate added per mm of rain that day
    real org_slow_fraction = 0.0;        // share of the organic N entering the slow (humus) pool, the rest the fresh pool

    real org_n_kg_t() const { return n_total_kg_t - nh4_n_kg_t - urea_n_kg_t - no3_n_kg_t; }
};

// An application method (physics_sim.field_methods/1 entry).
struct MethodDesc {
    std::string name;
    real soil_placed_fraction = 0.0;     // share of the ammoniacal N placed in the soil at once (injection)
    real k_vol_factor = 1.0;             // multiplies the surface volatilisation rate (band spreading)
    real incorporation_delay_h = -1.0;   // default hours until the surface is worked in; < 0 = never
    bool incorporated = false;           // lime reaction speed: placed in the plough layer
};

// Deposit of a material. Part of the operation effect record (see OperationEffect).
struct Deposit {
    int material = 0;                    // 1-based palette index, 0 = no deposit
    int method = 0;                      // 1-based method palette index
    real rate_t_ha = 0.0;                // fresh matter per hectare covered
    real incorporation_delay_h = -2.0;   // < -1.5 = the method's default; < 0 = none; else hours
};

// The complete operation effect record (deposit, tillage, harvest, sowing, spraying) is OperationEffect in
// ps/field/management_desc.h; F1b used only its deposit.

// First problem with the descriptor ("soil <name>: nutrients.<field> ..."), or empty.
std::string soil_nutrient_validation_error(const std::string& soil_name, const SoilNutrientDesc& sn);
std::string crop_nutrient_validation_error(const std::string& crop_name, const CropNutrientDesc& cn);
std::string material_validation_error(const MaterialDesc& m);
std::string method_validation_error(const MethodDesc& m);

} // namespace ps::field
