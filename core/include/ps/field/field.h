// ps/field/field.h - the field: a world-aligned agronomy grid with soil water
// (FAO-56) and a crop (AquaCrop style), stepped once per game day.
// Farming plan sections 3, 4.1-4.3 (F1a).
//
// Persistence. A Field is a stored object. Its state changes ONLY through
// step_day (weather in, growth and decay out) and through operations
// (sow, harvest, ...). Nothing in it is a function of the calendar date: the
// date reaches the model only through the weather records and the thermal time
// they produce. An unharvested crop stays in its cell until an operation removes
// it: it ripens, its canopy declines, its grain sheds, and the cell still holds
// it years later.
//
// Lazy stepping. A field stores the last day it was stepped (last_day());
// advance_to(day, source) replays the missed days from the cached daily weather
// record, so catch-up after a long idle spell is exactly the stepped run (the
// tests compare bit for bit). checkpoint()/restore() copy the whole state.
//
// Grid. Structure of arrays, row-major (index = iy * nx + ix), cells of
// cell_size_m (default 4 m) aligned to the world axes: cell (ix, iy) covers
// [origin + ix * size, origin + (ix + 1) * size). Cells never read each other,
// so step_cells(begin, end, w) over disjoint ranges can run on several threads
// and give the same state as one sequential pass (tests: 1 vs 10 threads).
//
// Water. Two buckets per cell below one surface: the root zone (depth zr,
// FAO-56 depletion D1) and the soil below it down to the soil depth (D2). Rain
// minus runoff enters the root zone, drainage out of the root zone fills the
// layer below and what that cannot hold leaves the profile. When the roots
// deepen, the strip they take over moves its water from D2 to D1 (and back at
// harvest), so total water is conserved. Evaporation of the surface layer is
// tracked separately (De, FAO-56 ch. 7) and drawn from the root-zone bucket.
// The ledger (precipitation, runoff, evaporation, transpiration, drainage,
// test-only irrigation, snow, storage) closes to rounding; see
// water_balance_residual_mm.
//
// Stress hook. Daily growth is multiplied by
//   min(stress_n, stress_pk, stress_ph, stress_weed) * stress_rotation
// (law of the minimum over the non-water factors, then the rotation factor);
// water enters through the transpiration itself. The stored factors are set by
// set_stress (1.0 by default); with FieldConfig::nutrients_enabled the model's
// own N, P/K and pH factors (NutrientView::f_n / f_pk / f_ph) multiply the
// stored n, pk and ph factors, so an external factor still works as a cap.
//
// Nutrients (F1b, nutrients_enabled). Per cell, kg/ha: nitrate in the two water
// layers, ammonium and a slow and a fresh organic pool in the root-zone layer,
// the ammoniacal N of freshly applied material lying on the surface (volatilises
// or infiltrates, first order and competing), crop N; plant-available and
// reserve P2O5 and K2O; an alkalinity pool (kg CaO/ha, pH = ph_init + alk /
// buffer) and lime pending in a surface and an incorporated pool. Materials are
// put on the field by apply(); crops take N along a critical dilution curve, P
// and K in proportion to growth. Ledgers of N, P2O5, K2O and CaO close per cell
// (n_balance_residual etc.). Everything is per cell and per day, so the
// stepping guarantees above (lazy catch-up, partitions, checkpoints) hold.
// With nutrients_enabled = false none of this state exists and the field is
// bit-identical to F1a.

#pragma once

#include "ps/field/crop_desc.h"
#include "ps/field/soil_desc.h"
#include "ps/field/weather_record.h"
#include "ps/types.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace ps::field {

struct FieldConfig {
    int nx = 1, ny = 1;
    real cell_size_m = 4.0;
    real origin_x_m = 0.0, origin_y_m = 0.0;
    std::vector<SoilDesc> soils;          // palette; cell_soil indexes it
    std::vector<CropDesc> crops;          // palette; crop id = index + 1, 0 = no crop
    std::vector<std::uint16_t> cell_soil; // empty = every cell soil 0, else nx * ny entries
    real initial_depletion_fraction = 0.0;  // of the root zone's TAW, and the layer below's
    real co2_ppm = 369.41;                // atmospheric CO2; AquaCrop's reference (factor 1). A crop's co2_response scales WP* and rue
    std::int64_t initial_day = -1;        // the day the initial state belongs to (the next step is initial_day + 1)
    // Nutrients (F1b). Every soil and crop of the palettes then needs its nutrient block.
    bool nutrients_enabled = false;
    std::vector<MaterialDesc> materials;  // palette; Deposit::material = index + 1
    std::vector<MethodDesc> methods;      // palette; Deposit::method = index + 1
    // Management (F1c): tillage state, residues, establishment from sowing, weeds, herbicides. Needs nutrients_enabled,
    // a management block in every soil and crop. false = bit-identical to the F1b field (the vectors below stay empty).
    bool management_enabled = false;
    std::vector<WeedGroupDesc> weeds;     // palette; empty = no weed model
    std::vector<HerbicideDesc> herbicides;   // palette; Spray::product = index + 1
    // Pathogens (F1d): inoculum and suppression per crop family and cell, built by each same-family crop, decaying over
    // break years, raised by surface residue, lowering the growth of the crop of the family through stress_rotation.
    // Needs management_enabled (residue) and a pathogen block and family in every crop. false = bit-identical.
    bool pathogens_enabled = false;
    // Soil organic carbon (F1d part 5): the slow organic N pool becomes humus with a carbon content that decays with the
    // mineralisation, is built from decomposed residue (humification coefficient), and sets the plant-available water of
    // the soil. Needs management_enabled and an organic_matter block in every soil. false = bit-identical.
    bool soc_enabled = false;
};

struct StressFactors {
    real n = 1.0, pk = 1.0, ph = 1.0, weed = 1.0, rotation = 1.0;
};

struct HarvestResult {
    int crop_id = 0;               // 0 = nothing was growing
    real grain_dm_t_ha = 0.0;      // dry matter
    real straw_dm_t_ha = 0.0;      // harvestable straw (see stubble_dm_t_ha below)
    real bbch = 0.0;
    // Nutrients (all zero without nutrients_enabled). Quantities of the crop at harvest, kg/ha.
    real n_grain_kg_ha = 0.0, n_straw_kg_ha = 0.0, n_root_kg_ha = 0.0;
    real p2o5_grain_kg_ha = 0.0, p2o5_straw_kg_ha = 0.0;
    real k2o_grain_kg_ha = 0.0, k2o_straw_kg_ha = 0.0;
    real lodging_loss = 0.0;       // share of the grain lost to lodging
    // Straw split (all zero when nothing grew). straw_dm_t_ha is the harvestable straw (CropDesc::straw_harvest_fraction of
    // the non-grain aboveground biomass); stubble_dm_t_ha the rest (stubble, chaff, leaf litter) that stays on the field.
    real stubble_dm_t_ha = 0.0;
};

// Plain-value view of one cell (copy; nothing here points into the field).
struct CellView {
    int soil = 0;
    int crop_id = 0;
    int stage = -1;
    real bbch = 0.0;
    real gdd_sow = 0.0, vern_days = 0.0;
    real cc = 0.0, stand = 0.0;
    real biomass_g_m2 = 0.0, grain_g_m2 = 0.0;
    real days_ripe = 0.0;
    real zr_m = 0.0;
    real d_root_mm = 0.0, d_sub_mm = 0.0, d_evap_mm = 0.0, swe_mm = 0.0;
    real taw_root_mm = 0.0, tew_mm = 0.0;
    // Last step's fluxes (mm) and stress factors.
    real last_tr_mm = 0.0, last_e_mm = 0.0, last_runoff_mm = 0.0, last_drain_mm = 0.0;
    real last_ks_sto = 1.0, last_ks_exp = 1.0;
};

// Nutrient state of one cell (copy; kg/ha unless noted).
struct NutrientView {
    real no3_root = 0.0, no3_sub = 0.0, nh4 = 0.0, org_slow = 0.0, org_fresh = 0.0, tan_surface = 0.0;
    real nmin = 0.0;                 // no3_root + no3_sub + nh4 (the soil mineral N)
    real n_crop = 0.0, n_root = 0.0, p2o5_crop = 0.0, k2o_crop = 0.0;
    real p2o5_avail = 0.0, p2o5_reserve = 0.0, k2o_avail = 0.0, k2o_reserve = 0.0;
    real p_test_mg100g = 0.0, k_test_mg100g = 0.0;   // CAL values of the plough layer
    real ph = 0.0, alk_cao = 0.0, lime_surface = 0.0, lime_incorporated = 0.0;
    real soil_temp_c = 0.0;
    real f_n = 1.0, f_pk = 1.0, f_ph = 1.0;          // the growth factors of the last step
    real lodging_loss = 0.0;
};

// Cumulative nutrient ledgers of one cell since construction (kg/ha).
struct NutrientLedger {
    // N
    real n_initial = 0.0;            // all pools at construction
    real n_fert_mineral = 0.0;       // ammoniacal + urea + nitrate N of applied material
    real n_fert_organic = 0.0;       // organic N of applied material
    real n_deposition = 0.0;
    real n_fixation = 0.0;           // taken from the air by the crop
    real n_removed = 0.0;            // leaves with harvested grain and straw
    real n_leached = 0.0;            // nitrate leaving the profile
    real n_volatilised = 0.0;        // ammonia
    real n_denitrified = 0.0;
    // P2O5, K2O
    real p_initial = 0.0, p_applied = 0.0, p_removed = 0.0;
    real k_initial = 0.0, k_applied = 0.0, k_removed = 0.0;
    // Calcium equivalent (kg CaO/ha): applied includes the (negative) neutralising value of acidifying fertiliser
    real cao_applied = 0.0, cao_lost = 0.0;
    // Diagnostics, not part of the balance
    real n_mineralised = 0.0;        // organic -> ammonium
};

// Management state of one cell (copy; management_enabled only). Residue masses are dry matter in t/ha.
struct WeedGroupView {
    real seeds_surface = 0.0, seeds_upper = 0.0, seeds_lower = 0.0, seeds_total = 0.0;   // seeds per m2 in the bank
    real plants_m2 = 0.0, biomass_g_m2 = 0.0, seed_on_plants_m2 = 0.0, tt_gdd = 0.0;
    real resistant_plants[kMaxMoa] = {};   // share of the plants resistant to each mode of action
    real resistant_seeds[kMaxMoa] = {};    // ... of the seed (bank and plants)
};
struct ManagementView {
    real straw_t_ha = 0.0, stubble_t_ha = 0.0, buried_upper_t_ha = 0.0, buried_lower_t_ha = 0.0;
    real residue_c_kg_ha = 0.0, residue_n_kg_ha = 0.0;
    real cover = 0.0;                    // soil cover by residue, 0..1
    real loosening = 0.0, damage = 0.0, seedbed = 0.0;
    real soc_gain_c_kg_ha = 0.0;         // carbon built into humus from decomposed residue since construction
    real plants_m2 = 0.0;                // established crop plants
    real sown_seeds_m2 = 0.0;            // seeds sown and not yet emerged (0 once emerged)
    real weed_factor = 1.0;              // growth factor from weed competition of the last step
    real weed_transpiration_mm = 0.0;    // cumulative transpiration of the weeds since construction (part of the water ledger's transpiration)
    std::vector<WeedGroupView> weeds;
    std::vector<real> herbicide_residual;   // residual activity per herbicide
};
// Pathogen state of one cell (copy; pathogens_enabled only). Index = family index (family_names()).
struct PathogenView {
    std::vector<real> inoculum;          // P per family, 0..1
    std::vector<real> suppression;       // S per family, 0..1 (the antagonists of take-all decline)
    real loss_untreated = 0.0;           // yield loss the growing crop was given at emergence (before any fungicide)
    real fungicide_protection = 0.0;     // protection of the growing crop by sprays now
    real growth_factor = 1.0;            // stress_rotation of the last step
};
// Cumulative carbon ledger of the residue and weed pools (kg C/ha).
struct CarbonLedger {
    real c_from_crop = 0.0;              // crop biomass entering the residue pools (harvest, termination, damage)
    real c_weed_growth = 0.0;            // carbon fixed by weeds
    real c_removed = 0.0;                // straw pickup, weed carbon leaving with it
    real c_co2 = 0.0;                    // respired during decomposition
    real c_humus = 0.0;                  // assimilated into microbial biomass and humus (kept); with soc_enabled: humified into the humus pool
    // soc_enabled only (zero otherwise):
    real c_humus_decay = 0.0;            // humus carbon respired (the mineralisation of the humus)
    real c_amendment = 0.0;              // carbon of applied organic material that joins the humus (its slow organic N x humus C:N)
    real c_humus_initial = 0.0;          // humus carbon at construction
};
// Soil organic carbon of one cell (copy; soc_enabled only).
struct SocView {
    real hum_c_kg_ha = 0.0;              // humus carbon
    real soc_pct = 0.0;                  // ... as % of the dry mass of the layer
    real soc_pct_initial = 0.0;
    real cn = 0.0;                       // C:N of the humus pool (hum_c / slow organic N)
    real dfc = 0.0;                      // change of theta_fc (profile mean) caused by the change of SOC since construction, m3/m3
    real taw_total_mm = 0.0;             // total available water of the profile now
};

// Cumulative water ledger of one cell since construction (mm).
struct WaterLedger {
    real precipitation = 0.0;
    real runoff = 0.0;
    real evaporation = 0.0;
    real transpiration = 0.0;
    real drainage = 0.0;
    real irrigation = 0.0;   // test-only irrigate_to_capacity
    real initial_storage = 0.0;
};

// Every per-cell array of the field (also the checkpoint payload).
struct FieldState {
    std::int64_t last_day = -1;
    // water
    std::vector<real> swe, d_root, d_sub, d_evap, zr;
    // crop
    std::vector<std::uint8_t> crop, stage;
    std::vector<real> gdd_sow, gdd_em, vern_days, cc, cc_peak, t_sen, biomass, grain, stand;
    std::vector<real> hi_gdd, pol_sum, pol_n, sto_sum, sto_n, days_ripe;   // pol_*: pollination sums, sto_*: stomatal HI factor sums
    // hooks
    std::vector<real> stress_n, stress_pk, stress_ph, stress_weed, stress_rotation, residue_evap_factor;
    // last-step diagnostics (not model state, but kept so a restore reproduces CSV rows)
    std::vector<real> last_tr, last_e, last_runoff, last_drain, last_ks_sto, last_ks_exp;
    // ledger
    std::vector<real> cum_precip, cum_runoff, cum_evap, cum_tr, cum_drain, cum_irrig, initial_storage;
    // nutrients (empty unless FieldConfig::nutrients_enabled)
    std::vector<real> no3_1, no3_2, nh4, org_slow, org_fresh, tan_surf, kv_w, ki_w, kir_w, inc_h, soil_t;
    std::vector<real> n_crop, n_root, p_crop, k_crop, p_av, p_res, k_av, k_res, alk, lime_surf, lime_inc, lodge;
    std::vector<real> f_n, f_pk, f_ph;
    std::vector<real> l_n_init, l_n_fmin, l_n_forg, l_n_dep, l_n_fix, l_n_rem, l_n_leach, l_n_nh3, l_n_den, l_n_min;
    std::vector<real> l_p_init, l_p_app, l_p_rem, l_k_init, l_k_app, l_k_rem, l_ca_app, l_ca_loss;
    // management (empty unless FieldConfig::management_enabled)
    std::array<std::vector<real>, kResPools> res_c, res_n;      // loose straw, stubble, buried upper, buried lower (kg C, kg N per ha)
    std::array<std::vector<real>, 2> res_p, res_k;              // P2O5, K2O still in the loose straw and the stubble
    std::vector<real> res_chop, loose, damage, seedbed, soc, tilth_e2;
    std::vector<real> sow_seeds, sow_fe, sow_viab, plants;      // sow_seeds < 0: legacy sow (full stand), 0: nothing sown
    std::vector<real> w_seed_s, w_seed_a, w_seed_b, w_dens, w_bio, w_tt, w_n, w_seed_pl, w_res_pl, w_res_sd;   // [cell * groups (* moa)]
    std::vector<real> h_act;                                    // [cell * herbicides]
    std::vector<real> l_c_crop, l_c_wg, l_c_rem, l_c_co2;
    std::vector<real> weed_f;                                   // weed competition factor (1 = none); always sized
    std::vector<real> cum_tr_weed;                              // transpiration of the weeds (part of cum_tr), mm; sized only with weed groups
    // pathogens (empty unless FieldConfig::pathogens_enabled)
    std::vector<real> path_p, path_s;                           // [cell * families]
    std::vector<real> path_loss0;                               // untreated loss fraction of the growing crop, fixed at emergence
    std::vector<std::uint8_t> res_crop;                         // crop id whose residue lies on the surface (0 = none)
    // soil organic carbon (empty unless FieldConfig::soc_enabled)
    std::vector<real> hum_c, hum_c0, soc_dfc, l_c_hdec, l_c_amend;
};

struct FieldCheckpoint {
    int nx = 0, ny = 0;
    FieldState state;
};

class Field {
public:
    // Throws std::invalid_argument for an inconsistent configuration.
    explicit Field(const FieldConfig& config);

    int nx() const { return nx_; }
    int ny() const { return ny_; }
    int cell_count() const { return nx_ * ny_; }
    real cell_size_m() const { return cfg_.cell_size_m; }
    const FieldConfig& config() const { return cfg_; }
    int cell_index(int ix, int iy) const { return iy * nx_ + ix; }
    // Cell containing the world point, -1 outside the grid.
    int cell_at_world(real x_m, real y_m) const;
    // Area of one cell, m2.
    real cell_area_m2() const { return cfg_.cell_size_m * cfg_.cell_size_m; }

    std::int64_t last_day() const { return s_.last_day; }

    // --- stepping ---
    // One game day for every cell; w.day must equal last_day() + 1.
    void step_day(const DailyWeather& w);
    // The same for cells [begin, end) only, without advancing last_day().
    // Call finish_day(w.day) after every range has been stepped.
    void step_cells(int begin, int end, const DailyWeather& w);
    void finish_day(std::int64_t day);
    // Steps day by day until last_day() == day (no-op when already there;
    // std::invalid_argument when asked to go back).
    void advance_to(std::int64_t day, DailyWeatherSource& source);

    // --- operations (test-only minimal set; F1c replaces them by data-driven effects) ---
    // Sows crop_id (1-based palette index) into the cell, replacing whatever stands there.
    void sow(int cell, int crop_id, real density_scale = 1.0);
    void sow_all(int crop_id, real density_scale = 1.0);
    // Removes the crop; returns what grew. The root zone returns to the fallow depth.
    // With nutrients, grain always leaves the field; `straw_removed` (0..1) is the share of the straw
    // taken away, the rest stays: its N joins the fresh organic pool, its P2O5 and K2O the available
    // pools. The root N joins the fresh pool.
    HarvestResult harvest(int cell, real straw_removed = 1.0);
    HarvestResult harvest_all(real straw_removed = 1.0);   // field mean

    // One operation's effect (nutrients_enabled only; std::logic_error otherwise). F1b: the deposit.
    // The effect covers `coverage` of the cell (a double pass is a coverage of 2).
    // With management_enabled the whole record applies in the order: crop and weed kill, harvest, straw pickup, tillage,
    // deposit, spraying, sowing. `harvest_out` (optional) receives what a harvest effect took.
    void apply(int cell, const OperationEffect& effect, HarvestResult* harvest_out = nullptr);
    HarvestResult apply_all(const OperationEffect& effect);   // the field mean of the harvest effects

    // With pathogens_enabled the model owns `rotation` (std::logic_error when f.rotation != 1).
    void set_stress(int cell, const StressFactors& f);
    // Test only: every day starts with the profile brought back to field
    // capacity (booked as irrigation, so the balance still closes).
    void set_irrigate_to_capacity(bool on) { irrigate_ = on; }

    // --- queries ---
    CellView view(int cell) const;
    WaterLedger ledger(int cell) const;
    // precipitation + irrigation + initial storage - runoff - ET - drainage -
    // final storage (mm); zero up to rounding.
    real water_balance_residual_mm(int cell) const;
    real storage_mm(int cell) const;   // plant-available water above wilting point in the whole soil + snow
    real taw_root_mm(int cell) const;
    // Nutrients (std::logic_error without nutrients_enabled).
    NutrientView nutrient_view(int cell) const;
    NutrientLedger nutrient_ledger(int cell) const;
    // Initial + applied - removed - lost - stored, per nutrient; zero up to rounding.
    real n_balance_residual(int cell) const;
    real p_balance_residual(int cell) const;
    real k_balance_residual(int cell) const;
    real cao_balance_residual(int cell) const;
    bool nutrients_enabled() const { return cfg_.nutrients_enabled; }
    // Management (std::logic_error without management_enabled).
    bool management_enabled() const { return cfg_.management_enabled; }
    ManagementView management_view(int cell) const;
    CarbonLedger carbon_ledger(int cell) const;
    // Carbon in - out - stored in the residue pools and living weeds; zero up to rounding.
    real c_balance_residual(int cell) const;

    // Pathogens (std::logic_error without pathogens_enabled).
    bool pathogens_enabled() const { return cfg_.pathogens_enabled; }
    const std::vector<std::string>& family_names() const { return family_names_; }
    PathogenView pathogen_view(int cell) const;
    // Test/scenario hook: sets the inoculum and suppression of a family in a cell (both in [0, 1)).
    void set_pathogen(int cell, const std::string& family, real inoculum, real suppression);

    // Soil organic carbon (std::logic_error without soc_enabled).
    bool soc_enabled() const { return cfg_.soc_enabled; }
    SocView soc_view(int cell) const;

    // --- persistence ---
    FieldCheckpoint checkpoint() const;
    // Throws std::invalid_argument when the checkpoint does not fit this grid.
    void restore(const FieldCheckpoint& cp);
    std::uint64_t fold_hash(std::uint64_t h) const;

private:
    struct DayFlows {            // what the water balance of the day hands to the nutrients
        real taw1 = 0.0, dp = 0.0, drain = 0.0, bio_before = 0.0, bbch = 0.0, shed_g_m2 = 0.0, gdd = 0.0;
        int stage = 0;
        bool emerged = false;
    };
    void step_cell(std::size_t c, const DailyWeather& w);
    void set_root_depth(std::size_t c, real z_new);
    // Nutrients
    void init_nutrients();
    void nutrient_growth_factors(std::size_t c, int st, bool emerged, real& f_n, real& f_pk, real& f_ph);
    void step_nutrients(std::size_t c, const DailyWeather& w, const DayFlows& fl);
    real ph_of(std::size_t c) const;
    real n_storage(std::size_t c) const;
    void drop_crop_nutrients(std::size_t c);   // a crop that leaves without a harvest returns all it holds
    // Management (management.cpp, weeds.cpp)
    void init_management();
    void manage_prestep(std::size_t c, const DailyWeather& w);
    void manage_poststep(std::size_t c, const DailyWeather& w, const DayFlows& fl);
    void establish(std::size_t c, const CropDesc& cr);
    void apply_management(std::size_t c, const OperationEffect& e, int phase, HarvestResult* harvest_out);   // phase 1: kill, harvest, pickup, tillage; 2: spray, sowing
    real weed_competition(std::size_t c) const;
    real weed_transpiration(std::size_t c, real taw1, real cc_star, real et0, real& cover) const;   // mm of the day's potential use, water-stress reduced; cover = combined weed canopy cover
    real crop_bbch(std::size_t c) const;
    void apply_deposit(std::size_t c, const OperationEffect& e);
    void reset_crop_state(std::size_t c);
    void do_tillage(std::size_t c, const TillageEffect& t, real cov);
    void do_harvest_mgmt(std::size_t c, const HarvestEffect& h, real cov, HarvestResult* out);
    void do_seeding(std::size_t c, const SeedPlacement& sp, real cov);
    void do_spray(std::size_t c, const Spray& sp, real cov);
    void do_straw_pickup(std::size_t c, real f);
    void crop_to_residue(std::size_t c, real fraction, bool with_grain);   // kills `fraction` of the standing crop into the residue pools
    void clear_crop(std::size_t c);
    void step_weeds(std::size_t c, const DailyWeather& w);
    // Pathogens (pathogens.cpp)
    void init_pathogens();
    void step_pathogens(std::size_t c, const DayFlows& fl);
    void pathogen_emergence(std::size_t c);              // fixes the untreated loss of the crop that emerged today
    real fungicide_protection(std::size_t c) const;      // of the growing crop, 0..1
    // Soil organic carbon (soc.cpp)
    void init_soc();
    void step_soc(std::size_t c);                        // moves the water capacity with the humus carbon, conserving the stored water
    // Soil water constants of a cell: the soil file's value plus the change the humus carbon has made (0 without soc_enabled).
    real dfc_of(std::size_t c) const { return s_.soc_dfc.empty() ? 0.0 : s_.soc_dfc[c]; }
    real fc_of(std::size_t c) const { return soil_of(c).theta_fc + dfc_of(c); }
    real taw_per_m_of(std::size_t c) const { return soil_of(c).taw_mm_per_m() + 1000.0 * dfc_of(c); }
    real tew_of(std::size_t c) const { return soil_of(c).tew_mm() + 1000.0 * dfc_of(c) * soil_of(c).evaporation_depth_m; }
    real total_taw_of(std::size_t c) const { return taw_per_m_of(c) * soil_of(c).depth_m; }
    void weed_to_residue(std::size_t c, std::size_t gi, real fraction);
    void release_residue_pk(std::size_t c, int pool, real fraction);
    real workability(std::size_t c) const;
    real rel_wet(std::size_t c) const;
    real residue_dm_surface(std::size_t c) const;
    real residue_cover(std::size_t c) const;
    int group_count() const { return static_cast<int>(cfg_.weeds.size()); }
    int herbicide_count() const { return static_cast<int>(cfg_.herbicides.size()); }
    const SoilDesc& soil_of(std::size_t c) const { return cfg_.soils[soil_idx_[c]]; }

    FieldConfig cfg_;
    std::vector<real> co2_f_;          // per crop (index = id - 1): WP* / rue multiplier of cfg_.co2_ppm
    std::vector<int> weed_vol_crop_;   // per weed group: crop id whose shed grain seeds it, 0 = none
    std::vector<std::string> family_names_;   // pathogens: sorted crop families
    std::vector<int> crop_family_;            // per crop (index = id - 1): index into family_names_
    std::vector<real> fam_decay_, fam_sdecay_;   // per family: daily factors of the inoculum and of the suppression
    int moa_count_ = 0;
    int nx_ = 0, ny_ = 0;
    std::vector<std::uint16_t> soil_idx_;
    FieldState s_;
    bool irrigate_ = false;
};

} // namespace ps::field
