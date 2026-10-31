// ps/field/management_desc.h - descriptors of the field management layer (farming F1c): the operation
// effect record, tillage and residue parameters of a soil, sowing parameters of a crop, weed groups and
// herbicides. Descriptors only; loaders are in ps/field/field_io.h and the data formats in docs/data_formats/
// (operation.md, weed_group.md, herbicide.md, soil.md "management", crop.md "management"). Every number in a
// data file carries a [C]/[D]/[M]/[AS] source tag (see soil_desc.h).
//
// Layers. The management layer works on three layers of a cell: the SURFACE (everything lying on the soil:
// loose straw, stubble, weed seed on the ground, lime, ammoniacal N of fresh material), the UPPER layer
// (0 to layer_a_m, 0.10 m) and the LOWER layer (layer_a_m to layer_a_m + layer_b_m, 0.30 m in total). The nutrient
// pools of F1b stay one plough-layer pool each; the surface of F1b (lime_surf, tan_surf) is the surface here.

#pragma once

#include "ps/field/nutrient_desc.h"
#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::field {

constexpr int kWeedKinds = 4;          // annual_grass, annual_broadleaf, perennial, volunteer
constexpr int kMaxMoa = 4;             // distinct herbicide modes of action per field
constexpr int kMaxWeedGroups = 8;
constexpr int kMaxHerbicides = 8;
constexpr int kResPools = 4;           // loose straw (surface), stubble (surface), buried upper, buried lower

// Weed kind names, index = kind.
const char* weed_kind_name(int kind);
int weed_kind_from_name(const std::string& name);   // -1 if unknown

// ----------------------------------------------------------------------------------------------------------
// The operation effect record (PLAN 4.6). One plain-data record per field operation, whoever produces it: a
// data file (physics_sim.operation/1) or, later, a machine tool. It holds no machine types. `coverage` is the
// share of the cell the pass covered (a double pass is 2); every fraction below is "of the covered area".
// A record with every member at its default does nothing and leaves the field bit-identical.
// ----------------------------------------------------------------------------------------------------------
struct TillageEffect {
    real depth_m = 0.0;                 // working depth of the tool pass; 0 = the tool does not engage the soil
    real mix_surface = 0.0;             // share of the surface layer mixed into the worked soil
    real mix_upper = 0.0;               // share of the upper layer (its worked part) stirred through the worked depth
    real mix_lower = 0.0;               // share of the lower layer (its worked part) stirred through the worked depth
    real inversion = 0.0;               // share of the worked soil turned over: surface and upper layer go under, the bottom comes up
    real residue_chop = 0.0;            // share of the loose surface straw chopped (fragmented)
    real loosening = 0.0;               // share of the compaction and consolidation removed in the worked layers
    real seedbed_fineness = 0.0;        // tilth the tool leaves in workable soil, 0 = clods, 1 = fine seedbed
    real crop_kill = 0.0;               // share of the standing crop destroyed (it goes into the residue pools)
    real weed_kill[kWeedKinds] = {0.0, 0.0, 0.0, 0.0};   // share of the emerged weeds destroyed, by kind
    bool active() const {
        if (depth_m > 0.0 || crop_kill > 0.0) return true;
        for (real k : weed_kill)
            if (k > 0.0) return true;
        return false;
    }
};

struct HarvestEffect {
    bool harvest = false;               // a harvest: the grain leaves, the crop ends, stubble and roots stay
    real straw_carried = 0.0;           // share of the harvestable straw that leaves with the grain (combine with a straw trailer)
    real straw_chopped = 0.0;           // share of the straw that stays on the field and is chopped and spread (the rest lies in a swath)
    real grain_loss = 0.0;              // share of the grain dropped on the ground by the machine (header and threshing losses); it seeds volunteers
};

struct SeedPlacement {
    int crop = 0;                       // 1-based crop palette index, 0 = no sowing
    real seeds_per_m2 = 0.0;            // seeds placed per m2 of covered area
    real depth_m = 0.0;                 // placement depth
    real quality = 1.0;                 // share of the seeds placed at that depth in contact with soil (speed, tool wear)
    real residue_tolerance = 0.5;       // 0 = residue in the seed row blocks emergence fully, 1 = the tool cuts through it
};

struct Spray {
    int product = 0;                    // 1-based herbicide palette index, 0 = no spraying
    real dose = 1.0;                    // relative to the label rate
};

struct OperationEffect {
    real coverage = 1.0;                // share of the cell the pass covered (a double pass is a coverage of 2)
    Deposit deposit;                    // F1b: fertiliser, manure, lime
    TillageEffect tillage;              // F1c
    HarvestEffect harvest;
    real straw_pickup = 0.0;            // share of the loose surface straw taken off (baler, rake)
    SeedPlacement seed;
    Spray spray;

    OperationEffect() = default;
    OperationEffect(real cov, const Deposit& d) : coverage(cov), deposit(d) {}   // the F1b form: a pass that only deposits

    // True when the record needs the management layer (anything beyond a deposit).
    bool needs_management() const {
        return tillage.active() || harvest.harvest || straw_pickup > 0.0 || seed.crop != 0 || spray.product != 0;
    }
};

// ----------------------------------------------------------------------------------------------------------
// Soil: tillage, structure, residue decomposition (physics_sim.soil/1, optional "management" block).
// ----------------------------------------------------------------------------------------------------------
struct SoilManagementDesc {
    real layer_a_m = 0.10;              // thickness of the upper layer
    real layer_b_m = 0.20;              // thickness of the lower layer
    // Workability (relative wetness of the root zone: 0 = wilting point, 1 = field capacity).
    real workable_wet_full = 0.75;      // at and below this wetness tillage works fully
    real workable_wet_zero = 1.0;       // at and above this wetness it works not at all (smearing, clods)
    real workable_dry_zero = 0.10;      // at and below this wetness the soil is too hard (clods)
    real workable_dry_full = 0.35;
    // Structure damage (compaction and smearing, 0..1) and tilth.
    real damage_gain = 0.3;             // damage added by a full-depth pass of a tool at zero workability
    real damage_recovery_per_day = 0.0007;
    real damage_cn_increase = 6.0;      // curve number added at damage 1
    real damage_root_penalty = 0.25;    // share of the maximum root depth lost at damage 1
    real loose_cn_decrease = 4.0;       // curve number removed at loosening 1 (rough surface holds rain)
    real loose_stage2_evap_reduction = 0.3;   // share of stage-2 soil evaporation removed at loosening 1 (dry mulch)
    real loose_bd_reduction = 0.2;      // bulk-density reduction at loosening 1 (reported, sets the settling work)
    real settle_per_mm_rain = 0.012;    // share of the loosening lost per mm of rain
    real settle_per_day = 0.002;        // ... and per day
    real tillage_dry_out_mm_per_m = 18.0;   // water lost the day a moist soil is opened, per m of working depth
    // Seedbed (0..1).
    real seedbed_rest = 0.45;           // quality of consolidated, untilled soil
    real seedbed_wet_floor = 0.15;      // quality of a tool pass in soil at zero workability
    real seedbed_settle_per_mm = 0.02;  // approach to seedbed_rest per mm of rain (slaking, crusting)
    real seedbed_settle_per_day = 0.003;
    // Residue.
    real cover_per_t_ha = 0.30;         // soil cover = 1 - exp(-cover_per_t_ha x residue dry matter in t/ha) (Gregory 1982 form)
    real mulch_evap_max_reduction = 0.5;    // evaporation reduction at 100 % cover
    real res_c_fraction = 0.44;         // carbon share of residue dry matter
    real res_k20_per_day = 0.012;       // decomposition of buried residue at 20 degC, optimum moisture
    real res_surface_factor = 0.4;      // loose straw lying on the surface relative to buried
    real res_stubble_factor = 0.25;     // standing stubble relative to buried
    real res_moisture_floor = 0.2;
    real res_chop_gain = 0.5;           // decomposition of fully chopped straw is faster by this share
    real res_assim_eff = 0.4;           // share of decomposed carbon built into microbial biomass and humus
    real res_cn_bio = 9.0;              // C:N of that biomass
    real res_n_floor_kg_ha = 3.0;       // mineral N that immobilisation cannot take
};

// Crop: yield-to-residue split, roots, sowing and establishment (physics_sim.crop/1, optional "management").
struct CropManagementDesc {
    real root_dm_fraction = 0.12;       // root dry matter as a share of aboveground dry matter at harvest
    real root_upper_share = 0.6;        // share of the root residue in the upper layer
    real seed_mass_g = 0.045;           // mass of one kernel
    real seed_viability = 0.95;         // germination capacity
    // Emergence from sowing depth (piecewise linear): full between depth_opt_min and depth_opt_max, falling
    // to depth_surface_factor at 0 and to 0 at depth_zero.
    real depth_opt_min_m = 0.02, depth_opt_max_m = 0.05, depth_zero_m = 0.10, depth_surface_factor = 0.5;
    real seedbed_floor = 0.55;          // emergence factor of the worst seedbed
    real residue_emergence_penalty = 0.35;   // emergence lost at full residue cover for a tool with residue_tolerance 0
    // Germination needs a moist seed layer; a dry seed layer stops development and kills seed.
    real germ_wet_min = 0.15, germ_wet_full = 0.5;   // wetness of the evaporation layer (1 - De/TEW)
    real dry_hazard_per_day = 0.03;
    // Plants per m2 at which canopy and yield reach the reference: ccx x (1-exp(-d/d0)) / (1-exp(-d_ref/d0)).
    real compensation_density_m2 = 40.0;
};

// ----------------------------------------------------------------------------------------------------------
// Weeds (physics_sim.weed_group/1) and herbicides (physics_sim.herbicide/1).
// ----------------------------------------------------------------------------------------------------------
struct WeedGroupDesc {
    std::string name;
    int kind = 0;                       // index into the weed kinds
    std::string volunteer_crop;         // crop name whose shed grain becomes seed of this group (kind volunteer), or empty
    bool perennial = false;
    // Seedbank (seeds per m2; plants per m2 for a perennial's clumps) and its fate.
    real seeds_init_m2 = 0.0;
    real clumps_init_m2 = 0.0;          // perennials: established clumps (plants per m2) at construction
    real init_surface = 0.05, init_upper = 0.65, init_lower = 0.30;   // layer shares at construction
    real decay_surface_per_day = 0.0, decay_upper_per_day = 0.0, decay_lower_per_day = 0.0;
    // Germination and emergence.
    real germ_t_min_c = 0.0, germ_t_opt_c = 0.0, germ_t_max_c = 0.0;   // soil temperature window (triangular response)
    real germ_rate_per_day = 0.0;       // share of the germinable bank germinating per day at the optimum
    real germ_wet_min = 0.15, germ_wet_full = 0.5;
    real emerge_surface = 0.8, emerge_upper = 0.8, emerge_lower = 0.03;   // share of the germinants that reach the light, by layer
    real seedling_g = 0.002;            // biomass of a seedling at emergence
    // Growth: per plant logistic in thermal time, reduced by crop shade.
    real t_base_c = 0.0;
    real growth_per_gdd = 0.0;
    real w_max_g = 0.0;                 // biomass of a full-grown plant without competition
    real area_max_g_m2 = 0.0;           // biomass per m2 the group reaches as a monoculture (growth stops at it)
    real self_thin_per_day = 0.0;       // plant mortality per day at full area_max (self-thinning; zero below 0.8 of it)
    real shade = 0.8;                   // growth reduction at crop canopy cover 1
    real n_conc_pct = 2.5;
    real c_frac = 0.42;
    real cold_kill_onset_c = -15.0, cold_kill_full_c = -25.0;
    // Development in thermal time since emergence.
    real gdd_flower = 0.0, gdd_ripe = 0.0, gdd_death = 0.0;   // death: annuals; ignored for a perennial
    real seeds_per_g = 0.0;             // seeds per g of plant dry matter at ripeness
    real shed_per_day = 0.1;            // share of the seed on the plants shed per day once ripe
    // Perennials: shoots die back below dormancy_t_c and regrow from the clumps.
    real dormancy_t_c = 3.0, senesce_per_day = 0.05, regrow_g = 0.05;
    // Competition (Cousens 1985 hyperbola): yield loss % = I d / (1 + I d / A), d = plants per m2.
    real cousens_i_pct = 0.0, cousens_a_pct = 0.0;
    real rel_emerge_decay = 0.5;        // I falls by exp(-this x lag/100 GDD) for weeds younger than the crop
    real resistance_init = 1.0e-6;      // share of the population resistant to each mode of action at construction
};

struct HerbicideKindResponse {
    real ed50_rel = 0.0;                // dose (relative to the label rate) killing half; 0 = no effect on this kind
    real slope = 2.0;                   // log-logistic slope
    real stage_gdd_full = 0.0;          // weed thermal time up to which the efficacy is complete
    real stage_gdd_zero = 0.0;          // ... and from which it is zero
};

struct HerbicideCropResponse {
    std::string crop;
    real max_bbch = 0.0;                // application up to this crop stage is safe
    real damage_in = 0.0;               // share of the stand lost at the label dose inside the window
    real damage_out = 0.0;              // ... outside the window
};

struct HerbicideDesc {
    std::string name;
    std::string moa;                    // mode-of-action group (HRAC); resistance is tracked per group and moa
    int moa_index = -1;                 // resolved by the Field
    real label_rate = 1.0;              // l or kg per ha (information; doses are relative to it)
    HerbicideKindResponse kind[kWeedKinds];
    real resistance_ed50_factor = 100.0;   // ED50 multiplier of the resistant biotype
    real residual_half_life_days = 0.0;
    real residual_germ_kill = 0.0;      // share of emerging weed seedlings killed at residual activity 1
    real damage_dose_exponent = 1.5;    // crop damage ~ dose^exponent
    real default_damage_in = 0.0, default_damage_out = 0.0, default_max_bbch = 0.0;   // for crops not listed
    std::vector<HerbicideCropResponse> crops;
};

// First problem with the descriptor ("soil <name>: management.<field> ..."), or empty.
std::string soil_management_validation_error(const std::string& soil_name, const SoilManagementDesc& m);
std::string crop_management_validation_error(const std::string& crop_name, const CropManagementDesc& m);
std::string weed_group_validation_error(const WeedGroupDesc& w);
std::string herbicide_validation_error(const HerbicideDesc& h);
std::string operation_validation_error(const std::string& name, const OperationEffect& e);

} // namespace ps::field
