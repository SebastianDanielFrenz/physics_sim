#!/usr/bin/env python3
"""Writes the F1c data files of ps_field: the management blocks of the two reference soils and of winter wheat (added
in place to data/soils/*.json and data/crops/winter_wheat.json), the weed groups, herbicides and operation records.
Deterministic; re-running gives the same bytes. Source tags: [C] cited, [D] derived, [M] measured, [AS] assumed.
Values recalled from the literature without a document in hand are tagged [AS] and say so."""
import json
import os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data")


def dump(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
        f.write("\n")


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


# --------------------------------------------------------------------------------------------------- soils
def soil_management(sandy):
    v = {
        "layer_a_m": 0.10, "layer_b_m": 0.20,
        "workable_wet_full": 0.80 if sandy else 0.70, "workable_wet_zero": 1.0 if sandy else 0.98,
        "workable_dry_zero": 0.05 if sandy else 0.10, "workable_dry_full": 0.25 if sandy else 0.35,
        "damage_gain": 0.20 if sandy else 0.35, "damage_recovery_per_day": 0.0007,
        "damage_cn_increase": 6.0, "damage_root_penalty": 0.25, "loose_cn_decrease": 4.0,
        "loose_stage2_evap_reduction": 0.3, "loose_bd_reduction": 0.2,
        "settle_per_mm_rain": 0.012, "settle_per_day": 0.002, "tillage_dry_out_mm_per_m": 18.0,
        "seedbed_rest": 0.5 if sandy else 0.45, "seedbed_wet_floor": 0.15,
        "seedbed_settle_per_mm": 0.02, "seedbed_settle_per_day": 0.003,
        "cover_per_t_ha": 0.30, "mulch_evap_max_reduction": 0.5, "res_c_fraction": 0.44,
        "res_k20_per_day": 0.012, "res_surface_factor": 0.4, "res_stubble_factor": 0.25, "res_moisture_floor": 0.2,
        "res_chop_gain": 0.5, "res_assim_eff": 0.4, "res_cn_bio": 9.0, "res_n_floor_kg_ha": 3.0,
    }
    s = {
        "layer_a_m": "[AS] thickness of the upper layer of the management model (the seedbed and the layer a cultivator mixes).",
        "layer_b_m": "[AS] thickness of the lower layer; the two together are the 0.30 m plough layer of the nutrient model.",
        "workable_wet_full": "[AS] relative root-zone wetness (0 wilting point, 1 field capacity) up to which tillage works fully; the workable window of a loam is wet-limited a little below field capacity (recalled, no document).",
        "workable_wet_zero": "[AS] wetness at which tillage works not at all (smearing, clods).",
        "workable_dry_zero": "[AS] wetness below which the soil is too hard to work (clods).",
        "workable_dry_full": "[AS] wetness above which a dry-side limit no longer applies.",
        "damage_gain": "[AS] structure damage (compaction/smearing) added by a full-depth pass in soil at zero workability; a silt loam smears more than a sandy loam.",
        "damage_recovery_per_day": "[AS] recovery of structure damage by frost and biology, about 1 - exp(-0.25) per year.",
        "damage_cn_increase": "[AS] SCS curve number added at damage 1 (surface sealing, slower infiltration).",
        "damage_root_penalty": "[AS] share of the maximum rooting depth lost at damage 1.",
        "loose_cn_decrease": "[AS] SCS curve number removed at loosening 1 (rough surface holds rain; the NRCS curve-number tables give about 4-6 units between a smooth and a rough fallow, recalled).",
        "loose_stage2_evap_reduction": "[AS] share of stage-2 soil evaporation removed at loosening 1 (a dry loose layer is a mulch).",
        "loose_bd_reduction": "[AS] bulk-density reduction at loosening 1 (reported only).",
        "settle_per_mm_rain": "[AS] share of the loosening lost per mm of rain (about 20 mm halves it).",
        "settle_per_day": "[AS] share of the loosening lost per day.",
        "tillage_dry_out_mm_per_m": "[AS] water lost the day a moist soil is opened, per m of working depth (about 4-5 mm for a 25 cm ploughing).",
        "seedbed_rest": "[AS] seedbed quality of consolidated untilled soil (0 clods .. 1 fine).",
        "seedbed_wet_floor": "[AS] seedbed quality a tool leaves at zero workability.",
        "seedbed_settle_per_mm": "[AS] approach of the seedbed to its rest value per mm of rain (slaking, crusting).",
        "seedbed_settle_per_day": "[AS] approach per day.",
        "cover_per_t_ha": "[AS] soil cover by residue = 1 - exp(-A m), m in t/ha dry matter; Gregory (1982) gives the form, A about 0.2-0.4 per t/ha for small-grain straw is recalled from the NRCS residue-cover tables, not checked in a document.",
        "mulch_evap_max_reduction": "[C] FAO-56 (Allen et al. 1998) ch. 11: organic mulch reduces soil evaporation by about 5 % for each 10 % of the soil covered, i.e. 50 % at full cover.",
        "res_c_fraction": "[C] carbon share of plant dry matter, about 0.44 (0.40-0.48 in the straw literature).",
        "res_k20_per_day": "[AS] first-order decomposition of buried straw at 20 degC and optimum moisture; with the Q10 and moisture factors the loss of buried wheat straw is about 60-70 % in the first year (recalled range from litterbag studies).",
        "res_surface_factor": "[AS] loose straw lying on the surface decomposes at this share of the buried rate (drier, less soil contact).",
        "res_stubble_factor": "[AS] standing stubble decomposes at this share of the buried rate.",
        "res_moisture_floor": "[AS] moisture factor of dry residue; rises linearly to 1 with the wetness of its layer.",
        "res_chop_gain": "[AS] decomposition of fully chopped straw is faster by this share (more surface, better contact).",
        "res_assim_eff": "[AS] share of decomposed carbon built into microbial biomass and kept as humus (carbon use efficiency of soil microbes about 0.3-0.5, recalled).",
        "res_cn_bio": "[AS] C:N of the microbial biomass, about 8-9 (Cleveland and Liptzin 2007, recalled).",
        "res_n_floor_kg_ha": "[AS] mineral N the decomposers cannot take.",
    }
    v["source"] = s
    return v


def add_soil_management():
    for fn, sandy in (("silt_loam_loess.json", False), ("sandy_loam.json", True)):
        p = os.path.join(ROOT, "soils", fn)
        d = load(p)
        d["management"] = soil_management(sandy)
        # keep "source" last
        src = d.pop("source")
        d["source"] = src
        dump(p, d)


# --------------------------------------------------------------------------------------------------- crop
def add_crop_management():
    p = os.path.join(ROOT, "crops", "winter_wheat.json")
    d = load(p)
    d["biomass"].pop("rs_ref_mj_m2", None)
    d["biomass"]["source"].pop("rs_ref_mj_m2", None)
    d["biomass"]["rue_g_mj"] = 1.4
    d["biomass"]["source"]["rue_g_mj"] = (
        "[AS] radiation-use cap on the daily biomass gain: at most rue x Rs x canopy cover (g/m2/day). The AquaCrop gain WP* x Tr/ET0 "
        "does not depend on the radiation at all, so a canopy at low spring light would grow as fast as in June; 1.4 g per MJ of "
        "global radiation is the order of the total-biomass radiation-use efficiency of wheat (about 2.8 g/MJ PAR, Gallagher and "
        "Biscoe 1978, recalled, not re-read). It bites in February-March only: B at BBCH 31 stays 6-7 t/ha where field figures are 2-3 t/ha - "
        "a structural property of the Tr/ET0 growth law, recorded in the F1c report.")
    d["yield"]["straw_harvest_fraction"] = 0.68
    d["yield"]["source"]["straw_harvest_fraction"] = (
        "[D] share of the non-grain aboveground biomass a harvest can carry off: LfL Bayern Bas22 Tab. 1a gives a straw:grain ratio of 0.8 "
        "(both 86 % DM); with the harvest index 0.46 the non-grain biomass is 0.54 per unit biomass, harvestable straw 0.8 x 0.46 = 0.368, "
        "0.368 / 0.54 = 0.68. The rest (cutting height, chaff, leaf litter) stays as stubble.")
    n = d["nutrients"]
    n["n_grain_share"] = 0.755
    n["p2o5_conc_pct_dm"] = 0.617
    n["p2o5_grain_share"] = 0.694
    n["k2o_conc_pct_dm"] = 1.176
    n["k2o_grain_share"] = 0.25
    s = n["source"]
    s["p2o5_conc_pct_dm"] = ("[D] LfL Bayern Bas22 Tab. 1a: 1.04 kg P2O5/dt grain with straw (grain 0.80, straw 0.24); total aboveground P2O5 per t grain "
                             "= 8.0 / 0.694 = 11.53 kg over the aboveground biomass 0.86 / 0.46 = 1.870 t dry matter = 0.617 %.")
    s["p2o5_grain_share"] = "[D] grain 8.0 kg / (8.0 + 2.4 / 0.68) kg P2O5 per t grain, LfL Bayern Bas22 Tab. 1a with the straw harvest share 0.68."
    s["k2o_conc_pct_dm"] = ("[D] LfL Bayern Bas22 Tab. 1a: 1.67 kg K2O/dt grain with straw (grain 0.55, straw 1.12); total aboveground K2O per t grain "
                            "= 5.5 / 0.25 = 22.0 kg over 1.870 t dry matter = 1.176 %.")
    s["k2o_grain_share"] = "[D] grain 5.5 kg / (5.5 + 11.2 / 0.68) kg K2O per t grain, LfL Bayern Bas22 Tab. 1a with the straw harvest share 0.68."
    s["n_grain_share"] = ("[D] LfL Bayern Bas22 Tab. 1a: N removal 1.81 kg/dt grain, 2.21 with straw (straw 0.40 kg/dt). With the harvestable share "
                          "0.68 of the non-grain biomass the grain holds 1.81 / (1.81 + 0.40 / 0.68) = 0.755 of the aboveground N.")
    d["management"] = {
        "root_dm_fraction": 0.12, "root_upper_share": 0.45, "seed_mass_g": 0.045, "seed_viability": 0.95,
        "depth_opt_min_m": 0.02, "depth_opt_max_m": 0.05, "depth_zero_m": 0.10, "depth_surface_factor": 0.5,
        "seedbed_floor": 0.55, "residue_emergence_penalty": 0.35, "germ_wet_min": 0.15, "germ_wet_full": 0.5,
        "dry_hazard_per_day": 0.03, "compensation_density_m2": 40.0,
        "source": {
            "root_dm_fraction": "[AS] root dry matter as a share of the aboveground dry matter at harvest; cereal root:shoot at maturity is about 0.1-0.2 (recalled).",
            "root_upper_share": "[AS] share of the root residue in the top 0.10 m of the 0.30 m management layers.",
            "seed_mass_g": "[D] thousand-kernel weight 45 g, the middle of the 40-50 g of German winter wheat varieties.",
            "seed_viability": "[AS] germination capacity of drilled seed (the German seed law minimum is 85 %, certified lots are above 92 %).",
            "depth_opt_min_m": "[AS] the recommended drilling depth of winter wheat is 2-4 cm (extension advice, recalled).",
            "depth_opt_max_m": "[AS] see depth_opt_min_m.",
            "depth_zero_m": "[AS] emergence fails from about 10 cm (the coleoptile of modern semi-dwarf wheats is short).",
            "depth_surface_factor": "[AS] emergence of seed lying on the surface (birds, drying) relative to the optimum depth.",
            "seedbed_floor": "[AS] emergence factor of the worst seedbed; field emergence of drilled wheat is typically 75-85 % of the seeds, which this reproduces together with seed_viability.",
            "residue_emergence_penalty": "[AS] emergence lost at full residue cover for a tool with residue_tolerance 0.",
            "germ_wet_min": "[AS] wetness of the evaporation layer (1 - De/TEW) below which the seed does not germinate.",
            "germ_wet_full": "[AS] wetness from which germination runs at full rate.",
            "dry_hazard_per_day": "[AS] share of the sown seed that dies per day in a seed layer below germ_wet_min.",
            "compensation_density_m2": "[AS] constant d0 of the yield-density law 1 - exp(-d/d0) (Willey and Heath 1969 form); wheat compensates thin stands by tillering, 100 plants/m2 give about 92 % of the 250 plants/m2 reference."
        }
    }
    src = d.pop("source")
    d["source"] = src
    dump(p, d)


# --------------------------------------------------------------------------------------------------- weeds
def weed(name, kind, volunteer_crop, seedbank, germ, growth, dev, perennial, comp, resistance, note):
    d = {"format": "physics_sim.weed_group/1", "name": name, "kind": kind}
    if volunteer_crop:
        d["volunteer_crop"] = volunteer_crop
    d["seedbank"] = seedbank
    d["germination"] = germ
    d["growth"] = growth
    d["development"] = dev
    if perennial:
        d["perennial"] = perennial
    d["competition"] = comp
    d["resistance"] = resistance
    d["source"] = note
    return d


AS_ALL = "[AS] "


def srcs(keys, text):
    return {k: AS_ALL + text for k in keys}


def write_weeds():
    sb_keys = ["seeds_init_m2", "init_surface", "init_upper", "init_lower", "decay_surface_per_day", "decay_upper_per_day", "decay_lower_per_day"]
    g_keys = ["t_min_c", "t_opt_c", "t_max_c", "rate_per_day", "wet_min", "wet_full", "emerge_surface", "emerge_upper", "emerge_lower", "seedling_g"]
    gr_keys = ["t_base_c", "per_gdd", "w_max_g", "area_max_g_m2", "self_thin_per_day", "shade", "n_conc_pct", "c_frac", "cold_kill_onset_c", "cold_kill_full_c"]
    dv_keys = ["gdd_flower", "gdd_ripe", "gdd_death", "seeds_per_g", "shed_per_day"]
    cp_keys = ["cousens_i_pct", "cousens_a_pct", "rel_emerge_decay"]

    blackgrass = weed(
        "black_grass", "annual_grass", "",
        {"seeds_init_m2": 400.0, "init_surface": 0.05, "init_upper": 0.65, "init_lower": 0.30,
         "decay_surface_per_day": 0.006, "decay_upper_per_day": 0.003, "decay_lower_per_day": 0.0015,
         "source": srcs(sb_keys, "Alopecurus myosuroides, an autumn-germinating grass weed of German cereals. A moderately infested field; the seedbank halves in about a year in the top layer (persistence of 2-3 years, recalled from the black-grass literature, no document in hand).")},
        {"t_min_c": 2.0, "t_opt_c": 12.0, "t_max_c": 28.0, "rate_per_day": 0.02, "wet_min": 0.15, "wet_full": 0.5,
         "emerge_surface": 0.7, "emerge_upper": 0.3, "emerge_lower": 0.01, "seedling_g": 0.002,
         "source": srcs(g_keys, "autumn flush of black-grass at soil temperatures of 5-15 degC with a seed layer moist enough; shares emerging by layer fall with depth (most emergence from the top 2-3 cm). Recalled, no document in hand.")},
        {"t_base_c": 0.0, "per_gdd": 0.014, "w_max_g": 3.0, "area_max_g_m2": 350.0, "self_thin_per_day": 0.03, "shade": 0.7, "n_conc_pct": 2.0, "c_frac": 0.42,
         "cold_kill_onset_c": -18.0, "cold_kill_full_c": -28.0,
         "source": srcs(gr_keys, "logistic growth in thermal time to the biomass of a plant under a crop; frost-hardy winter annual. c_frac 0.42 is the usual carbon share of herbaceous dry matter.")},
        {"gdd_flower": 1700.0, "gdd_ripe": 2150.0, "gdd_death": 2300.0, "seeds_per_g": 35.0, "shed_per_day": 0.12,
         "source": srcs(dv_keys, "thermal time since emergence to flowering (June) and seed ripeness (mid July, a few days before wheat harvest); 100-300 seeds per plant.")},
        None,
        {"cousens_i_pct": 0.8, "cousens_a_pct": 70.0, "rel_emerge_decay": 0.5,
         "source": srcs(cp_keys, "Cousens hyperbola (Cousens 1985): yield loss of about 0.5-1 % per plant/m2 at low density for black-grass under winter wheat, saturating at 40-90 % (recalled range, not checked in a document).")},
        {"init": 1.0e-6, "source": {"init": "[AS] share of the seedbank resistant to each herbicide mode of action in a field never selected; resistant biotypes are present at about 1e-6 (Powles and Yu 2010, order of magnitude, recalled)."}},
        "F1c weed group: black-grass. Every number is [AS]; ranges recalled, not verified against a document.")
    cleavers = weed(
        "cleavers", "annual_broadleaf", "",
        {"seeds_init_m2": 150.0, "init_surface": 0.05, "init_upper": 0.65, "init_lower": 0.30,
         "decay_surface_per_day": 0.005, "decay_upper_per_day": 0.0025, "decay_lower_per_day": 0.0012,
         "source": srcs(sb_keys, "Galium aparine; seeds persist for several years. Recalled.")},
        {"t_min_c": 0.0, "t_opt_c": 10.0, "t_max_c": 25.0, "rate_per_day": 0.015, "wet_min": 0.15, "wet_full": 0.5,
         "emerge_surface": 0.6, "emerge_upper": 0.3, "emerge_lower": 0.02, "seedling_g": 0.01,
         "source": srcs(g_keys, "autumn and early-spring emergence of cleavers (large seed, emerges from depth better than grasses). Recalled.")},
        {"t_base_c": 0.0, "per_gdd": 0.011, "w_max_g": 6.0, "area_max_g_m2": 450.0, "self_thin_per_day": 0.03, "shade": 0.5, "n_conc_pct": 2.5, "c_frac": 0.42,
         "cold_kill_onset_c": -15.0, "cold_kill_full_c": -25.0,
         "source": srcs(gr_keys, "scrambling annual that keeps growing under the crop canopy (weak shade effect). Recalled.")},
        {"gdd_flower": 1800.0, "gdd_ripe": 2200.0, "gdd_death": 2400.0, "seeds_per_g": 12.0, "shed_per_day": 0.15,
         "source": srcs(dv_keys, "flowers in June, seed ripe at the end of July. Recalled.")},
        None,
        {"cousens_i_pct": 1.5, "cousens_a_pct": 60.0, "rel_emerge_decay": 0.4,
         "source": srcs(cp_keys, "Cousens hyperbola; cleavers is one of the most competitive broadleaf weeds of cereals (1-3 % per plant/m2 recalled).")},
        {"init": 1.0e-6, "source": {"init": "[AS] as for black_grass."}},
        "F1c weed group: cleavers. Every number is [AS].")
    couch = weed(
        "couch_grass", "perennial", "",
        {"seeds_init_m2": 20.0, "clumps_init_m2": 3.0, "init_surface": 0.05, "init_upper": 0.65, "init_lower": 0.30,
         "decay_surface_per_day": 0.004, "decay_upper_per_day": 0.002, "decay_lower_per_day": 0.001,
         "source": srcs(sb_keys + ["clumps_init_m2"], "Elymus repens spreads mainly by rhizomes; the seedbank is small. clumps_init_m2 is a patchy infestation. Recalled.")},
        {"t_min_c": 3.0, "t_opt_c": 15.0, "t_max_c": 30.0, "rate_per_day": 0.003, "wet_min": 0.15, "wet_full": 0.5,
         "emerge_surface": 0.5, "emerge_upper": 0.3, "emerge_lower": 0.02, "seedling_g": 0.003,
         "source": srcs(g_keys, "spring germination of couch grass seed (rare). Recalled.")},
        {"t_base_c": 2.0, "per_gdd": 0.012, "w_max_g": 25.0, "area_max_g_m2": 800.0, "self_thin_per_day": 0.02, "shade": 0.6, "n_conc_pct": 1.8, "c_frac": 0.42,
         "cold_kill_onset_c": -20.0, "cold_kill_full_c": -30.0,
         "source": srcs(gr_keys, "vigorous perennial grass; rhizome clumps regrow from stored reserves each spring (regrow_g). Recalled.")},
        {"gdd_flower": 1000.0, "gdd_ripe": 1500.0, "gdd_death": 0.0, "seeds_per_g": 0.4, "shed_per_day": 0.1,
         "source": srcs(dv_keys, "a perennial does not die of age (gdd_death unused); few seeds. Recalled.")},
        {"dormancy_t_c": 4.0, "senesce_per_day": 0.04, "regrow_g": 0.5,
         "source": srcs(["dormancy_t_c", "senesce_per_day", "regrow_g"], "shoots die back below 4 degC mean temperature (4 % per day) and regrow from rhizome reserves (0.5 g per clump when active).")},
        {"cousens_i_pct": 0.9, "cousens_a_pct": 55.0, "rel_emerge_decay": 0.0,
         "source": srcs(cp_keys, "Cousens hyperbola per clump/m2; perennials are present before the crop (no emergence-time penalty). Recalled.")},
        {"init": 1.0e-6, "source": {"init": "[AS] as for black_grass."}},
        "F1c weed group: couch grass, a perennial. Every number is [AS].")
    volunteer = weed(
        "volunteer_wheat", "volunteer", "winter_wheat",
        {"seeds_init_m2": 10.0, "init_surface": 0.2, "init_upper": 0.5, "init_lower": 0.3,
         "decay_surface_per_day": 0.006, "decay_upper_per_day": 0.003, "decay_lower_per_day": 0.002,
         "source": srcs(sb_keys, "shed wheat grain: low dormancy, most of it germinates in the first autumn or is lost. seeds_init_m2 a small carry-over. Recalled.")},
        {"t_min_c": 1.0, "t_opt_c": 14.0, "t_max_c": 30.0, "rate_per_day": 0.05, "wet_min": 0.15, "wet_full": 0.5,
         "emerge_surface": 0.5, "emerge_upper": 0.5, "emerge_lower": 0.05, "seedling_g": 0.02,
         "source": srcs(g_keys, "wheat seed germinates readily; the surface layer loses seed to birds and drying. Recalled.")},
        {"t_base_c": 0.0, "per_gdd": 0.010, "w_max_g": 10.0, "area_max_g_m2": 400.0, "self_thin_per_day": 0.03, "shade": 0.5, "n_conc_pct": 2.0, "c_frac": 0.43,
         "cold_kill_onset_c": -12.0, "cold_kill_full_c": -22.0,
         "source": srcs(gr_keys, "a volunteer wheat plant under a crop of the same species.")},
        {"gdd_flower": 1700.0, "gdd_ripe": 2480.0, "gdd_death": 2800.0, "seeds_per_g": 22.0, "shed_per_day": 0.05,
         "source": srcs(dv_keys, "the stage table of winter_wheat (flowering 1760, maturity 2480 units); seeds_per_g from the 45 g thousand-kernel weight.")},
        None,
        {"cousens_i_pct": 0.4, "cousens_a_pct": 40.0, "rel_emerge_decay": 0.3,
         "source": srcs(cp_keys, "volunteer cereals compete weakly with a crop of the same species. Recalled.")},
        {"init": 1.0e-6, "source": {"init": "[AS] as for black_grass."}},
        "F1c weed group: volunteer winter wheat from shed grain. Every number is [AS].")
    for w in (blackgrass, cleavers, couch, volunteer):
        dump(os.path.join(ROOT, "weeds", w["name"] + ".json"), w)


def herbicide(name, moa, label, targets, res_factor, half_life, germ_kill, crops, exponent, note):
    t = {}
    for k, (ed50, slope, full, zero) in targets.items():
        t[k] = {"ed50_rel": ed50, "slope": slope, "stage_gdd_full": full, "stage_gdd_zero": zero,
                "source": {"ed50_rel": "[AS] dose relative to the label rate that kills half the susceptible plants; the label rate is set to kill > 95 %.",
                           "slope": "[AS] log-logistic slope (typical herbicide dose-response slopes 2-4, recalled).",
                           "stage_gdd_full": "[AS] weed thermal time since emergence up to which the efficacy is complete.",
                           "stage_gdd_zero": "[AS] thermal time from which the efficacy is zero (large weeds escape)."}}
    c = {}
    for k, (mb, di, do) in crops.items():
        c[k] = {"max_bbch": mb, "damage_in": di, "damage_out": do,
                "source": {"max_bbch": "[AS] last crop stage with the safe window.", "damage_in": "[AS] share of the stand lost at the label dose inside the window.",
                           "damage_out": "[AS] ... outside the window."}}
    return {
        "format": "physics_sim.herbicide/1", "name": name, "moa": moa, "label_rate": label,
        "targets": t,
        "resistance": {"ed50_factor": res_factor, "source": {"ed50_factor": "[AS] ED50 multiplier of the resistant biotype (target-site resistance gives resistance factors of 10 to over 100, recalled)."}},
        "residual": {"half_life_days": half_life, "germ_kill": germ_kill,
                     "source": {"half_life_days": "[AS] half-life of the soil-residual activity.", "germ_kill": "[AS] share of emerging seedlings killed at full residual activity."}},
        "crops": c,
        "damage": {"dose_exponent": exponent, "source": {"dose_exponent": "[AS] crop damage grows with dose^exponent (an overdose hurts more than proportionally)."}},
        "source": note,
    }


def write_herbicides():
    # fenoxaprop-type grass herbicide (ACCase, HRAC 1): kills grasses, not broadleaves
    dump(os.path.join(ROOT, "herbicides", "grass_herbicide.json"),
         herbicide("grass_herbicide", "group_1_accase", 1.0,
                   {"annual_grass": (0.25, 3.0, 400.0, 900.0), "volunteer": (0.3, 3.0, 400.0, 900.0), "perennial": (0.8, 2.0, 400.0, 900.0)},
                   100.0, 10.0, 0.2, {"winter_wheat": (39.0, 0.0, 0.5), "default": (30.0, 0.05, 0.8)}, 1.5,
                   "F1c herbicide: a generic post-emergence grass herbicide (ACCase type). Every number [AS]; no product label was consulted."))
    # a sulfonylurea-type broadleaf herbicide (ALS, HRAC 2)
    dump(os.path.join(ROOT, "herbicides", "broadleaf_herbicide.json"),
         herbicide("broadleaf_herbicide", "group_2_als", 1.0,
                   {"annual_broadleaf": (0.2, 3.0, 500.0, 1100.0), "annual_grass": (0.9, 2.0, 300.0, 700.0)},
                   50.0, 20.0, 0.1, {"winter_wheat": (39.0, 0.0, 0.4), "default": (30.0, 0.1, 0.8)}, 1.5,
                   "F1c herbicide: a generic post-emergence broadleaf herbicide (ALS type). Every number [AS]."))
    # glyphosate-type non-selective (EPSPS, HRAC 9); the crop is never safe
    dump(os.path.join(ROOT, "herbicides", "total_herbicide.json"),
         herbicide("total_herbicide", "group_9_epsps", 1.0,
                   {"annual_grass": (0.2, 3.0, 800.0, 2000.0), "annual_broadleaf": (0.2, 3.0, 800.0, 2000.0),
                    "perennial": (0.5, 2.5, 5000.0, 6000.0), "volunteer": (0.2, 3.0, 800.0, 2000.0)},
                   30.0, 0.0, 0.0, {"default": (0.0, 1.0, 1.0)}, 1.0,
                   "F1c herbicide: a generic non-selective herbicide (glyphosate type, no soil residual): it kills any emerged crop; use before sowing or on stubble. Every number [AS]."))


# --------------------------------------------------------------------------------------------------- operations
def tag(keys, text):
    return {k: "[AS] " + text for k in keys}


def op(name, note, **blocks):
    d = {"format": "physics_sim.operation/1", "name": name}
    d.update(blocks)
    d["source"] = note
    return d


def write_operations():
    def tillage(depth, **kw):
        t = {"depth_m": depth}
        t.update(kw)
        keys = list(t.keys())
        t["source"] = tag(keys, "parameter set of a tool pass; shares of the covered area (see docs/data_formats/operation.md). Recalled tool behaviour, not measured.")
        return t

    dump(os.path.join(ROOT, "operations", "plough.json"), op(
        "plough", "F1c operation: mouldboard plough, 25 cm. The inversion buries surface residue, seed and lime in the lower layer and brings the lower soil up.",
        tillage=tillage(0.25, mix_surface=0.0, mix_upper=0.0, mix_lower=0.0, inversion=0.9, residue_chop=0.0, loosening=0.8,
                        seedbed_fineness=0.25, crop_kill=1.0,
                        weed_kill={"annual_grass": 0.95, "annual_broadleaf": 0.95, "perennial": 0.6, "volunteer": 0.95})))
    dump(os.path.join(ROOT, "operations", "cultivator.json"), op(
        "cultivator", "F1c operation: field cultivator / chisel, 12 cm, no inversion. Mixes the worked layers and keeps part of the residue near the surface.",
        tillage=tillage(0.12, mix_surface=0.5, mix_upper=0.6, mix_lower=0.3, inversion=0.0, residue_chop=0.2, loosening=0.5,
                        seedbed_fineness=0.55, crop_kill=1.0,
                        weed_kill={"annual_grass": 0.8, "annual_broadleaf": 0.8, "perennial": 0.3, "volunteer": 0.8})))
    dump(os.path.join(ROOT, "operations", "stubble_cultivator.json"), op(
        "stubble_cultivator", "F1c operation: shallow stubble tillage, 6 cm: mixes the straw into the top layer, chops it, makes weed and volunteer seed germinate.",
        tillage=tillage(0.06, mix_surface=0.5, mix_upper=0.4, mix_lower=0.0, inversion=0.0, residue_chop=0.5, loosening=0.2,
                        seedbed_fineness=0.3, crop_kill=0.0,
                        weed_kill={"annual_grass": 0.7, "annual_broadleaf": 0.7, "perennial": 0.2, "volunteer": 0.7})))
    dump(os.path.join(ROOT, "operations", "seedbed_harrow.json"), op(
        "seedbed_harrow", "F1c operation: seedbed harrow, 5 cm: fine seedbed, shallow mixing.",
        tillage=tillage(0.05, mix_surface=0.4, mix_upper=0.5, mix_lower=0.0, inversion=0.0, residue_chop=0.1, loosening=0.2,
                        seedbed_fineness=0.9, crop_kill=0.0,
                        weed_kill={"annual_grass": 0.85, "annual_broadleaf": 0.85, "perennial": 0.2, "volunteer": 0.85})))
    dump(os.path.join(ROOT, "operations", "weeder.json"), op(
        "weeder", "F1c operation: mechanical weeding (harrow / hoe) at 3 cm in the standing crop: kills small weeds, damages a little of the crop.",
        tillage=tillage(0.03, mix_surface=0.0, mix_upper=0.1, mix_lower=0.0, inversion=0.0, residue_chop=0.0, loosening=0.1,
                        seedbed_fineness=0.0, crop_kill=0.03,
                        weed_kill={"annual_grass": 0.6, "annual_broadleaf": 0.6, "perennial": 0.0, "volunteer": 0.4})))
    drill = {"crop": "winter_wheat", "seeds_per_m2": 300.0, "depth_m": 0.035, "quality": 0.95, "residue_tolerance": 0.5}
    drill["source"] = tag(["seeds_per_m2", "depth_m", "quality", "residue_tolerance"],
                          "300 kernels/m2 at 3.5 cm (German practice 250-400 seeds/m2, 2-4 cm); quality 0.95 of the seeds placed in contact with soil; a drill with disc coulters cuts through half of the residue in the row.")
    drill["source"]["crop"] = "[AS] sown crop."
    dump(os.path.join(ROOT, "operations", "drill.json"), op("drill", "F1c operation: a drill with disc coulters sowing winter wheat. The sowing is the seed record; the soil disturbance is the small tillage block.",
                                                           tillage=tillage(0.04, mix_surface=0.05, mix_upper=0.1, mix_lower=0.0, inversion=0.0, residue_chop=0.0, loosening=0.05,
                                                                           seedbed_fineness=0.0, crop_kill=0.0),
                                                           seed=drill))
    direct = dict(drill)
    direct["residue_tolerance"] = 0.3
    direct["source"] = dict(drill["source"])
    direct["source"]["residue_tolerance"] = "[AS] a direct drill meets the whole untilled residue and cuts through less of it."
    dump(os.path.join(ROOT, "operations", "direct_drill.json"), op("direct_drill", "F1c operation: no-till drill, narrow slot, little soil disturbance.",
                                                                  tillage=tillage(0.03, mix_surface=0.0, mix_upper=0.03, mix_lower=0.0, inversion=0.0, residue_chop=0.0,
                                                                                  loosening=0.0, seedbed_fineness=0.0, crop_kill=0.0),
                                                                  seed=direct))
    hv = {"straw_carried": 0.0, "straw_chopped": 0.9, "grain_loss": 0.015, "source": tag(["straw_carried", "straw_chopped", "grain_loss"], "combine with a chopper/spreader: the straw is chopped and spread, nothing is carried off; header and threshing losses of 1-2 % of the grain stay on the ground (extension figures for a well-set combine, recalled).")}
    dump(os.path.join(ROOT, "operations", "combine_straw_chopped.json"), op("combine_straw_chopped", "F1c operation: combine harvest, straw chopped and spread.", harvest=hv))
    hv2 = {"straw_carried": 1.0, "straw_chopped": 0.0, "grain_loss": 0.015, "source": tag(["straw_carried", "straw_chopped", "grain_loss"], "combine with a swath and a baler pass: all harvestable straw is carried off; grain losses as for the chopping combine.")}
    dump(os.path.join(ROOT, "operations", "combine_straw_off.json"), op("combine_straw_off", "F1c operation: combine harvest, the harvestable straw baled and removed.", harvest=hv2))
    dump(os.path.join(ROOT, "operations", "spray_grass_herbicide.json"), op("spray_grass_herbicide", "F1c operation: field sprayer with the grass herbicide at label dose.", spray={"product": "grass_herbicide", "dose": 1.0}))
    dump(os.path.join(ROOT, "operations", "spray_broadleaf_herbicide.json"), op("spray_broadleaf_herbicide", "F1c operation: field sprayer with the broadleaf herbicide at label dose.", spray={"product": "broadleaf_herbicide", "dose": 1.0}))
    dump(os.path.join(ROOT, "operations", "spray_total_herbicide.json"), op("spray_total_herbicide", "F1c operation: field sprayer with the non-selective herbicide at label dose.", spray={"product": "total_herbicide", "dose": 1.0}))


if __name__ == "__main__":
    add_soil_management()
    add_crop_management()
    write_weeds()
    write_herbicides()
    write_operations()
    print("ok")
