#!/usr/bin/env python3
"""Writes the F1d crop files of ps_field: winter_barley, winter_rapeseed, grain_maize, sugar_beet (data/crops/).
They start from data/crops/winter_wheat.json; every key a crop does not override keeps the wheat value, with the
source tag downgraded to [AS] and the wheat text quoted ("not crop-specific"), so no wheat citation is claimed for
another crop. Deterministic; re-running gives the same bytes. Tags: [C] cited, [D] derived, [M] fitted/measured, [AS] assumed.

Sources used (all looked up this session unless the text says recalled):
  AquaCrop Reference Manual v7.3 (FAO, July 2026), Annex I (I.2 Maize, I.7 Sugar Beet, I.11 Barley), read from the PDF.
  FAO-56 (Allen et al. 1998) Table 12 (height) and Table 22 (p, rooting depth), fao.org/4/x0490e/.
  LfL Bayern, Basisdaten Duengung und Naehrstofffluesse (Bas22, 10.12.2021), Tab. 1a (nutrient contents).
  DWD CDC annual-reporter phenology of crops (1992-2024) and the Frankfurt/Main station 01420 daily Tmin/Tmax: stage tables.
  Plenet and Lemaire 2000 maize critical N curve 3.4 W^-0.37 (as quoted in Sci. Data 9:277, 2022, PMC9174182).
"""
import copy
import json
import os
import re

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "crops")
ANN = "AquaCrop Reference Manual v7.3 (FAO, July 2026), Annex I"
BAS = "LfL Bayern Bas22 (Basisdaten Duengung und Naehrstofffluesse, 10.12.2021) Tab. 1a"
DWD = ("development units accumulated by the Python port of the model's thermal-time rule (clamped daily Tmin/Tmax, "
       "Tbase/Tupper above) along the Frankfurt/Main daily record (DWD station 01420), 1992-2024, from the year's mean "
       "sowing date to the year's mean date of the DWD phase (annual reporters of all German stations, quality level <= 3; "
       "DWD CDC opendata phenology/annual_reporters/crops), averaged over 32 seasons")


def load(name):
    with open(os.path.join(ROOT, name), encoding="utf-8") as f:
        return json.load(f)


def dump(name, obj):
    with open(os.path.join(ROOT, name), "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
        f.write("\n")


def downgrade(text, crop):
    m = re.match(r"^(\[[A-Z]+\](\+\[[A-Z]+\])?)\s*(.*)$", text, re.S)
    body = m.group(3) if m else text
    return "[AS] value of winter_wheat.json reused for %s, not crop-specific (wheat entry: %s)" % (crop, body[:400])


def put(d, block, key, value, source):
    d[block][key] = value
    d[block].setdefault("source", {})[key] = source


def crop_from_wheat(name, family, text):
    w = load("winter_wheat.json")
    d = copy.deepcopy(w)
    d["name"] = name
    d["family"] = family
    for blk in d.values():
        if isinstance(blk, dict) and "source" in blk:
            for k in list(blk["source"]):
                blk["source"][k] = downgrade(blk["source"][k], name)
    d["source"] = text
    return d


def stages(rows):
    return [{"name": n, "bbch": b, "gdd": g} for n, b, g in rows]


def removal(grain_n, straw_n, grain_p, straw_p, grain_k, straw_k, straw_ratio, hi, frac):
    """Per tonne of fresh grain (kg/dt x 10): grain share of the aboveground nutrient and the concentration in the
    aboveground dry matter (%), with the harvestable straw share `frac` of the non-grain biomass."""
    def share(g, s):
        return g / (g + s * straw_ratio / frac)
    def conc(g, s, dm_fraction_grain):
        total_kg = 10.0 * (g + s * straw_ratio / frac)
        bio = dm_fraction_grain / hi
        return total_kg / (bio * 1000.0) * 100.0
    return share, conc


# ------------------------------------------------------------------------------------------------ winter barley
def winter_barley():
    d = crop_from_wheat("winter_barley", "cereal",
                        "F1d winter barley (family cereal with wheat: both are hosts of take-all and eyespot, see the pathogen "
                        "notes of the F1d report). Annex I.11 Barley values of the AquaCrop reference manual v7.3 are [C]; the "
                        "stage table is [D] from DWD phenology and the Frankfurt station; nutrient removal is [D] from LfL Bayern "
                        "Bas22; whatever else is [AS] says so. No vernalisation is modelled (the crop is only sown in autumn; "
                        "a spring sowing of this file would skip the cold requirement).")
    del d["vernalisation"]
    del d["phenology"]["vernalisation_until"]
    put(d, "thermal", "t_base_c", 0.0, "[C] %s.11 Barley, section 1.1: Tbase 0 degC." % ANN)
    put(d, "thermal", "t_upper_c", 15.0, "[C] %s.11 Barley, section 1.1: Tupper 15 degC (as printed; the wheat annex has 26)." % ANN)
    d["phenology"]["stages"] = stages([("sown", 0, 0), ("emergence", 10, 141), ("stem_elongation", 31, 1318), ("heading", 51, 1606),
                                       ("flowering", 65, 1700), ("senescence", 71, 1850), ("maturity", 87, 2171), ("overripe", 92, 2427)])
    d["phenology"]["source"] = {"stages": (
        "[D] emergence 141, stem_elongation 1318, heading 1606, maturity 2171, overripe 2427 are " + DWD + " (DWD winter barley phases 12 Auflaufen "
        "BBCH 10, 15 Schossen BBCH 31, 18 Aehrenschieben BBCH 51, 21 Gelbreife BBCH 87, 24 Ernte; sd of the seasons 13/85/89/100/103 units; "
        "mean doys sowing 263.8, emergence 275.4, BBCH 31 106.7, heading 133.0, yellow ripe 176.1, harvest 194.0). Flowering 1700 [AS]: "
        "BBCH 65 is not observed; barley flowers at about heading (about 90 units later). Senescence 1850 [AS]: start of canopy decline, "
        "inside the annex range emergence + 900-2000 and about 40 units after flowering ends.")}
    cc = d["canopy"]
    put(d, "canopy", "cc0_cm2_per_plant", 1.5, "[C] %s.11 Barley, section 1.2: soil surface covered by an individual seedling 1.50 cm2/plant." % ANN)
    put(d, "canopy", "plant_density_per_ha", 2400000, "[C] %s.11 Barley, section 1.2: 1 500 000 - 3 000 000 plants per hectare (management); 2.4 million is German practice (about 300 seeds/m2 drilled, 80 %% emerging), inside the range." % ANN)
    put(d, "canopy", "ccx", 0.95, "[C] %s.11 Barley, section 1.2: maximum canopy cover 50-99 %% (management); 0.95 inside the range." % ANN)
    put(d, "canopy", "cgc_per_gdd", 0.008, "[C] %s.11 Barley, section 1.2: canopy growth coefficient 0.008 per GDD." % ANN)
    put(d, "canopy", "cdc_per_gdd", 0.006, "[C] %s.11 Barley, section 1.2: canopy decline coefficient 0.006 per GDD." % ANN)
    put(d, "water", "kc_tr_x", 1.10, "[C] %s.11 Barley, section 2: KcTr,x 1.10." % ANN)
    put(d, "water", "height_max_m", 1.0, "[C] FAO-56 Table 12 (fao.org/4/x0490e/x0490e0b.htm): barley maximum crop height 1 m.")
    put(d, "water", "p_depletion", 0.55, "[C] FAO-56 Table 22 (fao.org/4/x0490e/x0490e0e.htm): barley p = 0.55 at ETc ~ 5 mm/day, rooting depth 1.0-1.5 m.")
    put(d, "water", "exp_p_upper", 0.20, "[C] %s.11 Barley, section 4.1: canopy expansion 0.20." % ANN)
    put(d, "water", "exp_p_lower", 0.65, "[C] %s.11 Barley, section 4.1: canopy expansion 0.65." % ANN)
    put(d, "water", "exp_shape", 3.0, "[C] %s.11 Barley, section 4.1: shape factor for canopy expansion 3.0." % ANN)
    put(d, "water", "sen_p_upper", 0.55, "[C] %s.11 Barley, section 4.1: canopy senescence psen 0.55." % ANN)
    put(d, "water", "sen_shape", 3.0, "[C] %s.11 Barley, section 4.1: shape factor for canopy senescence 3.0." % ANN)
    put(d, "biomass", "wp_star_g_m2", 15.0, "[C] %s.11 Barley, section 3.1: WP* 15.0 g/m2." % ANN)
    put(d, "biomass", "gdd_full", 14.0, "[C] %s.11 Barley, section 4.2: minimum growing degrees for full biomass production 14 degC day." % ANN)
    d["biomass"]["source"]["rue_g_mj"] = ("[AS] 1.1 g/MJ of global radiation as for wheat; Gallagher and Biscoe 1978 (J. Agric. Sci. 91:47-60) "
                                         "treat wheat and barley with the same RUE (about 2.2 g/MJ PAR), recalled for barley, not looked up.")
    put(d, "yield", "hi0", 0.45, "[C] %s.11 Barley, section 3.2: reference harvest index 30-50 %% (cultivar); 0.45 for a modern German variety, inside the range." % ANN)
    put(d, "yield", "hi_build_gdd", 471, "[D] development units from the flowering stage to maturity of the stage table (2171 - 1700).")
    put(d, "yield", "flowering_length_gdd", 200, "[C] %s.11 Barley, section 1.3: length of the flowering stage 150-250 GDD; 200 inside the range." % ANN)
    put(d, "yield", "ppol", 0.85, "[C] %s.11 Barley, section 4.1: pollination fails above a depletion of 0.85 of TAW." % ANN)
    put(d, "yield", "cold_t_c", 5.0, "[C] %s.11 Barley, section 4.2: cold stress for pollination below 5 degC." % ANN)
    put(d, "yield", "heat_t_c", 35.0, "[C] %s.11 Barley, section 4.2: heat stress for pollination above 35 degC." % ANN)
    frac = 0.655
    put(d, "yield", "straw_harvest_fraction", frac, "[D] %s: straw:grain ratio 0.8; with the harvest index 0.45 the harvestable straw is 0.8 x 0.45 = 0.36 of the biomass out of the non-grain 0.55; 0.36/0.55 = 0.655." % BAS)
    put(d, "roots", "zr_max_m", 1.5, "[C] FAO-56 Table 22: barley maximum rooting depth 1.0-1.5 m (upper end); AquaCrop allows up to 2.50 m (management).")
    put(d, "roots", "gdd_full", 1400, "[AS] development units from sowing at which the maximum depth is reached (before flowering).")
    put(d, "frost", "kill_onset_c", -10.0, "[AS] daily minimum at which frost starts to kill an emerged stand; winter barley is less frost-hardy than winter wheat (LT50 about -10 to -15 degC, recalled).")
    put(d, "frost", "kill_full_c", -18.0, "[AS] daily minimum at which the whole stand is killed.")
    n = d["nutrients"]
    sh = lambda g, s: g / (g + s * 0.8 / frac)
    cn = lambda g, s: 10.0 * (g + s * 0.8 / frac) / (0.86 / 0.45 * 1000.0) * 100.0
    put(d, "nutrients", "n_grain_share", round(sh(2.11, 0.50), 4), "[D] %s: Winterbraugerste/Wintergerste grain 2.11 kg N/dt, straw 0.50 kg/dt, ratio 0.8; %s." % (BAS, "grain/(grain + 0.40/0.655)"))
    put(d, "nutrients", "p2o5_grain_share", round(sh(0.80, 0.30), 4), "[D] %s: grain 0.80, straw 0.30 kg P2O5/dt, ratio 0.8." % BAS)
    put(d, "nutrients", "k2o_grain_share", round(sh(0.55, 1.40), 4), "[D] %s: grain 0.55, straw 1.40 kg K2O/dt, ratio 0.8." % BAS)
    put(d, "nutrients", "p2o5_conc_pct_dm", round(cn(0.80, 0.30), 4), "[D] %s: total aboveground P2O5 per t fresh grain over the aboveground dry matter 0.86/0.45 t." % BAS)
    put(d, "nutrients", "k2o_conc_pct_dm", round(cn(0.55, 1.40), 4), "[D] %s: as p2o5_conc_pct_dm for K2O." % BAS)
    put(d, "nutrients", "nc_a", 5.35, "[AS] Justes et al. 1994 wheat curve Nc = 5.35 W^-0.442 used for barley; no barley-specific curve was looked up.")
    put(d, "nutrients", "ph_opt_low", 5.8, "[AS] pH (CaCl2) from which barley is unrestricted; barley is the more lime-demanding cereal (extension advice, recalled).")
    put(d, "management", "seed_mass_g", 0.042, "[D] thousand-kernel weight 42 g, the middle of 38-46 g of German winter barley varieties.")
    return d


# ------------------------------------------------------------------------------------------------ winter rapeseed
def winter_rapeseed():
    d = crop_from_wheat("winter_rapeseed", "brassica",
                        "F1d winter rapeseed (family brassica). AquaCrop has no rapeseed in Annex I; parameters come from the canola "
                        "AquaCrop calibrations named in each entry (values read from search-result snippets, not from the papers) "
                        "and FAO-56. The stage table is [D] from DWD phenology and the Frankfurt station. Nutrient removal is [D] from "
                        "LfL Bayern Bas22. No vernalisation is modelled (autumn sowing only).")
    del d["vernalisation"]
    del d["phenology"]["vernalisation_until"]
    put(d, "thermal", "t_base_c", 0.0, "[AS] base temperature 0 degC; canola AquaCrop calibrations use 0-5 degC, not confirmed.")
    put(d, "thermal", "t_upper_c", 30.0, "[AS] upper temperature 30 degC.")
    d["phenology"]["stages"] = stages([("sown", 0, 0), ("emergence", 10, 216), ("rosette", 14, 558), ("buds", 50, 1755),
                                       ("flowering", 61, 1942), ("senescence", 71, 2450), ("maturity", 87, 3267), ("overripe", 92, 3504)])
    d["phenology"]["source"] = {"stages": (
        "[D] emergence 216, rosette 558, buds 1755, flowering 1942, maturity 3267, overripe 3504 are " + DWD + " (DWD winter rapeseed phases "
        "12 Auflaufen BBCH 10, 14 Rosettenbildung BBCH 14, 17 Knospenbildung BBCH 50, 5 Bluete Beginn BBCH 61, 22 Vollreife BBCH 87, 24 Ernte; "
        "sd 28/52/95/91/111/122 units; mean doys sowing 237.4 (25 Aug), emergence 249.5, rosette 271.9, buds 98.9, flowering 115.9, "
        "full ripe 193.8, harvest 205.6). Senescence 2450 [AS]: start of the canopy decline between the end of flowering and maturity.")}
    put(d, "canopy", "cc0_cm2_per_plant", 3.0, "[AS] canopy per rapeseed seedling at emergence (larger cotyledons than a cereal); not confirmed in a canola calibration.")
    put(d, "canopy", "plant_density_per_ha", 450000, "[AS] 40-50 established plants/m2 in autumn (German practice, 40-60 seeds/m2); 4.5e5/ha.")
    put(d, "canopy", "ccx", 0.95, "[AS] maximum canopy cover.")
    put(d, "canopy", "cgc_per_gdd", 0.008, "[AS] canopy growth coefficient per GDD.")
    put(d, "canopy", "cdc_per_gdd", 0.005, "[AS] canopy decline coefficient per GDD.")
    put(d, "water", "kc_tr_x", 1.05, "[AS] transpiration coefficient at full canopy; FAO-56 Table 12 gives a single Kc,mid of 1.0-1.15 for rapeseed/canola.")
    put(d, "water", "height_max_m", 0.6, "[C] FAO-56 Table 12 (fao.org/4/x0490e/x0490e0b.htm): rapeseed/canola maximum crop height 0.6 m.")
    put(d, "water", "p_depletion", 0.60, "[C] FAO-56 Table 22 (fao.org/4/x0490e/x0490e0e.htm): rapeseed/canola p = 0.60 at ETc ~ 5 mm/day, rooting depth 1.0-1.5 m.")
    put(d, "biomass", "wp_star_g_m2", 17.0, "[AS] normalised water productivity of canola: 15.3 g/m2 (spring canola, Karkag, calibrated) and 18.6 g/m2 (Wagga Wagga, Zeleke et al. 2011, Agron. J. 103:1610-1618) as found in search-result snippets; 17.0 between them. With the light-primary form WP* does not drive the gain, it enters only the CO2 type (f_type 1).")
    put(d, "biomass", "gdd_full", 12.0, "[AS] thermal limit of biomass production.")
    put(d, "biomass", "rue_g_mj", 0.9, "[M] 0.9 g/MJ of global radiation for rapeseed, set so that the 30-year mean on the home loess is about 8 %% above the Destatis 2016/21 national mean (33.6 dt/ha; 1.0 gave +22 %%), the site being favourable like the wheat site (+5 %%); the literature value of rapeseed RUE was not looked up.")
    put(d, "yield", "hi0", 0.30, "[AS] reference harvest index 0.30: a semi-arid canola AquaCrop study reports HI0 = 30 %% (search-result snippet); Bas22 straw ratio 1.7 gives 0.38 for the harvested grain/straw split.")
    put(d, "yield", "hi_build_gdd", 1325, "[D] development units from the flowering stage to maturity of the stage table (3267 - 1942).")
    put(d, "yield", "flowering_length_gdd", 300, "[AS] flowering of rapeseed lasts about 4 weeks (about 300 units at 11 units/day in April-May).")
    put(d, "yield", "cold_t_c", 3.0, "[AS] minimum temperature below which pollination starts to fail.")
    put(d, "yield", "heat_t_c", 29.5, "[AS] maximum temperature above which pollination starts to fail (B. napus is heat-sensitive from about 29.5 degC at flowering, recalled).")
    fr = 0.69
    put(d, "yield", "straw_harvest_fraction", fr, "[D] %s: winter rapeseed straw:grain 1.7 (86 %% / 91 %% dry matter) = 1.46 t straw dm per t grain fresh (0.91 t dm) = 1.6 per t grain dm; 1.6 x 0.30 = 0.48 of the biomass out of the non-grain 0.70: 0.69." % BAS)
    put(d, "roots", "zr_max_m", 1.2, "[C] FAO-56 Table 22: rapeseed/canola maximum rooting depth 1.0-1.5 m (middle).")
    put(d, "roots", "gdd_full", 2600, "[AS] development units from sowing at which the maximum depth is reached (before maturity).")
    put(d, "frost", "kill_onset_c", -15.0, "[AS] daily minimum at which frost starts to kill an emerged rosette (winter rapeseed LT50 about -15 to -20 degC, recalled).")
    put(d, "frost", "kill_full_c", -25.0, "[AS] daily minimum at which the whole stand is killed.")
    put(d, "nutrients", "nc_a", 4.5, "[AS] Colnenne, Meynard, Reau, Justes, Merrien 1998 (Ann. Bot. 81:311-317) critical N curve of winter oilseed rape, Nc = 4.5 W^-0.25 recalled; the abstract found this session confirms the paper but not the coefficients.")
    put(d, "nutrients", "nc_b", 0.25, "[AS] see nc_a.")
    put(d, "nutrients", "nc_w_min_t_ha", 1.0, "[AS] below 1 t/ha the curve is held at its value for 1 t/ha.")
    put(d, "nutrients", "nc_const_pct", 4.5, "[AS] Nc at W = 1 t/ha.")
    sh = lambda g, s: g / (g + s * 1.7 * 0.86 / 0.91 / fr)
    gram = 10.0
    put(d, "nutrients", "n_grain_share", round(sh(3.35, 0.70), 4), "[D] %s: Winterraps grain (91 %% dry matter) 3.35 kg N/dt, straw 0.70 kg/dt, straw ratio 1.7 dt/dt." % BAS)
    put(d, "nutrients", "p2o5_grain_share", round(sh(1.80, 0.40), 4), "[D] %s: grain 1.80, straw 0.40 kg P2O5/dt." % BAS)
    put(d, "nutrients", "k2o_grain_share", round(sh(1.00, 2.35), 4), "[D] %s: grain 1.00, straw 2.35 kg K2O/dt." % BAS)
    bio = 0.91 / 0.30 * 1000.0
    put(d, "nutrients", "p2o5_conc_pct_dm", round(10.0 * (1.80 + 0.40 * 1.7 / fr) / bio * 100.0, 4), "[D] %s: total aboveground P2O5 per t fresh grain over the aboveground dry matter 0.91/0.30 t." % BAS)
    put(d, "nutrients", "k2o_conc_pct_dm", round(10.0 * (1.00 + 2.35 * 1.7 / fr) / bio * 100.0, 4), "[D] %s: as p2o5_conc_pct_dm for K2O." % BAS)
    put(d, "nutrients", "ph_opt_low", 6.0, "[AS] pH (CaCl2) from which rapeseed is unrestricted; rapeseed is lime-demanding (VDLUFA target for heavy soils above 6, recalled).")
    put(d, "management", "seed_mass_g", 0.005, "[D] thousand-seed weight 5 g of winter rapeseed (4.5-6 g).")
    put(d, "management", "depth_opt_min_m", 0.01, "[AS] drilling depth of rapeseed 1-2 cm (extension advice, recalled).")
    put(d, "management", "depth_opt_max_m", 0.02, "[AS] see depth_opt_min_m.")
    put(d, "management", "depth_zero_m", 0.05, "[AS] emergence fails from about 5 cm for the small seed.")
    return d


# ------------------------------------------------------------------------------------------------ grain maize
def grain_maize():
    d = crop_from_wheat("grain_maize", "maize",
                        "F1d grain maize (family maize). Annex I.2 Maize values of the AquaCrop reference manual v7.3 are [C]; the stage "
                        "table is [D] from DWD phenology and the Frankfurt station (silage and grain maize reporters are not separated "
                        "by DWD; the harvest date is their mean); nutrient removal is [D] from LfL Bayern Bas22 (Koernermais). "
                        "Silage maize is not a separate file: its yield is the whole aboveground biomass at the same stage.")
    del d["vernalisation"]
    del d["phenology"]["vernalisation_until"]
    put(d, "thermal", "t_base_c", 8.0, "[C] %s.2 Maize, section 1.1: Tbase 8 degC." % ANN)
    put(d, "thermal", "t_upper_c", 30.0, "[C] %s.2 Maize, section 1.1: Tupper 30 degC." % ANN)
    d["phenology"]["stages"] = stages([("sown", 0, 0), ("emergence", 10, 95), ("stem_elongation", 31, 328), ("tassel", 53, 733),
                                       ("flowering", 61, 840), ("senescence", 75, 1250), ("maturity", 87, 1515), ("overripe", 92, 1800)])
    d["phenology"]["source"] = {"stages": (
        "[D] emergence 95, stem_elongation 328, tassel 733, flowering 840, maturity 1515 are " + DWD + " (DWD maize phases 12 Auflaufen BBCH 10, "
        "67 Laengenwachstum BBCH 31, 65 Fahnenschieben BBCH 53, 5 Bluete Beginn BBCH 61, 24 Ernte; sd 13/31/41/37/67 units; mean doys sowing 116.6, "
        "emergence 131.4, BBCH 31 158.9, tassel 195.0, flowering 203.8, yellow ripe 255.0, harvest 268.2; the harvest mean mixes silage and grain "
        "maize, so the grain maize maturity is at the lower edge of the annex range emergence + 1450-1850). Senescence 1250 [AS]: the annex range "
        "emergence + 1150-1500 starts at 1245. Overripe 1800 [AS]: grain left to dry on the plant.")}
    put(d, "canopy", "cc0_cm2_per_plant", 6.5, "[C] %s.2 Maize, section 1.2: soil surface covered by an individual seedling 6.50 cm2/plant." % ANN)
    put(d, "canopy", "plant_density_per_ha", 90000, "[C] %s.2 Maize, section 1.2: 50 000 - 100 000 plants per hectare (management); 90 000 is German grain maize practice, inside the range." % ANN)
    put(d, "canopy", "ccx", 0.95, "[C] %s.2 Maize, section 1.2: maximum canopy cover 65-99 %%; 0.95 inside the range." % ANN)
    put(d, "canopy", "cgc_per_gdd", 0.0125, "[C] %s.2 Maize, section 1.2: canopy growth coefficient 0.012-0.013 per GDD." % ANN)
    put(d, "canopy", "cdc_per_gdd", 0.010, "[C] %s.2 Maize, section 1.2: canopy decline coefficient 0.010 per GDD." % ANN)
    put(d, "water", "kc_tr_x", 1.05, "[C] %s.2 Maize, section 2: KcTr,x 1.05." % ANN)
    put(d, "water", "height_max_m", 2.0, "[C] FAO-56 Table 12 (fao.org/4/x0490e/x0490e0b.htm): field maize maximum crop height 2 m.")
    put(d, "water", "p_depletion", 0.55, "[C] FAO-56 Table 22 (fao.org/4/x0490e/x0490e0e.htm): field maize p = 0.55 at ETc ~ 5 mm/day, rooting depth 1.0-1.7 m.")
    put(d, "water", "exp_p_upper", 0.14, "[C] %s.2 Maize, section 4.1: canopy expansion 0.14." % ANN)
    put(d, "water", "exp_p_lower", 0.72, "[C] %s.2 Maize, section 4.1: canopy expansion 0.72." % ANN)
    put(d, "water", "exp_shape", 2.9, "[C] %s.2 Maize, section 4.1: shape factor for canopy expansion 2.9." % ANN)
    put(d, "water", "sen_p_upper", 0.69, "[C] %s.2 Maize, section 4.1: canopy senescence psen 0.69." % ANN)
    put(d, "water", "sen_shape", 2.7, "[C] %s.2 Maize, section 4.1: shape factor for canopy senescence 2.7." % ANN)
    put(d, "biomass", "wp_star_g_m2", 33.7, "[C] %s.2 Maize, section 3.1: WP* 33.7 g/m2 (C4: the CO2 table of AquaCrop enters through f_type = (40 - 33.7)/20 = 0.315)." % ANN)
    put(d, "biomass", "gdd_full", 12.0, "[C] %s.2 Maize, section 4.2: minimum growing degrees for full biomass production 12 degC day (estimated)." % ANN)
    put(d, "biomass", "rue_g_mj", 1.6, "[M] 1.6 g/MJ of global radiation for maize. The recalled literature value is 1.9 (Kiniry et al. 1989: about 3.8 g/MJ PAR before grain filling, x 0.5 PAR share; not looked up), which gave +29 %% over the Destatis 2016/21 national grain maize mean (95.3 dt/ha at 14 %% moisture); 1.6 puts the 30-year mean on the home loess about 9 %% above it, the site being favourable like the wheat site (+5 %%).")
    put(d, "biomass", "rue_t_low_c", 8.0, "[AS] the C4 light-use efficiency starts at the base temperature 8 degC ...")
    put(d, "biomass", "rue_t_full_c", 20.0, "[AS] ... and is full from 20 degC.")
    put(d, "yield", "hi0", 0.50, "[C] %s.2 Maize, section 3.2: reference harvest index 48-52 %%; 0.50." % ANN)
    put(d, "yield", "hi_build_gdd", 675, "[D] development units from the flowering stage to maturity of the stage table (1515 - 840).")
    put(d, "yield", "flowering_length_gdd", 175, "[C] %s.2 Maize, section 1.3: length of the flowering stage 150-200 GDD; 175." % ANN)
    put(d, "yield", "excess_fruits_pct", 10, "[AS] the annex lists the excess of potential fruits as small without a number.")
    put(d, "yield", "ppol", 0.80, "[C] %s.2 Maize, section 4.1: pollination fails above a depletion of 0.80 of TAW (estimate)." % ANN)
    put(d, "yield", "sto_coefficient_b", 3.0, "[AS] the annex lists the negative effect of stomatal closure on the harvest index as strong; b = 3 (1 = strongest, 20 = small).")
    put(d, "yield", "cold_t_c", 10.0, "[C] %s.2 Maize, section 4.2: cold stress for pollination below 10 degC (estimate)." % ANN)
    put(d, "yield", "heat_t_c", 40.0, "[C] %s.2 Maize, section 4.2: heat stress for pollination above 40 degC (estimate)." % ANN)
    put(d, "yield", "straw_harvest_fraction", 0.95, "[D] %s: Koernermais straw (stover) ratio 1:1 with the grain; with the harvest index 0.50 all of the non-grain biomass is stover, capped at 0.95 (stubble stays)." % BAS)
    put(d, "roots", "zr_max_m", 1.5, "[C] FAO-56 Table 22: field maize maximum rooting depth 1.0-1.7 m (middle); AquaCrop allows up to 2.80 m (management).")
    put(d, "roots", "gdd_full", 800, "[AS] development units from sowing at which the maximum depth is reached (before flowering).")
    put(d, "frost", "kill_onset_c", -2.0, "[AS] maize is frost-tender: leaves are damaged from about -2 degC.")
    put(d, "frost", "kill_full_c", -5.0, "[AS] daily minimum at which the whole stand is killed.")
    put(d, "frost", "snow_insulation_swe_mm", 10.0, "[AS] unused in practice (the crop is not in the field in snow).")
    put(d, "ripeness", "harvest_window_days", 60, "[AS] days after maturity before the grain starts to shed: maize ears stay on the plant.")
    put(d, "ripeness", "shed_rate_per_day", 0.001, "[AS] fraction of the remaining grain lost per day afterwards (ear drop, lodging).")
    put(d, "nutrients", "nc_a", 3.4, "[C] Plenet and Lemaire 2000 (Plant and Soil 216:65-82) critical N curve Nc = 3.4 W^-0.37, W in t dry matter/ha, as quoted in Sci. Data 9:277 (2022), PMC9174182 (value from the quoting paper, not the original).")
    put(d, "nutrients", "nc_b", 0.37, "[C] see nc_a.")
    put(d, "nutrients", "nc_w_min_t_ha", 1.0, "[AS] below 1 t/ha the curve is held at its value for 1 t/ha (the authors' own filter for the dataset).")
    put(d, "nutrients", "nc_const_pct", 3.4, "[C] Nc at W = 1 t/ha of the curve above.")
    fr = 0.95
    sh = lambda g, s: g / (g + s * 1.0 / fr)
    bio = 0.86 / 0.50 * 1000.0
    put(d, "nutrients", "n_grain_share", round(sh(1.38, 0.90), 4), "[D] %s: Koernermais grain 1.38 kg N/dt, stover 0.90 kg/dt, ratio 1:1." % BAS)
    put(d, "nutrients", "p2o5_grain_share", round(sh(0.80, 0.20), 4), "[D] %s: grain 0.80, stover 0.20 kg P2O5/dt." % BAS)
    put(d, "nutrients", "k2o_grain_share", round(sh(0.50, 2.00), 4), "[D] %s: grain 0.50, stover 2.00 kg K2O/dt." % BAS)
    put(d, "nutrients", "p2o5_conc_pct_dm", round(10.0 * (0.80 + 0.20 / fr) / bio * 100.0, 4), "[D] %s: total aboveground P2O5 per t fresh grain over the aboveground dry matter 0.86/0.50 t." % BAS)
    put(d, "nutrients", "k2o_conc_pct_dm", round(10.0 * (0.50 + 2.00 / fr) / bio * 100.0, 4), "[D] %s: as p2o5_conc_pct_dm for K2O." % BAS)
    put(d, "nutrients", "n_uptake_max_kg_ha_day", 6.0, "[AS] maximum daily N uptake.")
    put(d, "nutrients", "lodging_nmin_start_kg_ha", 400.0, "[AS] maize is much less prone to N-induced lodging than cereals.")
    put(d, "nutrients", "lodging_nmin_full_kg_ha", 900.0, "[AS] see lodging_nmin_start_kg_ha.")
    put(d, "nutrients", "lodging_max_loss", 0.05, "[AS] lodging loss at full risk.")
    put(d, "nutrients", "ph_opt_low", 5.5, "[AS] pH (CaCl2) from which maize is unrestricted.")
    put(d, "management", "seed_mass_g", 0.28, "[D] thousand-kernel weight 280 g of grain maize (250-350 g).")
    put(d, "management", "depth_opt_min_m", 0.04, "[AS] maize is drilled 4-6 cm deep (extension advice, recalled).")
    put(d, "management", "depth_opt_max_m", 0.06, "[AS] see depth_opt_min_m.")
    put(d, "management", "depth_zero_m", 0.12, "[AS] emergence fails from about 12 cm for the large seed.")
    return d


# ------------------------------------------------------------------------------------------------ sugar beet
def sugar_beet():
    d = crop_from_wheat("sugar_beet", "beet",
                        "F1d sugar beet (family beet). Annex I.7 Sugar Beet values of the AquaCrop reference manual v7.3 are [C]; the stage "
                        "table is [D] from DWD phenology (sowing and emergence of the annual reporters, row closure) and the Frankfurt "
                        "station; nutrient removal is [D] from LfL Bayern Bas22. The yield is the root dry matter (AquaCrop: 'sugar beet "
                        "yield is expressed as dry matter'; fresh matter = dry matter / 0.23 for the Bas22 beet at 23 %% dry matter). "
                        "A root crop: harvest index builds from the 'flowering' stage (= start of yield formation), no pollination (yield.pollination 0; the "
                        "pollination keys are required by the loader and unused).")
    del d["vernalisation"]
    del d["phenology"]["vernalisation_until"]
    put(d, "yield", "pollination", 0, "[AS] a root crop has no pollination: 0 switches the pollination factor off (the harvest index builds from the stage named flowering = start of yield formation).")
    put(d, "thermal", "t_base_c", 5.0, "[C] %s.7 Sugar Beet, section 1.1: Tbase 5 degC." % ANN)
    put(d, "thermal", "t_upper_c", 30.0, "[C] %s.7 Sugar Beet, section 1.1: Tupper 30 degC." % ANN)
    d["phenology"]["stages"] = stages([("sown", 0, 0), ("emergence", 10, 131), ("row_closure", 35, 619), ("flowering", 40, 1000),
                                       ("senescence", 70, 1900), ("maturity", 89, 2212), ("overripe", 92, 2500)])
    d["phenology"]["source"] = {"stages": (
        "[D] emergence 131, row_closure 619, maturity 2212 are " + DWD + " (DWD sugar beet phases 10 Bestellung, 12 Auflaufen BBCH 10, 13 Bestand "
        "geschlossen BBCH 35, 24 Ernte; sd 21/38/144 units; mean doys sowing 96.8, emergence 116.7, closure 162.5, harvest 282.5; the harvest is "
        "the beginning of the lifting campaign, so 'maturity' is the mean lifting date). The stage named flowering is the start of yield formation (no flowers on a beet), 1000 [C range]: AquaCrop annex I.7 section 1.3, "
        "start of yield formation at emergence + 800-1000 (here emergence + 869). Senescence 1900 [C range]: annex emergence + 1700-2300. Overripe 2500 [AS].")}
    # the loader's named stages are emergence/flowering/senescence/maturity; the start of yield formation takes the 'flowering' role.
    put(d, "canopy", "cc0_cm2_per_plant", 1.0, "[C] %s.7 Sugar Beet, section 1.2: soil surface covered by an individual seedling 1.00 cm2/plant." % ANN)
    put(d, "canopy", "plant_density_per_ha", 100000, "[C] %s.7 Sugar Beet, section 1.2: 80 000 - 120 000 plants per hectare (management); 100 000." % ANN)
    put(d, "canopy", "ccx", 0.95, "[C] %s.7 Sugar Beet, section 1.2: maximum canopy cover 'almost entirely covered'; 0.95." % ANN)
    put(d, "canopy", "cgc_per_gdd", 0.012, "[C] %s.7 Sugar Beet, section 1.2: canopy growth coefficient 0.010-0.012 per GDD; the upper end, because with 0.011 the canopy reached half cover at about 740 units after emergence while the DWD row closure (BBCH 35) is at 488 units after emergence." % ANN)
    put(d, "canopy", "cdc_per_gdd", 0.0035, "[C] %s.7 Sugar Beet, section 1.2: canopy decline coefficient 0.003-0.004 per GDD." % ANN)
    put(d, "water", "kc_tr_x", 1.10, "[C] %s.7 Sugar Beet, section 2: KcTr,x 1.10." % ANN)
    put(d, "water", "height_max_m", 0.5, "[C] FAO-56 Table 12 (fao.org/4/x0490e/x0490e0b.htm): sugar beet maximum crop height 0.5 m.")
    put(d, "water", "p_depletion", 0.55, "[C] FAO-56 Table 22 (fao.org/4/x0490e/x0490e0e.htm): sugar beet p = 0.55 at ETc ~ 5 mm/day (footnote 3), rooting depth 0.7-1.2 m.")
    put(d, "water", "exp_p_upper", 0.20, "[C] %s.7 Sugar Beet, section 4.1: canopy expansion 0.20." % ANN)
    put(d, "water", "exp_p_lower", 0.60, "[C] %s.7 Sugar Beet, section 4.1: canopy expansion 0.60." % ANN)
    put(d, "water", "exp_shape", 3.0, "[C] %s.7 Sugar Beet, section 4.1: shape factor for canopy expansion 3.0." % ANN)
    put(d, "water", "sen_p_upper", 0.75, "[C] %s.7 Sugar Beet, section 4.1: canopy senescence psen 0.75." % ANN)
    put(d, "water", "sen_shape", 3.0, "[C] %s.7 Sugar Beet, section 4.1: shape factor for canopy senescence 3.0." % ANN)
    put(d, "biomass", "wp_star_g_m2", 17.0, "[C] %s.7 Sugar Beet, section 3.1: WP* 17.0 g/m2." % ANN)
    put(d, "biomass", "gdd_full", 9.0, "[C] %s.7 Sugar Beet, section 4.2: minimum growing degrees for full biomass production 9 degC day (estimated)." % ANN)
    put(d, "biomass", "rue_g_mj", 1.8, "[AS] light-use efficiency of sugar beet, 1.8 g per MJ of global radiation (total dry matter): Scott and Jaggard (about 1.8 g/MJ intercepted solar radiation), recalled, not looked up.")
    put(d, "biomass", "rue_t_low_c", 5.0, "[AS] the light-use efficiency starts at the base temperature 5 degC ...")
    put(d, "biomass", "rue_t_full_c", 15.0, "[AS] ... and is full from 15 degC.")
    put(d, "yield", "hi0", 0.68, "[C] %s.7 Sugar Beet, section 3.2: reference harvest index 65-75 %% (root dry matter over aboveground-plus-root dry matter); 0.68; Bas22 beet:leaf gives 0.646." % ANN)
    put(d, "yield", "hi_build_gdd", 1212, "[D] development units from the yield formation stage to maturity of the stage table (2212 - 1000).")
    put(d, "yield", "straw_harvest_fraction", 0.95, "[D] %s: sugar beet leaves (Blatt) are 0.7 dt per dt beet at 18 %% dry matter against 23 %% for the beet: leaf dm 0.126 t per t beet, non-grain biomass at HI 0.68 is 0.47 per unit root dm = 0.108 t; all of it can be taken, capped at 0.95." % BAS)
    put(d, "roots", "zr_max_m", 1.8, "[AS] FAO-56 Table 22 gives 0.7-1.2 m for sugar beet and the AquaCrop annex I.7 'up to 2.40 m' (management); 1.8 m is the depth of the loess water bucket of the home soil (beet roots follow the water on deep loess).")
    put(d, "roots", "gdd_full", 1400, "[AS] development units from sowing at which the maximum depth is reached.")
    put(d, "frost", "kill_onset_c", -5.0, "[AS] daily minimum at which frost starts to kill an emerged stand; sugar beet seedlings are damaged from about -3 to -5 degC.")
    put(d, "frost", "kill_full_c", -10.0, "[AS] daily minimum at which the whole stand is killed.")
    put(d, "ripeness", "harvest_window_days", 120, "[AS] beets stay in the ground after maturity.")
    put(d, "ripeness", "shed_rate_per_day", 0.0005, "[AS] fraction of the remaining root dry matter lost per day afterwards (rot, respiration).")
    put(d, "nutrients", "nc_a", 4.23, "[C] critical N dilution curve of sugar beet Nc = 4.23 W^-0.49 (Inner Mongolia field experiments, Acta Agron. Sin., zwxb.chinacrops.org abstract 7280, value from a search-result snippet).")
    put(d, "nutrients", "nc_b", 0.49, "[C] see nc_a.")
    put(d, "nutrients", "nc_w_min_t_ha", 1.0, "[AS] below 1 t/ha the curve is held at its value for 1 t/ha.")
    put(d, "nutrients", "nc_const_pct", 4.23, "[C] Nc at W = 1 t/ha of the curve above.")
    fr = 0.95
    ratio = 0.7 * 0.18 / 0.23          # dry matter of the leaves per unit root dry matter... expressed per fresh beet below
    # per t of fresh beet: root dm 0.23 t; leaves 0.7 t fresh (dm 0.126 t)
    def share(g, s):
        return g / (g + s * 0.7 / fr)
    bio = 0.23 / 0.68 * 1000.0
    put(d, "nutrients", "n_grain_share", round(share(0.18, 0.40), 4), "[D] %s: beet 0.18 kg N/dt fresh, leaves 0.40 kg/dt, 0.7 dt leaves per dt beet." % BAS)
    put(d, "nutrients", "p2o5_grain_share", round(share(0.10, 0.11), 4), "[D] %s: beet 0.10, leaves 0.11 kg P2O5/dt." % BAS)
    put(d, "nutrients", "k2o_grain_share", round(share(0.25, 0.71), 4), "[D] %s: beet 0.25, leaves 0.71 kg K2O/dt." % BAS)
    put(d, "nutrients", "p2o5_conc_pct_dm", round(10.0 * (0.10 + 0.11 * 0.7 / fr) / bio * 100.0, 4), "[D] %s: total aboveground P2O5 per t fresh beet over the aboveground dry matter 0.23/0.68 t." % BAS)
    put(d, "nutrients", "k2o_conc_pct_dm", round(10.0 * (0.25 + 0.71 * 0.7 / fr) / bio * 100.0, 4), "[D] %s: as p2o5_conc_pct_dm for K2O." % BAS)
    put(d, "nutrients", "lodging_nmin_start_kg_ha", 1000.0, "[AS] no lodging in a root crop.")
    put(d, "nutrients", "lodging_nmin_full_kg_ha", 2000.0, "[AS] see lodging_nmin_start_kg_ha.")
    put(d, "nutrients", "lodging_max_loss", 0.0, "[AS] none.")
    put(d, "nutrients", "ph_opt_low", 6.0, "[AS] pH (CaCl2) from which sugar beet is unrestricted; beet is lime-demanding (VDLUFA target above 6, recalled).")
    put(d, "management", "seed_mass_g", 0.012, "[D] thousand-pellet weight 12 g of pelleted sugar beet seed (about 10-14 g).")
    put(d, "management", "depth_opt_min_m", 0.02, "[AS] beet is drilled 2-3 cm deep (extension advice, recalled).")
    put(d, "management", "depth_opt_max_m", 0.03, "[AS] see depth_opt_min_m.")
    put(d, "management", "depth_zero_m", 0.06, "[AS] emergence fails from about 6 cm for the small seed.")
    return d


if __name__ == "__main__":
    dump("winter_barley.json", winter_barley())
    dump("winter_rapeseed.json", winter_rapeseed())
    dump("grain_maize.json", grain_maize())
    d = sugar_beet()
    dump("sugar_beet.json", d)


# ------------------------------------------------------------------------------------------------ 30-year single-crop scenarios
SCN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "field_scenarios")

# crop -> (sow doy, sow in the harvest year?, harvest doy, [(doy, kg N)], extra operations)
PLANS = {
    "winter_barley": dict(dap=0.15, mop=0.17, sow=275, winter=True, harvest=196, n=[(60, 50), (95, 60), (125, 40)]),
    "winter_rapeseed": dict(dap=0.16, mop=0.17, sow=235, winter=True, harvest=208, n=[(50, 80), (80, 60), (110, 40)]),
    "grain_maize": dict(dap=0.19, mop=0.17, sow=118, winter=False, harvest=290, n=[(100, 90), (160, 60)]),
    "sugar_beet": dict(dap=0.20, mop=0.35, sow=98, winter=False, harvest=285, n=[(90, 90), (140, 30)]),
}


def scenario(name, years=30):
    p = PLANS[name]
    ops = [
        {"op": "apply", "material": "lime_carbonate_90", "method": "incorporated", "rate_t_ha": 4.0, "doy": 250, "every_years": 4},
        {"op": "apply", "material": "dap_18_46", "method": "broadcast", "rate_t_ha": p["dap"], "doy": 262},
        {"op": "apply", "material": "mop_60", "method": "broadcast", "rate_t_ha": p["mop"], "doy": 262},
        {"op": "sow", "crop": name, "doy": p["sow"]},
    ]
    fy = 1 if p["winter"] else 0
    for doy, kg in p["n"]:
        ops.append({"op": "apply", "material": "can_27", "method": "broadcast", "rate_n_kg_ha": kg, "doy": doy, "first_year": fy})
    ops.append({"op": "harvest", "doy": p["harvest"], "first_year": fy, "straw_removed": 0.0})
    return {
        "format": "physics_sim.field_scenario/1",
        "climate": "../climates/farm_de_home.json",
        "seed": 21,
        "years": years,
        "nutrients": True,
        "soils": ["../soils/silt_loam_loess.json"],
        "crops": ["../crops/%s.json" % name],
        "materials": ["../field_materials/can_27.json", "../field_materials/dap_18_46.json", "../field_materials/mop_60.json",
                      "../field_materials/lime_carbonate_90.json"],
        "methods": "../field_materials/methods.json",
        "grid": {"nx": 2, "ny": 2, "cell_size_m": 4.0},
        "initial_depletion_fraction": 0.2,
        "co2_ppm": 400.0,
        "operations": ops,
        "source": ("F1d single-crop validation run: %s %d years on the deep loess of the home climate (seed 21) with a good-practice "
                   "fertiliser plan (CAN %d kg N/ha in %d dressings, DAP %.2f t/ha and MOP %.2f t/ha each autumn, lime 4 t every fourth "
                   "year), straw/stover left on the field. Test-only operations, not an agronomic recommendation.")
                  % (name, years, sum(k for _, k in p["n"]), len(p["n"]), p["dap"], p["mop"]),
    }


if __name__ == "__main__":
    os.makedirs(SCN, exist_ok=True)
    for nme in PLANS:
        with open(os.path.join(SCN, "crop_%s_30y.json" % nme), "w", encoding="utf-8", newline="\n") as f:
            json.dump(scenario(nme), f, indent=2, ensure_ascii=False)
            f.write("\n")
