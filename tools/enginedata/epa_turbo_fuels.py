#!/usr/bin/env python3
"""Knock calibration phase A: the EPA NVFEL test fuels of the turbo benchmark packages as physics_sim.fuel/1 files.

Writes data/fuels/epa_*.json (deterministic, stdlib only, python -I). Every number is a literal reading of the
NVFEL Fuel Analysis Report named in the file, transcribed from the report PDFs' OCR text layer and cross-checked
against Table 4 of each package's Test Report (docx). Derivations (all [D], stated in each file's source):

  lhv_j_kg      "Net Heating Value" in BTU/lb x 2326 (exact: BTU(IT)/lb -> J/kg), the same conversion as
                data/fuels/tier3_e10_cert.json. Where the report prints two labs' values the independent lab's (Paragon) is used.
  density       "Density @ 60 deg F D4052" g/cm3 x 1000.
  afr_stoich    Heywood 1988 section 3.2 stoichiometry from the elemental mass fractions, the same method as
                tier3_e10_cert.json: n_O2 = n_C + n_H/4 - n_O/2 per gram, air = 138.24568 g per mol O2 (N2 + trace gases 28.16 g/mol,
                3.773 mol per mol O2). E0 fuels: only the carbon weight fraction is printed (D3343), hydrogen = 1 - C, no oxygen.
                E10 fuels: C and H are printed (D5291), oxygen = 100 - C - H (checked against the printed D5599 oxygen).
  knock         RON and MON as printed (ASTM D2699 / D2700).

Fuels NOT written here: Mazda 2.5 L Turbo Tier 3 uses FTAG 26781 = data/fuels/tier3_e10_cert.json (RON 92.0 / MON 84.0).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

WT = Path(__file__).resolve().parents[2]
OUT = WT / "data" / "fuels"

M_C, M_H, M_O = 12.011, 1.008, 15.999
AIR_PER_MOL_O2 = 31.998 + 3.773 * 28.16


def afr_stoich(c: float, h: float, o: float) -> float:
    return (c / M_C + h / M_H / 4.0 - o / M_O / 2.0) * AIR_PER_MOL_O2


# key: file name, report, composition, properties
FUELS = [
    dict(name="epa_tier2_25278", report="25278", fuel_label="EPA Tier 2 certification gasoline (E0), FTAG 25278",
         packages="2013 Ford 1.6 L EcoBoost Tier 2 and 2015 Ford 2.7 L EcoBoost V6 Tier 2 Cell 9",
         c=0.86640, c_note="Weight Fraction Carbon D3343 0.86640", h=None, o=0.0,
         btu_lb=18447.0, nhv_note="Net Heating Value 18447 BTU/lb (D240 in the PDF, D3338 in the Test Reports' Table 4)",
         density_g_cm3=0.74245, ron=96.6, mon=88.9, ethanol_vol=0.0,
         extra="Specific gravity 0.74319 (60/60 F), aromatics 30.6 vol% (D1319), olefins 0.6 vol%; sensitivity 7.7, (R+M)/2 92.75."),
    dict(name="epa_tier2_25842", report="25842", fuel_label="EPA Tier 2 certification gasoline (E0), FTAG 25842",
         packages="2016 Honda 1.5 L L15B7 Tier 2",
         c=0.86650, c_note="Weight Fraction Carbon D3343 0.86650", h=None, o=0.0,
         btu_lb=18443.0, nhv_note="Net Heating Value D240 18443 BTU/lb",
         density_g_cm3=0.74227, ron=96.1, mon=88.5, ethanol_vol=0.0,
         extra="Specific gravity 0.74301, aromatics 30.5 vol% (D1319), olefins 0.8 vol%; sensitivity 7.6, (R+M)/2 92.30."),
    dict(name="epa_tier2_26864", report="26864", fuel_label="EPA Tier 2 premium certification gasoline (E0), FTAG 26864",
         packages="2015 Ford 2.7 L EcoBoost V6 Tier 2 HD1 and 2016 Mazda 2.5 L Turbo SKYACTIV-G Tier 2",
         c=0.86633, c_note="Weight Fraction Carbon D3343 0.86633 (the Ford HD1 Test Report prints 0.86640, copied from the previous report)",
         h=None, o=0.0,
         btu_lb=18447.0, nhv_note="Net Heat of Combustion D3338 18447 BTU/lb",
         density_g_cm3=0.74246, ron=97.6, mon=89.0, ethanol_vol=0.0,
         extra="Specific gravity 0.74320, aromatics 30.2 vol% (D1319), olefins 0.6 vol%; sensitivity 8.6, (R+M)/2 93.30."),
    dict(name="epa_levIII_24670", report="24670", fuel_label="California LEV III certification gasoline (E10), FTAG 24670",
         packages="2013 Ford 1.6 L EcoBoost LEV III and 2015 Ford 2.7 L EcoBoost V6 LEV III Cell 9",
         c=0.8265, c_note="Carbon Content D5291 82.65 wt% (both labs)", h=0.1374,
         h_note="Hydrogen Content D5291 13.74 wt% (Paragon; Haltermann 13.66)", o=None,
         btu_lb=17963.0, nhv_note="Net Heating Value D240 17963 BTU/lb (Paragon; Haltermann 17999)",
         density_g_cm3=0.74818, ron=92.4, mon=84.3, ethanol_vol=9.34,
         extra="Independent lab (Paragon, 4/6/2015): RON 92.4, MON 84.3, (R+M)/2 88.35, sensitivity 8.1; the supplier's (Haltermann, 3/4/2015) values "
               "are RON 92.2 / MON 84.6 / 88.50 / 7.8. Specific gravity 0.74892; printed D5599 oxygen 3.44 wt% vs 3.61 from closure."),
    dict(name="epa_tier3_25206", report="25206", fuel_label="EPA Tier 3 certification gasoline (E10), FTAG 25206",
         packages="2016 Honda 1.5 L L15B7 Tier 3",
         c=0.8267, c_note="Carbon Content D5291 82.67 wt%", h=0.1366, h_note="Hydrogen Content D5291 13.66 wt%", o=None,
         btu_lb=17978.0, nhv_note="Net Heating Value D240 17978 BTU/lb",
         density_g_cm3=0.74776, ron=91.0, mon=83.5, ethanol_vol=9.91,
         extra="Ethanol 9.96 / 9.86 vol% (D4815 or D5599, mean 9.91, from the Test Report's Table 4; pages 1-2 of the PDF have no text layer); "
               "density from Table 4; sensitivity 7.5, (R+M)/2 87.25."),
    dict(name="epa_tier3_25402", report="25402", fuel_label="EPA Tier 3 premium certification gasoline (E10), FTAG 25402",
         packages="2015 Ford 2.7 L EcoBoost V6 Tier 3 HD",
         c=0.8267, c_note="Carbon Content D5291 82.67 wt%", h=0.1370, h_note="Hydrogen Content D5291 13.70 wt%", o=None,
         btu_lb=17967.0, nhv_note="Net Heating Value D240 17967 BTU/lb",
         density_g_cm3=0.74506, ron=97.8, mon=88.4, ethanol_vol=9.785,
         extra="Ethanol 9.80 / 9.77 vol% (D5599, mean 9.785); specific gravity 0.74580; printed oxygen 3.62-3.63 wt% vs 3.63 from closure; "
               "sensitivity 9.4, (R+M)/2 93.10."),
]


def build(f: dict) -> dict:
    c = f["c"]
    h = f["h"] if f["h"] is not None else 1.0 - c
    o = f["o"] if f["o"] is not None else 1.0 - c - h
    afr = afr_stoich(c, h, o)
    lhv = f["btu_lb"] * 2326.0
    rho = f["density_g_cm3"] * 1000.0
    comp = (f"C {c:.5f}, H {h:.5f}, O {o:.5f} mass fractions ({f['c_note']}; "
            + (f"{f['h_note']}; O = 1 - C - H" if f["h"] is not None else "H = 1 - C, no oxygenate")
            + ")")
    src = (f"EPA National Vehicle and Fuel Emissions Laboratory (NVFEL) Fuel Analysis Report {f['report']} and the Test Report (Table 4) of the "
           f"public benchmark packages {f['packages']} (US EPA, National Center for Advanced Technology). {f['fuel_label']}. "
           f"MEASURED: {f['nhv_note']}; Density @ 60 deg F D4052 {f['density_g_cm3']} g/cm3; ethanol {f['ethanol_vol']} vol%; {f['extra']} "
           f"DERIVED [D]: lhv_j_kg = {f['btu_lb']:.0f} BTU/lb x 2326 (exact BTU(IT)/lb -> J/kg) = {lhv:.0f} J/kg; density_kg_m3 = "
           f"{f['density_g_cm3']} g/cm3 x 1000; afr_stoich from the elemental composition by Heywood 1988 section 3.2 (n_O2 = n_C + n_H/4 - n_O/2 per gram, "
           f"{AIR_PER_MOL_O2:.5f} g air per mol O2), the method of data/fuels/tier3_e10_cert.json: {comp} -> {afr:.6f}. "
           "Generated by tools/enginedata/epa_turbo_fuels.py.")
    ksrc = (f"[C] EPA NVFEL Fuel Analysis Report {f['report']}: Research Octane Number (ASTM D2699) {f['ron']}, Motor Octane Number "
            f"(ASTM D2700) {f['mon']}, sensitivity {f['ron'] - f['mon']:.1f}, (R+M)/2 {(f['ron'] + f['mon']) / 2:.2f}.")
    return {
        "format": "physics_sim.fuel/1",
        "name": f["name"],
        "phase": "liquid",
        "lhv_j_kg": lhv,
        "afr_stoich": afr,
        "density_kg_m3": round(rho, 2),
        "source": src,
        "knock": {"model": "douaud_eyzat", "ron": f["ron"], "mon": f["mon"], "source": ksrc},
    }


def main() -> int:
    for f in FUELS:
        d = build(f)
        (OUT / f"{f['name']}.json").write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
        print(f"{f['name']}: lhv {d['lhv_j_kg']:.0f} afr {d['afr_stoich']:.4f} rho {d['density_kg_m3']} RON {f['ron']} MON {f['mon']} E{f['ethanol_vol']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
