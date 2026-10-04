#!/usr/bin/env python3
"""NIST N2O property data generator (N2O plan N0, dev tool, stdlib only).

Reads the tab-separated NIST Chemistry WebBook fluid-property downloads
(SRD 69, Lemmon & Span 2006 equation of state, reference state DEF) and writes
data/oxidizers/nitrous_oxide.json (physics_sim.oxidizer/1) deterministically:
the same raw files give the same bytes on every run (rows sorted by
temperature, numbers re-printed with repr(float), keys in a fixed order,
a SHA-256 of every raw input recorded in "source").

Raw files (saved under out/sources/, which is git-ignored; download with
--fetch, one URL each, all are US-government public data):
  n2o_satp_1K.tsv           saturation table 183..309 K in 1 K steps
  n2o_satp_pt_<T>.tsv       single saturation rows at 293.13, 303.37, 305.41,
                            307.36 and 309.5 K (the temperatures the N2O plan's
                            hand numbers use, and the near-critical end)
  n2o_isobar_1bar.tsv       the 1 bar isobar 200..400 K in 10 K steps (gas
                            enthalpy; vapour rows only are used)
  n2o_cbook.html            the WebBook thermochemistry page (dHf, molar mass)

Usage:
  python tools/oxidizerdata/nist_n2o.py [--raw out/sources] [--out data/oxidizers/nitrous_oxide.json] [--fetch]
"""

import argparse
import hashlib
import json
import os
import sys
import urllib.request

FLUID_BASE = ("https://webbook.nist.gov/cgi/fluid.cgi?Action=Data&Wide=on&ID=C10024972&Digits=5&RefState=DEF"
              "&TUnit=K&PUnit=bar&DUnit=kg%2Fm3&HUnit=kJ%2Fkg&WUnit=m%2Fs&VisUnit=uPa*s&STUnit=N%2Fm")
CBOOK_URL = "https://webbook.nist.gov/cgi/cbook.cgi?ID=C10024972&Units=SI&Mask=1"
EXTRA_T = ["293.13", "303.37", "305.41", "307.36", "309.5"]

RAW_FILES = {
    "n2o_satp_1K.tsv": FLUID_BASE + "&Type=SatP&THigh=309&TLow=183&TInc=1",
    "n2o_isobar_1bar.tsv": FLUID_BASE + "&Type=IsoBar&P=1&THigh=400&TLow=200&TInc=10",
    "n2o_cbook.html": CBOOK_URL,
}
for _t in EXTRA_T:
    RAW_FILES["n2o_satp_pt_%s.tsv" % _t] = FLUID_BASE + "&Type=SatP&THigh=%s&TLow=%s&TInc=1" % (_t, _t)

# Column names in the NIST wide table (liquid then vapour block).
COL_T = "Temperature (K)"
COL_P = "Pressure (bar)"
COL_RHO_L = "Density (l, kg/m3)"
COL_H_L = "Enthalpy (l, kJ/kg)"
COL_S_L = "Entropy (l, J/g*K)"
COL_RHO_V = "Density (v, kg/m3)"
COL_H_V = "Enthalpy (v, kJ/kg)"
COL_S_V = "Entropy (v, J/g*K)"

MOLAR_MASS_G_MOL = 44.0128   # NIST WebBook (cbook page)
DHF_KJ_MOL = 82.05           # Chase 1998, NIST WebBook (cbook page)
M_O2 = 31.998                # g/mol, only for the derived oxygen mass fraction
OXYGEN_MASS_FRACTION = 0.3635  # DERIVED: 0.5 * 31.998 / 44.0128 = 0.36351, rounded to 4 digits


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def read_table(path):
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.rstrip("\r\n") for ln in f if ln.strip()]
    header = lines[0].split("\t")
    rows = []
    for ln in lines[1:]:
        cells = ln.split("\t")
        rows.append({h: c for h, c in zip(header, cells)})
    return header, rows


def num(row, col, path):
    try:
        v = float(row[col])
    except (KeyError, ValueError):
        raise SystemExit("%s: column %r missing or not a number in row %r" % (path, col, row.get(COL_T)))
    return v


def fetch_missing(raw_dir):
    os.makedirs(raw_dir, exist_ok=True)
    for name, url in RAW_FILES.items():
        path = os.path.join(raw_dir, name)
        if os.path.exists(path):
            continue
        sys.stderr.write("fetching %s (%s)\n" % (name, url))
        with urllib.request.urlopen(url, timeout=60) as resp:
            data = resp.read()
        with open(path, "wb") as f:
            f.write(data)
        sys.stderr.write("  %d bytes\n" % len(data))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default="out/sources")
    ap.add_argument("--out", default="data/oxidizers/nitrous_oxide.json")
    ap.add_argument("--fetch", action="store_true", help="download any missing raw file first")
    args = ap.parse_args()

    if args.fetch:
        fetch_missing(args.raw)

    sat = {}  # temperature (float) -> dict
    sat_files = ["n2o_satp_1K.tsv"] + ["n2o_satp_pt_%s.tsv" % t for t in EXTRA_T]
    for name in sat_files:
        path = os.path.join(args.raw, name)
        _, rows = read_table(path)
        for r in rows:
            t = num(r, COL_T, path)
            rec = {
                "p_bar": num(r, COL_P, path),
                "rho_liquid": num(r, COL_RHO_L, path),
                "rho_vapour": num(r, COL_RHO_V, path),
                "h_liquid_kj_kg": num(r, COL_H_L, path),
                "h_vapour_kj_kg": num(r, COL_H_V, path),
                "s_liquid_j_gk": num(r, COL_S_L, path),
                "s_vapour_j_gk": num(r, COL_S_V, path),
            }
            if t in sat and sat[t] != rec:
                raise SystemExit("%s: conflicting saturation row at %r K" % (path, t))
            sat[t] = rec
    temps = sorted(sat)

    gas_path = os.path.join(args.raw, "n2o_isobar_1bar.tsv")
    gas_header, gas_rows = read_table(gas_path)
    # the isobar table has a single (non-split) property block and a Phase column
    phase_col = [h for h in gas_header if h.lower().startswith("phase")]
    col_h = "Enthalpy (kJ/kg)"
    gas_t, gas_h = [], []
    for r in gas_rows:
        if phase_col and "vapor" not in r[phase_col[0]].lower():
            continue
        gas_t.append(num(r, COL_T, gas_path))
        gas_h.append(num(r, col_h, gas_path))

    raw_hashes = {name: sha256(os.path.join(args.raw, name)) for name in sorted(RAW_FILES)}

    source = (
        "NIST Chemistry WebBook, NIST Standard Reference Database 69, fluid properties of nitrous oxide "
        "(CAS 10024-97-2; Lemmon and Span 2006 equation of state, reference state DEF, as tabulated by the WebBook): "
        "saturation table 183-309 K in 1 K steps plus single rows at 293.13, 303.37, 305.41, 307.36 and 309.5 K; "
        "gas enthalpy on the 1 bar isobar, 200-400 K in 10 K steps (vapour rows only). "
        "Formation enthalpy of the gas 82.05 kJ/mol (Chase 1998) and molar mass 44.0128 g/mol: WebBook thermochemistry page "
        + CBOOK_URL + ". Query URLs: " + "; ".join("%s = %s" % (n, RAW_FILES[n]) for n in sorted(RAW_FILES)) +
        ". Fetched 2026-10-04. Raw file SHA-256: " + ", ".join("%s %s" % (n, raw_hashes[n]) for n in sorted(raw_hashes)) +
        ". oxygen_mass_fraction 0.3635 is DERIVED: 0.5 * 31.998 / 44.0128 (2 N2O -> 2 N2 + O2). "
        "Generated by tools/oxidizerdata/nist_n2o.py (deterministic, same bytes on re-run)."
    )

    doc = {
        "format": "physics_sim.oxidizer/1",
        "name": "nitrous oxide",
        "molar_mass_g_mol": MOLAR_MASS_G_MOL,
        "formation_enthalpy_kj_mol": DHF_KJ_MOL,
        "oxygen_mass_fraction": OXYGEN_MASS_FRACTION,
        "saturation": {
            "t_k": temps,
            "p_bar": [sat[t]["p_bar"] for t in temps],
            "rho_liquid": [sat[t]["rho_liquid"] for t in temps],
            "rho_vapour": [sat[t]["rho_vapour"] for t in temps],
            "h_liquid_kj_kg": [sat[t]["h_liquid_kj_kg"] for t in temps],
            "h_vapour_kj_kg": [sat[t]["h_vapour_kj_kg"] for t in temps],
            "s_liquid_j_gk": [sat[t]["s_liquid_j_gk"] for t in temps],
            "s_vapour_j_gk": [sat[t]["s_vapour_j_gk"] for t in temps],
        },
        "gas_1bar": {"t_k": gas_t, "h_kj_kg": gas_h},
        "source": source,
    }
    # one row per line for the long arrays keeps diffs readable and the bytes stable
    text = json.dumps(doc, indent=1, ensure_ascii=True) + "\n"
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    sys.stderr.write("wrote %s (%d saturation rows, %d gas rows)\n" % (args.out, len(temps), len(gas_t)))


if __name__ == "__main__":
    main()
