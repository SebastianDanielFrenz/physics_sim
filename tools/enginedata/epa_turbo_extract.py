#!/usr/bin/env python3
"""Knock calibration phase A: EPA NVFEL turbo-engine benchmark rows -> JSON.

Reads the public EPA NVFEL "Test Data" workbooks (stdlib only: zipfile + xml, python -I) of the turbocharged engines

    Honda 1.5 L L15B7        Tier 2 (E0) and Tier 3 (E10) fuel
    Ford 2.7 L EcoBoost V6   Tier 2 cell 9 (E0, RON 96.6), Tier 2 HD1 (E0, RON 97.6), LEV III cell 9 (E10),
                             Tier 3 HD (E10 premium, RON 97.8)
    Ford 1.6 L EcoBoost      Tier 2 (E0), LEV III (E10)
    Mazda 2.5 L Turbo SKYACTIV-G   Tier 2 (E0), Tier 3 (E10)

and writes, per package,

    out/epa_turbo/<package>_rows.json                          every row of the three data sheets (all columns)
    tests/support/fixtures/epa_turbo/<package>_rows.json       the compact copy for tests: the selected high-load rows
    tests/support/fixtures/epa_turbo/<package>_light_rows.json the compact light-load copy (phase B's no-knock check)

Usage:   python -I tools/enginedata/epa_turbo_extract.py [--src DIR] [--stats]

The source packages are not in the repository (387 MB); --src defaults to S:/claude_code/physics_sim/out/epa_turbo_dl
(EPA URLs in urls.txt next to the zips). Deterministic: same inputs, same bytes.

Sheets read: "Steady State", "High Load Initial", "High Load Final" ("Steady State with Transmission" is a CVT/
transmission low-load series and is not read). Row numbers in the output are the 1-based workbook row numbers.

HIGH-LOAD SELECTION (the compact copy). A sweep row is selected when
    bmep_bar >= 0.5 * (the package's maximum bmep_bar)                                   [flag high_load]
and every selected row at the extremum of its speed bin carries
    envelope = true   <=>  it is the row with the highest torque among ALL rows of its 50-rpm speed bin
                           (bin = round(rpm / 50) * 50), over all three sheets.
So `envelope` is the WOT / maximum-load row per speed (what a WOT dyno curve would contain), and the other selected
rows are the near-WOT and part-boost rows below it. Nothing else is filtered: ECU protection rows (lambda < 0.9, spark
collapsed) stay in and are visible through their lambda / spark / turbine-inlet temperature. Phase B decides what is
"knock limited"; this extraction only removes the part-load (naturally aspirated) rows.

LIGHT-LOAD SELECTION (phase B, the <package>_light_rows.json copy): fired rows with
    map_kpa <= 60, 0.95 <= lambda <= 1.05, bmep_bar >= 0.3
- throttled, stoichiometric, positive work. The engine ran at the measured spark there without knock, so the true
knock-limited spark is at least the measured spark: a one-sided check, nothing is fitted to these rows' spark.

LIGHT-LOAD SELECTION (phase B, the <package>_light_rows.json copy): fired rows with
    map_kpa <= 60, 0.95 <= lambda <= 1.05, bmep_bar >= 0.3
- throttled, stoichiometric, positive work. The engine ran at the measured spark there without knock, so the true
knock-limited spark is at least the measured spark: a one-sided check, nothing is fitted to these rows' spark.

Units, all explicit in the keys: rpm; Nm; bar; degC; kPa absolute (kPaA in the workbooks); deg crank (CAD);
g/s; spark_btdc_deg is the "Average Spark Timing Across all Cylinders" (CAD BTDC, negative = after TDC).
Cam phases are normalised: intake_cam_adv_deg (positive = intake cam advanced, i.e. IVC earlier) and
exhaust_cam_ret_deg (positive = exhaust cam retarded). The raw value and the workbook's own sign convention are
kept (icam_raw, ecam_raw, cam_convention). Honda: intake "CAD ADV", exhaust "CAD RET". Ford 2.7 and Mazda: both "CAD RET".
Ford 1.6: both "CAD" with the description "Phaser Retard" (retard assumed).
There is NO air-flow channel in any of these workbooks (the A25A's did not have one either).
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

WT = Path(__file__).resolve().parents[2]
DEFAULT_SRC = Path(r"S:\claude_code\physics_sim\out\epa_turbo_dl")
OUT_DIR = WT / "out" / "epa_turbo"
FIXTURE_DIR = WT / "tests" / "support" / "fixtures" / "epa_turbo"

DATA_SHEETS = ["Steady State", "High Load Initial", "High Load Final"]
BMEP_FRACTION = 0.5
LIGHT_MAP_KPA = 60.0
LIGHT_LAMBDA = (0.95, 1.05)
LIGHT_BMEP_BAR = 0.3
SPEED_BIN_RPM = 50

# package key -> (directory fragment, engine, fuel file/name, fuel report, cam convention)
PACKAGES = {
    "honda_l15b7_t2": dict(frag="2016-honda-1-5l-l15b7-engine-tier-2", engine="honda_l15b7", fuel="epa_tier2_25842",
                           report="25842", icam="adv", ecam="ret"),
    "honda_l15b7_t3": dict(frag="2016-honda-1-5l-l15b7-engine-tier-3", engine="honda_l15b7", fuel="epa_tier3_25206",
                           report="25206", icam="adv", ecam="ret"),
    "ford_27_t2_cell9": dict(frag="2015-ford-2.7l-ecoboost-v6-engine-tier2-fuel-cell9", engine="ford_27_ecoboost",
                             fuel="epa_tier2_25278", report="25278", icam="ret", ecam="ret"),
    "ford_27_t2_hd1": dict(frag="2015-ford-2.7l-ecoboost-v6-engine-tier2-fuel-hd1", engine="ford_27_ecoboost",
                           fuel="epa_tier2_26864", report="26864", icam="ret", ecam="ret"),
    "ford_27_levIII_cell9": dict(frag="2015-ford-2.7l-ecoboost-v6-engine-leviii-fuel-cell-9", engine="ford_27_ecoboost",
                                 fuel="epa_levIII_24670", report="24670", icam="ret", ecam="ret"),
    "ford_27_t3_hd": dict(frag="2015-ford-2.7l-ecoboost-v6-engine-tier3-fuel-hd", engine="ford_27_ecoboost",
                          fuel="epa_tier3_25402", report="25402", icam="ret", ecam="ret"),
    "ford_16_t2": dict(frag="2013-ford-1.6l-ecoboost-engine-tier-2", engine="ford_16_ecoboost",
                       fuel="epa_tier2_25278", report="25278", icam="ret", ecam="ret"),
    "ford_16_levIII": dict(frag="2013-ford-1.6l-ecoboost-engine-lev-lev-3", engine="ford_16_ecoboost",
                           fuel="epa_levIII_24670", report="24670", icam="ret", ecam="ret"),
    "mazda_25t_t2": dict(frag="2016-mazda-2-5-l-turbo-skyactiv-g-engine-tier-2", engine="mazda_25t_skyactiv",
                         fuel="epa_tier2_26864", report="26864", icam="ret", ecam="ret"),
    "mazda_25t_t3": dict(frag="2016-mazda-2-5-l-turbo-skyactiv-g-engine-tier-3", engine="mazda_25t_skyactiv",
                         fuel="tier3_e10_cert", report="26781", icam="ret", ecam="ret"),
}

# workbook header (lower case) -> output key
HEADER_MAP = {
    "speed": "rpm",
    "torque": "torque_nm",
    "bmep": "bmep_bar",
    "cell temp": "cell_temp_c",
    "cell press": "cell_press_kpa",
    "bsfc": "bsfc_g_kwh",
    "bte": "bte_pct",
    "fuel meter flow": "fuel_meter_g_s",
    "injector fuel flow": "inj_fuel_g_s",
    "fuel rail press": "fuel_rail_mpag",
    "spark timing": "spark_btdc_deg",
    "coolant temp": "coolant_c",
    "oil sump temp": "oil_c",
    "cac inlet press": "cac_in_kpa",
    "cac inlet temp": "cac_in_c",
    "cac outlet press": "cac_out_kpa",
    "cac outlet temp": "cac_out_c",
    "intake manifold press": "map_kpa",
    "intake manifold temp": "imt_c",
    "exhaust port temp": "exh_port_c",
    "turbine inlet press": "turb_in_kpa",
    "turbine inlet temp": "turb_in_c",
    "catalyst inlet press": "cat_in_kpa",
    "catalyst inlet temp": "cat_in_c",
    "exhaust lambda": "lambda",
    "exhaust lambda left": "lambda_left",
    "exhaust lambda right": "lambda_right",
    "intake cam phase": "icam_raw",
    "exhaust cam phase": "ecam_raw",
    "valve overlap": "valve_overlap_deg",
    "compression ratio": "cr_effective",
    "compression stroke length": "compression_stroke_mm",
    "expansion ratio": "expansion_ratio_effective",
    "expansion stroke length": "expansion_stroke_mm",
    "epa atkinson ratio": "atkinson_ratio",
}

_NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
_RNS = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"


def _col(letters: str) -> int:
    n = 0
    for ch in letters:
        n = n * 26 + (ord(ch) - ord("A") + 1)
    return n - 1


def load_xlsx(path: Path) -> dict:
    """{sheet: {row (1-based): {col (0-based): value}}} - zipfile + xml.etree only."""
    with zipfile.ZipFile(path) as z:
        shared = []
        if "xl/sharedStrings.xml" in z.namelist():
            for si in ET.fromstring(z.read("xl/sharedStrings.xml")).findall("m:si", _NS):
                shared.append("".join(t.text or "" for t in si.iter("{%s}t" % _NS["m"])))
        wb = ET.fromstring(z.read("xl/workbook.xml"))
        rels = {r.get("Id"): r.get("Target") for r in ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))}
        out = {}
        for s in wb.find("m:sheets", _NS):
            target = rels[s.get(_RNS)]
            target = target if target.startswith("xl/") else "xl/" + target.lstrip("/")
            rows = {}
            for row_el in ET.fromstring(z.read(target)).iter("{%s}row" % _NS["m"]):
                vals = {}
                for c in row_el.findall("m:c", _NS):
                    ci = _col(re.match(r"[A-Z]+", c.get("r")).group())
                    t = c.get("t")
                    v = c.find("m:v", _NS)
                    if t == "s" and v is not None:
                        val = shared[int(v.text)]
                    elif t == "inlineStr":
                        val = "".join(x.text or "" for x in c.iter("{%s}t" % _NS["m"]))
                    elif v is not None:
                        try:
                            val = float(v.text)
                        except ValueError:
                            val = v.text
                    else:
                        val = None
                    vals[ci] = val
                rows[int(row_el.get("r"))] = vals
            out[s.get("name")] = rows
    return out


def find_workbook(src: Path, frag: str) -> Path:
    hits = sorted(src.glob(f"x_{frag}*/4-*.xlsx"))
    if len(hits) != 1:
        raise SystemExit(f"{frag}: expected exactly one test-data workbook, found {len(hits)}")
    return hits[0]


def r6(x):
    """6 significant digits, deterministic."""
    if x is None or not isinstance(x, float):
        return x
    if x == 0.0 or not math.isfinite(x):
        return x
    return float(f"{x:.6g}")


def extract(key: str, src: Path) -> dict:
    spec = PACKAGES[key]
    wb_path = find_workbook(src, spec["frag"])
    sheets = load_xlsx(wb_path)
    rows = []
    layout = None
    for sheet in DATA_SHEETS:
        if sheet not in sheets:
            continue
        s = sheets[sheet]
        names = s[4]
        units = s[5]
        cols = {}
        for ci, nm in names.items():
            if isinstance(nm, str) and nm.strip().lower() in HEADER_MAP:
                cols[ci] = HEADER_MAP[nm.strip().lower()]
        if layout is None:
            layout = {HEADER_MAP[names[ci].strip().lower()]: f"{names[ci].strip()} [{units.get(ci)}]" for ci in cols}
        for rn in sorted(s):
            if rn < 6:
                continue
            raw = s[rn]
            if not isinstance(raw.get(0), float):
                continue
            r = {"sheet": sheet, "row": rn}
            for ci, k in cols.items():
                v = raw.get(ci)
                if isinstance(v, float):
                    r[k] = r6(v)
            if "lambda" not in r and "lambda_left" in r and "lambda_right" in r:
                r["lambda"] = r6(0.5 * (raw[[c for c, k in cols.items() if k == "lambda_left"][0]]
                                        + raw[[c for c, k in cols.items() if k == "lambda_right"][0]]))
            if "icam_raw" in r:
                r["intake_cam_adv_deg"] = r["icam_raw"] if spec["icam"] == "adv" else -r["icam_raw"]
            if "ecam_raw" in r:
                r["exhaust_cam_ret_deg"] = r["ecam_raw"] if spec["ecam"] == "ret" else -r["ecam_raw"]
            rows.append(r)
    # selection
    bmep_max = max(r["bmep_bar"] for r in rows)
    best = {}
    for i, r in enumerate(rows):
        b = int(round(r["rpm"] / SPEED_BIN_RPM)) * SPEED_BIN_RPM
        r["speed_bin_rpm"] = b
        if b not in best or r["torque_nm"] > rows[best[b]]["torque_nm"]:
            best[b] = i
    for i, r in enumerate(rows):
        r["envelope"] = best[r["speed_bin_rpm"]] == i
        r["high_load"] = r["bmep_bar"] >= BMEP_FRACTION * bmep_max
    return {
        "format": "physics_sim.epa_turbo_rows/1",
        "package": key,
        "engine": spec["engine"],
        "fuel": spec["fuel"],
        "fuel_report": f"NVFEL Fuel Analysis Report {spec['report']}",
        "source": f"EPA NVFEL benchmark test-data package, workbook '{wb_path.name}' (package directory {wb_path.parent.name[2:]}); "
                  "public data, https://www.epa.gov/vehicle-and-fuel-emissions-testing/ (urls in out/epa_turbo_dl/urls.txt). "
                  "Literal workbook readings rounded to 6 significant digits; nothing derived except the normalised cam signs, "
                  "the two-bank lambda mean and the envelope / high_load flags.",
        "selection": f"high_load = bmep_bar >= {BMEP_FRACTION} * max(bmep_bar) = {r6(BMEP_FRACTION * bmep_max)} bar; "
                     f"envelope = highest torque among all rows of the {SPEED_BIN_RPM}-rpm speed bin",
        "cam_convention": {"intake": "workbook CAD " + ("ADV" if spec["icam"] == "adv" else "RET"),
                           "exhaust": "workbook CAD " + ("RET" if spec["ecam"] == "ret" else "ADV"),
                           "normalised": "intake_cam_adv_deg positive = advanced; exhaust_cam_ret_deg positive = retarded"},
        "columns": layout,
        "bmep_max_bar": r6(bmep_max),
        "row_count_all": len(rows),
        "rows": rows,
    }


def compact(d: dict) -> dict:
    c = {k: v for k, v in d.items() if k not in ("rows", "row_count_all", "columns")}
    keep = [r for r in d["rows"] if r["high_load"]]
    c["row_count_all"] = d["row_count_all"]
    c["row_count_selected"] = len(keep)
    c["rows"] = keep
    return c


def is_light(r: dict) -> bool:
    return (r.get("map_kpa", 1e9) <= LIGHT_MAP_KPA and LIGHT_LAMBDA[0] <= r.get("lambda", 0.0) <= LIGHT_LAMBDA[1]
            and r["bmep_bar"] >= LIGHT_BMEP_BAR and "spark_btdc_deg" in r)


def compact_light(d: dict) -> dict:
    c = {k: v for k, v in d.items() if k not in ("rows", "row_count_all", "columns")}
    keep = [r for r in d["rows"] if is_light(r)]
    c["selection"] = (f"light load: map_kpa <= {LIGHT_MAP_KPA}, {LIGHT_LAMBDA[0]} <= lambda <= {LIGHT_LAMBDA[1]}, "
                      f"bmep_bar >= {LIGHT_BMEP_BAR} (no-knock check: true knock limit >= measured spark)")
    c["row_count_all"] = d["row_count_all"]
    c["row_count_selected"] = len(keep)
    c["rows"] = keep
    return c


def dump(path: Path, obj: dict) -> None:
    """Header keys on one line each, one row per line (diff friendly), compact separators."""
    path.parent.mkdir(parents=True, exist_ok=True)
    sep = (",", ":")
    nl = chr(10)
    lines = ["{"]
    for k in obj:
        if k != "rows":
            lines.append(json.dumps(k) + ":" + json.dumps(obj[k], separators=sep) + ",")
    lines.append('"rows":[')
    lines.append(("," + nl).join(json.dumps(r, separators=sep) for r in obj["rows"]))
    lines.append("]}")
    path.write_text(nl.join(lines) + nl, encoding="utf-8")



def stats(d: dict) -> str:
    sel = [r for r in d["rows"] if r["high_load"]]
    env = [r for r in sel if r["envelope"]]

    def rng(rs, k):
        v = [r[k] for r in rs if k in r]
        return f"{min(v):.4g}..{max(v):.4g}" if v else "n/a"
    return (f"{d['package']}: all {d['row_count_all']}, selected {len(sel)}, envelope {len(env)}; "
            f"rpm {rng(sel, 'rpm')}; MAP {rng(sel, 'map_kpa')} kPa; spark {rng(sel, 'spark_btdc_deg')} deg BTDC; "
            f"lambda {rng(sel, 'lambda')}; torque {rng(sel, 'torque_nm')}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC)
    ap.add_argument("--stats", action="store_true", help="print the per-package summary only, write nothing")
    ap.add_argument("--package", action="append", help="limit to these package keys")
    args = ap.parse_args()
    for key in args.package or PACKAGES:
        d = extract(key, args.src)
        print(stats(d))
        if args.stats:
            continue
        dump(OUT_DIR / f"{key}_rows.json", d)
        dump(FIXTURE_DIR / f"{key}_rows.json", compact(d))
        dump(FIXTURE_DIR / f"{key}_light_rows.json", compact_light(d))
    return 0


if __name__ == "__main__":
    sys.exit(main())
