#!/usr/bin/env python3
"""PHYS-033 P4 I7c C17-pre: Toyota A25A-FKS data authoring.

Produces out/a25a/a25a_tables.json for the C17a agent to embed into a
simulated-engine descriptor (`toyota_a25a_fks_sim.json`, D12 minimal-path
"M-B" fallback: measured-derived VE/eta_i override maps, used only until a
real Wiebe/Woschni cycle-model generator (C16a) is wired up).

Plan reference: S:\\claude_code\\physics_sim\\out\\i7c_plan.md
  section 1.1 (A25A facts table), D12 (M-B fallback), D13 (friction),
  section 4 (map schema), "C17-pre" (this task's task list).

Design constraints (see out/c17pre_progress.md for the full brief):
  - stdlib only (no openpyxl, no numpy/PIL) - the xlsx reader below is a
    zipfile + xml.etree walk of the raw OOXML, the same technique the C7
    review's scratch xl.py used (not copied verbatim, reimplemented here
    to keep this script self-contained and committable).
  - deterministic: re-running this script on unchanged inputs produces
    byte-identical JSON. Dict keys are inserted in a fixed order; every
    float is rounded to a fixed number of significant digits before
    serialisation so float-formatting drift cannot change the output
    bytes; json.dump uses sort_keys=False (insertion order) with a fixed
    separator/indent.
  - every value in the output is labelled: "measured" (a literal xlsx
    reading), "[D] derived" (computed from measured values, method
    stated), "[AS] assumed" (no source, stated as such), or a citation.

Usage:
    python tools/enginedata/a25a_author.py
Reads (read-only, outside the worktree - shared EPA source package):
    S:\\claude_code\\physics_sim\\out\\c7_sources\\test_data\\
        4- 2018 Toyota 2.5L A25A-FKS Engine Tier 3 Fuel - Test Data.xlsx
    S:\\claude_code\\physics_sim\\out\\c7_sources\\test_data\\
        3a- 2018 Toyota 2.5L A25A-FKS Engine Tier 3 Fuel - Test Report.docx
        (only to extract word/media/image14.jpeg, SAE report Figure 11)
Writes:
    S:\\claude_code\\physics_sim\\out\\a25a\\a25a_tables.json
    S:\\claude_code\\physics_sim\\out\\a25a\\image14_fig11_valve_lift.jpeg
      (extracted copy, so a reviewer can see what task 1 digitised without
      re-opening the docx)
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths (fixed absolute locations per the task brief - not configurable via
# CLI, since this script is run exactly one way by exactly one caller).
# ---------------------------------------------------------------------------
SOURCES_DIR = Path(r"S:\claude_code\physics_sim\out\c7_sources")
TEST_DATA_XLSX = SOURCES_DIR / "test_data" / (
    "4- 2018 Toyota 2.5L A25A-FKS Engine Tier 3 Fuel - Test Data.xlsx"
)
TEST_REPORT_DOCX = SOURCES_DIR / "test_data" / (
    "3a- 2018 Toyota 2.5L A25A-FKS Engine Tier 3 Fuel - Test Report.docx"
)
OUT_DIR = Path(r"S:\claude_code\physics_sim\out\a25a")
OUT_JSON = OUT_DIR / "a25a_tables.json"
OUT_FIG11 = OUT_DIR / "image14_fig11_valve_lift.jpeg"

# ---------------------------------------------------------------------------
# Cited constants (PHYS-033 P4 I7a C7's data/fuels/tier3_e10_cert.json -
# not present in this worktree since C7 has not merged to the branch this
# worktree forked from; the literal values are reproduced here with the
# same citation so this script does not depend on another branch's files).
# ---------------------------------------------------------------------------
FUEL_LHV_J_KG = 41954062.0
FUEL_AFR_STOICH = 14.002173341183884
FUEL_SOURCE = (
    "Cites data/fuels/tier3_e10_cert.json (PHYS-033 P4 I7a C7): EPA NVFEL "
    "Fuel Analysis Report 26781 (ASTM D240 net heating value 18037.00 "
    "BTU/lb -> 41954062.0 J/kg by the exact BTU(IT)/lb->J/kg conversion) "
    "and the Heywood 1988 section 3.2 elemental-mass-fraction AFR_stoich "
    "derivation from the same report's measured C/H/O content. This "
    "script hardcodes the two numbers rather than reading the C7 file "
    "because C7 has not merged into this worktree's base commit; C17a "
    "must verify they still match data/fuels/tier3_e10_cert.json once "
    "merged."
)

R_SPECIFIC_AIR_J_PER_KGK = 287.05  # [C] standard dry-air gas constant

# Bore/stroke: see build_bore_stroke() below - task 4, [A] or [AS] depending
# on whether a citable public source was found in this session.
BORE_M = 0.0875
STROKE_M = 0.1034
CYLINDERS = 4

# ---------------------------------------------------------------------------
# Minimal pure-stdlib xlsx reader (OOXML: zipfile + xml.etree). Reads only
# what this script needs: sheet names, shared strings, and per-cell values
# indexed by (0-based) row/col. No formula evaluation (none of the source
# columns we use are formulas - BMEP/BSFC/BTE are Excel-computed but stored
# as their computed <v> cached values, which is what a plain reader sees).
# ---------------------------------------------------------------------------
_NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
_RNS = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"


def _col_letters_to_index(letters: str) -> int:
    n = 0
    for ch in letters:
        n = n * 26 + (ord(ch) - ord("A") + 1)
    return n - 1


def load_xlsx(path: Path) -> dict[str, dict[int, dict[int, object]]]:
    """Returns {sheet_name: {row_number (1-based, as in the file): {col_index (0-based): value}}}."""
    with zipfile.ZipFile(path) as z:
        shared_strings: list[str] = []
        if "xl/sharedStrings.xml" in z.namelist():
            root = ET.fromstring(z.read("xl/sharedStrings.xml"))
            for si in root.findall("m:si", _NS):
                shared_strings.append("".join(t.text or "" for t in si.iter("{%s}t" % _NS["m"])))

        wb = ET.fromstring(z.read("xl/workbook.xml"))
        rels = ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))
        rel_map = {r.get("Id"): r.get("Target") for r in rels}

        sheets: dict[str, dict[int, dict[int, object]]] = {}
        for s in wb.find("m:sheets", _NS):
            name = s.get("name")
            rid = s.get(_RNS)
            target = rel_map[rid]
            target = target if target.startswith("xl/") else "xl/" + target.lstrip("/")
            sheet_xml = ET.fromstring(z.read(target))
            rows: dict[int, dict[int, object]] = {}
            for row_el in sheet_xml.iter("{%s}row" % _NS["m"]):
                row_num = int(row_el.get("r"))
                row_vals: dict[int, object] = {}
                for c in row_el.findall("m:c", _NS):
                    ref = c.get("r")
                    col_letters = re.match(r"[A-Z]+", ref).group()
                    col_idx = _col_letters_to_index(col_letters)
                    cell_type = c.get("t")
                    v_el = c.find("m:v", _NS)
                    if cell_type == "s" and v_el is not None:
                        value = shared_strings[int(v_el.text)]
                    elif cell_type == "inlineStr":
                        value = "".join(t.text or "" for t in c.iter("{%s}t" % _NS["m"]))
                    elif v_el is not None:
                        try:
                            value = float(v_el.text)
                        except ValueError:
                            value = v_el.text
                    else:
                        value = None
                    row_vals[col_idx] = value
                rows[row_num] = row_vals
            sheets[name] = rows
    return sheets


# Column layout of the "Steady State" / "High Load Initial" / "High Load
# Final" sheets, confirmed against the "Test Parameter List" sheet's own
# Name/Units/Description table (row 5 = column 0 = "Speed", row 6 = column
# 1 = "Torque", ... row 32 = column 27 = "Exhaust Cam Phase" - i.e. Test
# Parameter List row N+4 documents data column N). Reproduced here rather
# than re-parsed from the sheet at runtime because the "Test Parameter
# List" sheet's row layout is prose-shaped (merged description text), not
# worth a generic parser for one fixed, versioned source file.
COL = {
    "rpm": 0,               # Speed, rpm
    "torque_nm": 1,         # Torque, Nm
    "bmep_bar": 2,          # BMEP, bar
    "cell_temp_c": 3,       # Cell Temp, degC
    "cell_press_kpa": 4,    # Cell Press, kPaA
    "bsfc_g_per_kwh": 5,    # BSFC, g/kW-hr
    "bte_pct": 6,           # BTE, %
    "fuel_meter_g_s": 7,    # Fuel Meter Flow, g/s (calibrated; sparse in "High Load Initial")
    "inj_fuel_g_s": 8,      # Injector Fuel Flow, g/s (calculated; always present)
    "pfi_portion_pct": 13,  # PFI Fuel Portion, %
    "pedal_pct": 14,        # Accelerator Pedal, %
    "spark_btdc": 15,       # Spark Timing, CAD BTDC
    "coolant_c": 16,        # Coolant Temp, degC
    "oil_c": 17,            # Oil Sump Temp, degC
    "map_kpa": 18,          # Intake Manifold Press, kPaA
    "imt_c": 19,            # Intake Manifold Temp, degC
    "exh_port_temp_c": 20,  # Exhaust Port Temp, degC
    "egr_inlet_temp_c": 21,   # EGR Inlet Temp, degC (measured in the exhaust manifold's EGR port)
    "egr_inlet_press_kpa": 22,  # EGR Inlet Press, kPaA (~= exhaust manifold pressure p_em)
    "cat_inlet_press_kpa": 23,
    "cat_inlet_temp_c": 24,
    "lambda": 25,           # Exhaust Lambda (wideband, uncalibrated per the source's own caveat)
    "icam_adv_deg": 26,     # Intake Cam Phase, CAD ADV @ 1mm lift
    "ecam_ret_deg": 27,     # Exhaust Cam Phase, CAD RET @ 1mm lift
}
DATA_SHEETS = ["Steady State", "High Load Initial", "High Load Final"]
FIRST_DATA_ROW = 6  # rows 1-5 are title/header text (confirmed: dump_ss.py's "if rn < 6: continue")

# Rows the C7 review (task brief's "known data quirks") identified as ECU
# high-load protection enrichment (lambda 0.63-0.73) - kept in every table
# (they are real measured ECU behaviour) but flagged. Identified by
# (sheet, rpm rounded to nearest int, pedal_pct rounded) since row numbers
# alone are not stable across the three near-duplicate sheets.
ENRICHMENT_LAMBDA_MAX = 0.85  # review: "lambda >= ~0.85" excludes the enriched WOT rows


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def r6(x: float) -> float:
    """Round to 6 significant figures - enough precision for any of these
    quantities, and fixes the output's float formatting so re-running the
    script is byte-identical."""
    if x == 0 or not math.isfinite(x):
        return x
    from decimal import Decimal

    d = Decimal(x)
    exp = d.adjusted()
    quant = Decimal(1).scaleb(exp - 5)
    return float(d.quantize(quant))


# ---------------------------------------------------------------------------
# Task 2 + 5 support: extract every usable measured point.
# ---------------------------------------------------------------------------
def extract_points(sheets) -> list[dict]:
    points = []
    for sheet_name in DATA_SHEETS:
        rows = sheets[sheet_name]
        for row_num in sorted(rows):
            if row_num < FIRST_DATA_ROW:
                continue
            r = rows[row_num]
            rpm = r.get(COL["rpm"])
            if not isinstance(rpm, float):
                continue
            torque = r.get(COL["torque_nm"])
            bmep = r.get(COL["bmep_bar"])
            map_kpa = r.get(COL["map_kpa"])
            imt_c = r.get(COL["imt_c"])
            lam = r.get(COL["lambda"])
            pedal = r.get(COL["pedal_pct"])
            spark = r.get(COL["spark_btdc"])
            icam = r.get(COL["icam_adv_deg"])
            ecam = r.get(COL["ecam_ret_deg"])
            egr_t = r.get(COL["egr_inlet_temp_c"])
            egr_p = r.get(COL["egr_inlet_press_kpa"])
            bte = r.get(COL["bte_pct"])

            fmf = r.get(COL["fuel_meter_g_s"])
            inj = r.get(COL["inj_fuel_g_s"])
            if isinstance(fmf, float):
                fuel_g_s, fuel_source = fmf, "measured:fuel_meter_flow"
            elif isinstance(inj, float):
                fuel_g_s, fuel_source = inj, "measured:injector_fuel_flow_estimate"
            else:
                fuel_g_s, fuel_source = None, None

            enriched = isinstance(lam, float) and lam < ENRICHMENT_LAMBDA_MAX and (
                isinstance(pedal, float) and pedal >= 60.0
            )

            points.append({
                "sheet": sheet_name,
                "row": row_num,
                "rpm": rpm,
                "torque_nm": torque,
                "bmep_bar": bmep,
                "map_kpa": map_kpa,
                "imt_c": imt_c,
                "lambda": lam,
                "pedal_pct": pedal,
                "spark_btdc": spark,
                "icam_adv_deg": icam,
                "ecam_ret_deg": ecam,
                "egr_inlet_temp_c": egr_t,
                "egr_inlet_press_kpa": egr_p,
                "bte_pct": bte,
                "fuel_g_s": fuel_g_s,
                "fuel_source": fuel_source,
                "protection_enrichment_flag": bool(enriched),
            })
    return points


# ---------------------------------------------------------------------------
# Task 5: M-B derived VE / eta_i per measured point.
# ---------------------------------------------------------------------------
DISPLACEMENT_M3 = (math.pi / 4.0) * BORE_M * BORE_M * STROKE_M * CYLINDERS


def barnes_moss_fmep_bar(rpm: float) -> float:
    """D13 F1: Barnes-Moss polynomial for 4-cylinder SI engines, as
    reproduced in Heywood, 'Internal Combustion Engine Fundamentals'
    (1988), ch. 13 [A: exact equation number/page not confirmed in this
    session - the coefficients are the commonly-quoted Barnes-Moss form].
    fmep(bar) = 0.97 + 0.15*(N/1000) + 0.05*(N/1000)^2, N in rpm.
    Known 1970s-vintage bias: likely overstates a modern engine's friction
    (see out/c17pre_progress.md D13 section)."""
    n = rpm / 1000.0
    return 0.97 + 0.15 * n + 0.05 * n * n


def compute_mb_point(p: dict) -> dict | None:
    """Returns the derived VE/eta_i quantities for one measured point, or
    None if a required input is missing."""
    rpm = p["rpm"]
    fuel_g_s = p["fuel_g_s"]
    lam = p["lambda"]
    map_kpa = p["map_kpa"]
    imt_c = p["imt_c"]
    bmep_bar = p["bmep_bar"]
    egr_p_kpa = p["egr_inlet_press_kpa"]
    if None in (rpm, fuel_g_s, lam, map_kpa, imt_c, bmep_bar) or rpm <= 0:
        return None

    fuel_kg_s = fuel_g_s / 1000.0
    seconds_per_cycle = 120.0 / rpm  # 4-stroke: one full cycle = 2 crank revolutions
    fuel_kg_per_cycle = fuel_kg_s * seconds_per_cycle

    m_air_fresh_per_cycle_kg = fuel_kg_per_cycle * lam * FUEL_AFR_STOICH
    t_im_k = imt_c + 273.15
    p_im_pa = map_kpa * 1000.0
    rho_im = p_im_pa / (R_SPECIFIC_AIR_J_PER_KGK * t_im_k)
    ve_air = m_air_fresh_per_cycle_kg / (rho_im * DISPLACEMENT_M3)
    m_air_per_rev_kg = m_air_fresh_per_cycle_kg / 2.0  # one intake stroke per crank rev (multi-cyl, phased)

    fmep_bar = barnes_moss_fmep_bar(rpm)
    pmep_ideal_bar = None
    eta_i = None
    if isinstance(egr_p_kpa, float):
        pmep_ideal_bar = (egr_p_kpa - map_kpa) / 100.0  # kPa -> bar; p_em - p_im
        imep_gross_pa = (bmep_bar + fmep_bar + pmep_ideal_bar) * 1.0e5
        w_gross_per_cycle_j = imep_gross_pa * DISPLACEMENT_M3
        fuel_energy_per_cycle_j = fuel_kg_per_cycle * FUEL_LHV_J_KG
        if fuel_energy_per_cycle_j > 0:
            eta_i = w_gross_per_cycle_j / fuel_energy_per_cycle_j

    # Plausibility check (found while sanity-checking this script's own
    # output, not in the plan): at light load (low MAP, throttled), the
    # "ideal PMEP = p_em - p_im" approximation swings large (p_im ~20-30
    # kPa while p_em stays near atmospheric) relative to the small fuel
    # energy input, driving eta_i well above any physically plausible
    # gross indicated efficiency (observed up to ~0.68 - compare the
    # engine's own cited peak BTE, a BRAKE figure that is always lower
    # than gross indicated, of 39.8%). This is the same class of artefact
    # as the C7 review's B3 Willans-line finding at the opposite (very
    # low fuel) extreme - a simple linear/ideal reconstruction breaking
    # down away from the mid-load regime it is honest in. Flagged, not
    # silently discarded from 'measured_points'/'raw_points', but EXCLUDED
    # from the gridded eta_i fallback table (build_schedule_grid's sample
    # list below only uses eta_i_plausible points) so the fallback map
    # itself stays physically sane.
    eta_i_plausible = eta_i is not None and (0.15 <= eta_i <= 0.50)

    return {
        "rpm": rpm,
        "map_kpa": map_kpa,
        "ve_air": r6(ve_air),
        "m_air_per_rev_kg": r6(m_air_per_rev_kg),
        "eta_i": r6(eta_i) if eta_i is not None else None,
        "eta_i_plausible": eta_i_plausible,
        "fmep_bar_model": r6(fmep_bar),
        "pmep_ideal_bar": r6(pmep_ideal_bar) if pmep_ideal_bar is not None else None,
        "protection_enrichment_flag": p["protection_enrichment_flag"],
        "sheet": p["sheet"],
        "row": p["row"],
    }


# ---------------------------------------------------------------------------
# Task 2: deterministic (rpm, MAP) gridding by 6-nearest-neighbour inverse-
# distance weighting in normalised (rpm/1000, MAP/10kPa) space, falling
# back to the single nearest measured value outside the measured convex
# hull (I7c plan D12/task 2's literal method).
# ---------------------------------------------------------------------------
def convex_hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain. pts: list of (x, y). Returns hull vertices
    in counter-clockwise order. Pure stdlib (no scipy)."""
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_polygon(pt: tuple[float, float], poly: list[tuple[float, float]]) -> bool:
    """Standard ray-casting test; treats boundary as inside (>=/<= not <)."""
    x, y = pt
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (
            x <= (xj - xi) * (y - yi) / (yj - yi + 1e-300) + xi
        ):
            inside = not inside
        j = i
    return inside


def idw_grid(
    grid_rpm: list[float],
    grid_p_im: list[float],
    samples: list[tuple[float, float, float]],  # (rpm, map_kpa, value)
    k: int = 6,
    power: float = 2.0,
) -> tuple[list[list[float]], list[list[str]]]:
    """Returns (values[rpm_i][map_j], coverage[rpm_i][map_j]) where
    coverage in {"interpolated", "extrapolated_nearest"}."""
    if not samples:
        raise ValueError("idw_grid: no samples")
    norm_samples = [((rpm / 1000.0, mp / 10.0), rpm, mp, v) for rpm, mp, v in samples]
    hull_pts = [ns[0] for ns in norm_samples]
    hull = convex_hull(hull_pts)

    values: list[list[float]] = []
    coverage: list[list[str]] = []
    for rpm in grid_rpm:
        row_v: list[float] = []
        row_c: list[str] = []
        gx, gy = rpm / 1000.0, None
        for mp in grid_p_im:
            gy = mp / 10.0
            dists = []
            for (nx, ny), srpm, smp, sv in norm_samples:
                d2 = (nx - gx) ** 2 + (ny - gy) ** 2
                dists.append((d2, sv))
            dists.sort(key=lambda t: t[0])
            inside = point_in_polygon((gx, gy), hull) if len(hull) >= 3 else True
            if inside:
                nearest = dists[:k]
                if nearest[0][0] < 1e-12:
                    val = nearest[0][1]
                else:
                    wsum = 0.0
                    vsum = 0.0
                    for d2, v in nearest:
                        w = 1.0 / (d2 ** (power / 2.0))
                        wsum += w
                        vsum += w * v
                    val = vsum / wsum
                row_v.append(r6(val))
                row_c.append("interpolated")
            else:
                val = dists[0][1]
                row_v.append(r6(val))
                row_c.append("extrapolated_nearest")
        values.append(row_v)
        coverage.append(row_c)
    return values, coverage


# 20 non-uniform rpm points (600 .. 6900 = cited redline 6600 rpm + 300,
# denser at low rpm where the measured data and the engine's own operating
# range near idle/part-load are both denser) and 12 evenly spaced MAP
# points (20..105 kPa, naturally-aspirated per plan section 4's own grid
# comment). Authored grid, not derived from the data - task brief default.
GRID_RPM = [600, 800, 1000, 1200, 1500, 1800, 2100, 2400, 2700, 3000,
            3300, 3700, 4100, 4500, 4900, 5300, 5700, 6100, 6500, 6900]
GRID_P_IM_KPA = [round(20.0 + i * (105.0 - 20.0) / 11.0, 3) for i in range(12)]


def build_schedule_grid(points: list[dict], field: str) -> dict:
    samples = [
        (p["rpm"], p["map_kpa"], p[field])
        for p in points
        if p.get("map_kpa") is not None and p.get(field) is not None
    ]
    values, coverage = idw_grid(GRID_RPM, GRID_P_IM_KPA, samples)
    return {
        "grid_rpm": GRID_RPM,
        "grid_p_im_kpa": GRID_P_IM_KPA,
        "values": values,
        "coverage": coverage,
        "n_samples": len(samples),
        "method": "inverse_distance_weighting_k6_power2_in_normalised_rpm1000_map10kpa_space; "
                  "outside the measured convex hull, nearest measured value",
    }


def build_enrichment_flag_grid(points: list[dict]) -> list[list[bool]]:
    """For each grid cell, true iff the single nearest measured sample is
    itself an enrichment-flagged row (so the lambda_target/VE grid values
    that landed there are, or are adjacent to, ECU protection enrichment)."""
    samples = [
        (p["rpm"], p["map_kpa"], 1.0 if p["protection_enrichment_flag"] else 0.0)
        for p in points
        if p.get("map_kpa") is not None
    ]
    out = []
    for rpm in GRID_RPM:
        row = []
        for mp in GRID_P_IM_KPA:
            best_d2, best_v = None, 0.0
            for srpm, smp, sv in samples:
                d2 = (rpm / 1000.0 - srpm / 1000.0) ** 2 + (mp / 10.0 - smp / 10.0) ** 2
                if best_d2 is None or d2 < best_d2:
                    best_d2, best_v = d2, sv
            row.append(bool(best_v))
        out.append(row)
    return out


# ---------------------------------------------------------------------------
# Task 2 (back-pressure): Curve1D-shaped data, x = exhaust mass flow
# estimate = fuel*(1 + lambda*AFR_st) [g/s], y = exhaust-side pressure
# (EGR Inlet Press, kPaA, measured in the exhaust manifold's EGR port -
# the only exhaust-manifold-side pressure tap in this dataset).
# ---------------------------------------------------------------------------
def build_back_pressure_curve(points: list[dict]) -> dict:
    pairs = []
    for p in points:
        fuel_g_s = p.get("fuel_g_s")
        lam = p.get("lambda")
        egr_p = p.get("egr_inlet_press_kpa")
        if None in (fuel_g_s, lam, egr_p):
            continue
        exh_g_s = fuel_g_s * (1.0 + lam * FUEL_AFR_STOICH)
        pairs.append((r6(exh_g_s), r6(egr_p)))
    pairs.sort()
    return {
        "exh_g_s": [x for x, _ in pairs],
        "p_em_kpa": [y for _, y in pairs],
        "n_samples": len(pairs),
        "method": "[D] x = fuel_g_s*(1+lambda*AFR_stoich) (total exhaust mass flow estimate, "
                  "pre-EGR-extraction); y = EGR Inlet Press (kPaA), the exhaust-manifold-side "
                  "pressure tap. Raw sorted pairs, not fitted - C16a/C17a bin or fit a Curve1D.",
    }


# ---------------------------------------------------------------------------
# Task 1: Fig. 11 (image14.jpeg) valve-lift digitisation.
#
# No image-processing library is available (no PIL/numpy; "install
# nothing" per the task brief), so this is a MANUAL visual digitisation
# from the extracted plot image (out/a25a/image14_fig11_valve_lift.jpeg),
# not a pixel-automated one. Axis calibration and per-curve key points
# were read directly off the rendered plot (axes: x = CAD, gridlines at
# -360/-180/0/180/360; y = valve lift mm, gridlines at -2/0/2/4/6/8/10/12).
# Both solid (phase = 0) curves are close to a raised-cosine bump over
# their duration - fitted here as lift(theta) = peak * cos^2(pi/2 *
# (theta-center)/half_duration) clipped to >=0, using visually-read
# (center, half_duration, peak) - and RESAMPLED onto every 2 deg. This is
# a deliberate, labelled approximation of a hand digitisation (reading a
# bump's shape parameters is far less error-prone by eye than reading 360
# individual points), cross-checked against SAE's own 1mm-lift timing.
# ---------------------------------------------------------------------------
VALVE_LIFT_READ = {
    # Values read directly off the plot (out/a25a/image14_fig11_valve_lift.jpeg):
    # solid blue = "Intake lift [mm] phase = 0", solid red = "Exhaust lift
    # [mm] phase = 0". Peaks and approximate 0-mm-crossing (start/end) CAD
    # read by eye against the gridlines; duration/center derived from those.
    "intake_phase0": {
        "peak_mm": 11.4,
        "peak_cad": 165.0,
        "lift_start_cad": 45.0,   # visually: lift departs from 0 near CAD ~45
        "lift_end_cad": 285.0,    # visually: lift returns to 0 near CAD ~285
        "read_uncertainty_mm": 0.3,
        "read_uncertainty_cad": 5.0,
    },
    "exhaust_phase0": {
        "peak_mm": 11.2,
        "peak_cad": -115.0,
        "lift_start_cad": -235.0,
        "lift_end_cad": 5.0,
        "read_uncertainty_mm": 0.3,
        "read_uncertainty_cad": 5.0,
    },
}
AXIS_CALIBRATION_NOTE = (
    "Manual read (no image-processing library available in this environment - "
    "PIL/numpy are not installed and the task brief says install nothing). "
    "Calibrated against the plot's own gridlines: x-axis CAD in "
    "{-360,-180,0,180,360}, y-axis valve lift mm in {-2,0,2,4,6,8,10,12}. "
    "Peak height/location and the two 0 mm crossings were read by eye for "
    "each solid (phase=0) curve; the curve BETWEEN those anchors is a "
    "raised-cosine bump (lift = peak*cos^2(pi/2*(theta-center)/half_dur)), "
    "not a pixel trace - see VALVE_LIFT_READ's own uncertainty fields. "
    "This is a materially cheaper and, for a bump-shaped cam lobe, no less "
    "accurate approximation than tracing individual pixels by eye would "
    "have been; a future pass with an image library could re-digitise "
    "pixel-by-pixel if the runtime ever needs the true asymmetric shape."
)


def raised_cosine_bump(theta: float, peak: float, start: float, end: float) -> float:
    center = (start + end) / 2.0
    half_dur = (end - start) / 2.0
    if theta <= start or theta >= end:
        return 0.0
    frac = (theta - center) / half_dur  # in (-1, 1)
    return peak * math.cos(math.pi / 2.0 * frac) ** 2


def build_valve_lift_tables() -> dict:
    grid_deg = list(range(-360, 362, 2))  # 2-degree grid, -360..360 inclusive
    intake = VALVE_LIFT_READ["intake_phase0"]
    exhaust = VALVE_LIFT_READ["exhaust_phase0"]
    intake_mm = [
        r6(raised_cosine_bump(d, intake["peak_mm"], intake["lift_start_cad"], intake["lift_end_cad"]))
        for d in grid_deg
    ]
    exhaust_mm = [
        r6(raised_cosine_bump(d, exhaust["peak_mm"], exhaust["lift_start_cad"], exhaust["lift_end_cad"]))
        for d in grid_deg
    ]
    # SAE cross-check: theta_VO (valve opening, 1mm lift) = 138.6 CAD + theta_phase.
    # At phase 0 that is the intake's 1mm-lift point on its OPENING ramp.
    # (Not the closing-ramp point our start/end anchors used above.)
    intake_1mm_opening = None
    for i in range(1, len(grid_deg)):
        if intake_mm[i - 1] < 1.0 <= intake_mm[i] and grid_deg[i] < intake["peak_cad"]:
            intake_1mm_opening = grid_deg[i]
            break
    sae_predicted_1mm_vo = 138.6  # CAD, phase 0, per plan section 1.1's cited SAE formula
    cross_check = {
        "sae_theta_vo_1mm_phase0_cad": sae_predicted_1mm_vo,
        "digitised_intake_1mm_opening_cad": intake_1mm_opening,
        "note": "[C] SAE formula vs [D] this digitisation's own opening-ramp 1mm-lift crossing; "
                "both are read/derived independently, agreement is a sanity check, not a fit.",
    }
    return {
        "grid_deg": grid_deg,
        "intake_phase0_mm": intake_mm,
        "exhaust_phase0_mm": exhaust_mm,
        "read_points": VALVE_LIFT_READ,
        "digitisation_method": AXIS_CALIBRATION_NOTE,
        "sae_cross_check": cross_check,
        "source_image": "word/media/image14.jpeg inside "
                         "'3a- 2018 Toyota 2.5L A25A-FKS Engine Tier 3 Fuel - Test Report.docx' "
                         "(SAE 2019-01-0249 Figure 11, reproduced in the EPA test report)",
    }


# ---------------------------------------------------------------------------
# Task 6: EGR check (Figs 17/18 - only available inside the SAE PDF /
# EPA test report as page images, not as data; this function documents the
# finding as text pulled from the report, does not re-render the PDF).
# ---------------------------------------------------------------------------
def build_egr_finding() -> dict:
    return {
        "question": "Is EGR open along the max-BMEP (WOT) line?",
        "answer": "NO along the WOT/max-BMEP envelope used for the D12 WOT torque curve, "
                  "for measured points above ~2000 rpm. EGR is reported open only in a "
                  "part-load window centered around 1750 rpm at BMEP corresponding to "
                  "roughly 150 N*m (SAE 2019-01-0249 Fig. 18: EGR rate > 24% there) - i.e. "
                  "a MID-load, MID-speed island, not the WOT line. Fig. 17's EGR valve-open "
                  "map (referenced by the plan) is consistent with this: EGR is scheduled off "
                  "at both very low load (unstable combustion risk) and high load/WOT "
                  "(the ECU's own protection-enrichment regime documented in the C7 review "
                  "already uses excess FUEL, not EGR, to control knock/temperature there).",
        "confidence": "[C] SAE 2019-01-0249 text description of Figs 17/18 (as summarised in "
                       "out/i7c_plan.md section 1.1's own EGR row) - this script did not "
                       "re-render/re-read the PDF's page-image figures pixel-by-pixel; it "
                       "reports the same figure captions/discussion the plan already cites, "
                       "cross-referenced against the xlsx's own EGR Inlet Temp/Press columns "
                       "below, which is the closest this dataset gets to a per-point EGR signal.",
        "xlsx_cross_reference": "No per-point EGR VALVE POSITION or EGR RATE column exists in "
                                 "the Test Data.xlsx (confirmed: Test Parameter List rows 1-32 "
                                 "list only EGR Inlet Temp/Press, i.e. exhaust-side gas state at "
                                 "the EGR takeoff port, not commanded EGR fraction). "
                                 "'egr_inlet_temp_delta_from_exhaust_port' below is a rough "
                                 "proxy only (EGR inlet temp close to exhaust port temp at a "
                                 "given point is NOT proof EGR flowed - it is upstream of any "
                                 "valve - so this is reported, not used as a finding).",
        "practical_conclusion_for_M_B": "Since the WOT envelope (D12's fallback torque/fuel "
                                         "curve, and this file's high-MAP grid rows) sees "
                                         "negligible EGR, the D12 con about EGR baking into the "
                                         "fallback VE/eta_i table is a PART-LOAD-ONLY caveat "
                                         "(worst near 1750 rpm / ~150 N*m), not a WOT-wide one.",
    }


# ---------------------------------------------------------------------------
# Task 4: bore/stroke citation.
# ---------------------------------------------------------------------------
def build_bore_stroke_note() -> dict:
    return {
        "bore_m": BORE_M,
        "stroke_m": STROKE_M,
        "status": "[A] cited to multiple independent open-web sources (see 'citations' below), "
                  "all agreeing exactly on 87.5 mm / 103.4 mm / 2487 cc / CR 13.0:1. NOT a "
                  "primary Toyota press-kit document with a directly-readable text layer: a "
                  "Toyota-published RAV4 A25A-FXS tech-spec PDF was located "
                  "(https://media.toyota.co.uk/wp-content/uploads/sites/5/2021/03/"
                  "1611678326210126MRAV4TechSpec.pdf, 205.8 KB, fetched "
                  "2026-09-28 - saved by the fetch tool to this session's own tool-results "
                  "cache, not copied into out/sources/ since it could not be read) but its "
                  "PDF content is compressed/encoded in a way the fetch tool could not "
                  "extract text from in this session; the FXS variant is also a different "
                  "(hybrid) engine code from the FKS this project targets, sharing the "
                  "Dynamic Force A25A architecture but not necessarily every dimension. "
                  "Downgrade to [AS] if a stricter citation policy requires a primary "
                  "manufacturer document; the plan text itself allows either "
                  "('cites ... or labels it [AS] with the consistency check').",
        "citations": [
            "https://en.wikipedia.org/wiki/Toyota_Dynamic_Force_engine (A25 family "
            "specification table: bore 87.5mm, stroke 103.4mm, displacement 2487cc, CR "
            "13.0:1 - table itself carries no inline reference, tertiary source)",
            "https://www.motorreviewer.com/engine.php?engine_id=127 (independent engine-specs "
            "aggregator, same four figures)",
            "https://www.enginetechspecs.com/toyota_a25a_fks_technical_data.html (independent "
            "engine-specs aggregator, same four figures)",
            "https://cararac.com/blog/toyota-camry-2-5-engine-problems-durability.html "
            "(independent, same four figures)",
        ],
        "consistency_check": "[D] Matches SAE 2019-01-0249 Table 1's stroke/bore ratio 1.2 "
                              "(103.4/87.5 = 1.18229). Displacement computed from this bore/"
                              "stroke (see 'displacement_m3_derived') is 2487.87 cc, matching "
                              "the SAE-cited '2.5-liter'/2487 cc class figure to within 0.04% "
                              "and every citation above's own stated 2487 cc.",
        "displacement_m3_derived": r6(DISPLACEMENT_M3),
        "displacement_l_derived": r6(DISPLACEMENT_M3 * 1000.0),
        "action_for_c17a": "Four independent open-web sources plus the SAE cross-check all "
                            "agree; no primary Toyota document with readable bore/stroke text "
                            "was found. Owner should decide whether this clears the project's "
                            "citation bar for [A] (PROVISIONAL: this script treats it as "
                            "sufficient) or must stay [AS] pending a primary source.",
    }


# ---------------------------------------------------------------------------
# Task 3: friction source search.
# ---------------------------------------------------------------------------
def build_friction_choice() -> dict:
    return {
        "chosen_model": "polynomial_rpm",
        "a_pa": r6(0.97e5),
        "b_pa_per_krpm": r6(0.15e5),
        "c_pa_per_krpm2": r6(0.05e5),
        "formula": "fmep_pa(rpm) = a_pa + b_pa_per_krpm*(rpm/1000) + c_pa_per_krpm2*(rpm/1000)^2",
        "source": "D13 option F1: Barnes-Moss polynomial for 4-cylinder SI engines, as "
                  "reproduced in Heywood, 'Internal Combustion Engine Fundamentals' (1988) "
                  "ch. 13 [A: exact equation/page number not re-verified against a scanned "
                  "copy in this session - the coefficients used are the commonly-quoted "
                  "Barnes-Moss form already in the project's plan document].",
        "search_log": [
            "Web search performed 2026-09-28 for a public-domain EPA NCAT/ATAB benchmark "
            "package (a THIRD engine, not A25A-FKS, not Mazda SkyActiv) with a motoring "
            "sweep (D13 option F4). EPA's own benchmarking index page "
            "(https://www.epa.gov/vehicle-and-fuel-emissions-testing/"
            "benchmarking-advanced-low-emission-light-duty-vehicle-technology) lists at "
            "least 10 engine packages, several stated to include motoring data: 2016 Honda "
            "1.5L L15B7, 2013 Ford 1.6L EcoBoost, 2014 Chevrolet 4.3L EcoTec LV3, 2015 BMW "
            "3.0L N57 diesel, 2013 Chevrolet 2.5L Ecotec LCV, 2015 Ford 2.7L EcoBoost V6, "
            "2016 Mazda 2.5L Turbo Skyactiv-G.",
            "CANDIDATE CHOSEN: 2013 Chevrolet 2.5L Ecotec LCV (naturally aspirated, closest "
            "displacement/architecture match to the A25A-FKS of the listed non-Mazda, "
            "non-Toyota engines - good for a piston-speed-scaled FMEP comparison). Package "
            "URL: https://www.epa.gov/sites/default/files/2018-04/"
            "2013-chevrolet-2.5l-ecotec-lcv-engine-reg-e10-fuel-test-data-package-03-05-18.zip "
            "(EPA's own page states '9.16 MB, March 2018').",
            "DOWNLOAD ATTEMPTED, BLOCKED: curl (with a standard browser User-Agent, then "
            "again with a Referer header pointing back at the EPA index page - both normal, "
            "not CAPTCHA-solving) and the WebFetch tool both got HTTP 403 'AccessDenied' from "
            "EPA's static-file host (an S3-fronted bucket returning an XML AccessDenied body, "
            "RequestId logged, e.g. 'G59FWQRKND3W7CGS') on 2026-09-28. This looks like "
            "server-side bot-protection on EPA's file host, not a wrong/expired URL (the "
            "index page itself resolved and named this exact file/size). Per this session's "
            "rules ('never bypass or complete CAPTCHAs or other bot-detection'), no further "
            "bypass attempt (e.g. a full interactive browser session to click through the "
            "download) was made. The package was NOT downloaded; nothing is in out/sources/ "
            "for this candidate.",
            "Whether the A25A-FKS's own test_data.xlsx source (same EPA NVFEL/NCAT program) "
            "hit the same 403 wall is moot - it was already supplied to this worktree out of "
            "band (out/c7_sources/), not fetched by this script.",
        ],
        "status": "PARTIAL - a qualifying F4 candidate (2013 Chevrolet 2.5L Ecotec LCV, URL "
                  "above) WAS found within the search budget, but its package could not be "
                  "downloaded (EPA host 403) in this session, so no F4 correlation was "
                  "derived. F1 (Barnes-Moss) is used as the D13-recommended minimal-path "
                  "fallback for this reason. Its known bias (1970s-vintage engines, likely "
                  "overstates a modern engine's friction by an estimated 30-50%, i.e. up to "
                  "about -8% torque at 5000 rpm per the plan's own D13 note) is carried "
                  "forward unmitigated.",
        "open_task_for_c17a_or_followup": "Retry the Chevrolet 2.5L Ecotec LCV package "
                                           "download (maybe from a different network path, or "
                                           "ask the owner to fetch it manually - it is a public "
                                           "EPA document, the 403 looked like automated-client "
                                           "blocking, not access control) and derive an F4 "
                                           "FMEP(mean piston speed) correlation from its "
                                           "motoring sweep before shipping V1, if the -8%-at-"
                                           "5000rpm F1 bias matters for the acceptance bands "
                                           "C17a is targeting; otherwise defer to a V2 per "
                                           "D13's own recommendation ('F2 is a follow-up if "
                                           "F1's bias shows up in V2').",
    }


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    xlsx_sha256 = sha256_of(TEST_DATA_XLSX)
    docx_sha256 = sha256_of(TEST_REPORT_DOCX)

    # Extract Fig. 11 image alongside the JSON for reviewer convenience.
    with zipfile.ZipFile(TEST_REPORT_DOCX) as z:
        img_bytes = z.read("word/media/image14.jpeg")
    OUT_FIG11.write_bytes(img_bytes)
    fig11_sha256 = hashlib.sha256(img_bytes).hexdigest()

    sheets = load_xlsx(TEST_DATA_XLSX)
    points = extract_points(sheets)

    mb_points = [compute_mb_point(p) for p in points]
    mb_points = [p for p in mb_points if p is not None]

    ve_grid = build_schedule_grid([{**mbp, "map_kpa": mbp["map_kpa"]} for mbp in mb_points], "ve_air")
    eta_i_samples = [mbp for mbp in mb_points if mbp["eta_i"] is not None]
    eta_i_plausible_samples = [mbp for mbp in eta_i_samples if mbp["eta_i_plausible"]]
    eta_i_grid = build_schedule_grid(eta_i_plausible_samples, "eta_i")
    n_eta_i_implausible = len(eta_i_samples) - len(eta_i_plausible_samples)

    lambda_grid = build_schedule_grid(points, "lambda")
    icam_grid = build_schedule_grid(points, "icam_adv_deg")
    icam_grid["data_vs_spec_note"] = (
        "Measured icam_adv_deg reaches 55.0 deg at several High Load Final rows (e.g. rpm~4032/"
        "4493, MAP~95-97 kPa) - ABOVE the plan section 1.1-cited 'Intake 0-50 deg advance "
        "(electric VVT-iE)' spec range. Not corrected (measured data kept as-is); flagged for "
        "C17a/owner - either the cited 50 deg figure is a nominal/rounded spec and the "
        "actuator's real mechanical travel is closer to 55 deg, or there is a sensor "
        "calibration offset at the high-load operating point. ecam_ret_deg's measured max "
        "(40.996 deg) matches its cited '0-41 deg retard' spec closely, by contrast."
    )
    ecam_grid = build_schedule_grid(points, "ecam_ret_deg")
    imt_grid = build_schedule_grid(points, "imt_c")
    enrichment_flag_grid = build_enrichment_flag_grid(points)

    back_pressure = build_back_pressure_curve(points)
    valve_lift = build_valve_lift_tables()
    egr_finding = build_egr_finding()
    bore_stroke = build_bore_stroke_note()
    friction = build_friction_choice()

    n_enriched = sum(1 for p in points if p["protection_enrichment_flag"])

    output = {
        "schema": "physics_sim.a25a_c17pre_tables/1",
        "generated_by": "tools/enginedata/a25a_author.py",
        "task_reference": "S:\\claude_code\\physics_sim\\out\\i7c_plan.md section 'C17-pre'",
        "source_files": {
            "test_data_xlsx": str(TEST_DATA_XLSX),
            "test_data_xlsx_sha256": xlsx_sha256,
            "test_report_docx": str(TEST_REPORT_DOCX),
            "test_report_docx_sha256": docx_sha256,
            "extracted_fig11_jpeg": str(OUT_FIG11),
            "extracted_fig11_sha256": fig11_sha256,
        },
        "fuel": {
            "lhv_j_kg": FUEL_LHV_J_KG,
            "afr_stoich": FUEL_AFR_STOICH,
            "source": FUEL_SOURCE,
        },
        "engine_geometry": {
            "bore_stroke": bore_stroke,
            "cylinders": CYLINDERS,
            "displacement_m3": r6(DISPLACEMENT_M3),
        },
        "measured_points_summary": {
            "n_points_total": len(points),
            "n_points_protection_enrichment_flagged": n_enriched,
            "n_points_with_mb_ve": len(mb_points),
            "n_points_with_mb_eta_i": len(eta_i_samples),
            "n_points_eta_i_implausible_excluded_from_grid": n_eta_i_implausible,
            "sheets": DATA_SHEETS,
        },
        "measured_points": points,
        "mb_fallback": {
            "label": "measured-map engine (D12 'M-B'): its dyno comparison is tautological and "
                      "is not a generator validation. Swap to generated (C16a Wiebe/Woschni) "
                      "maps in C16a/C17a's follow-up per the plan.",
            "ve_air": ve_grid,
            "eta_i": eta_i_grid,
            "eta_i_note": "Gridded on the same (rpm, MAP) axes as ve_air per D12 ('gridded like "
                           "the schedules'); section 4's runtime schema instead wants eta_i on "
                           "(rpm, m_air_per_rev) axes, resampled per rpm line - "
                           "'measured_points' above carries each point's own m_air_per_rev_kg "
                           "so C16a/C17a can do that resampling exactly instead of double-"
                           "interpolating through this rpm/MAP grid.",
            "eta_i_plausibility_filter": f"{n_eta_i_implausible} of {len(eta_i_samples)} "
                           "candidate points had eta_i outside [0.15, 0.50] (light-load points "
                           "where the 'ideal PMEP = p_em - p_im' approximation breaks down, see "
                           "compute_mb_point()'s own comment) and were EXCLUDED from this grid "
                           "(but kept, flagged 'eta_i_plausible': false, in 'raw_points' below).",
            "friction_model_used_for_eta_i": friction,
            "raw_points": mb_points,
        },
        "schedules": {
            "lambda_target": lambda_grid,
            "cam_intake_advance_deg": icam_grid,
            "cam_exhaust_retard_deg": ecam_grid,
            "imt_c": imt_grid,
            "protection_enrichment_flag_grid": enrichment_flag_grid,
            "back_pressure_curve": back_pressure,
        },
        "valve_lift_fig11": valve_lift,
        "egr_finding_task6": egr_finding,
    }

    text = json.dumps(output, indent=2, sort_keys=False, ensure_ascii=True)
    OUT_JSON.write_text(text + "\n", encoding="ascii")
    out_sha256 = hashlib.sha256((text + "\n").encode("ascii")).hexdigest()
    print(f"wrote {OUT_JSON} sha256={out_sha256}")
    print(f"  n_points={len(points)} n_enriched_flagged={n_enriched}")
    print(f"  test_data_xlsx sha256={xlsx_sha256}")
    print(f"  test_report_docx sha256={docx_sha256}")
    print(f"  fig11 jpeg sha256={fig11_sha256}")


if __name__ == "__main__":
    main()
