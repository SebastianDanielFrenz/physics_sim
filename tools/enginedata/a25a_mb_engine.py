#!/usr/bin/env python3
"""PHYS-033 P4 I7c C17a (fallback M-B): Toyota A25A-FKS simulated-engine data.

Writes, reproducibly (stdlib only, byte-identical on unchanged inputs):
    data/engines/toyota_a25a_fks_sim.json   "kind": "simulated", measured-derived maps
    data/vehicles/car_sedan_sim.json        car_sedan.json with that engine and a 350 N*m clutch
and prints (and writes to out/a25a_mb_report.txt) the comparison tables the
C17a report quotes. It extends tools/enginedata/ next to a25a_author.py
(C17-pre) and imports that script's xlsx reader and point extraction; it does
not rewrite it.

This is the I7c D12 "M-B" fallback: the VE / effective-eta_i maps are
back-solved from the EPA NVFEL dyno data, so a dyno comparison of this engine
is TAUTOLOGICAL (it reproduces its own input) and is NOT a validation of any
cycle-model generator (C16a).

Method (every step labelled in the output's "source" fields):
  1. Points: sheets "Steady State" + "High Load Final" (the "High Load
     Initial" repeat is used only for the repeatability figure), fired points
     only (fuel > 0.05 g/s).
  2. Friction F(rpm) [D]: the engine's OWN motoring rows (Steady State,
     fuel 0): F = -T_motoring - (max(p_em, p_cell) - MAP) * V_d/(4 pi), the
     measured brake torque minus the SAME ideal pumping term the runtime uses
     (tau_pump = -(p_amb - p + pmep_excess) V_d/(4 pi), with p_amb = cell
     pressure and pmep_excess = max(0, p_em - p_cell)), least-squares
     quadratic in rpm, tabulated 0..7000 rpm. The SAME tabulated F is used
     in the eta_i back-solve below, so runtime and derivation share one
     friction+pumping model (brief: "use the SAME friction and pumping model").
     Barnes-Moss (F1) and the Chevrolet Ecotec LCV motored-friction test (F4)
     are reported for comparison only.
  3. Per point [D]: m_air/rev = fuel * lambda * AFR_test / (rpm/60);
     VE = m_air * R * T_cell / (MAP * V_d/2) (the runtime's manifold density
     is p/(R T_ambient), so VE is referenced to the cell temperature);
     eta_eff = (T_b + F + (p_cell - MAP + pmep) V_d/(4 pi)) * 2 pi
               / (f_lambda(lambda) * LHV_rt * m_air / (lambda * AFR_rt)),
     with the RUNTIME fuel (tier3_e10_cert.json, the first in the engine's fuel
     list and the fuel the data were measured on) -
     so eta_eff is an "effective eta_i": it absorbs FMEP/PMEP model error and
     the lambda-sensor calibration, and the runtime reproduces the measured
     brake torque at the measured MAP by construction.
  4. Rows: points clustered by rpm (gap < 60 rpm); the High Load Final
     pedal-100 point is the row's WOT anchor (row rpm = its rpm); other points
     at or above the anchor's MAP - 0.5 kPa, or with more air than it, are
     dropped (a near-WOT duplicate would fold the table back); points within
     0.5 kPa MAP of each other are averaged.
  5. Gridding: per row, linear in MAP (VE, lambda, spark, pmep) or in
     air/rev (eta) inside the row's own data; outside it the shape of the
     rpm-nearest covering rows is transferred (ratio for VE/eta, offset for
     lambda/spark/pmep), above a WOT anchor the WOT value is held; with no
     covering neighbour the edge value is held.
  6. Monotonic caps, top-down per row, WOT knot never changed, rounding in
     the safe direction: VE*p and eta*m_air non-decreasing, lambda
     non-increasing in p, d(pmep)/dp <= 1. Together these make the steady
     torque non-decreasing in plate position for ANY ambient and for any
     bilinear rpm blend (SimulatedEngine::build_maps throws otherwise) -
     verified here by a Python replica of build_maps.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from decimal import ROUND_CEILING, ROUND_FLOOR, Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import a25a_author as A  # noqa: E402  (C17-pre: xlsx reader, point extraction, constants)

WT = Path(__file__).resolve().parents[2]
ENGINE_JSON = WT / "data" / "engines" / "toyota_a25a_fks_sim.json"
VEHICLE_JSON = WT / "data" / "vehicles" / "car_sedan_sim.json"
BASE_VEHICLE_JSON = WT / "data" / "vehicles" / "car_sedan.json"
REPORT_TXT = WT / "out" / "a25a_mb_report.txt"
FUEL_REFS = ["../fuels/tier3_e10_cert.json", "../fuels/e10.json"]  # first is burnt: the EPA Tier 3 E10 cert fuel the data were measured on
ECOTEC_XLSX = Path(r"S:\claude_code\physics_sim\out\sources\chevrolet_ecotec_lcv_25"
                   r"\4- 2013 Chevrolet 2.5L Ecotec LCV Engine Reg E10 Fuel - Test Data.xlsx")

R_AIR = 287.05          # SimulatedEngine::kGasConstant
GAMMA = 1.4             # SimulatedEngine::kGamma
ISA_P = 101325.0
ISA_T = 288.15
PSI_N = 1000            # SimulatedEngine::kPsiIntervals
PSI_REG = 0.99          # SimulatedEngine::kPsiRegularisationStart
VD = A.DISPLACEMENT_M3  # m^3, bore 87.5 x stroke 103.4 x 4
K_PUMP = VD * 0.5 / (2.0 * math.pi)  # N*m per Pa of mean effective pressure (4-stroke: V_d/(4 pi))
FIRED_MIN_G_S = 0.05
ROW_GAP_RPM = 60.0
WOT_DROP_KPA = 0.5
MERGE_KPA = 0.5
ETA_PLAUSIBLE = (0.15, 0.50)  # C7's plausibility band (brief: compare filtered vs unfiltered)

# [AS] model curves (no measured source; see "source" fields)
F_LAMBDA = {"x": [0.5, 1.0, 1.5], "y": [0.5, 1.0, 1.0]}
ETA_SPARK = {"x": [-40.0, -20.0, -8.0, 0.0, 8.0], "y": [0.55, 0.82, 0.965, 1.0, 0.965]}
CD = 0.8
INTAKE_VOLUME_L = 3.0
ACTUATOR_TAU_S = 0.05
IDLE = {"kp_per_rpm": 3e-5, "ki_per_rpm_s": 6e-5, "max_throttle": 0.1}
IDLE_RPM = 800.0
LEAK_FACTOR = 0.5  # plate-0 leak = this x the leak that gives zero net torque at idle_rpm (ISA)
LIMITER = {"type": "fuel_cut", "rpm": 6500, "hysteresis_rpm": 150}

SPARK_MIN, SPARK_MAX = -30.0, 70.0  # the loader's ecu.spark_advance_deg range
P_AXIS_REGULAR = [20.0 + 5.0 * i for i in range(16)]            # 20..95 kPa
M_AXIS_REGULAR = [round(0.1 * i, 1) for i in range(1, 16)]      # 0.1..1.5 g/rev

OUT_LINES: list[str] = []


def say(s: str = "") -> None:
    print(s)
    OUT_LINES.append(s)


r6 = A.r6


def r6_dir(x: float, up: bool) -> float:
    if x == 0 or not math.isfinite(x):
        return x
    d = Decimal(x)
    q = Decimal(1).scaleb(d.adjusted() - 5)
    return float(d.quantize(q, rounding=ROUND_CEILING if up else ROUND_FLOOR))


# ---------------------------------------------------------------------------
# piecewise-linear helpers (Curve1D / Table2D semantics: clamped at every edge)
# ---------------------------------------------------------------------------
def seg(xs: list[float], x: float) -> tuple[int, float]:
    n = len(xs)
    if x <= xs[0]:
        return 0, 0.0
    if x >= xs[-1]:
        return n - 2, 1.0
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    return lo, (x - xs[lo]) / (xs[lo + 1] - xs[lo])


def curve(c: dict, x: float) -> float:
    i, t = seg(c["x"], x)
    return c["y"][i] + t * (c["y"][i + 1] - c["y"][i])


def table(tb: dict, x: float, y: float) -> float:
    i, tx = seg(tb["x"], x)
    j, ty = seg(tb["y"], y)
    v = tb["v"]
    a = v[i][j] + ty * (v[i][j + 1] - v[i][j])
    b = v[i + 1][j] + ty * (v[i + 1][j + 1] - v[i + 1][j])
    return a + tx * (b - a)


def lin(xs: list[float], ys: list[float], x: float) -> float:
    """Linear interpolation inside [xs[0], xs[-1]] (caller guarantees)."""
    if len(xs) == 1:
        return ys[0]
    i, t = seg(xs, x)
    return ys[i] + t * (ys[i + 1] - ys[i])


def fit_quadratic(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    s = [sum(x ** k for x in xs) for k in range(5)]
    t = [sum(y * x ** k for x, y in zip(xs, ys)) for k in range(3)]
    m = [[s[0], s[1], s[2], t[0]], [s[1], s[2], s[3], t[1]], [s[2], s[3], s[4], t[2]]]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [a - f * b for a, b in zip(m[r], m[c])]
    return tuple(m[i][3] / m[i][i] for i in range(3))  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# data
# ---------------------------------------------------------------------------
def load_points() -> list[dict]:
    sh = A.load_xlsx(A.TEST_DATA_XLSX)
    pts = A.extract_points(sh)
    for p in pts:
        row = sh[p["sheet"]][p["row"]]
        p["p_cell_kpa"] = row.get(A.COL["cell_press_kpa"])
        p["t_cell_c"] = row.get(A.COL["cell_temp_c"])
    return pts


def is_fired(p: dict) -> bool:
    return isinstance(p["fuel_g_s"], float) and p["fuel_g_s"] > FIRED_MIN_G_S


def usable(p: dict) -> bool:
    keys = ("torque_nm", "map_kpa", "lambda", "spark_btdc", "p_cell_kpa", "t_cell_c", "egr_inlet_press_kpa")
    return all(isinstance(p[k], float) for k in keys)


def load_fuel(ref: str) -> dict:
    return json.loads((ENGINE_JSON.parent / ref).resolve().read_text())


# ---------------------------------------------------------------------------
# friction models
# ---------------------------------------------------------------------------
def friction_f5(pts: list[dict]) -> tuple[dict, list[dict], tuple[float, float, float]]:
    mot = [p for p in pts if p["sheet"] == "Steady State" and not is_fired(p)
           and isinstance(p["torque_nm"], float) and p["torque_nm"] < 0.0
           and all(isinstance(p[k], float) for k in ("map_kpa", "p_cell_kpa", "egr_inlet_press_kpa"))]
    rows = []
    for p in mot:
        p_ex = max(p["egr_inlet_press_kpa"], p["p_cell_kpa"])
        f = -p["torque_nm"] - (p_ex - p["map_kpa"]) * 1000.0 * K_PUMP
        rows.append({"row": p["row"], "rpm": p["rpm"], "t": p["torque_nm"], "map": p["map_kpa"],
                     "p_em": p["egr_inlet_press_kpa"], "p_cell": p["p_cell_kpa"], "f": f})
    rows.sort(key=lambda r: r["rpm"])
    a, b, c = fit_quadratic([r["rpm"] / 1000.0 for r in rows], [r["f"] for r in rows])
    xs = [500.0 * i for i in range(15)]
    tab = {"x": xs, "y": [r6(a + b * x / 1000.0 + c * (x / 1000.0) ** 2) for x in xs]}
    return tab, rows, (a, b, c)


def friction_f1(rpm: float) -> float:
    return A.barnes_moss_fmep_bar(rpm) * 1e5 * K_PUMP


def friction_f4() -> tuple[callable, list[tuple[float, float]]]:
    sh = A.load_xlsx(ECOTEC_XLSX)["Motored Friction"]
    data = [(sh[r][0], -sh[r][2]) for r in range(6, 11)]  # rpm, FMEP bar (= -BMEP motored)
    a, b, c = fit_quadratic([d[0] / 1000.0 for d in data], [d[1] for d in data])
    return (lambda rpm: (a + b * rpm / 1000.0 + c * (rpm / 1000.0) ** 2) * 1e5 * K_PUMP), data


# ---------------------------------------------------------------------------
# per-point derivation
# ---------------------------------------------------------------------------
def air_per_rev(p: dict) -> float:
    return p["fuel_g_s"] * 1e-3 / (p["rpm"] / 60.0) * p["lambda"] * A.FUEL_AFR_STOICH


def derive_ve(p: dict) -> float:
    return air_per_rev(p) * R_AIR * (p["t_cell_c"] + 273.15) / (p["map_kpa"] * 1000.0 * VD * 0.5)


def pmep_kpa(p: dict) -> float:
    return max(0.0, p["egr_inlet_press_kpa"] - p["p_cell_kpa"])


def derive_eta(p: dict, fric, fuel: dict, lam: float) -> float:
    comb = p["torque_nm"] + fric(p["rpm"]) + (p["p_cell_kpa"] - p["map_kpa"] + pmep_kpa(p)) * 1000.0 * K_PUMP
    m_fuel = air_per_rev(p) / (lam * fuel["afr_stoich"])
    return comb * 2.0 * math.pi / (curve(F_LAMBDA, lam) * fuel["lhv_j_kg"] * m_fuel)


# ---------------------------------------------------------------------------
# rows, gridding, caps
# ---------------------------------------------------------------------------
def wot_anchor(cl: list[dict]) -> dict | None:
    """The speed line's WOT anchor: its MAXIMUM measured brake torque over any
    pedal position (C7's B1 envelope, data/engines/toyota_a25a_fks.json), but
    only on a line that had a High Load Final sweep (the 807 rpm line has
    none; its maximum is a pedal-25 part-load point). This excludes the
    lambda 0.63-0.73 ECU protective-enrichment pedal-80/100 rows at 1326/1498 rpm
    (7-16 N*m below the pedal-70 row, at ~2 kPa higher MAP) exactly as
    C7 did."""
    if not any(p["sheet"] == "High Load Final" for p in cl):
        return None
    return max(cl, key=lambda p: (p["torque_nm"], -p["fuel_g_s"], p["sheet"], p["row"]))


def make_rows(fired: list[dict]) -> tuple[list[dict], list[dict]]:
    srt = sorted(fired, key=lambda p: (p["rpm"], p["sheet"], p["row"]))
    clusters: list[list[dict]] = []
    for p in srt:
        if clusters and p["rpm"] - clusters[-1][-1]["rpm"] < ROW_GAP_RPM:
            clusters[-1].append(p)
        else:
            clusters.append([p])
    rows, dropped = [], []
    for cl in clusters:
        wot = wot_anchor(cl)
        if wot is not None:
            rpm = r6(wot["rpm"])
            keep = [wot]
            for p in cl:
                if p is wot:
                    continue
                if p["map_kpa"] >= wot["map_kpa"] - WOT_DROP_KPA or air_per_rev(p) > air_per_rev(wot):
                    dropped.append(p)
                else:
                    keep.append(p)
        else:
            rs = sorted(p["rpm"] for p in cl)
            n = len(rs)
            rpm = r6(rs[n // 2] if n % 2 else 0.5 * (rs[n // 2 - 1] + rs[n // 2]))
            keep = list(cl)
        rows.append({"rpm": rpm, "wot": wot, "pts": keep})
    return rows, dropped


def merge_row_points(row: dict, fric, fuel: dict, lam_tab: dict | None) -> list[dict]:
    """Derived per-point quantities, merged within MERGE_KPA of MAP (the WOT
    anchor is never merged)."""
    der = []
    for p in row["pts"]:
        lam = p["lambda"] if lam_tab is None else table(lam_tab, row["rpm"], p["map_kpa"])
        der.append({"map": p["map_kpa"], "m": air_per_rev(p) * 1e3, "ve": derive_ve(p), "lam": p["lambda"],
                    "spark": p["spark_btdc"], "pmep": pmep_kpa(p), "eta": derive_eta(p, fric, fuel, lam),
                    "wot": p is row["wot"], "src": [(p["sheet"], p["row"])]})
    der.sort(key=lambda d: d["map"])
    out: list[dict] = []
    for d in der:
        if out and not d["wot"] and not out[-1]["wot"] and d["map"] - out[-1]["map0"] < MERGE_KPA:
            g = out[-1]
            g["_n"] += 1
            for k in ("map", "m", "ve", "lam", "spark", "pmep", "eta"):
                g["_s"][k] += d[k]
                g[k] = g["_s"][k] / g["_n"]
            g["src"] += d["src"]
        else:
            g = dict(d)
            g["map0"] = d["map"]
            g["_n"] = 1
            g["_s"] = {k: d[k] for k in ("map", "m", "ve", "lam", "spark", "pmep", "eta")}
            out.append(g)
    return out


def row_function(merged: list[dict], xkey: str, ykey: str) -> tuple[list[float], list[float]]:
    pairs = sorted((d[xkey], d[ykey]) for d in merged)
    xs, ys = [], []
    for x, y in pairs:
        if xs and x == xs[-1]:
            ys[-1] = 0.5 * (ys[-1] + y)
        else:
            xs.append(x)
            ys.append(y)
    return xs, ys


def grid_quantity(rows: list[dict], funcs: list[tuple[list[float], list[float]]], axis: list[float],
                  ratio: bool) -> list[list[float]]:
    """Grid every row's function onto `axis` (method docstring step 5)."""
    out = []
    for ir, row in enumerate(rows):
        xs, ys = funcs[ir]
        lo, hi = xs[0], xs[-1]
        vals = []
        for q in axis:
            if lo <= q <= hi:
                vals.append(lin(xs, ys, q))
                continue
            edge = lo if q < lo else hi
            own = lin(xs, ys, edge)
            if q > hi and row["wot"] is not None:
                vals.append(own)
                continue
            a, b = min(q, edge), max(q, edge)

            def covers(j: int) -> bool:
                return funcs[j][0][0] <= a and funcs[j][0][-1] >= b

            lower = next((j for j in range(ir - 1, -1, -1) if covers(j)), None)
            upper = next((j for j in range(ir + 1, len(rows)) if covers(j)), None)
            if lower is None and upper is None:
                vals.append(own)
                continue

            def nb(x: float) -> float:
                if lower is not None and upper is not None:
                    w = (row["rpm"] - rows[lower]["rpm"]) / (rows[upper]["rpm"] - rows[lower]["rpm"])
                    return (1 - w) * lin(*funcs[lower], x) + w * lin(*funcs[upper], x)
                j = lower if lower is not None else upper
                return lin(*funcs[j], x)

            vals.append(own * nb(q) / nb(edge) if ratio else own + nb(q) - nb(edge))
        out.append(vals)
    return out


def cap_product(vals: list[float], axis: list[float]) -> list[float]:
    """Round, then top-down: v_k <= v_{k+1} * (2 - x_k/x_{k+1}) so v(x)*x is
    non-decreasing on every segment (rounded DOWN when capped)."""
    v = [r6(x) for x in vals]
    for k in range(len(v) - 2, -1, -1):
        cap = v[k + 1] * (2.0 - axis[k] / axis[k + 1])
        if v[k] > cap:
            v[k] = r6_dir(cap, up=False)
    return v


def cap_nonincreasing(vals: list[float]) -> list[float]:
    v = [r6(x) for x in vals]
    for k in range(len(v) - 2, -1, -1):
        v[k] = max(v[k], v[k + 1])
    return v


def cap_slope_le_1(vals: list[float], axis: list[float]) -> list[float]:
    v = [max(0.0, r6(x)) for x in vals]
    for k in range(len(v) - 2, -1, -1):
        floor_ = v[k + 1] - (axis[k + 1] - axis[k])
        if v[k] < floor_:
            v[k] = r6_dir(floor_, up=True)
    return v


def build_axes(rows: list[dict]) -> tuple[list[float], list[float]]:
    p_axis = sorted(set(P_AXIS_REGULAR) | {r6(r["wot"]["map_kpa"]) for r in rows if r["wot"]})
    m_axis = sorted(set(M_AXIS_REGULAR) | {r6(air_per_rev(r["wot"]) * 1e3) for r in rows if r["wot"]})
    return p_axis, m_axis


def build_tables(rows: list[dict], fric, fuel: dict, eta_filter: bool) -> dict:
    p_axis, m_axis = build_axes(rows)
    rpm_axis = [r["rpm"] for r in rows]
    if any(b <= a for a, b in zip(rpm_axis, rpm_axis[1:])):
        raise RuntimeError(f"row rpms not strictly ascending: {rpm_axis}")
    # pass 1: lambda / VE / spark / pmep over MAP
    m1 = [merge_row_points(r, fric, fuel, None) for r in rows]
    lam = [cap_nonincreasing(v) for v in grid_quantity(rows, [row_function(m, "map", "lam") for m in m1], p_axis, False)]
    lam = [[min(2.0, max(0.5, x)) for x in v] for v in lam]
    ve = [cap_product(v, p_axis) for v in grid_quantity(rows, [row_function(m, "map", "ve") for m in m1], p_axis, True)]
    spark_raw = grid_quantity(rows, [row_function(m, "map", "spark") for m in m1], p_axis, False)
    # linear-in-MAP extrapolation below a row's lowest measured MAP overshoots (to -33.5 deg, outside the
    # loader's [-30, 70]); clamp every knot to the MEASURED spark range of the points the rows hold
    meas = [p["spark_btdc"] for r in rows for p in r["pts"]]
    lo_s, hi_s = max(SPARK_MIN, min(meas)), min(SPARK_MAX, max(meas))
    spark = [[r6(min(hi_s, max(lo_s, x))) for x in v] for v in spark_raw]
    flat = [x for v in spark_raw for x in v]
    say(f"spark grid: raw range {min(flat):.2f} .. {max(flat):.2f} deg, "
        f"{sum(1 for x in flat if not lo_s <= x <= hi_s)} knots clamped to the measured range [{lo_s:.3f}, {hi_s:.3f}]")
    pmep = [cap_slope_le_1(v, p_axis) for v in grid_quantity(rows, [row_function(m, "map", "pmep") for m in m1], p_axis, False)]
    lam_tab = {"x": rpm_axis, "y": p_axis, "v": lam}
    # pass 2: eta with the TABLE lambda at the row rpm and the point's MAP
    m2 = [merge_row_points(r, fric, fuel, lam_tab) for r in rows]
    dropped_eta = []
    if eta_filter:
        for i, m in enumerate(m2):
            keep = [d for d in m if d["wot"] or ETA_PLAUSIBLE[0] <= d["eta"] <= ETA_PLAUSIBLE[1]]
            dropped_eta += [(rows[i]["rpm"], d) for d in m if d not in keep]
            m2[i] = keep
    eta = [cap_product(v, m_axis) for v in grid_quantity(rows, [row_function(m, "m", "eta") for m in m2], m_axis, True)]
    return {"rpm": rpm_axis, "p": p_axis, "m": m_axis, "lam": lam, "ve": ve, "spark": spark, "pmep": pmep,
            "eta": eta, "merged": m2, "dropped_eta": dropped_eta}


# ---------------------------------------------------------------------------
# Python replica of the runtime (engine_simulated.cpp evaluate_torque_impl,
# manifold_rhs_impl, steady_p, build_maps' checks) - REPORTING ONLY; the C++
# unit test is the authority.
# ---------------------------------------------------------------------------
def psi_true(pr: float) -> float:
    g = GAMMA
    crit = (2.0 / (g + 1.0)) ** (g / (g - 1.0))
    if pr <= crit:
        return math.sqrt(g * (2.0 / (g + 1.0)) ** ((g + 1.0) / (g - 1.0)))
    return math.sqrt(2.0 * g / (g - 1.0) * max(0.0, pr ** (2.0 / g) - pr ** ((g + 1.0) / g)))


I_REG = int(PSI_REG * PSI_N + 0.5)
PSI_TAB = [psi_true(i / PSI_N) if i < I_REG else psi_true(I_REG / PSI_N) * (PSI_N - i) / (PSI_N - I_REG)
           for i in range(PSI_N + 1)]


def psi(pr: float) -> float:
    prc = min(1.0, max(0.0, pr))
    u = prc * PSI_N
    i = min(int(u), PSI_N - 1)
    return PSI_TAB[i] + (u - i) * (PSI_TAB[i + 1] - PSI_TAB[i])


class Replica:
    def __init__(self, eng: dict, fuel: dict):
        mp, ecu = eng["maps"], eng["ecu"]

        def t2(tb: dict, yk: str, ys: float = 1.0, vs: float = 1.0) -> dict:
            return {"x": tb["rpm"], "y": [y * ys for y in tb[yk]], "v": [[v * vs for v in r] for r in tb["values"]]}

        self.ve = t2(mp["ve"], "p_im_kpa", 1000.0)
        self.eta = t2(mp["eta_i"], "air_mass_per_rev_g", 1e-3)
        self.lam = t2(ecu["lambda_target"], "p_im_kpa", 1000.0)
        self.spark = t2(ecu["spark_advance_deg"], "p_im_kpa", 1000.0)
        self.mbt = t2(mp["theta_mbt_deg"], "p_im_kpa", 1000.0)
        self.pmep = t2(mp["pmep_excess_kpa"], "p_im_kpa", 1000.0, 1000.0)
        self.fl, self.es, self.fric = mp["f_lambda"], mp["eta_spark"], mp["friction_torque_nm"]
        self.c_k = curve(self.fric, 0.0)
        self.lhv, self.afr = fuel["lhv_j_kg"], fuel["afr_stoich"]
        self.vd = eng["cylinders"] * math.pi / 4.0 * (eng["bore_mm"] * 1e-3) ** 2 * eng["stroke_mm"] * 1e-3
        tb = eng["throttle_body"]
        self.a_full = tb["count"] * math.pi / 4.0 * (tb["diameter_mm"] * 1e-3) ** 2
        self.cd = tb["discharge_coefficient"]
        self.af = tb["area_fraction_vs_position"]
        self.v_im = eng["intake_manifold_volume_l"] * 1e-3

    def net(self, rpm: float, p: float, p_amb: float, t_amb: float, fired: bool = True) -> dict:
        """evaluate_torque(...).tau - c_k, omega > 0, fire 1 (limiter-free), no starter."""
        rt = R_AIR * t_amb
        m_a = table(self.ve, rpm, p) * p / rt * self.vd * 0.5
        comb = 0.0
        lam = table(self.lam, rpm, p)
        if fired:
            g = table(self.eta, rpm, m_a) * curve(self.fl, lam) * curve(
                self.es, table(self.spark, rpm, p) - table(self.mbt, rpm, p))
            comb = g * self.lhv / (2.0 * math.pi) * m_a / (lam * self.afr)
        pump = -(p_amb - p + table(self.pmep, rpm, p)) * self.vd * 0.5 / (2.0 * math.pi)
        return {"t": comb + pump - curve(self.fric, rpm), "m_a": m_a, "lam": lam}

    def f(self, rpm: float, p: float, x: float, p_amb: float, t_amb: float) -> float:
        rt = R_AIR * t_amb
        a_eff = self.cd * self.a_full * curve(self.af, x)
        k = a_eff / math.sqrt(rt)
        m_thr = k * p_amb * psi(p / p_amb) if p <= p_amb else -k * p * psi(p_amb / p)
        m_cyl = table(self.ve, rpm, p) * p / rt * self.vd * 0.5 * rpm / 60.0
        return m_thr - m_cyl

    def steady_p(self, rpm: float, x: float, p_amb: float, t_amb: float) -> float:
        lo, hi = 1e-3 * p_amb, p_amb
        if self.f(rpm, lo, x, p_amb, t_amb) <= 0:
            return lo
        if self.f(rpm, hi, x, p_amb, t_amb) >= 0:
            return hi
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if mid <= lo or mid >= hi:
                break
            if self.f(rpm, mid, x, p_amb, t_amb) > 0:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def steady(self, rpm: float, x: float, p_amb: float = ISA_P, t_amb: float = ISA_T, fired: bool = True) -> tuple[float, float]:
        p = self.steady_p(rpm, x, p_amb, t_amb)
        return p, self.net(rpm, p, p_amb, t_amb, fired)["t"]

    def check_monotone(self, limiter_rpm: float) -> tuple[bool, float]:
        worst = float("inf")
        r = 500.0
        while r <= limiter_rpm + 250.0 + 1e-9:
            prev = None
            for i in range(41):
                t = self.steady(r, i / 40.0)[1]
                if prev is not None:
                    worst = min(worst, t - prev)
                prev = t
            r += 250.0
        return worst >= -1e-9, worst

    def idle_ff(self, idle_rpm: float, max_x: float) -> tuple[float, float]:
        net = lambda x: self.steady(idle_rpm, x)[1]  # noqa: E731
        if net(0.0) >= 0.0:
            ff = 0.0
        elif net(max_x) <= 0.0:
            ff = max_x
        else:
            lo, hi = 0.0, max_x
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if net(mid) < 0.0:
                    lo = mid
                else:
                    hi = mid
            ff = 0.5 * (lo + hi)
        return ff, self.steady_p(idle_rpm, ff, ISA_P, ISA_T)


# ---------------------------------------------------------------------------
# JSON assembly
# ---------------------------------------------------------------------------
def tab_json(rpm: list[float], yk: str, y: list[float], vals: list[list[float]], source: str) -> dict:
    return {"source": source, "rpm": rpm, yk: y, "values": vals}


def engine_json(tb: dict, fric_tab: dict, throttle: dict, idle_note: str, fuels: dict,
                sabotage: str | None) -> dict:
    ve_vals = tb["ve"]
    p_axis = tb["p"]
    if sabotage == "transpose_ve":
        ve_vals = [list(c) for c in zip(*ve_vals)]
    elif sabotage == "reverse_ve_rows":
        ve_vals = list(reversed(ve_vals))
    elif sabotage == "kpa_as_pa":
        p_axis = [p * 1000.0 for p in p_axis]
    pmep_label = ("[D] measured exhaust-manifold (EGR-inlet) pressure minus measured cell pressure, "
                  "max(0, p_em - p_cell), per fired point, gridded like ve; capped to slope <= 1 in p")
    return {
        "format": "physics_sim.engine/1",
        "name": "toyota_a25a_fks_sim",
        "kind": "simulated",
        "source": (
            "measured-map engine (I7c D12 M-B): its dyno comparison is tautological and is not a generator "
            "validation. Toyota A25A-FKS 2.5 L I4 (2018 Camry). Maps back-solved from the US EPA NVFEL "
            "benchmark 'Engine Tier 3 Fuel - Test Data' package (sheets Steady State + High Load Final; "
            "measured = a literal xlsx reading, [D] derived with the method stated, [AS] assumed, [C] cited) by "
            "tools/enginedata/a25a_mb_engine.py (PHYS-033 P4 I7c C17a). Every WOT brake torque is reproduced at "
            "its measured MAP and cell ambient (tests/unit/test_a25a_sim_engine_data.cpp). Burns "
            "tier3_e10_cert.json (the fuel the data were measured on); e10.json selectable. WOT anchor per speed line = "
            "its maximum measured brake torque (C7's envelope, data/engines/toyota_a25a_fks.json), which excludes the "
            "measured lambda 0.73 / 0.63 ECU protective-enrichment pedal-80/100 rows at 1326/1498 rpm (7-16 N*m below "
            "the lambda 0.88 pedal-70 rows there); the 1003-2535 rpm anchors are themselves enriched (measured lambda "
            "0.67-0.88) and "
            "stay in the lambda_target table as measured. Known limitation: the (rpm, MAP) tables cannot carry the "
            "measured cam-phase dependence - Steady State points just below WOT MAP (intake cam 25-37 deg adv) make "
            "30-60 N*m less than the model, which follows the High Load Final points (intake cam 55 deg); see "
            "out/a25a_mb_report.txt for the per-point residuals."),
        "architecture": "piston_4stroke",
        "bore_mm": 87.5,
        "stroke_mm": 103.4,
        "cylinders": 4,
        "geometry_source": ("bore 87.5 mm, stroke 103.4 mm, compression ratio 13.0 (not used by this model): "
                            "secondary web sources (Toyota press material as quoted by third-party sites; see "
                            "out/a25a/a25a_tables.json bore_stroke_note), not a primary citation"),
        "intake_manifold_volume_l": INTAKE_VOLUME_L,
        "intake_manifold_volume_source": "[AS] assumed 3.0 L plenum+runners (no measurement); sets only the manifold filling time constant",
        "throttle_body": throttle,
        "throttle_input": "electronic",
        "throttle_actuator_tau_s": ACTUATOR_TAU_S,
        "throttle_actuator_source": "[AS] assumed 50 ms first-order electronic throttle actuator (as the C11 test fixture)",
        "ecu": {
            "lambda_target": tab_json(tb["rpm"], "p_im_kpa", p_axis, tb["lam"],
                                      "measured exhaust lambda (wideband) per fired point, gridded [D]; protective-enrichment "
                                      "WOT points kept; capped non-increasing in p (monotonic torque); clamped to [0.5, 2]"),
            "spark_advance_deg": tab_json(tb["rpm"], "p_im_kpa", p_axis, tb["spark"],
                                          "measured spark timing, CAD BTDC, per fired point, gridded [D]; knots the gridding "
                                          "extrapolates outside the measured spark range are clamped to it (no torque "
                                          "effect: theta_mbt_deg is the same table). Note (flagged, "
                                          "not corrected): the measured intake cam advance reaches ~55 deg at some points; "
                                          "its effect is inside ve/eta_i, the model has no cam input"),
        },
        "maps": {
            "source": ("I7c D12 M-B override maps, [D] derived from measured data (method: tools/enginedata/"
                       "a25a_mb_engine.py docstring). Rows = measured rpm clusters (row rpm = the High Load Final "
                       "WOT point's rpm where one exists, else the cluster median); p axis = 20..95 kPa every 5 kPa "
                       "plus every WOT MAP; air axis = 0.1..1.5 g/rev every 0.1 plus every WOT air mass."),
            "ve": tab_json(tb["rpm"], "p_im_kpa", p_axis, ve_vals,
                           "[D] VE = m_air*R*T_cell/(MAP*V_d/2), m_air = measured fuel flow * measured lambda * "
                           "AFR_stoich(test fuel 14.0022, EPA NVFEL fuel analysis via C7) / rev; referenced to the "
                           "measured cell temperature (the runtime's manifold density is p/(R*T_ambient)); capped so "
                           "VE*p is non-decreasing in p"),
            "eta_i": tab_json(tb["rpm"], "air_mass_per_rev_g", tb["m"], tb["eta"],
                              "[D] effective eta_i (absorbs FMEP/PMEP model error): back-solved per fired point so "
                              "that tau_comb = T_brake + F(rpm) + (p_cell - MAP + pmep_excess)*V_d/(4 pi) with F = "
                              "friction_torque_nm below and the runtime fuel (first 'fuels' entry, tier3_e10_cert.json) and "
                              "lambda_target table; capped so eta*m_air is non-decreasing"),
            "f_lambda": dict(F_LAMBDA, source="[AS] oxygen-limited idealisation f = min(lambda, 1) (rich: fuel beyond "
                                              "stoichiometric does no work). Divides out of eta_i's back-solve, so it "
                                              "only matters away from the lambda_target table (never, at present)"),
            "eta_spark": dict(ETA_SPARK, source="[AS] curve shape of the C11 test fixture (tests/support/mvem_fixtures.h); "
                                                "inactive here because theta_mbt_deg = spark_advance_deg"),
            "theta_mbt_deg": tab_json(tb["rpm"], "p_im_kpa", p_axis, tb["spark"],
                                      "[AS] MBT taken equal to the measured spark table (eta_spark = 1 everywhere): the "
                                      "data carry no spark sweeps, and any knock/MBT offset is already inside eta_i"),
            "friction_torque_nm": dict(fric_tab, source=(
                "[D] from THIS engine's own motoring rows (Steady State sheet, fuel 0, 10 points 1326-4954 rpm): "
                "F = -T_motoring - (max(p_em, p_cell) - MAP)*V_d/(4 pi) (the runtime's own pumping term removed), "
                "least-squares quadratic in rpm, tabulated; F(0) is the Coulomb part. Used identically in the "
                "eta_i back-solve")),
            "pmep_excess_kpa": tab_json(tb["rpm"], "p_im_kpa", p_axis, tb["pmep"], pmep_label),
        },
        "inertia_kgm2": 0.16,
        "breakaway_torque_nm": 45.0,
        "idle_rpm": IDLE_RPM,
        "stall_rpm": 400,
        "fire_rpm": 250,
        "start_complete_rpm": 550,
        "dfco_enable_rpm": 1300,
        "dfco_reenable_rpm": 1000,
        "idle_control": dict(IDLE),
        "limiter": dict(LIMITER),
        "starter": {"stall_torque_nm": 140.0, "free_speed_rpm": 400.0, "interlock": "clutch_or_neutral"},
        "crank_axis_local": [0, 1, 0],
        "initial_state": "running",
        "fuels": list(FUEL_REFS),
        "ignition": "spark",
        "fields_source": (
            "[AS] inertia 0.16 kg*m^2, breakaway 45 N*m, stall/fire/start_complete 400/250/550 rpm, DFCO 1300/1000 rpm, "
            "starter 140 N*m/400 rpm: as data/engines/i4_2l_na.json (no A25A measurement). "
            + idle_note +
            " Idle PI kp 3e-5 /rpm, ki 6e-5 /(rpm*s), max plate 0.1: [AS] as the C11 test fixture. "
            "Limiter fuel_cut 6500 rpm / 150 rpm hysteresis: the EPA test-cell speed limit [C] (the highest measured point "
            "is 5761 rpm); 6600 rpm is Toyota's rated-power speed, not a cut - PROVISIONAL, owner may prefer 6600. "
            "Rated 151 kW @ 6600 rpm / 249 N*m @ 4800 rpm are Toyota ratings, not used. Fuels: tier3_e10_cert burnt "
            f"(LHV {fuels['burnt']['lhv_j_kg']:.0f} J/kg, AFR {fuels['burnt']['afr_stoich']}), e10 selectable."),
    }


def dump_compact(obj, indent: int = 0) -> str:
    """JSON with numeric lists (and lists of numeric lists' rows) on one line."""
    sp = "  " * indent
    if isinstance(obj, dict):
        items = [f'{sp}  {json.dumps(k)}: {dump_compact(v, indent + 1).lstrip()}' for k, v in obj.items()]
        return sp + "{\n" + ",\n".join(items) + "\n" + sp + "}"
    if isinstance(obj, list):
        if all(isinstance(x, (int, float)) for x in obj):
            return sp + "[" + ", ".join(json.dumps(x) for x in obj) + "]"
        if all(isinstance(x, str) for x in obj):
            return sp + json.dumps(obj)
        inner = ",\n".join(dump_compact(x, indent + 1) for x in obj)
        return sp + "[\n" + inner + "\n" + sp + "]"
    return sp + json.dumps(obj)


def vehicle_json() -> dict:
    base = json.loads(BASE_VEHICLE_JSON.read_text())
    out = {}
    for k, v in base.items():
        if k == "name":
            out[k] = "car_sedan_sim"
        elif k == "source":
            out[k] = ("data/vehicles/car_sedan.json with its engine swapped for engines/toyota_a25a_fks_sim.json (the "
                      "I7c M-B measured-map simulated Toyota A25A-FKS, PHYS-033 P4 I7c C17a) and the clutch raised to "
                      "350 N*m [AS] (the plan's value: the A25A's ~230 N*m measured / 249 N*m rated peak with margin, "
                      "static factor unchanged). Chassis, gearbox (sedan_6mt), final drive and tyres are car_sedan's "
                      "unchanged (not a Camry). Generated by tools/enginedata/a25a_mb_engine.py. Original car_sedan "
                      "source: " + v)
        elif k == "powertrain":
            pt = json.loads(json.dumps(v))
            for c in pt["components"]:
                if c.get("id") == "engine":
                    c["ref"] = "../engines/toyota_a25a_fks_sim.json"
                elif c.get("type") == "clutch":
                    c["capacity_nm"] = 350.0
            out[k] = pt
        else:
            out[k] = v
    return out


# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sabotage", choices=["transpose_ve", "reverse_ve_rows", "kpa_as_pa"],
                    help="write a deliberately broken engine file (test sabotage demo; re-run without it to revert)")
    ap.add_argument("--no-report", action="store_true")
    args = ap.parse_args()

    fuels = {"burnt": load_fuel(FUEL_REFS[0]), "alt": load_fuel(FUEL_REFS[1])}
    fuel = fuels["burnt"]
    pts = load_points()
    fric_tab, mot_rows, fq = friction_f5(pts)
    fric = lambda rpm: curve(fric_tab, rpm)  # noqa: E731
    f4, f4_data = friction_f4()

    say("== A25A M-B authoring report (tools/enginedata/a25a_mb_engine.py) ==")
    say(f"V_d = {VD * 1e3:.5f} L; runtime fuel tier3_e10_cert LHV {fuel['lhv_j_kg']:.0f} AFR {fuel['afr_stoich']}; "
        f"test fuel LHV {A.FUEL_LHV_J_KG:.0f} AFR {A.FUEL_AFR_STOICH:.6f}")
    say(f"runtime fuel flow / measured = AFR_test/AFR_runtime = {A.FUEL_AFR_STOICH / fuel['afr_stoich']:.5f}; "
        f"eta_eff(runtime)/eta(test fuel) = {(fuel['afr_stoich'] / fuel['lhv_j_kg']) / (A.FUEL_AFR_STOICH / A.FUEL_LHV_J_KG):.5f}")
    say("")
    say("-- friction F5 (A25A motoring rows): rpm  T_mot  MAP  p_em  p_cell  F_point  F_fit")
    for r in mot_rows:
        say(f"   {r['rpm']:7.1f} {r['t']:7.2f} {r['map']:6.2f} {r['p_em']:7.2f} {r['p_cell']:7.2f} {r['f']:7.3f} {fric(r['rpm']):7.3f}")
    say(f"   fit F = {fq[0]:.4f} + {fq[1]:.4f} n + {fq[2]:.4f} n^2 (n = rpm/1000); table {fric_tab['y']}")
    say("-- friction comparison N*m at rpm 1000/2000/3000/4000/5000/6000: F5 (runtime) / F1 Barnes-Moss / F4 Ecotec motored")
    for rpm in (1000, 2000, 3000, 4000, 5000, 6000):
        say(f"   {rpm:5d}: {fric(rpm):7.2f} {friction_f1(rpm):7.2f} {f4(rpm):7.2f}")
    say(f"   F4 data (rpm, FMEP bar): {[(round(a, 1), round(b, 4)) for a, b in f4_data]}")

    fired_all = [p for p in pts if is_fired(p) and usable(p)]
    fired = [p for p in fired_all if p["sheet"] in ("Steady State", "High Load Final")]
    initial = [p for p in fired_all if p["sheet"] == "High Load Initial"]
    say("")
    say(f"points: {len(pts)} total, {len(fired_all)} fired+usable, {len(fired)} in Steady State + High Load Final, "
        f"{len(initial)} High Load Initial (repeatability only)")

    # implausible eta counts per friction model (measured lambda, runtime fuel constants)
    for name, fn in (("F1 Barnes-Moss", friction_f1), ("F4 Ecotec", f4), ("F5 A25A motoring", fric)):
        etas = [derive_eta(p, fn, fuel, p["lambda"]) for p in fired]
        bad = [e for e in etas if not ETA_PLAUSIBLE[0] <= e <= ETA_PLAUSIBLE[1]]
        say(f"eta outside [{ETA_PLAUSIBLE[0]}, {ETA_PLAUSIBLE[1]}] with {name}: {len(bad)} of {len(etas)} "
            f"(min {min(etas):.3f}, max {max(etas):.3f})")
    etas5 = sorted((derive_eta(p, fric, fuel, p["lambda"]), p) for p in fired)
    say("   F5 lowest 6: " + ", ".join(f"{e:.3f}@{p['rpm']:.0f}rpm/{p['map_kpa']:.1f}kPa/{p['torque_nm']:.1f}Nm"
                                         for e, p in etas5[:6]))
    say("   F5 highest 4: " + ", ".join(f"{e:.3f}@{p['rpm']:.0f}rpm/{p['map_kpa']:.1f}kPa/{p['torque_nm']:.1f}Nm"
                                          for e, p in etas5[-4:]))

    # repeatability
    finals = {round(p["rpm"] / 50): p for p in fired
              if p["sheet"] == "High Load Final" and isinstance(p["pedal_pct"], float) and p["pedal_pct"] >= 99.5}
    spreads = []
    for p in initial:
        if isinstance(p["pedal_pct"], float) and p["pedal_pct"] >= 99.5 and round(p["rpm"] / 50) in finals:
            q = finals[round(p["rpm"] / 50)]
            spreads.append((q["rpm"], (p["torque_nm"] - q["torque_nm"]) / q["torque_nm"]))
    say("WOT repeatability (High Load Initial vs Final, torque rel.): "
        + ", ".join(f"{r:.0f}:{s * 100:+.1f}%" for r, s in spreads))

    rows, dropped = make_rows(fired)
    say("")
    say("rows (rpm, n points, WOT MAP/torque):")
    for r in rows:
        w = r["wot"]
        say(f"   {r['rpm']:8.2f}  n={len(r['pts']):2d}  "
            + (f"WOT {w['map_kpa']:.2f} kPa {w['torque_nm']:.1f} N*m (row {w['row']})" if w else "no WOT"))
    say(f"dropped (>= WOT MAP - {WOT_DROP_KPA} kPa or more air than WOT): "
        + ", ".join(f"{p['sheet'][:4]}r{p['row']}@{p['rpm']:.0f}/{p['map_kpa']:.1f}" for p in dropped))

    variants = {}
    for filt in (False, True):
        tb = build_tables(rows, fric, fuel, filt)
        variants[filt] = tb

    # throttle sizing from the highest-flow WOT point (5761 rpm)
    top = max((r["wot"] for r in rows if r["wot"]), key=lambda p: p["rpm"])
    mdot = air_per_rev(top) * top["rpm"] / 60.0
    tk = top["t_cell_c"] + 273.15
    a_eff = mdot / (top["p_cell_kpa"] * 1000.0 / math.sqrt(R_AIR * tk) * psi(top["map_kpa"] / top["p_cell_kpa"]))
    dia_mm = r6(math.sqrt(4.0 * a_eff / (math.pi * CD)) * 1e3)
    say("")
    say(f"throttle sizing: {top['rpm']:.0f} rpm WOT air {mdot * 1e3:.2f} g/s, MAP/p_cell "
        f"{top['map_kpa']:.2f}/{top['p_cell_kpa']:.2f} -> A_eff {a_eff * 1e6:.1f} mm^2 -> D {dia_mm} mm at Cd {CD}")

    def throttle_block(y0: float, shape: list[float]) -> dict:
        return {"diameter_mm": dia_mm, "count": 1, "discharge_coefficient": CD,
                "area_fraction_vs_position": {"x": [0.0, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0], "y": [y0] + shape}}

    results = {}
    for filt, tb in variants.items():
        # zero-torque leak at idle (ISA), by bisection on y(0)
        shape = [0.012, 0.03, 0.12, 0.40, 0.75, 1.0]
        probe = Replica(engine_json(tb, fric_tab, throttle_block(1e-4, shape), "", fuels, None), fuel)

        def net_at_leak(y0: float) -> float:
            probe.af = {"x": [0.0, 0.05], "y": [y0, y0]}
            return probe.steady(IDLE_RPM, 0.0)[1]

        lo, hi = 1e-5, 0.012
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if net_at_leak(mid) < 0.0:
                lo = mid
            else:
                hi = mid
        y_zero = 0.5 * (lo + hi)
        y0 = r6(LEAK_FACTOR * y_zero)
        results[filt] = {"tb": tb, "y_zero": y_zero, "y0": y0, "shape": shape}

    # filtered vs unfiltered: reproduction of every selected fired point at its own rpm/MAP/cell ambient
    say("")
    say("-- filtered vs unfiltered (brake torque at measured MAP and cell ambient, all selected fired points)")
    for filt, res in results.items():
        eng = engine_json(res["tb"], fric_tab, throttle_block(res["y0"], res["shape"]), "", fuels, None)
        rep = Replica(eng, fuel)
        errs = []
        for p in fired:
            t = rep.net(p["rpm"], p["map_kpa"] * 1000.0, p["p_cell_kpa"] * 1000.0, p["t_cell_c"] + 273.15)["t"]
            errs.append((t - p["torque_nm"], p))
        rms = math.sqrt(sum(e * e for e, _ in errs) / len(errs))
        worst = max(errs, key=lambda e: abs(e[0]))
        res["rms"], res["worst"], res["errs"] = rms, worst, errs
        say(f"   {'filtered' if filt else 'unfiltered'}: eta points dropped {len(res['tb']['dropped_eta'])}, "
            f"RMS {rms:.3f} N*m, max |err| {abs(worst[0]):.3f} N*m at {worst[1]['rpm']:.0f} rpm "
            f"{worst[1]['map_kpa']:.1f} kPa (measured {worst[1]['torque_nm']:.1f})")
        drop_ids = {(p["sheet"], p["row"]) for p in dropped}
        kept = [(e, p) for e, p in errs if (p["sheet"], p["row"]) not in drop_ids]
        gone = [(e, p) for e, p in errs if (p["sheet"], p["row"]) in drop_ids]
        for label, sub in (("points the tables were built from", kept), ("dropped near-WOT-MAP points", gone)):
            if sub:
                r_ = math.sqrt(sum(e * e for e, _ in sub) / len(sub))
                w_ = max(sub, key=lambda e: abs(e[0]))
                say(f"      {label}: n {len(sub)}, RMS {r_:.3f} N*m, max |err| {abs(w_[0]):.3f} N*m "
                    f"at {w_[1]['rpm']:.0f} rpm {w_[1]['map_kpa']:.1f} kPa")
    chosen = min((False, True), key=lambda f: (results[f]["rms"], f))
    say(f"   chosen: {'filtered' if chosen else 'unfiltered'} (lower RMS; ties -> unfiltered)")
    res = results[chosen]
    tb = res["tb"]
    say(f"   largest part-load residuals ({'filtered' if chosen else 'unfiltered'}):")
    for e, p in sorted(res["errs"], key=lambda e: -abs(e[0]))[:8]:
        say(f"      {p['sheet'][:4]} r{p['row']:3d} {p['rpm']:7.1f} rpm {p['map_kpa']:6.2f} kPa T {p['torque_nm']:7.2f} err {e:+7.3f}")

    y_zero, y0 = res["y_zero"], res["y0"]
    if not y0 < res["shape"][0]:
        raise RuntimeError(f"leak y0 {y0} not below the next area-fraction knot {res['shape'][0]}")
    say("")
    say(f"idle leak: zero-net-torque plate-0 area fraction at {IDLE_RPM:.0f} rpm ISA = {y_zero:.6f}; "
        f"y(0) = {LEAK_FACTOR} x that = {y0}")
    idle_note = (f"Throttle: diameter {dia_mm} mm at Cd {CD} [D] is the WHOLE intake restriction, sized so the "
                 f"measured {top['rpm']:.0f} rpm WOT air flow ({mdot * 1e3:.1f} g/s) passes at the measured MAP "
                 f"{top['map_kpa']:.2f} kPa from cell pressure {top['p_cell_kpa']:.2f} kPa (Cd 0.8 [AS]); "
                 "area_fraction_vs_position shape [AS] (the C11 fixture's, knot 0.05 raised to 0.012 so the idle "
                 f"feed-forward stays inside max_throttle); y(0) = {y0} = {LEAK_FACTOR} x the plate-0 leak giving zero "
                 f"net torque at idle_rpm {IDLE_RPM:.0f} (ISA) [D], so the idle PI keeps authority - it under-reads "
                 "the measured DFCO manifold pressure (the real ECU opens the plate in DFCO). idle_rpm 800 [AS] as "
                 "i4_2l_na (the EPA cell's measured idle rows sit at 590-663 rpm under 9-16 N*m dyno load).")
    eng = engine_json(tb, fric_tab, throttle_block(y0, res["shape"]), idle_note, fuels, args.sabotage)
    rep = Replica(engine_json(tb, fric_tab, throttle_block(y0, res["shape"]), idle_note, fuels, None), fuel)

    say("")
    say("-- WOT reproduction at measured MAP and cell ambient (the unit test's check), and through the throttle model")
    say("   rpm      MAP    p_cell  T_cell  T_meas   T_model  rel       | steady WOT cell amb: MAP  T      | ISA WOT: MAP T")
    for r in rows:
        w = r["wot"]
        if w is None:
            continue
        pa, ta = w["p_cell_kpa"] * 1000.0, w["t_cell_c"] + 273.15
        t = rep.net(r6(w["rpm"]), r6(w["map_kpa"]) * 1000.0, r6(w["p_cell_kpa"]) * 1000.0, r6(w["t_cell_c"]) + 273.15)["t"]
        ps_, ts_ = rep.steady(w["rpm"], 1.0, pa, ta)
        pi_, ti_ = rep.steady(w["rpm"], 1.0)
        say(f"   {r6(w['rpm']):7.1f} {r6(w['map_kpa']):7.3f} {r6(w['p_cell_kpa']):7.3f} {r6(w['t_cell_c']):6.2f} "
            f"{r6(w['torque_nm']):7.2f} {t:8.3f} {(t - w['torque_nm']) / w['torque_nm']:+.2e} | "
            f"{ps_ / 1000:6.2f} {ts_:7.2f} ({(ts_ - w['torque_nm']) / w['torque_nm'] * 100:+.1f}%) | "
            f"{pi_ / 1000:6.2f} {ti_:7.2f}")
    for rpm in (6000.0, 6500.0):
        pw, tw = rep.steady(rpm, 1.0)
        say(f"   ISA WOT at {rpm:.0f} rpm (above the data, row-clamped): MAP {pw / 1000:.2f} kPa, {tw:.1f} N*m, "
            f"{tw * rpm * 2 * math.pi / 60 / 1000:.1f} kW (report only; Toyota rates 151 kW @ 6600)")

    say("")
    say("-- motoring (unfired) at measured MAP and cell ambient, and DFCO steady MAP with plate 0 at cell ambient")
    for r in mot_rows:
        p = next(q for q in pts if q["row"] == r["row"] and q["sheet"] == "Steady State")
        t = rep.net(r["rpm"], r["map"] * 1000.0, r["p_cell"] * 1000.0, p["t_cell_c"] + 273.15, fired=False)["t"]
        pd, td = rep.steady(r["rpm"], 0.0, r["p_cell"] * 1000.0, p["t_cell_c"] + 273.15, fired=False)
        say(f"   {r['rpm']:7.1f}: measured {r['t']:7.2f} N*m at {r['map']:5.2f} kPa | model at measured MAP {t:7.2f} | "
            f"DFCO plate 0: {pd / 1000:5.2f} kPa {td:7.2f} N*m")

    # cross-check against C7's torque-map engine from the same xlsx (data/engines/toyota_a25a_fks.json)
    c7 = json.loads((WT / "data" / "engines" / "toyota_a25a_fks.json").read_text(encoding="utf-8"))
    say("")
    say("-- cross-check vs C7 data/engines/toyota_a25a_fks.json (torque_map, same xlsx)")
    say("   WOT: C7 knot | this engine at its anchor's measured MAP/cell ambient | steady plate 1 at cell ambient")
    anchors = {round(r["wot"]["rpm"], 1): r["wot"] for r in rows if r["wot"]}
    c7w = c7["wot_torque_nm_vs_rpm"]
    for x, y in zip(c7w["x"], c7w["y"]):
        w = anchors.get(round(x, 1))
        if w is None:
            say(f"   {x:7.1f} C7 {y:7.2f}  (no anchor here: C7 knot outside the measured WOT lines)")
            continue
        t = rep.net(r6(w["rpm"]), r6(w["map_kpa"]) * 1000.0, r6(w["p_cell_kpa"]) * 1000.0, r6(w["t_cell_c"]) + 273.15)["t"]
        _, ts_ = rep.steady(w["rpm"], 1.0, w["p_cell_kpa"] * 1000.0, w["t_cell_c"] + 273.15)
        say(f"   {x:7.1f} C7 {y:7.2f} | {t:7.2f} ({(t - y) / y * 100:+.3f}%) | {ts_:7.2f} ({(ts_ - y) / y * 100:+.2f}%)")
    say("   motoring: C7 m(rpm) knot | this engine at the same row's measured MAP (the same SS motoring rows)")
    c7m = c7["motoring_torque_nm_vs_rpm"]
    for r in mot_rows:
        p = next(q for q in pts if q["row"] == r["row"] and q["sheet"] == "Steady State")
        t = rep.net(r["rpm"], r["map"] * 1000.0, r["p_cell"] * 1000.0, p["t_cell_c"] + 273.15, fired=False)["t"]
        k = min(range(len(c7m["x"])), key=lambda i: abs(c7m["x"][i] - r["rpm"]))
        say(f"   {r['rpm']:7.1f}: C7 knot {c7m['x'][k]:6.0f} rpm {c7m['y'][k]:7.2f} | model {t:7.2f} "
            f"(diff {t - c7m['y'][k]:+.2f} N*m)")

    ok, worst = rep.check_monotone(LIMITER["rpm"])
    ff, p_idle = rep.idle_ff(IDLE_RPM, IDLE["max_throttle"])
    say("")
    say(f"build_maps replica: steady torque non-decreasing in plate: {'PASS' if ok else 'FAIL'} (worst step {worst:.3e} N*m)")
    say(f"idle_ff {ff:.5f} (max_throttle {IDLE['max_throttle']}), idle MAP {p_idle / 1000:.2f} kPa at {IDLE_RPM:.0f} rpm ISA")
    if not ok:
        raise RuntimeError("monotonicity replica failed")
    say(f"table sizes: rpm {len(tb['rpm'])}, p {len(tb['p'])}, m {len(tb['m'])}")

    ENGINE_JSON.write_text(dump_compact(eng) + "\n", encoding="utf-8", newline="\n")
    VEHICLE_JSON.write_text(dump_compact(vehicle_json()) + "\n", encoding="utf-8", newline="\n")
    say(f"wrote {ENGINE_JSON.relative_to(WT)}" + (f"  ** SABOTAGED: {args.sabotage} **" if args.sabotage else ""))
    say(f"wrote {VEHICLE_JSON.relative_to(WT)}")
    if not args.no_report and not args.sabotage:
        REPORT_TXT.parent.mkdir(parents=True, exist_ok=True)
        REPORT_TXT.write_text("\n".join(OUT_LINES) + "\n", encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
