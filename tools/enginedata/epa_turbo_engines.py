#!/usr/bin/env python3
"""Knock calibration phase A: engine fixtures of the four EPA NVFEL turbo benchmark engines.

Writes tests/support/fixtures/epa_turbo/<engine>/engine.json (+ turbo_configuration.json) - deterministic, stdlib only,
python -I, same bytes on re-run. Inputs: data/engines/toyota_a25a_fks_gen.json (sizing rules, frozen Tier 2 constants,
lift shapes) and tests/support/fixtures/epa_turbo/<package>_rows.json (tools/enginedata/epa_turbo_extract.py).

The sizing rules are tests/support/fixtures/mazda_skyactiv_g20's (= the A25A file's, applied unchanged); nothing is
fitted to knock data (phase B fits). Tags: [C] cited, [D] derived from a cited/measured number by a stated rule,
[M] measured, [AS] assumed. Each engine's geometry/event tier table is in ENGINES below and in the file's "source".

Valve events (gas-exchange frame, 0 = intake TDC, 720 = next intake TDC, 1 mm lift crossings [AS]: the convention
of EPA's IVC/EVO is not stated in the workbooks):
  the workbook's "valve overlap" column = ov0 + intake_adv + exhaust_ret (checked to +-1e-4 deg on every row of
  Honda/Ford 1.6/Mazda), so ov0 (overlap at zero cam phase) is read off exactly. With the zero-phase IVC0 and EVO0 the
  1 mm durations follow: d_in + d_ex = 720 + ov0 + IVC0 - EVO0 (split equally [AS]).
  Honda: IVC0/EVO0 from the workbook's effective compression/expansion stroke columns (slider-crank inversion) [D].
  Mazda 2.5T: from the workbook's Atkinson ratio column (EVO0/IVC0 fit, stroke 100 mm and l/r 3.0 [AS]) [D].
  Ford: no stroke/Atkinson column - IVC0 235 / EVO0 500 [AS] (Honda-class; EVO 40 BBDC is the Mazda fixture's rule).
Cam tables: the measured cam phase of the highest-torque row of each 50-rpm bin (the extractor's "envelope"), held
constant along p_im; where a row lacks the channel the nearest (rpm, MAP) row of the same package that has it.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

WT = Path(__file__).resolve().parents[2]
FIX = WT / "tests" / "support" / "fixtures" / "epa_turbo"
A = json.loads((WT / "data" / "engines" / "toyota_a25a_fks_gen.json").read_text(encoding="utf-8"))
ISA_P_KPA = 101.325
UP = "../../../../../"  # engine dir -> repo root
RUNNER_TUNE_RPM = 4000.0

ENGINES = [
    dict(
        key="honda_l15b7", title="2016 Honda Civic 1.5 L L15B7 (turbo, DI)", package="honda_l15b7_t3",
        cam_fill_from="honda_l15b7_t3",
        bore=73.0, stroke=89.5, cyl=4, cr=10.6,
        geom_src="[C] bore 73.0 mm (EPA ALPHA map process note for the Civic, citing SAE 2016-01-1020; read via the coordinator's research note, not re-read here); stroke 89.5 mm [D] (the workbook's effective compression stroke 'cr_effective' column reproduces the cited geometric CR 10.6 with it; the EPA note quotes 89.4); CR [C] 10.6 (SAE 2018-01-0319 / EPA note).",
        cr_src="[C] 10.6:1 geometric (SAE 2018-01-0319 Table 8, EPA ALPHA note).",
        ivc0=234.9, evo0=505.6, ov0=-44.42,
        ev_src="[D] IVC0 234.9 / EVO0 505.6 deg (zero cam phase, 1 mm [AS]) from the workbook's effective compression/expansion stroke columns by slider-crank inversion (stroke 89.5, l/r 3.0 [AS]); the Atkinson ratio fit reproduces them (235.9/503.6); ov0 -44.42 [D] exact from the overlap column.",
        peak_rpm_note="speed of the highest-torque envelope row", limiter=6500.0, limiter_src="[AS] 6500 rpm (no cited redline in the package)",
        tc="single_small", tc_src="[AS] one GT28-class turbo (the real engine has a single-scroll MHI unit, not read here)",
        fuels=["epa_tier3_25206", "epa_tier2_25842"], fuel_note="primary Tier 3 E10 (FTAG 25206, RON 91.0 / MON 83.5), second Tier 2 E0 (FTAG 25842, RON 96.1 / MON 88.5)",
        data_pkg="2016 Honda 1.5L L15B7 Engine Tier 3 Fuel (EPA NVFEL, 02-04-19)",
        firing=None, banks=None,
    ),
    dict(
        key="ford_27", title="2015 Ford F-150 2.7 L EcoBoost V6 (twin turbo, DI)", package="ford_27_t2_cell9",
        cam_fill_from="ford_27_t2_cell9",
        bore=83.0, stroke=83.0, cyl=6, cr=10.0,
        geom_src="[AS] bore 83.0 / stroke 83.0 mm recalled (Ford spec, not in the local EPA packages); CR [C] 10.0 (SAE 2018-01-0319 Table 8, Ford).",
        cr_src="[C] 10.0:1 (SAE 2018-01-0319 Table 8; one other source says 10.3).",
        ivc0=175.0, evo0=500.0, ov0=None, d_each=209.2,
        ev_src="[AS] The 2.7 L workbooks carry no overlap, stroke-length or Atkinson column, and its 'Intake Cam Phaser Retard' (+15..+53 deg at high load) has an unknown zero (the 1.6 L workbook's retard is negative, i.e. advance, at its high-load rows). The zero-phase IVC0 is therefore PLACED so that the 2000 rpm envelope row closes at ~225 deg (IVC0 175 + 53 deg retard; the other engines' high-load IVC is 200-235 deg), EVO0 500 deg (40 BBDC, the Mazda fixture's rule), 1 mm durations 209.2 deg each (the Ford 1.6 L's [D] value) -> ov0 +23.4 deg by the duration identity. Only the measured cam DELTAS are [M]; absolute valve events of this engine are assumed. Phase B should treat the 2.7 L IVC as uncertain by ~+-30 deg.",
        limiter=6000.0, limiter_src="[AS] 6000 rpm",
        tc="twin_small", tc_src="[AS] two GT28-class turbos, one per bank",
        fuels=["epa_tier2_25278", "epa_levIII_24670", "epa_tier2_26864", "epa_tier3_25402"],
        fuel_note="primary Tier 2 Cell 9 E0 (FTAG 25278, RON 96.6 / MON 88.9); also LEV III E10 (FTAG 24670, RON 92.4 / MON 84.3), Tier 2 HD1 premium E0 (FTAG 26864, RON 97.6 / MON 89.0), Tier 3 HD premium E10 (FTAG 25402, RON 97.8 / MON 88.4)",
        data_pkg="2015 Ford 2.7L EcoBoost V6 Engine Tier 2 Fuel Cell 9 (EPA NVFEL, 11-27-19); cam phase and lambda tables from this Cell 9 package (the HD packages log no cam channel)",
        firing=[1, 4, 2, 5, 3, 6], banks=[[1, 2, 3], [4, 5, 6]], bank_angle=60.0,
        firing_src="[AS] 1-4-2-5-3-6, banks 1-2-3 / 4-5-6, 60 deg V (recalled Ford numbering), even 120 deg firing intervals.",
    ),
    dict(
        key="ford_16", title="2013 Ford Escape 1.6 L EcoBoost (turbo, DI)", package="ford_16_t2",
        cam_fill_from="ford_16_t2",
        bore=79.0, stroke=81.4, cyl=4, cr=10.0,
        geom_src="[AS] bore 79.0 / stroke 81.4 mm recalled (Ford spec, not in the local EPA packages); CR [C] 10.0 (SAE 2018-01-0319 Table 8).",
        cr_src="[C] 10.0:1 (SAE 2018-01-0319 Table 8).",
        ivc0=235.0, evo0=500.0, ov0=-36.6,
        ev_src="[AS] IVC0 235 / EVO0 500 deg (Honda-class; no stroke-length or Atkinson column in the Ford workbook); ov0 -36.6 [D] exact from the workbook overlap column.",
        limiter=6200.0, limiter_src="[AS] 6200 rpm",
        tc="single_small", tc_src="[AS] one GT28-class turbo",
        fuels=["epa_tier2_25278", "epa_levIII_24670"], fuel_note="primary Tier 2 E0 (FTAG 25278, RON 96.6 / MON 88.9), second LEV III E10 (FTAG 24670, RON 92.4 / MON 84.3)",
        data_pkg="2013 Ford 1.6L EcoBoost Engine Tier 2 Fuel (EPA NVFEL, 10-25-18)",
        firing=None, banks=None,
    ),
    dict(
        key="mazda_25t", title="2016 Mazda CX-9 2.5 L Turbo SKYACTIV-G (turbo, DI)", package="mazda_25t_t3",
        cam_fill_from="mazda_25t_t3",
        bore=89.0, stroke=100.0, cyl=4, cr=10.5,
        geom_src="[AS] bore 89.0 / stroke 100.0 mm recalled (Mazda Technical Review 2016 No. 33 not readable here); CR [C] 10.5 (SAE 2018-01-0319 Table 8, Ward's). Stroke 100 mm is also the value of the Atkinson fit below.",
        cr_src="[C] 10.5:1 (SAE 2018-01-0319 Table 8).",
        ivc0=281.9, evo0=478.2, ov0=-22.0,
        ev_src="[D] IVC0 281.9 / EVO0 478.2 deg (zero cam phase, 1 mm [AS]) from the workbook's EPA Atkinson-ratio column (EVO0/IVC0 fit over all rows, stroke 100 mm and l/r 3.0 [AS], rms ratio error 0.0013); ov0 -22.0 [D] exact from the overlap column. The 1 mm durations are therefore ~251 deg each. Cooled external EGR (part of the real knock strategy) is NOT in the cycle model.",
        limiter=6000.0, limiter_src="[AS] 6000 rpm",
        tc="single_medium", tc_src="[AS] one GTX30-class turbo",
        fuels=["epa_tier2_26864", "tier3_e10_cert"], fuel_note="primary Tier 2 premium E0 (FTAG 26864, RON 97.6 / MON 89.0), second Tier 3 E10 (FTAG 26781 = tier3_e10_cert, RON 92.0 / MON 84.0)",
        data_pkg="2016 Mazda 2.5L Turbo Skyactiv-G Engine Tier 3 Fuel (EPA NVFEL, 03-13-19)",
        firing=None, banks=None,
    ),
]
# Mazda: the package whose rows feed the tables is the one named first in fuels - keep package/fuels consistent
ENGINES[3]["package"] = "mazda_25t_t2"
ENGINES[3]["cam_fill_from"] = "mazda_25t_t2"
ENGINES[3]["data_pkg"] = "2016 Mazda 2.5L Turbo Skyactiv-G Engine Tier 2 Fuel (EPA NVFEL, 03-13-19)"
# Honda: primary Tier 3 rows. Ford 1.6 / 2.7: Tier 2 rows. Each engine's primary fuel is the package's fuel.


def crossings(x, y, level=1.0):
    out = []
    for i in range(1, len(x)):
        if (y[i - 1] - level) * (y[i] - level) < 0:
            out.append(x[i - 1] + (level - y[i - 1]) * (x[i] - x[i - 1]) / (y[i] - y[i - 1]))
    return out


def scaled_profile(src, open_deg_1mm, close_deg_1mm, lift_scale):
    x = src["lift_mm_vs_deg"]["x"]
    y = src["lift_mm_vs_deg"]["y"]
    c0, c1 = crossings(x, y)
    s = (close_deg_1mm - open_deg_1mm) / (c1 - c0)
    ys = [round(v * lift_scale, 6) for v in y]
    xs = [round(v * s, 6) for v in x]
    d0, d1 = crossings(xs, ys)
    s2 = (close_deg_1mm - open_deg_1mm) / (d1 - d0)
    xs = [round(v * s * s2, 6) for v in x]
    d0, d1 = crossings(xs, ys)
    ref = round(open_deg_1mm - d0, 6)
    return {"phase_ref_deg": ref, "lift_mm_vs_deg": {"x": xs, "y": ys}}, s * s2, ref + d0, ref + d1


def load_rows(pkg):
    return json.loads((FIX / f"{pkg}_rows.json").read_text(encoding="utf-8"))["rows"]


def nearest_cam(r, pool, key):
    best, bd = None, 1e18
    for q in pool:
        if q.get(key) is None:
            continue
        d = ((q["rpm"] - r["rpm"]) / 500.0) ** 2 + ((q["map_kpa"] - r["map_kpa"]) / 20.0) ** 2
        if d < bd:
            best, bd = q, d
    return best[key]


def envelope_table(rows):
    env = sorted((r for r in rows if r["envelope"]), key=lambda r: r["rpm"])
    out, last = [], None
    for r in env:
        if last is not None and r["rpm"] - last < 1.0:
            continue
        out.append(r)
        last = r["rpm"]
    return out


def build(e):
    rows = load_rows(e["package"])
    env = envelope_table(rows)
    cam_pool = [r for r in rows if r.get("intake_cam_adv_deg") is not None or r.get("exhaust_cam_ret_deg") is not None]
    # Ford 2.7: the HD packages carry no cam; engine tables come from Cell 9 (package above)
    bore, stroke, cyl, cr = e["bore"], e["stroke"], e["cyl"], e["cr"]
    vd_l = cyl * math.pi / 4 * (bore / 1000) ** 2 * (stroke / 1000) * 1000.0
    peak = max(env, key=lambda r: r["torque_nm"])
    peak_tq_rpm = round(peak["rpm"], 0)
    rated_rpm = round(max(r["rpm"] for r in env), -1)
    max_map = max(r["map_kpa"] for r in rows)
    boost_bar = math.floor((max(r["map_kpa"] for r in env) - ISA_P_KPA) / 5.0) * 0.05  # envelope max, floored to 0.05 bar

    if e.get("d_each") is not None:
        d_in = d_ex = e["d_each"]
        e["ov0"] = round(2 * d_in - 720.0 - e["ivc0"] + e["evo0"], 4)
    else:
        d_in = d_ex = (720.0 + e["ov0"] + e["ivc0"] - e["evo0"]) / 2.0
    ivo0, ivc0 = e["ivc0"] - d_in, e["ivc0"]
    evo0, evc0 = e["evo0"], e["evo0"] + d_ex
    lift_scale = bore / 87.5
    intake_lift, s_in, io, ic = scaled_profile(A["cycle"]["intake_lift"], ivo0, ivc0, lift_scale)
    exhaust_lift, s_ex, eo, ec_ = scaled_profile(A["cycle"]["exhaust_lift"], evo0, evc0, lift_scale)
    in_span = intake_lift["lift_mm_vs_deg"]["x"][-1] - intake_lift["lift_mm_vs_deg"]["x"][0]

    crank_r = stroke / 2
    rod = round(3.0 * crank_r, 4)
    iv_d = round(0.36 * bore, 4)
    ev_d = round(0.30 * bore, 4)
    a_iv = 2 * math.pi / 4 * iv_d ** 2
    a_ev = 2 * math.pi / 4 * ev_d ** 2
    runner_d = round(math.sqrt(4 * a_iv / math.pi), 4)
    a_snd = math.sqrt(1.4 * 287.05 * 288.15)
    t_open = in_span / (6.0 * RUNNER_TUNE_RPM)
    runner_l = round(a_snd * t_open / 4.0 * 1000.0, 4)
    port_in = round(80.0 * a_iv / 1000.0, 4)
    port_ex = round(250.0 * a_ev / 1000.0, 4)
    plenum = round(1.2 * vd_l, 4)
    pr = (ISA_P_KPA + 100.0 * boost_bar) / ISA_P_KPA
    q = vd_l / 1000.0 * rated_rpm / 120.0 * pr
    throttle_d = round(math.sqrt(4 * (q / 60.0) / math.pi) * 1000.0, 4)

    # cam and lambda tables from the measured envelope rows
    rpms, icam, ecam, lam = [], [], [], []
    for r in env:
        rpms.append(round(r["rpm"], 3))
        icam.append(round(r["intake_cam_adv_deg"] if r.get("intake_cam_adv_deg") is not None else nearest_cam(r, cam_pool, "intake_cam_adv_deg"), 4))
        ecam.append(round(r["exhaust_cam_ret_deg"] if r.get("exhaust_cam_ret_deg") is not None else nearest_cam(r, cam_pool, "exhaust_cam_ret_deg"), 4))
        lam.append(round(r["lambda"], 5))
    n_fill_i = sum(1 for r in env if r.get("intake_cam_adv_deg") is None)
    n_fill_e = sum(1 for r in env if r.get("exhaust_cam_ret_deg") is None)
    fill_note = (f" Rows without the channel ({n_fill_i} intake, {n_fill_e} exhaust of {len(env)}) take the value of the nearest (rpm/500, MAP/20 kPa) row of the same package that has it [D]."
                 if (n_fill_i or n_fill_e) else "")
    row_ids = ", ".join(f"{r['sheet']}!{r['row']}" for r in env)
    cam_p = [20.0, 300.0]
    lam_p = [20.0, 90.0, 100.0, 300.0]
    icam_tab = {"source": f"[M] measured intake cam advance (positive = advanced = earlier; workbook CAD {('ADV' if e['key'] == 'honda_l15b7' else 'RET, sign flipped')}) of the highest-torque row of each 50-rpm bin, {e['data_pkg']}, rows {row_ids}; held constant along p_im [AS: only high-load nodes are compared]; clamped outside {rpms[0]:.0f}-{rpms[-1]:.0f} rpm.{fill_note}",
                "rpm": rpms, "p_im_kpa": cam_p, "values": [[v, v] for v in icam]}
    ecam_tab = {"source": "[M] measured exhaust cam retard (positive = retarded = later) of the same rows, same rule as intake_advance_deg.",
                "rpm": rpms, "p_im_kpa": cam_p, "values": [[v, v] for v in ecam]}
    lam_tab = {"source": f"[M] measured exhaust lambda of the same rows at p_im >= 100 kPa; 1.0 up to 90 kPa [AS: part load is stoichiometric], blended between; clamped outside {rpms[0]:.0f}-{rpms[-1]:.0f} rpm.",
               "rpm": rpms, "p_im_kpa": lam_p, "values": [[1.0, 1.0, v, v] for v in lam]}

    grid_rpm = [1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0, 6000.0, e["limiter"]]
    grid_rpm = sorted(set(grid_rpm))
    top = int(math.ceil(max_map / 20.0) * 20 + 10)
    grid_p = [float(v) for v in range(90, top + 1, 20)]

    c = A["cycle"]
    cycle = {
        "source": f"Knock calibration phase A fixture (NOT a shipped engine): C15a 0D cycle-model description of the {e['title']}. Built by tools/enginedata/epa_turbo_engines.py from cited facts plus the toyota_a25a_fks_gen.json Tier 2/3 rules applied UNCHANGED (as tests/support/fixtures/mazda_skyactiv_g20); nothing is fitted to knock data. TIER TABLE - Tier 1 [C]: compression ratio, cylinder count (SAE 2018-01-0319 Table 8). Tier 1b [D]/[M]: valve events, cam phase and lambda schedules from the EPA NVFEL workbook ({e['data_pkg']}). Tier 2 (frozen descriptor constants, as A25A): Wiebe a 5 / m 2 / 40.3205 deg, ignition delay 3.8632, CA50 MBT 8, Woschni, wall 450 K, head factor 1.1, Cd vs L/D, combustion efficiency, F4 friction. Tier 3 [AS]: bore/stroke where not cited; rod l/r 3.0; pin offset 0; valve heads 0.36/0.30 x bore, 2+2 valves; lift shapes = the A25A's digitised curves stretched to the derived 1 mm events, max lift scaled with the bore (L/D as A25A); 1 mm duration split equally between intake and exhaust; runner quarter-wave at the tuning speed ({RUNNER_TUNE_RPM:.0f} rpm [AS]) over the intake-open span; runner area = intake valve head area; ports 80/250 mm x head area; plenum 1.2 V_d; lumped (no pipes): exhaust and boost are the turbo configuration's.",
        "rod_mm": rod,
        "rod_mm_source": "[AS] Tier 3: l/r = 3.0 x crank radius %.2f mm (the A25A rule)." % crank_r,
        "pin_offset_mm": 0.0,
        "pin_offset_mm_source": "[AS] no figure: 0.",
        "compression_ratio": cr,
        "compression_ratio_source": e["cr_src"],
        "intake_valves": 2,
        "exhaust_valves": 2,
        "intake_valves_source": "[AS] DOHC, 4 valves per cylinder (all four engines).",
        "intake_valve_d_mm": iv_d,
        "exhaust_valve_d_mm": ev_d,
        "intake_valve_d_mm_source": "[AS] Tier 3: 0.36 / 0.30 x bore (the A25A rule).",
        "intake_lift": intake_lift,
        "exhaust_lift": exhaust_lift,
        "intake_lift_source": "[AS] shape: the A25A's digitised SAE 2019-01-0249 Fig. 11 curves, x stretched by %.6f (intake) / %.6f (exhaust) so the 1 mm crossings land on IVO %.1f / IVC %.1f and EVO %.1f / EVC %.1f deg at zero cam phase (%s); lift x %.6f (bore ratio, L/D held)." % (s_in, s_ex, io, ic, eo, ec_, e["ev_src"], lift_scale),
        "runner_length_mm": runner_l,
        "runner_length_mm_source": "[AS] Tier 3 quarter-wave rule (the A25A's): 4L/a = the intake-open time (%.2f deg span) at %.0f rpm (tuning speed [AS], the Mazda 2.0 fixture's 4000), a = 340.29 m/s." % (in_span, RUNNER_TUNE_RPM),
        "runner_diameter_mm": runner_d,
        "runner_diameter_mm_source": "[AS] Tier 3: runner flow area = the total intake valve head area.",
        "intake_port_cc": port_in,
        "exhaust_port_cc": port_ex,
        "intake_port_cc_source": "[AS] Tier 3: 80 / 250 mm x the total intake / exhaust valve head area (the A25A rule).",
        "cd_vs_l_over_d": c["cd_vs_l_over_d"],
        "cd_vs_l_over_d_source": "Tier 2: the A25A's frozen curve.",
        "cam": {"intake_advance_deg": icam_tab, "exhaust_retard_deg": ecam_tab},
        "cam_source": "[M] measured WOT-envelope phases (tables' own sources).",
        "ignition_delay_deg": c["ignition_delay_deg"],
        "ignition_delay_deg_source": "Tier 2 [D]: the A25A's frozen value (MIT 2.61 Lecture 9 derivation).",
        "wiebe_a": c["wiebe_a"],
        "wiebe_m": c["wiebe_m"],
        "burn_duration_deg": c["burn_duration_deg"],
        "ca50_mbt_deg_atdc": c["ca50_mbt_deg_atdc"],
        "wiebe_source": "Tier 2: the A25A's frozen Wiebe (a 5, m 2, 40.3205 deg [C MIT 2.61], CA50 at MBT 8 ATDC).",
        "woschni_c1_gx": c["woschni_c1_gx"],
        "woschni_c1_cc": c["woschni_c1_cc"],
        "woschni_c2": c["woschni_c2"],
        "woschni_source": "Tier 2: Woschni 1967 (Heywood 1988 sec. 12.4.3).",
        "wall_temperature_k": c["wall_temperature_k"],
        "head_area_factor": c["head_area_factor"],
        "exhaust_backflow_temperature_k": c["exhaust_backflow_temperature_k"],
        "wall_temperature_k_source": "[AS] Tier 2 descriptor defaults (as A25A).",
        "combustion_eff_lean": c["combustion_eff_lean"],
        "combustion_eff_lean_source": "Tier 2: plan D14.",
        "friction": c["friction"],
        "friction_source": "Tier 2: the A25A file's F4 polynomial (fitted on another engine; irrelevant to theta_kl).",
        "grid_rpm": grid_rpm,
        "grid_p_im_kpa": grid_p,
        "grid_source": "[AS] high-load grid only: p_im 90 kPa up to the highest measured MAP of the package (%.1f kPa) + margin, in 20 kPa steps; at part load the measured 40-80 deg overlap makes the lumped cycle unstable (no IVC state). rpm 1000 to the assumed limiter." % max_map,
    }
    if e["firing"]:
        cycle["firing_order"] = e["firing"]
        cycle["banks"] = e["banks"]
        cycle["bank_angle_deg"] = e["bank_angle"]
        cycle["firing_order_source"] = e["firing_src"]

    engine = {
        "format": "physics_sim.engine/1",
        "name": f"{e['key']}_epa_turbo",
        "kind": "simulated",
        "source": f"Knock calibration phase A test fixture (NOT a shipped engine): {e['title']} as a generated-map engine; tier table in cycle.source. Measured data: US EPA NVFEL benchmark package '{e['data_pkg']}'; rows extracted by tools/enginedata/epa_turbo_extract.py into tests/support/fixtures/epa_turbo/{e['package']}_rows.json. Fuels: {e['fuel_note']}.",
        "architecture": "piston_4stroke",
        "bore_mm": bore,
        "stroke_mm": stroke,
        "cylinders": cyl,
        "geometry_source": e["geom_src"],
        "intake_manifold_volume_l": plenum,
        "intake_manifold_volume_source": "[AS] Tier 3: 1.2 x V_d (%.5f L)." % vd_l,
        "throttle_body": {
            "diameter_mm": throttle_d,
            "count": 1,
            "discharge_coefficient": A["throttle_body"]["discharge_coefficient"],
            "area_fraction_vs_position": A["throttle_body"]["area_fraction_vs_position"],
        },
        "throttle_body_source": f"[AS] Tier 3: 60 m/s at the highest measured envelope speed ({rated_rpm:.0f} rpm) with VE 1 at ISA and the boost pressure ratio {pr:.3f} of the turbo configuration's target ({boost_bar:.2f} bar gauge); Cd and area-fraction shape copied from the A25A file [AS].",
        "throttle_input": "electronic",
        "throttle_actuator_tau_s": 0.05,
        "throttle_actuator_source": "[AS] as the A25A file.",
        "ecu": {
            "lambda_target": lam_tab,
            "ignition_strategy": "knock_limited",
            "knock_margin_deg": 0.0,
        },
        "ecu_source": "knock_limited, margin 0, no spark table (MBT tracking) - the KV2 configuration of test_knock_calibration.cpp; phase B sets the margin.",
        "default_forced_induction": "turbo_configuration.json",
        "cycle": cycle,
    }
    for k in ("inertia_kgm2", "breakaway_torque_nm", "idle_rpm", "stall_rpm", "fire_rpm", "start_complete_rpm", "dfco_enable_rpm",
              "dfco_reenable_rpm", "idle_control", "starter", "crank_axis_local", "initial_state"):
        engine[k] = A[k]
    engine["limiter"] = {"type": "fuel_cut", "rpm": e["limiter"], "hysteresis_rpm": 150}
    engine["limiter_source"] = e["limiter_src"]
    engine["fuels"] = [f"{UP}data/fuels/{f}.json" for f in e["fuels"]]
    engine["ignition"] = "spark"
    engine["fields_source"] = "[AS] runtime fields (inertia, idle, DFCO, starter) copied from toyota_a25a_fks_gen.json; unused by the knock calibration."

    tc = {
        "extends": f"{UP}data/turbo_configurations/{e['tc']}.json",
        "name": f"{e['key']}_epa_turbo",
        "target_boost_bar": round(boost_bar, 2),
        "source": f"[AS] {e['tc_src']}; template {e['tc']}.json with target_boost_bar {boost_bar:.2f} bar gauge = the highest-torque envelope row's MAP maximum ({max(r['map_kpa'] for r in env):.1f} kPa abs, [M]) minus ISA 101.325 kPa, floored to 0.05 bar. Hardware and PID are the template's; boost is not a knock input (phase B replays the measured MAP/IMT).",
    }
    summary = dict(key=e["key"], vd=vd_l, rod=rod, valves=(iv_d, ev_d), runner=(runner_l, runner_d), ports=(port_in, port_ex), plenum=plenum,
                   throttle=throttle_d, d=(d_in, d_ex), events=(io, ic, eo, ec_), boost=boost_bar, env=len(env), peak_rpm=peak_tq_rpm, grid_p=grid_p)
    return engine, tc, summary


def main() -> int:
    for e in ENGINES:
        engine, tc, s = build(e)
        d = FIX / e["key"]
        d.mkdir(parents=True, exist_ok=True)
        (d / "engine.json").write_text(json.dumps(engine, indent=2) + "\n", encoding="utf-8", newline="\n")
        (d / "turbo_configuration.json").write_text(json.dumps(tc, indent=2) + "\n", encoding="utf-8", newline="\n")
        print(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
