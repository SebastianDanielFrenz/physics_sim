#!/usr/bin/env python3
"""PHYS-033 P4 I7c C17a M-A: Toyota A25A-FKS with GENERATED maps.

Writes, reproducibly (stdlib only, byte-identical on unchanged inputs):
    data/engines/toyota_a25a_fks_gen.json   "kind": "simulated", a C15a "cycle" block, NO "maps" block
                                            (the loader runs the C16a map generator on it)
    tests/support/fixtures/vehicles/car_sedan_gen.json
                                            data/vehicles/car_sedan_sim.json with only the engine ref swapped
                                            (a fixture since 2026-10-13: part paths rebased to ../../../../data/...)
    data/scenarios/vehicle_gen_engine.json  data/scenarios/vehicle_sim_engine.json's bodies/events on car_sedan_gen
                                            plus one throttle lift-off event (PROVISIONAL, see below)

Frozen-literature rule (i7c plan D1). The cycle block holds ONLY
  - Tier 1  engine description, measured or cited: bore/stroke [A], CR and crank offset [C SAE 2019-01-0249
            Table 1], valve lift profiles [D digitised SAE 2019-01-0249 Fig. 11], valve counts [C];
  - Tier 1b measured ECU schedules, gridded [D] by tools/enginedata/a25a_author.py (out/a25a/a25a_tables.json):
            intake/exhaust cam phase, lambda target and (C17a follow-up) spark advance over (rpm, MAP) - inputs,
            never fitting targets; plus the Tier 2 ignition delay (Wiebe start = spark + delay);
  - Tier 2  model constants frozen from literature (Wiebe, Woschni, Cd(L/D), combustion efficiency, gas
            properties = the descriptor's NIST-JANAF defaults, friction = D13 F4, the Ecotec fit);
  - Tier 3  [AS] geometry set by a stated sizing rule (rod, valve heads, runner, ports, plenum, throttle).
No value below was chosen from the A25A's (or the Mazda's) measured brake torque, fuel flow, BSFC, air flow,
WOT envelope or motoring curve. The idle leak y(0) (THROTTLE_LEAK_Y0) is a model-consistency rule on the
engine's OWN generated maps (0.5 x the plate-0 leak giving zero net torque at idle_rpm, ISA), measured by the
[.][diag] case "C17a-MA diag: idle leak bisection" in tests/validation/test_gen_engine_dyno.cpp; it is not
fitted to any A25A measurement.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from a25a_author import build_schedule_grid  # noqa: E402  (the SAME IDW gridding as the cam/lambda schedules)

WT = Path(__file__).resolve().parents[2]
ENGINE_JSON = WT / "data" / "engines" / "toyota_a25a_fks_gen.json"
VEHICLE_JSON = WT / "tests" / "support" / "fixtures" / "vehicles" / "car_sedan_gen.json"
SCENARIO_JSON = WT / "data" / "scenarios" / "vehicle_gen_engine.json"
BASE_VEHICLE_JSON = WT / "data" / "vehicles" / "car_sedan_sim.json"
BASE_SCENARIO_JSON = WT / "data" / "scenarios" / "vehicle_sim_engine.json"
MB_ENGINE_JSON = WT / "data" / "engines" / "toyota_a25a_fks_sim.json"
TABLES_JSON = Path(r"S:\claude_code\physics_sim\out\a25a\a25a_tables.json")

# ---------------------------------------------------------------- Tier 1 ---
BORE_MM = 87.5            # [A] open-web sources, consistent with SAE 2019-01-0249 Table 1's stroke/bore 1.2
STROKE_MM = 103.4         # [A] same
CYLINDERS = 4
CR = 13.0                 # [C] SAE 2019-01-0249 Table 1 (as cited by i7c plan section 1)
PIN_OFFSET_MM = 10.1      # [C] SAE 2019-01-0249 Table 1 crankshaft offset; sign [AS] (see source)
INTAKE_VALVES = 2         # [C] DOHC 4 valves per cylinder
EXHAUST_VALVES = 2

# ---------------------------------------------------------------- Tier 3 ---
ROD_OVER_CRANK = 3.0      # [AS] l/r at the low end of the 3-4 range for small SI engines (Heywood 1988 sec. 2.2, recalled)
INTAKE_HEAD_OVER_BORE = 0.36   # [AS] midpoint of 0.35-0.37 (4-valve pent roof, Heywood 1988 Table 6.2, recalled)
EXHAUST_HEAD_OVER_BORE = 0.30  # [AS] midpoint of 0.28-0.32 (same table)
TUNE_RPM = 4800.0         # [C] Toyota's rated peak-torque SPEED (only the speed is used, never the torque)
INTAKE_PORT_LEN_MM = 80.0  # [AS] port length x total intake valve head area
EXHAUST_PORT_LEN_MM = 250.0  # [AS] port + short 4-into-1 primary length x total exhaust valve head area
PLENUM_OVER_VD = 1.2      # [AS] i7c plan rule
THROTTLE_V_MPS = 60.0     # [AS] mean throttle-bore air velocity at rated-power speed, VE 1, ISA (rule of thumb, recalled)
RATED_POWER_RPM = 6600.0  # [C] Toyota's rated-power SPEED (only the speed is used)
THROTTLE_CD = 0.8         # [AS] as the M-B file
AREA_SHAPE_X = [0.0, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0]
AREA_SHAPE_Y_ABOVE0 = [0.012, 0.03, 0.12, 0.4, 0.75, 1.0]  # [AS] the M-B file's shape above position 0
# y(0): 0.5 x the plate-0 leak area fraction giving zero net torque at idle_rpm 800 (ISA) on THIS file's own
# generated maps, measured by the [.][diag] case named in the module docstring. None = not yet measured.
THROTTLE_LEAK_Y0: float | None = 0.0026313886  # 0.5 x y0* 0.00526277725 (diag, 2026-09-28, re-measured on the maps
# generated at the measured spark schedule, C17a follow-up; was 0.0027459713 = 0.5 x 0.00549194252 at constant MBT spark)
THROTTLE_LEAK_PLACEHOLDER = 0.0022046  # the M-B value, used ONLY until the diag measures this engine's own

# ---------------------------------------------------------------- Tier 2 ---
CD_VS_L_OVER_D = {"x": [0.0, 0.1, 0.2, 0.3], "y": [0.65, 0.62, 0.55, 0.45]}
WIEBE_A, WIEBE_M, BURN_DEG, CA50_DEG = 5.0, 2.0, 60.0, 8.0
WOSCHNI = (6.18, 2.28, 3.24e-3)
WALL_T_K = 450.0
HEAD_AREA_FACTOR = 1.1
COMB_EFF_LEAN = 0.98
BACKFLOW_T_K = 800.0
# D13 F4: Chevrolet 2.5 L Ecotec LCV motored-friction fit (a third engine), pinned by
# tests/unit/test_engine_map_generator.cpp ("F4 Ecotec motored-friction fit pinned").
# C17a follow-up, flame-development (ignition) delay: Wiebe start = spark + delay (PROVISIONAL P1). Literature: MIT
# 2.61 (Heywood's course) lecture 9 "SI engine combustion I", slide "Burn duration": "For modern fast-burn engines
# under medium speed, part load condition: 0-10% ~ 15o, 0-50% ~ 25o, 0-90% ~ 35o" (crank deg from the spark).
# The frozen Wiebe already takes BURN*(ln(1/0.9)/a)^(1/(m+1)) = 16.57 deg from its start to 10 % burned, so the
# delay that makes the model's spark->10 % equal the cited 15 deg is max(0, 15 - 16.57) = 0 (DERIVED).
FLAME_DEV_0_10_DEG = 15.0
# C17a follow-up, spark schedule smoothing (DERIVED, PROVISIONAL): the same parameter-free median-3 root along MAP
# as the cam schedules. Chosen from a model-consistency rejection, BEFORE any S1/S2 run: with the raw IDW spark
# grid the runtime SimulatedEngine refuses the engine (steady torque falls with plate by 0.49 + 0.11 N*m at
# 2750 rpm, p 96.2-97.4 kPa, where the raw grid retards 18.2 -> -0.8 deg BTDC between the 89.5 and 97.3 kPa
# cells at 2700 rpm). The root keeps the WOT (105 kPa) column and monotone runs unchanged; it changes 49/240
# cells: 97.3 kPa by <= 1.35 deg (1200-3300 rpm) and isolated nearest-neighbour excursions at 600-1000 rpm by up
# to 20.1 deg (e.g. 32.6 deg BTDC between 12.5 and 11.8 at 600 rpm).
SPARK_SMOOTH = True
F4 = {"model": "polynomial_piston_speed",
      "a_kpa": 66.619521802247167, "b_kpa": 0.091085076199249656, "c_kpa": 0.34680954153442133}

ISA_T = 288.15
R_AIR = 287.05
GAMMA = 1.4


def r4(v: float) -> float:
    return float(f"{v:.4f}")


def derived() -> dict:
    r_mm = STROKE_MM / 2.0
    vd_m3 = CYLINDERS * math.pi / 4.0 * (BORE_MM * 1e-3) ** 2 * (STROKE_MM * 1e-3)
    d_in = INTAKE_HEAD_OVER_BORE * BORE_MM
    d_ex = EXHAUST_HEAD_OVER_BORE * BORE_MM
    a_in_mm2 = INTAKE_VALVES * math.pi / 4.0 * d_in ** 2
    a_ex_mm2 = EXHAUST_VALVES * math.pi / 4.0 * d_ex ** 2
    a_sound = math.sqrt(GAMMA * R_AIR * ISA_T)
    wiebe_0_10 = BURN_DEG * (math.log(1.0 / 0.9) / WIEBE_A) ** (1.0 / (WIEBE_M + 1.0))
    return {"wiebe_0_10_deg": wiebe_0_10, "ignition_delay_deg": r4(max(0.0, FLAME_DEV_0_10_DEG - wiebe_0_10)),
            "rod_mm": r4(ROD_OVER_CRANK * r_mm), "vd_m3": vd_m3, "d_in": r4(d_in), "d_ex": r4(d_ex),
            "a_in_mm2": a_in_mm2, "a_ex_mm2": a_ex_mm2, "a_sound": a_sound,
            "runner_d_mm": r4(math.sqrt(4.0 * a_in_mm2 / math.pi)),
            "intake_port_cc": r4(INTAKE_PORT_LEN_MM * a_in_mm2 * 1e-3),
            "exhaust_port_cc": r4(EXHAUST_PORT_LEN_MM * a_ex_mm2 * 1e-3),
            "plenum_l": r4(PLENUM_OVER_VD * vd_m3 * 1e3),
            "throttle_d_mm": r4(1e3 * math.sqrt(4.0 * (vd_m3 * RATED_POWER_RPM / 120.0 / THROTTLE_V_MPS) / math.pi)),
            "theta_mbt_btdc": r4(-(CA50_DEG - BURN_DEG * (math.log(2.0) / WIEBE_A) ** (1.0 / (WIEBE_M + 1.0))))}


def lift_profile(grid: list, lift: list) -> tuple[float, list, list]:
    """The digitised curve between its two bounding 0 mm samples, x relative to the first zero."""
    nz = [i for i, v in enumerate(lift) if v > 0.0]
    i0, i1 = nz[0] - 1, nz[-1] + 1
    if not (lift[i0] == 0.0 and lift[i1] == 0.0):
        raise RuntimeError("lift curve is not bounded by exact zeros")
    x0 = grid[i0]
    return float(x0), [float(g - x0) for g in grid[i0:i1 + 1]], [float(v) for v in lift[i0:i1 + 1]]


def median3_root(row: list) -> list:
    """Tukey's running median of 3 along MAP, repeated until the row no longer changes (its root signal);
    the two end cells are kept. Parameter-free apart from the window. Removes isolated one- and two-cell
    excursions of the IDW gridding (nearest-neighbour steps outside the measured hull) and keeps monotone
    runs and plateaus unchanged - e.g. the WOT cells move by at most 0.06 (intake) / 0.68 (exhaust) deg."""
    cur = [float(v) for v in row]
    while True:
        nxt = list(cur)
        for j in range(1, len(cur) - 1):
            nxt[j] = sorted(cur[j - 1:j + 2])[1]
        if nxt == cur:
            return cur
        cur = nxt


def table(sched: dict, source: str, smooth: bool = False) -> dict:
    rows = [[float(v) for v in row] for row in sched["values"]]
    if smooth:
        rows = [median3_root(r) for r in rows]
    return {"source": source, "rpm": [float(v) for v in sched["grid_rpm"]],
            "p_im_kpa": [float(v) for v in sched["grid_p_im_kpa"]],
            "values": rows}


def engine_json(tables: dict, mb: dict, y0: float, y0_note: str) -> tuple[dict, dict]:
    d = derived()
    s = tables["schedules"]
    lift = tables["valve_lift_fig11"]
    in_ref, in_x, in_y = lift_profile(lift["grid_deg"], lift["intake_phase0_mm"])
    ex_ref, ex_x, ex_y = lift_profile(lift["grid_deg"], lift["exhaust_phase0_mm"])
    ex_ref += 720.0  # same event, expressed in the 0..720 frame (0 = gas-exchange TDC)
    rp = tables["valve_lift_fig11"]["read_points"]["intake_phase0"]
    open_deg = rp["lift_end_cad"] - rp["lift_start_cad"]
    t_open = open_deg / (6.0 * TUNE_RPM)
    runner_mm = r4(1e3 * d["a_sound"] * t_open / 4.0)
    grid_rpm = [float(v) for v in s["lambda_target"]["grid_rpm"]]
    grid_p = [float(v) for v in s["lambda_target"]["grid_p_im_kpa"]]
    for k in ("cam_intake_advance_deg", "cam_exhaust_retard_deg"):
        if s[k]["grid_rpm"] != s["lambda_target"]["grid_rpm"] or s[k]["grid_p_im_kpa"] != s["lambda_target"]["grid_p_im_kpa"]:
            raise RuntimeError("schedule grids differ")
    # C17a follow-up: the measured spark schedule, gridded by a25a_author.py's own build_schedule_grid (IDW k6,
    # nearest outside the measured hull) from the SAME measured points. Self-check first: that call reproduces the
    # shipped lambda grid exactly, so the method and the point set are the ones the cam/lambda tables used.
    points = tables["measured_points"]
    lam = build_schedule_grid(points, "lambda")
    if lam["values"] != s["lambda_target"]["values"] or lam["grid_rpm"] != s["lambda_target"]["grid_rpm"] or \
            lam["grid_p_im_kpa"] != s["lambda_target"]["grid_p_im_kpa"]:
        raise RuntimeError("build_schedule_grid does not reproduce the shipped lambda grid")
    spark = build_schedule_grid(points, "spark_btdc")
    n_extrap = sum(c == "extrapolated_nearest" for row in spark["coverage"] for c in row)
    n_enr = sum(bool(v) for row in s["protection_enrichment_flag_grid"] for v in row)
    n_enr_pts = sum(1 for q in points if q["protection_enrichment_flag"] and q.get("spark_btdc") is not None)
    sm_rows = [median3_root(r) for r in spark["values"]] if SPARK_SMOOTH else spark["values"]
    sm_diff = [abs(float(a) - b) for ra, rb in zip(spark["values"], sm_rows) for a, b in zip(ra, rb)]
    n_sm = sum(1 for v in sm_diff if v > 0.0)
    max_sm = max(sm_diff)
    tier_rows = []

    def tier(field, value, t, src):
        tier_rows.append((field, value, t, src))
        return src

    cyc = {
        "source": ("I7c C17a M-A: C15a 0D cycle-model description of the Toyota A25A-FKS; the loader generates every "
                   "runtime map from it (C16a). Tiers per i7c plan D1: Tier 1 = measured/cited engine description, "
                   "Tier 1b = measured ECU schedules (inputs only), Tier 2 = literature constants frozen before any "
                   "comparison, Tier 3 = [AS] geometry set by a stated sizing rule. Nothing here was chosen from the "
                   "A25A's measured torque, fuel flow, BSFC, air flow, WOT envelope or motoring curve. Gas properties: "
                   "the descriptor's defaults (NIST-JANAF linear cv fits [Tier 2, DERIVED], ps/drivetrain/"
                   "engine_cycle_desc.h). Not modelled: intake manifold temperature (charge at ambient), exhaust back "
                   "pressure (I7d), EGR (plan D3; the EPA data show the EGR valve closed along WOT above ~2000 rpm)."),
        "rod_mm": d["rod_mm"],
        "rod_mm_source": tier("rod_mm", d["rod_mm"], "3 [AS]",
                              f"[AS] Tier 3: l/r = {ROD_OVER_CRANK} (low end of the 3-4 range for small SI engines, "
                              "Heywood 1988 sec. 2.2 [recalled, page to verify]; low end because of the long stroke) "
                              f"x crank radius {STROKE_MM / 2.0} mm. No A25A rod length found."),
        "pin_offset_mm": PIN_OFFSET_MM,
        "pin_offset_mm_source": tier("pin_offset_mm", PIN_OFFSET_MM, "1 [C] (sign [AS])",
                                     "[C] Tier 1: crankshaft offset 10.1 mm, SAE 2019-01-0249 Table 1. Sign [AS]: "
                                     "positive = the rod stands closer to the cylinder axis during the expansion "
                                     "stroke (the purpose of a desaxe crank), geometry.h's convention."),
        "compression_ratio": CR,
        "compression_ratio_source": tier("compression_ratio", CR, "1 [C]",
                                         "[C] Tier 1: geometric CR 13:1, SAE 2019-01-0249 Table 1."),
        "intake_valves": INTAKE_VALVES,
        "exhaust_valves": EXHAUST_VALVES,
        "intake_valves_source": tier("intake/exhaust_valves", "2/2", "1 [C]", "[C] Tier 1: DOHC, 4 valves per cylinder."),
        "intake_valve_d_mm": d["d_in"],
        "exhaust_valve_d_mm": d["d_ex"],
        "intake_valve_d_mm_source": tier("intake/exhaust_valve_d_mm", f"{d['d_in']}/{d['d_ex']}", "3 [AS]",
                                         f"[AS] Tier 3: head diameters {INTAKE_HEAD_OVER_BORE} x bore (intake) and "
                                         f"{EXHAUST_HEAD_OVER_BORE} x bore (exhaust), the midpoints of the 4-valve "
                                         "pent-roof ranges 0.35-0.37 / 0.28-0.32 of Heywood 1988 Table 6.2 "
                                         "[recalled, page to verify]."),
        "intake_lift": {"phase_ref_deg": in_ref, "lift_mm_vs_deg": {"x": in_x, "y": in_y}},
        "exhaust_lift": {"phase_ref_deg": ex_ref, "lift_mm_vs_deg": {"x": ex_x, "y": ex_y}},
        "intake_lift_source": tier("intake/exhaust_lift", f"phase_ref {in_ref}/{ex_ref}, span {in_x[-1]}/{ex_x[-1]} deg",
                                   "1 [D]",
                                   "[D] Tier 1: SAE 2019-01-0249 Fig. 11 (reproduced in the EPA NVFEL A25A test "
                                   "report) digitised by hand at phase 0 on a 2 deg grid (out/a25a/a25a_tables.json "
                                   "valve_lift_fig11, read uncertainty 0.3 mm / 5 deg); each curve is taken between its "
                                   "bounding 0 mm samples, x relative to the first; phase_ref = that first sample in "
                                   "the 0..720 frame. The SAE text's theta_VO(1 mm) = 138.6 deg disagrees with the "
                                   "figure's 1 mm opening crossing (~68 deg); the figure is used."),
        "runner_length_mm": runner_mm,
        "runner_length_mm_source": tier("runner_length_mm", runner_mm, "3 [AS]",
                                        f"[AS] Tier 3, quarter-wave ram rule: the runner's quarter-wave period 4L/a "
                                        f"equals the intake-open time ({open_deg:g} deg, the digitised lift start/end "
                                        f"read points) at {TUNE_RPM:g} rpm (Toyota's rated peak-torque speed [C], the "
                                        f"speed only), a = sqrt(1.4*287.05*288.15) = {d['a_sound']:.2f} m/s."),
        "runner_diameter_mm": d["runner_d_mm"],
        "runner_diameter_mm_source": tier("runner_diameter_mm", d["runner_d_mm"], "3 [AS]",
                                          "[AS] Tier 3: runner flow area = the total intake valve head area."),
        "intake_port_cc": d["intake_port_cc"],
        "exhaust_port_cc": d["exhaust_port_cc"],
        "intake_port_cc_source": tier("intake/exhaust_port_cc", f"{d['intake_port_cc']}/{d['exhaust_port_cc']}",
                                      "3 [AS]",
                                      f"[AS] Tier 3: {INTAKE_PORT_LEN_MM:g} mm port length x the total intake valve "
                                      f"head area; {EXHAUST_PORT_LEN_MM:g} mm (port plus the short 4-into-1 primary "
                                      "[C: integrated exhaust manifold]) x the total exhaust valve head area."),
        "cd_vs_l_over_d": CD_VS_L_OVER_D,
        "cd_vs_l_over_d_source": tier("cd_vs_l_over_d", "0.65/0.62/0.55/0.45 at L/D 0/0.1/0.2/0.3", "2 [AS]",
                                      "[AS] Tier 2: discharge coefficient on the curtain area falling with L/D as in "
                                      "Heywood 1988 Fig. 6-19 [recalled, page to verify]; the C15a test fixture's "
                                      "curve, frozen before any comparison; clamped beyond L/D 0.3."),
        "cam": {
            "intake_advance_deg": table(s["cam_intake_advance_deg"],
                                        "[D] Tier 1b: measured icam_adv_deg per fired point, gridded (IDW k6, nearest "
                                        "outside the measured hull) by a25a_author.py, then a running median of 3 "
                                        "along MAP per rpm row repeated to its root signal (DERIVED, see cam_source); "
                                        "reaches 55 deg (above the cited 0-50 deg spec, kept as measured)", True),
            "exhaust_retard_deg": table(s["cam_exhaust_retard_deg"],
                                        "[D] Tier 1b: measured ecam_ret_deg per fired point, gridded as above, same "
                                        "median-of-3 root smoothing along MAP", True),
        },
        "cam_source": tier("cam", "20x12 tables, median-3 root along MAP", "1b [D]",
                           "[D] Tier 1b: the EPA-measured cam phase schedules, inputs only (never fitted). DERIVED "
                           "smoothing: the raw IDW grid made the cycle model's trapped air fall with rising MAP at "
                           "600/800/1000 rpm (by up to 6.4 %; nearest-neighbour cam steps of up to 24 deg exhaust "
                           "retard between adjacent MAP cells), which the generator rejects (eta_i is resampled "
                           "over air mass). Chosen from that model-side criterion only, before any dyno "
                           "comparison: 47/240 intake and 55/240 exhaust cells change; smallest air step after "
                           "it +7.6 %."),
        "spark_advance_deg": table(spark,
                                   "[D] Tier 1b: measured 'Spark Timing, CAD BTDC' per fired point (EPA NVFEL A25A "
                                   "Tier 3 xlsx), gridded (IDW k6, nearest outside the measured hull) by "
                                   "a25a_author.py's build_schedule_grid, then the cam tables' median-3 root along MAP "
                                   "(DERIVED; see spark_advance_deg_source)", smooth=SPARK_SMOOTH),
        "spark_advance_deg_source": tier("spark_advance_deg", f"20x12 table, {spark['n_samples']} points", "1b [D]",
                                         f"[D] Tier 1b (C17a follow-up, owner ruling 1): the EPA-measured ECU spark "
                                         f"schedule, an input only (never fitted); {spark['n_samples']} measured "
                                         f"points, {n_extrap}/240 cells outside their hull (nearest value). The "
                                         f"{n_enr_pts} protection-enrichment rows are kept (real ECU behaviour, as in "
                                         f"lambda_target); {n_enr}/240 cells have one as their nearest sample "
                                         "(schedules.protection_enrichment_flag_grid). The generator runs every grid "
                                         "node at this spark; it is also the ECU's runtime schedule (the loader copies "
                                         "it into ecu.spark_advance_deg). DERIVED smoothing, chosen before any dyno "
                                         "comparison: with the raw grid the runtime rejects the engine (steady torque "
                                         "falls with plate by 0.49 N*m at 2750 rpm, p 96-97 kPa, where the raw grid "
                                         "retards 18.2 -> -0.8 deg BTDC between the 89.5 and 97.3 kPa cells at 2700 "
                                         "rpm); the cam tables' parameter-free median-3 root along MAP keeps the WOT "
                                         f"column and monotone runs and changes {n_sm}/240 cells by up to "
                                         f"{max_sm:.2f} deg (isolated nearest-neighbour excursions at 600-1000 rpm; "
                                         "97.3 kPa by <= 1.35 deg at 1200-3300 rpm)."),
        "ignition_delay_deg": d["ignition_delay_deg"],
        "ignition_delay_deg_source": tier("ignition_delay_deg", d["ignition_delay_deg"], "2 [D] PROVISIONAL",
                                          "Tier 2 [D] PROVISIONAL: Wiebe start = spark + delay. Literature: MIT 2.61 "
                                          "(Heywood's course) lecture 9 'SI engine combustion I', slide 'Burn "
                                          "duration': modern fast-burn engines, medium speed part load, spark to "
                                          f"10 % burned ~{FLAME_DEV_0_10_DEG:g} crank deg. The frozen Wiebe (a 5, m 2, "
                                          f"60 deg) already spans {d['wiebe_0_10_deg']:.2f} deg from its start to 10 % "
                                          "burned, so delay = max(0, 15 - that) = 0: the Wiebe's own development "
                                          "phase already covers the cited flame development (a literal 15 deg would "
                                          "double-count it)."),
        "wiebe_a": WIEBE_A, "wiebe_m": WIEBE_M, "burn_duration_deg": BURN_DEG, "ca50_mbt_deg_atdc": CA50_DEG,
        "wiebe_source": tier("wiebe_a/m, burn_duration, ca50", f"{WIEBE_A}/{WIEBE_M}/{BURN_DEG}/{CA50_DEG}", "2",
                             "Tier 2: Wiebe a 5, m 2 (Heywood 1988 sec. 9.2.1); 0-100 % burn 60 deg [AS-literature "
                             "range]; CA50 at MBT 8 deg ATDC (plan D11 MBT rule) - the descriptor defaults, frozen."),
        "woschni_c1_gx": WOSCHNI[0], "woschni_c1_cc": WOSCHNI[1], "woschni_c2": WOSCHNI[2],
        "woschni_source": tier("woschni_c1_gx/c1_cc/c2", f"{WOSCHNI[0]}/{WOSCHNI[1]}/{WOSCHNI[2]}", "2 [A]",
                               "Tier 2: Woschni 1967 as given by Heywood 1988 sec. 12.4.3."),
        "wall_temperature_k": WALL_T_K,
        "head_area_factor": HEAD_AREA_FACTOR,
        "exhaust_backflow_temperature_k": BACKFLOW_T_K,
        "wall_temperature_k_source": tier("wall_temperature_k/head_area_factor/exhaust_backflow_temperature_k",
                                          f"{WALL_T_K}/{HEAD_AREA_FACTOR}/{BACKFLOW_T_K}", "2 [AS]",
                                          "[AS] Tier 2: descriptor defaults (wall 450 K, pent-roof head area 1.1 x "
                                          "bore area, backflow gas 800 K), frozen."),
        "combustion_eff_lean": COMB_EFF_LEAN,
        "combustion_eff_lean_source": tier("combustion_eff_lean", COMB_EFF_LEAN, "2",
                                           "Tier 2: plan D14 (eta_c 0.98 at lambda >= 1, x lambda below)."),
        "friction": F4,
        "friction_source": tier("friction", "F4 a/b/c 66.62/0.0911/0.3468 kPa", "2 (third engine)",
                                "Tier 2, plan D13 option F4: FMEP = a + b*s + c*s^2 kPa over mean piston speed s, "
                                "FITTED ON A THIRD ENGINE (the EPA NVFEL 2013 Chevrolet 2.5 L Ecotec LCV motored "
                                "rows, pumping subtracted with the C15a fixture's ASSUMED valvetrain), not on the "
                                "A25A. Known bias: low at high speed (tests/unit/test_engine_map_generator.cpp)."),
        "grid_rpm": grid_rpm,
        "grid_p_im_kpa": grid_p,
        "grid_source": tier("grid_rpm/grid_p_im_kpa", "20 x 12", "-",
                            "plan default: the Tier 1b schedule grid (600-6900 rpm x 20-105 kPa)."),
    }

    eng = {"format": "physics_sim.engine/1", "name": "toyota_a25a_fks_gen", "kind": "simulated"}
    eng["source"] = ("I7c C17a M-A generated-map engine: Toyota A25A-FKS 2.5 L I4 (2018 Camry) as a C15a cycle-model "
                     "description; the runtime maps are GENERATED at load time by the C16a map generator (no 'maps' "
                     "block). Validated against the US EPA NVFEL A25A Tier 3 dyno data it was never fitted to "
                     "(tests/validation/test_gen_engine_dyno.cpp). Written by tools/enginedata/a25a_gen_engine.py. "
                     "Idle/limiter/starter/DFCO/idle-PI facts are the same engine facts as "
                     "data/engines/toyota_a25a_fks_sim.json (M-B) and copied from it.")
    eng["architecture"] = "piston_4stroke"
    eng["bore_mm"] = BORE_MM
    eng["stroke_mm"] = STROKE_MM
    eng["cylinders"] = CYLINDERS
    eng["geometry_source"] = tier("bore_mm/stroke_mm/cylinders", f"{BORE_MM}/{STROKE_MM}/{CYLINDERS}", "1 [A]",
                                  mb["geometry_source"].replace(" (not used by this model)", " ([C] SAE 2019-01-0249 "
                                                                "Table 1, used by the cycle block)"))
    eng["intake_manifold_volume_l"] = d["plenum_l"]
    eng["intake_manifold_volume_source"] = tier("intake_manifold_volume_l", d["plenum_l"], "3 [AS]",
                                                f"[AS] Tier 3: plenum = {PLENUM_OVER_VD} x V_d "
                                                f"({d['vd_m3'] * 1e3:.5f} L) (i7c plan rule; runners not added).")
    eng["throttle_body"] = {"diameter_mm": d["throttle_d_mm"], "count": 1, "discharge_coefficient": THROTTLE_CD,
                            "area_fraction_vs_position": {"x": AREA_SHAPE_X, "y": [y0] + AREA_SHAPE_Y_ABOVE0}}
    eng["throttle_body_source"] = tier(
        "throttle_body", f"d {d['throttle_d_mm']} mm, Cd {THROTTLE_CD}, y(0) {y0}", "3 [AS]",
        f"[AS] Tier 3: bore sized for a {THROTTLE_V_MPS:g} m/s mean air velocity at {RATED_POWER_RPM:g} rpm (Toyota's "
        "rated-power SPEED [C]) with VE 1 at ISA [rule of thumb, recalled]. DISCLOSURE: the M-B file's air-flow-sized "
        "53.6171 mm bore had been seen before this rule was applied. Cd 0.8 [AS] and the area-fraction shape above "
        "position 0 [AS] are the M-B file's. " + y0_note)
    eng["throttle_input"] = "electronic"
    eng["throttle_actuator_tau_s"] = mb["throttle_actuator_tau_s"]
    eng["throttle_actuator_source"] = mb["throttle_actuator_source"]
    eng["ecu"] = {
        "lambda_target": table(s["lambda_target"],
                               "[D] Tier 1b: measured exhaust lambda per fired point, gridded (IDW k6, nearest outside "
                               "the measured hull) by a25a_author.py; raw, protective-enrichment WOT points kept"),
    }
    eng["ecu_source"] = ("ecu.spark_advance_deg is not given here: the loader takes it from cycle.spark_advance_deg "
                         "(the measured schedule the generator ran its nodes at; C17a follow-up).")
    tier("ecu.lambda_target", "20x12 table", "1b [D]", "measured lambda schedule, raw")
    tier("ecu.spark_advance_deg", "= cycle.spark_advance_deg", "1b [D]", "measured spark schedule (loader copy)")
    eng["cycle"] = cyc
    for k in ("inertia_kgm2", "breakaway_torque_nm", "idle_rpm", "stall_rpm", "fire_rpm", "start_complete_rpm",
              "dfco_enable_rpm", "dfco_reenable_rpm", "idle_control", "limiter", "starter", "crank_axis_local",
              "initial_state", "fuels", "ignition"):
        eng[k] = mb[k]
    eng["fields_source"] = tier(
        "ECU/idle/limiter/starter/inertia/fuels", "as M-B", "copied",
        "Copied from data/engines/toyota_a25a_fks_sim.json (the same engine facts): [AS] inertia 0.16 kg*m^2, "
        "breakaway 45 N*m (>= the F4 friction at 0 rpm, ~13.2 N*m), stall/fire/start_complete 400/250/550 rpm, DFCO "
        "1300/1000 rpm, starter 140 N*m/400 rpm (as data/engines/i4_2l_na.json, no A25A measurement); idle_rpm 800 "
        "[AS] (the measured cell idle rows sit at 590-663 rpm under dyno load); idle PI kp 3e-5 /rpm, ki 6e-5 "
        "/(rpm*s), max plate 0.1 [AS] (the C11 fixture's); limiter fuel_cut 6500 rpm / 150 rpm [AS] = the cited EPA "
        "test-cell limit [C] (Report 26781 Table 5) - PROVISIONAL, 6600 rpm is Toyota's rated-power speed, not a cut. "
        "Fuels: tier3_e10_cert burnt (the fuel the EPA data were measured on), e10 selectable.")
    return eng, {"rows": tier_rows}


def dump_compact(obj, indent: int = 0) -> str:
    """JSON with numeric lists on one line (as a25a_mb_engine.py)."""
    sp = "  " * indent
    if isinstance(obj, dict):
        items = [f'{sp}  {json.dumps(k)}: {dump_compact(v, indent + 1).lstrip()}' for k, v in obj.items()]
        return sp + "{\n" + ",\n".join(items) + "\n" + sp + "}"
    if isinstance(obj, list):
        if all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in obj):
            return sp + "[" + ", ".join(json.dumps(x) for x in obj) + "]"
        if all(isinstance(x, str) for x in obj):
            return sp + json.dumps(obj)
        inner = ",\n".join(dump_compact(x, indent + 1) for x in obj)
        return sp + "[\n" + inner + "\n" + sp + "]"
    return sp + json.dumps(obj)


def rebase_part_paths(node):
    """data/vehicles/x.json -> tests/support/fixtures/vehicles/x.json: every "../part/file.json" value becomes
    "../../../../data/part/file.json" (pure path values only; free text is untouched)."""
    if isinstance(node, dict):
        return {k: rebase_part_paths(v) for k, v in node.items()}
    if isinstance(node, list):
        return [rebase_part_paths(v) for v in node]
    if isinstance(node, str) and node.startswith("../") and node.endswith(".json") and " " not in node:
        return "../../../../data/" + node[3:]
    return node


def vehicle_json() -> dict:
    base = json.loads(BASE_VEHICLE_JSON.read_text(encoding="utf-8"))
    out = {}
    for k, v in base.items():
        if k == "name":
            out[k] = "car_sedan_gen"
        elif k == "source":
            out[k] = ("data/vehicles/car_sedan_sim.json with ONLY its engine ref swapped for "
                      "engines/toyota_a25a_fks_gen.json (the I7c C17a M-A generated-map simulated Toyota A25A-FKS). "
                      "Generated by tools/enginedata/a25a_gen_engine.py. car_sedan_sim source: " + v)
        elif k == "powertrain":
            pt = json.loads(json.dumps(v))
            n = 0
            for c in pt["components"]:
                if c.get("id") == "engine":
                    c["ref"] = "../engines/toyota_a25a_fks_gen.json"
                    n += 1
            if n != 1:
                raise RuntimeError("expected exactly one engine component")
            out[k] = pt
        else:
            out[k] = v
    return rebase_part_paths(out)


def scenario_json() -> dict:
    base = json.loads(BASE_SCENARIO_JSON.read_text(encoding="utf-8"))
    base["name"] = "vehicle_gen_engine"
    for v in base["vehicles"]:
        v["vehicle_json_path"] = "../../tests/support/fixtures/vehicles/car_sedan_gen.json"
    ev = base["control_events"]
    # PROVISIONAL: one throttle lift-off before the brake event so the S5 effect assertion
    # "p_im < 40 kPa within 0.5 s of lift-off" has a lift-off to observe.
    ev.append({"t": 5.5, "channel": "throttle", "value": 0.0})
    ev.sort(key=lambda e: e["t"])
    base["telemetry_csv"] = "vehicle_gen_engine.csv"
    return base


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tier-table", action="store_true", help="print the field/value/tier/source table")
    args = ap.parse_args()
    tables = json.loads(TABLES_JSON.read_text(encoding="utf-8"))
    mb = json.loads(MB_ENGINE_JSON.read_text(encoding="utf-8"))
    if THROTTLE_LEAK_Y0 is None:
        y0 = THROTTLE_LEAK_PLACEHOLDER
        note = "y(0) PLACEHOLDER (the M-B value) - not yet measured on this engine's maps; do not ship."
    else:
        y0 = THROTTLE_LEAK_Y0
        note = ("y(0) [D] = 0.5 x the plate-0 leak area fraction giving zero net torque at idle_rpm 800 (ISA) on this "
                "engine's OWN generated maps (model-consistency rule, as M-B; measured by the [.][diag] case 'C17a-MA "
                "diag: idle leak bisection'), so the idle PI keeps authority.")
    eng, info = engine_json(tables, mb, y0, note)
    ENGINE_JSON.write_text(dump_compact(eng) + "\n", encoding="utf-8", newline="\n")
    VEHICLE_JSON.write_text(dump_compact(vehicle_json()) + "\n", encoding="utf-8", newline="\n")
    SCENARIO_JSON.write_text(json.dumps(scenario_json(), indent=2) + "\n", encoding="utf-8", newline="\n")
    if args.tier_table:
        for f, v, t, s in info["rows"]:
            print(f"| {f} | {v} | {t} | {s[:160]} |")


if __name__ == "__main__":
    main()
