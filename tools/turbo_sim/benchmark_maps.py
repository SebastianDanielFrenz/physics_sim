"""Offline verification report and graphically digitized reference comparison.

Reference points are deliberately approximate (not raw manufacturer test data).
No coefficients are fitted here and these checks do not certify this hardware.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

from meanline import Generator, Gas, P_REF, T_REF, atomic_json, compressor_point


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("geometry", type=Path)
    parser.add_argument("report", type=Path)
    parser.add_argument("--cache", type=Path, default=Path("out/turbo_point_cache"))
    args = parser.parse_args()
    profile = json.loads(args.geometry.read_text(encoding="utf-8"))
    g = Generator(profile, args.cache)
    # Read from near-flat, identifiable speed lines on the published GTX3582R
    # Gen II chart. Approximate PR uncertainty +/-0.06, flow +/-0.01 kg/s.
    # Chart points outside this model's subsonic coverage remain visible.
    reference = [(50000, .13, 1.26), (67500, .18, 1.65), (85000, .23, 2.08),
                 (100000, .27, 2.59), (110000, .30, 3.03), (120000, .34, 3.52)]
    # Garrett's own speed-sensor instructions specify corrected-flow reference
    # conditions 545 Rankine and 13.95 psia, not our ISA map reference.
    garrett_k, garrett_pa = 545 * 5 / 9, 13.95 * 6894.757293168
    comparisons = []
    for nc, wc, pr in reference:
        # Evaluate the model at the manufacturer's reference inlet itself.
        # N is treated as corrected shaft speed; chart documentation ambiguity
        # is retained as uncertainty rather than silently assumed measured RPM.
        result = compressor_point(g.c, nc, wc, garrett_pa, garrett_k, g.air)
        comparisons.append({"reference_corrected_rpm": nc, "reference_flow_kg_s": wc,
                            "reference_pressure_ratio": pr, "digitization_pr_uncertainty": .06,
                            "model": result, "relative_pressure_error":
                            (result["pressure_ratio"] / pr - 1) if "pressure_ratio" in result else None})
    speeds = [0, 30000, 60000, 90000, 120000, 150000, 180000, 210000, 240000]
    flows = [i * .075 for i in range(13)]
    exponent = (g.exhaust.gamma - 1) / g.exhaust.gamma
    chi_max = (1 - 40 ** (-exponent)) ** .5
    expansions = [(1 - (chi_max * i / 8)**2) ** (-1 / exponent) for i in range(9)]
    start = time.perf_counter()
    first = g.adaptive_generate(speeds, flows, expansions, max_passes=2, workers=1)
    first_s = time.perf_counter() - start
    second = Generator(profile, args.cache)
    start = time.perf_counter()
    again = second.adaptive_generate(speeds, flows, expansions, max_passes=2, workers=3)
    second_s = time.perf_counter() - start
    assert first == again, "worker count/cache changed map contents"
    report = {"solver": first["solver"], "timing_s": {"first_run": first_s, "warm_run": second_s},
              "first_run_points": {"computed": g.computed, "reused": g.hits},
              "warm_run_points": {"computed": second.computed, "reused": second.hits, "full_map_hits": second.map_hits},
              "worker_cache_equivalence": True,
              "coverage": dict(Counter(p["status"] for p in first["compressor"])),
              "interpolation": first["sampling"], "manufacturer_calibrated": False,
              "reference_source": "https://www.garrettmotion.com/wp-content/uploads/2022/06/GTX3582R-G2-Comp-Map-Kg-Sec-scaled.jpg",
              "corrected_conditions_source": "https://www.garrettmotion.com/wp-content/uploads/2018/06/737639-34_781328_Speed_Sensor_Kit_Install_Instructions.pdf",
              "reference_limitations": "Visually digitized approximate points; assumed speed correction; no independent measured dataset. Comparisons flag modelling error, not calibration success.",
              "compressor_reference_comparison": comparisons}
    atomic_json(args.report, report)
    atomic_json(args.report.with_suffix(".maps.json"), first)
    print(json.dumps({key: report[key] for key in ("timing_s", "warm_run_points", "coverage", "interpolation")}, indent=2))
    print("Reference pressure errors:", [round(p["relative_pressure_error"] * 100, 1) if p["relative_pressure_error"] is not None else None for p in comparisons])


if __name__ == "__main__":
    main()
