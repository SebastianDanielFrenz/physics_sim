"""Reference outputs of tools/turbo_sim/meanline.py for the C++ port's cross-check (tests/unit/test_turbo_meanline.cpp).

Run from the repo root:  python tests/fixtures/turbo_meanline/make_reference.py
Writes standard_uniform.json (a complete physics_sim.turbo_stage_maps/1 file) and reference_cases.json (compact:
axes, sampling block and the per-point fields the cross-check compares). Python 3.10+, stdlib only.
"""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/turbo_sim"))
from meanline import Generator, atomic_json  # noqa: E402

HERE = Path(__file__).resolve().parent
COMP_FIELDS = ("pressure_ratio", "specific_work_j_kg", "efficiency", "outlet_total_k", "inlet_mach",
               "inlet_relative_mach", "exit_relative_mach", "shock_pressure_retention", "continuity_residual_kg_s")
TURB_FIELDS = ("mass_flow_kg_s", "specific_work_j_kg", "specific_torque_nm_per_kg_s", "efficiency", "outlet_total_k")


def uniform(g, rpm_max, ns, flow_max, nf, er_max, ne):
    speeds = [rpm_max * i / (ns - 1) for i in range(ns)]
    flows = [flow_max * i / (nf - 1) for i in range(nf)]
    a = (g.exhaust.gamma - 1) / g.exhaust.gamma
    chi = math.sqrt(1 - er_max ** (-a))
    ers = [(1 - (chi * i / (ne - 1)) ** 2) ** (-1 / a) for i in range(ne)]
    return speeds, flows, ers


def compact(name, profile, result, settings):
    def comp(p):
        return [p["status"]] + [p.get(k) for k in COMP_FIELDS]

    def turb(p):
        return [p[k] for k in TURB_FIELDS]
    return {"name": name, "settings": settings, "geometry": profile,
            "corrected_rpm": result["corrected_rpm"], "compressor_corrected_flow_kg_s": result["compressor_corrected_flow_kg_s"],
            "turbine_expansion_ratio": result["turbine_expansion_ratio"], "sampling": result.get("sampling"),
            "compressor_fields": ["status", *COMP_FIELDS], "turbine_fields": list(TURB_FIELDS),
            "compressor": [comp(p) for p in result["compressor"]], "turbine": [turb(p) for p in result["turbine"]]}


def main():
    standard = json.loads((ROOT / "tools/turbo_sim/standard_geometry.json").read_text())
    unsupported = copy.deepcopy(standard)
    unsupported["compressor"]["shock_model"] = "unsupported"
    large = copy.deepcopy(standard)
    large["name"] = "large_meanline_preliminary"
    for key in ("inducer_mm", "exducer_mm", "hub_mm", "exit_width_mm"):
        large["compressor"][key] *= 1.2
    large["turbine"]["diameter_mm"] *= 1.2
    large["turbine"]["effective_throat_mm2"] *= 1.2 ** 2
    single = json.loads((HERE / "single_120mm_geometry.json").read_text())
    cases = []
    with tempfile.TemporaryDirectory() as cache:
        g = Generator(standard, Path(cache) / "a")
        axes = uniform(g, 240000, 9, 0.9, 25, 40, 9)
        full = g.generate(*axes)
        atomic_json(HERE / "standard_uniform.json", full)
        cases.append(compact("standard_uniform", standard, full, {"kind": "uniform", "axes": [240000, 9, 0.9, 25, 40, 9]}))
        g = Generator(unsupported, Path(cache) / "b")
        axes = uniform(g, 240000, 13, 0.9, 25, 40, 5)
        cases.append(compact("unsupported_shock_uniform", unsupported, g.generate(*axes),
                             {"kind": "uniform", "axes": [240000, 13, 0.9, 25, 40, 5]}))
        g = Generator(single, Path(cache) / "c")
        axes = uniform(g, 140000, 9, 2.4, 17, 40, 5)
        cases.append(compact("single_120mm_uniform", single, g.generate(*axes),
                             {"kind": "uniform", "axes": [140000, 9, 2.4, 17, 40, 5]}))
        # build_examples.py's adaptive recipe for the 1.2 scale, with smaller budgets so the fixture stays small.
        g = Generator(large, Path(cache) / "d")
        scale = 1.2
        n = [240000 / scale * i / 8 for i in range(9)]
        w = [.9 * scale ** 2 * i / 12 for i in range(13)]
        a = (g.exhaust.gamma - 1) / g.exhaust.gamma
        chi = (1 - 40 ** (-a)) ** .5
        er = [(1 - (chi * i / 8) ** 2) ** (-1 / a) for i in range(9)]
        settings = {"kind": "adaptive", "speeds": n, "flows": w, "expansions": er, "tolerance": .03, "max_passes": 2,
                    "max_speed_lines": 33, "max_flow_points": 65, "max_expansion_points": 33}
        result = g.adaptive_generate(n, w, er, tolerance=.03, max_passes=2, max_speed_lines=33, max_flow_points=65,
                                     max_expansion_points=33)
        cases.append(compact("large_adaptive", large, result, settings))
        # build_examples.py's full "standard" recipe: only its refined axes and sampling block (the points are many).
        g = Generator(standard, Path(cache) / "e")
        n = [240000 * i / 8 for i in range(9)]
        jet_max = math.sqrt(2 * g.exhaust.cp * 1000 * g.t.nozzle_efficiency)
        terminal = 2 * jet_max / (g.t.diameter_mm * .0005) * 60 / math.tau
        if terminal > n[-1]:
            n.append(terminal)
        w = [.9 * i / 12 for i in range(13)]
        er = [(1 - (chi * i / 8) ** 2) ** (-1 / a) for i in range(9)]
        r = g.adaptive_generate(n, w, er, max_passes=5, max_speed_lines=129, max_flow_points=257)
        recipe = {"corrected_rpm": r["corrected_rpm"], "compressor_corrected_flow_kg_s": r["compressor_corrected_flow_kg_s"],
                  "turbine_expansion_ratio": r["turbine_expansion_ratio"], "sampling": r["sampling"],
                  "initial": {"speeds": n, "flows": w, "expansions": er}}
    (HERE / "reference_cases.json").write_text(json.dumps({"generator": "tools/turbo_sim/meanline.py", "cases": cases,
                                                           "standard_recipe": recipe},
                                                          sort_keys=True, separators=(",", ":")))
    for c in cases:
        print(c["name"], len(c["corrected_rpm"]), len(c["compressor_corrected_flow_kg_s"]), len(c["turbine_expansion_ratio"]),
              c["sampling"] and {k: c["sampling"][k] for k in ("passes", "budget_limited")})


if __name__ == "__main__":
    main()
