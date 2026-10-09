import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import unittest

from meanline import (Compressor, Gas, Generator, P_REF, T_REF, Turbine,
                      compressor_point, nozzle_flux, turbine_point, normal_shock_pressure_retention)


class MeanlineTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(Path(__file__).with_name("standard_geometry.json").read_text())
        self.c = replace(Compressor(**self.profile["compressor"]), shock_model="unsupported")
        self.t = Turbine(**self.profile["turbine"])

    def test_nozzle_matches_nasa_sonic_formula(self):
        g = Gas()
        expected = P_REF / math.sqrt(T_REF) * math.sqrt(g.gamma / g.r) * ((g.gamma + 1) / 2) ** (-(g.gamma + 1) / (2 * (g.gamma - 1)))
        self.assertAlmostEqual(nozzle_flux(P_REF, P_REF * 0.1, T_REF, g), expected, places=10)
        self.assertEqual(nozzle_flux(P_REF, P_REF * 0.1, T_REF), nozzle_flux(P_REF, P_REF * 0.4, T_REF))
        self.assertEqual(nozzle_flux(P_REF, P_REF, T_REF), 0)

    def test_normal_shock_pressure_and_entropy(self):
        self.assertEqual(normal_shock_pressure_retention(.8), 1)
        self.assertEqual(normal_shock_pressure_retention(1), 1)
        self.assertAlmostEqual(normal_shock_pressure_retention(2), .7208738614847454, places=12)
        previous = 1
        for mach in (1.001, 1.1, 1.5, 2, 3, 5):
            retained = normal_shock_pressure_retention(mach)
            self.assertLessEqual(retained, previous)
            self.assertGreater(retained, 0)
            previous = retained
        with self.assertRaises(ValueError):
            normal_shock_pressure_retention(float("nan"))

    def test_inlet_shock_is_coupled_to_continuity_and_energy(self):
        shock = replace(self.c, shock_model="inlet_normal_entropy")
        old = compressor_point(self.c, 140000, .3)
        result = compressor_point(shock, 140000, .3)
        self.assertEqual(old["status"], "transonic_unmodelled")
        self.assertEqual(result["status"], "ok")
        self.assertLess(result["pressure_ratio"], old["pressure_ratio"])
        self.assertGreater(result["shock_entropy_j_kg_k"], 0)
        self.assertLess(abs(result["continuity_residual_kg_s"]), 1e-10)
        self.assertLess(abs(result["energy_residual_w"]), 1e-8)
        changed = compressor_point(shock, 280000, .3, 2*P_REF, 4*T_REF)
        self.assertAlmostEqual(result["pressure_ratio"], changed["pressure_ratio"], places=10)
        self.assertAlmostEqual(result["shock_entropy_j_kg_k"], changed["shock_entropy_j_kg_k"], places=10)
        for key in ("pressure_ratio", "specific_work_j_kg"):
            self.assertEqual(compressor_point(shock, 100000, .3)[key], compressor_point(self.c, 100000, .3)[key])

    def test_nozzle_corrected_similarity(self):
        reference = nozzle_flux(P_REF, P_REF / 3, T_REF)
        changed = nozzle_flux(2 * P_REF, 2 * P_REF / 3, 4 * T_REF)
        self.assertAlmostEqual(reference, changed, places=11)

    def test_compressor_conservation_and_similarity(self):
        for speed in (60000, 100000, 140000):
            for flow in (0.12, 0.24, 0.36):
                result = compressor_point(self.c, speed, flow)
                if result["status"] == "exit_choke_or_no_positive_head":
                    continue
                self.assertLess(abs(result["continuity_residual_kg_s"]), 1e-10)
                self.assertLess(abs(result["energy_residual_w"]), 1e-8)
                self.assertGreater(result["pressure_ratio"], 1)
                self.assertGreater(result["efficiency"], 0)
                self.assertLessEqual(result["efficiency"], 1)
                # Corrected N=N/sqrt(T/Tref), W=W*sqrt(T/Tref)/(P/Pref).
                changed = compressor_point(self.c, speed * 2, flow, 2 * P_REF, 4 * T_REF)
                self.assertEqual(result["status"], changed["status"])
                self.assertAlmostEqual(result["pressure_ratio"], changed["pressure_ratio"], places=10)
                self.assertAlmostEqual(changed["specific_work_j_kg"], 4 * result["specific_work_j_kg"], places=7)

    def test_geometry_changes_map_not_just_inertia(self):
        normal = compressor_point(self.c, 120000, 0.3)
        different = compressor_point(replace(self.c, exit_angle_deg=75), 120000, 0.3)
        self.assertNotEqual(normal["pressure_ratio"], different["pressure_ratio"])
        # Geometric similarity: D*1.2, N/1.2, W*1.2^2 at same inlet.
        larger = replace(self.c, inducer_mm=79.2, exducer_mm=98.4, hub_mm=26.4, exit_width_mm=7.2)
        scaled = compressor_point(larger, 100000, 0.3 * 1.44)
        self.assertAlmostEqual(normal["pressure_ratio"], scaled["pressure_ratio"], places=10)

    def test_choke_explicit_and_no_speed_limiter(self):
        self.assertEqual(compressor_point(self.c, 120000, 10)["status"], "inlet_choke")
        low = compressor_point(self.c, 120000, 0.3)
        high = compressor_point(self.c, 220000, 0.3)
        self.assertGreater(high["specific_work_j_kg"], low["specific_work_j_kg"])

    def test_turbine_energy_and_overspeed(self):
        for er in (1, 1.2, 2, 4, 30):
            for speed in (0, 50000, 100000, 300000):
                result = turbine_point(self.t, speed, er)
                self.assertLess(abs(result["energy_residual_w"]), 1e-8)
                self.assertLessEqual(result["efficiency"], 1 + 1e-12)
                self.assertGreater(result["outlet_total_k"], 0)
        self.assertLess(turbine_point(self.t, 600000, 4)["shaft_power_w"], 0)
        self.assertEqual(turbine_point(self.t, 100000, 4)["mass_flow_kg_s"], turbine_point(self.t, 100000, 30)["mass_flow_kg_s"])

    def test_cache_worker_invariance_and_extension(self):
        with tempfile.TemporaryDirectory() as directory:
            args = ([50000, 100000], [0.1, 0.3], [1.1, 3])
            first = Generator(self.profile, directory)
            result = first.generate(*args)
            self.assertEqual(first.computed, 8)
            second = Generator(self.profile, directory)
            self.assertEqual(result, second.generate(*args, workers=3))
            self.assertEqual(second.computed, 0)
            self.assertEqual(second.hits, 8)
            # Refined grid preserves existing point keys.
            second.generate([50000, 75000, 100000], args[1], args[2])
            self.assertEqual(second.computed, 4)

    def test_cache_corruption_is_miss_and_cancel_retains_points(self):
        with tempfile.TemporaryDirectory() as directory:
            g = Generator(self.profile, directory)
            calls = []
            with self.assertRaises(InterruptedError):
                g.generate([50000, 100000], [0.1, 0.3], [1.1, 3],
                           cancel=lambda: len(calls) >= 2,
                           progress=lambda done, total: calls.append(done))
            self.assertEqual(g.computed, 2)
            next(Path(directory).rglob("*.json")).write_text("corrupt")
            h = Generator(self.profile, directory)
            h.generate([50000, 100000], [0.1, 0.3], [1.1, 3])
            self.assertEqual(h.hits, 1)
            self.assertEqual(h.computed, 7)

    def test_invalid_inputs_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            for key, value in (("hub_mm", 80), ("blockage", 1.1), ("exit_angle_deg", 0), ("passage_loss", -1)):
                profile = copy.deepcopy(self.profile)
                profile["compressor"][key] = value
                with self.assertRaises(ValueError):
                    Generator(profile, directory)
            g = Generator(self.profile, directory)
            with self.assertRaises(ValueError):
                g.generate([1, 1], [0.1, 0.3], [1, 3])

    def test_adaptive_sampling_reuses_points_and_reports_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            g = Generator(self.profile, directory)
            result = g.adaptive_generate([60000, 120000], [.1, .3], [1, 4],
                                        tolerance=.01, max_passes=1,
                                        max_speed_lines=3, max_flow_points=3, max_expansion_points=3)
            self.assertLessEqual(len(result["corrected_rpm"]), 3)
            self.assertLessEqual(len(result["turbine_expansion_ratio"]), 3)
            self.assertGreater(g.hits, 0)
            self.assertIn("audit", result["sampling"])
            self.assertFalse(result["sampling"]["certified_global_error_bound"])
            self.assertTrue(result["sampling"]["budget_limited"])
            again = Generator(self.profile, directory)
            self.assertEqual(result, again.adaptive_generate([60000, 120000], [.1, .3], [1, 4],
                tolerance=.01, max_passes=1, max_speed_lines=3, max_flow_points=3, max_expansion_points=3, workers=3))
            self.assertEqual(again.map_hits, 1)
            self.assertEqual(again.computed, 0)


if __name__ == "__main__":
    unittest.main()
