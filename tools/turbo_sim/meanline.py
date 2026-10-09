"""Offline preliminary turbo stage solver. SI internally; JSON pressures in bar.

This is a geometry-driven reduced-order calculation, NOT CFD or an OEM map.
Loss/turning coefficients are explicit assumed inputs. No shaft-speed limiter.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading

VERSION = "meanline-4"
P_REF = 101325.0
T_REF = 288.15


@dataclass(frozen=True)
class Gas:
    r: float = 287.05
    gamma: float = 1.4

    @property
    def cp(self):
        return self.r * self.gamma / (self.gamma - 1)

    def validate(self):
        if not math.isfinite(self.r) or self.r <= 0 or not 1 < self.gamma <= 1.67:
            raise ValueError("invalid ideal-gas properties")


def nozzle_flux(upstream_pa, downstream_pa, total_k, gas=Gas()):
    """kg/(m2 s), isentropic convergent nozzle, including sonic choking.

    Upstream pressure/temperature are stagnation quantities; downstream is
    static. Discharge coefficient belongs to effective area, not this function.
    NASA Glenn mass-flow relation, evaluated with downstream pressure ratio.
    """
    if upstream_pa <= 0 or downstream_pa <= 0 or total_k <= 0:
        raise ValueError("positive nozzle boundary conditions required")
    if downstream_pa >= upstream_pa:
        return 0.0
    g = gas.gamma
    q = max(downstream_pa / upstream_pa, (2 / (g + 1)) ** (g / (g - 1)))
    psi = math.sqrt(2 * g / (g - 1) * (q ** (2 / g) - q ** ((g + 1) / g)))
    return upstream_pa / math.sqrt(gas.r * total_k) * psi


def subsonic_velocity(mdot, area, pt, tt, gas):
    """Solve continuity + static/total relations on the subsonic branch.

    Return None rather than inventing a solution beyond sonic capacity.
    """
    critical = (2 / (gas.gamma + 1)) ** (gas.gamma / (gas.gamma - 1))
    capacity = area * nozzle_flux(pt, pt * critical, tt, gas)
    if mdot > capacity * (1 + 1e-10):
        return None
    lo, hi = 0.0, 1.0
    for _ in range(55):
        mach = (lo + hi) / 2
        ts = tt / (1 + (gas.gamma - 1) / 2 * mach * mach)
        ps = pt * (ts / tt) ** (gas.gamma / (gas.gamma - 1))
        velocity = mach * math.sqrt(gas.gamma * gas.r * ts)
        if area * ps / (gas.r * ts) * velocity < mdot:
            lo = mach
        else:
            hi = mach
    mach = (lo + hi) / 2
    ts = tt / (1 + (gas.gamma - 1) / 2 * mach * mach)
    return mach * math.sqrt(gas.gamma * gas.r * ts), mach


def normal_shock_pressure_retention(mach, gas=Gas()):
    """Perfect-gas total-pressure retention; unity on the subsonic branch.

    NASA normal-shock relation, evaluated in log space. Total enthalpy is
    unchanged; the associated entropy rise is -R*log(retention). Using the
    full inlet relative Mach represents an assumed normal shock, not a
    resolved blade shock angle or a transonic passage-flow solution.
    """
    if not math.isfinite(mach) or mach < 0:
        raise ValueError("finite nonnegative shock Mach required")
    if mach <= 1:
        return 1.0
    g, m2 = gas.gamma, mach * mach
    log_ratio = (g * math.log((g + 1) * m2 / ((g - 1) * m2 + 2))
                 + math.log((g + 1) / (2 * g * m2 - (g - 1)))) / (g - 1)
    return math.exp(log_ratio)


@dataclass(frozen=True)
class Compressor:
    inducer_mm: float
    exducer_mm: float
    hub_mm: float
    exit_width_mm: float
    blades: int
    exit_angle_deg: float  # relative blade angle, measured from tangent
    inlet_angle_deg: float # at RMS eye radius, from tangent
    blockage: float
    passage_loss: float
    incidence_loss: float
    diffuser_loss: float
    shock_model: str = "unsupported"

    def validate(self):
        if not 0 < self.hub_mm < self.inducer_mm < self.exducer_mm:
            raise ValueError("compressor requires hub < inducer < exducer")
        if self.exit_width_mm <= 0 or self.blades < 3 or int(self.blades) != self.blades:
            raise ValueError("invalid compressor blade geometry")
        if not 0 < self.blockage <= 1:
            raise ValueError("blockage must be in (0,1]")
        if not 0 < self.exit_angle_deg <= 90 or not 0 < self.inlet_angle_deg < 90:
            raise ValueError("invalid compressor blade angle")
        if min(self.passage_loss, self.incidence_loss, self.diffuser_loss) < 0:
            raise ValueError("loss coefficients must be nonnegative")
        if self.shock_model not in ("unsupported", "inlet_normal_entropy"):
            raise ValueError("unknown compressor shock model")


def compressor_point(c, rpm, mdot, pt=P_REF, tt=T_REF, gas=Gas()):
    """Euler work + losses + implicit exit continuity/thermodynamics.

    No pre-authored pressure/head curve: radial outlet velocity changes Euler
    work, density and pressure, so continuity is solved iteratively. Low-flow
    incidence is a risk indicator, not a resolved unsteady surge simulation.
    """
    if rpm < 0 or mdot < 0 or pt <= 0 or tt <= 0:
        raise ValueError("invalid compressor operating point")
    a1 = math.pi / 4 * (c.inducer_mm**2 - c.hub_mm**2) * 1e-6 * c.blockage
    a2 = math.pi * c.exducer_mm * c.exit_width_mm * 1e-6 * c.blockage
    inlet = subsonic_velocity(mdot, a1, pt, tt, gas)
    if inlet is None:
        return {"status": "inlet_choke"}
    cm1, mach1 = inlet
    omega = rpm * math.tau / 60
    u2 = omega * c.exducer_mm * 0.0005
    u1 = omega * math.sqrt((c.inducer_mm**2 + c.hub_mm**2) / 2) * 0.0005
    inlet_static_k = tt / (1 + (gas.gamma - 1) / 2 * mach1 * mach1)
    inlet_relative_mach = math.hypot(cm1, u1) / math.sqrt(gas.gamma * gas.r * inlet_static_k)
    shock_retention = (normal_shock_pressure_retention(inlet_relative_mach, gas)
                       if c.shock_model == "inlet_normal_entropy" else 1.0)
    shock_entropy = -gas.r * math.log(shock_retention)
    incidence = cm1 - u1 * math.tan(math.radians(c.inlet_angle_deg))
    cot_beta = 1 / math.tan(math.radians(c.exit_angle_deg))

    def state(cm2):
        # Stodola slip VELOCITY, not a fixed multiplier of the ideal triangle.
        # Blade count and exit angle therefore affect work independently.
        slip_velocity = u2 * math.pi * math.sin(math.radians(c.exit_angle_deg)) / c.blades
        cu2 = u2 - cm2 * cot_beta - slip_velocity
        work = u2 * cu2  # zero prewhirl: U2*Ctheta2 - U1*Ctheta1
        if work <= 0:
            return None
        relative2 = cm2**2 + (cu2 - u2)**2
        loss = (c.incidence_loss * incidence**2 + c.passage_loss * relative2
                + c.diffuser_loss * (cm2**2 + cu2**2)) / 2
        isentropic = work - loss
        if isentropic <= 0:
            return None
        tout = tt + work / gas.cp
        # Entropy is frame invariant: at unchanged Euler work/total enthalpy,
        # inlet shock entropy reduces outlet total pressure by this factor.
        # Apply INSIDE continuity solve; a post-hoc pressure adjustment would
        # violate the computed outlet mass-flow balance.
        pout = pt * (1 + isentropic / (gas.cp * tt)) ** (gas.gamma / (gas.gamma - 1)) * shock_retention
        # Absolute velocity is needed for outlet STATIC density (not total).
        ts = tout - (cm2**2 + cu2**2) / (2 * gas.cp)
        if ts <= 0:
            return None
        ps = pout * (ts / tout) ** (gas.gamma / (gas.gamma - 1))
        predicted = ps / (gas.r * ts) * a2 * cm2
        return predicted, work, loss, tout, pout, ts, cu2

    # Locate the first (subsonic, positive-work) continuity root. Multiple
    # roots are possible; do not jump to a supersonic outlet branch.
    previous = 0.0
    bracket = None
    upper = math.sqrt(gas.gamma * gas.r * tt) * 3 + u2
    for i in range(1, 161):
        trial = upper * i / 160
        s = state(trial)
        if s is None:
            continue
        if s[0] >= mdot:
            bracket = previous, trial
            break
        previous = trial
    if bracket is None:
        return {"status": "exit_choke_or_no_positive_head", "inlet_mach": mach1}
    lo, hi = bracket
    for _ in range(55):
        mid = (lo + hi) / 2
        s = state(mid)
        if s is None or s[0] < mdot:
            lo = mid
        else:
            hi = mid
    cm2 = (lo + hi) / 2
    predicted, work, loss, tout, pout, ts, cu2 = state(cm2)
    relative_mach = math.sqrt(cm2**2 + (cu2 - u2)**2) / math.sqrt(gas.gamma * gas.r * ts)
    # Supersonic exit/passage choking remains unresolved. Inlet shock loss
    # is optional and explicitly approximate, never a physical RPM limiter.
    status = "relative_choke" if relative_mach >= 1 else "transonic_unmodelled" if inlet_relative_mach >= 1 and c.shock_model == "unsupported" else "ok"
    if pout <= pt:
        status = "no_positive_pressure_rise"
    effective_isentropic = gas.cp * tt * ((pout / pt) ** ((gas.gamma - 1) / gas.gamma) - 1)
    return {"status": status, "pressure_ratio": pout / pt,
            "outlet_total_k": tout, "specific_work_j_kg": work,
            "shaft_power_w": mdot * work, "efficiency": effective_isentropic / work,
            "shock_model": c.shock_model, "shock_pressure_retention": shock_retention,
            "shock_entropy_j_kg_k": shock_entropy,
            "inlet_mach": mach1, "inlet_relative_mach": inlet_relative_mach,
            "exit_relative_mach": relative_mach,
            "incidence_risk": abs(incidence) / max(u1, 1e-12),
            "continuity_residual_kg_s": predicted - mdot,
            "energy_residual_w": mdot * (gas.cp * (tout - tt) - work)}


@dataclass(frozen=True)
class Turbine:
    diameter_mm: float
    effective_throat_mm2: float
    nozzle_angle_deg: float  # absolute jet angle from tangent
    nozzle_efficiency: float
    rotor_velocity_retention: float

    def validate(self):
        if self.diameter_mm <= 0 or self.effective_throat_mm2 <= 0:
            raise ValueError("positive turbine dimensions required")
        if not 0 < self.nozzle_efficiency <= 1 or not 0 <= self.rotor_velocity_retention <= 1:
            raise ValueError("invalid turbine losses")
        if not 0 < self.nozzle_angle_deg < 90:
            raise ValueError("invalid turbine nozzle angle")


def turbine_point(t, rpm, expansion_ratio, pt=P_REF, tt=1000.0, gas=Gas(gamma=1.33)):
    """Preliminary impulse-stage triangle; NOT a calibrated radial reaction rotor.

    Ideal nozzle expansion feeds a turning rotor. Overspeed can consume shaft
    work (negative Euler work); it is never clipped to a positive efficiency.
    """
    if rpm < 0 or expansion_ratio < 1 or pt <= 0 or tt <= 0:
        raise ValueError("invalid turbine operating point")
    mdot = t.effective_throat_mm2 * 1e-6 * nozzle_flux(pt, pt / expansion_ratio, tt, gas)
    his = gas.cp * tt * (1 - expansion_ratio ** (-(gas.gamma - 1) / gas.gamma))
    jet = math.sqrt(2 * his * t.nozzle_efficiency)
    cu1 = jet * math.cos(math.radians(t.nozzle_angle_deg))
    u = rpm * math.tau / 60 * t.diameter_mm * 0.0005
    cu2 = u - t.rotor_velocity_retention * (cu1 - u)
    work = u * (cu1 - cu2)
    specific_torque = t.diameter_mm * .0005 * (1 + t.rotor_velocity_retention) * (cu1 - u)
    tout = tt - work / gas.cp
    return {"status": "ok", "mass_flow_kg_s": mdot,
            "specific_work_j_kg": work, "shaft_power_w": mdot * work,
            "specific_torque_nm_per_kg_s": specific_torque,
            "outlet_total_k": tout, "efficiency": work / his if his > 0 else 0,
            "energy_residual_w": mdot * (gas.cp * (tt - tout) - work),
            "choked": expansion_ratio >= ((gas.gamma + 1) / 2) ** (gas.gamma / (gas.gamma - 1))}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(value, f, sort_keys=True, allow_nan=False, separators=(",", ":"))
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                    separators=(",", ":")).encode()).hexdigest()


class Generator:
    """Independent content-addressed point cache, resumable across sweeps.

    Grid changes do not invalidate completed points. Atomic per-point files
    survive cancellation/crashes; corrupted or mismatching entries are misses.
    """
    def __init__(self, profile, cache):
        if profile.get("format") != "physics_sim.turbo_geometry/1":
            raise ValueError("expected physics_sim.turbo_geometry/1")
        self.c = Compressor(**profile["compressor"])
        self.t = Turbine(**profile["turbine"])
        self.air = Gas(**profile.get("air", {}))
        self.exhaust = Gas(**profile.get("exhaust", {"gamma": 1.33}))
        for item in (self.c, self.t, self.air, self.exhaust):
            item.validate()
        # Reject nonfinite geometry BEFORE hashing or sampling.
        for item in (self.c, self.t):
            if any(not isinstance(v, (int, float)) or not math.isfinite(v)
                   for k, v in asdict(item).items() if k != "shock_model"):
                raise ValueError("nonfinite geometry")
        if not profile.get("source") or not profile.get("provenance"):
            raise ValueError("source and provenance required")
        self.profile = profile
        self.cache = Path(cache)
        self.computed = self.hits = 0
        self.map_hits = 0
        self.lock = threading.Lock()

    def point(self, stage, rpm, coordinate):
        rpm, coordinate = float(rpm), float(coordinate)
        geometry, gas = (self.c, self.air) if stage == "compressor" else (self.t, self.exhaust)
        identity = {"solver": VERSION, "stage": stage, "geometry": asdict(geometry),
                    "gas": asdict(gas), "rpm": rpm, "coordinate": coordinate}
        key = digest(identity)
        path = self.cache / stage / (key + ".json")
        try:
            saved = json.loads(path.read_text(encoding="utf-8"))
            if saved["identity"] == identity and saved["result_hash"] == digest(saved["result"]):
                with self.lock:
                    self.hits += 1
                return saved["result"]
        except (OSError, ValueError, KeyError, TypeError):
            pass
        result = (compressor_point(self.c, rpm, coordinate, gas=self.air) if stage == "compressor"
                  else turbine_point(self.t, rpm, coordinate, gas=self.exhaust))
        atomic_json(path, {"identity": identity, "result": result, "result_hash": digest(result)})
        with self.lock:
            self.computed += 1
        return result

    def generate(self, speeds, flows, expansions, workers=1, cancel=None, progress=None):
        for axis in (speeds, flows, expansions):
            if len(axis) < 2 or any(not math.isfinite(x) for x in axis) or any(b <= a for a, b in zip(axis, axis[1:])):
                raise ValueError("finite strictly increasing axes with >=2 points required")
        if speeds[0] < 0 or flows[0] < 0 or expansions[0] < 1:
            raise ValueError("invalid map axis domain")
        if workers < 1:
            raise ValueError("workers must be >=1")
        requests = [("compressor", s, q) for s in speeds for q in flows]
        requests += [("turbine", s, q) for s in speeds for q in expansions]
        completed = []
        # Bounded batches: cancellation does not queue an entire large sweep.
        with ThreadPoolExecutor(max_workers=workers) as pool:
            for start in range(0, len(requests), workers):
                if cancel and cancel():
                    raise InterruptedError("cancelled; completed operating points retained")
                batch = requests[start:start + workers]
                completed.extend(pool.map(lambda r: self.point(*r), batch))
                if progress:
                    progress(len(completed), len(requests))
        n = len(speeds) * len(flows)
        result = {"format": "physics_sim.turbo_stage_maps/1", "solver": VERSION,
                  "source": self.profile["source"], "provenance": self.profile["provenance"],
                  "geometry": self.profile, "reference_pressure_bar": P_REF / 1e5,
                  "compressor_reference_temperature_k": T_REF,
                  "turbine_reference_temperature_k": 1000.0,
                  "corrected_rpm": speeds, "compressor_corrected_flow_kg_s": flows,
                  "turbine_expansion_ratio": expansions,
                  "compressor": completed[:n], "turbine": completed[n:],
                  "validation": {"manufacturer_calibrated": False,
                    "compressor_shock": self.c.shock_model + "; assumed inlet shock only; supersonic outlet/passage not resolved",
                    "surge": "incidence risk only; unsteady surge not resolved",
                    "turbine": "impulse-stage approximation; reaction/volute losses not resolved",
                    "coverage": "invalid/choked points are explicit; no extrapolation is certified"}}
        result["content_hash"] = digest(result)
        return result

    def interpolation_audit(self, result):
        """Compare bilinear cell centers against independently solved points.

        This measures sampled interpolation error, not manufacturer accuracy
        or a proof that every point inside a cell has that error bound.
        """
        speeds = result["corrected_rpm"]
        reports = {}
        additions = {"speed": set(), "flow": set(), "expansion": set()}
        for stage, axis_key, fields in (
                ("compressor", "compressor_corrected_flow_kg_s", (("pressure_ratio", 1), ("specific_work_j_kg", 1000))),
                ("turbine", "turbine_expansion_ratio", (("mass_flow_kg_s", .001), ("specific_torque_nm_per_kg_s", 1)))):
            axis = result[axis_key]
            points = result[stage]
            worst = 0.0
            checked = mismatches = 0
            cells = []
            for ix in range(len(speeds) - 1):
                nc = (speeds[ix] + speeds[ix + 1]) / 2
                for iy in range(len(axis) - 1):
                    q = (axis[iy] + axis[iy + 1]) / 2
                    if stage == "turbine":
                        exponent = (self.exhaust.gamma - 1) / self.exhaust.gamma
                        chi = (math.sqrt(1 - axis[iy] ** (-exponent)) + math.sqrt(1 - axis[iy + 1] ** (-exponent))) / 2
                        q = (1 - chi * chi) ** (-1 / exponent)
                    actual = self.point(stage, nc, q)
                    corners = [points[x * len(axis) + y] for x in (ix, ix + 1) for y in (iy, iy + 1)]
                    valid = all(p["status"] == "ok" for p in corners)
                    if not valid or actual["status"] != "ok":
                        mismatches += int(valid != (actual["status"] == "ok"))
                        continue
                    error = max(abs(actual[key] - sum(p[key] for p in corners) / 4) /
                                max(abs(actual[key]), floor) for key, floor in fields)
                    cells.append((error, nc, q))
                    checked += 1
                    worst = max(worst, error)
            reports[stage] = {"cell_centers_checked": checked, "maximum_relative_error": worst,
                              "coverage_boundary_cells": mismatches}
            additions[stage] = cells
        return reports, additions

    def adaptive_generate(self, speeds, flows, expansions, tolerance=.03, max_passes=3,
                          max_speed_lines=65, max_flow_points=129, max_expansion_points=65,
                          workers=1, cancel=None, progress=None):
        if not 0 < tolerance < 1 or max_passes < 0:
            raise ValueError("invalid adaptive sampling settings")
        if any(len(axis) > budget for axis, budget in zip((speeds, flows, expansions),
                (max_speed_lines, max_flow_points, max_expansion_points))):
            raise ValueError("initial grid exceeds sampling budget")
        if cancel and cancel():
            raise InterruptedError("cancelled; operating points retained")
        identity = {"solver": VERSION, "adaptive_sampling_version": 1, "profile": self.profile,
                    "speeds": speeds, "flows": flows, "expansions": expansions,
                    "tolerance": tolerance, "max_passes": max_passes,
                    "budgets": [max_speed_lines, max_flow_points, max_expansion_points]}
        cached = self.cache / "maps" / (digest(identity) + ".json")
        try:
            saved = json.loads(cached.read_text(encoding="utf-8"))
            result = saved["result"]
            body = {k: v for k, v in result.items() if k != "content_hash"}
            if saved["identity"] == identity and digest(body) == result["content_hash"]:
                self.map_hits += 1
                if progress:
                    progress(1, 1)
                return result
        except (OSError, ValueError, KeyError, TypeError):
            pass
        exhausted = False
        for iteration in range(max_passes + 1):
            if cancel and cancel():
                raise InterruptedError("cancelled; operating points retained")
            result = self.generate(speeds, flows, expansions, workers, cancel, progress)
            audit, candidates = self.interpolation_audit(result)
            failing = [(error, "compressor", n, q) for error, n, q in candidates["compressor"] if error > tolerance]
            failing += [(error, "turbine", n, q) for error, n, q in candidates["turbine"] if error > tolerance]
            if not failing:
                break
            if iteration == max_passes:
                exhausted = True
                break
            new_speed, new_flow, new_er = set(speeds), set(flows), set(expansions)
            for error, stage, n, q in sorted(failing, reverse=True):
                if len(new_speed) < max_speed_lines:
                    new_speed.add(n)
                target, budget = (new_flow, max_flow_points) if stage == "compressor" else (new_er, max_expansion_points)
                if len(target) < budget:
                    target.add(q)
            if (len(new_speed), len(new_flow), len(new_er)) == (len(speeds), len(flows), len(expansions)):
                exhausted = True
                break
            speeds, flows, expansions = sorted(new_speed), sorted(new_flow), sorted(new_er)
        result["sampling"] = {"method": "adaptive_cell_centers", "target_relative_error": tolerance,
                              "passes": iteration, "budget_limited": exhausted,
                              "audit": audit, "certified_global_error_bound": False}
        del result["content_hash"]
        result["content_hash"] = digest(result)
        atomic_json(cached, {"identity": identity, "result": result})
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("geometry", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cache", type=Path, default=Path("out/turbo_point_cache"))
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--rpm-max", type=float, default=240000)
    parser.add_argument("--speed-lines", type=int, default=17)
    parser.add_argument("--flow-max", type=float, default=0.9)
    parser.add_argument("--flow-points", type=int, default=41)
    parser.add_argument("--expansion-max", type=float, default=40)
    parser.add_argument("--expansion-points", type=int, default=33)
    parser.add_argument("--adaptive", action="store_true")
    parser.add_argument("--relative-error", type=float, default=.03)
    parser.add_argument("--max-passes", type=int, default=3)
    args = parser.parse_args()
    if min(args.speed_lines, args.flow_points, args.expansion_points) < 2:
        parser.error("axis counts must be >=2")
    g = Generator(json.loads(args.geometry.read_text(encoding="utf-8")), args.cache)
    speeds = [args.rpm_max * i / (args.speed_lines - 1) for i in range(args.speed_lines)]
    flows = [args.flow_max * i / (args.flow_points - 1) for i in range(args.flow_points)]
    # Uniform isentropic spouting velocity, rather than uniform PR: resolves
    # the square-root flow/work behavior near expansion ratio one.
    exponent = (g.exhaust.gamma - 1) / g.exhaust.gamma
    if args.expansion_max <= 1:
        parser.error("expansion max must exceed one")
    chi_max = math.sqrt(1 - args.expansion_max ** (-exponent))
    expansions = [(1 - (chi_max * i / (args.expansion_points - 1))**2) ** (-1 / exponent)
                  for i in range(args.expansion_points)]
    try:
        progress = lambda done, total: print(f"\r{done}/{total}", end="", flush=True)
        result = (g.adaptive_generate(speeds, flows, expansions, args.relative_error,
                                     args.max_passes, workers=args.workers, progress=progress) if args.adaptive
                  else g.generate(speeds, flows, expansions, args.workers, progress=progress))
    except KeyboardInterrupt:
        print("\nCancelled. Completed points retained; previous output unchanged.")
        return 130
    atomic_json(args.output, result)
    print(f"\nWrote {args.output}; computed={g.computed}, reused={g.hits}, full_map_hits={g.map_hits}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
