"""Single source of truth for the M1 test ground layout: every meshed area's
coordinates, the circuit centerline definition, physical surface parameters,
spawn points and named test zones. build_mesh.py, collision.py, track_json.py,
terrain.py and validate.py all read this module so the geometry, the JSON
metadata, the terrain's flat-zone corridors and the checks can never drift
apart.

Frame: the *world* frame is local east-north-up (x=east, y=north, z=up),
metres, radians unless noted (*_deg fields are degrees for readability) --
algebraically the same right-handed x,y,z=up axes as a vehicle's own ISO 8855
frame (x forward, y left, z up), so no basis change is needed; the world is
simply oriented so the main spawn's forward direction (+x) is due east and
its left (+y) is due north. A vehicle's own forward/left only equal
east/north when it faces due east (heading 0). Origin (0,0,0) is the main
spawn point, at the centre of the terrain's flat disc (PLAN.md M1; terrain.py).

Scope note (2026-09-22): the world is 100 x 100 km (+-50 km); everything
outside the small meshed test-feature footprints is the procedural terrain
described by terrain.py/terrain.json, not a mesh. The features below are
exact static meshes placed on the terrain's flat zones (the 3 km main disc
around the origin, plus a flat corridor each for the long straight and the
airstrip, derived here from those features' own footprints).
"""
import math

from .path import Segment, CircuitSolver, build_samples, apply_elevation, drop_closing_duplicate

# --------------------------------------------------------------------- surfaces
# lambda_mu: Pacejka-style friction scale relative to dry asphalt = 1.0.
# crr: rolling-resistance coefficient (dimensionless, F_roll = crr * N).
# Sources noted per entry; where no direct measurement is cited the value is a
# plausible literature range and is flagged as such (per PLAN.md section 8).
#
# This is the *canonical* Python source for the table; build.py writes it out
# to its own file, data/surfaces/surfaces.json, and both test_ground.json and
# terrain.json reference that file rather than embedding it -- so a future
# real-world geodata module (PLAN.md P11) can map its land-cover classes onto
# the same ids/values instead of every terrain source keeping its own copy.
SURFACES = {
    "asphalt": dict(
        display_name="Dry asphalt", lambda_mu=1.00, crr=0.013,
        source="Reference surface (lambda_mu=1.0 by definition). Crr for dry asphalt/concrete "
               "roads: Wong, Theory of Ground Vehicles, 5th ed., Table 1.3 (typical range 0.010-0.015; "
               "midpoint used)."),
    "kerb": dict(
        display_name="Painted concrete kerb", lambda_mu=0.90, crr=0.013,
        source="Painted concrete: base Crr as asphalt (Wong Table 1.3); lambda_mu reduced ~10% from "
               "dry asphalt for the paint film, especially when wet -- illustrative test-ground estimate, "
               "not a measured value."),
    "grass": dict(
        display_name="Dry grass on firm soil", lambda_mu=0.42, crr=0.08,
        source="Wong, Theory of Ground Vehicles, Table 1.3 / typical terrain friction-coefficient "
               "tables: firm turf mu ~ 0.35-0.45 dry, Crr ~ 0.06-0.10; midpoints used, flagged as a "
               "plausible published range rather than a specific measurement."),
    "dirt": dict(
        display_name="Dry compact dirt / gravel road", lambda_mu=0.60, crr=0.04,
        source="Wong, Theory of Ground Vehicles, Table 1.3: dry, hard-packed dirt/gravel road, "
               "mu ~ 0.55-0.65, Crr ~ 0.03-0.05; midpoints used (plausible published range)."),
    "low_mu": dict(
        display_name="Polished ice (unstudded tyre)", lambda_mu=0.10, crr=0.010,
        source="Generic vehicle-dynamics references (e.g. SAE / SAE-adjacent tables cited by Milliken, "
               "Race Car Vehicle Dynamics, ch. 1) give unstudded-tyre-on-ice peak mu ~ 0.05-0.15; "
               "midpoint used. Crr assumed close to dry asphalt (smooth, hard surface) -- plausible "
               "range, not a specific measurement."),
    "field": dict(
        display_name="Cultivated field / tilled soil", lambda_mu=0.50, crr=0.09,
        source="Wong, Theory of Ground Vehicles, Table 1.3: tilled/cultivated soil bracketed between "
               "firm dirt and loose soil, mu ~ 0.45-0.55, Crr ~ 0.08-0.10; midpoints used (plausible "
               "published range). Used by terrain.py's surface rules for the procedural farmland, not "
               "by any mesh here."),
}

# ----------------------------------------------------------------------- straight
STRAIGHT = dict(
    length=1000.0, width=12.0, y=-150.0, x0=-500.0,   # centerline runs x0 .. x0+length at lateral y
    marker_every=100.0, marker_width=0.30, line_width=0.5,
)

# ------------------------------------------------------------------------ skidpad
SKIDPAD = dict(
    center=(0.0, 150.0), radius=40.0, width=10.0, line_width=0.25,
    radius_lines=(35.0, 37.5, 40.0, 42.5, 45.0),
)

# ------------------------------------------------------------------- surface patches
GRASS_PATCH = dict(center=(-300.0, -480.0), length=160.0, width=70.0)
DIRT_PATCH = dict(center=(0.0, -480.0), length=160.0, width=70.0)
SPLIT_MU = dict(center=(300.0, -480.0), length=300.0, width=11.0)  # left half (y>0) asphalt, right half low_mu

# ---------------------------------------------------------- long (top-speed) straight
# 10 km: several km to reach a hypercar's top speed, several more to brake to 0.
# Its own flat corridor (well outside the 3 km main disc; a small overlap with
# the disc's own blend is harmless, flat_mask() combines zones with max()).
LONG_STRAIGHT = dict(
    length=10_000.0, width=14.0, x0=3000.0, y=800.0, heading_deg=0.0,
    marker_every=500.0, marker_width=0.5,
    corridor=dict(center=(3000.0 + 10_000.0 / 2, 800.0), heading_deg=0.0,
                  half_length=10_000.0 / 2 + 300.0, half_width=120.0, blend=200.0),
)

# ----------------------------------------------------------------------- airstrip
# Runway 1500 m + a 200 m overrun each end = 1900 m paved; >= 2 km obstacle-free
# approach beyond each end (P6). Oriented along +y so it cannot be confused
# with any of the road features (all of which run roughly along +x).
AIRSTRIP_RUNWAY = 1500.0
AIRSTRIP_OVERRUN = 200.0
AIRSTRIP_PAVED = AIRSTRIP_RUNWAY + 2 * AIRSTRIP_OVERRUN
AIRSTRIP_APPROACH = 2000.0
AIRSTRIP = dict(
    x0=0.0, y0=19_000.0, heading_deg=90.0, width=30.0, length=AIRSTRIP_PAVED,
    overrun=AIRSTRIP_OVERRUN, runway=AIRSTRIP_RUNWAY,
    corridor=dict(center=(0.0, 19_000.0 + AIRSTRIP_PAVED / 2), heading_deg=90.0,
                  half_length=AIRSTRIP_PAVED / 2 + AIRSTRIP_APPROACH, half_width=300.0, blend=250.0),
)

# ------------------------------------------------------------------------- ramps
RAMP_WIDTH = 8.0
RAMP_APPROACH = 15.0
RAMP_PLATEAU = 25.0
RAMP_Y0 = 450.0
RAMP_SPACING = 24.0
RAMP_GRADIENTS = (0.05, 0.10, 0.15, 0.30, 0.50)


def ramp_profile(grade, slope_len):
    run = slope_len / math.sqrt(1.0 + grade * grade)
    rise = run * grade
    return run, rise


def ramp_geometry():
    """One entry per ramp: id, y offset, grade, slope length, run/rise, and the
    s-ranges (flat/up/plateau/down/flat) of its longitudinal profile."""
    out = []
    for i, g in enumerate(RAMP_GRADIENTS):
        slope_len = 60.0 if g == 0.15 else 30.0
        run, rise = ramp_profile(g, slope_len)
        a = RAMP_APPROACH
        p = RAMP_PLATEAU
        s_flat0 = (0.0, a)
        s_up = (a, a + run)
        s_plateau = (a + run, a + run + p)
        s_down = (a + run + p, a + 2 * run + p)
        s_flat1 = (a + 2 * run + p, 2 * a + 2 * run + p)
        out.append(dict(
            id=f"ramp_{int(round(g * 100)):02d}", grade=g, slope_len=slope_len, run=run, rise=rise,
            y=RAMP_Y0 + i * RAMP_SPACING, total_len=s_flat1[1],
            s_flat0=s_flat0, s_up=s_up, s_plateau=s_plateau, s_down=s_down, s_flat1=s_flat1,
        ))
    return out


RAMPS = ramp_geometry()
SIDE_SLOPE = dict(
    y=RAMP_Y0 + len(RAMP_GRADIENTS) * RAMP_SPACING, width=10.0, cross_grade=0.10,
    core_len=60.0, apron=10.0,
)

# -------------------------------------------------------------------------- circuit
CIRCUIT_ORIGIN = (-260.0, -1250.0)   # world position of the circuit's own path-local (0,0); its own
# footprint reaches y=-498.5 relative to a -1050 origin (measured from the actual samples), which
# crept within ~17 m of the grass/dirt/split-mu patches row at y in [-515,-445] -- shifted 200 m
# further south for a clear ~180 m margin (found by the cross-surface height check in validate.py).
CIRCUIT_HEADING0 = 0.0
CIRCUIT_WIDTH = 12.0
CIRCUIT_KERB_WIDTH = 1.0
CIRCUIT_ELEV_AMPLITUDE = 3.0  # metres, "a few metres at most"
CIRCUIT_STEP = 2.0


def circuit_segments():
    return [
        Segment("line", name="main_straight"),
        Segment("arc", radius=180, angle_deg=75, name="sweeper"),
        Segment("line", length=180, name="s1"),
        Segment("arc", radius=90, angle_deg=45, name="t2"),
        Segment("line", name="back_straight"),
        Segment("arc", radius=90, angle_deg=60, name="t3"),
        Segment("line", length=120, name="s3"),
        Segment("arc", radius=17, angle_deg=165, name="hairpin"),
        Segment("arc", radius=110, angle_deg=-105, name="compensator"),
        Segment("line", length=120, name="s4"),
        Segment("arc", radius=60, angle_deg=-30, name="esses_a"),
        Segment("line", length=25, name="s5b"),
        Segment("arc", radius=60, angle_deg=30, name="esses_b"),
        Segment("line", length=90, name="s5"),
        Segment("arc", radius=75, angle_deg=120, name="t6"),
    ]


# Segments whose *inside* edge (the side toward the turn centre) gets a kerb.
CIRCUIT_KERB_SEGMENTS = ("hairpin", "compensator", "t3", "t6", "esses_a", "esses_b")


def build_circuit():
    """Returns (segments, samples) with the loop closed and an elevation
    profile applied. Cached on first call."""
    segs = circuit_segments()
    CircuitSolver(segs, {"main_straight", "back_straight"}).solve()
    samples = build_samples(segs, step=CIRCUIT_STEP,
                             x0=CIRCUIT_ORIGIN[0], y0=CIRCUIT_ORIGIN[1], heading0=CIRCUIT_HEADING0)
    total_len = samples[-1].s

    def z_of_s(s):
        return CIRCUIT_ELEV_AMPLITUDE * 0.5 * (1.0 - math.cos(2.0 * math.pi * s / total_len))

    apply_elevation(samples, z_of_s)
    return segs, samples


def min_circuit_radius(segs):
    return min(s.radius for s in segs if s.kind == "arc")
