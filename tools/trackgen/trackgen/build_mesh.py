"""Builds the visual MeshBuilder scene for every meshed test feature: each is
a single connected mesh (see ribbon.py's docstring for why that guarantees no
internal seams). There is no floor mesh any more -- the ground the features
sit on is the procedural terrain (terrain.py), flat (z=0) by construction
under every feature here (see layout.py's flat-zone corridors).
"""
from assetgen.mesh import MeshBuilder

from . import layout as L
from .path import Segment, build_samples, drop_closing_duplicate, full_circle
from .ribbon import ribbon, flat_patch


# ------------------------------------------------------------------ straight
def build_straight(mb):
    s = L.STRAIGHT
    seg = Segment("line", length=s["length"], name="main")
    samples = build_samples([seg], step=0.5, x0=s["x0"], y0=s["y"], heading0=0.0)
    markers = [k * s["marker_every"] for k in range(int(s["length"] / s["marker_every"]) + 1)]
    half_mk = s["marker_width"] / 2.0

    def material_of(sm, t, i, j):
        for ms in markers:
            if abs(sm.s - ms) <= half_mk:
                return "marking"
        return "asphalt"

    return ribbon(mb, samples, s["width"], "straight", material_of, n_lateral=2), samples


# -------------------------------------------------------------------- skidpad
def build_skidpad(mb):
    s = L.SKIDPAD
    samples = full_circle(s["radius"], s["center"], step=0.5, ccw=True, name="skidpad")
    half_line = s["line_width"] / 2.0
    n_lateral = 80

    def material_of(sm, t, i, j):
        for rline in s["radius_lines"]:
            if abs(t - (rline - s["radius"])) <= half_line:
                return "marking"
        return "asphalt"

    recs = ribbon(mb, samples, s["width"], "skidpad", material_of, n_lateral=n_lateral, closed=True)
    return recs, samples


# ------------------------------------------------------------- surface patches
def build_patches(mb):
    g, d, sm = L.GRASS_PATCH, L.DIRT_PATCH, L.SPLIT_MU
    flat_patch(mb, (*g["center"], 0.0), g["length"], g["width"], "grass_patch", "grass")
    flat_patch(mb, (*d["center"], 0.0), d["length"], d["width"], "dirt_patch", "dirt")

    x0 = sm["center"][0] - sm["length"] / 2
    seg = Segment("line", length=sm["length"], name="splitmu")
    samples = build_samples([seg], step=1.0, x0=x0, y0=sm["center"][1], heading0=0.0)

    def material_of(smp, t, i, j):
        return "asphalt" if t >= 0 else "low_mu"

    ribbon(mb, samples, sm["width"], "split_mu", material_of, n_lateral=2)
    return samples


# ------------------------------------------------------------------------ ramps
def _ramp_z(r):
    a0, a1 = r["s_flat0"]
    u0, u1 = r["s_up"]
    p0, p1 = r["s_plateau"]
    d0, d1 = r["s_down"]
    f0, f1 = r["s_flat1"]
    rise = r["rise"]

    def z(s):
        if s <= a1:
            return 0.0
        if s <= u1:
            return rise * (s - u0) / (u1 - u0)
        if s <= p1:
            return rise
        if s <= d1:
            return rise * (1.0 - (s - d0) / (d1 - d0))
        return 0.0
    return z


def build_ramps(mb):
    out = {}
    for r in L.RAMPS:
        seg = Segment("line", length=r["total_len"], name=r["id"])
        samples = build_samples([seg], step=1.0, x0=0.0, y0=r["y"], heading0=0.0)
        zfn = _ramp_z(r)
        for sm in samples:
            sm.z = zfn(sm.s)

        def material_of(sm, t, i, j):
            return "asphalt"

        ribbon(mb, samples, L.RAMP_WIDTH, r["id"], material_of, n_lateral=2)
        out[r["id"]] = samples
    return out


def build_side_slope(mb):
    s = L.SIDE_SLOPE
    total = s["core_len"] + 2 * s["apron"]
    seg = Segment("line", length=total, name="side_slope")
    samples = build_samples([seg], step=1.0, x0=0.0, y0=s["y"], heading0=0.0)

    def material_of(sm, t, i, j):
        return "asphalt"

    # cross_grade only within the core span; flat (0 grade) on the aprons is
    # approximated by using the uniform cross_grade over the whole ribbon --
    # the aprons exist only to give a flat run-up/down in the *travel*
    # direction, the cross-tilt itself is constant across the feature.
    ribbon(mb, samples, s["width"], "side_slope", material_of, n_lateral=2, cross_grade=s["cross_grade"])
    return samples


# ------------------------------------------------------------- long straight
def build_long_straight(mb):
    s = L.LONG_STRAIGHT
    seg = Segment("line", length=s["length"], name="main")
    # Coarser than the short straight's 0.5 m step: dead flat over 10 km, and
    # 5 m divides the 500 m marker spacing exactly, so markers still land on
    # an exact sample row (see build_straight's same reasoning at 0.5 m/100 m).
    samples = build_samples([seg], step=5.0, x0=s["x0"], y0=s["y"], heading0=s["heading_deg"])
    markers = [k * s["marker_every"] for k in range(int(s["length"] / s["marker_every"]) + 1)]
    half_mk = s["marker_width"] / 2.0

    def material_of(sm, t, i, j):
        for ms in markers:
            if abs(sm.s - ms) <= half_mk:
                return "marking"
        return "asphalt"

    ribbon(mb, samples, s["width"], "long_straight", material_of, n_lateral=2)
    return samples


# ------------------------------------------------------------------ airstrip
def build_airstrip(mb):
    s = L.AIRSTRIP
    seg = Segment("line", length=s["length"], name="airstrip")
    samples = build_samples([seg], step=1.0, x0=s["x0"], y0=s["y0"], heading0=s["heading_deg"])
    thresholds = (s["overrun"], s["length"] - s["overrun"])
    half_mk = 1.5

    def material_of(sm, t, i, j):
        for th in thresholds:
            if abs(sm.s - th) <= half_mk:
                return "marking"
        return "asphalt"

    ribbon(mb, samples, s["width"], "airstrip", material_of, n_lateral=2)
    return samples


# --------------------------------------------------------------------- circuit
def build_circuit(mb):
    segs, samples_full = L.build_circuit()
    samples = drop_closing_duplicate(samples_full)
    kerb_ranges = [(s.s0, s.s1) for s in segs if s.name in L.CIRCUIT_KERB_SEGMENTS]
    kerb_side = {s.name: (1.0 if s.angle_deg > 0 else -1.0) for s in segs if s.kind == "arc"}
    seg_of_s = segs
    half = L.CIRCUIT_WIDTH / 2.0
    kw = L.CIRCUIT_KERB_WIDTH
    n_lateral = 12

    def find_segment(s_val):
        for sg in seg_of_s:
            if sg.s0 <= s_val < sg.s1 or (sg is seg_of_s[-1] and s_val >= sg.s0):
                return sg
        return seg_of_s[-1]

    start_finish_half_width = 2.5

    def material_of(sm, t, i, j):
        if sm.s <= start_finish_half_width or sm.s >= samples_full[-1].s - start_finish_half_width:
            return "marking"
        sg = find_segment(sm.s)
        if sg.name in L.CIRCUIT_KERB_SEGMENTS:
            side = kerb_side[sg.name]
            if side > 0 and t >= half - kw:
                return "kerb"
            if side < 0 and t <= -half + kw:
                return "kerb"
        return "asphalt"

    recs = ribbon(mb, samples, L.CIRCUIT_WIDTH, "circuit", material_of, n_lateral=n_lateral, closed=True)
    return recs, samples_full, segs


def build_all():
    mb = MeshBuilder()
    _, straight_samples = build_straight(mb)
    _, skidpad_samples = build_skidpad(mb)
    splitmu_samples = build_patches(mb)
    ramp_samples = build_ramps(mb)
    side_slope_samples = build_side_slope(mb)
    _, circuit_samples, circuit_segs = build_circuit(mb)
    long_straight_samples = build_long_straight(mb)
    airstrip_samples = build_airstrip(mb)
    return mb, dict(straight=straight_samples, skidpad=skidpad_samples, split_mu=splitmu_samples,
                     ramps=ramp_samples, side_slope=side_slope_samples, circuit=circuit_samples,
                     circuit_segments=circuit_segs, long_straight=long_straight_samples,
                     airstrip=airstrip_samples)
