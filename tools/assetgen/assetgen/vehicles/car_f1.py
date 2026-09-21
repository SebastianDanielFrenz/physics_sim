"""d) Formula car: a generic modern open-wheel single-seater (2026-style proportions).

Length 5.48 m (front wing endplate to rear wing endplate), width 1.90 m over
the front tyres, height 0.95 m to the top of the airbox / roll hoop, wheelbase
3.40 m, track 1.60 m front / 1.52 m rear (tyre centres; the rear track is
trimmed from ~1.55 so the 0.38 m rear tyres stay inside the 1.90 m width).
Tyres: 18-inch slicks, front r = 0.36 m w = 0.30 m, rear r = 0.36 m w = 0.38 m,
rim radius 0.2286 m, centre-lock multi-spoke rims. Front kingpin vertical,
0.10 m inboard of the wheel centre plane. Airbox top z = 0.955 m (the T-bar
camera on top reaches 0.986 m).
Ride height at rest: plank bottom z = 0.025 m, floor underside z = 0.035 m.

Interactive: 4 corners (travel, front steer, spin), front wing flap pair
(`front_flap`), rear wing flap (`rear_flap`, DRS / active aero), steering
wheel, lights (rain light, brake strips on the rear wing endplates).
Visual links (`aim`): wishbone legs, push rods, track rods / toe links and
rear drive shafts follow the wheel nodes.
Livery: generic violet / white over bare carbon.
"""
import math

from ..scene import Model
from ..parts import corner, WheelSpec
from .. import mesh as M
from ..vec import Curve, norm, add, sub, mul, cross, frame_z_to, xform_point, compose, translate

WB = 3.40
XF, XR = WB / 2, -WB / 2
TRACK_F, TRACK_R = 1.60, 1.52
R_TYRE = 0.36
KP_OFF = 0.10          # kingpin / ball-joint plane, inboard of the wheel centre plane
FRONT = WheelSpec(radius=0.36, width=0.30, rim_radius=0.2286, tyre="slick", rim="centerlock", spokes=10,
                  rim_color=(0.13, 0.13, 0.14), rim_metallic=0.7, caliper_color=(0.08, 0.08, 0.08))
REAR = WheelSpec(radius=0.36, width=0.38, rim_radius=0.2286, tyre="slick", rim="centerlock", spokes=10,
                 rim_color=(0.13, 0.13, 0.14), rim_metallic=0.7, caliper_color=(0.08, 0.08, 0.08))

# ---------------------------------------------------------------- aero layout
# Wing elements: leading edge (x, z), chord, angle (deg, + = trailing edge up =
# more downforce), span half-extent (|y| range). Sections are inverted NACA 4-digit.
FW_MAIN = dict(le=(2.82, 0.085), chord=0.30, ang=4.0, y=(0.0, 0.90), t=0.10, camber=0.05)
FW_FLAP1 = dict(le=(2.55, 0.128), chord=0.20, ang=20.0, y=(0.13, 0.90), t=0.09, camber=0.06)
FW_FLAP2 = dict(le=(2.39, 0.205), chord=0.15, ang=36.0, y=(0.13, 0.90), t=0.09, camber=0.06)
FW_PIVOT = (2.555, 0.0, 0.135)                 # hinge line of the flap pair (near flap-1 leading edge)
FW_FLAP_RANGE = (-5.0, 25.0)                   # deg, + = flatter (low-drag mode)
RW_MAIN = dict(le=(-2.08, 0.76), chord=0.34, ang=8.0, y=(0.0, 0.50), t=0.11, camber=0.07)
RW_FLAP = dict(le=(-2.385, 0.832), chord=0.22, ang=30.0, y=(0.0, 0.50), t=0.10, camber=0.06)
RW_BEAM = dict(le=(-2.10, 0.445), chord=0.20, ang=10.0, y=(0.0, 0.50), t=0.10, camber=0.05)
RW_FLAP_RANGE = (0.0, 30.0)                    # deg, + = opening (DRS / straight-line mode)


def _te(el):
    a = math.radians(el["ang"])
    return (el["le"][0] - el["chord"] * math.cos(a), el["le"][1] + el["chord"] * math.sin(a))


RW_PIVOT = (_te(RW_FLAP)[0], 0.0, _te(RW_FLAP)[1])   # DRS flap hinges about its trailing edge

# floor: underside height and half width along x (floor + diffuser in one loft)
FLOOR_T = 0.016
FLOOR_Z = Curve([(1.16, 0.15), (1.02, 0.09), (0.80, 0.045), (0.55, 0.035), (-1.22, 0.035), (-1.45, 0.07),
                 (-1.75, 0.15), (-2.02, 0.27)])
FLOOR_HW = Curve([(1.16, 0.36), (1.00, 0.62), (0.70, 0.80), (-1.02, 0.80), (-1.30, 0.56), (-1.38, 0.50),
                  (-2.02, 0.50)])
DIFF_X0, DIFF_X1 = -1.22, -2.02


# ---------------------------------------------------------------- helpers

def tsec(x, hw_top, hw_bot, zb, zt, n=3.0, count=48, yc=0.0, n_bot=None):
    """Tapered superellipse loop in the y-z plane at station x: half width hw_top
    at the top, hw_bot at the bottom (linear in height)."""
    pts = []
    zc, hh = (zb + zt) / 2, (zt - zb) / 2
    for k in range(count):
        t = 2 * math.pi * k / count
        c, s = math.cos(t), math.sin(t)
        e = 2.0 / (n if (s >= 0 or n_bot is None) else n_bot)
        u = math.copysign(abs(c) ** e, c)
        v = math.copysign(abs(s) ** e, s)
        w = hw_bot + (hw_top - hw_bot) * (v + 1) / 2
        pts.append((x, yc + u * w, zc + v * hh))
    return pts


def plate_xz(mb, pts, y0, thick, mat, part=""):
    """Flat plate: polygon given in (x, z), centred on y = y0, thickness along y.
    (Own implementation instead of mesh.extrude: see the note in the module
    footer - extrude can come out inside-out for thin plates away from the origin.)"""
    n = len(pts)
    area = sum(pts[k][0] * pts[(k + 1) % n][1] - pts[(k + 1) % n][0] * pts[k][1] for k in range(n))
    sgn = 1.0 if area > 0 else -1.0
    h = thick / 2
    lo = [(x, y0 - h, z) for (x, z) in pts]
    hi = [(x, y0 + h, z) for (x, z) in pts]
    mb.polygon(hi, mat, part, want_normal=(0.0, 1.0, 0.0))
    mb.polygon(lo, mat, part, want_normal=(0.0, -1.0, 0.0))
    for k in range(n):
        (ax, az), (bx, bz) = pts[k], pts[(k + 1) % n]
        out = (sgn * (bz - az), 0.0, -sgn * (bx - ax))   # outward edge normal in the x-z plane
        mb.polygon([lo[k], lo[(k + 1) % n], hi[(k + 1) % n], hi[k]], mat, part, want_normal=out)


def wing(mb, el, mat, part="", side=0, stations=None, twist=None, rise=None, count=36):
    """Loft an inverted NACA element along y. side = 0: symmetric about y = 0,
    +1 / -1: only that side (|y| from el['y'][0] to el['y'][1]).
    twist(|y|) -> extra angle deg, rise(|y|) -> extra LE height."""
    y0, y1 = el["y"]
    if stations is None:
        stations = [y0 + (y1 - y0) * k / 8 for k in range(9)]
    ys = []
    if side == 0:
        ys = [-s for s in reversed(stations) if s > 0] + ([0.0] if stations[0] <= 0 else []) + [s for s in stations if s > 0]
        ys = sorted(set(round(v, 6) for v in ys))
    else:
        ys = [side * s for s in stations]
    base = M.naca(1.0, el["t"], el["camber"], 0.4, count)
    secs = []
    for y in ys:
        a = math.radians(el["ang"] + (twist(abs(y)) if twist else 0.0))
        cdir = (-math.cos(a), 0.0, math.sin(a))     # along the chord: backwards and up
        ndir = (-math.sin(a), 0.0, -math.cos(a))    # suction side down (inverted wing)
        lx, lz = el["le"]
        lz += rise(abs(y)) if rise else 0.0
        o = (lx, y, lz)
        secs.append([add(o, add(mul(cdir, p * el["chord"]), mul(ndir, q * el["chord"]))) for (p, q) in base])
    return M.loft(mb, secs, mat, part)


def catmull(pts, sub_n):
    """Catmull-Rom spline through pts (end points repeated), sub_n segments per span."""
    P = [pts[0]] + list(pts) + [pts[-1]]
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        for k in range(sub_n):
            t = k / sub_n
            t2, t3 = t * t, t * t * t
            out.append(tuple(0.5 * (2 * p1[c] + (-p0[c] + p2[c]) * t + (2 * p0[c] - 5 * p1[c] + 4 * p2[c] - p3[c]) * t2
                                    + (-p0[c] + 3 * p1[c] - 3 * p2[c] + p3[c]) * t3) for c in range(3)))
    out.append(tuple(pts[-1]))
    return out


def link(model, name, parent, p0, p1, target, radius, mat="susp_carbon", note="", flat=False):
    """Visual link from p0 (node pivot) to p1, kept aimed at p1 carried by `target`."""
    model.node(name, parent, p0, model.aim(target, list(p1), stretch=False, note=note))
    if flat:  # aero-faired arm: flattened ellipse section, wide in the horizontal plane
        d = norm(sub(p1, p0))
        side = norm(cross(d, (0.0, 0.0, 1.0))) if abs(d[2]) < 0.95 else (1.0, 0.0, 0.0)
        up = cross(side, d)
        secs = []
        for p in (p0, p1):
            secs.append([add(p, add(mul(side, radius * 1.9 * math.cos(2 * math.pi * k / 12)),
                                    mul(up, radius * 0.6 * math.sin(2 * math.pi * k / 12)))) for k in range(12)])
        M.loft(model.mb, secs, mat, name)
    else:
        M.tube(model.mb, [p0, p1], radius, mat, name, count=10)
    M.ellipsoid(model.mb, p1, (radius * 1.5,) * 3, "susp_metal", name, count=8, rings=4)


# ---------------------------------------------------------------- build

def build():
    m = Model("car_f1", "Formula car (open-wheel single-seater)",
              "Generic modern open-wheel single-seater, 5.48 m, wheelbase 3.40 m, 18-inch slicks.",
              "Ground plane below the wheel centres (tyres touch z = 0), x = 0 midway between the axles, "
              "y = 0 on the centreline.")
    mb = m.mb
    m.mat("carbon", color=(1, 1, 1), tex=("carbon", {}), tile=0.09, roughness=0.3, double=True)
    m.mat("susp_carbon", color=(1, 1, 1), tex=("carbon", {}), tile=0.05, roughness=0.3)
    m.mat("susp_metal", color=(0.55, 0.56, 0.58), metallic=1.0, roughness=0.35)
    m.mat("paint", color=(1, 1, 1), tex=("paint", {"color": (0.28, 0.08, 0.52), "metallic": 0.4, "rough": 0.2,
                                                   "seed": 41}), tile=0.8, role="paint")
    m.mat("paint_accent", color=(1, 1, 1), tex=("paint", {"color": (0.93, 0.93, 0.95), "rough": 0.25, "seed": 43}),
          tile=0.8, role="paint")
    m.mat("gap", color=(0.0, 0.0, 0.0), roughness=1.0, double=True)
    m.mat("interior", color=(0.05, 0.05, 0.055), roughness=0.85, double=True)
    m.mat("seat_pad", color=(0.09, 0.09, 0.1), roughness=0.9)
    m.mat("plank", color=(0.42, 0.30, 0.18), roughness=0.8)
    m.mat("helmet", color=(0.95, 0.85, 0.12), metallic=0.1, roughness=0.2)
    m.mat("visor", color=(0.02, 0.02, 0.03), metallic=0.6, roughness=0.05)
    m.mat("suit", color=(0.2, 0.06, 0.4), roughness=0.8)
    m.mat("brake_disc", color=(0.12, 0.11, 0.11), metallic=0.2, roughness=0.8)   # carbon-carbon discs
    m.mat("mirror", color=(0.9, 0.9, 0.9), metallic=1.0, roughness=0.02)
    m.mat("display", color=(0.05, 0.2, 0.25), roughness=0.1)
    m.mat("button", color=(0.8, 0.1, 0.1), roughness=0.4)
    m.mat("light_rain", color=(0.45, 0.02, 0.02), roughness=0.15, emissive=(1.0, 0.03, 0.02), light="rain",
          light_strength=10.0)
    m.mat("light_brake", color=(0.5, 0.02, 0.02), roughness=0.15, emissive=(1.0, 0.02, 0.01), light="brake",
          light_strength=8.0)

    # ================================================================ tub + nose
    x_tip = 2.74
    tub_hw = Curve([(2.74, 0.075), (2.40, 0.10), (2.00, 0.14), (1.60, 0.19), (1.20, 0.25), (0.80, 0.33),
                    (0.50, 0.38), (0.0, 0.40), (-0.40, 0.39), (-0.80, 0.30), (-1.10, 0.20)])
    tub_zt = Curve([(2.74, 0.20), (2.40, 0.32), (2.00, 0.46), (1.60, 0.56), (1.20, 0.63), (0.80, 0.67),
                    (0.50, 0.70), (-0.30, 0.70), (-0.55, 0.64), (-0.80, 0.52), (-1.10, 0.40)])
    tub_zb = Curve([(2.74, 0.115), (2.40, 0.15), (2.00, 0.20), (1.60, 0.235), (1.30, 0.235), (1.05, 0.14),
                    (0.85, 0.05), (-1.10, 0.05)])
    tub_n = Curve([(2.74, 2.4), (2.0, 2.8), (1.2, 3.4), (0.5, 4.0), (-1.1, 4.0)])
    tip_r = 0.06

    # cockpit opening (plan view superellipse) and its well
    CK_XM, CK_AX, CK_AY = 0.10, 0.42, 0.235

    def ck_hw(x):
        return CK_AY - 0.07 * max(0.0, (x - CK_XM) / CK_AX)

    def in_cockpit(x, y, pad=0.0):
        t = (x - CK_XM) / (CK_AX + pad)
        if abs(t) >= 1:
            return False
        return abs(y) < (ck_hw(x) + pad) * (1 - abs(t) ** 3) ** (1 / 3)

    xs = []
    x = x_tip
    while x > -1.10:
        xs.append(x)
        if x > x_tip - tip_r:
            x -= 0.01
        elif -0.40 < x < 0.60:
            x -= 0.02
        else:
            x -= 0.06
    xs.append(-1.10)
    secs = []
    for x in xs:
        f = 1.0
        if x > x_tip - tip_r:
            f = math.sqrt(max(0.0, 1 - ((x - (x_tip - tip_r)) / tip_r) ** 2))
            f = max(f, 0.08)
        hw, zt, zb = tub_hw(x), tub_zt(x), tub_zb(x)
        zc = (zt + zb) / 2
        hh = (zt - zb) / 2 * f
        # nose tip: keep the underside flat (it sits on the main plane)
        zb2, zt2 = (zb, zb + 2 * hh) if x > x_tip - tip_r else (zc - hh, zc + hh)
        secs.append(tsec(x, hw * f * 0.8, hw * f, zb2, zt2, n=tub_n(x), n_bot=2.6, count=56))

    def tub_cls(i, j, c, n):
        # livery bands in loop parameter space (clean longitudinal edges); j = loop index
        x, y, z = c
        if n[2] > 0.25 and z > 0.55 and in_cockpit(x, y):
            return None
        t = 2 * math.pi * (j + 0.5) / 56
        if math.sin(t) < -0.25:
            return "carbon"
        if x > 2.44:
            return "paint_accent"
        if abs(math.cos(t)) < 0.1 and math.sin(t) > 0:
            return "paint_accent"
        return "paint"
    M.loft(mb, secs, "paint", classify=tub_cls, cap_material="carbon")

    # cockpit well + coaming
    loop = []
    for k in range(48):
        t = 2 * math.pi * k / 48
        c, s = math.cos(t), math.sin(t)
        x = CK_XM + CK_AX * math.copysign(abs(c) ** (2 / 3), c)
        y = ck_hw(x) * math.copysign(abs(s) ** (2 / 3), s)
        loop.append((x, y))
    top = [(x, y, tub_zt(x) - 0.004) for (x, y) in loop]
    bot = [(CK_XM + (x - CK_XM) * 0.9, y * 0.9, 0.30) for (x, y) in loop]
    M.loft(mb, [top, bot], "interior", cap_start=False, cap_end=True, auto_orient=False)
    rim = [(x + 0.012 * math.copysign(1, x - CK_XM) * abs(x - CK_XM) / CK_AX, y * 1.04, tub_zt(x) - 0.012)
           for (x, y) in loop]
    M.tube(mb, rim, 0.028, "seat_pad", count=10, closed=True)

    # driver: helmet, visor, shoulders, headrest pads
    hx, hz = -0.04, 0.785
    M.ellipsoid(mb, (hx, 0.0, hz), (0.145, 0.12, 0.13), "helmet", count=28, rings=14)
    M.ellipsoid(mb, (hx + 0.075, 0.0, hz + 0.015), (0.085, 0.105, 0.04), "visor", count=24, rings=8)
    M.ellipsoid(mb, (hx + 0.02, 0.0, 0.60), (0.22, 0.21, 0.10), "suit", count=20, rings=8)
    for s in (1, -1):
        M.rounded_box(mb, (hx - 0.03, s * 0.175, 0.705), (0.30, 0.07, 0.12), 0.03, "seat_pad")
    M.rounded_box(mb, (-0.235, 0.0, 0.71), (0.08, 0.30, 0.13), 0.03, "seat_pad")

    # steering wheel (small rectangular, +-180 deg)
    sw_c = (0.36, 0.0, 0.645)
    col = norm((1.0, 0.0, -0.22))            # column direction: from the wheel into the dash
    axis = mul(col, -1)
    m.node("steering_wheel", None, sw_c, m.revolute(axis, -180, 180, "steering wheel angle, + = left"))
    f = frame_z_to(axis, sw_c)               # local x = lateral (-y), local y = up, z = towards driver
    M.rounded_box(mb, (0, 0, 0), (0.13, 0.12, 0.035), 0.012, "interior", "steering_wheel", xf=f)
    for s in (1, -1):  # grips
        g = compose(f, translate((s * 0.12, -0.005, 0.0)))
        M.rounded_box(mb, (0, 0, 0), (0.05, 0.13, 0.045), 0.02, "seat_pad", "steering_wheel", xf=g)
    M.box(mb, (0, 0.012, 0.019), (0.09, 0.05, 0.004), "display", "steering_wheel", xf=f)
    for (bx, by) in ((-0.045, -0.035), (0.045, -0.035), (-0.05, 0.045), (0.05, 0.045)):
        M.cylinder(mb, xform_point(f, (bx, by, 0.017)), xform_point(f, (bx, by, 0.025)), 0.008, "button",
                   "steering_wheel", count=8)
    M.cylinder(mb, xform_point(f, (0, 0, -0.02)), add(sw_c, mul(col, 0.28)), 0.018, "interior", count=10)

    # halo (tube): centre pillar + horseshoe, smoothed through the key points
    pillar = catmull([(0.68, 0.0, 0.64), (0.62, 0.0, 0.76), (0.54, 0.0, 0.85), (0.47, 0.0, 0.89)], 4)
    M.tube(mb, pillar, 0.03, "paint", count=14)
    for s in (1, -1):
        path = catmull([(0.47, 0.0, 0.89), (0.40, s * 0.13, 0.90), (0.25, s * 0.235, 0.905),
                        (0.05, s * 0.285, 0.905), (-0.13, s * 0.30, 0.885), (-0.25, s * 0.305, 0.83),
                        (-0.30, s * 0.31, 0.75), (-0.31, s * 0.315, 0.64)], 5)
        M.tube(mb, path, 0.027, "paint", count=14)

    # ================================================================ airbox / engine cover
    ec = [(-0.25, 0.10, 0.12, 0.70, 0.92), (-0.26, 0.11, 0.13, 0.69, 0.93), (-0.30, 0.12, 0.16, 0.58, 0.945),
          (-0.40, 0.13, 0.24, 0.30, 0.955), (-0.55, 0.13, 0.33, 0.06, 0.95), (-0.75, 0.125, 0.35, 0.06, 0.91),
          (-1.00, 0.12, 0.32, 0.06, 0.83), (-1.25, 0.11, 0.28, 0.07, 0.72), (-1.50, 0.10, 0.24, 0.09, 0.62),
          (-1.75, 0.09, 0.19, 0.13, 0.53), (-1.95, 0.08, 0.15, 0.18, 0.47), (-2.10, 0.07, 0.11, 0.22, 0.43),
          (-2.22, 0.065, 0.09, 0.24, 0.40)]
    secs = [tsec(x, ht, hb, zb, zt, n=3.2, n_bot=3.5, count=48) for (x, ht, hb, zb, zt) in ec]
    EC_ZT = Curve([(e[0], e[4]) for e in ec])

    def ec_cls(i, j, c, n):
        t = 2 * math.pi * (j + 0.5) / 48
        if math.sin(t) < -0.3:
            return "carbon"
        return "paint"
    M.loft(mb, secs, "paint", classify=ec_cls, cap_material="paint")
    # airbox inlet: black opening on the front face, and a roll-structure lip
    inlet = [(-0.2485, 0.07 * math.cos(2 * math.pi * k / 24), 0.855 + 0.052 * math.sin(2 * math.pi * k / 24))
             for k in range(24)]
    mb.polygon(inlet, "gap", want_normal=(1, 0, 0))
    # T-bar camera on top of the roll hoop
    M.rounded_box(mb, (-0.40, 0.0, 0.975), (0.045, 0.20, 0.022), 0.008, "paint_accent")
    M.box(mb, (-0.40, 0.0, 0.955), (0.03, 0.02, 0.03), "carbon")
    # shark fin
    fin = [(-0.62, 0.93), (-1.00, 0.89), (-1.50, 0.78), (-1.86, 0.66), (-1.86, 0.49), (-1.50, 0.60),
           (-1.00, 0.80)]
    plate_xz(mb, fin, 0.0, 0.008, "paint")

    # rain light on the rear crash structure
    M.rounded_box(mb, (-2.222, 0.0, 0.32), (0.012, 0.12, 0.05), 0.004, "light_rain")
    M.box(mb, (-2.216, 0.0, 0.32), (0.006, 0.14, 0.07), "carbon")

    # ================================================================ sidepods
    sp = [  # x, yc, a (half width), zb, zt, bottom-width factor
        (0.47, 0.50, 0.19, 0.24, 0.53, 0.60), (0.44, 0.50, 0.205, 0.215, 0.555, 0.62),
        (0.30, 0.50, 0.22, 0.17, 0.57, 0.62), (0.00, 0.49, 0.22, 0.12, 0.56, 0.58),
        (-0.40, 0.46, 0.21, 0.09, 0.53, 0.55), (-0.80, 0.39, 0.17, 0.07, 0.45, 0.60),
        (-1.10, 0.32, 0.12, 0.07, 0.36, 0.7), (-1.40, 0.24, 0.08, 0.08, 0.26, 0.8)]
    for s in (1, -1):
        secs = []
        for (x, yc, a, zb, zt, bf) in sp:
            sec = tsec(x, a, a * bf, zb, zt, n=4.5, n_bot=3.0, count=40, yc=yc)
            secs.append([(p[0], s * p[1], p[2]) for p in sec])

        def sp_cls(i, j, c, n, s=s):
            k = j if s > 0 else 40 - 2 - j      # the right-hand loops get reversed by loft()
            t = 2 * math.pi * (k + 0.5) / 40
            if math.sin(t) < -0.2:
                return "carbon"
            if 0.3 < math.sin(t) < 0.45 and math.cos(t) > 0:
                return "paint_accent"
            return "paint"
        M.loft(mb, secs, "paint", classify=sp_cls, cap_material="carbon")
        c0 = secs[0]
        ctr = M.centroid(c0)
        mb.polygon([add(ctr, mul(sub(p, ctr), 0.88)) for p in c0], "gap", want_normal=(1, 0, 0))
        mb.polygon([add((0.0015, 0, 0), add(ctr, mul(sub(p, ctr), 0.88))) for p in c0], "gap",
                   want_normal=(1, 0, 0))

        # mirrors on stalks from the tub shoulder
        root = (0.46, s * 0.33, 0.66)
        head = (0.44, s * 0.52, 0.735)
        M.tube(mb, [root, (0.45, s * 0.44, 0.71), head], 0.011, "carbon", count=8)
        M.rounded_box(mb, (head[0], s * 0.555, head[2]), (0.07, 0.15, 0.07), 0.025, "paint")
        M.box(mb, (head[0] - 0.036, s * 0.555, head[2]), (0.004, 0.13, 0.055), "mirror")

    # ================================================================ floor, plank, diffuser
    fx = [1.16, 1.12, 1.06, 1.0, 0.9, 0.8, 0.7, 0.55, 0.2, -0.4, -1.02, -1.15, -1.22, -1.30, -1.38, -1.45, -1.55,
          -1.65, -1.75, -1.85, -1.95, -2.02]
    secs = []
    for x in fx:
        hw, z = FLOOR_HW(x), FLOOR_Z(x)
        sec = M.rrect(2 * hw, FLOOR_T, 0.0075, 2)
        secs.append(M.place2d(sec, (x, 0.0, z + FLOOR_T / 2), (0, 1, 0), (0, 0, 1)))
    M.loft(mb, secs, "carbon")
    M.box(mb, (-0.30, 0.0, 0.03), (1.70, 0.30, 0.01), "plank")   # under the flat floor section only
    # floor edges: upturned outer lip
    for s in (1, -1):
        edge = [(0.70, 0.05), (0.70, 0.085), (0.2, 0.095), (-1.0, 0.095), (-1.0, 0.05)]
        plate_xz(mb, edge, s * 0.795, 0.01, "carbon")
        # floor-front fences under the raised leading edge
        for yy in (0.18, 0.30, 0.42):
            fence = [(1.15, FLOOR_Z(1.15)), (0.72, FLOOR_Z(0.72)), (0.72, 0.035), (0.95, 0.05)]
            plate_xz(mb, fence, s * yy, 0.008, "carbon")
    # diffuser fences / strakes: from the diffuser roof down to the plank level
    for yy in (-0.495, -0.26, 0.0, 0.26, 0.495):
        pts = []
        for k in range(9):
            x = DIFF_X0 - 0.02 - (DIFF_X1 + 0.0 - DIFF_X0 + 0.0) * -k / 8
            pts.append((x, FLOOR_Z(x) + 0.004))
        pts += [(DIFF_X1, 0.05 if abs(yy) > 0.4 else 0.09), (DIFF_X0 - 0.1, 0.035)]
        plate_xz(mb, pts, yy, 0.008, "carbon")

    # ================================================================ front wing
    wing(mb, FW_MAIN, "carbon", stations=[0.0, 0.12, 0.25, 0.4, 0.55, 0.7, 0.8, 0.90],
         twist=lambda y: 3.0 * max(0.0, y - 0.3) / 0.6, rise=lambda y: 0.02 * max(0.0, y - 0.5) / 0.4)
    m.node("front_flap", None, FW_PIVOT,
           m.revolute((0, -1, 0), FW_FLAP_RANGE[0], FW_FLAP_RANGE[1],
                      "front wing flap pair angle about the hinge line, + = flatter (low drag)"))
    for s in (1, -1):
        wing(mb, FW_FLAP1, "carbon", "front_flap", side=s, twist=lambda y: 4.0 * max(0.0, y - 0.4) / 0.5,
             rise=lambda y: 0.012 * max(0.0, y - 0.5) / 0.4)
        wing(mb, FW_FLAP2, "paint_accent", "front_flap", side=s, twist=lambda y: 4.0 * max(0.0, y - 0.4) / 0.5,
             rise=lambda y: 0.02 * max(0.0, y - 0.5) / 0.4)
        # endplates
        ep = [(2.855, 0.055), (2.80, 0.045), (2.22, 0.045), (2.18, 0.10), (2.18, 0.33), (2.25, 0.36),
              (2.45, 0.34), (2.80, 0.21), (2.86, 0.12)]
        plate_xz(mb, ep, s * 0.901, 0.012, "paint")
        # footplate / lower tunnel strip
        plate_xz(mb, [(2.84, 0.045), (2.20, 0.045), (2.20, 0.055), (2.84, 0.055)], s * 0.87, 0.06, "carbon")

    # ================================================================ rear wing
    wing(mb, RW_MAIN, "carbon")
    wing(mb, RW_BEAM, "carbon")
    m.node("rear_flap", None, RW_PIVOT,
           m.revolute((0, -1, 0), RW_FLAP_RANGE[0], RW_FLAP_RANGE[1],
                      "rear wing flap angle about its trailing edge, + = opening (DRS / low drag)"))
    wing(mb, RW_FLAP, "paint_accent", "rear_flap")
    for s in (1, -1):
        ep = [(-2.03, 0.40), (-2.03, 0.72), (-2.07, 0.86), (-2.16, 0.935), (-2.30, 0.965), (-2.64, 0.965),
              (-2.64, 0.40), (-2.35, 0.37)]
        plate_xz(mb, ep, s * 0.502, 0.014, "paint")
        # brake light strip on the endplate trailing edge
        M.box(mb, (-2.644, s * 0.502, 0.72), (0.008, 0.012, 0.36), "light_brake")
    # pylon (central swan neck) from the crash structure to the main plane
    pyl = [(-2.02, 0.40), (-2.12, 0.52), (-2.16, 0.70), (-2.20, 0.795), (-2.34, 0.815), (-2.32, 0.70),
           (-2.24, 0.52), (-2.18, 0.39)]
    plate_xz(mb, pyl, 0.0, 0.016, "carbon")

    # ================================================================ corners + suspension
    ZB = R_TYRE
    corners = (("FL", XF, TRACK_F / 2, True, FRONT), ("FR", XF, -TRACK_F / 2, True, FRONT),
               ("RL", XR, TRACK_R / 2, False, REAR), ("RR", XR, -TRACK_R / 2, False, REAR))
    for key, x, y, steer, spec in corners:
        corner(m, key, (x, y, ZB), spec, steer=steer, steer_limits=(-22, 22),
               travel=(-0.04, 0.03) if steer else (-0.045, 0.035))
        if steer:
            # parts.corner puts the steer axis through the wheel centre. Move it to
            # the real kingpin: vertical through the upright ball joints, KP_OFF
            # inboard of the wheel centre plane (geometry is authored in model
            # coordinates, so only the rotation axis moves). Ball joints then stay
            # put while steering, as on the real car.
            m.nodes[f"steer_{key}"].pivot = (x, math.copysign(abs(y) - KP_OFF, y), ZB)

    # Suspension links are `aim` nodes (visual only, stretch off): a link keeps
    # its rest length and turns so that its rest direction follows the moving
    # end. Alternatives considered: parenting the links to the chassis (they
    # would not follow the wheel) or to the upright (they would detach from the
    # chassis), and stretch=True (the aim convention scales along the node's +x
    # axis, which only matches the link when it lies along +x at rest - not
    # the case for any of these links, and node rest rotations must stay
    # identity). With fixed-length links the question is only which end may
    # drift, so each link is anchored at the end where a drift would be seen:
    # - wishbone legs, rear toe links, drive shafts: pivot at the chassis
    #   pick-up, aimed at the ball joint carried by steer_XX (front; the ball
    #   joints lie on the kingpin, so steering does not move them) or susp_XX
    #   (rear). Near-horizontal, so the length error over full travel is
    #   < 7 mm and the end stays inside the upright.
    # - push rods and front track rods: parented to the upright (pivot at the
    #   outboard end) and aimed at the rocker / rack end carried by the chassis
    #   root. They are steep (push rods) or follow the rack, which is not
    #   modelled (track rods), so their length error reaches ~25 / ~45 mm;
    #   the inboard ends sit 7-12 cm deep inside the monocoque / gearbox
    #   bodywork, which hides it.
    for key, x, y, steer, spec in corners:
        s = 1 if y > 0 else -1
        front = steer
        carrier = f"steer_{key}" if steer else f"susp_{key}"
        yb = abs(y) - KP_OFF                     # ball-joint plane (kingpin), inside the rim
        if front:
            ub, lb = (x, s * yb, 0.525), (x, s * yb, 0.205)
            u_in = ((x + 0.25, s * 0.10, 0.44), (x - 0.27, s * 0.16, 0.52))
            l_in = ((x + 0.32, s * 0.09, 0.25), (x - 0.30, s * 0.15, 0.27))
            tie_in, tie_out = (x + 0.12, s * 0.05, 0.39), (x + 0.12, s * (yb + 0.005), 0.37)
            pr_out, pr_in = (x - 0.02, s * (yb - 0.02), 0.235), (x - 0.10, s * 0.07, 0.50)
        else:
            ub, lb = (x, s * yb, 0.525), (x, s * yb, 0.20)
            u_in = ((x + 0.25, s * 0.14, 0.52), (x - 0.25, s * 0.07, 0.42))
            l_in = ((x + 0.30, s * 0.20, 0.15), (x - 0.25, s * 0.09, 0.21))
            tie_in, tie_out = (x - 0.25, s * 0.09, 0.33), (x - 0.14, s * (yb + 0.005), 0.33)
            pr_out, pr_in = (x - 0.02, s * (yb - 0.03), 0.23), (x + 0.10, s * 0.06, 0.49)
        for nm, p_in, p_out in (("upper_front", u_in[0], ub), ("upper_rear", u_in[1], ub),
                                ("lower_front", l_in[0], lb), ("lower_rear", l_in[1], lb)):
            link(m, f"wishbone_{nm}_{key}", None, p_in, p_out, carrier, 0.012,
                 note="wishbone leg (visual): chassis pick-up -> upright ball joint", flat=True)
        if front:  # anchored on the steering arm (the rack end would slide), aimed at the rack inside the tub
            link(m, f"trackrod_{key}", carrier, tie_out, tie_in, m.id, 0.011,
                 note="steering track rod (visual): steering arm -> rack end inside the chassis", flat=True)
        else:
            link(m, f"toelink_{key}", None, tie_in, tie_out, carrier, 0.011, note="rear toe link (visual)",
                 flat=True)
        link(m, f"pushrod_{key}", carrier, pr_out, pr_in, m.id, 0.013,
             note="push rod (visual): upright -> rocker inside the chassis")
        if not front:
            dp0, dp1 = (x, s * 0.12, ZB), (x, s * (yb - 0.035), ZB)
            m.node(f"driveshaft_{key}", None, dp0, m.aim(carrier, list(dp1), stretch=False,
                                                         note="drive shaft (visual): gearbox -> hub"))
            M.tube(mb, [dp0, dp1], 0.024, "susp_metal", f"driveshaft_{key}", count=12)
            M.ellipsoid(mb, dp1, (0.035, 0.04, 0.04), "susp_carbon", f"driveshaft_{key}", count=12, rings=6)
        # upright (static on the carrier) with steering arm / toe-link lug
        ux = x - 0.01
        M.rounded_box(mb, (ux, s * (yb + 0.02), ZB), (0.13, 0.055, 0.38), 0.02, "susp_carbon", carrier)
        lug_x = tie_out[0]
        M.rounded_box(mb, ((ux + lug_x) / 2, s * (yb + 0.01), tie_out[2]), (abs(lug_x - ux) + 0.04, 0.035, 0.04),
                      0.01, "susp_carbon", carrier)
        M.rounded_box(mb, (pr_out[0], s * (yb - 0.005), pr_out[2]), (0.05, 0.05, 0.05), 0.01, "susp_carbon", carrier)
        # brake duct drum on the inboard face of the upright
        M.cylinder(mb, (x + 0.02, s * (yb - 0.005), ZB + 0.02), (x + 0.02, s * (yb - 0.045), ZB + 0.02), 0.07,
                   "carbon", carrier, count=16)

    # ================================================================ sockets
    m.socket("socket_driver_eye", None, (hx + 0.07, 0.0, hz + 0.02), "driver eye point (camera)")
    m.socket("socket_camera_onboard", None, (-0.40, 0.0, 0.99), "onboard T-bar camera on the roll hoop")
    m.socket("socket_jack_front", None, (2.74, 0.0, 0.12), "front jack point (nose tip, on the main plane)")
    m.socket("socket_jack_rear", None, (-2.23, 0.0, 0.30), "rear jack point (rear crash structure)")
    m.socket("socket_light_rear", None, (-2.23, 0.0, 0.32), "rear rain light")

    # ================================================================ physics data
    def el_info(el, span):
        te = _te(el)
        return {"le_x": el["le"][0], "le_z": el["le"][1], "te_x": round(te[0], 4), "te_z": round(te[1], 4),
                "chord": el["chord"], "span": round(span, 3), "angle_deg": el["ang"],
                "section": f"NACA {int(el['camber'] * 100)}4{int(el['t'] * 100):02d}, inverted"}
    fw_flap_span = 2 * (FW_FLAP1["y"][1] - FW_FLAP1["y"][0])
    m.extra_rig["aero"] = {
        "frame": "ISO model frame at rest; angle_deg = element chord angle, + = trailing edge up",
        "front_wing": {
            "main_plane": el_info(FW_MAIN, 2 * FW_MAIN["y"][1]),
            "flap_1": el_info(FW_FLAP1, fw_flap_span),
            "flap_2": el_info(FW_FLAP2, fw_flap_span),
            "flap_node": "front_flap", "pivot": list(FW_PIVOT), "axis": [0, -1, 0],
            "flap_range_deg": list(FW_FLAP_RANGE),
            "note": "flaps span 0.13 < |y| < 0.895 (the nose sits on the main plane); outer 0.4 m of each "
                    "element twisted up to +4 deg; endplates at |y| = 0.90, z 0.045..0.36",
        },
        "rear_wing": {
            "main_plane": el_info(RW_MAIN, 2 * RW_MAIN["y"][1]),
            "flap": el_info(RW_FLAP, 2 * RW_FLAP["y"][1]),
            "beam_wing": el_info(RW_BEAM, 2 * RW_BEAM["y"][1]),
            "flap_node": "rear_flap", "pivot": list(RW_PIVOT), "axis": [0, -1, 0],
            "flap_range_deg": list(RW_FLAP_RANGE),
            "note": "endplates at |y| = 0.50, x -2.03..-2.64, z 0.37..0.965",
        },
        "floor": {
            "x_front": 1.16, "x_diffuser_start": DIFF_X0, "x_diffuser_exit": DIFF_X1,
            "underside_z_flat": 0.035, "plank_bottom_z": 0.025, "half_width_max": 0.80,
            "diffuser_half_width": 0.50, "diffuser_exit_roof_z": 0.27,
            "leading_edge_z": 0.15,
            "half_width_profile": [[k[0], k[1]] for k in zip(FLOOR_HW.xs, FLOOR_HW.ys)],
            "underside_z_profile": [[k[0], k[1]] for k in zip(FLOOR_Z.xs, FLOOR_Z.ys)],
        },
    }
    m.notes += [
        "Generic open-wheel single-seater with 2026-style proportions; no real team, livery or sponsor.",
        "Front wing: main plane chord 0.30 m, span 1.79 m; flap 1 chord 0.20 m and flap 2 chord 0.15 m, "
        "each 2 x 0.765 m (|y| 0.13..0.895). Node front_flap carries both flaps and rotates about the hinge "
        f"line through {FW_PIVOT} along -y: + = flatter / low drag, range {FW_FLAP_RANGE} deg.",
        "Rear wing: main plane chord 0.34 m, flap chord 0.22 m, beam wing chord 0.20 m, all span 0.99 m. "
        f"Node rear_flap rotates about the flap trailing edge {tuple(round(v, 3) for v in RW_PIVOT)} along -y: "
        f"+ = opening (DRS), range {RW_FLAP_RANGE} deg.",
        "Floor from x = 1.16 (raised leading edge, z 0.15) to the diffuser exit at x = -2.02; flat underside "
        "z = 0.035 between x = 0.55 and -1.22, plank bottom z = 0.025 (x 0.55..-1.15, 0.30 m wide), diffuser "
        "roof rises to z = 0.27 at the exit, half width 0.50. Full aero geometry under rig 'aero'.",
        "Suspension travel: front -0.040..+0.030 m, rear -0.045..+0.035 m; at about +0.025 m bump the plank "
        "reaches the ground (the chassis bottoms out, as intended for a ground-effect car).",
        "Steering wheel +-180 deg vs road-wheel steer +-22 deg (overall ratio ~8.2:1). Kingpin: vertical "
        f"(no inclination / caster) through the upright ball joints, {KP_OFF} m inboard of the wheel centre "
        "plane (steer_XX pivot), so the front wheels swing about it when steered.",
        "Suspension links (wishbone legs, track rods, toe links, push rods, drive shafts) are visual `aim` "
        "nodes with stretch off; the sim should model the suspension from the susp_/steer_ joints, not these.",
        "Lights: 'rain' = rear rain light on the crash structure; 'brake' = LED strips on the rear wing "
        "endplate trailing edges.",
    ]
    return m
