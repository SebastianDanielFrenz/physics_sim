"""g) Combine harvester, class 8, with a 9.0 m grain header (generic design).

Key dimensions (ISO 8855 frame, origin on the ground midway between the axles):
- length 8.84 m without header (x -4.44 hitch .. +4.40 feeder face),
  11.32 m with the header (divider points at x +6.88),
- width 3.52 m over the front tyres; header cut width 9.0 m, 9.11 m over the
  end sheets, 9.27 m over the crop dividers; folded unloading auger and its
  cradle stick out to y +1.84,
- height 3.99 m (cab roof 3.84 m + beacon, folded unloading auger 3.975 m),
- wheelbase 3.90 m: front drive axle x = +1.95, rear steer axle x = -1.95,
- front drive tyres 900/60 R32 (r 0.955, w 0.90, rim r 0.406) at y = +-1.30,
- rear steer tyres 600/65 R28 (r 0.745, w 0.60, rim r 0.356) at y = +-1.20,
- unloading auger: tube diameter 0.35 m, 6.20 m from the swing pivot
  (x -1.55, y +1.62, z 3.80) to the tip; folded forward along the left side,
  swings out to the left by up to 100 deg,
- header: floor/cutter bar at z 0.15-0.19 at rest (skids 0.15 m clear),
  cross auger dia 0.60 m, reel dia 1.34 m over the tines.
Interactive: feeder house (header lift) -> header (lateral tilt) -> cross
auger, reel arms (lift) -> reel fore/aft -> reel; unloading auger swing +
spout tilt; grain tank covers L/R; cab door; folding ladder; steering wheel;
straw chopper; spreaders L/R; front drive wheels (rigid axle); rear pendulum
axle with steered wheels; hydraulic rams as aim links; lamps (head, work,
beacon, tail, brake, indicators, reverse).
"""
import math

from ..scene import Model
from ..parts import standard_materials, corner, WheelSpec, steering_wheel, seat, mirror
from .. import mesh as M
from ..vec import (add, sub, mul, norm, length, dist, compose, translate, rot_axis, rot_y, rot_z,
                   xform_point, frame_z_to, IDENTITY)

WB = 3.90
XF, XR = WB / 2, -WB / 2
YF, YR = 1.30, 1.20
FRONT = WheelSpec(radius=0.955, width=0.90, rim_radius=0.406, tyre="farm", rim="farm",
                  rim_color=(0.70, 0.70, 0.68), rim_metallic=0.5, disc=False, lugs=24, lug_height=0.06)
REAR = WheelSpec(radius=0.745, width=0.60, rim_radius=0.356, tyre="farm", rim="farm",
                 rim_color=(0.70, 0.70, 0.68), rim_metallic=0.5, disc=False, lugs=20, lug_height=0.05)

BODY = (0.23, 0.29, 0.35)   # slate blue-grey (generic, not a brand colour)

# unloading auger
AUG_PIVOT = (-1.55, 1.62, 3.80)
AUG_LEN = 6.20
AUG_R = 0.175

# feeder house / header
FEEDER_PIVOT = (2.40, 0.0, 1.62)
FEEDER_FACE = (4.40, 0.0, 0.80)
AUGER_C = (4.95, 0.0, 0.58)
REEL_PIVOT = (4.62, 0.0, 1.55)
REEL_DIR = norm((1.33, 0.0, -0.45))
REEL_DIST = 1.40
REEL_AXIS = add(REEL_PIVOT, mul(REEL_DIR, REEL_DIST))
Y_END = 4.556            # end sheet planes
Y_ARM = 4.45             # outer reel arms


# ------------------------------------------------------------------ helpers

def xz_frame(y):
    """extrude(): local (a, b) -> (x, z), extrusion depth along y, centred at y."""
    return ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, -1.0, y), (0.0, 1.0, 0.0, 0.0))


def yz_frame(x):
    """extrude(): local (a, b) -> (y, z), extrusion depth along x, centred at x."""
    return ((0.0, 0.0, 1.0, x), (1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0))


def about(axis, ang, c):
    return compose(translate(c), rot_axis(axis, ang), translate(mul(c, -1)))


def box(mb, part, mat, x0, x1, y0, y1, z0, z1, xf=IDENTITY):
    M.box(mb, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), mat, part, xf=xf)


def rbox(mb, part, mat, x0, x1, y0, y1, z0, z1, r, classify=None, seg=3, xf=IDENTITY):
    M.rounded_box(mb, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), r, mat,
                  part, seg=seg, classify=classify, xf=xf)


def dirty_below(zlim, clean="paint", dirty="paint_dirty"):
    def f(i, j, c, n):
        return dirty if c[2] < zlim else clean
    return f


def beam(mb, part, mat, p0, p1, w, h):
    """Rectangular beam from p0 to p1 (both in an x-z plane, same y): w across y, h in the x-z plane."""
    d = sub(p1, p0)
    L = length(d)
    g = math.atan2(-d[2], d[0])
    mid = mul(add(p0, p1), 0.5)
    M.rounded_box(mb, (0, 0, 0), (L, w, h), min(w, h) * 0.2, mat, part, seg=1,
                  xf=compose(translate(mid), rot_y(g)))


def lamp(mb, part, center, size, axis, sign, lens_mat, housing="trim_black"):
    """Box lamp housing with a lens on the face pointing along +-axis (0=x, 1=y, 2=z)."""
    M.box(mb, center, size, housing, part)
    c = list(center)
    c[axis] += sign * (size[axis] / 2 + 0.004)
    s = [v * 0.82 for v in size]
    s[axis] = 0.008
    M.box(mb, tuple(c), tuple(s), lens_mat, part)


def hyd_cylinder(m, name, base_parent, B, rod_parent, E, d_range, r=0.06):
    """Hydraulic ram as two aim links: barrel (on base_parent, pivot B) and rod
    (on rod_parent, pivot E). d_range = (min, max) pin distance over the joint range."""
    mb = m.mb
    dmin, dmax = d_range
    Lr = dmin - 0.08
    Lb = dmin - 0.05
    assert Lb + Lr >= dmax + 0.04, (name, dmin, dmax)
    bn = m.node(name + "_barrel", base_parent, B, m.aim(name + "_rod", E, note="ram barrel, aims at the rod eye"))
    rn = m.node(name + "_rod", rod_parent, E, m.aim(name + "_barrel", B, note="ram rod, aims at the barrel eye"))
    u = norm(sub(E, B))
    M.cylinder(mb, B, add(B, mul(u, Lb)), r, "chassis", bn, count=12)
    M.cylinder(mb, add(B, mul(u, Lb - 0.06)), add(B, mul(u, Lb)), r * 1.15, "chassis", bn, count=12)
    M.cylinder(mb, (B[0], B[1] - r, B[2]), (B[0], B[1] + r, B[2]), r * 0.9, "chassis", bn, count=10)
    M.cylinder(mb, E, sub(E, mul(u, Lr)), r * 0.55, "chrome", rn, count=10)
    M.cylinder(mb, (E[0], E[1] - r * 0.8, E[2]), (E[0], E[1] + r * 0.8, E[2]), r * 0.75, "chassis", rn, count=10)
    return bn, rn


def pin_distance(B, E, pivot, axis, angles):
    """Min/max distance B..E when E rotates about pivot/axis over angles (rad)."""
    ds = []
    for a in angles:
        e = xform_point(about(axis, a, pivot), E)
        ds.append(dist(B, e))
    return min(ds), max(ds)


# ------------------------------------------------------------------ build

def build():
    m = Model("combine", "Combine harvester (class 8, 9 m header)",
              "Class-8 combine harvester with a 9.0 m grain header: 900/60 R32 front drive tyres, "
              "600/65 R28 steered rear tyres on a pendulum axle, ~12,000 l grain tank, 6.2 m unloading auger.",
              "Ground plane below the tyres at rest, x = 0 midway between the front drive axle and the rear "
              "steer axle, y = 0 on the centreline.")
    standard_materials(m)
    m.mat("paint", color=(1, 1, 1), tex=("paint", {"color": BODY, "dirt": 0.12, "scratches": 0.0, "metallic": 0.25,
                                                   "rough": 0.38, "seed": 31}), tile=1.6, double=True, role="paint")
    m.mat("paint_dirty", color=(1, 1, 1), tex=("paint", {"color": BODY, "dirt": 0.8, "scratches": 0.1,
                                                         "metallic": 0.1, "rough": 0.5, "seed": 32}),
          tile=1.2, double=True)
    m.mat("paint_roof", color=(1, 1, 1), tex=("paint", {"color": (0.86, 0.86, 0.84), "dirt": 0.1, "rough": 0.35,
                                                        "seed": 33}), tile=1.5, double=True)
    m.mat("panel", color=(1, 1, 1), tex=("ribbed", {"color": BODY, "ribs": 6, "dirt": 0.2, "seed": 34}), tile=0.5)
    m.mat("panel_dirty", color=(1, 1, 1), tex=("ribbed", {"color": BODY, "ribs": 6, "dirt": 0.45, "seed": 35}),
          tile=0.5)
    m.mat("chassis", color=(1, 1, 1), tex=("paint", {"color": (0.09, 0.09, 0.10), "dirt": 0.55, "rough": 0.6,
                                                     "seed": 36}), tile=1.0, double=True)
    m.mat("grime", color=(1, 1, 1), tex=("grime", {}), tile=0.8, double=True)
    m.mat("steel", color=(0.6, 0.6, 0.62), metallic=0.9, roughness=0.35)
    m.mat("knife", color=(0.82, 0.82, 0.84), metallic=1.0, roughness=0.2)
    m.mat("tread", color=(1, 1, 1), tex=("tread_plate", {}), tile=0.4)
    m.mat("louvre", color=(1, 1, 1), tex=("honeycomb", {"cells": 8}), tile=0.15)
    m.mat("screen", color=(1, 1, 1), tex=("honeycomb", {"cells": 10, "color": (0.12, 0.12, 0.13)}), tile=0.1,
          double=True)
    m.mat("grain", color=(1, 1, 1), tex=("paint", {"color": (0.78, 0.64, 0.32), "rough": 0.95, "dirt": 0.25,
                                                   "seed": 40}), tile=0.6)
    m.mat("reel_bat", color=(0.13, 0.13, 0.14), metallic=0.3, roughness=0.5)
    m.mat("tine", color=(0.82, 0.82, 0.78), roughness=0.45)
    m.mat("seat", color=(1, 1, 1), tex=("fabric", {"color": (0.14, 0.14, 0.15)}), tile=0.2)
    m.mat("light_work", color=(0.85, 0.87, 0.9), metallic=0.3, roughness=0.1, tex=("lens", {}), tile=0.05,
          emissive=(1.0, 1.0, 0.95), light="work", light_strength=10.0)
    m.mat("light_beacon", color=(0.95, 0.5, 0.05), alpha=0.9, roughness=0.15, emissive=(1.0, 0.5, 0.0),
          light="beacon", light_strength=8.0)
    mb = m.mb

    # =============================================================== body
    # lower front (threshing drum / feeder entry), between the front tyres
    rbox(mb, "", "paint", 0.92, 2.46, -0.78, 0.78, 0.72, 2.02, 0.08, classify=dirty_below(1.35))
    # lower middle (cleaning shoe, fan), between the front and rear tyres
    rbox(mb, "", "paint", -1.06, 0.92, -1.10, 1.10, 0.58, 2.02, 0.10, classify=dirty_below(1.35))
    # rear keel (narrow: the rear tyres steer +-50 deg under the body) and the wide rear box above them
    rbox(mb, "", "paint", -3.35, -1.06, -0.40, 0.40, 1.00, 2.02, 0.06, classify=dirty_below(1.5))
    rbox(mb, "", "paint", -3.35, -1.06, -1.10, 1.10, 1.74, 2.06, 0.05, classify=dirty_below(1.85))
    # upper body (separator, engine bay)
    rbox(mb, "", "paint", -3.36, 1.30, -1.50, 1.50, 1.98, 3.20, 0.14, classify=dirty_below(2.1))
    # engine compartment on top at the rear
    rbox(mb, "", "paint", -3.36, -1.40, -1.35, 1.35, 3.12, 3.48, 0.08)

    # side access doors (slabs with a dark seam) and handles
    def door_panel(x0, x1, z0, z1, yface, side, mat="panel"):
        y = side * yface
        box(mb, "", "gap", x0 - 0.015, x1 + 0.015, min(y, y + side * 0.006), max(y, y + side * 0.006),
            z0 - 0.015, z1 + 0.015)
        y0, y1 = sorted((y + side * 0.004, y + side * 0.024))
        box(mb, "", mat, x0, x1, y0, y1, z0, z1)
        hy = y + side * 0.034
        box(mb, "", "trim_black", x1 - 0.14, x1 - 0.05, min(hy - 0.012, hy + 0.012), max(hy - 0.012, hy + 0.012),
            (z0 + z1) / 2 - 0.02, (z0 + z1) / 2 + 0.02)

    for side in (1, -1):
        door_panel(-1.30, -0.25, 2.14, 3.04, 1.50, side)
        door_panel(-0.15, 0.95, 2.14, 3.04, 1.50, side)
        door_panel(-0.95, -0.05, 0.74, 1.86, 1.10, side, "panel_dirty")
        door_panel(0.05, 0.78, 0.74, 1.86, 1.10, side, "panel_dirty")
    # right side engine louvres
    box(mb, "", "louvre", -3.15, -1.65, -1.515, -1.50, 2.25, 3.00)
    for k in range(9):
        z = 2.30 + k * 0.08
        M.box(mb, (0, 0, 0), (1.50, 0.012, 0.07), "paint", "",
              xf=compose(translate((-2.40, -1.53, z)), rot_axis((1, 0, 0), 0.5)))
    # top engine grille with slats
    box(mb, "", "louvre", -3.18, -1.62, -1.08, 1.08, 3.475, 3.485)
    for k in range(14):
        x = -3.12 + k * 0.11
        M.box(mb, (0, 0, 0), (0.08, 2.12, 0.012), "paint", "",
              xf=compose(translate((x, 0.0, 3.505)), rot_y(0.45)))
    box(mb, "", "paint", -3.20, -1.60, -1.10, -1.06, 3.48, 3.53)
    box(mb, "", "paint", -3.20, -1.60, 1.06, 1.10, 3.48, 3.53)
    # rear face grille of the engine bay
    box(mb, "", "louvre", -3.375, -3.36, -1.10, 1.10, 3.17, 3.44)
    # rotary cooling screen on the left side
    sc = (-2.45, 1.50, 2.62)
    fs = frame_z_to((0, 1, 0), sc)
    M.lathe(mb, [(0.54, -0.02), (0.54, 0.06), (0.50, 0.07), (0.50, 0.03)], "paint", "", fs, count=40, orient=True)
    M.lathe(mb, [(0.50, 0.035), (0.08, 0.035)], "screen", "", fs, count=40, orient=False)
    M.lathe(mb, [(0.10, 0.03), (0.10, 0.09), (0.001, 0.10)], "paint", "", fs, count=16, orient=True)
    for k in range(4):
        a = k * math.pi / 2 + 0.4
        p0 = xform_point(fs, (0.08 * math.cos(a), 0.08 * math.sin(a), 0.07))
        p1 = xform_point(fs, (0.50 * math.cos(a), 0.50 * math.sin(a), 0.07))
        M.tube(mb, [p0, p1], 0.012, "paint", "", count=6)
    # air pre-cleaner and exhaust on the engine hood
    M.cylinder(mb, (-1.85, 0.85, 3.48), (-1.85, 0.85, 3.72), 0.14, "trim_black", "", count=16)
    M.lathe(mb, [(0.17, 3.72), (0.17, 3.77), (0.10, 3.80), (0.001, 3.81)],
            "plastic_black", "", translate((-1.85, 0.85, 0.0)), count=16, orient=True)
    M.cylinder(mb, (-2.9, -1.05, 3.40), (-2.9, -1.05, 3.62), 0.13, "chassis", "", count=16)  # muffler
    M.tube(mb, [(-2.9, -1.05, 3.60), (-2.9, -1.05, 3.80), (-2.93, -1.05, 3.86), (-3.0, -1.05, 3.90)], 0.065,
           "steel", "", count=12)

    # front mudguards
    for side in (1, -1):
        rows = []
        for k in range(15):
            a = math.radians(28 + (150 - 28) * k / 14)
            x, z = XF + 1.03 * math.cos(a), FRONT.radius + 1.03 * math.sin(a)
            rows.append([(x, side * 0.84, z), (x, side * 1.79, z)])
        mb.grid(rows, "paint_dirty", "")
        for k in range(0, 15, 7):  # stays
            a = math.radians(28 + (150 - 28) * k / 14)
            x, z = XF + 1.03 * math.cos(a), FRONT.radius + 1.03 * math.sin(a)
            M.tube(mb, [(x, side * 0.80, z - 0.02), (x, side * 0.95, z - 0.02)], 0.025, "chassis", "", count=6)

    # front axle (rigid, final drives into the rims)
    box(mb, "", "chassis", 1.80, 2.10, -0.86, 0.86, 0.80, 1.10)
    for side in (1, -1):
        M.cylinder(mb, (XF, side * 0.78, FRONT.radius), (XF, side * 1.25, FRONT.radius), 0.26, "chassis", "",
                   count=20)
        # feeder ram bracket under the axle
        box(mb, "", "chassis", 1.97, 2.13, side * 0.70 - 0.06, side * 0.70 + 0.06 + side * 0.06, 0.60, 0.84)

    # hitch drawbar at the rear (for the header trailer)
    beam(mb, "", "chassis", (-3.20, 0.0, 1.02), (-4.30, 0.0, 0.64), 0.12, 0.12)
    box(mb, "", "chassis", -4.44, -4.28, -0.09, 0.09, 0.55, 0.68)
    M.cylinder(mb, (-4.38, 0.0, 0.52), (-4.38, 0.0, 0.73), 0.025, "steel", "", count=8)

    # =============================================================== straw hood, chopper, spreaders
    rows = []
    for i in range(7):
        f = i / 6
        x = -3.28 - 1.02 * f
        w = 0.95 - 0.10 * f
        zb = 1.15 + 0.10 * f
        zt = 2.62 - 0.72 * f ** 1.2
        rc = 0.26
        prof = [(w, zb), (w, (zb + zt - rc) / 2), (w, zt - rc)]
        for k in range(1, 5):
            a = k / 5 * math.pi / 2
            prof.append((w - rc + rc * math.cos(a), zt - rc + rc * math.sin(a)))
        prof += [(w - rc, zt), (0.0, zt), (-(w - rc), zt)]
        for k in range(1, 5):
            a = math.pi / 2 + k / 5 * math.pi / 2
            prof.append((-(w - rc) + rc * math.cos(a), zt - rc + rc * math.sin(a)))
        prof += [(-w, zt - rc), (-w, (zb + zt - rc) / 2), (-w, zb)]
        rows.append([(x, y, z) for (y, z) in prof])
    mb.grid(rows, "paint", "")
    # chopper bearings and spreader beam (static)
    for side in (1, -1):
        M.cylinder(mb, (-3.75, side * 0.72, 1.45), (-3.75, side * 0.93, 1.45), 0.08, "chassis", "", count=12)
    box(mb, "", "chassis", -4.17, -4.03, -0.86, 0.86, 1.30, 1.40)

    m.node("straw_chopper", None, (-3.75, 0.0, 1.45),
           m.continuous((0, 1, 0), "chopper rotor, + = top moving forward (works at ~3000 rpm)"))
    M.cylinder(mb, (-3.75, -0.72, 1.45), (-3.75, 0.72, 1.45), 0.16, "chassis", "straw_chopper", count=18)
    for k in range(4):
        for j in range(12):
            a = k * math.pi / 2 + j * 0.37
            y = -0.66 + j * 0.12
            c, s = math.cos(a), math.sin(a)
            p0 = (-3.75 + 0.15 * c, y, 1.45 + 0.15 * s)
            p1 = (-3.75 + 0.30 * c, y, 1.45 + 0.30 * s)
            M.box(mb, (0, 0, 0), (0.15, 0.012, 0.05), "steel", "straw_chopper",
                  xf=compose(translate(mul(add(p0, p1), 0.5)), rot_y(-a)))

    for name, side in (("spreader_L", 1), ("spreader_R", -1)):
        c = (-4.10, side * 0.42, 1.0)
        m.node(name, None, c, m.continuous((0, 0, 1), "residue spreader disc; L turns + (CCW from above), "
                                                      "R turns - in operation"))
        fz = translate(c)
        M.lathe(mb, [(0.34, 0.0), (0.34, 0.012), (0.001, 0.012)], "chassis", name, fz, count=32, orient=True)
        M.lathe(mb, [(0.34, 0.0), (0.001, 0.0)], "chassis", name, fz, count=32, orient=False)
        M.cylinder(mb, (c[0], c[1], c[2]), (c[0], c[1], 1.30), 0.05, "steel", name, count=10)
        for k in range(6):
            a = k * math.pi / 3
            ca, sa = math.cos(a), math.sin(a)
            mid = (c[0] + 0.2 * ca, c[1] + 0.2 * sa, c[2] + 0.07)
            M.box(mb, (0, 0, 0), (0.26, 0.012, 0.11), "paint_dirty", name, xf=compose(translate(mid), rot_z(a + 0.3)))

    # =============================================================== rear pendulum axle + wheels
    m.node("rear_axle", None, (XR, 0.0, 0.95),
           m.revolute((1, 0, 0), -8, 8, "pendulum (oscillating) rear axle roll, + = left wheel up"))
    box(mb, "rear_axle", "chassis", XR - 0.10, XR + 0.10, -0.55, 0.55, 0.66, 0.84)
    box(mb, "rear_axle", "chassis", XR - 0.16, XR + 0.16, -0.22, 0.22, 0.80, 0.98)
    M.cylinder(mb, (XR - 0.2, 0.0, 0.95), (XR + 0.2, 0.0, 0.95), 0.07, "steel", "rear_axle", count=12)
    for side in (1, -1):
        M.cylinder(mb, (XR, side * 0.55, 0.58), (XR, side * 0.88, 0.92), 0.08, "chassis", "rear_axle", count=12)
    for key, side in (("RL", 1), ("RR", -1)):
        corner(m, key, (XR, side * YR, REAR.radius), REAR, parent="rear_axle", steer=True, steer_limits=(-50, 50),
               susp_node=False, caliper=False)
        sn = f"steer_{key}"
        M.cylinder(mb, (XR, side * 0.60, REAR.radius), (XR, side * 1.16, REAR.radius), 0.09, "chassis", sn, count=14)
        m.nodes[sn].joint["note"] = ("rear-wheel steer angle, + = wheel points left (so + on the rear axle "
                                     "turns the machine right); kingpin simplified to the wheel centre")
    for key, side in (("FL", 1), ("FR", -1)):
        corner(m, key, (XF, side * YF, FRONT.radius), FRONT, parent=None, steer=False, susp_node=False,
               caliper=False)

    # =============================================================== cab
    zf, zr, xr, yc = 2.35, 3.66, 1.42, 0.90

    def xfr(z):
        return 3.30 - 0.25 * (z - zf) / (zr - zf)

    rbox(mb, "", "paint", 1.28, 3.36, -0.95, 0.95, 2.00, 2.36, 0.05)
    # corner posts
    for s in (1, -1):
        M.tube(mb, [(xfr(zf), s * yc, zf), (xfr(zr), s * yc, zr)], 0.04, "trim_black", "", count=8)
        M.tube(mb, [(xr + 0.01, s * yc, zf), (xr + 0.01, s * yc, zr)], 0.04, "trim_black", "", count=8)
    M.tube(mb, [(2.445, yc, zf), (2.445, yc, zr)], 0.025, "trim_black", "", count=8)
    # glass: front, right side, left fixed part, rear upper
    mb.polygon([(xfr(zf + 0.03), -yc + 0.03, zf + 0.03), (xfr(zf + 0.03), yc - 0.03, zf + 0.03),
                (xfr(zr - 0.02), yc - 0.03, zr - 0.02), (xfr(zr - 0.02), -yc + 0.03, zr - 0.02)], "glass", "")
    mb.polygon([(xr + 0.04, -yc, zf + 0.04), (xr + 0.04, -yc, zr - 0.02), (xfr(zr - 0.02) - 0.04, -yc, zr - 0.02),
                (xfr(zf + 0.04) - 0.04, -yc, zf + 0.04)], "glass", "")
    mb.polygon([(2.47, yc, zf + 0.04), (xfr(zf + 0.04) - 0.04, yc, zf + 0.04), (xfr(zr - 0.02) - 0.04, yc, zr - 0.02),
                (2.47, yc, zr - 0.02)], "glass", "")
    box(mb, "", "paint", xr - 0.03, xr + 0.03, -yc, yc, zf, 2.95)
    mb.polygon([(xr, -yc + 0.04, 2.95), (xr, yc - 0.04, 2.95), (xr, yc - 0.04, zr - 0.02), (xr, -yc + 0.04, zr - 0.02)],
               "glass", "")
    # roof with sun visor band
    rbox(mb, "", "paint_roof", 1.30, 3.24, -1.00, 1.00, 3.64, 3.84, 0.08)
    box(mb, "", "trim_black", 3.10, 3.25, -0.96, 0.96, 3.60, 3.65)
    # interior
    box(mb, "", "plastic_black", xr + 0.03, xfr(zf) - 0.05, -yc + 0.03, yc - 0.03, zf, zf + 0.02)
    seat(m, "", (2.00, 0.0, 2.86), width=0.52, recline=0.25)
    box(mb, "", "trim_black", 1.82, 2.18, -0.18, 0.18, zf, 2.74)
    rbox(mb, "", "interior", 1.85, 2.55, -0.50, -0.30, zf, 2.98, 0.04)                 # armrest console
    M.cylinder(mb, (2.40, -0.40, 2.98), (2.42, -0.40, 3.08), 0.022, "trim_black", "", count=8)  # joystick
    rbox(mb, "", "interior", 2.72, 2.98, -0.16, 0.16, zf, 2.80, 0.04)                  # steering pedestal
    steering_wheel(m, "steering_wheel", None, (2.62, 0.0, 3.08), (0.45, 0.0, -0.89), radius=0.19, lock_deg=720)
    rbox(mb, "", "interior", 2.60, 2.85, 0.30, 0.60, zf, 2.60, 0.03)                   # left side console

    # cab door (left, hinged at the rear edge, opens outward over the platform)
    m.node("door_L", None, (1.46, yc + 0.01, 3.0), m.revolute((0, 0, 1), 0, 55, "cab door opening angle"))
    dy = yc + 0.01
    M.tube(mb, [(1.475, dy, 2.40), (2.42, dy, 2.40), (2.42, dy, 3.60), (1.475, dy, 3.60)], 0.022, "trim_black",
           "door_L", count=6, closed=True)
    mb.polygon([(1.49, dy, 2.42), (2.40, dy, 2.42), (2.40, dy, 3.58), (1.49, dy, 3.58)], "glass", "door_L")
    box(mb, "door_L", "chrome", 2.25, 2.37, dy + 0.02, dy + 0.045, 2.97, 3.0)
    box(mb, "door_L", "trim_black", 2.30, 2.36, dy - 0.03, dy + 0.02, 2.95, 3.02)

    # platform on the left with handrail
    box(mb, "", "tread", 1.40, 3.26, 0.93, 1.75, 2.30, 2.35)
    for x in (1.60, 2.90):
        M.tube(mb, [(x, 0.96, 2.30), (x, 1.70, 2.06)], 0.035, "chassis", "", count=8)
    for x in (1.45, 2.35, 3.20):
        M.tube(mb, [(x, 1.72, 2.35), (x, 1.72, 3.35)], 0.022, "paint_roof", "", count=8)
    M.tube(mb, [(1.45, 0.95, 3.35), (1.45, 1.72, 3.35), (3.20, 1.72, 3.35)], 0.022, "paint_roof", "", count=8)
    M.tube(mb, [(1.45, 0.95, 2.85), (1.45, 1.72, 2.85), (3.20, 1.72, 2.85)], 0.018, "paint_roof", "", count=8)

    # folding ladder at the platform front, ahead of the left front tyre
    lp = (3.26, 1.315, 2.32)
    m.node("ladder", None, lp, m.revolute((0, -1, 0), 0, 80, "ladder fold, + = swings up forward (transport)"))
    top_x, bot_x, bot_z = 3.26, 3.60, 0.40
    for y in (1.08, 1.55):
        M.tube(mb, [(top_x, y, 2.32), (bot_x, y, bot_z)], 0.025, "chassis", "ladder", count=8)
        M.tube(mb, [(top_x, y, 2.32), (top_x + 0.02, y, 3.0)], 0.02, "paint_roof", "ladder", count=8)
    n_r = 7
    for k in range(n_r):
        f = (k + 0.5) / n_r
        x = bot_x + (top_x - bot_x) * f
        z = bot_z + (2.32 - bot_z) * f - 0.04
        box(mb, "ladder", "tread", x - 0.07, x + 0.07, 1.08, 1.55, z - 0.015, z + 0.015)

    # mirrors (on arms from the front pillars)
    for side in (1, -1):
        mirror(m, "", (3.05, side * 0.93, 3.33), side, size=(0.07, 0.20, 0.30), body_mat="trim_black", arm=0.36)

    # cradle for the folded unloading auger (from the cab roof front-left)
    M.tube(mb, [(3.02, 0.95, 3.64), (3.02, 1.30, 3.60), (3.02, 1.62, 3.59)], 0.03, "chassis", "", count=8)
    M.tube(mb, [(3.02, 1.40, 3.78), (3.02, 1.43, 3.59), (3.02, 1.81, 3.59), (3.02, 1.84, 3.78)], 0.02,
           "chassis", "", count=6)

    # cab lamps: head, work, indicators, beacon
    for side in (1, -1):
        lamp(mb, "", (3.24, side * 0.48, 3.74), (0.08, 0.22, 0.09), 0, 1, "light_head")
        for y in (0.18, 0.76):
            lamp(mb, "", (3.24, side * y, 3.74), (0.07, 0.14, 0.10), 0, 1, "light_work")
        ind = "light_indicator_left" if side > 0 else "light_indicator_right"
        lamp(mb, "", (3.18, side * 1.02, 3.74), (0.09, 0.06, 0.07), 1, side, ind)
        lamp(mb, "", (3.22, side * 0.99, 3.74), (0.06, 0.05, 0.07), 0, 1, ind)
        lamp(mb, "", (1.36, side * 0.60, 3.74), (0.07, 0.14, 0.10), 0, -1, "light_work")
    for c in ((1.55, 0.78, 3.84), (-3.15, -0.95, 3.48)):
        M.cylinder(mb, c, add(c, (0, 0, 0.025)), 0.075, "trim_black", "", count=14)
        M.ellipsoid(mb, add(c, (0, 0, 0.085)), (0.06, 0.06, 0.06), "light_beacon", "", count=14, rings=7)

    # =============================================================== grain tank, covers, unloading auger
    def tank_cls(i, j, c, n):
        return "grain" if n[2] > 0.9 else "paint"
    rbox(mb, "", "paint", -1.35, 1.28, -1.38, 1.38, 3.18, 3.70, 0.10, classify=tank_cls)
    for s in (1, -1):
        box(mb, "", "paint", -1.35, 1.28, min(s * 1.36, s * 1.40), max(s * 1.36, s * 1.40), 3.58, 3.87)
    box(mb, "", "paint", 1.24, 1.28, -1.36, 1.36, 3.58, 3.87)
    box(mb, "", "paint", -1.35, -1.31, -1.36, 1.36, 3.58, 3.87)
    mb.polygon([(1.285, -0.35, 3.30), (1.285, 0.35, 3.30), (1.285, 0.35, 3.62), (1.285, -0.35, 3.62)], "glass", "")
    lamp(mb, "", (-1.20, 1.40, 3.74), (0.12, 0.07, 0.09), 1, 1, "light_work")  # unloading work lamp

    for name, side in (("grain_tank_cover_L", 1), ("grain_tank_cover_R", -1)):
        m.node(name, None, (0.0, side * 1.40, 3.90),
               m.revolute((-side, 0, 0), 0, 100, "grain tank extension, + = folds up/out (open for harvest)"))
        y0, y1 = sorted((0.0, side * 1.40))
        box(mb, name, "panel", -1.33, 1.26, y0, y1, 3.87, 3.90)
        for x in (-0.9, 0.0, 0.9):
            box(mb, name, "paint", x - 0.03, x + 0.03, y0 + 0.02, y1 - 0.02, 3.90, 3.93)

    # vertical unloading tube (static) with the swing turret base
    ax, ay, az = AUG_PIVOT
    M.cylinder(mb, (ax, ay, 2.30), (ax, ay, 3.60), 0.17, "paint", "", count=20)
    box(mb, "", "paint", ax - 0.2, ax + 0.2, 1.42, 1.62, 2.25, 2.50)
    M.cylinder(mb, (ax, ay, 3.55), (ax, ay, 3.61), 0.23, "chassis", "", count=20)

    m.node("unloading_auger", None, AUG_PIVOT,
           m.revolute((0, 0, 1), 0, 100, "unloading auger swing, 0 = folded forward along the left side, "
                                         "+ = swings out to the left"),
           aero={"tube_diameter": 2 * AUG_R, "tube_length": AUG_LEN, "pivot": list(AUG_PIVOT),
                 "tube_axis_rest": [1.0, 0.0, 0.0],
                 "note": "straight horizontal tube on the node's +x axis at rest, starting at the pivot; "
                         "tip elbow and spout on child auger_spout"})
    M.cylinder(mb, (ax, ay, 3.60), (ax, ay, 3.98), 0.22, "paint", "unloading_auger", count=20)
    tip_x = ax + AUG_LEN
    M.cylinder(mb, (ax, ay, az), (tip_x - 0.12, ay, az), AUG_R, "paint", "unloading_auger", count=20)
    for k in range(1, 4):
        x = ax + k * AUG_LEN / 4
        M.cylinder(mb, (x - 0.03, ay, az), (x + 0.03, ay, az), AUG_R + 0.02, "chassis", "unloading_auger", count=20)
    rbox(mb, "unloading_auger", "paint", tip_x - 0.40, tip_x, ay - 0.20, ay + 0.20, az - 0.20, az + 0.20, 0.06)

    spout_p = (tip_x - 0.15, ay, az)
    m.node("auger_spout", "unloading_auger", spout_p,
           m.revolute((0, -1, 0), -25, 35, "spout tilt about the tube tip, + = outlet swings away from the pivot"))
    M.cylinder(mb, (spout_p[0], ay, az - 0.12), (spout_p[0], ay, 3.38), 0.16, "paint", "auger_spout", count=18)
    M.lathe(mb, [(0.17, 3.40), (0.17, 3.32), (0.12, 3.06)], "plastic_black", "auger_spout",
            translate((spout_p[0], ay, 0.0)), count=18, orient=True)
    lamp(mb, "auger_spout", (spout_p[0] + 0.19, ay, 3.62), (0.05, 0.10, 0.08), 0, 1, "light_work")
    m.socket("socket_auger_spout", "auger_spout", (spout_p[0], ay, 3.04),
             "grain outlet of the unloading auger spout (moves with swing and spout tilt)")

    # =============================================================== rear lamps
    for side in (1, -1):
        ind = "light_indicator_left" if side > 0 else "light_indicator_right"
        x = -3.37
        box(mb, "", "trim_black", x - 0.02, x + 0.01, side * 1.30 - 0.14, side * 1.30 + 0.14, 2.12, 2.44)
        lamp(mb, "", (x - 0.03, side * 1.30, 2.18), (0.04, 0.22, 0.07), 0, -1, "light_tail")
        lamp(mb, "", (x - 0.03, side * 1.30, 2.27), (0.04, 0.22, 0.07), 0, -1, "light_brake")
        lamp(mb, "", (x - 0.03, side * 1.30, 2.37), (0.04, 0.22, 0.07), 0, -1, ind)
        lamp(mb, "", (x - 0.03, side * 1.02, 2.20), (0.04, 0.12, 0.07), 0, -1, "light_reverse")
        lamp(mb, "", (x - 0.03, side * 0.80, 3.02), (0.05, 0.16, 0.11), 0, -1, "light_work")

    # =============================================================== feeder house
    fp = FEEDER_PIVOT
    m.node("feeder", None, fp, m.revolute((0, -1, 0), -2, 20, "feeder house lift about its top rear pivot, "
                                                             "+ = raises the header"))
    fr_x, fr_top, fr_bot = 2.15, 1.68, 1.12
    ff_x, ff_top, ff_bot = FEEDER_FACE[0], 1.16, 0.44
    hw = 0.66

    def fsec(x, zt, zb):
        return [(x, a, b) for (a, b) in M.rrect(2 * hw, zt - zb, 0.05, 2, 0.0, (zt + zb) / 2)]
    M.loft(mb, [fsec(fr_x, fr_top, fr_bot), fsec(ff_x, ff_top, ff_bot)], "paint", "feeder",
           classify=dirty_below(0.9))

    def f_top(x):
        return fr_top + (ff_top - fr_top) * (x - fr_x) / (ff_x - fr_x)

    def f_bot(x):
        return fr_bot + (ff_bot - fr_bot) * (x - fr_x) / (ff_x - fr_x)
    for x in (2.95, 3.45, 3.95):
        for s in (1, -1):
            box(mb, "feeder", "paint", x - 0.03, x + 0.03, min(s * hw, s * (hw + 0.02)), max(s * hw, s * (hw + 0.02)),
                f_bot(x) + 0.02, f_top(x) - 0.02)
    for s in (1, -1):  # side inspection doors
        y0, y1 = sorted((s * (hw + 0.002), s * (hw + 0.018)))
        box(mb, "feeder", "panel_dirty", 3.02, 3.38, y0, y1, f_bot(3.38) + 0.10, f_top(3.2) - 0.10)
    # face plate (header adapter ring)
    box(mb, "feeder", "chassis", ff_x - 0.04, ff_x, -0.72, 0.72, 1.12, 1.20)
    box(mb, "feeder", "chassis", ff_x - 0.04, ff_x, -0.72, 0.72, 0.40, 0.48)
    for s in (1, -1):
        box(mb, "feeder", "chassis", ff_x - 0.04, ff_x, min(s * 0.64, s * 0.72), max(s * 0.64, s * 0.72), 0.40, 1.20)
    box(mb, "feeder", "gap", ff_x - 0.05, ff_x - 0.03, -0.64, 0.64, 0.48, 1.12)

    # feeder lift rams: barrel on the front-axle brackets, rod eye on the feeder sides
    for s, tag in ((1, "L"), (-1, "R")):
        B = (2.05, s * 0.76, 0.66)
        E = (3.60, s * 0.76, 0.86)
        y0, y1 = sorted((s * hw, s * 0.80))
        box(mb, "feeder", "chassis", 3.52, 3.68, y0, y1, 0.80, 0.93)
        rng = pin_distance(B, E, fp, (0, -1, 0), [math.radians(a) for a in range(-2, 21)])
        hyd_cylinder(m, f"feeder_ram_{tag}", None, B, "feeder", E, rng, r=0.065)

    # =============================================================== header
    hp = FEEDER_FACE
    m.node("header", "feeder", hp, m.revolute((1, 0, 0), -4, 4, "header_tilt: lateral tilt about the feeder face "
                                                                "centre, + = left end up"),
           header={"cut_width": 9.0, "cutter_bar_x": 6.10, "cutter_bar_z": 0.18})
    H = "header"
    t = 0.012
    xb = 4.57
    arc_c, arc_r = AUGER_C, 0.38
    inner = [(xb, 0.58)]
    outer = [(xb - t, 0.58)]
    for k in range(1, 9):
        a = math.pi + k / 8 * math.pi / 2
        inner.append((arc_c[0] + arc_r * math.cos(a), arc_c[2] + arc_r * math.sin(a)))
        outer.append((arc_c[0] + (arc_r + t) * math.cos(a), arc_c[2] + (arc_r + t) * math.sin(a)))
    inner.append((6.02, 0.17))
    outer.append((6.02, 0.17 - t))
    M.extrude(mb, inner + list(reversed(outer)), 2 * Y_END, "paint_dirty", H, xf=xz_frame(0.0))
    # back wall with the feeder opening, top beam
    for s in (1, -1):
        y0, y1 = sorted((s * 0.66, s * Y_END))
        box(mb, H, "paint", xb - t, xb, y0, y1, 0.58, 1.42)
    box(mb, H, "paint", xb - t, xb, -0.66, 0.66, 1.15, 1.42)
    rbox(mb, H, "paint", 4.45, 4.70, -4.50, 4.50, 1.40, 1.60, 0.03)
    # adapter frame around the opening (behind the back wall)
    box(mb, H, "chassis", ff_x, xb - t, -0.78, 0.78, 1.15, 1.26)
    box(mb, H, "chassis", ff_x, xb - t, -0.78, 0.78, 0.40, 0.50)
    for s in (1, -1):
        box(mb, H, "chassis", ff_x, xb - t, min(s * 0.66, s * 0.78), max(s * 0.66, s * 0.78), 0.40, 1.26)
    # back-wall stiffeners
    for yy in (-3.4, -2.2, -1.1, 1.1, 2.2, 3.4):
        box(mb, H, "paint", xb - 0.10, xb - t, yy - 0.04, yy + 0.04, 0.58, 1.40)
    # end sheets and crop dividers
    es = [(4.52, 0.17), (6.14, 0.17), (6.12, 0.70), (5.10, 1.62), (4.52, 1.62)]
    for s in (1, -1):
        M.extrude(mb, es, t, "paint", H, xf=xz_frame(s * Y_END))
        yc_ = s * Y_END
        secs = []
        for x, hw_, zt_, sc in ((6.12, 0.08, 0.70, 1.0), (6.45, 0.06, 0.48, 0.7), (6.75, 0.035, 0.30, 0.35),
                                (6.88, 0.004, 0.21, 0.05)):
            zb_ = 0.17 + 0.03 * (1 - sc)
            secs.append([(x, yc_ - hw_, zb_), (x, yc_ + hw_, zb_), (x, yc_ + hw_ * 0.3, zt_), (x, yc_ - hw_ * 0.3, zt_)])
        M.loft(mb, secs, "paint", H)
        M.tube(mb, [(6.50, yc_, 0.44), (5.95, yc_ - s * 0.03, 1.05), (5.60, yc_ - s * 0.03, 1.20)], 0.015,
               "chassis", H, count=6)
    # skid shoes, cutter bar, knife, guards
    for yy in (-4.0, -2.4, -0.8, 0.8, 2.4, 4.0):
        box(mb, H, "steel", 5.00, 5.95, yy - 0.06, yy + 0.06, 0.15, 0.162)
    box(mb, H, "chassis", 5.98, 6.08, -Y_END, Y_END, 0.15, 0.19)
    box(mb, H, "knife", 6.05, 6.13, -4.50, 4.50, 0.19, 0.198)
    n_g = 118
    for k in range(n_g):
        y = -4.48 + 8.96 * k / (n_g - 1)
        poly = [(6.02, y - 0.018), (6.18, y - 0.007), (6.23, y), (6.18, y + 0.007), (6.02, y + 0.018)]
        M.extrude(mb, poly, 0.026, "steel", H, xf=translate((0.0, 0.0, 0.186)))
    # reel arm brackets and auger bearings
    for yy in (-Y_ARM, 0.0, Y_ARM):
        box(mb, H, "chassis", 4.50, 4.72, yy - 0.07, yy + 0.07, 1.45, 1.66)
    for s in (1, -1):
        M.cylinder(mb, (AUGER_C[0], s * 4.46, AUGER_C[2]), (AUGER_C[0], s * (Y_END + 0.02), AUGER_C[2]), 0.07,
                   "chassis", H, count=12)
    m.socket("socket_cutterbar_center", H, (6.10, 0.0, 0.18), "cutter bar centre (cut line, cut height reference)")
    for s, tag in ((1, "L"), (-1, "R")):
        m.socket(f"socket_divider_{tag}", H, (6.88, s * Y_END, 0.21), "crop divider point (header outer end)")

    # ---- cross auger
    m.node("header_auger", H, AUGER_C,
           m.continuous((0, 1, 0), "cross auger, + = working direction (front of the auger moves down, "
                                   "flights convey to the centre)"))
    A = "header_auger"
    M.cylinder(mb, (AUGER_C[0], -4.46, AUGER_C[2]), (AUGER_C[0], 4.46, AUGER_C[2]), 0.20, "paint", A, count=24)
    pitch, r_in, r_out, th = 0.62, 0.19, 0.30, 0.012
    for s in (1, -1):
        y_a, y_b = 0.62, 4.40
        n = int((y_b - y_a) / pitch * 22)
        secs = []
        for k in range(n + 1):
            ay_ = y_a + (y_b - y_a) * k / n
            al = -2 * math.pi * ay_ / pitch
            ca, sa = math.cos(al), math.sin(al)
            yy = s * ay_
            sec = []
            for (rr, dy_) in ((r_in, -th), (r_out, -th), (r_out, th), (r_in, th)):
                sec.append((AUGER_C[0] + rr * ca, yy + dy_, AUGER_C[2] + rr * sa))
            secs.append(sec)
        M.loft(mb, secs, "paint", A)
    for yy in (-0.40, 0.0, 0.40):
        for k in range(4):
            a = k * math.pi / 2 + (0.8 if yy == 0.0 else 0.0)
            ca, sa = math.cos(a), math.sin(a)
            M.tube(mb, [(AUGER_C[0] + 0.19 * ca, yy, AUGER_C[2] + 0.19 * sa),
                        (AUGER_C[0] + 0.36 * ca, yy, AUGER_C[2] + 0.36 * sa)], 0.012, "steel", A, count=6)

    # ---- reel arms, fore/aft slide, reel
    m.node("reel_arms", H, REEL_PIVOT, m.revolute((0, -1, 0), -6, 25, "reel lift, + = raises the reel"))
    for yy in (-Y_ARM, 0.0, Y_ARM):
        p0 = (REEL_PIVOT[0], yy, REEL_PIVOT[2])
        p1 = add(p0, mul(REEL_DIR, 1.78))
        beam(mb, "reel_arms", "paint", p0, p1, 0.07, 0.10)
        M.cylinder(mb, (p0[0], yy - 0.08, p0[2]), (p0[0], yy + 0.08, p0[2]), 0.04, "steel", "reel_arms", count=10)
    m.node("reel_fore_aft", "reel_arms", REEL_AXIS,
           m.prismatic(REEL_DIR, -0.25, 0.30, "reel fore/aft along the arms, + = forward"))
    for yy in (-Y_ARM, 0.0, Y_ARM):
        c = (REEL_AXIS[0], yy, REEL_AXIS[2])
        g = math.atan2(-REEL_DIR[2], REEL_DIR[0])
        M.rounded_box(mb, (0, 0, 0), (0.20, 0.11, 0.15), 0.02, "chassis", "reel_fore_aft", seg=1,
                      xf=compose(translate(c), rot_y(g)))
    m.node("reel", "reel_fore_aft", REEL_AXIS,
           m.continuous((0, 1, 0), "reel, + = working direction (top moves forward, bats sweep rearward at "
                                   "the bottom)"))
    R_ = "reel"
    rx, rz = REEL_AXIS[0], REEL_AXIS[2]
    M.cylinder(mb, (rx, -4.42, rz), (rx, 4.42, rz), 0.08, "paint", R_, count=16)
    r_bat, nb = 0.50, 6
    for s in (1, -1):
        for yy in (0.12, 1.55, 2.98, 4.36):
            y = s * yy
            M.lathe(mb, [(0.12, -0.03), (0.12, 0.03)], "chassis", R_, compose(translate((rx, y, rz)),
                    frame_z_to((0, 1, 0))), count=12, orient=True)
            for k in range(nb):
                a = 2 * math.pi * k / nb
                M.tube(mb, [(rx + 0.08 * math.cos(a), y, rz + 0.08 * math.sin(a)),
                            (rx + r_bat * math.cos(a), y, rz + r_bat * math.sin(a))], 0.018, "paint", R_, count=6)
        for k in range(nb):
            a = 2 * math.pi * k / nb
            ca, sa = math.cos(a), math.sin(a)
            bx, bz = rx + r_bat * ca, rz + r_bat * sa
            M.cylinder(mb, (bx, s * 0.10, bz), (bx, s * 4.40, bz), 0.026, "reel_bat", R_, count=8)
            # tines trail against the rotation (+alpha side)
            tang = (-sa, ca)
            dx = 0.93 * ca + 0.37 * tang[0]
            dz = 0.93 * sa + 0.37 * tang[1]
            nt = 21
            for j in range(nt):
                y = s * (0.22 + (4.30 - 0.22) * j / (nt - 1))
                M.cylinder(mb, (bx, y, bz), (bx + 0.17 * dx, y, bz + 0.17 * dz), 0.006, "tine", R_, count=4)
    for s, tag in ((1, "L"), (-1, "R")):
        B = (4.64, s * Y_ARM, 0.95)
        E = add(add(REEL_PIVOT, mul(REEL_DIR, 0.55)), (-0.32 * 0.07, 0.0, -0.947 * 0.07))
        E = (E[0], s * Y_ARM, E[2])
        rng = pin_distance(B, E, REEL_PIVOT, (0, -1, 0), [math.radians(a) for a in range(-6, 26)])
        hyd_cylinder(m, f"reel_ram_{tag}", H, B, "reel_arms", E, rng, r=0.035)

    # =============================================================== sockets and notes
    m.socket("socket_driver_eye", None, (2.05, 0.0, 3.50), "driver eye point (camera)")
    m.socket("socket_grain_tank_fill", None, (0.0, 0.0, 3.70),
             "grain tank fill point / grain surface centre when full (tank ~12,000 l)")
    m.socket("socket_hitch_rear", None, (-4.38, 0.0, 0.62), "rear hitch pin for the header transport trailer")
    m.socket("socket_exhaust", None, (-3.01, -1.05, 3.905), "exhaust outlet (points rearward-up)")

    m.extra_rig["aero_hints"] = {
        "unloading_auger": {"node": "unloading_auger", "shape": "cylinder", "diameter": 2 * AUG_R,
                            "length": AUG_LEN, "pivot": list(AUG_PIVOT), "axis_at_rest": [1, 0, 0],
                            "centre_of_area_from_pivot": [AUG_LEN / 2, 0.0, 0.0],
                            "swing_axis": [0, 0, 1], "swing_range_deg": [0, 100]},
    }
    m.notes += [
        "Generic class-8 combine; dimensions are class-typical, not a specific production machine.",
        f"Unloading auger (node unloading_auger): straight tube, diameter {2 * AUG_R:.2f} m, length {AUG_LEN:.2f} m "
        f"from the swing pivot at {list(AUG_PIVOT)} along the node +x axis at rest (pointing forward, folded). "
        "Swing is revolute about +z (0 = folded, + = out to the left, max 100 deg); side area ~2.2 m2, "
        "centre of area 3.1 m from the pivot. Crosswind yaw moment test: compare joint 0 vs ~1.57 rad.",
        "Front axle is rigid (no suspension node): the tyres are the only springs. Rear axle is a pendulum "
        "(rear_axle, +-8 deg roll about x at z 0.95) with steered wheels (+-50 deg).",
        "Rear-wheel steer sign: + still means the wheel points left, which turns the machine to the right.",
        "Header is carried by feeder (lift) and header (lateral tilt); cutter bar at z 0.18, skids 0.15 m "
        "above the ground at rest; feeder -2 deg puts the header on the ground.",
        "Hydraulic rams (feeder_ram_*, reel_ram_*) are aim links (visual only, no stretch).",
        "Typical masses for the sim: machine ~16.5 t empty (CG approx x +0.5, z 1.6), header ~2.8 t, "
        "full grain tank ~9 t (CG near socket_grain_tank_fill, z ~3.2).",
    ]
    return m
