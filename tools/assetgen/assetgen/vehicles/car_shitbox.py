"""a) Shitbox: a worn-out, cheap late-80s / early-90s boxy 3-door hatchback supermini
(generic, class-typical dimensions; no real make or model).

Length 3.74 m over bumpers (body 3.64 m), width 1.60 m (body, no mirrors), height 1.40 m (roof),
wheelbase 2.35 m, track 1.37 m, tyres 155/70 R13 (r = 0.274 m) on steel wheels with
hubcaps - the rear right hubcap is missing.
Character: sun-faded red paint with scratches and rust (heavier on the sills, arch lips
and lower door edges), a mismatched faded green junkyard door on the right, a dent in
the left rear quarter panel, unpainted black plastic bumpers with rub strips, rectangular
headlamps, small tail lamps, black plastic mirrors, a roof-pillar antenna and a rusty
tailpipe.
Interactive: 4 corners (travel, front steer, spin), 2 doors, hood, hatch (with the rear
glass and the rear wiper), steering wheel, 2 windshield wipers, rear wiper,
lamps (head, tail, brake, indicators, reverse).
"""
import math

from ..scene import Model
from ..carbody import CarBody, Panel
from ..parts import standard_materials, corner, WheelSpec, steering_wheel, seat, mirror
from .. import mesh as M
from ..vec import norm, sub, add, mul, cross, dot, frame_z_to

WB, TRACK = 2.35, 1.37
XF, XR = WB / 2, -WB / 2
X_REAR, X_FRONT = -1.78, 1.86

_STEEL = dict(radius=0.274, width=0.155, rim_radius=0.165, tyre="road", rim="steel",
              rim_metallic=0.5)
TYRE_F = WheelSpec(**_STEEL, rim_color=(0.30, 0.31, 0.32), caliper_color=(0.22, 0.17, 0.13))
# rear: drum brakes (no disc/caliper; a drum is modelled on the suspension node)
TYRE_R = WheelSpec(**_STEEL, rim_color=(0.30, 0.31, 0.32), disc=False)
TYRE_R_NOCAP = WheelSpec(**_STEEL, rim_color=(0.26, 0.20, 0.16), disc=False, hubcap=False)

# door: (name, side, x_rear_edge, x_front_edge)
DOORS = (("door_L", 1, -0.32, 0.84), ("door_R", -1, -0.32, 0.84))
HATCH_X = -1.53        # hatch hinge line (rear roof edge)
HATCH_ZC = 0.52        # hatch lower edge on the rear panel
HATCH_YC = 0.55        # hatch half-width on the rear panel
HOOD_X = 0.97          # hood rear edge (cowl)
HOOD_LS = 0.72         # hood side edges: section position (grid-aligned)


# ------------------------------------------------------------------ local helpers

def _clip(poly, a, b, c):
    """Sutherland-Hodgman: keep the part of a 2D polygon where a*y + b*z + c >= 0."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        fp, fq = a * p[0] + b * p[1] + c, a * q[0] + b * q[1] + c
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fq >= 0):
            t = fp / (fp - fq)
            out.append((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))
    return out


def _clip_all(poly, planes):
    for pl in planes:
        poly = _clip(poly, *pl)
        if len(poly) < 3:
            return []
    return poly


def _remove_cap(mb, key, x_cap):
    """Remove the flat end-cap triangles CarBody put at x = x_cap (all three vertices on it)."""
    p = mb.prims.get(key)
    if not p:
        return
    keep = []
    for k in range(0, len(p.idx), 3):
        tri = p.idx[k:k + 3]
        if all(abs(p.pos[i][0] - x_cap) < 1e-6 for i in tri):
            continue
        keep += tri
    p.idx[:] = keep


def _inner_skin(m, part, mats, depth=0.03):
    """Like CarBody's inner skins (interior_trim 3 cm inside), for extra paint materials."""
    mb = m.mb
    for mat in mats:
        src = mb.prims.get((part, mat))
        if not src:
            continue
        dst = mb.prim(part, "interior_trim")
        base = len(dst.pos)
        for v, nn in zip(src.pos, src.nrm):
            dst.pos.append(sub(v, mul(nn, depth)))
            dst.nrm.append(mul(nn, -1))
        dst.uv += src.uv
        for k in range(0, len(src.idx), 3):
            dst.idx += (base + src.idx[k], base + src.idx[k + 2], base + src.idx[k + 1])


def _dent(mb, keys, center, normal, radius, depth):
    """Push vertices near `center` along -normal (smooth falloff) and re-derive their normals."""
    for key in keys:
        p = mb.prims.get(key)
        if not p:
            continue
        moved = set()
        for i, v in enumerate(p.pos):
            d = math.dist(v, center)
            if d < radius and dot(p.nrm[i], normal) > 0.5:
                w = (1 - (d / radius) ** 2) ** 2
                p.pos[i] = sub(v, mul(normal, depth * w))
                moved.add(i)
        if not moved:
            continue
        acc = {i: (0.0, 0.0, 0.0) for i in moved}
        for k in range(0, len(p.idx), 3):
            a, b, c = p.idx[k:k + 3]
            if a in acc or b in acc or c in acc:
                fn = cross(sub(p.pos[b], p.pos[a]), sub(p.pos[c], p.pos[a]))
                for i in (a, b, c):
                    if i in acc:
                        acc[i] = add(acc[i], fn)
        for i, n in acc.items():
            nn = norm(n)
            if nn != (0.0, 0.0, 0.0):
                p.nrm[i] = nn


def _outline(body, z, xc, end, count=56):
    """Plan outline of the lower body at height z, from the left side (y+) around the
    front (end=+1) or rear (end=-1) to the right side, rays cast from (xc, 0)."""
    raw = []
    for k in range(241):
        th = math.pi / 2 - math.pi * k / 240
        d = (end * math.cos(th), math.sin(th))
        lo, hi = 0.0, 2.5
        for _ in range(40):
            mid = (lo + hi) / 2
            if body.contains(xc + d[0] * mid, d[1] * mid, z):
                lo = mid
            else:
                hi = mid
        raw.append((xc + d[0] * lo, d[1] * lo))
    seg = [math.dist(raw[i], raw[i + 1]) for i in range(len(raw) - 1)]
    total = sum(seg)
    out, i, acc = [], 0, 0.0
    for k in range(count + 1):
        t = total * k / count
        while i < len(seg) - 1 and acc + seg[i] < t:
            acc += seg[i]
            i += 1
        f = (t - acc) / seg[i] if seg[i] > 1e-12 else 0.0
        a, b = raw[i], raw[i + 1]
        out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    return out


def _sweep(mb, path2, z, profile, mat, part="", xc=0.0):
    """Sweep a 2D profile (a = outward horizontal, b = up) along a plan path at height z."""
    secs = []
    n = len(path2)
    for i in range(n):
        p = path2[i]
        q0, q1 = path2[max(0, i - 1)], path2[min(n - 1, i + 1)]
        t = norm((q1[0] - q0[0], q1[1] - q0[1], 0.0))
        o = (t[1], -t[0], 0.0)
        if o[0] * (p[0] - xc) + o[1] * p[1] < 0:
            o = mul(o, -1)
        secs.append([(p[0] + o[0] * a, p[1] + o[1] * a, z + b) for (a, b) in profile])
    return M.loft(mb, secs, mat, part)


# ------------------------------------------------------------------------ build

def build():
    m = Model("car_shitbox", "Shitbox (worn-out hatchback)",
              "Worn-out late-80s style 3-door hatchback supermini, 3.74 m, 155/70 R13 steel wheels, "
              "front-engine front-wheel-drive layout.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    standard_materials(m)
    red, green = (0.62, 0.13, 0.09), (0.36, 0.50, 0.36)
    m.mat("paint", color=(1, 1, 1), tex=("paint", {"color": red, "fade": 0.6, "scratches": 0.12, "rust": 0.12,
                                                   "dirt": 0.25, "rough": 0.45, "seed": 3}),
          tile=0.9, double=True, role="paint")
    m.mat("paint_rusty", color=(1, 1, 1), tex=("paint", {"color": red, "fade": 0.6, "scratches": 0.12, "rust": 0.85,
                                                         "dirt": 0.45, "rough": 0.5, "seed": 5}),
          tile=0.6, double=True, role="paint")
    m.mat("paint_door", color=(1, 1, 1), tex=("paint", {"color": green, "fade": 0.55, "scratches": 0.15,
                                                        "rust": 0.15, "dirt": 0.25, "rough": 0.45, "seed": 8}),
          tile=0.9, double=True, role="paint")
    m.mat("paint_door_rusty", color=(1, 1, 1), tex=("paint", {"color": green, "fade": 0.55, "scratches": 0.15,
                                                              "rust": 0.85, "dirt": 0.45, "rough": 0.5, "seed": 9}),
          tile=0.6, double=True, role="paint")
    m.mat("seat", color=(1, 1, 1), tex=("fabric", {"color": (0.22, 0.2, 0.17)}), tile=0.2)
    m.mat("rust_metal", color=(0.30, 0.16, 0.08), metallic=0.3, roughness=0.9)
    m.mat("rub_strip", color=(0.09, 0.09, 0.09), roughness=0.8)

    R = TYRE_F.radius
    y_out, y_in = TRACK / 2 + TYRE_F.width / 2, TRACK / 2 - TYRE_F.width / 2
    wheels = [dict(x=XF, z=R, r=R, y_out=y_out, y_in=y_in), dict(x=XR, z=R, r=R, y_out=y_out, y_in=y_in)]

    body = CarBody(
        m, x_rear=X_REAR, x_front=X_FRONT,
        W=[(X_REAR, 0.775), (-1.55, 0.795), (-1.1, 0.80), (1.1, 0.80), (1.6, 0.79), (X_FRONT, 0.765)],
        zb=[(X_REAR, 0.34), (-1.55, 0.25), (-1.0, 0.18), (0.9, 0.18), (1.55, 0.25), (X_FRONT, 0.30)],
        zt=[(X_REAR, 0.87), (-1.6, 0.90), (-1.0, 0.905), (0.95, 0.875), (1.3, 0.815), (1.7, 0.755),
            (X_FRONT, 0.71)],
        exp=[(X_REAR, 6.0), (-1.5, 8.0), (1.5, 8.0), (X_FRONT, 5.5)],
        end_r=(0.05, 0.10), wheels=wheels, step=0.024, around=160,
        green=dict(x0=-1.755, x1=0.95,
                   zroof=[(-1.755, 0.87), (-1.69, 1.11), (-1.61, 1.29), (-1.50, 1.36), (-1.2, 1.375),
                          (0.2, 1.37), (0.38, 1.345), (0.95, 0.87)],
                   wb=[(-1.755, 0.72), (-1.5, 0.76), (0.6, 0.765), (0.95, 0.74)],
                   wr=[(-1.755, 0.565), (-1.5, 0.645), (0.3, 0.665), (0.95, 0.66)],
                   r=0.05, crown=0.03))
    W = body.W

    # ---- opening panels
    def door_inside(side, x0, x1):
        def f(c, n):
            return (x0 - 0.012 < c[0] < x1 + 0.012 and c[1] * side > 0 and n[1] * side > 0.3 and n[2] < 0.75
                    and c[2] > 0.21 and not body.in_arch(c))
        return f

    def door_border(side, x0, x1):
        def f(c, n):
            return c[0] < x0 + 0.008 or c[0] > x1 - 0.008 or c[2] < 0.245
        return f

    for name, side, x0, x1 in DOORS:
        body.panels.append(Panel(name, door_inside(side, x0, x1), door_border(side, x0, x1)))
    body.panels.append(Panel(
        "hood", lambda c, n: c[0] > HOOD_X and body.ls > HOOD_LS and n[2] > 0.3,
        lambda c, n: c[0] < HOOD_X + 0.022 or body.ls < HOOD_LS + 0.02 or n[2] < 0.45, inner_skin=False))
    # lower part of the hatch: the rear panel above the bumper (the flat end cap is rebuilt below)
    body.panels.append(Panel(
        "hatch", lambda c, n: c[0] < X_REAR + 0.09 and c[2] > HATCH_ZC - 0.01 and abs(c[1]) < HATCH_YC + 0.01,
        lambda c, n: c[2] < HATCH_ZC + 0.006 or abs(c[1]) > HATCH_YC - 0.006))

    def rusty(c, n):
        x, y, z = c
        if z < 0.33:                                   # sills, lower door edges, valances
            return True
        for w in body.wheels:                          # arch lips
            d = math.hypot(x - w["x"], z - w["z"])
            ra = w["r"] + body.arch_clear
            if ra - 0.01 < d < ra + 0.075 and z < w["z"] + 0.30:
                return True
        if x < X_REAR + 0.15 and z < 0.46:             # rear valance below the hatch
            return True
        if abs(y) > 0.5 and n[2] > 0.4 and x < -1.5:   # water trap at the rear ledge
            return True
        return False

    def lower(c, n, b):
        if n[2] < -0.6:
            return "underbody"
        if b.in_arch(c):
            return "arch_liner"
        r = rusty(c, n)
        res = b.panel_face(c, n, "paint_rusty" if r else "paint")
        if isinstance(res, tuple) and res[0] == "door_R":
            res = ("door_R", "paint_door_rusty" if r else "paint_door")
        return res

    def green_cl(c, n, b):
        x, y, z = c
        seg, pos = b.gseg, b.gpos
        if seg in ("roof", "corner"):
            if x < HATCH_X:                                     # hatch: rear glass in a painted frame
                if x > HATCH_X - 0.024 or (seg == "corner" and pos < 0.25):
                    return "gap"
                if seg == "roof" and x < HATCH_X - 0.03 and z > 0.94:
                    # rear glass with a black ceramic border
                    if pos > 0.14 and x < HATCH_X - 0.06 and z > 0.97:
                        return ("hatch", "glass")
                    return ("hatch", "trim_black")
                return ("hatch", "paint")
            if x > 0.40 and seg == "roof":                      # windshield, black ceramic border
                if pos > 0.12 and 0.46 < x < 0.915:
                    return "glass"
                return "trim_black"
            return "paint"
        if seg == "side":
            side = 1 if y > 0 else -1
            for name, s, x0, x1 in DOORS:
                if s == side and x0 < x < x1:
                    if 0.11 < pos < 0.9 and x0 + 0.03 < x < x1 - 0.03:
                        return (name, "glass")
                    return (name, "trim_black")
            if -0.40 < x < -0.32:
                return "trim_black"                             # B-pillar
            if -1.44 < x < -0.40:                               # fixed rear quarter window
                if 0.11 < pos < 0.9 and -1.42 < x < -0.42:
                    return "glass"
                return "trim_black" if pos > 0.12 else "paint"
            return "paint"
        return "paint"

    body.build(lower, green_cl)

    # ---- rebuild the flat rear end cap: hatch part above HATCH_ZC, body part around it
    mb = m.mb
    _remove_cap(mb, ("", "paint"), X_REAR)
    loop = [(p[1], p[2]) for p in body._lower_section(X_REAR)]
    g = 0.004
    zc, yc = HATCH_ZC, HATCH_YC
    pieces = [
        ("hatch", "paint", [(0, 1, -(zc + g)), (-1, 0, yc - g), (1, 0, yc - g)]),
        ("", "paint_rusty", [(0, -1, zc - g)]),
        ("", "paint", [(0, 1, -(zc - g)), (1, 0, -(yc + g))]),
        ("", "paint", [(0, 1, -(zc - g)), (-1, 0, -(yc + g))]),
    ]
    back = (-1.0, 0.0, 0.0)
    for part, mat, planes in pieces:
        poly = _clip_all(loop, planes)
        if poly:
            pts = [(X_REAR, a, b) for (a, b) in poly]
            mb.polygon(pts, mat, part, want_normal=back)
            if part == "hatch":
                mb.polygon([(X_REAR + 0.03, a, b) for (a, b) in poly], "interior_trim", part, want_normal=(1, 0, 0))
    mb.polygon([(X_REAR + 0.004, a, b) for (a, b) in loop], "gap", "", want_normal=back)

    # (inner skins for every paint on the panels, incl. rusty and the junkyard door: CarBody)

    # a dent in the left rear quarter panel
    _dent(mb, [("", "paint"), ("", "paint_rusty")], (-0.62, W(-0.62), 0.58), (0, 1, 0), 0.18, 0.03)

    # ---- lamps, grille, plate
    rr = M.rrect
    zh = 0.565
    body.patch(rr(0.60, 0.115, 0.01, 2, 0.0, zh), "trim_black", "front", skirt_mat="plastic_black", offset=0.003)
    for k in range(3):  # horizontal grille slats
        body.patch(rr(0.58, 0.014, 0.004, 2, 0.0, zh - 0.035 + 0.035 * k), "plastic_black", "front", offset=0.009,
                   skirt=0)
    for s in (1, -1):
        ind = "light_indicator_left" if s > 0 else "light_indicator_right"
        body.patch(rr(0.30, 0.13, 0.012, 2, s * 0.465, zh), "light_head", "front", skirt_mat="plastic_black",
                   offset=0.005)
        body.patch(rr(0.075, 0.10, 0.01, 2, s * 0.675, zh), ind, "front", skirt_mat="plastic_black", offset=0.004)
        # rear cluster: indicator (top), tail + brake, reverse (bottom)
        yl = s * 0.66
        body.patch(rr(0.16, 0.26, 0.012, 2, yl, 0.70), "trim_black", "rear", skirt_mat="plastic_black", offset=0.003)
        body.patch(rr(0.145, 0.06, 0.006, 2, yl, 0.795), ind, "rear", offset=0.007, skirt=0)
        body.patch(rr(0.145, 0.095, 0.006, 2, yl, 0.707), "light_tail", "rear", offset=0.007, skirt=0)
        body.patch(rr(0.12, 0.045, 0.006, 2, yl, 0.707), "light_brake", "rear", offset=0.009, skirt=0)
        body.patch(rr(0.145, 0.05, 0.006, 2, yl, 0.62), "light_reverse", "rear", offset=0.007, skirt=0)
    body.patch(rr(0.46, 0.10, 0.008, 2, 0.0, 0.63), "plate", "rear", part="hatch", skirt_mat="trim_black",
               offset=0.004)
    body.patch(rr(0.16, 0.035, 0.01, 2, 0.0, 0.73), "plastic_black", "rear", part="hatch", offset=0.006)  # handle

    # ---- bumpers: unpainted black plastic with a rub strip, wrapped round to the arches
    zbf, zbr = 0.37, 0.40
    for end, xc, zz in ((1, 1.56, zbf), (-1, -1.55, zbr)):
        path = _outline(body, zz, xc, end)
        _sweep(mb, path, zz, M.rrect(0.11, 0.19, 0.035, 3, -0.005, 0.0), "plastic_black", xc=xc)
        _sweep(mb, path[2:-2], zz + 0.05, M.rrect(0.03, 0.04, 0.01, 2, 0.056, 0.0), "rub_strip", xc=xc)
        if end > 0:  # front plate on the bumper face, below the rub strip
            xp = path[len(path) // 2][0] + 0.05
            M.box(mb, (xp + 0.002, 0.0, zz - 0.035), (0.004, 0.47, 0.105), "trim_black")
            M.box(mb, (xp + 0.005, 0.0, zz - 0.035), (0.004, 0.46, 0.095), "plate")

    # ---- corners
    for key, x, y, steer, spec in (("FL", XF, TRACK / 2, True, TYRE_F), ("FR", XF, -TRACK / 2, True, TYRE_F),
                                   ("RL", XR, TRACK / 2, False, TYRE_R),
                                   ("RR", XR, -TRACK / 2, False, TYRE_R_NOCAP)):
        corner(m, key, (x, y, R), spec, steer=steer, steer_limits=(-38, 38), travel=(-0.08, 0.08),
               tag=key if spec is not TYRE_R_NOCAP else "RR_nocap")
        if not steer:  # drum brake + backing plate (does not spin)
            s = 1 if y > 0 else -1
            xf = frame_z_to((0.0, float(s), 0.0), (x, y, R))
            M.lathe(mb, [(0.115, -0.045), (0.115, 0.01), (0.03, 0.02), (0.001, 0.02)], "rust_metal",
                    f"susp_{key}", xf, count=24, orient=None, closed_profile=False)

    # ---- panel pivots (geometry already tagged with their names)
    for name, side, x0, x1 in DOORS:
        m.node(name, None, (x1, side * (W(x1) - 0.02), 0.60), m.revolute((0, 0, -side), 0, 68, "door opening angle"))
        hx = x0 + 0.10
        M.rounded_box(mb, (hx, side * (W(hx) + 0.004), 0.80), (0.12, 0.02, 0.03), 0.008, "plastic_black", name)
        # rub strip on the door
        body.patch(rr(x1 - x0 - 0.04, 0.035, 0.012, 2, (x0 + x1) / 2, 0.50), "rub_strip",
                   "left" if side > 0 else "right", part=name, offset=0.008, skirt_mat="rub_strip")
    m.node("hood", None, (HOOD_X, 0.0, body.zt(HOOD_X)), m.revolute((0, -1, 0), 0, 55, "hood opening angle"))
    hz = body.green["zroof"](HATCH_X) + body.green["crown"]
    m.node("hatch", None, (HATCH_X, 0.0, hz), m.revolute((0, 1, 0), 0, 75, "hatch opening angle"))

    # black plastic mirrors on the doors
    for name, side in (("door_L", 1), ("door_R", -1)):
        mirror(m, name, (0.76, side * (W(0.76) - 0.01), 0.95), side, size=(0.07, 0.13, 0.09), arm=0.015,
               body_mat="plastic_black")

    # ---- interior (static, on the body)
    M.box(mb, (-0.35, 0, 0.28), (2.2, 1.45, 0.04), "interior")                    # floor
    M.rounded_box(mb, (0.68, 0, 0.83), (0.34, 1.40, 0.18), 0.04, "interior")       # dashboard
    M.box(mb, (0.88, 0, 0.60), (0.06, 1.36, 0.40), "interior")                     # firewall
    for yy in (0.36, -0.36):
        seat(m, "", (-0.15, yy, 0.50), width=0.46, recline=0.30)
    seat(m, "", (-1.05, 0.0, 0.48), width=1.25, recline=0.25, headrest=False)
    M.box(mb, (-1.52, 0, 0.90), (0.36, 1.3, 0.02), "interior")                    # parcel shelf
    steering_wheel(m, "steering_wheel", None, (0.42, 0.36, 0.84), (0.9, 0.0, -0.44), radius=0.19, spokes=2)

    # engine under the hood
    M.rounded_box(mb, (1.40, 0, 0.55), (0.45, 0.62, 0.36), 0.04, "engine")
    M.rounded_box(mb, (1.40, 0.05, 0.75), (0.30, 0.45, 0.06), 0.02, "rust_metal")  # valve cover
    M.cylinder(mb, (1.6, -0.35, 0.65), (1.6, -0.35, 0.72), 0.12, "plastic_black", count=16)  # air filter

    # ---- wipers: rest along the cowl pointing +y, axis = windshield normal (+ sweeps up)
    zr = body.green["zroof"]
    a0, a1 = (0.95, zr(0.95)), (0.45, zr(0.45))
    ws_n = norm((a1[1] - a0[1], 0.0, a0[0] - a1[0]))
    for name, py, L in (("wiper_L", 0.05, 0.50), ("wiper_R", -0.52, 0.46)):
        px = 0.905
        p = (px, py, zr(px) + body.green["crown"] * (1 - (py / 0.66) ** 2) + 0.012)
        m.node(name, None, p, m.revolute(ws_n, 0, 100, "wiper sweep"))
        M.box(mb, (p[0] - 0.012, py + L / 2, p[2] + 0.01), (0.016, L, 0.01), "trim_black", name)
        M.box(mb, (p[0] - 0.022, py + L / 2 + 0.03, p[2] + 0.003), (0.012, L * 0.85, 0.01), "plastic_black", name)
        M.cylinder(mb, (p[0], p[1], p[2] - 0.015), (p[0], p[1], p[2] + 0.018), 0.016, "trim_black", name, count=10)

    # rear wiper: child of the hatch, pivot below the rear glass, rests pointing -y (+ sweeps up)
    r0, r1 = (-1.74, zr(-1.74)), (-1.60, zr(-1.60))
    rn = norm((-(r1[1] - r0[1]), 0.0, r1[0] - r0[0]))   # outward (rearward-up) normal of the rear glass
    pz = 1.00
    # x on the glass at height pz (bisect along the roof curve)
    lo, hi = -1.74, -1.55
    for _ in range(40):
        mid = (lo + hi) / 2
        if zr(mid) < pz:
            lo = mid
        else:
            hi = mid
    rx = lo
    rp = add((rx, 0.0, pz), mul(rn, 0.02))
    m.node("wiper_rear", "hatch", rp, m.revolute(rn, 0, 110, "rear wiper sweep"))
    L = 0.40
    arm_c = add(rp, (0.0, -L / 2, 0.0))
    xf = frame_z_to(rn, arm_c)
    M.box(mb, (0, 0, 0.008), (0.016, L, 0.01), "trim_black", "wiper_rear", xf=_yaw_fix(xf, arm_c))
    M.box(mb, (0, -0.02, 0.0), (0.012, L * 0.85, 0.01), "plastic_black", "wiper_rear", xf=_yaw_fix(xf, arm_c))
    M.cylinder(mb, sub(rp, mul(rn, 0.02)), add(rp, mul(rn, 0.015)), 0.016, "trim_black", "wiper_rear", count=10)

    # ---- exhaust tailpipe (rusty), antenna
    ex0, ex1 = (-1.45, -0.42, 0.21), (-1.83, -0.42, 0.21)
    M.cylinder(mb, ex0, ex1, 0.022, "rust_metal", count=12)
    M.cylinder(mb, (ex1[0] + 0.002, ex1[1], ex1[2]), (ex1[0] + 0.05, ex1[1], ex1[2]), 0.017, "gap", count=12,
               caps=False)
    ab = (0.90, -0.745, body.zt(0.90) - 0.02)
    M.cylinder(mb, ab, add(ab, (0.0, 0.0, 0.03)), 0.012, "plastic_black", count=8)
    M.cylinder(mb, ab, add(ab, (-0.30, 0.0, 0.68)), 0.0025, "chrome", count=5)

    # ---- sockets
    m.socket("socket_driver_eye", None, (-0.22, 0.36, 1.14), "driver eye point (camera)")
    m.socket("socket_tow_front", None, (1.90, 0.30, 0.26), "front towing eye")
    m.socket("socket_tow_rear", None, (-1.82, 0.30, 0.28), "rear towing eye")
    m.socket("socket_exhaust", None, ex1, "exhaust outlet (points -x)")

    m.notes += [
        "Generic worn-out late-80s style supermini hatchback; dimensions are class-typical, "
        "not a specific production car.",
        "Front-wheel drive, front-engine (transverse) layout; front discs, rear drums.",
        "Rear right wheel has no hubcap (wheel tag RR_nocap); visual only.",
        "door_R is a mismatched green junkyard door; it opens like door_L (mirror image).",
        "The hatch includes the rear glass, the plate and the rear wiper (wiper_rear is its child).",
        "Bumpers are unpainted plastic protruding ~5 cm from the body: overall length 3.74 m; "
        "the antenna tip reaches z = 1.54 m.",
    ]
    return m


def _yaw_fix(xf, origin):
    """Like frame_z_to(z, origin) but with local y = model +y projected onto the plane (stable yaw)."""
    z = (xf[0][2], xf[1][2], xf[2][2])
    y = norm(sub((0.0, 1.0, 0.0), mul(z, dot((0.0, 1.0, 0.0), z))))
    x = cross(y, z)
    return ((x[0], y[0], z[0], origin[0]), (x[1], y[1], z[1], origin[1]), (x[2], y[2], z[2], origin[2]))
