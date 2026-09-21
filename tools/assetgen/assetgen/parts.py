"""Reusable sub-assemblies: wheel corners, steering wheels, seats, mirrors, lamps.

Corner hierarchy made by `corner()` (XX = FL, FR, RL, RR, 1L, ...):
    susp_XX   prismatic along z (wheel travel; pivot = wheel centre at rest)
      steer_XX  revolute about z (only for steered wheels; vertical kingpin through
                the wheel centre, or kingpin_offset inboard) - carries the brake caliper
        wheel_XX  continuous about y (spin) - tyre, rim, brake disc
Non-steered corners skip steer_XX; the caliper then sits on susp_XX.
"""
import math

from .vec import add, sub, mul, norm, cross, frame_z_to, xform_point, compose, translate, rot_axis
from . import mesh as M


# ------------------------------------------------------------ materials

def standard_materials(model, paint=None):
    """Materials shared by most road vehicles. Returns nothing; use names."""
    model.mat("glass", color=(0.16, 0.2, 0.22), alpha=0.6, metallic=0.0, roughness=0.0, blend=True,
              role="glass")
    model.mat("chrome", color=(0.9, 0.9, 0.9), metallic=1.0, roughness=0.15)
    model.mat("trim_black", color=(0.03, 0.03, 0.035), roughness=0.55)
    model.mat("plastic_black", color=(0.05, 0.05, 0.05), roughness=0.7, tex=("rubber", {"seed": 21, "shade": 0.05}),
              tile=0.3)
    model.mat("underbody", color=(0.05, 0.05, 0.05), roughness=0.9)
    model.mat("arch_liner", color=(0.03, 0.03, 0.03), roughness=0.95)
    model.mat("gap", color=(0.0, 0.0, 0.0), roughness=1.0)
    model.mat("interior", color=(0.08, 0.08, 0.085), roughness=0.8)
    model.mat("interior_trim", color=(0.12, 0.12, 0.12), roughness=0.75, double=True)
    model.mat("grille", color=(1, 1, 1), tex=("honeycomb", {}), tile=0.12, roughness=0.6)
    model.mat("brake_disc", color=(0.45, 0.45, 0.45), metallic=1.0, roughness=0.45)
    model.mat("engine", color=(0.2, 0.2, 0.21), metallic=0.6, roughness=0.5)
    model.mat("mirror", color=(0.9, 0.9, 0.9), metallic=1.0, roughness=0.02)
    model.mat("plate", color=(0.92, 0.92, 0.9), roughness=0.4)
    lamp_materials(model)


def lamp_materials(model):
    """Lamp lenses: emission off in the file, the rig says what to switch on."""
    model.mat("light_head", color=(0.85, 0.87, 0.9), alpha=1.0, metallic=0.3, roughness=0.1,
              tex=("lens", {}), tile=0.05, emissive=(1.0, 0.95, 0.85), light="head", light_strength=8.0)
    model.mat("light_tail", color=(0.45, 0.02, 0.02), roughness=0.15, emissive=(1.0, 0.03, 0.02),
              light="tail", light_strength=2.0)
    model.mat("light_brake", color=(0.5, 0.02, 0.02), roughness=0.15, emissive=(1.0, 0.02, 0.01),
              light="brake", light_strength=8.0)
    model.mat("light_indicator_left", color=(0.8, 0.45, 0.05), roughness=0.15, emissive=(1.0, 0.5, 0.0),
              light="indicator_left", light_strength=6.0)
    model.mat("light_indicator_right", color=(0.8, 0.45, 0.05), roughness=0.15, emissive=(1.0, 0.5, 0.0),
              light="indicator_right", light_strength=6.0)
    model.mat("light_reverse", color=(0.85, 0.85, 0.85), roughness=0.1, emissive=(1.0, 1.0, 1.0),
              light="reverse", light_strength=5.0)


# --------------------------------------------------------------- wheels

class WheelSpec:
    """radius/width of the tyre, rim_radius = bead seat radius.
    tyre: road | slick | farm | trailer ; rim: steel | alloy5 | alloy_multi | centerlock | farm
    """

    def __init__(self, radius, width, rim_radius, tyre="road", rim="alloy5", rim_color=(0.75, 0.76, 0.78),
                 rim_metallic=0.9, disc=True, caliper_color=(0.15, 0.15, 0.15), lugs=22, lug_height=0.05,
                 spokes=5, rim_offset=0.0, hubcap=True, sidewall_text=False):
        self.radius, self.width, self.rim_radius = radius, width, rim_radius
        self.tyre, self.rim = tyre, rim
        self.rim_color, self.rim_metallic = rim_color, rim_metallic
        self.disc, self.caliper_color = disc, caliper_color
        self.lugs, self.lug_height = lugs, lug_height
        self.spokes = spokes
        self.rim_offset = rim_offset  # dish: + moves the rim face outward
        self.hubcap = hubcap


def _wheel_frame(center, side):
    """Local frame: z = outward axle direction, origin = wheel centre."""
    return frame_z_to((0.0, float(side), 0.0), center)


def build_wheel(model, part, center, spec, side, tag):
    """Tyre + rim + disc into `part`. side = +1 (left, outer face +y) or -1."""
    mb = model.mb
    R, W, Rr = spec.radius, spec.width, spec.rim_radius
    if spec.tyre == "farm":
        R -= spec.lug_height  # nominal radius is over the lugs (as tyre sizes are quoted)
    xf = _wheel_frame(center, side)
    hw = W / 2
    sh = min(0.35 * (R - Rr), 0.3 * W)  # shoulder rounding radius
    # --- tyre profile: outer bead -> outer sidewall -> crown (+a -> -a) -> inner sidewall -> inner bead
    prof = []
    bulge = 0.04 * W if spec.tyre != "slick" else 0.02 * W
    ns = 6
    for k in range(ns + 1):  # outer sidewall, bead to shoulder start
        t = k / ns
        r = Rr + (R - sh - Rr) * t
        a = hw * 0.92 + bulge * math.sin(math.pi * t)
        prof.append((r, a))
    crown_i0 = len(prof)
    for k in range(1, 5):  # shoulder
        t = k / 4 * math.pi / 2
        prof.append((R - sh + sh * math.sin(t), hw - sh + sh * math.cos(t) + bulge * 0.0))
    nc = 8
    for k in range(1, nc):
        prof.append((R, (hw - sh) - 2 * (hw - sh) * k / nc))
    for k in range(0, 5):
        t = k / 4 * math.pi / 2
        prof.append((R - sh + sh * math.cos(t), -(hw - sh) - sh * math.sin(t)))
    crown_i1 = len(prof) - 1
    for k in range(1, ns + 1):
        t = k / ns
        r = R - sh - (R - sh - Rr) * t
        a = -(hw * 0.92 + bulge * math.sin(math.pi * (1 - t)))
        prof.append((r, a))
    count = 64 if R < 0.5 else 96

    circ = 2 * math.pi * R
    tread_mat = f"tyre_tread_{tag}"
    side_mat = "tyre_sidewall"
    model.mat(side_mat, color=(1, 1, 1), tex=("rubber", {}), tile=0.4)
    if spec.tyre in ("road", "trailer"):
        pitch = 0.03 if spec.tyre == "road" else 0.06
        n_pitch = max(1, round(circ / pitch))
        crown_w = sum(math.dist(prof[i], prof[i + 1]) for i in range(crown_i0, crown_i1))
        model.mat(tread_mat, color=(1, 1, 1), tex=("tread_car", {"grooves": 4 if spec.tyre == "road" else 3}),
                  tile=(crown_w, circ / n_pitch))
    elif spec.tyre == "slick":
        model.mat(tread_mat, color=(1, 1, 1), tex=("tread_slick", {}), tile=0.5)
    else:  # farm: plain rubber crown; lugs are geometry
        model.mat(tread_mat, color=(1, 1, 1), tex=("rubber", {"seed": 4}), tile=0.5)

    arc = [0.0]
    for i in range(1, len(prof)):
        arc.append(arc[-1] + math.dist(prof[i - 1], prof[i]))
    a0 = arc[crown_i0]

    def uv(i, j):
        return (arc[i] - a0, j / count * circ)

    def cls(i, j, c, n):
        return tread_mat if crown_i0 <= i < crown_i1 else side_mat

    M.lathe(mb, [(r, a) for (r, a) in prof], tread_mat, part, xf, count=count, classify=cls, orient=False, uv=uv)

    if spec.tyre == "farm":
        _farm_lugs(model, part, xf, spec, tread_mat)

    # --- rim
    rim_mat = f"rim_{tag}"
    model.mat(rim_mat, color=spec.rim_color, metallic=spec.rim_metallic, roughness=0.3, double=True)
    _rim(model, part, xf, spec, rim_mat)

    # --- brake disc (spins with the wheel)
    if spec.disc:
        dr = Rr * 0.82
        off = -0.02 + spec.rim_offset * 0.5
        M.lathe(mb, [(dr, off + 0.014), (dr * 0.55, off + 0.014), (dr * 0.55, off - 0.014), (dr, off - 0.014)],
                "brake_disc", part, xf, count=48, closed_profile=True)


def _rim(model, part, xf, spec, rim_mat):
    mb = model.mb
    Rr, W = spec.rim_radius, spec.width
    hw = W / 2 * 0.9
    lip = Rr * 1.05
    face = spec.rim_offset + hw * 0.35  # axial position of the spoke face
    # barrel: outer lip -> seat -> inner lip (seen from both sides: double-sided material)
    barrel = [(lip, hw), (Rr, hw - 0.01), (Rr * 0.97, hw * 0.6), (Rr * 0.97, -hw * 0.6), (Rr, -hw + 0.01),
              (lip, -hw)]
    M.lathe(mb, barrel, rim_mat, part, xf, count=48, orient=True)
    hub_r = Rr * 0.22
    if spec.rim == "steel" or spec.rim == "farm":
        # dished disc with ventilation holes, and a centre hub
        prof = [(Rr * 0.97, face - 0.03), (Rr * 0.8, face), (Rr * 0.55, face + 0.01), (hub_r * 1.3, face + 0.015),
                (hub_r, face + 0.03)]
        holes = 6 if spec.rim == "steel" else 10
        count = 60

        def cls(i, j, c, n):
            if i == 2:
                seg = (j % (count // holes))
                if 2 <= seg <= (count // holes) - 3:
                    return None
            return rim_mat
        M.lathe(mb, prof, rim_mat, part, xf, count=count, classify=cls, orient=True)
        # wheel nuts / hub
        M.lathe(mb, [(hub_r, face + 0.03), (hub_r * 0.8, face + 0.05), (0.001, face + 0.055)], "chrome"
                if spec.rim == "steel" else rim_mat, part, xf, count=24, orient=True)
        if spec.rim == "steel" and spec.hubcap:
            model.mat("hubcap", color=(0.7, 0.72, 0.74), metallic=0.4, roughness=0.35)
            M.lathe(mb, [(Rr * 0.99, face - 0.01), (Rr * 0.9, face + 0.035), (Rr * 0.5, face + 0.05), (0.001, face + 0.055)],
                    "hubcap", part, xf, count=48, orient=True)
        return
    # alloy / centerlock: centre hub + spokes
    M.lathe(mb, [(hub_r * 1.4, face - 0.01), (hub_r * 1.3, face + 0.02), (hub_r, face + 0.03), (0.001, face + 0.032)],
            rim_mat, part, xf, count=32, orient=True)
    n = spec.spokes
    split = spec.rim in ("alloy_multi", "centerlock")
    for k in range(n):
        ang = 2 * math.pi * k / n
        for s in ((-0.09, 0.09) if split else (0.0,)):
            a = ang + s * (1.0 if split else 0.0)
            _spoke(mb, part, xf, a, hub_r * 1.2, Rr * 0.97, face, rim_mat,
                   width=(0.022 if split else 0.045) * (Rr / 0.23), depth=0.03)
    if spec.rim == "centerlock":
        model.mat("centerlock_nut", color=(0.85, 0.1, 0.1), metallic=0.8, roughness=0.3)
        M.lathe(mb, [(hub_r * 0.75, face + 0.03), (hub_r * 0.75, face + 0.07), (hub_r * 0.5, face + 0.08),
                     (0.001, face + 0.08)], "centerlock_nut", part, xf, count=12, orient=True)
    else:
        for k in range(5):  # wheel bolts
            a = 2 * math.pi * k / 5 + 0.3
            p = xform_point(xf, (hub_r * 0.65 * math.cos(a), hub_r * 0.65 * math.sin(a), face + 0.03))
            q = xform_point(xf, (hub_r * 0.65 * math.cos(a), hub_r * 0.65 * math.sin(a), face + 0.045))
            M.cylinder(mb, p, q, 0.009, "chrome", part, count=6)


def _spoke(mb, part, xf, ang, r0, r1, face, mat, width, depth):
    c, s = math.cos(ang), math.sin(ang)
    t = (-s, c, 0.0)
    pts = []
    # tapered spoke: loft of rectangles from hub to rim, dished slightly inward at the rim
    secs = []
    for k in range(5):
        f = k / 4
        r = r0 + (r1 - r0) * f
        w = width * (1.0 - 0.3 * f) if f < 0.9 else width * 1.2
        z = face + 0.02 - 0.03 * f * f
        ctr = (r * c, r * s, z)
        sec = []
        for (a, b) in ((-w / 2, -depth / 2), (w / 2, -depth / 2), (w / 2, depth / 2), (-w / 2, depth / 2)):
            p = (ctr[0] + t[0] * a, ctr[1] + t[1] * a, ctr[2] + b)
            sec.append(xform_point(xf, p))
        secs.append(sec)
    M.loft(mb, secs, mat, part)


def _farm_lugs(model, part, xf, spec, mat):
    """Chevron traction lugs, two rows (one per half of the tread)."""
    mb = model.mb
    R, W, h = spec.radius - spec.lug_height, spec.width, spec.lug_height  # R = carcass radius
    n = spec.lugs
    for half in (1, -1):
        for k in range(n):
            ang = 2 * math.pi * (k + (0.5 if half < 0 else 0.0)) / n
            # a lug runs from near the centreline out past the shoulder, angled ~40 deg
            a0, a1 = 0.03 * half, (W / 2 + 0.01) * half
            dang = 0.55 * (W / 2) / R  # angular sweep along the lug (chevron)
            p0 = (ang, a0)
            p1 = (ang + dang, a1)
            lw = 0.035 + 0.02 * R  # lug width (circumferential)
            secs = []
            steps = 4
            for s in range(steps + 1):
                f = s / steps
                th = p0[0] + (p1[0] - p0[0]) * f
                ax = p0[1] + (p1[1] - p0[1]) * f
                drop = 0.0 if abs(ax) < W / 2 - 0.06 else (abs(ax) - (W / 2 - 0.06)) * 0.8
                r_in, r_out = R - 0.01 - drop, R + h - drop
                dth = lw / R / 2
                sec = [(r_in, th - dth, ax), (r_in, th + dth, ax), (r_out, th + dth * 0.8, ax),
                       (r_out, th - dth * 0.8, ax)]
                secs.append([xform_point(xf, (r * math.cos(t), r * math.sin(t), a)) for (r, t, a) in sec])
            M.loft(mb, secs, mat, part)


def corner(model, key, center, spec, parent=None, steer=False, steer_limits=(-35, 35), travel=(-0.08, 0.08),
           caliper=True, tag=None, susp_node=True, kingpin_offset=0.0):
    """Create susp_/steer_/wheel_ nodes for one wheel and build its geometry.
    kingpin_offset: steer axis this far inboard of the wheel centre (vertical kingpin; 0 = through
    the wheel centre), so the wheel swings about the kingpin like a real knuckle.
    Returns the wheel node name."""
    side = 1 if center[1] > 0 else -1
    tag = tag or key
    p = parent
    if susp_node:
        p = model.node(f"susp_{key}", parent, center,
                       model.prismatic((0, 0, 1), travel[0], travel[1], "wheel travel (bump +)"))
    carrier = p
    if steer:
        kp = (center[0], center[1] - side * kingpin_offset, center[2])
        carrier = model.node(f"steer_{key}", p, kp,
                             model.revolute((0, 0, 1), steer_limits[0], steer_limits[1], "road-wheel steer angle, + = left"))
    w = model.node(f"wheel_{key}", carrier, center, model.continuous((0, 1, 0), "wheel spin, + = rolling forward"),
                   wheel={"radius": spec.radius, "width": spec.width, "side": "left" if side > 0 else "right"})
    build_wheel(model, w, center, spec, side, tag)
    if caliper and spec.disc and carrier:
        cm = f"caliper_{tag}"
        model.mat(cm, color=spec.caliper_color, metallic=0.3, roughness=0.4)
        dr = spec.rim_radius * 0.82
        off = -0.02 + spec.rim_offset * 0.5
        # caliper at the rear-top of the disc (towards -x, +z)
        a = math.radians(125)
        cx, cz = center[0] + dr * 0.85 * math.cos(a) * 1.0, center[2] + dr * 0.85 * math.sin(a)
        cy = center[1] + side * off
        M.rounded_box(model.mb, (cx, cy, cz), (0.07 * dr / 0.2, 0.06, 0.13 * dr / 0.2), 0.012, cm, carrier,
                      xf=_about_y((cx, cy, cz), -(a - math.pi / 2)))
    return w


def _about_y(c, ang):
    return compose(translate(c), rot_axis((0, 1, 0), ang), translate((-c[0], -c[1], -c[2])))


# ------------------------------------------------------- cabin details

def steering_wheel(model, name, parent, center, column_dir, radius=0.19, lock_deg=540, spokes=3,
                   mat="interior", rim_mat=None):
    """Steering wheel node rotating about the column (column_dir points from the wheel into the dash).
    + rotation = steering left (counter-clockwise as the driver sees it)."""
    axis = norm(mul(column_dir, -1))  # towards the driver; + about it = CCW seen by the driver
    model.node(name, parent, center, model.revolute(axis, -lock_deg, lock_deg, "steering wheel angle, + = left"))
    f = frame_z_to(axis, center)
    rim = [xform_point(f, (radius * math.cos(2 * math.pi * k / 32), radius * math.sin(2 * math.pi * k / 32), 0))
           for k in range(32)]
    M.tube(model.mb, rim, 0.017, rim_mat or mat, name, count=10, closed=True)
    hub = xform_point(f, (0, 0, -0.03))
    for k in range(spokes):
        a = -math.pi / 2 + (2 * math.pi * k / spokes if spokes != 3 else [0, 1.95, -1.95][k] + 0.0)
        if spokes == 3:
            a = [-math.pi / 2, 0.15, math.pi - 0.15][k]
        tip = xform_point(f, (radius * math.cos(a), radius * math.sin(a), 0))
        M.tube(model.mb, [hub, tip], 0.012, mat, name, count=6)
    M.cylinder(model.mb, xform_point(f, (0, 0, -0.07)), xform_point(f, (0, 0, 0.0)), radius * 0.3, mat, name,
               count=16)
    # column (static, on the parent)
    M.cylinder(model.mb, xform_point(f, (0, 0, -0.07)), add(center, mul(column_dir, 0.35)), 0.03, mat,
               "" if parent is None else parent, count=10)


def seat(model, part, pos, width=0.5, mat="seat", yaw=0.0, recline=0.35, height=0.12, headrest=True):
    """Seat: cushion + backrest + headrest. pos = cushion centre top."""
    mb = model.mb
    x, y, z = pos
    M.rounded_box(mb, (x, y, z - height / 2), (0.5, width, height), 0.04, mat, part)
    # backrest: a box tilted back about y through its bottom edge
    bx = x - 0.25
    back_h = 0.6
    xf = compose(translate((bx, y, z)), rot_axis((0, 1, 0), -recline), translate((0, 0, back_h / 2)))
    M.rounded_box(mb, (0, 0, 0), (0.12, width, back_h), 0.04, mat, part, xf=xf)
    if headrest:
        xf2 = compose(translate((bx, y, z)), rot_axis((0, 1, 0), -recline), translate((0.0, 0, back_h + 0.13)))
        M.rounded_box(mb, (0, 0, 0), (0.1, width * 0.55, 0.18), 0.04, mat, part, xf=xf2)


def mirror(model, part, root_pos, side, size=(0.1, 0.2, 0.12), body_mat="paint", arm=0.06):
    """Door/wing mirror: housing pointing outward from root_pos with the glass facing rearward."""
    mb = model.mb
    sx, sy, sz = size
    c = (root_pos[0], root_pos[1] + side * (arm + sy / 2), root_pos[2])
    M.rounded_box(mb, c, size, min(sx, sz) * 0.35, body_mat, part)
    M.box(mb, (c[0] - sx / 2 - 0.002, c[1], c[2]), (0.004, sy * 0.85, sz * 0.8), "mirror", part)
    M.box(mb, (root_pos[0], root_pos[1] + side * arm / 2, root_pos[2] - sz * 0.2), (0.04, arm, 0.03), "trim_black", part)
