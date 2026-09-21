"""e) ~200 hp row-crop tractor (generic, class-typical dimensions).

Length 5.09 m (front weights to drawbar clevis), width 2.68 m over the rear
tyres (2.53 m over the tyre centres + half widths is 2.61 m; sidewall bulge
adds the rest), cab roof 3.05 m (beacon top 3.20 m), wheelbase 2.80 m (front
axle x = +1.40, rear x = -1.40), track 1.90 m front and rear (tyre centres).
Rear tyres 710/70 R42 (r = 1.025 m over the lugs, w = 0.71 m, rim r = 0.533 m),
front tyres 600/70 R30 (r = 0.80 m, w = 0.60 m, rim r = 0.381 m), farm lug
tyres on painted steel rims.

Nodes and joints:
  front_axle          revolute x   (pendulum, +-10 deg, + = left wheel up)
    steer_FL/FR       revolute z   (kingpin at y = +-0.80, +-50 deg, + = left)
      wheel_FL/FR     continuous y
      steer_cyl_rod_FL/FR  aim -> steer_cyl_FL/FR (steering cylinder rod)
    steer_cyl_FL/FR   aim -> steer_FL/FR (steering cylinder barrel)
  wheel_RL/RR         continuous y (rigid rear axle, no suspension)
  door_L              revolute z   (hinge on the left rear cab post, + = open)
  rear_window         revolute y   (hinge at the top, + = bottom swings out)
  steering_wheel      revolute about the column
  lift_arm_L/R        revolute y   (+ = raise)
    lift_rod_L/R      aim + stretch -> lower_link_L/R (closes the loop)
    lift_ram_rod_L/R  aim -> lift_ram_L/R (assist cylinder rod)
  lift_ram_L/R        aim -> lift_arm_L/R (assist cylinder barrel)
  lower_link_L/R      revolute y   (+ = ball end up)
  top_link            revolute y   (+ = rear end up)
  pto                 continuous x (+ = clockwise seen from behind, the standard direction)
Lamps: head, work, beacon, tail, brake, indicators.
"""
import math

from ..scene import Model
from ..parts import standard_materials, corner, WheelSpec, steering_wheel, seat, mirror
from .. import mesh as M
from ..vec import Curve, norm, lerp, add, sub, mul
from ._farm import farm_materials, arc_shell, arc_lip, beam, rod, eye, lamp_box

# Drawbar pin height above flat ground; must equal trailer_tipper.HITCH_HEIGHT.
HITCH_HEIGHT = 0.50
HITCH_X = -2.45            # drawbar pin, 0.37 m behind the PTO stub end

WB, TRACK = 2.80, 1.90
XF, XR = WB / 2, -WB / 2
RR_, RF_ = 1.025, 0.80      # nominal tyre radii (over the lugs) = wheel centre heights
KINGPIN_Y = 0.80            # |y| of the front steering axes (0.15 m inboard of the tyre centre)
FF_R, FF_A0, FF_A1 = 0.88, 64.0, 118.0   # front fender radius and angular extent (deg from +x)
AXLE_PIVOT = (XF, 0.0, 0.86)

REAR = WheelSpec(radius=RR_, width=0.71, rim_radius=0.533, tyre="farm", rim="farm", lugs=23, lug_height=0.065,
                 rim_color=(0.66, 0.67, 0.64), rim_metallic=0.25, disc=False)
FRONT = WheelSpec(radius=RF_, width=0.60, rim_radius=0.381, tyre="farm", rim="farm", lugs=20, lug_height=0.05,
                  rim_color=(0.66, 0.67, 0.64), rim_metallic=0.25, disc=False)

CAB_FLOOR = 1.48
CAB_TOP = 2.86           # glass top / roof bottom
CAB_XF, CAB_XR = -0.05, -1.70   # front / rear post x
CAB_Y = 0.845            # upper cab side glass |y|
TUB_Y = 0.56             # lower cab |y| between the rear fenders
FENDER_R = 1.10          # rear fender inner radius (tyre r 1.025)


def _fender_line(x, r=1.17):
    """z of the cab side-glass lower edge above the rear fender."""
    d = x - XR
    return RR_ + math.sqrt(max(r * r - d * d, 0.0))


def _hood_dims(x):
    """(half width, bottom z, top z, waist half width at the bottom) of the hood at station x."""
    top = Curve([(0.0, 1.97), (0.8, 1.95), (1.6, 1.90), (2.05, 1.84), (2.16, 1.79)])(x)
    a = Curve([(0.0, 0.41), (0.4, 0.38), (1.1, 0.36), (1.8, 0.37), (2.16, 0.34)])(x)
    wb = Curve([(0.0, 0.40), (0.3, 0.38), (0.62, 0.15), (1.95, 0.15), (2.17, 0.24)])(x)
    bev = max(0.0, x - 2.10) / 0.07  # 0..1 over the nose bevel
    return a - 0.035 * bev * bev, 1.10, top - 0.04 * bev * bev, wb


HOOD_WAIST_TOP = 1.52


def _waist(x, z, a, wb, bot):
    """Width factor below HOOD_WAIST_TOP: the hood sides taper in towards the bottom so the
    steered front tyres clear it."""
    if z >= HOOD_WAIST_TOP:
        return 1.0
    t = (z - bot) / (HOOD_WAIST_TOP - bot)
    t = max(0.0, min(1.0, t))
    t = t * t * (3 - 2 * t) * 0.35 + t * 0.65
    return (wb + (a - wb) * t) / a


def hood_half_width(x, z):
    a, bot, t, wb = _hood_dims(x)
    hh = (t - bot) / 2
    r = min(1.0, abs(z - (bot + hh)) / hh)
    return a * (1 - r ** 3.2) ** (1 / 3.2) * _waist(x, z, a, wb, bot)


def build():
    m = Model("tractor", "Row-crop tractor (~200 hp)",
              "Generic ~200 hp row-crop tractor, 5.1 m, 710/70 R42 rear and 600/70 R30 front farm tyres, "
              "pendulum front axle, 4-post cab, rear three-point linkage, PTO and drawbar.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    standard_materials(m)
    farm_materials(m)
    m.mat("seat", color=(1, 1, 1), tex=("fabric", {"color": (0.12, 0.12, 0.13)}), tile=0.2)
    m.mat("tread_plate", color=(1, 1, 1), tex=("tread_plate", {}), tile=0.3)
    m.mat("light_work", color=(0.85, 0.87, 0.9), metallic=0.3, roughness=0.1, tex=("lens", {}), tile=0.05,
          emissive=(1.0, 0.97, 0.9), light="work", light_strength=10.0)
    m.mat("light_beacon", color=(0.85, 0.45, 0.05), alpha=0.85, roughness=0.1, blend=True,
          emissive=(1.0, 0.55, 0.0), light="beacon", light_strength=8.0)
    mb = m.mb

    # ================================================================ chassis
    # engine block + sump, transmission, rear centre housing
    M.rounded_box(mb, (0.20, 0, 0.91), (1.00, 0.56, 0.42), 0.06, "chassis")
    M.rounded_box(mb, (0.22, 0, 0.73), (0.86, 0.44, 0.08), 0.03, "mud")                  # oil pan
    M.rounded_box(mb, (-0.62, 0, 0.98), (0.70, 0.62, 0.64), 0.08, "chassis")            # gearbox
    M.rounded_box(mb, (-1.43, 0, 1.04), (1.04, 0.90, 0.84), 0.10, "chassis")            # rear housing
    # narrow front frame (steering clearance) carrying the axle pivot and the weights
    beam(mb, (0.60, 0, 1.045), (2.20, 0, 1.045), 0.24, 0.15, "chassis")
    M.rounded_box(mb, (1.00, 0, 1.10), (0.30, 0.30, 0.12), 0.03, "chassis")            # radiator mount
    for xb in (1.12, 1.68):  # pendulum pivot brackets
        M.rounded_box(mb, (xb, 0, 0.93), (0.07, 0.20, 0.14), 0.02, "chassis")
    # front drive shaft (gearbox -> front axle), passes under the sump
    rod(mb, (-0.30, 0, 0.64), (1.20, 0, 0.72), 0.045, "steel")
    # rear axle trumpets and final drives
    for s in (1, -1):
        M.cylinder(mb, (XR, s * 0.40, RR_), (XR, s * 0.62, RR_), 0.21, "mud", count=24)
        M.cylinder(mb, (XR, s * 0.62, RR_), (XR, s * 1.02, RR_), 0.11, "mud", count=16)

    # ================================================================ hood (wasp waist for steering lock)
    secs = []
    xs = [0.0 + k * 0.1 for k in range(21)] + [2.08, 2.12, 2.15, 2.17]
    for x in xs:
        a, bot, t, wb = _hood_dims(x)
        hh = (t - bot) / 2
        sec = []
        for (py, pz) in M.superellipse(a, hh, 3.2, 48):
            z = bot + hh + pz
            sec.append((py * _waist(x, z, a, wb, bot), z))
        secs.append(M.place2d(sec, (x, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    M.loft(mb, secs, "paint", cap_start=True, cap_end=True, cap_material="grille")
    # side vents
    for s in (1, -1):
        for zc in (1.58, 1.66, 1.74):
            n = 8
            for k in range(n):  # thin strips following the curved hood side
                xa, xb = 1.40 + 0.58 * k / n, 1.40 + 0.58 * (k + 1) / n
                pts = [(x, s * (hood_half_width(x, zc + dz) + 0.004), zc + dz) for (x, dz) in
                       ((xa, -0.028), (xb, -0.028), (xb, 0.028), (xa, 0.028))]
                mb.polygon(pts, "grille", want_normal=(0, s, 0))
        # headlights in the grille
        lamp_box(mb, "", (2.165, s * 0.20, 1.68), (0.05, 0.14, 0.09), "+x", "light_head")
        lamp_box(mb, "", (2.165, s * 0.20, 1.565), (0.05, 0.14, 0.06), "+x",
                 "light_indicator_left" if s > 0 else "light_indicator_right")
    # front weights on a carrier
    M.box(mb, (2.20, 0, 1.03), (0.06, 0.62, 0.30), "chassis")
    for k in range(6):
        x = 2.26 + k * 0.058
        M.rounded_box(mb, (x, 0, 0.99), (0.052, 0.80, 0.58), 0.02, "chassis")
    M.rounded_box(mb, (2.40, 0, 1.30), (0.34, 0.08, 0.05), 0.015, "chassis")            # lifting bar
    M.box(mb, (2.525, 0, 0.99), (0.01, 0.40, 0.10), "reflector_white")

    # ================================================================ front axle (pendulum)
    fa = m.node("front_axle", None, AXLE_PIVOT,
                m.revolute((1, 0, 0), -10, 10, "front axle pendulum about the centre pivot, + = left wheel up"))
    M.rounded_box(mb, (XF, 0, 0.80), (0.20, 2 * (KINGPIN_Y - 0.10), 0.20), 0.05, "mud", fa)
    M.ellipsoid(mb, (XF, 0, 0.78), (0.24, 0.26, 0.18), "mud", fa)
    rod(mb, (1.10, 0, 0.86), (1.70, 0, 0.86), 0.06, "steel", fa)                       # trunnion
    for s in (1, -1):
        for zz in (1.035, 0.565):  # kingpin yokes
            M.rounded_box(mb, (XF, s * (KINGPIN_Y - 0.06), zz), (0.20, 0.18, 0.05), 0.015, "mud", fa)
        M.rounded_box(mb, (XF, s * (KINGPIN_Y - 0.12), 0.80), (0.16, 0.06, 0.50), 0.02, "mud", fa)

    # ================================================================ front corners
    for key, s in (("FL", 1), ("FR", -1)):
        kp = (XF, s * KINGPIN_Y, RF_)
        st = m.node(f"steer_{key}", fa, kp,
                    m.revolute((0, 0, 1), -50, 50, "road-wheel steer angle about the kingpin, + = left"))
        rod(mb, (XF, s * KINGPIN_Y, 0.595), (XF, s * KINGPIN_Y, 1.005), 0.065, "mud", st)  # knuckle
        rod(mb, (XF, s * KINGPIN_Y, RF_), (XF, s * 0.93, RF_), 0.11, "mud", st)            # stub axle / hub
        arm_end = (XF - 0.18, s * (KINGPIN_Y - 0.20), 0.90)
        beam(mb, (XF, s * KINGPIN_Y, 0.90), arm_end, 0.05, 0.05, "mud", st)               # steering arm
        eye(mb, arm_end, (0, 0, 1), 0.035, 0.014, 0.05, "mud", st)
        corner(m, key, (XF, s * TRACK / 2, RF_), FRONT, parent=st, steer=False, susp_node=False,
               caliper=False, tag="F")
        # steering cylinder: barrel on the axle, rod on the steering arm (aim pair)
        cb = (XF - 0.18, s * 0.10, 0.90)
        dcy = sub(arm_end, cb)
        sb = m.node(f"steer_cyl_{key}", fa, cb,
                    m.aim(f"steer_{key}", arm_end, note="steering cylinder barrel, follows the steering arm"))
        rod(mb, cb, add(cb, mul(dcy, 0.80)), 0.035, "chassis", sb)
        sr = m.node(f"steer_cyl_rod_{key}", st, arm_end,
                    m.aim(f"steer_cyl_{key}", cb, note="steering cylinder rod, follows the barrel"))
        rod(mb, arm_end, add(arm_end, mul(dcy, -0.80)), 0.018, "rod", sr)
        # fender turning with the wheel, on a bracket from the knuckle
        c = (XF, RF_)
        hwf = 0.30
        y0, y1 = sorted((s * (TRACK / 2 - hwf), s * (TRACK / 2 + hwf)))
        a0, a1 = math.radians(FF_A0), math.radians(FF_A1)
        arc_shell(mb, st, "paint_dirty", c, y0, y1, FF_R, a0, a1, t=0.012, n=20)
        yo = s * (TRACK / 2 + hwf) - s * 0.006
        arc_lip(mb, st, "paint_dirty", c, yo, FF_R - 0.05, a0, a1, depth=0.05, t=0.012, n=20)
        br = [(XF, s * KINGPIN_Y, 1.0), (XF, s * 0.60, 1.10), (XF - 0.02, s * 0.585, 1.58),
              (XF - 0.06, s * (TRACK / 2 - hwf + 0.01), FF_R + RF_ - 0.004)]
        M.tube(mb, br, 0.028, "chassis", st, count=10)

    # ================================================================ rear wheels (rigid axle)
    for key, s in (("RL", 1), ("RR", -1)):
        corner(m, key, (XR, s * TRACK / 2, RR_), REAR, steer=False, susp_node=False, caliper=False, tag="R")

    # ================================================================ rear fenders + lamps
    for s in (1, -1):
        y0, y1 = sorted((s * 0.55, s * 1.39))
        arc_shell(mb, "", "paint_dirty", (XR, RR_), y0, y1, FENDER_R, math.radians(22), math.radians(172),
                  t=0.016, n=32)
        arc_lip(mb, "", "paint_dirty", (XR, RR_), s * 1.382, FENDER_R - 0.075, math.radians(22),
                math.radians(172), depth=0.075, t=0.014, n=32)
        # rear lamp cluster on the fender's rear flank
        a = math.radians(146)
        lx, lz = XR + (FENDER_R + 0.07) * math.cos(a), RR_ + (FENDER_R + 0.07) * math.sin(a)
        M.rounded_box(mb, (lx + 0.03, s * 1.22, lz), (0.10, 0.26, 0.13), 0.02, "trim_black")
        for (yc, lens) in ((1.13, "light_tail"), (1.22, "light_brake"),
                           (1.31, "light_indicator_left" if s > 0 else "light_indicator_right")):
            M.box(mb, (lx - 0.022, s * yc, lz), (0.012, 0.075, 0.10), lens)
        M.box(mb, (lx - 0.022, s * 1.22, lz - 0.10), (0.012, 0.16, 0.05), "reflector_red")

    # ================================================================ cab
    # floor (full width in front of the fenders, narrow tub between them), tub walls
    M.box(mb, (-0.89, 0, CAB_FLOOR - 0.02), (1.70, 2 * TUB_Y, 0.04), "chassis")
    M.box(mb, (-0.235, 0, CAB_FLOOR - 0.02), (0.39, 2 * CAB_Y, 0.04), "chassis")
    M.box(mb, (-0.89, 0, CAB_FLOOR + 0.005), (1.62, 2 * TUB_Y - 0.04, 0.01), "interior")  # floor mat
    for s in (1, -1):
        M.box(mb, (-1.06, s * TUB_Y, 1.82), (1.30, 0.02, 0.68), "paint")
        # closing panel between the door's lower edge and the fender front
        M.box(mb, (-0.42, s * (TUB_Y + CAB_Y) / 2, 1.575), (0.02, CAB_Y - TUB_Y, 0.19), "paint")
    M.box(mb, (CAB_XR - 0.025, 0, 1.82), (0.03, 2 * TUB_Y, 0.68), "paint")               # rear lower wall
    # posts
    for s in (1, -1):
        M.rounded_box(mb, (CAB_XF, s * CAB_Y, (CAB_FLOOR + CAB_TOP) / 2), (0.07, 0.07, CAB_TOP - CAB_FLOOR),
                      0.015, "trim_black")
        zr = _fender_line(CAB_XR, FENDER_R) - 0.01
        M.rounded_box(mb, (CAB_XR, s * CAB_Y, (zr + CAB_TOP) / 2), (0.07, 0.07, CAB_TOP - zr), 0.015,
                      "trim_black")
        # sill rail along the fender line
        xs_ = [CAB_XR + (-0.43 - CAB_XR) * k / 12 for k in range(13)]
        rail = [(x, s * CAB_Y, _fender_line(x) - 0.02) for x in xs_]
        M.tube(mb, rail, 0.02, "trim_black", count=8)
        # top rail
        M.rounded_box(mb, ((CAB_XF + CAB_XR) / 2, s * CAB_Y, CAB_TOP - 0.02), (CAB_XF - CAB_XR, 0.06, 0.05),
                      0.015, "trim_black")
    M.rounded_box(mb, (CAB_XF, 0, CAB_TOP - 0.02), (0.06, 2 * CAB_Y, 0.05), 0.015, "trim_black")
    M.rounded_box(mb, (CAB_XR, 0, CAB_TOP - 0.02), (0.06, 2 * CAB_Y, 0.05), 0.015, "trim_black")
    M.rounded_box(mb, (CAB_XR, 0, 2.16), (0.05, 2 * CAB_Y, 0.05), 0.015, "trim_black")  # rear sill
    # windscreen
    ws = [(CAB_XF + 0.005, y, z) for (y, z) in ((-0.81, 1.52), (0.81, 1.52), (0.81, CAB_TOP - 0.03),
                                                 (-0.81, CAB_TOP - 0.03))]
    mb.polygon(ws, "glass", want_normal=(1, 0, 0))

    def side_outline(y):
        pts = [(-0.10, y, 1.52), (-0.10, y, CAB_TOP - 0.04), (CAB_XR + 0.05, y, CAB_TOP - 0.04)]
        n = 14
        for k in range(n + 1):
            x = CAB_XR + 0.05 + (-0.43 - (CAB_XR + 0.05)) * k / n
            pts.append((x, y, _fender_line(x)))
        pts.append((-0.43, y, 1.52))
        return pts

    # right side: fixed glass
    mb.polygon(side_outline(-CAB_Y), "glass", want_normal=(0, -1, 0))
    M.tube(mb, side_outline(-CAB_Y - 0.004), 0.016, "trim_black", count=8, closed=True)
    # left door (hinged on the rear post)
    door = m.node("door_L", None, (CAB_XR + 0.02, CAB_Y + 0.01, 2.45),
                  m.revolute((0, 0, 1), 0, 80, "door opening angle (hinge on the rear left post)"))
    out = side_outline(CAB_Y + 0.004)
    mb.polygon(out, "glass", door, want_normal=(0, 1, 0))
    M.tube(mb, out, 0.018, "trim_black", door, count=8, closed=True)
    M.rounded_box(mb, (-0.25, CAB_Y + 0.03, 2.15), (0.16, 0.03, 0.035), 0.01, "chrome", door)   # handle
    M.rounded_box(mb, (-0.12, CAB_Y + 0.02, 2.15), (0.05, 0.03, 0.06), 0.01, "trim_black", door)
    for zz in (2.25, 2.70):
        rod(mb, (CAB_XR + 0.02, CAB_Y + 0.01, zz - 0.05), (CAB_XR + 0.02, CAB_Y + 0.01, zz + 0.05), 0.02,
            "trim_black", door)
    # rear window (hinged at the top)
    rw = m.node("rear_window", None, (CAB_XR - 0.045, 0, CAB_TOP - 0.035),
                m.revolute((0, 1, 0), 0, 40, "rear window opening angle (hinge at the top, + = swings out)"))
    rwo = [(CAB_XR - 0.045, y, z) for (y, z) in ((-0.79, 2.20), (0.79, 2.20), (0.79, CAB_TOP - 0.05),
                                                  (-0.79, CAB_TOP - 0.05))]
    mb.polygon(rwo, "glass", rw, want_normal=(-1, 0, 0))
    M.tube(mb, [(p[0] - 0.004, p[1], p[2]) for p in rwo], 0.014, "trim_black", rw, count=8, closed=True)
    M.rounded_box(mb, (CAB_XR - 0.07, 0, 2.23), (0.03, 0.18, 0.03), 0.01, "trim_black", rw)
    # roof
    M.rounded_box(mb, (-0.87, 0, 2.955), (2.04, 1.94, 0.19), 0.07, "paint")
    M.rounded_box(mb, (-0.87, 0, 3.045), (1.6, 1.3, 0.02), 0.01, "paint")
    for y in (-0.62, -0.30, 0.30, 0.62):  # front work lights
        lamp_box(mb, "", (0.17, y, 2.95), (0.05, 0.15, 0.08), "+x", "light_work")
    for y in (-0.45, 0.45):  # rear work lights
        lamp_box(mb, "", (-1.915, y, 2.95), (0.05, 0.15, 0.08), "-x", "light_work")
    for s in (1, -1):  # indicators at the front roof corners, road lamps in the roof
        lamp_box(mb, "", (0.155, s * 0.88, 2.95), (0.05, 0.10, 0.06), "+x",
                 "light_indicator_left" if s > 0 else "light_indicator_right")
    # beacon
    bc = (-1.60, 0.62, 3.055)
    M.cylinder(mb, bc, (bc[0], bc[1], bc[2] + 0.03), 0.075, "trim_black", count=16)
    M.lathe(mb, [(0.062, 0.0), (0.062, 0.07), (0.05, 0.11), (0.025, 0.135), (0.001, 0.14)], "light_beacon",
            axis_xf=((1, 0, 0, bc[0]), (0, 1, 0, bc[1]), (0, 0, 1, bc[2] + 0.03)), count=20, orient=False)
    # radio antenna
    rod(mb, (-1.60, -0.70, 3.05), (-1.62, -0.70, 3.55), 0.004, "trim_black", count=5)

    # ---- interior
    seat(m, "", (-1.02, 0, 1.93), width=0.52, mat="seat", recline=0.22)
    M.rounded_box(mb, (-1.02, 0, 1.66), (0.30, 0.30, 0.34), 0.04, "interior")          # seat suspension
    M.rounded_box(mb, (-0.95, -0.37, 1.97), (0.58, 0.16, 0.13), 0.04, "interior")      # armrest console
    rod(mb, (-0.80, -0.37, 2.03), (-0.78, -0.37, 2.12), 0.018, "trim_black", count=8)  # joystick
    M.rounded_box(mb, (-0.29, 0, 1.76), (0.22, 0.34, 0.56), 0.05, "interior")          # dash pedestal
    M.rounded_box(mb, (-0.33, 0, 2.07), (0.12, 0.36, 0.09), 0.03, "trim_black")        # instruments
    for yy in (0.14, -0.10, -0.20):
        M.box(mb, (-0.22, yy, 1.56), (0.10, 0.07, 0.02), "trim_black")                  # pedals
    steering_wheel(m, "steering_wheel", None, (-0.55, 0, 2.22), norm((0.45, 0, -0.9)), radius=0.2,
                   lock_deg=900, mat="interior")
    # mirrors on arms from the front posts
    for s in (1, -1):
        root = (0.06, s * 1.14, 2.58)
        M.tube(mb, [(CAB_XF, s * CAB_Y, 2.72), (0.03, s * 1.02, 2.66), root], 0.015, "trim_black", count=8)
        mirror(m, "", (root[0], s * 1.12, root[2] - 0.02), s, size=(0.06, 0.20, 0.30), body_mat="trim_black",
               arm=0.03)

    # ---- exhaust (right) and air intake (left) in front of the cab posts
    ex = [(0.30, -0.36, 1.78), (0.25, -0.56, 1.84), (0.24, -0.62, 2.0), (0.24, -0.62, 3.08)]
    M.tube(mb, ex, 0.05, "chassis", count=14)
    M.cylinder(mb, (0.24, -0.62, 2.10), (0.24, -0.62, 2.66), 0.10, "chassis", count=20)
    M.cylinder(mb, (0.24, -0.62, 3.08), (0.24, -0.62, 3.12), 0.052, "trim_black", count=14)
    ai = [(0.30, 0.36, 1.78), (0.25, 0.56, 1.84), (0.24, 0.62, 2.0), (0.24, 0.62, 2.42)]
    M.tube(mb, ai, 0.055, "paint", count=14)
    M.cylinder(mb, (0.24, 0.62, 2.42), (0.24, 0.62, 2.58), 0.10, "trim_black", count=20)
    M.cylinder(mb, (0.24, 0.62, 2.58), (0.24, 0.62, 2.61), 0.07, "trim_black", count=16)

    # ---- fuel tank (left), battery / tool box (right), steps (left)
    M.rounded_box(mb, (0.20, 0.49, 0.90), (0.44, 0.38, 0.56), 0.07, "paint_dirty")
    rod(mb, (0.30, 0.56, 1.18), (0.30, 0.56, 1.23), 0.045, "trim_black", count=12)
    M.rounded_box(mb, (0.20, -0.49, 0.92), (0.42, 0.38, 0.48), 0.05, "chassis")
    for xs_ in (-0.33, -0.07):
        M.box(mb, (xs_, 0.77, 0.97), (0.02, 0.30, 0.90), "chassis")                    # stringers
    for zz in (0.58, 0.88, 1.18):
        M.box(mb, (-0.20, 0.77, zz), (0.24, 0.30, 0.025), "tread_plate")
    rod(mb, (-0.36, 0.93, 1.25), (-0.36, 0.93, 2.10), 0.016, "trim_black", count=8)    # grab rail
    rod(mb, (-0.36, 0.86, 2.10), (-0.36, 0.93, 2.10), 0.016, "trim_black", count=8)

    # ================================================================ rear: drawbar, PTO, linkage
    hz = HITCH_HEIGHT
    beam(mb, (-1.52, 0, hz - 0.068), (HITCH_X - 0.065, 0, hz - 0.068), 0.16, 0.065, "steel")  # lower jaw
    M.box(mb, (HITCH_X + 0.015, 0, hz + 0.055), (0.16, 0.16, 0.04), "steel")               # upper jaw
    M.box(mb, (HITCH_X + 0.10, 0, hz), (0.02, 0.16, 0.15), "steel")                       # jaw web
    rod(mb, (HITCH_X, 0, hz - 0.11), (HITCH_X, 0, hz + 0.12), 0.02, "chrome", count=10)  # pin
    rod(mb, (HITCH_X, 0, hz + 0.12), (HITCH_X, 0, hz + 0.135), 0.035, "chrome", count=10)
    for xb in (-1.62, -1.88):  # hangers
        M.box(mb, (xb, 0, (hz - 0.035 + 0.64) / 2), (0.05, 0.16, 0.64 - hz + 0.035), "steel")
    # PTO: spinning stub (6 splines) + static shield
    pto_p = (-1.95, 0, 0.74)
    pto = m.node("pto", None, pto_p, m.continuous((1, 0, 0), "PTO shaft, + = clockwise seen from behind"))
    rod(mb, pto_p, (-2.09, 0, 0.74), 0.0175, "steel", pto, count=12)
    for k in range(6):
        a = 2 * math.pi * k / 6
        c = (0, 0.017 * math.cos(a), 0.017 * math.sin(a))
        beam(mb, add((-1.99, 0, 0.74), c), add((-2.085, 0, 0.74), c), 0.006, 0.006, "steel", pto)
    rod(mb, (-1.95, 0, 0.74), (-1.99, 0, 0.74), 0.05, "steel", pto)
    M.cylinder(mb, (-1.94, 0, 0.74), (-1.96, 0, 0.74), 0.09, "chassis", count=20)          # PTO bearing flange
    arc_shell_x(mb, "", "chassis", (0.0, 0.74), -2.08, -1.95, 0.13, math.radians(-10), math.radians(190))

    # three-point linkage
    TOP0, TOP1 = (-1.99, 0.0, 1.20), (-2.54, 0.0, 1.24)
    M.rounded_box(mb, (-1.975, 0, 1.20), (0.06, 0.16, 0.20), 0.02, "steel")               # top-link bracket
    M.cylinder(mb, (-1.72, -0.56, 1.52), (-1.72, 0.56, 1.52), 0.075, "mud", count=20)    # lift cross shaft
    tl = m.node("top_link", None, TOP0, m.revolute((0, 1, 0), -20, 20, "top link pitch, + = rear end up"))
    d = sub(TOP1, TOP0)
    rod(mb, TOP0, add(TOP0, mul(d, 0.35)), 0.028, "steel", tl)
    rod(mb, add(TOP0, mul(d, 0.30)), add(TOP0, mul(d, 0.62)), 0.042, "chassis", tl)        # turnbuckle
    rod(mb, add(TOP0, mul(d, 0.60)), add(TOP1, mul(d, -0.06)), 0.024, "steel", tl)
    eye(mb, TOP0, (0, 1, 0), 0.045, 0.02, 0.05, "steel", tl)
    eye(mb, TOP1, (0, 1, 0), 0.05, 0.022, 0.05, "steel", tl)
    m.socket("socket_3pt_top", tl, TOP1, "three-point linkage top link pin (implement top point)")

    loop = {}
    for key, s in (("L", 1), ("R", -1)):
        F, B = (-1.58, s * 0.42, 0.52), (-2.46, s * 0.50, 0.60)
        P = lerp(F, B, 0.58)
        P = (P[0], P[1], P[2] + 0.055)           # lift rod pin on top of the lower link
        A = (-1.72, P[1], 1.52)
        E = (-2.28, P[1], 1.42)
        ll = m.node(f"lower_link_{key}", None, F,
                    m.revolute((0, 1, 0), -20, 25, "lower link pitch about its front pivot, + = ball end up"))
        beam(mb, F, B, 0.05, 0.10, "chassis", ll, up=(0, 0, 1))
        eye(mb, F, (0, 1, 0), 0.055, 0.025, 0.06, "steel", ll)
        eye(mb, B, (0, 1, 0), 0.065, 0.028, 0.05, "steel", ll)
        M.ellipsoid(mb, B, (0.028, 0.022, 0.028), "chrome", ll, count=12, rings=6)       # ball
        M.box(mb, (P[0], P[1], P[2] - 0.02), (0.07, 0.05, 0.05), "chassis", ll)            # rod lug
        M.box(mb, (F[0], s * (0.42 - 0.06), 0.56), (0.12, 0.03, 0.14), "steel")            # pivot bracket
        M.box(mb, (F[0], s * (0.42 + 0.06), 0.56), (0.12, 0.03, 0.14), "steel")
        m.socket(f"socket_3pt_{key}", ll, B, "three-point linkage lower link ball end (implement lower hitch point)")
        # lift arm + lift rod (stretch aim closes the loop arm -> lower link)
        la = m.node(f"lift_arm_{key}", None, A, m.revolute((0, 1, 0), -10, 45, "lift arm angle, + = raise"))
        beam(mb, A, E, 0.06, 0.10, "chassis", la)
        M.cylinder(mb, (A[0], A[1] - 0.05, A[2]), (A[0], A[1] + 0.05, A[2]), 0.085, "chassis", la, count=18)
        eye(mb, E, (0, 1, 0), 0.04, 0.018, 0.06, "chassis", la)
        lr = m.node(f"lift_rod_{key}", la, E,
                    m.aim(f"lower_link_{key}", P, stretch=True,
                          note="lift rod: arm end -> lower link pin, stretches (closes the loop)"))
        dlr = sub(P, E)
        rod(mb, E, P, 0.02, "steel", lr, count=10)
        rod(mb, add(E, mul(dlr, 0.35)), add(E, mul(dlr, 0.65)), 0.032, "chassis", lr, count=10)  # adjuster
        # external assist ram: barrel on the axle trumpet, rod on the lift arm
        ry = s * 0.53
        C0 = (-1.62, ry, 0.92)
        Ea = lerp(A, E, 0.6)
        Ea = (Ea[0], ry, Ea[2])
        dr = sub(Ea, C0)
        L0 = math.dist(C0, Ea)
        brl = m.node(f"lift_ram_{key}", None, C0,
                     m.aim(f"lift_arm_{key}", Ea, note="assist cylinder barrel, follows the lift arm"))
        rod(mb, C0, add(C0, mul(dr, 0.62)), 0.042, "chassis", brl, count=14)
        eye(mb, C0, (0, 1, 0), 0.04, 0.016, 0.05, "chassis", brl)
        rrd = m.node(f"lift_ram_rod_{key}", la, Ea,
                     m.aim(f"lift_ram_{key}", C0, note="assist cylinder rod, follows the barrel"))
        rod(mb, Ea, add(Ea, mul(dr, -0.60)), 0.022, "rod", rrd, count=10)
        eye(mb, Ea, (0, 1, 0), 0.035, 0.014, 0.05, "chassis", rrd)
        rod(mb, (Ea[0], P[1], Ea[2]), (Ea[0], ry, Ea[2]), 0.014, "steel", la, count=8)  # pin to the arm
        M.box(mb, (C0[0] + 0.02, ry, C0[2] + 0.04), (0.06, 0.05, 0.12), "mud")          # barrel bracket
        loop[key] = (math.dist(E, P), math.dist(A, E), math.dist(F, P), L0)

    # ================================================================ sockets
    m.socket("socket_hitch", None, (HITCH_X, 0.0, HITCH_HEIGHT),
             f"drawbar pin centre (trailer eye couples here), {HITCH_HEIGHT:.2f} m above ground")
    m.socket("socket_pto", pto, (-2.09, 0.0, 0.74), "PTO stub end (1 3/8 in, 6 splines)")
    m.socket("socket_driver_eye", None, (-1.08, 0.0, 2.62), "driver eye point (camera)")
    m.socket("socket_exhaust", None, (0.24, -0.62, 3.12), "exhaust stack outlet (points up)")
    m.socket("socket_ballast_front", None, (2.40, 0.0, 0.99), "front ballast block centre (~900 kg)")

    lr_len, arm_len, ll_p, ram0 = loop["L"]
    m.notes += [
        "Generic ~200 hp row-crop tractor; dimensions are class-typical, not a specific production machine.",
        f"Drawbar pin (socket_hitch) at x = {HITCH_X:.2f}, z = {HITCH_HEIGHT:.2f} m (HITCH_HEIGHT, same value as "
        "trailer_tipper.HITCH_HEIGHT), 0.36 m behind the PTO stub end; the drawbar is rigid on the body.",
        "Farm tyres: wheel radius in the extras is the nominal radius over the lugs (rear 1.025, front 0.80); "
        "the carcass is lug_height smaller (rear 0.065, front 0.05).",
        f"Front axle: pendulum pivot at {AXLE_PIVOT} about x (+-10 deg); steering axes (kingpins) are vertical at "
        f"x = {XF}, y = +-{KINGPIN_Y}, i.e. 0.15 m inboard of the tyre centre (scrub radius). Both steer joints "
        "are independent: the sim sets Ackermann angles.",
        "Steering clearance: with the front axle level, +-50 deg lock clears the hood (wasp waist) and frame. "
        "With the axle oscillated towards a wheel, that wheel's fender/tyre touch the hood beyond ~25 deg lock "
        "at 10 deg oscillation (~35 deg at 6 deg); a sim wanting clean visuals can clamp the combination.",
        "Rear wheels are on the rigid body (no suspension); tyres touch z = 0 at rest.",
        f"Three-point linkage kinematic loop: lift rod rest length {lr_len:.3f} m between the lift arm end "
        f"(arm length {arm_len:.3f} m from its pivot) and the lower link pin ({ll_p:.3f} m from the link "
        "pivot). The sim should drive lower_link_* from lift_arm_* so this distance stays constant; the aim "
        "links follow either way (lift rods stretch, assist rams telescope).",
        "Aim links: rest direction pivot -> target_point is modelled; stretch scales along it.",
        "PTO: 540/1000 rpm stub at socket_pto, axis +x, + = clockwise seen from behind (standard).",
        "Mass hints (not in the geometry): ~9.5 t operating mass, ~40 % front / 60 % rear without ballast; "
        "front weight block ~0.9 t.",
    ]
    return m


def arc_shell_x(mb, part, mat, center_yz, x0, x1, r, a0, a1, t=0.008, n=16):
    """Shield: part of a cylinder about x (angles from +y towards +z), from x0 to x1."""
    cy, cz = center_yz
    sec = M.rrect(abs(x1 - x0), t, t * 0.45, 2)
    secs = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        c, s = math.cos(a), math.sin(a)
        o = ((x0 + x1) / 2, cy + (r + t / 2) * c, cz + (r + t / 2) * s)
        secs.append(M.place2d(sec, o, (1.0, 0.0, 0.0), (0.0, c, s)))
    return M.loft(mb, secs, mat, part)
