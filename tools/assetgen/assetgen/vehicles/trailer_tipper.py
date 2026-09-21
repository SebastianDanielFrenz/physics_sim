"""f) Tandem-axle agricultural tipping trailer (~14 t payload), generic design.

Length 7.63 m (hitch eye to the rear lamps), width 2.50 m (over the tyres /
bed 2.40 m), bed inner 6.00 x 2.30 x 1.20 m (16.6 m3), floor top 1.40 m above
ground, tyres 560/60 R22.5 (r = 0.62 m, w = 0.56 m, rim r = 0.286 m) on steel
disc wheels, track 1.90 m (tyre centres), axle spacing 1.60 m. Origin midway
between the two axles (axles at x = +-0.80). Hitch eye 4.95 m ahead of the
axle-group centre, at HITCH_HEIGHT = 0.50 m (same as tractor.HITCH_HEIGHT).

Nodes and joints:
  bogie              revolute y   (walking beams, pivot between the axles, +-5 deg, + = front axle down)
    wheel_1L/1R      continuous y (front axle, x = +0.80)
    wheel_2L/2R      continuous y (rear axle, x = -0.80)
  bed                revolute -y  (rear tipping hinge, 0..50 deg, + = tipping, front up)
    tailgate         revolute y   (hinge along the top rear edge, 0..100 deg, + = bottom swings out)
    ram_stages       aim + stretch -> ram_barrel base (telescopic ram inner stages, on the bed)
  ram_barrel         aim -> bed (ram outer barrel on the chassis cradle)
  jack               prismatic -z (parking leg, 0..0.32 m, + = foot down)
Lamps: tail, brake, indicators.
"""
import math

from ..scene import Model
from ..parts import standard_materials, corner, WheelSpec
from .. import mesh as M
from ..vec import lerp, add, sub, mul, norm
from ._farm import farm_materials, arc_shell, beam, rod, eye, lamp_box, tri_reflector

# Hitch eye height above flat ground; must equal tractor.HITCH_HEIGHT.
HITCH_HEIGHT = 0.50
EYE_X = 4.95                 # hitch eye centre, ahead of the axle-group centre (origin)

AXLE_X, TRACK = 0.80, 1.90
TYRE = WheelSpec(radius=0.62, width=0.56, rim_radius=0.286, tyre="trailer", rim="steel", hubcap=False,
                 rim_color=(0.62, 0.63, 0.60), rim_metallic=0.3, disc=False)
R = TYRE.radius

FLOOR = 1.40                 # bed floor top
BED_X0, BED_X1 = -2.50, 3.50  # inner length 6.0
BED_YI = 1.15                # inner half width (2.30)
WALL_T = 0.05
WALL_H = 1.20
HINGE = (BED_X0, 0.0, 1.28)  # tipping hinge axis point
CH_Y = 0.48                  # chassis beam centre |y|
CH_Z0, CH_Z1 = 0.97, 1.20    # chassis beam bottom / top
BOGIE_PIVOT = (0.0, 0.0, 0.72)
RAM_BASE = (3.52, 0.0, 0.60)  # on the chassis cradle
RAM_TOP = (3.28, 0.0, 1.235)  # under the bed floor front
TIP_MAX = 50.0


def build():
    m = Model("trailer_tipper", "Tandem tipping trailer (14 t)",
              "Generic tandem-axle agricultural rear-tipping trailer, 6.0 m bed, 560/60 R22.5, walking-beam bogie, "
              "telescopic front ram, A-frame drawbar.",
              "Ground plane below the wheel centres, x = 0 midway between the two axles, y = 0 on the centreline.")
    standard_materials(m)
    farm_materials(m)
    body = (0.07, 0.30, 0.33)
    m.mat("wall", color=(1, 1, 1), tex=("ribbed", {"color": body, "ribs": 4, "dirt": 0.35}), tile=0.6,
          double=True, role="paint")
    m.mat("floor", color=(1, 1, 1), tex=("tread_plate", {}), tile=0.4)
    mb = m.mb

    # ================================================================ chassis
    for s in (1, -1):
        M.rounded_box(mb, ((BED_X0 - 0.05 + 3.62) / 2, s * CH_Y, (CH_Z0 + CH_Z1) / 2),
                      (3.62 - BED_X0 + 0.05, 0.12, CH_Z1 - CH_Z0), 0.015, "chassis")
    for xc in (-2.50, -1.45, 0.0, 1.45, 2.60, 3.62):
        M.rounded_box(mb, (xc, 0, 1.05), (0.10, 2 * CH_Y, 0.14), 0.015, "chassis")
    # tipping hinge lugs at the rear
    for s in (1, -1):
        M.box(mb, (HINGE[0], s * CH_Y, 1.25), (0.16, 0.03, 0.12), "chassis")
    rod(mb, (HINGE[0], -0.56, HINGE[2]), (HINGE[0], 0.56, HINGE[2]), 0.03, "steel")
    m.socket("socket_tipping_hinge", None, HINGE, "bed tipping hinge axis (along y)")
    # bogie pivot hangers + shaft
    for s in (1, -1):
        M.box(mb, (0, s * CH_Y, (BOGIE_PIVOT[2] + CH_Z0) / 2 - 0.02), (0.22, 0.04, CH_Z0 - BOGIE_PIVOT[2] + 0.1),
              "chassis")
    rod(mb, (0, -0.66, BOGIE_PIVOT[2]), (0, 0.66, BOGIE_PIVOT[2]), 0.045, "steel")
    # ram cradle below the front cross member
    for s in (1, -1):
        beam(mb, (3.62, s * 0.14, 1.00), (RAM_BASE[0], s * 0.14, RAM_BASE[2] - 0.03), 0.04, 0.10, "chassis",
             up=(1, 0, 0))
    rod(mb, (RAM_BASE[0], -0.16, RAM_BASE[2]), (RAM_BASE[0], 0.16, RAM_BASE[2]), 0.03, "steel")

    # ================================================================ drawbar (A-frame, cranked down to the eye)
    ez = HITCH_HEIGHT
    apex = (4.45, 0.0, ez + 0.06)
    for s in (1, -1):
        beam(mb, (3.50, s * CH_Y, 1.05), (apex[0], s * 0.09, apex[2]), 0.10, 0.14, "chassis")
    beam(mb, (apex[0] - 0.12, 0, apex[2]), (EYE_X - 0.14, 0, ez), 0.12, 0.10, "chassis")
    beam(mb, (EYE_X - 0.16, 0, ez), (EYE_X - 0.06, 0, ez), 0.07, 0.055, "steel")  # eye neck (fits the clevis)
    M.rounded_box(mb, (apex[0] - 0.05, 0, apex[2]), (0.24, 0.30, 0.12), 0.02, "chassis")
    eye(mb, (EYE_X, 0, ez), (0, 0, 1), 0.075, 0.032, 0.05, "steel", count=20)
    m.socket("socket_hitch_eye", None, (EYE_X, 0.0, ez),
             f"hitch eye centre, {HITCH_HEIGHT:.2f} m above ground; couples to tractor socket_hitch")
    # jack (parking leg): outer tube on a plate between the A-frame arms, leg on the jack node
    JX = 4.10
    t = (JX - 3.50) / (apex[0] - 3.50)
    jz = 1.05 + (apex[2] - 1.05) * t
    jy = CH_Y + (0.09 - CH_Y) * t
    M.box(mb, (JX, 0, jz), (0.14, 2 * jy, 0.03), "chassis")
    rod(mb, (JX, 0, 0.44), (JX, 0, jz + 0.30), 0.045, "chassis", count=14)
    rod(mb, (JX, 0, jz + 0.30), (JX, 0, jz + 0.33), 0.03, "steel")
    M.tube(mb, [(JX, 0, jz + 0.31), (JX, 0.14, jz + 0.31), (JX, 0.14, jz + 0.22)], 0.009, "steel", count=6)
    jack = m.node("jack", None, (JX, 0, 0.27),
                  m.prismatic((0, 0, -1), 0.0, 0.32,
                              "parking leg extension, + = foot down; foot reaches the ground at 0.25"))
    rod(mb, (JX, 0, 0.27), (JX, 0, 0.62), 0.035, "steel", jack, count=12)
    M.rounded_box(mb, (JX, 0, 0.26), (0.20, 0.20, 0.02), 0.005, "chassis", jack)
    m.socket("socket_jack_foot", jack, (JX, 0, 0.25), "parking leg foot contact point")
    # hydraulic hose from the ram to the tractor coupler
    hose = [(RAM_BASE[0] + 0.03, 0.09, RAM_BASE[2] + 0.10), (3.62, 0.30, 0.98), (3.80, 0.40, 1.08),
            (4.05, 0.28, 0.86), (4.30, 0.17, 0.74), (4.40, 0.16, 0.73)]
    M.tube(mb, hose, 0.012, "hose", count=6)
    M.box(mb, (4.42, 0.12, 0.68), (0.06, 0.08, 0.04), "chassis")                           # coupler holder
    M.cylinder(mb, (4.40, 0.16, 0.73), (4.47, 0.16, 0.73), 0.018, "steel", count=8)       # coupler
    M.cylinder(mb, (4.47, 0.16, 0.73), (4.49, 0.16, 0.73), 0.02, "reflector_amber", count=8)

    # ================================================================ bogie + wheels
    bg = m.node("bogie", None, BOGIE_PIVOT,
                m.revolute((0, 1, 0), -5, 5, "walking-beam pitch, + = front axle down / rear axle up"))
    for s in (1, -1):
        M.rounded_box(mb, (0, s * 0.60, R + 0.02), (1.95, 0.08, 0.20), 0.02, "chassis", bg)
        eye(mb, (0, s * 0.60, BOGIE_PIVOT[2]), (0, 1, 0), 0.08, 0.045, 0.10, "chassis", bg)
    for ax in (AXLE_X, -AXLE_X):
        beam(mb, (ax, -0.86, R), (ax, 0.86, R), 0.12, 0.12, "mud", bg, up=(0, 0, 1))
    for key, x, y in (("1L", AXLE_X, 1), ("1R", AXLE_X, -1), ("2L", -AXLE_X, 1), ("2R", -AXLE_X, -1)):
        corner(m, key, (x, y * TRACK / 2, R), TYRE, parent=bg, steer=False, susp_node=False, caliper=False,
               tag="T")
    # mudguards (fixed to the chassis, spanning both wheels)
    for s in (1, -1):
        y0, y1 = sorted((s * 0.64, s * 1.26))
        rg = 0.71
        arc_shell(mb, "", "chassis", (AXLE_X, R), y0, y1, rg, math.radians(15), math.radians(90), t=0.01, n=10)
        arc_shell(mb, "", "chassis", (-AXLE_X, R), y0, y1, rg, math.radians(90), math.radians(165), t=0.01,
                  n=10)
        M.box(mb, (0, s * 0.95, R + rg + 0.005), (2 * AXLE_X, 0.62, 0.01), "chassis")
        for xb in (-0.4, 0.4):
            M.box(mb, (xb, s * 0.60, R + rg - 0.01), (0.05, 0.24, 0.03), "chassis")
        for xr in (-1.2, 1.2):
            M.box(mb, (xr, s * 1.262, R + rg - 0.12), (0.08, 0.006, 0.04), "reflector_amber")

    # ================================================================ rear: lights, under-run bar
    for s in (1, -1):
        beam(mb, (BED_X0 - 0.02, s * CH_Y, 1.02), (BED_X0 - 0.10, s * 1.00, 1.02), 0.06, 0.06, "chassis")
        lc = (BED_X0 - 0.13, s * 1.00, 1.02)
        M.rounded_box(mb, lc, (0.06, 0.34, 0.13), 0.015, "trim_black")
        for (yc, lens) in ((0.88, "light_tail"), (0.99, "light_brake"),
                           (1.10, "light_indicator_left" if s > 0 else "light_indicator_right")):
            M.box(mb, (lc[0] - 0.031, s * yc, lc[2]), (0.008, 0.10, 0.10), lens)
        M.box(mb, (BED_X0 - 0.02, s * 0.70, 0.78), (0.05, 0.05, 0.40), "chassis")      # under-run hangers
        tri_reflector(mb, "", (BED_X0 - 0.06, s * 0.70, 0.63), 0.15, "reflector_red")
    M.rounded_box(mb, (BED_X0 - 0.03, 0, 0.62), (0.07, 1.90, 0.10), 0.015, "chassis")    # under-run bar

    # ================================================================ bed (tips about the rear hinge)
    bed = m.node("bed", None, HINGE,
                 m.revolute((0, -1, 0), 0, TIP_MAX, "tipping angle about the rear hinge, + = front up"))
    xo0, xo1 = BED_X0 - WALL_T, BED_X1 + WALL_T
    yo = BED_YI + WALL_T
    L = BED_X1 - BED_X0
    xc = (BED_X0 + BED_X1) / 2
    # floor: tread plate on top, painted below
    M.box(mb, (xc, 0, FLOOR - 0.02), (L, 2 * yo, 0.04), "chassis", bed, face_materials={"+z": "floor"})
    # subframe: longitudinal beams on the chassis + tapered cross members
    for s in (1, -1):
        M.rounded_box(mb, ((xo0 + xo1) / 2, s * CH_Y, (CH_Z1 + 0.01 + FLOOR - 0.04) / 2),
                      (xo1 - xo0, 0.10, FLOOR - 0.04 - CH_Z1 - 0.01), 0.01, "chassis", bed)
    for k in range(9):
        x = BED_X0 + 0.25 + k * (L - 0.5) / 8
        for s in (1, -1):
            pts = [(x - 0.04, s * 0.0, FLOOR - 0.04), (x - 0.04, s * yo, FLOOR - 0.04),
                   (x - 0.04, s * yo, FLOOR - 0.10), (x - 0.04, s * 0.0, FLOOR - 0.16)]
            M.extrude(mb, [(p[1], p[2]) for p in pts], 0.08, "chassis", bed,
                      xf=((0, 0, 1, x), (1, 0, 0, 0), (0, 1, 0, 0)))
    # hinge eyes under the rear, ram bracket under the front
    for s in (1, -1):
        eye(mb, (HINGE[0], s * (CH_Y + 0.05), HINGE[2]), (0, 1, 0), 0.06, 0.032, 0.04, "chassis", bed)
        M.box(mb, (HINGE[0], s * (CH_Y + 0.05), (HINGE[2] + FLOOR - 0.04) / 2 + 0.02),
              (0.10, 0.04, FLOOR - 0.04 - HINGE[2]), "chassis", bed)
    M.rounded_box(mb, (RAM_TOP[0], 0, (RAM_TOP[2] + FLOOR - 0.04) / 2), (0.22, 0.26, FLOOR - 0.04 - RAM_TOP[2] + 0.04),
                  0.02, "chassis", bed)
    # side walls, headboard
    for s in (1, -1):
        M.box(mb, (xc, s * (BED_YI + WALL_T / 2), FLOOR + WALL_H / 2), (L + 2 * WALL_T, WALL_T, WALL_H), "wall", bed)
        M.rounded_box(mb, (xc, s * (yo - 0.01), FLOOR + WALL_H + 0.03), (L + 2 * WALL_T + 0.02, 0.08, 0.07), 0.015,
                      "paint", bed)  # top rail
        M.rounded_box(mb, (xc, s * (yo + 0.015), FLOOR - 0.03), (L + 2 * WALL_T, 0.07, 0.14), 0.015, "paint", bed)
        for k in range(7):  # stakes
            x = BED_X0 + 0.02 + k * L / 6
            x = min(max(x, BED_X0 + 0.05), BED_X1 - 0.05)
            M.rounded_box(mb, (x, s * (yo + 0.02), FLOOR + WALL_H / 2), (0.08, 0.04, WALL_H), 0.01, "paint", bed)
        for x in (BED_X0 + 0.4, xc, BED_X1 - 0.4):
            M.box(mb, (x + 0.5, s * (yo + 0.042), FLOOR + 0.03), (0.08, 0.004, 0.04), "reflector_amber", bed)
    M.box(mb, (BED_X1 + WALL_T / 2, 0, FLOOR + WALL_H / 2), (WALL_T, 2 * yo, WALL_H), "wall", bed)
    M.rounded_box(mb, (BED_X1 + WALL_T - 0.01, 0, FLOOR + WALL_H + 0.03), (0.08, 2 * yo + 0.02, 0.07), 0.015, "paint",
                  bed)
    for y in (-0.7, 0.0, 0.7):
        M.rounded_box(mb, (BED_X1 + WALL_T + 0.02, y, FLOOR + WALL_H / 2), (0.04, 0.08, WALL_H), 0.01, "paint", bed)
    # ladder on the headboard (left)
    lx = BED_X1 + WALL_T + 0.10
    for yy in (0.95, 0.60):
        rod(mb, (lx, yy, FLOOR - 0.35), (lx, yy, FLOOR + WALL_H + 0.05), 0.016, "steel", bed, count=8)
        rod(mb, (lx - 0.10, yy, FLOOR + WALL_H + 0.05), (lx, yy, FLOOR + WALL_H + 0.05), 0.016, "steel", bed, count=8)
    for k in range(6):
        z = FLOOR - 0.28 + k * 0.30
        rod(mb, (lx, 0.60, z), (lx, 0.95, z), 0.014, "steel", bed, count=8)
    m.socket("socket_bed_centre", bed, (xc, 0.0, FLOOR + WALL_H / 2),
             "centroid of the bed's inner volume (load mass point when level-full)")

    # tailgate
    tg_p = (xo0 - WALL_T / 2 + 0.0, 0.0, FLOOR + WALL_H + 0.02)
    tg = m.node("tailgate", bed, tg_p,
                m.revolute((0, 1, 0), 0, 100, "tailgate swing about its top hinge, + = bottom swings out"))
    tx = xo0 - WALL_T / 2
    M.box(mb, (tx, 0, FLOOR + WALL_H / 2), (WALL_T, 2 * yo, WALL_H - 0.01), "wall", tg)
    for zz in (FLOOR + 0.03, FLOOR + WALL_H - 0.04):
        M.rounded_box(mb, (tx - 0.02, 0, zz), (0.05, 2 * yo, 0.07), 0.015, "paint", tg)
    for y in (-yo + 0.04, -0.4, 0.4, yo - 0.04):
        M.rounded_box(mb, (tx - 0.035, y, FLOOR + WALL_H / 2), (0.04, 0.07, WALL_H), 0.01, "paint", tg)
    M.rounded_box(mb, (tx - 0.06, 0, FLOOR + 0.35), (0.02, 0.45, 0.30), 0.02, "chassis", tg)   # grain hatch
    for s in (1, -1):
        eye(mb, (tg_p[0], s * (yo - 0.06), tg_p[2]), (0, 1, 0), 0.04, 0.018, 0.05, "chassis", tg)
        M.box(mb, (tg_p[0] + 0.03, s * (yo - 0.01), tg_p[2]), (0.08, 0.03, 0.08), "chassis", bed)  # hinge lug
        M.box(mb, (xo0 + 0.02, s * (yo + 0.03), FLOOR + 0.05), (0.10, 0.04, 0.10), "chassis", bed)  # lock hook
        tri_reflector(mb, tg, (tx - 0.06, s * 0.95, FLOOR + 0.25), 0.15, "reflector_red")

    # ================================================================ tipping ram
    # outer barrel on the chassis cradle (aims at the bed bracket); inner stages hang from the bed bracket
    # and stretch back to the barrel base, so the ram extends from 0.66 m to ~5.7 m.
    d = sub(RAM_TOP, RAM_BASE)
    L0 = math.dist(RAM_BASE, RAM_TOP)
    u = norm(d)
    rb = m.node("ram_barrel", None, RAM_BASE, m.aim("bed", RAM_TOP, note="tipping ram outer barrel, follows the bed"))
    rod(mb, RAM_BASE, add(RAM_BASE, mul(u, L0 * 0.80)), 0.10, "chassis", rb, count=20)
    rod(mb, add(RAM_BASE, mul(u, L0 * 0.80 - 0.02)), add(RAM_BASE, mul(u, L0 * 0.80)), 0.112, "steel", rb, count=20)
    eye(mb, RAM_BASE, (0, 1, 0), 0.07, 0.03, 0.12, "chassis", rb)
    rs = m.node("ram_stages", "bed", RAM_TOP,
                m.aim("ram_barrel", RAM_BASE, stretch=True,
                      note="tipping ram inner stages: stretch from the bed bracket back to the barrel base"))
    # stage k spans t in [t0, t1] measured from the barrel base (0) to the bed bracket (1)
    stages = [(0.05, 0.42, 0.085), (0.40, 0.63, 0.072), (0.61, 0.82, 0.059), (0.80, 1.0, 0.046)]
    for (t0, t1, r) in stages:
        p0, p1 = add(RAM_BASE, mul(d, t0)), add(RAM_BASE, mul(d, t1))
        rod(mb, p0, p1, r, "rod", rs, count=18)
    M.ellipsoid(mb, RAM_TOP, (0.05, 0.08, 0.05), "chassis", rs, count=12, rings=6)

    # ================================================================ notes
    m.notes += [
        "Generic tandem tipping trailer; dimensions are class-typical, not a specific production machine.",
        f"Hitch eye (socket_hitch_eye) at x = {EYE_X:.2f}, z = {HITCH_HEIGHT:.2f} m (HITCH_HEIGHT, same value as "
        "tractor.HITCH_HEIGHT): coupled on flat ground it coincides with the tractor's socket_hitch and the "
        "trailer stands level.",
        f"Drawbar length for off-tracking: hitch eye to axle-group centre (origin, bogie pivot x) = {EYE_X:.2f} m "
        "(horizontal).",
        "Axles on a walking-beam bogie (bogie joint, +-5 deg, pivot at x = 0, z = 0.72); axle spacing 1.60 m, "
        f"track {TRACK:.2f} m; no steering. Tyres touch z = 0 at rest.",
        f"Bed tips about socket_tipping_hinge (x = {HINGE[0]:.2f}, z = {HINGE[2]:.2f}), 0..{TIP_MAX:.0f} deg. "
        f"Tipping ram: base {RAM_BASE} (chassis) to bed bracket {RAM_TOP} (bed), closed length {L0:.2f} m; "
        "barrel = aim link without stretch, inner stages = aim link with stretch along the rest direction.",
        "Bed inner 6.00 x 2.30 x 1.20 m = 16.6 m3 (floor top 1.40 m); socket_bed_centre is the centroid of the "
        "inner volume (child of bed, so it follows the tipping).",
        "Tailgate hangs from its top hinge (child of bed); + swings the bottom rearward (free-swinging or latched).",
        "Parking jack: prismatic, foot 0.25 m above ground at 0; value 0.25 touches the ground with the drawbar "
        "at hitch height.",
        "Mass hints (not in the geometry): ~5.5 t empty, 14 t payload, ~1.5-2 t static drawbar load loaded.",
    ]
    return m
