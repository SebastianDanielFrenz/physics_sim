"""c) Hyper car: a generic mid-engine two-seat hypercar (class-typical, no real model).

Length 4.65 m, width 2.05 m (body, no mirrors), height 1.15 m (roof; the rear
wing sits below the roof when retracted), wheelbase 2.70 m, track 1.70 m front /
1.65 m rear (between tyre centres). Ground clearance 0.10 m.
Tyres front 265/35 R20 (r = 0.347 m), rear 325/30 R21 (r = 0.364 m), centre-lock
rims, carbon-ceramic discs with large coloured calipers.
Interactive: 4 corners (travel, front steer, spin), butterfly doors on tilted
hinge axes at the A-pillar base, frunk lid, rear engine cover, steering wheel,
active rear wing (lift on its struts + pitch about the leading edge), active
front flaps, lamps (head, tail, brake, indicators, reverse) as thin LED strips.
Aero regions for the physics side are named nodes: `splitter`, `diffuser`,
`wing_lift`/`wing_flap`, `flap_front_L/R` (see the rig notes).
"""
import math

from ..scene import Model
from ..carbody import CarBody, Panel, resample_loop
from ..parts import standard_materials, corner, WheelSpec, steering_wheel, seat
from .. import mesh as M
from ..vec import Curve, norm, smoothstep, compose, translate, rot_axis

WB = 2.70
TRACK_F, TRACK_R = 1.70, 1.65
XF, XR = WB / 2, -WB / 2
TYRE_F = WheelSpec(radius=0.347, width=0.265, rim_radius=0.254, tyre="road", rim="centerlock", spokes=10, solid_visual=True,
                   rim_color=(0.16, 0.16, 0.17), rim_metallic=0.8, caliper_color=(0.95, 0.72, 0.05))
TYRE_R = WheelSpec(radius=0.364, width=0.325, rim_radius=0.267, tyre="road", rim="centerlock", spokes=10, solid_visual=True,
                   rim_color=(0.16, 0.16, 0.17), rim_metallic=0.8, caliper_color=(0.95, 0.72, 0.05))

# Rear wing (main plane on wing_flap), rest pose. Pitch = chord angle, trailing edge up.
WING_LE = (-1.93, 0.0, 0.975)
WING_CHORD, WING_SPAN, WING_PITCH = 0.30, 1.72, math.radians(8.0)
WING_LIFT_MAX = 0.28
WING_FLAP_RANGE = (-8.0, 55.0)   # deg about the rest pitch: -8 = flat (low drag), +55 = airbrake

# Front active flaps (one per side, in front of the front wheels on the splitter)
FLAP_LE_X, FLAP_Y, FLAP_Z, FLAP_CHORD, FLAP_SPAN = 2.31, 0.60, 0.128, 0.13, 0.34


class HyperBody(CarBody):
    """CarBody with two local extensions (kept in this module, the shared library is untouched):
    - fender bulges: the upper half of a section is lifted by bulge(x) * g(|y|/W), so the
      fenders rise above the low frunk/engine deck (g ramps from 0 at fender[0] to 1 at fender[1]);
    - side intakes: side points inside an (x, z) window are pushed inward by the intake depth.
    contains()/surface_y() include the bulge so patch() drapes lamps onto the bulged fenders."""

    def __init__(self, model, *, bulge, fender=(0.55, 0.8), intakes=(), **kw):
        super().__init__(model, **kw)
        self.bulge = Curve(bulge)
        self.fender = fender
        self.intakes = list(intakes)

    def _inner_skins(self):
        # The shared CarBody leaves paint-only reverse copies with unstitched edges.
        # Here all material regions (including glazing) own real perimeter returns.
        from .hyper_shell import skin_part
        self.model.flow_outer = {key:len(p.idx) for key,p in self.model.mb.prims.items()}
        for part in list(self.model.mb.parts()):
            skin_part(self.model.mb,part,thickness=.018,inset=.001 if part in
                      ('door_L','door_R','frunk','engine_cover','diffuser') else 0)
        self.model.flow_after_shell = {key:len(p.idx) for key,p in self.model.mb.prims.items()}

    def patch(self,outline,mat,face,part="",offset=.004,rings=5,skirt_mat=None,skirt=.004):
        # Same visible drape as CarBody; real backing behind the grille/lens,
        # not a cap across an intake or exhaust mouth.
        from .hyper_shell import clean_degenerate
        mb=self.model.mb
        def place(a,b,off):
            if face in ("front","rear"):
                e=1 if face=="front" else -1
                return (self.surface_x(a,b,e)+e*off,a,b)
            side=1 if face=="left" else -1
            return (a,side*(self.surface_y(a,b)+off),b)
        want={"front":(1,0,0),"rear":(-1,0,0),"left":(0,1,0),"right":(0,-1,0)}[face]
        if M.dot(M.newell([place(a,b,0) for a,b in outline]),want)<0: outline=list(reversed(outline))
        ca=sum(p[0] for p in outline)/len(outline);cb=sum(p[1] for p in outline)/len(outline)
        def rows(off):
            return [[place(ca+(a-ca)*k/rings,cb+(b-cb)*k/rings,off) for a,b in outline] for k in range(rings+1)]
        front=rows(offset);recs=mb.grid(front,mat,part,closed_v=True)
        back=rows(offset-skirt-.004)
        mb.grid([back[-1],front[-1]],skirt_mat or mat,part,closed_v=True,flip=True)
        mb.grid(back,skirt_mat or mat,part,closed_v=True,flip=True)
        clean_degenerate(mb)
        return recs

    def _g(self, yn):
        return smoothstep(self.fender[0], self.fender[1], yn)

    @staticmethod
    def _s(t):
        """Vertical profile of the bulge: saturates above the section's mid height, so the
        fender side wall rises with it (the arch cut then stays below the fender top)."""
        return smoothstep(0.0, 0.6, t)

    def _lift(self, x, y, z, W, zc, H):
        if z <= zc:
            return z
        b = self.bulge(x) * self._g(abs(y) / W)
        return z + b * self._s((z - zc) / H) if b > 0 else z

    @staticmethod
    def _ramp(lo, hi, v, w):
        return smoothstep(lo - w, lo + w, v) * smoothstep(hi + w, hi - w, v)

    def intake_w(self, x, ls):
        """0..1 recess weight of the side intake at station x and section position ls.
        The window is bounded by grid lines (x stations, ls), so its edges come out crisp."""
        for it in self.intakes:
            f = self._ramp(it["x0"], it["x1"], x, 0.02) * self._ramp(it["ls0"], it["ls1"], ls, 0.012)
            if f > 0:
                return f
        return 0.0

    def in_intake(self, x, ls):
        return any(it["x0"] < x < it["x1"] and it["ls0"] < ls < it["ls1"] for it in self.intakes)

    def intake_depth(self, x):
        for it in self.intakes:
            if it["x0"] - 0.05 < x < it["x1"] + 0.05:
                # scoop: deepest at the rear
                return it["depth"] * (0.5 + 0.5 * smoothstep(it["x1"], it["x0"], x))
        return 0.0

    def contains(self, x, y, z):
        if not (self.x_rear <= x <= self.x_front):
            return False
        W, zb, zt, n = self.lower_params(x)
        zc, H = (zt + zb) / 2, (zt - zb) / 2
        if z > zc:
            b = self.bulge(x) * self._g(abs(y) / W)
            if b > 0:  # invert the monotone lift z0 -> z0 + b s((z0 - zc) / H)
                lo, hi = zc, z
                for _ in range(30):
                    mid = (lo + hi) / 2
                    if mid + b * self._s((mid - zc) / H) < z:
                        lo = mid
                    else:
                        hi = mid
                z = lo
        return (abs(y) / W) ** n + (abs(z - zc) / H) ** n <= 1.0

    def surface_y(self, x, z):
        W, zb, zt, n = self.lower_params(x)
        if not self.contains(x, 0.0, z):
            return 0.0
        lo, hi = 0.0, W
        for _ in range(40):
            mid = (lo + hi) / 2
            if self.contains(x, mid, z):
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2  # ignores the side intakes

    def _green_section(self, x):
        """CarBody's greenhouse loop, with the belt flange (first 'side' point) pulled in to stay
        under this body's rounded, bulged shoulder: CarBody puts it at the section half width,
        which assumes a flat body top and made it poke out as a dark band along the doors."""
        loop = super()._green_section(x)
        nb = self.GREEN_SEGS[0][1]
        L = sum(c for _, c in self.GREEN_SEGS) + 1
        for i in (nb, L + (L - 2 - nb)):
            px, py, pz = loop[i]
            wb = self.green["wb"](x)
            y = max(wb, min(abs(py), self.surface_y(x, pz) - 0.015))
            loop[i] = (px, math.copysign(y, py), pz)
        return loop

    def _lower_section(self, x):
        W, zb, zt, n = self.lower_params(x)
        zc, H = (zt + zb) / 2, (zt - zb) / 2
        pts2 = M.superellipse(W, H, n, 720, 0.0, zc)
        pts2 = pts2[540:] + pts2[:540]
        pts2 = [(y, self._lift(x, y, z, W, zc, H)) for (y, z) in pts2]
        pts2 = resample_loop(pts2, self.around)
        d = self.intake_depth(x)
        out = []
        ab = self.arch_band
        n_low = len(pts2)
        for k, (y, z) in enumerate(pts2):
            if d > 0:
                f = self.intake_w(x, min(k, n_low - k) / (n_low / 2))
                if f > 0:
                    y = math.copysign(abs(y) - d * f, y)
            # Sculpted door cove: a continuous concavity sweeping upward into the intake.
            waist = math.exp(-((x + 0.02) / 0.78) ** 4)
            waist_z = 0.36 + 0.13 * smoothstep(0.85, -0.65, x)
            cove = 0.09 * waist * math.exp(-((z - waist_z) / 0.105) ** 2)
            y = math.copysign(max(0.0, abs(y) - cove), y)
            px, pz = x, z
            for w in self.wheels:  # arch carving, as CarBody._lower_section
                y_in = w["y_in"] - self.arch_depth_in
                if abs(y) <= y_in:
                    continue
                ra = w["r"] + self.arch_clear
                dx, dz = x - w["x"], z - w["z"]
                dd = math.hypot(dx, dz)
                if dd >= ra:
                    continue
                ux, uz = (dx / dd, dz / dd) if dd > 1e-9 else (0.0, 1.0)
                if dd >= ra - ab:
                    px, pz = w["x"] + ux * ra, w["z"] + uz * ra
                elif dd >= ra - 2 * ab:
                    px, pz = w["x"] + ux * (ra - 0.008), w["z"] + uz * (ra - 0.008)
                    y = math.copysign(y_in, y)
                else:
                    y = math.copysign(y_in, y)
                break
            out.append((px, y, pz))
        return out


def wing_mesh(mb, part, le, chord, span, pitch, thick, camber, mat, count=36, y0=None):
    """Inverted (downforce) airfoil main plane, spanwise along y, leading edge at `le`,
    chord running rearward and up by `pitch` (trailing edge up)."""
    ea = (-math.cos(pitch), 0.0, math.sin(pitch))       # chordwise, rearward
    eb = (-math.sin(pitch), 0.0, -math.cos(pitch))      # "upper" airfoil side points down: inverted
    sec = M.naca(chord, thick, camber, 0.45, count)
    secs = []
    for yy in ((-span / 2, span / 2) if y0 is None else y0):
        o = (le[0], le[1] + yy, le[2])
        secs.append(M.place2d(sec, o, ea, eb))
    return M.loft(mb, secs, mat, part)


def slab_xz(mb, poly_xz, y, thick, mat, part=""):
    """Thin plate: 2D outline in the (x, z) plane, extruded along y (centred on y)."""
    xf = ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, -1.0, y), (0.0, 1.0, 0.0, 0.0))  # a->x, b->z, c->-y
    return M.extrude(mb, poly_xz, thick, mat, part, xf=xf)


def surface_z(body, x, y):
    """Top of the lower body at (x, |y|) (bisection on HyperBody.contains)."""
    lo, hi = 0.0, 1.5
    if not body.contains(x, y, lo + 0.3):
        return 0.0
    lo = 0.3
    for _ in range(40):
        mid = (lo + hi) / 2
        if body.contains(x, y, mid):
            lo = mid
        else:
            hi = mid
    return lo


def build():
    m = Model("car_hyper", "Sculpted A — silver mid-engine hypercar",
              "Sculpted A: silver organic body, dark teardrop canopy, carbon aero, 4.65 m, 265/35 R20 front, 325/30 R21 rear, "
              "butterfly doors, active rear wing.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    standard_materials(m)
    m.mat("centerlock_nut", color=(0.64, 0.52, 0.25), metallic=0.85, roughness=0.25)
    # Clean metallic silver: fine paint grain is deliberately below the mesh's texel scale.
    m.mat("paint", color=(0.78, 0.80, 0.84), metallic=0.92, roughness=0.22,
          double=True, role="paint")
    m.materials["glass"].alpha = 0.18
    m.materials["glass"].color = (0.60, 0.70, 0.76)
    m.materials["glass"].roughness = 0.10
    m.materials["glass"].metallic = 0.0
    m.mat("carbon", color=(1, 1, 1), tex=("carbon", {}), tile=0.12, roughness=0.2, double=True)
    m.mat("intake", color=(1, 1, 1), tex=("honeycomb", {"cells": 6, "color": (0.03, 0.03, 0.035)}), tile=0.1,
          roughness=0.7)
    m.mat("seat", color=(1, 1, 1), tex=("fabric", {"color": (0.1, 0.1, 0.11)}), tile=0.2)
    m.mat("seat_accent", color=(0.36, 0.39, 0.44), roughness=0.6)
    m.mat("exhaust_tip", color=(0.35, 0.3, 0.28), metallic=1.0, roughness=0.35)
    m.mat("exhaust_soot", color=(0.02, 0.02, 0.02), roughness=1.0)
    m.mat("engine_red", color=(0.55, 0.05, 0.03), metallic=0.3, roughness=0.35)

    RF, RR = TYRE_F.radius, TYRE_R.radius
    wheels = [dict(x=XF, z=RF, r=RF, y_out=TRACK_F / 2 + TYRE_F.width / 2, y_in=TRACK_F / 2 - TYRE_F.width / 2),
              dict(x=XR, z=RR, r=RR, y_out=TRACK_R / 2 + TYRE_R.width / 2, y_in=TRACK_R / 2 - TYRE_R.width / 2)]

    X_REAR, X_FRONT = -2.35, 2.30
    intake = dict(x0=-0.97, x1=-0.59, ls0=0.445, ls1=0.605, depth=0.14)
    body = HyperBody(
        m, x_rear=X_REAR, x_front=X_FRONT,
        W=[(-2.35, 0.92), (-2.13, 1.00), (-1.55, 1.025), (-1.0, 1.01),
           (-0.40, 0.915), (0.25, 0.92), (0.80, 0.965), (1.35, 1.00),
           (1.80, 0.97), (2.15, 0.91), (2.30, 0.85)],
        zb=[(-2.35, 0.33), (-2.1, 0.24), (-1.75, 0.105), (1.7, 0.10), (2.1, 0.12), (2.30, 0.17)],
        zt=[(-2.35, 0.83), (-2.15, 0.9), (-1.8, 0.92), (-1.2, 0.905), (-0.6, 0.83), (0.5, 0.80), (0.95, 0.78),
            (1.35, 0.63), (1.9, 0.50), (2.30, 0.40)],
        exp=[(-2.35, 3.5), (-1.8, 4.0), (-0.5, 3.6), (0.7, 3.8), (1.8, 4.0), (2.30, 3.2)],
        bulge=[(-2.35, 0.0), (-2.1, 0.05), (-1.50, 0.11), (-1.10, 0.07),
               (-0.65, 0.0), (0.20, 0.0), (0.75, 0.02), (1.0, 0.14),
               (1.35, 0.28), (1.75, 0.24), (2.1, 0.13), (2.30, 0.04)],
        fender=(0.30, 0.74), intakes=[intake],
        end_r=(0.16, 0.2), wheels=wheels, around=136, step=0.022, arch_clear=0.05,
        green=dict(x0=-1.22, x1=0.97,
                   zroof=[(-1.22, 0.905), (-0.9, 0.99), (-0.55, 1.07), (-0.15, 1.105), (0.2, 1.095), (0.55, 1.0),
                          (0.97, 0.765)],
                   wb=[(-1.22, 0.42), (-0.9, 0.62), (-0.4, 0.70), (0.5, 0.70), (0.97, 0.63)],
                   wr=[(-1.22, 0.3), (-0.9, 0.45), (-0.3, 0.52), (0.4, 0.5), (0.97, 0.46)],
                   r=0.16, crown=0.05))


    W = body.W

    # ---- opening panels
    DX0, DX1 = -0.58, 0.90      # door: from the intake to just behind the front arch (A-pillar base)
    doors = [("door_L", 1), ("door_R", -1)]

    def door_inside(side):
        def f(c, n):
            return (DX0 - 0.012 < c[0] < DX1 + 0.012 and c[1] * side > 0.3 and c[2] > 0.235
                    and (n[1] * side > 0.3 or n[2] > 0.3) and n[2] > -0.5 and not in_arch(c))
        return f

    def door_border(c, n):
        return c[0] < DX0 + 0.01 or c[0] > DX1 - 0.01 or c[2] < 0.255

    for name, side in doors:
        body.panels.append(Panel(name, door_inside(side), door_border))

    LS_FRUNK, LS_COVER = 0.80, 0.78
    FR_X0, FR_X1 = 1.00, 2.12
    body.panels.append(Panel(
        "frunk", lambda c, n: FR_X0 < c[0] < FR_X1 and body.ls > LS_FRUNK and n[2] > 0.3,
        lambda c, n: c[0] < FR_X0 + 0.01 or c[0] > FR_X1 - 0.01 or body.ls < LS_FRUNK + 0.007))
    EC_X0, EC_X1 = -2.22, -1.24
    body.panels.append(Panel(
        "engine_cover", lambda c, n: EC_X0 < c[0] < EC_X1 and body.ls > LS_COVER and n[2] > 0.3,
        lambda c, n: c[0] > EC_X1 - 0.01 or c[0] < EC_X0 + 0.01 or body.ls < LS_COVER + 0.007))

    def in_arch(c):
        """Arch liner = inside the arch circle AND out at the wheel well (the frunk and engine
        deck dip below the arch tops, CarBody.in_arch would paint them as liner)."""
        for w in wheels:
            if (math.hypot(c[0] - w["x"], c[2] - w["z"]) < w["r"] + body.arch_clear - 0.001
                    and w["y_in"] - 0.04 < abs(c[1]) < body.section_half_width(c[0]) - 0.012):
                return True
        return False

    def lower(c, n, b):
        x, y, z = c
        ay = abs(y)
        if n[2] < -0.6:
            if x < -1.72:
                return ("diffuser", "carbon")
            return "underbody"
        if in_arch(c):
            return "arch_liner"
        # side intake: dark recess
        if b.in_intake(x, b.ls):
            return "intake"
        # carbon sills between the arches
        if -0.98 < x < 0.95 and z < 0.235 and ay > 0.5:
            return "carbon"
        # rear: diffuser surround and venting mesh
        if x < -2.0 and n[0] < -0.3:
            if z < 0.42:
                return ("diffuser", "carbon")
        # nose: lower lip in carbon
        if x > 2.0 and n[0] > 0.3 and z < 0.21:
            return "carbon"
        # engine cover: glass window over the engine, louvres behind it
        if EC_X0 + 0.06 < x < EC_X1 - 0.06 and b.ls > LS_COVER + 0.06 and n[2] > 0.3:
            if x > -1.72:
                return ("engine_cover", "glass")
            if x < -1.78 and b.ls > 0.9:
                return ("engine_cover", "intake")
        return b.panel_face(c, n, "paint")  # not default_lower(): its arch test also hits the low frunk

    def green(c, n, b):
        x, y, z = c
        seg, pos = b.gseg, b.gpos
        if seg == "roof":  # windshield / carbon roof / rear screen, glass with a black ceramic border
            if x > 0.30:
                return "trim_black" if pos < 0.1 or x < 0.37 or x > 0.90 else "glass"
            if x < -0.62:
                return "trim_black" if pos < 0.1 or x > -0.68 or x < -1.12 else "glass"
            return "carbon"
        side = 1 if y > 0 else -1
        name = "door_L" if side > 0 else "door_R"
        if seg == "corner":
            if x > 0.30:
                # This curved band belongs to the wraparound windscreen.
                # Painting the entire band made giant opaque wedges at eye level.
                return "glass" if 0.37 < x < 0.90 else "trim_black"
            if DX0 + 0.05 < x < 0.30:
                return (name, "glass")
            return (name, "trim_black") if DX0 < x < DX1 else "trim_black"
        if DX0 < x < DX1:  # side window in the door, framed
            if 0.15 < pos < 0.9 and DX0 + 0.05 < x < DX1 - 0.04:
                return (name, "glass")
            return (name, "trim_black")
        return "trim_black" if x > 0.0 else "paint"

    body.build(lower, green)

    # ---- lamps, intakes: patches draped onto the body surface
    rr = M.rrect
    for s in (1, -1):
        ind = "light_indicator_left" if s > 0 else "light_indicator_right"
        body.patch(rr(0.42, 0.115, 0.035, 4, s * 0.60, 0.245), "intake", "front",
                   skirt_mat="carbon", offset=0.003)
        def top_lens(outline, mat, off):
            rows = []
            cx = sum(v[0] for v in outline) / len(outline)
            cy = sum(v[1] for v in outline) / len(outline)
            for k in range(6):
                t = k / 5
                rows.append([(cx+(x-cx)*t, s*(cy+(y-cy)*t),
                              surface_z(body,cx+(x-cx)*t,cy+(y-cy)*t)+off)
                             for x,y in outline])
            m.mb.grid(rows,mat,closed_v=True,flip=s<0)
            back=[[(x,y,z-.004) for x,y,z in row] for row in rows]
            m.mb.grid(back,mat,closed_v=True,flip=s>=0)
            m.mb.grid([back[-1],rows[-1]],mat,closed_v=True,flip=s>=0)
        top_lens([(1.72,0.82),(2.16,0.46),(2.19,0.55),(1.80,0.86)],"trim_black",0.006)
        top_lens([(1.76,0.82),(2.14,0.51),(2.145,0.527),(1.78,0.834)],"light_head",0.010)
        top_lens([(1.81,0.825),(2.12,0.555),(2.12,0.567),(1.83,0.838)],"light_head",0.010)
        body.patch(rr(0.07,0.018,0.006,2,s*0.85,0.415),ind,"front",offset=0.005,skirt=0)
        # rear: indicator and brake segments on the full-width LED strip, reverse lamps
        body.patch(rr(0.12, 0.024, 0.01, 2, s * 0.80, 0.765), ind, "rear", offset=0.006, skirt=0)
        body.patch(rr(0.26, 0.018, 0.008, 2, s * 0.55, 0.765), "light_brake", "rear", offset=0.006, skirt=0)
        body.patch(rr(0.14, 0.025, 0.01, 2, s * 0.52, 0.69), "light_reverse", "rear", offset=0.004, skirt=0)
    body.patch(rr(1.70, 0.03, 0.013, 2, 0.0, 0.765), "light_tail", "rear", skirt_mat="trim_black", offset=0.004)
    body.patch(rr(0.40, 0.12, 0.04, 4, 0.0, 0.235), "intake", "front", skirt_mat="carbon", offset=0.002)
    body.patch(rr(1.46, 0.19, 0.05, 4, 0.0, 0.555), "intake", "rear", skirt_mat="carbon", offset=0.002)
    body.patch(rr(0.50, 0.11, 0.01, 2, 0.0, 0.555), "plate", "rear", skirt_mat="trim_black", offset=0.006)

    # ---- corners
    for key, x, y, spec, steer in (("FL", XF, TRACK_F / 2, TYRE_F, True), ("FR", XF, -TRACK_F / 2, TYRE_F, True),
                                   ("RL", XR, TRACK_R / 2, TYRE_R, False), ("RR", XR, -TRACK_R / 2, TYRE_R, False)):
        corner(m, key, (x, y, spec.radius), spec, steer=steer, steer_limits=(-32, 32), travel=(-0.06, 0.05))

    # ---- butterfly doors
    # Hinge at the A-pillar base. Wanted: with + the door's rear edge (direction d = -x from
    # the hinge) moves up AND outward. For a unit axis a the velocity of d is a x d =
    # (0, -a_z, a_y) (d = (-1, 0, 0)), so a_y > 0 lifts it and a_z < 0 moves it to +y (outward
    # on the left). The small +x component rakes the axis like the A-pillar so the upper
    # window frame swings forward/inward less. Left: a = norm(0.30, 0.75, -0.55) -> the
    # rear edge moves (0, 0.55, 0.75)/|a|: up and out, roughly 54 deg above horizontal.
    # Right door: an axis is a pseudovector, so under the mirror y -> -y it maps to
    # (-a_x, a_y, -a_z) = (-0.30, 0.75, 0.55); check: a x d = (0, -0.55, 0.75) -> up and out (-y).
    for name, side in doors:
        axis = norm((0.30 * side, 0.75, -0.55 * side))
        hx = DX1 - 0.02
        m.node(name, None, (hx, side * 0.80, 0.80), m.revolute(axis, 0, 75, "butterfly door opening angle"))
        # carbon mirror on a stalk from the door's shoulder
        mx, my = 0.66, 0.86
        mz = surface_z(body, mx, my)
        root = (mx + 0.02, side * my, mz - 0.01)
        head = (mx - 0.03, side * (my + 0.13), mz + 0.12)
        M.tube(m.mb, [root, (mx, side * (my + 0.06), mz + 0.07), (head[0] + 0.01, head[1] - side * 0.05, head[2])],
               0.012, "carbon", name, count=8)
        M.rounded_box(m.mb, head, (0.09, 0.16, 0.075), 0.03, "carbon", name)
        M.box(m.mb, (head[0] - 0.046, head[1], head[2]), (0.004, 0.14, 0.06), "mirror", name)
        # flush handle recess
        M.box(m.mb, (-0.35, side * (W(-0.35) - 0.001), 0.66), (0.14, 0.012, 0.022), "trim_black", name)
    m.node("frunk", None, (FR_X0, 0.0, body.zt(FR_X0)), m.revolute((0, -1, 0), 0, 55, "front lid opening angle"))
    m.node("engine_cover", None, (EC_X1, 0.0, body.zt(EC_X1) + 0.01),
           m.revolute((0, 1, 0), 0, 50, "engine cover opening angle (hinged at the cabin, rear edge lifts)"))

    mb = m.mb
    # ---- front splitter (carbon, aero region) and active flaps
    m.node("splitter", None, (1.95, 0.0, 0.09), aero={
        "kind": "splitter", "note": "front splitter plate: ground-effect region under the nose"})
    spl = [(1.70, -0.84), (2.22, -0.84), (2.32, -0.70), (2.34, 0.0), (2.32, 0.70), (2.22, 0.84), (1.70, 0.84)]
    M.extrude(mb, [(a, b) for (a, b) in spl], 0.018, "carbon", "splitter", xf=translate((0, 0, 0.09)))
    for s in (1, -1):  # splitter end fences
        slab_xz(mb, [(1.74, 0.08), (2.21, 0.08), (2.21, 0.15), (1.92, 0.21), (1.74, 0.21)], s * 0.85, 0.012,
                "carbon", "splitter")
    for s, key in ((1, "L"), (-1, "R")):
        name = f"flap_front_{key}"
        le = (FLAP_LE_X, s * FLAP_Y, FLAP_Z)
        m.node(name, "splitter", le, m.revolute((0, 1, 0), -10, 20,
                                                 "front aero flap pitch about its leading edge, + = trailing edge up"))
        wing_mesh(mb, name, le, FLAP_CHORD, FLAP_SPAN, math.radians(10), 0.12, 0.06, "carbon", count=24)

    # ---- diffuser fins (carbon, aero region; ramp faces were tagged above)
    m.node("diffuser", None, (-2.05, 0.0, 0.10), aero={
        "kind": "diffuser", "note": "rear diffuser: underbody ramp from x=-1.75 (z=0.105) to the rear (z=0.33)"})
    zb = body.zb
    for yy in (-0.62, -0.36, -0.12, 0.12, 0.36, 0.62):
        xs = [-1.78 - 0.05 * k for k in range(12)]
        top = [(x, zb(x) + 0.01) for x in xs]
        poly = [(xs[0], 0.105)] + top[1:] + [(xs[-1], 0.19)]
        slab_xz(mb, poly, yy, 0.012, "carbon", "diffuser")

    # ---- rear wing: struts on wing_lift, main plane + end plates on wing_flap
    le = WING_LE
    m.node("wing_lift", None, le, m.prismatic((0, 0, 1), 0.0, WING_LIFT_MAX, "rear wing raise on its struts, + = up"))
    m.node("wing_flap", "wing_lift", le,
           m.revolute((0, 1, 0), WING_FLAP_RANGE[0], WING_FLAP_RANGE[1],
                      "rear wing pitch about the leading edge, + = trailing edge up (airbrake)"),
           aero={"kind": "wing", "chord": WING_CHORD, "span": WING_SPAN, "pivot": list(le),
                 "rest_pitch_deg": math.degrees(WING_PITCH), "pitch_range_deg": list(WING_FLAP_RANGE),
                 "profile": "inverted NACA 4-digit, 12 % thick, 6 % camber"})
    wing_mesh(mb, "wing_flap", le, WING_CHORD, WING_SPAN, WING_PITCH, 0.12, 0.06, "carbon", count=40)
    for s in (1, -1):
        y = s * (WING_SPAN / 2 + 0.006)
        x0, z0 = le[0] + 0.05, le[2]
        plate = [(x0, z0 - 0.035), (x0 - 0.36, z0 - 0.035 + 0.0), (x0 - 0.38, z0 + 0.045), (x0 - 0.10, z0 + 0.035),
                 (x0, z0 + 0.02)]
        slab_xz(mb, plate, y, 0.012, "paint", "wing_flap")
        # strut: plate from inside the deck up to the wing's leading edge, where the pivot is,
        # so the pitching main plane stays attached to it
        sy = s * 0.36
        sx = le[0] - 0.03
        strut = [(sx + 0.05, le[2] - 0.005), (sx - 0.06, le[2] - 0.01), (sx - 0.07, 0.62), (sx + 0.06, 0.62)]
        slab_xz(mb, strut, sy, 0.016, "trim_black", "wing_lift")
        M.cylinder(mb, (le[0], sy - 0.02, le[2]), (le[0], sy + 0.02, le[2]), 0.018, "chrome", "wing_lift",
                   count=12)
    # deck slots the struts slide in (static)
    for s in (1, -1):
        mx = le[0] - 0.035
        M.box(mb, (mx, s * 0.36, surface_z(body, mx, 0.36) + 0.002), (0.16, 0.04, 0.006), "gap")

    # Explicit exterior selection before hidden cabin/engine/tub details are emitted.
    m.flow_ranges=[]
    for key,p in mb.prims.items():
        if key in m.flow_outer:
            m.flow_ranges.append((key,0,m.flow_outer[key]))
        start=m.flow_after_shell.get(key,0)
        if len(p.idx)>start: m.flow_ranges.append((key,start,len(p.idx)-start))

    # Fitted enclosed cockpit; its door cards move with the butterfly doors.
    from .hyper_interior import build_cabin
    build_cabin(m, body)

    # ---- frunk tub (luggage well between the front wheel wells), walls kept under the lid
    TX0, TX1, TY, TZ = 1.08, 1.98, 0.48, 0.24
    M.box(mb, ((TX0 + TX1) / 2, 0, TZ), (TX1 - TX0, 2 * TY, 0.02), "interior")
    xs = [TX0 + (TX1 - TX0) * k / 12 for k in range(13)]
    for yy in (TY, -TY):
        top = [(x, surface_z(body, x, TY + 0.02) - 0.03) for x in xs]
        slab_xz(mb, [(TX0, TZ), (TX1, TZ)] + top[::-1], yy, 0.02, "interior")
    for xx in (TX0, TX1):
        zt_ = surface_z(body, xx, 0.0) - 0.03
        M.box(mb, (xx, 0, (TZ + zt_) / 2), (0.02, 2 * TY, zt_ - TZ), "interior")

    # ---- engine bay: V engine under the glass cover, intake plenum, gearbox
    M.rounded_box(mb, (-1.50, 0, 0.52), (0.62, 0.52, 0.34), 0.05, "engine")
    for s in (1, -1):
        M.rounded_box(mb, (-1.50, s * 0.17, 0.74), (0.60, 0.20, 0.10), 0.03, "engine_red",
                      xf=compose(translate((-1.50, s * 0.17, 0.74)), rot_axis((1, 0, 0), s * 0.35),
                                 translate((1.50, -s * 0.17, -0.74))))
    M.rounded_box(mb, (-1.50, 0, 0.80), (0.50, 0.14, 0.08), 0.03, "carbon")
    M.rounded_box(mb, (-2.0, 0, 0.42), (0.40, 0.40, 0.26), 0.05, "engine")      # gearbox
    M.box(mb, (-1.60, 0, 0.36), (1.2, 1.5, 0.02), "underbody")                   # bay floor

    # ---- exhaust: twin centre outlets through the rear mesh
    flow_before_exhaust={key:len(p.idx) for key,p in mb.prims.items()}
    ex_pts = []
    for s in (1, -1):
        p0, p1 = (-2.05, s * 0.45, 0.555), (X_REAR - 0.012, s * 0.45, 0.555)
        # Annular tube wall: open bore with inner wall and rolled-end returns.
        # No flat disk at the outlet. The existing recessed soot back remains.
        from ..vec import frame_z_to
        xf=frame_z_to(tuple(p1[i]-p0[i] for i in range(3)),p0)
        L=math.dist(p0,p1)
        M.lathe(mb,[(.052,0),(.052,L),(.047,L),(.047,0)],"exhaust_tip",axis_xf=xf,count=20,closed_profile=True)
        M.cylinder(mb, (p1[0] + 0.02, p1[1], p1[2]), (p1[0] + 0.021, p1[1], p1[2]), 0.049, "exhaust_soot",
                   count=20)
        ex_pts.append(p1)

    for key,p in mb.prims.items():
        start=flow_before_exhaust.get(key,0)
        if len(p.idx)>start: m.flow_ranges.append((key,start,len(p.idx)-start))
    m.flow_selection={
        "format":"physics_sim.visual_flow_selection/1",
        "frame":"ISO x forward,y left,z up; rig ground origin, not chassis COM",
        "status":"visual_surface_selection_not_fluid_solid",
        "cfd_ready":False,
        "ranges":[{"node":key[0] or m.id,"material":key[1],"first_triangle":start//3,"triangle_count":count//3}
                  for key,start,count in m.flow_ranges if count],
        "excluded":"Inner shell returns, headliner, cabin upholstery/dashboard/seats, frunk tub, engine and bay details.",
        "limitations":"Overlapping visual components and occluded faces remain; not a boolean union. Cooling grilles are backed visual facades; no resolved cooling flow. Exhaust bore has a recessed soot termination. Fluid-solid preparation/pressure-region choices required before CFD."
    }

    # ---- sockets
    m.socket("socket_driver_eye", None, (-0.28, 0.33, 0.93), "driver eye point (camera)")
    m.socket("socket_tow_front", None, (2.27, 0.30, 0.24), "front towing eye")
    m.socket("socket_tow_rear", None, (X_REAR + 0.02, -0.45, 0.40), "rear towing eye")
    for s, p in zip(("left", "right"), ex_pts):
        m.socket(f"socket_exhaust_{s}", None, p, f"exhaust outlet ({s}), flow along -x")

    # ---- notes for the physics side
    m.notes += [
        "Generic mid-engine hypercar; dimensions are class-typical, not a specific production car.",
        "Different wheel radii: front 0.347 m, rear 0.364 m; at rest the wheel centres are at z = r per axle.",
        f"Rear wing: node wing_flap = main plane, chord {WING_CHORD} m, span {WING_SPAN} m, inverted NACA 4-digit "
        f"(12 % thick, 6 % camber). Pivot = leading edge {list(WING_LE)} at rest (moves with wing_lift). "
        f"Rest chord angle {math.degrees(WING_PITCH):.0f} deg trailing edge up (downforce). wing_flap range "
        f"{WING_FLAP_RANGE[0]:.0f}..+{WING_FLAP_RANGE[1]:.0f} deg about the rest angle (-8 = flat/low drag, "
        f"+55 = airbrake). wing_lift raises the whole wing 0..{WING_LIFT_MAX} m along +z.",
        f"Front flaps flap_front_L/R: chord {FLAP_CHORD} m, span {FLAP_SPAN} m each, leading edge x={FLAP_LE_X}, "
        f"|y|={FLAP_Y}, z={FLAP_Z}; rest chord angle 10 deg trailing edge up; range -10..+20 deg.",
        "Aero regions: node 'splitter' = front splitter plate (x 1.70..2.38, |y|<0.85, z=0.09); node 'diffuser' = "
        "rear diffuser ramp + 6 fins (underbody from x=-1.75, z=0.105 rising to z=0.33 at x=-2.35). "
        "Flat underbody at z=0.10 between them. Frontal area approx. 2.0 m^2.",
        "Butterfly doors: tilted hinge axis at the A-pillar base; door + mirror open together.",
    ]
    from .hyper_shell import clean_degenerate
    if clean_degenerate(mb):
        raise ValueError("late triangle cleanup would invalidate exterior selection ranges")
    m.notes.append("Visual material shells have stitched inner returns; intentional intake/exhaust mouths and wheel arches are retained. Not a boolean-unioned CFD solid.")
    m.notes.append("Design A: sculpted silver body, concave door coves, enlarged side scoops, swept LED blades, dark canopy.")
    return m
