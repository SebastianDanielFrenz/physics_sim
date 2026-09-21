"""Parametric closed-body car shell: a lofted lower body plus a lofted greenhouse.

Lower body: superellipse sections along x (half-width W(x), bottom zb(x),
top zt(x), exponent n(x)), rounded at both ends, with wheel arches carved in
by pushing side points inward inside a circle around each wheel.
Greenhouse: rounded trapezoid sections from the belt line up to the roof;
its height goes to zero at the windshield base and the rear, which makes the
windshield and rear window.

Faces are classified by position and normal, so lamps, grilles, glass,
pillars and opening panels (doors, hood, hatch) all come out of the same
continuous surface: a panel is the set of faces inside its region, moved to
its own node, and the surrounding band becomes a dark shut line.
"""
import math

from .vec import Curve, add, sub, mul, norm, smoothstep, dist
from . import mesh as M


class Panel:
    """An opening panel. inside(c, n) -> bool selects its faces (c = face centre,
    n = face normal); border(c, n) -> bool marks the shut-line band left on the body."""

    def __init__(self, part, inside, border, inner_skin=True):
        self.part, self.inside, self.border, self.inner_skin = part, inside, border, inner_skin


class CarBody:
    def __init__(self, model, *, x_rear, x_front, W, zb, zt, exp, end_r=(0.12, 0.12), wheels=(),
                 arch_clear=0.045, arch_depth_in=0.02, step=0.022, around=128, arch_band=0.045,
                 green=None, paint="paint", green_step=None, green_around=64):
        """W, zb, zt, exp: Curve or list of (x, value) keys.
        wheels: list of dicts {x, z, r, y_out, y_in} (arch centre, tyre radius, outer/inner tyre face |y|).
        green: dict x0, x1, zroof (keys), wb (keys), wr (keys), r (corner radius), crown (roof arch height).
        """
        c = lambda k: k if isinstance(k, Curve) else Curve(k)
        self.model = model
        self.x_rear, self.x_front = x_rear, x_front
        self.W, self.zb, self.zt, self.exp = c(W), c(zb), c(zt), c(exp)
        self.end_r = end_r
        self.wheels = list(wheels)
        self.arch_clear = arch_clear
        self.arch_depth_in = arch_depth_in
        self.arch_band = arch_band
        self.step = step
        self.around = around
        self.paint = paint
        self.panels = []
        self.green = None
        if green:
            g = dict(green)
            g["zroof"], g["wb"], g["wr"] = c(g["zroof"]), c(g["wb"]), c(g["wr"])
            g.setdefault("r", 0.08)
            g.setdefault("crown", 0.03)
            self.green = g
        self.green_step = green_step or step
        self.green_around = green_around

    # ------------------------------------------------------------ queries
    def arch_hit(self, x, z, pad=0.0):
        """(push 0..1, wheel) for a point at (x, z) on the body side."""
        best, bw = 0.0, None
        for w in self.wheels:
            d = math.hypot(x - w["x"], z - w["z"])
            ra = w["r"] + self.arch_clear
            p = smoothstep(ra + 0.025 + pad, ra - 0.025 + pad, d)
            if p > best:
                best, bw = p, w
        return best, bw

    def in_arch(self, c, margin=0.012):
        """Face centre inside an arch circle and set in from the body side (lip + well wall)."""
        for w in self.wheels:
            if (math.hypot(c[0] - w["x"], c[2] - w["z"]) < w["r"] + self.arch_clear - 0.001
                    and w["y_in"] - self.arch_depth_in - 0.01 < abs(c[1]) < self.section_half_width(c[0]) - margin):
                return True  # lip or well wall; faces further inboard (e.g. a low lid) are not
        return False

    def section_half_width(self, x):
        return self.W(x) - self._inset(x)

    def _inset(self, x):
        rr, rf = self.end_r
        s_r = x - self.x_rear
        s_f = self.x_front - x
        ins = 0.0
        if s_r < rr:
            ins = max(ins, rr - math.sqrt(max(0.0, rr * rr - (rr - s_r) ** 2)))
        if s_f < rf:
            ins = max(ins, rf - math.sqrt(max(0.0, rf * rf - (rf - s_f) ** 2)))
        return ins

    def in_green_footprint(self, c, margin=0.02):
        g = self.green
        if not g:
            return False
        return (g["x0"] + margin < c[0] < g["x1"] - margin and abs(c[1]) < g["wb"](c[0]) - margin)

    def green_h(self, c):
        """Relative height of c inside the greenhouse section (0 at belt, 1 at roof)."""
        g = self.green
        z0 = self.zt(c[0])
        z1 = g["zroof"](c[0])
        return (c[2] - z0) / max(1e-4, z1 - z0)

    # ------------------------------------------------ surface projection
    def lower_params(self, x):
        ins = self._inset(x)
        W = max(self.W(x) - ins, 0.01)
        zb, zt = self.zb(x) + ins * 0.8, self.zt(x) - ins * 0.6
        if zt - zb < 0.02:
            zt = zb + 0.02
        return W, zb, zt, self.exp(x)

    def contains(self, x, y, z):
        if not (self.x_rear <= x <= self.x_front):
            return False
        W, zb, zt, n = self.lower_params(x)
        zc, H = (zt + zb) / 2, (zt - zb) / 2
        return (abs(y) / W) ** n + (abs(z - zc) / H) ** n <= 1.0

    def surface_x(self, y, z, end):
        """x of the lower-body surface seen from the front (end=+1) or rear (end=-1)."""
        x_out = self.x_front if end > 0 else self.x_rear
        if self.contains(x_out, y, z):
            return x_out
        x = x_out
        for _ in range(200):  # march inward in 1 cm steps, then bisect
            xn = x - end * 0.01
            if self.contains(xn, y, z):
                lo, hi = xn, x
                for _ in range(30):
                    mid = (lo + hi) / 2
                    if self.contains(mid, y, z):
                        lo = mid
                    else:
                        hi = mid
                return (lo + hi) / 2
            x = xn
        return x

    def surface_y(self, x, z):
        """|y| of the lower-body side at (x, z) (ignores arches)."""
        W, zb, zt, n = self.lower_params(x)
        zc, H = (zt + zb) / 2, (zt - zb) / 2
        t = min(1.0, abs(z - zc) / H)
        return W * (1 - t ** n) ** (1 / n)

    def patch(self, outline, mat, face, part="", offset=0.004, rings=5, skirt_mat=None, skirt=0.004):
        """Drape a flat outline onto the lower body: a lamp, grille, plate, badge...
        face: 'front'/'rear' (outline in (y, z)) or 'left'/'right' (outline in (x, z)).
        The patch stands `offset` proud of the paint; its edge is exactly the outline."""
        mb = self.model.mb

        def place(a, b, off):
            if face in ("front", "rear"):
                e = 1 if face == "front" else -1
                return (self.surface_x(a, b, e) + e * off, a, b)
            s = 1 if face == "left" else -1
            return (a, s * (self.surface_y(a, b) + off), b)

        want = {"front": (1, 0, 0), "rear": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0)}[face]
        o3 = [place(a, b, 0) for (a, b) in outline]
        if M.dot(M.newell(o3), want) < 0:
            outline = list(reversed(outline))
        ca = sum(p[0] for p in outline) / len(outline)
        cb = sum(p[1] for p in outline) / len(outline)
        rows = []
        for k in range(rings + 1):
            f = k / rings
            rows.append([place(ca + (a - ca) * f, cb + (b - cb) * f, offset) for (a, b) in outline])
        recs = mb.grid(rows, mat, part, closed_v=True)
        if skirt_mat is not None or skirt:
            base = [place(a, b, offset - skirt - 0.004) for (a, b) in outline]
            mb.grid([base, rows[-1]], skirt_mat or mat, part, closed_v=True, flip=True)
        return recs

    # ---------------------------------------------------------- sections
    def _lower_section(self, x):
        W = self.W(x)
        zb, zt = self.zb(x), self.zt(x)
        ins = self._inset(x)
        W, zb, zt = max(W - ins, 0.01), zb + ins * 0.8, zt - ins * 0.6
        if zt - zb < 0.02:
            zt = zb + 0.02
        n = self.exp(x)
        pts2 = M.superellipse(W, (zt - zb) / 2, n, 720, 0.0, (zt + zb) / 2)
        # rotate start to bottom centre (t = -pi/2 -> index 540)
        pts2 = pts2[540:] + pts2[:540]
        pts2 = resample_loop(pts2, self.around)
        out = []
        b = self.arch_band
        for (y, z) in pts2:
            px, pz = x, z
            for w in self.wheels:
                y_in = w["y_in"] - self.arch_depth_in
                if abs(y) <= y_in:
                    continue
                ra = w["r"] + self.arch_clear
                dx, dz = x - w["x"], z - w["z"]
                d = math.hypot(dx, dz)
                if d >= ra:
                    continue
                # Snap the vertices nearest the arch onto two concentric circles so the arch
                # edge is a clean circle whatever the grid spacing: outer ring = body side,
                # inner ring = start of the wheel-well wall; everything further in is the wall.
                ux, uz = (dx / d, dz / d) if d > 1e-9 else (0.0, 1.0)
                if d >= ra - b:
                    px, pz = w["x"] + ux * ra, w["z"] + uz * ra
                elif d >= ra - 2 * b:
                    px, pz = w["x"] + ux * (ra - 0.008), w["z"] + uz * (ra - 0.008)
                    y = math.copysign(y_in, y)
                else:
                    y = math.copysign(y_in, y)
                break
            out.append((px, y, pz))
        return out

    GREEN_SEGS = (("bottom", 3), ("side", 12), ("corner", 5), ("roof", 9))

    def _green_section(self, x):
        """Loop with a fixed number of points per segment (bottom, side, corner, roof), so
        segment borders (pillars, window frames) run along grid lines."""
        g = self.green
        zb = self.zt(x) - 0.04
        zr = max(g["zroof"](x), zb + 0.004)
        wb = g["wb"](x)
        wr = min(g["wr"](x), wb)
        h = zr - zb
        rc = max(0.002, min(g["r"], h * 0.45, wr * 0.45))
        crown = g["crown"] * min(1.0, h / 0.3)
        side_len = math.hypot(wb - wr, h)
        f = max(0.0, 1 - rc / max(side_len, 1e-6))
        p1 = (wb + (wr - wb) * f, zb + h * f)
        p2 = (wr - rc, zr)
        (_, nb), (_, ns), (_, nc), (_, nr) = self.GREEN_SEGS
        half = []
        for k in range(nb):
            half.append((wb * k / nb, zb))
        # side: first point is a flange tucked under the body's top surface out to near the
        # body side, so the joint between the body and the window wall can never open a slot
        # (clamped to stay 1.5 cm inside the body surface at that height: rounded shoulders)
        half.append((max(wb, self.surface_y(x, zb - 0.03) - 0.015), zb - 0.03))
        for k in range(ns - 1):
            t = k / (ns - 1)
            half.append((wb + (p1[0] - wb) * t, zb + (p1[1] - zb) * t))
        for k in range(nc):
            t = k / nc
            u = 1 - t
            half.append((u * u * p1[0] + 2 * u * t * wr + t * t * p2[0], u * u * p1[1] + 2 * u * t * zr + t * t * p2[1]))
        for k in range(nr + 1):
            yy = p2[0] * (1 - k / nr)
            half.append((yy, zr + crown * (1 - (yy / max(wr, 1e-3)) ** 2)))
        loop = half + [(-y, z) for (y, z) in reversed(half[1:-1])]
        return [(x, y, z) for (y, z) in loop]

    def green_segment(self, j, n_loop, reversed_):
        """(segment name, 0..1 position inside it) of greenhouse face column j."""
        if reversed_:
            j = (n_loop - 2 - j) % n_loop
        H = n_loop // 2
        q = j if j < H else n_loop - 1 - j
        for name, cnt in self.GREEN_SEGS:
            if q < cnt:
                return name, (q + 0.5) / cnt
            q -= cnt
        return "roof", 1.0

    # ------------------------------------------------------------- build
    def build(self, classify_lower=None, classify_green=None):
        """classify_*(c, n, body) -> material | (part, material) | None | Ellipsis (= default)."""
        xs = stations(self.x_rear, self.x_front, self.step, self.end_r)
        secs = [self._lower_section(x) for x in xs]
        paint = self.paint
        n_low = len(secs[0])
        d = sub(M.centroid(secs[-1]), M.centroid(secs[0]))
        lrev = M.dot(M.newell(secs[0]), d) > 0
        if lrev:
            secs = [list(reversed(sct)) for sct in secs]

        def cl(i, j, c, n):
            # ls: position around the section, 0 = bottom centre, 1 = top centre (both sides).
            # Constant along grid lines, so panel edges defined with it are crisp.
            jj = (n_low - 2 - j) % n_low if lrev else j
            H = n_low / 2
            q = jj + 0.5
            self.ls = (q if q < H else n_low - q) / H
            # open the body top under the cabin; the cut runs just INSIDE the window wall's
            # base width wb so the wall overlaps the remaining body top (no slot at the belt)
            if n[2] > 0.3 and self.in_green_footprint(c, margin=0.008):
                return None
            if classify_lower:
                r = classify_lower(c, n, self)
                if r is not Ellipsis:
                    return r
            return self.default_lower(c, n)

        M.loft(self.model.mb, secs, paint, "", classify=cl, cap_material=paint)

        if self.green:
            g = self.green
            gx = stations(g["x0"], g["x1"], self.green_step, (0.0, 0.0))
            gsecs = [self._green_section(x) for x in gx]
            n_loop = len(gsecs[0])
            d = sub(M.centroid(gsecs[-1]), M.centroid(gsecs[0]))
            rev = M.dot(M.newell(gsecs[0]), d) > 0
            if rev:  # orient here (loft would do it too) so the segment lookup knows about it
                gsecs = [list(reversed(sct)) for sct in gsecs]

            def cg(i, j, c, n):
                self.gseg, self.gpos = self.green_segment(j, n_loop, rev)
                if self.gseg == "bottom":
                    return None
                if classify_green:
                    r = classify_green(c, n, self)
                    if r is not Ellipsis:
                        return r
                return paint

            M.loft(self.model.mb, gsecs, paint, "", classify=cg, cap_start=False, cap_end=False,
                   auto_orient=False)
        self._inner_skins()

    def default_lower(self, c, n):
        if n[2] < -0.6:
            return "underbody"
        if self.in_arch(c):
            return "arch_liner"
        return self.panel_face(c, n, self.paint)

    def panel_face(self, c, n, mat):
        for p in self.panels:
            if p.inside(c, n):
                if p.border(c, n):
                    return "gap"
                return (p.part, mat)
        return mat

    def _inner_skins(self):
        mb = self.model.mb
        mats = self.model.materials
        for p in self.panels:
            if not p.inner_skin:
                continue
            # every paint on the panel (main paint, rusty variants, a replacement door's colour)
            for (part, mname), src in list(mb.prims.items()):
                if part != p.part or not (mname == self.paint or mname.startswith("paint")
                                          or getattr(mats.get(mname), "role", None) == "paint"):
                    continue
                self._skin(src, mb.prim(p.part, "interior_trim"))

    @staticmethod
    def _skin(src, dst):
        """Reversed copy of src, 3 cm inside it: the trim seen when a panel is open."""
        base = len(dst.pos)
        for v, nn in zip(src.pos, src.nrm):
            dst.pos.append(sub(v, mul(nn, 0.03)))
            dst.nrm.append(mul(nn, -1))
        dst.uv += src.uv
        for k in range(0, len(src.idx), 3):
            dst.idx += (base + src.idx[k], base + src.idx[k + 2], base + src.idx[k + 1])


def stations(x0, x1, step, end_r):
    """x stations: uniform, plus denser ones inside the end roundings."""
    xs = set()
    n = max(2, int(round((x1 - x0) / step)))
    for k in range(n + 1):
        xs.add(round(x0 + (x1 - x0) * k / n, 6))
    for (e, sgn, base) in ((end_r[0], 1, x0), (end_r[1], -1, x1)):
        if e > 0:
            for k in range(1, 7):
                t = k / 7 * math.pi / 2
                xs.add(round(base + sgn * e * (1 - math.cos(t)), 6))
    return sorted(xs)


def resample_loop(pts, count):
    """Resample a closed 2D polyline to `count` points evenly by arc length, keeping pts[0]."""
    n = len(pts)
    seg = [math.dist(pts[i], pts[(i + 1) % n]) for i in range(n)]
    total = sum(seg)
    if total < 1e-9:
        return [pts[0]] * count
    out = []
    i, acc = 0, 0.0
    for k in range(count):
        target = total * k / count
        while acc + seg[i] < target and i < n - 1:
            acc += seg[i]
            i += 1
        t = (target - acc) / seg[i] if seg[i] > 1e-12 else 0.0
        a, b = pts[i], pts[(i + 1) % n]
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out
