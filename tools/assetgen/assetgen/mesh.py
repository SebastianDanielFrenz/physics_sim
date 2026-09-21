"""Mesh building: parametric grids, polygons and the primitives built on them.

A MeshBuilder collects triangles into primitives keyed by (part, material).
Geometry is authored in model-root coordinates (ISO 8855); the scene later
hands each part to a node and re-expresses it relative to the node's pivot.
UVs are arc length in metres; each material's tile size scales them at export.
"""
import math
from .vec import (add, sub, mul, dot, cross, norm, length, dist, xform_point, xform_dir,
                  frame_z_to, is_rigid, IDENTITY, compose, translate)


class Prim:
    __slots__ = ("pos", "nrm", "uv", "idx")

    def __init__(self):
        self.pos, self.nrm, self.uv, self.idx = [], [], [], []


class MeshBuilder:
    def __init__(self):
        self.prims = {}  # (part, material) -> Prim

    def prim(self, part, material):
        key = (part, material)
        p = self.prims.get(key)
        if p is None:
            p = self.prims[key] = Prim()
        return p

    def parts(self):
        return sorted({k[0] for k in self.prims})

    # ------------------------------------------------------------------ grid
    def grid(self, P, material, part="", closed_u=False, closed_v=False, classify=None,
             flip=False, uv=None):
        """Quad grid P[i][j]. Outward normal is dPu x dPv (use flip otherwise).

        classify(i, j, center, normal) -> material | (part, material) | None (drop face).
        uv(i, j) -> (u, v) overrides the arc-length default.
        Returns emission records for orient_outward().
        """
        nu, nv = len(P), len(P[0])
        iu = nu if closed_u else nu - 1
        jv = nv if closed_v else nv - 1

        def at(i, j):
            return P[i % nu][j % nv]

        # Arc-length coordinates, including the wrapped seam row/column.
        U = [[0.0] * (nv + 1) for _ in range(nu + 1)]
        V = [[0.0] * (nv + 1) for _ in range(nu + 1)]
        for j in range(nv + 1):
            for i in range(1, nu + 1):
                U[i][j] = U[i - 1][j] + dist(at(i - 1, j), at(i, j))
        for i in range(nu + 1):
            for j in range(1, nv + 1):
                V[i][j] = V[i][j - 1] + dist(at(i, j - 1), at(i, j))

        # Classify every face first: normals are smoothed only among faces of the same
        # (part, material), so material/part borders (arch lips, lamp edges, panel gaps,
        # glass frames) are hard edges instead of being shaded as one smooth surface.
        faces = {}
        for i in range(iu):
            for j in range(jv):
                a, b, c, d = at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1)
                fn = add(cross(sub(b, a), sub(c, a)), cross(sub(c, a), sub(d, a)))
                if flip:
                    fn = mul(fn, -1)
                key = (part, material)
                if classify is not None:
                    center = mul(add(add(a, c), add(b, d)), 0.25)
                    r = classify(i, j, center, norm(fn))
                    if r is None:
                        continue
                    key = r if isinstance(r, tuple) else (part, r)
                faces[(i, j)] = (key, fn)

        normals = {}  # key -> {(i % nu, j % nv): summed normal}
        for (i, j), (key, fn) in faces.items():
            acc = normals.setdefault(key, {})
            for (ii, jj) in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)):
                k = (ii % nu, jj % nv)
                acc[k] = add(acc.get(k, (0.0, 0.0, 0.0)), fn)

        records = {}
        vmaps = {}

        def vert(prim_key, prim, i, j):
            vm = vmaps.setdefault(prim_key, {})
            k = (i, j)
            idx = vm.get(k)
            if idx is None:
                idx = len(prim.pos)
                vm[k] = idx
                prim.pos.append(at(i, j))
                n = norm(normals[prim_key][(i % nu, j % nv)])
                prim.nrm.append(n if n != (0.0, 0.0, 0.0) else (0.0, 0.0, 1.0))
                prim.uv.append(uv(i, j) if uv else (U[i][j], V[i][j]))
            return idx

        for i in range(iu):
            for j in range(jv):
                f = faces.get((i, j))
                if f is None:
                    continue
                key = f[0]
                prim = self.prim(*key)
                if key not in records:
                    records[key] = [prim, len(prim.idx), None, len(prim.pos), None]
                va = vert(key, prim, i, j)
                vb = vert(key, prim, i + 1, j)
                vc = vert(key, prim, i + 1, j + 1)
                vd = vert(key, prim, i, j + 1)
                if flip:
                    prim.idx += (va, vc, vb, va, vd, vc)
                else:
                    prim.idx += (va, vb, vc, va, vc, vd)
        out = []
        for key, r in records.items():
            r[2] = len(r[0].idx)
            r[4] = len(r[0].pos)
            out.append(tuple(r))
        return out

    # --------------------------------------------------------------- polygon
    def polygon(self, pts, material, part="", want_normal=None):
        """Flat planar polygon (any simple shape), ear-clipped. Returns records."""
        n = newell(pts)
        if want_normal is not None and dot(n, want_normal) < 0:
            pts = list(reversed(pts))
            n = mul(n, -1)
        n = norm(n)
        if n == (0.0, 0.0, 0.0):
            return []
        # Planar basis for ear clipping and UVs.
        ref = (0.0, 0.0, 1.0) if abs(n[2]) < 0.9 else (1.0, 0.0, 0.0)
        ex = norm(cross(ref, n))
        ey = cross(n, ex)
        o = pts[0]
        p2 = [(dot(sub(p, o), ex), dot(sub(p, o), ey)) for p in pts]
        tris = ear_clip(p2)
        prim = self.prim(part, material)
        rec = [prim, len(prim.idx), None, len(prim.pos), None]
        base = len(prim.pos)
        for p, q in zip(pts, p2):
            prim.pos.append(p)
            prim.nrm.append(n)
            prim.uv.append(q)
        for (a, b, c) in tris:
            prim.idx += (base + a, base + b, base + c)
        rec[2], rec[4] = len(prim.idx), len(prim.pos)
        return [tuple(rec)]

    def orient_outward(self, records):
        """Flip the emitted triangles of a closed piece if its signed volume is negative."""
        vol = 0.0
        for prim, i0, i1, _, _ in records:
            P = prim.pos
            for k in range(i0, i1, 3):
                a, b, c = P[prim.idx[k]], P[prim.idx[k + 1]], P[prim.idx[k + 2]]
                vol += dot(a, cross(b, c))
        if vol >= 0:
            return records
        for prim, i0, i1, v0, v1 in records:
            for k in range(i0, i1, 3):
                prim.idx[k + 1], prim.idx[k + 2] = prim.idx[k + 2], prim.idx[k + 1]
            for v in range(v0, v1):
                prim.nrm[v] = mul(prim.nrm[v], -1)
        return records

    def merge(self, other, xf=IDENTITY, part_map=None):
        """Append another builder, transformed (used for instanced sub-assemblies)."""
        mirror = not is_rigid(xf)
        for (prt, mat), p in other.prims.items():
            prt2 = part_map(prt) if part_map else prt
            q = self.prim(prt2, mat)
            base = len(q.pos)
            q.pos += [xform_point(xf, v) for v in p.pos]
            q.nrm += [norm(xform_dir(xf, v)) for v in p.nrm]
            q.uv += p.uv
            if mirror:
                for k in range(0, len(p.idx), 3):
                    q.idx += (base + p.idx[k], base + p.idx[k + 2], base + p.idx[k + 1])
            else:
                q.idx += [base + i for i in p.idx]

    def bounds(self, part):
        lo = [1e9] * 3
        hi = [-1e9] * 3
        for (prt, _), p in self.prims.items():
            if prt != part:
                continue
            for v in p.pos:
                for k in range(3):
                    lo[k] = min(lo[k], v[k])
                    hi[k] = max(hi[k], v[k])
        return (tuple(lo), tuple(hi)) if lo[0] < 1e9 else None

    def triangle_count(self):
        return sum(len(p.idx) // 3 for p in self.prims.values())


def newell(pts):
    nx = ny = nz = 0.0
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return (nx, ny, nz)


def ear_clip(p):
    n = len(p)
    if n < 3:
        return []
    area = sum(p[i][0] * p[(i + 1) % n][1] - p[(i + 1) % n][0] * p[i][1] for i in range(n))
    idx = list(range(n)) if area > 0 else list(range(n))[::-1]

    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(pt, a, b, c):
        return cr(a, b, pt) >= 0 and cr(b, c, pt) >= 0 and cr(c, a, pt) >= 0

    tris = []
    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        m = len(idx)
        for k in range(m):
            ia, ib, ic = idx[k - 1], idx[k], idx[(k + 1) % m]
            a, b, c = p[ia], p[ib], p[ic]
            if cr(a, b, c) <= 1e-14:
                continue
            if any(inside(p[j], a, b, c) for j in idx if j not in (ia, ib, ic)):
                continue
            tris.append((ia, ib, ic) if area > 0 else (ia, ic, ib))
            idx.pop(k)
            break
        else:
            break  # degenerate remainder: fan it
    if len(idx) >= 3:
        for k in range(1, len(idx) - 1):
            t = (idx[0], idx[k], idx[k + 1])
            tris.append(t if area > 0 else (t[0], t[2], t[1]))
    return tris


# ======================================================================
# 2D section generators (return lists of (a, b) points, counter-clockwise)
# ======================================================================

def superellipse(a, b, n=4.0, count=48, ca=0.0, cb=0.0):
    pts = []
    e = 2.0 / n
    for k in range(count):
        t = 2 * math.pi * k / count
        c, s = math.cos(t), math.sin(t)
        pts.append((ca + a * math.copysign(abs(c) ** e, c), cb + b * math.copysign(abs(s) ** e, s)))
    return pts


def rrect(w, h, r, seg=4, ca=0.0, cb=0.0):
    """Rounded rectangle, full width w, height h, corner radius r."""
    r = min(r, w / 2 - 1e-4, h / 2 - 1e-4)
    hw, hh = w / 2 - r, h / 2 - r
    pts = []
    for q, (cx, cy) in enumerate(((hw, hh), (-hw, hh), (-hw, -hh), (hw, -hh))):
        for k in range(seg + 1):
            t = (q + k / seg) * math.pi / 2
            pts.append((ca + cx + r * math.cos(t), cb + cy + r * math.sin(t)))
    return pts


def circle(r, count=24, ca=0.0, cb=0.0):
    return [(ca + r * math.cos(2 * math.pi * k / count), cb + r * math.sin(2 * math.pi * k / count))
            for k in range(count)]


def naca(chord, thickness, camber=0.0, camber_pos=0.4, count=40):
    """NACA 4-digit airfoil loop, leading edge at a=0, chord along +a, thickness along b.
    Counter-clockwise, starting at the trailing edge (upper surface first)."""
    def yt(x):
        return 5 * thickness * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x * x
                                + 0.2843 * x ** 3 - 0.1036 * x ** 4)

    def yc(x):
        m, p = camber, camber_pos
        if m == 0:
            return 0.0, 0.0
        if x < p:
            return m / p ** 2 * (2 * p * x - x * x), 2 * m / p ** 2 * (p - x)
        return m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x * x), 2 * m / (1 - p) ** 2 * (p - x)

    half = count // 2
    xs = [0.5 * (1 - math.cos(math.pi * k / half)) for k in range(half + 1)]
    upper, lower = [], []
    for x in xs:
        c, dc = yc(x)
        th = math.atan(dc)
        t = yt(x)
        upper.append(((x - t * math.sin(th)) * chord, (c + t * math.cos(th)) * chord))
        lower.append(((x + t * math.sin(th)) * chord, (c - t * math.cos(th)) * chord))
    loop = list(reversed(upper)) + lower[1:-1]
    return loop


# ======================================================================
# 3D primitives
# ======================================================================

def place2d(sec, origin, ea, eb):
    """Map 2D section points into 3D using origin + a*ea + b*eb."""
    return [add(origin, add(mul(ea, a), mul(eb, b))) for (a, b) in sec]


def loft(mb, sections, material, part="", cap_start=True, cap_end=True, classify=None,
         cap_material=None, auto_orient=True):
    """Skin a list of closed 3D loops (same point count). Caps are flat polygons."""
    d = sub(centroid(sections[-1]), centroid(sections[0]))
    if dot(newell(sections[0]), d) > 0:
        sections = [list(reversed(s)) for s in sections]
    recs = mb.grid(sections, material, part, closed_v=True, classify=classify)
    cm = cap_material or material
    if cap_start:
        recs += mb.polygon(sections[0], cm, part, want_normal=mul(d, -1))
    if cap_end:
        recs += mb.polygon(sections[-1], cm, part, want_normal=d)
    if auto_orient and cap_start and cap_end:
        mb.orient_outward(recs)
    return recs


def centroid(pts):
    s = (0.0, 0.0, 0.0)
    for p in pts:
        s = add(s, p)
    return mul(s, 1.0 / len(pts))


def box(mb, center, size, material, part="", xf=IDENTITY, face_materials=None):
    """Axis-aligned (then xf-transformed) box with flat faces."""
    hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
    cx, cy, cz = center
    faces = {
        "+x": [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)],
        "-x": [(-hx, hy, -hz), (-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz)],
        "+y": [(hx, hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz)],
        "-y": [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)],
        "+z": [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)],
        "-z": [(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)],
    }
    recs = []
    for key, quad in faces.items():
        m = (face_materials or {}).get(key, material)
        if m is None:
            continue
        pts = [xform_point(xf, (cx + x, cy + y, cz + z)) for (x, y, z) in quad]
        recs += mb.polygon(pts, m, part)
    return recs


def rounded_box(mb, center, size, r, material, part="", xf=IDENTITY, seg=3, classify=None):
    """Box with all edges rounded (radius r): loft along x of rounded rectangles."""
    sx, sy, sz = size
    r = min(r, sx / 2 - 1e-4, sy / 2 - 1e-4, sz / 2 - 1e-4)
    secs = []
    stations = []
    for k in range(seg + 1):  # rear rounding
        t = (math.pi / 2) * (1 - k / seg)
        stations.append((-sx / 2 + r - r * math.sin(t), r - r * math.cos(t)))
    for k in range(seg + 1):  # front rounding
        t = (math.pi / 2) * (k / seg)
        stations.append((sx / 2 - r + r * math.sin(t), r - r * math.cos(t)))
    for x, inset in stations:
        w, h = max(sy - 2 * inset, 1e-3), max(sz - 2 * inset, 1e-3)
        rr = max(r - inset, 1e-4)
        sec = rrect(w, h, rr, seg)
        pts = [xform_point(xf, (center[0] + x, center[1] + a, center[2] + b)) for (a, b) in sec]
        secs.append(pts)
    return loft(mb, secs, material, part, classify=classify)


def tube(mb, path, radius, material, part="", count=16, closed=False, caps=True, classify=None):
    """Circular tube along a polyline. radius may be a number or a list per point."""
    n = len(path)
    radii = radius if isinstance(radius, (list, tuple)) else [radius] * n
    tangents = []
    for i in range(n):
        if closed:
            t = sub(path[(i + 1) % n], path[i - 1])
        elif i == 0:
            t = sub(path[1], path[0])
        elif i == n - 1:
            t = sub(path[-1], path[-2])
        else:
            t = add(norm(sub(path[i], path[i - 1])), norm(sub(path[i + 1], path[i])))
        tangents.append(norm(t))
    # Parallel-transport frames.
    ref = (0.0, 0.0, 1.0) if abs(tangents[0][2]) < 0.9 else (1.0, 0.0, 0.0)
    ex = norm(cross(ref, tangents[0]))
    secs = []
    for i in range(n):
        t = tangents[i]
        ex = norm(sub(ex, mul(t, dot(ex, t))))
        ey = cross(t, ex)
        secs.append([add(path[i], add(mul(ex, radii[i] * math.cos(2 * math.pi * k / count)),
                                      mul(ey, radii[i] * math.sin(2 * math.pi * k / count))))
                     for k in range(count)])
    if closed:
        recs = mb.grid(secs, material, part, closed_u=True, closed_v=True, classify=classify)
        return mb.orient_outward(recs)  # a closed tube is a torus: signed volume decides
    return loft(mb, secs, material, part, cap_start=caps, cap_end=caps, classify=classify)


def _flip_records(recs):
    for prim, i0, i1, v0, v1 in recs:
        for k in range(i0, i1, 3):
            prim.idx[k + 1], prim.idx[k + 2] = prim.idx[k + 2], prim.idx[k + 1]
        for v in range(v0, v1):
            prim.nrm[v] = mul(prim.nrm[v], -1)
    return recs


def cylinder(mb, p0, p1, r, material, part="", count=24, caps=True, cap_material=None):
    axis = sub(p1, p0)
    f = frame_z_to(axis)
    ex = (f[0][0], f[1][0], f[2][0])
    ey = (f[0][1], f[1][1], f[2][1])
    secs = []
    for p in (p0, p1):
        secs.append([add(p, add(mul(ex, r * math.cos(2 * math.pi * k / count)),
                                mul(ey, r * math.sin(2 * math.pi * k / count)))) for k in range(count)])
    return loft(mb, secs, material, part, cap_start=caps, cap_end=caps, cap_material=cap_material)


def lathe(mb, profile, material, part="", axis_xf=IDENTITY, count=32, classify=None, closed_profile=False,
          orient=None, uv=None):
    """Revolve profile [(radius, axial)] about local z of axis_xf.
    orient: None -> auto (closed profiles only), True -> flip, False -> as is."""
    rows = []
    for (r, a) in profile:
        rows.append([xform_point(axis_xf, (r * math.cos(2 * math.pi * k / count),
                                           r * math.sin(2 * math.pi * k / count), a))
                     for k in range(count)])
    recs = mb.grid(rows, material, part, closed_u=closed_profile, closed_v=True, classify=classify,
                   flip=bool(orient), uv=uv)
    if orient is None:
        mb.orient_outward(recs)
    return recs


def extrude(mb, poly, depth, material, part="", xf=IDENTITY, smooth=False, cap_material=None,
            side_material=None):
    """Extrude a 2D loop (local a->x, b->y) along local z, centred on z=0."""
    # Side quads face outward only for a counter-clockwise loop under a non-mirroring xf;
    # normalise the loop so sides and caps agree (else orient_outward flips a mixed set).
    area = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
               for i in range(len(poly)))
    if (area < 0) == is_rigid(xf):
        poly = list(reversed(poly))
    s0 = [xform_point(xf, (a, b, -depth / 2)) for (a, b) in poly]
    s1 = [xform_point(xf, (a, b, depth / 2)) for (a, b) in poly]
    sm = side_material or material
    if smooth:
        return loft(mb, [s0, s1], sm, part, cap_material=cap_material or material)
    recs = []
    n = len(poly)
    for k in range(n):
        q = [s0[k], s0[(k + 1) % n], s1[(k + 1) % n], s1[k]]
        recs += mb.polygon(q, sm, part)
    d = sub(centroid(s1), centroid(s0))
    recs += mb.polygon(s0, cap_material or material, part, want_normal=mul(d, -1))
    recs += mb.polygon(s1, cap_material or material, part, want_normal=d)
    mb.orient_outward(recs)
    return recs


def ellipsoid(mb, center, radii, material, part="", xf=IDENTITY, count=24, rings=12):
    prof = [(math.sin(math.pi * k / rings), -math.cos(math.pi * k / rings)) for k in range(rings + 1)]
    m = compose(xf, translate(center), _diag(radii))
    rows = [[xform_point(m, (r * math.cos(2 * math.pi * j / count), r * math.sin(2 * math.pi * j / count), a))
             for j in range(count)] for (r, a) in prof]
    recs = mb.grid(rows, material, part, closed_v=True)
    return mb.orient_outward(recs)


def _diag(r):
    return ((r[0], 0.0, 0.0, 0.0), (0.0, r[1], 0.0, 0.0), (0.0, 0.0, r[2], 0.0))


def arc_path(center, radius, a0, a1, count, plane=("x", "z"), fixed=0.0):
    """Points on a circular arc in an axis plane: plane=("x","z") -> x=cx+r cos, z=cz+r sin, y=fixed."""
    idx = {"x": 0, "y": 1, "z": 2}
    ia, ib = idx[plane[0]], idx[plane[1]]
    ic = 3 - ia - ib
    pts = []
    for k in range(count + 1):
        t = a0 + (a1 - a0) * k / count
        p = [0.0, 0.0, 0.0]
        p[ia] = center[ia] + radius * math.cos(t)
        p[ib] = center[ib] + radius * math.sin(t)
        p[ic] = fixed if fixed is not None else center[ic]
        pts.append(tuple(p))
    return pts
