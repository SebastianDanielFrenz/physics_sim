"""Small geometry/material helpers shared by the farm machinery generators
(tractor, trailer_tipper). Private to this package; not part of the library API.
"""
import math

from .. import mesh as M
from ..vec import add, sub, mul, norm, cross, dot, length, lerp

# Drawbar pin / hitch eye height above flat ground when tractor and trailer are
# coupled (both modules repeat this value as their own named constant).
HITCH_HEIGHT = 0.50


def farm_materials(m, body=(0.07, 0.30, 0.33), chassis=(0.13, 0.13, 0.14), seed=31):
    """Paint set for farm machinery: body paint (clean and dirty), chassis paint, mud."""
    m.mat("paint", color=(1, 1, 1), tex=("paint", {"color": body, "rough": 0.3, "dirt": 0.12, "seed": seed}),
          tile=1.2, double=True, role="paint")
    m.mat("paint_dirty", color=(1, 1, 1), tex=("paint", {"color": body, "rough": 0.4, "dirt": 0.55,
                                                         "seed": seed + 1}),
          tile=2.0, double=True, role="paint")
    m.mat("chassis", color=(1, 1, 1), tex=("paint", {"color": chassis, "rough": 0.45, "dirt": 0.55,
                                                     "metallic": 0.2, "seed": seed + 2}), tile=1.0, double=True)
    m.mat("mud", color=(1, 1, 1), tex=("grime", {"seed": seed + 3}), tile=1.5)
    m.mat("steel", color=(0.42, 0.43, 0.44), metallic=0.9, roughness=0.45)
    m.mat("rod", color=(0.86, 0.87, 0.88), metallic=1.0, roughness=0.12)
    m.mat("hose", color=(0.04, 0.04, 0.04), roughness=0.6)
    m.mat("reflector_red", color=(0.6, 0.03, 0.03), roughness=0.1)
    m.mat("reflector_amber", color=(0.85, 0.45, 0.05), roughness=0.1)
    m.mat("reflector_white", color=(0.85, 0.85, 0.85), roughness=0.1)


def arc_shell(mb, part, mat, center, y0, y1, r, a0, a1, t=0.014, n=24, rr=None):
    """Curved sheet (mudguard / fender): a strip of the cylinder about y with inner
    radius r around center (x, z), from angle a0 to a1 (rad, measured from +x towards +z),
    spanning y0..y1, thickness t. Built as a loft of thin rounded rectangles."""
    cx, cz = center
    yc, w = (y0 + y1) / 2, abs(y1 - y0)
    sec = M.rrect(w, t, rr if rr is not None else t * 0.45, 2)
    secs = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        c, s = math.cos(a), math.sin(a)
        o = (cx + (r + t / 2) * c, yc, cz + (r + t / 2) * s)
        secs.append(M.place2d(sec, o, (0.0, 1.0, 0.0), (c, 0.0, s)))
    return M.loft(mb, secs, mat, part)


def arc_lip(mb, part, mat, center, y, r, a0, a1, depth=0.08, t=0.012, n=24):
    """Radial lip (flange) along a fender edge at y, from radius r outward by depth."""
    cx, cz = center
    sec = M.rrect(t, depth, t * 0.45, 2)
    secs = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        c, s = math.cos(a), math.sin(a)
        o = (cx + (r + depth / 2) * c, y, cz + (r + depth / 2) * s)
        secs.append(M.place2d(sec, o, (0.0, 1.0, 0.0), (c, 0.0, s)))
    return M.loft(mb, secs, mat, part)


def beam(mb, p0, p1, w, h, mat, part="", up=(0.0, 0.0, 1.0), r=None):
    """Rectangular bar from p0 to p1: width w across (perpendicular to up), height h along up."""
    d = norm(sub(p1, p0))
    s = cross(up, d)
    if length(s) < 1e-6:
        s = cross((1.0, 0.0, 0.0), d)
    s = norm(s)
    u = cross(d, s)
    sec = M.rrect(w, h, r if r is not None else min(w, h) * 0.2, 2)
    return M.loft(mb, [M.place2d(sec, p0, s, u), M.place2d(sec, p1, s, u)], mat, part)


def rod(mb, p0, p1, radius, mat, part="", count=12):
    return M.cylinder(mb, p0, p1, radius, mat, part, count=count)


def eye(mb, center, axis, r_out, r_in, width, mat, part="", count=16):
    """Ring (clevis eye / hitch eye) around `axis` through center."""
    f = frame_along(axis, center)
    prof = [(r_in, -width / 2), (r_out, -width / 2), (r_out, width / 2), (r_in, width / 2)]
    return M.lathe(mb, prof, mat, part, f, count=count, closed_profile=True)


def frame_along(axis, origin):
    from ..vec import frame_z_to
    return frame_z_to(axis, origin)


def lamp_box(mb, part, center, size, face, lens, housing="trim_black", depth_frac=0.25):
    """Lamp housing box with a lens slab on the face pointing along `face` (+-x / +-y / +-z)."""
    M.rounded_box(mb, center, size, min(size) * 0.2, housing, part)
    ax = {"x": 0, "y": 1, "z": 2}[face[1]]
    sg = 1.0 if face[0] == "+" else -1.0
    c = list(center)
    c[ax] += sg * size[ax] / 2
    sz = [s * 0.86 for s in size]
    sz[ax] = 0.012
    M.box(mb, tuple(c), tuple(sz), lens, part)


def tri_reflector(mb, part, center, size, mat, normal=(-1.0, 0.0, 0.0)):
    """Triangular reflector (trailer rear marking) facing `normal` (+-x only)."""
    x, y, z = center
    h = size * math.sqrt(3) / 2
    pts = [(x, y - size / 2, z - h / 3), (x, y + size / 2, z - h / 3), (x, y, z + 2 * h / 3)]
    mb.polygon(pts, mat, part, want_normal=normal)
