"""Tiny vector / rigid-transform helpers (stdlib only).

Everything in assetgen is authored in the core frame, ISO 8855:
x forward, y left, z up, metres, right-handed. The glTF writer does the one
basis change to glTF's +Y-up, +Z-forward (+X left).
"""
import math

def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def mul(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def length(a): return math.sqrt(dot(a, a))
def norm(a):
    l = length(a)
    return (a[0] / l, a[1] / l, a[2] / l) if l > 1e-12 else (0.0, 0.0, 0.0)
def lerp(a, b, t): return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)
def dist(a, b): return length(sub(a, b))

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x
def smoothstep(e0, e1, x):
    if e0 == e1:
        return 0.0 if x < e0 else 1.0
    t = clamp((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)
def mix(a, b, t): return a + (b - a) * t


# 3x4 affine transforms as ((r00,r01,r02,tx),(r10,...),(r20,...)).
IDENTITY = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0))

def translate(t):
    return ((1.0, 0.0, 0.0, t[0]), (0.0, 1.0, 0.0, t[1]), (0.0, 0.0, 1.0, t[2]))

def rot_axis(axis, ang):
    x, y, z = norm(axis)
    c, s = math.cos(ang), math.sin(ang)
    C = 1 - c
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0.0),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0.0),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0.0))

def rot_x(a): return rot_axis((1, 0, 0), a)
def rot_y(a): return rot_axis((0, 1, 0), a)
def rot_z(a): return rot_axis((0, 0, 1), a)

def scale(s):
    sx, sy, sz = (s, s, s) if isinstance(s, (int, float)) else s
    return ((sx, 0.0, 0.0, 0.0), (0.0, sy, 0.0, 0.0), (0.0, 0.0, sz, 0.0))

def compose(*ms):
    """compose(A, B, C) applies C first, then B, then A (like A*B*C)."""
    out = IDENTITY
    for m in ms:
        out = _mm(out, m)
    return out

def _mm(a, b):
    r = []
    for i in range(3):
        row = []
        for j in range(4):
            v = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            if j == 3:
                v += a[i][3]
            row.append(v)
        r.append(tuple(row))
    return tuple(r)

def xform_point(m, p):
    return (m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3],
            m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3],
            m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3])

def xform_dir(m, d):
    return (m[0][0] * d[0] + m[0][1] * d[1] + m[0][2] * d[2],
            m[1][0] * d[0] + m[1][1] * d[1] + m[1][2] * d[2],
            m[2][0] * d[0] + m[2][1] * d[1] + m[2][2] * d[2])

def frame_z_to(axis, origin=(0.0, 0.0, 0.0)):
    """Transform mapping local +z onto `axis` (local x/y chosen stably), placed at origin."""
    z = norm(axis)
    ref = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0)
    x = norm(cross(ref, z))
    y = cross(z, x)
    return ((x[0], y[0], z[0], origin[0]), (x[1], y[1], z[1], origin[1]), (x[2], y[2], z[2], origin[2]))

def is_rigid(m):
    """True when the linear part has determinant > 0 (no mirroring)."""
    a = m
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
           - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
           + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    return det > 0


class Curve:
    """Monotone cubic (Fritsch-Carlson) interpolation through (x, y) keys.
    Used for body profiles: no overshoot between keys."""

    def __init__(self, keys):
        keys = sorted(keys)
        self.xs = [k[0] for k in keys]
        self.ys = [k[1] for k in keys]
        n = len(keys)
        d = [(self.ys[i + 1] - self.ys[i]) / (self.xs[i + 1] - self.xs[i]) for i in range(n - 1)]
        m = [0.0] * n
        m[0], m[-1] = d[0], d[-1]
        for i in range(1, n - 1):
            m[i] = 0.0 if d[i - 1] * d[i] <= 0 else (d[i - 1] + d[i]) / 2
        for i in range(n - 1):
            if d[i] == 0:
                m[i] = m[i + 1] = 0.0
                continue
            a, b = m[i] / d[i], m[i + 1] / d[i]
            s = a * a + b * b
            if s > 9:
                t = 3 / math.sqrt(s)
                m[i], m[i + 1] = t * a * d[i], t * b * d[i]
        self.m = m

    def __call__(self, x):
        xs, ys, m = self.xs, self.ys, self.m
        if x <= xs[0]:
            return ys[0]
        if x >= xs[-1]:
            return ys[-1]
        lo, hi = 0, len(xs) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if xs[mid] <= x:
                lo = mid
            else:
                hi = mid
        h = xs[hi] - xs[lo]
        t = (x - xs[lo]) / h
        t2, t3 = t * t, t * t * t
        return ((2 * t3 - 3 * t2 + 1) * ys[lo] + (t3 - 2 * t2 + t) * h * m[lo]
                + (-2 * t3 + 3 * t2) * ys[hi] + (t3 - t2) * h * m[hi])
