"""Track-specific tileable PBR textures (stdlib only). Reuses the public
noise and PNG helpers from tools/assetgen/assetgen/textures.py (fbm, png)
without modifying that module; the small per-pixel image helpers below are
local because the assetgen equivalents are private.

Each generator returns {"base": Image, "mr": Image} (Image = (w, h, ch, bytes)),
following the same glTF convention as assetgen: mr.G = roughness, mr.B = metallic.
"""
from assetgen.textures import fbm, png  # noqa: F401  (png re-exported for callers)


def _b(v):
    return max(0, min(255, int(v * 255 + 0.5)))


def _lerp3(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def _img(size, ch, fn):
    data = bytearray(size * size * ch)
    k = 0
    for y in range(size):
        for x in range(size):
            v = fn(x, y)
            for c in range(ch):
                data[k] = _b(v[c])
                k += 1
    return (size, size, ch, bytes(data))


def asphalt(size=160, seed=21, color=(0.085, 0.085, 0.09)):
    n = fbm(size, 20, 4, seed)
    g = fbm(size, 4, 3, seed + 5)

    def fb(x, y):
        v = 0.85 + 0.22 * n[y][x] - 0.06 * g[y][x]
        return (color[0] * v, color[1] * v, color[2] * v, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.75 + 0.15 * n[y][x], 0.0))
    return {"base": base, "mr": mr}


def marking(size=8, color=(0.92, 0.92, 0.88)):
    base = _img(size, 4, lambda x, y: (color[0], color[1], color[2], 1.0))
    mr = _img(size, 3, lambda x, y: (0.0, 0.55, 0.0))
    return {"base": base, "mr": mr}


def kerb(size=128, red=(0.62, 0.08, 0.06), white=(0.88, 0.86, 0.82)):
    """Stripes across the kerb (v = along the kerb, one red+white pair per tile)."""
    n = fbm(size, 16, 3, 23)

    def fb(x, y):
        stripe = white if (y // (size // 2)) % 2 == 0 else red
        v = 0.85 + 0.15 * n[y][x]
        return (stripe[0] * v, stripe[1] * v, stripe[2] * v, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.6, 0.0))
    return {"base": base, "mr": mr}


def grass(size=160, seed=31, color=(0.20, 0.34, 0.12)):
    n = fbm(size, 24, 4, seed)
    blade = fbm(size, 60, 2, seed + 9)

    def fb(x, y):
        c = _lerp3(color, (0.28, 0.42, 0.16), blade[y][x])
        v = 0.75 + 0.4 * n[y][x]
        return (c[0] * v, c[1] * v, c[2] * v, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.92, 0.0))
    return {"base": base, "mr": mr}


def dirt(size=160, seed=37, color=(0.36, 0.27, 0.18)):
    n = fbm(size, 20, 4, seed)
    speck = fbm(size, 48, 2, seed + 11)

    def fb(x, y):
        c = _lerp3(color, (0.24, 0.18, 0.12), speck[y][x] * 0.6)
        v = 0.8 + 0.3 * n[y][x]
        return (c[0] * v, c[1] * v, c[2] * v, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.88 + 0.1 * n[y][x], 0.0))
    return {"base": base, "mr": mr}


def low_mu(size=128, seed=43, color=(0.72, 0.80, 0.86)):
    """Polished ice: pale blue-white, low roughness (glossy)."""
    n = fbm(size, 12, 3, seed)
    crack = fbm(size, 30, 2, seed + 4)

    def fb(x, y):
        v = 0.85 + 0.2 * n[y][x]
        c = _lerp3(color, (0.55, 0.65, 0.72), max(0.0, crack[y][x] - 0.6) * 1.5)
        return (c[0] * v, c[1] * v, c[2] * v, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.12 + 0.1 * n[y][x], 0.0))
    return {"base": base, "mr": mr}
