"""Procedural, tileable PBR textures written as PNG (stdlib only: zlib + struct).

Each generator returns {"base": Image, "mr": Image, "normal": Image} (any subset).
Image = (width, height, channels, bytes). metallicRoughness follows glTF:
G = roughness, B = metallic. Normal maps are tangent space, +Y up (OpenGL).
All patterns wrap at the texture edge, so materials tile seamlessly.
"""
import math
import struct
import zlib


# ----------------------------------------------------------------- PNG

def png(img):
    w, h, ch, data = img
    ctype = {1: 0, 3: 2, 4: 6}[ch]
    raw = bytearray()
    stride = w * ch
    for y in range(h):
        raw.append(0)
        raw += data[y * stride:(y + 1) * stride]

    def chunk(tag, body):
        c = struct.pack(">I", len(body)) + tag + body
        return c + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, ctype, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))


# --------------------------------------------------------------- noise

def _hash(ix, iy, seed):
    n = (ix * 374761393 + iy * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    n = ((n ^ (n >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((n ^ (n >> 16)) & 0xFFFF) / 65535.0


def value_noise(size, period, seed):
    """Tileable value noise: `period` lattice cells across the texture."""
    lat = [[_hash(i, j, seed) for i in range(period)] for j in range(period)]
    out = [[0.0] * size for _ in range(size)]
    f = period / size
    for y in range(size):
        gy = y * f
        j0 = int(gy)
        ty = gy - j0
        ty = ty * ty * (3 - 2 * ty)
        r0, r1 = lat[j0 % period], lat[(j0 + 1) % period]
        row = out[y]
        for x in range(size):
            gx = x * f
            i0 = int(gx)
            tx = gx - i0
            tx = tx * tx * (3 - 2 * tx)
            i1 = (i0 + 1) % period
            i0 %= period
            a = r0[i0] + (r0[i1] - r0[i0]) * tx
            b = r1[i0] + (r1[i1] - r1[i0]) * tx
            row[x] = a + (b - a) * ty
    return out


def fbm(size, period, octaves, seed, gain=0.5):
    acc = [[0.0] * size for _ in range(size)]
    amp, tot = 1.0, 0.0
    for o in range(octaves):
        p = period * (2 ** o)
        if p > size:
            break
        n = value_noise(size, p, seed + 101 * o)
        for y in range(size):
            ra, rn = acc[y], n[y]
            for x in range(size):
                ra[x] += rn[x] * amp
        tot += amp
        amp *= gain
    for y in range(size):
        ra = acc[y]
        for x in range(size):
            ra[x] /= tot
    return acc


# ------------------------------------------------------------- helpers

def _img(size, ch, fn):
    data = bytearray(size * size * ch)
    k = 0
    for y in range(size):
        for x in range(size):
            v = fn(x, y)
            for c in range(ch):
                data[k] = max(0, min(255, int(v[c] * 255 + 0.5)))
                k += 1
    return (size, size, ch, bytes(data))


def normal_from_height(hmap, strength):
    size = len(hmap)

    def fn(x, y):
        dx = hmap[y][(x + 1) % size] - hmap[y][(x - 1) % size]
        dy = hmap[(y + 1) % size][x] - hmap[(y - 1) % size][x]
        nx, ny, nz = -dx * strength, dy * strength, 1.0
        l = math.sqrt(nx * nx + ny * ny + nz * nz)
        return (nx / l * 0.5 + 0.5, ny / l * 0.5 + 0.5, nz / l * 0.5 + 0.5)

    return _img(size, 3, fn)


def _lerp3(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def _ss(e0, e1, x):
    t = max(0.0, min(1.0, (x - e0) / (e1 - e0)))
    return t * t * (3 - 2 * t)


def srgb(c):
    """Colours below are given in sRGB 0..1 (what glTF expects in base colour textures)."""
    return c


# ---------------------------------------------------------- generators

def paint(color, rust=0.0, fade=0.0, dirt=0.0, scratches=0.0, metallic=0.0, rough=0.35, seed=1, size=256):
    """Car / machine paint. rust, fade, dirt, scratches in 0..1."""
    blot = fbm(size, 4, 5, seed)
    grain = fbm(size, 32, 3, seed + 7)
    streak = fbm(size, 2, 4, seed + 13)
    rust_c = (0.42, 0.20, 0.08)
    rust_dark = (0.22, 0.10, 0.05)
    dirt_c = (0.33, 0.28, 0.21)
    primer = (0.55, 0.55, 0.52)
    base = bytearray(size * size * 4)
    mr = bytearray(size * size * 3)
    k = kk = 0
    for y in range(size):
        for x in range(size):
            b, g, s = blot[y][x], grain[y][x], streak[y][x]
            c = color
            if fade > 0:  # sun-faded, chalky, blotchy
                c = _lerp3(c, (0.8, 0.8, 0.78), fade * (0.25 + 0.5 * s))
            r = rough + fade * 0.35 * s
            m = metallic
            if scratches > 0:
                sc = abs(math.sin((x * 0.9 + y * 0.35 + 40 * s) * 0.6))
                if sc > 1 - 0.02 * scratches and g > 0.5:
                    c = primer
                    r = 0.6
            if rust > 0:
                t = _ss(1 - rust * 0.55, 1 - rust * 0.45, b * 0.8 + g * 0.2)
                if t > 0:
                    rc = _lerp3(rust_c, rust_dark, g)
                    # a primer halo around rust spots
                    halo = _ss(1 - rust * 0.62, 1 - rust * 0.55, b * 0.8 + g * 0.2) - t
                    c = _lerp3(c, primer, max(0.0, halo) * 0.6)
                    c = _lerp3(c, rc, t)
                    r = r + (0.95 - r) * t
                    m = m * (1 - t)
            if dirt > 0:
                t = _ss(0.35, 0.8, s * 0.6 + g * 0.4) * dirt
                c = _lerp3(c, dirt_c, t)
                r = r + (0.95 - r) * t
            v = 0.94 + 0.12 * g
            base[k:k + 4] = bytes((_b(c[0] * v), _b(c[1] * v), _b(c[2] * v), 255))
            mr[kk:kk + 3] = bytes((0, _b(r), _b(m)))
            k += 4
            kk += 3
    return {"base": (size, size, 4, bytes(base)), "mr": (size, size, 3, bytes(mr))}


def _b(v):
    return max(0, min(255, int(v * 255 + 0.5)))


def rubber(seed=3, size=128, shade=0.075):
    n = fbm(size, 16, 3, seed)
    base = _img(size, 4, lambda x, y: (shade + 0.03 * n[y][x],) * 3 + (1.0,))
    mr = _img(size, 3, lambda x, y: (0.0, 0.85 + 0.1 * n[y][x], 0.0))
    return {"base": base, "mr": mr}


def tread_car(grooves=4, size=256, seed=5):
    """Road tyre crown. Texture x = across the crown (the full tread width per tile),
    texture y = around the tyre (one tread pitch per tile)."""
    n = fbm(size, 16, 3, seed)
    h = [[0.0] * size for _ in range(size)]
    for y in range(size):
        u = y / size
        for x in range(size):
            v = x / size
            hh = 1.0
            for g in range(grooves):  # circumferential grooves
                gc = (g + 1) / (grooves + 1)
                if abs(v - gc) < 0.028:
                    hh = 0.0
            # angled lateral grooves in the shoulders, one per pitch
            if v < 0.2 or v > 0.8:
                off = (u + (v if v < 0.5 else 1 - v) * 0.8) % 1.0
                if abs(off - 0.5) < 0.06:
                    hh = 0.0
            else:  # sipes in the centre ribs
                off = (u * 3 + v * 1.5) % 1.0
                if abs(off - 0.5) < 0.02:
                    hh = min(hh, 0.4)
            h[y][x] = hh
    base = _img(size, 4, lambda x, y: ((0.05 + 0.04 * h[y][x] + 0.02 * n[y][x]),) * 3 + (1.0,))
    mr = _img(size, 3, lambda x, y: (0.0, 0.8 + 0.15 * (1 - h[y][x]), 0.0))
    return {"base": base, "mr": mr, "normal": normal_from_height(h, 6.0)}


def tread_slick(size=128, seed=6):
    n = fbm(size, 8, 4, seed)
    base = _img(size, 4, lambda x, y: ((0.06 + 0.035 * n[y][x]),) * 3 + (1.0,))
    mr = _img(size, 3, lambda x, y: (0.0, 0.55 + 0.25 * n[y][x], 0.0))
    return {"base": base, "mr": mr}


def carbon(size=256, weave=8, tint=(0.05, 0.05, 0.055)):
    """2x2 twill carbon fibre under clear coat. `weave` tows per tile."""
    h = [[0.0] * size for _ in range(size)]
    alb = [[0.0] * size for _ in range(size)]
    cell = size // weave
    for y in range(size):
        for x in range(size):
            cx, cy = x // cell, y // cell
            fx, fy = (x % cell) / cell, (y % cell) / cell
            warp = ((cx + cy) // 2) % 2 == 0  # twill: diagonal steps of two
            t = fx if warp else fy
            across = fy if warp else fx
            bump = math.sin(math.pi * t)
            h[y][x] = 0.6 * bump + 0.4 * (1 - abs(across - 0.5) * 2) ** 0.3
            alb[y][x] = (0.55 + 0.45 * bump) * (1.0 if warp else 0.8)
    base = _img(size, 4, lambda x, y: (tint[0] * (0.6 + alb[y][x]), tint[1] * (0.6 + alb[y][x]),
                                       tint[2] * (0.6 + alb[y][x]), 1.0))
    mr = _img(size, 3, lambda x, y: (0.0, 0.18 + 0.1 * (1 - alb[y][x]), 0.0))
    return {"base": base, "mr": mr, "normal": normal_from_height(h, 1.5)}


def honeycomb(size=128, cells=8, color=(0.06, 0.06, 0.065)):
    """Grille mesh: dark hexagonal cells with deeper black holes."""
    h = [[0.0] * size for _ in range(size)]
    s = size / cells
    for y in range(size):
        for x in range(size):
            # offset rows -> hex-ish pattern
            row = int(y / s)
            ox = (x / s + (0.5 if row % 2 else 0.0)) % 1.0
            oy = (y / s) % 1.0
            d = max(abs(ox - 0.5) * 1.15 + abs(oy - 0.5) * 0.58, abs(oy - 0.5) * 1.15)
            h[y][x] = _ss(0.36, 0.44, d)
    base = _img(size, 4, lambda x, y: (color[0] * (0.2 + h[y][x]), color[1] * (0.2 + h[y][x]),
                                       color[2] * (0.2 + h[y][x]), 1.0))
    mr = _img(size, 3, lambda x, y: (0.0, 0.5, 0.2 * h[y][x]))
    return {"base": base, "mr": mr, "normal": normal_from_height(h, 5.0)}


def tread_plate(size=256, seed=9):
    """Steel diamond (checker) plate, worn."""
    n = fbm(size, 8, 4, seed)
    h = [[0.0] * size for _ in range(size)]
    k = 8
    for y in range(size):
        for x in range(size):
            cx, cy = (x * k / size) % 1.0, (y * k / size) % 1.0
            parity = (int(x * k / size) + int(y * k / size)) % 2
            if parity:
                a, b = cx - 0.5, cy - 0.5
                along, across = (a + b) * 0.707, (a - b) * 0.707
            else:
                a, b = cx - 0.5, cy - 0.5
                along, across = (a - b) * 0.707, (a + b) * 0.707
            d = (along / 0.38) ** 2 + (across / 0.09) ** 2
            h[y][x] = max(0.0, 1 - d)
    base = _img(size, 4, lambda x, y: ((0.45 + 0.25 * h[y][x] - 0.15 * n[y][x]),) * 3 + (1.0,))
    mr = _img(size, 3, lambda x, y: (0.0, 0.55 - 0.25 * h[y][x] + 0.2 * n[y][x], 0.9))
    return {"base": base, "mr": mr, "normal": normal_from_height(h, 3.0)}


def ribbed(color, size=128, ribs=4, seed=11, dirt=0.3):
    """Pressed/ribbed sheet metal (trailer walls), ribs run along v."""
    n = fbm(size, 4, 4, seed)
    h = [[0.5 + 0.5 * math.cos(2 * math.pi * ribs * x / size) for x in range(size)] for _ in range(size)]
    dirt_c = (0.33, 0.28, 0.21)

    def fb(x, y):
        t = _ss(0.4, 0.9, n[y][x]) * dirt
        c = _lerp3(color, dirt_c, t)
        v = 0.85 + 0.15 * h[y][x]
        return (c[0] * v, c[1] * v, c[2] * v, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.45 + 0.4 * _ss(0.4, 0.9, n[y][x]) * dirt, 0.3))
    return {"base": base, "mr": mr, "normal": normal_from_height(h, 2.0)}


def fabric(color, size=128, seed=13):
    n = fbm(size, 16, 2, seed)

    def fb(x, y):
        wv = 0.8 + 0.2 * (((x // 2) + (y // 2)) % 2) + 0.1 * n[y][x]
        return (color[0] * wv, color[1] * wv, color[2] * wv, 1.0)

    base = _img(size, 4, fb)
    mr = _img(size, 3, lambda x, y: (0.0, 0.95, 0.0))
    return {"base": base, "mr": mr}


def lens(size=128):
    """Headlight lens: fresnel-ish ribs (normal map only; colour comes from the factor)."""
    h = [[0.5 + 0.5 * math.sin(2 * math.pi * 8 * x / size) * math.sin(2 * math.pi * 4 * y / size)
          for x in range(size)] for y in range(size)]
    return {"normal": normal_from_height(h, 1.0)}


def grime(size=256, seed=17, color=(0.30, 0.26, 0.20)):
    """Mud/dust layer for farm machinery undersides and wheels: colour + roughness."""
    n = fbm(size, 4, 5, seed)
    sp = fbm(size, 32, 2, seed + 3)

    def fb(x, y):
        v = 0.7 + 0.5 * n[y][x] + 0.2 * sp[y][x]
        return (color[0] * v, color[1] * v, color[2] * v, 1.0)

    return {"base": _img(size, 4, fb), "mr": _img(size, 3, lambda x, y: (0.0, 0.9 + 0.1 * n[y][x], 0.0))}
