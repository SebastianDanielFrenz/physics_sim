"""Deterministic terrain height/surface spec for the 100 x 100 km world
(PLAN.md M1, scope note 2026-09-22: "the test ground must be much bigger").

This module is the reference (Python) evaluator. A C++ tile system (built
later, in core/, not by this package) must reproduce it bit-for-bit; every
operation below is written to be trivially portable: 32-bit unsigned integer
hashing (wraparound add/multiply, logical shifts -- native uint32_t behaviour
in C/C++) and IEEE-754 double-precision arithmetic throughout, matching the
core's own `ps::real = double` (PLAN.md D4). terrain_golden.json is a set of
(x, y) -> (z, surface) samples for the C++ side to test against.

Frame: world east-north-up, x=east, y=north, z=up, metres -- see layout.py's
module docstring (same right-handed axes as a vehicle's own ISO 8855 frame;
the world is oriented so the main spawn's forward is due east).

This is *one* terrain source (PLAN.md section 6/P11 seam for a later
real-world DEM + land-cover source behind the same interface): a height field
plus a *per-cell* surface id (one id per tile-sized cell, see `tile_key()` /
`terrain_surface_cell()`), kept structurally separate from the feature
meshes, which carry their own per-triangle surface id via the collision
grouping in collision.py/test_ground_collision.json. The two are never mixed:
a point is either inside a meshed feature's exact footprint (that mesh's
surface applies) or it is open terrain (this per-cell surface applies), and
every meshed feature sits on a flat zone (below) so its footprint is exactly
flat ground here regardless of the noise. Physical surface parameters
(lambda_mu, crr) for every surface id used by either source live in one
place, data/surfaces/surfaces.json -- not duplicated here or in
test_ground.json.

Height model
------------
height(x, y) = height_noise(x, y) * (1 - flat_mask(x, y))

`flat_mask` is 1.0 inside a test area (exactly flat ground for the meshed
features) and smoothly falls to 0.0 over a blend width outside it, so the
noise is faded out to exactly 0 rather than the ground stepping to it.

`height_noise` is a sum of fixed-parameter octaves of *value noise* (bilinear
interpolation, with a smoothstep-remapped fractional coordinate, of hashed
lattice values), giving gentle rolling farmland; a further "hill" octave is
scaled in by `roughness(x, y)`, which ramps up with distance from the origin,
for occasional steeper hills further out.

Hash: a MurmurHash3-style 32-bit finalizer (public domain, Austin Appleby),
seeded with the lattice cell and a per-octave seed. This is not a
floating-point sin() hash (those vary slightly between compilers/platforms
and are not bit-portable); every step here is exact 32-bit unsigned integer
arithmetic until the final division to a double.
"""
import json
import math
import os
import struct
import zlib

MASK32 = 0xFFFFFFFF


# --------------------------------------------------------------------- hash
def hash2d(ix, iy, seed):
    """uint32 hash of an integer lattice point + seed. All arithmetic wraps
    modulo 2**32 (native uint32_t semantics); shifts are logical (unsigned)."""
    h = (((ix & MASK32) * 0x27D4EB2D) & MASK32)
    h = (h + (((iy & MASK32) * 0x165667B1) & MASK32)) & MASK32
    h = (h + (seed & MASK32)) & MASK32
    h ^= h >> 15
    h = (h * 0x85EBCA6B) & MASK32
    h ^= h >> 13
    h = (h * 0xC2B2AE35) & MASK32
    h ^= h >> 16
    return h


def lattice_value(ix, iy, seed):
    """[-1, 1], double precision. hash / UINT32_MAX is exact in a double."""
    return (hash2d(ix, iy, seed) / 4294967295.0) * 2.0 - 1.0


def smoothstep01(t):
    return t * t * (3.0 - 2.0 * t)


def value_noise(x, y, wavelength, seed):
    """Value noise in [-1, 1], continuous, C1 (smoothstep-interpolated)."""
    gx, gy = x / wavelength, y / wavelength
    ix0, iy0 = math.floor(gx), math.floor(gy)
    fx, fy = gx - ix0, gy - iy0
    sx, sy = smoothstep01(fx), smoothstep01(fy)
    ix0, iy0 = int(ix0), int(iy0)
    v00 = lattice_value(ix0, iy0, seed)
    v10 = lattice_value(ix0 + 1, iy0, seed)
    v01 = lattice_value(ix0, iy0 + 1, seed)
    v11 = lattice_value(ix0 + 1, iy0 + 1, seed)
    a = v00 + (v10 - v00) * sx
    b = v01 + (v11 - v01) * sx
    return a + (b - a) * sy


# ------------------------------------------------------------------- octaves
# Base seed; every octave/channel below derives its own seed by adding a fixed
# per-channel offset, so channels never accidentally correlate.
BASE_SEED = 0x5EED0001

# "Gentle rolling farmland": wavelength and amplitude (metres) per octave,
# roughly geometric in both so each octave adds detail without dominating.
ROLLING_OCTAVES = [
    {"wavelength": 2000.0, "amplitude": 22.0, "seed_offset": 1},
    {"wavelength": 600.0, "amplitude": 10.0, "seed_offset": 2},
    {"wavelength": 200.0, "amplitude": 4.0, "seed_offset": 3},
    {"wavelength": 70.0, "amplitude": 1.4, "seed_offset": 4},
]

# A larger-amplitude "hill" octave, scaled in by roughness(dist) far from the
# origin -- "a few steeper hills further out".
HILL_OCTAVE = {"wavelength": 900.0, "amplitude": 55.0, "seed_offset": 5}
ROUGHNESS_NEAR_M = 8000.0    # roughness(dist) = 0 at/inside this radius
ROUGHNESS_FAR_M = 30000.0    # roughness(dist) = 1 at/beyond this radius


def roughness(x, y):
    d = math.hypot(x, y)
    if d <= ROUGHNESS_NEAR_M:
        return 0.0
    if d >= ROUGHNESS_FAR_M:
        return 1.0
    return smoothstep01((d - ROUGHNESS_NEAR_M) / (ROUGHNESS_FAR_M - ROUGHNESS_NEAR_M))


def height_noise(x, y):
    h = 0.0
    for oc in ROLLING_OCTAVES:
        h += oc["amplitude"] * value_noise(x, y, oc["wavelength"], BASE_SEED + oc["seed_offset"])
    r = roughness(x, y)
    if r > 0.0:
        h += r * HILL_OCTAVE["amplitude"] * value_noise(x, y, HILL_OCTAVE["wavelength"],
                                                          BASE_SEED + HILL_OCTAVE["seed_offset"])
    return h


# ---------------------------------------------------------------- flat zones
# Disc (radius, blend) plus a list of oriented rectangular corridors (centre,
# heading_deg, half_length, half_width, blend), one per long feature that
# cannot fit in the disc. Populated from trackgen.layout so the flat zone a
# feature sits on is always derived from that feature's own true footprint
# (single source of truth) -- see layout.py's LONG_STRAIGHT / AIRSTRIP.
MAIN_DISC = {"center": (0.0, 0.0), "radius": 3000.0, "blend": 400.0}


def _corridor_mask(x, y, c):
    cx, cy = c["center"]
    h = math.radians(c["heading_deg"])
    fx, fy = math.cos(h), math.sin(h)
    lx, ly = -math.sin(h), math.cos(h)
    dx, dy = x - cx, y - cy
    along = dx * fx + dy * fy
    across = dx * lx + dy * ly
    hl, hw, b = c["half_length"], c["half_width"], c["blend"]
    # distance outside the flat rectangle in each axis (0 if inside)
    d_along = max(0.0, abs(along) - hl)
    d_across = max(0.0, abs(across) - hw)
    d = math.hypot(d_along, d_across)
    if d <= 0.0:
        return 1.0
    if d >= b:
        return 0.0
    return 1.0 - smoothstep01(d / b)


def _disc_mask(x, y, disc):
    cx, cy = disc["center"]
    d = math.hypot(x - cx, y - cy) - disc["radius"]
    if d <= 0.0:
        return 1.0
    if d >= disc["blend"]:
        return 0.0
    return 1.0 - smoothstep01(d / disc["blend"])


def flat_zones():
    """Disc + corridors. Imported lazily to avoid a hard import cycle (layout
    does not need terrain, but terrain derives corridors from layout)."""
    from . import layout as L
    zones = [dict(MAIN_DISC, kind="disc", name="main_disc")]
    zones.append(dict(kind="corridor", name="long_straight_corridor", **L.LONG_STRAIGHT["corridor"]))
    zones.append(dict(kind="corridor", name="airstrip_corridor", **L.AIRSTRIP["corridor"]))
    return zones


def flat_mask(x, y, zones=None):
    if zones is None:
        zones = flat_zones()
    m = 0.0
    for z in zones:
        m = max(m, _disc_mask(x, y, z) if z["kind"] == "disc" else _corridor_mask(x, y, z))
    return m


def height(x, y, zones=None):
    return height_noise(x, y) * (1.0 - flat_mask(x, y, zones))


# ------------------------------------------------------------- surface rules
# Ordered list; first matching rule wins, "grass" is the default. Each
# non-default rule names a noise channel (its own wavelength/seed) and a
# threshold on that channel's *normalised* [0, 1] value.
SURFACE_RULES = [
    {"surface": "dirt", "wavelength": 900.0, "seed_offset": 101, "min": 0.62},
    {"surface": "field", "wavelength": 1600.0, "seed_offset": 102, "min": 0.55},
]
DEFAULT_SURFACE = "grass"


def terrain_surface(x, y):
    """Continuous evaluation of the surface rules at an exact point. Not the
    authoritative API -- see terrain_surface_cell(); exposed for anyone who
    already has a tile centre point in hand."""
    for rule in SURFACE_RULES:
        v01 = value_noise(x, y, rule["wavelength"], BASE_SEED + rule["seed_offset"]) * 0.5 + 0.5
        if v01 >= rule["min"]:
            return rule["surface"]
    return DEFAULT_SURFACE


def tile_key(x, y, tile_size=None):
    """(tile_x, tile_y): the tile containing world point (x, y). floor(x /
    tile_size), floor(y / tile_size) -- integer division toward -inf, so
    tiles tile the plane with no gap/overlap through the origin."""
    ts = tile_size or TILE_SIZE_M
    return int(math.floor(x / ts)), int(math.floor(y / ts))


def tile_center(tile_x, tile_y, tile_size=None):
    ts = tile_size or TILE_SIZE_M
    return (tile_x + 0.5) * ts, (tile_y + 0.5) * ts


def terrain_surface_cell(tile_x, tile_y, tile_size=None):
    """The authoritative surface id: one per (tile_x, tile_y) cell at
    `tile_size` (default TILE_SIZE_M) granularity, taken by evaluating the
    continuous rules at the tile's centre point. A future land-cover source
    behind the same interface would instead return that tile's dominant
    raster class -- callers must only ever ask for a surface by tile key, not
    by an arbitrary point, so both sources present the same shape of API."""
    cx, cy = tile_center(tile_x, tile_y, tile_size)
    return terrain_surface(cx, cy)


# --------------------------------------------------------------------- spec
WORLD_HALF_EXTENT_M = 50_000.0
TILE_SIZE_M = 256.0
SAMPLE_SPACING_NEAR_M = 1.0
SAMPLE_SPACING_FAR_M = 8.0
LOD_NEAR_RADIUS_M = 500.0


def build_spec():
    from . import layout as L
    return {
        "format": "physics_sim.terrain/1",
        "seed": BASE_SEED,
        "world": {"half_extent_m": WORLD_HALF_EXTENT_M,
                  "frame": "Local east-north-up: x=east, y=north, z=up, metres (see layout.py's module "
                          "docstring -- algebraically the same right-handed axes as a vehicle's own "
                          "ISO 8855 frame; the world is oriented so the main spawn's forward is east).",
                  "note": "Square world, x and y each in [-half_extent_m, +half_extent_m]."},
        "precision": "Evaluate in IEEE-754 double precision throughout (matches ps::real = double, "
                     "PLAN.md D4). The hash is exact-integer (uint32_t wraparound); only the final "
                     "hash-to-float division is floating point.",
        "hash": {
            "name": "murmur3_fmix32_seeded",
            "reference": "MurmurHash3 fmix32 finalizer (public domain, Austin Appleby), with an added "
                         "seeded lattice-coordinate mix before the finaliser.",
            "pseudocode": [
                "h = (uint32)(ix) * 0x27D4EB2D",
                "h = h + (uint32)(iy) * 0x165667B1   (mod 2**32)",
                "h = h + (uint32)(seed)               (mod 2**32)",
                "h = h ^ (h >> 15)",
                "h = h * 0x85EBCA6B                   (mod 2**32)",
                "h = h ^ (h >> 13)",
                "h = h * 0xC2B2AE35                   (mod 2**32)",
                "h = h ^ (h >> 16)",
                "return h                              // uint32",
            ],
            "lattice_value": "value01 = hash / 4294967295.0; value = value01*2.0 - 1.0  -> [-1, 1] double",
        },
        "value_noise": {
            "description": "Bilinear interpolation of the 4 surrounding lattice values, using the "
                           "fractional coordinate remapped by smoothstep(t) = t*t*(3-2*t) on both axes "
                           "(standard Hermite smoothstep, applied to x then y).",
            "formula": "gx=x/wavelength; gy=y/wavelength; ix0=floor(gx); iy0=floor(gy); "
                      "fx=gx-ix0; fy=gy-iy0; sx=smoothstep(fx); sy=smoothstep(fy); "
                      "a=lerp(v(ix0,iy0),v(ix0+1,iy0),sx); b=lerp(v(ix0,iy0+1),v(ix0+1,iy0+1),sx); "
                      "noise=lerp(a,b,sy)",
        },
        "height_noise": {
            "formula": "sum(amplitude_k * value_noise(x, y, wavelength_k, seed + seed_offset_k)) over "
                      "ROLLING_OCTAVES, plus roughness(x,y) * hill.amplitude * "
                      "value_noise(x,y,hill.wavelength,seed+hill.seed_offset)",
            "rolling_octaves": ROLLING_OCTAVES,
            "hill_octave": HILL_OCTAVE,
            "roughness": {"near_radius_m": ROUGHNESS_NEAR_M, "far_radius_m": ROUGHNESS_FAR_M,
                         "formula": "0 at/inside near_radius_m, 1 at/beyond far_radius_m, "
                                   "smoothstep(dist_from_origin) in between"},
        },
        "flat_zones": {
            "description": "height(x,y) = height_noise(x,y) * (1 - flat_mask(x,y)); flat_mask is the max "
                           "over every zone below of: 1 inside the zone, smoothstep-blended to 0 over "
                           "the zone's own blend width outside it.",
            "zones": flat_zones(),
        },
        "surface_rules": {
            "description": "Surface is a *per-cell* property, one id per (tile_x, tile_y) cell at "
                           "tiling.tile_size_m granularity: tile_x=floor(x/tile_size_m), "
                           "tile_y=floor(y/tile_size_m); a cell's surface is these rules evaluated at "
                           "its centre point ((tile_x+0.5)*tile_size_m, (tile_y+0.5)*tile_size_m) -- "
                           "never at an arbitrary point, so every point in a cell agrees. This is the "
                           "fill surface of the open terrain only; a meshed test feature's own "
                           "footprint (always on a flat zone, above) is covered by that mesh's own "
                           "per-triangle collision surface instead (test_ground_collision.json) -- the "
                           "two are never blended. Rules evaluated in order, first whose channel value "
                           ">= 'min' wins; default is '" + DEFAULT_SURFACE + "' if none match. Each "
                           "rule's channel is value_noise(cell_cx,cell_cy,wavelength,seed+seed_offset), "
                           "renormalised from [-1,1] to [0,1] before the threshold.",
            "default": DEFAULT_SURFACE,
            "rules": SURFACE_RULES,
        },
        "tiling": {
            "tile_size_m": TILE_SIZE_M,
            "sample_spacing_near_m": SAMPLE_SPACING_NEAR_M,
            "sample_spacing_far_m": SAMPLE_SPACING_FAR_M,
            "lod_near_radius_m": LOD_NEAR_RADIUS_M,
            "note": "256 m tiles are both the height-mesh LOD unit (suggestion, not a requirement: 1 m "
                   "vertex spacing within lod_near_radius_m of the player/camera, coarsening towards "
                   "sample_spacing_far_m with distance; regenerate/re-mesh a tile deterministically "
                   "from (tile_x, tile_y, seed), no persisted state needed) and the surface-id cell "
                   "size (a requirement -- see surface_rules).",
        },
        "surface_params_ref": "data/surfaces/surfaces.json: physical lambda_mu/crr values for every "
                              "surface id used here (grass, dirt, field) or by the feature meshes "
                              "(asphalt, kerb, low_mu) -- one shared table, not duplicated per source "
                              "(PLAN.md P11: a later real-world land-cover source maps onto the same ids).",
    }


# ------------------------------------------------------------------- golden
def _grid_points(n, half_extent):
    if n < 2:
        return [0.0]
    return [-half_extent + 2 * half_extent * k / (n - 1) for k in range(n)]


def golden_points():
    """A deterministic, hand-structured (not RNG-based) set of (x, y) probes:
    a coarse grid over the whole world, plus explicit boundary-condition
    points (disc centre/edge/blend, corridor points, far hill region)."""
    from . import layout as L
    pts = []
    for x in _grid_points(13, WORLD_HALF_EXTENT_M):
        for y in _grid_points(13, WORLD_HALF_EXTENT_M):
            pts.append((x, y))
    d = MAIN_DISC
    for frac in (0.0, 0.5, 0.999, 1.0, 1.0 + 0.5 * d["blend"] / d["radius"], 1.0 + 2.0 * d["blend"] / d["radius"]):
        r = d["radius"] * frac
        for ang_deg in (0, 45, 90, 135, 180, 225, 270, 315):
            a = math.radians(ang_deg)
            pts.append((d["center"][0] + r * math.cos(a), d["center"][1] + r * math.sin(a)))
    for corridor in (L.LONG_STRAIGHT["corridor"], L.AIRSTRIP["corridor"]):
        cx, cy = corridor["center"]
        h = math.radians(corridor["heading_deg"])
        fx, fy = math.cos(h), math.sin(h)
        lx, ly = -math.sin(h), math.cos(h)
        for along_frac in (-1.05, -1.0, -0.5, 0.0, 0.5, 1.0, 1.05):
            for across_frac in (0.0, 1.0, 1.0 + corridor["blend"] / max(corridor["half_width"], 1.0) * 0.5):
                along = along_frac * corridor["half_length"]
                across = across_frac * corridor["half_width"]
                pts.append((cx + along * fx + across * lx, cy + along * fy + across * ly))
    for ang_deg in range(0, 360, 15):
        a = math.radians(ang_deg)
        for r in (10000.0, 20000.0, 35000.0, 49000.0):
            pts.append((r * math.cos(a), r * math.sin(a)))
    # de-duplicate (some corners coincide) while preserving order
    seen = set()
    out = []
    for p in pts:
        key = (round(p[0], 3), round(p[1], 3))
        if key not in seen:
            seen.add(key)
            out.append(p)
    return out


def build_golden():
    """z is height(x, y) at the exact point (continuous). surface is the
    *cell's* surface (terrain_surface_cell of the tile containing (x, y)),
    the authoritative per-cell model -- tile_x/tile_y are included so the
    C++ side can look its own cell up directly instead of re-deriving it."""
    zones = flat_zones()
    pts = golden_points()
    out = []
    for x, y in pts:
        z = height(x, y, zones)
        tx, ty = tile_key(x, y)
        s = terrain_surface_cell(tx, ty)
        out.append({"x": round(x, 3), "y": round(y, 3), "z": round(z, 6),
                    "tile_x": tx, "tile_y": ty, "surface": s})
    return {"format": "physics_sim.terrain_golden/2", "seed": BASE_SEED, "tile_size_m": TILE_SIZE_M,
            "count": len(out), "points": out}


def write_terrain_files(out_dir):
    spec = build_spec()
    golden = build_golden()
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "terrain.json"), "w", encoding="utf-8") as f:
        json.dump(spec, f, indent=2)
        f.write("\n")
    with open(os.path.join(out_dir, "terrain_golden.json"), "w", encoding="utf-8") as f:
        json.dump(golden, f, indent=2)
        f.write("\n")
    return spec, golden


# -------------------------------------------------------------- PNG preview
def _png_gray(w, h, rows):
    """rows: list of h bytes-objects of length w (uint8 grayscale)."""
    raw = bytearray()
    for row in rows:
        raw.append(0)
        raw += row

    def chunk(tag, body):
        c = struct.pack(">I", len(body)) + tag + body
        return c + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))


def _height_lowpass(x, y, zones, min_wavelength):
    """Same as height(), but omitting any octave whose wavelength is below
    `min_wavelength` -- a preview-only anti-aliasing aid (point-sampling a
    1024 px image of a +-50 km world lands well over a pixel per fine-octave
    wavelength otherwise, which aliases to visual noise; this is not part of
    the authoritative spec, just how render_heightmap avoids that)."""
    h = 0.0
    for oc in ROLLING_OCTAVES:
        if oc["wavelength"] < min_wavelength:
            continue
        h += oc["amplitude"] * value_noise(x, y, oc["wavelength"], BASE_SEED + oc["seed_offset"])
    r = roughness(x, y)
    if r > 0.0 and HILL_OCTAVE["wavelength"] >= min_wavelength:
        h += r * HILL_OCTAVE["amplitude"] * value_noise(x, y, HILL_OCTAVE["wavelength"],
                                                          BASE_SEED + HILL_OCTAVE["seed_offset"])
    return h * (1.0 - flat_mask(x, y, zones))


def render_heightmap(path, center, half_extent, size=1024, antialias=True):
    """Top-down grayscale heightmap PNG: dark = low, bright = high, mid-grey
    = 0 (flat zones and sea-level-ish rolling terrain)."""
    zones = flat_zones()
    pixel_spacing = 2.0 * half_extent / size
    min_wavelength = 3.0 * pixel_spacing if antialias else 0.0
    zmax = 1e-6
    heights = [[0.0] * size for _ in range(size)]
    for row in range(size):
        y = center[1] + half_extent - 2 * half_extent * (row + 0.5) / size
        for col in range(size):
            x = center[0] - half_extent + 2 * half_extent * (col + 0.5) / size
            h = _height_lowpass(x, y, zones, min_wavelength) if antialias else height(x, y, zones)
            heights[row][col] = h
            zmax = max(zmax, abs(h))
    rows = []
    for row in range(size):
        buf = bytearray(size)
        for col in range(size):
            v = heights[row][col] / zmax  # [-1, 1]
            buf[col] = max(0, min(255, int((v * 0.5 + 0.5) * 255 + 0.5)))
        rows.append(bytes(buf))
    with open(path, "wb") as f:
        f.write(_png_gray(size, size, rows))
    return zmax
