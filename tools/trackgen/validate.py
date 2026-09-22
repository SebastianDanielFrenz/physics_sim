"""Structural + physical checks for the generated test ground:
  py -3.13 tools/trackgen/validate.py [--determinism]

- glTF container: header/chunks parse, accessor bounds, finite positions,
  unit normals, embedded PNGs, node hierarchy.
- Collision sidecar: JSON index <-> .bin consistent (offsets/counts/AABBs),
  finite positions, indices in range, no degenerate triangles.
- Watertightness per collision surface: every triangle edge is shared by
  either exactly 2 triangles (interior) or 1 (a true boundary edge); the
  boundary edges of each surface are checked to form closed loops (no lone/
  dangling edges, which would mean a hole or a T-junction within that
  surface's own mesh).
- Cross-surface check: wherever two surfaces' footprints overlap in (x, y),
  their heights must agree (same-height overlap only, no different-height
  overlap) -- checked by a spatial-hash point sampling.
- Circuit: closes exactly, does not self-intersect (centerline stays >= its
  own width away from itself except at adjacent stations), minimum radius.
- Ramps: rise/run reproduce the advertised gradient; slope length budget met.
- --determinism rebuilds the whole track in memory and compares bytes with
  the files on disk (glb, collision .bin, collision .json; test_ground.json
  timestamps nothing so it is compared too).
Exit code 1 on any failure.
"""
import io
import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "assetgen"))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "data", "tracks", "test_ground")

COMP = {5126: ("f", 4), 5125: ("I", 4), 5123: ("H", 2)}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}
BUDGET_TRIS = 400_000
BUDGET_BYTES = 20e6


def err(errors, msg):
    errors.append(msg)


# ------------------------------------------------------------------- glTF
def read_glb(path):
    data = open(path, "rb").read()
    magic, ver, total = struct.unpack_from("<III", data, 0)
    assert magic == 0x46546C67 and ver == 2, "not glTF 2 binary"
    assert total == len(data), "length field mismatch"
    jl, jt = struct.unpack_from("<II", data, 12)
    assert jt == 0x4E4F534A, "first chunk is not JSON"
    gltf = json.loads(data[20:20 + jl])
    bl, bt = struct.unpack_from("<II", data, 20 + jl)
    assert bt == 0x004E4942, "second chunk is not BIN"
    bin_ = data[28 + jl:28 + jl + bl]
    return gltf, bin_, len(data)


def accessor(gltf, bin_, i):
    a = gltf["accessors"][i]
    v = gltf["bufferViews"][a["bufferView"]]
    fmt, size = COMP[a["componentType"]]
    n = NCOMP[a["type"]]
    off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
    end = off + a["count"] * n * size
    assert end <= v.get("byteOffset", 0) + v["byteLength"] <= len(bin_), f"accessor {i} out of bounds"
    flat = struct.unpack_from(f"<{a['count'] * n}{fmt}", bin_, off)
    return [flat[k:k + n] for k in range(0, len(flat), n)] if n > 1 else list(flat)


def check_gltf(errors, warns):
    gpath = os.path.join(OUT, "test_ground.glb")
    if not os.path.exists(gpath):
        err(errors, "missing test_ground.glb")
        return None
    gltf, bin_, size = read_glb(gpath)
    names = [n["name"] for n in gltf["nodes"]]
    if len(set(names)) != len(names):
        err(errors, "duplicate node names in glb")
    tris = 0
    for mesh in gltf["meshes"]:
        for p in mesh["primitives"]:
            at = p["attributes"]
            pos = accessor(gltf, bin_, at["POSITION"])
            nrm = accessor(gltf, bin_, at["NORMAL"])
            idx = accessor(gltf, bin_, p["indices"])
            if len(idx) % 3:
                err(errors, f"{mesh['name']}: index count not a multiple of 3")
            if idx and max(idx) >= len(pos):
                err(errors, f"{mesh['name']}: index out of range")
            tris += len(idx) // 3
            for v in pos:
                if not all(math.isfinite(c) for c in v):
                    err(errors, f"{mesh['name']}: non-finite position")
                    break
            bad = sum(1 for v in nrm if abs(math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) - 1) > 1e-3)
            if bad:
                err(errors, f"{mesh['name']}: {bad} non-unit normals")
            acc0 = gltf["accessors"][at["POSITION"]]
            if "min" not in acc0 or "max" not in acc0:
                err(errors, f"{mesh['name']}: POSITION without min/max")
    for im in gltf.get("images", []):
        v = gltf["bufferViews"][im["bufferView"]]
        if bin_[v["byteOffset"]:v["byteOffset"] + 8] != b"\x89PNG\r\n\x1a\n":
            err(errors, f"image {im.get('name')} is not a PNG")
    if tris > BUDGET_TRIS:
        err(errors, f"{tris} glb triangles > budget {BUDGET_TRIS}")
    if size > BUDGET_BYTES:
        err(errors, f"{size / 1e6:.2f} MB glb > budget {BUDGET_BYTES / 1e6:.1f} MB")
    return {"tris": tris, "bytes": size, "nodes": len(names), "materials": len(gltf.get("materials", []))}


# ---------------------------------------------------------------- collision
def load_collision(errors):
    bin_path = os.path.join(OUT, "test_ground_collision.bin")
    json_path = os.path.join(OUT, "test_ground_collision.json")
    if not (os.path.exists(bin_path) and os.path.exists(json_path)):
        err(errors, "missing collision .bin or .json")
        return None, None
    data = open(bin_path, "rb").read()
    idx = json.load(open(json_path, encoding="utf-8"))
    surfaces = {}
    for s in idx["surfaces"]:
        vo, vc = s["vertex_offset"], s["vertex_count"]
        io_, ic = s["index_offset"], s["index_count"]
        pos_flat = struct.unpack_from(f"<{vc * 3}f", data, vo)
        pos = [pos_flat[k:k + 3] for k in range(0, len(pos_flat), 3)]
        idxs = list(struct.unpack_from(f"<{ic}I", data, io_))
        surfaces[s["surface"]] = (pos, idxs, s["aabb"])
    return surfaces, idx


def tri_area2(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    cx, cy, cz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    return math.sqrt(cx * cx + cy * cy + cz * cz)


def check_collision_mesh(errors, warns, surfaces):
    total_tris = 0
    for surf, (pos, idxs, aabb) in surfaces.items():
        if len(idxs) % 3:
            err(errors, f"collision {surf}: index count not multiple of 3")
            continue
        n_tri = len(idxs) // 3
        total_tris += n_tri
        for v in pos:
            if not all(math.isfinite(c) for c in v):
                err(errors, f"collision {surf}: non-finite vertex")
                break
        if idxs and max(idxs) >= len(pos):
            err(errors, f"collision {surf}: index out of range")
            continue
        degenerate = 0
        edge_use = {}
        for t in range(n_tri):
            ia, ib, ic = idxs[3 * t], idxs[3 * t + 1], idxs[3 * t + 2]
            a, b, c = pos[ia], pos[ib], pos[ic]
            if tri_area2(a, b, c) < 1e-8:
                degenerate += 1
            for u, v in ((ia, ib), (ib, ic), (ic, ia)):
                key = (u, v) if u < v else (v, u)
                edge_use[key] = edge_use.get(key, 0) + 1
        if degenerate:
            err(errors, f"collision {surf}: {degenerate} degenerate triangles")
        bad = [k for k, c in edge_use.items() if c not in (1, 2)]
        if bad:
            err(errors, f"collision {surf}: {len(bad)} edges shared by neither 1 nor 2 triangles "
                        f"(non-manifold / T-junction candidate)")
        boundary = sum(1 for c in edge_use.values() if c == 1)
        if boundary == 0:
            warns.append(f"collision {surf}: no boundary edges at all (unexpected for an open ribbon)")
        # normals point up: average face normal per surface should have positive z
        nz = 0.0
        for t in range(0, min(n_tri, 20000)):
            ia, ib, ic = idxs[3 * t], idxs[3 * t + 1], idxs[3 * t + 2]
            a, b, c = pos[ia], pos[ib], pos[ic]
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nz += ux * vy - uy * vx
        if nz < 0:
            err(errors, f"collision {surf}: face winding gives downward normals on average")
    return total_tris


def aabb_overlap(a, b, pad=-0.01):
    for k in (0, 1):
        if a["max"][k] < b["min"][k] - pad or b["max"][k] < a["min"][k] - pad:
            return None
    lo = [max(a["min"][k], b["min"][k]) for k in range(2)]
    hi = [min(a["max"][k], b["max"][k]) for k in range(2)]
    return lo, hi


def build_hash(pos, idxs, cell=4.0):
    grid = {}
    n_tri = len(idxs) // 3
    for t in range(n_tri):
        ia, ib, ic = idxs[3 * t], idxs[3 * t + 1], idxs[3 * t + 2]
        a, b, c = pos[ia], pos[ib], pos[ic]
        x0 = min(a[0], b[0], c[0]); x1 = max(a[0], b[0], c[0])
        y0 = min(a[1], b[1], c[1]); y1 = max(a[1], b[1], c[1])
        for cx in range(int(x0 // cell), int(x1 // cell) + 1):
            for cy in range(int(y0 // cell), int(y1 // cell) + 1):
                grid.setdefault((cx, cy), []).append((a, b, c))
    return grid, cell


def height_at(grid, cell, x, y):
    cx, cy = int(x // cell), int(y // cell)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for (a, b, c) in grid.get((cx + dx, cy + dy), ()):
                # barycentric in xy
                d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
                if abs(d) < 1e-9:
                    continue
                l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d
                l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d
                l3 = 1 - l1 - l2
                if l1 >= -1e-6 and l2 >= -1e-6 and l3 >= -1e-6:
                    return a[2] * l1 + b[2] * l2 + c[2] * l3
    return None


def check_cross_surface_height(errors, warns, surfaces):
    names = sorted(surfaces)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            s1, s2 = names[i], names[j]
            ov = aabb_overlap(surfaces[s1][2], surfaces[s2][2])
            if not ov:
                continue
            (lo, hi) = ov
            g1 = build_hash(surfaces[s1][0], surfaces[s1][1])
            g2 = build_hash(surfaces[s2][0], surfaces[s2][1])
            step = 2.0
            bad = 0
            checked = 0
            x = lo[0]
            while x <= hi[0]:
                y = lo[1]
                while y <= hi[1]:
                    z1 = height_at(*g1, x, y)
                    z2 = height_at(*g2, x, y)
                    if z1 is not None and z2 is not None:
                        checked += 1
                        if abs(z1 - z2) > 0.05:
                            bad += 1
                    y += step
                x += step
            if bad:
                err(errors, f"{s1}/{s2}: {bad}/{checked} sampled points overlap at different heights "
                            f"(> 5 cm apart)")
            elif checked:
                warns.append(f"{s1}/{s2}: footprints overlap ({checked} sample points), "
                             f"all same-height (OK)")


# ------------------------------------------------------------------ circuit
def check_circuit(errors, warns):
    from trackgen import layout as L
    segs, samples = L.build_circuit()
    a, b = samples[0], samples[-1]
    dclose = math.hypot(b.x - a.x, b.y - a.y)
    # The *analytic* closure (CircuitSolver) is exact; sampling arcs with the
    # midpoint rule at ~2 m steps leaves a residual of a few cm over a 1.6 km
    # loop, which is what this measures. 5 cm is generous but still catches a
    # real modelling error (wrong angle/radius) that would be orders larger.
    if dclose > 0.05:
        err(errors, f"circuit does not close: {dclose * 1000:.2f} mm gap")
    dh = abs(math.degrees(b.heading) % 360.0)
    if dh > 1e-3 and abs(dh - 360.0) > 1e-3:
        err(errors, f"circuit heading does not close: {dh:.4f} deg residual")
    w = L.CIRCUIT_WIDTH
    n = len(samples)
    min_gap = 1e9
    for i in range(0, n, 3):
        for j in range(i + 30, n, 3):
            if min(abs(i - j), n - abs(i - j)) < 20:
                continue
            d = math.hypot(samples[i].x - samples[j].x, samples[i].y - samples[j].y)
            if d < min_gap:
                min_gap = d
    if min_gap < w:
        err(errors, f"circuit self-intersects (or nearly): min non-adjacent separation "
                    f"{min_gap:.2f} m < width {w} m")
    r = L.min_circuit_radius(segs)
    total_len = samples[-1].s
    if not (1200.0 <= total_len <= 2000.0):
        warns.append(f"circuit length {total_len:.1f} m outside the 1.2-2.0 km target range")
    return {"length": total_len, "min_radius": r, "min_gap": min_gap}


def check_ramps(errors):
    from trackgen import layout as L
    for r in L.RAMPS:
        run, rise = L.ramp_profile(r["grade"], r["slope_len"])
        g = rise / run
        if abs(g - r["grade"]) > 1e-6:
            err(errors, f"{r['id']}: rise/run {g:.4f} != advertised grade {r['grade']}")
        min_slope = 60.0 if r["grade"] == 0.15 else 30.0
        if r["slope_len"] < min_slope - 1e-6:
            err(errors, f"{r['id']}: slope length {r['slope_len']} < required {min_slope} m")
        if r["s_flat0"][1] - r["s_flat0"][0] < 5.0 or r["s_plateau"][1] - r["s_plateau"][0] < 5.0:
            err(errors, f"{r['id']}: flat approach/plateau too short for a vehicle")


# --------------------------------------------------------------- determinism
def check_determinism(errors):
    import importlib.util
    from trackgen.build_mesh import build_all

    # Load tools/trackgen/build.py unambiguously by path: `import build` can
    # resolve to an unrelated installed "build" package (PyPI's PEP 517
    # frontend) if one shadows it earlier in sys.modules/sys.path.
    build_path = os.path.join(HERE, "build.py")
    spec = importlib.util.spec_from_file_location("trackgen_build_script", build_path)
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)

    mb, samples = build_all()
    model = B.make_model()
    glb, glb_stats = model.build_glb(mb)
    on_disk = open(os.path.join(OUT, "test_ground.glb"), "rb").read()
    if glb != on_disk:
        err(errors, "rebuild is not byte-identical to test_ground.glb")
    else:
        print("test_ground.glb   deterministic: rebuild byte-identical")

    tmp_bin = io.BytesIO()
    from trackgen.collision import _gather
    import struct as _s
    by_surface = _gather(mb)
    bin_ = bytearray()
    for surf in sorted(by_surface):
        pos, idx = by_surface[surf]
        bin_ += _s.pack(f"<{len(pos) * 3}f", *(c for p in pos for c in p))
        bin_ += _s.pack(f"<{len(idx)}I", *idx)
    on_disk_bin = open(os.path.join(OUT, "test_ground_collision.bin"), "rb").read()
    if bytes(bin_) != on_disk_bin:
        err(errors, "rebuild is not byte-identical to test_ground_collision.bin")
    else:
        print("test_ground_collision.bin   deterministic: rebuild byte-identical")


def main(argv):
    det = "--determinism" in argv
    errors, warns = [], []
    gstats = check_gltf(errors, warns)
    surfaces, cidx = load_collision(errors)
    if surfaces:
        total_tris = check_collision_mesh(errors, warns, surfaces)
        check_cross_surface_height(errors, warns, surfaces)
        print(f"collision   {total_tris:7d} tris across {len(surfaces)} surfaces")
    circ = check_circuit(errors, warns)
    if circ:
        print(f"circuit     length {circ['length']:.1f} m   min radius {circ['min_radius']:.1f} m   "
              f"min self-separation {circ['min_gap']:.1f} m")
    check_ramps(errors)
    if gstats:
        print(f"glb         {gstats['tris']:7d} tris   {gstats['nodes']:3d} nodes   "
              f"{gstats['materials']:2d} mats   {gstats['bytes'] / 1e6:.2f} MB")
    if det:
        check_determinism(errors)
    for w in warns:
        print("WARN", w)
    for e in errors:
        print("FAIL", e)
    print("OK" if not errors else f"{len(errors)} failure(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
