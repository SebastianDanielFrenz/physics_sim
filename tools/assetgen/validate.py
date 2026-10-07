"""Structural checks for the generated models:  py tools/assetgen/validate.py [--determinism] [id ...]

Per model:
- .glb container: header, chunk lengths, JSON parse, buffer bounds of every
  bufferView/accessor, index ranges, finite positions, unit normals, embedded
  PNG signatures, POSITION min/max present.
- nodes: unique names, rest pose = translation only (no rotation/scale),
  mesh/children references valid.
- rig <-> glb: same node set and parents; pivots match the glTF translations;
  joint axes unit length, limits ordered; aim targets exist; sockets exist;
  light materials exist.
- wheels: each wheel_* node's mesh reaches the ground (min z within 1 cm of 0
  at rest) and its extent matches the rig radius within 3 %.
- budget: triangle count and file size.
--determinism rebuilds each model in memory and compares bytes with the file.
Exit code 1 on any failure.
"""
import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "data", "models")

BUDGET = {"car": (130_000, 6.5e6), "farm": (220_000, 8.5e6),
          "car_hyper": (185_000, 6.5e6)}  # physical inner skins/returns; other budgets unchanged
COMP = {5126: ("f", 4), 5125: ("I", 4), 5123: ("H", 2), 5121: ("B", 1)}
NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def iso(g):
    return (g[2], g[0], g[1])


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
    assert len(bin_) == gltf["buffers"][0]["byteLength"] or len(bin_) - gltf["buffers"][0]["byteLength"] < 4
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


def check_model(mid, errors, warns):
    def err(msg):
        errors.append(f"{mid}: {msg}")

    gpath = os.path.join(OUT, mid, mid + ".glb")
    rpath = os.path.join(OUT, mid, mid + ".rig.json")
    if not (os.path.exists(gpath) and os.path.exists(rpath)):
        err("missing .glb or .rig.json")
        return None
    gltf, bin_, size = read_glb(gpath)
    rig = json.load(open(rpath, encoding="utf-8"))
    nodes = gltf["nodes"]
    names = [n["name"] for n in nodes]
    if len(set(names)) != len(names):
        err("duplicate node names")
    parent = {}
    for i, n in enumerate(nodes):
        if "rotation" in n or "scale" in n or "matrix" in n:
            err(f"node {n['name']} has a non-identity rest rotation/scale")
        for c in n.get("children", []):
            if c >= len(nodes):
                err(f"node {n['name']} bad child index")
            parent[c] = i
    # world rest positions (translation-only hierarchy)
    world = {}

    def wpos(i):
        if i in world:
            return world[i]
        t = nodes[i].get("translation", [0, 0, 0])
        p = wpos(parent[i]) if i in parent else (0.0, 0.0, 0.0)
        world[i] = (p[0] + t[0], p[1] + t[1], p[2] + t[2])
        return world[i]

    tris = 0
    mesh_zmin = {}
    mesh_ext = {}
    for i, n in enumerate(nodes):
        if "mesh" not in n:
            continue
        wp = wpos(i)
        lo, hi = [1e9] * 3, [-1e9] * 3
        for p in gltf["meshes"][n["mesh"]]["primitives"]:
            at = p["attributes"]
            pos = accessor(gltf, bin_, at["POSITION"])
            acc = gltf["accessors"][at["POSITION"]]
            if "min" not in acc or "max" not in acc:
                err(f"{n['name']}: POSITION without min/max")
            nrm = accessor(gltf, bin_, at["NORMAL"])
            idx = accessor(gltf, bin_, p["indices"])
            if len(idx) % 3:
                err(f"{n['name']}: index count not a multiple of 3")
            if idx and max(idx) >= len(pos):
                err(f"{n['name']}: index out of range")
            tris += len(idx) // 3
            for v in pos:
                if not all(math.isfinite(c) for c in v):
                    err(f"{n['name']}: non-finite position")
                    break
                for k in range(3):
                    lo[k] = min(lo[k], v[k] + wp[k])
                    hi[k] = max(hi[k], v[k] + wp[k])
            bad = sum(1 for v in nrm if abs(math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2) - 1) > 1e-3)
            if bad:
                err(f"{n['name']}: {bad} non-unit normals")
            if p.get("material", -1) >= len(gltf.get("materials", [])):
                err(f"{n['name']}: bad material index")
        mesh_zmin[n["name"]] = lo[1]          # glTF Y = up
        mesh_ext[n["name"]] = (lo, hi)
    for im in gltf.get("images", []):
        v = gltf["bufferViews"][im["bufferView"]]
        if bin_[v["byteOffset"]:v["byteOffset"] + 8] != b"\x89PNG\r\n\x1a\n":
            err(f"image {im.get('name')} is not a PNG")

    # rig consistency
    rn = {n["name"]: n for n in rig["nodes"]}
    rs = {s["name"]: s for s in rig["sockets"]}
    if set(rn) | set(rs) != set(names):
        err(f"rig/glb node sets differ: {sorted(set(names) ^ (set(rn) | set(rs)))}")
    idx_of = {n: i for i, n in enumerate(names)}
    for name, e in list(rn.items()) + list(rs.items()):
        if name not in idx_of:
            continue
        i = idx_of[name]
        par = names[parent[i]] if i in parent else None
        if par != e["parent"]:
            err(f"{name}: rig parent {e['parent']} != glb parent {par}")
        piv = e.get("pivot", e.get("position"))
        g = iso(wpos(i))
        if max(abs(a - b) for a, b in zip(g, piv)) > 1e-4:
            err(f"{name}: rig pivot {piv} != glb {g}")
    for name, e in rn.items():
        j = e.get("joint")
        if not j:
            continue
        if j["type"] in ("revolute", "continuous", "prismatic"):
            a = j["axis"]
            if abs(math.sqrt(sum(c * c for c in a)) - 1) > 1e-6:
                err(f"{name}: joint axis not unit length {a}")
        if j["type"] in ("revolute", "prismatic") and not j["min"] < j["max"]:
            err(f"{name}: joint limits not ordered")
        if j["type"] == "revolute" and not (j["min"] <= 0 <= j["max"]):
            warns.append(f"{mid}: {name}: rest pose (0) outside joint limits")
        if j["type"] == "aim" and j["target"] not in idx_of:
            err(f"{name}: aim target {j['target']} missing")
        gj = nodes[idx_of[name]].get("extras", {}).get("joint")
        if not gj or gj["type"] != j["type"]:
            err(f"{name}: joint missing from glb extras")
    matnames = {m["name"] for m in gltf.get("materials", [])}
    for fn, l in rig["lights"].items():
        for m in l["materials"]:
            if m not in matnames:
                err(f"light {fn}: material {m} not in glb")
    # wheels on the ground, radius consistent
    for name, e in rn.items():
        w = e.get("wheel")
        if not w or name not in mesh_ext:
            continue
        lo, hi = mesh_ext[name]
        if abs(lo[1]) > 0.01 and not e.get("off_ground"):
            err(f"{name}: lowest point z={lo[1]:.4f} m, expected ~0 (tyre on the ground)")
        r_geom = (hi[1] - lo[1]) / 2
        if abs(r_geom - w["radius"]) > 0.03 * w["radius"]:
            err(f"{name}: mesh radius {r_geom:.3f} != rig radius {w['radius']:.3f}")
    kind = "farm" if mid in ("tractor", "trailer_tipper", "combine") else "car"
    tb, sb = BUDGET.get(mid,BUDGET[kind])
    if tris > tb:
        err(f"{tris} triangles > budget {tb}")
    if size > sb:
        err(f"{size / 1e6:.2f} MB > budget {sb / 1e6:.1f} MB")
    return {"tris": tris, "mb": size / 1e6, "nodes": len(nodes),
            "joints": sum(1 for e in rn.values() if e.get("joint")), "sockets": len(rs),
            "lights": len(rig["lights"])}


def main(argv):
    det = "--determinism" in argv
    ids = [a for a in argv if not a.startswith("--")]
    if not ids:
        ids = [m["id"] for m in json.load(open(os.path.join(OUT, "index.json"), encoding="utf-8"))]
    errors, warns = [], []
    for mid in ids:
        s = check_model(mid, errors, warns)
        if s:
            print(f"{mid:16s} {s['tris']:7d} tris {s['mb']:5.2f} MB {s['nodes']:3d} nodes "
                  f"{s['joints']:3d} joints {s['sockets']:2d} sockets {s['lights']:2d} light fns")
        if det:
            import importlib
            model = importlib.import_module(f"assetgen.vehicles.{mid}").build()
            glb, _ = model._build_glb()
            if glb != open(os.path.join(OUT, mid, mid + ".glb"), "rb").read():
                errors.append(f"{mid}: rebuild is not byte-identical to the file on disk")
            else:
                print(f"{mid:16s} deterministic: rebuild byte-identical")
    for w in warns:
        print("WARN", w)
    for e in errors:
        print("FAIL", e)
    print("OK" if not errors else f"{len(errors)} failure(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
