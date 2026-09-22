"""Writes test_ground.glb: node-per-part hierarchy + materials + geometry.

Adapted from tools/assetgen/assetgen/scene.py's Model._build_glb (same glTF
conventions -- see data/models/README.md and this package's own README
section) but trimmed for a static scene: no joints, sockets or lights, since
the test ground has none. Collision is exported separately (collision.py),
not as glb nodes -- see data/tracks/README.md for why.

Conventions:
- Authoring (MeshBuilder) and all of trackgen use the core frame, ISO 8855:
  x forward, y left, z up, metres. gltf(x, y, z) = iso(y, z, x) (a proper
  rotation, same basis change as tools/assetgen).
- One node per MeshBuilder "part" (e.g. "floor", "circuit", "ramp_15"),
  parented to a single root "test_ground" node at the world origin.
"""
import json
import os
import struct

from . import textures as T


def to_gltf(v):
    return (v[1], v[2], v[0])


def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class Material:
    def __init__(self, name, color=(0.8, 0.8, 0.8), metallic=0.0, roughness=0.6, tex=None, tile=1.0):
        self.name = name
        self.color = color
        self.metallic = metallic
        self.roughness = roughness
        self.tex = tex   # (generator_name, kwargs) looked up in trackgen.textures
        self.tile = tile if isinstance(tile, tuple) else (tile, tile)


class TrackModel:
    def __init__(self, model_id, title, description):
        self.id = model_id
        self.title = title
        self.description = description
        self.materials = {}
        self.parts = []        # ordered list of part names -> node "test_ground/<part>"

    def mat(self, name, **kw):
        if name not in self.materials:
            self.materials[name] = Material(name, **kw)
        return name

    def write(self, mb, out_path):
        glb, stats = self.build_glb(mb)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "wb") as f:
            f.write(glb)
        return stats

    def build_glb(self, mb):
        parts = mb.parts()
        used_mats = sorted({m for (_, m) in mb.prims})
        for _, m in mb.prims:
            if m not in self.materials:
                raise ValueError(f"{self.id}: material '{m}' not defined")

        bin_ = bytearray()
        views, accessors = [], []

        def view(data, target=None):
            while len(bin_) % 4:
                bin_.append(0)
            v = {"buffer": 0, "byteOffset": len(bin_), "byteLength": len(data)}
            if target:
                v["target"] = target
            bin_.extend(data)
            views.append(v)
            return len(views) - 1

        def acc(data_list, comp, typ, count, target, minmax=False):
            fmt = {"f": "<%df", "H": "<%dH", "I": "<%dI"}[comp]
            flat = [x for t in data_list for x in t] if typ != "SCALAR" else data_list
            raw = struct.pack(fmt % len(flat), *flat)
            a = {"bufferView": view(raw, target), "componentType": {"f": 5126, "H": 5123, "I": 5125}[comp],
                 "count": count, "type": typ}
            if minmax:
                n = len(data_list[0])
                a["min"] = [min(t[k] for t in data_list) for k in range(n)]
                a["max"] = [max(t[k] for t in data_list) for k in range(n)]
            accessors.append(a)
            return len(accessors) - 1

        images, textures_, tex_cache, sets = [], [], {}, {}

        def texture(kind, gen, kwargs):
            key = (gen, json.dumps(kwargs, sort_keys=True), kind)
            if key in tex_cache:
                return tex_cache[key]
            setkey = (gen, json.dumps(kwargs, sort_keys=True))
            if setkey not in sets:
                sets[setkey] = getattr(T, gen)(**kwargs)
            img = sets[setkey].get(kind)
            if img is None:
                tex_cache[key] = None
                return None
            images.append({"bufferView": view(T.png(img)), "mimeType": "image/png",
                           "name": f"{gen}_{kind}_{len(images)}"})
            textures_.append({"source": len(images) - 1, "sampler": 0})
            tex_cache[key] = len(textures_) - 1
            return tex_cache[key]

        mat_index, mats_json = {}, []
        for name in used_mats:
            m = self.materials[name]
            pbr = {"metallicFactor": m.metallic, "roughnessFactor": m.roughness}
            col = [srgb_to_linear(c) for c in m.color] + [1.0]
            if m.tex:
                gen, kw = m.tex
                tb = texture("base", gen, kw)
                if tb is not None:
                    pbr["baseColorTexture"] = {"index": tb}
                    col = [1.0, 1.0, 1.0, 1.0]
                tm = texture("mr", gen, kw)
                if tm is not None:
                    pbr["metallicRoughnessTexture"] = {"index": tm}
                    pbr["metallicFactor"] = 1.0
                    pbr["roughnessFactor"] = 1.0
            pbr["baseColorFactor"] = col
            mat_index[name] = len(mats_json)
            mats_json.append({"name": name, "pbrMetallicRoughness": pbr})

        # One mesh-node per part, parented to a root "test_ground" node.
        meshes, nodes_json = [], []
        nodes_json.append({"name": self.id, "children": list(range(1, 1 + len(parts)))})
        tri_total = 0
        for part in parts:
            prims = []
            for (prt, mname), p in sorted(mb.prims.items()):
                if prt != part or not p.idx:
                    continue
                m = self.materials[mname]
                pos = [to_gltf(v) for v in p.pos]
                nrm = [to_gltf(v) for v in p.nrm]
                uv = [(u / m.tile[0], v / m.tile[1]) for (u, v) in p.uv]
                comp = "H" if len(pos) < 65536 else "I"
                prims.append({
                    "attributes": {
                        "POSITION": acc(pos, "f", "VEC3", len(pos), 34962, minmax=True),
                        "NORMAL": acc(nrm, "f", "VEC3", len(nrm), 34962),
                        "TEXCOORD_0": acc(uv, "f", "VEC2", len(uv), 34962),
                    },
                    "indices": acc(p.idx, comp, "SCALAR", len(p.idx), 34963),
                    "material": mat_index[mname],
                })
                tri_total += len(p.idx) // 3
            nj = {"name": part or "root"}
            if prims:
                meshes.append({"name": part, "primitives": prims})
                nj["mesh"] = len(meshes) - 1
            nodes_json.append(nj)

        gltf = {
            "asset": {"version": "2.0", "generator": "physics_sim tools/trackgen",
                      "extras": {"title": self.title, "frame": "glTF: +Y up, +Z forward, +X left",
                                 "collision": self.id + "_collision.bin/.json (see test_ground.json)"}},
            "scene": 0,
            "scenes": [{"name": self.id, "nodes": [0]}],
            "nodes": nodes_json,
            "meshes": meshes,
            "materials": mats_json,
            "accessors": accessors,
            "bufferViews": views,
            "buffers": [{"byteLength": 0}],
        }
        if images:
            gltf["images"] = images
            gltf["textures"] = textures_
            gltf["samplers"] = [{"magFilter": 9729, "minFilter": 9987, "wrapS": 10497, "wrapT": 10497}]
        while len(bin_) % 4:
            bin_.append(0)
        gltf["buffers"][0]["byteLength"] = len(bin_)
        js = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
        while len(js) % 4:
            js += b" "
        out = bytearray()
        total = 12 + 8 + len(js) + 8 + len(bin_)
        out += struct.pack("<III", 0x46546C67, 2, total)
        out += struct.pack("<II", len(js), 0x4E4F534A) + js
        out += struct.pack("<II", len(bin_), 0x004E4942) + bin_
        return bytes(out), {"triangles": tri_total, "bytes": len(out), "nodes": len(nodes_json),
                             "materials": len(mats_json), "images": len(images)}
