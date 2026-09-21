"""Model = node hierarchy + materials + geometry, written as .glb and .rig.json.

Conventions (see also data/models/README.md):
- Authoring and the .rig.json use the core frame, ISO 8855: x forward, y left,
  z up, metres, radians. Origin on the ground plane, centred between the
  axles (the trailer: centred on its axle group), at rest ride height.
- The .glb uses glTF's frame: +Y up, +Z forward, +X left, so
  gltf(x, y, z) = iso(y, z, x). This is a proper rotation (no mirroring).
- Every node's rest rotation is identity; a node's origin is its joint pivot.
  Moving parts are driven by rotating/translating a node about the joint axis
  given in its extras (glTF frame) or in the rig file (ISO frame).
- Lights are materials whose emission is off in the file. The rig names the
  materials per light function and the colour/strength to switch on.
"""
import json
import math
import os
import struct

from .mesh import MeshBuilder
from .vec import sub, add
from . import textures as T


def to_gltf(v):
    return (v[1], v[2], v[0])


def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class Material:
    def __init__(self, name, color=(0.8, 0.8, 0.8), alpha=1.0, metallic=0.0, roughness=0.5, tex=None,
                 tile=1.0, emissive=None, light=None, light_strength=4.0, blend=False, double=False,
                 normal_scale=1.0, role=None):
        self.name = name
        self.color = color          # sRGB 0..1; ignored (white) when tex bakes colour
        self.alpha = alpha
        self.metallic = metallic
        self.roughness = roughness
        self.tex = tex              # (generator_name, kwargs) from textures.py
        self.tile = tile if isinstance(tile, tuple) else (tile, tile)  # metres per texture repeat (u, v)
        self.emissive = emissive    # colour when the light is on
        self.light = light          # light function name, e.g. "brake"
        self.light_strength = light_strength
        self.blend = blend
        self.double = double or blend
        self.normal_scale = normal_scale
        self.role = role


class Node:
    def __init__(self, name, parent, pivot, joint=None, extras=None):
        self.name = name
        self.parent = parent
        self.pivot = tuple(float(x) for x in pivot)  # model frame (ISO), at rest
        self.joint = joint
        self.extras = extras or {}
        self.children = []


class Model:
    def __init__(self, model_id, title, description, origin_note):
        self.id = model_id
        self.title = title
        self.description = description
        self.origin_note = origin_note
        self.mb = MeshBuilder()
        self.materials = {}
        self.root = Node(model_id, None, (0, 0, 0))
        self.nodes = {model_id: self.root}
        self.order = [model_id]
        self.notes = []
        self.extra_rig = {}

    # ------------------------------------------------------------ authoring
    def mat(self, name, **kw):
        if name not in self.materials:
            self.materials[name] = Material(name, **kw)
        return name

    def node(self, name, parent=None, pivot=(0, 0, 0), joint=None, **extras):
        assert name not in self.nodes, name
        p = self.nodes[parent] if parent else self.root
        n = Node(name, p, pivot, joint, extras)
        p.children.append(n)
        self.nodes[name] = n
        self.order.append(name)
        return name

    def socket(self, name, parent, pos, note):
        return self.node(name, parent, pos, None, socket=True, note=note)

    @staticmethod
    def revolute(axis, lo_deg, hi_deg, note=""):
        return {"type": "revolute", "axis": axis, "min": math.radians(lo_deg), "max": math.radians(hi_deg),
                "note": note}

    @staticmethod
    def continuous(axis, note=""):
        return {"type": "continuous", "axis": axis, "note": note}

    @staticmethod
    def prismatic(axis, lo, hi, note=""):
        return {"type": "prismatic", "axis": axis, "min": lo, "max": hi, "note": note}

    @staticmethod
    def aim(target, target_point, stretch=False, note=""):
        """Visual-only link: keep the node's +x axis pointing at `target_point`
        (model frame at rest) carried by node `target`; optionally stretch along x."""
        return {"type": "aim", "target": target, "target_point": target_point, "stretch": stretch, "note": note}

    # --------------------------------------------------------------- export
    def world_part(self, name):
        return "" if name == self.id else name

    def write(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        glb, stats = self._build_glb()
        with open(os.path.join(out_dir, self.id + ".glb"), "wb") as f:
            f.write(glb)
        with open(os.path.join(out_dir, self.id + ".rig.json"), "w", encoding="utf-8") as f:
            json.dump(self._rig(), f, indent=2)
            f.write("\n")
        return stats

    def _check(self):
        known = (set(self.nodes) - {self.id}) | {""}
        for (part, _) in self.mb.prims:
            if part not in known:
                raise ValueError(f"{self.id}: geometry part '{part}' has no node")
        for (_, m) in self.mb.prims:
            if m not in self.materials:
                raise ValueError(f"{self.id}: material '{m}' not defined")

    def _build_glb(self):
        self._check()
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

        def acc(data_list, comp, typ, count, target, minmax=False, normalized=False):
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

        # Textures -> images (cached per generator + params)
        images, textures_, tex_cache = [], [], {}

        def texture(kind, gen, kwargs):
            key = (gen, json.dumps(kwargs, sort_keys=True), kind)
            if key in tex_cache:
                return tex_cache[key]
            setkey = (gen, json.dumps(kwargs, sort_keys=True))
            sets = texture.sets
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
        texture.sets = {}

        used_mats = sorted({m for (_, m) in self.mb.prims})
        mat_index = {}
        mats_json = []
        for name in used_mats:
            m = self.materials[name]
            pbr = {"metallicFactor": m.metallic, "roughnessFactor": m.roughness}
            col = [srgb_to_linear(c) for c in m.color] + [m.alpha]
            if m.tex:
                gen, kw = m.tex
                tb = texture("base", gen, kw)
                if tb is not None:
                    pbr["baseColorTexture"] = {"index": tb}
                    col = [1.0, 1.0, 1.0, m.alpha]
                tm = texture("mr", gen, kw)
                if tm is not None:
                    pbr["metallicRoughnessTexture"] = {"index": tm}
                    pbr["metallicFactor"] = 1.0
                    pbr["roughnessFactor"] = 1.0
            pbr["baseColorFactor"] = col
            mj = {"name": name, "pbrMetallicRoughness": pbr}
            if m.tex:
                tn = texture("normal", *m.tex)
                if tn is not None:
                    mj["normalTexture"] = {"index": tn, "scale": m.normal_scale}
            if m.blend:
                mj["alphaMode"] = "BLEND"
            if m.double:
                mj["doubleSided"] = True
            ex = {}
            if m.light:
                ex["light"] = m.light
                ex["emissive_on"] = list(m.emissive)
                ex["emissive_strength_on"] = m.light_strength
            if m.role:
                ex["role"] = m.role
            if ex:
                mj["extras"] = ex
            mat_index[name] = len(mats_json)
            mats_json.append(mj)

        # Meshes per node
        meshes, nodes_json = [], []
        node_index = {n: i for i, n in enumerate(self.order)}
        tri_total = 0
        for name in self.order:
            nd = self.nodes[name]
            part = self.world_part(name)
            prims = []
            for (prt, mname), p in sorted(self.mb.prims.items()):
                if prt != part or not p.idx:
                    continue
                m = self.materials[mname]
                pos = [to_gltf(sub(v, nd.pivot)) for v in p.pos]
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
            nj = {"name": name}
            parent_pivot = nd.parent.pivot if nd.parent else (0.0, 0.0, 0.0)
            t = to_gltf(sub(nd.pivot, parent_pivot))
            if any(abs(x) > 0 for x in t):
                nj["translation"] = list(t)
            if prims:
                meshes.append({"name": name, "primitives": prims})
                nj["mesh"] = len(meshes) - 1
            if nd.children:
                nj["children"] = [node_index[c.name] for c in nd.children]
            ex = dict(nd.extras)
            if nd.joint:
                j = dict(nd.joint)
                if "axis" in j:
                    j["axis"] = list(to_gltf(j["axis"]))
                if "target_point" in j:
                    j["target_point"] = list(to_gltf(j["target_point"]))
                ex["joint"] = j
            if ex:
                nj["extras"] = ex
            nodes_json.append(nj)

        gltf = {
            "asset": {"version": "2.0", "generator": "physics_sim tools/assetgen",
                      "extras": {"title": self.title, "frame": "glTF: +Y up, +Z forward, +X left",
                                 "rig": self.id + ".rig.json"}},
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

    def _rig(self):
        lo, hi = [1e9] * 3, [-1e9] * 3
        for p in self.mb.prims.values():
            for v in p.pos:
                for k in range(3):
                    lo[k] = min(lo[k], v[k])
                    hi[k] = max(hi[k], v[k])
        nodes, sockets = [], []
        for name in self.order:
            nd = self.nodes[name]
            b = self.mb.bounds(self.world_part(name))
            entry = {"name": name, "parent": nd.parent.name if nd.parent else None,
                     "pivot": [round(x, 5) for x in nd.pivot]}
            if nd.extras.get("socket"):
                sockets.append({"name": name, "parent": entry["parent"], "position": entry["pivot"],
                                "note": nd.extras.get("note", "")})
                continue
            if nd.joint:
                entry["joint"] = nd.joint
            if b:
                entry["bounds_local"] = {"min": [round(x - c, 4) for x, c in zip(b[0], nd.pivot)],
                                         "max": [round(x - c, 4) for x, c in zip(b[1], nd.pivot)]}
            for k, v in nd.extras.items():
                entry[k] = v
            nodes.append(entry)
        lights = {}
        for m in self.materials.values():
            if m.light and any(mm == m.name for (_, mm) in self.mb.prims):
                l = lights.setdefault(m.light, {"materials": [], "emissive_on": list(m.emissive),
                                                "emissive_strength_on": m.light_strength})
                l["materials"].append(m.name)
        return {
            "format": "physics_sim.rig/1",
            "model": self.id + ".glb",
            "title": self.title,
            "description": self.description,
            "frame": ("ISO 8855: x forward, y left, z up; metres; radians. The .glb itself uses glTF axes "
                      "(+Y up, +Z forward, +X left): gltf(x,y,z) = iso(y,z,x)."),
            "origin": self.origin_note,
            "rest_pose": "All node rotations are identity at rest; a node's origin is its joint pivot. "
                         "Joint values are offsets from the rest pose (0 = as modelled).",
            "overall_bounds": {"min": [round(x, 4) for x in lo], "max": [round(x, 4) for x in hi]},
            "nodes": nodes,
            "sockets": sockets,
            "lights": lights,
            **self.extra_rig,
            "notes": self.notes,
        }
