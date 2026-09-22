"""Exports collision geometry as a compact binary sidecar plus a JSON index,
grouped by physical surface (not by visual part/feature) -- see
data/tracks/README.md for the format and the reasoning for this choice over
embedding col_<surface> nodes in the glb.

test_ground_collision.bin: for each surface, in the order listed by the JSON
index: float32 positions (x, y, z, core ISO frame -- no basis change, unlike
the glb) then uint32 indices *local to that surface's own vertex block*
(index 0 = the first vertex written for that surface). Everything is
little-endian, 4-byte aligned.
"""
import json
import math
import os
import struct

# Visual material -> physical collision surface (paint/markings are not a
# distinct physical surface; they collision-wise are their base surface).
SURFACE_OF_MATERIAL = {
    "grid_floor": "grid_floor",
    "asphalt": "asphalt",
    "marking": "asphalt",
    "kerb": "kerb",
    "grass": "grass",
    "dirt": "dirt",
    "low_mu": "low_mu",
}


def _gather(mb):
    """surface -> (positions[], indices[]) merged across every (part, material)
    that maps to it, each part's vertices kept in its own contiguous block
    (so indices stay simple and no vertex is shared/welded across parts --
    parts never need to share a vertex since each is its own closed mesh or
    meets another part only at matching, independently-generated boundary
    coordinates, see ribbon.py)."""
    out = {}
    for (part, material), prim in sorted(mb.prims.items()):
        if not prim.idx:
            continue
        surf = SURFACE_OF_MATERIAL[material]
        pos_list, idx_list = out.setdefault(surf, ([], []))
        base = len(pos_list)
        pos_list.extend(prim.pos)
        idx_list.extend(base + i for i in prim.idx)
    return out


def write_collision(mb, bin_path, json_path):
    by_surface = _gather(mb)
    bin_ = bytearray()
    index = []
    total_tris = 0
    for surf in sorted(by_surface):
        pos, idx = by_surface[surf]
        lo = [min(p[k] for p in pos) for k in range(3)]
        hi = [max(p[k] for p in pos) for k in range(3)]
        for v in pos:
            if not all(math.isfinite(c) for c in v):
                raise ValueError(f"collision surface {surf}: non-finite vertex {v}")
        vertex_offset = len(bin_)
        bin_ += struct.pack(f"<{len(pos) * 3}f", *(c for p in pos for c in p))
        index_offset = len(bin_)
        bin_ += struct.pack(f"<{len(idx)}I", *idx)
        tris = len(idx) // 3
        total_tris += tris
        index.append({
            "surface": surf, "vertex_offset": vertex_offset, "vertex_count": len(pos),
            "index_offset": index_offset, "index_count": len(idx), "triangle_count": tris,
            "aabb": {"min": [round(c, 4) for c in lo], "max": [round(c, 4) for c in hi]},
        })
    os.makedirs(os.path.dirname(bin_path), exist_ok=True)
    with open(bin_path, "wb") as f:
        f.write(bin_)
    doc = {
        "format": "physics_sim.collision/1",
        "file": os.path.basename(bin_path),
        "frame": "World east-north-up: x=east, y=north, z=up, metres (no basis change -- unlike the "
                 ".glb; same right-handed axes as ISO 8855, see data/tracks/README.md).",
        "layout": "Per surface, in this order: float32[vertex_count*3] positions (x,y,z interleaved), "
                  "then uint32[index_count] triangle indices local to that surface's own vertex block "
                  "(index 0 = its first vertex). Little-endian, 4-byte aligned throughout.",
        "surfaces": index,
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    return {"bytes": len(bin_), "total_triangles": total_tris, "surfaces": index}
