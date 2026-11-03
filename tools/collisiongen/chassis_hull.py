"""Chassis collision hull from a vehicle model (physics_sim.chassis_hull/1).

A dynamic chassis cannot carry a triangle mesh (JPH::MeshShape::MustBeStatic), so the
hull is a compound of convex pieces: the model's closed body nodes are cut into slices
along the car's x axis and every slice keeps only the support points of its vertices
in a fixed set of directions (an inner approximation of the slice's convex hull; Jolt
builds the exact hull of these points). Slices overlap by `overlap_m` so neighbouring
pieces leave no gap.

Frames. The rig is ISO 8855 (x forward, y left, z up) with its origin on the ground
below the wheel centres; the .glb uses glTF axes, gltf(x, y, z) = iso(y, z, x). The
hull is written in CHASSIS BODY coordinates: iso + offset, offset = the mean over the
wheels of (vehicle attachment_local - rig suspension pivot) - the alignment
racing_game's vehicle_visual.gd applies to the model, so hull and drawn car coincide.

Underbody. Points below the floor are raised onto it: floor = (full-bump ground plane)
+ floor_clearance_m between the axles, rising at floor_ramp beyond them. With the
wheels inside their travel the hull never meets flat ground, so the wheels alone carry
the car; the hull touches the road only past a bump stop, flipped, or on an obstacle.

Deterministic: stdlib + numpy, fixed direction set, output rounded to 0.1 mm.

    python tools/collisiongen/chassis_hull.py car_hyper
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]

# Per model: the vehicle file the hull is aligned to and which glb nodes form the body.
# The rear wing (wing_lift, wing_flap) moves and is thin: in a slice hull it would fill
# the space between the engine cover and the wing, so it is left out; so is the
# steering wheel (inside the shell anyway).
MODELS = {
    "car_hyper": {
        "glb": "data/models/car_hyper/car_hyper.glb",
        "rig": "data/models/car_hyper/car_hyper.rig.json",
        "vehicle": "data/vehicles/car_hyper.json",
        "nodes": ["car_hyper", "door_L", "door_R", "frunk", "engine_cover", "splitter", "flap_front_L",
                  "flap_front_R", "diffuser"],
        "slices": 10,
        "overlap_m": 0.04,
        "floor_clearance_m": 0.02,
        "floor_ramp": 0.08,
        "out": "data/collision/car_hyper_hull.json",
    },
}

COMPONENT_DTYPES = {5126: np.float32}


def load_glb(path: Path):
    data = path.read_bytes()
    json_len = struct.unpack("<I", data[12:16])[0]
    gltf = json.loads(data[20:20 + json_len])
    bin_start = 20 + json_len + 8
    return gltf, data[bin_start:]


def read_positions(gltf, binary, accessor_index):
    acc = gltf["accessors"][accessor_index]
    view = gltf["bufferViews"][acc["bufferView"]]
    if acc["componentType"] != 5126 or acc["type"] != "VEC3":
        raise ValueError("POSITION accessor is not float VEC3")
    offset = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride", 12)
    count = acc["count"]
    raw = np.frombuffer(binary, dtype=np.uint8, count=stride * (count - 1) + 12, offset=offset)
    rows = np.lib.stride_tricks.as_strided(raw, shape=(count, 12), strides=(stride, 1))
    return np.ascontiguousarray(rows).view(np.float32).reshape(count, 3).astype(np.float64)


def node_world_translation(gltf, index, parents):
    t = np.zeros(3)
    while index is not None:
        node = gltf["nodes"][index]
        if "rotation" in node or "scale" in node or "matrix" in node:
            raise ValueError(f"node {node.get('name')} has a non-translation transform at rest")
        t += np.array(node.get("translation", [0.0, 0.0, 0.0]))
        index = parents.get(index)
    return t


def body_points_iso(gltf, binary, names):
    parents = {c: i for i, n in enumerate(gltf["nodes"]) for c in n.get("children", [])}
    by_name = {n.get("name"): i for i, n in enumerate(gltf["nodes"])}
    chunks = []
    for name in names:
        index = by_name[name]
        node = gltf["nodes"][index]
        if "mesh" not in node:
            continue
        t = node_world_translation(gltf, index, parents)
        for prim in gltf["meshes"][node["mesh"]]["primitives"]:
            p = read_positions(gltf, binary, prim["attributes"]["POSITION"]) + t
            chunks.append(p[:, [2, 0, 1]])  # gltf (x, y, z) -> iso (z, x, y)
    return np.concatenate(chunks)


def fibonacci_directions(n):
    golden = math.pi * (3.0 - math.sqrt(5.0))
    out = []
    for i in range(n):
        z = 1.0 - 2.0 * (i + 0.5) / n
        r = math.sqrt(max(0.0, 1.0 - z * z))
        out.append((r * math.cos(golden * i), r * math.sin(golden * i), z))
    # The axis-aligned extremes always: a slice's flat faces stay exact.
    out += [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    return np.array(out)


def support_points(points, directions):
    idx = sorted(set(int(np.argmax(points @ d)) for d in directions))
    return points[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model", choices=sorted(MODELS))
    ap.add_argument("--directions", type=int, default=160)
    args = ap.parse_args()
    cfg = MODELS[args.model]

    gltf, binary = load_glb(ROOT / cfg["glb"])
    rig = json.loads((ROOT / cfg["rig"]).read_text())
    vehicle = json.loads((ROOT / cfg["vehicle"]).read_text())
    pivots = {n["name"].split("_", 1)[1]: n["pivot"] for n in rig["nodes"] if n["name"].startswith("susp_")}

    offsets = []
    full_bump_ground = []
    axle_x = []
    for w in vehicle["wheels"]:
        attach = np.array(w["attachment_local"], dtype=float)
        offsets.append(attach - np.array(pivots[w["name"]], dtype=float))
        radius = next(n["wheel"]["radius"] for n in rig["nodes"] if n["name"] == "wheel_" + w["name"])
        if w["suspension"]["axis"] != [0, 0, 1]:
            raise ValueError("suspension axis is not body +z")
        full_bump_ground.append(attach[2] + w["suspension"]["travel_max"] - radius)
        axle_x.append(attach[0])
    offset = np.mean(offsets, axis=0)
    floor_z = max(full_bump_ground) + cfg["floor_clearance_m"]
    x_front, x_rear = max(axle_x), min(axle_x)

    pts = body_points_iso(gltf, binary, cfg["nodes"]) + offset
    beyond = np.maximum(pts[:, 0] - x_front, 0.0) + np.maximum(x_rear - pts[:, 0], 0.0)
    floor = floor_z + cfg["floor_ramp"] * beyond
    pts[:, 2] = np.maximum(pts[:, 2], floor)

    directions = fibonacci_directions(args.directions)
    x_min, x_max = float(pts[:, 0].min()), float(pts[:, 0].max())
    edges = np.linspace(x_min, x_max, cfg["slices"] + 1)
    pieces = []
    for i in range(cfg["slices"]):
        lo, hi = edges[i] - cfg["overlap_m"], edges[i + 1] + cfg["overlap_m"]
        sel = pts[(pts[:, 0] >= lo) & (pts[:, 0] <= hi)]
        sup = support_points(sel, directions)
        pieces.append([[round(float(c), 4) for c in p] for p in sup])

    out = {
        "format": "physics_sim.chassis_hull/1",
        "name": args.model,
        "source": (
            f"Generated by tools/collisiongen/chassis_hull.py from {cfg['glb']} (nodes {', '.join(cfg['nodes'])}; "
            f"rear wing and steering wheel left out). Chassis body coordinates (ISO, origin = the chassis body origin "
            f"of {cfg['vehicle']}): rig + offset [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}], the mean of "
            f"attachment_local - suspension pivot over the wheels, the same alignment as racing_game's "
            f"vehicle_visual.gd. {cfg['slices']} slices along x overlapping by {cfg['overlap_m']} m, each the support "
            f"points of its vertices in {len(directions)} directions. Underbody raised to z = {floor_z:.4f} m "
            f"(the full-bump ground plane + {cfg['floor_clearance_m']} m [AS]) between the axles, rising "
            f"{cfg['floor_ramp']} m/m beyond them [AS], so the hull clears flat ground over the whole wheel travel."
        ),
        "convex_radius_m": 0.02,
        "pieces": pieces,
    }
    path = ROOT / cfg["out"]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1) + "\n", newline="\n")
    print(f"{path}: {len(pieces)} pieces, {sum(len(p) for p in pieces)} points, "
          f"x [{x_min:.3f}, {x_max:.3f}], floor {floor_z:.4f}, offset {offset.round(6).tolist()}")


if __name__ == "__main__":
    main()
