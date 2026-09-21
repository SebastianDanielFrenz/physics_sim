"""Build the vehicle models: python tools/assetgen/build.py [model_id ...]

Writes data/models/<id>/<id>.glb and <id>.rig.json, plus data/models/index.json.
Stdlib only. Deterministic: same code -> byte-identical files.
"""
import importlib
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "data", "models")

MODELS = [
    "car_shitbox",
    "car_sedan",
    "car_hyper",
    "car_f1",
    "tractor",
    "trailer_tipper",
    "combine",
]


def main(argv):
    ids = argv or MODELS
    unknown = [i for i in ids if i not in MODELS]
    if unknown:
        sys.exit(f"unknown model(s): {unknown}; known: {MODELS}")
    for mid in ids:
        t0 = time.time()
        mod = importlib.import_module(f"assetgen.vehicles.{mid}")
        model = mod.build()
        stats = model.write(os.path.join(OUT, mid))
        print(f"{mid:16s} {stats['triangles']:7d} tris  {stats['nodes']:3d} nodes  "
              f"{stats['materials']:3d} mats  {stats['images']:3d} imgs  {stats['bytes'] / 1e6:6.2f} MB  "
              f"{time.time() - t0:5.1f} s")
    index = []
    for mid in MODELS:
        rig = os.path.join(OUT, mid, mid + ".rig.json")
        if os.path.exists(rig):
            with open(rig, encoding="utf-8") as f:
                title = json.load(f)["title"]
            index.append({"id": mid, "title": title, "glb": f"{mid}/{mid}.glb", "rig": f"{mid}/{mid}.rig.json"})
    with open(os.path.join(OUT, "index.json"), "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2)
        f.write("\n")


if __name__ == "__main__":
    main(sys.argv[1:])
