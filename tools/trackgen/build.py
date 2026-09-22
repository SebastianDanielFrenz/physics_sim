"""Build the M1 test ground: py -3.13 tools/trackgen/build.py

Writes data/tracks/test_ground/{test_ground.glb, test_ground_collision.bin,
test_ground_collision.json, test_ground.json, terrain.json,
terrain_golden.json} plus the shared data/surfaces/surfaces.json. Stdlib
only. Deterministic: same code -> byte-identical files (checked by
validate.py --determinism).
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "assetgen"))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "data", "tracks", "test_ground")
SURFACES_OUT = os.path.join(REPO, "data", "surfaces")

from trackgen.build_mesh import build_all           # noqa: E402
from trackgen.gltf_export import TrackModel          # noqa: E402
from trackgen import layout as L                     # noqa: E402
from trackgen import collision as C                  # noqa: E402
from trackgen import track_json as J                 # noqa: E402
from trackgen import terrain as T                    # noqa: E402


def make_model():
    m = TrackModel("test_ground", "M1 test ground",
                    "Procedural, deterministic test ground: flat floor, straight, skidpad, "
                    "surface patches, ramps, side slope, small circuit. See data/tracks/README.md.")
    m.mat("asphalt", color=(1, 1, 1), tex=("asphalt", {}), tile=5.0, roughness=0.8)
    m.mat("marking", color=(1, 1, 1), tex=("marking", {}), tile=1.0, roughness=0.6)
    m.mat("kerb", color=(1, 1, 1), tex=("kerb", {}), tile=1.5, roughness=0.75)
    m.mat("grass", color=(1, 1, 1), tex=("grass", {}), tile=3.0, roughness=0.95)
    m.mat("dirt", color=(1, 1, 1), tex=("dirt", {}), tile=3.0, roughness=0.92)
    m.mat("low_mu", color=(1, 1, 1), tex=("low_mu", {}), tile=4.0, roughness=0.18)
    return m


def write_surfaces_file():
    os.makedirs(SURFACES_OUT, exist_ok=True)
    doc = {
        "format": "physics_sim.surfaces/1",
        "description": "Physical surface parameters shared by every terrain source (see PLAN.md "
                       "section 6 / P11): data/tracks/test_ground/terrain.json's per-cell surface ids "
                       "and every feature mesh's per-triangle collision surface (test_ground_collision) "
                       "both reference the ids in this file rather than embedding their own copy, so a "
                       "later real-world land-cover source can map its classes onto the same ids/values.",
        "fields": {"lambda_mu": "Pacejka-style friction scale relative to dry asphalt = 1.0.",
                  "crr": "Rolling-resistance coefficient (dimensionless, F_roll = crr * N)."},
        "surfaces": L.SURFACES,
    }
    path = os.path.join(SURFACES_OUT, "surfaces.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    return path


def main():
    t0 = time.time()
    mb, samples = build_all()
    model = make_model()
    os.makedirs(OUT, exist_ok=True)
    write_surfaces_file()
    glb_stats = model.write(mb, os.path.join(OUT, "test_ground.glb"))

    col_stats = C.write_collision(mb, os.path.join(OUT, "test_ground_collision.bin"),
                                   os.path.join(OUT, "test_ground_collision.json"))

    doc = J.build_json(samples, glb_stats, col_stats)
    with open(os.path.join(OUT, "test_ground.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")

    spec, golden = T.write_terrain_files(OUT)

    print(f"test_ground   {glb_stats['triangles']:7d} tris  {glb_stats['nodes']:3d} nodes  "
          f"{glb_stats['materials']:3d} mats  {glb_stats['bytes'] / 1e6:6.2f} MB glb  "
          f"{col_stats['total_triangles']:7d} collision tris  {col_stats['bytes'] / 1e6:6.2f} MB col  "
          f"{time.time() - t0:5.1f} s")
    print(f"terrain       {golden['count']:4d} golden points   "
          f"flat zones: {len(spec['flat_zones']['zones'])}")


if __name__ == "__main__":
    main()
