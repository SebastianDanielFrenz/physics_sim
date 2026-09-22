"""Renders coarse top-down heightmap previews of the terrain spec (stdlib
PNG writer, no dependencies): py -3.13 tools/trackgen/render_terrain_preview.py

Writes out/renders/terrain_heightmap_full.png (the whole 100 x 100 km world,
~100 m/pixel) and out/renders/terrain_heightmap_origin.png (a zoomed view
around the origin showing the main flat disc and its blend). Grey = 0 m,
darker = lower, brighter = higher; each image's own max |height| is printed
so the two are not on the same scale.
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "assetgen"))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
OUT = os.path.join(REPO, "out", "renders")

from trackgen import terrain as T  # noqa: E402


def main():
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    p1 = os.path.join(OUT, "terrain_heightmap_full.png")
    zmax1 = T.render_heightmap(p1, center=(0.0, 0.0), half_extent=T.WORLD_HALF_EXTENT_M, size=1024)
    print(f"{p1}   +-{T.WORLD_HALF_EXTENT_M / 1000:.0f} km, |z|max {zmax1:.1f} m   {time.time() - t0:.1f}s")

    t1 = time.time()
    p2 = os.path.join(OUT, "terrain_heightmap_origin.png")
    zmax2 = T.render_heightmap(p2, center=(0.0, 0.0), half_extent=6000.0, size=1024)
    print(f"{p2}   +-6 km, |z|max {zmax2:.2f} m   {time.time() - t1:.1f}s")


if __name__ == "__main__":
    main()
