"""trackgen: procedural generator for the M1 test ground (data/tracks/test_ground).

Stdlib-only Python (like tools/assetgen). Imports geometry/texture helpers from
tools/assetgen where they are generic (vec transforms, MeshBuilder, noise) but
never modifies that package; anything track-specific lives here.
"""
