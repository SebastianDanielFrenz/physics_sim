"""Builds a ribbon surface (constant or varying material across its width)
along a path.Sample list, using assetgen.mesh's grid() primitive so every
ribbon is a single connected mesh: no internal seams, hence no cracks or
T-junctions within one feature (see data/tracks/README.md).

Cross-sections are purely horizontal offsets from the centerline (no banking):
world point at longitudinal sample `sm` and lateral offset `t` (metres, +t =
left) is `sm.point(t)` with an optional uniform cross_grade added to z.
"""
from .path import left_vec


def ribbon(mb, samples, width, part, material_of, n_lateral=2, closed=False, cross_grade=0.0):
    """material_of(sample, t_mid, i, j) -> material name | None (drop the quad).
    `t_mid` is the lateral offset (metres, +left) of the quad's centre.
    Returns the grid() emission records (for orient checks, bounds, etc.)."""
    half = width / 2.0
    cols = [-half + width * j / n_lateral for j in range(n_lateral + 1)]
    P = []
    for sm in samples:
        lx, ly, _ = left_vec(sm.heading)
        row = [(sm.x + t * lx, sm.y + t * ly, sm.z + cross_grade * t) for t in cols]
        P.append(row)

    def classify(i, j, center, normal):
        sm = samples[i % len(samples)]
        t_mid = (cols[j] + cols[j + 1]) / 2.0
        mat = material_of(sm, t_mid, i, j)
        if mat is None:
            return None
        return (part, mat)

    return mb.grid(P, material="_unused", part=part, closed_u=closed, closed_v=False, classify=classify)


def flat_patch(mb, center, length, width, part, material, heading_deg=0.0):
    """A single flat rectangular patch (grass/dirt): one quad via polygon()."""
    import math
    h = math.radians(heading_deg)
    fx, fy = math.cos(h), math.sin(h)
    lx, ly = -math.sin(h), math.cos(h)
    hl, hw = length / 2.0, width / 2.0
    cx, cy, cz = center
    pts = [
        (cx + hl * fx - hw * lx, cy + hl * fy - hw * ly, cz),
        (cx + hl * fx + hw * lx, cy + hl * fy + hw * ly, cz),
        (cx - hl * fx + hw * lx, cy - hl * fy + hw * ly, cz),
        (cx - hl * fx - hw * lx, cy - hl * fy - hw * ly, cz),
    ]
    return mb.polygon(pts, material, part, want_normal=(0.0, 0.0, 1.0))
