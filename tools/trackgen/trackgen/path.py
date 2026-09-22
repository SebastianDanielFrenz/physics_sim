"""2D centerline paths built from line/arc segments, used for every ribbon-shaped
surface (straight, split-mu strip, skidpad ring, ramps, circuit).

Convention: core frame ISO 8855 (x forward, y left, z up). Heading is the angle
of the forward tangent from +x, positive turning left (CCW), matching a
right-handed z-up frame. A path carries no banking: the lateral (width) axis at
any station is purely horizontal, `left(heading) = (-sin(heading), cos(heading), 0)`,
so elevation is a separate z(s) profile applied after the 2D path is built ---
correct for the flat-cross-section ramps/roads this project needs (no banked
turns are required by PLAN.md's M1 test ground).

`CircuitSolver` closes a loop made of fixed-radius arcs and two free-length
straights by solving a 2x2 linear system for the two unknown lengths, so the
lap always returns exactly (to float precision) to its start point and heading
without hand-tuned numbers.
"""
import math


def left_vec(heading):
    return (-math.sin(heading), math.cos(heading), 0.0)


def fwd_vec(heading):
    return (math.cos(heading), math.sin(heading), 0.0)


class Sample:
    __slots__ = ("s", "x", "y", "z", "heading", "curvature")

    def __init__(self, s, x, y, heading, curvature, z=0.0):
        self.s, self.x, self.y, self.z, self.heading, self.curvature = s, x, y, z, heading, curvature

    def point(self, lateral=0.0):
        lx, ly, _ = left_vec(self.heading)
        return (self.x + lateral * lx, self.y + lateral * ly, self.z)


class Segment:
    """A named piece of a path: a straight ('line') or an arc, with the s-range
    it occupies once built, filled in by PathBuilder.build()."""

    def __init__(self, kind, length=None, radius=None, angle_deg=None, name=""):
        self.kind = kind          # "line" | "arc"
        self.length = length      # None => unknown, solved by CircuitSolver
        self.radius = radius
        self.angle_deg = angle_deg
        self.name = name
        self.s0 = self.s1 = None  # filled by build()

    def arc_length(self):
        if self.kind == "line":
            return self.length
        return abs(math.radians(self.angle_deg)) * self.radius

    def end_heading(self, h0):
        if self.kind == "line":
            return h0
        return h0 + math.radians(self.angle_deg)

    def displacement(self, h0):
        """2D displacement (dx, dy) traveling this segment starting at heading h0.

        For an arc, the centre sits one radius to the left (k>0) or right (k<0)
        of the start point; the point at any heading h is centre - sign*r*left(h)
        (verified against the textbook case h0=0, k=+90deg, r -> end (r, r))."""
        if self.kind == "line":
            return self.length * math.cos(h0), self.length * math.sin(h0)
        k = math.radians(self.angle_deg)  # signed turn
        r = self.radius
        if abs(k) < 1e-12:
            return 0.0, 0.0
        sign = 1.0 if k > 0 else -1.0
        ccx = sign * r * (-math.sin(h0))
        ccy = sign * r * math.cos(h0)
        h1 = h0 + k
        ex = ccx - sign * r * (-math.sin(h1))
        ey = ccy - sign * r * math.cos(h1)
        return ex, ey


class CircuitSolver:
    """Solve the lengths of the segments named `unknowns` (exactly two) so the
    segment chain closes exactly onto (0, 0) heading 0 after all of them."""

    def __init__(self, segments, unknown_names):
        self.segments = segments
        self.unknown_names = unknown_names

    def solve(self):
        unk = [s for s in self.segments if s.name in self.unknown_names]
        assert len(unk) == 2, f"need exactly 2 unknown segments, got {len(unk)}"
        # Single pass: heading only evolves through arcs, so it does not depend
        # on the two unresolved lengths. Record the entry heading of each
        # unknown segment, and accumulate the fixed (known-segment) displacement.
        h = 0.0
        cx = cy = 0.0
        heads = {}
        for seg in self.segments:
            if seg.name in self.unknown_names:
                heads[seg.name] = h
            else:
                dx, dy = seg.displacement(h)
                cx += dx
                cy += dy
            h = seg.end_heading(h)
        total_turn = h  # heading after the full loop; must be a multiple of 2*pi
        assert abs(math.sin(total_turn)) < 1e-9 and math.cos(total_turn) > 0.999, \
            f"segment angles do not sum to a full turn (ended at {math.degrees(total_turn):.3f} deg)"
        h1 = heads[unk[0].name]
        h2 = heads[unk[1].name]
        # [cos h1  cos h2] [L1]   [-cx]
        # [sin h1  sin h2] [L2] = [-cy]
        det = math.cos(h1) * math.sin(h2) - math.cos(h2) * math.sin(h1)
        assert abs(det) > 1e-9, "the two free straights are parallel; cannot solve closure"
        L1 = (-cx * math.sin(h2) + cy * math.cos(h2)) / det
        L2 = (cx * math.sin(h1) - cy * math.cos(h1)) / det
        assert L1 > 1.0 and L2 > 1.0, f"closure solved to a non-physical length ({L1:.1f}, {L2:.1f})"
        unk[0].length = L1
        unk[1].length = L2
        return L1, L2


def build_samples(segments, step, x0=0.0, y0=0.0, heading0=0.0):
    """Sample a fully-resolved (all lengths known) segment chain at ~`step`
    spacing. Returns a list of Sample (z left at 0; apply an elevation profile
    separately) and fills each segment's [s0, s1) range."""
    samples = []
    s = 0.0
    x, y, h = x0, y0, math.radians(heading0)
    samples.append(Sample(s, x, y, h, 0.0))
    for seg in segments:
        seg.s0 = s
        length = seg.arc_length()
        n = max(1, round(length / step))
        if seg.kind == "line":
            d = length / n
            for _ in range(n):
                x += d * math.cos(h)
                y += d * math.sin(h)
                s += d
                samples.append(Sample(s, x, y, h, 0.0))
        else:
            k_total = math.radians(seg.angle_deg)
            dtheta = k_total / n
            d = seg.radius * abs(dtheta)
            curvature = (1.0 if seg.angle_deg > 0 else -1.0) / seg.radius
            for _ in range(n):
                # midpoint rule: advance heading by half the turn, step, then the rest
                h += dtheta / 2
                x += d * math.cos(h)
                y += d * math.sin(h)
                h += dtheta / 2
                s += d
                samples.append(Sample(s, x, y, h, curvature))
        seg.s1 = s
    return samples


def apply_elevation(samples, z_of_s):
    for sm in samples:
        sm.z = z_of_s(sm.s)
    return samples


def drop_closing_duplicate(samples, eps=1.0):
    """For a fully closed loop, build_samples's first and last sample coincide
    only up to the midpoint-rule integration error of sampling arcs (a few cm
    over a >1 km loop) -- the *analytic* closure (CircuitSolver) is exact, so
    this drops the redundant near-duplicate last vertex and lets grid()'s
    closed_u wraparound connect the second-to-last row directly back to the
    exact first vertex, rather than leaving a hairline sliver of near-zero-
    width quads at the seam. eps=1 m is generous relative to the sampling
    step (~2 m) but far below the distance to any genuinely different point."""
    a, b = samples[0], samples[-1]
    if math.hypot(b.x - a.x, b.y - a.y) < eps and abs(b.z - a.z) < eps:
        return samples[:-1]
    return samples


def full_circle(radius, center, step, ccw=True, name="ring"):
    """A closed circular path of the given centreline radius, centred at
    `center` (x, y), starting at the +x point heading tangentially."""
    x0, y0 = center[0] + radius, center[1]
    heading0 = 90.0 if ccw else -90.0
    seg = Segment("arc", radius=radius, angle_deg=360.0 if ccw else -360.0, name=name)
    samples = build_samples([seg], step=step, x0=x0, y0=y0, heading0=heading0)
    return drop_closing_duplicate(samples)
