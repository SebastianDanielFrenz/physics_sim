// adapters/godot/src/render_interp.h — render-time pose interpolation
// (PLAN.md D3: "the adapter runs the sim on its own thread and reads
// interpolated snapshots").
//
// The sim thread runs at a fixed 240 Hz; Godot renders at whatever frame
// rate the host machine sustains (commonly higher, so most frames fall
// strictly between two ticks, but a slow frame can also span more than one
// tick). Reading the latest published tick's pose directly (what this
// adapter did before this file existed) therefore renders a staircase: the
// same pose for however many frames land before the next tick publishes,
// then a sudden jump - visible judder on a fast-moving body (measured:
// +-0.3 m frame-to-frame on the chassis at ~50 m/s, alternating between
// frames that saw a 1-tick vs. a 2-tick sim advance since the last frame).
//
// Fix: render at a continuous point in time ONE TICK BEHIND the sim thread,
// blending between the last two published poses instead of snapping to the
// latest one. This file holds exactly that math - Godot-free (plain ps::
// math types only, same style as origin_rebase.h/triple_buffer.h) so
// adapters/godot/tests can exercise it without linking godot-cpp, and so a
// future non-Godot adapter (a UE5 port) can reuse it unchanged.
//
// Where the two poses this blends between come from: PsSimulation's sim
// thread (post_step, ps_simulation.cpp) already keeps the previous tick's
// pose for every body/articulation link it exposes (a plain "shift the old
// latest into prev, store the new latest" per tick - see AdapterSnapshot in
// ps_simulation.h), so a Godot frame that lands between two ticks - or even
// spans a skipped tick under load - always has a well-defined (prev,
// latest) pair to blend, independent of exactly how many ticks ran between
// this frame and the last one.
//
// Teleports (PsSimulation::reset_vehicle, main.gd's zone keys): a real
// prev/latest blend across a teleport would render a smear sweeping across
// the map for one tick. The sim thread instead sets `prev := latest` for
// the teleported body on the tick the reset happens (see
// ps_simulation.cpp's post_step) - not a separate flag this header needs to
// know about, since interpolating between two IDENTICAL poses returns that
// pose at any alpha (see InterpolatedPose's own comment).

#pragma once

#include "ps/math/pose.h"

#include <algorithm>
#include <chrono>

namespace ps_godot {

// A body/link's previous and latest published pose. `prev == latest` is the
// correct representation of "nothing to interpolate from" - the first tick
// after PsSimulation::start() (no real previous pose exists yet) and a
// teleported body's own tick (see this file's header comment) both use it,
// so interpolate_pose() below needs no separate "is this valid" branch:
// blending two identical poses at any alpha trivially returns that pose.
struct InterpolatedPose {
    ps::Pose prev;
    ps::Pose latest;
};

// Blend factor in [0, 1] for "how far the render clock (`now`) has moved
// past the moment the LATEST tick was published (`publish_time`), as a
// fraction of one sim tick period (`tick_dt_s`)". 0 renders exactly the
// previous tick's pose, 1 renders exactly the latest tick's pose.
//
// Godot typically renders faster than the sim's 240 Hz, so `now` usually
// lands inside [publish_time, publish_time + tick_dt_s) and this returns a
// value strictly between 0 and 1 - the whole point of this file (a
// continuous blend instead of the old staircase). Clamped at both ends:
// alpha > 1 (a slow frame, or the sim thread has stalled) renders the
// latest published pose rather than extrapolating past it; alpha < 0
// (clock skew, or called before any tick has published) renders the
// previous pose rather than going backwards past it.
//
// tick_dt_s <= 0 is a degenerate configuration (no sim running yet, or a
// misconfigured caller) and returns 1 - "no interpolation, just show the
// latest pose" is the safe fallback, not a divide-by-zero.
inline double render_interp_alpha(std::chrono::steady_clock::time_point publish_time, std::chrono::steady_clock::time_point now, double tick_dt_s) {
    if (tick_dt_s <= 0.0) return 1.0;
    const double elapsed_s = std::chrono::duration<double>(now - publish_time).count();
    return std::clamp(elapsed_s / tick_dt_s, 0.0, 1.0);
}

// Interpolates position (lerp) and orientation (ps::slerp - ps/math/quat.h,
// already shortest-path/hemisphere-correct: it negates the second
// quaternion when the dot product is negative before interpolating, so two
// poses that are numerically q and -q for the same rotation - which can
// happen tick to tick since a rigid body's orientation has no preferred
// sign - never produce a spurious long-way-round spin here).
//
// Deliberately plain ps:: double-precision math, called BEFORE the
// ISO->Godot basis change (frame_convert.h) - PLAN.md P2's "all conversion
// between the sim's ISO 8855 frame and Godot's happens in exactly one
// place" - so this stays upstream of that single conversion point and
// origin rebasing (origin_rebase.h) is unaffected: the caller interpolates
// two absolute ISO poses first, then converts the ONE result through
// iso_to_godot_transform, exactly like a non-interpolated pose always did.
inline ps::Pose interpolate_pose(const InterpolatedPose& p, double alpha) {
    return ps::Pose{
        ps::lerp(p.prev.position, p.latest.position, static_cast<ps::real>(alpha)),
        ps::slerp(p.prev.orientation, p.latest.orientation, static_cast<ps::real>(alpha)),
    };
}

} // namespace ps_godot
