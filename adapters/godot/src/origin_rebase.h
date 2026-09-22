// adapters/godot/src/origin_rebase.h — floating-origin bookkeeping
// (PLAN.md P2: "Godot positions are relative to a floating origin near the
// camera, moved in large steps").
//
// Godot-free (plain ps:: math types only) so adapters/godot/tests can
// exercise it without linking godot-cpp. The actual basis change happens
// in frame_convert_core.h/frame_convert.h; this type only tracks WHERE the
// origin currently is and WHEN to move it.

#pragma once

#include "ps/math/vec3.h"

namespace ps_godot {

class OriginRebase {
public:
    // rebase_threshold_m: once the focus point (camera, or the tracked
    // vehicle) drifts this far from the current origin, the origin snaps to
    // the focus point's own position (not a grid-quantised step - nothing
    // in this adapter needs the origin to land on a fixed lattice, and
    // snapping exactly to the focus point maximises the distance before the
    // next rebase is needed). Default 500 m keeps camera-relative
    // coordinates always under 500 m in magnitude, comfortably inside
    // float32's exact-integer range (2^24 mm ~ 16.7 km) with wide margin.
    explicit OriginRebase(ps::real rebase_threshold_m = 500.0) : threshold_(rebase_threshold_m) {}

    // Call once per frame (or once per sim tick) with the focus point's
    // ABSOLUTE sim-space position (double precision, not yet basis-changed
    // or origin-relative). Returns true if the origin moved this call.
    bool update(const ps::Vec3& focus_absolute_m) {
        const ps::real dist = (focus_absolute_m - origin_).length();
        if (dist > threshold_) {
            origin_ = focus_absolute_m;
            return true;
        }
        return false;
    }

    [[nodiscard]] const ps::Vec3& origin() const { return origin_; }

    // Forces a specific origin (e.g. resetting to zero, or a test fixture).
    void set_origin(const ps::Vec3& origin_m) { origin_ = origin_m; }

private:
    ps::Vec3 origin_{};
    ps::real threshold_;
};

} // namespace ps_godot
