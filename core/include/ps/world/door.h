// ps/world/door.h - engine-neutral moving-door driver (kinematic bodies).
//
// A door is a Kinematic body (BodyDesc::motion = Kinematic) moved along a
// fixed path by a commanded fraction 0 (closed) .. 1 (open). DoorDriver is a
// plain value type: given the sim tick it produces the next target pose,
// allocation-free and deterministic (closed-form, no accumulated trig), and
// it holds still - reporting `blocked` - when moving into the next pose would
// bury a dynamic body deeper than DoorDesc::max_penetration_m. The overlap
// test is a callable the caller supplies (World uses collide_shape_into), so
// this header has no backend, Godot or World dependency.
//
// Kinds: Hinged rotates about a pivot/axis by open_angle_rad; Sliding
// translates by `slide`. A roller door is a rigid Sliding door (the rolled-up
// slats are not modelled). `speed` is rad/s for Hinged and m/s for Sliding.

#pragma once

#include "ps/backend/shape_desc.h"
#include "ps/math/pose.h"
#include "ps/world/ids.h"

#include <cmath>
#include <stdexcept>

namespace ps {

enum class DoorKind { Hinged, Sliding };

struct DoorDesc {
    DoorKind kind = DoorKind::Sliding;
    Pose closed = Pose::identity();      // the body's pose when fraction == 0
    // Sliding: world-space displacement closed -> open.
    Vec3 slide{0.0, 0.0, 0.0};
    // Hinged: world-space hinge point and axis; open pose = closed rotated about it by open_angle_rad.
    Vec3 pivot{0.0, 0.0, 0.0};
    Vec3 axis{0.0, 0.0, 1.0};
    real open_angle_rad = 0.0;
    real speed = 0.5;                    // m/s (Sliding) or rad/s (Hinged); > 0
    // Config, not code: the door holds when a dynamic body would be overlapped deeper than this at the next pose.
    // A kinematic door has unlimited force, so penetration depth is the only meaningful limit; it also bounds how
    // far a free body can be shoved before the door stops (about this value plus one tick of travel).
    real max_penetration_m = 0.02;
    // Bodies heavier than this are never pushed (a car or tractor in the way): they block at the much tighter
    // unpushable_penetration_m, so the door stops at first contact instead of shoving them. Lighter bodies (crates,
    // people-sized props) are pushed at door speed up to max_penetration_m. Default: push anything.
    real max_push_mass_kg = 1.0e30;
    real unpushable_penetration_m = 0.001;
    // The door body's collision shape, posed at the body pose (used for the overlap query at the next pose).
    ShapeDesc probe_shape = BoxShape{};
};

struct DoorId {
    id_index_t index = 0;
    id_generation_t generation = 0;
    [[nodiscard]] constexpr bool valid() const { return generation != 0; }
    constexpr bool operator==(const DoorId&) const = default;
};

struct DoorState {
    real fraction = 0.0;   // current position, 0 closed .. 1 open
    real command = 0.0;    // commanded position, 0..1
    bool moving = false;   // a move was issued this tick
    bool blocked = false;  // wanted to move but a body is in the way; holding
    BodyId blocker{};      // deepest blocking body while blocked
    real blocker_penetration_m = 0.0;
};

// Result of the overlap callable: the worst overlap of the door shape with any dynamic body. `excess` is the overlap
// depth minus that body's limit (max_penetration_m for a pushable body, unpushable_penetration_m otherwise); the door
// is blocked when excess > 0. No overlapping body: excess = -1e30.
struct DoorProbe {
    real excess = -1.0e30;
    real penetration = 0.0;
    BodyId body{};
};

// What the caller must do with the backend after DoorDriver::tick.
struct DoorTick {
    bool move = false;   // call move_kinematic(target)
    bool stop = false;   // call stop_kinematic (door just came to rest)
    Pose target = Pose::identity();
};

class DoorDriver {
public:
    DoorDriver() = default;
    explicit DoorDriver(const DoorDesc& desc) : desc_(desc) { validate(desc); }

    static void validate(const DoorDesc& d) {
        if (!(d.speed > 0.0) || !std::isfinite(d.speed)) throw std::invalid_argument("DoorDesc: speed must be > 0");
        if (!(d.max_penetration_m >= 0.0) || !std::isfinite(d.max_penetration_m)) throw std::invalid_argument("DoorDesc: max_penetration_m must be >= 0");
        if (!(d.unpushable_penetration_m >= 0.0) || !(d.max_push_mass_kg > 0.0)) throw std::invalid_argument("DoorDesc: bad push limits");
        if (d.kind == DoorKind::Sliding && !(d.slide.length() > 0.0)) throw std::invalid_argument("DoorDesc: sliding door needs a non-zero slide");
        if (d.kind == DoorKind::Hinged) {
            if (!(d.axis.length() > 0.0)) throw std::invalid_argument("DoorDesc: hinged door needs a non-zero axis");
            if (!(std::abs(d.open_angle_rad) > 0.0) || !std::isfinite(d.open_angle_rad)) throw std::invalid_argument("DoorDesc: hinged door needs a non-zero open_angle_rad");
        }
    }

    [[nodiscard]] const DoorDesc& desc() const { return desc_; }

    // Path length in m (Sliding) or rad (Hinged).
    [[nodiscard]] real travel() const {
        return desc_.kind == DoorKind::Sliding ? desc_.slide.length() : std::abs(desc_.open_angle_rad);
    }

    // Body pose at fraction f (clamped to 0..1). Closed form, so there is no drift.
    [[nodiscard]] Pose pose_at(real f) const {
        f = f < 0.0 ? 0.0 : (f > 1.0 ? 1.0 : f);
        if (desc_.kind == DoorKind::Sliding) return Pose{desc_.closed.position + desc_.slide * f, desc_.closed.orientation};
        const Quat q = Quat::from_axis_angle(desc_.axis, desc_.open_angle_rad * f);
        return Pose{desc_.pivot + q.rotate(desc_.closed.position - desc_.pivot), (q * desc_.closed.orientation).normalized()};
    }

    // Next fraction toward `command`, limited to speed*dt of path.
    [[nodiscard]] real approach(real f, real command, real dt) const {
        const real step = desc_.speed * dt / travel();
        const real d = command - f;
        if (d > step) return f + step;
        if (d < -step) return f - step;
        return command;
    }

    // One sim tick. `probe(pose)` returns the deepest overlap (DoorProbe) of the door shape at `pose` with bodies the
    // door must not push (see DoorProbe). Called at most twice, only when the door wants to move. No allocation here.
    template <class Probe>
    DoorTick tick(DoorState& s, real dt, Probe&& probe) const {
        DoorTick out;
        const real f = s.fraction;
        const real fn = approach(f, s.command, dt);
        if (fn == f) {
            s.blocked = false;
            s.blocker = BodyId{};
            s.blocker_penetration_m = 0.0;
            out.stop = s.moving;
            s.moving = false;
            return out;
        }
        const Pose target = pose_at(fn);
        const DoorProbe next = probe(target);
        if (next.excess > 0.0) {
            // Moving away from an obstacle we already overlap deeper than the limit is allowed (reopen after blocked).
            const DoorProbe now = probe(pose_at(f));
            if (!(next.excess < now.excess)) {
                s.blocked = true;
                s.blocker = next.body;
                s.blocker_penetration_m = next.penetration;
                out.stop = s.moving;
                s.moving = false;
                return out;
            }
        }
        s.fraction = fn;
        s.moving = true;
        s.blocked = false;
        s.blocker = BodyId{};
        s.blocker_penetration_m = 0.0;
        out.move = true;
        out.target = target;
        return out;
    }

private:
    DoorDesc desc_{};
};

} // namespace ps
