// ps/math/pose.h — rigid transform (Pose) and velocity (Motion) pairs.
//
// Pose is the plain-old-data building block Frame (ps/world/frame.h) wraps
// with a backend query; kept separate so math/ has no dependency on world/.

#pragma once

#include "ps/math/quat.h"
#include "ps/math/vec3.h"

namespace ps {

struct Pose {
    Vec3 position = Vec3::zero();
    Quat orientation = Quat::identity();

    static Pose identity() { return Pose{}; }

    [[nodiscard]] Vec3 transform_point(const Vec3& local) const {
        return position + orientation.rotate(local);
    }
    [[nodiscard]] Vec3 transform_direction(const Vec3& local) const {
        return orientation.rotate(local);
    }
    [[nodiscard]] Pose operator*(const Pose& o) const {
        return Pose{transform_point(o.position), orientation * o.orientation};
    }
    [[nodiscard]] Pose inverse() const {
        const Quat inv_rot = orientation.inverse();
        return Pose{inv_rot.rotate(-position), inv_rot};
    }
};

// Linear velocity of the frame origin, angular velocity of the frame,
// both expressed in world axes (ISO 8855, PLAN.md D4).
struct Motion {
    Vec3 linear = Vec3::zero();
    Vec3 angular = Vec3::zero();

    // Velocity of a world-space point rigidly attached to this motion,
    // given the point's offset from the reference origin (world axes).
    [[nodiscard]] Vec3 velocity_at(const Vec3& world_offset_from_origin) const {
        return linear + cross(angular, world_offset_from_origin);
    }
};

// A force+torque pair, both about/at a common reference point (world axes).
// This is what IForceElement::evaluate produces per PLAN.md section 5.
struct Wrench {
    Vec3 force = Vec3::zero();
    Vec3 torque = Vec3::zero();

    constexpr Wrench operator+(const Wrench& o) const { return {force + o.force, torque + o.torque}; }
    constexpr Wrench& operator+=(const Wrench& o) { force += o.force; torque += o.torque; return *this; }
};

} // namespace ps
