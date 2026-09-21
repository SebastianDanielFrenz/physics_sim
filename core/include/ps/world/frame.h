// ps/world/frame.h — transform-and-velocity provider (PLAN.md section 5).
//
// Today a frame is "a rigid body plus a local offset". Every force element
// attaches to a Frame, never directly to a body - this is the seam PLAN.md
// section 6 keeps open for a future frame that is a node of a deformable
// structure instead. Frame never caches anything: world_pose/world_motion
// query the backend fresh every call.

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/math/pose.h"
#include "ps/world/ids.h"

namespace ps {

class Frame {
public:
    Frame() = default;
    explicit Frame(BodyId body, Pose local_offset = Pose::identity())
        : body_(body), local_offset_(local_offset) {}

    [[nodiscard]] BodyId body() const { return body_; }
    [[nodiscard]] const Pose& local_offset() const { return local_offset_; }

    [[nodiscard]] Pose world_pose(const IRigidBackend& backend) const {
        return backend.get_pose(body_) * local_offset_;
    }

    [[nodiscard]] Motion world_motion(const IRigidBackend& backend) const {
        const Pose body_pose = backend.get_pose(body_);
        const Motion body_motion = backend.get_motion(body_);
        // Offset from the body's origin to this frame's origin, in world axes.
        const Vec3 world_offset = body_pose.transform_direction(local_offset_.position);
        return Motion{body_motion.velocity_at(world_offset), body_motion.angular};
    }

private:
    BodyId body_;
    Pose local_offset_ = Pose::identity();
};

} // namespace ps
