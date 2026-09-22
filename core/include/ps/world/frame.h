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

#include <cstdint>

namespace ps {

// Additive (PLAN.md D10/P1.5a): identifies one link of one articulation,
// for a Frame anchored to an articulation instead of a Jolt body. Opaque
// to force elements - only World and the articulation subsystem interpret
// the fields.
struct ArticulationLinkRef {
    std::uint32_t articulation_index = 0;
    std::int32_t link_index = -1; // -1 == the articulation's own root
};

// Additive seam so world/frame.h does not need to depend on
// core/src/articulation (internal): implemented by World's articulation
// subsystem, injected into a Frame at construction. Kept minimal - just
// enough for Frame::world_pose/world_motion below.
class ILinkStateSource {
public:
    virtual ~ILinkStateSource() = default;
    [[nodiscard]] virtual Pose link_world_pose(ArticulationLinkRef ref) const = 0;
    [[nodiscard]] virtual Motion link_world_motion(ArticulationLinkRef ref) const = 0;
};

class Frame {
public:
    Frame() = default;
    explicit Frame(BodyId body, Pose local_offset = Pose::identity())
        : body_(body), local_offset_(local_offset) {}

    // Additive (PLAN.md D10/P1.5a): a frame anchored to an articulation
    // link instead of a rigid body - "force elements attach to link
    // Frames exactly like to body Frames" (PLAN.md P1.5a scope item 2).
    Frame(const ILinkStateSource& link_source, ArticulationLinkRef link, Pose local_offset = Pose::identity())
        : local_offset_(local_offset), link_source_(&link_source), link_(link) {}

    [[nodiscard]] BodyId body() const { return body_; } // invalid (default-constructed) for an articulation-link frame
    [[nodiscard]] const Pose& local_offset() const { return local_offset_; }

    [[nodiscard]] bool is_articulation_link() const { return link_source_ != nullptr; }
    [[nodiscard]] ArticulationLinkRef articulation_link() const { return link_; }

    [[nodiscard]] Pose world_pose(const IRigidBackend& backend) const {
        if (link_source_) return link_source_->link_world_pose(link_) * local_offset_;
        return backend.get_pose(body_) * local_offset_;
    }

    [[nodiscard]] Motion world_motion(const IRigidBackend& backend) const {
        Pose body_pose;
        Motion body_motion;
        if (link_source_) {
            body_pose = link_source_->link_world_pose(link_);
            body_motion = link_source_->link_world_motion(link_);
        } else {
            body_pose = backend.get_pose(body_);
            body_motion = backend.get_motion(body_);
        }
        // Offset from the body's origin to this frame's origin, in world axes.
        const Vec3 world_offset = body_pose.transform_direction(local_offset_.position);
        return Motion{body_motion.velocity_at(world_offset), body_motion.angular};
    }

private:
    BodyId body_;
    Pose local_offset_ = Pose::identity();
    const ILinkStateSource* link_source_ = nullptr;
    ArticulationLinkRef link_{};
};

} // namespace ps
