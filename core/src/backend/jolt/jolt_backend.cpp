// core/src/backend/jolt/jolt_backend.cpp — the only P1 IRigidBackend
// implementation (PLAN.md D2). Every Jolt type stays confined to this file
// (plus the internal jolt_job_system.h bridge); ps/backend/i_rigid_backend.h
// exposes only create_jolt_backend() returning the abstract interface.

#include "ps/backend/i_rigid_backend.h"
#include "../../jobs/jolt_job_system.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/MotionQuality.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

namespace ps {
namespace {

// ---------------------------------------------------------------------
// Object/broadphase layers - the standard two-layer setup from Jolt's own
// samples (static vs. moving). P1 has no need for more layers.
// ---------------------------------------------------------------------
namespace Layers {
constexpr JPH::ObjectLayer NON_MOVING = 0;
constexpr JPH::ObjectLayer MOVING = 1;
constexpr JPH::ObjectLayer NUM_LAYERS = 2;
} // namespace Layers

namespace BPLayers {
constexpr JPH::BroadPhaseLayer NON_MOVING(0);
constexpr JPH::BroadPhaseLayer MOVING(1);
constexpr JPH::uint NUM_LAYERS = 2;
} // namespace BPLayers

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        object_to_broadphase_[Layers::NON_MOVING] = BPLayers::NON_MOVING;
        object_to_broadphase_[Layers::MOVING] = BPLayers::MOVING;
    }
    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM_LAYERS; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override { return object_to_broadphase_[layer]; }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    // Jolt's own CMakeLists only defines JPH_PROFILE_ENABLED for Debug-type
    // configs (confirmed: absent from the asan/release preset compile
    // command, whose build type is RelWithDebInfo/Release), so this
    // override must not exist unconditionally - the base class method
    // itself is behind the same guard (Physics/Collision/BroadPhase/
    // BroadPhaseLayer.h).
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == BPLayers::NON_MOVING ? "NON_MOVING" : "MOVING";
    }
#endif
private:
    JPH::BroadPhaseLayer object_to_broadphase_[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer1, JPH::BroadPhaseLayer layer2) const override {
        switch (layer1) {
            case Layers::NON_MOVING: return layer2 == BPLayers::MOVING;
            case Layers::MOVING: return true;
            default: return false;
        }
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        switch (a) {
            case Layers::NON_MOVING: return b == Layers::MOVING;
            case Layers::MOVING: return true;
            default: return false;
        }
    }
};

// ---------------------------------------------------------------------
// Conversions. Positions are RVec3 (double, since JPH_DOUBLE_PRECISION is
// on - PLAN.md D2); velocities/forces/rotations stay Jolt's native
// single-precision Vec3/Quat even in double-precision mode (Jolt's double
// precision is specifically for large-world positions).
// ---------------------------------------------------------------------
JPH::RVec3 to_rvec3(const Vec3& v) { return JPH::RVec3(v.x, v.y, v.z); }
Vec3 from_rvec3(const JPH::RVec3& v) { return Vec3{v.GetX(), v.GetY(), v.GetZ()}; }
JPH::Vec3 to_vec3f(const Vec3& v) { return JPH::Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)); }
Vec3 from_vec3f(const JPH::Vec3& v) { return Vec3{real(v.GetX()), real(v.GetY()), real(v.GetZ())}; }
JPH::Quat to_jph_quat(const Quat& q) {
    return JPH::Quat(static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z), static_cast<float>(q.w));
}
Quat from_jph_quat(const JPH::Quat& q) { return Quat{real(q.GetX()), real(q.GetY()), real(q.GetZ()), real(q.GetW())}; }

JPH::RefConst<JPH::Shape> make_shape(const ShapeDesc& desc) {
    return std::visit(
        [](auto&& s) -> JPH::RefConst<JPH::Shape> {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, BoxShape>) {
                return JPH::RefConst<JPH::Shape>(new JPH::BoxShape(to_vec3f(s.half_extents)));
            } else if constexpr (std::is_same_v<T, SphereShape>) {
                return JPH::RefConst<JPH::Shape>(new JPH::SphereShape(static_cast<float>(s.radius)));
            } else {
                static_assert(std::is_same_v<T, CapsuleShape>);
                return JPH::RefConst<JPH::Shape>(new JPH::CapsuleShape(static_cast<float>(s.half_height), static_cast<float>(s.radius)));
            }
        },
        desc);
}

// Jolt's own BodyID is an internal slot index/generation we don't control
// (and don't want to leak as ps::BodyId, whose slot scheme is ours - see
// JoltRigidBackend::alloc_slot). Every body's Jolt user-data field instead
// carries our own BodyId (index in the low 32 bits, generation in the high
// 32 bits), set once at creation, so query results (ray/shape casts,
// contact callbacks) can be translated back to a ps::BodyId in O(1).
std::uint64_t pack_body_id(BodyId id) {
    return (static_cast<std::uint64_t>(id.generation) << 32) | static_cast<std::uint64_t>(id.index);
}
BodyId unpack_body_id(std::uint64_t user_data) {
    return BodyId{static_cast<id_index_t>(user_data & 0xffffffffu), static_cast<id_generation_t>(user_data >> 32)};
}

JPH::EMotionType to_jph_motion(BodyMotionType m) {
    switch (m) {
        case BodyMotionType::Static: return JPH::EMotionType::Static;
        case BodyMotionType::Kinematic: return JPH::EMotionType::Kinematic;
        case BodyMotionType::Dynamic: return JPH::EMotionType::Dynamic;
    }
    return JPH::EMotionType::Static;
}

JPH::EMotionQuality to_jph_motion_quality(MotionQuality q) {
    switch (q) {
        case MotionQuality::Discrete: return JPH::EMotionQuality::Discrete;
        case MotionQuality::LinearCast: return JPH::EMotionQuality::LinearCast;
    }
    return JPH::EMotionQuality::Discrete;
}

// FNV-1a, folding raw bytes so the hash reflects the exact bit pattern of
// every double (PLAN.md D7: bit-identical state, not merely numerically
// close) - see ps/world/world.h's World::state_hash.
std::uint64_t fnv1a(std::uint64_t h, const void* data, std::size_t n) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}
template <typename T>
std::uint64_t fnv1a_val(std::uint64_t h, const T& v) { return fnv1a(h, &v, sizeof(T)); }

// Registered exactly once per process (RegisterTypes touches process-global
// Jolt state); every JoltRigidBackend instance shares it.
void trace_impl(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buffer[1024];
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    std::fprintf(stderr, "[Jolt] %s\n", buffer);
    std::fflush(stderr);
}

#ifdef JPH_ENABLE_ASSERTS
bool assert_failed_impl(const char* expression, const char* message, const char* file, JPH::uint line) {
    std::fprintf(stderr, "[Jolt ASSERT FAILED] %s:%u: (%s) %s\n", file, line, expression, message ? message : "");
    std::fflush(stderr);
    return true; // trigger a breakpoint when a debugger is attached
}
#endif

class JoltGlobalRegistration {
public:
    JoltGlobalRegistration() {
        JPH::Trace = trace_impl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assert_failed_impl;)
        JPH::RegisterDefaultAllocator();
        factory_ = new JPH::Factory();
        JPH::Factory::sInstance = factory_;
        JPH::RegisterTypes();
    }
    ~JoltGlobalRegistration() {
        JPH::UnregisterTypes();
        delete factory_;
        JPH::Factory::sInstance = nullptr;
    }
    JoltGlobalRegistration(const JoltGlobalRegistration&) = delete;

private:
    JPH::Factory* factory_ = nullptr;
};

void ensure_jolt_registered() {
    static JoltGlobalRegistration registration; // constructed on first use, thread-safe (C++11 magic static)
    (void)registration;
}

class BackendContactListener final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, JPH::ContactSettings&) override {
        record(body1, body2, manifold);
    }
    void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, JPH::ContactSettings&) override {
        record(body1, body2, manifold);
    }

    std::vector<ContactEvent> drain() {
        std::lock_guard<std::mutex> lock(m_);
        std::vector<ContactEvent> out;
        out.swap(pending_);
        return out;
    }

private:
    // Called with all bodies locked, from potentially many of Jolt's own
    // parallel contact jobs at once - a mutex here is backend bookkeeping,
    // not one of our own force-element/vehicle-island jobs, so it does not
    // fall under D8's "no locks inside hot-path jobs" (that rule targets
    // code we schedule on ps::jobs, not Jolt's internal solver callbacks).
    void record(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold) {
        ContactEvent ev;
        ev.a = unpack_body_id(body1.GetUserData());
        ev.b = unpack_body_id(body2.GetUserData());
        ev.point = manifold.mRelativeContactPointsOn1.empty() ? Vec3::zero() : from_rvec3(manifold.GetWorldSpaceContactPointOn1(0));
        ev.normal = from_vec3f(manifold.mWorldSpaceNormal);
        ev.normal_impulse = 0.0; // not known at OnContactAdded time (see Jolt's own doc comment)
        std::lock_guard<std::mutex> lock(m_);
        pending_.push_back(ev);
    }

    std::mutex m_;
    std::vector<ContactEvent> pending_;
};

class JoltRigidBackend final : public IRigidBackend {
public:
    JoltRigidBackend(jobs::JobSystem& job_system, const Vec3& gravity, std::size_t max_bodies, const RigidBackendSettings& settings)
        : pool_(job_system), collision_steps_(std::max(1, settings.collision_steps)) {
        ensure_jolt_registered();

        // Scale broadphase/contact/temp-allocator capacity with max_bodies
        // rather than a large fixed constant - a fixed 65536-pair budget
        // demanded a temp-allocation up front that a small single-body
        // scenario's 16 MB arena could not satisfy (observed via Jolt's own
        // Trace callback: "TempAllocator: Out of memory trying to allocate
        // 31457280 bytes" - found by wiring up JPH::Trace/AssertFailed,
        // which are unset by default and otherwise fail via a silent
        // JPH_BREAKPOINT with no message at all, see ensure_jolt_registered).
        const JPH::uint max_body_pairs = std::max<JPH::uint>(1024, static_cast<JPH::uint>(max_bodies) * 4);
        const JPH::uint max_contact_constraints = max_body_pairs;
        const std::size_t temp_allocator_bytes = std::max<std::size_t>(16 * 1024 * 1024, max_bodies * 16 * 1024);

        temp_allocator_ = std::make_unique<JPH::TempAllocatorImpl>(temp_allocator_bytes);
        jolt_job_system_ = std::make_unique<jobs::JoltJobSystemBridge>(job_system);

        physics_system_.Init(static_cast<JPH::uint>(max_bodies), 0, max_body_pairs, max_contact_constraints,
                              bp_layer_interface_, bp_filter_, layer_filter_);
        physics_system_.SetGravity(to_vec3f(gravity));
        physics_system_.SetContactListener(&contact_listener_);

        // Part A energy-loss study (PLAN.md): every field here defaults to
        // Jolt's own PhysicsSettings default (RigidBackendSettings' own
        // comments name the exact field), so a World built with a
        // default-constructed WorldConfig gets an identical PhysicsSettings
        // to before this seam existed.
        JPH::PhysicsSettings physics_settings = physics_system_.GetPhysicsSettings();
        physics_settings.mBaumgarte = static_cast<float>(settings.baumgarte);
        physics_settings.mNumVelocitySteps = settings.num_velocity_steps;
        physics_settings.mNumPositionSteps = settings.num_position_steps;
        physics_settings.mPenetrationSlop = static_cast<float>(settings.penetration_slop);
        physics_settings.mSpeculativeContactDistance = static_cast<float>(settings.speculative_contact_distance);
        physics_settings.mMaxPenetrationDistance = static_cast<float>(settings.max_penetration_distance);
        physics_settings.mLinearCastThreshold = static_cast<float>(settings.linear_cast_threshold);
        physics_settings.mLinearCastMaxPenetration = static_cast<float>(settings.linear_cast_max_penetration);
        physics_system_.SetPhysicsSettings(physics_settings);
    }

    ~JoltRigidBackend() override {
        // Wait for the pool to be fully idle before touching anything a
        // still-in-flight fire-and-forget job might reference (see
        // ps::jobs::JobSystem::drain_idle's doc comment for the exact bug
        // this fixes: JoltJobSystemBridge's own Job freelist, below, must
        // outlive every worker that could still be inside
        // QueueJob(...)::lambda's job->Execute() call).
        pool_.drain_idle();

        // Constraints first (RemoveConstraint while every body they
        // reference is still valid), then bodies fully destroyed (not just
        // removed from the broadphase) - both explicit rather than relying
        // on physics_system_'s own destructor to unwind everything, since
        // our joint_slots_/slots_ members are destroyed (in C++'s reverse
        // declaration order) before physics_system_ itself, and physics_system_'s
        // own destruction order relative to anything we did not explicitly
        // detach first should not be relied on.
        for (auto& j : joint_slots_) {
            if (j.alive && j.constraint) physics_system_.RemoveConstraint(j.constraint.GetPtr());
        }
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            if (slots_[i].alive) {
                body_interface().RemoveBody(slots_[i].jph_id);
                body_interface().DestroyBody(slots_[i].jph_id);
            }
        }
    }

    BodyId create_body(const BodyDesc& desc) override {
        JPH::RefConst<JPH::Shape> shape = make_shape(desc.shape);
        const JPH::EMotionType motion = to_jph_motion(desc.motion);
        const JPH::ObjectLayer layer = (desc.motion == BodyMotionType::Static) ? Layers::NON_MOVING : Layers::MOVING;

        JPH::BodyCreationSettings settings(shape, to_rvec3(desc.pose.position), to_jph_quat(desc.pose.orientation), motion, layer);
        settings.mLinearVelocity = to_vec3f(desc.velocity.linear);
        settings.mAngularVelocity = to_vec3f(desc.velocity.angular);
        settings.mFriction = static_cast<float>(desc.friction);
        settings.mRestitution = static_cast<float>(desc.restitution);
        settings.mLinearDamping = static_cast<float>(desc.linear_damping);
        settings.mAngularDamping = static_cast<float>(desc.angular_damping);
        settings.mGravityFactor = desc.gravity_enabled ? 1.0f : 0.0f;
        settings.mAllowSleeping = desc.allow_sleep;
        settings.mMotionQuality = to_jph_motion_quality(desc.motion_quality);
        settings.mMaxAngularVelocity = static_cast<float>(desc.max_angular_velocity);
        settings.mMaxLinearVelocity = static_cast<float>(desc.max_linear_velocity);
        if (desc.motion != BodyMotionType::Static) {
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = static_cast<float>(desc.mass);
        }

        const JPH::BodyID jph_id = body_interface().CreateAndAddBody(settings, JPH::EActivation::Activate);
        const BodyId id = alloc_slot(jph_id);
        body_interface().SetUserData(jph_id, pack_body_id(id));
        return id;
    }

    void destroy_body(BodyId id) override {
        const JPH::BodyID jid = lookup(id);
        body_interface().RemoveBody(jid);
        body_interface().DestroyBody(jid);
        free_slot(id);
    }

    JointId create_joint(const JointDesc& desc) override {
        JPH::Body* body_a = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(lookup(desc.body_a));
        JPH::Body* body_b = desc.body_b.valid() ? physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(lookup(desc.body_b)) : &JPH::Body::sFixedToWorld;

        JPH::Ref<JPH::Constraint> constraint;
        if (desc.type == JointType::Fixed) {
            JPH::FixedConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mAutoDetectPoint = false;
            settings.mPoint1 = to_rvec3(body_a_world_point(desc, desc.anchor_a_local, true));
            settings.mPoint2 = to_rvec3(body_b_world_point(desc, desc.anchor_b_local, body_a, body_b));
            constraint = settings.Create(*body_a, *body_b);
        } else {
            JPH::HingeConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPoint1 = to_rvec3(body_a_world_point(desc, desc.anchor_a_local, true));
            settings.mPoint2 = to_rvec3(body_b_world_point(desc, desc.anchor_b_local, body_a, body_b));
            const Vec3 axis_a_world = from_jph_quat(body_a->GetRotation()).rotate(desc.axis_a_local).normalized();
            settings.mHingeAxis1 = to_vec3f(axis_a_world);
            settings.mHingeAxis2 = desc.body_b.valid() ? to_vec3f(from_jph_quat(body_b->GetRotation()).rotate(desc.axis_b_local).normalized()) : to_vec3f(axis_a_world);
            const Vec3 normal_a_world = any_perpendicular(axis_a_world);
            settings.mNormalAxis1 = to_vec3f(normal_a_world);
            settings.mNormalAxis2 = to_vec3f(normal_a_world);
            settings.mMaxFrictionTorque = static_cast<float>(desc.friction_torque);
            if (desc.limits_enabled) {
                settings.mLimitsMin = static_cast<float>(desc.limit_min_rad);
                settings.mLimitsMax = static_cast<float>(desc.limit_max_rad);
            }
            constraint = settings.Create(*body_a, *body_b);
        }
        physics_system_.AddConstraint(constraint);
        return alloc_joint_slot(constraint);
    }

    void destroy_joint(JointId id) override {
        JPH::Constraint* c = joint_slots_[id.index].constraint.GetPtr();
        if (c) physics_system_.RemoveConstraint(c);
        joint_slots_[id.index].constraint = nullptr;
        joint_slots_[id.index].alive = false;
        joint_free_list_.push_back(id.index);
    }

    Pose get_pose(BodyId id) const override {
        JPH::RVec3 pos;
        JPH::Quat rot;
        body_interface().GetPositionAndRotation(lookup(id), pos, rot);
        return Pose{from_rvec3(pos), from_jph_quat(rot)};
    }

    Motion get_motion(BodyId id) const override {
        const JPH::BodyID jid = lookup(id);
        return Motion{from_vec3f(body_interface().GetLinearVelocity(jid)), from_vec3f(body_interface().GetAngularVelocity(jid))};
    }

    void set_pose(BodyId id, const Pose& pose) override {
        body_interface().SetPositionAndRotation(lookup(id), to_rvec3(pose.position), to_jph_quat(pose.orientation), JPH::EActivation::Activate);
    }

    void set_motion(BodyId id, const Motion& motion) override {
        body_interface().SetLinearAndAngularVelocity(lookup(id), to_vec3f(motion.linear), to_vec3f(motion.angular));
    }

    real get_mass(BodyId id) const override {
        const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(lookup(id));
        if (!body || body->IsStatic()) return 0.0;
        const float inv_mass = body->GetMotionProperties()->GetInverseMassUnchecked();
        return inv_mass > 0.0f ? real(1.0f / inv_mass) : 0.0;
    }

    Mat3 get_inertia_world(BodyId id) const override {
        const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(lookup(id));
        if (!body || body->IsStatic()) return Mat3::zero();
        const JPH::Mat44 inv_i = body->GetInverseInertia();
        const Vec3 c0 = from_vec3f(inv_i.GetColumn3(0));
        const Vec3 c1 = from_vec3f(inv_i.GetColumn3(1));
        const Vec3 c2 = from_vec3f(inv_i.GetColumn3(2));
        const Mat3 inv_inertia{Vec3{c0.x, c1.x, c2.x}, Vec3{c0.y, c1.y, c2.y}, Vec3{c0.z, c1.z, c2.z}};
        return inv_inertia.inverse();
    }

    void apply_wrench(BodyId id, const Wrench& wrench_at_com) override {
        body_interface().AddForceAndTorque(lookup(id), to_vec3f(wrench_at_com.force), to_vec3f(wrench_at_com.torque), JPH::EActivation::Activate);
    }

    void step(real dt) override {
        physics_system_.Update(static_cast<float>(dt), collision_steps_, temp_allocator_.get(), jolt_job_system_.get());
    }

    RayCastHit ray_cast(const Vec3& origin, const Vec3& direction, real max_distance) const override {
        JPH::RRayCast ray(to_rvec3(origin), to_vec3f(direction.normalized() * max_distance));
        JPH::RayCastResult result;
        RayCastHit out;
        if (physics_system_.GetNarrowPhaseQuery().CastRay(ray, result)) {
            out.hit = true;
            out.fraction = real(result.mFraction);
            out.point = from_rvec3(ray.GetPointOnRay(result.mFraction));
            const JPH::Body* hit_body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(result.mBodyID);
            out.body = hit_body ? unpack_body_id(hit_body->GetUserData()) : BodyId{};
        }
        return out;
    }

    ShapeCastHit shape_cast(const ShapeDesc& shape, const Pose& start_pose, const Vec3& sweep_world, real max_distance) const override {
        JPH::RefConst<JPH::Shape> jph_shape = make_shape(shape);
        JPH::RMat44 start = JPH::RMat44::sRotationTranslation(to_jph_quat(start_pose.orientation), to_rvec3(start_pose.position));
        JPH::RShapeCast cast = JPH::RShapeCast::sFromWorldTransform(jph_shape, JPH::Vec3::sReplicate(1.0f), start, to_vec3f(sweep_world.normalized() * max_distance));
        JPH::ShapeCastSettings settings;
        JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
        physics_system_.GetNarrowPhaseQuery().CastShape(cast, settings, to_rvec3(start_pose.position), collector);
        ShapeCastHit out;
        if (collector.HadHit()) {
            const Vec3 start_w = from_rvec3(cast.mCenterOfMassStart.GetTranslation());
            const Vec3 dir_w = from_vec3f(cast.mDirection);
            out.hit = true;
            out.fraction = real(collector.mHit.mFraction);
            out.point = start_w + dir_w * out.fraction;
            // "You can use -mPenetrationAxis.Normalized() as contact normal" (Jolt's own CollideShapeResult doc comment).
            out.normal = from_vec3f(-collector.mHit.mPenetrationAxis.Normalized());
            const JPH::Body* hit_body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(collector.mHit.mBodyID2);
            out.body = hit_body ? unpack_body_id(hit_body->GetUserData()) : BodyId{};
        }
        return out;
    }

    std::vector<ContactEvent> drain_contact_events() override { return contact_listener_.drain(); }

    std::uint64_t compute_state_hash(std::uint64_t seed) const override {
        std::uint64_t h = seed;
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            if (!slots_[i].alive) continue;
            const BodyId id{static_cast<id_index_t>(i), slots_[i].generation};
            const Pose pose = get_pose(id);
            const Motion motion = get_motion(id);
            h = fnv1a_val(h, i);
            h = fnv1a(h, &pose.position, sizeof(pose.position));
            h = fnv1a(h, &pose.orientation, sizeof(pose.orientation));
            h = fnv1a(h, &motion.linear, sizeof(motion.linear));
            h = fnv1a(h, &motion.angular, sizeof(motion.angular));
        }
        return h;
    }

    std::size_t body_count() const override {
        std::size_t n = 0;
        for (const auto& s : slots_) if (s.alive) ++n;
        return n;
    }

    std::vector<BodyId> live_body_ids() const override {
        std::vector<BodyId> ids;
        ids.reserve(slots_.size());
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            if (slots_[i].alive) ids.push_back(BodyId{static_cast<id_index_t>(i), slots_[i].generation});
        }
        return ids;
    }

private:
    struct Slot {
        JPH::BodyID jph_id;
        bool alive = false;
        id_generation_t generation = 0;
    };
    struct JointSlot {
        JPH::Ref<JPH::Constraint> constraint;
        bool alive = false;
    };

    JPH::BodyInterface& body_interface() { return physics_system_.GetBodyInterface(); }
    const JPH::BodyInterface& body_interface() const { return physics_system_.GetBodyInterface(); }

    JPH::BodyID lookup(BodyId id) const {
        assert(id.index < slots_.size() && slots_[id.index].alive && slots_[id.index].generation == id.generation && "stale or invalid BodyId");
        return slots_[id.index].jph_id;
    }

    BodyId alloc_slot(JPH::BodyID jph_id) {
        if (!free_list_.empty()) {
            const id_index_t idx = free_list_.back();
            free_list_.pop_back();
            slots_[idx] = Slot{jph_id, true, slots_[idx].generation + 1};
            return BodyId{idx, slots_[idx].generation};
        }
        slots_.push_back(Slot{jph_id, true, 1});
        return BodyId{static_cast<id_index_t>(slots_.size() - 1), 1};
    }

    void free_slot(BodyId id) {
        slots_[id.index].alive = false;
        free_list_.push_back(id.index);
    }

    JointId alloc_joint_slot(JPH::Ref<JPH::Constraint> c) {
        if (!joint_free_list_.empty()) {
            const id_index_t idx = joint_free_list_.back();
            joint_free_list_.pop_back();
            joint_slots_[idx] = JointSlot{c, true};
            return JointId{idx, 1};
        }
        joint_slots_.push_back(JointSlot{c, true});
        return JointId{static_cast<id_index_t>(joint_slots_.size() - 1), 1};
    }

    // World-space anchor point for a hinge/fixed constraint, derived from
    // the body's current pose (JointDesc anchors are given in body-local
    // space, but Jolt constraints are set up in world space here to avoid
    // reasoning about COM offsets - see jolt_backend.cpp's header comment).
    Vec3 body_a_world_point(const JointDesc& desc, const Vec3& local, bool /*isA*/) const {
        const JPH::BodyID jid = lookup(desc.body_a);
        JPH::RVec3 pos;
        JPH::Quat rot;
        body_interface().GetPositionAndRotation(jid, pos, rot);
        return from_rvec3(pos) + from_jph_quat(rot).rotate(local);
    }
    Vec3 body_b_world_point(const JointDesc& desc, const Vec3& local, JPH::Body* body_a, JPH::Body* body_b) const {
        if (!desc.body_b.valid()) {
            // Anchored to the world: interpret local as a world-space point
            // directly (there is no body B pose to compose with).
            (void)body_a;
            (void)body_b;
            return local;
        }
        const JPH::BodyID jid = lookup(desc.body_b);
        JPH::RVec3 pos;
        JPH::Quat rot;
        body_interface().GetPositionAndRotation(jid, pos, rot);
        return from_rvec3(pos) + from_jph_quat(rot).rotate(local);
    }

    static Vec3 any_perpendicular(const Vec3& axis) {
        const Vec3 ref = (std::abs(axis.x) < real(0.9)) ? Vec3::unit_x() : Vec3::unit_y();
        return cross(axis, ref).normalized();
    }

    jobs::JobSystem& pool_; // must be declared (and so initialised) first - see the destructor's drain_idle() call
    int collision_steps_ = 1; // PhysicsSystem::Update's inCollisionSteps (Part A study, see RigidBackendSettings)
    BPLayerInterfaceImpl bp_layer_interface_;
    ObjectVsBroadPhaseLayerFilterImpl bp_filter_;
    ObjectLayerPairFilterImpl layer_filter_;
    BackendContactListener contact_listener_;

    std::unique_ptr<JPH::TempAllocatorImpl> temp_allocator_;
    std::unique_ptr<jobs::JoltJobSystemBridge> jolt_job_system_;
    JPH::PhysicsSystem physics_system_;

    std::vector<Slot> slots_;
    std::vector<id_index_t> free_list_;
    std::vector<JointSlot> joint_slots_;
    std::vector<id_index_t> joint_free_list_;
};

} // namespace

std::unique_ptr<IRigidBackend> create_jolt_backend(jobs::JobSystem& job_system, const Vec3& gravity, std::size_t max_bodies,
                                                     const RigidBackendSettings& settings) {
    return std::make_unique<JoltRigidBackend>(job_system, gravity, max_bodies, settings);
}

} // namespace ps
