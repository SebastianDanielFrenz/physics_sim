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
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>

#include <algorithm>
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

// ---------------------------------------------------------------------
// Surface ids through Jolt materials (PLAN.md M1: "surface ids must
// survive collision queries and contact events"). Jolt's own per-triangle
// (MeshShape) and per-cell (HeightFieldShape) material index is 8 bits
// (Array<uint8> mMaterialIndices - see HeightFieldShape.h), so this backend
// shares ONE fixed, 256-entry PhysicsMaterialList across every mesh/
// heightfield shape it ever creates, built once (not per shape, and
// critically not inside JoltRigidBackend::update_heightfield_body, which
// PLAN.md D8 requires to be allocation-free on its hot tile-streaming path -
// see that method). Slot 255 is reserved to mean ps::kInvalidSurfaceId (the
// "no surface id supplied" sentinel both ShapeDesc's own doc comments and
// this file's callers use), never a real surface - so a real surface id
// must be < 255 to survive a round trip through a Jolt mesh/heightfield;
// this project never defines anywhere close to that many.
class SurfaceMaterial final : public JPH::PhysicsMaterial {
public:
    SurfaceMaterial() = default;
    explicit SurfaceMaterial(SurfaceId id) : surface_id(id) {}
    SurfaceId surface_id = kInvalidSurfaceId;
};

constexpr std::size_t kMaxJoltSurfaceMaterials = 256;
constexpr std::uint8_t kJoltInvalidSurfaceMaterialIndex = 255;

const JPH::PhysicsMaterialList& shared_surface_material_list() {
    static const JPH::PhysicsMaterialList list = [] {
        JPH::PhysicsMaterialList l;
        l.reserve(kMaxJoltSurfaceMaterials);
        for (std::size_t i = 0; i + 1 < kMaxJoltSurfaceMaterials; ++i) {
            l.push_back(new SurfaceMaterial(static_cast<SurfaceId>(i)));
        }
        l.push_back(new SurfaceMaterial(kInvalidSurfaceId)); // slot 255, see comment above
        return l;
    }();
    return list;
}

// A real (non-sentinel) surface id above this is not representable in a
// Jolt mesh/heightfield material index and is silently mapped to the
// invalid slot instead of corrupting an unrelated surface's slot.
std::uint8_t jolt_material_index_for_surface(SurfaceId id) {
    if (id == kInvalidSurfaceId || id >= kMaxJoltSurfaceMaterials - 1) return kJoltInvalidSurfaceMaterialIndex;
    return static_cast<std::uint8_t>(id);
}

SurfaceId surface_id_from_material(const JPH::PhysicsMaterial* mat) {
    const auto* sm = dynamic_cast<const SurfaceMaterial*>(mat);
    return sm ? sm->surface_id : kInvalidSurfaceId;
}

// Fixed rotation wrapping a Jolt HeightFieldShape (native Y-up: "position =
// offset + scale*(x, height, y)", HeightFieldShapeSettings' own doc comment)
// so it presents our own Z-up, body-local convention (HeightfieldShape's own
// doc comment, shape_desc.h) at the ShapeDesc seam: local point
// (ix*spacing, iy*spacing, height) rather than (ix*spacing, height,
// iy*spacing). This is a rotation about local X by +90 degrees - (x,y,z) ->
// (x,-z,y) - written as an exact constant (sin(45deg) == cos(45deg) for a
// 90-degree rotation) rather than computed from a runtime trig call, both to
// avoid PLAN.md D7's platform-libm grep entirely and because it is a fixed
// geometric constant, not scenario data.
//
// Under this rotation alone, sample row iy_jolt lands at body-local
// y = -(iy_jolt*spacing) - NEGATIVE of what we want. The first version of
// this code "fixed" that by making the inner shape's mScale.z NEGATIVE
// instead (so the sign cancelled) - wrong, found empirically (a raycast
// straight down never hit the heightfield at all, while a narrowphase
// CollideShape query against the exact same shape did): a negative scale
// component mirrors the shape, which flips every triangle's winding and
// therefore its normal. JPH::NarrowPhaseQuery::CastRay back-face-culls by
// default, so every ray from above saw only back faces and reported no hit;
// worse, a real contact (as opposed to a query with no side effects) would
// have resolved penetration along the flipped (inward-pointing) normal.
// The actual fix keeps the shape's own scale strictly positive (correct
// winding) and instead reverses which SAMPLE ROW is stored at which Jolt
// row index - see reverse_heightfield_rows_into below - compensated by a
// constant position offset on the wrapping shape's translation (see
// heightfield_wrapper_position, used by both create_body's heightfield
// branch and make_heightfield_shape).
const JPH::Quat kHeightfieldYupToZup(0.70710678118654752440f, 0.0f, 0.0f, 0.70710678118654752440f);

// Position offset needed for the RotatedTranslatedShape wrapper so that,
// combined with kHeightfieldYupToZup and the ROW-REVERSED sample layout
// (see reverse_heightfield_rows_into), body-local sample (ix, iy) lands at
// exactly origin + (ix, iy, 0)*spacing, height along +z - see this file's
// header comment on kHeightfieldYupToZup for the derivation.
JPH::Vec3 heightfield_wrapper_position(const HeightfieldShape& s) {
    return to_vec3f(Vec3{s.origin.x, s.origin.y + static_cast<real>(s.sample_count - 1) * s.sample_spacing, s.origin.z});
}

// Copies `src` (our own row-major, y-major convention - row `our_iy` at
// src[our_iy*n .. ]) into `dst` with rows reversed (Jolt row `iy` <- our row
// `n-1-iy`) - see kHeightfieldYupToZup's comment for why. `dst` must already
// be sized `n*n` (heights) or `n*n` cells for materials; this function does
// not resize (PLAN.md D8: callers on the hot update path own their own
// preallocated scratch).
void reverse_heightfield_rows_into(const std::vector<real>& src, std::uint32_t n, float* dst) {
    for (std::uint32_t iy = 0; iy < n; ++iy) {
        const std::uint32_t src_row = n - 1 - iy;
        for (std::uint32_t ix = 0; ix < n; ++ix) {
            const std::size_t src_i = static_cast<std::size_t>(src_row) * n + ix;
            dst[static_cast<std::size_t>(iy) * n + ix] = (src_i < src.size()) ? static_cast<float>(src[src_i]) : 0.0f;
        }
    }
}
void reverse_heightfield_material_rows_into(const std::vector<SurfaceId>& src, std::uint32_t cells, std::uint8_t* dst) {
    for (std::uint32_t iy = 0; iy < cells; ++iy) {
        const std::uint32_t src_row = cells - 1 - iy;
        for (std::uint32_t ix = 0; ix < cells; ++ix) {
            const std::size_t src_i = static_cast<std::size_t>(src_row) * cells + ix;
            const SurfaceId sid = (src_i < src.size()) ? src[src_i] : kInvalidSurfaceId;
            dst[static_cast<std::size_t>(iy) * cells + ix] = jolt_material_index_for_surface(sid);
        }
    }
}

// Unlike HeightFieldShape (256 materials, see shared_surface_material_list),
// JPH::MeshShape caps out at 32 materials ("Supporting max 32 materials per
// mesh" - MeshShape.cpp, found empirically when this used the shared
// 256-entry list unconditionally). A per-mesh local list built from only the
// DISTINCT ids this particular mesh actually uses is not on any hot path
// (mesh creation happens once, e.g. the test-ground loader's static
// geometry - PLAN.md M1 step 4), so this is a small one-time allocation,
// unlike the heightfield's shared/fixed list (which exists specifically to
// keep the HOT tile-update path allocation-free). data/tracks/test_ground's
// own collision format groups triangles BY surface already (one surface per
// mesh in practice), so this limit is generous, not a real constraint.
JPH::RefConst<JPH::Shape> make_mesh_shape(const MeshShape& s) {
    JPH::VertexList verts;
    verts.reserve(s.vertices.size());
    for (const Vec3& v : s.vertices) verts.push_back(JPH::Float3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)));

    JPH::PhysicsMaterialList local_materials;
    std::vector<SurfaceId> local_ids; // local_materials[i] <-> local_ids[i]
    auto local_index_for = [&](SurfaceId sid) -> std::uint32_t {
        for (std::size_t i = 0; i < local_ids.size(); ++i) {
            if (local_ids[i] == sid) return static_cast<std::uint32_t>(i);
        }
        if (local_ids.size() >= 32) return 0; // clamp: see this function's header comment
        local_ids.push_back(sid);
        local_materials.push_back(new SurfaceMaterial(sid));
        return static_cast<std::uint32_t>(local_ids.size() - 1);
    };

    const std::size_t tri_count = s.indices.size() / 3;
    JPH::IndexedTriangleList tris;
    tris.reserve(tri_count);
    for (std::size_t t = 0; t < tri_count; ++t) {
        const SurfaceId sid = (t < s.triangle_surface_ids.size()) ? s.triangle_surface_ids[t] : kInvalidSurfaceId;
        tris.push_back(JPH::IndexedTriangle(s.indices[3 * t], s.indices[3 * t + 1], s.indices[3 * t + 2], local_index_for(sid)));
    }

    JPH::MeshShapeSettings settings(verts, tris, local_materials);
    JPH::Shape::ShapeResult result = settings.Create();
    if (result.HasError()) {
        std::fprintf(stderr, "[ps] MeshShape creation failed: %s\n", result.GetError().c_str());
        std::fflush(stderr);
        return nullptr;
    }
    return result.Get();
}

// Builds the inner (native Y-up) HeightFieldShape only - see
// kHeightfieldYupToZup's comment for the wrapping rotation applied by
// make_heightfield_shape below. Shared between initial creation and
// JoltRigidBackend::update_heightfield_body's re-creation path is
// deliberately NOT shared (update mutates in place instead, see that
// method) - this helper is create-only.
JPH::RefConst<JPH::HeightFieldShape> make_inner_heightfield(const HeightfieldShape& s) {
    assert(s.sample_count % 2 == 0 && s.sample_count >= 2 &&
           "HeightfieldShape::sample_count must be even (see shape_desc.h's doc comment) so Jolt's own "
           "block-size rounding never disagrees with update_heightfield_body's caller-supplied array sizes");
    std::vector<float> heights(static_cast<std::size_t>(s.sample_count) * s.sample_count);
    reverse_heightfield_rows_into(s.heights, s.sample_count, heights.data());

    const std::size_t cell_count = (s.sample_count >= 1) ? static_cast<std::size_t>(s.sample_count - 1) * (s.sample_count - 1) : 0;
    std::vector<std::uint8_t> material_indices(cell_count);
    reverse_heightfield_material_rows_into(s.cell_surface_ids, s.sample_count >= 1 ? s.sample_count - 1 : 0, material_indices.data());

    // Positive scale throughout (correct winding - see kHeightfieldYupToZup's
    // comment for why a negative component here was the actual bug).
    JPH::HeightFieldShapeSettings settings(heights.data(), JPH::Vec3::sZero(),
                                            JPH::Vec3(static_cast<float>(s.sample_spacing), 1.0f, static_cast<float>(s.sample_spacing)),
                                            s.sample_count, material_indices.empty() ? nullptr : material_indices.data(),
                                            shared_surface_material_list());
    JPH::Shape::ShapeResult result = settings.Create();
    if (result.HasError()) {
        std::fprintf(stderr, "[ps] HeightFieldShape creation failed: %s\n", result.GetError().c_str());
        std::fflush(stderr);
        return nullptr;
    }
    return JPH::RefConst<JPH::HeightFieldShape>(JPH::StaticCast<JPH::HeightFieldShape>(result.Get().GetPtr()));
}

JPH::RefConst<JPH::Shape> make_heightfield_shape(const HeightfieldShape& s) {
    JPH::RefConst<JPH::HeightFieldShape> inner = make_inner_heightfield(s);
    return JPH::RefConst<JPH::Shape>(new JPH::RotatedTranslatedShape(heightfield_wrapper_position(s), kHeightfieldYupToZup, inner.GetPtr()));
}

JPH::RefConst<JPH::Shape> make_shape(const ShapeDesc& desc) {
    return std::visit(
        [](auto&& s) -> JPH::RefConst<JPH::Shape> {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, BoxShape>) {
                return JPH::RefConst<JPH::Shape>(new JPH::BoxShape(to_vec3f(s.half_extents)));
            } else if constexpr (std::is_same_v<T, SphereShape>) {
                return JPH::RefConst<JPH::Shape>(new JPH::SphereShape(static_cast<float>(s.radius)));
            } else if constexpr (std::is_same_v<T, CapsuleShape>) {
                return JPH::RefConst<JPH::Shape>(new JPH::CapsuleShape(static_cast<float>(s.half_height), static_cast<float>(s.radius)));
            } else if constexpr (std::is_same_v<T, CylinderShape>) {
                return JPH::RefConst<JPH::Shape>(new JPH::CylinderShape(static_cast<float>(s.half_height), static_cast<float>(s.radius)));
            } else if constexpr (std::is_same_v<T, MeshShape>) {
                return make_mesh_shape(s);
            } else {
                static_assert(std::is_same_v<T, HeightfieldShape>);
                return make_heightfield_shape(s);
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

    // PHYS-009 Step 3: out.swap(pending_) used to hand pending_'s entire
    // buffer to a freshly default-constructed local `out` and leave
    // pending_ at zero capacity - so the very next OnContactAdded/
    // OnContactPersisted callback (called from inside Jolt's own solver
    // jobs, i.e. inside a job per PLAN.md D8) had to reallocate, every
    // single tick with any live contact (found via a debug_alloc backtrace
    // landing right here, called from JobFindCollisions). Copying into a
    // caller-owned, already-capacity'd `out` and clear()ing (not swapping)
    // pending_ lets both buffers stabilise independently after warm-up.
    void drain_into(std::vector<ContactEvent>& out) {
        std::lock_guard<std::mutex> lock(m_);
        out = pending_;
        pending_.clear();
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
        // PLAN.md M1: surface ids survive into ContactEvent via each body's
        // own shape + the manifold's own per-body sub shape id (the same
        // GetMaterial(SubShapeID) query collide_shape_into uses below) -
        // kInvalidSurfaceId for a shape with no materials (Box/Sphere/Capsule).
        ev.surface_a = surface_id_from_material(body1.GetShape()->GetMaterial(manifold.mSubShapeID1));
        ev.surface_b = surface_id_from_material(body2.GetShape()->GetMaterial(manifold.mSubShapeID2));
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
        // HeightfieldShape needs its own path so the raw JPH::HeightFieldShape*
        // can be stashed for update_heightfield_body (see Slot::heightfield's
        // comment) - make_shape() alone only returns the fully-wrapped
        // top-level JPH::Shape (a RotatedTranslatedShape for this variant).
        JPH::HeightFieldShape* raw_heightfield = nullptr;
        JPH::RefConst<JPH::Shape> shape;
        if (const auto* hf = std::get_if<HeightfieldShape>(&desc.shape)) {
            JPH::RefConst<JPH::HeightFieldShape> inner = make_inner_heightfield(*hf);
            raw_heightfield = const_cast<JPH::HeightFieldShape*>(inner.GetPtr());
            shape = JPH::RefConst<JPH::Shape>(new JPH::RotatedTranslatedShape(heightfield_wrapper_position(*hf), kHeightfieldYupToZup, inner.GetPtr()));
        } else {
            shape = make_shape(desc.shape);
        }
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
        const BodyId id = alloc_slot(jph_id, raw_heightfield);
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

    bool is_static(BodyId id) const override {
        const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(lookup(id));
        return body != nullptr && body->IsStatic();
    }

    // Load-time only (allocates) - see i_rigid_backend.h. Decorated/compound
    // shapes (e.g. the heightfield's RotatedTranslatedShape wrapper) cannot
    // be triangulated directly, so the body's shape is first flattened into
    // its leaf TransformedShapes. Vertices are returned relative to the
    // body's own COM (float precision stays good far from the world origin)
    // and offset back into world space in double.
    void body_triangle_xy_bounds_into(BodyId id, std::vector<XyBounds>& out) const override {
        out.clear();
        if (id.index >= slots_.size() || !slots_[id.index].alive || slots_[id.index].generation != id.generation) return;
        const JPH::TransformedShape root = body_interface().GetTransformedShape(slots_[id.index].jph_id);
        if (root.mShape == nullptr) return;
        JPH::AllHitCollisionCollector<JPH::TransformedShapeCollector> leaves;
        root.CollectTransformedShapes(JPH::AABox::sBiggest(), leaves);
        const JPH::RVec3 base = root.mShapePositionCOM;
        const Vec3 base_w = from_rvec3(base);
        constexpr int kBatch = JPH::Shape::cGetTrianglesMinTrianglesRequested * 8;
        std::vector<JPH::Float3> verts(static_cast<std::size_t>(kBatch) * 3);
        for (const JPH::TransformedShape& leaf : leaves.mHits) {
            JPH::Shape::GetTrianglesContext ctx;
            leaf.GetTrianglesStart(ctx, JPH::AABox::sBiggest(), base);
            for (;;) {
                const int count = leaf.GetTrianglesNext(ctx, kBatch, verts.data());
                if (count <= 0) break;
                for (int t = 0; t < count; ++t) {
                    XyBounds b{1e300, 1e300, -1e300, -1e300};
                    for (int k = 0; k < 3; ++k) {
                        const JPH::Float3& v = verts[static_cast<std::size_t>(t) * 3 + k];
                        const real x = base_w.x + static_cast<real>(v.x);
                        const real y = base_w.y + static_cast<real>(v.y);
                        b.min_x = std::min(b.min_x, x);
                        b.max_x = std::max(b.max_x, x);
                        b.min_y = std::min(b.min_y, y);
                        b.max_y = std::max(b.max_y, y);
                    }
                    out.push_back(b);
                }
            }
        }
    }

    void set_pose(BodyId id, const Pose& pose) override {
        body_interface().SetPositionAndRotation(lookup(id), to_rvec3(pose.position), to_jph_quat(pose.orientation), JPH::EActivation::Activate);
    }

    void set_motion(BodyId id, const Motion& motion) override {
        body_interface().SetLinearAndAngularVelocity(lookup(id), to_vec3f(motion.linear), to_vec3f(motion.angular));
    }

    // PLAN.md M1 tile streaming (see i_rigid_backend.h's doc comment for the
    // full contract). Mutates the existing HeightFieldShape's samples via
    // Jolt's own SetHeights/SetMaterials, both of which take a
    // JPH::TempAllocator for their temporary working memory - passing this
    // backend's own preallocated temp_allocator_ (the same arena
    // IRigidBackend::step() hands to Jolt's Update(), never touched
    // concurrently here since World's terrain_tiles pipeline stage runs
    // strictly after backend_step within one tick, PLAN.md section 5) means
    // this call's OWN temporary memory is a bump-pointer arena, not the
    // heap. The float/uint8 conversion buffers below are the only other
    // per-call storage; they are ordinary std::vector members that grow
    // once to the largest tile seen and never shrink (the same "steady-
    // state after warm-up" pattern as every other scratch buffer in this
    // codebase, e.g. AbaScratch) - not allocation-free on the very first
    // call for a given size, but zero-allocation for every call after that,
    // which is what PLAN.md D8 measures (see test_terrain_tile_manager.cpp).
    void update_heightfield_body(BodyId id, const std::vector<real>& heights, const std::vector<SurfaceId>& cell_surface_ids) override {
        (void)lookup(id); // validates the id (asserts on stale/invalid)
        JPH::HeightFieldShape* hf = slots_[id.index].heightfield;
        assert(hf && "update_heightfield_body called on a body without a heightfield shape");
        const std::uint32_t n = hf->GetSampleCount();
        assert(heights.size() == static_cast<std::size_t>(n) * n);

        // Row-reversed the same way as creation (reverse_heightfield_rows_into's
        // comment / kHeightfieldYupToZup's comment) - content must keep
        // matching the fixed wrapper rotation/position baked in at creation.
        if (heightfield_height_scratch_.size() < heights.size()) heightfield_height_scratch_.resize(heights.size());
        reverse_heightfield_rows_into(heights, n, heightfield_height_scratch_.data());
        hf->SetHeights(0, 0, n, n, heightfield_height_scratch_.data(), static_cast<std::ptrdiff_t>(n), *temp_allocator_);

        const std::size_t cell_count = (n >= 1) ? static_cast<std::size_t>(n - 1) * (n - 1) : 0;
        if (heightfield_material_scratch_.size() < cell_count) heightfield_material_scratch_.resize(cell_count);
        reverse_heightfield_material_rows_into(cell_surface_ids, n >= 1 ? n - 1 : 0, heightfield_material_scratch_.data());
        if (cell_count > 0) {
            hf->SetMaterials(0, 0, n - 1, n - 1, heightfield_material_scratch_.data(), static_cast<std::ptrdiff_t>(n - 1), nullptr, *temp_allocator_);
        }
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

    // PHYS-033 I1 W1: reuses get_mass/get_inertia_world's TryGetBody
    // null-safety pattern. Unlike get_inertia_world (which returns the
    // backend's own -inverted- convention for callers that want the
    // inertia itself), this returns the RAW inverse Jolt's own integrator
    // uses directly, at the CURRENT rotation - the I1 plan's own condition 1
    // notes Jolt applies the whole tick's accumulated torque with the
    // rotation at STEP START, so a caller that wants bit-for-bit agreement
    // must read this once per tick, before backend_step runs, not re-read
    // it mid-tick.
    BodyIntegrationParams get_integration_params(BodyId id) const override {
        BodyIntegrationParams out;
        const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(lookup(id));
        if (!body || body->IsStatic()) return out;
        const JPH::MotionProperties* mp = body->GetMotionProperties();
        out.inv_mass = real(mp->GetInverseMassUnchecked());
        const JPH::Mat44 inv_i = body->GetInverseInertia();
        const Vec3 c0 = from_vec3f(inv_i.GetColumn3(0));
        const Vec3 c1 = from_vec3f(inv_i.GetColumn3(1));
        const Vec3 c2 = from_vec3f(inv_i.GetColumn3(2));
        out.inv_inertia_world = Mat3{Vec3{c0.x, c1.x, c2.x}, Vec3{c0.y, c1.y, c2.y}, Vec3{c0.z, c1.z, c2.z}};
        out.gravity_factor = real(mp->GetGravityFactor());
        out.linear_damping = real(mp->GetLinearDamping());
        out.angular_damping = real(mp->GetAngularDamping());
        out.gyroscopic = body->GetApplyGyroscopicForce();
        return out;
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
            if (hit_body) out.surface = surface_id_from_material(hit_body->GetShape()->GetMaterial(collector.mHit.mSubShapeID2));
        }
        return out;
    }

    void drain_contact_events_into(std::vector<ContactEvent>& out) override { contact_listener_.drain_into(out); }

    // PLAN.md P1.5b strategy A: pure narrowphase overlap query, no sweep, no
    // side effects. AllHitCollisionCollector (not the ClosestHit collector
    // shape_cast above uses) - a compliant-contact link can genuinely rest
    // on more than one body's worth of geometry at once (e.g. straddling a
    // seam between two static pieces), and summing every overlap's force is
    // the physically correct thing to do, not just the closest one.
    void collide_shape_into(const ShapeDesc& shape, const Pose& pose, std::vector<ShapeContactHit>& out) const override {
        out.clear();
        JPH::RefConst<JPH::Shape> jph_shape = make_shape(shape);
        JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(to_jph_quat(pose.orientation), to_rvec3(pose.position));
        JPH::CollideShapeSettings settings;
        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
        physics_system_.GetNarrowPhaseQuery().CollideShape(jph_shape, JPH::Vec3::sReplicate(1.0f), transform, settings,
                                                             to_rvec3(pose.position), collector);
        out.reserve(collector.mHits.size());
        for (const auto& hit : collector.mHits) {
            const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(hit.mBodyID2);
            if (!body) continue;
            ShapeContactHit sch;
            sch.body = unpack_body_id(body->GetUserData());
            // mContactPointOn2 is relative to inBaseOffset (passed as
            // pose.position above) - Jolt's convention for keeping contact
            // points precise far from the origin even though the query
            // itself runs in float (Jolt/Physics/Collision/NarrowPhaseQuery.h).
            sch.point = pose.position + from_vec3f(hit.mContactPointOn2);
            // Same convention as shape_cast above ("You can use
            // -mPenetrationAxis.Normalized() as contact normal", Jolt's own
            // CollideShapeResult doc comment) - mPenetrationAxis points from
            // shape 1 (the query shape, `pose`) to shape 2 (`body`), so its
            // negation points away from body's surface toward the query
            // shape, matching ShapeContactHit::normal's documented direction.
            sch.normal = from_vec3f(-hit.mPenetrationAxis.Normalized());
            sch.penetration = real(hit.mPenetrationDepth);
            sch.surface = surface_id_from_material(body->GetShape()->GetMaterial(hit.mSubShapeID2));
            out.push_back(sch);
        }
    }

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

    void live_body_ids_into(std::vector<BodyId>& out) const override {
        out.clear(); // keeps capacity (PHYS-009 Step 3): no allocation once the caller's vector has stabilised
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            if (slots_[i].alive) out.push_back(BodyId{static_cast<id_index_t>(i), slots_[i].generation});
        }
    }

private:
    struct Slot {
        JPH::BodyID jph_id;
        bool alive = false;
        id_generation_t generation = 0;
        // Non-null only for a body created from a HeightfieldShape - a raw
        // observer pointer (kept alive by the body's own shape reference
        // chain: RotatedTranslatedShape holds a RefConst to it), so
        // update_heightfield_body can mutate content in place without
        // walking the DecoratedShape chain on every call (PLAN.md D8 hot
        // tile-streaming path).
        JPH::HeightFieldShape* heightfield = nullptr;
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

    BodyId alloc_slot(JPH::BodyID jph_id, JPH::HeightFieldShape* heightfield = nullptr) {
        if (!free_list_.empty()) {
            const id_index_t idx = free_list_.back();
            free_list_.pop_back();
            slots_[idx] = Slot{jph_id, true, slots_[idx].generation + 1, heightfield};
            return BodyId{idx, slots_[idx].generation};
        }
        slots_.push_back(Slot{jph_id, true, 1, heightfield});
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

    // update_heightfield_body's scratch conversion buffers - see that
    // method's comment.
    std::vector<float> heightfield_height_scratch_;
    std::vector<std::uint8_t> heightfield_material_scratch_;
};

} // namespace

std::unique_ptr<IRigidBackend> create_jolt_backend(jobs::JobSystem& job_system, const Vec3& gravity, std::size_t max_bodies,
                                                     const RigidBackendSettings& settings) {
    return std::make_unique<JoltRigidBackend>(job_system, gravity, max_bodies, settings);
}

} // namespace ps
