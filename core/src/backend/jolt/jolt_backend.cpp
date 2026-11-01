// core/src/backend/jolt/jolt_backend.cpp — the only P1 IRigidBackend
// implementation (PLAN.md D2). Every Jolt type stays confined to this file
// (plus the internal jolt_job_system.h bridge); ps/backend/i_rigid_backend.h
// exposes only create_jolt_backend() returning the abstract interface.

#include "ps/backend/i_rigid_backend.h"
#include "ps/jobs/debug_alloc.h"
#include "../../jobs/jolt_job_system.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyFilter.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/Memory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/StreamOut.h>
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
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Core/QuickSort.h>
#include <Jolt/Core/STLLocalAllocator.h>
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
#include <stdexcept>
#include <string>
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
//
// `error` (create_shape's path, racing_game R2): when non-null, a Jolt
// rejection is reported through it (the caller throws) instead of being
// printed; the desc path (error == nullptr) is unchanged. Thread safety
// (create_shape runs this on any thread): everything here is local - the
// settings object, its cached result, the local material list - and Jolt's
// MeshShape construction (Sanitize, sFindActiveEdges, AABBTreeBuilder,
// AABBTreeToBuffer) touches no mutable global state (Jolt v5.6.0: the only
// mutable statics in the shape sources are debug-draw flags); materials and
// shapes are RefTarget-counted with std::atomic.
JPH::RefConst<JPH::Shape> make_mesh_shape(const MeshShape& s, std::string* error = nullptr) {
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
        if (error != nullptr) {
            *error = result.GetError().c_str();
            return nullptr;
        }
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

// Builds a query-only shape WITHOUT heap allocation for the convex
// primitives (PHYS-033 I1b D-C; Jolt's own VehicleCollisionTesterCastCylinder
// pattern - see that file's Collide() implementation). SAME constructor
// arguments as make_shape above, so the geometry is bit-identical to what
// make_shape would have built (this is what keeps shape_cast/
// collide_shape_into's own output byte-for-byte unchanged - the only thing
// that changes is where the Shape object itself lives, not its content).
// SetEmbedded() is mandatory: a stray JPH::Ref/RefConst taken on the stack
// object inside `fn` must never call delete on it (JPH::RefTarget::
// SetEmbedded, Core/Reference.h - AddRef/Release become no-ops).
// Mesh/Heightfield fall back to make_shape (heap-allocates; these are never
// used as a wheel/query cast shape on any hot path in this codebase, so the
// allocation there is not a concern D-C is trying to remove).
template <class Fn>
decltype(auto) with_query_shape(const ShapeDesc& desc, Fn&& fn) {
    return std::visit(
        [&](auto&& s) -> decltype(auto) {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, BoxShape>) {
                JPH::BoxShape sh(to_vec3f(s.half_extents));
                sh.SetEmbedded();
                return fn(static_cast<const JPH::Shape&>(sh));
            } else if constexpr (std::is_same_v<T, SphereShape>) {
                JPH::SphereShape sh(static_cast<float>(s.radius));
                sh.SetEmbedded();
                return fn(static_cast<const JPH::Shape&>(sh));
            } else if constexpr (std::is_same_v<T, CapsuleShape>) {
                JPH::CapsuleShape sh(static_cast<float>(s.half_height), static_cast<float>(s.radius));
                sh.SetEmbedded();
                return fn(static_cast<const JPH::Shape&>(sh));
            } else if constexpr (std::is_same_v<T, CylinderShape>) {
                JPH::CylinderShape sh(static_cast<float>(s.half_height), static_cast<float>(s.radius));
                sh.SetEmbedded();
                return fn(static_cast<const JPH::Shape&>(sh));
            } else {
                JPH::RefConst<JPH::Shape> ref = make_shape(desc);
                return fn(*ref);
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

// A JPH::StreamOut that folds every written byte into an FNV-1a hash instead
// of storing it - debug_heightfield_digest's allocation-free sink for Jolt's
// own HeightFieldShape::SaveBinaryState.
class FnvStreamOut final : public JPH::StreamOut {
public:
    explicit FnvStreamOut(std::uint64_t seed) : h_(seed) {}
    void WriteBytes(const void* data, std::size_t n) override { h_ = fnv1a(h_, data, n); }
    bool IsFailed() const override { return false; }
    std::uint64_t hash() const { return h_; }

private:
    std::uint64_t h_;
};

// Digest of one inner HeightFieldShape: Jolt's own binary serialisation
// (offset/scale/sample count/quantisation range, every range block, height
// sample and active-edge bit, the material index bits) plus the surface id
// behind each entry of its material list, in list order.
std::uint64_t heightfield_digest(const JPH::HeightFieldShape& hf) {
    FnvStreamOut out(1469598103934665603ULL);
    hf.SaveBinaryState(out);
    std::uint64_t h = out.hash();
    const JPH::PhysicsMaterialList& mats = hf.GetMaterialList();
    h = fnv1a_val(h, static_cast<std::uint64_t>(mats.size()));
    for (const JPH::PhysicsMaterial* m : mats) h = fnv1a_val(h, surface_id_from_material(m));
    return h;
}

// The one content-write sequence for a heightfield shape, shared by
// update_heightfield_body (in-tick, backend scratch + temp allocator) and
// build_heightfield_spare (any thread, the caller's own build context), so
// the two paths cannot drift apart. `height_scratch`/`material_scratch`
// grow once to the tile size and never shrink.
void write_heightfield_content(JPH::HeightFieldShape& hf, const std::vector<real>& heights, const std::vector<SurfaceId>& cell_surface_ids,
                               std::vector<float>& height_scratch, std::vector<std::uint8_t>& material_scratch, JPH::TempAllocator& temp) {
    const std::uint32_t n = hf.GetSampleCount();
    assert(heights.size() == static_cast<std::size_t>(n) * n);

    // Row-reversed the same way as creation (reverse_heightfield_rows_into's
    // comment / kHeightfieldYupToZup's comment) - content must keep
    // matching the fixed wrapper rotation/position baked in at creation.
    if (height_scratch.size() < heights.size()) height_scratch.resize(heights.size());
    reverse_heightfield_rows_into(heights, n, height_scratch.data());
    hf.SetHeights(0, 0, n, n, height_scratch.data(), static_cast<std::ptrdiff_t>(n), temp);

    const std::size_t cell_count = (n >= 1) ? static_cast<std::size_t>(n - 1) * (n - 1) : 0;
    if (material_scratch.size() < cell_count) material_scratch.resize(cell_count);
    reverse_heightfield_material_rows_into(cell_surface_ids, n >= 1 ? n - 1 : 0, material_scratch.data());
    if (cell_count > 0) {
        hf.SetMaterials(0, 0, n - 1, n - 1, material_scratch.data(), static_cast<std::ptrdiff_t>(n - 1), nullptr, temp);
    }
}

// IRigidBackend::create_heightfield_build_context's concrete type: one per
// builder thread. The temp allocator only ever serves SetHeights'
// active-edge normals block (<= 2*45*45*sizeof(Vec3), ~64 KB, see Jolt's
// HeightFieldShape::CalculateActiveEdges) and SetMaterials' 256-byte remap
// table - never a full-tile temp copy, since a whole-tile SetHeights has no
// neighbouring block to re-read - so 256 KB covers every sample count with
// a wide margin.
class JoltHeightfieldBuildContext final : public IHeightfieldBuildContext {
public:
    explicit JoltHeightfieldBuildContext(std::uint32_t sample_count)
        : temp(256 * 1024),
          heights(static_cast<std::size_t>(sample_count) * sample_count),
          materials(sample_count >= 1 ? static_cast<std::size_t>(sample_count - 1) * (sample_count - 1) : 0) {}
    JPH::TempAllocatorImpl temp;
    std::vector<float> heights;
    std::vector<std::uint8_t> materials;
};

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

// PHYS-033 I1b (D-D): debug_alloc's own global operator new/delete overrides
// cannot see Jolt's allocations - Jolt shapes use JPH_OVERRIDE_NEW_DELETE
// (Core/Memory.h), which routes through JPH::Allocate/Reallocate/
// AlignedAllocate (defaulted to plain malloc/_aligned_malloc by
// RegisterDefaultAllocator() below), never through operator new. Wrapping
// those three function pointers - AFTER RegisterDefaultAllocator() installs
// the defaults we then save and chain to, BEFORE any Jolt allocation can
// happen (the Factory just below is the first) - makes every allocation
// Jolt itself makes on a job's stack (shape construction, its own internal
// job-body allocations, PhysicsSystem::Update's own arrays) visible to
// debug_alloc, same rules as any other job-body allocation (armed/in-job/
// re-entrancy). Free/AlignedFree are left untouched: pairing an allocation
// made through the (possibly wrapped) Allocate with Jolt's own unwrapped
// Free is exactly what Jolt itself always does internally, so nothing here
// needs to intercept the free side at all.
//
// #ifndef JPH_DISABLE_CUSTOM_ALLOCATOR: Jolt only ever defines that macro for
// its own ReleaseASAN/ReleaseTSAN CMake configurations (Jolt.cmake), never
// for any preset this project builds - guarded anyway so a future change to
// that would fail to compile loudly here rather than silently reference
// symbols Jolt no longer exports in that mode (Memory.h's own #else branch
// declares Allocate/Reallocate/etc. as plain non-overridable functions, not
// variables this file could point elsewhere).
#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR
JPH::AllocateFunction g_jolt_alloc = nullptr;
JPH::ReallocateFunction g_jolt_realloc = nullptr;
JPH::AlignedAllocateFunction g_jolt_aligned_alloc = nullptr;

void* counting_allocate(size_t n) {
    ps::jobs::debug_alloc::note_external_allocation();
    return g_jolt_alloc(n);
}
void* counting_reallocate(void* p, size_t old_size, size_t new_size) {
    ps::jobs::debug_alloc::note_external_allocation();
    return g_jolt_realloc(p, old_size, new_size);
}
void* counting_aligned_allocate(size_t n, size_t alignment) {
    ps::jobs::debug_alloc::note_external_allocation();
    return g_jolt_aligned_alloc(n, alignment);
}

void install_counting_jolt_allocator() {
    g_jolt_alloc = JPH::Allocate;
    g_jolt_realloc = JPH::Reallocate;
    g_jolt_aligned_alloc = JPH::AlignedAllocate;
    JPH::Allocate = counting_allocate;
    JPH::Reallocate = counting_reallocate;
    JPH::AlignedAllocate = counting_aligned_allocate;
}
#endif // !JPH_DISABLE_CUSTOM_ALLOCATOR

class JoltGlobalRegistration {
public:
    JoltGlobalRegistration() {
        JPH::Trace = trace_impl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assert_failed_impl;)
        JPH::RegisterDefaultAllocator();
#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR
        // hook_available() is false under ASan/TSan (debug_alloc.cpp's own
        // operator new/delete overrides are compiled out there to avoid a
        // duplicate-symbol link error) - installing this wrapper anyway
        // would be harmless (note_external_allocation() would just never be
        // called with the hook armed) but there's no reason to pay even the
        // one indirect-call layer's cost on a build that can never observe
        // it.
        if (ps::jobs::debug_alloc::hook_available()) install_counting_jolt_allocator();
#endif
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

// ---------------------------------------------------------------------
// Prebuilt shapes (racing_game R2, ps/backend/shape_handle.h). A
// JoltPreparedShape is the immutable payload behind a ShapeHandle: the
// fully built top-level Jolt shape (exactly what make_shape returns for the
// same desc). Every body created from it holds a shared_ptr to it in its
// Slot (JoltRigidBackend::Slot::prepared), and the Jolt Body itself holds
// its own RefConst to `shape` - so by the time this object is destroyed no
// body uses `shape` any more and its reference is the last one (counted in
// debug_prepared_shape_backend_frees for the lifetime test).
//
// Thread safety of create_jolt_shape_handle (any thread, concurrently with
// PhysicsSystem::Update and other builds), Jolt v5.6.0:
// - Jolt is registered once (ensure_jolt_registered, a magic static - the
//   call below makes a first use on a builder thread safe too); allocation
//   goes through JPH::Allocate, a function pointer written only inside that
//   registration (and through it debug_alloc's thread_local job check).
// - Convex shapes: plain constructors over their arguments (material
//   pointer null, no lookup of PhysicsMaterial::sDefault).
// - MeshShape: see make_mesh_shape's comment - local settings, local
//   material list, no mutable global state.
// - RefTarget reference counts (shapes, materials) are std::atomic, so a
//   handle released on another thread while PhysicsSystem::Update reads the
//   same shape through a body is race-free; the shape itself is immutable.
// - No JoltRigidBackend member is touched (create_shape is const and calls
//   only this free function), so there is nothing of ours to lock.
// HeightfieldShape is rejected: heightfield content is mutable
// (update_heightfield_body / the prefetch ring's spares), which a shared
// immutable handle cannot honour.
class JoltPreparedShape final : public PreparedShape {
public:
    JoltPreparedShape(ShapeKind kind, JPH::RefConst<JPH::Shape> s) : PreparedShape(kind), shape(std::move(s)) {}
    ~JoltPreparedShape() override {
        if (shape != nullptr && shape->GetRefCount() == 1) debug_note_prepared_shape_backend_free();
    }
    const JPH::RefConst<JPH::Shape> shape;
};

ShapeKind shape_kind_of(const ShapeDesc& desc) {
    return std::visit(
        [](auto&& s) -> ShapeKind {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, BoxShape>) return ShapeKind::Box;
            else if constexpr (std::is_same_v<T, SphereShape>) return ShapeKind::Sphere;
            else if constexpr (std::is_same_v<T, CapsuleShape>) return ShapeKind::Capsule;
            else if constexpr (std::is_same_v<T, CylinderShape>) return ShapeKind::Cylinder;
            else if constexpr (std::is_same_v<T, MeshShape>) return ShapeKind::Mesh;
            else return ShapeKind::Heightfield;
        },
        desc);
}

ShapeHandle create_jolt_shape_handle(const ShapeDesc& desc) {
    ensure_jolt_registered();
    if (std::holds_alternative<HeightfieldShape>(desc)) {
        throw std::invalid_argument("create_shape: HeightfieldShape is not supported (heightfield content is mutable - use the "
                                    "terrain tile system / heightfield spares)");
    }
    JPH::RefConst<JPH::Shape> shape;
    if (const auto* mesh = std::get_if<MeshShape>(&desc)) {
        std::string error;
        shape = make_mesh_shape(*mesh, &error);
        if (shape == nullptr) throw std::runtime_error("create_shape: MeshShape rejected by Jolt: " + error);
    } else {
        shape = make_shape(desc); // the same constructor arguments as create_body(desc)
    }
    return ShapeHandle(std::make_shared<const JoltPreparedShape>(shape_kind_of(desc), std::move(shape)));
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

// Terrain tile seams (PLAN.md P8, vault PHYS-021): Jolt's HeightFieldShape
// marks every edge on its own outer border ACTIVE (it cannot see the
// neighbouring tile or static mesh), so a body sliding from tile A onto tile B picks up a
// speculative contact against B's first border edge whose normal tilts back
// against the motion - a one-tick vertical kick of O(100-1000) m/s^2 that a
// single seamless heightfield never shows. Fix: every body-vs-static-surface
// pair runs Jolt's own enhanced internal-edge removal (Terdiman's algorithm,
// JPH::InternalEdgeRemovingCollector - CollideWithAll + void every vertex of
// a face contact, drop edge/vertex contacts whose feature is fully voided),
// but with the dynamic body collided against EVERY heightfield/mesh body its
// bounds overlap, in ascending BodyID order, so a face contact on A voids
// the shared seam vertices of B's border edge. Only hits on the pair's own
// body2 are forwarded; the neighbours' hits only void features. This is a
// line-for-line copy of Jolt v5.6.0's InternalEdgeRemovingCollector (its
// AddHit/Flush) plus that forwarding flag - the stock class has no way to
// see a second body. Vertex tolerance is 1 cm, not Jolt's default 0.1 mm:
// the two tiles quantise the same seam sample against different per-block
// height ranges (8 bits/sample), so a shared vertex can differ by a few mm;
// distinct grid vertices are always >= one sample spacing (~1 m+) apart.
constexpr float kTerrainSeamVertexToleranceSq = 1.0e-4f; // (1 cm)^2

class TerrainSeamWeldingCollector final : public JPH::CollideShapeCollector {
    static constexpr JPH::uint kMaxLocalDelayed = 32;
    static constexpr JPH::uint kMaxLocalVoided = 128;

public:
    TerrainSeamWeldingCollector(JPH::CollideShapeCollector& chained, float vertex_tolerance_sq)
        : JPH::CollideShapeCollector(chained), chained_(chained), vertex_tolerance_sq_(vertex_tolerance_sq) {
        voided_.reserve(kMaxLocalVoided);
        delayed_.reserve(kMaxLocalDelayed);
    }

    // true for the pair's own body2 (the only hits passed on); false
    // while colliding neighbouring static surface geometry.
    void set_forward(bool forward) { forward_ = forward; }

    void AddHit(const JPH::CollideShapeResult& r) override {
        if (r.mShape2Face.size() < 3) return chain_and_void(r, forward_);
        const JPH::Vec3 tri_n = (r.mShape2Face[1] - r.mShape2Face[0]).Cross(r.mShape2Face[2] - r.mShape2Face[0]);
        const float tri_n_len = tri_n.Length();
        if (tri_n_len < 1e-6f) return chain_and_void(r, forward_);
        // Face contact: contact normal within 1 degree of the triangle normal.
        const float contact_n_len = r.mPenetrationAxis.Length();
        if (tri_n.Dot(-r.mPenetrationAxis) > 0.999848f * contact_n_len * tri_n_len) return chain_and_void(r, forward_);
        delayed_.push_back(Delayed{r, forward_});
    }

    // Must be called once after every body has been collided.
    void flush() {
        JPH::Array<JPH::uint, JPH::STLLocalAllocator<JPH::uint, kMaxLocalDelayed>> order;
        order.resize(delayed_.size());
        for (JPH::uint i = 0; i < JPH::uint(delayed_.size()); ++i) order[i] = i;
        JPH::QuickSort(order.begin(), order.end(), [this](JPH::uint a, JPH::uint b) {
            return delayed_[a].result.mPenetrationDepth > delayed_[b].result.mPenetrationDepth;
        });
        for (JPH::uint i = 0; i < JPH::uint(delayed_.size()); ++i) {
            const Delayed& d = delayed_[order[i]];
            const JPH::CollideShapeResult& r = d.result;
            // Closest vertex or edge of the face to the contact point.
            float best_dist_sq = FLT_MAX;
            JPH::uint best_v1 = 0;
            JPH::uint best_v2 = 0;
            const JPH::uint num_v = JPH::uint(r.mShape2Face.size());
            JPH::uint v1_idx = num_v - 1;
            JPH::Vec3 v1 = r.mShape2Face[v1_idx] - r.mContactPointOn2;
            for (JPH::uint v2_idx = 0; v2_idx < num_v; ++v2_idx) {
                const JPH::Vec3 v2 = r.mShape2Face[v2_idx] - r.mContactPointOn2;
                const JPH::Vec3 v1_v2 = v2 - v1;
                const float denom = v1_v2.LengthSq();
                if (denom < JPH::Square(FLT_EPSILON)) {
                    const float d1 = v1.LengthSq();
                    if (d1 < best_dist_sq) { best_dist_sq = d1; best_v1 = v1_idx; best_v2 = v1_idx; }
                } else {
                    const float fraction = -v1.Dot(v1_v2) / denom;
                    if (fraction < 1.0e-6f) {
                        const float d1 = v1.LengthSq();
                        if (d1 < best_dist_sq) { best_dist_sq = d1; best_v1 = v1_idx; best_v2 = v1_idx; }
                    } else if (fraction < 1.0f - 1.0e-6f) {
                        const float dc = (v1 + fraction * v1_v2).LengthSq();
                        if (dc < best_dist_sq) { best_dist_sq = dc; best_v1 = v1_idx; best_v2 = v2_idx; }
                    }
                }
                v1_idx = v2_idx;
                v1 = v2;
            }
            const bool voided = is_voided(r.mSubShapeID1, r.mShape2Face[best_v1]) &&
                                (best_v1 == best_v2 || is_voided(r.mSubShapeID1, r.mShape2Face[best_v2]));
            if (!voided && d.forward) chain(r);
            void_features(r);
        }
        voided_.clear();
        delayed_.clear();
    }

private:
    struct Voided {
        JPH::Float3 feature; // read with sLoadFloat3Unsafe, so must not be the last member
        JPH::SubShapeID sub_shape_id;
    };
    struct Delayed {
        JPH::CollideShapeResult result;
        bool forward;
    };

    bool is_voided(const JPH::SubShapeID& id, JPH::Vec3 v) const {
        for (const Voided& vf : voided_) {
            if (vf.sub_shape_id == id && v.IsClose(JPH::Vec3::sLoadFloat3Unsafe(vf.feature), vertex_tolerance_sq_)) return true;
        }
        return false;
    }
    void void_features(const JPH::CollideShapeResult& r) {
        for (const JPH::Vec3& v : r.mShape2Face) {
            if (!is_voided(r.mSubShapeID1, v)) {
                Voided vf;
                v.StoreFloat3(&vf.feature);
                vf.sub_shape_id = r.mSubShapeID1;
                voided_.push_back(vf);
            }
        }
    }
    void chain(const JPH::CollideShapeResult& r) {
        chained_.SetContext(GetContext());
        chained_.AddHit(r);
        UpdateEarlyOutFraction(chained_.GetEarlyOutFraction());
    }
    void chain_and_void(const JPH::CollideShapeResult& r, bool forward) {
        if (forward) chain(r);
        void_features(r);
    }

    JPH::CollideShapeCollector& chained_;
    JPH::Array<Voided, JPH::STLLocalAllocator<Voided, kMaxLocalVoided>> voided_;
    JPH::Array<Delayed, JPH::STLLocalAllocator<Delayed, kMaxLocalDelayed>> delayed_;
    float vertex_tolerance_sq_;
    bool forward_ = true;
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
        // Tile-seam welding for every body-vs-heightfield pair (see
        // TerrainSeamWeldingCollector); every other pair keeps Jolt's default.
        physics_system_.SetSimCollideBodyVsBody(
            [this](const JPH::Body& b1, const JPH::Body& b2, JPH::Mat44Arg t1, JPH::Mat44Arg t2, JPH::CollideShapeSettings& s,
                   JPH::CollideShapeCollector& c, const JPH::ShapeFilter& f) { collide_body_vs_body(b1, b2, t1, t2, s, c, f); });

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
        return add_body(desc, shape, raw_heightfield, std::holds_alternative<MeshShape>(desc.shape), nullptr);
    }

    // racing_game R2 (ps/backend/shape_handle.h): any thread, touches no
    // member - see create_jolt_shape_handle's comment.
    ShapeHandle create_shape(const ShapeDesc& desc) const override { return create_jolt_shape_handle(desc); }

    BodyId create_body(const BodyDesc& desc, const ShapeHandle& handle) override {
        if (!handle.valid()) throw std::invalid_argument("create_body: empty ShapeHandle");
        const auto* prepared = dynamic_cast<const JoltPreparedShape*>(handle.prepared().get());
        if (prepared == nullptr) throw std::invalid_argument("create_body: ShapeHandle was not built by the Jolt backend");
        const bool is_mesh = prepared->kind() == ShapeKind::Mesh;
        if (is_mesh && desc.motion != BodyMotionType::Static) {
            throw std::invalid_argument("create_body: a MeshShape handle needs a Static body (JPH::MeshShape::MustBeStatic)");
        }
        // O(1) in the shape's size: no shape is built here, the body just
        // takes a reference (bounds come from the BVH root).
        return add_body(desc, prepared->shape, nullptr, is_mesh, handle.prepared());
    }

    std::uint64_t debug_body_shape_digest(BodyId id) const override {
        const JPH::BodyID jid = lookup(id);
        const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(jid);
        assert(body != nullptr);
        const JPH::Shape* shape = body->GetShape();
        FnvStreamOut out(1469598103934665603ULL);
        JPH::Shape::ShapeToIDMap shape_map;
        JPH::Shape::MaterialToIDMap material_map;
        shape->SaveWithChildren(out, shape_map, material_map);
        std::uint64_t h = out.hash();
        JPH::PhysicsMaterialList mats;
        shape->SaveMaterialState(mats);
        h = fnv1a_val(h, static_cast<std::uint64_t>(mats.size()));
        for (const JPH::PhysicsMaterial* m : mats) h = fnv1a_val(h, surface_id_from_material(m));
        const JPH::Vec3 com = shape->GetCenterOfMass();
        const float com_f[3] = {com.GetX(), com.GetY(), com.GetZ()};
        h = fnv1a(h, com_f, sizeof(com_f));
        return h;
    }

private:
    // The one body-creation sequence shared by both create_body overloads:
    // identical Jolt calls in identical order whether `shape` was just built
    // from desc.shape or comes from a prebuilt handle, so a handle body is
    // bit-identical to the desc body (same BodyCreationSettings, same slot /
    // BodyId assignment, same seam-welding lists). `prepared` (handle path
    // only) is kept in the slot so the shape outlives every handle copy
    // until destroy_body.
    BodyId add_body(const BodyDesc& desc, const JPH::RefConst<JPH::Shape>& shape, JPH::HeightFieldShape* raw_heightfield, bool is_mesh,
                    std::shared_ptr<const PreparedShape> prepared) {
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
        slots_[id.index].prepared = std::move(prepared);
        slots_[id.index].wheel_contact = desc.wheel_contact;
        body_interface().SetUserData(jph_id, pack_body_id(id));
        if (raw_heightfield != nullptr || is_mesh) {
            terrain_surface_body_ids_.insert(std::lower_bound(terrain_surface_body_ids_.begin(), terrain_surface_body_ids_.end(), jph_id), jph_id);
        }
        if (is_mesh) {
            mesh_body_ids_.insert(std::lower_bound(mesh_body_ids_.begin(), mesh_body_ids_.end(), jph_id), jph_id);
        }
        return id;
    }

public:
    void destroy_body(BodyId id) override {
        const JPH::BodyID jid = lookup(id);
        const auto it = std::lower_bound(terrain_surface_body_ids_.begin(), terrain_surface_body_ids_.end(), jid);
        if (it != terrain_surface_body_ids_.end() && *it == jid) terrain_surface_body_ids_.erase(it);
        const auto mesh_it = std::lower_bound(mesh_body_ids_.begin(), mesh_body_ids_.end(), jid);
        if (mesh_it != mesh_body_ids_.end() && *mesh_it == jid) mesh_body_ids_.erase(mesh_it);
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

    // See i_rigid_backend.h's comment; applied in step() below.
    void set_integrate_sub_threshold_rotation(BodyId id, bool enable) override {
        assert(id.index < slots_.size() && slots_[id.index].alive && slots_[id.index].generation == id.generation && "stale or invalid BodyId");
        slots_[id.index].integrate_sub_threshold_rotation = enable;
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
        write_heightfield_content(*hf, heights, cell_surface_ids, heightfield_height_scratch_, heightfield_material_scratch_, *temp_allocator_);
    }

    // --- Terrain prefetch ring: detached spare heightfields (see
    // i_rigid_backend.h's doc comment on create_heightfield_spare). ---
    HeightfieldSpareId create_heightfield_spare(const HeightfieldShape& desc) override {
        // Exactly create_body's heightfield branch minus the body, so a
        // spare's inner shape is byte-identical to a pool body's created
        // from the same desc (same quantisation range, same material list).
        JPH::RefConst<JPH::HeightFieldShape> inner = make_inner_heightfield(desc);
        Spare sp;
        sp.inner = const_cast<JPH::HeightFieldShape*>(inner.GetPtr());
        sp.wrapper = JPH::RefConst<JPH::Shape>(new JPH::RotatedTranslatedShape(heightfield_wrapper_position(desc), kHeightfieldYupToZup, inner.GetPtr()));
        sp.alive = true;
        if (!spare_free_list_.empty()) {
            const std::uint32_t idx = spare_free_list_.back();
            spare_free_list_.pop_back();
            spares_[idx] = std::move(sp);
            return HeightfieldSpareId{idx};
        }
        spares_.push_back(std::move(sp));
        return HeightfieldSpareId{static_cast<std::uint32_t>(spares_.size() - 1)};
    }

    void destroy_heightfield_spare(HeightfieldSpareId spare) override {
        assert(spare.index < spares_.size() && spares_[spare.index].alive && "stale or invalid HeightfieldSpareId");
        spares_[spare.index] = Spare{};
        spare_free_list_.push_back(spare.index);
    }

    std::unique_ptr<IHeightfieldBuildContext> create_heightfield_build_context(std::uint32_t sample_count) override {
        return std::make_unique<JoltHeightfieldBuildContext>(sample_count);
    }

    // Reads only spares_[spare.index] (never resized while builds run - see
    // the interface's contract) and the caller's own context: no other
    // backend member is touched, so this is safe on any thread concurrently
    // with step() and with other spares' builds.
    void build_heightfield_spare(HeightfieldSpareId spare, const std::vector<real>& heights, const std::vector<SurfaceId>& cell_surface_ids,
                                 IHeightfieldBuildContext& ctx) override {
        assert(spare.index < spares_.size() && spares_[spare.index].alive && "stale or invalid HeightfieldSpareId");
        auto& c = static_cast<JoltHeightfieldBuildContext&>(ctx);
        write_heightfield_content(*spares_[spare.index].inner, heights, cell_surface_ids, c.heights, c.materials, c.temp);
    }

    void swap_heightfield_spare(BodyId id, HeightfieldSpareId spare) override {
        const JPH::BodyID jid = lookup(id);
        Slot& slot = slots_[id.index];
        assert(spare.index < spares_.size() && spares_[spare.index].alive && "stale or invalid HeightfieldSpareId");
        Spare& sp = spares_[spare.index];
        assert(slot.heightfield && "swap_heightfield_spare called on a body without a heightfield shape");
        assert(slot.heightfield->GetSampleCount() == sp.inner->GetSampleCount());
        JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(jid);
        assert(body != nullptr);
        // Keep the body's current wrapper alive in the spare (a refcount
        // bump, no allocation, nothing freed here).
        JPH::RefConst<JPH::Shape> previous(body->GetShape());
        // Every wrapper built from the same desc has the same centre of
        // mass (the wrapper position), so SetShapeInternal moves nothing;
        // no mass update (static body), no broadphase notify - the
        // caller's set_pose recomputes bounds and notifies once, exactly as
        // after update_heightfield_body.
        assert(previous->GetCenterOfMass() == sp.wrapper->GetCenterOfMass() && "spare built from a different HeightfieldShape desc than the body");
        body->SetShapeInternal(sp.wrapper.GetPtr(), false);
        sp.wrapper = std::move(previous);
        std::swap(slot.heightfield, sp.inner);
    }

    std::uint64_t debug_heightfield_digest(BodyId id) const override {
        (void)lookup(id);
        const JPH::HeightFieldShape* hf = slots_[id.index].heightfield;
        assert(hf && "debug_heightfield_digest called on a body without a heightfield shape");
        return heightfield_digest(*hf);
    }

    std::uint64_t debug_heightfield_spare_digest(HeightfieldSpareId spare) const override {
        assert(spare.index < spares_.size() && spares_[spare.index].alive && "stale or invalid HeightfieldSpareId");
        return heightfield_digest(*spares_[spare.index].inner);
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
        integrate_sub_threshold_rotation(dt);
    }

    // Parking-slope creep fix (defect D2, i_rigid_backend.h's own comment on
    // set_integrate_sub_threshold_rotation). Reproduces exactly what Jolt's
    // own Body::AddRotationStep (Jolt/Physics/Body/Body.inl) would have
    // applied this Update, had it not silently dropped any sub-step whose
    // |omega * dt_substep| <= 1.0e-6f rad: PhysicsSystem::Update calls
    // AddRotationStep once per collision step, each time with
    // GetAngularVelocity() * dt_substep (PhysicsSystem.cpp's own
    // JobIntegrateVelocity, "body.AddRotationStep(body.GetAngularVelocity()
    // * delta_time)"). Single-threaded, right after Update and before any
    // query/job can observe an in-between state - see the header comment
    // for why this must never run inside a job.
    //
    // Approximation: omega is read ONCE (its post-Update value) and treated
    // as constant across all collision_steps_ sub-steps, so the total
    // missed rotation is just that one sub-step's dropped angle times
    // collision_steps_, about the same fixed axis - exact whenever omega
    // does not change during the Update (the common case this fix targets:
    // a body sitting at a small, steady angular velocity with no net
    // torque), and a good approximation otherwise since only sub-threshold
    // (near-zero) angular velocities are affected at all.
    void integrate_sub_threshold_rotation(real dt) {
        bool any = false;
        for (const Slot& s : slots_) {
            if (s.alive && s.integrate_sub_threshold_rotation) { any = true; break; }
        }
        if (!any) return;

        const float dt_s = static_cast<float>(dt) / static_cast<float>(collision_steps_);
        for (const Slot& s : slots_) {
            if (!s.alive || !s.integrate_sub_threshold_rotation) continue;

            const JPH::Vec3 omega = body_interface().GetAngularVelocity(s.jph_id);
            const JPH::Vec3 av_dt_s = omega * dt_s; // exactly Jolt's own AddRotationStep argument, per sub-step
            const float len = av_dt_s.Length();
            if (!(len > 0.0f && len <= 1.0e-6f)) continue; // 0: no rotation to apply; > 1e-6f: Jolt already integrated it itself

            const JPH::Quat delta = JPH::Quat::sRotation(av_dt_s / len, len * static_cast<float>(collision_steps_));
            const JPH::Quat q_old = body_interface().GetRotation(s.jph_id);
            const JPH::Quat q_new = (delta * q_old).Normalized();

            const JPH::RVec3 com = body_interface().GetCenterOfMassPosition(s.jph_id);
            const JPH::Body* body = physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(s.jph_id);
            const JPH::Vec3 local_com = body != nullptr ? body->GetShape()->GetCenterOfMass() : JPH::Vec3::sZero();
            const JPH::RVec3 new_origin = com - (q_new * local_com); // RVec3 - Vec3 is defined directly (Jolt Math/DVec3.h)

            body_interface().SetPositionAndRotation(s.jph_id, new_origin, q_new, JPH::EActivation::DontActivate);
        }
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

    RayCastHit ray_cast_excluding(const Vec3& origin,const Vec3& direction,real max_distance,BodyId excluded) const override {
        JPH::RRayCast ray(to_rvec3(origin),to_vec3f(direction.normalized()*max_distance));
        JPH::RayCastResult result;
        const JPH::IgnoreSingleBodyFilter filter(lookup(excluded));
        RayCastHit out;
        if(physics_system_.GetNarrowPhaseQuery().CastRay(ray,result,{}, {},filter)) {
            const auto* body=physics_system_.GetBodyLockInterfaceNoLock().TryGetBody(result.mBodyID);
            if(body) {
                out.hit=true; out.body=unpack_body_id(body->GetUserData()); out.fraction=real(result.mFraction);
                out.point=from_rvec3(ray.GetPointOnRay(result.mFraction));
                out.normal=from_vec3f(body->GetWorldSpaceSurfaceNormal(result.mSubShapeID2,ray.GetPointOnRay(result.mFraction)));
            }
        }
        return out;
    }

    // Wheel casts skip bodies whose Slot::wheel_contact is false. Reads slots_ only (written at body creation / set_wheel_contact between
    // steps), allocation-free, safe from the wheel jobs.
    void set_wheel_contact(BodyId id, bool value) override { slots_[id.index].wheel_contact = value; }

    ShapeCastHit shape_cast(const ShapeDesc& shape, const Pose& start_pose, const Vec3& sweep_world, real max_distance,
                            bool skip_non_wheel_bodies) const override {
        const WheelBodyFilter wheel_filter(slots_);
        const JPH::BodyFilter& body_filter = skip_non_wheel_bodies ? static_cast<const JPH::BodyFilter&>(wheel_filter) : static_cast<const JPH::BodyFilter&>(default_body_filter_);
        return with_query_shape(shape, [&](const JPH::Shape& jsh) -> ShapeCastHit {
            JPH::RMat44 start = JPH::RMat44::sRotationTranslation(to_jph_quat(start_pose.orientation), to_rvec3(start_pose.position));
            JPH::RShapeCast cast = JPH::RShapeCast::sFromWorldTransform(&jsh, JPH::Vec3::sReplicate(1.0f), start, to_vec3f(sweep_world.normalized() * max_distance));
            JPH::ShapeCastSettings settings;
            JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
            physics_system_.GetNarrowPhaseQuery().CastShape(cast, settings, to_rvec3(start_pose.position), collector, {}, {}, body_filter);
            // D0 (dog release spike): with the default settings Jolt's GJK cast of a cylinder that is nearly axis
            // aligned against a very large thin box (the 1000 m test-rig ground) occasionally reports a phantom -
            // fraction 0 / "started penetrating" (penetration 0.18 m with 0.07 m true clearance) or a miss with the
            // ground inside the sweep [M: 3 wrong of 200000 near-aligned random wheel casts]. A wheel fed that hit
            // jumps to full bump for one substep (xdot ~70 m/s, Fz ~540 kN, hub +1300 rad/s, the car kicked). Only
            // such a SUSPICIOUS result is re-cast with the shrunken shape plus convex radius (Jolt's own
            // VehicleCollisionTesterCastCylinder mode) and replaced when that cast finds a clean hit (fraction > 0);
            // a genuine start-in-penetration or a genuine miss stays as it was. Every non-suspicious cast is
            // bit-identical to before [M: 0 wrong of 200000 with the re-cast]; of the shipped scenarios only
            // vehicle_tractor_i4.json met one (a rear wheel: fraction 0, penetration 0.186 m, true fraction 0.5005).
            if (!collector.HadHit() || collector.mHit.mFraction <= 0.0f) {
                JPH::ShapeCastSettings recheck_settings;
                recheck_settings.mUseShrunkenShapeAndConvexRadius = true;
                JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> recheck;
                physics_system_.GetNarrowPhaseQuery().CastShape(cast, recheck_settings, to_rvec3(start_pose.position), recheck, {}, {}, body_filter);
                if (recheck.HadHit() && recheck.mHit.mFraction > 0.0f) collector = recheck;
            }
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
                // PHYS-033 I1b C4, additive: same base-offset convention as
                // collide_shape_into's own mContactPointOn2 use below
                // (base offset = start_pose.position, matching the CastShape
                // call's own inBaseOffset argument just above).
                out.contact_point = start_pose.position + from_vec3f(collector.mHit.mContactPointOn2);
                out.penetration_depth = real(collector.mHit.mPenetrationDepth);
                out.started_penetrating = collector.mHit.mFraction <= 0.0f;
            }
            return out;
        });
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
        with_query_shape(shape, [&](const JPH::Shape& jsh) -> void {
            JPH::RMat44 transform = JPH::RMat44::sRotationTranslation(to_jph_quat(pose.orientation), to_rvec3(pose.position));
            JPH::CollideShapeSettings settings;
            JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
            physics_system_.GetNarrowPhaseQuery().CollideShape(&jsh, JPH::Vec3::sReplicate(1.0f), transform, settings,
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
        });
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
        // Parking-slope creep fix (defect D2, i_rigid_backend.h's own
        // comment on set_integrate_sub_threshold_rotation): off by default,
        // reset to off whenever a slot is (re)allocated (alloc_slot below
        // always aggregate-inits a Slot, so a reused index never inherits
        // the previous occupant's opt-in).
        bool integrate_sub_threshold_rotation = false;
        bool wheel_contact = true; // BodyDesc::wheel_contact
        // Non-null only for a body created from a ShapeHandle (racing_game
        // R2): keeps the prepared shape alive while the body exists, so
        // releasing every handle copy never frees a shape in use. Released
        // in free_slot, AFTER DestroyBody dropped the body's own reference.
        std::shared_ptr<const PreparedShape> prepared{};
    };
    class WheelBodyFilter final : public JPH::BodyFilter {
    public:
        explicit WheelBodyFilter(const std::vector<Slot>& slots) : slots_(slots) {}
        bool ShouldCollideLocked(const JPH::Body& body) const override {
            const BodyId id = unpack_body_id(body.GetUserData());
            return id.index >= slots_.size() || slots_[id.index].wheel_contact;
        }
    private:
        const std::vector<Slot>& slots_;
    };

    JPH::BodyFilter default_body_filter_;
    struct JointSlot {
        JPH::Ref<JPH::Constraint> constraint;
        bool alive = false;
    };
    // A detached heightfield (create_heightfield_spare): `wrapper` owns the
    // RotatedTranslatedShape (and through it `inner`); after a swap it holds
    // whichever shape the body carried before.
    struct Spare {
        JPH::RefConst<JPH::Shape> wrapper;
        JPH::HeightFieldShape* inner = nullptr;
        bool alive = false;
    };

    // PhysicsSystem's SimCollideBodyVsBody hook (installed in the
    // constructor), called from Jolt's narrow-phase jobs during Update.
    // terrain_surface_body_ids_ and every static body's pose/bounds are only
    // mutated outside Update (create/destroy/set_pose/update_heightfield_body
    // run in World stages other than backend_step), so reading them here
    // unlocked is race-free. t1/t2 are relative to body1's COM
    // (PhysicsSystem::ProcessBodyPair); neighbours use the same convention.
    void collide_body_vs_body(const JPH::Body& b1, const JPH::Body& b2, JPH::Mat44Arg t1, JPH::Mat44Arg t2, JPH::CollideShapeSettings& settings,
                              JPH::CollideShapeCollector& collector, const JPH::ShapeFilter& filter) const {
        if (!std::binary_search(terrain_surface_body_ids_.begin(), terrain_surface_body_ids_.end(), b2.GetID())) {
            JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(b1, b2, t1, t2, settings, collector, filter);
            return;
        }
        JPH::AABox query = b1.GetWorldSpaceBounds();
        query.ExpandBy(JPH::Vec3::sReplicate(settings.mMaxSeparationDistance));
        const auto& bodies = physics_system_.GetBodyLockInterfaceNoLock();
        const bool mesh_pair = std::binary_search(mesh_body_ids_.begin(), mesh_body_ids_.end(), b2.GetID());
        if (mesh_pair) {
            bool touches_heightfield = false;
            for (const JPH::BodyID id : terrain_surface_body_ids_) {
                if (std::binary_search(mesh_body_ids_.begin(), mesh_body_ids_.end(), id)) continue;
                const auto* other = bodies.TryGetBody(id);
                if (other && other->IsInBroadPhase() && other->GetWorldSpaceBounds().Overlaps(query)) {
                    touches_heightfield = true;
                    break;
                }
            }
            if (!touches_heightfield) {
                JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(b1, b2, t1, t2, settings, collector, filter);
                return;
            }
        }
        settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
        TerrainSeamWeldingCollector weld(collector, kTerrainSeamVertexToleranceSq);
        const JPH::Vec3 one = JPH::Vec3::sOne();
        const JPH::AABox own = b2.GetWorldSpaceBounds();
        const bool inside_xy = query.mMin.GetX() > own.mMin.GetX() && query.mMax.GetX() < own.mMax.GetX() &&
                               query.mMin.GetY() > own.mMin.GetY() && query.mMax.GetY() < own.mMax.GetY();
        bool touches_mesh = mesh_pair;
        if (inside_xy && !mesh_pair) {
            // A mesh may border a suppressed hole INSIDE a heightfield AABB.
            // Only use the old interior fast path if no mesh overlaps the query.
            for (const JPH::BodyID id : mesh_body_ids_) {
                const auto* other = bodies.TryGetBody(id);
                if (other && other->IsInBroadPhase() && other->GetWorldSpaceBounds().Overlaps(query)) {
                    touches_mesh = true;
                    break;
                }
            }
        }
        if (inside_xy && !touches_mesh) {
            JPH::SubShapeIDCreator p1, p2;
            weld.set_forward(true);
            JPH::CollisionDispatch::sCollideShapeVsShape(b1.GetShape(), b2.GetShape(), one, one, t1, t2, p1, p2, settings, weld, filter);
            weld.flush();
            return;
        }

        // Ascending BodyID order, so the pairs (b1, A) and (b1, B) both see
        // the identical hit sequence and make identical voiding decisions.
        const JPH::RVec3 offset = b1.GetCenterOfMassPosition();
        const JPH::ShapeFilter accept_all;
        for (const JPH::BodyID id : terrain_surface_body_ids_) {
            const bool self = (id == b2.GetID());
            const JPH::Body* other = self ? &b2 : bodies.TryGetBody(id);
            if (!self && (other == nullptr || !other->IsInBroadPhase() || !other->GetWorldSpaceBounds().Overlaps(query))) continue;
            const JPH::Mat44 t_other = self ? JPH::Mat44(t2) : other->GetCenterOfMassTransform().PostTranslated(-offset).ToMat44();
            JPH::SubShapeIDCreator p1, p2;
            weld.set_forward(self);
            JPH::CollisionDispatch::sCollideShapeVsShape(b1.GetShape(), other->GetShape(), one, one, t1, t_other, p1, p2, settings, weld,
                                                         self ? filter : accept_all);
            if (weld.ShouldEarlyOut()) break;
        }
        weld.flush();
    }

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
        slots_[id.index].prepared.reset(); // may free the shape: DestroyBody already released the body's reference
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
    // Every live heightfield or static mesh body, ascending - collide_body_vs_body's
    // seam-welding candidate set and its canonical collide order.
    std::vector<JPH::BodyID> terrain_surface_body_ids_;
    std::vector<JPH::BodyID> mesh_body_ids_;

    // update_heightfield_body's scratch conversion buffers - see that
    // method's comment.
    std::vector<float> heightfield_height_scratch_;
    std::vector<std::uint8_t> heightfield_material_scratch_;

    // Terrain prefetch ring spares (create_heightfield_spare). Declared
    // after physics_system_, so destroyed before it - they are plain shapes
    // no body references, independent of the physics system.
    std::vector<Spare> spares_;
    std::vector<std::uint32_t> spare_free_list_;
};

} // namespace

std::unique_ptr<IRigidBackend> create_jolt_backend(jobs::JobSystem& job_system, const Vec3& gravity, std::size_t max_bodies,
                                                     const RigidBackendSettings& settings) {
    return std::make_unique<JoltRigidBackend>(job_system, gravity, max_bodies, settings);
}

} // namespace ps
