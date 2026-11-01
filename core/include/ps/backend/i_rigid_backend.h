// ps/backend/i_rigid_backend.h — backend seam (PLAN.md D2).
//
// create/destroy bodies, shapes (via our own descriptors, or an opaque
// prebuilt ps::ShapeHandle - never a backend shape type), joints; apply wrench; step; ray/shape casts; contacts.
// No Jolt type appears anywhere in this header - the only implementation in
// P1 is core/src/backend/jolt/jolt_backend.cpp, reached only through
// create_jolt_backend() below, which returns the abstract interface.

#pragma once

#include "ps/backend/shape_desc.h"
#include "ps/backend/shape_handle.h"
#include "ps/jobs/job_system.h"
#include "ps/math/mat3.h"
#include "ps/math/pose.h"
#include "ps/world/ids.h"

#include <cstdint>
#include <memory>
#include <variant>
#include <vector>

namespace ps {

enum class BodyMotionType { Static, Kinematic, Dynamic };

// Per-body continuous-collision-detection quality (JoltPhysics
// EMotionQuality, Physics/Body/MotionQuality.h - kept as our own enum so no
// Jolt type crosses the backend seam, PLAN.md D2). Discrete is Jolt's own
// default (cheapest; can tunnel through thin shapes at high velocity);
// LinearCast sweeps the shape from start to destination each step so it
// cannot tunnel through thin static geometry (but a long thin body can still
// tunnel via rotation - see Jolt's own doc comment on EMotionQuality).
enum class MotionQuality { Discrete, LinearCast };

struct BodyDesc {
    ShapeDesc shape = SphereShape{};
    BodyMotionType motion = BodyMotionType::Dynamic;
    real mass = 1.0;              // ignored for Static/Kinematic
    // A wheel (shape_cast with skip_non_wheel_bodies) may stand on this body. true = today's behaviour. false: the body is invisible to wheel
    // casts - a parked machine's collision box, a woken vehicle, a bale: the chassis/collision shapes still meet it, only the wheel does not
    // read its side or top as ground. Read in the wheel jobs, so set at creation (or between steps with set_wheel_contact), never mid-step.
    bool wheel_contact = true;
    Pose pose = Pose::identity();
    Motion velocity{};
    real friction = 0.5;
    real restitution = 0.0;
    real linear_damping = 0.0;
    real angular_damping = 0.0;
    bool gravity_enabled = true;
    // Allow the body to sleep/deactivate. Off by default: determinism tests
    // step bodies that may sit still (box on slope below the friction
      // angle) and must keep producing state, not silently stop updating.
    bool allow_sleep = false;

    // Continuous collision detection quality (see MotionQuality above).
    // Default matches JoltPhysics' own BodyCreationSettings default
    // (Discrete) - additive field, existing behaviour unchanged.
    MotionQuality motion_quality = MotionQuality::Discrete;

    // Maximum angular speed this body can reach (rad/s); Jolt clamps to this
    // every step (MotionProperties::mMaxAngularVelocity). Default equals
    // JoltPhysics' own BodyCreationSettings default, 0.25*pi*60 rad/s
    // (Physics/Body/BodyCreationSettings.h) - additive field, existing
    // behaviour unchanged.
    real max_angular_velocity = 47.123889803846896; // 0.25 * pi * 60

    // Maximum linear speed this body can reach (m/s); Jolt silently clamps
    // to this every step (MotionProperties::mMaxLinearVelocity). Default
    // equals JoltPhysics' own BodyCreationSettings default, 500 m/s
    // (Physics/Body/BodyCreationSettings.h) - additive field, existing
    // behaviour unchanged. Found relevant during the Part A energy study: a
    // body in free fall for long enough / hit hard enough exceeds 500 m/s
    // and the clamp then silently removes kinetic energy every step it is
    // active, which looks like unexplained energy loss unless traced back
    // to this default.
    real max_linear_velocity = 500.0;
};

enum class JointType { Fixed, Hinge };

struct JointDesc {
    JointType type = JointType::Fixed;
    BodyId body_a;                          // must be valid
    BodyId body_b;                          // invalid == anchored to the world
    Vec3 anchor_a_local = Vec3::zero();      // attach point, body A local space
    Vec3 anchor_b_local = Vec3::zero();      // attach point, body B local space (or world space if body_b invalid)
    Vec3 axis_a_local = Vec3::unit_z();      // hinge axis, body A local space
    Vec3 axis_b_local = Vec3::unit_z();      // hinge axis, body B local space (or world space if body_b invalid)
    real friction_torque = 0.0;              // hinge only; 0 == frictionless
    bool limits_enabled = false;             // hinge only
    real limit_min_rad = 0.0;
    real limit_max_rad = 0.0;
};

struct RayCastHit {
    bool hit = false;
    BodyId body;
    real fraction = 1.0;
    Vec3 point;
    Vec3 normal;
};

struct ShapeCastHit {
    bool hit = false;
    BodyId body;
    real fraction = 1.0;
    // The cast shape's ORIGIN (its centre of mass for Box/Sphere/Capsule/
    // Cylinder) at the hit: start + fraction*sweep. NOT a surface point
    // (PHYS-033 I1b) - kept exactly as before for every existing caller
    // (ps::vehicle::WheelInstance::measure_contact and
    // tests/unit/test_mesh_heightfield_shapes.cpp both rely on this meaning).
    Vec3 point;
    // Contact normal pointing from the hit body toward the cast shape
    // (-penetration axis). Unreliable when started_penetrating below is true.
    Vec3 normal;
    // Surface id of the hit triangle/cell of `body`'s shape (PLAN.md M1),
    // kInvalidSurfaceId if that shape carries no surface ids or there was
    // no hit.
    SurfaceId surface = kInvalidSurfaceId;
    // PHYS-033 I1b, additive: a point on the HIT body's surface at impact
    // (Jolt's mContactPointOn2 + the cast's own base offset). For a line or
    // face contact this is ONE point of the contact set, arbitrary along it
    // (Jolt's GJK does not guarantee which) - only its component along
    // `normal` is well defined, which is all ps::vehicle's D-A patch-point
    // computation relies on.
    Vec3 contact_point;
    real penetration_depth = 0.0; // > 0 only when started_penetrating below
    // True when the cast shape already overlapped the hit body at the start
    // of the sweep (Jolt's own mFraction <= 0): `normal`/`contact_point` come
    // from GJK without an EPA pass in that case and are not trustworthy -
    // callers should fall back to a known-good direction (e.g. the sweep's
    // own axis) instead of trusting them.
    bool started_penetrating = false;
};

struct ContactEvent {
    BodyId a;
    BodyId b;
    Vec3 point;
    Vec3 normal;
    real normal_impulse = 0.0;
    // Surface id of the triangle/cell of a's shape (kInvalidSurfaceId if a's
    // shape is not a MeshShape/HeightfieldShape, or carries no surface ids)
    // and of b's shape, respectively - PLAN.md M1's "surface ids must survive
    // collision queries and contact events, so the tyre/contact code can
    // look up a surface".
    SurfaceId surface_a = kInvalidSurfaceId;
    SurfaceId surface_b = kInvalidSurfaceId;
};

// One overlap of a queried shape against a live body, from collide_shape_into
// below (PLAN.md P1.5b, strategy A: "compliant contact force elements... Jolt
// used only as a collision query"). `normal` points away from `body`'s
// surface, toward the queried shape - i.e. the direction that resolves the
// overlap (same convention shape_cast's own doc comment above uses for its
// penetration axis). `penetration` is the overlap depth (> 0 while
// overlapping); point is a representative world-space contact point.
struct ShapeContactHit {
    BodyId body;
    Vec3 point;
    Vec3 normal;
    real penetration = 0.0;
    // Surface id of the triangle/cell of `body`'s shape at `point`
    // (kInvalidSurfaceId if that shape carries no surface ids, e.g. a plain
    // Box/Sphere/Capsule) - PLAN.md M1.
    SurfaceId surface = kInvalidSurfaceId;
};

// World-space XY bounding rectangle of one collision triangle - see
// IRigidBackend::body_triangle_xy_bounds_into.
struct XyBounds {
    real min_x = 0.0;
    real min_y = 0.0;
    real max_x = 0.0;
    real max_y = 0.0;
};

// See IRigidBackend::get_integration_params below. Zero inverse mass for a
// static/missing body.
struct BodyIntegrationParams {
    real inv_mass = 0.0;
    Mat3 inv_inertia_world = Mat3::zero(); // at the body's CURRENT rotation (Jolt uses the rotation at step START)
    real gravity_factor = 1.0;
    real linear_damping = 0.0;
    real angular_damping = 0.0;
    bool gyroscopic = false;
};

// Terrain prefetch ring (IRigidBackend's heightfield spare API below): an
// opaque handle to a detached HeightfieldShape (no body references it while
// it is a spare) and an opaque per-builder-thread scratch context.
struct HeightfieldSpareId {
    std::uint32_t index = 0xFFFFFFFFu;
    [[nodiscard]] constexpr bool valid() const { return index != 0xFFFFFFFFu; }
    constexpr bool operator==(const HeightfieldSpareId&) const = default;
};
class IHeightfieldBuildContext {
public:
    virtual ~IHeightfieldBuildContext() = default;
};

class IRigidBackend {
public:
    virtual ~IRigidBackend() = default;

    virtual BodyId create_body(const BodyDesc& desc) = 0;
    virtual void destroy_body(BodyId id) = 0;

    // Prebuilt shapes (racing_game R2) - full contract in
    // ps/backend/shape_handle.h. create_shape is const and touches no
    // backend instance state: safe on ANY thread, concurrently with step(),
    // with body creation/destruction and with other create_shape calls.
    // Throws std::invalid_argument for a HeightfieldShape, std::runtime_error
    // when the backend rejects the shape. Allocates (never call it from a
    // ps::jobs job body).
    [[nodiscard]] virtual ShapeHandle create_shape(const ShapeDesc& desc) const = 0;
    // Sim thread, between steps (like create_body(desc)). Exactly
    // create_body(desc) with desc.shape replaced by the shape `shape` was
    // built from - same BodyId sequence, bit-identical body; desc.shape is
    // ignored. O(1) in the shape's size. The body keeps the shape alive
    // until destroy_body. Throws std::invalid_argument for an empty handle,
    // a handle another backend type built, or a MeshShape handle on a
    // non-Static body.
    virtual BodyId create_body(const BodyDesc& desc, const ShapeHandle& shape) = 0;

    // Test-only (allocates): FNV-1a over the backend's own binary
    // serialisation of the body's collision shape (Jolt SaveWithChildren -
    // for a mesh its whole BVH/triangle buffer) plus the surface ids of its
    // materials. Equal digests == byte-identical shapes.
    [[nodiscard]] virtual std::uint64_t debug_body_shape_digest(BodyId id) const = 0;

    virtual JointId create_joint(const JointDesc& desc) = 0;
    virtual void destroy_joint(JointId id) = 0;

    [[nodiscard]] virtual Pose get_pose(BodyId id) const = 0;
    [[nodiscard]] virtual Motion get_motion(BodyId id) const = 0;
    virtual void set_pose(BodyId id, const Pose& pose) = 0;
    virtual void set_motion(BodyId id, const Motion& motion) = 0;

    // Parking-slope creep fix (defect D2): Jolt's own Body::AddRotationStep
    // (Jolt/Physics/Body/Body.inl) silently DROPS any rotation step whose
    // angular displacement this sub-step, |omega * dt_substep|, is <=
    // 1.0e-6f rad - at a 240 Hz tick with the default 1 collision step per
    // Update, that is any |omega| <~ 2.4e-4 rad/s. A body can then carry a
    // small but real, steady angular velocity (as reported by
    // get_motion/GetAngularVelocity) that never actually integrates into
    // its orientation - a "rolling without rotating" mode nothing else in
    // the pipeline resists (a vehicle's own shadow-chassis kinematics and
    // tyre contact-velocity sampling DO use the real omega, so a small
    // steady chassis pitch/roll rate can cancel a wheel's translational
    // contact-patch velocity and make a parked car creep with no tyre force
    // ever opposing it).
    //
    // Default OFF for every body (existing behaviour, and every existing
    // resting-body scenario hash, is unchanged). A caller that opts a body
    // in gets JoltRigidBackend::step() reapplying the exact rotation Jolt's
    // own AddRotationStep would have applied absent the 1e-6f dead zone,
    // computed with Jolt's own Quat::sRotation right after
    // PhysicsSystem::Update and before any query - single-threaded, never
    // from inside a job (moving a body from inside vehicle_islands_
    // substep's parallel_for while another vehicle instance's job might
    // shape-cast against it would risk worker-count nondeterminism).
    // World::create_vehicle opts the chassis body in; nothing else does.
    virtual void set_integrate_sub_threshold_rotation(BodyId id, bool enable) = 0;

    // True for a Static body (BodyDesc::motion at creation), false for
    // Kinematic/Dynamic and for an invalid/stale id. PLAN.md M1: lets a
    // caller distinguish "another STATIC body already covers this point"
    // (ps::terrain::TileManager's terrain/static-feature coincident-
    // collision fix) from a dynamic body that merely happens to be passing
    // through at query time - the latter must never be treated as
    // permanent ground coverage.
    [[nodiscard]] virtual bool is_static(BodyId id) const = 0;

    // Appends, into caller-owned `out` (out.clear()'d first), the
    // world-space XY bounding rectangle of every triangle of `id`'s
    // collision shape at its current pose. Convex primitives are
    // triangulated by the backend. Nothing is appended for an invalid/stale
    // id. Load-time only (it allocates): ps::terrain::TileManager's
    // suppression precompute uses it to bin static feature geometry to
    // terrain tile keys from the actual triangles, not a body's overall
    // AABB (one surface-grouped mesh body can span the whole world).
    virtual void body_triangle_xy_bounds_into(BodyId id, std::vector<XyBounds>& out) const = 0;

    // Updates an EXISTING HeightfieldShape-backed body's height/surface
    // samples in place - PLAN.md M1's tile system reuses a fixed pool of
    // bodies across many different tiles over a vehicle's lifetime
    // (create_body/destroy_body per tile-crossing would allocate a brand
    // new Jolt shape every time, which PLAN.md D8's "no allocation inside
    // jobs" forbids on a path the 240 Hz tick can reach - see
    // JoltRigidBackend's implementation for how this instead reuses the
    // backend's own preallocated TempAllocator arena). `heights` and
    // `cell_surface_ids` must match the sample_count the body's shape was
    // originally created with (HeightfieldShape::sample_count) exactly -
    // this call changes CONTENT only, never resolution or position;
    // reposition the tile with set_pose (same call every caller already
    // needs for a moved/reused body) and change resolution by
    // destroy_body+create_body instead (an infrequent, off-hot-path
    // operation - e.g. a config change, never a tile-streaming event).
    // `cell_surface_ids` may be empty to mean kInvalidSurfaceId everywhere,
    // same convention as HeightfieldShape itself. Must only be called for a
    // body created with a HeightfieldShape; behaviour is otherwise
    // undefined.
    virtual void update_heightfield_body(BodyId id, const std::vector<real>& heights, const std::vector<SurfaceId>& cell_surface_ids) = 0;

    // Terrain prefetch ring (owner-approved 2026-09-27): build a tile's
    // heightfield content OFF the tick, then install it in the tick with a
    // pointer swap instead of update_heightfield_body's in-tick upload.
    //
    // A SPARE is a detached heightfield shape, created at load time
    // (allocates) from a desc that must be identical to the one the pool
    // bodies it will be swapped into were created from (same sample_count,
    // sample_spacing, origin and placeholder heights - the placeholder fixes
    // Jolt's quantisation range, see ps/terrain/terrain_source.h's TileSample
    // comment). create/destroy_heightfield_spare must not run concurrently
    // with any build_heightfield_spare call.
    //
    // build_heightfield_spare writes `heights`/`cell_surface_ids` into the
    // spare exactly like update_heightfield_body writes into a body's shape
    // (same row reversal, same SetHeights/SetMaterials calls), using only the
    // caller's own build context - safe on ANY thread, concurrently with the
    // backend's own step and with builds of OTHER spares, as long as the
    // spare is not being swapped at the same time. Allocation-free once the
    // context exists. The result is a pure function of the inputs (the
    // spare's previous content does not matter - tests/unit/
    // test_heightfield_spare.cpp pins this through debug_heightfield_digest).
    //
    // swap_heightfield_spare(body, spare) (sim thread, between steps or in
    // the terrain_tiles stage, never during step) exchanges the body's
    // heightfield with the spare's: the body now carries the freshly built
    // shape, the spare handle now holds the body's previous shape (kept
    // alive, never freed here - no deallocation in the tick). It does NOT
    // notify the broadphase; the caller must follow it with set_pose (the
    // same call update_heightfield_body's callers already make), which
    // recomputes the world bounds and notifies once - exactly today's
    // update_heightfield_body + set_pose sequence, so state_hash, contact
    // order and every query result are identical to the in-tick upload.
    [[nodiscard]] virtual HeightfieldSpareId create_heightfield_spare(const HeightfieldShape& desc) = 0;
    virtual void destroy_heightfield_spare(HeightfieldSpareId spare) = 0;
    // One per builder thread; owns its own Jolt temp allocator and conversion
    // scratch for tiles of `sample_count` samples per edge (allocates).
    [[nodiscard]] virtual std::unique_ptr<IHeightfieldBuildContext> create_heightfield_build_context(std::uint32_t sample_count) = 0;
    virtual void build_heightfield_spare(HeightfieldSpareId spare, const std::vector<real>& heights, const std::vector<SurfaceId>& cell_surface_ids,
                                         IHeightfieldBuildContext& ctx) = 0;
    virtual void swap_heightfield_spare(BodyId id, HeightfieldSpareId spare) = 0;
    // Test-only (allocation-free, but not meant for the tick): FNV-1a over
    // Jolt's own binary serialisation of the body's (or spare's) inner
    // HeightFieldShape - every quantised sample, range block, active-edge
    // bit and material index - plus its material list's surface ids.
    [[nodiscard]] virtual std::uint64_t debug_heightfield_digest(BodyId id) const = 0;
    [[nodiscard]] virtual std::uint64_t debug_heightfield_spare_digest(HeightfieldSpareId spare) const = 0;

    // Queried fresh every call, never cached by a caller (PLAN.md section 6:
    // mass properties come from the body every tick, ready for a future
    // body whose mass changes, e.g. fuel burn, loader bucket fill).
    [[nodiscard]] virtual real get_mass(BodyId id) const = 0;
    [[nodiscard]] virtual Mat3 get_inertia_world(BodyId id) const = 0; // about world-space COM

    // Accumulates a force+torque (about the body's world-space COM) for the
    // step about to run; cleared after step(). Must only be called from
    // World's sequential, fixed-order gather stage (PLAN.md D8) - never
    // concurrently for the same body.
    virtual void apply_wrench(BodyId id, const Wrench& wrench_at_com) = 0;

    // Exactly what the backend's own velocity integrator will apply to this
    // body in the NEXT step() (PHYS-033 I1 W1: VehicleInstance's shadow
    // chassis reproduces Jolt's own tick-start-inertia, no-gyroscopic-term,
    // gravity-factor/damping-aware velocity update - see the I1 plan
    // section 2's numbered conditions and jolt_backend.cpp's implementation
    // for the exact fields each one maps to). Read-only, no side effects -
    // safe to call from the vehicle job like get_mass/get_inertia_world.
    [[nodiscard]] virtual BodyIntegrationParams get_integration_params(BodyId id) const = 0;

    // Advances the backend by exactly dt (PLAN.md D3: fixed-rate stepping;
    // no wall clock is read here or anywhere in the backend).
    virtual void step(real dt) = 0;

    // Pure narrowphase query (no side effects) - same concurrent-read-only
    // safety as collide_shape_into below: safe to call from multiple threads
    // at once as long as the backend is not concurrently stepping and no
    // body is being created/destroyed/moved at the same time. (PLAN.md M1's
    // tile-suppression fix, ps::terrain::TileManager::suppress_covered_samples,
    // measured calling this from inside a parallel_batches job and found no
    // benefit for its own workload - see that function's own header comment
    // point (d) - but the safety guarantee itself held and may still be
    // useful elsewhere.)
    [[nodiscard]] virtual RayCastHit ray_cast(const Vec3& origin, const Vec3& direction, real max_distance) const = 0;
    [[nodiscard]] virtual RayCastHit ray_cast_excluding(const Vec3& origin,const Vec3& direction,real distance,BodyId excluded) const {
        auto hit=ray_cast(origin,direction,distance);
        return hit.hit&&hit.body==excluded ? RayCastHit{} : hit;
    }
    // skip_non_wheel_bodies: bodies created with BodyDesc::wheel_contact == false are not hit (the wheel path; the D0 re-cast uses the same
    // filter). false = every body, as before.
    [[nodiscard]] virtual ShapeCastHit shape_cast(const ShapeDesc& shape, const Pose& start_pose, const Vec3& sweep_world, real max_distance,
                                                  bool skip_non_wheel_bodies = false) const = 0;
    // Sim thread, between steps only. Default: no backend keeps the flag.
    virtual void set_wheel_contact(BodyId, bool) {}

    // Pure narrowphase overlap query (no sweep, no side effects on the
    // backend) - PLAN.md P1.5b strategy A: "Jolt is used only as a collision
    // query (shape casts / collide-shape) against static and dynamic
    // geometry". Every live body overlapping `shape` at `pose` is appended
    // into caller-owned `out` (out.clear()'d first, so this can be called
    // every substep from preallocated scratch with no allocation - PLAN.md
    // D8). Safe to call concurrently with other read-only queries as long as
    // the backend is not concurrently stepping (World's pipeline only calls
    // this between vehicle_islands_substep and backend_step, PLAN.md section
    // 5 - never overlapping with IRigidBackend::step()).
    //
    // PHYS-033 I1b: a Box/Sphere/Capsule/Cylinder `shape` is now built on the
    // stack (jolt_backend.cpp's with_query_shape), so the QUERY SHAPE itself
    // no longer allocates either - only Mesh/Heightfield still fall back to a
    // heap-allocated shape (never used as a query shape on any hot path).
    // The COLLECTOR still allocates on any call that finds >= 1 hit (its
    // internal JPH::Array grows on first push_back) - a known follow-up, not
    // fixed by I1b (see the I1b report's handover list).
    virtual void collide_shape_into(const ShapeDesc& shape, const Pose& pose, std::vector<ShapeContactHit>& out) const = 0;

    // Contact events observed during the most recent step(), drained (and
    // cleared) by the caller once per tick, appended into caller-owned
    // `out` (out is reassigned to hold exactly this tick's events - see
    // JoltRigidBackend::drain_contact_events_into). PHYS-009 Step 3: World's
    // backend-step stage calls this every tick, inside a job; a fresh
    // by-value vector allocated inside the contact listener's own drain
    // every tick with a live contact - see BackendContactListener::drain_into's
    // comment (jolt_backend.cpp) for the debug_alloc backtrace that found
    // it. World reuses its own last_contact_events_ member as `out`.
    virtual void drain_contact_events_into(std::vector<ContactEvent>& out) = 0;

    // Convenience wrapper for non-hot-path callers (tests, tools).
    std::vector<ContactEvent> drain_contact_events() {
        std::vector<ContactEvent> out;
        drain_contact_events_into(out);
        return out;
    }

    // Folds every live body's pose+velocity (in a fixed, id-ascending
    // order) into a running FNV-1a hash, starting from seed. Part of
    // World::state_hash() (PLAN.md D7).
    [[nodiscard]] virtual std::uint64_t compute_state_hash(std::uint64_t seed) const = 0;

    [[nodiscard]] virtual std::size_t body_count() const = 0;

    // Every live BodyId, ascending by index (fixed, deterministic order),
    // appended into caller-owned `out` (out.clear()'d first). PHYS-009 Step
    // 3: World's snapshot stage (World::stage_snapshot) calls this every
    // tick, and every pipeline stage runs inside a job (PLAN.md D8) - a
    // fresh by-value vector here allocated once per tick forever, found via
    // a debug_alloc backtrace landing in JoltRigidBackend::live_body_ids()
    // called from World::stage_snapshot(). World reuses the same
    // already-capacity'd vector every tick (World::live_body_ids_scratch_).
    virtual void live_body_ids_into(std::vector<BodyId>& out) const = 0;

    // Convenience wrapper for non-hot-path callers (tests, tools) that just
    // want the list once and do not care about the allocation.
    [[nodiscard]] std::vector<BodyId> live_body_ids() const {
        std::vector<BodyId> out;
        live_body_ids_into(out);
        return out;
    }
};

// Jolt solver settings exposed for the P1 Part A energy-loss study
// (PLAN.md; see JoltPhysics v5.6.0's Physics/PhysicsSettings.h). Every
// default below equals Jolt's own default, so a caller who does not touch
// this struct gets today's behaviour unchanged. collision_steps is not a
// PhysicsSettings field; it is PhysicsSystem::Update's own inCollisionSteps
// parameter (substeps per World tick, each running the full velocity/
// position solve at dt/collision_steps).
struct RigidBackendSettings {
    real baumgarte = 0.2;                       // PhysicsSettings::mBaumgarte
    unsigned num_velocity_steps = 10;           // PhysicsSettings::mNumVelocitySteps
    unsigned num_position_steps = 2;            // PhysicsSettings::mNumPositionSteps
    int collision_steps = 1;                    // PhysicsSystem::Update's inCollisionSteps
    real penetration_slop = 0.02;               // PhysicsSettings::mPenetrationSlop
    real speculative_contact_distance = 0.02;   // PhysicsSettings::mSpeculativeContactDistance
    real max_penetration_distance = 0.2;        // PhysicsSettings::mMaxPenetrationDistance
    real linear_cast_threshold = 0.75;          // PhysicsSettings::mLinearCastThreshold
    real linear_cast_max_penetration = 0.25;    // PhysicsSettings::mLinearCastMaxPenetration
};

// The only P1 implementation. job_system is the single pool Jolt's own
// internal parallel work (broadphase/narrowphase/island solving) is
// submitted to via a JPH::JobSystem bridge (core/src/jobs/jolt_job_system.h,
// internal) - never a second thread pool (PLAN.md D8).
std::unique_ptr<IRigidBackend> create_jolt_backend(jobs::JobSystem& job_system, const Vec3& gravity, std::size_t max_bodies,
                                                     const RigidBackendSettings& settings = {});

} // namespace ps
