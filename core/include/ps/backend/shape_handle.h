// ps/backend/shape_handle.h - prebuilt collision shapes (racing_game request
// R2, 2026-10-05): build a shape's backend representation (for a MeshShape,
// Jolt's whole BVH) on ANY thread, then create bodies from it on the sim
// thread at O(1) cost in the shape's size.
//
//     // any thread, also while World::step runs on the sim thread:
//     ps::ShapeHandle deck = world.create_shape(ps::ShapeDesc{std::move(mesh)});
//     // sim thread, between steps (same rule as every other create_body):
//     ps::BodyDesc bd;                       // bd.shape is IGNORED here
//     bd.motion = ps::BodyMotionType::Static;
//     bd.pose = ...;
//     const ps::BodyId id = world.create_body(bd, deck);
//     deck.reset();                          // optional - the body keeps the shape alive
//
// Contract (decision record: vault, out/prebuilt_shapes_progress.md):
//
// * Ownership: a ShapeHandle is a REFERENCE-COUNTED, immutable, opaque
//   shape. Copying a handle shares the shape; the shape is freed when the
//   last handle copy AND the last body created from it are gone. A body
//   keeps its shape alive until World::destroy_body (or the World's own
//   destruction). Releasing a handle (destructor, reset(), assignment) never
//   frees a shape a live body still uses.
// * Threads: create_shape is safe on any thread, concurrently with other
//   create_shape calls, with World::step and with body creation/destruction
//   on the sim thread. Handle copies may be created, copied and released on
//   any thread concurrently (std::shared_ptr semantics: distinct ShapeHandle
//   OBJECTS may be used concurrently, one object must not be written by two
//   threads at once). create_body/destroy_body stay sim-thread-only between
//   steps, exactly like the BodyDesc path.
// * Equivalence: a body created from create_shape(d) + create_body(bd, h) is
//   bit-identical in behaviour and World::state_hash to create_body(bd with
//   bd.shape = d): same shape construction arguments, same BodyId sequence,
//   same terrain-suppression hook. The handle itself never enters the hash.
// * Order: building handles in any order or on any threads does not change
//   anything; only the order of create_body calls decides BodyIds (install
//   in a deterministic order, e.g. racing_game's (way_id, start_station)).
// * Lifetime vs. World: a handle may outlive the World (or backend) that
//   built it and may be used with any World in the process (shapes are
//   immutable and backend-instance independent). Do not keep handles in
//   static storage that is destroyed after main() returns.
// * Kinds: Box, Sphere, Capsule, Cylinder and MeshShape. HeightfieldShape is
//   rejected (std::invalid_argument): heightfield content is mutable
//   (update_heightfield_body, the prefetch ring's spares) and a shared,
//   immutable heightfield would break that contract; the terrain prefetch
//   ring already builds heightfields off the tick.
// * Allocation (PLAN.md D8): create_shape allocates. Call it from your own
//   threads, never from inside a ps::jobs job body (pool or background lane)
//   - debug_alloc counts allocations only on threads currently inside a job
//   (a thread_local depth set by JobSystem::run_job_counted), so a
//   create_shape on an ordinary std::thread is never a violation, even while
//   the World's own jobs are armed.
// * Errors: create_shape throws std::invalid_argument for a HeightfieldShape
//   and std::runtime_error when the backend rejects the shape (e.g. a mesh
//   with no non-degenerate triangles) - on the calling worker, never later
//   on the sim thread. create_body(bd, h) throws std::invalid_argument for an
//   empty handle, a handle from a different backend type, or a MeshShape
//   handle on a non-Static body.

#pragma once

#include <cstdint>
#include <memory>

namespace ps {

enum class ShapeKind : std::uint8_t { Box, Sphere, Capsule, Cylinder, Mesh, Heightfield };

// Backend-defined payload behind a ShapeHandle. Opaque: only the backend
// that created it knows its concrete type (jolt_backend.cpp's
// JoltPreparedShape); no backend type appears here (PLAN.md D2).
class PreparedShape {
public:
    PreparedShape(const PreparedShape&) = delete;
    PreparedShape& operator=(const PreparedShape&) = delete;
    virtual ~PreparedShape();

    [[nodiscard]] ShapeKind kind() const { return kind_; }

protected:
    explicit PreparedShape(ShapeKind kind);

private:
    ShapeKind kind_;
};

class ShapeHandle {
public:
    ShapeHandle() noexcept = default;
    // Backend use only (IRigidBackend::create_shape).
    explicit ShapeHandle(std::shared_ptr<const PreparedShape> prepared) noexcept : prepared_(std::move(prepared)) {}

    [[nodiscard]] bool valid() const noexcept { return prepared_ != nullptr; }
    explicit operator bool() const noexcept { return valid(); }
    // Precondition: valid().
    [[nodiscard]] ShapeKind kind() const { return prepared_->kind(); }

    // Releases this copy (safe on any thread).
    void reset() noexcept { prepared_.reset(); }

    // Diagnostic only: handle copies plus bodies currently sharing this
    // shape (std::shared_ptr::use_count - approximate under concurrent use).
    [[nodiscard]] long use_count() const noexcept { return prepared_.use_count(); }

    // Backend use only.
    [[nodiscard]] const std::shared_ptr<const PreparedShape>& prepared() const noexcept { return prepared_; }

    // Same prepared shape (not: equal content).
    friend bool operator==(const ShapeHandle& a, const ShapeHandle& b) noexcept { return a.prepared_ == b.prepared_; }

private:
    std::shared_ptr<const PreparedShape> prepared_;
};

// Diagnostics / tests only, process-wide (every backend and World):
// PreparedShape objects currently alive (a handle copy or a body still
// references each), and the number of backend shapes a PreparedShape's
// destruction actually freed (it held the last backend reference - always
// the case once every body using it is destroyed).
[[nodiscard]] std::uint64_t debug_live_prepared_shape_count();
[[nodiscard]] std::uint64_t debug_prepared_shape_backend_frees();
// Backend use only (jolt_backend.cpp): counted by the destructor.
void debug_note_prepared_shape_backend_free();

} // namespace ps
