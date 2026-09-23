// adapters/godot/src/ps_simulation.h — PsSimulation: the adapter's main
// Godot node (PLAN.md P2). Owns a ps::World, loads a JSON scenario into it,
// runs it on its own fixed-240 Hz thread (SimThread, decoupled from
// Godot's frame rate - D3), and exposes race-free read access to the
// running sim for the rest of the adapter (PsBodyNode3D, debug draw, HUD).
//
// Thread-safety contract this class enforces: the ONLY state read from the
// Godot/main thread while the sim thread is running is either (a)
// ps::World's own lock-free snapshot triple buffer (rigid-body poses), or
// (b) this adapter's OWN triple buffer (triple_buffer.h) for the data core
// does not snapshot yet (articulation link poses, per-wheel vehicle
// telemetry, this tick's contact events - all captured inside SimThread's
// post_step callback, which runs synchronously on the sim thread right
// after World::step() returns, so there is no concurrency at the point of
// the read). No method here ever calls ps::World::get_pose/get_motion/
// wheel_telemetry/backend() directly while running() is true; see
// get_body_transform() and get_articulation_link_transform().
//
// PLAN.md M1: World-mutating setup (import_test_ground, create_body_
// dynamic, spawn_vehicle) must be called AFTER load_scenario() but BEFORE
// start() - exactly like every other "setup time" World call in this
// codebase (create_body, set_terrain_source, ...). start() is the point
// where this class captures the finished vehicle_order_/articulation_
// order_ into the post_step callback and hands the World to the sim
// thread; calling a setup method afterwards returns false without
// touching the World (see is_running() guards below).

#pragma once

#include "collision_import.h"
#include "frame_convert.h"
#include "frame_timing.h"
#include "origin_rebase.h"
#include "sim_thread.h"
#include "triple_buffer.h"

#include "ps/io/scenario.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/world.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ps_godot {

struct ArticulationLinkPoseSet {
    ps::Pose root_pose; // World::link_frame(id, -1)
    std::vector<ps::Pose> link_poses; // index == link_index
};

// PLAN.md M1 step 3/5: one wheel's DYNAMIC state (World::wheel_telemetry is
// not snapshot-buffered by core - reading it off the sim thread would race
// vehicle::System::step_all exactly like a raw get_pose() would race the
// backend, see this file's header comment). Static wheel geometry
// (attachment point, radius, steered/driven flags, ...) does not change
// per tick, so it is cached once at spawn_vehicle() time instead
// (vehicle_desc_by_name_) rather than duplicated into every tick's snapshot.
struct WheelDynamicSnapshot {
    ps::real compression = 0.0;
    ps::real omega = 0.0;
    ps::real slip_ratio = 0.0;
    ps::real slip_angle = 0.0;
    ps::real fx = 0.0, fy = 0.0, fz = 0.0;
    ps::SurfaceId surface = ps::kInvalidSurfaceId;
};

struct VehicleDynamicSnapshot {
    std::vector<WheelDynamicSnapshot> wheels; // index == wheel index, matches vehicle_desc_by_name_'s wheels
};

struct AdapterSnapshot {
    // Indexed the same way as PsSimulation's articulations_by_name_ /
    // articulation_order_ (registration order at build_scenario time).
    std::vector<ArticulationLinkPoseSet> articulations;
    std::vector<ps::ContactEvent> contacts;
    // Indexed the same way as vehicle_order_ (registration order at
    // spawn_vehicle time, finalised when start() is called).
    std::vector<VehicleDynamicSnapshot> vehicles;
    std::uint64_t tick = 0;
};

class PsSimulation : public godot::Node {
    GDCLASS(PsSimulation, godot::Node)

public:
    PsSimulation() = default;
    ~PsSimulation() override { stop(); }

    // --- Scenario / lifecycle ------------------------------------------
    bool load_scenario(const godot::String& absolute_json_path);
    void start();
    void stop();
    [[nodiscard]] bool is_running() const { return sim_thread_ && sim_thread_->running(); }

    [[nodiscard]] std::int64_t get_step_count() const { return sim_thread_ ? static_cast<std::int64_t>(sim_thread_->ticks_run()) : 0; }
    [[nodiscard]] float get_sim_time() const { return world_ ? static_cast<float>(world_->sim_time()) : 0.0f; }
    [[nodiscard]] float get_tick_rate_hz() const { return world_ ? static_cast<float>(1.0 / world_->dt()) : 0.0f; }
    [[nodiscard]] std::int64_t get_body_count() const { return world_ ? static_cast<std::int64_t>(world_->backend().body_count()) : 0; }
    [[nodiscard]] godot::PackedStringArray get_body_names() const;
    [[nodiscard]] godot::PackedStringArray get_articulation_names() const;
    [[nodiscard]] std::int64_t get_articulation_link_count(const godot::String& articulation_name) const;

    // --- Origin rebasing (PLAN.md P2 large-world paragraph) -------------
    // Pass the camera's (or any focus node's) CURRENT Godot-space position;
    // returns the position that node should be set to instead (identity if
    // no rebase happened this call). Also rebase every other manually
    // placed Godot node's position the same way, or keep such nodes
    // childed under the same origin-relative parent this call updates -
    // the demo's free_cam.gd/chase_cam.gd do the former for the camera
    // itself.
    [[nodiscard]] godot::Vector3 rebase_focus(const godot::Vector3& focus_godot_position);
    [[nodiscard]] godot::Vector3 origin_as_godot_reference() const { return godot::Vector3(0, 0, 0); } // origin is always local (0,0,0) by construction

    // --- Body / articulation transforms (race-free: snapshot-backed) ---
    [[nodiscard]] godot::Transform3D get_body_transform(const godot::String& body_name) const;
    [[nodiscard]] godot::Transform3D get_articulation_link_transform(const godot::String& articulation_name, int link_index) const;
    // Body speed in m/s (World's own BodySnapshot::motion, same race-free
    // path as get_body_transform - PLAN.md M1 HUD's "speed (km/h)").
    [[nodiscard]] float get_body_speed_mps(const godot::String& body_name) const;

    // --- Named control channels (PLAN.md P2: "input reaches the sim only
    // as named control channels ... in ISO/SI terms, never as Godot
    // types"). PLAN.md P3/M1: "steer"/"throttle"/"brake"/"handbrake" now
    // feed every vehicle created via create_vehicle (out/handover_p3.md
    // decision 5: global, not per-vehicle - matches M1's one-car scope).
    //
    // Cross-thread note: World::set_control/get_control (ps/world/world.h)
    // are plain unordered_map accesses with NO synchronisation of their
    // own - safe only because vehicle::System::step_all (sim thread) is
    // the ONLY thing that ever calls them, via SimThread's pre_step hook
    // (rebuild_post_step_and_start, ps_simulation.cpp), which copies THIS
    // class's own control_channels_ into World right before each
    // World::step(). control_channels_ here is a std::unordered_map whose
    // VALUES are std::atomic<float> (properly defined concurrent access)
    // and whose KEYS are pre-seeded once, single-threaded, in
    // load_scenario() (before start() ever runs) - std::unordered_map
    // references to already-inserted elements stay valid across further
    // insertions, so set_control() from the Godot main thread only ever
    // stores into an existing atomic<float>, never touching the map's own
    // structure once the sim thread is running.
    void set_control(const godot::String& channel, float value);
    [[nodiscard]] float get_control(const godot::String& channel) const;

    // --- Debug draw data (race-free via AdapterSnapshot, see file comment) --
    [[nodiscard]] godot::TypedArray<godot::Dictionary> get_contact_events() const;

    // --- Static collision import (PLAN.md P2) ---------------------------
    // Scans `scene_root` for StaticBody3D/CollisionShape3D nodes and
    // creates matching static bodies in the sim (collision_import.h).
    std::int64_t import_static_collision(godot::Node* scene_root);
    // PLAN.md M1 step 1: loads the test ground's REAL collision (mesh, per
    // surface group) plus the procedural terrain source and the shared
    // SurfaceTable (collision_import.h's import_test_ground_real) -
    // replaces the old P2 flat-plane placeholder now that core has
    // MeshShape/HeightfieldShape and the loaders it needs. Must be called
    // before start(). Returns false (see get_last_error()) on any load
    // failure.
    bool import_test_ground(const godot::String& track_dir_absolute_path, std::int64_t max_resident_tiles = 512, std::int64_t max_tile_fills_per_tick = 16);
    [[nodiscard]] const godot::String& get_last_error() const { return last_error_; }
    [[nodiscard]] std::int64_t terrain_resident_tile_count() const { return world_ ? static_cast<std::int64_t>(world_->terrain_resident_tile_count()) : 0; }
    [[nodiscard]] std::int64_t terrain_starved_tile_count() const { return world_ ? static_cast<std::int64_t>(world_->terrain_starved_tile_count()) : 0; }

    // --- Vehicles (PLAN.md M1) -------------------------------------------
    // Generic dynamic-body creation for a vehicle chassis (the scenario
    // schema has no "spawn a lone body at runtime" concept - PsSimulation
    // needs one so main.gd can place the car at a track spawn point that is
    // only known once import_test_ground() has resolved a track dir, not
    // baked into a scenario file). `sim_position`/`yaw_rad` are RAW sim-
    // space (ISO 8855 / test-ground ENU, algebraically identical per
    // data/tracks/README.md's "Frame" section - see that file) numbers
    // carried through a godot::Vector3 purely as a convenient 3-float
    // struct; this is NOT a Godot scene-graph position and does not go
    // through frame_convert.h's basis change - every OTHER method in this
    // adapter that touches a Godot node's transform still goes through
    // that single conversion point. Must be called before start().
    bool create_body_dynamic(const godot::String& name, const godot::Vector3& sim_position, float yaw_rad, float mass, const godot::Vector3& half_extents);
    // Loads vehicle_json_absolute_path (ps::io::load_vehicle_json) and
    // attaches it to the already-created chassis body `chassis_body_name`
    // (see create_body_dynamic above). Registers a terrain interest point
    // for this vehicle (updated every tick from the chassis's own position,
    // see ps_simulation.cpp's post_step lambda) with `stream_radius_m`.
    // Must be called before start().
    bool spawn_vehicle(const godot::String& vehicle_name, const godot::String& vehicle_json_absolute_path, const godot::String& chassis_body_name, float stream_radius_m);
    // Teleports the vehicle's chassis body back to sim_position/yaw_rad
    // with zero velocity (IRigidBackend::set_pose/set_motion - PLAN.md M1
    // "reset vehicle" control). KNOWN LIMITATION: only the chassis rigid
    // body is reset; per-wheel internal state (tyre relaxation deflection,
    // wheel spin rate) has no public reset API (see this adapter's report
    // for the exact API this needs, e.g. a World::reset_vehicle(VehicleId)
    // that zeroes vehicle::Instance's internal wheel state) - in practice
    // this settles within well under a second once the chassis stops
    // moving, since the wheel state relaxes toward the (now zero) chassis
    // motion every substep.
    bool reset_vehicle(const godot::String& vehicle_name, const godot::Vector3& sim_position, float yaw_rad);

    [[nodiscard]] godot::PackedStringArray get_vehicle_names() const;
    [[nodiscard]] godot::String get_vehicle_chassis_body_name(const godot::String& vehicle_name) const;
    [[nodiscard]] std::int64_t get_vehicle_wheel_count(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::String get_wheel_name(const godot::String& vehicle_name, int wheel_index) const;
    // Static per-wheel geometry (cached VehicleDesc, see file comment) -
    // Vector3/float fields are RAW sim-space numbers, same convention as
    // create_body_dynamic above (chassis-local, not Godot-space).
    [[nodiscard]] godot::Vector3 get_wheel_attachment_local(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] godot::Vector3 get_wheel_suspension_axis(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_radius(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_width(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_travel_max(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] bool get_wheel_steered(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] bool get_wheel_driven(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] bool get_wheel_is_front(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_vehicle_max_wheel_angle(const godot::String& vehicle_name) const;
    [[nodiscard]] float get_vehicle_ackermann_fraction(const godot::String& vehicle_name) const;
    [[nodiscard]] float get_vehicle_rear_steer_fraction(const godot::String& vehicle_name) const;

    // Dynamic per-wheel telemetry (race-free via AdapterSnapshot).
    [[nodiscard]] float get_wheel_compression(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_omega(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_slip_ratio(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_slip_angle(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_load_n(const godot::String& vehicle_name, int wheel_index) const; // fz
    [[nodiscard]] float get_wheel_fx(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_fy(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] godot::String get_wheel_surface_name(const godot::String& vehicle_name, int wheel_index) const;

    // --- Adapter main-thread frame-time accounting (frame_timing.h) -----
    void add_adapter_time_us(std::int64_t us) { FrameTiming::add_us(us); }
    [[nodiscard]] std::int64_t consume_adapter_frame_time_us() { return FrameTiming::consume_us(); }

    // Internal (not bound to ClassDB) - used by other C++ adapter classes
    // in the same extension (debug_draw.cpp, ps_body_node.cpp) that need
    // direct World access for things not yet exposed as a bound method.
    // Never call world() methods that touch the backend from outside the
    // post_step callback while is_running() is true.
    [[nodiscard]] ps::World* world() { return world_.get(); }
    [[nodiscard]] const ps::Vec3& origin() const { return origin_rebase_.origin(); }

protected:
    static void _bind_methods();

private:
    [[nodiscard]] const ps::vehicle::WheelDesc* wheel_desc(const std::string& vehicle_name, int wheel_index) const;
    [[nodiscard]] const WheelDynamicSnapshot* dynamic_wheel(const std::string& vehicle_name, int wheel_index) const;
    void rebuild_post_step_and_start(); // start()'s helper: finalises vehicle/articulation ordering, installs pre/post_step, starts SimThread

    // PLAN.md M1 "reset vehicle" control - see reset_vehicle()'s doc
    // comment: applied on the sim thread (SimThread::set_pre_step) rather
    // than directly from here, so a live reset while driving never races
    // IRigidBackend::step(). Mutex-guarded because it is a rare, human-
    // triggered event (a button press), not a hot path - unlike
    // control_channels_ below, which IS touched every tick and therefore
    // uses std::atomic instead.
    struct PendingVehicleReset {
        std::string vehicle_name;
        ps::Pose pose;
    };
    std::mutex pending_reset_mutex_;
    std::vector<PendingVehicleReset> pending_vehicle_resets_;

    std::unique_ptr<ps::World> world_;
    std::unordered_map<std::string, ps::BodyId> bodies_by_name_;
    std::unordered_map<std::string, ps::ArticulationId> articulations_by_name_;
    std::vector<std::string> articulation_order_; // registration index -> name, matches AdapterSnapshot::articulations
    std::unordered_map<std::string, std::size_t> articulation_link_counts_;

    // PLAN.md M1: vehicles (see spawn_vehicle above). vehicle_order_ is
    // finalised (captured into the post_step lambda) when start() runs -
    // spawn_vehicle may only be called before that point, same rule as
    // World::create_body/create_vehicle's own "setup time" contract.
    std::unordered_map<std::string, ps::VehicleId> vehicles_by_name_;
    std::unordered_map<std::string, std::string> vehicle_chassis_body_name_;
    std::unordered_map<std::string, ps::vehicle::VehicleDesc> vehicle_desc_by_name_;
    std::vector<std::string> vehicle_order_;
    std::unordered_map<std::string, std::uint64_t> vehicle_terrain_interest_id_;
    std::unordered_map<std::string, float> vehicle_stream_radius_m_;
    std::uint64_t next_terrain_interest_id_ = 1;

    std::shared_ptr<const ps::io::SurfaceTable> surface_table_; // set by import_test_ground; null == no surface names available

    std::unique_ptr<SimThread> sim_thread_;
    OriginRebase origin_rebase_;
    TripleBuffer<AdapterSnapshot> adapter_snapshots_;

    std::unordered_map<std::string, std::atomic<float>> control_channels_; // see set_control's doc comment above
    godot::String last_error_;
};

} // namespace ps_godot
