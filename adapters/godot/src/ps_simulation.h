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
// does not snapshot yet (articulation link poses, this tick's contact
// events - both captured inside SimThread's post_step callback, which runs
// synchronously on the sim thread right after World::step() returns, so
// there is no concurrency at the point of the read). No method here ever
// calls ps::World::get_pose/backend() directly while running() is true;
// see get_body_transform() and get_articulation_link_transform().

#pragma once

#include "collision_import.h"
#include "frame_convert.h"
#include "origin_rebase.h"
#include "sim_thread.h"
#include "triple_buffer.h"

#include "ps/io/scenario.h"
#include "ps/world/world.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ps_godot {

struct ArticulationLinkPoseSet {
    ps::Pose root_pose; // World::link_frame(id, -1)
    std::vector<ps::Pose> link_poses; // index == link_index
};

struct AdapterSnapshot {
    // Indexed the same way as PsSimulation's articulations_by_name_ /
    // articulation_order_ (registration order at build_scenario time).
    std::vector<ArticulationLinkPoseSet> articulations;
    std::vector<ps::ContactEvent> contacts;
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
    // the demo's free_cam.gd does the former for the camera itself.
    [[nodiscard]] godot::Vector3 rebase_focus(const godot::Vector3& focus_godot_position);
    [[nodiscard]] godot::Vector3 origin_as_godot_reference() const { return godot::Vector3(0, 0, 0); } // origin is always local (0,0,0) by construction

    // --- Body / articulation transforms (race-free: snapshot-backed) ---
    [[nodiscard]] godot::Transform3D get_body_transform(const godot::String& body_name) const;
    [[nodiscard]] godot::Transform3D get_articulation_link_transform(const godot::String& articulation_name, int link_index) const;

    // --- Named control channels (PLAN.md P2: "input reaches the sim only
    // as named control channels ... in ISO/SI terms, never as Godot
    // types"). NOTE (report-flagged request): ps::World has no equivalent
    // registry yet for a real vehicle/drivetrain to read from - none
    // exists before P3. This is a plain float map on the adapter side so
    // the *boundary* contract already holds today; wiring it to an actual
    // consumer is P3+'s job.
    void set_control(const godot::String& channel, float value);
    [[nodiscard]] float get_control(const godot::String& channel) const;

    // --- Debug draw data (race-free via AdapterSnapshot, see file comment) --
    [[nodiscard]] godot::TypedArray<godot::Dictionary> get_contact_events() const;

    // --- Static collision import (PLAN.md P2) ---------------------------
    // Scans `scene_root` for StaticBody3D/CollisionShape3D nodes and
    // creates matching static bodies in the sim (collision_import.h).
    std::int64_t import_static_collision(godot::Node* scene_root);
    // Loads the M1 test ground's collision data (see collision_import.h's
    // KNOWN LIMITATION comment: a flat plane approximation today, not the
    // exact mesh - core has no mesh/heightfield ShapeDesc yet).
    bool import_test_ground(const godot::String& track_dir_absolute_path);

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
    std::unique_ptr<ps::World> world_;
    std::unordered_map<std::string, ps::BodyId> bodies_by_name_;
    std::unordered_map<std::string, ps::ArticulationId> articulations_by_name_;
    std::vector<std::string> articulation_order_; // registration index -> name, matches AdapterSnapshot::articulations
    std::unordered_map<std::string, std::size_t> articulation_link_counts_;

    std::unique_ptr<SimThread> sim_thread_;
    OriginRebase origin_rebase_;
    TripleBuffer<AdapterSnapshot> adapter_snapshots_;

    std::unordered_map<std::string, float> control_channels_;

    std::string track_dir_for_flat_plane_; // set by import_test_ground, used lazily if needed
};

} // namespace ps_godot
