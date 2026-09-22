// adapters/godot/src/debug_draw.h — PsDebugDraw: immediate-mode debug
// drawing of contact points/normals (PLAN.md P2: "debug draw of force
// vectors and contacts"). Draws each tick's contact events (World::
// last_contact_events, read race-free via PsSimulation's AdapterSnapshot -
// see ps_simulation.h's file comment) as a short line along the contact
// normal, scaled by the normal impulse, plus a small point marker.

#pragma once

#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

namespace ps_godot {

class PsSimulation;

class PsDebugDraw : public godot::MeshInstance3D {
    GDCLASS(PsDebugDraw, godot::MeshInstance3D)

public:
    void _ready() override;
    void _process(double delta) override;

    void set_simulation_path(const godot::NodePath& path) { simulation_path_ = path; }
    [[nodiscard]] const godot::NodePath& get_simulation_path() const { return simulation_path_; }

    void set_vector_scale(float scale) { vector_scale_ = scale; }
    [[nodiscard]] float get_vector_scale() const { return vector_scale_; }

protected:
    static void _bind_methods();

private:
    [[nodiscard]] PsSimulation* resolve_simulation() const;

    godot::NodePath simulation_path_;
    godot::Ref<godot::ImmediateMesh> immediate_mesh_;
    godot::Ref<godot::StandardMaterial3D> material_;
    float vector_scale_ = 0.01f; // metres per (N*s) of normal impulse
};

} // namespace ps_godot
