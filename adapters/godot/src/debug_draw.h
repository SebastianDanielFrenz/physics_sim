// adapters/godot/src/debug_draw.h — PsDebugDraw: immediate-mode debug
// drawing (PLAN.md P2: "debug draw of force vectors and contacts"; PLAN.md
// M1 step 6: "contact points, tyre force vectors, suspension travel" plus
// a toggle). Draws:
//  - each tick's contact events (World::last_contact_events, read
//    race-free via PsSimulation's AdapterSnapshot - see ps_simulation.h's
//    file comment) as a short red line along the contact normal, scaled
//    by the normal impulse, plus a small point marker;
//  - per vehicle/per wheel (PsSimulation::get_vehicle_names() et al.): a
//    green line for suspension travel (static attachment point at full
//    extension -> current wheel centre), a yellow line for the combined
//    tyre force (fx/fy, APPROXIMATED using the chassis's own forward/
//    lateral axes rather than the true steered wheel axes - core exposes
//    no per-wheel steer-angle accessor yet, see this adapter's report),
//    and a blue line for the suspension (normal) force along the
//    suspension axis.
// Toggled on/off via debug_enabled (bound property, driven by main.gd from
// PsInputMap::get_debug_toggle()).

#pragma once

#include <godot_cpp/classes/immediate_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

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

    // Separate scale for per-wheel force vectors (Newtons - typically
    // 1e3-1e4 N, three orders of magnitude bigger than a contact impulse
    // in N*s, so vector_scale's 0.01 default would draw multi-metre-long
    // lines through the roof of the car - found by actually looking at a
    // screenshot).
    void set_wheel_force_scale(float scale) { wheel_force_scale_ = scale; }
    [[nodiscard]] float get_wheel_force_scale() const { return wheel_force_scale_; }

    void set_debug_enabled(bool enabled) { debug_enabled_ = enabled; }
    [[nodiscard]] bool get_debug_enabled() const { return debug_enabled_; }

protected:
    static void _bind_methods();

private:
    [[nodiscard]] PsSimulation* resolve_simulation() const;
    void draw_contacts(PsSimulation& sim);
    void draw_vehicles(PsSimulation& sim);

    godot::NodePath simulation_path_;
    godot::Ref<godot::ImmediateMesh> immediate_mesh_;
    godot::Ref<godot::StandardMaterial3D> material_;
    float vector_scale_ = 0.01f; // metres per (N*s) of contact normal impulse
    float wheel_force_scale_ = 0.0006f; // metres per N of a wheel suspension/tyre force
    bool debug_enabled_ = true;
};

} // namespace ps_godot
