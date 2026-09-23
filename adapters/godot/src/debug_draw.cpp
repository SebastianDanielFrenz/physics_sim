#include "debug_draw.h"
#include "frame_convert.h"
#include "frame_timing.h"
#include "ps_simulation.h"

#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

using godot::ClassDB;
using godot::D_METHOD;
using godot::Transform3D;
using godot::Vector3;

namespace ps_godot {

namespace {

// `raw_sim_vec` carries raw ISO/sim-space components (chassis-local, not
// yet basis-changed - see PsSimulation::get_wheel_attachment_local's doc
// comment). Converts it to a WORLD-space Godot direction/offset relative
// to `chassis` (a Godot Transform3D already produced by PsSimulation::
// get_body_transform).
//
// `chassis.basis`'s column i is "the sim's own local axis i (ISO 8855: x
// forward, y left, z up), expressed in Godot world space" - that is
// exactly frame_convert_core.h's iso_to_godot_basis's own contract
// ("expands q_sim's rotation of the sim's own local x/y/z axes, then maps
// each resulting world direction through iso_to_godot_dir"), which is
// LINEAR - so for a chassis-local vector v = a*e_x + b*e_y + c*e_z (in ISO
// axes), the corresponding world Godot direction is simply
// chassis.basis.xform(Vector3(a, b, c)): applying iso_to_godot_dir to
// (a, b, c) FIRST and then chassis.basis would double-apply the axis
// change (correct only by coincidence at identity orientation) - found by
// actually running this (chase_cam.gd's near-identical bug, see that
// file's own fix comment).
Vector3 chassis_local_offset_to_world(const Transform3D& chassis, const Vector3& raw_sim_vec) {
    return chassis.origin + chassis.basis.xform(raw_sim_vec);
}

Vector3 chassis_local_dir_to_world(const Transform3D& chassis, const Vector3& raw_sim_vec) {
    return chassis.basis.xform(raw_sim_vec);
}

} // namespace

PsSimulation* PsDebugDraw::resolve_simulation() const {
    if (simulation_path_.is_empty()) return nullptr;
    godot::Node* node = get_node_or_null(simulation_path_);
    return godot::Object::cast_to<PsSimulation>(node);
}

void PsDebugDraw::_ready() {
    immediate_mesh_.instantiate();
    material_.instantiate();
    material_->set_shading_mode(godot::BaseMaterial3D::SHADING_MODE_UNSHADED);
    material_->set_flag(godot::BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    material_->set_transparency(godot::BaseMaterial3D::TRANSPARENCY_DISABLED);
    set_mesh(immediate_mesh_);
    set_process(true);
}

void PsDebugDraw::_process(double /*delta*/) {
    const ScopedAdapterTimer timer; // frame_timing.h: this is adapter main-thread work
    PsSimulation* sim = resolve_simulation();
    immediate_mesh_->clear_surfaces();
    if (sim == nullptr || !debug_enabled_) return;

    immediate_mesh_->surface_begin(godot::Mesh::PRIMITIVE_LINES, material_);
    draw_contacts(*sim);
    draw_vehicles(*sim);
    immediate_mesh_->surface_end();
}

void PsDebugDraw::draw_contacts(PsSimulation& sim) {
    const godot::TypedArray<godot::Dictionary> contacts = sim.get_contact_events();
    for (int i = 0; i < contacts.size(); ++i) {
        const godot::Dictionary c = contacts[i];
        const Vector3 point = c["point"];
        const Vector3 normal = c["normal"];
        const float impulse = c["normal_impulse"];
        const Vector3 tip = point + normal * (vector_scale_ * godot::Math::max(impulse, 0.05f));

        immediate_mesh_->surface_set_color(godot::Color(1.0f, 0.15f, 0.15f, 1.0f));
        immediate_mesh_->surface_add_vertex(point);
        immediate_mesh_->surface_add_vertex(tip);
    }
}

void PsDebugDraw::draw_vehicles(PsSimulation& sim) {
    const godot::PackedStringArray vehicle_names = sim.get_vehicle_names();
    for (int vi = 0; vi < vehicle_names.size(); ++vi) {
        const godot::String vehicle_name = vehicle_names[vi];
        const godot::String chassis_body_name = sim.get_vehicle_chassis_body_name(vehicle_name);
        if (chassis_body_name.is_empty()) continue;
        const Transform3D chassis = sim.get_body_transform(chassis_body_name);

        const std::int64_t wheel_count = sim.get_vehicle_wheel_count(vehicle_name);
        for (std::int64_t w = 0; w < wheel_count; ++w) {
            const int wi = static_cast<int>(w);
            const Vector3 attach_local = sim.get_wheel_attachment_local(vehicle_name, wi); // raw sim-space, x=0 (full extension)
            const Vector3 axis_local = sim.get_wheel_suspension_axis(vehicle_name, wi).normalized();
            const float compression = sim.get_wheel_compression(vehicle_name, wi);

            const Vector3 static_point_world = chassis_local_offset_to_world(chassis, attach_local);
            const Vector3 wheel_center_local = attach_local + axis_local * compression;
            const Vector3 wheel_center_world = chassis_local_offset_to_world(chassis, wheel_center_local);

            // Suspension travel: green, static mount -> current wheel centre.
            immediate_mesh_->surface_set_color(godot::Color(0.2f, 0.9f, 0.2f, 1.0f));
            immediate_mesh_->surface_add_vertex(static_point_world);
            immediate_mesh_->surface_add_vertex(wheel_center_world);

            // Suspension (normal) force: blue, along the (world) suspension
            // axis, scaled by fz.
            const float fz = sim.get_wheel_load_n(vehicle_name, wi);
            const Vector3 axis_world_dir = chassis_local_dir_to_world(chassis, axis_local);
            immediate_mesh_->surface_set_color(godot::Color(0.2f, 0.4f, 1.0f, 1.0f));
            immediate_mesh_->surface_add_vertex(wheel_center_world);
            immediate_mesh_->surface_add_vertex(wheel_center_world + axis_world_dir * (fz * wheel_force_scale_));

            // Combined tyre force: yellow. APPROXIMATION - uses the
            // chassis's own forward/lateral axes, not the true steered
            // wheel axes (core has no per-wheel steer-angle accessor yet;
            // see this adapter's report for the exact API requested). Exact
            // for non-steered wheels (steer angle 0); off by the wheel's
            // current steer angle for steered ones.
            const float fx = sim.get_wheel_fx(vehicle_name, wi);
            const float fy = sim.get_wheel_fy(vehicle_name, wi);
            const Vector3 forward_world = chassis_local_dir_to_world(chassis, Vector3(1, 0, 0));
            const Vector3 lateral_world = chassis_local_dir_to_world(chassis, Vector3(0, 1, 0));
            const Vector3 tyre_force_world = forward_world * fx + lateral_world * fy;
            immediate_mesh_->surface_set_color(godot::Color(1.0f, 0.85f, 0.1f, 1.0f));
            immediate_mesh_->surface_add_vertex(wheel_center_world);
            immediate_mesh_->surface_add_vertex(wheel_center_world + tyre_force_world * wheel_force_scale_);
        }
    }
}

void PsDebugDraw::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_simulation_path", "path"), &PsDebugDraw::set_simulation_path);
    ClassDB::bind_method(D_METHOD("get_simulation_path"), &PsDebugDraw::get_simulation_path);
    ClassDB::bind_method(D_METHOD("set_vector_scale", "scale"), &PsDebugDraw::set_vector_scale);
    ClassDB::bind_method(D_METHOD("get_vector_scale"), &PsDebugDraw::get_vector_scale);
    ClassDB::bind_method(D_METHOD("set_wheel_force_scale", "scale"), &PsDebugDraw::set_wheel_force_scale);
    ClassDB::bind_method(D_METHOD("get_wheel_force_scale"), &PsDebugDraw::get_wheel_force_scale);
    ClassDB::bind_method(D_METHOD("set_debug_enabled", "enabled"), &PsDebugDraw::set_debug_enabled);
    ClassDB::bind_method(D_METHOD("get_debug_enabled"), &PsDebugDraw::get_debug_enabled);

    ClassDB::add_property("PsDebugDraw", godot::PropertyInfo(godot::Variant::NODE_PATH, "simulation_path"), "set_simulation_path", "get_simulation_path");
    ClassDB::add_property("PsDebugDraw", godot::PropertyInfo(godot::Variant::FLOAT, "vector_scale"), "set_vector_scale", "get_vector_scale");
    ClassDB::add_property("PsDebugDraw", godot::PropertyInfo(godot::Variant::FLOAT, "wheel_force_scale"), "set_wheel_force_scale", "get_wheel_force_scale");
    ClassDB::add_property("PsDebugDraw", godot::PropertyInfo(godot::Variant::BOOL, "debug_enabled"), "set_debug_enabled", "get_debug_enabled");
}

} // namespace ps_godot
