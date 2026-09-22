#include "debug_draw.h"
#include "ps_simulation.h"

#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

using godot::ClassDB;
using godot::D_METHOD;

namespace ps_godot {

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
    PsSimulation* sim = resolve_simulation();
    immediate_mesh_->clear_surfaces();
    if (sim == nullptr) return;

    const godot::TypedArray<godot::Dictionary> contacts = sim->get_contact_events();
    if (contacts.size() == 0) return;

    immediate_mesh_->surface_begin(godot::Mesh::PRIMITIVE_LINES, material_);
    for (int i = 0; i < contacts.size(); ++i) {
        const godot::Dictionary c = contacts[i];
        const godot::Vector3 point = c["point"];
        const godot::Vector3 normal = c["normal"];
        const float impulse = c["normal_impulse"];
        const godot::Vector3 tip = point + normal * (vector_scale_ * godot::Math::max(impulse, 0.05f));

        immediate_mesh_->surface_set_color(godot::Color(1.0f, 0.15f, 0.15f, 1.0f));
        immediate_mesh_->surface_add_vertex(point);
        immediate_mesh_->surface_add_vertex(tip);
    }
    immediate_mesh_->surface_end();
}

void PsDebugDraw::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_simulation_path", "path"), &PsDebugDraw::set_simulation_path);
    ClassDB::bind_method(D_METHOD("get_simulation_path"), &PsDebugDraw::get_simulation_path);
    ClassDB::bind_method(D_METHOD("set_vector_scale", "scale"), &PsDebugDraw::set_vector_scale);
    ClassDB::bind_method(D_METHOD("get_vector_scale"), &PsDebugDraw::get_vector_scale);

    ClassDB::add_property("PsDebugDraw", godot::PropertyInfo(godot::Variant::NODE_PATH, "simulation_path"), "set_simulation_path", "get_simulation_path");
    ClassDB::add_property("PsDebugDraw", godot::PropertyInfo(godot::Variant::FLOAT, "vector_scale"), "set_vector_scale", "get_vector_scale");
}

} // namespace ps_godot
