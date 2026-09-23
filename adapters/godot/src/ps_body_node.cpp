#include "ps_body_node.h"
#include "frame_timing.h"
#include "ps_simulation.h"

#include <godot_cpp/core/class_db.hpp>

using godot::ClassDB;
using godot::D_METHOD;

namespace ps_godot {

PsSimulation* PsBodyNode3D::resolve_simulation() const {
    if (simulation_path_.is_empty()) return nullptr;
    godot::Node* node = get_node_or_null(simulation_path_);
    return godot::Object::cast_to<PsSimulation>(node);
}

void PsBodyNode3D::_ready() {
    set_process(true);
}

void PsBodyNode3D::_process(double /*delta*/) {
    const ScopedAdapterTimer timer; // frame_timing.h: this is adapter main-thread work
    PsSimulation* sim = resolve_simulation();
    if (sim == nullptr) return;

    if (!articulation_name_.is_empty()) {
        set_transform(sim->get_articulation_link_transform(articulation_name_, articulation_link_index_));
    } else if (!body_name_.is_empty()) {
        set_transform(sim->get_body_transform(body_name_));
    }
}

void PsBodyNode3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_simulation_path", "path"), &PsBodyNode3D::set_simulation_path);
    ClassDB::bind_method(D_METHOD("get_simulation_path"), &PsBodyNode3D::get_simulation_path);
    ClassDB::bind_method(D_METHOD("set_body_name", "name"), &PsBodyNode3D::set_body_name);
    ClassDB::bind_method(D_METHOD("get_body_name"), &PsBodyNode3D::get_body_name);
    ClassDB::bind_method(D_METHOD("set_articulation_name", "name"), &PsBodyNode3D::set_articulation_name);
    ClassDB::bind_method(D_METHOD("get_articulation_name"), &PsBodyNode3D::get_articulation_name);
    ClassDB::bind_method(D_METHOD("set_articulation_link_index", "index"), &PsBodyNode3D::set_articulation_link_index);
    ClassDB::bind_method(D_METHOD("get_articulation_link_index"), &PsBodyNode3D::get_articulation_link_index);

    ClassDB::add_property("PsBodyNode3D", godot::PropertyInfo(godot::Variant::NODE_PATH, "simulation_path"), "set_simulation_path", "get_simulation_path");
    ClassDB::add_property("PsBodyNode3D", godot::PropertyInfo(godot::Variant::STRING, "body_name"), "set_body_name", "get_body_name");
    ClassDB::add_property("PsBodyNode3D", godot::PropertyInfo(godot::Variant::STRING, "articulation_name"), "set_articulation_name", "get_articulation_name");
    ClassDB::add_property("PsBodyNode3D", godot::PropertyInfo(godot::Variant::INT, "articulation_link_index"), "set_articulation_link_index", "get_articulation_link_index");
}

} // namespace ps_godot
