// adapters/godot/src/ps_body_node.h — PsBodyNode3D: mirrors one sim body
// (or one articulation link) onto a Node3D's transform every frame
// (PLAN.md P2: "map snapshot poses onto Node3D transforms, including
// articulation links").

#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/node_path.hpp>

namespace ps_godot {

class PsSimulation;

class PsBodyNode3D : public godot::Node3D {
    GDCLASS(PsBodyNode3D, godot::Node3D)

public:
    void _ready() override;
    void _process(double delta) override;

    void set_simulation_path(const godot::NodePath& path) { simulation_path_ = path; }
    [[nodiscard]] const godot::NodePath& get_simulation_path() const { return simulation_path_; }

    void set_body_name(const godot::String& name) { body_name_ = name; }
    [[nodiscard]] const godot::String& get_body_name() const { return body_name_; }

    // When non-empty, this node follows an articulation link instead of a
    // plain rigid body (articulation_link_index -1 == the articulation's
    // own root).
    void set_articulation_name(const godot::String& name) { articulation_name_ = name; }
    [[nodiscard]] const godot::String& get_articulation_name() const { return articulation_name_; }
    void set_articulation_link_index(int index) { articulation_link_index_ = index; }
    [[nodiscard]] int get_articulation_link_index() const { return articulation_link_index_; }

protected:
    static void _bind_methods();

private:
    [[nodiscard]] PsSimulation* resolve_simulation() const;

    godot::NodePath simulation_path_;
    godot::String body_name_;
    godot::String articulation_name_;
    int articulation_link_index_ = -1;
};

} // namespace ps_godot
