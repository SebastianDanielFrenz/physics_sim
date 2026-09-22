#include "ps_simulation.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using godot::ClassDB;
using godot::D_METHOD;
using godot::Dictionary;
using godot::String;
using godot::TypedArray;

namespace ps_godot {

namespace {
std::string to_std_string(const String& s) {
    const godot::CharString utf8 = s.utf8();
    return std::string(utf8.get_data());
}
} // namespace

bool PsSimulation::load_scenario(const String& absolute_json_path) {
    stop();
    try {
        const std::string path = to_std_string(absolute_json_path);
        ps::io::Scenario scenario = ps::io::load_scenario(path);

        articulation_link_counts_.clear();
        for (const ps::io::ScenarioArticulation& sa : scenario.articulations) articulation_link_counts_[sa.name] = sa.desc.links.size();

        ps::io::BuiltScenario built = ps::io::build_scenario(scenario);
        world_ = std::move(built.world);
        bodies_by_name_ = std::move(built.bodies_by_name);
        articulations_by_name_ = std::move(built.articulations_by_name);

        articulation_order_.clear();
        for (const auto& [name, id] : articulations_by_name_) {
            (void)id;
            articulation_order_.push_back(name);
        }
        // Captured by the post_step lambda below via `this`
        // (articulation_order_ + articulations_by_name_ + articulation_link_counts_),
        // stashed as a member-sized parallel structure indexed the same
        // way as articulation_order_.
        std::vector<std::size_t> link_count_by_order;
        link_count_by_order.reserve(articulation_order_.size());
        for (const std::string& name : articulation_order_) {
            link_count_by_order.push_back(articulation_link_counts_.count(name) ? articulation_link_counts_.at(name) : 0);
        }

        sim_thread_ = std::make_unique<SimThread>(*world_);
        sim_thread_->set_post_step([this, link_count_by_order]() {
            AdapterSnapshot snap;
            snap.tick = world_->tick();
            snap.contacts = world_->last_contact_events();
            snap.articulations.reserve(articulation_order_.size());
            for (std::size_t i = 0; i < articulation_order_.size(); ++i) {
                const ps::ArticulationId id = articulations_by_name_.at(articulation_order_[i]);
                ArticulationLinkPoseSet set;
                set.root_pose = world_->link_frame(id, -1).world_pose(world_->backend());
                set.link_poses.reserve(link_count_by_order[i]);
                for (std::size_t link = 0; link < link_count_by_order[i]; ++link) {
                    set.link_poses.push_back(world_->link_frame(id, static_cast<int>(link)).world_pose(world_->backend()));
                }
                snap.articulations.push_back(std::move(set));
            }
            adapter_snapshots_.write_slot() = std::move(snap);
            adapter_snapshots_.publish();
        });

        origin_rebase_.set_origin(ps::Vec3::zero());
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::print("ps_godot: load_scenario failed: ", e.what());
        world_.reset();
        sim_thread_.reset();
        return false;
    }
}

void PsSimulation::start() {
    if (sim_thread_) sim_thread_->start();
}

void PsSimulation::stop() {
    if (sim_thread_) sim_thread_->stop();
}

godot::Vector3 PsSimulation::rebase_focus(const godot::Vector3& focus_godot_position) {
    const ps::Vec3 absolute = godot_to_iso(focus_godot_position, origin_rebase_.origin());
    origin_rebase_.update(absolute);
    return iso_to_godot(absolute, origin_rebase_.origin());
}

godot::Transform3D PsSimulation::get_body_transform(const String& body_name) const {
    if (!world_) return godot::Transform3D();
    const std::string name = to_std_string(body_name);
    const auto it = bodies_by_name_.find(name);
    if (it == bodies_by_name_.end()) return godot::Transform3D();
    const ps::BodyId id = it->second;

    // Race-free: World's own lock-free snapshot triple buffer, published by
    // the sim thread's snapshot stage every tick (ps/world/snapshot.h).
    const ps::WorldSnapshot& snap = const_cast<ps::World*>(world_.get())->snapshots().read();
    for (const ps::BodySnapshot& b : snap.bodies) {
        if (b.id == id) return iso_to_godot_transform(b.pose, origin_rebase_.origin());
    }
    return godot::Transform3D();
}

godot::Transform3D PsSimulation::get_articulation_link_transform(const String& articulation_name, int link_index) const {
    const std::string name = to_std_string(articulation_name);
    std::size_t order_index = static_cast<std::size_t>(-1);
    for (std::size_t i = 0; i < articulation_order_.size(); ++i) {
        if (articulation_order_[i] == name) {
            order_index = i;
            break;
        }
    }
    if (order_index == static_cast<std::size_t>(-1)) return godot::Transform3D();

    const AdapterSnapshot& snap = const_cast<TripleBuffer<AdapterSnapshot>&>(adapter_snapshots_).read();
    if (order_index >= snap.articulations.size()) return godot::Transform3D();
    const ArticulationLinkPoseSet& set = snap.articulations[order_index];
    const ps::Pose& pose = (link_index < 0) ? set.root_pose : (static_cast<std::size_t>(link_index) < set.link_poses.size() ? set.link_poses[static_cast<std::size_t>(link_index)] : set.root_pose);
    return iso_to_godot_transform(pose, origin_rebase_.origin());
}

godot::PackedStringArray PsSimulation::get_body_names() const {
    godot::PackedStringArray out;
    for (const auto& [name, id] : bodies_by_name_) {
        (void)id;
        out.push_back(String(name.c_str()));
    }
    return out;
}

godot::PackedStringArray PsSimulation::get_articulation_names() const {
    godot::PackedStringArray out;
    for (const std::string& name : articulation_order_) out.push_back(String(name.c_str()));
    return out;
}

std::int64_t PsSimulation::get_articulation_link_count(const String& articulation_name) const {
    const auto it = articulation_link_counts_.find(to_std_string(articulation_name));
    return it == articulation_link_counts_.end() ? 0 : static_cast<std::int64_t>(it->second);
}

void PsSimulation::set_control(const String& channel, float value) { control_channels_[to_std_string(channel)] = value; }

float PsSimulation::get_control(const String& channel) const {
    const auto it = control_channels_.find(to_std_string(channel));
    return it == control_channels_.end() ? 0.0f : it->second;
}

TypedArray<Dictionary> PsSimulation::get_contact_events() const {
    TypedArray<Dictionary> out;
    const AdapterSnapshot& snap = const_cast<TripleBuffer<AdapterSnapshot>&>(adapter_snapshots_).read();
    for (const ps::ContactEvent& c : snap.contacts) {
        Dictionary d;
        d["point"] = iso_to_godot(c.point, origin_rebase_.origin());
        d["normal"] = to_godot(iso_to_godot_dir(c.normal));
        d["normal_impulse"] = static_cast<float>(c.normal_impulse);
        out.push_back(d);
    }
    return out;
}

std::int64_t PsSimulation::import_static_collision(godot::Node* scene_root) {
    if (!world_ || scene_root == nullptr) return 0;
    int skipped = 0;
    const int created = import_static_collision_from_scene(scene_root, *world_, origin_rebase_.origin(), &skipped);
    return created;
}

bool PsSimulation::import_test_ground(const String& track_dir_absolute_path) {
    if (!world_) return false;
    track_dir_for_flat_plane_ = to_std_string(track_dir_absolute_path);
    const ps::BodyId id = import_test_ground_flat_plane(track_dir_for_flat_plane_, *world_);
    return id.valid();
}

void PsSimulation::_bind_methods() {
    ClassDB::bind_method(D_METHOD("load_scenario", "absolute_json_path"), &PsSimulation::load_scenario);
    ClassDB::bind_method(D_METHOD("start"), &PsSimulation::start);
    ClassDB::bind_method(D_METHOD("stop"), &PsSimulation::stop);
    ClassDB::bind_method(D_METHOD("is_running"), &PsSimulation::is_running);
    ClassDB::bind_method(D_METHOD("get_step_count"), &PsSimulation::get_step_count);
    ClassDB::bind_method(D_METHOD("get_sim_time"), &PsSimulation::get_sim_time);
    ClassDB::bind_method(D_METHOD("get_tick_rate_hz"), &PsSimulation::get_tick_rate_hz);
    ClassDB::bind_method(D_METHOD("get_body_count"), &PsSimulation::get_body_count);
    ClassDB::bind_method(D_METHOD("get_body_names"), &PsSimulation::get_body_names);
    ClassDB::bind_method(D_METHOD("get_articulation_names"), &PsSimulation::get_articulation_names);
    ClassDB::bind_method(D_METHOD("get_articulation_link_count", "articulation_name"), &PsSimulation::get_articulation_link_count);
    ClassDB::bind_method(D_METHOD("rebase_focus", "focus_godot_position"), &PsSimulation::rebase_focus);
    ClassDB::bind_method(D_METHOD("get_body_transform", "body_name"), &PsSimulation::get_body_transform);
    ClassDB::bind_method(D_METHOD("get_articulation_link_transform", "articulation_name", "link_index"), &PsSimulation::get_articulation_link_transform);
    ClassDB::bind_method(D_METHOD("set_control", "channel", "value"), &PsSimulation::set_control);
    ClassDB::bind_method(D_METHOD("get_control", "channel"), &PsSimulation::get_control);
    ClassDB::bind_method(D_METHOD("get_contact_events"), &PsSimulation::get_contact_events);
    ClassDB::bind_method(D_METHOD("import_static_collision", "scene_root"), &PsSimulation::import_static_collision);
    ClassDB::bind_method(D_METHOD("import_test_ground", "track_dir_absolute_path"), &PsSimulation::import_test_ground);
}

} // namespace ps_godot
