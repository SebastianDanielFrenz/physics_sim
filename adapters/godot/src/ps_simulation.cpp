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
using godot::Vector3;

namespace ps_godot {

namespace {
std::string to_std_string(const String& s) {
    const godot::CharString utf8 = s.utf8();
    return std::string(utf8.get_data());
}

// vehicle_order_/articulation_order_ are both small (this demo has one car
// and a handful of articulations at most); a linear scan avoids keeping a
// second name->index map in sync with either.
std::size_t index_in_order(const std::vector<std::string>& order, const std::string& name) {
    for (std::size_t i = 0; i < order.size(); ++i) {
        if (order[i] == name) return i;
    }
    return static_cast<std::size_t>(-1);
}

// PHYS-033 P4 I3 plan section 12.1's get_vehicle_powertrain: string forms
// for the HUD/getter Dictionary (a GDScript caller has no access to the
// core's C++ enum types).
const char* shift_phase_name(ps::drivetrain::ShiftPhase p) {
    switch (p) {
        case ps::drivetrain::ShiftPhase::Idle: return "idle";
        case ps::drivetrain::ShiftPhase::ClutchOpening: return "clutch_opening";
        case ps::drivetrain::ShiftPhase::Releasing: return "releasing";
        case ps::drivetrain::ShiftPhase::Selecting: return "selecting";
        case ps::drivetrain::ShiftPhase::Synchronising: return "synchronising";
        case ps::drivetrain::ShiftPhase::ClutchClosing: return "clutch_closing";
    }
    return "idle";
}
const char* engine_state_name(ps::drivetrain::EngineState s) {
    switch (s) {
        case ps::drivetrain::EngineState::Off: return "off";
        case ps::drivetrain::EngineState::Stalled: return "stalled";
        case ps::drivetrain::EngineState::Cranking: return "cranking";
        case ps::drivetrain::EngineState::Running: return "running";
    }
    return "off";
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

        // PLAN.md M1: vehicles are spawned at runtime (spawn_vehicle, after
        // a track dir resolves a spawn point), not from this scenario file
        // - a fresh scenario load always starts with none.
        vehicles_by_name_.clear();
        vehicle_chassis_body_name_.clear();
        vehicle_desc_by_name_.clear();
        vehicle_order_.clear();
        vehicle_terrain_interest_id_.clear();
        vehicle_stream_radius_m_.clear();
        next_terrain_interest_id_ = 1;
        surface_table_.reset();
        terrain_source_.reset();
        fall_detectors_.clear();
        {
            std::lock_guard<std::mutex> lock(pending_reset_mutex_);
            pending_vehicle_resets_.clear();
        }

        // Pre-seed every control channel name this adapter's input map can
        // drive (input_map_core.h's ControlChannels) so later set_control()
        // calls from the Godot main thread only ever update an EXISTING
        // atomic<float>, never insert into control_channels_ while the sim
        // thread might be concurrently reading it in SimThread's pre_step
        // hook (see ps_simulation.h's set_control doc comment).
        control_channels_.clear();
        for (const char* name : {"steer", "throttle", "brake", "handbrake", "clutch"}) {
            control_channels_.emplace(name, 0.0f);
        }

        sim_thread_ = std::make_unique<SimThread>(*world_);
        // pre_step/post_step are installed by start() (rebuild_post_step_
        // and_start), once every setup call (import_test_ground,
        // create_body_dynamic, spawn_vehicle) has finished - see this
        // class's header comment.

        origin_rebase_.set_origin(ps::Vec3::zero());
        last_error_ = String();
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::print("ps_godot: load_scenario failed: ", e.what());
        last_error_ = String("load_scenario failed: ") + String(e.what());
        world_.reset();
        sim_thread_.reset();
        return false;
    }
}

void PsSimulation::start() {
    if (!sim_thread_ || is_running()) return;
    rebuild_post_step_and_start();
}

void PsSimulation::rebuild_post_step_and_start() {
    // PHYS-033 P4 I3 plan section 12.1: seed every I3 drivetrain channel
    // this adapter did NOT already pre-seed in load_scenario() (steer/
    // throttle/brake/handbrake/clutch, still 0) with the CORE's OWN default
    // (World::create_vehicle's ensure_control_channel_index calls, resolved
    // once spawn_vehicle has run - so this must happen here, after every
    // setup call and before the sim thread starts, not in load_scenario()
    // where no vehicle exists yet). So ignition = 1, gear_request = "none"
    // and the assists come from the core's own defaults unless the user (or
    // a scenario's own control_channels block) has already touched them -
    // finding 0.4's fix (a naive 0.0f seed switched the engine off on the
    // very first adapter-driven tick).
    if (world_) {
        for (const char* name :
             {"shift_up_count", "shift_down_count", "gear_request", "ignition", "starter", "assist.auto_clutch",
              "assist.auto_blip", "assist.auto_shift"}) {
            if (control_channels_.find(name) == control_channels_.end()) {
                control_channels_.emplace(name, static_cast<float>(world_->get_control(name)));
            }
        }
    }

    fall_detectors_.clear();
    for (const std::string& name : vehicle_order_) fall_detectors_.emplace(name, FallDetector{}); // no insertion from the sim thread later
    sim_thread_->set_pre_step([this]() {
        // Control channels: the only writer of VALUES from here on is the
        // Godot main thread (set_control), the only writer of KEYS was
        // load_scenario() before start() ran - see the header comment.
        for (auto& kv : control_channels_) {
            world_->set_control(kv.first, static_cast<ps::real>(kv.second.load(std::memory_order_relaxed)));
        }

        // PLAN.md M1 "reset vehicle" - applied here (sim thread, right
        // before World::step()) so a live reset never races
        // IRigidBackend::step() running concurrently with a main-thread
        // backend write.
        std::vector<PendingVehicleReset> resets;
        {
            std::lock_guard<std::mutex> lock(pending_reset_mutex_);
            resets.swap(pending_vehicle_resets_);
        }
        for (const PendingVehicleReset& r : resets) {
            const auto chassis_name_it = vehicle_chassis_body_name_.find(r.vehicle_name);
            if (chassis_name_it == vehicle_chassis_body_name_.end()) continue;
            const auto body_it = bodies_by_name_.find(chassis_name_it->second);
            if (body_it == bodies_by_name_.end()) continue;
            world_->backend().set_pose(body_it->second, r.pose);
            world_->backend().set_motion(body_it->second, ps::Motion{});

            // PHYS-033 P4 I3 plan section 12.1: also reset the drivetrain
            // network itself (right after the chassis pose/velocity above,
            // same tick, same pre_step call - no intervening World::step()
            // sees the stale pre-reset state) - closes ps_simulation.h's own
            // former KNOWN LIMITATION comment on reset_vehicle. Keeps
            // whatever gear the vehicle was in (kKeepGear); the chassis is
            // now at rest, so hubs_roll_with_chassis=false (zero hub omega,
            // not whatever the pre-reset chassis velocity was) and
            // clutch_locked=false (Launch mode, not locked at a stale gear
            // ratio's speed).
            const auto vid_it = vehicles_by_name_.find(r.vehicle_name);
            if (vid_it != vehicles_by_name_.end()) {
                ps::vehicle::VehicleResetOptions opts;
                opts.gear = ps::vehicle::VehicleResetOptions::kKeepGear;
                opts.engine_state = ps::drivetrain::EngineState::Running;
                opts.hubs_roll_with_chassis = false;
                opts.clutch_locked = false;
                world_->reset_vehicle(vid_it->second, opts);
            }
        }
    });

    sim_thread_->set_post_step([this]() {
        AdapterSnapshot snap;
        snap.tick = world_->tick();
        snap.contacts = world_->last_contact_events();

        snap.articulations.reserve(articulation_order_.size());
        for (const std::string& name : articulation_order_) {
            const ps::ArticulationId id = articulations_by_name_.at(name);
            ArticulationLinkPoseSet set;
            set.root_pose = world_->link_frame(id, -1).world_pose(world_->backend());
            const std::size_t link_count = articulation_link_counts_.count(name) ? articulation_link_counts_.at(name) : 0;
            set.link_poses.reserve(link_count);
            for (std::size_t link = 0; link < link_count; ++link) {
                set.link_poses.push_back(world_->link_frame(id, static_cast<int>(link)).world_pose(world_->backend()));
            }
            snap.articulations.push_back(std::move(set));
        }

        // PLAN.md M1 steps 1/5: per-vehicle wheel telemetry (race-free
        // capture, see ps_simulation.h's WheelDynamicSnapshot comment) plus
        // following each vehicle with its own terrain interest point so
        // tiles stream around it while driving (PLAN.md M1: "the vehicle
        // as the terrain interest point").
        snap.vehicles.reserve(vehicle_order_.size());
        for (const std::string& name : vehicle_order_) {
            const ps::VehicleId vid = vehicles_by_name_.at(name);
            VehicleDynamicSnapshot vs;
            const std::size_t wheel_count = world_->vehicle_wheel_count(vid);
            vs.wheels.reserve(wheel_count);
            for (std::size_t w = 0; w < wheel_count; ++w) {
                const ps::vehicle::WheelTelemetry& wt = world_->wheel_telemetry(vid, w);
                WheelDynamicSnapshot ws;
                ws.compression = wt.compression;
                ws.omega = wt.omega;
                ws.slip_ratio = wt.slip_ratio;
                ws.slip_angle = wt.slip_angle;
                ws.fx = wt.fx;
                ws.fy = wt.fy;
                ws.fz = wt.fz;
                ws.surface = wt.surface;
                ws.steer_angle = world_->wheel_state(vid, w).steer_angle; // real value, see WheelDynamicSnapshot's own comment
                vs.wheels.push_back(ws);
            }
            vs.powertrain = world_->powertrain_state(vid); // plan section 12.1, same race-free path as the wheels above
            snap.vehicles.push_back(std::move(vs));

            const auto chassis_name_it = vehicle_chassis_body_name_.find(name);
            if (chassis_name_it != vehicle_chassis_body_name_.end()) {
                const auto body_it = bodies_by_name_.find(chassis_name_it->second);
                if (body_it != bodies_by_name_.end()) {
                    const ps::Pose chassis_pose = world_->get_pose(body_it->second); // safe: sim thread, right after step()
                    const auto radius_it = vehicle_stream_radius_m_.find(name);
                    const ps::real radius = radius_it != vehicle_stream_radius_m_.end() ? static_cast<ps::real>(radius_it->second) : 500.0;
                    world_->set_terrain_interest_point(vehicle_terrain_interest_id_.at(name), chassis_pose.position, radius);

                    // Fall diagnostic: ONE report block per fall event, then keep running.
                    FallDetector& fall = fall_detectors_.at(name);
                    if (fall.update(terrain_source_.get(), chassis_pose.position)) {
                        const std::string report = fall.format_report(name, *world_, vid, body_it->second, terrain_source_.get(), surface_table_.get());
                        godot::UtilityFunctions::print(String(report.c_str()));
                    }
                }
            }
        }

        adapter_snapshots_.write_slot() = std::move(snap);
        adapter_snapshots_.publish();
    });

    sim_thread_->start();
}

void PsSimulation::stop() {
    if (sim_thread_) sim_thread_->stop();
}

godot::Vector3 PsSimulation::rebase_focus(const godot::Vector3& focus_godot_position) {
    const ps::Vec3 absolute = godot_to_iso(focus_godot_position, origin_rebase_.origin());
    origin_rebase_.update(absolute);
    return iso_to_godot(absolute, origin_rebase_.origin());
}

godot::Vector3 PsSimulation::get_world_origin_godot_position() const {
    return iso_to_godot(ps::Vec3{0.0, 0.0, 0.0}, origin_rebase_.origin());
}

godot::Transform3D PsSimulation::get_world_origin_godot_transform() const {
    return iso_to_godot_transform(ps::Pose::identity(), origin_rebase_.origin());
}

godot::Dictionary PsSimulation::probe_ground_height(float sim_x, float sim_y) const {
    Dictionary out;
    out["hit"] = false;
    out["height_m"] = 0.0f;
    if (!world_) return out;
    const ps::real half_range = 4000.0; // generous: above the tallest test-ground feature, below the deepest
    const ps::RayCastHit hit = world_->backend().ray_cast(
        ps::Vec3{static_cast<ps::real>(sim_x), static_cast<ps::real>(sim_y), half_range},
        ps::Vec3{0.0, 0.0, -1.0}, half_range * ps::real(2.0));
    if (!hit.hit) return out;
    out["hit"] = true;
    out["height_m"] = static_cast<float>(hit.point.z);
    return out;
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

float PsSimulation::get_body_speed_mps(const String& body_name) const {
    if (!world_) return 0.0f;
    const std::string name = to_std_string(body_name);
    const auto it = bodies_by_name_.find(name);
    if (it == bodies_by_name_.end()) return 0.0f;
    const ps::BodyId id = it->second;
    const ps::WorldSnapshot& snap = const_cast<ps::World*>(world_.get())->snapshots().read();
    for (const ps::BodySnapshot& b : snap.bodies) {
        if (b.id == id) return static_cast<float>(b.motion.linear.length());
    }
    return 0.0f;
}

godot::Transform3D PsSimulation::get_articulation_link_transform(const String& articulation_name, int link_index) const {
    const std::string name = to_std_string(articulation_name);
    const std::size_t order_index = index_in_order(articulation_order_, name);
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

void PsSimulation::set_control(const String& channel, float value) {
    const std::string name = to_std_string(channel);
    const auto it = control_channels_.find(name);
    if (it != control_channels_.end()) {
        it->second.store(value, std::memory_order_relaxed);
    } else if (!is_running()) {
        // Not pre-seeded (an unexpected channel name) but the sim thread
        // isn't running yet - inserting here is still single-threaded.
        control_channels_.emplace(name, value);
    } else {
        godot::UtilityFunctions::print("ps_godot: set_control(\"", channel, "\") ignored - unknown channel, not pre-seeded before start()");
    }
}

float PsSimulation::get_control(const String& channel) const {
    const auto it = control_channels_.find(to_std_string(channel));
    return it == control_channels_.end() ? 0.0f : it->second.load(std::memory_order_relaxed);
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

bool PsSimulation::import_test_ground(const String& track_dir_absolute_path, std::int64_t max_resident_tiles, std::int64_t max_tile_fills_per_tick) {
    if (!world_) {
        last_error_ = "import_test_ground: no world (call load_scenario first)";
        return false;
    }
    if (is_running()) {
        last_error_ = "import_test_ground: must be called before start()";
        return false;
    }
    const std::string track_dir = to_std_string(track_dir_absolute_path);
    ps::terrain::TerrainConfig tcfg;
    tcfg.max_resident_tiles = static_cast<std::size_t>(max_resident_tiles);
    tcfg.max_tile_fills_per_tick = static_cast<std::size_t>(max_tile_fills_per_tick);

    std::shared_ptr<const ps::io::SurfaceTable> surfaces;
    const TestGroundLoadResult result = import_test_ground_real(track_dir, *world_, surfaces, tcfg);
    if (!result.ok) {
        last_error_ = String(result.error.c_str());
        return false;
    }
    surface_table_ = std::move(surfaces);
    terrain_source_ = result.terrain_source;
    return true;
}

bool PsSimulation::create_body_dynamic(const String& name, const Vector3& sim_position, float yaw_rad, float mass, const Vector3& half_extents) {
    if (!world_) {
        last_error_ = "create_body_dynamic: no world";
        return false;
    }
    if (is_running()) {
        last_error_ = "create_body_dynamic: must be called before start()";
        return false;
    }
    const std::string body_name = to_std_string(name);
    if (bodies_by_name_.count(body_name)) {
        last_error_ = String("create_body_dynamic: a body named \"") + name + "\" already exists";
        return false;
    }

    ps::BodyDesc desc;
    desc.motion = ps::BodyMotionType::Dynamic;
    desc.mass = static_cast<ps::real>(mass);
    desc.shape = ps::BoxShape{ps::Vec3{static_cast<ps::real>(half_extents.x), static_cast<ps::real>(half_extents.y), static_cast<ps::real>(half_extents.z)}};
    desc.pose = ps::Pose{
        ps::Vec3{static_cast<ps::real>(sim_position.x), static_cast<ps::real>(sim_position.y), static_cast<ps::real>(sim_position.z)},
        ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, static_cast<ps::real>(yaw_rad))};
    desc.friction = 0.5;
    desc.allow_sleep = false;

    const ps::BodyId id = world_->create_body(desc);
    bodies_by_name_[body_name] = id;
    return true;
}

bool PsSimulation::spawn_vehicle(const String& vehicle_name, const String& vehicle_json_absolute_path, const String& chassis_body_name, float stream_radius_m) {
    if (!world_) {
        last_error_ = "spawn_vehicle: no world";
        return false;
    }
    if (is_running()) {
        last_error_ = "spawn_vehicle: must be called before start()";
        return false;
    }
    const std::string name = to_std_string(vehicle_name);
    const std::string chassis_name = to_std_string(chassis_body_name);
    const auto body_it = bodies_by_name_.find(chassis_name);
    if (body_it == bodies_by_name_.end()) {
        last_error_ = String("spawn_vehicle: unknown chassis body \"") + chassis_body_name + "\"";
        return false;
    }

    try {
        ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(to_std_string(vehicle_json_absolute_path));
        const ps::VehicleId vid = world_->create_vehicle(desc, body_it->second);
        vehicles_by_name_[name] = vid;
        vehicle_chassis_body_name_[name] = chassis_name;
        vehicle_order_.push_back(name);
        vehicle_terrain_interest_id_[name] = next_terrain_interest_id_++;
        vehicle_stream_radius_m_[name] = stream_radius_m;
        vehicle_desc_by_name_[name] = std::move(desc);
        return true;
    } catch (const std::exception& e) {
        last_error_ = String("spawn_vehicle failed: ") + String(e.what());
        godot::UtilityFunctions::print("ps_godot: ", last_error_);
        return false;
    }
}

bool PsSimulation::reset_vehicle(const String& vehicle_name, const Vector3& sim_position, float yaw_rad) {
    if (!world_) return false;
    const std::string name = to_std_string(vehicle_name);
    const auto chassis_name_it = vehicle_chassis_body_name_.find(name);
    if (chassis_name_it == vehicle_chassis_body_name_.end()) return false;
    const auto body_it = bodies_by_name_.find(chassis_name_it->second);
    if (body_it == bodies_by_name_.end()) return false;

    const ps::Pose pose{
        ps::Vec3{static_cast<ps::real>(sim_position.x), static_cast<ps::real>(sim_position.y), static_cast<ps::real>(sim_position.z)},
        ps::Quat::from_axis_angle(ps::Vec3{0.0, 0.0, 1.0}, static_cast<ps::real>(yaw_rad))};

    if (!is_running()) {
        // Not running yet - single-threaded, safe to apply directly.
        world_->backend().set_pose(body_it->second, pose);
        world_->backend().set_motion(body_it->second, ps::Motion{});
        return true;
    }

    std::lock_guard<std::mutex> lock(pending_reset_mutex_);
    pending_vehicle_resets_.push_back(PendingVehicleReset{name, pose});
    return true;
}

godot::PackedStringArray PsSimulation::get_vehicle_names() const {
    godot::PackedStringArray out;
    for (const std::string& name : vehicle_order_) out.push_back(String(name.c_str()));
    return out;
}

String PsSimulation::get_vehicle_chassis_body_name(const String& vehicle_name) const {
    const auto it = vehicle_chassis_body_name_.find(to_std_string(vehicle_name));
    return it == vehicle_chassis_body_name_.end() ? String() : String(it->second.c_str());
}

std::int64_t PsSimulation::get_vehicle_wheel_count(const String& vehicle_name) const {
    const auto it = vehicles_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicles_by_name_.end() || !world_) return 0;
    return static_cast<std::int64_t>(world_->vehicle_wheel_count(it->second));
}

String PsSimulation::get_wheel_name(const String& vehicle_name, int wheel_index) const {
    const auto it = vehicles_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicles_by_name_.end() || !world_) return String();
    if (static_cast<std::size_t>(wheel_index) >= world_->vehicle_wheel_count(it->second)) return String();
    return String(world_->wheel_name(it->second, static_cast<std::size_t>(wheel_index)).c_str());
}

const ps::vehicle::WheelDesc* PsSimulation::wheel_desc(const std::string& vehicle_name, int wheel_index) const {
    const auto it = vehicle_desc_by_name_.find(vehicle_name);
    if (it == vehicle_desc_by_name_.end()) return nullptr;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= it->second.wheels.size()) return nullptr;
    return &it->second.wheels[static_cast<std::size_t>(wheel_index)];
}

Vector3 PsSimulation::get_wheel_attachment_local(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    if (w == nullptr) return Vector3();
    return Vector3(static_cast<float>(w->attachment_local.x), static_cast<float>(w->attachment_local.y), static_cast<float>(w->attachment_local.z));
}

Vector3 PsSimulation::get_wheel_suspension_axis(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    if (w == nullptr) return Vector3(0, 0, 1);
    return Vector3(static_cast<float>(w->suspension.axis.x), static_cast<float>(w->suspension.axis.y), static_cast<float>(w->suspension.axis.z));
}

float PsSimulation::get_wheel_radius(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->wheel_radius);
}

float PsSimulation::get_wheel_width(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->wheel_width);
}

float PsSimulation::get_wheel_travel_max(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->suspension.travel_max);
}

bool PsSimulation::get_wheel_steered(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    return w != nullptr && w->steered;
}

bool PsSimulation::get_wheel_driven(const String& vehicle_name, int wheel_index) const {
    // PHYS-033 P4 I2 commit 5a: WheelDesc::driven is gone - which wheels
    // are driven is now the /2 powertrain graph's own shape, so this reads
    // it from World (ps::World::wheel_is_driven) instead of the cached
    // WheelDesc wheel_desc() above still serves for the other, still-
    // static wheel fields (radius, width, travel_max, steered, is_front).
    const auto it = vehicles_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicles_by_name_.end() || !world_) return false;
    if (static_cast<std::size_t>(wheel_index) >= world_->vehicle_wheel_count(it->second)) return false;
    return world_->wheel_is_driven(it->second, static_cast<std::size_t>(wheel_index));
}

bool PsSimulation::get_wheel_is_front(const String& vehicle_name, int wheel_index) const {
    const ps::vehicle::WheelDesc* w = wheel_desc(to_std_string(vehicle_name), wheel_index);
    return w != nullptr && w->is_front;
}

float PsSimulation::get_vehicle_max_wheel_angle(const String& vehicle_name) const {
    const auto it = vehicle_desc_by_name_.find(to_std_string(vehicle_name));
    return it == vehicle_desc_by_name_.end() ? 0.0f : static_cast<float>(it->second.steering.max_wheel_angle);
}

float PsSimulation::get_vehicle_rear_steer_fraction(const String& vehicle_name) const {
    const auto it = vehicle_desc_by_name_.find(to_std_string(vehicle_name));
    return it == vehicle_desc_by_name_.end() ? 0.0f : static_cast<float>(it->second.steering.rear_steer_fraction);
}

Dictionary PsSimulation::get_vehicle_powertrain(const String& vehicle_name) const {
    Dictionary d;
    const std::size_t order_index = index_in_order(vehicle_order_, to_std_string(vehicle_name));
    if (order_index == static_cast<std::size_t>(-1)) return d;
    const AdapterSnapshot& snap = const_cast<TripleBuffer<AdapterSnapshot>&>(adapter_snapshots_).read();
    if (order_index >= snap.vehicles.size()) return d;
    const ps::drivetrain::PowertrainSnapshot& p = snap.vehicles[order_index].powertrain;

    d["gear"] = static_cast<int>(p.gear);
    d["gear_target"] = static_cast<int>(p.gear_target);
    d["shift_phase"] = String(shift_phase_name(p.shift_phase));
    d["rpm"] = static_cast<float>(p.engine_rpm);
    d["engine_state"] = String(engine_state_name(p.engine_state));
    d["speedo_kmh"] = static_cast<float>(p.speedo_mps * 3.6);
    d["clutch_engagement"] = static_cast<float>(p.clutch_engagement);
    // 0 or 1 engine in I3 (powertrain_state.h's own comment on
    // PowertrainSnapshot::engines) - the limiter/starter/dfco flags live on
    // the engine sound-state entry, not on PowertrainSnapshot itself.
    d["limiter"] = !p.engines.empty() && p.engines[0].limiter_active;
    d["starter_active"] = !p.engines.empty() && p.engines[0].starter_active;
    d["dfco"] = !p.engines.empty() && p.engines[0].dfco_active;
    d["grind_active"] = p.driveline.grind_active;
    d["assist_auto_clutch"] = (p.assist_active_bits & 1u) != 0;
    d["assist_auto_blip"] = (p.assist_active_bits & 2u) != 0;
    d["assist_auto_shift"] = (p.assist_active_bits & 4u) != 0;
    return d;
}

const WheelDynamicSnapshot* PsSimulation::dynamic_wheel(const std::string& vehicle_name, int wheel_index) const {
    const std::size_t order_index = index_in_order(vehicle_order_, vehicle_name);
    if (order_index == static_cast<std::size_t>(-1)) return nullptr;
    const AdapterSnapshot& snap = const_cast<TripleBuffer<AdapterSnapshot>&>(adapter_snapshots_).read();
    if (order_index >= snap.vehicles.size()) return nullptr;
    const std::vector<WheelDynamicSnapshot>& wheels = snap.vehicles[order_index].wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return nullptr;
    return &wheels[static_cast<std::size_t>(wheel_index)];
}

float PsSimulation::get_wheel_compression(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->compression);
}

float PsSimulation::get_wheel_omega(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->omega);
}

float PsSimulation::get_wheel_steer_angle(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->steer_angle);
}

float PsSimulation::get_wheel_slip_ratio(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->slip_ratio);
}

float PsSimulation::get_wheel_slip_angle(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->slip_angle);
}

float PsSimulation::get_wheel_load_n(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->fz);
}

float PsSimulation::get_wheel_fx(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->fx);
}

float PsSimulation::get_wheel_fy(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->fy);
}

String PsSimulation::get_wheel_surface_name(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    if (w == nullptr || surface_table_ == nullptr || w->surface == ps::kInvalidSurfaceId) return String();
    return String(surface_table_->name_for(w->surface).c_str());
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
    ClassDB::bind_method(D_METHOD("get_world_origin_godot_position"), &PsSimulation::get_world_origin_godot_position);
    ClassDB::bind_method(D_METHOD("get_world_origin_godot_transform"), &PsSimulation::get_world_origin_godot_transform);
    ClassDB::bind_method(D_METHOD("probe_ground_height", "sim_x", "sim_y"), &PsSimulation::probe_ground_height);
    ClassDB::bind_method(D_METHOD("get_body_transform", "body_name"), &PsSimulation::get_body_transform);
    ClassDB::bind_method(D_METHOD("get_body_speed_mps", "body_name"), &PsSimulation::get_body_speed_mps);
    ClassDB::bind_method(D_METHOD("get_articulation_link_transform", "articulation_name", "link_index"), &PsSimulation::get_articulation_link_transform);
    ClassDB::bind_method(D_METHOD("set_control", "channel", "value"), &PsSimulation::set_control);
    ClassDB::bind_method(D_METHOD("get_control", "channel"), &PsSimulation::get_control);
    ClassDB::bind_method(D_METHOD("get_contact_events"), &PsSimulation::get_contact_events);
    ClassDB::bind_method(D_METHOD("import_static_collision", "scene_root"), &PsSimulation::import_static_collision);
    ClassDB::bind_method(D_METHOD("import_test_ground", "track_dir_absolute_path", "max_resident_tiles", "max_tile_fills_per_tick"), &PsSimulation::import_test_ground, DEFVAL(512), DEFVAL(16));
    ClassDB::bind_method(D_METHOD("get_last_error"), &PsSimulation::get_last_error);
    ClassDB::bind_method(D_METHOD("terrain_resident_tile_count"), &PsSimulation::terrain_resident_tile_count);
    ClassDB::bind_method(D_METHOD("terrain_starved_tile_count"), &PsSimulation::terrain_starved_tile_count);

    ClassDB::bind_method(D_METHOD("create_body_dynamic", "name", "sim_position", "yaw_rad", "mass", "half_extents"), &PsSimulation::create_body_dynamic);
    ClassDB::bind_method(D_METHOD("spawn_vehicle", "vehicle_name", "vehicle_json_absolute_path", "chassis_body_name", "stream_radius_m"), &PsSimulation::spawn_vehicle);
    ClassDB::bind_method(D_METHOD("reset_vehicle", "vehicle_name", "sim_position", "yaw_rad"), &PsSimulation::reset_vehicle);
    ClassDB::bind_method(D_METHOD("get_vehicle_names"), &PsSimulation::get_vehicle_names);
    ClassDB::bind_method(D_METHOD("get_vehicle_chassis_body_name", "vehicle_name"), &PsSimulation::get_vehicle_chassis_body_name);
    ClassDB::bind_method(D_METHOD("get_vehicle_wheel_count", "vehicle_name"), &PsSimulation::get_vehicle_wheel_count);
    ClassDB::bind_method(D_METHOD("get_wheel_name", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_name);
    ClassDB::bind_method(D_METHOD("get_wheel_attachment_local", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_attachment_local);
    ClassDB::bind_method(D_METHOD("get_wheel_suspension_axis", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_suspension_axis);
    ClassDB::bind_method(D_METHOD("get_wheel_radius", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_radius);
    ClassDB::bind_method(D_METHOD("get_wheel_width", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_width);
    ClassDB::bind_method(D_METHOD("get_wheel_travel_max", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_travel_max);
    ClassDB::bind_method(D_METHOD("get_wheel_steered", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_steered);
    ClassDB::bind_method(D_METHOD("get_wheel_driven", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_driven);
    ClassDB::bind_method(D_METHOD("get_wheel_is_front", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_is_front);
    ClassDB::bind_method(D_METHOD("get_vehicle_max_wheel_angle", "vehicle_name"), &PsSimulation::get_vehicle_max_wheel_angle);
    ClassDB::bind_method(D_METHOD("get_vehicle_rear_steer_fraction", "vehicle_name"), &PsSimulation::get_vehicle_rear_steer_fraction);
    ClassDB::bind_method(D_METHOD("get_vehicle_powertrain", "vehicle_name"), &PsSimulation::get_vehicle_powertrain);
    ClassDB::bind_method(D_METHOD("get_wheel_compression", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_compression);
    ClassDB::bind_method(D_METHOD("get_wheel_omega", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_omega);
    ClassDB::bind_method(D_METHOD("get_wheel_steer_angle", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_steer_angle);
    ClassDB::bind_method(D_METHOD("get_wheel_slip_ratio", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_slip_ratio);
    ClassDB::bind_method(D_METHOD("get_wheel_slip_angle", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_slip_angle);
    ClassDB::bind_method(D_METHOD("get_wheel_load_n", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_load_n);
    ClassDB::bind_method(D_METHOD("get_wheel_fx", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_fx);
    ClassDB::bind_method(D_METHOD("get_wheel_fy", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_fy);
    ClassDB::bind_method(D_METHOD("get_wheel_surface_name", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_surface_name);

    ClassDB::bind_method(D_METHOD("add_adapter_time_us", "us"), &PsSimulation::add_adapter_time_us);
    ClassDB::bind_method(D_METHOD("consume_adapter_frame_time_us"), &PsSimulation::consume_adapter_frame_time_us);
}

} // namespace ps_godot
