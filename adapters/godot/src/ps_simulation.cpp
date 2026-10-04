#include "engine_bank_generator.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include "ps_simulation.h"

#include <filesystem>
#include <algorithm>
#include <memory>
#include "ps/jobs/job_system.h"
#include "ps/environment/calendar.h"
#include "ps/environment/climate_io.h"
#include "ps/environment/environment.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using godot::ClassDB;
using godot::D_METHOD;
using godot::Dictionary;
using godot::String;
using godot::TypedArray;
using godot::Vector2;
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

        // Gamepad rumble (haptics_core.h): cache this World's actual
        // gravity vector (ps::io::Scenario::world_config is already a
        // public field - no core change) so the chassis proper-acceleration
        // computation in post_step removes exactly what this World applies,
        // not an assumed constant.
        gravity_ = scenario.world_config.gravity;
        chassis_accel_state_.clear();

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
        engine_definition_paths_.clear(); requested_turbo_paths_.clear();
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
              "assist.auto_blip", "assist.auto_shift", "park_lock"}) {
            if (control_channels_.find(name) == control_channels_.end()) {
                control_channels_.emplace(name, static_cast<float>(world_->get_control(name)));
            }
        }
    }

    fall_detectors_.clear();
    for (const std::string& name : vehicle_order_) fall_detectors_.emplace(name, FallDetector{}); // no insertion from the sim thread later

    // Render-time interpolation (render_interp.h): finalise body_order_/
    // body_order_index_ from bodies_by_name_ (same "finalised when start()
    // runs" point as vehicle_order_/articulation_order_ above - every
    // create_body_dynamic call happens before this), then seed
    // prev_body_pose_/prev_articulation_poses_ with each tracked body's/
    // link's CURRENT pose (single-threaded here, sim thread not started
    // yet) so the first published tick's InterpolatedPose already has a
    // real (prev == latest) pair instead of needing a separate first-tick
    // flag - see render_interp.h's InterpolatedPose comment.
    body_order_.clear();
    body_order_index_.clear();
    for (const auto& [name, id] : bodies_by_name_) {
        (void)id;
        body_order_index_[name] = body_order_.size();
        body_order_.push_back(name);
    }
    prev_body_pose_.resize(body_order_.size());
    for (std::size_t i = 0; i < body_order_.size(); ++i) {
        prev_body_pose_[i] = world_->get_pose(bodies_by_name_.at(body_order_[i]));
    }
    body_teleported_this_tick_.assign(body_order_.size(), false);

    prev_articulation_poses_.resize(articulation_order_.size());
    for (std::size_t i = 0; i < articulation_order_.size(); ++i) {
        const std::string& name = articulation_order_[i];
        const ps::ArticulationId id = articulations_by_name_.at(name);
        ArticulationPrevPose& prev = prev_articulation_poses_[i];
        prev.root = world_->link_frame(id, -1).world_pose(world_->backend());
        const std::size_t link_count = articulation_link_counts_.count(name) ? articulation_link_counts_.at(name) : 0;
        prev.links.resize(link_count);
        for (std::size_t link = 0; link < link_count; ++link) {
            prev.links[link] = world_->link_frame(id, static_cast<int>(link)).world_pose(world_->backend());
        }
    }

    // Nominal-clock pose history (render_interp.h): flat layout = every
    // body, then per articulation its root followed by its links. Allocated
    // here, before the sim thread starts.
    articulation_pose_offset_.assign(articulation_order_.size(), 0);
    std::size_t flat_pose_count = body_order_.size();
    for (std::size_t i = 0; i < articulation_order_.size(); ++i) {
        articulation_pose_offset_[i] = flat_pose_count;
        flat_pose_count += 1 + prev_articulation_poses_[i].links.size();
    }
    pose_history_.reset(flat_pose_count);
    history_pose_scratch_.assign(flat_pose_count, ps::Pose{});
    history_teleport_scratch_.assign(flat_pose_count, 0);
    last_publish_wall_ns_ = 0;
    render_sample_.valid = false;
    cached_render_snapshot_ = nullptr; // force a fresh sample on the first frame after a (re)start

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
        std::vector<PendingTurboUpdate> turbo_updates;
        { std::lock_guard<std::mutex> lock(turbo_update_mutex_); turbo_updates.swap(pending_turbo_updates_); }
        for(const auto& update:turbo_updates) {
            const auto vehicle=vehicles_by_name_.find(update.vehicle_name);
            if(vehicle==vehicles_by_name_.end()) continue;
            std::string component;
            for(const auto& c:vehicle_desc_by_name_.at(update.vehicle_name).powertrain.components)
                if(std::holds_alternative<ps::drivetrain::SimulatedEngineDesc>(c.params)) { component=c.id; break; }
            if(!component.empty()) world_->replace_turbo_configuration(vehicle->second,component,update.desc);
            std::lock_guard<std::mutex> lock(turbo_update_mutex_);
            turbo_configuration_status_="Active";
        }
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
            // Gamepad rumble: a teleport is a discontinuous velocity jump,
            // not a real acceleration - drop the previous-velocity sample so
            // post_step treats the next tick as the first one again (accel
            // 0) instead of reading a huge spurious spike.
            chassis_accel_state_.erase(r.vehicle_name);
            // Render-time interpolation (render_interp.h): a real prev/
            // latest blend across THIS tick would render a smear sweeping
            // from the old position to the teleport target over the next
            // frame or two - mark this body so post_step forces prev :=
            // latest for it this tick instead of blending through the jump
            // (render_interp.h's InterpolatedPose comment).
            const auto order_it = body_order_index_.find(chassis_name_it->second);
            if (order_it != body_order_index_.end() && order_it->second < body_teleported_this_tick_.size()) {
                body_teleported_this_tick_[order_it->second] = true;
            }

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
        if (const ps::environment::EnvironmentSnapshot* env = world_->environment_snapshot()) {
            snap.has_environment = true;
            snap.environment = *env;
        }
        // Render-time interpolation (render_interp.h): stamp the moment
        // THIS tick's pose data becomes the new "latest" - ensure_render_
        // frame_cached() measures a render frame's alpha against this.
        snap.publish_time = std::chrono::steady_clock::now();
        snap.contacts = world_->last_contact_events();

        snap.articulations.reserve(articulation_order_.size());
        for (std::size_t i = 0; i < articulation_order_.size(); ++i) {
            const std::string& name = articulation_order_[i];
            const ps::ArticulationId id = articulations_by_name_.at(name);
            ArticulationPrevPose& prev = prev_articulation_poses_[i];

            ArticulationLinkPoseSet set;
            const ps::Pose latest_root = world_->link_frame(id, -1).world_pose(world_->backend());
            set.root = InterpolatedPose{prev.root, latest_root};
            prev.root = latest_root;

            const std::size_t link_count = articulation_link_counts_.count(name) ? articulation_link_counts_.at(name) : 0;
            set.links.reserve(link_count);
            for (std::size_t link = 0; link < link_count; ++link) {
                const ps::Pose latest_link = world_->link_frame(id, static_cast<int>(link)).world_pose(world_->backend());
                set.links.push_back(InterpolatedPose{prev.links[link], latest_link});
                prev.links[link] = latest_link;
            }
            snap.articulations.push_back(std::move(set));
        }

        // Render-time interpolation (render_interp.h): every named body
        // (body_order_, see rebuild_post_step_and_start's own comment) -
        // blend source for get_body_transform. A teleported body
        // (body_teleported_this_tick_, set in this tick's pre_step reset
        // handler above) forces prev := latest instead of carrying over
        // last tick's pose, so it renders at the teleport target
        // immediately rather than smearing across the jump.
        snap.bodies.reserve(body_order_.size());
        for (std::size_t i = 0; i < body_order_.size(); ++i) {
            const ps::Pose latest = world_->get_pose(bodies_by_name_.at(body_order_[i])); // safe: sim thread, right after step()
            const ps::Pose prev = body_teleported_this_tick_[i] ? latest : prev_body_pose_[i];
            snap.bodies.push_back(InterpolatedPose{prev, latest});
            history_pose_scratch_[i] = latest;
            history_teleport_scratch_[i] = body_teleported_this_tick_[i] ? 1 : 0;
            prev_body_pose_[i] = latest;
            body_teleported_this_tick_[i] = false; // consumed for this tick
        }
        for (std::size_t i = 0; i < snap.articulations.size(); ++i) {
            const ArticulationLinkPoseSet& set = snap.articulations[i];
            const std::size_t base = articulation_pose_offset_[i];
            history_pose_scratch_[base] = set.root.latest;
            for (std::size_t l = 0; l < set.links.size(); ++l) history_pose_scratch_[base + 1 + l] = set.links[l].latest;
        }
        // Nominal-clock history (render_interp.h): stamp with the tick's
        // scheduled time, not the wall time this callback happens to run at.
        // Publish lateness/gap histograms are the measured jitter the old
        // publish-time alpha suffered from.
        pose_history_.push(sim_thread_->tick_nominal_time(), snap.tick, sim_thread_->tick_clock_break(), history_pose_scratch_, history_teleport_scratch_);
        {
            const auto wall = std::chrono::steady_clock::now();
            const std::int64_t wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall.time_since_epoch()).count();
            publish_lateness_hist_.add(std::chrono::duration_cast<std::chrono::nanoseconds>(wall - sim_thread_->tick_nominal_time()).count());
            if (last_publish_wall_ns_ != 0) publish_gap_hist_.add(wall_ns - last_publish_wall_ns_);
            last_publish_wall_ns_ = wall_ns;
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

            // ps_audio A11: feed this tick's engine sound state into the
            // vehicle's own voice (if it has one - most don't, see
            // spawn_vehicle's VehicleVoice::try_create call). Sim-thread
            // only, same as everything else in this post_step callback;
            // push_tick itself just writes into the voice's lock-free SPSC
            // queue, so this never blocks on the Runner's render thread.
            if (!vs.powertrain.engines.empty()) {
                const auto voice_it = vehicle_voice_by_name_.find(name);
                if (voice_it != vehicle_voice_by_name_.end()) {
                    voice_it->second->push_tick(vs.powertrain.engines[0], world_->sim_time());
                }
            }
            snap.vehicles.push_back(std::move(vs));

            const auto chassis_name_it = vehicle_chassis_body_name_.find(name);
            if (chassis_name_it != vehicle_chassis_body_name_.end()) {
                const auto body_it = bodies_by_name_.find(chassis_name_it->second);
                if (body_it != bodies_by_name_.end()) {
                    const ps::Pose chassis_pose = world_->get_pose(body_it->second); // safe: sim thread, right after step()

                    // Gamepad rumble (haptics_core.h): chassis PROPER
                    // (gravity-removed) linear acceleration, CHASSIS frame -
                    // finite-difference the chassis's own world-frame linear
                    // velocity (World::get_motion) between this tick and the
                    // previous one, at the World's fixed tick dt (post_step
                    // runs exactly once per World::step(), so the sample
                    // spacing is exactly world_->dt() - the SIM's own 240 Hz,
                    // not whatever rate a reader happens to poll at). Proper
                    // acceleration = coordinate acceleration (dv/dt) minus
                    // the gravity vector (what an accelerometer bolted to the
                    // chassis reads: ~0 in free fall, ~+9.81 upward at rest -
                    // matches haptics_core.h's HapticsInput::accel_x/y/z
                    // contract), then rotated from world into the chassis's
                    // OWN frame (orientation.inverse().rotate, world->local)
                    // using this tick's orientation. First tick after a fresh
                    // spawn or a reset_vehicle teleport (chassis_accel_state_
                    // erased there) has no previous sample - accel_local
                    // stays zero rather than reading a bogus spike.
                    ChassisAccelState& accel_state = chassis_accel_state_[name];
                    const ps::Motion chassis_motion = world_->get_motion(body_it->second);
                    ps::Vec3 accel_local = ps::Vec3::zero();
                    if (accel_state.valid) {
                        const ps::Vec3 accel_coord_world = (chassis_motion.linear - accel_state.prev_velocity_world) / world_->dt();
                        const ps::Vec3 accel_proper_world = accel_coord_world - gravity_;
                        accel_local = chassis_pose.orientation.inverse().rotate(accel_proper_world);
                    }
                    accel_state.prev_velocity_world = chassis_motion.linear;
                    accel_state.valid = true;
                    // NOTE: `vs` was already moved into snap.vehicles above
                    // (push_back(std::move(vs))) - write through the pushed
                    // element itself (snap.vehicles' last entry is exactly
                    // this vehicle's, single-threaded, nothing else pushes
                    // to snap.vehicles between the two), not the now-moved-
                    // from local `vs`.
                    snap.vehicles.back().accel_local = accel_local;
                    snap.vehicles.back().ground_speed_mps = chassis_motion.linear.length();

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

    // ps_audio A11: start every vehicle's voice Runner thread alongside the
    // sim thread (start() is idempotent, see VehicleVoice::start's own
    // comment - rebuild_post_step_and_start can run more than once across
    // a scenario reload without double-starting anything).
    for (auto& [name, voice] : vehicle_voice_by_name_) {
        voice->start();
    }

    sim_thread_->start();
    if(engine_audio_bank_enabled_.load()) for(const auto& entry:vehicle_voice_by_name_) {
        const auto def=engine_definition_paths_.find(entry.first);
        if(def!=engine_definition_paths_.end()) {
            const auto cache=(std::filesystem::path(def->second).parent_path().parent_path().parent_path()/"out"/"godot_engine_cache").string();
            const auto override_path=requested_turbo_paths_.find(entry.first);
            enqueue_bank_request(BankRequest{entry.first,def->second,override_path==requested_turbo_paths_.end()?"":override_path->second,cache,override_path!=requested_turbo_paths_.end()});
        }
    }
}

void PsSimulation::stop() {
    { std::lock_guard<std::mutex> lock(turbo_update_mutex_); bank_worker_stop_=true; }
    bank_request_cv_.notify_all();
    if(turbo_generation_thread_.joinable()) turbo_generation_thread_.join();
    { std::lock_guard<std::mutex> lock(turbo_update_mutex_); bank_requests_.clear(); pending_turbo_updates_.clear(); }
    if (sim_thread_) sim_thread_->stop();
    // ps_audio A11: stop every voice's Runner thread too, so nothing keeps
    // rendering (or holding its thread) once the sim itself is stopped.
    for (auto& [name, voice] : vehicle_voice_by_name_) {
        voice->stop();
    }
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
    const auto it = body_order_index_.find(name);
    if (it == body_order_index_.end()) return godot::Transform3D();

    // Render-time interpolation (render_interp.h): race-free via
    // PsSimulation's OWN triple buffer (adapter_snapshots_, see
    // ensure_render_frame_cached()'s doc comment) rather than World's own
    // snapshot triple buffer - the core one only ever carries the latest
    // tick's pose, not the (prev, latest, publish_time) this needs to blend.
    ensure_render_frame_cached();
    if (!render_sample_.valid || it->second >= render_sample_.poses.size()) return godot::Transform3D();
    return iso_to_godot_transform(render_sample_.poses[it->second], origin_rebase_.origin());
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

    // Render-time interpolation (render_interp.h) - see get_body_transform's
    // comment; same frame-cached (snapshot, alpha) pair as every other pose
    // read this frame.
    ensure_render_frame_cached();
    if (!render_sample_.valid || order_index >= articulation_pose_offset_.size()) return godot::Transform3D();
    const std::size_t base = articulation_pose_offset_[order_index];
    const auto links_it = articulation_link_counts_.find(articulation_order_[order_index]);
    const std::size_t link_count = links_it == articulation_link_counts_.end() ? 0 : links_it->second;
    const std::size_t flat = (link_index >= 0 && static_cast<std::size_t>(link_index) < link_count) ? base + 1 + static_cast<std::size_t>(link_index) : base;
    if (flat >= render_sample_.poses.size()) return godot::Transform3D();
    return iso_to_godot_transform(render_sample_.poses[flat], origin_rebase_.origin());
}

void PsSimulation::ensure_render_frame_cached() const {
    // Engine::get_singleton() is always valid once godot-cpp's own
    // extension entry point has run (register_types.cpp) - PsSimulation is
    // a godot::Node, which cannot exist before that - so no null check is
    // needed here for the normal running case; this function is only ever
    // called from get_body_transform/get_articulation_link_transform,
    // never from a plain-C++ test (those exercise render_interp.h directly,
    // Godot-free - see adapters/godot/tests/test_render_interp.cpp).
    const std::uint64_t frame = godot::Engine::get_singleton()->get_process_frames();
    if (cached_render_snapshot_ != nullptr && frame == cached_render_frame_) return; // already sampled this Godot frame
    cached_render_frame_ = frame;
    cached_render_snapshot_ = &const_cast<TripleBuffer<AdapterSnapshot>&>(adapter_snapshots_).read();
    const double tick_dt_s = world_ ? world_->dt() : 0.0;
    const auto now = std::chrono::steady_clock::now();
    last_sample_time_us_ = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    if (render_legacy_alpha_) {
        // A/B diagnostic: the old publish-time alpha on the snapshot's
        // (prev, latest) pairs, written into the same flat sample layout.
        cached_render_alpha_ = render_interp_alpha(cached_render_snapshot_->publish_time, now, tick_dt_s);
        const AdapterSnapshot& snap = *cached_render_snapshot_;
        render_sample_.poses.resize(history_pose_scratch_.size());
        for (std::size_t i = 0; i < snap.bodies.size() && i < render_sample_.poses.size(); ++i) {
            render_sample_.poses[i] = interpolate_pose(snap.bodies[i], cached_render_alpha_);
        }
        for (std::size_t a = 0; a < snap.articulations.size() && a < articulation_pose_offset_.size(); ++a) {
            const std::size_t base = articulation_pose_offset_[a];
            render_sample_.poses[base] = interpolate_pose(snap.articulations[a].root, cached_render_alpha_);
            for (std::size_t l = 0; l < snap.articulations[a].links.size(); ++l) {
                render_sample_.poses[base + 1 + l] = interpolate_pose(snap.articulations[a].links[l], cached_render_alpha_);
            }
        }
        render_sample_.valid = !snap.bodies.empty() || !snap.articulations.empty();
        render_sample_.alpha = cached_render_alpha_;
        render_sample_.tick_a = snap.tick > 0 ? snap.tick - 1 : 0;
        render_sample_.tick_b = snap.tick;
        render_sample_.late = render_sample_.early = render_sample_.stepped = false;
    } else {
        pose_history_.sample(render_time_for(now, tick_dt_s, render_delay_ticks_), render_sample_);
        cached_render_alpha_ = render_sample_.alpha;
    }
}

godot::Dictionary PsSimulation::get_render_diagnostics() const {
    godot::Dictionary d;
    d["legacy_alpha"] = render_legacy_alpha_;
    d["delay_ticks"] = render_delay_ticks_;
    d["frames_sampled"] = static_cast<std::int64_t>(pose_history_.frames_sampled());
    d["frames_late"] = static_cast<std::int64_t>(pose_history_.frames_late());
    d["frames_early"] = static_cast<std::int64_t>(pose_history_.frames_early());
    d["frames_stepped"] = static_cast<std::int64_t>(pose_history_.frames_stepped());
    d["alpha"] = render_sample_.alpha;
    d["tick_a"] = static_cast<std::int64_t>(render_sample_.tick_a);
    d["tick_b"] = static_cast<std::int64_t>(render_sample_.tick_b);
    d["late"] = render_sample_.late;
    d["sample_time_us"] = last_sample_time_us_;
    d["resyncs"] = sim_thread_ ? static_cast<std::int64_t>(sim_thread_->resync_count()) : 0;
    godot::PackedInt64Array lateness;
    godot::PackedInt64Array gap;
    for (std::size_t i = 0; i < publish_lateness_hist_.size(); ++i) {
        lateness.push_back(static_cast<std::int64_t>(publish_lateness_hist_.count(i)));
        gap.push_back(static_cast<std::int64_t>(publish_gap_hist_.count(i)));
    }
    d["publish_lateness_hist"] = lateness;
    d["publish_gap_hist"] = gap;
    d["hist_bin_ms"] = static_cast<double>(publish_lateness_hist_.bin_ns()) / 1.0e6;
    return d;
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
    const AdapterSnapshot& snap = render_snapshot();
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

bool PsSimulation::import_test_ground(const String& track_dir_absolute_path, std::int64_t max_resident_tiles, std::int64_t max_tile_fills_per_tick, const String& surfaces_path_override) {
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
    const TestGroundLoadResult result = import_test_ground_real(track_dir, *world_, surfaces, tcfg, 64, to_std_string(surfaces_path_override));
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
        // ps_audio A11: a .psmaps disk cache (I7c C16b) for a simulated
        // engine's load-time-generated maps (e.g. demo_i4_pipes.json's
        // cycle block, car_sedan_pipes.json) - without it every launch
        // regenerates them (0.17-2.2 s per engine, see core's CLAUDE.md),
        // which is fine once but a needless wait on every "drive.ps1" run.
        // Derived from the vehicle file's own path rather than a new
        // GDExtension parameter: <repo_root>/out/godot_engine_cache, where
        // repo_root is two directories up from data/vehicles/<file>.json.
        // A vehicle with no simulated/cycle engine ignores cache_dir
        // entirely (EngineMapOptions' own contract), so this is a no-op
        // for every non-pipe-engine vehicle.
        ps::io::EngineMapOptions map_options;
        {
            const std::string vpath = to_std_string(vehicle_json_absolute_path);
            const std::filesystem::path vehicle_dir = std::filesystem::path(vpath).parent_path();
            const std::filesystem::path repo_root = vehicle_dir.parent_path().parent_path();
            map_options.cache_dir = (repo_root / "out" / "godot_engine_cache").string();
        }
        // A cache miss generates the maps here (car_hyper's 8 x 6 grid is
        // ~30 s on one thread); a pool that lives only for the load, as in
        // ps_run --map-workers, cuts that to a few seconds.
        auto map_pool = std::make_unique<ps::jobs::JobSystem>(std::max(1, ps::jobs::physical_core_count()));
        map_options.pool = map_pool.get();
        ps::vehicle::VehicleDesc desc = ps::io::load_vehicle_json(to_std_string(vehicle_json_absolute_path), map_options);
        map_pool.reset();
        {
            const auto vpath=to_std_string(vehicle_json_absolute_path);
            std::ifstream file(vpath);const auto json=nlohmann::json::parse(file);
            for(const auto& c:json.at("powertrain").at("components")) if(c.at("type")=="engine" && c.contains("ref"))
                engine_definition_paths_[name]=(std::filesystem::path(vpath).parent_path()/c.at("ref").get<std::string>()).lexically_normal().string();
        }
        const ps::VehicleId vid = world_->create_vehicle(desc, body_it->second);
        vehicles_by_name_[name] = vid;
        vehicle_chassis_body_name_[name] = chassis_name;
        vehicle_order_.push_back(name);
        vehicle_terrain_interest_id_[name] = next_terrain_interest_id_++;
        vehicle_stream_radius_m_[name] = stream_radius_m;
        vehicle_desc_by_name_[name] = std::move(desc);

        // ps_audio A11: build this vehicle's voice from its OWN already-
        // loaded engine desc (vehicle_desc_by_name_[name], just stored
        // above) - no second file load, no separate "sound engine". A
        // vehicle whose engine has no "cycle" block (most of them) simply
        // gets no voice; logged once, not an error (see vehicle_voice.h).
        {
            std::string reason;
            std::unique_ptr<VehicleVoice> voice = VehicleVoice::try_create(vehicle_desc_by_name_.at(name), reason);
            godot::UtilityFunctions::print(String("ps_godot: vehicle \"") + vehicle_name + "\" engine voice: " +
                                            String(reason.c_str()));
            if (voice) {
                vehicle_voice_underrun_count_[name] = 0;
                vehicle_voice_by_name_[name] = std::move(voice);
            }
        }
        return true;
    } catch (const std::exception& e) {
        last_error_ = String("spawn_vehicle failed: ") + String(e.what());
        godot::UtilityFunctions::print("ps_godot: ", last_error_);
        return false;
    }
}

void PsSimulation::enqueue_bank_request(BankRequest request) {
    {
        std::lock_guard<std::mutex> lock(turbo_update_mutex_);
        bank_worker_stop_=false;
        // Latest request for this vehicle wins without disturbing its current audio.
        bank_requests_.erase(std::remove_if(bank_requests_.begin(),bank_requests_.end(),[&](const auto& r){ return r.vehicle_name==request.vehicle_name; }),bank_requests_.end());
        bank_requests_.push_back(std::move(request));engine_audio_bank_status_="Queued";
    }
    if(!turbo_generation_thread_.joinable()) turbo_generation_thread_=std::thread([this] {
        for(;;) {
            BankRequest task;
            { std::unique_lock<std::mutex> lock(turbo_update_mutex_);
                bank_request_cv_.wait(lock,[this]{return bank_worker_stop_ || !bank_requests_.empty();});
                if(bank_worker_stop_) break;
                task=std::move(bank_requests_.front());bank_requests_.erase(bank_requests_.begin());
                engine_audio_bank_status_="Generating or loading cache";
            }
            try {
                if(task.apply_physics) {
                    ps::io::EngineMapOptions options;options.cache_dir=task.cache_dir;
                    auto loaded=ps::io::load_engine_json(task.engine_path,options);
                    auto desc=std::get<ps::drivetrain::SimulatedEngineDesc>(loaded);
                    if(!task.turbo_path.empty()) desc.turbo_pair=ps::io::load_turbo_configuration_json(task.turbo_path,task.cache_dir);
                    if(!desc.turbo_pair) throw std::runtime_error("Engine has no default turbo configuration");
                    std::lock_guard<std::mutex> lock(turbo_update_mutex_);
                    if(bank_worker_stop_ || std::any_of(bank_requests_.begin(),bank_requests_.end(),[&](const auto& r){return r.vehicle_name==task.vehicle_name;})) continue;
                    pending_turbo_updates_.push_back(PendingTurboUpdate{task.vehicle_name,*desc.turbo_pair});
                    turbo_configuration_status_="Ready; waiting for tick boundary";
                }
                if(!engine_audio_bank_enabled_.load()) {
                    std::lock_guard<std::mutex> lock(turbo_update_mutex_);
                    engine_audio_bank_status_="Disabled";
                    continue;
                }
                auto bank=ps_tools::prepare_engine_audio_bank(task.engine_path,task.turbo_path,task.cache_dir,[this,&task] {
                    std::lock_guard<std::mutex> lock(turbo_update_mutex_);
                    return bank_worker_stop_ || std::any_of(bank_requests_.begin(),bank_requests_.end(),[&](const auto& r){return r.vehicle_name==task.vehicle_name;});
                });
                std::lock_guard<std::mutex> lock(turbo_update_mutex_);
                const bool superseded=std::any_of(bank_requests_.begin(),bank_requests_.end(),[&](const auto& r){ return r.vehicle_name==task.vehicle_name; });
                if(!bank_worker_stop_ && !superseded) {
                    const auto voice=vehicle_voice_by_name_.find(task.vehicle_name);
                    if(voice!=vehicle_voice_by_name_.end()) voice->second->set_audio_bank(std::move(bank));
                    engine_audio_bank_status_="Ready";
                }
            } catch(const std::exception& e) {
                std::lock_guard<std::mutex> lock(turbo_update_mutex_);
                if(!bank_worker_stop_ && !std::any_of(bank_requests_.begin(),bank_requests_.end(),[&](const auto& r){return r.vehicle_name==task.vehicle_name;}))
                    engine_audio_bank_status_=std::string("Failed; keeping previous audio: ")+e.what();
            }
        }
    });
    bank_request_cv_.notify_one();
}

bool PsSimulation::request_vehicle_turbo_configuration(const String& vehicle_name,const String& path) {
    const auto name=to_std_string(vehicle_name);const auto engine=engine_definition_paths_.find(name);
    if(engine==engine_definition_paths_.end() || vehicle_voice_by_name_.find(name)==vehicle_voice_by_name_.end()) {
        last_error_="Vehicle has no referenced simulated cycle engine";return false;
    }
    const auto cache=(std::filesystem::path(engine->second).parent_path().parent_path().parent_path()/"out"/"godot_engine_cache").string();
    requested_turbo_paths_[name]=to_std_string(path);
    enqueue_bank_request(BankRequest{name,engine->second,to_std_string(path),cache,true});
    return true;
}
void PsSimulation::set_engine_audio_bank_enabled(bool enabled) { engine_audio_bank_enabled_.store(enabled); }
bool PsSimulation::get_engine_audio_bank_enabled() const { return engine_audio_bank_enabled_.load(); }
String PsSimulation::get_turbo_configuration_status() const {
    std::lock_guard<std::mutex> lock(turbo_update_mutex_);return String(turbo_configuration_status_.c_str());
}
String PsSimulation::get_engine_audio_bank_status() const {
    std::lock_guard<std::mutex> lock(turbo_update_mutex_);return String(engine_audio_bank_status_.c_str());
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

bool PsSimulation::install_environment(const String& climate_absolute_path, double time_scale, const String& start_season,
                                       std::int64_t start_day, double start_clock_hours) {
    if (!world_) {
        last_error_ = "install_environment: no world (call load_scenario first)";
        return false;
    }
    if (is_running()) {
        last_error_ = "install_environment: setup only, call it before start()";
        return false;
    }
    try {
        const ps::environment::ClimateDesc climate = ps::io::load_climate_json(to_std_string(climate_absolute_path));
        ps::environment::EnvironmentConfig cfg;
        cfg.time_scale = time_scale;
        if (start_day >= 0) {
            ps::environment::EnvironmentStart start;
            start.season = to_std_string(start_season);
            start.day = static_cast<int>(start_day);
            // The start is given as a 24-hour clock reading: a game day of any length maps
            // onto 24 h (EnvironmentSnapshot::clock_hours), so resolve it against that
            // day's own length.
            const ps::environment::Calendar calendar(climate);
            const int season = start.season.empty() ? 0 : calendar.season_index(start.season);
            if (season < 0) throw std::invalid_argument("unknown start season '" + start.season + "'");
            if (start.day < 0 || start.day >= calendar.season_days(season)) throw std::invalid_argument("start day out of range for its season");
            const ps::real day_length_s = calendar.day(calendar.season_start_day(season) + start.day).length_s();
            start.time_of_day_s = ps::real(start_clock_hours) / 24.0 * day_length_s;
            cfg.start = start;
        }
        world_->install_environment(std::make_shared<ps::environment::Environment>(climate, cfg));
        last_error_ = String();
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::print("ps_godot: install_environment failed: ", e.what());
        last_error_ = String("install_environment failed: ") + String(e.what());
        return false;
    }
}

bool PsSimulation::has_environment() const { return render_snapshot().has_environment; }

Dictionary PsSimulation::get_environment() const {
    Dictionary d;
    const AdapterSnapshot& snap = render_snapshot();
    if (!snap.has_environment) return d;
    const ps::environment::EnvironmentSnapshot& e = snap.environment;
    d["t_env_s"] = static_cast<double>(e.t_env_s);
    d["time_scale"] = static_cast<double>(e.time_scale);
    d["year"] = static_cast<std::int64_t>(e.year);
    d["season"] = static_cast<int>(e.season);
    d["season_name"] = String(e.season_name);
    d["day_in_season"] = static_cast<int>(e.day_in_season);
    d["day_of_year"] = static_cast<int>(e.day_of_year);
    d["time_of_day_s"] = static_cast<double>(e.time_of_day_s);
    d["day_s"] = static_cast<double>(e.day_s);
    d["night_s"] = static_cast<double>(e.night_s);
    d["day_fraction"] = static_cast<double>(e.day_fraction);
    d["clock_hours"] = static_cast<double>(e.clock_hours);
    d["is_day"] = e.is_day;
    d["sun_elevation_deg"] = static_cast<double>(e.sun_elevation_rad * 180.0 / 3.14159265358979323846);
    d["sun_azimuth_deg"] = static_cast<double>(e.sun_azimuth_rad * 180.0 / 3.14159265358979323846);
    // A direction: the same sim -> Godot axis fix-up as every other direction
    // (frame_convert_core.h), no origin shift.
    d["sun_direction"] = to_godot(iso_to_godot_dir(e.sun_dir_world));
    d["ghi_w_m2"] = static_cast<double>(e.ghi_w_m2);
    d["cloud_cover"] = static_cast<double>(e.cloud_cover);
    return d;
}

Dictionary PsSimulation::get_vehicle_powertrain(const String& vehicle_name) const {
    Dictionary d;
    const std::size_t order_index = index_in_order(vehicle_order_, to_std_string(vehicle_name));
    if (order_index == static_cast<std::size_t>(-1)) return d;
    const AdapterSnapshot& snap = render_snapshot();
    if (order_index >= snap.vehicles.size()) return d;
    const ps::drivetrain::PowertrainSnapshot& p = snap.vehicles[order_index].powertrain;

    d["gear"] = static_cast<int>(p.gear);
    d["gear_target"] = static_cast<int>(p.gear_target);
    d["shift_phase"] = String(shift_phase_name(p.shift_phase));
    d["rpm"] = static_cast<float>(p.engine_rpm);
    d["engine_state"] = String(engine_state_name(p.engine_state));
    d["speedo_kmh"] = static_cast<float>(p.speedo_mps * 3.6);
    d["clutch_engagement"] = static_cast<float>(p.clutch_engagement);
    d["fuel_flow_g_s"] = static_cast<double>(p.fuel_flow_g_s);
    d["fuel_mass_kg"] = static_cast<double>(p.fuel_mass_kg);
    d["turbo_rpm"] = p.engines.empty() ? 0.0 : static_cast<double>(p.engines[0].turbo_rpm);
    d["boost_bar"] = p.engines.empty() ? 0.0 : static_cast<double>(p.engines[0].boost_bar);
    d["p_exhaust_bar"] = p.engines.empty() ? 0.0 : static_cast<double>(p.engines[0].p_exhaust_pa) / 1e5; // absolute
    d["drive_power_kw"] = p.engines.empty() ? 0.0 : static_cast<double>(-p.driveline.clutch_torque * p.engines[0].omega / 1000.0);
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

    // Tractor demo (2026-09-28): fields that are 0/empty for a vehicle with
    // no such drivetrain component (car_sedan) - see CLAUDE.md's "Adapter
    // seams" for each field's own PowertrainSnapshot contract.
    d["awd_engagement"] = static_cast<float>(p.awd_engagement);
    d["park_lock_engaged"] = p.park_lock_engaged != 0.0;
    d["pto_engagement"] = static_cast<float>(p.pto_engagement);
    d["pto_mode"] = static_cast<float>(p.pto_mode);
    d["pto_rpm"] = static_cast<float>(p.pto_rpm);
    d["governor_setpoint_rpm"] = static_cast<float>(p.governor_setpoint_rpm);
    d["hand_throttle"] = static_cast<float>(p.hand_throttle);

    TypedArray<Dictionary> diffs;
    for (int i = 0; i < static_cast<int>(p.diff_count); ++i) {
        const ps::drivetrain::DiffSnapshot& ds = p.diffs[static_cast<std::size_t>(i)];
        Dictionary dd;
        dd["bias"] = static_cast<float>(ds.bias);
        dd["locked"] = ds.locked;
        dd["lock_pending"] = ds.lock_pending;
        diffs.push_back(dd);
    }
    d["diffs"] = diffs;
    d["diff_count"] = static_cast<int>(p.diff_count);
    return d;
}

Dictionary PsSimulation::get_vehicle_gauge_info(const String& vehicle_name) const {
    Dictionary d;
    const auto it = vehicle_desc_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicle_desc_by_name_.end()) return d;

    // ComponentParams (ps/drivetrain/powertrain_desc.h) is a closed variant
    // visited exhaustively elsewhere in core; here we only need the first
    // engine/gearbox component present - PHYS-033 I3's vehicle files each
    // carry exactly one of each (see data/vehicles/*.json's own
    // "powertrain.components").
    ps::real idle_rpm = 0.0, limiter_rpm = 0.0;
    ps::real fuel_capacity_kg = 0.0, fuel_density_kg_m3 = 0.0;
    std::int64_t gear_count = 0;
    for (const ps::drivetrain::ComponentDesc& c : it->second.powertrain.components) {
        if (const auto* engine = std::get_if<ps::drivetrain::TorqueMapEngineDesc>(&c.params)) {
            idle_rpm = engine->idle_rpm;
            limiter_rpm = engine->limiter.rpm;
        } else if (const auto* sim = std::get_if<ps::drivetrain::SimulatedEngineDesc>(&c.params)) {
            idle_rpm = sim->idle_rpm;
            limiter_rpm = sim->limiter.rpm;
        } else if (const auto* tank = std::get_if<ps::drivetrain::FuelTankDesc>(&c.params)) {
            fuel_capacity_kg += tank->capacity_kg;
            fuel_density_kg_m3 = tank->fuel.density_kg_m3;
        } else if (const auto* gearbox = std::get_if<ps::drivetrain::GearboxDesc>(&c.params)) {
            gear_count = static_cast<std::int64_t>(gearbox->forward_ratios.size());
        }
    }

    d["idle_rpm"] = static_cast<float>(idle_rpm);
    d["limiter_rpm"] = static_cast<float>(limiter_rpm);
    d["gear_count"] = gear_count;
    d["fuel_capacity_kg"] = static_cast<double>(fuel_capacity_kg);
    d["fuel_density_kg_m3"] = static_cast<double>(fuel_density_kg_m3);
    // Data-driven and empty today on purpose - see this method's header
    // comment (ps_simulation.h) for why.
    d["fitted_assists"] = godot::PackedStringArray();
    return d;
}

const WheelDynamicSnapshot* PsSimulation::dynamic_wheel(const std::string& vehicle_name, int wheel_index) const {
    const std::size_t order_index = index_in_order(vehicle_order_, vehicle_name);
    if (order_index == static_cast<std::size_t>(-1)) return nullptr;
    const AdapterSnapshot& snap = render_snapshot();
    if (order_index >= snap.vehicles.size()) return nullptr;
    const std::vector<WheelDynamicSnapshot>& wheels = snap.vehicles[order_index].wheels;
    if (wheel_index < 0 || static_cast<std::size_t>(wheel_index) >= wheels.size()) return nullptr;
    return &wheels[static_cast<std::size_t>(wheel_index)];
}

Vector3 PsSimulation::get_vehicle_chassis_accel_local(const String& vehicle_name) const {
    const std::size_t order_index = index_in_order(vehicle_order_, to_std_string(vehicle_name));
    if (order_index == static_cast<std::size_t>(-1)) return Vector3();
    const AdapterSnapshot& snap = render_snapshot();
    if (order_index >= snap.vehicles.size()) return Vector3();
    const ps::Vec3& a = snap.vehicles[order_index].accel_local;
    return Vector3(static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z));
}

float PsSimulation::get_wheel_compression(const String& vehicle_name, int wheel_index) const {
    const WheelDynamicSnapshot* w = dynamic_wheel(to_std_string(vehicle_name), wheel_index);
    return w == nullptr ? 0.0f : static_cast<float>(w->compression);
}

Vector2 PsSimulation::get_vehicle_steer_geometry(const String& vehicle_name) const {
    const auto it = vehicle_desc_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicle_desc_by_name_.end()) return Vector2();
    ps::real front_x = 0.0, rear_x = 0.0;
    int n_front = 0, n_rear = 0;
    for (const auto& w : it->second.wheels) {
        if (w.is_front) { front_x += w.attachment_local.x; ++n_front; }
        else { rear_x += w.attachment_local.x; ++n_rear; }
    }
    if (n_front == 0 || n_rear == 0) return Vector2();
    const ps::real wheelbase = front_x / n_front - rear_x / n_rear;
    return Vector2(static_cast<float>(wheelbase), static_cast<float>(it->second.steering.max_wheel_angle));
}

float PsSimulation::get_vehicle_ground_speed_mps(const String& vehicle_name) const {
    const std::size_t order_index = index_in_order(vehicle_order_, to_std_string(vehicle_name));
    if (order_index == static_cast<std::size_t>(-1)) return 0.0f;
    const AdapterSnapshot& snap = render_snapshot();
    if (order_index >= snap.vehicles.size()) return 0.0f;
    return static_cast<float>(snap.vehicles[order_index].ground_speed_mps);
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

// ps_audio A11: the Godot-facing voice-readback API. VehicleVoice/its
// Runner thread are built and started in spawn_vehicle/rebuild_post_step_
// and_start above; these just expose read-only status plus the one
// side-effecting call (read_vehicle_voice_audio, which drains the
// Runner's ring and counts underruns) to GDScript.
bool PsSimulation::has_vehicle_voice(const String& vehicle_name) const {
    return vehicle_voice_by_name_.find(to_std_string(vehicle_name)) != vehicle_voice_by_name_.end();
}

double PsSimulation::get_vehicle_voice_sample_rate_hz(const String& vehicle_name) const {
    const auto it = vehicle_voice_by_name_.find(to_std_string(vehicle_name));
    return it == vehicle_voice_by_name_.end() ? 0.0 : it->second->sample_rate_hz();
}

int PsSimulation::get_vehicle_voice_channel_count(const String& vehicle_name) const {
    const auto it = vehicle_voice_by_name_.find(to_std_string(vehicle_name));
    return it == vehicle_voice_by_name_.end() ? 0 : it->second->channel_count();
}

godot::PackedVector3Array PsSimulation::get_vehicle_voice_source_positions(const String& vehicle_name) const {
    godot::PackedVector3Array out;
    const auto it = vehicle_voice_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicle_voice_by_name_.end()) return out;
    for (int c = 0; c < it->second->channel_count(); ++c) {
        const auto p = it->second->source(c).position_m;
        out.push_back(godot::Vector3(p.x, p.y, p.z));
    }
    return out;
}

bool PsSimulation::connect_vehicle_voice_spatial(const String& name,const godot::Ref<PsSpatialAudio>& spatial,int latency_ms) {
    const auto it=vehicle_voice_by_name_.find(to_std_string(name));
    if(it==vehicle_voice_by_name_.end() || spatial.is_null()) return false;
    return spatial->attach_native_source(it->second->native_output()->acquire(),latency_ms,static_cast<float>(1./voice_full_scale_pa_));
}

godot::PackedFloat32Array PsSimulation::read_vehicle_voice_audio(const String& vehicle_name, int frames) {
    godot::PackedFloat32Array out;
    if (frames <= 0) return out;
    const std::string name = to_std_string(vehicle_name);
    const auto it = vehicle_voice_by_name_.find(name);
    if (it == vehicle_voice_by_name_.end()) return out;

    const int channels = it->second->channel_count();
    if (channels <= 0) return out;

    // Local scratch, not a persistent member: this runs on Godot's main
    // thread from _process, at most a few hundred frames at a time (one
    // AudioStreamGenerator fill per frame) - not the sim's own hot path,
    // so an ordinary per-call allocation is fine here (unlike core/audio's
    // own zero-alloc-after-warmup rules, which apply to the sim/render
    // threads, not this adapter-side readback call).
    std::vector<float> scratch(static_cast<std::size_t>(frames) * static_cast<std::size_t>(channels));
    const std::size_t got = it->second->runner().read(scratch.data(), static_cast<std::size_t>(frames));
    it->second->record_output(scratch.data(), got);
    // Only an EMPTY ring counts: Godot asks for its whole free buffer
    // (up to 0.2 s), which the ring normally only partly holds - a partial
    // read is not a dropout. Godot-side dropouts are
    // AudioStreamGeneratorPlayback.get_skips() (main.gd prints both).
    if (got == 0) {
        vehicle_voice_underrun_count_[name] += 1;
    }
    const float gain = static_cast<float>(1.0 / voice_full_scale_pa_);
    const int got_samples = static_cast<int>(got) * channels;
    out.resize(got_samples);
    for (int i = 0; i < got_samples; ++i) {
        out[i] = scratch[static_cast<std::size_t>(i)] * gain;
    }
    return out;
}

void PsSimulation::set_voice_full_scale_pa(double pa) {
    if (pa > 0.0) voice_full_scale_pa_ = pa;
}

double PsSimulation::get_voice_full_scale_pa() const { return voice_full_scale_pa_; }

std::int64_t PsSimulation::get_vehicle_voice_underrun_count(const String& vehicle_name) const {
    const auto it = vehicle_voice_underrun_count_.find(to_std_string(vehicle_name));
    return it == vehicle_voice_underrun_count_.end() ? 0 : it->second;
}

godot::Dictionary PsSimulation::take_vehicle_voice_timing(const String& vehicle_name) {
    godot::Dictionary d;
    const auto it = vehicle_voice_by_name_.find(to_std_string(vehicle_name));
    if (it == vehicle_voice_by_name_.end() || !it->second) return d;
    ps_audio::Voice& v = it->second->voice();
    d["hitches"] = static_cast<std::int64_t>(v.published_hitch_count());
    d["arrivals"] = static_cast<std::int64_t>(v.published_arrivals());
    d["max_arrival_gap_ms"] = static_cast<double>(v.take_max_arrival_gap_s()) * 1000.0;
    d["max_crank_error_deg"] = static_cast<double>(v.take_max_crank_error_deg());
    d["crank_saturated_samples"] = static_cast<std::int64_t>(v.take_crank_saturated_samples());
    d["crank_saturation_flips"] = static_cast<std::int64_t>(v.take_crank_saturation_flips());
    d["rate_min"] = static_cast<double>(v.take_min_rate());
    d["rate_max"] = static_cast<double>(v.take_max_rate());
    d["max_step_ms"] = sim_thread_ ? static_cast<double>(sim_thread_->take_max_step_ns()) / 1.0e6 : -1.0;
    d["catch_up_wakeups"] = sim_thread_ ? static_cast<std::int64_t>(sim_thread_->catch_up_wakeups()) : -1;
    d["runner_max_block_ms"] = static_cast<double>(it->second->runner().max_block_render_ns()) / 1.0e6;
    d["runner_blocks"] = static_cast<std::int64_t>(it->second->runner().blocks_rendered());
    return d;
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
    ClassDB::bind_method(D_METHOD("import_test_ground", "track_dir_absolute_path", "max_resident_tiles", "max_tile_fills_per_tick", "surfaces_path_override"), &PsSimulation::import_test_ground, DEFVAL(512), DEFVAL(16), DEFVAL(String()));
    ClassDB::bind_method(D_METHOD("get_last_error"), &PsSimulation::get_last_error);
    ClassDB::bind_method(D_METHOD("terrain_resident_tile_count"), &PsSimulation::terrain_resident_tile_count);
    ClassDB::bind_method(D_METHOD("terrain_starved_tile_count"), &PsSimulation::terrain_starved_tile_count);

    ClassDB::bind_method(D_METHOD("create_body_dynamic", "name", "sim_position", "yaw_rad", "mass", "half_extents"), &PsSimulation::create_body_dynamic);
    ClassDB::bind_method(D_METHOD("spawn_vehicle", "vehicle_name", "vehicle_json_absolute_path", "chassis_body_name", "stream_radius_m"), &PsSimulation::spawn_vehicle);
    ClassDB::bind_method(D_METHOD("request_vehicle_turbo_configuration","vehicle_name","path"),&PsSimulation::request_vehicle_turbo_configuration);
    ClassDB::bind_method(D_METHOD("get_engine_audio_bank_status"),&PsSimulation::get_engine_audio_bank_status);
    ClassDB::bind_method(D_METHOD("set_engine_audio_bank_enabled","enabled"),&PsSimulation::set_engine_audio_bank_enabled);
    ClassDB::bind_method(D_METHOD("get_engine_audio_bank_enabled"),&PsSimulation::get_engine_audio_bank_enabled);
    ClassDB::bind_method(D_METHOD("get_turbo_configuration_status"),&PsSimulation::get_turbo_configuration_status);
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
    ClassDB::bind_method(D_METHOD("get_vehicle_gauge_info", "vehicle_name"), &PsSimulation::get_vehicle_gauge_info);
    // Weather plan W2
    ClassDB::bind_method(D_METHOD("install_environment", "climate_absolute_path", "time_scale", "start_season", "start_day", "start_clock_hours"),
                         &PsSimulation::install_environment, DEFVAL(1.0), DEFVAL(String()), DEFVAL(-1), DEFVAL(12.0));
    ClassDB::bind_method(D_METHOD("has_environment"), &PsSimulation::has_environment);
    ClassDB::bind_method(D_METHOD("get_environment"), &PsSimulation::get_environment);
    ClassDB::bind_method(D_METHOD("get_vehicle_chassis_accel_local", "vehicle_name"), &PsSimulation::get_vehicle_chassis_accel_local);
    ClassDB::bind_method(D_METHOD("get_wheel_compression", "vehicle_name", "wheel_index"), &PsSimulation::get_wheel_compression);
    ClassDB::bind_method(D_METHOD("get_vehicle_steer_geometry", "vehicle_name"), &PsSimulation::get_vehicle_steer_geometry);
    ClassDB::bind_method(D_METHOD("get_vehicle_ground_speed_mps", "vehicle_name"), &PsSimulation::get_vehicle_ground_speed_mps);
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

    // ps_audio A11
    ClassDB::bind_method(D_METHOD("get_vehicle_voice_source_positions", "vehicle_name"), &PsSimulation::get_vehicle_voice_source_positions);
    ClassDB::bind_method(D_METHOD("has_vehicle_voice", "vehicle_name"), &PsSimulation::has_vehicle_voice);
    ClassDB::bind_method(D_METHOD("connect_vehicle_voice_spatial", "vehicle_name", "spatial", "latency_ms"), &PsSimulation::connect_vehicle_voice_spatial,DEFVAL(12));
    ClassDB::bind_method(D_METHOD("get_vehicle_voice_sample_rate_hz", "vehicle_name"), &PsSimulation::get_vehicle_voice_sample_rate_hz);
    ClassDB::bind_method(D_METHOD("get_vehicle_voice_channel_count", "vehicle_name"), &PsSimulation::get_vehicle_voice_channel_count);
    ClassDB::bind_method(D_METHOD("read_vehicle_voice_audio", "vehicle_name", "frames"), &PsSimulation::read_vehicle_voice_audio);
    ClassDB::bind_method(D_METHOD("get_vehicle_voice_underrun_count", "vehicle_name"), &PsSimulation::get_vehicle_voice_underrun_count);
    ClassDB::bind_method(D_METHOD("take_vehicle_voice_timing", "vehicle_name"), &PsSimulation::take_vehicle_voice_timing);
    ClassDB::bind_method(D_METHOD("get_render_diagnostics"), &PsSimulation::get_render_diagnostics);
    ClassDB::bind_method(D_METHOD("set_render_delay_ticks", "ticks"), &PsSimulation::set_render_delay_ticks);
    ClassDB::bind_method(D_METHOD("get_render_delay_ticks"), &PsSimulation::get_render_delay_ticks);
    ClassDB::bind_method(D_METHOD("set_render_legacy_alpha", "legacy"), &PsSimulation::set_render_legacy_alpha);
    ClassDB::bind_method(D_METHOD("get_render_legacy_alpha"), &PsSimulation::get_render_legacy_alpha);
    ClassDB::bind_method(D_METHOD("set_voice_full_scale_pa", "pa"), &PsSimulation::set_voice_full_scale_pa);
    ClassDB::bind_method(D_METHOD("get_voice_full_scale_pa"), &PsSimulation::get_voice_full_scale_pa);
}

} // namespace ps_godot
