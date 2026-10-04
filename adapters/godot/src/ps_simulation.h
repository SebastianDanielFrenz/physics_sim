#include <condition_variable>
#include <thread>
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
// does not snapshot yet (articulation link poses, per-wheel vehicle
// telemetry, this tick's contact events - all captured inside SimThread's
// post_step callback, which runs synchronously on the sim thread right
// after World::step() returns, so there is no concurrency at the point of
// the read). No method here ever calls ps::World::get_pose/get_motion/
// wheel_telemetry/backend() directly while running() is true; see
// get_body_transform() and get_articulation_link_transform().
//
// PLAN.md M1: World-mutating setup (import_test_ground, create_body_
// dynamic, spawn_vehicle) must be called AFTER load_scenario() but BEFORE
// start() - exactly like every other "setup time" World call in this
// codebase (create_body, set_terrain_source, ...). start() is the point
// where this class captures the finished vehicle_order_/articulation_
// order_ into the post_step callback and hands the World to the sim
// thread; calling a setup method afterwards returns false without
// touching the World (see is_running() guards below).

#pragma once

#include "collision_import.h"
#include "fall_detector.h"
#include "frame_convert.h"
#include "frame_timing.h"
#include "origin_rebase.h"
#include "render_interp.h"
#include "sim_thread.h"
#include "triple_buffer.h"
#include "vehicle_voice.h"
#include "spatial_audio.h"
#include <godot_cpp/variant/packed_vector3_array.hpp>

#include "ps/drivetrain/powertrain_state.h"
#include "ps/environment/environment_snapshot.h"
#include "ps/io/scenario.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ps_godot {

// render_interp.h's InterpolatedPose (prev tick's pose + this tick's pose)
// is what every pose-bearing field below now carries, instead of a single
// ps::Pose - see this file's own "Render-time interpolation" section
// further down and render_interp.h's header comment for why (PLAN.md D3
// judder fix).
struct ArticulationLinkPoseSet {
    InterpolatedPose root; // World::link_frame(id, -1)
    std::vector<InterpolatedPose> links; // index == link_index
};

// PLAN.md M1 step 3/5: one wheel's DYNAMIC state (World::wheel_telemetry is
// not snapshot-buffered by core - reading it off the sim thread would race
// vehicle::System::step_all exactly like a raw get_pose() would race the
// backend, see this file's header comment). Static wheel geometry
// (attachment point, radius, steered/driven flags, ...) does not change
// per tick, so it is cached once at spawn_vehicle() time instead
// (vehicle_desc_by_name_) rather than duplicated into every tick's snapshot.
struct WheelDynamicSnapshot {
    ps::real compression = 0.0;
    ps::real omega = 0.0;
    ps::real slip_ratio = 0.0;
    ps::real slip_angle = 0.0;
    ps::real fx = 0.0, fy = 0.0, fz = 0.0;
    ps::SurfaceId surface = ps::kInvalidSurfaceId;
    // Real per-wheel steer angle (World::wheel_state's steer_angle, not the
    // wheel_telemetry fields above - see wheel_state.h). Added so
    // vehicle_visual.gd can animate steering from the ACTUAL simulated
    // Ackermann/anti-Ackermann/rear-steer result instead of recomputing an
    // approximation client-side from SteeringDesc params (this struct's own
    // earlier comment on why WheelState is a separate type from
    // WheelTelemetry applies here too - vault PHYS-020).
    ps::real steer_angle = 0.0;
};

struct VehicleDynamicSnapshot {
    std::vector<WheelDynamicSnapshot> wheels; // index == wheel index, matches vehicle_desc_by_name_'s wheels
    // PHYS-033 P4 I3 plan section 12.1: the adapter-facing powertrain
    // snapshot (gear/rpm/clutch/assist state, ...), copied from
    // World::powertrain_state(vid) in post_step - the same race-free path
    // as the wheels above (World::powertrain_state is sim-thread-only live
    // state, same contract as wheel_telemetry).
    ps::drivetrain::PowertrainSnapshot powertrain;
    // Gamepad rumble (haptics_core.h): the chassis's PROPER (gravity-
    // removed) linear acceleration, CHASSIS frame (x fwd, y left, z up),
    // computed once per SIM TICK (240 Hz) in post_step by finite-
    // differencing the chassis's own world-frame linear velocity between
    // consecutive ticks (World::get_motion, right after step() - safe,
    // same sim-thread contract as the chassis pose read just below this
    // struct's use in ps_simulation.cpp) and rotating the result into the
    // chassis's current orientation - see ps_simulation.cpp's post_step
    // lambda for the exact derivation and PsInputMap::_process for how
    // often a reader actually picks up a new value (render rate, which is
    // usually slower than 240 Hz, so the same value is read more than
    // once per new tick).
    ps::Vec3 accel_local = ps::Vec3::zero();
    // |chassis linear velocity|, m/s (haptics: slip only vibrates while
    // something moves over the ground).
    ps::real ground_speed_mps = 0.0;
};

struct AdapterSnapshot {
    // Indexed the same way as PsSimulation's articulations_by_name_ /
    // articulation_order_ (registration order at build_scenario time).
    std::vector<ArticulationLinkPoseSet> articulations;
    std::vector<ps::ContactEvent> contacts;
    // Indexed the same way as vehicle_order_ (registration order at
    // spawn_vehicle time, finalised when start() is called).
    std::vector<VehicleDynamicSnapshot> vehicles;
    // Weather plan W2: this tick's environment record (time + sun), copied
    // from World::environment_snapshot() in post_step; has_environment is
    // false while none is installed. A plain POD, so the per-tick copy never
    // allocates.
    bool has_environment = false;
    ps::environment::EnvironmentSnapshot environment;
    // Render-time interpolation (render_interp.h, PLAN.md D3 judder fix):
    // every body PsSimulation exposes by name (bodies_by_name_), indexed
    // the same way as PsSimulation::body_order_ (registration order,
    // finalised when start() is called - same pattern as vehicle_order_/
    // articulation_order_ above). Each entry carries both this tick's pose
    // and the previous tick's (InterpolatedPose), NOT just the latest -
    // get_body_transform() blends between them instead of snapping to the
    // latest one every render frame.
    std::vector<InterpolatedPose> bodies;
    std::uint64_t tick = 0;
    // Wall-clock time this snapshot was published (sim thread,
    // std::chrono::steady_clock - the same clock domain
    // ensure_render_frame_cached() reads via steady_clock::now() when
    // computing a render frame's blend factor). Default-constructed
    // (clock epoch) before the first tick ever publishes - harmless: every
    // pose vector above is also still empty at that point, so
    // get_body_transform/get_articulation_link_transform's own bounds
    // checks return an identity transform regardless of the resulting
    // (huge, but clamped) alpha.
    std::chrono::steady_clock::time_point publish_time;
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
    // the demo's free_cam.gd/chase_cam.gd do the former for the camera
    // itself.
    [[nodiscard]] godot::Vector3 rebase_focus(const godot::Vector3& focus_godot_position);
    [[nodiscard]] godot::Vector3 origin_as_godot_reference() const { return godot::Vector3(0, 0, 0); } // origin is always local (0,0,0) by construction

    // Godot-space position of the sim's fixed ABSOLUTE (0, 0, 0) - the
    // world's true origin (data/tracks/README.md: "Origin (0, 0, 0) is the
    // main spawn point"), which is NOT the same point as the floating
    // rebase origin above once that has moved. Pure query, no rebase side
    // effects (unlike rebase_focus, which both reads AND moves the shared
    // OriginRebase state) - for demo-only static world geometry whose own
    // local origin already sits at this fixed point (M1 visual pass:
    // terrain_visual.gd's test_ground.glb), so it can be re-placed every
    // frame exactly like a body/camera without corrupting the one shared
    // rebase focus (the vehicle/camera's own rebase_focus calls).
    [[nodiscard]] godot::Vector3 get_world_origin_godot_position() const;

    // Same fixed ISO-origin point as get_world_origin_godot_position(), but
    // as a full Transform3D (rotation included, PLAN.md P2's "single
    // ISO->Godot conversion" via frame_convert.h::iso_to_godot_transform,
    // orientation = identity in ISO/ENU axes - see data/tracks/README.md's
    // "Frame" section). A demo-authored asset whose OWN local axes are
    // already ISO/ENU-aligned at rest (e.g. terrain_visual.gd's
    // test_ground.glb, once its glTF-local axes are permuted into ISO order
    // - see that script's header comment) needs this ROTATION applied too,
    // not just the position offset get_world_origin_godot_position() alone
    // gives: ISO/ENU axes are not the same as Godot's own axes (x forward/y
    // left/z up vs. x right/y up/-z forward), so leaving out the rotation
    // silently placed such an asset in the wrong orientation entirely (M1
    // visual pass bug: the terrain visual and the real collision agreed on
    // WHERE the ground was but not on which way was up).
    [[nodiscard]] godot::Transform3D get_world_origin_godot_transform() const;

    // Diagnostic-only (M1 visual pass bug 2 report): straight-down ray_cast
    // (IRigidBackend::ray_cast, same pattern as core/src/terrain/
    // tile_manager.cpp's own height probe) against the REAL sim collision at
    // a given RAW ISO x/y - ground truth to compare a visual mesh's Godot-
    // space height against. Returns {"hit": bool, "height_m": float} (ISO z
    // of the hit point, NOT Godot-space - the caller applies its own
    // iso_to_godot conversion so this stays a pure sim-side query).
    [[nodiscard]] godot::Dictionary probe_ground_height(float sim_x, float sim_y) const;

    // --- Body / articulation transforms (race-free: snapshot-backed) ---
    [[nodiscard]] godot::Transform3D get_body_transform(const godot::String& body_name) const;
    [[nodiscard]] godot::Transform3D get_articulation_link_transform(const godot::String& articulation_name, int link_index) const;
    // Body speed in m/s (World's own BodySnapshot::motion, same race-free
    // path as get_body_transform - PLAN.md M1 HUD's "speed (km/h)").
    [[nodiscard]] float get_body_speed_mps(const godot::String& body_name) const;

    // --- Named control channels (PLAN.md P2: "input reaches the sim only
    // as named control channels ... in ISO/SI terms, never as Godot
    // types"). PLAN.md P3/M1: "steer"/"throttle"/"brake"/"handbrake" now
    // feed every vehicle created via create_vehicle (out/handover_p3.md
    // decision 5: global, not per-vehicle - matches M1's one-car scope).
    //
    // Cross-thread note: World::set_control/get_control (ps/world/world.h)
    // are plain unordered_map accesses with NO synchronisation of their
    // own - safe only because vehicle::System::step_all (sim thread) is
    // the ONLY thing that ever calls them, via SimThread's pre_step hook
    // (rebuild_post_step_and_start, ps_simulation.cpp), which copies THIS
    // class's own control_channels_ into World right before each
    // World::step(). control_channels_ here is a std::unordered_map whose
    // VALUES are std::atomic<float> (properly defined concurrent access)
    // and whose KEYS are pre-seeded once, single-threaded, in
    // load_scenario() (before start() ever runs) - std::unordered_map
    // references to already-inserted elements stay valid across further
    // insertions, so set_control() from the Godot main thread only ever
    // stores into an existing atomic<float>, never touching the map's own
    // structure once the sim thread is running.
    void set_control(const godot::String& channel, float value);
    [[nodiscard]] float get_control(const godot::String& channel) const;

    // --- Debug draw data (race-free via AdapterSnapshot, see file comment) --
    [[nodiscard]] godot::TypedArray<godot::Dictionary> get_contact_events() const;

    // --- Static collision import (PLAN.md P2) ---------------------------
    // Scans `scene_root` for StaticBody3D/CollisionShape3D nodes and
    // creates matching static bodies in the sim (collision_import.h).
    std::int64_t import_static_collision(godot::Node* scene_root);
    // PLAN.md M1 step 1: loads the test ground's REAL collision (mesh, per
    // surface group) plus the procedural terrain source and the shared
    // SurfaceTable (collision_import.h's import_test_ground_real) -
    // replaces the old P2 flat-plane placeholder now that core has
    // MeshShape/HeightfieldShape and the loaders it needs. Must be called
    // before start(). Returns false (see get_last_error()) on any load
    // failure. surfaces_path_override: see import_test_ground_real (empty =
    // the shared surfaces.json).
    bool import_test_ground(const godot::String& track_dir_absolute_path, std::int64_t max_resident_tiles = 512, std::int64_t max_tile_fills_per_tick = 16, const godot::String& surfaces_path_override = godot::String());
    [[nodiscard]] const godot::String& get_last_error() const { return last_error_; }
    [[nodiscard]] std::int64_t terrain_resident_tile_count() const { return world_ ? static_cast<std::int64_t>(world_->terrain_resident_tile_count()) : 0; }
    [[nodiscard]] std::int64_t terrain_starved_tile_count() const { return world_ ? static_cast<std::int64_t>(world_->terrain_starved_tile_count()) : 0; }

    // --- Vehicles (PLAN.md M1) -------------------------------------------
    // Generic dynamic-body creation for a vehicle chassis (the scenario
    // schema has no "spawn a lone body at runtime" concept - PsSimulation
    // needs one so main.gd can place the car at a track spawn point that is
    // only known once import_test_ground() has resolved a track dir, not
    // baked into a scenario file). `sim_position`/`yaw_rad` are RAW sim-
    // space (ISO 8855 / test-ground ENU, algebraically identical per
    // data/tracks/README.md's "Frame" section - see that file) numbers
    // carried through a godot::Vector3 purely as a convenient 3-float
    // struct; this is NOT a Godot scene-graph position and does not go
    // through frame_convert.h's basis change - every OTHER method in this
    // adapter that touches a Godot node's transform still goes through
    // that single conversion point. Must be called before start().
    bool create_body_dynamic(const godot::String& name, const godot::Vector3& sim_position, float yaw_rad, float mass, const godot::Vector3& half_extents);
    // Loads vehicle_json_absolute_path (ps::io::load_vehicle_json) and
    // attaches it to the already-created chassis body `chassis_body_name`
    // (see create_body_dynamic above). Registers a terrain interest point
    // for this vehicle (updated every tick from the chassis's own position,
    // see ps_simulation.cpp's post_step lambda) with `stream_radius_m`.
    // Must be called before start().
    bool spawn_vehicle(const godot::String& vehicle_name, const godot::String& vehicle_json_absolute_path, const godot::String& chassis_body_name, float stream_radius_m);

    // ps_audio A11: whether spawn_vehicle built a VehicleVoice for this
    // vehicle (its engine has a "cycle" block - see vehicle_voice.h). false
    // for an unknown vehicle name too, not an error.
    bool has_vehicle_voice(const godot::String& vehicle_name) const;
    bool connect_vehicle_voice_spatial(const godot::String&,const godot::Ref<PsSpatialAudio>&,int latency_ms=12);
    // Sample rate / interleaved channel count of the vehicle's voice (0 if
    // has_vehicle_voice() is false) - a GDScript AudioStreamGenerator must
    // be configured to match sample_rate before this is read from.
    double get_vehicle_voice_sample_rate_hz(const godot::String& vehicle_name) const;
    int get_vehicle_voice_channel_count(const godot::String& vehicle_name) const;
    godot::PackedVector3Array get_vehicle_voice_source_positions(const godot::String& vehicle_name) const;
    // Reads up to `frames` interleaved frames (channel_count() floats each)
    // already rendered by this vehicle's Runner thread into its ring
    // buffer - call every Godot _process() frame, mix channel_count() down
    // to however many an AudioStreamGenerator needs (typically 1: mono
    // sum), and push_buffer() the result. Returns fewer than `frames` (down
    // to 0) whenever the Runner has not rendered enough yet - each such
    // call also increments this vehicle's own underrun counter, read via
    // get_vehicle_voice_underrun_count(). Empty array for an unknown
    // vehicle or one with no voice (not an error - see has_vehicle_voice).
    godot::PackedFloat32Array read_vehicle_voice_audio(const godot::String& vehicle_name, int frames);
    // Total number of read_vehicle_voice_audio() calls, over this voice's
    // whole life, that returned fewer frames than requested - i.e. the
    // Runner thread underran Godot's own draw rate. 0 for a healthy run;
    // -1 for an unknown vehicle or one with no voice.
    std::int64_t get_vehicle_voice_underrun_count(const godot::String& vehicle_name) const;
    // Engine-audio timing diagnostics (main thread): the voice's time-map
    // hitch/arrival counters (cumulative), the largest audio-time gap between
    // two snapshot arrivals and the longest sim step (world step + post_step)
    // since the previous call (both reset by this call), sim catch-up
    // wake-ups (cumulative) and the Runner's worst block. Empty for an
    // unknown vehicle or one without a voice.
    godot::Dictionary take_vehicle_voice_timing(const godot::String& vehicle_name);
    // Render-interpolation diagnostics (render_interp.h PoseHistory): frames
    // sampled/late/early/stepped (cumulative), the last sample's blend, the
    // delay in use, SimThread resyncs and histograms of the sim's publish
    // lateness (wall publish time minus nominal tick time) and of the wall
    // gap between consecutive publishes (0.5 ms bins, last bin = overflow).
    godot::Dictionary get_render_diagnostics() const;
    // Render delay D in ticks (default 2.0, see render_interp.h). Main thread.
    void set_render_delay_ticks(double ticks) { render_delay_ticks_ = ticks; }
    double get_render_delay_ticks() const { return render_delay_ticks_; }
    // A/B diagnostic: true renders with the OLD publish-time alpha
    // (render_interp_alpha) instead of the nominal-clock history.
    void set_render_legacy_alpha(bool legacy) { render_legacy_alpha_ = legacy; }
    bool get_render_legacy_alpha() const { return render_legacy_alpha_; }
    // Listener gain (Pa at 1 m mapped to full scale); > 0, default 15.
    void set_voice_full_scale_pa(double pa);
    double get_voice_full_scale_pa() const;
    // Teleports the vehicle's chassis body back to sim_position/yaw_rad
    // with zero velocity (IRigidBackend::set_pose/set_motion - PLAN.md M1
    // "reset vehicle" control), THEN calls ps::World::reset_vehicle (plan
    // section 4.5/12.1) with hubs_roll_with_chassis=false and
    // clutch_locked=false (the chassis is now at rest, gear kept as-is) -
    // closes the KNOWN LIMITATION this comment used to carry: per-wheel
    // tyre relaxation state and the drivetrain network's own omega are now
    // reset too, not left to relax toward zero over the following second.
    bool request_vehicle_turbo_configuration(const godot::String& vehicle_name,const godot::String& path);
    godot::String get_turbo_configuration_status() const;
    // Baked engine audio bank (40923cf). Off by default since 2026-10-04: its
    // 0.5 s sample-time loops are not crank-synchronous, so loop joins and
    // plane blends produce audible steps every ~0.5 s under throttle. While
    // off, vehicles play the live voice only; turbo configuration swaps still
    // apply their physics. Set before start(); the demo's --m1-audio-bank
    // turns it on.
    void set_engine_audio_bank_enabled(bool enabled);
    bool get_engine_audio_bank_enabled() const;
    godot::String get_engine_audio_bank_status() const;
    bool reset_vehicle(const godot::String& vehicle_name, const godot::Vector3& sim_position, float yaw_rad);
    // PHYS-033 P4 I3 plan section 12.1: gear/gear_target/shift_phase(str)/
    // rpm/engine_state(str)/speedo_kmh/clutch_engagement/limiter/
    // grind_active/starter_active/dfco/assist_auto_clutch/assist_auto_blip/
    // assist_auto_shift, from the race-free per-tick snapshot (same path as
    // the wheel getters below) - empty Dictionary for an unknown vehicle.
    [[nodiscard]] godot::Dictionary get_vehicle_powertrain(const godot::String& vehicle_name) const;
    // Driver HUD tachometer gauge (PLAN.md P4 "Driver HUD"): static, per-
    // vehicle scale data the gauge needs but get_vehicle_powertrain's
    // per-tick PowertrainSnapshot does not carry - idle_rpm/limiter_rpm read
    // off the vehicle's own TorqueMapEngineDesc (whichever "engine" component
    // its powertrain graph has, PHYS-033 I3's ComponentParams variant),
    // gear_count off its GearboxDesc::forward_ratios (the "gearbox"
    // component) - so the same gauge script draws car_sedan.json's 6-speed/
    // 6800 rpm limiter and tractor.json's 12-speed/2400 rpm governor from one
    // formula, never a per-car constant. fitted_assists is a PackedStringArray,
    // empty until a future increment adds a driver-assist system with its own
    // fitted flag (ABS/TC/ESP/LC/HILL do not exist in core yet) - present now
    // so hud/tach_gauge code has one stable place to read it from later
    // without another adapter change. Sourced entirely from the cached
    // VehicleDesc (vehicle_desc_by_name_, already read by
    // get_vehicle_max_wheel_angle et al. above) - no core change. Empty
    // Dictionary for an unknown vehicle.
    [[nodiscard]] godot::Dictionary get_vehicle_gauge_info(const godot::String& vehicle_name) const;

    // Weather plan W2 (out/weather_plan.md 1.3): clock + calendar + sun. The
    // logic lives in core (ps::environment); this adapter only installs it
    // and exposes the per-tick record. install_environment loads the climate
    // file (absolute path), anchors time at `start_clock_hours` (0-24; a game
    // day of any length maps onto 24 h) of day `start_day` of `start_season`
    // (start_day < 0: the climate's own epoch; empty season: the first one) at
    // `time_scale` environment seconds per sim second, and installs it into the
    // World. Setup only: after load_scenario() and BEFORE start() (returns
    // false with last_error set while the sim thread runs, or on any load
    // error). Without it the demo has no environment and every other adapter
    // behaviour is unchanged.
    bool install_environment(const godot::String& climate_absolute_path, double time_scale, const godot::String& start_season,
                             std::int64_t start_day, double start_clock_hours);
    [[nodiscard]] bool has_environment() const;
    // Race-free (render snapshot). Empty Dictionary without an environment.
    // Keys: t_env_s, time_scale, year, season (index), season_name, day_in_season,
    // day_of_year, time_of_day_s, day_s, night_s, day_fraction, clock_hours,
    // is_day, sun_elevation_deg, sun_azimuth_deg, sun_direction (Godot-frame
    // unit Vector3 TOWARDS the sun), ghi_w_m2, cloud_cover.
    [[nodiscard]] godot::Dictionary get_environment() const;

    [[nodiscard]] godot::PackedStringArray get_vehicle_names() const;
    [[nodiscard]] godot::String get_vehicle_chassis_body_name(const godot::String& vehicle_name) const;
    [[nodiscard]] std::int64_t get_vehicle_wheel_count(const godot::String& vehicle_name) const;
    [[nodiscard]] godot::String get_wheel_name(const godot::String& vehicle_name, int wheel_index) const;
    // Static per-wheel geometry (cached VehicleDesc, see file comment) -
    // Vector3/float fields are RAW sim-space numbers, same convention as
    // create_body_dynamic above (chassis-local, not Godot-space).
    [[nodiscard]] godot::Vector3 get_wheel_attachment_local(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] godot::Vector3 get_wheel_suspension_axis(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_radius(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_width(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_travel_max(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] bool get_wheel_steered(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] bool get_wheel_driven(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] bool get_wheel_is_front(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_vehicle_max_wheel_angle(const godot::String& vehicle_name) const;
    [[nodiscard]] float get_vehicle_rear_steer_fraction(const godot::String& vehicle_name) const;

    // Gamepad rumble (haptics_core.h): chassis proper (gravity-removed)
    // linear acceleration, CHASSIS frame, race-free via AdapterSnapshot -
    // see VehicleDynamicSnapshot::accel_local's own comment for exactly how
    // this is derived. Zero for an unknown vehicle or before the second
    // tick (no previous velocity sample yet).
    [[nodiscard]] godot::Vector3 get_vehicle_chassis_accel_local(const godot::String& vehicle_name) const;

    // Dynamic per-wheel telemetry (race-free via AdapterSnapshot).
    [[nodiscard]] float get_wheel_compression(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_vehicle_ground_speed_mps(const godot::String& vehicle_name) const;
    // x = wheelbase (m, mean front-axle x minus mean rear-axle x from the
    // wheel attachments), y = SteeringDesc::max_wheel_angle (rad); zero
    // vector if unknown. For the input mapper's steering assist.
    [[nodiscard]] godot::Vector2 get_vehicle_steer_geometry(const godot::String& vehicle_name) const;
    [[nodiscard]] float get_wheel_omega(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_steer_angle(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_slip_ratio(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_slip_angle(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_load_n(const godot::String& vehicle_name, int wheel_index) const; // fz
    [[nodiscard]] float get_wheel_fx(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] float get_wheel_fy(const godot::String& vehicle_name, int wheel_index) const;
    [[nodiscard]] godot::String get_wheel_surface_name(const godot::String& vehicle_name, int wheel_index) const;

    // --- Adapter main-thread frame-time accounting (frame_timing.h) -----
    void add_adapter_time_us(std::int64_t us) { FrameTiming::add_us(us); }
    [[nodiscard]] std::int64_t consume_adapter_frame_time_us() { return FrameTiming::consume_us(); }

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
    [[nodiscard]] const ps::vehicle::WheelDesc* wheel_desc(const std::string& vehicle_name, int wheel_index) const;
    [[nodiscard]] const WheelDynamicSnapshot* dynamic_wheel(const std::string& vehicle_name, int wheel_index) const;
    void rebuild_post_step_and_start(); // start()'s helper: finalises vehicle/articulation ordering, installs pre/post_step, starts SimThread

    // Render-time interpolation (render_interp.h, PLAN.md D3 judder fix).
    // Refreshes the frame-local cache below at most once per Godot frame
    // (keyed by Engine::get_singleton()->get_process_frames(), not by
    // "whichever node's _process happens to run first this frame" - the
    // camera (chase_cam.gd, process_priority -1000), PsBodyNode3D per body/
    // link, and any future reader each have their own place in the scene
    // tree's process order, and PLAN.md gives none of them priority over
    // the others) so every get_body_transform/get_articulation_link_
    // transform call within the SAME frame reads the SAME published
    // (prev, latest) snapshot at the SAME alpha - the task's own "camera,
    // car and wheels use the same interpolation time" requirement. Callable
    // from any const method; safe to call redundantly (idempotent within a
    // frame).
    void ensure_render_frame_cached() const;
    mutable std::uint64_t cached_render_frame_ = ~static_cast<std::uint64_t>(0); // sentinel: no frame cached yet
    mutable const AdapterSnapshot* cached_render_snapshot_ = nullptr;
    mutable double cached_render_alpha_ = 1.0;
    // Nominal-clock path (render_interp.h PoseHistory): every body, then per
    // articulation its root and links, flattened in this order into one pose
    // vector per published tick. render_sample_ is the frame's interpolated
    // result (getters read it); the legacy A/B path fills it from the
    // snapshot's (prev, latest) pairs and publish_time alpha instead.
    PoseHistory pose_history_;
    mutable RenderSample render_sample_;
    mutable std::int64_t last_sample_time_us_ = 0; // steady_clock time of the frame's sample (diagnostics)
    std::vector<std::size_t> articulation_pose_offset_; // per articulation_order_ index: index of its root in the flat layout
    std::vector<ps::Pose> history_pose_scratch_;        // sim thread only
    std::vector<std::uint8_t> history_teleport_scratch_; // sim thread only
    double render_delay_ticks_ = 2.0;
    bool render_legacy_alpha_ = false;
    TimingHistogram<32> publish_lateness_hist_{500000};
    TimingHistogram<32> publish_gap_hist_{500000};
    std::int64_t last_publish_wall_ns_ = 0; // sim thread only
    // The ONLY way main-thread code reads adapter_snapshots_: every getter
    // must go through the same once-per-frame sample. A second read() in
    // the same frame could swap the triple buffer's read slot and hand the
    // slot cached_render_snapshot_ still points at back to the sim thread's
    // writer (a torn read), besides mixing two ticks within one frame.
    const AdapterSnapshot& render_snapshot() const {
        ensure_render_frame_cached();
        return *cached_render_snapshot_;
    }

    // PLAN.md M1 "reset vehicle" control - see reset_vehicle()'s doc
    // comment: applied on the sim thread (SimThread::set_pre_step) rather
    // than directly from here, so a live reset while driving never races
    // IRigidBackend::step(). Mutex-guarded because it is a rare, human-
    // triggered event (a button press), not a hot path - unlike
    // control_channels_ below, which IS touched every tick and therefore
    // uses std::atomic instead.
    struct PendingVehicleReset {
        std::string vehicle_name;
        ps::Pose pose;
    };
    struct PendingTurboUpdate { std::string vehicle_name; ps::drivetrain::TurboPairDesc desc; };
    std::thread turbo_generation_thread_;
    struct BankRequest { std::string vehicle_name, engine_path, turbo_path, cache_dir; bool apply_physics=false; };
    std::unordered_map<std::string,std::string> engine_definition_paths_;
    std::unordered_map<std::string,std::string> requested_turbo_paths_;
    std::vector<BankRequest> bank_requests_;
    std::atomic<bool> engine_audio_bank_enabled_{false};
    std::condition_variable bank_request_cv_;
    bool bank_worker_stop_=false;
    std::string engine_audio_bank_status_="Live acoustic model";
    void enqueue_bank_request(BankRequest request);

    mutable std::mutex turbo_update_mutex_;
    std::vector<PendingTurboUpdate> pending_turbo_updates_;
    std::string turbo_configuration_status_="Default configuration";
    std::mutex pending_reset_mutex_;
    std::vector<PendingVehicleReset> pending_vehicle_resets_;

    std::unique_ptr<ps::World> world_;
    std::unordered_map<std::string, ps::BodyId> bodies_by_name_;
    std::unordered_map<std::string, ps::ArticulationId> articulations_by_name_;
    std::vector<std::string> articulation_order_; // registration index -> name, matches AdapterSnapshot::articulations
    std::unordered_map<std::string, std::size_t> articulation_link_counts_;

    // Render-time interpolation (render_interp.h): bodies_by_name_'s own
    // stable per-tick order, matching AdapterSnapshot::bodies - built once
    // in rebuild_post_step_and_start() (same "finalised when start() runs"
    // rule as vehicle_order_/articulation_order_ above; bodies_by_name_
    // itself is only ever populated before start(), by create_body_dynamic).
    std::vector<std::string> body_order_;
    std::unordered_map<std::string, std::size_t> body_order_index_; // name -> body_order_ index, mirrors index_in_order() for O(1) lookup on the hot get_body_transform path

    // Sim-thread-only (post_step/pre_step): the previous tick's pose per
    // body_order_ index, and per articulation per articulation_order_
    // index - what post_step blends this tick's freshly-read pose against
    // to build AdapterSnapshot's InterpolatedPose fields. Both are (re)sized
    // and seeded with each tracked body/link's CURRENT pose in
    // rebuild_post_step_and_start() (single-threaded, before the sim thread
    // starts), so the very first published tick already has a real (if
    // trivial - prev == latest, see render_interp.h's InterpolatedPose
    // comment) previous pose rather than needing a separate "first tick"
    // flag.
    std::vector<ps::Pose> prev_body_pose_;
    // Set (by index into body_order_) in the pre_step reset-vehicle handler
    // for the tick a teleport lands on, consumed and cleared again in that
    // same tick's post_step - see reset_vehicle()'s doc comment and
    // render_interp.h's header comment on why a teleport forces prev :=
    // latest instead of blending through it.
    std::vector<bool> body_teleported_this_tick_;
    struct ArticulationPrevPose {
        ps::Pose root;
        std::vector<ps::Pose> links; // index == link_index, sized to articulation_link_counts_ at rebuild_post_step_and_start() time
    };
    std::vector<ArticulationPrevPose> prev_articulation_poses_; // index == articulation_order_'s index

    // PLAN.md M1: vehicles (see spawn_vehicle above). vehicle_order_ is
    // finalised (captured into the post_step lambda) when start() runs -
    // spawn_vehicle may only be called before that point, same rule as
    // World::create_body/create_vehicle's own "setup time" contract.
    std::unordered_map<std::string, ps::VehicleId> vehicles_by_name_;
    std::unordered_map<std::string, std::string> vehicle_chassis_body_name_;
    std::unordered_map<std::string, ps::vehicle::VehicleDesc> vehicle_desc_by_name_;
    std::vector<std::string> vehicle_order_;
    std::unordered_map<std::string, std::uint64_t> vehicle_terrain_interest_id_;
    std::unordered_map<std::string, float> vehicle_stream_radius_m_;
    std::uint64_t next_terrain_interest_id_ = 1;

    // ps_audio A11: one VehicleVoice per vehicle whose engine has a "cycle"
    // block (see vehicle_voice.h) - built once in spawn_vehicle from that
    // vehicle's OWN already-loaded VehicleDesc, started/stopped alongside
    // the sim thread (start()/stop()), fed every tick from post_step (sim
    // thread only, same race-free point PowertrainSnapshot itself is read
    // at - see rebuild_post_step_and_start()). A vehicle with no cycle
    // block simply has no entry here (get_vehicle_voice_underrun_count
    // below returns -1 for it) - never a placeholder silent voice.
    std::unordered_map<std::string, std::unique_ptr<VehicleVoice>> vehicle_voice_by_name_;
    // Main-thread only (read_vehicle_voice_audio is called from Godot's
    // _process, never the sim thread) - a plain map is fine, no atomics.
    std::unordered_map<std::string, std::int64_t> vehicle_voice_underrun_count_;
    // Listener gain: read_vehicle_voice_audio divides the voice's Pa at 1 m
    // by this, like tools/ps_engine_audio's --full-scale-pa. 15 Pa is the
    // level of the owner's 'loud' listening renders (the calibrated 200 Pa
    // default of ps_audio::Calibration sits near -40 dBFS - too quiet in-game).
    double voice_full_scale_pa_ = 15.0;

    // Gamepad rumble (haptics_core.h): one previous world-frame chassis
    // linear-velocity sample per vehicle, for the finite-difference
    // acceleration computed once per tick in post_step (see
    // VehicleDynamicSnapshot::accel_local's comment) - `valid` is false
    // until the second sample (or right after a reset_vehicle teleport,
    // which would otherwise read as a huge spurious accel spike).
    struct ChassisAccelState {
        ps::Vec3 prev_velocity_world = ps::Vec3::zero();
        bool valid = false;
    };
    std::unordered_map<std::string, ChassisAccelState> chassis_accel_state_;
    // Cached from the loaded scenario's own WorldConfig::gravity
    // (ps::io::Scenario::world_config, already a public field - no core
    // change needed) so accel_local above can remove EXACTLY the gravity
    // this World was actually built with, not an assumed constant.
    ps::Vec3 gravity_ = ps::Vec3{0.0, 0.0, -9.81};

    std::shared_ptr<const ps::io::SurfaceTable> surface_table_; // set by import_test_ground; null == no surface names available
    // Fall diagnostic (fall_detector.h): the installed terrain source (set
    // by import_test_ground) and one detector per vehicle, created in
    // start() and touched only by the sim thread's post_step afterwards.
    std::shared_ptr<const ps::terrain::ITerrainSource> terrain_source_;
    std::unordered_map<std::string, FallDetector> fall_detectors_;

    std::unique_ptr<SimThread> sim_thread_;
    OriginRebase origin_rebase_;
    TripleBuffer<AdapterSnapshot> adapter_snapshots_;

    std::unordered_map<std::string, std::atomic<float>> control_channels_; // see set_control's doc comment above
    godot::String last_error_;
};

} // namespace ps_godot
