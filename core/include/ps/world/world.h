// ps/world/world.h — World (PLAN.md section 5).
//
// Owns bodies/joints (via IRigidBackend), force elements, the job system,
// snapshot buffer and telemetry registry, and drives the staged tick
// pipeline. No Jolt or Godot type appears here (PLAN.md D1/D2).

#pragma once

#include "ps/articulation/articulation_desc.h"
#include "ps/backend/i_rigid_backend.h"
#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/engine_maps.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/drivetrain/turbo_envelope.h"
#include "ps/io/telemetry.h"
#include "ps/jobs/job_system.h"
#include "ps/jobs/task_graph.h"
#include "ps/io/surface_table.h"
#include "ps/machine/coupling.h"
#include "ps/machine/soil_source.h"
#include "ps/machine/tool.h"
#include "ps/machine/work.h"
#include "ps/machine/hitch_limits.h"
#include "ps/terrain/terrain_config.h"
#include "ps/terrain/terrain_source.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/vehicle/wheel_state.h"
#include "ps/vehicle/wheel_telemetry.h"
#include "ps/world/ambient.h"
#include "ps/world/force_element.h"
#include "ps/world/ids.h"
#include "ps/world/snapshot.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

// Forward-declared only: the articulation runtime (PLAN.md D10/P1.5a) is
// internal (core/src/articulation/system.h), never included from a public
// header - World owns it through this incomplete-type pointer exactly the
// way it owns IRigidBackend's Jolt implementation (PLAN.md D2's seam).
namespace ps::articulation { class System; class ContactCoupling; } // ContactCoupling: PLAN.md P1.5b, core/src/articulation/contact.h
// Forward-declared only: PLAN.md M1's tile system (core/src/terrain/tile_manager.h)
// is internal for the same reason - World owns it via this incomplete-type
// pointer, same pattern as articulation::System above.
namespace ps::terrain { class TileManager; }
// Forward-declared only: the vehicle runtime (PLAN.md 5.1/P3,
// core/src/vehicle/system.h) is internal for the same reason - World owns
// it via this incomplete-type pointer, same pattern as articulation::System.
namespace ps::vehicle { class System; }
// Forward-declared only (weather plan W2): the environment library
// (core/include/ps/environment, target ps_environment) is held by shared_ptr.
namespace ps::environment { class Environment; struct EnvironmentSnapshot; struct WeatherPresetDesc; }

namespace ps {

struct WorldConfig {
    real tick_rate_hz = 240.0;      // PLAN.md D3: rigid-body tick.
    real substep_rate_hz = 960.0;   // documented stub until P3 (vehicles).
    Vec3 gravity = Vec3{0.0, 0.0, -9.81};
    // kg/m^3, ISA sea level at 15 degC. Unused until commit 6c wires aero
    // drag into the vehicle system (PHYS-033 P4 I3 plan section 4.5);
    // Scenario JSON reads it as an optional "air_density" key
    // (ps::io::load_scenario, scenario.cpp).
    real air_density = 1.225;
    Vec3 wind_world{};
    int job_workers = 0;            // 0 = auto (physical cores - 2, PLAN.md D8)
    // 0 = auto (ps::jobs::JobSystem::kDefaultQueueCapacity, 4096 - every real
    // workload in this codebase). A test may set a small explicit value to
    // force the queue-full inline-execution fallback deliberately (P1.5b
    // review item; see ps::jobs::JobSystem::queue_full_fallback_count).
    std::size_t job_queue_capacity = 0;
    std::size_t max_bodies = 4096;
    // Chunk size for the force-element parallel_for stage; 0 = auto
    // (spreads elements evenly over the worker count).
    std::size_t force_element_grain = 0;
    // Chunk size for the vehicle-island (articulation) parallel_for stage
    // (PLAN.md D8/P1.5a, PHYS-009 Step 1); 0 = auto, same formula as
    // force_element_grain.
    std::size_t articulation_grain = 0;

    // Jolt solver settings, exposed for the Part A energy-loss study
    // (PLAN.md). Defaults equal Jolt's own defaults (see
    // ps::RigidBackendSettings, i_rigid_backend.h, for the exact
    // PhysicsSettings field each one maps to) - unless a caller sets one of
    // these, behaviour is unchanged from before this field existed.
    real baumgarte = 0.2;
    unsigned num_velocity_steps = 10;
    unsigned num_position_steps = 2;
    int collision_steps = 1;
    // Sweeps over a chain's couplings per substep (Gauss-Seidel, one exact solve per coupling). A single coupling
    // takes one whatever this says. Tractor plus two trailers: 2 and 8 agree to 3 decimals in mm, 1 leaves the first
    // link's error at half its value because that link never sees the second one's force in the same substep.
    int coupling_passes = 2;
    real penetration_slop = 0.02;
    real speculative_contact_distance = 0.02;
    real max_penetration_distance = 0.2;
    real linear_cast_threshold = 0.75;
    real linear_cast_max_penetration = 0.25;
};

class World {
public:
    explicit World(const WorldConfig& config = {});
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // Creating/destroying a Static body after set_terrain_source also drops
    // the terrain tile system's cached suppression results (see
    // TerrainConfig::precompute_suppression_at_load).
    BodyId create_body(const BodyDesc& desc);
    void destroy_body(BodyId id);

    // Prebuilt shapes (racing_game R2; contract in ps/backend/shape_handle.h).
    // create_shape: ANY thread, also while step() runs on the sim thread and
    // concurrently with other create_shape calls (the World must outlive the
    // call; the handle may outlive the World). Allocates - call it from your
    // own threads, not from a ps::jobs job body. Throws std::invalid_argument
    // for a HeightfieldShape, std::runtime_error for a mesh Jolt rejects.
    [[nodiscard]] ShapeHandle create_shape(const ShapeDesc& desc) const { return backend_->create_shape(desc); }
    // Sim thread between steps, like create_body(desc): the same body, BodyId
    // sequence and state_hash as create_body(desc with desc.shape = the
    // handle's ShapeDesc); desc.shape is ignored. O(1) in the shape's size.
    // A Static body drops the terrain suppression cache exactly like the
    // desc path. The body keeps the shape alive until destroy_body.
    BodyId create_body(const BodyDesc& desc, const ShapeHandle& shape);
    JointId create_joint(const JointDesc& desc) { return backend_->create_joint(desc); }
    void destroy_joint(JointId id) { backend_->destroy_joint(id); }

    [[nodiscard]] Pose get_pose(BodyId id) const { return backend_->get_pose(id); }
    [[nodiscard]] Motion get_motion(BodyId id) const { return backend_->get_motion(id); }
    [[nodiscard]] real get_mass(BodyId id) const { return backend_->get_mass(id); }

    // PLAN.md D10/P1.5a: our own reduced-coordinate (Featherstone) solver
    // for articulated chains. Runs in the vehicle-island stage,
    // sub-stepped at WorldConfig::substep_rate_hz inside each tick (D3/D8);
    // see PLAN.md section 5 and P1.5a's scope for the model. Additive to
    // every existing World method; a World that never calls this behaves
    // exactly as before.
    ArticulationId create_articulation(const articulation::ArticulationDesc& desc);
    [[nodiscard]] real joint_position(ArticulationId id, int link_index) const;
    [[nodiscard]] real joint_velocity(ArticulationId id, int link_index) const;
    // Direct joint force/torque actuation (PLAN.md P1.5a scope item 2);
    // persists until changed, applied every substep.
    void set_joint_force(ArticulationId id, int link_index, real tau);
    // A Frame anchored to link_index (-1 == the articulation's own root -
    // meaningful only for a floating root, PLAN.md P1.5a scope item 2).
    // Force elements attach to this exactly like to a body Frame; their
    // wrenches reach the articulation as external spatial forces.
    [[nodiscard]] Frame link_frame(ArticulationId id, int link_index, Pose local_offset = Pose::identity()) const;
    [[nodiscard]] Pose root_pose(ArticulationId id) const { return link_frame(id, -1).world_pose(*backend_); }
    [[nodiscard]] Motion root_motion(ArticulationId id) const { return link_frame(id, -1).world_motion(*backend_); }

    // PLAN.md M1/section 6: the terrain-tile system. Installing a source
    // (re)creates the tile pool per `config`; may only be called once per
    // World (not a hot-path operation - setup time, like create_body).
    // Interest points (one per vehicle/camera) drive which tiles stream in;
    // radius_m is in metres. Wired into the "terrain_tiles" pipeline stage
    // (World::stage_terrain_tiles) - a no-op tick if no source was set,
    // exactly as before this feature existed.
    void set_terrain_source(std::shared_ptr<const terrain::ITerrainSource> source, const terrain::TerrainConfig& config = {});
    void set_terrain_interest_point(std::uint64_t id, const Vec3& world_xy, real stream_radius_m);
    void remove_terrain_interest_point(std::uint64_t id);
    [[nodiscard]] std::size_t terrain_resident_tile_count() const;
    [[nodiscard]] std::size_t terrain_starved_tile_count() const;
    // Tiles whose static-geometry suppression was precomputed inside
    // set_terrain_source (TerrainConfig::precompute_suppression_at_load).
    [[nodiscard]] std::size_t terrain_precomputed_tile_count() const;
    // R1 (real-world terrain, 90-880 m ASL): how many newly-filled tiles had
    // local relief exceeding TileManager's per-tile datum window (see
    // ps/terrain/terrain_source.h's TileSample doc comment) - such a tile is
    // still uploaded, clamped, not dropped, so this is a "notice", not an
    // error return.
    [[nodiscard]] std::size_t terrain_relief_overflow_count() const;
    // Terrain prefetch ring counters (TerrainConfig::prefetch_margin_tiles/
    // prefetch_max_tiles; all 0 with prefetch off or no source).
    [[nodiscard]] terrain::PrefetchStats terrain_prefetch_stats() const;

    // PLAN.md P2 adapter report / this task's step 3: named control
    // channels (e.g. "steer" -1..1, "throttle" 0..1), SI/ISO, so input can
    // reach vehicle code without any Godot (or other front-end) type
    // crossing into the core. Each vehicle has its own namespace: create_vehicle's
    // channel_scope names it ("trailer_1.brake"), and without a scope vehicle index
    // 0 reads the bare names ("steer") and vehicle i "v<i>." ones (PLAN.md P4 I0).
    // vehicle_channel gives the full name, so a caller with several vehicles need
    // not know the order they were created in. Storage is index-based (see
    // ensure_control_channel_index below): resolved once at create_vehicle, no
    // string hashing in the tick.
    void set_control(const std::string& channel, real value) {
        control_channel_values_[ensure_control_channel_index(channel)] = value;
    }
    [[nodiscard]] real get_control(const std::string& channel) const {
        const auto it = control_channel_index_.find(channel);
        return it == control_channel_index_.end() ? real(0.0) : control_channel_values_[it->second];
    }
    // PHYS-033 P4 I4 C3: every control channel name World currently knows,
    // in unspecified (hash-map) enumeration order - introspection/testing
    // only (e.g. "did create_vehicle add exactly the channels I expect",
    // compared as a set). Never call this to decide anything about
    // simulation state or order (control_channel_index_'s own "never
    // iterated" comment, PLAN.md D7, still governs the hot path).
    [[nodiscard]] std::vector<std::string> control_channel_names() const;

    // PHYS-033 P4 I7b C9 (ps/world/ambient.h): the ambient atmospheric
    // sample every vehicle's engine Sensors sees. Sim-thread-only, like
    // set_terrain_source/set_surface_table - call between steps; takes
    // effect at the next tick's vehicle::VehicleTickEnv (World::step reads
    // ambient_ once per tick, not mid-step). Never hashed (an ECU/
    // environment input, like a control channel, not simulation state).
    // Aero setters and live telemetry are stepping-thread-only, called BETWEEN ticks.
    // Renderers should read VehicleSnapshot::aero instead of the live accessor.
    void set_aero_environment(const aero::Environment& environment);
    [[nodiscard]] aero::Environment aero_environment() const { return {config_.air_density,config_.wind_world}; }
    void set_aero_wakes(std::vector<aero::WakeSource> wakes); // optional non-vehicle sources (e.g. trucks)
    void set_aero_surface_command(VehicleId, std::size_t surface, real command);
    void set_aero_fan_command(VehicleId, std::size_t fan, real command);
    // Caller MUST debit this credit from its electrical/engine energy store. No automatic coupling.
    // Reservoir starts empty; motors never generate unpaid suction. Tick telemetry reports consumption.
    void credit_aero_fan_energy(VehicleId, real joules);
    [[nodiscard]] const aero::Telemetry& vehicle_aero_telemetry(VehicleId) const;
    // Weather plan W4 (owner question Q12, PROVISIONAL): while an environment
    // that provides the atmosphere (EnvironmentConfig::weather != Off) is
    // installed it owns the ambient sample, the air density and the wind of every
    // vehicle, so set_ambient and set_aero_environment throw std::logic_error
    // instead of being silently overwritten next tick; after remove_environment()
    // (or with an environment that has no weather) they behave as before.
    void set_ambient(const AmbientSample& a);
    [[nodiscard]] const AmbientSample& ambient() const { return ambient_; }
    // W4: the air the environment computed for a vehicle this tick (ps/environment/
    // vehicle_air.h; valid == false without an atmosphere-providing environment).
    // Sim thread only; renderers read VehicleSnapshot::air.
    [[nodiscard]] environment::VehicleAir vehicle_air(VehicleId id) const;
    // W4 diagnostics (tests): the simulated engine's full steady-grid rebuild count
    // (stays 0 under live ambient changes) and the ambient sample its Sensors saw in
    // the last substep.
    [[nodiscard]] std::uint64_t vehicle_engine_steady_rebuild_count(VehicleId id) const;
    [[nodiscard]] AmbientSample vehicle_engine_ambient(VehicleId id) const;

    // Weather plan W2 (out/weather_plan.md 1.3): the environment (clock,
    // calendar, sun; later weather and surface condition). Sim thread, between
    // steps. install_environment anchors the environment's clock at the
    // CURRENT tick (a mid-run install is a "weather starts now" event) and
    // evaluates its snapshot for that tick; from then on the work runs inside
    // the existing freeze_tick_start_state stage (no new stage, no new
    // telemetry channel), the environment's clock anchor is folded into
    // state_hash() and WorldSnapshot::environment is filled. With no
    // environment installed nothing here runs and every hash, CSV header and
    // allocation count is exactly as before. Throws std::invalid_argument for
    // a null pointer; replaces a previously installed environment.
    void install_environment(std::shared_ptr<environment::Environment> env);
    void remove_environment();
    // Null when none is installed. The pointee is shared with the caller: only
    // call mutating methods on it from the sim thread between steps (prefer
    // set_environment_time/_scale below, which pass the right tick).
    [[nodiscard]] environment::Environment* environment() const { return environment_.get(); }
    // This tick's evaluated snapshot (the one stage_snapshot publishes); null
    // when none is installed. Sim thread only (live state).
    [[nodiscard]] const environment::EnvironmentSnapshot* environment_snapshot() const;
    // Jump env time to `env_s` at the next step / change the scale without a
    // jump (re-anchored at the current tick). Throw std::logic_error when no
    // environment is installed, std::invalid_argument for a bad value.
    void set_environment_time(real env_s);
    void set_environment_time_scale(real scale);

    // Weather plan W3b (owner answer Q12), sim thread between steps, each effective at the next
    // step; all throw std::logic_error without an environment (and, for the weather ones, without a
    // weather timeline). The owner's "manual weather": the preset replaces the weather from now to the
    // END OF THE CURRENT SEASON (ramp-in/-out over ramp_s, the ramp-out finishing at the boundary) and
    // the next season starts on the natural weather; with the season locked it lasts until ended or
    // removed. Returns the override id. Reaches the vehicles through the ordinary per-vehicle air path.
    int add_weather_override_to_season_end(const environment::WeatherPresetDesc& preset, real ramp_s);
    // Soft end: every override ramps out from now over its own ramp_s.
    void end_weather_overrides();
    // Immediate end (an exact reset to the natural weather).
    bool remove_weather_override(int id);
    void clear_weather_overrides();
    // Season lock: fixed_season = a season name locks it, fixed_day_of_year >= 0 pins one game day,
    // empty / -1 = Dynamic. Re-anchors the clock keeping the game day and the fraction of the day
    // (Environment::set_calendar_lock). std::invalid_argument for an unknown season.
    void set_environment_calendar_lock(const std::string& fixed_season, int fixed_day_of_year = -1);

    // PLAN.md section 6 / P3: shared surface-parameter lookup (lambda_mu,
    // crr) for every vehicle's tyre model. Setup-time only (like
    // set_terrain_source); must be called before the first step() if any
    // vehicle should see non-default (1.0/0.0) surface values.
    void set_surface_table(std::shared_ptr<const io::SurfaceTable> table) { surface_table_ = std::move(table); }
    // The installed table (null = none); weather plan W5 maps surface condition classes onto it.
    [[nodiscard]] const io::SurfaceTable* surface_table() const { return surface_table_.get(); }

    // PLAN.md 5.1/P3: per-corner suspension + tyre + steering + brakes + a
    // placeholder drive, on an existing chassis body (created separately,
    // like an articulation's own body creation). Stepped in the
    // "vehicle_islands_substep" stage, right after articulation stepping
    // (out/handover_p3.md decision 3) - one job per vehicle.
    // `test_only_extra_channels` (PHYS-033 P4 I4 C3, plan section 2.4):
    // {name, default_value} pairs merged into this vehicle's own
    // drivetrain::Powertrain::declare_control_channels() call (after its
    // PowertrainDesc's own declarations, duplicates merged the same way) -
    // the only way to reach a non-empty ControlChannelIndices::component
    // through the real create_vehicle path before a real declaring
    // component (C5's diff_lock) exists, so a unit test can exercise
    // declare -> ensure_control_channel_index -> VehicleInstance::
    // component_channel_values_ -> ComponentSubstepCtx::channels end to
    // end. Plain {string, real} pairs, not drivetrain::ChannelDecl
    // (internal, core/src/drivetrain/powertrain.h - not in any public
    // header). Always empty for every production caller.
    // `channel_scope`: this vehicle's channels are "<scope>.<name>"; a stable name for a vehicle whose creation order
    // is not. No dots, and two vehicles cannot share one (std::invalid_argument).
    VehicleId create_vehicle(const vehicle::VehicleDesc& desc, BodyId chassis,
                              const std::vector<std::pair<std::string, real>>& test_only_extra_channels = {},
                              std::string_view channel_scope = {});
    // The full name of this vehicle's channel `name`.
    [[nodiscard]] std::string vehicle_channel(VehicleId id, std::string_view name) const;
    // A hitch between two vehicles (ps/machine/coupling.h). attach, detach and destroy_vehicle run on the sim thread
    // between steps and rebuild the islands there: the coupled vehicles then step together in one job. From inside a
    // step they throw std::logic_error. attach measures the sockets as they are and ramps them together over the
    // class's latch time, without a jump in force; check_attach says whether it would, and what stands in the way, so
    // a caller can ask without catching. A refused attach throws std::invalid_argument with the same message.
    [[nodiscard]] machine::AttachCheck check_attach(const machine::CouplingDesc& desc) const;
    HitchId attach(const machine::CouplingDesc& desc);
    // The same by the names in the vehicles' hitch_points lists: one towing point, one towed point of the same class,
    // the towing point's class file decides the hitch. A point takes one hitch at a time.
    [[nodiscard]] machine::AttachCheck check_attach(VehicleId towing, std::string_view towing_point, VehicleId towed, std::string_view towed_point) const;
    HitchId attach(VehicleId towing, std::string_view towing_point, VehicleId towed, std::string_view towed_point);
    // What a live hitch carries, and the findings against its towing point's ratings (none on a hitch built by hand).
    // Tongue load comes from the last tick; std::invalid_argument for a dead hitch.
    [[nodiscard]] machine::HitchLoads hitch_loads(HitchId id) const;
    [[nodiscard]] std::vector<drivetrain::LimitFinding> hitch_limits(HitchId id, const drivetrain::LimitPolicy* policy = nullptr) const;
    void detach(HitchId id);
    [[nodiscard]] bool hitch_alive(HitchId id) const;
    [[nodiscard]] const machine::CouplingState& coupling_state(HitchId id) const;
    // A linkage's lift reads the towing vehicle's channels (LinkageDesc): attach sets the lever to where the implement
    // hangs and the mode to Position, so that nothing moves until someone asks.
    // The PTO shaft across a live hitch. On it, the implement's rotor is on the tractor's pto.out: engaging is the
    // tractor's "pto" channel. The shaft comes off with the hitch. Sim thread, between steps.
    [[nodiscard]] machine::PtoCheck check_pto(HitchId id) const;
    void connect_pto(HitchId id); // std::invalid_argument with check_pto's message
    void disconnect_pto(HitchId id);
    [[nodiscard]] bool pto_connected(HitchId id) const;
    // What the tools work in. Read from the vehicle jobs, concurrently; replaced only between steps. nullptr: no tool
    // has soil under it. The source stays alive through this pointer.
    void set_soil_source(std::shared_ptr<const machine::ISoilSource> soil);
    [[nodiscard]] std::size_t vehicle_tool_count(VehicleId id) const;
    [[nodiscard]] const machine::ToolDesc& tool_desc(VehicleId id, std::size_t tool) const;
    [[nodiscard]] const machine::ToolState& tool_state(VehicleId id, std::size_t tool) const;
    // A roller or gauge wheel's depth setting, metres.
    void set_tool_gauge_depth(VehicleId id, std::size_t tool, real depth_m);
    // Switches an applicator tool (ToolKind::Applicator) on or off; between steps only, std::invalid_argument for a soil tool.
    void set_tool_enabled(VehicleId id, std::size_t tool, bool on);
    // Appends the quads the tools swept since the last drain, in (tick, vehicle, tool) order. Between steps; machine::
    // WorkLedger turns them into cells. Each tool holds 4096 quads; a fuller buffer drops the newest and counts them.
    void drain_work(std::vector<machine::WorkQuad>& out);
    [[nodiscard]] std::uint64_t work_dropped() const;
    // Takes a vehicle out of the simulation, detaching its hitches. Its chassis stays a plain body, for the caller to
    // destroy or reuse; every other vehicle keeps its index and channels. Whatever checks a VehicleId refuses the stale
    // one, and WorldSnapshot::vehicles keeps its slot with an invalid id and no wheels.
    void destroy_vehicle(VehicleId id);
    [[nodiscard]] std::size_t vehicle_wheel_count(VehicleId id) const;
    [[nodiscard]] const vehicle::WheelTelemetry& wheel_telemetry(VehicleId id, std::size_t wheel_index) const;
    // Assists I2a (diagnostics, same live-state contract as wheel_telemetry): each substep's wheel telemetry of the
    // last tick, substep < vehicle_substep_telemetry_count (at most 16). Never hashed.
    [[nodiscard]] std::size_t vehicle_substep_telemetry_count(VehicleId id) const;
    [[nodiscard]] const vehicle::WheelTelemetry& wheel_substep_telemetry(VehicleId id, std::size_t substep, std::size_t wheel_index) const;
    [[nodiscard]] const std::string& wheel_name(VehicleId id, std::size_t wheel_index) const;
    // PLAN.md M1, task 2 of the P3 fix-up phase: steer angle, suspension
    // travel, integrated spin angle, load and slip - everything a render/
    // adapter thread needs to place and orient a wheel mesh (wheel_state.h).
    // Same live-state-only caveat as wheel_telemetry above - a reader on
    // another thread wants WorldSnapshot::vehicles instead (snapshot.h).
    [[nodiscard]] vehicle::WheelState wheel_state(VehicleId id, std::size_t wheel_index) const;
    // PHYS-033 P4 I2 commit 5a: WheelDesc::driven is gone (which wheels are
    // driven is now the /2 powertrain graph's own shape) - this replaces
    // it for callers that only need the bool (e.g. the Godot adapter's
    // get_wheel_driven, adapters/godot/src/ps_simulation.cpp). Sim-thread-
    // only, same live-state contract as wheel_telemetry above.
    [[nodiscard]] bool wheel_is_driven(VehicleId id, std::size_t wheel_index) const;
    // PHYS-033 P4 I2 commit 5b (plan section 2): the drivetrain hub's own
    // angular velocity (network state) and the vehicle's cumulative/
    // instantaneous mechanical energy ledger. Sim-thread-only live
    // accessors, same contract as wheel_telemetry above.
    [[nodiscard]] real wheel_omega(VehicleId id, std::size_t wheel_index) const;
    [[nodiscard]] const drivetrain::PowertrainEnergy& powertrain_energy(VehicleId id) const;
    // PHYS-033 P4 I7a C6: the vehicle's fuel tank content in kg (live,
    // sim-thread-only; 0 for a vehicle without a fuel_tank).
    [[nodiscard]] real fuel_tank_mass(VehicleId id) const;
    // PHYS-033 P4 I7a C6: sets the tank content - applied at the NEXT tick
    // boundary (start of the next step()), clamped to [0, capacity], and
    // booked signed (new - old) into PowertrainEnergy::fuel_external_added.
    // A later call before that step replaces an earlier one. Throws
    // std::runtime_error for a vehicle without a fuel_tank or a NaN mass.
    // Call between steps only (never from a job). In I7a the fuel mass does
    // not change the chassis mass; a later increment adds it as a point mass
    // at the tank position (PLAN.md "Fuel mass moves the chassis").
    void set_fuel_tank_mass(VehicleId id, real kg);
    // N2O plan N1: the vehicle's N2O bottle content in kg (live, sim-thread-only; 0 without a nitrous kit) and
    // the same tick-boundary set as set_fuel_tank_mass (clamped to [0, capacity], booked signed into
    // PowertrainEnergy::n2o_external_added; throws for a vehicle without a bottle or a NaN mass).
    [[nodiscard]] real n2o_bottle_mass(VehicleId id) const;
    void set_n2o_bottle_mass(VehicleId id, real kg);
    // Contents mass (vehicle::ContentsMassDesc): the fuel + N2O mass the chassis body carries right now on top of
    // its dry mass (0 when the vehicle does not opt in), and that dry mass (the chassis BodyDesc mass; 0 without
    // opt-in). The body's mass is dry + this; it trails the true contents by less than update_threshold_kg.
    [[nodiscard]] real chassis_contents_mass(VehicleId id) const;
    [[nodiscard]] real chassis_dry_mass(VehicleId id) const;
    // Engine thermal plan T1 (vault PHYS-063): a vehicle with a cooling_system keeps lumped coolant/oil temperatures.
    // engine_thermal_state returns the live values in deg C (sim thread only; throws for a vehicle without thermal),
    // set_engine_thermal_state queues a new state for the next tick boundary (wax/play follow the coolant, fan off;
    // throws for a vehicle without thermal or a non-finite value). reset_vehicle does not touch thermal state.
    [[nodiscard]] drivetrain::EngineThermalState engine_thermal_state(VehicleId id) const;
    void set_engine_thermal_state(VehicleId id, const drivetrain::EngineThermalState& state);
    // Runtime cooling-system factors (a damaged radiator / fan): queued for the next tick boundary like
    // set_engine_thermal_state, kept until the next call (reset_vehicle does not touch them). (1, 1) = the loaded data,
    // bit for bit. Throws for an unknown vehicle, a vehicle without thermal, or a factor not finite / outside [0, 1e3].
    // The effective factors are in PowertrainSnapshot::thermal (radiator_ua_scale, fan_flow_scale).
    void set_engine_cooling_capacity(VehicleId id, const drivetrain::CoolingCapacity& capacity);
    // Runtime engine-internals wear (worn bearings / pistons): EngineWear::friction_scale multiplies the plant's friction
    // scale k(T_oil) (more friction torque, more heat into the oil). Queued for the next tick boundary, kept until the
    // next call (reset_vehicle does not touch it). 1.0 = as new, bit for bit. Needs the engine thermal model (the friction
    // scale lives there). Throws for an unknown vehicle, a vehicle without thermal, or a factor not finite / outside
    // [0, 1e3]. The effective factor is PowertrainSnapshot::thermal.wear_friction_scale.
    void set_engine_wear(VehicleId id, const drivetrain::EngineWear& wear);
    // PHYS-033 P4 I7c C16b: replaces a vehicle's simulated engine's runtime
    // maps (e.g. a background Full-quality regeneration replacing a Preview
    // one, or a tuned/edited map file/cache hit) - EngineMapsPtr is the C16a
    // generator's own public output type (ps/drivetrain/engine_maps.h);
    // internally this rebuilds the engine's richer runtime
    // SimulatedEngineMaps (derived tables included) from the vehicle's
    // original SimulatedEngineDesc with only the maps portion replaced, off
    // the sim thread's job-parallel region (this call itself), then queues
    // just a pointer for the actual swap. Applied at the NEXT tick boundary,
    // mirroring set_fuel_tank_mass's contract exactly: vehicle::System::
    // step_all applies every pending swap sequentially, BEFORE dispatching
    // that tick's per-vehicle jobs, so it takes effect before any substep
    // and the OLD maps are destroyed outside any job (D9's "no allocation
    // /release inside a job" requirement - this call may allocate, the
    // eventual swap and release never do). A later call before that step
    // replaces an earlier one (last request wins). The maps' content_hash is
    // already folded into World::state_hash() every tick via
    // SimulatedEngine::fold_state_hash (D9's "fold every tick" requirement
    // was already satisfied by existing code - nothing new needed there).
    // Call between steps only (never from a job). Throws std::runtime_error
    // for an unknown vehicle id, an unknown/mismatched component id, a
    // component that is not a simulated engine, or null maps.
    void replace_turbo_configuration(VehicleId id, std::string_view component_id, const drivetrain::TurboPairDesc& desc);
    // Q5: the same swap with the full-load envelope built off this thread (ps/drivetrain/turbo_envelope.h). prepare_*:
    // caller thread between steps, cheap. replace (prepared): installs prepared.turbo, takes the prebuilt envelope when its
    // maps are still the engine's (else builds synchronously), never allocates an envelope in a tick. Both throw like
    // replace_turbo_configuration for an unknown vehicle/component.
    [[nodiscard]] drivetrain::PreparedTurboEnvelope prepare_turbo_envelope(VehicleId id, std::string_view component_id,
                                                                           const drivetrain::TurboPairDesc& desc) const;
    void replace_turbo_configuration(VehicleId id, std::string_view component_id, const drivetrain::PreparedTurboEnvelope& prepared);
    void replace_engine_maps(VehicleId id, std::string_view component_id, drivetrain::EngineMapsPtr new_maps);
    // PHYS-033 P4 I4 C3 (plan section 2.4): this tick's copy of every
    // generic component-declared control channel's value for this vehicle
    // (VehicleInstance::component_channel_values(), Powertrain::declare_
    // control_channels' own order) - same live-state-only, sim-thread-only
    // contract as wheel_telemetry above. Empty for every vehicle today (no
    // shipped component declares a channel yet) unless create_vehicle's
    // test_only_extra_channels was used.
    [[nodiscard]] const std::vector<real>& vehicle_component_channel_values(VehicleId id) const;
    // PHYS-033 P4 I3 plan section 4.5: the adapter-facing per-vehicle
    // powertrain snapshot and a by-value debug payload for tests. Both are
    // filled with defaults only until commit 6 wires VehicleInstance's real
    // engine/clutch/gearbox/TCU state into them (powertrain_state.h's own
    // comment). Sim-thread-only live accessors, same contract as
    // wheel_telemetry above.
    [[nodiscard]] const drivetrain::PowertrainSnapshot& powertrain_state(VehicleId id) const;
    [[nodiscard]] drivetrain::PowertrainDebug powertrain_debug(VehicleId id) const;
    // PHYS-033 P4 I3 plan section 4.5: an out-of-band "teleport" of a
    // vehicle's drivetrain state (vehicle_reset.h's own header comment) -
    // call only between ticks, never from a job.
    void reset_vehicle(VehicleId id, const vehicle::VehicleResetOptions& opts);
    // PHYS-033 I1 W1 (section 3.5): sim-thread-only live accessors, same
    // contract as wheel_telemetry above - see VehicleInstance::
    // shadow_dv_error's own comment for what these mean and their one-tick
    // offset.
    [[nodiscard]] real vehicle_shadow_dv_error(VehicleId id) const;
    [[nodiscard]] real vehicle_shadow_dw_error(VehicleId id) const;

    // Registration order is the element's fixed evaluation/gather id
    // (PLAN.md D8: deterministic ordered gather). Ownership stays with the
    // caller, who must keep the element alive (and remove it first if it
    // will be destroyed before the World).
    ForceElementId add_force_element(const IForceElement* element);
    void remove_force_element(ForceElementId id);

    // Advances exactly one fixed tick (WorldConfig::tick_rate_hz) through
    // the staged pipeline (PLAN.md section 5): freeze tick-start state ->
    // scene queries -> vehicle islands -> force elements -> ordered gather
    // -> [solver seam] -> backend step -> terrain -> snapshot.
    void step();

    [[nodiscard]] std::uint64_t tick() const { return tick_; }
    [[nodiscard]] real dt() const { return dt_; }
    [[nodiscard]] real sim_time() const { return sim_time_; }

    // Deterministic hash of the full world state (PLAN.md D7): same binary
    // + same inputs -> bit-identical value regardless of worker count or
    // scheduling.
    [[nodiscard]] std::uint64_t state_hash() const;

    // Wall-clock ms the named pipeline stage (e.g. "terrain_tiles") took in
    // the last step() - the value its "stage.<name>.ms" telemetry channel
    // records. Profiling only (bench_terrain_tiles); -1 for an unknown name.
    [[nodiscard]] double last_stage_ms(const std::string& stage) const;

    [[nodiscard]] jobs::JobSystem& job_system() { return *job_system_; }
    [[nodiscard]] IRigidBackend& backend() { return *backend_; }
    [[nodiscard]] const IRigidBackend& backend() const { return *backend_; }

    [[nodiscard]] SnapshotTripleBuffer& snapshots() { return *snapshots_; }
    [[nodiscard]] io::TelemetryRegistry& telemetry() { return *telemetry_; }

    // Named seam for a future solver stage between force evaluation and the
    // backend step (PLAN.md section 6). No-op unless set; a later phase can
    // install a callback here without changing the pipeline shape.
    using BetweenForcesAndBackendStepHook = std::function<void(World&)>;
    void set_between_forces_and_backend_step_hook(BetweenForcesAndBackendStepHook hook) { between_hook_ = std::move(hook); }

    [[nodiscard]] const jobs::TaskGraph& pipeline() const { return *pipeline_; }

private:
    // Contents mass, one entry per vehicle (index == VehicleId.index; a destroyed vehicle keeps its slot).
    struct ContentsMassState {
        bool enabled = false;
        BodyId chassis{};
        real dry_kg = 0.0;
        real applied_kg = 0.0; // contents currently in the body mass
        real threshold_kg = 0.1;
    };
    std::vector<ContentsMassState> contents_mass_;
    void apply_contents_mass(bool force);
    void build_pipeline();
    // PLAN.md P4 I0 (PHYS-033 section 8, Q4): returns channel's index into
    // control_channel_values_, creating a slot the first time `channel` is
    // named (by set_control or by a vehicle's channel-block resolution at
    // create_vehicle) so the two can be called in either order. `default_value`
    // (PHYS-033 P4 I3 plan section 7.1) seeds a newly-created slot only - it
    // is ignored when the channel already exists, so calling this again for
    // an already-resolved channel never changes its current value. Never
    // called from inside a job.
    std::size_t ensure_control_channel_index(const std::string& channel, real default_value = real(0.0));
    void stage_vehicle_islands();
    void stage_force_elements();
    void stage_gather();
    void stage_contact_coupling_pre();  // PLAN.md P1.5b; between solver_seam and backend_step
    void stage_backend_step();
    void stage_contact_coupling_post(); // PLAN.md P1.5b; between backend_step and terrain_tiles
    void stage_terrain_tiles();         // PLAN.md M1; no-op until set_terrain_source is called
    void stage_snapshot();
    void stage_freeze_tick_start_state(); // weather plan W2: environment clock/sun, no-op without one

    WorldConfig config_;
    real dt_;
    std::uint64_t tick_ = 0;
    bool in_step_ = false;
    std::vector<std::string> vehicle_channel_prefix_; // one per vehicle, retired ones included
    real sim_time_ = 0.0;
    std::vector<aero::WakeSource> aero_wakes_;
    std::shared_ptr<environment::Environment> environment_; // weather plan W2; null == none installed
    // W4: one entry per vehicle (grown in create_vehicle, filled by stage_freeze_tick_start_state while an
    // atmosphere-providing environment is installed). Never hashed.
    std::vector<environment::VehicleAir> vehicle_air_;
    AmbientSample ambient_; // PHYS-033 P4 I7b C9; never hashed, see set_ambient/ambient() above

    std::unique_ptr<jobs::JobSystem> job_system_;
    std::unique_ptr<IRigidBackend> backend_;
    std::unique_ptr<jobs::TaskGraph> pipeline_;
    std::unique_ptr<articulation::System> articulation_system_; // PLAN.md D10/P1.5a; forward-declared above
    std::unique_ptr<terrain::TileManager> terrain_manager_;      // PLAN.md M1; forward-declared above; null until set_terrain_source
    std::unique_ptr<vehicle::System> vehicle_system_;             // PLAN.md 5.1/P3; forward-declared above
    std::shared_ptr<const io::SurfaceTable> surface_table_;       // PLAN.md section 6/P3; null == every surface reads as lambda_mu=1, crr=0
    // PLAN.md P2/M1 step 3, indexed since P4 I0 (PHYS-033 section 8, Q4):
    // name -> index into control_channel_values_. Lookup/insert only,
    // never iterated (D7) - set_control/get_control and per-vehicle
    // channel-block resolution (create_vehicle) are the only callers, and
    // none of this runs inside a job.
    std::unordered_map<std::string, std::size_t> control_channel_index_;
    // Index == a channel's resolved index (ensure_control_channel_index).
    // vehicle::System::step_all reads this directly by each vehicle's own
    // pre-resolved indices (set at create_vehicle) - see
    // core/src/vehicle/vehicle_instance.h's ControlChannelIndices - so the
    // vehicle job never hashes a string.
    std::vector<real> control_channel_values_;
    // PLAN.md P1.5b; index i pairs with ArticulationId{i, 1} (articulations
    // are never destroyed, so index == id.index always); nullptr == that
    // articulation's contact_strategy was None. Declared after backend_
    // (above) so it is destroyed before backend_ (reverse declaration
    // order) - ContactCoupling's destructor calls backend_.destroy_body()
    // on any ProxyBody strategy bodies it created.
    std::vector<std::unique_ptr<articulation::ContactCoupling>> articulation_contacts_;

    std::vector<const IForceElement*> force_elements_; // index == ForceElementId.index; nullptr == removed
    std::vector<Wrench> force_element_wrenches_;        // scatter slots, parallel to force_elements_

    BetweenForcesAndBackendStepHook between_hook_;

    std::unique_ptr<SnapshotTripleBuffer> snapshots_;
    std::unique_ptr<io::TelemetryRegistry> telemetry_;

    // Per-stage timing + worker-utilisation telemetry channels (P1 scope).
    // Wall-clock reads here are profiling/observability only - they never
    // feed back into simulation state or control dt (PLAN.md's "no wall
    // clock in the core" targets the deterministic stepping logic itself).
    std::vector<std::size_t> stage_timing_channels_;
    std::vector<std::size_t> worker_utilization_channels_;
    std::size_t queue_full_fallback_channel_ = 0; // P1.5b review item
    // PLAN.md P3 scope item 7: one row of channel indices per wheel,
    // registered in create_vehicle (so a scenario's CSV header names every
    // wheel by its VehicleId/name before the first step() ever records a
    // row) - fields are slip_ratio, slip_angle, fx, fy, fz (== load),
    // surface (SurfaceId, numeric). Flat per-wheel struct rather than a
    // nested map: World::step()'s row-building stays a tight loop, no
    // per-tick allocation.
    struct WheelTelemetryChannels {
        std::size_t slip_ratio, slip_angle, fx, fy, fz, surface;
        // PHYS-033 P4 I2 commit 5b (plan section 8): the hub's own angular
        // velocity - the network's state now, previously WheelInstance's
        // own omega_ (see wheel_instance.cpp's post_solve, which still
        // fills WheelTelemetry::omega from the network's omega_new).
        std::size_t omega;
    };
    std::vector<std::vector<WheelTelemetryChannels>> wheel_telemetry_channels_; // outer index == VehicleId.index

    // PHYS-033 P4 I2 commit 5b (plan section 8): one row per vehicle,
    // registered right after that vehicle's chassis channels in
    // create_vehicle (same "CSV header complete before the first
    // record_row" reasoning as WheelTelemetryChannels above).
    struct EnergyTelemetryChannels {
        std::size_t engine_out, wheel_out, tyre_loss, brake_loss, gear_loss, integrator_loss, stored, balance_error;
        std::size_t friction_cap_hits, nonspd_hits, lcp_stage3_hits;
        // Owner test-drive wheelspin diagnosis (2026-09-25): CSV-only reads
        // of the tick's PowertrainSnapshot/PowertrainDebug, registered right
        // after lcp_stage3_hits. gear = current gearbox position (-m..n),
        // shift_phase/engine_state = the ps::drivetrain enum's integer value,
        // clutch_slip = omega(clutch.in) - omega(clutch.out) in rad/s,
        // clutch_torque = the clutch friction row's lambda (N*m). Never
        // hashed; recording them changes no simulation state.
        std::size_t gear, engine_rpm, clutch_engagement, clutch_slip, clutch_torque, shift_phase, engine_state;
    };
    std::vector<EnergyTelemetryChannels> energy_telemetry_channels_; // outer index == VehicleId.index

    // PHYS-033 I1 W1 (section 3.5): one row per vehicle, registered right
    // after that vehicle's wheel channels in create_vehicle (same
    // "CSV header complete before the first record_row" reasoning as
    // WheelTelemetryChannels above). The value recorded in tick k is the
    // error of the prediction MADE in tick k-1 (a one-tick offset, section
    // 3.3's own comment) - 0 on a vehicle's first tick.
    struct ChassisTelemetryChannels {
        std::size_t shadow_dv_error, shadow_dw_error;
    };
    std::vector<ChassisTelemetryChannels> chassis_telemetry_channels_; // outer index == VehicleId.index

    // PHYS-033 P4 I4 C8 (plan section 3.1): one entry per differential that
    // is NOT plain open (kind != Open, or lockable) - registered right after
    // that vehicle's powertrain-state channels in create_vehicle, so a
    // plain-open-only vehicle's CSV header (car_sedan.json etc.) stays
    // byte-identical to before this existed. diff_index indexes both
    // PowertrainSnapshot::diffs[] and Powertrain::diff(i)/diffs_, in
    // desc.powertrain.components array order (same order Powertrain builds
    // diffs_ in).
    struct DiffTelemetryChannels {
        std::size_t diff_index;
        std::size_t bias_channel;
        std::size_t locked_channel;
    };
    std::vector<std::vector<DiffTelemetryChannels>> diff_telemetry_channels_; // outer index == VehicleId.index

    // PHYS-033 P4 I7a C5 (plan section 5), broadened C5d: one entry per
    // vehicle whose engine has ANY fuel model (a calibrated fuel_map or the
    // crude no-map estimate, ps::drivetrain::has_fuel_model), registered
    // right after that vehicle's diff-telemetry channels in create_vehicle -
    // a vehicle with no fuel model at all gets an empty inner vector, so its
    // CSV header stays byte-identical. Mirrors diff_telemetry_channels_'s own
    // vector-of-0-or-more-entries pattern rather than adding <optional>.
    struct FuelTelemetryChannels {
        std::size_t flow_g_s, energy_in, heat;
        bool has_tank = false; // PHYS-033 P4 I7a C6: tank_kg registered only for a vehicle with a fuel_tank
        std::size_t tank_kg = 0;
    };
    std::vector<std::vector<FuelTelemetryChannels>> fuel_telemetry_channels_; // outer index == VehicleId.index; inner has 0 or 1 entries

    // N2O plan N1: vehicle<i>.n2o.{flow_g_s,bottle_kg,bottle_bar,active}, registered (and recorded) only for a
    // vehicle with a nitrous kit, right after its engine-maps channel. Outer index == VehicleId.index; inner 0 or 1.
    struct N2oTelemetryChannels {
        std::size_t flow_g_s, bottle_kg, bottle_bar, active;
    };
    std::vector<std::vector<N2oTelemetryChannels>> n2o_telemetry_channels_;

    // Engine thermal plan T1: vehicle<i>.thermal.{coolant_c,oil_c,thermostat,fan,q_wall_w,q_friction_w,q_radiator_w,
    // friction_scale}, registered (and recorded) only for a vehicle with a cooling_system, right after its n2o
    // channels. Outer index == VehicleId.index; inner 0 or 1. With a water jacket three more follow:
    // {metal_c,jacket_heat_w,coolant_flow_l_min} (absent without one, so a jacket-less CSV keeps its old header).
    struct ThermalTelemetryChannels {
        std::size_t coolant_c, oil_c, thermostat, fan, q_wall_w, q_friction_w, q_radiator_w, friction_scale;
        bool jacket = false;
        std::size_t metal_c = 0, jacket_heat_w = 0, coolant_flow_l_min = 0;
    };
    std::vector<std::vector<ThermalTelemetryChannels>> thermal_telemetry_channels_;

    // PHYS-033 P4 I7c C16b: one entry per vehicle with a simulated engine
    // (a ps::drivetrain::SimulatedEngineDesc component, never a
    // TorqueMapEngine), registered right after that vehicle's fuel-telemetry
    // channels in create_vehicle - a vehicle without one gets an empty inner
    // vector, so its CSV header stays byte-identical to before this existed.
    // Mirrors fuel_telemetry_channels_'s own vector-of-0-or-1-entries
    // pattern. CSV-only (plain read, never hashed - the maps' content_hash
    // is already folded into state_hash every tick via SimulatedEngine::
    // fold_state_hash, D9).
    struct EngineMapsTelemetryChannels {
        std::size_t maps_hash_lo32;
    };
    std::vector<std::vector<EngineMapsTelemetryChannels>> engine_maps_hash_channels_; // outer index == VehicleId.index; inner has 0 or 1 entries

    std::vector<double> last_stage_ms_;
    std::vector<ContactEvent> last_contact_events_;
    // PHYS-009 Step 3: reused every tick by stage_snapshot() instead of
    // taking IRigidBackend::live_body_ids()'s by-value return - see that
    // method's comment (i_rigid_backend.h) for why a fresh vector there
    // allocated inside a job every single tick.
    std::vector<BodyId> live_body_ids_scratch_;

public:
    [[nodiscard]] const std::vector<ContactEvent>& last_contact_events() const { return last_contact_events_; }
};

} // namespace ps
