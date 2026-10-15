#include "ps/world/world.h"

#include "ps/environment/environment.h" // weather plan W2

#include "../articulation/contact.h" // internal; PLAN.md P1.5b
#include "../articulation/system.h" // internal; World owns it via the forward-declared pointer (world.h)
#include "../drivetrain/powertrain.h" // internal; PHYS-033 P4 I4 C3's declare_control_channels (create_vehicle)
#include "../terrain/tile_manager.h" // internal; World owns it via the forward-declared pointer (world.h); PLAN.md M1
#include "../vehicle/system.h" // internal; World owns it via the forward-declared pointer (world.h); PLAN.md 5.1/P3

#include <algorithm>
#include <chrono>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <variant>

namespace ps {
void World::set_aero_environment(const aero::Environment& e) {
    // Weather plan W4 (Q12, PROVISIONAL): an atmosphere-providing environment owns density and wind.
    if (environment_ && environment_->provides_atmosphere())
        throw std::logic_error("World::set_aero_environment: the installed environment provides the air density and wind (remove_environment first)");
    aero::validate(e); config_.air_density=e.air_density; config_.wind_world=e.wind_world;
}
void World::set_ambient(const AmbientSample& a) {
    if (environment_ && environment_->provides_atmosphere())
        throw std::logic_error("World::set_ambient: the installed environment provides the ambient atmosphere (remove_environment first)");
    ambient_ = a;
}
environment::VehicleAir World::vehicle_air(VehicleId id) const {
    if (!vehicle_system_->contains(id) || id.index >= vehicle_air_.size()) throw std::invalid_argument("invalid vehicle for vehicle_air");
    return vehicle_air_[id.index];
}
std::uint64_t World::vehicle_engine_steady_rebuild_count(VehicleId id) const {
    if (!vehicle_system_->contains(id)) throw std::invalid_argument("invalid vehicle for vehicle_engine_steady_rebuild_count");
    return vehicle_system_->at(id).engine_steady_rebuild_count();
}
AmbientSample World::vehicle_engine_ambient(VehicleId id) const {
    if (!vehicle_system_->contains(id)) throw std::invalid_argument("invalid vehicle for vehicle_engine_ambient");
    return vehicle_system_->at(id).engine_sensors_ambient();
}
void World::set_aero_wakes(std::vector<aero::WakeSource> wakes) {
    for(const auto& w:wakes) {
        aero::validate(w.desc);
        auto finite=[](Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); };
        if(!finite(w.position_world)||!finite(w.velocity_world)) throw std::invalid_argument("nonfinite aero wake source");
    }
    aero_wakes_=std::move(wakes);
}
void World::set_aero_surface_command(VehicleId id,std::size_t i,real cmd) {
    if(!vehicle_system_->contains(id)) throw std::invalid_argument("invalid vehicle for aero command");
    vehicle_system_->at(id).set_aero_surface_command(i,cmd);
}
void World::set_aero_fan_command(VehicleId id,std::size_t i,real cmd) {
    if(!vehicle_system_->contains(id)) throw std::invalid_argument("invalid vehicle for aero command");
    vehicle_system_->at(id).set_aero_fan_command(i,cmd);
}
void World::credit_aero_fan_energy(VehicleId id,real joules) {
    if(!vehicle_system_->contains(id)) throw std::invalid_argument("invalid vehicle for aero energy credit");
    vehicle_system_->at(id).credit_aero_fan_energy(joules);
}
const aero::Telemetry& World::vehicle_aero_telemetry(VehicleId id) const {
    if(!vehicle_system_->contains(id)) throw std::invalid_argument("invalid vehicle for aero telemetry");
    return vehicle_system_->at(id).aero_telemetry();
}


World::World(const WorldConfig& config) : config_(config), dt_(real(1.0) / config.tick_rate_hz) {
    aero::validate(aero::Environment{config.air_density,config.wind_world});
    job_system_ = std::make_unique<jobs::JobSystem>(config.job_workers, config.job_queue_capacity);
    articulation_system_ = std::make_unique<articulation::System>();
    vehicle_system_ = std::make_unique<vehicle::System>();
    RigidBackendSettings backend_settings;
    backend_settings.baumgarte = config.baumgarte;
    backend_settings.num_velocity_steps = config.num_velocity_steps;
    backend_settings.num_position_steps = config.num_position_steps;
    backend_settings.collision_steps = config.collision_steps;
    backend_settings.penetration_slop = config.penetration_slop;
    backend_settings.speculative_contact_distance = config.speculative_contact_distance;
    backend_settings.max_penetration_distance = config.max_penetration_distance;
    backend_settings.linear_cast_threshold = config.linear_cast_threshold;
    backend_settings.linear_cast_max_penetration = config.linear_cast_max_penetration;
    backend_ = create_jolt_backend(*job_system_, config.gravity, config.max_bodies, backend_settings);
    snapshots_ = std::make_unique<SnapshotTripleBuffer>();
    telemetry_ = std::make_unique<io::TelemetryRegistry>();

    build_pipeline();

    for (const auto& name : pipeline_->stage_names()) {
        stage_timing_channels_.push_back(telemetry_->register_channel("stage." + name + ".ms"));
    }
    for (int w = 0; w < job_system_->worker_count(); ++w) {
        worker_utilization_channels_.push_back(telemetry_->register_channel("jobs.worker" + std::to_string(w) + ".utilization"));
    }
    queue_full_fallback_channel_ = telemetry_->register_channel("jobs.queue_full_fallbacks");
}

World::~World() {
    telemetry_->stop();
}

void World::build_pipeline() {
    pipeline_ = std::make_unique<jobs::TaskGraph>();

    // Per-stage timing (profiling only, see world.h's comment on
    // stage_timing_channels_): each stage lambda times itself and writes
    // into last_stage_ms_[slot]; stages never run concurrently with each
    // other in this pipeline (each depends on the previous one), so a plain
    // member vector is safe to write from inside them without an atomic.
    auto timed = [this](std::size_t slot, std::function<void()> fn) -> jobs::TaskGraph::StageFn {
        return [this, slot, fn]() {
            const auto t0 = std::chrono::steady_clock::now();
            fn();
            const auto t1 = std::chrono::steady_clock::now();
            last_stage_ms_[slot] = std::chrono::duration<double, std::milli>(t1 - t0).count();
        };
    };

    std::size_t slot = 0;
    const auto freeze = pipeline_->add_stage("freeze_tick_start_state", timed(slot++, [this] { stage_freeze_tick_start_state(); }));
    const auto queries = pipeline_->add_stage("scene_queries", timed(slot++, [] {}), {freeze});
    const auto islands = pipeline_->add_stage("vehicle_islands_substep", timed(slot++, [this] { stage_vehicle_islands(); }), {queries});
    const auto force_el = pipeline_->add_stage("force_elements", timed(slot++, [this] { stage_force_elements(); }), {islands});
    const auto gather = pipeline_->add_stage("gather", timed(slot++, [this] { stage_gather(); }), {force_el});
    const auto solver_seam = pipeline_->add_stage(
        "solver_seam", timed(slot++, [this] { if (between_hook_) between_hook_(*this); }), {gather});
    // PLAN.md P1.5b: Compliant queries+applies its contact wrenches here
    // (right before backend_step, so the reaction it applies to the other
    // body affects THIS tick's step); ProxyBody pose/velocity-syncs its
    // proxy bodies here so backend_step solves their contacts below.
    const auto contact_pre = pipeline_->add_stage("contact_coupling_pre", timed(slot++, [this] { stage_contact_coupling_pre(); }), {solver_seam});
    const auto backend_step = pipeline_->add_stage("backend_step", timed(slot++, [this] { stage_backend_step(); }), {contact_pre});
    // PLAN.md P1.5b: ProxyBody reads back each proxy's post-step velocity
    // change here and accumulates the resulting wrench for next tick
    // (Compliant is a no-op here, already fully applied above).
    const auto contact_post = pipeline_->add_stage("contact_coupling_post", timed(slot++, [this] { stage_contact_coupling_post(); }), {backend_step});
    const auto terrain = pipeline_->add_stage("terrain_tiles", timed(slot++, [this] { stage_terrain_tiles(); }), {contact_post});
    pipeline_->add_stage("snapshot", timed(slot++, [this] { stage_snapshot(); }), {terrain});

    last_stage_ms_.assign(slot, 0.0);
}

std::size_t World::ensure_control_channel_index(const std::string& channel, real default_value) {
    const auto it = control_channel_index_.find(channel);
    if (it != control_channel_index_.end()) return it->second;
    const std::size_t idx = control_channel_values_.size();
    control_channel_values_.push_back(default_value);
    control_channel_index_.emplace(channel, idx);
    return idx;
}

std::vector<std::string> World::control_channel_names() const {
    std::vector<std::string> names;
    names.reserve(control_channel_index_.size());
    for (const auto& [name, idx] : control_channel_index_) {
        (void)idx;
        names.push_back(name);
    }
    return names;
}

ForceElementId World::add_force_element(const IForceElement* element) {
    force_elements_.push_back(element);
    force_element_wrenches_.push_back(Wrench{});
    return ForceElementId{static_cast<id_index_t>(force_elements_.size() - 1), 1};
}

void World::remove_force_element(ForceElementId id) {
    if (id.index < force_elements_.size()) force_elements_[id.index] = nullptr;
}

void World::stage_force_elements() {
    if (force_elements_.empty()) return;
    const std::size_t n = force_elements_.size();
    const int workers = std::max(1, job_system_->worker_count());
    const std::size_t grain = config_.force_element_grain != 0
        ? config_.force_element_grain
        : std::max<std::size_t>(1, (n + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));

    job_system_->parallel_for(n, grain, [this](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            const IForceElement* el = force_elements_[i];
            if (!el) {
                force_element_wrenches_[i] = Wrench{};
                continue;
            }
            const Frame& frame = el->frame();
            ForceElementContext ctx;
            ctx.frame_pose = frame.world_pose(*backend_);
            ctx.frame_motion = frame.world_motion(*backend_);
            ctx.dt = dt_;
            ctx.tick = tick_;
            // Scatter: writes only to this element's own pre-allocated slot
            // (PLAN.md D8) - no other job reads or writes index i.
            force_element_wrenches_[i] = el->evaluate(ctx);
        }
    });
}

void World::stage_vehicle_islands() {
    // PLAN.md D10/P1.5a/D8, delivered in PHYS-009 Step 1: one job per
    // articulation (System::step_all, batched by config_.articulation_grain
    // - see its own comment in system.h for why no separate gather step is
    // needed here). Consumes the external wrenches gathered during the
    // PREVIOUS tick's stage_gather (D3: multi-rate stepping - islands
    // substep using state/forces already known at the start of this tick,
    // then force_elements/gather below compute this tick's wrenches for
    // the NEXT tick's islands stage), then clears them so stage_gather can
    // accumulate fresh ones for next tick.
    if (!articulation_system_->empty()) {
        articulation::InstanceConfig cfg;
        cfg.gravity = config_.gravity;
        cfg.substep_rate_hz = config_.substep_rate_hz;
        articulation_system_->step_all(*job_system_, cfg, dt_, config_.articulation_grain);
        articulation_system_->clear_external_wrenches();
    }

    // PLAN.md 5.1/P3 (out/handover_p3.md decision 3): vehicles step in the
    // same stage, right after articulations - both are "vehicle islands" in
    // PLAN.md D8's sense (per-instance internal state sub-stepped at
    // substep_rate_hz, one job per instance). PLAN.md P4 I0 (PHYS-033
    // section 8, Q4): each vehicle now reads its OWN pre-resolved channel
    // indices (create_vehicle) out of control_channel_values_ - passed
    // through unchanged here, so this stage does no per-tick string lookup
    // and no allocation.
    if (!vehicle_system_->empty()) {
        // PHYS-033 I1 W1: the shadow chassis needs gravity to build its own
        // per-substep acceleration estimate (it never talks to World
        // otherwise) - see vehicle::VehicleTickEnv.
        vehicle::VehicleTickEnv env{config_.gravity, config_.air_density, ambient_, config_.wind_world, {}, aero_wakes_};
        // Weather plan W6: hand the installed environment's condition grip to
        // the wheels (nullptr without surface conditions, a no-op then).
        if (environment_) env.grip = environment_->condition_grip();
        // Weather plan W4: per-vehicle air (valid entries only while an atmosphere-providing environment is installed).
        vehicle_system_->step_all(*backend_, *job_system_, control_channel_values_, env, dt_, config_.substep_rate_hz,
                                  std::span<const environment::VehicleAir>(vehicle_air_));
    }
}

void World::stage_gather() {
    // Ordered gather, sequential in fixed element-id order (PLAN.md D8:
    // "never sum floats in completion order"): result is independent of
    // which worker computed which wrench or in what order.
    for (std::size_t i = 0; i < force_elements_.size(); ++i) {
        const IForceElement* el = force_elements_[i];
        if (!el) continue;
        const Frame& frame = el->frame();
        if (frame.is_articulation_link()) {
            // PLAN.md D10/P1.5a: "their wrenches reach the articulation as
            // external spatial forces" - held constant across this
            // articulation's substeps next tick (D3), same as a Jolt
            // body's accumulated wrench is held for one backend step.
            articulation_system_->accumulate_external_wrench(
                frame.articulation_link(), frame.world_pose(*backend_), force_element_wrenches_[i]);
        } else {
            backend_->apply_wrench(frame.body(), force_element_wrenches_[i]);
            // PHYS-033 I1 W1 (section 3.4): a no-op for any body that is not
            // a vehicle chassis (System::note_body_wrench's own array
            // lookup). Sequential, fixed element order - same determinism
            // guarantee as the apply_wrench call above.
            vehicle_system_->note_body_wrench(frame.body(), force_element_wrenches_[i]);
        }
    }
}

void World::stage_contact_coupling_pre() {
    // Sequential, fixed ArticulationId (== index) order (PLAN.md D8's
    // determinism contract needs a fixed order here, not a parallel
    // scatter/gather - see contact.h's comment on why that is enough for
    // the contact counts P1.5b's scenarios actually have).
    for (std::size_t i = 0; i < articulation_contacts_.size(); ++i) {
        if (!articulation_contacts_[i]) continue;
        const ArticulationId id{static_cast<id_index_t>(i), 1};
        articulation_contacts_[i]->pre_backend_step(articulation_system_->at(id), *backend_);
    }
}

void World::stage_contact_coupling_post() {
    for (std::size_t i = 0; i < articulation_contacts_.size(); ++i) {
        if (!articulation_contacts_[i]) continue;
        const ArticulationId id{static_cast<id_index_t>(i), 1};
        articulation_contacts_[i]->post_backend_step(*articulation_system_, id, *backend_, dt_, config_.gravity);
    }
}

void World::stage_backend_step() {
    backend_->step(dt_);
    backend_->drain_contact_events_into(last_contact_events_);
}

// Weather plan W2: the one place environment work runs (a new stage would add
// a stage.<name>.ms channel and change every scenario's CSV header). The
// environment evaluates this tick's snapshot - a closed-form function of the
// tick, allocation-free. No environment: nothing.
//
// W4: with an atmosphere-providing environment the same stage also samples the
// air at every chassis (ISA lift to the chassis altitude, humid density, mean
// wind + gust at its position) into vehicle_air_, which vehicle_islands_substep
// hands to each vehicle. Sequential, fixed vehicle order, allocation-free.
void World::stage_freeze_tick_start_state() {
    if (!environment_) return;
    environment_->begin_tick(static_cast<std::int64_t>(tick_));
    if (!environment_->provides_atmosphere()) return;
    const std::size_t n = vehicle_system_->count();
    for (std::size_t i = 0; i < n; ++i) {
        const VehicleId id{static_cast<id_index_t>(i), 1};
        vehicle_air_[i] = environment_->sample_air(backend_->get_pose(vehicle_system_->at(id).chassis()).position);
    }
}

void World::install_environment(std::shared_ptr<environment::Environment> env) {
    if (!env) throw std::invalid_argument("World::install_environment: null environment");
    // W5: the condition classes are indexed by SurfaceId, so they must be this
    // world's surface table, in id order (checked before anything changes).
    if (const environment::ConditionGrid* g = env->surface_conditions(); g && surface_table_) {
        bool same = static_cast<std::size_t>(g->class_count()) == surface_table_->size();
        for (int i = 0; same && i < g->class_count(); ++i)
            same = g->class_name(i) == surface_table_->name_for(static_cast<SurfaceId>(i));
        if (!same)
            throw std::invalid_argument("World::install_environment: the environment's surface classes do not match "
                                        "the world's surface table (pass its names in id order as "
                                        "EnvironmentConfig::surface_names)");
    }
    env->attach(static_cast<std::int64_t>(tick_), config_.tick_rate_hz);
    environment_ = std::move(env);
    // The air of an environment that does not provide one (or of the previous
    // environment) must not leak into the next step.
    for (environment::VehicleAir& a : vehicle_air_) a = environment::VehicleAir{};
}

void World::remove_environment() {
    environment_.reset();
    for (environment::VehicleAir& a : vehicle_air_) a = environment::VehicleAir{};
}

const environment::EnvironmentSnapshot* World::environment_snapshot() const {
    return environment_ ? &environment_->current() : nullptr;
}

void World::set_environment_time(real env_s) {
    if (!environment_) throw std::logic_error("World::set_environment_time: no environment installed");
    environment_->set_time(env_s, static_cast<std::int64_t>(tick_));
}

void World::set_environment_time_scale(real scale) {
    if (!environment_) throw std::logic_error("World::set_environment_time_scale: no environment installed");
    environment_->set_time_scale(scale, static_cast<std::int64_t>(tick_));
}

int World::add_weather_override_to_season_end(const environment::WeatherPresetDesc& preset, real ramp_s) {
    if (!environment_) throw std::logic_error("World::add_weather_override_to_season_end: no environment installed");
    return environment_->add_season_weather_override(preset, ramp_s, static_cast<std::int64_t>(tick_));
}

void World::end_weather_overrides() {
    if (!environment_) throw std::logic_error("World::end_weather_overrides: no environment installed");
    environment_->end_weather_overrides(static_cast<std::int64_t>(tick_));
}

bool World::remove_weather_override(int id) {
    if (!environment_) throw std::logic_error("World::remove_weather_override: no environment installed");
    return environment_->remove_weather_override(id, static_cast<std::int64_t>(tick_));
}

void World::clear_weather_overrides() {
    if (!environment_) throw std::logic_error("World::clear_weather_overrides: no environment installed");
    environment_->clear_weather_overrides(static_cast<std::int64_t>(tick_));
}

void World::set_environment_calendar_lock(const std::string& fixed_season, int fixed_day_of_year) {
    if (!environment_) throw std::logic_error("World::set_environment_calendar_lock: no environment installed");
    environment_->set_calendar_lock(fixed_season, fixed_day_of_year, static_cast<std::int64_t>(tick_));
}

void World::stage_terrain_tiles() {
    if (!terrain_manager_) return; // PLAN.md M1: no-op until set_terrain_source is called
    terrain_manager_->update(*job_system_);
}

void World::stage_snapshot() {
    WorldSnapshot& snap = snapshots_->write_slot();
    snap.tick = tick_;
    snap.sim_time = sim_time_;
    snap.bodies.clear();
    snap.deformation_payload.clear(); // seam only (PLAN.md section 6); unused in P1

    backend_->live_body_ids_into(live_body_ids_scratch_);
    for (BodyId id : live_body_ids_scratch_) {
        snap.bodies.push_back(BodySnapshot{id, backend_->get_pose(id), backend_->get_motion(id)});
    }

    // PLAN.md D10/P1.5a + P2 adapter seam (this task's step 3): every live
    // articulation's link poses, in ArticulationId order (System::count()'s
    // own comment - instances_ is already indexed that way). This stage
    // runs inside a job every tick (PLAN.md D8), so - exactly like `bodies`
    // above - it must stay allocation-free once warmed up: resize() (not
    // clear()+push_back on the outer vector) keeps each of the triple
    // buffer's 3 WorldSnapshot slots' own `articulations` AND each
    // ArticulationSnapshot's own `links` vector at their steady-state
    // capacity across ticks, rather than destroying and rebuilding them
    // fresh every tick (an earlier version of this stage did exactly that
    // and tripped test_articulation_parallel_alloc.cpp's zero-alloc check).
    const std::size_t articulation_count = articulation_system_->count();
    snap.articulations.resize(articulation_count);
    for (std::size_t i = 0; i < articulation_count; ++i) {
        const ArticulationId id{static_cast<id_index_t>(i), 1};
        const articulation::Instance& inst = articulation_system_->at(id);
        ArticulationSnapshot& asnap = snap.articulations[i];
        asnap.id = id;
        asnap.links.clear();
        asnap.links.push_back(ArticulationLinkSnapshot{-1, inst.link_world_pose(-1), inst.link_world_motion(-1)});
        for (int link_index = 0; link_index < inst.link_count(); ++link_index) {
            asnap.links.push_back(ArticulationLinkSnapshot{link_index, inst.link_world_pose(link_index), inst.link_world_motion(link_index)});
        }
    }

    // PLAN.md M1, task 2 of the P3 fix-up phase: every live vehicle's
    // per-wheel render state - same zero-alloc-once-warmed-up pattern as
    // `articulations` above (outer resize(), inner clear()+push_back()).
    const std::size_t vehicle_count = vehicle_system_->count();
    snap.vehicles.resize(vehicle_count);
    for (std::size_t vi = 0; vi < vehicle_count; ++vi) {
        const VehicleId id{static_cast<id_index_t>(vi), 1};
        const vehicle::VehicleInstance& vinst = vehicle_system_->at(id);
        VehicleSnapshot& vsnap = snap.vehicles[vi];
        vsnap.id = id;
        vsnap.wheels.clear();
        const std::size_t wc = vinst.wheel_count();
        for (std::size_t w = 0; w < wc; ++w) vsnap.wheels.push_back(vinst.wheel_state(w));
        // PHYS-033 P4 I3 plan section 4.3 commit 5: default-constructed
        // until commit 6 wires real engine/clutch/gearbox/TCU state into
        // VehicleInstance::powertrain_state_.
        vsnap.powertrain = vinst.powertrain_state();
        vsnap.aero=vinst.aero_telemetry();
        vsnap.air = vi < vehicle_air_.size() ? vehicle_air_[vi] : environment::VehicleAir{};
    }

    // Weather plan W2: this tick's environment record (a POD copy, no
    // allocation); left default and flagged absent without an environment.
    snap.has_environment = static_cast<bool>(environment_);
    if (environment_) snap.environment = environment_->current();

    // PLAN.md P2 adapter seam (this task's step 3): a race-free copy of this
    // tick's contact events - see snapshot.h's own comment on why this must
    // be a copy, not a reference to last_contact_events_, for a cross-thread
    // reader (PLAN.md D3). Assignment (not clear()+insert) so the slot's
    // existing capacity is reused once warmed up, same reasoning as above.
    snap.contacts = last_contact_events_;

    snapshots_->publish();
}

void World::step() {
    job_system_->reset_worker_stats();
    const auto tick_t0 = std::chrono::steady_clock::now();

    // PHYS-033 P4 I7c C16b (D9): queued engine-maps hot swaps are applied,
    // and the old maps released, here - on the calling (sim) thread, before
    // the pipeline, outside any job (every pipeline stage runs as a job).
    vehicle_system_->apply_pending_engine_maps_swaps();

    pipeline_->run(*job_system_);

    // Assists I7c: control-channel writes the vehicles queued during their step (a drive-mode change writes its TC /
    // launch-control default into the channel itself). Applied HERE - sequentially, on the sim thread, after every job
    // has finished - in vehicle id order, straight into the flat channel array (no allocation). When two vehicles name
    // the same (global) channel the later vehicle overwrites the earlier one. A set_control made before the next
    // step() therefore wins over the physics' write from the next tick on.
    for (std::size_t vi = 0; vi < vehicle_system_->count(); ++vi) {
        for (const vehicle::PendingChannelWrite& w : vehicle_system_->at(VehicleId{static_cast<id_index_t>(vi), 1}).pending_channel_writes()) {
            control_channel_values_[w.index] = w.value;
        }
    }

    const auto tick_t1 = std::chrono::steady_clock::now();
    const double tick_ns = std::max(
        1.0, std::chrono::duration<double, std::nano>(tick_t1 - tick_t0).count());

    std::size_t wheel_channel_count = 0;
    for (const auto& channels : wheel_telemetry_channels_) wheel_channel_count += channels.size() * 7; // +omega, PHYS-033 P4 I2 commit 5b
    const std::size_t chassis_channel_count = chassis_telemetry_channels_.size() * 2; // PHYS-033 I1 W1
    const std::size_t energy_channel_count = energy_telemetry_channels_.size() * 18;  // PHYS-033 P4 I2 commit 5b (11) + powertrain state (7)
    std::size_t diff_channel_count = 0; // PHYS-033 P4 I4 C8: bias + locked per registered (non-plain-open) diff
    for (const auto& channels : diff_telemetry_channels_) diff_channel_count += channels.size() * 2;
    std::size_t fuel_channel_count = 0; // PHYS-033 P4 I7a C5: flow_g_s + energy_in + heat, only for a fuel-mapped engine
    for (const auto& channels : fuel_telemetry_channels_) {
        for (const FuelTelemetryChannels& fc : channels) fuel_channel_count += fc.has_tank ? 4 : 3; // + tank_kg (PHYS-033 P4 I7a C6)
    }

    std::vector<real> row;
    row.reserve(stage_timing_channels_.size() + worker_utilization_channels_.size() + 1 + wheel_channel_count +
                chassis_channel_count + energy_channel_count + diff_channel_count + fuel_channel_count);
    for (double ms : last_stage_ms_) row.push_back(real(ms));
    for (int w = 0; w < job_system_->worker_count(); ++w) {
        const double busy_ns = static_cast<double>(job_system_->worker_busy_ns(w));
        row.push_back(real(busy_ns / tick_ns));
    }
    row.push_back(real(job_system_->queue_full_fallback_count())); // cumulative, P1.5b review item
    // PLAN.md P3 scope item 7: per-wheel telemetry. Pushed in exactly the
    // same (vehicle index, wheel index, field) order create_vehicle
    // registered the channels in above, so these push_backs land on the
    // matching column without needing to index by channel number.
    for (std::size_t vi = 0; vi < wheel_telemetry_channels_.size(); ++vi) {
        const VehicleId vid{static_cast<id_index_t>(vi), 1};
        const std::vector<WheelTelemetryChannels>& channels = wheel_telemetry_channels_[vi];
        for (std::size_t w = 0; w < channels.size(); ++w) {
            const vehicle::WheelTelemetry& t = vehicle_system_->at(vid).wheel_telemetry(w);
            row.push_back(t.slip_ratio);
            row.push_back(t.slip_angle);
            row.push_back(t.fx);
            row.push_back(t.fy);
            row.push_back(t.fz);
            row.push_back(real(t.surface));
            row.push_back(t.omega); // PHYS-033 P4 I2 commit 5b
        }
        // PHYS-033 I1 W1 (section 3.5): right after this vehicle's wheel
        // values, matching the registration order in create_vehicle above.
        row.push_back(vehicle_system_->at(vid).shadow_dv_error());
        row.push_back(vehicle_system_->at(vid).shadow_dw_error());

        // PHYS-033 P4 I2 commit 5b (plan section 8): right after this
        // vehicle's chassis values, matching create_vehicle's registration
        // order.
        const vehicle::VehicleInstance& vehicle_instance = vehicle_system_->at(vid);
        const drivetrain::PowertrainEnergy& e = vehicle_instance.powertrain_energy();
        row.push_back(e.engine_out);
        row.push_back(e.wheel_out);
        row.push_back(e.tyre_loss);
        row.push_back(e.brake_loss);
        row.push_back(e.gear_loss);
        row.push_back(e.integrator_loss);
        // PHYS-033 P4 I4 C13: "stored" includes the compliant shafts' spring
        // energy - only for a vehicle that has any (Rigid: literally rotor_ke).
        row.push_back(vehicle_instance.powertrain_has_compliant_shafts() ? e.rotor_ke + e.shaft_pe : e.rotor_ke);
        row.push_back(e.balance_error);
        row.push_back(real(vehicle_instance.powertrain_friction_cap_hits()));
        row.push_back(real(vehicle_instance.powertrain_nonspd_hits()));
        row.push_back(real(vehicle_instance.powertrain_lcp_stage3_hits()));
        // Powertrain state channels (EnergyTelemetryChannels::gear..
        // engine_state, same registration order) - plain reads, never hashed.
        const drivetrain::PowertrainSnapshot& ps_snap = vehicle_instance.powertrain_state();
        const drivetrain::PowertrainDebug ps_dbg = vehicle_instance.powertrain_debug();
        row.push_back(real(ps_snap.gear));
        row.push_back(ps_snap.engine_rpm);
        row.push_back(ps_snap.clutch_engagement);
        row.push_back(ps_dbg.clutch_slip);
        row.push_back(ps_dbg.clutch_torque);
        row.push_back(real(static_cast<int>(ps_snap.shift_phase)));
        row.push_back(real(static_cast<int>(ps_snap.engine_state)));

        // PHYS-033 P4 I4 C8 (plan section 3.1): per-diff bias/locked, right
        // after the powertrain-state values, matching create_vehicle's
        // registration order. Empty for a plain-open-only vehicle (every
        // shipped vehicle today) - reuses the already-read ps_snap above.
        if (vi < diff_telemetry_channels_.size()) {
            for (const DiffTelemetryChannels& dc : diff_telemetry_channels_[vi]) {
                row.push_back(ps_snap.diffs[dc.diff_index].bias);
                row.push_back(ps_snap.diffs[dc.diff_index].locked ? real(1.0) : real(0.0));
            }
        }

        // PHYS-033 P4 I7a C5, broadened C5d: fuel flow/energy_in/heat, right
        // after this vehicle's diff values, matching create_vehicle's
        // registration order. Empty for a vehicle with no fuel model at all
        // (map or crude) - reuses ps_snap/e above.
        if (vi < fuel_telemetry_channels_.size() && !fuel_telemetry_channels_[vi].empty()) {
            row.push_back(ps_snap.fuel_flow_g_s);
            row.push_back(e.fuel_energy_in);
            row.push_back(e.engine_heat);
            if (fuel_telemetry_channels_[vi].front().has_tank) row.push_back(ps_snap.fuel_mass_kg); // PHYS-033 P4 I7a C6
        }

        // PHYS-033 P4 I7c C16b: the current engine maps' content_hash, low
        // 32 bits, right after this vehicle's fuel values, matching
        // create_vehicle's registration order. Empty for a vehicle without a
        // simulated engine.
        if (vi < engine_maps_hash_channels_.size() && !engine_maps_hash_channels_[vi].empty()) {
            row.push_back(real(static_cast<std::uint32_t>(vehicle_instance.simulated_engine_maps_content_hash())));
        }

        // N2O plan N1: the kit-only channels, right after the engine-maps value, matching create_vehicle's order.
        if (vi < n2o_telemetry_channels_.size() && !n2o_telemetry_channels_[vi].empty()) {
            const drivetrain::NitrousSystem* n2o = vehicle_instance.nitrous_system();
            row.push_back(n2o->status().mdot_n_kg_s * real(1000.0));
            row.push_back(n2o->bottle_kg());
            row.push_back(n2o->bottle_gauge_bar());
            row.push_back(n2o->status().flowing ? real(1.0) : real(0.0));
        }

        // Engine thermal plan T1: the thermal channels, right after the n2o block, matching create_vehicle's order.
        if (vi < thermal_telemetry_channels_.size() && !thermal_telemetry_channels_[vi].empty()) {
            const drivetrain::EngineThermalSnapshot& th = vehicle_instance.thermal_snapshot();
            row.push_back(th.coolant_c);
            row.push_back(th.oil_c);
            row.push_back(th.thermostat_opening);
            row.push_back(th.fan_on ? real(1.0) : real(0.0));
            row.push_back(th.wall_heat_w);
            row.push_back(th.friction_heat_w);
            row.push_back(th.radiator_heat_w);
            row.push_back(th.friction_scale);
        }
    }
    telemetry_->record_row(tick_, sim_time_, std::move(row));

    ++tick_;
    sim_time_ += dt_;
}

std::uint64_t World::state_hash() const {
    std::uint64_t h = 14695981039346656037ULL; // FNV-1a offset basis
    h ^= tick_;
    h *= 1099511628211ULL;
    h = backend_->compute_state_hash(h);
    // PLAN.md D10/P1.5a: articulation joint state folds in too (D7: "same
    // binary + same inputs -> bit-identical state" applies to it exactly
    // as it does to every Jolt body).
    h = articulation_system_->fold_state_hash(h);
    // PLAN.md 5.1/P3: every wheel's internal (not otherwise observable)
    // state folds in too - see vehicle::System::fold_state_hash's comment.
    h = vehicle_system_->fold_state_hash(h);
    // Weather plan W2: the environment's clock anchor folds in only while one
    // is installed, so a World without one hashes exactly as before.
    if (environment_) h = environment_->fold_state_hash(h);
    return h;
}

ArticulationId World::create_articulation(const articulation::ArticulationDesc& desc) {
    const ArticulationId id = articulation_system_->create(desc);
    if (desc.contact_strategy != articulation::ContactStrategy::None) {
        // PLAN.md P1.5b: articulation_contacts_[i] pairs with ArticulationId{i, 1}
        // (see world.h's comment on the member) - grow to keep that index
        // aligned even though articulations are never destroyed, so id.index
        // is always articulation_contacts_.size() - 1 the first time this fires
        // for a given id, but writing it this way keeps the invariant explicit.
        if (articulation_contacts_.size() <= id.index) articulation_contacts_.resize(id.index + 1);
        articulation_contacts_[id.index] = std::make_unique<articulation::ContactCoupling>(desc, articulation_system_->at(id), *backend_);
        if (desc.contact_strategy == articulation::ContactStrategy::Compliant) {
            // PLAN.md P1.5b/contact_source.h: wired once here, never per
            // tick - Instance::step_tick calls through this pointer at
            // substep rate for the rest of this articulation's lifetime.
            articulation_system_->at(id).set_contact_source(articulation_contacts_[id.index].get(), backend_.get());
        }
    }
    return id;
}

real World::joint_position(ArticulationId id, int link_index) const {
    return articulation_system_->at(id).joint_position(link_index);
}

real World::joint_velocity(ArticulationId id, int link_index) const {
    return articulation_system_->at(id).joint_velocity(link_index);
}

void World::set_joint_force(ArticulationId id, int link_index, real tau) {
    articulation_system_->at(id).set_joint_force(link_index, tau);
}

Frame World::link_frame(ArticulationId id, int link_index, Pose local_offset) const {
    return articulation_system_->link_frame(id, link_index, local_offset);
}

VehicleId World::create_vehicle(const vehicle::VehicleDesc& desc, BodyId chassis,
                                 const std::vector<std::pair<std::string, real>>& test_only_extra_channels) {
    aero::validate(desc.aero);
    // PLAN.md P4 I0 (PHYS-033 section 8, Q4): resolve this vehicle's four
    // control channels to indices ONCE, here (setup time, off the tick
    // path - never called from inside a job). Vehicle index 0 (the next
    // instance's index, since vehicles are never destroyed - same
    // invariant wheel_telemetry_channels_ below relies on) is the
    // designated input vehicle and reads bare global names ("steer", ...);
    // every other vehicle reads its own "v<i>." prefixed names. The
    // string API (World::set_control/get_control) is unaffected - a
    // caller may still set "v1.steer" etc. at any time, before or after
    // this vehicle is created (ensure_control_channel_index handles both
    // orders).
    constexpr id_index_t kDesignatedInputVehicle = 0;
    const id_index_t vehicle_index = static_cast<id_index_t>(vehicle_system_->count());
    const std::string channel_prefix = vehicle_index == kDesignatedInputVehicle ? std::string() : ("v" + std::to_string(vehicle_index) + ".");
    vehicle::ControlChannelIndices channel_indices;
    channel_indices.steer = ensure_control_channel_index(channel_prefix + "steer");
    channel_indices.throttle = ensure_control_channel_index(channel_prefix + "throttle");
    channel_indices.brake = ensure_control_channel_index(channel_prefix + "brake");
    channel_indices.handbrake = ensure_control_channel_index(channel_prefix + "handbrake");
    // PHYS-033 P4 I3 plan section 7.1: the shaft-network powertrain's
    // clutch/gearshift/ignition/assist channels - not yet read by
    // VehicleInstance::step_tick anywhere (commit 6 consumes them), but
    // resolved here now so a later scenario/adapter can start driving them
    // before commit 6 lands, same "either order" reasoning as the four
    // channels above. Plan section 7.1's defaults table; if a vehicle has
    // no manual_tcu, the assist channels are still created (ignored).
    channel_indices.clutch = ensure_control_channel_index(channel_prefix + "clutch");
    channel_indices.shift_up_count = ensure_control_channel_index(channel_prefix + "shift_up_count");
    channel_indices.shift_down_count = ensure_control_channel_index(channel_prefix + "shift_down_count");
    channel_indices.gear_request = ensure_control_channel_index(channel_prefix + "gear_request", drivetrain::kGearRequestNone);
    channel_indices.ignition = ensure_control_channel_index(channel_prefix + "ignition", real(1.0));
    channel_indices.starter = ensure_control_channel_index(channel_prefix + "starter");
    // assist.*: the RUNTIME switch for each driver aid (plan section 7.1 -
    // toggled from the gamepad/HUD); the TCU desc's own `enabled` flags are
    // only these channels' defaults. No manual_tcu: created, default 0,
    // ignored.
    // Engine garage P8 (A15): a NOT-FITTED assist still registers its channel (stable channel list) but with
    // default 0, and VehicleInstance ignores the channel.
    real assist_clutch_default = real(0.0), assist_blip_default = real(0.0), assist_shift_default = real(0.0);
    std::string launch_channel_name = "assist.launch_control";
    bool launch_not_fitted = false;  // a P8 launch_control controller with fitted = false
    bool has_launch_control = false; // the channel is registered only for a car that has the part (list unchanged for every other car)
    for (const drivetrain::ControllerDesc& cd : desc.controllers) {
        if (const auto* tcu = std::get_if<drivetrain::ManualTcuDesc>(&cd.params)) {
            assist_clutch_default = (tcu->auto_clutch.enabled && tcu->auto_clutch.fitted) ? real(1.0) : real(0.0);
            assist_blip_default = (tcu->auto_blip.enabled && tcu->auto_blip.fitted) ? real(1.0) : real(0.0);
            assist_shift_default = (tcu->auto_shift.enabled && tcu->auto_shift.fitted) ? real(1.0) : real(0.0);
        } else if (const auto* lc = std::get_if<drivetrain::LaunchControlDesc>(&cd.params)) {
            launch_channel_name = lc->channel;
            has_launch_control = true;
            launch_not_fitted = !lc->fitted;
        }
    }
    channel_indices.assist_auto_clutch = ensure_control_channel_index(channel_prefix + "assist.auto_clutch", assist_clutch_default);
    channel_indices.assist_auto_blip = ensure_control_channel_index(channel_prefix + "assist.auto_blip", assist_blip_default);
    channel_indices.assist_auto_shift = ensure_control_channel_index(channel_prefix + "assist.auto_shift", assist_shift_default);
    channel_indices.assist_traction_control = ensure_control_channel_index(channel_prefix + "assist.traction_control",ps::vehicle::traction_default_channel_value(desc.traction_control,desc.assist_unit,desc.drive_modes));
    // assists I7: registered only on a car with a drive_modes part; default = the part's default mode code.
    if (desc.drive_modes) {
        channel_indices.assist_drive_mode = ensure_control_channel_index(channel_prefix + "assist.drive_mode",
                                                                         static_cast<real>(static_cast<int>(desc.drive_modes->default_mode)));
    }
    // Q3 / I8: the EPB switch level, registered only on a car with an epb part (the channel list is unchanged for every
    // other car); default 0 = released.
    if (desc.epb) channel_indices.epb = ensure_control_channel_index(channel_prefix + "epb", real(0));
    // assists I5: registered only on a car whose assist unit carries a fitted aero_control (the channel list is
    // unchanged for every other car); default = the block's default_enabled.
    if (const auto aero_surface = ps::vehicle::find_aero_control_surface(desc.aero, desc.assist_unit)) {
        (void)aero_surface;
        channel_indices.assist_aero_control = ensure_control_channel_index(channel_prefix + "assist.aero_control",
                                                                           desc.assist_unit->aero_control->default_enabled ? real(1) : real(0));
    }
    // engine garage P8: never read on a car without the part (VehicleInstance gates it on fit_launch_)
    // assists I4: a car whose assist unit carries a launch_control block has the channel too (name assist.launch_control,
    // default = the block's default_enabled); a car with BOTH variants is rejected by the loader and VehicleInstance.
    real launch_default = real(0);
    if (desc.assist_unit && desc.assist_unit->launch_control) {
        has_launch_control = true;
        launch_default = desc.assist_unit->launch_control->default_enabled ? real(1) : real(0);
    }
    // assists I7b: with a drive_modes part whose default mode sets launch_control_default, that is the channel default
    // (like the TC channel); a not-fitted P8 two-step keeps default 0 (it ignores the channel anyway).
    if (has_launch_control && !launch_not_fitted) launch_default = ps::vehicle::launch_control_default_channel_value(launch_default, desc.drive_modes);
    if (has_launch_control) channel_indices.assist_launch_control = ensure_control_channel_index(channel_prefix + launch_channel_name, launch_default);
    // ABS / ESC: registered only on a car whose assist unit carries the block (the channel list is unchanged for every
    // other car). Defaults: the default drive mode's abs_default / esc_default, else the block's own default.
    if (desc.assist_unit && desc.assist_unit->abs) {
        channel_indices.assist_abs = ensure_control_channel_index(channel_prefix + "assist.abs",
                                                                  ps::vehicle::abs_default_channel_value(*desc.assist_unit->abs, desc.drive_modes));
    }
    if (desc.assist_unit && desc.assist_unit->esc) {
        channel_indices.assist_esc = ensure_control_channel_index(channel_prefix + "assist.esc",
                                                                  ps::vehicle::esc_default_channel_value(*desc.assist_unit->esc, desc.drive_modes));
    }

    // PHYS-033 P4 I4 C3 (plan section 2.4): resolve every control channel
    // this vehicle's powertrain components declare, same prefix convention
    // as every channel above - {} for every shipped vehicle today (no
    // component declares one yet). test_only_extra_channels (see world.h's
    // own comment) is the test-only path to a non-empty list before a real
    // declaring component exists.
    std::vector<drivetrain::ChannelDecl> test_only_extra;
    test_only_extra.reserve(test_only_extra_channels.size());
    for (const auto& [name, default_value] : test_only_extra_channels) test_only_extra.push_back({name, default_value});
    const std::vector<drivetrain::ChannelDecl> declared_component_channels =
        drivetrain::Powertrain::declare_control_channels(desc.powertrain, test_only_extra);
    channel_indices.component.reserve(declared_component_channels.size());
    for (const drivetrain::ChannelDecl& decl : declared_component_channels) {
        channel_indices.component.push_back(ensure_control_channel_index(channel_prefix + decl.name, decl.default_value));
    }

    // Parking-slope creep fix (defect D2): opt the chassis body in to
    // IRigidBackend::set_integrate_sub_threshold_rotation - see that
    // method's own comment (i_rigid_backend.h) and CLAUDE.md's "Jolt
    // backend" section. Every other body (ground, terrain, non-vehicle
    // bodies) stays opted out, unchanged from before this fix existed.
    backend_->set_integrate_sub_threshold_rotation(chassis, true);

    const VehicleId id = vehicle_system_->create(desc, chassis, surface_table_.get(), channel_indices);
    vehicle_air_.resize(vehicle_system_->count()); // weather plan W4; default (valid == false) until the environment fills it
    // PLAN.md P3 scope item 7: register this vehicle's per-wheel telemetry
    // channels now (setup time, not a hot path) so the CSV header names
    // every wheel by its vehicle index and the wheel's own name (e.g.
    // "vehicle0.front_left.slip_ratio") before the first step() records a
    // row - VehicleId.index doubles as wheel_telemetry_channels_'s outer
    // index (vehicles are never destroyed, same invariant
    // articulation_contacts_ relies on above).
    if (wheel_telemetry_channels_.size() <= id.index) wheel_telemetry_channels_.resize(id.index + 1);
    std::vector<WheelTelemetryChannels>& channels = wheel_telemetry_channels_[id.index];
    const std::size_t n = vehicle_system_->at(id).wheel_count();
    channels.resize(n);
    for (std::size_t w = 0; w < n; ++w) {
        const std::string prefix = "vehicle" + std::to_string(id.index) + "." + vehicle_system_->at(id).wheel_name(w) + ".";
        channels[w].slip_ratio = telemetry_->register_channel(prefix + "slip_ratio");
        channels[w].slip_angle = telemetry_->register_channel(prefix + "slip_angle");
        channels[w].fx = telemetry_->register_channel(prefix + "fx");
        channels[w].fy = telemetry_->register_channel(prefix + "fy");
        channels[w].fz = telemetry_->register_channel(prefix + "fz");
        channels[w].surface = telemetry_->register_channel(prefix + "surface");
        channels[w].omega = telemetry_->register_channel(prefix + "omega"); // PHYS-033 P4 I2 commit 5b
    }

    // PHYS-033 I1 W1 (section 3.5): right after this vehicle's wheel
    // channels, so World::step's row-building loop below can push these two
    // values immediately after that vehicle's wheel values and land on the
    // matching columns purely by push order.
    if (chassis_telemetry_channels_.size() <= id.index) chassis_telemetry_channels_.resize(id.index + 1);
    const std::string chassis_prefix = "vehicle" + std::to_string(id.index) + ".chassis.";
    chassis_telemetry_channels_[id.index].shadow_dv_error = telemetry_->register_channel(chassis_prefix + "shadow_dv_error");
    chassis_telemetry_channels_[id.index].shadow_dw_error = telemetry_->register_channel(chassis_prefix + "shadow_dw_error");

    // PHYS-033 P4 I2 commit 5b (plan section 8): right after this vehicle's
    // chassis channels, same reasoning.
    if (energy_telemetry_channels_.size() <= id.index) energy_telemetry_channels_.resize(id.index + 1);
    EnergyTelemetryChannels& ec = energy_telemetry_channels_[id.index];
    const std::string energy_prefix = "vehicle" + std::to_string(id.index) + ".energy.";
    ec.engine_out = telemetry_->register_channel(energy_prefix + "engine_out");
    ec.wheel_out = telemetry_->register_channel(energy_prefix + "wheel_out");
    ec.tyre_loss = telemetry_->register_channel(energy_prefix + "tyre_loss");
    ec.brake_loss = telemetry_->register_channel(energy_prefix + "brake_loss");
    ec.gear_loss = telemetry_->register_channel(energy_prefix + "gear_loss");
    ec.integrator_loss = telemetry_->register_channel(energy_prefix + "integrator_loss");
    ec.stored = telemetry_->register_channel(energy_prefix + "stored");
    ec.balance_error = telemetry_->register_channel(energy_prefix + "balance_error");
    const std::string powertrain_prefix = "vehicle" + std::to_string(id.index) + ".powertrain.";
    ec.friction_cap_hits = telemetry_->register_channel(powertrain_prefix + "friction_cap_hits");
    ec.nonspd_hits = telemetry_->register_channel(powertrain_prefix + "nonspd_hits");
    ec.lcp_stage3_hits = telemetry_->register_channel(powertrain_prefix + "lcp_stage3_hits");
    ec.gear = telemetry_->register_channel(powertrain_prefix + "gear");
    ec.engine_rpm = telemetry_->register_channel(powertrain_prefix + "engine_rpm");
    ec.clutch_engagement = telemetry_->register_channel(powertrain_prefix + "clutch_engagement");
    ec.clutch_slip = telemetry_->register_channel(powertrain_prefix + "clutch_slip");
    ec.clutch_torque = telemetry_->register_channel(powertrain_prefix + "clutch_torque");
    ec.shift_phase = telemetry_->register_channel(powertrain_prefix + "shift_phase");
    ec.engine_state = telemetry_->register_channel(powertrain_prefix + "engine_state");

    // PHYS-033 P4 I4 C8 (plan section 3.1): one <diff id>.bias/.locked
    // channel pair per differential that is NOT plain open (kind != Open,
    // or lockable - a lockable Open diff still needs its own "am I
    // currently locked" telemetry even though its kind reads Open),
    // registered in desc.powertrain.components array order - the same
    // order Powertrain builds diffs_/PowertrainSnapshot::diffs in, so
    // diff_index here matches PowertrainSnapshot::diffs[diff_index]. A
    // plain-open-only vehicle (both shipped car_sedan/tractor diffs today)
    // registers nothing here, so its CSV header stays byte-identical to
    // before this existed.
    if (diff_telemetry_channels_.size() <= id.index) diff_telemetry_channels_.resize(id.index + 1);
    std::vector<DiffTelemetryChannels>& diff_channels = diff_telemetry_channels_[id.index];
    diff_channels.clear();
    std::size_t diff_component_index = 0;
    for (const drivetrain::ComponentDesc& component : desc.powertrain.components) {
        const auto* diff_desc = std::get_if<drivetrain::DifferentialDesc>(&component.params);
        if (diff_desc != nullptr) {
            const bool plain_open = diff_desc->kind == drivetrain::DifferentialKind::Open && !diff_desc->lockable;
            if (!plain_open) {
                const std::string diff_prefix = "vehicle" + std::to_string(id.index) + "." + component.id + ".";
                DiffTelemetryChannels dc;
                dc.diff_index = diff_component_index;
                dc.bias_channel = telemetry_->register_channel(diff_prefix + "bias");
                dc.locked_channel = telemetry_->register_channel(diff_prefix + "locked");
                diff_channels.push_back(dc);
            }
            ++diff_component_index;
        }
    }

    // PHYS-033 P4 I7a C5 (plan section 5), broadened C5d: vehicle<i>.fuel.
    // {flow_g_s, energy_in, heat}, registered for a vehicle whose engine has
    // ANY fuel model - a calibrated fuel_map OR the crude no-map estimate
    // (drivetrain::has_fuel_model) - right after this vehicle's diff-
    // telemetry channels. A vehicle with neither registers nothing here, so
    // its CSV header stays byte-identical (true of every vehicle shipped
    // before C5; car_sedan/tractor/etc. gain this block once their engine
    // JSON files gain a "fuels" list in C5d). C6 adds tank_kg for a tank.
    if (fuel_telemetry_channels_.size() <= id.index) fuel_telemetry_channels_.resize(id.index + 1);
    std::vector<FuelTelemetryChannels>& fuel_channels = fuel_telemetry_channels_[id.index];
    fuel_channels.clear();
    for (const drivetrain::ComponentDesc& component : desc.powertrain.components) {
        const auto* engine_desc = std::get_if<drivetrain::TorqueMapEngineDesc>(&component.params);
        const auto* sim_engine_desc = std::get_if<drivetrain::SimulatedEngineDesc>(&component.params); // I7b C12: always fuelled
        if ((engine_desc != nullptr && drivetrain::has_fuel_model(*engine_desc)) ||
            (sim_engine_desc != nullptr && drivetrain::has_fuel_model(*sim_engine_desc))) {
            const std::string fuel_prefix = "vehicle" + std::to_string(id.index) + ".fuel.";
            FuelTelemetryChannels fc;
            fc.flow_g_s = telemetry_->register_channel(fuel_prefix + "flow_g_s");
            fc.energy_in = telemetry_->register_channel(fuel_prefix + "energy_in");
            fc.heat = telemetry_->register_channel(fuel_prefix + "heat");
            // PHYS-033 P4 I7a C6: vehicle<i>.fuel.tank_kg, only with a tank
            // (a tank implies a fuel model - the loader and Powertrain both
            // reject a tank feeding an engine without one).
            if (vehicle_system_->at(id).has_fuel_tank()) {
                fc.has_tank = true;
                fc.tank_kg = telemetry_->register_channel(fuel_prefix + "tank_kg");
            }
            fuel_channels.push_back(fc);
            break; // at most one engine per vehicle
        }
    }

    // PHYS-033 P4 I7c C16b: vehicle<i>.engine.maps_hash_lo32, right after
    // this vehicle's fuel-telemetry channels - registered only for a
    // simulated engine (sim_engine_desc above, re-scanned here since it was
    // local to the fuel loop above), never a TorqueMapEngine. A vehicle
    // without a simulated engine gets an empty inner vector, so its CSV
    // header stays byte-identical to before this existed.
    if (engine_maps_hash_channels_.size() <= id.index) engine_maps_hash_channels_.resize(id.index + 1);
    std::vector<EngineMapsTelemetryChannels>& engine_maps_channels = engine_maps_hash_channels_[id.index];
    engine_maps_channels.clear();
    for (const drivetrain::ComponentDesc& component : desc.powertrain.components) {
        if (std::get_if<drivetrain::SimulatedEngineDesc>(&component.params) != nullptr) {
            const std::string engine_prefix = "vehicle" + std::to_string(id.index) + ".engine.";
            EngineMapsTelemetryChannels mc;
            mc.maps_hash_lo32 = telemetry_->register_channel(engine_prefix + "maps_hash_lo32");
            engine_maps_channels.push_back(mc);
            break; // at most one engine per vehicle
        }
    }

    // N2O plan N1: vehicle<i>.n2o.{flow_g_s,bottle_kg,bottle_bar,active}, only for a vehicle with a nitrous kit
    // (an empty inner vector keeps every kit-less CSV header byte-identical).
    if (n2o_telemetry_channels_.size() <= id.index) n2o_telemetry_channels_.resize(id.index + 1);
    std::vector<N2oTelemetryChannels>& n2o_channels = n2o_telemetry_channels_[id.index];
    n2o_channels.clear();
    if (vehicle_system_->at(id).has_nitrous()) {
        const std::string n2o_prefix = "vehicle" + std::to_string(id.index) + ".n2o.";
        N2oTelemetryChannels nc;
        nc.flow_g_s = telemetry_->register_channel(n2o_prefix + "flow_g_s");
        nc.bottle_kg = telemetry_->register_channel(n2o_prefix + "bottle_kg");
        nc.bottle_bar = telemetry_->register_channel(n2o_prefix + "bottle_bar");
        nc.active = telemetry_->register_channel(n2o_prefix + "active");
        n2o_channels.push_back(nc);
    }

    // Engine thermal plan T1: vehicle<i>.thermal.*, only for a vehicle with a cooling_system (an empty inner
    // vector keeps every other CSV header byte-identical).
    if (thermal_telemetry_channels_.size() <= id.index) thermal_telemetry_channels_.resize(id.index + 1);
    std::vector<ThermalTelemetryChannels>& thermal_channels = thermal_telemetry_channels_[id.index];
    thermal_channels.clear();
    if (vehicle_system_->at(id).has_thermal()) {
        const std::string th_prefix = "vehicle" + std::to_string(id.index) + ".thermal.";
        ThermalTelemetryChannels tc;
        tc.coolant_c = telemetry_->register_channel(th_prefix + "coolant_c");
        tc.oil_c = telemetry_->register_channel(th_prefix + "oil_c");
        tc.thermostat = telemetry_->register_channel(th_prefix + "thermostat");
        tc.fan = telemetry_->register_channel(th_prefix + "fan");
        tc.q_wall_w = telemetry_->register_channel(th_prefix + "q_wall_w");
        tc.q_friction_w = telemetry_->register_channel(th_prefix + "q_friction_w");
        tc.q_radiator_w = telemetry_->register_channel(th_prefix + "q_radiator_w");
        tc.friction_scale = telemetry_->register_channel(th_prefix + "friction_scale");
        thermal_channels.push_back(tc);
    }

    return id;
}

std::size_t World::vehicle_wheel_count(VehicleId id) const {
    return vehicle_system_->at(id).wheel_count();
}

const vehicle::WheelTelemetry& World::wheel_telemetry(VehicleId id, std::size_t wheel_index) const {
    return vehicle_system_->at(id).wheel_telemetry(wheel_index);
}

std::size_t World::vehicle_substep_telemetry_count(VehicleId id) const {
    return vehicle_system_->at(id).substep_telemetry_count();
}

const vehicle::WheelTelemetry& World::wheel_substep_telemetry(VehicleId id, std::size_t substep, std::size_t wheel_index) const {
    return vehicle_system_->at(id).substep_wheel_telemetry(substep, wheel_index);
}

const std::string& World::wheel_name(VehicleId id, std::size_t wheel_index) const {
    return vehicle_system_->at(id).wheel_name(wheel_index);
}

vehicle::WheelState World::wheel_state(VehicleId id, std::size_t wheel_index) const {
    return vehicle_system_->at(id).wheel_state(wheel_index);
}

bool World::wheel_is_driven(VehicleId id, std::size_t wheel_index) const {
    return vehicle_system_->at(id).wheel_is_driven(wheel_index);
}

real World::wheel_omega(VehicleId id, std::size_t wheel_index) const {
    return vehicle_system_->at(id).wheel_omega(wheel_index);
}

const drivetrain::PowertrainEnergy& World::powertrain_energy(VehicleId id) const {
    return vehicle_system_->at(id).powertrain_energy();
}

real World::fuel_tank_mass(VehicleId id) const {
    return vehicle_system_->at(id).fuel_tank_mass();
}

void World::set_fuel_tank_mass(VehicleId id, real kg) {
    vehicle_system_->at(id).request_fuel_tank_mass(kg);
}

real World::n2o_bottle_mass(VehicleId id) const {
    return vehicle_system_->at(id).n2o_bottle_mass();
}

void World::set_n2o_bottle_mass(VehicleId id, real kg) {
    vehicle_system_->at(id).request_n2o_bottle_mass(kg);
}

drivetrain::EngineThermalState World::engine_thermal_state(VehicleId id) const {
    const vehicle::VehicleInstance& v = vehicle_system_->at(id);
    if (!v.has_thermal()) throw std::runtime_error("engine_thermal_state: this vehicle has no cooling_system");
    drivetrain::EngineThermalState s;
    s.coolant_c = v.thermal()->engine_temperature_k() - drivetrain::kCelsiusToKelvin;
    s.oil_c = v.thermal()->oil_temperature_k() - drivetrain::kCelsiusToKelvin;
    return s;
}

void World::set_engine_thermal_state(VehicleId id, const drivetrain::EngineThermalState& state) {
    vehicle_system_->at(id).request_thermal_state(state);
}

void World::replace_turbo_configuration(VehicleId id,std::string_view component_id,const drivetrain::TurboPairDesc& desc) {
    if(!vehicle_system_->contains(id)) throw std::runtime_error("replace_turbo_configuration: unknown vehicle");
    vehicle_system_->at(id).replace_turbo_configuration(component_id,desc);
}

drivetrain::PreparedTurboEnvelope World::prepare_turbo_envelope(VehicleId id,std::string_view component_id,const drivetrain::TurboPairDesc& desc) const {
    if(!vehicle_system_->contains(id)) throw std::runtime_error("prepare_turbo_envelope: unknown vehicle");
    return vehicle_system_->at(id).prepare_turbo_envelope(component_id,desc);
}

void World::replace_turbo_configuration(VehicleId id,std::string_view component_id,const drivetrain::PreparedTurboEnvelope& prepared) {
    if(!vehicle_system_->contains(id)) throw std::runtime_error("replace_turbo_configuration: unknown vehicle");
    vehicle_system_->at(id).replace_turbo_configuration(component_id,prepared.turbo,&prepared);
}

void World::replace_engine_maps(VehicleId id, std::string_view component_id, drivetrain::EngineMapsPtr new_maps) {
    // Documented (world.h) to throw for an unknown vehicle id; System::at is a
    // plain operator[], so check here rather than index out of bounds.
    if (!vehicle_system_->contains(id)) throw std::runtime_error("replace_engine_maps: unknown vehicle id");
    vehicle_system_->at(id).request_replace_engine_maps(component_id, std::move(new_maps));
}

const std::vector<real>& World::vehicle_component_channel_values(VehicleId id) const {
    return vehicle_system_->at(id).component_channel_values();
}

const drivetrain::PowertrainSnapshot& World::powertrain_state(VehicleId id) const {
    return vehicle_system_->at(id).powertrain_state();
}

drivetrain::PowertrainDebug World::powertrain_debug(VehicleId id) const {
    return vehicle_system_->at(id).powertrain_debug();
}

void World::reset_vehicle(VehicleId id, const vehicle::VehicleResetOptions& opts) {
    vehicle_system_->at(id).reset(opts, *backend_);
}

real World::vehicle_shadow_dv_error(VehicleId id) const {
    return vehicle_system_->at(id).shadow_dv_error();
}

real World::vehicle_shadow_dw_error(VehicleId id) const {
    return vehicle_system_->at(id).shadow_dw_error();
}

void World::set_terrain_source(std::shared_ptr<const terrain::ITerrainSource> source, const terrain::TerrainConfig& config) {
    terrain_manager_ = std::make_unique<terrain::TileManager>(std::move(source), config, *backend_);
    terrain_manager_->precompute_suppression(*job_system_); // PHYS-021 Task B: load time, off the tick path
    // Terrain prefetch ring: after the precompute (its enqueue rule relies on
    // it); a no-op unless config enables it and the source supports it.
    terrain_manager_->start_prefetch(*job_system_);
}

BodyId World::create_body(const BodyDesc& desc) {
    const BodyId id = backend_->create_body(desc);
    if (terrain_manager_ && desc.motion == BodyMotionType::Static) terrain_manager_->on_static_geometry_changed();
    return id;
}

BodyId World::create_body(const BodyDesc& desc, const ShapeHandle& shape) {
    const BodyId id = backend_->create_body(desc, shape);
    if (terrain_manager_ && desc.motion == BodyMotionType::Static) terrain_manager_->on_static_geometry_changed();
    return id;
}

void World::destroy_body(BodyId id) {
    const bool was_static = terrain_manager_ && backend_->is_static(id);
    backend_->destroy_body(id);
    if (was_static) terrain_manager_->on_static_geometry_changed();
}

void World::set_terrain_interest_point(std::uint64_t id, const Vec3& world_xy, real stream_radius_m) {
    if (terrain_manager_) terrain_manager_->set_interest_point(id, world_xy, stream_radius_m);
}

void World::remove_terrain_interest_point(std::uint64_t id) {
    if (terrain_manager_) terrain_manager_->remove_interest_point(id);
}

std::size_t World::terrain_resident_tile_count() const {
    return terrain_manager_ ? terrain_manager_->resident_tile_count() : 0;
}

std::size_t World::terrain_starved_tile_count() const {
    return terrain_manager_ ? terrain_manager_->starved_tile_count() : 0;
}

std::size_t World::terrain_precomputed_tile_count() const {
    return terrain_manager_ ? terrain_manager_->precomputed_tile_count() : 0;
}

std::size_t World::terrain_relief_overflow_count() const {
    return terrain_manager_ ? terrain_manager_->relief_overflow_count() : 0;
}

terrain::PrefetchStats World::terrain_prefetch_stats() const {
    return terrain_manager_ ? terrain_manager_->prefetch_stats() : terrain::PrefetchStats{};
}

double World::last_stage_ms(const std::string& stage) const {
    const std::vector<std::string>& names = pipeline_->stage_names();
    for (std::size_t i = 0; i < names.size() && i < last_stage_ms_.size(); ++i) {
        if (names[i] == stage) return last_stage_ms_[i];
    }
    return -1.0;
}

} // namespace ps
