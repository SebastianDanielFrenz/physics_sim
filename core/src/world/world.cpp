#include "ps/world/world.h"

#include "../articulation/contact.h" // internal; PLAN.md P1.5b
#include "../articulation/system.h" // internal; World owns it via the forward-declared pointer (world.h)
#include "../terrain/tile_manager.h" // internal; World owns it via the forward-declared pointer (world.h); PLAN.md M1

#include <algorithm>
#include <chrono>
#include <string>

namespace ps {

World::World(const WorldConfig& config) : config_(config), dt_(real(1.0) / config.tick_rate_hz) {
    job_system_ = std::make_unique<jobs::JobSystem>(config.job_workers, config.job_queue_capacity);
    articulation_system_ = std::make_unique<articulation::System>();
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
    const auto freeze = pipeline_->add_stage("freeze_tick_start_state", timed(slot++, [] {}));
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
    if (articulation_system_->empty()) return;
    articulation::InstanceConfig cfg;
    cfg.gravity = config_.gravity;
    cfg.substep_rate_hz = config_.substep_rate_hz;
    articulation_system_->step_all(*job_system_, cfg, dt_, config_.articulation_grain);
    articulation_system_->clear_external_wrenches();
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
    snapshots_->publish();
}

void World::step() {
    job_system_->reset_worker_stats();
    const auto tick_t0 = std::chrono::steady_clock::now();

    pipeline_->run(*job_system_);

    const auto tick_t1 = std::chrono::steady_clock::now();
    const double tick_ns = std::max(
        1.0, std::chrono::duration<double, std::nano>(tick_t1 - tick_t0).count());

    std::vector<real> row;
    row.reserve(stage_timing_channels_.size() + worker_utilization_channels_.size() + 1);
    for (double ms : last_stage_ms_) row.push_back(real(ms));
    for (int w = 0; w < job_system_->worker_count(); ++w) {
        const double busy_ns = static_cast<double>(job_system_->worker_busy_ns(w));
        row.push_back(real(busy_ns / tick_ns));
    }
    row.push_back(real(job_system_->queue_full_fallback_count())); // cumulative, P1.5b review item
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
    return articulation_system_->fold_state_hash(h);
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

void World::set_terrain_source(std::shared_ptr<const terrain::ITerrainSource> source, const terrain::TerrainConfig& config) {
    terrain_manager_ = std::make_unique<terrain::TileManager>(std::move(source), config, *backend_);
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

} // namespace ps
