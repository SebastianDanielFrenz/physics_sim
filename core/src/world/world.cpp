#include "ps/world/world.h"

#include <algorithm>
#include <chrono>
#include <string>

namespace ps {

World::World(const WorldConfig& config) : config_(config), dt_(real(1.0) / config.tick_rate_hz) {
    job_system_ = std::make_unique<jobs::JobSystem>(config.job_workers);
    backend_ = create_jolt_backend(*job_system_, config.gravity, config.max_bodies);
    snapshots_ = std::make_unique<SnapshotTripleBuffer>();
    telemetry_ = std::make_unique<io::TelemetryRegistry>();

    build_pipeline();

    for (const auto& name : pipeline_->stage_names()) {
        stage_timing_channels_.push_back(telemetry_->register_channel("stage." + name + ".ms"));
    }
    for (int w = 0; w < job_system_->worker_count(); ++w) {
        worker_utilization_channels_.push_back(telemetry_->register_channel("jobs.worker" + std::to_string(w) + ".utilization"));
    }
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
    const auto islands = pipeline_->add_stage("vehicle_islands_substep", timed(slot++, [] {}), {queries});
    const auto force_el = pipeline_->add_stage("force_elements", timed(slot++, [this] { stage_force_elements(); }), {islands});
    const auto gather = pipeline_->add_stage("gather", timed(slot++, [this] { stage_gather(); }), {force_el});
    const auto solver_seam = pipeline_->add_stage(
        "solver_seam", timed(slot++, [this] { if (between_hook_) between_hook_(*this); }), {gather});
    const auto backend_step = pipeline_->add_stage("backend_step", timed(slot++, [this] { stage_backend_step(); }), {solver_seam});
    const auto terrain = pipeline_->add_stage("terrain_tiles", timed(slot++, [] {}), {backend_step});
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

void World::stage_gather() {
    // Ordered gather, sequential in fixed element-id order (PLAN.md D8:
    // "never sum floats in completion order"): result is independent of
    // which worker computed which wrench or in what order.
    for (std::size_t i = 0; i < force_elements_.size(); ++i) {
        const IForceElement* el = force_elements_[i];
        if (!el) continue;
        backend_->apply_wrench(el->frame().body(), force_element_wrenches_[i]);
    }
}

void World::stage_backend_step() {
    backend_->step(dt_);
    last_contact_events_ = backend_->drain_contact_events();
}

void World::stage_snapshot() {
    WorldSnapshot& snap = snapshots_->write_slot();
    snap.tick = tick_;
    snap.sim_time = sim_time_;
    snap.bodies.clear();
    snap.deformation_payload.clear(); // seam only (PLAN.md section 6); unused in P1

    for (BodyId id : backend_->live_body_ids()) {
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
    row.reserve(stage_timing_channels_.size() + worker_utilization_channels_.size());
    for (double ms : last_stage_ms_) row.push_back(real(ms));
    for (int w = 0; w < job_system_->worker_count(); ++w) {
        const double busy_ns = static_cast<double>(job_system_->worker_busy_ns(w));
        row.push_back(real(busy_ns / tick_ns));
    }
    telemetry_->record_row(tick_, sim_time_, std::move(row));

    ++tick_;
    sim_time_ += dt_;
}

std::uint64_t World::state_hash() const {
    std::uint64_t h = 14695981039346656037ULL; // FNV-1a offset basis
    h ^= tick_;
    h *= 1099511628211ULL;
    return backend_->compute_state_hash(h);
}

} // namespace ps
