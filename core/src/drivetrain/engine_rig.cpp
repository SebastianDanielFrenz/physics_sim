#include "ps/drivetrain/engine_rig.h"

#include "engine_params_util.h"
#include "engine_sound_state.h"
#include "powertrain.h"
#include "ps/io/engine_thermal_io.h"
#include "ps/vehicle/vehicle_reset.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ps {

namespace {
constexpr real kTwoPi = real(2.0) * real(3.14159265358979323846);
}

struct EngineRig::Impl {
    drivetrain::PowertrainDesc pt_desc;
    drivetrain::Powertrain powertrain;
    bool has_dyno = false;
    real tick_h = real(1.0) / real(240.0);
    int substeps_per_tick = 4;
    real sim_time = 0.0;
    drivetrain::ComponentSubstepCtx ctx;
    std::vector<real> channel_values;
    int dyno_scale_channel_index = -1;
    drivetrain::EngineSoundState sound_state;
    AmbientSample ambient{};
    real ram_speed_mps = 0.0;

    Impl(drivetrain::PowertrainDesc desc, bool dyno_enabled, real tick_hz, real substep_hz, const std::string& name)
        : pt_desc(std::move(desc)),
          powertrain(pt_desc, drivetrain::BrakeSystemDesc{}, std::vector<vehicle::WheelDesc>{},
                     std::vector<drivetrain::ControllerDesc>{}, name),
          has_dyno(dyno_enabled) {
        if (!(tick_hz > real(0.0)) || !(substep_hz > real(0.0))) {
            throw std::runtime_error("EngineRig: tick_hz and substep_hz must be > 0");
        }
        const real ratio = substep_hz / tick_hz;
        const int n = static_cast<int>(ratio + real(0.5));
        if (n < 1 || std::fabs(ratio - real(n)) > real(1e-9)) {
            throw std::runtime_error("EngineRig: substep_hz must be an integer multiple of tick_hz");
        }
        substeps_per_tick = n;
        tick_h = real(1.0) / substep_hz;
        ctx.h = tick_h;

        // Own single live channel: the dyno's scale_channel (if a dyno is
        // configured). No World here, so EngineRig resolves it itself -
        // same declare/allocate-once-then-index-by-position pattern
        // World::create_vehicle uses for VehicleInstance::
        // component_channel_values_ (PLAN.md D8: zero-alloc once built).
        const std::vector<drivetrain::ChannelDecl> decls = drivetrain::Powertrain::declare_control_channels(pt_desc);
        channel_values.resize(decls.size());
        for (std::size_t i = 0; i < decls.size(); ++i) {
            channel_values[i] = decls[i].default_value;
            if (decls[i].name == "dyno_scale") dyno_scale_channel_index = static_cast<int>(i);
        }
        ctx.channels = channel_values.empty() ? nullptr : channel_values.data();
        ctx.channel_count = static_cast<int>(channel_values.size());
    }
};

EngineRig::EngineRig(const std::string& engine_path, const io::EngineMapOptions& map_options,
                      const std::string* installation_override_path)
    : EngineRig(engine_path, map_options, installation_override_path, Config{}) {}

EngineRig::EngineRig(const std::string& engine_path, const io::EngineMapOptions& map_options,
                      const std::string* installation_override_path, const Config& config)
    : EngineRig(io::load_engine_json(engine_path, map_options, installation_override_path), config, &engine_path, 0) {}

EngineRig::EngineRig(const drivetrain::ComponentParams& engine, drivetrain::EngineMapsPtr maps)
    : EngineRig(engine, std::move(maps), Config{}) {}

EngineRig::EngineRig(const drivetrain::ComponentParams& engine, drivetrain::EngineMapsPtr maps, const Config& config)
    : EngineRig(drivetrain::with_engine_maps(engine, maps, "EngineRig"), config, nullptr, 0) {}

EngineRig::EngineRig(drivetrain::ComponentParams engine_params, const Config& config, const std::string* engine_path, int) {
    drivetrain::PowertrainDesc pt;
    pt.components.push_back(drivetrain::ComponentDesc{"engine", std::move(engine_params)});

    drivetrain::ShaftDesc crank_shaft;
    crank_shaft.ports.push_back(drivetrain::PortRef{"engine", "crank", false});
    if (config.dyno.enabled) {
        drivetrain::LoadDynoDesc dyno = config.dyno.desc;
        dyno.scale_channel = "dyno_scale"; // EngineRig's own fixed name (Config::DynoConfig's own header comment)
        pt.components.push_back(drivetrain::ComponentDesc{"dyno", dyno});
        crank_shaft.ports.push_back(drivetrain::PortRef{"dyno", "shaft", false});
    }
    pt.shafts.push_back(crank_shaft);
    if (!config.cooling_system_path.empty()) {
        drivetrain::EngineThermalConfig tc;
        std::string thermal_path = config.engine_thermal_path;
        if (thermal_path.empty() && engine_path != nullptr) thermal_path = io::engine_default_thermal_path(*engine_path);
        if (thermal_path.empty()) {
            throw std::runtime_error(
                engine_path != nullptr
                    ? "EngineRig: a cooling_system needs an engine thermal file, but engine file '" + *engine_path +
                          "' has no 'default_thermal' and Config::engine_thermal_path is empty"
                    : std::string("EngineRig: a cooling_system needs an engine thermal file, but this rig was built from a descriptor "
                                  "(no file, no 'default_thermal') and Config::engine_thermal_path is empty"));
        }
        tc.engine = io::load_engine_thermal_json(thermal_path);
        tc.cooling = io::load_cooling_system_json(config.cooling_system_path);
        tc.start = config.thermal_start;
        pt.thermal = std::move(tc);
    }

    impl_ = std::make_unique<Impl>(std::move(pt), config.dyno.enabled, config.tick_hz, config.substep_hz,
                                   engine_path != nullptr ? *engine_path : std::string("engine"));

    vehicle::VehicleResetOptions ro;
    ro.engine_state = config.initial_engine_state;
    impl_->powertrain.reset(ro, /*hub_speeds=*/{});
    impl_->sound_state = drivetrain::fill_engine_sound_state(impl_->powertrain);
}

EngineRig::~EngineRig() = default;
EngineRig::EngineRig(EngineRig&&) noexcept = default;
EngineRig& EngineRig::operator=(EngineRig&&) noexcept = default;

void EngineRig::set_throttle(real v) { impl_->ctx.throttle = v; }
void EngineRig::set_starter(real v) { impl_->ctx.starter = v; }
void EngineRig::set_ignition(real v) { impl_->ctx.ignition = v; }

void EngineRig::set_dyno_torque_scale(real v) {
    if (impl_->dyno_scale_channel_index >= 0) {
        impl_->channel_values[static_cast<std::size_t>(impl_->dyno_scale_channel_index)] = v;
    }
}

void EngineRig::set_ambient(const AmbientSample& a) {
    impl_->ambient = a; // the thermal model's air
    impl_->ctx.ambient = a; // and the engine's
}
void EngineRig::set_thermal_env(real ambient_k, real ram_speed_mps) {
    impl_->ambient.temperature_k = ambient_k; // the thermal model only; the engine keeps its own ambient (set_ambient)
    impl_->ram_speed_mps = ram_speed_mps;
}
bool EngineRig::has_thermal() const { return impl_->powertrain.has_thermal(); }
const drivetrain::EngineThermalSnapshot& EngineRig::thermal_snapshot() const { return impl_->powertrain.thermal_snapshot(); }
drivetrain::EngineThermalState EngineRig::engine_thermal_state() const {
    const drivetrain::EngineThermal* t = impl_->powertrain.thermal();
    if (t == nullptr) throw std::runtime_error("EngineRig::engine_thermal_state: no cooling_system configured");
    return drivetrain::EngineThermalState{t->engine_temperature_k() - drivetrain::kCelsiusToKelvin,
                                          t->oil_temperature_k() - drivetrain::kCelsiusToKelvin};
}
void EngineRig::set_engine_thermal_state(const drivetrain::EngineThermalState& s) { impl_->powertrain.request_thermal_state(s); }
void EngineRig::set_thermal_friction_scale_override_for_test(real k) { impl_->powertrain.thermal_friction_scale_override_for_test(k); }

void EngineRig::step() {
    drivetrain::Powertrain& pt = impl_->powertrain;
    pt.set_thermal_env(impl_->ambient.temperature_k, impl_->ambient.pressure_pa, impl_->ram_speed_mps);
    pt.begin_tick();
    for (int k = 0; k < impl_->substeps_per_tick; ++k) {
        impl_->ctx.first_substep_of_tick = (k == 0);
        pt.begin_substep();
        pt.pre_solve_components(impl_->ctx);
        pt.solve(impl_->ctx.h);
        pt.post_solve_components(impl_->ctx);
    }
    pt.end_tick();
    impl_->sim_time += impl_->tick_h * real(impl_->substeps_per_tick);
    impl_->sound_state = drivetrain::fill_engine_sound_state(pt);
}

real EngineRig::sim_time_s() const { return impl_->sim_time; }
real EngineRig::engine_omega_rad_s() const { return impl_->powertrain.engine_omega(); }
real EngineRig::engine_rpm() const { return impl_->powertrain.engine_omega() * (real(60.0) / kTwoPi); }
drivetrain::EngineState EngineRig::engine_state() const { return impl_->sound_state.engine_state; }
const drivetrain::EngineSoundState& EngineRig::engine_sound_state() const { return impl_->sound_state; }
const drivetrain::PowertrainEnergy& EngineRig::ledger() const { return impl_->powertrain.ledger(); }
bool EngineRig::has_dyno() const { return impl_->has_dyno; }

} // namespace ps
