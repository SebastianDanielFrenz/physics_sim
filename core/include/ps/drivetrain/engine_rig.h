// ps/drivetrain/engine_rig.h - ps_audio A10 (out/audio_plan.md section 6):
// EngineRig, a public, additive, host-tool-facing wrapper around the real
// engine model with no chassis or wheels - "the engine on a dyno". Built for
// tools/ps_engine_audio's offline/live render (the owner's first listening
// test) and reusable as the I7c dyno harness the plan's D2/V2 also asks for.
//
// Loads an engine file (plus its default installation, or a given one, for a
// simulated engine with pipe blocks - ps::io::load_engine_json), builds a
// wheel-less ps::drivetrain::Powertrain whose only shaft is the engine's
// crank, optionally coupled to a LoadDyno (Brake/Viscous/Quadratic; omitted
// = a free rev - vault PHYS-054's A5 finding: a crank-only shaft with no
// dyno is legal, a single-port shaft alone satisfies Powertrain's "every
// non-wheel port must be connected by some shaft" rule). Steps at a fixed
// tick/substep rate like ps::World (240/960 Hz default). Same real engine
// model (idle PI, limiter, starter, fuel cut, and for a simulated engine the
// mean-value manifold/VE/eta_i/spark tables) as a car's - EngineRig adds no
// engine physics of its own. Deterministic; allocation-free per step() once
// built (construction, including any load-time map generation, may
// allocate).
#pragma once

#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/powertrain_desc.h" // LoadDynoDesc
#include "ps/drivetrain/powertrain_state.h" // EngineSoundState, EngineState
#include "ps/io/vehicle_io.h"               // EngineMapOptions

#include <memory>
#include <string>

namespace ps {

class EngineRig {
public:
    struct DynoConfig {
        bool enabled = false;
        // Read at construction time only (mode, inertia_kgm2, torque_nm/
        // viscous_nm_s_rad/quadratic_nm_s2_rad2, axis_local). `scale_channel`
        // is overwritten internally - EngineRig always wires its own single
        // live scale input (set_dyno_torque_scale) through it, so any value
        // set here is ignored.
        drivetrain::LoadDynoDesc desc;
    };

    struct Config {
        DynoConfig dyno; // default: no dyno (free rev)
        real tick_hz = 240.0;
        real substep_hz = 960.0; // must be an integer multiple of tick_hz
        // A real key-off engine does not spin on its own; the caller drives
        // set_starter/set_ignition/set_throttle. Running seeds the crank at
        // idle instead (Powertrain's own bootstrap convention), for a
        // harness that wants to start already spinning.
        drivetrain::EngineState initial_engine_state = drivetrain::EngineState::Off;
    };

    // Throws std::runtime_error on a malformed engine/fuel/installation file
    // or an invalid Config (ps::io::load_engine_json's rejections, plus the
    // wheel-less Powertrain graph's own - see powertrain.h's constructor
    // comment - and a non-integer substep_hz/tick_hz ratio).
    //
    // Split into two overloads (rather than one constructor with `config =
    // Config{}` as a default argument) because a nested class's own default
    // member initializers (DynoConfig::enabled above) cannot be evaluated to
    // build a class-scope default-argument temporary of ANOTHER nested class
    // (Config) while EngineRig itself is still an incomplete type - a clang-cl
    // 19.1.5 hard error ("default member initializer for 'enabled' needed
    // within definition of enclosing class 'EngineRig' outside of member
    // functions"), not merely a style choice. The single-Config overload
    // below still builds a default-constructed Config{}, just out of line in
    // engine_rig.cpp where EngineRig (and Config) are already complete.
    explicit EngineRig(const std::string& engine_path, const io::EngineMapOptions& map_options = io::EngineMapOptions{},
                        const std::string* installation_override_path = nullptr);
    EngineRig(const std::string& engine_path, const io::EngineMapOptions& map_options,
              const std::string* installation_override_path, const Config& config);
    ~EngineRig();
    EngineRig(EngineRig&&) noexcept;
    EngineRig& operator=(EngineRig&&) noexcept;
    EngineRig(const EngineRig&) = delete;
    EngineRig& operator=(const EngineRig&) = delete;

    // Inputs. Held at their last value across step() calls (World::
    // set_control's own convention) - a caller re-issues throttle/starter/
    // ignition every tick it wants them to keep applying.
    void set_throttle(real v);          // [0, 1], the pedal/lever the throttle-command stage reads
    void set_starter(real v);           // >= 0.5 engages the starter (StarterDesc/interlock rules as in a car)
    void set_ignition(real v);          // >= 0.5 = spark/fuel allowed; < 0.5 forces EngineState::Off immediately, from any state (engine_simulated.cpp's state machine checks this before its per-state transitions)
    void set_dyno_torque_scale(real v); // multiplies the configured LoadDyno's coefficient; 1.0 = as configured. No-op without a dyno.

    // Advances exactly one tick (config.substep_hz / config.tick_hz
    // substeps, each 1/config.substep_hz s of sim time). Same tick-0 seeded-
    // rotor convention as the rest of the codebase (vault PHYS-049 F-T0):
    // ledger().balance_error is large at the FIRST step() call (the
    // constructor's reset() seeds rotor_ke without booking it), and closes
    // from the second step() call on.
    void step();

    [[nodiscard]] real sim_time_s() const;
    [[nodiscard]] real engine_omega_rad_s() const;
    [[nodiscard]] real engine_rpm() const;
    [[nodiscard]] drivetrain::EngineState engine_state() const;
    // This tick's EngineSoundState (crank angle, load, p_intake_pa, lambda,
    // spark_advance_deg, fuel_flow_g_s, ...) - exactly what a car's
    // VehicleSnapshot::powertrain.engines[0] carries (drivetrain::
    // fill_engine_sound_state is shared code), so a Voice fed from EngineRig
    // and a Voice fed from a car agree on every field's meaning.
    [[nodiscard]] const drivetrain::EngineSoundState& engine_sound_state() const;
    [[nodiscard]] const drivetrain::PowertrainEnergy& ledger() const;
    [[nodiscard]] bool has_dyno() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ps
