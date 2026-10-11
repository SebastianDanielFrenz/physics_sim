# Handoff: live turbo configuration and simulated engine audio

## Completed

The hypercar now uses a simulated One:1-inspired V8 with separate twin-turbo hardware and default configuration files:

- `data/turbos/hyper_turbo_approx.json`
- `data/turbo_configurations/hyper_twin_default.json`
- `data/engines/hyper_v8_one1_audio.json`

Turbo state includes rotor inertia, compressor/turbine power, boost, charge temperature, wastegate behavior, exhaust back-pressure, and shaft coast-down. Generated turbo maps are validated and cached.

Godot can request a live replacement between physics ticks:

```gdscript
simulation.request_vehicle_turbo_configuration("car", absolute_config_path)
simulation.get_turbo_configuration_status()
simulation.get_engine_audio_bank_status()
```

An empty path restores the default.

## Audio banks

Engine banks are generated from the simulated combustion/pipe renderer. They are keyed by the engine definition, referenced turbo/configuration dependencies, and generator settings. Missing or corrupt banks generate on a background worker while the current renderer continues playing; the completed bank crossfades in over 120 ms.

Banks contain five RPM points and two manifold-load planes with all source channels. Since PSAB0002 each plane is a loop of whole engine cycles with a known crank angle at frame 0, played back at the live voice's crank angle (crank-synchronous; see `audio_bank.h`); old PSAB0001 cache files are a silent miss and regenerate. New requests supersede queued requests. Obsolete generation is cancelled. Failed requests keep the previous audio and simulation running.

Relevant files:

- `audio/include/ps_audio/audio_bank.h`
- `audio/src/audio_bank.cpp`
- `audio/include/ps_audio/audio_bank_reference.h` (old sample-time playback, measurement/test reference only)
- `engine_tools/src/audio_bank_prep.cpp` (the bank baker; was tools/common/engine_bank_generator.cpp until engine garage P6)
- `adapters/godot/src/ps_simulation.cpp`
- `audio/src/voice.cpp`

Caches are under `out/godot_engine_cache/audio` and `out/godot_engine_cache/turbo`.

## Verification

- Godot adapter rebuild succeeded.
- Turbo/audio-bank tests: 20 assertions passed.
- Existing mean-value engine regression: 75,841 assertions passed in 11 cases.
- WASAPI run maintained about 240 Hz while generating the initial bank in about 27 seconds, then swapped it live.
- A second override applied successfully and reported `Active`.
- A missing override kept the simulation running and reported `Failed; keeping previous audio`.
- Damaged cache files are rejected by checksum validation.

Logs: `out/one1_reference/bank_probe.log` and `out/one1_reference/bank_swap_probe.log`.

## Limits

The turbo model uses authored quasi-steady curves and simplified thermal assumptions. It does not model OEM compressor maps, surge, individual turbo imbalance, or a full resolved gas network. Geometry is based on published dimensions; firing order, pipe lengths, and calibration values are assumptions.

The reported 200 ms end-to-end latency was not independently timed. Turbo whine and blow-off remain disabled. Successful banks remain retained for the voice lifetime, so repeated swaps increase memory use until the voice is destroyed.

## Next steps

1. Listen to the generated bank in Godot and tune source gain and tone.
2. Add progress UI if cache generation should be visible.
3. Retire superseded banks after crossfade if memory matters.
4. Stage only turbo/audio/cabin changes for a focused commit; preserve unrelated SIMD, dashboard, Atmos, model, and sanitizer work separately.

## Final commit scope, 2026-10-03

The owner requested all pending project changes: engine/turbos, generated maps and audio banks, threaded/SIMD rendering, cockpit/model/dashboard, tyre damping, reusable spatial backend, diagnostics and authoring assets. Native checks passed (76,616 assertions / 27 cases), adapter rebuilt and Godot editor import passed. Runtime live engine and bank-swap checks passed. The local nested backup checkout and generated diagnostic DLL/CSV remain ignored. Verification artifacts are under S:/claude_code/physics_sim/out/hyper_commit_verify. Broad sanitizer checks are queued overnight.

## Shutdown observation, 2026-10-03

The first cold live-swap run completed generation, override and invalid-file fallback, then exited with Windows access violation 0xC0000005 during shutdown. A debugger run and six subsequent WASAPI live-swap/explicit-stop runs all exited zero, so the fault could not be reproduced or located; it is not claimed fixed. Dashboard/fuel and cockpit surface audio checks passed. Focused spatial lifecycle passed with endpoint rejection 0x80004001; downstream Atmos/headset output is not certified. Follow up with overnight sanitizer coverage and cold-cache shutdown reproduction. Logs: out/hyper_commit_verify/live_swap.log, shutdown_stack.log, shutdown_repeat_1.log through shutdown_repeat_6.log, dashboard.log, surface_audio.log, spatial_audio.log.
