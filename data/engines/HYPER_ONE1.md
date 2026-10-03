# One:1-inspired simulated hypercar engine

`car_hyper` now drives `hyper_v8_one1_audio.json`: a mean-value combustion
engine coupled to a reduced twin-turbo rotor/charge/exhaust model. Engine
fuel flow, torque, boost and sound use this configuration. Its E5 fuel matches
the vehicle tank. `hyper_v8_torque.json` remains an alternative static envelope.

Published geometry is eight cylinders, 92 mm bore, 95.25 mm stroke, 9:1
compression, four valves per cylinder and an 8250 rpm limit. The rounded
geometry integrates to approximately 5.066 L. The published One:1 targets are
1371 Nm at 6000 rpm and 1 MW at 7500 rpm:
https://www.koenigsegg.com/technical-specifications-one1

Firing order [1,5,4,8,6,3,7,2], bank layout, cams, pipe lengths, VE,
efficiency, lambda, ignition, turbo inertia and performance curves are authored
assumptions. They are not measured manufacturer data. The present held-speed
regression produces approximately 1363 Nm / 856 kW at 6000 rpm and 1049 Nm /
824 kW at 7500 rpm. This is an approximation, not the published rated curve.

## Configuration and live replacement

The engine's `default_turbo_configuration` points to
`data/turbo_configurations/hyper_twin_default.json`. That configuration names
`data/turbos/hyper_turbo_approx.json`, turbine count, boost target and
intercooler effectiveness. Hardware quantities such as shaft inertia, reference
speed, flow capacity, turbine area, efficiencies and shaft losses live in the
separate turbo file; per-turbo quantities are multiplied by turbine count.

Rotor energy integrates turbine power minus compressor power and shaft loss.
Compressor pressure/temperature feed the manifold and acoustic intake supply;
exhaust back-pressure affects pumping loss. Wastegate PID control responds to charge pressure. The symmetric pair uses authored quasi-steady head curves,
assumed exhaust temperature and a relaxed charge pressure. Surge, individual
bank imbalance and a resolved compressor/turbine gas network are not modeled.

Godot exposes:

```gdscript
simulation.request_vehicle_turbo_configuration("car", absolute_config_path)
# Empty path restores the engine default.
simulation.get_turbo_configuration_status()
simulation.get_engine_audio_bank_status()
```

Requests return immediately. Derived turbo maps are prepared on a background
worker and applied between physics ticks, preserving manifold state and shaft
speed. Queued requests for one vehicle are coalesced; newer requests cancel
obsolete audio generation. Overrides survive stopping and restarting simulation.

## Simulated audio banks

Audio banks are generated from the combustion/pipe renderer, not recordings.
The cache key covers engine JSON, referenced fuel/installation/default turbo
files, an override and its hardware, and generator settings/version. Derived
data resides in `out/godot_engine_cache/turbo` and `audio`. Missing or corrupt
banks are regenerated while driving; generation does not run on the physics or
audio thread. Existing audio remains active until a complete bank is ready.

Each bank contains five RPM knots and two manifold-load planes, all radiating
source channels, with overlapping loop boundaries. Playback interpolates load
and RPM, with pitch following current engine speed. A 120 ms crossfade replaces
the old bank. Ignition/limiter/DFCO cuts blend back to the continuously running
acoustic renderer. On first launch that live renderer supplies the fallback.
Failed generation keeps the previous audio; closing the demo cancels generation.
Installed banks are retained until the voice is destroyed, avoiding large frees
on its audio thread. This currently costs memory for each successful live swap.
Turbo whine/blow-off sound remains disabled; a rejected synthetic layer is not
reintroduced by this configuration work.

## Verification, 2026-10-02

- Turbo/audio-bank focused tests: 20 assertions passed, including boost/coast,
  configuration replacement, cache corruption and continuous loop interpolation.
- Existing mean-value engine regressions: 75,841 assertions passed in 11 cases.
- Godot adapter rebuilt. WASAPI headless run generated the first bank in about
  27 seconds while simulation maintained approximately 240 Hz, then swapped it.
- Live swap/cache/error-path logs: `out/one1_reference/bank*probe.log`.
- Cabin tyres receive an 18 dB reduction and 900 Hz low-pass, smoothly blended
  when switching cameras; exterior rolling audio is preserved.
- Reduced live queues and faster acoustic intake control address avoidable
  buffering. End-to-end hardware latency and subjective sound need listening;
  the reported 200 ms gap has not been independently timed.
## Commit verification, 2026-10-03

The staged source built independently under out/hyper_commit_verify. Focused engine, turbo, audio-bank, SIMD and thread checks passed: 76,367 assertions in 22 cases. Physics maps are now generated from the engine cycle (the hand-authored maps were removed). Current WOT held-speed results are 1363 Nm at 6000 rpm and 824 kW at 7500 rpm; these are model results rather than a fitted manufacturer curve. Long sanitizer checks remain queued for overnight verification.

Final complete staged-source native checks: 76,616 assertions in 27 cases passed, including boosted map generation, cache-key coverage, low-speed tyre grip, and serial/threaded/SIMD equality. Godot adapter rebuilt; editor import passed. Full sanitizer suites are pending overnight, not declared passed.

## Shutdown observation, 2026-10-03

The first cold live-swap run completed generation, override and invalid-file fallback, then exited with Windows access violation 0xC0000005 during shutdown. A debugger run and six subsequent WASAPI live-swap/explicit-stop runs all exited zero, so the fault could not be reproduced or located; it is not claimed fixed. Dashboard/fuel and cockpit surface audio checks passed. Focused spatial lifecycle passed with endpoint rejection 0x80004001; downstream Atmos/headset output is not certified. Follow up with overnight sanitizer coverage and cold-cache shutdown reproduction. Logs: out/hyper_commit_verify/live_swap.log, shutdown_stack.log, shutdown_repeat_1.log through shutdown_repeat_6.log, dashboard.log, surface_audio.log, spatial_audio.log.
