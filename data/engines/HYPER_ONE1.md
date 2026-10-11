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

Firing order [1,5,4,8,6,3,7,2], bank layout, cams, ports, pipe lengths,
ECU settings and turbo hardware are authored assumptions, not measured
manufacturer internals. VE, indicated efficiency, spark response, pumping and
friction remain generated from the physical cycle; no torque multiplier or
hand-authored engine-performance map was added.

## Physical calibration, 2026-10-03

The fixed bore, stroke, compression, 8250 rpm limiter and 1.8 bar charge boost
target are unchanged. E5 and the vehicle's 1500 kg chassis are unchanged.
The fresh baseline below subtracts both smooth and Coulomb friction; previous
824 kW / 1363 Nm statements included the 26.854 Nm Coulomb loss and are not net
shaft results. At 7500 rpm the old VE was 0.850, charge temperature 308.49 K,
and exhaust pressure 318.3 kPa; spark efficiency was already 0.993. At 3000 rpm
boost settled at 0.386 bar after 15 seconds: a low-speed turbo equilibrium,
not merely spool lag.

Assumed intake valves grow 36â†’38 mm, exhaust 32â†’34 mm, with 12 mm lift.
Intake duration 250â†’270 degrees delays closing to 86 ABDC, retaining IVO
4 BTDC. Exhaust duration 254â†’264 opens at 80 BBDC and closes at 4 ATDC;
overlap stays eight degrees. High-lift discharge coefficient becomes 0.50
(previously 0.45); runner diameters become 50/46 mm and primaries 46 mm.
The cycle throttle becomes 80 mm to match the runtime throttle. Explicit
7500 and 8000 rpm generation nodes capture the top-end curve.

Per-turbo effective turbine throat 750â†’500 mmÂ² sustains low-rpm boost.
Reference compressor head 3.24â†’4.5 bar and assumed compressor/turbine
efficiencies 0.80 support the delivery; rotor inertia and flow capacity are
unchanged. Intercooler effectiveness 0.85â†’0.95 gives 294.25 K charge air at
ISA. This is optimistic steady cooling: heat soak and cooler pressure loss
are not resolved. The smaller throat raises exhaust pressure at 7500 rpm
to 412.9 kPa; it trades back-pressure for low-speed boost. Pumping remains
about âˆ’180 Nm there. Improved filling (VE 0.986) and cooler charge supply
the power gain. RPM-dependent high-load ignition retard trims the resulting
midrange; it is an explicit ECU calibration, not a global torque scale.

Held-crank dyno: 960 Hz, fresh engine at each point, 15 seconds WOT,
final one-second average. ISA ambient 101325 Pa / 288.15 K. Boost below is
manifold gauge pressure; charge pressure is 281325 Pa (1.8 bar gauge) at all
listed calibrated points. Fuel is actual model E5 mass flow. Power and torque
are **net crank output**, including 26.854 Nm Coulomb friction.

| RPM | Before Nm | Before kW | After Nm | After kW | Boost bar | Fuel g/s |
|---:|---:|---:|---:|---:|---:|---:|
| 3000 | 654.5 | 205.6 | 1230.3 | 386.5 | 1.783 | 28.94 |
| 4000 | 1420.2 | 594.9 | 1273.8 | 533.6 | 1.776 | 40.45 |
| 4500 | 1431.7 | 674.7 | 1338.6 | 630.8 | 1.770 | 48.42 |
| 5000 | 1440.9 | 754.5 | 1376.1 | 720.5 | 1.758 | 56.85 |
| 6000 | 1336.3 | 839.6 | 1371.0 | 861.4 | 1.732 | 72.02 |
| 7000 | 1140.0 | 835.7 | 1323.4 | 970.1 | 1.712 | 81.30 |
| 7500 | 1021.9 | 802.6 | 1273.5 | 1000.2 | 1.703 | 85.31 |
| 8000 | 904.1 | 757.4 | 1184.3 | 992.1 | 1.699 | 86.91 |

Charge ripple in the final window is below 0.02 Pa. This verifies sampled
steady points, not every continuous RPM or transient road condition. The
published anchors are from [Koenigsegg's One:1 specifications](https://www.koenigsegg.com/technical-specifications-one1).
The generator explicitly sets `has_theta_kl=false`: there is no validated
knock constraint. Keeping E5 does **not** establish that a real E5 engine could
safely make this power. Actual cam/port/turbo measurements and combustion/knock
validation are needed before claiming manufacturer fidelity.

Reproduce with the `bench_hyper_calibration` native target and the shipped
engine JSON. The tool prints VE, manifold/charge/exhaust pressures, temperatures,
rotor speed, lambda, spark/MBT, combustion/pumping/friction and fuel diagnostics.
Cycle geometry changes also feed the existing live simulated audio renderer;
content-keyed cached engine/audio maps regenerate through the existing path.
No recorded engine audio replaces it.

Focused staged-source verification: native build passed; `[turbo]` passed
33 assertions, including net power/torque anchors, 3000/8000 rpm delivery,
boost settling, spool (0.140 s to 90% target at 4500 rpm), coast, stopped rotor
energy, and live hardware replacement continuity. The shipped-file dyno
reproduced the table above. No Godot adapter or racing-game DLL was rebuilt
for this data calibration; integration build ownership stays with that chat.

## Configuration and live replacement

The engine's `default_forced_induction` (before engine garage P2: `default_turbo_configuration`) points to
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

Each bank (PSAB0002) contains five RPM knots and two manifold-load planes, all
radiating source channels. Each plane is a loop of whole engine cycles (about
1 s, at least 12 cycles; knots moved to rpm = 48000*120/n so a cycle is n whole
frames) with the crank angle of its first frame stored and a cycle-aligned seam
crossfade. Playback reads every plane at the live voice's own crank angle, so
blended planes stay phase-aligned with each other and the engine; it
interpolates load and RPM at that crank angle. (The PSAB0001 sample-time
playback made level steps every ~0.5 s under throttle; it survives only as a
reference in `audio_bank_reference.h`.) The demo keeps the bank off by default
(`--m1-audio-bank` turns it on). A 120 ms crossfade replaces
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

The staged source built independently under out/hyper_commit_verify. Focused engine, turbo, audio-bank, SIMD and thread checks passed: 76,367 assertions in 22 cases. Physics maps are now generated from the engine cycle (the hand-authored maps were removed). The earlier WOT values here included Coulomb friction; the consistent net before/after calibration is reported above. Long sanitizer checks remain queued for overnight verification.

Final complete staged-source native checks: 76,616 assertions in 27 cases passed, including boosted map generation, cache-key coverage, low-speed tyre grip, and serial/threaded/SIMD equality. Godot adapter rebuilt; editor import passed. Full sanitizer suites are pending overnight, not declared passed.

## Shutdown observation, 2026-10-03

The first cold live-swap run completed generation, override and invalid-file fallback, then exited with Windows access violation 0xC0000005 during shutdown. A debugger run and six subsequent WASAPI live-swap/explicit-stop runs all exited zero, so the fault could not be reproduced or located; it is not claimed fixed. Dashboard/fuel and cockpit surface audio checks passed. Focused spatial lifecycle passed with endpoint rejection 0x80004001; downstream Atmos/headset output is not certified. Follow up with overnight sanitizer coverage and cold-cache shutdown reproduction. Logs: out/hyper_commit_verify/live_swap.log, shutdown_stack.log, shutdown_repeat_1.log through shutdown_repeat_6.log, dashboard.log, surface_audio.log, spatial_audio.log.

## Part files (engine garage P2)

The ECU calibration (throttle actuator, idle control, lambda and spark tables, idle rpm, fuel-cut limiter) lives in
`data/ecus/hyper_v8_one1.json` and the valve lifts in `data/cams/hyper_v8_one1.json`; the engine names them with
`default_ecu`/`default_cam`. They were moved verbatim: the assembled engine equals the pre-split file (same `.psmaps`
key, turbo tables and state hash - `tests/unit/test_engine_parts.cpp` against
`tests/support/fixtures/garage_p2/hyper_inline.json`). See `docs/data_formats/ecu.md` and `cam.md`.
