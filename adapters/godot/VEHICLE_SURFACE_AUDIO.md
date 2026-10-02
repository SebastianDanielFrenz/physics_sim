# Telemetry-driven tyre and turbo audio

The demo now creates positional AudioStreamPlayer3D feeds at each wheel and,
for car_hyper, a separate engine-bay turbo sound feed. These are procedural
presentation sounds; they add no force, tyre, weather or turbo physics.
They currently use Godot positional mixing, not the engine's separate native
Windows spatial-object stream. Cabin occlusion/filtering is not yet modeled.

Tyres consume the existing wheel load, omega/radius, slip ratio/angle, fx/fy,
vehicle speed and physical surface name. Load gates contact sounds. Rolling
level and spectral cutoff rise with speed. Slip/force work proxies drive scrub,
slide/squeal and burnout noise, including rear-wheel spin at zero road speed.
Authored profiles distinguish asphalt, kerb, dirt, grass, field and low_mu ice.
Loose surfaces have rougher broadband textures and less tonal rubber squeal.
Seeds differ per wheel, so identical conditions do not create coherent noise.
Unknown surface names use the generic asphalt profile. Numeric gains and
filters are sound-design assumptions, not measured tyre spectra.

Wetness is explicit audio input in [0,1]. The simulator has no weather/water
state, so no weather is inferred and friction is unchanged. Wetness adds
high-frequency spray and reduces tonal dry-rubber squeal. Set individual
surfaces with SurfaceAudio.set_surface_wetness(name, amount), or audition a
uniform condition with `drive --m1-vehicle=car_hyper --m1-road-wetness=0.7`.

The current powertrain exposes no turbo shaft RPM or boost pressure. The turbo
layer therefore uses engine RPM, throttle, fuel flow and running state only:
smooth spool/whine and flow-noise envelopes plus a lift-off pressure-release
cue. These proxies are NOT physics telemetry and are never shown as measured
boost/RPM in the dashboard. If actual `turbo_rpm` and `boost_bar` fields become
available, the layer uses them automatically, including measured coast-down.
The twin-tone frequencies, envelope times and noise gains are authored
approximations. No turbo compressor/turbine simulation or exhaust-pulse
attenuation was added; the full offline V8 acoustic reference remains separate.

Controls: `--m1-no-tyre-audio` and `--m1-no-turbo-audio` independently mute layers.
Runtime queues target 50 ms at 24 kHz. Graceful shutdown drains audio commands
before SceneTree exits, avoiding leaked generator playback objects.

Verification (2026-10-02):
- Headless model checks pass: stationary/unloaded silence; speed and slip energy
  response; surface/wetness changes; stationary burnout; seeded reproducibility;
  spool/lift-off/shutdown envelopes; supplied turbo telemetry overrides estimates.
- Ten-second real hypercar autopilot, wetness 0.7, asphalt/dirt transitions:
  distance 75.14 m, max speed 65.69 km/h, upright, no NaN, max gear 2, engine
  running. Final verbose run has no script errors or leaked audio resources.
- Headless checks do not establish subjective speaker/headset mix quality.
- No DLL rebuild or extra physics simulation was performed.

Run checks: Godot --headless --path adapters/godot/demo --xr-mode off
--script res://scripts/test_surface_audio.gd

Audition files (export gain +12 dB for listening, not the runtime gain):
`out/one1_reference/tyres_comparison.wav`: 2 seconds each, dry asphalt 15 m/s,
dry asphalt 40 m/s, asphalt drift 30 m/s, wet asphalt 30 m/s, dirt slide,
grass slide, stationary burnout. `turbo_cues.wav`: six seconds of authored
spool/lift-off cues. Re-export with scripts/export_surface_audio.gd.
Logs: out/one1_reference/surface_audio_tests.log and surface_audio_verified.log.