# Telemetry-driven tyre audio

The demo uses per-wheel positional Godot streams driven by existing speed,
wheel load, tread speed, slip and forces. No additional physics simulation.
Rolling retains the accepted asphalt synthesis from audition sections 1 and 2.
Sliding and stationary burnout use the licensed three-second asphalt squeal
sample in demo/audio/tyres, with a 50 ms overlapping loop crossfade and a 40 ms
level envelope. Small braking slip stays silent: squeal starts gradually above
an authored combined slip threshold (0.10 longitudinal ratio / 0.08 rad lateral),
with a sliding-speed gate to reject creeping slip-ratio spikes. This is audio
calibration, not telemetry for the exact tyre force limit. Independent wheel playback offsets reduce coherent repetition.
See audio/tyres/LICENSE.md for attribution, source and conversion details.

Dirt, grass and other surfaces currently share this generic asphalt fallback.
There are no distinct loose-surface recordings. Rejected synthetic spray is
muted; wetness reduces squeal gain but does not change rolling sound or friction.
The rejected synthetic turbo layer is muted even when requested by main.gd.
The underlying turbo prototype remains available for development checks only.

Controls: --m1-no-tyre-audio disables tyre sound. These streams use Godot
positional mixing, not the native Windows spatial-object engine stream.
Cabin occlusion/filtering is not modeled. Queues target 50 ms at 24 kHz;
graceful shutdown drains audio commands before exit.

Verification: focused model checks cover stationary/unloaded silence, speed
and slip response, generic/wet fallback, burnout and seeded reproducibility.
The tests also cover the dormant turbo prototype; passing them does not mean
its subjective sound is approved. No DLL rebuild is required.

Audition: out/one1_reference/tyres_sample_comparison.wav, two seconds each:
asphalt rolling 15 m/s, rolling 40 m/s, drift, wet rolling, dirt fallback,
grass fallback, stationary burnout. Export gain +12 dB, not runtime gain.
Re-export with scripts/export_surface_audio.gd. Original comparison is retained.
