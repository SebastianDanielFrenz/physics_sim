# One:1-inspired hypercar engine approximation

The driven car uses `hyper_v8_torque.json`. Its published full-load anchors are
1371 Nm at 6000 rpm, 1 MW at 7500 rpm, more than 1000 Nm from 3000 through
8000 rpm, and an 8250 rpm fuel-cut limit.
Source: https://www.koenigsegg.com/technical-specifications-one1
The 7500 rpm torque is derived as P/omega = 1273.239545 Nm.
Intermediate/low-speed points, inertia, idle control and motoring losses are
assumed. This is a steady boosted torque envelope, not a measured dyno curve;
it does not simulate turbo lag or fuel-dependent power. Existing E5/E10 fuel
handling and crude fuel consumption remain approximations. The authored
clutch capacity is 1650 Nm so it can carry the new peak torque.

`hyper_v8_one1_audio.json` is a SEPARATE offline acoustic-development reference:
8 cylinders, 92 mm bore, 95.25 mm stroke, 9:1 compression and four valves per
cylinder follow the published geometry. The manufacturer calls it a 5.0 L V8;
the rounded published dimensions give approximately 5.066 L when integrated.
Firing order [1,5,4,8,6,3,7,2], bank assignments/90-degree angle, cams, port
sizes, pipe lengths, efficiency maps and installation are explicit assumptions.
The intake and two exhaust outlets are rendered from combustion/pipe physics.
The complete reference maps avoid unnecessary load-time map generation.

The acoustic reference is unboosted. There is currently no turbine/compressor,
spool, wastegate/blow-off or turbo-whine model. The shared exhaust merge and
loss elements do not claim to reproduce the One:1 twin-turbo installation.
Do not substitute its approximate naturally aspirated torque for the driven
car's rated torque envelope. It is not connected to car_hyper live audio:
that integration awaits the separately planned baked sound-map playback path.

Render from the repository root:

```powershell
out/build/release/tools/ps_engine_audio/ps_engine_audio.exe data/engines/hyper_v8_one1_audio.json --wav out/one1_reference/preview.wav --script data/scripts/engine_audio/hyper_v8_one1_preview.json --full-scale-pa 20
```

With the current uncommitted A9b acceleration work, add `--voice-threads 4`.
The authored four-second preview includes starter, idle, throttle blip,
lift-off and ignition cut. It is a listening reference, not a recording.

Verification on 2026-10-02:
- Native loaders accepted the updated car and offline reference/installation.
- Ten-second 240 Hz driving sequence: 2400 ticks, matching hashes at 1 and
  10 workers: `0xc4949eef4b251068`.
- Four-second 48 kHz mono PCM preview: no clipping, signal in every second.
- Serial/four-thread WAVs are byte-identical (SHA256
  `75275DEEF659BD77228166ACB250EE951B47E77F6A76BA9591ECED98C9716C65`).
- No Godot adapter rebuild or VR listening verification in this change.
- Logs and preview: `out/one1_reference/`.