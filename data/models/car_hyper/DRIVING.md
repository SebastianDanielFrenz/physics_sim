# Sculpted A driving configuration

Launch from the simulator root:

```cmd
drive -VR --m1-vehicle=car_hyper
```

`data/vehicles/car_hyper.json` now supplies the physical wheels, suspension,
brakes, steering and rear-drive powertrain. Its dependencies are
`hyper_v8_one1_audio.json`, `hyper_7mt.json` and separate front/rear tyre files.
The demo's `VEHICLE_TABLE` selects the hypercar art, chassis and spawn.

This is authored tuning with a One:1-inspired simulated twin-turbo V8, seven-speed assisted manual, 1500 kg chassis, 42/58 static load split and 2.5 torque-biasing differential. Live combustion/pipe audio and asynchronously generated audio banks follow the engine and turbo configuration.

The collision proxy is deliberately narrow (half extents 2.0, 0.4, 0.12 m).
The initial 0.72 m half-width caused full-compression wheel contacts and
violent neutral launch/rollover; changing only that width to 0.4 resolved
the issue. Keep wheel contact queries clear of the chassis proxy.

Earlier Godot verification with the original 600 kW setup:

- Native vehicle load and hypercar GLB load both succeeded.
- Ten-second headless autopilot: distance 63.61 m, maximum speed 57.46 km/h,
  upright=true, nan=false, maximum gear=2, stalled=false, engine_state=running.
- Missing-file negative check: `--m1-vehicle=does_not_exist` reports
  `Missing vehicle definition` and exits with code 1 before simulation creation.
- VR headset operation was not tested; the driving check ran headless.

Previous main.gd preserved under `out/asset_backups/hyper_physics_*`.

The current engine and live turbo/audio-bank configuration are documented in ../../engines/HYPER_ONE1.md. Published power figures are targets; see that document for measured approximation results.

Reverse and Park (2026-10-03): from stationary Neutral, press shift-down once for R. Reverse now has an authored 140 Nm synchronizer, matching the first forward gear, so the assisted clutch can stop the spinning input shaft before dog engagement. The hypercar now has a 1500 Nm output-shaft parking lock, accepting engagement below 1 rad/s. In the physics_sim demo, N toggles park_lock; use Neutral before engaging it, and the digital cluster displays P when the pawl is engaged. Park is a separate lock rather than another manual transmission ratio. Other frontends must forward the park_lock control (0 release / 1 request) and expose an input; racing_game currently does not forward that channel. The Godot regression test test_hyper_gears.gd verifies N-to-R selection and park engagement/release through the shipped vehicle and adapter.
