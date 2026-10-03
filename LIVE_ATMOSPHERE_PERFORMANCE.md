# Live atmosphere engine performance fix

The game reported 84-110 ms native physics ticks after enabling altitude-aware
engine pressure/temperature. The aero ground query was not the cause.

`SimulatedEngine::pre_solve` compared every atmospheric sample exactly and
called `set_ambient`. That setter regenerated the full RPM/plate steady table
and the full electronic RPM/request inverse. Each inverse grid entry nests
60 throttle bisections inside steady-manifold root solves. Even a microscopic
chassis-height change caused another complete grid rebuild in the vehicle job.

Runtime `Sensors::ambient` now follows each finite positive pressure/temperature
sample immediately, without quantization, hysteresis or a background delay.
Only the idle feed-forward/authority operating point is updated on a boundary
change. When the materialized setup cache does not match the current atmosphere,
the requested pedal and torque-demand points are calculated on demand.
Throttle/idle-control root solves are bounded to 24 bisections (actuator
uncertainty at most 2^-24); the physical manifold solver and actual torque/fuel
equations retain the exact current pressure/temperature. No full grid is built
or memory allocated in this hot path. Constant reference-atmosphere behavior
continues to use its existing tables and arithmetic.

The explicit internal engine `set_ambient` remains an expensive **setup/offline**
API for callers that need a materialized complete table. `steady_tables()` is
that setup cache, not a live atmospheric snapshot. `ambient_pressure()` and
`ambient_temperature()` expose the real current boundary. `World::set_ambient`
still takes the same API and is safe for continuous stepping-thread updates.
Full-table maps-swap preparation stays outside jobs. A newer atmospheric sample
at swap consumption refreshes only the bounded idle point. Non-reference setup
cache anchors are hashed because they select the tabulated-control path; cached
table values derive from those anchors and immutable maps.

## Measurements

`bench_aero_ground [vehicle.json]` is a native measurement tool, not a fragile
CI wall-clock gate. It creates 25 nonflat 256x256 terrain heightfields near
(-2955.3,-3562.8,251.5), runs 5000 self-excluding ground queries and compares
native ticks with aero on/off and continuous tiny atmospheric changes. This
isolates the engine/aero code without the renderer, audio output or terrain
streaming. Its stationary hypercar benchmark changes pressure by .001 Pa and
temperature by .00001 K per tick. The racing-game chat separately reproduced
the same problem on real world terrain (105.586 ms with atmosphere enabled).

Same optimized private native cache, actual hypercar engine:

| Workers | Aero | Before live atmosphere | After live atmosphere |
|---|---|---:|---:|
| 1 | off | 105.025 ms/tick | .148 ms/tick |
| 1 | on | 103.604 ms/tick | .173 ms/tick |
| 4 | off | 100.775 ms/tick | .178 ms/tick |
| 4 | on | 102.955 ms/tick | .200 ms/tick |

Ground queries remain about .61-.65 microseconds each. These are focused native
measurements, not a claim about final game-frame timing; racing_game must pin
and rebuild the fix and verify its real-world runtime. Aero forces, stall,
ground effect, wake sampling, wing actuation and fan energy remain enabled.

Regression coverage checks continuously changing exact atmospheric inputs,
zero full-grid rebuilds, altitude manifold/torque and pedal response against
the materialized setup reference, job allocation, deterministic hashes,
map swaps, turbo behavior and existing MVEM/engine integration checks.
Logs: out/ambient_perf_before.log, out/ambient_perf_after.log,
out/ambient_perf_tests.log, out/ambient_integration_tests.log. Long Linux
sanitizers remain in the 05:00 queue; no Godot/game DLL was replaced here.

Verification completed: the combined live-atmosphere/MVEM/turbo/aero/powertrain
set passed 77,409 assertions in 56 cases. The final identical-swap refinement
then passed all live-atmosphere and C16b map-swap cases: 610 assertions in 18
cases. Existing worker-parity/allocation checks pass; known ungated Jolt
internal-allocation warnings remain. The final refinement preserves identical
map swaps at non-reference atmosphere without changing the measured runtime
control path. No full sanitizer result is claimed.
