# Hypercar rolling auto-clutch flare, 2026-10-04

Owner log preserved before any launch at
`out/hyper_clutch_flare/godot_owner_20261004.log`. The reported 119.5–122.5 s
flare occurred in seventh gear, with wheel and GPS speeds agreeing. The log
has no clutch/wheel-slip diagnostics, so it alone does not prove the cause.

## Native reproduction and cause

The shipped calibrated `car_hyper` was loaded with its generated cycle maps.
A complete native World/vehicle rig (1500 kg, flat ground, 240/960 Hz) was
settled, reset to seventh gear at 70/80/100 km/h with the auto-clutch released
and engine running at idle, then held at WOT for ten seconds. It uses real
powertrain, clutch solver and tyres, not a scripted RPM curve.

At the 70 km/h entry, the original code reached 8103 rpm at 1.5 s and remained
around 8200 rpm for the ten-second observation. At 2 s the gearbox input was
about 1439 rpm, clutch engagement 0.266 and capacity 438.5 Nm. Driven-wheel
slips were only 0.016. The failure is clutch slip without significant tyre
spin. The 80/100 km/h controls locked normally.

The torque-following rolling close consumes `PreSolveResult::torque_demand`.
The MVEM steady throttle tables solve against ambient supply, omitting a
spinning turbo's actual charge. They therefore under-requested clutch capacity
once the launch slip crossed into rolling re-engagement under boost. The
clutch under-carried the engine and the resulting flare became trapped on the
limiter. This also predates the latest power calibration in principle; the
specific demonstrated reproduction uses that calibrated engine.

## Correction

For a turbo MVEM with combustion enabled, its torque demand now takes the
larger of the existing steady request and current-charge net crank torque
(including current exhaust pressure, charge temperature, manifold filling,
fuel/ECU efficiency and Coulomb friction), evaluated without rev-limiter cuts.
It uses the existing torque evaluator, adds no simulation or new map, and
retains the one-substep-lagged TCU sensor exchange.

Rev-limiter pulses must not reopen the clutch: hypothetical uncut current
charge is used for that reference. Explicit TCU shift cuts and DFCO still
reduce the demand, preserving bounded flywheel-energy transfer during shifts.
Naturally aspirated/map engines, the launch RPM law, anti-stall thresholds,
manual-clutch gating and engine calibration are unchanged.

With the correction, the 70 km/h case is locked by 1.5 s (1377 rpm, zero
clutch slip); the 80/100 km/h controls remain locked. The regression requires
lock within two seconds, sustained low clutch slip thereafter, no engine
stall or gear change, bounded driven-wheel slip and continued acceleration.
Additional engine checks retain boost torque across a limiter pulse and
reduce it for a deliberate ignition shift cut. Existing TCU launch, anti-stall,
shift-cut and arbitration tests run in the same focused selection.

## Verification and integration

Native staged-source build and `[tcu],[sim_engine]` focused checks pass: 14,544 assertions in 15 cases; exact
assertion/case count is in `out/hyper_clutch_flare/focused.log`. Before/after
reproduction logs and original owner log are retained in that directory.
This is a demonstrated matching failure mechanism, not a replay of the exact
owner road/throttle trace. Integration still needs the racing-game chat's
DLL build and a road retest. No game source, Godot process or DLL was modified.
Long Linux sanitizer verification is queued for the overnight owner, with
existing queues preserved; full CI is not claimed green.

## Signed anti-stall threshold follow-up, 2026-10-04

The owner's follow-up requires an engaged rolling clutch to stay closed when
both actual crank RPM and the RPM imposed by the driveline are safe. At
70 km/h in seventh the input is about +1230 rpm, above the shipped 850 rpm
anti-stall threshold but below the old 1300 rpm rolling threshold and 1400 rpm
launch bite. Rolling/post-shift safety now uses the signed anti-stall threshold;
the legacy rolling_reengage_rpm field remains accepted for compatibility.

Locked auto-clutch release is immediate if actual signed crank RPM or signed
coupled RPM falls below that threshold. Backward shaft rotation is unsafe,
not a high positive RPM: sustained negative input holds the clutch open, with
a lock-slip-speed deadband for near-rest jitter. The auto-clutch output remains
gated by the assist channel; with it disabled the manual clutch stays authoritative.
Normal gearbox shifts and standstill launch modulation remain separate behaviors.

The full-vehicle backward test starts with a running, forward-coupled drivetrain,
then reverses chassis travel to -60/-120/-200 km/h without resetting or
pre-reversing the crank. The old controller fails this reproduction, including
actual engine stalls. The corrected controller protects the running crank,
leaves the clutch open against reverse input, and never stalls in these cases.
Additional unit checks hold full engagement at +1230 input RPM across zero,
half and full throttle, release for actual/coupled RPM below 850, and check
manual-assist gating. Evidence: out/hyper_clutch_flare/backward_before.log and
signed_tests.log. This is a focused velocity-reversal proxy, not a full road
replay of the owner's 180-degree spin.

Final staged-source focused verification: **23,811 assertions in 17 cases passed**.
The old controller failed the backward proxy with 3,513 failed assertions,
including 1,200 sampled stalled-engine failures. No game/Godot DLL was rebuilt.
