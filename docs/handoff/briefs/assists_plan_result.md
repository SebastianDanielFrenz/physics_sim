# Assists plan: hypercar grass-slope crawl, TC modes, launch control, aero automation

Status: PLAN, 2026-10-12, opus assists plan session (worktree branch `worktree-agent-a4097b223cb9eb4ec`).
Number tags: [C] cited, [D] derived from data/code, [AS] assumed, [M] measured in this session.
Rig: `tests/unit/test_assist_slope_crawl_diag.cpp` (hidden `[.][diag][traction_control][vehicle]`),
fixture `tests/support/fixtures/assists/surfaces_racing_game.json`. Raw outputs: scratchpad `assists/`
(matrix.txt, freerev.txt, stallhunt.txt, ab.txt, trace_p1.txt).

---

## Decision summary

1. **The car cannot climb because of torque, not grip.** At part pedal, car_hyper's electronic throttle delivers
   almost no net torque at crawl speed. The request is linear between motoring (about -70 N*m) and steady WOT, so
   the first ~17 % of pedal [D] only cancels motoring. Net crank torque [M] at 960-1570 rpm: about 0 N*m at
   pedal 0.1, 9-11 at 0.2, 46-57 at 0.3, 84-103 at 0.4. Climbing 8 % grass needs 85 N*m at the crank [D]. The
   auto-clutch launch law then holds the engine just above bite (1400 rpm [D]) and passes exactly that torque on:
   - pedal 0.2: rolls back on every slope, even asphalt [M];
   - pedal 0.4: barely holds 8 % grass (0.05 m/s after 8 s) [M].

   Tyre use is only 0.37-0.56 [M], so this is not a traction limit.
2. **With TC on (racing_game's default), wide-open throttle turns into a stall-prone limit cycle.** The trace shows
   this sequence [M]:
   1. The clutch locks onto rear wheels spinning at kappa 3.
   2. TC cuts the torque fraction to 0.
   3. The locked engine is dragged from 2400 rpm down to 850 rpm, and anti-stall dumps the clutch.
   4. A tyre-deflection spring-back pushes the car backwards (fx -3300..-3750 N for ~0.15 s).
   5. The launch law re-engages, and the cycle repeats every ~0.6 s.

   Results:
   - On 12 % grass the car is stuck: 0.27-0.34 m/s with TC on vs 2.9 m/s with TC off [M].
   - Minimum rpm in the cycle is 675-800 [M].
   - The stall itself (below 400 rpm) did not reproduce in 180+ hunt runs [M]. It needs only one dump that comes
     late, so it is a plausible owner event, but it is UNPROVEN.

   The root problem is geometric. In 1st gear, idle rpm puts the wheel rim at 3.49 m/s [D]. So a locked clutch
   below ~3.1 m/s *requires* wheelspin. TC can only remove that spin by dragging the engine below idle. At crawl
   speed TC must act through clutch slip, not through engine torque.
3. **Fix in the clutch/TCU and the pedal map, in every TC mode** (D-LS1..D-LS4 below):
   - D-LS1: at crawl speed, TC's torque limit is applied as an auto-clutch capacity cap. The clutch slips; the engine
     is not pulled down.
   - D-LS2: the auto-clutch never locks while the ground-speed-equivalent input rpm is below anti-stall.
   - D-LS3: TC no longer lowers the launch law's throttle_request.
   - D-LS4: car_hyper gets a progressive pedal_map in data. This needs owner approval and changes car_hyper's own
     behaviour only.

   Lowering anti-stall/bite does NOT fix it: the A/B at 600/1100/1600 rpm left pedal 0.4 on 8 % grass at
   0.043-0.048 m/s [M].
4. **TC modes:** keep the single channel `assist.traction_control` as an append-only level code:
   0 Off, 1 On (= today, bit-identical), 2 Relaxed, 3 Sport, 4 Drift.
   - Today's reader treats > 0.5 as on, so 2-4 mean "on" on old code.
   - Per-mode parameters come from a new electronics kind `assist_unit` (physics_sim.electronics/1). Its grade is
     `standard` (Off/On) or `performance` (all five plus launch control and aero automation).
   - A mode the unit lacks maps statelessly to the nearest available mode with *more* intervention.
5. **Launch control (new assist, in the performance unit).**
   - Arms on throttle > 0.9 AND brake > 0.5 below 0.5 m/s.
   - Holds a launch rpm through P8's existing `TcuRequest::limiter_rpm`.
   - On brake release it uses a slip-targeted torque ceiling: through clutch capacity first, then the TC fraction.
   - Hands over to TC (the current mode) at 6 m/s or 3 s.
   - Commands the rear wing to its maximum-downforce offset.
   - A car with both the P8 two-step and the new LC is rejected by the loader.
6. **Aero automation:** racing_game's rear-wing law (session.cpp:816-828) moves into a physics_sim assist with its own
   `fitted` flag and channel `assist.aero_control`. It is stateless, and it yields to LC while LC is active.
7. **Hash rule for all of this:** no assist_unit part means bit-identical. Mode 1 means bit-identical to today's TC.
   New state is hashed only on cars that carry the part. No shipped scenario uses car_hyper, TC or LC [D], so every
   shipped scenario hash stays unchanged.

---

## Part A: measured diagnosis

### Rig

- World 240/960 Hz with a car_hyper desc loaded once (maps generated, cache in temp).
- A tilted `flat_surface_mesh(300 m)` tagged with one surface. The plane rises along +X.
- Chassis box 2 x 0.4 x 0.15 m, 1500 kg.
- Settle 1.5 s on the handbrake in 1st with `assist.auto_clutch` = 1 and `assist.traction_control` = tc.
- Then `set_motion` to v0 along the slope plus `reset_vehicle{hubs_roll_with_chassis}`, release the handbrake,
  apply throttle (optionally lift at t for a duration), and run 8 s.
- Per tick it records: v and s along the slope, rpm, engine state, clutch e/slip/torque, the TC fraction/slip/
  estimate, rear kappa, fx, fz, utilisation, boost and the idle integrator.
- Surfaces:
  - `grass`: physics_sim, lambda_mu 0.42, crr 0.08 [D].
  - `grass_rg`: racing_game's value, 0.63 / 0.08 [D, RACE-002].
  - `asphalt`.
- Cases: the matrix (3-12 %, throttle 0.2-1.0, TC 0/1, v0 0.3/0.6), auto-clutch threshold A/B, an env-driven trace,
  a stall hunt (patterns: constant, lift-reapply, rolling back at -0.8 m/s, 2nd gear, auto-shift), and a free rev in
  neutral (torque per pedal).

### Requirement vs. grip (hypothesis 3: rejected)

F_req = m*g*(sin theta + crr*cos theta); F_avail = pdx1*lambda_mux*lambda_mu*Fz_rear (settled rear load) [D]:

| grade | F_req | F_avail, grass 0.42 | F_avail, grass_rg 0.63 |
|---|---|---|---|
| 5 % | 1911 N | 4392 N | 6588 N |
| 8 % | 2347 N | 4425 N | 6637 N |
| 12 % | ~2930 N | ~4.4 kN | ~6.6 kN |

Grip exceeds the requirement by 1.5-3.5x [D]. At pedal 0.4, utilisation was 0.53-0.56 on grass and 0.35-0.37 on
grass_rg [M].

### Part-pedal torque (hypothesis 4: confirmed, dominant)

Net crank torque demand in neutral, free rev, 960-1570 rpm [M]:

| pedal | 0.1 | 0.2 | 0.3 | 0.4 | 0.6 | 1.0 |
|---|---|---|---|---|---|---|
| net torque, N*m | ~0 | 9-11 | 46-57 | 84-103 | 158-224 | 307 at 960 rpm, rising with boost |

- Cause [D]: SimulatedEngine maps the electronic request r to net torque = mot + r*(wot_steady - mot). Motoring is
  about -70 N*m near 1400 rpm and WOT_steady about 330-450 N*m, so zero net torque needs r ~0.17.
- Wheel force needed [D]:
  - 8 % grass: 2347 N * 0.364 m / (10.385 * 0.97) = 84.8 N*m at the crank.
  - 12 % grass: ~105 N*m.
  - 5 % grass: ~69 N*m.
- So pedal 0.4 is the minimum for 8 %, and pedal 0.3 is not enough for 5 % if motoring is included.

Matrix rows, all [M], 8 s, v0 0.4:

| surface | grade | pedal | TC | v_end m/s | rpm (last 3 s) | e_max | clutch torque max |
|---|---|---|---|---|---|---|---|
| grass | 5 % | 0.2 | 0/1 | -0.015 | 1406 | 0.007 | 11 N*m |
| grass | 5 % | 0.4 | 0/1 | 2.42 | 1466 | 0.055 | 91 N*m |
| grass | 8 % | 0.4 | 0/1 | 0.048 | 1466 | 0.055 | 91 N*m |
| grass_rg | 8 % | 0.4 | 0/1 | 0.048 | 1466 | 0.055 | 91 N*m |
| asphalt | 5/8/12 % | 0.2 | 0 | -0.7 / -2.9 / -6.2 | ~1406 | <0.01 | 11 N*m |

The launch law [D, manual_tcu.cpp] is e = clamp((rpm - bite)/(full - bite)) with bite 1400 and
full = 2000 + pedal*1500 [D, car_hyper.json]. Its torque slope at the bite point is steep:
1650 N*m / (full - 1400) per rpm, so ~2.75 N*m per rpm at pedal 0 [D]. The engine therefore settles a few rpm above
bite, where clutch torque equals its tiny part-pedal net torque. The clutch passes on everything the engine makes;
the engine just makes too little.

### Auto-clutch thresholds (hypothesis 2: rejected as the cause)

A/B at pedal 0.4, 8 % grass [M]:

| anti_stall / bite / full_min | v_end | rpm |
|---|---|---|
| 850 / 1400 / 2000 (stock) | 0.048 | 1466 |
| 600 / 1400 / 2000 | 0.048 | 1466 |
| 600 / 1100 / 1600 | 0.043 | 1166 |

None of these changes helps. A lower bite makes the engine sit at lower rpm with no more torque. At WOT with TC on
the results scatter chaotically between variants (8 % grass: 11.3 m/s stock vs 3.0 at anti-stall 600). That scatter
is the limit cycle below, not a threshold effect.

### TC at crawl speed (hypothesis 1: confirmed for WOT / TC on)

| case | TC off | TC on | note |
|---|---|---|---|
| grass 12 % WOT, v_end | 2.9 m/s | 0.27-0.34 m/s | TC on: 97-98 % of ticks intervening, min fraction 0.0 |
| grass_rg 12 % WOT, v_end | 13.4 m/s | 3.1 m/s | |
| grass 5 % WOT, v_end | 10.4 m/s | 3.3 m/s | |
| grass_rg 3-8 % WOT | worse | better | TC is a gain on the higher-grip surface at gentle slopes |

At pedal 0.4, TC never intervened (slip ≤ 0.065 against target 0.08) [M].

All [M] unless tagged otherwise. Trace: grass 12 %, WOT, TC on, lift at 2 s for 1.5 s, re-apply at 3.5 s.

| t (s) | what happens |
|---|---|
| 3.55-3.68 | The launch law closes the clutch (e 0.05 -> 0.29). Wheels spin to the kappa clamp of 3.0, fx 3980 N, utilisation 0.79. |
| 3.70-3.88 | e rises at 4/s to 1.0 and the clutch locks (slip 0) onto the spinning wheels. The car crawls at 0.02-0.07 m/s. |
| 3.78-3.83 | TC slip 4.4 (ratio over the 2 m/s floor), fraction 1.0 -> 0.0. The ignition cut is active. |
| 3.83-4.10 | The engine and the locked driveline decelerate together, 2430 -> 844 rpm. The wheels keep spinning (kappa 3) the whole time. |
| 4.10 | rpm < 850 triggers the anti-stall emergency release (e -> 0). |
| 4.13-4.30 | The tyre's low-speed deflection state springs back: fx -3424..-3753 N for ~0.15 s, v 0.17 -> -0.27 m/s. This costs ~0.4 m/s per cycle (impulse ~510 N*s [D]). |
| 4.38 | The launch law re-engages and the cycle repeats every ~0.6 s (dumps at 4.10, 4.68, 5.28 s). rpm minima 796-817. |

Geometry [D]:
- Total ratio in 1st is 3.1 * 3.35 = 10.385 and the rear radius is 0.364 m.
- At 950 rpm idle the wheel rim speed is 99.5 rad/s / 10.385 * 0.364 = 3.49 m/s.
- At 850 rpm anti-stall it is 3.12 m/s.

So below ~3.1 m/s a locked 1st-gear clutch means the wheels spin at least (3.1 - v) m/s. TC asks for
v_driven < v + 0.08 * 2.0 = v + 0.16 m/s. The only way to get there is to drag the engine below anti-stall. The TC
controller and the clutch's lock state fight each other.

Stall hunt [M]: 180 rows, 0 stalls.

| pattern | rpm_min | ticks below anti-stall | anti-stall dumps | v_end |
|---|---|---|---|---|
| p1, grass 12 %, pedal 0.7, TC on, lift-reapply | 675 | 124 | — | 0.16 m/s |
| p2, rolling back at -0.8 m/s then WOT, TC on | 739-822 | many | 7-22 | 0.1-3.6 m/s |
| TC off (comparison) | ~831-839 | — | — | — |

Stall rpm is 400 [D]. The margin left after a dump is 275-400 rpm. A late dump (a heavier idle-integrator state,
racing_game's quadratic rolling law, a different grass value, a gear request mid-cycle) could plausibly close it.
That is UNPROVEN.

### Finding F4: tyre spring-back at near-zero speed (secondary)

When the driven wheels stop spinning at Vx ~0.1 m/s, the relaxed longitudinal slip goes from +3 to about -2.4. The
tyre then pushes the car backwards with ~0.8 * Fz for ~0.15 s [M].

- Mechanism [D, tyre_model step_slip_relaxation]: at Vx -> 0 the relaxation ODE becomes a pure integration of slip
  velocity, clamped to +-3. The clamp state therefore releases as a reverse force.
- It is physical in kind (carcass deflection), but at the clamp magnitude it acts as a spring that stores far more
  than a real carcass can.
- It is not the root cause, but every TC/anti-stall cycle loses ~0.4 m/s to it.
- Follow-up: a diag case that logs hub omega and kappa_eq through a dump (I2 below). A fix belongs with the tyre
  owner (a deflection cap tied to sigma, e.g. |kappa_eq| <= kappa at the force peak, while Vx < v_low). This changes
  every vehicle hash, so it needs an owner decision (Q8).

### Diagnosis

| # | hypothesis | verdict |
|---|---|---|
| 1 | sensor TC at near-zero speed | CONFIRMED for WOT / TC on: limit cycle with the auto-clutch, and the source of the stall risk |
| 2 | auto-clutch anti-stall / launch law | REJECTED as a threshold problem. It is a LOCK-STATE problem: the clutch locks onto spinning wheels at crawl speed, and TC then fights the lock |
| 3 | traction | REJECTED: grip 1.5-3.5x the requirement |
| 4a | electronic pedal-to-torque law | CONFIRMED, DOMINANT at part pedal: 17 % dead band [D], 85 N*m needed vs 9-57 N*m delivered below pedal 0.35 [M] |
| 4b | tyre low-speed spring-back | CONTRIBUTING: costs ~0.4 m/s per dump cycle |

Owner symptom ("cannot climb a slight grass slope from 1-2 km/h, stalled once"):
- In racing_game (TC on by default, grass 0.63), light pedal explains not climbing (4a).
- Flooring it gives the stuck limit cycle with anti-stall dumps, and plausibly the stall (1 + 2-lock + 4b).

---

## Part B: plan

### B1. TC modes OFF / Drift / Sport / Relaxed / ON

**Channel (PROVISIONAL DECISION D-TC1):** keep `assist.traction_control` and give it an append-only level code.

| code | mode | intent |
|---|---|---|
| 0 | Off | no TC |
| 1 | On | today's controller and parameters, bit-identical |
| 2 | Relaxed | slightly higher slip target |
| 3 | Sport | high slip target, late and soft intervention, yaw-blind |
| 4 | Drift | TC active only above a large slip, never cuts below a torque floor; lets the rear step out |

- Read as `mode = clamp(round(value), 0, 4)` and record `mode_requested`.
- The order of intervention is On > Relaxed > Sport > Drift > Off (mode numbers are NOT ordered by intervention, so
  the code keeps an explicit intervention-rank table).
- Old data and racing_game write 0 or 1, so nothing changes for them.

Alternatives:
- (a) A separate `assist.traction_mode` channel next to the boolean.
  - Pro: the boolean is untouched.
  - Con: two channels can disagree (mode 3 with TC 0), and every input map must write both.
  - Rejected.
- (b) Fractional slip-target channel.
  - Pro: continuous.
  - Con: not a discrete mode the HUD and the garage can name, and no per-mode parameter set.
  - Rejected.
- Chosen: one level channel. Today's reader treats value > 0.5 as on, so codes 2-4 even degrade gracefully to On on
  an older build.

**Per-mode parameters (D-TC2):** a mode table in data, in a new electronics kind `assist_unit` (B1-unit below).
Each mode lists overrides of `TractionControlDesc`:

| field | On [D, today] | Relaxed [AS] | Sport [AS] | Drift [AS] |
|---|---|---|---|---|
| target_slip | 0.08 | 0.12 | 0.18 | 0.35 |
| kp | 2 | 2 | 1.5 | 1 |
| ki_per_s | 6 | 5 | 4 | 2 |
| attack_rate_per_s | 20 | 15 | 10 | 6 |
| recovery_rate_per_s | 8 | 8 | 10 | 12 |
| min_fraction (new; torque floor) | 0 | 0 | 0.15 | 0.35 |
| ignition_cut | true | true | false | false |

- New per-mode keys:
  - `min_fraction`: the TC never cuts below it. Default 0 means today's behaviour.
  - `ignition_cut`: today it is always true.
  - `slip_speed_floor_m_s`: from B2; default = today's `reference_floor_m_s` 2.0.
- Mode 1's row is NOT stored; it is the car's own `traction_control` block. That is why mode 1 stays bit-identical.
- Values are [AS] starting points; I1 measures them against the B6 bands.

**Default mode per car (D-TC3):**
- The `assist_unit` carries `default_mode`. It becomes the channel default when the car registers
  `assist.traction_control`.
- Without a unit, the default is the existing `traction_control.enabled` (0/1).
- car_hyper (no TC block today, so the default is Off): recommend a performance unit with default_mode 1 (On) (Q2).
- racing_game sets the channel itself today, so its default stays its own.

**Assist-unit grade as an electronics part (D-TC4):** new kind `assist_unit` in physics_sim.electronics/1:

```json
{ "format": "physics_sim.electronics/1", "kind": "assist_unit", "grade": "performance",
  "traction_control": { "modes": ["off","on","relaxed","sport","drift"], "default_mode": "on",
                        "mode_params": { "relaxed": {...}, "sport": {...}, "drift": {...} } },
  "launch_control": { ... B3 ... }, "aero_control": { ... B4 ... }, "source": "[AS] ..." }
```

- `grade` is a label plus a validation preset:
  - `standard` allows the modes {off, on} and no LC or aero blocks.
  - `performance` allows everything.
  - The `modes` list is authoritative; grade only rejects a block it does not allow.
- `extends` works as for every electronics part.
- Attach it through a vehicle controllers entry of type `electronics` or `VehicleLoadOptions::electronics`, exactly
  like P8 (a path replaces or adds, "none" removes).

How `fitted` maps:
- `traction_control.fitted = false` (a P8 assist_fitment) wins over everything: no modes and the channel is ignored.
- An `assist_unit` without a `traction_control` block leaves TC as the car's base desc defines it (modes {off, on}).
- An LC or aero block in the unit sets that assist's `fitted` = true. Absent means not fitted.
- `assist_fitted_bits` gets bit6 for aero automation. LC reuses bit4 (one LC kind per car, see B3).

**Unavailable modes (D-TC5):**
- The requested code maps to the nearest available mode with MORE intervention, walking the rank table
  Drift -> Sport -> Relaxed -> On. Off is never chosen as a substitute.
- If Off itself is not in `modes` (a "TC cannot be disabled" unit), Off maps to the least-intervening available mode.
- Stateless: computed from the channel value every tick, never latched.
- The effective mode goes into the snapshot.
- Alternative: reject or ignore the request (keep the previous mode). That would need hashed state, and a stale mode
  survives an input-map bug. Rejected.

**Hash rules (D-TC6):**
- `fold_traction_control_hash` folds as today. The mode-table desc is folded only when an assist_unit with a TC
  block exists.
- The effective mode is a pure function of channel and desc, so it is not state and not hashed.
- The controller state (estimate, integral, fraction) is unchanged.
- A mode switch keeps the integral and fraction, so there is no jump.
- Mode 1 runs the identical code path with identical parameters, so it is bit-identical. A test pins it.

### B2. Low-speed fix (valid in every TC mode)

The cause is in the clutch/TC interaction and the pedal law, not in TC tuning. So the fix goes there, and every mode
inherits it.

**D-LS1: TC acts through the clutch at crawl speed.**
- When the auto-clutch is in Launch mode, or Locked with the ground-speed-equivalent gearbox-input rpm below
  `anti_stall_rpm * 1.25` [AS], TC's ceiling turns into a clutch capacity cap:
  `cap_tc = fraction * T_driver_demand + (1 - fraction) * T_floor`.
  - `T_driver_demand` is the engine torque demand at the driver's command.
  - `T_floor = max(0, mode.min_fraction * T_driver_demand)`.
- The engine keeps the driver's command, and the idle PI keeps rpm. The clutch slips instead of dragging the engine.
- Above that speed, today's engine-ceiling path applies unchanged.
- This is how an automated launch with clutch slip works on a real DCT [AS: no cited source read].
- Alternatives:
  - (a) Raise anti-stall: shorter cycle, same fight. Rejected by the A/B [M].
  - (b) Disable TC below 3 m/s: TC-off WOT on 12 % grass reached 2.9 m/s and spins freely; it loses the grass_rg
    3-8 % gains [M]. Rejected.
  - (c) Slip-velocity TC criterion only (no clutch path): TC still has no way to slow the wheels except the engine.
    Necessary but not sufficient.

**D-LS2: no lock onto spinning wheels.**
- In Launch mode the TCU does not take the Locked transition while the input rpm implied by the vehicle speed is
  below `anti_stall_rpm`, i.e. while |v_ref| * K_input < anti_stall_rpm.
  - K_input = 60 * ratio_total / (2 * pi * r) [D].
  - v_ref is the TC estimate when TC is fitted, else the mean undriven-wheel peripheral speed. A 4WD car with no
    undriven wheel falls back to today's behaviour.
- Unlocked, the friction row still sticks physically if the capacity exceeds the torque. So with TC Off at WOT,
  e = 1 behaves as today. The difference is that the anti-stall emergency path cannot be armed by a "lock" at crawl
  speed.
- New sensor input `Sensors::ground_speed_mps`, unhashed. ManualTcu's lock decision reads it.
- Hashed: nothing new (`launch_lock_inhibit` is derived per substep).

**D-LS3: TC ceiling out of the launch law.**
- powertrain.cpp:1912 lowers `Sensors::throttle_request` by the TC ceiling. That moves full_rpm down while TC
  intervenes, which closes the clutch faster onto the spin.
- Under D-LS1 this coupling is replaced by the capacity cap. The launch law sees the driver's command.

**D-LS4: pedal law (data, per car; Q3).**
- car_hyper's ECU or throttle_control gets a progressive `pedal_map` (an existing feature, C3) that removes the 17 %
  dead band.
- Proposal [AS], pedal -> command: (0, 0), (0.05, 0.17), (0.2, 0.32), (0.4, 0.52), (0.7, 0.8), (1, 1).
- With it, pedal 0.4 commands about 0.52, i.e. ~140 N*m [D from the free-rev table], 1.6x the 8 % requirement.
- Pedal 0 still means motoring and DFCO.
- This changes only car_hyper's behaviour. No shipped scenario uses car_hyper [D].
- Alternative: change SimulatedEngine's request law globally (zero net torque at r = 0+).
  - Pro: fixes every simulated car.
  - Con: moves every simulated-engine scenario hash and the I7 pins.
  - Recommended later as an owner-approved engine increment, not here.

**D-LS5: tyre spring-back (F4).** Diag first (I2a). Any fix is owner-gated (Q8) because it moves every vehicle hash.

Hold/creep (hill-hold, DCT creep at pedal 0) is out of scope (Q9). It would answer the pedal-0 rollback on asphalt
(-0.7 to -6.2 m/s [M]), which is real-car behaviour for a manual and a gap for an automated clutch.

### B3. Launch control (new assist)

PROVISIONAL DECISION D-LC1: a new controller variant `LaunchAssistDesc` inside the `assist_unit`.
- It reuses P8's `LaunchControl` machinery for the rpm hold (`TcuRequest::limiter_rpm` through the engine's own
  limiter: fuel cut plus soft cut) and adds arming, release and hand-over.
- At most one launch variant per car: the loader rejects a P8 two-step plus an assist_unit LC with
  "launch control: a car has either a two-step (launch_control) or an assist-unit launch control, not both" (Q5).
  Both would drive `limiter_rpm`, and a two-owner limiter is a hash and debug trap.
- Channel `assist.launch_control` stays the enable (same name as P8, so the HUD and input maps do not change). It is
  registered only on cars with either part.

Phases (one hashed byte plus a timer, only on cars with the part):

| phase | enter | action | exit |
|---|---|---|---|
| Idle | — | nothing | arm condition |
| Armed | enable >= 0.5 AND throttle > `arm_throttle` 0.9 [AS] AND brake > `arm_brake` 0.5 [AS] AND speedo < 0.5 m/s [AS] AND gear 1 | limiter_rpm = `launch_rpm` (car_hyper 4500 [AS]); the auto-clutch is held open (e = 0, as P8's clutch-in condition); TC is suspended (its estimate keeps running); aero = launch wing | brake < `release_brake` 0.2 [AS] -> Launch; throttle < 0.5 -> Idle |
| Launch | from Armed | clutch closes on a torque-following ramp to full capacity in `clutch_ramp_s` 0.25 [AS]; engine torque ceiling from a slip PI on the rear kappa toward `target_slip` (mode-dependent: On 0.10, Sport 0.14, Drift 0.2 [AS]) using the TC estimate; the limiter is lifted at release | speedo > `handover_speed_mps` 6 [AS] OR t > `max_launch_s` 3 [AS] OR throttle < 0.5 OR a shift -> Handover |
| Handover | from Launch | the TC fraction starts from the LC's last ceiling (no jump); the wing returns to the aero controller | immediate -> Idle |

- Holding launch rpm against an open clutch builds boost on a turbo engine. The anti-lag channel stays separate. The
  plan does NOT auto-enable anti-lag (Q6).
- "Holds boost" is measured, not commanded: the test band is boost at release >= 0.4 bar [AS] on car_hyper at
  4500 rpm.
- The P8 two-step's clutch-pedal arming stays as is for manual-clutch cars.

### B4. Aero automation

PROVISIONAL DECISION D-AE1: assist `aero_control` in the `assist_unit`; `fitted` = block present.
- It runs in VehicleInstance before the aero step, per tick (stateless).
- It writes the surface command the same way `World::set_aero_surface_command` does, as a computed override:
  - with channel `assist.aero_control` >= 0.5 the controller owns the listed surfaces;
  - otherwise the World command stands.
- Port of racing_game session.cpp:816-828 [D]. Surface `rear_wing`, speed = the forward airspeed from the aero
  environment (the speedo when there is no wind):
  - `target_deg = cruise_offset_deg (4) * clamp((v - 10)/20, 0, 1)`
  - if v >= `airbrake_min_speed_m_s` (20): `demand = clamp((brake - 0.45)/0.55, 0, 1)`;
    `target += (airbrake_offset_deg (45) - target) * demand`
  - `command = target_rad / limit` (limit = max_offset for positive targets)
- Data block fields (defaults = racing_game's rg/environment.h values [D]): `surface`, `cruise_offset_deg`,
  `cruise_ramp_start_m_s`, `cruise_ramp_end_m_s`, `airbrake_offset_deg`, `airbrake_min_speed_m_s`,
  `airbrake_brake_start`, `airbrake_brake_full`, `launch_offset_deg`.
- Yield order: LC (Armed/Launch: `launch_offset_deg` = the max-downforce offset), then airbrake, then cruise.
  - car_hyper max downforce is near an offset of 9-11 deg [D: alpha = -(8 deg + offset), cl0 -0.15,
    slope 4.5/rad, stall 0.3 rad]. Data value 10 [D].
  - The airbrake can never be active during LC because LC ends at 6 m/s, under 20 m/s.
- Hash: no new state; the surface offsets are already hashed and change only through the slewed command.
- racing_game then deletes its own law and sets `assist.aero_control` (a racing_game session, not here).
- Alternative: leave the law in racing_game and only add an LC hook.
  - Con: the owner asked for physics_sim ownership, and the LC wing would need a cross-repo signal.
  - Rejected.

### B5. Interfaces

Channels:
- `assist.traction_control`: level 0-4, existing channel, append-only codes.
- `assist.launch_control`: existing name; now also registered for an assist_unit LC.
- `assist.aero_control`: new; registered only on cars with an aero block; default = the block's `default_enabled`
  (car_hyper recommended 1).

Snapshot (PowertrainSnapshot or VehicleSnapshot, never hashed):
- `traction_mode_requested`, `traction_mode_effective` (uint8);
- `traction_modes_available_bits` (bit per code);
- `launch_phase` (0 Idle, 1 Armed, 2 Launch, 3 Handover);
- `launch_slip_target`;
- `aero_control_active`, `aero_control_target_deg`;
- `assist_fitted_bits` bit6 aero, `assist_active_bits` bit6 aero (bit4 stays launch);
- `clutch_capacity_cap_tc` (debug only, PowertrainDebug).

Formats and code:
- `docs/data_formats/electronics.md`: kind `assist_unit`, grades, mode table, LC block, aero block, rejections.
- `ps/io/electronics_io.h` load path.
- New public header `ps/vehicle/assist_unit_desc.h` (AssistUnitDesc, TractionModeParams, LaunchAssistDesc,
  AeroControlDesc).
- traction_control.h: `step_traction_control` takes the mode params. Mode 1 uses the base desc, so its code path is
  identical.
- ManualTcu: `Sensors::ground_speed_mps`, the D-LS2 lock inhibit and the D-LS1 capacity cap input
  (`TcuRequest::clutch_capacity_cap_nm`, +inf = none).

Docs to update after implementation:
- electronics.md, vehicle.md (assist_unit attachment), engine.md (pedal_map recommendation for e-throttle cars);
- AERODYNAMICS.md (automation section);
- CLAUDE.md layout entries: assist_unit, the test files, [assist] tag (owner or main session; not this step);
- vault: draft tickets in the progress file;
- racing_game: remove the session.cpp wing law, map the TC-mode button to the codes, LC input (throttle + brake);
- tools/ci.ps1 PathRules: nothing new if the tests carry [assist] and [vehicle] — PROPOSAL only.

### B6. Validation plan: bands committed before the first run

All on car_hyper unless stated; rig = the Part A crawl rig promoted to a non-hidden fixture where noted. Numbers are
[AS] pass bands chosen now.

| id | case | pass band |
|---|---|---|
| V0 | every shipped scenario hash; car_hyper without assist_unit over a 10 s drive state hash; TC mode 1 vs today's TC on | bit-identical |
| V1 | grass 0.42, 8 %, v0 0.4, pedal 0.5, TC Off/On/Relaxed/Sport | v at 8 s >= 1.5 m/s; 0 stalls |
| V2 | grass 0.42, 12 %, v0 0.4, WOT, TC On/Relaxed | v at 8 s >= 2.5 m/s (TC off reaches 2.9 [M]); anti-stall dumps = 0; rpm_min >= 900 |
| V3 | grass_rg 12 % WOT TC On | v at 8 s >= 8 m/s (today 3.1 [M]) |
| V4 | stall hunt, all 180 rows | 0 stalls; with TC On/Relaxed/Sport, rpm_min >= anti_stall 850 and dumps = 0 |
| V5 | D-LS4 pedal map: grass 5 %, pedal 0.3 | v at 8 s >= 1.0 m/s; pedal 0 still gives DFCO/motoring on overrun |
| V6 | mode ordering, grass_rg 5 % WOT | mean rear kappa On < Relaxed < Sport < Drift; Drift peak yaw rate in a 30 m/s step steer > 1.3x On |
| V7 | unavailable-mode mapping: standard unit with codes 2-4 | effective = On; Off stays Off |
| V8 | LC on asphalt from 0 | Armed rpm within +-150 of launch_rpm; boost at release >= 0.4 bar; mean rear kappa over 0.2-1.0 s after release within target +-0.05; 0-100 km/h <= TC-On WOT clutch-dump launch; no stall; wing offset within 1 deg of launch_offset by release |
| V9 | LC rejection | two-step + assist_unit LC is rejected with the message |
| V10 | aero port | wing command equals the racing_game formula to 1e-12 over v in {0, 5, 10, 15, 20, 25, 40} x brake in {0, 0.45, 0.7, 1}; airbrake inactive below 20 m/s |
| V11 | determinism | 1 vs 10 workers equal hash for V2, V8; zero own allocation in the step with all assists active |

Each of V1-V4 and V8 carries a recorded sabotage (the old coupling restored must fail).

### B7. Increments

| inc | content | who | cost [AS] | depends |
|---|---|---|---|---|
| I1 | assist_unit kind, TC mode table, mode mapping, snapshot fields, V0/V6/V7 | Sonnet | 1 session | — |
| I2a | diag: hub omega / kappa_eq through a dump (F4) | Sonnet | 0.3 session | — |
| I2 | low-speed fix D-LS1..D-LS3 (TCU capacity cap, lock inhibit, launch-law decoupling), V1-V4 | **Opus** (control interaction, hash discipline, sabotages) | 1-1.5 sessions | I1 |
| I3 | D-LS4 car_hyper pedal_map (data, after Q3), V5 | Sonnet | 0.2 session | Q3 |
| I4 | launch-control assist (B3), V8/V9 | Sonnet with Opus review | 1 session | I1, I2 |
| I5 | aero automation (B4), V10 | Sonnet | 0.5 session | I1 |
| I6 | docs, racing_game migration brief, V11 sweep | Sonnet | 0.3 session | I4, I5 |

Order: I1 -> I2a and I2 -> I3 -> I5 -> I4 -> I6. I5 can run parallel to I2 because it touches different files. I4
waits for I2 because both edit the ManualTcu clutch path.

### B8. Owner questions (each with a recommended answer)

| # | question | recommended answer |
|---|---|---|
| Q1 | One channel with codes 0 Off, 1 On, 2 Relaxed, 3 Sport, 4 Drift? | Yes: append-only codes, 1 = today. |
| Q2 | car_hyper default: fit a performance assist_unit with TC default On? | Yes, default mode On. Today car_hyper has no TC block and defaults to Off; racing_game overrides anyway. |
| Q3 | Give car_hyper a progressive pedal_map (pedal 0.4 -> about 0.52 command)? | Yes, as data. Later decide on a global request-law change for all simulated engines (moves hashes). |
| Q4 | Unavailable mode: map to the nearest more-intervening mode? | Yes; never substitute Off. |
| Q5 | P8 two-step and assist-unit LC on one car? | Reject in the loader. |
| Q6 | Should LC auto-enable anti-lag while Armed? | No: anti-lag stays its own switch. |
| Q7 | LC arming thresholds (throttle > 0.9, brake > 0.5, release brake < 0.2, launch 4500 rpm, hand-over 6 m/s / 3 s)? | Yes, as [AS] data defaults, tuned by V8. |
| Q8 | Fix the tyre low-speed spring-back (F4) even though it moves every vehicle hash? | Diag first (I2a). Fix only with a cited or derived carcass-deflection cap, as its own increment. |
| Q9 | Add creep and hill-hold to the auto-clutch (rollback at pedal 0)? | Later, as its own assist (fitted flag), not in this batch. |
| Q10 | racing_game grass 0.63 vs physics_sim 0.42? | Keep both. The crawl fix must pass on both (bands above). |
