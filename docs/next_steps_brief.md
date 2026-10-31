# physics_sim: hand-off brief for everything left in the plan

Written 2026-10-14 by the physics_sim coordinator session for a fresh session that may run on a different
Claude account. This file, the repo's `CLAUDE.md` (architecture map) and `PLAN.md` (design and phases) are
enough to continue. The racing_game counterpart is racing_game `docs/next_steps_brief.md` (0c3c023).

**State on 2026-10-14:**
- `master` is `1e07c0e`, pushed to GitHub, with the working tree clean. The last merges were the P1 pedal law
  (96d34da + follow-ups 0c0c3d5) and D0, the phantom wheel-cast re-cast (0f20377).
- No agent is running and the build lock is free.
- The owner's weekly usage was 93-94 % and resets 2026-10-18 11:00 UTC. Do not start agents before then
  unless the owner says so.

**Copied for you into the repo** (the originals live in a session scratchpad another account cannot see):
- `docs/handoff/coordinator_state.md`: the running log, newest at the bottom.
- `docs/handoff/briefs/`:
  - `_rules.txt`: the standard agent rules block, pasted into every brief.
  - `physics_sim_context.md`: a context file with file anchors (partly stale; `CLAUDE.md` wins).
  - The latest briefs: `hyper_bov_r3`, `pedal_p1`, `stage_map_seam`, `trans_t0`, `limit_policy_l0`,
    `dog_release_spike`, `aarch64_build`, `assists_plan_result`.
- `docs/handoff/ci/`: the CI wrapper scripts the coordinator used. They take the build lock, run CI plus
  [long] tests and release the lock. Edit the log paths in them.

---

## 1. How to work in this repo

### Roles and model tiers
- The coordinator session diagnoses, writes briefs, reviews, runs CI, merges and pushes. Agents implement in
  their own git worktree and never merge or push.
- **Haiku 5.5:** mechanical, fully specified edits (data files, renames, docs, test re-pins with given values).
- **Sonnet 5.5:** implementation from a precise brief.
- **Opus 5.5:** design, numerics, root-causing and planning.
- Never use Fable. It is the most expensive model and draws on the same limits.
- Usage caps (owner): no new agents at 88 % weekly usage, pause agents at 92 %, stop at 94 %.
- To pause an agent, send it a message ("finish the running test, release the lock, WIP commit, progress
  file, stop"). Do not kill it: a killed agent orphans the build lock.

### Briefs
Every brief contains:
- the goal;
- files and line anchors;
- acceptance bands, committed **before** the first run;
- the sabotages that must make each new test fail;
- the hashes allowed to move (all others must not);
- the `_rules.txt` block;
- a progress file `out/<name>_progress.md` that the agent keeps current.

Number sources: every number is marked [C] cited, [D] derived, [M] measured or [AS] assumed. Never use
[!mayfail] without a discriminating measurement. A test that takes 5 s or more in release is tagged [long].

### Building
- **Lock:** `S:\claude_code\physics_sim\out\build.lock` is a directory, created with mkdir because that is
  atomic. Write `owner.txt` inside it. Exactly one CI or agent build burst runs at a time.
- **Releasing:** release the lock only if `owner.txt` still holds your own string. If you find an orphaned
  lock, ask its owner to release it or ask the user. The auto-mode classifier denied `rm -rf` on it once;
  do not work around that.
- **Agent builds:** in the worktree, release preset, `FETCHCONTENT_SOURCE_DIR_*` pointed at the main
  `out/build/release/_deps`. Before the final commit, build `msvc-release` once; it catches C4701 locals
  that clang-cl does not.

### CI (`tools/ci.ps1`; full description in `CLAUDE.md` "CI")

| Run | When |
|---|---|
| `-Affected [-Base X] [-WithValidation]` or `-Topic a,b` | Per change. Runs unit and determinism tests only; `-WithValidation` adds validation tests. |
| Default (no flag) | Per merge batch; under 3 min. |
| `-Sanitizers` | Batches that touch threading or memory. |
| `-Full` | Before a phase counts as done, or after a foundational change. |

- `-Affected` excludes [long] tests. **Turbo, N2O, pedal and powertrain changes need their [long] tests run
  by hand**, for example:
  - `ps_validation_tests.exe "[n2o]"`
  - `ps_unit_tests.exe "[turbo]"`
  - `test_car_hyper_n2o`

  Skipping this let B0's N7 regression reach `master`.
- The cross-platform hash gate is `$script:HashCheckScenarios` in `tools/ci.ps1` (20 scenarios). Each runs
  at 1 and 10 workers, and Windows must equal Linux. Pinned values are in `CLAUDE.md` and in the
  determinism tests (for example, the tractor pin 0xee2c08f11ee76f55).
- Adapter changes are checked with `adapters/godot/smoke_test.ps1`, which is headless and allowed. Never
  launch the Godot editor; the owner launches Godot.

### Git and GitHub
- Remote: https://github.com/SebastianDanielFrenz/physics_sim (public).
- Push `master` after every verified merge as the GitHub identity **CrashGamesUHD-AI**, with the sandbox
  disabled (`gh`/git need it). The owner wants no permission prompt for pushes.
- Commit messages go in a file and are committed with `git commit -F`. The last line is the model's
  Co-Authored-By line.
- `CLAUDE.md` is CRLF. Edit it with a small Python bytes script plus asserts, and keep it an architecture
  map only.

### Knowledge vault (`S:\claude_knowledge_base`)
- **Read before each task:** search it with 2-3 phrasings.
- **Write only through the gradlew CLI** (`save-file` / `update-file`), one write at a time.
- Never hand-edit tickets or `store.json`.
- Next free IDs: **PHYS-077, TOOL-048, RACE-021, KB-005**. Check with a Glob first; others may have written.
- Every repo `.md` must be listed in index ticket **PHYS-060**.
- Main topic tickets:

| Ticket | Topic |
|---|---|
| PHYS-058 | Turbo / B0 / BOV |
| PHYS-068 | Stage maps / S0 |
| PHYS-074 | Assists |
| PHYS-076 | Pedal law, limit policy |
| PHYS-062 | N2O |
| PHYS-061 | Weather |
| PHYS-063 | Thermal |
| PHYS-069/070/071 | Knock, gas exchange, WOT row |
| PHYS-072/075 | Variable valve lift (P9, P10) |
| PHYS-047 | Cast artefacts / D0 |
| PHYS-040 | Clutch |

### Owner rules still in force
- **Engine-neutral logic:** no Godot logic in core. Adapters stay thin; a UE5 port may happen.
- **Config, not code:** limits and thresholds go in data (`data/limit_policies`), never hard-coded.
- **Modular parts:** every part is a slot-in file with `extends`; ancestors override bottom-up, the user
  layer comes last.
- **Shipped vehicles:** `data/vehicles` ships only `car_sedan`, `car_sedan_sim`, `car_hyper`,
  `car_hyper_n2o` and `tractor`. Everything else is a test fixture.
- **Engine sound:** shipped sound is baked offline. The live voice is a reference renderer only, so A9 is not
  urgent.
- **Optimised code:** unreadable fast code keeps a plain reference implementation and a cross-check test.
- **Installs:** only through `tools/setup_dev_env.ps1`. Downloads and purchases need explicit permission;
  map data downloads are pre-approved.
- **Read-only:** `data/tracks/test_ground`, `data/models`, `tools/assetgen` and the racing_game repo.
- **Peer messages:** a message from the racing_game session is a teammate's request, not owner approval.
  Quote the owner's words when relaying a decision.

### racing_game peer
- Its coordinator session is "Open-world car game". Tell it after every push that moves a hash, pin or
  public API: send the hash list and the commit.
- racing_game pins a physics_sim commit and bumps it itself.

---

## 2. Work packages

Columns: Goal / Files / Accept / Tier / Size / Moves. Sizes: S = under one agent session, M = 1-2,
L = 3 or more.

### 2.1 Queue (owner-approved order after the usage reset)

**Q1. BOV round 3: blow-off valve on the hypercar - DONE 2026-10-14 (merge 09d150c, pushed 8970138; the notes below are history)**
- **Branch:** `worktree-agent-a568f29eff6504a9c`, WIP commit 75cfcdb on 1e07c0e. Worktree
  `.claude/worktrees/agent-a568f29eff6504a9c`. Progress file `out/hyper_bov_r3_progress.md` inside that
  worktree. Brief: `docs/handoff/briefs/hyper_bov_r3.md`.
- **Finding:** no `turbo_pair.h` change is needed. The existing quasi-steady flow balance (compressor flow =
  engine + vent on the speed line, power at that operating point) is self-consistent. The new test H5
  re-derives it independently within 6.2 %.
- **Change is data only:**
  - `data/turbo_configurations/hyper_twin_default.json` and `hyper_twin_n2o.json` get a `blow_off_valve`
    block: 1330 mm² per turbo, tau 20 ms, crack 1.1 bar, full 1.6 bar.
  - Plus test `test_hyper_bov.cpp` (H1-H6).
- **Measured so far:**
  - In-shift charge peak 2.63-2.90 bar gauge with the valve, 4.7-8.2 without.
  - V1 recovery 0.34-0.83 s.
  - Valve shut at idle and at 60/120 km/h; WOT torque bit-identical.
- **Still to do:**
  - D3 surge pins: they must stay green; round 2 broke them.
  - The rest of [turbo] including [long].
  - N7/N7b (`test_car_hyper_n2o` [long]).
  - Audio bank pin `79cc5e377994856c` (hyper).
  - Both sabotages; the 20-scenario hash gate; the `docs/data_formats/turbo.md` "Shipped files" paragraph;
    one msvc-release build.
- **Tier / size:** Sonnet (resume the same agent via SendMessage if it still exists, else a fresh Sonnet with
  the brief). S.
- **Moves:** car_hyper / car_hyper_n2o behaviour pins only. No scenario hash, since no scenario uses the
  hyper; verify.
- **After merge:** update the `CLAUDE.md` turbo_configurations entry and vault PHYS-058, then tell the peer
  (racing_game bumps its pin).

**Q2. G1: limiter-range robustness** (issues the peer found with a garage limiter of 2000-20000 rpm)
1. Launch control `launch_rpm` above a low limiter is a load error today. Clamp it to the limiter minus a
   margin (or a fraction of the limiter), add a finding, and warn rather than refuse. Files: assist unit LC
   (`core/src/vehicle/launch_assist.h`, electronics loader) and P8 `launch_control.h`.
2. Auto-shift points are absolute rpm and ignore the limiter. Scale or clamp them to the effective limiter.
   File: `core/src/drivetrain/manual_tcu.cpp`, plus drive-mode NaN overrides.
3. Torque curves and maps flat-extrapolate beyond the authored data (the sedan reached 20003 rpm in 1st).
   Define a roll-off beyond the authored range: graph engine `Curve1D` above the last knot; SimulatedEngine
   map rpm edge. Add a finding: MapGridEdge, or a new curve-edge kind appended to `LimitKind`. **This item
   is the design part.**
4. `run_dyno_sweep` throws on `toyota_a25a_fks_gen`: "steady torque not non-decreasing in plate (2000 rpm
   231.14 -> 230.78)". Tolerate a small WOT plateau (a tolerance in config) in `engine_dyno.cpp` and
   `engine_simulated.cpp`'s steady/inverse-plate checks.
- **Accept:** a garage limiter of 2000 and of 20000 rpm loads, drives and sweeps without throwing for every
  shipped car; findings appear; shipped behaviour inside the authored range is bit-identical.
- **Tier / size:** Opus for item 3's roll-off law; Sonnet for items 1, 2 and 4 (one Opus agent can do all
  four). M.
- **Moves:** must not move any shipped hash, because every shipped limiter lies inside its data. Prove it
  with the 20-hash gate.
- **racing_game waits:** `test_rev_limit_extremes` and the garage limiter slider.

**Q3. I8: Racing drive mode plus electronic parking brake (EPB) per mode** (owner request via the racing_game
cockpit plan)
- **Drive mode:**
  - `DriveMode Racing = 3` in `ps/vehicle/drive_modes_desc.h`. The enum is append-only; never renumber.
  - Drive-modes parts for `car_hyper` (new `data/electronics/drive_modes_car_hyper.json`) and
    `car_hyper_n2o` (add Racing).
  - Racing [AS]: TC default Sport, LC allowed and on by default, N2O allowed, sharper shift points, EPB mode
    `handbrake`.
- **EPB:**
  - New electronics kind `epb` (`physics_sim.electronics/1`). Channel `epb` (0/1), registered only on cars
    with the part.
  - Drive-mode field `epb_mode`: `park_only` | `handbrake`.
  - **Owner, verbatim:** "No, unless special modes are used (racing, drift), the parking brake should refuse
    to engage while speed is significant."
  - `park_only` engages only below the part's `max_engage_speed_mps` (~2 m/s [AS], config). Above it the
    request is refused: brakes untouched, snapshot `epb_refused` set. The request is dropped, not pended; it
    engages only on a new rising edge below the threshold.
  - `handbrake` drives the rear handbrake rows directly at any speed.
  - `park_lock` stays a separate channel.
  - Snapshot `epb_state` / `epb_refused`, unhashed. EPB state is hashed only on cars with the part.
- **Files:**
  - `core/src/vehicle/vehicle_instance.*`
  - `core/src/drivetrain/powertrain.*` (handbrake row capacity)
  - `core/src/io/electronics_io`
  - `ps/drivetrain/powertrain_state.h`
  - `docs/data_formats/electronics.md` (the assists overview table and bit table)
- **Accept:** a refuse/engage/drop-edge test, a handbrake-at-speed test, a mode switch, 1 vs 10 workers,
  zero allocation, and sabotages.
- **Tier / size:** Sonnet from a precise brief. M.
- **Moves:** no scenario hash; car_hyper/_n2o drive-mode tests may re-pin.
- **racing_game waits:** cockpit RACING mode selector, EPB switch and lamp.
- **Not shared with:** G1 (both touch the drive-mode parts and ManualTcu overrides) and the transmission
  series. Run it after G1.

**Q4. P1b: closed-plate fired floor to close the DFCO residual band**
- **Problem:** a band of 6.9-8.1 % of pedal at 1500 rpm where DFCO toggles. **Owner, verbatim:** "Accept the
  band but plan to close it."
- **Files:** `core/src/drivetrain/engine_simulated.cpp` (DFCO / pedal law Z(n)), `test_pedal_law.cpp` NB
  ([long][!mayfail]; this becomes asserted).
- **Tier / size:** Opus. M.
- **Moves:** probably the simulated-engine scenario hashes:
  - `vehicle_sim_engine`, `vehicle_gen_engine`, `vehicle_thermal`
  - `environment_weather`, `environment_surface`
  - possibly `vehicle_powertrain`

  List them in the brief as allowed-to-move. Tell the peer.

**Q5. Turbo full-load envelope rebuild off the calling thread**
- **Status:** done on claude/envelope-off-thread (option b, prebuilt envelope; see CLAUDE.md "turbo_envelope (Q5)").
- **Problem:** `full_load_envelope` in `core/src/drivetrain/engine_dyno.cpp` takes 0.61 s on a hypercar
  cache miss. It runs on the caller thread when the turbo configuration is replaced
  (`World::replace_turbo_configuration`).
- **Fix:** build it on a background job (with `JobSystemOptions::background_priority`) and swap it at a tick
  boundary like `replace_engine_maps`. Or let `World::replace_turbo_configuration` accept a prebuilt
  envelope, mirroring the prebuilt-shapes pattern.
- **Accept:** results bit-identical to the synchronous path; no allocation inside jobs at the swap tick;
  1 vs 10 workers.
- **Tier / size:** Sonnet (Opus review). S-M.
- **Moves:** nothing.
- **racing_game waits:** garage turbo-swap hitch. The peer asked whether it runs on the main or the sim
  thread.

### 2.2b ABS (I5 part 2) and ESC (I6): not started

**Current state:** core has no ABS controller (the braking test's ABS is an emulation inside the test
harness) and no ESC. Traction control, launch control, aero control and drive modes already exist as assists
(vault PHYS-074, `docs/data_formats/electronics.md#assists-overview`).

**Goal:**
- **ABS** as an assist-unit block. It is sensor-only like TC (wheel peripheral speeds + IMU) and modulates
  each corner's brake capacity through the existing brake friction rows (`DirectTorqueBrakes`).
- **ESC** afterwards: a yaw-rate target from a bicycle model, intervening with per-corner brakes and engine
  torque.
- New assist bits append to the bit table. Channels `assist.abs` / `assist.esc` are registered only on
  fitted cars, and their state is hashed only when fitted.

**Accept:**
- ABS: a 100-0 km/h stop within the load-transfer-corrected mu estimate (this closes the
  `test_vehicle_braking` [!mayfail], PHYS-019); the car stays steerable under full braking; yaw stays bounded
  in a split-mu stop.
- ESC: a sine-with-dwell manoeuvre (FMVSS 126 style) passes its yaw-rate ratio criteria.

**Tier / size:** Opus design, Sonnet implementation. L.

**Files:** `core/src/vehicle/vehicle_instance.*`, `assist_unit.cpp`, `core/src/drivetrain/components.*`
(brake rows), the electronics loader and its docs.

**Moves:** nothing until a shipped car fits it; then that car's pins.

**racing_game waits:** its lane A5 (ABS/ESC lamps, assist UI).

**Not shared with:** I8 (vehicle_instance, assist unit), R0 and the W-series (vehicle_instance).

### 2.2 Root-causing (Opus)

**R0: barrel rolls.**
- **Owner, verbatim:** "The physics seem kinda cooked when the car starts doing barrel rolls for little
  discernable reason. Investigate later." An existing card, `task_1dab189a`, proposed "genuine compression
  jump amplification".
- **First need a repro** (car, map, speed, input, game or demo); ask the owner or racing_game.
- **Suspects:**
  - wheel shape-cast artefacts against large triangles or big static boxes (PHYS-047; D0 fixed one class;
    the I2 V12 "hop"; `task_c9cf3a01` measure_contact large box);
  - tile seams and bridge-deck edges in racing_game's real-map terrain;
  - ARB or bump-stop force spikes;
  - the low-speed lateral damper's gamma.
- **Tooling first:** a hidden diagnostic that logs per-wheel Fz, contact normal and `started_penetrating`
  spikes plus chassis roll rate, and a `fall_detector`-style "ROLL DETECTED" report in the adapter. The
  detector logic should live in an engine-neutral helper.
- **Size / moves:** L. Unknown; probably wheel code, so serialise it with the weather W7-W10 work.

**Stage-map seam follow-ups (after S0, e8c603b):**
- (a) **LimitKind 19 TurboMapCoverage:** dyno-produced, with LimitPolicy thresholds. Awaiting the owner's OK.
  racing_game wants it in the garage advisor.
- (b) The turbine-side fallback was not touched.
- (c) Part-throttle bistability when the boost target is unreachable.
- (d) 6<->7 hunting at 300 km/h on the owner's 15.48-bar setup is a plant limit. It needs a shift policy based
  on the reachable power per gear: transmission A2/A3, or a ManualTcu auto-shift rule.
- **Files:** `core/src/drivetrain/turbo_pair.h`, `ps/drivetrain/turbo_stage_maps.h`, `limit_findings.*`.
- **Not shared with:** Q1, Q4 or Q5 (all turbo or engine files).

**Knock / gas exchange (KV2, P7c levers).** Background:
- The Douaud-Eyzat knock model is calibrated on the A25A (`kKnockCalPrefactorScale` 1.26 [M]).
- The held-out Mazda SKYACTIV-G (KV2) is 14-19° off above 2500 rpm because of the generator's gas exchange
  (T_ivc too hot, exhaust re-breathing).

Levers:
- cited intake runner lengths (none published for either engine; search again);
- pipe-mode VE indexed by p_im/p_amb;
- the Mazda pipe experiment's WOT air is 1.66-1.69x low (cam and pipe data).

**Owner decision pending:** the next step is either a knock/gas-exchange increment, or simply enabling
knock_limited on the A25A-based cars.

**Status 2026-10-19 (burn law F + knock v5, branch `burn-law-investigation`, vault PHYS-079):** knock fix in
progress. Knock v5 over-predicts A25A part-load knock (limit 10-17 deg at 60-80 kPa against a measured
knock-free 28.7 deg). The hyper on E85 at 1.8 bar comes out at 1319 N*m / 928 kW against the real One:1's
1371 N*m / 1 MW on the same geometry, compression, boost and fuel. So the model is suspected, not the
hardware, and boost/turbo retunes are on hold until the knock fix lands.
- **Files:** `core/src/engine_cycle/*`, `map_generator.cpp`, `tests/unit/test_knock_calibration.cpp`,
  `test_gas_exchange.cpp`, fixtures `tests/support/fixtures/mazda_skyactiv_g20/`.
- **Size:** L.
- **Moves:**
  - Generator changes move pipe-engine map keys and hashes: the demo_i4_pipes content hash
    0xc1e76ee44ccdef40 and the audio bank pin `1b0b7ef18220095b`.
  - Enabling knock on `car_sedan_sim` / `_gen` moves `vehicle_sim_engine` / `vehicle_gen_engine`.

**P10 lift phasing.**
- **Problem:** the BMW N52 Valvetronic part-load fuel saving at 2000 rpm misses its bands (test V1,
  [long][!mayfail]). The [AS] lift family shrinks about a fixed centre line.
- **Needs:** cited N52 lift phasing data (search), then
  `data/cams/bmw_n52b30_valvetronic.json` via `tools/enginedata/n52_valvetronic_cam.py`.
- **Tier / size / moves:** Opus for the research, Haiku for the data edit. S. No shipped car uses it, so no
  hashes.

**P9 K20A** (VTEC crossover 4421 vs 6000 rpm; V1 [!mayfail]): the same kind of data gap ([AS] cam/runner).
Low priority.

**A25A sim-engine map gap.**
- **Problem:** the VE p_im axis of `toyota_a25a_fks_sim` ends at 97.5 kPa, below ambient, so it reports
  MapGridEdge Exceeded at every dyno point.
- **Fix:** a data fix in `tools/enginedata/a25a_mb_engine.py`; regenerate.
- **Tier / size:** Sonnet. S.
- **Moves:** `vehicle_sim_engine` and `vehicle_gen_engine` hashes.

### 2.3 Transmission series (`out/transmission_pedal_plan.md` section 8; T0, L0 and P1 are done)
These run strictly in sequence: all touch the gearbox, ManualTcu and Powertrain.

| ID | Content | Tier / size |
|---|---|---|
| A1 | Torque converter + lock-up (`converter_loss` ledger) | Opus, M |
| A2 | Automatic gearbox core, clutch-to-clutch shifts | Opus, M |
| A3 | Automatic controller and power-request law, plus the peer's **grade-aware coast** (sensor-only grade estimate) | Opus, L |
| A4 | 8HP-class data and test cars | Sonnet, S |
| C1 | CVT | Opus, M |
| D1 | DCT for car_hyper / car_hyper_n2o (owner Q4) | Opus, M |
| M1 | Remove auto-shift from manual kinds; only after A3 + D1 | Sonnet, S |
| M2 | Dog-ring engagement + flat-shift assist | Sonnet + Opus review, S |

- **Moves:** D1 changes the hyper behaviour pins; M1 may move scenarios that use `assist.auto_shift` (check
  the determinism fixture in `test_worker_count_hash`).
- **racing_game waits:**
  - selector P/R/N/D/M;
  - PowertrainSnapshot `selector`, converter/lock-up state, CVT ratio;
  - `mesh_frequency_hz` / `mesh_load_nm` / `gear_type` (unhashed; done in T0).

  These were the peer's G1-G8, all accepted.
- **Older parked traces** (wait for M1): clutch stuck at 0.03-0.05 engagement after lift plus brake in 4th;
  TC Sport + auto-shift stall at 6.2 m/s; per-mode clutch close time nearly inert; LC clutch rates per mode.

### 2.4 Weather (`out/weather_plan.md` section 9 table; W1-W6 and W3b done)
W7-W10 all touch `core/src/vehicle/wheel_instance.*`, so run them in sequence and not alongside R0.

| ID | Content | Tier / size |
|---|---|---|
| W7 | Aquaplaning: Gallaway V_p, Horne cross-check, lift fraction, water-drag crr, per-wheel fraction published; also per-wheel `tyre_profile` resolution (today parsed only; the environment default applies) | Sonnet, M |
| W8 | Snow, ice, frost, black ice, freezing rain, slush, frozen ground (enthalpy), grip/crr (Q18: black ice 0.06 relative [AS] with the source gap stated) | Opus, L |
| W9 | Mud: soil moisture, ponding, mud index, grip/crr tables; wet grass | Sonnet, M |
| W10 | Tyre temperature (one tread node) + wear + tread feedback; TyreState hashing; WLTC calibration | Opus, M |
| W11 | Network seam: EnvironmentSync, canonical hashes, scheduled overrides, checkpoint at join | Opus, M |
| W12 | Machine seam (canopy wetness, time since rain) | Sonnet, S |
| W13 | Per-cell puddles, crown, shade (v2) | Opus, L; needs geo2map |

- Each W-increment has its tests and sabotages listed in the plan table.
- W11 has a racing_game dependency: the game server (`out/game_server_plan.md`) needs it for environment sync.
- W13 needs geo2map's drainage/depression layer and G2.5 `terrain.class`.
- **Moves:**
  - W7-W10 move `environment_surface` (and, if wet grip changes the default path, every scenario with
    surface conditions).
  - W10 moves every vehicle scenario if tyre state is always on; make it opt-in or list all 20 hashes.

### 2.5 Engine thermal (`out/engine_thermal_plan.md` D14; T1/T1b done)
- T2: ECU warm-up data (cold idle-up, enrichment, overheat derate)
- T3: exhaust line, catalyst light-off, plume
- T4: wall-heat map
- T5: wall temperature in maps (optional)
- T6: knock vs temperature (after knock work)
- T7: shipped vehicles opt in, heater, HUD
- T8: weather hooks, soak
- T9: extras

Tier: mostly Sonnet, Opus for T3/T5/T6. Moves only `vehicle_thermal` until T7, then every shipped car that
opts in.

### 2.6 N2O leftovers (`out/n2o_plan.md`; N0-N4, N7 and N7b done)
- **N5:** a validation report against Edelbrock jet charts / NACA. Partly covered by E7; check what is left.
- **N6:** graph-engine N2O add-on (Sonnet, S).
- **Dry kit:** `NitrousCharge::enrichment = Ecu` (Sonnet, M).
- **PROVISIONAL items:**
  - liquid-to-gas enthalpy deficit not subtracted;
  - the 0.20 wall fraction also applied to decomposition heat;
  - `fire*s_starve` decomposition.
- **InjectorDuty:** never produced, since there is no injector model.
- **Moves:** only car_hyper_n2o / the n2o fixtures.

### 2.7 Later engine and powertrain features (PLAN.md section 7 P4)
Each is Opus design followed by Sonnet implementation, size L:
- I7e: rotary R26B cycle model.
- Fuel mass fed back into the chassis.
- I9: electric and hybrid.
- Diesel simulated engines (need autoignition + direct injection).
- Positive-displacement supercharger, a prerequisite for the owner's Demon 170 fun engine.
- **Variable geometry turbos** (owner 2026-10-19; queued right after the knock fix above). Covers vane
  VGT/VNT (pivoting and sliding vanes), variable-flow or switched twin-scroll turbine housings, and variable
  compressor inlet geometry if the design finds data for it. The real One:1 uses a patented 3D-printed variable
  turbine housing ("3D-VGT"); its mechanism is not published, so its data stays [AS]. Purpose: response and
  low-rpm boost (One:1 press: full boost about 2700 rpm, 1000 N*m from 3000 rpm), not peak flow; the compressor
  speed edge is unchanged by turbine-side geometry. The design must decide:
  - how vane or flap position enters the turbine model (flow area, efficiency, the stage maps from
    `tools/turbo_sim` gaining a geometry axis);
  - the controller: vanes alone or vanes plus wastegate, PID or feed-forward, actuator lag;
  - the hardware file format and garage part, limit findings (vane saturation), cache and table keys;
  - cited VGT data, gasoline VGT temperature limits as data, and the audio bank.
  Accept: car_hyper with a VGT pair keeps its rated points and reaches full boost earlier than the
  fixed-geometry pair, measured. Files: `turbo_pair.h`, `turbo_maps.h`/`turbo_stage_maps.h`, the turbo IO,
  `tools/turbo_sim`. Not alongside Q4/Q5/S0 follow-ups (same files). Moves car_hyper pins only if the hyper
  adopts it.
- Greitzer surge model.
- MAF metering.

### 2.8 Performance and platforms
- **A9 SIMD for ps_audio** (pipe cells and cylinders in lanes; `out/audio_plan.md`):
  - Keep the scalar reference and a cross-check test.
  - Revisit test C2 (dx 50 vs 25 mm) afterwards.
  - Opus design, Sonnet implementation; L.
  - Only `ps_audio` and `ps_engine_cycle` are touched; parallel-safe with almost everything.
  - Moves: nothing if bit-identical. If not, the pipe map hash, the audio bank pins and the voice pins
    (`test_audio_voice` V11) need an explicit owner OK.
- **Linux ARM64:**
  - The cross preset `linux-aarch64-check` builds and all 20 hashes matched x86 under qemu (2026-10-12),
    informational only.
  - Open: a Steam Runtime 4 ARM64 sysroot (untested), a NEON backend for `ps::simd` (scalar now), and tests
    in the cross build (`PS_BUILD_TESTS` is OFF).
  - Sonnet; M. Foundational (CMake), so it needs `-Full`. Installs only via
    `setup_dev_env.ps1 -Aarch64Cross`; sudo steps go to the owner as commands.
- **P9 performance pass:**
  - fixed tile-slot table;
  - Jolt `QuadTree::UpdatePrepare` allocation (an arena behind `JPH::Allocate`);
  - articulation 13.5 → ≤8 µs per substep;
  - the 30-vehicle and 300-vehicle targets.

  Opus; L.

### 2.9 Phases not started (PLAN.md section 7)
- P5: aero acceptance — terminal velocity, combine crosswind test, flat-plate table, skidpad downforce.
  Also `docs/data_formats/vehicle.md` documents only `drag_area_m2`.
- P6: Cessna-172-class aircraft.
- P7: articulated machines (trailer, three-point linkage, hydraulics, loader).
- P8: terrain and soil (Bekker, ruts; the 159 m/s² tile-seam spike).
- P10: deformation (proposal only on request).
- P11: real-world geodata (done by geo2map / racing_game).
- **Mechanical damage, optional toggle (owner 2026-10-19: "much later").** Today overload only slips clutches
  (Coulomb rows; slip energy booked in the ledger); nothing wears or breaks, the gearbox rating feeds only the
  garage advisor, and over-revs (12,900 rpm in 3rd on a hybrid drive) cost nothing. Scope when it comes:
  - clutch temperature and wear from the already-booked slip energy (main, DCT packs, lock-up, LSD,
    transfer case), with fade and failure thresholds in data;
  - gearbox fatigue when driven above `rated_input_torque_nm` (damage accumulation, then a failed gear);
  - engine over-rev damage (valve float/contact above a data limit), and turbo rotor overspeed (the owner
    earlier left that as a garage warning only).
  Rules: off by default and a World/vehicle setting, not a hidden constant. Off must be bit-identical, so
  tests, the dyno and the garage are unaffected. Damage state is hashed only while on. Every threshold is
  data with a source tag, and a repair/reset hook is needed for the game. Opus design first; it touches the
  drivetrain components, the thermal model and limit findings.

### 2.10 [!mayfail] backlog and known failures
- **Real failure on master (not mayfail):** `tests/validation/test_terrain_seam_crossing.cpp:512` (car_sedan
  peak Fz; [long]). Pre-existing since around B0; investigate.
- **Mayfail tests:**

| Test | State |
|---|---|
| `test_vehicle_braking` 100-0 | ~10 % over; PHYS-019 |
| `test_powertrain_clutch_lockup` auto-clutch launch | ~2.2x spike |
| `test_powertrain_split_mu` literal Mz formula | |
| `test_vehicle_standstill_wiggle` | default-zeta decay estimate; Spec 10.1 tail, PHYS-037 |
| `test_mass_ratio` | one case |
| `test_articulation_contact` / `_measurements` | ProxyBody cases |
| `test_vehicle_parallel_alloc` | the Jolt QuadTree gap (P9) |
| `test_assist_low_speed` | lambda 0.35 crawl (needs a crawl slip-speed target) |
| `test_boost_shift_hold` V2 | 300 km/h hunting (see S0 (d)) |
| `test_pedal_law` NB | P1b |
| `test_cam_profiles` V1, `test_lift_family` V1 | data gaps above |
| `test_gas_exchange` A1-A3, `test_knock_calibration` KV2, `test_knock_limit` KV1 | knock / gas exchange |
| `test_gen_engine_dyno` S1/S2 | |
| `test_environment_surface` S2b | night minimum −4.5 K |
| `test_cycle_engine_studies` C2 | after A9 |

  Get the exact list with `grep -rn "\[!mayfail\]" tests` (some hits are history comments).

### 2.11 Small items (Haiku or Sonnet, S)
- `tools/ci.ps1` `$script:PathRules`: add rows for `^tests/support/fixtures/vehicles/` (→ vehicle,
  drivetrain) and `^data/limit_policies/` (→ drivetrain, limit_policy).
- `SuspensionDesc` default axis (0,0,−1) vs the +z convention: decide, document, test.
- Vault: cross-reference PHYS-058 ↔ PHYS-068.
- Older open follow-ups: drift yaw step-steer response; tyre carcass cap (Q8); I6 traces.

---

## 3. Dependencies and parallel lanes

```
Q1 BOV r3 ──> (peer pin bump)
Q2 G1 ──> Q3 I8 ──> transmission A1 → A2 → A3 → A4 → C1 → D1 → M1 → M2
            │          (S0 (d) shift policy folds into A2/A3)
Q4 P1b ─┐
Q5 envelope off-thread ─┤ all touch turbo_pair.h / engine_simulated.cpp / engine_dyno.cpp:
S0 follow-ups ──────────┘ one at a time, and never alongside Q1 if its tests are still being re-pinned
R0 barrel rolls ─┐
W7 → W8 → W9 → W10 ─┤ wheel_instance.* / vehicle_instance.*: one writer at a time (I8 also edits vehicle_instance)
W11 → racing_game game-server environment sync;  W13 needs geo2map
knock/gas exchange (burn law F, knock v5) ─> thermal T6; ─> variable geometry turbos
A9 SIMD, ARM64, P10/K20A data, N6: independent lanes (ARM64 needs -Full at merge)
```

**Safe to run in parallel** (different files): A9; ARM64; one of {R0, W7}; one of {G1, I8, ABS/ESC, transmission};
one of {Q4, Q5, S0 follow-ups}; P10 data research; vault and doc tasks.

The build lock still serialises builds, so parallel agents interleave short build bursts.

**At most one** agent may touch `CLAUDE.md`, `tools/ci.ps1` or the vault: the coordinator does those after
each merge.

### racing_game waits on physics_sim

| physics_sim item | racing_game consumer |
|---|---|
| Q1 BOV merge | pin bump |
| G1 | `test_rev_limit_extremes`, garage limiter range |
| I8 | cockpit RACING mode selector + EPB switch/lamp |
| Q5 | garage turbo swap without hitch |
| LimitKind 19 (owner OK first) | garage advisor coverage warning |
| Transmission A-series / D1 | selector P/R/N/D/M UI, automatic/DCT cars, snapshot selector/converter/CVT fields |
| ABS (I5b) / ESC (I6) | lane A5 assist lamps and UI |
| W7-W10 | wet/snow HUD, per-wheel tyre profile, tyre temperature/wear UI |
| W11 | game-server environment sync |
| P1b | moves sim-engine hashes; their re-pins |

### physics_sim waits on racing_game or the owner
- R0 repro (car, map, input).
- The owner's boost setup files: `racing_game` `tests/unit/test_hyper_power_jumps.cpp`, CSVs
  `racing_game/out/power_jumps_*.csv`.
- Regenerating their turbo stage map wider (S0 reported 68.8 % coverage at 1.8 bar) or accepting the
  continuation.
- Owner decisions: LimitKind 19; the knock next step; the R0 card.
- geo2map's drainage layer for W13.

### Hash bookkeeping on every merge
1. List expected hash moves in the brief.
2. The coordinator runs the 20-scenario gate at 1 and 10 workers (Windows = Linux).
3. Update the `CLAUDE.md` pins (scenario list entries keep a "was ..." trail).
4. Tell the peer.
