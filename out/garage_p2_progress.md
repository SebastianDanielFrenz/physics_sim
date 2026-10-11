# Engine garage P2 - part slots, `extends`, the user layer (progress)

Agent: Opus (garage P2), worktree branch `worktree-agent-a10fc0e9ee7627080`, base master cb1f475.
Brief: scratchpad briefs/garage_p2.md; design racing_game out/engine_garage_design.md sections 5, 6, 7, 11.4, 15.1.

## Baseline (master cb1f475, release, ps_run --workers 1 / 10, identical at both)

free_fall 0x01f1d2661fa9abfe, pendulum 0xb3efb3973ae51a06, joint_chain 0xdb801a6117cf77f4,
articulation_chain 0xf0017629d03b426d, multi_articulation 0x4231f6d23b97b61a, contact_articulation 0xb30f268333db87d5,
terrain_tiles 0x5afffd7d796d4f34, terrain_multi_interest 0xd27f5f629161cee4, terrain_prefetch_drive 0xe15af43bd0bc199c,
terrain_prefetch_multi_interest 0x03813a5308e97990, vehicle_step_steer 0xe77fd6569d82786c,
vehicle_powertrain 0xaadddd7bcd24ac15, vehicle_tractor_i4 0x44327cc4b10c57b5, vehicle_fuel_tank 0xb08736ed4b3a8d4f,
vehicle_sim_engine 0x3aed0da9057e5829, vehicle_gen_engine 0x194fc20db754f52c, environment_day 0xce55f12949e0ee73,
environment_weather 0xa3ca509638ae73cc, environment_surface 0xcef127a9c2643925, vehicle_thermal 0x3178e9e670d1cb5d.

## Design (decided before coding)

### D1 Where slots act: on the JSON, before validation ("assemble, then read")
Every slot that carries engine-body content (ecu, cam) is merge-patched (RFC 7396) into the engine's JSON object
before the existing reader (read_engine_fields) runs. The reader, every rejection and the .psmaps / turbo-table keys
then see exactly what an inline file would give - an engine split into parts and the same engine written inline
produce the same JSON object (nlohmann objects are key-sorted), so the same desc, keys and state hash by construction.
Slots that name a separately-loaded part (forced_induction, installation, thermal) resolve to a path handed to the
existing loader parameter (the N7b turbo override path, the A5 installation override path, the T1 thermal path).

| Alternative | Pro | Contra | Verdict |
|---|---|---|---|
| A. JSON-level assembly (chosen) | one reader, every existing check applies to the merged content; keys equal by construction; ECU/cam formats are just restricted patches | loader works on JSON, not descs; error messages name the engine (the merged object), not the ECU file, for value errors | chosen |
| B. Desc-level overlay (read the ECU file into a partial desc, overwrite desc fields) | messages can name the part file | a second reader per part; optional/required rules duplicated; inline-vs-part equality must be proven field by field | rejected: duplication and drift risk |
| C. Materialise merged copies on disk (racing_game R6 interim) | no loader change | copies + path rewriting; already the interim | rejected for P2 (stays as the interim) |

### D2 Resolution order
1. Engine file: `extends` chain resolved first (deepest parent, each child, the file itself).
2. Engine inline content (the resolved object).
3. Engine default slot (`default_ecu`, `default_cam`, `default_forced_induction`, `cycle.default_installation`,
   `default_thermal`) - paths relative to the file that wrote the key.
4. Vehicle engine component slot (`ecu`, `cam`, `forced_induction`, `installation`, `thermal`), relative to the vehicle file.
5. User layer (`VehicleLoadOptions::part_overrides[component id].slots`), relative to the vehicle file (D6).
A slot's VALUE (the file it names) is replaced by the highest level that sets it: one ECU file, one cam file, one
forced-induction configuration is applied. Value-level edits are made inside a part file with `extends` (a user
ECU `extends` the stock ECU) - one mechanism for content, one for choice.
The winning ECU / cam file is applied as a merge patch over the engine's inline content of that part. With an
engine-level `default_ecu` the engine has no inline ECU content (inline + slot in one file is rejected), so the
file IS the ECU; on an engine that inlines its ECU, a vehicle/user `ecu` patches over the inline fields.

| Alternative | Pro | Contra | Verdict |
|---|---|---|---|
| Replace slot value, content via extends (chosen) | the design's rule 5.1; one ECU file in effect, easy to explain; user ECU = `extends` stock | a user ECU that only changes the limiter must `extends` the stock ECU | chosen |
| Patch every level's file over the previous | a two-line user ECU works without extends | three files silently combine; a vehicle-level ECU can no longer remove a default's key; differs from installation/thermal (already replace) | rejected |

### D3 `extends` and relative paths per file
`ps::io::load_json_with_extends` (ps_environment_io, already linked PRIVATE by ps_core; json_extends.cpp) gains an
overload with a rebase callback: before an ANCESTOR file's object is merged, each of its path-valued keys is
rewritten relative to the ROOT file's directory (lexically, forward slashes; absolute when no relative form exists).
The root file's own keys are untouched, so a file without `extends` (every shipped file today) is byte-for-byte the
same object. Path keys per format: engine `fuel`, `fuels[]`, `default_thermal`, `default_ecu`, `default_cam`,
`default_forced_induction` (not "none"), `cycle.default_installation`, `default_turbo_configuration` (so the rename
message fires even from a parent); turbo configuration `turbo`; turbo hardware `performance_map`; n2o bottle
`oxidizer`; nitrous kit `default_controller`; ecu, cam, installation, nitrous controller: none.
A from-memory variant (`resolve_json_extends(object, virtual path)`) serves `load_engine_json_from_string`.

| Alternative | Pro | Contra | Verdict |
|---|---|---|---|
| Rebase ancestor path keys to the root dir (chosen) | readers keep resolving against one directory; no provenance tracking | each format lists its path keys (a new path key must be added there - documented next to the list) | chosen |
| Track per-key provenance (a parallel tree of base dirs) | no per-format list | every reader would need the provenance tree; huge change | rejected |
| Absolute paths for every rebased key | simplest | desc path strings differ from an inline file even in the same directory (strings are not keyed, but the desc comparison in tests would differ) | fallback only (different drive) |
| Move json_extends into a new shared target | clean layering | ps_core already links ps_environment_io; nothing to gain | rejected (kept in ps_environment_io) |

### D4 ECU and cam formats are restricted engine patches
`physics_sim.ecu/1` mirrors the engine's own key layout (top level `idle_rpm`, `stall_rpm`, `fire_rpm`,
`start_complete_rpm`, `dfco_enable_rpm`, `dfco_reenable_rpm`, `idle_control`, `limiter`, `throttle_actuator_tau_s`;
`ecu.lambda_target`, `ecu.spark_advance_deg`; `cycle.spark_advance_deg` (the schedule-engine generator input) and
`cycle.cam` (phaser schedule)), plus metadata `format`, `name`, `source`, `extends` and `*_source` notes. Anything
else is rejected by name. Mirroring keeps one documentation of every field (engine.md) and removes the ambiguity of
where a flat `spark_advance_deg` would go (runtime ECU table or generator schedule; engine.md forbids both).
`physics_sim.cam/1`: `intake_lift`, `exhaust_lift` (each `lift_mm_vs_deg` + `phase_ref_deg`, as in the cycle block),
`fits` (checked against the engine), metadata; its lifts land in `cycle`.

| Alternative | Pro | Contra | Verdict |
|---|---|---|---|
| ECU mirrors engine paths (chosen) | no mapping layer; same docs; a spark table's destination is explicit | `ecu` object inside an ECU file reads oddly | chosen |
| Flat ECU keys with a mapping | reads nicer | spark destination ambiguous on cycle engines (generator schedule vs runtime table) - would need a flag | rejected |

### D5 Forced induction rename and "none"
`default_turbo_configuration` / vehicle `turbo_configuration` are rejected with a message naming
`default_forced_induction` / `forced_induction`. Value: a path (a `physics_sim.turbo_configuration/1` file) or
`"none"`. A vehicle/user `forced_induction` on a simulated engine WITHOUT a default is now allowed (adding forced
induction, design 7.1) - N7b's "only allowed ... whose file has default_turbo_configuration" rejection is dropped
for simulated engines and kept for torque-map engines. `"none"` on a pipe engine whose `grid_p_im_kpa` has a node
above ISA ambient (101.325 kPa) is rejected at load (also validate-only) with a message naming the fix (replace
`cycle.grid_p_im_kpa` with nodes at or below ambient in a user engine that `extends` this one); lumped engines are
left to the generator (design 7.3: their boosted nodes may generate).

### D6 User layer paths
`PartOverride::ref` and `slots` values resolve relative to the VEHICLE file's directory unless absolute - the same
rule as the vehicle-level slot they replace; the garage passes absolute paths for files in its own store.
Applied by rewriting the component's JSON keys (`ref`, slot keys) before it is read, so the user layer is exactly
"the vehicle file as if it said this". Component types: `ref` for engine, gearbox, n2o_bottle, nitrous_kit; slots
for engine (ecu, cam, forced_induction, installation, thermal) and nitrous_kit (controller). An unknown component
id, an unknown slot name or a type without `ref` is rejected.

| Alternative | Pro | Contra | Verdict |
|---|---|---|---|
| Relative to the vehicle file (chosen) | same rule as the vehicle slot; portable tests | the garage must know the vehicle path (it does) | chosen |
| Absolute only | unambiguous | tests and hand-written overrides need absolute paths | rejected |
| Relative to the engine file | matches default slots | a user file is never next to the stock engine | rejected |

## Done
- 391cf53 loader:
  - core/src/io/engine_parts.{h,cpp}: assemble_engine, the ECU/cam patches, slot reading and the per-format rebase.
  - json_extends gains rebase overloads.
  - `extends` in every engine-family part: engine, ECU, cam, turbo configuration and hardware, installation, and
    n2o bottle/kit/controller (thermal/cooling already had it).
  - Slots: default_ecu, default_cam and default_forced_induction on the engine; ecu, cam, forced_induction,
    installation and thermal on the vehicle engine component.
  - The rename: the old keys are rejected with a rename message.
  - "none", plus the pipe-grid check.
  - VehicleLoadOptions, PartOverride and load_vehicle_json(path, options).
  - EngineSlotOverrides on load_engine_json, and load_engine_json_from_string.
  - mass_kg and mass_source.
  - The rename touched data/engines/hyper_v8_one1_audio.json, data/vehicles/car_hyper_n2o.json,
    tests/unit/test_turbo_io.cpp and tests/validation/test_car_hyper_n2o.cpp (a comment only).
- 3d075e3 split and tests:
  - data/ecus/hyper_v8_one1.json and data/cams/hyper_v8_one1.json are split verbatim out of
    hyper_v8_one1_audio.json. Merging them back gives the pre-split JSON.
  - Fixture tests/support/fixtures/garage_p2/hyper_inline.json holds the pre-split engine.
  - New tests/unit/test_engine_parts.cpp (11 cases), plus an H3 case in test_engine_map_cache.cpp.
- b79ec8e docs:
  - New: docs/data_formats/ecu.md and cam.md.
  - Updated: engine.md, vehicle.md, turbo.md, installation.md, nitrous.md, README.md and data/engines/HYPER_ONE1.md.
- msvc-release configure and build of ps_core, ps_unit_tests and ps_run: clean.

## Measurements
- All 20 scenario hashes above are identical at --workers 1 and 10, both after the loader commit and after the split.
- release ctest:

  | Selection | Cases | Assertions | Notes |
  |---|---|---|---|
  | [io]~[long] | 168 | 4490 | |
  | [turbo]~[long] | 102 | 153612 | |
  | [n2o]~[long] | 47 | 127332 | |
  | [thermal]~[long] | 28 | 2542 | |
  | [dyno]~[long] | 43 | 1474 | |
  | [cycle]~[long] | | | pass, 1 pre-existing mayfail |
  | [audio]~[long] | 45 | 84258 | |
  | [hyper_clutch]~[long] | 7 | 29114 | |
  | ps_environment_tests | | | pass, 1 expected mayfail |
  | determinism ~[long] | 21 | 309 | |
  | validation [n2o]~[long] | 5 | | pass |
  | "garage P2*" | 11 | 151 | |

- garage P2 timing with a cold map cache: the worst case (hyper split vs inline) takes 4.3 s and all 11 cases
  10.3 s. No case needs [long].
- NA hyper (default_forced_induction "none"): the wide-open plenum reaches 97.87 kPa at 8000 rpm [M], so the test
  grid is {25,50,75,95} [AS].
- Sabotages (2026-10-11). Each was reverted and each made the suite fail:

  | ID | Sabotage | Failure |
  |---|---|---|
  | S1 | cam patch not merged | "cycle: 'intake_lift' object is required" |
  | S2 | no rebase callback | "cannot open file ... child\../fuels/gasoline.json" |
  | S3 | max depth 100 | the depth-9 case throws nothing |
  | S4 | pick prefers the engine default | vehicle level reads idle 900 / phase -10 |
  | S5 | PartOverride::ref ignored | 6 gear ratios, expected 7 |
  | S6 | no inline-ECU-field check | no throw |
  | S7 | "none" grid check off | no throw |
  | S8 | from_string without base_dir | "demo_i4_pipes.json: cannot open file" |

## Proposed CLAUDE.md lines
- Layout, core/src/io: "engine_parts.h/.cpp (engine garage P2): assemble_engine merges the chosen ECU
  (physics_sim.ecu/1) and cam (physics_sim.cam/1) part into the engine object at JSON level, before
  read_engine_fields. Slot resolution: extends chain < inline < default_* < vehicle component < user layer. An
  extends parent's relative paths are rebased per format (json_extends rebase overload)."
- ps/io/vehicle_io.h: "PartOverride{ref, slots}, VehicleLoadOptions{maps, part_overrides}, load_vehicle_json(path,
  VehicleLoadOptions), EngineSlotOverrides on load_engine_json, load_engine_json_from_string(text, base_dir, options,
  slots). Engine keys: default_ecu, default_cam, default_forced_induction (renamed from default_turbo_configuration;
  the old key is rejected). Vehicle engine component keys: ecu, cam, forced_induction (renamed from
  turbo_configuration), installation, thermal. "none" removes forced induction; a pipe engine whose grid goes above
  101.325 kPa is then rejected. Optional mass_kg/mass_source, in no key and no hash."
- data: "data/ecus/ and data/cams/ (P2): one hyper_v8_one1.json each, split verbatim out of hyper_v8_one1_audio.json".
- tests/unit: "test_engine_parts ([io][drivetrain][turbo], garage P2): split == inline (keys, turbo hash, state hash
  at 1 and 10 workers), extends chains, slot precedence, user layer, rejections, none, from_string; 8 recorded
  sabotages".
- docs: "docs/data_formats/ecu.md, cam.md".
- The car_hyper_n2o line: change "turbo_configuration override" to "forced_induction slot".

## Open points
- racing_game: its vehicle_setup.cpp materialiser and its own data still write default_turbo_configuration and
  turbo_configuration. Those keys are rejected now, so racing_game must migrate when it takes this physics_sim.
- engine.md "Limit findings", engine_dyno.h and limit_findings.h (P3b) use "turbo_configuration" as a PartRef slot
  label. It is not a JSON key, so I left it. Rename it to "forced_induction" with the P4/P3b owners if wanted.
- Audio-bank key heuristic (tools/common engine_bank_generator): any string in the engine JSON that ends in ".json"
  is followed as a file dependency. A part's free-text source must therefore not end in ".json".
- The hyper's audio-bank key changes because the engine JSON now names part files. This is a cache key, not a pin.
- toyota_a25a_fks_gen.json is not split: it is generator output written by tools/enginedata.
- The "none" check does not use P4's ambient-grid helpers (we worked in parallel). It compares the top grid node, and
  pressure_sampling.max_kpa, with 101.325 kPa itself. It could switch to P4's helper once both are merged.
