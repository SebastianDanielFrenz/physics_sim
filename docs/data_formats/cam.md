# Cam format — `physics_sim.cam/1`

A cam part file holds a simulated engine's valve lift curves — the
`cycle.intake_lift` and `cycle.exhaust_lift` blocks of [engine.md](engine.md)
(lift curve and `phase_ref_deg`) — plus `fits` metadata describing which
engines it was drawn for (engine garage P2). The cam **phaser** schedule
(`cycle.cam`) is ECU calibration and lives in an [ECU part](ecu.md).

Like an ECU part, a cam file is merged (RFC 7396) into the engine's `cycle`
block before the engine reader runs, so the lifts have exactly their inline
meaning and rejections, and the engine's `.psmaps` key, generated maps and
state hash equal those of the same engine written inline. Lift curves are
generator inputs: a different cam moves the map key (and regenerates maps).

Shipped: `data/cams/hyper_v8_one1.json` (the lifts of
`data/engines/hyper_v8_one1_audio.json`, moved there verbatim).

## How an engine gets its cam

Same levels as an ECU ([ecu.md](ecu.md#how-an-engine-gets-its-ecu)): the
engine file's `"default_cam"`, a vehicle engine component's `"cam"`, the user
layer's `slots["cam"]`, a standalone load's `EngineSlotOverrides{{"cam", path}}`.
The highest level wins; the file is merged into the engine's `cycle` block.
A cam needs an engine with a `cycle` block.

## Fields

| Key | Required | Meaning |
|---|---|---|
| `format` | yes | `"physics_sim.cam/1"` |
| `name`, `source` | no | the part's own metadata (not merged) |
| `intake_lift`, `exhaust_lift` | no | merged into `cycle` as given (a cam may carry one of them) |
| `profiles` | no | engine garage P9: discrete cam profiles (VTEC-style), see below; excludes the two keys above |
| `intake_lift_family` | no | engine garage P10: a continuous intake lift family (Valvetronic-style), see below; excludes `intake_lift` and `profiles` |
| `*_source` | no | source notes, merged into `cycle` |
| `fits` | no | compatibility metadata, checked, not merged |

### `fits` — what is checked

| Key | Type | Check against the engine's `cycle` block |
|---|---|---|
| `intake_valves`, `exhaust_valves` | positive integer | must EQUAL `cycle.intake_valves` / `exhaust_valves` (`"the cam does not fit this engine: it is for 2 intake_valves, the engine's cycle block has 4"`) |
| `intake_valve_d_mm`, `exhaust_valve_d_mm` | `[min, max]`, min <= max | the engine's valve head diameter must lie inside (`"... cycle.intake_valve_d_mm 38.000000 mm is outside fits.intake_valve_d_mm [...]"`) |
| `family` | string | not checked (a label for a garage's part list) |
| `source` | any | not checked |

Any other `fits` key is rejected (`"fits: unknown key '<key>'"`). Absent keys
are not checked. Lift geometry itself is checked only by the engine reader
(first and last lift exactly 0, ascending crank angles, ...); `fits` does not
check lift against valve-to-piston clearance — there is no such model.

## `profiles` — discrete cam profiles (engine garage P9)

A cam with two lift sets the ECU switches between (Honda VTEC and similar: a
low and a high lobe, one oil-pressure-driven switch):

```json
"profiles": [
  { "id": "low",  "intake_lift": { ... }, "exhaust_lift": { ... } },
  { "id": "high", "intake_lift": { ... }, "exhaust_lift": { ... } }
]
```

- exactly **2** profiles, low first (`CycleModelDesc::kMaxCamProfiles`: one
  ECU switching rule selects between two; three-stage VTEC is an open point);
- `id`: a non-empty string, unique; a label only (renaming it moves no cache
  key and is change class None);
- each profile carries both `intake_lift` and `exhaust_lift` in the engine's
  lift format ([engine.md](engine.md), `cycle` block); `*_source` notes are
  allowed inside a profile, any other key is rejected;
- the engine then generates **one complete map set per profile** (each with
  its own `.psmaps` file, keyed exactly like the single-lift engine written
  with that profile's lifts), and needs an ECU with a `"switching"` block
  ([ecu.md](ecu.md#switching--cam-profile-switching-engine-garage-p9)) - a cam
  with profiles and an ECU without switching (or the reverse) is rejected;
- profile 0's lifts are also the engine's `intake_lift`/`exhaust_lift` (every
  single-lift reader sees the low cam);
- a part file is the WHOLE cam: a profile cam slotted onto an engine with
  single lifts replaces them, a single-lift cam slotted onto a profile engine
  removes the profiles. Through `"extends"`, a child that turns a profile cam
  into a single-lift one sets `"profiles": null` and gives both lifts.

Shipped: `data/cams/honda_k20a_dc5_vtec.json` (written by
`tools/enginedata/k20a_vtec_cam.py`; every lift number [AS], no OEM K20A lift
data exists).

## `intake_lift_family` — continuous intake lift (engine garage P10)

A cam whose intake lift the ECU varies continuously (BMW Valvetronic and
similar: an eccentric shaft sets the intake lift and duration; the exhaust
lift is fixed). The family is a set of nodes over the normalised shaft
position `u` (0 = minimum lift, 1 = full lift):

```json
"intake_lift_family": [
  { "u": 0.0, "intake_lift": { ... } },
  { "u": 0.15, "intake_lift": { ... } },
  ...
  { "u": 1.0, "intake_lift": { ... } }
],
"exhaust_lift": { ... }
```

- **2 to 8** nodes (`CycleModelDesc::kMinLiftNodes` / `kMaxLiftNodes`), `u` in
  `[0, 1]`, strictly ascending, the last exactly `u = 1`;
- each node carries `u` and an `intake_lift` in the engine's lift format
  ([engine.md](engine.md), `cycle` block); `source` / `*_source` notes are
  allowed inside a node, any other key is rejected;
- the exhaust lift stays a single top-level `exhaust_lift`;
- the u = 1 node's lift is also the engine's `intake_lift` (every
  single-lift reader sees full lift);
- the engine then generates **one complete map set per node** (each with its
  own `.psmaps` file, keyed exactly like the single-lift engine written with
  that node's intake lift - `cycle_desc_for_lift_node`), and needs an ECU with
  a `"lift_control"` block ([ecu.md](ecu.md#lift_control--continuous-intake-lift-engine-garage-p10));
  a family without lift_control (or the reverse) is rejected;
- with a family the engine needs `throttle_input` `"electronic"`, no forced
  induction, no `maps` block, and (with a knock-limited ECU) a spark table;
- a part file is the WHOLE cam, as for profiles.

`check_buildable` checks the family (rules `LiftFamilyOrder`, `LiftMinimumBand`
and `CamLiftOverValve` per node, [engine.md](engine.md#intake-lift-family-continuous-lift-engine-garage-p10)).

Shipped: `data/cams/bmw_n52b30_valvetronic.json` (written by
`tools/enginedata/n52_valvetronic_cam.py`; the 0.18-9.9 mm lift range and the
full-lift durations are cited, the node shapes, the node `u` values and the
fixed centre lines are [AS] - no OEM lift-curve set was available).

## Rejections

- format other than `physics_sim.cam/1`
- any other top-level key: `"'<key>' is not a cam field (a cam part sets intake_lift, exhaust_lift, profiles, fits)"`
- `profiles` next to a top-level `intake_lift`/`exhaust_lift`:
  `"a cam with 'profiles' carries intake_lift/exhaust_lift inside each profile, not at the top level"`
  (an engine `cycle` block writing both: `"'profiles' and 'intake_lift'/'exhaust_lift' together ..."`)
- `"'profiles' must hold exactly 2 profiles ..."`, `"duplicate profile id '<id>'"`,
  `"'id' must be a non-empty string"`, `"unknown key '<key>' (a profile has id, intake_lift, exhaust_lift)"`
- `profiles` and `intake_lift_family` together:
  `"a cam has either 'profiles' (P9) or 'intake_lift_family' (P10), not both"`
- `intake_lift_family` next to a top-level `intake_lift`:
  `"a cam with 'intake_lift_family' carries the intake lift inside each node, not at the top level ..."`
  (an engine `cycle` block: `"'intake_lift_family' and 'intake_lift' together ..."`)
- `"'intake_lift_family' must be an array of {u, intake_lift}"`, `"... must hold 2 to 8 nodes, got <n>"`,
  `"unknown key '<key>' (a lift node has u, intake_lift)"`, `"'u' must lie in [0, 1], got <u>"`,
  `"the nodes' 'u' must be strictly ascending (...)"`, `"the last 'intake_lift_family' node must be u = 1 (full lift), got u = <u>"`
- a family without an ECU `lift_control` block: `"the cam has an intake lift family (cycle.intake_lift_family) but the ECU has no 'lift_control' block ..."`
- a family with a `maps` block, forced induction, a non-electronic throttle, or a knock-limited ECU without a spark table
- profiles without an ECU `switching` block: `"the cam has profiles (cycle.profiles) but the ECU has no 'switching' block ..."`
- an engine with `default_cam` **and** inline `cycle.intake_lift`/`exhaust_lift`:
  `"'default_cam' and the inline cam field 'cycle.<key>' - give the cam either inline or as a part file, not both ..."`
- a cam on an engine without a `cycle` block; `"none"` as a cam
- the `fits` checks above

## `extends`

A cam may `"extends"` another cam (depth <= 8, cycles rejected, relative paths
per file); it has no path-valued keys.

## Example

```json
{
  "format": "physics_sim.cam/1",
  "name": "hyper_v8_one1_late_intake",
  "source": "[AS] intake 6 deg later than stock",
  "extends": "../cams/hyper_v8_one1.json",
  "fits": { "intake_valves": 2, "exhaust_valves": 2, "intake_valve_d_mm": [36, 40] },
  "intake_lift": { "phase_ref_deg": 2 }
}
```

Note: a merge patch replaces arrays whole, so a child that changes one lift
point must give the whole `lift_mm_vs_deg` curve; changing `phase_ref_deg` alone
(as above) keeps the parent's curve.
