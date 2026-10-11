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

## Rejections

- format other than `physics_sim.cam/1`
- any other top-level key: `"'<key>' is not a cam field (a cam part sets intake_lift, exhaust_lift, fits)"`
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
