# ECU format — `physics_sim.ecu/1`

An ECU part file holds an engine's control calibration: idle and start
thresholds, idle control, the limiter, the throttle actuator, the lambda and
spark tables and the cam phaser schedule. It exists so a garage (or a person)
can swap a calibration without copying the whole engine file (engine garage P2,
`out/engine_garage_design.md` sections 4.4-7).

An ECU file is **not** a separate descriptor. The loader (`core/src/io/engine_parts.cpp`,
`assemble_engine`) merges it into the engine JSON (RFC 7396 merge patch) **before**
the engine reader runs, so every field below has exactly the meaning, units,
defaults and rejections it has inline in an engine file — see
[engine.md](engine.md). An engine that names an ECU part therefore loads to the
same descriptor, the same `.psmaps` key and the same state hash as the same
engine written inline.

Shipped: `data/ecus/hyper_v8_one1.json` (the calibration of
`data/engines/hyper_v8_one1_audio.json`, moved there verbatim).

## How an engine gets its ECU

| Level | Key | Paths relative to |
|---|---|---|
| engine file | `"default_ecu": "<path>"` | the file that wrote the key (an `extends` parent's own directory) |
| vehicle engine component | `"ecu": "<path>"` | the vehicle file |
| user layer | `PartOverride::slots["ecu"]` (`load_vehicle_json(path, VehicleLoadOptions)`) | the vehicle file |
| standalone engine | `EngineSlotOverrides{{"ecu", path}}` (`load_engine_json`, `load_engine_json_from_string`) | the engine file / `base_dir` |

The highest level that names an ECU picks **the** file; lower levels' ECU files
are not read. The chosen file is merged over the engine's own (inline) ECU
content. Absent everywhere = the engine's inline content only (bit-identical to
before P2).

## Fields

`format` (required, `"physics_sim.ecu/1"`), `name` and `source` are the part's
own metadata; they are not merged into the engine. Every other key must be one of:

| Key | Engine meaning ([engine.md](engine.md)) |
|---|---|
| `idle_rpm`, `stall_rpm`, `fire_rpm`, `start_complete_rpm` | engine state thresholds |
| `dfco_enable_rpm`, `dfco_reenable_rpm` | deceleration fuel cut-off |
| `idle_control` | the idle PI block |
| `limiter` | the rev limiter block |
| `throttle_actuator_tau_s` | electronic throttle actuator lag |
| `ecu.lambda_target`, `ecu.spark_advance_deg` | the simulated engine's `ecu` block tables |
| `cycle.spark_advance_deg` | the cycle block's spark schedule (copied into `ecu.spark_advance_deg` by the loader, as inline) |
| `cycle.cam` | the cam phaser schedule of the cycle block |
| `*_source` | a source note next to any of the above (top level, inside `ecu` or `cycle`) |

A part may carry any subset; it is merged as given (a partial ECU leaves the
engine's other inline fields alone). `cycle.*` keys need an engine with a
`cycle` block.

## Rejections

- `format` missing or other than `physics_sim.ecu/1` (`"<path>: unsupported 'format' \"...\" (expected \"physics_sim.ecu/1\")"`)
- any other key: `"'<key>' is not an ECU field (docs/data_formats/ecu.md lists the fields)"`,
  `"'ecu.<key>' is not an ECU field ..."`, `"'cycle.<key>' is not an ECU field ..."`
- `ecu`/`cycle` not an object
- cycle fields on an engine without a `cycle` block
- an engine file with `default_ecu` **and** inline ECU content (any key of the
  table above in the same engine object):
  `"'default_ecu' and the inline ECU field '<key>' - give the ECU either inline or as a part file, not both (a child engine removes an inherited field with null)"`
- `"none"` as an ECU (`"none"` is only for forced induction)
- `default_ecu` / `ecu` not a non-empty string

The merged engine is then validated by the engine reader as usual (ranges,
table shapes, a fuel-cut limiter's hysteresis, ...).

## `extends`

An ECU file may `"extends"` another ECU file (RFC 7396, depth <= 8, cycles
rejected, relative paths per file) — e.g. a "stage 1" calibration that changes
only `limiter.rpm`. It has no path-valued keys.

## Example

```json
{
  "format": "physics_sim.ecu/1",
  "name": "my_v8_stage1",
  "source": "[AS] +250 rpm limiter over the stock calibration",
  "extends": "../ecus/hyper_v8_one1.json",
  "limiter": { "rpm": 8500 }
}
```
