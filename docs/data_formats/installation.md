# Installation format — `physics_sim.installation/1`

Files: `data/installations/*.json` (none shipped yet — ps_audio A4 is
data-only, see below). Loader: `ps::io::load_installation_json`
(`core/src/io/vehicle_io.cpp`). Public descriptor: `ps::drivetrain::
InstallationDesc` (`core/include/ps/drivetrain/installation_desc.h`).

An installation file describes everything an engine's own `cycle.intake`/
`cycle.exhaust` pipe pair (`docs/data_formats/engine.md`'s "intake/exhaust:
the finite-volume pipe pair" section) plugs INTO — airbox, snorkel,
catalyst, silencer/expansion chamber, tailpipe(s) and their outlet
positions — everything from the engine's own throttle-body inlet through
the first flange after the last engine-mounted part is the ENGINE file's
job; everything downstream (or upstream of the throttle body, on the
intake side) is the installation's (PLAN.md/out/audio_plan.md section 3.1's
ownership split, "D-4/Q1"). A vehicle typically shares one installation
across several engines it might carry (a swap), or overrides an engine's
own `default_installation` per vehicle — see `docs/data_formats/vehicle.md`
and `engine.md`'s own `default_installation` field.

**ps_audio A4 status: data only.** Nothing reads an `InstallationDesc` at
runtime yet — the loader validates and round-trips it. A5 wires it into
the map generator/pipe network; a later ps_audio increment wires it into
the live audio path. `data/installations/` does not exist yet; every
fixture referenced by this document and by `tests/unit/
test_engine_installation_io.cpp` lives only in that test's own temp
directory, never under `data/`.

## Top-level fields

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required | — | must equal `"physics_sim.installation/1"` | see README |
| `source` | string | **required** | — | non-empty | provenance (PLAN.md section 8) |
| `intake` | object | **required** | — | see below | airbox/snorkel/inlet |
| `exhaust` | object | **required** | — | see below | catalyst/pipe/silencer/outlet chain |

Unknown top-level keys are rejected.

## `intake`

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `airbox_m3` | number | **required** | — | `> 0` | airbox volume (a 0D reservoir in the pipe network, A5) |
| `snorkel` | object | optional | none | see below | the duct from ambient into the airbox; absent = the airbox opens straight to ambient |
| `snorkel.length_m`, `snorkel.d_m` | number | required if `snorkel` given | — | `> 0` each | snorkel pipe geometry |
| `inlet_position_m` | `[x, y, z]` array | **required** | — | exactly 3 numbers | where the snorkel (or airbox, if no snorkel) opens to ambient — audio-only (A7+), never a physics input |

## `exhaust`

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `elements` | array of element objects | **required** | — | non-empty; see below | an ordered chain from the engine-file `exhaust.interface` flange to one or more outlets |

Each element is `{"type": ..., ...}`. `type` is one of:

| `type` | Extra fields | Meaning |
|---|---|---|
| `"catalyst"` | `length_m`, `d_m`, `k_loss` (all `> 0`) | a lumped flow-loss element |
| `"pipe"` | `length_m`, `d_m` (`> 0`) | a plain length of pipe |
| `"expansion_chamber"` | `length_m`, `d_m` (`> 0`) | same fields as `"pipe"`, distinguished only by intent/labelling — A5+ may give it a distinct area profile |
| `"outlet"` | `position_m` (`[x, y, z]`) | where this chain of the exhaust ends — audio-only, never a physics input |
| `"branch"` | `branches`: array of >= 2 element chains (each itself an array of element objects) | splits one chain into two or more, e.g. a dual exhaust; PROVISIONAL — `out/audio_plan.md` describes branching only in prose, this is a minimal concrete schema with no owner ruling yet on its exact shape |

A chain (the top-level `elements` array, or any one of a `"branch"`'s
`branches` entries) must end in exactly one `"outlet"` or `"branch"`
element, and that terminal element must be the LAST element of its chain —
`"outlet"`/`"branch"` mid-chain, or a chain ending in anything else, is
rejected. Every element object's keys are validated against exactly its
own `type`'s field list; an extra or misspelled key is rejected.

## Minimal complete example

```json
{
  "format": "physics_sim.installation/1",
  "source": "test fixture, not physical data",
  "intake": {
    "airbox_m3": 0.010,
    "snorkel": { "length_m": 0.4, "d_m": 0.07 },
    "inlet_position_m": [0.9, 0.3, 0.5]
  },
  "exhaust": {
    "elements": [
      { "type": "catalyst", "length_m": 0.12, "d_m": 0.11, "k_loss": 4.0 },
      { "type": "pipe", "length_m": 1.6, "d_m": 0.05 },
      { "type": "outlet", "position_m": [-2.2, -0.45, 0.25] }
    ]
  }
}
```

A dual-outlet variant using `"branch"`:

```json
{
  "format": "physics_sim.installation/1",
  "source": "test fixture, not physical data",
  "intake": { "airbox_m3": 0.010, "inlet_position_m": [0.9, 0.3, 0.5] },
  "exhaust": {
    "elements": [
      { "type": "catalyst", "length_m": 0.12, "d_m": 0.11, "k_loss": 4.0 },
      {
        "type": "branch",
        "branches": [
          [ { "type": "pipe", "length_m": 1.0, "d_m": 0.05 },
            { "type": "outlet", "position_m": [-2.2, -0.45, 0.25] } ],
          [ { "type": "pipe", "length_m": 1.0, "d_m": 0.05 },
            { "type": "outlet", "position_m": [-2.2, 0.45, 0.25] } ]
        ]
      }
    ]
  }
}
```

## How an engine reaches an installation

An engine file with `cycle.intake`/`cycle.exhaust` pipe blocks must also
give `cycle.default_installation` — a path resolved relative to the ENGINE
file's own directory. A vehicle's engine component may override this with
its own `"installation"` field (a path resolved relative to the VEHICLE
file's own directory) — only legal on a simulated engine whose cycle block
has the pipe pair. Whichever wins is loaded via `load_installation_json`
into `CycleModelDesc::installation`, with `CycleModelDesc::
installation_path` recording which file actually got loaded (for
provenance/debugging — never itself read back into physics) and
`CycleModelDesc::default_installation_path` always recording the resolved
default regardless of whether an override was used. See `engine.md`'s
`default_installation` field and `vehicle.md`'s per-engine `installation`
field.

## Cache key

`core/src/engine_cycle/map_cache.cpp`'s `engine_map_cache_key` folds every
field in the two tables above EXCEPT `inlet_position_m` and every
`"outlet"` element's `position_m` (audio-only, never affects a generated
map) and `source` (pure citation). An engine with no pipe blocks — hence no
installation at all — has an installation-independent key, bit-identical
to before A4 (`tests/unit/test_engine_map_cache.cpp`'s H3 section covers
this table-driven, one field at a time).

## Common mistakes

- **`intake`/`exhaust` missing or not an object.** Both are required;
  `"'intake' object is required"` / `"'exhaust' object is required"`.
- **A `snorkel`/element with an extra or misspelled key.** Every object's
  key set is checked exactly against its own type's fields —
  `"unknown key '<key>'"`.
- **A chain ending in a `"pipe"`/`"catalyst"`/`"expansion_chamber"`, not an
  `"outlet"`/`"branch"`.** `"must end in an 'outlet' or 'branch' element"`.
- **An `"outlet"`/`"branch"` in the middle of a chain.** `"must be the last
  element of its chain"`.
- **A `"branch"` with fewer than 2 `branches` entries.** `"'branches' must
  be an array of at least 2 element chains"` — a single-chain "branch"
  makes no sense (use a plain chain instead).
- **A position field given as `{"x":..,"y":..,"z":..}`.** Every `Vec3` in
  this loader (like every other `physics_sim.*` format) is a 3-element
  array `[x, y, z]`, not an object.
