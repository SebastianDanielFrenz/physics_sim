# Surface table format — `physics_sim.surfaces/1`

File: `data/surfaces/surfaces.json` (a single shared file, referenced by
path from a scenario's `"surface_table_path"` — see `scenario.md`). Loader:
`ps::io::SurfaceTable::SurfaceTable` (`core/src/io/surface_table.cpp`).
Public header: `core/include/ps/io/surface_table.h`.

A surface table assigns every named physical surface (asphalt, dirt,
grass, ...) a small integer `SurfaceId` plus two physical parameters
(`lambda_mu`, `crr`) that the tyre model reads. The same table is meant to
be shared by every terrain/collision source in the project (a procedural
terrain's per-cell surface, a static feature mesh's per-triangle surface)
so they all mean the same thing by, say, `SurfaceId` 3.

## Fields

| Field | Type | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `format` | string | required | must equal `"physics_sim.surfaces/1"`, else `"surface table <path>: missing required 'format' field..."` / `"...unsupported 'format' \"<actual>\"..."` | see README |
| `surfaces` | object, name → entry | **required** | must exist and be a JSON object, else `"surface table <path> has no 'surfaces' object"` | one entry per named surface |
| `description`, `fields` | free text | optional | **not read by the loader at all** | documentation-only; see "Discrepancies found" in README |

Each entry under `surfaces`:

| Field | Type | Req./opt. | Default | Meaning |
|---|---|---|---|---|
| `lambda_mu` | real | optional | `1.0` | Pacejka-style friction scale relative to dry asphalt (`= 1.0` by convention); combined with a tyre's own `lambda_mux`/`lambda_muy` |
| `crr` | real | optional | `0.0` | rolling-resistance coefficient (dimensionless; `F_roll = crr * N`) |
| `display_name`, `source` | string | optional | — | documentation-only, not read by the loader |

## `SurfaceId` assignment — alphabetical, not file order

**This is the one behaviour in this whole format most worth understanding
before editing the file.** `SurfaceTable` collects every key under
`"surfaces"` and **sorts the names alphabetically**, then assigns
`SurfaceId 0, 1, 2, ...` in that sorted order — *not* the order the keys
appear in the JSON file. This is because `nlohmann::json`'s default object
type does not preserve a file's own key order, and the project needs the
id assignment to be deterministic (PLAN.md D7) regardless of how the file
happens to be formatted or re-serialized.

For `data/surfaces/surfaces.json`'s current key set (`asphalt`, `dirt`,
`field`, `grass`, `kerb`, `low_mu`), alphabetical order gives: `asphalt=0,
dirt=1, field=2, grass=3, kerb=4, low_mu=5`.

**Practical consequence: adding or renaming a surface can silently
renumber every other surface's id.** Any other file that embeds a raw
`SurfaceId` integer directly (rather than looking it up by name at load
time) would need re-generating. As of this writing nothing in `data/`
does that — every consumer (procedural terrain specs, static collision
meshes) looks up ids by name through this same table at load time — but
it's worth checking before adding a new consumer.

A table with more surfaces than `SurfaceId` can hold (`0xFFFF`, i.e.
65535 — `kInvalidSurfaceId` is reserved as "no surface") throws
`"surface table <path> has too many surfaces (max 65535)"`; this is not a
realistic limit to hit by hand.

An id that was never assigned (e.g. `kInvalidSurfaceId`, or any id beyond
what this table's own name list produced) reads back as `lambda_mu = 1.0`,
`crr = 0.0` (dry-asphalt-like, no rolling resistance) rather than an
error — `lambda_mu_for`/`crr_for`'s deliberate safe default for "a
collision query that produced no surface id at all" (e.g. a plain
Box/Sphere shape with no per-triangle material).

## Minimal complete example

```json
{
  "format": "physics_sim.surfaces/1",
  "surfaces": {
    "asphalt": { "lambda_mu": 1.0, "crr": 0.013 },
    "grass":   { "lambda_mu": 0.42, "crr": 0.08 }
  }
}
```

(`asphalt` gets id 0, `grass` gets id 1 — alphabetical, and here that
happens to also match declaration order; it would not if a third surface
named e.g. `"dirt"` were inserted between them in the file.)

## Common mistakes

- **Missing `"format"`**, or the wrong value — same pattern as every other
  format in this project, see README.
- **Missing or non-object `"surfaces"`.** Throws `"surface table <path>
  has no 'surfaces' object"`.
- **Assuming file order determines `SurfaceId`.** It doesn't — see the
  alphabetical-assignment section above. If you need a *specific* surface
  to be id 0 (or any specific id), its name must sort first (last, etc.)
  alphabetically among the surfaces present.
- **Adding a `"fields"`/`"description"` block and expecting it to be
  validated or affect behaviour.** It's ignored by the loader; typos there
  are invisible.
