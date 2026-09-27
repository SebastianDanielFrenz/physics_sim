# Fuel format — `physics_sim.fuel/1`

Files: `data/fuels/*.json`. Loader: `ps::io::load_fuel_json`
(`core/src/io/fuel_io.cpp`). Public descriptor: `ps::drivetrain::FuelDesc`
(`core/include/ps/drivetrain/fuel_desc.h`) — lives in `ps::drivetrain`
because its planned consumers (an engine descriptor and a `fuel_tank`
component) are both there, not because a fuel is itself a drivetrain
concept.

A fuel file carries only bulk thermochemical/physical properties — lower
heating value, stoichiometric air/fuel ratio, liquid density — not a
combustion model. Every numeric value must be traceable to a specific,
readable source (publication, standard, table or problem, page/edition);
see each shipped file's own `"source"` field for the exact citation used
(PLAN.md section 8's "cite your source" convention, same as
`data/tyres/*.json`/`data/vehicles/*.json`).

## Fields

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required | — | must equal `"physics_sim.fuel/1"` | see README |
| `name` | string | **required** | — | must be non-empty | fuel name, e.g. `"gasoline"`, `"diesel"` |
| `phase` | string | **required** | — | must equal `"liquid"`, else `"phase \"<value>\" is not yet supported"` | phase of matter the fuel is modelled as — only `"liquid"` is implemented (a future `"gaseous"` phase, e.g. CNG/LPG/hydrogen, needs its own pressure/temperature-dependent density convention and is not modelled yet) |
| `lhv_j_kg` | real (J/kg) | **required** | — | must be `> 0` and finite | lower heating value |
| `afr_stoich` | real | **required** | — | must be `> 0` and finite | stoichiometric air/fuel mass ratio |
| `density_kg_m3` | real (kg/m³) | **required** | — | must be `> 0` and finite | liquid density at the cited reference condition |
| `source` | string | **required** | — | must be non-empty | provenance note (README's "source" convention) — every numeric value's exact citation |

Unlike `tyre.md`/`engine.md`'s mix of optional-with-defaults fields, every
field in `physics_sim.fuel/1` is required — there is no plausible default
for a fuel's own heating value or density, unlike (say) a tyre's Pacejka
coefficients, so the loader accepts no partial file.

Whether a fuel is meant for spark or compression ignition is not a field
here: any fuel autoignites given enough compression, so ignition type is a
property of the *engine* (declared on the engine descriptor), not the fuel
— a fuel's own contribution to that pairing is its ignition-quality numbers
(octane/cetane), which will arrive alongside a future knock/autoignition
model, not as a field on `FuelDesc`.

## Minimal complete example

```json
{
  "format": "physics_sim.fuel/1",
  "name": "gasoline",
  "phase": "liquid",
  "lhv_j_kg": 44000000.0,
  "afr_stoich": 14.5,
  "density_kg_m3": 747.5,
  "source": "Heywood, Internal Combustion Engine Fundamentals, Table 9.7, 'Gasoline' column; density EN 228:2012 midpoint of 720.0-775.0 kg/m^3."
}
```

(`data/fuels/gasoline.json`/`diesel.json` carry the full, precise citation
text in their own `"source"` fields — see those files directly rather than
this shortened example.)

## Shipped files

| File | LHV (J/kg) | AFR<sub>s</sub> | Density (kg/m³) |
|---|---:|---:|---:|
| `gasoline.json` (E0) | 44,000,000 | 14.5 | 747.5 |
| `diesel.json` | 42,000,000 | 14.925373 | 830.0 |
| `ethanol.json` (E100) | 26,800,000 | 9.0 | 794.0 |
| `e5.json` | 43,089,334 | 14.208799 | 749.825 |
| `e10.json` | 42,184,298 | 13.919398 | 752.150 |
| `e25.json` | 39,502,453 | 13.061831 | 759.125 |
| `e85.json` | 29,250,430 | 9.783568 | 787.025 |

`gasoline.json` is the pure (E0) base component and is never edited by a
blend — its values stay exactly Heywood Table 9.7's gasoline column. `ethanol.json`
is the pure (E100) blending component: LHV 26.8 MJ/kg, stoichiometric A/F 9.0
(Heywood Table 9.7 / Eq. 3.11), density from specific gravity 0.794 (SG × 1000
kg/m³ water-at-15°C reference; water at 15°C is 999.1 kg/m³, so this differs
from an exact reference by ≤ 0.1%) — the **anhydrous** (water-free) EN 15376
blending-component composition, not hydrous/denatured ethanol as sold at the
pump.

### Ethanol blend mixing rule (PHYS-033 P4 I7a C5c)

`e5.json`/`e10.json`/`e25.json`/`e85.json` are DERIVED files, not independently
sourced — each is a stated volume fraction `v` (5%, 10%, 25%, 85%) of
`ethanol.json` mixed into `gasoline.json`, computed by:

```
rho   = v*rho_e + (1-v)*rho_g            (ideal volume additivity)
w_e   = v*rho_e / rho                    (ethanol mass fraction)
LHV   = w_e*LHV_e + (1-w_e)*LHV_g        (mass-additive, heat of mixing neglected)
AFR_s = w_e*AFR_e + (1-w_e)*AFR_g        (exact: stoichiometric air per kg of
                                           fuel is mass-additive regardless of
                                           mixing)
```

where `rho_g`/`LHV_g`/`AFR_g` and `rho_e`/`LHV_e`/`AFR_e` are `gasoline.json`'s
and `ethanol.json`'s own loaded values (never hand-duplicated constants).
Density and mass-fraction mixing are both exact under the stated assumptions;
the LHV mixing rule neglects the (small) heat of mixing; the AFR mixing rule
is exact regardless of mixing physics, because stoichiometric air mass per kg
of fuel is a linear (mass-weighted) quantity by definition.

**Stated assumptions** (repeated in each blend file's own `"source"` field):

- **Ideal mixing.** The small negative excess volume real ethanol-gasoline
  blends exhibit is neglected.
- **Anhydrous ethanol.** The blending component is water-free (EN 15376), not
  the hydrous ethanol sometimes sold directly at the pump.
- **Nominal label composition, not a certified value.** EN 228 states 5%/10%
  v/v as *maximum* ethanol content for E5/E10 (a real pump fuel may contain
  less); EN 15293:2018 defines E85 as *nominally* 85% v/v, with seasonal
  grades permitted lower (e.g. a winter grade around 70% v/v). E25 has **no
  EU standard** of its own — it is a nominal label composition only, included
  because the owner asked for it, not because it corresponds to a specific
  regulated blend.
- **The base gasoline's own spread dominates.** EN 228's density range alone
  spans ±3.7% around `gasoline.json`'s 747.5 kg/m³ midpoint — larger than any
  error this mixing rule itself introduces.

**Range cross-checks:** E5/E10's densities (749.825/752.150 kg/m³) fall inside
EN 228:2012's 720.0-775.0 kg/m³ range; E85's density (787.025 kg/m³) falls
inside EN 15293:2018's 755-800 kg/m³ range; E25 has no EU range to check
against (its density, 759.125 kg/m³, sits between the gasoline and E85 ranges,
consistent with being a genuine intermediate blend).

No engine currently references any blend file (or `ethanol.json`) — wiring one
up is a later commit (PHYS-033 P4 I7a C5d).

## Common mistakes

- **Missing or non-`"liquid"` `phase`.** Throws `"fuel '<name>': phase
  \"<value>\" is not yet supported (only \"liquid\" is implemented)"` —
  there is no silent fallback to liquid.
- **`lhv_j_kg`/`afr_stoich`/`density_kg_m3` <= 0, non-finite, or omitted.**
  Each is a hard error naming the offending field and the value read; there
  is no compiled-in default for any of them (unlike most of `tyre.md`'s
  fields).
- **Missing `name` or `source`.** Both are required non-empty strings, same
  as `TyreDesc`/`VehicleDesc`'s own `source` convention.
- **Inventing or "typical"-izing a number.** Every shipped
  `data/fuels/*.json` file's values must come from a source that can be
  read and cited precisely (publication/table/problem, page, edition) — a
  plausible-sounding round number with no citation is not acceptable here,
  unlike some of `tyre.md`'s "published or plausible" Pacejka coefficients.
