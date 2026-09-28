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
| `tier3_e10_cert.json` | 41,954,062 | 14.002173341183884 | 743.26 |

`tier3_e10_cert.json` (PHYS-033 P4 I7a C7) is not a blend — it is the actual
EPA Tier 3 Bin 30 certification fuel (10 vol% ethanol) `data/engines/
toyota_a25a_fks.json`'s fuel map was measured on, from EPA NVFEL Fuel
Analysis Report 26781 ("2018 Toyota 2.5L A25A-FKS Engine Tier 3 Fuel - Test
Data Package", public domain, data.gov ScienceHub), cross-checked against
SAE 2019-01-0249 Table 4. `lhv_j_kg`/`density_kg_m3` are unit conversions of
the report's own measured ASTM D240/D4052 values; `afr_stoich` is derived
from the report's measured elemental mass fractions (ASTM D5291/D5599, C/H/O
wt%) via Heywood 1988 section 3.2's stoichiometry — see the file's own
`"source"` field for the full derivation, independently re-derived (and
cross-checked against a corrupted-literal negative control) by
`tests/unit/test_cited_engine_data.cpp`.

`gasoline.json` is the pure (E0) base component and is never edited by a
blend — its LHV and AFR_s stay exactly Heywood Table 9.7's gasoline column, and
its density stays the EN 228:2012 range midpoint. `ethanol.json`
is the pure (E100) blending component: LHV 26.8 MJ/kg, stoichiometric A/F 9.0
(Heywood Table 9.7; also stated after Eq. 3.11, p. 72), density taken as
specific gravity 0.794 × 1000 kg/m³ = 794.0 kg/m³ (derived; Table 9.7 states no
reference temperature). Against water at 15°C (999.1 kg/m³) it would be
793.3 kg/m³, −0.1%, below the table's own 3-digit precision. This is
**anhydrous fuel-grade ethanol** (EN 15376 blending component), not hydrous
ethanol.

### Ethanol blend mixing rule (PHYS-033 P4 I7a C5c)

`e5.json`/`e10.json`/`e25.json`/`e85.json` are DERIVED files, not independently
sourced — each is a stated volume fraction `v` (5%, 10%, 25%, 85%) of
`ethanol.json` mixed into `gasoline.json`, computed by:

```
rho   = v*rho_e + (1-v)*rho_g            (ideal volume additivity)
w_e   = v*rho_e / rho                    (ethanol mass fraction)
LHV   = w_e*LHV_e + (1-w_e)*LHV_g        (mass-additive, heat of mixing neglected)
AFR_s = w_e*AFR_e + (1-w_e)*AFR_g        (exact in w_e: elemental composition
                                           is mass-additive; w_e inherits the
                                           ideal-mixing assumption)
```

where `rho_g`/`LHV_g`/`AFR_g` and `rho_e`/`LHV_e`/`AFR_e` are `gasoline.json`'s
and `ethanol.json`'s own loaded values (never hand-duplicated constants).
Density and mass-fraction mixing are both exact under the stated assumptions;
the LHV mixing rule neglects the (small) heat of mixing; the AFR rule is exact
in w_e (elemental composition is mass-additive); w_e inherits the
ideal-volume-mixing assumption.

**Stated assumptions** (repeated in each blend file's own `"source"` field):

- **Ideal mixing.** Any excess volume of mixing is neglected; not quantified
  here.
- **Anhydrous fuel-grade ethanol.** The blending component is EN 15376
  blending-grade ethanol, not hydrous ethanol.
- **Nominal label composition, not a certified value.** EN 228 states 5%/10%
  v/v as *maximum* ethanol content for E5/E10 (a real pump fuel may contain
  less); EN 15293:2018's scope defines E85 as nominally 85 % (V/V) ethanol,
  also allowing seasonal grades containing more than 50 % (V/V) ethanol, so a
  real pump E85 may contain considerably less than this file's 85 %. E25 has
  **no EU standard** of its own — it is a nominal label composition only,
  included because the owner asked for it, not because it corresponds to a
  specific regulated blend.
- **For scale.** EN 228:2012's density range spans ±3.7% around the
  747.5 kg/m³ midpoint; the error of the ideal-mixing assumption is not
  quantified here.

**Range cross-checks:** E5/E10's densities (749.825/752.150 kg/m³) fall inside
EN 228:2012's 720.0-775.0 kg/m³ range; E85's density (787.025 kg/m³) falls
inside EN 15293:2018's 755-800 kg/m³ range; E25: none - no standard gives an
E25 density range.

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
