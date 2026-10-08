# Aero coefficient table format and API

Optional runtime input for offline CFD: no coefficients or lower hypercar drag are shipped by this feature. Without a coefficient_map reference the existing authored model is unchanged. Import coefficients only after geometry, solver output and validation are available. Do not enable a map by default or fabricate CFD provenance.

Vehicle aero entry (paths relative to vehicle JSON directory):

```json
"coefficient_map": {
  "file": "../aero/my_verified_map.json",
  "scope": "replace_passive_aero",
  "ground_probe": {
    "position_local_m": [0,0,0],
    "direction_local": [0,0,-1],
    "range_m": 2,
    "missing": "authored_fallback"
  },
  "wing_surface": "rear_wing"
}
```

Omit ground_probe unless ride_height_m exists. Omit wing_surface unless wing_offset_deg or wing_lift_m exists. Table enabled by explicit reference; default vehicles have no reference. The wing name resolves to an existing authored surface used as a rate-limited actuator even when its passive forces are replaced. Offset is actual State.offset_rad in degrees, relative to rest angle; it is not the input command or absolute incidence. A table without a wing axis describes one fixed configuration; callers must not infer CFD support for configuration changes from it.

Table structure (shown without coefficient rows; not a loadable physical asset):

```json
{
  "schema": "physics_sim.aero_coefficients/1",
  "frame": "ISO_BODY_X_FORWARD_Y_LEFT_Z_UP",
  "units": "SI_DEGREES_DIMENSIONLESS_COEFFICIENTS",
  "reference": {
    "area_m2": 1.9,
    "lengths_m": [2.7,2.7,2.7],
    "point_local_m": [0,0,0]
  },
  "provenance": {
    "geometry_sha256": "lowercase 64-hex SHA256 of the solved geometry",
    "solver": "solver/version and setup identifier",
    "source": "result artifact / sweep identifier"
  },
  "axes": {
    "yaw_deg": [-180,-90,0,90,180],
    "pitch_deg": [-90,0,90],
    "speed_m_s": [0,30,60],
    "ride_height_m": [0.1,0.3,1.0],
    "wing_offset_deg": [0,25,55]
  },
  "coefficients": []
}
```

Axis order is fixed: yaw, pitch, speed, optional ride height, optional wing offset. Each flattened row contains exactly [Cx,Cy,Cz,Cl,Cm,Cn], with last present axis fastest. An axis omitted does not occupy a dimension; present axes cannot be empty. All knots finite, strictly increasing; speed/height nonnegative. Dense row count must equal the axis-size product. Limit one million rows and 4096 knots per axis, file256 MiB. Unknown or duplicate map JSON keys, bad schema/frame/units, missing provenance/reference/dimensions, NaN/infinity, wrong row widths, duplicate knots and inconsistent seams are errors at load time.

## Force conventions and ownership

Let v be vehicle point velocity relative to local air, transformed to ISO body axes (+x forward,+y left,+z up). Existing ambient wind and all wake entrainment are included at the declared reference point. Yaw=atan2(v.y,v.x), pitch=atan2(v.z,hypot(v.x,v.y)); these describe vehicle-through-air velocity, NOT the incoming wind vector. Thus forward travel at yaw0 normally requires negative Cx. Side/backward force signs come from signed table rows, not a global drag-sign correction. At a pitch pole yaw is canonically0; at zero speed both angles0, q0. Rotation to world includes inverted bodies naturally.

q=0.5*rho*|v|^2 uses actual air-relative speed even if interpolation speed is clamped. F_body=q*area*[Cx,Cy,Cz]. Intrinsic moment at the declared point is q*area*[lengths.x*Cl,lengths.y*Cm,lengths.z*Cn]. All points are local offsets from chassis centre of mass; total CoM moment adds point cross F. No authored damping or turbulence coefficient scaling is added to a mapped force. Wake velocity still reduces q and changes lookup angles.

replace_body suppresses authored body/legacy CdA only, retaining all authored surfaces and fans. Its CFD integration must include body force/moment only, excluding surface/fan loads. replace_passive_aero suppresses body/legacy and every passive surface load; the CFD integration includes all passive body/wing/floor geometry at the stated configuration. Surface actuators still advance; powered suction fan forces and prepaid energy accounting always remain separate and MUST be excluded from CFD coefficients. The scope never adds mapped and replaced passive forces together.

For ride-height maps a real body-local road probe is appended after surface/fan ground samples. Misses, invalid clearance, short sample span, or non-facing normals select the complete authored passive fallback for that substep, retaining fan behavior. No artificial clearance or arbitrary free-air slice is used. Probe range is explicit; hits outside the height domain clamp to its endpoints. A free-air map should omit ride_height_m. Its valid installation/configuration is a provenance responsibility.

Yaw must span exact[-180,+180] with at least one interior knot; corresponding seam rows must match exactly. Pitch must span exact[-90,+90], and rows at each pitch pole must be identical across yaw because those angles describe the same direction. Interpolation is multilinear, deterministic and allocation-free: binary search plus at most64 corners (32 when wing lift is absent). Yaw wraps periodically; other dimensions clamp to endpoints. There is no extrapolation or sparse-cell fill.

## Native integration and telemetry

Public ps/aero/coefficient_map.h provides immutable CoefficientMap::create(CoefficientData), evaluate(array<real,5>) and content_hash(). Creation validates and allocates before stepping. Desc.coefficient_map references a shared immutable table. No disk access/parsing/allocation on aero::step. World vehicle loading resolves the file before creation; no map hot swap is added.

Telemetry: coefficient_map_active, coefficient_map_ground_missing, coefficient_map_clamped_axes, coefficient_map_inputs (yaw,pitch,speed,height,offset), coefficient_map_coefficients. Clamp mask bits0..4 follow axis order; missing ground is separate. Existing body force/moment and total wrench report mapped results; replaced surfaces have zero passive wrench while their angle/offset remains reported. World snapshots carry these fields through native aero telemetry; adapters may expose them as needed.

Map content/provenance and reference scope/probe/wing binding contribute to vehicle state hashes only when a map exists. Paths do not affect hash. Reset keeps immutable map configuration but resets actuator state/commands as before. No-map hashes preserve existing folding. CFD-result CSV conversion and actual solver execution live outside the core.

## Consumer-owned selection file

Public header ps/io/aero_coefficient_map_io.h exposes:

```cpp
ps::io::load_aero_coefficient_map_reference(selection_json_path, cloned_vehicle.aero);
```

Selection root is exactly `{ "enabled": false }` when disabled, or `{ "enabled": true, "coefficient_map": { ...the reference above... } }` when enabled. enabled is required. Selection may include coefficient_map when disabled; it is not loaded. Paths resolve relative to the selection, not the original vehicle file. Disabled selection removes only a previously attached map, preserving authored body/surfaces/fans. Enabled parsing/validation commits atomically to the supplied descriptor; errors leave it untouched. Invoke before create_vehicle and on a retained descriptor clone; no read-only vehicle asset changes required. Provenance/validation sidecars are consumer tooling responsibilities, not additional keys in this strict selection file.

Focused Windows Release verification: [aero] passed 6,492 assertions in23 cases, including existing authored/legacy aero checks, signed map forces/moments, wind/wakes, full-yaw seam, zero speed, strict schema/row/provenance errors, public selection strong exception guarantee, ownership/fan energy, real-ground fallback, actual actuator offset, reset and1-vs-4-worker hash parity with allocation guard. Long Linux sanitizer coverage remains queued; no CFD solver run or real coefficient validation claimed.


## Optional wing lift axis

Schema /1 additionally accepts optional `wing_lift_m` as axis index 5, after
`wing_offset_deg`; rows remain dense and the last present axis varies fastest.
The public CoefficientData axes and Telemetry coefficient_map_inputs now have
six slots. Evaluate accepts six inputs; explicitly typed legacy five-input
arrays remain accepted with lift zero. Lift knots must be finite, nonnegative
and strictly increasing. Clamp bit5 (32) reports lift outside the map. Pitch
and lift axes may be present independently or together; either requires the
same existing wing_surface binding. Missing lift preserves five-axis lookup,
table hashes and native behavior exactly. Six-axis interpolation uses up to64
corners, with no allocation.

The map samples actual State.lift_m after actuator advancement, alongside
actual offset_rad, before passive-force suppression. Current native actuator
kinematics derive height from its rate-limited offset and max_lift_m, exactly
as the existing visual/surface telemetry did; this extension adds no independent
lift controller or extra simulation. The table can represent independent lift
and pitch configurations without collapsing them onto a single baked axis.
Snapshots copy all six telemetry inputs; new-map state hashes include actual
bound lift. Reset clears the lift cache with aero State. Old-map hashes omit
the new cache because height was already derived from the hashed offset.
