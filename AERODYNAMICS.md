# Reusable real-time aerodynamics

The public model is `ps/aero/aerodynamics.h`, linked into `ps_core`. It is a
deterministic quasi-steady coefficient approximation. It supports arbitrary
rigid-body orientation, signed lift, spoilers/downforce, body side force,
angular airflow, finite drafting wakes, local ground effect, active surfaces
and optional energy-budgeted suction fans. Values in the shipped hypercar
are fictional authored coefficients, not measured Koenigsegg/One:1 data.

## Frame, units and forces

SI throughout: metres, seconds, kg/m3, radians, Pa, W, J, N and N m. Core axes
are right-handed ISO: +x forward, +y left, +z up. All `*_local` points are
relative to the rigid body's centre of mass; all environment velocities and
returned wrenches are world-space. `Motion::linear` is the CoM velocity.

Each surface samples `linear + angular cross world_arm - local_wake_wind`.
Its chord and normal are orthonormal **local** unit vectors. Its span is
`normal cross chord`. Positive mechanical incidence rotates about span:
with the default axes this is +y, pitching the chord nose-down. Positive CL
produces lift along the surface normal projected perpendicular to the flow;
negative CL produces downforce. No world-down correction is applied.

Lift/drag are `q * area * coefficient`, with `q = 0.5*rho*airspeed^2`.
Alpha is `atan2(-flow dot normal, flow dot chord)` after mechanical rotation;
beta is `asin(flow_direction dot span)`. Attached lift is `cl0 + slope*alpha`.
From the stall angle through 1.5 times that angle a smoothstep blends to the
signed flat-plate `sin(2*alpha)` approximation. Camber bonus disappears in
reverse flow. Spanwise flow suppresses lift continuously. Drag includes
`cd0 + induced_drag*CL^2 + stalled_drag*blend*sin(alpha)^2` and always opposes
flow. Turbulence attenuates CL by up to 50%. This is an authored stall model,
not a measured aerofoil polar; there is no Mach/Reynolds, compressibility,
unsteady stall or CFD model. The coefficient approach and distinction between
lift and induced drag follow [NASA's lift equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/)
and [inclination effects on drag](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/inclination-effects-on-drag/).
The blending, wake and ground-effect fits here are our approximations.

Body drag uses independent forward/side/vertical CdA values and component-wise
quadratic resistance. Optional body side force opposes sideslip, perpendicular
to airflow. Intrinsic local roll/pitch/yaw moments use
`q*A*L*(moment_coeff - angular_damping*omega_local*L/(2*max(airspeed,1)))`.
Offset forces add `world_arm cross force`. Body +y torque pitches nose-down.
Legacy `drag_area_m2` means isotropic CdA at the CoM; existing zero-wind forces
retain their exact world-space arithmetic. Positive legacy CdA combined with
explicit body/surface aero is rejected, preventing double counting.

## Ground effect and inverted operation

`ground.probe_local` points toward the nearby road, independently of gravity.
Ground multiplier is `1 + gain*max(0,1-clearance/range)^2`, bounded to [1,4]
by validation. Negative ground clearance, short/missing samples and misses
give multiplier 1. Probe hit normals must face the probe within 60 degrees.
Jolt's new `ray_cast_excluding` excludes the entire chassis, including probes
starting inside it. Its normals and offsets are world-space. Other backends
should override this method and return real hit normals; the conservative
default cannot see through self and otherwise returns their normal ray result.
Existing terrain collision tiles are queried just like other backend geometry;
there is no flat-road assumption when a tile or nearby road is absent.

When inverted, local downforce and local road probes rotate toward the overhead
road. Actual body/wheel collision and suspension forces still decide whether
adhesion succeeds: sufficient aero normal load must exceed gravity and other
required contact loads. This is not an upside-down attachment or gravity override.

## Wakes and drafting

Every registered vehicle contributes an immutable tick-start source snapshot,
in vehicle creation order. Direction is opposite its **wind-relative** linear
velocity, including reversing trucks; stationary wind-relative sources below
0.5 m/s produce no wake. A finite expanding cone has radius
`radius_m + expansion*distance`, a smooth radial edge and downstream decay.
An onset ramp avoids a source-plane discontinuity. Source airflow is entrained
into this cone, reducing trailing relative speed/dynamic pressure when aligned.
Off-axis, upstream, outside-cone and self sources contribute nothing.
Overlapping entrainment is normalized to a convex combination bounded to 75%
of source velocity. Turbulence combines as `1-product(1-source_turbulence)`.
No random forcing or order depending on job completion is used. The model has
no advected wake history or resolved vortices. All sources remain fixed during
the receiving vehicle's substeps; this deliberate tick-granularity approximation
makes worker-count parity possible without inter-vehicle job dependencies.

Car example: `{length_m:24,radius_m:1,expansion:.12,deficit:.28,turbulence:.15}`.
Truck example: `{length_m:60,radius_m:2,expansion:.15,deficit:.45,turbulence:.2}`.
These are **uncalibrated reference parameters**. Non-vehicle sources can be
published through `World::set_aero_wakes(vector<WakeSource>)`; update their
world position/velocity each tick on the stepping thread. Avoid duplicating an
already registered vehicle source. `owner` excludes self when it matches a body.

## World API and independent aircraft use

Call setters only on the stepping thread **between ticks**:

```cpp
world.set_aero_environment(ps::aero::Environment{density_kg_m3, wind_world});
world.set_aero_surface_command(vehicle, surface_index, command_minus1_to_plus1);
world.set_aero_fan_command(vehicle, fan_index, command_zero_to_one);
world.credit_aero_fan_energy(vehicle, prepaid_joules);
```

Environment is independent of `set_ambient` (engine temperature/pressure).
`aero_environment()` reads the active input. Density is finite [0,100] kg/m3;
wind must be finite. Environment and commands are external inputs, not state
hash material. Surface/fan actuator state and remaining fan energy are hashed.

A native `vehicle::VehicleDesc` with **zero wheels** can register an aircraft
on a dynamic body using `World::create_vehicle`; it receives the same substep
and wake pipeline. A focused test verifies positive lift, shadow agreement and
snapshot publication with no wheels. The existing vehicle JSON loader still
requires wheels; no new aircraft file format or flight-control UI is provided.
For other systems, `aero::step` is independently callable without World, wheels
or a backend. Validate descriptors/environment once, provide finite normalized
poses/motions, dt >= 0, bounded commands and nonnegative finite prepaid energy.
Supply one `GroundSample` per surface followed by one per fan, or omit the span.
Apply the returned wrench about the body's CoM. World does this automatically
in the SAME shadow/substep integration as tyre and suspension forces.

## Active wing and telemetry

Command 0 means rest. Positive command scales `max_offset_rad`; negative command
scales `min_offset_rad`. Actual offset changes by at most `rate_rad_s*dt`.
Both force calculation and render telemetry use this actual offset. Optional
`max_lift_m` translates along the unrotated surface normal, proportional to
positive offset/max offset; negative offset does not raise it. Drag/downforce
follow the continuously changing flow angle, including airbrake stall.

Hypercar surface indices are **0 splitter, 1 underfloor, 2 rear_wing**.
Wing rest = 8 degrees; offsets [-8,+55] degrees; rate 35 degrees/s; lift [0,.28]m.
Its CoM-local pivot is [-1.714,0,.475]m, from art [-1.93,0,.975] plus mounting
shift [+.216,0,-.50]. Render rotation is ISO +y (glTF/Godot rig +x), using
`offset_rad` in addition to the asset's authored rest rotation. `angle_rad`
includes the rest angle. `lift_m` is the actual lift, not a command target.
By default the game chooses cruise/airbrake policy through the surface command API;
the aero library itself (`ps/aero/aerodynamics.h`) never reacts to brake/throttle inputs.

### Automation (assists I5)

A vehicle may carry an **assist unit** with an `aero_control` block (grade Racing,
[electronics.md](docs/data_formats/electronics.md#assist-aero), vault PHYS-074). The vehicle layer, not the aero model,
then runs racing_game's rear-wing law per tick: target offset = `cruise_offset_deg` (4) ramped over 10..30 m/s of
forward **airspeed** (chassis velocity minus the vehicle's own wind), blended toward `airbrake_offset_deg` (45) by the
`brake` channel between 0.45 and 1.0 once airspeed is at least 20 m/s; a launch hook (for the launch control of assists
I4) replaces both with `launch_offset_deg`. The target becomes a surface command (`radians / max_offset_rad`, or
`/ -min_offset_rad` for a negative target) which the surface's `rate_rad_s` slews as usual. Channel
`assist.aero_control` (registered only when the car has the named surface) switches it: on, the controller owns that
surface's command and `World::set_aero_surface_command` for it is ignored; off, the World command stands as before. The
controller is stateless and adds no hashed state. Snapshot: `aero_control_active`, `aero_control_target_deg`, bit 6 of
the assist bit masks.

`vehicle_aero_telemetry(vehicle)` is a stepping-thread live accessor.
Render/readout threads use the frame-latched `WorldSnapshot::vehicles[i].aero`.
Total wrench and front/rear downforce are tick averages; per-surface/fan flow,
force, coefficients, clearance, seal and actuator telemetry are the last substep.
Aggregate fan energy is this tick's sum and power is its tick average.
Body relative airflow/airspeed/q/turbulence include actual sampled wakes, not
just mean wind. Forces/moments are world-space; body force/moment are local.

With both front and rear wheel groups, `axle_balance_valid=true` and the World
front equivalent aero load is `(local_pitch_moment - rear_axle_x*D)/wheelbase`;
rear is D-front. This includes drag-height and intrinsic pitch moments. Values
may be negative or fractions outside [0,1] (unloading); they are not clamped.
They are **aero contributions**, not total tyre load. With no axle groups,
standalone step/World group forces by pressure-point x>=0/<0, and set the flag
false. Do not present that grouping as axle load balance.

## Optional suction fans and energy

Stock hypercar has no fans. A fan starts with command/level zero and an empty
energy reservoir. `pressure_pa*area_m2*seal*power_fraction` acts in its local
direction, even at zero road speed. Seal is
`max(0,1-clearance/seal_height)^2`; no ground means zero suction. Command is
rate limited. Requested motor power is `max_power_w*actual_level`, bounded by
remaining prepaid energy/dt; actual pressure fraction is power/max_power_w.
An open skirt still consumes motor energy while spinning. Multiple fans share
the prepaid reservoir in configured order, deterministically.

The caller MUST debit energy from its battery or engine/electrical accounting
before crediting it here; credits are the explicit coupling seam. Telemetry
reports joules consumed/remaining and actual power. There is no automatic engine
shaft accessory or vehicle battery component added in this change. Pressure,
leakage and motor power are authored normalized approximations, not a resolved
fan gas-flow curve. No fitted fan and no supplied energy means no fan force.
Vehicle reset clears actuators/commands but preserves remaining prepaid energy.

## Validation and verification

JSON `aero` uses the public descriptor field names. Unknown keys, malformed
vectors/arrays, duplicate names, nonfinite or negative dimensions, non-unit or
nonorthogonal axes, excessive ground gain, out-of-range wake deficits and
invalid actuator limits are rejected. Maximum 32 surfaces and 8 fans per body.
Native descriptors receive the same validation at vehicle construction.

Focused Windows results and build logs are under `out/aero_*.log`; the test
case tag is `[aero]`. Full Linux ASan/UBSan and TSan are queued for the owner's
05:00 run rather than blocking daytime work. No Godot or racing_game DLL is
replaced by this change. The game integrates weather, control policy and art.

Daytime verification completed 2026-10-03: 1,319 assertions / 54 unit-regression
cases and 53 assertions / 4 shadow-chassis validation cases passed. Ground/fan/
multiwake and airborne vehicle hashes match exactly at 1 and 4 workers, and the
new ground/fan/wake path has zero own heap allocations inside jobs. The existing
allocation regression reports known ungated Jolt-internal allocations. Full
Linux sanitizers remain pending in out/terrain_prefetch_overnight_pending.md;
worker details are in out/aero_worker_report.md.
