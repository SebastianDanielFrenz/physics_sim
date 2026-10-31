# The three-point linkage (`linkage` block of a hitch point)

A towing hitch point of class `three_point_locked` carries a `linkage` block in its vehicle file. Descriptor
`ps::machine::LinkageDesc` (`ps/machine/linkage.h`), loader `read_linkage` in `core/src/io/vehicle_io.cpp`, rows
`solve_linkage` in `core/src/machine/coupling_solver.cpp`. Unknown keys are rejected. A hitch point without the block stays
a plain rigid hitch.

The links are massless and collapse on the tractor's centre plane: the two lower links into one, from the lower pivot to the
middle of the lower pins, the top link from the top pivot to the top pin. That is a planar four-bar with one degree of freedom,
the lower link's angle. The implement's pitch follows from the geometry. The towed point's `lower_pin` and `top_pin` are the
sockets of its own hitch point: the two `socket_3pt_L/R` and `socket_3pt_top` in its `sockets` list.

## Keys

| Key | Type | Required | Meaning |
|---|---|---|---|
| `source` | string | no | where the numbers come from, `[C]`, `[D]`, `[M]` or `[AS]` |
| `lower_pivot` | `[x, y, z]` | yes | the lower links' pivot on the chassis, centre plane |
| `top_pivot` | `[x, y, z]` | yes | the top link's pivot on the chassis, centre plane |
| `min_deg`, `max_deg` | number | yes | the lower link's stops, from the hitch point's own pose (`min < 0 < max`) |
| `lift.max_force_n` | number | yes | the lift at the lower pins, across the link |
| `lift.full_stroke_s` | number | yes | stop to stop at the pump's flow |
| `lift.follow_slack_deg` | number | yes | how far the held angle may run below a link the ground holds up |
| `draft.gain_per_s` | number | yes | lift rate over stroke rate, per relative draft error |
| `draft.filter_s` | number | yes | the sensing pin's lag |
| `draft.band` | number | yes | relative draft error inside which the lift holds still |
| `channels.position` | string | no, `lift` | lever, 0 lowest to 1 highest |
| `channels.mode` | string | no, `lift_mode` | 0 position, 1 draft, 2 float |
| `channels.draft` | string | no, `lift_draft` | draft setpoint, newtons |

The channels are declared on the vehicle, `world.vehicle_channel(tractor, "lift")`.

## Modes

**Position** holds the lever's height: the lift's target moves toward the lever at the stroke rate. The lift only lifts, so its
row is one-sided in `[0, max_force]`; past `max_force` it holds what it has and the implement sinks. A ground that holds the
link up leaves the target running at most `follow_slack_deg` below it.

**Draft** raises and lowers to hold `lower_tension_n`, through the sensing pin's lag, at the setpoint. The lever is the deepest
it goes. The sensed value is what a sensing pin on the lower links reads, so it holds the weight's couple as well as the draft:
the setpoint is a lever, not the tool's force in newtons.

**Float** takes the lift off: the implement follows the ground between the stops.

A rigid tool whose edge is far behind the pins must not run in float or with the lever fully down. The two links' centre comes
in toward the edge as they drop, the force the soil must give grows without end, and the machine noses down and jacks up the
tractor's rear. Run such a tool in position or draft. Float wants a roller near the implement's centre of mass.

## State and findings

`world.coupling_state(hitch).linkage` is a `ps::machine::LinkageState`: mode, angle, position 0..1, target, lift force, lower
link tension (pulled positive), top link force (compressed positive), sensed draft, `saturated`. The lift's
target, the sensed-draft filter, the link angle and the tension are hashed while a linkage hitch is live.

`world.hitch_limits(hitch, policy)` adds `LiftCapacity` (22): the force against `lift.max_force_n` at `hitch_near_share`,
`hitch_limiting_share`, `hitch_exceeded_share`, and Exceeded outright when the lift saturated. A hanging implement also counts
against the point's `max_tongue_load_n`.

## The implement's side

A mounted machine is a body with a wheel-less vehicle on it: `"wheels": []` is legal only together with a `machine` block
([machine.md](machine.md)). Its hitch point is `role: "towed"` with the same class and the three sockets. `World::attach` sets
the lever to where the implement hangs and the mode to position, so nothing moves until someone asks.

```json
{ "name": "socket_3pt", "class": "../hitch_classes/three_point_locked.json", "role": "towing",
  "sockets": { "socket_3pt_top": [-2.12, 0.0, -0.04], "socket_3pt_L": [-2.04, 0.5, -0.68], "socket_3pt_R": [-2.04, -0.5, -0.68] },
  "max_tongue_load_n": 30000.0,
  "linkage": {
    "source": "[AS] ...",
    "lower_pivot": [-1.16, 0.0, -0.76], "top_pivot": [-1.57, 0.0, -0.08],
    "min_deg": -20.0, "max_deg": 25.0,
    "lift": { "max_force_n": 40000.0, "full_stroke_s": 3.0, "follow_slack_deg": 2.0 },
    "draft": { "gain_per_s": 1.0, "filter_s": 0.3, "band": 0.05 }
  } }
```

The numbers are `data/vehicles/tractor.json`'s own; check there before you quote one.

## The PTO shaft

`world.check_pto(hitch)` and `world.connect_pto(hitch)` put the implement's tool with a `pto` block on the tractor's `Pto`
component's `out` node: the rotor's inertia and torque act in the tractor's own shaft network, no second network and no
compliant link. The check refuses a hitch without a PTO on the tractor, without a rotor on the machine, or a shaft already on.
Engaging stays the tractor's `pto` channel, 540 or 1000 stays its `pto_speed` channel. `world.disconnect_pto(hitch)` and
`world.detach(hitch)` take the shaft off. The tool's power reaches the tractor's ledger as `external_absorbed`.
