extends Camera3D
# adapters/godot/demo/scripts/chase_cam.gd — PLAN.md M1 step 7: "a chase
# camera that works at 50 km from the origin (origin rebasing already
# exists) and does not jitter".
#
# Follows the vehicle chassis's OWN Transform3D (already race-free/origin-
# relative via PsSimulation.get_body_transform - ps_simulation.h's file
# comment), smoothed with an exponential lag so small physics-tick jitter
# doesn't show, then rebases itself through PsSimulation.rebase_focus every
# frame exactly like free_cam.gd - this IS the origin-rebasing loop PLAN.md
# P2 asks for: once this camera (not the chassis - the camera's own Godot
# position) drifts past OriginRebase's threshold, the origin snaps to it and
# rebase_focus's return value is `position` re-expressed in the new frame.
# Because the camera always stays within follow_distance_m of the chassis,
# and that is far under the 500 m rebase threshold, the origin effectively
# follows the car around the whole 100x100 km world with no jitter at any
# distance from the world's own (0,0,0) - PROVIDED every piece of this
# script's own persistent Godot-space state is shifted by the same delta
# rebase_focus applied to `position`, not just `position` itself. Below,
# `_smoothed_pos` gets that same shift right after the rebase_focus call.
# It didn't always: an earlier version left `_smoothed_pos` in the OLD
# frame, so the very next frame's lerp target (`desired_pos`, freshly
# re-derived from the chassis's own transform and therefore already correct
# in the NEW frame) was up to ~threshold metres away from where
# `_smoothed_pos` still thought the camera was - visibly "the camera swings
# around as if the car had teleported", then drags back over about a second
# as the lerp catches up. Owner-reported live-demo bug, happening every
# ~500 m of driving; see this fix's commit for the smoke-test check that
# catches a regression of it.

@export var simulation_path: NodePath
@export var chassis_body_name: String = ""
@export var follow_distance_m: float = 7.0
@export var follow_height_m: float = 2.5
@export var look_height_m: float = 1.0
@export var position_lag: float = 6.0 # higher = snappier
@export var rotation_lag: float = 4.0

var _simulation: Node
var _smoothed_pos: Vector3 = Vector3.ZERO
var _smoothed_basis: Basis = Basis.IDENTITY
var _initialized: bool = false

func _ready() -> void:
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)

func _process(delta: float) -> void:
	var t0 := Time.get_ticks_usec()
	if _simulation == null or chassis_body_name == "":
		return

	var chassis_xform: Transform3D = _simulation.get_body_transform(chassis_body_name)
	var chassis_pos: Vector3 = chassis_xform.origin
	var chassis_basis: Basis = chassis_xform.basis

	# Desired camera position: behind and above the chassis. NOTE:
	# chassis_basis's columns are NOT Godot's usual node-local convention
	# (where .z is "backward") - PsSimulation's chassis transform comes
	# through frame_convert.h's iso_to_godot_basis, whose column i is
	# "the sim's own local axis i (ISO 8855: x forward, y left, z up),
	# expressed in Godot world space" (see that file's header comment).
	# So chassis_basis.x is the car's FORWARD direction in Godot space,
	# and "behind" is its negation - NOT chassis_basis.z (that is the
	# car's UP direction, which is why using it here made the look_at
	# target/eye nearly colinear with world UP whenever the car was
	# level - a bug found by actually running this and seeing Godot's own
	# "Target and up vectors are colinear" warning fire every frame).
	var behind := -chassis_basis.x.normalized() * follow_distance_m
	var desired_pos := chassis_pos + behind + Vector3.UP * follow_height_m
	var desired_look_at := chassis_pos + Vector3.UP * look_height_m

	if not _initialized:
		_smoothed_pos = desired_pos
		_initialized = true

	var pos_alpha: float = 1.0 - exp(-position_lag * delta)
	_smoothed_pos = _smoothed_pos.lerp(desired_pos, pos_alpha)

	position = _smoothed_pos
	look_at(desired_look_at, Vector3.UP)

	# Origin rebasing (see file comment) - keeps this camera's own position
	# always well under float32's precision problems, at any distance from
	# the sim's own absolute origin. _smoothed_pos is OUR OWN persistent
	# state, not re-derived from the sim each frame the way chassis_pos/
	# desired_pos are - it must be carried across the same delta the origin
	# snap applies to `position`, or it silently keeps living in the old
	# frame (see file comment for the resulting bug).
	var rebased_pos: Vector3 = _simulation.rebase_focus(position)
	var rebase_delta: Vector3 = rebased_pos - position
	position = rebased_pos
	_smoothed_pos += rebase_delta

	var elapsed_us := Time.get_ticks_usec() - t0
	_simulation.add_adapter_time_us(elapsed_us)
