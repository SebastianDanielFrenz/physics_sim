extends Camera3D
# adapters/godot/demo/scripts/free_cam.gd — free camera for the P2 demo.
#
# Arrow keys (+ PageUp/PageDown for altitude) move the camera; deliberately
# NOT WASD, which is reserved for steer/throttle/brake (project.godot's
# [input] map) so the two never collide even before a vehicle exists to
# drive. Every frame calls PsSimulation.rebase_focus() with its own current
# position and adopts the returned position - this IS the origin-rebasing
# loop PLAN.md P2 asks for (large-world paragraph): once this camera drifts
# past OriginRebase's threshold (origin_rebase.h, default 500 m) from the
# current origin, the sim adapter snaps the origin to the camera and this
# script's position jumps by the exact opposite amount in the same call,
# so the camera's on-screen position never visibly moves.

@export var simulation_path: NodePath
@export var move_speed_m_s: float = 20.0
@export var boost_multiplier: float = 5.0

var _simulation: Node
# Godot-space position of the sim's absolute origin as of this camera's
# previous frame - see _follow_origin_shift.
var _last_origin_godot: Vector3 = Vector3.ZERO
var _origin_seen: bool = false

func _ready() -> void:
	# Same reason as chase_cam.gd: rebase before anything else reads a pose.
	set_process_priority(-1000)
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)

func _process(delta: float) -> void:
	var t0 := Time.get_ticks_usec()
	if _simulation == null:
		return
	position += _follow_origin_shift()
	# Only the CURRENT camera drives the floating origin: two cameras both
	# calling rebase_focus would fight over it whenever they are more than
	# the rebase threshold apart. The inactive one just follows the shift.
	if current:
		_handle_movement(delta)
		position = _simulation.rebase_focus(position)
	_last_origin_godot = _simulation.get_world_origin_godot_position()
	_simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)

# How far the floating origin moved (in Godot space) since this camera's
# last frame, e.g. because the other camera rebased. Every persistent
# Godot-space position this script holds must move by the same amount to
# stay put in the sim's absolute frame.
func _follow_origin_shift() -> Vector3:
	var now: Vector3 = _simulation.get_world_origin_godot_position()
	var shift := Vector3.ZERO
	if _origin_seen:
		shift = now - _last_origin_godot
	_origin_seen = true
	_last_origin_godot = now
	return shift

func _handle_movement(delta: float) -> void:
	var dir := Vector3.ZERO
	if Input.is_key_pressed(KEY_UP):
		dir -= transform.basis.z
	if Input.is_key_pressed(KEY_DOWN):
		dir += transform.basis.z
	if Input.is_key_pressed(KEY_LEFT):
		dir -= transform.basis.x
	if Input.is_key_pressed(KEY_RIGHT):
		dir += transform.basis.x
	if Input.is_key_pressed(KEY_PAGEUP):
		dir += Vector3.UP
	if Input.is_key_pressed(KEY_PAGEDOWN):
		dir -= Vector3.UP

	if dir.length_squared() > 0.0:
		dir = dir.normalized()
		var speed := move_speed_m_s
		if Input.is_key_pressed(KEY_SHIFT):
			speed *= boost_multiplier
		position += dir * speed * delta
