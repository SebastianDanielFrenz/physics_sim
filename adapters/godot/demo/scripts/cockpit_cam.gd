extends Camera3D
# adapters/godot/demo/scripts/cockpit_cam.gd — driver's-eye camera (owner
# request 2026-09-29). Pure presentation: nothing in the sim reads it.
#
# Sits at the art's own "socket_driver_eye" node (data/models/<m>/<m>.glb,
# also listed in <m>.rig.json's "sockets"), read through vehicle_visual.gd's
# driver_eye_local() - the eye in the chassis body's ISO-local frame, i.e.
# after that script's model-to-physics-wheel alignment - so it lands in the
# modelled seat for every vehicle without a per-vehicle number here.
#
# Rigidly mounted: position and orientation follow the chassis's own
# render-interpolated transform (PsSimulation.get_body_transform), so the
# car stays still in view and the world moves, pitch and roll included.
# No smoothing - a lag here would let the eye drift through the dashboard.
#
# Right stick looks around like chase_cam.gd's orbit: the stick's POSITION
# sets the view yaw about the chassis's up axis (up = ahead, right = look
# right, down = look back), releasing it returns to straight ahead.
#
# Origin rebasing is chase_cam.gd's loop minus its persistent state: the
# position is re-derived from the chassis every frame, so only `position`
# itself has to be carried through rebase_focus.

@export var simulation_path: NodePath
@export var chassis_body_name: String = ""
@export var vehicle_visual_path: NodePath
@export var look_deadzone: float = 0.2
@export var look_lag: float = 14.0 # higher = snappier; <= 0 = no smoothing
@export var max_look_yaw_rad: float = 2.6 # ~150 deg; full stick-down = over the shoulder
@export var look_pitch_deg: float = -3.0 # a slight downward gaze, like a seated driver's

var _simulation: Node
var _vehicle_visual: Node
var _look_yaw: float = 0.0
var _warned_no_eye: bool = false
# Used only when the model has no socket_driver_eye (or failed to load):
# roughly a sedan driver's head, ISO chassis-local. Warned once.
const FALLBACK_EYE_ISO := Vector3(-0.35, 0.38, 0.6)

func _ready() -> void:
	# Same reason as chase_cam.gd's _ready: rebase_focus below may move the
	# floating origin, so run before every node that places itself from it.
	set_process_priority(-1000)
	near = 0.03 # the dashboard is ~0.5 m away; the default 0.05 is fine, this is margin for a closer seat
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)
	if vehicle_visual_path != NodePath():
		_vehicle_visual = get_node_or_null(vehicle_visual_path)

func _target_look_yaw() -> float:
	var pads := Input.get_connected_joypads()
	if pads.is_empty():
		return 0.0
	# Godot's stick Y is +down, so "up" is -Y.
	var stick := Vector2(Input.get_joy_axis(pads[0], JOY_AXIS_RIGHT_X),
			-Input.get_joy_axis(pads[0], JOY_AXIS_RIGHT_Y))
	var mag: float = min(stick.length(), 1.0)
	if mag <= look_deadzone:
		return 0.0
	var weight: float = smoothstep(look_deadzone, 1.0, mag)
	# atan2 spans +-PI; scale it so straight down reaches max_look_yaw_rad.
	return atan2(stick.x, stick.y) / PI * max_look_yaw_rad * weight

func _eye_local() -> Vector3:
	if _vehicle_visual != null and _vehicle_visual.has_method("driver_eye_local") and _vehicle_visual.has_driver_eye():
		return _vehicle_visual.driver_eye_local()
	if not _warned_no_eye:
		_warned_no_eye = true
		push_warning("cockpit_cam.gd: no socket_driver_eye in the vehicle model - using a fallback eye point")
	return FALLBACK_EYE_ISO

func _process(delta: float) -> void:
	if _simulation == null or chassis_body_name == "":
		return
	if not current:
		return # nothing to show; the current camera owns the origin
	var t0 := Time.get_ticks_usec()

	var chassis_xform: Transform3D = _simulation.get_body_transform(chassis_body_name)
	# get_body_transform's basis columns are the sim's ISO axes in Godot
	# space (x forward, y left, z up - see chase_cam.gd's comment).
	var fwd: Vector3 = chassis_xform.basis.x.normalized()
	var left: Vector3 = chassis_xform.basis.y.normalized()
	var up: Vector3 = chassis_xform.basis.z.normalized()

	var target_yaw := _target_look_yaw()
	if look_lag <= 0.0:
		_look_yaw = target_yaw
	else:
		_look_yaw = lerp_angle(_look_yaw, target_yaw, 1.0 - exp(-look_lag * delta))

	# +yaw looks right; rotating about +up by a positive angle turns left.
	var dir := fwd.rotated(up, -_look_yaw)
	var side := left.rotated(up, -_look_yaw)
	dir = dir.rotated(side, -deg_to_rad(look_pitch_deg)) # rotating about +left tips the nose down
	position = chassis_xform * _eye_local()
	basis = Basis.looking_at(dir, up)

	var rebased_pos: Vector3 = _simulation.rebase_focus(position)
	position = rebased_pos

	_simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)
