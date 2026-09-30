extends XROrigin3D
# adapters/godot/demo/scripts/vr_rig.gd — SteamVR (OpenXR) driver's-eye view
# (owner request 2026-09-29). Pure presentation: nothing in the sim reads it.
#
# Only created by main.gd when Godot was started with "--xr-mode on"
# (drive -VR) and the OpenXR interface actually initialised; the project
# keeps xr/openxr/enabled=false so a normal or headless run never touches
# OpenXR or starts SteamVR.
#
# The origin is cockpit_cam.gd's rigid mount: every frame it is placed at
# the vehicle's socket_driver_eye (vehicle_visual.gd's driver_eye_local)
# on the chassis's render-interpolated transform, facing the car's forward
# axis with the car's up as up. The headset pose then moves the child
# XRCamera3D relative to it, so the tracking origin (seated "local" space)
# must coincide with the driver's head: recenter() makes the head's current
# pose the origin, run once the session begins, whenever the runtime
# reports a recenter (SteamVR dashboard), and on F9.
#
# Floating origin: this origin drives PsSimulation.rebase_focus like the
# current flat camera does (main.gd makes no flat camera current in VR).
# The position is re-derived from the chassis every frame, so only
# `position` itself has to be carried through the rebase.

@export var simulation_path: NodePath
@export var chassis_body_name: String = ""
@export var vehicle_visual_path: NodePath

var camera: XRCamera3D
var _simulation: Node
var _vehicle_visual: Node
var _recenter_key_prev: bool = false
const FALLBACK_EYE_ISO := Vector3(-0.35, 0.38, 0.6) # as cockpit_cam.gd

func _ready() -> void:
	# Before every node that places itself from the floating origin - see
	# chase_cam.gd's _ready.
	set_process_priority(-1000)
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)
	if vehicle_visual_path != NodePath():
		_vehicle_visual = get_node_or_null(vehicle_visual_path)
	camera = XRCamera3D.new()
	camera.name = "XRCamera"
	camera.near = 0.03
	add_child(camera)
	current = true
	camera.current = true
	var xr := XRServer.find_interface("OpenXR")
	if xr != null:
		if xr.has_signal("session_begun"):
			xr.session_begun.connect(_on_session_begun)
		if xr.has_signal("pose_recentered"):
			xr.pose_recentered.connect(recenter)
		# The runtime only displays the app once the session is VISIBLE (and
		# gives it input once FOCUSED); log every state so a "nothing in the
		# headset" report can be read from the log.
		for s in ["session_begun", "session_synchronized", "session_visible", "session_focussed", "session_stopping", "session_loss_pending"]:
			if xr.has_signal(s):
				xr.connect(s, func() -> void: print("ps_godot vr_rig.gd: OpenXR ", s, " at ", Time.get_ticks_msec(), " ms"))
		# Keyboard (and Steam-routed gamepad) input only reaches Godot while
		# its desktop window has the OS focus; SteamVR tends to take it.
		var win := get_window()
		win.focus_entered.connect(func() -> void: print("ps_godot vr_rig.gd: desktop window focused - keyboard input active"))
		win.focus_exited.connect(func() -> void: print("ps_godot vr_rig.gd: desktop window LOST focus - no keyboard input until it is clicked"))
		win.grab_focus()
		print("ps_godot vr_rig.gd: window focused=", win.has_focus(), " joypads=", Input.get_connected_joypads().map(func(i: int) -> String: return Input.get_joy_name(i)))
		print("ps_godot vr_rig.gd: blend modes supported=", xr.get_supported_environment_blend_modes(),
			" current=", xr.environment_blend_mode, " render target=", xr.get_render_target_size(),
			" refresh=", xr.get_display_refresh_rate())

func _on_session_begun() -> void:
	# Tracking may take a moment to become valid after the OpenXR session starts.
	await get_tree().create_timer(1.0).timeout
	if is_inside_tree():
		recenter()

func recenter() -> void:
	# Seated: the head's current pose becomes the origin (yaw only, keep
	# the tilt; keep_height false so the eye sits exactly at the socket).
	XRServer.center_on_hmd(XRServer.RESET_BUT_KEEP_TILT, false)
	print("ps_godot vr_rig.gd: recentered on the headset")

func _eye_local() -> Vector3:
	if _vehicle_visual != null and _vehicle_visual.has_method("has_driver_eye") and _vehicle_visual.has_driver_eye():
		return _vehicle_visual.driver_eye_local()
	return FALLBACK_EYE_ISO

func _process(_delta: float) -> void:
	var key := Input.is_physical_key_pressed(KEY_F9)
	if key and not _recenter_key_prev:
		recenter()
	_recenter_key_prev = key

	if _simulation == null or chassis_body_name == "":
		return
	var t0 := Time.get_ticks_usec()
	var chassis_xform: Transform3D = _simulation.get_body_transform(chassis_body_name)
	# Basis columns are the sim's ISO axes in Godot space: x forward, z up.
	var fwd: Vector3 = chassis_xform.basis.x.normalized()
	var up: Vector3 = chassis_xform.basis.z.normalized()
	position = chassis_xform * _eye_local()
	basis = Basis.looking_at(fwd, up)
	position = _simulation.rebase_focus(position)
	_simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)
