extends Node3D
# adapters/godot/demo/scripts/vehicle_visual.gd — PLAN.md M1 step 3: shows
# the vehicle with existing art (data/models/car_sedan/car_sedan.glb, an
# asset outside this project's res:// tree - owned by a separate session,
# per this adapter's lane note, so loaded at RUNTIME via GLTFDocument
# rather than importing it through the editor's res:// pipeline).
#
# Wheel visuals follow suspension travel and spin from the REAL simulated
# per-tick state (World::wheel_telemetry's compression/omega, read via
# PsSimulation.get_wheel_compression/get_wheel_omega). Steering does NOT
# have a real per-wheel state to read - core exposes no per-wheel steer-
# angle accessor (WheelTelemetry has slip_ratio/slip_angle/fx/fy/fz/omega/
# compression/surface, but no steer_angle - see this adapter's report for
# the exact API this needs, e.g. WheelTelemetry::steer_angle or a
# World::wheel_steer_angle(VehicleId, wheel_index)). Steering below is
# therefore an APPROXIMATION: it recomputes the same Ackermann formula the
# core's own vehicle_instance.cpp uses (out/handover_p3.md), from the
# vehicle's own SteeringDesc params (get_vehicle_max_wheel_angle/
# get_vehicle_ackermann_fraction/get_vehicle_rear_steer_fraction) and the
# live "steer" control channel - matches the real value whenever
# steer_rate_limit is 0 (car_sedan.json's is), and is otherwise a lagged
# approximation.
#
# Node names below match data/models/car_sedan/car_sedan.rig.json exactly
# ("susp_<corner>"/"steer_<corner>"/"wheel_<corner>", corner in
# {FL,FR,RL,RR} - same corner codes as ps::vehicle::WheelDesc::name /
# World::wheel_name). Rear wheels have no steer_<corner> node (not
# steered) - handled below by falling back to susp_<corner> as the wheel
# node's direct parent lookup, matching the rig's own hierarchy.
#
# FOUND, NOT FIXED HERE (data/models is a separate session's lane): car_
# sedan.rig.json documents "the .glb itself uses glTF axes (+Y up, +Z
# forward, +X left)", i.e. gltf(x,y,z) = iso(y,z,x) - but the actual
# imported geometry renders lying on its SIDE (its 4.5 m length along
# Godot's vertical +Y) unless rotated -90 deg about local X after loading,
# which is exactly the standard "Z-up content loaded as if it were Y-up"
# symptom. That means the FILE's real local axes match ISO 8855 directly
# (x forward, y left, z up - a straight identity, not the documented y,z,x
# permutation): every local-axis choice below (suspension bump on local Z,
# not Y; wheel spin about local Y, not X; steer about local Z, not Y) was
# picked to match that ACTUAL behaviour, confirmed by screenshotting the
# corrected orientation - not by re-deriving it from the rig.json text,
# which appears to describe what the pipeline intended rather than what it
# produced. Report this to whoever owns tools/assetgen/the rig.json format.

@export var simulation_path: NodePath
@export var vehicle_name: String = ""
@export var model_absolute_path: String = "" # set by main.gd before add_child

var _simulation: Node
var _wheel_nodes: Array = []
var _model_root: Node3D
var _load_ok: bool = false

func _ready() -> void:
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)
	_load_model()
	_bind_wheel_nodes()
	set_process(true)

func model_loaded() -> bool:
	return _load_ok

func _load_model() -> void:
	if model_absolute_path == "" or not FileAccess.file_exists(model_absolute_path):
		push_warning("vehicle_visual.gd: model not found at '%s'" % model_absolute_path)
		return
	var gltf_doc := GLTFDocument.new()
	var gltf_state := GLTFState.new()
	var err: int = gltf_doc.append_from_file(model_absolute_path, gltf_state)
	if err != OK:
		push_warning("vehicle_visual.gd: GLTFDocument.append_from_file failed (err=%d) for %s" % [err, model_absolute_path])
		return
	_model_root = gltf_doc.generate_scene(gltf_state)
	if _model_root == null:
		push_warning("vehicle_visual.gd: GLTFDocument.generate_scene returned null for %s" % model_absolute_path)
		return
	add_child(_model_root)
	# WORKAROUND, not a fix to the asset itself (see file header comment):
	# the loaded geometry is authored Z-up, not the Y-up its own rig.json
	# claims - rotate the whole model -90 deg about local X so it renders
	# lying flat like a normal car instead of standing on its length axis.
	_model_root.rotation.x = -PI / 2.0
	_load_ok = true
	print("ps_godot vehicle_visual.gd: loaded model ", model_absolute_path)

func _bind_wheel_nodes() -> void:
	if _simulation == null or _model_root == null or vehicle_name == "":
		return
	var wheel_count: int = _simulation.get_vehicle_wheel_count(vehicle_name)
	var missing := 0
	for i in range(wheel_count):
		var wname: String = _simulation.get_wheel_name(vehicle_name, i)
		var susp: Node3D = _model_root.find_child("susp_" + wname, true, false)
		var steer: Node3D = _model_root.find_child("steer_" + wname, true, false)
		var wheel: Node3D = _model_root.find_child("wheel_" + wname, true, false)
		var entry := {
			"susp": susp,
			"steer": steer,
			"wheel": wheel,
			"rest_susp_pos": (susp.position if susp != null else Vector3.ZERO),
			"spin_angle": 0.0,
			"is_front": _simulation.get_wheel_is_front(vehicle_name, i),
			"steered": _simulation.get_wheel_steered(vehicle_name, i),
		}
		_wheel_nodes.append(entry)
		if susp == null or wheel == null:
			missing += 1
	if missing > 0:
		push_warning("vehicle_visual.gd: %d/%d wheel(s) missing a susp_*/wheel_* node in the model - those stay at their static rest position" % [missing, wheel_count])

func _process(delta: float) -> void:
	if _simulation == null or vehicle_name == "" or _wheel_nodes.is_empty():
		return
	var t0 := Time.get_ticks_usec()

	var max_wheel_angle: float = _simulation.get_vehicle_max_wheel_angle(vehicle_name)
	var ackermann_fraction: float = _simulation.get_vehicle_ackermann_fraction(vehicle_name)
	var rear_steer_fraction: float = _simulation.get_vehicle_rear_steer_fraction(vehicle_name)
	var steer_input: float = _simulation.get_control("steer") # -1..1, same channel the vehicle itself reads

	for i in range(_wheel_nodes.size()):
		var w = _wheel_nodes[i]
		var compression: float = _simulation.get_wheel_compression(vehicle_name, i)
		var omega: float = _simulation.get_wheel_omega(vehicle_name, i)
		w["spin_angle"] = fmod(w["spin_angle"] + omega * delta, TAU)

		if w["susp"] != null:
			# Suspension axis is ISO [0,0,1] (up) for every corner of this
			# rig - the model's own local Z, given the file's ACTUAL axes
			# (see file header comment: local axes match ISO directly, not
			# the documented y,z,x permutation). Real simulated state
			# (World::wheel_telemetry's compression).
			w["susp"].position = w["rest_susp_pos"] + Vector3(0, 0, 1) * compression

		if w["steer"] != null:
			if w["steered"]:
				# APPROXIMATION - see file header comment. Steer axis ISO
				# [0,0,1] -> model-local Z.
				w["steer"].rotation = Vector3(0, 0, steer_input * max_wheel_angle * ackermann_fraction)
			elif not w["is_front"] and rear_steer_fraction != 0.0:
				w["steer"].rotation = Vector3(0, 0, -steer_input * max_wheel_angle * ackermann_fraction * rear_steer_fraction)

		if w["wheel"] != null:
			# Real simulated state (World::wheel_telemetry's omega,
			# integrated here into a visual spin angle). Wheel joint axis
			# ISO [0,1,0] ("+ = rolling forward") -> model-local Y.
			w["wheel"].rotation = Vector3(0, w["spin_angle"], 0)

	_simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)
