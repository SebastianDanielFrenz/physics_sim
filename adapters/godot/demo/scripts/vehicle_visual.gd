extends Node3D
# adapters/godot/demo/scripts/vehicle_visual.gd — PLAN.md M1 step 3: shows
# the vehicle with existing art (data/models/car_sedan/car_sedan.glb, an
# asset outside this project's res:// tree - owned by a separate session,
# per this adapter's lane note, so loaded at RUNTIME via GLTFDocument
# rather than importing it through the editor's res:// pipeline).
#
# Wheel visuals follow suspension travel, spin AND steering from the REAL
# simulated per-tick state: World::wheel_telemetry's compression/omega (via
# PsSimulation.get_wheel_compression/get_wheel_omega) and World::wheel_state's
# steer_angle (ps/vehicle/wheel_state.h, via PsSimulation.get_wheel_steer_angle
# - added alongside this fix; previously core exposed no per-wheel steer-angle
# accessor, so this script recomputed an APPROXIMATION of the Ackermann
# formula client-side from the vehicle's SteeringDesc params, which (a) was
# wrong for any ackermann_fraction other than 1 - a parallel-steer car
# (fraction 0) showed its front wheels not turning at all, and an
# anti-Ackermann car (fraction < 0) showed them turned the wrong way, both
# just multiplying the same visual angle by ackermann_fraction regardless of
# sign or magnitude - and (b) lagged the real value whenever
# steer_rate_limit was nonzero (car_sedan.json's is 0, so this specific
# lag never showed up in the shipped demo). get_wheel_steer_angle returns
# ackermann_wheel_angle()'s actual per-wheel result (vehicle_instance.cpp) -
# correct for every ackermann_fraction and steer_rate_limit value, front or
# rear wheel, with no client-side recomputation.
#
# Node names below match data/models/car_sedan/car_sedan.rig.json exactly
# ("susp_<corner>"/"steer_<corner>"/"wheel_<corner>", corner in
# {FL,FR,RL,RR} - same corner codes as ps::vehicle::WheelDesc::name /
# World::wheel_name). Rear wheels have no steer_<corner> node (not
# steered) - handled below by falling back to susp_<corner> as the wheel
# node's direct parent lookup, matching the rig's own hierarchy.
#
# MODEL-SPACE FIX (M1 visual pass, corrects the previous session's
# workaround): the earlier "-90 deg about local X" rotation here was based
# on a misdiagnosis. Verified directly against car_sedan.glb's own bytes
# (parsed the glTF JSON+BIN chunks by hand, no Blender/Godot needed): both
# every node's `translation` (e.g. susp_FL = glTF(0.78, 0.317, 1.35) vs.
# rig.json's ISO pivot [1.35, 0.78, 0.317]) AND the root mesh's own vertex
# bounding box (glTF-local min/max size [1.82, 1.339, 4.584] vs. the rig's
# ISO body bounds size [4.584 fwd, 1.82 left, 1.338 up]) satisfy the rig's
# documented gltf(x,y,z) = iso(y,z,x) permutation EXACTLY (matches to the
# last digit). The asset is internally consistent and matches its own
# rig.json; there is no Z-up/Y-up authoring bug here (nothing to report
# upstream). "Lying on its side" was this adapter's own bug: a raw local
# gltf offset must be re-permuted into ISO order (iso.x=gltf.z, iso.y=
# gltf.x, iso.z=gltf.y) BEFORE the chassis's PsBodyNode3D applies its own
# ISO->Godot basis change (frame_convert_core.h) to the whole subtree - an
# extra ad hoc rotation on top of that (rather than exactly this
# permutation) fights the parent's own conversion instead of feeding it
# correctly-labelled axes. The fix below sets _model_root's basis directly
# to that permutation matrix (columns = images of gltf's local x/y/z axes:
# gltf+X/left -> iso+Y, gltf+Y/up -> iso+Z, gltf+Z/forward -> iso+X), and
# every local axis used for animation below is likewise the RAW gltf axis
# that the rig says carries that motion (bump/steer on gltf's own +Y - the
# file's up axis; wheel spin about gltf's own +X - the file's left/right
# axle axis) - child nodes' local translate/rotate stay in the file's own,
# un-permuted axes; only _model_root's own basis needs the permutation, once.
# Confirmed by screenshotting the corrected orientation (out/renders/).

@export var simulation_path: NodePath
@export var vehicle_name: String = ""
@export var model_absolute_path: String = "" # set by main.gd before add_child

var _simulation: Node
var _wheel_nodes: Array = []
var _model_root: Node3D
var _load_ok: bool = false
var _eye_node: Node3D # socket_driver_eye, null if the model has none

func _ready() -> void:
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)
	_load_model()
	_bind_wheel_nodes()
	set_process(true)

func model_loaded() -> bool:
	return _load_ok

# Driver eye point for cockpit_cam.gd: the art's own "socket_driver_eye"
# node (every data/models rig has one), expressed in THIS node's frame -
# the chassis body's ISO-local frame, _model_root's gltf->ISO permutation
# and the physics-wheel alignment offset included. Walks the local
# transforms up to this node rather than using global_position, so the
# result does not depend on whether the chassis node has been placed yet
# this frame.
func has_driver_eye() -> bool:
	return _eye_node != null

func driver_eye_local() -> Vector3:
	var xf := Transform3D.IDENTITY
	var n: Node = _eye_node
	while n != null and n != self:
		if n is Node3D:
			xf = (n as Node3D).transform * xf
		n = n.get_parent()
	return xf.origin

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
	# gltf-local -> ISO-local permutation (see file header comment):
	# iso.x (fwd) = gltf.z, iso.y (left) = gltf.x, iso.z (up) = gltf.y.
	# As a basis, column i is where gltf's local axis i (x,y,z) ends up:
	# gltf+X (left) -> (0,1,0), gltf+Y (up) -> (0,0,1), gltf+Z (fwd) -> (1,0,0).
	# A pure axis permutation (determinant +1: an even permutation of the
	# three axes), so this never mirrors the mesh.
	_model_root.transform.basis = Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0))
	_load_ok = true
	_eye_node = _model_root.find_child("socket_driver_eye", true, false) as Node3D
	_matte_inside_of_body_shell(_model_root)

# Cockpit view fix (owner 2026-09-29, "the windows are floating"): the art
# is built to be seen from outside. From the driver's eye (a) single-sided
# exterior parts - the black trim framing the glass, door frames - are
# back-face culled, so there is open sky where they should be, and (b) the
# double-sided METALLIC paint shell (no headliner) mirrors the sky from
# inside. Both made the glass look like panes floating in the air. So every
# opaque surface gets a next_pass that draws only its back faces (cull
# front) as matte dark trim; a double-sided metallic surface is first made
# single-sided so its outside keeps the art's material. Double-sided
# non-metallic surfaces (interior trim, rims) already look right from
# behind and are left alone, as is the glass. From outside only what is
# seen through the windows changes. The art files stay untouched
# (data/models is read-only here).
func _matte_inside_of_body_shell(root: Node) -> void:
	var inner := StandardMaterial3D.new()
	inner.cull_mode = BaseMaterial3D.CULL_FRONT
	inner.albedo_color = Color(0.05, 0.05, 0.055)
	inner.metallic = 0.0
	inner.roughness = 0.9
	var patched := {} # original material -> its patched copy, shared across meshes
	for mi in root.find_children("*", "MeshInstance3D", true, false):
		var mesh: Mesh = (mi as MeshInstance3D).mesh
		if mesh == null:
			continue
		for s in range(mesh.get_surface_count()):
			var m := mesh.surface_get_material(s) as BaseMaterial3D
			if m == null or m.transparency != BaseMaterial3D.TRANSPARENCY_DISABLED:
				continue
			var single_sided := m.cull_mode == BaseMaterial3D.CULL_BACK
			var mirror_inside := m.cull_mode == BaseMaterial3D.CULL_DISABLED and m.metallic >= 0.5
			if not (single_sided or mirror_inside):
				continue
			if not patched.has(m):
				var outer := m.duplicate() as BaseMaterial3D
				outer.cull_mode = BaseMaterial3D.CULL_BACK
				outer.next_pass = inner
				patched[m] = outer
			(mi as MeshInstance3D).set_surface_override_material(s, patched[m])
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
	_align_model_to_physics_wheels()

# Mount the model so its wheel centres coincide with the PHYSICS wheel
# centres. The rig's origin is the ground point below the wheel centres
# (car_sedan.rig.json "origin"), but the chassis body's origin is wherever
# main.gd put the chassis box - ~0.6 m up for car_sedan. Placing the model
# origin there floated the whole car by that much (owner report
# 2026-09-24). Physics: a wheel centre sits at attachment_local +
# axis*compression (wheel_instance.cpp, axis = ISO +z here). Model: at
# rest_susp_up + susp offset. So the model root shifts by the mean of
# (attach - rest_susp_pos, re-permuted into ISO) per axis, and each susp
# node carries its own small residual bias, making every wheel exact even
# if the art and the vehicle file disagree a little.
#
# X/Y generalisation (PHYS-033 P4 I5 prep, car_sedan 60/40 static-split
# data change, 2026-09-25): this used to compute Z only, because
# car_sedan.json's wheel attachment_local x/y always matched the rig's own
# susp_* pivots exactly. Moving car_sedan.json's front/rear attachment_local
# x (wheelbase kept, only the split relative to the chassis body origin/CoM
# changed - see that file's own "source" field) broke that assumption, so
# this now aligns all three axes the same way Z always was: the model root
# (body mesh + every wheel arch together) shifts by the mean per-axis
# offset, and wheels stay in their own (unmoved) wheel arches; only the
# invisible Jolt collision box can now sit very slightly off the visible
# mesh (already documented as coarser than the visible geometry - see
# vehicle_test_helpers.h's build_vehicle_rig doc comment). The tractor's
# rear attachment (PHYS-033 G1, Z-only) is the precedent this generalises.
func _align_model_to_physics_wheels() -> void:
	var diffs: Array = [] # ISO-local (dx, dy, dz) per wheel, or null
	for i in range(_wheel_nodes.size()):
		var w = _wheel_nodes[i]
		if w["susp"] == null:
			diffs.append(null)
			continue
		var attach: Vector3 = _simulation.get_wheel_attachment_local(vehicle_name, i)
		# susp_* is a direct child of the model's body node at the origin,
		# its position in the file's own gltf axes. gltf(x,y,z) = iso(y,z,x)
		# (car_sedan.rig.json "frame"), so re-permuting rest_susp_pos back
		# into ISO gives (rest_susp_pos.z, rest_susp_pos.x, rest_susp_pos.y).
		var rest_susp_pos: Vector3 = w["rest_susp_pos"]
		var rest_iso := Vector3(rest_susp_pos.z, rest_susp_pos.x, rest_susp_pos.y)
		diffs.append(attach - rest_iso)
	var sum := Vector3.ZERO
	var n := 0
	for d in diffs:
		if d != null:
			sum += d
			n += 1
	if n == 0:
		return
	var d_mean: Vector3 = sum / n
	_model_root.position = d_mean # ISO body-local, like the basis above
	for i in range(_wheel_nodes.size()):
		_wheel_nodes[i]["susp_bias"] = (diffs[i] - d_mean) if diffs[i] != null else Vector3.ZERO

func _process(delta: float) -> void:
	if _simulation == null or vehicle_name == "" or _wheel_nodes.is_empty():
		return
	var t0 := Time.get_ticks_usec()

	for i in range(_wheel_nodes.size()):
		var w = _wheel_nodes[i]
		var compression: float = _simulation.get_wheel_compression(vehicle_name, i)
		var omega: float = _simulation.get_wheel_omega(vehicle_name, i)
		w["spin_angle"] = fmod(w["spin_angle"] + omega * delta, TAU)

		if w["susp"] != null:
			# Suspension axis is ISO [0,0,1] (up) for every corner of this
			# rig. susp_*'s own local translation is a RAW gltf-local
			# offset (child nodes keep the file's own, un-permuted axes -
			# see _load_model's comment), and the file's up axis is its
			# own +Y (gltf is Y-up; iso.z=gltf.y). Real simulated state
			# (World::wheel_telemetry's compression), plus this wheel's own
			# residual (dx, dy, dz) bias left over after
			# _align_model_to_physics_wheels' common per-axis mean was
			# removed (usually exactly zero - see that function's own
			# comment - but not for a rig whose art and vehicle-file
			# attachment points disagree per-wheel, e.g. PHYS-033 G1),
			# re-permuted from ISO back into the file's own gltf-local axes
			# (inverse of the iso(x,y,z)=gltf(z,x,y) permutation used when
			# the bias was computed: gltf.x=iso.y, gltf.y=iso.z, gltf.z=iso.x).
			var bias_iso: Vector3 = w.get("susp_bias", Vector3.ZERO)
			var bias_gltf := Vector3(bias_iso.y, bias_iso.z, bias_iso.x)
			w["susp"].position = w["rest_susp_pos"] + Vector3(0, compression, 0) + bias_gltf

		if w["steer"] != null and w["steered"]:
			# Real simulated per-wheel steer angle (World::wheel_state's
			# steer_angle, ackermann_wheel_angle()'s actual result for THIS
			# wheel - correct for any ackermann_fraction, front or rear,
			# see file header comment). ackermann_wheel_angle() already
			# returns 0 for a non-steered wheel, so this only needs the
			# w["steered"] guard to skip the get_wheel_steer_angle call
			# (and leave steer_*'s rotation at its rest value) for wheels
			# with no steer_<corner> node's rotation to drive at all.
			# Steer axis ISO [0,0,1] (up) -> gltf-local +Y (see susp_*
			# comment above).
			var steer_angle: float = _simulation.get_wheel_steer_angle(vehicle_name, i)
			w["steer"].rotation = Vector3(0, steer_angle, 0)

		if w["wheel"] != null:
			# Real simulated state (World::wheel_telemetry's omega,
			# integrated here into a visual spin angle). Wheel joint axis
			# ISO [0,1,0] (left/right axle axis, "+ = rolling forward") ->
			# gltf-local +X (iso.y=gltf.x - see _load_model's comment).
			w["wheel"].rotation = Vector3(w["spin_angle"], 0, 0)

	_simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)
