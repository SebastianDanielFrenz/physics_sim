extends Node3D
# adapters/godot/demo/scripts/terrain_visual.gd — M1 visual pass: loads
# data/tracks/test_ground/test_ground.glb (the SAME feature meshes the real
# collision comes from) purely for rendering. PsSimulation.import_test_
# ground only imports COLLISION (data/tracks/README.md's "Collision format"
# section - a binary triangle-soup sidecar, no glTF parser involved on the
# core side at all); main.gd's own GroundGrid is a flat, textureless
# world-space shader plane meant only as a speed/distance reference, not a
# stand-in for the real ground - so before this script, the straight/
# skidpad/grass/dirt/ramps/etc. were entirely invisible and the asphalt/
# grass/dirt surfaces the HUD names were never visually distinguishable
# (M1 visual pass problem 2). Loaded at runtime via GLTFDocument, same
# reason as vehicle_visual.gd (data/tracks is a separate session's lane,
# outside res://).
#
# Frame: data/tracks/README.md - "gltf(x, y, z) = world(y, z, x)", exactly
# the same axis permutation as data/models' own convention (independently
# verified against car_sedan.glb's own node transforms and mesh vertex
# bounding boxes - see vehicle_visual.gd's header comment for the full
# derivation), applied to this root node the same way.
#
# Unlike the vehicle, this mesh's own local origin already IS the world's
# fixed absolute (0,0,0) (data/tracks/README.md: "Origin (0, 0, 0) is the
# main spawn point") - it is not attached to any BodyId, so PsBodyNode3D
# does not apply here. Instead this script re-places itself every frame
# from PsSimulation.get_world_origin_godot_position() - a PURE query (see
# that method's own comment) that tracks the shared floating origin the
# same way a body/camera's transform does, without calling rebase_focus
# (which has side effects on that shared state and would fight the
# vehicle/cameras' own calls if aimed at an unrelated fixed point).

@export var simulation_path: NodePath
@export var glb_absolute_path: String = ""

var _simulation: Node
var _model_root: Node3D
var _load_ok: bool = false

func _ready() -> void:
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)
	_load_model()
	set_process(true)

func model_loaded() -> bool:
	return _load_ok

func _load_model() -> void:
	if glb_absolute_path == "" or not FileAccess.file_exists(glb_absolute_path):
		push_warning("terrain_visual.gd: model not found at '%s'" % glb_absolute_path)
		return
	var gltf_doc := GLTFDocument.new()
	var gltf_state := GLTFState.new()
	var err: int = gltf_doc.append_from_file(glb_absolute_path, gltf_state)
	if err != OK:
		push_warning("terrain_visual.gd: GLTFDocument.append_from_file failed (err=%d) for %s" % [err, glb_absolute_path])
		return
	_model_root = gltf_doc.generate_scene(gltf_state)
	if _model_root == null:
		push_warning("terrain_visual.gd: GLTFDocument.generate_scene returned null for %s" % glb_absolute_path)
		return
	add_child(_model_root)
	# Same gltf-local -> world-local permutation as vehicle_visual.gd's
	# _model_root fix (see that file's header comment for the full
	# derivation): gltf+X -> world+Y, gltf+Y -> world+Z, gltf+Z -> world+X.
	# A pure axis permutation (determinant +1), never mirrors the mesh.
	_model_root.transform.basis = Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0))
	_load_ok = true
	print("ps_godot terrain_visual.gd: loaded model ", glb_absolute_path)

func _process(_delta: float) -> void:
	if _simulation == null:
		return
	position = _simulation.get_world_origin_godot_position()
