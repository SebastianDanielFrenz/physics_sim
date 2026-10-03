extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var demo := (load("res://scenes/main.tscn") as PackedScene).instantiate()
	root.add_child(demo)
	await create_timer(0.5).timeout
	demo.set_process(false)
	var camera: Camera3D = demo.free_camera
	camera.set_process(false)
	demo._set_camera_mode(demo.CameraMode.FREE)
	assert(root.get_camera_3d() == camera)
	var sim: Node = demo.simulation
	var sources: PackedVector3Array = sim.get_vehicle_voice_source_positions("car")
	assert(sources.size() > 0)
	var body: Transform3D = sim.get_body_transform("car_chassis")
	var source: Vector3 = body * sources[0]
	camera.global_transform = Transform3D(Basis.IDENTITY,source+Vector3(0,0,5))
	var front: Vector3 = camera.global_transform.affine_inverse()*source
	assert(front.z < -4.9)
	camera.global_basis = Basis(Vector3.UP,PI)
	var rear: Vector3 = camera.global_transform.affine_inverse()*source
	assert(rear.z > 4.9)
	print("DIRECTION_CHECK active_free_camera=",camera.name," front=",front," rear=",rear," PASS")
	quit()
