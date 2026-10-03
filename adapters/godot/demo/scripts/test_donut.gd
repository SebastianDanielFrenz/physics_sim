extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var demo := (load("res://scenes/main.tscn") as PackedScene).instantiate()
	root.add_child(demo)
	await create_timer(0.5).timeout
	demo.set_process(false)
	var sim: Node = demo.get_node("PsSimulation")
	sim.set_control("shift_up_count",1.0)
	sim.set_control("throttle",0.7)
	for i in range(240):
		if i == 25: sim.set_control("steer",1.0)
		if i == 65:
			sim.set_control("throttle",0.0)
			sim.set_control("brake",0.6)
		if i == 85: sim.set_control("brake",0.0)
		if i == 110: sim.set_control("throttle",0.5)
		if i == 170: sim.set_control("steer",0.0)
		await create_timer(0.1).timeout
		if i % 5 == 0:
			var row: Array = [i*0.1,sim.get_vehicle_ground_speed_mps("car")]
			for w in range(4):
				row.append([sim.get_wheel_omega("car",w),sim.get_wheel_slip_ratio("car",w),sim.get_wheel_slip_angle("car",w),sim.get_wheel_fx("car",w),sim.get_wheel_fy("car",w),sim.get_wheel_load_n("car",w)])
			print("DONUT ",row)
	quit()
