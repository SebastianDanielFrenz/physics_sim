extends Node
# Opt-in, presentation-side recorder. Reads adapter snapshots only.
var simulation: Node
var vehicle_name := "car"
var chassis_name := "car_chassis"
var file: FileAccess
var started := 0
var previous_time := 0.0
var previous_position := Vector3.ZERO
var previous_basis := Basis.IDENTITY
var previous_velocity := Vector3.ZERO
var have_previous := false
var elapsed := 0.0
var flush_elapsed := 0.0
func begin(path: String) -> bool:
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	file = FileAccess.open(path,FileAccess.WRITE)
	if file == null:
		push_error("Vehicle telemetry cannot write " + path)
		return false
	var header := PackedStringArray(["t_s","speed_mps","rpm","gear","throttle","brake","steer","x_m","y_m","z_m","vx_mps","vy_mps","vz_mps","accel_mps2","angular_speed_rad_s","up_dot"])
	for w in range(simulation.get_vehicle_wheel_count(vehicle_name)):
		var prefix: String = simulation.get_wheel_name(vehicle_name,w) + "_"
		for field in ["omega_rad_s","kappa","alpha_rad","fx_n","fy_n","fz_n","compression_m","steer_rad","surface"]:
			header.append(prefix+field)
	file.store_csv_line(header)
	started = Time.get_ticks_usec()
	print("ps_godot tyre recorder: ",path," (60 Hz; flush each second)")
	return true
func _process(delta: float) -> void:
	if file == null or simulation == null: return
	elapsed += delta
	if elapsed < 1.0/60.0: return
	flush_elapsed += elapsed
	elapsed = 0.0
	var t := (Time.get_ticks_usec()-started)/1000000.0
	var transform: Transform3D = simulation.get_body_transform(chassis_name)
	# Absolute position survives the demo's floating-origin rebases.
	var position: Vector3 = transform.origin - simulation.get_world_origin_godot_position()
	var velocity := Vector3.ZERO
	var acceleration := 0.0
	var angular_speed := 0.0
	if have_previous and t > previous_time:
		velocity = (position-previous_position)/(t-previous_time)
		acceleration = (velocity-previous_velocity).length()/(t-previous_time)
		angular_speed = (previous_basis.inverse()*transform.basis).get_rotation_quaternion().get_angle()/(t-previous_time)
	previous_time=t; previous_position=position; previous_basis=transform.basis; previous_velocity=velocity; have_previous=true
	var pt: Dictionary = simulation.get_vehicle_powertrain(vehicle_name)
	var row := PackedStringArray()
	for value in [t,simulation.get_vehicle_ground_speed_mps(vehicle_name),pt.get("rpm",0),pt.get("gear",0),simulation.get_control("throttle"),simulation.get_control("brake"),simulation.get_control("steer"),position.x,position.y,position.z,velocity.x,velocity.y,velocity.z,acceleration,transform.basis.z.dot(Vector3.UP)]:
		row.append(str(value))
	# Insert angular speed before up_dot to match the header.
	row.insert(row.size()-1,str(angular_speed))
	for w in range(simulation.get_vehicle_wheel_count(vehicle_name)):
		for value in [simulation.get_wheel_omega(vehicle_name,w),simulation.get_wheel_slip_ratio(vehicle_name,w),simulation.get_wheel_slip_angle(vehicle_name,w),simulation.get_wheel_fx(vehicle_name,w),simulation.get_wheel_fy(vehicle_name,w),simulation.get_wheel_load_n(vehicle_name,w),simulation.get_wheel_compression(vehicle_name,w),simulation.get_wheel_steer_angle(vehicle_name,w),simulation.get_wheel_surface_name(vehicle_name,w)]:
			row.append(str(value))
	file.store_csv_line(row)
	if flush_elapsed >= 1.0:
		file.flush(); flush_elapsed=0.0
func _exit_tree() -> void:
	if file != null:
		file.flush(); file.close()
