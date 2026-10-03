extends SceneTree
func _initialize() -> void:
	call_deferred("play_tone")
func play_tone() -> void:
	var spatial := PsSpatialAudio.new()
	spatial.set_reference_distance_m(8.0)
	if not spatial.open(48000,1):
		print(spatial.get_status())
		quit(1)
		return
	print("REAR TONE: ",spatial.get_status())
	spatial.set_positions(PackedVector3Array([Vector3(0,0,3)]))
	var frame := 0
	while frame < 144000:
		var needed := mini(spatial.get_frames_needed(),144000-frame)
		var samples := PackedFloat32Array()
		samples.resize(needed)
		for i in range(needed):
			var f := frame+i
			var envelope := minf(1.0,minf(f/480.0,(144000-f)/480.0))
			samples[i]=0.1*envelope*sin(TAU*440.0*f/48000.0)
		spatial.push_audio(samples)
		frame+=needed
		await create_timer(0.01).timeout
	await create_timer(0.1).timeout
	spatial.close()
	quit()
