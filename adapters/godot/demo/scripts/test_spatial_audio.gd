extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var spatial := PsSpatialAudio.new()
	assert(not spatial.open(44100, 2))
	assert(not spatial.is_active())
	assert(not spatial.open(48000, 0))
	var opened := spatial.open(48000, 2)
	print("SPATIAL_CHECK opened=",opened," status=",spatial.get_status())
	if opened:
		spatial.set_positions(PackedVector3Array([Vector3(-1,0,-1),Vector3(1,0,1)]))
		for i in range(20):
			var samples := PackedFloat32Array()
			samples.resize(spatial.get_frames_needed()*2)
			spatial.push_audio(samples)
			await create_timer(0.02).timeout
		assert(spatial.is_active())
	spatial.close()
	spatial.close()
	assert(not spatial.is_active())
	print("SPATIAL_CHECK lifecycle PASS")
	quit()
