extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var spatial := PsSpatialAudio.new()
	assert(not spatial.open_static_channel("unknown"))
	for channel in ["Surround L","Surround Back L"]:
		assert(spatial.open_static_channel(channel))
		print("DIRECT_CHANNEL_CHECK ",channel,": ",spatial.get_status())
		for i in range(10):
			var samples := PackedFloat32Array()
			samples.resize(spatial.get_frames_needed())
			spatial.push_audio(samples)
			await create_timer(0.02).timeout
		assert(spatial.is_active())
		spatial.close()
	assert(spatial.open(48000,1))
	spatial.close()
	print("DIRECT_CHANNEL_CHECK PASS")
	quit()
