extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var audio := PsSpatialAudio.new()
	assert(not audio.open_pcm_channel("unknown"))
	for channel in ["Surround L","Surround Back L"]:
		var opened := audio.open_pcm_channel(channel)
		print("PCM_CHECK ",channel," opened=",opened," ",audio.get_status())
		if opened:
			for i in range(15):
				var samples := PackedFloat32Array()
				samples.resize(audio.get_frames_needed())
				audio.push_audio(samples)
				await create_timer(0.02).timeout
			assert(audio.is_active())
		audio.close()
	assert(audio.open(48000,1))
	audio.close()
	print("PCM_CHECK lifecycle PASS")
	quit()
