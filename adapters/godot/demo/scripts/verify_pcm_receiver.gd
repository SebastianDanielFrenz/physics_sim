extends SceneTree
func _initialize() -> void:
	call_deferred("check")
func check() -> void:
	var audio := PsSpatialAudio.new()
	if not audio.open_pcm_channel("Surround L"):
		print(audio.get_status()); quit(1); return
	print("LIVE_PCM ",audio.get_status())
	var frame := 0
	while frame < 48000*20:
		var needed := mini(audio.get_frames_needed(),48000*20-frame)
		var samples := PackedFloat32Array()
		samples.resize(needed)
		for i in range(needed):
			samples[i]=0.04*sin(TAU*440*(frame+i)/48000.0)
		audio.push_audio(samples)
		frame+=needed
		await create_timer(0.01).timeout
	audio.close()
	quit()
