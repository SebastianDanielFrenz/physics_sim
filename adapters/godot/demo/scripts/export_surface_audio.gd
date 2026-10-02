extends SceneTree
const Tyre = preload("res://scripts/tyre_sound_model.gd")
const Turbo = preload("res://scripts/turbo_sound_model.gd")
const RATE := 24000
func save_wave(path: String, data: PackedByteArray) -> void:
    var wav := AudioStreamWAV.new()
    wav.format = AudioStreamWAV.FORMAT_16_BITS
    wav.mix_rate = RATE
    wav.stereo = false
    wav.data = data
    var result := wav.save_to_wav(path)
    assert(result == OK)
func _initialize() -> void:
    var states: Array = [["asphalt",15,0,0],["asphalt",40,0,0],["asphalt",30,0.8,0],
        ["asphalt",30,0,1],["dirt",30,0.3,0],["grass",30,0.3,0],["asphalt",0,1,0]]
    var pcm := PackedByteArray()
    pcm.resize(states.size() * RATE * 2 * 2)
    var offset := 0
    for state in states:
        var model := Tyre.new(1729)
        model.update(state[1], state[1]+30*state[2],state[2],0.35*state[2],4000,2500,2000,state[0],state[3])
        for i in range(RATE*2):
            var fade := minf(1.0,minf(float(i)/480.0,float(RATE*2-i)/480.0))
            pcm.encode_s16(offset,int(clampf(model.sample(1.0/RATE)*fade*4,-1,1)*32767))
            offset += 2
    save_wave("S:/claude_code/physics_sim/out/one1_reference/tyres_sample_comparison.wav",pcm)
    print("Exported tyres_sample_comparison.wav (+12 dB audition gain).")
    quit()
