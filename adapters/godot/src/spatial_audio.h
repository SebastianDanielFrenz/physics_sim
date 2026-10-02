#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <memory>
namespace ps_godot {
// Main-thread Godot interface; Windows COM and rendering live on a private thread.
class PsSpatialAudio : public godot::RefCounted {
    GDCLASS(PsSpatialAudio, godot::RefCounted)
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool open_internal(int sample_rate, int channels, int static_type, int pcm_channel = -1);
protected:
    static void _bind_methods();
public:
    PsSpatialAudio();
    ~PsSpatialAudio();
    bool open(int sample_rate, int channels);
    bool open_static_channel(const godot::String &channel_name);
    bool open_pcm_channel(const godot::String &channel_name);
    void close();
    bool is_active() const;
    godot::String get_status() const;
    int get_frames_needed() const;
    void push_audio(const godot::PackedFloat32Array &samples);
    void set_positions(const godot::PackedVector3Array &listener_relative);
    void set_reference_distance_m(double metres);
    void set_reflections_gain(double gain);
};
}
