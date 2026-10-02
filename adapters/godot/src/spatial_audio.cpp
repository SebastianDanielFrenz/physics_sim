#include "spatial_audio.h"
#include <godot_cpp/core/class_db.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <future>
#include <mutex>
#include <thread>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <ksmedia.h>
#include <initguid.h>
#include <functiondiscoverykeys_devpkey.h>
#include <spatialaudioclient.h>
#include <wrl/client.h>
#endif
namespace ps_godot {
struct PsSpatialAudio::Impl {
    static constexpr int capacity = 16384;
    std::atomic<bool> active{false}, stop{false};
    mutable std::mutex mutex;
    std::thread worker;
    int channels = 0, rate = 0, read = 0, count = 0;
    float reference_distance_m = 1.0f;
    float reflections_gain = 0.0f;
    int static_type = 0;
    int pcm_channel = -1;
    std::vector<float> ring;
    std::vector<godot::Vector3> positions;
    std::string status = "Closed";
    void message(const std::string &s) { std::lock_guard<std::mutex> lock(mutex); status = s; }
    void run_pcm(std::promise<bool> ready) {
#ifdef _WIN32
        using Microsoft::WRL::ComPtr;
        HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        const bool com = SUCCEEDED(hr);
        HANDLE event = nullptr;
        {
            ComPtr<IMMDeviceEnumerator> enumerator;
            ComPtr<IMMDevice> device;
            ComPtr<IAudioClient> client;
            ComPtr<IAudioRenderClient> render;
            if (SUCCEEDED(hr)) hr = CoCreateInstance(__uuidof(MMDeviceEnumerator),nullptr,CLSCTX_ALL,IID_PPV_ARGS(&enumerator));
            if (SUCCEEDED(hr)) hr = enumerator->GetDefaultAudioEndpoint(eRender,eMultimedia,&device);
            if (SUCCEEDED(hr)) hr = device->Activate(__uuidof(IAudioClient),CLSCTX_INPROC_SERVER,nullptr,reinterpret_cast<void **>(client.GetAddressOf()));
            WAVEFORMATEXTENSIBLE format{};
            format.Format.wFormatTag=WAVE_FORMAT_EXTENSIBLE;
            format.Format.nChannels=8; format.Format.nSamplesPerSec=48000;
            format.Format.wBitsPerSample=16; format.Format.nBlockAlign=16;
            format.Format.nAvgBytesPerSec=48000*16; format.Format.cbSize=22;
            format.Samples.wValidBitsPerSample=16;
            format.dwChannelMask=0x63f; format.SubFormat=KSDATAFORMAT_SUBTYPE_PCM;
            if (SUCCEEDED(hr)) hr=client->IsFormatSupported(AUDCLNT_SHAREMODE_EXCLUSIVE,&format.Format,nullptr);
            REFERENCE_TIME duration=1000000; // 100 ms diagnostic buffer.
            if (SUCCEEDED(hr)) hr=client->Initialize(AUDCLNT_SHAREMODE_EXCLUSIVE,AUDCLNT_STREAMFLAGS_EVENTCALLBACK,duration,duration,&format.Format,nullptr);
            if (hr==AUDCLNT_E_BUFFER_SIZE_NOT_ALIGNED) {
                UINT32 aligned=0;
                hr=client->GetBufferSize(&aligned);
                duration=static_cast<REFERENCE_TIME>(10000000.0*aligned/48000.0+0.5);
                client.Reset();
                if (SUCCEEDED(hr)) hr=device->Activate(__uuidof(IAudioClient),CLSCTX_INPROC_SERVER,nullptr,reinterpret_cast<void **>(client.GetAddressOf()));
                if (SUCCEEDED(hr)) hr=client->Initialize(AUDCLNT_SHAREMODE_EXCLUSIVE,AUDCLNT_STREAMFLAGS_EVENTCALLBACK,duration,duration,&format.Format,nullptr);
            }
            UINT32 frames=0;
            if (SUCCEEDED(hr)) { event=CreateEvent(nullptr,FALSE,FALSE,nullptr); if (!event) hr=HRESULT_FROM_WIN32(GetLastError()); }
            if (SUCCEEDED(hr)) hr=client->SetEventHandle(event);
            if (SUCCEEDED(hr)) hr=client->GetBufferSize(&frames);
            if (SUCCEEDED(hr)) hr=client->GetService(IID_PPV_ARGS(&render));
            BYTE *initial=nullptr;
            if (SUCCEEDED(hr)) hr=render->GetBuffer(frames,&initial);
            if (SUCCEEDED(hr)) hr=render->ReleaseBuffer(frames,AUDCLNT_BUFFERFLAGS_SILENT);
            if (SUCCEEDED(hr)) hr=client->Start();
            if (FAILED(hr)) {
                char error[160]; snprintf(error,sizeof(error),"7.1 PCM unavailable (HRESULT 0x%08lx). Check HDMI PCM support and exclusive access; no stereo fallback.",static_cast<unsigned long>(hr));
                message(error); ready.set_value(false);
            } else {
                message("Exclusive 7.1 PCM active: 48 kHz / 16-bit / mask 0x63f; check Denon MULTI CH IN without upmixing");
                active=true; ready.set_value(true);
                while (!stop) {
                    if (WaitForSingleObject(event,500)!=WAIT_OBJECT_0) { if (!stop) hr=HRESULT_FROM_WIN32(ERROR_TIMEOUT); break; }
                    if (stop) break;
                    BYTE *buffer=nullptr;
                    hr=render->GetBuffer(frames,&buffer);
                    if (FAILED(hr)) break;
                    auto out=reinterpret_cast<short *>(buffer);
                    std::fill(out,out+frames*8,0);
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        const int got=std::min(count,static_cast<int>(frames));
                        for (int f=0;f<got;++f) out[f*8+pcm_channel]=static_cast<short>(std::clamp(ring[(read+f)%capacity],-1.0f,1.0f)*32767.0f);
                        read=(read+got)%capacity; count-=got;
                    }
                    hr=render->ReleaseBuffer(frames,0);
                    if (FAILED(hr)) break;
                }
                active=false;
                if (FAILED(hr)) message("7.1 PCM endpoint lost; reconnect audio");
                client->Stop();
            }
        }
        if (event) CloseHandle(event);
        if (com) CoUninitialize();
#else
        message("Windows PCM diagnostic unavailable on this platform"); ready.set_value(false);
#endif
    }
    void run(std::promise<bool> ready) {
        if (pcm_channel >= 0) { run_pcm(std::move(ready)); return; }
#ifdef _WIN32
        using Microsoft::WRL::ComPtr;
        HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        const bool com = SUCCEEDED(hr);
        HANDLE event = nullptr;
        // All interfaces are created, used and released in this COM apartment.
        {
            ComPtr<IMMDeviceEnumerator> enumerator;
            ComPtr<IMMDevice> device;
            ComPtr<ISpatialAudioClient> client;
            ComPtr<ISpatialAudioObjectRenderStream> stream;
            std::vector<ComPtr<ISpatialAudioObject>> objects(channels);
            const AudioObjectType reflection_types[] = {AudioObjectType_BackLeft, AudioObjectType_BackRight,
                AudioObjectType_TopFrontLeft, AudioObjectType_TopFrontRight, AudioObjectType_TopBackLeft, AudioObjectType_TopBackRight};
            ComPtr<ISpatialAudioObject> reflections[6];
            // Decorrelated short filtered delay tails, an artistic diffuse mix.
            const int delays[] = {1493,1871,2137,2477,2909,3253};
            std::vector<float> delay_lines[6];
            int delay_read[6]{};
            float low_pass[6]{};
            for (int c = 0; c < 6; ++c) delay_lines[c].assign(delays[c], 0.0f);
            UINT32 max_objects = 0;
            AudioObjectType native_mask = AudioObjectType_None;
            std::string device_name = "default multimedia endpoint";
            if (SUCCEEDED(hr)) hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, IID_PPV_ARGS(&enumerator));
            if (SUCCEEDED(hr)) hr = enumerator->GetDefaultAudioEndpoint(eRender, eMultimedia, &device);
            if (SUCCEEDED(hr)) hr = device->Activate(__uuidof(ISpatialAudioClient), CLSCTX_INPROC_SERVER, nullptr, reinterpret_cast<void **>(client.GetAddressOf()));
            if (SUCCEEDED(hr)) hr = client->GetMaxDynamicObjectCount(&max_objects);
            if (SUCCEEDED(hr)) hr = client->GetNativeStaticObjectTypeMask(&native_mask);
            if (device) {
                ComPtr<IPropertyStore> properties;
                if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &properties))) {
                    PROPVARIANT name{};
                    if (SUCCEEDED(properties->GetValue(PKEY_Device_FriendlyName, &name)) && name.vt == VT_LPWSTR) {
                        char utf8[512]{};
                        WideCharToMultiByte(CP_UTF8, 0, name.pwszVal, -1, utf8, sizeof(utf8), nullptr, nullptr);
                        device_name = utf8;
                    }
                    PropVariantClear(&name);
                }
            }
            if (SUCCEEDED(hr) && !static_type && max_objects < static_cast<UINT32>(channels)) hr = E_NOTIMPL;
            if (SUCCEEDED(hr) && static_type && !(native_mask & static_cast<AudioObjectType>(static_type))) hr = E_NOTIMPL;
            WAVEFORMATEX format{};
            format.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
            format.nChannels = 1; format.nSamplesPerSec = rate;
            format.wBitsPerSample = 32; format.nBlockAlign = 4; format.nAvgBytesPerSec = rate * 4;
            if (SUCCEEDED(hr)) hr = client->IsAudioObjectFormatSupported(&format);
            if (SUCCEEDED(hr)) {
                event = CreateEvent(nullptr, FALSE, FALSE, nullptr);
                if (!event) hr = HRESULT_FROM_WIN32(GetLastError());
            }
            SpatialAudioObjectRenderStreamActivationParams params{};
            params.ObjectFormat = &format; params.StaticObjectTypeMask = AudioObjectType_None;
            if (static_type) params.StaticObjectTypeMask = static_cast<AudioObjectType>(static_type);
            if (!static_type && reflections_gain > 0) for (const auto type : reflection_types)
                if ((native_mask & type) != AudioObjectType_None) params.StaticObjectTypeMask |= type;
            params.MinDynamicObjectCount = static_type ? 0 : channels; params.MaxDynamicObjectCount = static_type ? 0 : channels;
            params.Category = AudioCategory_GameEffects; params.EventHandle = event;
            PROPVARIANT activation{}; activation.vt = VT_BLOB;
            activation.blob.cbSize = sizeof(params); activation.blob.pBlobData = reinterpret_cast<BYTE *>(&params);
            if (SUCCEEDED(hr)) hr = client->ActivateSpatialAudioStream(&activation, __uuidof(ISpatialAudioObjectRenderStream), reinterpret_cast<void **>(stream.GetAddressOf()));
            if (SUCCEEDED(hr)) hr = stream->Start();
            if (FAILED(hr)) {
                char error[100]; snprintf(error, sizeof(error), "Spatial audio unavailable (HRESULT 0x%08lx); use Godot fallback", static_cast<unsigned long>(hr));
                message(error); ready.set_value(false);
            } else {
                char capabilities[180];
                snprintf(capabilities, sizeof(capabilities), "; native speaker bed mask=0x%lx, dynamic capacity=%u; renderer selected in Windows Sound settings", static_cast<unsigned long>(native_mask), max_objects);
                message(std::string(static_type ? "Direct static speaker channel active on " : "Windows spatial objects active on ") + device_name + capabilities);
                active = true; ready.set_value(true);
                while (!stop) {
                    if (WaitForSingleObject(event, 250) != WAIT_OBJECT_0) { if (!stop) hr = HRESULT_FROM_WIN32(ERROR_TIMEOUT); break; }
                    if (stop) break;
                    UINT32 available = 0, frames = 0;
                    hr = stream->BeginUpdatingAudioObjects(&available, &frames);
                    if (FAILED(hr)) break;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        const int got = std::min(count, static_cast<int>(frames));
                        for (int c = 0; c < 6; ++c) {
                            if (static_type) break;
                            if ((params.StaticObjectTypeMask & reflection_types[c]) == AudioObjectType_None) continue;
                            if (!reflections[c]) hr = stream->ActivateSpatialAudioObject(reflection_types[c], &reflections[c]);
                            if (FAILED(hr)) break;
                            BYTE *buffer = nullptr; UINT32 bytes = 0;
                            hr = reflections[c]->GetBuffer(&buffer, &bytes);
                            if (FAILED(hr) || bytes < frames*sizeof(float)) { hr = E_FAIL; break; }
                            auto out = reinterpret_cast<float *>(buffer);
                            for (UINT32 f = 0; f < frames; ++f) {
                                float input = 0.0f;
                                if (f < static_cast<UINT32>(got)) for (int source = 0; source < channels; ++source) {
                                    const float attenuation = reference_distance_m / std::max(reference_distance_m, static_cast<float>(positions[source].length()));
                                    input += ring[((read+f)%capacity)*channels+source]*attenuation / channels;
                                }
                                const float tail = delay_lines[c][delay_read[c]];
                                low_pass[c] += 0.12f*(input + 0.55f*tail - low_pass[c]);
                                delay_lines[c][delay_read[c]] = low_pass[c];
                                delay_read[c] = (delay_read[c]+1)%delays[c];
                                out[f] = std::clamp(tail*reflections_gain,-1.0f,1.0f);
                            }
                        }
                        for (int c = 0; c < channels; ++c) {
                            if (FAILED(hr)) break;
                            if (!objects[c]) hr = stream->ActivateSpatialAudioObject(static_type ? static_cast<AudioObjectType>(static_type) : AudioObjectType_Dynamic, &objects[c]);
                            if (FAILED(hr)) break;
                            const auto p = positions[c];
                            if (!static_type) hr = objects[c]->SetPosition(p.x, p.y, p.z);
                            if (FAILED(hr)) break;
                            // Preserve full gain within the reference distance,
                            // then roll off as 1/r. Source positions stay real.
                            const float gain = static_type ? 1.0f : reference_distance_m / std::max(reference_distance_m, static_cast<float>(p.length()));
                            BYTE *buffer = nullptr; UINT32 bytes = 0;
                            hr = objects[c]->GetBuffer(&buffer, &bytes);
                            if (FAILED(hr) || bytes < frames * sizeof(float)) { hr = E_FAIL; break; }
                            auto out = reinterpret_cast<float *>(buffer);
                            std::fill(out, out + frames, 0.0f);
                            for (int f = 0; f < got; ++f) out[f] = std::clamp(ring[((read + f) % capacity) * channels + c] * gain, -1.0f, 1.0f);
                        }
                        read = (read + got) % capacity; count -= got;
                    }
                    const HRESULT end = stream->EndUpdatingAudioObjects();
                    if (FAILED(hr) || FAILED(end)) { if (SUCCEEDED(hr)) hr = end; break; }
                }
                active = false;
                if (FAILED(hr)) message("Spatial endpoint lost; use Godot fallback");
                stream->Stop(); stream->Reset();
            }
        }
        if (event) CloseHandle(event);
        if (com) CoUninitialize();
#else
        message("Windows spatial audio is unavailable on this platform"); ready.set_value(false);
#endif
    }
};
PsSpatialAudio::PsSpatialAudio() : impl_(std::make_unique<Impl>()) {}
PsSpatialAudio::~PsSpatialAudio() { close(); }
bool PsSpatialAudio::open(int rate, int channels) {
    return open_internal(rate, channels, 0);
}
bool PsSpatialAudio::open_static_channel(const godot::String &name) {
    // AudioObjectType values are defined by the Windows SDK, not speaker indices.
    if (name == "Surround L") return open_internal(48000, 1, 1 << 5);
    if (name == "Surround Back L") return open_internal(48000, 1, 1 << 7);
    close(); impl_->message("Unknown direct speaker channel"); return false;
}
bool PsSpatialAudio::open_pcm_channel(const godot::String &name) {
    // WAVEFORMATEXTENSIBLE 7.1 order: FL FR FC LFE BL BR SL SR.
    if (name == "Surround L") return open_internal(48000,1,0,6);
    if (name == "Surround Back L") return open_internal(48000,1,0,4);
    close(); impl_->message("Unknown PCM speaker channel"); return false;
}
bool PsSpatialAudio::open_internal(int rate, int channels, int static_type, int pcm_channel) {
    close();
    impl_->static_type = static_type;
    impl_->pcm_channel = pcm_channel;
    if (rate != 48000 || channels < 1 || channels > 16) { impl_->message("Requires 48 kHz and 1–16 source channels; use Godot fallback"); return false; }
    impl_->rate = rate; impl_->channels = channels; impl_->read = impl_->count = 0;
    impl_->ring.assign(Impl::capacity * channels, 0); impl_->positions.assign(channels, godot::Vector3(0, 0, -1));
    impl_->stop = false;
    std::promise<bool> ready; auto result = ready.get_future();
    impl_->worker = std::thread(&Impl::run, impl_.get(), std::move(ready));
    return result.get();
}
void PsSpatialAudio::close() { impl_->stop = true; if (impl_->worker.joinable()) impl_->worker.join(); impl_->active = false; }
bool PsSpatialAudio::is_active() const { return impl_->active; }
godot::String PsSpatialAudio::get_status() const { std::lock_guard<std::mutex> lock(impl_->mutex); return godot::String(impl_->status.c_str()); }
int PsSpatialAudio::get_frames_needed() const { std::lock_guard<std::mutex> lock(impl_->mutex); return is_active() ? std::max(0, impl_->rate / (impl_->pcm_channel >= 0 ? 5 : 50) - impl_->count) : 0; }
void PsSpatialAudio::push_audio(const godot::PackedFloat32Array &samples) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!is_active() || impl_->channels < 1) return;
    const int frames = std::min(static_cast<int>(samples.size() / impl_->channels), Impl::capacity - impl_->count);
    for (int f = 0; f < frames; ++f) for (int c = 0; c < impl_->channels; ++c) {
        const float s = samples[f * impl_->channels + c];
        impl_->ring[((impl_->read + impl_->count + f) % Impl::capacity) * impl_->channels + c] = std::isfinite(s) ? s : 0;
    }
    impl_->count += frames;
}
void PsSpatialAudio::set_positions(const godot::PackedVector3Array &positions) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (positions.size() != impl_->channels) return;
    for (int c = 0; c < impl_->channels; ++c) if (positions[c].is_finite()) impl_->positions[c] = positions[c];
}
void PsSpatialAudio::_bind_methods() {
    // Distance is a mix control, separate from the object's real position.
    using namespace godot;
    ClassDB::bind_method(D_METHOD("open", "sample_rate", "channels"), &PsSpatialAudio::open);
    ClassDB::bind_method(D_METHOD("open_static_channel", "channel_name"), &PsSpatialAudio::open_static_channel);
    ClassDB::bind_method(D_METHOD("open_pcm_channel", "channel_name"), &PsSpatialAudio::open_pcm_channel);
    ClassDB::bind_method(D_METHOD("close"), &PsSpatialAudio::close);
    ClassDB::bind_method(D_METHOD("is_active"), &PsSpatialAudio::is_active);
    ClassDB::bind_method(D_METHOD("get_status"), &PsSpatialAudio::get_status);
    ClassDB::bind_method(D_METHOD("get_frames_needed"), &PsSpatialAudio::get_frames_needed);
    ClassDB::bind_method(D_METHOD("push_audio", "samples"), &PsSpatialAudio::push_audio);
    ClassDB::bind_method(D_METHOD("set_positions", "listener_relative"), &PsSpatialAudio::set_positions);
    ClassDB::bind_method(D_METHOD("set_reference_distance_m", "metres"), &PsSpatialAudio::set_reference_distance_m);
    ClassDB::bind_method(D_METHOD("set_reflections_gain", "gain"), &PsSpatialAudio::set_reflections_gain);
}
void PsSpatialAudio::set_reference_distance_m(double metres) {
    if (!std::isfinite(metres) || metres < 0.1 || metres > 1000.0) return;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->reference_distance_m = static_cast<float>(metres);
}
void PsSpatialAudio::set_reflections_gain(double gain) {
    if (!std::isfinite(gain) || gain < 0.0 || gain > 0.5) return;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->reflections_gain = static_cast<float>(gain);
}
}
