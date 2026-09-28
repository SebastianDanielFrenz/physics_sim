#include "wav_writer.h"

#include <fstream>
#include <stdexcept>

namespace ps_tools {

namespace {

void put_u32(std::vector<std::uint8_t>& buf, std::uint32_t v) {
    buf.push_back(static_cast<std::uint8_t>(v & 0xFF));
    buf.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
    buf.push_back(static_cast<std::uint8_t>((v >> 24) & 0xFF));
}

void put_u16(std::vector<std::uint8_t>& buf, std::uint16_t v) {
    buf.push_back(static_cast<std::uint8_t>(v & 0xFF));
    buf.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
}

void put_tag(std::vector<std::uint8_t>& buf, const char tag[4]) {
    buf.push_back(static_cast<std::uint8_t>(tag[0]));
    buf.push_back(static_cast<std::uint8_t>(tag[1]));
    buf.push_back(static_cast<std::uint8_t>(tag[2]));
    buf.push_back(static_cast<std::uint8_t>(tag[3]));
}

} // namespace

void write_wav_pcm16(const std::string& path, int sample_rate_hz, int channels, const std::vector<int16_t>& samples) {
    const std::uint32_t bytes_per_sample = 2;
    const std::uint32_t block_align = static_cast<std::uint32_t>(channels) * bytes_per_sample;
    const std::uint32_t byte_rate = static_cast<std::uint32_t>(sample_rate_hz) * block_align;
    const std::uint32_t data_bytes = static_cast<std::uint32_t>(samples.size()) * bytes_per_sample;

    std::vector<std::uint8_t> header;
    header.reserve(44);
    put_tag(header, "RIFF");
    put_u32(header, 36 + data_bytes); // RIFF chunk size: everything after this field
    put_tag(header, "WAVE");
    put_tag(header, "fmt ");
    put_u32(header, 16); // fmt chunk size (PCM)
    put_u16(header, 1);  // format tag: 1 = PCM
    put_u16(header, static_cast<std::uint16_t>(channels));
    put_u32(header, static_cast<std::uint32_t>(sample_rate_hz));
    put_u32(header, byte_rate);
    put_u16(header, static_cast<std::uint16_t>(block_align));
    put_u16(header, 16); // bits per sample
    put_tag(header, "data");
    put_u32(header, data_bytes);

    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) throw std::runtime_error("write_wav_pcm16: could not open '" + path + "' for writing");
    f.write(reinterpret_cast<const char*>(header.data()), static_cast<std::streamsize>(header.size()));
    if (!samples.empty()) {
        // Little-endian host only (this project's build targets are all
        // x86_64) - a plain write of int16_t reproduces the byte layout
        // put_u16 above writes by hand.
        f.write(reinterpret_cast<const char*>(samples.data()),
                static_cast<std::streamsize>(samples.size() * sizeof(std::int16_t)));
    }
    if (!f) throw std::runtime_error("write_wav_pcm16: write failed for '" + path + "'");
}

} // namespace ps_tools
