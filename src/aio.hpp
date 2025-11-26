#ifndef AIO_HPP
#define AIO_HPP

#if !defined(SNDIO) && !defined(AUDIO4)
#include <alsa_audio.hpp>
#elif defined(SNDIO)
#include <sio_audio.hpp>
#else
#include <bsd_audio.hpp>
#endif

#include <array>
#include <span>

using byte = char;

class output : private audio {
   public:
    output();

   public:
    void play_samples(std::span<byte> bytes);
};
class input : private audio {
   public:
    input();

   public:
    std::array<byte, audio::buffer_size> get_samples();
};


output::output() : audio(audio_stream_t::playback) {}
void output::play_samples(std::span<byte> bytes) {
    int ret, writed_bytes = 0;

    while (writed_bytes < bytes.size()) {
        write(bytes.data() + writed_bytes, period_size);
	writed_bytes += period_size * channels * 2;
    }
}

input::input() : audio(audio_stream_t::capture) {}
std::array<byte, audio::buffer_size> input::get_samples() {
    std::array<byte, audio::buffer_size> bytes;
    read(bytes.data(), period_size);
    return bytes;
}

#endif
