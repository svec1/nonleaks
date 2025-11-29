#ifndef AIO_HPP
#define AIO_HPP

#if !defined(SNDIO) && !defined(AUDIO4)
#define AR_NAME_D "ALSA";
#include <alsa_audio.hpp>
#elif defined(SNDIO)
#define AR_NAME_D "SNDIO";
#include <sio_audio.hpp>
#else
#define AR_NAME_D "AUDIO4"
#include <bsd_audio.hpp>
#endif

#include <array>
#include <span>

using byte = char;

static constexpr auto AR_NAME = AR_NAME_D;

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
    int writed_bytes = 0;

    while (writed_bytes < bytes.size()) {
        write(bytes.data() + writed_bytes);
        writed_bytes += period_size * channels * 2;
    }
}

input::input() : audio(audio_stream_t::capture) {}
std::array<byte, audio::buffer_size> input::get_samples() {
    std::array<byte, audio::buffer_size> bytes;
    read(bytes.data());
    return bytes;
}

#endif
