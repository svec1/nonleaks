#ifndef AIO_HPP
#define AIO_HPP

#include <array>
#include <span>

#include "base_audio.hpp"

class input : private audio {
  public:
    input();

  public:
    audio::buffer_t get_samples();
};
class output : private audio {
  public:
    output();

  public:
    void play_samples(const audio::buffer_t &bytes);
};

input::input() : audio(audio_stream_mode::capture) {}
audio::buffer_t input::get_samples() {
    audio::buffer_t bytes;
    read(bytes);
    return bytes;
}

output::output() : audio(audio_stream_mode::playback) {}
void output::play_samples(const audio::buffer_t &bytes) { write(bytes); }

#endif
