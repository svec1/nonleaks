#ifndef AIO_HPP
#define AIO_HPP

#include "base_audio.hpp"

using default_base_audio = audio<{.bitrate = 120'000,
                                  .latency = 10,
                                  .channels = 2,
                                  .sample_rate = 48000,
                                  .bits_per_sample = 16}>;

class input : public default_base_audio {
  public:
    input();

  public:
    buffer_type get_samples();
};
class output : public default_base_audio {
  public:
    output();

  public:
    void play_samples(const buffer_type &bytes);
};

input::input() : default_base_audio(audio_stream_mode::capture) {}
input::buffer_type input::get_samples() {
    buffer_type buffer;
    read(buffer);
    return buffer;
}

output::output() : default_base_audio(audio_stream_mode::playback) {}
void output::play_samples(const buffer_type &bytes) { write(bytes); }

#endif
