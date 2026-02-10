#ifndef STREAM_AUDIO_HPP
#define STREAM_AUDIO_HPP

#include "base_audio.hpp"
#include "coder_audio.hpp"

class stream_audio {
public:
    using default_base_audio = audio<{.bitrate         = 120'000,
                                      .latency         = 10,
                                      .channels        = 2,
                                      .sample_rate     = 48000,
                                      .bits_per_sample = 16}>;

    using ca_type            = coder_audio<default_base_audio::cfg>;
    using encode_buffer_type = ca_type::encode_buffer_type;
    using buffer_type        = encode_buffer_type;

public:
    static constexpr std::size_t encode_buffer_size = ca_type::encode_buffer_size;

private:
    class input : public default_base_audio {
    public:
        input();

    public:
        buffer_type read_samples();
    };
    class output : public default_base_audio {
    public:
        output();

    public:
        void write_samples(const buffer_type &bytes);
    };

public:
    ca_type::encode_buffer_type read();

    void write(const ca_type::encode_buffer_type &buffer, bool lost);

public:
    void set_mute_read(bool flag) { in.set_mute(flag, false); }
    void set_mute_write(bool flag) { out.set_mute(flag, flag); }

private:
    input  in;
    output out;

    ca_type ca;
};
stream_audio::input::input() : default_base_audio(stream_audio_mode::capture) {
}
stream_audio::input::buffer_type stream_audio::input::read_samples() {
    buffer_type buffer;
    this->read(buffer);
    return buffer;
}
stream_audio::output::output() : default_base_audio(stream_audio_mode::playback) {
}
void stream_audio::output::write_samples(const buffer_type &bytes) {
    this->write(bytes);
}
stream_audio::ca_type::encode_buffer_type stream_audio::read() {
    return ca.encode(in.read_samples());
}

void stream_audio::write(const ca_type::encode_buffer_type &buffer, bool lost) {
    out.write_samples(ca.decode(buffer, lost));
}

#endif
