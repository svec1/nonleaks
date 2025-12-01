#ifndef SUDIO_HPP
#define SUDIO_HPP

#include <sndio.h>

#include <base_audio.hpp>
#include <cstring>

class audio : public base_audio<sio_hdl, sio_par> {
   public:
    audio(audio_stream_t mode);
    ~audio();

   public:
    void read(char* buffer) override;
    void write(char* buffer) override;

   private:
    void init_handle() override;
    void init_params() override;
    void init_sound_device() override;

    void dump_handle() override;
    void dump_params() override;
};

audio::audio(audio_stream_t _mode) { init(_mode); }
audio::~audio() { dump(); }
void audio::read(char* buffer) {
    std::memset(buffer, 0, buffer_size);
    ret = sio_read(handle, buffer, buffer_size);

    if (ret <= 0) throw std::runtime_error("Error reading audio.");
}
void audio::write(char* buffer) {
    ret = sio_write(handle, buffer, buffer_size);

    if (ret <= 0) throw std::runtime_error("Error writing audio.");
}
void audio::init_handle() {
    switch (mode) {
        default:
        case audio_stream_t::playback:
            handle = sio_open(device_playback.data(), SIO_PLAY, 0);
            break;
        case audio_stream_t::capture:
            handle = sio_open(device_capture.data(), SIO_REC, 0);
            break;
    }
}
void audio::init_params() {
    static sio_par params_g;
    sio_initpar(&params_g);

    params_g.bits = bits_per_sample;
    params_g.sig = 1;
    params_g.le = SIO_LE_NATIVE;
    params_g.pchan = channels;
    params_g.rate = sample_rate;

    params = &params_g;
}
void audio::init_sound_device() {
    if (!sio_setpar(handle, params))
        throw std::runtime_error("Failed to set audio params.");
    if (!sio_start(handle))
        throw std::runtime_error("Failed to start audio device.");
}
void audio::dump_handle() {
    sio_stop(handle);
    sio_close(handle);
}
void audio::dump_params() {}
#endif
