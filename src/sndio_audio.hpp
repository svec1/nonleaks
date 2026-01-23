#ifndef SUDIO_HPP
#define SUDIO_HPP

#include <sndio.h>
#include <sys/poll.h>

#include "base_audio.hpp"

template <audio_config _cfg> class audio : public base_audio<_cfg> {
  public:
    audio(audio_stream_mode mode);
    ~audio() override;

  protected:
    void pread(buffer_type::value_type *buffer) override;
    void pwrite(const buffer_type::value_type *buffer) override;

  private:
    static void sndio_init_params(sio_par &params);

  private:
    sio_hdl *handle;
    sio_par *params;
};

template <audio_config _cfg>
audio<_cfg>::audio(audio_stream_mode _mode) : base_audio(_mode) {
    switch (this->mode) {
    default:
    case audio_stream_mode::playback:
        handle = sio_open(this->device_playback.data(), SIO_PLAY, 0);
        break;
    case audio_stream_mode::capture:
        handle = sio_open(this->device_capture.data(), SIO_REC, 0);
        break;
    case audio_stream_mode::bidirect:
        handle = sio_open(this->device_playback.data(), SIO_PLAY | SIO_REC, 0);
        break;
    }
    sndio_init_params();

    if (!sio_setpar(handle, &params))
        audio::template throw_error<
            audio::audio_stream_error::failed_set_params>();
    if (!sio_getpar(handle, &params))
        audio::template throw_error<
            audio::audio_stream_error::failed_get_params>();

    if (!sio_start(handle))
        audio::template throw_error<audio::audio_stream_error::failed_start>();
}
template <audio_config _cfg> audio<_cfg>::~audio() {
    if (!sio_stop(handle))
        audio::template throw_error<audio::audio_stream_error::failed_stop>();
    sio_close(handle);
}
template <audio_config _cfg>
void audio<_cfg>::pread(audio::buffer_type::value_type *buffer) {
    if (sio_read(handle, buffer, this->cfg.buffer_size) <= 0)
        audio::template throw_error<audio::audio_stream_error::error_reading>();
}
template <audio_config _cfg>
void audio<_cfg>::pwrite(const audio::buffer_type::value_type *buffer) {
    if (sio_write(handle, buffer, this->cfg.buffer_size) <= 0)
        audio::template throw_error<audio::audio_stream_error::error_writing>();
}
template <audio_config _cfg>
void audio<_cfg>::sndio_init_params(sio_par &params) {
    sio_initpar(&params);

    params.bits = audio::cfg.bits_per_sample;
    params.bps = audio::cfg.bytes_per_sample;
    params.sig = 1;
    params.le = SIO_LE_NATIVE;
    params.pchan = audio::cfg.channels;
    params.rchan = audio::cfg.channels;
    params.rate = audio::cfg.sample_rate;
    params.appbufsz = audio::cfg.buffer_size;
}

#endif
