#ifndef SUDIO_HPP
#define SUDIO_HPP

#include <sys/poll.h>
#include <sndio.h>

#include <base_audio.hpp>

class audio : public base_audio<sio_hdl*, sio_par> {
   public:
    audio(audio_stream_mode mode);
    ~audio() override;

   protected:
    void pread(char* buffer) override;
    void pwrite(const char* buffer) override;

   protected:
    void init_handle() override;
    void init_params() override;
    void init_sound_device() override;
    void dump_handle() override;

    void start_audio() override;
    void stop_audio() override;
};

audio::audio(audio_stream_mode _mode) { init(_mode); }
audio::~audio() { dump(); }
void audio::pread(char* buffer) {
    if (sio_read(handle, buffer, buffer_size) <= 0)
        throw_error<audio_stream_error::error_reading>();
}
void audio::pwrite(const char* buffer) {
    if (sio_write(handle, buffer, buffer_size) <= 0)
        throw_error<audio_stream_error::error_writing>();
}
void audio::init_handle() {
    switch (mode) {
	default:
        case audio_stream_mode::playback:
            handle = sio_open(device_playback.data(), SIO_PLAY, 0);
            break;
        case audio_stream_mode::capture:
            handle = sio_open(device_capture.data(), SIO_REC, 0);
            break;
	case audio_stream_mode::bidirect:
            handle = sio_open(device_playback.data(), SIO_PLAY | SIO_REC, 0);
            break;
    }
    
    handle_initialized = handle != NULL;
}
void audio::init_params() {
    sio_initpar(&params);

    params.bits = bits_per_sample;
    params.sig = 1;
    params.le = SIO_LE_NATIVE;
    params.pchan = channels;
    params.rchan = channels;
    params.rate = sample_rate;
    params.appbufsz = buffer_size;
}
void audio::init_sound_device() {
    if (!sio_setpar(handle, &params))
        throw_error<audio_stream_error::failed_set_params>();

    if (!sio_getpar(handle, &params))
        throw_error<audio_stream_error::failed_get_params>();
}
void audio::dump_handle() {
    sio_close(handle);
}
void audio::start_audio() { 
    base_audio<sio_hdl*, sio_par>::start_audio();
    if (!sio_start(handle))
        throw_error<audio_stream_error::failed_start>();
}
void audio::stop_audio() { 
    base_audio<sio_hdl*, sio_par>::stop_audio();
    if (!sio_stop(handle))
        throw_error<audio_stream_error::failed_stop>();
}

#endif
