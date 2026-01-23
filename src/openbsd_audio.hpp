#ifndef OPENBSD_AUDIO_HPP
#define OPENBSD_AUDIO_HPP

#include <sys/audioio.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include "sio_base_audio.hpp"

template<audio_config _cfg>
class audio : public sio_base_audio<_cfg> {
public:
    audio(audio_stream_mode _mode);
    ~audio() override;

protected:
    void init_params() override;
};

template<audio_config _cfg>
audio<_cfg>::audio(audio_stream_mode _mode) : sio_base_audio<_cfg>(_mode) {
    audio_status status;
    if (ioctl(handle, AUDIO_GETSTATUS, &status) == -1)
        audio::template throw_error<audio::audio_stream_error::failed_get_status>();

    if (!status.pause)
        return;

    if (ioctl(handle, AUDIO_START) == -1)
        audio::template throw_error<audio::audio_stream_error::failed_start>();
}

template<audio_config _cfg>
audio<_cfg>::~audio() {
    audio_status status;
    if (ioctl(handle, AUDIO_GETSTATUS, &status) == -1)
        audio::template throw_error<audio::audio_stream_error::failed_get_status>();

    if (status.pause)
        return;

    if (ioctl(handle, AUDIO_STOP) == -1)
        audio::template throw_error<audio::audio_stream_error::failed_stop>();
}
template<audio_config _cfg>
void audio<_cfg>::init_params() {
    audio_swpar ap;

    AUDIO_INITPAR(&ap);

    ap.sig   = 1;
    ap.le    = 1;
    ap.bits  = bits_per_sample;
    ap.bps   = bytes_per_sample;
    ap.msb   = 0;
    ap.rate  = sample_rate;
    ap.pchan = channels;
    ap.rchan = channels;
    ap.nblks = 4;
    ap.round = buffer_size / 4;

    if (ioctl(handle, AUDIO_SETPAR, &ap) == -1)
        audio::template throw_error<audio::audio_stream_error::failed_set_params>();

    if (ioctl(handle, AUDIO_GETPAR, &ap) == -1)
        audio::template throw_error<audio::audio_stream_error::failed_get_params>();
}
#endif
