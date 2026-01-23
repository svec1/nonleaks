#ifndef NETBSD_AUDIO_HPP
#define NETBSD_AUDIO_HPP

#include <sys/audioio.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#include "sio_base_audio.hpp"

template <audio_config _cfg> class audio : public sio_base_audio<_cfg> {
  public:
    audio(audio_stream_mode _mode);
    ~audio() override;

  protected:
    void init_params() override;
};

template <audio_config _cfg>
audio<_cfg>::audio(audio_stream_mode _mode) : sio_base_audio<_cfg>(_mode) {}
template <audio_config _cfg> audio<_cfg>::~audio() {}
template <audio_config _cfg> void audio<_cfg>::init_params() {
    audio_info_t ap;

    AUDIO_INITINFO(&ap);

    ap.play.channels = channels;
    ap.play.sample_rate = sample_rate;
    ap.play.encoding = AUDIO_ENCODING_SLINEAR_LE;
    ap.play.precision = bits_per_sample;

    ap.record.channels = channels;
    ap.record.sample_rate = sample_rate;
    ap.record.encoding = AUDIO_ENCODING_SLINEAR_LE;
    ap.record.precision = bits_per_sample;

    if (ioctl(handle, AUDIO_SETINFO, &ap) == -1)
        audio::template throw_error<
            audio::audio_stream_error::failed_set_params>();

    if (ioctl(handle, AUDIO_GETINFO, &ap) == -1)
        audio::template throw_error<
            audio::audio_stream_error::failed_get_params>();
}

#endif
