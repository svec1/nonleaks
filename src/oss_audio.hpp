#ifndef OSS_AUDIO_HPP
#define OSS_AUDIO_HPP

#include <sys/ioctl.h>
#include <sys/soundcard.h>

#include "sio_base_audio.hpp"

template<audio_config _cfg>
class audio : public sio_base_audio<_cfg> {
public:
    audio(stream_audio_mode _mode);
    ~audio() override = default;

protected:
    void init_params() override;
};

template<audio_config _cfg>
audio<_cfg>::audio(stream_audio_mode _mode) : sio_base_audio<_cfg>(_mode) {
}

template<audio_config _cfg>
void audio<_cfg>::init_params() {
    std::size_t cur_param = AFMT_S16_LE;
    if (ioctl(this->handle, SNDCTL_DSP_SETFMT, &cur_param) == -1
        || cur_param != AFMT_S16_LE)
        audio::template throw_error<audio::stream_audio_error::failed_set_params>(
            "Failed to set the format audio.");

    cur_param = this->cfg.channels;
    if (ioctl(this->handle, SNDCTL_DSP_CHANNELS, &cur_param) == -1
        || cur_param != this->cfg.channels)
        audio::template throw_error<audio::stream_audio_error::failed_set_params>(
            "Failed to set the count channels audio.");

    cur_param = this->cfg.sample_rate;
    if (ioctl(this->handle, SNDCTL_DSP_SPEED, &cur_param) == -1
        || cur_param != this->cfg.sample_rate)
        audio::template throw_error<audio::stream_audio_error::failed_set_params>(
            "Failed to set the sample rate audio.");
}
#endif
