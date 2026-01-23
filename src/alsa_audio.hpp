#ifndef AAUDIO_HPP
#define AAUDIO_HPP

#include <alsa/asoundlib.h>

#include "base_audio.hpp"

template <audio_config _cfg>
class audio : public noheap::pseudoheap_monotonic_array<1024>,
              public base_audio<_cfg> {
  protected:
    static constexpr snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    static constexpr snd_pcm_format_t format =
        []<typename T,
           std::integral_constant<std::size_t, sizeof(T)> wrapper = {}>(T t) {
        if constexpr (wrapper() == 8)
            return SND_PCM_FORMAT_S8;
        else if constexpr (wrapper() == 16)
            return SND_PCM_FORMAT_S16_LE;
        else if constexpr (wrapper() == 24)
            return SND_PCM_FORMAT_S24_LE;
        else if constexpr (wrapper() == 32)
            return SND_PCM_FORMAT_S32_LE;
        else
            static_assert(false, "Not the correct type of audio format.");
    }
    (std::array<char, audio::cfg.bits_per_sample>{});

  public:
    audio(audio_stream_mode _mode);
    ~audio() override;

  protected:
    void pread(audio::buffer_type::value_type *buffer) override;
    void pwrite(const audio::buffer_type::value_type *buffer) override;

  private:
    void init_params();

  private:
    static void alsa_init_params(snd_pcm_t *&handle,
                                 snd_pcm_hw_params_t *&params);
    template <typename T, typename... Args>
    static constexpr void alsa_throw_if_error(T &&func, Args &&...args);

  private:
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
};
template <audio_config _cfg>
audio<_cfg>::audio(audio_stream_mode _mode) : base_audio<_cfg>(_mode) {
    switch (this->mode) {
    default:
    case audio_stream_mode::playback:
        alsa_throw_if_error(snd_pcm_open, &handle, this->device_playback.data(),
                            SND_PCM_STREAM_PLAYBACK, 0);
        break;
    case audio_stream_mode::capture:
        alsa_throw_if_error(snd_pcm_open, &handle, this->device_capture.data(),
                            SND_PCM_STREAM_CAPTURE, 0);
        break;
    case audio_stream_mode::bidirect:
        this->template throw_error<
            audio::audio_stream_error::architectural_feature>(
            "Bidirect mode is not supported.");
    }

    init_params();

    alsa_throw_if_error(snd_pcm_hw_params, handle, this->params);
    alsa_throw_if_error(snd_pcm_prepare, handle);
}
template <audio_config _cfg> audio<_cfg>::~audio() {
    alsa_throw_if_error(snd_pcm_drop, handle);
    alsa_throw_if_error(snd_pcm_close, handle);
}
template <audio_config _cfg>
void audio<_cfg>::pread(audio<_cfg>::buffer_type::value_type *buffer) {
    while (snd_pcm_readi(handle, buffer, this->cfg.period_size) < 0)
        alsa_throw_if_error(snd_pcm_recover, handle, this->cfg.period_size, 0);
}
template <audio_config _cfg>
void audio<_cfg>::pwrite(const audio<_cfg>::buffer_type::value_type *buffer) {
    while (snd_pcm_writei(handle, buffer, this->cfg.period_size) < 0)
        alsa_throw_if_error(snd_pcm_prepare, handle);
}

template <audio_config _cfg> void audio<_cfg>::init_params() {
    static snd_pcm_hw_params_t *params_global;

    if (this->possible_bidirect_stream) {
        if (!params_global) {
            params_global = this->template malloc<snd_pcm_hw_params_t *>(
                snd_pcm_hw_params_sizeof());
            alsa_init_params(handle, params_global);
        }

        params = params_global;
    } else {
        params = this->template malloc<snd_pcm_hw_params_t *>(
            snd_pcm_hw_params_sizeof());
        alsa_init_params(handle, params);
    }
}
template <audio_config _cfg>
void audio<_cfg>::alsa_init_params(snd_pcm_t *&handle,
                                   snd_pcm_hw_params_t *&params) {
    static snd_pcm_uframes_t pcm_buffer_size =
        audio::cfg.buffer_size * audio::cfg.diviser_for_hardware_buffer;
    static snd_pcm_uframes_t pcm_period_size =
        audio::cfg.period_size * audio::cfg.diviser_for_hardware_buffer;

    alsa_throw_if_error(snd_pcm_hw_params_any, handle, params);

    alsa_throw_if_error(snd_pcm_hw_params_set_access, handle, params, access);
    alsa_throw_if_error(snd_pcm_hw_params_set_format, handle, params, format);
    alsa_throw_if_error(snd_pcm_hw_params_set_channels, handle, params,
                        audio::cfg.channels);
    alsa_throw_if_error(snd_pcm_hw_params_set_rate, handle, params,
                        audio::cfg.sample_rate, 0);

    alsa_throw_if_error(snd_pcm_hw_params_set_buffer_size_near, handle, params,
                        &pcm_buffer_size);
    alsa_throw_if_error(snd_pcm_hw_params_set_period_size_near, handle, params,
                        &pcm_period_size, nullptr);
}

template <audio_config _cfg>
template <typename T, typename... Args>
constexpr void audio<_cfg>::alsa_throw_if_error(T &&func, Args &&...args) {
    static int ret;
    if (ret = func(args...); ret < 0)
        audio::template throw_error<
            audio::audio_stream_error::architectural_feature>(
            "{}", snd_strerror(ret));
}

#endif
