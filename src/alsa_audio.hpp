#ifndef AAUDIO_HPP
#define AAUDIO_HPP

#include <alsa/asoundlib.h>
#include <base_audio.hpp>

#include <cstdio>
#include <format>
#include <functional>
#include <string>

#define snd_call(func, ...)                  \
{                                            \
	snd_call_(func, #func, __VA_ARGS__); \
}


class audio : public base_audio<snd_pcm_t, snd_pcm_hw_params_t>{
   public:
    audio(audio_stream_t _mode);
    
   public:
    void read(char* buffer, unsigned int period_size) override;
    void write(char* buffer, unsigned int period_size) override;

   private:
    void init_handle() override;
    void init_params() override;
    void init_sound_device() override;

    void dump_handle() override;
    void dump_params() override;

   private:
    template <typename T, typename... Args>
    static void snd_call_(T&& func, std::string_view name_func, Args&&... args) {
    	if (int ret = func(args...); ret < 0) {
        	throw std::runtime_error(
            		std::format("Error({}): {}", name_func, snd_strerror(ret)));
    	}
    }

   protected:
    static constexpr snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    static constexpr snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
};
audio::audio(audio_stream_t _mode) : base_audio<snd_pcm_t, snd_pcm_hw_params_t>(_mode) {
}
void audio::read(char* buffer, unsigned int period_size) {
     static int ret;
     while ((ret = snd_pcm_readi(handle, buffer, period_size)) < 0)
            snd_call(snd_pcm_prepare, handle);
}
void audio::write(char* buffer, unsigned int period_size) {
     static int ret;
     while ((ret = snd_pcm_writei(handle, buffer, period_size)) < 0)
            snd_call(snd_pcm_prepare, handle);
}
void audio::init_handle() {
    snd_call(snd_pcm_open, &handle,
             (mode == audio_stream_t::playback ? device_playback : device_capture).data(), mode == audio_stream_t::playback ? snd_pcm_stream_t::SND_PCM_STREAM_PLAYBACK ? snd_pcm_stream_t::SND_PCM_STREAM_CAPTURE, 0);
}
void audio::init_params() {
    static snd_pcm_uframes_t pcm_period_size = 940;

    snd_call(snd_pcm_hw_params_malloc, &params);
    snd_call(snd_pcm_hw_params_any, handle, params);

    snd_call(snd_pcm_hw_params_set_access, handle, params, access);
    snd_call(snd_pcm_hw_params_set_format, handle, params, format);
    snd_call(snd_pcm_hw_params_set_channels, handle, params, channels);
    snd_call(snd_pcm_hw_params_set_rate, handle, params, sample_rate, 0);

    snd_call(snd_pcm_hw_params_set_period_size_near, handle, params,
             &pcm_period_size, nullptr);
}
void audio::init_sound_device(){
    snd_call(snd_pcm_hw_params, handle, params);
    snd_call(snd_pcm_prepare, handle);
}
void audio::dump_handle(){
    snd_pcm_close(pcm_handle);
} 
void audio::dump_params(){
    snd_pcm_hw_params_free(params);
} 

#endif
