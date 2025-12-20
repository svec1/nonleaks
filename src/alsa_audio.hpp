#ifndef AAUDIO_HPP
#define AAUDIO_HPP

#include <alsa/asoundlib.h>

#include <base_audio.hpp>

class audio : public base_audio<snd_pcm_t*, snd_pcm_hw_params_t*> {
   protected:
    static constexpr snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    static constexpr snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
   
   public:
    audio(audio_stream_mode _mode);
    ~audio() override;

   private:
    void init_handle() override;
    void init_params() override;
    void init_sound_device() override;
    void dump_handle() override;
    
    void start_audio() override;
    void stop_audio() override;
   
   protected:
    void pread(char* buffer) override;
    void pwrite(const char* buffer) override;
   
   private:
    template <typename T, typename... Args>
    void throw_if(T&& func, Args&&... args) {
	static int ret;
        if (ret = func(args...); ret < 0)
	    throw_error("{}", snd_strerror(ret));
    }
};
audio::audio(audio_stream_mode _mode) { init(_mode); }
audio::~audio() {
    dump();
    snd_pcm_hw_params_free(params);
}
void audio::pread(char* buffer) {
    while (snd_pcm_readi(handle, buffer, period_size) < 0)
        throw_if(snd_pcm_recover, handle, period_size, 0);
}
void audio::pwrite(const char* buffer) {
    while (snd_pcm_writei(handle, buffer, period_size) < 0)
        throw_if(snd_pcm_prepare, handle);
}
void audio::init_handle() {
    switch (mode) {
	default:
        case audio_stream_mode::playback:
            throw_if(snd_pcm_open, &handle, device_playback.data(),
                     SND_PCM_STREAM_PLAYBACK, 0);
            break;
        case audio_stream_mode::capture:
            throw_if(snd_pcm_open, &handle, device_capture.data(),
                     SND_PCM_STREAM_CAPTURE, 0);
            break;
        case audio_stream_mode::bidirect:
	    throw_error("Bidirect mode is not supported.");
    }

    handle_initialized = handle > 0;
}
void audio::init_params() {
    static constexpr snd_pcm_uframes_t pcm_buffer_size = buffer_size*multiplier_for_hardware_buffer;
    static constexpr snd_pcm_uframes_t pcm_period_size = period_size*multiplier_for_hardware_buffer;

    throw_if(snd_pcm_hw_params_malloc, &params);
    throw_if(snd_pcm_hw_params_any, handle, params);

    throw_if(snd_pcm_hw_params_set_access, handle, params, access);
    throw_if(snd_pcm_hw_params_set_format, handle, params, format);
    throw_if(snd_pcm_hw_params_set_channels, handle, params, channels);
    throw_if(snd_pcm_hw_params_set_rate, handle, params, sample_rate, 0);

    throw_if(snd_pcm_hw_params_set_buffer_size_near, handle, params,
             &pcm_buffer_size);
    throw_if(snd_pcm_hw_params_set_period_size_near, handle, params,
             &pcm_period_size, nullptr);
}
void audio::init_sound_device() {
    throw_if(snd_pcm_hw_params, handle, params);
    throw_if(snd_pcm_prepare, handle);
}
void audio::dump_handle() {
    throw_if(snd_pcm_close, handle);
}
void audio::start_audio() {
    throw_if(snd_pcm_start, handle);
}
void audio::audio::stop_audio(){
    throw_if(snd_pcm_drain, handle);
    throw_if(snd_pcm_stop, handle);
} 

#endif
