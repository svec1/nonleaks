#ifndef OSS_AUDIO_HPP
#define OSS_AUDIO_HPP

#include <sys/ioctl.h>
#include <sys/soundcard.h>

#include <sio_base_audio.hpp>

class audio : public sio_base_audio {
    public:
	audio(audio_stream_mode _mode);
	~audio() override;   
 
    protected:
	void init_params() override;
};

audio::audio(audio_stream_t _mode) { init(_mode); }
audio::~audio() { dump(); }
void audio::init_params(){
    std::size_t cur_param = AFMT_S16_LE;
    if(ioctl(handle, SNDCTL_DSP_SETFMT, &cur_param) == -1 
		|| cur_param != AFMT_S16_LE)
        throw_error<audio_stream_error::failed_set_params>("Failed to set the format audio.");
   
    cur_param = channels;
    if(ioctl(handle, SNDCTL_DSP_CHANNELS, &cur_param) == -1
		|| cur_param != channels)    
        throw_error<audio_stream_error::failed_set_params>("Failed to set the count channels for audio.");

    cur_param = sample_rate;
    if(ioctl(handle, SNDCTL_DSP_SPEED, &cur_param) == -1 
		|| cur_param != sample_rate)
	throw_error<audio_stream_error::failed_set_params>();
}
#endif
