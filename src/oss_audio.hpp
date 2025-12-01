#ifndef OSS_AUDIO_HPP
#define OSS_AUDIO_HPP

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>

#include <base_audio.hpp>

class audio : public base_audio<int, void> {
    public:
	audio(audio_stream_t _mode);
    
    public:
	void read(char* buffer) override;
	void write(char* buffer) override;
    
    public:
	void init_handle() override;
	void init_params() override;
	void init_sound_device() override;

	void dump_handle() override;
	void dump_params() override;
};

audio::audio(audio_stream_t _mode) {
    init(_mode);
}
void audio::read(char* buffer) {
    std::memset(buffer, 0, buffer_size);
    if(::read(*handle, buffer, buffer_size) == -1)
	    throw std::runtime_error("Error reading audio.");
}
void audio::write(char* buffer) {
    if(::write(*handle, buffer, buffer_size) == -1)
	    throw std::runtime_error("Error writing audio.");
} 

void audio::init_handle(){
    static int dsp_st = open(device_playback.data(), O_RDWR);
    handle = &dsp_st;

    if(!dsp_st)
	    throw std::runtime_error("Failed to open audio device.");
}

void audio::init_params(){
    unsigned int cur_param = AFMT_S16_LE;
    if(ioctl(*handle, SNDCTL_DSP_SETFMT, &cur_param) == -1 
		|| cur_param != AFMT_S16_LE)
	throw std::runtime_error("Failed to set the format audio.");	
   
    cur_param = channels;
    if(ioctl(*handle, SNDCTL_DSP_CHANNELS, &cur_param) == -1
		|| cur_param != channels)    
	throw std::runtime_error("Failed to set the count channels for audio.");	

    cur_param = sample_rate;
    if(ioctl(*handle, SNDCTL_DSP_SPEED, &cur_param) == -1 
		|| cur_param != sample_rate)
	throw std::runtime_error("Failed to set the format audio.");
}

void audio::init_sound_device(){
    
}
void audio::dump_handle(){
    close(*handle);
}
void audio::dump_params(){
}

#endif
