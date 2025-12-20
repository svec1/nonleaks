#ifndef OPENBSD_AUDIO_HPP
#define OPENBSD_AUDIO_HPP

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/audioio.h>

#include <sio_base_audio.hpp>

class audio : public sio_base_audio {
    public:
	audio(audio_stream_mode _mode);
	~audio() override;   
 
    protected:
	void init_params() override;
    	void start_audio() override;
    	void stop_audio() override;
};

audio::audio(audio_stream_mode _mode) { init(_mode); }
audio::~audio() { dump(); }
void audio::init_params(){
    audio_swpar ap; 

    AUDIO_INITPAR(&ap);

    ap.sig = 1;
    ap.le = 1;
    ap.bits = bits_per_sample;
    ap.bps = bits_per_sample/8;
    ap.msb = 0; 
    ap.rate = sample_rate;
    ap.pchan = channels;
    ap.rchan = channels;
    ap.nblks = 4;
    ap.round = buffer_size/4;

    message("{}, {}", buffer_size, ap.round);
    if(ioctl(handle, AUDIO_SETPAR, &ap) == -1)
        throw_error<audio_stream_error::failed_set_params>();
    
    if(ioctl(handle, AUDIO_GETPAR, &ap) == -1)
        throw_error<audio_stream_error::failed_get_params>();

    message("{}, {}", ap.nblks, ap.round);

}
void audio::start_audio() {
    sio_base_audio::start_audio();
    
    audio_status status;
    if(ioctl(handle, AUDIO_GETSTATUS, &status) == -1)
        throw_error<audio_stream_error::failed_get_status>();

    if(!status.pause)
	return;

    if(ioctl(handle, AUDIO_START) == -1)
        throw_error<audio_stream_error::failed_start>();
}
void audio::stop_audio(){
    sio_base_audio::stop_audio();
    
    audio_status status;
    if(ioctl(handle, AUDIO_GETSTATUS, &status) == -1)
        throw_error<audio_stream_error::failed_get_status>();

    if(status.pause)
	return;

    if(ioctl(handle, AUDIO_STOP) == -1)
        throw_error<audio_stream_error::failed_stop>();
}
#endif
