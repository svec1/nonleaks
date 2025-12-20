#ifndef NETBSD_AUDIO_HPP
#define NETBSD_AUDIO_HPP

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
};

audio::audio(audio_stream_mode _mode) { init(_mode); }
audio::~audio() { dump(); }
void audio::init_params(){
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
    
    if(ioctl(handle, AUDIO_SETINFO, &ap) == -1)
        throw_error<audio_stream_error::failed_set_params>();
    
    if(ioctl(handle, AUDIO_GETINFO, &ap) == -1)
        throw_error<audio_stream_error::failed_get_params>();
}

#endif
