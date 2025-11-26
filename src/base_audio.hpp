#ifndef BASE_AUDIO_HPP
#define BASE_AUDIO_HPP

#include <format>
#include <exception>

enum audio_stream_t{
	playback = 0,
	capture
};

template<typename THandle, typename TParams>
class base_audio{
    public:
     virtual ~base_audio() = default;
   
     void init(audio_stream_t _mode);
     void dump();

    public:
     virtual void init_handle() = 0;
     virtual void init_params() = 0;
     virtual void init_sound_device() = 0;

     virtual void dump_handle() = 0;
     virtual void dump_params() = 0;

    public:
     virtual void read(char* buffer, unsigned int period_size) = 0;
     virtual void write(char* buffer, unsigned int period_size) = 0;

    public:
     static constexpr std::string_view default_device_playback = "default";
     static constexpr std::string_view default_device_capture = "default";

     static constexpr unsigned int bits_per_sample = 16;
     static constexpr unsigned int sample_rate = 44100;
     static constexpr unsigned int channels    = 2;
     static constexpr unsigned int period_size = 512;
     static constexpr unsigned int buffer_size = period_size * channels * bits_per_sample/8;

     static std::string_view device_playback;
     static std::string_view device_capture;
   
    protected:
     THandle* handle;
     audio_stream_t mode;

     static TParams* params;
    private:
     static unsigned int counter_astream;
};

template<typename THandle, typename TParams>
void base_audio<THandle, TParams>::init(audio_stream_t _mode){
    try{
	mode = _mode;
	init_handle();
    	if(!params) init_params();
        init_sound_device();
        ++counter_astream;
    }
    catch(std::runtime_error &excp){
        throw std::runtime_error(
            std::format("Failed to open the stream({}:{}): {}",
                        (!(int)mode ? device_playback : device_capture).data(),
                        !(int)mode ? "PLAYBACK" : "CAPTURE", excp.what()));

    }
}
template<typename THandle, typename TParams>
void base_audio<THandle, TParams>::dump(){
    dump_handle();	
    if(!counter_astream--) dump_params();	
}


template<typename THandle, typename TParams>
TParams* base_audio<THandle, TParams>::params;
template<typename THandle, typename TParams>
unsigned int base_audio<THandle, TParams>::counter_astream;

template<typename THandle, typename TParams>
std::string_view base_audio<THandle, TParams>::device_playback = base_audio<THandle, TParams>::default_device_playback;
template<typename THandle, typename TParams>
std::string_view base_audio<THandle, TParams>::device_capture = base_audio<THandle, TParams>::default_device_capture;

#endif
