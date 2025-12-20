#ifndef BASE_AUDIO_HPP
#define BASE_AUDIO_HPP

#include <utils.hpp>

#if defined(ALSA)
#define ARSND_NAME_D "ALSA"
#elif defined(SNDIO)
#define ARSND_NAME_D "SNDIO"
#elif defined(FREEBSD_AUDIO)
#define ARSND_NAME_D "FREEBSD_AUDIO"
#elif defined(OPENBSD_AUDIO)
#define ARSND_NAME_D "OPENBSD_AUDIO"
#elif defined(NETBSD_AUDIO)
#define ARSND_NAME_D "NETBSD_AUDIO"
#endif

enum audio_stream_mode : std::size_t { playback = 0, capture, bidirect };
enum audio_stream_error : std::size_t {
    null = 0,
    failed_open = 1,
    failed_start,
    failed_stop,
    failed_close,
    failed_set_params,
    failed_get_params,
    failed_get_status,
    error_writing,
    error_reading,
    architectural_feature,
};

static constexpr std::string_view audio_stream_mode_to_string(audio_stream_mode mode){
    switch(mode){
	case audio_stream_mode::playback:
	    return "PLAYBACK";
	case audio_stream_mode::capture:
	    return "CAPTURE";
	case audio_stream_mode::bidirect:
	    return "BIDIRECT";
	default:
	    return "UNDEFINED";
    }
}

static consteval std::string_view audio_stream_error_to_string(audio_stream_error error_number){
    switch(error_number){
	case audio_stream_error::failed_open:
	    return "Failed to open the stream.";
	case audio_stream_error::failed_start:
	    return "Failed to start the stream.";
	case audio_stream_error::failed_stop:
	    return "Failed to stop the stream.";
	case audio_stream_error::failed_close:
	    return "Failed to close the stream.";
	case audio_stream_error::failed_set_params:
	    return "Failed to set parameters of the stream.";
	case audio_stream_error::failed_get_params:
	    return "Failed to get parameters of the stream.";
	case audio_stream_error::failed_get_status:
	    return "Failed to get status of the stream.";
	case audio_stream_error::error_writing:
	    return "Error writing audio.";
	case audio_stream_error::error_reading:
	    return "Error writing audio.";
	case audio_stream_error::architectural_feature:
	    return "Architecture error.";
	default:
	    return "Undefined error.";
    }
}

template <typename THandle, typename TParams>
class base_audio {
   public:
    static constexpr std::string_view arsnd_name = ARSND_NAME_D;
    static constexpr std::string_view default_device_playback = "default";
    static constexpr std::string_view default_device_capture = "default";

    static constexpr float latency = 0.01;
    static constexpr std::size_t bits_per_sample = 16;
    static constexpr std::size_t sample_rate = 44100;
    static constexpr std::size_t channels = 2;
    static constexpr std::size_t period_size = sample_rate * latency;
    static constexpr std::size_t buffer_size = period_size * channels * bits_per_sample / 8;
    static constexpr std::size_t diviser_for_hardware_buffer = 4;

    static constexpr noheap::log_impl::owner_impl::buffer_t buffer_owner =
	noheap::log_impl::create_owner(arsnd_name);
    static constexpr log_handler log{buffer_owner};
    
    using buffer_t = noheap::buffer_t<buffer_size>;
    using handle_t = THandle;
    using params_t = TParams;
	
   public:
    base_audio() = default;
    virtual ~base_audio() = default;

   public:
    constexpr void read(buffer_t& buffer);
    constexpr void write(const buffer_t& buffer);

   protected:
    void init(audio_stream_mode _mode);
    void dump();
    
   protected:
    virtual void init_handle() = 0;
    virtual void init_params() = 0;
    virtual void init_sound_device() = 0;
    virtual void dump_handle() = 0;
    
    virtual void start_audio();
    virtual void stop_audio();
   
   protected:
    virtual void pread(char* buffer) = 0;
    virtual void pwrite(const char* buffer) = 0;

   protected:
    template<typename... Args>
    constexpr void message(std::format_string<Args...> format, Args&&... args){
	log.to_console(format, std::forward<Args>(args)...);
    }
    template<audio_stream_error error_number, bool on_errno = true, typename... Args>
    constexpr void throw_error(std::format_string<Args...> format = "", Args&&... args){
	noheap::runtime_error::buffer_t buffer, buffer_format;

	auto end_it = buffer.begin();
    	std::fill_n(end_it, buffer.size(), 0);
    	std::fill_n(buffer_format.begin(), buffer_format.size(), 0);
	
	if constexpr(error_number != audio_stream_error::null){	
	    end_it = std::format_to_n(end_it, noheap::runtime_error::buffer_size, "{}", audio_stream_error_to_string(error_number)).out;
	    if(!format.get().empty()){
	    	std::format_to_n(buffer_format.begin(), std::abs(std::distance(buffer.end(), end_it)), format, std::forward<Args>(args)...);
	    	end_it = std::format_to_n(end_it, std::abs(std::distance(buffer.end(), end_it)), "\n<{}>", buffer_format.data()).out;
	    }
	}
	
	if (on_errno && errno){
	    end_it = std::format_to_n(end_it, noheap::runtime_error::buffer_size, ".errno = ").out;
	    strerror_r(errno, end_it, std::abs(std::distance(buffer.end(), end_it)));
     	    errno = 0;
	}
	
	noheap::runtime_error error(std::move(buffer));
	error.set_owner(buffer_owner);
	throw std::move(error);
    }

   public:
    static std::string_view device_playback;
    static std::string_view device_capture;
	
   protected:
    THandle handle;
    static TParams params;
    
    audio_stream_mode mode;
    bool possible_bidirect_stream,
	 handle_initialized = false,
	 params_initialized = false,
	 mute_writing = false,
	 mute_reading = false;
};

template <typename THandle, typename TParams>
void base_audio<THandle, TParams>::init(audio_stream_mode _mode) {
    try {
	mode = _mode;
	possible_bidirect_stream = (device_playback == device_capture);

 	if(mode == audio_stream_mode::bidirect && !possible_bidirect_stream)
	   throw_error<audio_stream_error::architectural_feature>("For bidirectional, different device names were specified."); 
        
	init_handle();
	if (!handle_initialized) throw_error<audio_stream_error::null, true>();
	if (!params_initialized){ 
	    init_params();
	    params_initialized = true;
	}
        init_sound_device();
	start_audio();
    } catch (noheap::runtime_error& excp) {
	throw_error<audio_stream_error::failed_open>("{} {}: {}",
                        audio_stream_mode_to_string(mode),
                        (!(bool)mode ? device_playback : device_capture),
			excp.what());
    }
}
template <typename THandle, typename TParams>
void base_audio<THandle, TParams>::dump() {
    stop_audio();
    dump_handle();
}
template <typename THandle, typename TParams>
constexpr void base_audio<THandle, TParams>::read(buffer_t& buffer){
    if(mode == audio_stream_mode::playback)
	throw_error<audio_stream_error::architectural_feature>("The reading cannot be played back to the {} stream.", audio_stream_mode_to_string(mode));
    std::fill_n(buffer.begin(), buffer_size, 0);

    if(!mute_reading) pread(buffer.data());
}
template <typename THandle, typename TParams>
constexpr void base_audio<THandle, TParams>::write(const buffer_t& buffer){
    if(mode == audio_stream_mode::capture)
	throw_error<audio_stream_error::architectural_feature>("The writing cannot be played back to the {} stream.", audio_stream_mode_to_string(mode));
    
    if(!mute_writing) pwrite(buffer.data());
}
template <typename THandle, typename TParams>
void base_audio<THandle, TParams>::start_audio() {
    mute_writing = false;
    mute_reading = false;
}
template <typename THandle, typename TParams>
void base_audio<THandle, TParams>::stop_audio(){
    mute_writing = true;
    mute_reading = true;
}
template <typename THandle, typename TParams>
TParams base_audio<THandle, TParams>::params;

template <typename THandle, typename TParams>
std::string_view base_audio<THandle, TParams>::device_playback =
    base_audio<THandle, TParams>::default_device_playback;
template <typename THandle, typename TParams>
std::string_view base_audio<THandle, TParams>::device_capture =
    base_audio<THandle, TParams>::default_device_capture;

#if defined(ALSA)
#include <alsa_audio.hpp>
#elif defined(SNDIO)
#include <sndio_audio.hpp>
#elif defined(FREEBSD_AUDIO)
#include <oss_audio.hpp>
#elif defined(OPENBSD_AUDIO)
#include <openbsd_audio.hpp>
#elif defined(NETBSD_AUDIO)
#include <netbsd_audio.hpp>
#endif

#endif
