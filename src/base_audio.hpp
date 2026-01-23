#ifndef BASE_AUDIO_HPP
#define BASE_AUDIO_HPP

#include "utils.hpp"

#if defined(ALSA) || defined(__linux__)
#define ARSND_NAME_D "ALSA"
#elif defined(SNDIO)
#define ARSND_NAME_D "SNDIO"
#elif defined(FREEBSD_AUDIO)
#define ARSND_NAME_D "FREEBSD_AUDIO"
#elif defined(OPENBSD_AUDIO)
#define ARSND_NAME_D "OPENBSD_AUDIO"
#elif defined(NETBSD_AUDIO)
#define ARSND_NAME_D "NETBSD_AUDIO"
#else
#error "Audio architecture is not defined."
#endif

enum audio_stream_mode : std::size_t { playback = 0, capture, bidirect };

static constexpr std::string_view
audio_stream_mode_to_string(audio_stream_mode mode) {
    switch (mode) {
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

struct audio_config {
    using byte_type = std::int8_t;

  public:
    std::size_t bitrate;
    std::size_t latency;
    std::size_t channels;
    std::size_t sample_rate;
    std::size_t bits_per_sample;
    std::size_t bytes_per_sample =
        bits_per_sample % 8 ? bits_per_sample / 8 : (bits_per_sample + 1) / 8;
    std::size_t period_size = sample_rate * latency / 1000;
    std::size_t buffer_size = period_size * channels * bytes_per_sample;
    std::size_t diviser_for_hardware_buffer = 4;
};

template <audio_config _cfg> class base_audio {
  public:
    static constexpr audio_config cfg = _cfg;
    static constexpr std::string_view arsnd_name = ARSND_NAME_D;
    static constexpr std::string_view default_device_playback = "default";
    static constexpr std::string_view default_device_capture = "default";

  public:
    using buffer_type =
        noheap::buffer_bytes_type<cfg.buffer_size, audio_config::byte_type>;

  protected:
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

  protected:
    base_audio(audio_stream_mode mode);
    virtual ~base_audio() = default;

  protected:
    void read(buffer_type &buffer);
    void write(const buffer_type &buffer);

    void set_mute(bool _mute_writing, bool _mute_reading);

  protected:
    virtual void pread(buffer_type::value_type *buffer) = 0;
    virtual void pwrite(const buffer_type::value_type *buffer) = 0;

  protected:
    static consteval std::string_view
    audio_stream_error_to_string(audio_stream_error error_number);

    template <typename... Args>
    constexpr void message(std::format_string<Args...> format, Args &&...args);

    template <audio_stream_error error_number, bool on_errno = false,
              typename... Args>
    static constexpr void throw_error(std::format_string<Args...> format = "",
                                      Args &&...args);

  public:
    static std::string_view device_playback;
    static std::string_view device_capture;

  protected:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner(arsnd_name);
    static constexpr log_handler log{buffer_owner};

  protected:
    audio_stream_mode mode;
    bool possible_bidirect_stream;

  private:
    bool mute_writing = false;
    bool mute_reading = false;
};

template <audio_config _cfg>
base_audio<_cfg>::base_audio(audio_stream_mode _mode) {
    try {
        mode = _mode;
        possible_bidirect_stream = (device_playback == device_capture);

        if (mode == audio_stream_mode::bidirect && !possible_bidirect_stream)
            throw_error<audio_stream_error::architectural_feature>(
                "For bidirectional, different device names were specified.");

    } catch (noheap::runtime_error &excp) {
        throw_error<audio_stream_error::failed_open>(
            "{} {}: {}", audio_stream_mode_to_string(mode),
            (!(bool)mode ? device_playback : device_capture), excp.what());
    }
}
template <audio_config _cfg> void base_audio<_cfg>::read(buffer_type &buffer) {
    if (mode == audio_stream_mode::playback)
        throw_error<audio_stream_error::error_reading>(
            "The reading cannot be played back to the {} stream.",
            audio_stream_mode_to_string(mode));

    if (!mute_reading)
        pread(buffer.data());
}
template <audio_config _cfg>
void base_audio<_cfg>::write(const buffer_type &buffer) {
    if (mode == audio_stream_mode::capture)
        throw_error<audio_stream_error::error_writing>(
            "The writing cannot be played back to the {} stream.",
            audio_stream_mode_to_string(mode));

    if (!mute_writing)
        pwrite(buffer.data());
}
template <audio_config _cfg>
void base_audio<_cfg>::set_mute(bool _mute_writing, bool _mute_reading) {
    mute_writing = _mute_writing;
    mute_reading = _mute_reading;
}

template <audio_config _cfg>
consteval std::string_view base_audio<_cfg>::audio_stream_error_to_string(
    audio_stream_error error_number) {
    switch (error_number) {
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
template <audio_config _cfg>
template <typename... Args>
constexpr void base_audio<_cfg>::message(std::format_string<Args...> format,
                                         Args &&...args) {
    log.to_console(format, std::forward<Args>(args)...);
}
template <audio_config _cfg>
template <base_audio<_cfg>::audio_stream_error error_number, bool on_errno,
          typename... Args>
constexpr void base_audio<_cfg>::throw_error(std::format_string<Args...> format,
                                             Args &&...args) {
    noheap::runtime_error::buffer_type buffer{}, buffer_format{};

    auto end_it = buffer.begin();
    if constexpr (error_number != audio_stream_error::null) {
        end_it =
            std::format_to_n(end_it, noheap::runtime_error::buffer_size, "{}",
                             audio_stream_error_to_string(error_number))
                .out;
        if (!format.get().empty()) {
            std::format_to_n(buffer_format.begin(),
                             std::abs(std::distance(buffer.end(), end_it)),
                             format, std::forward<Args>(args)...);
            end_it = std::format_to_n(
                         end_it, std::abs(std::distance(buffer.end(), end_it)),
                         "\n   | {}", buffer_format.data())
                         .out;
        }
    }

    if (on_errno && errno) {
        end_it = std::format_to_n(end_it, noheap::runtime_error::buffer_size,
                                  ".errno = ")
                     .out;
        strerror_r(errno, end_it,
                   std::abs(std::distance(buffer.end(), end_it)));
        errno = 0;
    }

    noheap::runtime_error error(std::move(buffer));
    error.set_owner(buffer_owner);
    throw std::move(error);
}

template <audio_config _cfg>
std::string_view base_audio<_cfg>::device_playback =
    base_audio<_cfg>::default_device_playback;
template <audio_config _cfg>
std::string_view base_audio<_cfg>::device_capture =
    base_audio<_cfg>::default_device_capture;

#if defined(ALSA) || defined(__linux__)
#include "alsa_audio.hpp"
#elif defined(SNDIO)
#include "sndio_audio.hpp"
#elif defined(FREEBSD_AUDIO)
#include "oss_audio.hpp"
#elif defined(OPENBSD_AUDIO)
#include "openbsd_audio.hpp"
#elif defined(NETBSD_AUDIO)
#include "netbsd_audio.hpp"
#endif

#endif
