#ifndef SIO_BASE_AUDIO_HPP
#define SIO_BASE_AUDIO_HPP

#include <errno.h>
#include <fcntl.h>
#include <sys/poll.h>

#include "base_audio.hpp"

template <audio_config _cfg> class sio_base_audio : public base_audio<_cfg> {
  public:
    sio_base_audio(audio_stream_mode _mode);
    virtual ~sio_base_audio() override = default;

  protected:
    void pread(sio_base_audio::buffer_type::value_type *buffer) override;
    void pwrite(const sio_base_audio::buffer_type::value_type *buffer) override;

  protected:
    virtual void init_params() = 0;

  private:
    int handle;
    pollfd pfd;
};

template <audio_config _cfg>
sio_base_audio<_cfg>::sio_base_audio(audio_stream_mode _mode)
    : base_audio<_cfg>(_mode) {
    if (this->possible_bidirect_stream) {
        static int handle_st = ::open(this->device_playback.data(), O_RDWR);
        this->handle = handle_st;
    } else
        switch (this->mode) {
        default:
        case audio_stream_mode::playback:
            this->handle = ::open(this->device_playback.data(), O_RDONLY);
            break;
        case audio_stream_mode::capture:
            this->handle = ::open(this->device_capture.data(), O_WRONLY);
            break;
        case audio_stream_mode::bidirect:
            this->handle = ::open(this->device_playback.data(), O_RDWR);
            break;
        }

    pfd.fd = handle;
    pfd.events = POLLIN | POLLOUT;
    pfd.revents = 0;

    init_params();
}
template <audio_config _cfg>
void sio_base_audio<_cfg>::pread(
    sio_base_audio::buffer_type::value_type *buffer) {
    if (poll(&pfd, 1, -1) == -1)
        sio_base_audio::template throw_error<
            sio_base_audio::audio_stream_error::architectural_feature>(
            "Error call of poll.");
    else if (!(pfd.revents & POLLIN)) {
        this->message("Audio stream is empty.");
        return;
    }

    if (::read(handle, buffer, this->cfg.buffer_size) == -1)
        sio_base_audio::template throw_error<
            sio_base_audio::audio_stream_error::error_reading>();
}
template <audio_config _cfg>
void sio_base_audio<_cfg>::pwrite(
    const sio_base_audio::buffer_type::value_type *buffer) {
    if (poll(&pfd, 1, -1) == -1)
        sio_base_audio::template throw_error<
            sio_base_audio::audio_stream_error::architectural_feature>(
            "Error call of poll.");
    else if (!(pfd.revents & POLLOUT)) {
        this->message("Audio stream is overheap.");
        return;
    }

    if (::write(handle, buffer, this->cfg.buffer_size) == -1)
        sio_base_audio::template throw_error<
            sio_base_audio::audio_stream_error::error_writing>();
}

#endif
