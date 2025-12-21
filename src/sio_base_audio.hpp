#ifndef SIO_BASE_AUDIO_HPP
#define SIO_BASE_AUDIO_HPP

#include <errno.h>
#include <fcntl.h>
#include <sys/poll.h>

#include "base_audio.hpp"

class sio_base_audio : public base_audio<int, int> {
  public:
    sio_base_audio();
    virtual ~sio_base_audio() override = default;

  protected:
    void pread(char *buffer) override;
    void pwrite(const char *buffer) override;

  protected:
    void init_handle() override;
    void init_sound_device() override;
    void dump_handle() override;

  private:
    pollfd pfd;
};

sio_base_audio::sio_base_audio() {}
void sio_base_audio::pread(char *buffer) {
    if (poll(&pfd, 1, -1) == -1)
        throw_error<audio_stream_error::architectural_feature>(
            "Error call of poll.");
    else if (!(pfd.revents & POLLIN)) {
        message("Audio stream is empty.");
        return;
    }

    if (::read(handle, buffer, buffer_size) == -1)
        throw_error<audio_stream_error::error_reading>();
}
void sio_base_audio::pwrite(const char *buffer) {
    if (poll(&pfd, 1, -1) == -1)
        throw_error<audio_stream_error::architectural_feature>(
            "Error call of poll.");
    else if (!(pfd.revents & POLLOUT)) {
        message("Audio stream is overheap.");
        return;
    }

    if (::write(handle, buffer, buffer_size) == -1)
        throw_error<audio_stream_error::error_writing>();
}
void sio_base_audio::init_handle() {
    if (possible_bidirect_stream) {
        static handle_t handle_st = ::open(device_playback.data(), O_RDWR);
        handle = handle_st;
    } else
        switch (mode) {
        default:
        case audio_stream_mode::playback:
            handle = ::open(device_playback.data(), O_RDONLY);
            break;
        case audio_stream_mode::capture:
            handle = ::open(device_capture.data(), O_WRONLY);
            break;
        case audio_stream_mode::bidirect:
            handle = ::open(device_playback.data(), O_RDWR);
            break;
        }

    pfd.fd = handle;
    pfd.events = POLLIN | POLLOUT;
    pfd.revents = 0;

    handle_initialized = handle > 0;
}
void sio_base_audio::init_sound_device() {
    if (!params_initialized || possible_bidirect_stream)
        return;

    init_params();
}
void sio_base_audio::dump_handle() { ::close(handle); }

#endif
