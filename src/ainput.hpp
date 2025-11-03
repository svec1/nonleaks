#ifndef AINPUT_HPP
#define AINPUT_HPP

#include <aaudio.hpp>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

class input : private audio {
   public:
    input();

   public:
    std::vector<byte> get_samples();
};

input::input() : audio(SND_PCM_STREAM_CAPTURE) {}

std::vector<byte> input::get_samples() {
    std::vector<byte> bytes(buffer_size);
    int ret;

    while ((ret = snd_pcm_readi(pcm_handle, bytes.data(), period_size)) < 0)
        snd_call(snd_pcm_prepare, pcm_handle);

    return bytes;
}
#endif
