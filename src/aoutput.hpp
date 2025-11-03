#ifndef AOUTPUT_HPP
#define AOUTPUT_HPP

#include <aaudio.hpp>
#include <cmath>
#include <vector>

class output : private audio {
   public:
    output();

   public:
    void play_samples(const std::vector<byte>& bytes);
};

output::output() : audio(SND_PCM_STREAM_PLAYBACK) {}

void output::play_samples(const std::vector<byte>& bytes) {
    int ret, writed_bytes = 0;

    while (writed_bytes < bytes.size()) {
        while ((ret = snd_pcm_writei(pcm_handle, bytes.data() + writed_bytes,
                                     period_size)) < 0)
            snd_call(snd_pcm_prepare, pcm_handle);
        writed_bytes += period_size * channels * 2;
    }
}
#endif
