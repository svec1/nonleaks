#ifndef CODER_AUDIO
#define CODER_AUDIO

#include <opus/opus.h>

#include "base_audio.hpp"
#include "utils.hpp"

template <audio_config _cfg>
class coder_audio : public noheap::pseudoheap_monotonic_array<102'400> {
  public:
    static constexpr audio_config cfg = _cfg;

    static constexpr std::size_t noencode_buffer_size = cfg.buffer_size;
    static constexpr std::size_t encode_buffer_size =
        cfg.bitrate / 8 * cfg.latency / 1000;

  public:
    using noencode_buffer_type =
        noheap::buffer_bytes_type<noencode_buffer_size,
                                  audio_config::byte_type>;
    using encode_buffer_type =
        noheap::buffer_bytes_type<encode_buffer_size, audio_config::byte_type>;

  public:
    coder_audio();

  public:
    encode_buffer_type encode(const noencode_buffer_type &buffer);
    noencode_buffer_type decode(const encode_buffer_type &buffer, bool lost);

    std::size_t get_bitrate() const;

  private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("CODER_AUDIO");
    static constexpr log_handler log{buffer_owner};

  private:
    OpusEncoder *enc;
    OpusDecoder *dec;
};

template <audio_config _cfg> coder_audio<_cfg>::coder_audio() {
    std::ssize_t error;

    enc = this->malloc<OpusEncoder *>(opus_encoder_get_size(cfg.channels));
    dec = this->malloc<OpusDecoder *>(opus_decoder_get_size(cfg.channels));

    error = opus_encoder_init(enc, cfg.sample_rate, cfg.channels,
                              OPUS_APPLICATION_VOIP);
    if (error)
        throw noheap::runtime_error(buffer_owner, "Failed to init encoder.");

    error = opus_decoder_init(dec, cfg.sample_rate, cfg.channels);
    if (error)
        throw noheap::runtime_error(buffer_owner, "Failed to init decoder.");

    opus_encoder_ctl(enc, OPUS_SET_BITRATE(cfg.bitrate));
    opus_decoder_ctl(dec, OPUS_SET_BITRATE(cfg.bitrate));
}

template <audio_config _cfg>
coder_audio<_cfg>::encode_buffer_type
coder_audio<_cfg>::encode(const noencode_buffer_type &buffer) {
    encode_buffer_type buffer_tmp{};
    std::ssize_t count_frames = opus_encode(
        enc, reinterpret_cast<const opus_int16 *>(buffer.data()),
        cfg.period_size, reinterpret_cast<std::uint8_t *>(buffer_tmp.data()),
        encode_buffer_size);

    if (count_frames == -1)
        throw noheap::runtime_error(buffer_owner,
                                    "Failed to encode buffer of samples.");

    return buffer_tmp;
}
template <audio_config _cfg>
coder_audio<_cfg>::noencode_buffer_type
coder_audio<_cfg>::decode(const encode_buffer_type &buffer, bool lost) {
    noencode_buffer_type buffer_tmp{};
    std::ssize_t count_frames = opus_decode(
        dec,
        lost ? NULL : reinterpret_cast<const std::uint8_t *>(buffer.data()),
        encode_buffer_size, reinterpret_cast<opus_int16 *>(buffer_tmp.data()),
        cfg.period_size, 0);

    if (count_frames == -1)
        throw noheap::runtime_error(buffer_owner,
                                    "Failed to decode buffer of samples.");

    return buffer_tmp;
}

#endif
