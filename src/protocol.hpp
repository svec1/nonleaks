#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include "audio_flow.hpp"
#include "net.hpp"
#include "noise.hpp"

namespace protocol {

static constexpr std::size_t max_payload_data_size        = 512;
static constexpr std::size_t max_jitter_size              = 64;
static constexpr std::size_t min_count_packets_for_handle = max_jitter_size / 2;

struct extention_data_type {
    struct payload_data_type {
        static constexpr std::size_t  max_count_contributers = 4;
        static constexpr std::size_t  uuid_size              = 8;
        static constexpr std::uint8_t local_payload_value    = static_cast<int8_t>(
            (stream_audio::default_base_audio::cfg.sample_rate
             + stream_audio::default_base_audio::cfg.bitrate
             + (stream_audio::default_base_audio::cfg.channels * 1000) / 1000));

        using audio_buffer_type = stream_audio::encode_buffer_type;
        using uuid_type         = noheap::buffer_bytes_type<uuid_size, std::uint8_t>;
        using uuids_type        = std::array<uuid_type, max_count_contributers>;

    public:
        audio_buffer_type audio_buffer;

        uuids_type    uuids;
        std::uint16_t sequence_number;
        std::uint8_t  payload_type;
        bool          lost;
    };

public:
    static constexpr std::size_t payload_data_size = sizeof(payload_data_type);
    static_assert(payload_data_size <= max_payload_data_size,
                  "Payload size is too large.");

public:
    payload_data_type payload;

    noheap::buffer_bytes_type<max_payload_data_size - payload_data_size> padding;
};

using payload_size_ad_type           = std::size_t;
static constexpr std::size_t ad_size = sizeof(payload_size_ad_type);

template<ntn_relation relation_type>
using noise_context_type          = noise_context<relation_type>;
using payload_packet_type         = packet_native_type<extention_data_type, ad_size>;
using noise_handshake_packet_type = packet_native_type<
    noise_context_type<ntn_relation::PTU>::buffer_handshake_packet_type, ad_size>;

struct payload_protocol_type
    : public protocol_native_type<payload_packet_type,
                                  noheap::log_impl::create_owner("VOICE_PROTOCOL")> {
    using packet_type        = payload_protocol_type::packet_type;
    using noise_context_type = noise_context_type<ntn_relation::PTU>;
    using jitter_buffer_type = noheap::jitter_buffer<packet_type, max_jitter_size>;
    using uuid_type = packet_type::extention_data_type::payload_data_type::uuid_type;

public:
    constexpr void
        prepare(packet_type &pckt, buffer_address_type addr,
                payload_protocol_type::callback_prepare_type callback) const override {
        check_cipher_state();
        try {
            callback(pckt);

            pckt->payload.sequence_number = local_sequence_number++;
            pckt->payload.uuids[0]        = uuid;

            cipher_state->input_buffer.set(
                {reinterpret_cast<std::uint8_t *>(pckt.extention_data()),
                 pckt.extention_size()},
                packet_type::extention_data_type::payload_data_size);
            cipher_state->encrypt();

            pckt.payload_ad = noheap::to_buffer<packet_type::ad_type>(
                cipher_state->input_buffer.get().size);

        } catch (noheap::runtime_error &excp) {
            excp.set_owner(buffer_owner);
            throw;
        }
    }
    constexpr void
        handle(packet_type &pckt, buffer_address_type addr,
               payload_protocol_type::callback_handle_type callback) const override {
        check_cipher_state();
        try {
            cipher_state->output_buffer.set(
                {reinterpret_cast<std::uint8_t *>(pckt.extention_data()),
                 pckt.extention_size()},
                noheap::represent_bytes<payload_size_ad_type>(pckt.payload_ad));
            cipher_state->decrypt();

            buffer.push(pckt, pckt->payload.sequence_number);

            if (buffer.get_count_elements() == min_count_packets_for_handle)
                filled = true;
            else if (!buffer.get_count_elements())
                filled = false;

            if (filled) {
                auto jitter_element = buffer.pop();
                pckt                = jitter_element.first;
                pckt->payload.lost  = jitter_element.second;

                callback(std::move(jitter_element.first));
            }
        } catch (noheap::runtime_error &excp) {
            excp.set_owner(buffer_owner);
            throw;
        }
    }

public:
    void set_noise_cipher_state(noise_context_type::cipher_state &_cipher_state) const {
        cipher_state = &_cipher_state;
    }
    void set_local_sequence_number(std::uint16_t _local_sequence_number) const {
        local_sequence_number = _local_sequence_number;
    }
    void set_uuid(uuid_type &&_uuid) const { uuid = std::move(_uuid); }

public:
    float get_loss_per_cent() const {
        std::size_t count_pushed_packets = buffer.get_count_pushed_elements();
        std::size_t count_lost_packets   = buffer.get_count_lost_elements();

        float loss_per_cent =
            static_cast<float>(count_lost_packets - last_count_lost_packets)
            / (count_pushed_packets - last_count_pushed_packets);

        last_count_pushed_packets = count_pushed_packets;
        last_count_lost_packets   = count_lost_packets;

        return loss_per_cent;
    }

private:
    void check_cipher_state() const {
        if (cipher_state)
            return;

        throw noheap::runtime_error(buffer_owner, "Cipher state is null.");
    }

private:
    mutable jitter_buffer_type                buffer;
    mutable noise_context_type::cipher_state *cipher_state;

    mutable bool filled = false;

    mutable uuid_type     uuid;
    mutable std::uint32_t local_sequence_number;

    mutable std::size_t last_count_lost_packets   = 0;
    mutable std::size_t last_count_pushed_packets = 0;
};

template<ntn_relation relation_type>
struct noise_handshake_protocol_type
    : public protocol_native_type<noise_handshake_packet_type,
                                  noheap::log_impl::create_owner("NOISE_HANDSHAKE")> {
    using noise_context_type = noise_context_type<relation_type>;

public:
    constexpr void prepare(packet_type &pckt, buffer_address_type addr,
                           callback_prepare_type callback) const override {
        check_noise_action(noise_action::WRITE_MESSAGE);

        try {
            noise_ctx_p->get_handshake_buffer().set(
                std::span<std::uint8_t>(
                    reinterpret_cast<std::uint8_t *>(pckt.extention_data()),
                    pckt.extention_size()),
                0);
            noise_ctx_p->set_handshake_message();

            pckt.payload_ad = noheap::to_buffer<packet_type::ad_type>(
                noise_ctx_p->get_handshake_buffer().get().size);

            noheap::println("Send: {}",
                            std::span(pckt.extention_data(),
                                      noheap::represent_bytes<payload_size_ad_type>(
                                          pckt.payload_ad)));
        } catch (noheap::runtime_error &excp) {
            excp.set_owner(buffer_owner);
            throw;
        }
    }
    constexpr void handle(packet_type &pckt, buffer_address_type addr,
                          callback_handle_type callback) const override {
        check_noise_action(noise_action::READ_MESSAGE);
        noheap::println(
            "Receive: {}",
            std::span(pckt.extention_data(),
                      noheap::represent_bytes<payload_size_ad_type>(pckt.payload_ad)));

        try {
            noise_ctx_p->get_handshake_buffer().set(
                std::span<std::uint8_t>(
                    reinterpret_cast<std::uint8_t *>(pckt.extention_data()),
                    pckt.extention_size()),
                noheap::represent_bytes<payload_size_ad_type>(pckt.payload_ad));
            noise_ctx_p->get_handshake_message();
        } catch (noheap::runtime_error &excp) {
            excp.set_owner(buffer_owner);
            throw;
        }
    }

public:
    void set_noise_context(noise_context_type &noise_ctx) const {
        noise_ctx_p = &noise_ctx;
    }
    noise_context_type &get_noise_context() const {
        check_noise_ctx_p();
        return *noise_ctx_p;
    }

private:
    void check_noise_action(noise_action expected) const {
        check_noise_ctx_p();
        auto action = noise_ctx_p->get_action();
        if (action == noise_action::FAILED)
            throw noheap::runtime_error(buffer_owner, "Failed to handshake.");

        if (action == expected)
            return;

        if (action == noise_action::WRITE_MESSAGE)
            throw noheap::runtime_error(buffer_owner, "Expected message to be sent.");
        else if (action == noise_action::READ_MESSAGE)
            throw noheap::runtime_error(buffer_owner, "Expected message to be received.");
        else
            throw noheap::runtime_error(buffer_owner, "Handshake already completed.");
    }

    void check_noise_ctx_p() const {
        if (!noise_ctx_p)
            noheap::runtime_error(buffer_owner, "Noise context is null.");
    }

private:
    mutable noise_context_type *noise_ctx_p;
};

template<ntn_relation relation_type>
using noise_handshake_packet =
    packet<noise_handshake_packet_type, noise_handshake_protocol_type<relation_type>>;
using payload_packet = packet<payload_packet_type, payload_protocol_type>;

struct noise_handshake_action final : public action<noise_handshake_packet_type> {
    using packet_type = noise_handshake_action::packet_type;

public:
    constexpr void init_packet(packet_type &pckt) override {}
    constexpr void process_packet(packet_type &&pckt) override {}
};

struct payload_action final : action<payload_packet_type> {
    static constexpr std::size_t max_stream_size = max_jitter_size;

public:
    constexpr void init_packet(packet_type &pckt) override {
        audio.pop(pckt->payload.audio_buffer);
    }
    constexpr void process_packet(packet_type &&pckt) override {
        audio.push(std::move(pckt->payload.audio_buffer), pckt->payload.lost);
    }

private:
    audio_flow<max_stream_size> audio;
};
} // namespace protocol

#endif
