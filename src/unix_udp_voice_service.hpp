#ifndef UDP_VOICE_SERVICE_HPP
#define UDP_VOICE_SERVICE_HPP

#include "crypt.hpp"
#include "net.hpp"
#include "stream_audio.hpp"

using namespace boost;

template<std::size_t _max_stream_size>
struct voice_extention {
    static constexpr std::size_t max_stream_size        = _max_stream_size;
    static constexpr std::size_t max_count_contributers = 4;

public:
    struct packet_t : public packet_native_t<stream_audio::encode_buffer_size> {
        static constexpr std::size_t uuid_size = 8;

        using uuid_type = openssl_context::buffer_type<uuid_size>;

    public:
        std::int8_t rtp_meta_inf;

        // Payload type: (sample_rate + bitrate + (channels * 100)) / 1000;
        std::uint8_t  payload_type;
        std::uint16_t sequence_number;
        std::uint32_t ssrc;

        std::array<uuid_type, max_count_contributers> csrc;
    };
    struct protocol_t : public protocol_native_t<packet_t, noheap::log_impl::create_owner(
                                                               "VOICE_PROTOCOL")> {
    public:
        constexpr void prepare(packet_t                      &pckt,
                               protocol_t::callback_prepare_t callback) const override {
            static openssl_context ossl_ctx;

            if (!initialized) {
                uuid                  = ossl_ctx.get_random_bytes<packet_t::uuid_size>();
                local_sequence_number = *reinterpret_cast<std::uint16_t *>(
                    ossl_ctx.get_random_bytes<sizeof(std::uint16_t)>().data());

                initialized = true;
            }
            callback(pckt);

            ++local_sequence_number;

            pckt.rtp_meta_inf = 2 | (1 << 4);
            pckt.payload_type = static_cast<int8_t>(
                (stream_audio::default_base_audio::cfg.sample_rate
                 + stream_audio::default_base_audio::cfg.bitrate
                 + (stream_audio::default_base_audio::cfg.channels * 100) / 1000));
            pckt.sequence_number = local_sequence_number;
            pckt.ssrc            = 0;
            pckt.csrc[0]         = uuid;
        }
        constexpr void handle(packet_t                     &pckt,
                              protocol_t::callback_handle_t callback) const override {
            std::ssize_t diff =
                jitter_buffer.size()
                    ? pckt.sequence_number - (jitter_buffer.end() - 1)->sequence_number
                    : 0;
            if (diff < 0) {
                auto it = jitter_buffer.end() + diff - 1;
                if (it->buffer.empty())
                    *it = pckt;
                else
                    jitter_buffer.emplace(it, pckt);
            } else {
                if (diff > 1) {
                    count_lost_packets += diff;
                    for (; diff > 0; --diff)
                        jitter_buffer.push_back(packet_t{});
                }
                jitter_buffer.push_back(std::move(pckt));
            }
            ++count_received_packets;

            if (jitter_buffer.size() == max_stream_size)
                filled = true;

            if (filled)
                callback(jitter_buffer.pop_front());
        }

    public:
        std::size_t get_loss_per_cent() const {
            if (!count_received_packets)
                return 0;
            return count_lost_packets / count_received_packets;
        }

    private:
        mutable bool                         filled                 = false;
        mutable bool                         initialized            = false;
        mutable std::size_t                  count_lost_packets     = 0;
        mutable std::size_t                  count_received_packets = 0;
        mutable std::uint32_t                local_sequence_number;
        mutable typename packet_t::uuid_type uuid;

        mutable noheap::monotonic_array<packet_t, max_stream_size> jitter_buffer;
    };

public:
    using packet = packet<packet_t, protocol_t>;

public:
    struct action : public ::action<packet> {
        using packet = action::packet;

    public:
        action() : running(true), filled(0) {
            in_stream  = std::async(std::launch::async, [this] {
                try {
                    while (running.load()) {
                        typename stream_audio::encode_buffer_type buffer_tmp =
                            io_audio.read();
                        {
                            std::lock_guard lock(in_stream_m);
                            in_stream_buffer.push(buffer_tmp);
                        }
                        filled.fetch_add(1);
                        filled.notify_one();
                    }
                } catch (...) {
                    running.store(false);
                    throw;
                }
            });
            out_stream = std::async(std::launch::async, [this] {
                try {
                    while (running.load()) {
                        std::size_t current_out_stream_size;

                        typename packet::packet_type::buffer_type buffer_tmp;
                        {
                            std::lock_guard lock(out_stream_m);
                            current_out_stream_size = out_stream_buffer.size();
                            buffer_tmp              = out_stream_buffer.pop();
                        }
                        io_audio.write(buffer_tmp,
                                       current_out_stream_size && buffer_tmp.empty());
                    }
                } catch (...) {
                    running.store(false);
                    throw;
                }
            });
        }
        ~action() {
            running.store(false);
            wait_stopping();
        }

    public:
        constexpr void init_packet(packet::packet_t &pckt) override {
            check_running();

            if (!filled.load())
                filled.wait(0);
            filled.fetch_sub(1);

            std::lock_guard lock(in_stream_m);
            pckt.buffer = in_stream_buffer.pop();
        }
        constexpr void process_packet(packet::packet_t &&pckt) override {
            check_running();

            std::lock_guard lock(out_stream_m);
            out_stream_buffer.push(pckt.buffer);
        }

    private:
        void check_running() {
            if (running.load())
                return;

            wait_stopping();
        }

        void wait_stopping() {
            if (in_stream.valid())
                in_stream.get();
            if (out_stream.valid())
                out_stream.get();
        }

    private:
        std::mutex               in_stream_m, out_stream_m;
        std::future<void>        in_stream, out_stream;
        std::atomic<bool>        running;
        std::atomic<std::size_t> filled;

        stream_audio io_audio;

        noheap::ring_buffer<typename packet::buffer_type, max_stream_size>
            in_stream_buffer, out_stream_buffer;
    };
};

class unix_udp_voice_service {
public:
    static constexpr ipv v = ipv::v4;

    using voice_extention_d = voice_extention<64>;
    using udp_stream_type   = net_stream_udp<typename voice_extention_d::action, v>;
    using packet            = voice_extention_d::packet;
    using ipv_type          = udp_stream_type::ipv_type;

public:
    unix_udp_voice_service(asio::ip::port_type port);

public:
    void run(const ipv_type &addr);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

private:
    asio::io_context io;
    udp_stream_type  udp_stream;
};

unix_udp_voice_service::unix_udp_voice_service(asio::ip::port_type port)
    : udp_stream(io, port) {
}
void unix_udp_voice_service::run(const ipv_type &addr) {
    try {
        std::future<void> voice_handler = std::async(std::launch::async, [&] {
            packet pckt_for_receiving{}, pckt_for_sending{};

            udp_stream.register_send_handler<0>(pckt_for_sending, addr);
            udp_stream.register_receive_handler(pckt_for_receiving);

            io.run();
        });

        if (voice_handler.valid())
            voice_handler.get();

    } catch (noheap::runtime_error &excp) {
        if (!excp.has_setting_owner())
            excp.set_owner(buffer_owner);
        throw;
    }
}

#endif
