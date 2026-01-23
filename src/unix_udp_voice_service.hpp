#ifndef ALSA_UDP_VOICE_SERVICE_HPP
#define ALSA_UDP_VOICE_SERVICE_HPP

#include "aio.hpp"
#include "coder_audio.hpp"
#include "crypt.hpp"
#include "net.hpp"

using namespace boost;

template<std::size_t _max_stream_size, std::size_t _min_count_blocks_for_nonwaiting>
struct voice_extention {
    static constexpr std::size_t max_stream_size = _max_stream_size;
    static constexpr std::size_t min_count_blocks_for_nonwaiting =
        _min_count_blocks_for_nonwaiting;

    using ca_type = coder_audio<default_base_audio::cfg>;

public:
    struct packet_t : public packet_native_t<ca_type::encode_buffer_size> {
        static constexpr std::size_t uuid_size = 32;
        using uuid_type                        = openssl_context::buffer_type<uuid_size>;

        uuid_type uuid;
        size_t    sequence_number = 0;
    };
    struct protocol_t : public protocol_native_t<packet_t, noheap::log_impl::create_owner(
                                                               "VOICE_PROTOCOL")> {
    public:
        constexpr void prepare(packet_t                      &pckt,
                               protocol_t::callback_prepare_t callback) const override {
            static openssl_context                    ossl_ctx;
            static const typename packet_t::uuid_type uuid{
                ossl_ctx.get_random_bytes<packet_t::uuid_size>()};

            callback(pckt);

            pckt.uuid            = uuid;
            pckt.sequence_number = local_sequence_number++;
        }
        constexpr void handle(packet_t                     &pckt,
                              protocol_t::callback_handle_t callback) const override {
            jitter_buffer.push_back(std::move(pckt));

            if (jitter_buffer.size() == max_stream_size)
                filled = true;

            if (filled)
                callback(jitter_buffer.pop_front());
        }

    private:
        mutable noheap::monotonic_array<packet_t, max_stream_size> jitter_buffer;

        mutable std::size_t local_sequence_number = 0;
        mutable bool        filled                = false;
    };

public:
    using packet = packet<packet_t, protocol_t>;

public:
    struct action : public ::action<packet> {
        using packet = ::action<packet>::packet;

    public:
        action() : running(true) {
            in_stream  = std::async(std::launch::async, [this] {
                try {
                    input in;

                    while (running.load()) {
                        typename ca_type::encode_buffer_type buffer_tmp =
                            ca.encode(in.get_samples());
                        {
                            std::lock_guard lock(in_stream_m);
                            in_stream_buffer.push(buffer_tmp);
                        }
                    }
                } catch (...) {
                    running.store(false);
                    throw;
                }
            });
            out_stream = std::async(std::launch::async, [this] {
                try {
                    output out;

                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        default_base_audio::cfg.latency * max_stream_size));
                    while (running.load()) {
                        if (out_stream_buffer.size() < min_count_blocks_for_nonwaiting)
                            std::this_thread::sleep_for(std::chrono::milliseconds(
                                default_base_audio::cfg.latency));
                        typename packet::packet_type::buffer_type buffer_tmp;
                        {
                            std::lock_guard lock(out_stream_m);
                            buffer_tmp = out_stream_buffer.pop();
                        }
                        auto begin = get_now_ms();
                        out.play_samples(ca.decode(buffer_tmp, false));
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

    public:
        constexpr void init_packet(packet::packet_t &pckt) override {
            check_running();
            std::lock_guard lock(in_stream_m);
            pckt.buffer = in_stream_buffer.pop();
        }
        constexpr void process_packet(packet::packet_t &&pckt) override {
            check_running();
            std::lock_guard lock(out_stream_m);
            out_stream_buffer.push(pckt.buffer);
        }

    private:
        std::mutex        in_stream_m, out_stream_m;
        std::future<void> in_stream, out_stream;
        std::atomic<bool> running;

        ca_type ca;

        noheap::ring_buffer<typename packet::buffer_type, max_stream_size>
            in_stream_buffer, out_stream_buffer;
    };
};

class unix_udp_voice_service {
public:
    static constexpr ipv v = ipv::v4;

    using voice_extention_d = voice_extention<64, 16>;
    using udp_stream_t      = net_stream_udp<typename voice_extention_d::action, v>;
    using packet            = voice_extention_d::packet;
    using ipv_t             = udp_stream_t::ipv_t;

public:
    unix_udp_voice_service(asio::ip::port_type port);

public:
    void run(const ipv_t &addr);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

private:
    asio::io_context io;
    udp_stream_t     udp_stream;
};

unix_udp_voice_service::unix_udp_voice_service(asio::ip::port_type port)
    : udp_stream(io, port) {
}
void unix_udp_voice_service::run(const ipv_t &addr) {
    try {
        thread_local packet pckt_for_receiving, pckt_for_sending;

        udp_stream.register_receive_handler(pckt_for_receiving);
        udp_stream.register_send_handler<default_base_audio::cfg.latency>(
            pckt_for_sending, addr);

        io.run();

    } catch (noheap::runtime_error &excp) {
        if (!excp.has_setting_owner())
            excp.set_owner(buffer_owner);
        throw;
    }
}

#endif
