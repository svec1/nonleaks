#ifndef ALSA_UDP_VOICE_SERVICE_HPP
#define ALSA_UDP_VOICE_SERVICE_HPP

#include "aio.hpp"
#include "crypt.hpp"
#include "net.hpp"

using namespace boost;

template <std::size_t _buffer_size, std::size_t _max_count_addrs,
          std::size_t _max_count_jitter_packet>
struct voice_extention {
    static constexpr std::size_t max_count_jitter_packet =
        _max_count_jitter_packet;

  public:
    struct packet_t : public packet_native_t<_buffer_size, _max_count_addrs> {
        static constexpr std::size_t uuid_size = 32;
        using uuid_type = openssl_context::buffer_t<uuid_size>;

        uuid_type uuid;
        size_t sequence_number = 0;
    };
    struct protocol_t
        : public protocol_native_t<packet_t, noheap::log_impl::create_owner(
                                                 "VOICE_PROTOCOL")> {
        using array_jitter_buffers_t = noheap::monotonic_array<
            std::pair<typename packet_t::uuid_type,
                      noheap::ring_buffer<packet_t, max_count_jitter_packet>>,
            packet_t::max_count_addrs>;

        constexpr void prepare(packet_t &pckt) const override {
            static openssl_context ossl_ctx;
            static typename packet_t::uuid_type uuid{
                ossl_ctx.get_random_bytes<packet_t::uuid_size>()};

            pckt.uuid = uuid;
            pckt.sequence_number = local_sequence_number++;
        }
        constexpr void
        handle(packet_t &pckt,
               protocol_t::callback_func_t callback) const override {
            if (!filled &&
                !std::none_of(buffer.begin(), buffer.end(),
                              [](const array_jitter_buffers_t::value_type &el) {
                                  return el.second.size() + 1 ==
                                         max_count_jitter_packet;
                              }))
                filled = true;

            auto it = buffer.begin();
            if (it = std::find_if(
                    buffer.begin(), buffer.end(),
                    [&](const array_jitter_buffers_t::value_type &el) {
                        return !std::memcmp(el.first.data(), pckt.uuid.data(),
                                            packet_t::uuid_size);
                    });
                it == buffer.cend()) {
                if (buffer.size() == packet_t::max_count_addrs)
                    return;

                buffer.push_back(
                    std::make_pair<
                        typename array_jitter_buffers_t::value_type::first_type,
                        typename array_jitter_buffers_t::value_type::
                            second_type>(std::move(pckt.uuid), {}));

                it = buffer.end() - 1;
            }
            it->second.push(std::move(pckt));
            if (filled)
                callback(it->second.pop());
        }

      private:
        mutable array_jitter_buffers_t buffer;
        mutable std::size_t local_sequence_number = 0;
        mutable bool filled = false;
    };

  public:
    using packet = packet<packet_t, protocol_t>;

  public:
    struct action : public ::action<packet> {
        using packet = ::action<packet>::packet;

      public:
        constexpr void init_buffer(packet::buffer_t &buffer) const override {
            static input in;
            for (std::size_t i = 0;
                 i < packet_t::buffer_size / audio::buffer_size; ++i) {
                audio::buffer_t buffer_tmp = in.get_samples();
                std::copy(buffer_tmp.begin(), buffer_tmp.end(),
                          buffer.begin() + i * audio::buffer_size);
            }
        }
        constexpr void process_packet(packet &&pckt) const override {
            using array_buffers_t = noheap::ring_buffer<
                std::pair<std::size_t, typename packet::buffer_t>,
                max_count_jitter_packet>;

            static std::mutex m;
            static array_buffers_t buffers;
            {
                std::lock_guard lock(m);
                if (auto it = std::find_if(
                        buffers.lbegin(), buffers.lend(),
                        [&](const typename array_buffers_t::buffer_type::
                                value_type &el) {
                            return el.first == pckt.sequence_number;
                        });
                    it != buffers.lend()) {
                    std::transform(it->second.cbegin(), it->second.cend(),
                                   pckt.buffer.cend(), it->second.begin(),
                                   [](const auto &right, const auto &left) {
                                       return (right + left) / 2;
                                   });
                } else
                    buffers.push(typename array_buffers_t::value_type{
                        pckt.sequence_number, pckt.buffer});
            }
            static std::jthread j([]() {
                output out;

                while (true) {
                    typename packet::packet_t::buffer_t buffer;
                    {
                        std::lock_guard lock(m);
                        buffer = buffers.pop().second;
                    }
                    out.play_samples(buffer);
                }
            });
        }
    };
};

template <std::size_t _max_count_addrs> class unix_udp_voice_service {
  public:
    static constexpr ipv v = ipv::v4;

    using nstream_t = nstream<v>;
    using voice_extention_d =
        voice_extention<audio::buffer_size, _max_count_addrs, 64>;
    using packet = voice_extention_d::packet;

  public:
    unix_udp_voice_service(
        const noheap::monotonic_array<nstream_t::ipv_t, packet::max_count_addrs>
            &addrs,
        asio::ip::port_type port);

  private:
    void run(const noheap::monotonic_array<nstream_t::ipv_t,
                                           packet::max_count_addrs> &addrs);

  private:
    static constexpr noheap::log_impl::owner_impl::buffer_t buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

  private:
    asio::io_context io;
    nstream_t net;
};
template <std::size_t _max_count_addrs>
unix_udp_voice_service<_max_count_addrs>::unix_udp_voice_service(
    const noheap::monotonic_array<nstream_t::ipv_t, packet::max_count_addrs>
        &addrs,
    asio::ip::port_type port)
    : net(io, port) {
    run(addrs);
}
template <std::size_t _max_count_addrs>
void unix_udp_voice_service<_max_count_addrs>::run(
    const noheap::monotonic_array<nstream_t::ipv_t, packet::max_count_addrs>
        &addrs) {
    try {
        packet pckt_for_sender, pckt_for_receiver, pckt_for_receiver2;

        net.receive_from<typename voice_extention_d::action>(pckt_for_receiver);
        net.send_to<typename voice_extention_d::action>(pckt_for_sender, addrs);

        io.run();

    } catch (noheap::runtime_error &excp) {
        if (!excp.has_setting_owner())
            excp.set_owner(buffer_owner);
        throw;
    }
}

#endif
