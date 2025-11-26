#ifndef ALSA_UDP_VOICE_SERVICE_HPP
#define ALSA_UDP_VOICE_SERVICE_HPP

#include <thread>
#include <vector>

#include <aio.hpp>
#include <net.hpp>

using namespace boost;

template <typename T>
class alsa_udp_voice_service {
   public:
    static constexpr ipv v = ipv::v4;

    using nstream_t = nstream<v>;
    using package_t = T;

   public:
    alsa_udp_voice_service(const std::vector<nstream_t::ipv_t>& addrs,
                           asio::ip::port_type port)
        : net(io, port) {
        run(addrs);
    }

   private:
    void run(const std::vector<nstream_t::ipv_t>& addrs) {
        std::vector<std::thread> ts_receivers;
        std::for_each(
            addrs.begin(), addrs.end(), [&](const nstream_t::ipv_t& addr) {
                ts_receivers.emplace_back(this->receive_samples, std::ref(net),
                                          std::ref(out), std::cref(addr));
            });
        std::thread sender(this->send_samples, std::ref(net), std::ref(in),
                           std::cref(addrs));

        sender.join();
        std::for_each(ts_receivers.begin(), ts_receivers.end(),
                      [&](auto& t_receiver) { t_receiver.detach(); });
    }

   private:
    static void send_samples(nstream_t& net, input& in,
                             const std::vector<nstream_t::ipv_t>& addrs) {
        try {
            static package_t pckg;
            while (true) {
                for (unsigned int i = 0;
                     i < package_t::buffer_size / audio::buffer_size; ++i) {
                    const auto arr = in.get_samples();
                    std::copy(arr.begin(), arr.end(),
                              pckg.buffer.begin() + i * audio::buffer_size);
                }
                std::for_each(addrs.begin(), addrs.end(), [&](auto& addr) {
                    net.send_to(std::move(pckg), addr);
                });
            }
        } catch (std::runtime_error& excp) {
            std::println("{}", excp.what());
            exit(1);
        }
    }
    static void receive_samples(nstream_t& net, output& out,
                                const nstream_t::ipv_t& addr) {
        try {
            static package_t pckg;
            while (true) {
                net.receive_last(pckg, addr);
                out.play_samples(
                    std::span<typename package_t::buffer_el_t,
                              package_t::buffer_size>{pckg.buffer});
            }
        } catch (std::runtime_error& excp) {
            std::println("{}", excp.what());
            exit(1);
        }
    }

   public:
    input in;
    output out;

    asio::io_context io;
    nstream_t net;
};

#endif
