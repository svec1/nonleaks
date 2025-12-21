#ifndef ALSA_UDP_VOICE_SERVICE_HPP
#define ALSA_UDP_VOICE_SERVICE_HPP

#include <atomic>

#include "aio.hpp"
#include "net.hpp"

using namespace boost;

template <typename T> class unix_udp_voice_service {
  public:
    static constexpr ipv v = ipv::v4;

    using nstream_t = nstream<v>;
    using package_t = T;

  public:
    unix_udp_voice_service(std::span<nstream_t::ipv_t> addrs,
                           asio::ip::port_type port);

  private:
    void run(const std::span<nstream_t::ipv_t> &addrs);

  private:
    static void send_samples(nstream_t &net, input &in,
                             const std::span<nstream_t::ipv_t> &addrs);
    static void receive_samples(nstream_t &net, output &out,
                                const nstream_t::ipv_t &addr);

  private:
    static constexpr noheap::log_impl::owner_impl::buffer_t buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

  private:
    input in;
    output out;

    asio::io_context io;
    nstream_t net;

    static std::atomic<bool> end;
};

template <typename T>
unix_udp_voice_service<T>::unix_udp_voice_service(
    std::span<nstream_t::ipv_t> addrs, asio::ip::port_type port)
    : net(io, port) {
    run(addrs);
}
template <typename T>
void unix_udp_voice_service<T>::run(const std::span<nstream_t::ipv_t> &addrs) {
    if (addrs.size() > package_t::max_count_senders)
        throw noheap::runtime_error(
            "The senders IP limit has been exceeded: {}.",
            package_t::max_count_senders);

    noheap::vector_stack<std::future<void>, package_t::max_count_senders>
        ts_receivers;
    std::for_each(
        addrs.begin(), addrs.end(), [&](const nstream_t::ipv_t &addr) {
            ts_receivers.data.push_back(
                std::async(std::launch::async, this->receive_samples,
                           std::ref(net), std::ref(out), std::cref(addr)));
        });
    std::future sender = std::async(this->send_samples, std::ref(net),
                                    std::ref(in), std::cref(addrs));

    sender.get();
    std::for_each(ts_receivers.data.begin(), ts_receivers.data.end(),
                  [&](auto &t_receiver) { t_receiver.get(); });
}
template <typename T>
void unix_udp_voice_service<T>::send_samples(
    nstream_t &net, input &in, const std::span<nstream_t::ipv_t> &addrs) {
    try {
        static package_t pckg;
        while (!end.load()) {
            for (std::size_t i = 0;
                 i < package_t::buffer_size / audio::buffer_size; ++i) {
                const auto buffer_tmp = in.get_samples();
                std::copy(buffer_tmp.begin(), buffer_tmp.end(),
                          pckg.buffer.begin() + i * audio::buffer_size);
            }
            std::for_each(addrs.begin(), addrs.end(), [&](auto &addr) {
                net.send_to(std::move(pckg), addr);
            });
        }
    } catch (...) {
        end.store(true);
    }
}
template <typename T>
void unix_udp_voice_service<T>::receive_samples(nstream_t &net, output &out,
                                                const nstream_t::ipv_t &addr) {
    try {
        static package_t pckg;
        while (!end.load()) {
            net.receive_last(pckg, addr);
            for (std::size_t i = 0;
                 i < package_t::buffer_size / audio::buffer_size; ++i) {
                audio::buffer_t buffer_tmp;
                std::copy(pckg.buffer.begin() + i * audio::buffer_size,
                          pckg.buffer.begin() + (i + 1) * audio::buffer_size,
                          buffer_tmp.begin());
                out.play_samples(buffer_tmp);
            }
        }
    } catch (...) {
        end.store(true);
    }
}
template <typename T> std::atomic<bool> unix_udp_voice_service<T>::end = false;

#endif
