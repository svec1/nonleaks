#ifndef NET_HPP
#define NET_HPP

#include <array>
#include <boost/asio.hpp>
#include <chrono>
#include <print>
#include <unordered_set>

using namespace boost;

enum class ipv { v4 = 0, v6 };

static constexpr ipv IPV4 = ipv::v4;
static constexpr ipv IPV6 = ipv::v6;

template <typename T>
struct applied_native_protocol {
    using cfunction_t = void(T&);

    constexpr virtual void prepare(T& pckt, const asio::ip::address& addr) {}
    constexpr virtual void was_accepted(T& pckt,
                                        const asio::ip::address& addr) {}
};

template <unsigned int _buffer_size = 4096>
struct packet_native_t {
    using buffer_el_t = char;
    using buffer_t = std::array<buffer_el_t, _buffer_size>;

   public:
    static constexpr unsigned int buffer_size = _buffer_size;

   public:
    packet_native_t() = default;

    virtual buffer_el_t* get_buffer() {
        return reinterpret_cast<buffer_el_t*>(this) + 8;
    }

   public:
    buffer_t buffer;
};

template <unsigned int _buffer_size>
struct debug_extention {
    struct packet_t : public packet_native_t<_buffer_size> {
        unsigned long long mark_time;
    };

    struct protocol : public applied_native_protocol<packet_t> {
        constexpr void prepare(packet_t& pckt,
                               const asio::ip::address& addr) override {
            pckt.mark_time =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();
        }
        constexpr void was_accepted(packet_t& pckt,
                                    const asio::ip::address& addr) override {
            static std::vector<asio::ip::address> ip_s;
            static unsigned long long count_accepted = 0;
            static unsigned long long during_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();

            unsigned long long now_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();

            ++count_accepted;
            if (std::find(ip_s.begin(), ip_s.end(), addr) == ip_s.end()) {
                ip_s.push_back(addr);
                std::println(
                    "[DEBUG_PROTOCOL]: Sent a packet for the first time: {}",
                    addr.to_string());
            }
            if (now_ms - during_ms > 1000) {
                std::println(
                    "[DEBUG PROTOCOL]: Was recieved last packet {} "
                    "ms({} packet/s.)",
                    now_ms - pckt.mark_time, count_accepted);
                during_ms = now_ms;
                count_accepted = 0;
            }
        }
    };
};

template <
    typename T = packet_native_t<>,
    typename _aprotocol = applied_native_protocol<T>,
    typename = std::enable_if_t<
        std::is_base_of_v<packet_native_t<T::buffer_size>, T>, void>,
    typename = std::enable_if_t<
        std::is_base_of_v<applied_native_protocol<T>, _aprotocol>, void>,
    typename = std::void_t<
        typename _aprotocol::cfunction_t,
        decltype(std::declval<_aprotocol&&>().prepare(
            std::declval<T&>(), std::declval<const asio::ip::address&>())),
        decltype(std::declval<_aprotocol&&>().was_accepted(
            std::declval<T&>(), std::declval<const asio::ip::address&>()))>>
class packet final : public T {
   public:
    using packet_t = T;
    using aprotocol = _aprotocol;

   public:
    static constexpr unsigned int size = sizeof(T) - 8;

   public:
    packet() = default;
    packet(T&& pckg) : T(pckg) {}

   public:
    static constexpr void prepare(T& pckt, const asio::ip::address& addr) {
        aprotocol{}.prepare(pckt, addr);
    }
    static constexpr void was_accepted(T& pckt, const asio::ip::address& addr) {
        aprotocol{}.was_accepted(pckt, addr);
    }
};

template <unsigned int _buffer_size>
using debug_packet = packet<typename debug_extention<_buffer_size>::packet_t,
                            typename debug_extention<_buffer_size>::protocol>;

template <ipv v = ipv::v4,
          typename _ipv_t = std::conditional_t<
              static_cast<bool>(v), asio::ip::address_v6, asio::ip::address_v4>>
class nstream final {
    static constexpr asio::ip::udp udp() {
        if constexpr (v == ipv::v6)
            return asio::ip::udp::v6();
        else
            return asio::ip::udp::v4();
    }

   public:
    using ipv_t = _ipv_t;

   public:
    nstream(asio::io_context& io, asio::ip::port_type _port)
        : sock(io), port(_port) {
        boost::system::error_code ec;

        sock.open(udp(), ec);
        if (ec.value())
            throw std::runtime_error(
                std::format("Failed to open udp socket({}).", ec.message()));

        sock.bind({udp(), port}, ec);
        if (ec.value())
            throw std::runtime_error(
                std::format("Failed to bind udp socket({}).", ec.message()));

        sock.set_option(asio::socket_base::reuse_address(true));
    }

   public:
    template <typename T>
    std::enable_if_t<
        std::is_same_v<T, packet<typename T::packet_t, typename T::aprotocol>>>
    send_to(T&& pckt, ipv_t addr) {
        static constexpr auto handle_send = [](const system::error_code& ec,
                                               unsigned int size) {
            if (ec.value())
                throw std::runtime_error(
                    std::format("Failed to send packet({}).", ec.message()));
            else if (size != T::size)
                throw std::runtime_error(std::format(
                    "An incomplete package was sent( bytes).", size));
        };
        std::span<typename T::buffer_el_t> buffer_tmp(pckt.get_buffer(),
                                                      T::size);

        asio::ip::udp::endpoint p_sender{addr, port};

        T::prepare(pckt, addr);
        sock.async_send_to(boost::asio::buffer(buffer_tmp), p_sender,
                           handle_send);
    }

    template <typename T>
    std::enable_if_t<
        std::is_same_v<T, packet<typename T::packet_t, typename T::aprotocol>>>
    receive_last(T& pckt, ipv_t addr) {
        std::span<typename T::buffer_el_t> buffer_tmp(pckt.get_buffer(),
                                                      T::size);
        asio::ip::udp::endpoint p_sender{addr, port};
        boost::system::error_code ec;

        unsigned int size =
            sock.receive_from(boost::asio::buffer(buffer_tmp), p_sender, 0, ec);

        if (ec.value())
            throw std::runtime_error(
                std::format("Failed to process packet({}).", ec.message()));
        else if (size != T::size)
            throw std::runtime_error(std::format(
                "The package arrived incomplete({}:{}).", size, T::size));

        T::was_accepted(pckt, addr);
    }

   private:
    asio::ip::udp::socket sock;
    asio::ip::port_type port;
};

#endif
