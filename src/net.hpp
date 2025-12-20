#ifndef NET_HPP
#define NET_HPP

#include <utils.hpp>
#include <boost/asio.hpp>
#include <chrono>

using namespace boost;

enum class ipv { v4 = 0, v6 };

static constexpr ipv IPV4 = ipv::v4;
static constexpr ipv IPV6 = ipv::v6;

template <typename T>
struct applied_native_protocol {
    constexpr virtual void prepare(T& pckt, const asio::ip::address& addr) {}
    constexpr virtual void was_accepted(T& pckt,
                                        const asio::ip::address& addr) {}

    protected:
	static constexpr noheap::log_impl::owner_impl::buffer_t buffer_owner = 
		noheap::log_impl::create_owner("PROTOCOL"); 
	static constexpr log_handler log{buffer_owner};
};

template <std::size_t _buffer_size, std::size_t _max_count_senders>
struct packet_native_t {
   public:
    static constexpr std::size_t buffer_size = _buffer_size;
    static constexpr std::size_t max_count_senders = _max_count_senders;
    
    using buffer_t = noheap::buffer_t<buffer_size>;

   public:
    packet_native_t() = default;

    virtual buffer_t::value_type* get_buffer() {
        return reinterpret_cast<buffer_t::value_type*>(this) + 8;
    }

   public:
    buffer_t buffer;
};

template <std::size_t _buffer_size, std::size_t _max_count_sender>
struct debug_extention {
    struct packet_t : public packet_native_t<_buffer_size, _max_count_sender> {
        std::size_t mark_time;
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
            static noheap::vector_stack<asio::ip::address, packet_t::max_count_senders> ip_s;
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
            if (std::find(ip_s.data.begin(), ip_s.data.end(), addr) == ip_s.data.end()) {
                ip_s.data.push_back(addr);
                noheap::println(
                    "[DEBUG_PROTOCOL]: Sent a packet for the first time: {}",
                    addr.to_string());
            }
            if (now_ms - during_ms > 1000) {
                noheap::println(
                    "[DEBUG PROTOCOL]: Was recieved last packet {} "
                    "ms({} packet/s.)",
                    now_ms - pckt.mark_time, count_accepted);
                during_ms = now_ms;
                count_accepted = 0;
            }
        }
    };
};

template <typename T,
          typename _aprotocol = applied_native_protocol<T>,
          typename = std::enable_if_t<
              std::is_base_of_v<packet_native_t<T::buffer_size, T::max_count_senders>, T>, void>,
          typename = std::enable_if_t<
              std::is_base_of_v<applied_native_protocol<T>, _aprotocol>, void>>
class packet final : public T {
   public:
    using packet_t = T;
    using aprotocol = _aprotocol;

   public:
    static constexpr std::size_t size = sizeof(T) - 8;

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

template <std::size_t _buffer_size, std::size_t _max_count_senders>
using debug_packet = packet<typename debug_extention<_buffer_size, _max_count_senders>::packet_t,
                            typename debug_extention<_buffer_size, _max_count_senders>::protocol>;

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
            throw noheap::runtime_error("Failed to open udp socket: {}.", ec.message());

        sock.bind({udp(), port}, ec);
        if (ec.value())
            throw noheap::runtime_error("Failed to bind udp socket: {}.", ec.message());

        sock.set_option(asio::socket_base::reuse_address(true));
    }

   public:
    template <typename T>
    std::enable_if_t<
        std::is_same_v<T, packet<typename T::packet_t, typename T::aprotocol>>>
    send_to(T&& pckt, ipv_t addr) {
        static constexpr auto handle_send = [](const system::error_code& ec,
                                               std::size_t size) {
            if (ec.value())
                throw noheap::runtime_error("Failed to send packet: {}.", ec.message());
            else if (size != T::size)
                throw noheap::runtime_error("An incomplete package was sent: {} bytes.", size);
        };
        static asio::ip::udp::endpoint p_sender{addr, port};
        
	const std::span<typename T::buffer_t::value_type> buffer_tmp(pckt.get_buffer(),
                                                      T::size);

        T::prepare(pckt, addr);
        sock.async_send_to(buffer_tmp, p_sender, handle_send);
    }

    template <typename T>
    std::enable_if_t<
        std::is_same_v<T, packet<typename T::packet_t, typename T::aprotocol>>>
    receive_last(T& pckt, ipv_t addr) {
        static boost::system::error_code ec;
        static asio::ip::udp::endpoint p_sender{addr, port};
        
	const std::span<typename T::buffer_t::value_type> buffer_tmp(pckt.get_buffer(),
                                                      T::size);

        std::size_t size =
            sock.receive_from(buffer_tmp, p_sender, 0, ec);

        if (ec.value())
            throw noheap::runtime_error("Failed to process packet: {}.", ec.message());
        else if (size != T::size)
            throw noheap::runtime_error("The package arrived incomplete: were accepted - {}; were expected - {}.", size, T::size);

        T::was_accepted(pckt, addr);
    }

   private:
    asio::ip::udp::socket sock;
    asio::ip::port_type port;
};

#endif
