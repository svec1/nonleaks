#ifndef NET_HPP
#define NET_HPP

#include <boost/asio.hpp>

#include "utils.hpp"

namespace network {

using namespace boost;

constexpr std::size_t timeout_ms              = 2500;
constexpr std::size_t max_buffer_address_size = asio::ip::address_v6::bytes_type{}.size();

using buffer_address_type = noheap::buffer_type<noheap::ubyte, max_buffer_address_size>;

using buffer_address_v4_type = noheap::buffer_type<buffer_address_type::value_type, 4>;
using buffer_address_v6_type = noheap::buffer_type<buffer_address_type::value_type, 8>;

enum class ipv { v4 = 0, v6 };

template<ipv v>
struct buffer_address_v {
private:
    static consteval auto get_buffer_address_type() {
        if constexpr (v == ipv::v4)
            return buffer_address_v4_type{};
        else
            return buffer_address_v6_type{};
    }

public:
    using type = decltype(get_buffer_address_type());
};

template<typename T>
struct packet_native_type;
template<typename T>
concept Packet_native_t =
    std::same_as<std::decay_t<T>,
                 packet_native_type<typename std::decay_t<T>::extention_data_type>>;

template<Packet_native_t TPacket>
struct action;
template<typename T>
concept Derived_from_action =
    std::derived_from<std::decay_t<T>, action<typename std::decay_t<T>::packet_type>>;

template<typename T>
struct packet_native_type {
public:
    using extention_data_type = T;
    using represent_type      = noheap::rbyte;

public:
    packet_native_type() = default;

public:
    packet_native_type(const packet_native_type &packet) { *this = packet; }
    packet_native_type(packet_native_type &&packet) { *this = std::move(packet); }
    packet_native_type &operator=(const packet_native_type &packet) {
        this->_extention_data = packet._extention_data;
        return *this;
    }
    packet_native_type &operator=(packet_native_type &&packet) {
        this->_extention_data = std::move(packet._extention_data);
        return *this;
    }

public:
    extention_data_type       *operator->() noexcept { return _extention_data_p; }
    const extention_data_type *operator->() const noexcept { return _extention_data_p; }

    void                set_address(buffer_address_type _address) { address = _address; }
    buffer_address_type get_address() const { return address; }

public:
    static constexpr std::size_t size() noexcept { return sizeof(extention_data_type); }

public:
    template<typename TSelf>
    decltype(auto) data(this TSelf &&self) noexcept {
        if constexpr (std::is_const_v<std::remove_reference_t<TSelf>>)
            return reinterpret_cast<const represent_type *>(&self._extention_data);
        else
            return reinterpret_cast<represent_type *>(&self._extention_data);
    }

private:
    extention_data_type          _extention_data;
    extention_data_type *const   _extention_data_p = &_extention_data;
    network::buffer_address_type address{};
};

template<Packet_native_t TPacket>
struct action {
public:
    using packet_type = TPacket;

public:
    void                init_packet(packet_type &pckt);
    void                process_packet(packet_type &&pckt);
    buffer_address_type get_remote_address();
    void                set_error(const noheap::runtime_error &_excp);
};

template<Packet_native_t T, noheap::log_impl::owner_impl::buffer_type _buffer_owner>
struct protocol_native_type {
public:
    using packet_type = T;
    using action_type = action<packet_type>;

public:
    template<typename TFunc_callback>
    void prepare(packet_type &, TFunc_callback &&) const;
    template<typename TFunc_callback>
    void handle(packet_type &&, TFunc_callback &&) const;

public:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        _buffer_owner;

protected:
    static constexpr log_handler log{buffer_owner};
};

template<Derived_from_action Action, ipv _v>
class udp_stream;

template<typename T>
concept Udp_stream = std::same_as<T, udp_stream<typename T::action_type, T::v>>;

template<Derived_from_action Action, ipv _v>
class udp_stream {
private:
    static asio::ip::udp get_ipv() {
        if constexpr (v == ipv::v6)
            return decltype(get_ipv())::v6();
        else
            return decltype(get_ipv())::v4();
    }

public:
    static constexpr ipv v = _v;

    using basic_socket_type = asio::ip::udp;
    using socket_type       = basic_socket_type::socket;
    using action_type       = Action;
    using endpoint_type     = basic_socket_type::endpoint;
    using address_type = std::conditional_t<static_cast<bool>(v), asio::ip::address_v6,
                                            asio::ip::address_v4>;
    using port_type    = asio::ip::port_type;
    using async_future_wrapper = future_wrapper<std::size_t>;

public:
    udp_stream(asio::io_context &io, asio::ip::port_type _port);
    udp_stream(udp_stream &&)      = delete;
    udp_stream(const udp_stream &) = delete;

    ~udp_stream();

public:
    auto          &get_action() { return act; }
    decltype(auto) get_executor() { return socket.get_executor(); }
    decltype(auto) get_port() const { return port; }
    decltype(auto) get_running() const { return running; }

    void close() {
        socket.cancel();
        socket.close();
        running.store(false);
    }

    void register_async_send();
    void register_async_receive();

    static buffer_address_type get_address_bytes(address_type addr);
    static address_type        get_address_object(buffer_address_type addr);
    static address_type        get_address_object(asio::ip::address addr);

private:
    static void handle_error(const system::error_code &ec);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NSTREAM");
    static constexpr log_handler log{buffer_owner};

private:
    socket_type socket;
    Action      act;

    port_type         port;
    std::atomic<bool> running;

    typename action_type::packet_type receive_pckt;
    endpoint_type                     receive_endpoint;
};

template<Derived_from_action Action, ipv v>
udp_stream<Action, v>::udp_stream(asio::io_context &io, port_type _port)
    : socket(io), port(_port), running(true) {
    system::error_code ec;

    socket.open(get_ipv(), ec);
    handle_error(ec);

    socket.set_option(typename socket_type::reuse_address(true));
    socket.set_option(typename socket_type::broadcast(false));

    socket.bind({get_ipv(), port}, ec);
    handle_error(ec);
}

template<Derived_from_action Action, ipv v>
udp_stream<Action, v>::~udp_stream() {
    socket.close();
}

template<Derived_from_action Action, ipv v>
void udp_stream<Action, v>::register_async_send() {
    // TODO: supporting pseudo proxy such as obfs4 or webtunnel
    if (!running.load() || !act.get_running())
        return;

    asio::post(socket.get_executor(), [this] {
        system::error_code                ec;
        typename action_type::packet_type pckt{};

        try {
            // Resolve remote address by the request to act
            pckt.set_address(act.get_remote_address());

            // Performs init a packet base on TWrapper_packet protocol
            act.init_packet(pckt);

            // Sends the packet
            this->socket.send_to(
                asio::mutable_buffer{pckt.data(), pckt.size()},
                endpoint_type{this->get_address_object(pckt.get_address()), this->port},
                0, ec);
            this->handle_error(ec);

        } catch (const noheap::runtime_error &_excp) {
            act.set_error(_excp);
            return;
        }

        this->register_async_send();
    });
}
template<Derived_from_action Action, ipv v>
void udp_stream<Action, v>::register_async_receive() {
    if (!running.load() || !act.get_running())
        return;

    // Waits for a new packet
    receive_pckt = {};
    this->socket.async_receive_from(
        asio::mutable_buffer{receive_pckt.data(), receive_pckt.size()}, receive_endpoint,
        0, [this](system::error_code ec, std::size_t) {
            try {
                this->handle_error(ec);

                receive_pckt.set_address(this->get_address_bytes(
                    this->get_address_object(receive_endpoint.address())));

                // Handles the packet TWrapper_packet protocol
                act.process_packet(std::move(receive_pckt));

            } catch (const noheap::runtime_error &_excp) {
                act.set_error(_excp);
                return;
            }

            this->register_async_receive();
        });
}

template<Derived_from_action Action, ipv v>
buffer_address_type udp_stream<Action, v>::get_address_bytes(address_type addr) {
    return noheap::to_new_buffer<buffer_address_type>(addr.to_bytes());
}
template<Derived_from_action Action, ipv v>
udp_stream<Action, v>::address_type
    udp_stream<Action, v>::get_address_object(buffer_address_type addr) {
    return address_type(noheap::to_new_buffer<decltype(address_type{}.to_bytes())>(addr));
}
template<Derived_from_action Action, ipv v>
udp_stream<Action, v>::address_type
    udp_stream<Action, v>::get_address_object(asio::ip::address addr) {
    if constexpr (std::same_as<address_type, asio::ip::address_v4>)
        return addr.to_v4();
    else
        return addr.to_v6();
}

template<Derived_from_action Action, ipv v>
void udp_stream<Action, v>::handle_error(const system::error_code &ec) {
    if (!ec.value())
        return;

    log.throw_exception("Network error: {}", ec.message());
}

} // namespace network

#endif
