#ifndef NET_HPP
#define NET_HPP

#include <boost/asio.hpp>

#include "utils.hpp"

namespace network {

using namespace boost;

constexpr std::size_t max_buffer_address_size = asio::ip::address_v6::bytes_type{}.size();

using buffer_address_type = noheap::buffer_type<noheap::ubyte, max_buffer_address_size>;
using buffer_string_address_type = noheap::buffer_chars_type<39>;
using port_type                  = std::uint16_t;
using address_type               = asio::ip::address;

enum class ipv {
    v4 = 0,
    v6,
    v4v6, // Dual mode for only starting udp_stream
};

namespace utils {
    buffer_address_type        get_address_bytes(address_type addr);
    address_type               get_address_object(buffer_address_type addr, ipv v);
    buffer_string_address_type bytes_address_to_string(buffer_address_type address,
                                                       ipv                 v);
    buffer_address_type        string_address_to_bytes(std::string_view address, ipv v);
} // namespace utils

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

struct native_endpoint {
    ipv                 v;
    buffer_address_type address;
    port_type           port;
};

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
        this->endpoint        = packet.endpoint;
        return *this;
    }
    packet_native_type &operator=(packet_native_type &&packet) {
        this->_extention_data = std::move(packet._extention_data);
        this->endpoint        = std::move(packet.endpoint);
        return *this;
    }

public:
    extention_data_type       *operator->() noexcept { return _extention_data_p; }
    const extention_data_type *operator->() const noexcept { return _extention_data_p; }

    void set_endpoint(const native_endpoint &_endpoint) { endpoint = _endpoint; }
    const native_endpoint &get_endpoint() const { return endpoint; }

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
    extention_data_type        _extention_data;
    extention_data_type *const _extention_data_p = &_extention_data;
    native_endpoint            endpoint;
};

template<Packet_native_t TPacket>
struct action {
public:
    using packet_type = TPacket;

public:
    bool init_packet(packet_type &pckt);
    void handle_packet(packet_type &&pckt);
    void set_error(const noheap::runtime_error &_excp);
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

template<Derived_from_action Action>
class udp_stream;

template<typename T>
concept Udp_stream = std::same_as<T, udp_stream<typename T::action_type>>;

template<Derived_from_action Action>
class udp_stream {
public:
    using basic_socket_type = asio::ip::udp;
    using socket_type       = basic_socket_type::socket;
    using action_type       = Action;
    using endpoint_type     = basic_socket_type::endpoint;

public:
    udp_stream(asio::io_context &io, asio::ip::port_type _port);
    udp_stream(udp_stream &&)      = delete;
    udp_stream(const udp_stream &) = delete;

    ~udp_stream();

public:
    auto          &get_action();
    decltype(auto) get_executor();
    decltype(auto) get_port() const;
    decltype(auto) get_running() const;
    void           open(ipv v);
    void           close();

private:
    void register_async_send();
    void register_async_receive(ipv v);

    static void handle_error(const system::error_code &ec);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NSTREAM");
    static constexpr log_handler log{buffer_owner};

private:
    mutable std::mutex m;
    Action             act;
    socket_type        socket_v4;
    socket_type        socket_v6;

    port_type port;
    bool      running;
    ipv       v;

    typename action_type::packet_type receive_pckt_v4, receive_pckt_v6;
    endpoint_type                     receive_endpoint_v4, receive_endpoint_v6;
};

template<Derived_from_action Action>
udp_stream<Action>::udp_stream(asio::io_context &io, port_type _port)
    : socket_v4(io), socket_v6(io), port(_port), running(false) {
}
template<Derived_from_action Action>
udp_stream<Action>::~udp_stream() {
    if (running)
        close();
}

template<Derived_from_action Action>
auto &udp_stream<Action>::get_action() {
    std::lock_guard<std::mutex> m_lock(m);
    return act;
}
template<Derived_from_action Action>
decltype(auto) udp_stream<Action>::get_executor() {
    std::lock_guard<std::mutex> m_lock(m);
    return socket_v4.get_executor();
}
template<Derived_from_action Action>
decltype(auto) udp_stream<Action>::get_port() const {
    std::lock_guard<std::mutex> m_lock(m);
    return port;
}
template<Derived_from_action Action>
decltype(auto) udp_stream<Action>::get_running() const {
    std::lock_guard<std::mutex> m_lock(m);
    return running;
}
template<Derived_from_action Action>
void udp_stream<Action>::open(ipv _v) {
    {
        static constexpr const auto init_socket = [](socket_type &socket, ipv v,
                                                     port_type port) {
            system::error_code ec;

            socket.open(v == ipv::v4 ? asio::ip::udp::v4() : asio::ip::udp::v6(), ec);
            handle_error(ec);

            socket.set_option(typename socket_type::reuse_address(true));
            socket.set_option(typename socket_type::broadcast(false));

            socket.bind({v == ipv::v4 ? asio::ip::udp::v4() : asio::ip::udp::v6(), port},
                        ec);
            handle_error(ec);
        };

        std::lock_guard<std::mutex> m_lock(m);
        if (running)
            log.throw_exception("Failed to open udp stream.");

        v       = _v;
        running = true;

        if (v == ipv::v4 || v == ipv::v4v6)
            init_socket(socket_v4, ipv::v4, port);
        if (v == ipv::v6 || v == ipv::v4v6)
            init_socket(socket_v6, ipv::v6, port);
    }

    register_async_send();
    if (v == ipv::v4 || v == ipv::v4v6)
        register_async_receive(ipv::v4);
    if (v == ipv::v6 || v == ipv::v4v6)
        register_async_receive(ipv::v6);
}
template<Derived_from_action Action>
void udp_stream<Action>::close() {
    std::lock_guard<std::mutex> m_lock(m);
    if (!running)
        log.throw_exception("Failed to close udp stream.");

    if (v == ipv::v4 || v == ipv::v4v6) {
        socket_v4.cancel();
        socket_v4.close();
    }
    if (v == ipv::v6 || v == ipv::v4v6) {
        socket_v6.cancel();
        socket_v6.close();
    }
    running = false;
}

template<Derived_from_action Action>
void udp_stream<Action>::register_async_send() {
    std::lock_guard<std::mutex> m_lock(m);
    if (!running || !act.get_running())
        return;

    asio::post(socket_v4.get_executor(), [this] {
        try {
            typename action_type::packet_type pckt;

            // Performs init a packet base on TWrapper_packet protocol
            // NOTE: init_packet can wait
            if (!this->act.init_packet(pckt))
                return;

            auto native_remote_endpoint = pckt.get_endpoint();

            // Sends the packet
            {
                system::error_code          ec;
                std::lock_guard<std::mutex> m_lock(this->m);
                asio::const_buffer          buffer{pckt.data(), pckt.size()};
                endpoint_type               remote_endpoint{
                    utils::get_address_object(native_remote_endpoint.address,
                                              native_remote_endpoint.v),
                    native_remote_endpoint.port};

                if (native_remote_endpoint.v == ipv::v4)
                    this->socket_v4.send_to(buffer, remote_endpoint, 0, ec);
                else
                    this->socket_v6.send_to(buffer, remote_endpoint, 0, ec);

                handle_error(ec);
            }

            this->register_async_send();
        } catch (const noheap::runtime_error &_excp) {
            this->act.set_error(_excp);
            return;
        }
    });
}
template<Derived_from_action Action>
void udp_stream<Action>::register_async_receive(ipv v) {
    std::lock_guard<std::mutex> m_lock(m);
    if (!running || !act.get_running())
        return;

    const auto handler = [this, v](system::error_code ec, std::size_t) {
        try {
            handle_error(ec);
            {
                std::lock_guard<std::mutex> m_lock(this->m);
                // Handles the packet TWrapper_packet protocol
                if (v == ipv::v4) {
                    this->receive_pckt_v4.set_endpoint(
                        {v, utils::get_address_bytes(this->receive_endpoint_v4.address()),
                         this->receive_endpoint_v4.port()});
                    this->act.handle_packet(std::move(this->receive_pckt_v4));
                    this->receive_pckt_v4 = {};
                } else {
                    this->receive_pckt_v6.set_endpoint(
                        {v, utils::get_address_bytes(this->receive_endpoint_v6.address()),
                         this->receive_endpoint_v6.port()});
                    this->act.handle_packet(std::move(this->receive_pckt_v6));
                    this->receive_pckt_v6 = {};
                }
            }
            this->register_async_receive(v);
        } catch (const noheap::runtime_error &_excp) {
            this->act.set_error(_excp);
            return;
        };
    };

    // Waits for a new packet
    if (v == ipv::v4)
        this->socket_v4.async_receive_from(
            asio::mutable_buffer{receive_pckt_v4.data(), receive_pckt_v4.size()},
            receive_endpoint_v4, 0, handler);
    else
        this->socket_v6.async_receive_from(
            asio::mutable_buffer{receive_pckt_v6.data(), receive_pckt_v6.size()},
            receive_endpoint_v6, 0, handler);
}

template<Derived_from_action Action>
void udp_stream<Action>::handle_error(const system::error_code &ec) {
    if (!ec.value())
        return;

    log.throw_exception("Network error: {}", ec.message());
}

buffer_address_type utils::get_address_bytes(address_type addr) {
    if (addr.is_v4())
        return noheap::to_new_buffer<buffer_address_type>(addr.to_v4().to_bytes());
    else
        return noheap::to_new_buffer<buffer_address_type>(addr.to_v6().to_bytes());
}
address_type utils::get_address_object(buffer_address_type addr, ipv v) {
    if (v == ipv::v4)
        return asio::ip::address(asio::ip::address_v4(
            noheap::to_new_buffer<decltype(asio::ip::address{}.to_v4().to_bytes())>(
                addr)));
    else
        return asio::ip::address(asio::ip::address_v6(
            noheap::to_new_buffer<decltype(asio::ip::address{}.to_v6().to_bytes())>(
                addr)));
}
buffer_string_address_type utils::bytes_address_to_string(buffer_address_type address,
                                                          ipv                 v) {
    decltype(bytes_address_to_string(address, v)) buffer_tmp{};
    if (v == ipv::v4)
        std::format_to_n(buffer_tmp.begin(), buffer_tmp.size(), "{:d}.{:d}.{:d}.{:d}",
                         address[0], address[1], address[2], address[3]);
    else {
        auto it = buffer_tmp.begin();
        for (std::size_t i = 0; i < address.size(); i += 2) {
            it = std::format_to_n(it, buffer_tmp.size(), "{:x}",
                                  *reinterpret_cast<std::uint16_t *>(address.begin() + i))
                     .out;
            if (i != address.size() - 1)
                std::format_to_n(it, buffer_tmp.size(), ":");
        }
    }
    return buffer_tmp;
}

buffer_address_type utils::string_address_to_bytes(std::string_view address, ipv v) {
    try {
        decltype(string_address_to_bytes(address, v)) buffer_tmp{};

        if (v == ipv::v4) {
            std::size_t i = 0, pos = 0, pos_curr;
            while ((pos_curr = address.find_first_of(".", pos)) != address.npos
                   || pos < address.size()) {
                if (pos_curr == address.npos && pos < address.size()) {
                    if (i != 3)
                        throw std::exception{};
                    pos_curr = address.size() - 1;
                }

                auto ip_block_string = address.substr(pos, pos_curr);
                std::from_chars(ip_block_string.begin(), ip_block_string.end(),
                                buffer_tmp[i++]);
                pos = pos_curr + 1;
            }
            if (i != 4)
                throw std::exception{};
        } else {
            std::size_t i = 0, pos = 0, pos_curr;
            while ((pos_curr = address.find_first_of(":", pos)) != address.npos
                   || pos < address.size()) {
                if (pos_curr == address.npos && pos < address.size()) {
                    if (i != 7)
                        throw std::exception{};
                    pos_curr = address.size() - 1;
                }

                auto ip_block_string = address.substr(pos, pos_curr);

                noheap::buffer_chars_type<4> ip_block_buffer_tmp{};
                std::copy(ip_block_string.begin(), ip_block_string.end(),
                          ip_block_buffer_tmp.begin());
                *reinterpret_cast<std::uint16_t *>(&buffer_tmp[i]) =
                    noheap::represent_bytes<std::uint16_t>(
                        noheap::hex_decode(ip_block_buffer_tmp));

                pos = pos_curr;
                i += 2;
            }
            if (i != 8)
                throw std::exception{};
        }

        return buffer_tmp;
    } catch (...) {
        throw noheap::runtime_error("Invalid string of ip address: {}.", address);
    }
}

} // namespace network

#endif
