#ifndef NET_HPP
#define NET_HPP

#include <boost/asio.hpp>

#include "utils.hpp"

using namespace boost;

enum class ipv { v4 = 0, v6 };

static constexpr ipv IPV4 = ipv::v4;
static constexpr ipv IPV6 = ipv::v6;

template<std::size_t _buffer_size>
struct packet_native_t {
public:
    static constexpr std::size_t buffer_size = _buffer_size;

    using buffer_type = noheap::buffer_bytes_type<buffer_size, std::int8_t>;

public:
    packet_native_t() = default;

    virtual buffer_type::value_type *get_buffer() {
        return reinterpret_cast<buffer_type::value_type *>(this) + 8;
    }

public:
    buffer_type buffer{};
};

template<typename T>
concept Derived_from_packet_native_t =
    std::derived_from<T, packet_native_t<T::buffer_size>>;

template<Derived_from_packet_native_t              T,
         noheap::log_impl::owner_impl::buffer_type _buffer_owner =
             noheap::log_impl::create_owner("PROTOCOL")>
struct protocol_native_t {
public:
    using packet_type        = T;
    using callback_prepare_t = std::function<void(packet_type &)>;
    using callback_handle_t  = std::function<void(packet_type &&)>;

public:
    constexpr virtual void prepare(packet_type &pckt, callback_prepare_t callback) const {
    }
    constexpr virtual void handle(packet_type &pckt, callback_handle_t callback) const {}

public:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        _buffer_owner;

protected:
    static constexpr log_handler log{buffer_owner};
};

template<typename T>
concept Derived_from_protocol_native_t =
    std::derived_from<T, protocol_native_t<typename T::packet_type, T::buffer_owner>>;

template<std::size_t _buffer_size>
struct debug_extention {
    struct packet_t : public packet_native_t<_buffer_size> {
        std::size_t mark_time;
    };

    struct protocol_t : public protocol_native_t<packet_t, noheap::log_impl::create_owner(
                                                               "DEBUG_PROTOCOL")> {
        constexpr void prepare(packet_t                      &pckt,
                               protocol_t::callback_prepare_t callback) const override {
            callback(pckt);

            pckt.mark_time = get_now_ms();
        }
        constexpr void handle(packet_t                     &pckt,
                              protocol_t::callback_handle_t callback) const override {
            static std::size_t count_accepted = 0;
            static std::size_t during         = get_now_ms();

            std::size_t now = get_now_ms();

            ++count_accepted;
            if (now - during > 1000) {
                this->log.template to_all<log_handler::output_type::async>(
                    "Was recieved last packet {} "
                    "ms({} packet/s.)",
                    now - pckt.mark_time, count_accepted);
                during         = now;
                count_accepted = 0;
            }

            callback(std::move(pckt));
        }
    };
};

template<Derived_from_packet_native_t   TPacket_internal,
         Derived_from_protocol_native_t TProtocol = protocol_native_t<TPacket_internal>>
class packet final : public TPacket_internal {
public:
    using packet_type   = TPacket_internal;
    using protocol_type = TProtocol;

public:
    static constexpr std::size_t size = sizeof(packet_type) - 8;

public:
    packet() = default;
    packet(packet_type &&pckg) : packet_type(pckg) {}

public:
    static constexpr void prepare(packet_type                      &pckt,
                                  protocol_type::callback_prepare_t callback) {
        prt.prepare(pckt, callback);
    }
    static constexpr void handle(packet_type                    &&pckt,
                                 protocol_type::callback_handle_t callback) {
        prt.handle(pckt, callback);
    }

public:
    static constexpr const protocol_type &get_protocol() { return prt; }

private:
    static constexpr protocol_type prt{};
};

template<typename T>
concept Packet =
    std::same_as<T, packet<typename T::packet_type, typename T::protocol_type>>;

template<Packet TPacket>
struct action {
public:
    using packet = TPacket;

public:
    action() = default;

public:
    constexpr virtual void init_packet(packet::packet_type &pckt)     = 0;
    constexpr virtual void process_packet(packet::packet_type &&pckt) = 0;
};

template<typename T>
concept Derived_from_action = std::derived_from<T, action<typename T::packet>>;

template<std::size_t _buffer_size>
using debug_packet = packet<typename debug_extention<_buffer_size>::packet_type,
                            typename debug_extention<_buffer_size>::protocol_type>;
template<typename TSocket>
concept Socket =
    std::same_as<TSocket, asio::ip::udp> || std::same_as<TSocket, asio::ip::tcp>;

template<Socket TSocket, ipv v>
static constexpr asio::ip::udp get_ipv() {
    if constexpr (v == ipv::v6)
        return TSocket::v6();
    else
        return TSocket::v4();
}
template<Socket TSocket, Derived_from_action Action, ipv v>
class net_stream_basic {
protected:
    using basic_socket_type = TSocket;
    using socket_type       = TSocket::socket;
    using endpoint_type     = basic_socket_type::endpoint;
    using ipv_type = std::conditional_t<static_cast<bool>(v), asio::ip::address_v6,
                                        asio::ip::address_v4>;

    enum class async_socket_operation { send_to = 0, receive_from, timer };

protected:
    net_stream_basic(asio::io_context &_io, asio::ip::port_type _port);

public:
    const Action &get_action() const { return act; }
    bool          get_running() const { return running; }
    void          set_running(bool _running) { running = _running; }

protected:
    template<async_socket_operation async_op, std::size_t delay, typename Func,
             typename TBuffer>
    void register_async_socket_operation(Func &&func, TBuffer &&buffer,
                                         endpoint_type &endpoint);

    void send_to(asio::const_buffer buffer, const endpoint_type &endpoint);
    void receive_from(asio::mutable_buffer buffer, endpoint_type &endpoint);
    void connect(const endpoint_type &endpoint);

protected:
    void handle_error(const system::error_code &ec);

protected:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NSTREAM");
    static constexpr log_handler log{buffer_owner};

protected:
    asio::io_context   &io;
    asio::ip::port_type port;
    bool                running;

    Action act;

private:
    socket_type socket;
};
template<Socket TSocket, Derived_from_action Action, ipv v>
net_stream_basic<TSocket, Action, v>::net_stream_basic(asio::io_context   &_io,
                                                       asio::ip::port_type _port)
    : io(_io), port(_port), running(true), socket(io) {
    boost::system::error_code ec;

    socket.open(get_ipv<asio::ip::udp, v>(), ec);
    if (ec.value())
        throw noheap::runtime_error(buffer_owner, "Failed to open udp socket: {}.",
                                    ec.message());

    socket.bind({get_ipv<asio::ip::udp, v>(), port}, ec);
    if (ec.value())
        throw noheap::runtime_error(buffer_owner, "Failed to bind udp socket: {}.",
                                    ec.message());

    socket.set_option(asio::socket_base::reuse_address(true));
}
template<Socket TSocket, Derived_from_action Action, ipv v>
template<net_stream_basic<TSocket, Action, v>::async_socket_operation async_op,
         std::size_t delay, typename Func, typename TBuffer>
void net_stream_basic<TSocket, Action, v>::register_async_socket_operation(
    Func &&func, TBuffer &&buffer, endpoint_type &endpoint) {
    if (!running)
        return;

    if constexpr (async_op == async_socket_operation::send_to)
        socket.async_send_to(
            std::forward<TBuffer>(buffer), endpoint,
            std::bind(std::forward<Func>(func), asio::placeholders::error));
    else if constexpr (async_op == async_socket_operation::receive_from)
        socket.async_receive_from(
            std::forward<TBuffer>(buffer), endpoint,
            std::bind(std::forward<Func>(func), asio::placeholders::error));
    else {
        thread_local asio::steady_timer t(this->io);

        t.expires_after(std::chrono::milliseconds(delay));
        t.async_wait(std::bind(std::forward<Func>(func), asio::placeholders::error));
    }
}

template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::send_to(asio::const_buffer   buffer,
                                                   const endpoint_type &endpoint) {
    if (!running)
        return;

    boost::system::error_code ec;
    socket.send_to(buffer, endpoint, 0, ec);

    this->handle_error(ec);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::receive_from(asio::mutable_buffer buffer,
                                                        endpoint_type       &endpoint) {
    if (!running)
        return;

    boost::system::error_code ec;
    socket.reveice_from(buffer, endpoint, 0, ec);

    this->handle_error(ec);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::connect(const endpoint_type &endpoint) {
    if (!running)
        return;

    socket->connect(endpoint);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::handle_error(const system::error_code &ec) {
    if (!ec.value())
        return;

    throw noheap::runtime_error(buffer_owner, "Network error: {}", ec.message());
}

template<Derived_from_action Action, ipv v = ipv::v4>
class net_stream_udp final : public net_stream_basic<asio::ip::udp, Action, v> {
public:
    using socket_type            = net_stream_udp::socket_type;
    using ipv_type               = net_stream_udp::ipv_type;
    using endpoint_type          = net_stream_udp::endpoint_type;
    using async_socket_operation = net_stream_udp::async_socket_operation;

public:
    net_stream_udp(asio::io_context &_io, asio::ip::port_type _port);

public:
    template<std::size_t delay, Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void register_send_handler(TPacket &pckt, const ipv_type &addr);

    template<Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void register_receive_handler(TPacket &pckt);
};

template<Derived_from_action Action, ipv v>
net_stream_udp<Action, v>::net_stream_udp(asio::io_context   &_io,
                                          asio::ip::port_type _port)
    : net_stream_basic<asio::ip::udp, Action, v>(_io, _port) {
}

template<Derived_from_action Action, ipv v>
template<std::size_t delay, Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_udp<Action, v>::register_send_handler(TPacket        &pckt,
                                                      const ipv_type &addr) {
    thread_local asio::ip::udp::endpoint receiver_endpoint{addr, this->port};
    thread_local const auto do_send = [this, &pckt, &addr](system::error_code ec) {
        TPacket::prepare(
            pckt, std::bind(&Action::init_packet, &this->act, std::placeholders::_1));
        this->send_to(asio::buffer(pckt.get_buffer(), TPacket::size), receiver_endpoint);
        this->register_send_handler<delay>(pckt, addr);
    };

    this->template register_async_socket_operation<async_socket_operation::timer, delay>(
        do_send, asio::const_buffer{}, receiver_endpoint);
}

template<Derived_from_action Action, ipv v>
template<Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_udp<Action, v>::register_receive_handler(TPacket &pckt) {
    thread_local asio::ip::udp::endpoint sender_endpoint{};
    thread_local const auto handle_receive = [this, &pckt](const system::error_code &ec) {
        this->handle_error(ec);
        TPacket::handle(std::move(pckt), std::bind(&Action::process_packet, &this->act,
                                                   std::placeholders::_1));
        this->register_receive_handler(pckt);
    };
    this->template register_async_socket_operation<async_socket_operation::receive_from,
                                                   0>(
        handle_receive, boost::asio::buffer(pckt.get_buffer(), TPacket::size),
        sender_endpoint);
}

template<Derived_from_action Action, ipv v = ipv::v4>
class net_stream_tcp final : public net_stream_basic<asio::ip::tcp, Action, v> {
public:
    using socket_type = net_stream_tcp::socket_type;
    using ipv_type    = net_stream_tcp::ipv_type;

public:
    net_stream_tcp(socket_type &&_socket);
    net_stream_tcp(asio::io_context &_io, const ipv_type &addr,
                   asio::ip::port_type _port);

public:
    template<Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void send(TPacket &pckt);

    template<Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void receive(TPacket &pckt);
};
template<Derived_from_action Action, ipv v>
net_stream_tcp<Action, v>::net_stream_tcp(socket_type &&_socket)
    : net_stream_basic<asio::ip::tcp, Action, v>(_socket.get_executor(), 0) {
    socket = std::move(_socket);
}
template<Derived_from_action Action, ipv v>
net_stream_tcp<Action, v>::net_stream_tcp(asio::io_context &_io, const ipv_type &addr,
                                          asio::ip::port_type _port)
    : net_stream_basic<asio::ip::tcp, Action, v>(_io, _port) {
    this->connect(asio::ip::tcp::endpoint{addr, this->port});
}

template<Derived_from_action Action, ipv v>
template<Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_tcp<Action, v>::send(TPacket &pckt) {
    thread_local const auto handle_send = [this, &pckt](const system::error_code &ec) {
        this->handle_error(ec);
    };

    TPacket::prepare(pckt,
                     std::bind(&Action::init_packet, &this->act, std::placeholders::_1));

    this->send_to(boost::asio::buffer(pckt.get_buffer(), TPacket::size), {}, handle_send);
}

template<Derived_from_action Action, ipv v>
template<Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_tcp<Action, v>::receive(TPacket &pckt) {
    typename socket_type::endpoint endpoint_sender;
    thread_local const auto handle_receive = [this, &pckt](const system::error_code &ec) {
        this->handle_error(ec);
        TPacket::handle(std::move(pckt), std::bind(&Action::process_packet, &this->act,
                                                   std::placeholders::_1));
    };

    this->receive_from(boost::asio::buffer(pckt.get_buffer(), TPacket::size),
                       endpoint_sender, handle_receive);
}

#endif
