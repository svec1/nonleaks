#ifndef NET_HPP
#define NET_HPP

#include <boost/asio.hpp>

#include "utils.hpp"

using namespace boost;

enum class ipv { v4 = 0, v6 };

static constexpr ipv IPV4 = ipv::v4;
static constexpr ipv IPV6 = ipv::v6;

static constexpr std::size_t max_buffer_address_size =
    asio::ip::address_v6::bytes_type{}.size();

using buffer_address_type = asio::ip::address_v6::bytes_type;

template<typename T, std::size_t ad_size>
struct packet_native_type;
template<typename T>
concept Packet_native_t =
    std::same_as<T, packet_native_type<typename T::extention_data_type, T{}.ad_size()>>;

template<Packet_native_t T, noheap::log_impl::owner_impl::buffer_type _buffer_owner =
                                noheap::log_impl::create_owner("PROTOCOL")>
struct protocol_native_type;
template<typename T>
concept Derived_from_protocol_native_t =
    std::derived_from<T, protocol_native_type<typename T::packet_type, T::buffer_owner>>;

template<Packet_native_t TPacket>
struct action;
template<typename T>
concept Derived_from_action = std::derived_from<T, action<typename T::packet_type>>;

template<typename T, std::size_t _ad_size>
struct packet_native_type {
public:
    using extention_data_type = T;
    using ad_type             = noheap::buffer_bytes_type<_ad_size>;
    using represent_type      = std::int8_t;

public:
    packet_native_type() = default;

    extention_data_type *operator->() noexcept { return _extention_data_p; }

public:
    constexpr std::size_t size() const noexcept {
        return sizeof(*this) - sizeof(_extention_data_p);
    }
    constexpr std::size_t ad_size() const noexcept { return _ad_size; }
    constexpr std::size_t extention_size() const noexcept {
        return sizeof(extention_data_type);
    }

public:
    constexpr represent_type *data() noexcept {
        return reinterpret_cast<represent_type *>(this);
    }
    constexpr represent_type *extention_data() noexcept {
        return reinterpret_cast<represent_type *>(&_extention_data);
    }

public:
    ad_type payload_ad;

private:
    extention_data_type  _extention_data;
    extention_data_type *_extention_data_p = &_extention_data;
};

template<Packet_native_t TPacket>
struct action {
public:
    using packet_type         = TPacket;
    using init_packet_type    = std::function<void(packet_type &)>;
    using process_packet_type = std::function<void(packet_type &&)>;

public:
    action() = default;

public:
    constexpr virtual void init_packet(packet_type &pckt)     = 0;
    constexpr virtual void process_packet(packet_type &&pckt) = 0;
};

template<Packet_native_t T, noheap::log_impl::owner_impl::buffer_type _buffer_owner>
struct protocol_native_type {
public:
    using packet_type           = T;
    using action_type           = action<packet_type>;
    using callback_prepare_type = action_type::init_packet_type;
    using callback_handle_type  = action_type::process_packet_type;

public:
    constexpr virtual void prepare(packet_type &pckt, buffer_address_type addr,
                                   callback_prepare_type callback) const {}
    constexpr virtual void handle(packet_type &pckt, buffer_address_type addr,
                                  callback_handle_type callback) const {}

public:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        _buffer_owner;

protected:
    static constexpr log_handler log{buffer_owner};
};

struct debug_extention {
    struct extention_data_type {
        std::size_t mark_time;
    };

public:
    using packet_type = packet_native_type<extention_data_type, 0>;

    struct protocol_type
        : public protocol_native_type<packet_type,
                                      noheap::log_impl::create_owner("DEBUG_PROTOCOL")> {
        constexpr void prepare(packet_type &pckt, buffer_address_type addr,
                               callback_prepare_type callback) const override {
            callback(pckt);

            pckt->mark_time = get_now_ms();
        }
        constexpr void handle(packet_type &pckt, buffer_address_type addr,
                              callback_handle_type callback) const override {
            static std::size_t count_accepted = 0;
            static std::size_t during         = get_now_ms();

            std::size_t now = get_now_ms();

            ++count_accepted;
            if (now - during > 1000) {
                this->log.template to_all<log_handler::output_type::async>(
                    "Was recieved last packet {} "
                    "ms({} packet/s.)",
                    now - pckt->mark_time, count_accepted);
                during         = now;
                count_accepted = 0;
            }

            callback(std::move(pckt));
        }
    };
};

template<Packet_native_t TPacket_internal, Derived_from_protocol_native_t TProtocol =
                                               protocol_native_type<TPacket_internal>>
class packet final : public TPacket_internal {
public:
    using packet_type   = TPacket_internal;
    using protocol_type = TProtocol;

public:
    packet() = default;
    packet(packet_type &&pckg) : packet_type(pckg) {}

public:
    static constexpr void prepare(packet_type &pckt, buffer_address_type addr,
                                  protocol_type::callback_prepare_type callback) {
        prt.prepare(pckt, addr, callback);
    }
    static constexpr void handle(packet_type &&pckt, buffer_address_type addr,
                                 protocol_type::callback_handle_type callback) {
        prt.handle(pckt, addr, callback);
    }

public:
    static constexpr const protocol_type &get_protocol() { return prt; }

private:
    static constexpr protocol_type prt{};
};

template<typename T>
concept Packet =
    std::same_as<T, packet<typename T::packet_type, typename T::protocol_type>>;

using debug_packet = packet<typename debug_extention::packet_type,
                            typename debug_extention::protocol_type>;
template<typename TSocket>
concept Socket =
    std::same_as<TSocket, asio::ip::udp> || std::same_as<TSocket, asio::ip::tcp>;

template<Socket TSocket, Derived_from_action Action, ipv _v>
class net_stream_basic;
template<Derived_from_action Action, ipv _v>
class net_stream_tcp;
template<Derived_from_action Action, ipv _v>
class net_stream_udp;

template<typename TStream_tcp>
class acceptor;

template<Socket TSocket, Derived_from_action Action, ipv _v>
class net_stream_basic {
protected:
    static constexpr TSocket get_ipv() {
        if constexpr (v == ipv::v6)
            return TSocket::v6();
        else
            return TSocket::v4();
    }

public:
    static constexpr ipv v = _v;

protected:
    using basic_socket_type = TSocket;
    using socket_type       = TSocket::socket;
    using action_type       = Action;
    using acceptor_type     = acceptor<net_stream_tcp<Action, _v>>;
    using endpoint_type     = basic_socket_type::endpoint;
    using address_type = std::conditional_t<static_cast<bool>(v), asio::ip::address_v6,
                                            asio::ip::address_v4>;

    enum class async_socket_operation { send_to = 0, receive_from, connect, timer };

protected:
    friend acceptor_type;

protected:
    net_stream_basic(net_stream_basic &&)                 = default;
    net_stream_basic(const net_stream_basic &)            = delete;
    net_stream_basic &operator=(const net_stream_basic &) = delete;

    net_stream_basic(asio::io_context &_io);
    net_stream_basic(asio::io_context &_io, asio::ip::port_type _port);

protected:
    static void init_socket(net_stream_basic<TSocket, Action, v> &stream,
                            asio::ip::port_type                   port);

    buffer_address_type get_address_bytes(asio::ip::address addr);

public:
    void open(asio::ip::port_type _port);
    void close();

public:
    bool is_open() const { return socket.is_open(); }

    const Action &get_action() const { return act; }
    bool          get_running() const { return running; }
    void          set_running(bool _running) { running = _running; }

    std::size_t io_context_run();

protected:
    template<async_socket_operation async_op, std::size_t delay, typename Func,
             typename TBuffer>
    void register_async_socket_operation(Func &&func, TBuffer &&buffer,
                                         endpoint_type &endpoint);

    void send_to(asio::const_buffer buffer, const endpoint_type &endpoint);
    void receive_from(asio::mutable_buffer buffer, endpoint_type &endpoint);
    void connect(const endpoint_type &endpoint, system::error_code &ec);

    address_type get_remote_address() const;

protected:
    static void handle_error(const system::error_code &ec);

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
net_stream_basic<TSocket, Action, v>::net_stream_basic(asio::io_context &_io)
    : io(_io), port(0), running(true), socket(io) {
}
template<Socket TSocket, Derived_from_action Action, ipv v>
net_stream_basic<TSocket, Action, v>::net_stream_basic(asio::io_context   &_io,
                                                       asio::ip::port_type _port)
    : io(_io), port(_port), running(true), socket(io) {
    init_socket(*this, port);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::init_socket(
    net_stream_basic<TSocket, Action, v> &stream, asio::ip::port_type port) {
    system::error_code ec;

    stream.socket.close();
    stream.socket.open(get_ipv(), ec);
    if (ec.value())
        handle_error(ec);

    stream.socket.set_option(typename socket_type::reuse_address(true));
    stream.socket.set_option(typename socket_type::broadcast(false));

    stream.socket.bind({get_ipv(), port}, ec);
    if (ec.value() && ec != asio::error::address_in_use)
        handle_error(ec);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
buffer_address_type
    net_stream_basic<TSocket, Action, v>::get_address_bytes(asio::ip::address addr) {
    if constexpr (std::same_as<address_type, asio::ip::address_v4>) {
        buffer_address_type buffer;

        auto addr_uint = addr.to_v4().to_uint();
        std::copy(buffer.begin(), buffer.begin() + sizeof(addr_uint), &addr_uint);

        return buffer;
    } else
        return addr.to_v6().to_bytes();
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::open(asio::ip::port_type _port) {
    if (socket.is_open())
        socket.close();

    port = _port;

    init_socket(*this, port);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::close() {
    socket.close();
}

template<Socket TSocket, Derived_from_action Action, ipv v>
std::size_t net_stream_basic<TSocket, Action, v>::io_context_run() {
    return io.run();
}

template<Socket TSocket, Derived_from_action Action, ipv v>
template<net_stream_basic<TSocket, Action, v>::async_socket_operation async_op,
         std::size_t delay, typename Func, typename TBuffer>
void net_stream_basic<TSocket, Action, v>::register_async_socket_operation(
    Func &&func, TBuffer &&buffer, endpoint_type &endpoint) {
    if (!running)
        return;

    const auto async_operation_handler = [&](const system::error_code &ec) {
        if constexpr (std::invocable<Func, system::error_code>)
            func(ec);
        else {
            handle_error(ec);
            func();
        }
    };
    const auto handler = std::bind(async_operation_handler, asio::placeholders::error);

    if constexpr (async_op == async_socket_operation::send_to)
        socket.async_send_to(std::forward<TBuffer>(buffer), endpoint, handler);
    else if constexpr (async_op == async_socket_operation::receive_from)
        socket.async_receive_from(std::forward<TBuffer>(buffer), endpoint, handler);
    else if constexpr (async_op == async_socket_operation::connect)
        socket.async_connect(endpoint, handler);
    else if constexpr (async_op == async_socket_operation::timer) {
        thread_local asio::steady_timer t(this->io);

        t.expires_after(std::chrono::milliseconds(delay));
        t.async_wait(handler);
    } else
        static_assert(false, "Unknown async operation.");
}

template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::send_to(asio::const_buffer   buffer,
                                                   const endpoint_type &endpoint) {
    if (!running)
        return;

    system::error_code ec;
    if constexpr (std::same_as<basic_socket_type, asio::ip::udp>)
        socket.send_to(buffer, endpoint, 0, ec);
    else
        socket.send(buffer, 0, ec);

    this->handle_error(ec);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::receive_from(asio::mutable_buffer buffer,
                                                        endpoint_type       &endpoint) {
    if (!running)
        return;

    system::error_code ec;
    if constexpr (std::same_as<basic_socket_type, asio::ip::udp>)
        socket.receive_from(buffer, endpoint, 0, ec);
    else
        socket.receive(buffer, 0, ec);

    this->handle_error(ec);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::connect(const endpoint_type &endpoint,
                                                   system::error_code  &ec) {
    if (!running)
        return;

    socket.connect(endpoint, ec);
}
template<Socket TSocket, Derived_from_action Action, ipv v>
net_stream_basic<TSocket, Action, v>::address_type
    net_stream_basic<TSocket, Action, v>::get_remote_address() const {
    address_type       addr;
    system::error_code ec;

    addr = socket.remote_endpoint(ec).address().to_v4();
    if (ec)
        this->handle_error(ec);

    return addr;
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
    using basic_socket_type      = net_stream_udp::basic_socket_type;
    using socket_type            = net_stream_udp::socket_type;
    using action_type            = net_stream_udp::action_type;
    using address_type           = net_stream_udp::address_type;
    using endpoint_type          = net_stream_udp::endpoint_type;
    using async_socket_operation = net_stream_udp::async_socket_operation;

public:
    net_stream_udp(asio::io_context &_io, asio::ip::port_type _port);

public:
    template<std::size_t delay, Packet TPacket>
        requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
    void register_send_handler(TPacket &pckt, const address_type &addr);

    template<Packet TPacket>
        requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
    void register_receive_handler(TPacket &pckt);
};

template<Derived_from_action Action, ipv v>
net_stream_udp<Action, v>::net_stream_udp(asio::io_context   &_io,
                                          asio::ip::port_type _port)
    : net_stream_basic<asio::ip::udp, Action, v>(_io, _port) {
}

template<Derived_from_action Action, ipv v>
template<std::size_t delay, Packet TPacket>
    requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
void net_stream_udp<Action, v>::register_send_handler(TPacket            &pckt,
                                                      const address_type &addr) {
    thread_local asio::ip::udp::endpoint receiver_endpoint{addr, this->port};

    thread_local const auto do_send = [this, &pckt, &addr]() {
        TPacket::prepare(
            pckt, this->get_address_bytes(receiver_endpoint.address()),
            std::bind(&Action::init_packet, &this->act, std::placeholders::_1));
        this->send_to(asio::buffer(pckt.data(), pckt.size()), receiver_endpoint);
        this->register_send_handler<delay>(pckt, addr);
    };

    this->template register_async_socket_operation<async_socket_operation::timer, delay>(
        do_send, asio::const_buffer{}, receiver_endpoint);
}

template<Derived_from_action Action, ipv v>
template<Packet TPacket>
    requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
void net_stream_udp<Action, v>::register_receive_handler(TPacket &pckt) {
    thread_local asio::ip::udp::endpoint sender_endpoint{};
    thread_local const auto              handle_receive = [this, &pckt]() {
        TPacket::handle(
            std::move(pckt), this->get_address_bytes(sender_endpoint.address()),
            std::bind(&Action::process_packet, &this->act, std::placeholders::_1));
        this->register_receive_handler(pckt);
    };
    this->template register_async_socket_operation<async_socket_operation::receive_from,
                                                   0>(
        handle_receive, boost::asio::buffer(pckt.data(), pckt.size()), sender_endpoint);
}

template<Derived_from_action Action, ipv v = ipv::v4>
class net_stream_tcp final : public net_stream_basic<asio::ip::tcp, Action, v> {
public:
    using basic_socket_type      = net_stream_tcp::basic_socket_type;
    using socket_type            = net_stream_tcp::socket_type;
    using action_type            = net_stream_tcp::action_type;
    using acceptor_type          = net_stream_tcp::acceptor_type;
    using address_type           = net_stream_tcp::address_type;
    using endpoint_type          = net_stream_tcp::endpoint_type;
    using async_socket_operation = net_stream_tcp::async_socket_operation;

public:
    net_stream_tcp(asio::io_context &_io);
    net_stream_tcp(asio::io_context &_io, asio::ip::port_type _port);

public:
    template<Packet TPacket>
        requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
    void send(TPacket &pckt);

    template<Packet TPacket>
        requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
    void receive(TPacket &pckt);

    bool wait_connect(const endpoint_type &endpoint);

    template<typename Func>
    void async_connect(const endpoint_type &endpoint, Func &&func);
};
template<Derived_from_action Action, ipv v>
net_stream_tcp<Action, v>::net_stream_tcp(asio::io_context   &_io,
                                          asio::ip::port_type _port)
    : net_stream_basic<asio::ip::tcp, Action, v>(_io, _port) {
}

template<Derived_from_action Action, ipv v>
template<Packet TPacket>
    requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
void net_stream_tcp<Action, v>::send(TPacket &pckt) {
    TPacket::prepare(pckt, this->get_address_bytes(this->get_remote_address()),
                     std::bind(&Action::init_packet, &this->act, std::placeholders::_1));

    this->send_to(boost::asio::buffer(pckt.data(), pckt.size()), {});
}

template<Derived_from_action Action, ipv v>
template<Packet TPacket>
    requires std::same_as<typename Action::packet_type, typename TPacket::packet_type>
void net_stream_tcp<Action, v>::receive(TPacket &pckt) {
    endpoint_type endpoint_sender;

    this->receive_from(boost::asio::buffer(pckt.data(), pckt.size()), endpoint_sender);
    TPacket::handle(
        std::move(pckt), this->get_address_bytes(this->get_remote_address()),
        std::bind(&Action::process_packet, &this->act, std::placeholders::_1));
}
template<Derived_from_action Action, ipv v>
bool net_stream_tcp<Action, v>::wait_connect(const endpoint_type &endpoint) {
    system::error_code ec;

    this->connect(endpoint, ec);

    if (ec == system::errc::success)
        return true;
    return false;
}
template<Derived_from_action Action, ipv v>
template<typename Func>
void net_stream_tcp<Action, v>::async_connect(const endpoint_type &endpoint,
                                              Func               &&func) {
    this->template register_async_socket_operation<async_socket_operation::connect, 0>(
        std::forward<Func>(func), asio::const_buffer{}, endpoint);
}

template<typename TStream_tcp>
class acceptor final {
public:
    using stream_type = TStream_tcp;

public:
    acceptor(asio::io_context &_io, asio::ip::port_type _port);

public:
    void accept(stream_type &stream);
    template<typename Func>
    void async_accept(stream_type &stream, Func &&func);

private:
    asio::io_context       &io;
    asio::ip::port_type     port;
    asio::ip::tcp::acceptor acceptor_handle;
};
template<typename TStream_tcp>
acceptor<TStream_tcp>::acceptor(asio::io_context &_io, asio::ip::port_type _port)
    : io(_io), port(_port), acceptor_handle(io) {
    system::error_code ec;

    acceptor_handle.open(stream_type::get_ipv(), ec);
    if (ec)
        stream_type::handle_error(ec);

    acceptor_handle.set_option(typename stream_type::socket_type::reuse_address(true));

    acceptor_handle.bind({stream_type::get_ipv(), port}, ec);
    if (ec)
        stream_type::handle_error(ec);
}
template<typename TStream_tcp>
void acceptor<TStream_tcp>::accept(stream_type &stream) {
    system::error_code ec;

    acceptor_handle.listen();
    acceptor_handle.accept(stream.socket, ec);

    if (ec)
        stream_type::handle_error(ec);
}

template<typename TStream_tcp>
template<typename Func>
void acceptor<TStream_tcp>::async_accept(stream_type &stream, Func &&func) {
    acceptor_handle.async_accept(stream.socket, std::forward<Func>(func));
}

#endif
