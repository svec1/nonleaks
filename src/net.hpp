#ifndef NET_HPP
#define NET_HPP

#include <boost/asio.hpp>

#include "utils.hpp"

using namespace boost;

enum class ipv { v4 = 0, v6 };

static constexpr ipv IPV4 = ipv::v4;
static constexpr ipv IPV6 = ipv::v6;

template <std::size_t _buffer_size> struct packet_native_t {
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

template <typename T>
concept Derived_from_packet_native_t =
    std::derived_from<T, packet_native_t<T::buffer_size>>;

template <Derived_from_packet_native_t T,
          noheap::log_impl::owner_impl::buffer_type _buffer_owner =
              noheap::log_impl::create_owner("PROTOCOL")>
struct protocol_native_t {
  public:
    using packet_type = T;
    using callback_prepare_t = std::function<void(packet_type &)>;
    using callback_handle_t = std::function<void(packet_type &&)>;

  public:
    constexpr virtual void prepare(packet_type &pckt,
                                   callback_prepare_t callback) const {}
    constexpr virtual void handle(packet_type &pckt,
                                  callback_handle_t callback) const {}

  public:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        _buffer_owner;

  protected:
    static constexpr log_handler log{buffer_owner};
};

template <typename T>
concept Derived_from_protocol_native_t = std::derived_from<
    T, protocol_native_t<typename T::packet_type, T::buffer_owner>>;

template <std::size_t _buffer_size> struct debug_extention {
    struct packet_t : public packet_native_t<_buffer_size> {
        std::size_t mark_time;
    };

    struct protocol_t
        : public protocol_native_t<packet_t, noheap::log_impl::create_owner(
                                                 "DEBUG_PROTOCOL")> {
        constexpr void
        prepare(packet_t &pckt,
                protocol_t::callback_prepare_t callback) const override {
            callback(pckt);

            pckt.mark_time = get_now_ms();
        }
        constexpr void
        handle(packet_t &pckt,
               protocol_t::callback_handle_t callback) const override {
            static unsigned long long count_accepted = 0;
            static unsigned long long during = get_now_ms();

            unsigned long long now = get_now_ms();

            ++count_accepted;
            if (now - during > 1000) {
                this->log.template to_all<log_handler::output_type::async>(
                    "Was recieved last packet {} "
                    "ms({} packet/s.)",
                    now - pckt.mark_time, count_accepted);
                during = now;
                count_accepted = 0;
            }

            callback(std::move(pckt));
        }
    };
};

template <Derived_from_packet_native_t TPacket_internal,
          Derived_from_protocol_native_t TProtocol =
              protocol_native_t<TPacket_internal>>
class packet final : public TPacket_internal {
  public:
    using packet_type = TPacket_internal;
    using protocol_type = TProtocol;

  public:
    static constexpr std::size_t size = sizeof(packet_type) - 8;

  public:
    packet() = default;
    packet(packet_type &&pckg) : packet_type(pckg) {}

  public:
    static constexpr void prepare(packet_type &pckt,
                                  protocol_type::callback_prepare_t callback) {
        prt.prepare(pckt, callback);
    }
    static constexpr void handle(packet_type &&pckt,
                                 protocol_type::callback_handle_t callback) {
        prt.handle(pckt, callback);
    }

  private:
    static constexpr protocol_type prt{};
};

template <typename T>
concept Packet =
    std::same_as<T, packet<typename T::packet_type, typename T::protocol_type>>;

template <Packet TPacket> struct action {
  public:
    using packet = TPacket;

  public:
    action() = default;

  public:
    constexpr virtual void init_packet(packet::packet_type &pckt) = 0;
    constexpr virtual void process_packet(packet::packet_type &&pckt) = 0;
};

template <typename T>
concept Derived_from_action = std::derived_from<T, action<typename T::packet>>;

template <std::size_t _buffer_size>
using debug_packet =
    packet<typename debug_extention<_buffer_size>::packet_type,
           typename debug_extention<_buffer_size>::protocol_type>;
template <typename TSocket>
concept Socket = std::same_as<TSocket, asio::ip::udp> ||
                 std::same_as<TSocket, asio::ip::tcp>;

template <Socket TSocket, ipv v> static constexpr asio::ip::udp get_ipv() {
    if constexpr (v == ipv::v6)
        return TSocket::v6();
    else
        return TSocket::v4();
}
template <Socket TSocket, Derived_from_action Action, ipv v>
class net_stream_basic {
  protected:
    using socket_t = TSocket::socket;
    using ipv_t = std::conditional_t<static_cast<bool>(v), asio::ip::address_v6,
                                     asio::ip::address_v4>;

  protected:
    net_stream_basic(asio::io_context &_io, asio::ip::port_type _port);

  protected:
    void handle_error(const system::error_code &ec);

  protected:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NSTREAM");
    static constexpr log_handler log{buffer_owner};

  protected:
    asio::io_context &io;
    asio::ip::port_type port;

    socket_t socket;
    Action act;
};
template <Socket TSocket, Derived_from_action Action, ipv v>
net_stream_basic<TSocket, Action, v>::net_stream_basic(
    asio::io_context &_io, asio::ip::port_type _port)
    : io(_io), port(_port), socket(io) {
    thread_local boost::system::error_code ec;

    socket.open(get_ipv<asio::ip::udp, v>(), ec);
    if (ec.value())
        throw noheap::runtime_error(
            buffer_owner, "Failed to open udp socket: {}.", ec.message());

    socket.bind({get_ipv<asio::ip::udp, v>(), port}, ec);
    if (ec.value())
        throw noheap::runtime_error(
            buffer_owner, "Failed to bind udp socket: {}.", ec.message());

    socket.set_option(asio::socket_base::reuse_address(true));
}
template <Socket TSocket, Derived_from_action Action, ipv v>
void net_stream_basic<TSocket, Action, v>::handle_error(
    const system::error_code &ec) {
    if (!ec.value())
        return;

    noheap::runtime_error exception(this->buffer_owner, "Network error: {}",
                                    ec.message());
}

template <Derived_from_action Action, ipv v = ipv::v4>
class net_stream_udp final : public net_stream_basic<asio::ip::udp, Action, v> {
  public:
    using ipv_t = net_stream_udp::ipv_t;

  public:
    net_stream_udp(asio::io_context &_io, asio::ip::port_type _port);

  public:
    template <std::size_t delay, Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void register_send_handler(TPacket &pckt, const ipv_t &addr);

    template <Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void register_receive_handler(TPacket &pckt);
};

template <Derived_from_action Action, ipv v>
net_stream_udp<Action, v>::net_stream_udp(asio::io_context &_io,
                                          asio::ip::port_type _port)
    : net_stream_basic<asio::ip::udp, Action, v>(_io, _port) {}

template <Derived_from_action Action, ipv v>
template <std::size_t delay, Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_udp<Action, v>::register_send_handler(TPacket &pckt,
                                                      const ipv_t &addr) {
    thread_local asio::steady_timer t(this->io);
    thread_local const auto do_send = [this, &pckt,
                                       &addr](system::error_code ec) {
        TPacket::prepare(pckt, std::bind(&Action::init_packet, &this->act,
                                         std::placeholders::_1));
        std::size_t size = this->socket.send_to(
            asio::buffer(pckt.get_buffer(), TPacket::size),
            asio::ip::udp::endpoint{addr, this->port}, 0, ec);

        this->handle_error(ec);
        if (size != TPacket::size)
            throw noheap::runtime_error(
                this->buffer_owner, "An incomplete package was sent: {} bytes.",
                size);

        this->register_send_handler<delay>(pckt, addr);
    };

    t.expires_after(std::chrono::milliseconds(delay));
    t.async_wait(std::bind(do_send, asio::placeholders::error));
}

template <Derived_from_action Action, ipv v>
template <Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_udp<Action, v>::register_receive_handler(TPacket &pckt) {
    thread_local asio::ip::udp::endpoint sender_endpoint;
    thread_local const auto handle_receive =
        [this, &pckt](const system::error_code &ec, std::size_t size) {
            this->handle_error(ec);
            if (size != TPacket::size)
                noheap::runtime_error(
                    this->buffer_owner,
                    "An incomplete package was receive: {} bytes.", size);

            TPacket::handle(std::move(pckt),
                            std::bind(&Action::process_packet, &this->act,
                                      std::placeholders::_1));
            this->register_receive_handler(pckt);
        };

    this->socket.async_receive_from(
        boost::asio::buffer(pckt.get_buffer(), TPacket::size), sender_endpoint,
        handle_receive);
}

template <Derived_from_action Action, ipv v = ipv::v4>
class net_stream_tcp final : public net_stream_basic<asio::ip::tcp, Action, v> {
  public:
    using socket_t = net_stream_tcp::socket_t;
    using ipv_t = net_stream_tcp::ipv_t;

  public:
    net_stream_tcp(socket_t &&_socket);
    net_stream_tcp(asio::io_context &_io, const ipv_t &addr,
                   asio::ip::port_type _port);

  public:
    template <Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void send(TPacket &pckt);

    template <Packet TPacket>
        requires std::same_as<typename decltype(Action{})::packet, TPacket>
    void receive(TPacket &pckt);
};
template <Derived_from_action Action, ipv v>
net_stream_tcp<Action, v>::net_stream_tcp(socket_t &&_socket)
    : net_stream_basic<asio::ip::tcp, Action, v>(_socket.get_executor(), 0) {
    socket = std::move(_socket);
}
template <Derived_from_action Action, ipv v>
net_stream_tcp<Action, v>::net_stream_tcp(asio::io_context &_io,
                                          const ipv_t &addr,
                                          asio::ip::port_type _port)
    : net_stream_basic<asio::ip::tcp, Action, v>(_io, _port) {
    this->socket.connect(asio::ip::tcp::endpoint{addr, this->port});
}

template <Derived_from_action Action, ipv v>
template <Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_tcp<Action, v>::send(TPacket &pckt) {
    thread_local const auto handle_send =
        [this, &pckt](const system::error_code &ec) { this->handle_error(ec); };

    TPacket::prepare(pckt, std::bind(&Action::init_packet, &this->act,
                                     std::placeholders::_1));

    this->socket.async_send(asio::buffer(pckt.get_buffer(), TPacket::size),
                            handle_send);
}

template <Derived_from_action Action, ipv v>
template <Packet TPacket>
    requires std::same_as<typename decltype(Action{})::packet, TPacket>
void net_stream_tcp<Action, v>::receive(TPacket &pckt) {
    thread_local const auto handle_receive =
        [this, &pckt](const system::error_code &ec) {
            this->handle_error(ec);
            TPacket::handle(std::move(pckt),
                            std::bind(&Action::process_packet, &this->act,
                                      std::placeholders::_1));
        };

    this->socket.async_receive(asio::buffer(pckt.get_buffer(), TPacket::size),
                               handle_receive);
}

#endif
