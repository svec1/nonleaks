#ifndef NET_HPP
#define NET_HPP

#include <boost/asio.hpp>

#include "utils.hpp"

namespace network {

using namespace boost;

enum class ipv { v4 = 0, v6 };

constexpr std::size_t max_count_addresses     = 4;
constexpr std::size_t max_buffered_packets    = 16;
constexpr std::size_t timeout_ms              = 2500;
constexpr std::size_t max_buffer_address_size = asio::ip::address_v6::bytes_type{}.size();

using buffer_address_type = asio::ip::address_v6::bytes_type;

using buffer_address_v4_type = noheap::buffer_type<buffer_address_type::value_type, 4>;
using buffer_address_v6_type = noheap::buffer_type<buffer_address_type::value_type, 8>;

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

template<Packet_native_t T, noheap::log_impl::owner_impl::buffer_type _buffer_owner>
struct protocol_native_type;
template<typename T>
concept Derived_from_protocol_native_t =
    std::derived_from<std::decay_t<T>,
                      protocol_native_type<typename std::decay_t<T>::packet_type,
                                           std::decay_t<T>::buffer_owner>>;

template<Packet_native_t TPacket>
struct action;
template<Packet_native_t TPacket>
struct decoy_action;
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
    void init_packet(packet_type &pckt);
    void process_packet(packet_type &&pckt);
};

template<Packet_native_t TPacket>
struct decoy_action : public action<TPacket> {
    using packet_type = decoy_action::packet_type;

public:
    void init_packet(packet_type &) {}
    void process_packet(packet_type &&) {}
};

template<Packet_native_t T, noheap::log_impl::owner_impl::buffer_type _buffer_owner>
struct protocol_native_type {
public:
    using packet_type           = T;
    using action_type           = action<packet_type>;
    using callback_prepare_type = action_type::init_packet_type;
    using callback_handle_type  = action_type::process_packet_type;

public:
    void prepare(packet_type &pckt, buffer_address_type addr,
                 callback_prepare_type callback) const;
    void handle(packet_type &pckt, buffer_address_type addr,
                callback_handle_type callback) const;

public:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        _buffer_owner;

protected:
    static constexpr log_handler log{buffer_owner};
};

template<Packet_native_t TPacket_internal, Derived_from_protocol_native_t TProtocol>
    requires std::same_as<TPacket_internal, typename TProtocol::packet_type>
class wrapper_packet final : public TPacket_internal {
public:
    using packet_type   = TPacket_internal;
    using protocol_type = TProtocol;

public:
    wrapper_packet() = default;
    wrapper_packet(packet_type &&pckt) : packet_type(pckt) {}

public:
    static void prepare(packet_type &pckt, buffer_address_type addr,
                        protocol_type::callback_prepare_type callback) {
        prt.prepare(pckt, addr, callback);
    }
    static void handle(packet_type &&pckt, buffer_address_type addr,
                       protocol_type::callback_handle_type callback) {
        prt.handle(pckt, addr, callback);
    }

public:
    template<typename TSelf>
    decltype(auto) get_native_packet(this TSelf &&self) {
        if constexpr (std::is_const_v<std::remove_reference_t<TSelf>>)
            return *static_cast<
                const typename std::remove_reference_t<TSelf>::packet_type *>(&self);
        else
            return *static_cast<typename std::remove_reference_t<TSelf>::packet_type *>(
                &self);
    }
    static constexpr const protocol_type &get_protocol() { return prt; }

private:
    static constexpr protocol_type prt{};
};

template<typename T>
concept Wrapper_packet =
    std::same_as<std::decay_t<T>,
                 wrapper_packet<typename std::decay_t<T>::packet_type,
                                typename std::decay_t<T>::protocol_type>>;
template<typename TPacket, typename TAction>
concept Compatible_wrapper_packet_with_action =
    Wrapper_packet<TPacket> && Derived_from_action<TAction>
    && std::same_as<typename std::decay_t<TAction>::packet_type,
                    typename std::decay_t<TPacket>::packet_type>;

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

private:
    struct buffered_received_packet_type {
        action_type::packet_type pckt;
        std::size_t              received_ms;
    };
    struct receive_buffer_of_address_type {
        receive_buffer_of_address_type() = default;
        receive_buffer_of_address_type(address_type _addr) : addr(_addr) {}
        receive_buffer_of_address_type(const receive_buffer_of_address_type &other) {
            this->operator=(other);
        }
        receive_buffer_of_address_type &
            operator=(const receive_buffer_of_address_type &other) {
            addr   = other.addr;
            pckt_s = other.pckt_s;
            return *this;
        }

    public:
        address_type                                                             addr;
        noheap::ring_buffer<buffered_received_packet_type, max_buffered_packets> pckt_s;

        std::condition_variable cv;
        std::mutex              m;
    };
    struct send_buffer_of_address_type {
        address_type addr;
        noheap::ring_buffer<typename action_type::packet_type, max_buffered_packets>
            pckt_s;
    };
    using receive_buffer_type =
        noheap::monotonic_array<receive_buffer_of_address_type, max_count_addresses>;
    using send_buffer_type =
        noheap::monotonic_array<send_buffer_of_address_type, max_count_addresses>;

public:
    udp_stream(asio::io_context &io, asio::ip::port_type _port);
    udp_stream(udp_stream &&)      = delete;
    udp_stream(const udp_stream &) = delete;

    ~udp_stream();

public:
    decltype(auto) get_action() { return act; }
    decltype(auto) get_executor() { return socket.get_executor(); }
    decltype(auto) get_port() const { return port; }
    decltype(auto) get_running() const { return running; }

    void close() {
        socket.close();

        running.store(false);
        cv_send.notify_all();
    }

    void register_async_send();
    void register_async_receive();
    void register_address(address_type addr);

    template<Compatible_wrapper_packet_with_action<Action> TWrapper_packet>
    void send_to(address_type addr);
    template<Compatible_wrapper_packet_with_action<Action> TWrapper_packet>
    bool receive_from(address_type addr);

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

    typename action_type::packet_type receive_pckt{};
    endpoint_type                     receive_endpoint;

    std::mutex              m_send;
    std::condition_variable cv_send;

    std::atomic<bool> running;

    port_type port;

    send_buffer_type    send_buffer;
    receive_buffer_type receive_buffer;
};

template<Derived_from_action Action, ipv v>
udp_stream<Action, v>::udp_stream(asio::io_context &io, port_type _port)
    : socket(io), running(true), port(_port) {
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
    asio::post(socket.get_executor(), [this] {
        {
            // Waits for a signal from send_to that there is new packets for send
            std::unique_lock<decltype(m_send)> m_send_lock(m_send);
            cv_send.wait(m_send_lock, [this] {
                if (!running.load())
                    return true;

                for (auto &buffer_of_address : this->send_buffer)
                    if (buffer_of_address.pckt_s.size())
                        return true;
                return false;
            });

            system::error_code ec;
            for (auto &buffer_of_address : send_buffer) {
                while (buffer_of_address.pckt_s.size()) {
                    auto pckt = buffer_of_address.pckt_s.pop();
                    this->socket.send_to(
                        asio::mutable_buffer{pckt.data(), pckt.size()},
                        endpoint_type{buffer_of_address.addr, this->port}, 0, ec);
                    this->handle_error(ec);
                }
            }
        }
        this->register_async_send();
    });
}
template<Derived_from_action Action, ipv v>
void udp_stream<Action, v>::register_async_receive() {
    // Waits for a new packet
    this->socket.async_receive_from(
        asio::mutable_buffer{receive_pckt.data(), receive_pckt.size()}, receive_endpoint,
        0, [this](system::error_code ec, std::size_t) {
            this->handle_error(ec);
            auto remote_addr = this->get_address_object(receive_endpoint.address());

            // Find it of receive_buffer and checks that the address is registered
            decltype(receive_buffer.begin()) it;
            if (it = std::find_if(
                    receive_buffer.begin(), receive_buffer.end(),
                    [&remote_addr](const auto &it) { return it.addr == remote_addr; });
                it == receive_buffer.end()) {
                if (receive_buffer.size() == max_count_addresses)
                    return;
                this->register_address(remote_addr);
            }

            {
                // Adds the packet to receive buffer
                std::lock_guard<decltype(it->m)> m_receive_lock(it->m);
                it->pckt_s.push(
                    buffered_received_packet_type{receive_pckt, get_now_ms()});

                // Signals receive_from that there is new packet for local receive
                it->cv.notify_all();
            }

            this->register_async_receive();
        });
}
template<Derived_from_action Action, ipv v>
void udp_stream<Action, v>::register_address(address_type addr) {
    if (auto it = std::find_if(send_buffer.begin(), send_buffer.end(),
                               [&addr](const auto &it) { return it.addr == addr; });
        it != send_buffer.end())
        log.throw_exception(
            "Address [{}] has already been registered.",
            std::string_view(noheap::hex_encode(this->get_address_bytes(addr))));
    else if (send_buffer.size() == max_count_addresses)
        log.throw_exception("Limit of connections has been reached.");

    send_buffer.push_back(typename decltype(send_buffer)::value_type{addr});
    receive_buffer.push_back(typename decltype(receive_buffer)::value_type{addr});
}

template<Derived_from_action Action, ipv v>
template<Compatible_wrapper_packet_with_action<Action> TWrapper_packet>
void udp_stream<Action, v>::send_to(address_type addr) {
    typename action_type::packet_type pckt{};

    // Performs init a packet base on TWrapper_packet protocol
    std::decay_t<TWrapper_packet>::prepare(
        pckt, this->get_address_bytes(this->get_address_object(addr)),
        std::bind(&Action::init_packet, &this->act, std::placeholders::_1));

    // Find it of send_buffer and checks that the address is registered
    decltype(this->send_buffer.begin()) it;
    if (it = std::find_if(send_buffer.begin(), send_buffer.end(),
                          [&addr](const auto &it) { return it.addr == addr; });
        it == send_buffer.end())
        log.throw_exception(
            "Address [{}] is not registered.",
            std::string_view(noheap::hex_encode(this->get_address_bytes(addr))));

    // Pushes the packet to send_buffer
    {
        std::lock_guard<decltype(m_send)> m_send_lock(m_send);
        it->pckt_s.push(pckt);

        // Wakes the main loop of sending packets up(register_async_send)
        cv_send.notify_one();
    }
}

template<Derived_from_action Action, ipv v>
template<Compatible_wrapper_packet_with_action<Action> TWrapper_packet>
bool udp_stream<Action, v>::receive_from(address_type addr) {
    typename action_type::packet_type pckt;

    // Find it of receive_buffer and checks that the address is registered
    decltype(this->receive_buffer.begin()) it;
    if (it = std::find_if(receive_buffer.begin(), receive_buffer.end(),
                          [&addr](const auto &it) { return it.addr == addr; });
        it == receive_buffer.end())
        log.throw_exception(
            "Address [{}] is not registered.",
            std::string_view(noheap::hex_encode(this->get_address_bytes(addr))));

    // Waits for a signal from register_async_receive that a packet was received
    {
        std::unique_lock<decltype(it->m)> m_receive_lock(it->m);
        bool                              received_packet = false;
        if (!it->cv.wait_for(m_receive_lock, std::chrono::milliseconds(timeout_ms),
                             [&] {
                                 if (!running.load())
                                     return true;
                                 received_packet = true;
                                 return it->pckt_s.size() > 0;
                             })
            || !received_packet)
            return false;

        pckt = it->pckt_s.pop().pckt;
    }

    // Handle the packet base on TWrapper_packet protocol
    std::decay_t<TWrapper_packet>::handle(
        std::move(pckt), this->get_address_bytes(addr),
        std::bind(&Action::process_packet, &this->act, std::placeholders::_1));

    return true;
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
