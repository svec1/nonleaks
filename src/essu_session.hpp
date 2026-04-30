#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include "essu_protocol.hpp"

using namespace boost;

namespace essu {

template<network::Udp_stream TStream>
class session {
public:
    using udp_stream = TStream;

public:
    session(udp_stream &_stream, udp_stream::address_type _remote_addr,
            noise::noise_role _role, noise::prologue_extention_type _ext,
            const noise::pre_shared_key_type       &_pre_shared_key,
            const noise_context_type::keypair_type &_local_keypair,
            const noise_context_type::dh_key_type  &_remote_public_key);

public:
    // Establishes connection with node(remote_addr): performs noise handshake
    void establish_connection();
    void register_connection();
    void wait();
    void terminate();

    bool is_running() const;
    bool is_failed() const;

private:
    void send();
    void receive();

    void self_post(std::function<bool()> func);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION");
    static constexpr log_handler log{buffer_owner};

private:
    udp_stream       &stream;
    session_info_type info;

    noheap::buffer_chars_type<
        noheap::buffer_size<typename network::buffer_address_v<TStream::v>::type> * 2>
                           buffer_hex_remote_addr;
    const std::string_view hex_remote_addr{buffer_hex_remote_addr};

    std::atomic<bool>        running = false;
    std::atomic<bool>        failed  = false;
    std::atomic<std::size_t> io_stop = 0; // 2 - is full stop

    std::optional<noheap::runtime_error> excp;
};
} // namespace essu

template<network::Udp_stream TStream>
essu::session<TStream>::session(udp_stream                             &_stream,
                                udp_stream::address_type                _remote_addr,
                                noise::noise_role                       _role,
                                noise::prologue_extention_type          _ext,
                                const noise::pre_shared_key_type       &_pre_shared_key,
                                const noise_context_type::keypair_type &_local_keypair,
                                const noise_context_type::dh_key_type &_remote_public_key)
    : stream(_stream), info(stream.get_address_bytes(_remote_addr)),
      buffer_hex_remote_addr(
          noheap::clip_buffer<noheap::buffer_size<decltype(buffer_hex_remote_addr)>, 0>(
              noheap::hex_encode(info.addr))) {
    essu::wrapper_packet_type::get_protocol().register_session_info(
        info, _role, _ext, _pre_shared_key, _local_keypair, _remote_public_key);
}
template<network::Udp_stream TStream>
void essu::session<TStream>::establish_connection() {
    try {
        decltype(auto) protocol = essu::wrapper_packet_type::get_protocol();
        decltype(protocol.get_handshake_action(info)) action;

        // Performs noise handshake
        protocol.start_handshake(info);
        while ((action = protocol.get_handshake_action(info))
               != noise::noise_action::SPLIT) {
            if (action == noise::noise_action::WRITE_MESSAGE)
                send();
            else if (action == noise::noise_action::READ_MESSAGE)
                receive();
        }
        protocol.stop_handshake(info);

        log.to_all("Number of handshake: {}", protocol.get_handshake_number(info));
    } catch (const noheap::runtime_error &excp) {
        failed.store(true);
        log.throw_exception<noheap::runtime_error>("Failed to establish connection [{}]",
                                                   excp.what());
    }
}

template<network::Udp_stream TStream>
void essu::session<TStream>::register_connection() {
    if (!essu::wrapper_packet_type::get_protocol().can_send_packet(info)
        || !essu::wrapper_packet_type::get_protocol().can_receive_packet(info))
        log.throw_exception<noheap::runtime_error>("Failed to register connection.");

    io_stop.store(0);
    running.store(true);

    self_post([this] {
        this->send();
        return this->running.load()
               && essu::wrapper_packet_type::get_protocol().can_send_packet(this->info);
    });
    self_post([this] {
        this->receive();
        return this->running.load()
               && essu::wrapper_packet_type::get_protocol().can_receive_packet(
                   this->info);
    });
}
template<network::Udp_stream TStream>
void essu::session<TStream>::self_post(std::function<bool()> func) {
    asio::post(stream.get_executor(), [this, func] {
        const auto stop_post = [this] {
            ++this->io_stop;
            if (this->io_stop == 2)
                this->running.store(false);
            this->io_stop.notify_all();
        };

        try {
            if (func())
                this->self_post(func);
            else
                stop_post();
        } catch (const noheap::runtime_error &_excp) {
            if (!excp)
                excp = _excp;
            this->failed.store(true);
            stop_post();
        } catch (...) {
            this->failed.store(true);
            stop_post();
            throw;
        }
    });
}
template<network::Udp_stream TStream>
void essu::session<TStream>::wait() {
    io_stop.wait(0);
    io_stop.wait(1);
    if (excp)
        throw *excp;
}
template<network::Udp_stream TStream>
void essu::session<TStream>::terminate() {
    running.store(false);
}
template<network::Udp_stream TStream>
bool essu::session<TStream>::is_running() const {
    return running.load();
}
template<network::Udp_stream TStream>
bool essu::session<TStream>::is_failed() const {
    return failed.load();
}

template<network::Udp_stream TStream>
void essu::session<TStream>::send() {
    stream.template send_to<essu::wrapper_packet_type>(
        TStream::get_address_object(info.addr));
}

template<network::Udp_stream TStream>
void essu::session<TStream>::receive() {
    if (!stream.template receive_from<essu::wrapper_packet_type>(
            TStream::get_address_object(info.addr)))
        log.throw_exception<noheap::runtime_error>("Timeout has been reached.");
}

#endif
