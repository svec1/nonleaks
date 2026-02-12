#ifndef UDP_VOICE_SERVICE_HPP
#define UDP_VOICE_SERVICE_HPP

#include <boost/json.hpp>
#include <boost/json/src.hpp>

#include "crypt.hpp"
#include "net.hpp"
#include "protocol.hpp"

using namespace boost;

class unix_udp_voice_service {
public:
    static constexpr ipv v = ipv::v4;

    static constexpr std::size_t max_size_config = 4096;
    using buffer_config_type = noheap::buffer_bytes_type<max_size_config>;

    template<ntn_relation relation_type>
    using noise_handshake_packet = protocol::noise_handshake_packet<relation_type>;
    using payload_packet         = protocol::payload_packet;

    using stream_tcp_type = net_stream_tcp<protocol::noise_handshake_action, v>;
    using acceptor_type   = stream_tcp_type::acceptor_type;
    using stream_udp_type = net_stream_udp<protocol::payload_action, v>;
    using address_type    = stream_udp_type::address_type;

private:
    struct config_type {
        noise_pattern pattern;
        noise_role    role;

        buffer_key_type<max_size_key> local_private_key;
        buffer_key_type<max_size_key> local_public_key;
        buffer_key_type<max_size_key> remote_public_key;
        buffer_key_type<max_size_key> pre_shared_key;
    };

public:
    unix_udp_voice_service(address_type &&_addr, asio::ip::port_type _port);

    void run();
    void configurate(buffer_config_type &&buffer);

private:
    template<Packet packet, typename TUDPStream>
    static void run_payload_consume(TUDPStream &udp_stream, address_type addr);

    template<ntn_relation relation_type>
    static void noise_handshake(stream_tcp_type &tcp_stream, config_type &&config);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

private:
    config_type config;

    asio::io_context    io;
    address_type        addr;
    asio::ip::port_type tcp_port;
    asio::ip::port_type udp_port;
    stream_udp_type     udp_stream;

    static openssl_context ossl_ctx;
};

openssl_context unix_udp_voice_service::ossl_ctx;

unix_udp_voice_service::unix_udp_voice_service(address_type      &&_addr,
                                               asio::ip::port_type _port)
    : addr(_addr), tcp_port(_port), udp_port(tcp_port + 1), udp_stream(io, udp_port) {
}
template<ntn_relation relation_type>
void unix_udp_voice_service::noise_handshake(stream_tcp_type &tcp_stream,
                                             config_type    &&config) {
    using packet_type        = noise_handshake_packet<relation_type>;
    using noise_context_type = protocol::noise_context_type<relation_type>;
    packet_type pckt{};

    auto &noise_ctx =
        noise_handshake_packet<ntn_relation::PTU>::get_protocol().get_noise_context();
    noise_ctx.set_prologue({});

    auto noise_name_id       = noise_ctx.get_name_id();
    auto noise_prologue      = noise_ctx.get_prologue();
    auto noise_prologue_hash = noheap::to_hex_string<noheap::buffer_bytes_type<256>>(
        ossl_ctx.get_hash<openssl_context::algorithm::SHA1>(
            std::string_view(noise_prologue.data(), noise_prologue.size())));
    log.to_all("Starting noise handshake: {}",
               std::string_view(noise_name_id.data(), noise_name_id.size()));
    log.to_all("Handshake prologue: {}", noise_prologue_hash.data());

    noise_ctx.set_local_keypair(
        {noheap::to_new_array<typename noise_context_type::dh_key_type>(
             config.local_private_key),
         noheap::to_new_array<typename noise_context_type::dh_key_type>(
             config.local_public_key)});
    noise_ctx.set_remote_public_key(
        noheap::to_new_array<typename noise_context_type::dh_key_type>(
            config.remote_public_key));
    noise_ctx.set_pre_shared_key(
        noheap::to_new_array<typename noise_context_type::pre_shared_key_type>(
            config.pre_shared_key));

    noise_ctx.start();
    while (true) {
        auto action = noise_ctx.get_action();
        if (action == noise_action::WRITE_MESSAGE)
            tcp_stream.template send<decltype(pckt)>(pckt);
        else if (action == noise_action::READ_MESSAGE)
            tcp_stream.template receive<decltype(pckt)>(pckt);
        else
            break;
    }
    noise_ctx.stop();
}
void unix_udp_voice_service::run() {
    try {
        if (is_ptu(config.pattern)) {
            const auto &payload_prt = payload_packet::get_protocol();

            noise_context<ntn_relation::PTU>::cipher_state cipher_state;

            {
                const auto &noise_handshake_prt =
                    noise_handshake_packet<ntn_relation::PTU>::get_protocol();

                noise_context<ntn_relation::PTU> noise_ctx(config.pattern, config.role);
                stream_tcp_type                  tcp_stream(io, tcp_port);

                if (!tcp_stream.wait_connect({addr, tcp_port})) {
                    log.to_console("Listen...");

                    tcp_stream.close();

                    acceptor_type ac(io, tcp_port);
                    ac.accept(tcp_stream);
                }

                noise_handshake_prt.set_noise_context(noise_ctx);
                noise_handshake<ntn_relation::PTU>(tcp_stream, std::move(config));

                cipher_state = noise_ctx.get_cipher_state();
                payload_prt.set_noise_cipher_state(cipher_state);
            }

            payload_prt.set_local_sequence_number(
                *reinterpret_cast<decltype(payload_packet{}->payload.sequence_number) *>(
                    ossl_ctx
                        .get_random_bytes<sizeof(
                            payload_packet{}->payload.sequence_number)>()
                        .data()));
            payload_prt.set_uuid(ossl_ctx.get_random_bytes<
                                 protocol::payload_protocol_type::uuid_type{}.size()>());

            log.to_all("Payload consumer is executing...");
            std::future<void> payload_consumer =
                std::async(std::launch::async,
                           unix_udp_voice_service::run_payload_consume<payload_packet,
                                                                       stream_udp_type>,
                           std::ref(udp_stream), addr);
            if (payload_consumer.valid())
                payload_consumer.get();
        }

    } catch (noheap::runtime_error &excp) {
        if (!excp.has_setting_owner())
            excp.set_owner(buffer_owner);
        throw;
    }
}

template<Packet packet, typename TUDPStream>
void unix_udp_voice_service::run_payload_consume(TUDPStream  &udp_stream,
                                                 address_type addr) {
    std::future<void> payload_handler = std::async(std::launch::async, [&] {
        packet pckt_for_receiving{}, pckt_for_sending{};

        udp_stream.template register_send_handler<0>(pckt_for_sending, addr);
        udp_stream.register_receive_handler(pckt_for_receiving);

        udp_stream.io_context_run();
    });

    const auto &payload_prt = payload_packet::get_protocol();
    auto        begin_ms    = get_now_ms();
    float       loss_avg    = 0;

    while (payload_handler.valid()
           && payload_handler.wait_for(std::chrono::milliseconds(100))
                  != std::future_status::ready) {
        if (auto end_ms = get_now_ms(); end_ms - begin_ms >= 1000) {
            float loss_current = payload_prt.get_loss_per_cent();
            if (loss_current > loss_avg)
                log.to_all("Packet loss: {:.2f}%", loss_current);

            loss_avg = (loss_current + loss_avg) / 2;
            begin_ms = end_ms;
        }
    }

    if (payload_handler.valid())
        payload_handler.get();
}
void unix_udp_voice_service::configurate(buffer_config_type &&buffer) {
    log.to_all("Setting of configurate.");

    {
        constexpr std::string_view role_string          = "role";
        constexpr std::string_view pattern_string       = "pattern";
        constexpr std::string_view local_private_string = "local_private_key";
        constexpr std::string_view local_public_string  = "local_public_key";
        constexpr std::string_view remote_public_string = "remote_public_key";
        constexpr std::string_view pre_shared_string    = "pre_shared_key";

        noheap::buffer_bytes_type<8192, std::uint8_t> json_buffer_tmp;
        noheap::buffer_bytes_type<1024>               buffer_tmp;

        json::static_resource json_mr(json_buffer_tmp.data(), json_buffer_tmp.size());

        json::value data(&json_mr);
        data = json::parse(buffer.data(), &json_mr);

        const auto &get_bytes_key = [&](const std::string_view field_name, auto &buffer) {
            auto value_p = data.try_at(field_name);
            if (!value_p)
                return;

            auto string_p = value_p->try_as_string();
            if (string_p) {
                if (string_p->size() >= buffer.size())
                    throw noheap::runtime_error(
                        buffer_owner,
                        "The specified key field has a large size:\n\t{}: {}",
                        buffer.size(), field_name, *string_p);
                std::copy(string_p->begin(), string_p->end(), buffer.begin());
            } else
                throw noheap::runtime_error(buffer_owner,
                                            "Field of key must be a string.");
        };

        auto value_p = data.try_at(role_string);
        if (!value_p)
            throw noheap::runtime_error(buffer_owner, "Field of role not specified.");
        auto role_p = value_p->try_as_string();
        if (!role_p)
            throw noheap::runtime_error(buffer_owner, "Field of role must be a string.");

        value_p = data.try_at(pattern_string);
        if (!value_p)
            throw noheap::runtime_error(buffer_owner, "Field of pattern  not specified.");
        auto pattern_p = value_p->try_as_string();
        if (!pattern_p)
            throw noheap::runtime_error(buffer_owner,
                                        "Field of pattern must be a string.");

        config.pattern = get_noise_pattern(*pattern_p);
        config.role    = get_noise_role(*role_p);
        get_bytes_key(local_private_string, config.local_private_key);
        get_bytes_key(local_public_string, config.local_public_key);
        get_bytes_key(remote_public_string, config.remote_public_key);
        get_bytes_key(pre_shared_string, config.pre_shared_key);
    }
}

#endif
