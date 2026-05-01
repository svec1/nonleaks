#ifndef XXCORE_SERVICE_HPP
#define XXCORE_SERVICE_HPP

#include <boost/json.hpp>
#include <boost/json/src.hpp>

#include "audio_flow.hpp"
#include "essu_session.hpp"
#include "stream_audio.hpp"

using namespace boost;

// For test
struct test_action final : network::action<essu::packet_type> {
    static constexpr std::size_t max_stream_size = 32;

    using audio_flow_type = audio_flow<max_stream_size>;

public:
    void init_packet(test_action::packet_type &pckt) {
        audio_flow_type::buffer_type buffer_tmp{};
        audio.pop(buffer_tmp);
        pckt->units[0].header.type = decltype(pckt->units[0].header.type)::data;
        std::copy(reinterpret_cast<noheap::rbyte *>(buffer_tmp.begin()),
                  reinterpret_cast<noheap::rbyte *>(buffer_tmp.end()),
                  pckt->units[0].buffer.begin());
        for (std::size_t i = 1; i < pckt->units.size(); ++i)
            pckt->units[i].header.type = decltype(pckt->units[0].header.type)::dummy;
    }
    void process_packet(test_action::packet_type &&pckt) {
        audio_flow_type::buffer_type buffer_tmp;
        std::copy(pckt->units[0].buffer.begin(),
                  pckt->units[0].buffer.begin() + buffer_tmp.size(),
                  reinterpret_cast<noheap::rbyte *>(buffer_tmp.begin()));
        audio.push(std::move(buffer_tmp), false);
    }

private:
    audio_flow_type audio;
};

struct config_type {
    noise::noise_role                      role;
    essu::noise_context_type::keypair_type keypair;
    essu::noise_context_type::dh_key_type  rpubk;
    essu::noise_context_type::dh_key_type  psk;
};

struct json_config {
private:
    static constexpr std::string_view role_string  = "role";
    static constexpr std::string_view privk_string = "privk";
    static constexpr std::string_view pubk_string  = "pubk";
    static constexpr std::string_view rpubk_string = "rpubk";
    static constexpr std::string_view psk_string   = "psk";

public:
    static constexpr std::size_t max_size_config = BOOST_JSON_STACK_BUFFER_SIZE;
    using buffer_config_type = noheap::buffer_type<char, max_size_config>;

public:
    void set_buffer_config(const buffer_config_type &buffer, bool new_keypair);
    void get_buffer_config(buffer_config_type &buffer);

    config_type &get_config();

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("JSON_CONFIG");
    static constexpr log_handler log{buffer_owner};

private:
    config_type config{};
};

class xxcore_service {
public:
    static constexpr std::size_t workers_number = network::max_count_addresses + 2;

    using udp_stream   = network::udp_stream<test_action, network::ipv::v4>;
    using address_type = udp_stream::address_type;
    using port_type    = udp_stream::port_type;
    using session_type = essu::session<udp_stream>;

public:
    xxcore_service(config_type &config, address_type &&_addr, asio::ip::port_type _port);

    void run();

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

private:
    config_type     &config;
    address_type     addr;
    asio::io_context io;
    udp_stream       stream;
};

xxcore_service::xxcore_service(config_type &_config, address_type &&_addr,
                               asio::ip::port_type _port)
    : config(_config), addr(std::move(_addr)), stream(io, _port) {
}

void xxcore_service::run() {
    noheap::buffer_type<future_wrapper<void>, workers_number>  workers;
    asio::executor_work_guard<asio::io_context::executor_type> work_guard(
        io.get_executor());

    for (auto &worker : workers)
        worker = typename std::decay_t<decltype(worker)>{[this] { this->io.run(); }};

    {
        scope_guard session_stop([&] {
            io.stop();
            stream.close();
        });

        stream.register_address(addr);
        stream.register_async_send();
        stream.register_async_receive();

        // Tests
        session_type session_test(stream, addr, config.role, {}, config.keypair,
                                  config.rpubk, config.psk);

        do {
            session_test.establish_connection();
            session_test.register_connection();

            session_test.wait();
        } while (!session_test.is_failed());
    }

    for (auto &worker : workers)
        worker.get();
}

void json_config::set_buffer_config(const buffer_config_type &buffer, bool new_keypair) {
    noheap::buffer_bytes_type<BOOST_JSON_STACK_BUFFER_SIZE, noheap::ubyte>
        json_buffer_tmp;

    json::static_resource json_mr(json_buffer_tmp.data(), json_buffer_tmp.size());

    json::value data          = json::parse(buffer.data(), &json_mr);
    auto        global_object = data.as_object();

    auto get_bytes_key = [&](const std::string_view field_name, auto &buffer_key,
                             bool hex_encoding = true) {
        if (!global_object.contains(field_name))
            return;

        auto field_key = global_object.at(field_name);
        if (!field_key.is_string())
            throw noheap::runtime_error(buffer_owner, "Field of key must be a string.");

        auto string_key = field_key.as_string();
        if (string_key.size() >= buffer_key.size() && !hex_encoding)
            throw noheap::runtime_error(
                buffer_owner, "The specified key field has a large size: {}", field_name);

        if (hex_encoding) {
            decltype(noheap::hex_encode(buffer_key)) buffer_key_hex{};
            std::copy(reinterpret_cast<noheap::rbyte *>(string_key.begin()),
                      reinterpret_cast<noheap::rbyte *>(string_key.end()),
                      reinterpret_cast<noheap::rbyte *>(buffer_key_hex.begin()));
            buffer_key = noheap::to_buffer<decltype(buffer_key)>(
                noheap::hex_decode(buffer_key_hex));
        } else
            std::copy(reinterpret_cast<noheap::rbyte *>(string_key.begin()),
                      reinterpret_cast<noheap::rbyte *>(string_key.end()),
                      reinterpret_cast<noheap::rbyte *>(buffer_key.begin()));
    };

    if (!global_object.contains(role_string))
        throw noheap::runtime_error(buffer_owner, "Field of role not specified.");

    auto field_role = global_object.at(role_string);
    if (!field_role.is_string())
        throw noheap::runtime_error(buffer_owner, "Field of role must be string.");
    config.role = noise::get_noise_role(field_role.as_string());

    if (new_keypair) {
        config.keypair = essu::noise_context_type::generate_keypair();
    } else {
        get_bytes_key(privk_string, config.keypair.priv);
        get_bytes_key(pubk_string, config.keypair.pub);
    }
    get_bytes_key(rpubk_string, config.rpubk);
    get_bytes_key(psk_string, config.psk, false);
}
void json_config::get_buffer_config(buffer_config_type &buffer) {
    noheap::buffer_bytes_type<BOOST_JSON_STACK_BUFFER_SIZE, noheap::ubyte>
        json_buffer_tmp;

    json::static_resource json_mr(json_buffer_tmp.data(), json_buffer_tmp.size());
    json::serializer      sz(&json_mr);
    buffer = {};

    json::value data = {
        {role_string, noise::get_noise_role_string(config.role)},
        {privk_string, std::string_view(noheap::hex_encode(config.keypair.priv))},
        {pubk_string, std::string_view(noheap::hex_encode(config.keypair.pub))},
        {rpubk_string, config.rpubk != std::decay_t<decltype(config.rpubk)>{}
                           ? std::string_view(noheap::hex_encode(config.rpubk))
                           : ""},
        {psk_string,
         std::string_view(noheap::to_buffer<noheap::buffer_chars_type<
                              noheap::buffer_size<decltype(config.psk)>>>(config.psk)
                              .data(),
                          std::strlen(reinterpret_cast<char *>(config.psk.data())))}};

    sz.reset(&data);
    sz.read(buffer.data(), buffer.size());
}
config_type &json_config::get_config() {
    return config;
}
#endif
