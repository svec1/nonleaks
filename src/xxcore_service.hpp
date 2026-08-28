#ifndef XXCORE_SERVICE_HPP
#define XXCORE_SERVICE_HPP

#include <boost/json.hpp>
#include <boost/json/src.hpp>

#include "essu_session.hpp"
#include "stream_audio.hpp"

using namespace boost;

struct config_type {
    network::port_type                 listen_port;
    bool                               on_ipv6;
    essu::session_handler::config_type local_config;
    noheap::monotonic_array<std::pair<noheap::buffer_chars_type<16>,
                                      essu::session_handler::endpoint_config_type>,
                            essu::max_session_number>
        endpoint_config_s;
};

struct json_config {
private:
    static constexpr std::string_view listen_port_string = "listen_port";
    static constexpr std::string_view on_ipv6_string     = "on_ipv6";
    static constexpr std::string_view ipv6_string        = "ipv6";
    static constexpr std::string_view address_string     = "address";
    static constexpr std::string_view port_string        = "port";
    static constexpr std::string_view role_string        = "role";
    static constexpr std::string_view privk_string       = "privk";
    static constexpr std::string_view pubk_string        = "pubk";
    static constexpr std::string_view psk_string         = "psk";

public:
    static constexpr std::size_t max_size_config = BOOST_JSON_STACK_BUFFER_SIZE;
    using buffer_config_type = noheap::buffer_type<char, max_size_config>;

public:
    inline json_config(const log_handler &handler);

public:
    inline void set_buffer_config(const buffer_config_type &buffer, bool new_keypair);
    inline void get_buffer_config(buffer_config_type &buffer);

    inline config_type &get_config();

private:
    template<typename T>
    inline T get_object_field(const json::object    &value,
                              const std::string_view field_string);

private:
    log_handler::proxy log;

private:
    config_type config{};
};

class xxcore_service {
public:
    static constexpr std::size_t workers_number = essu::max_session_number + 2;

public:
    inline xxcore_service(const log_handler &handler, config_type &config);
    inline void run();

private:
    log_handler::proxy log;

private:
    config_type     &config;
    asio::io_context io;
};

xxcore_service::xxcore_service(const log_handler &handler, config_type &_config)
    : log(handler.create_proxy({"XXCORE"})), config(_config) {
}

void xxcore_service::run() {
    noheap::buffer_type<future_wrapper<void>, workers_number>  workers;
    asio::executor_work_guard<asio::io_context::executor_type> work_guard(
        io.get_executor());

    for (auto &worker : workers)
        worker = typename std::decay_t<decltype(worker)>{[this] { this->io.run(); }};

    log.info("Start...");

    // For testing: will be deleted
    {
        essu::session_handler action(log.get_handler(), config.local_config);
        network::udp_stream<essu::session_handler> stream(log.get_handler(), action, io,
                                                          config.listen_port);
        scope_guard                                session_stop([&] {
            stream.close();
            io.stop();
        });

        for (auto &endpoint_config_pair : config.endpoint_config_s) {
            action.add_endpoint_config(endpoint_config_pair.second);
            action.register_new_session(endpoint_config_pair.second);
        }

        stream.open(config.on_ipv6 ? network::ipv::v4v6 : network::ipv::v4);

        {
            std::size_t count_received_packets = 0;
            auto        start                  = std::chrono::steady_clock::now();
            while (action.is_valid()) {
                while (action.exist_received_packets()) {
                    (void) action.pop_packet();
                    ++count_received_packets;
                }
                for (decltype(auto) it = action.get_registered_session_s().cbegin();
                     it < action.get_registered_session_s().cend(); ++it) {
                    essu::packet_type pckt;
                    essu::set_dummy_packet(pckt);
                    action.push_packet({pckt, it});
                }
                for (decltype(auto) it = action.get_incoming_session_s().cbegin();
                     it < action.get_incoming_session_s().cend(); ++it)
                    action.register_incoming_session(it);

                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                if (auto end = std::chrono::steady_clock::now();
                    std::chrono::duration_cast<std::chrono::seconds>(end - start)
                    == std::chrono::seconds(1)) {
                    log.info("Pckts/s: {}", count_received_packets);
                    start                  = end;
                    count_received_packets = 0;
                }
            }
            throw action.get_error();
        }
    }

    log.info("Stop.");

    for (auto &worker : workers)
        worker.get();
}
json_config::json_config(const log_handler &handler)
    : log(handler.create_proxy({"JSON_CONFIG"})) {
}
void json_config::set_buffer_config(const buffer_config_type &buffer, bool new_keypair) {
    noheap::buffer_bytes_type<BOOST_JSON_STACK_BUFFER_SIZE, noheap::ubyte>
        json_buffer_tmp;

    json::static_resource json_mr(json_buffer_tmp.data(), json_buffer_tmp.size());

    json::value    data          = json::parse(buffer.data(), &json_mr);
    decltype(auto) global_object = data.as_object();

    const auto get_bytes_key = [this](const json::object    &object,
                                      const std::string_view field_name, auto &buffer_key,
                                      bool hex_encoding = true) {
        auto string_key = this->get_object_field<std::string_view>(object, field_name);
        if (string_key.size() >= buffer_key.size() && !hex_encoding)
            log.throw_and_log<noheap::runtime_error>(
                {}, "The key[{}] field has a large size", field_name);

        if (hex_encoding) {
            decltype(noheap::hex_encode(buffer_key)) buffer_key_hex{};
            std::copy(reinterpret_cast<const noheap::rbyte *>(string_key.begin()),
                      reinterpret_cast<const noheap::rbyte *>(string_key.end()),
                      reinterpret_cast<noheap::rbyte *>(buffer_key_hex.begin()));
            buffer_key = noheap::to_buffer<decltype(buffer_key)>(
                noheap::hex_decode(buffer_key_hex));
        } else
            std::copy(reinterpret_cast<const noheap::rbyte *>(string_key.begin()),
                      reinterpret_cast<const noheap::rbyte *>(string_key.end()),
                      reinterpret_cast<noheap::rbyte *>(buffer_key.begin()));
    };

    config.listen_port =
        get_object_field<std::uint16_t>(global_object, listen_port_string);
    config.on_ipv6 = get_object_field<bool>(global_object, on_ipv6_string);
    if (new_keypair) {
        config.local_config.keypair = essu::noise_context_type::generate_keypair();
    } else {
        get_bytes_key(global_object, privk_string, config.local_config.keypair.priv);
        get_bytes_key(global_object, pubk_string, config.local_config.keypair.pub);
    }

    for (const auto &global_field : global_object) {
        decltype(auto) global_field_key   = global_field.key();
        decltype(auto) global_field_value = global_field.value();

        if (!global_field_value.is_object())
            continue;

        if (std::find_if(config.endpoint_config_s.begin(), config.endpoint_config_s.end(),
                         [global_field_key](const auto &el) {
                             return !std::strcmp(el.first.data(),
                                                 global_field_key.data());
                         })
            != config.endpoint_config_s.end())
            log.throw_and_log<noheap::runtime_error>({}, "Endpoint[{}] is already exist.",
                                                     global_field_key.data());

        decltype(config_type::endpoint_config_s)::value_type::second_type
            endpoint_config{};
        decltype(config_type::endpoint_config_s)::value_type::first_type
                       endpoint_config_name{};
        decltype(auto) object = global_field_value.as_object();

        std::copy(global_field_key.begin(),
                  global_field_key.begin()
                      + std::clamp<std::size_t>(global_field_key.size(), 0,
                                                endpoint_config_name.size()),
                  endpoint_config_name.begin());
        endpoint_config.endpoint.v       = get_object_field<bool>(object, ipv6_string)
                                               ? network::ipv::v6
                                               : network::ipv::v4;
        endpoint_config.endpoint.address = network::utils::string_address_to_bytes(
            get_object_field<std::string_view>(object, address_string),
            endpoint_config.endpoint.v);
        endpoint_config.endpoint.port =
            get_object_field<network::port_type>(object, port_string);
        endpoint_config.local_role = noise::get_noise_role(
            get_object_field<std::string_view>(object, role_string));

        get_bytes_key(object, pubk_string, endpoint_config.public_key);
        get_bytes_key(object, psk_string, endpoint_config.pre_shared_key, false);

        config.endpoint_config_s.push_back(
            decltype(config_type::endpoint_config_s)::value_type{endpoint_config_name,
                                                                 endpoint_config});
    }
}
void json_config::get_buffer_config(buffer_config_type &buffer) {
    noheap::buffer_bytes_type<BOOST_JSON_STACK_BUFFER_SIZE, noheap::ubyte>
        json_buffer_tmp;

    json::static_resource json_mr(json_buffer_tmp.data(), json_buffer_tmp.size());
    json::serializer      sz(&json_mr);
    buffer = {};

    json::value  data;
    json::object global_object;

    global_object.emplace(listen_port_string, config.listen_port);
    global_object.emplace(on_ipv6_string, config.on_ipv6);
    global_object.emplace(privk_string, std::string_view(noheap::hex_encode(
                                            config.local_config.keypair.priv)));
    global_object.emplace(pubk_string, std::string_view(noheap::hex_encode(
                                           config.local_config.keypair.pub)));

    for (const auto &endpoint_config_pair : config.endpoint_config_s) {
        decltype(auto) endpoint_config_name = endpoint_config_pair.first;
        decltype(auto) endpoint_config      = endpoint_config_pair.second;

        global_object.emplace(
            std::string_view(endpoint_config_name.data()),
            json::object{
                {ipv6_string, endpoint_config.endpoint.v == network::ipv::v6},
                {address_string, std::string_view(network::utils::bytes_address_to_string(
                                                      endpoint_config.endpoint.address,
                                                      endpoint_config.endpoint.v)
                                                      .data())},
                {port_string, endpoint_config.endpoint.port},
                {role_string, noise::get_noise_role_string(endpoint_config.local_role)},

                {pubk_string,
                 endpoint_config.public_key
                         != std::decay_t<decltype(endpoint_config.public_key)>{}
                     ? std::string_view(noheap::hex_encode(endpoint_config.public_key))
                     : ""},
                {psk_string,
                 std::string_view(
                     noheap::to_buffer<const noheap::buffer_chars_type<
                         noheap::buffer_size<decltype(endpoint_config.pre_shared_key)>>>(
                         endpoint_config.pre_shared_key)
                         .data())}});
    }

    data.emplace_object() = global_object;

    sz.reset(&data);
    sz.read(buffer.data(), buffer.size());
}
config_type &json_config::get_config() {
    return config;
}
template<typename T>
T json_config::get_object_field(const json::object    &object,
                                const std::string_view field_string) {
    if (!object.contains(field_string))
        log.throw_and_log<noheap::runtime_error>({}, "Field[{}] does not existed.",
                                                 field_string);
    decltype(auto) field = object.at(field_string);
    if constexpr (std::same_as<T, bool>) {
        if (!field.is_bool())
            log.throw_and_log<noheap::runtime_error>({}, "Field[{}] must be bool.",
                                                     field_string);
        return field.as_bool();
    } else if constexpr (std::is_integral_v<T>) {
        if (!field.is_int64())
            log.throw_and_log<noheap::runtime_error>({}, "Field[{}] must be number.",
                                                     field_string);
        return field.as_int64();
    } else if constexpr (std::same_as<T, std::string_view>) {
        if (!field.is_string())
            log.throw_and_log<noheap::runtime_error>({}, "Field[{}] must be string.",
                                                     field_string);
        return field.as_string();
    } else
        static_assert(false, "Invalid T type.");
}

#endif
