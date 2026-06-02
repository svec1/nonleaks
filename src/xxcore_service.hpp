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
    inline void set_buffer_config(const buffer_config_type &buffer, bool new_keypair);
    inline void get_buffer_config(buffer_config_type &buffer);

    inline config_type &get_config();

private:
    template<typename T>
    inline T get_object_field(const json::object    &value,
                              const std::string_view field_string);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("JSON_CONFIG");
    static constexpr log_handler log{buffer_owner};

private:
    config_type config{};
};

class xxcore_service {
public:
    static constexpr std::size_t workers_number = essu::max_session_number + 2;

    using udp_stream = network::udp_stream<essu::session_handler>;

public:
    inline xxcore_service(config_type &config);
    inline void run();

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("UUV_SERVICE");
    static constexpr log_handler log{buffer_owner};

private:
    config_type     &config;
    asio::io_context io;
};

xxcore_service::xxcore_service(config_type &_config) : config(_config) {
}

void xxcore_service::run() {
    noheap::buffer_type<future_wrapper<void>, workers_number>  workers;
    asio::executor_work_guard<asio::io_context::executor_type> work_guard(
        io.get_executor());

    for (auto &worker : workers)
        worker = typename std::decay_t<decltype(worker)>{[this] { this->io.run(); }};

    {
        udp_stream  stream(io, config.listen_port, config.local_config);
        scope_guard session_stop([&] {
            io.stop();
            stream.close();
        });

        decltype(auto) act = stream.get_action();
        for (auto &endpoint_config_pair : config.endpoint_config_s) {
            act.add_endpoint_config(endpoint_config_pair.second);
            act.register_session(endpoint_config_pair.second);
        }

        stream.open(config.on_ipv6 ? network::ipv::v4v6 : network::ipv::v4);

        log.to_all("Start...");

        // For testing: will be deleted
        {
            while (act.is_valid()) {
                try {
                    decltype(auto) session_list = act.get_session_list();
                    for (decltype(auto) it = session_list.begin();
                         it < session_list.end(); ++it) {
                        decltype(auto) session_info = *it;

                        act.handle(session_info);

                        essu::packet_type pckt;
                        essu::set_dummy_packet(pckt);
                        act.push_packet({pckt, session_info});
                    }
                    if (act.exist_received_packets())
                        act.pop_packet();

                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                } catch (const essu::base_error &excp) {
                    decltype(auto) session_info = excp.get_session_info();
                    session_info.log.get_log_handler().to_all("{}", excp.what());
                    act.remove_session(session_info);
                }
            }
            throw act.get_error();
        }

        log.to_all("Stop.");
    }

    for (auto &worker : workers)
        worker.get();
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
            throw noheap::runtime_error(buffer_owner,
                                        "The key[{}] field has a large size", field_name);

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
            throw noheap::runtime_error(buffer_owner, "Endpoint[{}] is already exist.",
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
        throw noheap::runtime_error(buffer_owner, "Field[{}] does not existed.",
                                    field_string);
    decltype(auto) field = object.at(field_string);
    if constexpr (std::same_as<T, bool>) {
        if (!field.is_bool())
            throw noheap::runtime_error(buffer_owner, "Field[{}] must be bool.",
                                        field_string);
        return field.as_bool();
    } else if constexpr (std::is_integral_v<T>) {
        if (!field.is_int64())
            throw noheap::runtime_error(buffer_owner, "Field[{}] must be number.",
                                        field_string);
        return field.as_int64();
    } else if constexpr (std::same_as<T, std::string_view>) {
        if (!field.is_string())
            throw noheap::runtime_error(buffer_owner, "Field[{}] must be string.",
                                        field_string);
        return field.as_string();
    } else
        static_assert(false, "Invalid T type.");
}

#endif
