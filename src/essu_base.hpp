#ifndef ESSU_BASE_HPP
#define ESSU_BASE_HPP

#include "network.hpp"
#include "noise.hpp"
#include "utils.hpp"

namespace essu {

constexpr std::size_t timeout_ms                 = 7500;
constexpr std::size_t keep_alive_ms              = 2500;
constexpr std::size_t packet_size                = 1376;
constexpr std::size_t header_data_size           = 24;
constexpr std::size_t min_random_bytes_number    = 32;
constexpr std::size_t batch_units_number         = 4;
constexpr std::size_t control_unit_number        = 3;
constexpr std::size_t unit_per_rekey_number      = 6;
constexpr std::size_t batch_window_number        = 256;
constexpr std::size_t skip_batch_window_number   = 16;
constexpr std::size_t max_batch_handshake_number = 30;
constexpr std::size_t min_available_batch_number = 1024;
constexpr std::size_t max_available_batch_number =
    (std::uint32_t(-1) - 3) / batch_units_number;
constexpr std::size_t max_available_handshake_number = std::uint16_t(-1);
constexpr std::size_t max_session_number             = 4;
constexpr std::size_t unit_size                      = packet_size / batch_units_number;
constexpr std::size_t buffer_data_size               = unit_size - header_data_size;
constexpr std::size_t payload_data_size = buffer_data_size - min_random_bytes_number;

static constexpr noise::noise_context_config<
    noise::noise_pattern::XX_HFS, noise::dh_type::X25519, noise::dh_type::MLKEM768,
    noise::cipher_type::XCHACHAPOLY, noise::hash_type::SHA3512>
    noise_config;
using noise_context_type = noise::noise_context<noise_config>;

// Transport unit
struct unit_type {
public:
    enum class unit_type_enum : std::uint64_t {
        session_request = 0,
        session_created,
        session_confirmed,
        retry,
        hole_punch,
        data,
        dummy,
    };

    struct header_data_type {
        std::uint64_t  connection_id;
        unit_type_enum type;
        std::uint32_t  number;
        std::uint32_t  key_iteration_number;
    };

public:
    constexpr std::size_t buffer_size_without_mac() {
        return buffer_data_size - noise::get_mac_size<noise_config.cipher>();
    }
    static_assert(sizeof(header_data_type) == header_data_size, "Invalid header size.");

public:
    header_data_type header{};

    noheap::buffer_bytes_type<buffer_data_size, noheap::rbyte> buffer{};
};

// Packet(Batch)
struct extention_payload_data_type {
    noheap::buffer_type<unit_type, batch_units_number> units;
};
struct noise_handshake_context;
struct session_info_type;
struct session_info_type_extended;
class protocol;
class session_handler;
using packet_type = network::packet_native_type<extention_payload_data_type>;
using session_info_proxy_type =
    std::optional<std::reference_wrapper<session_info_type_extended>>;

template<typename T>
concept Packet_type = std::same_as<std::decay_t<T>, packet_type>;

class base_error : public noheap::runtime_error {
protected:
    using runtime_error::runtime_error;

public:
    void set_session_info(session_info_proxy_type _session_info) {
        session_info = _session_info;
    }
    decltype(auto) get_session_info() const {
        if (!session_info.has_value())
            throw noheap::logic_error("Failed to get session info.");

        return session_info.value();
    }

private:
    session_info_proxy_type session_info = std::nullopt;
};
class protocol_error : public base_error {
public:
    using base_error::base_error;
};
class session_error : public base_error {
public:
    using base_error::base_error;
};

namespace utils {
    inline auto get_string_unit_type(unit_type::unit_type_enum unit_type) noexcept {
        switch (unit_type) {
            case unit_type::unit_type_enum::session_request:
                return "session_request";
            case unit_type::unit_type_enum::session_created:
                return "session_created";
            case unit_type::unit_type_enum::session_confirmed:
                return "session_confirmed";
            case unit_type::unit_type_enum::retry:
                return "retry";
            case unit_type::unit_type_enum::hole_punch:
                return "hole_punch";
            case unit_type::unit_type_enum::data:
                return "data";
            case unit_type::unit_type_enum::dummy:
                return "dummy";
            default:
                throw noheap::runtime_error("Undefined unit type.");
        }
    }
} // namespace utils

template<Packet_type T>
inline decltype(auto) get_last_unit(T &&pckt) {
    return pckt->units[batch_units_number - 1];
}
template<Packet_type T>
inline decltype(auto) get_control_unit(T &&pckt) {
    return pckt->units[control_unit_number - 1];
}

inline bool is_control_session_unit_type(unit_type::unit_type_enum type) {
    return type == unit_type::unit_type_enum::session_request
           || type == unit_type::unit_type_enum::session_created
           || type == unit_type::unit_type_enum::session_confirmed;
}
inline bool is_control_payload_packet_type(const packet_type &pckt) {
    return get_control_unit(pckt).header.type == unit_type::unit_type_enum::data;
}
inline bool is_control_session_packet_type(const packet_type &pckt) {
    return is_control_session_unit_type(get_control_unit(pckt).header.type);
}
inline bool is_dummy_packet(const packet_type &pckt) {
    return (pckt->units[0].header.type == pckt->units[1].header.type
            && pckt->units[1].header.type == pckt->units[2].header.type
            && pckt->units[2].header.type == pckt->units[3].header.type
            && pckt->units[3].header.type == unit_type::unit_type_enum::dummy);
}
inline bool is_handshake_packet(const packet_type &pckt) {
    return is_control_session_packet_type(pckt)
           && pckt->units[0].header.type == unit_type::unit_type_enum::dummy
           && pckt->units[1].header.type == unit_type::unit_type_enum::dummy
           && pckt->units[3].header.type == unit_type::unit_type_enum::dummy;
}
inline bool is_posthandshake_packet(const packet_type &pckt) {
    return (get_control_unit(pckt).header.type == unit_type::unit_type_enum::dummy
            || get_control_unit(pckt).header.type == unit_type::unit_type_enum::retry)
           && !is_control_session_unit_type(pckt->units[0].header.type)
           && !is_control_session_unit_type(pckt->units[1].header.type)
           && !is_control_session_unit_type(pckt->units[3].header.type);
}
inline void set_control_session_packet(packet_type              &pckt,
                                       unit_type::unit_type_enum type) {
    if (!is_control_session_unit_type(type))
        return;
    get_control_unit(pckt).header.type = type;
}
inline void set_dummy_packet(packet_type &pckt) {
    for (decltype(auto) unit : pckt->units)
        unit.header.type = unit_type::unit_type_enum::dummy;
}

} // namespace essu
#endif
