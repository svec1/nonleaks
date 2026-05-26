#ifndef ESSU_BASE_HPP
#define ESSU_BASE_HPP

#include "network.hpp"
#include "noise.hpp"
#include "utils.hpp"

namespace essu {

constexpr std::size_t timeout_ms                   = 7500;
constexpr std::size_t keep_alive_ms                = 2500;
constexpr std::size_t packet_size                  = 1376;
constexpr std::size_t header_data_size             = 24;
constexpr std::size_t min_random_bytes_number      = 32;
constexpr std::size_t batch_units_number           = 4;
constexpr std::size_t control_unit_number          = 3;
constexpr std::size_t unit_per_rekey_number        = 6;
constexpr std::size_t batch_window_number          = 256;
constexpr std::size_t max_undecrypted_batch_number = 16;
constexpr std::size_t min_available_batch_number   = 1024;
constexpr std::size_t max_available_batch_number =
    (std::uint32_t(-1) - 3) / batch_units_number;
constexpr std::size_t max_available_handshake_number = std::uint16_t(-1);
constexpr std::size_t max_session_number             = 4;
constexpr std::size_t shared_value_number            = 7;
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
    enum class unit_type_enum : std::uint8_t {
        session_request = 0,
        session_created,
        session_confirmed,
        retry,
        hole_punch,
        data,
        dummy,
    };

    struct header_data_type {
        std::uint64_t  shared_value;
        std::uint32_t  number;
        std::uint32_t  key_iteration_number;
        unit_type_enum type;
        // Reserved
        std::uint8_t byte1;
        std::uint8_t byte2;
        std::uint8_t byte3;
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
struct protocol;
using packet_type = network::packet_native_type<extention_payload_data_type>;
using buffer_shared_value_s_type =
    noheap::buffer_type<std::uint64_t, shared_value_number>;

template<typename T>
concept Packet_type = std::same_as<std::decay_t<T>, packet_type>;

class base_error : public noheap::runtime_error {
protected:
    using runtime_error::runtime_error;
};
class protocol_error : public base_error {
public:
    using base_error::base_error;
};
class session_error : public base_error {
public:
    using base_error::base_error;
};

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
           || type == unit_type::unit_type_enum::session_confirmed
           || type == unit_type::unit_type_enum::retry;
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
    return get_control_unit(pckt).header.type == unit_type::unit_type_enum::dummy
           && !is_control_session_unit_type(pckt->units[0].header.type)
           && !is_control_session_unit_type(pckt->units[1].header.type)
           && !is_control_session_unit_type(pckt->units[3].header.type);
}
inline void set_dummy_unit(unit_type &unit) {
    unit.header.type = unit_type::unit_type_enum::dummy;
}
inline void set_control_session_unit(unit_type &unit, unit_type::unit_type_enum type) {
    if (!is_control_session_unit_type(type))
        return;
    unit.header.type = type;
}
inline void set_control_session_packet(packet_type              &pckt,
                                       unit_type::unit_type_enum type) {
    if (!is_control_session_unit_type(type))
        return;
    set_control_session_unit(get_control_unit(pckt), type);
}
inline void set_dummy_packet(packet_type &pckt) {
    pckt->units = {};
    for (decltype(auto) unit : pckt->units)
        set_dummy_unit(unit);
}

} // namespace essu
#endif
