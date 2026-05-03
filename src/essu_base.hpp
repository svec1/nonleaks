#ifndef ESSU_BASE_HPP
#define ESSU_BASE_HPP

#include "network.hpp"
#include "noise.hpp"
#include "utils.hpp"

namespace essu {

constexpr std::size_t packet_size                     = 1376;
constexpr std::size_t header_data_size                = 16;
constexpr std::size_t min_random_bytes_number         = 64;
constexpr std::size_t batch_units_number              = 4;
constexpr std::size_t units_per_rekey_number          = 17;
constexpr std::size_t batches_window_number           = 16;
constexpr std::size_t max_undecrypted_batches_number  = 16;
constexpr std::size_t max_available_batches_number    = 10;
constexpr std::size_t max_available_handshakes_number = 10000;
constexpr std::size_t unit_size                       = packet_size / batch_units_number;
constexpr std::size_t buffer_data_size                = unit_size - header_data_size;
constexpr std::size_t payload_data_size = buffer_data_size - min_random_bytes_number;

static constexpr noise::noise_context_config<
    noise::noise_pattern::XX_HFS, noise::ecdh_type::X25519_KYBER1024,
    noise::cipher_type::XCHACHAPOLY, noise::hash_type::SHA3512>
    noise_config;
using noise_context_type = noise::noise_context<noise_config>;

// Transport unit
struct unit_type {
public:
    enum class unit_type_enum : noheap::ubyte {
        session_request = 0,
        session_created,
        session_confirmed,
        token_request,
        hole_punch,
        data,
        dummy,
        retry,
    };
    enum class flag_type_enum : noheap::ubyte {
        none = 0,
        wait_next,
    };

    struct header_data_type {
        std::uint64_t  number;
        std::uint32_t  key_iteration_number;
        unit_type_enum type;
        flag_type_enum flag;
        // Reserved
        std::uint8_t byte1;
        std::uint8_t byte2;
    };

public:
    static constexpr std::size_t buffer_size_without_mac =
        buffer_data_size - noise::get_mac_size<noise_config.cipher>();

    static_assert(sizeof(header_data_type) == header_data_size,
                  "Header size is invalid.");

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
struct protocol_type;
using packet_type = network::packet_native_type<extention_payload_data_type>;

} // namespace essu
#endif
