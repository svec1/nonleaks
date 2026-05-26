#ifndef EDDP_BASE_HPP
#define EDDP_BASE_HPP

#include "essu_base.hpp"
#include "mldsa_native.hpp"

namespace eddp {
constexpr std::size_t packet_size      = essu::payload_data_size;
constexpr std::size_t header_data_size = 40;
constexpr std::size_t buffer_data_size = packet_size - header_data_size;

struct payload_packet_type {
    enum class payload_packet_type_enum : std::uint64_t {
        session_request = 0,
        session_created,
        session_confirmed,
        node_resolved,
        data
    };
    struct header_data_type {
        payload_packet_type_enum type;
        std::uint64_t            destination_id;
        std::uint64_t            number;
        std::uint64_t            ack_through;
        std::uint64_t            bitmap;
    };

    static_assert(sizeof(header_data_type) == header_data_size, "Invalid header size.");

public:
    header_data_type                                           header;
    noheap::buffer_bytes_type<buffer_data_size, noheap::rbyte> buffer;
};

using mldsa_wrapper = mldsa_native::mldsa_native_wrapper<std::to_array("EDDP")>;

static inline void encapsulate_packet_in_essu_unit(essu::unit_type           &unit,
                                                   const payload_packet_type &packet) {
    std::memcpy(unit.buffer.data(), &packet, packet_size);
}
static inline void decapsulate_packet_from_essu_unit(const essu::unit_type &unit,
                                                     payload_packet_type   &packet) {
    std::memcpy(&packet, unit.buffer.data(), packet_size);
}

} // namespace eddp
#endif
