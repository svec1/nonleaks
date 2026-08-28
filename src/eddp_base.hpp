#ifndef EDDP_BASE_HPP
#define EDDP_BASE_HPP

#include "essu_base.hpp"
#include "mldsa_native.hpp"

// Protocol description:
// 1. Node can be firewalled and nonfirewalled; Nonfirewalled node can become trust node, Firewalled node can become pseudo-trust node.
//    Trust node is provider of SFU-session or be STUN/RTUN to other nodes.
//    Pseudo-trust node is RTUN between two trust nodes.
// 2. When a node connects to trust node, the trust node have to create session or attach it to the existing one. 
// 3. When tust node creates session, initiator must decide a second trust node that needs be a pseudo-provider - this node will just be a RTUN to provider of session.
// 	  Initiator must decide a packet distribution - when and which packet will be shipped. 
// 4. Payload of a sent packet is encrypted for the endpoint and, in addition, is encrypted again between nodes.
namespace eddp {
constexpr std::size_t packet_size      = essu::payload_data_size;
constexpr std::size_t header_data_size = 33;
constexpr std::size_t buffer_data_size = packet_size - header_data_size;

using mldsa_wrapper = mldsa_native::mldsa_native_wrapper<std::to_array("EDDP")>;

struct [[gnu::packed]] payload_packet_type {
    enum class payload_packet_type_enum : std::uint8_t {
        session_request = 0,
        session_created,
        session_confirmed,
        node_resolved,
        data
    };
    struct [[gnu::packed]] header_data_type {
        std::uint64_t            destination_id;
        std::uint64_t            number;
        std::uint64_t            ack_through;
        std::uint64_t            bitmap;
        payload_packet_type_enum type;
    };

    static_assert(sizeof(header_data_type) == header_data_size, "Invalid header size.");

public:
    header_data_type                                           header;
    noheap::buffer_bytes_type<buffer_data_size, noheap::rbyte> buffer;
};

struct [[gnu::packed]] trust_node_info_type {
	essu::noise_context_type::buffer_key_type public_key;
	mldsa_wrapper::buffer_public_key sign_public_key;
};

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
