#ifndef ESSU_NOISE_HANDSHAKE_CONTEXT_HPP
#define ESSU_NOISE_HANDSHAKE_CONTEXT_HPP

#include "essu_base.hpp"

namespace essu {

// Noise handshake context for establishing shared secret key
struct noise_handshake_context {
private:
    using buffer_unique_value_type =
        noise::buffer_type<noise_config.nonce_size * 2 + sizeof(std::uint64_t)
                           + sizeof(std::uint16_t)>;

    enum class status_enum : std::size_t {
        HS1 = 0,
        HS2,
        HS3,
        NEEDS_COMPLETE,
        COMPLETE,
    };

public:
    static constexpr noise::hash_type config_hash_type = noise_config.hash;
    static constexpr noise::hash_type current_state_hash_type =
        noise::hash_type::RIPEMD160;

    using buffer_config_hash_type =
        typename noise_context_type::hash_state<config_hash_type>::buffer_type;
    using buffer_current_state_hash_type =
        typename noise_context_type::hash_state<current_state_hash_type>::buffer_type;

public:
    inline noise_handshake_context(
        noise::noise_role _role, noise::buffer_prologue_extention_type _ext,
        const noise_context_type::buffer_key_type &_remote_public_key,
        const noise::buffer_pre_shared_key_type   &_pre_shared_key,
        const noise_context_type::keypair_type    &_local_keypair);

public:
    inline void init_packet(packet_type &pckt);
    inline void handle_packet(packet_type &&pckt);

    inline bool                                       is_complete() const;
    inline noise::noise_role                          get_role() const;
    inline noise::noise_action                        get_action() const;
    inline std::uint16_t                              get_available_batch_number() const;
    inline std::uint64_t                              get_handshake_id() const;
    inline buffer_current_state_hash_type             get_current_state_hash();
    inline const noise_context_type::buffer_key_type &get_remote_public_key() const;
    inline typename noise_context_type::hash_state<
        essu::noise_handshake_context::config_hash_type>                                               &
        get_hash_state();
    inline typename noise_context_type::cipher_state &get_payload_cipher_state();
    inline typename noise_context_type::cipher_state &get_header_cipher_state_sender();
    inline typename noise_context_type::cipher_state &get_header_cipher_state_receiver();
    inline typename noise_context_type::random_state &get_random_state();

    inline void start();
    inline void stop();

private:
    inline void check_noise_action(noise::noise_action expected);
    inline void generate_pair_ephemeral_obfs_key();
    inline void generate_posthandshake_unique_values();
    inline void update_current_state_hash();

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NOISE_HANDSHAKE_CONTEXT");
    static constexpr log_handler log{buffer_owner};

private:
    status_enum status;

    noise_context_type                                      noise_context;
    typename noise_context_type::cipher_state               payload_cipher_state;
    typename noise_context_type::cipher_state               header_cipher_state_sender;
    typename noise_context_type::cipher_state               header_cipher_state_receiver;
    typename noise_context_type::random_state               random_state;
    noise_context_type::hash_state<config_hash_type>        hash_state;
    noise_context_type::hash_state<current_state_hash_type> current_state_hash_state;

    noise::noise_role                       role;
    noise::buffer_prologue_extention_type   ext;
    noise_context_type::buffer_key_type     remote_public_key;
    noise::buffer_pre_shared_key_type       pre_shared_key;
    const noise_context_type::keypair_type &local_keypair;

    typename noise::buffer_handshake_packet_type buffer_handshake_message{};
    std::size_t                                  offset_noise_handshake_unit{};
    bool                                         fragmentation{};

    typename noise::buffer_handshake_payload_type handshake_payload{};
    buffer_config_hash_type                       handshake_hash{};
    buffer_unique_value_type                      unique_value{};
    std::uint16_t                                 available_batch_number{};
    std::uint64_t                                 handshake_id{};
};

} // namespace essu

essu::noise_handshake_context::noise_handshake_context(
    noise::noise_role _role, noise::buffer_prologue_extention_type _ext,
    const noise_context_type::buffer_key_type &_remote_public_key,
    const noise::buffer_pre_shared_key_type   &_pre_shared_key,
    const noise_context_type::keypair_type    &_local_keypair)
    : role(_role), ext(_ext), remote_public_key(_remote_public_key),
      pre_shared_key(_pre_shared_key), local_keypair(_local_keypair) {
}

void essu::noise_handshake_context::init_packet(packet_type &pckt) {
    check_noise_action(noise::noise_action::WRITE_MESSAGE);

    decltype(auto) control_unit = get_control_unit(pckt);

    // Gets noise message
    if (!fragmentation) {
        // Generates random value
        if (status == status_enum::HS3) {
            random_state.padding_buffer.set(handshake_payload, 0);
            random_state.pad();
            noise_context.get_handshake_payload_buffer().set(handshake_payload,
                                                             handshake_payload.size());
        }

        noise_context.get_handshake_buffer().set(buffer_handshake_message, 0);
        noise_context.set_handshake_message();
    }

    // Copy payload of the noise message
    std::copy(buffer_handshake_message.begin() + offset_noise_handshake_unit,
              buffer_handshake_message.begin() + offset_noise_handshake_unit
                  + control_unit.buffer.size(),
              reinterpret_cast<noheap::rbyte *>(control_unit.buffer.begin()));
    offset_noise_handshake_unit += control_unit.buffer.size();

    // Determines type of payload data
    if (status == status_enum::HS1)
        control_unit.header.type = unit_type::unit_type_enum::session_request;
    else if (status == status_enum::HS2)
        control_unit.header.type = unit_type::unit_type_enum::session_created;
    else if (status == status_enum::HS3)
        control_unit.header.type = unit_type::unit_type_enum::session_confirmed;
    else
        this->log.abort_invalid_state();

    // If fragmentation
    if (offset_noise_handshake_unit < noise_context.get_handshake_buffer().get().size) {
        fragmentation = true;
        return;
    }

    std::size_t occupied_bytes =
        (noise_context.get_handshake_buffer().get().size
         - (offset_noise_handshake_unit - control_unit.buffer.size()));
    random_state.padding_buffer.set(
        {control_unit.buffer.data(), control_unit.buffer.size()}, occupied_bytes);
    random_state.pad();

    buffer_handshake_message    = {};
    offset_noise_handshake_unit = 0;
    fragmentation               = false;
    status                      = status_enum(static_cast<std::size_t>(status) + 1);
}
void essu::noise_handshake_context::handle_packet(packet_type &&pckt) {
    check_noise_action(noise::noise_action::READ_MESSAGE);

    decltype(auto) control_unit = get_control_unit(pckt);

    // Determines size of payload data
    std::uint64_t payload_size = 0;
    if (status == status_enum::HS1)
        payload_size = noise_config.get_hs1_size();
    else if (status == status_enum::HS2)
        payload_size = noise_config.get_hs2_size();
    else if (status == status_enum::HS3)
        payload_size = noise_config.get_hs3_size();
    else
        this->log.abort_invalid_state();

    if ((status == status_enum::HS1
         && control_unit.header.type != unit_type::unit_type_enum::session_request)
        || (status == status_enum::HS2
            && control_unit.header.type != unit_type::unit_type_enum::session_created)
        || (status == status_enum::HS3
            && control_unit.header.type != unit_type::unit_type_enum::session_confirmed))
        this->log.throw_exception<protocol_error>("Invalid type of handshake packet.");

    // Copies accepted unit to buffer of noise handshake message
    std::copy(control_unit.buffer.begin(), control_unit.buffer.end(),
              buffer_handshake_message.begin() + offset_noise_handshake_unit);
    offset_noise_handshake_unit += control_unit.buffer.size();

    // If fragmentation
    if (payload_size > offset_noise_handshake_unit)
        return;

    if (status == status_enum::HS3)
        // Sets buffer to get random value
        noise_context.get_handshake_payload_buffer().set(handshake_payload, 0);

    // Sets noise message
    noise_context.get_handshake_buffer().set(buffer_handshake_message, payload_size);
    noise_context.get_handshake_message();

    buffer_handshake_message    = {};
    offset_noise_handshake_unit = 0;
    status                      = status_enum(static_cast<std::size_t>(status) + 1);
}

typename essu::noise_context_type::cipher_state &
    essu::noise_handshake_context::get_payload_cipher_state() {
    return payload_cipher_state;
}
typename essu::noise_context_type::cipher_state &
    essu::noise_handshake_context::get_header_cipher_state_sender() {
    return header_cipher_state_sender;
}
typename essu::noise_context_type::cipher_state &
    essu::noise_handshake_context::get_header_cipher_state_receiver() {
    return header_cipher_state_receiver;
}
typename essu::noise_context_type::hash_state<
    essu::noise_handshake_context::config_hash_type> &
    essu::noise_handshake_context::get_hash_state() {
    return hash_state;
}
typename essu::noise_context_type::random_state &
    essu::noise_handshake_context::get_random_state() {
    return random_state;
}
bool essu::noise_handshake_context::is_complete() const {
    return status == status_enum::COMPLETE;
}
noise::noise_role essu::noise_handshake_context::get_role() const {
    return noise_context.get_role();
}
noise::noise_action essu::noise_handshake_context::get_action() const {
    return fragmentation ? noise::noise_action::WRITE_MESSAGE
                         : noise_context.get_action();
}
std::uint16_t essu::noise_handshake_context::get_available_batch_number() const {
    return available_batch_number;
}
essu::noise_handshake_context::buffer_current_state_hash_type
    essu::noise_handshake_context::get_current_state_hash() {
    buffer_current_state_hash_type current_state_hash{};

    noheap::transform_buffers(current_state_hash, handshake_payload, std::bit_xor{});
    noheap::transform_buffers(current_state_hash, handshake_hash, std::bit_xor{});
    noheap::transform_buffers(current_state_hash, unique_value, std::bit_xor{});
    noheap::transform_buffers(
        current_state_hash,
        noheap::to_buffer<const noheap::buffer_bytes_type<sizeof(available_batch_number),
                                                          noheap::rbyte>>(
            available_batch_number),
        std::bit_xor{});
    noheap::transform_buffers(
        current_state_hash,
        noheap::to_buffer<
            const noheap::buffer_bytes_type<sizeof(handshake_id), noheap::rbyte>>(
            handshake_id),
        std::bit_xor{});
    if (payload_cipher_state.has_encrypt_key()) {
        noheap::transform_buffers(
            current_state_hash, payload_cipher_state.get_encrypt_nonce(), std::bit_xor{});
        noheap::transform_buffers(
            current_state_hash, payload_cipher_state.get_decrypt_nonce(), std::bit_xor{});
    }

    return current_state_hash_state.get_hash(hash_state.get_hash(current_state_hash));
}
std::uint64_t essu::noise_handshake_context::get_handshake_id() const {
    return handshake_id;
}
const essu::noise_context_type::buffer_key_type &
    essu::noise_handshake_context::get_remote_public_key() const {
    return remote_public_key;
}

void essu::noise_handshake_context::start() {
    check_noise_action(noise::noise_action::NONE);

    status                      = status_enum::HS1;
    offset_noise_handshake_unit = 0;
    fragmentation               = false;
    buffer_handshake_message    = {};
    handshake_payload           = {};
    handshake_hash              = {};
    available_batch_number      = 0;
    header_cipher_state_sender.init({});
    header_cipher_state_receiver.init({});
    payload_cipher_state.init({});
    random_state.reseed();

    generate_pair_ephemeral_obfs_key();

    noise_context.init(role);
    noise_context.set_prologue(ext);
    noise_context.set_local_keypair(local_keypair);
    noise_context.set_remote_public_key(remote_public_key);
    noise_context.set_pre_shared_key(pre_shared_key);
    noise_context.start();
}
void essu::noise_handshake_context::stop() {
    check_noise_action(noise::noise_action::SPLIT);

    // If local rpk is non empty it checks rpk from handshake, for XX pattern
    if (auto handshake_remote_public_key = noise_context.get_remote_public_key();
        handshake_remote_public_key != remote_public_key) {
        if (remote_public_key == noise_context_type::buffer_key_type{})
            remote_public_key = handshake_remote_public_key;
        else
            this->log.throw_exception<protocol_error>(
                "Invalid remote public key from handshake.");
    }

    noise_context.stop();
    noise_context.get_cipher_state(payload_cipher_state);
    handshake_hash = noise_context.get_handshake_hash();
    generate_posthandshake_unique_values();

    noise_context.dump();

    status = status_enum(static_cast<std::size_t>(status) + 1);
}

void essu::noise_handshake_context::check_noise_action(noise::noise_action expected) {
    auto action = noise_context.get_action();

    if (action == noise::noise_action::FAILED)
        this->log.throw_exception<protocol_error>("Failed to handshake.");

    if (action == expected
        || (expected == noise::noise_action::WRITE_MESSAGE && fragmentation))
        return;

    if (action == noise::noise_action::WRITE_MESSAGE)
        this->log.throw_exception<noheap::runtime_error>("Expected message to be sent.");
    else if (action == noise::noise_action::READ_MESSAGE)
        this->log.throw_exception<noheap::runtime_error>(
            "Expected message to be received.");
    else if (action == noise::noise_action::SPLIT)
        this->log.throw_exception<noheap::runtime_error>("Expected to stop handshake.");
    else if (action == noise::noise_action::COMPLETE)
        this->log.throw_exception<noheap::runtime_error>("Handshake already completed.");
    else if (action == noise::noise_action::NONE)
        this->log.throw_exception<noheap::runtime_error>("Action is not required.");
    else
        this->log.abort_invalid_state();
}

// Generates ephemeral header obfuscation key + ephmeral obfuscation key for HS1
void essu::noise_handshake_context::generate_pair_ephemeral_obfs_key() {
    typename noise_context_type::buffer_key_type public_key{};

    // Mixes the empty public key with own and remote public keys
    if (remote_public_key != noise_context_type::buffer_key_type{}) {
        noheap::transform_buffers(public_key, local_keypair.pub, std::bit_xor{});
        noheap::transform_buffers(public_key, remote_public_key, std::bit_xor{});
    }
    noheap::transform_buffers(public_key, unique_value, std::bit_xor{});

    // Gets 32 bytes-hash of public key
    auto public_key_hash = noheap::clip_buffer<32, 0>(hash_state.get_hash(public_key));

    // Generates keystream
    noise::buffer_type<noheap::buffer_size<noise_context_type::buffer_key_type> * 2
                       + sizeof(std::uint64_t) + noise_config.mac_size>
                                     keystream{};
    noise_context_type::cipher_state cipher_tmp;
    cipher_tmp.encrypt_buffer.set(keystream, keystream.size() - noise_config.mac_size);
    cipher_tmp.set_encrypt_key(public_key_hash);
    cipher_tmp.encrypt({});

    // Gets two ephemeral header obfuscation keys
    auto header_obfs_key1 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>, 0>(
            keystream);
    auto header_obfs_key2 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                            noheap::buffer_size<noise_context_type::buffer_key_type>>(
            keystream);

    // Sets these keys and reset nonces of header cipher states
    if (role == noise::noise_role::INITIATOR) {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key1);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key2);
    } else {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key2);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key1);
    }
    header_cipher_state_sender.set_encrypt_nonce({});
    header_cipher_state_receiver.set_encrypt_nonce({});

    // Sets available batch number and handshake id
    available_batch_number = 0;
    handshake_id ^= noheap::represent_bytes<std::uint64_t>(
        noheap::clip_buffer<sizeof(std::uint64_t),
                            noheap::buffer_size<noise_context_type::buffer_key_type> * 2>(
            keystream));
}

// Generates posthandshake header obfuscation key + unique value
void essu::noise_handshake_context::generate_posthandshake_unique_values() {
    // Mixes the current handshake payload with the unique value
    noheap::transform_buffers(handshake_payload, unique_value, std::bit_xor{});

    // Generates unique values
    std::decay_t<decltype(handshake_hash)> unique_value_one, unique_value_two;
    hash_state.hkdf(handshake_hash, handshake_payload, unique_value_one,
                    unique_value_two);

    // Mixes the unique value with new
    noheap::transform_buffers(unique_value, unique_value_one, std::bit_xor{});

    // Handles the first unique_value
    {
        available_batch_number = std::clamp<std::uint16_t>(
            noheap::represent_bytes<std::uint16_t>(
                noheap::clip_buffer<sizeof(std::uint16_t), 0>(unique_value)),
            min_available_batch_number, std::uint16_t(max_available_batch_number));
        handshake_id ^= noheap::represent_bytes<std::uint64_t>(
            noheap::clip_buffer<sizeof(std::uint64_t), sizeof(std::uint16_t)>(
                unique_value));

        // Gets two nonce values
        noise_context_type::buffer_nonce_type value1 =
            noheap::represent_bytes<noise_context_type::buffer_nonce_type>(
                noheap::clip_buffer<
                    noheap::buffer_size<noise_context_type::buffer_nonce_type>,
                    sizeof(std::uint16_t) + sizeof(std::uint64_t)>(unique_value));
        noise_context_type::buffer_nonce_type value2 =
            noheap::represent_bytes<noise_context_type::buffer_nonce_type>(
                noheap::clip_buffer<
                    noheap::buffer_size<noise_context_type::buffer_nonce_type>,
                    sizeof(std::uint16_t) + sizeof(std::uint64_t)
                        + noheap::buffer_size<noise_context_type::buffer_nonce_type>>(
                    unique_value));

        // Sets these nonce values
        if (noise_context.get_role() == noise::noise_role::INITIATOR) {
            payload_cipher_state.set_encrypt_nonce(value1);
            payload_cipher_state.set_decrypt_nonce(value2);
        } else {
            payload_cipher_state.set_encrypt_nonce(value2);
            payload_cipher_state.set_decrypt_nonce(value1);
        }
    }

    // Handles the second unique_value
    {
        // Generates keystream - the header obfuscation key
        noise::buffer_type<noheap::buffer_size<noise_context_type::buffer_key_type> * 2
                           + noise_config.mac_size>
                                         keystream{};
        noise_context_type::cipher_state cipher_tmp;
        cipher_tmp.set_encrypt_key(
            noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                                0>(unique_value_two));
        cipher_tmp.encrypt_buffer.set(keystream,
                                      keystream.size() - noise_config.mac_size);
        cipher_tmp.encrypt({});

        // Gets two header obfuscation keys
        auto header_obfs_key1 =
            noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                                0>(keystream);
        auto header_obfs_key2 =
            noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                                noheap::buffer_size<noise_context_type::buffer_key_type>>(
                keystream);

        // Sets these keys and reset nonces of header cipher states
        if (role == noise::noise_role::INITIATOR) {
            header_cipher_state_sender.set_encrypt_key(header_obfs_key1);
            header_cipher_state_receiver.set_encrypt_key(header_obfs_key2);
        } else {
            header_cipher_state_sender.set_encrypt_key(header_obfs_key2);
            header_cipher_state_receiver.set_encrypt_key(header_obfs_key1);
        }
        header_cipher_state_sender.set_encrypt_nonce({});
        header_cipher_state_receiver.set_encrypt_nonce({});
    }
}

#endif
