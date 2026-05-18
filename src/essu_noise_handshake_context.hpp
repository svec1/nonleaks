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
    inline noise_handshake_context(
        noise::noise_role _role, noise::buffer_prologue_extention_type _ext,
        const noise_context_type::buffer_key_type &_remote_public_key,
        const noise::buffer_pre_shared_key_type   &_pre_shared_key,
        const noise_context_type::keypair_type    &_local_keypair);
    inline noise_handshake_context(noise_handshake_context &&other);

public:
    inline void init_packet(packet_type &pckt);
    inline void handle_packet(packet_type &&pckt);

    inline bool                                       is_complete() const;
    inline noise::noise_action                        get_action() const;
    inline std::uint16_t                              get_available_batch_number() const;
    inline std::uint64_t                              get_handshake_id() const;
    inline const noise_context_type::buffer_key_type &get_remote_public_key() const;
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

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NOISE_HANDSHAKE_CONTEXT");
    static constexpr log_handler log{buffer_owner};

private:
    status_enum status;

    noise_context_type                        noise_context;
    typename noise_context_type::cipher_state payload_cipher_state;
    typename noise_context_type::cipher_state header_cipher_state_sender;
    typename noise_context_type::cipher_state header_cipher_state_receiver;
    typename noise_context_type::random_state random_state;
    noise_context_type::hash_state            hash_state;

    noise::noise_role                       role;
    noise::buffer_prologue_extention_type   ext;
    noise_context_type::buffer_key_type     remote_public_key;
    noise::buffer_pre_shared_key_type       pre_shared_key;
    const noise_context_type::keypair_type &local_keypair;

    typename noise::buffer_handshake_packet_type buffer_handshake_message{};
    std::size_t                                  offset_noise_handshake_unit;
    bool                                         fragmentation;

    noise_context_type::buffer_key_type                  ephemeral_obfs_key{};
    typename noise::buffer_handshake_payload_type        handshake_payload{};
    typename noise_context_type::hash_state::buffer_type handshake_hash{};
    buffer_unique_value_type                             unique_value{};
    buffer_unique_value_type                             unique_value_previous{};
    std::uint16_t                                        available_batch_number;
    std::uint64_t                                        handshake_id;
};

} // namespace essu
essu::noise_handshake_context::noise_handshake_context(noise_handshake_context &&other)
    : noise_handshake_context(other.role, other.ext, other.remote_public_key,
                              other.pre_shared_key, other.local_keypair) {
}
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

	set_dummy_packet(pckt);
    auto &control_unit = get_control_unit(pckt);

    // Gets noise message
    if (!fragmentation) {
        // Generates random value
        if (status == status_enum::HS3) {
            random_state.padding_buffer.set(
                {handshake_payload.data(), handshake_payload.size()}, 0);
            random_state.pad();
            noise_context.get_handshake_payload_buffer().set(
                {handshake_payload.data(), handshake_payload.size()},
                handshake_payload.size());
        }

        noise_context.get_handshake_buffer().set(
            {buffer_handshake_message.data(), buffer_handshake_message.size()}, 0);
        noise_context.set_handshake_message();

        // Adds ephemeral key obfuscation on ephemeral key
        if (status == status_enum::HS1) {
            std::transform(buffer_handshake_message.begin(),
                           buffer_handshake_message.begin() + ephemeral_obfs_key.size(),
                           ephemeral_obfs_key.data(), buffer_handshake_message.begin(),
                           std::bit_xor{});
        }
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

    auto &control_unit = get_control_unit(pckt);

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
        this->log.throw_exception("Invalid type of handshake packet.");

    // Copies accepted unit to buffer of noise handshake message
    std::copy(control_unit.buffer.begin(), control_unit.buffer.end(),
              buffer_handshake_message.begin() + offset_noise_handshake_unit);
    offset_noise_handshake_unit += control_unit.buffer.size();

    // If fragmentation
    if (payload_size >= offset_noise_handshake_unit)
        return;

    if (status == status_enum::HS1)
        // Deletes ephemeral key obfuscation
        std::transform(buffer_handshake_message.begin(),
                       buffer_handshake_message.begin() + ephemeral_obfs_key.size(),
                       ephemeral_obfs_key.begin(), buffer_handshake_message.begin(),
                       std::bit_xor{});
    else if (status == status_enum::HS3)
        // Sets buffer to get random value
        noise_context.get_handshake_payload_buffer().set(
            {handshake_payload.data(), handshake_payload.size()}, 0);

    // Sets noise message
    noise_context.get_handshake_buffer().set(
        {buffer_handshake_message.data(), offset_noise_handshake_unit}, payload_size);
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
typename essu::noise_context_type::random_state &
    essu::noise_handshake_context::get_random_state() {
    return random_state;
}
bool essu::noise_handshake_context::is_complete() const {
    return status == status_enum::COMPLETE;
}
noise::noise_action essu::noise_handshake_context::get_action() const {
    return fragmentation ? noise::noise_action::WRITE_MESSAGE
                         : noise_context.get_action();
}
std::uint16_t essu::noise_handshake_context::get_available_batch_number() const {
    return available_batch_number;
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
    ephemeral_obfs_key          = {};
    unique_value_previous       = unique_value;
    unique_value                = {};
    payload_cipher_state.init({});
    header_cipher_state_sender.init({});
    header_cipher_state_receiver.init({});
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
            this->log.throw_exception("Invalid remote public key from handshake.");
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
        this->log.throw_exception("Failed to handshake.");

    if (action == expected
        || (expected == noise::noise_action::WRITE_MESSAGE && fragmentation))
        return;

    if (action == noise::noise_action::WRITE_MESSAGE)
        this->log.throw_exception("Expected message to be sent.");
    else if (action == noise::noise_action::READ_MESSAGE)
        this->log.throw_exception("Expected message to be received.");
    else if (action == noise::noise_action::SPLIT)
        this->log.throw_exception("Expected to stop handshake.");
    else if (action == noise::noise_action::COMPLETE)
        this->log.throw_exception("Handshake already completed.");
    else if (action == noise::noise_action::NONE)
        this->log.throw_exception("Action is not required.");
    else
        this->log.abort_invalid_state();
}

// Generates ephemeral header obfuscation key + ephmeral obfuscation key for HS1
void essu::noise_handshake_context::generate_pair_ephemeral_obfs_key() {
    typename noise_context_type::buffer_key_type public_key{};
    const auto xor_public_key_with_buffer = [&](const auto &buffer) {
        std::transform(public_key.begin(),
                       public_key.begin()
                           + std::clamp<std::size_t>(buffer.size(), buffer.size(),
                                                     public_key.size()),
                       reinterpret_cast<const noheap::rbyte *>(buffer.begin()),
                       public_key.begin(), std::bit_xor{});
    };

    // Mixes the empty public key with own and remote public keys
    if (remote_public_key != noise_context_type::buffer_key_type{}) {
        xor_public_key_with_buffer(local_keypair.pub);
        xor_public_key_with_buffer(remote_public_key);
    }
    // Mixes the empty public key with the previous handshake payload
    xor_public_key_with_buffer(hash_state.get_hash(
        {unique_value_previous.data(), unique_value_previous.size()}));

    // Gets 32 bytes-hash of public key
    auto public_key_hash = noheap::clip_buffer<32, 0>(
        hash_state.get_hash({public_key.data(), public_key.size()}));

    // Generates keystream
    noise::buffer_type<noheap::buffer_size<noise_context_type::buffer_key_type> * 3
                       + noise_config.mac_size>
                                     keystream{};
    noise_context_type::cipher_state cipher_tmp;
    cipher_tmp.set_encrypt_key(public_key_hash);
    cipher_tmp.encrypt_buffer.set({keystream.data(), keystream.size()},
                                  keystream.size() - noise_config.mac_size);
    cipher_tmp.encrypt({});

    auto header_obfs_key1 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>, 0>(
            keystream);
    auto header_obfs_key2 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                            noheap::buffer_size<noise_context_type::buffer_key_type>>(
            keystream);

    if (role == noise::noise_role::INITIATOR) {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key1);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key2);
    } else {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key2);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key1);
    }

    ephemeral_obfs_key =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                            noheap::buffer_size<noise_context_type::buffer_key_type> * 2>(
            keystream);
}

// Generates posthandshake header obfuscation key + unique value
void essu::noise_handshake_context::generate_posthandshake_unique_values() {
    // Mixes the current handshake payload with the previous handshake payload
    std::transform(handshake_payload.begin(),
                   handshake_payload.begin() + unique_value_previous.size(),
                   unique_value_previous.begin(), handshake_payload.begin(),
                   std::bit_xor{});

    // Generates unique values
    std::decay_t<decltype(handshake_hash)> unique_value_two;
    hash_state.hkdf({handshake_hash.data(), handshake_hash.size()},
                    {handshake_payload.data(), handshake_payload.size()},
                    {unique_value.data(), unique_value.size()},
                    {unique_value_two.data(), unique_value_two.size()});
    // Handles the first unique_value
    {
        available_batch_number = std::clamp<std::uint16_t>(
            noheap::represent_bytes<std::uint16_t>(
                noheap::clip_buffer<sizeof(std::uint16_t), 0>(unique_value)),
            min_available_batch_number, max_available_batch_number);
        handshake_id = noheap::represent_bytes<std::uint64_t>(
            noheap::clip_buffer<sizeof(std::uint64_t), sizeof(std::uint16_t)>(
                unique_value));

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

        // Maximum initial value of nonce(counter block) is 2^48−1
        (*reinterpret_cast<std::uint64_t *>(value1.data())) >>= 16;
        (*reinterpret_cast<std::uint64_t *>(value2.data())) >>= 16;

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
        cipher_tmp.encrypt_buffer.set({keystream.data(), keystream.size()},
                                      keystream.size() - noise_config.mac_size);
        cipher_tmp.encrypt({});

        auto header_obfs_key1 =
            noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                                0>(keystream);
        auto header_obfs_key2 =
            noheap::clip_buffer<noheap::buffer_size<noise_context_type::buffer_key_type>,
                                noheap::buffer_size<noise_context_type::buffer_key_type>>(
                keystream);

        if (role == noise::noise_role::INITIATOR) {
            header_cipher_state_sender.set_encrypt_key(header_obfs_key1);
            header_cipher_state_receiver.set_encrypt_key(header_obfs_key2);
        } else {
            header_cipher_state_sender.set_encrypt_key(header_obfs_key2);
            header_cipher_state_receiver.set_encrypt_key(header_obfs_key1);
        }
    }
}
