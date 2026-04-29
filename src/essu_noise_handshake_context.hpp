#include "essu_base.hpp"

namespace essu {

// Noise handshake context for establishing shared secret key
struct noise_handshake_context {
public:
    using buffer_unique_value_type = noise::buffer_type<32>;

    enum class status_enum : std::size_t {
        hs1 = 0,
        hs2,
        hs3,
        is_complete,
    };

public:
    noise_handshake_context() = default;
    noise_handshake_context(noise_handshake_context &&other);
    noise_handshake_context &operator=(noise_handshake_context &&other);
    noise_handshake_context(noise::noise_role role, noise::prologue_extention_type ext,
                            const noise::pre_shared_key_type       &pre_shared_key,
                            const noise_context_type::keypair_type &local_keypair,
                            const noise_context_type::dh_key_type  &remote_public_key);

public:
    void init_packet(packet_type &pckt);
    void process_packet(packet_type &&pckt);

    bool                                       is_complete() const;
    noise::noise_action                        get_action() const;
    noise::noise_role                          get_role() const;
    buffer_unique_value_type                   get_unique_value() const;
    const noise_context_type::dh_key_type     &get_remote_public_key() const;
    typename noise_context_type::cipher_state &get_payload_cipher_state();
    typename noise_context_type::cipher_state &get_header_cipher_state_sender();
    typename noise_context_type::cipher_state &get_header_cipher_state_receiver();
    typename noise_context_type::random_state &get_random_state();

    void start();
    void stop();

private:
    void check_noise_action(noise::noise_action expected);
    void generate_pair_ephemeral_obfs_key();
    void generate_posthandshake_unique_values();

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NOISE_HANDSHAKE_CONTEXT");
    static constexpr log_handler log{buffer_owner};

private:
    status_enum status;

    noise_context_type                        noise_ctx;
    typename noise_context_type::cipher_state payload_cipher_state;
    typename noise_context_type::cipher_state header_cipher_state_sender;
    typename noise_context_type::cipher_state header_cipher_state_receiver;
    typename noise_context_type::random_state random_state;
    noise_context_type::hash_state            hash_state;

    noise::noise_role                role;
    noise::prologue_extention_type   ext;
    noise::pre_shared_key_type       pre_shared_key;
    noise_context_type::keypair_type local_keypair;
    noise_context_type::dh_key_type  remote_public_key;

    typename noise::buffer_handshake_packet_type buffer_handshake_message{};
    std::size_t                                  offset_noise_handshake_unit;
    bool                                         fragmentation;

    noise_context_type::dh_key_type                      ephemeral_obfs_key{};
    typename noise::buffer_handshake_payload_type        handshake_payload{};
    typename noise_context_type::hash_state::buffer_type handshake_hash{};
    buffer_unique_value_type                             unique_value{};
};

} // namespace essu
essu::noise_handshake_context::noise_handshake_context(noise_handshake_context &&other)
    : noise_handshake_context(other.role, other.ext, other.pre_shared_key,
                              other.local_keypair, other.remote_public_key) {
}
essu::noise_handshake_context::noise_handshake_context(
    noise::noise_role _role, noise::prologue_extention_type _ext,
    const noise::pre_shared_key_type       &_pre_shared_key,
    const noise_context_type::keypair_type &_local_keypair,
    const noise_context_type::dh_key_type  &_remote_public_key)
    : role(_role), ext(_ext), pre_shared_key(_pre_shared_key),
      local_keypair(_local_keypair), remote_public_key(_remote_public_key) {
}
essu::noise_handshake_context &
    essu::noise_handshake_context::operator=(noise_handshake_context &&other) {
    role              = other.role;
    ext               = other.ext;
    pre_shared_key    = other.pre_shared_key;
    local_keypair     = other.local_keypair;
    remote_public_key = other.remote_public_key;

    return *this;
}
void essu::noise_handshake_context::init_packet(packet_type &pckt) {
    check_noise_action(noise::noise_action::WRITE_MESSAGE);

    auto &payload_unit = pckt->units[0];

    // Handshake does not need to send 2 units
    pckt->units[1].header.type = unit_type::unit_type_enum::dummy;

    // Gets noise message
    if (!fragmentation) {
        // Generates random value
        if (status == status_enum::hs3) {
            handshake_payload =
                noheap::to_buffer<std::decay_t<decltype(handshake_payload)>>(
                    noheap::get_random_bytes<
                        noheap::buffer_size<decltype(handshake_payload)>>());
            noise_ctx.get_handshake_payload_buffer().set(
                {handshake_payload.data(), handshake_payload.size()},
                handshake_payload.size());
        }

        noise_ctx.get_handshake_buffer().set(
            {buffer_handshake_message.data(), buffer_handshake_message.size()}, 0);
        noise_ctx.set_handshake_message();

        // Adds ephemeral key obfuscation on ephemeral key
        if (status == status_enum::hs1) {
            std::transform(buffer_handshake_message.begin(),
                           buffer_handshake_message.begin() + ephemeral_obfs_key.size(),
                           ephemeral_obfs_key.data(), buffer_handshake_message.begin(),
                           std::bit_xor{});
        }
    }

    // Copy payload of the noise message
    std::copy(buffer_handshake_message.begin() + offset_noise_handshake_unit,
              buffer_handshake_message.begin() + offset_noise_handshake_unit
                  + payload_unit.buffer.size(),
              reinterpret_cast<noheap::rbyte *>(payload_unit.buffer.begin()));
    offset_noise_handshake_unit += payload_unit.buffer.size();

    // Determines type of payload data
    if (status == status_enum::hs1)
        payload_unit.header.type = unit_type::unit_type_enum::session_request;
    else if (status == status_enum::hs2)
        payload_unit.header.type = unit_type::unit_type_enum::session_created;
    else if (status == status_enum::hs3)
        payload_unit.header.type = unit_type::unit_type_enum::session_confirmed;
    else
        this->log.throw_exception("Unexpected behaviour during the noise handshake.");

    // If fragmentation
    if (offset_noise_handshake_unit < noise_ctx.get_handshake_buffer().get().size) {
        payload_unit.header.flag = decltype(payload_unit.header.flag)::wait_next;
        fragmentation            = true;
        return;
    }

    buffer_handshake_message    = {};
    offset_noise_handshake_unit = 0;
    fragmentation               = false;

    status = status_enum(static_cast<std::size_t>(status) + 1);
}
void essu::noise_handshake_context::process_packet(packet_type &&pckt) {
    check_noise_action(noise::noise_action::READ_MESSAGE);

    auto &payload_unit = pckt->units[0];

    // Determines size of payload data
    std::uint64_t payload_size;
    if (status == status_enum::hs1
        && payload_unit.header.type == unit_type::unit_type_enum::session_request)
        payload_size = unit_config_type::hs1_size;
    else if (status == status_enum::hs2
             && payload_unit.header.type == unit_type::unit_type_enum::session_created)
        payload_size = unit_config_type::hs2_size;
    else if (status == status_enum::hs3
             && payload_unit.header.type == unit_type::unit_type_enum::session_confirmed)
        payload_size = unit_config_type::hs3_size;
    else
        this->log.throw_exception("Unexpected behaviour during the noise handshake.");

    // Copies accepted unit to buffer of noise handshake message
    std::copy(payload_unit.buffer.begin(), payload_unit.buffer.end(),
              buffer_handshake_message.begin() + offset_noise_handshake_unit);
    offset_noise_handshake_unit += payload_unit.buffer.size();

    // If fragmentation
    if (payload_size >= payload_unit.buffer.size()
        && payload_unit.header.flag == decltype(payload_unit.header.flag)::wait_next)
        return;

    if (status == status_enum::hs1)
        // Deletes ephemeral key obfuscation
        std::transform(buffer_handshake_message.begin(),
                       buffer_handshake_message.begin() + ephemeral_obfs_key.size(),
                       ephemeral_obfs_key.begin(), buffer_handshake_message.begin(),
                       std::bit_xor{});
    else if (status == status_enum::hs3)
        // Sets buffer to get random value
        noise_ctx.get_handshake_payload_buffer().set(
            {handshake_payload.data(), handshake_payload.size()}, 0);

    // Sets noise message
    noise_ctx.get_handshake_buffer().set(
        {buffer_handshake_message.data(), offset_noise_handshake_unit}, payload_size);
    noise_ctx.get_handshake_message();

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
    return status == status_enum::is_complete;
}
noise::noise_action essu::noise_handshake_context::get_action() const {
    return fragmentation ? noise::noise_action::WRITE_MESSAGE : noise_ctx.get_action();
}
noise::noise_role essu::noise_handshake_context::get_role() const {
    return role;
}
essu::noise_handshake_context::buffer_unique_value_type
    essu::noise_handshake_context::get_unique_value() const {
    return unique_value;
}
const essu::noise_context_type::dh_key_type &
    essu::noise_handshake_context::get_remote_public_key() const {
    return remote_public_key;
}

void essu::noise_handshake_context::start() {
    check_noise_action(noise::noise_action::NONE);

    status                      = status_enum::hs1;
    offset_noise_handshake_unit = 0;
    fragmentation               = false;
    buffer_handshake_message    = {};
    handshake_payload           = {};
    handshake_hash              = {};
    payload_cipher_state.init({});
    header_cipher_state_sender.init({});
    header_cipher_state_receiver.init({});
    random_state.reseed();

    generate_pair_ephemeral_obfs_key();
    unique_value       = {};
    ephemeral_obfs_key = {};

    noise_ctx.init(role);
    noise_ctx.set_prologue(ext);
    noise_ctx.set_local_keypair(local_keypair);
    noise_ctx.set_remote_public_key(remote_public_key);
    noise_ctx.set_pre_shared_key(pre_shared_key);
    noise_ctx.start();
}
void essu::noise_handshake_context::stop() {
    check_noise_action(noise::noise_action::SPLIT);

    // If local rpk is non empty it checks rpk from handshake, for XX pattern
    if (auto handshake_remote_public_key = noise_ctx.get_remote_public_key();
        handshake_remote_public_key != remote_public_key) {
        if (remote_public_key == noise_context_type::dh_key_type{})
            remote_public_key = handshake_remote_public_key;
        else
            this->log.throw_exception("Remote public key from handshake is invalid.");
    }

    handshake_hash = noise_ctx.get_handshake_hash();
    generate_posthandshake_unique_values();

    noise_ctx.stop();
    noise_ctx.get_cipher_state(payload_cipher_state);
    noise_ctx.dump();
}

void essu::noise_handshake_context::check_noise_action(noise::noise_action expected) {
    auto action = noise_ctx.get_action();

    if (action == noise::noise_action::FAILED)
        this->log.throw_exception("Failed to handshake.");

    if (action == expected)
        return;

    if (action == noise::noise_action::WRITE_MESSAGE)
        this->log.throw_exception("Expected message to be sent.");
    else if (action == noise::noise_action::READ_MESSAGE)
        this->log.throw_exception("Expected message to be received.");
    else
        this->log.throw_exception("Handshake already completed.");
}

// Generates ephemeral header obfuscation key + ephmeral obfuscation key for hs1
void essu::noise_handshake_context::generate_pair_ephemeral_obfs_key() {
    typename noise_context_type::dh_key_type public_key{};
    const auto xor_public_key_with_buffer = [&](const auto &buffer) {
        std::transform(public_key.begin(), public_key.begin() + buffer.size(),
                       reinterpret_cast<const noheap::rbyte *>(buffer.begin()),
                       public_key.begin(), std::bit_xor{});
    };

    // Derives shared key using own and remote public keys
    if (remote_public_key != noise_context_type::dh_key_type{}) {
        xor_public_key_with_buffer(local_keypair.pub);
        xor_public_key_with_buffer(remote_public_key);
    }
    xor_public_key_with_buffer(unique_value);
    xor_public_key_with_buffer(ephemeral_obfs_key);

    // Gets 32 bytes-hash of public key
    auto public_key_hash = noheap::clip_buffer<32, 0>(
        hash_state.get_hash({public_key.data(), public_key.size()}));

    // Generates keystream
    noise::buffer_type<noheap::buffer_size<noise_context_type::dh_key_type> * 3
                       + noise_context_type::mac_size>
                                     keystream{};
    noise_context_type::cipher_state cipher_tmp;
    cipher_tmp.set_encrypt_key(public_key_hash);
    cipher_tmp.encrypt_buffer.set({keystream.data(), keystream.size()},
                                  keystream.size() - noise_context_type::mac_size);
    cipher_tmp.encrypt({});

    auto header_obfs_key1 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::dh_key_type>, 0>(
            keystream);
    auto header_obfs_key2 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::dh_key_type>,
                            noheap::buffer_size<noise_context_type::dh_key_type>>(
            keystream);

    if (role == noise::noise_role::INITIATOR) {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key1);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key2);
    } else {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key2);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key1);
    }

    ephemeral_obfs_key =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::dh_key_type>,
                            noheap::buffer_size<noise_context_type::dh_key_type> * 2>(
            keystream);
}

// Generates posthandshake header obfuscation key + unique value
void essu::noise_handshake_context::generate_posthandshake_unique_values() {
    // Generates unique values
    std::decay_t<decltype(handshake_hash)> output_tmp;
    hash_state.hkdf({handshake_hash.data(), handshake_hash.size()},
                    {handshake_payload.data(), handshake_payload.size()},
                    {output_tmp.data(), output_tmp.size()},
                    {unique_value.data(), unique_value.size()});

    // Generates keystream - the header obfuscation key
    noise::buffer_type<noheap::buffer_size<noise_context_type::dh_key_type> * 2
                       + noise_context_type::mac_size>
                                     keystream{};
    noise_context_type::cipher_state cipher_tmp;
    cipher_tmp.set_encrypt_key(
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::dh_key_type>, 0>(
            output_tmp));
    cipher_tmp.encrypt_buffer.set({keystream.data(), keystream.size()},
                                  keystream.size() - noise_context_type::mac_size);
    cipher_tmp.encrypt({});

    auto header_obfs_key1 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::dh_key_type>, 0>(
            keystream);
    auto header_obfs_key2 =
        noheap::clip_buffer<noheap::buffer_size<noise_context_type::dh_key_type>,
                            noheap::buffer_size<noise_context_type::dh_key_type>>(
            keystream);

    if (role == noise::noise_role::INITIATOR) {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key1);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key2);
    } else {
        header_cipher_state_sender.set_encrypt_key(header_obfs_key2);
        header_cipher_state_receiver.set_encrypt_key(header_obfs_key1);
    }
}
