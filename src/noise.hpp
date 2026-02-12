#ifndef NOISE_HPP
#define NOISE_HPP

#include "utils.hpp"

#include <noise/protocol.h>

enum class noise_pattern : std::uint16_t {
    UNKNOWN = 0,

    // Peer to known NT-unit
    NK = NOISE_PATTERN_NK,

    // Peer to unknown NT-unit
    NX = NOISE_PATTERN_NX,

    // Peer to peer
    NN = NOISE_PATTERN_NN,

    // Known NT-unit to known NT-unit
    // or Known peer to known peer
    KK = NOISE_PATTERN_KK,

    // Known NT-unit to unknown NT-unit(new unit in NT-space)
    KX = NOISE_PATTERN_KX,

    // Unknown NT-unit(new unit in NT-space) to known NT-unit
    XK = NOISE_PATTERN_XK,

    // Unknown NT-unit to unknown NT-unit(During the creation of
    // new NT-space)
    XX = NOISE_PATTERN_XX,
};

enum noise_role : std::uint16_t {
    UNKNOWN = 0,

    INITIATOR = NOISE_ROLE_INITIATOR,
    RESPONDER = NOISE_ROLE_RESPONDER,
};

enum noise_action : std::uint16_t {
    NONE          = NOISE_ACTION_NONE,
    WRITE_MESSAGE = NOISE_ACTION_WRITE_MESSAGE,
    READ_MESSAGE  = NOISE_ACTION_READ_MESSAGE,
    FAILED        = NOISE_ACTION_FAILED,
    SPLIT         = NOISE_ACTION_SPLIT,
    COMPLETE      = NOISE_ACTION_COMPLETE,
};

enum ntn_relation : std::uint8_t { PTU = 0, UTU };

bool is_ptu(noise_pattern pattern) {
    if (pattern == noise_pattern::NK || pattern == noise_pattern::NX
        || pattern == noise_pattern::NN || pattern == noise_pattern::KK)
        return true;
    return false;
}

noise_pattern get_noise_pattern(std::string_view pattern_string) {
    if (pattern_string == "NK")
        return noise_pattern::NK;
    else if (pattern_string == "NX")
        return noise_pattern::NX;
    else if (pattern_string == "NN")
        return noise_pattern::NN;
    else if (pattern_string == "KK")
        return noise_pattern::KK;
    else if (pattern_string == "KX")
        return noise_pattern::KX;
    else if (pattern_string == "XK")
        return noise_pattern::XK;
    else if (pattern_string == "XX")
        return noise_pattern::XX;
    return noise_pattern::UNKNOWN;
}

noise_role get_noise_role(std::string_view role_string) {
    if (role_string == "INITIATOR")
        return noise_role::INITIATOR;
    else if (role_string == "RESPONDER")
        return noise_role::RESPONDER;
    return noise_role::UNKNOWN;
}

template<std::uint16_t ec>
consteval std::size_t get_key_size() {
    if constexpr (ec == NOISE_DH_CURVE25519)
        return 32;
    else if constexpr (ec == NOISE_DH_CURVE448)
        return 56;
    else
        static_assert(false, "The passed EC type for DH is not supported.");
}

static constexpr std::size_t max_size_key = get_key_size<NOISE_DH_CURVE448>();
template<std::size_t size>
    requires(size <= max_size_key)
using buffer_key_type = noheap::buffer_bytes_type<size, std::uint8_t>;

template<ntn_relation _relation_type>
class noise_context {
private:
    static constexpr ntn_relation relation_type = _relation_type;
    static constexpr bool         ptu           = relation_type == ntn_relation::PTU;

    static constexpr struct {
        std::uint16_t prefix_id = ptu ? NOISE_PREFIX_PSK : NOISE_PREFIX_STANDARD;
        std::uint16_t dh_id     = ptu ? NOISE_DH_CURVE25519 : NOISE_DH_CURVE448;
        std::uint16_t cipher_id = NOISE_CIPHER_CHACHAPOLY;
        std::uint16_t hash_id   = ptu ? NOISE_HASH_BLAKE2s : NOISE_HASH_BLAKE2b;
        std::uint16_t hybrid_id = ptu ? NOISE_DH_NONE : NOISE_DH_KYBER1024;
    } nid_static;

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NOISE_CONTEXT");
    static constexpr log_handler log{buffer_owner};

public:
    static constexpr std::size_t handshake_packet_size   = 512;
    static constexpr std::size_t prologue_extention_size = 16;
    static constexpr std::size_t pre_shared_key_size     = 32;
    static constexpr std::size_t dh_key_size = get_key_size<nid_static.dh_id>();
    static constexpr std::size_t max_buffer_name_id_size = 64;

    using buffer_handshake_packet_type =
        noheap::buffer_bytes_type<handshake_packet_size, std::uint8_t>;

    using prologue_extention_type = buffer_key_type<prologue_extention_size>;
    using pre_shared_key_type     = buffer_key_type<pre_shared_key_size>;
    using dh_key_type             = buffer_key_type<dh_key_size>;

    using name_id = noheap::buffer_bytes_type<max_buffer_name_id_size>;

private:
    struct {
        std::uint16_t           cipher;
        std::uint16_t           psk;
        std::uint16_t           dh;
        std::uint16_t           hash;
        std::uint16_t           hybrid;
        std::uint16_t           pattern;
        prologue_extention_type ext;
    } prologue = {.cipher = nid_static.cipher_id,
                  .psk    = nid_static.prefix_id,
                  .dh     = nid_static.dh_id,
                  .hash   = nid_static.hash_id,
                  .hybrid = nid_static.hybrid_id,
                  .ext    = {}};

public:
    struct local_keypair_type {
        dh_key_type priv;
        dh_key_type pub;
    };

public:
    struct noise_buffer_view {
    public:
        constexpr NoiseBuffer *operator->() noexcept;
        constexpr NoiseBuffer &operator*() noexcept;

    public:
        void set_buffer(std::span<std::uint8_t> buffer, std::size_t payload_size);

    private:
        NoiseBuffer buffer{};
    };

    struct cipher_state {
        friend class noise_context<_relation_type>;

    public:
        cipher_state()                                = default;
        cipher_state(cipher_state &&handle)           = default;
        cipher_state &operator=(cipher_state &&)      = default;
        cipher_state(const cipher_state &)            = delete;
        cipher_state &operator=(const cipher_state &) = delete;

        ~cipher_state();

    public:
        void encrypt();
        void decrypt();

    private:
        void check_completed_handshake();

    public:
        noise_buffer_view input_buffer{};
        noise_buffer_view output_buffer{};

    private:
        NoiseCipherState *send_cipher    = nullptr;
        NoiseCipherState *receive_cipher = nullptr;
        NoiseRandState   *randstate      = nullptr;

        bool completed_handshake = false;
    };

public:
    noise_context(noise_pattern pattern, noise_role role);

    noise_context(noise_context &&)                 = delete;
    noise_context(const noise_context &)            = delete;
    noise_context &operator=(const noise_context &) = delete;

    ~noise_context();

public:
    void start();
    void stop();

    noise_buffer_view &get_handshake_buffer();
    cipher_state       get_cipher_state();

public:
    name_id      get_name_id() const;
    std::ssize_t get_action();

    void set_handshake_message();
    void get_handshake_message();

public:
    void set_prologue(prologue_extention_type &&ext);

    void set_local_keypair(const local_keypair_type &kp);
    void set_remote_public_key(dh_key_type &&key);
    void set_pre_shared_key(pre_shared_key_type &&key);

public:
    local_keypair_type generate_local_keypair();

private:
    static void handle_error(std::size_t error, std::string_view extention_error);

private:
    NoiseProtocolId      nid;
    NoiseHandshakeState *handshakestate = nullptr;

    noise_buffer_view handshake_buffer{};
    cipher_state      cipher_st{};
};
template<ntn_relation _relation_type>
constexpr NoiseBuffer *
    noise_context<_relation_type>::noise_buffer_view::operator->() noexcept {
    return &buffer;
}
template<ntn_relation _relation_type>
constexpr NoiseBuffer &
    noise_context<_relation_type>::noise_buffer_view::operator*() noexcept {
    return buffer;
}

template<ntn_relation _relation_type>
void noise_context<_relation_type>::noise_buffer_view::set_buffer(
    std::span<std::uint8_t> buffer, std::size_t payload_size) {
    noise_buffer_set_inout(this->buffer, buffer.data(), payload_size, buffer.size());
}

template<ntn_relation _relation_type>
noise_context<_relation_type>::cipher_state::~cipher_state() {
    noise_randstate_free(randstate);
    noise_cipherstate_free(send_cipher);
    noise_cipherstate_free(receive_cipher);
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::cipher_state::encrypt() {
    check_completed_handshake();
    std::size_t ret;
    if ((ret = noise_randstate_pad(randstate, output_buffer->data, output_buffer->size,
                                   output_buffer->max_size
                                       - noise_cipherstate_get_mac_length(send_cipher),
                                   NOISE_PADDING_RANDOM))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to pad");

    if ((ret = noise_cipherstate_encrypt(send_cipher, &*output_buffer))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to encrypt");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::cipher_state::decrypt() {
    check_completed_handshake();
    std::size_t ret;
    if ((ret = noise_cipherstate_decrypt(send_cipher, &*input_buffer))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to decrypt");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::cipher_state::check_completed_handshake() {
    if (!completed_handshake)
        handle_error(0, "The handshake is not completed");
}

template<ntn_relation _relation_type>
noise_context<_relation_type>::noise_context(noise_pattern pattern, noise_role role)
    : nid{.prefix_id  = nid_static.prefix_id,
          .pattern_id = static_cast<std::uint16_t>(pattern),
          .dh_id      = nid_static.dh_id,
          .cipher_id  = nid_static.cipher_id,
          .hash_id    = nid_static.hash_id,
          .hybrid_id  = nid_static.hybrid_id,
          .reserved   = {}} {
    prologue.pattern = nid.pattern_id;

    if (ptu && !is_ptu(pattern) || !ptu && is_ptu(pattern))
        throw noheap::runtime_error(buffer_owner,
                                    "Relation type doesn't follow the pattern.");

    std::size_t ret;
    if ((ret = noise_init()) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init noise.");

    if ((ret = noise_handshakestate_new_by_id(&handshakestate, &nid, role))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get new state of handshake.");
}
template<ntn_relation _relation_type>
noise_context<_relation_type>::~noise_context() {
    noise_handshakestate_free(handshakestate);
}

template<ntn_relation _relation_type>
void noise_context<_relation_type>::start() {
    std::size_t ret;

    if ((ret = noise_handshakestate_start(handshakestate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to start handshake");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::stop() {
    if (this->get_action() != NOISE_ACTION_SPLIT)
        handle_error(0, "Failed to complete handshake");

    std::size_t ret;
    if ((ret = noise_handshakestate_split(handshakestate, &cipher_st.send_cipher,
                                          &cipher_st.receive_cipher))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to split handshake");

    if ((ret = noise_randstate_new(&cipher_st.randstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init randstate");

    cipher_st.completed_handshake = true;
}
template<ntn_relation _relation_type>
noise_context<_relation_type>::noise_buffer_view &
    noise_context<_relation_type>::get_handshake_buffer() {
    return handshake_buffer;
}
template<ntn_relation _relation_type>
noise_context<_relation_type>::cipher_state
    noise_context<_relation_type>::get_cipher_state() {
    return std::move(cipher_st);
}

template<ntn_relation _relation_type>
noise_context<_relation_type>::name_id
    noise_context<_relation_type>::get_name_id() const {
    name_id buffer_tmp{};
    noise_protocol_id_to_name(buffer_tmp.data(), buffer_tmp.size(), &nid);
    return buffer_tmp;
}
template<ntn_relation _relation_type>
std::ssize_t noise_context<_relation_type>::get_action() {
    return noise_handshakestate_get_action(handshakestate);
}

template<ntn_relation _relation_type>
void noise_context<_relation_type>::set_handshake_message() {
    std::size_t ret;
    if ((ret =
             noise_handshakestate_write_message(handshakestate, &*handshake_buffer, NULL))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set handshake message.");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::get_handshake_message() {
    std::size_t ret;
    if ((ret =
             noise_handshakestate_read_message(handshakestate, &*handshake_buffer, NULL))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get handshake message");
}

template<ntn_relation _relation_type>
void noise_context<_relation_type>::set_prologue(prologue_extention_type &&ext) {
    prologue.ext = std::move(ext);

    std::size_t ret;
    if ((ret = noise_handshakestate_set_prologue(
             handshakestate, reinterpret_cast<char *>(&prologue), sizeof(prologue)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set prologue");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::set_pre_shared_key(pre_shared_key_type &&key) {
    if (!noise_handshakestate_needs_pre_shared_key(handshakestate))
        return;

    std::size_t ret;
    if ((ret = noise_handshakestate_set_pre_shared_key(handshakestate, key.data(),
                                                       key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set pre shared key");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::set_remote_public_key(dh_key_type &&key) {
    if (!noise_handshakestate_needs_remote_public_key(handshakestate))
        return;

    NoiseDHState *dh = noise_handshakestate_get_remote_public_key_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_set_public_key(dh, key.data(), key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set remote public key");
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::set_local_keypair(const local_keypair_type &kp) {
    if (!noise_handshakestate_needs_local_keypair(handshakestate))
        return;

    NoiseDHState *dh = noise_handshakestate_get_local_keypair_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_set_keypair(dh, kp.priv.data(), kp.priv.size(),
                                         kp.pub.data(), kp.pub.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set local keypair");
}
template<ntn_relation _relation_type>
noise_context<_relation_type>::local_keypair_type
    noise_context<_relation_type>::generate_local_keypair() {
    local_keypair_type kp;
    NoiseDHState      *dh = noise_handshakestate_get_local_keypair_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_generate_keypair(dh)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to generate keypair");
    if ((ret = noise_dhstate_get_keypair(dh, kp.priv.data(), kp.priv.size(),
                                         kp.pub.data(), kp.pub.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get generated keypair");

    return kp;
}
template<ntn_relation _relation_type>
void noise_context<_relation_type>::handle_error(std::size_t      error,
                                                 std::string_view extention_error) {
    if (error) {
        noheap::buffer_bytes_type<64> buffer_noise_error{};
        noise_strerror(error, buffer_noise_error.data(), buffer_noise_error.size());
        throw noheap::runtime_error(buffer_owner, "{}: {}.", extention_error,
                                    buffer_noise_error.data());
    } else
        throw noheap::runtime_error(buffer_owner, "{}.", extention_error);
}

#endif
