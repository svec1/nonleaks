#ifndef NOISE_HPP
#define NOISE_HPP

#include "utils.hpp"

#include <noise/protocol.h>

namespace noise {

constexpr std::size_t handshake_packet_size   = 2048;
constexpr std::size_t handshake_payload_size  = 32;
constexpr std::size_t prologue_extention_size = 16;
constexpr std::size_t pre_shared_key_size     = 32;

template<std::size_t size>
using buffer_type                   = noheap::buffer_bytes_type<size, noheap::rbyte>;
using buffer_handshake_packet_type  = buffer_type<handshake_packet_size>;
using buffer_handshake_payload_type = buffer_type<handshake_payload_size>;
using prologue_extention_type       = buffer_type<prologue_extention_size>;
using pre_shared_key_type           = buffer_type<pre_shared_key_size>;
using buffer_name_id_type           = noheap::buffer_chars_type<NOISE_MAX_PROTOCOL_NAME>;

enum class noise_pattern : std::uint16_t {
    UNKNOWN = 0,
    XX      = NOISE_PATTERN_XX,
    XX_HFS  = NOISE_PATTERN_XX_HFS, // with KEM
    XK      = NOISE_PATTERN_XK,
    XK_HFS  = NOISE_PATTERN_XK_HFS, // with KEM
};

enum class noise_role : std::uint16_t {
    UNKNOWN   = 0,
    INITIATOR = NOISE_ROLE_INITIATOR,
    RESPONDER = NOISE_ROLE_RESPONDER,
};

enum class noise_action : std::uint16_t {
    UNKNOWN       = 0,
    NONE          = NOISE_ACTION_NONE,
    WRITE_MESSAGE = NOISE_ACTION_WRITE_MESSAGE,
    READ_MESSAGE  = NOISE_ACTION_READ_MESSAGE,
    FAILED        = NOISE_ACTION_FAILED,
    SPLIT         = NOISE_ACTION_SPLIT,
    COMPLETE      = NOISE_ACTION_COMPLETE,
};

enum class ecdh_type : std::uint16_t {
    UNKNOWN          = 0,
    X25519           = NOISE_DH_CURVE25519,
    X25519_KYBER1024 = NOISE_DH_CURVE25519 ^ NOISE_DH_KYBER1024,
};

enum class cipher_type : std::uint16_t {
    UNKNOWN    = 0,
    CHACHAPOLY = NOISE_CIPHER_CHACHAPOLY,
    AESGCM     = NOISE_CIPHER_AESGCM,
};

enum class hash_type : std::uint16_t {
    UNKNOWN = 0,
    SHA256  = NOISE_HASH_SHA256,
    SHA512  = NOISE_HASH_SHA512,
    SHA3256 = NOISE_HASH_SHA512,
    SHA3512 = NOISE_HASH_SHA3512,
};

noise_pattern get_noise_pattern(std::string_view pattern_string) {
    if (pattern_string == "XX")
        return noise_pattern::XX;
    else if (pattern_string == "XX_HFS")
        return noise_pattern::XX_HFS;
    else if (pattern_string == "XK")
        return noise_pattern::XK;
    else if (pattern_string == "XK_HFS")
        return noise_pattern::XK_HFS;
    return noise_pattern::UNKNOWN;
}

noise_role get_noise_role(std::string_view role_string) {
    if (role_string == "INITIATOR")
        return noise_role::INITIATOR;
    else if (role_string == "RESPONDER")
        return noise_role::RESPONDER;
    return noise_role::UNKNOWN;
}
template<noise_pattern pattern, ecdh_type ecdh>
consteval std::size_t pattern_ecdh_is_compatible() {
    if constexpr (((pattern == noise_pattern::XX || pattern == noise_pattern::XK)
                   && ecdh == ecdh_type::X25519)
                  || ((pattern == noise_pattern::XX_HFS
                       || pattern == noise_pattern::XK_HFS)
                      && ecdh == ecdh_type::X25519_KYBER1024))
        return true;
    return false;
}

template<ecdh_type ecdh>
consteval std::size_t get_dh_key_size() {
    if constexpr (ecdh == ecdh_type::X25519 || ecdh == ecdh_type::X25519_KYBER1024)
        return 32;
    else
        static_assert(false, "The passed ECDH type is not supported.");
}
template<ecdh_type ecdh>
consteval std::size_t get_kem_key_size() {
    if constexpr (ecdh == ecdh_type::X25519_KYBER1024)
        return 1568;
    else
        return 0;
}
template<ecdh_type ecdh>
consteval std::size_t get_kem_cipher_text_size() {
    if constexpr (ecdh == ecdh_type::X25519_KYBER1024)
        return 1568;
    else
        return 0;
}

template<cipher_type cipher>
consteval std::size_t get_mac_size() {
    if constexpr (cipher == cipher_type::CHACHAPOLY)
        return 16;
    else if constexpr (cipher == cipher_type::AESGCM)
        return 16;
    else
        static_assert(false, "The passed cipher type is not supported.");
}

template<hash_type hash>
consteval std::size_t get_hash_size() {
    if constexpr (hash == hash_type::SHA256 || hash == hash_type::SHA3256)
        return 32;
    else if constexpr (hash == hash_type::SHA512 || hash == hash_type::SHA3512)
        return 64;
    else
        static_assert(false, "The passed hash type is not supported.");
}

template<noise_pattern _pattern, ecdh_type _ecdh, cipher_type _cipher, hash_type _hash>
struct noise_context_config {
    static constexpr noise_pattern pattern = _pattern;
    static constexpr ecdh_type     ecdh    = _ecdh;
    static constexpr cipher_type   cipher  = _cipher;
    static constexpr hash_type     hash    = _hash;

    static consteval std::size_t get_hs1_size() {
        if constexpr (pattern == noise_pattern::XX)
            return get_dh_key_size<ecdh>() + get_mac_size<cipher>();
        if constexpr (pattern == noise_pattern::XK)
            return get_dh_key_size<ecdh>() + get_mac_size<cipher>();
        else if constexpr (pattern == noise_pattern::XX_HFS
                           || pattern == noise_pattern::XK_HFS) {
            return get_kem_key_size<ecdh>() + get_dh_key_size<ecdh>()
                   + get_mac_size<cipher>() * 2;
        }
    }
    static consteval std::size_t get_hs2_size() {
        if constexpr (pattern == noise_pattern::XX)
            return noise::get_dh_key_size<ecdh>() * 2 + get_mac_size<cipher>() * 2;
        if constexpr (pattern == noise_pattern::XK)
            return get_dh_key_size<ecdh>() + get_mac_size<cipher>();
        else if constexpr (pattern == noise_pattern::XX_HFS) {
            return get_kem_cipher_text_size<ecdh>() + noise::get_dh_key_size<ecdh>() * 2
                   + get_mac_size<cipher>() * 2 + 16;
        } else if constexpr (pattern == noise_pattern::XK_HFS) {
            return get_kem_cipher_text_size<ecdh>() + noise::get_dh_key_size<ecdh>()
                   + get_mac_size<cipher>();
        } else
            static_assert(false, "Invalid noise context config.");
    }
    static consteval std::size_t get_hs3_size() {
        if constexpr (pattern == noise_pattern::XX || pattern == noise_pattern::XK
                      || pattern == noise_pattern::XX_HFS
                      || pattern == noise_pattern::XK_HFS)
            return noise::get_dh_key_size<ecdh>() + get_mac_size<cipher>() * 2
                   + handshake_payload_size;
        else
            static_assert(false, "Invalid noise context config.");
    }

private:
    static_assert(noise::pattern_ecdh_is_compatible<pattern, ecdh>(),
                  "Noise pattern and ecdh type is not compatible.");
};

template<noise::noise_context_config _config>
class noise_context {
public:
    static constexpr noise::noise_context_config config = _config;
    static constexpr bool hybrid_kyber1024 = (config.ecdh == ecdh_type::X25519_KYBER1024);
    static constexpr ecdh_type ecdh =
        hybrid_kyber1024
            ? ecdh_type(static_cast<std::uint16_t>(config.ecdh) ^ NOISE_DH_KYBER1024)
            : config.ecdh;

    static constexpr NoiseProtocolId nid_static{
        .prefix_id  = NOISE_PREFIX_PSK,
        .pattern_id = static_cast<std::uint16_t>(config.pattern),
        .dh_id      = static_cast<std::uint16_t>(ecdh),
        .cipher_id  = static_cast<std::uint16_t>(config.cipher),
        .hash_id    = static_cast<std::uint16_t>(config.hash),
        .hybrid_id  = hybrid_kyber1024 ? NOISE_DH_KYBER1024 : NOISE_DH_NONE,
        .reserved   = {},
    };

public:
    static constexpr std::size_t mac_size = get_mac_size<config.cipher>();
    using dh_key_type                     = buffer_type<get_dh_key_size<ecdh>()>;

private:
    struct {
        std::uint16_t           prefix;
        std::uint16_t           pattern;
        std::uint16_t           dh;
        std::uint16_t           cipher;
        std::uint16_t           hash;
        std::uint16_t           hybrid;
        prologue_extention_type ext;
    } prologue = {.prefix  = nid_static.prefix_id,
                  .pattern = nid_static.pattern_id,
                  .dh      = nid_static.dh_id,
                  .cipher  = nid_static.cipher_id,
                  .hash    = nid_static.hash_id,
                  .hybrid  = nid_static.hybrid_id,
                  .ext     = {}};

public:
    static constexpr std::size_t prologue_size = sizeof(prologue);
    using buffer_prologue_type                 = noheap::buffer_bytes_type<prologue_size>;

    struct keypair_type {
        dh_key_type priv;
        dh_key_type pub;
    };

    struct noise_buffer_view {
    public:
        void set(std::span<noheap::rbyte> buffer, std::size_t payload_size) {
            noise_buffer_set_inout(this->buffer,
                                   reinterpret_cast<noheap::ubyte *>(buffer.data()),
                                   payload_size, buffer.size());
        }

        template<typename T>
        auto &get(this T &&_this) {
            return _this.buffer;
        }

    private:
        NoiseBuffer buffer{};
    };

    struct random_state {
    public:
        random_state();

        random_state(random_state &&)                 = delete;
        random_state(const random_state &)            = delete;
        random_state &operator=(const random_state &) = delete;

        ~random_state();

    public:
        void pad();
        void reseed();

    public:
        noise_buffer_view padding_buffer{};

    private:
        NoiseRandState *randstate = nullptr;
    };

    struct cipher_state {
        friend class noise::noise_context<_config>;

    public:
        cipher_state();

        cipher_state(cipher_state &&)                 = delete;
        cipher_state(const cipher_state &)            = delete;
        cipher_state &operator=(const cipher_state &) = delete;

        ~cipher_state();

    public:
        void init(cipher_state &&other);
        void dump();

        void encrypt(std::span<noheap::rbyte> buffer_ad);
        void decrypt(std::span<noheap::rbyte> buffer_ad);
        void rekey_encrypt();
        void rekey_decrypt();

        void set_encrypt_nonce(std::uint64_t nonce);
        void set_decrypt_nonce(std::uint64_t nonce);
        void set_encrypt_key(const dh_key_type &key);
        void set_decrypt_key(const dh_key_type &key);

    private:
        void set_states(NoiseCipherState *_encrypt_state,
                        NoiseCipherState *_decrypt_state);

        void check_encrypt_key() const;
        void check_decrypt_key() const;

    public:
        noise_buffer_view encrypt_buffer{};
        noise_buffer_view decrypt_buffer{};

    private:
        NoiseCipherState *encrypt_state = nullptr;
        NoiseCipherState *decrypt_state = nullptr;
    };

    struct hash_state {
        using buffer_type = buffer_type<get_hash_size<config.hash>()>;

    public:
        hash_state();

        hash_state(hash_state &&handle)           = delete;
        hash_state(const hash_state &)            = delete;
        hash_state &operator=(const hash_state &) = delete;

        ~hash_state();

    public:
        buffer_type get_hash(std::span<noheap::rbyte> buffer);

        void hkdf(std::span<const noheap::rbyte> buffer,
                  std::span<const noheap::rbyte> key, std::span<noheap::rbyte> output1,
                  std::span<noheap::rbyte> output2);

    private:
        NoiseHashState *hashstate = nullptr;
    };

public:
    noise_context() = default;

    noise_context(noise_context &&)                 = delete;
    noise_context(const noise_context &)            = delete;
    noise_context &operator=(const noise_context &) = delete;

    ~noise_context();

public:
    void init(noise_role _role);
    void dump();
    void fallback();
    void start();
    void stop();

    noise_buffer_view &get_handshake_buffer();
    noise_buffer_view &get_handshake_payload_buffer();
    void               get_cipher_state(cipher_state &_cipher_state);

    noise_action get_action() const;
    noise_role   get_role() const;

    void set_handshake_message();
    void get_handshake_message();

    void                    set_prologue(prologue_extention_type ext);
    buffer_prologue_type    get_prologue();
    dh_key_type             get_remote_public_key();
    hash_state::buffer_type get_handshake_hash();

    void set_local_keypair(const keypair_type &kp);
    void set_remote_public_key(const dh_key_type &key);
    void set_pre_shared_key(const pre_shared_key_type &key);

public:
    static buffer_name_id_type get_name_id();
    static keypair_type        generate_keypair();

private:
    static void handle_error(std::size_t error, std::string_view extention_error);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("NOISE_CONTEXT");
    static constexpr log_handler log{buffer_owner};

private:
    NoiseHandshakeState *handshakestate = nullptr;

    cipher_state cipher_st{};

    noise_buffer_view handshake_buffer{};
    noise_buffer_view handshake_payload_buffer{};
};
} // namespace noise

// Random state
template<noise::noise_context_config _config>
noise::noise_context<_config>::random_state::random_state() {
    std::size_t ret;
    if ((ret = noise_randstate_new(&randstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init random state.");
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::random_state::~random_state() {
    noise_randstate_free(randstate);
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::random_state::pad() {
    auto &noise_buffer = padding_buffer.get();

    std::size_t ret;
    if ((ret = noise_randstate_pad(randstate, noise_buffer.data, noise_buffer.size,
                                   noise_buffer.max_size, NOISE_PADDING_RANDOM))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to pad.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::random_state::reseed() {
    std::size_t ret;
    if ((ret = noise_randstate_reseed(randstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to reseed.");
}

// Cipher state
template<noise::noise_context_config _config>
noise::noise_context<_config>::cipher_state::cipher_state() {
    std::size_t ret;
    if ((ret = noise_cipherstate_new_by_id(&encrypt_state,
                                           static_cast<std::uint16_t>(config.cipher)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init encrypt cipher state.");
    if ((ret = noise_cipherstate_new_by_id(&decrypt_state,
                                           static_cast<std::uint16_t>(config.cipher)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init decrypt cipher state.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::init(cipher_state &&other) {
    dump();

    encrypt_state = other.encrypt_state;
    decrypt_state = other.decrypt_state;

    other.encrypt_state = other.decrypt_state = nullptr;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::dump() {
    if (encrypt_state == nullptr)
        return;

    std::size_t ret;
    if ((ret = noise_cipherstate_free(encrypt_state)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free encrypt cipher state.");
    if ((ret = noise_cipherstate_free(decrypt_state)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free decrypt cipher state.");

    encrypt_state = decrypt_state = nullptr;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::cipher_state::~cipher_state() {
    dump();
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::encrypt(
    std::span<noheap::rbyte> buffer_ad) {
    check_encrypt_key();
    std::size_t ret;

    if ((ret = noise_cipherstate_encrypt_with_ad(
             encrypt_state, reinterpret_cast<noheap::ubyte *>(buffer_ad.data()),
             buffer_ad.size(), &encrypt_buffer.get()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to encrypt.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::decrypt(
    std::span<noheap::rbyte> buffer_ad) {
    check_decrypt_key();
    std::size_t ret;
    if ((ret = noise_cipherstate_decrypt_with_ad(
             decrypt_state, reinterpret_cast<noheap::ubyte *>(buffer_ad.data()),
             buffer_ad.size(), &decrypt_buffer.get()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to decrypt.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::rekey_encrypt() {
    check_encrypt_key();
    std::size_t ret;
    if ((ret = noise_cipherstate_rekey(encrypt_state)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to rekey for encrypt state.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::rekey_decrypt() {
    check_decrypt_key();
    std::size_t ret;
    if ((ret = noise_cipherstate_rekey(decrypt_state)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to rekey for decrypt state.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_encrypt_nonce(std::uint64_t nonce) {
    check_encrypt_key();
    std::size_t ret;
    if ((ret = noise_cipherstate_set_nonce(encrypt_state, nonce)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set encrypting nonce.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_decrypt_nonce(std::uint64_t nonce) {
    check_decrypt_key();
    std::size_t ret;
    if ((ret = noise_cipherstate_set_nonce(decrypt_state, nonce)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set decrypting nonce.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_encrypt_key(
    const dh_key_type &key) {
    std::size_t ret;
    if ((ret = noise_cipherstate_init_key(
             encrypt_state, reinterpret_cast<const noheap::ubyte *>(key.data()),
             key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set encrypt key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_decrypt_key(
    const dh_key_type &key) {
    std::size_t ret;
    if ((ret = noise_cipherstate_init_key(
             decrypt_state, reinterpret_cast<const noheap::ubyte *>(key.data()),
             key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set decrypt key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::check_encrypt_key() const {
    if (!noise_cipherstate_has_key(encrypt_state))
        handle_error(0, "The encrypt state does not has a key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::check_decrypt_key() const {
    if (!noise_cipherstate_has_key(decrypt_state))
        handle_error(0, "The decrypt state does not has a key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_states(
    NoiseCipherState *_encrypt_state, NoiseCipherState *_decrypt_state) {
    dump();

    encrypt_state = _encrypt_state;
    decrypt_state = _decrypt_state;
}

// Hash state
template<noise::noise_context_config _config>
noise::noise_context<_config>::hash_state::hash_state() {
    std::size_t ret;
    if ((ret = noise_hashstate_new_by_id(&hashstate,
                                         static_cast<std::uint16_t>(config.hash)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init hash state.");
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::hash_state::~hash_state() {
    std::size_t ret;
    if ((ret = noise_hashstate_free(hashstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free hash state.");
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::hash_state::buffer_type
    noise::noise_context<_config>::hash_state::get_hash(std::span<noheap::rbyte> buffer) {
    decltype(get_hash(buffer)) buffer_tmp{};
    std::size_t                ret;
    if ((ret = noise_hashstate_hash_one(
             hashstate, reinterpret_cast<noheap::ubyte *>(buffer.data()), buffer.size(),
             reinterpret_cast<noheap::ubyte *>(buffer_tmp.data()), buffer_tmp.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get hash.");

    return buffer_tmp;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::hash_state::hkdf(
    std::span<const noheap::rbyte> buffer, std::span<const noheap::rbyte> key,
    std::span<noheap::rbyte> output1, std::span<noheap::rbyte> output2) {
    std::size_t ret;
    if ((ret = noise_hashstate_hkdf(
             hashstate, reinterpret_cast<const noheap::ubyte *>(key.data()), key.size(),
             reinterpret_cast<const noheap::ubyte *>(buffer.data()), buffer.size(),
             reinterpret_cast<noheap::ubyte *>(output1.data()), output1.size(),
             reinterpret_cast<noheap::ubyte *>(output2.data()), output2.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to perform KDF.");
}

// Noise context
template<noise::noise_context_config _config>
noise::noise_context<_config>::~noise_context<_config>() {
    this->dump();
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::init(noise_role role) {
    std::size_t ret;
    cipher_st.dump();
    if ((ret = noise_handshakestate_new_by_id(&handshakestate, &nid_static,
                                              static_cast<std::uint16_t>(role)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get new state of handshake.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::dump() {
    if (!handshakestate)
        return;

    noise_handshakestate_free(handshakestate);
    handshakestate           = nullptr;
    handshake_buffer         = {};
    handshake_payload_buffer = {};
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::fallback() {
    std::size_t ret;
    if ((ret = noise_handshakestate_fallback(handshakestate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to fallback.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::start() {
    std::size_t ret;
    if ((ret = noise_handshakestate_start(handshakestate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to start handshake.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::stop() {
    if (this->get_action() != noise_action::SPLIT)
        handle_error(0, "Failed to complete handshake.");

    std::size_t ret;
    if ((ret = noise_handshakestate_split(handshakestate, &cipher_st.encrypt_state,
                                          &cipher_st.decrypt_state))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to split handshake.");
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::noise_buffer_view &
    noise::noise_context<_config>::get_handshake_buffer() {
    return handshake_buffer;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::noise_buffer_view &
    noise::noise_context<_config>::get_handshake_payload_buffer() {
    return handshake_payload_buffer;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::get_cipher_state(cipher_state &_cipher_st) {
    _cipher_st.init(std::move(cipher_st));
}

template<noise::noise_context_config _config>
noise::noise_action noise::noise_context<_config>::get_action() const {
    return noise_action(noise_handshakestate_get_action(handshakestate));
}
template<noise::noise_context_config _config>
noise::noise_role noise::noise_context<_config>::get_role() const {
    return noise_role(noise_handshakestate_get_role(handshakestate));
}

template<noise::noise_context_config _config>
void noise::noise_context<_config>::set_handshake_message() {
    if (handshake_payload_buffer.get().size > handshake_payload_size)
        handle_error(0, "Invalid size of handshake payload.");

    std::size_t ret;
    if ((ret = noise_handshakestate_write_message(handshakestate, &handshake_buffer.get(),
                                                  handshake_payload_buffer.get().data
                                                      ? &handshake_payload_buffer.get()
                                                      : NULL))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set handshake message.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::get_handshake_message() {
    if (handshake_payload_buffer.get().size > handshake_payload_size)
        handle_error(0, "Invalid size of handshake payload.");

    std::size_t ret;
    if ((ret = noise_handshakestate_read_message(handshakestate, &handshake_buffer.get(),
                                                 handshake_payload_buffer.get().data
                                                     ? &handshake_payload_buffer.get()
                                                     : NULL))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get handshake message.");
}

template<noise::noise_context_config _config>
void noise::noise_context<_config>::set_prologue(prologue_extention_type ext) {
    prologue.ext = std::move(ext);

    std::size_t ret;
    if ((ret = noise_handshakestate_set_prologue(
             handshakestate, reinterpret_cast<char *>(&prologue), sizeof(prologue)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set prologue.");
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::buffer_prologue_type
    noise::noise_context<_config>::get_prologue() {
    buffer_prologue_type buffer_tmp{};
    std::copy(buffer_tmp.begin(), buffer_tmp.end(), reinterpret_cast<char *>(&prologue));
    return buffer_tmp;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::dh_key_type
    noise::noise_context<_config>::get_remote_public_key() {
    dh_key_type   buffer_tmp{};
    NoiseDHState *dh = noise_handshakestate_get_remote_public_key_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_get_public_key(
             dh, reinterpret_cast<noheap::ubyte *>(buffer_tmp.data()),
             buffer_tmp.size())))
        handle_error(ret, "Failed to get remote public key.");

    return buffer_tmp;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::hash_state::buffer_type
    noise::noise_context<_config>::get_handshake_hash() {
    decltype(get_handshake_hash()) buffer_hash{};

    std::size_t ret;
    if ((ret = noise_handshakestate_get_handshake_hash(
             handshakestate, reinterpret_cast<noheap::ubyte *>(buffer_hash.data()),
             buffer_hash.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get handshake hash.");

    return buffer_hash;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::set_pre_shared_key(const pre_shared_key_type &key) {
    if (!noise_handshakestate_needs_pre_shared_key(handshakestate))
        return;

    std::size_t ret;
    if ((ret = noise_handshakestate_set_pre_shared_key(
             handshakestate, reinterpret_cast<const noheap::ubyte *>(key.data()),
             key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set pre shared key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::set_remote_public_key(const dh_key_type &key) {
    if (!noise_handshakestate_needs_remote_public_key(handshakestate))
        return;

    NoiseDHState *dh = noise_handshakestate_get_remote_public_key_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_set_public_key(
             dh, reinterpret_cast<const noheap::ubyte *>(key.data()), key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set remote public key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::set_local_keypair(const keypair_type &kp) {
    if (!noise_handshakestate_needs_local_keypair(handshakestate))
        return;

    NoiseDHState *dh = noise_handshakestate_get_local_keypair_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_set_keypair_private(
             dh, reinterpret_cast<const noheap::ubyte *>(kp.priv.data()), kp.priv.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set local keypair.");

    dh_key_type derived_public_key;
    if ((ret = noise_dhstate_get_public_key(
             dh, reinterpret_cast<noheap::ubyte *>(derived_public_key.data()),
             derived_public_key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get local public keypair.");

    if (derived_public_key != kp.pub)
        handle_error(ret, "The passed local public key is invalid.");
}
template<noise::noise_context_config _config>
noise::buffer_name_id_type noise::noise_context<_config>::get_name_id() {
    buffer_name_id_type buffer_tmp{};
    noise_protocol_id_to_name(buffer_tmp.data(), buffer_tmp.size(), &nid_static);
    return buffer_tmp;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::keypair_type
    noise::noise_context<_config>::generate_keypair() {
    keypair_type                  kp;
    noise::noise_context<_config> context_tmp;

    context_tmp.init(noise_role::INITIATOR);
    NoiseDHState *dh =
        noise_handshakestate_get_local_keypair_dh(context_tmp.handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_generate_keypair(dh)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to generate keypair.");
    if ((ret = noise_dhstate_get_keypair(
             dh, reinterpret_cast<noheap::ubyte *>(kp.priv.data()), kp.priv.size(),
             reinterpret_cast<noheap::ubyte *>(kp.pub.data()), kp.pub.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get generated keypair.");

    return kp;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::handle_error(std::size_t      error,
                                                 std::string_view extention_error) {
    if (error) {
        noheap::buffer_type<char, 64> buffer_noise_error{};
        noise_strerror(error, buffer_noise_error.data(), buffer_noise_error.size());
        log.throw_exception("{} {}", extention_error, buffer_noise_error.data());
    } else
        log.throw_exception("{}", extention_error);
}

#endif
