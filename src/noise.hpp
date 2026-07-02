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
using buffer_type                    = noheap::buffer_bytes_type<size, noheap::rbyte>;
using buffer_handshake_packet_type   = buffer_type<handshake_packet_size>;
using buffer_handshake_payload_type  = buffer_type<handshake_payload_size>;
using buffer_prologue_extention_type = buffer_type<prologue_extention_size>;
using buffer_pre_shared_key_type     = buffer_type<pre_shared_key_size>;
using buffer_name_id_type            = noheap::buffer_chars_type<NOISE_MAX_PROTOCOL_NAME>;

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

enum class dh_type : std::uint16_t {
    UNKNOWN   = 0,
    NONE      = NOISE_DH_NONE,
    X25519    = NOISE_DH_CURVE25519,
    MLKEM768  = NOISE_DH_MLKEM768,
    MLKEM1024 = NOISE_DH_MLKEM1024,
};

enum class cipher_type : std::uint16_t {
    UNKNOWN     = 0,
    CHACHAPOLY  = NOISE_CIPHER_CHACHAPOLY,
    XCHACHAPOLY = NOISE_CIPHER_XCHACHAPOLY,
    AESGCM      = NOISE_CIPHER_AESGCM,
};

enum class hash_type : std::uint16_t {
    UNKNOWN   = 0,
    RIPEMD160 = NOISE_HASH_RIPEMD160,
    SHA256    = NOISE_HASH_SHA256,
    SHA512    = NOISE_HASH_SHA512,
    SHA3256   = NOISE_HASH_SHA3256,
    SHA3512   = NOISE_HASH_SHA3512,
};

inline noise_pattern get_noise_pattern(const std::string_view pattern_string) {
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
inline noise_role get_noise_role(const std::string_view role_string) {
    if (role_string == "INITIATOR")
        return noise_role::INITIATOR;
    else if (role_string == "RESPONDER")
        return noise_role::RESPONDER;
    return noise_role::UNKNOWN;
}
inline std::string_view get_noise_role_string(noise_role role) {
    if (role == noise_role::INITIATOR)
        return "INITIATOR";
    else if (role == noise_role::RESPONDER)
        return "RESPONDER";
    return "UNKNOWN";
}

template<noise_pattern pattern, dh_type dh, dh_type hybrid_dh>
consteval std::size_t pattern_dh_is_compatible() {
    if constexpr (((pattern == noise_pattern::XX || pattern == noise_pattern::XK)
                   && dh == dh_type::X25519)
                  || ((pattern == noise_pattern::XX_HFS
                       || pattern == noise_pattern::XK_HFS)
                      && dh == dh_type::X25519
                      && (hybrid_dh == dh_type::MLKEM768
                          || hybrid_dh == dh_type::MLKEM1024)))
        return true;
    return false;
}

template<dh_type dh>
consteval std::size_t get_shared_secret_key_size() {
    if constexpr (dh == dh_type::X25519 || dh == dh_type::MLKEM768
                  || dh == dh_type::MLKEM1024)
        return 32;
    else
        return 0;
}
template<dh_type dh>
consteval std::size_t get_private_key_size() {
    if constexpr (dh == dh_type::X25519)
        return 32;
    else if constexpr (dh == dh_type::MLKEM768)
        return 2400;
    else if constexpr (dh == dh_type::MLKEM1024)
        return 3168;
    else
        return 0;
}
template<dh_type dh>
consteval std::size_t get_public_key_size() {
    if constexpr (dh == dh_type::X25519)
        return 32;
    else if constexpr (dh == dh_type::MLKEM768)
        return 1184;
    else if constexpr (dh == dh_type::MLKEM1024)
        return 1568;
    else
        return 0;
}
template<dh_type dh>
consteval std::size_t get_cipher_text_size() {
    if constexpr (dh == dh_type::MLKEM768)
        return 1088;
    else if constexpr (dh == dh_type::MLKEM1024)
        return 1568;
    else
        return 0;
}

template<cipher_type cipher>
consteval std::size_t get_mac_size() {
    if constexpr (cipher == cipher_type::CHACHAPOLY || cipher == cipher_type::XCHACHAPOLY
                  || cipher == cipher_type::AESGCM)
        return 16;
    else
        return 0;
}
template<cipher_type cipher>
consteval std::size_t get_nonce_size() {
    if constexpr (cipher == cipher_type::CHACHAPOLY || cipher == cipher_type::AESGCM)
        return 12;
    else if constexpr (cipher == cipher_type::XCHACHAPOLY)
        return 24;
    else
        return 0;
}

template<hash_type hash>
consteval std::size_t get_hash_size() {
    if constexpr (hash == hash_type::RIPEMD160)
        return 20;
    else if constexpr (hash == hash_type::SHA256 || hash == hash_type::SHA3256)
        return 32;
    else if constexpr (hash == hash_type::SHA512 || hash == hash_type::SHA3512)
        return 64;
    else
        return 0;
}

template<noise_pattern _pattern, dh_type _dh, dh_type _hybrid_dh, cipher_type _cipher,
         hash_type _hash>
struct noise_context_config {
    static constexpr noise_pattern pattern   = _pattern;
    static constexpr dh_type       dh        = _dh;
    static constexpr dh_type       hybrid_dh = _hybrid_dh;
    static constexpr cipher_type   cipher    = _cipher;
    static constexpr hash_type     hash      = _hash;

    static constexpr std::size_t mac_size   = get_mac_size<cipher>();
    static constexpr std::size_t nonce_size = get_nonce_size<cipher>();

    static consteval std::size_t get_hs1_size() {
        if constexpr (pattern == noise_pattern::XX)
            return get_public_key_size<dh>() + get_mac_size<cipher>();
        else if constexpr (pattern == noise_pattern::XK)
            return get_public_key_size<dh>() + get_mac_size<cipher>();
        else if constexpr (pattern == noise_pattern::XX_HFS
                           || pattern == noise_pattern::XK_HFS) {
            return get_public_key_size<hybrid_dh>() + get_public_key_size<dh>()
                   + get_mac_size<cipher>() * 2;
        } else
            static_assert(false, "Invalid noise context config.");
    }
    static consteval std::size_t get_hs2_size() {
        if constexpr (pattern == noise_pattern::XX)
            return get_public_key_size<dh>() * 2 + get_mac_size<cipher>() * 2;
        else if constexpr (pattern == noise_pattern::XK)
            return get_public_key_size<dh>() + get_mac_size<cipher>();
        else if constexpr (pattern == noise_pattern::XX_HFS)
            return get_cipher_text_size<hybrid_dh>()
                   + noise::get_public_key_size<dh>() * 2 + get_mac_size<cipher>() * 2
                   + 16;
        else if constexpr (pattern == noise_pattern::XK_HFS)
            return get_cipher_text_size<hybrid_dh>() + noise::get_public_key_size<dh>()
                   + get_mac_size<cipher>();
        else
            static_assert(false, "Invalid noise context config.");
    }
    static consteval std::size_t get_hs3_size() {
        if constexpr (pattern == noise_pattern::XX || pattern == noise_pattern::XK
                      || pattern == noise_pattern::XX_HFS
                      || pattern == noise_pattern::XK_HFS)
            return noise::get_shared_secret_key_size<dh>() + get_mac_size<cipher>() * 2
                   + handshake_payload_size;
        else
            static_assert(false, "Invalid noise context config.");
    }

    static_assert(hash != hash_type::UNKNOWN, "Undefined hash type.");
    static_assert(cipher != cipher_type::UNKNOWN, "Undefined cipher type.");
    static_assert(noise::pattern_dh_is_compatible<pattern, dh, hybrid_dh>(),
                  "Pattern and dh types is not compatible.");
};

template<noise::noise_context_config _config>
class noise_context : noncopyable {
public:
    static constexpr noise::noise_context_config config = _config;
    static constexpr NoiseProtocolId             nid_static{
                    .prefix_id  = NOISE_PREFIX_PSK,
                    .pattern_id = static_cast<std::uint16_t>(config.pattern),
                    .dh_id      = static_cast<std::uint16_t>(config.dh),
                    .cipher_id  = static_cast<std::uint16_t>(config.cipher),
                    .hash_id    = static_cast<std::uint16_t>(config.hash),
                    .hybrid_id  = static_cast<std::uint16_t>(config.hybrid_dh),
                    .reserved   = {},
    };

public:
    using buffer_key_type   = buffer_type<get_shared_secret_key_size<config.dh>()>;
    using buffer_nonce_type = buffer_type<get_nonce_size<config.cipher>()>;

private:
    struct {
        std::uint16_t                  prefix;
        std::uint16_t                  pattern;
        std::uint16_t                  dh;
        std::uint16_t                  cipher;
        std::uint16_t                  hash;
        std::uint16_t                  hybrid;
        buffer_prologue_extention_type ext;
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
        buffer_key_type priv;
        buffer_key_type pub;
    };

    struct noise_buffer_view {
    public:
        void set(std::span<noheap::rbyte> buffer, std::size_t payload_size) noexcept {
            noise_buffer_set_inout(this->buffer,
                                   reinterpret_cast<noheap::ubyte *>(buffer.data()),
                                   payload_size, buffer.size());
        }

        template<typename T>
        auto &get(this T &&_this) noexcept {
            return _this.buffer;
        }

    private:
        NoiseBuffer buffer{};
    };

    struct random_state : noncopyable {
        using result_type = std::size_t;

        result_type operator()() {
            result_type value;
            padding_buffer.set({reinterpret_cast<noheap::rbyte *>(&value), sizeof(value)},
                               0);
            pad();
            reseed();
            return value;
        }

        static constexpr std::size_t min() { return 0; }
        static constexpr std::size_t max() { return result_type(-1); }

    public:
        random_state();
        random_state(random_state &&other);
        ~random_state();

        random_state &operator=(random_state &&other);

    public:
        void pad();
        void reseed();

    public:
        noise_buffer_view padding_buffer;

    private:
        NoiseRandState *randstate = nullptr;
    };

    struct cipher_state : noncopyable {
        friend class noise_context;

    public:
        cipher_state();
        cipher_state(cipher_state &&other);
        ~cipher_state();

        cipher_state &operator=(cipher_state &&other);

    public:
        void encrypt(std::span<const noheap::rbyte> buffer_ad);
        void decrypt(std::span<const noheap::rbyte> buffer_ad);
        void rekey_encrypt();
        void rekey_decrypt();

        void set_encrypt_nonce(const buffer_nonce_type &nonce);
        void set_decrypt_nonce(const buffer_nonce_type &nonce);
        void set_encrypt_counter_block(std::uint64_t counter);
        void set_decrypt_counter_block(std::uint64_t counter);
        void set_encrypt_key(const buffer_key_type &key);
        void set_decrypt_key(const buffer_key_type &key);

        buffer_nonce_type get_encrypt_nonce() const;
        buffer_nonce_type get_decrypt_nonce() const;
        std::uint64_t     get_encrypt_counter_block() const;
        std::uint64_t     get_decrypt_counter_block() const;

        bool has_encrypt_key() const;
        bool has_decrypt_key() const;

    private:
        void set_states(NoiseCipherState *_encrypt_state,
                        NoiseCipherState *_decrypt_state);
        void dump();

        void check_encrypt_key() const;
        void check_decrypt_key() const;

    public:
        noise_buffer_view encrypt_buffer{};
        noise_buffer_view decrypt_buffer{};

    private:
        NoiseCipherState *encrypt_state = nullptr;
        NoiseCipherState *decrypt_state = nullptr;
    };

    template<dh_type _dh>
    struct dh_state : noncopyable {
        static constexpr dh_type dh = _dh;

        using buffer_private_key_type = buffer_type<get_private_key_size<dh>()>;
        using buffer_public_key_type  = buffer_type<get_public_key_size<dh>()>;
        using buffer_shared_secret_key_type =
            buffer_type<get_shared_secret_key_size<config.dh>()>;

    public:
        dh_state();
        dh_state(dh_state &&other);
        ~dh_state();

        dh_state &operator=(dh_state &&other);

    public:
        void generate_keypair();
        void generate_dependent_keypair(const dh_state &other);
        void calculate(const dh_state                &other,
                       buffer_shared_secret_key_type &shared_secret_key);

        void       set_role(noise_role role);
        void       set_keypair(const keypair_type &keypair);
        void       set_public_key(const buffer_public_key_type &public_key);
        noise_role get_role();
        void       get_keypair(keypair_type &keypair);

    private:
        NoiseDHState *dhstate = nullptr;
    };

    template<hash_type _hash>
    struct hash_state : noncopyable {
        static constexpr hash_type hash = _hash;

        using buffer_type = buffer_type<get_hash_size<hash>()>;

    public:
        hash_state();
        hash_state(hash_state &&other);
        ~hash_state();

        hash_state &operator=(hash_state &&other);

    public:
        buffer_type get_hash(std::span<const noheap::rbyte> buffer) const;

        void hkdf(std::span<const noheap::rbyte> buffer,
                  std::span<const noheap::rbyte> key, std::span<noheap::rbyte> output1,
                  std::span<noheap::rbyte> output2) const;

    private:
        // This is so because interface of this class does not imply an update function
        mutable NoiseHashState *hashstate = nullptr;
    };

public:
    noise_context() = default;
    noise_context(noise_context &&other);
    ~noise_context();

    noise_context &operator=(noise_context &&other);

public:
    void init(noise_role _role);
    void dump();
    void fallback();
    void start();
    void stop();

    noise_buffer_view                           &get_handshake_buffer() noexcept;
    noise_buffer_view                           &get_handshake_payload_buffer() noexcept;
    noise::noise_context<_config>::cipher_state &get_cipher_state() noexcept;

    noise_action get_action() const noexcept;
    noise_role   get_role() const noexcept;

    void set_handshake_message();
    void get_handshake_message();

    void                 set_prologue(buffer_prologue_extention_type ext);
    buffer_prologue_type get_prologue();
    buffer_key_type      get_remote_public_key();
    hash_state<_config.hash>::buffer_type get_handshake_hash();

    void set_local_keypair(const keypair_type &kp);
    void set_remote_public_key(const buffer_key_type &key);
    void set_pre_shared_key(const buffer_pre_shared_key_type &key);

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
noise::noise_context<_config>::random_state::random_state(random_state &&other) {
    (void) this->operator=(std::move(other));
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::random_state::~random_state() {
    noise_randstate_free(randstate);
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::random_state &
    noise::noise_context<_config>::random_state::operator=(random_state &&other) {
    std::swap(randstate, other.randstate);
    std::swap(padding_buffer, other.padding_buffer);
    return *this;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::random_state::pad() {
    auto &noise_buffer = padding_buffer.get();

    std::size_t ret;
    if ((ret = noise_randstate_pad(randstate, noise_buffer.data, noise_buffer.size,
                                   noise_buffer.max_size, NOISE_PADDING_RANDOM))
        != NOISE_ERROR_NONE) {
        handle_error(ret, "Failed to pad.");
    }

    padding_buffer.set({}, 0);
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
noise::noise_context<_config>::cipher_state::cipher_state(cipher_state &&other) {
    (void) this->operator=(std::move(other));
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::cipher_state::~cipher_state() {
    dump();
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::cipher_state &
    noise::noise_context<_config>::cipher_state::operator=(cipher_state &&other) {
    std::swap(encrypt_buffer, other.encrypt_buffer);
    std::swap(decrypt_buffer, other.decrypt_buffer);
    std::swap(encrypt_state, other.encrypt_state);
    std::swap(decrypt_state, other.decrypt_state);
    return *this;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::encrypt(
    std::span<const noheap::rbyte> buffer_ad) {
    check_encrypt_key();

    std::size_t ret;
    if ((ret = noise_cipherstate_encrypt_with_ad(
             encrypt_state, reinterpret_cast<const noheap::ubyte *>(buffer_ad.data()),
             buffer_ad.size(), &encrypt_buffer.get()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to encrypt.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::decrypt(
    std::span<const noheap::rbyte> buffer_ad) {
    check_decrypt_key();

    std::size_t ret;
    if ((ret = noise_cipherstate_decrypt_with_ad(
             decrypt_state, reinterpret_cast<const noheap::ubyte *>(buffer_ad.data()),
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
void noise::noise_context<_config>::cipher_state::set_encrypt_nonce(
    const buffer_nonce_type &nonce) {
    check_encrypt_key();

    std::size_t ret;
    if ((ret = noise_cipherstate_set_nonce(
             encrypt_state, reinterpret_cast<const noheap::ubyte *>(nonce.data()),
             get_nonce_size<config.cipher>()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set encrypting nonce.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_decrypt_nonce(
    const buffer_nonce_type &nonce) {
    check_decrypt_key();

    std::size_t ret;
    if ((ret = noise_cipherstate_set_nonce(
             decrypt_state, reinterpret_cast<const noheap::ubyte *>(nonce.data()),
             get_nonce_size<config.cipher>()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set decrypting nonce.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_encrypt_counter_block(
    std::uint64_t counter) {
    decltype(auto) nonce_tmp = this->get_encrypt_nonce();
    std::memcpy(&nonce_tmp, &counter, sizeof(counter));
    this->set_encrypt_nonce(nonce_tmp);
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_decrypt_counter_block(
    std::uint64_t counter) {
    decltype(auto) nonce_tmp = this->get_decrypt_nonce();
    std::memcpy(&nonce_tmp, &counter, sizeof(counter));
    this->set_decrypt_nonce(nonce_tmp);
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_encrypt_key(
    const buffer_key_type &key) {
    std::size_t ret;
    if ((ret = noise_cipherstate_init_key(
             encrypt_state, reinterpret_cast<const noheap::ubyte *>(key.data()),
             key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set encrypt key.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::set_decrypt_key(
    const buffer_key_type &key) {
    std::size_t ret;
    if ((ret = noise_cipherstate_init_key(
             decrypt_state, reinterpret_cast<const noheap::ubyte *>(key.data()),
             key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set decrypt key.");
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::buffer_nonce_type
    noise::noise_context<_config>::cipher_state::get_encrypt_nonce() const {
    buffer_nonce_type buffer_nonce;
    std::size_t       ret;
    if ((ret = noise_cipherstate_get_nonce(
             encrypt_state, reinterpret_cast<noheap::ubyte *>(buffer_nonce.data()),
             buffer_nonce.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get encrypt key.");
    return buffer_nonce;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::buffer_nonce_type
    noise::noise_context<_config>::cipher_state::get_decrypt_nonce() const {
    buffer_nonce_type buffer_nonce;
    std::size_t       ret;
    if ((ret = noise_cipherstate_get_nonce(
             decrypt_state, reinterpret_cast<noheap::ubyte *>(buffer_nonce.data()),
             buffer_nonce.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get decrypt key.");
    return buffer_nonce;
}
template<noise::noise_context_config _config>
std::uint64_t
    noise::noise_context<_config>::cipher_state::get_encrypt_counter_block() const {
    std::uint64_t  counter;
    decltype(auto) nonce_tmp = this->get_encrypt_nonce();
    std::memcpy(&counter, &nonce_tmp, sizeof(counter));
    return counter;
}
template<noise::noise_context_config _config>
std::uint64_t
    noise::noise_context<_config>::cipher_state::get_decrypt_counter_block() const {
    std::uint64_t  counter;
    decltype(auto) nonce_tmp = this->get_decrypt_nonce();
    std::memcpy(&counter, &nonce_tmp, sizeof(counter));
    return counter;
}
template<noise::noise_context_config _config>
bool noise::noise_context<_config>::cipher_state::has_encrypt_key() const {
    return noise_cipherstate_has_key(encrypt_state);
}
template<noise::noise_context_config _config>
bool noise::noise_context<_config>::cipher_state::has_decrypt_key() const {
    return noise_cipherstate_has_key(decrypt_state);
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
template<noise::noise_context_config _config>
void noise::noise_context<_config>::cipher_state::dump() {
    std::size_t ret;
    if ((ret = noise_cipherstate_free(encrypt_state)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free encrypt cipher state.");
    if ((ret = noise_cipherstate_free(decrypt_state)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free decrypt cipher state.");

    encrypt_state = decrypt_state = nullptr;
}

// DH state
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
noise::noise_context<_config>::dh_state<_dh>::dh_state() {
    std::size_t ret;
    if ((ret = noise_dhstate_new_by_id(&dhstate, static_cast<std::uint16_t>(dh)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init dh state.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
noise::noise_context<_config>::dh_state<_dh>::dh_state(dh_state &&other) {
    (void) this->operator=(std::move(other));
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
noise::noise_context<_config>::dh_state<_dh>::~dh_state() {
    std::size_t ret;
    if ((ret = noise_dhstate_free(dhstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free dh state.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
noise::noise_context<_config>::dh_state<_dh> &
    noise::noise_context<_config>::dh_state<_dh>::operator=(dh_state &&other) {
    std::swap(dhstate, other.dhstate);
    return *this;
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::generate_keypair() {
    std::size_t ret;
    if ((ret = noise_dhstate_generate_keypair(dhstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to generate keypair.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::generate_dependent_keypair(
    const dh_state &other) {
    std::size_t ret;
    if ((ret = noise_dhstate_generate_dependent_keypair(dhstate, other.dhstate))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to generate dependent keypair.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::calculate(
    const dh_state &other, buffer_shared_secret_key_type &shared_secret_key) {
    std::size_t ret, shared_secret_key_size;
    if ((ret = noise_dhstate_calculate(dhstate, other.dhstate, shared_secret_key.data(),
                                       &shared_secret_key_size))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to calculate dh.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::set_role(noise_role role) {
    noise_dhstate_set_role(dhstate, static_cast<std::uint16_t>(role));
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::set_keypair(
    const keypair_type &keypair) {
    std::size_t ret;
    if ((ret =
             noise_dhstate_set_keypair(dhstate, keypair.priv.data(), keypair.priv.size(),
                                       keypair.pub.data(), keypair.pub.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set keypair.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::set_public_key(
    const buffer_public_key_type &public_key) {
    std::size_t ret;
    if ((ret =
             noise_dhstate_set_public_key(dhstate, public_key.data(), public_key.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to set public key.");
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
noise::noise_role noise::noise_context<_config>::dh_state<_dh>::get_role() {
    return noise_role(noise_dhstate_get_role(dhstate));
}
template<noise::noise_context_config _config>
template<noise::dh_type _dh>
void noise::noise_context<_config>::dh_state<_dh>::get_keypair(keypair_type &keypair) {
    std::size_t ret;
    if ((ret = noise_dhstate_get_keypair(
             dhstate, reinterpret_cast<noheap::ubyte *>(keypair.priv.data()),
             keypair.priv.size(), reinterpret_cast<noheap::ubyte *>(keypair.pub.data()),
             keypair.pub.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get keypair.");
}

// Hash state
template<noise::noise_context_config _config>
template<noise::hash_type _hash>
noise::noise_context<_config>::hash_state<_hash>::hash_state() {
    std::size_t ret;
    if ((ret = noise_hashstate_new_by_id(&hashstate, static_cast<std::uint16_t>(hash)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to init hash state.");
}
template<noise::noise_context_config _config>
template<noise::hash_type _hash>
noise::noise_context<_config>::hash_state<_hash>::hash_state(hash_state &&other) {
    (void) this->operator=(std::move(other));
}
template<noise::noise_context_config _config>
template<noise::hash_type _hash>
noise::noise_context<_config>::hash_state<_hash>::~hash_state() {
    std::size_t ret;
    if ((ret = noise_hashstate_free(hashstate)) != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to free hash state.");
}
template<noise::noise_context_config _config>
template<noise::hash_type _hash>
noise::noise_context<_config>::hash_state<_hash> &
    noise::noise_context<_config>::hash_state<_hash>::operator=(hash_state &&other) {
    std::swap(hashstate, other.hashstate);
    return *this;
}
template<noise::noise_context_config _config>
template<noise::hash_type _hash>
noise::noise_context<_config>::hash_state<_hash>::buffer_type
    noise::noise_context<_config>::hash_state<_hash>::get_hash(
        std::span<const noheap::rbyte> buffer) const {
    decltype(get_hash(buffer)) buffer_tmp{};
    std::size_t                ret;
    if ((ret = noise_hashstate_hash_one(
             hashstate, reinterpret_cast<const noheap::ubyte *>(buffer.data()),
             buffer.size(), reinterpret_cast<noheap::ubyte *>(buffer_tmp.data()),
             buffer_tmp.size()))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get hash.");

    return buffer_tmp;
}
template<noise::noise_context_config _config>
template<noise::hash_type _hash>
void noise::noise_context<_config>::hash_state<_hash>::hkdf(
    std::span<const noheap::rbyte> buffer, std::span<const noheap::rbyte> key,
    std::span<noheap::rbyte> output1, std::span<noheap::rbyte> output2) const {
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
noise::noise_context<_config>::noise_context(noise_context &&other) {
    (void) this->operator=(std::move(other));
}
template<noise::noise_context_config _config>
noise::noise_context<_config> &
    noise::noise_context<_config>::operator=(noise_context &&other) {
    std::swap(handshakestate, other.handshakestate);
    std::swap(cipher_st, other.cipher_st);
    std::swap(handshake_buffer, other.handshake_buffer);
    std::swap(handshake_payload_buffer, other.handshake_payload_buffer);
    return *this;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::~noise_context<_config>() {
    this->dump();
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::init(noise_role role) {
    std::size_t ret;
    if ((ret = noise_handshakestate_new_by_id(&handshakestate, &nid_static,
                                              static_cast<std::uint16_t>(role)))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to get new state of handshake.");
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::dump() {
    if (handshakestate)
        noise_handshakestate_free(handshakestate);

    handshakestate           = nullptr;
    cipher_st                = {};
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
    NoiseCipherState *encrypt_state, *decrypt_state;

    std::size_t ret;
    if ((ret = noise_handshakestate_split(handshakestate, &encrypt_state, &decrypt_state))
        != NOISE_ERROR_NONE)
        handle_error(ret, "Failed to split handshake.");

    cipher_st.set_states(encrypt_state, decrypt_state);
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::noise_buffer_view &
    noise::noise_context<_config>::get_handshake_buffer() noexcept {
    return handshake_buffer;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::noise_buffer_view &
    noise::noise_context<_config>::get_handshake_payload_buffer() noexcept {
    return handshake_payload_buffer;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::cipher_state &
    noise::noise_context<_config>::get_cipher_state() noexcept {
    return cipher_st;
}

template<noise::noise_context_config _config>
noise::noise_action noise::noise_context<_config>::get_action() const noexcept {
    return noise_action(noise_handshakestate_get_action(handshakestate));
}
template<noise::noise_context_config _config>
noise::noise_role noise::noise_context<_config>::get_role() const noexcept {
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
void noise::noise_context<_config>::set_prologue(buffer_prologue_extention_type ext) {
    prologue.ext = ext;

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
    std::copy(buffer_tmp.begin(), buffer_tmp.begin() + sizeof(prologue),
              reinterpret_cast<char *>(&prologue));
    return buffer_tmp;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::buffer_key_type
    noise::noise_context<_config>::get_remote_public_key() {
    buffer_key_type buffer_tmp{};
    NoiseDHState   *dh = noise_handshakestate_get_remote_public_key_dh(handshakestate);

    std::size_t ret;
    if ((ret = noise_dhstate_get_public_key(
             dh, reinterpret_cast<noheap::ubyte *>(buffer_tmp.data()),
             buffer_tmp.size())))
        handle_error(ret, "Failed to get remote public key.");

    return buffer_tmp;
}
template<noise::noise_context_config _config>
noise::noise_context<_config>::hash_state<_config.hash>::buffer_type
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
void noise::noise_context<_config>::set_pre_shared_key(
    const buffer_pre_shared_key_type &key) {
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
void noise::noise_context<_config>::set_remote_public_key(const buffer_key_type &key) {
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

    buffer_key_type derived_public_key;
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
    keypair_type                       keypair;
    noise_context::dh_state<config.dh> dh_state_tmp;

    dh_state_tmp.generate_keypair();
    dh_state_tmp.get_keypair(keypair);

    return keypair;
}
template<noise::noise_context_config _config>
void noise::noise_context<_config>::handle_error(std::size_t      error,
                                                 std::string_view extention_error) {
    if (error) {
        noheap::buffer_chars_type<64> buffer_noise_error{};
        noise_strerror(error, buffer_noise_error.data(), buffer_noise_error.size());
        log.throw_exception("{} {}", extention_error, buffer_noise_error.data());
    } else
        log.throw_exception("{}", extention_error);
}

#endif
