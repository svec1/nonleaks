#ifndef MLDSA_NATIVE_HPP
#define MLDSA_NATIVE_HPP

#include <mldsa_native.h>
#include <mldsa_native_config.h>

#include "utils.hpp"

namespace mldsa_native {

template<std::size_t size>
using buffer_type = noheap::buffer_bytes_type<size, noheap::rbyte>;

template<auto _context>
    requires noheap::Buffer_bytes_or_chars<decltype(_context)>
class mldsa_native_wrapper {
public:
    static constexpr auto        context = _context;
    static constexpr std::size_t type    = MLD_CONFIG_PARAMETER_SET;

    using buffer_priv_type      = buffer_type<CRYPTO_SECRETKEYBYTES>;
    using buffer_pub_type       = buffer_type<CRYPTO_PUBLICKEYBYTES>;
    using buffer_signature_type = buffer_type<CRYPTO_BYTES>;

    struct keypair_type {
        buffer_priv_type priv;
        buffer_pub_type  pub;
    };

public:
    mldsa_native_wrapper() = delete;

    static inline void generate_keypair(keypair_type &keypair);
    static inline void sign_data(const buffer_priv_type        &private_key,
                                 const std::span<noheap::ubyte> buffer_data,
                                 buffer_signature_type         &buffer_signature);
    static inline void sign_data(const buffer_priv_type        &private_key,
                                 const std::span<noheap::ubyte> buffer_data,
                                 std::span<noheap::ubyte>       output_data);
    static inline bool sign_validate(const buffer_pub_type         &public_key,
                                     const std::span<noheap::ubyte> buffer_data,
                                     const buffer_signature_type   &buffer_signature);
    static inline bool sign_open(const buffer_pub_type         &public_key,
                                 const std::span<noheap::ubyte> buffer_data,
                                 std::span<noheap::ubyte>       output_data);
};

} // namespace mldsa_native

template<auto _context>
    requires noheap::Buffer_bytes_or_chars<decltype(_context)>
void mldsa_native::mldsa_native_wrapper<_context>::generate_keypair(
    keypair_type &keypair) {
    if (crypto_sign_keypair(keypair.priv.data(), keypair.pub.data()))
        throw noheap::runtime_error("Failed to generate keypair.");
}
template<auto _context>
    requires noheap::Buffer_bytes_or_chars<decltype(_context)>
void mldsa_native::mldsa_native_wrapper<_context>::sign_data(
    const buffer_priv_type &private_key, const std::span<noheap::ubyte> buffer_data,
    buffer_signature_type &buffer_signature) {
    std::size_t signature_size = 0;
    if (crypto_sign_signature(
            reinterpret_cast<noheap::ubyte *>(buffer_signature.data()), &signature_size,
            reinterpret_cast<const noheap::ubyte *>(buffer_data.data()),
            buffer_data.size(), reinterpret_cast<const noheap::ubyte *>(context.data()),
            context.size(), reinterpret_cast<const noheap::ubyte *>(private_key.data())))
        throw noheap::runtime_error("Failed to sign data.");
}
template<auto _context>
    requires noheap::Buffer_bytes_or_chars<decltype(_context)>
void mldsa_native::mldsa_native_wrapper<_context>::sign_data(
    const buffer_priv_type &private_key, const std::span<noheap::ubyte> buffer_data,
    std::span<noheap::ubyte> output_data) {
    std::size_t output_data_size = 0;

    if (output_data.size()
        < buffer_data.size() + noheap::buffer_size<buffer_signature_type>)
        throw noheap::runtime_error("Invalid output data size.");

    if (crypto_sign(
            reinterpret_cast<noheap::ubyte *>(output_data.data()), &output_data_size,
            reinterpret_cast<const noheap::ubyte *>(buffer_data.data()),
            buffer_data.size(), reinterpret_cast<const noheap::ubyte *>(context.data()),
            context.size(), reinterpret_cast<const noheap::ubyte *>(private_key.data())))
        throw noheap::runtime_error("Failed to sign data.");
}
template<auto _context>
    requires noheap::Buffer_bytes_or_chars<decltype(_context)>
bool mldsa_native::mldsa_native_wrapper<_context>::sign_open(
    const buffer_pub_type &public_key, const std::span<noheap::ubyte> buffer_data,
    std::span<noheap::ubyte> output_data) {
    std::size_t output_data_size = 0;

    if (buffer_data.size() < noheap::buffer_size<buffer_signature_type>
        || output_data.size()
               < buffer_data.size() - noheap::buffer_size<buffer_signature_type>)
        throw noheap::runtime_error("Invalid output data size.");

    return !crypto_sign_open(
        reinterpret_cast<noheap::ubyte *>(output_data.data()), &output_data_size,
        reinterpret_cast<const noheap::ubyte *>(buffer_data.data()), buffer_data.size(),
        reinterpret_cast<const noheap::ubyte *>(context.data()), context.size(),
        reinterpret_cast<const noheap::ubyte *>(public_key.data()));
}
template<auto _context>
    requires noheap::Buffer_bytes_or_chars<decltype(_context)>
bool mldsa_native::mldsa_native_wrapper<_context>::sign_validate(
    const buffer_pub_type &public_key, const std::span<noheap::ubyte> buffer_data,
    const buffer_signature_type &buffer_signature) {
    if (crypto_sign_verify(
            reinterpret_cast<const noheap::ubyte *>(buffer_signature.data()),
            buffer_signature.size(),
            reinterpret_cast<const noheap::ubyte *>(buffer_data.data()),
            buffer_data.size(), reinterpret_cast<const noheap::ubyte *>(context.data()),
            context.size(), reinterpret_cast<const noheap::ubyte *>(public_key.data())))
        throw noheap::runtime_error("Failed to verify signing of data.");
}

#endif
