#ifndef CRYPT_HPP
#define CRYPT_HPP

#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/sha.h>

#include "utils.hpp"

class openssl_context {
  public:
    template <std::size_t size>
    using buffer_type = noheap::buffer_bytes_type<size, std::uint8_t>;

  public:
    enum class algorithm { SHA1 = 0, SHA256, MD5 };
    template <algorithm algorithm_crypt>
    static consteval std::size_t get_hash_size() {
        if constexpr (algorithm_crypt == algorithm::SHA1)
            return 20;
        else if constexpr (algorithm_crypt == algorithm::SHA256)
            return 32;
        else if constexpr (algorithm_crypt == algorithm::MD5)
            return 16;
    }

    template <algorithm algorithm_crypt> struct hash {
        using type = buffer_type<get_hash_size<algorithm_crypt>()>;
    };

  public:
    openssl_context();
    ~openssl_context();

  public:
    template <algorithm algorithm_crypt>
    hash<algorithm_crypt>::type get_hash(std::string_view data);

    template <std::size_t buffer_size>
    static buffer_type<buffer_size> get_random_bytes();

    template <algorithm algorithm_crypt>
    static bool hash_compare(const hash<algorithm_crypt>::type &h1,
                             const hash<algorithm_crypt>::type &h2);

  public:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("CRYPT");

  protected:
    static constexpr log_handler log{buffer_owner};

  private:
    EVP_MD_CTX *ctx;
};

openssl_context::openssl_context() {
    ctx = EVP_MD_CTX_new();
    if (!ctx)
        throw noheap::runtime_error(buffer_owner,
                                    "Failed to init the openssl context.");
}
openssl_context::~openssl_context() { EVP_MD_CTX_free(ctx); }

template <openssl_context::algorithm algorithm_crypt>
openssl_context::hash<algorithm_crypt>::type
openssl_context::get_hash(std::string_view buffer) {
    const EVP_MD *algorithm_ctx;
    if constexpr (algorithm_crypt == algorithm::SHA1)
        algorithm_ctx = EVP_sha1();
    else if constexpr (algorithm_crypt == algorithm::SHA256)
        algorithm_ctx = EVP_sha256();
    else if constexpr (algorithm_crypt == algorithm::MD5)
        algorithm_ctx = EVP_md5();

    if (EVP_DigestInit_ex(ctx, algorithm_ctx, nullptr) != 1)
        throw noheap::runtime_error(buffer_owner,
                                    "Failed to set SHA1 alghorithm.");
    if (EVP_DigestUpdate(ctx, buffer.data(), buffer.size()) != 1)
        throw noheap::runtime_error(buffer_owner,
                                    "Failed to set data for openssl context.");

    typename hash<algorithm_crypt>::type buffer_tmp;
    if (EVP_DigestFinal_ex(ctx, buffer_tmp.data(), nullptr) != 1)
        throw noheap::runtime_error(buffer_owner, "Failed to calculate hash.");

    return buffer_tmp;
}
template <std::size_t size>
openssl_context::buffer_type<size> openssl_context::get_random_bytes() {
    buffer_type<size> buffer_tmp;
    RAND_bytes(buffer_tmp.data(), size);
    return buffer_tmp;
}
template <openssl_context::algorithm algorithm_crypt>
bool openssl_context::hash_compare(const hash<algorithm_crypt>::type &h1,
                                   const hash<algorithm_crypt>::type &h2) {
    std::size_t mismatch = 0;
    for (std::size_t i = 0; i < hash<algorithm_crypt>::size; ++i)
        mismatch |= h1[i] ^ h2[i];

    return !mismatch;
}
#endif
