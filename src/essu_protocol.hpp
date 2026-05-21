#ifndef ESSU_HPP
#define ESSU_HPP

#include "essu_base.hpp"
#include "essu_noise_handshake_context.hpp"

namespace essu {

// Session info corresponding to an individual node
struct session_info_type {
    friend struct protocol;

    enum class status_enum : std::size_t {
        START = 0,
        EXCHANGE,
        STOP,
        COMPLETE,
    };

public:
    session_info_type(network::ipv _v, network::buffer_address_type _remote_address,
                      std::uint16_t _remote_port, noise::noise_role _role,
                      noise::buffer_prologue_extention_type      _ext,
                      const noise_context_type::buffer_key_type &_remote_public_key,
                      const noise::buffer_pre_shared_key_type   &_pre_shared_key,
                      const noise_context_type::keypair_type    &_local_keypair)
        : remote_endpoint(_v, _remote_address, _remote_port),
          handshake_context(_role, _ext, _remote_public_key, _pre_shared_key,
                            _local_keypair) {
        reset_state();
    }
    session_info_type(const session_info_type &) = delete;
    session_info_type(session_info_type &&)      = delete;

private:
    void reset_state() {
        status                        = status_enum::START;
        batch_sent_number             = 0;
        batch_received_number         = 0;
        sender_unit_number            = 0;
        receiver_unit_number          = 0;
        sender_key_iteration_number   = 0;
        receiver_key_iteration_number = 0;
        undecrypted_batch_number      = 0;
        was_sent_retry                = false;
        was_received_retry            = false;
    }

public:
    network::native_endpoint remote_endpoint;

private:
    noise_handshake_context handshake_context;
    std::uint64_t           handshake_number = 0;

    status_enum   status;
    std::uint64_t batch_sent_number;
    std::uint64_t batch_received_number;
    std::uint64_t sender_unit_number;
    std::uint64_t receiver_unit_number;
    std::uint64_t sender_key_iteration_number;
    std::uint64_t receiver_key_iteration_number;
    std::uint64_t undecrypted_batch_number;
    bool          was_sent_retry;
    bool          was_received_retry;
};

struct protocol final {
    protocol() = delete;

public:
    static inline void prepare(packet_type &pckt, session_info_type &session_info);
    static inline void handle(packet_type &pckt, session_info_type &session_info);
    static inline void start_handshake(session_info_type &session_info);
    static inline void stop_handshake(session_info_type &session_info);
    static inline session_info_type::status_enum
        update_status(session_info_type &session_info);

    static inline std::uint64_t
        get_handshake_number(const session_info_type &session_info);
    static inline std::uint64_t get_handshake_id(const session_info_type &session_info);
    static inline bool          can_send_packet(const session_info_type &session_info);
    static inline bool          can_receive_packet(const session_info_type &session_info);

private:
    static inline void check_protocol_compliance(const session_info_type &session_info,
                                                 packet_type             &pckt);
    static inline noise::buffer_type<header_data_size> derive_header_obfs_key(
        typename noise_context_type::cipher_state &header_cipher_state);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_PROTOCOL");
    static constexpr log_handler log{buffer_owner};
};

} // namespace essu

void essu::protocol::prepare(packet_type &pckt, session_info_type &session_info) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_sender();
    decltype(auto) random_state     = session_info.handshake_context.get_random_state();
    bool session_handshake_complete = session_info.handshake_context.is_complete();

    check_protocol_compliance(session_info, pckt);
    if (session_handshake_complete && !can_send_packet(session_info))
        log.throw_exception("Expected to rehandshake.");

    // Inits packet like handshake message if necessary
    if (!session_handshake_complete && can_send_packet(session_info)
        && session_info.batch_sent_number % 2 == 0)
        session_info.handshake_context.init_packet(pckt);

    // If retry or available batch number is reached
    if (session_info.was_received_retry
        || (session_handshake_complete
            && session_info.batch_sent_number
                   == session_info.handshake_context.get_available_batch_number())) {
        set_control_session_packet(pckt, unit_type::unit_type_enum::retry);
        session_info.was_sent_retry = true;
    }

    // Forces control-dummy and last unit to be dummy
    if (!is_control_session_packet_type(pckt))
        set_dummy_unit(get_control_unit(pckt));
    set_dummy_unit(get_last_unit(pckt));

    for (std::uint64_t i = 0; i < pckt->units.size(); ++i) {
        unit_type &unit = pckt->units[i];

        unit.header.number               = session_info.sender_unit_number++;
        unit.header.key_iteration_number = session_info.sender_key_iteration_number;

        // Adds random padding
        {
            // Determines payload size of the unit to define size of random
            // padding
            std::uint64_t payload_size;
            switch (unit.header.type) {
                case unit_type::unit_type_enum::session_request:
                case unit_type::unit_type_enum::session_created:
                case unit_type::unit_type_enum::session_confirmed:
                    payload_size = unit.buffer.size();
                    break;
                case unit_type::unit_type_enum::data:
                    payload_size = payload_data_size;
                    break;
                case unit_type::unit_type_enum::hole_punch:
                    payload_size = 8;
                    break;
                case unit_type::unit_type_enum::dummy:
                case unit_type::unit_type_enum::retry:
                    payload_size = 0;
                    break;
                default:
                    log.throw_exception("Packet type[{}] is not allowed.",
                                        static_cast<std::size_t>(unit.header.type));
            }

            // Adds random padding after payload data
            random_state.padding_buffer.set(
                {reinterpret_cast<noheap::rbyte *>(unit.buffer.data()),
                 unit.buffer.size()},
                payload_size);
            random_state.pad();
        }

        // Encrypts buffer data and authenticates based on the header
        if (session_handshake_complete) {
            payload_cipher_state.encrypt_buffer.set(
                {unit.buffer.data(), unit.buffer.size()}, unit.buffer_size_without_mac());
            payload_cipher_state.encrypt(
                {reinterpret_cast<noheap::rbyte *>(&unit.header), sizeof(unit.header)});

            // Performs rekey for encryption
            if (unit.header.number % unit_per_rekey_number == 0) {
                payload_cipher_state.rekey_encrypt();
                ++session_info.sender_key_iteration_number;
            }
        }

        // Generates header obfuscation key based on the unit_number
        auto obfs_key_tmp = derive_header_obfs_key(header_cipher_state);

        // Adds header data obfuscation
        std::transform(reinterpret_cast<noheap::rbyte *>(&unit.header),
                       reinterpret_cast<noheap::rbyte *>(&unit.header)
                           + sizeof(unit.header),
                       obfs_key_tmp.data(),
                       reinterpret_cast<noheap::rbyte *>(&unit.header), std::bit_xor{});
    }

    // Shuffles units in batch
    std::shuffle(pckt->units.begin(), pckt->units.end(), random_state.generator);

    ++session_info.batch_sent_number;
}

void essu::protocol::handle(packet_type &pckt, session_info_type &session_info) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_receiver();
    bool session_handshake_complete = session_info.handshake_context.is_complete();

    if (session_handshake_complete && !can_receive_packet(session_info))
        log.throw_exception("Expected to rehandshake.");

    // Selects possible unit number
    std::uint64_t count_decrypted_units   = 0;
    std::uint64_t attempts_decrypt_number = 0;
    std::uint64_t available_units_window_number =
        session_info.receiver_unit_number + batch_window_number * batch_units_number;
    std::uint64_t possible_unit_number = session_info.receiver_unit_number;
    for (; possible_unit_number < available_units_window_number; ++possible_unit_number) {
        // Generates header obfuscation key based on the possible_unit_number
        auto obfs_key_tmp = derive_header_obfs_key(header_cipher_state);

        for (auto &unit : pckt->units) {
            unit_type test_unit = unit;

            // Deletes header data obfuscation
            std::transform(reinterpret_cast<noheap::rbyte *>(&test_unit.header),
                           reinterpret_cast<noheap::rbyte *>(&test_unit.header)
                               + sizeof(test_unit.header),
                           obfs_key_tmp.data(),
                           reinterpret_cast<noheap::rbyte *>(&test_unit.header),
                           std::bit_xor{});

            if (test_unit.header.number != possible_unit_number)
                continue;

            // Loop handling rekeys performed on the remote node.
            for (; session_info.receiver_key_iteration_number
                   < test_unit.header.key_iteration_number;
                 ++session_info.receiver_key_iteration_number)
                payload_cipher_state.rekey_decrypt();

            // Tries to decrypt buffer data
            if (session_handshake_complete) {
                payload_cipher_state.decrypt_buffer.set(
                    {test_unit.buffer.data(), test_unit.buffer.size()},
                    test_unit.buffer.size());
                try {
                    payload_cipher_state.decrypt(
                        {reinterpret_cast<noheap::rbyte *>(&test_unit.header),
                         sizeof(test_unit.header)});
                } catch (const noheap::runtime_error &excp) {
                    // Increments counter block of decrypt nonce
                    noheap::println("{}",
                                    payload_cipher_state.get_decrypt_counter_block());
                    payload_cipher_state.set_decrypt_counter_block(
                        payload_cipher_state.get_decrypt_counter_block() + 1);
                    ++attempts_decrypt_number;
                    continue;
                }
            }

            unit = test_unit;
            ++count_decrypted_units;
            break;
        }

        if (count_decrypted_units == pckt->units.size()) {
            session_info.receiver_unit_number = possible_unit_number;
            break;
        }
    }

    // If it was not possible to decrypt all units in batch
    if (count_decrypted_units != pckt->units.size()) {
        ++session_info.undecrypted_batch_number;

        // Sets the previous value of decrypt payload and encrypt header nonces
        {
            if (session_handshake_complete)
                payload_cipher_state.set_decrypt_counter_block(
                    payload_cipher_state.get_decrypt_counter_block()
                    - attempts_decrypt_number);
            header_cipher_state.set_encrypt_counter_block(
                header_cipher_state.get_encrypt_counter_block()
                - (possible_unit_number - session_info.receiver_unit_number));
        }

        // If performs handshake or was failed to decrypt
        // max_undecrypted_batches_number count packets after handshake
        if (!session_handshake_complete
            || session_info.undecrypted_batch_number == max_undecrypted_batch_number)
            log.throw_exception("Failed to decrypt last batches. [attempts to decrypt payload: {}]", attempts_decrypt_number);
        return;
    } else
        session_info.undecrypted_batch_number = 0;

    // Restores order of units in batch
    std::sort(pckt->units.begin(), pckt->units.end(),
              [](const auto &el_left, const auto &el_right) {
                  return el_left.header.number < el_right.header.number;
              });

    ++session_info.receiver_unit_number;
    ++session_info.batch_received_number;

    // If available batch number is reached
    if (session_handshake_complete
        && get_control_unit(pckt).header.type == unit_type::unit_type_enum::retry)
        session_info.was_received_retry = true;

    // Handles packet like handshake message if necessary
    if (!session_handshake_complete && can_receive_packet(session_info)
        && is_control_session_packet_type(pckt))
        session_info.handshake_context.handle_packet(std::move(pckt));
}
void essu::protocol::start_handshake(session_info_type &session_info) {
    session_info.reset_state();
    session_info.handshake_context.start();
}
void essu::protocol::stop_handshake(session_info_type &session_info) {
    session_info.reset_state();
    session_info.handshake_context.stop();
    ++session_info.handshake_number;
    session_info.status = session_info_type::status_enum::COMPLETE;
}
essu::session_info_type::status_enum
    essu::protocol::update_status(session_info_type &session_info) {
    auto action = session_info.handshake_context.get_action();
    if (action == noise::noise_action::NONE
        && session_info.handshake_context.is_complete())
        session_info.status = session_info_type::status_enum::COMPLETE;
    else if (action == noise::noise_action::SPLIT)
        session_info.status = session_info_type::status_enum::STOP;
    else if (action == noise::noise_action::WRITE_MESSAGE
             || action == noise::noise_action::READ_MESSAGE)
        session_info.status = session_info_type::status_enum::EXCHANGE;
    return session_info.status;
}

std::uint64_t
    essu::protocol::get_handshake_number(const session_info_type &session_info) {
    return session_info.handshake_number;
}
std::uint64_t essu::protocol::get_handshake_id(const session_info_type &session_info) {
    return session_info.handshake_context.get_handshake_id();
}
bool essu::protocol::can_send_packet(const session_info_type &session_info) {
    return session_info.handshake_context.get_action()
               == noise::noise_action::WRITE_MESSAGE
           || (session_info.handshake_context.is_complete()
               && !session_info.was_sent_retry);
}
bool essu::protocol::can_receive_packet(const session_info_type &session_info) {
    return session_info.handshake_context.get_action()
               == noise::noise_action::READ_MESSAGE
           || (session_info.handshake_context.is_complete()
               && !session_info.was_received_retry);
}

void essu::protocol::check_protocol_compliance(const session_info_type &session_info,
                                               packet_type             &pckt) {
    decltype(auto) control_unit = get_control_unit(pckt);

    if (session_info.handshake_number == max_available_handshake_number)
        log.throw_exception("Limit of handshakes has been reached.");
    if (session_info.handshake_context.is_complete()) {
        if (control_unit.header.type != unit_type::unit_type_enum::dummy
            || is_control_session_unit_type(pckt->units[0].header.type)
            || is_control_session_unit_type(pckt->units[1].header.type)
            || is_control_session_unit_type(pckt->units[3].header.type))
            log.throw_exception("Invalid packet setting after handshake.");
    } else {
        if (!is_dummy_packet_type(pckt))
            log.throw_exception("Invalid packet setting for handshake.");
    }
}
noise::buffer_type<essu::header_data_size> essu::protocol::derive_header_obfs_key(
    typename noise_context_type::cipher_state &header_cipher_state) {
    noise::buffer_type<sizeof(typename essu::unit_type::header_data_type)
                       + noise_config.mac_size>
        obfs_key_tmp{};
    header_cipher_state.encrypt_buffer.set({obfs_key_tmp.data(), obfs_key_tmp.size()},
                                           obfs_key_tmp.size() - noise_config.mac_size);
    header_cipher_state.encrypt({});

    return noheap::to_buffer<decltype(derive_header_obfs_key(header_cipher_state))>(
        obfs_key_tmp);
}

#endif
