#ifndef ESSU_HPP
#define ESSU_HPP

#include "essu_base.hpp"
#include "essu_noise_handshake_context.hpp"

namespace essu {

// Session info corresponding to an individual node
struct session_info_type : noncopyable {
    friend class protocol;

    enum class handshake_status_enum : std::size_t {
        START = 0,
        EXCHANGE,
        STOP,
        COMPLETE,
    };

protected:
    session_info_type(network::native_endpoint _remote_endpoint, noise::noise_role _role,
                      noise::buffer_prologue_extention_type      _ext,
                      const noise_context_type::buffer_key_type &_remote_public_key,
                      const noise::buffer_pre_shared_key_type   &_pre_shared_key,
                      const noise_context_type::keypair_type    &_local_keypair)
        : remote_endpoint(_remote_endpoint),
          handshake_context(_role, _ext, _remote_public_key, _pre_shared_key,
                            _local_keypair) {
        reset_state();
    }
    session_info_type(session_info_type &&) = default;

public:
    decltype(auto) get_remote_endpoint() const noexcept { return (remote_endpoint); }
    decltype(auto) get_constructed_random_uint16() const noexcept {
        return constructed_random_uint16;
    }
    bool payload_stream_is_available() const noexcept {
        return handshake_status == handshake_status_enum::COMPLETE && !was_sent_retry
               && !was_received_retry;
    }

private:
    void reset_state() noexcept {
        batch_sent_number             = 0;
        batch_received_number         = 0;
        batch_received_skipped_number = 0;
        sender_unit_number            = 0;
        receiver_unit_number          = 0;
        sender_key_iteration_number   = 0;
        receiver_key_iteration_number = 0;
        was_sent_retry                = false;
        was_received_retry            = false;
    }

private:
    network::native_endpoint remote_endpoint;
    noise_handshake_context  handshake_context;

    handshake_status_enum handshake_status{};
    std::uint16_t         handshake_number{};
    std::uint16_t         constructed_random_uint16{
        std::uniform_int_distribution<decltype(constructed_random_uint16)>(0, -1)(
            handshake_context.get_random_state())};

    std::uint32_t batch_sent_number;
    std::uint32_t batch_received_number;
    std::uint32_t batch_received_skipped_number;
    std::uint32_t sender_unit_number;
    std::uint32_t receiver_unit_number;
    std::uint32_t sender_key_iteration_number;
    std::uint32_t receiver_key_iteration_number;
    bool          was_sent_retry;
    bool          was_received_retry;
};

class protocol final {
    protocol() = delete;

public:
    static inline void prepare(session_info_type &session_info, packet_type &pckt);
    static inline bool try_handle(session_info_type &session_info, packet_type &pckt);
    static inline void start_handshake(session_info_type &session_info);
    static inline void stop_handshake(session_info_type &session_info);
    static inline session_info_type::handshake_status_enum
        get_handshake_status(const session_info_type &session_info) noexcept;
    static inline std::uint64_t
        get_handshake_number(const session_info_type &session_info) noexcept;

    static inline noise_handshake_context::buffer_current_state_hash_type
        get_current_state_hash(const session_info_type &session_info);

private:
    static inline bool
                       determine_affiliation_packet(session_info_type &session_info,
                                                    const packet_type &pckt,
                                                    std::uint32_t     &different_receiver_unit_number);
    static inline void update_handshake_status(session_info_type &session_info) noexcept;
    static inline void check_packet_compliance(const session_info_type &session_info,
                                               const packet_type       &pckt);
    static inline noise::buffer_type<header_data_size> derive_header_obfs_key(
        typename noise_context_type::cipher_state &header_cipher_state);
};

} // namespace essu

void essu::protocol::prepare(session_info_type &session_info, packet_type &pckt) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_sender();
    decltype(auto) random_state      = session_info.handshake_context.get_random_state();
    decltype(auto) control_unit_type = (get_control_unit(pckt).header.type);
    bool           session_handshake_complete =
        (session_info.handshake_status
         == session_info_type::handshake_status_enum::COMPLETE);

    check_packet_compliance(session_info, pckt);

    // Inits packet like handshake message if necessary
    if (session_info.handshake_context.get_action() == noise::noise_action::WRITE_MESSAGE
        && std::uniform_int_distribution<std::size_t>(1, 100)(random_state)
               >= sent_handshake_batch_factor)
        session_info.handshake_context.init_packet(pckt);

    // If retry or available batch number is reached
    if (session_info.was_received_retry
        || (session_handshake_complete
            && session_info.batch_sent_number
                   == session_info.handshake_context.get_available_batch_number() - 1)) {
        control_unit_type           = unit_type::unit_type_enum::retry;
        session_info.was_sent_retry = true;
    }

    // Forces last unit to be dummy
    get_last_unit(pckt).header.type = unit_type::unit_type_enum::dummy;

    pckt.set_endpoint(session_info.remote_endpoint);

    for (std::size_t i = 0; i < pckt->units.size(); ++i) {
        decltype(auto) unit = pckt->units[i];

        // Sets some data of header
        unit.header.connection_id = session_info.handshake_context.get_handshake_id();
        unit.header.number        = session_info.sender_unit_number++;
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
                    abort_invalid_state();
            }

            // Adds random padding after payload data
            random_state.padding_buffer.set(unit.buffer, payload_size);
            random_state.pad();
        }

        // Encrypts buffer data and authenticates based on the header
        if (session_handshake_complete) {
            payload_cipher_state.encrypt_buffer.set(unit.buffer,
                                                    unit.buffer_size_without_mac());
            payload_cipher_state.encrypt(
                {reinterpret_cast<noheap::rbyte *>(&unit.header), sizeof(unit.header)});

            // Performs rekey for encryption
            if (unit.header.number % unit_per_rekey_number == 0) {
                payload_cipher_state.rekey_encrypt();
                ++session_info.sender_key_iteration_number;
            }
        }

        // Adds header data obfuscation
        std::transform(reinterpret_cast<noheap::rbyte *>(&unit.header),
                       reinterpret_cast<noheap::rbyte *>(&unit.header)
                           + sizeof(unit.header),
                       derive_header_obfs_key(header_cipher_state).data(),
                       reinterpret_cast<noheap::rbyte *>(&unit.header), std::bit_xor{});
    }

    // Shuffles units in batch
    std::shuffle(pckt->units.begin(), pckt->units.end(), random_state);
    ++session_info.batch_sent_number;
    update_handshake_status(session_info);
}

bool essu::protocol::try_handle(session_info_type &session_info, packet_type &pckt) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_receiver();
    decltype(auto) control_unit_type = (get_control_unit(pckt).header.type);
    bool           session_handshake_complete =
        (session_info.handshake_status
         == session_info_type::handshake_status_enum::COMPLETE);

    std::uint32_t different_receiver_unit_number;
    if (!determine_affiliation_packet(session_info, pckt, different_receiver_unit_number))
        return false;

    // Updates session's remote endpoint
    {
        decltype(auto) packet_endpoint = pckt.get_endpoint();
        bool endpoints_are_different = (packet_endpoint != session_info.remote_endpoint);
        if (endpoints_are_different)
            session_info.remote_endpoint = packet_endpoint;
    }

    if (different_receiver_unit_number == std::uint32_t(-1)) {
        if (session_info.batch_received_skipped_number == skip_batch_window_number)
            throw protocol_error("Window of skip batch has been reached.");

        set_dummy_packet(pckt);
        ++session_info.batch_received_skipped_number;
    } else {
        session_info.batch_received_skipped_number = 0;
        session_info.receiver_unit_number += different_receiver_unit_number;
        if (session_handshake_complete)
            payload_cipher_state.set_decrypt_counter_block(
                payload_cipher_state.get_decrypt_counter_block()
                + different_receiver_unit_number);

        // Selects possible unit number
        std::uint64_t decrypted_units_number = 0;
        std::uint64_t available_units_window_number =
            session_info.receiver_unit_number + batch_units_number;

        for (; session_info.receiver_unit_number < available_units_window_number;
             ++session_info.receiver_unit_number) {
            // Generates header obfuscation key based
            decltype(auto) obfs_key_tmp = derive_header_obfs_key(header_cipher_state);

            for (auto &unit : pckt->units) {
                {
                    auto test_header = unit.header;

                    // Deletes header data obfuscation
                    std::transform(reinterpret_cast<noheap::rbyte *>(&test_header),
                                   reinterpret_cast<noheap::rbyte *>(&test_header)
                                       + sizeof(test_header),
                                   obfs_key_tmp.data(),
                                   reinterpret_cast<noheap::rbyte *>(&test_header),
                                   std::bit_xor{});

                    if (test_header.connection_id
                            != session_info.handshake_context.get_handshake_id()
                        || test_header.number != session_info.receiver_unit_number)
                        continue;

                    unit.header = test_header;
                }

                // Tries to decrypt buffer data
                if (session_handshake_complete) {
                    // Loop handling rekeys performed in the remote endpoint
                    for (; session_info.receiver_key_iteration_number
                           < unit.header.key_iteration_number;
                         ++session_info.receiver_key_iteration_number)
                        payload_cipher_state.rekey_decrypt();

                    payload_cipher_state.decrypt_buffer.set(unit.buffer,
                                                            unit.buffer.size());
                    try {
                        payload_cipher_state.decrypt(
                            {reinterpret_cast<noheap::rbyte *>(&unit.header),
                             sizeof(unit.header)});
                    } catch (const noheap::runtime_error &excp) {
                        throw protocol_error("Invalid MAC.");
                    }
                }

                ++decrypted_units_number;
                break;
            }
        }

        if (decrypted_units_number != pckt->units.size())
            throw protocol_error("Invalid header of units: {}", decrypted_units_number);

        // Restores order of units in batch
        std::sort(pckt->units.begin(), pckt->units.end(),
                  [](const auto &el_left, const auto &el_right) {
                      return el_left.header.number < el_right.header.number;
                  });

        check_packet_compliance(session_info, pckt);

        // If handshake is completed and the packet has a control unit retry
        if (session_handshake_complete
            && control_unit_type == unit_type::unit_type_enum::retry) {
            session_info.was_received_retry = true;
        }

        // Handles the packet like handshake message if necessary
        if (session_info.handshake_context.get_action()
                == noise::noise_action::READ_MESSAGE
            && is_control_session_unit_type(control_unit_type))
            session_info.handshake_context.handle_packet(std::move(pckt));

        ++session_info.batch_received_number;
        update_handshake_status(session_info);
    }

    return true;
}
void essu::protocol::start_handshake(session_info_type &session_info) {
    if (session_info.handshake_number == max_available_handshake_number)
        throw protocol_error("Limit of handshakes has been reached.");

    session_info.reset_state();
    session_info.handshake_context.start();
    update_handshake_status(session_info);
}
void essu::protocol::stop_handshake(session_info_type &session_info) {
    session_info.reset_state();
    session_info.handshake_context.stop();
    ++session_info.handshake_number;
    update_handshake_status(session_info);
}
std::uint64_t
    essu::protocol::get_handshake_number(const session_info_type &session_info) noexcept {
    return session_info.handshake_number;
}
essu::session_info_type::handshake_status_enum
    essu::protocol::get_handshake_status(const session_info_type &session_info) noexcept {
    return session_info.handshake_status;
}

bool essu::protocol::determine_affiliation_packet(
    session_info_type &session_info, const packet_type &pckt,
    std::uint32_t &different_receiver_unit_number) {
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_receiver();

    std::uint64_t count_suitable_units = 0;
    std::uint64_t available_units_window_number =
        session_info.receiver_unit_number + batch_window_number * batch_units_number;
    std::uint64_t possible_unit_number = session_info.receiver_unit_number;
    for (; possible_unit_number < available_units_window_number; ++possible_unit_number) {
        // Generates header obfuscation key based
        decltype(auto) obfs_key_tmp = derive_header_obfs_key(header_cipher_state);

        for (const auto &unit : pckt->units) {
            auto test_header = unit.header;

            // Deletes header data obfuscation, without shared value
            std::transform(
                reinterpret_cast<noheap::rbyte *>(&test_header),
                reinterpret_cast<noheap::rbyte *>(&test_header) + sizeof(test_header),
                obfs_key_tmp.data(), reinterpret_cast<noheap::rbyte *>(&test_header),
                std::bit_xor{});

            if (test_header.connection_id
                    != session_info.handshake_context.get_handshake_id()
                || test_header.number != possible_unit_number)
                continue;

            ++count_suitable_units;
            break;
        }

        if (count_suitable_units == pckt->units.size())
            break;
    }

    // If at least one of them is decrypted
    if (count_suitable_units > 0) {
        different_receiver_unit_number = ((possible_unit_number - batch_units_number + 1)
                                          - session_info.receiver_unit_number);
		// Restores nonce of header_cioher_state
        header_cipher_state.set_encrypt_counter_block(
            header_cipher_state.get_encrypt_counter_block() - batch_units_number);
        return true;
    }

    header_cipher_state.set_encrypt_counter_block(
        header_cipher_state.get_encrypt_counter_block()
        - (possible_unit_number - session_info.receiver_unit_number));

    if (session_info.remote_endpoint.address == pckt.get_endpoint().address
        && session_info.handshake_number && !session_info.batch_received_number) {
        different_receiver_unit_number = -1;
        return true;
    }
    return false;
}
essu::noise_handshake_context::buffer_current_state_hash_type
    essu::protocol::get_current_state_hash(const session_info_type &session_info) {
    return session_info.handshake_context.get_current_state_hash();
}

void essu::protocol::update_handshake_status(session_info_type &session_info) noexcept {
    auto action = session_info.handshake_context.get_action();
    if (action == noise::noise_action::NONE
        && session_info.handshake_context.is_complete())
        session_info.handshake_status =
            session_info_type::handshake_status_enum::COMPLETE;
    else if (action == noise::noise_action::SPLIT)
        session_info.handshake_status = session_info_type::handshake_status_enum::STOP;
    else if (action == noise::noise_action::WRITE_MESSAGE
             || action == noise::noise_action::READ_MESSAGE)
        session_info.handshake_status =
            session_info_type::handshake_status_enum::EXCHANGE;

    // If retry packet exchange is reproduced after handshake
    if (session_info.handshake_status
            == session_info_type::handshake_status_enum::COMPLETE
        && session_info.was_sent_retry && session_info.was_received_retry)
        session_info.handshake_status = session_info_type::handshake_status_enum::START;

    // If during handshake batch sent number has been reached to max
    if (session_info.handshake_status
            == session_info_type::handshake_status_enum::EXCHANGE
        && (session_info.batch_sent_number == max_batch_handshake_number
            || session_info.batch_received_number == max_batch_handshake_number))
        session_info.handshake_status = session_info_type::handshake_status_enum::START;
}
void essu::protocol::check_packet_compliance(const session_info_type &session_info,
                                             const packet_type       &pckt) {
    if (session_info.handshake_status
        == session_info_type::handshake_status_enum::COMPLETE) {
        if (!is_posthandshake_packet(pckt))
            throw protocol_error("Invalid packet setting after handshake.");
    } else if (!is_handshake_packet(pckt) && !is_dummy_packet(pckt))
        throw protocol_error("Invalid packet setting for handshake.");
}
noise::buffer_type<essu::header_data_size> essu::protocol::derive_header_obfs_key(
    typename noise_context_type::cipher_state &header_cipher_state) {
    noise::buffer_type<sizeof(typename essu::unit_type::header_data_type)
                       + noise_config.mac_size>
        obfs_key_tmp{};
    header_cipher_state.encrypt_buffer.set(obfs_key_tmp,
                                           obfs_key_tmp.size() - noise_config.mac_size);
    header_cipher_state.encrypt({});

    return noheap::to_buffer<decltype(derive_header_obfs_key(header_cipher_state))>(
        obfs_key_tmp);
}

#endif
