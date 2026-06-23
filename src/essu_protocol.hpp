#ifndef ESSU_HPP
#define ESSU_HPP

#include "essu_base.hpp"
#include "essu_noise_handshake_context.hpp"

namespace essu {

// Session info corresponding to an individual node
struct session_info_type {
    friend class protocol;

    enum class status_enum : std::size_t {
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
        : remote_endpoint(_remote_endpoint), log(log_handler),
          handshake_context(_role, _ext, _remote_public_key, _pre_shared_key,
                            _local_keypair) {
        reset_state();
        update_dynamic_owner();
    }
    session_info_type(const session_info_type &) = delete;
    session_info_type(session_info_type &&)      = delete;

public:
    decltype(auto) get_remote_endpoint() const noexcept { return remote_endpoint; }
    decltype(auto) get_log() const noexcept { return log; }

private:
    void reset_state() noexcept {
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
    void update_dynamic_owner() {
        log.set_dynamic_owner(
            noheap::to_new_buffer<noheap::log_impl::owner_impl::buffer_type>(
                network::utils::bytes_address_to_string(remote_endpoint.address,
                                                        remote_endpoint.v)));
    }

private:
    network::native_endpoint remote_endpoint;

private:
    log_proxy log;

private:
    noise_handshake_context handshake_context;
    std::uint16_t           handshake_number{0};

    status_enum   status;
    std::uint32_t batch_sent_number;
    std::uint32_t batch_received_number;
    std::uint32_t sender_unit_number;
    std::uint32_t receiver_unit_number;
    std::uint32_t sender_key_iteration_number;
    std::uint32_t receiver_key_iteration_number;
    std::uint32_t undecrypted_batch_number;
    bool          was_sent_retry;
    bool          was_received_retry;

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION");
    static constexpr log_handler log_handler{buffer_owner};
};

class protocol final {
    protocol() = delete;

public:
    static inline void prepare(session_info_type &session_info, packet_type &pckt);
    static inline void handle(session_info_type &session_info, packet_type &pckt);
    static inline void start_handshake(session_info_type &session_info);
    static inline void stop_handshake(session_info_type &session_info);
    static inline session_info_type::status_enum
        update_status(session_info_type &session_info);

    static inline bool check_affiliation_packet(session_info_type &session_info,
                                                const packet_type &pckt);

    static inline std::uint64_t
                       get_handshake_number(const session_info_type &session_info);
    static inline bool can_send_packet(const session_info_type &session_info);
    static inline bool can_receive_packet(const session_info_type &session_info);

private:
    static inline void check_packet_compliance(const session_info_type &session_info,
                                               const packet_type       &pckt);
    static inline void check_protocol_compliance(const session_info_type &session_info,
                                                 bool                     receive);
    static inline noise::buffer_type<header_data_size> derive_header_obfs_key(
        typename noise_context_type::cipher_state &header_cipher_state);
};

} // namespace essu

void essu::protocol::prepare(session_info_type &session_info, packet_type &pckt) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_sender();
    decltype(auto) random_state     = session_info.handshake_context.get_random_state();
    bool session_handshake_complete = session_info.handshake_context.is_complete();
    unit_type::unit_type_enum control_unit_type = unit_type::unit_type_enum::dummy;

    check_protocol_compliance(session_info, false);
    check_packet_compliance(session_info, pckt);

    try {
        // Inits packet like handshake message if necessary
        if (session_info.handshake_context.get_action()
                == noise::noise_action::WRITE_MESSAGE
            && std::uniform_int_distribution(0, 1)(random_state.generator))
            session_info.handshake_context.init_packet(pckt);

        // If retry or available batch number is reached
        if (session_info.was_received_retry
            || (session_handshake_complete
                && session_info.batch_sent_number
                       == session_info.handshake_context.get_available_batch_number()
                              - 1)) {
            set_control_session_packet(pckt, unit_type::unit_type_enum::retry);
            session_info.was_sent_retry = true;
        }
        control_unit_type = get_control_unit(pckt).header.type;

        // Forces control and last unit to be dummy
        if (!is_control_session_unit_type(control_unit_type))
            get_control_unit(pckt).header.type = unit_type::unit_type_enum::dummy;

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
                        session_info.log.throw_exception<noheap::runtime_error>(
                            "Undefined packet type: {}.",
                            static_cast<std::size_t>(unit.header.type));
                }

                // Adds random padding after payload data
                random_state.padding_buffer.set(noheap::make_span(unit.buffer),
                                                payload_size);
                random_state.pad();
            }

            // Encrypts buffer data and authenticates based on the header
            if (session_handshake_complete) {
                payload_cipher_state.encrypt_buffer.set(noheap::make_span(unit.buffer),
                                                        unit.buffer_size_without_mac());
                payload_cipher_state.encrypt(
                    {reinterpret_cast<noheap::rbyte *>(&unit.header),
                     sizeof(unit.header)});

                // Performs rekey for encryption
                if (unit.header.number % unit_per_rekey_number == 0) {
                    payload_cipher_state.rekey_encrypt();
                    ++session_info.sender_key_iteration_number;
                }
            }

            // Adds header data obfuscation
            std::transform(
                reinterpret_cast<noheap::rbyte *>(&unit.header),
                reinterpret_cast<noheap::rbyte *>(&unit.header) + sizeof(unit.header),
                derive_header_obfs_key(header_cipher_state).data(),
                reinterpret_cast<noheap::rbyte *>(&unit.header), std::bit_xor{});
        }
    } catch (const protocol_error &excp) {
        session_info.log.throw_exception<protocol_error>(
            "Failed to prepare the passed batch: {}", excp.what());
    }

    // Shuffles units in batch
    std::shuffle(pckt->units.begin(), pckt->units.end(), random_state.generator);
    ++session_info.batch_sent_number;

    if (is_control_session_unit_type(control_unit_type))
        session_info.log.to_all("{} -> [{}] packet is prepared.",
                                session_handshake_complete ? "Posthandshake"
                                                           : "Handshake",
                                utils::get_string_unit_type(control_unit_type));
}

void essu::protocol::handle(session_info_type &session_info, packet_type &pckt) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_receiver();
    bool session_handshake_complete = session_info.handshake_context.is_complete();
    unit_type::unit_type_enum control_unit_type = unit_type::unit_type_enum::dummy;

    check_protocol_compliance(session_info, true);

    try {
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

                    payload_cipher_state.decrypt_buffer.set(
                        noheap::make_span(unit.buffer), unit.buffer.size());
                    try {
                        payload_cipher_state.decrypt(
                            {reinterpret_cast<noheap::rbyte *>(&unit.header),
                             sizeof(unit.header)});
                    } catch (const noheap::runtime_error &excp) {
                        session_info.log.throw_exception<protocol_error>("Invalid MAC.");
                    }
                }

                ++decrypted_units_number;
                break;
            }
        }

        if (decrypted_units_number != batch_units_number)
            session_info.log.throw_exception<protocol_error>("Invalid header of units.");
    } catch (const protocol_error &excp) {
        session_info.log.throw_exception<protocol_error>(
            "Failed to decrypt last batch: {}", excp.what());
    }

    // Restores order of units in batch
    std::sort(pckt->units.begin(), pckt->units.end(),
              [](const auto &el_left, const auto &el_right) {
                  return el_left.header.number < el_right.header.number;
              });

    check_packet_compliance(session_info, pckt);
    ++session_info.batch_received_number;

    // Updates session's remote endpoint
    {
        decltype(auto) packet_endpoint = pckt.get_endpoint();
        bool endpoints_are_different = (packet_endpoint == session_info.remote_endpoint);
        if (endpoints_are_different)
            session_info.remote_endpoint = pckt.get_endpoint();
        session_info.update_dynamic_owner();
    }
    control_unit_type = get_control_unit(pckt).header.type;

    // If handshake is completed and the packet has a control unit retry
    if (session_handshake_complete
        && control_unit_type == unit_type::unit_type_enum::retry)
        session_info.was_received_retry = true;

    // Handles the packet like handshake message if necessary
    if (session_info.handshake_context.get_action() == noise::noise_action::READ_MESSAGE
        && is_control_session_unit_type(control_unit_type))
        session_info.handshake_context.handle_packet(std::move(pckt));

    if (is_control_session_unit_type(control_unit_type))
        session_info.log.to_all("{} -> [{}] packet is handled.",
                                session_handshake_complete ? "Posthandshake"
                                                           : "Handshake",
                                utils::get_string_unit_type(control_unit_type));
}
void essu::protocol::start_handshake(session_info_type &session_info) {
    session_info.reset_state();
    session_info.handshake_context.start();
}
void essu::protocol::stop_handshake(session_info_type &session_info) {
    session_info.handshake_context.stop();
    session_info.reset_state();

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
bool essu::protocol::check_affiliation_packet(session_info_type &session_info,
                                              const packet_type &pckt) {
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_receiver();
    bool session_handshake_complete = session_info.handshake_context.is_complete();

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

    if (count_suitable_units > 0) {
        std::uint64_t min_unit_number = ((possible_unit_number - batch_units_number + 1)
                                         - session_info.receiver_unit_number);
        if (session_handshake_complete)
            payload_cipher_state.set_decrypt_counter_block(
                payload_cipher_state.get_decrypt_counter_block() + min_unit_number);
        header_cipher_state.set_encrypt_counter_block(
            header_cipher_state.get_encrypt_counter_block() - batch_units_number);
        session_info.receiver_unit_number += min_unit_number;
        return true;
    }

    header_cipher_state.set_encrypt_counter_block(
        header_cipher_state.get_encrypt_counter_block()
        - (possible_unit_number - session_info.receiver_unit_number));
    return false;
}

std::uint64_t
    essu::protocol::get_handshake_number(const session_info_type &session_info) {
    return session_info.handshake_number;
}
bool essu::protocol::can_send_packet(const session_info_type &session_info) {
    return session_info.handshake_context.is_complete()
               ? !session_info.was_sent_retry
               : session_info.batch_received_number
                     || session_info.handshake_context.get_role()
                            == noise::noise_role::INITIATOR;
}
bool essu::protocol::can_receive_packet(const session_info_type &session_info) {
    return session_info.handshake_context.is_complete()
               ? !session_info.was_received_retry
               : session_info.handshake_context.get_role()
                     == noise::noise_role::RESPONDER;
}

void essu::protocol::check_packet_compliance(const session_info_type &session_info,
                                             const packet_type       &pckt) {
    if (session_info.status == session_info_type::status_enum::COMPLETE) {
        if (!is_posthandshake_packet(pckt))
            session_info.log.throw_exception<protocol_error>(
                "Invalid packet setting after handshake.");
    } else if (!is_handshake_packet(pckt) && !is_dummy_packet(pckt))
        session_info.log.throw_exception<protocol_error>(
            "Invalid packet setting for handshake.");
}
void essu::protocol::check_protocol_compliance(const session_info_type &session_info,
                                               bool                     receive) {
    if (session_info.handshake_number == max_available_handshake_number)
        session_info.log.throw_exception<protocol_error>(
            "Limit of handshakes has been reached.");
    if (session_info.status == session_info_type::status_enum::COMPLETE
        && (receive
                ? (session_info.was_received_retry
                   || session_info.batch_received_number
                          == session_info.handshake_context.get_available_batch_number())
                : (session_info.was_sent_retry
                   || session_info.batch_sent_number
                          == session_info.handshake_context
                                 .get_available_batch_number())))
        session_info.log.throw_exception<protocol_error>("Expected to rehandshake.");
}
noise::buffer_type<essu::header_data_size> essu::protocol::derive_header_obfs_key(
    typename noise_context_type::cipher_state &header_cipher_state) {
    noise::buffer_type<sizeof(typename essu::unit_type::header_data_type)
                       + noise_config.mac_size>
        obfs_key_tmp{};
    header_cipher_state.encrypt_buffer.set(noheap::make_span(obfs_key_tmp),
                                           obfs_key_tmp.size() - noise_config.mac_size);
    header_cipher_state.encrypt({});

    return noheap::to_buffer<decltype(derive_header_obfs_key(header_cipher_state))>(
        obfs_key_tmp);
}

#endif
