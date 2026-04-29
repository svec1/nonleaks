#ifndef ESSU_HPP
#define ESSU_HPP

#include "essu_base.hpp"
#include "essu_noise_handshake_context.hpp"

namespace essu {

// Session info corresponding to an individual node
struct session_info_type {
    friend struct protocol_type;

public:
    session_info_type(network::buffer_address_type _addr) : addr(_addr) { reset_state(); }

private:
    void reset_state() {
        id                            = 0;
        batches_sent_number           = 0;
        batches_received_number       = 0;
        sender_units_number           = 0;
        receiver_units_number         = 0;
        sender_key_iteration_number   = 0;
        receiver_key_iteration_number = 0;
        undecrypted_batch_number      = 0;
        was_sent_retry                = false;
        was_received_retry            = false;
    }

public:
    const network::buffer_address_type addr;
    std::uint64_t                      id;

private:
    noise_handshake_context handshake_context;
    std::uint64_t           batches_sent_number;
    std::uint64_t           batches_received_number;
    std::uint64_t           sender_units_number;
    std::uint64_t           receiver_units_number;
    std::uint64_t           sender_key_iteration_number;
    std::uint64_t           receiver_key_iteration_number;
    std::uint64_t           undecrypted_batch_number;
    bool                    was_sent_retry;
    bool                    was_received_retry;

    std::uint64_t handshake_number = 0;
};

struct protocol_type final
    : public network::protocol_native_type<packet_type, noheap::log_impl::create_owner(
                                                            "ESSU_PROTOCOL")> {
    using session_info_s_type =
        noheap::monotonic_array<session_info_type *, network::max_count_addresses>;

public:
    void prepare(packet_type &pckt, network::buffer_address_type addr,
                 protocol_type::callback_prepare_type callback) const;
    void handle(packet_type &pckt, network::buffer_address_type addr,
                protocol_type::callback_handle_type callback) const;

public:
    void register_session_info(
        session_info_type &session_info, noise::noise_role role,
        noise::prologue_extention_type          ext,
        const noise::pre_shared_key_type       &pre_shared_key,
        const noise_context_type::keypair_type &local_keypair,
        const noise_context_type::dh_key_type  &remote_public_key) const;
    void start_handshake(session_info_type &session_info) const;
    void stop_handshake(session_info_type &session_info) const;

    noise::noise_role   get_role(const session_info_type &session_info) const;
    noise::noise_action get_handshake_action(const session_info_type &session_info) const;
    std::uint64_t       get_handshake_number(const session_info_type &session_info) const;
    bool                can_send_packet(const session_info_type &session_info) const;
    bool                can_receive_packet(const session_info_type &session_info) const;

private:
    session_info_s_type::iterator
         find_session_info(network::buffer_address_type addr) const;
    void check_sesssion_state(const session_info_type &session_info) const;
    void check_protocol_compliance(bool                      handshake_complete,
                                   unit_type::unit_type_enum batch_type,
                                   unit_type::unit_type_enum payload_unit_two_type) const;
    noise::buffer_type<header_data_size> derive_header_obfs_key(
        typename noise_context_type::cipher_state &header_cipher_state,
        std::uint64_t                              number) const;

private:
    mutable session_info_s_type session_info_s;
};

using wrapper_packet_type = network::wrapper_packet<packet_type, protocol_type>;

} // namespace essu

void essu::protocol_type::prepare(packet_type &pckt, network::buffer_address_type addr,
                                  protocol_type::callback_prepare_type callback) const {
    auto session_info_it = find_session_info(addr);
    if (session_info_it == session_info_s.end())
        this->log.throw_exception("Session info is invalid for address.");

    decltype(auto) session_info = *(*session_info_it);
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_sender();
    decltype(auto) random_state = session_info.handshake_context.get_random_state();

    bool handshake_already_complete = session_info.handshake_context.is_complete();

    check_sesssion_state(session_info);
    if (handshake_already_complete && !can_send_packet(session_info))
        this->log.throw_exception("Expected to rehandshake.");

    // Calls callback(action) to init packet
    if (handshake_already_complete)
        callback(pckt);
    else
        session_info.handshake_context.init_packet(pckt);

    if (session_info.handshake_context.get_role() == noise::noise_role::INITIATOR
        && session_info.batches_sent_number >= max_available_batches_number) {
        pckt->units[2].header.type  = unit_type::unit_type_enum::retry;
        session_info.was_sent_retry = true;
    } else if (session_info.handshake_context.get_role() == noise::noise_role::RESPONDER
               && session_info.was_received_retry) {
        pckt->units[2].header.type  = unit_type::unit_type_enum::retry_ok;
        session_info.was_sent_retry = true;
    }

    check_protocol_compliance(handshake_already_complete, pckt->units[0].header.type,
                              pckt->units[1].header.type);

    for (std::uint64_t i = 0; i < pckt->units.size(); ++i) {
        unit_type &unit = pckt->units[i];

        unit.header.number               = session_info.sender_units_number++;
        unit.header.key_iteration_number = session_info.sender_key_iteration_number;

        // Forces units to be dummy if necessary
        if (i >= 2 && unit.header.type != unit_type::unit_type_enum::retry
            && unit.header.type != unit_type::unit_type_enum::retry_ok)
            unit.header.type = unit_type::unit_type_enum::dummy;

        // Adds random padding
        {
            // Determines payload size of the unit to define size of random
            // padding
            std::uint64_t payload_size;
            switch (unit.header.type) {
                case unit_type::unit_type_enum::session_request:
                    payload_size = unit_config_type::hs1_size;
                    break;
                case unit_type::unit_type_enum::session_created:
                    payload_size = unit_config_type::hs2_size;
                    break;
                case unit_type::unit_type_enum::session_confirmed:
                    payload_size = unit_config_type::hs3_size;
                    break;
                case unit_type::unit_type_enum::hole_punch:
                    payload_size = 8;
                    break;
                case unit_type::unit_type_enum::data:
                    payload_size = payload_data_size;
                    break;
                case unit_type::unit_type_enum::dummy:
                case unit_type::unit_type_enum::retry:
                case unit_type::unit_type_enum::retry_ok:
                    payload_size = 0;
                    break;
                default:
                    this->log.throw_exception("Packet type[{}] is not allowed.",
                                              static_cast<std::size_t>(unit.header.type));
            }

            // Adds random padding after payload data
            random_state.padding_buffer.set(
                {reinterpret_cast<noheap::rbyte *>(unit.buffer.data()),
                 unit.buffer.size()},
                std::clamp<std::uint64_t>(payload_size, 0, unit.buffer.size()));
            random_state.pad();
        }

        // Encrypts buffer data and authenticates based on the header
        if (unit.header.type == unit_type::unit_type_enum::data) {
            payload_cipher_state.encrypt_buffer.set(
                {unit.buffer.data(), unit.buffer.size()}, unit.buffer_size_without_mac);
            payload_cipher_state.set_encrypt_nonce(unit.header.number);
            payload_cipher_state.encrypt(
                {reinterpret_cast<noheap::rbyte *>(&unit.header), sizeof(unit.header)});
        }

        // Generates header obfuscation key based on the unit_number
        auto obfs_key_tmp =
            derive_header_obfs_key(header_cipher_state, unit.header.number);

        // Adds header data obfuscation
        std::transform(reinterpret_cast<noheap::rbyte *>(&unit.header),
                       reinterpret_cast<noheap::rbyte *>(&unit.header)
                           + sizeof(unit.header),
                       obfs_key_tmp.data(),
                       reinterpret_cast<noheap::rbyte *>(&unit.header), std::bit_xor{});
    }

    // Shuffles units in batch
    std::random_device rd;
    std::mt19937       generator(rd());
    std::shuffle(pckt->units.begin(), pckt->units.end(), generator);

    // Performs rekey for encryption
    ++session_info.batches_sent_number;
    if (handshake_already_complete
        && session_info.batches_sent_number % batches_per_rekey_number == 0) {
        payload_cipher_state.rekey_encrypt();
        ++session_info.sender_key_iteration_number;
    }
}

void essu::protocol_type::handle(packet_type &pckt, network::buffer_address_type addr,
                                 protocol_type::callback_handle_type callback) const {
    auto session_info_it = find_session_info(addr);
    if (session_info_it == session_info_s.end())
        return;

    decltype(auto) session_info = *(*session_info_it);
    decltype(auto) payload_cipher_state =
        session_info.handshake_context.get_payload_cipher_state();
    decltype(auto) header_cipher_state =
        session_info.handshake_context.get_header_cipher_state_receiver();

    bool handshake_already_complete = session_info.handshake_context.is_complete();

    check_sesssion_state(session_info);
    if (handshake_already_complete && !can_receive_packet(session_info))
        this->log.throw_exception("Expected to rehandshake.");

    // Selects possible unit number
    std::uint64_t count_decrypted_units = 0;
    for (; session_info.receiver_units_number
           < session_info.receiver_units_number
                 + batches_window_number * batch_units_number;
         ++session_info.receiver_units_number) {
        // Generates header obfuscation key based on the possible_unit_number
        auto obfs_key_tmp = derive_header_obfs_key(header_cipher_state,
                                                   session_info.receiver_units_number);

        for (auto &unit : pckt->units) {
            unit_type test_unit = unit;

            // Deletes header data obfuscation
            std::transform(reinterpret_cast<noheap::rbyte *>(&test_unit.header),
                           reinterpret_cast<noheap::rbyte *>(&test_unit.header)
                               + sizeof(test_unit.header),
                           obfs_key_tmp.data(),
                           reinterpret_cast<noheap::rbyte *>(&test_unit.header),
                           std::bit_xor{});

            if (test_unit.header.number != session_info.receiver_units_number)
                continue;

            // Loop handling rekeys performed on the remote node.
            for (; session_info.receiver_key_iteration_number
                   < test_unit.header.key_iteration_number;
                 ++session_info.receiver_key_iteration_number)
                payload_cipher_state.rekey_decrypt();

            // Tries to decrypt buffer data
            if (test_unit.header.type == unit_type::unit_type_enum::data) {
                payload_cipher_state.decrypt_buffer.set(
                    {test_unit.buffer.data(), test_unit.buffer.size()},
                    test_unit.buffer.size());
                payload_cipher_state.set_decrypt_nonce(test_unit.header.number);
                try {
                    payload_cipher_state.decrypt(
                        {reinterpret_cast<noheap::rbyte *>(&test_unit.header),
                         sizeof(test_unit.header)});
                } catch (noheap::runtime_error &excp) {
                    continue;
                }
            }

            unit = test_unit;
            ++count_decrypted_units;
            break;
        }

        if (count_decrypted_units == pckt->units.size())
            break;
    }

    // If it was not possible to decrypt all units in batch
    if (count_decrypted_units != pckt->units.size()) {
        ++session_info.undecrypted_batch_number;

        // If performs handshake or was failed to decrypt
        // max_undecrypted_batches_number count packets after handshake
        if (!handshake_already_complete
            || session_info.undecrypted_batch_number == max_undecrypted_batches_number)
            this->log.throw_exception("Failed to decrypt last batches.");
        return;
    } else
        session_info.undecrypted_batch_number = 0;

    // Restores order of units in batch
    std::sort(pckt->units.begin(), pckt->units.end(),
              [](const auto &el_left, const auto &el_right) {
                  return el_left.header.number < el_right.header.number;
              });

    ++session_info.receiver_units_number;
    ++session_info.batches_received_number;

    if (pckt->units[2].header.type == unit_type::unit_type_enum::retry
        || pckt->units[2].header.type == unit_type::unit_type_enum::retry_ok)
        session_info.was_received_retry = true;

    // Calls callback(action) to handle packet
    if (handshake_already_complete)
        callback(std::move(pckt));
    else
        session_info.handshake_context.process_packet(std::move(pckt));
};

void essu::protocol_type::register_session_info(
    session_info_type &session_info, noise::noise_role role,
    noise::prologue_extention_type ext, const noise::pre_shared_key_type &pre_shared_key,
    const noise_context_type::keypair_type &local_keypair,
    const noise_context_type::dh_key_type  &remote_public_key) const {
    if (find_session_info(session_info.addr) != session_info_s.end())
        this->log.throw_exception("Session already exist.");
    if (session_info_s.size() == network::max_count_addresses)
        this->log.throw_exception("Sessions limit has been reached.");

    const_cast<session_info_s_type &>(session_info_s).push_back(&session_info);

    session_info_s[session_info_s.size() - 1]->handshake_context =
        noise_handshake_context{role, ext, pre_shared_key, local_keypair,
                                remote_public_key};
}
void essu::protocol_type::start_handshake(session_info_type &session_info) const {
    session_info.reset_state();
    session_info.handshake_context.start();
}
void essu::protocol_type::stop_handshake(session_info_type &session_info) const {
    session_info.handshake_context.stop();

    auto unique_value = session_info.handshake_context.get_unique_value();
    session_info.id   = noheap::represent_bytes<std::uint64_t>(
        noheap::clip_buffer<sizeof(std::uint64_t), 0>(unique_value));

    const std::uint32_t value1 = noheap::represent_bytes<std::uint32_t>(
        noheap::clip_buffer<sizeof(std::uint32_t), sizeof(std::uint64_t)>(unique_value));
    const std::uint32_t value2 = noheap::represent_bytes<std::uint32_t>(
        noheap::clip_buffer<sizeof(std::uint32_t),
                            sizeof(std::uint64_t) + sizeof(std::uint32_t)>(unique_value));

    if (session_info.handshake_context.get_role() == noise::noise_role::INITIATOR) {
        session_info.sender_units_number   = value1;
        session_info.receiver_units_number = value2;
    } else {
        session_info.sender_units_number   = value2;
        session_info.receiver_units_number = value1;
    }

    ++session_info.handshake_number;
}
noise::noise_role
    essu::protocol_type::get_role(const session_info_type &session_info) const {
    return session_info.handshake_context.get_role();
}
noise::noise_action essu::protocol_type::get_handshake_action(
    const session_info_type &session_info) const {
    return session_info.handshake_context.get_action();
}
std::uint64_t essu::protocol_type::get_handshake_number(
    const session_info_type &session_info) const {
    return session_info.handshake_number;
}
bool essu::protocol_type::can_send_packet(const session_info_type &session_info) const {
    return !session_info.was_sent_retry;
}
bool essu::protocol_type::can_receive_packet(
    const session_info_type &session_info) const {
    return !session_info.was_received_retry;
}

essu::protocol_type::session_info_s_type::iterator
    essu::protocol_type::find_session_info(network::buffer_address_type addr) const {
    return std::find_if(session_info_s.begin(), session_info_s.end(),
                        [&](auto el) { return el->addr == addr; });
}
void essu::protocol_type::check_sesssion_state(
    const session_info_type &session_info) const {
    if (session_info.handshake_number > max_available_handshakes_number)
        this->log.throw_exception("Limit of handshakes has been reached.");
}
void essu::protocol_type::check_protocol_compliance(
    bool handshake_complete, unit_type::unit_type_enum batch_type,
    unit_type::unit_type_enum payload_unit_two_type) const {
    if (handshake_complete
        && (batch_type != unit_type::unit_type_enum::data
            || (payload_unit_two_type != unit_type::unit_type_enum::data
                && payload_unit_two_type != unit_type::unit_type_enum::dummy)))
        this->log.throw_exception("Expected payload unit.");
}
noise::buffer_type<essu::header_data_size> essu::protocol_type::derive_header_obfs_key(
    typename noise_context_type::cipher_state &header_cipher_state,
    std::uint64_t                              number) const {
    noise::buffer_type<sizeof(typename essu::unit_type::header_data_type)
                       + noise_context_type::mac_size>
        obfs_key_tmp{};
    header_cipher_state.encrypt_buffer.set({obfs_key_tmp.data(), obfs_key_tmp.size()},
                                           obfs_key_tmp.size()
                                               - noise_context_type::mac_size);

    header_cipher_state.set_encrypt_nonce(number);
    header_cipher_state.encrypt({});

    return noheap::to_buffer<decltype(derive_header_obfs_key(header_cipher_state,
                                                             number))>(obfs_key_tmp);
}

#endif
