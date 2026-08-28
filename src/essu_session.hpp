#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include "essu_protocol.hpp"

namespace essu {
using namespace boost;

struct session_info_type_extended : private session_info_type {
    friend class session_handler;

public:
    session_info_type_extended(
        network::native_endpoint _remote_endpoint, noise::noise_role _role,
        noise::buffer_prologue_extention_type      _ext,
        const noise_context_type::buffer_key_type &_remote_public_key,
        const noise::buffer_pre_shared_key_type   &_pre_shared_key,
        const noise_context_type::keypair_type    &_local_keypair)
        : session_info_type(_remote_endpoint, _role, _ext, _remote_public_key,
                            _pre_shared_key, _local_keypair),
          last_received_ms(get_now_ms()) {}
    session_info_type_extended(session_info_type_extended &&) = default;

public:
    decltype(auto) get_remote_endpoint_address() const {
        decltype(auto) remote_endpoint = this->get_remote_endpoint();
        return network::utils::bytes_address_to_string(remote_endpoint.address,
                                                       remote_endpoint.v);
    }

private:
    std::size_t last_received_ms;
};

class session_handler : public network::action<packet_type>, noncopyable {
    static constexpr std::size_t buffer_packets_size = 128;

private:
    using buffer_session_s_type =
        noheap::monotonic_placement_new_array<session_info_type_extended,
                                              max_session_number>;

public:
    struct packet_type_extended : public packet_type {
        buffer_session_s_type::const_iterator session_info_it{};
    };

private:
    using send_buffer_packet_s_type =
        noheap::monotonic_array<packet_type_extended, buffer_packets_size>;
    using receive_buffer_packet_s_type =
        noheap::monotonic_array<packet_type, buffer_packets_size>;

public:
    struct config_type {
        noise::buffer_prologue_extention_type ext;
        noise_context_type::keypair_type      keypair;
    };
    struct endpoint_config_type {
        network::native_endpoint            endpoint;
        noise::noise_role                   local_role;
        noise_context_type::buffer_key_type public_key;
        noise::buffer_pre_shared_key_type   pre_shared_key;
    };

    using buffer_endpoint_config_s_type = noheap::monotonic_placement_new_array<
        std::reference_wrapper<const endpoint_config_type>, max_session_number>;

public:
    inline session_handler(const log_handler &handler, config_type &_config)
        : log(handler.create_proxy({"ESSU_HANDLER"})), config(_config) {}

public:
    inline void add_endpoint_config(const endpoint_config_type &endpoint_config);
    inline void register_new_session(const endpoint_config_type &endpoint_config);
    inline void
        register_incoming_session(buffer_session_s_type::const_iterator session_info_it);
    inline void
        delete_registered_session(buffer_session_s_type::const_iterator session_info_it);
    inline void
        delete_incoming_session(buffer_session_s_type::const_iterator session_info_it);
    inline const buffer_session_s_type &get_registered_session_s() const;
    inline const buffer_session_s_type &get_incoming_session_s() const;

    inline void                 push_packet(packet_type_extended &&pckt);
    inline packet_type_extended pop_packet();

    inline bool                  is_valid() const;
    inline bool                  exist_received_packets() const;
    inline noheap::runtime_error get_error() const;

public:
    inline void init_packet(packet_type &pckt);
    inline void handle_packet(packet_type &&pckt);
    inline void set_error(const noheap::runtime_error &_excp);

private:
    inline void add_endpoint_config_(const endpoint_config_type &endpoint_config);
    inline void register_new_session_(const endpoint_config_type &endpoint_config);
    inline void
        register_incoming_session_(buffer_session_s_type::const_iterator session_info_it);
    inline void
        delete_registered_session_(buffer_session_s_type::const_iterator session_info_it);
    inline void
        delete_incoming_session_(buffer_session_s_type::const_iterator session_info_it);

    inline void
        register_new_incoming_session(const endpoint_config_type &endpoint_config);
    inline void handle_session(session_info_type_extended &session_info);

private:
    log_handler::proxy log;

private:
    mutable std::shared_mutex   m_run;
    std::condition_variable_any cv_io;

    config_type                  &config;
    buffer_endpoint_config_s_type buffer_endpoint_config_s;

    buffer_session_s_type        registered_session_s;
    buffer_session_s_type        incoming_session_s;
    send_buffer_packet_s_type    send_buffer;
    receive_buffer_packet_s_type receive_buffer;
    noheap::runtime_error        excp;
};

} // namespace essu

// Adds the passed endpoint config to internal buffer of endpoint configs
void essu::session_handler::add_endpoint_config(
    const endpoint_config_type &endpoint_config) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    add_endpoint_config_(endpoint_config);
}
// Adds a new session based on endpoint_config to internal buffer of registered sessions
void essu::session_handler::register_new_session(
    const endpoint_config_type &endpoint_config) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    register_new_session_(endpoint_config);
}
// Adds a incoming session from incoming session buffer to registered session buffer
void essu::session_handler::register_incoming_session(
    buffer_session_s_type::const_iterator session_info_it) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    register_incoming_session_(session_info_it);
    delete_incoming_session_(session_info_it);
}
// Delete the session from registered session buffer
void essu::session_handler::delete_registered_session(
    buffer_session_s_type::const_iterator session_info_it) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    delete_registered_session_(session_info_it);
}
// Delete the session from incoming session buffer
void essu::session_handler::delete_incoming_session(
    buffer_session_s_type::const_iterator session_info_it) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    delete_incoming_session_(session_info_it);
}
// Returns a size of registered session buffer
const essu::session_handler::buffer_session_s_type &
    essu::session_handler::get_registered_session_s() const {
    return registered_session_s;
}
// Returns a size of incoming session buffer
const essu::session_handler::buffer_session_s_type &
    essu::session_handler::get_incoming_session_s() const {
    return incoming_session_s;
}
// Returns a last error
noheap::runtime_error essu::session_handler::get_error() const {
    std::shared_lock<decltype(m_run)> m_run_lock(m_run);
    return excp;
}
// Pushes packet to internal send buffer of packets
void essu::session_handler::push_packet(packet_type_extended &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    if (pckt.session_info_it == registered_session_s.end())
        log.abort("Invalid session info.");
    decltype(auto) session_info =
        const_cast<session_info_type_extended &>(*pckt.session_info_it);

    // Prepares the packet
    try {
        handle_session(session_info);
        protocol::prepare(session_info, pckt);
    } catch (const base_error &excp) {
        log.error("{}-{}: {}", session_info.get_remote_endpoint_address(),
                  session_info.get_constructed_random_uint16(), excp.what());
        delete_registered_session_(pckt.session_info_it);
        return;
    }
    send_buffer.push_back(pckt);

    // Sets current session for possible work
    cv_io.notify_one();
}
// Returns packet from internal receive buffer of packets
essu::session_handler::packet_type_extended essu::session_handler::pop_packet() {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    packet_type    pckt            = receive_buffer.pop_front();
    decltype(auto) session_info_it = registered_session_s.begin();
    decltype(auto) remote_endpoint = pckt.get_endpoint();

    try {
        // Finds suitable session by packet
        for (; session_info_it < registered_session_s.end(); ++session_info_it) {
            handle_session(*session_info_it);
            if (protocol::try_handle(*session_info_it, pckt)) {
                break;
            }
        }
    } catch (const base_error &excp) {
        log.error("{}-{}: {}", session_info_it->get_remote_endpoint_address(),
                  session_info_it->get_constructed_random_uint16(), excp.what());
        delete_registered_session_(session_info_it);
        return {{}, registered_session_s.end()};
    }

    // Registers new session if session is not found
    if (session_info_it == registered_session_s.end()) {
        auto config_it = std::find_if(
            buffer_endpoint_config_s.begin(), buffer_endpoint_config_s.end(),
            [&remote_endpoint](const auto &config) {
                return config.get().endpoint.address == remote_endpoint.address;
            });

        if (config_it != buffer_endpoint_config_s.end()) {
            if (std::find_if(incoming_session_s.begin(), incoming_session_s.end(),
                             [&remote_endpoint](const auto &session_it) {
                                 return session_it.get_remote_endpoint().address
                                        == remote_endpoint.address;
                             })
                != incoming_session_s.end())
                throw session_error("Incoming session didn't handle before.");
            register_new_incoming_session(*config_it);
        } else
            return {{}, registered_session_s.end()};

        session_info_it = incoming_session_s.end() - 1;
        handle_session(*session_info_it);
        if (!protocol::try_handle(*session_info_it, pckt)) {
            delete_incoming_session_(session_info_it);
            return {{}, registered_session_s.end()};
        }
    }

    session_info_it->last_received_ms = get_now_ms();
    cv_io.notify_one();

    return {pckt, session_info_it};
}
bool essu::session_handler::exist_received_packets() const {
    std::shared_lock<decltype(m_run)> m_run_lock(m_run);
    return receive_buffer.size();
}
bool essu::session_handler::is_valid() const {
    std::shared_lock<decltype(m_run)> m_run_lock(m_run);
    return !excp.is_set();
}

void essu::session_handler::init_packet(packet_type &pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    // Waits run-thread
    cv_io.wait(m_run_lock, [this] { return excp.is_set() || this->send_buffer.size(); });
    if (excp.is_set())
        return;

    pckt = send_buffer.pop_front();
}
void essu::session_handler::handle_packet(packet_type &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    cv_io.wait(m_run_lock, [this] {
        return excp.is_set()
               || this->receive_buffer.size() < this->receive_buffer.buffer_size();
    });
    if (excp.is_set())
        return;

    receive_buffer.push_back(pckt);
}
void essu::session_handler::set_error(const noheap::runtime_error &_excp) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    excp = _excp;
    cv_io.notify_all();
}

void essu::session_handler::add_endpoint_config_(
    const endpoint_config_type &endpoint_config) {
    if (buffer_endpoint_config_s.size() == max_session_number)
        log.throw_and_log<noheap::runtime_error>({}, "Failed to add config of endpoint.");

    // Saves passed config of endpoint
    buffer_endpoint_config_s.push_back(
        std::reference_wrapper<const endpoint_config_type>(endpoint_config));
}
void essu::session_handler::register_new_session_(
    const endpoint_config_type &endpoint_config) {
    if (registered_session_s.size() == max_session_number)
        log.throw_and_log<noheap::runtime_error>({}, "Failed to add session.");

    registered_session_s.emplace_back(
        endpoint_config.endpoint, endpoint_config.local_role, config.ext,
        endpoint_config.public_key, endpoint_config.pre_shared_key, config.keypair);

    decltype(auto) session_info = registered_session_s[registered_session_s.size() - 1];
    log.info("{}-{}: {}", session_info.get_remote_endpoint_address(),
             session_info.get_constructed_random_uint16(), "New session registered.");
}
void essu::session_handler::register_new_incoming_session(
    const endpoint_config_type &endpoint_config) {
    if (incoming_session_s.size() == max_session_number)
        log.throw_and_log<noheap::runtime_error>({}, "Failed to add session.");

    incoming_session_s.emplace_back(endpoint_config.endpoint, endpoint_config.local_role,
                                    config.ext, endpoint_config.public_key,
                                    endpoint_config.pre_shared_key, config.keypair);

    decltype(auto) session_info = incoming_session_s[incoming_session_s.size() - 1];
    log.info("{}-{}: {}", session_info.get_remote_endpoint_address(),
             session_info.get_constructed_random_uint16(),
             "New incoming session registered.");
}
void essu::session_handler::register_incoming_session_(
    buffer_session_s_type::const_iterator session_info_it) {
    if (incoming_session_s.size() == max_session_number)
        log.throw_and_log<noheap::runtime_error>({}, "Failed to add session.");

    // Copies new session
    if (session_info_it == incoming_session_s.end())
        log.throw_and_log<noheap::runtime_error>({},
                                                 "Invalid the index of session info.");

    registered_session_s.push_back(
        std::move(const_cast<session_info_type_extended &>(*session_info_it)));

    decltype(auto) session_info = registered_session_s[registered_session_s.size() - 1];
    log.info("{}-{}: {}", session_info.get_remote_endpoint_address(),
             session_info.get_constructed_random_uint16(),
             "Incoming session registered.");
}
void essu::session_handler::delete_registered_session_(
    buffer_session_s_type::const_iterator session_info_it) {
    if (session_info_it == registered_session_s.end())
        log.throw_and_log<noheap::runtime_error>({},
                                                 "Invalid the index of session info.");

    decltype(auto) session_info = *session_info_it;
    log.info("{}-{}: {}", session_info.get_remote_endpoint_address(),
             session_info.get_constructed_random_uint16(), "Session removed.");

    registered_session_s.erase(session_info_it);
}
void essu::session_handler::delete_incoming_session_(
    buffer_session_s_type::const_iterator session_info_it) {
    if (session_info_it == incoming_session_s.end())
        log.throw_and_log<noheap::runtime_error>({},
                                                 "Invalid the index of session info.");

    decltype(auto) session_info = *session_info_it;
    log.info("{}-{}: {}", session_info.get_remote_endpoint_address(),
             session_info.get_constructed_random_uint16(), "Incoming session removed.");

    incoming_session_s.erase(session_info_it);
}
void essu::session_handler::handle_session(session_info_type_extended &session_info) {
    scope_guard    sc([this] { this->cv_io.notify_all(); });
    std::size_t    now_ms         = get_now_ms();
    decltype(auto) session_status = protocol::get_handshake_status(session_info);

    // Tries to control session and handles possible errors
    if (now_ms - session_info.last_received_ms >= timeout_ms)
        throw session_error("Timeout has been reached.");

    // If status of session is START or STOP:
    if (session_status == session_info_type::handshake_status_enum::START) {
        protocol::start_handshake(session_info);
        log.info("{}-{}: {} Performing handshake...",
                 session_info.get_remote_endpoint_address(),
                 session_info.get_constructed_random_uint16(),
                 std::string_view(
                     noheap::hex_encode(protocol::get_current_state_hash(session_info))));
    } else if (session_status == session_info_type::handshake_status_enum::STOP) {
        protocol::stop_handshake(session_info);
        log.info("{}-{}: {} Handshake completed.",
                 session_info.get_remote_endpoint_address(),
                 session_info.get_constructed_random_uint16(),
                 std::string_view(
                     noheap::hex_encode(protocol::get_current_state_hash(session_info))));
    }
}

#endif
