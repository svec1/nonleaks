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

private:
    std::size_t last_received_ms;
};

class session_handler : public network::action<packet_type>, noncopyable {
    static constexpr std::size_t buffer_packets_size = 128;

public:
    struct packet_type_extended : public packet_type {
        std::size_t session_info_it;
    };

private:
    using buffer_session_s_type =
        noheap::monotonic_placement_new_array<session_info_type_extended,
                                              max_session_number>;
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
    inline session_handler(config_type &_config) : config(_config) {}

public:
    inline void        add_endpoint_config(const endpoint_config_type &endpoint_config);
    inline void        register_new_session(const endpoint_config_type &endpoint_config);
    inline void        register_incoming_session(std::size_t session_info_it);
    inline void        delete_registered_session(std::size_t session_info_it);
    inline void        delete_incoming_session(std::size_t session_info_it);
    inline std::size_t get_registered_session_s_size() const;
    inline std::size_t get_incoming_session_s_size() const;

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
    inline void add_endpoint_config_internal(const endpoint_config_type &endpoint_config);
    inline void
                register_new_session_internal(buffer_session_s_type      &session_s,
                                              const endpoint_config_type &endpoint_config);
    inline void register_incoming_session_internal(std::size_t session_info_it);
    inline void delete_registered_session_internal(std::size_t session_info_it);
    inline void delete_incoming_session_internal(std::size_t session_info_it);
    inline void handle_session(session_info_type_extended &session_info);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION_HANDLER");
    static constexpr log_handler log{buffer_owner};

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
    add_endpoint_config_internal(endpoint_config);
}
// Adds a new session based on endpoint_config to internal buffer of registered sessions
void essu::session_handler::register_new_session(
    const endpoint_config_type &endpoint_config) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    register_new_session_internal(registered_session_s, endpoint_config);
}
// Adds a incoming session from incoming session buffer to registered session buffer
void essu::session_handler::register_incoming_session(std::size_t session_info_it) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    register_incoming_session_internal(session_info_it);
}
// Delete the session from registered session buffer
void essu::session_handler::delete_registered_session(std::size_t session_info_it) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    delete_registered_session_internal(session_info_it);
}
// Delete the session from incoming session buffer
void essu::session_handler::delete_incoming_session(std::size_t session_info_it) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    delete_incoming_session_internal(session_info_it);
}
// Returns a size of registered session buffer
std::size_t essu::session_handler::get_registered_session_s_size() const {
    std::shared_lock<decltype(m_run)> m_run_lock(m_run);
    return registered_session_s.size();
}
// Returns a size of incoming session buffer
std::size_t essu::session_handler::get_incoming_session_s_size() const {
    std::shared_lock<decltype(m_run)> m_run_lock(m_run);
    return incoming_session_s.size();
}
// Returns a last error
noheap::runtime_error essu::session_handler::get_error() const {
    std::shared_lock<decltype(m_run)> m_run_lock(m_run);
    return excp;
}
// Pushes packet to internal send buffer of packets
void essu::session_handler::push_packet(packet_type_extended &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    if (pckt.session_info_it >= registered_session_s.size())
        log.throw_exception<noheap::runtime_error>("Invalid the index of session info.");
    decltype(auto) session_info = registered_session_s[pckt.session_info_it];

    // Prepares the packet
    try {
        handle_session(session_info);
        protocol::prepare(session_info, pckt);
    } catch (const base_error &) {
        delete_registered_session_internal(pckt.session_info_it);
        throw;
    }
    send_buffer.push_back(pckt);

    // Sets current session for possible work
    cv_io.notify_one();
}
// Returns packet from internal receive buffer of packets
essu::session_handler::packet_type_extended essu::session_handler::pop_packet() {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    packet_type    pckt            = receive_buffer.pop_front();
    decltype(auto) session_info_it = registered_session_s.end();
    decltype(auto) remote_endpoint = pckt.get_endpoint();

    // Finds suitable session by packet
    for (decltype(auto) it = registered_session_s.begin();
         it < registered_session_s.end(); ++it) {
        try {
            handle_session(*it);
            if (protocol::try_handle(*it, pckt)) {
				session_info_it = it;
                break;
            }
        } catch (const base_error &) {
            delete_registered_session_internal(std::size_t(
                std::distance(registered_session_s.begin(), session_info_it)));
            throw;
        }
    }

    // Registers new session if session is not found
    if (session_info_it == registered_session_s.end()) {
        buffer_endpoint_config_s_type::iterator config_it = std::find_if(
            buffer_endpoint_config_s.begin(), buffer_endpoint_config_s.end(),
            [&remote_endpoint](const auto &config) {
                return config.get().endpoint.address == remote_endpoint.address;
            });

        if (config_it != buffer_endpoint_config_s.end()) {
            register_new_session_internal(incoming_session_s, *config_it);
        } else {
            // Will delete!
            static endpoint_config_type config_tmp = {
                {remote_endpoint.v, remote_endpoint.address, remote_endpoint.port},
                noise::noise_role::RESPONDER,
                {},
                {}};
            add_endpoint_config_internal(config_tmp);
            register_new_session_internal(incoming_session_s, config_tmp);
        }
        session_info_it = registered_session_s.end() - 1;
        try {
            handle_session(*session_info_it);
            if (!protocol::try_handle(*session_info_it, pckt))
                session_info_it->get_log().throw_exception("Received invalid packet.");
        } catch (const base_error &) {
            delete_registered_session_internal(std::size_t(
                std::distance(registered_session_s.begin(), session_info_it)));
            throw;
        }
    }
    session_info_it->last_received_ms = get_now_ms();
    cv_io.notify_one();

    return {pckt,
            std::size_t(std::distance(registered_session_s.begin(), session_info_it))};
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

void essu::session_handler::add_endpoint_config_internal(
    const endpoint_config_type &endpoint_config) {
    if (buffer_endpoint_config_s.size() == max_session_number)
        log.throw_exception<session_error>("Failed to add config of endpoint.");

    // Saves passed config of endpoint
    buffer_endpoint_config_s.push_back(
        std::reference_wrapper<const endpoint_config_type>(endpoint_config));
}
void essu::session_handler::register_new_session_internal(
    buffer_session_s_type &session_s, const endpoint_config_type &endpoint_config) {
    if (session_s.size() == max_session_number)
        log.throw_exception<session_error>("Failed to add session.");

    // Creates new session with this config
    decltype(auto) endpoint_config_it =
        std::find_if(buffer_endpoint_config_s.begin(), buffer_endpoint_config_s.end(),
                     [&endpoint_config](const auto &_endpoint_config) {
                         return &_endpoint_config.get() == &endpoint_config;
                     });
    if (endpoint_config_it == buffer_endpoint_config_s.end())
        log.throw_exception<session_error>("Config of endpoint is not added.");

    decltype(auto) endpoint_config_r = endpoint_config_it->get();
    session_s.emplace_back(endpoint_config_r.endpoint, endpoint_config_r.local_role,
                           config.ext, endpoint_config_r.public_key,
                           endpoint_config_r.pre_shared_key, config.keypair);

    session_s[session_s.size() - 1].get_log().to_all("New session registered.");
}
void essu::session_handler::register_incoming_session_internal(
    std::size_t session_info_it) {
    if (incoming_session_s.size() == max_session_number)
        log.throw_exception<session_error>("Failed to add session.");

    // Copies new session
    if (session_info_it >= incoming_session_s.size())
        log.throw_exception<session_error>("Incoming session info does not exist.");

    registered_session_s.push_back(std::move(incoming_session_s[session_info_it]));

    incoming_session_s[incoming_session_s.size() - 1].get_log().to_all(
        "Incoming session registered.");
}
void essu::session_handler::delete_registered_session_internal(
    std::size_t session_info_it) {
    if (session_info_it >= registered_session_s.size())
        log.throw_exception<noheap::runtime_error>("Invalid the index of session info.");

    registered_session_s[session_info_it].get_log().to_all("Registered session removed.");
    registered_session_s.erase(registered_session_s.begin() + session_info_it);
}
void essu::session_handler::delete_incoming_session_internal(
    std::size_t session_info_it) {
    if (session_info_it >= incoming_session_s.size())
        log.throw_exception<noheap::runtime_error>("Invalid the index of session info.");

    incoming_session_s[session_info_it].get_log().to_all("Incoming session removed.");
    incoming_session_s.erase(incoming_session_s.begin() + session_info_it);
}
void essu::session_handler::handle_session(session_info_type_extended &session_info) {
    scope_guard    sc([this] { this->cv_io.notify_all(); });
    std::size_t    now_ms         = get_now_ms();
    decltype(auto) session_status = protocol::get_handshake_status(session_info);

    // Tries to control session and handles possible errors
    if (now_ms - session_info.last_received_ms >= timeout_ms)
        session_info.get_log().throw_exception<session_error>(
            "Timeout has been reached.");

    // If status of session is START or STOP:
    if (session_status == session_info_type::handshake_status_enum::START) {
        protocol::start_handshake(session_info);
        session_info.get_log().to_all(
            "{} Performing handshake...",
            std::string_view(
                noheap::hex_encode(protocol::get_current_state_hash(session_info))));
    } else if (session_status == session_info_type::handshake_status_enum::STOP) {
        protocol::stop_handshake(session_info);
        session_info.get_log().to_all(
            "{} Handshake completed.",
            std::string_view(
                noheap::hex_encode(protocol::get_current_state_hash(session_info))));
    }
}

#endif
