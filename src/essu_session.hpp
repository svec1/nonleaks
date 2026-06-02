#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include <mutex>

#include "essu_protocol.hpp"

namespace essu {
using namespace boost;

struct session_info_type_extended : private session_info_type {
    friend class session_handler;

public:
    session_info_type_extended(
        const log_handler &_log_handler, network::native_endpoint _remote_endpoint,
        noise::noise_role _role, noise::buffer_prologue_extention_type _ext,
        const noise_context_type::buffer_key_type &_remote_public_key,
        const noise::buffer_pre_shared_key_type   &_pre_shared_key,
        const noise_context_type::keypair_type    &_local_keypair)
        : session_info_type(log, _remote_endpoint, _role, _ext, _remote_public_key,
                            _pre_shared_key, _local_keypair),
          string_remote_address(network::utils::bytes_address_to_string(
              remote_endpoint.address, remote_endpoint.v)),
          dynamic_owner(noheap::to_new_buffer<noheap::log_impl::owner_impl::buffer_type>(
              string_remote_address)),
          last_received_ms(get_now_ms()), log(_log_handler, dynamic_owner) {}

private:
    const network::buffer_string_address_type       string_remote_address;
    const noheap::log_impl::owner_impl::buffer_type dynamic_owner;
    std::size_t                                     last_received_ms;

public:
    const log_proxy log;
};

class session_handler : public network::action<packet_type> {
    static constexpr std::size_t buffer_packets_size = 128;

public:
    struct packet_type_extended : public packet_type {
        session_info_type_extended &session_info_r;
    };

private:
    using buffer_session_s_type =
        noheap::monotonic_placement_new_array<session_info_type_extended,
                                              max_session_number>;

    using send_buffer_packet_s_type =
        noheap::monotonic_placement_new_array<packet_type_extended, buffer_packets_size>;
    using receive_buffer_packet_s_type =
        noheap::monotonic_placement_new_array<packet_type, buffer_packets_size>;

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
    inline void add_endpoint_config(const endpoint_config_type &endpoint_config);
    inline void register_session(const endpoint_config_type &endpoint_config);
    inline void remove_session(const session_info_type_extended &_session_info);
    inline buffer_session_s_type &get_session_list();
    inline void                   handle(const session_info_type_extended &_session_info);
    inline noheap::runtime_error  get_error() const;

    inline void                 push_packet(packet_type_extended &&pckt);
    inline packet_type_extended pop_packet();
    inline bool                 exist_received_packets() const;

public:
    inline void init_packet(packet_type &pckt);
    inline void handle_packet(packet_type &&pckt);
    inline void set_error(const noheap::runtime_error &_excp);
    inline bool is_valid() const;

private:
    inline void add_endpoint_config_internal(const endpoint_config_type &endpoint_config);
    inline void register_session_internal(const endpoint_config_type &endpoint_config);
    inline void handle_internal(const session_info_type_extended &_session_info);
    inline void handle_runtime_error(const noheap::runtime_error &_excp);
    inline buffer_session_s_type::iterator
        get_session_info(const session_info_type &session_info);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION_HANDLER");
    static constexpr log_handler log{buffer_owner};

private:
    config_type                  &config;
    buffer_endpoint_config_s_type buffer_endpoint_config_s;

    mutable std::mutex           m_run;
    std::condition_variable      cv_io;
    buffer_session_s_type        session_s;
    send_buffer_packet_s_type    send_buffer;
    receive_buffer_packet_s_type receive_buffer;
    noheap::runtime_error        excp;
    bool                         failed_io = false;
};

} // namespace essu

void essu::session_handler::add_endpoint_config(
    const endpoint_config_type &endpoint_config) {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    add_endpoint_config_internal(endpoint_config);
}
void essu::session_handler::register_session(
    const endpoint_config_type &endpoint_config) {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    register_session_internal(endpoint_config);

    log.to_all("Session registered: {}",
               std::string_view(session_s[session_s.size() - 1].string_remote_address));
}
void essu::session_handler::remove_session(
    const session_info_type_extended &_session_info) {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);
    decltype(auto)                   session_info = get_session_info(_session_info);

    log.to_all("Session removed: {}",
               std::string_view(session_info->string_remote_address));

    session_s.erase(session_info);
}
essu::session_handler::buffer_session_s_type &essu::session_handler::get_session_list() {
    return session_s;
}

void essu::session_handler::handle(const session_info_type_extended &_session_info) {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);
    if (failed_io)
        return;
    handle_internal(_session_info);
}
noheap::runtime_error essu::session_handler::get_error() const {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);
    return excp;
}
void essu::session_handler::push_packet(packet_type_extended &&pckt) {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);

    // Prepares the packet
    try {
        protocol::prepare(pckt.session_info_r, pckt);
    } catch (base_error &_excp) {
        _excp.set_session_info(pckt.session_info_r);
        throw;
    }
    send_buffer.push_back(pckt);

    // Sets current session for possible work
    cv_io.notify_one();
}
essu::session_handler::packet_type_extended essu::session_handler::pop_packet() {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);

    packet_type    pckt            = receive_buffer.at(0);
    decltype(auto) session_info_it = session_s.end();
    decltype(auto) remote_endpoint = pckt.get_endpoint();

    receive_buffer.erase(receive_buffer.begin());

    // Finds suitable session by packet
    for (decltype(auto) it = session_s.begin(); it < session_s.end(); ++it) {
        if (protocol::check_affiliation_packet(*it, pckt)) {
            it->remote_endpoint  = remote_endpoint;
            it->last_received_ms = get_now_ms();
            session_info_it      = it;
            break;
        }
    }

    // Registers new session if session is not found
    if (session_info_it == session_s.end()) {
        buffer_endpoint_config_s_type::iterator it = std::find_if(
            buffer_endpoint_config_s.begin(), buffer_endpoint_config_s.end(),
            [&remote_endpoint](const auto &config) {
                return config.get().endpoint.address == remote_endpoint.address;
            });

        if (it != buffer_endpoint_config_s.end()) {
            register_session_internal(*it);
        } else {
            static endpoint_config_type config_tmp = {
                {remote_endpoint.v, remote_endpoint.address, remote_endpoint.port},
                noise::noise_role::RESPONDER,
                {},
                {}};
            add_endpoint_config_internal(config_tmp);
            register_session_internal(config_tmp);
        }
        session_info_it = session_s.end() - 1;
        handle_internal(*session_info_it);
    }

    // Handles the packet
    try {
        protocol::handle(*session_info_it, pckt);
    } catch (base_error &_excp) {
        _excp.set_session_info(*session_info_it);
        throw;
    }

    // Sets current session for possible work
    cv_io.notify_one();

    return {pckt, *session_info_it};
}
bool essu::session_handler::exist_received_packets() const {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);
    return receive_buffer.size();
}
bool essu::session_handler::is_valid() const {
    std::lock_guard<decltype(m_run)> m_run_lock(m_run);
    return !failed_io;
}
void essu::session_handler::init_packet(packet_type &pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    // Waits run-thread
    cv_io.wait(m_run_lock, [this] { return failed_io || this->send_buffer.size(); });
    if (failed_io)
        return;

    pckt = send_buffer.at(0);
    send_buffer.erase(send_buffer.begin());
}
void essu::session_handler::handle_packet(packet_type &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    cv_io.wait(m_run_lock, [this] {
        return failed_io
               || this->receive_buffer.size() < this->receive_buffer.buffer_size();
    });
    if (failed_io)
        return;

    receive_buffer.push_back(pckt);
}
void essu::session_handler::add_endpoint_config_internal(
    const endpoint_config_type &endpoint_config) {
    if (buffer_endpoint_config_s.size() == max_session_number)
        log.throw_exception<session_error>("Failed to add config of endpoint.");

    // Saves passed config of endpoint
    buffer_endpoint_config_s.push_back(
        std::reference_wrapper<const endpoint_config_type>(endpoint_config));
}
void essu::session_handler::register_session_internal(
    const endpoint_config_type &endpoint_config) {
    if (session_s.size() == max_session_number)
        log.throw_exception<session_error>("Failed to register session.");

    // Creates new session with this config
    decltype(auto) endpoint_config_it =
        std::find_if(buffer_endpoint_config_s.begin(), buffer_endpoint_config_s.end(),
                     [&endpoint_config](const auto &_endpoint_config) {
                         return &_endpoint_config.get() == &endpoint_config;
                     });
    if (endpoint_config_it == buffer_endpoint_config_s.end())
        log.throw_exception<session_error>("Config of endpoint is not added.");

    decltype(auto) endpoint_config_r = endpoint_config_it->get();
    session_s.emplace_back(log, endpoint_config_r.endpoint, endpoint_config_r.local_role,
                           config.ext, endpoint_config_r.public_key,
                           endpoint_config_r.pre_shared_key, config.keypair);
}
void essu::session_handler::handle_internal(
    const session_info_type_extended &_session_info) {
    try {
        // Locks run_m and performs loop
        scope_guard sc([this] { this->cv_io.notify_all(); });
        std::size_t now_ms = get_now_ms();

        decltype(auto) session_info   = *get_session_info(_session_info);
        decltype(auto) session_status = protocol::update_status(session_info);

        // Tries to control session and handles possible errors
        if (now_ms - session_info.last_received_ms >= timeout_ms)
            session_info.log.throw_exception<session_error>("Timeout has been reached.");

        // If status of session is START or STOP:
        if (session_status == session_info_type::status_enum::START) {
            protocol::start_handshake(session_info);
            session_info.log.to_all("Performing handshake...");
        } else if (session_status == session_info_type::status_enum::STOP) {
            protocol::stop_handshake(session_info);
            session_info.log.to_all("Handshake completed.");
        }
    } catch (base_error &_excp) {
        _excp.set_session_info(_session_info);
        throw;
    } catch (const noheap::runtime_error &_excp) {
        handle_runtime_error(_excp);
        throw;
    }
}

void essu::session_handler::set_error(const noheap::runtime_error &_excp) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    handle_runtime_error(_excp);
    cv_io.notify_one();
}
void essu::session_handler::handle_runtime_error(const noheap::runtime_error &_excp) {
    excp      = _excp;
    failed_io = true;
}
essu::session_handler::buffer_session_s_type::iterator
    essu::session_handler::get_session_info(const session_info_type &session_info) {
    decltype(auto) session_info_it = std::find_if(
        session_s.begin(), session_s.end(), [&session_info](const auto &_session_info) {
            return &_session_info == &session_info;
        });
    if (session_info_it == session_s.end())
        log.throw_exception<session_error>("Session is not registered.");

    return session_info_it;
}

#endif
