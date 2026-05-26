#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include <mutex>

#include "essu_protocol.hpp"

namespace essu {
using namespace boost;

class session_handler : public network::action<packet_type> {
    static constexpr std::size_t run_thread_wake_up_ms = 500;
    static constexpr std::size_t buffer_packets_size   = 128;

    struct session_info_type_extended : public session_info_type {
        friend class session_handler;

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
              dynamic_owner(
                  noheap::to_new_buffer<noheap::log_impl::owner_impl::buffer_type>(
                      string_remote_address)),
              last_received_ms(get_now_ms()), log(_log_handler, dynamic_owner) {}

    private:
        const network::buffer_string_address_type       string_remote_address;
        const noheap::log_impl::owner_impl::buffer_type dynamic_owner;
        std::size_t                                     last_received_ms;

    public:
        const log_proxy log;
    };

    struct packet_type_extended : public packet_type {
        session_info_type_extended &session_info_r;
    };

    using session_s_type =
        noheap::monotonic_placement_new_array<session_info_type_extended,
                                              max_session_number>;

    using buffer_packet_s_type =
        noheap::monotonic_placement_new_array<packet_type_extended, buffer_packets_size>;

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

    using buffer_endpoint_config_s_type =
        noheap::monotonic_array<endpoint_config_type, max_session_number>;

public:
    inline void set_running(bool value);
    inline void set_config(config_type &&_config);
    inline void register_session(endpoint_config_type &endpoint_config);
    inline void run();

    inline void        send_packet(packet_type &&pckt);
    inline packet_type get_packet();

public:
    inline void init_packet(packet_type &pckt);
    inline void handle_packet(packet_type &&pckt);
    inline void set_error(const noheap::runtime_error &_excp);
    inline bool is_running();

private:
    inline void handle_runtime_error(const noheap::runtime_error &_excp);
    inline void handle_base_error(const base_error                 &_excp,
                                  const session_info_type_extended &session_info);
    inline void remove_session(const session_info_type_extended &session_info);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION_HANDLER");
    static constexpr log_handler log{buffer_owner};

private:
    config_type                   config{};
    buffer_endpoint_config_s_type buffer_endpoint_config_s;

    std::mutex              m_run;
    std::condition_variable cv_run;
    std::condition_variable cv_io;
    session_s_type          session_s;
    buffer_packet_s_type    send_buffer;
    buffer_packet_s_type    receive_buffer;
    noheap::runtime_error   excp;
    bool                    running = true;
    bool                    failed  = false;
};

} // namespace essu

inline void essu::session_handler::set_config(config_type &&_config) {
    config  = std::move(_config);
    _config = {};
}
void essu::session_handler::register_session(endpoint_config_type &endpoint_config) {
    std::lock_guard<std::mutex> m_run_lock(m_run);

    if (session_s.size() == max_session_number)
        log.throw_exception<session_error>("Failed to register session.");

    // Saves passed config of endpoint
    buffer_endpoint_config_s.push_back(endpoint_config);
    endpoint_config = {};

    // Creates new session with this config
    decltype(auto) endpoint_config_r = *(buffer_endpoint_config_s.end() - 1);
    session_s.emplace_back(log, endpoint_config_r.endpoint, endpoint_config_r.local_role,
                           config.ext, endpoint_config_r.public_key,
                           endpoint_config_r.pre_shared_key, config.keypair);

    log.to_all("Session registered: {}",
               std::string_view(session_s[session_s.size() - 1].string_remote_address));
}
void essu::session_handler::set_running(bool value) {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    running = value;
}
bool essu::session_handler::is_running() {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    return running;
}
void essu::session_handler::run() {
    log.to_all("Running...");

    try {
        // Locks run_m and performs loop
        std::unique_lock<decltype(m_run)> m_run_lock(m_run);
        while (running) {
            scope_guard sc([this] { this->cv_io.notify_all(); });
            std::size_t now_ms = get_now_ms();
            for (decltype(auto) it = session_s.begin(); it < session_s.end(); ++it) {
                decltype(auto) session_info   = *it;
                decltype(auto) session_status = protocol::update_status(session_info);

                // Tries to control session and handles possible errors
                try {
                    if (now_ms - session_info.last_received_ms >= timeout_ms)
                        session_info.log.throw_exception<session_error>(
                            "Timeout has been reached.");

                    // If status of session is START or STOP:
                    if (session_status == session_info_type::status_enum::START) {
                        protocol::start_handshake(session_info);
                        session_info.log.to_all("Performing handshake...");
                    } else if (session_status == session_info_type::status_enum::STOP) {
                        protocol::stop_handshake(session_info);
                        session_info.log.to_all("Handshake completed.");
                    }

                } catch (const base_error &excp) {
                    handle_base_error(excp, session_info);
                    continue;
                }

                // Dummy trafic
                if (send_buffer.size() == 0 && protocol::can_send_packet(session_info)
                    && (session_status != session_info_type::status_enum::COMPLETE
                        || now_ms - session_info.last_received_ms >= keep_alive_ms)) {
                    packet_type pckt;
                    pckt.set_endpoint(session_info.remote_endpoint);
                    set_dummy_packet(pckt);
                    send_buffer.push_back(packet_type_extended{pckt, session_info});
                }

                // For testing
                if (receive_buffer.size())
                    receive_buffer.erase(receive_buffer.begin());
            }

            // Wait for notify
            cv_run.wait_for(m_run_lock, std::chrono::milliseconds(run_thread_wake_up_ms));
        }
    } catch (const noheap::runtime_error &_excp) {
        handle_runtime_error(_excp);
    }

    log.to_all("Stop.");

    if (failed)
        throw excp;
}
void essu::session_handler::init_packet(packet_type &pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    // Sets current session for possible work in the run-thread
    cv_run.notify_one();

    // Waits run-thread
    cv_io.wait(m_run_lock, [this] { return !this->running || this->send_buffer.size(); });
    if (!running)
        return;

    {
        decltype(auto) packet_extended = *send_buffer.begin();

        // Inits packet
        try {
            protocol::prepare(packet_extended, packet_extended.session_info_r);
        } catch (const protocol_error &excp) {
            handle_base_error(excp, packet_extended.session_info_r);
            return;
        }

        pckt = packet_extended;
        send_buffer.erase(send_buffer.begin());
    }

    // For testing: will delete
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return;
}
void essu::session_handler::handle_packet(packet_type &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    if (!running)
        return;

    session_s_type::iterator session_info_it = session_s.begin();
    decltype(auto)           remote_endpoint = pckt.get_endpoint();

    // Finds suitable session by packet
    bool session_found = false;
    for (decltype(auto) it = session_s.begin(); it < session_s.end(); ++it) {
        if (protocol::check_affiliation_packet(*it, pckt)) {
            it->remote_endpoint  = remote_endpoint;
            it->last_received_ms = get_now_ms();
            session_info_it      = it;
            session_found        = true;
            break;
        }
    }

    // Registers new session if session is not found
    if (!session_found) {
        buffer_endpoint_config_s_type::iterator it =
            std::find_if(buffer_endpoint_config_s.begin(), buffer_endpoint_config_s.end(),
                         [&remote_endpoint](const auto &config) {
                             return config.endpoint.address == remote_endpoint.address;
                         });

        {
            m_run_lock.unlock();
            if (it != buffer_endpoint_config_s.end()) {
                register_session(*it);
            } else {
                endpoint_config_type config_tmp = {
                    {remote_endpoint.v, remote_endpoint.address, remote_endpoint.port},
                    noise::noise_role::RESPONDER,
                    {},
                    {}};
                register_session(config_tmp);
            }
            session_info_it = session_s.end() - 1;
            m_run_lock.lock();
        }
        cv_run.notify_one();
        cv_io.wait(m_run_lock);

        if (!running)
            return;
    }

    // Handles packet
    try {
        protocol::handle(pckt, *session_info_it);
    } catch (const protocol_error &excp) {
        handle_base_error(excp, *session_info_it);
        return;
    }

    receive_buffer.push_back(packet_type_extended{pckt, *session_info_it});

    // Sets current session for possible work in the run thread
    cv_run.notify_one();
}

void essu::session_handler::set_error(const noheap::runtime_error &_excp) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    handle_runtime_error(_excp);
    cv_run.notify_one();
}

void essu::session_handler::handle_runtime_error(const noheap::runtime_error &_excp) {
    excp    = _excp;
    failed  = true;
    running = false;
}
void essu::session_handler::handle_base_error(
    const base_error &_excp, const session_info_type_extended &session_info) {
    // Logs error
    log.to_all("{}", _excp.what());

    remove_session(session_info);
}
void essu::session_handler::remove_session(
    const session_info_type_extended &session_info) {
    log.to_all("Session removed: {}",
               std::string_view(session_info.string_remote_address));
    session_s.erase(std::find_if(session_s.begin(), session_s.end(),
                                 [&session_info](const auto &_session_info) {
                                     return &_session_info == &session_info;
                                 }));
}

#endif
