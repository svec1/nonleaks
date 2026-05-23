#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include <mutex>

#include "essu_protocol.hpp"

namespace essu {
using namespace boost;

class session_handler : public network::action<packet_type> {
    static constexpr std::size_t run_thread_wake_up_ms = 1000;
    static constexpr std::size_t buffer_packets_size   = 128;

    struct session_info_type_extended : session_info_type {
        friend class session_handler;

        session_info_type_extended(
            const log_handler &_log_handler, network::ipv _v,
            network::buffer_address_type _remote_address, std::uint16_t _remote_port,
            noise::noise_role _role, noise::buffer_prologue_extention_type _ext,
            const noise_context_type::buffer_key_type &_remote_public_key,
            const noise::buffer_pre_shared_key_type   &_pre_shared_key,
            const noise_context_type::keypair_type    &_local_keypair)
            : session_info_type(log, _v, _remote_address, _remote_port, _role, _ext,
                                _remote_public_key, _pre_shared_key, _local_keypair),
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
    using session_s_type =
        noheap::monotonic_placement_new_array<session_info_type_extended,
                                              max_session_number>;

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

public:
    inline void set_config(config_type &&_config);
    inline void register_session(const endpoint_config_type &endpoint_config);
    inline void set_running(bool value);
    inline bool get_running();
    inline void run();

    inline void        send_packet(packet_type &&pckt);
    inline packet_type get_packet();

public:
    inline bool init_packet(packet_type &pckt);
    inline void handle_packet(packet_type &&pckt);
    inline void set_error(const noheap::runtime_error &_excp,
                          network::buffer_address_type remote_endpoint_address);

private:
    inline bool contain_session(network::buffer_address_type remote_endpoint_address);
    inline essu::session_handler::session_s_type::value_type &
        at_session(network::buffer_address_type remote_endpoint_address);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION_HANDLER");
    static constexpr log_handler log{buffer_owner};

private:
    config_type config{};

    std::mutex                                                m_run;
    std::condition_variable                                   cv_run;
    std::condition_variable                                   cv_io;
    session_s_type                                            session_s;
    noheap::monotonic_array<packet_type, buffer_packets_size> send_buffer;
    noheap::monotonic_array<packet_type, buffer_packets_size> receive_buffer;
    bool                                                      running = true;
};

} // namespace essu

inline void essu::session_handler::set_config(config_type &&_config) {
    config  = std::move(_config);
    _config = {};
}
void essu::session_handler::register_session(
    const endpoint_config_type &endpoint_config) {
    std::lock_guard<std::mutex> m_run_lock(m_run);

    session_s.emplace_back(
        log, endpoint_config.endpoint.v, endpoint_config.endpoint.address,
        endpoint_config.endpoint.port, endpoint_config.local_role, config.ext,
        endpoint_config.public_key, endpoint_config.pre_shared_key, config.keypair);

    session_s[session_s.size() - 1].log.to_all("Session registered.");
}
void essu::session_handler::set_running(bool value) {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    running = value;
}
bool essu::session_handler::get_running() {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    return running;
}
void essu::session_handler::run() {
    log.to_all("Running...");

    // Locks run_m and performs loop
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    while (running) {
        scope_guard sc([this] { this->cv_io.notify_all(); });
        std::size_t now_ms = get_now_ms();
        for (decltype(auto) session_info : session_s) {
            try {
                if (now_ms - session_info.last_received_ms >= timeout_ms)
                    session_info.log.throw_exception("Timeout has been reached.");

                // If status of session is START or STOP:
                decltype(auto) session_status = protocol::update_status(session_info);
                if (session_status == session_info_type::status_enum::START) {
                    protocol::start_handshake(session_info);
                    session_info.log.to_all("Performing handshake...");
                } else if (session_status == session_info_type::status_enum::STOP) {
                    protocol::stop_handshake(session_info);
                    session_info.log.to_all("Handshake completed.");
                }

                // Dummy trafic
                if (send_buffer.size() == 0 && protocol::can_send_packet(session_info)) {
                    packet_type pckt;
                    pckt.set_endpoint(session_info.remote_endpoint);
                    set_dummy_packet(pckt);
                    send_buffer.push_back(pckt);
                }

                // For testing
                if (receive_buffer.size())
                    receive_buffer.pop_front();
            } catch (const noheap::runtime_error &_excp) {
                m_run_lock.unlock();
                set_error(_excp, session_info.remote_endpoint.address);
                m_run_lock.lock();
            }
        }

        // Wait for notify
        cv_run.wait_for(m_run_lock, std::chrono::milliseconds(run_thread_wake_up_ms));
    }
}
bool essu::session_handler::init_packet(packet_type &pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    // Sets current session for possible work in the run-thread
    cv_run.notify_one();

    // Waits run-thread
    cv_io.wait(m_run_lock, [this] { return !this->running || this->send_buffer.size(); });
    if (!running)
        return false;

    pckt = send_buffer.pop_front();

    {
        decltype(auto) session_info = at_session(pckt.get_endpoint().address);

        protocol::prepare(pckt, session_info);

        // For testing: will delete
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return true;
}
void essu::session_handler::handle_packet(packet_type &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    decltype(auto)                    remote_endpoint = pckt.get_endpoint();

    // Registers new session if session_s does not contain it
    if (!contain_session(remote_endpoint.address)) {
        m_run_lock.unlock();
        register_session(
            {{remote_endpoint.v, remote_endpoint.address, remote_endpoint.port},
             noise::noise_role::RESPONDER,
             {},
             {}});
        m_run_lock.lock();
        cv_run.notify_one();
        cv_io.wait(m_run_lock);
    } else
        // Waits when session can receive packet
        cv_io.wait(m_run_lock, [this] {
            return !this->running || receive_buffer.size() < buffer_packets_size;
        });

    if (!running)
        return;

    {
        decltype(auto) session_info       = at_session(remote_endpoint.address);
        session_info.remote_endpoint.port = remote_endpoint.port;
        session_info.last_received_ms     = get_now_ms();

        protocol::handle(pckt, session_info);

        receive_buffer.push_back(pckt);
    }

    // Sets current session for possible work in the run thread
    cv_run.notify_one();
}

void essu::session_handler::set_error(
    const noheap::runtime_error &excp,
    network::buffer_address_type remote_endpoint_address) {
    std::lock_guard<std::mutex> m_run_lock(m_run);

    // Erases session from list of sessions
    if (contain_session(remote_endpoint_address)) {
        session_s.erase(std::find_if(
            session_s.begin(), session_s.end(),
            [&remote_endpoint_address](const auto &session_info) {
                return session_info.remote_endpoint.address == remote_endpoint_address;
            }));
    }

    // Logs error
    log.to_all("{}", excp.what());

    cv_run.notify_one();
}

bool essu::session_handler::contain_session(
    network::buffer_address_type remote_endpoint_address) {
    return std::find_if(session_s.begin(), session_s.end(),
                        [remote_endpoint_address](const session_info_type &el) {
                            return el.remote_endpoint.address == remote_endpoint_address;
                        })
           != session_s.end();
}
essu::session_handler::session_s_type::value_type &essu::session_handler::at_session(
    network::buffer_address_type remote_endpoint_address) {
    decltype(auto) it =
        std::find_if(session_s.begin(), session_s.end(),
                     [remote_endpoint_address](const session_info_type &el) {
                         return el.remote_endpoint.address == remote_endpoint_address;
                     });
    if (it == session_s.end())
        log.throw_exception("Invalid packet address.");

    return *it;
}

#endif
