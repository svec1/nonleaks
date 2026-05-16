#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include "essu_protocol.hpp"

using namespace boost;

namespace essu {

class session_handler : public network::action<packet_type> {
    static constexpr std::size_t buffer_packets_size = 128;

    using session_s_type =
        noheap::monotonic_placement_new_array<session_info_type, max_session_number>;

public:
    void register_session(network::ipv v, network::buffer_address_type remote_address,
                          network::port_type remote_port, noise::noise_role role,
                          noise::buffer_prologue_extention_type      ext,
                          const noise_context_type::keypair_type    &local_keypair,
                          const noise_context_type::buffer_key_type &remote_public_key,
                          const noise::buffer_pre_shared_key_type   &pre_shared_key);
    void set_running(bool value);
    bool get_running();
    void run();

    void        send_packet(packet_type &&pckt);
    packet_type get_packet();

public:
    bool init_packet(packet_type &pckt);
    void handle_packet(packet_type &&pckt);
    void set_error(const noheap::runtime_error &_excp);

private:
    essu::session_handler::session_s_type::iterator
        find_session(network::buffer_address_type remote_address);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION_HANDLER");
    static constexpr log_handler log{buffer_owner};

private:
    std::mutex                                                m_run;
    std::condition_variable                                   cv_run;
    std::condition_variable                                   cv_io;
    session_s_type                                            session_s;
    noheap::runtime_error                                     excp;
    noheap::monotonic_array<packet_type, buffer_packets_size> send_buffer;
    noheap::monotonic_array<packet_type, buffer_packets_size> receive_buffer;
    bool                                                      running = true;
    bool                                                      failed  = false;
};
} // namespace essu

void essu::session_handler::register_session(
    network::ipv v, network::buffer_address_type remote_address,
    network::port_type remote_port, noise::noise_role role,
    noise::buffer_prologue_extention_type      ext,
    const noise_context_type::keypair_type    &local_keypair,
    const noise_context_type::buffer_key_type &remote_public_key,
    const noise::buffer_pre_shared_key_type   &pre_shared_key) {
    std::lock_guard<std::mutex> m_run_lock(m_run);

    session_s.emplace_back(v, remote_address, remote_port, role, ext, remote_public_key,
                           pre_shared_key, local_keypair);

    log.to_all("Register new session: {}",
               std::string_view(session_s[session_s.size() - 1].string_remote_address));
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
    try {
        // Locks run_m and wait cv_run notify
        std::unique_lock<decltype(m_run)> m_run_lock(m_run);
        cv_run.wait(m_run_lock, [this] {
            scope_guard sc([this] { this->cv_io.notify_all(); });

            for (decltype(auto) session_info : session_s) {
                decltype(auto) session_status = protocol::get_status(session_info);

                // For testing
                if (session_status == session_info_type::status_enum::COMPLETE) {
                    if (!send_buffer.size() && protocol::can_send_packet(session_info)) {
                        packet_type pckt;
                        pckt.set_endpoint({session_info.v, session_info.remote_address,
                                           session_info.remote_port});
                        pckt->units[0].header.type = pckt->units[1].header.type =
                            decltype(pckt->units[0].header.type)::dummy;
                        send_buffer.push_back(pckt);
                    }
                    if (receive_buffer.size())
                        receive_buffer.pop_front();
                }

                // If status of session is START:
                //  1. If session has 0 handshake before(session has just been registered)
                //  2. If it was received retry-packet
                session_status = protocol::update_status(session_info);
                if (session_status == session_info_type::status_enum::START)
                    protocol::start_handshake(session_info);

                // If status of session is EXCHANGE:
                //  - if session waits to send or receive handshake packet
                session_status = protocol::update_status(session_info);
                if (session_status == session_info_type::status_enum::EXCHANGE) {
                    if (protocol::can_receive_packet(session_info)) {
                        if (receive_buffer.size())
                            protocol::handle_handshake_packet(session_info,
                                                              receive_buffer.pop_front());
                    }
                    if (protocol::can_send_packet(session_info)) {
                        packet_type pckt;
                        pckt.set_endpoint({session_info.v, session_info.remote_address,
                                           session_info.remote_port});

                        protocol::init_handshake_packet(session_info, pckt);
                        send_buffer.push_back(pckt);
                    }
                }

                // If status of session is STOP
                session_status = protocol::get_status(session_info);
                if (session_status == session_info_type::status_enum::STOP)
                    protocol::stop_handshake(session_info);
            }

            return !running;
        });
    } catch (const noheap::runtime_error &excp) {
        set_error(excp);
    }

    if (failed)
        throw excp;
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
        decltype(auto) session_info_it = this->find_session(pckt.get_endpoint().address);
        decltype(auto) session_info    = *session_info_it;

        protocol::prepare(pckt, session_info);

        // For testing: will delete
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return true;
}
void essu::session_handler::handle_packet(packet_type &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    // Waits when session can receive packet
    cv_io.wait(m_run_lock, [this] {
        return !this->running || receive_buffer.size() < buffer_packets_size;
    });

    if (!running)
        return;

    {
        decltype(auto) session_info_it = this->find_session(pckt.get_endpoint().address);
        decltype(auto) session_info    = *session_info_it;
        session_info.remote_port       = pckt.get_endpoint().port;

        protocol::handle(pckt, session_info);

        receive_buffer.push_back(pckt);
    }

    // Sets current session for possible work in the run thread
    cv_run.notify_one();
}

void essu::session_handler::set_error(const noheap::runtime_error &_excp) {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    excp    = _excp;
    running = false;
    failed  = true;
    cv_run.notify_one();
}

essu::session_handler::session_s_type::iterator
    essu::session_handler::find_session(network::buffer_address_type remote_address) {
    decltype(auto) it = std::find_if(session_s.begin(), session_s.end(),
                                     [remote_address](const session_info_type &el) {
                                         return el.remote_address == remote_address;
                                     });
    if (it == session_s.end())
        log.throw_exception("Invalid packet address.");

    return it;
}

#endif
