#ifndef ESSU_SESSION_HPP
#define ESSU_SESSION_HPP

#include "essu_protocol.hpp"

using namespace boost;

namespace essu {

class session_handler : public network::action<packet_type> {
    struct session_info_type_internal : public session_info_type {
        session_info_type_internal(
            network::buffer_address_type _addr, noise::noise_role _role,
            noise::buffer_prologue_extention_type      _ext,
            const noise_context_type::buffer_key_type &_remote_public_key,
            const noise::buffer_pre_shared_key_type   &_pre_shared_key,
            const noise_context_type::keypair_type    &_local_keypair)
            : session_info_type(_addr, _role, _ext, _remote_public_key, _pre_shared_key,
                                _local_keypair) {}

    public:
        std::mutex m;
    };

    using session_s_type =
        noheap::monotonic_placement_new_array<session_info_type_internal,
                                              max_session_number>;

public:
    void register_session(network::buffer_address_type               remote_addr,
                          noise::noise_role                          role,
                          noise::buffer_prologue_extention_type      ext,
                          const noise_context_type::keypair_type    &local_keypair,
                          const noise_context_type::buffer_key_type &remote_public_key,
                          const noise::buffer_pre_shared_key_type   &pre_shared_key);
    void set_running(bool value);
    void run();

public:
    void                         init_packet(packet_type &pckt);
    void                         process_packet(packet_type &&pckt);
    network::buffer_address_type get_remote_address();
    void                         set_error(const noheap::runtime_error &_excp);

private:
    essu::session_handler::session_s_type::iterator find_session(packet_type &pckt);

private:
    static constexpr noheap::log_impl::owner_impl::buffer_type buffer_owner =
        noheap::log_impl::create_owner("ESSU_SESSION_HANDLER");
    static constexpr log_handler log{buffer_owner};

private:
    session_s_type session_s;

    std::mutex               m_run;
    std::condition_variable  cv_run;
    std::condition_variable  cv_io;
    std::size_t              session_it_for_send     = 0;
    session_s_type::iterator current_session_info_it = session_s.end();

    std::atomic<bool>     running = true;
    noheap::runtime_error excp;
};
} // namespace essu

void essu::session_handler::register_session(
    network::buffer_address_type remote_addr, noise::noise_role role,
    noise::buffer_prologue_extention_type      ext,
    const noise_context_type::keypair_type    &local_keypair,
    const noise_context_type::buffer_key_type &remote_public_key,
    const noise::buffer_pre_shared_key_type   &pre_shared_key) {
    std::unique_lock<std::mutex> m_run_lock(m_run);

    session_s.emplace_back(remote_addr, role, ext, remote_public_key, pre_shared_key,
                           local_keypair);
}
void essu::session_handler::set_running(bool value) {
    running = value;
}
void essu::session_handler::run() {
    // Locks run_m and wait cv_run notify
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);
    cv_run.wait(m_run_lock, [this] {
        if (!running)
            return true;
        decltype(auto) session_info = *current_session_info_it;
        {
            std::lock_guard<std::mutex> session_info_m_lock(session_info.m);
            decltype(auto) session_action = protocol::get_handshake_action(session_info);

            if (session_action == noise::noise_action::NONE
                && !protocol::can_send_packet(session_info)
                && !protocol::can_receive_packet(session_info))
                protocol::start_handshake(session_info);
            else if (session_action == noise::noise_action::SPLIT)
                protocol::stop_handshake(session_info);
        }
        current_session_info_it = session_s.end();
        cv_io.notify_all();
        return false;
    });

    if (!running)
        throw excp;
}
void essu::session_handler::init_packet(packet_type &pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    decltype(auto) session_info_it = this->find_session(pckt);
    decltype(auto) session_info    = *session_info_it;

    // Sets current session for possible work in the run-thread
    current_session_info_it = session_info_it;
    cv_run.notify_one();

    // Waits when session can send packet
    cv_io.wait(m_run_lock, [&session_info] {
        std::lock_guard<std::mutex> session_info_m_lock(session_info.m);
        return protocol::can_send_packet(session_info);
    });

    {
        std::lock_guard<std::mutex> session_info_m_lock(session_info.m);
        decltype(auto) session_action = protocol::get_handshake_action(session_info);

        // Initializes the packet
        if (session_action == noise::noise_action::NONE) {
            for (std::size_t i = 0; i < pckt->units.size(); ++i)
                pckt->units[i].header.type = decltype(pckt->units[0].header.type)::dummy;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        protocol::prepare(pckt, session_info);
    }
}
void essu::session_handler::process_packet(packet_type &&pckt) {
    std::unique_lock<decltype(m_run)> m_run_lock(m_run);

    decltype(auto) session_info_it = this->find_session(pckt);
    decltype(auto) session_info    = *session_info_it;

    {
        std::lock_guard<std::mutex> session_info_m_lock(session_info.m);
        protocol::handle(pckt, session_info);
    }

    // Sets current session for possible work in the run thread
    current_session_info_it = session_info_it;
    cv_run.notify_one();

    // Waits when session can receive packet
    cv_io.wait(m_run_lock, [&session_info] {
        std::lock_guard<std::mutex> session_info_m_lock(session_info.m);
        return protocol::can_receive_packet(session_info);
    });
}
network::buffer_address_type essu::session_handler::get_remote_address() {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    if (!session_s.size())
        throw noheap::runtime_error("No registered session.");

    if (session_it_for_send == session_s.size())
        session_it_for_send = 0;
    return session_s.at(session_it_for_send++).addr;
}
void essu::session_handler::set_error(const noheap::runtime_error &_excp) {
    std::lock_guard<std::mutex> m_run_lock(m_run);
    excp    = _excp;
    running = false;
    cv_run.notify_one();
}

essu::session_handler::session_s_type::iterator
    essu::session_handler::find_session(packet_type &pckt) {
    decltype(auto) it =
        std::find_if(session_s.begin(), session_s.end(),
                     [addr = pckt.get_address()](const session_info_type &el) {
                         return el.addr == addr;
                     });
    if (it == session_s.end())
        throw noheap::runtime_error("Invalid packet address.");

    return it;
}

#endif
