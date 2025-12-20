#include <unix_udp_voice_service.hpp>

static constexpr std::size_t max_count_senders = 64;

using namespace boost;

using mpacket = debug_packet<audio::buffer_size, max_count_senders>;
using uuv_service = unix_udp_voice_service<mpacket>;

struct vcu_config {
    std::string_view device;
    asio::ip::port_type port;
    noheap::vector_stack<uuv_service::nstream_t::ipv_t, mpacket::max_count_senders> addrs;
};

static void parse_options(vcu_config& cfg, int argc, char* argv[]) {
    static constexpr auto throw_usage = [](auto arg,
                                           int expected_argument = 0) {
	noheap::runtime_error::buffer_t buffer;	

	auto end_it = std::format_to(buffer.begin(), "Usage: alsa_tcp_voice [-D sound device] [-h IP Host] [-p port]\n");
        if (expected_argument == 1)
            end_it = std::format_to(end_it, "Option requires an argument: {}", arg);
        else if (expected_argument == -1)
            end_it = std::format_to(end_it, "Invalid argument: {}", arg);
        else
            end_it = std::format_to(end_it, "Invalid option: {}", arg);
	
        throw noheap::runtime_error(std::move(buffer));
    };

    static constexpr auto get_argument = [](int argc, char* argv[], int& i) {
        if (std::strlen(argv[i]) > 2)
            return argv[i] + 2;
        else {
            if (argc <= i + 1)
                throw_usage(argv[i], 1);
            else if (argv[i + 1][0] == '-')
                throw_usage(argv[i + 1], -1);

            return argv[++i];
        }
    };

    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] != '-') throw_usage(argv[i]);

        std::string_view option = argv[i];
        std::string_view current_value = get_argument(argc, argv, i);

        try {
            switch (option[1]) {
                case 'D': {
                    cfg.device = current_value;
                    break;
                }
                case 'h': {
		    auto addr = asio::ip::make_address_v4(current_value);
                    if (std::find(cfg.addrs.data.begin(), cfg.addrs.data.end(), addr) ==
                        cfg.addrs.data.end())
                        cfg.addrs.data.push_back(addr);
                    break;
                }
                case 'p': {
                    cfg.port = std::stoi(current_value.data());
                    break;
                }
                default:
                    throw_usage(option[1]);
            }
        } catch (system::system_error&) {
            throw_usage(current_value, -1);
        }
    }
}

void print_cfg(const vcu_config& cfg) {
    noheap::println("-- Sound architecture: {}", audio::arsnd_name);
    noheap::println("-- Sound device: {}", cfg.device);
    noheap::println("-- Listening port: {}", cfg.port);
    noheap::println("-- Possible contact addresses:");
    std::for_each(cfg.addrs.data.begin(), cfg.addrs.data.end(), [](auto& addr) {
        noheap::println("   | {}", addr.to_string());
    });
}

int main(int argc, char* argv[]) {
    try {
        vcu_config cfg = {.device = audio::default_device_playback,
                          .port = 8888};
        parse_options(cfg, argc, argv);
        print_cfg(cfg);

        audio::device_playback = cfg.device;
        audio::device_capture = cfg.device;

        uuv_service vsc(cfg.addrs.data, cfg.port);
    } catch (noheap::runtime_error& excp) {
        noheap::log_impl::logx(1, excp.has_set_owner() ? excp.get_owner() : noheap::log_impl::create_owner(""), "{}", excp.what());
        return 1;
    }

    return 0;
}
