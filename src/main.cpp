#include <unistd.h>

#include <unix_udp_voice_service.hpp>

using namespace boost;

using mpacket = debug_packet<audio::buffer_size>;
using uuv_service = unix_udp_voice_service<mpacket>;

struct vcu_config {
    std::string_view device;
    asio::ip::port_type port;
    std::vector<uuv_service::nstream_t::ipv_t> addrs;
};

static void parse_options(vcu_config& cfg, int argc, char* argv[]) {
    static constexpr auto throw_usage = [](std::string_view arg,
                                           int expected_argument = 0) {
        std::string throw_string;

        if (expected_argument == 1)
            throw_string = std::format("Option requires an argument: {}", arg);
        else if (expected_argument == -1)
            throw_string = std::format("Invalid argument: {}", arg);
        else
            throw_string = std::format("Invalid option: {}", arg);

        throw std::runtime_error(std::format(
            "{}\n"
            "Usage: alsa_tcp_voice [-D sound device] [-h IP Host] [-p port]",
            throw_string));
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
                    if (std::find(cfg.addrs.begin(), cfg.addrs.end(), addr) ==
                        cfg.addrs.end())
                        cfg.addrs.push_back(addr);
                    break;
                }
                case 'p': {
                    cfg.port = std::stoi(current_value.data());
                    break;
                }
                default:
                    throw_usage(std::string("-") + option[1]);
            }
        } catch (system::system_error&) {
            throw_usage(current_value, -1);
        }
    }
}

void print_cfg(const vcu_config& cfg) {
    std::println("-- Sound architecture: {}", AR_NAME);
    std::println("-- Sound device: {}", cfg.device);
    std::println("-- Listening port: {}", cfg.port);
    std::println("-- Possible contact addresses:");
    std::for_each(cfg.addrs.begin(), cfg.addrs.end(), [](auto& addr) {
        std::println("   | {}", addr.to_string());
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

        uuv_service vsc(cfg.addrs, cfg.port);
    } catch (std::runtime_error& excp) {
        std::println("{}", excp.what());
        return 1;
    }

    return 0;
}
