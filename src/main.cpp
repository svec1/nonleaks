#include "unix_udp_voice_service.hpp"

using namespace boost;
using dba         = stream_audio::default_base_audio;
using uuv_service = unix_udp_voice_service;

static constexpr log_handler log_main{{}};

struct vcu_config {
    std::string_view      device;
    asio::ip::port_type   port;
    uuv_service::ipv_type addr;
};

static void parse_options(vcu_config &cfg, int argc, char *argv[]) {
    static constexpr auto throw_usage = [](auto arg, int expected_argument = 0) {
        using re = noheap::runtime_error;

        re::buffer_type buffer;
        auto            end_it = buffer.begin();

        end_it = std::format_to_n(end_it, re::buffer_size,
                                  "Usage: alsa_tcp_voice [-D sound device] [-h "
                                  "IP Host] [-p port]\n")
                     .out;
        if (expected_argument == 1)
            end_it = std::format_to_n(end_it, re::buffer_size,
                                      "Option requires an argument: {}", arg)
                         .out;
        else if (expected_argument == -1)
            end_it =
                std::format_to_n(end_it, re::buffer_size, "Invalid argument: {}", arg)
                    .out;
        else
            end_it =
                std::format_to_n(end_it, re::buffer_size, "Invalid option: {}", arg).out;

        throw re(std::move(buffer));
    };

    static constexpr auto get_argument = [](int argc, char *argv[], int &i) {
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
        if (argv[i][0] != '-')
            throw_usage(argv[i]);

        std::string_view option        = argv[i];
        std::string_view current_value = get_argument(argc, argv, i);

        try {
            switch (option[1]) {
                case 'D': {
                    cfg.device = current_value;
                    break;
                }
                case 'h': {
                    cfg.addr = asio::ip::make_address_v4(current_value);
                    break;
                }
                case 'p': {
                    cfg.port = std::stoi(current_value.data());
                    break;
                }
                default:
                    throw_usage(option[1]);
            }
        } catch (system::system_error &) {
            throw_usage(current_value, -1);
        }
    }
}

void print_cfg(const vcu_config &cfg) {
    using ca_type = stream_audio::ca_type;

    log_main.to_console(" -- Sound architecture: {}", dba::arsnd_name);
    log_main.to_console(" -- Sound device: {}", cfg.device);
    log_main.to_console(" -- Listening port: {}", cfg.port);
    log_main.to_console(" -- Contact address: {}", cfg.addr.to_string());
    log_main.to_console(" -- Audio config: ");
    log_main.to_console("    | Bitrate: {} kbit/s", dba::cfg.bitrate / 1000);
    log_main.to_console("    | Latency: {} ms", dba::cfg.latency);
    log_main.to_console("    | Channels: {}", dba::cfg.channels);
    log_main.to_console("    | Rate: {} hz", dba::cfg.sample_rate);
    log_main.to_console("    | Sample size: {} bits", dba::cfg.bits_per_sample);
}

int main(int argc, char *argv[]) {
    try {
        vcu_config cfg = {.device = dba::default_device_playback, .port = 8888};
        parse_options(cfg, argc, argv);
        print_cfg(cfg);

        dba::device_playback = cfg.device;
        dba::device_capture  = cfg.device;

        unix_udp_voice_service vsc(cfg.port);
        vsc.run(cfg.addr);

    } catch (noheap::runtime_error &excp) {
        log_main.exception_to_all(excp);
        return 1;
    } catch (std::exception &excp) {
        log_main.to_all("Program panic: {}.", excp.what());
        return 1;
    }

    return 0;
}
