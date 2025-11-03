#include <ainput.hpp>
#include <aoutput.hpp>
#include <boost/asio.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: alsa_tcp_voice <port>" << std::endl;
        return 1;
    }
    input in;
    output op;
    while (true) {
        op.play_samples(in.get_samples());
    }
    return 0;
}
