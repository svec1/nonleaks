#ifndef AUDIO_FLOW_HPP
#define AUDIO_FLOW_HPP

#include "stream_audio.hpp"
#include "utils.hpp"

template<std::size_t _max_stream_size>
    requires(_max_stream_size != 0)
struct audio_flow {
    static constexpr std::size_t max_stream_size = _max_stream_size;
    using buffer_type                            = typename stream_audio::buffer_type;

public:
    audio_flow() {
        in_stream  = std::async(std::launch::async, [this] {
            try {
                while (running.load()) {
                    buffer_type buffer_tmp = io_audio.read();
                    {
                        std::lock_guard lock(in_stream_m);
                        in_stream_buffer.push(buffer_tmp);
                    }
                    filled.fetch_add(1);
                    filled.notify_one();
                }
            } catch (...) {
                running.store(false);
                throw;
            }
        });
        out_stream = std::async(std::launch::async, [this] {
            try {
                while (running.load()) {
                    buffer_type buffer_tmp;

                    std::size_t current_out_stream_size;
                    bool        lost = true;

                    {
                        std::lock_guard lock(out_stream_m);
                        current_out_stream_size = out_stream_buffer.size();
                        if (auto el = out_stream_buffer.pop(); el.has_value()) {
                            buffer_tmp = el.value();
                            lost       = false;
                        }
                    }
                    io_audio.write(buffer_tmp, lost);
                }
            } catch (...) {
                running.store(false);
                throw;
            }
        });
    }
    ~audio_flow() {
        running.store(false);
        wait_stopping();
    }

public:
    constexpr void pop(buffer_type &buffer) {
        check_running();

        if (!filled.load())
            filled.wait(0);
        filled.fetch_sub(1);

        std::lock_guard lock(in_stream_m);
        buffer = in_stream_buffer.pop();
    }
    constexpr void push(buffer_type &&buffer, bool lost) {
        check_running();

        std::lock_guard lock(out_stream_m);
        out_stream_buffer.push(lost ? std::nullopt : std::make_optional(buffer));
    }

private:
    void check_running() {
        if (running.load())
            return;

        wait_stopping();
    }

    void wait_stopping() {
        if (in_stream.valid())
            in_stream.get();
        if (out_stream.valid())
            out_stream.get();
    }

private:
    std::mutex               in_stream_m, out_stream_m;
    std::future<void>        in_stream, out_stream;
    std::atomic<bool>        running = true;
    std::atomic<std::size_t> filled  = 0;

    stream_audio io_audio;

    noheap::ring_buffer<buffer_type, max_stream_size>                in_stream_buffer;
    noheap::ring_buffer<std::optional<buffer_type>, max_stream_size> out_stream_buffer;
};

#endif
