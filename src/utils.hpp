#ifndef UTILS_HPP
#define UTILS_HPP

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <format>
#include <future>
#include <memory_resource>
#include <mutex>
#include <queue>
#include <span>
#include <string_view>
#include <vector>

namespace std {
using ssize_t = std::make_signed_t<std::size_t>;
} // namespace std

constexpr std::size_t get_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

namespace noheap {

static constexpr std::size_t output_buffer_size = 1024;

template<std::size_t buffer_size, typename T = char>
    requires std::is_integral<T>::value
using buffer_bytes_type = std::array<T, buffer_size>;

template<typename T>
concept Buffer_bytes_type =
    std::same_as<std::decay_t<T>,
                 buffer_bytes_type<T{}.size(), typename std::decay_t<T>::value_type>>;

template<typename T>
concept Buffer =
    std::same_as<std::decay_t<T>,
                 std::array<typename std::decay_t<T>::value_type, sizeof(T)>>;

template<Buffer TReturn, typename TSource>
    requires(TReturn{}.size() >= sizeof(TSource))
constexpr TReturn to_buffer(TSource el) {
    TReturn buffer_tmp{};

    auto begin = reinterpret_cast<std::int8_t *>(&el);
    auto end   = reinterpret_cast<std::int8_t *>(&el) + sizeof(TSource);

    std::copy(begin, end, buffer_tmp.begin());

    return buffer_tmp;
}

template<Buffer TReturn, Buffer TSource>
constexpr TReturn to_new_buffer(TSource &&buffer) {
    TReturn buffer_tmp{};
    auto    begin = buffer.begin();
    auto    end   = begin;

    if (buffer.size() >= buffer_tmp.size())
        end += buffer_tmp.size();
    else
        end += buffer.size();

    std::copy(begin, end, buffer_tmp.begin());
    return buffer_tmp;
}

template<Buffer TReturn, Buffer TSource>
constexpr TReturn to_hex_string(TSource &&buffer) {
    TReturn buffer_tmp{};

    auto it = buffer_tmp.begin();

    for (auto ch : buffer)
        it = std::format_to_n(it, 2, "{:X}", ch).out;

    return buffer_tmp;
}

template<typename TReturn, Buffer TSource>
    requires(!std::is_pointer_v<TReturn>
             && std::decay_t<TSource>{}.size() == sizeof(TReturn))
constexpr TReturn represent_bytes(TSource &&buffer) {
    return *reinterpret_cast<TReturn *>(buffer.data());
}

class print_impl final {
public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_type                        = buffer_bytes_type<buffer_size>;

public:
    template<char end_ch, typename... Args>
    static void out(std::format_string<Args...> format, Args &&...args) {
        buffer_type buffer{};
        auto        end_it = std::format_to_n(buffer.begin(), buffer_size, format,
                                              std::forward<Args>(args)...);
        *end_it.out        = end_ch;

        out_buffer(std::move(buffer));
    }

    static void out_buffer(buffer_type &&buffer, std::size_t outstream = 1) {
        ::write(outstream, buffer.data(), buffer_size);
    }
};

template<typename... Args>
constexpr void print(std::format_string<Args...> format, Args &&...args) {
    print_impl::out<'\0'>(format, std::forward<Args>(args)...);
}
template<typename... Args>
constexpr void println(std::format_string<Args...> format, Args &&...args) {
    print_impl::out<'\n'>(format, std::forward<Args>(args)...);
}

class log_impl final {
public:
    struct owner_impl final {
        static constexpr std::size_t buffer_size = 24;
        using buffer_type                        = buffer_bytes_type<buffer_size>;
    };

    static consteval owner_impl::buffer_type create_owner(std::string_view owner) {
        owner_impl::buffer_type buffer_owner{};

        for (std::size_t i = 0; i < owner_impl::buffer_size && i < owner.size(); ++i)
            buffer_owner[i] = owner[i];

        return buffer_owner;
    };

    template<typename... Args>
    static constexpr print_impl::buffer_type
        create_log_data(owner_impl::buffer_type     buffer_owner,
                        std::format_string<Args...> format, Args &&...args) {
        print_impl::buffer_type buffer{};
        auto                    end_it = buffer.begin();

        if (buffer_owner[0] != '\0') {
            std::transform(buffer_owner.begin(), buffer_owner.end(), buffer_owner.begin(),
                           [](unsigned char ch) { return std::toupper(ch); });
            end_it = std::format_to_n(buffer.begin(), print_impl::buffer_size,
                                      "[{}]: ", buffer_owner.data())
                         .out;
        }

        end_it = std::format_to_n(end_it, std::abs(std::distance(buffer.end(), end_it)),
                                  format, std::forward<Args>(args)...)
                     .out;
        *end_it = '\n';
        return buffer;
    }
};

class runtime_error final : public std::exception {
public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_type                        = buffer_bytes_type<buffer_size>;

public:
    template<typename... Args>
    runtime_error(std::format_string<Args...> format, Args &&...args) {
        if (format.get().size()) {
            auto end_it = std::format_to_n(buffer.begin(), buffer_size, format,
                                           std::forward<Args>(args)...);
            *end_it.out = '\0';
        }
    }
    template<typename... Args>
    runtime_error(noheap::log_impl::owner_impl::buffer_type _buffer_owner,
                  std::format_string<Args...>               format, Args &&...args)
        : runtime_error(format, std::forward<Args>(args)...) {
        buffer_owner = _buffer_owner;
    }
    runtime_error() = default;
    runtime_error(buffer_type &&_buffer) : buffer(std::move(_buffer)) {}
    runtime_error(const runtime_error &excp) {
        buffer = excp.buffer;
        set_owner(excp.buffer_owner);
    }
    ~runtime_error() override = default;

public:
    void set_owner(log_impl::owner_impl::buffer_type _buffer_owner) {
        buffer_owner = _buffer_owner;
        owner_set    = true;
    }

public:
    const char *what() const noexcept override { return buffer.data(); }
    log_impl::owner_impl::buffer_type get_owner() const noexcept { return buffer_owner; }
    bool has_setting_owner() const noexcept { return owner_set; }

private:
    buffer_type                       buffer{};
    log_impl::owner_impl::buffer_type buffer_owner{};

    bool owner_set;
};

template<std::size_t _buffer_size>
struct pseudoheap_monotonic_array;

template<typename T, std::size_t _buffer_size>
struct basic_array {
    friend struct pseudoheap_monotonic_array<_buffer_size>;

public:
    static constexpr std::size_t buffer_size = _buffer_size;

    using value_type  = T;
    using buffer_type = std::array<value_type, buffer_size>;

protected:
    buffer_type buffer{};
};

template<std::size_t _buffer_size>
struct pseudoheap_monotonic_array {
private:
    using basic_array_type = basic_array<int8_t, _buffer_size>;

public:
    pseudoheap_monotonic_array() = default;

public:
    template<typename T>
        requires std::is_pointer<T>::value
    T malloc(std::size_t area_size) {
        if (offset + area_size >= basic_array_type::buffer_size)
            throw runtime_error("Pseudoheap is full. Last request: {}", area_size);

        typename basic_array_type::value_type *ptr =
            this->basic_array.buffer.data() + offset;
        offset += area_size;
        return reinterpret_cast<T>(ptr);
    }

private:
    basic_array_type basic_array;
    std::size_t      offset = 0;
};
template<typename T, std::size_t _buffer_size>
struct const_array : public basic_array<T, _buffer_size> {
public:
    constexpr const_array() = default;

public:
    constexpr operator typename const_array::buffer_type &() const & noexcept {
        return this->buffer;
    }
};
template<typename T, std::size_t _buffer_size>
class monotonic_array : public basic_array<T, _buffer_size> {
public:
    constexpr monotonic_array() = default;
    monotonic_array(monotonic_array &&array) {
        std::swap(this->buffer, array.buffer);
        count_pushed = array.count_pushed;
    }

public:
    std::size_t                            size() const { return count_pushed; }
    monotonic_array::buffer_type::iterator begin() { return this->buffer.begin(); }
    monotonic_array::buffer_type::iterator end() {
        return this->buffer.begin() + count_pushed;
    }
    monotonic_array::buffer_type::iterator       bend() { return this->buffer.end(); }
    monotonic_array::buffer_type::const_iterator begin() const {
        return this->buffer.cbegin();
    }
    monotonic_array::buffer_type::const_iterator end() const {
        return this->buffer.cbegin() + count_pushed;
    }
    monotonic_array::buffer_type::const_iterator bend() const {
        return this->buffer.end();
    }
    monotonic_array::buffer_type::const_iterator cbegin() const {
        return this->buffer.begin();
    }
    monotonic_array::buffer_type::const_iterator cend() const {
        return this->buffer.begin() + count_pushed;
    }
    monotonic_array::buffer_type::const_iterator cbend() const {
        return this->buffer.end();
    }

public:
    template<typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push_back(_T &&el) {
        if (count_pushed == monotonic_array::buffer_size)
            throw runtime_error("Buffer overflow.");
        this->buffer[count_pushed++] = std::forward<_T>(el);
    }
    template<typename... Args>
        requires std::constructible_from<std::decay_t<T>, Args...>
    void emplace(monotonic_array::buffer_type::iterator it, Args &&...args) {
        if (count_pushed == monotonic_array::buffer_size)
            throw runtime_error("Buffer overflow.");
        else if (it == this->buffer.end())
            throw runtime_error("Invalid access.");

        for (auto it_tmp = this->end(); it_tmp >= it; --it_tmp)
            std::swap(*it_tmp, *(it_tmp + 1));

        *it = T{std::forward<Args>(args)...};
        ++count_pushed;
    }
    T pop_front() {
        if (count_pushed == 0)
            throw runtime_error("Invalid access.");

        typename monotonic_array::value_type tmp = std::move(this->buffer[0]);

        for (std::size_t i = 0; i < count_pushed - 1; ++i)
            this->buffer[i] = std::move(this->buffer[i + 1]);

        --count_pushed;

        return tmp;
    }

public:
    T       &operator[](std::size_t it) { return this->buffer[it]; }
    const T &operator[](std::size_t it) const { return this->buffer[it]; }

    T &at(std::size_t it) {
        if (it >= count_pushed)
            throw runtime_error("Invalid access.");
        return this->operator[](it);
    }
    const T &at(std::size_t it) const {
        if (it >= count_pushed)
            throw runtime_error("Invalid access.");
        return this->operator[](it);
    }

private:
    std::size_t count_pushed = 0;
};
template<typename T, std::size_t _buffer_size>
class ring_buffer : public basic_array<T, _buffer_size> {
public:
    constexpr ring_buffer() = default;

public:
    template<typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push(_T &&el) {
        this->buffer[back] = std::forward<_T>(el);
        back               = (back + 1) % ring_buffer::buffer_size;

        if (count_pushed < ring_buffer::buffer_size)
            ++count_pushed;
        else
            front = (front + 1) % ring_buffer::buffer_size;
    }
    T pop() {
        if (!count_pushed)
            return {};

        typename ring_buffer::value_type tmp = std::move(this->buffer[front]);
        front                                = (front + 1) % ring_buffer::buffer_size;
        --count_pushed;
        return tmp;
    }

public:
    std::size_t                        size() const { return count_pushed; }
    ring_buffer::buffer_type::iterator lbegin() { return this->buffer.begin(); }
    ring_buffer::buffer_type::iterator lend() {
        return this->buffer.begin() + count_pushed;
    }

private:
    std::size_t back = 0, front = 0, count_pushed = 0;
};

template<typename T, std::size_t _buffer_size>
class jitter_buffer
    : private monotonic_array<std::tuple<T, std::uint16_t, bool>, _buffer_size> {
public:
    using value_type = jitter_buffer::value_type;

    static constexpr std::size_t element_index         = 0;
    static constexpr std::size_t sequence_number_index = 1;
    static constexpr std::size_t lost_index            = 2;

    using element_type         = std::tuple_element_t<element_index, value_type>;
    using sequence_number_type = std::tuple_element_t<sequence_number_index, value_type>;
    using lost_type            = std::tuple_element_t<lost_index, value_type>;

    using pop_type = std::pair<element_type, lost_type>;

public:
    constexpr jitter_buffer() = default;

public:
    template<typename _T>
        requires std::same_as<std::decay_t<_T>, element_type>
    void push(_T &&el, sequence_number_type sequence_number) {
        std::ssize_t diff =
            this->size()
                ? sequence_number - std::get<sequence_number_index>(*(this->end() - 1))
                : 1;
        if (diff < 0) {
            auto it = this->end() + diff - 1;
            if (std::abs(diff) >= this->size()) {
                pop_if_full();
                this->emplace(this->begin(), std::forward<_T>(el), sequence_number,
                              false);
            } else {
                if (std::get<lost_index>(*it))
                    std::get<element_index>(*it) = el;
                else {
                    pop_if_full();
                    this->emplace(it, std::forward<_T>(el), sequence_number, false);
                }
            }
        } else {
            pop_if_full();
            if (diff > 1) {
                T el_tmp;

                sequence_number_type current_sequence_number =
                    std::get<sequence_number_index>(*this->end());
                count_lost_elements += diff - 1;
                for (; diff > 1; --diff) {
                    ++current_sequence_number;

                    this->push_back(
                        std::make_tuple(el_tmp, current_sequence_number, true));
                    pop_if_full();
                }
            }
            this->push_back(std::make_tuple(std::forward<T>(el), sequence_number, false));
        }
        ++count_pushed_elements;
    }

    pop_type pop() {
        if (!this->size())
            return std::make_tuple(element_type{}, true);

        auto tuple_element = this->pop_front();
        return std::make_tuple(std::move(std::get<element_index>(tuple_element)),
                               std::get<lost_index>(tuple_element));
    }

public:
    std::size_t get_count_elements() const { return this->size(); }
    bool        full() const { return this->size() == this->buffer_size; }

    std::size_t get_count_pushed_elements() const { return count_pushed_elements; }
    std::size_t get_count_lost_elements() const { return count_lost_elements; }

private:
    void pop_if_full() {
        if (this->full())
            this->pop();
    }

private:
    std::size_t count_pushed_elements = 0;
    std::size_t count_lost_elements   = 0;
};

} // namespace noheap

class log_handler {
public:
    static constexpr std::size_t max_outstream_count = 2;

    enum output_type : std::size_t { flush = 0, async };

public:
    constexpr log_handler(noheap::log_impl::owner_impl::buffer_type _buffer_owner)
        : buffer_owner(_buffer_owner) {
        out_streams[0] = 1;
    }
    constexpr log_handler(noheap::log_impl::owner_impl::buffer_type _buffer_owner,
                          std::span<std::size_t>                    _out_streams)
        : buffer_owner(_buffer_owner) {
        if (out_streams.size() > max_outstream_count)
            throw noheap::runtime_error("The streams limit has been exceeded: {}.",
                                        max_outstream_count);
        for (std::size_t i = 0; i < out_streams.size(); ++i)
            out_streams[i] = _out_streams[i];
    }

public:
    template<output_type async = output_type::flush, typename... Args>
    void to_console(std::format_string<Args...> format, Args &&...args) const {
        this->log<async>(1, buffer_owner, format, std::forward<Args>(args)...);
    }

    template<output_type async = output_type::flush, typename... Args>
    void to_stream(std::size_t it_outstream, std::format_string<Args...> format,
                   Args &&...args) const {
        this->log<async>(out_streams.at(it_outstream), buffer_owner, format,
                         std::forward<Args>(args)...);
    }

    template<output_type async = output_type::flush, typename... Args>
    void to_all(std::format_string<Args...> format, Args &&...args) const {
        std::for_each(out_streams.begin(), out_streams.end(), [&](std::size_t outstream) {
            if (!outstream)
                return;
            this->log<async>(out_streams.at(outstream), buffer_owner, format,
                             std::forward<Args>(args)...);
        });
    }
    template<output_type async = output_type::flush, typename... Args>
    void to_all_with_subowner(noheap::log_impl::owner_impl::buffer_type buffer_subowner,
                              std::format_string<Args...> format, Args &&...args) const {
        std::for_each(out_streams.begin(), out_streams.end(), [&](std::size_t outstream) {
            if (!outstream)
                return;
            noheap::print_impl::buffer_type buffer;
            auto                            end_it = buffer.begin();
            end_it = std::format_to_n(end_it, noheap::print_impl::buffer_size, format,
                                      std::forward<Args>(args)...)
                         .out;

            this->log<async>(outstream, "{} {}", buffer_subowner.data(), buffer.data());
        });
    }

    template<output_type async = output_type::flush, typename... Args>
    void exception_to_all(noheap::runtime_error &excp) const {
        std::for_each(out_streams.begin(), out_streams.end(), [&](std::size_t outstream) {
            if (!outstream)
                return;
            this->log<async>(outstream, excp.get_owner(), "{}", excp.what());
        });
    }

private:
    template<output_type async, typename... Args>
    static constexpr void log(std::size_t                               outstream,
                              noheap::log_impl::owner_impl::buffer_type buffer_owner,
                              std::format_string<Args...> format, Args &&...args) {
        switch (async) {
            default:
            case output_type::flush: {
                noheap::print_impl::out_buffer(
                    noheap::log_impl::create_log_data(buffer_owner, format,
                                                      std::forward<Args>(args)...),
                    outstream);
            } break;
            case output_type::async: {
                static std::future<void> future_object;

                if (future_object.valid())
                    future_object.get();

                future_object =
                    std::async(std::launch::async, noheap::print_impl::out_buffer,
                               noheap::log_impl::create_log_data(
                                   buffer_owner, format, std::forward<Args>(args)...),
                               outstream);
            } break;
        }
    }

private:
    std::array<std::size_t, max_outstream_count> out_streams{};
    noheap::log_impl::owner_impl::buffer_type    buffer_owner;
};

#endif
