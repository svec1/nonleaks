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
#include <random>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace std {
using ssize_t = std::make_signed_t<std::size_t>;
} // namespace std

std::size_t get_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

namespace noheap {

constexpr std::size_t output_buffer_size = 1024;

using byte  = std::int_least8_t;
using ubyte = std::uint_least8_t;
using rbyte = std::byte;

template<typename T>
concept Byte =
    (std::same_as<T, byte> || std::same_as<T, ubyte> || std::same_as<T, rbyte>);

template<typename T, std::size_t buffer_size>
using buffer_type = std::array<T, buffer_size>;
template<std::size_t buffer_size, Byte T = byte>
using buffer_bytes_type = buffer_type<T, buffer_size>;
template<std::size_t buffer_size>
using buffer_chars_type = buffer_type<char, buffer_size>;

template<typename T>
concept Buffer =
    std::same_as<std::decay_t<T>, std::array<typename std::decay_t<T>::value_type,
                                             std::tuple_size_v<std::decay_t<T>>>>;
template<Buffer T>
constexpr std::size_t buffer_size = std::tuple_size_v<std::decay_t<T>>;

template<typename T>
concept Buffer_chars = std::same_as<std::decay_t<T>, buffer_chars_type<buffer_size<T>>>;

template<typename T>
concept Buffer_bytes =
    std::same_as<std::decay_t<T>,
                 buffer_bytes_type<buffer_size<T>, typename std::decay_t<T>::value_type>>;

template<Buffer TReturn, typename TSource>
    requires(buffer_size<TReturn> <= buffer_size<TSource>)
constexpr TReturn to_buffer(TSource &&el) {
    return *reinterpret_cast<std::remove_reference_t<TReturn> *>(&el);
}
template<std::size_t output_size, std::size_t begin, Buffer TSource>
    requires(begin + output_size <= buffer_size<TSource>)
constexpr noheap::buffer_type<typename std::decay_t<TSource>::value_type, output_size>
    clip_buffer(TSource &&buffer) {
    return *(reinterpret_cast<decltype(clip_buffer<output_size, begin, TSource>(
                 std::forward<TSource>(buffer))) *>(buffer.begin() + begin));
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

template<Buffer_bytes TSource>
constexpr buffer_chars_type<buffer_size<TSource> * 2> hex_encode(TSource &&buffer) {
    decltype(hex_encode(buffer)) buffer_tmp{};

    for (std::size_t i = 0, j = 0; i < buffer_tmp.size() - 1; i += 2, ++j) {
        static constexpr auto to_hex = [](ubyte ch) {
            return "0123456789abcdef"[static_cast<std::size_t>(ch)];
        };

        buffer_tmp[i]     = to_hex(static_cast<ubyte>(buffer[j]) >> 4);
        buffer_tmp[i + 1] = to_hex(static_cast<ubyte>(buffer[j]) & 0xf);
    }

    return buffer_tmp;
}
template<Buffer_chars TSource>
    requires(buffer_size<TSource> % 2 == 0)
constexpr buffer_type<ubyte, buffer_size<TSource> / 2> hex_decode(TSource &&buffer) {
    decltype(hex_decode(buffer)) buffer_tmp{};

    for (std::size_t i = 0, j = 0; i < buffer.size() - 1; i += 2, ++j) {
        // https://lemire.me/blog/2019/04/17/parsing-short-hexadecimal-strings-efficiently/
        static constexpr auto convertone = [](ubyte ch) {
            return static_cast<ubyte>((ch & 0xf) + (9 * (ch >> 6)));
        };

        buffer_tmp[j] = (convertone(buffer[i]) << 4) | convertone(buffer[i + 1]);
    }

    return buffer_tmp;
}

template<typename TReturn, Buffer TSource>
    requires(!std::is_pointer_v<TReturn> && buffer_size<TSource> >= sizeof(TReturn))
constexpr TReturn represent_bytes(TSource &&buffer) {
    return *reinterpret_cast<std::decay_t<TReturn> *>(&buffer);
}

template<std::size_t count_bytes>
buffer_bytes_type<count_bytes, ubyte> get_random_bytes() {
    decltype(get_random_bytes<count_bytes>()) buffer_tmp;

    std::random_device rd;
    std::mt19937       gen(rd());

    std::uniform_int_distribution<ubyte> distrib(std::numeric_limits<ubyte>::min(),
                                                 std::numeric_limits<ubyte>::max());

    for (auto &it : buffer_tmp)
        it = distrib(rd);

    return buffer_tmp;
}

class print_impl final {
public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_type                        = buffer_chars_type<buffer_size>;

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
        using buffer_type                        = buffer_chars_type<buffer_size>;
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

        end_it  = std::format_to_n(end_it, std::abs(std::distance(buffer.end(), end_it)),
                                   format, std::forward<Args>(args)...)
                      .out;
        *end_it = '\n';
        return buffer;
    }
};

class runtime_error final : public std::exception {
public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_type                        = buffer_chars_type<buffer_size>;

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

template<typename T>
concept Derived_from_runtime_error = std::derived_from<T, runtime_error>;

template<std::size_t _buffer_size>
struct pseudoheap_monotonic_array;

template<typename T, std::size_t _buffer_size>
struct basic_array {
    friend struct pseudoheap_monotonic_array<_buffer_size>;

public:
    static constexpr std::size_t buffer_size() { return _buffer_size; }

    using value_type  = T;
    using buffer_type = std::array<value_type, buffer_size()>;

protected:
    buffer_type buffer{};
};

template<std::size_t _buffer_size>
struct pseudoheap_monotonic_array {
private:
    using basic_array_type = basic_array<rbyte, _buffer_size>;

public:
    pseudoheap_monotonic_array() = default;

protected:
    template<typename T>
        requires std::is_pointer<T>::value
    T malloc(std::size_t area_size) {
        if (offset + area_size >= this->buffer_size())
            throw runtime_error("Pseudoheap is full. Last request: {}", area_size);

        typename basic_array_type::value_type *ptr = this->array.buffer.data() + offset;
        offset += area_size;
        return reinterpret_cast<T>(ptr);
    }

    void free_last(std::size_t area_size) {
        if (area_size > offset)
            throw runtime_error("Invalid area size: {}", area_size);

        offset -= area_size;
    }

    template<typename T, typename _T>
        requires std::is_pointer<T>::value
    decltype(auto) data(this _T &&_this) {
        return reinterpret_cast<T>(_this.array.buffer.data());
    }

public:
    std::size_t buffer_size() const { return basic_array_type::buffer_size(); }
    std::size_t size() const { return this->buffer_size() - offset; }

private:
    basic_array_type array;
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
    using iterator       = monotonic_array::buffer_type::iterator;
    using const_iterator = monotonic_array::buffer_type::const_iterator;

public:
    constexpr monotonic_array() = default;
    monotonic_array(std::initializer_list<T> list) {
        if (list.size() >= monotonic_array::buffer_size)
            throw runtime_error("Buffer overflow.");

        for (const auto &it : list)
            this->push_back(it);
    }
    monotonic_array(const monotonic_array &array) { this->operator=(array); }
    monotonic_array(monotonic_array &&array) { this->operator=(std::move(array)); }
    monotonic_array &operator=(const monotonic_array &array) {
        std::copy(array.begin(), array.end(), this->buffer.begin());
        count_pushed = array.count_pushed;
        return *this;
    }
    monotonic_array &operator=(monotonic_array &&array) {
        std::swap(this->buffer, array.buffer);
        count_pushed = array.count_pushed;
        return *this;
    }

public:
    std::size_t    size() const { return count_pushed; }
    iterator       begin() { return this->buffer.begin(); }
    iterator       end() { return this->buffer.begin() + count_pushed; }
    iterator       bend() { return this->buffer.end(); }
    const_iterator begin() const { return this->buffer.cbegin(); }
    const_iterator end() const { return this->buffer.cbegin() + count_pushed; }
    const_iterator bend() const { return this->buffer.end(); }
    const_iterator cbegin() const { return this->buffer.begin(); }
    const_iterator cend() const { return this->buffer.begin() + count_pushed; }

public:
    template<typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push_back(_T &&el) {
        if (count_pushed == this->buffer_size())
            throw runtime_error("Buffer overflow.");
        this->buffer[count_pushed++] = std::forward<_T>(el);
    }
    template<typename... Args>
        requires std::constructible_from<std::decay_t<T>, Args...>
    void emplace(iterator it, Args &&...args) {
        if (count_pushed == this->buffer_size())
            throw runtime_error("Buffer overflow.");
        else if (it >= this->buffer.end())
            throw runtime_error("Invalid access.");

        for (auto it_tmp = this->end(); it_tmp >= it; --it_tmp)
            std::swap(*it_tmp, *(it_tmp + 1));

        *it = T{std::forward<Args>(args)...};
        ++count_pushed;
    }
    iterator erase(iterator it) {
        if (count_pushed == this->buffer_size())
            throw runtime_error("Buffer overflow.");
        else if (it >= this->buffer.end())
            throw runtime_error("Invalid access.");

        for (auto it_tmp = it; it_tmp < this->end(); ++it_tmp)
            std::swap(*it_tmp, *(it_tmp + 1));

        --count_pushed;

        return this->end() + 1;
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
    ring_buffer(std::initializer_list<T> list) {
        for (const auto &it : list)
            this->push(it);
    }

public:
    template<typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push(_T &&el) {
        this->buffer[back] = std::forward<_T>(el);
        back               = (back + 1) % this->buffer_size();

        if (count_pushed < this->buffer_size())
            ++count_pushed;
        else
            front = (front + 1) % this->buffer_size();
    }
    T pop() {
        if (!count_pushed)
            return {};

        typename ring_buffer::value_type tmp = std::move(this->buffer[front]);
        front                                = (front + 1) % this->buffer_size();
        --count_pushed;
        return tmp;
    }

public:
    std::size_t    size() const { return count_pushed; }
    decltype(auto) begin() { return this->buffer.begin(); }
    decltype(auto) end() { return this->buffer.begin() + count_pushed; }

private:
    std::size_t back = 0, front = 0, count_pushed = 0;
};

template<typename T, std::size_t _buffer_size>
class monotonic_placement_new_array
    : public pseudoheap_monotonic_array<sizeof(T) * _buffer_size> {
public:
    using value_type = T;

public:
    constexpr monotonic_placement_new_array() = default;
    monotonic_placement_new_array(std::initializer_list<T> list) {
        for (const auto &it : list)
            this->emplace_back(it);
    }

public:
    class iterator {
    public:
        explicit iterator(monotonic_placement_new_array<T, _buffer_size> &_array,
                          std::size_t                                     _it)
            : array(_array), it(_it) {}
        decltype(auto) operator++() {
            ++it;
            return *this;
        }
        auto operator++(int) {
            iterator retval = *this;
            ++(*this);
            return retval;
        }
        decltype(auto) operator==(iterator other) const { return it == other.it; }
        decltype(auto) operator!=(iterator other) const { return !(*this == other); }
        decltype(auto) operator*() const { return array.get().at(it); }

    private:
        std::reference_wrapper<monotonic_placement_new_array<T, _buffer_size>> array;
        std::size_t                                                            it;
    };

    iterator begin() { return iterator(*this, 0); }
    iterator end() { return iterator(*this, this->size()); }

public:
    template<typename _T, typename... Args>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void emplace_back(_T &&el) {
        if (count_pushed == this->buffer_size())
            throw runtime_error("Buffer overflow.");

        decltype(auto) storage_p = this->template malloc<rbyte *>(sizeof(T));
        ::new (reinterpret_cast<void *>(storage_p)) T(std::forward<_T>(el));
        ++count_pushed;
    }
    template<typename... Args>
    void emplace_back(Args &&...args) {
        if (count_pushed == this->buffer_size())
            throw runtime_error("Buffer overflow.");

        decltype(auto) storage_p = this->template malloc<rbyte *>(sizeof(T));
        ::new (reinterpret_cast<void *>(storage_p)) T(std::forward<Args>(args)...);
        ++count_pushed;
    }
    void erase_back() {
        if (count_pushed == 0)
            throw runtime_error("Invalid access.");

        this->free_last();
        --count_pushed;
    }

    std::size_t size() const { return count_pushed; }

public:
    template<typename _T>
    decltype(auto) at(this _T &&_this, std::size_t it) {
        if (it >= _this.count_pushed)
            throw runtime_error("Invalid access.");

        return _this.operator[](it);
    }

    template<typename _T>
    decltype(auto) operator[](this _T &&_this, std::size_t it) {
        return *(_this.template data<T *>() + sizeof(T) * it);
    }

private:
    std::size_t count_pushed = 0;
};

template<typename T, typename TSequence_number_type, std::size_t _buffer_size>
class jitter_buffer
    : private monotonic_array<std::tuple<T, TSequence_number_type, bool>, _buffer_size> {
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

    template<noheap::Derived_from_runtime_error TExcp = noheap::runtime_error,
             typename... Args>
    [[noreturn]] void throw_exception(std::format_string<Args...> format,
                                      Args &&...args) const {
        throw TExcp(this->buffer_owner, format, std::forward<Args>(args)...);
    }

    template<output_type                        async = output_type::flush,
             noheap::Derived_from_runtime_error TExcp>
    void exception_to_all(const TExcp &excp) const {
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

template<typename TReturn>
struct future_wrapper {
    using future_type = std::future<TReturn>;

public:
    future_wrapper() = default;

    template<typename Func>
        requires std::same_as<std::invoke_result_t<std::decay_t<Func>>, TReturn>
    future_wrapper(Func &&func)
        : future_object(std::async(std::launch::async, std::forward<Func>(func))) {}

    future_wrapper(future_type &&_future_object)
        : future_object(std::move(_future_object)) {}

    future_wrapper(future_wrapper &&other)
        : future_object(std::move(other.future_object)) {}

    future_wrapper &operator=(future_wrapper &&other) {
        future_object = std::move(other.future_object);
        return *this;
    }

public:
    TReturn get() {
        if constexpr (!std::same_as<TReturn, void>)
            return future_object.get();
        else
            future_object.get();
    }
    bool valid() const { return future_object.valid(); }

public:
    bool is_completed(std::size_t timeout_ms) const {
        if (!valid())
            return false;

        switch (std::future_status status =
                    future_object.wait_for(std::chrono::milliseconds(timeout_ms));
                status) {
            case std::future_status::ready:
                return true;
            case std::future_status::timeout:
                return false;

            case std::future_status::deferred:
            default:
                throw noheap::runtime_error("Invalid status of future object.");
        }
    }

private:
    future_type future_object;
};

template<typename Func>
future_wrapper(Func &&func) -> future_wrapper<std::invoke_result_t<std::decay_t<Func>>>;

class scope_guard {
    using callback_type = std::function<void()>;

public:
    scope_guard(callback_type &&_callback) : callback(_callback) {}
    scope_guard(scope_guard &&other) : callback(other.callback) { other.callback = {}; }
    scope_guard(const scope_guard &other) = delete;

    ~scope_guard() { callback(); }

private:
    callback_type callback;
};

#endif
