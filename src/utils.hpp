#ifndef UTILS_HPP
#define UTILS_HPP

#include <unistd.h>

#include <algorithm>
#include <array>
#include <boost/stacktrace.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <format>
#include <functional>
#include <future>
#include <span>
#include <string_view>
#include <utility>

inline std::size_t get_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

class noncopyable {
public:
    noncopyable()                               = default;
    ~noncopyable()                              = default;
    noncopyable(noncopyable &&)                 = default;
    noncopyable &operator=(noncopyable &&)      = default;
    noncopyable(const noncopyable &)            = delete;
    noncopyable &operator=(const noncopyable &) = delete;
};
class nonmoveable {
public:
    nonmoveable()                               = default;
    ~nonmoveable()                              = default;
    nonmoveable(nonmoveable &&)                 = delete;
    nonmoveable &operator=(nonmoveable &&)      = delete;
    nonmoveable(const nonmoveable &)            = default;
    nonmoveable &operator=(const nonmoveable &) = default;
};

namespace noheap {
using ssize_t = std::make_signed_t<std::size_t>;

constexpr std::size_t output_buffer_size = 1024;

using byte  = std::int_least8_t;
using ubyte = std::uint_least8_t;
using rbyte = std::byte;

template<typename T>
concept Byte =
    (std::same_as<std::decay_t<T>, byte> || std::same_as<std::decay_t<T>, ubyte>
     || std::same_as<std::decay_t<T>, rbyte>);
template<typename T>
concept Char = std::same_as<std::decay_t<T>, char>;
template<typename T>
concept Byte_or_char = Byte<T> || Char<T>;

template<typename T, std::size_t buffer_size>
using buffer_type = std::array<T, buffer_size>;
template<std::size_t buffer_size, Byte T = byte>
using buffer_bytes_type = buffer_type<T, buffer_size>;
template<std::size_t buffer_size>
using buffer_chars_type = buffer_type<char, buffer_size>;

template<typename T>
concept Buffer =
    std::same_as<std::decay_t<T>, buffer_type<typename std::decay_t<T>::value_type,
                                              std::tuple_size_v<std::decay_t<T>>>>;
template<Buffer T>
constexpr std::size_t buffer_size = std::tuple_size_v<std::decay_t<T>>;
template<Buffer T>
using buffer_value_type = typename std::decay_t<T>::value_type;

template<typename T>
concept Buffer_bytes =
    std::same_as<std::decay_t<T>,
                 buffer_bytes_type<buffer_size<T>, typename std::decay_t<T>::value_type>>;

template<typename T>
concept Buffer_chars = std::same_as<std::decay_t<T>, buffer_chars_type<buffer_size<T>>>;

template<typename T>
concept Buffer_bytes_or_chars = Buffer_bytes<T> || Buffer_chars<T>;

template<Buffer TReturn, typename TSource>
    requires(buffer_size<TReturn> <= sizeof(TSource))
constexpr TReturn to_buffer(TSource &&el) noexcept {
    return *reinterpret_cast<std::remove_reference_t<TReturn> *>(&el);
}
template<std::size_t output_size, std::size_t begin, Buffer TSource>
    requires(begin + output_size <= buffer_size<TSource>)
constexpr noheap::buffer_type<typename std::decay_t<TSource>::value_type, output_size>
    clip_buffer(TSource &&buffer) noexcept {
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
template<Byte_or_char T, std::size_t size>
constexpr buffer_type<T, size> to_buffer(std::string_view string) noexcept {
    decltype(to_buffer<T, size>(string)) buffer_bytes{};

    for (std::size_t i = 0; i < buffer_bytes.size() && i < string.size(); ++i)
        buffer_bytes[i] = string[i];

    return buffer_bytes;
}

template<Buffer_bytes TSource>
constexpr buffer_chars_type<buffer_size<TSource> * 2>
    hex_encode(TSource &&buffer) noexcept {
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
constexpr buffer_type<ubyte, buffer_size<TSource> / 2>
    hex_decode(TSource &&buffer) noexcept {
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
constexpr TReturn represent_bytes(TSource &&buffer) noexcept {
    return *reinterpret_cast<std::decay_t<TReturn> *>(&buffer);
}

template<Buffer TSource_one, Buffer TSource_two, typename Func>
    requires std::same_as<buffer_value_type<TSource_one>, buffer_value_type<TSource_two>>
void transform_buffers(TSource_one &&buffer_one, TSource_two &&buffer_two, Func &&func) {
    std::transform(buffer_one.begin(),
                   buffer_one.begin()
                       + std::clamp<std::size_t>(buffer_one.size(), buffer_one.size(),
                                                 buffer_two.size()),
                   buffer_two.begin(), buffer_one.begin(), func);
}

class const_error : public std::exception {
public:
    const_error(const std::string_view _what_str) noexcept : what_str(_what_str) {}

public:
    const char *what() const noexcept override { return what_str.data(); }

private:
    const std::string_view what_str;
};
class const_runtime_error : public const_error {
public:
    using const_error::const_error;
};
class const_logic_error : public const_error {
public:
    using const_error::const_error;
};

class print_impl final {
public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_type                        = buffer_chars_type<buffer_size>;

public:
    template<char end_ch, typename... Args>
    static void out(std::format_string<Args...> format, Args &&...args) {
        buffer_type buffer;
        std::size_t out_size =
            std::formatted_size(format, std::forward<Args>(args)...) + 1;
        if (noheap::buffer_size<decltype(buffer)> < out_size)
            throw const_runtime_error("Formattable output has large size.");

        auto end_it = std::format_to_n(buffer.begin(), buffer_size, format,
                                       std::forward<Args>(args)...);
        *end_it.out = end_ch;

        out_buffer(buffer, out_size);
    }

    static void out_buffer(const buffer_type &buffer, std::size_t size,
                           std::size_t outstream = 1) noexcept {
        ::write(outstream, buffer.data(), size);
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
    struct message_type final {
        static constexpr std::size_t buffer_size = 10;
        using buffer_type                        = buffer_chars_type<buffer_size>;
    };

    template<typename... Args>
    static constexpr print_impl::buffer_type
        create_log_data(std::size_t &out_size, message_type::buffer_type message_type,
                        std::format_string<Args...> format, Args &&...args) {
        print_impl::buffer_type buffer;
        auto                    end_it = buffer.begin();
        auto                    time_now =
            std::chrono::current_zone()->to_local(std::chrono::system_clock::now());

        out_size = 0;
        out_size += std::formatted_size("[{}] {:<10} ", time_now, message_type.data());
        end_it = std::format_to_n(buffer.begin(), out_size, "[{}] {:<10} ", time_now,
                                  message_type.data())
                     .out;

        out_size += std::formatted_size(format, std::forward<Args>(args)...) + 1;
        if (noheap::buffer_size<decltype(buffer)> < out_size)
            throw const_runtime_error("Formattable output has large size.");

        end_it =
            std::format_to_n(end_it, out_size, format, std::forward<Args>(args)...).out;
        *end_it = '\n';
        return buffer;
    }
};

class error : public std::exception {
public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_type                        = buffer_chars_type<buffer_size>;

public:
    error()                  = default;
    error(const error &excp) = default;
    error(buffer_type &&_buffer) noexcept : buffer(_buffer) { set = true; }

    template<typename... Args>
    error(std::format_string<Args...> format, Args &&...args) {
        if (format.get().size()) {
            auto end_it = std::format_to_n(buffer.begin(), buffer_size, format,
                                           std::forward<Args>(args)...);
            *end_it.out = '\0';
        }

        set = true;
    }

    ~error() override = default;

public:
    const char *what() const noexcept override { return buffer.data(); }
    bool        is_set() const noexcept { return set; }
    bool        owner_is_set() const noexcept { return owner_set; }

private:
    buffer_type buffer{};
    bool        set{};
    bool        owner_set{};
};

class runtime_error : public error {
public:
    using error::error;
};

class logic_error : public error {
public:
    using error::error;
};

template<typename T>
concept Derived_from_error = std::derived_from<T, error>;

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
    buffer_type buffer;
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
        if (offset + area_size > this->buffer_size())
            throw logic_error("Pseudoheap is full. Last request: {}", area_size);

        typename basic_array_type::value_type *ptr = this->array.buffer.data() + offset;
        offset += area_size;
        return reinterpret_cast<T>(ptr);
    }

    void free_last(std::size_t area_size) {
        if (area_size > offset)
            throw logic_error("Invalid area size: {}", area_size);

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
            throw logic_error("Buffer overflow.");

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
            throw logic_error("Buffer overflow.");
        this->buffer[count_pushed++] = std::forward<_T>(el);
    }
    template<typename... Args>
        requires std::constructible_from<std::decay_t<T>, Args...>
    void emplace(iterator it, Args &&...args) {
        if (count_pushed == this->buffer_size())
            throw logic_error("Buffer overflow.");
        else if (it >= this->buffer.end())
            throw logic_error("Invalid access.");

        for (auto it_tmp = this->end() - 2; it_tmp >= it; --it_tmp)
            std::swap(*it_tmp, *(it_tmp + 1));

        *it = T(std::forward<Args>(args)...);
        ++count_pushed;
    }
    iterator erase(iterator it) {
        if (!count_pushed || it >= this->buffer.end())
            throw logic_error("Invalid access.");

        for (auto it_tmp = it; it_tmp < this->end(); ++it_tmp)
            std::swap(*it_tmp, *(it_tmp + 1));

        --count_pushed;

        return this->end() + 1;
    }
    T pop_back() {
        if (count_pushed == 0)
            throw logic_error("Invalid access.");

        return this->buffer[--count_pushed];
    }
    T pop_front() {
        if (count_pushed == 0)
            throw logic_error("Invalid access.");

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
            throw logic_error("Invalid access.");
        return this->operator[](it);
    }
    const T &at(std::size_t it) const {
        if (it >= count_pushed)
            throw logic_error("Invalid access.");
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
        using value_type      = T;
        using difference_type = std::ptrdiff_t;
        using pointer         = value_type *;
        using reference       = value_type &;

    public:
        explicit iterator(monotonic_placement_new_array<T, _buffer_size> &_array,
                          std::size_t                                     _it)
            : array(_array), it(_it) {}
        decltype(auto) operator++() {
            ++it;
            return *this;
        }
        decltype(auto) operator--() {
            --it;
            return *this;
        }
        auto operator++(int) {
            iterator retval = *this;
            ++(*this);
            return retval;
        }
        decltype(auto) operator+(std::size_t _it) const {
            return iterator(array, it + _it);
        }
        decltype(auto) operator-(std::size_t _it) const {
            return iterator(array, it - _it);
        }
        auto operator<=>(const iterator &other) const { return it <=> other.it; }
        bool operator==(const iterator &other) const { return it == other.it; }
        decltype(auto) operator*() { return array.get().at(it); }
        decltype(auto) operator->() { return &array.get().at(it); }

    private:
        std::reference_wrapper<monotonic_placement_new_array<T, _buffer_size>> array;
        std::size_t                                                            it;
    };

    iterator begin() { return iterator(*this, 0); }
    iterator end() { return iterator(*this, this->size()); }

public:
    template<typename... Args>
    void emplace(iterator it, Args &&...args) {
        if (count_pushed == buffer_size())
            throw logic_error("Buffer overflow.");
        if (it > end())
            throw logic_error("Invalid access.");

        this->template malloc<rbyte *>(sizeof(T));

        // Copies bytes to it+1
        auto        data      = this->template data<noheap::ubyte *>();
        auto        bytes_end = data + sizeof(T) * count_pushed;
        auto        bytes_it  = data + sizeof(T) * std::distance(this->begin(), it);
        std::size_t it_tmp    = count_pushed;
        for (auto bytes_it_tmp = bytes_end - 1; bytes_it_tmp >= bytes_it;
             --bytes_it_tmp) {
            std::size_t distance_mod = std::distance(data, bytes_it_tmp) % sizeof(T);
            if (distance_mod == 0)
                --it_tmp;
            std::swap(*bytes_it_tmp, *(data + distance_mod + sizeof(T) * it_tmp));
        }

        ++count_pushed;
        ::new (reinterpret_cast<void *>(&(*it))) T(std::forward<Args>(args)...);
    }

    template<typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push_back(_T &&el) {
        if (count_pushed == buffer_size())
            throw logic_error("Buffer overflow.");

        decltype(auto) storage_p = this->template malloc<rbyte *>(sizeof(T));
        ::new (reinterpret_cast<void *>(storage_p)) T(std::forward<_T>(el));
        ++count_pushed;
    }
    template<typename... Args>
    void emplace_back(Args &&...args) {
        emplace(size() ? end() : begin(), std::forward<Args>(args)...);
    }

    void erase(iterator it) {
        if (!count_pushed || it == end())
            throw logic_error("Invalid access.");

        it->~T();

        // Copies bytes to it-1
        auto        data      = this->template data<noheap::ubyte *>();
        auto        bytes_end = data + sizeof(T) * count_pushed;
        auto        bytes_it  = data + sizeof(T) * std::distance(this->begin(), it);
        std::size_t it_tmp    = std::distance(this->begin(), it);
        for (auto bytes_it_tmp = bytes_it; bytes_it_tmp < bytes_end; ++bytes_it_tmp) {
            std::size_t distance_mod = std::distance(data, bytes_it_tmp) % sizeof(T);
            if (distance_mod == 0)
                ++it_tmp;
            std::swap(*bytes_it_tmp, *(data + distance_mod + sizeof(T) * it_tmp));
        }

        this->free_last(sizeof(T));
        --count_pushed;
    }

    std::size_t size() const { return count_pushed; }
    std::size_t buffer_size() const { return _buffer_size; }

public:
    template<typename _T>
    decltype(auto) at(this _T &&_this, std::size_t it) {
        if (it >= _this.size())
            throw logic_error("Invalid access.");

        return _this.operator[](it);
    }

    template<typename _T>
    decltype(auto) operator[](this _T &&_this, std::size_t it) {
        return *(_this.template data<T *>() + it);
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
        ssize_t diff =
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
                    std::get<sequence_number_index>(*(this->end() - 1));
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

template<std::size_t size>
struct std::formatter<noheap::buffer_chars_type<size>> {
public:
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }
    auto format(const noheap::buffer_chars_type<size> &s, std::format_context &ctx) const {
        return std::format_to(ctx.out(), "{}", s.data());
    }
};

[[noreturn]] static void abort_invalid_state(std::size_t depth = 1) {
    noheap::println("Invalid state.\n{}", boost::stacktrace::to_string(
                                              boost::stacktrace::stacktrace(depth, 5)));
    std::abort();
}

class log_handler : noncopyable {
public:
    static constexpr std::size_t max_outstream_count = 3;

    class proxy : noncopyable {
        friend class log_handler;

    public:
        struct owner_impl final {
            static constexpr std::size_t buffer_size = 16;
            using buffer_type = noheap::buffer_chars_type<buffer_size>;
        };
        struct dynamic_owner_impl final {
            static constexpr std::size_t buffer_size = 32;
            using buffer_type = noheap::buffer_chars_type<buffer_size>;
        };

    private:
        proxy(const log_handler &_handler, owner_impl::buffer_type _owner) noexcept
            : handler(_handler), owner(_owner) {}

    public:
        proxy(proxy &&other) noexcept : proxy(other.handler, other.owner) {}
        proxy &operator=(proxy &&) = delete;

    public:
        decltype(auto) get_handler() const { return handler; }

        template<typename... Args>
        void info(std::format_string<Args...> format, Args &&...args) const {
            handler.info("{}",
                         create_log_data(format, std::forward<Args>(args)...).data());
        }
        template<typename... Args>
        void warning(std::format_string<Args...> format, Args &&...args) const {
            handler.warning("{}",
                            create_log_data(format, std::forward<Args>(args)...).data());
        }
        template<typename... Args>
        void error(std::format_string<Args...> format, Args &&...args) const {
            handler.error("{}",
                          create_log_data(format, std::forward<Args>(args)...).data());
        }
        template<noheap::Derived_from_error TExcp, typename... Args>
        [[noreturn]] void throw_and_log(TExcp &&excp, std::format_string<Args...> format,
                                        Args &&...args) const {
            handler.throw_and_log(
                std::forward<TExcp>(excp), "{}",
                create_log_data(format, std::forward<Args>(args)...).data());
        }
        template<typename... Args>
        void exit(std::format_string<Args...> format, Args &&...args) const {
            handler.exit("{}",
                         create_log_data(format, std::forward<Args>(args)...).data());
        }
        template<typename... Args>
        void abort(std::format_string<Args...> format, Args &&...args) const {
            handler.abort("{}",
                          create_log_data(format, std::forward<Args>(args)...).data());
        }

    private:
        template<typename... Args>
        noheap::print_impl::buffer_type
            create_log_data(std::format_string<Args...> format, Args &&...args) const {
            noheap::print_impl::buffer_type buffer_out{}, buffer_tmp{};
            auto                            end_it = buffer_tmp.begin();
            end_it = std::format_to_n(end_it, noheap::print_impl::buffer_size, format,
                                      std::forward<Args>(args)...)
                         .out;

            end_it = buffer_out.begin();
            std::format_to_n(end_it, noheap::print_impl::buffer_size, "{:<16} {}",
                             owner.data(), buffer_tmp.data());
            return buffer_out;
        }

    private:
        const log_handler      &handler;
        owner_impl::buffer_type owner;
    };

public:
    log_handler(std::string_view file_name) {
        fd_log = open(file_name.data(), O_CREAT | O_WRONLY | O_TRUNC,
                      S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        if (fd_log == -1)
            throw noheap::const_error("Failed to open log file.");
    }
    ~log_handler() noexcept(false) {
        if (close(fd_log) == -1)
            throw noheap::const_error("Failed to close log file.");
    }

public:
    proxy create_proxy(proxy::owner_impl::buffer_type owner) const noexcept {
        return proxy(*this, owner);
    }

    template<typename... Args>
    void info(std::format_string<Args...> format, Args &&...args) const {
        this->log(1, {"INFO"}, format, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void warning(std::format_string<Args...> format, Args &&...args) const {
        this->log(2, {"WARNING"}, format, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void error(std::format_string<Args...> format, Args &&...args) const {
        this->log(2, {"ERROR"}, format, std::forward<Args>(args)...);
    }
    template<noheap::Derived_from_error TExcp, typename... Args>
    [[noreturn]] void throw_and_log(TExcp &&excp, std::format_string<Args...> format,
                                    Args &&...args) const {
        error(format, std::forward<Args>(args)...);
        throw std::forward<TExcp>(excp);
    }
    template<typename... Args>
    void exit(std::format_string<Args...> format, Args &&...args) const {
        this->fatal(format, std::forward<Args>(args)...);
        std::exit(0);
    }
    template<typename... Args>
    void abort(std::format_string<Args...> format, Args &&...args) const {
        this->fatal(format, std::forward<Args>(args)...);
        std::abort();
    }

private:
    template<typename... Args>
    void fatal(std::format_string<Args...> format, Args &&...args) const {
        this->log(1, {"FATAL"}, format, std::forward<Args>(args)...);
    }
    template<typename... Args>
    void log(std::size_t                                 outstream,
             noheap::log_impl::message_type::buffer_type message_type,
             std::format_string<Args...>                 format, Args &&...args) const {
        std::lock_guard<decltype(m)> lock(m);
        std::size_t                  out_size;
        auto buffer = noheap::log_impl::create_log_data(out_size, message_type, format,
                                                        std::forward<Args>(args)...);

        noheap::print_impl::out_buffer(buffer, out_size, outstream);
        noheap::print_impl::out_buffer(buffer, out_size, fd_log);
    }

private:
    int                fd_log;
    mutable std::mutex m;
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
                throw noheap::logic_error("Invalid status of future object.");
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
