#ifndef UTILS_HPP
#define UTILS_HPP

#include <unistd.h>

#include <thread>
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

#ifdef _LIBCPP_VERSION

class jthread{
  public:
    template<typename Func, typename... Args>
      requires std::regular_invocable<Func, Args...> 
    explicit jthread(Func &&func, Args&&... args) : t(std::forward<Func>(func), std::forward<Args>(args)...){
    }
    ~jthread(){
	if(t.joinable())
	    t.join();
    }

  private:
    std::thread t;
};

#endif

}

constexpr std::size_t get_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

namespace noheap {

static constexpr std::size_t output_buffer_size = 512;

template <std::size_t buffer_size, typename T = char>
using buffer_bytes_t = std::array<T, buffer_size>;

class print_impl final {
  public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_t = buffer_bytes_t<buffer_size>;

  public:
    template <char end_ch, typename... Args>
    static void out(std::format_string<Args...> format, Args &&...args) {
        buffer_t buffer{};
        auto end_it = std::format_to_n(buffer.begin(), buffer_size, format,
                                       std::forward<Args>(args)...);
        *end_it.out = end_ch;

        out_buffer(std::move(buffer));
    }

    static void out_buffer(buffer_t &&buffer, std::size_t outstream = 1) {
        ::write(outstream, buffer.data(), buffer_size);
    }
};

template <typename... Args>
constexpr void print(std::format_string<Args...> format, Args &&...args) {
    print_impl::out<'\0'>(format, std::forward<Args>(args)...);
}
template <typename... Args>
constexpr void println(std::format_string<Args...> format, Args &&...args) {
    print_impl::out<'\n'>(format, std::forward<Args>(args)...);
}

class log_impl final {
  public:
    struct owner_impl final {
        static constexpr std::size_t buffer_size = 24;
        using buffer_t = buffer_bytes_t<buffer_size>;
    };

    static consteval owner_impl::buffer_t create_owner(std::string_view owner) {
        owner_impl::buffer_t buffer_owner{};

        for (std::size_t i = 0; i < owner_impl::buffer_size; ++i) {
            if (i == owner.size())
                break;
            buffer_owner[i] = owner[i];
        }

        return buffer_owner;
    };

    template <typename... Args>
    static constexpr print_impl::buffer_t
    create_log_data(owner_impl::buffer_t buffer_owner,
                    std::format_string<Args...> format, Args &&...args) {
        print_impl::buffer_t buffer{};
        auto end_it = buffer.begin();

        if (buffer_owner[0] != '\0') {
            std::transform(buffer_owner.begin(), buffer_owner.end(),
                           buffer_owner.begin(),
                           [](unsigned char ch) { return std::toupper(ch); });
            end_it = std::format_to_n(buffer.begin(), print_impl::buffer_size,
                                      "[{}]: ", buffer_owner.data())
                         .out;
        }

        end_it = std::format_to_n(end_it,
                                  std::abs(std::distance(buffer.end(), end_it)),
                                  format, std::forward<Args>(args)...)
                     .out;
        *end_it = '\n';
        return buffer;
    }
};

class runtime_error final : public std::exception {
  public:
    static constexpr std::size_t buffer_size = output_buffer_size;
    using buffer_t = buffer_bytes_t<buffer_size>;

  public:
    template <typename... Args>
    runtime_error(std::format_string<Args...> format, Args &&...args) {
        if (format.get().size()) {
            auto end_it = std::format_to_n(buffer.begin(), buffer_size, format,
                                           std::forward<Args>(args)...);
            *end_it.out = '\0';
        }
    }
    template <typename... Args>
    runtime_error(noheap::log_impl::owner_impl::buffer_t _buffer_owner,
                  std::format_string<Args...> format, Args &&...args)
        : runtime_error(format, std::forward<Args>(args)...) {
        buffer_owner = _buffer_owner;
    }
    runtime_error() = default;
    runtime_error(buffer_t &&_buffer) : buffer(std::move(_buffer)) {}
    runtime_error(const runtime_error &excp) {
        buffer = excp.buffer;
        set_owner(excp.buffer_owner);
    }
    ~runtime_error() override = default;

  public:
    void set_owner(log_impl::owner_impl::buffer_t _buffer_owner) {
        buffer_owner = _buffer_owner;
        owner_set = true;
    }

  public:
    const char *what() const noexcept override { return buffer.data(); }
    log_impl::owner_impl::buffer_t get_owner() const noexcept {
        return buffer_owner;
    }
    bool has_setting_owner() const noexcept { return owner_set; }

  private:
    buffer_t buffer{};
    log_impl::owner_impl::buffer_t buffer_owner{};

    bool owner_set;
};

template <typename T, std::size_t _buffer_size> class monotonic_array final {
  public:
    static constexpr std::size_t buffer_size = _buffer_size;

    using value_type = T;
    using buffer_type = std::array<value_type, buffer_size>;

  public:
    constexpr monotonic_array() : buffer{} {}
    monotonic_array(monotonic_array &&array) {
        std::swap(buffer, array.buffer);
        count_pushed = array.count_pushed;
    }

  public:
    std::size_t size() const { return count_pushed; }
    buffer_type::iterator begin() { return buffer.begin(); }
    buffer_type::iterator end() { return buffer.begin() + count_pushed; }
    buffer_type::iterator bend() { return buffer.end(); }
    buffer_type::const_iterator begin() const { return buffer.cbegin(); }
    buffer_type::const_iterator end() const {
        return buffer.cbegin() + count_pushed;
    }
    buffer_type::const_iterator bend() const { return buffer.end(); }
    buffer_type::const_iterator cbegin() const { return buffer.begin(); }
    buffer_type::const_iterator cend() const {
        return buffer.begin() + count_pushed;
    }
    buffer_type::const_iterator cbend() const { return buffer.end(); }

  public:
    template <typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push_back(_T &&el) {
        if (count_pushed == buffer_size)
            throw runtime_error("Buffer overflow.");
        buffer[count_pushed++] = std::forward<_T>(el);
    }
    T pop_front() {
        if (count_pushed == 0)
            throw runtime_error("Invalid access.");

        value_type tmp = std::move(buffer[0]);

        for (std::size_t i = 0; i < count_pushed - 1; ++i)
            buffer[i] = std::move(buffer[i + 1]);

        --count_pushed;

        return tmp;
    }

  public:
    T &operator[](std::size_t it) { return buffer[it]; }
    const T &operator[](std::size_t it) const { return buffer[it]; }

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
    buffer_type buffer;
    std::size_t count_pushed = 0;
};
template <typename T, std::size_t _max_count_elements> class ring_buffer {
  public:
    static constexpr std::size_t max_count_elements = _max_count_elements;

    using value_type = T;
    using buffer_type = std::array<T, max_count_elements>;

  public:
    constexpr ring_buffer() = default;

  public:
    template <typename _T>
        requires std::same_as<std::decay_t<_T>, std::decay_t<T>>
    void push(_T &&el) {
        buffer[back] = std::forward<T>(el);
        back = (back + 1) % max_count_elements;

        if (count_pushed < max_count_elements)
            ++count_pushed;
        else
            front = (front + 1) % max_count_elements;
    }
    T pop() {
        if (!count_pushed)
            return {};

        value_type tmp = std::move(buffer[front]);
        front = (front + 1) % max_count_elements;
        --count_pushed;
        return tmp;
    }

  public:
    std::size_t size() const { return count_pushed; }
    buffer_type::iterator lbegin() { return buffer.begin(); }
    buffer_type::iterator lend() { return buffer.begin() + count_pushed; }

  private:
    buffer_type buffer{};
    std::size_t back = 0, front = 0, count_pushed = 0;
};

namespace pmr {

static constexpr std::size_t default_buffer_size = 1024;

template <typename T>
concept buffer_resouce_static =
    std::derived_from<T, std::pmr::memory_resource> &&
    std::same_as<T, decltype(T{std::declval<char *>(), std::size_t{}})>;

static constexpr std::size_t
calculate_number_bytes_for_alignment(std::ptrdiff_t ptr,
                                     std::size_t alignment) {
    return (ptr + alignment - 1) & ~(alignment - 1) - ptr;
}

class monotonic_buffer_resource_static final
    : public std::pmr::memory_resource {
  public:
    monotonic_buffer_resource_static(char *_buffer, std::size_t _buffer_size)
        : buffer(_buffer), buffer_size(_buffer_size) {
        if (buffer == nullptr)
            throw runtime_error("Invalid buffer.");
    }

  protected:
    void *do_allocate(std::size_t bytes, std::size_t alignment) override {
        offset += bytes + calculate_number_bytes_for_alignment(
                              reinterpret_cast<std::size_t>(buffer + offset),
                              alignment);

        if (offset >= buffer_size)
            throw runtime_error("The allocator buffer is full: {} bytes were "
                                "allocated. Required to allocate: {} bytes.",
                                buffer_size, bytes);
        return buffer + offset - bytes;
    }
    void do_deallocate(void *p, std::size_t bytes,
                       std::size_t alignment) override {}
    bool do_is_equal(
        const std::pmr::memory_resource &other) const noexcept override {
        try {
            if (dynamic_cast<decltype(this)>(&other)->buffer == this->buffer)
                return true;
        } catch (...) {
        }
        return false;
    }

  private:
    char *buffer;
    std::size_t buffer_size, offset = 0;
};

template <std::size_t _max_count_areas>
class synchronized_pool_resource_static final
    : public std::pmr::memory_resource {
  public:
    static constexpr std::size_t max_count_areas = _max_count_areas;

  public:
    synchronized_pool_resource_static(char *_buffer, std::size_t _buffer_size)
        : buffer(_buffer), buffer_size(_buffer_size),
          area_size(buffer_size / max_count_areas) {
        if (buffer == nullptr || area_size == 0)
            throw runtime_error("Invalid buffer.");
    }

  protected:
    void *do_allocate(std::size_t bytes, std::size_t alignment) override {
        std::lock_guard<std::mutex> lock(m);

        std::size_t area_it = 0;
        std::size_t free_bytes = 0;
        for (std::size_t i = 0; i < areas.size(); ++i) {
            if (free_bytes >= bytes)
                break;

            if (!areas[i]) {
                if (!free_bytes)
                    area_it = i;
                free_bytes += area_size;
            } else
                free_bytes = 0;
        }
        if (free_bytes < bytes)
            throw runtime_error(
                "The allocator buffer is full: {} "
                "bytes were allocated. Required to allocate: {} bytes.",
                buffer_size, bytes);

        std::for_each(areas.begin() + area_it,
                      areas.begin() +
                          (area_it * area_size + free_bytes) / area_size,
                      [](bool &area_busy) { area_busy = true; });

        buffer += area_it * area_size;
        return buffer + calculate_number_bytes_for_alignment(
                            reinterpret_cast<std::size_t>(buffer), alignment);
    }
    void do_deallocate(void *area, std::size_t bytes,
                       std::size_t alignment) override {
        std::lock_guard<std::mutex> lock(m);
        std::size_t area_it =
            (reinterpret_cast<std::size_t>(area) -
             reinterpret_cast<std::size_t>(buffer - alignment)) /
            area_size;
        while (area_it < max_count_areas && bytes > 0) {
            areas[area_it++] = true;
            bytes -= area_size;
        }
    }
    bool do_is_equal(
        const std::pmr::memory_resource &other) const noexcept override {
        try {
            if (dynamic_cast<decltype(this)>(&other)->buffer == this->buffer)
                return true;
        } catch (...) {
        }
        return false;
    }

  private:
    char *buffer;
    const std::size_t buffer_size, area_size;

    std::array<bool, max_count_areas> areas{};
    std::mutex m;
};

template <typename TContainer, std::size_t _buffer_size,
          buffer_resouce_static _buffer_resource_t>
struct basic_container {
    static constexpr std::size_t buffer_size = _buffer_size;

    using container_t = TContainer;
    using buffer_resource_t = _buffer_resource_t;

  public:
    container_t &operator*() { return data; }
    const container_t &operator*() const { return data; }
    container_t *operator->() { return &data; }
    const container_t *operator->() const { return &data; }

  private:
    buffer_bytes_t<buffer_size> buffer{};
    buffer_resource_t buffer_r{buffer.data(), buffer_size};
    std::pmr::polymorphic_allocator<typename TContainer::value_type> allocator{
        &buffer_r};

  private:
    TContainer data{allocator};
};

template <typename T, std::size_t _buffer_size = default_buffer_size,
          buffer_resouce_static _buffer_resource_t =
              monotonic_buffer_resource_static>
struct vector : public basic_container<std::pmr::vector<T>, _buffer_size,
                                       _buffer_resource_t> {};
template <typename T, std::size_t _buffer_size = default_buffer_size,
          buffer_resouce_static _buffer_resource_t =
              synchronized_pool_resource_static<_buffer_size / sizeof(T)>>
struct deque : public basic_container<std::pmr::deque<T>, _buffer_size,
                                      _buffer_resource_t> {};

template <typename T, std::size_t max_count>
using queue = std::queue<T, deque<T, max_count>>;

} // namespace pmr

} // namespace noheap

class log_handler {
  public:
    static constexpr std::size_t max_outstream_count = 2;

    enum output_type : std::size_t { flush = 0, async };

  public:
    constexpr log_handler(noheap::log_impl::owner_impl::buffer_t _buffer_owner)
        : buffer_owner(_buffer_owner) {
        outstreams[0] = 1;
    }
    constexpr log_handler(noheap::log_impl::owner_impl::buffer_t _buffer_owner,
                          std::span<std::size_t> _outstreams)
        : buffer_owner(_buffer_owner) {
        if (outstreams.size() > max_outstream_count)
            throw noheap::runtime_error(
                "The streams limit has been exceeded: {}.",
                max_outstream_count);
        for (std::size_t i = 0; i < outstreams.size(); ++i)
            outstreams[i] = _outstreams[i];
    }

  public:
    template <output_type async = output_type::flush, typename... Args>
    void to_console(std::format_string<Args...> format, Args &&...args) const {
        this->log<async>(1, buffer_owner, format, std::forward<Args>(args)...);
    }

    template <output_type async = output_type::flush, typename... Args>
    void to_stream(std::size_t it_outstream, std::format_string<Args...> format,
                   Args &&...args) const {
        this->log<async>(outstreams.at(it_outstream), buffer_owner, format,
                         std::forward<Args>(args)...);
    }

    template <output_type async = output_type::flush, typename... Args>
    void to_all(std::format_string<Args...> format, Args &&...args) const {
        std::for_each(
            outstreams.begin(), outstreams.end(), [&](std::size_t outstream) {
                if (!outstream)
                    return;
                this->log<async>(outstreams.at(outstream), buffer_owner, format,
                                 std::forward<Args>(args)...);
            });
    }
    template <output_type async = output_type::flush, typename... Args>
    void
    to_all_with_subowner(noheap::log_impl::owner_impl::buffer_t buffer_subowner,
                         std::format_string<Args...> format,
                         Args &&...args) const {
        std::for_each(
            outstreams.begin(), outstreams.end(), [&](std::size_t outstream) {
                if (!outstream)
                    return;
                noheap::print_impl::buffer_t buffer;
                auto end_it = buffer.begin();
                end_it =
                    std::format_to_n(end_it, noheap::print_impl::buffer_size,
                                     format, std::forward<Args>(args)...)
                        .out;

                this->log<async>(outstream, "{} {}", buffer_subowner.data(),
                                 buffer.data());
            });
    }

    template <output_type async = output_type::flush, typename... Args>
    void exception_to_all(noheap::runtime_error &excp) const {
        std::for_each(outstreams.begin(), outstreams.end(),
                      [&](std::size_t outstream) {
                          if (!outstream)
                              return;
                          this->log<async>(outstream, excp.get_owner(), "{}",
                                           excp.what());
                      });
    }

  private:
    template <output_type async, typename... Args>
    static constexpr void
    log(std::size_t outstream,
        noheap::log_impl::owner_impl::buffer_t buffer_owner,
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

            future_object = std::async(
                std::launch::async, noheap::print_impl::out_buffer,
                noheap::log_impl::create_log_data(buffer_owner, format,
                                                  std::forward<Args>(args)...),
                outstream);
        } break;
        }
    }

  private:
    std::array<std::size_t, max_outstream_count> outstreams{};
    noheap::log_impl::owner_impl::buffer_t buffer_owner;
};

#endif
