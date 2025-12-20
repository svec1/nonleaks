#ifndef UTILS_HPP
#define UTILS_HPP

#include <exception>
#include <algorithm>
#include <format>
#include <array>
#include <memory_resource>
#include <vector>
#include <cstdio>
#include <string_view>
#include <span>

#include <unistd.h>

namespace noheap{

template<std::size_t buffer_size>
using buffer_t = std::array<char, buffer_size>;

static constexpr std::size_t output_buffer_size = 512;

class print_impl final {
   public:
	static constexpr std::size_t buffer_size = output_buffer_size;   
    	using buffer_t = buffer_t<buffer_size>;
   
   public:
	template<char end_ch, typename... Args>
	static void out(std::format_string<Args...> format, Args&&... args){
            buffer_t buffer; 
    	    std::fill_n(buffer.begin(), buffer_size, 0);
    
            auto end_it = std::format_to_n(buffer.begin(), buffer_size, format, std::forward<Args>(args)...); 
    	    *end_it.out = end_ch;

	    out_buffer(std::move(buffer));		
	}
		
	static void out_buffer(buffer_t&& buffer, std::size_t outstream = 1){
	    ::write(outstream, buffer.data(), buffer_size); 
	}
};

template<typename... Args>
constexpr void print(std::format_string<Args...> format, Args&&... args){
    print_impl::out<'\0'>(format, std::forward<Args>(args)...); 
}
template<typename... Args>
constexpr void println(std::format_string<Args...> format, Args&&... args){
    print_impl::out<'\n'>(format, std::forward<Args>(args)...); 
}

class log_impl final {
   public:
	struct owner_impl final {
	    static constexpr std::size_t buffer_size = 24;
	    using buffer_t = buffer_t<buffer_size>;
	};

	static consteval owner_impl::buffer_t create_owner(std::string_view owner){
	    owner_impl::buffer_t buffer_owner;
    	    std::fill_n(buffer_owner.begin(), owner_impl::buffer_size, 0);
	   
	    std::size_t i = 0; 
	    std::for_each(owner.begin(), owner.end(), [&](const char ch){
		buffer_owner[i++] = ch;
	    });

	    return buffer_owner;
	};
	
	template<typename... Args>
	static void logx(std::size_t outstream, owner_impl::buffer_t buffer_owner, std::format_string<Args...> format, Args&&... args){
    	    print_impl::buffer_t buffer;
    	    std::fill_n(buffer.begin(), print_impl::buffer_size, 0);
	    auto end_it = buffer.begin();
   
	    if(buffer_owner[0] != '\0'){ 
    	    	std::transform(buffer_owner.begin(), buffer_owner.end(),
			buffer_owner.begin(), 
			[](unsigned char ch){ return std::toupper(ch); });
		end_it = std::format_to_n(buffer.begin(), print_impl::buffer_size, "[{}]: ", buffer_owner.data()).out;
    	    }
	    end_it = std::format_to_n(end_it, std::abs(std::distance(buffer.end(), end_it)), format, std::forward<Args>(args)...).out; 
    	    *end_it = '\n';

    	    print_impl::out_buffer(std::move(buffer), outstream); 
	}
};

class runtime_error final : public std::exception{
   public:
	static constexpr std::size_t buffer_size = output_buffer_size;   
	using buffer_t = buffer_t<buffer_size>;

   public:
	template<typename... Args>
	runtime_error(std::format_string<Args...> format, Args&&... args){
    	    std::fill_n(buffer.begin(), buffer_size, 0);

    	    if(format.get().size()){
    	    	auto end_it = std::format_to_n(buffer.begin(), buffer_size, format, std::forward<Args>(args)...); 
    		*end_it.out = '\0'; 
    	    }
	} 
	runtime_error(buffer_t&& _buffer) : buffer(_buffer){} 
	~runtime_error() override = default;

   public:
	void set_owner(log_impl::owner_impl::buffer_t _buffer_owner){
	    buffer_owner = _buffer_owner;
	    was_set_owner = true;
	}

   public:
	const char* what() const noexcept override {
	    return buffer.data();
	}
	log_impl::owner_impl::buffer_t get_owner() const noexcept {
	    return buffer_owner;
	}
	bool has_set_owner() const noexcept{
	    return was_set_owner;
	} 

   private:
	buffer_t buffer;
	log_impl::owner_impl::buffer_t buffer_owner;

	bool was_set_owner;
};

class monotonic_buffer_resource_static final : public std::pmr::memory_resource{
   public:
	monotonic_buffer_resource_static(char* _buffer, std::size_t _buffer_size) : buffer(_buffer), buffer_size(_buffer_size) {}
	
   protected:
	void* do_allocate(std::size_t bytes, std::size_t alignment){
	    static constexpr auto calc_alignment_bytes = [](std::size_t offset, std::size_t alignment){
	    	std::size_t offset_end = offset;
		while(offset_end % alignment) offset_end += offset_end % alignment;
		return offset_end-offset; 
	    };
	    const std::size_t bytes_alignment = calc_alignment_bytes(offset, alignment);
	     
	    offset += bytes+bytes_alignment;

	    if(offset >= buffer_size)
		throw runtime_error("The allocator buffer is full: {} bytes were allocated.", buffer_size);
	    return buffer+offset-bytes-bytes_alignment;
	}
	void do_deallocate(void* p, std::size_t bytes, std::size_t alignment){
	}
	bool do_is_equal(const std::pmr::memory_resource& other) const noexcept{
	    try{
	    	if(dynamic_cast<decltype(this)>(&other)->buffer == this->buffer)
		    return true;
	    }
	    catch(...){}
	    return false;
	}

   private:
	char* buffer;
	std::size_t buffer_size, offset = 0;
};

template<typename T, std::size_t max_count>
struct vector_stack {
	static constexpr std::size_t buffer_size = sizeof(T)*max_count;

   private:
	buffer_t<buffer_size> buffer;
	monotonic_buffer_resource_static mbr{buffer.data(), buffer.size()};
	std::pmr::polymorphic_allocator<T> allocator{&mbr};
	
   public:
	std::pmr::vector<T> data{allocator};
};

}

class log_handler {
   public:
	static constexpr std::size_t max_outstream_count = 2;

   public:
	constexpr log_handler(noheap::log_impl::owner_impl::buffer_t _buffer_owner) : buffer_owner(_buffer_owner), buffer({}){
	    buffer[0] = 1;
	}
	constexpr log_handler(noheap::log_impl::owner_impl::buffer_t _buffer_owner, std::span<std::size_t> _buffer) 
		: buffer_owner(_buffer_owner) {
	    for(std::size_t i = 0; i < _buffer.size(); ++i)
		buffer.at(i) = _buffer[i];
	}
	
   public:
	template<typename... Args>
	void to_console(std::format_string<Args...> format, Args&&... args) const{
	    noheap::log_impl::logx(1, buffer_owner, format, std::forward<Args>(args)...);
	}

	template<typename... Args>
	void to_stream(std::size_t it_stream, std::format_string<Args...> format, Args&&... args) const{
	    noheap::log_impl::logx(buffer.at(it_stream), buffer_owner, format, std::forward<Args>(args)...);
	}

	template<typename... Args>
	void to_all(std::format_string<Args...> format, Args&&... args) const{
	    std::for_each(buffer.begin(), buffer.end(), [&](std::size_t outstream){
		if(!outstream)
		    return;
		noheap::log_impl::logx(outstream, buffer_owner, format, std::forward<Args>(args)...);
	    });
	}
	template<typename... Args>
	void to_all_with_subowner(noheap::log_impl::owner_impl::buffer_t buffer_subowner, std::format_string<Args...> format, Args&&... args) const{
	    std::for_each(buffer.begin(), buffer.end(), [&](std::size_t outstream){
		if(!outstream)
		    return;
		noheap::log_impl::logx(outstream, buffer_owner, "");
		noheap::log_impl::logx(outstream, buffer_subowner, format, std::forward<Args>(args)...);
	    });
	}

   private:
	std::array<std::size_t, max_outstream_count> buffer;
	noheap::log_impl::owner_impl::buffer_t buffer_owner;
};

#endif
