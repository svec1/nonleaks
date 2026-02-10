cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

option(SNDIO "Use sndio audio interface." OFF)

set(UUV unix_udp_voice)

set(CMAKE_CXX_STANDARD 23)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

project(${UUV})
add_executable(${UUV} ${SRC_FILES})
        
find_package(OpenSSL)	
set(OPUS_BUILD_SHARED_LIBRARY ON)
add_subdirectory(thirdparty/opus)	

set(NOISE_C_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/noise-c)

file(GLOB_RECURSE NOISE_C_SOURCE_FILES ${NOISE_C_DIR}/src/*)
set(NOISE_C_INCLUDE_DIR ${NOISE_C_DIR}/include)
set(
    NOISE_C_LIBS
    ${NOISE_C_DIR}/src/protocol/libnoiseprotocol.a
    ${NOISE_C_DIR}/src/keys/libnoisekeys.a
)

add_custom_command(
    OUTPUT ./configure
    WORKING_DIRECTORY ${NOISE_C_DIR}
    COMMAND ${NOISE_C_DIR}/autogen.sh > /dev/null
    VERBATIM
)

set(noise-c_configure_output src/Makefile src/protocol/Makefile src/keys/Makefile)
add_custom_command(
    OUTPUT ${noise-c_configure_output}  
    WORKING_DIRECTORY ${NOISE_C_DIR}
    COMMAND ./configure > /dev/null
    VERBATIM
)

add_custom_target(
    noise-c
    WORKING_DIRECTORY ${NOISE_C_DIR}
    COMMAND make > /dev/null
    DEPENDS ${noise-c_configure_output}
)
add_dependencies(${UUV} noise-c)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -pthread")

target_include_directories(
    ${UUV}
    PUBLIC 
    src/
    ${LIBBOOST_PATH}
    ${OpenSSL_INCLUDE_DIRS}
    ${NOISE_C_INCLUDE_DIR}
)
target_link_libraries(
    ${UUV}
    PRIVATE
    OpenSSL::Crypto
    Opus::opus
    ${NOISE_C_LIBS}
)

if(SNDIO AND (LINUX OR BSD))
	find_library(LIBSNDIO NAMES sndio)
        if("${LIBSNDIO}" STREQUAL "LIBSNDIO-NOTFOUND")	
		message(FATAL_ERROR "[sndio] lib was not found.")
	endif()
	
	message(STATUS "Building with sndio.")
	target_link_libraries(${UUV} PRIVATE ${LIBSNDIO})
	target_compile_options(${UUV} PRIVATE "-DSNDIO")
elseif(LINUX)
	find_package(ALSA)
	
    set(AUDIO_SYSTEM "ALSA")
    target_link_libraries(${UUV} PRIVATE ALSA::ALSA)
	target_compile_options(${UUV} PRIVATE "-DALSA")
elseif(BSD)
	message(STATUS "Building with ${CMAKE_SYSTEM_NAME} audio.")
	if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
		target_compile_options(${UUV} PRIVATE "-DFREEBSD_AUDIO")
	elseif(CMAKE_SYSTEM_NAME STREQUAL "OpenBSD")
		target_compile_options(${UUV} PRIVATE "-DOPENBSD_AUDIO")
	elseif(CMAKE_SYSTEM_NAME STREQUAL "NetBSD")
		target_compile_options(${UUV} PRIVATE "-DNETBSD_AUDIO")
	endif()
endif()
