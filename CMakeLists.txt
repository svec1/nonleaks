cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

option(SNDIO "Use sndio audio interface." OFF)

set(UUV unix_udp_voice)

set(CMAKE_CXX_STANDARD 23)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

project(${UUV})
add_executable(${UUV} ${SRC_FILES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -pthread")
target_include_directories(${UUV} PRIVATE ${LIBBOOST_PATH} src/)


find_package(OpenSSL)	
target_link_libraries(${UUV} PRIVATE OpenSSL::Crypto)

set(OPUS_BUILD_SHARED_LIBRARY ON)
add_subdirectory(thirdparty/opus)	
target_link_libraries(${UUV} PRIVATE Opus::opus)

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
