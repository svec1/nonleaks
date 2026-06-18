cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

option(XXCORE_SNDIO "Use sndio audio interface." OFF)
option(XXCORE_NOTUSE_AUDIO "Do not use audio class." OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS YES)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_C_COMPILER clang)

set(XXCORE xxcore)
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

project(${XXCORE})

# Builds OPUS lib
set(OPUS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opus)
set(OPUS_INCLUDE_DIR ${OPUS_DIR}/include)
set(OPUS_BUILD_SHARED_LIBRARY ON)
set(OPUS_VAR_ARRAYS ON)
set(OPUS_NONTHREADSAFE_PSEUDOSTACK OFF)
set(OPUS_USE_ALLOCA OFF)
add_subdirectory(${OPUS_DIR})	

# Builds CNOISE lib
set(CNOISE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cnoise)
set(CNOISE_TESTS OFF)
add_subdirectory(${CNOISE_DIR})	

# Builds MLDSA_NATIVE lib
set(MLDSA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake/mldsa-native.cmake)
include(${MLDSA_DIR})
message(STATUS "Use MLDSA-${MLDSA_NATIVE_TYPE} type.")

add_executable(${XXCORE} ${SRC_FILES})
target_compile_options(${XXCORE} PRIVATE -Wall -Wextra -Wpedantic -pthread -O1)
target_link_options(${XXCORE} PRIVATE -rdynamic)
target_include_directories(
    ${XXCORE}
    PUBLIC 
    src/
    ${BOOST_INCLUDE_DIR}
    ${OPUS_INCLUDE_DIR}
    ${CNOISE_INCLUDE_DIR}
	${MLDSA_NATIVE_INCLUDE_DIR}
)
target_link_libraries(
    ${XXCORE}
    PRIVATE
    Opus::opus
    cnoise
	mldsa_native
)

set(XXCORE_AUDIO_SYSTEM "UNDEFINED")    
if(NOT ${XXCORE_NOTUSE_AUDIO})
    if(${XXCORE_SNDIO} AND (LINUX OR BSD))
	    find_library(LIBSNDIO NAMES sndio)
        if("${LIBSNDIO}" STREQUAL "LIBSNDIO-NOTFOUND")	
		    message(FATAL_ERROR "[sndio] lib was not found.")
	    endif()
	
        set(XXCORE_AUDIO_SYSTEM "SNDIO")    
	    target_link_libraries(${XXCORE} PRIVATE ${LIBSNDIO})
	    target_compile_options(${XXCORE} PRIVATE "-DSNDIO")
    elseif(LINUX)
        find_package(ALSA)
        if(NOT ${ALSA_FOUND})	
		    message(FATAL_ERROR "[alsa] lib was not found.")
	    endif()

        set(XXCORE_AUDIO_SYSTEM "ALSA")    
        target_link_libraries(${XXCORE} PRIVATE ALSA::ALSA)
        target_compile_options(${XXCORE} PRIVATE "-DALSA")
    elseif(BSD)
        set(XXCORE_AUDIO_SYSTEM ${CMAKE_SYSTEM_NAME})    
	    if(CMAKE_SYSTEM_NAME STREQUAL "FreeBSD")
		    target_compile_options(${XXCORE} PRIVATE "-DFREEBSD_AUDIO")
	    elseif(CMAKE_SYSTEM_NAME STREQUAL "OpenBSD")
		    target_compile_options(${XXCORE} PRIVATE "-DOPENBSD_AUDIO")
	    elseif(CMAKE_SYSTEM_NAME STREQUAL "NetBSD")
		    target_compile_options(${XXCORE} PRIVATE "-DNETBSD_AUDIO")
	    endif()
    endif()
endif()
	
message(STATUS "Using audio system ${XXCORE_AUDIO_SYSTEM}")

file(COPY xxcore.json DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
