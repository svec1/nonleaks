cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(ATV alsa_tcp_voice)

project(ALSA_TCP_VOICE)
set(CMAKE_CXX_STANDARD 20)
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(${ATV} ${SRC_FILES})

target_link_libraries(${ATV} PRIVATE asound)
target_include_directories(${ATV} PRIVATE src/)
