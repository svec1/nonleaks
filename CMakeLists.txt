cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

option(SNDIO "Use sndio audio interface." OFF)

set(UUV unix_udp_voice)
set(CMAKE_CXX_STANDARD 23)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

project(${UUV})
add_executable(${UUV} ${SRC_FILES})

target_include_directories(${UUV} PRIVATE ${LIBBOOST_PATH} src/)
if(SNDIO)
	message(STATUS "Building with sndio.")
	target_link_libraries(${UUV} PRIVATE sndio)
	target_compile_options(${UUV} PRIVATE "-DSNDIO")
elseif(BSD)
	message(STATUS "Building with audio.")
	target_compile_options(${UUV} PRIVATE "-DAUDIO4")
else()
	message(STATUS "Building with alsa.")
	target_link_libraries(${UUV} PRIVATE asound)
endif()
