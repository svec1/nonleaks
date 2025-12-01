cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

option(SNDIO "Use sndio audio interface." OFF)

set(UUV unix_udp_voice)

set(CMAKE_CXX_STANDARD 23)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

project(${UUV})
add_executable(${UUV} ${SRC_FILES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
target_include_directories(${UUV} PRIVATE ${LIBBOOST_PATH} src/)
if(SNDIO AND (LINUX OR BSD))
	find_library(LIBSNDIO sndio)
	if(NOT ${LIBSNDIO})	
		message(FATAL_ERROR "[sndio] lib was not found.")
	endif()
	
	message(STATUS "Building with sndio.")
	target_link_libraries(${UUV} PRIVATE ${LIBSNDIO})
	target_compile_options(${UUV} PRIVATE "-DSNDIO")
endif()

if(LINUX)
	find_library(LIBASOUND asound)
	if(NOT ${LIBASOUND})	
		message(FATAL_ERROR "[asound] lib was not found.")
	endif()
	
	message(STATUS "Building with alsa.")
	target_link_libraries(${UUV} PRIVATE ${LIBASOUND})
	target_compile_options(${UUV} PRIVATE "-DALSA")
elseif(BSD)
	message(STATUS "Building with oss.")
	target_compile_options(${UUV} PRIVATE "-DOSS")
endif()
