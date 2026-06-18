cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(MLDSA_NATIVE mldsa_native)
set(MLDSA_NATIVE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mldsa-native/)
set(MLDSA_NATIVE_SOURCE_DIR ${MLDSA_NATIVE_DIR}/mldsa/)
set(MLDSA_NATIVE_INCLUDE_DIR ${MLDSA_NATIVE_SOURCE_DIR})
file(GLOB_RECURSE MLDSA_NATIVE_SOURCE ${MLDSA_NATIVE_SOURCE_DIR}/*.c)

if(NOT MLDSA_NATIVE_TYPE)
	set(MLDSA_NATIVE_TYPE 65)
endif()

add_library(${MLDSA_NATIVE} STATIC ${MLDSA_NATIVE_SOURCE})
target_compile_options(
	${MLDSA_NATIVE}
	PRIVATE -Wall -Wextra -Werror=unused-result 
	-Wpedantic -Werror -Wmissing-prototypes -Wshadow 
	-Wpointer-arith -Wredundant-decls -Wconversion 
	-Wsign-conversion -Wno-long-long -Wno-unknown-pragmas 
	-Wno-unused-command-line-argument -O3 -fomit-frame-pointer 
	-std=c99 -pedantic -MMD
	
)
target_compile_definitions(
	${MLDSA_NATIVE}
	PUBLIC
    MLD_CONFIG_NAMESPACE_PREFIX=mldsa
	MLD_CONFIG_PARAMETER_SET=${MLDSA_NATIVE_TYPE}
)
target_include_directories(${MLDSA_NATIVE} PUBLIC ${MLDSA_NATIVE_INCLUDE_DIR})
