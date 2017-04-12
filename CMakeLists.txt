# Copyright (c) 2016, Robert Lukierski.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.1)

# ------------------------------------------------------------------------------
# Default build configuration
# ------------------------------------------------------------------------------
if(DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Build type.")
else()
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type.")
endif()

# ------------------------------------------------------------------------------
# Project name & version
# ------------------------------------------------------------------------------
project(MiscCpp)
set(${PROJECT_NAME}_VERSION 1.0.0)

# ------------------------------------------------------------------------------
# Enable C++11 and high level warnings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable)

# ------------------------------------------------------------------------------
# Ready for testing
# ------------------------------------------------------------------------------
option(BUILD_TESTS "Enable to build unit tests" ON)
find_package(GTest QUIET)
if(BUILD_TESTS AND GTest_FOUND)
  enable_testing()
    
  # Code Coverage Build Type
  set(CMAKE_CXX_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage" CACHE STRING "Flags used by the C++ compiler during coverage builds." FORCE)
  set(CMAKE_C_FLAGS_COVERAGE "-g -O0 --coverage -fprofile-arcs -ftest-coverage" CACHE STRING "Flags used by the C compiler during coverage builds." FORCE )
  set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "" CACHE STRING "Flags used for linking binaries during coverage builds." FORCE )
  set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "" CACHE STRING "Flags used by the shared libraries linker during coverage builds." FORCE)
  mark_as_advanced(CMAKE_CXX_FLAGS_COVERAGE CMAKE_C_FLAGS_COVERAGE CMAKE_EXE_LINKER_FLAGS_COVERAGE CMAKE_SHARED_LINKER_FLAGS_COVERAGE)
endif()

include(GenerateExportHeader)

# ---------------------------------------------
# Where to look for Find*.cmake
# ---------------------------------------------
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/scripts)

# ------------------------------------------------------------------------------
# Print Project Info
# ------------------------------------------------------------------------------
message("Project: ${PROJECT_NAME} / ${${PROJECT_NAME}_VERSION}, build type: ${CMAKE_BUILD_TYPE}, compiled on: ${CMAKE_SYSTEM}, flags: ${CMAKE_CXX_FLAGS}")

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(cereal QUIET)
set(HAVE_CEREAL ${cereal_FOUND})

find_package(ZeroMQ QUIET)
set(HAVE_ZEROMQ ${ZEROMQ_FOUND})

find_package(NanoMSG QUIET)
set(HAVE_NANOMSG ${NANOMSG_FOUND})

find_package(Snappy QUIET)
set(HAVE_SNAPPY ${SNAPPY_FOUND})

find_package(LibUSB QUIET)
set(HAVE_LIBUSB ${LibUSB_FOUND})

if(NOT DEFINED ENABLE_NETLINK)
    set(ENABLE_NETLINK FALSE)
    set(HAVE_LIBNL FALSE)
else()
    find_package(NL QUIET)
    set(HAVE_LIBNL ${NL_FOUND})
endif()

if(NOT DEFINED ENABLE_XENOMAI)
    set(ENABLE_XENOMAI FALSE)
    set(HAVE_XENOMAI FALSE)
else()
    find_package(Xenomai QUIET)
    set(HAVE_XENOMAI ${Xenomai_FOUND})
endif()

#set(HAVE_AVAHI FALSE) # TODO FIXME
#set(HAVE_DNSSD FALSE) # TODO FIXME

# ---------------------------------------------
# List of sources
# ---------------------------------------------
set(HEADERS
include/misccpp/BufferDecode.hpp
include/misccpp/BufferEncode.hpp
include/misccpp/drivers/SerialPort.hpp
include/misccpp/lowlevelcom/bridge.hpp
include/misccpp/lowlevelcom/bridge_linux.hpp
include/misccpp/lowlevelcom/cobs.hpp
include/misccpp/lowlevelcom/lowlevelcom.hpp
include/misccpp/lowlevelcom/mux.hpp
include/misccpp/lowlevelcom/mux_linux.hpp
include/misccpp/lowlevelcom/transport/transport_can.hpp
include/misccpp/lowlevelcom/transport/transport_i2c.hpp
include/misccpp/lowlevelcom/transport/transport_raw.hpp
include/misccpp/lowlevelcom/transport/transport_serial_port.hpp
include/misccpp/lowlevelcom/transport/transport_spi.hpp
include/misccpp/lowlevelcom/utils.hpp
)

set(SOURCES
sources/drivers/SerialPort.cpp
sources/lowlevelcom/lowlevelcom.cpp
sources/lowlevelcom/transport_can.cpp
sources/lowlevelcom/transport_i2c.cpp
sources/lowlevelcom/transport_serial_port.cpp
sources/lowlevelcom/transport_spi.cpp
)

if(HAVE_ZEROMQ)
    list(APPEND HEADERS include/misccpp/zmq.hpp)
    list(APPEND HEADERS include/misccpp/lowlevelcom/transport/transport_zeromq.hpp)
endif()

if(HAVE_NANOMSG)
    list(APPEND HEADERS include/misccpp/nanomsg.hpp)
    list(APPEND HEADERS include/misccpp/lowlevelcom/transport/transport_nanomsg.hpp)
endif()

if(HAVE_SNAPPY)
    list(APPEND HEADERS include/misccpp/snappy.hpp)
    list(APPEND SOURCES sources/snappy.cpp)
endif()

if(HAVE_CEREAL)
    list(APPEND HEADERS include/misccpp/cereal_raw_binary.hpp)
endif()

if(HAVE_LIBUSB)
    list(APPEND HEADERS include/misccpp/lowlevelcom/transport/transport_usb.hpp)
    list(APPEND SOURCES sources/lowlevelcom/transport_usb.cpp)
endif()

if(HAVE_LIBNL)
    list(APPEND HEADERS include/misccpp/lowlevelcom/transport/transport_netlink.hpp)
    list(APPEND SOURCES sources/lowlevelcom/transport_netlink.cpp)
endif()

# ------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE sources)
    
if(HAVE_ZEROMQ)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${ZEROMQ_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${ZEROMQ_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC MISCCPP_HAVE_ZEROMQ)
endif()

if(HAVE_NANOMSG)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${NANOMSG_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${NANOMSG_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC MISCCPP_HAVE_NANOMSG)
endif()

if(HAVE_SNAPPY)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${SNAPPY_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${SNAPPY_INCLUDE_DIR})
    target_compile_definitions(${PROJECT_NAME} PRIVATE MISCCPP_HAVE_SNAPPY)
endif()
    
if(HAVE_CEREAL)
    target_link_libraries(${PROJECT_NAME} PUBLIC cereal)
    target_compile_definitions(${PROJECT_NAME} PUBLIC MISCCPP_HAVE_CEREAL)
endif()

if(HAVE_LIBUSB)
    target_include_directories(${PROJECT_NAME} PRIVATE ${LibUSB_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${LibUSB_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PUBLIC MISCCPP_HAVE_LIBUSB)
endif()

if(HAVE_LIBNL)
    target_include_directories(${PROJECT_NAME} PRIVATE ${NL_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${NL_LIBRARIES})
endif()

if(HAVE_XENOMAI)
    target_include_directories(${PROJECT_NAME} PRIVATE ${Xenomai_INCLUDE_DIR})
    target_compile_definitions(${PROJECT_NAME} PRIVATE ${Xenomai_DEFINITIONS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${Xenomai_LIBRARY_NATIVE} ${Xenomai_LIBRARY_XENOMAI} ${Xenomai_LIBRARY_PTHREAD_RT} ${Xenomai_LIBRARY_RTDM})
endif()
    
# ------------------------------------------------------------------------------
# Unit Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTS AND GTest_FOUND)
    add_subdirectory(tests)
endif()

# ------------------------------------------------------------------------------
# Installation - library
# ------------------------------------------------------------------------------
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h)
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${${PROJECT_NAME}_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION 1)
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_${PROJECT_NAME}_MAJOR_VERSION 1)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING ${PROJECT_NAME}_MAJOR_VERSION)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(FILES ${HEADERS} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h" DESTINATION include)

# ------------------------------------------------------------------------------
# Installation - generate version file
# ------------------------------------------------------------------------------
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# ------------------------------------------------------------------------------
# Installation - Export Targets
# ------------------------------------------------------------------------------
export(EXPORT ${PROJECT_NAME}Targets FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake") 
configure_file(${PROJECT_NAME}Config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets FILE ${PROJECT_NAME}Targets.cmake DESTINATION ${ConfigPackageLocation})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION ${ConfigPackageLocation})
export(PACKAGE ${PROJECT_NAME})
