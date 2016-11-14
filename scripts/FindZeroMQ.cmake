# - Try to find the ZeroMQ library
#
#  This module defines the following variables
#
#  ZEROMQ_FOUND - Was ZeroMQ found
#  ZEROMQ_INCLUDE_DIRS - the ZeroMQ include directories
#  ZEROMQ_LIBRARIES - Link to this
#
#  This module accepts the following variables
#
#  ZEROMQ_ROOT - Can be set to ZeroMQ install path or Windows build path
#

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_ZEROMQ REQUIRED libzmq)
if(PC_ZEROMQ_FOUND)
    set(ZEROMQ_FOUND TRUE)
    set(ZEROMQ_INCLUDE_DIRS ${PC_ZEROMQ_INCLUDE_DIRS})
    set(ZEROMQ_LIBRARY_DIRS ${PC_ZEROMQ_LIBRARY_DIRS})
    set(ZEROMQ_LIBRARIES ${PC_ZEROMQ_LIBRARIES})
else()
    set(ZEROMQ_FOUND FALSE)
endif()
