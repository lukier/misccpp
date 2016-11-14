# - Try to find the NanoMSG library
#
#  This module defines the following variables
#
#  NANOMSG_FOUND - Was NanoMSG found
#  NANOMSG_INCLUDE_DIRS - the NanoMSG include directories
#  NANOMSG_LIBRARIES - Link to this
#
#  This module accepts the following variables
#
#  NANOMSG_ROOT - Can be set to NanoMSG install path or Windows build path
#

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_NANOMSG ${NANOMSG_FIND_REQUIRED} libnanomsg)
if(PC_NANOMSG_FOUND)
    set(NANOMSG_FOUND TRUE)
    set(NANOMSG_INCLUDE_DIRS ${PC_NANOMSG_INCLUDE_DIRS})
    set(NANOMSG_LIBRARY_DIRS ${PC_NANOMSG_LIBRARY_DIRS})
    set(NANOMSG_LIBRARIES ${PC_NANOMSG_LIBRARIES})
else()
    pkg_check_modules(PC_NANOMSG ${NANOMSG_FIND_REQUIRED} nanomsg)
    if(PC_NANOMSG_FOUND)
        set(NANOMSG_FOUND TRUE)
        set(NANOMSG_INCLUDE_DIRS ${PC_NANOMSG_INCLUDE_DIRS})
        set(NANOMSG_LIBRARY_DIRS ${PC_NANOMSG_LIBRARY_DIRS})
        set(NANOMSG_LIBRARIES ${PC_NANOMSG_LIBRARIES})
    else()
        set(NANOMSG_FOUND FALSE)
    endif()
endif()
