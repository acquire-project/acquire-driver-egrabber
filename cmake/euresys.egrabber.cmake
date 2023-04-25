# Euresys EGrabber library
# This is a GenICam header-only library

find_path(euresys_egrabber_include_dir "EGrabber.h"
    PATH_SUFFIXES "euresys/egrabber/include"
    DOC "Directory that contains EGrabber.h"
    NO_CACHE)

if(euresys_egrabber_include_dir)
    message(STATUS "Euresys EGrabber ${euresys_egrabber_include_dir}")

    set(tgt egrabber)
    add_library(${tgt} IMPORTED INTERFACE)
    set_target_properties(${tgt} PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${euresys_egrabber_include_dir})
endif()