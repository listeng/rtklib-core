cmake_minimum_required(VERSION 3.22.1)

project("rtklib_core" C CXX)

# Platform-neutral RTKLIB demo5 core (rtklib/) plus the C API for apps (api/). Built as a static
# library; each app adds its own binding on top (Android: JNI in alba-nav/rtklib).
#
# Use from another CMake project:
#   add_subdirectory(<path>/rtklib-core rtklib-core)
#   target_link_libraries(<target> rtklib_core)

# RTKLIB demo5 library sources plus the receiver decoders it dispatches to
# (rcvraw.c -> input_unicore(), input_ubx(), ...). comnav.c and tersus.c are
# excluded, as in demo5's own CMake build: they no longer compile and rcvraw.c
# does not call them.
file(GLOB RTKLIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/*.c)
# Local additions inside rtklib/ (not upstream demo5): built with warnings on.
set(RTKLIB_LOCAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/gpt3.c)
list(REMOVE_ITEM RTKLIB_SOURCES ${RTKLIB_LOCAL_SOURCES})
set(RTKLIB_RCV_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/binex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/crescent.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/javad.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/novatel.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/nvs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/rt17.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/septentrio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/skytraq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/swiftnav.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/ublox.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/rcv/unicore.c
)
set(API_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/api/ppk_core.cpp)

add_library(rtklib_core STATIC
    ${RTKLIB_SOURCES}
    ${RTKLIB_RCV_SOURCES}
    ${RTKLIB_LOCAL_SOURCES}
    ${API_SOURCES}
)

# Consumers include both "ppk_core.h" and "rtklib.h".
target_include_directories(rtklib_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rtklib
    ${CMAKE_CURRENT_SOURCE_DIR}/api
)

# Same options as demo5's console apps (app/consapp/*/gcc/makefile). PUBLIC on purpose:
# NFREQ/NEXOBS and the ENA* switches change struct layouts in rtklib.h, so every file that
# includes it must see the same values.
target_compile_definitions(rtklib_core PUBLIC
    TRACE
    ENAGLO
    ENAQZS
    ENAGAL
    ENACMP
    ENAIRN
    NFREQ=4
    NEXOBS=3
)

# Several demo5 files define _POSIX_C_SOURCE 199506, which on Apple SDKs (macOS, iOS) hides
# C99 functions such as snprintf. Re-expose the full SDK; Android's bionic is unaffected.
if(APPLE)
    target_compile_definitions(rtklib_core PRIVATE _DARWIN_C_SOURCE)
endif()

# Vendor code: build as C99 with warnings silenced.
set_source_files_properties(${RTKLIB_SOURCES} ${RTKLIB_RCV_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-std=gnu99;-O2;-w"
)
set_source_files_properties(${RTKLIB_LOCAL_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-std=gnu99;-O2;-Wall"
)
set_source_files_properties(${API_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-std=c++17;-Wall"
)

# Callers that build a shared library from this archive need position-independent code.
set_target_properties(rtklib_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(rtklib_core PUBLIC ${MATH_LIBRARY})
endif()
