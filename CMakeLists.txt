cmake_minimum_required(VERSION 3.22.1)

project("rtklib" C CXX)

# RTKLIB demo5 library sources plus the receiver decoders it dispatches to
# (rcvraw.c -> input_unicore(), input_ubx(), ...). comnav.c and tersus.c are
# excluded, as in demo5's own CMake build: they no longer compile and rcvraw.c
# does not call them.
file(GLOB RTKLIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/rtklib/*.c)
set(RTKLIB_RCV_SOURCES
    rtklib/rcv/binex.c
    rtklib/rcv/crescent.c
    rtklib/rcv/javad.c
    rtklib/rcv/novatel.c
    rtklib/rcv/nvs.c
    rtklib/rcv/rt17.c
    rtklib/rcv/septentrio.c
    rtklib/rcv/skytraq.c
    rtklib/rcv/swiftnav.c
    rtklib/rcv/ublox.c
    rtklib/rcv/unicore.c
)

add_library(rtklib SHARED
    ${RTKLIB_SOURCES}
    ${RTKLIB_RCV_SOURCES}
    ppk_core.cpp
    jni_rtklib.cpp
)

target_include_directories(rtklib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/rtklib)

# Same options as demo5's console apps (app/consapp/*/gcc/makefile).
target_compile_definitions(rtklib PRIVATE
    TRACE
    ENAGLO
    ENAQZS
    ENAGAL
    ENACMP
    ENAIRN
    NFREQ=4
    NEXOBS=3
)

# Vendor code: build as C99 with warnings silenced.
set_source_files_properties(${RTKLIB_SOURCES} ${RTKLIB_RCV_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-std=gnu99;-O2;-w"
)
set_source_files_properties(ppk_core.cpp jni_rtklib.cpp PROPERTIES
    COMPILE_OPTIONS "-std=c++17;-Wall"
)

target_link_libraries(rtklib log m)
