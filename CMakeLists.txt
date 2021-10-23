cmake_minimum_required(VERSION 3.20)
project(sync3flash C)
option(NOT_USE_PREBUILT_ABOOT
        "Enable this option if you prefer to build TI bootloader, otherwise prebuilt bootloader will be used")

set(CMAKE_C_STANDARD 99)
set(CMAKE_VERBOSE_MAKEFILE ON)
#set(CMAKE_SHARED_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
#set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
add_executable(sync3flash host/usbboot.c host/usb_linux.c host/aboot_bin.c)

if(NOT_USE_PREBUILT_ABOOT)

    set(TEXT_BASE 0x40309000)

    add_subdirectory(target/)

    add_executable(mkheader host/tools/mkheader.c)

    add_custom_target(aboot_bin_c ALL
            COMMAND mkheader  ${TEXT_BASE} target/aboot.bin > ${CMAKE_CURRENT_LIST_DIR}/host/aboot_bin.c
            DEPENDS mkheader aboot.bin
            VERBATIM)
    add_dependencies(sync3flash aboot_bin_c)

endif()