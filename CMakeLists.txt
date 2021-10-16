cmake_minimum_required(VERSION 3.20)
project(sync3flash C)

set(CMAKE_C_STANDARD 99)
#set(CMAKE_SHARED_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
#set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++ -static")
add_executable(sync3flash host/usbboot.c host/usb_linux.c host/aboot.c)
