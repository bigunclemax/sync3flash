enable_language(ASM)

if (NOT DEFINED TEXT_BASE)
    set(TEXT_BASE 0x40309000)
endif ()

if (NOT DEFINED tools)
    MESSAGE(FATAL_ERROR "tools path not specified")
endif ()

set(CMAKE_C_COMPILER "${tools}/bin/arm-eabi-gcc")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Os  -Wall -fno-builtin -ffreestanding -march=armv7-a -mthumb -ffunction-sections -fdata-sections ")
set(CMAKE_LINKER "${tools}/bin/arm-eabi-ld")
set(CMAKE_OBJCOPY "${tools}/bin/arm-eabi-objcopy")
set(CMAKE_ASM_COMPILER "${tools}/bin/arm-eabi-gcc")
SET(ASM_OPTIONS "-Wa,-march=armv7-a+sec ")
SET(CMAKE_ASM_FLAGS "${CFLAGS} ${ASM_OPTIONS}" )
SET(CMAKE_C_LINK_FLAGS "-Bstatic --gc-sections -T ${CMAKE_CURRENT_LIST_DIR}/arch/omap5/iboot.lds -Ttext ${TEXT_BASE}")

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_LINKER> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> <LINK_LIBRARIES> -o <TARGET>")

set(OMAP_COMMON_OBJS
    arch/common/serial.c
    arch/common/gpio.c
    arch/common/mem_alloc.c
    arch/common/omap_usb.c
    arch/common/rom_mmc.c
    arch/common/rom_i2c.c
    arch/common/rom_wdtimer.c)

set(COMMON_OBJS
    boot_common.c
    libc/printf.c
    libc/string.c
    libc/ctype.c
    arch/common/misc.c)

set(PROC_COMMON_OBJS
    arch/omap5/clock.c
    arch/omap5/sdram.c
    arch/omap5/gpmc.c
    arch/omap5/id.c
    arch/omap5/smartio.c
    arch/omap5/pmic.c
    arch/common/rom_sata.c)

set(BOARD_OBJS
    arch/omap5/board/board_omap5uevm.c)


add_executable(aboot aboot.c)

target_sources(aboot PRIVATE
        ${OMAP_COMMON_OBJS}
        ${COMMON_OBJS}
        ${PROC_COMMON_OBJS}
        ${BOARD_OBJS}
        arch/common/start.S
        arch/common/rom_usb.c)

target_include_directories(aboot PRIVATE
        include/
        include/aboot
        include/omap5
        include/common
        include/libc)

execute_process(
        COMMAND ${CMAKE_C_COMPILER} ${TOOLCHAIN_C_FLAGS} --print-libgcc-file-name
        OUTPUT_VARIABLE LIBGCC_FILE_NAME
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
target_link_libraries(aboot ${LIBGCC_FILE_NAME})

add_custom_target(aboot.bin ALL
        COMMAND ${CMAKE_OBJCOPY} --gap-fill=0xee -O binary aboot aboot.bin
        DEPENDS aboot
        VERBATIM)