cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform")

include(pico_sdk_import.cmake)
project(Pico-10BASE-T C CXX ASM)
set(target_name Pico-10BASE-T)

pico_sdk_init()

add_executable(${target_name})

pico_generate_pio_header(${target_name} ${CMAKE_CURRENT_LIST_DIR}/ser_10base_t.pio)
pico_generate_pio_header(${target_name} ${CMAKE_CURRENT_LIST_DIR}/rx_10base_t.pio)

target_sources(${target_name} PRIVATE
    main.c
    udp.c
    eth_rx.c
    )

target_link_libraries(${target_name} PRIVATE
    pico_stdlib
    hardware_pio
    hardware_irq
    hardware_dma
    )

# enable usb output, disable uart output
pico_enable_stdio_usb(${target_name} 1)
pico_enable_stdio_uart(${target_name} 0)

# Default binaries to Copy code to RAM when booting from flash.
# Code execution speed may improve, but code size is limited by RAM capacity.
#pico_set_binary_type(${target_name} copy_to_ram)

pico_add_extra_outputs(${target_name})
