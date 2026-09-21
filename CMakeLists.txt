cmake_minimum_required(VERSION 3.13)

# Vor dem Bauen setzen:
#   export PICO_SDK_PATH=/pfad/zu/pico-sdk
#   export PICO_EXTRAS_PATH=/pfad/zu/pico-extras
#   export FREERTOS_KERNEL_PATH=/pfad/zu/FreeRTOS-Kernel
#
# Zusätzlich einmalig zwei Import-Dateien hierher kopieren (analog zu
# pico_sdk_import.cmake, das schon im Projekt liegt):
#   cp $PICO_EXTRAS_PATH/external/pico_extras_import.cmake .
#   cp $FREERTOS_KERNEL_PATH/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake .
#
# Siehe README.md für Details.

set(PICO_BOARD pico CACHE STRING "Board type")

# WICHTIG: ohne explizit gesetzten Build-Type kompiliert CMake mit -O0
# (keine Optimierung). Auf einem FPU-losen Chip mit viel Float-Code wie
# hier macht das einen Unterschied von Faktor 5-10x - siehe DEVLOG.
# Als Default absichern, falls -DCMAKE_BUILD_TYPE beim cmake-Aufruf
# vergessen wird; explizit übergebenes -DCMAKE_BUILD_TYPE=... überschreibt
# das trotzdem, weil "CACHE ... FORCE" hier bewusst NICHT verwendet wird.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" )
endif()

# Alle drei Import-Dateien MÜSSEN vor project() eingebunden werden.
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(envelope_talkbox C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(envelope_talkbox
    src/main.cpp
)

# FreeRTOSConfig.h liegt im Projekt-Root (nicht unter src/) - Pfad hier
# bekanntmachen, damit FreeRTOS.h ihn findet.
target_include_directories(envelope_talkbox PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Generischer Pico + externes PCM5100A/PCM5102A-Breakout: I2S-Pins sind
# frei wählbar - an deine tatsächliche Verkabelung anpassen.
target_compile_definitions(envelope_talkbox PRIVATE
    PICO_AUDIO_I2S_DATA_PIN=18
    PICO_AUDIO_I2S_CLOCK_PIN_BASE=16
)

# pico_multicore nicht mehr nötig - Rechenlast ist mit nur einem
# Envelope-Follower trivial, ein Kern reicht bequem (siehe DEVLOG).
target_link_libraries(envelope_talkbox PRIVATE
    pico_stdlib
    pico_audio_i2s
    hardware_adc
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_enable_stdio_usb(envelope_talkbox 1)
pico_enable_stdio_uart(envelope_talkbox 0)

pico_add_extra_outputs(envelope_talkbox)
