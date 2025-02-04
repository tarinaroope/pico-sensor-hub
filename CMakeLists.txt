cmake_minimum_required(VERSION 3.12)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(sensor-hub C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w) # Set the board used

# Initialize the Pico SDK
pico_sdk_init() 

add_subdirectory(lib/pico-simple-mqtt) # Source code dir
add_subdirectory(lib/pmicro-rf)
add_subdirectory(src)