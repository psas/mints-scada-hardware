cmake_minimum_required(VERSION 3.22)



#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME module_io)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object typef
add_executable(${CMAKE_PROJECT_NAME}

        Core/Inc/configuration.h
        Core/Inc/stm32f0xx_hal_conf.h
        Core/Inc/mcp346x.h
        Core/Inc/adc.h
        Core/Inc/can.h
        Core/Inc/spi.h
        Core/Inc/i2c.h
        Core/Inc/stm32f0xx_it.h
        Core/Inc/gpio.h
        Core/Inc/uprintf.h
        Core/Inc/buscommands.h
        Core/Inc/datapacket.h
        Core/Inc/main.h
        Core/Inc/interface_module.h

        Core/Src/stm32f0xx_it.c
        Core/Src/can.c
        Core/Src/interface_module.c
        Core/Src/syscalls.c
        Core/Src/sysmem.c
        Core/Src/mcp346x.c
        Core/Src/stm32f0xx_hal_msp.c
        Core/Src/datapacket.c
        Core/Src/gpio.c
        Core/Src/main.c
        Core/Src/uprintf.c
        Core/Src/adc.c
        Core/Src/spi.c
        Core/Src/i2c.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Convert to bin file
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
