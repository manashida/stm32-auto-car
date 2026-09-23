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
set(CMAKE_PROJECT_NAME auto_car)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/motor.c
    Core/Src/car.c
    Core/Src/motion_control.c
    Core/Src/oled_ui.c
    Core/Src/ultrasonic.c
    Core/Src/bluetooth.c
    Core/Src/app_health.c
    Core/Src/auto_run.c
    Core/Src/follow_task.c
    Core/Src/servo.c
    Core/Src/hx711.c
    Core/Src/uwb_follow.c
    Core/Src/ws2812.c
    Core/Src/rgb_led.c
    Core/Src/tracking.c
    Core/Src/encoder.c
    Core/Src/buzzer.c
    Core/Src/status_led.c
    Core/Src/transport_diag.c
    Core/Src/transport_load.c
    Core/Src/transport_nav.c
    Core/Src/transport_prompt.c
    Core/Src/transport_task.c
    Core/Src/app_mode_task.c
)

set_source_files_properties(Core/Src/ws2812.c PROPERTIES COMPILE_OPTIONS "-O2")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI)

if(STM32_PROGRAMMER_CLI)
    add_custom_target(flash
        COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD -w $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -v -rst
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Flashing ${CMAKE_PROJECT_NAME} with STM32CubeProgrammer"
        VERBATIM
    )
else()
    message(WARNING "STM32_Programmer_CLI not found. Install STM32CubeProgrammer or add it to PATH to enable the flash target.")
endif()
