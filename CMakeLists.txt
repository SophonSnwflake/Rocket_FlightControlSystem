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
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rocket_26)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(CubeMX_BSP/cmake/stm32cubemx)

# RSL: RocketStandardLibrary (algorithms + device drivers)
add_subdirectory(RSL)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_SRC
    Application/Rocket/src/*.c
    Application/Rocket/src/*.cpp
    Application/Logging/src/*.c
    Application/Logging/src/*.cpp
    Application/Command/src/*.c
    Application/Command/src/*.cpp
    Application/Communication/src/*.c
    Application/Communication/src/*.cpp
    Task/*.c
    Task/*.cpp
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application/Rocket/inc
    Application/Command/inc
    Application/Communication/inc
    Application/Logging/inc
    Application/Logging/Agreement
    Task
    Task/inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add compile options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined compile options
    -fdiagnostics-color=always
    -O2
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined link options
    -u _printf_float
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    rsl
)
