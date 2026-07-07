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
set(CMAKE_PROJECT_NAME sentry_gimbal)

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

# TSL: TemporarySTM32Library (algorithms + device drivers)
add_subdirectory(TSL)

# TemporarySTM32Library: Algorithm + Device modules
file(GLOB_RECURSE TEMP_LIB_SRC
    TemporarySTM32Library/Algorithm/src/*.c
    TemporarySTM32Library/Algorithm/src/*.cpp
    TemporarySTM32Library/Device/src/*.c
    TemporarySTM32Library/Device/src/*.cpp
)
if(TEMP_LIB_SRC)
    add_library(TemporarySTM32Library STATIC ${TEMP_LIB_SRC})
    target_include_directories(TemporarySTM32Library PUBLIC
        TemporarySTM32Library/Algorithm/inc
        TemporarySTM32Library/Device/inc
    )
    target_link_libraries(TemporarySTM32Library PUBLIC stm32cubemx)
else()
    add_library(TemporarySTM32Library INTERFACE)
    target_include_directories(TemporarySTM32Library INTERFACE
        TemporarySTM32Library/Algorithm/inc
        TemporarySTM32Library/Device/inc
    )
    target_link_libraries(TemporarySTM32Library INTERFACE stm32cubemx)
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_SRC
    LogicCode/*.c
    LogicCode/*.cpp
    Task/*.c
    Task/*.cpp
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    LogicCode/inc
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
    TemporarySTM32Library
    tsl
)
