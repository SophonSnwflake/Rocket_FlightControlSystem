cmake_minimum_required(VERSION 3.22)

project(tsl LANGUAGES C CXX)

add_library(${PROJECT_NAME} STATIC)

# CMSIS core path required by CMSIS-DSP
set(CMSISCORE "${CMAKE_SOURCE_DIR}/CubeMX_BSP/Drivers/CMSIS")
add_subdirectory(Dependence/CMSIS-DSP)
add_subdirectory(Dependence/eigen)

file(GLOB_RECURSE TSL_ALGORITHM_SOURCES
    "Algorithm/src/*.c"
    "Algorithm/src/*.cpp"
)

file(GLOB_RECURSE TSL_DEVICE_SOURCES
    "Device/*.c"
    "Device/*.cpp"
)

file(GLOB_RECURSE TSL_DRIVER_SOURCES
    "Driver/src/*.c"
    "Driver/src/*.cpp"
)

target_sources(${PROJECT_NAME} PRIVATE
    ${TSL_ALGORITHM_SOURCES}
    ${TSL_DEVICE_SOURCES}
    ${TSL_DRIVER_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    Algorithm/inc
    Device/BMI088
    Device/GNSS/inc
    Driver/inc
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    EIGEN_NO_MALLOC
)

target_compile_features(${PROJECT_NAME} PUBLIC
    c_std_11
    cxx_std_17
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    stm32cubemx
    CMSISDSP
    eigen
)
