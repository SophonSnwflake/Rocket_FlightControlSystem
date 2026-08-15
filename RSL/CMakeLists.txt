cmake_minimum_required(VERSION 3.22)

project(rsl LANGUAGES C CXX)

add_library(${PROJECT_NAME} STATIC)

# CMSIS core path required by CMSIS-DSP
set(CMSISCORE "${CMAKE_SOURCE_DIR}/CubeMX_BSP/Drivers/CMSIS")
add_subdirectory(Dependence/CMSIS-DSP)
add_subdirectory(Dependence/eigen)


file(GLOB_RECURSE RSL_ALGORITHM_AHRS_SOURCES
    "Algorithm/Ahrs/src/*.c"
    "Algorithm/Ahrs/src/*.cpp"
)

file(GLOB_RECURSE RSL_ALGORITHM_FILTER_SOURCES
    "Algorithm/Filter/src/*.c"
    "Algorithm/Filter/src/*.cpp"
)

file(GLOB_RECURSE RSL_ALGORITHM_GENERAL_SOURCES
    "Algorithm/General/src/*.c"
    "Algorithm/General/src/*.cpp"
)

file(GLOB_RECURSE RSL_ALGORITHM_KALMANFILTER_SOURCES
    "Algorithm/KalmanFilter/src/*.c"
    "Algorithm/KalmanFilter/src/*.cpp"
)

file(GLOB_RECURSE RSL_COMMAND_SOURCES
    "Command/src/*.c"
    "Command/src/*.cpp"
)


file(GLOB_RECURSE RSL_DEVICE_BAROMETER_SOURCES
    "Device/Barometer/src/*.c"
    "Device/Barometer/src/*.cpp"
)

file(GLOB_RECURSE RSL_DEVICE_BUZZER_SOURCES
    "Device/Buzzer/src/*.c"
    "Device/Buzzer/src/*.cpp"
)

file(GLOB_RECURSE RSL_DEVICE_FLASH_SOURCES
    "Device/Flash/src/*.c"
    "Device/Flash/src/*.cpp"
)

file(GLOB_RECURSE RSL_DEVICE_GNSS_SOURCES
    "Device/Gnss/src/*.c"
    "Device/Gnss/src/*.cpp"
)

file(GLOB_RECURSE RSL_DEVICE_IMU_SOURCES
    "Device/Imu/src/*.c"
    "Device/Imu/src/*.cpp"
)

file(GLOB_RECURSE RSL_DEVICE_LORA_SOURCES
    "Device/Lora/src/*.c"
    "Device/Lora/src/*.cpp"
)


file(GLOB_RECURSE RSL_DRIVER_SPI_SOURCES
    "Driver/Spi/src/*.c"
    "Driver/Spi/src/*.cpp"
)

file(GLOB_RECURSE RSL_DRIVER_TIME_SOURCES
    "Driver/Time/src/*.c"
    "Driver/Time/src/*.cpp"
)

file(GLOB_RECURSE RSL_DRIVER_UART_SOURCES
    "Driver/Uart/src/*.c"
    "Driver/Uart/src/*.cpp"
)

file(GLOB_RECURSE RSL_MIDDLEWARE_LOGGER_SOURCES
    "Middleware/Logger/src/*.c"
    "Middleware/Logger/src/*.cpp"
)

file(GLOB_RECURSE RSL_MIDDLEWARE_COMMAND_SOURCES
    "Middleware/Command/src/*.c"
    "Middleware/Command/src/*.cpp"
)

target_sources(${PROJECT_NAME} PRIVATE
    ${RSL_ALGORITHM_AHRS_SOURCES}
    ${RSL_ALGORITHM_FILTER_SOURCES}
    ${RSL_ALGORITHM_GENERAL_SOURCES}
    ${RSL_ALGORITHM_KALMANFILTER_SOURCES}

    ${RSL_DEVICE_BAROMETER_SOURCES}
    ${RSL_DEVICE_BUZZER_SOURCES}
    ${RSL_DEVICE_FLASH_SOURCES}
    ${RSL_DEVICE_GNSS_SOURCES}
    ${RSL_DEVICE_IMU_SOURCES}
    ${RSL_DEVICE_LORA_SOURCES}

    ${RSL_DRIVER_SPI_SOURCES}
    ${RSL_DRIVER_TIME_SOURCES}
    ${RSL_DRIVER_UART_SOURCES}

    ${RSL_MIDDLEWARE_LOGGER_SOURCES}
    ${RSL_MIDDLEWARE_COMMAND_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    Algorithm/Ahrs/inc
    Algorithm/Filter/inc
    Algorithm/General/inc
    Algorithm/KalmanFilter/inc

    Command/inc

    Device/Barometer/inc
    Device/Buzzer/inc
    Device/Flash/inc
    Device/Gnss/inc
    Device/Imu/inc
    Device/Lora/inc
    Device/define
    Device/GNSS/inc

    Driver/Spi/inc
    Driver/Time/inc
    Driver/Uart/inc
    Middleware/Logger/inc
    Middleware/Command/inc
    Agrement/inc
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    EIGEN_NO_MALLOC
)

target_compile_features(${PROJECT_NAME} PUBLIC
    c_std_11
    cxx_std_17
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -O2
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    stm32cubemx
    CMSISDSP
    eigen
)
