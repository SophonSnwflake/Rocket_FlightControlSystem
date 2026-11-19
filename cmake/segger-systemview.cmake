option(ROCKET_ENABLE_SYSTEMVIEW "Enable SEGGER SystemView tracing" ON)
option(
    ROCKET_SYSTEMVIEW_POST_MORTEM
    "Use the post-mortem RAM ring buffer instead of J-Link live RTT recording"
    OFF
)

if(ROCKET_ENABLE_SYSTEMVIEW)
    if(ROCKET_SYSTEMVIEW_POST_MORTEM)
        set(ROCKET_SYSTEMVIEW_POST_MORTEM_VALUE 1)
        message(STATUS "SEGGER SystemView mode: post-mortem RAM ring buffer")
    else()
        set(ROCKET_SYSTEMVIEW_POST_MORTEM_VALUE 0)
        message(STATUS "SEGGER SystemView mode: J-Link live RTT")
    endif()

    set(SEGGER_RTT_ROOT
        ${CMAKE_CURRENT_LIST_DIR}/../RSL/Dependence/SEGGER/RTT-8.58.0
    )
    set(SEGGER_SYSVIEW_ROOT
        ${CMAKE_CURRENT_LIST_DIR}/../RSL/Dependence/SEGGER/SystemView-4.12.0
    )
    set(ROCKET_SYSVIEW_ROOT
        ${CMAKE_CURRENT_LIST_DIR}/../RSL/Middleware/SystemView
    )

    add_library(segger_systemview_headers INTERFACE)
    target_include_directories(segger_systemview_headers INTERFACE
        ${SEGGER_RTT_ROOT}/RTT
        ${SEGGER_SYSVIEW_ROOT}/SEGGER
        ${SEGGER_SYSVIEW_ROOT}/SYSVIEW
        ${ROCKET_SYSVIEW_ROOT}/config
        ${ROCKET_SYSVIEW_ROOT}/freertos
    )
    target_compile_definitions(segger_systemview_headers INTERFACE
        ROCKET_ENABLE_SYSTEMVIEW=1
        SEGGER_SYSVIEW_POST_MORTEM_MODE=${ROCKET_SYSTEMVIEW_POST_MORTEM_VALUE}
    )

    add_library(segger_systemview STATIC)
    target_sources(segger_systemview PRIVATE
        ${SEGGER_RTT_ROOT}/RTT/SEGGER_RTT.c
        ${SEGGER_SYSVIEW_ROOT}/SYSVIEW/SEGGER_SYSVIEW.c
        ${ROCKET_SYSVIEW_ROOT}/freertos/SEGGER_SYSVIEW_FreeRTOS.c
        ${ROCKET_SYSVIEW_ROOT}/config/SEGGER_SYSVIEW_Config_FreeRTOS.c
    )
    target_link_libraries(segger_systemview
        PUBLIC segger_systemview_headers
        PRIVATE stm32cubemx
    )

    # FreeRTOS is an independent OBJECT target. It includes FreeRTOSConfig.h,
    # which in turn includes the SystemView trace macros, so it needs the
    # adapter include directory and feature definition directly.
    target_link_libraries(FreeRTOS PRIVATE segger_systemview_headers)

    # CubeMX-generated application sources also include FreeRTOSConfig.h.
    target_link_libraries(stm32cubemx INTERFACE segger_systemview_headers)

    # The CubeMX-generated CMake file uses the plain link signature for the
    # executable, so keep the same signature here.
    target_link_libraries(${CMAKE_PROJECT_NAME} segger_systemview)
endif()
