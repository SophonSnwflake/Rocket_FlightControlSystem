#include "FreeRTOS.h"
#include "SEGGER_SYSVIEW.h"

extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;

#define SYSVIEW_APP_NAME       "Rocket Flight Control"
#define SYSVIEW_DEVICE_NAME    "STM32F411CE"
#define SYSVIEW_TIMESTAMP_FREQ (configCPU_CLOCK_HZ / 16u)
#define SYSVIEW_CPU_FREQ       configCPU_CLOCK_HZ
#define SYSVIEW_RAM_BASE       0x20000000u

#if SEGGER_RTT_MAX_INTERRUPT_PRIORITY != configMAX_SYSCALL_INTERRUPT_PRIORITY
#error "SEGGER RTT lock priority must match the FreeRTOS syscall interrupt priority"
#endif

static void SendSystemDescription(void)
{
    SEGGER_SYSVIEW_SendSysDesc("N=" SYSVIEW_APP_NAME ",D=" SYSVIEW_DEVICE_NAME ",O=FreeRTOS");
    SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick");
}

void SEGGER_SYSVIEW_Conf(void)
{
    SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ,
                        SYSVIEW_CPU_FREQ,
                        &SYSVIEW_X_OS_TraceAPI,
                        SendSystemDescription);
    SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}
