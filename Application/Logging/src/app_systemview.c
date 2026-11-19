#include "app_systemview.h"

#if defined(ROCKET_ENABLE_SYSTEMVIEW)
#include "FreeRTOS.h"
#include "SEGGER_SYSVIEW.h"
#include "drv_time.h"
#endif

void AppSystemView_Init(void)
{
#if defined(ROCKET_ENABLE_SYSTEMVIEW)
    DWT_Init();
    SEGGER_SYSVIEW_Conf();
#if SEGGER_SYSVIEW_POST_MORTEM_MODE == 1
    SEGGER_SYSVIEW_Start();
#endif
#endif
}

void AppSystemView_SampleTaskStacks(void)
{
#if defined(ROCKET_ENABLE_SYSTEMVIEW)
    if (SEGGER_SYSVIEW_IsStarted() != 0)
    {
        SYSVIEW_UpdateTaskStackUsage();
    }
#endif
}

void AppSystemView_Stop(void)
{
#if defined(ROCKET_ENABLE_SYSTEMVIEW)
    SEGGER_SYSVIEW_Stop();
#endif
}
