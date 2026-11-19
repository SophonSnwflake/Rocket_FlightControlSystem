#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/* Channel 0 is kept as a tiny terminal channel. SystemView allocates channel 1. */
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS    2u
#if defined(SEGGER_SYSVIEW_POST_MORTEM_MODE) && SEGGER_SYSVIEW_POST_MORTEM_MODE == 1
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS  1u
#else
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS  2u
#endif
#define BUFFER_SIZE_UP                   16u
#define BUFFER_SIZE_DOWN                 16u
#define SEGGER_RTT_PRINTF_BUFFER_SIZE    0u

/* The optional SEGGER_RTT_ASM_ARMv7M.S file is not vendored in this project. */
#define RTT_USE_ASM                     0

/* Match configMAX_SYSCALL_INTERRUPT_PRIORITY for the STM32F411 project. */
#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY 0x50u

#endif
