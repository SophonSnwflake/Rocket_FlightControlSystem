#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/* Channel 0 is kept as a tiny terminal channel. SystemView allocates channel 1. */
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS    2u
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS  1u
#define BUFFER_SIZE_UP                   16u
#define BUFFER_SIZE_DOWN                 16u
#define SEGGER_RTT_PRINTF_BUFFER_SIZE    0u

/* Match configMAX_SYSCALL_INTERRUPT_PRIORITY for the STM32F411 project. */
#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY 0x50u

#endif
