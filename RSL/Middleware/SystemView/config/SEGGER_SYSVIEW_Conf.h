#ifndef SEGGER_SYSVIEW_CONF_H
#define SEGGER_SYSVIEW_CONF_H

/* SystemView uses RTT up-buffer 1 when channel 0 is selected here. */
#define SEGGER_SYSVIEW_RTT_CHANNEL       0
#define SEGGER_SYSVIEW_RTT_BUFFER_SIZE   4096

/* Keep only the newest trace data in RAM for halted-target export. */
#define SEGGER_SYSVIEW_POST_MORTEM_MODE  1
#define SEGGER_SYSVIEW_SYNC_PERIOD_SHIFT 8

/* DWT CYCCNT divided by 16: 6.25 MHz at the configured 100 MHz CPU clock. */
#define SEGGER_SYSVIEW_GET_TIMESTAMP()   ((*(volatile U32 *)0xE0001004u) >> 4)
#define SEGGER_SYSVIEW_TIMESTAMP_BITS    28

#define SEGGER_SYSVIEW_ID_BASE           0x20000000u
#define SEGGER_SYSVIEW_USE_STATIC_BUFFER 1
#define SEGGER_SYSVIEW_MAX_STRING_LEN    64
#define SEGGER_SYSVIEW_MAX_ARGUMENTS     8

#endif
