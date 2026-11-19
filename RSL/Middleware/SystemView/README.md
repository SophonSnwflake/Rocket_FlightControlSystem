# SEGGER SystemView target integration

This directory contains project-owned configuration and the FreeRTOS V10
adapter for the STM32F411 flight-control firmware. Upstream, unmodified SEGGER
sources are kept separately under `RSL/Dependence/SEGGER/`.

## Selected mode

- SystemView target source: 4.12.0
- RTT target source: 8.58.0
- FreeRTOS kernel: 10.3.1, GCC ARM_CM4F port
- Default recording: continuous J-Link RTT with host-controlled start and stop
- Fallback recording: 4 KiB post-mortem RAM ring buffer selected by CMake
- Timestamp: DWT CYCCNT divided by 16 (6.25 MHz at 100 MHz CPU clock)
- Live probe workflow: run with J-Link, then connect and record from SystemView

RTT channel 0 is reserved for the terminal, and channel 1 carries SystemView
traffic in both directions. The terminal buffers are intentionally only 16
bytes. SystemView allocates its own 4 KiB up-buffer and an 8-byte command
down-buffer in live mode. The optional RTT assembly implementation is not
vendored, so the project selects the official C fallback implementation.

## Build integration

`cmake/segger-systemview.cmake` creates:

- `segger_systemview_headers`: configuration macros and include paths shared by
  every target that includes `FreeRTOSConfig.h`;
- `segger_systemview`: the four target-side source files;
- a direct dependency from the independent `FreeRTOS` object target to the
  trace headers.

SystemView is enabled in live RTT mode by default:

```sh
cmake --preset Debug -DROCKET_ENABLE_SYSTEMVIEW=ON -DROCKET_SYSTEMVIEW_POST_MORTEM=OFF
cmake --build --preset Debug
```

Select the original post-mortem ring buffer for an ST-LINK session with:

```sh
cmake --preset Debug -DROCKET_ENABLE_SYSTEMVIEW=ON -DROCKET_SYSTEMVIEW_POST_MORTEM=ON
cmake --build --preset Debug
```

SystemView can be completely disabled for a comparison build:

```sh
cmake --preset Debug -DROCKET_ENABLE_SYSTEMVIEW=OFF
cmake --build --preset Debug
```

With SystemView enabled, `AppSystemView_Init()` starts DWT and initializes RTT
before CubeMX creates the application tasks. Live recording starts only after
the SystemView host sends its start command. Post-mortem recording still starts
immediately during initialization.

## Task stack monitoring

FreeRTOS records the high address of every task stack while SystemView is
enabled. The SystemView adapter stores the exact stack capacity in bytes and a
cached peak usage value for each registered task. `LOG` refreshes the cache once
per second with `uxTaskGetStackHighWaterMark()` from normal task context.

When a sampled peak changes, the LOG task sends updated task information so a
live recording refreshes its Stack Information column. The post-mortem metadata
callback only transmits cached values; it does not scan task stacks from a
scheduler or interrupt trace path. SystemView is given standard stack fields
with these units:

- `StackBase`: lowest stack address;
- `StackSize`: allocated capacity in bytes;
- `StackUsage`: peak usage in bytes since task creation.

The idle task is intentionally represented by SystemView's idle context and is
not included in the task metadata table. The Cortex-M MSP/interrupt stack is
also outside this FreeRTOS task-stack monitor.

## Live recording with J-Link

1. Configure and build the live mode shown above.
2. Flash and run the target with either `J-Link USB + SystemView 实时追踪` or
   `J-Link Wi-Fi + SystemView 实时追踪` in Cortex-Debug. Continue execution if
   the debugger stops after reset.
3. Create a SystemView project and add `build/Debug/rocket_26.elf` as the
   application ELF file.
4. Select the J-Link recorder, `STM32F411CE`, SWD, and a 4 MHz interface speed.
   Select the configured USB serial number or the J-Link IP address.
5. Locate the RTT control block from the ELF file, or use automatic detection
   after `AppSystemView_Init()` has executed.
6. Start and stop recording from SystemView. The commands travel to the target
   through RTT down-channel 1; events return through up-channel 1.

The dedicated Cortex-Debug profiles disable Live Watch because frequent memory
polling competes with RTT traffic. If SystemView reports overflow events, first
keep Live Watch disabled, confirm a stable 4 MHz or faster SWD connection, and
then reduce the recorded event rate if necessary. Once the connection is
verified, the 4 KiB buffer may be reduced to reclaim RAM.

## Halted-target export with ST-LINK

This workflow is valid only for a build configured with
`ROCKET_SYSTEMVIEW_POST_MORTEM=ON`.

Quick reference for the Cortex-Debug GDB console:

```gdb
# First click Pause in Cortex-Debug; do not reset the target.
source RSL/Middleware/SystemView/scripts/dump-systemview.gdb
# Resume with the Cortex-Debug Continue button after the file is written.
```

For a command-line GDB session, the equivalent sequence is:

```gdb
interrupt
source RSL/Middleware/SystemView/scripts/dump-systemview.gdb
continue
```

The `source` command cannot read the buffer while the target is running. Close
any previously loaded `build/Debug/systemview-postmortem.bin` recording in
SystemView before exporting again; otherwise GDB may report `Permission
denied` when it tries to overwrite the file.

1. Run the firmware under Cortex-Debug and let the event of interest occur.
2. Pause the target without resetting it. A reset destroys the RAM recording.
3. In the Cortex-Debug GDB console, run:

   ```gdb
   source RSL/Middleware/SystemView/scripts/dump-systemview.gdb
   ```

4. Load `build/Debug/systemview-postmortem.bin` with **File > Load Recording**
   in SystemView.

The script reads RTT up-buffer 1 through `_SEGGER_RTT`, uses its live read and
write offsets, and rotates wrapped data into chronological order. It must only
be run while the target is halted.

## FreeRTOS 10.3.1 maintenance

The SEGGER adapter shipped in the `Sample/FreeRTOSV10` directory is the closest
match, but its reference patch targets an older V10 kernel. The project-specific
changes are saved in `patches/freertos-10.3.1-systemview.patch` and cover:

- new no-op trace hooks in `FreeRTOS.h`;
- delayed/suspended state transitions and ready-list reinsertion in `tasks.c`;
- balanced SysTick ISR entry/exit events in the GCC ARM_CM4F `port.c`.

The task metadata table currently has eight entries. The generated project uses
five application tasks plus the timer task; the idle task is skipped, leaving
two spare entries. Increase `SYSVIEW_FREERTOS_MAX_NOF_TASKS` before adding more
tasks.

## CubeMX regeneration

- `freertos.c` calls and includes are inside `USER CODE` regions.
- `FreeRTOSConfig.h` includes the adapter inside its `USER CODE Defines` region.
- Everything under `RSL/Dependence/SEGGER`, `RSL/Middleware/SystemView`, and
  `cmake/` is project-maintained and is not generated by CubeMX.
- CubeMX may replace the three patched FreeRTOS kernel/port files. After every
  middleware regeneration, compare the kernel version and reapply or port the
  saved patch; never replace the complete 10.3.1 files with another V10 release.

## Remaining hardware validation

Compilation and static RAM placement are verified. A bench run must still
confirm that the installed SystemView desktop version can control and decode
the live 4.12.0 target stream over both the USB and Wi-Fi J-Link paths, that no
overflow events occur at the chosen SWD speed, and that task and interrupt
stack margins remain safe with tracing enabled.
