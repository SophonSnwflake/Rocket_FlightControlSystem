# Rocket Flight Control System

An experimental flight control system based on STM32F411 and FreeRTOS. Integrates Attitude solving, GNSS Positionsing, LoRa telemetry and Flight Data Recording.

## Hardware

- STM32F411
- SX1268 LoRa
- BMI088 IMU
- NEO-M9N GNSS
- W25Q128 Flash
- BMP388 Barometer

## Function
- An embedded system based on STM32F411 and FreeRTOS
- IMU: attitude solving
- Barometer: barometer data reading and altitude solving
- GNSS: Positioning
- Flash: Flight recorded data writing
- Logger: Self-Described data coding form and data recording
- Commander: The commander coding and decoding system based on command tree. Support ground station to receiving and transmit command
- Communication Lora: Two-way communication system
- Flight phase monitor and State Machine management
- Parachute deploy control

## Engineering Structure

```text
Rocket_FlightControl/
├── Application/                # Application Level, invoking ground library
│   ├── Rocket/                 # Aircraft's total class
│   └── Logging/                # Logging system's class formation
│   └── Command/                # Command system
│   └── Communication/          # Lora's upper communication, package Lora loop and functions of coding forms.
│
├── RSL/                        # Standard library
│   ├── Agreement/              # Agreement
│   └── Algorithm/              # Mathematical algorithm
|   └── Dependence/             # outside depending library
|   └── Device/                 # Device
|   └── Driver/                 # Driver
|   └── include/                # Universal header files
|   └── Middleware/             # Middleware(Neither belongs to hardware or algorithm)
|   └── CMakeList.txt           # RSL's compiling inventory
│
├── Task/                       # Specific tasks
│   ├── inc/
│   └── src/
│       ├── tsk_rocket.cpp      # Main task loop entry
│       ├── tsk_imu.cpp         # IMU task
│       ├── tsk_isr.cpp         # FreeRTOS's interruption call back functions, not task
│       ├── tsk_logger.cpp      # Logger task
│       ├── ......              # Other tasks
│
│
├── CMakeLists.txt              # Main project compiling inventory
├── CMakePresets.json           # Cmake compiling setting
```

## Compile Command

### Preset

#### Debug Mode, use it when testing
```
cmake --preset Debug
```

#### Release Mode

```
cmake --preset Release
```

### Compile

#### Debug Mode

```
cmake --build build/Debug
```

#### Release Mode

```
cmake --build build/Release
```

### Program

#### ST-Link

```
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Debug/rocket_26.elf verify reset exit"
```

## Current Schedule

This avionics system has been tested on Oct 3 2026, using Qiong Yu Aerospace's small rocket near the Yellow River. This flight confirmed that all systems functioned properly and received flight data successfully. Flight log and related information have been uploaded to the documentation site：

https://rocket26.wiki/flight_records/Oct_3_2026/

## Future Plans

Building on this avionics system's software and hardware platform, future development could take three directions:

1：INS & Sensor Fusion algorithm 

2：Flight Control Algorithm's simulation and development

3：Upper machine's Visual Recognition and communication between upper and lower machine

Now I am learning INS's related algorithm literature. Plan to build a exportable demo based on STM32 in the middle of September. Expected using Error Extended Kalman Filter & Attitude Soft Couping.

## TimeLine

- Oct 5–18: Study the complete inertial navigation modeling process by working through the derivations in the literature. Goal: independently derive the full mathematical model with the help of my notes.

- Oct 19–21: Review the code structure and coding style of existing C/C++ inertial navigation projects, and select the most representative one as a reference for subsequent development.

- Oct 22–28: Begin implementing an inertial navigation library within the existing RSL codebase.

- Oct 29 – mid-November: Debug and test the implementation.

Hopefully everything will go according to plan... I’m not sure how busy things will get toward the end of the semester, or whether anything else important will come up.
