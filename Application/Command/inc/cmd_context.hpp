#pragma once

#include <cstdint>

class Rocket;   // 前置声明

namespace Application::Command
{

enum class CommandSource : uint8_t
{
    UART,
    LoRa
};

struct CommandContext
{
    Rocket* rocket;
    CommandSource source;
};

}