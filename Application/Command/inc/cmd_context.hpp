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

enum class PendingAction : uint8_t {
    None = 0,
    FlashErase,
    SystemReboot,
    LogErase
};

struct CommandContext
{
    Rocket* rocket;
    CommandSource source;
    PendingAction pendingAction = PendingAction::None;
};

}