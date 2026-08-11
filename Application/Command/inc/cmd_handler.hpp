#pragma once

#include "mid_command.hpp"

namespace Application::Command
{

RSL::Command::CommandHandlerResult handlePhaseSet(
    void* context,
    size_t argc,
    const char* const* argv
);

}