#pragma once

#include "mid_command.hpp"
#include "app_rocket.hpp"

namespace Application::Command
{

struct PhaseEntry
{
    const char* name;
    Rocket::LaunchPhase phase;
};

RSL::Command::CommandHandlerResult handlePhaseSet(
    void* context,
    size_t argc,
    const char* const* argv
);

}