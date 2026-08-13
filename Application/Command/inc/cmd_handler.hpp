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

RSL::Command::CommandHandlerResult handlePhaseSet(void* context, size_t argc, const char* const* argv);

RSL::Command::CommandHandlerResult handlePhaseGet(void* context, size_t argc, const char* const* argv);

RSL::Command::CommandHandlerResult handleFlashErase(void* context, std::size_t argc, const char* const* argv);

RSL::Command::CommandHandlerResult handleFlashReadAll(void* context, std::size_t argc, const char* const* argv);

RSL::Command::CommandHandlerResult handleYes(void* context, std::size_t argc, const char* const* argv);

RSL::Command::CommandHandlerResult handleNo(void* context, std::size_t argc, const char* const* argv);

}