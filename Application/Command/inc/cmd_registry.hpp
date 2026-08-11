#pragma once

#include "RSL_common.h"
#include "mid_command.hpp"

namespace Application::Command
{

const RSL::Command::CommandNode* getRootCommands();

size_t getRootCommandCount();

}