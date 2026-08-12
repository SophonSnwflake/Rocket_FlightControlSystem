#include <iterator>
#include "cmd_registry.hpp"
#include "cmd_handler.hpp"

namespace Application::Command
{
using RSL::Command::CommandNode;


static const CommandNode phaseCommands[] =
{
{
    "set",
    "Phase Set Command",
    "set <command>",
    handlePhaseSet,
    nullptr,
    0,
    1,
    1
    }
};

static const CommandNode rocketCommands[] =
{
    {
    "phase",
    "phase Command",
    "phase <command>",
    nullptr,
    phaseCommands,
    std::size(phaseCommands),
    0,
    0
    }

};

static const CommandNode rootCommands[] =
{
    {
        "rocket",
        "Rocket commands",
        "rocket <command>",
        nullptr,
        rocketCommands,
        std::size(rocketCommands),
        0,
        0
    }
};

const CommandNode* getRootCommands()
{
    return rootCommands;
}

size_t getRootCommandCount()
{
    return std::size(rootCommands);
}

}