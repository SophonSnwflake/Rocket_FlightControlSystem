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
    },

    {
    "get",
    "Phase Get Command",
    "get <command>",
    handlePhaseGet,
    nullptr,
    0,
    0,
    0
    }
};

static const CommandNode loggerCommands[] =
{
{
    "eraseall",
    "[DANGER!]EraseALLChip",
    "eraseall",
    handleFlashErase,
    nullptr,
    0,
    0,
    0
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
    },
    {
    "logger",
    "logger Command",
    "logger <command>",
    nullptr,
    loggerCommands,
    std::size(loggerCommands),
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
    },
    {
        "yes",
        "Confirm pending operation",
        "yes",
        handleYes,
        nullptr,
        0,
        0,
        0
    },
    {
        "no",
        "Cancel pending operation",
        "no",
        handleNo,
        nullptr,
        0,
        0,
        0
    },
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