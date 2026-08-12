#include "cmd_handler.hpp"
#include "app_rocket.hpp"
#include "cmd_context.hpp"
namespace Application::Command
{

static constexpr PhaseEntry phaseTable[] =
{
    {"STANDBY",   Rocket::LaunchPhase::STANDBY},
    {"SELF_TEST", Rocket::LaunchPhase::SELF_TEST},
    {"ARMED",     Rocket::LaunchPhase::ARMED},
    {"ASCENT",    Rocket::LaunchPhase::ASCENT},
    {"DESCENT",   Rocket::LaunchPhase::DESCENT},
    {"LANDED",    Rocket::LaunchPhase::LANDED}
};

const char* launchPhaseToString(Rocket::LaunchPhase phase)
{
    switch (phase)
    {
        case Rocket::LaunchPhase::SELF_TEST:
            return "SELF_TEST";

        case Rocket::LaunchPhase::STANDBY:
            return "STANDBY";

        case Rocket::LaunchPhase::ARMED:
            return "ARMED";

        case Rocket::LaunchPhase::ASCENT:
            return "ASCENT";

        case Rocket::LaunchPhase::DESCENT:
            return "DESCENT";

        case Rocket::LaunchPhase::LANDED:
            return "LANDED";

        default:
            return "UNKNOWN";
    }
}

RSL::Command::CommandHandlerResult handlePhaseSet(void* context, size_t argc, const char* const* argv)
{
    if (context == nullptr){
        return RSL::Command::CommandHandlerResult::InvalidState;
    }

    auto* commandContext =
        static_cast<Application::Command::CommandContext*>(context);

    if (commandContext->rocket == nullptr)
    {
        return RSL::Command::CommandHandlerResult::InvalidState;
    }


    if (argc != 1){
        printf("[command]Invalid argument count!\r\n");
        return RSL::Command::CommandHandlerResult::InvalidArgument;   
    }

    for (const auto& entry : phaseTable){
        if(std::strcmp(argv[0], entry.name) == 0){
            if (!commandContext->rocket->setPhase(entry.phase)){
                return RSL::Command::CommandHandlerResult::InvalidState;
            }
            return RSL::Command::CommandHandlerResult::OK;
        }
    }

    return RSL::Command::CommandHandlerResult::InvalidArgument;
    
}

RSL::Command::CommandHandlerResult handlePhaseGet(void* context, size_t argc, const char* const* argv){
    if (context == nullptr){
        return RSL::Command::CommandHandlerResult::InvalidState;
    }
    auto* commandContext = static_cast<Application::Command::CommandContext*>(context);

    if (commandContext->rocket == nullptr)
    {
        return RSL::Command::CommandHandlerResult::InvalidState;
    }

    if (commandContext->source == Application::Command::CommandSource::UART){
        printf("[command] Flight phase is: %s\r\n", launchPhaseToString(commandContext->rocket->getPhase()));
    }

    return RSL::Command::CommandHandlerResult::OK;

}

}