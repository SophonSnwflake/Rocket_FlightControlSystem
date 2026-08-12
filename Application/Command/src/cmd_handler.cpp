#include "cmd_handler.hpp"
#include "app_rocket.hpp"
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

RSL::Command::CommandHandlerResult handlePhaseSet(void* context, size_t argc, const char* const* argv)
{
    Rocket* rocket = static_cast<Rocket*>(context);
    if (context == nullptr){
    return RSL::Command::CommandHandlerResult::InvalidState;
    }

    if (argc != 1){
        printf("[command]Invalid argument count!\r\n");
        return RSL::Command::CommandHandlerResult::InvalidArgument;   
    }

    for (const auto& entry : phaseTable){
        if(std::strcmp(argv[0], entry.name) == 0){
            if (!rocket->setPhase(entry.phase)){
                return RSL::Command::CommandHandlerResult::InvalidState;
            }
            return RSL::Command::CommandHandlerResult::OK;
        }
    }

    return RSL::Command::CommandHandlerResult::InvalidArgument;
    
}

}