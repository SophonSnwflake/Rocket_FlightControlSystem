#include "cmd_handler.hpp"
#include "app_rocket.hpp"
#include "cmd_context.hpp"
namespace Application::Command
{

static constexpr PhaseEntry phaseTable[] =
{
    {"STANDBY",   Rocket::LaunchPhase::STANDBY},
    // {"SELF_TEST", Rocket::LaunchPhase::SELF_TEST},
    {"ARMED",     Rocket::LaunchPhase::ARMED},
    {"ASCENT",    Rocket::LaunchPhase::ASCENT}
    // {"DESCENT",   Rocket::LaunchPhase::DESCENT},
    // {"LANDED",    Rocket::LaunchPhase::LANDED}
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
            if (!commandContext->rocket->setPhaseBetweenSTANDBYandARMED(entry.phase)){
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

RSL::Command::CommandHandlerResult handleFlashErase(void* context, std::size_t argc, const char* const* argv)
{
    auto* ctx =
        static_cast<CommandContext*>(context);

    if (!ctx || !ctx->rocket) {
        return RSL::Command::CommandHandlerResult::InvalidState;
    }

    ctx->pendingAction = PendingAction::FlashErase;

    if (ctx->source == Application::Command::CommandSource::UART){
        printf("WARNING: This will erase the entire flash.\r\n""Are you sure? Type 'yes' or 'no'.\r\n");
    }

    return RSL::Command::CommandHandlerResult::OK;
}

RSL::Command::CommandHandlerResult handleFlashReadAll(void* context, std::size_t argc, const char* const* argv){
    if (context == nullptr){
        return RSL::Command::CommandHandlerResult::InvalidState;
    }
    auto* commandContext = static_cast<Application::Command::CommandContext*>(context);

    if (commandContext->rocket == nullptr)
    {
        return RSL::Command::CommandHandlerResult::InvalidState;
    }
    if (commandContext->source == Application::Command::CommandSource::UART){
        printf("[command] Trying to transmit data through UART...\r\n");
        Rocket::RocketError state;
        state = commandContext->rocket->readAllFlashDataThroughUART();
        if(state != Rocket::RocketError::OK){
            printf("[command] Transmit Failed!\r\n");
            return RSL::Command::CommandHandlerResult::Unsupported;
        }else{
            printf("[command] Transmit Success!\r\n");
            return RSL::Command::CommandHandlerResult::OK;
        }
    }

    return RSL::Command::CommandHandlerResult::OK;
}

RSL::Command::CommandHandlerResult handleYes(void* context, std::size_t argc, const char* const* argv)
{
    auto* ctx =
        static_cast<CommandContext*>(context);

    if (!ctx || !ctx->rocket) {
        return RSL::Command::CommandHandlerResult::InvalidState;
    }

    const PendingAction action = ctx->pendingAction;

    // 先清掉，确保一次确认只能执行一次
    ctx->pendingAction = PendingAction::None;

    switch (action) {

    case PendingAction::FlashErase:
        printf("Erasing flash...\r\n");
        Rocket::RocketError state;
        state = ctx->rocket->eraseAllChipForNewFlight();
        if(state != Rocket::RocketError::OK){
            if (ctx->source == Application::Command::CommandSource::UART){
                printf("Flash erase failed!\r\n");
            }
            return RSL::Command::CommandHandlerResult::Unsupported;
        }

        if (ctx->source == Application::Command::CommandSource::UART){
            printf("Flash erase success!\r\n");
        }
        
        return RSL::Command::CommandHandlerResult::OK;

    case PendingAction::SystemReboot:
        // ctx->rocket->reboot();
        return RSL::Command::CommandHandlerResult::OK;

    case PendingAction::LogErase:
        // return ctx->rocket->eraseLog();

    case PendingAction::None:
    default:
        printf("No operation is waiting for confirmation.\r\n");
        return RSL::Command::CommandHandlerResult::InvalidState;
    }
}

RSL::Command::CommandHandlerResult handleNo(void* context, std::size_t argc, const char* const* argv)
{
    auto* ctx =
        static_cast<CommandContext*>(context);

    if (!ctx) {
        return RSL::Command::CommandHandlerResult::InvalidState;
    }

    if (ctx->pendingAction == PendingAction::None) {
        printf("Nothing to cancel.\r\n");
        return RSL::Command::CommandHandlerResult::InvalidState;
    }

    ctx->pendingAction = PendingAction::None;

    printf("Operation cancelled.\r\n");

    return RSL::Command::CommandHandlerResult::OK;
}

}