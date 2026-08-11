#include "cmd_handler.hpp"
#include "app_rocket.hpp"
namespace Application::Command
{

RSL::Command::CommandHandlerResult handlePhaseSet(void* context, size_t argc, const char* const* argv)
{
    Rocket* rocket =
        static_cast<Rocket*>(context);

    // argv[0] -> "boost"
    // 根据字符串转换为 RocketPhase
    // rocket->setPhase(...)

    return RSL::Command::CommandHandlerResult::OK;
}

}