#include "app_command.hpp"
#include "cmd_registry.hpp"

RocketCommand::RocketCommand(Rocket& rocket, Application::Command::CommandSource source) :
    m_rocket(rocket),
    m_context{&rocket, source},
    m_engine(
        Application::Command::getRootCommands(),
        Application::Command::getRootCommandCount(),
        resultCallback,
        this
    )
{
}

void RocketCommand::feed(const char* data, size_t length){
    if (data == nullptr || length == 0)return;
    m_engine.feed(data, length, &m_context);
}

void RocketCommand::resultCallback(void* userData, const RSL::Command::CommandEngine::CommandResult& result)
{
    if (userData == nullptr)return;

    RocketCommand* self = static_cast<RocketCommand*>(userData);

    self->handleResult(result);
}

void RocketCommand::handleResult(const RSL::Command::CommandEngine::CommandResult& result)
{
    // TODO:
    // 后续将结果上报给 Command Task
}