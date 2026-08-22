#pragma once

#include "RSL_common.h"
#include "app_rocket.hpp"
#include "mid_command.hpp"
#include "cmd_context.hpp"

class Rocket; 

class RocketCommand final{
public:

private:
    Application::Command::CommandContext m_context;
    RSL::Command::CommandEngine m_engine;
    static void resultCallback(void* userData, const RSL::Command::CommandEngine::CommandResult& result);
    void handleResult(const RSL::Command::CommandEngine::CommandResult& result);

public:
    RocketCommand(Rocket& rocket, Application::Command::CommandSource source);
    ~RocketCommand() = default;
    void feed(const char* data, size_t length);


};