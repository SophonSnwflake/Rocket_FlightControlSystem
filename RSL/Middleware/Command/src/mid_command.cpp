#include <cstddef>
#include <cstdint>
#include <cstring>
#include "mid_command.hpp"

namespace RSL::Command
{
CommandEngine::CommandEngine(
    const CommandNode* root,
    size_t rootCount,
    CommandResultCallback resultCallback,
    void* resultUserData):
    m_root(root),
    m_rootCount(rootCount),
    m_linelength(0),
    m_resultCallback(resultCallback),
    m_resultUserData(resultUserData)
{
}

void CommandEngine::feed(const char *data, size_t length, void *context){
    for(size_t i = 0;i < length;i ++){
        processByte(data[i], context);
    }
}

void CommandEngine::processByte(const char data, void *context){
    if(m_receiveState == ReceiveState::Discarding){
        if(data == '\n'){
            m_receiveState = ReceiveState::Receiving;
            m_linelength = 0;
        }
        return;
    }

    // 忽略 CR，兼容 \r\n
    if (data == '\r') return;

    // 收到完整一行
    if (data == '\n')
    {
        m_lineBuffer[m_linelength] = '\0';

        if (m_linelength > 0)
        {
            CommandResult result = processLine(context);
            reportResult(result);
        }

        m_linelength = 0;
        return;
    }

    if (m_linelength >= LINE_BUFFER_SIZE - 1)
    {
        m_receiveState = ReceiveState::Discarding;
        m_linelength = 0;

        CommandResult result;
        result.engineResult = CommandEngineResult::LineTooLong;
        result.handlerResult = CommandHandlerResult::OK;
        result.handlerExecuted = false;

        reportResult(result);

        return;
    }

    m_lineBuffer[m_linelength] = data;
    ++m_linelength;
}

CommandEngine::CommandResult CommandEngine::processLine(void* context){
    size_t argumentCount = 0;

    CommandEngineResult result = Tokenize(argumentCount);

    if (result != CommandEngineResult::OK)
    {
        CommandResult commandResult;
        commandResult.engineResult = result;
        commandResult.handlerResult = CommandHandlerResult::OK;
        commandResult.handlerExecuted = false;

        return commandResult;
    }

    if (argumentCount == 0)
    {
        CommandResult commandResult;
        commandResult.engineResult = CommandEngineResult::OK;
        commandResult.handlerResult = CommandHandlerResult::OK;
        commandResult.handlerExecuted = false;

        return commandResult;
    }

    return Dispatch(argumentCount, context);
}

CommandEngine::CommandEngineResult CommandEngine::Tokenize(size_t& argumentCount)
{
    argumentCount = 0;
    bool inToken = false;

    for (size_t i = 0; i < m_linelength; ++i)
    {
        if (m_lineBuffer[i] == ' ' ||
            m_lineBuffer[i] == '\t')
        {
            m_lineBuffer[i] = '\0';
            inToken = false;
        }
        else if (!inToken)
        {
            if (argumentCount >= MAX_ARGUMENTS)
            {
                return CommandEngineResult::TooManyArguments;
            }

            m_argv[argumentCount] = &m_lineBuffer[i];
            ++argumentCount;

            inToken = true;
        }
    }

    return CommandEngineResult::OK;
}

CommandEngine::CommandResult CommandEngine::Dispatch(size_t argumentCount, void* context){
    const CommandNode* currentNodes = m_root;
    size_t currentCount = m_rootCount;

    for(size_t i = 0; i < argumentCount;++ i){
        const CommandNode* matchedNode = nullptr;

        for(size_t y = 0; y < currentCount; ++ y){
            if(strcmp(m_argv[i], currentNodes[y].name) == 0){
                matchedNode = &currentNodes[y];
                break;
            }
        }

        if (matchedNode == nullptr)
        {
            CommandResult result;
            result.engineResult = CommandEngineResult::UnknownCommand;
            result.handlerResult = CommandHandlerResult::OK;
            result.handlerExecuted = false;

            return result;
        }

        if(matchedNode->handler != nullptr){
            size_t handlerArgc = argumentCount - (i + 1);

            if (handlerArgc < matchedNode->minArgs || handlerArgc > matchedNode->maxArgs)
            {
                CommandResult result;
                result.engineResult = CommandEngineResult::InvalidArgumentCount;
                result.handlerResult = CommandHandlerResult::OK;
                result.handlerExecuted = false;

                return result;
            }

            const char* const* handlerArgv =&m_argv[i + 1];

            CommandHandlerResult handlerResult =
                matchedNode->handler(context, handlerArgc, handlerArgv);

            CommandResult result;
            result.engineResult = CommandEngineResult::OK;
            result.handlerResult = handlerResult;
            result.handlerExecuted = true;

            return result;
        }

        currentNodes = matchedNode->children;
        currentCount = matchedNode->childCount;
    }

    CommandResult result;
    result.engineResult = CommandEngineResult::IncompleteCommand;
    result.handlerResult = CommandHandlerResult::OK;
    result.handlerExecuted = false;

    return result;
}

void CommandEngine::reportResult(const CommandResult& result){
    if (m_resultCallback != nullptr)
    {
        m_resultCallback(m_resultUserData,result);
    }
}

}