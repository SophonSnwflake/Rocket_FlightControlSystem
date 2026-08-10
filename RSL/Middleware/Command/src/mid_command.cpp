#include <cstddef>
#include <cstdint>
#include "mid_command.hpp"
#include <cstring>

namespace RSL::Command
{
CommandEngine::CommandEngine(const CommandNode* root, size_t rootCount) :
    m_root(root),
    m_rootCount(rootCount),
    m_linelength(0)
{
}

void CommandEngine::feed(const char *data, size_t length, void *context){
    for(size_t i = 0;i < length;i ++){
        processByte(data[i], context);
    }
}

CommandEngine::CommandEngineResult CommandEngine::processByte(const char data, void *context){
    if(m_receiveState == ReceiveState::Discarding){
        if(data == '\n'){
            m_receiveState = ReceiveState::Receiving;
            m_linelength = 0;
        }
        return CommandEngineResult::OK;
    }

    // 忽略 CR，兼容 \r\n
    if (data == '\r') return CommandEngineResult::OK;

    // 收到完整一行
    if (data == '\n')
        {
            m_lineBuffer[m_linelength] = '\0';

            CommandEngineResult result =
                CommandEngineResult::OK;

            if (m_linelength > 0)
            {
                result = processLine(context);
            }

            m_linelength = 0;
            return result;
        }

    if (m_linelength >= LINE_BUFFER_SIZE - 1)
    {
        m_receiveState = ReceiveState::Discarding;
        m_linelength = 0;
        return CommandEngineResult::LineTooLong;
    }

    m_lineBuffer[m_linelength] = data;
    ++m_linelength;
    return CommandEngineResult::OK;
}
 
CommandEngine::CommandEngineResult CommandEngine::processLine(void* context){
    size_t argumentCount = 0;

    CommandEngineResult result = Tokenize(argumentCount);

    if (result != CommandEngineResult::OK)return result;

    if (argumentCount == 0)return CommandEngineResult::OK;

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

CommandEngine::CommandEngineResult CommandEngine::Dispatch(size_t argumentCount, void* context){
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

        if (matchedNode == nullptr) return CommandEngineResult::UnknownCommand;

        if(matchedNode->handler != nullptr){
            size_t handlerArgc = argumentCount - (i + 1);
            if (handlerArgc < matchedNode->minArgs || handlerArgc > matchedNode->maxArgs)
            {
                return CommandEngineResult::InvalidArgumentCount;
            }

            const char* const* handlerArgv =&m_argv[i + 1];
            CommandHandlerResult handlerResult = matchedNode->handler(context, handlerArgc, handlerArgv);
            return CommandEngineResult::OK;
        }

        currentNodes = matchedNode->children;
        currentCount = matchedNode->childCount;
        
    }
    return CommandEngineResult::IncompleteCommand;
}

}