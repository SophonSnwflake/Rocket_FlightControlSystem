/**
 * @file    mid_command.cpp
 * @brief   CommandEngine 命令解析与调度实现
 *
 * @details
 * 负责接收连续字符流，将字符组装成完整命令行，
 * 对命令行进行分词，并根据静态 CommandNode 命令树完成查找和调度。
 * 当命令解析完成、发生解析错误或 Handler 执行结束时，
 * 通过 CommandResultCallback 向上层报告结构化执行结果。
 *
 * 本模块仅负责通用命令解析与调度机制，
 * 不依赖具体 UART、LoRa、FreeRTOS 或 Rocket 业务逻辑。
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "mid_command.hpp"

namespace RSL::Command
{

/**
 * @brief  构造 CommandEngine
 * @param  root 命令树根层节点数组首地址
 * @param  rootCount 根层节点数量
 * @param  resultCallback 命令结果回调函数
 * @param  resultUserData 传递给结果回调的用户数据指针
 *
 * CommandEngine 不拥有命令树和 userData，
 * 因此它们的生命周期必须覆盖 CommandEngine 的使用周期。
 */
CommandEngine::CommandEngine(const CommandNode* root,
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

/**
 * @brief  向命令引擎输入一段连续字符数据
 * @param  data 输入数据首地址
 * @param  length 输入数据长度
 * @param  context 传递给命令 Handler 的运行上下文
 *
 * 输入数据可以是一条命令的一部分，也可以包含多条命令。
 * 本函数逐字节调用 processByte()，命令边界由换行符确定。
 */
void CommandEngine::feed(const char *data, size_t length, void *context){
    for(size_t i = 0;i < length;i ++){
        processByte(data[i], context);
    }
}

/**
 * @brief  处理一个输入字符
 * @param  data 当前输入字符
 * @param  context 传递给命令 Handler 的运行上下文
 *
 * 负责命令行组装、CR/LF 处理以及超长命令检测。
 * 收到换行符后调用 processLine() 处理完整命令，
 * 并通过 reportResult() 上报结果。
 * 超长命令会进入 Discarding 状态直到下一次换行。
 */
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

/**
 * @brief  处理一条已经接收完整的命令行
 * @param  context 传递给命令 Handler 的运行上下文
 * @return 当前命令的完整 CommandResult
 *
 * 首先调用 Tokenize() 将命令行拆分为 argv，
 * 分词成功后调用 Dispatch() 搜索命令树并执行 Handler。
 * 若分词失败，则直接构造对应的命令结果返回。
 */
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

/**
 * @brief  将当前命令行拆分为多个字符串参数
 * @param  argumentCount 输出实际得到的 token 数量
 * @return 分词结果状态
 *
 * 本函数直接修改 m_lineBuffer，
 * 将空格和制表符替换为 '\0'，
 * 并让 m_argv[] 指向每个 token 的起始位置。
 * 不进行字符串复制或动态内存分配。
 */
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

/**
 * @brief  根据 argv 在命令树中查找并执行目标命令
 * @param  argumentCount 当前 argv 中的 token 数量
 * @param  context 传递给命令 Handler 的运行上下文
 * @return 当前命令的完整 CommandResult
 *
 * 从根节点开始逐层匹配命令 token。
 * 找到 Handler 后检查剩余参数数量并调用 Handler。
 * 同时记录 Engine 调度结果和 Handler 执行结果。
 */
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

/**
 * @brief  向上层报告一条命令的处理结果
 * @param  result 待上报的命令结果
 *
 * 若注册了 CommandResultCallback，
 * 则调用回调函数并传入用户数据和 CommandResult。
 * 本函数不负责结果的打印、传输或持久化。
 */
void CommandEngine::reportResult(const CommandResult& result){
    if (m_resultCallback != nullptr)
    {
        m_resultCallback(m_resultUserData,result);
    }
}

}