#pragma once

// #include <cstdint>
#include "RSL_common.h"
#include "mid_command_types.hpp"

namespace RSL::Command
{

class CommandEngine{
public:
    static constexpr size_t LINE_BUFFER_SIZE = 256U;
    static constexpr size_t MAX_ARGUMENTS = 8U;

    enum class CommandEngineResult : uint8_t
    {
        OK = 0,

        LineTooLong,
        TooManyArguments,
        UnknownCommand,
        IncompleteCommand,
        InvalidArgumentCount
    };

    enum class ReceiveState : uint8_t
    {
        Receiving,
        Discarding
    };

private:
    const CommandNode* m_root; // 命令树
    size_t m_rootCount;    
    ReceiveState m_receiveState = ReceiveState::Receiving;

private:
    char m_lineBuffer[LINE_BUFFER_SIZE]{};
    char* m_argv[MAX_ARGUMENTS]{};
    size_t m_linelength = 0; // 当前缓冲区存了多少有效字节

public:
    CommandEngine(const CommandNode* root, size_t rootCount);
    ~CommandEngine() = default;
    void feed(const char *data, size_t length, void *context); // 给回调函数用的通用context接口

private:
    CommandEngineResult processByte(const char data, void *context); 
    CommandEngineResult processLine(void* context);

    // 解析器
    CommandEngineResult Tokenize(size_t& argumentCount);
    CommandEngineResult Dispatch(size_t argumentCount, void* context);   // 调度器

    
};


}