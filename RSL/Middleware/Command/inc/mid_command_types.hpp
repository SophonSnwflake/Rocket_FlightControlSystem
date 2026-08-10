/**
 * @file mid_command_types.hpp
 * @brief Command 中间件公共类型定义
 *
 * @details
 * 本文件用于定义 RSL Command 中间件对外共享的基础数据类型，
 * 包括命令处理结果、命令处理函数类型以及命令树节点结构。
 *
 * 这些类型仅描述“命令系统的通用接口和数据结构”，
 * 不包含命令解析、字符接收、命令查找或具体业务逻辑的实现。
 *
 * 本文件中的定义应保持与具体应用解耦，不应依赖 Rocket、
 * FreeRTOS、UART、LoRa、Logger、IMU 等具体业务或硬件模块，
 * 以保证 Command 中间件能够在其他嵌入式工程中复用。
 *
 * 主要包含：
 * - CommandHandlerResult：命令处理函数的执行结果
 * - CommandHandler：命令处理函数的统一函数指针类型
 * - CommandNode：静态命令树节点描述结构
 *
 * @note
 * CommandEngine 的运行时状态，例如行缓冲区、参数数组、
 * 解析状态等，不应定义在本文件中，而应作为 CommandEngine
 * 的内部实现细节进行管理。
 */

#pragma once

#include "RSL_common.h"

namespace RSL::Command
{

enum class CommandHandlerResult : uint8_t
{
    OK = 0,          // 命令执行成功

    InvalidArgument, // 参数内容非法
    InvalidState,    // 当前系统状态不允许执行
    Busy,            // 目标模块当前忙，可稍后重试
    NotAllowed,      // 权限或来源不允许执行
    Unsupported,     // 当前系统不支持该操作
    ExecutionFailed  // 底层执行失败
};

using CommandHandler = CommandHandlerResult (*)(
    void* context,
    size_t argc,             // 收到了几个命令后的用户参数？
    const char* const* argv  // 指向参数字符串的指针
);

// 命令树节点描述
struct CommandNode
{
    const char* name;             // 节点名称
    const char* description;      // 命令描述
    const char* usage;            // 参数用法

    CommandHandler handler;       // 非 nullptr 表示该节点可以执行

    const CommandNode* children;  // 子节点数组
    size_t childCount;            // 子节点数量

    uint8_t minArgs;              // Handler 最少参数数量
    uint8_t maxArgs;              // Handler 最多参数数量
};

} // namespace RSL::Command