#pragma once

#include "RSL_common.h"
#include <stdio.h>
// #include <cstdint>
#include "dvc_flash.hpp"

namespace RocketLog
{
class RocketLogger final{
public:
    enum class FlashLogError{
        OK = 0,
        NotPrepared,
        InvalidArgument,
        StorageFull,
        FlashError
    };
private:
    Flash *m_flash;
    Flash::Result m_lastFlashResult = Flash::Result::OK; // 最近一次底层 Flash 操作返回的结果

private:  
    static constexpr uint32_t BufferSize = 256U;
    uint32_t m_flashCapacity;          // flash总容量字节
    uint8_t m_buffer[BufferSize];
    uint32_t m_writeAddress = 0U;      // 下一次向 Flash 实际写入数据的起始地址
    // uint32_t m_bytesWritten = 0U;   // 本次日志中已经成功写入 Flash 的总字节数 (本次飞行任务采用单个芯片存储全部数据的模式，所以删掉这个重复变量。Wang 26-8-6)
    uint16_t m_bufferedLength = 0U;    // 当前 RAM 缓冲区中尚未写入 Flash 的字节数
    bool m_isPrepared        = false;  // 是否已经准备好（是否已经全片擦除）

public:
    RocketLogger(Flash *flash);
    ~RocketLogger() = default;

    FlashLogError prepareNewFlight();
    FlashLogError append(uint8_t *data, uint32_t length);
    FlashLogError flush();
    bool isPrepared() const {return m_isPrepared;}
    uint32_t bytesWritten() const {return m_writeAddress;}
    uint32_t bytesBuffered() const {return m_bufferedLength;}
    uint32_t bytesAccepted() const {return m_writeAddress + m_bufferedLength;}
    uint32_t remainingCapacity() const;
    Flash::Result lastFlashResult() const;

    
    


};

} // namespace RocketLog