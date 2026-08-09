#include "mid_logger_writer.hpp"

namespace RocketLog
{

RocketLogWriter::RocketLogWriter(Flash *flash) :
    m_writeAddress(0U),
    // m_bytesWritten(0U),
    m_bufferedLength(0U),
    m_lastFlashResult(Flash::Result::OK),
    m_isPrepared(false),
    m_flash(flash)
{
}

RocketLogWriter::FlashLogError RocketLogWriter::prepareNewFlight(){
    Flash::Result result;
    Flash::Geometry temGeometry;
    temGeometry = m_flash->geometry();
    const uint32_t m_flashCapacity = temGeometry.capacity;
    result = m_flash->erase(0U, m_flashCapacity);
    m_lastFlashResult = result;
    if (result != Flash::Result::OK){
        return FlashLogError::FlashError;
    }
    m_writeAddress = 0U;
    // m_bytesWritten = 0U;
    m_bufferedLength = 0U;
    m_isPrepared = true;
    memset(m_buffer, 0U, BufferSize);
    return FlashLogError::OK;
}

RocketLogWriter::FlashLogError RocketLogWriter::append(uint8_t *data, uint32_t length){
    // 状态参量
    FlashLogError state;

    // 输入值防呆检测
    if(m_isPrepared != true) return FlashLogError::NotPrepared;
    if(data == nullptr || length == 0) return FlashLogError::InvalidArgument;
    if (m_bufferedLength > BufferSize)return FlashLogError::FlashError;

    // 临时变量
    uint8_t *dataptr;
    uint8_t *bufferptr;
    uint16_t leftBufferLength;
    uint32_t leftDataLength;

    // 初始化临时变量
    bufferptr = m_buffer;
    dataptr = data;
    leftBufferLength = BufferSize - m_bufferedLength;
    leftDataLength = length;

    // 循环压弹发射
    while(leftDataLength > leftBufferLength){
        memcpy(bufferptr + m_bufferedLength, dataptr, leftBufferLength);
        m_bufferedLength += leftBufferLength;
        state = flush();
        if(state != FlashLogError::OK) return state;
        dataptr += leftBufferLength;
        leftDataLength -= leftBufferLength;
        leftBufferLength = BufferSize - m_bufferedLength;
    }
    memcpy(bufferptr + m_bufferedLength, dataptr, leftDataLength);
    m_bufferedLength += leftDataLength;

    if (m_bufferedLength == BufferSize)return flush();
    return FlashLogError::OK;

}

RocketLogWriter::FlashLogError RocketLogWriter::flush(){
    if(m_bufferedLength == 0) return FlashLogError::OK;
    if(m_writeAddress >= m_flashCapacity)return FlashLogError::StorageFull;

    const uint32_t remainingCapacity =
    m_flashCapacity - m_writeAddress;

    if (m_bufferedLength > remainingCapacity)
    {
        return FlashLogError::StorageFull;
    }

    uint32_t temWriteLength;
    if(m_bufferedLength + m_writeAddress > m_flashCapacity){
        return FlashLogError::FlashError;
    }else{
        temWriteLength = m_bufferedLength;
    }
    const Flash::Result result =m_flash->program(m_writeAddress, m_buffer, temWriteLength);
    if (result != Flash::Result::OK) return FlashLogError::FlashError;
    m_writeAddress += temWriteLength;
    m_bufferedLength = 0;
    return FlashLogError::OK;
}

uint32_t RocketLogWriter::remainingCapacity() const
{
    if (m_writeAddress > m_flashCapacity)
    {
        return 0U;
    }

    const uint32_t remaining =
        m_flashCapacity - m_writeAddress;

    if (m_bufferedLength > remaining)
    {
        return 0U;
    }

    return remaining - m_bufferedLength;
}

Flash::Result RocketLogWriter::lastFlashResult() const
{
    return m_lastFlashResult;
}

}