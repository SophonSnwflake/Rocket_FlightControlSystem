#include "app_logger.hpp"

namespace RocketLog  
{             

FlightLogger::FlightLogger(RocketLogger *LogWriter) : 
    m_LogWriter(LogWriter),
    m_isStarted(false)

{    
}

FlightLogger::FlightLoggerError FlightLogger::start(uint64_t timestampUs)
{
    // if (!m_LogWriter->isPrepared())
    // {
    //     return FlightLoggerError::WriterNotPrepared;
    // }
    
    // if (m_LogWriter->bytesAccepted() != 0U)
    // {
    //     return FlightLoggerError::InvalidState;
    // }

    LOG_TRY(writeFileHeader(timestampUs));
    LOG_TRY(writeFlagBits());
    LOG_TRY(writeAllFormats());
    LOG_TRY(writeAllSubscriptions());

    const RocketLogger::FlashLogError writerResult =
        m_LogWriter->flush();

    if (writerResult != RocketLogger::FlashLogError::OK)
    {
        m_lastWriterError = writerResult;
        return FlightLoggerError::WriterError;
    }

    m_isStarted = true;

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFileHeader(uint64_t timestampUs){
    ulog_file_header_s header{};
    memcpy(header.magic, ULOG_MAGIC, sizeof(header.magic));
    header.timestamp = timestampUs;

    const RocketLogger::FlashLogError result = m_LogWriter->append(reinterpret_cast<uint8_t*>(&header), sizeof(header));
    if (result != RocketLogger::FlashLogError::OK){
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFlagBits(){
    ulog_message_flag_bits_s flag{};

    flag.msg_size = static_cast<uint16_t>(sizeof(ulog_message_flag_bits_s)- sizeof(ulog_message_header_s));

    flag.msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

    const RocketLogger::FlashLogError result = m_LogWriter->append(reinterpret_cast<uint8_t*>(&flag), sizeof(flag));

    if (result != RocketLogger::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeAllFormats(){
    // IMU
    LOG_TRY(writeFormat(
        IMU_RAW_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(IMU_RAW_MESSAGE_FORMAT) - 1U)));


    // GNSS
    LOG_TRY(writeFormat(
        GNSS_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(GNSS_MESSAGE_FORMAT) - 1U)));

    // AHRS
    LOG_TRY(writeFormat(
        AHRS_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(AHRS_MESSAGE_FORMAT) - 1U)));


    // FLIGHT_ESTIMATE_MESSAGE
    LOG_TRY(writeFormat(
        FLIGHT_ESTIMATE_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(FLIGHT_ESTIMATE_MESSAGE_FORMAT) - 1U)));

    // FLIGHT_STATE
    LOG_TRY(writeFormat(
        FLIGHT_STATE_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(FLIGHT_STATE_MESSAGE_FORMAT) - 1U)));


    // POWER
    LOG_TRY(writeFormat(
        POWER_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(POWER_MESSAGE_FORMAT) - 1U)));


    // SYSTEM_HEALTH
    LOG_TRY(writeFormat(
        SYSTEM_HEALTH_MESSAGE_FORMAT,
        static_cast<uint16_t>(
            sizeof(SYSTEM_HEALTH_MESSAGE_FORMAT) - 1U)));



    return FlightLoggerError::OK;

}

FlightLogger::FlightLoggerError FlightLogger::writeFormat(const char* format, uint16_t formatLength)
{
    if (format == nullptr || formatLength == 0U)
    {
        return FlightLoggerError::InvalidArgument;
    }

    ulog_message_format_s message{};

    if (formatLength > sizeof(message.format))
    {
        return FlightLoggerError::InvalidArgument;
    }

    message.msg_size = formatLength;
    message.msg_type =
        static_cast<uint8_t>(ULogMessageType::FORMAT);

    memcpy(message.format, format, formatLength);

    const uint32_t writeLength =
        static_cast<uint32_t>(
            offsetof(ulog_message_format_s, format))
        + formatLength;

    const RocketLogger::FlashLogError result =
        m_LogWriter->append(
            reinterpret_cast<uint8_t*>(&message),
            writeLength);

    if (result != RocketLogger::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeAllSubscriptions(){
    LOG_TRY(writeSingleSubscription(ULogMessageId::ImuRaw, IMU_RAW_MESSAGE_NAME, sizeof(IMU_RAW_MESSAGE_NAME)));
    LOG_TRY(writeSingleSubscription(ULogMessageId::Gnss, GNSS_MESSAGE_NAME, sizeof(GNSS_MESSAGE_NAME)));
    LOG_TRY(writeSingleSubscription(ULogMessageId::Ahrs, AHRS_MESSAGE_NAME, sizeof(AHRS_MESSAGE_NAME)));
    LOG_TRY(writeSingleSubscription(ULogMessageId::FlightEstimate, FLIGHT_ESTIMATE_MESSAGE_NAME, sizeof(FLIGHT_ESTIMATE_MESSAGE_NAME)));
    LOG_TRY(writeSingleSubscription(ULogMessageId::FlightState, FLIGHT_STATE_MESSAGE_NAME, sizeof(FLIGHT_STATE_MESSAGE_NAME)));
    LOG_TRY(writeSingleSubscription(ULogMessageId::Power, POWER_MESSAGE_NAME, sizeof(POWER_MESSAGE_NAME)));
    LOG_TRY(writeSingleSubscription(ULogMessageId::SystemHealth, SYSTEM_HEALTH_MESSAGE_NAME, sizeof(SYSTEM_HEALTH_MESSAGE_NAME)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeSingleSubscription(ULogMessageId message_id, const char* messageName, uint16_t nameLength){
    if (messageName == nullptr || nameLength <= 1U)
    {
        return FlightLoggerError::InvalidArgument;
    }
    ulog_message_add_logged_s adds{};
    const uint16_t actualNameLength = nameLength - 1U;
    if (actualNameLength > sizeof(adds.message_name))
    {
        return FlightLoggerError::InvalidArgument;
    }
    adds.msg_id = static_cast<uint16_t>(message_id);
    adds.msg_type = static_cast<uint8_t>(ULogMessageType::ADD_LOGGED_MSG);
    memcpy(adds.message_name, messageName, nameLength - 1);
    adds.multi_id = 0U;
    adds.msg_size = nameLength + 1U + 2U - 1U;

    const uint32_t writeLength = static_cast<uint32_t>(offsetof(ulog_message_add_logged_s, message_name))+ nameLength - 1;
    const RocketLogger::FlashLogError result =
    m_LogWriter->append(
        reinterpret_cast<uint8_t*>(&adds),
        writeLength);

    if (result != RocketLogger::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeIMU(IMURawMessage *imuMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::ImuRaw, imuMessage, IMU_RAW_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeGNSS(GNSSMessage *gnssMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::Gnss, gnssMessage, GNSS_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeAHRS(AHRSMessage *ahrsMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::Ahrs, ahrsMessage, AHRS_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFlightEstimate(FlightEstimateMessage *flightEstimateMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::FlightEstimate, flightEstimateMessage, FLIGHT_ESTIMATE_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFlightState(FlightStateMessage *flightStateMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::FlightState, flightStateMessage, FLIGHT_STATE_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writePower(PowerMessage *powerMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::Power, powerMessage, POWER_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeSystemHealth(SystemHealthMessage *systemHealthMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::SystemHealth, systemHealthMessage, SYSTEM_HEALTH_MESSAGE_PAYLOAD_SIZE));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeData(ULogMessageId messageID, void *payload, uint32_t length){
    if ((payload == nullptr) || (length == 0U)) return FlightLoggerError::InvalidArgument;
    constexpr uint32_t MAX_PAYLOAD_SIZE = GNSS_MESSAGE_PAYLOAD_SIZE;
    static_assert(IMU_RAW_MESSAGE_PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE &&
                  AHRS_MESSAGE_PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE &&
                  FLIGHT_ESTIMATE_MESSAGE_PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE &&
                  FLIGHT_STATE_MESSAGE_PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE &&
                  POWER_MESSAGE_PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE &&
                  SYSTEM_HEALTH_MESSAGE_PAYLOAD_SIZE <= MAX_PAYLOAD_SIZE);
    if (length > MAX_PAYLOAD_SIZE) return FlightLoggerError::InvalidArgument;

    ulog_message_data_s dataHeader{};
    dataHeader.msg_type = static_cast<uint8_t>(ULogMessageType::DATA);
    dataHeader.msg_id = static_cast<uint16_t>(messageID);
    dataHeader.msg_size = static_cast<uint16_t>(length + sizeof(dataHeader.msg_id));
    uint8_t dataBuffer[sizeof(dataHeader) + MAX_PAYLOAD_SIZE];
    memcpy(dataBuffer, &dataHeader, sizeof(dataHeader));
    memcpy(dataBuffer + sizeof(dataHeader), payload, length);

    const RocketLogger::FlashLogError result =
        m_LogWriter->append(dataBuffer, sizeof(dataHeader) + length);
    if (result != RocketLogger::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeSync()
{
    if (!m_isStarted) {
        return FlightLoggerError::NotStarted;
    }

    static constexpr uint8_t SYNC_MAGIC[8] = {
        0x2F, 0x73, 0x13, 0x20,
        0x25, 0x0C, 0xBB, 0x12
    };

    ulog_message_sync_s sync{};
    sync.msg_size = sizeof(sync.sync_magic);
    memcpy(sync.sync_magic, SYNC_MAGIC, sizeof(SYNC_MAGIC));

    const RocketLogger::FlashLogError result =
        m_LogWriter->append(
            reinterpret_cast<uint8_t*>(&sync),
            sizeof(sync));

    if (result != RocketLogger::FlashLogError::OK) {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::flush(){
    RocketLogger::FlashLogError result = m_LogWriter->flush();
    if (result != RocketLogger::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::stop(){
    return flush();
}

}
