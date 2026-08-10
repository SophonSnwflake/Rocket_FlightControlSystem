#include "app_logger.hpp"

namespace RocketLog  
{             

FlightLogger::FlightLogger(RocketLogWriter *LogWriter) : 
    m_LogWriter(LogWriter),
    m_isStarted(false)

{    
}

FlightLogger::FlightLoggerError FlightLogger::start(uint64_t timestampUs)
{
    if (!m_LogWriter->isPrepared())
    {
        return FlightLoggerError::WriterNotPrepared;
    }
    
    if (m_LogWriter->bytesAccepted() != 0U)
    {
        return FlightLoggerError::InvalidState;
    }

    LOG_TRY(writeFileHeader(timestampUs));
    LOG_TRY(writeFlagBits());
    LOG_TRY(writeAllFormats());
    LOG_TRY(writeAllSubscriptions());

    const RocketLogWriter::FlashLogError writerResult =
        m_LogWriter->flush();

    if (writerResult != RocketLogWriter::FlashLogError::OK)
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

    const RocketLogWriter::FlashLogError result = m_LogWriter->append(reinterpret_cast<uint8_t*>(&header), sizeof(header));
    if (result != RocketLogWriter::FlashLogError::OK){
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFlagBits(){
    ulog_message_flag_bits_s flag{};

    flag.msg_size = static_cast<uint16_t>(sizeof(ulog_message_flag_bits_s)- sizeof(ulog_message_header_s));

    flag.msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

    const RocketLogWriter::FlashLogError result = m_LogWriter->append(reinterpret_cast<uint8_t*>(&flag), sizeof(flag));

    if (result != RocketLogWriter::FlashLogError::OK)
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

    const RocketLogWriter::FlashLogError result =
        m_LogWriter->append(
            reinterpret_cast<uint8_t*>(&message),
            writeLength);

    if (result != RocketLogWriter::FlashLogError::OK)
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
    const RocketLogWriter::FlashLogError result =
    m_LogWriter->append(
        reinterpret_cast<uint8_t*>(&adds),
        writeLength);

    if (result != RocketLogWriter::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeIMU(IMURawMessage *imuMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::ImuRaw, imuMessage, sizeof(*imuMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeGNSS(GNSSMessage *gnssMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::Gnss, gnssMessage, sizeof(*gnssMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeAHRS(AHRSMessage *ahrsMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::Ahrs, ahrsMessage, sizeof(*ahrsMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFlightEstimate(FlightEstimateMessage *flightEstimateMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::FlightEstimate, flightEstimateMessage, sizeof(*flightEstimateMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeFlightState(FlightStateMessage *flightStateMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::FlightState, flightStateMessage, sizeof(*flightStateMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writePower(PowerMessage *powerMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::Power, powerMessage, sizeof(*powerMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeSystemHealth(SystemHealthMessage *systemHealthMessage){
    if(m_isStarted != true) return FlightLoggerError::NotStarted;
    LOG_TRY(writeData(ULogMessageId::SystemHealth, systemHealthMessage, sizeof(*systemHealthMessage)));
    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::writeData(ULogMessageId messageID, void *payload, uint32_t length){
    if ((payload == nullptr) || (length == 0U)) return FlightLoggerError::InvalidArgument;
    if (length > static_cast<uint16_t>(UINT16_MAX - sizeof(uint16_t))) return FlightLoggerError::InvalidArgument;

    ulog_message_data_s dataHeader{};
    dataHeader.msg_type = static_cast<uint8_t>(ULogMessageType::DATA);
    dataHeader.msg_id = static_cast<uint16_t>(messageID);
    dataHeader.msg_size = static_cast<uint16_t>(length + sizeof(dataHeader.msg_id));
    RocketLogWriter::FlashLogError result = m_LogWriter->append(reinterpret_cast<uint8_t*>(&dataHeader), sizeof(dataHeader));

    if (result != RocketLogWriter::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    result = m_LogWriter->append(reinterpret_cast<uint8_t*>(payload), length);
    if (result != RocketLogWriter::FlashLogError::OK)
    {
        m_lastWriterError = result;
        return FlightLoggerError::WriterError;
    }

    return FlightLoggerError::OK;
}

FlightLogger::FlightLoggerError FlightLogger::flush(){
    RocketLogWriter::FlashLogError result = m_LogWriter->flush();
    if (result != RocketLogWriter::FlashLogError::OK)
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