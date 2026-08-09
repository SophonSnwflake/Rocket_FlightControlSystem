#pragma once

#include "RSL_common.h"
#include "mid_logger_writer.hpp"
#include "mid_protocal.hpp"
#include "mid_logger_message.hpp"

#define LOG_TRY(expr)                                      \
    do                                                     \
    {                                                      \
        const auto logTryResult = (expr);                  \
        if (logTryResult != FlightLoggerError::OK)         \
        {                                                  \
            return logTryResult;                           \
        }                                                  \
    } while (false)


namespace RocketLog
{

class FlightLogger final{
public:
    enum class FlightLoggerError : uint8_t
    {
        OK = 0,
        AlreadyStarted,
        NotStarted,
        WriterNotPrepared,
        InvalidArgument,
        InvalidState,
        WriterError
    };

private:
    RocketLogWriter::FlashLogError m_lastWriterError;
    RocketLogWriter *m_LogWriter;

    bool m_isStarted = false;

public:
    FlightLogger(RocketLogWriter *LogWriter);
    ~FlightLogger() = default;
    FlightLoggerError start(uint64_t timestampUs);
    FlightLoggerError writeIMU(IMURawMessage *imuMessage);
    FlightLoggerError writeGNSS(GNSSMessage *gnssMessage);
    FlightLoggerError writeAHRS(AHRSMessage *ahrsMessage);
    FlightLoggerError writeFlightEstimate(FlightEstimateMessage *flightEstimateMessage);
    FlightLoggerError writeFlightState(FlightStateMessage *flightStateMessage);
    FlightLoggerError writePower(PowerMessage *powerMessage);
    FlightLoggerError writeSystemHealth(SystemHealthMessage *systemHealthMessage);

    FlightLoggerError flush();
    FlightLoggerError stop();

private:
    FlightLoggerError writeFileHeader(uint64_t timestampUs);
    FlightLoggerError writeFlagBits();
    FlightLoggerError writeAllFormats();
    FlightLoggerError writeFormat(const char* format, uint16_t formatLength);
    FlightLoggerError writeAllSubscriptions();
    FlightLoggerError writeSingleSubscription(ULogMessageId message_id, const char* messageName, uint16_t nameLength);

    FlightLoggerError writeData(ULogMessageId messageID, void *payload, uint32_t length);


};

}