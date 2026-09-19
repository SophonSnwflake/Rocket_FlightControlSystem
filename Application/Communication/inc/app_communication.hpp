#pragma once
#include "FreeRTOS.h"
#include "RSL_common.h"
#include "agr_telemetry_protocal.hpp"
#include "dvc_lora.hpp"
#include "queue.h"

class Communicator final{
public:
    static constexpr size_t HEADER_SIZE = 8;
    static constexpr size_t FLIGHT_PAYLOAD_SIZE = 19;
    static constexpr size_t GNSS_PAYLOAD_SIZE   = 19;
    static constexpr size_t SYSTEM_PAYLOAD_SIZE = 11;
    static constexpr size_t RAW_DATA_MAX_LENGTH = 128;
    static constexpr size_t RAW_DATA_FIFO_LENGTH = 10;

    enum class CommunicatorError : uint8_t{
        OK = 0,
        DeviceError,
        QueueError,
        QueueFull,
        DidNotInit,
        BadParama,
        RxPacketTooLong,
        TxPacketTooLong
    };

    enum class CommunicatorEventType : uint8_t
    {
        Flight,
        GNSS,
        System,
        RawData
    };    
    struct RawDataPayload
    {
        uint16_t length;
        uint8_t data[RAW_DATA_MAX_LENGTH];
    };

private:
    LoRa *m_lora;
    uint16_t m_sequence = 0;

    CommunicatorEventType m_txIndex = CommunicatorEventType::Flight;
    StaticQueue_t m_flightQueueControlBlock;
    StaticQueue_t m_GNSSQueueControlBlock;
    StaticQueue_t m_systemQueueControlBlock;
    StaticQueue_t m_rawDataQueueControlBlock;

    uint8_t m_flightQueueStorage[1 * sizeof(Telemetry::FlightTelemetryPayload)];
    uint8_t m_GNSSQueueStorage[1 * sizeof(Telemetry::GNSSTelemetryPayload)];
    uint8_t m_systemQueueStorage[1 * sizeof(Telemetry::SystemTelemetryPayload)];
    uint8_t m_rawDataQueueStorage[RAW_DATA_FIFO_LENGTH * sizeof(Communicator::RawDataPayload)];

    QueueHandle_t m_flightQueue;
    QueueHandle_t m_GNSSQueue;
    QueueHandle_t m_systemQueue;
    QueueHandle_t m_rawDataQueue;
    
    uint16_t m_communicatorDroppedCount = 0;
    bool m_rxRecoveryRequired = false;

    fp32 m_rxTxAirtimeError = 0;
    TickType_t m_lastTime = 0;

public:
    Communicator(LoRa *lora);
    ~Communicator() = default;
    CommunicatorError CommunicatorLoop(uint8_t *rxBuffer, size_t rxCapacity, size_t &rxLength, bool &isReceivedData,  fp32 RXPercentage);
    CommunicatorError sendFlightTelemetryPayload(const Telemetry::FlightTelemetryPayload *payload);
    CommunicatorError sendGNSSTelemetryPayload(const Telemetry::GNSSTelemetryPayload *payload);
    CommunicatorError sendSystemTelemetryPayload(const Telemetry::SystemTelemetryPayload *payload);
    CommunicatorError sendRawData(const uint8_t* data, size_t length);

private:   
    CommunicatorError encodeHeaderTelemetry(const Telemetry::PacketType type, uint8_t *buffer, uint16_t sequence, uint16_t payloadLength);
    CommunicatorError encodeFlightTelemetry(const Telemetry::FlightTelemetryPayload *payload, uint8_t *buffer, size_t bufferLength);
    CommunicatorError encodeGNSSTelemetry(const Telemetry::GNSSTelemetryPayload *payload, uint8_t *buffer, size_t bufferLength);
    CommunicatorError encodeSystemTelemetry(const Telemetry::SystemTelemetryPayload *payload, uint8_t *buffer, size_t bufferLength);
    bool isLoraGetIRQ();
    void writeU16LE(uint8_t *buffer, uint16_t value);
    void writeI16LE(uint8_t *buffer, int16_t value);
    void writeU32LE(uint8_t *buffer, uint32_t value);
    void writeI32LE(uint8_t *buffer, int32_t value);
};