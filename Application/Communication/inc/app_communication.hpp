#pragma once
#include "FreeRTOS.h"
#include "RSL_common.h"
#include "agr_telemetry_protocal.hpp"
#include "dvc_lora.hpp"
#include "queue.h"

class Communicator final{
public:
    static constexpr uint32_t COMMUNICATOR_QUEUE_LENGTH = 64;
    static constexpr size_t HEADER_SIZE = 8;
    static constexpr size_t FLIGHT_PAYLOAD_SIZE = 19;
    static constexpr size_t GNSS_PAYLOAD_SIZE   = 19;
    static constexpr size_t SYSTEM_PAYLOAD_SIZE = 11;
    enum class CommunicatorError : uint8_t{
        OK = 0,
        DeviceError,
        QueueError,
        QueueFull,
        DidNotInit,
        BadParama,
        RxPacketTooLong
    };

    enum class CommunicatorEventType : uint8_t
    {
        Flight,
        GNSS,
        System
    };

    enum class CommunicationState : uint8_t{
        TX,
        RX,
    };

    struct CommunicatorEvent
    {
        CommunicatorEventType type;
        union
        {
            Telemetry::FlightTelemetryPayload flight;
            Telemetry::GNSSTelemetryPayload gnss;
            Telemetry::SystemTelemetryPayload system;
        } data;
    };

private:
    LoRa *m_lora;
    uint16_t m_sequence = 0;
    static constexpr size_t LORA_RX_BUFFER_SIZE = 255U;
    uint8_t m_rxBuffer[LORA_RX_BUFFER_SIZE];
    size_t m_rxLength = 0U;
    bool m_isReceivedData = false;

    CommunicationState m_communicationState = CommunicationState::RX;
    StaticQueue_t m_communicatorQueueControlBlock;
    uint8_t m_communicatorQueueStorage[COMMUNICATOR_QUEUE_LENGTH * sizeof(CommunicatorEvent)];
    QueueHandle_t m_communicatorQueue;
    uint16_t m_communicatorDroppedCount = 0;

public:
    Communicator(LoRa *lora);
    ~Communicator() = default;
    CommunicatorError CommunicatorLoop();
    CommunicatorError sendFlightTelemetryPayload(const Telemetry::FlightTelemetryPayload *payload);
    CommunicatorError sendGNSSTelemetryPayload(const Telemetry::GNSSTelemetryPayload *payload);
    CommunicatorError sendSystemTelemetryPayload(const Telemetry::SystemTelemetryPayload *payload);
    void setReceiveFlagFalse() {m_isReceivedData = false;}
    uint8_t getRxLength(){return m_rxLength;}

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