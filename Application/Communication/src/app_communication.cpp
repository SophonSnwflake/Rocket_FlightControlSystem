#include "app_communication.hpp"
#include "agr_telemetry_protocal.hpp"



Communicator::Communicator(LoRa *lora) :
    m_lora(lora)
{
    m_communicatorQueue = xQueueCreateStatic(COMMUNICATOR_QUEUE_LENGTH, sizeof(CommunicatorEvent), m_communicatorQueueStorage, &m_communicatorQueueControlBlock);
}

Communicator::CommunicatorError Communicator::CommunicatorLoop(){
    if (m_lora == nullptr)return CommunicatorError::DidNotInit;
    if (!m_lora->isLoRaBegined())return CommunicatorError::DidNotInit;
    if (m_communicatorQueue == nullptr)return CommunicatorError::QueueError;
    CommunicatorEvent event{};

    if (xQueueReceive(m_communicatorQueue, &event, pdMS_TO_TICKS(10)) != pdPASS) return CommunicatorError::OK;

    switch (event.type)
    {
        case CommunicatorEventType::Flight:
        {
            const uint16_t sequence = m_sequence;
            uint8_t buff[HEADER_SIZE + FLIGHT_PAYLOAD_SIZE];
            CommunicatorError result = encodeHeaderTelemetry(Telemetry::PacketType::FlightTelemetry, buff, sequence, FLIGHT_PAYLOAD_SIZE);
            if (result != CommunicatorError::OK) return result;

            result = encodeFlightTelemetry(&event.data.flight, &buff[HEADER_SIZE], FLIGHT_PAYLOAD_SIZE);
            if (result != CommunicatorError::OK)return result;

            LoRa::LoraError loraResult = m_lora->transmit(buff, HEADER_SIZE + FLIGHT_PAYLOAD_SIZE);
            m_sequence ++;
            if (loraResult != LoRa::LoraError::OK)return CommunicatorError::DeviceError;
            break;
        }

        case CommunicatorEventType::GNSS:
        {
            const uint16_t sequence = m_sequence;
            uint8_t buff[HEADER_SIZE + GNSS_PAYLOAD_SIZE];
            CommunicatorError result = encodeHeaderTelemetry(Telemetry::PacketType::GNSSTelemetry, buff, sequence, GNSS_PAYLOAD_SIZE);
            if (result != CommunicatorError::OK) return result;

            result = encodeGNSSTelemetry(&event.data.gnss, &buff[HEADER_SIZE], GNSS_PAYLOAD_SIZE);
            if (result != CommunicatorError::OK)return result;

            LoRa::LoraError loraResult = m_lora->transmit(buff, HEADER_SIZE + GNSS_PAYLOAD_SIZE);
            m_sequence ++;
            if (loraResult != LoRa::LoraError::OK)return CommunicatorError::DeviceError;
            break;
        }

        case CommunicatorEventType::System:
        {
            const uint16_t sequence = m_sequence;
            uint8_t buff[HEADER_SIZE + SYSTEM_PAYLOAD_SIZE];
            CommunicatorError result = encodeHeaderTelemetry(Telemetry::PacketType::GNSSTelemetry, buff, sequence, SYSTEM_PAYLOAD_SIZE);
            if (result != CommunicatorError::OK) return result;

            result = encodeSystemTelemetry(&event.data.system, &buff[HEADER_SIZE], SYSTEM_PAYLOAD_SIZE);
            if (result != CommunicatorError::OK)return result;

            LoRa::LoraError loraResult = m_lora->transmit(buff, HEADER_SIZE + SYSTEM_PAYLOAD_SIZE);
            m_sequence ++;
            if (loraResult != LoRa::LoraError::OK)return CommunicatorError::DeviceError;
            break;
        }

        default:
            break;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendFlightTelemetryPayload(const Telemetry::FlightTelemetryPayload *payload){
    if (m_communicatorQueue == nullptr)return CommunicatorError::QueueError;
    if (payload == nullptr)return CommunicatorError::BadParama;
    CommunicatorEvent event{};
    event.type = CommunicatorEventType::Flight;
    event.data.flight = *payload;

    if (xQueueSend(m_communicatorQueue, &event, 0) != pdPASS)
    {
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueError;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendGNSSTelemetryPayload(const Telemetry::GNSSTelemetryPayload *payload){
    if (m_communicatorQueue == nullptr)return CommunicatorError::QueueError;
    if (payload == nullptr)return CommunicatorError::BadParama;
    CommunicatorEvent event{};
    event.type = CommunicatorEventType::GNSS;
    event.data.gnss = *payload;

    if (xQueueSend(m_communicatorQueue, &event, 0) != pdPASS)
    {
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueError;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendSystemTelemetryPayload(const Telemetry::SystemTelemetryPayload *payload){
    if (m_communicatorQueue == nullptr)return CommunicatorError::QueueError;
    if (payload == nullptr)return CommunicatorError::BadParama;
    CommunicatorEvent event{};
    event.type = CommunicatorEventType::System;
    event.data.system = *payload;

    if (xQueueSend(m_communicatorQueue, &event, 0) != pdPASS)
    {
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueError;
    }
    return CommunicatorError::OK;
}



Communicator::CommunicatorError Communicator::encodeHeaderTelemetry(const Telemetry::PacketType type, uint8_t *buffer, uint16_t sequence, uint16_t payloadLength){
    if (buffer == nullptr)return CommunicatorError::BadParama;
    writeU16LE(&buffer[0], Telemetry::PACKET_MAGIC);
    buffer[2] = Telemetry::PROTOCOL_VERSION;
    buffer[3] = static_cast<uint8_t>(type);
    writeU16LE(&buffer[4], sequence);
    writeU16LE(&buffer[6], payloadLength);
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::encodeFlightTelemetry(const Telemetry::FlightTelemetryPayload *payload, uint8_t *buffer, size_t bufferLength){
    if (payload == nullptr || buffer == nullptr)return CommunicatorError::BadParama;
    if(bufferLength < FLIGHT_PAYLOAD_SIZE) return CommunicatorError::BadParama;
    writeU32LE(&buffer[0],payload->timeStamp_ms);
    writeI16LE(&buffer[4],payload->roll_centidegree);
    writeI16LE(&buffer[6],payload->yaw_centidegree);
    writeI16LE(&buffer[8],payload->pitch_centidegree);

    writeI32LE(&buffer[10], payload->relative_altitude_mm);
    
    writeI32LE(&buffer[14], payload->vertical_velocity_mm_s);

    buffer[18] = static_cast<uint8_t>(payload->flight_phase);
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::encodeGNSSTelemetry(const Telemetry::GNSSTelemetryPayload *payload, uint8_t *buffer, size_t bufferLength){
    if (payload == nullptr || buffer == nullptr)return CommunicatorError::BadParama;
    if(bufferLength < GNSS_PAYLOAD_SIZE) return CommunicatorError::BadParama;
    writeU32LE(&buffer[0],payload->timestamp_ms);
    writeI32LE(&buffer[4],payload->latitude_deg_e7);
    writeI32LE(&buffer[8],payload->longitude_deg_e7);
    writeI32LE(&buffer[12],payload->altitude_msl_mm);
    buffer[16] = payload->fix_type;
    buffer[17] = payload->num_satellites;
    buffer[18] = payload->valid_flags;
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::encodeSystemTelemetry(const Telemetry::SystemTelemetryPayload *payload, uint8_t *buffer, size_t bufferLength){
    if (payload == nullptr || buffer == nullptr)return CommunicatorError::BadParama;
    if(bufferLength < SYSTEM_PAYLOAD_SIZE) return CommunicatorError::BadParama;
    writeU32LE(&buffer[0],payload->timestamp_ms);
    writeU16LE(&buffer[4],payload->battery_mv);
    writeU32LE(&buffer[6],payload->log_dropped_count);
    buffer[10] = payload->system_status;
    return CommunicatorError::OK;
}

// 按照小段序将uint16转为uint8
void Communicator::writeU16LE(uint8_t *buffer, uint16_t value){
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
}

// 按照小段序将int16转为uint8
void Communicator::writeI16LE(uint8_t *buffer, int16_t value){
    uint16_t raw = static_cast<uint16_t>(value);
    buffer[0] = static_cast<uint8_t>(raw);
    buffer[1] = static_cast<uint8_t>(raw >> 8U);
}

// 按照小段序将uint32转为uint8
void Communicator::writeU32LE(uint8_t *buffer, uint32_t value){
    buffer[0] = static_cast<uint8_t>(value);
    buffer[1] = static_cast<uint8_t>(value >> 8U);
    buffer[2] = static_cast<uint8_t>(value >> 16U);
    buffer[3] = static_cast<uint8_t>(value >> 24U);
}

// 按照小段序将int32转为uint8
void Communicator::writeI32LE(uint8_t *buffer, int32_t value){
    uint32_t raw = static_cast<uint32_t>(value);
    buffer[0] = static_cast<uint8_t>(raw);
    buffer[1] = static_cast<uint8_t>(raw >> 8U);
    buffer[2] = static_cast<uint8_t>(raw >> 16U);
    buffer[3] = static_cast<uint8_t>(raw >> 24U);
}
