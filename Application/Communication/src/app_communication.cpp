#include "app_communication.hpp"
#include "agr_telemetry_protocal.hpp"


Communicator::Communicator(LoRa *lora) :
    m_lora(lora)
{
    m_flightQueue = xQueueCreateStatic(1, sizeof(Telemetry::FlightTelemetryPayload), m_flightQueueStorage, &m_flightQueueControlBlock);
    m_GNSSQueue = xQueueCreateStatic(1, sizeof(Telemetry::GNSSTelemetryPayload), m_GNSSQueueStorage, &m_GNSSQueueControlBlock);
    m_systemQueue = xQueueCreateStatic(1, sizeof(Telemetry::SystemTelemetryPayload), m_systemQueueStorage, &m_systemQueueControlBlock);
    m_rawDataQueue = xQueueCreateStatic(RAW_DATA_FIFO_LENGTH, sizeof(Communicator::RawDataPayload), m_rawDataQueueStorage, &m_rawDataQueueControlBlock);

    m_lastTime = xTaskGetTickCount();
}

/**
 * @brief 执行一次通信循环，处理 LoRa 接收事件及遥测发送队列。
 *
 * 优先检查 LoRa DIO1 IRQ；若检测到 RX_DONE，则读取收到的数据包，
 * 保存实际包长并标记新数据可用。随后检查发送队列，并根据事件类型
 * 编码 Flight、GNSS 或 System 遥测数据后通过 LoRa 阻塞发送。
 * 每次发送完成后重新进入持续接收模式，以恢复下行命令监听。
 *
 * 本函数本身不包含永久循环，应由上层 Communication Task 周期调用。
 *
 * @return CommunicatorError::OK 本轮处理成功或无事件；
 *         其他返回值表示初始化、队列、编码或 LoRa 设备错误。
 */
Communicator::CommunicatorError Communicator::CommunicatorLoop(uint8_t *rxBuffer, size_t rxCapacity, size_t &rxLength, bool &isReceivedData, fp32 RXPercentage){
    if (m_lora == nullptr)return CommunicatorError::DidNotInit;
    if (!m_lora->isLoRaBegined())return CommunicatorError::DidNotInit;
    if (rxBuffer == nullptr || rxCapacity == 0U) return CommunicatorError::BadParama;
    if (RXPercentage < 0.0f || RXPercentage > 1.0f) return CommunicatorError::BadParama;
    if (m_flightQueue == nullptr || m_GNSSQueue == nullptr || m_systemQueue == nullptr || m_rawDataQueue == nullptr) return CommunicatorError::QueueError;

    rxLength = 0U;
    isReceivedData = false;

    if (m_rxRecoveryRequired) {
        if (m_lora->startReceive(rxCapacity, SX126X_RX_TIMEOUT_INF) != LoRa::LoraError::OK) {
            return CommunicatorError::DeviceError;
        }
        m_rxRecoveryRequired = false;
        m_lastTime = xTaskGetTickCount();
        return CommunicatorError::OK;
    }

    TickType_t now = xTaskGetTickCount();
    fp32 dt = (fp32)(now - m_lastTime) * 1000.0f / configTICK_RATE_HZ;
    m_lastTime = now;
    
    m_rxTxAirtimeError += (1.0f - RXPercentage) * dt;


    // ================= RX =================
    const LoRa::RadioEvent radioEvent = m_lora->getEvent();
    if(radioEvent == LoRa::RadioEvent::RxDone)
    {
        printf("YES I RECEIVED! \r\n");
        const LoRa::LoraError loraResult =  m_lora->readData(rxBuffer,rxCapacity, rxLength);

        if(loraResult == LoRa::LoraError::PacketTooLong) {
            m_rxRecoveryRequired = true;
            return CommunicatorError::RxPacketTooLong;
        }
            
        if(loraResult != LoRa::LoraError::OK) {
            m_rxRecoveryRequired = true;
            return CommunicatorError::DeviceError;
        }
        
        isReceivedData = true;
        return CommunicatorError::OK;
    }else if(radioEvent == LoRa::RadioEvent::Timeout || radioEvent == LoRa::RadioEvent::CrcError || radioEvent == LoRa::RadioEvent::HeaderError){
        m_rxRecoveryRequired = true;
    }
    // ================= TX =================
    else if (m_rxTxAirtimeError >= 0.0f){
        CommunicatorError txResult = CommunicatorError::OK;
        TickType_t txStart = 0;
        TickType_t txEnd = 0;
        isReceivedData = false;

        bool packetFound = false; // 本次是否找到要发送的包     
        bool transmitAttempted = false; // 本次是否尝试发送包(编码之后，发送之前)

        for (uint8_t attempt = 0; attempt < 4; ++attempt){
            switch (m_txIndex){
                case CommunicatorEventType::Flight:{
                    Telemetry::FlightTelemetryPayload flightEvent;
                    if(xQueueReceive(m_flightQueue, &flightEvent, 0) == pdPASS){
                        packetFound = true;
                        const uint16_t sequence = m_sequence;
                        uint8_t buff[HEADER_SIZE + FLIGHT_PAYLOAD_SIZE];

                        txResult = encodeHeaderTelemetry(Telemetry::PacketType::FlightTelemetry, buff, sequence, FLIGHT_PAYLOAD_SIZE);
                        if (txResult  == CommunicatorError::OK) {
                            txResult  = encodeFlightTelemetry(&flightEvent, &buff[HEADER_SIZE], FLIGHT_PAYLOAD_SIZE);
                        }
                       
                        if (txResult  == CommunicatorError::OK) {
                            transmitAttempted = true;
                            m_rxRecoveryRequired = true;
                            txStart = xTaskGetTickCount();
                            LoRa::LoraError loraResult = m_lora->transmit(buff, HEADER_SIZE + FLIGHT_PAYLOAD_SIZE);
                            txEnd = xTaskGetTickCount();
                            if (loraResult == LoRa::LoraError::OK){
                                m_sequence ++;
                            } else{
                                m_communicatorDroppedCount ++;
                                txResult = CommunicatorError::DeviceError;
                            }
                        }else{
                            m_communicatorDroppedCount ++;
                        }
                    }
                    break;
                }
                
                case CommunicatorEventType::GNSS:{
                    Telemetry::GNSSTelemetryPayload gnssEvent;
                    if(xQueueReceive(m_GNSSQueue, &gnssEvent, 0) == pdPASS){
                        packetFound = true;
                        const uint16_t sequence = m_sequence;
                        uint8_t buff[HEADER_SIZE + GNSS_PAYLOAD_SIZE];

                        txResult = encodeHeaderTelemetry(Telemetry::PacketType::GNSSTelemetry, buff, sequence, GNSS_PAYLOAD_SIZE);
                        if (txResult  == CommunicatorError::OK) {
                            txResult  = encodeGNSSTelemetry(&gnssEvent, &buff[HEADER_SIZE], GNSS_PAYLOAD_SIZE);
                        }
                       
                        if (txResult  == CommunicatorError::OK) {
                            transmitAttempted = true;
                            m_rxRecoveryRequired = true;
                            txStart = xTaskGetTickCount();
                            LoRa::LoraError loraResult = m_lora->transmit(buff, HEADER_SIZE + GNSS_PAYLOAD_SIZE);
                            txEnd = xTaskGetTickCount();
                            if (loraResult == LoRa::LoraError::OK){
                                m_sequence ++;
                            } else{
                                m_communicatorDroppedCount ++;
                                txResult = CommunicatorError::DeviceError;
                            }
                        }else{
                            m_communicatorDroppedCount ++;
                        }
                    }
                    break;
                }

                case CommunicatorEventType::System:{
                    Telemetry::SystemTelemetryPayload systemEvent;
                    if (xQueueReceive(m_systemQueue, &systemEvent, 0) == pdPASS){
                        packetFound = true;
                        const uint16_t sequence = m_sequence;
                        uint8_t buff[HEADER_SIZE + SYSTEM_PAYLOAD_SIZE];

                        txResult = encodeHeaderTelemetry(Telemetry::PacketType::SystemTelemetry, buff, sequence, SYSTEM_PAYLOAD_SIZE);
                        if (txResult  == CommunicatorError::OK) {
                            txResult  = encodeSystemTelemetry(&systemEvent, &buff[HEADER_SIZE], SYSTEM_PAYLOAD_SIZE);
                        }
                       
                        if (txResult  == CommunicatorError::OK) {
                            transmitAttempted = true;
                            m_rxRecoveryRequired = true;
                            txStart = xTaskGetTickCount();
                            LoRa::LoraError loraResult = m_lora->transmit(buff, HEADER_SIZE + SYSTEM_PAYLOAD_SIZE);
                            txEnd = xTaskGetTickCount();
                            if (loraResult == LoRa::LoraError::OK){
                                m_sequence ++;
                            } else{
                                m_communicatorDroppedCount ++;
                                txResult = CommunicatorError::DeviceError;
                            }
                        }else{
                            m_communicatorDroppedCount ++;
                        }
                    }
                    break;
                }

                case CommunicatorEventType::RawData:{
                    Communicator::RawDataPayload rawEvent;
                    if (xQueueReceive(m_rawDataQueue, &rawEvent, 0) == pdPASS){
                        packetFound = true;
                        transmitAttempted = true;
                        m_rxRecoveryRequired = true;
                        txStart = xTaskGetTickCount();
                        LoRa::LoraError loraResult = m_lora->transmit(rawEvent.data, rawEvent.length);
                        txEnd = xTaskGetTickCount();
                        if (loraResult != LoRa::LoraError::OK){
                            m_communicatorDroppedCount ++;
                            txResult = CommunicatorError::DeviceError;
                        }
                    }
                    break;
                }
                
            }
            if(m_txIndex == static_cast<CommunicatorEventType>(3)){
                m_txIndex = static_cast<CommunicatorEventType>(0);
            }else{
                m_txIndex = static_cast<CommunicatorEventType>(static_cast<uint8_t>(m_txIndex) + 1);
            }

            if (packetFound) break;
        }

        if (!packetFound) return CommunicatorError::OK;

        if (transmitAttempted){
            const fp32 txTime = (fp32)(txEnd - txStart) * 1000.0f / configTICK_RATE_HZ;
            m_rxTxAirtimeError -= RXPercentage * txTime;
            m_lastTime = txEnd;
            const LoRa::LoraError rxResult = m_lora->startReceive( rxCapacity, SX126X_RX_TIMEOUT_INF);
            if (rxResult == LoRa::LoraError::OK) {
                m_rxRecoveryRequired = false;
            } else {
                return CommunicatorError::DeviceError;
            }
        }
        
        return txResult;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendFlightTelemetryPayload(const Telemetry::FlightTelemetryPayload *payload){
    if (m_flightQueue == nullptr)return CommunicatorError::QueueError;
    if (payload == nullptr)return CommunicatorError::BadParama;

    if (xQueueOverwrite(m_flightQueue, payload) != pdPASS)
    {
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueError;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendGNSSTelemetryPayload(const Telemetry::GNSSTelemetryPayload *payload){
    if (m_GNSSQueue == nullptr)return CommunicatorError::QueueError;
    if (payload == nullptr)return CommunicatorError::BadParama;

    if (xQueueOverwrite(m_GNSSQueue, payload) != pdPASS)
    {
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueError;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendSystemTelemetryPayload(const Telemetry::SystemTelemetryPayload *payload){
    if (m_systemQueue == nullptr)return CommunicatorError::QueueError;
    if (payload == nullptr)return CommunicatorError::BadParama;

    if (xQueueOverwrite(m_systemQueue, payload) != pdPASS)
    {
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueError;
    }
    return CommunicatorError::OK;
}

Communicator::CommunicatorError Communicator::sendRawData(const uint8_t* data, size_t length){
    if (data == nullptr || length == 0U) return CommunicatorError::BadParama;
        
    if (length > RAW_DATA_MAX_LENGTH) return CommunicatorError::TxPacketTooLong;
    
    RawDataPayload event{};

    event.length = static_cast<uint16_t>(length);  

    memcpy(event.data, data, length);

    if (xQueueSend(m_rawDataQueue, &event, 0) != pdPASS){
        ++m_communicatorDroppedCount;
        return CommunicatorError::QueueFull;
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
