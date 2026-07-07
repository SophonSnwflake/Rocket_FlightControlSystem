#pragma once

#include <cstdint>

class GNSS
{
protected:
    bool m_isConnected = false;
    bool m_isDecodeCompleted = false;

public:
    GNSS();
};

class NEOM9N : public GNSS
{
public:
    NEOM9N();
    void receiveRxDataFromISR(const uint8_t *RxData);
};
