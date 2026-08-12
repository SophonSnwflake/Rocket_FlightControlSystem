#pragma once

#include "RSL_common.h"
#include "alg_general.hpp"
#include "cmsis_os.h"
#include "stm32f411xe.h"
#include "drv_spi.h"


class Flash{
public:
    typedef enum : uint8_t
    {
        OK,
        INVALID_ARGUMENT,
        OUT_OF_RANGE,
        UNALIGNED,
        NOT_INITIALIZED,
        DEVICE_NOT_FOUND,
        BUSY,
        TIME_OUT,
        COMMUNICATION_ERROR,
        PROGRAM_ERROR,
        ERASE_ERROR,
        VERIFY_FAILED,
        WRITE_PROTECTED
    }Result;

    typedef struct 
    {
        uint32_t capacity;      // 总容量，单位字节
        uint32_t pageSize;      // 页大小
        uint32_t eraseSize;     // 最小擦除单位
        uint8_t erasedValue;    // 擦除后的值，通常为 0xFF
    } Geometry;


public:
    Flash() = default;
    virtual ~Flash() = default;
    virtual Result Init() = 0;
    virtual Geometry geometry() const = 0;
    Result read(uint32_t address, uint8_t* data, uint32_t length);
    Result program(uint32_t address, const uint8_t* data, uint32_t length);
    Result erase(uint32_t address, uint32_t length);

protected:
    virtual Result doRead(uint32_t address, uint8_t* data, uint32_t length) = 0;
    virtual Result doProgram(uint32_t address, const uint8_t* data, uint32_t length) = 0;
    virtual Result doErase(uint32_t address, uint32_t length) = 0;
    bool isRangeValid(uint32_t address, uint32_t length) const;
};

class W25Q128 final: public Flash{
public:
    //芯片参数
    static constexpr uint32_t Capacity   = 16U * 1024U * 1024U;
    static constexpr uint32_t PageSize   = 256U;
    static constexpr uint32_t SectorSize = 4U * 1024U;
    static constexpr uint32_t Block32Size = 32U * 1024U;
    static constexpr uint32_t Block64Size = 64U * 1024U;

    //等待时间
    static constexpr uint32_t PageProgramTimeoutMs = 50U;   
    static constexpr uint32_t ReadyTimeoutMs = 500U;
    static constexpr uint32_t SectorEraseTimeoutMs = 2000U;
    static constexpr uint32_t Block32EraseTimeoutMs = 2000U;
    static constexpr uint32_t Block64EraseTimeoutMs = 4000U;
    static constexpr uint32_t FullChipEraseTimeoutMs = 250000U;

    //命令
    static constexpr uint8_t CMD_READ_STATUS1 = 0x05;
    static constexpr uint8_t CMD_WRITE_ENABLE = 0x06;
    static constexpr uint8_t CMD_READ_JEDEC_ID = 0x9F;
    static constexpr uint8_t CMD_READ_COMMON = 0x03;
    static constexpr uint8_t CMD_WRITE_COMMAND = 0x02;
    static constexpr uint8_t CMD_ERASE_SECTOR = 0x20;
    static constexpr uint8_t CMD_ERASE_BLOCK_32K = 0x52;
    static constexpr uint8_t CMD_ERASE_BLOCK_64K = 0xD8;
    static constexpr uint8_t CMD_ERASE_FULL_CHIP = 0xC7;

    static constexpr uint32_t MaxTransferChunk = 4096U;   // 单次 SPI 调用的字节上限
    static constexpr uint32_t SpiTimeoutMs     = 1000U;   

    typedef struct{
        SPI_HandleTypeDef *spiHandle;
        GPIO_TypeDef *cs_port;
        uint16_t cs_pin;
        uint32_t jedecId;
        bool initialized;
    } W25QXX_HandleTypeDef;

private:
    W25QXX_HandleTypeDef m_handleTypeDef;

public:
    W25Q128(SPI_HandleTypeDef& spi, GPIO_TypeDef* csPort, uint16_t csPin);
    Result Init() override;
    Geometry geometry() const override;
    bool isInitialized() const
    {
        return m_handleTypeDef.initialized;
    }

    uint32_t getjedecId() const
    {
        return m_handleTypeDef.jedecId;
    }

    SPI_HandleTypeDef* getSpiHandle();

protected:
    Result doRead(uint32_t address, uint8_t* data, uint32_t length) override;
    Result doProgram(uint32_t address, const uint8_t* data, uint32_t length) override;
    Result doErase(uint32_t address, uint32_t length) override;

private:
    void select();
    void deselect();
    Result sendAddress(uint32_t address);
    Result writeEnable();
    Result waitReady(uint32_t timeoutMs);
    Result readStatus1(uint8_t& status);
    Result readJedecId(uint32_t& id);
    Result programPage(uint32_t address, const uint8_t* data, uint32_t length);
    Result eraseSector(uint32_t address);
    Result eraseBlock32K(uint32_t address);
    Result eraseBlock64K(uint32_t address);
    Result eraseChip();

private:
    //总线锁
    static constexpr uint32_t BusLockTimeoutMs = 50U;

    /**
     * @brief 总线锁 + 片选的 RAII 守卫
     * @note  构造:拿总线锁 -> 拉低 CS;析构:拉高 CS -> 放总线锁(严格反序)
     * @note  构造后必须检查 ok(),拿不到锁时 CS 不会被拉低
     * @warning 持有本守卫期间不得调用 waitReady()——它内部的 readStatus1
     *          要重新拿锁,普通互斥量不可重入,会自锁死
     */
    class BusGuard
    {
    public:
        BusGuard(W25Q128& owner, uint32_t timeoutMs)
            : m_owner(owner),
              m_locked(SPI_BusLock(owner.m_handleTypeDef.spiHandle, timeoutMs) == HAL_OK)
        {
            if (m_locked) m_owner.select();
        }

        ~BusGuard()
        {
            if (m_locked)
            {
                m_owner.deselect();
                SPI_BusUnlock(m_owner.m_handleTypeDef.spiHandle);
            }
        }

        bool ok() const { return m_locked; }

        BusGuard(const BusGuard&)            = delete;
        BusGuard& operator=(const BusGuard&) = delete;

    private:
        W25Q128& m_owner;
        bool     m_locked;
    };
};