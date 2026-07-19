/**
 * @file    dvc_flash.cpp
 * @brief   SPI NOR Flash 设备驱动：Flash 抽象基类 + W25Q128 具体实现
 *
 * 采用 NVI（Non-Virtual Interface）模式：
 *   公开的 read/program/erase 是非虚函数，统一做参数校验和范围检查；
 *   校验通过后转交给受保护的虚函数 doRead/doProgram/doErase 由具体芯片实现。
 *   换其他型号的 flash 只需继承 Flash 并实现三个 doXxx，上层校验逻辑不用重写。
 *
 * 并发模型：
 *   同一条 SPI 总线上挂了多个器件，靠 drv_spi 的总线互斥锁仲裁。
 *   本文件通过 BusGuard（RAII 守卫）持锁，构造时拿锁+拉低 CS，析构时拉高 CS+放锁。
 *
 * @warning 全文贯穿一条铁律：**持有 BusGuard 期间绝不能调用 waitReady()**。
 *          waitReady 内部每毫秒调一次 readStatus1，那里要重新拿总线锁，
 *          而普通互斥量不可重入 —— 会阻塞到超时（BusLockTimeoutMs），
 *          且这个错误不会有任何编译期提示。
 */

#include "dvc_flash.hpp"


/**
 * @brief 检查 [address, address + length) 是否落在芯片容量范围内
 * @param address 起始地址
 * @param length 字节数
 * @retval true 范围合法
 * @retval false 越界
 * @note  用减法而不是 address + length > capacity，避免加法回绕。
 *        若写成加法，address = 0xFFFFFF00、length = 0x200 时和值绕回小数，
 *        检查会被绕过。
 * @note  容量通过虚函数 geometry() 取得，因此基类不需要知道具体芯片型号。
 */
bool Flash::isRangeValid(uint32_t address,
                         uint32_t length) const
{
    const Geometry flashGeometry = geometry();
    const uint32_t capacity = flashGeometry.capacity;
    if (address >= capacity)return false;
    const uint32_t remainingSpace = capacity - address;
    if (length > remainingSpace)return false;
    return true;
}

/**
 * @brief 读取 flash 数据（公开接口）
 * @param address 起始地址
 * @param data 接收缓冲区，容量不得小于 length
 * @param length 字节数，0 视为成功
 * @retval Result::OK 成功
 * @retval Result::INVALID_ARGUMENT data 为空
 * @retval Result::OUT_OF_RANGE 超出芯片容量
 * @retval 其余错误码由 doRead 产生
 * @note  NVI 模式的公开入口：只做校验，实际收发交给 doRead。
 * @note  length 为 0 返回 OK 而非错误——空操作视为成功，调用方不必特判。
 */
Flash::Result Flash::read(uint32_t address, uint8_t* data, uint32_t length){
    if(length == 0) return Result::OK;
    if (data == nullptr) return Result::INVALID_ARGUMENT;
    if (!isRangeValid(address, length)) return Result::OUT_OF_RANGE;
    return doRead(address, data, length);
}

/**
 * @brief 写入 flash 数据（公开接口）
 * @param address 起始地址，无需页对齐
 * @param data 待写数据
 * @param length 字节数，0 视为成功
 * @retval Result::OK 成功
 * @retval Result::INVALID_ARGUMENT data 为空
 * @retval Result::OUT_OF_RANGE 超出芯片容量
 * @retval 其余错误码由 doProgram 产生
 * @warning NOR flash 只能把 1 写成 0，不能把 0 写回 1。目标区域必须先擦除，
 *          否则写入结果是原值与新值按位相与。本函数不做擦除检查。
 */
Flash::Result Flash::program(uint32_t address, const uint8_t* data, uint32_t length){
    if(length == 0) return Result::OK;
    if (data == nullptr) return Result::INVALID_ARGUMENT;
    if (!isRangeValid(address, length)) return Result::OUT_OF_RANGE;
    return doProgram(address, data, length);
}

/**
 * @brief 擦除 flash（公开接口）
 * @param address 起始地址，必须按最小擦除单位对齐
 * @param length 字节数，必须是最小擦除单位的整数倍
 * @retval Result::OK 成功
 * @retval Result::OUT_OF_RANGE 超出芯片容量
 * @retval Result::UNALIGNED 地址或长度未对齐
 * @retval 其余错误码由 doErase 产生
 * @note  擦除单位由 geometry().eraseSize 给出（W25Q128 是 4KB 扇区）。
 *        NOR flash 的擦除粒度由硬件决定，无法擦除任意长度。
 * @note  局部变量 g 是必须的：早期版本这里用的是成员 m_geometry，
 *        多任务同时调用会互相覆盖。
 * @warning 与 doErase 中的同类检查返回了不同的错误码（那边是
 *          INVALID_ARGUMENT），上层无法用同一个分支处理对齐失败。
 */
Flash::Result Flash::erase(uint32_t address, uint32_t length){
    if(length == 0) return Result::OK;
    if (!isRangeValid(address, length)) return Result::OUT_OF_RANGE;
    const Geometry g = geometry();
    if ((address % g.eraseSize)!=0 || (length % g.eraseSize)!=0){
        return Result::UNALIGNED;
    }

    return doErase(address, length);
}

/**
 * @brief W25Q128 构造函数
 * @param spi SPI 句柄引用（用引用而非指针，保证不会传空）
 * @param csPort 片选引脚所在的 GPIO 端口
 * @param csPin 片选引脚
 * @note  只保存句柄和引脚，不进行任何硬件操作。
 *        这是硬性要求：本对象若定义为全局/静态对象，构造函数会在 main() 之前
 *        由启动代码的 __libc_init_array 调用，那时 MX_SPIx_Init() 尚未执行，
 *        外设时钟和寄存器都没配置好。硬件初始化一律放在 Init()。
 * @note  initialized 和 jedecId 未在初始化列表中赋值，依赖 Init() 开头的清零。
 */
W25Q128::W25Q128(SPI_HandleTypeDef& spi,
                 GPIO_TypeDef* csPort,
                 uint16_t csPin)
    : m_handleTypeDef{&spi, csPort, csPin}
{
}

/**
 * @brief 初始化并识别芯片
 * @retval Result::OK 芯片存在、型号正确、未被写保护
 * @retval Result::INVALID_ARGUMENT 句柄或 GPIO 端口为空
 * @retval Result::DEVICE_NOT_FOUND JEDEC ID 不匹配
 * @retval Result::WRITE_PROTECTED 状态寄存器的块保护位被置位
 * @retval Result::TIME_OUT 芯片一直报忙
 *
 * @note  调用时机：必须在 MX_SPIx_Init() 和 SPI_BusInit() 之后。
 *        通常在 osKernelStart() 之前，此时 SPI_BusLock 直接放行、
 *        waitReady 走 HAL_Delay 分支，代码已处理这种情况。
 * @note  可重复调用：开头会先清除 initialized 和 jedecId。
 * @note  空指针检查在 deselect() 之前——顺序不能颠倒，deselect 会解引用 cs_port。
 * @note  memoryType 不做严格校验，兼容片的该字节取值可能与 0x40 不同。
 *
 * @warning 块保护位（BP0-2/TB/SEC，掩码 0x7C）是**非易失**的，掉电保持。
 *          带保护时芯片会静默丢弃所有 program/erase 命令，驱动却一路返回 OK，
 *          表现为"写了一整趟，读回来全是 0xFF"。这个检查是必须的。
 *          清除保护需要发 Write Status Register(0x01) 写 0，本驱动未实现。
 * @warning 芯片不在位时 MISO 悬空被上拉，状态寄存器读回 0xFF，BUSY 位恒为 1，
 *          此时返回的是 TIME_OUT 而不是 DEVICE_NOT_FOUND，会把排查方向
 *          引到时序问题上，实际是没焊上或片选没接。
 */
W25Q128::Result W25Q128::Init()
{
    // 每次重新初始化时，先清除原来的状态
    m_handleTypeDef.initialized = false;
    m_handleTypeDef.jedecId = 0U;

    

    // 根据你的 m_handleTypeDef 实际定义决定是否保留
    if (m_handleTypeDef.spiHandle == nullptr ||
        m_handleTypeDef.cs_port == nullptr)
    {
        return Result::INVALID_ARGUMENT;
    }
    
    // 确保 Flash 没有被片选
    deselect();
    // 检查状态寄存器通信，并等待芯片空闲
    Result result = waitReady(ReadyTimeoutMs);

    if (result != Result::OK)
    {
        return result;
    }

    // 读取 JEDEC ID
    uint32_t id = 0U;

    result = readJedecId(id);

    if (result != Result::OK)
    {
        return result;
    }

    // 拆分 JEDEC ID 的三个字节
    const uint8_t manufacturerId =
        static_cast<uint8_t>((id >> 16) & 0xFFU);

    const uint8_t memoryType =
        static_cast<uint8_t>((id >> 8) & 0xFFU);

    const uint8_t capacityId =
        static_cast<uint8_t>(id & 0xFFU);

    // Winbond 的厂商 ID
    static constexpr uint8_t ExpectedManufacturerId = 0xEFU;

    // W25Q128 的容量代码：128 Mbit
    static constexpr uint8_t ExpectedCapacityId = 0x18U;

    // 检查厂商和容量
    if (manufacturerId != ExpectedManufacturerId ||
        capacityId != ExpectedCapacityId)
    {
        return Result::DEVICE_NOT_FOUND;
    }

    // memoryType 暂时不严格检查。
    // 常见 W25Q128 可能读取到 EF 40 18 或其他兼容类型值。
    (void)memoryType;

    // 保存真实读取到的 JEDEC ID
    m_handleTypeDef.jedecId = id;
    uint8_t status = 0U;
    Result readStatusResult = readStatus1(status);
    if (readStatusResult != Result::OK) return readStatusResult;
    if ((status & 0x7CU) != 0U) { return Result::WRITE_PROTECTED; }
    // 所有检查通过
    m_handleTypeDef.initialized = true;

    return Result::OK;
}

/**
 * @brief 返回芯片几何参数
 * @retval Geometry 容量 16MB、页 256B、扇区 4KB、擦除后为 0xFF
 * @note  基类的 isRangeValid 和 erase 通过这个虚函数取容量和擦除粒度，
 *        因此基类代码完全不需要知道具体型号。
 * @note  每次调用都现场构造返回值，不缓存。全是编译期常量，开销为零。
 */
W25Q128::Geometry W25Q128::geometry() const{
    return {Capacity, PageSize, SectorSize, 0xFF};
}

/**
 * @brief 读取数据（doXxx 实现）
 * @param address 起始地址
 * @param data 接收缓冲区
 * @param length 字节数
 * @retval Result::NOT_INITIALIZED 未调用 Init() 或 Init 失败
 * @retval Result::BUSY 总线锁超时
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 *
 * @note  waitReady 必须在 BusGuard 之前——它内部要反复拿放总线锁。
 * @note  读命令 0x03 之后，**CS 必须全程保持拉低**，因此 guard 活到函数结尾，
 *        不能像 program/erase 那样用花括号提前释放。
 * @note  分块的理由：drv_spi 的 length 形参是 uint16_t，直接传 uint32_t
 *        会被静默截断。最凶的情况是 length = 65537 截成 1 —— 只读回 1 字节，
 *        函数却返回 OK。分块把每次调用压在 MaxTransferChunk(4096) 以内，
 *        static_cast 因此是安全的。
 * @note  分块不需要重发命令和地址：CS 拉低期间芯片内部地址自动递增，
 *        两次时钟突发之间空多久都不影响，SPI 是同步总线，从机不会超时。
 *
 * @warning 读命令 0x03 的时钟上限是 50MHz（手册值）。F411 的 SPI1 挂在
 *          APB2 上，100MHz 二分频正好 50MHz，卡在边界。若预分频设为 /2，
 *          建议改用 Fast Read(0x0B)，代价是地址后要多发一个 dummy 字节。
 * @warning 整个函数期间独占总线。读 4096 字节在 5MHz 下约 6.5ms，
 *          这段时间同总线的其他器件全部阻塞。回读大段日志时应由**调用方**
 *          切成多次 read()，让事务之间有释放总线的机会。
 */
W25Q128::Result W25Q128::doRead(uint32_t address, uint8_t* data, uint32_t length)
{
    if (!isInitialized()) return Result::NOT_INITIALIZED;

    Result result = waitReady(ReadyTimeoutMs);
    if (result != Result::OK) return result;

    const uint8_t command = CMD_READ_COMMON;

    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;

    if (SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1U, SpiTimeoutMs) != HAL_OK)
    {
        return Result::COMMUNICATION_ERROR;
    }

    result = sendAddress(address);
    if (result != Result::OK)
    {
        return result;
    }

    // CS 保持拉低，分块读。芯片内部地址自增，不需要重发 cmd + address。
    uint32_t remaining = length;
    uint8_t* cursor    = data;

    while (remaining > 0U)
    {
        const uint32_t chunk =
            (remaining < MaxTransferChunk) ? remaining : MaxTransferChunk;

        if (SPI_Receive(m_handleTypeDef.spiHandle, cursor,
                        static_cast<uint16_t>(chunk), SpiTimeoutMs) != HAL_OK)
        {
            return Result::COMMUNICATION_ERROR;
        }

        cursor    += chunk;
        remaining -= chunk;
    }
    return Result::OK;
}

/**
 * @brief 写入数据（doXxx 实现），按页切分
 * @param address 起始地址，无需页对齐
 * @param data 待写数据
 * @param length 字节数
 * @retval Result::NOT_INITIALIZED 未初始化
 * @retval Result::OUT_OF_RANGE 越界
 * @retval 其余错误码由 programPage 产生，出错立即中止，已写部分不回滚
 *
 * @note  **本函数的核心是那个切分循环，不是可选优化。**
 *        NOR flash 的页编程命令不会跨越 256 字节页边界：写到页尾之后
 *        地址会回卷到**同一页的页首**，把刚写的数据覆盖掉，而芯片不报任何错。
 *        典型现象是写 300 字节，读回来前 44 字节被后 44 字节覆盖了。
 * @note  首次循环 currentLength 可能小于 256（address 不在页边界上），
 *        之后每次都是整页，直到最后一段。
 * @note  切分顺带保证了传给 SPI 的长度恒 ≤ 256，因此写路径不存在
 *        doRead 那里的 uint16_t 截断问题。
 * @note  每页是一次独立事务（拿锁→发送→放锁→等待），页与页之间总线是自由的，
 *        同总线的其他器件可以插空使用。
 *
 * @warning 与基类 program() 的范围检查重复。基类已经用 isRangeValid 查过一遍。
 */
W25Q128::Result W25Q128::doProgram(uint32_t address, const uint8_t* data, uint32_t length){
    if (!isInitialized())return Result::NOT_INITIALIZED;
    if (address >= Capacity) return Result::OUT_OF_RANGE;
    if (length > Capacity - address)return Result::OUT_OF_RANGE;
    while(length >0){
        uint32_t pageOffset = address % PageSize;
        uint32_t remainSpace = PageSize - pageOffset;
        uint32_t currentLength = (length < remainSpace) ? length : remainSpace;

        Result result = programPage(address,data, currentLength);
        if(result != Result::OK) return result;       
        address += currentLength;
        data += currentLength; 
        length -= currentLength;
    }
    return Result::OK;
}

/**
 * @brief 擦除（doXxx 实现），贪心选用最大可用的擦除单位
 * @param address 起始地址，必须 4KB 对齐
 * @param length 字节数，必须是 4KB 的整数倍
 * @retval Result::NOT_INITIALIZED 未初始化
 * @retval Result::OUT_OF_RANGE 越界
 * @retval Result::INVALID_ARGUMENT 未对齐
 * @retval 其余错误码由各擦除函数产生
 *
 * @note  贪心策略：地址对齐到 64KB 且剩余量够就用 64K 块擦除，
 *        其次 32K，最后退化到 4KB 扇区。擦 1MB 用扇区要发 256 条命令、
 *        累计等待十几秒；用 64K 块只要 16 条，快一个数量级。
 * @note  全片擦除单独走 eraseChip()（命令 0xC7 不带地址）。
 *
 * @warning 对齐检查返回 INVALID_ARGUMENT，而基类 erase() 同样条件返回
 *          UNALIGNED，两处不一致。
 * @warning 结尾 `if(length < SectorSize && length != 0)` 是死代码：
 *          前面已保证 length 是 SectorSize 的整数倍，循环退出时必为 0。
 * @warning 中途失败直接返回，已擦除的部分不会恢复，芯片处于半擦除状态。
 */
W25Q128::Result W25Q128::doErase(uint32_t address, uint32_t length){
    if (!isInitialized())return Result::NOT_INITIALIZED;
    if (length == 0U)return Result::OK;
    if (address >= Capacity) return Result::OUT_OF_RANGE;
    if (length > Capacity - address)return Result::OUT_OF_RANGE;
    if ((address % SectorSize) != 0U) return Result::INVALID_ARGUMENT;
    if ((length % SectorSize) != 0U) return Result::INVALID_ARGUMENT;
    if(address == 0 && length == Capacity) return eraseChip();
    while(length >= SectorSize){
        if(address % Block64Size  == 0 && length >= Block64Size){
            Result result;
            result = eraseBlock64K(address);
            if (result != Result::OK)return result;
            address += Block64Size;
            length -= Block64Size;
        }
        else if(address % Block32Size == 0 && length >= Block32Size){
            Result result;
            result = eraseBlock32K(address);
            if (result != Result::OK)return result;
            address += Block32Size;
            length -= Block32Size;
        }
        else{
            Result result;
            result = eraseSector(address);
            if (result != Result::OK)return result;
            address += SectorSize;
            length -= SectorSize;
        }
    }
    if(length < SectorSize && length != 0){
        return Result::ERASE_ERROR;
    }
    return Result::OK;
}

/**
 * @brief 拉低片选，选中芯片
 * @note  W25Q128 的 CS 低有效。由 BusGuard 的构造函数调用，
 *        普通代码不应直接调用——绕过 guard 意味着没拿总线锁就动了 CS。
 */
void W25Q128::select(){
    HAL_GPIO_WritePin(m_handleTypeDef.cs_port, m_handleTypeDef.cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief 拉高片选，释放芯片
 * @note  由 BusGuard 的析构函数调用。全文件仅 Init() 开头直接调用一次，
 *        用于上电时把 CS 置到确定的高电平状态。
 * @note  拉高 CS 同时也是"命令结束"的信号：擦除和页编程正是在 CS 的
 *        上升沿开始真正执行的。
 */
void W25Q128::deselect(){
    HAL_GPIO_WritePin(m_handleTypeDef.cs_port, m_handleTypeDef.cs_pin, GPIO_PIN_SET);
}

/**
 * @brief 发送 24 位地址（高字节在前）
 * @param address 24 位地址
 * @retval Result::OK 成功
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 *
 * @warning **调用方必须已持有 BusGuard。** 本函数不拿锁也不动 CS，
 *          它是事务的一部分而非完整事务。自己拿锁会导致互斥量重入死锁。
 * @note  W25Q128 容量 16MB，恰好等于 24 位地址空间（2^24），
 *        因此三字节寻址刚好够用，不需要 4 字节地址模式。
 * @note  超时写的是字面量 1000 而非 SpiTimeoutMs。
 */
W25Q128::Result W25Q128::sendAddress(uint32_t address)
{
    uint8_t addressBytes[3];

    addressBytes[0] = static_cast<uint8_t>((address >> 16) & 0xFFU);
    addressBytes[1] = static_cast<uint8_t>((address >> 8)  & 0xFFU);
    addressBytes[2] = static_cast<uint8_t>(address & 0xFFU);

    HAL_StatusTypeDef transmitStatus =
        SPI_Transmit(
            m_handleTypeDef.spiHandle,
            addressBytes,
            3U,
            1000U
        );

    if (transmitStatus != HAL_OK)
    {
        return Result::COMMUNICATION_ERROR;
    }

    return Result::OK;
}

/**
 * @brief 读状态寄存器 1
 * @param[out] status 读回的寄存器值
 * @retval Result::OK 成功
 * @retval Result::BUSY 总线锁超时
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 *
 * @note  状态寄存器 1 的位定义：
 *        bit0 BUSY  —— 擦除/编程进行中
 *        bit1 WEL   —— 写使能锁存
 *        bit2-4 BP0-2、bit5 TB、bit6 SEC —— 块保护（非易失）
 *        bit7 SRP0  —— 状态寄存器保护
 * @note  0x05 是芯片忙碌期间**唯一**还能响应的命令，这正是轮询用它的原因。
 * @note  自成一个完整事务，内部管理 BusGuard。调用方不得在持锁状态下调用。
 * @warning 失败时 status 保持调用方给的初值不变，调用方必须先判返回值
 *          再用 status，否则会把 0 当成"不忙且无保护"。
 */
W25Q128::Result W25Q128::readStatus1(uint8_t& status){
    uint8_t rxData = 0; 
    const uint8_t command = CMD_READ_STATUS1;
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    HAL_StatusTypeDef receivestatus = SPI_Receive(m_handleTypeDef.spiHandle, &rxData, 1, 1000);
    if (receivestatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }

    status = rxData;
    return Result::OK;
}

/**
 * @brief 发送写使能命令 0x06
 * @retval Result::OK 命令已发出
 * @retval Result::BUSY 总线锁超时
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 *
 * @note  每次 program、erase、写状态寄存器**之前都必须重发一次**。
 *        芯片在每条写/擦命令执行完毕后会自动清除 WEL 位，不是一次使能长期有效。
 * @note  自成一个完整事务。因此调用它的 programPage / eraseXxx 必须在
 *        它返回之后才创建自己的 BusGuard —— 反过来会重入死锁。
 *
 * @warning **只确认命令发出去了，不确认 WEL 位真的被置上。**
 *          若芯片被硬件写保护（WP# 引脚）或状态寄存器保护，WEL 置不上，
 *          后续的页编程会被芯片静默丢弃，而本函数返回 OK。
 *          稳妥做法是发完回读一次状态寄存器，确认 bit1 为 1。
 * @warning writeEnable 返回到调用方创建 BusGuard 之间存在一个空档。
 *          若有第二个任务在这个窗口里插入一次写操作，会消耗掉 WEL，
 *          导致本次写入失败。单任务使用 flash 时不存在此问题。
 */
W25Q128::Result W25Q128::writeEnable(){
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    const uint8_t command = CMD_WRITE_ENABLE;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    return Result::OK;
}

/**
 * @brief 轮询状态寄存器直到芯片空闲或超时
 * @param timeoutMs 最长等待毫秒数
 * @retval Result::OK 芯片已空闲
 * @retval Result::TIME_OUT 超时仍在忙
 * @retval 其余错误码由 readStatus1 产生
 *
 * @note  **绝不能在持有 BusGuard 时调用。** 内部每毫秒调一次 readStatus1，
 *        那里要重新拿总线锁，普通互斥量不可重入。
 * @note  不忙等：每轮让出 CPU 1ms。擦除一个扇区最坏 400ms，忙等会把
 *        整条总线和 CPU 一起占死。让出期间同总线的其他器件可以正常工作，
 *        本函数只是每毫秒借用总线读 2 字节，占空比极低。
 * @note  调度器未启动时（Init 阶段）走 HAL_Delay 分支：那时没有当前任务，
 *        vTaskDelay 会 configASSERT 失败或 HardFault。
 * @note  用 HAL_GetTick() 计时而非 xTaskGetTickCount()：前者在调度器启动
 *        前后都可用。uint32_t 减法天然处理计数器回绕（49.7 天一圈）。
 *
 * @warning timeoutTicks、currentTick、elapsedTicks 三个变量已经没有任何
 *          使用者——超时判断改用毫秒直接比较后，这几行是残留的死代码，
 *          `-Wunused-variable` 会报 elapsedTicks。顺带每轮多调了一次 HAL_GetTick()。
 * @warning startTick 声明为 TickType_t 但存的是 HAL 毫秒计数，类型名有误导性，
 *          应为 uint32_t。当前 configTICK_RATE_HZ 为 1000 时两者数值一致。
 * @warning vTaskDelay 的参数单位是 tick 不是毫秒。此处传 1 在 1000Hz 下
 *          恰好是 1ms，改成 500Hz 后会变成 2ms，应写 pdMS_TO_TICKS(1)。
 */
W25Q128::Result W25Q128::waitReady(uint32_t timeoutMs)
{
    static constexpr uint8_t STATUS_BUSY_MASK = 0x01U;
    const TickType_t startTick = HAL_GetTick();
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);
    // 防止 timeoutMs 不为 0，但由于系统 Tick 精度较低而转换成 0 Tick
    if ((timeoutMs > 0U) && (timeoutTicks == 0U))
    {
        timeoutTicks = 1U;
    }

    while (true)
    {
        uint8_t status = 0U;
        Result readStatusResult = readStatus1(status);
        if (readStatusResult != Result::OK)return readStatusResult;
        const bool isBusy = (status & STATUS_BUSY_MASK) != 0U;
        if (!isBusy)return Result::OK;

        const TickType_t currentTick =
            HAL_GetTick();

        const TickType_t elapsedTicks =
            currentTick - startTick;

        if ((HAL_GetTick() - startTick) >= timeoutMs) return Result::TIME_OUT;
        if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) HAL_Delay(1);
        else vTaskDelay(1U);
    }
}

/**
 * @brief 读取 JEDEC ID（命令 0x9F）
 * @param[out] id 三字节 ID 拼成的 32 位值，高位补零
 * @retval Result::OK 成功
 * @retval Result::BUSY 总线锁超时
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 *
 * @note  W25Q128JV 的正确返回值是 0xEF4018：
 *        0xEF 厂商 Winbond、0x40 存储类型、0x18 容量 128Mbit。
 * @note  这是最好的硬件自检手段。首次上板按读回值判断：
 *        0xEF4018 —— 接线与 SPI 配置全部正确
 *        0xFFFFFF —— MISO 悬空或 CS 没拉低
 *        0x000000 —— MISO 短地，或时钟没有输出
 *        其他值   —— CPOL/CPHA 配错（本芯片需 Mode 0 或 Mode 3）
 * @note  自成一个完整事务，内部管理 BusGuard。
 */
W25Q128::Result W25Q128::readJedecId(uint32_t& id){
    uint8_t rxData[3] = {0}; 
    const uint8_t command = CMD_READ_JEDEC_ID;
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    HAL_StatusTypeDef receivestatus = SPI_Receive(m_handleTypeDef.spiHandle, rxData, 3, 1000);
    if (receivestatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    id = (static_cast<uint32_t>(rxData[0]) << 16) | (static_cast<uint32_t>(rxData[1]) << 8)  | static_cast<uint32_t>(rxData[2]);
    return Result::OK;
}

/**
 * @brief 页编程：写入不超过一页且不跨页边界的数据
 * @param address 起始地址
 * @param data 待写数据
 * @param length 字节数，1~256，且不得跨越页边界
 * @retval Result::OK 写入完成（已等待芯片编程结束）
 * @retval Result::OUT_OF_RANGE 长度非法或跨页
 * @retval Result::BUSY 总线锁超时
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 * @retval Result::TIME_OUT 编程超时
 *
 * @note  跨页检查用的是取模而非位与。早期写成 `address & PageSize` 是错的：
 *        PageSize = 256 = 0x100，与运算只取出 bit8，结果非 0 即 256，
 *        导致所有奇数页的写入被误判为越界。正确的位运算写法是 & (PageSize-1)。
 * @note  执行顺序是硬性的：waitReady（等上次操作完）→ writeEnable（置 WEL）
 *        → 拿锁发命令 → 放锁 → waitReady（等本次编程完）。
 *        两次 waitReady 都在锁外，中间那段才持锁。
 * @note  那对独立的花括号是**功能性的**，不是格式：它让 BusGuard 在结尾的
 *        waitReady 之前析构。少了它，waitReady 会在持锁状态下被调用而重入死锁。
 * @note  页编程期间（典型 0.7ms、最坏 3ms）芯片自己忙，总线是空闲的，
 *        提前放锁让同总线的其他器件能用。
 */
W25Q128::Result W25Q128::programPage(uint32_t address, const uint8_t* data, uint32_t length){
    if (length == 0 || length > PageSize) return Result::OUT_OF_RANGE;
    uint32_t offset = address % PageSize;
    if(offset + length > PageSize) return Result::OUT_OF_RANGE;
    const uint8_t command = CMD_WRITE_COMMAND; 
    Result result = waitReady(ReadyTimeoutMs);
    if (result != Result::OK)return result;
    result = writeEnable();
    if (result != Result::OK)return result;
    {
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    result = sendAddress(address);
    if (result != Result::OK){
        return result;
    }
    transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, data, length, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    }
    return waitReady(PageProgramTimeoutMs);

}

/**
 * @brief 擦除一个 4KB 扇区（命令 0x20）
 * @param address 扇区起始地址，必须 4KB 对齐
 * @retval Result::OK 擦除完成
 * @retval Result::OUT_OF_RANGE 越界
 * @retval Result::INVALID_ARGUMENT 未对齐
 * @retval Result::BUSY 总线锁超时
 * @retval Result::TIME_OUT 擦除超时
 *
 * @note  4KB 是 W25Q128 的最小擦除单位，无法擦除更小的粒度。
 *        要改写单个字节也必须把整个扇区读出、修改、擦除、写回。
 * @note  擦除耗时典型 45ms、最坏 400ms。命令发出后立刻释放总线（花括号），
 *        这几百毫秒里同总线的其他器件完全不受影响，
 *        本函数只是通过 waitReady 每毫秒借用总线读一次状态。
 *
 * @warning 前置 waitReady 用的是 SectorEraseTimeoutMs。这个值同时被
 *          eraseBlock32K/64K 当作前置等待使用，而 64KB 块擦除最坏耗时
 *          就是 2000ms，与该常量当前取值相等，余量为零。
 *          前置等待应当单独定义一个更宽松的常量。
 */
W25Q128::Result W25Q128::eraseSector(uint32_t address){
    if (address > Capacity - SectorSize)return Result::OUT_OF_RANGE;
    if ((address % SectorSize) != 0U)return Result::INVALID_ARGUMENT;
    Result result = waitReady(SectorEraseTimeoutMs);
    if (result != Result::OK)return result;
    const uint8_t command = CMD_ERASE_SECTOR; 
    result = writeEnable();
    if (result != Result::OK)return result;
    {
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    result = sendAddress(address);
    if (result != Result::OK){
        return result;
    }
    }
    return waitReady(SectorEraseTimeoutMs);
    
}


/**
 * @brief 擦除一个 32KB 块（命令 0x52）
 * @param address 块起始地址，必须 32KB 对齐
 * @retval 同 eraseSector
 * @note  耗时典型 120ms、最坏 1600ms。擦同样容量比逐扇区快约 8 倍
 *        （8 个扇区 8 条命令 vs 1 条命令）。
 * @note  函数体与 eraseSector、eraseBlock64K 高度重复，仅命令字和超时不同，
 *        可合并为 eraseCommand(command, address, timeoutMs)。
 */
W25Q128::Result W25Q128::eraseBlock32K(uint32_t address){
    if (address > Capacity - Block32Size)return Result::OUT_OF_RANGE;
    if(address % Block32Size != 0) return Result::INVALID_ARGUMENT;
    Result result = waitReady(SectorEraseTimeoutMs);
    if (result != Result::OK)return result;
    const uint8_t command = CMD_ERASE_BLOCK_32K; 
    result = writeEnable();
    if (result != Result::OK)return result;
    {
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    result = sendAddress(address);
    if (result != Result::OK){
        return result;
    }
    }
    return waitReady(Block32EraseTimeoutMs);
    
}

/**
 * @brief 擦除一个 64KB 块（命令 0xD8）
 * @param address 块起始地址，必须 64KB 对齐
 * @retval 同 eraseSector
 * @note  耗时典型 150ms、最坏 2000ms。W25Q128 共 256 个 64KB 块。
 *        这是擦除大段区域时效率最高的单位，doErase 的贪心策略优先选它。
 * @warning 前置 waitReady 传的是 SectorEraseTimeoutMs（2000ms），
 *          恰好等于本函数自身最坏耗时。连续擦多个 64K 块时，
 *          若上一块用满最坏时间，这里可能误报 TIME_OUT。
 */
W25Q128::Result W25Q128::eraseBlock64K(uint32_t address){
    if (address > Capacity - Block64Size)return Result::OUT_OF_RANGE;
    if(address % Block64Size  != 0) return Result::INVALID_ARGUMENT;
    Result result = waitReady(SectorEraseTimeoutMs);
    if (result != Result::OK)return result;
    const uint8_t command = CMD_ERASE_BLOCK_64K; 
    result = writeEnable();
    if (result != Result::OK)return result;
    {
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    result = sendAddress(address);
    if (result != Result::OK){
        return result;
    }
    }
    return waitReady(Block64EraseTimeoutMs);
    
}

/**
 * @brief 全片擦除（命令 0xC7）
 * @retval Result::OK 全片擦除完成
 * @retval Result::BUSY 总线锁超时
 * @retval Result::COMMUNICATION_ERROR SPI 层错误
 * @retval Result::TIME_OUT 超时
 *
 * @note  唯一不带地址参数的擦除命令，只发一个字节。
 * @note  耗时典型 40s、最坏 **200s**。FullChipEraseTimeoutMs 取 250000
 *        正是为了覆盖最坏值——芯片擦写次数上去之后会明显变慢。
 * @note  这三分多钟里芯片自己在忙，总线是空闲的：命令发完立刻放锁，
 *        waitReady 每毫秒借用一次总线读状态。同总线的其他器件几乎不受影响。
 *
 * @warning 调用方（doErase）在 address==0 && length==Capacity 时才走到这里。
 *          飞行途中绝对不能调用——三分钟内 flash 完全不可写。
 *          全片擦除应当只在地面准备阶段进行。
 */
W25Q128::Result W25Q128::eraseChip(){
    Result result = waitReady(ReadyTimeoutMs);
    if (result != Result::OK)return result;
    result = writeEnable();
    if (result != Result::OK)return result;
    const uint8_t command = CMD_ERASE_FULL_CHIP; 
    {
    BusGuard guard(*this, BusLockTimeoutMs);
    if (!guard.ok()) return Result::BUSY;
    HAL_StatusTypeDef transmitstatus = SPI_Transmit(m_handleTypeDef.spiHandle, &command, 1, 1000);
    if(transmitstatus != HAL_OK){
        return Result::COMMUNICATION_ERROR;
    }
    }
    return waitReady(FullChipEraseTimeoutMs);
}