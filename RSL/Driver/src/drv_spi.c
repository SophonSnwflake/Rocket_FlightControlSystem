/**
 * @file    drv_spi.c
 * @brief   SPI 板级驱动层：在 HAL 之上提供统一的收发接口、回调管理和总线互斥锁
 *
 * 本文件位于 HAL 与设备驱动层（dvc_xxx）之间，向上屏蔽 HAL 的句柄细节，
 * 向下负责多设备共享同一条 SPI 总线时的互斥仲裁。
 *
 * 两套相互独立的机制：
 *   1. 总线锁      —— s_busMutex[]，按 SPI 外设实例分配，防止多任务交叠访问
 *   2. 控制器对象  —— spiController[]，保存每条总线的句柄、接收缓冲和完成回调
 *
 * @note  两套机制各自用一套下标：总线锁用 SPI_getBusIndex()，控制器用
 *        SPI_GetManageObject() 的线性查找。两者顺序恰好一致，但代码上没有约束，
 *        修改任一处时需同步检查另一处。
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "drv_spi.h"


/**
 * @brief 总线锁与控制器数组的容量
 * @note  STM32F411 只有 SPI1~SPI5，实际只用到 5 个。第 6 个槽位恒为空，
 *        多占 sizeof(StaticSemaphore_t) + sizeof(SemaphoreHandle_t) 的 .bss。
 */
#define SPI_BUS_COUNT 6
//-------------------------总线锁-------------------------
//该总线锁仅用于阻塞传输，如果是DMA或中断传输，总线锁将不起作用

/**
 * @brief 互斥量的静态存储区，每条总线一份
 * @note  必须是文件级静态变量（静态存储期）。FreeRTOS 会一直使用这块内存，
 *        放在栈上会在函数返回后被覆盖。
 * @note  依赖 FreeRTOSConfig.h 中 configSUPPORT_STATIC_ALLOCATION 为 1。
 */
static StaticSemaphore_t s_busMutexBuffer[SPI_BUS_COUNT];

/**
 * @brief 互斥量句柄数组，NULL 表示该总线尚未调用过 SPI_BusInit()
 */
static SemaphoreHandle_t s_busMutex[SPI_BUS_COUNT] = {NULL, NULL, NULL, NULL, NULL, NULL};

/**
 * @brief 根据传入的spi句柄获得总线锁实例索引
 * @param hspi spi句柄
 * @retval 0~4  对应 SPI1~SPI5 的数组下标
 * @retval -1   未识别的外设实例，调用方按“该总线无需加锁”处理
 * @note  返回 -1 是正常路径而非错误：独占总线的外设不需要互斥。
 * @warning 不检查 hspi 是否为 NULL，传空指针会在解引用 Instance 时 HardFault。
 */
static int SPI_getBusIndex(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) return 0;
    if (hspi->Instance == SPI2) return 1;
    if (hspi->Instance == SPI3) return 2;
    if (hspi->Instance == SPI4) return 3;
    if (hspi->Instance == SPI5) return 4;

    return -1;                              
}

/**
 * @brief 根据传入的spi句柄初始化总线锁实例
 * @param hspi spi句柄
 * @note  幂等：重复调用同一条总线不会重复创建，第二次直接返回。
 * @note  调用时机必须在 MX_SPIx_Init() 之后（需要 hspi->Instance 已赋值）、
 *        调度器启动之前。每条用到的总线都要单独调一次。
 * @note  main.c 中要放在 /* USER CODE BEGIN 2 *​/ 标记内，否则 CubeMX
 *        重新生成代码时这几行会被覆盖删除。
 * @warning 无返回值。index < 0 或创建失败（静态分配未开启）时静默跳过，
 *          调用方无从得知。失败的后果要到 SPI_BusLock() 返回 HAL_ERROR 才暴露。
 */
void SPI_BusInit(SPI_HandleTypeDef *hspi)
{
    const int index = SPI_getBusIndex(hspi);
    if (index < 0) return;
    if (s_busMutex[index] != NULL) return;   
    s_busMutex[index] = xSemaphoreCreateMutexStatic(&s_busMutexBuffer[index]);
}

/**
 * @brief 启动总线锁
 * @param hspi spi句柄
 * @param timeoutMs 最大锁止时间
 * @retval HAL_OK       已取得总线所有权，或该总线无需加锁
 * @retval HAL_TIMEOUT  超时未取得，调用方应放弃本次操作而非继续访问总线
 * @retval HAL_ERROR    互斥量不存在，通常是忘了调用 SPI_BusInit()
 *
 * @note  三个前置判断的顺序不可调换：
 *        1) index < 0        —— 独占总线，无锁可拿，直接放行
 *        2) 调度器未启动     —— 此时不存在并发，且 xSemaphoreTake 会因为没有
 *                               当前任务而 configASSERT 或 HardFault。外设
 *                               初始化（如 flash 的 init()）正是跑在这个阶段。
 *        3) 句柄为 NULL      —— 配置错误，报错而非静默放行
 *
 * @note  使用 xSemaphoreCreateMutexStatic 创建的互斥量带优先级继承：
 *        低优先级任务持锁期间被高优先级任务等待时，会临时提升其优先级，
 *        避免优先级反转。二值信号量没有这个特性。
 *
 * @warning 每次成功返回 HAL_OK 都必须配对一次 SPI_BusUnlock()。
 *          漏掉一次即整条总线永久死锁。
 * @warning 互斥量不可重入：同一任务连续两次 Lock 会自锁死。因此调用链上
 *          不得出现“持锁时再调用一个内部也要拿锁的函数”，典型例子是
 *          flash 驱动里持锁调用 waitReady()。
 * @warning 严禁在中断服务程序中调用：ISR 没有任务身份，无法阻塞，也无法继承优先级。
 */
HAL_StatusTypeDef SPI_BusLock(SPI_HandleTypeDef *hspi, uint32_t timeoutMs)
{
    const int index = SPI_getBusIndex(hspi);
    if (index < 0) return HAL_OK;
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return HAL_OK;
    if (s_busMutex[index] == NULL) return HAL_ERROR;

    if (xSemaphoreTake(s_busMutex[index], pdMS_TO_TICKS(timeoutMs)) == pdTRUE)return HAL_OK;
    return HAL_TIMEOUT;
}

/**
 * @brief 释放总线锁
 * @param hspi spi句柄
 *
 * @note  三个前置判断与 SPI_BusLock() 严格镜像——Lock 在哪些情况下没有真正
 *        取锁，Unlock 就必须在同样的情况下不释放。特别是调度器未启动那一条：
 *        此时 Lock 直接放行并未取锁，Unlock 若照常 Give，就是释放一把无人
 *        持有的互斥量，会破坏 FreeRTOS 内部的持有者记录和优先级继承簿记。
 *
 * @note  xSemaphoreGive 返回 pdFALSE 意味着当前任务并不持有这把锁，
 *        即 Lock/Unlock 未配对，属于调用方的逻辑错误。
 *
 * @warning configASSERT 在 Release 构建中通常展开为空语句，此时该错误被静默吞掉。
 *          断言只在 Debug 构建下有效。
 */
void SPI_BusUnlock(SPI_HandleTypeDef *hspi)
{
    const int index = SPI_getBusIndex(hspi);
    if (index < 0) return;
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
    if (s_busMutex[index] == NULL) return;
    if (xSemaphoreGive(s_busMutex[index]) != pdTRUE)
    {
        configASSERT(0);   /* lock/unlock 不配对 */
    }
}

//-------------------------SPI实现-------------------------

/**
 * @brief 每条总线的控制器对象，保存句柄、接收缓冲区和完成回调
 * @warning 未加 static，是全局符号。其他编译单元可以直接引用，
 *          且与同名全局变量会产生链接冲突。
 */
SPI_Controller_t spiController[6] = {0};

/**
 * @brief 外设实例查找表，下标与 spiController[] 一一对应
 * @note  只有 5 个初值，第 6 项由 C 语言规则隐式初始化为 NULL。
 *        NULL 永远不会等于任何有效的 hspi->Instance，因此不会误匹配。
 * @note  下标顺序与 SPI_getBusIndex() 的返回值一致（SPI1→0 … SPI5→4），
 *        但这是人为保证的，编译器不会检查。
 */
static SPI_TypeDef *const SPI_Instances[SPI_BUS_COUNT] = {
    SPI1, SPI2, SPI3, SPI4, SPI5
};

/**
 * @brief 由 spi 句柄查找对应的控制器对象
 * @param hspi spi句柄
 * @retval 非 NULL  对应的控制器指针
 * @retval NULL     该外设不在查找表中
 * @note  线性扫描，最坏 6 次比较。每次收发都会调用一次，
 *        高频轮询场景（如 flash 的 1kHz 忙等待）会持续付出这份开销。
 * @note  硬编码的循环上界 6 与 SPI_BUS_COUNT 重复，改宏时需同步修改此处。
 */
static SPI_Controller_t *SPI_GetManageObject(SPI_HandleTypeDef *hspi){
    for(uint8_t i = 0; i < 6; i++){
        if(SPI_Instances[i] == hspi->Instance){
            return &spiController[i];
        }
    }
    return NULL;
}

/**
 * @brief 绑定 spi 句柄与完成回调到对应的控制器对象
 * @param hspi spi句柄
 * @param callback 中断传输完成后调用的回调，可传 NULL 表示暂不设置
 * @note  只登记，不做任何硬件配置——外设本身由 CubeMX 生成的 MX_SPIx_Init() 初始化。
 * @note  与 SPI_BusInit() 是两件独立的事，都需要调用。
 * @warning 无返回值。句柄不在查找表中时静默失败，后续所有收发都会返回 HAL_ERROR。
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback callback){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        spi_controller->hspi = hspi;
        spi_controller->callback = callback;
    }
}

/**
 * @brief 运行期更换某条总线的完成回调
 * @param hspi spi句柄
 * @param callback 新的回调函数
 * @note  用于同一条总线上多个设备轮流使用中断传输时切换处理函数。
 * @warning 不检查此刻是否有传输正在进行。传输途中更换回调，
 *          本次传输完成时会由新回调接手，数据归属会错乱。
 */
void SPI_SwitchCallBackFunction(SPI_HandleTypeDef *hspi, SPI_Callback callback){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        spi_controller->callback = callback;
    }
}

/**
 * @brief 阻塞方式发送
 * @param hspi spi句柄
 * @param pData 待发送数据，函数不会修改其内容
 * @param length 字节数，上限 65535
 * @param timeout HAL 层超时，单位毫秒
 * @retval HAL_OK / HAL_TIMEOUT / HAL_ERROR / HAL_BUSY
 * @note  函数返回时数据已全部移出发送移位寄存器，因此调用方持有的总线锁
 *        可以在返回后立即释放。
 * @note  查表得到的 spi_controller 只用于判断该外设是否已登记，
 *        其成员并未参与传输。
 * @warning length 为 uint16_t。调用方若持有 uint32_t 长度，超过 65535 会被
 *          隐式截断且不产生任何警告——需由调用方自行切块。
 */
HAL_StatusTypeDef SPI_Transmit(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t length,uint32_t timeout){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Transmit(hspi, pData,length,timeout);
    }
    return HAL_ERROR;
}

/**
 * @brief 阻塞方式接收
 * @param hspi spi句柄
 * @param pData 接收缓冲区，由调用方提供并保证容量不小于 length
 * @param length 字节数，上限 65535
 * @param timeout HAL 层超时，单位毫秒
 * @retval HAL_OK / HAL_TIMEOUT / HAL_ERROR / HAL_BUSY
 * @note  接收期间 MOSI 上发出的是 HAL 内部的填充数据，从机看到的是无效字节。
 * @warning 同 SPI_Transmit：length 为 uint16_t，超长会被静默截断。
 */
HAL_StatusTypeDef SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length, uint32_t timeout){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Receive(hspi,pData,length,timeout);
    }
    return HAL_ERROR;
}

/**
 * @brief 阻塞方式全双工收发
 * @param hspi spi句柄
 * @param pTxData 待发送数据
 * @param pRxData 接收缓冲区
 * @param length 收发共用的字节数——全双工下两者必然相等
 * @param timeout HAL 层超时，单位毫秒
 * @retval HAL_OK / HAL_TIMEOUT / HAL_ERROR / HAL_BUSY
 * @note  适用于“发命令的同时读回数据”的器件。若只关心其中一个方向，
 *        用 SPI_Transmit / SPI_Receive 更省事。
 * @note  pTxData 与 pRxData 可以指向同一块内存，HAL 支持原地收发。
 */
HAL_StatusTypeDef SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData,uint16_t length, uint32_t timeout){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_TransmitReceive(hspi, pTxData,pRxData,length,timeout);
    }
    return HAL_ERROR;
}

/**
 * @brief 中断方式发送，立即返回
 * @param hspi spi句柄
 * @param pData 待发送数据
 * @param length 字节数
 * @retval HAL_OK 表示传输已启动，不代表已完成
 *
 * @warning pData 指向的内存在传输结束前不得释放或修改。函数返回时
 *          数据尚未发出，HAL 只保存了指针。局部数组会在函数退出后失效。
 * @warning 与总线锁不兼容：函数返回即脱钩，调用方此时释放锁，
 *          另一任务会在本次传输仍在后台进行时拉低自己的 CS。
 *          共享总线上的设备应使用阻塞版本。
 * @warning 本文件未实现 HAL_SPI_TxCpltCallback，因此纯发送完成时
 *          不会产生任何通知，调用方无从得知传输何时结束。
 * @note    pData 未加 const，与阻塞版 SPI_Transmit 的签名不一致。
 */
HAL_StatusTypeDef SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL){
        return HAL_SPI_Transmit_IT(hspi, pData, length);
    }
    return HAL_ERROR;
}

/**
 * @brief 中断方式接收，立即返回。数据收进控制器内部缓冲区
 * @param hspi spi句柄
 * @param length 字节数，不得超过 RX_BUFFER_SIZE
 * @retval HAL_OK    传输已启动
 * @retval HAL_ERROR length 超出内部缓冲容量，或该外设未登记
 *
 * @note  与其他函数不同，本函数不接受外部缓冲区——数据固定落在
 *        spi_controller->rxBuffer，完成后由 HAL_SPI_RxCpltCallback 转交给回调。
 * @note  length 的边界检查位于取控制器之后、判空之前。此时 spi_controller
 *        可能为 NULL，但该分支并未解引用它，因此不会出错。
 * @warning 同 SPI_Transmit_IT：与总线锁不兼容。
 */
HAL_StatusTypeDef SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if (length > RX_BUFFER_SIZE) return HAL_ERROR;
    if(spi_controller != NULL){
        return HAL_SPI_Receive_IT(hspi, spi_controller->rxBuffer, length);
    }
    return HAL_ERROR;
}

/**
 * @brief 中断方式全双工收发，立即返回。接收数据落在控制器内部缓冲区
 * @param hspi spi句柄
 * @param pTxData 待发送数据
 * @param length 收发共用的字节数
 * @retval HAL_OK    传输已启动
 * @retval HAL_ERROR 该外设未登记
 *
 * @warning pTxData 指向的内存在传输结束前不得释放或修改。
 * @warning 同 SPI_Transmit_IT：与总线锁不兼容。
 * @note    pTxData 未加 const，与阻塞版 SPI_TransmitReceive 的签名不一致。
 */
HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t length){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(length > RX_BUFFER_SIZE) return HAL_ERROR;
    if(spi_controller != NULL){
        return HAL_SPI_TransmitReceive_IT(hspi, pTxData, spi_controller->rxBuffer, length);
    }
    return HAL_ERROR;
}


/**
 * @brief HAL 全双工中断传输完成回调，覆盖 HAL 的弱定义
 * @param hspi 触发本次回调的 spi 句柄，由 HAL 传入
 *
 * @note  由 SPI_TransmitReceive_IT 启动的传输完成后，在中断上下文中被 HAL 调用。
 * @note  hspi->RxXferSize 是本次请求的字节数，不是实际成功接收的字节数。
 *
 * @warning 运行在中断上下文：用户回调内不得调用任何阻塞 API，
 *          FreeRTOS 相关调用必须使用 xxxFromISR 版本。
 * @warning 该函数覆盖 HAL 的 __weak 定义。若工程内其他文件也定义了同名函数，
 *          链接阶段会报重复符号。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    SPI_Controller_t *spi_controller = SPI_GetManageObject(hspi);
    if(spi_controller != NULL && spi_controller->callback != NULL){
        spi_controller->callback(spi_controller->rxBuffer, hspi->RxXferSize);
    }
}

/**
 * @brief HAL 单向接收中断完成回调，覆盖 HAL 的弱定义
 * @param hspi 触发本次回调的 spi 句柄，由 HAL 传入
 *
 * @note  由 SPI_Receive_IT 启动的传输完成后被调用。HAL 对纯接收和全双工
 *        使用两个不同的回调，因此这两个函数都必须实现，缺一个对应的
 *        启动函数就永远等不到通知。
 * @note  函数体与 HAL_SPI_TxRxCpltCallback 完全相同，仅局部变量命名不同。
 *
 * @warning 同上：运行在中断上下文，且覆盖 HAL 弱定义。
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
    SPI_Controller_t *c = SPI_GetManageObject(hspi);
    if(c != NULL && c->callback != NULL){
        c->callback(c->rxBuffer, hspi->RxXferSize);
    }
}