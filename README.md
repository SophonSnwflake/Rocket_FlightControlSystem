# 火箭飞行控制系统

基于STM32F411与FreeRTOS的实验火箭飞行控制系统，集成姿态解算、GNSS 定位、LoRa 遥测与飞行数据记录。

## 硬件

- STM32F411
- SX1268 LoRa
- BMI088 IMU
- NEO-M9N GNSS
- W25Q128 Flash
- BMP388 气压计

## 功能
- 基于STM32与FreeRtos的嵌入式飞行控制系统
- IMU 姿态解算
- 气压计 气压解算以及高度测算
- GNSS 定位与导航数据获取
- Flash 飞行数据写入
- Logger 自描述格式的信息编码和数据记录
- Commander 命令树结构的命令行编码以及解码系统，支持地面站指令接收与执行
- Communication Lora 双向无线通信与遥测
- 飞行状态监测与状态机管理
- 系统状态、电压及传感器信息遥测
- 支持降落伞部署控制

## 工程结构

```text
Rocket_FlightControl/
├── Application/                # 应用层，调用底层RSL库
│   ├── Rocket/                 # 飞行器总类
│   └── Logging/                # 日志系统类实现，封装每个message的写入函数
│   └── Command/                # 命令系统
│   └── Communication/          # Lora上层通信，封装Lora循环以及编码格式相关函数
│
├── RSL/                        # 标准库
│   ├── Aggreement/             # 协议   
│   └── Algorithm/              # 数学算法
|   └── Dependence/             # 外部依赖库
|   └── Device/                 # 设备
|   └── Driver/                 # 驱动
|   └── include/                # 通用头文件
|   └── Middleware/             # 中间层（既不属于硬件又不属于算法）
|   └── CMakeList.txt           # RSL编译清单
│
├── Task/                       # 具体任务，由FreeRTOS驱动
│   ├── inc/
│   └── src/
│       ├── tsk_rocket.cpp      # 主任务循环入口
│       ├── tsk_imu.cpp         # IMU任务
│       ├── tsk_isr.cpp         # FreeRTOS中断回调注册函数，非任务
│       ├── tsk_logger.cpp         # 日志任务
│
│
├── CMakeLists.txt              # 总工程编译清单
├── CMakePresets.json           # Cmake编译配置
```


## 当前进度

项目目前处于软件功能集成与系统联调阶段。

### 已完成

* [x] BMI088 IMU 驱动与数据采集
* [x] 基于四元数的卡尔曼滤波解算与 AHRS 基础功能
* [x] W25Q128 Flash 驱动
* [x] SX1268 LoRa 驱动
* [x] GNSS 驱动
* [x] LoRa 基础数据发送与接收
* [x] Command 指令系统设计与实现
* [x] 飞行数据 Logger 多类型飞行数据记录框架
* [x] FreeRTOS 多任务运行框架
* [x] GNSS 数据读取验证
* [x] 基础遥测数据结构设计
* [x] Communicator 通信模块基础框架

### 进行中

* [ ] LoRa 双向通信流程完善与稳定性测试
* [ ] 遥测数据编码、封包与解析
* [ ] Communicator 与各飞控模块的数据流集成
* [ ] GNSS 在正式飞控 PCB 上的硬件联调
* [ ] AHRS 参数调试
* [ ] 地面站遥测数据解析

### 待实现

* [ ] 火箭飞行状态机
* [ ] 飞行阶段识别与状态转换逻辑
* [ ] 降落伞部署控制
* [ ] 电池电压监测
* [ ] 完整系统异常处理与故障保护
* [ ] 地面站控制与数据可视化
* [ ] 整机地面测试
* [ ] 实际飞行测试
