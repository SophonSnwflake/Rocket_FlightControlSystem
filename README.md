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

## 编译命令

### Preset

#### Debug模式，调试时用这个
```
cmake --preset Debug
```

#### Release模式

```
cmake --preset Release
```

### 编译

#### Debug模式

```
cmake --build build/Debug
```

#### Release模式

```
cmake --build build/Release
```

### 烧录

#### ST-Link

```
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Debug/rocket_26.elf verify reset exit"
```

## 当前进度

本航电已于Oct 3 2026，搭载穹宇航天的小型火箭在河南新乡黄河滩地试飞，验证了所有系统工作正常且成功回收并读取飞行数据。任务日志与相关信息已上传至文档站。

## 未来计划

将依托此航电软件和硬件为平台，向上分为三个可能的方向开发：

1：惯性导航（传感器融合）算法。

2：飞行控制算法模拟及实现。

3：上位机视觉识别 + 上下位机通讯。

目前正在学习惯性导航相关算法文献。计划在11月中旬前在STM32上搭建出一个可输出的demo。预计使用误差卡尔曼观测器 & 姿态松耦合 实现。


## 计划 TimeLine

- Oct 5 - Oct 18 跟着文献内的推导学习完惯导建模全流程。完成目标：在笔记辅助下独立推导出整个数学建模流程。

- Oct 19 - Oct 21 阅读已有的C/C++惯导项目的代码形式/风格，选出最有代表性的一个进行后续参考。

- Oct 22 - Oct 28 在当前RSL下开始加入惯性导航库实现。

- Oct 29 - Nov 中旬 debug及测试


希望能够按计划进行吧,,,我不确定学期末的时候会不会特别忙或者有其他什么重要的事情冒出来,,
