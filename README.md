## Project Structure

```text
Rocket_FlightControl/
├── Application/                # 应用层，调用底层RSL库
│   ├── Rocket/                 # 飞行器总类
│   └── Logging/                # 日志系统类实现，封装每个message的写入函数
│   └── Command/                # 命令系统
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
│       ├── tsk_Log.cpp         # 日志任务
│
│
├── CMakeLists.txt              # 总工程编译清单
├── CMakePresets.json           # Cmake编译配置
```
