#!/bin/bash
# flash.sh
# 自动编译并烧录 STM32 项目，支持 DAP-Link / ST-Link

# ========== 配置区（根据你的项目修改这里）==========
PROJECT_DIR="/home/snowflake/Development/Personal/Inertial-Guidance-Lower"        # 项目根目录
BUILD_DIR="$PROJECT_DIR/build/Debug"             # 编译输出目录

# 烧录器选择（取消注释你要用的那一行）
# INTERFACE_CFG="interface/cmsis-dap.cfg"          # DAP-Link（有线/无线通用）
INTERFACE_CFG="interface/stlink.cfg"           # ST-Link

# 芯片型号选择（取消注释你要用的那一行）
# TARGET_CFG="target/stm32f1x.cfg"              # F103系列
# TARGET_CFG="target/stm32f4x.cfg"              # F407系列（大疆C板）
TARGET_CFG="target/stm32f4x.cfg"                # F411系列（BlackPill）← 同样用f4x
# ===================================================

# 查找 .elf 文件
ELF_FILE=$(find "$BUILD_DIR" -name "*.elf" | head -n 1)

if [ -z "$ELF_FILE" ]; then
    echo "❌ 未找到 .elf 文件，请先编译项目！"
    exit 1
fi

echo "-----------------------------------"
echo "📦 固件文件: $ELF_FILE"
echo "🔌 烧录器:   $INTERFACE_CFG"
echo "🎯 目标芯片: $TARGET_CFG"
echo "-----------------------------------"
echo "🔥 开始烧录..."

# 执行烧录
openocd -f "$INTERFACE_CFG" \
        -f "$TARGET_CFG" \
        -c "program $ELF_FILE verify reset exit"

# 判断结果
if [ $? -eq 0 ]; then
    echo "✅ 烧录成功！"
else
    echo "❌ 烧录失败，请检查："
    echo "   1. DAP-Link 是否已通过 attach_daplink.ps1 转发给 WSL"
    echo "   2. 单片机是否正确连接"
    echo "   3. INTERFACE_CFG 和 TARGET_CFG 是否与实际硬件匹配"
fi
