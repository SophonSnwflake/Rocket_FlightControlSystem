#pragma once

#include <cstdint>

// 本协议基于 ULog 文件格式，采用自描述逻辑，参考：
// https://github.com/PX4/PX4-Autopilot.git

namespace RocketLog
{

/**
 * @brief ULog 文件魔数与协议版本标识
 *
 * 这 8 个字节必须位于整个 ULog 文件的最开始。
 *
 * 字节含义：
 * - 0x55, 0x4C, 0x6F, 0x67：ASCII 字符串 "ULog"
 * - 0x01, 0x12, 0x35：ULog 固定魔数字节
 * - 0x01：ULog 文件格式版本号
 *
 * 创建 ulog_file_header_s 时，应将该数组复制到 magic 字段中。
 */
inline constexpr uint8_t ULOG_MAGIC[8] = {
    0x55, 0x4C, 0x6F, 0x67,
    0x01, 0x12, 0x35, 0x01
};

/**
 * @brief ULog 消息类型
 *
 * 除 ulog_file_header_s 外，ULog 中的每一条消息都有一个
 * ASCII 字符形式的消息类型。
 *
 * msg_type 字段实际保存的是这些字符对应的 uint8_t 数值。
 */
enum class ULogMessageType : uint8_t
{
    /**
     * @brief 格式定义消息
     *
     * 描述一种可记录数据的名称、字段类型和字段顺序。
     *
     * 示例：
     * "rocket_imu:uint64_t timestamp;int16_t[3] accel;"
     */
    FORMAT = 'F',

    /**
     * @brief 数据消息
     *
     * 保存某个 msg_id 对应的实际二进制采样数据。
     */
    DATA = 'D',

    /**
     * @brief 兼容性标志消息
     *
     * 描述当前 ULog 文件使用的兼容功能、不兼容功能，
     * 以及是否存在追加的数据段。
     *
     * 通常应紧跟在 ulog_file_header_s 后面。
     */
    FLAG_BITS = 'B',

    /**
     * @brief 日志信息消息
     *
     * 保存固件版本、硬件版本、设备名称等日志元信息。
     */
    INFO = 'I',

    /**
     * @brief 参数消息
     *
     * 保存本次运行实际使用的某项参数及其值。
     */
    PARAMETER = 'P',

    /**
     * @brief 添加记录对象消息
     *
     * 建立 msg_id 与 FORMAT 消息名称之间的映射关系。
     */
    ADD_LOGGED_MSG = 'A',

    /**
     * @brief 移除记录对象消息
     *
     * 表示从当前位置开始，不再记录某个 msg_id。
     */
    REMOVE_LOGGED_MSG = 'R',

    /**
     * @brief 同步消息
     *
     * 在日志中插入固定同步标记，使解析器在局部数据损坏后
     * 有机会重新找到后续消息边界。
     */
    SYNC = 'S',
};


/*
 * ULog 是一种直接按字节写入存储介质的二进制协议。
 *
 * 必须禁止编译器在结构体字段之间自动插入 padding，
 * 否则结构体内存布局将与 ULog 文件格式不一致。
 *
 * 只对本区域内的协议结构体使用 1 字节对齐。
 */
#pragma pack(push, 1)

/**
 * @brief ULog 文件头
 *
 * 位于整个 ULog 字节流的最开始，并且只出现一次。
 *
 * 文件布局：
 *
 * [magic: 8 bytes]
 * [timestamp: 8 bytes]
 *
 * 该结构体不带普通 ULog 消息的：
 *
 * [msg_size][msg_type]
 *
 * 因为它本身就是整个文件的固定起始头。
 */
struct ulog_file_header_s
{
    /**
     * @brief ULog 文件魔数和格式版本
     *
     * 应复制 RocketLog::ULOG_MAGIC 中的 8 个字节。
     */
    uint8_t magic[8];

    /**
     * @brief 日志开始时间戳
     *
     * 建议统一使用微秒作为单位。
     *
     * 你的项目中可以定义为：
     * 飞控启动后经过的单调时间，单位为微秒。
     */
    uint64_t timestamp;
};


/**
 * @brief ULog 普通消息的公共头
 *
 * 除 ulog_file_header_s 外，Definitions 区和 Data 区中的
 * 每一条 ULog 消息都以这个 3 字节公共头开始。
 *
 * 字节布局：
 *
 * [msg_size: 2 bytes]
 * [msg_type: 1 byte]
 */
struct ulog_message_header_s
{
    /**
     * @brief 消息 payload 的长度
     *
     * 不包括当前公共消息头本身的 3 个字节：
     *
     * - uint16_t msg_size
     * - uint8_t msg_type
     *
     * 因此一条消息的完整字节长度为：
     *
     * 3 + msg_size
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型
     *
     * 取值对应 ULogMessageType 中定义的 ASCII 字符。
     *
     * 例如：
     * - 'F'：FORMAT
     * - 'D'：DATA
     * - 'A'：ADD_LOGGED_MSG
     */
    uint8_t msg_type;
};


/**
 * @brief FORMAT 格式定义消息
 *
 * FORMAT 消息描述一种业务数据的名称、字段类型和字段顺序。
 *
 * 示例：
 *
 * "rocket_imu:uint64_t timestamp;"
 * "uint32_t sequence;"
 * "int16_t[3] accel_raw;"
 *
 * FORMAT 只定义 payload 的结构，不分配 msg_id。
 *
 * msg_id 与 FORMAT 名称之间的映射由
 * ulog_message_add_logged_s 建立。
 */
struct ulog_message_format_s
{
    /**
     * @brief FORMAT 字符串的实际长度
     *
     * 不包括 3 字节公共消息头。
     *
     * 对 FORMAT 消息来说：
     *
     * msg_size = format 字符串的实际字节数
     *
     * 通常不包含字符串末尾的 '\0'。
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型，固定为 FORMAT，即字符 'F'
     */
    uint8_t msg_type =
        static_cast<uint8_t>(ULogMessageType::FORMAT);

    /**
     * @brief FORMAT 字符串临时缓冲区
     *
     * 字符串格式：
     *
     * "message_name:type field;type field;"
     *
     * 例如：
     *
     * "rocket_imu:uint64_t timestamp;"
     * "int16_t[3] accel_raw;"
     * "int16_t[3] gyro_raw;"
     *
     * 1600 是该构造缓冲区的最大容量，
     * 不表示每条 FORMAT 消息固定写入 1600 字节。
     *
     * 实际写入长度必须是：
     *
     * 3 + msg_size
     *
     * 不能直接使用 sizeof(ulog_message_format_s) 写入。
     */
    char format[1600];
};


/**
 * @brief ADD_LOGGED_MSG 消息
 *
 * 建立一个 msg_id 与某个 FORMAT 名称之间的对应关系。
 *
 * 例如：
 *
 * FORMAT：
 * "rocket_imu:uint64_t timestamp;..."
 *
 * ADD_LOGGED_MSG：
 * msg_id = 1
 * message_name = "rocket_imu"
 *
 * 此后 DATA 消息中的 msg_id = 1，
 * 就应按 rocket_imu 的 FORMAT 解析。
 */
struct ulog_message_add_logged_s
{
    /**
     * @brief 消息 payload 长度
     *
     * 计算方式：
     *
     * sizeof(multi_id)
     * + sizeof(msg_id)
     * + message_name 的实际长度
     *
     * 不包括 3 字节公共消息头。
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型，固定为 ADD_LOGGED_MSG，即字符 'A'
     */
    uint8_t msg_type =
        static_cast<uint8_t>(
            ULogMessageType::ADD_LOGGED_MSG);

    /**
     * @brief 同一种消息类型的实例编号
     *
     * 例如系统存在两颗 IMU：
     *
     * multi_id = 0：主 IMU
     * multi_id = 1：备用 IMU
     *
     * 如果只有一个实例，固定填写 0。
     */
    uint8_t multi_id;

    /**
     * @brief Logger 内部消息编号
     *
     * 后续 ulog_message_data_s 使用相同的 msg_id，
     * 表示其 payload 属于这里指定的 message_name。
     */
    uint16_t msg_id;

    /**
     * @brief FORMAT 消息中定义的消息名称
     *
     * 例如：
     *
     * "rocket_imu"
     * "rocket_baro"
     * "rocket_gnss"
     *
     * 名称必须和 FORMAT 字符串中冒号前面的名称完全相同。
     *
     * 255 是临时缓冲区最大容量，不表示固定写入 255 字节。
     *
     * 实际写入长度由 msg_size 决定，通常不写末尾 '\0'。
     */
    char message_name[255];
};


/**
 * @brief REMOVE_LOGGED_MSG 消息
 *
 * 表示从当前日志位置开始，停止记录指定的 msg_id。
 *
 * 对于整个航次都固定记录相同数据源的系统，
 * 第一版通常不需要使用该消息。
 */
struct ulog_message_remove_logged_s
{
    /**
     * @brief 消息 payload 长度
     *
     * 当前消息的 payload 只有一个 uint16_t msg_id，
     * 因此应为：
     *
     * sizeof(uint16_t) = 2
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型，固定为 REMOVE_LOGGED_MSG，即字符 'R'
     */
    uint8_t msg_type =
        static_cast<uint8_t>(
            ULogMessageType::REMOVE_LOGGED_MSG);

    /**
     * @brief 需要停止记录的消息编号
     *
     * 必须对应此前通过 ADD_LOGGED_MSG 注册过的 msg_id。
     */
    uint16_t msg_id;
};


/**
 * @brief SYNC 同步消息
 *
 * 用于在 ULog 字节流中周期性插入固定同步标记。
 *
 * 当日志局部损坏、消息长度错误或字节边界丢失时，
 * 解析器可以向后搜索 sync_magic，尝试恢复消息边界。
 *
 * SYNC 不是 CRC，不能检测所有数据错误，
 * 主要用于重新同步解析位置。
 */
struct ulog_message_sync_s
{
    /**
     * @brief 消息 payload 长度
     *
     * payload 只有 8 字节 sync_magic，因此应为 8。
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型，固定为 SYNC，即字符 'S'
     */
    uint8_t msg_type =
        static_cast<uint8_t>(ULogMessageType::SYNC);

    /**
     * @brief ULog 标准同步魔数
     *
     * 应写入规范规定的固定 8 字节同步序列，
     * 不能在每次写入时随机生成。
     */
    uint8_t sync_magic[8];
};


/**
 * @brief DATA 数据消息固定头
 *
 * DATA 消息包含某个 msg_id 对应的实际业务数据。
 *
 * 该结构体只定义到 msg_id，真正的业务 payload
 * 紧跟在该结构体后面。
 *
 * 实际布局：
 *
 * [msg_size]
 * [msg_type = 'D']
 * [msg_id]
 * [业务 payload...]
 *
 * 例如：
 *
 * [DATA header]
 * [ImuLogPayload]
 */
struct ulog_message_data_s
{
    /**
     * @brief DATA 消息 payload 的总长度
     *
     * 这里的 payload 包括：
     *
     * - msg_id
     * - 实际业务数据
     *
     * 因此计算方式为：
     *
     * msg_size = sizeof(msg_id) + sizeof(actual_payload)
     *
     * 不包括最前面的 3 字节公共消息头。
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型，固定为 DATA，即字符 'D'
     */
    uint8_t msg_type =
        static_cast<uint8_t>(ULogMessageType::DATA);

    /**
     * @brief 当前 payload 对应的消息编号
     *
     * 解析器通过此前的 ADD_LOGGED_MSG 消息找到：
     *
     * msg_id -> message_name
     *
     * 再通过 FORMAT 消息找到该 payload 的字段结构。
     */
    uint16_t msg_id;
};


/**
 * @brief PARAMETER 参数消息
 *
 * 保存本次航次实际使用的某项参数值。
 *
 * 参数消息逻辑布局：
 *
 * [key_len]
 * [key]
 * [binary value]
 *
 * key 中同时包含参数类型和参数名称，例如：
 *
 * "float launch_accel_threshold"
 *
 * value 则是该类型对应的二进制数据，例如 4 字节 float。
 */
struct ulog_message_parameter_s
{
    /**
     * @brief 参数消息 payload 的总长度
     *
     * 计算方式：
     *
     * sizeof(key_len)
     * + key 的实际字节数
     * + value 的二进制字节数
     *
     * 不包括 3 字节公共消息头。
     */
    uint16_t msg_size;

    /**
     * @brief 消息类型，固定为 PARAMETER，即字符 'P'
     */
    uint8_t msg_type =
        static_cast<uint8_t>(
            ULogMessageType::PARAMETER);

    /**
     * @brief key 部分的长度
     *
     * 用于将 key_value_str 拆分成：
     *
     * 前 key_len 个字节：参数类型和参数名称
     * 剩余字节：参数值的二进制数据
     */
    uint8_t key_len;

    /**
     * @brief 参数 key 与二进制 value 的组合缓冲区
     *
     * 例如：
     *
     * ["float launch_accel_threshold"]
     * [32.0f 对应的 4 字节二进制值]
     *
     * 注意：
     *
     * value 部分可能包含 0x00，因此整个数组不能作为普通
     * C 字符串使用，不能对整个区域调用 strlen()。
     *
     * 255 是最大构造容量，不表示每次固定写入 255 字节。
     */
    char key_value_str[255];
};


/**
 * @brief ULog 文件功能标志消息
 *
 * 该消息用于声明当前 ULog 文件使用了哪些兼容或不兼容特性，
 * 以及文件末尾是否存在追加数据区域。
 *
 * 对于当前基础日志实现：
 * - compat_flags 全部置 0；
 * - incompat_flags 全部置 0；
 * - appended_offsets 全部置 0。
 *
 * msg_size 表示消息负载长度，不包含前面的
 * uint16_t msg_size 和 uint8_t msg_type 共 3 字节。
 */
struct ulog_message_flag_bits_s {
	uint16_t msg_size;
	uint8_t msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

	uint8_t compat_flags[8] = {};
	uint8_t incompat_flags[8] = {}; ///< @see ULOG_INCOMPAT_FLAG_*
	uint64_t appended_offsets[3] = {}; ///< file offset(s) for appended data if ULOG_INCOMPAT_FLAG0_DATA_APPENDED_MASK is set
};


/*
 * 恢复进入该头文件之前的结构体对齐设置，
 * 避免影响其他普通 C++ 结构体。
 */
#pragma pack(pop)


/**
 * @brief 编译期协议布局检查
 *
 * 如果编译器没有按照预期进行 1 字节对齐，
 * 或者以后有人修改了字段布局，这些检查会直接导致编译失败。
 */
static_assert(
    sizeof(ulog_file_header_s) == 16U,
    "ULog file header must be exactly 16 bytes");

static_assert(
    sizeof(ulog_message_header_s) == 3U,
    "ULog message header must be exactly 3 bytes");

static_assert(
    sizeof(ulog_message_remove_logged_s) == 5U,
    "ULog remove-logged message must be exactly 5 bytes");

static_assert(
    sizeof(ulog_message_sync_s) == 11U,
    "ULog sync message must be exactly 11 bytes");

static_assert(
    sizeof(ulog_message_data_s) == 5U,
    "ULog data prefix must be exactly 5 bytes");

static_assert(
    sizeof(ulog_message_flag_bits_s) == 43U,
    "ulog_message_flag_bits_s size must be 43 bytes");

} // namespace RocketLog