#include "mydw1000timestamp.h"
#include "deca_spi.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "string.h"

// 获取5个字节的时间戳
int16_t my_get_tx_timestamp_u8_5byte(uint8_t *timestamp)
{
    dwt_readfromdeviceFromISR(TX_TIME_ID, 0, RX_TIME_RX_STAMP_LEN,
                              timestamp); // Get the adjusted time of arrival
    return 0;
}

// 获取5个字节的时间戳
int16_t my_get_rx_timestamp_u8_5byte(uint8_t *timestamp)
{
    dwt_readfromdeviceFromISR(RX_TIME_ID, 0, RX_TIME_RX_STAMP_LEN,
                              timestamp); // Get the adjusted time of arrival
    return 0;
}

/**
 * @brief  将一个 5 字节的时间戳数组（小端格式）转换为一个 64 位的无符号整数。
 * @param  timestamp 指向 5 字节源数据缓冲区的指针。
 * @return uint64_t  转换后的 64 位时间戳值。如果输入指针为 NULL，则返回 0。
 */
uint64_t u8_5byte_TO_u64(const uint8_t *timestamp)
{
    uint64_t ts = 0;

    // 1. 参数校验
    if (timestamp == NULL) {
        return 0; // 对于 uint64_t，返回 0 比 -1 更合适地表示错误
    }

    // 2. 核心转换逻辑 (小端字节序解析)
    for (int i = 4; i >= 0; i--) {
        // 将当前结果左移 8 位
        ts <<= 8;
        // 将当前字节合并到结果的最低位
        ts |= timestamp[i];
    }

    return ts;
}

/**
 * @brief  将一个 64 位的无符号整数转换为一个 5 字节的时间戳数组（小端格式）。
 * @param  timestamp    [输入] 64 位源时间戳数据。只会使用其低 40 位。
 * @param  buffer       [输出] 指向 5 字节目标缓冲区的指针。
 * @return int16_t      成功则返回 0，如果目标缓冲区指针为空，则返回 -1 表示错误。
 */
int16_t u64_TO_u8_5byte(uint64_t timestamp, uint8_t *buffer)
{
    // 1. 参数校验
    if (buffer == NULL) {
        return -1; // 目标缓冲区为空，返回错误
    }
    timestamp = timestamp; // 清空后9位，这个直接被硬件忽略了
    // 2. 核心转换逻辑：
    //    通过循环和位移操作，将 64 位整数的低 40 位逐字节“拆解”出来。
    for (int i = 0; i < 5; i++) {
        // 提取当前最低位的字节，并存入数组
        buffer[i] = (uint8_t)(timestamp & 0xFF);
        // 将时间戳右移 8 位，准备提取下一个字节
        timestamp >>= 8;
    }

    // 3. 返回成功
    return 0;
}

// 直接获取一个64位时间戳
uint64_t get_rx_timestamp_u64_fromisr(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    my_get_rx_timestamp_u8_5byte(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

uint64_t get_tx_timestamp_u64_fromisr(void)
{
    uint8 ts_tab[5];
    uint64_t ts = 0;
    int i;
    my_get_tx_timestamp_u8_5byte(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

int16_t mymemcopytimestamp(uint8_t *dest, volatile uint8_t *source)
{
    // 1. 参数校验：检查源和目标指针是否有效
    if (dest == NULL || source == NULL) {
        return -1; // 如果任一指针为空，则返回错误
    }

    // 2. 核心操作：使用 memcpy 复制 5 个字节
    //    memcpy 是最高效的内存块复制方式

    for (uint8_t i = 0; i < 5; i++) {
        dest[i] = source[i];
    }

    // 3. 返回成功
    return 0;
}

/**
 * @brief  将 UWB 微秒 (UUS, 约 1.026 us) 转换为 DW1000 内部时间戳单位的转换系数。
 * @note   1 UUS = 512 * (1 / 499.2 MHz) ≈ 1.0256 µs
 * 1 DWT Time Unit = 1 / (499.2 MHz * 128) ≈ 15.65 ps
 * 转换系数 = (1 UUS) / (1 DWT Time Unit) = 512 * 128 = 65536
 */

// 定义 DWT 时间单位的时长（秒）
#define MYDWT_TIME_UNITS (1.0 / (499.2 * 1000000.0 * 128.0))

// 将毫秒转换为 DWT 时间单位的宏
#define MS_TO_DWT_TIME (1.0 / (MYDWT_TIME_UNITS * 1000.0))

uint64_t transmitDelayTime(uint64_t last_tx_timestamp, uint16_t DELAY_MS)
{
    uint64_t future_tx_timestamp_64;
    future_tx_timestamp_64 = last_tx_timestamp + (uint64_t)(DELAY_MS * MS_TO_DWT_TIME);

    return future_tx_timestamp_64;
}

uint64_t get_timestamp_difference_u64(uint64_t end_ts, uint64_t start_ts)
{
    // 定义一个 40 位的掩码，用于将计算结果限制在 40 位的范围内。
    // 1ULL << 40 表示 2 的 40 次方。
    const uint64_t TS_40BIT_MASK = (1ULL << 40) - 1;

    // 核心计算：
    // 1. (end_ts - start_ts): 使用标准的无符号整数减法。
    //    如果 end_ts < start_ts (发生了回绕), 结果会自动产生一个 "借位"，
    //    在无符号数的世界里，这等效于 (end_ts + 2^64 - start_ts)，
    //    其低 40 位的结果是正确的。
    // 2. & TS_40BIT_MASK: 使用按位与操作，确保只保留结果的低 40 位，
    //    清除掉因 64 位运算可能产生的高位数据，得到纯净的 40 位差值。
    return (end_ts - start_ts) & TS_40BIT_MASK;
}
