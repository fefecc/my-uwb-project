#include <stdint.h> // 为了 uint8_t 类型
#include <string.h> // 为了 memcpy 函数
#include <stdio.h>  // 为了错误处理中的 printf (可选)

/**
 * @brief 将 double 类型的值转换为 8 字节的 uint8_t 数组表示。
 *
 * @param d           输入的 double 值。
 * @param bytes_out   指向用于存储结果的 8 字节 uint8_t 数组的指针。
 * @param buffer_size 输出缓冲区的大小 (必须 >= 8)。
 * @return int        成功返回 1，失败返回 0 (例如大小不匹配或缓冲区太小)。
 */
int double_to_bytes(double d, uint8_t *bytes_out, size_t buffer_size)
{
    // 确保 double 是 8 字节，并且输出缓冲区足够大
    if (sizeof(double) != 8 || buffer_size < 8 || bytes_out == NULL) {
        // fprintf(stderr, "Error: double size is not 8 bytes or buffer is too small/NULL.\n");
        return 0; // 失败
    }

    // 使用 memcpy 进行字节拷贝
    memcpy(bytes_out, &d, 8);

    return 1; // 成功
}

/**
 * @brief 将 8 字节的 uint8_t 数组表示转换为 double 类型的值。
 *
 * @param bytes_in    指向包含 8 字节二进制表示的 uint8_t 数组的指针。
 * @param buffer_size 输入缓冲区的大小 (必须 >= 8)。
 * @param success     指向 int 的指针，用于返回转换是否成功 (1 成功, 0 失败)。
 * 如果传入 NULL，则不报告状态。
 * @return double     转换后的 double 值。如果失败，返回 0.0 并设置 success 为 0。
 */
double bytes_to_double(const uint8_t *bytes_in, size_t buffer_size, int *success)
{
    double d = 0.0; // 初始化

    // 确保 double 是 8 字节，并且输入缓冲区有效且足够大
    if (sizeof(double) != 8 || buffer_size < 8 || bytes_in == NULL) {
        // fprintf(stderr, "Error: double size is not 8 bytes or input buffer is too small/NULL.\n");
        if (success != NULL) {
            *success = 0; // 标记失败
        }
        return 0.0; // 返回 0.0 表示错误
    }

    // 使用 memcpy 进行字节拷贝
    memcpy(&d, bytes_in, 8);

    if (success != NULL) {
        *success = 1; // 标记成功
    }
    return d;
}