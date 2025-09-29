#include "bsp_asm330.h"

#include "asm330lhh_reg.h"

#define ASM330_CS_Pin GPIO_PIN_4
#define ASM330_CS_GPIO_Port GPIOC

extern SPI_HandleTypeDef hspi1;

stmdev_ctx_t dev_ctx;  // imu设备

static void SPI_CS_Control(uint8_t state) {
  if (state == 0)
    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_SET);
}

static int32_t Asm330_write(void *handle, uint8_t reg, const uint8_t *bufp,
                            uint16_t len) {
  if (bufp == NULL || len == 0) {
    return -1;
  }
  SPI_CS_Control(0);  // 拉低cs
  uint8_t temp = reg;
  temp &= 0x7F;  // 确保最高位为0，表示写操作
  if (HAL_SPI_Transmit(&hspi1, &temp, 1, 1000) != HAL_OK) {
    SPI_CS_Control(1);  // 出错时及时释放片选
    return -2;          // 地址发送失败
  }
  // 发送要写入的数据（长度为len）
  if (HAL_SPI_Transmit(&hspi1, bufp, len, 1000) != HAL_OK) {
    SPI_CS_Control(1);  // 出错时及时释放片选
    return -3;          // 数据发送失败
  }

  SPI_CS_Control(1);  // 拉高cs
  return 0;
}

static int32_t Asm330_read(void *handle, uint8_t reg, uint8_t *bufp,
                           uint16_t len) {
  if (len == 0) {
    return -1;
  }
  SPI_CS_Control(0);  // 拉低cs

  uint8_t temp = reg;
  temp |= 0x80;  // 确保最高位为1，表示读操作
  if (HAL_SPI_Transmit(&hspi1, &temp, 1, 1000) != HAL_OK) {
    SPI_CS_Control(1);  // 出错时及时释放片选
    return -2;          // 地址发送失败
  }

  // 读出数据（长度为len）
  if (HAL_SPI_Receive(&hspi1, bufp, len, 1000) != HAL_OK) {
    SPI_CS_Control(1);
    return -3;
  }
  SPI_CS_Control(1);  // 拉高cs

  return 0;
}

int32_t Asm330_Drive_Init(void) {
  SPI_CS_Control(1);  // CS high
  dev_ctx.write_reg = Asm330_write;
  dev_ctx.read_reg = Asm330_read;
  dev_ctx.mdelay = HAL_Delay;

  return 0;
}
