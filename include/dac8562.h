#ifndef DAC8562_H
#define DAC8562_H

#include <stdio.h>
#include <stdint.h>
#include "spi.h"

// SPI 总线配置
#define DAC8562_SPI_DEVICE   "/dev/spidev1.0"
#define DAC8562_SPI_MODE     SPI_MODE0
#define DAC8562_SPI_SPEED    1000000

// 初始化序列帧间间隔(us) 与 内部参考稳定时间(ms)
#define DAC8562_STEP_DELAY_US    200
#define DAC8562_REF_SETTLE_MS    10

// A 通道满量程电压(V): 由内部参考(2.5V)与增益决定。
// 增益 x1 → 2.5V; 增益 x2 → 5.0V。请按实际增益设置修改。
#define DAC8562_A_FULL_SCALE_VOLTS   5.0f

// DAC8562 命令字 (命令3bit + 地址3bit + 填充2bit)
#define DAC8562_CMD_WRITE_INPUT_A    0x00
#define DAC8562_CMD_WRITE_INPUT_B    0x01
#define DAC8562_CMD_UPDATE_A         0x18
#define DAC8562_CMD_UPDATE_B         0x19
#define DAC8562_CMD_UPDATE_AB        0x17
#define DAC8562_CMD_UPDATE_ALL       0x0F
#define DAC8562_CMD_POWER_DOWN       0x21
#define DAC8562_CMD_GAIN_SETUP       0x28
#define DAC8562_CMD_RESET            0x28
#define DAC8562_CMD_INTERNAL_REF     0x38

// DAC8562 控制命令 (3字节 SPI 帧)
typedef struct {
    uint8_t data[3];
} DAC8562_CMD_t;

DAC8562_CMD_t Make_DAC8562_CMD(uint8_t cmd_8bit, uint16_t data_16bit);
uint16_t DAC8562_Write(uint8_t *CMD);

// A 通道输出: 直接给 16 位码值
uint16_t DAC8562_Set_Output_A(uint16_t data_16bit);
// A 通道输出: 给电压(超量程会限幅), 基于 DAC8562_A_FULL_SCALE_VOLTS
uint16_t DAC8562_Set_Output_A_Voltage(float volts);

uint16_t DAC8562_Set_Output_B(uint16_t data_16bit);
uint16_t DAC8562_Set_Output(uint16_t data_16bit);
uint16_t DAC8562_Set_A(uint16_t data_16bit);
uint16_t DAC8562_Set_B(uint16_t data_16bit);
uint16_t DAC8562_Output_AB(uint16_t data_16bit);
uint16_t DAC8562_Reset_All(void);
uint16_t DAC8562_Set_Gain(uint16_t gain);
uint16_t DAC8562_CloseB(void);
uint16_t DAC8562_In_Ref(void);
uint16_t DAC8562_Init(void);
void DAC8562_Close(void);

#endif // DAC8562_H
