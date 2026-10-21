#ifndef ADS1115_H
#define ADS1115_H

#include <stdio.h>
#include <stdint.h>

// I2C 总线与从机地址
#define ADS1115_I2C_DEVICE      "/dev/i2c-0"
#define ADS1115_ADDR            0x48

// 寄存器地址
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

// 配置寄存器: MUX 输入选择 (bit14:12)
#define ADS1115_MUX_AIN0_AIN1   0x0000
#define ADS1115_MUX_AIN0_AIN3   0x1000
#define ADS1115_MUX_AIN1_AIN3   0x2000
#define ADS1115_MUX_AIN2_AIN3   0x3000
#define ADS1115_MUX_AIN0_GND    0x4000
#define ADS1115_MUX_AIN1_GND    0x5000
#define ADS1115_MUX_AIN2_GND    0x6000
#define ADS1115_MUX_AIN3_GND    0x7000

// 配置寄存器: PGA 增益 (bit11:9)
#define ADS1115_PGA_6_144V      0x0000
#define ADS1115_PGA_4_096V      0x0200
#define ADS1115_PGA_2_048V      0x0400
#define ADS1115_PGA_1_024V      0x0600
#define ADS1115_PGA_0_512V      0x0800
#define ADS1115_PGA_0_256V      0x0A00

// 配置寄存器: 数据速率 (bit7:5)
#define ADS1115_RATE_8SPS       0x0000
#define ADS1115_RATE_16SPS      0x0020
#define ADS1115_RATE_32SPS      0x0040
#define ADS1115_RATE_64SPS      0x0060
#define ADS1115_RATE_128SPS     0x0080
#define ADS1115_RATE_250SPS     0x00A0
#define ADS1115_RATE_475SPS     0x00C0
#define ADS1115_RATE_860SPS     0x00E0

// 配置寄存器: 其他位
#define ADS1115_OS_START        0x8000
#define ADS1115_MODE_SINGLE     0x0100
#define ADS1115_MODE_CONT       0x0000
#define ADS1115_COMP_DISABLE    0x0003

// 默认采样率与转换等待上限
#define ADS1115_RATE_DEFAULT    ADS1115_RATE_128SPS
#define ADS1115_CONV_TIMEOUT_MS 100

// 通道定义: 通道0 = I (AIN0), 通道1 = U (AIN1)
#define ADS1115_CH_I_MUX        ADS1115_MUX_AIN0_GND   // 通道0 → I
#define ADS1115_CH_U_MUX        ADS1115_MUX_AIN1_GND   // 通道1 → U

// ---- 基础读写 ----
void ADS1115_Write_Config(uint16_t cmd);
void ADS1115_Read_Conversion(uint16_t *data);

// ---- 初始化/关闭 ----
int  ADS1115_Init(void);
void ADS1115_Close(void);

// ---- 通道读取 ----
// 读任意通道(自动加锁); 返回 0 成功, -1 失败(此时 *value = 0xFFFF)
int      ADS1115_Read_Channel(uint16_t mux, uint16_t pga, uint16_t *value);
// 通道0 = I
uint16_t ADS1115_Read_ADC_I(void);
// 通道1 = U
uint16_t ADS1115_Read_ADC_U(void);
// 一次持锁内读两个通道: *I = 通道0, *U = 通道1
void     ADS1115_Read_ADC(uint16_t *I, uint16_t *U);
// 一次持锁内读全部四个通道(AIN0~AIN3 对 GND); values 至少 4 个元素, 失败通道填 0xFFFF;
// 返回成功读取的通道数(0~4), 参数或 I2C 失败返回 -1
int      ADS1115_Read_All_Channels(uint16_t *values, uint16_t pga);
int32_t ads1115_uv(int16_t raw);
int ADS1115_Read_ADC_Physical(int32_t *current_ua, int32_t *output_mv);
#endif // ADS1115_H
