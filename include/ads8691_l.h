#ifndef ADS8691_L_H
#define ADS8691_L_H

#include <stdint.h>

// ==================== ADS8691 (_l 版, SPI2.0) ====================
// 与 ads8691.h 的区别: 本模块使用独立 SPI 设备/spi2.0, 函数带 _l 后缀,
// 初始化时固定写入 RANGE_SEL=0000 → ±3*Vref = ±12.288V 双极性量程。

// 参考电压与分辨率
#define ADS8691_L_VREF_MV       4096        // 内部基准 4.096V
#define ADS8691_L_RESOLUTION    (1 << 18)   // 262144 (18-bit)

// 满量程与 LSB (量程 ±3*Vref = ±12.288V, 18 位)
#define ADS8691_L_FULL_SCALE_MV 12288       // 满量程 12.288V
#define ADS8691_L_LSB_NV        93750       // 1 LSB ≈ 93.75uV

// 读取失败时的返回值 (ADS8691_Read_l 返回此值表示 SPI 未初始化或传输失败)
#define ADS8691_L_ERR_VAL       0xFFFFFFFFu

// 初始化: 打开 SPI 设备并写入 ±12.288V 量程
// 返回: 0 成功, -1 失败
int ADS8691_Init_l(void);

// 读取一次转换结果 (18 位有效, 0 ~ 262143)
// 返回: 转换值; 失败时返回 ADS8691_L_ERR_VAL
uint32_t ADS8691_Read_l(void);

#endif // ADS8691_L_H
