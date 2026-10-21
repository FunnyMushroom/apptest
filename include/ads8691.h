#ifndef ADS8691_H
#define ADS8691_H

#include <stdint.h>

// ADS8691 参考电压与分辨率
#define ADS8691_VREF_MV     4096        // 内部基准 4.096V
#define ADS8691_RESOLUTION  (1 << 18)   // 262144 (18-bit)

int ADS8691_Init(void);
uint32_t ADS8691_Read(void);
void ADS8691_Close(void);
int ADS8691_Read_Multi(uint32_t *buf, uint8_t count);
void ADS8691_Set_Range_12V(void);
#endif // ADS8691_H