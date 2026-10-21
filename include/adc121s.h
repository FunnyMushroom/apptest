#ifndef ADC121S_H
#define ADC121S_H

#include <stdint.h>

// ADC121S101 参考电压与分辨率
#define ADC_VREF_MV     5000
#define ADC_RESOLUTION  4096

int ADC121S_Init(void);
uint16_t ADC121S_Read(void);
void ADC121S_Close(void);
uint16_t ADC121S_Read_Multi(uint16_t *buf, uint8_t count);
#endif // ADC121S_H
