#ifndef GPIO_APP_H
#define GPIO_APP_H

// ==================== GPIO 应用层封装 ====================
// 开关选择 / 开关释放 / FPGA 复位 / MCU-DAC 触发
//
// 注意: 各函数内部临时 gpio_init + gpio_set_value + gpio_close,
//       使用完即释放 line handle, 不长期占用 GPIO。

// 选择开关通道 value(0~15):
//   0-7 : 拉高 PA11, 地址线 PG10(A0)/PA15(A1)/PA12(A2) 按 value 二进制置位
//   8-15: 拉高 PB0 , 地址线 PC2 (A0)/PC1 (A1)/PC0 (A2) 按 value 二进制置位
//   其它: 不操作, 返回 -1
int select_switch(int value);

// 释放开关选择: 拉低 PA11 和 PB0
int no_select_switch(void);

// FPGA 复位: 操作 PE11(拉低复位 -> 延时 -> 拉高释放)
int fpgareset(void);

// MCU-DAC 触发: 拉高 PD13 后拉低
int mculdac(void);
void init_all_gpios(void);

int disable_all_switch(void) ;
int disable_all_switch_via_cpld(void);   // CPLD 广播关闭: 向 0x00~0x0F 全部 16 个卡地址发 CH=0
int mculdac0();

// [2026-10-16] PD14/PD15 同步脉冲输出(110 = PD14, 111 = PD15)
// 扫描(0x1D)采样完成后由 task.c 调用: 拉高 -> 定时 1 秒 -> 拉低
int pd1415_high(void);
int pd1415_low(void);
#endif // GPIO_APP_H
