#ifndef _485_CPLD_H
#define _485_CPLD_H

#include <stdint.h>

// [2026-11-04] 已与 app923 工程的多通道选择实现对齐: CPLD 帧统一为 5 字节
//   发: AA a b freq ED (a=开关卡地址 0~15, b=通道 1~128, freq=频率系数, 单位 10kHz)
//   收: a b (2 字节, 只回显地址和通道, 频率不回)
//
//   频率合法域 1~200: 0xED(237) 是帧尾, 送 0 或 >200 会被 CPLD 判为坏帧,
//   整帧丢弃且不回包 -> 板子重发耗尽 -> 该通道切不过去。故调用方必须校验。
//   注意: CPLD 只有一个频率寄存器且同一时刻只输出一路,
//         "每路独立频率"是靠"选通道的同一帧里带上该路频率"实现的。
#define CPLD_FRAME_LEN        5

#define CPLD_FREQ_MIN         1
#define CPLD_FREQ_MAX         200

// 未显式指定频率时使用的默认参数: 100 -> 1MHz (与 CPLD 上电默认 FREQ_DEFAULT 一致)
#define CPLD_DEFAULT_FREQ     100

// 打开/关闭 CPLD 串口
int cpld_init(void);
void cpld_deinit(void);

// 快速发送一帧: 0xAA a b freq 0xED
// 只写入串口发送缓冲即返回, 不等待发送完成、不等待回复
int cpld_send_hex_command_block(uint8_t a, uint8_t b, uint8_t freq);

// 非阻塞接收 2 个任意字节: 把串口缓冲里已有的数据读完即返回, 不等待
// 返回: 0 = 收到 2 字节; 1 = 只收到 1 字节; -1 = 无数据/未初始化
int cpld_recv_two(uint8_t *buf);

// 发送一帧(0xAA a b freq 0xED)后阻塞等待应答, 要求收回的 2 字节依次等于 a、b。
// 每次等待超时 10ms: 超时未收到匹配应答则重发, 最多重发 3 次(合计最多发送 4 次)。
// echo: 可传 NULL; 成功时写入收到的 2 字节(echo[0] == a, echo[1] == b)
// 返回: 0 = 成功; -1 = 未初始化/发送失败; -2 = 重发耗尽仍未收到有效应答
int cpld_send_hex_command_wait_reply(uint8_t a, uint8_t b, uint8_t freq, uint8_t *echo);

#endif
