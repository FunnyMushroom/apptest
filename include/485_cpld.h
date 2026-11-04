#ifndef _485_CPLD_H
#define _485_CPLD_H

#include <stdint.h>

// 打开/关闭 CPLD 串口
int cpld_init(void);
void cpld_deinit(void);

// 快速发送一帧: 0xAA a b 0xED
// 只写入串口发送缓冲即返回, 不等待发送完成、不等待回复
int cpld_send_hex_command_block(uint8_t a, uint8_t b);

// 非阻塞接收 2 个任意字节: 把串口缓冲里已有的数据读完即返回, 不等待
// 返回: 0 = 收到 2 字节; 1 = 只收到 1 字节; -1 = 无数据/未初始化
int cpld_recv_two(uint8_t *buf);

// 发送一帧(0xAA a b 0xED)后阻塞等待应答, 要求收回的 2 字节依次等于 a、b。
// 每次等待超时 1ms: 超时未收到匹配应答则重发, 最多重发 3 次(合计最多发送 4 次)。
// echo: 可传 NULL; 成功时写入收到的 2 字节(echo[0] == a, echo[1] == b)
// 返回: 0 = 成功; -1 = 未初始化/发送失败; -2 = 重发耗尽仍未收到有效应答
int cpld_send_hex_command_wait_reply(uint8_t a, uint8_t b, uint8_t *echo);

// ==================== 频率设置(5 字节帧, 发送后等待回显) ====================
// 发送 0xAA addr ch freq 0xED, 由 0x23(多通道选择)调用。
//   addr: 0~15   开关卡地址(= 开关卡卡号)
//   ch:   1~128  开关卡通道号(= 开关卡通道 + 1, 与 split_num 的 1 基约定一致)
//   freq: 1~256  频率系数, 单位 10kHz(1 = 10kHz, 100 = 1MHz, 256 = 2.56MHz)
//
//   [2026-11-04] FPGA 实测行为: 收到频率帧后会回显 2 字节(地址+通道, 如 0F 01),
//   表示频率设置成功。因此本函数发完后等待回显:
//       单次等待上限 5ms(回显通常 1~2ms 内到达), 不重发;
//       回显 2 字节依次等于 addr、ch 即判定成功。
//   不采用 wait_reply 的"10ms×重发4次": 0x23 在接收线程内同步执行,
//   0x23→0x1D 间隔实测仅 ~13ms, 最坏 40ms+ 的等待会推迟 0x24 回包、撞上 0x1D;
//   5ms 上限下 0x23 总耗时仍 < 7ms, 时序安全。超时只说明"未确认成功",
//   由调用方决定是否容忍(task.c: 频率帧尽力而为, 通道选择照常)。
//   若超时后回显才姗姗到达, 残留字节由下一次 wait_reply 开头的
//   tcflush(TCIFLUSH) 清掉, 不会污染切通道命令的应答判定。
//
// 返回: 0 = 已发出且回显匹配(设置成功); -1 = 未初始化 / 参数越界 / 写失败;
//       -2 = 帧已发出但未收到匹配回显(未确认成功, 频率可能未生效)
#define CPLD_FREQ_MIN      1
#define CPLD_FREQ_MAX      256
#define CPLD_FREQ_TO_BYTE(f)  ((uint8_t)(((f) >= 256) ? 0x00 : (f)))   // 256 -> 0x00
#define CPLD_FREQ_ECHO_TIMEOUT_US  5000                                // 回显等待上限 5ms
int cpld_send_freq_frame(uint8_t addr, uint8_t ch, uint16_t freq);

#endif
