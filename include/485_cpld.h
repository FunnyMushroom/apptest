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

#endif