#ifndef REALTIME_H
#define REALTIME_H

#include <stdint.h>

// ==================== U/I (ADS1115) 实时采集(生产者线程) ====================
int  realtime_vi_start(void);
void realtime_vi_stop(void);
// 从 U/I 队列取一组数据; timeout_ms 内无数据返回0(超时), 成功返回1并写入 *u *i
int  realtime_vi_recv(uint16_t *u, uint16_t *i, int timeout_ms);

#endif // REALTIME_H
