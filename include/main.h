#ifndef MAIN_H
#define MAIN_H

#include "threadpool.h"
#include <stdatomic.h>
#define THREAD_COUNT 10

// 全局线程池(全工程唯一), 由 main() 创建后使用
// 协议解析/任务分发直接使用它, 无需再层层传递 app 结构体

extern threadpool_t *g_thread_pool;
extern atomic_int adc_stop_flag;
extern atomic_int HV_stop_flag;
#endif