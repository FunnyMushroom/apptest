#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L   // 启用 clock_gettime / CLOCK_REALTIME (须在系统头文件之前)
#endif
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include "realtime.h"
#include "ads1115.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_ADC121
#include "log.h"

// ==================== 通用有界环形队列(线程安全) ====================
#define RT_QUEUE_CAP 64   // 队列容量

// U/I 队列: 元素为一组 {U, I}
typedef struct {
    uint16_t u, i;
} vi_sample_t;

typedef struct {
    vi_sample_t buf[RT_QUEUE_CAP];
    int      head, tail, count;
    pthread_mutex_t mtx;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
} ring_vi_t;

static void ring_vi_init(ring_vi_t *q) {
    memset(q, 0, sizeof(*q));
    pthread_mutex_init(&q->mtx, NULL);
    pthread_cond_init(&q->not_empty, NULL);
    pthread_cond_init(&q->not_full, NULL);
}

static void ring_vi_push(ring_vi_t *q, vi_sample_t *s) {
    pthread_mutex_lock(&q->mtx);
    while (q->count == RT_QUEUE_CAP) {
        pthread_cond_wait(&q->not_full, &q->mtx);
    }
    q->buf[q->tail] = *s;
    q->tail = (q->tail + 1) % RT_QUEUE_CAP;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mtx);
}

static int ring_vi_pop_timeout(ring_vi_t *q, vi_sample_t *out, int timeout_ms) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }

    pthread_mutex_lock(&q->mtx);
    while (q->count == 0) {
        int rc = pthread_cond_timedwait(&q->not_empty, &q->mtx, &ts);
        if (rc == ETIMEDOUT) {
            pthread_mutex_unlock(&q->mtx);
            return 0;
        }
    }
    *out = q->buf[q->head];
    q->head = (q->head + 1) % RT_QUEUE_CAP;
    q->count--;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->mtx);
    return 1;
}

// ==================== 生产线程控制(引用计数, 加锁保护) ====================
static pthread_mutex_t g_ctrl = PTHREAD_MUTEX_INITIALIZER;

static int g_vi_run = 0;
static int g_vi_refs = 0;
static pthread_t g_vi_tid;
static ring_vi_t g_vi_q;

static pthread_once_t g_once = PTHREAD_ONCE_INIT;
static void realtime_init_once(void) {
    ring_vi_init(&g_vi_q);
}

// 毫秒级休眠(用 nanosleep 替代 usleep, 兼容 POSIX 特性宏)
static void rt_sleep_ms(long ms) {
    struct timespec req;
    req.tv_sec  = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&req, NULL);
}

// ---------- U/I 生产者: 周期采样 ADS1115 并入队 ----------
static void *vi_producer_thread(void *arg) {
    (void)arg;
    printf("[RT_VI] ▶ U/I 采集线程已启动, 采样周期 100ms\n");
    while (g_vi_run) {
        vi_sample_t s;
        s.u = ADS1115_Read_ADC_U();        // 采集 U
        s.i = ADS1115_Read_ADC_I();        // 采集 I
        ring_vi_push(&g_vi_q, &s);         // 放入队列
        rt_sleep_ms(100);                  // 100ms 采样周期
    }
    printf("[RT_VI] U/I 采集线程已退出\n");
    return NULL;
}

int realtime_vi_start(void) {
    pthread_once(&g_once, realtime_init_once);
    pthread_mutex_lock(&g_ctrl);
    if (g_vi_refs++ == 0) {
        printf("[RT_VI] ▶ 启动 U/I 采集: 引用计数 0 -> 1, 创建采集线程\n");
        g_vi_run = 1;
        if (pthread_create(&g_vi_tid, NULL, vi_producer_thread, NULL) != 0) {
            g_vi_refs--;
            g_vi_run = 0;
            pthread_mutex_unlock(&g_ctrl);
            printf("[RT_VI] ❌ 启动失败: 创建采集线程出错\n");
            return -1;
        }
    } else {
        printf("[RT_VI]   已启动, 引用计数递增到 %d\n", g_vi_refs);
    }
    pthread_mutex_unlock(&g_ctrl);
    return 0;
}

void realtime_vi_stop(void) {
    pthread_mutex_lock(&g_ctrl);
    if (g_vi_refs > 0 && --g_vi_refs == 0) {
        g_vi_run = 0;
        pthread_mutex_unlock(&g_ctrl);
        pthread_join(g_vi_tid, NULL);
        printf("[RT_VI] ✅ 停止完成: U/I 采集线程已回收\n");
        return;
    }
    printf("[RT_VI]   停止请求已记录, 剩余引用计数 %d\n", g_vi_refs);
    pthread_mutex_unlock(&g_ctrl);
}

int realtime_vi_recv(uint16_t *u, uint16_t *i, int timeout_ms) {
    vi_sample_t s;

    if (u == NULL || i == NULL) {
        printf("[RT_VI] ❌ 取数失败: 输出指针为 NULL(u=%p, i=%p)\n",
               (void *)u, (void *)i);
        return 0;
    }

    if (ring_vi_pop_timeout(&g_vi_q, &s, timeout_ms) == 0) {
        return 0;
    }
    *u = s.u;
    *i = s.i;
    return 1;
}
