#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "protocol.h"
#include <main.h>
#include "task.h"
#include "tcpserver.h"
#include "uart485.h"
#include "ota.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_PROTO
#include "log.h"

// ==================== 命令单飞(去重)控制 ====================
// 同一操作码在未执行完之前, 不允许再次投递新任务, 防止相同命令并发执行
// (同时覆盖 TCP 与 485 两条通道同时到达同一命令的场景)
#define OPCODE_MAX 0x100
// #define OPCODE_ADC_PRINT 0x1D   // ADC 现场读取: 内部有 detach 发送线程, 释放时机特殊

static pthread_mutex_t g_exec_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t         g_opcode_busy[OPCODE_MAX];
static uint64_t        g_opcode_busy_since[OPCODE_MAX];   // 本次占用开始时刻(ms)

// 单调时钟(ms): 只用于算时间差, 不受系统改时间影响
static uint64_t proto_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

// 尝试占用操作码: 返回1=成功占用, 0=该操作码正在执行中
static int opcode_try_acquire(uint8_t opcode) {
    int ret;
    pthread_mutex_lock(&g_exec_lock);
    ret = (g_opcode_busy[opcode] == 0);
    if (ret) {
        g_opcode_busy[opcode] = 1;
        g_opcode_busy_since[opcode] = proto_now_ms();
    }
    pthread_mutex_unlock(&g_exec_lock);
    return ret;
}

// 读取某操作码"本次占用"的开始时刻: 用于判断缓存应答是否属于当前这次执行
static uint64_t opcode_busy_since(uint8_t opcode) {
    uint64_t v;
    pthread_mutex_lock(&g_exec_lock);
    v = g_opcode_busy_since[opcode];
    pthread_mutex_unlock(&g_exec_lock);
    return v;
}

// 释放操作码占用(由任务真正执行完毕后调用)
int opcode_release(uint8_t opcode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          {
    pthread_mutex_lock(&g_exec_lock);
    g_opcode_busy[opcode] = 0;
    pthread_mutex_unlock(&g_exec_lock);
    return 0;
}

// 任务包装: 执行完成后自动释放操作码占用标记
// ⚠️ 释放时机 = 任务函数 return 的那一刻。因此任务函数内部在"发完回包之后"
//    不允许再有任何阻塞动作(usleep / 等锁 / 等 IO): 那段时间的操作码仍处于
//    "忙"态, 上位机紧接着发同一条命令就会被判重复指令丢弃(回 0xF1)。
//    [2026-10-23] 0x1D 扫描收尾的 PD14/PD15 1 秒脉冲原先写在任务函数里,
//    导致每次扫描后 0x1D 多忙 1 秒, 连续扫多通道必吃 0xF1/0x1D00;
//    现已改为 task.c 里的 pd1415_pulse_async() 异步线程, 此处无需再特殊处理。
// 注意: w->arg(task_arg) 由任务函数自己释放, 包装器只释放包装结构本身
typedef struct {
    uint8_t opcode;
    void (*function)(void *);
    void *arg;
} exec_wrap_t;

static void exec_wrapper(void *p) {
    exec_wrap_t *w = (exec_wrap_t *)p;
    uint8_t op = w->opcode;
    w->function(w->arg);
    opcode_release(op);
    free(w);
}

// ==================== 重发请求: 原应答回放 ====================
// 现网失效链: 上位机 3s 没等到应答 -> 重发同一条命令。
//   旧行为: 下位机判"重复指令", 回 0xF1;
//           而上位机等的是 0x24/0x0E/0x1C, 于是这条命令**永远**拿不到应答 -> 步骤必失败。
//   新行为: 若上一次执行的应答确实已经发出(只是上位机没收到), 且本次重发的数据与原请求
//           逐字节相同, 就直接把那份应答再发一遍, 让重试真正生效。
// 三个限制保证不会误回放:
//   1) 只对 485 通道做(TCP 是字节流, 不会凭空丢包, 不需要);
//   2) 缓存的应答必须属于"当前这次占用"(ts >= busy_since), 即上一次执行的成果,
//      不能是几分钟前某次同名命令的应答;
//   3) 太新(<20ms)不回放: 那种情况第一份应答还在路上, 再发一份会造成重复应答。
// 窗口取 5s: 上位机超时 3s 重发, 留出余量; 同时"同一命令隔很久又被执行"的情况
// 由 "ts >= busy_since" 挡住, 不会被误回放成旧答案。
#define DUP_CACHE_MAX_AGE_MS   5000   // 缓存超过这个年龄不再回放
#define DUP_CACHE_MIN_AGE_MS     20   // 缓存太新(第一份应答可能还在路上)不回放
#define DUP_CACHE_FP_MAX          8   // 数据指纹长度上限
#define DUP_CACHE_RESP_MAX       64   // 可缓存的回包长度上限

typedef struct {
    uint8_t  valid;
    uint8_t  fp_len;
    uint8_t  fp[DUP_CACHE_FP_MAX];
    uint16_t resp_len;
    uint8_t  resp[DUP_CACHE_RESP_MAX];
    uint64_t ts_ms;
} dup_cache_t;

static pthread_mutex_t g_dup_lock = PTHREAD_MUTEX_INITIALIZER;
static dup_cache_t     g_dup_cache[OPCODE_MAX];

// 流式数据命令(0x1D 扫描)的应答是一帧帧的 0x1E, 缓存/回放毫无意义, 必须排除
static int dup_cache_opcode_ok(uint8_t opcode) {
    return (opcode != 0x1D && opcode != 0xFF);
}

// 记录一份已发出的回包(供重发请求回放)。仅 485 通道 + 短回包 + 非错误码。
static void dup_cache_store(const command_t *command, const uint8_t *buf, int len) {
    if (command == NULL || buf == NULL) return;
    if (command->channel != RESPONSE_CHANNEL_RS485) return;
    if (!dup_cache_opcode_ok(command->opcode)) return;
    if (len < (int)PROTOCOL_DATA_OFFSET || len > DUP_CACHE_RESP_MAX) return;
    if (command->req_fp_len == 0 || command->req_fp_len > DUP_CACHE_FP_MAX) return;

    uint8_t resp_opcode = buf[PROTOCOL_HEADER_SIZE - 1];   // 回包操作码(帧内偏移 6)
    if (resp_opcode == 0xF1 || resp_opcode == 0xF2 || resp_opcode == 0xF3) {
        return;   // 错误码应答不是"答案", 不能拿去回放
    }

    pthread_mutex_lock(&g_dup_lock);
    dup_cache_t *c = &g_dup_cache[command->opcode];
    c->valid    = 1;
    c->fp_len   = command->req_fp_len;
    memcpy(c->fp, command->req_fp, command->req_fp_len);
    c->resp_len = (uint16_t)len;
    memcpy(c->resp, buf, (size_t)len);
    c->ts_ms    = proto_now_ms();
    pthread_mutex_unlock(&g_dup_lock);
}

// 重发请求能否用缓存应答答复: 返回1=已回放, 0=不能用缓存(调用方仍按重复指令处理)
static int dup_cache_replay(const command_t *command, uint64_t busy_since_ms) {
    uint8_t buf[DUP_CACHE_RESP_MAX];
    int len = 0;

    if (command == NULL || !dup_cache_opcode_ok(command->opcode)) return 0;
    if (command->req_fp_len == 0 || command->req_fp_len > DUP_CACHE_FP_MAX) return 0;

    pthread_mutex_lock(&g_dup_lock);
    dup_cache_t *c = &g_dup_cache[command->opcode];
    uint64_t now = proto_now_ms();
    if (c->valid &&
        c->fp_len == command->req_fp_len &&
        memcmp(c->fp, command->req_fp, command->req_fp_len) == 0 &&
        c->ts_ms >= busy_since_ms &&
        now >= c->ts_ms + DUP_CACHE_MIN_AGE_MS &&
        now - c->ts_ms <= DUP_CACHE_MAX_AGE_MS) {
        len = (int)c->resp_len;
        memcpy(buf, c->resp, (size_t)len);
    }
    pthread_mutex_unlock(&g_dup_lock);

    if (len <= 0) return 0;
    if (app_send_response(command, buf, len) <= 0) return 0;
    printf("[EXEC] 🔁 重发请求: 0x%02X 的原应答(%d 字节)已回放, 不再回 0xF1\n",
           command->opcode, len);
    return 1;
}

// ==================== 协议解析与命令分发 ====================
// 帧格式: AB CD | data_len(4,大端) | opcode(1) | checksum(2,大端) | data
// 校验码 = 整包(不含校验码自身) 64位累加 & 0xFFFF

// 整包校验和: 对(包头+长度+操作码+数据)求和取低16位, 不含校验码自身
static uint16_t compute_checksum(const uint8_t *buf, uint32_t total_len) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < total_len; i++) {
        if (i == PROTOCOL_HEADER_SIZE || i == PROTOCOL_HEADER_SIZE + 1) {
            continue;   // 跳过校验码字段自身
        }
        sum += buf[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

static void *duplicate_task_data(const uint8_t *data, uint32_t data_len) {
    if (data == NULL || data_len == 0) {
        printf("[PARSE] ⚠️ 复制任务数据跳过: data=%p, len=%u\n",
               (const void *)data, (unsigned)data_len);
        return NULL;
    }
    uint8_t *copy = (uint8_t *)malloc(data_len);
    if (copy == NULL) {
        printf("[PARSE] ❌ 复制任务数据失败: 分配 %u 字节内存出错\n", (unsigned)data_len);
        return NULL;
    }
    memcpy(copy, data, data_len);
    return copy;
}



#define OPCODE_BUSY_REPLY      0xF1
#define OPCODE_ERR_CHECKSUM    0xF2
#define OPCODE_ERR_LENGTH      0xF3

void send_error_reply(const command_t *command, uint8_t reply_opcode, uint8_t err_opcode) {
    if (command == NULL) {
        printf("[REPLY] ⚠️ 发送错误回包跳过: command 为 NULL\n");
        return;
    }
    
    uint8_t resp[64];
    uint32_t idx = 0;

    resp[idx++] = 0xAB;
    resp[idx++] = 0xCD;
    resp[idx++] = 0x00;
    resp[idx++] = 0x00;
    resp[idx++] = 0x00;
    resp[idx++] = 0x02;              // data_len = 2
    resp[idx++] = reply_opcode;
    resp[idx++] = 0x00;              // checksum 占位
    resp[idx++] = 0x00;
    resp[idx++] = err_opcode;        // 高8位: 命令码
    resp[idx++] = 0x00;              // 低8位: 0

    uint32_t total_len = idx;
    uint16_t cs = compute_checksum(resp, total_len);
    resp[PROTOCOL_HEADER_SIZE]     = (cs >> 8) & 0xFF;
    resp[PROTOCOL_HEADER_SIZE + 1] = cs & 0xFF;

    app_send_response(command, resp, (int)total_len);
    printf("[REPLY] ❌ 已回错误包: reply_opcode=0x%02X, 出错命令=0x%02X, 共 %u 字节\n",
           reply_opcode, command->opcode, (unsigned)total_len);
}



static int dispatch_task(threadpool_t *pool, void (*task_function)(void *),
                         const command_t *command) {
    if (pool == NULL || command == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 线程池或命令为 NULL\n");
        return -1;
    }
    command_t *task_arg = (command_t *)calloc(1, sizeof(*task_arg));
    if (task_arg == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 分配 task_arg 内存出错\n");
        return -1;
    }
    task_arg->is_valid    = command->is_valid;
    task_arg->opcode      = command->opcode;
    task_arg->data_len    = command->data_len;
    task_arg->channel     = command->channel;
    task_arg->tcp_fd      = command->tcp_fd;
    task_arg->serial_port = command->serial_port;
    task_arg->data        = NULL;
    if (command->data_len > 0) {
        task_arg->data = duplicate_task_data(command->data, command->data_len);
        if (task_arg->data == NULL) {
            printf("[EXEC] ❌ 分发任务失败: 复制命令负载数据出错 (0x%02X)\n", command->opcode);
            free(task_arg);
            return -1;
        }
    }

    // 请求指纹快照: 必须在分发时取。任务函数往往在回包前就 free(task_arg->data),
    // 等到 app_send_response 时已经没有原始数据可比对了。
    task_arg->req_fp_len = 0;
    if (command->data != NULL && command->data_len > 0) {
        uint32_t n = command->data_len > sizeof(task_arg->req_fp)
                     ? (uint32_t)sizeof(task_arg->req_fp) : command->data_len;
        memcpy(task_arg->req_fp, command->data, n);
        task_arg->req_fp_len = (uint8_t)n;
    }

    // 单飞去重: 同一操作码正在执行时, 不再新开线程执行
    if (opcode_try_acquire(task_arg->opcode) == 0) {
        // 优先尝试回放上次的应答: 上位机"超时后重发"是最常见的重复来源,
        // 直接回 0xF1 会让它等的那条应答永远不到(实锤导致标定步骤必然失败)。
        if (dup_cache_replay(task_arg, opcode_busy_since(task_arg->opcode))) {
            free(task_arg->data);
            free(task_arg);
            return -3;   // -3 表示重发请求已用缓存应答答复
        }
        printf("[EXEC] ⚠️ 0x%02X 正在执行中, 重复指令已丢弃\n", task_arg->opcode);
        send_error_reply(command, 0xF1, task_arg->opcode);
        free(task_arg->data);
        free(task_arg);
        return -2;   // -2 表示重复指令已被丢弃(非错误)
    }

    // 用包装器投递: 任务结束后自动释放占用标记
    exec_wrap_t *wrap = (exec_wrap_t *)calloc(1, sizeof(*wrap));
    if (wrap == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 分配 exec_wrap 内存出错 (0x%02X)\n",
               task_arg->opcode);
        opcode_release(task_arg->opcode);
        free(task_arg->data);
        free(task_arg);
        return -1;
    }
    wrap->opcode   = task_arg->opcode;
    wrap->function = task_function;
    wrap->arg      = task_arg;

    if (threadpool_add(pool, exec_wrapper, wrap) != 0) {
        printf("[EXEC] ❌ 分发任务失败: 投标到线程池出错 (0x%02X)\n", task_arg->opcode);
        opcode_release(task_arg->opcode);
        free(wrap);
        free(task_arg->data);
        free(task_arg);
        return -1;
    }
    return 0;
}



static int dispatch_command_if_len_ok(threadpool_t *pool, const command_t *command,
                                      uint32_t expected_len, void (*task_function)(void *),
                                      const char *command_name) {
    if (command->data_len != expected_len) {
        // 打印：操作码对应的命令负载长度不正确，显示实际长度和期望长度
        // 例如：[EXEC] ❌ 0x01 (detect_collect): 负载长度 3 不合法, 期望 2
        printf("[EXEC] ❌ 0x%02X (%s) 参数长度不合法: 实际 %u 字节, 期望 %u 字节\n",
               command->opcode, command_name, (unsigned)command->data_len,
               (unsigned)expected_len);
        send_error_reply(command,0xf3,command->opcode);
        return -1;
    }
    int rc = dispatch_task(pool, task_function, command);
    if (rc == -2 || rc == -3) {
        // -2: 该操作码正在执行中, 重复指令已丢弃
        // -3: 该操作码正在执行中, 但重发请求已用上次应答回放
        // 两种情况都已自行回包(或有意不回), 不算错误
        return 0;
    }
    if (rc != 0) {
        printf("[EXEC] ❌ 0x%02X (%s) 分发到线程池失败\n",
               command->opcode, command_name);
        return -1;
    }
    // 打印：命令成功分发到线程池，显示操作码和命令名称
    printf("[EXEC] ✅ 0x%02X (%s) 已分发到线程池\n", command->opcode, command_name);
    return 0;
}

static int execute_command(const command_t *command) {
    if (g_thread_pool == NULL || command == NULL) {
        printf("[EXEC] ❌ 执行命令失败: 线程池或命令为 NULL\n");
        return -1;
    }
    if (!command->is_valid) {
        printf("[EXEC] ❌ 执行命令失败: 命令无效 (is_valid=0)\n");
        return -1;
    }

    switch (command->opcode) {
        // ---------- 1. 板卡检测 ----------
        case 0x01:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Detect_Collection_Card, "detect_collect") != 0)
                return -1;
            break;

        case 0x03:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Detect_Switch_Card, "detect_switch") != 0)
                return -1;
            break;

        // ---------- 2. 设置高压 ----------
        case 0x05:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Set_HV, "set_hv") != 0)
                return -1;
            break;

        // ---------- 3. 读取高压 ----------
        case 0x07:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_Read_HV, "read_hv") != 0)
                return -1;
            break;

        // ---------- 4. 设置操作延时/次数 ----------
        case 0x09:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Control_Times_HV, "set_delay") != 0)
                return -1;
            break;

        // ---------- 5. 自检 ----------
        case 0x0B:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Check_Self, "self_test") != 0)
                return -1;
            break;

        // ---------- 6. 增益 ----------
        case 0x0D:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_Set_Buff, "set_gain") != 0)
                return -1;
            break;

        // ---------- 7. 复位 ----------
        case 0x0F:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Reset_Fpga, "reset_fpga") != 0)
                return -1;
            break;

        // ---------- 8. 使能/开始 ----------
        case 0x11:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Test_En, "enable_start") != 0)
                return -1;
            break;

        // ---------- 9. 停止 ----------
        case 0x13:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Test_Stop, "stop_test") != 0)
                return -1;
            break;

        // ---------- 测试/调试命令 ----------
        // case 0x15:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Select_Collect_Card, "sel_collect") != 0)
        //         return -1;
        //     break;

        // case 0x17:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Select_Switch_Card, "sel_switch") != 0)
        //         return -1;
        //     break;

        // case 0x19:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Select_Collect_Card_Channel, "sel_channel") != 0)
        //         return -1;
        //     break;

        case 0x1B:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Test_Read_ADC121_Times, "set_adc_times") != 0)
                return -1;
            break;

        // case 0x1D:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Read_ADC121_Print, "print_adc") != 0)
        //         return -1;
        //     break;
            case 0x1D:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_ADC_Auto_Scan, "auto_scan") != 0)
                return -1;
            break;


        case 0x1F:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_Read_HV_I, "read_hv_i") != 0)
                return -1;
            break;
        case 0x21:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
            Task_Adc_Stop, "stop") != 0)
                return -1;
            break;
        case 0x23:
        if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
            Task_Set_Scan_Channels, "scan") != 0)
                return -1;
            break;
        case 0x25:
        if (dispatch_command_if_len_ok(g_thread_pool, command, 8,
            Task_Disable_All_Switch, "disable_switch") != 0)
                return -1;
            break;

        // ---------- OTA 在线升级(单可执行文件 + 回滚) ----------
        // 命令数据(9字节): [0..3]=新程序字节数(大端,0=不校验) [4..7]=CRC32(大端,0=不校验) [8]=模式(0=安装 1=只校验 2=回滚)
        // 任务在专用端口等待 TCP 文件, 校验后备份/替换 /usr/bin/program_arm
        case 0x55:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 9,
                    Task_OTA_Upgrade, "ota_upgrade") != 0)
                return -1;
            break;

        // ---------- 打印开关(分模块, 每文件独立开关) ----------
        // 命令数据(2字节大端): 16 位掩码, 每一位对应一个(组).c 文件的打印开关
        //   bit = 1 -> 打开该模块打印; bit = 0 -> 关闭
        //   例: 掩码 0xFFFF = 全部打开; 0xFFFB = 关闭 bit2(SPI)
        //   位号与文件的对应关系见 include/log.h 的 LOG_MOD_xxx
        // 回包: opcode 0x67, 数据 2 字节 = 已生效的掩码
        case 0x66:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Set_Print_Mask, "set_print_mask") != 0)
                return -1;
            break;

        // ---------- 实时采集打印任务(队列通信) ----------
        // 0x21: ADC121S 实时采集并打印, 200ms 超时退出
        // case 0x21:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
        //             Task_Realtime_ADC_Print, "rt_adc_print") != 0)
        //         return -1;
        //     break;

        // // 0x23: U/I (ADS1115) 实时采集并打印, 200ms 超时退出
        // case 0x23:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
        //             Task_Realtime_VI_Print, "rt_vi_print") != 0)
        //         return -1;
        //     break;

        // ---------- 测试(调试)命令 ----------
        case 0xFC:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_SETBUFF_CESHI, "set_buff_test") != 0)
                return -1;
            break;

        case 0xFD:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_CESHI2, "switch_test") != 0)
                return -1;
            break;

        case 0xFE:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_CESHI, "sel_card_test") != 0)
                return -1;
            break;

        case 0xFF:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_ADC_Auto_Scan, "auto_scan_ff") != 0)
                return -1;
            break;

        default:
            printf("[EXEC] ❌ 执行命令失败: 未知操作码 0x%02X\n", command->opcode);
            return -1;
    }

    return 0;
}



int app_send_response(const command_t *command, const uint8_t *buf, int len) {
    if (command == NULL || buf == NULL || len <= 0) {
        printf("[REPLY] ❌ 回包失败: 参数无效(cmd=%p, buf=%p, len=%d)\n",
               (const void *)command, (const void *)buf, len);
        return -1;
    }

    // 记录本次应答: 上位机若没收到而重发同一条命令, 可以用它回放(见 dup_cache_store)
    dup_cache_store(command, buf, len);

    if (command->channel == RESPONSE_CHANNEL_RS485) {
        if (command->serial_port == NULL) {
            printf("[REPLY] ❌ 回包失败: RS485 通道的串口句柄为空\n");
            return -1;
        }
        int result = uart485_send(buf, len);
        // [2026-10-13] 成功回执不再打印: 高频回包路径(ADC 扫描每点一包)会刷爆串口。
        // 失败仍打印, 便于定位。
        if (result < 0) {
            printf("[REPLY] ❌ 回包失败: RS485 发送 %d 字节出错\n", len);
        }
        return result;
    }

    // TCP: 优先发给发起命令的客户端, fd 无效时退回"最近发送者"
    if (command->tcp_fd > 0) {
        int result = tcp_server_send_to_fd(command->tcp_fd, buf, len);
        if (result < 0) {
            printf("[REPLY] ❌ 回包失败: TCP 发送到客户端 fd=%d 出错\n", command->tcp_fd);
        }
        return result;
    }
    int result = tcp_server_send_to_last(buf, len);
    if (result < 0) {
        printf("[REPLY] ❌ 回包失败: TCP 发送到最近客户端出错 (无客户端连接?)\n");
    }
    return result;
}

void protocol_parser_init(protocol_parser_t *parser) {
    if (parser == NULL) {
        return;
    }
    parser->state = PROTOCOL_PARSE_HEADER;
    parser->need = 2;
    parser->len = 0;
    parser->parse_start_ms = 0;   // IDLE 状态, 不计时
    // 注意: 不重置 timeout_ms —— 每收完一帧都会调用本函数, 若在这里恢复默认值,
    // 485 通道设置的短超时会被悄悄改回 TCP 的 1000ms。
    if (parser->timeout_ms == 0 || parser->timeout_ms > PARSE_TIMEOUT_MS_MAX) {
        parser->timeout_ms = PARSE_TIMEOUT_MS;
    }
    parser->command.is_valid = 0;
    parser->command.opcode = 0;
    parser->command.data = NULL;
    parser->command.data_len = 0;
    parser->command.req_fp_len = 0;
}

// 设置本通道的残帧超时(必须在 protocol_parser_init 之后调用)
void protocol_parser_set_timeout(protocol_parser_t *parser, uint32_t ms) {
    if (parser == NULL) {
        return;
    }
    if (ms == 0 || ms > PARSE_TIMEOUT_MS_MAX) {
        ms = PARSE_TIMEOUT_MS;
    }
    parser->timeout_ms = ms;
}

// 收齐一个完整命令包后调用: 执行命令
static void protocol_parser_on_packet(protocol_parser_t *parser) {
    if (g_thread_pool == NULL) {
        printf("[PARSE] ❌ 处理命令包失败: 全局线程池未初始化\n");
        protocol_parser_init(parser);
        return;
    }

    parser->command.is_valid = 1;
    parser->command.data = parser->buf + PROTOCOL_DATA_OFFSET;

    // 打印：解析到完整命令，显示操作码和数据长度
    printf("[PARSE] ✅ 收到完整命令: opcode=0x%02X, 数据长度=%u 字节\n",
           parser->command.opcode, (unsigned)parser->command.data_len);

    if (execute_command(&parser->command) != 0) {
        printf("[PARSE] ❌ 命令执行失败: opcode=0x%02X\n", parser->command.opcode);
    }

    protocol_parser_init(parser);
}

// 整包收齐后校验 checksum: 通过则执行命令, 失败则丢弃本包并重新同步
static void protocol_parser_verify_packet(protocol_parser_t *parser) {
    uint16_t received_checksum = ((uint16_t)parser->buf[PROTOCOL_HEADER_SIZE] << 8) |
                                 (uint16_t)parser->buf[PROTOCOL_HEADER_SIZE + 1];
    uint16_t calculated_checksum = compute_checksum(parser->buf, parser->len);

    if (received_checksum != calculated_checksum) {
        // 打印：校验和不匹配，显示收到的校验码和计算出的校验码
        printf("[PARSE] ❌ 校验和不匹配: 收到=0x%04X, 计算=0x%04X, 本包已丢弃\n",
               received_checksum, calculated_checksum);
        send_error_reply(&parser->command,0xf2,parser->command.opcode);
        protocol_parser_init(parser);
        return;
    }

    protocol_parser_on_packet(parser);
}



// 向状态机喂入字节流; 支持粘包与拆包
void protocol_parser_feed(protocol_parser_t *parser, const uint8_t *data, int len) {
    if (parser == NULL || data == NULL || len <= 0) {
        return;
    }

    // 每收到一批字节就为残帧计时续期: 这是"字节间超时"(inter-byte), 不是"整帧超时",
    // 所以慢慢到达但一直在推进的长帧不会被误杀。
    uint64_t now = proto_now_ms();
    if (parser->state != PROTOCOL_PARSE_HEADER || parser->len > 0) {
        parser->parse_start_ms = now;
    }

    for (int i = 0; i < len; i++) {
        const uint8_t byte = data[i];
        switch (parser->state) {
        case PROTOCOL_PARSE_HEADER:
            if (parser->len == 0) {
                if (byte == 0xAB) {
                    parser->buf[parser->len++] = byte;
                }
            } else {
                if (byte == 0xCD) {
                    parser->buf[parser->len++] = byte;
                    parser->state = PROTOCOL_PARSE_LEN;
                    parser->need = 4;
                    parser->parse_start_ms = now;   // 开始计时
                } else {
                    parser->len = 0;
                    if (byte == 0xAB) {
                        parser->buf[parser->len++] = byte;
                    }
                }
            }
            break;

        case PROTOCOL_PARSE_LEN:
            parser->buf[parser->len++] = byte;
            if (--parser->need == 0) {
                parser->command.data_len = ((uint32_t)parser->buf[2] << 24) |
                                           ((uint32_t)parser->buf[3] << 16) |
                                           ((uint32_t)parser->buf[4] << 8) |
                                           parser->buf[5];
                if (parser->command.data_len >
                    MAX_PACKET_SIZE - PROTOCOL_HEADER_SIZE - PROTOCOL_CHECKSUM_SIZE) {
                    printf("[PARSE] ❌ 包长度非法: data_len=%u 超出上限, 已丢弃并重新同步\n",
                           (unsigned)parser->command.data_len);
                    protocol_parser_init(parser);
                    break;
                }
                parser->state = PROTOCOL_PARSE_OPCODE;
                parser->need = 1;
            }
            break;

        case PROTOCOL_PARSE_OPCODE:
            parser->buf[parser->len++] = byte;
            parser->command.opcode = byte;
            parser->state = PROTOCOL_PARSE_CHECKSUM;
            parser->need = PROTOCOL_CHECKSUM_SIZE;
            break;

        case PROTOCOL_PARSE_CHECKSUM:
            parser->buf[parser->len++] = byte;
            if (--parser->need == 0) {
                parser->state = PROTOCOL_PARSE_DATA;
                parser->need = parser->command.data_len;
                if (parser->need == 0) {
                    protocol_parser_verify_packet(parser);
                }
            }
            break;

        case PROTOCOL_PARSE_DATA:
            parser->buf[parser->len++] = byte;
            if (--parser->need == 0) {
                protocol_parser_verify_packet(parser);
            }
            break;
        }
    }
}

// 检查解析器是否超时: 返回1=超时(调用方应重置parser), 0=正常
int protocol_parser_check_timeout(protocol_parser_t *parser) {
    if (parser == NULL || parser->parse_start_ms == 0) {
        return 0;   // IDLE 状态, 不需要检查
    }
    uint64_t now = proto_now_ms();
    uint32_t limit = (parser->timeout_ms == 0) ? PARSE_TIMEOUT_MS : parser->timeout_ms;
    if (now >= parser->parse_start_ms && now - parser->parse_start_ms >= (uint64_t)limit) {
        printf("[PARSE] ⚠️ 解析超时: 状态=%d, 已收 %u 字节, %ums 未续上, 解析器已重置\n",
               parser->state, (unsigned)parser->len, (unsigned)limit);
        return 1;
    }
    return 0;
}