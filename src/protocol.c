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

// [2026-11-02] 兜底补救钩子: 当"刚执行完的帧 + 2 字节尾巴"这一特征出现时,
//   交给 task.c 判断是不是上位机的 0x23 帧没整帧重建(见 task.c 里的详细说明)。
//   刻意用 extern 声明而**不写进头文件**: 头文件不同步时也能编译通过。
extern int task_rescue_broken_set_channels(command_t *cmd, const uint8_t *tail, int tail_len);

// 最近一次成功执行的帧(用于上面那个特征判断): 操作码 + 时刻
static uint8_t  g_last_exec_op = 0;
static uint64_t g_last_exec_ms = 0;

// 滑动重同步累计丢弃的字节数(仅用于日志)。
// [2026-11-02] 刻意做成文件级静态量, 而不是 protocol_parser_t 的成员:
//   这个统计只有日志价值, 放进结构体就要求 include/protocol.h 必须跟着一起同步,
//   一旦头文件没同步到位(比如只拷贝了 src/), 整个工程直接编译不过。
//   改成静态量后, 解析器代码对头文件没有任何新增依赖。
static uint64_t        g_parse_discarded = 0;

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

// ==================== 接收线程内同步执行(时序确定性) ====================
// [2026-11-02 二次修复] 为什么需要它:
//   0x0D/0x1B/0x23 是"每个测试点开头必发的三条短命令", 原来都投进线程池。
//   线程池调度有 2~12ms 的抖动(实测同一份日志里 0x0D 的应答时延 4~12ms),
//   后果有两个:
//     1) 上位机的步骤超时判据被这抖动撞上 -> 重发 -> 时序越滚越乱;
//     2) 更要命的是 0x23(入队) 与 0x1D(开始扫描) 落在两个线程上, 先后没有保证:
//        一旦 0x1D 先跑, 队列还是空的, 而扫描开头又要"采集期间 PD14/PD15 必须为低",
//        于是拉低动作被推到扫描开始之后 -> 上位机认为本轮已结束 / 前若干点读到 0V。
//   这三条命令本身都在 1~4ms 内完成, 直接在接收线程里同步执行即可:
//     - 应答时延变成确定值(收到命令后 1~2ms 内必回);
//     - 0x24 必定在 0x1D 之前发出, 0x1D 到达时队列必定就绪 —— 顺序天然成立。
static int dispatch_command_inline_if_len_ok(const command_t *command,
                                             uint32_t expected_len,
                                             void (*task_function)(void *),
                                             const char *command_name) {
    if (command->data_len != expected_len) {
        printf("[EXEC] ❌ 0x%02X (%s) 参数长度不合法: 实际 %u 字节, 期望 %u 字节\n",
               command->opcode, command_name, (unsigned)command->data_len,
               (unsigned)expected_len);
        send_error_reply(command, 0xf3, command->opcode);
        return -1;
    }

    uint8_t opcode = command->opcode;
    command_t *task_arg = (command_t *)calloc(1, sizeof(*task_arg));
    if (task_arg == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 分配 task_arg 内存出错 (0x%02X)\n", opcode);
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
            printf("[EXEC] ❌ 分发任务失败: 复制命令负载数据出错 (0x%02X)\n", opcode);
            free(task_arg);
            return -1;
        }
    }
    task_arg->req_fp_len = 0;
    if (command->data != NULL && command->data_len > 0) {
        uint32_t n = command->data_len > sizeof(task_arg->req_fp)
                     ? (uint32_t)sizeof(task_arg->req_fp) : command->data_len;
        memcpy(task_arg->req_fp, command->data, n);
        task_arg->req_fp_len = (uint8_t)n;
    }

    if (opcode_try_acquire(opcode) == 0) {
        if (dup_cache_replay(task_arg, opcode_busy_since(opcode))) {
            free(task_arg->data);
            free(task_arg);
            return 0;
        }
        printf("[EXEC] ⚠️ 0x%02X 正在执行中, 重复指令已丢弃\n", opcode);
        send_error_reply(command, 0xF1, opcode);
        free(task_arg->data);
        free(task_arg);
        return 0;
    }

    // 同步执行: 任务函数自己负责 free(task_arg)
    printf("[EXEC] ✅ 0x%02X (%s) 接收线程内同步执行(应答与后续命令严格有序)\n",
           opcode, command_name);
    task_function(task_arg);
    opcode_release(opcode);
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
        // [2026-11-02] 改接收线程内同步执行(应答时延确定, 见 dispatch_command_inline_if_len_ok)
        case 0x0D:
            if (dispatch_command_inline_if_len_ok(command, 1,
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
            // [2026-11-02] 同步执行: 只写一个全局变量+回包, 微秒级完成
            if (dispatch_command_inline_if_len_ok(command, 2,
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
            // [2026-11-02] 同步执行: 必须保证"入队 + 立即下发 FPGA 帧 + 回 0x24"都发生在
            //   0x1D 被解析之前, 否则扫描可能在队列还空着时启动(以及 PD14/PD15 的拉低
            //   被推迟到扫描开始之后)。
            // [2026-11-04] 与 app923 工程对齐: 只收 5 字节新格式
            //   collect(2) + switch(2) + 频率系数(1, ×10kHz);
            //   旧 4 字节格式一律回 0xF3 拒绝(上位机工具已全部升级为 5 字节)。
            if (command->data_len != 5) {
                printf("[EXEC] ❌ 0x23 (scan) 参数长度不合法: 实际 %u 字节, 期望 5 字节"
                       "(collect 2 + switch 2 + freq 1)\n",
                       (unsigned)command->data_len);
                send_error_reply(command, 0xf3, command->opcode);
                return -1;
            }
            if (dispatch_command_inline_if_len_ok(command, command->data_len,
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
    parser->need = 0;
    parser->len = 0;
    // 注意: g_parse_discarded 是全局累计量, 不随解析器重置(它只用于日志统计)
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

// ==================== 流式解析: 滑动窗口扫描 ====================
// [2026-11-02 二次修复] 原来的"状态机 + 收齐整包才校验"存在一个致命缺陷:
//   线上偶尔出现**尾部被截掉 2 字节**的帧(实测 2026_11_02_1_scan.log 4 次/7 轮)。
//   状态机于是停在 DATA 态等"剩下的 2 字节", 把随后到达的下一条命令(例如 0x1D)
//   的前 2 个字节当成残缺帧的数据咽掉, 随后校验失败 -> 整条命令丢失, 上位机一直等到超时。
//   现象就是"命令时快时慢、偶尔整步失败"。
//
// 新做法: 收到的字节全部进缓冲, 然后在一个循环里:
//   1) 对齐帧头 AB CD(不是就丢掉 1 字节继续找);
//   2) 头齐了算 data_len, 数据没齐就等下一批字节;
//   3) 数据齐了就校验; 校验失败**只丢 1 字节**继续找下一个帧头(不再整包丢弃);
//   4) 校验通过就执行, 然后从缓冲里移走这一帧。
// 这样任何粘包/拆包/残帧/多字节垃圾都只影响它自己, 再也不会"吞掉后续命令"。

// 从缓冲头部丢弃 n 字节
static void parser_consume(protocol_parser_t *parser, uint32_t n) {
    if (n >= parser->len) {
        parser->len = 0;
        return;
    }
    memmove(parser->buf, parser->buf + n, (size_t)(parser->len - n));
    parser->len -= n;
}

// 执行一个已通过校验的完整帧(位于 buf[0..total)), 由调用方负责随后移出缓冲
static void parser_exec_frame(protocol_parser_t *parser, uint32_t total) {
    parser->command.is_valid = 1;
    parser->command.opcode   = parser->buf[PROTOCOL_HEADER_SIZE - 1];
    parser->command.data_len = total - PROTOCOL_DATA_OFFSET;
    parser->command.data     = parser->buf + PROTOCOL_DATA_OFFSET;

    // 打印：解析到完整命令，显示操作码和数据长度
    printf("[PARSE] ✅ 收到完整命令: opcode=0x%02X, 数据长度=%u 字节\n",
           parser->command.opcode, (unsigned)parser->command.data_len);

    if (g_thread_pool == NULL) {
        printf("[PARSE] ❌ 处理命令包失败: 全局线程池未初始化\n");
    } else if (execute_command(&parser->command) != 0) {
        printf("[PARSE] ❌ 命令执行失败: opcode=0x%02X\n", parser->command.opcode);
    }

    parser->command.is_valid = 0;
    parser->command.data     = NULL;
    parser->command.data_len = 0;
}

// 扫描缓冲: 能凑出完整且校验正确的帧就执行, 否则滑动窗口重新找帧头
static void protocol_parser_scan(protocol_parser_t *parser) {
    for (;;) {
        if (parser->len < 2) {
            // 只剩 1 字节: 不是 0xAB 就丢掉, 是 0xAB 就留着等下一字节
            if (parser->len == 1 && parser->buf[0] != 0xAB) {
                g_parse_discarded++;
                parser_consume(parser, 1);
                continue;
            }
            break;
        }

        // 1) 对齐帧头
        if (parser->buf[0] != 0xAB || parser->buf[1] != 0xCD) {
            uint32_t drop = 1;
            while (drop < parser->len && parser->buf[drop] != 0xAB) {
                drop++;
            }
            // [2026-11-02] "刚执行的帧 + 2~3 字节尾巴" = 上位机发送缓冲区没整帧重建的
            //   特征(现场 0x1B 后面跟 06 Cx)。交给 task.c 尝试补救:
            //   返回 1 = 已按真正的意图入队并回包 0x24, 这些字节已消费完。
            //   返回 0 = 没补救(开关关闭/判据不符), 按错位数据正常丢弃。
            //   [2026-11-03] 0x23 改为 5 字节后尾巴可能是 3 字节(开关卡2+频率1), 一并交给
            //   task.c 判断(补救路径只用前 2 字节, 频率字节丢弃)。
            if ((parser->len == 2 || parser->len == 3) && drop == parser->len &&
                g_last_exec_op == 0x1B &&
                g_last_exec_ms != 0 && proto_now_ms() - g_last_exec_ms < 1000 &&
                task_rescue_broken_set_channels(&parser->command, parser->buf, (int)parser->len) == 1) {
                g_last_exec_op = 0;
                parser_consume(parser, parser->len);
                continue;
            }
            if (drop > 1) {
                printf("[PARSE] ⚠️ 丢弃 %u 字节错位数据, 重新找帧头 AB CD\n", (unsigned)drop);
            }
            g_parse_discarded += drop;
            parser_consume(parser, drop);
            continue;
        }

        // 2) 头(2+4+1+2=9 字节)是否收齐
        if (parser->len < PROTOCOL_DATA_OFFSET) {
            parser->need = PROTOCOL_DATA_OFFSET - parser->len;
            break;
        }

        uint32_t dl = ((uint32_t)parser->buf[2] << 24) | ((uint32_t)parser->buf[3] << 16) |
                      ((uint32_t)parser->buf[4] << 8)  |  (uint32_t)parser->buf[5];
        if (dl > (uint32_t)(MAX_PACKET_SIZE - PROTOCOL_DATA_OFFSET)) {
            printf("[PARSE] ❌ 包长度非法: data_len=%u 超出上限, 丢弃帧头继续找\n", (unsigned)dl);
            g_parse_discarded++;
            parser_consume(parser, 1);
            continue;
        }

        uint32_t total = (uint32_t)PROTOCOL_DATA_OFFSET + dl;
        if (parser->len < total) {
            parser->need = total - parser->len;   // 数据未齐, 等后续字节(超时由上层兜底)
            break;
        }

        // 3) 校验: 失败只丢 1 字节, 不整包丢弃 —— 后面可能还藏着一个好帧
        uint16_t received_checksum = ((uint16_t)parser->buf[PROTOCOL_HEADER_SIZE] << 8) |
                                     (uint16_t)parser->buf[PROTOCOL_HEADER_SIZE + 1];
        uint16_t calculated_checksum = compute_checksum(parser->buf, total);
        if (received_checksum != calculated_checksum) {
            printf("[PARSE] ❌ 校验和不匹配: 收到=0x%04X, 计算=0x%04X (op=0x%02X, len=%u) "
                   "-> 只丢 1 字节继续找帧头, 不再整包丢弃\n",
                   received_checksum, calculated_checksum, parser->buf[6], (unsigned)dl);
            g_parse_discarded++;
            parser_consume(parser, 1);
            continue;
        }

        // 4) 执行并移出缓冲
        g_last_exec_op = parser->buf[6];
        g_last_exec_ms = proto_now_ms();
        parser_exec_frame(parser, total);
        parser_consume(parser, total);
    }

    parser->need = parser->len;
}

// 残帧超时后调用: 丢到"下一个 AB CD 帧头"处再重新扫描。
// 不能直接清空缓冲 —— 残帧后面往往紧跟着一条完好的命令(例如 0x1D), 直接清空就把人丢了。
// [2026-11-02] 由 protocol_parser_check_timeout() 内部调用, 不再需要调用方手动调用。
//   两点刻意设计, 都是为了让"只同步了部分文件"也能编过:
//   1) 不在 include/protocol.h 里声明 —— 头文件没同步时, 调用方不会出现隐式声明问题;
//   2) 保留外部链接(不加 static) —— 万一某个调用方文件还是旧版(仍在手动调用它),
//      链接时依然找得到符号, 不会变成 undefined reference。
void protocol_parser_flush_stale(protocol_parser_t *parser) {
    if (parser == NULL || parser->len == 0) {
        return;
    }

    uint32_t idx = 0;
    for (uint32_t i = 1; i + 1 < parser->len; i++) {
        if (parser->buf[i] == 0xAB && parser->buf[i + 1] == 0xCD) {
            idx = i;
            break;
        }
    }

    if (idx > 0) {
        printf("[PARSE]   ↳ 残帧后找到新帧头(偏移 %u), 丢弃前面的残字节并继续解析\n",
               (unsigned)idx);
        g_parse_discarded += idx;
        parser_consume(parser, idx);
        protocol_parser_scan(parser);
        // 重新起算: 剩下的字节属于"新帧", 给它们一个完整的超时窗口,
        // 否则 parse_start_ms 还是旧的, 下一轮会立刻再次判定超时。
        parser->parse_start_ms = (parser->len > 0) ? proto_now_ms() : 0;
        parser->state = (parser->len > 0) ? PROTOCOL_PARSE_DATA : PROTOCOL_PARSE_HEADER;
        parser->need = parser->len;
        return;
    }

    // 没有完整帧头: 末尾若是 0xAB 就留着当下一帧起点, 其余全部丢弃
    uint8_t keep = (parser->buf[parser->len - 1] == 0xAB) ? 1u : 0u;
    g_parse_discarded += parser->len - keep;
    parser->len = keep;
    if (keep) {
        parser->buf[0] = 0xAB;
    }
    // 不论是否留了一个 0xAB, 都停止计时: 否则只剩一个 0xAB 时会每 80ms "超时"打印一次。
    // 新字节到达时 feed() 会重新开始计时。
    parser->parse_start_ms = 0;
    parser->state = PROTOCOL_PARSE_HEADER;
    parser->need = parser->len;
}



// 向解析器喂入字节流: 支持粘包/拆包/残帧, 实现见上面的协议扫描器
void protocol_parser_feed(protocol_parser_t *parser, const uint8_t *data, int len) {
    if (parser == NULL || data == NULL || len <= 0) {
        return;
    }

    // 单批就超过缓冲: 只留最后 MAX_PACKET_SIZE 字节(一帧最长也就这么多), 保证还能继续同步
    if (len > MAX_PACKET_SIZE) {
        printf("[PARSE] ⚠️ 单批收到 %d 字节, 超过缓冲上限 %d, 只保留尾部\n",
               len, MAX_PACKET_SIZE);
        data += (len - MAX_PACKET_SIZE);
        len = MAX_PACKET_SIZE;
        parser->len = 0;
    }

    // 缓冲不够: 丢掉最老的字节(滑动), 绝不让缓冲溢出把解析器卡死
    if (parser->len + (uint32_t)len > MAX_PACKET_SIZE) {
        uint32_t overflow = parser->len + (uint32_t)len - MAX_PACKET_SIZE;
        printf("[PARSE] ⚠️ 接收缓冲将溢出, 丢弃最老的 %u 字节\n", (unsigned)overflow);
        g_parse_discarded += overflow;
        parser_consume(parser, overflow);
    }

    memcpy(parser->buf + parser->len, data, (size_t)len);
    parser->len += (uint32_t)len;

    // 收到字节就为残帧计时续期: 这是"字节间超时"(inter-byte), 不是"整帧超时"
    parser->parse_start_ms = proto_now_ms();
    parser->state = PROTOCOL_PARSE_DATA;

    protocol_parser_scan(parser);

    if (parser->len == 0) {
        parser->parse_start_ms = 0;                 // 缓冲已清空 -> 空闲, 不计时
        parser->state = PROTOCOL_PARSE_HEADER;
    }
    parser->need = parser->len;                     // 兼容字段: 表示还有多少字节待处理
}

// 检查解析器是否超时: 返回1=已超时并完成重新同步, 0=正常
// [2026-11-02] 重同步动作(flush_stale)已内联在本函数里:
//   调用方只需 `if (protocol_parser_check_timeout(&p)) { ...打日志/计数... }`,
//   不需要再调用任何新接口 —— 这样即使 include/protocol.h 没同步也不会编译失败。
int protocol_parser_check_timeout(protocol_parser_t *parser) {
    if (parser == NULL || parser->parse_start_ms == 0 || parser->len == 0) {
        return 0;   // IDLE 状态, 不需要检查
    }
    uint64_t now = proto_now_ms();
    uint32_t limit = (parser->timeout_ms == 0) ? PARSE_TIMEOUT_MS : parser->timeout_ms;
    if (now >= parser->parse_start_ms && now - parser->parse_start_ms >= (uint64_t)limit) {
        printf("[PARSE] ⚠️ 残帧超时: 缓冲 %u 字节 %ums 没续上(还差 %u 字节), 重新同步"
               "(累计丢弃 %llu 字节)\n",
               (unsigned)parser->len, (unsigned)limit, (unsigned)parser->need,
               (unsigned long long)g_parse_discarded);
        protocol_parser_flush_stale(parser);   // 只丢残帧, 保住残帧后面的完好命令
        return 1;
    }
    return 0;
}