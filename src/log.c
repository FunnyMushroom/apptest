// clock_gettime / localtime_r 需要特性宏(必须在包含任何头文件之前定义)
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "log.h"

/* 本文件内部要用真正的 printf / fprintf / perror, 先解除重定向 */
#undef printf
#undef fprintf
#undef perror

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>     // getenv
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>      // open / O_WRONLY / O_NONBLOCK
#include <unistd.h>     // write / isatty
#include <sys/ioctl.h>  // ioctl / TIOCOUTQ

/* ==================== 分模块打印开关 ====================
 * 本文件的打印已在上方 #undef 解除重定向, 所以不会自己过滤自己。
 * 这段实现放在 #if LOG_ENABLE 之外: 即使编译期关闭了所有打印,
 * 上位机 0x66 命令调用的 log_set_module_mask() / log_module_name()
 * 仍然必须存在, 否则会链接失败。
 */
volatile unsigned int g_log_module_mask = 0xFFFFFFFFu;

void log_set_module_mask(unsigned int mask)
{
    /* 只改低 16 位(对应 LOG_MOD_xxx); 高 16 位保持 1,
     * 供未指定 LOG_MODULE 的文件(LOG_MOD_UNSET)使用 */
    g_log_module_mask = 0xFFFF0000u | (mask & 0xFFFFu);
}

unsigned int log_get_module_mask(void)
{
    return g_log_module_mask & 0xFFFFu;
}

const char *log_module_name(int bit)
{
    switch (bit) {
        case LOG_MOD_MAIN:    return "main/threadpool";
        case LOG_MOD_I2C:     return "i2c";
        case LOG_MOD_SPI:     return "spi";
        case LOG_MOD_GPIO:    return "gpio/gpio_app";
        case LOG_MOD_USART:   return "usart";
        case LOG_MOD_RS485:   return "uart485/485_cpld";
        case LOG_MOD_ADC121:  return "adc121s/realtime";
        case LOG_MOD_ADS1115: return "ads1115";
        case LOG_MOD_ADS8691: return "ads8691";
        case LOG_MOD_DAC8562: return "dac8562";
        case LOG_MOD_FPGA:    return "fpga_i2c";
        case LOG_MOD_TCP:     return "tcpserver";
        case LOG_MOD_NET:     return "net_config/eth";
        case LOG_MOD_TASK:    return "task";
        case LOG_MOD_PROTO:   return "protocol";
        case LOG_MOD_SYS:     return "ota/sd";
        default:              return "(未分配)";
    }
}

#if LOG_ENABLE

/* ==================== 全局开关 ==================== */
volatile int g_log_enabled = 1;   /* 运行时总开关: 0 = 静默 */
volatile int g_log_prefix  = 1;   /* 是否输出时间戳/级别前缀 */

/* 输出锁: 防止多线程日志互相交错(跨行打印时尤其明显) */
static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;

void log_set_enabled(int on) { g_log_enabled = on ? 1 : 0; }
int  log_get_enabled(void)   { return g_log_enabled; }

void log_set_prefix(int on)  { g_log_prefix = on ? 1 : 0; }
int  log_get_prefix(void)    { return g_log_prefix; }

/* ==================== 内部辅助 ==================== */

// 级别标签(固定 4 字符宽, 便于对齐)
static const char *log_level_tag(int level)
{
    switch (level) {
        case LOG_LEVEL_WARN:  return "WARN";
        case LOG_LEVEL_ERROR: return "ERR ";
        default:              return "INFO";
    }
}

// 级别对应的输出流: INFO/WARN -> stdout, ERROR -> stderr(与原 printf/fprintf 行为一致)
static FILE *log_level_stream(int level)
{
    return (level == LOG_LEVEL_ERROR) ? stderr : stdout;
}

// 生成 "[HH:MM:SS.mmm] [XXXX] " 前缀; 失败时返回空串
static void log_make_prefix(int level, char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return;
    }
    out[0] = '\0';

    if (!g_log_prefix) {
        return;   // 前缀关闭: 退化为纯 printf 风格
    }

    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        return;
    }

    struct tm tm_local;
    if (localtime_r(&ts.tv_sec, &tm_local) == NULL) {
        return;
    }

    // [app923] 工程标识 + 时间戳, 与其他工程(app929)的日志区分
    snprintf(out, out_size, "[app923 %02d:%02d:%02d.%03ld] [%s] ",
             tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec,
             ts.tv_nsec / 1000000L, log_level_tag(level));
}

/* ==================== 统一输出入口 ==================== */
/* [2026-10-13] 串口旁路(dual output)
 * 自启时 stdout 已重定向到 /tmp/program_arm.log —— 目的是不与内核 printk / init
 * 争抢 console(争抢会导致 write 阻塞, 而此处又持着日志锁, 整个程序就卡死)。
 * 但为了仍能在串口终端上"直接看到"输出(无需敲 tail -f), 这里把同一份日志
 * 再尽力写一份到 /dev/console: 用 O_NONBLOCK 打开, 写不进去就丢弃,
 * 绝不阻塞任何线程 —— 即"串口拥塞只丢日志, 不拖累业务"。
 * 可用环境变量 PROGRAM_ARM_CONSOLE=0 关闭该旁路(仅保留文件)。 */
static int g_console_fd = -1;      // -1 = 未打开/打开失败
static int g_console_tried = 0;    // 只尝试打开一次
/* [2026-10-16] 旁路重试: 队列满时小睡重试而不是直接丢行, 让初始化收尾日志
 * (如"全部子系统初始化完成")能完整上串口 —— 32M 板多次实测收尾几行被挤掉。
 * 两道保险丝, 保证绝不退回"持锁阻塞卡死"老路:
 *   ① 单行最多重试 CONSOLE_RETRY_LINE_US(500ms), 超时放弃本行
 *   ② 累计重试超 CONSOLE_RETRY_TOTAL_US(5s) 后, 永久退回旧的"只丢不等"模式
 * 串口写用独立互斥锁串行化(修复两个线程交叉写导致行序错乱/半行),
 * 不占日志锁, 日志落盘永远不被串口拖累。 */
#define CONSOLE_RETRY_LINE_US   500000    /* 单行重试上限 500ms */
#define CONSOLE_RETRY_TOTAL_US  5000000   /* 全程累计重试上限 5s */
#define CONSOLE_QUEUE_LIMIT     3500      /* 队列水位: outq + 剩余字节 < 此值才写 */
static pthread_mutex_t g_console_lock = PTHREAD_MUTEX_INITIALIZER;
static long long g_console_retry_spent_us = 0;  // 累计重试等待(g_console_lock 保护)
static int g_console_retry_disabled = 0;        // 1 = 保险丝烧断, 退回丢弃模式

void log_emit(int level, const char *fmt, ...)
{
    if (!g_log_enabled || fmt == NULL) {
        return;
    }

    char prefix[64];
    log_make_prefix(level, prefix, sizeof(prefix));
    FILE *out = log_level_stream(level);

    /* 1) 整行(前缀 + 正文)先格式化到内存缓冲, 之后文件/串口共用这一份 */
    char line[512];
    int n = 0;
    if (prefix[0] != '\0') {
        int p = snprintf(line, sizeof(line), "%s", prefix);
        n = (p > 0) ? p : 0;
        if (n >= (int)sizeof(line)) n = (int)sizeof(line) - 1;
    }
    if (n < (int)sizeof(line) - 1) {
        va_list ap;
        va_start(ap, fmt);
        int m = vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
        va_end(ap);
        if (m > 0 && (size_t)(n + m) < sizeof(line)) {
            n += m;
        } else {
            n = (int)sizeof(line) - 1;   // 过长则截断(保留结尾 '\0')
        }
    }
    if (n <= 0) {
        return;
    }

    /* 2) 主输出(stdout, 自启时=文件): 完整落盘, 不受串口影响 */
    pthread_mutex_lock(&g_log_lock);
    fwrite(line, 1, (size_t)n, out);
    fflush(out);   // 立即落盘, 避免崩溃/卡死时日志丢失
    pthread_mutex_unlock(&g_log_lock);

    /* 3) 旁路输出(/dev/console = 串口)
     *    [2026-10-16 改版] 队列满不再直接丢行, 而是独立锁内小睡重试:
     *    - 每次写前查 TIOCOUTQ 水位, 够放剩余字节才写;
     *    - write 部分写入时继续补完(EAGAIN 除外), 消灭半行;
     *    - 单行 500ms / 累计 5s 两道保险丝, 超限退回丢弃模式。
     *    独立 g_console_lock 串行化串口写(行序不再错乱), 日志锁不被占用。 */
    if (g_console_fd < 0 && !g_console_tried) {
        g_console_tried = 1;
        const char *on = getenv("PROGRAM_ARM_CONSOLE");
        /* 仅当 stdout 不是终端时才旁路(自启时 stdout=文件需要旁路;
         * 手动前台运行时 stdout 已是串口终端, 再旁路会重复输出)。 */
        if ((on == NULL || on[0] != '0') && !isatty(STDOUT_FILENO)) {
            g_console_fd = open("/dev/console", O_WRONLY | O_NONBLOCK);
        }
    }
    if (g_console_fd >= 0 && !g_console_retry_disabled) {
        pthread_mutex_lock(&g_console_lock);
        if (!g_console_retry_disabled) {
            const char *p = line;
            size_t left = (size_t)n;
            long long line_spent = 0;
            while (left > 0) {
                int outq = -1;
                if (ioctl(g_console_fd, TIOCOUTQ, &outq) != 0) {
                    outq = -1;
                }
                if (outq < 0 || outq + (int)left < CONSOLE_QUEUE_LIMIT) {
                    ssize_t w = write(g_console_fd, p, left);
                    if (w > 0) {
                        p += w;
                        left -= (size_t)w;
                        continue;   /* 部分写入: 继续补完剩余字节 */
                    }
                    if (w == 0) break;
                    if (errno != EAGAIN && errno != EWOULDBLOCK) break;
                    /* w<0 且 EAGAIN: 队列满, 走下面的重试等待 */
                }
                /* 队列满: 重试等待(两道保险丝) */
                if (line_spent >= CONSOLE_RETRY_LINE_US ||
                    g_console_retry_spent_us >= CONSOLE_RETRY_TOTAL_US) {
                    break;
                }
                usleep(20000);
                line_spent += 20000;
                g_console_retry_spent_us += 20000;
            }
            if (g_console_retry_spent_us >= CONSOLE_RETRY_TOTAL_US) {
                g_console_retry_disabled = 1;   /* 保险丝烧断: 此后只丢不等 */
            }
        }
        pthread_mutex_unlock(&g_console_lock);
    }
}

/* ==================== perror 替代 ==================== */
// 输出格式与 perror 一致: "msg: 错误描述\n", 但带前缀且受总开关控制
void log_perror(const char *msg)
{
    if (!g_log_enabled) {
        return;
    }

    int err = errno;   // 先保存, 避免后续调用改写 errno

    char prefix[64];
    log_make_prefix(LOG_LEVEL_ERROR, prefix, sizeof(prefix));

    pthread_mutex_lock(&g_log_lock);

    if (prefix[0] != '\0') {
        fputs(prefix, stderr);
    }
    fprintf(stderr, "%s: %s\n",
            (msg != NULL) ? msg : "",
            strerror(err));
    fflush(stderr);

    pthread_mutex_unlock(&g_log_lock);
}

#endif /* LOG_ENABLE */

/* ==================== fprintf 转发 ====================
 * 始终编译(即使 LOG_ENABLE=0), 因为 ota.c 等要用它写文件:
 *   - stdout / stderr  -> 按日志规则输出(受总开关 + 分模块开关控制)
 *   - 其它 stream      -> 原样透传给真正的 fprintf, 不受任何开关影响
 *
 * mod_on 由 log.h 的 fprintf 宏传入(本文件的分模块开关),
 * 写文件时必须忽略它, 否则关掉日志会把写文件一起干掉。
 */
int log_fprintf(int mod_on, FILE *stream, const char *fmt, ...)
{
    if (stream == NULL || fmt == NULL) {
        return -1;
    }

    va_list ap;

    // 非标准流(写文件等): 直接透传, 保证写文件行为完全不变
    if (stream != stdout && stream != stderr) {
        va_start(ap, fmt);
        int n = vfprintf(stream, fmt, ap);
        va_end(ap);
        return n;
    }

#if LOG_ENABLE
    if (!g_log_enabled || !mod_on) {
        return 0;
    }

    int level = (stream == stderr) ? LOG_LEVEL_ERROR : LOG_LEVEL_INFO;

    char prefix[64];
    log_make_prefix(level, prefix, sizeof(prefix));

    pthread_mutex_lock(&g_log_lock);

    int n = 0;
    if (prefix[0] != '\0') {
        n += fputs(prefix, stream);
    }
    va_start(ap, fmt);
    n += vfprintf(stream, fmt, ap);
    va_end(ap);
    fflush(stream);

    pthread_mutex_unlock(&g_log_lock);
    return n;
#else
    (void)ap;
    (void)mod_on;
    return 0;   // 编译期关闭: 丢弃标准流输出
#endif /* LOG_ENABLE */
}
