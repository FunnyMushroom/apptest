#ifndef LOG_H
#define LOG_H

/* ============================================================
 *  统一日志模块 —— 一键开关所有打印
 * ============================================================
 *
 *  两种开关方式(可同时使用):
 *
 *  1) 编译期总开关(改下面这一个宏即可):
 *         LOG_ENABLE = 1   打开所有打印(默认)
 *         LOG_ENABLE = 0   关闭所有打印: LOG/LOGW/LOGE/LOGPERROR
 *                          全部展开为空语句, 零运行时开销
 *
 *  2) 运行时开关(不用重新编译):
 *         log_set_enabled(0);   // 一键静默所有打印
 *         log_set_enabled(1);   // 重新打开
 *
 *  3) 分模块(按文件)打印开关 —— 16 位掩码, 每 1 位控制一个(组)文件
 *         上位机命令 0x66: 数据 2 字节(大端 16 位掩码), bit=1 打开, bit=0 关闭
 *         例: 关闭 spi.c 和 task.c 的打印 -> 掩码 = 0xFFFF & ~((1<<SPI)|(1<<TASK))
 *         也可以在程序里直接调 log_set_module_mask(0x00FF);
 *
 *      ┌─ 用法: 在 .c 文件里, 于 #include "log.h" 之前指定本文件所属模块位 ─┐
 *      │      #define LOG_MODULE  LOG_MOD_SPI                            │
 *      │      #include "log.h"                                          │
 *      └─────────────────────────────────────────────────────────────────┘
 *      之后该文件内所有 printf / fprintf(stderr) / perror / LOG / LOGW / LOGE
 *      都自动受这一位控制。未指定 LOG_MODULE 的文件不受分模块开关影响。
 *      总开关 log_set_enabled(0) 仍然能一键静默全部打印。
 *
 *  用法: 调用点什么都不用改!
 *  本头文件会把标准打印重定向到日志模块:
 *         printf("x=%d\n", x);            -> 输出到 stdout(带前缀)
 *         fprintf(stderr, "x=%d\n", x);   -> 输出到 stderr(带前缀)
 *         perror("recv");                 -> 输出 "msg: 错误描述"
 *         fprintf(fp, "%d\n", v);         -> 写文件, 原样透传(不受开关影响)
 *
 *  输出格式(自动加时间戳 + 级别前缀):
 *         [12:34:56.789] [INFO] I2C init ok
 *         [12:34:56.790] [ERR ] open /dev/i2c-0 failed
 *
 *  说明:
 *    - 内部加锁, 多线程输出不会互相交错, 每条日志后立即 fflush
 *    - log_set_prefix(0) 可去掉前缀, 退化成纯 printf 风格
 *    - 想在调用点显式写级别, 可用 LOG() / LOGW() / LOGE() / LOGPERROR()
 *
 *  ⚠️ 只能从 .c 文件包含本头文件, 不要放进其它 .h 里。
 *
 *  也可以在编译命令上全局覆盖, 无需改代码:
 *         gcc -DLOG_ENABLE=0 ...        // 整个工程不产生任何打印
 * ============================================================ */

#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

/* 必须先包含 <stdio.h>, 保证标准声明不被文件末尾的重定向宏影响 */
#include <stdio.h>
#include <stddef.h>

#if LOG_ENABLE

/* 日志级别 */
typedef enum {
    LOG_LEVEL_INFO  = 0,
    LOG_LEVEL_WARN  = 1,
    LOG_LEVEL_ERROR = 2
} log_level_t;

/* 运行时总开关: 0 = 全部静默, 非0 = 正常输出(默认 1) */
extern volatile int g_log_enabled;

/* 是否输出时间戳/级别前缀: 0 = 退化为纯 printf 风格(默认 1) */
extern volatile int g_log_prefix;

/* 一键开关: on=0 关闭所有打印, on!=0 打开 */
void log_set_enabled(int on);
int  log_get_enabled(void);

/* 前缀开关 */
void log_set_prefix(int on);
int  log_get_prefix(void);

/* 统一输出入口(一般不要直接调用, 用下面的宏) */
void log_emit(int level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* 等价于 perror(): 输出 "msg: 错误描述\n", 受总开关控制 */
void log_perror(const char *msg);

#define LOG(...)        do { if (LOG_MOD_ON()) log_emit(LOG_LEVEL_INFO,  __VA_ARGS__); } while (0)
#define LOGW(...)       do { if (LOG_MOD_ON()) log_emit(LOG_LEVEL_WARN,  __VA_ARGS__); } while (0)
#define LOGE(...)       do { if (LOG_MOD_ON()) log_emit(LOG_LEVEL_ERROR, __VA_ARGS__); } while (0)
#define LOGPERROR(msg)  do { if (LOG_MOD_ON()) log_perror(msg); } while (0)

#else  /* ---------------- LOG_ENABLE == 0: 全部编译成空语句 ---------------- */

#define LOG(...)        ((void)0)
#define LOGW(...)       ((void)0)
#define LOGE(...)       ((void)0)
#define LOGPERROR(msg)  ((void)0)

#define log_set_enabled(on)  ((void)(on))
#define log_get_enabled()    (0)
#define log_set_prefix(on)   ((void)(on))
#define log_get_prefix()     (0)

#endif /* LOG_ENABLE */


/* ============================================================
 *  分模块打印开关（16 位，每个 bit 对应一个/一组 .c 文件）
 * ============================================================
 *
 *  用法（必须写在本文件的 #include "log.h" 之前）:
 *
 *      #define LOG_MODULE  LOG_MOD_SPI
 *      #include "log.h"
 *
 *  掩码语义: bit=1 -> 打开该模块打印; bit=0 -> 关闭
 *  总开关 log_set_enabled(0) 优先级最高, 能一键静默全部。
 *
 *  ⚠️ 若想调整"哪一位对应哪个文件", 只改下面的位号定义即可,
 *     但同时要同步上位机脚本 tools/set_print_mask.py 里的名称表。
 * ============================================================ */

#define LOG_MOD_COUNT   16      /* 上位机下发的掩码位数 */

#define LOG_MOD_MAIN     0      /* main.c, threadpool.c        */
#define LOG_MOD_I2C      1      /* i2c.c                       */
#define LOG_MOD_SPI      2      /* spi.c                       */
#define LOG_MOD_GPIO     3      /* gpio.c, gpio_app.c          */
#define LOG_MOD_USART    4      /* usart.c                     */
#define LOG_MOD_RS485    5      /* uart485.c, 485_cpld.c       */
#define LOG_MOD_ADC121   6      /* adc121s.c, realtime.c       */
#define LOG_MOD_ADS1115  7      /* ads1115.c                   */
#define LOG_MOD_ADS8691  8      /* ads8691.c                   */
#define LOG_MOD_DAC8562  9      /* dac8562.c                   */
#define LOG_MOD_FPGA    10      /* fpga_i2c.c                  */
#define LOG_MOD_TCP     11      /* tcpserver.c                 */
#define LOG_MOD_NET     12      /* net_config.c, eth.c         */
#define LOG_MOD_TASK    13      /* task.c                      */
#define LOG_MOD_PROTO   14      /* protocol.c                  */
#define LOG_MOD_SYS     15      /* ota.c, sd.c                 */

/* 未指定 LOG_MODULE 的文件属于"未分配"模块:
 * 对应高 16 位(始终为1), 因此不受分模块开关影响, 只跟随总开关 */
#define LOG_MOD_UNSET   16

#ifndef LOG_MODULE
#define LOG_MODULE      LOG_MOD_UNSET
#endif

/* 本文件的分模块开关是否打开 */
#define LOG_MOD_ON()    (((g_log_module_mask) >> (LOG_MODULE)) & 1u)

/* 分模块开关掩码: 低 16 位对应 LOG_MOD_xxx(1=打开), 高 16 位保留为 1 */
extern volatile unsigned int g_log_module_mask;

/* 设置/读取低 16 位掩码(上位机 0x66 命令用) */
void         log_set_module_mask(unsigned int mask);
unsigned int log_get_module_mask(void);

/* 位号 -> 可读模块名(用于打印/自检) */
const char  *log_module_name(int bit);


/* ============================================================
 *  标准打印重定向
 *  必须放在本文件最后: 上面所有用到 printf 的地方都已处理完毕。
 * ============================================================ */

/* 写文件时仍需真正的 fprintf, 所以始终经过这一层转发。
 * 第一个参数是本模块的分模块开关(由 LOG_MOD_ON() 传入):
 *   - stdout / stderr -> 走日志, 受总开关 + 分模块开关控制
 *   - 其它 stream     -> 原样透传(写文件), 不受任何开关影响 */
int log_fprintf(int mod_on, FILE *stream, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

#if LOG_ENABLE
#  define printf(...)   do { if (LOG_MOD_ON()) log_emit(LOG_LEVEL_INFO, __VA_ARGS__); } while (0)
#  define fprintf(...)  log_fprintf(LOG_MOD_ON(), __VA_ARGS__)
#  define perror(msg)   do { if (LOG_MOD_ON()) log_perror(msg); } while (0)
#else
#  define printf(...)   ((void)0)
#  define perror(msg)   ((void)0)
   /* fprintf 不能置空: ota.c 等仍要用它写文件, 由 log_fprintf 内部判断丢弃/透传 */
#  define fprintf(...)  log_fprintf(1, __VA_ARGS__)
#endif /* LOG_ENABLE */

#endif /* LOG_H */
