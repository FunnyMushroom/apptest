#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "485_cpld.h"
#include "usart.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_RS485
#include "log.h"

// ==================== 配置宏 ====================
#define CPLD_UART_DEVICE      "/dev/ttyS6"
#define CPLD_UART_BAUD        115200
#define CPLD_FRAME_HEAD       0xAA
#define CPLD_FRAME_TAIL       0xED

// ==================== 全局变量 ====================
static usart_t *g_uart = NULL;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static int g_initialized = 0;

// ==================== 内部函数 ====================

// 设置串口为非阻塞模式
static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        printf("[CPLD] ❌ 取文件状态失败(fd=%d): %s\n", fd, strerror(errno));
        return -1;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
        printf("[CPLD] ❌ 设置非阻塞失败(fd=%d): %s\n", fd, strerror(errno));
        return -1;
    }
    printf("[CPLD]   已设为非阻塞模式, fd=%d\n", fd);
    return 0;
}

// ==================== 外部函数 ====================

// 初始化CPLD（打开串口，设置为非阻塞）
int cpld_init(void) {
    pthread_mutex_lock(&g_mutex);
    
    if (g_initialized) {
        printf("[CPLD]   已初始化, 无需重复打开\n");
        pthread_mutex_unlock(&g_mutex);
        return 0;
    }

    printf("[CPLD] ▶ 初始化开始: %s @ %d\n", CPLD_UART_DEVICE, CPLD_UART_BAUD);

    g_uart = usart_init(CPLD_UART_DEVICE, CPLD_UART_BAUD);
    if (g_uart == NULL) {
        printf("[CPLD] ❌ 初始化失败: 打开 %s @ %d 失败\n",
               CPLD_UART_DEVICE, CPLD_UART_BAUD);
        pthread_mutex_unlock(&g_mutex);
        return -1;
    }

    // 设置为非阻塞模式
    if (set_nonblocking(g_uart->fd) != 0) {
        printf("[CPLD] ❌ 初始化失败: 设置非阻塞模式未成功\n");
        usart_close(g_uart);
        g_uart = NULL;
        pthread_mutex_unlock(&g_mutex);
        return -1;
    }

    g_initialized = 1;
    printf("[CPLD] ✅ 初始化完成: %s @ %d, 非阻塞模式, fd=%d\n",
           CPLD_UART_DEVICE, CPLD_UART_BAUD, g_uart->fd);
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

// 反初始化CPLD（关闭串口）
void cpld_deinit(void) {
    pthread_mutex_lock(&g_mutex);
    
    if (g_initialized && g_uart != NULL) {
        usart_close(g_uart);
        g_uart = NULL;
        g_initialized = 0;
        printf("[CPLD] ✅ 关闭完成: 串口已释放\n");
    } else {
        printf("[CPLD] ⚠️ 关闭跳过: 未初始化\n");
    }
    
    pthread_mutex_unlock(&g_mutex);
}

// ==================== 快速发送: 0xAA a b 0xED ====================
// 只把 4 字节帧写进串口发送缓冲就返回, 不 tcflush/tcdrain、不等待回复。
int cpld_send_hex_command_block(uint8_t a, uint8_t b) {
    if (!g_initialized || g_uart == NULL) {
        printf("[CPLD] ❌ 发送失败: 未初始化\n");
        return -1;
    }

    uint8_t cmd[4] = { CPLD_FRAME_HEAD, a, b, CPLD_FRAME_TAIL };
    int fd = g_uart->fd;

    pthread_mutex_lock(&g_mutex);   // 仅保证 4 字节帧不被其它线程插队

    // 确保非阻塞(即使别处改回阻塞模式也能快速返回)
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1 && !(flags & O_NONBLOCK)) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    ssize_t sent = 0;
    int retry = 0;
    while (sent < (ssize_t)sizeof(cmd)) {
        ssize_t n = write(fd, cmd + sent, sizeof(cmd) - (size_t)sent);
        if (n > 0) {
            sent += n;
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;   // 被信号打断, 立即重试
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (++retry > 5) {          // 发送缓冲满: 不阻塞, 丢弃本帧
                pthread_mutex_unlock(&g_mutex);
                printf("[CPLD] ❌ 发送失败: 发送缓冲满, 丢弃帧 %02X %02X\n", a, b);
                return -1;
            }
            usleep(100);                // 100us 让出 CPU, 不作长时间等待
            continue;
        }
        printf("[CPLD] ❌ 发送失败: 帧 %02X %02X: %s\n", a, b, strerror(errno));
        pthread_mutex_unlock(&g_mutex);
        return -1;
    }

    pthread_mutex_unlock(&g_mutex);
    return 0;
}

// ==================== 频率设置: 0xAA addr ch freq 0xED (发送后等待回显) ====================
// FPGA 的 5 字节命令, 用于设置某一路开关通道的激励频率; 由 0x23(多通道选择)调用。
//   [2026-11-04] FPGA 实测: 收到频率帧后会回显 2 字节(地址+通道, 如 0F 01),
//   表示频率设置成功。本函数发完后等待这 2 字节回显:
//     - 单次等待上限 CPLD_FREQ_ECHO_TIMEOUT_US(5ms), 不重发;
//     - 回显依次等于 addr、ch = 成功(返回 0);
//     - 超时/不匹配 = 未确认(返回 -2), 只打警告, 由调用方容忍, 通道选择照常。
//   不采用 wait_reply 的"10ms×重发4次"(最坏 40ms+): 0x23 在接收线程内同步执行,
//   0x23→0x1D 间隔实测仅 ~13ms, 5ms 上限下 0x23 总耗时仍 < 7ms, 时序安全。
//   若超时后回显才到达, 残留字节由下一次 wait_reply 开头的 tcflush(TCIFLUSH) 清掉,
//   不会污染切通道命令的应答判定。
//
//   帧格式(与 FPGA 约定): AA | 地址(0~15) | 通道(1~128) | 频率(1~256, 单位 10kHz) | ED
//   与 4 字节帧(AA a b ED)的唯一区别是多了一个"频率"字节, 帧头帧尾不变。
//   [2026-11-04] 频率系数放宽到 1~256, 但帧内只有 1 字节: 256 编码为 0x00。
//
//   返回: 0 = 已发出且回显匹配; -1 = 未初始化 / 参数越界 / 写失败; -2 = 未收到匹配回显
int cpld_send_freq_frame(uint8_t addr, uint8_t ch, uint16_t freq) {
    if (!g_initialized || g_uart == NULL) {
        printf("[CPLD] ❌ 频率帧发送失败: 未初始化\n");
        return -1;
    }

    // 范围校验放在最前: 越界值一旦下发会把 FPGA 侧解析带偏, 宁可不发
    if (addr > 15 || ch < 1 || ch > 128 || freq < CPLD_FREQ_MIN || freq > CPLD_FREQ_MAX) {
        printf("[CPLD] ❌ 频率帧参数越界: 地址=%u(0~15), 通道=%u(1~128), 频率=%u(1~256)\n",
               addr, ch, freq);
        return -1;
    }

    const uint8_t freq_byte = CPLD_FREQ_TO_BYTE(freq);      // 256 -> 0x00
    const uint8_t cmd[5] = { CPLD_FRAME_HEAD, addr, ch, freq_byte, CPLD_FRAME_TAIL };
    const long    wait_us = CPLD_FREQ_ECHO_TIMEOUT_US;      // 回显等待上限 5ms

    int fd = g_uart->fd;

    pthread_mutex_lock(&g_mutex);

    // 确保非阻塞: 短写用 100us 重试补完, 整帧 5 字节通常一次写完
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1 && !(flags & O_NONBLOCK)) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    // 丢掉残留字节(比如上一条帧迟到的回显), 保持总线干净
    tcflush(fd, TCIFLUSH);

    // ---- 1) 发送整帧(非阻塞写循环, 短写用 100us 重试补完) ----
    ssize_t sent = 0;
    int retry = 0;
    while (sent < (ssize_t)sizeof(cmd)) {
        ssize_t n = write(fd, cmd + sent, sizeof(cmd) - (size_t)sent);
        if (n > 0) {
            sent += n;
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (++retry > 50) {         // 最多让出 ~5ms, 仍写不完按失败处理
                break;
            }
            usleep(100);
            continue;
        }
        break;
    }

    if (sent < (ssize_t)sizeof(cmd)) {
        pthread_mutex_unlock(&g_mutex);
        printf("[CPLD] ❌ 频率帧发送失败: 只写出 %d/%u 字节, 丢弃帧 [%02X %02X %02X %02X %02X]\n",
               (int)sent, (unsigned)sizeof(cmd),
               cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        return -1;
    }

    // ---- 2) 等待 FPGA 回显 2 字节(地址+通道), 与 wait_reply 相同的 select 循环 ----
    uint8_t buf[2] = { 0, 0 };
    int got = 0;
    struct timespec t0, now;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (got < 2) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_us = (now.tv_sec - t0.tv_sec) * 1000000L
                        + (now.tv_nsec - t0.tv_nsec) / 1000L;
        long remain_us = wait_us - elapsed_us;
        if (remain_us <= 0) {
            break;                      // 5ms 超时
        }

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = remain_us;         // 只等剩余时间

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int sret = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sret < 0) {
            if (errno == EINTR) {
                continue;
            }
            printf("[CPLD] ❌ 频率帧等待回显 select 出错(fd=%d): %s\n", fd, strerror(errno));
            break;
        }
        if (sret == 0) {
            break;                      // 窗口内无新数据 → 超时
        }

        ssize_t n = read(fd, buf + got, (size_t)(2 - got));
        if (n > 0) {
            got += (int)n;
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;                   // 数据被其它线程取走, 继续等剩余时间
        }
        printf("[CPLD] ❌ 频率帧读取回显出错(fd=%d): %s\n", fd, strerror(errno));
        break;
    }

    pthread_mutex_unlock(&g_mutex);

    // ---- 3) 判定: 回显 2 字节必须依次等于 addr、ch ----
    if (got == 2 && buf[0] == addr && buf[1] == ch) {
        printf("[CPLD] ✅ 频率帧设置成功: [%02X %02X %02X %02X %02X] 回显 [%02X %02X]"
               " (频率系数=%u%s, =%u kHz)\n",
               cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], buf[0], buf[1],
               (unsigned)freq, (freq >= 256u) ? "(满量程, 编码 0x00)" : "",
               (unsigned)freq * 10u);
        return 0;
    }

    // 未确认成功: 帧已发出, 但回显缺失/不完整/不匹配, 频率可能未生效
    printf("[CPLD] ⚠️ 频率帧已发出但回显未匹配(期望 [%02X %02X], 实际 [%02X %02X], got=%d)"
           " (频率系数=%u%s, =%u kHz): 频率可能未生效, 不重发\n",
           addr, ch, buf[0], buf[1], got,
           (unsigned)freq, (freq >= 256u) ? "(满量程, 编码 0x00)" : "",
           (unsigned)freq * 10u);
    return -2;
}

// ==================== 非阻塞接收 2 个字节(不等待, 读完即返回) ====================
// 把串口接收缓冲里已有的数据一次读完(最多 2 字节), 不用 select、不做任何等待。
// 注意: 非阻塞只能读到“已经到达”的数据, 若第 2 个字节还没到就只会拿到 1 个,
//       调用方可稍后再调一次。
// 返回: 0 = 收到 2 字节; 1 = 只收到 1 字节; -1 = 无数据/未初始化/读错误
int cpld_recv_two(uint8_t *buf) {
    if (buf == NULL) {
        printf("[CPLD] ❌ 接收失败: 输出缓冲区为 NULL\n");
        return -1;
    }

    if (!g_initialized || g_uart == NULL) {
        printf("[CPLD] ❌ 接收失败: 未初始化\n");
        return -1;
    }

    int fd = g_uart->fd;

    pthread_mutex_lock(&g_mutex);

    // 确保非阻塞: 读不到数据就立即返回 EAGAIN, 不会卡住
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1 && !(flags & O_NONBLOCK)) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    // 把当前缓冲里已有的数据尽量读满 2 字节, 一旦无数据立即退出
    int got = 0;
    while (got < 2) {
        ssize_t n = read(fd, buf + got, 2 - got);
        if (n > 0) {
            got += (int)n;
            continue;
        }
        if (n < 0 && errno == EINTR) {
            continue;   // 被信号打断, 立即重试
        }
        break;          // EAGAIN/EWOULDBLOCK(无数据) 或出错 → 立即返回
    }

    pthread_mutex_unlock(&g_mutex);

    // 只在真正收到数据时打印: 本函数会被上层轮询调用,
    // 若把“无数据”也打出来会刷屏
    if (got == 2) {
        printf("[CPLD] ✅ 接收完成: %02X %02X\n", buf[0], buf[1]);
        return 0;
    }
    if (got == 1) {
        printf("[CPLD] ⚠️ 接收不完整: 只读到 1 字节 %02X (第2字节尚未到达)\n", buf[0]);
        return 1;
    }

    return -1;   // 无数据: 静默返回, 由调用方重试
}

// ==================== 发送后阻塞等待应答: 0xAA a b 0xED -> 期望收回 a b ====================
// 发送 4 字节帧后阻塞等待回复, 要求收到的 2 个字节依次等于发送的 a、b
// (即 buf[0] == a && buf[1] == b)才判定成功。
//   - 日志同时打印“期望字节”和“实际收到字节”, 便于排查;
//   - 每次等待上限 10ms: 10ms 内没收到匹配的 2 字节即视为本次失败, 立即重发;
//   - 最多重发 3 次(即最多合计发送 4 次);
//   - 只收到 1 个字节、或 2 个字节不匹配, 都丢弃并重发。
// echo: 可为 NULL; 成功时写入收到的 2 字节(echo[0] == a, echo[1] == b)
// 返回: 0 = 成功; -1 = 未初始化/发送失败; -2 = 重发耗尽仍未收到有效应答
int cpld_send_hex_command_wait_reply(uint8_t a, uint8_t b, uint8_t *echo)
{
    if (!g_initialized || g_uart == NULL) {
        printf("[CPLD] ❌ 发送并等待应答失败: 未初始化\n");
        return -1;
    }

    uint8_t cmd[4] = { CPLD_FRAME_HEAD, a, b, CPLD_FRAME_TAIL };
    int fd = g_uart->fd;

    const int  max_resend = 3;      // 最多重发 3 次
    const long wait_us    = 10000;   // 每次等待 10ms

    printf("[CPLD] ▶ 发送并等待应答开始: 期望 [%02X %02X], 最多重发 %d 次\n",
           a, b, max_resend);

    pthread_mutex_lock(&g_mutex);

    // 确保非阻塞: select 负责“等”, read 负责“取”, 配合起来就是阻塞式等待
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1 && !(flags & O_NONBLOCK)) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    for (int attempt = 0; attempt <= max_resend; attempt++) {
        // 丢弃上一轮可能残留的半截数据, 避免污染本次判定
        tcflush(fd, TCIFLUSH);

        // ---- 1) 发送整帧(与 cpld_send_hex_command_block 相同的非阻塞写循环) ----
        ssize_t sent = 0;
        int retry = 0;
        int write_failed = 0;

        while (sent < (ssize_t)sizeof(cmd)) {
            ssize_t n = write(fd, cmd + sent, sizeof(cmd) - (size_t)sent);
            if (n > 0) {
                sent += n;
                continue;
            }
            if (n < 0 && errno == EINTR) {
                continue;   // 被信号打断, 立即重试
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (++retry > 5) {      // 发送缓冲满
                    write_failed = 1;
                    break;
                }
                usleep(100);
                continue;
            }
            printf("[CPLD] ❌ 发送失败: 帧 %02X %02X: %s\n", a, b, strerror(errno));
            write_failed = 1;
            break;
        }
        if (write_failed) {
            pthread_mutex_unlock(&g_mutex);
            printf("[CPLD] ❌ 发送失败: 发送缓冲满, 丢弃帧 %02X %02X\n", a, b);
            return -1;
        }

        // ---- 2) 阻塞等待最多 10ms, 期间尽量凑满 2 字节 ----
        uint8_t buf[2] = { 0, 0 };
        int got = 0;
        struct timespec t0, now;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        while (got < 2) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsed_us = (now.tv_sec - t0.tv_sec) * 1000000L
                            + (now.tv_nsec - t0.tv_nsec) / 1000L;
            long remain_us = wait_us - elapsed_us;
            if (remain_us <= 0) {
                break;                  // 10ms 超时
            }

            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = remain_us;     // 只等剩余时间

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);

            int sret = select(fd + 1, &rfds, NULL, NULL, &tv);
            if (sret < 0) {
                if (errno == EINTR) {
                    continue;
                }
                printf("[CPLD] ❌ select 等待应答出错(fd=%d): %s\n", fd, strerror(errno));
                break;
            }
            if (sret == 0) {
                break;                  // 10ms 内没有新数据 → 超时
            }

            ssize_t n = read(fd, buf + got, (size_t)(2 - got));
            if (n > 0) {
                got += (int)n;
                continue;
            }
            if (n < 0 && errno == EINTR) {
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                continue;               // 数据被其它线程取走, 继续等剩余时间
            }
            printf("[CPLD] ❌ 读取应答出错(fd=%d): %s\n", fd, strerror(errno));
            break;
        }

        // ---- 3) 判定: 收到的 2 字节必须依次等于 a、b ----
        if (got == 2 && buf[0] == a && buf[1] == b) {
            if (echo != NULL) {
                echo[0] = buf[0];
                echo[1] = buf[1];
            }
            pthread_mutex_unlock(&g_mutex);
            printf("[CPLD] ✅ 应答匹配(第 %d/%d 次发送): 期望 [%02X %02X] 实际 [%02X %02X]\n",
                   attempt + 1, max_resend + 1, a, b, buf[0], buf[1]);
            return 0;
        }

        if (got == 2) {
            printf("[CPLD] ⚠️ 第 %d/%d 次发送应答无效: 期望 [%02X %02X] 实际 [%02X %02X] (不匹配)%s\n",
                   attempt + 1, max_resend + 1, a, b, buf[0], buf[1],
                   (attempt < max_resend) ? ", 超时后重发" : ", 重发次数已用尽");
        } else if (got == 1) {
            printf("[CPLD] ⚠️ 第 %d/%d 次发送应答无效: 期望 [%02X %02X] 实际 [%02X] (不完整)%s\n",
                   attempt + 1, max_resend + 1, a, b, buf[0],
                   (attempt < max_resend) ? ", 超时后重发" : ", 重发次数已用尽");
        } else {
            printf("[CPLD] ⚠️ 第 %d/%d 次发送应答无效: 期望 [%02X %02X] 实际 [无应答]%s\n",
                   attempt + 1, max_resend + 1, a, b,
                   (attempt < max_resend) ? ", 超时后重发" : ", 重发次数已用尽");
        }
    }

    pthread_mutex_unlock(&g_mutex);
    printf("[CPLD] ❌ 发送并等待应答失败: 重发 %d 次后仍未收到匹配应答, 期望 [%02X %02X]\n",
           max_resend, a, b);
    return -2;
}


