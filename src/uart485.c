#include "uart485.h"
#include "usart.h"
#include "protocol.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#ifdef __linux__
#include <linux/serial.h>
#endif
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_RS485
#include "log.h"

// ==================== 485 通道全局状态 ====================
static usart_t        *g_uart   = NULL;   // 串口句柄
static pthread_t       g_thread;          // 独立接收线程，不占用线程池
static volatile int    g_running = 0;     // 线程运行标志
static protocol_parser_t g_parser;        // 485 通道独立的协议状态机，支持粘包/拆包
static pthread_mutex_t g_send_lock = PTHREAD_MUTEX_INITIALIZER;  // 半双工发送互斥

// 运行统计: 现场只看日志判断链路是否健康(close 时打印一次)
static unsigned long g_stat_rx_bytes = 0;   // 累计收到字节
static unsigned long g_stat_resync  = 0;    // 累计因残帧超时重置次数
static unsigned long g_stat_tx_fail  = 0;   // 累计发送失败次数

// 十六进制打印一屏字节(单条 printf, 避免日志前缀插到行中间)
static void log_hex_line(const char *tag, const uint8_t *data, int len)
{
    char hex[3 * 64 + 8];
    int n = (len > 64) ? 64 : len;
    int off = 0;
    hex[0] = '\0';
    for (int i = 0; i < n; i++) {
        off += snprintf(hex + off, sizeof(hex) - (size_t)off, "%02X ", data[i]);
        if (off >= (int)sizeof(hex) - 4) break;   // 防御: 不越界
    }
    printf("[485]   %s %d 字节: %s%s\n", tag, len, hex, (len > 64) ? "..." : "");
}

// 打印当前 485 换向方式: 内核 RS485 模式(DE 由 RTS 控制) 还是板载自动收发电路
static void uart485_log_direction_mode(usart_t *u)
{
#if defined(__linux__) && defined(TIOCGRS485)
    struct serial_rs485 rs485;
    memset(&rs485, 0, sizeof(rs485));
    if (ioctl(u->fd, TIOCGRS485, &rs485) == 0) {
        printf("[485]   内核 RS485 模式: flags=0x%08X (ENABLED=%d)\n",
               (unsigned)rs485.flags, (rs485.flags & SER_RS485_ENABLED) ? 1 : 0);
        if (!(rs485.flags & SER_RS485_ENABLED)) {
            printf("[485]   ⚠️ 内核未启用 RS485 模式 -> 依赖板载自动换向(自动收发)电路\n");
        }
        return;
    }
    printf("[485]   ioctl(TIOCGRS485) 不可用(%s) -> 按自动收发电路处理\n", strerror(errno));
#else
    printf("[485]   未编译内核 RS485 支持 -> 按自动收发电路处理\n");
#endif
}

// 独立接收线程：select() 等待串口数据，读入后喂给协议状态机
static void *uart485_rx_loop(void *arg)
{
    (void)arg;
    uint8_t buf[256];

    printf("[485] ▶ 接收线程已启动 (fd=%d), 开始监听串口数据\n", g_uart->fd);

    while (g_running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(g_uart->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000;   // 50ms 超时: 既用于及时检查退出标志, 也用于驱动残帧超时检查

        int ret = select(g_uart->fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;   // 被信号打断, 不是错误
            }
            if (g_running) {
                printf("[485] ❌ select 出错: %s\n", strerror(errno));
            }
            break;
        }

        if (ret > 0 && FD_ISSET(g_uart->fd, &rfds)) {
            int n = read(g_uart->fd, buf, sizeof(buf));
            if (n > 0) {
                g_stat_rx_bytes += (unsigned long)n;
                // [2026-11-02] 485 上跑的是"命令", 量很小(每秒几帧), 全量打印不刷屏,
                // 但能一眼看出"上位机发了什么/下位机收到了什么", 是排查收发错位的关键证据。
                log_hex_line("RX", buf, n);
                // 喂给协议状态机：自动处理粘包/拆包，收齐一帧后投递到线程池
                protocol_parser_feed(&g_parser, buf, n);
            } else if (n == 0) {
                // 非阻塞 fd 无数据可读时返回 0, 不是错误
            } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                printf("[485] ❌ 读取串口失败: %s\n", strerror(errno));
            }
        }

        // [2026-11-02 修复] 残帧超时检查 —— 485 通道原先**完全没有**这一步
        //   (只有 tcpserver 的 TCP 解析器每轮检查), 后果: 只要丢一个字节, 状态机就
        //   停在中途等"剩下的字节", 把后续每一条命令的字节都当成本帧的数据吞掉,
        //   表现为"上位机连续发命令, 下位机部分命令毫无反应", 直到重启进程。
        // [2026-11-02 二次修复] 超时后不再 protocol_parser_init(整包清空), 而是
        //   只丢到下一个 AB CD 帧头处再继续解析 —— 残帧后面紧跟着的
        //   0x1D 之类的完好命令还能被救回来, 不会因为前一帧被截断就整步失败。
        //   (重同步动作已内联在 protocol_parser_check_timeout 内完成)
        if (protocol_parser_check_timeout(&g_parser)) {
            g_stat_resync++;
            printf("[485] ⚠️ 残帧已重新同步 (累计 %lu 次)\n", g_stat_resync);
        }
    }

    printf("[485] 接收线程已退出 (累计收 %lu 字节, 残帧重同步 %lu 次, 发送失败 %lu 次)\n",
           g_stat_rx_bytes, g_stat_resync, g_stat_tx_fail);
    return NULL;
}

int uart485_init(void)
{
    if (g_running) {
        printf("[485]   已初始化, 无需重复启动\n");
        return 0;
    }

    printf("[485] ▶ 初始化开始: %s @ %d\n", UART485_DEVICE, UART485_BAUD);

    g_uart = usart_init(UART485_DEVICE, UART485_BAUD);
    if (g_uart == NULL) {
        printf("[485] ❌ 初始化失败: 打开 %s @ %d 失败\n", UART485_DEVICE, UART485_BAUD);
        return -1;
    }

    uart485_log_direction_mode(g_uart);

    protocol_parser_init(&g_parser);
    // [2026-11-02] 485 用短超时: 半双工短命令一帧只有十几字节, 300ms 还没收满必是残帧,
    //   立刻丢弃重同步, 别等 TCP 那套 1s。必须在 init 之后设置(init 不会覆盖它)。
    protocol_parser_set_timeout(&g_parser, PARSE_TIMEOUT_MS_485);
    // 标记该解析器的回包来源为 485，app_send_response 会据此走串口回包
    g_parser.command.channel = RESPONSE_CHANNEL_RS485;
    g_parser.command.serial_port = g_uart;
    g_parser.command.tcp_fd = -1;
    printf("[485]   协议解析器就绪, 回包来源已设为 RS485, 残帧超时 %dms\n",
           PARSE_TIMEOUT_MS_485);

    g_running = 1;
    if (pthread_create(&g_thread, NULL, uart485_rx_loop, NULL) != 0) {
        printf("[485] ❌ 初始化失败: 创建接收线程失败: %s\n", strerror(errno));
        g_running = 0;
        usart_close(g_uart);
        g_uart = NULL;
        return -1;
    }

    printf("[485] ✅ 初始化完成: %s @ %d, 接收线程已创建\n", UART485_DEVICE, UART485_BAUD);
    return 0;
}

// 高频调用路径(每个命令的回包都会走), 成功时不打印, 避免刷屏; 仅失败时打印
int uart485_send(const uint8_t *data, int len)
{
    if (g_uart == NULL || data == NULL || len <= 0) {
        printf("[485] ❌ 发送失败: 参数无效(handle=%p, data=%p, len=%d)\n",
               (void *)g_uart, (const void *)data, len);
        return -1;
    }

    // 半双工总线：一次只允许一个线程发送，避免多任务并发时帧交错
    pthread_mutex_lock(&g_send_lock);

    // 命令帧(非 0x1E 数据帧)打印出来: 与 RX 一起构成"一问一答"的原始记录
    if (len >= (int)PROTOCOL_DATA_OFFSET && data[PROTOCOL_HEADER_SIZE - 1] != 0x1E) {
        log_hex_line("TX", data, len);
    }

    int n = usart_send(g_uart, data, len);
    // 说明: 这里不再调 tcdrain() 等物理发完再放锁 —— 本板是自动换向电路, 且 usart_send
    // 已保证"在锁内把整帧连续写进内核缓冲"(短写会重试, 写不满直接报错), 因此不会出现
    // 两帧交错。tcdrain 在 TX 被拉死时会无限等, 会把回包链路一起挂住, 风险大于收益。
    pthread_mutex_unlock(&g_send_lock);

    if (n <= 0 || n != len) {
        g_stat_tx_fail++;
        printf("[485] ❌ 发送失败: 期望 %d 字节, 实际 %d (累计失败 %lu 次)\n",
               len, n, g_stat_tx_fail);
        return -1;
    }
    return n;
}

void uart485_close(void)
{
    if (!g_running) {
        printf("[485] ⚠️ 关闭跳过: 未初始化\n");
        return;
    }
    g_running = 0;
    pthread_join(g_thread, NULL);
    usart_close(g_uart);
    g_uart = NULL;
    printf("[485] ✅ 关闭完成: 接收线程已回收, 串口已释放\n");
}
