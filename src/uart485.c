#include "uart485.h"
#include "usart.h"
#include "protocol.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_RS485
#include "log.h"

// ==================== 485 通道全局状态 ====================
static usart_t        *g_uart   = NULL;   // 串口句柄
static pthread_t       g_thread;          // 独立接收线程，不占用线程池
static volatile int    g_running = 0;     // 线程运行标志
static protocol_parser_t g_parser;        // 485 通道独立的协议状态机，支持粘包/拆包
static pthread_mutex_t g_send_lock = PTHREAD_MUTEX_INITIALIZER;  // 半双工发送互斥

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
        tv.tv_usec = 100000;   // 100ms 超时，用于及时检查退出标志

        int ret = select(g_uart->fd + 1, &rfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (g_running) {
                printf("[485] ❌ select 出错: %s\n", strerror(errno));
            }
            break;
        }

        if (ret > 0 && FD_ISSET(g_uart->fd, &rfds)) {
            int n = read(g_uart->fd, buf, sizeof(buf));
            if (n > 0) {
                printf("[485]   收到 %d 字节, 送入协议状态机\n", n);
                // 喂给协议状态机：自动处理粘包/拆包，收齐一帧后投递到线程池
                protocol_parser_feed(&g_parser, buf, n);
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                printf("[485] ❌ 读取串口失败: %s\n", strerror(errno));
            }
        }
    }

    printf("[485] 接收线程已退出\n");
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

    protocol_parser_init(&g_parser);
    // 标记该解析器的回包来源为 485，app_send_response 会据此走串口回包
    g_parser.command.channel = RESPONSE_CHANNEL_RS485;
    g_parser.command.serial_port = g_uart;
    g_parser.command.tcp_fd = -1;
    printf("[485]   协议解析器就绪, 回包来源已设为 RS485\n");

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
    int n = usart_send(g_uart, data, len);
    pthread_mutex_unlock(&g_send_lock);

    if (n <= 0) {
        printf("[485] ❌ 发送失败: 期望 %d 字节, 实际 %d\n", len, n);
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