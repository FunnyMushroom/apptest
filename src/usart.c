#include "usart.h"
#include <poll.h>
#include <stdint.h>
#include <time.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_USART
#include "log.h"

// 波特率转换
static int usart_get_baud(int baudrate) {
    switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

usart_t *usart_init(const char *device, int baudrate) {
    if (device == NULL) {
        printf("[USART] ❌ 初始化失败: 设备路径为 NULL\n");
        return NULL;
    }

    printf("[USART] ▶ 初始化开始: 设备=%s, 波特率=%d\n", device, baudrate);

    usart_t *usart = (usart_t *)calloc(1, sizeof(usart_t));
    if (usart == NULL) {
        printf("[USART] ❌ 初始化失败: 分配 usart_t 内存失败: %s\n", strerror(errno));
        return NULL;
    }

    strncpy(usart->device, device, sizeof(usart->device) - 1);
    usart->device[sizeof(usart->device) - 1] = '\0';
    usart->baudrate = baudrate;
    usart->is_open = false;

    if (access(device, F_OK) != 0) {
        printf("[USART] ❌ 初始化失败: 设备节点不存在 %s\n", device);
        free(usart);
        return NULL;
    }

    // 非阻塞打开
    usart->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (usart->fd == -1) {
        printf("[USART] ❌ 初始化失败: 打开 %s 失败: %s\n", device, strerror(errno));
        free(usart);
        return NULL;
    }
    printf("[USART]   设备已打开, fd=%d\n", usart->fd);

    // 配置串口
    struct termios options;
    if (tcgetattr(usart->fd, &options) != 0) {
        printf("[USART] ❌ 初始化失败: 读取 %s 终端属性失败: %s\n",
               device, strerror(errno));
        close(usart->fd);
        free(usart);
        return NULL;
    }

    int baud = usart_get_baud(baudrate);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;      // 无校验
    options.c_cflag &= ~CSTOPB;      // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;          // 8位数据
    options.c_cflag &= ~CRTSCTS;     // 无硬件流控

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcflush(usart->fd, TCIOFLUSH);
    if (tcsetattr(usart->fd, TCSANOW, &options) != 0) {
        printf("[USART] ❌ 初始化失败: 设置 %s 终端属性失败: %s\n",
               device, strerror(errno));
        close(usart->fd);
        free(usart);
        return NULL;
    }

    usart->is_open = true;
    printf("[USART] ✅ 初始化完成: %s, 波特率=%d, fd=%d (非阻塞)\n",
           device, baudrate, usart->fd);
    return usart;
}

// 一次 write 在总线上最多等这么久(累计), 防止半双工总线异常时把发送线程永久堵死
#define USART_TX_TIMEOUT_MS  200

static int64_t usart_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + (int64_t)(ts.tv_nsec / 1000000L);
}

// 高频调用路径(每个命令都会发), 成功时不打印, 避免刷屏; 仅失败时打印
//
// [2026-11-02 修复] fd 是 O_NONBLOCK 打开的, 原先只调一次 write():
//   1) 缓冲区满时 write 返回 -1/EAGAIN -> 整帧一个字节都没发出去, 却只报"失败";
//   2) 更糟的是短写(返回 0 < n < len): 旧代码把 n 原样返回, 调用方 task_reply()
//      只判断 "<= 0", 于是**半帧被当成发送成功**, 对端从此帧错位。
//      这正是"上位机偶发某个点收不到应答"的一个可能来源。
//   现在: 循环写满整帧, EAGAIN 时用 poll 等可写; 写不满就 flush 掉残留并明确报错,
//   绝不把半帧当成功返回。
int usart_send(usart_t *usart, const unsigned char *data, int len) {
    if (usart == NULL || !usart->is_open || usart->fd < 0) {
        printf("[USART] ❌ 发送失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL || len <= 0) {
        printf("[USART] ❌ 发送失败: 参数无效(data=%p, len=%d)\n",
               (const void *)data, len);
        return -1;
    }

    int total = 0;
    int64_t deadline = -1;

    while (total < len) {
        int off = total;
        int n = (int)write(usart->fd, data + off, (size_t)(len - off));
        if (n > 0) {
            total += n;
            continue;
        }

        if (n < 0 && errno == EINTR) {
            continue;                       // 被信号打断, 直接续写
        }

        // n == 0 也按"暂时写不进去"处理: 内核一个字节都没接受, 直接重试会变成忙等
        if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
            if (deadline < 0) {
                deadline = usart_now_ms() + USART_TX_TIMEOUT_MS;
            }
            int64_t left = deadline - usart_now_ms();
            if (left <= 0) {
                printf("[USART] ❌ 发送超时: %s 等可写超过 %dms (已写 %d/%d 字节)\n",
                       usart->device, USART_TX_TIMEOUT_MS, total, len);
                break;
            }
            struct pollfd pfd;
            pfd.fd = usart->fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            int pr = poll(&pfd, 1, (int)left);
            if (pr > 0) {
                continue;                   // 可写了, 回去续写
            }
            if (pr == 0) {
                printf("[USART] ❌ 发送超时: %s poll 等可写超时 (已写 %d/%d 字节)\n",
                       usart->device, total, len);
            } else if (errno != EINTR) {
                printf("[USART] ❌ 发送失败: %s poll 出错: %s\n",
                       usart->device, strerror(errno));
            } else {
                continue;
            }
            break;
        }

        // 其它 errno: 真正的发送错误
        printf("[USART] ❌ 发送失败: %s, len=%d, 已写 %d: %s\n",
               usart->device, len, total, strerror(errno));
        break;
    }

    if (total == len) {
        return total;
    }

    // 半帧不能留在 FIFO 里: 对端会把它当一帧的开头, 后面正常帧的字节全被当数据吞掉。
    // 丢掉未发完的部分, 让对端按"帧不完整"超时重传。
    tcflush(usart->fd, TCOFLUSH);
    printf("[USART] ❌ 发送不完整: %s 期望 %d 字节, 实际写出 %d 字节, 残余已丢弃\n",
           usart->device, len, total);
    return -1;
}

int usart_send_string(usart_t *usart, const char *str) {
    if (usart == NULL || str == NULL) {
        printf("[USART] ❌ 发送字符串失败: 参数为 NULL\n");
        return -1;
    }
    return usart_send(usart, (const unsigned char *)str, (int)strlen(str));
}

// 高频调用路径(接收线程循环调用), 成功时不打印, 避免刷屏; 仅真错误时打印
int usart_recv(usart_t *usart, unsigned char *buffer, int buffer_size) {
    if (usart == NULL || !usart->is_open || usart->fd < 0) {
        printf("[USART] ❌ 接收失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (buffer == NULL || buffer_size <= 0) {
        printf("[USART] ❌ 接收失败: 参数无效(buffer=%p, size=%d)\n",
               (void *)buffer, buffer_size);
        return -1;
    }

    // 二进制协议接收: 直接读满整个缓冲区, 不附加字符串终止符
    int n = read(usart->fd, buffer, buffer_size);
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        printf("[USART] ❌ 接收失败: %s: %s\n", usart->device, strerror(errno));
    }
    return n;
}

void usart_close(usart_t *usart) {
    if (usart == NULL) {
        return;
    }

    if (usart->is_open && usart->fd > 0) {
        close(usart->fd);
        usart->fd = -1;
        usart->is_open = false;
        printf("[USART] ✅ 关闭完成: %s\n", usart->device);
    }

    free(usart);
}


