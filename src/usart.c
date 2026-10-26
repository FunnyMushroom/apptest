#include "usart.h"
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

// 高频调用路径(每个命令都会发), 成功时不打印, 避免刷屏; 仅失败时打印
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

    int n = write(usart->fd, data, len);
    if (n < 0) {
        printf("[USART] ❌ 发送失败: %s, len=%d: %s\n",
               usart->device, len, strerror(errno));
    }
    return n;
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


