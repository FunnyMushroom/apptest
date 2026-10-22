#include "spi.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_SPI
#include "log.h"

spi_t *spi_init(const char *device, int mode, int speed_hz) {
    if (device == NULL) {
        printf("[SPI] ❌ 初始化失败: 设备路径为 NULL\n");
        return NULL;
    }

    printf("[SPI] ▶ 初始化开始: 设备=%s, mode=%d, speed=%d Hz\n",
           device, mode, speed_hz);

    spi_t *spi = (spi_t *)calloc(1, sizeof(spi_t));
    if (spi == NULL) {
        printf("[SPI] ❌ 初始化失败: 分配 spi_t 内存失败: %s\n", strerror(errno));
        return NULL;
    }

    strncpy(spi->device, device, sizeof(spi->device) - 1);
    spi->device[sizeof(spi->device) - 1] = '\0';
    spi->mode = mode;
    spi->speed_hz = speed_hz;
    spi->bits_per_word = 8;
    spi->is_open = false;

    // 检查设备文件是否存在
    if (access(device, F_OK) != 0) {
        printf("[SPI] ❌ 初始化失败: 设备节点不存在 %s\n", device);
        free(spi);
        return NULL;
    }

    // 打开 SPI 设备
    printf("[SPI]   设备节点存在, 开始打开 %s ...\n", device);
    spi->fd = open(device, O_RDWR | O_NOCTTY);
    if (spi->fd == -1) {
        printf("[SPI] ❌ 初始化失败: 打开 %s 失败: %s\n", device, strerror(errno));
        free(spi);
        return NULL;
    }
    printf("[SPI]   设备已打开, fd=%d\n", spi->fd);

    // 设置 SPI 模式
    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &spi->mode) < 0) {
        printf("[SPI] ❌ 初始化失败: 设置 mode=%d 失败(%s): %s\n",
               spi->mode, device, strerror(errno));
        close(spi->fd);
        free(spi);
        return NULL;
    }
    printf("[SPI]   mode=%d 设置成功\n", spi->mode);

    // 设置数据位宽
    if (ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &spi->bits_per_word) < 0) {
        printf("[SPI] ❌ 初始化失败: 设置 bits_per_word=%d 失败(%s): %s\n",
               spi->bits_per_word, device, strerror(errno));
        close(spi->fd);
        free(spi);
        return NULL;
    }
    printf("[SPI]   bits_per_word=%d 设置成功\n", spi->bits_per_word);

    // 设置 SPI 速率
    if (ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi->speed_hz) < 0) {
        printf("[SPI] ❌ 初始化失败: 设置 speed=%d Hz 失败(%s): %s\n",
               spi->speed_hz, device, strerror(errno));
        close(spi->fd);
        free(spi);
        return NULL;
    }
    printf("[SPI]   speed=%d Hz 设置成功\n", spi->speed_hz);

    spi->is_open = true;
    printf("[SPI] ✅ 初始化完成: %s, fd=%d, mode=%d, speed=%d Hz, bits=%d\n",
           device, spi->fd, spi->mode, spi->speed_hz, spi->bits_per_word);
    return spi;
}

int spi_send(spi_t *spi, const unsigned char *data, int len) {
    if (spi == NULL || !spi->is_open || spi->fd < 0) {
        printf("[SPI] ❌ 发送失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL || len <= 0) {
        printf("[SPI] ❌ 发送失败: 参数无效(data=%p, len=%d)\n",
               (const void *)data, len);
        return -1;
    }

    // 动态分配接收缓冲区, 避免 VLA 栈溢出
    unsigned char *dummy = (unsigned char *)calloc(1, len);
    if (dummy == NULL) {
        printf("[SPI] ❌ 发送失败: 分配接收缓冲区失败: %s\n", strerror(errno));
        return -1;
    }

    // 片选(CS)不用用户态手动拉: 内核 SPI core 会在本消息开始前自动选中(拉低),
    // 全部字节发完后自动释放(拉高)。这里显式写 cs_change=0, 表示 len 个字节
    // 必须在同一个 CS 有效脉冲内连续发完, 中途不释放片选。
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)data,
        .rx_buf = (unsigned long)dummy,
        .len = len,
        .speed_hz = spi->speed_hz,
        .bits_per_word = spi->bits_per_word,
        .cs_change = 0,   // 0 = 消息内不释放 CS(默认值, 显式写出便于阅读)
    };

    int ret = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &tr);
    free(dummy);
    if (ret < 0) {
        printf("[SPI] ❌ 发送失败: len=%d: %s\n", len, strerror(errno));
        return -1;
    }

    printf("[SPI] ✅ 发送完成: %d 字节\n", len);
    return len;
}

int spi_transfer(spi_t *spi, const unsigned char *tx_data, int tx_len, unsigned char *rx_data, int rx_len) {
    if (spi == NULL || !spi->is_open || spi->fd < 0) {
        printf("[SPI] ❌ 双向传输失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (tx_data == NULL || tx_len <= 0) {
        printf("[SPI] ❌ 双向传输失败: 发送参数无效(tx_data=%p, tx_len=%d)\n",
               (const void *)tx_data, tx_len);
        return -1;
    }
    if (rx_data == NULL || rx_len <= 0) {
        printf("[SPI] ❌ 双向传输失败: 接收参数无效(rx_data=%p, rx_len=%d)\n",
               (void *)rx_data, rx_len);
        return -1;
    }

    int len = (tx_len < rx_len) ? tx_len : rx_len;

    // CS 由内核自动控制: 消息开始拉低选中, 收发完成后拉高释放。
    // cs_change=0 => 收发 len 字节期间 CS 一直保持有效, 不中途释放。
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = (unsigned long)rx_data,
        .len = len,
        .speed_hz = spi->speed_hz,
        .bits_per_word = spi->bits_per_word,
        .cs_change = 0,   // 0 = 消息内不释放 CS(默认值, 显式写出便于阅读)
    };

    int ret = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        printf("[SPI] ❌ 双向传输失败: len=%d: %s\n", len, strerror(errno));
        return -1;
    }

    // 高频调用路径(ADS8691_Read 等), 成功时不打印, 避免刷屏
    return len;
}

// ==================== 批量帧: 一次 ioctl 下发 n 个 SPI 消息 ====================
int spi_transfer_msgs(spi_t *spi, struct spi_ioc_transfer *tr, int n) {
    if (spi == NULL || !spi->is_open || spi->fd < 0) {
        printf("[SPI] ❌ 批量传输失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (tr == NULL || n <= 0) {
        printf("[SPI] ❌ 批量传输失败: 参数无效(tr=%p, n=%d)\n", (void *)tr, n);
        return -1;
    }

    // 未显式指定的字段在此补齐, 避免调用方漏填 bits_per_word 时被内核按 8 位默认值处理出错
    for (int i = 0; i < n; i++) {
        if (tr[i].speed_hz == 0)         tr[i].speed_hz = (uint32_t)spi->speed_hz;
        if (tr[i].bits_per_word == 0)    tr[i].bits_per_word = (uint8_t)spi->bits_per_word;
    }

    // SPI_IOC_MESSAGE(n): n 个消息在同一个 ioctl 内完成, 只有一次系统调用/一次内核入口
    int ret = ioctl(spi->fd, SPI_IOC_MESSAGE(n), tr);
    if (ret < 0) {
        printf("[SPI] ❌ 批量传输失败: n=%d: %s\n", n, strerror(errno));
        return -1;
    }
    return ret;
}

int spi_recv(spi_t *spi, unsigned char *buffer, int len) {
    if (spi == NULL || !spi->is_open || spi->fd < 0) {
        printf("[SPI] ❌ 接收失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (buffer == NULL || len <= 0) {
        printf("[SPI] ❌ 接收失败: 参数无效(buffer=%p, len=%d)\n",
               (void *)buffer, len);
        return -1;
    }

    // 动态分配发送缓冲区(全0xFF驱动时钟), 避免 VLA 栈溢出
    unsigned char *tx_data = (unsigned char *)malloc(len);
    if (tx_data == NULL) {
        printf("[SPI] ❌ 接收失败: 分配发送缓冲区失败: %s\n", strerror(errno));
        return -1;
    }
    memset(tx_data, 0xFF, len);

    // CS 由内核自动控制: 消息开始拉低选中, 读完后拉高释放。
    // cs_change=0 => 读 len 字节期间 CS 一直保持有效, 不中途释放。
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_data,
        .rx_buf = (unsigned long)buffer,
        .len = len,
        .speed_hz = spi->speed_hz,
        .bits_per_word = spi->bits_per_word,
        .cs_change = 0,   // 0 = 消息内不释放 CS(默认值, 显式写出便于阅读)
    };

    int ret = ioctl(spi->fd, SPI_IOC_MESSAGE(1), &tr);
    free(tx_data);
    if (ret < 0) {
        printf("[SPI] ❌ 接收失败: len=%d: %s\n", len, strerror(errno));
        return -1;
    }

    printf("[SPI] ✅ 接收完成: %d 字节\n", len);
    return len;
}

void spi_close(spi_t *spi) {
    if (spi == NULL) {
        return;
    }

    if (spi->is_open && spi->fd > 0) {
        close(spi->fd);
        spi->fd = -1;
        spi->is_open = false;
        printf("[SPI] ✅ 关闭完成: %s\n", spi->device);
    }

    free(spi);
}