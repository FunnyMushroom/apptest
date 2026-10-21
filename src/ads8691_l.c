#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "ads8691_l.h"

// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_ADS8691
#include "log.h"

// ==================== 配置 ====================
#define ADS8691_SPI_DEV   "/dev/spidev2.0"
#define ADS8691_SPI_MODE  SPI_MODE_0
#define ADS8691_SPI_SPEED 10000000          // 先 10MHz，稳定后再往上加
#define ADS8691_BITS      8

// RANGE_SEL_REG 地址 0x14，RANGE_SEL[3:0]=0000 => ±3*Vref = ±12.288V
#define ADS8691_CMD_RANGE_12V   0xD0140000

// ==================== 内部状态 ====================
static int g_fd = -1;
static uint32_t g_speed = ADS8691_SPI_SPEED;

// ==================== 底层：发 4 字节，收 4 字节 ====================
static int ads8691_xfer4_l(const uint8_t *tx, uint8_t *rx)
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;
    tr.len           = 4;
    tr.speed_hz      = g_speed;
    tr.bits_per_word = ADS8691_BITS;
    tr.cs_change     = 0;   // 消息结束由内核拉高 CS，上升沿启动下次转换

    int ret = ioctl(g_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        printf("[ADS8691] ❌ SPI 传输失败: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// ==================== 初始化 ====================
int ADS8691_Init_l(void)
{
    if (g_fd >= 0) {
        return 0;   // 已初始化
    }

    printf("[ADS8691] ▶ 初始化: %s, mode=%d, speed=%d Hz\n",
           ADS8691_SPI_DEV, ADS8691_SPI_MODE, ADS8691_SPI_SPEED);

    g_fd = open(ADS8691_SPI_DEV, O_RDWR);
    if (g_fd < 0) {
        printf("[ADS8691] ❌ 打开 %s 失败: %s\n", ADS8691_SPI_DEV, strerror(errno));
        return -1;
    }

    uint8_t  mode  = ADS8691_SPI_MODE;
    uint8_t  bits  = ADS8691_BITS;
    uint32_t speed = ADS8691_SPI_SPEED;

    if (ioctl(g_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(g_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(g_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        printf("[ADS8691] ❌ 配置 SPI 失败: %s\n", strerror(errno));
        close(g_fd);
        g_fd = -1;
        return -1;
    }
    g_speed = speed;

    printf("[ADS8691]   SPI 就绪: fd=%d, mode=%d, bits=%d, speed=%u Hz\n",
           g_fd, mode, bits, g_speed);

    // ---- 配置量程 ±12.288V ----
    uint8_t tx[4] = {
        (ADS8691_CMD_RANGE_12V >> 24) & 0xFF,
        (ADS8691_CMD_RANGE_12V >> 16) & 0xFF,
        (ADS8691_CMD_RANGE_12V >> 8)  & 0xFF,
        (ADS8691_CMD_RANGE_12V)       & 0xFF,
    };
    uint8_t rx[4] = {0};

    if (ads8691_xfer4_l(tx, rx) < 0) {
        printf("[ADS8691] ❌ 写量程失败\n");
        return -1;
    }
    printf("[ADS8691]   量程已写: ±12.288V (0x%08X)\n", ADS8691_CMD_RANGE_12V);

    printf("[ADS8691] ✅ 初始化完成\n");
    return 0;
}

// ==================== 读取一次（18 位） ====================
uint32_t ADS8691_Read_l(void)
{
    if (g_fd < 0) {
        if (ADS8691_Init_l() < 0) {
            printf("[ADS8691] ❌ 读取失败: SPI 未初始化\n");
            return 0xFFFFFFFF;
        }
    }

    // 等上一次转换完成：ADS8691 tCONV = 665ns，留 2us 余量
    usleep(2);

    uint8_t tx[4] = {0, 0, 0, 0};   // NOP
    uint8_t rx[4] = {0};

    if (ads8691_xfer4_l(tx, rx) < 0) {
        printf("[ADS8691] ❌ 读取失败: SPI 传输出错\n");
        return 0xFFFFFFFF;
    }
    // CS 已被内核拉高，此刻启动下一次转换

    uint32_t raw = ((uint32_t)rx[0] << 24) |
                   ((uint32_t)rx[1] << 16) |
                   ((uint32_t)rx[2] << 8)  |
                   (uint32_t)rx[3];

    uint32_t adc = (raw >> 14) & 0x3FFFF;

    // 与 ads8691.c 的 ADS8691_Read 一致: 每次读取都打印会把定时采样的串口刷爆。
    // 本函数目前全工程未被调用, 一并注释掉, 免得以后启用时踩同一个坑。
    // printf("[ADS8691] raw=0x%08X (%u), adc=0x%05X (%u)\n",
    //        raw, raw, adc, adc);

    return adc;
}