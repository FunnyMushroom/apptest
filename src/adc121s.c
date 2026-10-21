#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include <stdio.h>
#include <unistd.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_ADC121
#include "log.h"

#define ADC_VREF_MV     5000
#define ADC_RESOLUTION  4096

// SPI 总线参数
#define ADC121S_SPI_DEV      "/dev/spidev2.0"
#define ADC121S_SPI_MODE     SPI_MODE1    // 必须使用 MODE1 (CPOL=0, CPHA=1)
#define ADC121S_SPI_SPEED_HZ 1000000      // 1MHz
#define ADC121S_SPI_BITS     8

static int spi_fd = -1;

int ADC121S_Init(void) {
    if (spi_fd >= 0) {
        printf("[ADC121S]   已初始化, 无需重复打开\n");
        return 0;
    }

    printf("[ADC121S] ▶ 初始化开始: %s, MODE1, %u Hz\n",
           ADC121S_SPI_DEV, (unsigned)ADC121S_SPI_SPEED_HZ);

    spi_fd = open(ADC121S_SPI_DEV, O_RDWR);
    if (spi_fd < 0) {
        printf("[ADC121S] ❌ 初始化失败: 打开 %s 失败: %s\n",
               ADC121S_SPI_DEV, strerror(errno));
        return -1;
    }
    printf("[ADC121S]   设备已打开, fd=%d\n", spi_fd);

    // 必须使用 SPI_MODE1 (CPOL=0, CPHA=1)
    uint8_t mode = ADC121S_SPI_MODE;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        printf("[ADC121S] ❌ 初始化失败: 设置 MODE1 失败: %s\n", strerror(errno));
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    uint32_t speed = ADC121S_SPI_SPEED_HZ;
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        printf("[ADC121S] ❌ 初始化失败: 设置 %u Hz 失败: %s\n",
               (unsigned)speed, strerror(errno));
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    uint8_t bits = ADC121S_SPI_BITS;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        printf("[ADC121S] ❌ 初始化失败: 设置 %u bit 位宽失败: %s\n",
               (unsigned)bits, strerror(errno));
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    printf("[ADC121S] ✅ 初始化完成: %s, MODE1, 配置速率=%u Hz, 位宽=%u\n",
           ADC121S_SPI_DEV, (unsigned)speed, (unsigned)bits);

    // ⚠️ 提示: 下面 ADC121S_Read() 的单次传输里写死了 speed_hz=40MHz,
    //          它会覆盖这里的配置速率。若读回数据异常, 先回来核查这个值。
    printf("[ADC121S] ⚠️ 注意: ADC121S_Read() 单次传输指定 40000000 Hz, 将覆盖上述配置速率\n");
    return 0;
}


uint16_t ADC121S_Read(void) {
    if (spi_fd < 0) {
        if (ADC121S_Init() < 0) {
            printf("[ADC121S] ❌ 读取失败: SPI 未初始化\n");
            return 0xFFFF;
        }
    }
    
    uint8_t tx_buf[2] = {0x00, 0x00};
    uint8_t rx_buf[2] = {0};
    
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = 2,
        .speed_hz = 10000000,   // ⚠️ 写死 10MHz, 会覆盖 ADC121S_Init 里的速率
        .bits_per_word = 8,
        .delay_usecs = 0,
        .cs_change = 0,
    };
 
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        printf("[ADC121S] ❌ 读取失败: SPI 传输出错: %s\n", strerror(errno));
        return 0xFFFF;
    }
    
    uint8_t status = rx_buf[0] & 0x1f;  // 高4位是状态位
    
    // 直接返回原始16位数据(含3个引导零)
    uint16_t raw = (status<< 8) | rx_buf[1];
printf("[ADC121S]   读取完成: raw=%u, 状态=%u, 电压=%.3fV\n", raw, status, (float)(raw & 0x0FFF) * 5.0f / 4095.0f);
    return raw;
}

uint16_t ADC121S_Read_Multi(uint16_t *buf, uint8_t count) {
    if (buf == NULL || count == 0) {
        printf("[ADC121S] ❌ 批量读取失败: 参数无效(buf=%p, count=%u)\n",
               (void *)buf, (unsigned)count);
        return 0xFFF1;
    }

    printf("[ADC121S] ▶ 批量读取开始: 共 %u 个采样点\n", (unsigned)count);

    for (int i = 0; i < count; i++) {
        buf[i] = ADC121S_Read();
        if (buf[i] == 0xFFFF) {
            printf("[ADC121S] ❌ 批量读取中断: 第 %d/%u 个采样点失败\n",
                   i + 1, (unsigned)count);
            return 0xFFF0;
        }
    }

    printf("[ADC121S] ✅ 批量读取完成: 共 %u 个采样点, 首值=%u, 末值=%u\n",
           (unsigned)count, buf[0], buf[count - 1]);
    return 0;
}


void ADC121S_Close(void) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
        printf("[ADC121S] ✅ 关闭完成\n");
    }
}
