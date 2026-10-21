// usleep 需要特性宏(必须在包含任何头文件之前定义), 与 gcc 默认 -std=gnu17 行为一致
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "dac8562.h"
#include "spi.h"
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_DAC8562
#include "log.h"

static spi_t *spi = NULL;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static int g_inited = 0;   // 初始化序列是否已执行

// ==================== 错误码 ====================
#define DAC8562_OK            0x0000
#define DAC8562_ERR_SPI_OPEN  0xFFF0   // SPI 打开失败
#define DAC8562_ERR_SPI_SEND  0xFFF1   // SPI 发送失败
#define DAC8562_ERR_INIT      0xFFF2   // 初始化失败 / 参数非法

// ==================== 核心发送 ====================
// 返回: 0=成功, 0xFFF0=SPI未初始化, 0xFFF1=SPI发送失败
static uint16_t dac8562_write(uint8_t cmd, uint16_t data) {
    uint8_t buf[3] = {cmd, data >> 8, data & 0xFF};
    
    pthread_mutex_lock(&lock);
    
    if (spi == NULL) {
        printf("[DAC8562]   首次写入, 打开 SPI %s ...\n", DAC8562_SPI_DEVICE);
        spi = spi_init(DAC8562_SPI_DEVICE, DAC8562_SPI_MODE, DAC8562_SPI_SPEED);
        if (spi == NULL) {
            pthread_mutex_unlock(&lock);
            printf("[DAC8562] ❌ 写入失败: SPI %s 打开失败 (cmd=0x%02X)\n",
                   DAC8562_SPI_DEVICE, cmd);
            return 0xFFF0;  // SPI初始化失败
        }
    }
    
    int ret = spi_send(spi, buf, 3);
    pthread_mutex_unlock(&lock);
    
    if (ret != 3) {
        printf("[DAC8562] ❌ 写入失败: cmd=0x%02X data=0x%04X (spi_send 返回 %d)\n",
               cmd, data, ret);
        return 0xFFF1;  // SPI发送失败
    }
    
    return 0;  // 成功
}

// 以 24 位帧发送: 高字节为命令, 低 16 位为数据
static uint16_t dac8562_write_frame(uint32_t frame24) {
    return dac8562_write((uint8_t)((frame24 >> 16) & 0xFF),
                         (uint16_t)(frame24 & 0xFFFF));
}

// 对外: 直接发一帧(3 字节数组), 方便其它模块扩展
uint16_t DAC8562_Write(uint8_t *CMD) {
    if (CMD == NULL) {
        printf("[DAC8562] ❌ 写帧失败: 入参为 NULL\n");
        return DAC8562_ERR_SPI_SEND;
    }
    printf("[DAC8562] ▶ 写帧: %02X %02X %02X\n", CMD[0], CMD[1], CMD[2]);
    return dac8562_write(CMD[0], (uint16_t)((CMD[1] << 8) | CMD[2]));
}

// ==================== 初始化 ====================
// 按指定顺序下发初始化命令(每条为 24 位帧, 高字节在前):
//   0x280001 -> 0x200003 -> 0x380001 -> 0x020003 -> 0x300003 -> 0x1F0000
// 返回: 0 = 成功; 其它 = 错误码
uint16_t DAC8562_Init(void) {
    static const uint32_t init_seq[] = {
        0x280001,
        0x200003,
        0x380001,
        0x020003,
        0x300003,
        0x1F0000,
    };
    const int step_total = (int)(sizeof(init_seq) / sizeof(init_seq[0]));

    printf("[DAC8562] ▶ 初始化开始, 共 %d 条命令\n", step_total);

    for (int i = 0; i < step_total; i++) {
        uint16_t ret = dac8562_write_frame(init_seq[i]);
        if (ret != DAC8562_OK) {
            printf("[DAC8562] ❌ 初始化失败: 第 %d/%d 步 0x%06X (err=0x%04X)\n",
                   i + 1, step_total, (unsigned)init_seq[i], ret);
            return DAC8562_ERR_INIT;
        }
        printf("[DAC8562]   步骤 %d/%d: 0x%06X 完成\n",
               i + 1, step_total, (unsigned)init_seq[i]);

        if (i + 1 < step_total) {
            usleep(DAC8562_STEP_DELAY_US);   // 帧间留一点间隔, 保证内部寄存器锁存
        }
    }

    // 内部参考使能后需要一点稳定时间
    printf("[DAC8562]   等待内部参考稳定 %d ms ...\n", DAC8562_REF_SETTLE_MS);
    usleep(DAC8562_REF_SETTLE_MS * 1000);

    g_inited = 1;
    printf("[DAC8562] ✅ 初始化完成: 共 %d 条命令全部成功\n", step_total);
    return DAC8562_OK;
}

// ==================== 设置 A 通道输出 ====================
// code: 16 位 DAC 码值 (0 ~ 65535)
// 返回: 0 = 成功; 其它 = 错误码
uint16_t DAC8562_Set_Output_A(uint16_t code) {
    // 防护: 未初始化就设置输出时先自动初始化一次
    if (!g_inited) {
        printf("[DAC8562] ⚠️ 尚未初始化, 先自动执行初始化\n");
        uint16_t ret = DAC8562_Init();
        if (ret != DAC8562_OK) {
            return ret;
        }
    }

    // 命令 0x18: 写输入寄存器 A 并立即更新 A 通道输出
    uint16_t ret = dac8562_write(DAC8562_CMD_UPDATE_A, code);
    if (ret != DAC8562_OK) {
        printf("[DAC8562] ❌ 设置 A 通道失败: code=%u (err=0x%04X)\n", code, ret);
        return ret;
    }

    printf("[DAC8562] ✅ A 通道输出: code=%u (%.3f V)\n",
           code, (double)code / 65535.0 * DAC8562_A_FULL_SCALE_VOLTS);
    return DAC8562_OK;
}

// 按电压设置 A 通道输出(满量程见 DAC8562_A_FULL_SCALE_VOLTS, 超范围会限幅)
uint16_t DAC8562_Set_Output_A_Voltage(float volts) {
    printf("[DAC8562] ▶ 设置 A 通道电压: %.3f V\n", (double)volts);

    if (volts < 0.0f) {
        printf("[DAC8562] ❌ 电压 %.3fV 非法(不能为负)\n", (double)volts);
        return DAC8562_ERR_INIT;
    }
    if (volts > DAC8562_A_FULL_SCALE_VOLTS) {
        printf("[DAC8562] ⚠️ 电压 %.3fV 超出满量程 %.3fV, 按满量程输出\n",
               (double)volts, (double)DAC8562_A_FULL_SCALE_VOLTS);
        volts = DAC8562_A_FULL_SCALE_VOLTS;
    }

    uint32_t code = (uint32_t)((volts / DAC8562_A_FULL_SCALE_VOLTS) * 65535.0f + 0.5f);
    if (code > 65535u) {
        code = 65535u;
    }

    return DAC8562_Set_Output_A((uint16_t)code);
}

// 兼容旧调用名
uint16_t DAC8562_Set_A(uint16_t code) {
    return DAC8562_Set_Output_A(code);
}

// 设置增益 (命令 0x28: 0=双02倍, 1=双1倍, 2=A2倍/B1倍, 3=A1倍/B2倍)
// 增益档位 → 实际倍率(便于看日志时直接对应)
static const char *dac8562_gain_desc(uint16_t gain) {
    switch (gain) {
        case 0:  return "A/B 均为 2 倍";
        case 1:  return "A/B 均为 1 倍";
        case 2:  return "A 为 2 倍 / B 为 1 倍";
        default: return "A 为 1 倍 / B 为 2 倍";
    }
}

uint16_t DAC8562_Set_Gain(uint16_t gain) {
    printf("[DAC8562] ▶ 设置增益开始: gain=%u\n", gain);

    if (gain > 3) {
        printf("[DAC8562] ❌ 增益参数非法: %u (应在 0~3)\n", gain);
        return DAC8562_ERR_INIT;
    }

    uint16_t ret = dac8562_write(DAC8562_CMD_GAIN_SETUP, gain);
    if (ret != DAC8562_OK) {
        printf("[DAC8562] ❌ 设置增益失败: gain=%u (err=0x%04X)\n", gain, ret);
        return ret;
    }

    printf("[DAC8562] ✅ 设置增益完成: %u (%s)\n", gain, dac8562_gain_desc(gain));
    return ret;
}

// ==================== 关闭 ====================
void DAC8562_Close(void) {
    pthread_mutex_lock(&lock);
    if (spi) {
        spi_close(spi);
        spi = NULL;
        g_inited = 0;
        printf("[DAC8562] ✅ 关闭完成: SPI 句柄已释放\n");
    } else {
        printf("[DAC8562]   未初始化, 无需关闭\n");
    }
    pthread_mutex_unlock(&lock);
}