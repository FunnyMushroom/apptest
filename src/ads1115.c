// usleep 需要特性宏(必须在包含任何头文件之前定义), 与 gcc 默认 -std=gnu17 行为一致
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "ads1115.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <i2c.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_ADS1115
#include "log.h"

// ==================== 共享 I2C 句柄（只初始化一次，其余通信直接复用） ====================
// ADS1115 始终挂在同一 I2C 总线上 (/dev/i2c-0, 0x48)，无需每次通信都 open/close。
// 这里用一个全局句柄：第一次使用时打开，之后所有函数复用同一个 fd，
// 直到进程结束或调用 ADS1115_Close() 才关闭，避免频繁打开/关闭文件描述符的开销。
// ==================== 共享 I2C 句柄（只初始化一次，其余通信直接复用） ====================
// ADS1115 始终挂在同一 I2C 总线上，无需每次通信都 open/close。
// 这里用一个全局句柄：第一次使用时打开，之后所有函数复用同一个 fd，
// 直到进程结束或调用 ADS1115_Close() 才关闭，避免频繁打开/关闭文件描述符的开销。

// 各通道量程(必须与下面 U/I 换算所用的 LSB 保持一致):
//   通道0 = I: ±0.256V, LSB = 7.8125 µV, 采样电阻 47Ω
//   通道1 = U: ±4.096V, LSB = 125  µV, 分压比 200
#define ADS1115_PGA_I   ADS1115_PGA_0_256V
#define ADS1115_PGA_U   ADS1115_PGA_4_096V

static i2c_t *g_i2c = NULL;
static pthread_mutex_t g_i2c_lock = PTHREAD_MUTEX_INITIALIZER;

// 获取全局句柄（懒加载，只打开一次，不关闭）
static i2c_t *ads1115_i2c_get(void) {
    if (g_i2c != NULL && g_i2c->is_open) {
        return g_i2c;
    }

    printf("[ADS1115] ▶ 首次访问, 打开 I2C 总线 %s, 从机地址 0x%02X ...\n",
           ADS1115_I2C_DEVICE, ADS1115_ADDR);

    g_i2c = i2c_init(ADS1115_I2C_DEVICE, ADS1115_ADDR);
    if (g_i2c == NULL) {
        printf("[ADS1115] ❌ I2C 总线打开失败: %s, 从机地址 0x%02X\n",
               ADS1115_I2C_DEVICE, ADS1115_ADDR);
    } else {
        printf("[ADS1115] ✅  总线已建立(全局复用): %s, 从机地址 0x%02X, fd=%d\n",
               g_i2c->device, g_i2c->addr, g_i2c->fd);
    }
    return g_i2c;
}

// 加锁并获取句柄（一次完整的 I2C 通信全程持锁，防止多线程同时操作同一个 fd）
static i2c_t *ads1115_i2c_lock(void) {
    pthread_mutex_lock(&g_i2c_lock);
    return ads1115_i2c_get();
}

static void ads1115_i2c_unlock(void) {
    pthread_mutex_unlock(&g_i2c_lock);
}


// 写配置寄存器
void ADS1115_Write_Config(uint16_t cmd)
{
    printf("[ADS1115] ▶ 写配置寄存器: 0x%04X\n", cmd);

    i2c_t *i2c_hv = ads1115_i2c_lock();
    if (i2c_hv == NULL) {
        ads1115_i2c_unlock();
        printf("[ADS1115] ❌ 写配置失败: I2C 句柄不可用\n");
        return;
    }

    uint8_t config[2];
    config[0] = (cmd >> 8) & 0xFF;   // 高字节
    config[1] = cmd & 0xFF;          // 低字节

    int ret = i2c_write_bytes(i2c_hv, 0x01, config, 2);
    ads1115_i2c_unlock();

    if (ret != 0) {
        printf("[ADS1115] ❌ 写配置失败: cmd=0x%04X\n", cmd);
        return;
    }
    printf("[ADS1115] ✅ 写配置完成: 0x%04X\n", cmd);
}

// 读取转换结果
void ADS1115_Read_Conversion(uint16_t *data)
{
    if (data == NULL) {
        printf("[ADS1115] ❌ 读转换结果失败: 输出指针为 NULL\n");
        return;
    }

    i2c_t *i2c_hv = ads1115_i2c_lock();
    if (i2c_hv == NULL) {
        ads1115_i2c_unlock();
        printf("[ADS1115] ❌ 读转换结果失败: I2C 句柄不可用\n");
        *data = 0xFFFF;
        return;
    }

    uint8_t raw[2] = { 0, 0 };

    int ret = i2c_read_bytes(i2c_hv, 0x00, raw, 2);
    ads1115_i2c_unlock();

    if (ret != 0) {
        printf("[ADS1115] ❌ 读转换结果失败: 读寄存器 0x00 出错\n");
        *data = 0xFFFF;
        return;
    }

    *data = (uint16_t)((raw[0] << 8) | raw[1]);
    printf("[ADS1115] ✅ 转换结果: raw=0x%04X (%u)\n", *data, *data);
}

// 显式关闭共享 I2C 句柄（进程退出前调用；之后如需再次使用会重新打开）
void ADS1115_Close(void)
{
    pthread_mutex_lock(&g_i2c_lock);
    if (g_i2c != NULL) {
        i2c_close(g_i2c);
        g_i2c = NULL;
        printf("[ADS1115] ✅ 关闭完成: 共享 I2C 句柄已释放\n");
    }
    pthread_mutex_unlock(&g_i2c_lock);
}


// ==================== 单通道读取(核心) ====================
// ADS1115 配置寄存器位定义:
//   bit15   OS     : 写 1 启动一次转换; 读回 1 表示转换已完成
//   bit14-12 MUX   : 100 = AIN0 对 GND(通道0)   101 = AIN1 对 GND(通道1)
//   bit11-9 PGA    : 增益
//   bit8    MODE   : 1 = 单次转换
//   bit7-5  DR     : 采样率
//   bit4-0  比较器 : 关闭
// 流程: 写配置(启动转换) -> 回读校验 MUX -> 延时 100ms 等待转换完成 -> 读转换结果
// 返回: 0 成功; -1 失败(此时 *value = 0xFFFF)
// 注意: 调用者必须已持有 I2C 锁
static int ads1115_read_channel_locked(i2c_t *h, uint16_t mux, uint16_t pga, uint16_t *value)
{
    if (h == NULL || value == NULL) {
        printf("[ADS1115] ❌ 单通道读取失败: 参数无效(h=%p, value=%p)\n",
               (void *)h, (void *)value);
        return -1;
    }
    *value = 0xFFFF;

    uint16_t cfg = (uint16_t)(0x8000 | mux | pga |
                              0x0100 | 0x0080 |
                              0x0003);
    uint8_t w[2] = { (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    uint8_t r[2] = { 0, 0 };

    // 1) 写配置寄存器(OS=1, 立即启动一次转换)
    if (i2c_write_bytes(h, 0x01, w, 2) != 0) {
        printf("[ADS1115] ❌ 写配置失败: mux=0x%04X, cfg=0x%04X\n", mux, cfg);
        return -1;
    }
    printf("[ADS1115]   已启动转换: mux=0x%04X, cfg=0x%04X\n", mux, cfg);

    // 2) 回读配置, 确认 MUX 位真的写进去了。
    //    若不校验: I2C 写失败时器件仍停留在上一个通道, 表现就是"通道0和通道1读到同一个值"。
    if (i2c_read_bytes(h, 0x01, r, 2) == 0) {
        uint16_t back = (uint16_t)((r[0] << 8) | r[1]);
        if ((back & 0x7000) != (mux & 0x7000)) {
            printf("[ADS1115] ⚠️ MUX 未生效: 期望 0x%04X 实际 0x%04X\n",
                   (unsigned)(mux & 0x7000), (unsigned)(back & 0x7000));
        }
    } else {
        printf("[ADS1115] ⚠️ 配置寄存器回读失败, 无法校验 MUX\n");
    }

    // 3) 启动转换后固定延时 100ms, 等待本次转换完成后再读取结果
    usleep(100 * 1000);

    // 4) 读转换结果(高字节在前, 有符号数按 uint16 原样返回)
    if (i2c_read_bytes(h, 0x00, r, 2) != 0) {
        printf("[ADS1115] ❌ 读转换结果失败: mux=0x%04X, 读寄存器 0x00 出错\n", mux);
        return -1;
    }

    *value = (uint16_t)((r[0] << 8) | r[1]);
    printf("[ADS1115]   mux=0x%04X 采样值 = %u (0x%04X)\n", mux, *value, *value);
    return 0;
}

// 读取任意通道(自动加锁)
//   mux: 0x4000(AIN0 对 GND) / 0x5000(AIN1 对 GND) ...
//   pga: 0x0200(±4.096V) 等
int ADS1115_Read_Channel(uint16_t mux, uint16_t pga, uint16_t *value)
{
    printf("[ADS1115] ▶ 读通道开始: mux=0x%04X, pga=0x%04X\n", mux, pga);

    i2c_t *h = ads1115_i2c_lock();
    if (h == NULL) {
        ads1115_i2c_unlock();
        printf("[ADS1115] ❌ 读通道失败: I2C 句柄不可用\n");
        return -1;
    }

    int ret = ads1115_read_channel_locked(h, mux, pga, value);
    ads1115_i2c_unlock();

    if (ret != 0) {
        printf("[ADS1115] ❌ 读通道失败: mux=0x%04X\n", mux);
    }
    return ret;
}
uint16_t ADS1115_Read_ADC_I(void)
{
    uint16_t v = 0xFFFF;

    if (ADS1115_Read_Channel(ADS1115_CH_I_MUX, ADS1115_PGA_I, &v) != 0) {
        printf("[ADS1115_I] ❌ 通道0(AIN0)读取失败\n");
        return 0xFFFF;
    }

    int16_t raw = (int16_t)v;
    int32_t uv  = ((int32_t)raw * 125) / 16;   // 引脚电压 µV，LSB=7.8125µV (±0.256V)
    int32_t ua  = uv / 47;                     // 电流 µA，采样电阻 47Ω

    printf("[ADS1115_I] ✅ 通道0(AIN0)电流: raw=%d, 引脚=%ld.%03ld mV, 电流=%ld uA\n",
           raw,
           (long)(uv / 1000), (long)labs(uv % 1000),
           (long)ua);
    return v;
}

// 通道1 = U (AIN1 对 GND)，量程 ±4.096V，LSB=125µV
uint16_t ADS1115_Read_ADC_U(void)
{
    uint16_t v = 0xFFFF;

    if (ADS1115_Read_Channel(ADS1115_CH_U_MUX, ADS1115_PGA_U, &v) != 0) {
        printf("[ADS1115_U] ❌ 通道1(AIN1)读取失败\n");
        return 0xFFFF;
    }

    int16_t raw   = (int16_t)v;
    int32_t uv    = (int32_t)raw * 125;    // 引脚电压 µV，LSB=125µV (±4.096V)
    int32_t hv_uv = uv * 501;              // HV = 引脚电压 × 501（与 DAC 一致）
    int32_t hv_mv = hv_uv / 1000;          // HV mV

    printf("[ADS1115_U] ✅ 通道1(AIN1)电压: raw=%d, 引脚=%ld.%03ld mV, HV=%ld.%03ld V\n",
           raw,
           (long)(uv / 1000), (long)labs(uv % 1000),
           (long)(hv_mv / 1000), (long)labs(hv_mv % 1000));
    return v;
}

// 一次持锁内读两个通道: I = 通道0(AIN0), U = 通道1(AIN1)
// 全程持锁, 避免两次读取之间被其它线程插入导致读错通道
void ADS1115_Read_ADC(uint16_t *I, uint16_t *U)
{
    printf("[ADS1115] ▶ 双通道读取开始: I=通道0(AIN0), U=通道1(AIN1)\n");

    i2c_t *h = ads1115_i2c_lock();
    if (h == NULL) {
        ads1115_i2c_unlock();
        printf("[ADS1115] ❌ 双通道读取失败: I2C 句柄不可用\n");
        if (I) *I = 0xFFFF;
        if (U) *U = 0xFFFF;
        return;
    }

    uint16_t v_i = 0xFFFF;
    uint16_t v_u = 0xFFFF;

    // ⚠️ 量程必须与 ADS1115_Read_ADC_I / ADS1115_Read_ADC_U 保持一致,
    //     否则同一路信号在这里和单独读取时会得到不同的码值与不同的物理量换算
    int r_i = ads1115_read_channel_locked(h, ADS1115_CH_I_MUX, ADS1115_PGA_I, &v_i);   // 通道0 = I
    int r_u = ads1115_read_channel_locked(h, ADS1115_CH_U_MUX, ADS1115_PGA_U, &v_u);   // 通道1 = U

    ads1115_i2c_unlock();

    if (I) *I = v_i;
    if (U) *U = v_u;

    if (r_i != 0 || r_u != 0) {
        printf("[ADS1115] ❌ 双通道读取部分失败: I=%s, U=%s\n",
               (r_i == 0) ? "OK" : "失败", (r_u == 0) ? "OK" : "失败");
        return;
    }

    // I: LSB = 7.8125µV, 采样电阻 47Ω
    int32_t i_uv = ((int32_t)(int16_t)v_i * 125) / 16;
    int32_t i_ua = i_uv / 47;
    // U: LSB = 125µV, 分压比 200
    int32_t u_uv = (int32_t)(int16_t)v_u * 125;
    int32_t u_hv_mv = (u_uv * 200) / 1000;

    printf("[ADS1115] ✅ 双通道完成: I(通道0)=%ld uA (raw=%u), U(通道1)=%ld.%03ld V (raw=%u)\n",
           (long)i_ua, v_i,
           (long)(u_hv_mv / 1000), (long)abs(u_hv_mv % 1000), v_u);
}

// 一次持锁内读取全部四个通道(均为单端输入: AIN0/AIN1/AIN2/AIN3 对 GND)
//   values: 调用者提供的数组, 至少 4 个元素; values[0..3] 依次对应通道 0~3 的原始值
//           读取失败的通道会被填入 0xFFFF
//   pga   : 增益, 如 0x0200(±4.096V)、0x0400(±2.048V) ...
// 返回: 成功读取的通道数(0~4); 参数或 I2C 打开失败返回 -1
// 说明: 全程只加一次锁, 四个通道在同一持锁区间内依次采样,
//       避免其它线程插入导致通道值互相串扰
int ADS1115_Read_All_Channels(uint16_t *values, uint16_t pga)
{
    if (values == NULL) {
        printf("[ADS1115] ❌ 四通道读取失败: 输出缓冲区为空\n");
        return -1;
    }

    printf("[ADS1115] ▶ 四通道读取开始: AIN0~AIN3, pga=0x%04X\n", pga);

    // MUX 依次对应 AIN0/AIN1/AIN2/AIN3 对 GND 的配置寄存器位
    static const uint16_t mux[4] = { 0x4000, 0x5000, 0x6000, 0x7000 };

    i2c_t *h = ads1115_i2c_lock();
    if (h == NULL) {
        ads1115_i2c_unlock();
        printf("[ADS1115] ❌ 四通道读取失败: I2C 句柄不可用\n");
        return -1;
    }

    int ok = 0;
    for (int ch = 0; ch < 4; ch++) {
        values[ch] = 0xFFFF;
        if (ads1115_read_channel_locked(h, mux[ch], pga, &values[ch]) == 0) {
            ok++;
        } else {
            printf("[ADS1115] ⚠️ 通道 %d (mux=0x%04X) 读取失败\n", ch, mux[ch]);
        }
    }

    ads1115_i2c_unlock();

    if (ok != 4) {
        printf("[ADS1115] ❌ 四通道读取不完整: AIN0=%u, AIN1=%u, AIN2=%u, AIN3=%u (成功 %d/4)\n",
               values[0], values[1], values[2], values[3], ok);
        return ok;
    }

    printf("[ADS1115] ✅ 四通道读取完成: AIN0=%u, AIN1=%u, AIN2=%u, AIN3=%u (成功 %d/4)\n",
           values[0], values[1], values[2], values[3], ok);
    return ok;
}

// 初始化: 让器件进入已知状态(单次模式 + 比较器关闭), 并顺带自检 I2C 通路
int ADS1115_Init(void)
{
    printf("[ADS1115] ▶ 初始化开始: %s, 从机地址 0x%02X\n",
           ADS1115_I2C_DEVICE, ADS1115_ADDR);

    i2c_t *h = ads1115_i2c_lock();
    if (h == NULL) {
        ads1115_i2c_unlock();
        printf("[ADS1115] ❌ 初始化失败: I2C 总线不可用\n");
        return -1;
    }

    uint16_t cfg = (uint16_t)(0x8000 | 0x4000 |
                              0x0200 | 0x0100 |
                              0x0080 | 0x0003);
    uint8_t w[2] = { (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };

    int ret = i2c_write_bytes(h, 0x01, w, 2);
    ads1115_i2c_unlock();

    if (ret != 0) {
        printf("[ADS1115] ❌ 初始化失败: 写配置 0x%04X 出错\n", cfg);
        return -1;
    }

    printf("[ADS1115] ✅ 初始化完成: 配置 = 0x%04X (单次模式/±4.096V/128SPS)\n", cfg);
    return 0;
}


// 等价于裸机 main.c 的 read_adc()
// 读通道0→i，通道1→v，算出 current_ua 和 output_mv
// 返回 0 成功，-1 失败

int ADS1115_Read_ADC_Physical(int32_t *current_ua, int32_t *output_mv)
{
    uint16_t raw_i = 0xFFFF, raw_v = 0xFFFF;

    printf("[ADS1115] ▶ 读取物理量开始: 通道0=电流(I), 通道1=电压(U)\n");

    // 一次持锁读两个通道（用你现有的 ADS1115_Read_ADC）
    ADS1115_Read_ADC(&raw_i, &raw_v);

    // 失败判断：裸机是任一读失败就返回
    if (raw_i == 0xFFFF || raw_v == 0xFFFF) {
        printf("[ADS1115] ❌ 物理量读取失败: raw_i=0x%04X, raw_v=0x%04X\n",
               raw_i, raw_v);
        return -1;
    }

    // uint16 → int16（关键：负数必须转有符号）
    int16_t i = (int16_t)raw_i;
    int16_t v = (int16_t)raw_v;

    // ⚠️ 注意: ads1115_uv() 按 LSB=62.5µV (对应 ±2.048V 量程) 换算,
    //          而 ADS1115_Read_ADC() 配置的是 ±0.256V / ±4.096V,
    //          两者不一致会导致算出的物理量偏小。标定时需确认此处量程。
    if (current_ua) *current_ua = ads1115_uv(i) / 47;
    if (output_mv)  *output_mv  = (ads1115_uv(v) * 501) / 1000;

    printf("[ADS1115] ✅ 物理量: raw_i=%d, raw_v=%d, 电流=%ld uA, 电压=%ld mV\n",
           i, v, (long)(current_ua ? *current_ua : 0),
           (long)(output_mv ? *output_mv : 0));
    return 0;
}

// raw → 微伏（与裸机 ads1115_uv 完全一致，LSB=62.5µV）
// ⚠️ 本函数的 LSB 与 ADS1115_Read_ADC_I()/U() 的 7.8125µV / 125µV 不同,
//    只适用于 ±2.048V 量程的采样值, 不要拿它换算其它量程的 raw。
int32_t ads1115_uv(int16_t raw)
{
    return ((int32_t)raw * 125) / 2;
}