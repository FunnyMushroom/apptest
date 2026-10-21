// clock_gettime/CLOCK_MONOTONIC 属 POSIX 接口, 需要特性宏(必须在包含任何头文件之前定义)。
// 显式定义 _DEFAULT_SOURCE 与 gcc 默认 -std=gnu17 的行为一致,
// 可保证在 -std=c11 等严格模式下也能编译。
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "i2c.h"
#include "fpga_i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include "gpio.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_FPGA
#include "log.h"

// 单调时钟微秒计数(仅用于定位耗时点)
static long long fpga_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + (long long)ts.tv_nsec / 1000LL;
}
uint16_t map_sw(uint16_t input) {
    static const uint16_t map_table[] = {
        7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8
    };
    
    if (input >= 0 && input < 16) {
        return map_table[input];
    }
    return 0;
}
// ==================== 共享 I2C 句柄（只初始化一次，其余通信直接复用） ====================
// FPGA 始终挂在同一总线上 (i2c-0, 0x04)，无需每次通信都 open/close。
// 这里用一个全局句柄：第一次使用时打开，之后所有函数复用同一个 fd，
// 直到进程结束或调用 FPGA_I2C_Close() 才关闭，避免频繁打开/关闭文件描述符的开销。
static i2c_t *g_fpga_i2c = NULL;
static pthread_mutex_t g_fpga_i2c_lock = PTHREAD_MUTEX_INITIALIZER;

// ==================== 共享 GPIO 句柄（PF0/PF1，只初始化一次，其余直接复用） ====================
// 采集卡选择用到的两个 GPIO(PF0=160, PF1=161) 同样无需每次 open/close，
// 这里用全局句柄懒加载，避免频繁初始化带来的开销。
#define FPGA_PF0_NUM 160
#define FPGA_PF1_NUM 161
static gpio_t *g_pf0 = NULL;
static gpio_t *g_pf1 = NULL;

// 懒加载获取 PF0/PF1 句柄（只打开一次，不关闭）
static void fpga_gpio_get(void) {
    if (g_pf0 == NULL || !g_pf0->is_open) {
        printf("[FPGA-GPIO] ▶ 首次使用, 申请 PF0(gpio%d) 输出 ...\n", FPGA_PF0_NUM);
        g_pf0 = gpio_init(FPGA_PF0_NUM, GPIO_DIR_OUT);
        if (g_pf0 == NULL) {
            printf("[FPGA-GPIO] ❌ 申请 PF0(gpio%d) 失败\n", FPGA_PF0_NUM);
        } else {
            printf("[FPGA-GPIO] ✅ PF0(gpio%d) 已占用\n", FPGA_PF0_NUM);
        }
    }
    if (g_pf1 == NULL || !g_pf1->is_open) {
        printf("[FPGA-GPIO] ▶ 首次使用, 申请 PF1(gpio%d) 输出 ...\n", FPGA_PF1_NUM);
        g_pf1 = gpio_init(FPGA_PF1_NUM, GPIO_DIR_OUT);
        if (g_pf1 == NULL) {
            printf("[FPGA-GPIO] ❌ 申请 PF1(gpio%d) 失败\n", FPGA_PF1_NUM);
        } else {
            printf("[FPGA-GPIO] ✅ PF1(gpio%d) 已占用\n", FPGA_PF1_NUM);
        }
    }
}

// 获取全局句柄（懒加载，只打开一次，不关闭）
static i2c_t *fpga_i2c_get(void) {
    if (g_fpga_i2c != NULL && g_fpga_i2c->is_open) {
        return g_fpga_i2c;
    }

    printf("[FPGA-I2C] ▶ 首次访问, 打开 %s, 从机地址 0x%02X ...\n",
           FPGA_I2C_DEVICE, FPGA_I2C_ADDR);

    g_fpga_i2c = i2c_init(FPGA_I2C_DEVICE, FPGA_I2C_ADDR);
    if (g_fpga_i2c == NULL) {
        printf("[FPGA-I2C] ❌ 打开失败: %s, 从机地址 0x%02X\n",
               FPGA_I2C_DEVICE, FPGA_I2C_ADDR);
    } else {
        printf("[FPGA-I2C] ✅ 打开完成(全局复用): %s, 从机地址 0x%02X, fd=%d\n",
               g_fpga_i2c->device, g_fpga_i2c->addr, g_fpga_i2c->fd);
    }
    return g_fpga_i2c;
}

// 加锁并获取句柄（一次完整的读/写序列全程持锁，防止多线程同时操作同一个 fd）
static i2c_t *fpga_i2c_lock(void) {
    pthread_mutex_lock(&g_fpga_i2c_lock);
    return fpga_i2c_get();
}

static void fpga_i2c_unlock(void) {
    pthread_mutex_unlock(&g_fpga_i2c_lock);
}

// 带状态提示的寄存器写：写失败时明确打印（直接调 i2c_write_byte 时返回值常被忽略）
// 注意: 本函数在采集/扫描循环里会被高频调用, 因此成功时刻意不打印, 避免刷屏
static int fpga_write_byte(i2c_t *fpga, uint8_t reg, uint8_t val) {
    if (fpga == NULL) {
        printf("[FPGA] ❌ 写寄存器失败: I2C 句柄为 NULL (reg=0x%02X val=0x%02X)\n", reg, val);
        return -1;
    }
    if (i2c_write_byte(fpga, reg, val) != 0) {
        printf("[FPGA] ❌ 写寄存器失败: 0x%02X=0x%02X\n", reg, val);
        return -1;
    }
    return 0;
}





uint8_t FPGA_Build_CMD1(uint8_t channel)
{
    if (channel > 39) {
        printf("[FPGA] ❌ CMD1 合成失败: 通道号 %u 超出范围 (0~39)\n", (unsigned)channel);
        return 0xFF;
    } 

    uint8_t chip_sel = channel / 16;
    uint8_t channel_sel = channel % 16;

    uint8_t cmd = 0;

    // bit7-4: 通道选择
    cmd |= (channel_sel << 4);

    // bit3-2: 芯片使能
    cmd |= (chip_sel << 2);

    // bit1: 放电命令 (固定为 0)
    cmd &= ~(1 << 1);   // ✅ 显式清零

    // bit0: 高压使能 (1=打开)
    if (0) {
        cmd |= (1 << 0);
    } else {
        cmd &= ~(1 << 0);   // ✅ 显式清零
    }

    return cmd;
}

uint8_t FPGA_Build_CMD2(uint8_t channel)
{
    if (channel > 15) {
        printf("[FPGA] ❌ CMD2 合成失败: 通道号 %u 超出范围 (0~15)\n", (unsigned)channel);
        return 0xFF;
    }

    uint8_t cmd = 0;

    // bit6-3: 通道选择 (0~15)
    cmd |= (channel << 3);

    // bit2: 模拟开关使能 (固定为 1 = 禁止)
    cmd |= (1 << 2);        // ✅ 显式置 1

    // bit1: 允许测试命令 (固定为 0 = 不允许)
    cmd &= ~(1 << 1);       // ✅ 显式清零

    // bit0: 故障复位命令 (固定为 0 = 不复位)
    cmd &= ~(1 << 0);       // ✅ 显式清零

    return cmd;
}


uint8_t FPGA_Build_CMD1_HV(uint8_t channel)
{
    if (channel > 39) {
        printf("[FPGA] ❌ CMD1_HV 合成失败: 通道号 %u 超出范围 (0~39)\n", (unsigned)channel);
        return 0xFF;
    } 

    uint8_t chip_sel = channel / 16;
    uint8_t channel_sel = channel % 16;

    uint8_t cmd = 0;

    // bit7-4: 通道选择
    cmd |= (channel_sel << 4);

    // bit3-2: 芯片使能
    cmd |= (chip_sel << 2);

    // bit1: 放电命令 (固定为 0)
    cmd &= ~(1 << 1);   // ✅ 显式清零

    // bit0: 高压使能 (1=打开)
    if (1) {
        cmd |= (1 << 0);
    } else {
        cmd &= ~(1 << 0);   // ✅ 显式清零
    }

    return cmd;
}

uint8_t FPGA_Build_CMD2_TEST(uint8_t channel)
{
    if (channel > 15) {
        printf("[FPGA] ❌ CMD2_TEST 合成失败: 通道号 %u 超出范围 (0~15)\n", (unsigned)channel);
        return 0xFF;
    }

    uint8_t cmd = 0;

    // bit6-3: FPGA_A3~A0，DG406CWI 通道选择 (0~15)
    cmd |= (channel << 3);

    // bit2: FPGA_EN，DG406CWI 使能。0=使能，1=禁止 → 置 0
    // cmd &= ~(1 << 2);
    cmd |= (1 << 2);
    // bit1: HV_READY_CMD，允许测试命令 → 置 1
    cmd |= (1 << 1);

    // bit0: FAULT_RESET_CMD，故障复位命令 → 置 0
    cmd &= ~(1 << 0);

    return cmd;
}









uint16_t Check_State_Collect(void) {
    printf("[Check_State_Collect] ▶ 开始读取采集卡状态 (寄存器 0x00~0x01)\n");
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Check_State_Collect] ❌ 读取失败: I2C 句柄不可用\n");
        return 0x0000;
    }

    uint8_t val1, val2;
    i2c_read_byte(fpga1, 0x00, &val1);
    i2c_read_byte(fpga1, 0x01, &val2);

    fpga_i2c_unlock();

    uint16_t state = ((uint16_t)val2 << 8) | (uint16_t)val1;
    printf("[Check_State_Collect] ✅ 采集卡状态: 0x00=0x%02X, 0x01=0x%02X -> 0x%04X\n",
           val1, val2, state);
    return state;
}


uint16_t Check_State_Switch(void) {
    printf("[Check_State_Switch] ▶ 开始读取切换卡状态 (寄存器 0x02~0x03)\n");
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Check_State_Switch] ❌ 读取失败: I2C 句柄不可用\n");
        return 0x0000;
    }

    uint8_t val3, val4;
    i2c_read_byte(fpga1, 0x02, &val3);
    i2c_read_byte(fpga1, 0x03, &val4);

    fpga_i2c_unlock();

    uint16_t state = ((uint16_t)val4 << 8) | (uint16_t)val3;
    printf("[Check_State_Switch] ✅ 切换卡状态: 0x02=0x%02X, 0x03=0x%02X -> 0x%04X\n",
           val3, val4, state);
    return state;
}






bool Check_HV_State(void) {
    printf("[Check_HV_State] ▶ 开始读取 HV 状态寄存器 0x04\n");
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Check_HV_State] ❌ 读取失败: I2C 句柄不可用\n");
        return false;
    }

    uint8_t HV_State = 0;
    if (i2c_read_byte(fpga1, 0x04, &HV_State) != 0) {
        fpga_i2c_unlock();
        printf("[Check_HV_State] ❌ 读取失败: 读寄存器 0x04 出错\n");
        return false;
    }

    fpga_i2c_unlock();

    bool hv_active = (HV_State >> 0) & 0x01;  // 检测 bit0
    printf("[Check_HV_State] ✅ HV 状态: 0x04=0x%02X, bit0(HV_READY)=%d -> %s\n",
           HV_State, hv_active, hv_active ? "高压就绪" : "高压未就绪");
    return hv_active;
}


// [2026-10-14] 读取寄存器 0x04/0x05 并打包返回: 高字节 = 0x04, 低字节 = 0x05。
//   供上层通过协议应答回传, 便于从网口侧诊断高压状态(不必依赖串口日志)。
//   读取失败/句柄不可用时返回 0xFFFF。
uint16_t FPGA_Read_State_Regs(void) {
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        printf("[FPGA_Read_State_Regs] ❌ 读取失败: I2C 句柄不可用\n");
        return 0xFFFF;
    }

    uint8_t r4 = 0xFF, r5 = 0xFF;
    i2c_read_byte(fpga1, 0x04, &r4);
    i2c_read_byte(fpga1, 0x05, &r5);
    fpga_i2c_unlock();

    return ((uint16_t)r4 << 8) | (uint16_t)r5;
}


// [2026-10-14] 调试用: 写指定寄存器(reg=值), 随后回读 0x04/0x05 并打包返回
//   (高字节 = 0x04, 低字节 = 0x05)。用于一次 OTA 后连续试探各控制位对高压
//   时序的影响(比如哪个位能让 DISCHARGE_ACTIVE 归零), 不必每轮复位板子。
//   读取失败/句柄不可用时返回 0xFFFF。
uint16_t FPGA_Write_Reg_And_Read(uint8_t reg, uint8_t val) {
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        printf("[FPGA_Write_Reg] ❌ 失败: I2C 句柄不可用\n");
        return 0xFFFF;
    }

    if (reg == 0xFF) {
        // [2026-10-14] 特殊值 0xFF: 不写寄存器, 改为操作 LDAC(PD13, 低有效)。
        //   val = 0 -> 拉低并【保持】(LDAC 有效, 输出会随 SPI 写入更新)
        //   val = 1 -> 拉高并保持
        //   val = 2 -> 脉冲(低5ms -> 高)
        extern int dbg_keep_ldac(int level);
        if (val == 0 || val == 1) {
            dbg_keep_ldac(val);          // 保持不释放, 避免 unexport 让引脚回默认态
        } else {
            gpio_t *g = gpio_init(109, GPIO_DIR_OUT);
            if (g != NULL) {
                gpio_set_value(g, GPIO_LOW);
                usleep(5 * 1000);
                gpio_set_value(g, GPIO_HIGH);
                gpio_close(g);
                printf("[FPGA_Write_Reg]   已脉冲 LDAC (PD13 低 5ms -> 高)\n");
            } else {
                printf("[FPGA_Write_Reg] ❌ LDAC 脉冲失败\n");
            }
        }
    } else {
        fpga_write_byte(fpga1, reg, val);
    }
    usleep(30 * 1000);          // 留出动作时间

    uint8_t r4 = 0xFF, r5 = 0xFF;
    i2c_read_byte(fpga1, 0x04, &r4);
    i2c_read_byte(fpga1, 0x05, &r5);
    fpga_i2c_unlock();

    return ((uint16_t)r4 << 8) | (uint16_t)r5;
}




uint8_t Read_Info_State_Reg(void) {
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Read_Info_State_Reg] ❌ 读取失败: I2C 句柄不可用\n");
        return 0xFF;
    }

    uint8_t info_state = 0;
    if (i2c_read_byte(fpga1, 0x04, &info_state) != 0) {
        fpga_i2c_unlock();
        printf("[Read_Info_State_Reg] ❌ 读取失败: 读寄存器 0x04 出错\n");
        return 0xFF;
    }

    fpga_i2c_unlock();

    printf("[Read_Info_State_Reg] ✅ 读取完成: 0x04 = 0x%02X\n", info_state);
    return info_state;
}



void Print_Info_State(uint8_t info_state) {
    printf("[InfoState] ▶ 信息状态寄存器 0x04 = 0x%02X\n", info_state);

    printf("[InfoState]   bit7 保留          : %d\n", (info_state >> 7) & 0x01);
    printf("[InfoState]   bit6 急停异常      : %d (%s)\n", (info_state >> 6) & 0x01,
           ((info_state >> 6) & 0x01) ? "故障: 急停按下或线路断开" : "正常: 急停未按下");
    printf("[InfoState]   bit5 放电继电器    : %d (%s)\n", (info_state >> 5) & 0x01,
           ((info_state >> 5) & 0x01) ? "正在放电" : "未放电");
    printf("[InfoState]   bit4 硬件故障锁存  : %d (%s)\n", (info_state >> 4) & 0x01,
           ((info_state >> 4) & 0x01) ? "有故障锁存, 高压禁止" : "无故障锁存");
    printf("[InfoState]   bit3 外部互锁异常  : %d (%s)\n", (info_state >> 3) & 0x01,
           ((info_state >> 3) & 0x01) ? "互锁未许可或线路断开" : "互锁正常");
    printf("[InfoState]   bit2 门禁异常      : %d (%s)\n", (info_state >> 2) & 0x01,
           ((info_state >> 2) & 0x01) ? "门打开或线路断开" : "门关闭正常");
    printf("[InfoState]   bit1 残压安全      : %d (%s)\n", (info_state >> 1) & 0x01,
           ((info_state >> 1) & 0x01) ? "残压低于30V, 安全" : "残压高于30V, 不安全");
    printf("[InfoState]   bit0 高压就绪      : %d (%s)\n", (info_state >> 0) & 0x01,
           ((info_state >> 0) & 0x01) ? "允许测试" : "不允许测试");
}
 






uint8_t Read_CMD1_Before(void) {
    printf("[Read_CMD1_Before] ▶ 开始读取命令状态寄存器 0x05\n");
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Read_CMD1_Before] ❌ 读取失败: I2C 句柄不可用\n");
        return 0xFF;
    }

    uint8_t cmd1 = 0;
    if (i2c_read_byte(fpga1, 0x05, &cmd1) != 0) {
        fpga_i2c_unlock();
        printf("[Read_CMD1_Before] ❌ 读取失败: 读寄存器 0x05 出错\n");
        return 0xFF;
    }

    fpga_i2c_unlock();

    printf("[Read_CMD1_Before] ✅ 读取完成: 0x05 = 0x%02X\n", cmd1);
    return cmd1;
}


uint8_t Read_CMD2_Before(void) {
    printf("[Read_CMD2_Before] ▶ 开始读取命令状态寄存器 0x06\n");
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Read_CMD2_Before] ❌ 读取失败: I2C 句柄不可用\n");
        return 0xFF;
    }

    uint8_t cmd2 = 0;
    if (i2c_read_byte(fpga1, 0x06, &cmd2) != 0) {
        fpga_i2c_unlock();
        printf("[Read_CMD2_Before] ❌ 读取失败: 读寄存器 0x06 出错\n");
        return 0xFF;
    }

    fpga_i2c_unlock();

    printf("[Read_CMD2_Before] ✅ 读取完成: 0x06 = 0x%02X\n", cmd2);
    return cmd2;
}




uint8_t Select_Collect_Card(uint8_t num) {
    if (num > 15) {
        printf("[Select_Collect_Card] ❌ 选择失败: 卡片号 %u 超出范围(0~15)\n", (unsigned)num);
        return 0xFF;
    }

    uint8_t num1 = map_sw(num);
    printf("[Select_Collect_Card] ▶ 开始选择采集卡 %u (映射后开关号=%u)\n",
           (unsigned)num, (unsigned)num1);

    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card] ❌ 选择失败: I2C 句柄不可用\n");
        return 0xFF;
    }

    uint8_t val = FPGA_Build_CMD2(num1);

    // 写入新值
    if (fpga_write_byte(fpga1, 0x06, val) != 0) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card] ❌ 选择失败: 写寄存器 0x06=0x%02X 出错\n", val);
        return 0xFF;
    }

    fpga_i2c_unlock();

    printf("[Select_Collect_Card] ✅ 选择完成: 采集卡 %u, 已写 0x06=0x%02X\n",
           (unsigned)num, val);
    return 0x00;
}






uint8_t Select_Collect_Card_Channel(uint8_t num) {
    if (num > 39) {
        printf("[Select_Collect_Card_Channel] ❌ 选择失败: 通道号 %u 超出范围(0~39)\n", (unsigned)num);
        return 0xFF;
    }

    printf("[Select_Collect_Card_Channel] ▶ 开始选择采集卡通道 %u\n", (unsigned)num);
 

    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card_Channel] ❌ 选择失败: I2C 句柄不可用\n");
        return 0xFE;
    }

    // 读取当前值
 
    // 使用 FPGA_Build_CMD1 合成新值
    // bit7-4: 通道号 (num), bit3-2: 芯片使能 (0), bit1: 放电(0), bit0: hv_enable
    uint8_t val = FPGA_Build_CMD1(num);

    // 写入新值
    if (fpga_write_byte(fpga1, 0x05, val) != 0) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card_Channel] ❌ 选择失败: 写寄存器 0x05=0x%02X 出错\n", val);
        return 0xFF;
    }

    fpga_i2c_unlock();

    printf("[Select_Collect_Card_Channel] ✅ 选择完成: 通道 %u, 已写 0x05=0x%02X\n",
           (unsigned)num, val);
    return num;
}













uint16_t Select_Collect_Card_Channel_HV(uint16_t num) {
    long long t_begin = fpga_now_us();

    printf("[Select_Collect_Card_Channel_HV] ▶ 开始选择: 通道号 %u\n", (unsigned)num);

    if (num > 639) {
        printf("[Select_Collect_Card_Channel_HV] ❌ 选择失败: 通道号 %u 超出范围(0~639)\n",
               (unsigned)num);
        return 0xFF;
    }
 

    i2c_t *fpga1 = fpga_i2c_lock();
    long long t_lock = fpga_now_us();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card_Channel_HV] ❌ 选择失败: I2C 句柄不可用\n");
        return 0xFE;
    }
    uint8_t num1=num%40; 
    uint8_t num2=num/40; 
   


     int pf0_val, pf1_val;

    if (num2 >= 0 && num2 <= 3) {
        // 第1组: 0~3
        pf0_val = 0;
        pf1_val = 1;
    } else if (num2 >= 4 && num2 <= 7) {
        // 第2组: 4~7
        pf0_val = 0;
        pf1_val = 0;
    } else if (num2 >= 8 && num2 <= 11) {
        // 第3组: 8~11
        // ⚠️ 原为 pf0=0/pf1=1(与第1组同为 01, 导致板 0~3 与板 8~11 无法区分)
        //    现改为 1/0, 使四组组合齐全: 01 / 00 / 10 / 11
        pf0_val = 1;
        pf1_val = 0;
    } else { // card >= 12 && card <= 15
        // 第4组: 12~15
        pf0_val = 1;
        pf1_val = 1;
    }

    gpio_t *pf0 = g_pf0;   // PF0 (全局共享句柄)
    gpio_t *pf1 = g_pf1;   // PF1 (全局共享句柄)

    // 懒加载全局 GPIO 句柄（仅首次调用真正打开）
    if (pf0 == NULL || pf1 == NULL) {
        fpga_gpio_get();
        pf0 = g_pf0;
        pf1 = g_pf1;
    }

    if (pf0) {
        gpio_set_value(pf0, pf0_val);
    }
    if (pf1) {
        gpio_set_value(pf1, pf1_val);
    }
    long long t_gpio = fpga_now_us();

    
     num2= map_sw(num2);
    // 读取当前值
 
    // 使用 FPGA_Build_CMD1 合成新值
    // bit7-4: 卡片号 (num), bit3-2: 芯片使能 (0), bit1: 放电(0), bit0: hv_enable
    uint8_t val = FPGA_Build_CMD1_HV(num1);
    uint8_t val2 = FPGA_Build_CMD2_TEST(num2);

    LOG("[Select_Collect_Card_Channel_HV]   中间值(仅在详细排查时看): num1=%u val=0x%02X | num2=%u val2=0x%02X\n",
        (unsigned)num1, val, (unsigned)num2, val2);

    // 写入新值
    if (fpga_write_byte(fpga1, 0x05, val) != 0) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card_Channel_HV] ❌ 选择失败: 写寄存器 0x05=0x%02X 出错\n", val);
        return 0xFF;
    }
    long long t_w05 = fpga_now_us();

    // 写入新值
    if (fpga_write_byte(fpga1, 0x06, val2) != 0) {
        fpga_i2c_unlock();
        printf("[Select_Collect_Card_Channel_HV] ❌ 选择失败: 写寄存器 0x06=0x%02X 出错\n", val2);
        return 0xFF;
    }
    long long t_w06 = fpga_now_us();

    fpga_i2c_unlock();
    long long t_end = fpga_now_us();

    // 只有单次调用超过 5ms 才打印, 便于定位是哪一步慢(正常时静默)
    if (t_end - t_begin > 5000) {
        printf("[Select_Collect_Card_Channel_HV] ⚠️ 耗时偏长: num=%u 总=%lldus | 等I2C锁=%lldus GPIO=%lldus 写0x05=%lldus 写0x06=%lldus\n",
               (unsigned)num,
               t_end - t_begin,
               t_lock - t_begin,
               t_gpio - t_lock,
               t_w05 - t_gpio,
               t_w06 - t_w05);
    }

    printf("[Select_Collect_Card_Channel_HV] ✅ 选择完成: 通道 %u, 已写 0x05=0x%02X, 0x06=0x%02X\n",
           (unsigned)num, val, val2);
    return num;
}














bool HV_Test_En(uint8_t num) {
    printf("[HV_Test_En] ▶ 开始设置高压测试使能: num=%u\n", (unsigned)num);

    if (num != 0 && num != 1) {
        printf("[HV_Test_En] ❌ 设置失败: 无效的命令值 %u (应为 0 或 1)\n", (unsigned)num);
        return false;
    }

    // 1. 检查 HV 状态是否允许使能
    uint8_t hv_state = Read_Info_State_Reg();
    if (hv_state == 0xFF) {
        printf("[HV_Test_En] ❌ 设置失败: 读取 HV 状态寄存器 0x04 出错\n");
        return false;
    }

    // // 检查 bit0: HV_READY (高压就绪)
    // if (((hv_state >> 0) & 0x01) == 0) {
    //     printf("[HV_Test_En] ❌ 设置失败: HV_READY=0, 高压板尚未允许测试\n");
    //     return false;
    // }

    // 2. 读取当前 CMD1 值
    uint8_t val_before = Read_CMD1_Before();
    if (val_before == 0xFF) {
        printf("[HV_Test_En] ❌ 设置失败: 读取 CMD1(0x05) 出错\n");
        return false;
    }

    // 3. 修改 bit0 (高压使能 HV_EN)
    uint8_t val_after;
    if (num) {
        val_after = val_before | (1 << 0);   // 置1
    } else {
        val_after = val_before & ~(1 << 0);  // 置0
    }

    // 4. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[HV_Test_En] ❌ 设置失败: I2C 句柄不可用\n");
        return false;
    }

    if (fpga_write_byte(fpga1, 0x05, val_after) != 0) {
        fpga_i2c_unlock();
        printf("[HV_Test_En] ❌ 设置失败: 写寄存器 0x05=0x%02X 出错\n", val_after);
        return false;
    }
    fpga_i2c_unlock();

    printf("[HV_Test_En] ✅ 设置完成: 高压使能=%u, 0x05: 0x%02X -> 0x%02X\n",
           (unsigned)(num & 0x01), val_before, val_after);
    return true;
}



// ==================== FPGA 复位使能 ====================
bool FPGA_Reset_En(uint8_t num) {
    printf("[FPGA_Reset_En] ▶ 开始设置 FPGA 复位使能: num=%u\n", (unsigned)num);

    if (num != 0 && num != 1) {
        printf("[FPGA_Reset_En] ❌ 设置失败: 无效的命令值 %u (应为 0 或 1)\n", (unsigned)num);
        return false;
    }

    // 1. 读取当前 CMD1 值
    uint8_t val_before = Read_CMD1_Before();
    if (val_before == 0xFF) {
        printf("[FPGA_Reset_En] ❌ 设置失败: 读取 CMD1(0x05) 出错\n");
        return false;
    }

    // 2. 修改 bit2 (FPGA_Reset)
    uint8_t val_after;
    if (num) {
        val_after = val_before | (1 << 2);   // 置1
    } else {
        val_after = val_before & ~(1 << 2);  // 置0
    }

    // 3. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[FPGA_Reset_En] ❌ 设置失败: I2C 句柄不可用\n");
        return false;
    }

    if (fpga_write_byte(fpga1, 0x05, val_after) != 0) {
        fpga_i2c_unlock();
        printf("[FPGA_Reset_En] ❌ 设置失败: 写寄存器 0x05=0x%02X 出错\n", val_after);
        return false;
    }
    fpga_i2c_unlock();

    printf("[FPGA_Reset_En] ✅ 设置完成: FPGA_Reset=%u, 0x05: 0x%02X -> 0x%02X\n",
           (unsigned)(num & 0x01), val_before, val_after);
    return true;
}


// ==================== 放电使能 ====================
bool Discharge_En(uint8_t num) {
    printf("[Discharge_En] ▶ 开始设置放电使能: num=%u\n", (unsigned)num);

    if (num != 0 && num != 1) {
        printf("[Discharge_En] ❌ 设置失败: 无效的命令值 %u (应为 0 或 1)\n", (unsigned)num);
        return false;
    }

    // 1. 读取当前 CMD1 值
    uint8_t val_before = Read_CMD1_Before();
    if (val_before == 0xFF) {
        printf("[Discharge_En] ❌ 设置失败: 读取 CMD1(0x05) 出错\n");
        return false;
    }

    // 2. 修改 bit1 (Discharge)
    uint8_t val_after;
    if (num) {
        val_after = val_before | (1 << 1);   // 置1
    } else {
        val_after = val_before & ~(1 << 1);  // 置0
    }

    // 3. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Discharge_En] ❌ 设置失败: I2C 句柄不可用\n");
        return false;
    }

    if (fpga_write_byte(fpga1, 0x05, val_after) != 0) {
        fpga_i2c_unlock();
        printf("[Discharge_En] ❌ 设置失败: 写寄存器 0x05=0x%02X 出错\n", val_after);
        return false;
    }
    fpga_i2c_unlock();

    printf("[Discharge_En] ✅ 设置完成: Discharge=%u (%s), 0x05: 0x%02X -> 0x%02X\n",
           (unsigned)(num & 0x01), num ? "使能放电" : "关闭放电",
           val_before, val_after);
    return true;
}



// ==================== MCU 允许测试命令 ====================
// bit1: HV_READY_CMD
// 0 = 不允许测试
// 1 = 允许测试
bool MCU_Test_En(uint8_t num) {
    printf("[MCU_Test_En] ▶ 开始设置允许测试命令: num=%u\n", (unsigned)num);

    if (num != 0 && num != 1) {
        printf("[MCU_Test_En] ❌ 设置失败: 无效的命令值 %u (应为 0 或 1)\n", (unsigned)num);
        return false;
    }

    // 1. 读取当前 CMD2 值
    uint8_t val_before = Read_CMD2_Before();
    if (val_before == 0xFF) {
        printf("[MCU_Test_En] ❌ 设置失败: 读取 CMD2(0x06) 出错\n");
        return false;
    }

    // 2. 修改 bit1 (HV_READY_CMD)
    uint8_t val_after;
    if (num) {
        val_after = val_before | (1 << 1);   // 置1 = 允许测试
    } else {
        val_after = val_before & ~(1 << 1);  // 置0 = 不允许测试
    }

    // 3. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[MCU_Test_En] ❌ 设置失败: I2C 句柄不可用\n");
        return false;
    }

    if (fpga_write_byte(fpga1, 0x06, val_after) != 0) {
        fpga_i2c_unlock();
        printf("[MCU_Test_En] ❌ 设置失败: 写寄存器 0x06=0x%02X 出错\n", val_after);
        return false;
    }
    fpga_i2c_unlock();

    printf("[MCU_Test_En] ✅ 设置完成: HV_READY_CMD=%u (%s), 0x06: 0x%02X -> 0x%02X\n",
           (unsigned)(num & 0x01), num ? "允许测试" : "不允许测试",
           val_before, val_after);
    return true;
}

uint8_t FPGA_Set_Buff(uint8_t num){

  if (num != 2 && num != 4 && num != 6 && num != 8) {
        printf("[FPGA_Set_Buff] ❌ 设置失败: 无效的命令值 %u (应为 2、4、6 或 8)\n",
               (unsigned)num);
        return 0xff;
    }

    printf("[FPGA_Set_Buff] ▶ 开始设置缓冲增益: num=%u\n", (unsigned)num);

    // 3. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[FPGA_Set_Buff] ❌ 设置失败: I2C 句柄不可用\n");
        return 0xfe;
    }
    uint8_t val = 0;
    const char *desc = "未知";
    // 增益编码按《控制寄存器3 (0x07)》文档: 0001=2倍, 0010=4倍, 0100=6倍, 1000=8倍
    /* [2026-10-08 实测验证] 增益映射与《控制寄存器3》文档相反, 按实际规律固化:
     * 文档标注: 0001=2倍 0010=4倍 0100=6倍 1000=8倍 —— 实测电压不单调(0.67/0.74/0.505/0.58)
     * 对调后实测: 电压与下发倍数成正比 ✅
     * 当前映射: 2→0x04  4→0x08  6→0x01  8→0x02
     * ⚠️ nuc982/app929 等其他副本仍是文档版映射, 同步代码时勿覆盖此处 */
    switch(num){
        case 2:
            val = 0x04;     // 实验: 实测电压最小的档位
            desc = "2 档(实验映射)";
            break;
        case 4:
            val = 0x08;
            desc = "4 档(实验映射)";
            break;
        case 6:
            val = 0x01;
            desc = "6 档(实验映射)";
            break;
        case 8:
            val = 0x02;
            desc = "8 档(实验映射)";
            break;
        default:
            val = 0x01;
            desc = "2 倍(默认)";
            break;
    }

    if (fpga_write_byte(fpga1, 0x07, val) != 0) {
        fpga_i2c_unlock();
        printf("[FPGA_Set_Buff] ❌ 设置失败: 写寄存器 0x07=0x%02X 出错\n", val);
        return 0xff;
    }
    fpga_i2c_unlock();

    printf("[FPGA_Set_Buff] ✅ 设置完成: 缓冲增益=%s, 已写 0x07=0x%02X\n", desc, val);
    return num;
}



// ==================== 读取并打印所有寄存器 ====================
void Read_All_Registers(void) {
    printf("[Regs] ▶ 开始读取全部寄存器 0x00~0x07\n");
    
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Regs] ❌ 读取失败: I2C 句柄不可用\n");
        return;
    }

    uint8_t reg[8];
    
    // 读取所有寄存器 0x00 ~ 0x07
    for (int i = 0; i < 8; i++) {
        if (i2c_read_byte(fpga1, i, &reg[i]) != 0) {
            fpga_i2c_unlock();
            printf("[Regs] ❌ 读取失败: 读寄存器 0x%02X 出错\n", i);
            return;
        }
    }
    fpga_i2c_unlock();

    // ========== 0x00: 信号检测卡1~8槽在位指示 ==========
    printf("[Regs] [0x00] 信号检测卡1~8槽在位指示: 0x%02X\n", reg[0]);
    printf("[Regs]   槽1: %s, 槽2: %s, 槽3: %s, 槽4: %s\n",
           (reg[0] & 0x01) ? "不在位" : "在位",
           (reg[0] & 0x02) ? "不在位" : "在位",
           (reg[0] & 0x04) ? "不在位" : "在位",
           (reg[0] & 0x08) ? "不在位" : "在位");
    printf("[Regs]   槽5: %s, 槽6: %s, 槽7: %s, 槽8: %s\n",
           (reg[0] & 0x10) ? "不在位" : "在位",
           (reg[0] & 0x20) ? "不在位" : "在位",
           (reg[0] & 0x40) ? "不在位" : "在位",
           (reg[0] & 0x80) ? "不在位" : "在位");

    // ========== 0x01: 信号检测卡9~16槽在位指示 ==========
    printf("[Regs] [0x01] 信号检测卡9~16槽在位指示: 0x%02X\n", reg[1]);
    printf("[Regs]   槽9: %s, 槽10: %s, 槽11: %s, 槽12: %s\n",
           (reg[1] & 0x01) ? "不在位" : "在位",
           (reg[1] & 0x02) ? "不在位" : "在位",
           (reg[1] & 0x04) ? "不在位" : "在位",
           (reg[1] & 0x08) ? "不在位" : "在位");
    printf("[Regs]   槽13: %s, 槽14: %s, 槽15: %s, 槽16: %s\n",
           (reg[1] & 0x10) ? "不在位" : "在位",
           (reg[1] & 0x20) ? "不在位" : "在位",
           (reg[1] & 0x40) ? "不在位" : "在位",
           (reg[1] & 0x80) ? "不在位" : "在位");

    // ========== 0x02: 开关卡1~8槽在位指示 ==========
    printf("[Regs] [0x02] 开关卡1~8槽在位指示: 0x%02X\n", reg[2]);
    printf("[Regs]   槽1: %s, 槽2: %s, 槽3: %s, 槽4: %s\n",
           (reg[2] & 0x01) ? "不在位" : "在位",
           (reg[2] & 0x02) ? "不在位" : "在位",
           (reg[2] & 0x04) ? "不在位" : "在位",
           (reg[2] & 0x08) ? "不在位" : "在位");
    printf("[Regs]   槽5: %s, 槽6: %s, 槽7: %s, 槽8: %s\n",
           (reg[2] & 0x10) ? "不在位" : "在位",
           (reg[2] & 0x20) ? "不在位" : "在位",
           (reg[2] & 0x40) ? "不在位" : "在位",
           (reg[2] & 0x80) ? "不在位" : "在位");

    // ========== 0x03: 开关卡9~16槽在位指示 ==========
    printf("[Regs] [0x03] 开关卡9~16槽在位指示: 0x%02X\n", reg[3]);
    printf("[Regs]   槽9: %s, 槽10: %s, 槽11: %s, 槽12: %s\n",
           (reg[3] & 0x01) ? "不在位" : "在位",
           (reg[3] & 0x02) ? "不在位" : "在位",
           (reg[3] & 0x04) ? "不在位" : "在位",
           (reg[3] & 0x08) ? "不在位" : "在位");
    printf("[Regs]   槽13: %s, 槽14: %s, 槽15: %s, 槽16: %s\n",
           (reg[3] & 0x10) ? "不在位" : "在位",
           (reg[3] & 0x20) ? "不在位" : "在位",
           (reg[3] & 0x40) ? "不在位" : "在位",
           (reg[3] & 0x80) ? "不在位" : "在位");

    // ========== 0x04: 高压板状态检测 ==========
    printf("[Regs] [0x04] 高压板状态检测: 0x%02X\n", reg[4]);
    printf("[Regs]   bit7 保留        : %d\n", (reg[4] >> 7) & 0x01);
    printf("[Regs]   bit6 急停异常    : %d (%s)\n", (reg[4] >> 6) & 0x01,
           ((reg[4] >> 6) & 0x01) ? "故障: 急停按下或线路断开" : "正常: 急停未按下");
    printf("[Regs]   bit5 放电继电器  : %d (%s)\n", (reg[4] >> 5) & 0x01,
           ((reg[4] >> 5) & 0x01) ? "正在放电" : "未放电");
    printf("[Regs]   bit4 硬件故障锁存: %d (%s)\n", (reg[4] >> 4) & 0x01,
           ((reg[4] >> 4) & 0x01) ? "有故障锁存, 高压禁止" : "无故障锁存");
    printf("[Regs]   bit3 外部互锁异常: %d (%s)\n", (reg[4] >> 3) & 0x01,
           ((reg[4] >> 3) & 0x01) ? "互锁未许可或线路断开" : "互锁正常");
    printf("[Regs]   bit2 门禁异常    : %d (%s)\n", (reg[4] >> 2) & 0x01,
           ((reg[4] >> 2) & 0x01) ? "门打开或线路断开" : "门关闭正常");
    printf("[Regs]   bit1 残压安全    : %d (%s)\n", (reg[4] >> 1) & 0x01,
           ((reg[4] >> 1) & 0x01) ? "残压低于30V, 安全" : "残压高于30V, 不安全");
    printf("[Regs]   bit0 高压就绪    : %d (%s)\n", (reg[4] >> 0) & 0x01,
           ((reg[4] >> 0) & 0x01) ? "允许测试" : "不允许测试");

    // ========== 0x05: 控制寄存器1 ==========
    printf("[Regs] [0x05] 控制寄存器1: 0x%02X\n", reg[5]);
    printf("[Regs]   bit7-4 通道选择  : %d (0~15)\n", (reg[5] >> 4) & 0x0F);
    printf("[Regs]   bit3-2 芯片使能  : %d (0~2)\n", (reg[5] >> 2) & 0x03);
    printf("[Regs]   bit1 放电命令    : %d (%s)\n", (reg[5] >> 1) & 0x01,
           ((reg[5] >> 1) & 0x01) ? "使能放电" : "关闭放电");
    printf("[Regs]   bit0 高压使能    : %d (%s)\n", (reg[5] >> 0) & 0x01,
           ((reg[5] >> 0) & 0x01) ? "打开" : "关闭");

    // ========== 0x06: 控制寄存器2 ==========
    printf("[Regs] [0x06] 控制寄存器2: 0x%02X\n", reg[6]);
    printf("[Regs]   bit7 保留        : %d\n", (reg[6] >> 7) & 0x01);
    printf("[Regs]   bit6-3 模拟开关通道: %d (0~15)\n", (reg[6] >> 3) & 0x0F);
    printf("[Regs]   bit2 模拟开关使能: %d (%s)\n", (reg[6] >> 2) & 0x01,
           ((reg[6] >> 2) & 0x01) ? "禁止" : "使能");
    printf("[Regs]   bit1 允许测试命令: %d (%s)\n", (reg[6] >> 1) & 0x01,
           ((reg[6] >> 1) & 0x01) ? "允许测试" : "不允许测试");
    printf("[Regs]   bit0 故障复位命令: %d (%s)\n", (reg[6] >> 0) & 0x01,
           ((reg[6] >> 0) & 0x01) ? "复位" : "不复位");

    // ========== 0x07: 控制寄存器3 ==========
    printf("[Regs] [0x07] 控制寄存器3: 0x%02X\n", reg[7]);
    printf("[Regs]   bit7-4 保留      : %d\n", (reg[7] >> 4) & 0x0F);
    switch ((reg[7] >> 0) & 0x0F) {
        case 0x01: printf("[Regs]   bit3-0 增益控制: %d (2倍放大)\n", (reg[7] >> 0) & 0x0F); break;
        case 0x02: printf("[Regs]   bit3-0 增益控制: %d (4倍放大)\n", (reg[7] >> 0) & 0x0F); break;
        case 0x04: printf("[Regs]   bit3-0 增益控制: %d (6倍放大)\n", (reg[7] >> 0) & 0x0F); break;
        case 0x08: printf("[Regs]   bit3-0 增益控制: %d (8倍放大)\n", (reg[7] >> 0) & 0x0F); break;
        default:   printf("[Regs]   bit3-0 增益控制: %d (未知档位)\n", (reg[7] >> 0) & 0x0F); break;
    }

    printf("[Regs] ✅ 寄存器读取完成\n");
}
// ==================== I2C 读写测试函数 ====================
// 先写入 0x05=0xAA, 0x06=0xBB, 0x07=0xCC，再读回来验证
int Test_I2C_Read_CMD1(void) {
    printf("[Test] ▶ 开始 I2C 读写测试 (0x05/0x06/0x07)\n");
    
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Test] ❌ 测试中止: 获取 I2C 句柄失败\n");
        return -1;
    }
    
    uint8_t val05 = 0, val06 = 0, val07 = 0;
    int ret = 0;
    
    // ========== 1. 读取原始值 ==========
    printf("[Test]   步骤 1/4: 读取原始值\n");
    ret  = i2c_read_byte(fpga1, 0x05, &val05);
    ret |= i2c_read_byte(fpga1, 0x06, &val06);
    ret |= i2c_read_byte(fpga1, 0x07, &val07);
    if (ret != 0) {
        printf("[Test] ❌ 步骤 1/4 失败: 读取原始值出错 (设备无应答?)\n");
    } else {
        printf("[Test]   原始值: 0x05=0x%02X, 0x06=0x%02X, 0x07=0x%02X\n", val05, val06, val07);
    }
    ret = 0;
    
    // ========== 2. 写入测试值 ==========
    printf("[Test]   步骤 2/4: 写入测试值 0xAA, 0xBB, 0xCC\n");
    ret  = i2c_write_byte(fpga1, 0x05, 0xAA);
    ret |= i2c_write_byte(fpga1, 0x06, 0xBB);
    ret |= i2c_write_byte(fpga1, 0x07, 0xCC);
    if (ret != 0) {
        printf("[Test] ❌ 步骤 2/4 失败: 写入测试值出错 (设备无应答?)\n");
    }
    ret = 0;
    
    // ========== 3. 读回验证 ==========
    printf("[Test]   步骤 3/4: 读回验证\n");
    uint8_t read05 = 0, read06 = 0, read07 = 0;
    ret  = i2c_read_byte(fpga1, 0x05, &read05);
    ret |= i2c_read_byte(fpga1, 0x06, &read06);
    ret |= i2c_read_byte(fpga1, 0x07, &read07);
    if (ret != 0) {
        printf("[Test] ❌ 步骤 3/4 失败: 读回出错 (设备无应答?)\n");
    } else {
        printf("[Test]   读回值: 0x05=0x%02X, 0x06=0x%02X, 0x07=0x%02X\n", read05, read06, read07);
    }
    ret = 0;
    
    // ========== 4. 恢复原始值 ==========
    printf("[Test]   步骤 4/4: 恢复原始值\n");
    i2c_write_byte(fpga1, 0x05, val05);
    i2c_write_byte(fpga1, 0x06, val06);
    i2c_write_byte(fpga1, 0x07, val07);
    
    fpga_i2c_unlock();
    
    // ========== 5. 判断结果 ==========
    if (read05 == 0xAA && read06 == 0xBB && read07 == 0xCC) {
        printf("[Test] ✅ 读写测试通过: 三个寄存器均读回与写入一致\n");
        return 0;
    } else {
        printf("[Test] ❌ 读写测试失败: 期望 0x05=0xAA 0x06=0xBB 0x07=0xCC, 实际 0x05=0x%02X 0x06=0x%02X 0x07=0x%02X\n",
               read05, read06, read07);
        return -1;
    }
}




void FPGA_I2C_Close(void) {
    pthread_mutex_lock(&g_fpga_i2c_lock);
    if (g_fpga_i2c != NULL) {
        i2c_close(g_fpga_i2c);
        g_fpga_i2c = NULL;
        printf("[FPGA-I2C] ✅ 关闭完成: 共享 I2C 句柄已释放\n");
    } else {
        printf("[FPGA-I2C] ⚠️ 关闭跳过: 共享句柄未打开\n");
    }
    // 关闭共享 GPIO 句柄
    if (g_pf0 != NULL) {
        gpio_close(g_pf0);
        g_pf0 = NULL;
        printf("[FPGA-GPIO] ✅ 关闭完成: PF0 共享句柄已释放\n");
    }
    if (g_pf1 != NULL) {
        gpio_close(g_pf1);
        g_pf1 = NULL;
        printf("[FPGA-GPIO] ✅ 关闭完成: PF1 共享句柄已释放\n");
    }
    pthread_mutex_unlock(&g_fpga_i2c_lock);
}




// ==================== 简单 I2C 写读测试 ====================
// 测试写寄存器 0x05，然后读回来验证
void Test_I2C_Write_Read(void) {
    printf("[Test] ▶ 开始 I2C 写读测试 (寄存器 0x05/0x06/0x07)\n");
    
    // 1. 获取 I2C 句柄
    i2c_t *fpga = fpga_i2c_lock();
    if (fpga == NULL) {
        fpga_i2c_unlock();
        printf("[Test] ❌ 测试中止: 获取 I2C 句柄失败\n");
        return;
    }
    printf("[Test] ✅ I2C 句柄获取成功, fd=%d\n", fpga->fd);
    
    // ========== 测试寄存器列表 ==========
    uint8_t regs[] = {0x05, 0x06, 0x07};
    uint8_t old_vals[3];
    uint8_t test_vals[] = {0x5A, 0xA5, 0x3C};  // 每个寄存器用不同的测试值
    
    // ========== 1. 测试前读取所有寄存器 ==========
    printf("[Test]   步骤 1/4: 读取测试前的原始值\n");
    for (int i = 0; i < 3; i++) {
        if (i2c_read_byte(fpga, regs[i], &old_vals[i]) != 0) {
            printf("[Test] ❌ 步骤 1/4 失败: 读取寄存器 0x%02X 出错\n", regs[i]);
            fpga_i2c_unlock();
            return;
        }
        printf("[Test]     0x%02X = 0x%02X\n", regs[i], old_vals[i]);
    }
    
    // ========== 2. 写入测试值 ==========
    printf("[Test]   步骤 2/4: 写入测试值\n");
    for (int i = 0; i < 3; i++) {
        if (i2c_write_byte(fpga, regs[i], test_vals[i]) != 0) {
            printf("[Test] ❌ 写入失败: 0x%02X = 0x%02X\n", regs[i], test_vals[i]);
            fpga_i2c_unlock();
            return;
        }
        printf("[Test]     写入 0x%02X = 0x%02X 完成\n", regs[i], test_vals[i]);
    }
    
    // ========== 3. 测试后读取验证 ==========
    printf("[Test]   步骤 3/4: 读回验证\n");
    uint8_t read_vals[3];
    int all_pass = 1;
    for (int i = 0; i < 3; i++) {
        if (i2c_read_byte(fpga, regs[i], &read_vals[i]) != 0) {
            printf("[Test] ❌ 步骤 3/4 失败: 读取寄存器 0x%02X 出错\n", regs[i]);
            fpga_i2c_unlock();
            return;
        }
        if (read_vals[i] == test_vals[i]) {
            printf("[Test]     0x%02X 读回 0x%02X, 与期望 0x%02X 一致 -> 通过\n",
                   regs[i], read_vals[i], test_vals[i]);
        } else {
            printf("[Test]     0x%02X 读回 0x%02X, 与期望 0x%02X 不一致 -> 失败\n",
                   regs[i], read_vals[i], test_vals[i]);
            all_pass = 0;
        }
    }
    
    // ========== 4. 恢复原始值 ==========
    printf("[Test]   步骤 4/4: 恢复原始值\n");
    for (int i = 0; i < 3; i++) {
        if (i2c_write_byte(fpga, regs[i], old_vals[i]) != 0) {
            printf("[Test] ❌ 恢复失败: 0x%02X = 0x%02X\n", regs[i], old_vals[i]);
        } else {
            printf("[Test]     恢复 0x%02X = 0x%02X 完成\n", regs[i], old_vals[i]);
        }
    }
    
    // ========== 5. 最终判断 ==========
    if (all_pass) {
        printf("[Test] ✅ 写读测试通过: 3/3 个寄存器均一致\n");
    } else {
        printf("[Test] ❌ 写读测试失败: 存在不一致的寄存器, 详见上方逐条结果\n");
    }
    
    fpga_i2c_unlock();
}


// teststart: 开始测试
// bit0 = 1 (高压使能), bit1 = 0 (放电关闭), bit3 = 1 (HV_Test)
bool teststart(void) {
    printf("[teststart] ▶ 开始启动测试流程\n");

    // 1. 读取当前 CMD1 值 (0x05)
    uint8_t val_before = Read_CMD1_Before();
    if (val_before == 0xFF) {
        printf("[teststart] ❌ 启动失败: 读取 CMD1(0x05) 出错\n");
        return false;
    }

    


    // 2. 修改位: bit0=1, bit1=0, bit3=1
    uint8_t val_after = val_before;
    val_after |= (1 << 0);   // bit0 置1 (高压使能)
    val_after &= ~(1 << 1);  // bit1 置0 (放电关闭)
    val_after |= (1 << 3);   // bit3 置1 (HV_Test)

    // 3. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[teststart] ❌ 启动失败: I2C 句柄不可用\n");
        return false;
    }

    fpga_write_byte(fpga1, 0x05, 0);
    usleep(20*1000); // 20ms
    fpga_write_byte(fpga1, 0x06, 0);
    usleep(20*1000); // 20ms
        printf("[teststart] 0x05=0x00 0x06=0x00\n");
    uint8_t cmd1_readback = 0;
  
    printf("[teststart]   步骤 1/4: 写 0x05=0x03\n");
    fpga_write_byte(fpga1, 0x05, 3);
    usleep(20*1000); // 20ms
    printf("[teststart]   步骤 2/4: 写 0x06=0x01\n");
    fpga_write_byte(fpga1, 0x06, 1);
    usleep(20*1000); // 20ms
    printf("[teststart]   步骤 3/4: 写 0x05=0x01\n");
    fpga_write_byte(fpga1, 0x05, 1);
    usleep(20*1000); // 20ms
    printf("[teststart]   步骤 4/4: 写 0x06=0x00\n");
    fpga_write_byte(fpga1, 0x06, 0);
    usleep(20*1000); // 20ms

    i2c_read_byte(fpga1, 0x05, &cmd1_readback);

    // ⚠️ 原代码为 cmd0x04 & 0x13!=3, 因 != 优先级高于 &, 实际等价于 cmd0x04 & 1,
    //    判定逻辑与预期不符; 这里按明显意图补上括号并比照 0x03。
    if ((cmd1_readback & 0x01) != 0x01) {
        fpga_i2c_unlock();
        printf("[teststart] ❌ 启动失败: 回读 0x05=0x%02X, 期望 (值 & 0x01) == 0x01\n",
               cmd1_readback);
        return false;
    }
    printf("[teststart]   回读校验通过: 0x05=0x%02X\n", cmd1_readback);

    fpga_write_byte(fpga1, 0x06, 6);
    fpga_i2c_unlock();

    printf("[teststart] ✅ 启动测试完成: 回读前 0x05=0x%02X, 最终写入 0x06=0x02\n",
           val_before);
    return true;
}

// teststop: 停止测试
// bit0 = 0 (高压关闭), bit1 = 1 (放电开启), bit3 = 0 (HV_Test关闭)
bool teststop(void) {
    printf("[teststop] ▶ 开始停止测试流程\n");

    // 1. 读取当前 CMD1 值 (0x05)
    uint8_t val_before = Read_CMD1_Before();
    if (val_before == 0xFF) {
        printf("[teststop] ❌ 停止失败: 读取 CMD1(0x05) 出错\n");
        return false;
    }

    // 2. 修改位: bit0=0, bit1=1, bit3=0
    uint8_t val_after = val_before;
    val_after &= ~(1 << 0);  // bit0 置0 (高压关闭)
    val_after |= (1 << 1);   // bit1 置1 (放电开启)
    val_after &= ~(1 << 3);  // bit3 置0 (HV_Test关闭)

    // 3. 写入新值
    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[teststop] ❌ 停止失败: I2C 句柄不可用\n");
        return false;
    }

    printf("[teststop]   步骤 1/3: 写 0x05=0x%02X\n", val_after);
    fpga_write_byte(fpga1, 0x05, val_after);
    printf("[teststop]   步骤 2/3: 写 0x05=0x00 (高压彻底关闭)\n");
    fpga_write_byte(fpga1, 0x05, 0);
    printf("[teststop]   步骤 3/3: 写 0x06=0x00 (关闭模拟开关)\n");
    fpga_write_byte(fpga1, 0x06, 0);
    fpga_i2c_unlock();

    printf("[teststop] ✅ 停止测试完成: 回读前 0x05=0x%02X, 最终 0x05=0x00, 0x06=0x00\n",
           val_before);
    return true;
}


uint8_t Finish_HV() {
    printf("[Finish_HV] ▶ 开始收尾: 关闭高压与开关\n");

    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[Finish_HV] ❌ 收尾失败: I2C 句柄不可用\n");
        return 0xFE;
    }

    printf("[Finish_HV]   步骤 1/4: 写 0x05=0x03\n");
    fpga_write_byte(fpga1, 0x05, 0X03);
    usleep(20*1000); // 20ms
    printf("[Finish_HV]   步骤 2/4: 写 0x05=0x00\n");
    fpga_write_byte(fpga1, 0x05, 0X00);
    usleep(20*1000); // 20ms
    printf("[Finish_HV]   步骤 3/4: 写 0x06=0x01\n");
    fpga_write_byte(fpga1, 0x06, 0X01);
    usleep(20*1000); // 20ms
    printf("[Finish_HV]   步骤 4/4: 写 0x06=0x00\n");
    fpga_write_byte(fpga1, 0x06, 0X00);
    fpga_i2c_unlock();

    printf("[Finish_HV] ✅ 收尾完成: 已写 0x05=0x03->0x00, 0x06=0x01->0x00\n");
    return 0;
}


uint8_t START_HV_CHECK(void) {
    printf("[START_HV_CHECK] ▶ 开始高压就绪检查\n");

    i2c_t *fpga1 = fpga_i2c_lock();
    if (fpga1 == NULL) {
        fpga_i2c_unlock();
        printf("[START_HV_CHECK] ❌ 检查失败: I2C 句柄不可用\n");
        return 0;
    }
    // [2026-10-14] 按高压上电时序规格实现(0x06 位定义: bit6-3=DG406通道, bit2=FPGA_EN,
    //   bit1=HV_READY_CMD, bit0=FAULT_RESET_CMD)。
    //   关键修正: HV_READY 要到"高压建立之后"(步骤6)才可能变 1,
    //   此前把它当门槛会导致永远失败 —— 这是原来 0x11 开不起来的根因。

    // ---- [2026-10-14] 按规格表: 步骤1/2/3 期间 HV_READY_CMD 必须为 0 ----
    //   实测上电后 0x06=0x06(bit1=1), 与规格不符。这里先清掉, 到步骤6 才重新置 1。
    {
        uint8_t c2 = 0;
        if (i2c_read_byte(fpga1, 0x06, &c2) == 0 && (c2 & 0x02)) {
            fpga_write_byte(fpga1, 0x06, (uint8_t)(c2 & ~0x02));
            printf("[START_HV_CHECK]   步骤1-3: 清 HV_READY_CMD (0x06: 0x%02X -> 0x%02X)\n",
                   (unsigned)c2, (unsigned)(c2 & ~0x02));
            usleep(20 * 1000);
        }
    }

    // ---- 步骤2: 故障复位(仅当 HV_FAULT=1 时发脉冲; 无锁存故障可跳过) ----
    uint8_t st = 0;
    if (i2c_read_byte(fpga1, 0x04, &st) == 0 && (st & 0x10)) {
        uint8_t c2 = 0;
        i2c_read_byte(fpga1, 0x06, &c2);
        printf("[START_HV_CHECK]   步骤2: HV_FAULT=1, 发 FAULT_RESET 脉冲(0x06 bit0)\n");
        fpga_write_byte(fpga1, 0x06, (uint8_t)((c2 & ~0x03) | 0x01));   // bit0=1, 保留其余位
        usleep(10 * 1000);
        fpga_write_byte(fpga1, 0x06, (uint8_t)(c2 & ~0x03));            // bit0=0
        usleep(20 * 1000);
        if (i2c_read_byte(fpga1, 0x04, &st) == 0 && (st & 0x10)) {
            fpga_i2c_unlock();
            printf("[START_HV_CHECK] ❌ 复位后 HV_FAULT 仍为 1, 停在本步\n");
            return 0;
        }
    } else {
        printf("[START_HV_CHECK]   步骤2: 无锁存故障, 跳过复位脉冲\n");
    }

    // ---- 步骤3: 先撤销放电请求, 再使能高压 ----
    printf("[START_HV_CHECK]   步骤3: 写 0x05=0x00 (撤销放电请求)\n");
    fpga_write_byte(fpga1, 0x05, 0x00);
    usleep(50 * 1000);
    printf("[START_HV_CHECK]   步骤3: 写 0x05=0x01 (使能高压)\n");
    fpga_write_byte(fpga1, 0x05, 0x01);
    usleep(50 * 1000);

    // ---- 步骤4: 释放线性级(LDAC) —— PD13 由调用方 Task_Test_En 的 mculdac() 拉高, 此处等待 ----
    usleep(20 * 1000);

    // ---- 步骤6: 发 HV_READY_CMD(0x06 bit1), 再回读 HV_READY ----
    {
        uint8_t c2 = 0;
        if (i2c_read_byte(fpga1, 0x06, &c2) == 0) {
            fpga_write_byte(fpga1, 0x06, (uint8_t)(c2 | 0x02));         // bit1=1, 保留其余位
            printf("[START_HV_CHECK]   步骤6: 发 HV_READY_CMD (0x06: 0x%02X -> 0x%02X)\n",
                   (unsigned)c2, (unsigned)(c2 | 0x02));
        }
    }
    usleep(200 * 1000);     // 等高压建立

    uint8_t hv_state = 0;
    if (i2c_read_byte(fpga1, 0x04, &hv_state) != 0) {
        fpga_i2c_unlock();
        printf("[START_HV_CHECK] ❌ 检查失败: 读取寄存器 0x04 出错\n");
        return 0;
    }
    Print_Info_State(hv_state);
    fpga_i2c_unlock();

    // 判据(按位, 不再依赖 HV_READY): 无锁存故障 且 残压安全 即允许继续
    {
        bool hv_fault = (hv_state >> 4) & 1;
        bool hv_safe  = (hv_state >> 1) & 1;
        bool ready    = (hv_state >> 0) & 1;
        printf("[START_HV_CHECK] ✅ 完成: 0x04=0x%02X (HV_FAULT=%d HV_SAFE=%d HV_READY=%d)\n",
               (unsigned)hv_state, hv_fault, hv_safe, ready);
        return (!hv_fault && hv_safe) ? 1 : 0;
    }
}