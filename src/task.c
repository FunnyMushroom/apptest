// clock_gettime/CLOCK_MONOTONIC 与 usleep 需要特性宏(必须在包含任何头文件之前定义)。
// 显式定义 _DEFAULT_SOURCE 与 gcc 默认 -std=gnu17 的行为一致,
// 可保证在 -std=c11 等严格模式下也能编译。
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "task.h"
#include "tcpserver.h"
#include <stdint.h>
#include <spi.h>
#include <fpga_i2c.h>
#include <gpio.h>
#include "protocol.h"
#include "fpga_i2c.h"
#include "dac8562.h"
#include "ads8691.h"
#include "ads1115.h"
#include <usart.h>
#include <main.h>
#include "realtime.h"
#include <mqueue.h>
#include <time.h>
#include <pthread.h>
#include <gpio_app.h>

// [2026-10-13] SD 卡功能已整体移除
#include "485_cpld.h"

// 本文件原先漏了 log.h, 导致 task.c 里所有 printf 都绕过日志模块
// (既没有时间戳前缀, 也不受一键开关和分模块开关控制)。补上后统一纳管。
// 注意: LOG_MODULE 必须在 #include "log.h" 之前定义。
#define LOG_MODULE  LOG_MOD_TASK
#include "log.h"


// ==================== 全局变量(ADC 数据缓存与 HV 值) ====================
uint16_t I;                      // HV 电流值
uint16_t U;                      // HV 电压值
uint16_t Times_Read_ADC121 = 0;   // ADC121 读取次数
uint8_t HV = 0;   // ADC121 读取次数

// ==================== 扫描通道队列 (opcode 0x23 设置) ====================
// 扫描通道结构体：collect 为采集卡通道，switch 为开关卡通道
typedef struct {
    uint16_t collect;   // 采集卡通道号
    uint16_t sw;        // 开关卡通道号 (switch 是 C++ 关键字，用 sw 代替)
} scan_channel_t;

// [2026-10-16] 1024 -> 4096: 现场开关卡要插满 16 张(卡号 0~15),
// 16 卡 × 128 通道 = 2048 个扫描通道, 旧上限 1024 一次装不下(会回"队列已满 0xFFFFFFFE")。
// 4096 留一倍余量; 静态内存 4096 × 4B = 16KB, 可忽略。
#define MAX_SCAN_CHANNELS 4096
static scan_channel_t g_scan_queue[MAX_SCAN_CHANNELS];
static int g_scan_queue_len = 0;
static pthread_mutex_t g_scan_queue_lock = PTHREAD_MUTEX_INITIALIZER;

// [2026-11-02 二次修复] 最后一次"成功入队"的通道。
//   用途: 线上偶发 0x23 帧尾部被截断(收不全) -> 这条入队被丢弃 -> 0x1D 到达时队列为空
//   -> 一个 0x1E 都不发 -> 上位机判该点失败。
//   0x1D 的语义就是"扫我刚才入队的那一个通道", 队列为空时退回最后一个入队成功的通道,
//   比"什么都不发"好得多(数据可能是上一次的通道, 但至少链路不会整点失败, 且日志会写清楚)。
static scan_channel_t g_last_scan_ch = { 0, 0 };
static int            g_last_scan_valid = 0;

// [2026-11-02 二次修复] 队列"新鲜度": 队列里的通道是给紧接着的那条 0x1D 用的。
//   若 0x1D 因为线路问题没到, 队列里的通道不会被取走; 下一次 0x23 再追加一条,
//   下一次 0x1D 就会扫 2 个通道、回 2 倍 0x1E 帧(2000 帧) —— 上位机按 1 个通道的组收,
//   整组串组。用时间戳判断: 同一批(多点扫描)的 0x23 之间只隔十几毫秒,
//   而"上一轮遗留"至少隔了好几秒(上位机单步超时 3s), 2s 阈值足够安全。
#define SCAN_QUEUE_STALE_MS   2000
static long long g_scan_queue_ts = 0;      // 最近一次入队时刻(ms)

// 前向声明
static void setup_card_gpio(uint8_t card);

// 单调时钟微秒计数(仅用于定位耗时点)
static long long task_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + (long long)ts.tv_nsec / 1000LL;
}

// 注：回包缓冲已改为各任务函数内的局部变量（线程池并发执行时互不干扰），不再使用全局缓冲

// 整包 checksum：用 64 位累加器对（包头+长度+操作码+数据）求和取低16位，不含校验码自身
static uint16_t compute_checksum(const uint8_t *buf, uint32_t total_len) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < total_len; i++) {
        if (i == PROTOCOL_HEADER_SIZE || i == PROTOCOL_HEADER_SIZE + 1) {
            continue;   // 跳过校验码自身
        }
        sum += buf[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

static void Make_Response_Packet(uint8_t opcode, const uint8_t *data, uint32_t data_len,
                                  uint8_t *out_buf, uint32_t *out_len) {
    if (out_buf == NULL || out_len == NULL) {
        return;
    }

    uint32_t total_len = PROTOCOL_HEADER_SIZE + PROTOCOL_CHECKSUM_SIZE + data_len;
    uint32_t idx = 0;

    // 1. 包头 (0xABCD, 大端)
    out_buf[idx++] = 0xAB;
    out_buf[idx++] = 0xCD;

    // 2. 数据长度 (4字节, 大端)
    out_buf[idx++] = (data_len >> 24) & 0xFF;
    out_buf[idx++] = (data_len >> 16) & 0xFF;
    out_buf[idx++] = (data_len >> 8) & 0xFF;
    out_buf[idx++] = data_len & 0xFF;

    // 3. 操作码
    out_buf[idx++] = opcode;

    // 4. 校验码占位（2字节），数据填充后回填整包 checksum
    out_buf[idx++] = 0x00;
    out_buf[idx++] = 0x00;

    // 5. 数据
    if (data != NULL && data_len > 0) {
        memcpy(out_buf + idx, data, data_len);
        idx += data_len;
    }

    // 6. 计算整包 checksum（不含校验码自身），大端(网络字节序)回填
    uint16_t cs = compute_checksum(out_buf, total_len);
    out_buf[PROTOCOL_HEADER_SIZE]     = (cs >> 8) & 0xFF;
    out_buf[PROTOCOL_HEADER_SIZE + 1] = cs & 0xFF;

    *out_len = total_len;
}

static void uint16_to_bytes_be(uint16_t val, uint8_t *buf) {
    if (buf == NULL) return;
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

static void uint32_to_bytes_be(uint32_t val, uint8_t *buf) {
    if (buf == NULL) return;
    buf[0] = (val >> 24) & 0xFF;
    buf[1] = (val >> 16) & 0xFF;
    buf[2] = (val >> 8) & 0xFF;
    buf[3] = val & 0xFF;
}

// ==================== Reply helpers ====================
// 从数据段解析大端字段
static uint16_t be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}
static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

// 构造并发送回包(线程安全, 使用局部缓冲区)
// 注意: 不在本函数内释放 task_arg, 由调用方在回复完成后统一释放
static void task_reply(command_t *task_arg, uint8_t opcode,
                       const uint8_t *data, uint32_t data_len,
                       const char *tag) {
    uint8_t resp[64];
    uint32_t resp_len = 0;
    Make_Response_Packet(opcode, data, data_len, resp, &resp_len);
    int send_result = app_send_response(task_arg, resp, (int)resp_len);
    // [2026-10-13] 成功回执不再打印: ADC 扫描时每个采样点都回一包, 这行会以
    // 每点 1 条的频率刷屏(与电压日志叠加是串口拥塞的两大来源之一)。
    // 失败仍打印, 出问题时可见。
    if (send_result <= 0) {
        printf("[%s] ❌ 回包发送失败: opcode=0x%02X, %u 字节\n",
               tag, opcode, (unsigned)resp_len);
    }
}

// 常用类型回包(自动转换大端字节序)
static void task_reply_u8(command_t *task_arg, uint8_t opcode, uint8_t status, const char *tag) {
    uint8_t data_buf[1] = { status };
    task_reply(task_arg, opcode, data_buf, 1, tag);
}
static void task_reply_u16(command_t *task_arg, uint8_t opcode, uint16_t status, const char *tag) {
    uint8_t data_buf[2];
    uint16_to_bytes_be(status, data_buf);
    task_reply(task_arg, opcode, data_buf, 2, tag);
}
static void task_reply_u32(command_t *task_arg, uint8_t opcode, uint32_t status, const char *tag) {
    uint8_t data_buf[4];
    uint32_to_bytes_be(status, data_buf);
    task_reply(task_arg, opcode, data_buf, 4, tag);
}
static void task_reply_u16_array(command_t *task_arg, uint8_t opcode, uint16_t *data_array, uint8_t count, const char *tag) {
    if (data_array == NULL || count == 0) return;
    
    uint8_t data_buf[512];
    for (int i = 0; i < count; i++) {
        uint16_to_bytes_be(data_array[i], &data_buf[i * 2]);
    }
    task_reply(task_arg, opcode, data_buf, count * 2, tag);
}

// 3 个 16 位字回包(大端), 共 6 字节; 用于把超过 32 位的数据拆成 w0/w1/w2 三段发送
static void task_reply_u16x3(command_t *task_arg, uint8_t opcode,
                             uint16_t w0, uint16_t w1, uint16_t w2, const char *tag) {
    uint8_t data_buf[6];
    uint16_to_bytes_be(w0, &data_buf[0]);
    uint16_to_bytes_be(w1, &data_buf[2]);
    uint16_to_bytes_be(w2, &data_buf[4]);
    task_reply(task_arg, opcode, data_buf, 6, tag);
}


void Task_Detect_Collection_Card(void *arg) {
    command_t *task_arg = (command_t *)arg;
    if (task_arg->data) free(task_arg->data);

    // 打印：任务开始
    printf("[DetectCollect] ▶ 任务开始: 检测采集卡状态\n");

    // 读取采集卡状态 (FPGA 寄存器 0x00~0x01)
    uint16_t status = Check_State_Collect();
    // 打印：采集卡状态
    printf("[DetectCollect] ✅ 任务完成: 采集卡状态=0x%04X\n", status);
    task_reply_u16(task_arg, 0x02, status, "Collect");
    free(task_arg);
}


void Task_Detect_Switch_Card(void *arg) {
    command_t *task_arg = (command_t *)arg;
    if (task_arg->data) free(task_arg->data);

    // 打印：任务开始
    printf("[DetectSwitch] ▶ 任务开始: 检测开关卡状态\n");

    // 读取开关卡状态 (FPGA 寄存器 0x02~0x03)
    uint16_t status = Check_State_Switch();
    // 打印：开关卡状态

    printf("[DetectSwitch] ✅ 任务完成: 开关卡状态=0x%04X\n", status);
    task_reply_u16(task_arg, 0x04, status, "Switch");
    free(task_arg);

}


void Task_Set_HV(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint16_t data = be16(task_arg->data);   /* 大端 */
    free(task_arg->data);

    // 打印：任务开始(目标电压)
    printf("[SetHV] ▶ 任务开始: 目标电压=%u\n", data);

    // 设置高压输出: 先初始化 DAC8562, 再写入目标电压到 A 通道
    uint16_t status = 0;
    // if (DAC8562_Set_Output_A(2000) == 0) {
    //     // 打印：高压设置成功
    //     printf("[SetHV] success: %u\n", data);
    //     status = data;
    //     HV=status;
    // }

//     for (data = 2000; data <= 4000; data += 2000) {
//     if (DAC8562_Set_Output_A(data) == 0) {
//         // 打印：高压设置成功
//         printf("[SetHV] success: %u\n", data);
//         status = data;
//         HV = status;
//     }

//     if (data < 40000) {
//         sleep(5);   // 间隔 5s
//     }
// }
DAC8562_Set_Output_A(data);
    // else {
    //     // 打印：高压设置失败
    //     fprintf(stderr, "[SetHV] failed\n");
    // }
    task_reply_u16(task_arg, 0x06, status, "SetHV");
    free(task_arg);
}


void Task_Read_HV(void *arg) {
    command_t *task_arg = (command_t *)arg;
    if (task_arg->data) free(task_arg->data);

    // 打印：任务开始
    printf("[ReadHV] ▶ 任务开始: 读取高压电压\n");

    // 读取 HV 电压 (ADS1115 通道)
    U = ADS1115_Read_ADC_U();
    // 打印：HV 电压值
    printf("[ReadHV] ✅ 任务完成: 电压=%u\n", U);
    task_reply_u16(task_arg, 0x08, U, "ReadHV");
    free(task_arg);
}


void Task_Control_Times_HV(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint32_t data = be32(task_arg->data);   /* 大端 */
    free(task_arg->data);

    // 打印：任务开始
    printf("[SetDelay] ▶ 任务开始: 设置延时/次数\n");

    // 设置操作次数/延时 (当前仅记录并回传)
    printf("[SetDelay] ✅ 任务完成: %u ms\n", data);
    task_reply_u32(task_arg, 0x0A, data, "SetDelay");
    free(task_arg);
}


void Task_Check_Self(void *arg) {
    command_t *task_arg = (command_t *)arg;
    free(task_arg->data);

    // 打印：任务开始
    printf("[SelfCheck] ▶ 任务开始: 系统自检\n");

    // 自检: 读取 FPGA 信息状态寄存器 0x04
    uint8_t status = Read_Info_State_Reg();
    // 打印：自检结果
    printf("[SelfCheck] ✅ 任务完成: 状态=0x%02X\n", status);
    task_reply_u8(task_arg, 0x0C, status, "SelfCheck");
    free(task_arg);
}


void Task_Set_Buff(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint8_t data = task_arg->data[0];  /* 从任务参数提取数据 */
    free(task_arg->data);

    // 打印：任务开始
    printf("[SetGain] ▶ 任务开始: 设置增益=%u\n", data);

    // 设置增益 (FPGA)
    uint8_t result = FPGA_Set_Buff(data);
    // 打印：增益设置
    printf("[SetGain] ✅ 任务完成: 增益参数 %u, 器件回读 %u\n", data, result);
    task_reply_u8(task_arg, 0x0E, result, "SetGain");
    free(task_arg);
}


void Task_Reset_Fpga(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint32_t data = be32(task_arg->data);   /* 大端 */
    free(task_arg->data);

    // 打印：任务开始
    printf("[ResetFPGA] ▶ 任务开始: 复位 FPGA, 参数=%u\n", data);

    // 复位 FPGA (CMD1 bit2)
    uint32_t status = 0xFFFFFFFF;
    // if (FPGA_Reset_En(data) == true) {
    //     // 打印：FPGA 复位成功
    //     printf("[ResetFPGA] success\n");
    //     status = 0;
    // } else {
    //     // 打印：FPGA 复位失败
    //     fprintf(stderr, "[ResetFPGA] failed\n");
    // }

    fpgareset();

    task_reply_u32(task_arg, 0x10, status, "ResetFPGA");
    free(task_arg);

}


void Task_Test_En(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint32_t data = be32(task_arg->data);   /* 大端 */
    free(task_arg->data);
    // 打印：任务开始
    printf("[TestEn] ▶ 任务开始: 使能测试, 参数=%u\n", data);
    // fpgareset();
    atomic_store(&adc_stop_flag, 1);

    if (START_HV_CHECK() == 1) {
        printf("[TestEn] ✅ 高压就绪, 开始测试\n");
    } else {
        // [2026-10-14] 应答里带上 0x04/0x05 状态(高16位=标志0xFFFF, 低16位: 高字节=0x04,
        //   低字节=0x05), 便于从网口直接诊断"高压为什么没就绪"。
        //   先回包再打印: 串口输出受阻时, 排在 printf 后面的应答会发不出去。
        uint16_t st = FPGA_Read_State_Regs();
        task_reply_u32(task_arg, 0x12, 0xFFFF0000u | (uint32_t)st, "TestEn");
        printf("[TestEn] ❌ 高压未就绪(0x04=0x%02X, 0x05=0x%02X), 测试无法开始\n",
               (unsigned)((st >> 8) & 0xFF), (unsigned)(st & 0xFF));
        free(task_arg);
        return;
    }
    // [2026-10-14] 停用 teststart(): 其硬编码序列(0x05=0x00->0x03->0x01)会把
    //   HV_ENABLE 与 DISCHARGE 同时置1(0x03), 与 START_HV_CHECK 步骤3 重复且多一次
    //   "边开高压边放电"。START_HV_CHECK 已按规格完成"撤销放电 + 使能高压"。
    // teststart();

    

    usleep(10 * 1000);  // 等待 10ms, 确保 DAC 输出稳定
      
    mculdac();
    printf("[TestEn]   已触发 MCU-DAC, PD13 置高\n");
 

    // 进入采样循环: 每 2s 读一次 I/U, 直到 HV_stop_flag 被置 1
    printf("[TestEn]   开始 1/U 采样循环 (每 2s 一次, 等 HV_stop_flag=1 退出)\n");
    uint32_t status = 0xFFFFFFFF;
        task_reply_u32(task_arg, 0x12, status, "TestEn");
   atomic_store(&HV_stop_flag, 0);
int val = 0;
while (1) {
    // 每50ms检测一次
    sleep(2);
    ADS1115_Read_ADC_I();
    ADS1115_Read_ADC_U();
    // 检查停止信号
    val = atomic_load(&HV_stop_flag);
    if (val == 1) {
        break;
    }
    
    // // 读取高压状态
    // uint8_t hv_state = Read_Info_State_Reg();
    // if (hv_state == 0xFF) {
    //     send_error_reply(task_arg, 0xF6, task_arg->opcode);
    //     break;
    // }
    
    // // bit5: 放电继电器 (必须为0)
    // if (((hv_state >> 5) & 0x01) == 1) {
    //     send_error_reply(task_arg, 0xF6, task_arg->opcode);
    //     break;
    // }
    
    // // bit4: 硬件故障锁存 (必须为0)
    // if (((hv_state >> 4) & 0x01) == 1) {
    //     send_error_reply(task_arg, 0xF6, task_arg->opcode);
    //     break;
    // }
    
    // // bit0: 高压使能 (必须为1)
    // if (((hv_state >> 0) & 0x01) == 0) {
    //     send_error_reply(task_arg, 0xF6, task_arg->opcode);
    //     break;
    // }
    
    // // 读取电压
    // U = ADS1115_Read_ADC_U();
    
    // // 检查电压范围
    // if (U > HV + HV / 5) {
    //     send_error_reply(task_arg, 0xF4, task_arg->opcode);
    //     break;
    // } else if (U < HV - HV / 5) {
    //     send_error_reply(task_arg, 0xF5, task_arg->opcode);
    //     break;
    // }
}
    HV = 0;
    teststop();
    atomic_store(&HV_stop_flag, 1);
    atomic_store(&adc_stop_flag, 1);
    DAC8562_Set_Output_A(0);
    disable_all_switch();
    Finish_HV();
    mculdac0();
    printf("[TestEn] ✅ 任务完成: 已关闭高压与全部开关, 采样循环已退出\n");
    free(task_arg);
}


void Task_Test_Stop(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint32_t data = be32(task_arg->data);   /* 大端 */
    free(task_arg->data);
    
    // 打印：任务开始
    printf("[TestStop] ▶ 任务开始: 停止测试, 参数=%u\n", data);
    
    atomic_store(&adc_stop_flag, 1);
    atomic_store(&HV_stop_flag, 1);
    // 停止测试: 清除 HV_Test 命令 (CMD1 bit3)
    uint32_t status = 0xFFFFFFFF;
    // if (HV_Test_En(0) == true) {
    //     // 打印：测试停止成功
    //     printf("[TestStop] stopped\n");
    //     status = 0;
    // } else {
    //     // 打印：测试停止失败
    //     fprintf(stderr, "[TestStop] failed\n");
    // }
    task_reply_u32(task_arg, 0x14, status, "TestStop");
    free(task_arg);
}

// void Task_Test_Select_Collect_Card(void *arg) {
//     command_t *task_arg = (command_t *)arg;
//     uint16_t data = be16(task_arg->data);   /* 大端 */
//     free(task_arg->data);

//     // 打印：任务开始
//     printf("[SelCollect] ▶ 任务开始: 选择采集卡, 掩码=0x%04X\n", data);

//     // 选择采集卡: data 为单位置位掩码
//     uint16_t status = 0xFFFE;
//     if (data != 0 && (data & (data - 1)) == 0) {
//         uint8_t card = 0;
//         for (uint8_t num = 0; num < 16; num++) {
//             if (data & (1 << num)) {
//                 card = num;
//             }
//         }

//         setup_card_gpio(card);
//         Select_Collect_Card(card);
//         printf("[SelCollect] data=0x%04X, card=%u\n", data, card);
//         status = data;
//     } else {
//         fprintf(stderr, "[SelCollect] invalid mask: 0x%04X\n", data);
//     }
//     task_reply_u16(task_arg, 0x16, status, "SelCollect");
//     free(task_arg);
// }


// void Task_Test_Select_Switch_Card(void *arg) {
//     command_t *task_arg = (command_t *)arg;
//     uint16_t data = be16(task_arg->data);   /* 大端 */
//     free(task_arg->data);

//     // 打印：任务开始
//     printf("[SelSwitch] ▶ 任务开始: 选择开关卡, 掩码=0x%04X\n", data);

//     // 选择开关卡 (当前仅校验掩码后返回0)
//     uint16_t status = 0xFFFF;
//     if (data != 0 && (data & (data - 1)) == 0) {
//         uint8_t card = 0;
//         for (uint8_t num = 0; num < 16; num++) {
//             if (data & (1 << num)) {
//                 card = num;
//             }
//         }

//         select_switch(card);
//         // 打印：已选择开关卡
//         printf("[SelSwitch] card %u\n", card);

        
//         status = 0;
//     } else {
//         // 打印：无效掩码
//         fprintf(stderr, "[SelSwitch] invalid mask: 0x%04X\n", data);
//     }
//     task_reply_u16(task_arg, 0x18, status, "SelSwitch");
//     free(task_arg);
// }


// // 合并选择采集卡+通道, 输入16位通道号(0~639)
// // 自动计算 card=ch/40, card_ch=ch%40, 设置GPIO, 选择采集卡和通道, 并加入扫描队列
// void Task_Test_Select_Collect_Card_Channel(void *arg) {
//     command_t *task_arg = (command_t *)arg;
//     uint16_t ch = be16(task_arg->data);   /* 大端 16位 */
//     free(task_arg->data);

//     // 打印：任务开始
//     printf("[SelChannel] ▶ 任务开始: 选择采集卡通道, ch=%u\n", ch);

//     if (ch > 639 && ch != 65535) {
//         fprintf(stderr, "[SelChannel] invalid channel: %u (max 639)\n", ch);
//         task_reply_u16(task_arg, 0x1A, 0xFFFF, "SelChannel");
//         free(task_arg);
//         return;
//     }

//     // 检查adc_stop_flag是否为1
//     if (atomic_load(&adc_stop_flag) != 1) {
//         printf("[SelChannel] 失败: adc_stop_flag=%d (需要为1才能设置队列)\n", atomic_load(&adc_stop_flag));
//         task_reply_u16(task_arg, 0x1A, 0xFFFF, "SelChannel");
//         free(task_arg);
//         return;
//     }

//     // 加锁操作队列
//     pthread_mutex_lock(&g_scan_queue_lock);
    
//     // 如果ch是65535，清空队列
//     if (ch == 65535) {
//         printf("[SelChannel] 收到65535，清空队列\n");
//         g_scan_queue_len = 0;
//         memset(g_scan_queue, 0, sizeof(g_scan_queue));
//         pthread_mutex_unlock(&g_scan_queue_lock);
        
//         task_reply_u16(task_arg, 0x1A, 0xFFFF, "SelChannel");
//         free(task_arg);
//         return;
//     }
    
//     // 检查队列是否已满
//     if (g_scan_queue_len >= MAX_SCAN_CHANNELS) {
//         fprintf(stderr, "[SelChannel] 队列已满，丢弃通道 %u (当前%d个)\n", ch, g_scan_queue_len);
//         pthread_mutex_unlock(&g_scan_queue_lock);
//         task_reply_u16(task_arg, 0x1A, 0xFFFE, "SelChannel");  // 0xFFFE 表示队列满
//         free(task_arg);
//         return;
//     }
    
//     // 正常通道：加入队列
//     uint8_t card = ch / 40;
//     uint8_t card_ch = ch % 40;

//     // // 设置GPIO并选择采集卡//keyoukewu
//     setup_card_gpio(card);

//     // 选择卡内通道
//     Select_Collect_Card_Channel_HV(ch);
//     char buffer_que[32];
//     // 打印当前队列数据

//     sprintf(buffer_que, "%d\n", ch);

//     sd_write_append("lxlceshi.txt", buffer_que);
//     // g_scan_queue[g_scan_queue_len++] = ch;  // 加入新通道
    
//     // // 打印当前队列数据
//     // printf("[SelChannel] 当前队列: ");
//     // for (int i = 0; i < g_scan_queue_len; i++) {
//     //     printf("%d ", g_scan_queue[i]);
//     // }
//     // printf("(共%d个)\n", g_scan_queue_len);
    
//     pthread_mutex_unlock(&g_scan_queue_lock);

//     // 回包: 高8位=card, 低8位=card_ch
//     uint16_t status = ch;
//     task_reply_u16(task_arg, 0x1A, status, "SelChannel");
//     free(task_arg);
// }








void Task_Test_Read_ADC121_Times(void *arg) {
    command_t *task_arg = (command_t *)arg;
    
    // 打印：任务开始
    printf("[ADCTimes] ▶ 任务开始: 设置 ADC121 采样次数\n");
    
    // 检查adc_stop_flag是否为0
    if (atomic_load(&adc_stop_flag) == 0) {
        printf("[ADCTimes] ❌ 设置被拒: adc_stop_flag=%d (需先置1才能设置)\n",
               atomic_load(&adc_stop_flag));
        send_error_reply(task_arg, 0xF7, task_arg->opcode);
        free(task_arg);
        return;
    }
    
    // 数据现在是大端16位 (2字节)
    uint16_t data = be16(task_arg->data);
    free(task_arg->data);
    
    printf("[ADCTimes]   收到采样次数参数=%u\n", data);
    
    // 设置 ADC121 读取次数 (只取低8位，最大255)
    Times_Read_ADC121 = data;
    printf("[ADCTimes] ✅ 任务完成: 采样次数=%u\n", Times_Read_ADC121);
    
    task_reply_u8(task_arg, 0x1C, Times_Read_ADC121, "ADCTimes");
    free(task_arg);
}










// void* send_adc_data_task(void *arg) {
//     command_t *task_arg = (command_t *)arg;
    
//     printf("[SendADC] ▶ 发送任务开始\n");
    
//     mqd_t mq = mq_open("/adc121s_data", O_RDONLY | O_NONBLOCK);
//     if (mq == (mqd_t)-1) {
//         perror("mq_open");
//         opcode_release(task_arg->opcode);   // 0x1D 占用提前释放
//         free(task_arg);
//         return NULL;
//     }
    
//     uint16_t data;
//     unsigned int prio;
//     int val;
//     int count = 0;  // 统计发送数量
        
//     while (1) {
//         val = atomic_load(&adc_stop_flag);
//         if (val == 1) {
//             while (mq_receive(mq, (char*)&data, 2, &prio) > 0) {
//                 count++;
//                 //task_reply_u16(task_arg, 0x1E, data, "Readadc"); 
//                 //这条注释  注释就是停止发送的时候就立刻停止回复上位机 取消就是发送剩余队列给上位机
//             }
//             break;
//         }
        
//         if (mq_receive(mq, (char*)&data, 2, &prio) > 0) {
//             count++;
//             task_reply_u16(task_arg, 0x1E, data, "Readadc");
//         } else {
//             if (errno == EAGAIN) {
//                 sleep(1);
//             } else {
//                 perror("mq_receive");
//                 break;
//             }
//         }
//     }
    
//     printf("[发送任务] 共发送 %d 条数据\n", count);
//     mq_close(mq);
//     opcode_release(task_arg->opcode);       // 发送线程真正结束后才释放 0x1D 占用
//     free(task_arg);
//     return NULL;
// }







// void Task_Test_Read_ADC121_Print(void *arg) {
//     command_t *task_arg = (command_t *)arg;
//     free(task_arg->data);
//     task_arg->data = NULL;
    
//     // 打印：任务开始
//     printf("[ADC121Print] ▶ 任务开始: ADC121 现场读取\n");
    
//     mqd_t mq = mq_open("/adc121s_data", O_RDWR);
//     atomic_store(&adc_stop_flag, 0);
    
//     printf("\n========== ADC121 现场读取 ==========\n");
//     printf("采样次数: %u\n", Times_Read_ADC121);

//     if (Times_Read_ADC121 == 0) {
//         printf("[ADC121] ⚠️ 采样次数为 0\n");
//         printf("========== 结束 ==========\n\n");
//         opcode_release(task_arg->opcode);   // 未进入采集, 提前释放 0x1D 占用
//         free(task_arg);
//         return;
//     }

//     if (ADC121S_Init() < 0) {
//         printf("[ADC121] ❌ ADC 初始化失败\n");
//         opcode_release(task_arg->opcode);   // 未进入采集, 提前释放 0x1D 占用
//         free(task_arg);
//         return;
//     }

//     pthread_t send_thread;
//     if (pthread_create(&send_thread, NULL, send_adc_data_task, task_arg) != 0) {
//         perror("pthread_create send_adc_data_task");
//         opcode_release(task_arg->opcode);   // 线程创建失败, 释放 0x1D 占用
//         free(task_arg);
//         return;
//     }
//     pthread_detach(send_thread);

//     int val;
//     uint16_t ADC121_buf = 0;
//     uint16_t mapped_value = 0;
    
//     // 20个区间计数器 (2466-4096 分20档)
//     int range_count[20] = {0};
//     char range_names[20][20];
    
//     // 生成区间名称
//     int step = (4096 - 2466) / 20;  // 约81.5
//     for (int i = 0; i < 20; i++) {
//         int start = 2466 + i * step;
//         int end = 2466 + (i + 1) * step - 1;
//         if (i == 19) end = 4096;
//         snprintf(range_names[i], sizeof(range_names[i]), "%d-%d", start, end);
//     }
    
//     for (uint16_t i = 0; i < Times_Read_ADC121; i++) {
//         val = atomic_load(&adc_stop_flag);
//         if (val == 1) {
//             printf("[ADC121] ⚠️ 收到停止信号，已采集 %d 次\n", i);
//             break;
//         }
        
//         ADC121_buf = ADC121S_Read();
        
//         // 映射规则：大于等于2466保持原值，小于2466计算对称值
//         if (ADC121_buf >= 2466) {
//             mapped_value = ADC121_buf;
//         } else {
//             // 对称值 = 2466 + (2466 - ADC121_buf)
//             mapped_value = 2466 + (2466 - ADC121_buf);
//             if (mapped_value > 4096) mapped_value = 4096;
//         }
        
            
//         mq_send(mq, (char*)&mapped_value, 2, 0);
        
        
//         // 统计区间 (20档)
//         if (mapped_value >= 2466 && mapped_value <= 4096) {
//             int index = (mapped_value - 2466) / step;
//             if (index > 19) index = 19;
//             range_count[index]++;
//         }
        
//         float voltage = (float)mapped_value / 4095.0 * 5.0;
//         printf("[%03d] 原始值: %5u → 映射值: %5u →  %.3f V\n", 
//                i + 1, ADC121_buf, mapped_value, voltage);
//         usleep(10000);
//     }
    
//     ADC121S_Close();
    
//     // 打印统计结果
//     printf("\n========== 区间统计 (20档, 2466-4096) ==========\n");
//     for (int i = 0; i < 20; i++) {
//         printf("  %s: %d 次 (%.1f%%)\n", 
//                range_names[i], 
//                range_count[i],
//                (float)range_count[i] / Times_Read_ADC121 * 100);
//     }
//     printf("================================================\n\n");
    
//     // Times_Read_ADC121 = 0;
//     atomic_store(&adc_stop_flag, 1);
//     printf("[ADC121Print] ✅ 任务结束\n");
// }


// ==================== 采集卡 GPIO 选择辅助函数 ====================
// 根据采集卡编号设置 GPIO PF0(160)/PF1(161), 与 Task_Test_Select_Collect_Card 相同逻辑
static void setup_card_gpio(uint8_t card) {
    gpio_t *gpio_pf0 = gpio_init(160, GPIO_DIR_OUT);
    gpio_t *gpio_pf1 = gpio_init(161, GPIO_DIR_OUT);
    if (gpio_pf0 && gpio_pf1) {
        gpio_value_t pf0_val, pf1_val;
        if (card <= 3) {
            pf0_val = GPIO_LOW;  pf1_val = GPIO_HIGH;
        } else if (card <= 7) {
            pf0_val = GPIO_LOW;  pf1_val = GPIO_LOW;
        } else if (card <= 11) {
            pf0_val = GPIO_LOW;  pf1_val = GPIO_HIGH;
        } else {
            pf0_val = GPIO_HIGH; pf1_val = GPIO_HIGH;
        }
        gpio_set_value(gpio_pf0, pf0_val);
        gpio_set_value(gpio_pf1, pf1_val);
        gpio_close(gpio_pf0);
        gpio_close(gpio_pf1);
    }
}
void split_num(uint16_t num, uint8_t *high, uint8_t *low)
{
    /* [2026-10-09 修复边界] 上位机 num 为 0 基连续编号(num = 卡号*128 + 界面通道0~127),
     * CPLD CH 为 1 基(CH=0 = 全关)。转换: 卡号 = num/128, CPLD CH = num%128 + 1。
     * 原写法 v=num+1 后再拆, num%128==127(每卡第128路)会算出 CH=0(误关断)且卡号错位进位;
     * 改为先拆后 +1, 对非边界通道行为不变, 边界安全(CH 范围 1~128) */
    if (high) *high = (uint8_t)(num / 128);
    if (low)  *low  = (uint8_t)(num % 128 + 1);
}

// [2026-11-03] 频率随 0x23 下发的辅助: 把开关卡号/通道换算成 FPGA 的 5 字节频率帧
//   地址 = switch/128, 通道 = switch%128 + 1(与 split_num 的 1 基约定一致)。
//   [2026-11-04] FPGA 收到频率帧会回显 2 字节(地址+通道), cpld_send_freq_frame
//   发完后最多等 5ms 回显: 0 = 成功; -2 = 已发出但未确认; -1 = 未发出。
//   频率帧是"尽力而为": 任何失败都只打印, 不影响通道选择与 0x24 回包。
//   [2026-11-04] freq 扩到 1~256: 0 或 >256 直接返回(0 = 调用方表示"不下发",
//   256 由 cpld_send_freq_frame 编码成 0x00)。越界在调用处已打印警告。
static void set_scan_freq_frame(uint16_t switch_num, uint16_t freq) {
    if (freq < CPLD_FREQ_MIN || freq > CPLD_FREQ_MAX) {
        return;
    }
    uint8_t faddr = 0, fch = 0;
    split_num(switch_num, &faddr, &fch);
    int ret = cpld_send_freq_frame(faddr, fch, freq);
    if (ret == -2) {
        printf("[SetScanChannels] ⚠️ 频率帧已发出但 FPGA 未回显(开关卡=%u, 通道=%u, 频率=%u): "
               "频率可能未生效, 通道选择不受影响\n", faddr, fch, (unsigned)freq);
    } else if (ret != 0) {
        printf("[SetScanChannels] ⚠️ 频率帧未发出(开关卡=%u, 通道=%u, 频率=%u), 通道选择不受影响\n",
               faddr, fch, (unsigned)freq);
    }
}
// ==================== 操作码 0x23: 设置扫描通道队列 ====================
// 数据: 5字节, 前2字节为 collect (大端), 后2字节为 switch (大端), 第5字节为频率系数
//   第5字节 = 频率系数(1~256, 单位 10kHz), 下位机按开关卡号/通道换算成
//   FPGA 5字节帧 AA 地址 通道 频率 ED(地址=switch/128, 通道=switch%128+1, 同 split_num),
//   发出后等待 FPGA 回显(地址+通道), 最多 5ms, 不影响 0x1D 时序。
//   [2026-11-04] 频率范围 1~128 -> 1~256: 帧里只有 1 字节, 256 编码为 0x00。
//   因此"不下发频率"的判断改为看**帧长**(4 字节 = 旧格式 = 不下发), 不再用 freq==0 判断。
//   兼容旧格式: 数据为 4 字节(无频率系数)时不修改频率, 其余处理相同。
// 一次只添加一个通道到队列
// 回包 0x24: 数据位宽与请求保持一致(新格式 5 字节, 旧格式 4 字节)
//   成功: 原样回显收到的数据 (collect+switch+freq), 便于上位机核对
//   失败: 0xFFFFFFFE = 队列已满, 0xFFFFFFFF = adc_stop_flag 未置 1(被拒), 均为 4 字节
// [2026-11-02] 提前声明: 本函数里要先把上一次扫描留下的 PD14/PD15 脉冲拉低
//   (定义在本文件后部 PD14/PD15 脉冲那一节)
static int pd1415_pulse_cancel(void);

void Task_Set_Scan_Channels(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint8_t *data = task_arg->data;

    // 打印：任务开始
    printf("[SetScanChannels] ▶ 任务开始: 设置扫描通道队列\n");

    // 检查adc_stop_flag是否为1
    if (atomic_load(&adc_stop_flag) != 1) {
        printf("[SetScanChannels] ❌ 设置被拒: adc_stop_flag=%d (需先置1才能设置队列)\n",
               atomic_load(&adc_stop_flag));
        task_reply_u32(task_arg, 0x24, 0xFFFFFFFFu, "SetScanChannels");
        free(task_arg->data);
        free(task_arg);
        return;
    }

    // ==================== 关键时序修复 ====================
    // [2026-11-02] 在这里(0x23 到达时)就把上一次扫描留下的 PD14/PD15 脉冲拉低。
    //
    //   现场现象: 上位机偶尔把一次本来正常的扫描判成失败, 对应的日志是
    //       xx:xx:xx.701 [pd1415] ✅ ... 已恢复低电平        <- 扫描已经开始之后!
    //       xx:xx:xx.702 [AutoScan] 队列共 1 个通道 ...
    //   实测每一次扫描开始时(0x1D 之后的 4~6ms)都会出现一次"拉低", 因为:
    //       扫描结束 -> PD14/PD15 拉高(1 秒同步脉冲)
    //       上位机 355ms 后发 0x23, 再 13ms 后发 0x1D
    //       原来要等 0x1D 进到扫描任务里才去打断脉冲 -> 拉低落在"扫描开始之后"
    //   上位机若在这段窗口里读到高->低跳变, 就会认为"本轮扫描已结束"(实测 9 次撞 1 次)。
    //
    //   移到 0x23: 拉低发生在 0x1D 之前(实测早 13ms), 整个扫描过程 PD14/PD15 恒为低,
    //   扫描期间不再出现任何下降沿; 而这次"拉低"仍然紧跟在上一次扫描结束之后, 语义不变。
    //   配合 gpio_app.c 里 PD14/PD15 改为常驻句柄(拉低从 ~6ms 降到 <0.1ms), 窗口彻底消失。
    if (pd1415_pulse_cancel()) {
        printf("[SetScanChannels]   PD14/PD15 已在上一次脉冲自然结束前被拉低(上位机回访偏快), 详见上行宽度\n");
    }

    // 解析数据: 前2字节 collect, 后2字节 switch (大端); 第5字节为频率系数(新格式)
    uint16_t collect_ch = be16(data);      // 前2字节
    uint16_t switch_ch = be16(data + 2);   // 后2字节
    // [2026-11-03] 第 5 字节 = 频率系数(1~256, ×10kHz); 旧 4 字节格式没有该字节 = 不修改频率
    // [2026-11-04] 频率扩到 1~256 后, 0x00 不再表示"不下发", 而是"256"。
    //   "是否下发"只看帧长: data_len >= 5 -> 一定下发; 4 -> 不下发。
    uint16_t freq = 0;                     // 0 = 不下发频率帧(仅旧4字节格式)
    if (task_arg->data_len >= 5) {
        freq = data[4];
        if (freq == 0) {
            freq = 256;                    // 0x00 约定为满量程 256 (×10kHz = 2.56MHz)
        }
    }
    // 回包原样回显收到的数据(5 或 4 字节), 先拷到局部缓冲, 后面会 free(task_arg->data)
    uint8_t  echo_buf[5] = { 0 };
    uint32_t echo_len = (task_arg->data_len >= 5) ? 5 : 4;
    memcpy(echo_buf, data, echo_len);

    printf("[SetScanChannels]   收到通道: collect=%u, switch=%u, freq=%u%s\n",
           collect_ch, switch_ch, (unsigned)freq,
           (echo_len == 5) ? "" : " (旧4字节格式, 不修改频率)");
    if (echo_len == 5 && freq > CPLD_FREQ_MAX) {
        printf("[SetScanChannels] ⚠️ 频率系数越界: %u (有效 1~256), 本次不下发频率帧, 通道选择照常\n",
               (unsigned)freq);
    }

    pthread_mutex_lock(&g_scan_queue_lock);

    // [2026-11-02 二次修复] 陈旧队列清理: 上一轮的队列如果几秒钟都没被 0x1D 取走,
    //   说明那条 0x1D 丢了(或上位机重试了整点)。此时若继续往后追加, 下一次扫描会
    //   多扫一个通道、多回整整 1000 帧 0x1E, 上位机按单通道的组去收 -> 整组串组。
    long long now_ms = task_now_us() / 1000;
    if (g_scan_queue_len > 0 && g_scan_queue_ts > 0 &&
        now_ms - g_scan_queue_ts > SCAN_QUEUE_STALE_MS) {
        printf("[SetScanChannels] ⚠️ 上一轮的 %d 个通道已停留 %lldms 未被 0x1D 取走"
               "(说明那条 0x1D 丢了), 先清空队列再入队, 避免下次扫出 2 倍帧数\n",
               g_scan_queue_len, now_ms - g_scan_queue_ts);
        g_scan_queue_len = 0;
    }

    // 检查队列是否已满
    if (g_scan_queue_len >= MAX_SCAN_CHANNELS) {
        printf("[SetScanChannels] ⚠️ 队列已满(%d), 丢弃通道 collect=%u\n",
               MAX_SCAN_CHANNELS, collect_ch);
        pthread_mutex_unlock(&g_scan_queue_lock);
        task_reply_u32(task_arg, 0x24, 0xFFFFFFFEu, "SetScanChannels");  // 0xFFFFFFFE 表示队列满
        free(task_arg->data);
        free(task_arg);
        return;
    }
    // [2026-11-02 修复] 幂等: 同一条 0x23 被重发时不能重复入队
    //   场景: 下位机已入队并回了 0x24, 但应答在总线上丢了 -> 上位机超时重发同一条 0x23。
    //   此时上一次执行已经结束(操作码不再"忙"), 去重机制拦不住, 于是同一通道被入队两次,
    //   下一次 0x1D 会扫 2 个通道、回 2 倍帧数, 上位机按 1 个通道的组去收 -> 整组串组。
    //   判据: 队列末尾已经是同一条通道(collect+switch 都相同), 即视为重发, 不再追加。
    if (g_scan_queue_len > 0 &&
        g_scan_queue[g_scan_queue_len - 1].collect == collect_ch &&
        g_scan_queue[g_scan_queue_len - 1].sw == switch_ch) {
        printf("[SetScanChannels] 🔁 重发请求: 尾部已是同一通道(collect=%u, switch=%u), "
               "不重复入队\n", collect_ch, switch_ch);
        pthread_mutex_unlock(&g_scan_queue_lock);
        // [2026-11-03] 重发也补一次频率帧: 频率设置是幂等的, 重发场景下保证频率不丢
        set_scan_freq_frame(switch_ch, freq);
        free(task_arg->data);
        task_arg->data = NULL;
        task_reply(task_arg, 0x24, echo_buf, echo_len, "SetScanChannels");
        free(task_arg);
        return;
    }

    // 添加到队列
    g_scan_queue[g_scan_queue_len].collect = collect_ch;
    g_scan_queue[g_scan_queue_len].sw = switch_ch;
    g_scan_queue_len++;
    // 记住最后一次成功入队的通道: 0x1D 万一在"队列为空"的情况下到达, 用它兜底
    g_last_scan_ch.collect = collect_ch;
    g_last_scan_ch.sw      = switch_ch;
    g_last_scan_valid      = 1;
    g_scan_queue_ts        = task_now_us() / 1000;
    // [2026-10-13] 移除 SD 落盘(lxlceshi.txt): 现场不使用 SD 卡
    // 把整条队列拼成一行输出: 逐个 printf 会让日志前缀插到行中间
    {
        char line[2048];
        int off = snprintf(line, sizeof(line), "[SetScanChannels]   当前队列(%d 个): ",
                           g_scan_queue_len);
        for (int i = 0; i < g_scan_queue_len && off > 0 && off < (int)sizeof(line); i++) {
            off += snprintf(line + off, sizeof(line) - (size_t)off,
                            "(collect=%u,sw=%u) ",
                            g_scan_queue[i].collect, g_scan_queue[i].sw);
        }
        printf("%s\n", line);
    }

    pthread_mutex_unlock(&g_scan_queue_lock);

    // [2026-11-04] 频率随通道选择一起下发(在锁外、回包前): 发 5 字节帧并等 FPGA 回显,
    //   回显通常 1~2ms, 最长 5ms; 成功/未确认都只打日志, 不阻塞 0x24 回包
    set_scan_freq_frame(switch_ch, freq);

    free(task_arg->data);
    task_arg->data = NULL;
    printf("[SetScanChannels] ✅ 任务完成: 已入队 %d 个通道\n", g_scan_queue_len);
    // 回包 0x24: 与请求同宽, 原样回显收到的数据 (collect+switch+freq)
    printf("[SetScanChannels]   回包 0x24 回显数据: %u 字节 (collect=%u, switch=%u, freq=%u)\n",
           (unsigned)echo_len, collect_ch, switch_ch, (unsigned)freq);
    task_reply(task_arg, 0x24, echo_buf, echo_len, "SetScanChannels");
    free(task_arg);
}


// ============================================================================
// [2026-11-02] 兜底补救: 上位机的 0x23 帧"没整帧重建"时, 把该测试点救回来
//
//   现场证据(2026_11_02_1_scan.log, 9 个测试点, 1732/1733 两点失败):
//     每个测试点是 0x1B -> 0x23 -> 0x1D 三步, 间隔都是 ~13ms。失败那两点
//     的第 2 步收到的却是(read 13 字节):
//         AB CD 00 00 00 02 1B 02 80 03 E8 | 06 C3
//         └─ 一条完好的 0x1B 帧(11 字节) ─┘   └2字节┘
//     data_len / opcode / 校验 / collect 全是第 1 步那条 0x1B 的内容, 只有最后
//     2 字节(开关通道号)是新的, 而且比本次要设的通道落后一格(1731 / 1732,
//     而该轮实际缺的正是 1732 / 1733)。
//     => 上位机 write() 的时候发送缓冲区只被更新了一部分(帧头还是上一条命令的),
//        下位机只能按 0x1B 回 0x1C; 上位机却在等 0x24 -> 空等 5.3s -> 放弃该点。
//
//   开启本补救后: 只要能确认"尾巴 == 最后一次入队的通道号"(即确实落后一格),
//   就按 tail+1 入队并回 0x24, 上位机接着发 0x1D, 该点就能正常扫完。
//
//   ⚠️ 风险与核对办法: 判据依赖"通道号连续"这一实测规律。回包 0x24 会原样回显
//   实际入队的通道号, 上位机可自行校验; 日志里也会打印。若某次补救的 switch 与
//   上位机期望不一致(例如测试点序列在坏帧处发生跳变 1735 -> 1789), 请关掉本开关。
//
//   [2026-11-03] 0x23 改为 5 字节(末尾多 1 字节频率系数)后, 坏帧尾巴可能是
//   2 字节(旧格式)或 3 字节(新格式, 开关卡2字节+频率1字节)。本补救只认前 2 字节
//   开关卡通道号, 频率字节丢弃不下发(补救路径保持"不改频率"), 回包仍按 4 字节旧格式
//   回显 —— 本开关默认关闭(SCAN_CHANNEL_RESCUE=0), 仅作诊断, 不影响正常链路。
// ============================================================================
#define SCAN_CHANNEL_RESCUE   0     // 0=只诊断不补救, 1=开启补救

int task_rescue_broken_set_channels(command_t *cmd, const uint8_t *tail, int tail_len) {
    if (cmd == NULL || tail == NULL || (tail_len != 2 && tail_len != 3)) {
        return 0;
    }
    uint16_t tail_sw = (uint16_t)(((uint16_t)tail[0] << 8) | (uint16_t)tail[1]);

#if !SCAN_CHANNEL_RESCUE
    printf("[SetScanChannels] ⚠️ 收到疑似未整帧重建的帧: 0x1B 后面多了 2 字节 0x%04X"
           "(上位机这一步本应发 0x23, 它在等 0x24 却收到 0x1C -> 空等后放弃该点)。"
           "补救开关 SCAN_CHANNEL_RESCUE=0, 本次未补救\n", tail_sw);
    return 0;
#else
    if (!g_last_scan_valid) {
        printf("[SetScanChannels] ⚠️ 补救失败: 还没有过成功入队的通道, 无法推断本轮通道\n");
        return 0;
    }
    if (tail_sw != g_last_scan_ch.sw) {
        printf("[SetScanChannels] ⚠️ 补救失败: 尾巴 0x%04X 与最后一次入队通道 %u 不一致,"
               "无法确认它落后一格\n", tail_sw, g_last_scan_ch.sw);
        return 0;
    }
    if (atomic_load(&adc_stop_flag) != 1) {
        printf("[SetScanChannels] ⚠️ 补救失败: adc_stop_flag=%d (需先置1)\n",
               atomic_load(&adc_stop_flag));
        return 0;
    }

    uint16_t collect_ch = g_last_scan_ch.collect;
    uint16_t switch_ch  = (uint16_t)(g_last_scan_ch.sw + 1u);
    uint32_t echo_data  = ((uint32_t)collect_ch << 16) | (uint32_t)switch_ch;

    // 与正常 0x23 一致: 入队前先确保 PD14/PD15 为低
    if (pd1415_pulse_cancel()) {
        printf("[SetScanChannels]   补救路径: PD14/PD15 已在脉冲自然结束前被拉低\n");
    }

    long long now_ms = task_now_us() / 1000;
    pthread_mutex_lock(&g_scan_queue_lock);
    if (g_scan_queue_len > 0 && g_scan_queue_ts > 0 &&
        now_ms - g_scan_queue_ts > SCAN_QUEUE_STALE_MS) {
        printf("[SetScanChannels] ⚠️ 补救路径: 上一轮 %d 个通道停留 %lldms 未被取走, 先清空\n",
               g_scan_queue_len, now_ms - g_scan_queue_ts);
        g_scan_queue_len = 0;
    }
    if (g_scan_queue_len >= MAX_SCAN_CHANNELS) {
        pthread_mutex_unlock(&g_scan_queue_lock);
        printf("[SetScanChannels] ❌ 补救失败: 队列已满\n");
        return 0;
    }
    // 幂等: 尾部已是同一通道说明已经补救过(或上位机补发了正确的 0x23)
    if (g_scan_queue_len > 0 &&
        g_scan_queue[g_scan_queue_len - 1].collect == collect_ch &&
        g_scan_queue[g_scan_queue_len - 1].sw == switch_ch) {
        pthread_mutex_unlock(&g_scan_queue_lock);
        printf("[SetScanChannels] 🔁 补救路径: 尾部已是 (collect=%u, switch=%u), 不重复入队\n",
               collect_ch, switch_ch);
        task_reply_u32(cmd, 0x24, echo_data, "SetScanChannels");
        return 1;
    }

    g_scan_queue[g_scan_queue_len].collect = collect_ch;
    g_scan_queue[g_scan_queue_len].sw      = switch_ch;
    g_scan_queue_len++;
    g_last_scan_ch.collect = collect_ch;
    g_last_scan_ch.sw      = switch_ch;
    g_last_scan_valid      = 1;
    g_scan_queue_ts        = now_ms;
    pthread_mutex_unlock(&g_scan_queue_lock);

    printf("[SetScanChannels] 🆘 补救成功: 上位机 0x23 帧未整帧重建(尾巴 0x%04X), "
           "已按 (collect=%u, switch=%u) 入队并回包 0x24 —— "
           "请核对是否与上位机本轮期望的通道一致\n", tail_sw, collect_ch, switch_ch);
    task_reply_u32(cmd, 0x24, echo_data, "SetScanChannels");
    return 1;
#endif
}

// [2026-10-15] 通道切换后"弛豫丢弃"参数:
//   被测节点为容性+高阻源, 空闲期间被激励经被测件慢慢充高; 开始采样后测量通路
//   把节点拉回平衡点(实测 τ≈2s, 1.5V→0.44V 平台)。不丢弃的话, 隔久了的第一次
//   扫描会看到整条衰减曲线(实测 10 次约出现 1~2 次)。
//   策略: 每 SCAN_SETTLE_BLOCK 点求块均值, 相邻块均值差 < SCAN_SETTLE_DIFF_LSB
//   视为已稳定(节点已稳时约 0.4s 通过; 有弛豫时按上限强制开采)。
//   [2026-10-16] 上限从 1000 点(≈5.7s)降到 350 点(≈2.0s, 实测约 5.7ms/点):
//   用户反馈"等待太久", 但保留 2 秒确保慢弛豫节点也能等稳。

//2026-10-26    SCAN_SETTLE_BLOCK:20 -> 100
#define SCAN_SETTLE_BLOCK        100     // 块大小(点), ~0.11s
//2026-10-26    SCAN_SETTLE_DIFF_LSB:150 -> 30
#define SCAN_SETTLE_DIFF_LSB     30    // 稳定判据: 相邻块均值差 < 150 LSB(≈14mV)
//2026-10-26    SCAN_SETTLE_MAX_POINTS:350 -> 2400
#define SCAN_SETTLE_MAX_POINTS   2400    // 丢弃上限(~2.0s), 超时强制进入正式采样

// ==================== PD14/PD15 同步脉冲(异步执行) ====================
// [2026-10-23 修复 连续多通道扫描丢包]
//   原实现: 扫描任务在发完 0x1E 结束帧之后, 自己 usleep(1s) 等 PD14/PD15 脉冲
//           结束再拉低, 然后才 return。
//   问题:   0x1D 的"单飞占用"标记(g_opcode_busy)要等任务函数 return 之后,
//           才由 protocol.c 的 exec_wrapper() 调用 opcode_release() 释放。
//           于是每次扫描结束后, 0x1D 还要再"忙"整整 1 秒。
//   现场现象: 上位机连续扫多个不同通道时, 收到 0x1E 结束帧后立刻发下一条 0x1D,
//           正好落在这 1 秒窗口内 -> 被 dispatch_task() 判为"重复指令"丢弃,
//           回 0xF1(错误码 0x1D00 = "当前操作忙"), 且一条 0x1E 数据都不发。
//   修法:   把"拉高保持 N 毫秒再拉低"挪到独立的一次性线程里, 扫描任务发完
//           结束帧立即 return, 0x1D 随即释放, 忙窗口只剩收尾的几十微秒。
//   ⚠️ [2026-10-23 二次修复] 异步化之后暴露了硬件耦合:
//   实测(2026_10_23_scan.log)每一次扫描, 数据从 ~0V 跳到正确值的时刻, 都精确等于
//   "pd1415 恢复低电平"的时刻, 例如:
//       00:08:25.371 第111次 ADC=131095 (0.002V)
//       00:08:25.401 PD14/PD15 已恢复低电平     <- 同一时刻
//       00:08:25.437 第121次 ADC=155172 (2.259V)
//   即: **PD14/PD15 为高电平时, ADS8691 读到的是 ~0V 无效值**(疑似该引脚驱动了
//   放电/复用/切换回路, 把测量通路拉到地)。旧代码因为要在任务里 sleep 1s 等脉冲
//   结束才 return, 下一次扫描必然已经在脉冲之后, 所以掩盖了这个耦合;
//   异步化后下一次扫描可以抢在脉冲结束前开始 -> 前约 0.9 秒(前 ~100~120 点)全是
//   0V 垃圾值, 正是用户反馈的"之后扫描的数据前 100 次异常"。
//   修法: 新扫描开始前调用 pd1415_pulse_cancel(), 打断尚未走完的脉冲并立即拉低,
//   保证整个采集过程中 PD14/PD15 恒为低。代价: 连续扫描时同步脉冲会被截断,
//   这是"不忙 1 秒"与"采到有效数据"二者不可兼得的必然取舍。
// [2026-11-02 三次修订] 脉冲宽度从 1000ms 改成 300ms, 让脉冲"每次都能完整走完":
//   实测(2026_11_02_1_scan.log 共 6 轮)上位机的回访节奏: 扫描结束 -> +331~905ms 发下一条命令,
//   其中 0x23 最早在扫描结束 +331(0x0D) +13(0x1B) +13(0x23) ≈ +357ms 到达。
//   取 1000ms 意味着**每一次**脉冲都会被 0x23 打断, 于是实际宽度随上位机节奏在
//   593 / 765 / 935 / 1002ms 之间乱跳(示波器上就是"时序乱")。
//   改成 300ms 后: 300 < 357, 脉冲永远在 0x23 之前自然结束, 宽度恒为 300ms。
//   若外部设备需要更长的同步窗口, 把这里调大即可 —— 只要仍小于 ~350ms, 脉冲依然完整;
//   超过就会被 0x23 打断, 日志会打印"脉冲被提前结束: 只保持 Xms"。
#define PD1415_PULSE_MS   300           // 同步脉冲高电平保持时间(ms)

static pthread_mutex_t g_pd1415_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned long   g_pd1415_seq  = 0;   // 脉冲序号: 只有最新一条脉冲线程有权拉低
static int             g_pd1415_high_now = 0;  // 当前是否正处于脉冲高电平
static long long       g_pd1415_high_ms  = 0;  // 本次脉冲拉高的时刻(ms), 用于报告实际宽度

static long long pd1415_now_ms(void) {
    return task_now_us() / 1000;
}

// 锁内调用: 只操作引脚并返回本次高电平实际宽度(ms); <0 表示当时并不在高电平
// ⚠️ 这里绝不做 printf: 打印可能阻塞几毫秒, 而 0x23 的处理路径要拿同一把锁,
//    一旦打印在锁内, 0x24 应答就会被拖慢, 反而制造上位机超时重发。
static long long pd1415_do_low_locked(void) {
    pd1415_low();
    long long now = pd1415_now_ms();
    long long w = -1;
    if (g_pd1415_high_ms > 0 && now >= g_pd1415_high_ms) {
        w = now - g_pd1415_high_ms;
    }
    g_pd1415_high_ms = 0;
    return w;
}

// 锁外调用: 把本次脉冲宽度写进日志(宽度对不上就是"时序乱"的直接证据)
static void pd1415_report(long long width_ms, const char *reason) {
    if (width_ms < 0) {
        printf("[pd1415] ✅ PD14/PD15 已拉低 (%s)\n", reason);
        return;
    }
    if (width_ms + 10 < PD1415_PULSE_MS) {
        printf("[pd1415] ⚠️ 脉冲被提前结束: 只保持 %lldms / 设定 %dms (%s)\n",
               width_ms, PD1415_PULSE_MS, reason);
    } else {
        printf("[pd1415] ✅ 脉冲完整: 高电平 %lldms / 设定 %dms (%s)\n",
               width_ms, PD1415_PULSE_MS, reason);
    }
}

static void *pd1415_pulse_worker(void *arg) {
    unsigned long my_seq = (unsigned long)(uintptr_t)arg;

    usleep(PD1415_PULSE_MS * 1000);

    pthread_mutex_lock(&g_pd1415_lock);
    int mine = (my_seq == g_pd1415_seq) && g_pd1415_high_now;
    long long width = -1;
    if (mine) {
        g_pd1415_high_now = 0;
        width = pd1415_do_low_locked();
    }
    unsigned long cur_seq = g_pd1415_seq;
    pthread_mutex_unlock(&g_pd1415_lock);

    if (mine) {
        pd1415_report(width, "定时到");
    } else {
        // 被新扫描打断, 或已被更新的脉冲接管: 老线程不能再拉低(否则会把新脉冲截短)
        printf("[pd1415]   旧脉冲线程退场(当前序号=%lu, 本线程=%lu), 不再操作引脚\n",
               cur_seq, my_seq);
    }
    return NULL;
}

// 触发一次"拉高 -> 保持 PD1415_PULSE_MS 毫秒 -> 拉低"的同步脉冲, 不阻塞调用者
static int pd1415_pulse_async(void) {
    pthread_t tid;
    unsigned long seq;
    int rc;

    // 置位与拉高在同一把锁内完成, 避免与 pd1415_pulse_cancel() 交错
    // (否则可能出现"cancel 先拉低、随后这里才拉高"的漏网窗口)
    pthread_mutex_lock(&g_pd1415_lock);
    seq = ++g_pd1415_seq;
    g_pd1415_high_now = 1;
    rc = pd1415_high();
    if (rc == 0) {
        g_pd1415_high_ms = pd1415_now_ms();
    }
    pthread_mutex_unlock(&g_pd1415_lock);

    if (rc != 0) {
        pthread_mutex_lock(&g_pd1415_lock);
        g_pd1415_high_now = 0;
        g_pd1415_high_ms  = 0;
        pthread_mutex_unlock(&g_pd1415_lock);
        printf("[pd1415] ❌ 脉冲触发失败: PD14/PD15 置高失败\n");
        return -1;
    }

    if (pthread_create(&tid, NULL, pd1415_pulse_worker,
                       (void *)(uintptr_t)seq) != 0) {
        printf("[pd1415] ❌ 脉冲线程创建失败, 立即拉低兜底\n");
        pthread_mutex_lock(&g_pd1415_lock);
        g_pd1415_high_now = 0;
        long long w = pd1415_do_low_locked();
        pthread_mutex_unlock(&g_pd1415_lock);
        pd1415_report(w, "线程创建失败兜底");
        return -1;
    }
    pthread_detach(tid);
    printf("[pd1415] ▶ 同步脉冲开始(序号=%lu, 高电平 %d ms, 扫描结束起算)\n",
           seq, PD1415_PULSE_MS);
    return 0;
}

// 打断尚未结束的同步脉冲: 立即拉低, 并让挂起的脉冲线程失效。
// 返回 1 = 确实打断了一个还在高电平的脉冲, 0 = 本来就已经是低电平。
// 正常情况下脉冲 300ms 就自然结束了(上位机最快 331ms 后才回来), 这里应该是空操作;
// 一旦真的打断到, 说明上位机比平时更早回访, 日志会打印实际宽度便于核对。
static int pd1415_pulse_cancel(void) {
    pthread_mutex_lock(&g_pd1415_lock);
    int was_high = g_pd1415_high_now;
    g_pd1415_high_now = 0;
    g_pd1415_seq++;                 // 让挂起线程到点后不再拉低(它已过期)
    long long width = -1;
    if (was_high) {
        width = pd1415_do_low_locked();
    }
    pthread_mutex_unlock(&g_pd1415_lock);

    if (was_high) {
        pd1415_report(width, "下一次扫描开始前");
    }
    return was_high;
}

// ==================== 扫描收尾: 关闭全部开关通道 ====================
// [2026-10-23] 用户要求: 自动扫描(0x1D)结束后关闭所有通道, 断开输出回路,
//   使示波器测量时不再有开关切换 / 激励残留波形。
//
// ⚠️ 与 [2026-10-15] 的决议相反: 当时发现关断后被扫节点失源, 电压从激励值
//   指数缓降(τ≈2s, 1.58V->0.43V 平台), 为此撤销过扫描结束关断、恢复"激励保持"。
//   本次是按当前需求重新开启。若后续需要保压, 把下面的
//   SCAN_DISABLE_ALL_ON_DONE 改成 0 即可一键回退, 不必再翻历史提交。
#ifndef SCAN_DISABLE_ALL_ON_DONE
#define SCAN_DISABLE_ALL_ON_DONE   1
#endif

// reason: 调用原因, 仅用于日志区分"正常扫完"与"被 0x21 中断"
static void scan_disable_all_switch(const char *reason) {
#if SCAN_DISABLE_ALL_ON_DONE
    printf("[AutoScan] ▶ 收尾: 关闭全部开关通道(%s)\n", reason);

    // 1) CPLD 广播: 向 0x00~0x0F 全部 16 个卡地址发 CH=0, 断开各卡已选通道
    int r1 = disable_all_switch_via_cpld();
    // 2) GPIO: PA11 / PB0 拉低, 释放 0-7 / 8-15 两组地址锁存
    //    (内部还会补发一帧 CPLD 0x00 0x00)
    int r2 = disable_all_switch();

    printf("[AutoScan] %s 全部开关通道已关闭(cpld_ret=%d, gpio_ret=%d), 输出端已静默\n",
           (r1 == 0 && r2 == 0) ? "✅" : "⚠️", r1, r2);
#else
    (void)reason;
    printf("[AutoScan]   跳过关闭通道: SCAN_DISABLE_ALL_ON_DONE=0(保压模式)\n");
#endif
}

// ==================== 采样值软件滤波(默认全部关闭) ====================
// [2026-10-23] 对 0x1D 自动扫描逐点采样值做平滑。三种算法互斥, 默认一段都不启用
//   (保持原样输出原始 ADC 值)。启用步骤见下方"用法"。
//
// ⚠️ 三种滤波各有代价, 选之前先想清楚要滤什么:
//   - 一阶低通: 对白噪声最省资源(O(1)), 但会引入相位滞后, 信号跳变沿被抹平;
//   - 滑动均值: 对白噪声抑制最强, 同样抹平跳变沿, 且对尖峰(毛刺)几乎无效;
//   - 滑动中值: 专门吃单点尖峰/毛刺, 且能保住跳变沿, 但对白噪声的抑制弱于均值。
//   连续扫多通道时, 通道切换的瞬态已由前面的"弛豫丢弃"处理, 滤波主要压稳态噪声。
//
// 用法(三步都做, 少一步编译不过或用不上):
//   1. 把本段里的宏定义块(含 SCAN_FILTER_*)整体解开注释;
//   2. 把要用的那种滤波对应的"状态变量 + 实现函数"整体解开注释
//      (三种可以全解开放着, 靠 SCAN_FILTER 宏决定实际走哪条路, 不冲突);
//   3. 把下面两处调用行的注释解开:
//        - 每通道弛豫结束后: scan_filter_reset();
//        - 采样循环内:       adc_val = scan_filter_apply(adc_val);
//
// #define SCAN_FILTER_NONE    0
// #define SCAN_FILTER_LPF     1
// #define SCAN_FILTER_MEAN    2
// #define SCAN_FILTER_MEDIAN  3
// #define SCAN_FILTER         SCAN_FILTER_LPF   // <- 改这里切换算法

// #define SCAN_FILTER_ALPHA   0.2f   // 一阶低通系数(0,1]: 越小越平滑、滞后越大
// #define SCAN_FILTER_WINDOW  7      // 均值/中值窗口点数: 中值请用奇数
//                                    // 窗口越大越平滑, 滞后 = (WINDOW-1)/2 个采样点

// ---------- 一阶低通(IIR): y[n] = y[n-1] + alpha * (x[n] - y[n-1]) ----------
// 每段都用 #if 包住: 三段可以全解开放着, 只有被 SCAN_FILTER 选中的那段参与编译,
// 不会产生"static 函数未使用"告警。
// #if SCAN_FILTER == SCAN_FILTER_LPF
// static float    g_lpf_y    = 0.0f;
// static int      g_lpf_init = 0;
// static uint32_t scan_filter_lpf(uint32_t x) {
//     if (!g_lpf_init) {              // 首点直接建立, 避免从 0 缓慢爬升
//         g_lpf_y    = (float)x;
//         g_lpf_init = 1;
//     } else {
//         g_lpf_y += SCAN_FILTER_ALPHA * ((float)x - g_lpf_y);
//     }
//     return (uint32_t)(g_lpf_y + 0.5f);
// }
// #endif

// ---------- 滑动均值: 窗口内算术平均 ----------
// #if SCAN_FILTER == SCAN_FILTER_MEAN
// static uint32_t g_mean_buf[SCAN_FILTER_WINDOW];
// static int      g_mean_idx = 0;
// static int      g_mean_cnt = 0;
// static uint32_t scan_filter_mean(uint32_t x) {
//     g_mean_buf[g_mean_idx] = x;
//     g_mean_idx = (g_mean_idx + 1) % SCAN_FILTER_WINDOW;
//     if (g_mean_cnt < SCAN_FILTER_WINDOW) g_mean_cnt++;
//     uint32_t sum = 0;
//     for (int i = 0; i < g_mean_cnt; i++) sum += g_mean_buf[i];
//     return sum / (uint32_t)g_mean_cnt;
// }
// #endif

// ---------- 滑动中值: 窗口内取中位数(插入排序, 窗口小所以开销可接受) ----------
// #if SCAN_FILTER == SCAN_FILTER_MEDIAN
// static uint32_t g_med_buf[SCAN_FILTER_WINDOW];
// static int      g_med_idx = 0;
// static int      g_med_cnt = 0;
// static uint32_t scan_filter_median(uint32_t x) {
//     g_med_buf[g_med_idx] = x;
//     g_med_idx = (g_med_idx + 1) % SCAN_FILTER_WINDOW;
//     if (g_med_cnt < SCAN_FILTER_WINDOW) g_med_cnt++;
//
//     uint32_t tmp[SCAN_FILTER_WINDOW];
//     for (int i = 0; i < g_med_cnt; i++) tmp[i] = g_med_buf[i];
//     for (int i = 1; i < g_med_cnt; i++) {          // 插入排序(升序)
//         uint32_t key = tmp[i];
//         int j = i - 1;
//         while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
//         tmp[j + 1] = key;
//     }
//     return tmp[g_med_cnt / 2];                     // 奇数窗口=正中; 偶数取偏大一侧
// }
// #endif

// ---------- 统一入口 + 状态复位 ----------
// 每个通道切换时调用一次, 清空窗口历史, 防止上一通道的残值污染本通道开头几点
// static void scan_filter_reset(void) {
// #if SCAN_FILTER == SCAN_FILTER_LPF
//     g_lpf_init = 0;
// #elif SCAN_FILTER == SCAN_FILTER_MEAN
//     g_mean_idx = 0; g_mean_cnt = 0;
// #elif SCAN_FILTER == SCAN_FILTER_MEDIAN
//     g_med_idx  = 0; g_med_cnt  = 0;
// #endif
// }
// static uint32_t scan_filter_apply(uint32_t x) {
// #if SCAN_FILTER == SCAN_FILTER_LPF
//     return scan_filter_lpf(x);
// #elif SCAN_FILTER == SCAN_FILTER_MEAN
//     return scan_filter_mean(x);
// #elif SCAN_FILTER == SCAN_FILTER_MEDIAN
//     return scan_filter_median(x);
// #else
//     return x;
// #endif
// }

// ==================== 自动 ADC 采集 ====================
// 从队列取通道, 每通道采 Times_Read_ADC121 次(0x1B 设置), 发送原始数据
void Task_ADC_Auto_Scan(void *arg) {
    command_t *task_arg = (command_t *)arg;
    if (task_arg->data) { free(task_arg->data); task_arg->data = NULL; }
    uint8_t echo[2];
    // 打印：任务开始
    printf("[AutoScan] ▶ 任务开始: 自动 ADC 扫描, 每通道采样=%d 次\n", Times_Read_ADC121);
 atomic_store(&HV_stop_flag, 0);

    // [2026-10-23] 采集前必须先确保 PD14/PD15 为低: 上一次扫描结束时的 1 秒同步
    // 脉冲可能还没走完, 而该引脚为高会把 ADS8691 的读数压到 ~0V(实测), 导致本次
    // 扫描前 ~100 个点全是垃圾值。这里打断未结束的脉冲并立即拉低。
    // [2026-11-02] 兜底: 正常情况下 0x23 那一步已经把它拉低了(见 Task_Set_Scan_Channels)。
    //   走到这里还能打断到高电平, 说明上位机这次没先发 0x23 就直接发了 0x1D ——
    //   此时拉低落在"扫描开始之后", 上位机可能误判为"扫描结束", 所以显式告警便于取证。
    if (pd1415_pulse_cancel()) {
        printf("[AutoScan] ⚠️ 0x1D 到达时 PD14/PD15 仍为高电平(此前未收到 0x23): "
               "拉低发生在扫描开始之后, 上位机可能误判为扫描已结束\n");
    }

    // [2026-11-02] adc_stop_flag 的置 0 挪到"队列已取到"之后:
    //   0x23(入队)要求 adc_stop_flag==1 才受理, 若这里先置 0, 一旦 0x23 的线程
    //   比 0x1D 晚跑, 它就会被判"设置被拒"而不入队 -> 本次扫描 0 个通道 -> 该点必失败。
    //   放到后面既保留了"采集中标志位为 0"的原语义, 又给 0x23 留出了执行窗口。

    uint32_t total_reads = 0;
    int samples_per_ch = Times_Read_ADC121;

    // 先把队列快照到本地, 加锁区间尽量短:
    //   1. 扫描过程中不再持有队列锁, 不会阻塞 SelChannel/SetScanChannels;
    //   2. 避免循环里对非递归锁重复 lock 造成死锁
    //      (原循环末尾的 lock 没有配对 unlock, 队列里有第 2 个通道时就会卡死)。
    scan_channel_t local_queue[MAX_SCAN_CHANNELS];
    int queue_len = 0;

    // [2026-11-02] 队列为空时短等一会儿: 0x23 与 0x1D 是两条命令, 由线程池里
    //   不同的线程执行, 理论上 0x1D 有可能先跑完(队列还没被 0x23 填上)。
    //   原来遇到这种情况会"扫 0 个通道 -> 本次一帧 0x1E 都不发", 上位机直接判该点失败。
    // [2026-11-02 二次修复] 0x23 已改为在接收线程内同步执行(见 protocol.c), 只要 0x23
    //   先于 0x1D 到达就必定已完成入队, 所以这里只是"万无一失"的兜底, 等待时间收到 50ms。
    for (int wait_ms = 0; ; wait_ms += 10) {
        pthread_mutex_lock(&g_scan_queue_lock);
        queue_len = g_scan_queue_len;
        if (queue_len > MAX_SCAN_CHANNELS) {
            queue_len = MAX_SCAN_CHANNELS;
        }
        if (queue_len > 0) {
            memcpy(local_queue, g_scan_queue, sizeof(scan_channel_t) * (size_t)queue_len);
            g_scan_queue_len = 0;      // 队列已被本次扫描取走, 清空供下次使用
        }
        pthread_mutex_unlock(&g_scan_queue_lock);

        if (queue_len > 0 || wait_ms >= 50) {
            break;
        }
        if (wait_ms == 0) {
            printf("[AutoScan] ⚠️ 扫描队列暂时为空(0x23 可能还没执行完), 最多等 50ms\n");
        }
        usleep(10 * 1000);
    }
    if (queue_len == 0) {
        // [2026-11-02 二次修复] 兜底: 0x23 帧在线路上被截断时会"整条入队丢失",
        //   此时若直接按空队列往下走, 本次一帧 0x1E 都不发, 上位机必然判该点失败。
        //   0x1D 的语义就是"扫我刚入队的那一个通道", 所以队列为空时退回最后一个
        //   入队成功的通道 —— 数据可能属于上一次的通道(日志会明确写出来), 但链路不中断。
        pthread_mutex_lock(&g_scan_queue_lock);
        int have_last = g_last_scan_valid;
        scan_channel_t last = g_last_scan_ch;
        pthread_mutex_unlock(&g_scan_queue_lock);

        if (have_last) {
            local_queue[0] = last;
            queue_len = 1;
            printf("[AutoScan] ⚠️ 队列为空(0x23 未成功入队), 退回最后一次入队成功的通道: "
                   "collect=%u, switch=%u —— 本次数据可能属于上一次测试点, 请核对 0x23 是否丢帧\n",
                   last.collect, last.sw);
        } else {
            printf("[AutoScan] ❌ 队列为空且没有历史通道: 本次不会回任何 0x1E 数据帧"
                   "(上位机会判该点失败)\n");
        }
    }

    // 队列已取到, 现在才把"采集中"标志置 0(见上面挪动的说明)
    atomic_store(&adc_stop_flag, 0);

// card = 1, channel = 101
    printf("[AutoScan]   队列共 %d 个通道, 开始逐个扫描\n", queue_len);

    for (int q = 0; q < queue_len; q++) {
        if (atomic_load(&adc_stop_flag)) break;

        scan_channel_t ch_info = local_queue[q];
        printf("[AutoScan]   通道 %d/%d: collect=%u, switch=%u\n",
               q + 1, queue_len, ch_info.collect, ch_info.sw);

        long long t0 = task_now_us();
        uint8_t card, channel;
        split_num(ch_info.sw, &card, &channel);
            //  cpld_send_hex_command_block(0x03,0x07);
            // [2026-10-13] 检查切通道结果: 原来忽略返回值, CPLD 超时(内部重发 4 次仍失败,
            //   返回 -2)时仍继续往下采, 等于把"开关卡没切过去"的值当成该通道数据 -> 数据错乱。
            // [2026-10-23] 整通道级重试 1 次: 连续扫多通道时, 若 485 总线正被别的
            //   命令占用(内部 4 次重发都撞上), 原来会直接跳过该通道 -> 上位机一直
            //   等这一通道的 0x1E 等不到。这里隔 20ms 再试一次, 仍失败才跳过。
            int cpld_ret = cpld_send_hex_command_wait_reply(card, channel, echo);  // 0=成功
            if (cpld_ret != 0) {
                printf("[adc自动扫描] ⚠️ 开关卡切通道失败(card=%d, ch=%d, ret=%d), 20ms 后重试一次\n",
                       card, channel, cpld_ret);
                usleep(20 * 1000);
                cpld_ret = cpld_send_hex_command_wait_reply(card, channel, echo);
            }
            if (cpld_ret != 0) {
                printf("[adc自动扫描] ❌ 开关卡切通道重试仍失败(card=%d, ch=%d, ret=%d), "
                       "跳过该通道(本通道不会回 0x1E)\n", card, channel, cpld_ret);
                continue;
            }
            // usleep(1000*1000);
            long long t1 = task_now_us();
        // 设置开关卡通道
        // cpld_send_hex_command_block(ch_info.sw, 1);
        Select_Collect_Card_Channel_HV(ch_info.collect);
        long long t2 = task_now_us();

        // [2026-10-13] 通道切换后留稳定时间: 继电器/模拟开关切换后, 信号通路需要
        //   建立时间, 此前"切完立刻采"会把切换瞬态当成读数(每通道首个点尤其明显)。
        //   代价: 每通道 +2ms。多点扫描时每通道只切换一次, 总耗时影响可忽略。
        usleep(2000);

        // [2026-10-15] 弛豫丢弃(自适应, 不回包不计数):
        //   每 SCAN_SETTLE_BLOCK 点求块均值, 相邻块差 < SCAN_SETTLE_DIFF_LSB 判稳定;
        //   SPI 失败样本不计入块均值但计入总量, 保证必然终止。
        {
            uint32_t block_sum = 0, prev_avg = 0;
            int in_block = 0, blocks = 0, settled = 0, dropped = 0;
            while (dropped < SCAN_SETTLE_MAX_POINTS) {
                uint32_t v = ADS8691_Read();
                dropped++;

                // [滤波-可选] 启用滤波时解开: 用弛豫期的读数预热窗口。
                //   弛豫通常丢几十点, 足够填满窗口, 这样正式采样第 1 点就是干净的;
                //   不预热的话, 正式采样开头 WINDOW-1 点处于"窗口填充期", 输出可能
                //   明显偏离(实测中值滤波若不预热, 前 6 点最大偏差可达 3000 LSB)。
                //   注意跳过 SPI 失败值, 别把 0xFFFFFFFF 灌进窗口。
                // if (v != 0xFFFFFFFF) (void)scan_filter_apply(v);

                // [2026-10-16] 丢弃期进度打印: 每 50 点一行, 避免这段时间"像卡住"
                if (dropped % 50 == 0) {
                    printf("[AutoScan]   通道 %u: 弛豫中, 已丢 %d/%d 点, 最新 ADC=%u\n",
                           ch_info.collect, dropped, SCAN_SETTLE_MAX_POINTS, v);
                }
                if (v == 0xFFFFFFFF) continue;   // SPI 失败: 不进块均值
                block_sum += v;
                if (++in_block < SCAN_SETTLE_BLOCK) continue;
                uint32_t avg = block_sum / SCAN_SETTLE_BLOCK;
                block_sum = 0;
                in_block = 0;
                blocks++;
                if (blocks >= 2) {
                    uint32_t d = (avg > prev_avg) ? (avg - prev_avg) : (prev_avg - avg);
                    if (d < SCAN_SETTLE_DIFF_LSB) {
                        if (++settled >= 1) break;   // 连续一次块比较稳定即可
                    } else {
                        settled = 0;
                    }
                }
                prev_avg = avg;
            }
            printf("[AutoScan]   通道 %u: 弛豫丢弃 %d 点后开始正式采样\n",
                   ch_info.collect, dropped);
        }

        // [滤波-调用点 1/2] 启用滤波时解开: 换通道后清空窗口历史
        // scan_filter_reset();

        for (int i = 0; i < samples_per_ch; i++) {
            if (atomic_load(&adc_stop_flag)) break;

             uint32_t adc_val = ADS8691_Read();

            // [滤波-调用点 2/2] 启用滤波时解开: 对采样值套用选定算法
            // 注意: 下行之后的 voltage 计算与 0x1E 上传用的都是滤波后的值;
            //       若想"上传原始值、只在日志里看滤波效果", 改用 adc_val_f 即可。
            // adc_val = scan_filter_apply(adc_val);

            // uint32_t adc_val1 = ADS8691_Read_l();

            //    uint32_t adc_val = ADC121S_Read();
            total_reads++;

            // 满跨度是 ±3*Vref = ±12.288V => 全跨度 24.576V(不是 24.0)
            // 1 LSB = 24.576 / 262143 ≈ 93.75uV
            float voltage = ((float)adc_val - 131072.0f) * 24.576f / 262143.0f;

            // float voltage1 = ((float)adc_val1 - 131072.0f) * 24.576f / 262143.0f;

            // [2026-10-16] 降频打印: 串口 11.5KB/s 追不上逐点打印(实测约 10KB/s),
            // 500 点必然丢行(收尾的"共采集 500 次"行最容易被丢)。
            // 改为每 10 点 1 行 + 每通道最后一点必打, 日志量降到约 1KB/s, 串口不再丢;
            // 采样数据本身仍逐点经 0x1E 帧上传, 不受影响。
            if (i % 10 == 0 || i == samples_per_ch - 1) {
                printf("[adc自动扫描] 第%u次 采集通道=%d 开关通道=%d ADC=%u 电压=%.6f V\n",
                       total_reads,
                       ch_info.collect,
                       ch_info.sw,
                       adc_val,
                       voltage);
            }

            // 打包为 3 个 16 位字(共 48 位), 大端发送:
            //   w0 = adc_val 高 16 位 (bit17..2)
            //   w1 = adc_val 低 2 位 (bit1..0) + collect 14 位
            //   w2 = sw 16 位
            // 拼起来就是: adc_val(18) | collect(14) | sw(16)
            uint16_t w0 = (uint16_t)((adc_val >> 2) & 0xFFFF);
            uint16_t w1 = (uint16_t)(((adc_val & 0x3) << 14) | ((uint32_t)ch_info.collect & 0x3FFF));
            uint16_t w2 = (uint16_t)(ch_info.sw & 0xFFFF);
            task_reply_u16x3(task_arg, 0x1E, w0, w1, w2, "AutoScanSample");
        }
        long long t3 = task_now_us();

        // 单通道总耗时超过 20ms 才打印, 便于定位卡顿在哪一步(正常时静默)
        if (t3 - t0 > 20000) {
            printf("[AutoScan] ⚠️ 通道耗时偏长: ch=%u 选通道=%lldus cpld发送=%lldus 采样%d次=%lldus 总=%lldus\n",
                   ch_info.collect, t1 - t0, t2 - t1,
                   samples_per_ch, t3 - t2, t3 - t0);
        }

        // 此处不再加锁: 原来这里 lock 之后没有配对 unlock, 非递归锁在同一线程
        // 第二次 lock 会直接死锁(队列长度 >= 2 时必现)。
    }

    // [2026-10-16] 区分"正常扫完"与"被中断(0x21 或中途 break)":
    //   正常扫完 -> PD14/PD15 立刻拉高(同步脉冲), 1 秒后拉低;
    //   被中断   -> PD14/PD15 直接保持低, 不再产生高脉冲。
    int aborted = atomic_load(&adc_stop_flag);

    if (aborted) {
        // 中断时同样要打断可能挂着的脉冲(可能是上一次扫描留下的)
        pd1415_pulse_cancel();
        printf("[AutoScan] ⚠️ 采集被中断, PD14/PD15 保持低电平(不脉冲)\n");
    } else {
        // [2026-10-23] 异步脉冲: 立刻返回, 不再让 0x1D 多"忙" 1 秒
        // (原实现在这里 pd1415_high() + 末尾 usleep(1s) + pd1415_low(),
        //  把 0x1D 的占用标记一直拖到脉冲结束才释放)
        pd1415_pulse_async();
    }

    atomic_store(&adc_stop_flag, 1);
    printf("[AutoScan] ✅ 任务完成: 共采集 %u 次\n", total_reads);

    // [2026-10-23] 扫描结束(含被 0x21 中断)一律关闭全部开关通道, 使输出端静默,
    //   示波器测量时不会看到开关切换 / 激励残留波形。
    //   ⚠️ 必须在这里同步完成, 不能挪到异步线程: 上位机收到 0x1E 后会立刻下发
    //   下一次 0x1D, 若关闭动作异步执行, 下一次扫描刚切好通道就被本动作全部关掉,
    //   下一批数据会全废。放在回包前 = 借用 0x1D 的忙窗口做串行化。
    //   ⚠️ 必须晚于脉冲处理: 先脉冲再关的话, 关断瞬态会被示波器拍进脉冲窗口;
    //      先关后脉冲, 整个脉冲窗口内硬件都已是静默状态。
    // scan_disable_all_switch(aborted ? "采集中断" : "正常扫描完成");

    task_reply_u32(task_arg, 0x1E, total_reads, "AutoScanDone");

    // ⚠️ [2026-10-23] 这里之后不能再有任何阻塞动作(usleep/等锁/等 IO):
    // 本函数 return 之后 protocol.c 的 exec_wrapper() 才会释放 0x1D 的占用标记,
    // 多停 1 秒就等于让上位机连续扫下一通道时吃到 0xF1 / 0x1D00 "当前操作忙"。
    // PD14/PD15 的 1 秒脉冲已改由 pd1415_pulse_async() 的独立线程完成。
    free(task_arg);
}



void Task_Adc_Stop(void *arg) {
    command_t *task_arg = (command_t *)arg;
    free(task_arg->data);
    task_arg->data = NULL;
    
    // 打印：任务开始
    printf("[AdcStop] ▶ 任务开始: 停止 ADC 采集\n");
    
    atomic_store(&adc_stop_flag, 1);
    printf("[AdcStop]   已置位停止信号, 采集线程将退出\n");

    // [2026-10-16] 停止采集时同步把 PD14/PD15 拉低(不管是否正处在扫描后的 1 秒高电平窗口)
    // [2026-10-23] 用 cancel 而不是直接 low: 同时要让挂起的脉冲线程失效,
    //              否则它到点后还会再操作一次引脚。
    pd1415_pulse_cancel();
    printf("[AdcStop]   PD14/PD15 已拉低\n");

    task_reply_u8(task_arg, 0x22, 0, "stop");
    free(task_arg);
}



 



void Task_Read_HV_I(void *arg) {  
    command_t *task_arg = (command_t *)arg;
    if (task_arg->data) free(task_arg->data);

    // 打印：任务开始
    printf("[ReadHVI] ▶ 任务开始: 读取高压电流\n");

    // 读取 HV 电流 (ADS1115 通道)
    I = ADS1115_Read_ADC_I();
    // 打印：HV 电流值
    printf("[ReadHVI] ✅ 任务完成: 电流=%u\n", I);
    task_reply_u16(task_arg, 0x20, I, "ReadHVI");
    free(task_arg);
}




void Task_Disable_All_Switch(void *arg){
 command_t *task_arg = (command_t *)arg;
    if (task_arg->data) free(task_arg->data);
    // 打印：任务开始
    // printf("[DisableAllSwitch] ▶ 任务开始: 关闭全部开关\n");
    // disable_all_switch();
    // // 打印：任务完成
    // printf("[DisableAllSwitch] ✅ 全部开关已关闭\n");
    for (uint8_t a = 0x00; a <= 0x0F; a++) {
        uint8_t b = 0x00;
        if (cpld_send_hex_command_block(a, b) != 0) {
            printf("[CPLD] ❌ 发送失败: 帧 0x%02X 0x%02X\n", a, b);
            // 如需失败即停, 在这里 break; 否则继续发送剩余帧
            // break;
        }
    }

    task_reply_u8(task_arg, 0x25, 0, "DisableAllSwitch");
    free(task_arg);
}

// void Task_Realtime_ADC_Print(void *arg) {
//     command_t *task_arg = (command_t *)arg;
//     uint8_t count = 0;
//     if (task_arg->data && task_arg->data_len >= 1) count = task_arg->data[0];
//     free(task_arg->data);
//     if (count == 0) count = 50;                 // 默认打印 50 个采样点

//     // 启动 ADC 采集线程(生产者), 由队列把采集值送过来
//     if (realtime_adc_start() != 0) {
//         fprintf(stderr, "[RT_ADC] producer start failed\n");
//         task_reply_u8(task_arg, 0x22, 0xFF, "RT_ADC");
//         return;
//     }

//     uint16_t val;
//     uint32_t n = 0;
//     printf("[RT_ADC] realtime printing, target=%u samples\n", count);
//     while (n < count) {
//         // 实时接收: 200ms 内无数据则超时退出
//         if (realtime_adc_recv(&val, 200) == 0) {
//             break;                              // 超时退出
//         }
//         printf("[RT_ADC] #%u ADC=%u\n", n, val);
//         n++;
//     }
//     realtime_adc_stop();                        // 停止采集线程
//     printf("[RT_ADC] exit, samples=%u\n", n);
//     task_reply_u32(task_arg, 0x22, n, "RT_ADC");
// }




// void Task_Realtime_VI_Print(void *arg) {
//     command_t *task_arg = (command_t *)arg;
//     uint8_t count = 0;
//     if (task_arg->data && task_arg->data_len >= 1) count = task_arg->data[0];
//     free(task_arg->data);
//     if (count == 0) count = 50;                 // 默认打印 50 个采样点

//     // 启动 U/I 采集线程(生产者), 由队列把 U/I 送过来
//     if (realtime_vi_start() != 0) {
//         fprintf(stderr, "[RT_VI] producer start failed\n");
//         task_reply_u8(task_arg, 0x24, 0xFF, "RT_VI");
//         return;
//     }

//     uint16_t u, i;
//     uint32_t n = 0;
//     printf("[RT_VI] realtime printing, target=%u samples\n", count);
//     while (n < count) {
//         // 实时接收: 200ms 内无数据则超时退出
//         if (realtime_vi_recv(&u, &i, 200) == 0) {
//             break;                              // 超时退出
//         }
//         printf("[RT_VI] #%u U=%u I=%u\n", n, u, i);
//         n++;
//     }
//     realtime_vi_stop();                         // 停止采集线程
//     printf("[RT_VI] exit, samples=%u\n", n);
//     task_reply_u32(task_arg, 0x24, n, "RT_VI");
// }


void LXLCESHI(void *arg) {
    command_t *task_arg = (command_t *)arg;

    // 打印：任务开始
    printf("[LXLCESHI] ▶ 任务开始: I2C 读写测试\n");

    Test_I2C_Write_Read();
    // if (task_arg->data) free(task_arg->data);

    // // 启动时初始化 ADC 一次
    // if (ADC121S_Init() < 0) {
    //     // 打印：ADC 初始化失败
    //     fprintf(stderr, "[TestADC] init failed\n");
    //     free(task_arg);
    //     return;
    // }
    // uint32_t i = 1000;
    // // 循环读取
    // while (i--) {
    //     uint16_t val = ADC121S_Read();
    //     float Vi = (float)val / 4095.0 * 5.0;
    //     // 打印：ADC 读取值
    //     printf("[TestADC] %u -> %.3fV\n", val, Vi);
    //     usleep(10000);
    // }
    // ADC121S_Close();
    printf("[LXLCESHI] ✅ 任务完成\n");
    free(task_arg);
}


void Task_CESHI(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint8_t *payload = task_arg->data;
    uint8_t data1 = payload[0];
    uint8_t data2 = payload[1];
    free(task_arg->data);

    // 打印：任务开始
    printf("[CESHI] ▶ 任务开始: 选择采集卡=%u, 通道=%u\n", data1, data2);

    Select_Collect_Card(data1);
    Select_Collect_Card_Channel(data2);
    // 打印：任务完成
    printf("[CESHI] ✅ 任务完成\n");
    free(task_arg);
    return;
}


void Task_CESHI2(void *arg) {
    command_t *task_arg = (command_t *)arg;

    // [2026-10-14] 调试用: 参数 = [寄存器地址][写入值]
    //   写该寄存器 -> 回读 0x04/0x05 -> 通过应答返回(高字节=0x04, 低字节=0x05)。
    //   一次 OTA 后即可连续试探各控制值, 不必每轮复位板子。
    //   注意: 不再调用 Read_All_Registers(), 避免它的数十行串口输出把应答拖住。
    uint8_t *p = (uint8_t *)task_arg->data;
    uint8_t reg = p[0], val = p[1];
    free(task_arg->data);

    printf("[CESHI2] ▶ 写寄存器 0x%02X = 0x%02X\n", (unsigned)reg, (unsigned)val);

    uint16_t v = FPGA_Write_Reg_And_Read(reg, val);
    task_reply_u16(task_arg, 0xFE, v, "WriteReg");
    printf("[CESHI2]   回读: 0x04=0x%02X, 0x05=0x%02X\n",
           (unsigned)((v >> 8) & 0xFF), (unsigned)(v & 0xFF));

    // 打印：任务完成
    printf("[CESHI2] ✅ 任务完成\n");
    free(task_arg);
    return;
}


void Task_SETBUFF_CESHI(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint8_t *payload = task_arg->data;
    uint8_t data1 = payload[0];
    free(task_arg->data);

    // 打印：任务开始
    printf("[SETBUFF_CESHI] ▶ 任务开始: 设置增益=%u\n", data1);

    FPGA_Set_Buff(data1);
    // 打印：任务完成
    printf("[SETBUFF_CESHI] ✅ 任务完成\n");
    free(task_arg);
    return;
}

// ==================== 打印开关(分模块, 命令 0x66) ====================
// 命令数据: 2 字节大端 16 位掩码, 每一位对应一个(组).c 文件的打印开关
//   bit = 1 -> 打开该模块打印;  bit = 0 -> 关闭
// 位号与文件的对应关系见 include/log.h 里的 LOG_MOD_xxx / log_module_name()
// 回包: opcode 0x67, 数据 2 字节 = 已生效的掩码
void Task_Set_Print_Mask(void *arg) {
    command_t *task_arg = (command_t *)arg;
    uint16_t mask = be16(task_arg->data);
    free(task_arg->data);

    printf("[PrintMask] ▶ 任务开始: 收到打印开关掩码=0x%04X\n", mask);

    // ⚠️ 必须先打印"将要生效"的清单, 再真正应用掩码:
    //    若新掩码把本模块(task)关掉了, 应用之后本文件的 printf 就全部失效了,
    //    那样这份清单就永远打不出来。
    printf("[PrintMask]   将要生效的模块打印开关(%d 位, 1=打开 0=关闭):\n",
           LOG_MOD_COUNT);
    for (int bit = 0; bit < LOG_MOD_COUNT; bit++) {
        printf("[PrintMask]     bit%-2d %-16s : %s\n",
               bit, log_module_name(bit),
               (mask & (1u << bit)) ? "打开" : "关闭");
    }

    unsigned int old_mask = log_get_module_mask();
    log_set_module_mask(mask);
    unsigned int new_mask = log_get_module_mask();

    printf("[PrintMask] ✅ 任务完成: 掩码 0x%04X -> 0x%04X\n", old_mask, new_mask);
    printf("[PrintMask]   提示: 本任务(task)的开关是 bit%d, 若已关闭则不会再看到后续打印\n",
           LOG_MOD_TASK);

    // 回包: 0x66 + 1 = 0x67, 数据为已生效的掩码(大端)
    task_reply_u16(task_arg, 0x67, (uint16_t)new_mask, "PrintMask");
    free(task_arg);
}