#include <stdio.h>
#include <unistd.h>

#include "gpio_app.h"
#include "gpio.h"

#include "stdint.h"
#include "485_cpld.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_GPIO
#include "log.h"
// 单引脚一次操作: 初始化 -> 置位 -> 释放
static int gpio_write_once(int pin, gpio_value_t value) {
    printf("[GPIO_APP] ▶ 单次写引脚: pin=%d, 值=%s\n",
           pin, value == GPIO_HIGH ? "HIGH" : "LOW");

    gpio_t *g = gpio_init(pin, GPIO_DIR_OUT);
    if (g == NULL) {
        printf("[GPIO_APP] ❌ 单次写引脚失败: gpio%d 初始化失败\n", pin);
        return -1;
    }
    int r = gpio_set_value(g, value);
    gpio_close(g);
    if (r != 0) {
        printf("[GPIO_APP] ❌ 单次写引脚失败: gpio%d 置位失败\n", pin);
        return r;
    }
    printf("[GPIO_APP] ✅ 单次写引脚完成: gpio%d = %s\n",
           pin, value == GPIO_HIGH ? "HIGH" : "LOW");
    return r;
}





// ==================== 对外接口 ====================

int select_switch(int value) {

printf("[select_switch] ▶ 开始选择开关: value=%d\n", value);

if (value < 0 || value > 15) {
    printf("[select_switch] ❌ 参数越界: value=%d (应在 0~15)\n", value);
    return -1;
}

int bank_enable, bank_disable;
int addr0, addr1, addr2;

if (value <= 7) {
    // 0-7: PA11使能，PB0禁止
    bank_enable = 11;    // PA11
    bank_disable = 32;   // PB0
    addr0 = 202;         // PG10
    addr1 = 15;          // PA15
    addr2 = 12;          // PA12
} else {
    // 8-15: PB0使能，PA11禁止
    bank_enable = 32;    // PB0
    bank_disable = 11;   // PA11
    addr0 = 66;          // PC2
    addr1 = 65;          // PC1
    addr2 = 64;          // PC0
}

gpio_t *g_en = gpio_init(bank_enable, GPIO_DIR_OUT);
gpio_t *g_dis = gpio_init(bank_disable, GPIO_DIR_OUT);
gpio_t *g_a0 = gpio_init(addr0, GPIO_DIR_OUT);
gpio_t *g_a1 = gpio_init(addr1, GPIO_DIR_OUT);
gpio_t *g_a2 = gpio_init(addr2, GPIO_DIR_OUT);

if (!g_en || !g_dis || !g_a0 || !g_a1 || !g_a2) {
    printf("[select_switch] ❌ 初始化失败: 使能/地址引脚占用未成功\n");
    if (g_en) gpio_close(g_en);
    if (g_dis) gpio_close(g_dis);
    if (g_a0) gpio_close(g_a0);
    if (g_a1) gpio_close(g_a1);
    if (g_a2) gpio_close(g_a2);
    return -1;
}

printf("[select_switch]   使能引脚 gpio%d, 禁止引脚 gpio%d, 地址引脚 %d/%d/%d\n",
       bank_enable, bank_disable, addr0, addr1, addr2);

gpio_set_value(g_en, GPIO_HIGH);
gpio_set_value(g_dis, GPIO_LOW);
gpio_set_value(g_a0, (value & 0x01) ? GPIO_HIGH : GPIO_LOW);
gpio_set_value(g_a1, (value & 0x02) ? GPIO_HIGH : GPIO_LOW);
gpio_set_value(g_a2, (value & 0x04) ? GPIO_HIGH : GPIO_LOW);

printf("[select_switch]   地址位: A2=%d A1=%d A0=%d (bank=%s)\n",
       (value >> 2) & 0x01, (value >> 1) & 0x01, value & 0x01,
       (value <= 7) ? "PA11" : "PB0");

gpio_close(g_en);
gpio_close(g_dis);
gpio_close(g_a0);
gpio_close(g_a1);
gpio_close(g_a2);

printf("[select_switch] ✅ 选择开关完成: value=%d\n", value);
return 0;

    
}

void init_all_gpios(void) {
    // [2026-10-16] 新增 PD14/PD15 (110/111) 作输出, 默认低电平
    int pins[] = {64, 65, 66, 12, 15, 202, 11,  35,  161, 160, 110, 111};
    // PC0, PC1, PC2, PA12, PA15, PG10, PA11, PB0,  PF1, PF0, PD14, PD15

    printf("[init_all_gpios] ▶ 开始初始化全部 GPIO: "
           "gpio64, gpio65, gpio66, gpio12, gpio15, "
           "gpio202, gpio11, gpio35, gpio161, gpio160, gpio110(PD14), gpio111(PD15)\n");
    
    for (int i = 0; i < 12; i++) {
        gpio_t *gpio = gpio_init(pins[i], GPIO_DIR_OUT);
        if (gpio) {
            gpio_set_value(gpio, GPIO_LOW);
            gpio_close(gpio);
            printf("[init_all_gpios]   gpio%d 已初始化为输出低电平\n", pins[i]);
        } else {
            printf("[init_all_gpios] ⚠️ gpio%d 初始化失败, 已跳过\n", pins[i]);
        }
    }
    gpio_t *g = gpio_init(35, GPIO_DIR_OUT);
    if (!g) {
        printf("[init_all_gpios] ❌ 初始化失败: gpio35 占用未成功\n");
        return ;
    }
    gpio_set_value(g, GPIO_HIGH);
    gpio_close(g);
    printf("[init_all_gpios] ✅ 初始化完成: gpio35 已置为高电平\n");
    return ;
}

// ==================== PD14/PD15 同步脉冲输出 ====================
// [2026-10-16] 用途: ADC 扫描(0x1D)全部采样完成后, PD14/PD15 立刻输出高电平,
// 定时 1 秒后恢复低电平, 供外部设备做同步/触发。
// 编号: PD14 = 96 + 14 = 110, PD15 = 96 + 15 = 111
#define PD14_GPIO_NUM   110
#define PD15_GPIO_NUM   111

int pd1415_high(void) {
    gpio_t *g14 = gpio_init(PD14_GPIO_NUM, GPIO_DIR_OUT);
    gpio_t *g15 = gpio_init(PD15_GPIO_NUM, GPIO_DIR_OUT);

    if (!g14 || !g15) {
        printf("[pd1415] ❌ 初始化失败: PD14/PD15 占用未成功\n");
        if (g14) gpio_close(g14);
        if (g15) gpio_close(g15);
        return -1;
    }
    gpio_set_value(g14, GPIO_HIGH);
    gpio_set_value(g15, GPIO_HIGH);
    gpio_close(g14);
    gpio_close(g15);
    printf("[pd1415] ✅ PD14(110)/PD15(111) 已输出高电平\n");
    return 0;
}

int pd1415_low(void) {
    gpio_t *g14 = gpio_init(PD14_GPIO_NUM, GPIO_DIR_OUT);
    gpio_t *g15 = gpio_init(PD15_GPIO_NUM, GPIO_DIR_OUT);

    if (!g14 || !g15) {
        printf("[pd1415] ❌ 初始化失败: PD14/PD15 占用未成功\n");
        if (g14) gpio_close(g14);
        if (g15) gpio_close(g15);
        return -1;
    }
    gpio_set_value(g14, GPIO_LOW);
    gpio_set_value(g15, GPIO_LOW);
    gpio_close(g14);
    gpio_close(g15);
    printf("[pd1415] ✅ PD14(110)/PD15(111) 已恢复低电平\n");
    return 0;
}
int disable_all_switch(void) {
    int pa11 = 11;   // PA11
    int pb0 = 32;    // PB0

    cpld_send_hex_command_block(0x00,00);
    printf("[disable_all_switch] ▶ 开始关闭全部开关\n");

    gpio_t *g_pa11 = gpio_init(pa11, GPIO_DIR_OUT);
    gpio_t *g_pb0 = gpio_init(pb0, GPIO_DIR_OUT);

    if (!g_pa11 || !g_pb0) {
        printf("[disable_all_switch] ❌ 初始化失败: gpio11/gpio32 占用未成功\n");
        if (g_pa11) gpio_close(g_pa11);
        if (g_pb0) gpio_close(g_pb0);
        return -1;
    }

    gpio_set_value(g_pa11, GPIO_LOW);
    gpio_set_value(g_pb0, GPIO_LOW);

    gpio_close(g_pa11);
    gpio_close(g_pb0);

    printf("[disable_all_switch] ✅ 全部开关已关闭: PA11=LOW, PB0=LOW\n");
    return 0;
}

// 通过 CPLD 广播关闭全部 16 路开关(发送 0x00~0x0F 共 16 帧)。
// 说明: 这段逻辑原先写在 disable_all_switch() 的 return 之后, 属于不可达代码,
//       永远不会执行; 这里把它提出来作为独立函数, 需要时由上层显式调用。
int disable_all_switch_via_cpld(void) {
    printf("[disable_all_switch_cpld] ▶ 开始通过 CPLD 广播关闭全部开关 (0x00~0x0F)\n");

    if (cpld_init() != 0) {
        printf("[disable_all_switch_cpld] ❌ 执行失败: CPLD 初始化失败\n");
        return -1;
    }

    for (uint8_t a = 0x00; a <= 0x0F; a++) {
        uint8_t b = 0x00;
        if (cpld_send_hex_command_block(a, b) != 0) {
            printf("[disable_all_switch_cpld] ❌ 发送失败: 帧 0x%02X 0x%02X\n", a, b);
            // 如需失败即停, 在这里 break; 否则继续发送剩余帧
            // break;
        } else {
            printf("[disable_all_switch_cpld]   已发送: 0x%02X 0x%02X\n", a, b);
        }
    }

    printf("[disable_all_switch_cpld] ✅ 广播发送完成: 共 16 帧\n");
    return 0;
}

// FPGA 复位: PE11 拉低复位 -> 延时 -> 拉高释放(低有效复位)
int fpgareset(void) {
    printf("[fpgareset] ▶ 开始复位 FPGA: gpio139 (PE11) 拉低 10ms 后拉高\n");

    gpio_t *g = gpio_init(139, GPIO_DIR_OUT);  // PE11 = 128 + 11 = 139
    if (g == NULL) {
        printf("[fpgareset] ❌ 初始化失败: gpio139 占用未成功\n");
        return -1;
    }
    gpio_set_value(g, GPIO_LOW);
    usleep(10 * 1000);  // 10ms
    gpio_set_value(g, GPIO_HIGH);
    gpio_close(g);

    printf("[fpgareset] ✅ 复位完成: PE11 已拉低 10ms 后释放\n");
    return 0;
}

// MCU-DAC 触发: PD13 拉高后拉低
int mculdac(void) {
    printf("[mculdac] ▶ 开始 MCU-DAC 触发: gpio109 (PD13) 拉高\n");

    gpio_t *g = gpio_init(109, GPIO_DIR_OUT);  // PD13 = 96 + 13 = 109
    if (g == NULL) {
        printf("[mculdac] ❌ 初始化失败: gpio109 占用未成功\n");
        return -1;
    }
    gpio_set_value(g, GPIO_HIGH);
    // usleep(10 * 1000);  // 10ms
    // gpio_set_value(g, GPIO_LOW);
    gpio_close(g);

    // 注意: 下面拉低与 10ms 延时已被注释, 因此这里只完成“拉高”
    printf("[mculdac] ✅ 触发完成: PD13 已置为高电平(拉低步骤当前被注释, 未执行)\n");
    return 0;
}
int mculdac0(void) {
    printf("[mculdac0] ▶ 开始 MCU-DAC 触发: gpio109 (PD13) 拉低\n");

    gpio_t *g = gpio_init(109, GPIO_DIR_OUT);  // PD13 = 96 + 13 = 109
    if (g == NULL) {
        printf("[mculdac0] ❌ 初始化失败: gpio109 占用未成功\n");
        return -1;
    }
    gpio_set_value(g, GPIO_LOW);
    // usleep(10 * 1000);  // 10ms
    // gpio_set_value(g, GPIO_LOW);
    gpio_close(g);

    printf("[mculdac0] ✅ 触发完成: PD13 已置为低电平\n");
    return 0;
}

// [2026-10-14] 调试用: 保持 PD13(LDAC) 电平, 不释放句柄(unexport 会让引脚回默认态)。
//   level: 1=HIGH, 0=LOW。用于验证"同步模式下 LDAC 需要保持低才更新输出"。
static gpio_t *g_dbg_ldac = NULL;
int dbg_keep_ldac(int level)
{
    if (g_dbg_ldac == NULL) {
        g_dbg_ldac = gpio_init(109, GPIO_DIR_OUT);   // PD13 = 96 + 13 = 109
        if (g_dbg_ldac == NULL) {
            printf("[dbg_keep_ldac] ❌ gpio109 初始化失败\n");
            return -1;
        }
    }
    gpio_set_value(g_dbg_ldac, level ? GPIO_HIGH : GPIO_LOW);
    printf("[dbg_keep_ldac] ✅ PD13(LDAC) 保持为 %s\n", level ? "HIGH" : "LOW");
    return 0;
}


// PB3: 读取转换状态 (1=已完成, 0=未完成)
int read_conv_status(void) {
    gpio_t *g = gpio_init(35, GPIO_DIR_IN);  // PB3 = 35
    if (g == NULL) {
        printf("[read_conv_status] ❌ 初始化失败: gpio35 占用未成功\n");
        return -1;
    }
    
    gpio_value_t val = GPIO_LOW;
    if (gpio_get_value(g, &val) != 0) {
        gpio_close(g);
        printf("[read_conv_status] ❌ 读取失败: gpio35 电平读取出错\n");
        return -1;
    }
    gpio_close(g);
    
    printf("[read_conv_status] ✅ 转换状态: gpio35=%s -> %s\n",
           (val == GPIO_HIGH) ? "HIGH" : "LOW",
           (val == GPIO_HIGH) ? "已完成" : "未完成");
    return (val == GPIO_HIGH) ? 1 : 0;
}

// PB1: 复位 (拉低 -> 延时 -> 拉高)
int reset_pb1(void) {
    printf("[reset_pb1] ▶ 开始复位: gpio33 (PB1) 拉低 10ms 后拉高\n");

    gpio_t *g = gpio_init(33, GPIO_DIR_OUT);  // PB1 = 33
    if (g == NULL) {
        printf("[reset_pb1] ❌ 初始化失败: gpio33 占用未成功\n");
        return -1;
    }
    
    gpio_set_value(g, GPIO_LOW);
    usleep(10 * 1000);  // 10ms
    gpio_set_value(g, GPIO_HIGH);
    gpio_close(g);
    
    printf("[reset_pb1] ✅ 复位完成: PB1 已拉低 10ms 后释放\n");
    return 0;
}

// PD13: DAC8562 LDAC 触发 (拉低 -> 延时 -> 拉高)
int dac_ldac_trigger(void) {
    printf("[dac_ldac_trigger] ▶ 开始 LDAC 触发: gpio109 (PD13) 拉低 1ms 后拉高\n");

    gpio_t *g = gpio_init(109, GPIO_DIR_OUT);  // PD13 = 109
    if (g == NULL) {
        printf("[dac_ldac_trigger] ❌ 初始化失败: gpio109 占用未成功\n");
        return -1;
    }
    
    gpio_set_value(g, GPIO_LOW);
    usleep(1 * 1000);  // 1ms
    gpio_set_value(g, GPIO_HIGH);
    gpio_close(g);
    
    printf("[dac_ldac_trigger] ✅ 触发完成: PD13 已拉低 1ms 后释放\n");
    return 0;
}