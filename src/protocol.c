#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "protocol.h"
#include <main.h>
#include "task.h"
#include "tcpserver.h"
#include "uart485.h"
#include "ota.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_PROTO
#include "log.h"

// ==================== 命令单飞(去重)控制 ====================
// 同一操作码在未执行完之前, 不允许再次投递新任务, 防止相同命令并发执行
// (同时覆盖 TCP 与 485 两条通道同时到达同一命令的场景)
#define OPCODE_MAX 0x100
// #define OPCODE_ADC_PRINT 0x1D   // ADC 现场读取: 内部有 detach 发送线程, 释放时机特殊

static pthread_mutex_t g_exec_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t         g_opcode_busy[OPCODE_MAX];

// 尝试占用操作码: 返回1=成功占用, 0=该操作码正在执行中
static int opcode_try_acquire(uint8_t opcode) {
    int ret;
    pthread_mutex_lock(&g_exec_lock);
    ret = (g_opcode_busy[opcode] == 0);
    if (ret) {
        g_opcode_busy[opcode] = 1;
    }
    pthread_mutex_unlock(&g_exec_lock);
    return ret;
}

// 释放操作码占用(由任务真正执行完毕后调用)
int opcode_release(uint8_t opcode)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          {
    pthread_mutex_lock(&g_exec_lock);
    g_opcode_busy[opcode] = 0;
    pthread_mutex_unlock(&g_exec_lock);
    return 0;
}

// 任务包装: 执行完成后自动释放操作码占用标记
// ⚠️ 释放时机 = 任务函数 return 的那一刻。因此任务函数内部在"发完回包之后"
//    不允许再有任何阻塞动作(usleep / 等锁 / 等 IO): 那段时间的操作码仍处于
//    "忙"态, 上位机紧接着发同一条命令就会被判重复指令丢弃(回 0xF1)。
//    [2026-10-23] 0x1D 扫描收尾的 PD14/PD15 1 秒脉冲原先写在任务函数里,
//    导致每次扫描后 0x1D 多忙 1 秒, 连续扫多通道必吃 0xF1/0x1D00;
//    现已改为 task.c 里的 pd1415_pulse_async() 异步线程, 此处无需再特殊处理。
// 注意: w->arg(task_arg) 由任务函数自己释放, 包装器只释放包装结构本身
typedef struct {
    uint8_t opcode;
    void (*function)(void *);
    void *arg;
} exec_wrap_t;

static void exec_wrapper(void *p) {
    exec_wrap_t *w = (exec_wrap_t *)p;
    uint8_t op = w->opcode;
    w->function(w->arg);
    opcode_release(op);
    free(w);
}

// ==================== 协议解析与命令分发 ====================
// 帧格式: AB CD | data_len(4,大端) | opcode(1) | checksum(2,大端) | data
// 校验码 = 整包(不含校验码自身) 64位累加 & 0xFFFF

// 整包校验和: 对(包头+长度+操作码+数据)求和取低16位, 不含校验码自身
static uint16_t compute_checksum(const uint8_t *buf, uint32_t total_len) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < total_len; i++) {
        if (i == PROTOCOL_HEADER_SIZE || i == PROTOCOL_HEADER_SIZE + 1) {
            continue;   // 跳过校验码字段自身
        }
        sum += buf[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

static void *duplicate_task_data(const uint8_t *data, uint32_t data_len) {
    if (data == NULL || data_len == 0) {
        printf("[PARSE] ⚠️ 复制任务数据跳过: data=%p, len=%u\n",
               (const void *)data, (unsigned)data_len);
        return NULL;
    }
    uint8_t *copy = (uint8_t *)malloc(data_len);
    if (copy == NULL) {
        printf("[PARSE] ❌ 复制任务数据失败: 分配 %u 字节内存出错\n", (unsigned)data_len);
        return NULL;
    }
    memcpy(copy, data, data_len);
    return copy;
}



#define OPCODE_BUSY_REPLY      0xF1
#define OPCODE_ERR_CHECKSUM    0xF2
#define OPCODE_ERR_LENGTH      0xF3

void send_error_reply(const command_t *command, uint8_t reply_opcode, uint8_t err_opcode) {
    if (command == NULL) {
        printf("[REPLY] ⚠️ 发送错误回包跳过: command 为 NULL\n");
        return;
    }
    
    uint8_t resp[64];
    uint32_t idx = 0;

    resp[idx++] = 0xAB;
    resp[idx++] = 0xCD;
    resp[idx++] = 0x00;
    resp[idx++] = 0x00;
    resp[idx++] = 0x00;
    resp[idx++] = 0x02;              // data_len = 2
    resp[idx++] = reply_opcode;
    resp[idx++] = 0x00;              // checksum 占位
    resp[idx++] = 0x00;
    resp[idx++] = err_opcode;        // 高8位: 命令码
    resp[idx++] = 0x00;              // 低8位: 0

    uint32_t total_len = idx;
    uint16_t cs = compute_checksum(resp, total_len);
    resp[PROTOCOL_HEADER_SIZE]     = (cs >> 8) & 0xFF;
    resp[PROTOCOL_HEADER_SIZE + 1] = cs & 0xFF;

    app_send_response(command, resp, (int)total_len);
    printf("[REPLY] ❌ 已回错误包: reply_opcode=0x%02X, 出错命令=0x%02X, 共 %u 字节\n",
           reply_opcode, command->opcode, (unsigned)total_len);
}



static int dispatch_task(threadpool_t *pool, void (*task_function)(void *),
                         const command_t *command) {
    if (pool == NULL || command == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 线程池或命令为 NULL\n");
        return -1;
    }
    command_t *task_arg = (command_t *)calloc(1, sizeof(*task_arg));
    if (task_arg == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 分配 task_arg 内存出错\n");
        return -1;
    }
    task_arg->is_valid    = command->is_valid;
    task_arg->opcode      = command->opcode;
    task_arg->data_len    = command->data_len;
    task_arg->channel     = command->channel;
    task_arg->tcp_fd      = command->tcp_fd;
    task_arg->serial_port = command->serial_port;
    task_arg->data        = NULL;
    if (command->data_len > 0) {
        task_arg->data = duplicate_task_data(command->data, command->data_len);
        if (task_arg->data == NULL) {
            printf("[EXEC] ❌ 分发任务失败: 复制命令负载数据出错 (0x%02X)\n", command->opcode);
            free(task_arg);
            return -1;
        }
    }

    // 单飞去重: 同一操作码正在执行时, 不再新开线程执行
    if (opcode_try_acquire(task_arg->opcode) == 0) {
        printf("[EXEC] ⚠️ 0x%02X 正在执行中, 重复指令已丢弃\n", task_arg->opcode);
        send_error_reply(command, 0xF1, task_arg->opcode);
        free(task_arg->data);
        free(task_arg);
        return -2;   // -2 表示重复指令已被丢弃(非错误)
    }

    // 用包装器投递: 任务结束后自动释放占用标记
    exec_wrap_t *wrap = (exec_wrap_t *)calloc(1, sizeof(*wrap));
    if (wrap == NULL) {
        printf("[EXEC] ❌ 分发任务失败: 分配 exec_wrap 内存出错 (0x%02X)\n",
               task_arg->opcode);
        opcode_release(task_arg->opcode);
        free(task_arg->data);
        free(task_arg);
        return -1;
    }
    wrap->opcode   = task_arg->opcode;
    wrap->function = task_function;
    wrap->arg      = task_arg;

    if (threadpool_add(pool, exec_wrapper, wrap) != 0) {
        printf("[EXEC] ❌ 分发任务失败: 投标到线程池出错 (0x%02X)\n", task_arg->opcode);
        opcode_release(task_arg->opcode);
        free(wrap);
        free(task_arg->data);
        free(task_arg);
        return -1;
    }
    return 0;
}



static int dispatch_command_if_len_ok(threadpool_t *pool, const command_t *command,
                                      uint32_t expected_len, void (*task_function)(void *),
                                      const char *command_name) {
    if (command->data_len != expected_len) {
        // 打印：操作码对应的命令负载长度不正确，显示实际长度和期望长度
        // 例如：[EXEC] ❌ 0x01 (detect_collect): 负载长度 3 不合法, 期望 2
        printf("[EXEC] ❌ 0x%02X (%s) 参数长度不合法: 实际 %u 字节, 期望 %u 字节\n",
               command->opcode, command_name, (unsigned)command->data_len,
               (unsigned)expected_len);
        send_error_reply(command,0xf3,command->opcode);
        return -1;
    }
    int rc = dispatch_task(pool, task_function, command);
    if (rc == -2) {
        // 该操作码正在执行中, 重复指令已丢弃(不算错误, 不回包)
        return 0;
    }
    if (rc != 0) {
        printf("[EXEC] ❌ 0x%02X (%s) 分发到线程池失败\n",
               command->opcode, command_name);
        return -1;
    }
    // 打印：命令成功分发到线程池，显示操作码和命令名称
    printf("[EXEC] ✅ 0x%02X (%s) 已分发到线程池\n", command->opcode, command_name);
    return 0;
}

static int execute_command(const command_t *command) {
    if (g_thread_pool == NULL || command == NULL) {
        printf("[EXEC] ❌ 执行命令失败: 线程池或命令为 NULL\n");
        return -1;
    }
    if (!command->is_valid) {
        printf("[EXEC] ❌ 执行命令失败: 命令无效 (is_valid=0)\n");
        return -1;
    }

    switch (command->opcode) {
        // ---------- 1. 板卡检测 ----------
        case 0x01:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Detect_Collection_Card, "detect_collect") != 0)
                return -1;
            break;

        case 0x03:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Detect_Switch_Card, "detect_switch") != 0)
                return -1;
            break;

        // ---------- 2. 设置高压 ----------
        case 0x05:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Set_HV, "set_hv") != 0)
                return -1;
            break;

        // ---------- 3. 读取高压 ----------
        case 0x07:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_Read_HV, "read_hv") != 0)
                return -1;
            break;

        // ---------- 4. 设置操作延时/次数 ----------
        case 0x09:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Control_Times_HV, "set_delay") != 0)
                return -1;
            break;

        // ---------- 5. 自检 ----------
        case 0x0B:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Check_Self, "self_test") != 0)
                return -1;
            break;

        // ---------- 6. 增益 ----------
        case 0x0D:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_Set_Buff, "set_gain") != 0)
                return -1;
            break;

        // ---------- 7. 复位 ----------
        case 0x0F:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Reset_Fpga, "reset_fpga") != 0)
                return -1;
            break;

        // ---------- 8. 使能/开始 ----------
        case 0x11:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Test_En, "enable_start") != 0)
                return -1;
            break;

        // ---------- 9. 停止 ----------
        case 0x13:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
                    Task_Test_Stop, "stop_test") != 0)
                return -1;
            break;

        // ---------- 测试/调试命令 ----------
        // case 0x15:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Select_Collect_Card, "sel_collect") != 0)
        //         return -1;
        //     break;

        // case 0x17:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Select_Switch_Card, "sel_switch") != 0)
        //         return -1;
        //     break;

        // case 0x19:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Select_Collect_Card_Channel, "sel_channel") != 0)
        //         return -1;
        //     break;

        case 0x1B:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Test_Read_ADC121_Times, "set_adc_times") != 0)
                return -1;
            break;

        // case 0x1D:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
        //             Task_Test_Read_ADC121_Print, "print_adc") != 0)
        //         return -1;
        //     break;
            case 0x1D:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_ADC_Auto_Scan, "auto_scan") != 0)
                return -1;
            break;


        case 0x1F:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_Read_HV_I, "read_hv_i") != 0)
                return -1;
            break;
        case 0x21:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
            Task_Adc_Stop, "stop") != 0)
                return -1;
            break;
        case 0x23:
        if (dispatch_command_if_len_ok(g_thread_pool, command, 4,
            Task_Set_Scan_Channels, "scan") != 0)
                return -1;
            break;
        case 0x25:
        if (dispatch_command_if_len_ok(g_thread_pool, command, 8,
            Task_Disable_All_Switch, "disable_switch") != 0)
                return -1;
            break;

        // ---------- OTA 在线升级(单可执行文件 + 回滚) ----------
        // 命令数据(9字节): [0..3]=新程序字节数(大端,0=不校验) [4..7]=CRC32(大端,0=不校验) [8]=模式(0=安装 1=只校验 2=回滚)
        // 任务在专用端口等待 TCP 文件, 校验后备份/替换 /usr/bin/program_arm
        case 0x55:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 9,
                    Task_OTA_Upgrade, "ota_upgrade") != 0)
                return -1;
            break;

        // ---------- 打印开关(分模块, 每文件独立开关) ----------
        // 命令数据(2字节大端): 16 位掩码, 每一位对应一个(组).c 文件的打印开关
        //   bit = 1 -> 打开该模块打印; bit = 0 -> 关闭
        //   例: 掩码 0xFFFF = 全部打开; 0xFFFB = 关闭 bit2(SPI)
        //   位号与文件的对应关系见 include/log.h 的 LOG_MOD_xxx
        // 回包: opcode 0x67, 数据 2 字节 = 已生效的掩码
        case 0x66:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_Set_Print_Mask, "set_print_mask") != 0)
                return -1;
            break;

        // ---------- 实时采集打印任务(队列通信) ----------
        // 0x21: ADC121S 实时采集并打印, 200ms 超时退出
        // case 0x21:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
        //             Task_Realtime_ADC_Print, "rt_adc_print") != 0)
        //         return -1;
        //     break;

        // // 0x23: U/I (ADS1115) 实时采集并打印, 200ms 超时退出
        // case 0x23:
        //     if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
        //             Task_Realtime_VI_Print, "rt_vi_print") != 0)
        //         return -1;
        //     break;

        // ---------- 测试(调试)命令 ----------
        case 0xFC:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 1,
                    Task_SETBUFF_CESHI, "set_buff_test") != 0)
                return -1;
            break;

        case 0xFD:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_CESHI2, "switch_test") != 0)
                return -1;
            break;

        case 0xFE:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_CESHI, "sel_card_test") != 0)
                return -1;
            break;

        case 0xFF:
            if (dispatch_command_if_len_ok(g_thread_pool, command, 2,
                    Task_ADC_Auto_Scan, "auto_scan_ff") != 0)
                return -1;
            break;

        default:
            printf("[EXEC] ❌ 执行命令失败: 未知操作码 0x%02X\n", command->opcode);
            return -1;
    }

    return 0;
}



int app_send_response(const command_t *command, const uint8_t *buf, int len) {
    if (command == NULL || buf == NULL || len <= 0) {
        printf("[REPLY] ❌ 回包失败: 参数无效(cmd=%p, buf=%p, len=%d)\n",
               (const void *)command, (const void *)buf, len);
        return -1;
    }

    if (command->channel == RESPONSE_CHANNEL_RS485) {
        if (command->serial_port == NULL) {
            printf("[REPLY] ❌ 回包失败: RS485 通道的串口句柄为空\n");
            return -1;
        }
        int result = uart485_send(buf, len);
        // [2026-10-13] 成功回执不再打印: 高频回包路径(ADC 扫描每点一包)会刷爆串口。
        // 失败仍打印, 便于定位。
        if (result < 0) {
            printf("[REPLY] ❌ 回包失败: RS485 发送 %d 字节出错\n", len);
        }
        return result;
    }

    // TCP: 优先发给发起命令的客户端, fd 无效时退回"最近发送者"
    if (command->tcp_fd > 0) {
        int result = tcp_server_send_to_fd(command->tcp_fd, buf, len);
        if (result < 0) {
            printf("[REPLY] ❌ 回包失败: TCP 发送到客户端 fd=%d 出错\n", command->tcp_fd);
        }
        return result;
    }
    int result = tcp_server_send_to_last(buf, len);
    if (result < 0) {
        printf("[REPLY] ❌ 回包失败: TCP 发送到最近客户端出错 (无客户端连接?)\n");
    }
    return result;
}

void protocol_parser_init(protocol_parser_t *parser) {
    if (parser == NULL) {
        return;
    }
    parser->state = PROTOCOL_PARSE_HEADER;
    parser->need = 2;
    parser->len = 0;
    parser->parse_start = 0;   // IDLE 状态, 不计时
    parser->command.is_valid = 0;
    parser->command.opcode = 0;
    parser->command.data = NULL;
    parser->command.data_len = 0;
}

// 收齐一个完整命令包后调用: 执行命令
static void protocol_parser_on_packet(protocol_parser_t *parser) {
    if (g_thread_pool == NULL) {
        printf("[PARSE] ❌ 处理命令包失败: 全局线程池未初始化\n");
        protocol_parser_init(parser);
        return;
    }

    parser->command.is_valid = 1;
    parser->command.data = parser->buf + PROTOCOL_DATA_OFFSET;

    // 打印：解析到完整命令，显示操作码和数据长度
    printf("[PARSE] ✅ 收到完整命令: opcode=0x%02X, 数据长度=%u 字节\n",
           parser->command.opcode, (unsigned)parser->command.data_len);

    if (execute_command(&parser->command) != 0) {
        printf("[PARSE] ❌ 命令执行失败: opcode=0x%02X\n", parser->command.opcode);
    }

    protocol_parser_init(parser);
}

// 整包收齐后校验 checksum: 通过则执行命令, 失败则丢弃本包并重新同步
static void protocol_parser_verify_packet(protocol_parser_t *parser) {
    uint16_t received_checksum = ((uint16_t)parser->buf[PROTOCOL_HEADER_SIZE] << 8) |
                                 (uint16_t)parser->buf[PROTOCOL_HEADER_SIZE + 1];
    uint16_t calculated_checksum = compute_checksum(parser->buf, parser->len);

    if (received_checksum != calculated_checksum) {
        // 打印：校验和不匹配，显示收到的校验码和计算出的校验码
        printf("[PARSE] ❌ 校验和不匹配: 收到=0x%04X, 计算=0x%04X, 本包已丢弃\n",
               received_checksum, calculated_checksum);
        send_error_reply(&parser->command,0xf2,parser->command.opcode);
        protocol_parser_init(parser);
        return;
    }

    protocol_parser_on_packet(parser);
}



// 向状态机喂入字节流; 支持粘包与拆包
void protocol_parser_feed(protocol_parser_t *parser, const uint8_t *data, int len) {
    if (parser == NULL || data == NULL || len <= 0) {
        return;
    }

    for (int i = 0; i < len; i++) {
        const uint8_t byte = data[i];
        switch (parser->state) {
        case PROTOCOL_PARSE_HEADER:
            if (parser->len == 0) {
                if (byte == 0xAB) {
                    parser->buf[parser->len++] = byte;
                }
            } else {
                if (byte == 0xCD) {
                    parser->buf[parser->len++] = byte;
                    parser->state = PROTOCOL_PARSE_LEN;
                    parser->need = 4;
                    parser->parse_start = time(NULL);   // 开始计时
                } else {
                    parser->len = 0;
                    if (byte == 0xAB) {
                        parser->buf[parser->len++] = byte;
                    }
                }
            }
            break;

        case PROTOCOL_PARSE_LEN:
            parser->buf[parser->len++] = byte;
            if (--parser->need == 0) {
                parser->command.data_len = ((uint32_t)parser->buf[2] << 24) |
                                           ((uint32_t)parser->buf[3] << 16) |
                                           ((uint32_t)parser->buf[4] << 8) |
                                           parser->buf[5];
                if (parser->command.data_len >
                    MAX_PACKET_SIZE - PROTOCOL_HEADER_SIZE - PROTOCOL_CHECKSUM_SIZE) {
                    printf("[PARSE] ❌ 包长度非法: data_len=%u 超出上限, 已丢弃并重新同步\n",
                           (unsigned)parser->command.data_len);
                    protocol_parser_init(parser);
                    break;
                }
                parser->state = PROTOCOL_PARSE_OPCODE;
                parser->need = 1;
            }
            break;

        case PROTOCOL_PARSE_OPCODE:
            parser->buf[parser->len++] = byte;
            parser->command.opcode = byte;
            parser->state = PROTOCOL_PARSE_CHECKSUM;
            parser->need = PROTOCOL_CHECKSUM_SIZE;
            break;

        case PROTOCOL_PARSE_CHECKSUM:
            parser->buf[parser->len++] = byte;
            if (--parser->need == 0) {
                parser->state = PROTOCOL_PARSE_DATA;
                parser->need = parser->command.data_len;
                if (parser->need == 0) {
                    protocol_parser_verify_packet(parser);
                }
            }
            break;

        case PROTOCOL_PARSE_DATA:
            parser->buf[parser->len++] = byte;
            if (--parser->need == 0) {
                protocol_parser_verify_packet(parser);
            }
            break;
        }
    }
}

// 检查解析器是否超时: 返回1=超时(调用方应重置parser), 0=正常
int protocol_parser_check_timeout(protocol_parser_t *parser) {
    if (parser == NULL || parser->parse_start == 0) {
        return 0;   // IDLE 状态, 不需要检查
    }
    if (time(NULL) - parser->parse_start >= PARSE_TIMEOUT_SEC) {
        printf("[PARSE] ⚠️ 解析超时: 状态=%d, 已收 %u 字节, 未收齐整包, 解析器已重置\n",
               parser->state, (unsigned)parser->len);
        return 1;
    }
    return 0;
}