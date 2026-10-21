#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <time.h>

// 线上帧格式: AB CD | data_len(4,大端) | opcode(1) | checksum(2,大端) | data
// checksum = 整包(不含校验码自身) 64位累加 & 0xFFFF
#define PROTOCOL_HEADER_SIZE   7
#define PROTOCOL_CHECKSUM_SIZE 2
#define PROTOCOL_DATA_OFFSET   (PROTOCOL_HEADER_SIZE + PROTOCOL_CHECKSUM_SIZE)
#define MAX_PACKET_SIZE       2048
#define PARSE_TIMEOUT_SEC      1   // 解析超时: 5秒内未收完一帧则重置

// 命令来源通道 (0 = TCP 客户端, 1 = RS-485 总线)
#define RESPONSE_CHANNEL_TCP   0
#define RESPONSE_CHANNEL_RS485 1

// 命令结构: 命令内容 + 回包路由
typedef struct {
    int      is_valid;      // 是否有效
    uint8_t  opcode;        // 操作码
    uint8_t *data;          // 数据段
    uint32_t data_len;      // 数据长度
    int      channel;       // 命令来源通道 (RESPONSE_CHANNEL_TCP / RESPONSE_CHANNEL_RS485)
    int    tcp_fd;               // TCP: 目标客户端 fd
    void  *serial_port;          // 485: 串口句柄
} command_t;

// 统一回包入口: 按命令来源路由到 TCP 或 485
int app_send_response(const command_t *command, const uint8_t *buf, int len);

// 命令单飞(去重): 释放操作码占用标记(任务真正执行完毕后由任务内部调用)
int opcode_release(uint8_t opcode);

// 协议状态机状态
// 流式协议解析器(每个通道一份, 支持粘包/拆包)
typedef enum {
    PROTOCOL_PARSE_HEADER,     // 等待包头 0xAB 0xCD
    PROTOCOL_PARSE_LEN,        // 等待数据长度(4字节)
    PROTOCOL_PARSE_OPCODE,     // 等待操作码(1字节)
    PROTOCOL_PARSE_CHECKSUM,   // 等待校验码(2字节)
    PROTOCOL_PARSE_DATA,       // 等待数据段
} protocol_parse_state_t;

typedef struct {
    protocol_parse_state_t state;
    uint32_t need;
    uint32_t len;
    time_t   parse_start;   // 进入非IDLE状态时的时间戳, 0=IDLE
    uint8_t  buf[MAX_PACKET_SIZE];
    command_t command;      
} protocol_parser_t;

void protocol_parser_init(protocol_parser_t *parser);
void protocol_parser_feed(protocol_parser_t *parser, const uint8_t *data, int len);
int  protocol_parser_check_timeout(protocol_parser_t *parser);  // 返回1=超时需重置
void send_error_reply(const command_t *command, uint8_t reply_opcode, uint8_t err_opcode);
#endif // PROTOCOL_H
