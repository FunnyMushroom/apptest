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

// 解析超时(自上次收到字节起, 多久还没收满一帧就判定残帧并重置状态机)
// [2026-11-02] 改为毫秒级 + 每通道可配:
//   原先用 time(NULL) 只有 1 秒粒度, 且 485 通道根本没有调用超时检查,
//   丢一个字节后残帧会把后续命令的字节当成本帧数据咽掉 -> 整条链路静默失效。
#define PARSE_TIMEOUT_MS       1000   // TCP: 1s 未收完一帧则重置
// [2026-11-02 二次修复] 485 从 300ms 收紧到 80ms:
//   485 上一帧最长 15 字节(≈1.3ms @115200), 模块之间的最小间隔实测 11ms,
//   80ms 已是帧长的 60 倍, 绝不会误杀正常帧; 而残帧停留时间越短, 吞掉后续命令的窗口越小。
#define PARSE_TIMEOUT_MS_485     80
#define PARSE_TIMEOUT_MS_MAX  60000   // 非法配置的上限保护

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
    // 请求数据指纹: 分发任务时对 data 前 8 字节做的快照。
    // 用途: 重发请求(上位机超时重试)时, 只有指纹与原请求一致才允许回放上次应答。
    // 注意: 任务函数常在回包前就 free(data), 所以指纹必须在分发时取, 不能在回包时取。
    uint8_t  req_fp[8];
    uint8_t  req_fp_len;
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
    protocol_parse_state_t state;   // 仅用于日志显示(缓冲非空=正在收帧)
    uint32_t need;                  // 仅用于日志: 还差多少字节
    uint32_t len;                   // 缓冲里已收、尚未处理的字节数
    uint64_t parse_start_ms;        // 最近一次收到字节的单调时钟(ms), 0=IDLE
    uint32_t timeout_ms;            // 本通道的残帧超时(0=用默认值)
    uint8_t  buf[MAX_PACKET_SIZE];
    command_t command;
} protocol_parser_t;

void protocol_parser_init(protocol_parser_t *parser);
void protocol_parser_feed(protocol_parser_t *parser, const uint8_t *data, int len);
// 返回1=已超时并完成重新同步(只丢到下一个 AB CD 帧头处再继续解析, 不整包清空),
// 这样跟在残帧后面的好命令还能被救回来。返回0=正常。
int  protocol_parser_check_timeout(protocol_parser_t *parser);
void protocol_parser_set_timeout(protocol_parser_t *parser, uint32_t ms); // 设置本通道残帧超时
void send_error_reply(const command_t *command, uint8_t reply_opcode, uint8_t err_opcode);
#endif // PROTOCOL_H
