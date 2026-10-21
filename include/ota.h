#ifndef OTA_H
#define OTA_H

#include <stdint.h>

// ==================== 在线升级(单可执行文件 + 失败回滚) ====================
// 目标: 在线升级 /usr/bin/program_arm 单个可执行文件
//
// 命令: 0x23, 数据9字节:
//   [0..3]=新程序字节数(大端, 0=不校验) [4..7]=CRC32(大端, 0=不校验) [8]=模式
//   模式: 0=安装 1=只校验 2=立即回滚
// 应答: 0x24, 数据2字节: [status][0xFF]
//
// 安装流程(带回滚):
//   接收 -> 暂存 /usr/bin/program_arm.new -> 校验(大小/CRC32/ELF) -> 备份当前程序到 .bak
//   -> rename 新程序为 program_arm -> 写"待确认"状态文件
// 回滚机制:
//   - 新程序首次启动时 main 调用 ota_watchdog_start(): 超时未 ota_mark_ok() 则自动还原 .bak 并重启
//   - 应用初始化完成后调用 ota_mark_ok(): 标记本次升级成功
//   - 也可发 0x23(mode=2) 立即回滚

#define OTA_CMD_OPCODE    0x55
#define OTA_REPLY_OPCODE  0x56

#define OTA_TARGET        "/usr/bin/program_arm"   // 目标可执行文件
#define OTA_BACKUP        "/usr/bin/program_arm.bak" // 备份(回滚用)
#define OTA_STAGING       "/usr/bin/program_arm.new" // 暂存(与目标同目录, 保证 rename 原子性)
#define OTA_STATE_FILE    "/var/lib/program_arm.ota_state" // 升级状态文件
#define OTA_MAX_BIN_SIZE  (5*1024*1024)   // 程序上限 5MB
/* 新程序首次启动的"确认期"秒数: 超时未 ota_mark_ok() 就自动回滚。
 * 原来 30 秒太紧 —— 实测启动到确认要 19.7 秒, 其中 18.6 秒耗在 ADS1115 的
 * i2c-0 超时重试上(三次 ~6 秒的 Remote I/O error), 余量只剩 ~10 秒。
 * 哪天 I2C 再慢一点就会超过 30 秒 → 误回滚(表现为"刷完重启又变回旧版")。
 * 放到 120 秒: 正常仍然 31 秒确认, 只是真卡住时多等一会儿才回滚。 */
#define OTA_WATCHDOG_SEC  120             // 新程序首次启动的"确认期"秒数

#define OTA_FILE_PORT         9090
#define OTA_ACCEPT_TIMEOUT_MS 60000
#define OTA_RECV_TIMEOUT_SEC  30

// 模式
#define OTA_MODE_INSTALL  0
#define OTA_MODE_VERIFY   1
#define OTA_MODE_ROLLBACK 2

// 状态文件取值
#define OTA_STATE_NONE     0   // 无状态/正常
#define OTA_STATE_PENDING  1   // 新程序待确认
#define OTA_STATE_OK       2   // 确认成功
#define OTA_STATE_ROLLBACK 3   // 已回滚

// 应答状态
#define OTA_OK           0x00
#define OTA_ERR_LISTEN   0x01
#define OTA_ERR_RECV     0x02
#define OTA_ERR_SIZE     0x03
#define OTA_ERR_CRC      0x04
#define OTA_ERR_TOOBIG   0x05
#define OTA_ERR_ELF      0x06
#define OTA_ERR_WRITE    0x07
#define OTA_ERR_BACKUP   0x08
#define OTA_ERR_NOBACKUP 0x09
#define OTA_ERR_STATE    0x0A
#define OTA_ERR_MODE     0x0B

// 协议任务入口(与其它任务同签名, 由线程池调用)
void Task_OTA_Upgrade(void *arg);

// ---- 回滚支撑接口(由应用在 main 中调用) ----
// 新程序启动时调用: 若上次升级处于"待确认", 启动看门狗, 超时未确认则自动回滚并重启旧程序
void ota_watchdog_start(int timeout_sec);
// 应用初始化完成后调用: 标记本次升级成功
void ota_mark_ok(void);
// 立即回滚(还原 .bak); 返回 OTA_OK 或错误码
int  ota_do_rollback(void);

#endif // OTA_H
