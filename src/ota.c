#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ota.h"
#include "protocol.h"
#include "task.h"
#include "tcpserver.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_SYS
#include "log.h"

// ==================== 内部工具 ====================

// 大端字段解析
static uint32_t ota_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}
static uint16_t ota_be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

// ---------- CRC32 (IEEE 802.3, 表驱动) ----------
static uint32_t s_crc32_table[256];
static int      s_crc32_ready = 0;

static void ota_crc32_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        s_crc32_table[i] = c;
    }
    s_crc32_ready = 1;
}

// 对内存块计算 CRC32(标准 CRC-32/IEEE)
static uint32_t ota_crc32(const uint8_t *buf, long len) {
    if (!s_crc32_ready) ota_crc32_init();
    uint32_t crc = 0xFFFFFFFFu;
    for (long i = 0; i < len; i++) {
        crc = s_crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// 整包校验和(与 task.c/protocol.c 一致: 对包头+长度+操作码+数据求和取低16位, 不含校验码自身)
static uint16_t ota_compute_checksum(const uint8_t *buf, uint32_t total_len) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < total_len; i++) {
        if (i == PROTOCOL_HEADER_SIZE || i == PROTOCOL_HEADER_SIZE + 1) {
            continue;   // 跳过校验码字段自身
        }
        sum += buf[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

// 构造并发送 OTA 应答包: AB CD | len(4,大端)=2 | opcode=0x24 | checksum(2) | [status][0xFF]
// 回包路由复用命令自带的 channel/tcp_fd/serial_port, 按来源(TCP/485)回给发起方
static void ota_reply(command_t *task_arg, uint8_t status) {
    if (task_arg == NULL) {
        printf("[OTA] ⚠️ 回包跳过: task_arg 为 NULL\n");
        return;
    }
    uint8_t resp[16];
    uint32_t idx = 0;

    resp[idx++] = 0xAB;
    resp[idx++] = 0xCD;
    resp[idx++] = 0x00;
    resp[idx++] = 0x00;
    resp[idx++] = 0x00;
    resp[idx++] = 0x02;              // data_len = 2
    resp[idx++] = OTA_REPLY_OPCODE;
    resp[idx++] = 0x00;              // checksum 占位
    resp[idx++] = 0x00;
    resp[idx++] = status;            // 状态
    resp[idx++] = 0xFF;              // 保留

    uint32_t total_len = idx;
    uint16_t cs = ota_compute_checksum(resp, total_len);
    resp[PROTOCOL_HEADER_SIZE]     = (uint8_t)(cs >> 8);
    resp[PROTOCOL_HEADER_SIZE + 1] = (uint8_t)(cs & 0xFF);

    int r = app_send_response(task_arg, resp, (int)total_len);
    if (r > 0) {
        printf("[OTA] ✅ 回包已发送: 状态=0x%02X, 共 %u 字节\n", status, (unsigned)total_len);
    } else {
        printf("[OTA] ❌ 回包发送失败: 状态=0x%02X, 共 %u 字节\n", status, (unsigned)total_len);
    }
}

// ==================== 升级状态文件(回滚支撑) ====================
// 状态: 0=无 1=待确认 2=成功 3=已回滚(见 ota.h OTA_STATE_*)
static int ota_state_read(void) {
    FILE *fp = fopen(OTA_STATE_FILE, "r");
    if (fp == NULL) {
        return OTA_STATE_NONE;
    }
    char buf[32] = {0};
    if (fgets(buf, sizeof(buf), fp) == NULL) {
        fclose(fp);
        return OTA_STATE_NONE;
    }
    fclose(fp);
    int v = atoi(buf);
    return (v >= OTA_STATE_NONE && v <= OTA_STATE_ROLLBACK) ? v : OTA_STATE_NONE;
}

static int ota_state_write(int v) {
    FILE *fp = fopen(OTA_STATE_FILE, "w");
    if (fp == NULL) {
        return -1;
    }
    fprintf(fp, "%d\n", v);
    fflush(fp);
    fsync(fileno(fp));
    fclose(fp);
    return 0;
}

// ==================== 文件接收(TCP) ====================

// 建立专用监听端口, 等待文件客户端连入(带超时, 避免长期占用线程池线程)
// 返回: 已连入的客户端 fd; <0 失败/超时
static int ota_wait_client(void) {
    printf("[OTA] ▶ 等待文件客户端连接, 监听端口 %d (超时 %d ms)\n",
           OTA_FILE_PORT, OTA_ACCEPT_TIMEOUT_MS);

    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0) {
        printf("[OTA] ❌ 监听失败: 创建 socket 出错: %s\n", strerror(errno));
        return -1;
    }

    int opt = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(OTA_FILE_PORT);

    if (bind(lfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("[OTA] ❌ 监听失败: 绑定端口 %d 出错: %s\n", OTA_FILE_PORT, strerror(errno));
        close(lfd);
        return -1;
    }
    if (listen(lfd, 1) < 0) {
        printf("[OTA] ❌ 监听失败: listen 出错: %s\n", strerror(errno));
        close(lfd);
        return -1;
    }

    // 等待连接, 带超时
    struct pollfd pfd;
    pfd.fd     = lfd;
    pfd.events = POLLIN;
    int pr = poll(&pfd, 1, OTA_ACCEPT_TIMEOUT_MS);
    if (pr <= 0) {
        printf("[OTA] ❌ 等待失败: %s\n",
               (pr == 0) ? "等待文件客户端超时" : strerror(errno));
        close(lfd);
        return -1;
    }

    int cfd = accept(lfd, NULL, NULL);
    close(lfd);                     // 一次只服务一个文件客户端
    if (cfd < 0) {
        printf("[OTA] ❌ 等待失败: accept 出错: %s\n", strerror(errno));
        return -1;
    }

    // 接收超时, 防止客户端连上但不发数据把任务卡死
    struct timeval tv;
    tv.tv_sec  = OTA_RECV_TIMEOUT_SEC;
    tv.tv_usec = 0;
    setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    printf("[OTA] ✅ 文件客户端已连接: fd=%d, 接收超时 %d 秒\n", cfd, OTA_RECV_TIMEOUT_SEC);
    return cfd;
}

// 从客户端 fd 读取裸数据并写入暂存文件。
// expect_size > 0: 收满这么多字节立刻结束 —— 命令包里已经带了文件大小, 不必等对端关闭。
// expect_size == 0: 退化为老行为, 读到对端关闭为止。
// 返回: 收到的字节数; <0 出错
static long ota_receive_file(int cfd, const char *path, uint32_t expect_size) {
    printf("[OTA] ▶ 开始接收文件 -> 暂存 %s", path);
    if (expect_size > 0) {
        printf(" (期望 %u 字节, 收满即结束)\n", expect_size);
    } else {
        printf(" (未指定大小, 读到对端关闭为止)\n");
    }

    FILE *fp = fopen(path, "wb");
    if (fp == NULL) {
        printf("[OTA] ❌ 接收失败: 打开暂存文件 %s 出错: %s\n", path, strerror(errno));
        return -1;
    }

    uint8_t buf[64 * 1024];
    long total = 0;
    int  got_all = 0;               // 是否已收满 expect_size
    while (1) {
        ssize_t n = recv(cfd, buf, sizeof(buf), 0);
        if (n > 0) {
            if (fwrite(buf, 1, (size_t)n, fp) != (size_t)n) {
                printf("[OTA] ❌ 接收失败: 写入 %s 出错: %s\n", path, strerror(errno));
                fclose(fp);
                return -1;
            }
            total += n;
            printf("[OTA]   已接收 %ld 字节\n", total);

            /* 命令包里带了文件大小, 收满就结束 —— 不必等对端关闭连接。
             * 否则客户端"发完"到串口刷出"接收完毕"之间会一直沉默;
             * 客户端要是忘了关连接, 还会一路等到 SO_RCVTIMEO 超时失败。 */
            if (expect_size > 0 && total >= (long)expect_size) {
                got_all = 1;
                break;
            }
        } else if (n == 0) {
            break;                  // 对端关闭, 传输结束(未指定大小时靠这个)
        } else {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                printf("[OTA] ❌ 接收失败: 等数据超时(已收 %ld 字节)\n", total);
            } else {
                printf("[OTA] ❌ 接收失败: recv 出错: %s\n", strerror(errno));
            }
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    printf("[OTA] ✅ 文件接收完成: %s, 共 %ld 字节%s\n",
           path, total, got_all ? " (已收满期望大小, 无需等对端关闭)" : "");
    return total;
}

// ==================== 安装 / 回滚(替换单个可执行文件) ====================

// 新程序是否像 ELF 可执行文件(防止把程序刷成垃圾)
static int ota_is_elf(const uint8_t *buf, long len) {
    return (len >= 4 && buf[0] == 0x7F && buf[1] == 'E' &&
            buf[2] == 'L' && buf[3] == 'F');
}

// 安装: 备份当前程序 -> 新程序就位 -> 标记"待确认"
// 返回: OTA_OK 或错误码
static int ota_install(void) {
    printf("[OTA] ▶ 开始安装新程序\n");

    struct stat st;
    if (stat(OTA_STAGING, &st) != 0 || st.st_size <= 0) {
        printf("[OTA] ❌ 安装失败: 暂存文件 %s 不存在或为空\n", OTA_STAGING);
        return OTA_ERR_RECV;
    }
    if (chmod(OTA_STAGING, 0755) != 0) {
        printf("[OTA] ❌ 安装失败: chmod %s 出错: %s\n", OTA_STAGING, strerror(errno));
        return OTA_ERR_WRITE;
    }

    // 1. 备份当前程序(rename 原子移动; 运行中的进程不受影响)
    if (access(OTA_TARGET, F_OK) == 0) {
        if (rename(OTA_TARGET, OTA_BACKUP) != 0) {
            printf("[OTA] ❌ 安装失败: 备份 %s -> %s 出错: %s\n",
                   OTA_TARGET, OTA_BACKUP, strerror(errno));
            return OTA_ERR_BACKUP;
        }
        printf("[OTA]   已备份当前程序: %s -> %s\n", OTA_TARGET, OTA_BACKUP);
    } else {
        printf("[OTA] ⚠️ 当前程序 %s 不存在, 跳过备份\n", OTA_TARGET);
    }

    // 2. 新程序就位(同目录 rename, 原子替换)
    if (rename(OTA_STAGING, OTA_TARGET) != 0) {
        printf("[OTA] ❌ 安装失败: 替换 %s -> %s 出错: %s\n",
               OTA_STAGING, OTA_TARGET, strerror(errno));
        return OTA_ERR_WRITE;
    }
    sync();   // 持久化, 防掉电丢文件
    printf("[OTA]   新程序已就位: %s\n", OTA_TARGET);

    // 3. 标记"待确认": 下次启动未确认成功则自动回滚
    if (ota_state_write(OTA_STATE_PENDING) != 0) {
        printf("[OTA] ❌ 安装失败: 写入待确认状态出错: %s\n", strerror(errno));
        return OTA_ERR_STATE;
    }
    printf("[OTA] ✅ 安装完成: 已标记为待确认, 下次启动未确认将自动回滚\n");
    return OTA_OK;
}

// 立即回滚: 用 .bak 还原目标程序
int ota_do_rollback(void) {
    printf("[OTA] ▶ 开始回滚\n");

    if (access(OTA_BACKUP, F_OK) != 0) {
        printf("[OTA] ❌ 回滚失败: 没有备份文件 %s\n", OTA_BACKUP);
        return OTA_ERR_NOBACKUP;
    }
    if (rename(OTA_BACKUP, OTA_TARGET) != 0) {
        printf("[OTA] ❌ 回滚失败: 恢复 %s 出错: %s\n", OTA_TARGET, strerror(errno));
        return OTA_ERR_WRITE;
    }
    sync();
    ota_state_write(OTA_STATE_ROLLBACK);
    printf("[OTA] ✅ 回滚完成: 已还原为上一个版本 (%s -> %s)\n", OTA_BACKUP, OTA_TARGET);
    return OTA_OK;
}

// 标记升级成功(应用初始化完成后调用)
void ota_mark_ok(void) {
    if (ota_state_read() == OTA_STATE_PENDING) {
        ota_state_write(OTA_STATE_OK);
        printf("[OTA] ✅ 升级已确认成功, 回滚看门狗任务解除\n");
    }
}

// 看门狗线程: 新程序首次启动后在确认期内未 mark_ok, 则自动回滚并重启旧程序
static void *ota_watchdog_thread(void *arg) {
    int sec = *(int *)arg;
    free(arg);
    printf("[OTA]   回滚看门狗开始计时: %d 秒\n", sec);
    sleep(sec);
    if (ota_state_read() == OTA_STATE_PENDING) {
        printf("[OTA] ❌ 看门狗超时: %d 秒内未确认升级, 开始自动回滚\n", sec);
        if (ota_do_rollback() == OTA_OK) {
            printf("[OTA]   正在重启旧版本程序: %s\n", OTA_TARGET);
            execl(OTA_TARGET, OTA_TARGET, (char *)NULL);
            printf("[OTA] ❌ 重启旧版本失败: execl 出错: %s\n", strerror(errno));
        }
    } else {
        printf("[OTA]   看门狗计时结束: 升级已确认, 无需回滚\n");
    }
    return NULL;
}

// 应用启动时调用: 若上次升级处于"待确认", 启动回滚看门狗
void ota_watchdog_start(int timeout_sec) {
    int state = ota_state_read();
    
    if (state != OTA_STATE_PENDING) {
        const char *state_str = "UNKNOWN";
        switch (state) {
            case OTA_STATE_NONE:     state_str = "NONE(无升级)"; break;
            case OTA_STATE_OK:       state_str = "OK(已确认)"; break;
            case OTA_STATE_ROLLBACK: state_str = "ROLLBACK(已回滚)"; break;
            default:                 state_str = "UNKNOWN"; break;
        }
        printf("[OTA]   无需启动看门狗, 当前状态=%s\n", state_str);
        return;
    }

    printf("[OTA] ⚠️ 检测到待确认升级(PENDING), 启动回滚看门狗\n");
    printf("[OTA]   看门狗倒计时: %d 秒, 超时将自动回滚到旧版本\n", timeout_sec);
    printf("[OTA]   提示: 程序初始化完成后请调用 ota_mark_ok() 确认升级成功\n");

    pthread_t tid;
    int *arg = malloc(sizeof(*arg));
    if (arg == NULL) {
        printf("[OTA] ❌ 看门狗启动失败: 分配参数内存出错\n");
        return;
    }
    *arg = (timeout_sec > 0) ? timeout_sec : OTA_WATCHDOG_SEC;

    int ret = pthread_create(&tid, NULL, ota_watchdog_thread, arg);
    if (ret != 0) {
        free(arg);
        printf("[OTA] ❌ 看门狗启动失败: 创建线程出错: %s\n", strerror(ret));
        return;
    }

    pthread_detach(tid);
    printf("[OTA] ✅ 回滚看门狗已启动: %d 秒后超时\n", *arg);
    printf("[OTA]   升级确认倒计时已开始\n");
}

// ==================== 任务入口 ====================

// 0x23 命令触发的在线升级任务:
//   专用端口等文件 -> 暂存 -> 校验(大小/CRC32/ELF) -> 安装(备份+替换+待确认) / 只校验 / 立即回滚
void Task_OTA_Upgrade(void *arg) {
    command_t *task_arg = (command_t *)arg;

    // 命令数据(9字节): [0..3]=新程序字节数(大端,0=不校验) [4..7]=CRC32(大端,0=不校验) [8]=模式
    uint32_t expect_size = 0;
    uint32_t expect_crc  = 0;
    uint8_t  mode        = OTA_MODE_INSTALL;
    if (task_arg->data && task_arg->data_len >= 9) {
        expect_size = ota_be32(task_arg->data);
        expect_crc  = ota_be32(task_arg->data + 4);
        mode        = task_arg->data[8];
    }
    free(task_arg->data);
    task_arg->data = NULL;

    if (mode != OTA_MODE_INSTALL && mode != OTA_MODE_VERIFY && mode != OTA_MODE_ROLLBACK) {
        fprintf(stderr, "[OTA] ❌ 不支持的模式: mode=%u (应为 0=安装 1=只校验 2=回滚)\n", mode);
        ota_reply(task_arg, OTA_ERR_MODE);
        free(task_arg);
        return;
    }

    // 立即回滚(不需要文件)
    if (mode == OTA_MODE_ROLLBACK) {
        printf("[OTA]   收到手动回滚请求, 开始回滚\n");
        int st = ota_do_rollback();
        ota_reply(task_arg, (uint8_t)st);
        free(task_arg);
        return;
    }

    printf("[OTA] ▶ 在线升级任务开始: mode=%u, 期望大小=%u 字节, 期望CRC32=0x%08X\n",
           mode, expect_size, expect_crc);
    printf("[OTA]   等待文件客户端于端口 %d\n", OTA_FILE_PORT);

    // 1. 等待文件客户端连入(带超时)
    int cfd = ota_wait_client();
    if (cfd < 0) {
        fprintf(stderr, "[OTA] ❌ 升级失败: %d ms 内没有文件客户端连接\n", OTA_ACCEPT_TIMEOUT_MS);
        ota_reply(task_arg, OTA_ERR_LISTEN);
        free(task_arg);
        return;
    }

    // 2. 接收新程序到暂存文件(带上期望大小, 收满即结束, 不必等对端关闭)
    long total = ota_receive_file(cfd, OTA_STAGING, expect_size);
    close(cfd);
    if (total < 0) {
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_ERR_RECV);
        free(task_arg);
        return;
    }
    printf("[OTA]   实际收到 %ld 字节\n", total);

    // 3. 大小校验
    if (expect_size != 0 && total != (long)expect_size) {
        fprintf(stderr, "[OTA] ❌ 升级失败: 文件大小不匹配, 收到 %ld 字节, 期望 %u 字节\n",
                total, expect_size);
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_ERR_SIZE);
        free(task_arg);
        return;
    }
    if (total > OTA_MAX_BIN_SIZE) {
        fprintf(stderr, "[OTA] ❌ 升级失败: 文件过大 %ld > %d 字节\n", total, OTA_MAX_BIN_SIZE);
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_ERR_TOOBIG);
        free(task_arg);
        return;
    }

    // 4. 读入内存做 CRC/ELF 校验
    uint8_t *buf = malloc((size_t)total);
    if (buf == NULL) {
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_ERR_RECV);
        free(task_arg);
        return;
    }
    FILE *fp = fopen(OTA_STAGING, "rb");
    if (fp == NULL || fread(buf, 1, (size_t)total, fp) != (size_t)total) {
        if (fp) fclose(fp);
        free(buf);
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_ERR_RECV);
        free(task_arg);
        return;
    }
    fclose(fp);

    if (expect_crc != 0) {
        uint32_t got = ota_crc32(buf, (long)total);
        if (got != expect_crc) {
            fprintf(stderr, "[OTA] ❌ 升级失败: CRC32 不匹配, 实际 0x%08X, 期望 0x%08X\n",
                    got, expect_crc);
            free(buf);
            unlink(OTA_STAGING);
            ota_reply(task_arg, OTA_ERR_CRC);
            free(task_arg);
            return;
        }
        printf("[OTA] ✅ CRC32 校验通过: 0x%08X\n", got);
    }
    if (!ota_is_elf(buf, (long)total)) {
        fprintf(stderr, "[OTA] ❌ 升级失败: 文件不是 ELF 可执行格式, 拒绝安装\n");
        free(buf);
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_ERR_ELF);
        free(task_arg);
        return;
    }
    free(buf);

    if (mode == OTA_MODE_VERIFY) {
        printf("[OTA] ✅ 仅校验模式: 校验通过, 未安装任何文件\n");
        unlink(OTA_STAGING);
        ota_reply(task_arg, OTA_OK);
        free(task_arg);
        return;
    }

    // 5. 安装(备份+替换+待确认)
    int st = ota_install();
    if (st == OTA_OK) {
        printf("[OTA] ✅ 安装完成: %s (%ld 字节)\n", OTA_TARGET, total);
        printf("[OTA]   重启后生效; 若未能确认成功将自动回滚\n");
    } else {
        fprintf(stderr, "[OTA] ❌ 安装失败: 状态码=0x%02X\n", st);
        unlink(OTA_STAGING);
    }
    ota_reply(task_arg, (uint8_t)st);
    free(task_arg);
}
