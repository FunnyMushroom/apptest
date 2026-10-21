#include "tcpserver.h"

#include <ctype.h>
#include <dirent.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/mount.h>
#include <sys/stat.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_TCP
#include "log.h"

// 全局记录服务器实例，供广播发送使用（tcp_server_start 成功启动后赋值）
static tcp_server_t *g_server = NULL;

static int set_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
    {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// 开启 TCP keepalive 断线探测(内核级, 上位机无需配合)
// 连接空闲 idle 秒后开始发探测包, 每 interval 秒一次, 连续 cnt 次无响应,
// 内核将该 socket 置为错误状态 -> select 报告可读 -> recv 返回错误 -> close_client 回收
static void set_keepalive(int fd)
{
    int keepalive = 1;
    int idle     = KEEPALIVE_IDLE_SEC;
    int interval = KEEPALIVE_INTERVAL;
    int cnt      = KEEPALIVE_PROBE_CNT;

    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
#ifdef TCP_KEEPIDLE
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
}

// 根据 socket inode 查找持有该 socket 的进程 PID（遍历 /proc/*/fd 符号链接）
static int find_pid_by_socket_inode(unsigned long inode)
{
    char inode_str[64];
    snprintf(inode_str, sizeof(inode_str), "socket:[%lu]", inode);

    DIR *proc = opendir("/proc");
    if (proc == NULL)
    {
        return -1;
    }

    int found_pid = -1;
    struct dirent *de;
    while ((de = readdir(proc)) != NULL)
    {
        if (!isdigit((unsigned char)de->d_name[0]))
        {
            continue; /* 只扫描数字命名的目录（PID） */
        }

        char fdpath[320];
        snprintf(fdpath, sizeof(fdpath), "/proc/%s/fd", de->d_name);

        DIR *fdd = opendir(fdpath);
        if (fdd == NULL)
        {
            continue; /* 无权限或无 fd 目录 */
        }

        struct dirent *fde;
        while ((fde = readdir(fdd)) != NULL)
        {
            if (fde->d_name[0] == '.')
            {
                continue;
            }

            char linkpath[560];
            char target[256];
            snprintf(linkpath, sizeof(linkpath), "%s/%s", fdpath, fde->d_name);

            ssize_t len = readlink(linkpath, target, sizeof(target) - 1);
            if (len <= 0)
            {
                continue;
            }
            target[len] = '\0';

            if (strcmp(target, inode_str) == 0)
            {
                found_pid = atoi(de->d_name);
                break;
            }
        }
        closedir(fdd);
        if (found_pid > 0)
        {
            break;
        }
    }
    closedir(proc);
    return found_pid;
}


static int ensure_proc_mounted(void)
{
    struct stat st;
    if (stat("/proc/net/tcp", &st) == 0)
    {
        return 0; /* /proc 已挂载 */
    }

    /* /proc 目录可能不存在，先创建 */
    if (mkdir("/proc", 0755) != 0 && errno != EEXIST)
    {
        printf("[TCP] ❌ 创建 /proc 目录失败: %s\n", strerror(errno));
        return -1;
    }

    if (mount("proc", "/proc", "proc", 0, NULL) == 0)
    {
        printf("[TCP] ✅ /proc 自动挂载成功\n");
        return 0;
    }

    printf("[TCP] ❌ /proc 挂载失败: %s\n", strerror(errno));
    return -1;
}

// 强制结束监听指定端口的所有进程（纯 C 实现：读 /proc/net/tcp 定位 inode → 找 PID → kill）
static void kill_process_on_port(int port)
{
    if (ensure_proc_mounted() < 0)
    {
        printf("[TCP] ⚠️ /proc 不可用, 无法清理端口 %d 上的残留进程\n", port);
        return;
    }

    char port_hex[16];
    snprintf(port_hex, sizeof(port_hex), ":%04X", port & 0xFFFF);

    const char *paths[] = { "/proc/net/tcp", "/proc/net/tcp6" };
    for (int p = 0; p < 2; ++p)
    {
        FILE *fp = fopen(paths[p], "r");
        if (fp == NULL)
        {
            printf("[TCP] ⚠️ 无法读取 %s: %s\n", paths[p], strerror(errno));
            continue;
        }

        char line[512];
        fgets(line, sizeof(line), fp); /* 跳过表头 */
        while (fgets(line, sizeof(line), fp) != NULL)
        {
            char *fields[16] = { NULL };
            int n = 0;
            char *save = NULL;
            char *tok = strtok_r(line, " \t\r\n", &save);
            while (tok != NULL && n < 16)
            {
                fields[n++] = tok;
                tok = strtok_r(NULL, " \t\r\n", &save);
            }
            if (n < 10)
            {
                continue;
            }

            /* fields[1] = 本地地址(ip:port 十六进制)，fields[9] = inode */
            if (strstr(fields[1], port_hex) == NULL)
            {
                continue;
            }

            unsigned long inode = strtoul(fields[9], NULL, 10);
            if (inode == 0)
            {
                continue;
            }

            int pid = find_pid_by_socket_inode(inode);
            if (pid <= 0)
            {
                printf("[TCP] ⚠️ 端口 %d (inode=%lu) 已被占用, 但找不到对应进程\n", port, inode);
                continue;
            }

            printf("[TCP] ▶ 端口 %d 被 PID %d 占用, 正在强制结束\n", port, pid);
            if (kill(pid, SIGKILL) == 0)
            {
                printf("[TCP] ✅ 已结束进程 PID %d\n", pid);
                usleep(100000); /* 稍等内核回收端口，确保下面 bind 成功 */
            }
            else
            {
                printf("[TCP] ❌ 结束进程 PID %d 失败: %s\n", pid, strerror(errno));
            }
        }
        fclose(fp);
    }
}

static void close_client(tcp_server_t *server, int idx)
{
    if (idx < 0 || idx >= MAX_CLIENTS || server == NULL)
    {
        return;
    }

    int fd = server->clients[idx].fd;
    if (fd != -1)
    {
        close(fd);
        FD_CLR(fd, &server->all_fds);
        server->clients[idx].fd = -1;
        protocol_parser_init(&server->clients[idx].parser);
        // 若被关闭的正是"最近发消息"客户端, 清除记录, 避免后续回包误发到已关闭 fd
        if (server->last_sender_fd == fd)
        {
            server->last_sender_fd = -1;
            server->last_sender_ip[0] = '\0';
            server->last_sender_port = 0;
        }
    }
}

static void handle_new_connection(tcp_server_t *server)
{
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int client_fd = accept(server->server_fd, (struct sockaddr *)&client_addr, &addr_len);
    if (client_fd < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            printf("[TCP] ❌ 接受连接失败: %s\n", strerror(errno));
        }
        return;
    }

    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
    
    // 限制同时连接的上位机数量（按不同 IP 计，超出拒绝）。
    // 同一个 IP 允许从不同端口建立多条连接（例如多窗口调试），不会误拒。
    // 1) 若该 IP 已有连接，直接放行
    int same_ip = 0;
    for (int i = 0; i < MAX_CLIENTS; ++i)
    {
        if (server->clients[i].fd != -1 && strcmp(server->clients[i].ip, client_ip) == 0)
        {
            same_ip = 1;
            break;
        }
    }
    if (!same_ip)
    {
        // 2) 统计当前已连接的不同 IP 数量
        int active_ips = 0;
        for (int i = 0; i < MAX_CLIENTS; ++i)
        {
            if (server->clients[i].fd != -1)
            {
                int unique = 1;
                for (int j = 0; j < i; ++j)
                {
                    if (server->clients[j].fd != -1 &&
                        strcmp(server->clients[j].ip, server->clients[i].ip) == 0)
                    {
                        unique = 0;
                        break;
                    }
                }
                if (unique)
                {
                    active_ips++;
                }
            }
        }
        if (active_ips >= MAX_CONNECTIONS)
        {
            printf("[TCP] ⚠️ 连接数已达上限(%d), 拒绝新连接 %s:%d (fd=%d)\n",
                   MAX_CONNECTIONS, client_ip, ntohs(client_addr.sin_port), client_fd);
            close(client_fd);
            return;
        }
    }

    if (set_nonblocking(client_fd) < 0)
    {
        printf("[TCP] ❌ 设置客户端非阻塞失败(fd=%d): %s\n", client_fd, strerror(errno));
        close(client_fd);
        return;
    }

    for (int i = 0; i < MAX_CLIENTS; ++i)
    {
        if (server->clients[i].fd == -1)
        {
            server->clients[i].fd = client_fd;
            protocol_parser_init(&server->clients[i].parser);
            // 标记回包来源：该客户端发来的命令，响应回发给它
            server->clients[i].parser.command.channel = RESPONSE_CHANNEL_TCP;
            server->clients[i].parser.command.tcp_fd = client_fd;
            server->clients[i].parser.command.serial_port = NULL;
            // 保存客户端 IP 和端口，供后续打印/定向回复使用
            inet_ntop(AF_INET, &client_addr.sin_addr,
                      server->clients[i].ip, sizeof(server->clients[i].ip));
            server->clients[i].port = ntohs(client_addr.sin_port);
            // 开启 TCP keepalive 断线探测, 失联连接由内核探测后经 select/recv 错误回收
            set_keepalive(client_fd);
            FD_SET(client_fd, &server->all_fds);
            if (client_fd > server->max_fd)
            {
                server->max_fd = client_fd;
            }
            // 打印：客户端已接入，显示 IP、端口和 fd
            printf("[TCP] ✅ 客户端已接入: %s:%d (fd=%d)\n",
                   server->clients[i].ip, server->clients[i].port, client_fd);
            return;
        }
    }

    close(client_fd);
}

static void handle_client_data(tcp_server_t *server, int idx)
{
    client_t *cli = &server->clients[idx];
    uint8_t read_buf[BUFFER_SIZE];

    while (1)
    {
        ssize_t n = recv(cli->fd, read_buf, sizeof(read_buf), 0);
        if (n > 0)
        {
            // 记录是谁发来的消息，供 tcp_server_last_sender_fd / tcp_server_send_to_last 使用
            server->last_sender_fd = cli->fd;
            snprintf(server->last_sender_ip, sizeof(server->last_sender_ip), "%s", cli->ip);
            server->last_sender_port = cli->port;
            // 交给协议状态机解析：自动处理粘包（多条指令）与拆包（跨多次收包）
            protocol_parser_feed(&cli->parser, read_buf, (int)n);
        }
        else if (n == 0)
        {
            close_client(server, idx);
            return;
        }
        else
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }
            printf("[TCP] ❌ 接收数据失败: %s:%d (fd=%d): %s\n",
                   cli->ip, cli->port, cli->fd, strerror(errno));
            close_client(server, idx);
            return;
        }
    }
}

static void *tcp_server_loop(void *arg)
{
    tcp_server_t *server = (tcp_server_t *)arg;
    while (server != NULL)
    {
        fd_set read_fds = server->all_fds;
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select(server->max_fd + 1, &read_fds, NULL, NULL, &tv);
        if (ret < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            printf("[TCP] ❌ select 出错, 服务器主循环退出: %s\n", strerror(errno));
            break;
        }

        if (ret > 0)
        {
            if (FD_ISSET(server->server_fd, &read_fds))
            {
                handle_new_connection(server);
            }

            for (int i = 0; i < MAX_CLIENTS; ++i)
            {
                if (server->clients[i].fd != -1 && FD_ISSET(server->clients[i].fd, &read_fds))
                {
                    handle_client_data(server, i);
                }
            }
        }

        // 断线回收: TCP keepalive 探测到失联后, 该 socket 进入错误状态,
        // select 将其报告为可读, 上方 handle_client_data 中 recv 返回错误即走 close_client

        // 解析超时检查: 卡住超过 PARSE_TIMEOUT_SEC 秒的解析器强制重置
        for (int i = 0; i < MAX_CLIENTS; ++i) {
            if (server->clients[i].fd != -1 &&
                protocol_parser_check_timeout(&server->clients[i].parser)) {
                printf("[TCP] ⚠️ 解析超时, 已重置解析器: fd=%d (%s:%d)\n",
                       server->clients[i].fd,
                       server->clients[i].ip, server->clients[i].port);
                protocol_parser_init(&server->clients[i].parser);
            }
        }
    }

    return NULL;
}

int tcp_server_start(int port)
{
    tcp_server_t *server = (tcp_server_t *)calloc(1, sizeof(*server));
    if (server == NULL)
    {
        printf("[TCP] ❌ 服务器启动失败: 分配 tcp_server_t 内存出错: %s\n",
               strerror(errno));
        return -1;
    }

    for (int i = 0; i < MAX_CLIENTS; ++i)
    {
        server->clients[i].fd = -1;
    }

    server->server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server->server_fd < 0)
    {
        printf("[TCP] ❌ 启动失败: 创建 socket 出错: %s\n", strerror(errno));
        free(server);
        return -1;
    }

    int opt = 1;
    setsockopt(server->server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 建立监听前，强制结束占用该端口的旧进程，保证本次能正常 bind
    kill_process_on_port(port);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    /* bind：失败且为 EADDRINUSE 时，再次清理占用进程并重试，尽量保证能启动 */
    int bound = 0;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        if (bind(server->server_fd, (struct sockaddr *)&addr, sizeof(addr)) == 0)
        {
            bound = 1;
            break;
        }
        if (errno != EADDRINUSE || attempt == 2)
        {
            printf("[TCP] ❌ 启动失败: bind 端口 %d 出错: %s\n", port, strerror(errno));
            close(server->server_fd);
            free(server);
            return -1;
        }
        // 打印：端口仍被占用，重试清理
        printf("[TCP] ⚠️ 端口 %d 仍被占用, 清理后重试 (第 %d 次)\n", port, attempt + 1);
        kill_process_on_port(port);
        usleep(200000);
    }
    if (!bound)
    {
        printf("[TCP] ❌ 启动失败: 端口 %d 连续清理后仍无法 bind\n", port);
        close(server->server_fd);
        free(server);
        return -1;
    }

    if (listen(server->server_fd, BACKLOG) < 0)
    {
        printf("[TCP] ❌ 启动失败: listen 出错: %s\n", strerror(errno));
        close(server->server_fd);
        free(server);
        return -1;
    }

    if (set_nonblocking(server->server_fd) < 0)
    {
        printf("[TCP] ❌ 启动失败: 设置监听 socket 非阻塞出错: %s\n", strerror(errno));
        close(server->server_fd);
        free(server);
        return -1;
    }

    server->last_sender_fd = -1;
    server->last_sender_ip[0] = '\0';
    server->last_sender_port = 0;
    FD_ZERO(&server->all_fds);
    FD_SET(server->server_fd, &server->all_fds);
    server->max_fd = server->server_fd;

    pthread_t tid;
    int ret = pthread_create(&tid, NULL, tcp_server_loop, server);
    if (ret != 0)
    {
        // 注意: pthread_create 用返回值报错, 不设 errno
        printf("[TCP] ❌ 启动失败: 创建服务器线程出错: %s\n", strerror(ret));
        close(server->server_fd);
        free(server);
        return -1;
    }

    // 记录实例，供 tcp_server_broadcast 使用
    g_server = server;

    // 打印：服务器启动成功，显示端口号
    printf("[TCP] ✅ 服务器已启动: 监听端口 %d, 最大连接数 %d\n", port, MAX_CONNECTIONS);
    return 0;
}

// ==================== 向所有已连接客户端广播 ====================
// 返回成功发送的客户端数量；data/len 为要发送的原始字节
int tcp_server_broadcast(const uint8_t *data, int len)
{
    if (g_server == NULL || data == NULL || len <= 0)
    {
        printf("[TCP] ❌ 广播失败: 参数无效(server=%p, data=%p, len=%d)\n",
               (void *)g_server, (const void *)data, len);
        return -1;
    }

    int sent_count = 0;
    for (int i = 0; i < MAX_CLIENTS; ++i)
    {
        int fd = g_server->clients[i].fd;
        if (fd == -1)
        {
            continue;
        }
        // MSG_NOSIGNAL: 对端断开时避免触发 SIGPIPE 导致进程退出
        ssize_t n = send(fd, data, (size_t)len, MSG_NOSIGNAL);
        if (n > 0)
        {
            sent_count++;
        }
        // n < 0 的情况（EAGAIN 缓冲区满 / EPIPE 对端关闭）交给 select 循环统一回收
    }
    return sent_count;
}

// ==================== 获取最近一次发消息的客户端 ====================
// 返回最近一次发消息给我的客户端 fd；没有则返回 -1
int tcp_server_last_sender_fd(void)
{
    if (g_server == NULL)
    {
        return -1;
    }
    return g_server->last_sender_fd;
}

// ==================== 回复给指定客户端 ====================
// 把 data/len 发送给指定 fd 的客户端；成功返回 1，失败返回 -1
int tcp_server_send_to_fd(int fd, const uint8_t *data, int len)
{
    if (g_server == NULL || data == NULL || len <= 0 || fd <= 0)
    {
        return -1;
    }

    ssize_t n = send(fd, data, (size_t)len, MSG_NOSIGNAL);
    if (n > 0)
    {
        return 1;
    }
    return -1;
}


int tcp_server_send_to_last(const uint8_t *data, int len)
{
    if (g_server == NULL || data == NULL || len <= 0)
    {
        return -1;
    }

    int fd = g_server->last_sender_fd;
    if (fd == -1)
    {
        // 打印：还没有客户端发过消息
        printf("[TCP] ⚠️ 没有客户端发送过消息, 无法回复\n");
        return -1;
    }

    ssize_t n = send(fd, data, (size_t)len, MSG_NOSIGNAL);
    if (n > 0)
    {
        return 1;
    }
    return -1;
}


