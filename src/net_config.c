/*
 * net_config.c —— 用 C 程序（ioctl）配置网卡 IP/掩码/网关，
 * 不依赖 ifconfig / route 等外部命令。
 *
 * 需要以 root 权限运行（SIOCSIFADDR / SIOCADDRT 均需 root）。
 */

/* 必须放在所有 #include 之前：
 * 较老的 glibc 工具链（如 NUC980 交叉编译环境）中，struct ifreq / struct rtentry
 * 的完整定义受 __USE_MISC 保护，需 _GNU_SOURCE 才会暴露出来，
 * 否则会报 “不允许使用不完整的类型 struct ifreq”。 */
#define _GNU_SOURCE

#include "net_config.h"

#include <arpa/inet.h>
#include <errno.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/route.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_NET
#include "log.h"

/* 运行时配置文件: 存在则覆盖下面编译时的默认值 */
#define NET_CONF_FILE "/etc/app_net.conf"

/* 生效的网络参数(初值是编译时默认值, net_config_load() 可覆盖) */
static char g_ip[32]   = NET_IP;
static char g_mask[32] = NET_MASK;
static char g_gw[32]   = NET_GW;

/* 网卡 MAC。空字符串 = 不改 MAC(沿用内核里硬编码的那个)。
 * ⚠️ 内核驱动里 MAC 是写死的(nuc980_ether0.c: u8 nuc980_mac0[6]),
 *    所以【每一块刷同一固件的板子 MAC 都一样】。多块板子接同一个网络时
 *    会 MAC 冲突(比 IP 冲突更隐蔽, 表现为间歇性断网), 这里配一个唯一的才行。
 *    —— 这一块是从 app1 移植回来的: app920/app923 当初漏掉了整段配置文件读取。 */
static char g_mac[32]  = "";

/* 校验一个字符串是不是合法 IPv4 */
static int is_valid_ipv4(const char *s)
{
    struct in_addr a;
    return (s != NULL) && (inet_pton(AF_INET, s, &a) == 1);
}

/* 解析 "08:00:27:00:01:A6" → 6 字节。成功返回 0 */
static int parse_mac(const char *s, unsigned char *out)
{
    unsigned int b[6];
    if (s == NULL) return -1;
    if (sscanf(s, "%x:%x:%x:%x:%x:%x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return -1;
    }
    for (int i = 0; i < 6; i++) {
        if (b[i] > 0xFF) return -1;
        out[i] = (unsigned char)b[i];
    }
    return 0;
}

/* 去掉字符串首尾的空白字符(原地修改) */
static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\n' || e[-1] == '\r')) {
        *--e = '\0';
    }
    return s;
}

/* 读配置文件; 文件不存在或某项非法 → 该项保留编译时的默认值 */
static void net_config_load(void)
{
    FILE *fp = fopen(NET_CONF_FILE, "r");
    if (fp == NULL)
    {
        printf("[NET]   无配置文件 %s, 使用编译时默认值\n", NET_CONF_FILE);
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), fp) != NULL)
    {
        char *p = trim(line);
        if (*p == '#' || *p == '\0')
        {
            continue;                    /* 注释 / 空行 */
        }

        char *eq = strchr(p, '=');
        if (eq == NULL)
        {
            continue;                    /* 不是 key=value, 跳过 */
        }
        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);

        if (!strcmp(key, "ip"))
        {
            if (is_valid_ipv4(val)) snprintf(g_ip, sizeof(g_ip), "%s", val);
            else printf("[NET] ⚠️ 配置文件里的 ip 非法, 忽略: %s\n", val);
        }
        else if (!strcmp(key, "mask"))
        {
            if (is_valid_ipv4(val)) snprintf(g_mask, sizeof(g_mask), "%s", val);
            else printf("[NET] ⚠️ 配置文件里的 mask 非法, 忽略: %s\n", val);
        }
        else if (!strcmp(key, "gw"))
        {
            if (is_valid_ipv4(val)) snprintf(g_gw, sizeof(g_gw), "%s", val);
            else printf("[NET] ⚠️ 配置文件里的 gw 非法, 忽略: %s\n", val);
        }
        else if (!strcmp(key, "mac"))
        {
            unsigned char tmp[6];
            if (parse_mac(val, tmp) == 0) snprintf(g_mac, sizeof(g_mac), "%s", val);
            else printf("[NET] ⚠️ 配置文件里的 mac 非法, 忽略: %s\n", val);
        }
    }
    fclose(fp);

    printf("[NET] ✅ 已加载 %s: ip=%s mask=%s gw=%s%s%s\n",
           NET_CONF_FILE, g_ip, g_mask, g_gw,
           (g_mac[0] != '\0') ? " mac=" : "", g_mac);
}

/* 通过 ioctl 设置接口 IP 地址与子网掩码（SIOCSIFADDR / SIOCSIFNETMASK） */
static int net_set_addr(int fd, const char *ifname, const char *ip, const char *mask)
{
    struct ifreq ifr;
    struct sockaddr_in *sin;

    /* 1) 设置 IP 地址 */
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    sin = (struct sockaddr_in *)&ifr.ifr_addr;
    sin->sin_family = AF_INET;
    if (inet_pton(AF_INET, ip, &sin->sin_addr) != 1)
    {
        printf("[NET] ❌ 设置地址失败: 非法 IP %s\n", ip);
        return -1;
    }
    printf("[NET]   正在设置 %s 的 IP = %s ...\n", ifname, ip);
    if (ioctl(fd, SIOCSIFADDR, &ifr) < 0)
    {
        printf("[NET] ❌ 设置地址失败: SIOCSIFADDR(%s): %s\n", ip, strerror(errno));
        return -1;
    }

    /* 2) 设置子网掩码 */
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    sin = (struct sockaddr_in *)&ifr.ifr_netmask;
    sin->sin_family = AF_INET;
    if (inet_pton(AF_INET, mask, &sin->sin_addr) != 1)
    {
        printf("[NET] ❌ 设置掩码失败: 非法掩码 %s\n", mask);
        return -1;
    }
    printf("[NET]   正在设置 %s 的掩码 = %s ...\n", ifname, mask);
    if (ioctl(fd, SIOCSIFNETMASK, &ifr) < 0)
    {
        printf("[NET] ❌ 设置掩码失败: SIOCSIFNETMASK(%s): %s\n", mask, strerror(errno));
        return -1;
    }
    printf("[NET] ✅ 地址与掩码设置完成: %s = %s/%s\n", ifname, ip, mask);

    return 0;
}

/* 使接口 up（保留原有标志，追加 IFF_UP | IFF_RUNNING） */
static int net_set_up(int fd, const char *ifname)
{
    struct ifreq ifr;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0)
    {
        printf("[NET] ❌ 使能接口失败: SIOCGIFFLAGS(%s): %s\n", ifname, strerror(errno));
        return -1;
    }

    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;

    if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0)
    {
        printf("[NET] ❌ 使能接口失败: SIOCSIFFLAGS(%s): %s\n", ifname, strerror(errno));
        return -1;
    }

    printf("[NET] ✅ 接口已使能: %s (UP|RUNNING)\n", ifname);
    return 0;
}

/* 关闭接口。设 MAC 前需要先降下来 —— 内核在接口 UP 时会拒绝改 MAC(EBUSY)。 */
static int net_set_down(int fd, const char *ifname)
{
    struct ifreq ifr;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0)
    {
        return -1;      /* 拿不到标志位就交给下面 net_set_mac 去报错 */
    }
    if ((ifr.ifr_flags & IFF_UP) == 0)
    {
        return 0;       /* 本来就是 DOWN 的, 不用动 */
    }

    ifr.ifr_flags &= ~IFF_UP;
    if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0)
    {
        printf("[NET] ❌ 关闭接口失败: SIOCSIFFLAGS(%s): %s\n", ifname, strerror(errno));
        return -1;
    }
    return 0;
}

/* 设置网卡 MAC。只有配置文件里写了 mac= 才会调用。
 * ⚠️ 必须在接口 UP 之前设, 否则内核会以 EBUSY 拒绝。 */
static int net_set_mac(int fd, const char *ifname, const char *mac)
{
    struct ifreq ifr;
    unsigned char hw[6];

    if (parse_mac(mac, hw) != 0)
    {
        printf("[NET] ❌ 设置 MAC 失败: 非法 MAC %s\n", mac);
        return -1;
    }

    /* 接口若是 UP 的, 先降下再改, 改完由后面的 net_set_up 重新拉起 */
    net_set_down(fd, ifname);

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_hwaddr.sa_family = ARPHRD_ETHER;
    memcpy(ifr.ifr_hwaddr.sa_data, hw, 6);

    printf("[NET]   正在设置 %s 的 MAC = %s ...\n", ifname, mac);
    if (ioctl(fd, SIOCSIFHWADDR, &ifr) < 0)
    {
        printf("[NET] ❌ 设置 MAC 失败: SIOCSIFHWADDR(%s): %s\n", mac, strerror(errno));
        return -1;
    }
    printf("[NET] ✅ MAC 设置完成: %s = %s\n", ifname, mac);
    return 0;
}

int net_config_init(void)
{
    /* 先看有没有运行时配置文件, 有就用它覆盖编译时的默认值 */
    net_config_load();

    printf("[NET] ▶ 网卡配置开始: %s = %s/%s\n", NET_IFACE, g_ip, g_mask);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        printf("[NET] ❌ 网卡配置失败: 创建 socket 失败: %s\n", strerror(errno));
        return -1;
    }

    /* 配置里指定了 mac= 才改 MAC; 没写就沿用内核硬编码的那个。
     * 失败不致命 —— 但会打印 ❌, 看到就说明这块板子的 MAC 还是默认值,
     * 和别的板子撞上了会间歇性断网。 */
    if (g_mac[0] != '\0')
    {
        net_set_mac(fd, NET_IFACE, g_mac);
    }

    if (net_set_addr(fd, NET_IFACE, g_ip, g_mask) < 0)
    {
        printf("[NET] ❌ 网卡配置失败: IP/掩码配置未成功\n");
        close(fd);
        return -1;
    }

    if (net_set_up(fd, NET_IFACE) < 0)
    {
        printf("[NET] ❌ 网卡配置失败: 接口 %s 使能未成功\n", NET_IFACE);
        close(fd);
        return -1;
    }

    close(fd);
    printf("[NET] ✅ 网卡配置完成: %s = %s/%s\n", NET_IFACE, g_ip, g_mask);
    return 0;
}

int net_config_add_default_gw(void)
{
    printf("[NET] ▶ 添加默认网关开始: %s\n", g_gw);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        printf("[NET] ❌ 添加默认网关失败: 创建 socket 失败: %s\n", strerror(errno));
        return -1;
    }

    /* 先尝试删除已有的默认路由，避免重复添加时报错（不存在则忽略） */
    {
        struct rtentry old;
        struct sockaddr_in *sin;

        memset(&old, 0, sizeof(old));
        sin = (struct sockaddr_in *)&old.rt_dst;
        sin->sin_family = AF_INET;
        sin->sin_addr.s_addr = INADDR_ANY;

        sin = (struct sockaddr_in *)&old.rt_genmask;
        sin->sin_family = AF_INET;
        sin->sin_addr.s_addr = INADDR_ANY;

        if (ioctl(fd, SIOCDELRT, &old) == 0) {
            printf("[NET]   已清除旧的默认路由\n");
        } else {
            printf("[NET]   无旧默认路由可清除(可忽略): %s\n", strerror(errno));
        }
    }

    /* 添加默认路由：0.0.0.0/0 -> 网关 */
    {
        struct rtentry rt;
        struct sockaddr_in *sin;

        memset(&rt, 0, sizeof(rt));

        sin = (struct sockaddr_in *)&rt.rt_dst;
        sin->sin_family = AF_INET;
        sin->sin_addr.s_addr = INADDR_ANY;

        sin = (struct sockaddr_in *)&rt.rt_genmask;
        sin->sin_family = AF_INET;
        sin->sin_addr.s_addr = INADDR_ANY;

        sin = (struct sockaddr_in *)&rt.rt_gateway;
        sin->sin_family = AF_INET;
        if (inet_pton(AF_INET, g_gw, &sin->sin_addr) != 1)
        {
            printf("[NET] ❌ 添加默认网关失败: 非法网关 %s\n", g_gw);
            close(fd);
            return -1;
        }

        rt.rt_flags = RTF_UP | RTF_GATEWAY;
        rt.rt_dev = (char *)NET_IFACE;

        if (ioctl(fd, SIOCADDRT, &rt) < 0)
        {
            printf("[NET] ❌ 添加默认网关失败: SIOCADDRT(%s via %s): %s\n",
                   NET_IFACE, g_gw, strerror(errno));
            close(fd);
            return -1;
        }
    }

    close(fd);
    printf("[NET] ✅ 默认网关添加完成: %s via %s\n", NET_IFACE, g_gw);
    return 0;
}
