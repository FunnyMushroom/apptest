#include "eth.h"
#include <stdio.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_NET
#include "log.h"

// 网卡静态配置参数(如需修改, 改这里即可)
#define ETH_IFACE     "eth0"
#define ETH_IP        "192.168.0.106"
#define ETH_MASK      "255.255.255.0"
#define ETH_GATEWAY   "192.168.0.1"

int network_init(void)
{
    int ret = 0;

    printf("[ETH] ▶ 网络配置开始: %s = %s/%s, 网关=%s\n",
           ETH_IFACE, ETH_IP, ETH_MASK, ETH_GATEWAY);

    // 配置 IP 地址
    {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "ifconfig %s %s netmask %s up",
                 ETH_IFACE, ETH_IP, ETH_MASK);
        printf("[ETH]   步骤 1/2: 设置 IP 与掩码 -> %s\n", cmd);
        if (system(cmd) != 0) {
            printf("[ETH] ❌ 步骤 1/2 失败: IP 配置失败\n");
            ret = -1;
        }
    }

    // 添加默认网关
    {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "route add default gw %s", ETH_GATEWAY);
        printf("[ETH]   步骤 2/2: 添加默认网关 -> %s\n", cmd);
        if (system(cmd) != 0) {
            printf("[ETH] ❌ 步骤 2/2 失败: 网关配置失败\n");
            ret = -1;
        }
    }

    if (ret == 0) {
        printf("[ETH] ✅ 网络配置完成: %s = %s/%s, 网关=%s\n",
               ETH_IFACE, ETH_IP, ETH_MASK, ETH_GATEWAY);
    } else {
        printf("[ETH] ❌ 网络配置失败: 部分步骤未成功, 请检查 %s 是否存在\n", ETH_IFACE);
    }

    return ret;
}