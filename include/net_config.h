#ifndef NET_CONFIG_H
#define NET_CONFIG_H

/* 网络参数（如需修改 IP/网关，只需改这里） */
#define NET_IFACE   "eth0"          /* 网卡接口名 */
#define NET_IP      "192.168.0.106" /* 本机 IP(没有 /etc/app_net.conf 时用这个); [2026-10-16] 与 rootfs overlay 的 app_net.conf 保持一致, 原 .100 */
#define NET_MASK    "255.255.255.0" /* 子网掩码 */
#define NET_GW      "192.168.0.1"   /* 默认网关 */

/* 配置网卡 IP、掩码并使接口 up（等价于 ifconfig eth0 ... up）。
 * 返回 0 成功，-1 失败。 */
int net_config_init(void);

/* 添加默认网关（等价于 route add default gw ...）。
 * 返回 0 成功，-1 失败。 */
int net_config_add_default_gw(void);

#endif /* NET_CONFIG_H */
