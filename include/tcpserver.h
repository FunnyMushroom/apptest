#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdint.h>
#include "threadpool.h"
#include "protocol.h"

#define SERVER_PORT 8080
#define BACKLOG 5
#define BUFFER_SIZE 1024
#define MAX_CLIENTS 32
#define MAX_CONNECTIONS 2   // 同时连接的上位机数量(按不同 IP 计)

// TCP keepalive 断线探测: 连接空闲 KEEPALIVE_IDLE_SEC 后开始发探测包,
// 每 KEEPALIVE_INTERVAL 秒一次, 连续 KEEPALIVE_PROBE_CNT 次无响应判为断开并回收
#define KEEPALIVE_IDLE_SEC    30
#define KEEPALIVE_INTERVAL    10
#define KEEPALIVE_PROBE_CNT   3

// 客户端结构体
typedef struct {
    int fd;
    char ip[INET_ADDRSTRLEN];
    int port;
    protocol_parser_t parser;
} client_t;

// TCP 服务器结构体
typedef struct {
    int server_fd;
    client_t clients[MAX_CLIENTS];
    fd_set all_fds;
    int max_fd;
    int last_sender_fd;             // 最近发消息的客户端 fd
    char last_sender_ip[INET_ADDRSTRLEN];
    int last_sender_port;
} tcp_server_t;

int tcp_server_start(int port);
int tcp_server_broadcast(const uint8_t *data, int len);
int tcp_server_last_sender_fd(void);
int tcp_server_send_to_fd(int fd, const uint8_t *data, int len);
int tcp_server_send_to_last(const uint8_t *data, int len);

#endif