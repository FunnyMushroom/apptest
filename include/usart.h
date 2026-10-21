#ifndef USART_H
#define USART_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <stdbool.h>

#ifndef CRTSCTS
#define CRTSCTS 0
#endif

// 串口结构体
typedef struct {
    int fd;
    char device[64];
    int baudrate;
    bool is_open;
} usart_t;

usart_t *usart_init(const char *device, int baudrate);
int usart_send(usart_t *usart, const unsigned char *data, int len);
int usart_send_string(usart_t *usart, const char *str);
int usart_recv(usart_t *usart, unsigned char *buffer, int buffer_size);
void usart_close(usart_t *usart);

#endif