#ifndef UART485_H
#define UART485_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

// 485 通道配置(自动收发电路)
#define UART485_DEVICE   "/dev/ttyS5"
#define UART485_BAUD     115200

int uart485_init(void);
int uart485_send(const uint8_t *data, int len);
void uart485_close(void);

#endif // UART485_H
