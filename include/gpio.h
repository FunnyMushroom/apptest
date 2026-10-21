#ifndef GPIO_H
#define GPIO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>

// GPIO 方向
typedef enum {
    GPIO_DIR_IN = 0,
    GPIO_DIR_OUT = 1,
} gpio_direction_t;

// GPIO 电平
typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1,
} gpio_value_t;

// GPIO 控制结构体 (基于 /dev/gpiochip 字符设备 ioctl 操作)
typedef struct {
    int gpio_num;     // GPIO 编号 (gpiochip 行号)
    int line_fd;      // GPIO line handle 文件描述符
    gpio_direction_t dir;
    bool is_open;
} gpio_t;

gpio_t *gpio_init(int gpio_num, gpio_direction_t dir);
int gpio_set_value(gpio_t *gpio, gpio_value_t value);
int gpio_get_value(gpio_t *gpio, gpio_value_t *value);
int gpio_set_direction(gpio_t *gpio, gpio_direction_t dir);
void gpio_close(gpio_t *gpio);

#endif // GPIO_H
