#include "gpio.h"
#include <stdint.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_GPIO
#include "log.h"

// 通过 /dev/gpiochip0 字符设备 ioctl 操作 GPIO
// (替代旧的 sysfs 方式: /sys/class/gpio 导出 + 读写 value 文件)
#define GPIO_CHIP_DEVICE  "/dev/gpiochip0"

// 向 gpiochip 请求一个 line handle, 返回 line fd (>=0), 失败返回 -1
static int gpio_request_line(int gpio_num, gpio_direction_t dir) {
    int chip_fd = open(GPIO_CHIP_DEVICE, O_RDONLY);
    if (chip_fd < 0) {
        printf("[GPIO] ❌ 申请 gpio%d 失败: 打开 %s 出错: %s\n",
               gpio_num, GPIO_CHIP_DEVICE, strerror(errno));
        return -1;
    }
    printf("[GPIO]   已打开 %s (fd=%d)\n", GPIO_CHIP_DEVICE, chip_fd);

    struct gpiohandle_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffsets[0] = (uint32_t)gpio_num;   // 目标行号
    req.flags = (dir == GPIO_DIR_IN) ? GPIOHANDLE_REQUEST_INPUT
                                     : GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 0;                 // 输出默认低电平
    req.lines = 1;

    int ret = ioctl(chip_fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
    close(chip_fd);                            // 请求成功后 chip fd 可关闭
    if (ret < 0) {
        printf("[GPIO] ❌ 申请 gpio%d 失败: GPIO_GET_LINEHANDLE_IOCTL 出错: %s\n",
               gpio_num, strerror(errno));
        return -1;
    }
    printf("[GPIO] ✅ gpio%d 占用成功: line_fd=%d, 方向=%s\n",
           gpio_num, req.fd, dir == GPIO_DIR_OUT ? "OUTPUT" : "INPUT");
    return req.fd;
}

gpio_t *gpio_init(int gpio_num, gpio_direction_t dir) {
    printf("[GPIO] ▶ 初始化开始: gpio%d, 方向=%s\n",
           gpio_num, dir == GPIO_DIR_OUT ? "OUTPUT" : "INPUT");

    gpio_t *gpio = (gpio_t *)calloc(1, sizeof(gpio_t));
    if (gpio == NULL) {
        printf("[GPIO] ❌ 初始化失败: 分配 gpio_t 内存失败: %s\n", strerror(errno));
        return NULL;
    }

    gpio->gpio_num = gpio_num;
    gpio->dir = dir;
    gpio->line_fd = gpio_request_line(gpio_num, dir);
    gpio->is_open = (gpio->line_fd >= 0);
    if (!gpio->is_open) {
        printf("[GPIO] ❌ 初始化失败: gpio%d 占用失败\n", gpio_num);
        free(gpio);
        return NULL;
    }

    printf("[GPIO] ✅ 初始化完成: gpio%d, 方向=%s, line_fd=%d\n",
           gpio_num, dir == GPIO_DIR_OUT ? "OUTPUT" : "INPUT", gpio->line_fd);
    return gpio;
}

int gpio_set_value(gpio_t *gpio, gpio_value_t value) {
    if (gpio == NULL || !gpio->is_open || gpio->line_fd < 0) {
        printf("[GPIO] ❌ 设置电平失败: 句柄无效(gpio=%p, is_open=%d, line_fd=%d)\n",
               (void*)gpio, gpio ? gpio->is_open : -1, gpio ? gpio->line_fd : -1);
        return -1;
    }

    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));
    data.values[0] = (value == GPIO_HIGH) ? 1 : 0;

    if (ioctl(gpio->line_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0) {
        printf("[GPIO] ❌ 设置电平失败: gpio%d -> %s: %s\n",
               gpio->gpio_num, value == GPIO_HIGH ? "HIGH(1)" : "LOW(0)",
               strerror(errno));
        return -1;
    }

    printf("[GPIO] ✅ gpio%d 输出 %s\n", gpio->gpio_num,
           value == GPIO_HIGH ? "HIGH(1)" : "LOW(0)");
    return 0;
}

int gpio_get_value(gpio_t *gpio, gpio_value_t *value) {
    if (gpio == NULL || !gpio->is_open || gpio->line_fd < 0) {
        printf("[GPIO] ❌ 读取电平失败: 句柄无效(gpio=%p, is_open=%d, line_fd=%d)\n",
               (void*)gpio, gpio ? gpio->is_open : -1, gpio ? gpio->line_fd : -1);
        return -1;
    }
    if (value == NULL) {
        printf("[GPIO] ❌ 读取电平失败: 输出指针为 NULL\n");
        return -1;
    }

    struct gpiohandle_data data;
    memset(&data, 0, sizeof(data));

    if (ioctl(gpio->line_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0) {
        printf("[GPIO] ❌ 读取电平失败: gpio%d: %s\n",
               gpio->gpio_num, strerror(errno));
        return -1;
    }

    *value = data.values[0] ? GPIO_HIGH : GPIO_LOW;
    printf("[GPIO] ✅ gpio%d 读回 %s\n", gpio->gpio_num,
           *value == GPIO_HIGH ? "HIGH(1)" : "LOW(0)");
    return 0;
}

int gpio_set_direction(gpio_t *gpio, gpio_direction_t dir) {
    if (gpio == NULL) {
        printf("[GPIO] ❌ 切换方向失败: gpio 句柄为 NULL\n");
        return -1;
    }

    // chardev 方式: 方向在请求 line handle 时确定, 需关闭后重新请求
    if (gpio->line_fd >= 0) {
        printf("[GPIO]   先释放 gpio%d (line_fd=%d) 以重新申请方向\n",
               gpio->gpio_num, gpio->line_fd);
        close(gpio->line_fd);
        gpio->line_fd = -1;
    }
    gpio->line_fd = gpio_request_line(gpio->gpio_num, dir);
    if (gpio->line_fd < 0) {
        gpio->is_open = false;
        printf("[GPIO] ❌ 切换方向失败: gpio%d 重新申请失败\n", gpio->gpio_num);
        return -1;
    }
    gpio->dir = dir;
    gpio->is_open = true;
    printf("[GPIO] ✅ 方向已切换: gpio%d -> %s, line_fd=%d\n",
           gpio->gpio_num, dir == GPIO_DIR_OUT ? "OUTPUT" : "INPUT", gpio->line_fd);
    return 0;
}

void gpio_close(gpio_t *gpio) {
    if (gpio == NULL) {
        printf("[GPIO] ⚠️ 关闭跳过: gpio 句柄为 NULL\n");
        return;
    }

    if (gpio->line_fd >= 0) {
        close(gpio->line_fd);
        gpio->line_fd = -1;
    }

    gpio->is_open = false;
    printf("[GPIO] ✅ 关闭完成: gpio%d\n", gpio->gpio_num);

    free(gpio);
}
