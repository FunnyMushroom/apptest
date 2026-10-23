#include "i2c.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_I2C
#include "log.h"

i2c_t *i2c_init(const char *device, int addr) {
    if (device == NULL) {
        printf("[I2C] ❌ 初始化失败: 设备路径为 NULL\n");
        return NULL;
    }

    printf("[I2C] ▶ 初始化开始: 设备=%s, 从机地址=0x%02X\n", device, addr);

    i2c_t *i2c = (i2c_t *)calloc(1, sizeof(i2c_t));
    if (i2c == NULL) {
        printf("[I2C] ❌ 初始化失败: 分配 i2c_t 内存失败: %s\n", strerror(errno));
        return NULL;
    }

    strncpy(i2c->device, device, sizeof(i2c->device) - 1);
    i2c->device[sizeof(i2c->device) - 1] = '\0';
    i2c->addr = addr;
    i2c->is_open = false;

    // 检查设备文件是否存在
    if (access(device, F_OK) != 0) {
        printf("[I2C] ❌ 初始化失败: 设备节点不存在 %s\n", device);
        free(i2c);
        return NULL;
    }

    // 打开 I2C 设备
    printf("[I2C]   设备节点存在, 开始打开 %s ...\n", device);
    i2c->fd = open(device, O_RDWR);
    if (i2c->fd < 0) {
        printf("[I2C] ❌ 初始化失败: 打开 %s 失败: %s\n", device, strerror(errno));
        free(i2c);
        return NULL;
    }
    printf("[I2C]   设备已打开, fd=%d\n", i2c->fd);

    // 设置从机地址
    if (ioctl(i2c->fd, I2C_SLAVE, addr) < 0) {
        printf("[I2C] ❌ 初始化失败: 设置从机地址 0x%02X 失败(%s): %s\n",
               addr, device, strerror(errno));
        close(i2c->fd);
        free(i2c);
        return NULL;
    }

    i2c->is_open = true;
    printf("[I2C] ✅ 初始化完成: %s, 从机地址=0x%02X, fd=%d\n",
           device, addr, i2c->fd);
    return i2c;
}

// 写一个字节到指定寄存器
int i2c_write_byte(i2c_t *i2c, unsigned char reg, unsigned char data) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 写字节失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }

    unsigned char buf[2] = { reg, data };
    struct i2c_msg msg = {
        .addr = (uint16_t)i2c->addr,
        .flags = 0,
        .len = 2,
        .buf = buf,
    };
    struct i2c_rdwr_ioctl_data rdwr = { .msgs = &msg, .nmsgs = 1 };

    if (ioctl(i2c->fd, I2C_RDWR, &rdwr) < 0) {
        printf("[I2C] ❌ 写字节失败: reg=0x%02X data=0x%02X addr=0x%02X: %s\n",
               reg, data, i2c->addr, strerror(errno));
        return -1;
    }

    // 等待从机锁存寄存器
    usleep(1000);
    return 0;
}

// 从指定寄存器读一个字节(combined 传输: 写寄存器地址 + 读数据, 中间无 STOP)
int i2c_read_byte(i2c_t *i2c, unsigned char reg, unsigned char *data) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 读字节失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL) {
        printf("[I2C] ❌ 读字节失败: 输出指针为 NULL\n");
        return -1;
    }

    struct i2c_msg msgs[2] = {
        { .addr = (uint16_t)i2c->addr, .flags = 0,        .len = 1, .buf = &reg },
        { .addr = (uint16_t)i2c->addr, .flags = I2C_M_RD, .len = 1, .buf = data },
    };
    struct i2c_rdwr_ioctl_data rdwr = { .msgs = msgs, .nmsgs = 2 };

    if (ioctl(i2c->fd, I2C_RDWR, &rdwr) < 0) {
        printf("[I2C] ❌ 读字节失败: reg=0x%02X addr=0x%02X: %s\n",
               reg, i2c->addr, strerror(errno));
        return -1;
    }

    return 0;
}

// 写多个字节到指定寄存器(连续写入)
int i2c_write_bytes(i2c_t *i2c, unsigned char reg, const unsigned char *data, int len) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 写多字节失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL || len <= 0) {
        printf("[I2C] ❌ 写多字节失败: 参数无效(data=%p, len=%d)\n",
               (const void *)data, len);
        return -1;
    }

    // 动态构建缓冲区: 寄存器地址 + 数据
    unsigned char *buf = (unsigned char *)malloc(1 + len);
    if (buf == NULL) {
        printf("[I2C] ❌ 写多字节失败: 分配缓冲区失败: %s\n", strerror(errno));
        return -1;
    }

    buf[0] = reg;
    memcpy(buf + 1, data, len);

    struct i2c_msg msg = {
        .addr = (uint16_t)i2c->addr,
        .flags = 0,
        .len = 1 + len,
        .buf = buf,
    };
    struct i2c_rdwr_ioctl_data rdwr = { .msgs = &msg, .nmsgs = 1 };

    int ret = ioctl(i2c->fd, I2C_RDWR, &rdwr);
    free(buf);

    if (ret < 0) {
        printf("[I2C] ❌ 写多字节失败: reg=0x%02X len=%d addr=0x%02X: %s\n",
               reg, len, i2c->addr, strerror(errno));
        return -1;
    }

    printf("[I2C] ✅ 写多字节完成: reg=0x%02X len=%d\n", reg, len);

    // 等待从机锁存寄存器
    usleep(1000);
    return 0;
}

// 从指定寄存器读多个字节(连续读取, combined 传输)
int i2c_read_bytes(i2c_t *i2c, unsigned char reg, unsigned char *data, int len) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 读多字节失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL || len <= 0) {
        printf("[I2C] ❌ 读多字节失败: 参数无效(data=%p, len=%d)\n",
               (void *)data, len);
        return -1;
    }

    struct i2c_msg msgs[2] = {
        { .addr = (uint16_t)i2c->addr, .flags = 0,        .len = 1, .buf = &reg },
        { .addr = (uint16_t)i2c->addr, .flags = I2C_M_RD, .len = (uint16_t)len, .buf = data },
    };
    struct i2c_rdwr_ioctl_data rdwr = { .msgs = msgs, .nmsgs = 2 };

    if (ioctl(i2c->fd, I2C_RDWR, &rdwr) < 0) {
        printf("[I2C] ❌ 读多字节失败: reg=0x%02X len=%d addr=0x%02X: %s\n",
               reg, len, i2c->addr, strerror(errno));
        return -1;
    }

    printf("[I2C] ✅ 读多字节完成: reg=0x%02X len=%d\n", reg, len);
    return 0;
}

// 直接写原始数据(无寄存器地址)
int i2c_write_raw(i2c_t *i2c, const unsigned char *data, int len) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 写原始数据失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL || len <= 0) {
        printf("[I2C] ❌ 写原始数据失败: 参数无效(data=%p, len=%d)\n",
               (const void *)data, len);
        return -1;
    }

    if (write(i2c->fd, data, len) != len) {
        printf("[I2C] ❌ 写原始数据失败: len=%d: %s\n", len, strerror(errno));
        return -1;
    }

    printf("[I2C] ✅ 写原始数据完成: len=%d\n", len);
    return 0;
}

// 直接读原始数据(无寄存器地址)
int i2c_read_raw(i2c_t *i2c, unsigned char *data, int len) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 读原始数据失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (data == NULL || len <= 0) {
        printf("[I2C] ❌ 读原始数据失败: 参数无效(data=%p, len=%d)\n",
               (void *)data, len);
        return -1;
    }

    if (read(i2c->fd, data, len) != len) {
        printf("[I2C] ❌ 读原始数据失败: len=%d: %s\n", len, strerror(errno));
        return -1;
    }

    printf("[I2C] ✅ 读原始数据完成: len=%d\n", len);
    return 0;
}

// 修改寄存器某一位(读-改-写)
int i2c_mod_bit(i2c_t *i2c, unsigned char reg, uint8_t bit, uint8_t value) {
    if (i2c == NULL || !i2c->is_open || i2c->fd < 0) {
        printf("[I2C] ❌ 位修改失败: 句柄无效(未初始化或已关闭)\n");
        return -1;
    }
    if (bit > 7 || value > 1) {
        printf("[I2C] ❌ 位修改失败: 参数越界(bit=%u, value=%u)\n",
               (unsigned)bit, (unsigned)value);
        return -1;
    }

    uint8_t cur;
    if (i2c_read_byte(i2c, reg, &cur) != 0) {
        printf("[I2C] ❌ 位修改失败: 读回寄存器 0x%02X 失败\n", reg);
        return -1;
    }

    uint8_t old = cur;
    if (value)
        cur |=  (1 << bit);
    else
        cur &= ~(1 << bit);

    if (i2c_write_byte(i2c, reg, cur) != 0) {
        printf("[I2C] ❌ 位修改失败: 回写寄存器 0x%02X 失败\n", reg);
        return -1;
    }

    printf("[I2C] ✅ 位修改完成: reg=0x%02X bit%u=%u, 0x%02X -> 0x%02X\n",
           reg, (unsigned)bit, (unsigned)value, old, cur);
    return 0;
}

void i2c_close(i2c_t *i2c) {
    if (i2c == NULL) {
        return;
    }

    if (i2c->is_open && i2c->fd >= 0) {
        close(i2c->fd);
        i2c->fd = -1;
        i2c->is_open = false;
        printf("[I2C] ✅ 关闭完成: %s\n", i2c->device);
    }

    free(i2c);
}
