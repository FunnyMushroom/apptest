#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

// I2C 通信结构体
typedef struct {
    char device[64];
    int fd;
    int addr;
    bool is_open;
} i2c_t;

i2c_t *i2c_init(const char *device, int addr);
int i2c_write_byte(i2c_t *i2c, unsigned char reg, unsigned char data);
// 同上, 但由调用方指定写后等待时间(us)。
// [2026-10-22] 扫描流程里切通道要连写 2 个 FPGA 寄存器, 默认的 1ms/次会白等 2ms/通道;
//   I2C_RDWR 返回时字节已上线, 等待只为让从机锁存, 200us 足够。
//   高频路径用 delay_us=200, 其它场合继续用 i2c_write_byte(默认 1000us)。
int i2c_write_byte_ex(i2c_t *i2c, unsigned char reg, unsigned char data, unsigned int delay_us);
int i2c_read_byte(i2c_t *i2c, unsigned char reg, unsigned char *data);
int i2c_write_bytes(i2c_t *i2c, unsigned char reg, const unsigned char *data, int len);
int i2c_read_bytes(i2c_t *i2c, unsigned char reg, unsigned char *data, int len);
int i2c_write_raw(i2c_t *i2c, const unsigned char *data, int len);
int i2c_read_raw(i2c_t *i2c, unsigned char *data, int len);
int i2c_mod_bit(i2c_t *i2c, unsigned char reg, uint8_t bit, uint8_t value);
void i2c_close(i2c_t *i2c);

#endif // I2C_H
