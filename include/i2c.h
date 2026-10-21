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
int i2c_read_byte(i2c_t *i2c, unsigned char reg, unsigned char *data);
int i2c_write_bytes(i2c_t *i2c, unsigned char reg, const unsigned char *data, int len);
int i2c_read_bytes(i2c_t *i2c, unsigned char reg, unsigned char *data, int len);
int i2c_write_raw(i2c_t *i2c, const unsigned char *data, int len);
int i2c_read_raw(i2c_t *i2c, unsigned char *data, int len);
int i2c_mod_bit(i2c_t *i2c, unsigned char reg, uint8_t bit, uint8_t value);
void i2c_close(i2c_t *i2c);

#endif // I2C_H
