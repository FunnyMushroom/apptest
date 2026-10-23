#ifndef SPI_H
#define SPI_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

// SPI 模式宏定义
#ifndef SPI_MODE0
#define SPI_MODE0 0
#endif
#ifndef SPI_MODE1
#define SPI_MODE1 1
#endif
#ifndef SPI_MODE2
#define SPI_MODE2 2
#endif
#ifndef SPI_MODE3
#define SPI_MODE3 3
#endif

// SPI 通信结构体
typedef struct {
    char device[64];
    int fd;
    int mode;
    int speed_hz;
    int bits_per_word;
    bool is_open;
} spi_t;

// ==================== 片选(CS)说明 ====================
// 本模块**不手动操作 CS**, 片选由 Linux 内核 SPI core 自动"下拉选中 / 上拉释放":
//   - 每次调用 spi_send / spi_transfer / spi_recv 会下发一个 SPI 消息,
//     内核在消息开始前把该设备的 CS 拉为有效(低电平有效, 设备树标了
//     spi-cs-high 则相反), 消息全部字节传完后自动恢复为无效(拉高)。
//   - 消息内部的 cs_change = 0, 表示整条消息期间 CS 保持有效, 不会中途释放,
//     所以一次调用内的多个字节天然处于同一个片选脉冲里。
//   - CS 编号由设备节点决定: /dev/spidevX.N 中的 N 就是要用的片选号
//     (N=0 用 CS0)。打开哪个节点就用哪个 CS, 用户态无需也不能直接改。
// 注意: 若硬件上的片选**没有接到 SPI 控制器的 CS 引脚**而是接在普通 GPIO 上,
//       内核不会去动它, 此时需要自己在传输前后用 gpio 拉低/拉高该引脚。
spi_t *spi_init(const char *device, int mode, int speed_hz);
int spi_send(spi_t *spi, const unsigned char *data, int len);
int spi_transfer(spi_t *spi, const unsigned char *tx_data, int tx_len, unsigned char *rx_data, int rx_len);
int spi_recv(spi_t *spi, unsigned char *buffer, int len);
void spi_close(spi_t *spi);

#endif 