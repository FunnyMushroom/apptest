#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "spi.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_ADS8691
#include "log.h"

#define ADS8691_VREF_MV     4096
#define ADS8691_RESOLUTION  (1 << 18)

// SPI 总线参数
#define ADS8691_SPI_DEV   "/dev/spidev2.0"
#define ADS8691_SPI_MODE  SPI_MODE0
// 速率: 原为 50MHz, 实测读数错误(同一时刻 ADS8691_Read_l 读到的才是对的)。
//   4 字节帧 @50MHz 只有 0.64us, 比 ADS8691 的转换时间(tCONV ≈ 665ns)还短,
//   数据在输出寄存器完成更新前就被移出 → 读到的是无效/过期值。
//   降到 10MHz 后帧长 3.2us, 读数与 _l 版一致。
//   以后想提速请逐步上调(20/40MHz)并实测, 不要直接回到 50MHz。
#define ADS8691_SPI_SPEED 10000000

// ==================== ADS8691 命令定义 ====================
#define RST_PWRCRL       0xD0046900   // 复位命令1
#define RST_PWRCRL_N     0xD0040000   // 复位命令2
#define RANGE_BIP_3_0    0xD0140000   // 双极性 ±3*Vref (约±12V)
#define RANGE_BIP_2      0xD0140003   //
// 双极性 ±3*Vref = ±12.288V (RANGE_SEL_REG[3:0]=0000), ADS8691_Init 里直接下发
#define ADS8691_CMD_RANGE_12V  0xD0140000
static spi_t *g_spi = NULL;

// ==================== 发送32位命令 ====================
static void ADS8691_SendCmd(uint32_t cmd) {
    if (g_spi == NULL) {
        printf("[ADS8691] ❌ 发送命令失败: SPI 句柄未初始化 (cmd=0x%08X)\n", cmd);
        return;
    }
    
    uint8_t tx_buf[4] = {
        (cmd >> 24) & 0xFF,
        (cmd >> 16) & 0xFF,
        (cmd >> 8) & 0xFF,
        cmd & 0xFF
    };
    uint8_t rx_buf[4] = {0};
    
    // 修正：tx_len=4, rx_len=4
    int ret = spi_transfer(g_spi, tx_buf, 4, rx_buf, 4);
    if (ret < 0) {
        printf("[ADS8691] ❌ 发送命令失败: cmd=0x%08X\n", cmd);
        return;
    }
    printf("[ADS8691]   命令已下发: 0x%08X -> 回读 %02X %02X %02X %02X\n",
           cmd, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
}

// ==================== 初始化SPI ====================
// 说明: 初始化成功后**直接下发双极性 ±12.288V 量程**(RANGE_SEL=0),
//       所以调用方 ADS8691_Init() 之后无需再调 ADS8691_Set_Range_12V() 就能直接读。
//       ADS8691_Set_Range_12V() 仍然保留, 它会再做一遍软复位 + 写量程 + 回读校验。
int ADS8691_Init(void) {
    if (g_spi != NULL) {
        printf("[ADS8691]   已初始化, 无需重复打开\n");
        return 0;
    }

    printf("[ADS8691] ▶ 初始化开始: %s, MODE0, %d Hz\n",
           ADS8691_SPI_DEV, ADS8691_SPI_SPEED);
    
    g_spi = spi_init(ADS8691_SPI_DEV, ADS8691_SPI_MODE, ADS8691_SPI_SPEED);
    if (g_spi == NULL) {
        printf("[ADS8691] ❌ 初始化失败: SPI 总线 %s 打开失败\n", ADS8691_SPI_DEV);
        return -1;
    }
    
    printf("[ADS8691] ✅ 初始化完成: SPI 已就绪 (%s, MODE0, %d Hz)\n",
           ADS8691_SPI_DEV, ADS8691_SPI_SPEED);

    // ---- 直接写量程: 双极性 ±3*Vref = ±12.288V ----
    printf("[ADS8691] ▶ 写量程: 双极性 ±12.288V (0x%08X)\n",
           (unsigned)ADS8691_CMD_RANGE_12V);
    ADS8691_SendCmd(ADS8691_CMD_RANGE_12V);
    usleep(1000);   // 等量程寄存器写入生效
    printf("[ADS8691] ✅ 量程已设: 双极性 ±12.288V\n");

    return 0;
}

// ==================== 设置正负12V量程 ====================
void ADS8691_Set_Range_12V(void) {
    // if (g_spi == NULL) {
    //     printf("[ADS8691]   尚未初始化, 先执行 ADS8691_Init()\n");
    //     ADS8691_Init();
    // }
    
    // printf("[ADS8691] ▶ 开始设置 ±12V 量程\n");
    
    // // 1. 软件复位
    // printf("[ADS8691]   步骤 1/3: 软复位命令 0x%08X\n", (unsigned)RST_PWRCRL);
    // ADS8691_SendCmd(RST_PWRCRL);
    // usleep(1000);
    // printf("[ADS8691]   步骤 2/3: 软复位保留命令 0x%08X\n", (unsigned)RST_PWRCRL_N);
    // ADS8691_SendCmd(RST_PWRCRL_N);
    // usleep(1000);
    
    // // 2. 设置量程 ±3*Vref (约±12V)
    // printf("[ADS8691]   步骤 3/3: 量程命令 0x%08X\n", (unsigned)RANGE_BIP_3_0);
    //  ADS8691_SendCmd(RANGE_BIP_3_0);
    // // ADS8691_SendCmd(RANGE_BIP_2);
    // usleep(1000);
    
    
    // printf("[ADS8691] ✅ 量程设置完成: ±12V (实际 ±12.288V)\n");
#define RANGE_BIP_12V    0xD0140000           // ±3*Vref = ±12.288V
    
if (g_spi == NULL) {
    if (ADS8691_Init() < 0) {
        printf("[ADS8691] ❌ 初始化失败\n");
        return;
    }
}

printf("[ADS8691] ▶ 开始设置双极性 ±12V 量程\n");

// ---------- 1. 软件复位 ----------
printf("[ADS8691]   步骤 1/4: 软复位 0x%08X\n", (unsigned)RST_PWRCRL);
ADS8691_SendCmd(RST_PWRCRL);
usleep(1000);

printf("[ADS8691]   步骤 2/4: 软复位保留 0x%08X\n", (unsigned)RST_PWRCRL_N);
ADS8691_SendCmd(RST_PWRCRL_N);
usleep(1000);

// ---------- 2. 写量程 ±3*Vref = ±12.288V ----------
printf("[ADS8691]   步骤 3/4: 写量程 0x%08X\n", (unsigned)RANGE_BIP_12V);
ADS8691_SendCmd(RANGE_BIP_12V);
usleep(1000);

// ---------- 3. 发 READ_HWORD 读回 RANGE_SEL_REG ----------
printf("[ADS8691]   步骤 4/4: 读回 RANGE_SEL_REG (0xC8140000)\n");
ADS8691_SendCmd(0xC8140000);
usleep(1000);

// ---------- 4. 再发一帧读回数据 ----------
uint8_t tx_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t rx_buf[4] = {0};
if (spi_transfer(g_spi, tx_buf, 4, rx_buf, 4) < 0) {
    printf("[ADS8691] ❌ 读回失败: SPI 传输出错\n");
    return;
}

uint32_t raw = ((uint32_t)rx_buf[0] << 24) |
               ((uint32_t)rx_buf[1] << 16) |
               ((uint32_t)rx_buf[2] << 8) |
               (uint32_t)rx_buf[3];

uint16_t reg_val = (uint16_t)(raw >> 16);
uint8_t  range   = reg_val & 0x0F;

printf("[ADS8691]   回读: raw=0x%08X, RANGE_SEL_REG=0x%04X, RANGE_SEL[3:0]=0x%X\n",
       raw, reg_val, range);

if (range == 0x00) {
    printf("[ADS8691] ✅ 量程设置成功: 双极性 ±12.288V\n");
    return;
} else {
    printf("[ADS8691] ❌ 量程设置失败: 期望 0x0, 实际 0x%X\n", range);
    return;
}
}


// [2026-10-13] 每点采样次数: 每次 ADS8691_Read() 内部连续采 N 次取平均, 抑制随机噪声。
//   噪声按 1/sqrt(N) 下降, N=8 约降 2.8 倍。
//   [2026-10-22] 改成"一次 ioctl 突发读完", 单点耗时从毫秒级降到微秒级,
//   N 不再需要为耗时让步, 保持 8 不变(测量结果的口径与旧版完全一致)。
//   需要更高精度可上调到 16(仍是一次 ioctl, 只多 8 帧 SPI 时钟)。
#define ADS8691_AVG_N   8

// 每帧 4 字节, 帧间要等转换完成: tCONV ≈ 665ns, 留 2us 余量。
// 交给内核 delay_usecs 做 udelay(微秒精度), 不再用 usleep(2)
// —— usleep 的实际粒度是"至少一个调度节拍"(实测 50~60us), N=8 时白等 0.5ms。
#define ADS8691_CONV_WAIT_US  2
#define ADS8691_BURST_MAX     (ADS8691_AVG_N + 1)   // +1 = 丢弃的首帧

// ==================== 一次 ioctl 连读 n 帧 ====================
// ADS8691 的 SDO 输出的是"上一次转换结果", 每帧传输结束的 CS 上升沿才启动下一次转换。
// 因此这里把 n 帧放进同一条 SPI_IOC_MESSAGE(n):
//   - tr[i].cs_change = 1 (非最后一帧): 该帧传完内核就释放 CS -> 上升沿启动转换;
//   - tr[i].delay_usecs = 2           : 给 tCONV 留时间, 下一帧读到的才是新结果;
// 相比原来"每帧一次 ioctl + usleep(2)", 系统调用从 2n 次降到 1 次。
// 返回: 0 成功(out[0..n-1] 为 18 位码值); -1 失败
static int ads8691_read_burst(uint32_t *out, int n)
{
    uint8_t tx[ADS8691_BURST_MAX][4];
    uint8_t rx[ADS8691_BURST_MAX][4];
    struct spi_ioc_transfer tr[ADS8691_BURST_MAX];

    if (out == NULL || n < 1 || n > ADS8691_BURST_MAX) {
        return -1;
    }

    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));
    memset(tr, 0, sizeof(tr));

    for (int i = 0; i < n; i++) {
        int last = (i == n - 1);
        tr[i].tx_buf        = (unsigned long)tx[i];   // 读数据帧, MOSI 保持 0
        tr[i].rx_buf        = (unsigned long)rx[i];
        tr[i].len           = 4;
        tr[i].speed_hz      = ADS8691_SPI_SPEED;
        tr[i].bits_per_word = 8;
        tr[i].cs_change     = last ? 0 : 1;           // 非最后一帧: 传完释放 CS
        tr[i].delay_usecs   = last ? 0 : ADS8691_CONV_WAIT_US;
    }

    if (spi_transfer_msgs(g_spi, tr, n) < 0) {
        return -1;
    }

    for (int i = 0; i < n; i++) {
        uint32_t raw = ((uint32_t)rx[i][0] << 24) |
                       ((uint32_t)rx[i][1] << 16) |
                       ((uint32_t)rx[i][2] << 8) |
                       (uint32_t)rx[i][3];
        out[i] = (raw >> 14) & 0x3FFFF;
    }
    return 0;
}

// 单次读取(内部使用, 仅作批量路径失败时的回退): 双读同步, 丢弃第 1 帧
static uint32_t ads8691_read_once(void)
{
    uint8_t tx_buf[4] = {0, 0, 0, 0};
    uint8_t rx_buf[4] = {0};

    // ---- 双读同步 ----
    // ADS8691 每一帧输出的是**上一次**转换结果, 不是本次。上一次转换可能启动于切换
    //   通道之前, 直接返回就会把旧通道的值当成当前读数。
    //   铁证: 同一通道连续采样出现精确重复的码值(139450×2、140361×2...), 真实硬件
    //   噪声不可能重复到同一 LSB, 只能是重复读到了同一个旧结果。
    // 因此这里连发两帧: 第 1 帧丢弃, 只取第 2 帧。
    if (spi_transfer(g_spi, tx_buf, 4, rx_buf, 4) < 0) {
        printf("[ADS8691] ❌ 读取失败: 第 1 帧 SPI 传输出错\n");
        return 0xFFFFFFFF;
    }

    // 第 1 帧的 CS 上升沿才启动本次转换: tCONV ≈ 665ns, 这里留 2us 余量。
    // ⚠ 这个等待不可省: 两帧紧挨着的话, 第 2 帧开始时转换还没跑完, 读到的仍是不新鲜的值。
    usleep(2);

    if (spi_transfer(g_spi, tx_buf, 4, rx_buf, 4) < 0) {
        printf("[ADS8691] ❌ 读取失败: 第 2 帧 SPI 传输出错\n");
        return 0xFFFFFFFF;
    }

    uint32_t raw = ((uint32_t)rx_buf[0] << 24) |
                   ((uint32_t)rx_buf[1] << 16) |
                   ((uint32_t)rx_buf[2] << 8) |
                   (uint32_t)rx_buf[3];

    return (raw >> 14) & 0x3FFFF;
}

uint32_t ADS8691_Read(void) {
    if (g_spi == NULL) {
        if (ADS8691_Init() < 0) {
            printf("[ADS8691] ❌ 读取失败: SPI 未初始化\n");
            return 0xFFFFFFFF;
        }
    }

    // [2026-10-22] 快路径: 1 + N 帧一次 ioctl 读完。
    //   第 0 帧丢弃(它的值来自切换通道之前的那次转换), 第 1..N 帧才是当前通道的新数据;
    //   与旧实现"丢首帧 + N 次采样平均"在数学上完全等价, 只是省掉了 2N 次系统调用。
    uint32_t buf[ADS8691_BURST_MAX];
    if (ads8691_read_burst(buf, ADS8691_AVG_N + 1) == 0) {
        uint32_t sum = 0;
        for (int i = 1; i <= ADS8691_AVG_N; i++) {
            sum += buf[i];
        }
        return (sum + ADS8691_AVG_N / 2) / ADS8691_AVG_N;   // 四舍五入
    }

    // ---- 回退路径: 内核/驱动不支持批量帧(或偶发传输失败)时用原来的逐帧方式 ----
    // (只提示一次, 避免驱动不支持时每点刷一行)
    static int burst_warned = 0;
    if (!burst_warned) {
        burst_warned = 1;
        printf("[ADS8691] ⚠️ 批量读取失败, 回退逐帧读取(本提示只打一次)\n");
    }
    uint32_t sum = 0;
    uint32_t n = 0;
    for (int a = 0; a < ADS8691_AVG_N; a++) {
        uint32_t v = ads8691_read_once();
        if (v == 0xFFFFFFFF) {
            continue;       // 本次失败, 丢弃该样本
        }
        sum += v;
        n++;
    }

    if (n == 0) {
        return 0xFFFFFFFF;  // N 次全失败, 如实上报错误
    }
    return (sum + n / 2) / n;   // 四舍五入
}









// ==================== 批量读取 ====================
int ADS8691_Read_Multi(uint32_t *buf, uint8_t count) {
    if (buf == NULL || count == 0) {
        printf("[ADS8691] ❌ 批量读取失败: 参数无效(buf=%p, count=%u)\n",
               (void *)buf, (unsigned)count);
        return -1;
    }
    if (g_spi == NULL) {
        if (ADS8691_Init() < 0) {
            printf("[ADS8691] ❌ 批量读取失败: SPI 未初始化\n");
            return -1;
        }
    }

    printf("[ADS8691] ▶ 批量读取开始: 共 %u 个采样点\n", (unsigned)count);
    
    for (int i = 0; i < count; i++) {
        buf[i] = ADS8691_Read();
        if (buf[i] == 0xFFFFFFFF) {
            printf("[ADS8691] ❌ 批量读取中断: 第 %d/%u 个采样点失败\n",
                   i + 1, (unsigned)count);
            return -1;
        }
        usleep(100);
    }

    printf("[ADS8691] ✅ 批量读取完成: 共 %u 个采样点, 首值=%u, 末值=%u\n",
           (unsigned)count, buf[0], buf[count - 1]);
    return 0;
}

// ==================== 关闭 ====================
void ADS8691_Close(void) {
    if (g_spi != NULL) {
        spi_close(g_spi);
        g_spi = NULL;
        printf("[ADS8691] ✅ 关闭完成: SPI 句柄已释放\n");
    }
}