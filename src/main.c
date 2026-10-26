#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "tcpserver.h"
#include "threadpool.h"
#include "dac8562.h"
#include "task.h"
#include "eth.h"
// [2026-10-13] SD 卡功能整体移除(现场不使用, 且挂载等待会拖慢启动)
#include "net_config.h"
#include "main.h"
#include "gpio_app.h"
#include "protocol.h"
#include "ota.h"
#include "fpga_i2c.h"
#include "ads1115.h"
#include "uart485.h"
#include "usart.h"
#include <mqueue.h>
#include "ads8691.h"
#include <sys/reboot.h>
#include "485_cpld.h"
// 本文件属于哪个分模块打印开关(必须在 #include "log.h" 之前定义)
#define LOG_MODULE  LOG_MOD_MAIN
#include "log.h"

// 编译时间戳: 由 Makefile 通过 -DBUILD_STAMP="..." 注入(每次 make 都重新求值)。
// 万一手动用 gcc 直接编译(没走 Makefile), 则回退到编译器宏, 保证始终有值。
#ifndef BUILD_STAMP
#define BUILD_STAMP __DATE__ " " __TIME__
#endif

#define SERVER_PORT 8080

threadpool_t *g_thread_pool = NULL;   // 全局线程池(全工程唯一, 定义处)
atomic_int adc_stop_flag;
atomic_int HV_stop_flag;
static volatile int running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}



int main(void)
{
    // [2026-10-13] 输出重定向: 开机自启时 stdout/stderr 是 /dev/console, 会与内核
    // printk、init 及其他启动脚本争抢同一个串口; 一旦 write 阻塞(而此时又持着日志锁),
    // 整个程序就卡在初始化中途(表现为日志停住、永远到不了"进入主循环")。
    // 重定向到内存文件可彻底避开该竞争, 且日志完整可查。
    // 需要手动调试、想看串口输出时: PROGRAM_ARM_LOG=console ./program_arm
    {
        const char *lp = getenv("PROGRAM_ARM_LOG");
        if (lp == NULL) lp = "/tmp/program_arm.log";
        if (strcmp(lp, "console") != 0) {
            freopen(lp, "w", stdout);
            freopen(lp, "w", stderr);
        }
    }

    // 行缓冲(在重定向之后设置, 作用于新的 stdout)
    setvbuf(stdout, NULL, _IOLBF, 4096);
    signal(SIGINT, signal_handler);

    // [2026-10-13] 清掉上次的"启动完成"标记; 本次跑到主循环时会重新写入。
    // 这样该文件始终代表"本次启动是否已就绪"。
    remove("/tmp/program_arm.ready");

    printf("[MAIN] ▶ 程序启动, 开始初始化各子系统\n");

    // [2026-10-13] 版本打印提前到所有初始化之前: 日志后段可能因串口拥塞而
    // 不完整, 放在第一行才能保证"开机一看便知板上跑的是哪一版"。
    // 编译时刻由编译器宏烙进二进制(无需时间头文件/RTC), 每次重编 main.c 都会更新。
    printf("[MAIN] 程序版本: app923, 编译于 %s\n", BUILD_STAMP);

    // 打印开关现状: 总开关 + 分模块掩码(上位机命令 0x66 可随时修改)
    printf("[MAIN]   总开关=%s, 分模块掩码=0x%04X (1=打开)\n",
           log_get_enabled() ? "打开" : "关闭",
           log_get_module_mask());
    {
        char mod_line[512];
        int off = 0;
        for (int bit = 0; bit < LOG_MOD_COUNT && off < (int)sizeof(mod_line); bit++) {
            off += snprintf(mod_line + off, sizeof(mod_line) - (size_t)off,
                            "%d=%s%s", bit, log_module_name(bit),
                            (bit + 1 < LOG_MOD_COUNT) ? " " : "");
        }
        printf("[MAIN]   分模块位号: %s\n", mod_line);
        printf("[MAIN]   提示: 上位机发命令 0x66(2 字节大端掩码)可单独开关各文件打印\n");
    }

    // OTA 回滚看门狗: 若上次升级处于"待确认"且本次未能确认成功, 将自动回滚并重启旧程序
    printf("[MAIN]   步骤: 启动 OTA 回滚看门狗 (超时 %d 秒)\n", OTA_WATCHDOG_SEC);
    ota_watchdog_start(OTA_WATCHDOG_SEC);
//   while (1)
//     {
//     // 发送命令 5，HEX数据就是 AA 05
//     int ret = cpld_send_command(5);
    
//     if (ret == 0) {
//         printf("发送成功，收到回复\n");
//     } else {
//         printf("发送失败\n");
//     }
// }



    printf("[MAIN]   步骤: 初始化 CPLD (485 转串口)\n");
    // [2026-10-13] 全工程只在这里初始化一次 CPLD(原来重复调用了 3 次, 每次都会
    // open("/dev/ttyS6") 造成 fd 泄漏)
    if (cpld_init() != 0) {
        printf("[MAIN] ❌ CPLD 初始化失败\n");
    } else {
        printf("[MAIN] ✅ CPLD 初始化完成\n");
    }


    // Select_Collect_Card_Channel_HV(639);

    //     printf("[MAIN]   步骤: 初始化 fpga\n");
    // fpgareset();




    printf("[MAIN]   步骤: 初始化 DAC8562 并设置 A 通道输出\n");
    if (DAC8562_Init() != 0) {
        printf("[MAIN] ❌ DAC8562 初始化失败\n");
    }
    usleep(10 * 1000);  // 等待 DAC 输出稳定
    ADS1115_Init();
    usleep(10 * 1000);  // 等待 DAC 输出稳定
    DAC8562_Set_Output_A(0);
    usleep(10 * 1000);  // 等待 DAC 输出稳定
    printf("[MAIN]   步骤: 触发 MCU-DAC (PD13 拉低)\n");
    mculdac0();
    usleep(10 * 1000);  // 等待 DAC 输出稳定
    printf("[MAIN]   步骤: 高压收尾 (关闭高压与开关)\n");
    Finish_HV();

    printf("[MAIN]   步骤: 读取一次 I/U 现场值\n");
    ADS1115_Read_ADC_I();
    ADS1115_Read_ADC_U();
//  const uint16_t seq[] = { 0, 2000, 4000, 6000, 8000, 10000,12000,14000,16000,18000,20000 };
// const unsigned seq_len = sizeof(seq) / sizeof(seq[0]);


//     for (unsigned k = 0; k < seq_len; k++) {
//         DAC8562_Set_Output_A(seq[k]);
//         usleep(500 * 1000);    
//                   // 等输出稳定
//         /* 这里可以读 ADC */

//     }



//     while (1) {
//   uint16_t ch[4];

//     /* 读四通道，PGA=±4.096V */
// ADS1115_Read_ADC_I();
// ADS1115_Read_ADC_U();

//     usleep(1000000);  // 延时 1 秒
//     ADS1115_Close();
//     }

    



//  uint8_t buf[2];

//         while (1){
//  cpld_send_hex_command_block(0x0A,0x24);
//     usleep(10000 * 1000);
    
//   int ret = cpld_recv_two(buf);
//     if (ret == 0) {
//         printf("收到: %02X %02X\n", buf[0], buf[1]);
//     } 
// }
    
  
    // [2026-10-13] SD 卡整块移除: 现场不使用 SD 卡, 且 sd_mount() 的挂载等待
    // 会拖慢启动(有 OTA 看门狗超时风险)。原第 3 次重复的 cpld_init() 一并删除。














    printf("[MAIN]   步骤: 创建线程池\n");
    // 创建线程池
    threadpool_t *pool = threadpool_create(THREAD_COUNT);
    if (pool == NULL)
    {
        printf("[MAIN] ❌ 创建线程池失败\n");
        return 1;
    }
    printf("[MAIN] ✅ 线程池已就绪, 工作线程数=%d\n", THREAD_COUNT);

    // 保存全局线程池(协议解析/任务分发直接使用)
    g_thread_pool = pool;

    // 自动配置本机 IP/网关(如需要请取消注释)
    printf("[MAIN]   步骤: 配置网卡 %s\n", NET_IFACE);
    if (net_config_init() != 0)
    {
        printf("[MAIN] ❌ 网卡配置失败: %s 未配置成功\n", NET_IFACE);
    }
    if (net_config_add_default_gw() != 0)
    {
        printf("[MAIN] ❌ 默认网关配置失败\n");
    }

    // 启动 TCP 服务器
    printf("[MAIN]   步骤: 启动 TCP 服务器 (端口 %d)\n", SERVER_PORT);
    if (tcp_server_start(SERVER_PORT) != 0)
    {
        printf("[MAIN] ❌ TCP 服务器启动失败\n");
        threadpool_destroy(pool);
        return 1;
    }
    printf("[MAIN] ✅ TCP 服务器已启动, 监听端口 %d\n", SERVER_PORT);

    // 启动 485 通道(独立线程,与 TCP 并行控制)
    printf("[MAIN]   步骤: 启动 485 控制通道\n");
    if (uart485_init() != 0)
    {
        printf("[MAIN] ❌ 485 通道启动失败 (不影响 TCP 运行)\n");
    }
    else
    {
        printf("[MAIN] ✅ 485 通道已启动\n");
    }

    struct mq_attr attr = {0};
attr.mq_maxmsg = 1024;
attr.mq_msgsize = 2;

printf("[MAIN]   步骤: 创建 ADC121 消息队列 /adc121s_data\n");
mqd_t mq = mq_open("/adc121s_data", O_CREAT | O_RDWR, 0644, &attr);
if (mq == (mqd_t)-1) {
    printf("[MAIN] ❌ 消息队列创建失败: %s\n", strerror(errno));
} else {
    printf("[MAIN] ✅ 消息队列已创建: /adc121s_data (maxmsg=%ld, msgsize=%ld)\n",
           (long)attr.mq_maxmsg, (long)attr.mq_msgsize);
}
    // atomic_store(&adc_stop_flag, 1);

    // atomic_store(&HV_stop_flag, 1);



    // [2026-10-13] 初值改为 1: 让 0x1B(设采样次数)/0x23(设扫描通道) 开箱可用。
    // 原为 0 时, 启动后这两个命令会被拒(回 0xF7 / 0xFFFFFFFF), 表现为"发命令没反应"。
    // 0x1D 扫描不检查此标志(它内部会置 0), 因此改成 1 不影响扫描流程。
    atomic_store(&adc_stop_flag, 1);

    atomic_store(&HV_stop_flag, 1);





    // 打印：485 通道启动成功
    printf("[MAIN]   步骤: 初始化 ADS8691 (±12V 量程)\n");

    ADS8691_Init();           // 第一次调用：初始化
    // ADS8691_Set_Range_12V(); 


//  ADS8691_Init_l();






    printf("[MAIN]   步骤: 初始化全部 GPIO\n");
    init_all_gpios();
   
    // 各子系统已就绪, 确认本次升级成功(OTA 使用)
    ota_mark_ok();

    // [2026-10-13] 落"启动完成"标记: 启动期串口输出可能被挤掉(带宽仅 11.5KB/s),
    // 但只要这个文件在, 就证明初始化完整跑到了主循环 —— 不依赖任何日志。
    // 上位机/测试脚本可轮询该文件(或看它时间戳)来判断"可以下发命令了"。
    {
        FILE *fp = fopen("/tmp/program_arm.ready", "w");
        if (fp) {
            fprintf(fp, "init ok\n");
            fclose(fp);
        } else {
            printf("[MAIN] ⚠️ 就绪标记写入失败: %s\n", strerror(errno));
        }
    }
    printf("[MAIN] ✅ 全部子系统初始化完成, 进入主循环 (日志: /tmp/program_arm.log, 就绪标记: /tmp/program_arm.ready)\n");
    printf("[MAIN] : 当前版本:app923 编译时间: %s\n", BUILD_STAMP);

    // 主循环(等待退出信号)
    while (running)
    {
        sleep(1);
    }

    // 清理资源
    printf("[MAIN] ▶ 收到退出信号, 开始清理资源\n");
    uart485_close();
    threadpool_destroy(pool);
    FPGA_I2C_Close();
    printf("[MAIN] ✅ 清理完成, 进程正常退出\n");
    return 0;
}