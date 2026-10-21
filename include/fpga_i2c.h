#ifndef FPGA_I2C_H
#define FPGA_I2C_H

#include <stdint.h>
#include <stdbool.h>

// FPGA I2C 寄存器地址
#define FPGA_REG_STATUS0        0x00
#define FPGA_REG_STATUS1        0x01
#define FPGA_REG_STATUS2        0x02
#define FPGA_REG_STATUS3        0x03
#define FPGA_REG_HV_STATE       0x04
#define FPGA_REG_CMD_STATE0     0x05
#define FPGA_REG_CMD_STATE1     0x06

// 命令寄存器 0x05 位定义
#define FPGA_BIT_HV_EN         0
#define FPGA_BIT_DISCHARGE_EN  1
#define FPGA_BIT_FPGA_RESET    2
#define FPGA_BIT_HV_TEST       3

// I2C 从机地址与总线设备
#define FPGA_I2C_ADDR           0x04
#define FPGA_I2C_DEVICE         "/dev/i2c-1"

uint8_t FPGA_Build_CMD1(uint8_t channel);
uint8_t FPGA_Build_CMD2(uint8_t channel);
uint16_t Check_State_Collect(void);
uint16_t Check_State_Switch(void);
bool     Check_HV_State(void);
uint8_t  Read_Info_State_Reg(void);
void     Print_Info_State(uint8_t info_state);
uint8_t  Read_CMD1_Before(void);
uint8_t  Read_CMD2_Before(void);
uint8_t  Select_Collect_Card(uint8_t num);
uint8_t  Select_Collect_Card_Channel(uint8_t num );
bool     HV_Test_En(uint8_t num);
bool     FPGA_Reset_En(uint8_t num);
bool     Discharge_En(uint8_t num);
bool     MCU_Test_En(uint8_t num);
uint8_t FPGA_Set_Buff(uint8_t num);
void     Read_All_Registers(void);
// 共享 I2C 句柄: 只初始化一次, 用完调用 FPGA_I2C_Close() 显式关闭
void     FPGA_I2C_Close(void);
int Test_I2C_Read_CMD1(void);
uint16_t map_sw(uint16_t input);
void Test_I2C_Write_Read(void);
uint16_t Select_Collect_Card_Channel_HV(uint16_t num);
bool teststart(void);
bool teststop(void);
uint8_t Finish_HV();

uint8_t START_HV_CHECK() ;

// [2026-10-14] 读取 0x04/0x05 并打包返回(高字节=0x04, 低字节=0x05); 失败返回 0xFFFF
uint16_t FPGA_Read_State_Regs(void);

// [2026-10-14] 调试用: 写寄存器 reg=val, 再回读 0x04/0x05 打包返回(高=0x04, 低=0x05)
uint16_t FPGA_Write_Reg_And_Read(uint8_t reg, uint8_t val);
#endif // FPGA_I2C_H
