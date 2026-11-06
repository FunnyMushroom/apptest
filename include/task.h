#ifndef TASK_H
#define TASK_H

#include <stdint.h>


void Task_Detect_Collection_Card(void *arg);
void Task_Detect_Switch_Card(void *arg);
void Task_Set_HV(void *arg);
void Task_Read_HV(void *arg);
void Task_Control_Times_HV(void *arg);
void Task_Set_Buff(void *arg);
void Task_Check_Self(void *arg);
void Task_Reset_Fpga(void *arg);
void Task_Test_En(void *arg);
void Task_Test_Stop(void *arg);
void Task_Test_Select_Collect_Card(void *arg);
void LXLCESHI(void *arg);
void Task_CESHI(void *arg);
void Task_CESHI2(void *arg);
void Task_SETBUFF_CESHI(void *arg);
void Task_Test_Select_Switch_Card(void *arg);
void Task_Test_Select_Collect_Card_Channel(void *arg) ;
void Task_Test_Read_ADC121_Times(void *arg);
void Task_Read_HV_I(void *arg);
void Task_Realtime_VI_Print(void *arg);
void Task_Adc_Stop(void *arg);
void Task_ADC_Auto_Scan(void *arg);
void Task_Set_Scan_Channels(void *arg);
void Task_Disable_All_Switch(void *arg);
void Task_Set_Print_Mask(void *arg);

// ==================== [IDLE-CLOSE] 3 秒无操作自动关闭开关通道 ====================
// 实现全部在 src/task.c 顶部的 "IDLE-CLOSE" 段, 排查问题时搜 IDLE-CLOSE 即可。
//   idle_close_start()            启动看门狗线程(main.c 调用一次)
//   idle_close_note_activity()    protocol.c 每收到一条合法命令调用, 刷新计时
//   idle_close_hold_enter/leave() 长任务(0x1D 扫描)期间挂起空闲判定, 必须成对
//   idle_close_wait_if_closing()  0x23/0x1D 开头等待"正在进行的自动关闭"结束
int  idle_close_start(void);
void idle_close_note_activity(void);
void idle_close_hold_enter(const char *who);
void idle_close_hold_leave(const char *who);
void idle_close_wait_if_closing(const char *who);

#endif