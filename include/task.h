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
#endif