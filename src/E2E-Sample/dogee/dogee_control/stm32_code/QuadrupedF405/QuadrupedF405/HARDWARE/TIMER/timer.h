#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "system.h"

#define REMOTE_CHECK_STK_SIZE 128
#define REMOTE_CHECK_TASK_PRIO 3

/*-------------航模遥控程序使用和接线-----------
TIM1的 4个通道可以接航模遥控
PA8 PA9 PA10 PA11对应4个通道，具体的对应关系取决于控制代码
代码仅仅是采集，具体需要如使用这个指令可以编程设定
初始化航模遥控，即可通过中断读取航模遥控的脉宽，放在下列的变量里面
Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;//航模遥控采集相关变量
-----------航模遥控接线-----------*/

void TIM1_Cap_Init(u16 arr, u16 psc);
void TIM1_CC_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void Remote_Check_task(void *pvParameters);

extern u32 Remoter_Ch1, Remoter_Ch2, Remoter_Ch3, Remoter_Ch4;         // 航模遥控采集相关变量
extern u32 L_Remoter_Ch1, L_Remoter_Ch2, L_Remoter_Ch3, L_Remoter_Ch4; // 航模遥控接收变量
extern u8 Remote_Check_flag;
extern int Remote_count;
#endif
