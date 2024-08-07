#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "system.h"

#define REMOTE_CHECK_STK_SIZE 128
#define REMOTE_CHECK_TASK_PRIO 3

/*-------------��ģң�س���ʹ�úͽ���-----------
TIM1�� 4��ͨ�����ԽӺ�ģң��
PA8 PA9 PA10 PA11��Ӧ4��ͨ��������Ķ�Ӧ��ϵȡ���ڿ��ƴ���
��������ǲɼ���������Ҫ��ʹ�����ָ����Ա���趨
��ʼ����ģң�أ�����ͨ���ж϶�ȡ��ģң�ص������������еı�������
Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;//��ģң�زɼ���ر���
-----------��ģң�ؽ���-----------*/

void TIM1_Cap_Init(u16 arr, u16 psc);
void TIM1_CC_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void Remote_Check_task(void *pvParameters);

extern u32 Remoter_Ch1, Remoter_Ch2, Remoter_Ch3, Remoter_Ch4;         // ��ģң�زɼ���ر���
extern u32 L_Remoter_Ch1, L_Remoter_Ch2, L_Remoter_Ch3, L_Remoter_Ch4; // ��ģң�ؽ��ձ���
extern u8 Remote_Check_flag;
extern int Remote_count;
#endif
