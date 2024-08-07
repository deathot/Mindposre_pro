#ifndef __KEY_H
#define __KEY_H
#include "sys.h"
#include "system.h"

extern u8 key_state;

#define KEY_TASK_PRIO 4  // �������ȼ�
#define KEY_STK_SIZE 256 // �����ջ��С

#define KEY PBin(14)
#define MODE PBin(12)
void KEY_Init(void); // ������ʼ��
void key_task(void *pvParameters);
u8 click_N_Double(u8 time); // ��������ɨ���˫������ɨ��
u8 click(void);             // ��������ɨ��
u8 Long_Press(void);        // ����ɨ��
u8 select(void);
#endif
