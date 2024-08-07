#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_TASK_PRIO 3  // �������ȼ�
#define LED_STK_SIZE 256 // �����ջ��С

// LED�˿ڶ���
#define LED0 PBout(13)

void LED_Init(void); // ��ʼ��
void led_task(void *pvParameters);
#endif
