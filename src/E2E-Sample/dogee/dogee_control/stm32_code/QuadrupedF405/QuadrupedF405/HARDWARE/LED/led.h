#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_TASK_PRIO 3  // 任务优先级
#define LED_STK_SIZE 256 // 任务堆栈大小

// LED端口定义
#define LED0 PBout(13)

void LED_Init(void); // 初始化
void led_task(void *pvParameters);
#endif
