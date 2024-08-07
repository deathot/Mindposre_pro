#ifndef __KEY_H
#define __KEY_H
#include "sys.h"
#include "system.h"

extern u8 key_state;

#define KEY_TASK_PRIO 4  // 任务优先级
#define KEY_STK_SIZE 256 // 任务堆栈大小

#define KEY PBin(14)
#define MODE PBin(12)
void KEY_Init(void); // 按键初始化
void key_task(void *pvParameters);
u8 click_N_Double(u8 time); // 单击按键扫描和双击按键扫描
u8 click(void);             // 单击按键扫描
u8 Long_Press(void);        // 长按扫描
u8 select(void);
#endif
