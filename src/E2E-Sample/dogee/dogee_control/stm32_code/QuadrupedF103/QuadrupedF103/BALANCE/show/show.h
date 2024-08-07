#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "oled.h"
#include "system.h"
#define SHOW_TASK_PRIO 2
#define SHOW_STK_SIZE 512

void show_task(void *pvParameters);
void oled_show(void);
void APP_Show(void);

#endif
