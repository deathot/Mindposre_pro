#ifndef _BEE_H
#define _BEE_H
#include "sys.h"
#include "system.h"

#define BEE_TASK_PRIO 3
#define BEE_STK_SIZE 128
void bee_flash(void);
void bee_on(void);
void bee_off(void);
void Bee_Init(void);
void bee(int count, u8 flag);
void bee_task(void *pvParameters);
extern int bee_count;

#define PA12 PAout(12)

#endif
