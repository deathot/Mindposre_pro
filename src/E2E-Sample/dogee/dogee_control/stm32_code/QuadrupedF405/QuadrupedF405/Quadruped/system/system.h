#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos 配置文件 */
#include "FreeRTOSConfig.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usartx.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "show.h"
#include "Datascope_DP.h"
#include "key.h"
#include "adc.h"
#include "pstwo.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "get_control.h"
#include "motor.h"
#include "pid.h"
#include "inverse.h"
#include "quadruped.h"
#include "BEE.h"
#include "can.h"
#include "encoder.h"
#include "timer.h"

#define RATE_5_HZ 5
#define RATE_10_HZ 10
#define RATE_20_HZ 20
#define RATE_25_HZ 25
#define RATE_50_HZ 50
#define RATE_100_HZ 100
#define RATE_200_HZ 200
#define RATE_250_HZ 250
#define RATE_500_HZ 500
#define RATE_1000_HZ 1000
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_1000_HZ / RATE_HZ)) == 0)

#define PI 3.14159

// 解锁标志位
typedef enum
{
	LOCKED = (1 << 1),
	READY = (1 << 2),
	RUNNING = (1 << 3),
	STOP = (1 << 4),
	SHUTDWON = (1 << 5),
	ARMING_DISABLED_COMPASS_NOT_CALIBRATED = (1 << 6),
	ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED = (1 << 7),
} armingFlag_e;
extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

// 运动模式标志位
typedef enum
{
	ANGLE_MODE = (1 << 0),
	ACRO_MODE = (1 << 1),
	HEADFREE_MODE = (1 << 2),
	NAV_ALTHOLD_MODE = (1 << 3),
	NAV_RTH_MODE = (1 << 4),
	NAV_POSHOLD_MODE = (1 << 5),
	NAV_LAUNCH_MODE = (1 << 7),
	FAILSAFE_MODE = (1 << 8),
	BEEPER_ON_MODE = (1 << 9), // 蜂鸣器常开，寻机模式。
} flightModeFlags_e;
extern uint32_t motionModeFlags;

extern u8 RUN_Control;
extern u8 APP_ON_Flag, PS2_ON_Flag, USART_ON_FLAG, Remote_ON_Flag;
extern u8 APP_START_STOP_FLAG;
extern u8 APP_READY_FLAG, APP_STOP_FLAG, APP_KEY;
extern u8 Flag_Pose, Flag_Direction, Turn_Flag, PID_Send; // 蓝牙遥控相关的标志位
extern u8 USART_KEY, Remote_key_control_flag, START_COUNT, Remote_KEY;
extern u8 start_up_15_second;
void systemInit(void);
void SoftReset(void);

#endif /* __SYSTEM_H */
