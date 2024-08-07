#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos �����ļ� */
#include "FreeRTOSConfig.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*��������ͷ�ļ�*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu6050.h"
#include "show.h"
#include "key.h"
#include "pid.h"
#include "robot_select_init .h"

typedef struct
{
    float Encoder;     // ��������ֵ
    float Motor_Pwm;   // ���PWM��ֵ
    float Target;      // ���Ŀ���ٶ�ֵ
    float Velocity_KP; // �ٶȿ���PID����
    float Velocity_KI; // �ٶȿ���PID����
} Motor_parameter;

typedef struct
{
    float VX; // ���ƽ�����ƴ���������
    float VY; // ���ƽ�����ƴ���������
    float VZ; // ���ƽ�����ƴ���������
} Smooth_Control;

extern unsigned char temp_show;

void systemInit(void);
#define CONTROL_DELAY 1000 // �����ʵ��ʱ����10��
#define CAR_NUMBER 7       // һ�����ٸ���
#define RATE_1_HZ 1
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

/*һЩC�⺯�������ͷ�ļ�*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif /* __SYSTEM_H */
