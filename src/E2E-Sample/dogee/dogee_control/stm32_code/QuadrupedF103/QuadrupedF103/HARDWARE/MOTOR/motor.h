#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"

#define PWMD TIM8->CCR2
#define PWMC TIM8->CCR1
#define PWMB TIM8->CCR4
#define PWMA TIM8->CCR3
#define IND1 PCout(12)
#define INC1 PDout(2)
#define INB1 PCout(5)
#define INA1 PCout(4)

#define IND2 PBout(5)
#define INC2 PBout(4)
#define INB2 PBout(1)
#define INA2 PBout(0)

#define EN PAin(12)
void Enable_Pin(void);
void MiniBalance_PWM_Init(u16 arr, u16 psc);
void MiniBalance_Motor_Init(void);
void Servo_PWM_Init(u16 arr, u16 psc);
#endif
