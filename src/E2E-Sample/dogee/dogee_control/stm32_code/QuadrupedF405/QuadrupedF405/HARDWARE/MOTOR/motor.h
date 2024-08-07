#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>
#include "stm32f4xx_tim.h"
#include "system.h"

#define PWMD TIM8->CCR2 // PC7
#define PWMC TIM8->CCR1 // PC6
#define PWMB TIM8->CCR4 // PC9
#define PWMA TIM8->CCR3 // PC8

#define INA1 PCout(4)
#define INA2 PBout(0)

#define INB1 PCout(5)
#define INB2 PBout(1)

#define INC1 PDout(2)
#define INC2 PBout(4)

#define IND1 PCout(12)
#define IND2 PBout(5)

void MiniBalance_PWM_Init(u16 arr, u16 psc);
void MiniBalance_Motor_Init(void);
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d);
void PowerOutputControl(void);
u32 myabs(long int a);
extern long int Motor_A, Motor_B, Motor_C, Motor_D;
#endif
