#ifndef __BALANCE_H
#define __BALANCE_H
#include "sys.h"
#include "system.h"
#define BALANCE_TASK_PRIO 5  // 任务优先级
#define BALANCE_STK_SIZE 512 // 任务堆栈大小

void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert, float low, float high);
int target_limit_int(int insert, int low, int high);

u32 myabs(long int a);

void Key(void);
void Get_Encoder(void);

float float_abs(float insert);

int Position_PID_A(int Encoder, int Target, int iLimit);
int Position_PID_B(int Encoder, int Target, int iLimit);
int Position_PID_C(int Encoder, int Target, int iLimit);
int Position_PID_D(int Encoder, int Target, int iLimit);

typedef struct
{
    int Encoder;
    int Pwm;
    short Target;
} MOTOR_T;
extern MOTOR_T MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;

extern float Position_KP, Position_KI, Position_KD;

#endif
