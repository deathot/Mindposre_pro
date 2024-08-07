#ifndef __QUADRUPED_H
#define __QUADRUPED_H
#include "sys.h"
#include "inverse.h"
#include "pid.h"
#include "system.h"

// �����תһ�ܵı����������� ������������*���ٱ�*��Ƶ����500*27*4 = 54000
#define EncoderCircleNumber 54000
// ���ÿ��תһ�ȶ�Ӧ�ı��������� 54000/360=150
#define EncoderOneDegree 150
#define Ts 0.60				// ��̬����s
#define lam 0.5				// ռ�ձ�
#define detaT 0.001			// ʱ������
#define Tratimes Ts / detaT // һ�����ڵĴ���

#define QUADRUPED_TASK_PRIO 5  // �������ȼ�
#define QUADRUPED_STK_SIZE 800 // �����ջ��С

typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float high;
} control_t;
extern control_t control; /*������Ʋ���*/

typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float high;
} RelAttitude_t;
extern RelAttitude_t RelAttitude;

typedef struct
{
	float gyrox;
	float gyroy;
	float gyroz;
} RelRate_t;
extern RelRate_t RelRate;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float high;
} TarAttitude_t;
extern TarAttitude_t TarAttitude;
typedef struct
{
	short M1;
	short M2;
	short M3;
	short M4;
	short M5;
	short M6;
	short M7;
	short M8;
} EncoderTarget_t;
extern EncoderTarget_t EncoderTarget;

typedef struct
{
	int M1;
	int M2;
	int M3;
	int M4;
	int M5;
	int M6;
	int M7;
	int M8;
} MotorInit_t;

extern MotorInit_t MotorInit;
typedef struct
{
	float Leg1X;
	float Leg1Z;
	float Leg2X;
	float Leg2Z;
	float Leg3X;
	float Leg3Z;
	float Leg4X;
	float Leg4Z;
} TarTragectoryOut_t;
extern TarTragectoryOut_t TarTragectoryOut;
extern long int Target_1A, Target_1B, Target_2A, Target_2B;
extern long int Target_3A, Target_3B, Target_4A, Target_4B;
extern EncoderTarget_t EncoderTarget;
extern int TarTargectoryLeftX, TarTargectoryRightX;
void Quadruped_task(void *pvParameters);
void LegPositionInit(void);
void AttitudeControl(control_t *controlT, RelAttitude_t *RelAttitudeT, RelRate_t *RelRateT, TarAttitude_t *TarAttitudeT);
void UpdataFlags(u8 flag_t);
void MotorOutputChoose(void);
void FootTrajectoryLeg1(float TarX, float TarZ, int timeCount);
void FootTrajectoryLeg2(float TarX, float TarZ, int timeCount);
void FootTrajectoryLeg3(float TarX, float TarZ, int timeCount);
void FootTrajectoryLeg4(float TarX, float TarZ, int timeCount);

#endif
