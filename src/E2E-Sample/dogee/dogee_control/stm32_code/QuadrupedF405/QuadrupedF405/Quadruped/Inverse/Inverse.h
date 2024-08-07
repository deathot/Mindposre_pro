#ifndef __INVERSE_H
#define __INVERSE_H

#include "sys.h"
#include "math.h"
#include "system.h"
// 四足腿参数设定 单位：mm
// 电机减速比1：27
#define LegUp 90	// 上臂的机械长度
#define LegDown 162 // 下臂的机械长度

#define InitPositionX 0.0f
#define InitPositionZ 180.0f
extern u8 FlagStart;

enum
{
	Leg1f = 0,
	Leg1b,
	Leg2f,
	Leg2b,
	Leg3f,
	Leg3b,
	Leg4f,
	Leg4b
};

extern float TarMotorAngle[8];

typedef struct
{
	int Leg1f;
	int Leg1b;
	int Leg2f;
	int Leg2b;
	int Leg3f;
	int Leg3b;
	int Leg4f;
	int Leg4b;

} TargetMotorAngle_t;

extern TargetMotorAngle_t TargetMotorAngle;

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
} SubPositionLeg_t;

void MotorPositionInit(void);
extern SubPositionLeg_t LegTarPosition;

void UpdataTargerAngles(TargetMotorAngle_t *TarMotorAng_t, SubPositionLeg_t *TarPosition);

float InverseKinematicsLegAngle1(float Px_t, float Pz_t);
float InverseKinematicsLegAngle2(float Px_t, float Pz_t);
float InverseKinematicsLegAngle3(float Px_t, float Pz_t);
float InverseKinematicsLegAngle4(float Px_t, float Pz_t);

void UpdataLegAngle(void);
#endif
