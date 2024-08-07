#ifndef __PID_H
#define __PID_H
// #include "config_param.h"
// #include "filter.h"
#include "sys.h"
#include "system.h"

enum pidIndex
{
	BODY_ROLL = 0,
	BODY_PITCH,
	BODY_YAW,
	BODY_HIGH,
	//	MOTOR_A,
	//	MOTOR_B,
	//	MOTOR_C,
	//	MOTOR_D,
	PID_NUM
};

typedef struct
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	float desired;	   //< set point
	float error;	   //< error
	float prevError;   //< previous error
	float integ;	   //< integral
	float deriv;	   //< derivative
	float kp;		   //< proportional gain
	float ki;		   //< integral gain
	float kd;		   //< derivative gain
	float outP;		   //< proportional output (debugging)
	float outI;		   //< integral output (debugging)
	float outD;		   //< derivative output (debugging)
	float iLimit;	   //< integral limit
	float outputLimit; //< total PID output limit, absolute value. '0' means no limit.
} PidObject;
extern PidObject pid[PID_NUM];
void allPinInit(void);

/*pid�ṹ���ʼ��*/
void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit);
float pidCaulate(PidObject *pid, float error, float rateT);

float pidUpdate(PidObject *pid, float error);
void pidReset(PidObject *pid);
void pidResetIntegral(PidObject *pid);
void pidSetIntegral(PidObject *pid, float integ);

int constrain(int amt, int low, int high);
float constrainf(float amt, float low, float high);
#endif /* __PID_H */
