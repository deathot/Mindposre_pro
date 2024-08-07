#include "pid.h"
// #include "maths.h"
//  pid积分限幅
#define PID_BODY_ROLL_INTEGRATION_LIMIT 200.0
#define PID_BODY_PITCH_INTEGRATION_LIMIT 200.0
#define PID_BODY_HIGH_INTEGRATION_LIMIT 200.0
#define PID_MOTOR_POSITION_INTEGRATION_LIMIT 7200.0

// pid输出限幅
#define PID_BODY_ROLL_OUTPUT_LIMIT 200.0
#define PID_BODY_PITCH_OUTPUT_LIMIT 200.0
#define PID_BODY_HIGH_OUTPUT_LIMIT 200.0
#define PID_MOTOR_POSITION_OUTPUT_LIMIT 6900.0

PidObject pid[PID_NUM];

// 初始化pid参数
void allPinInit(void)
{
	//	pidInit_t pidParam[PID_NUM];
	//
	//		pidParam[BODY_ROLL].kp = 1.0f;
	//		pidParam[BODY_ROLL].ki = 1.0f;
	//		pidParam[BODY_ROLL].kd = 1.0f;

	//		pidParam[BODY_PITCH].kp = 1.0f;
	//		pidParam[BODY_PITCH].ki = 1.0f;
	//		pidParam[BODY_PITCH].kd = 1.0f;
	//
	//		pidParam[BODY_HIGH].kp = 1.0f;
	//		pidParam[BODY_HIGH].ki = 1.0f;
	//		pidParam[BODY_HIGH].kd = 1.0f;
	//
	//		pidParam[MOTOR_A].kp = 1.0f;
	//		pidParam[MOTOR_A].ki = 0.2f;
	//		pidParam[MOTOR_A].kd = 2.0f;
	//
	//			pidParam[MOTOR_B].kp = 1.0f;
	//		pidParam[MOTOR_B].ki = 0.2f;
	//		pidParam[MOTOR_B].kd = 2.0f;
	//
	//
	//				pidParam[MOTOR_C].kp = 1.0f;
	//		pidParam[MOTOR_C].ki = 0.2f;
	//		pidParam[MOTOR_C].kd = 2.0f;
	//
	//					pidParam[MOTOR_D].kp = 1.0f;
	//		pidParam[MOTOR_D].ki = 0.2f;
	//		pidParam[MOTOR_D].kd = 2.0f;
	//
	//		pidInit(&pid[BODY_ROLL], pidParam[BODY_ROLL].kp, pidParam[BODY_ROLL].ki, pidParam[BODY_ROLL].kd,
	//		PID_BODY_ROLL_INTEGRATION_LIMIT, PID_BODY_ROLL_OUTPUT_LIMIT);
	//
	//		pidInit(&pid[BODY_PITCH], pidParam[BODY_PITCH].kp, pidParam[BODY_PITCH].ki, pidParam[BODY_PITCH].kd,
	//		PID_BODY_PITCH_INTEGRATION_LIMIT, PID_BODY_PITCH_OUTPUT_LIMIT);
	//
	//		pidInit(&pid[BODY_HIGH], pidParam[BODY_HIGH].kp, pidParam[BODY_HIGH].ki, pidParam[BODY_HIGH].kd,
	//		PID_BODY_HIGH_INTEGRATION_LIMIT, PID_BODY_HIGH_OUTPUT_LIMIT);
	//
	//		pidInit(&pid[MOTOR_A], pidParam[MOTOR_A].kp, pidParam[MOTOR_A].ki, pidParam[MOTOR_A].kd,
	//		PID_MOTOR_POSITION_INTEGRATION_LIMIT, PID_MOTOR_POSITION_OUTPUT_LIMIT);
	//
	//				pidInit(&pid[MOTOR_B], pidParam[MOTOR_B].kp, pidParam[MOTOR_B].ki, pidParam[MOTOR_B].kd,
	//		PID_MOTOR_POSITION_INTEGRATION_LIMIT, PID_MOTOR_POSITION_OUTPUT_LIMIT);
	//
	//				pidInit(&pid[MOTOR_C], pidParam[MOTOR_C].kp, pidParam[MOTOR_C].ki, pidParam[MOTOR_C].kd,
	//		PID_MOTOR_POSITION_INTEGRATION_LIMIT, PID_MOTOR_POSITION_OUTPUT_LIMIT);
	//
	//				pidInit(&pid[MOTOR_D], pidParam[MOTOR_D].kp, pidParam[MOTOR_D].ki, pidParam[MOTOR_D].kd,
	//		PID_MOTOR_POSITION_INTEGRATION_LIMIT, PID_MOTOR_POSITION_OUTPUT_LIMIT);
}

void pidInit(PidObject *pid, float kp, float ki, float kd, float iLimit, float outputLimit)
{
	pid->desired = 0;
	pid->error = 0;
	pid->prevError = 0;
	pid->integ = 0;
	pid->deriv = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->outP = 0;
	pid->outI = 0;
	pid->outD = 0;
	pid->iLimit = iLimit;
	pid->outputLimit = outputLimit;
}

float pidUpdate(PidObject *pid, float error) // error = desired - measured
{
	float output = 0.0f;

	pid->error = error;

	pid->integ += pid->error;

	// 积分限幅
	if (pid->iLimit != 0)
	{
		pid->integ = constrainf(pid->integ, -pid->iLimit, pid->iLimit);
	}

	pid->deriv = (pid->error - pid->prevError);

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;

	// 输出限幅
	if (pid->outputLimit != 0)
	{
		output = constrainf(output, -pid->outputLimit, pid->outputLimit);
	}

	pid->prevError = pid->error;

	return output;
}

float pidCaulate(PidObject *pid, float error, float rateT) // error = desired - measured
{
	float output = 0.0f;

	pid->error = error;

	pid->integ += pid->error;

	// 积分限幅
	if (pid->iLimit != 0)
	{
		pid->integ = constrainf(pid->integ, -pid->iLimit, pid->iLimit);
	}

	pid->deriv = rateT;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;

	// 输出限幅
	if (pid->outputLimit != 0)
	{
		output = constrainf(output, -pid->outputLimit, pid->outputLimit);
	}

	pid->prevError = pid->error;

	return output;
}

void pidReset(PidObject *pid)
{
	pid->error = 0;
	pid->prevError = 0;
	pid->integ = 0;
	pid->deriv = 0;
}

void pidResetIntegral(PidObject *pid)
{
	pid->integ = 0;
}

void pidSetIntegral(PidObject *pid, float integ)
{
	pid->integ = integ;
}

int constrain(int amt, int low, int high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

float constrainf(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
