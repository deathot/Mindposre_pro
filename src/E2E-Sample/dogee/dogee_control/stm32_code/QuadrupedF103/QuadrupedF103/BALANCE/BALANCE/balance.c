#include "balance.h"
int robot_mode_check_flag = 0; // 机器人模式是否出错检测标志位
int Time_count = 0;
MOTOR_T MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
float Position_KP = 1.0f, Position_KI = 0, Position_KD = 1.0f;
/**************************************************************************
函数功能：核心控制相关
入口参数：
返回  值：
**************************************************************************/
void Balance_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		//			if(Time_count<3000)Time_count++;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ)); // 此任务以100Hz的频率运行（10ms控制一次）
		Get_Encoder();									   // 获取编码器数据
		Key();											   // 按键修改陀螺仪零点
		MOTOR_A.Pwm = Position_PID_A(MOTOR_A.Encoder, MOTOR_A.Target, 3000);
		MOTOR_B.Pwm = Position_PID_B(MOTOR_B.Encoder, MOTOR_B.Target, 3000);
		MOTOR_C.Pwm = Position_PID_C(MOTOR_C.Encoder, MOTOR_C.Target, 3000);
		MOTOR_D.Pwm = Position_PID_D(MOTOR_D.Encoder, MOTOR_D.Target, 3000);

		Limit_Pwm(6900);
		Set_Pwm(-MOTOR_B.Pwm, -MOTOR_A.Pwm, MOTOR_C.Pwm, MOTOR_D.Pwm);
	}
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
	if (motor_a < 0)
		INA2 = 1, INA1 = 0;
	else
		INA2 = 0, INA1 = 1;
	PWMA = myabs(motor_a);

	if (motor_b < 0)
		INB2 = 1, INB1 = 0;
	else
		INB2 = 0, INB1 = 1;
	PWMB = myabs(motor_b);

	if (motor_c > 0)
		INC2 = 1, INC1 = 0;
	else
		INC2 = 0, INC1 = 1;
	PWMC = myabs(motor_c);

	if (motor_d > 0)
		IND2 = 1, IND1 = 0;
	else
		IND2 = 0, IND1 = 1;
	PWMD = myabs(motor_d);
}

/**************************************************************************
函数功能：限制PWM赋值
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{
	MOTOR_A.Pwm = target_limit_int(MOTOR_A.Pwm, -amplitude, amplitude);
	MOTOR_B.Pwm = target_limit_int(MOTOR_B.Pwm, -amplitude, amplitude);
	MOTOR_C.Pwm = target_limit_int(MOTOR_C.Pwm, -amplitude, amplitude);
	MOTOR_D.Pwm = target_limit_int(MOTOR_D.Pwm, -amplitude, amplitude);
}
/**************************************************************************
函数功能：限幅函数，设定高低阈值
入口参数：幅值
返回  值：
**************************************************************************/
float target_limit_float(float insert, float low, float high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}
int target_limit_int(int insert, int low, int high)
{
	if (insert < low)
		return low;
	else if (insert > high)
		return high;
	else
		return insert;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{
	u32 temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_A(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // 计算偏差
	Integral_bias += Bias;	 // 求出偏差的积分
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // 积分限幅
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																		   // 保存上一次偏差
	return Pwm;																				   // 增量输出
}
int Position_PID_B(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // 计算偏差
	Integral_bias += Bias;	 // 求出偏差的积分
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // 积分限幅
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																		   // 保存上一次偏差
	return Pwm;																				   // 增量输出
}
int Position_PID_C(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // 计算偏差
	Integral_bias += Bias;	 // 求出偏差的积分
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // 积分限幅
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																		   // 保存上一次偏差
	return Pwm;																				   // 增量输出
}
int Position_PID_D(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // 计算偏差
	Integral_bias += Bias;	 // 求出偏差的积分
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // 积分限幅
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
	Last_Bias = Bias;																		   // 保存上一次偏差
	return Pwm;																				   // 增量输出
}

/**************************************************************************
函数功能：按键即时更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp;
	tmp = click();
	if (tmp == 1)
		memcpy(Deviation_gyro, Original_gyro, sizeof(gyro)); // 单击更新陀螺仪零点
}
/**************************************************************************
函数功能：读取模式并采集编码器
入口参数：无
返回  值：无
**************************************************************************/
void Get_Encoder(void)
{
	MOTOR_A.Encoder += Read_Encoder(2);
	MOTOR_B.Encoder += Read_Encoder(3);
	MOTOR_C.Encoder += Read_Encoder(5);
	MOTOR_D.Encoder += Read_Encoder(4);
}
/**************************************************************************
函数功能：浮点型数据取绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if (insert >= 0)
		return insert;
	else
		return -insert;
}
