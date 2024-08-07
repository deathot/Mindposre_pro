#include "balance.h"
int robot_mode_check_flag = 0; // ������ģʽ�Ƿ�������־λ
int Time_count = 0;
MOTOR_T MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
float Position_KP = 1.0f, Position_KI = 0, Position_KD = 1.0f;
/**************************************************************************
�������ܣ����Ŀ������
��ڲ�����
����  ֵ��
**************************************************************************/
void Balance_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		//			if(Time_count<3000)Time_count++;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ)); // ��������100Hz��Ƶ�����У�10ms����һ�Σ�
		Get_Encoder();									   // ��ȡ����������
		Key();											   // �����޸����������
		MOTOR_A.Pwm = Position_PID_A(MOTOR_A.Encoder, MOTOR_A.Target, 3000);
		MOTOR_B.Pwm = Position_PID_B(MOTOR_B.Encoder, MOTOR_B.Target, 3000);
		MOTOR_C.Pwm = Position_PID_C(MOTOR_C.Encoder, MOTOR_C.Target, 3000);
		MOTOR_D.Pwm = Position_PID_D(MOTOR_D.Encoder, MOTOR_D.Target, 3000);

		Limit_Pwm(6900);
		Set_Pwm(-MOTOR_B.Pwm, -MOTOR_A.Pwm, MOTOR_C.Pwm, MOTOR_D.Pwm);
	}
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
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
�������ܣ�����PWM��ֵ
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{
	MOTOR_A.Pwm = target_limit_int(MOTOR_A.Pwm, -amplitude, amplitude);
	MOTOR_B.Pwm = target_limit_int(MOTOR_B.Pwm, -amplitude, amplitude);
	MOTOR_C.Pwm = target_limit_int(MOTOR_C.Pwm, -amplitude, amplitude);
	MOTOR_D.Pwm = target_limit_int(MOTOR_D.Pwm, -amplitude, amplitude);
}
/**************************************************************************
�������ܣ��޷��������趨�ߵ���ֵ
��ڲ�������ֵ
����  ֵ��
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
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
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
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ��
e(k-1)������һ�ε�ƫ��
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Position_PID_A(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // ����ƫ��
	Integral_bias += Bias;	 // ���ƫ��Ļ���
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // �����޷�
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																		   // ������һ��ƫ��
	return Pwm;																				   // �������
}
int Position_PID_B(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // ����ƫ��
	Integral_bias += Bias;	 // ���ƫ��Ļ���
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // �����޷�
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																		   // ������һ��ƫ��
	return Pwm;																				   // �������
}
int Position_PID_C(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // ����ƫ��
	Integral_bias += Bias;	 // ���ƫ��Ļ���
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // �����޷�
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																		   // ������һ��ƫ��
	return Pwm;																				   // �������
}
int Position_PID_D(int Encoder, int Target, int iLimit)
{
	static float Bias, Pwm, Integral_bias, Last_Bias;
	Bias = Target - Encoder; // ����ƫ��
	Integral_bias += Bias;	 // ���ƫ��Ļ���
	if (Integral_bias > iLimit)
		Integral_bias = iLimit;
	if (Integral_bias < (-iLimit))
		Integral_bias = -iLimit;															   // �����޷�
	Pwm = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // λ��ʽPID������
	Last_Bias = Bias;																		   // ������һ��ƫ��
	return Pwm;																				   // �������
}

/**************************************************************************
�������ܣ�������ʱ�������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
	u8 tmp;
	tmp = click();
	if (tmp == 1)
		memcpy(Deviation_gyro, Original_gyro, sizeof(gyro)); // �����������������
}
/**************************************************************************
�������ܣ���ȡģʽ���ɼ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Encoder(void)
{
	MOTOR_A.Encoder += Read_Encoder(2);
	MOTOR_B.Encoder += Read_Encoder(3);
	MOTOR_C.Encoder += Read_Encoder(5);
	MOTOR_D.Encoder += Read_Encoder(4);
}
/**************************************************************************
�������ܣ�����������ȡ����ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if (insert >= 0)
		return insert;
	else
		return -insert;
}
