#include "quadruped.h"

control_t control; /*������Ʋ���*/
RelAttitude_t RelAttitude;
TarAttitude_t TarAttitude;
RelRate_t RelRate;
MotorInit_t MotorInit;
TarTragectoryOut_t TarTragectoryOut;
EncoderTarget_t EncoderTarget;

int Time_t = 0, Time2_t = 0;						 // ����㣬��������֧����Ͱڶ���
long int Target_1A, Target_1B, Target_2A, Target_2B; // �˸������Ŀ����ת�Ƕ�
long int Target_3A, Target_3B, Target_4A, Target_4B;

int TarTargectoryLeftX = 0, TarTargectoryRightX = 0; // ��������Ⱥ��ұ��ȵĿ�����
void Quadruped_task(void *pvParameters)
{
	u32 tick = 0;
	u32 lastWakeTime = getSysTickCnt(); // ��ȡ�����ϴλ���ʱ�䣬����ִ�о�����ʱ����
	while (1)
	{

		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ)); // ������ʱ����������������1000HzƵ������
		//************************************************************************************************************************//
		// ���±�����������
		//************************************************************************************************************************//
		UpdataEncoder();

		//************************************************************************************************************************//
		// ���������̬������
		// ʵ������̬���Ƶ�ԭ����ͨ���ȵĸ߶���ϣ���ʵ�ֲ�ͬ�Ļ�����̬
		// PS2����ֱ�ҡ��/APP�İ���ģʽ ���Կ������㾲ֹʱˮƽ�������̬��ҡ�˲���ʱ����Ϊˮƽ
		//************************************************************************************************************************//
		TarAttitude.pitch = constrain((int)-smooth_pose_y, -50, 50); // ����ǰ����
		TarAttitude.roll = constrain((int)-smooth_pose_x, -40, 40);	 // �������Ҳ���

		control.pitch = pidUpdate(&pid[BODY_PITCH], TarAttitude.pitch - RelAttitude.pitch); // ��Ŀ��Ƕȼ�ȥ�����ĽǶ�
		control.roll = pidUpdate(&pid[BODY_ROLL], TarAttitude.roll - RelAttitude.roll);		// �Ի�����̬��һ��PIDƽ�⻷

		//************************************************************************************************************************//
		// �����˶�������
		//************************************************************************************************************************//
		TarTargectoryLeftX = move_x - constrain(move_z, -55, 55);  // ������ߵ���
		TarTargectoryRightX = move_x + constrain(move_z, -55, 55); // �����ұߵ���

		TarTargectoryLeftX = constrain(TarTargectoryLeftX, -90, 90);   // ����������޷�
		TarTargectoryRightX = constrain(TarTargectoryRightX, -90, 90); // ����������޷�

		//************************************************************************************************************************//
		// ��˹켣�滮��Ҫ�õ���ʱ��㣬��������֧����Ͱڶ���
		//************************************************************************************************************************//
		if (Time_t == 0)
			Time_t = Tratimes;
		Time_t--;
		if (Time2_t == Tratimes)
			Time2_t = 0;
		Time2_t++;

		//************************************************************************************************************************//
		// ��̬����˼·����һ�������������˶����ڣ��˶��켣��ÿ���㻮�ֳ�����ÿ������Ϊһ��Ŀ�������ȥʵ��
		//************************************************************************************************************************//
		// �����������ɵ�һ���ȣ���һ����Ϊһ������������
		// �������룬ʵʱ�����X��Z�᷽�������
		/*************��ߵ�������*****************/
		FootTrajectoryLeg1(TarTargectoryLeftX, -70, Time_t); // ����·���滮������������������̧�����ߵ�Ϊ-70�����߶ȳ���70ʱ������Ӧ�ٶȸ�����
		FootTrajectoryLeg2(TarTargectoryLeftX, -70, Time_t);
		/*************�ұߵ�������*****************/
		FootTrajectoryLeg3(TarTargectoryRightX, -70, Time2_t);
		FootTrajectoryLeg4(TarTargectoryRightX, -70, Time2_t);

		//************************************************************************************************************************//
		// Ԥ�˶�״̬(READY)  ���� �к��� �˶���״̬�л�ʱ��������һ��̤�������ɣ����˶���ʱ����ᾲֹ��̤��
		// �˶���̤��(RUNNING)���������޿�����ʱ����̤�� ��PS2ģʽ�ſ��ã�
		// վ������           ��ǿ������˲��ֹͣ�˶����뾲ֹ̬��READY , RUNNING�����ã�
		// z�᷽���������Ҫ����һ��������֧��ֵ����Ϊz����=0ʱ��������������״̬�������磺-180��-150
		//************************************************************************************************************************//
		if (ARMING_FLAG(READY)) // Ԥ�˶�״̬
		{
			if (RUN_Control == 0) // û���յ�����������ʱ���㾲ֹ
			{
				LegTarPosition.Leg1X = 0;
				LegTarPosition.Leg1Z = -180 + control.pitch - control.roll; // �߶Ȼ�+�켣����+������̬ƽ�⣨����ƽ���ǰ��ƽ�⣩

				LegTarPosition.Leg2X = 0;
				LegTarPosition.Leg2Z = -180 + control.pitch + control.roll;

				LegTarPosition.Leg3X = 0;
				LegTarPosition.Leg3Z = -180 - control.pitch - control.roll;

				LegTarPosition.Leg4X = 0;
				LegTarPosition.Leg4Z = -180 - control.pitch + control.roll;
			}
			else if (RUN_Control == 1) // ���յ�����������̤���û������ȶ�
			{
				LegTarPosition.Leg1X = 0;															 // x�᷽��������0��������̤��
				LegTarPosition.Leg1Z = -150 + TarTragectoryOut.Leg1Z + control.pitch - control.roll; // �߶Ȼ�+�켣����+������̬ƽ�⣨����ƽ���ǰ��ƽ�⣩

				LegTarPosition.Leg2X = 0;
				LegTarPosition.Leg2Z = -150 + TarTragectoryOut.Leg2Z + control.pitch + control.roll;

				LegTarPosition.Leg3X = 0;
				LegTarPosition.Leg3Z = -150 + TarTragectoryOut.Leg3Z - control.pitch - control.roll;

				LegTarPosition.Leg4X = 0;
				LegTarPosition.Leg4Z = -150 + TarTragectoryOut.Leg4Z - control.pitch + control.roll;
			}
			else if (RUN_Control == 2) // �����ȶ����ٿ�ʼ�˶�
			{
				LegTarPosition.Leg1X = TarTragectoryOut.Leg1X;										 // �켣����
				LegTarPosition.Leg1Z = -150 + TarTragectoryOut.Leg1Z + control.pitch - control.roll; // �߶Ȼ�+�켣����+������̬ƽ�⣨����ƽ���ǰ��ƽ�⣩

				LegTarPosition.Leg2X = TarTragectoryOut.Leg2X;
				LegTarPosition.Leg2Z = -150 + TarTragectoryOut.Leg2Z + control.pitch + control.roll;

				LegTarPosition.Leg3X = TarTragectoryOut.Leg3X;
				LegTarPosition.Leg3Z = -150 + TarTragectoryOut.Leg3Z - control.pitch - control.roll;

				LegTarPosition.Leg4X = TarTragectoryOut.Leg4X;
				LegTarPosition.Leg4Z = -150 + TarTragectoryOut.Leg4Z - control.pitch + control.roll;
			}
		}

		else if (ARMING_FLAG(RUNNING)) // ̤�����˶���PS2ģʽ���ã�
		{
			LegTarPosition.Leg1X = TarTragectoryOut.Leg1X;										 // �켣����
			LegTarPosition.Leg1Z = -150 + TarTragectoryOut.Leg1Z + control.pitch - control.roll; // �߶Ȼ�+�켣����+������̬ƽ�⣨����ƽ���ǰ��ƽ�⣩

			LegTarPosition.Leg2X = TarTragectoryOut.Leg2X;
			LegTarPosition.Leg2Z = -150 + TarTragectoryOut.Leg2Z + control.pitch + control.roll;

			LegTarPosition.Leg3X = TarTragectoryOut.Leg3X;
			LegTarPosition.Leg3Z = -150 + TarTragectoryOut.Leg3Z - control.pitch - control.roll;

			LegTarPosition.Leg4X = TarTragectoryOut.Leg4X;
			LegTarPosition.Leg4Z = -150 + TarTragectoryOut.Leg4Z - control.pitch + control.roll;
		}
		else // վ������
		{
			LegTarPosition.Leg1X = 0;
			LegTarPosition.Leg1Z = -180 + control.pitch - control.roll; // ����վ���͸߶Ⱥ��˶��ĸ߶Ȳ�ͬ

			LegTarPosition.Leg2X = 0;
			LegTarPosition.Leg2Z = -180 + control.pitch + control.roll;

			LegTarPosition.Leg3X = 0;
			LegTarPosition.Leg3Z = -180 - control.pitch - control.roll;

			LegTarPosition.Leg4X = 0;
			LegTarPosition.Leg4Z = -180 - control.pitch + control.roll;
		}

		LegTarPosition.Leg1Z = constrain((int)LegTarPosition.Leg1Z, -230, -100);
		LegTarPosition.Leg2Z = constrain((int)LegTarPosition.Leg2Z, -230, -100);
		LegTarPosition.Leg3Z = constrain((int)LegTarPosition.Leg3Z, -230, -100);
		LegTarPosition.Leg4Z = constrain((int)LegTarPosition.Leg4Z, -230, -100); // ����ȵĸ߶��޷�

		//************************************************************************************************************************//
		// ���˶�ѧ���㣬���ÿ�������Ե�Ŀ������Ƕ�
		//************************************************************************************************************************//
		UpdataTargerAngles(&TargetMotorAngle, &LegTarPosition); // �����ȵ�Ŀ����������۵�ʵ��Ŀ��Ƕ�

		//************************************************************************************************************************//
		// ����������ֵ��Ӧʵ�ʵı۵�λ��
		// ���㷽ʽ���ñ������Ļ�׼ֵ���ϣ��������Ǽ���Ŀ��Ƕ�
		// Ŀ��Ƕ���Ҫת����ʵ�ʶ�Ӧ�ı���������ֵ�����յó��µı�������ֵ��
		//************************************************************************************************************************//
		//================���ʵ��Ŀ����ת�Ƕ�================
		Target_1A = MotorInit.M1 - TargetMotorAngle.Leg1f * EncoderOneDegree;
		Target_1B = MotorInit.M2 + (180 - TargetMotorAngle.Leg1b) * EncoderOneDegree;
		Target_2A = MotorInit.M3 - TargetMotorAngle.Leg2f * EncoderOneDegree;
		Target_2B = MotorInit.M4 + (180 - TargetMotorAngle.Leg2b) * EncoderOneDegree;

		Target_3A = MotorInit.M5 - TargetMotorAngle.Leg3f * EncoderOneDegree;
		Target_3B = MotorInit.M6 + (180 - TargetMotorAngle.Leg3b) * EncoderOneDegree;
		Target_4A = MotorInit.M7 - TargetMotorAngle.Leg4f * EncoderOneDegree;
		Target_4B = MotorInit.M8 + (180 - TargetMotorAngle.Leg4b) * EncoderOneDegree;

		MotorOutputChoose();  // ���� ����/�˶�/��ֹ/�ػ� ״̬��⣬�������Ӧ��Ŀ���������ֵ
		PowerOutputControl(); // ���ص��ĸ����λ�û�PID���㣲����ֵPWM

		//************************************************************************************************************************//
		// ���ӻ����Ƶ��ĸ����Ŀ��λ��ֵ����Ϣ���͸��ӻ�
		//************************************************************************************************************************//
		CAN_data_transition(); // ��Ҫ���͸��ӻ������ݽ��з�װ
		CAN1_SEND();		   // ͨ��CAN�����ݷ��ͳ�ȥ

		/*******************************************************************************************************************/
		tick++;
	}
}

/**************************************************************************
�������ܣ����½�����־
��ڲ�����״̬�л�ֵ��APP�����ڡ���ģ ����ʱģ����PS2���Ƶļ�λֵ��ʵ��ͬ����Ч��
����  ֵ����
**************************************************************************/
void UpdataFlags(u8 flag_t)
{
	switch (flag_t)
	{
	case 0x01: // select
	{
		ENABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(READY);
		break;
	}
	case 0x04: // �����ֱ���start���������㿪ʼվ��
	{
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(STOP);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(LOCKED); // ʹ�ܽ���
		break;
	}
	case 5:
		break; // Ԥ��
	case 6:
		break; // Ԥ��
	case 7:
		break; // Ԥ��
	case 8:
		break; // Ԥ��

	case 13: // ��-�� �ֱ��������ΰ��� ��̤����
	{
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(STOP);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(RUNNING); // ʹ���˶�
		break;
	}
	case 14: // ��-��  �ֱ���Բ�ΰ�����ֹͣ��
	{
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(STOP);
		break;
	}
	case 15: // ��-�� �ֱ��Ĳ�水����Ԥ��̬��
	{
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(STOP);
		DISABLE_ARMING_FLAG(SHUTDWON);
		ENABLE_ARMING_FLAG(READY);
		break;
	}
	case 16:
	{ // ��-�� �ֱ��������ΰ��� ����λ��
		DISABLE_ARMING_FLAG(READY);
		DISABLE_ARMING_FLAG(RUNNING);
		DISABLE_ARMING_FLAG(STOP);
		ENABLE_ARMING_FLAG(SHUTDWON); // ʹ�ܹ�λ
		break;
	}
	default:
		break;
	}
}

/**************************************************************************
�������ܣ�ѡ��ÿ������Ŀ�������
��ڲ�������
����  ֵ����
�˺���˵���������ϵ缰���е����̣�����ɱ�ʾΪ
�ϵ� EncoderTarget.XX = 0����ʼ�ϵ��ʱ�����㾲ֹ����
|
վ������ʱ�������״̬���ս���֮������վ����
|
̤��/���� ���������㿪ʼ�˶������Ե��ڿ粽�Ĵ�С
|
����ػ�
**************************************************************************/
u8 flag_idle = 1;
u32 time_start = 0;
void MotorOutputChoose(void)
{
	// �����󣬻��建��վ����
	if (ARMING_FLAG(LOCKED)) // ����״̬
	{
		if (flag_idle == 1) // ���������ȿ�ʼվ��
		{
			time_start++;

			EncoderTarget.M1 = -4.0f * time_start; // ���ڵ���İ�װ��ʽ������ȡ������ͬ
			EncoderTarget.M2 = 4.0f * time_start;
			EncoderTarget.M3 = -4.0f * time_start;
			EncoderTarget.M4 = 4.0f * time_start;

			EncoderTarget.M5 = -4.0f * time_start;
			EncoderTarget.M6 = 4.0f * time_start;
			EncoderTarget.M7 = -4.0f * time_start;
			EncoderTarget.M8 = 4.0f * time_start;

			if (time_start >= 4250) // 425*4=17000������ͨ�������������ı�����վ���͹�λ���̵��ٶ�
			{
				time_start = 0; // �������㣬վ�����̽���
				flag_idle = 2;	// վ��������־λ
			}
		}

		if (ARMING_FLAG(STOP) && (flag_idle == 2)) // ����վ��ֹͣ ����̬����
		{
			EncoderTarget.M1 = -17000;
			EncoderTarget.M2 = 17000;
			EncoderTarget.M3 = -17000;
			EncoderTarget.M4 = 17000; // �����ϵ翪��������վ����Ŀ���������ֵ

			EncoderTarget.M5 = -17000;
			EncoderTarget.M6 = 17000;
			EncoderTarget.M7 = -17000;
			EncoderTarget.M8 = 17000;
		}
		else if (ARMING_FLAG(SHUTDWON) && (flag_idle == 2)) // ����ػ���������վ������ԭ����ͬ
		{
			time_start++;

			EncoderTarget.M1 = -17000 + 4.0f * time_start;
			EncoderTarget.M2 = 17000 - 4.0f * time_start;
			EncoderTarget.M3 = -17000 + 4.0f * time_start;
			EncoderTarget.M4 = 17000 - 4.0f * time_start;

			EncoderTarget.M5 = -17000 + 4.0f * time_start;
			EncoderTarget.M6 = 17000 - 4.0f * time_start;
			EncoderTarget.M7 = -17000 + 4.0f * time_start;
			EncoderTarget.M8 = 17000 - 4.0f * time_start;
			if (time_start >= 4250)
			{
				time_start = 0; // �������㣬��λ���̽���
				flag_idle = 3;	// ��λ������־λ���ػ��󲻿���ֱ�����¿�������Ҫ����������λ��ſ������¿���
			}
		}
		else if (flag_idle == 2) // ����̤��������˶�
		{
			EncoderTarget.M1 = Target_1A;
			EncoderTarget.M2 = Target_1B;
			EncoderTarget.M3 = Target_2A;
			EncoderTarget.M4 = Target_2B;

			EncoderTarget.M5 = Target_3A;
			EncoderTarget.M6 = Target_3B;
			EncoderTarget.M7 = Target_4A;
			EncoderTarget.M8 = Target_4B;
		}
	}
	else // �ϵ��δվ��ʱ����ʱ�����ס
	{
		EncoderTarget.M1 = 0;
		EncoderTarget.M2 = 0;
		EncoderTarget.M3 = 0;
		EncoderTarget.M4 = 0;

		EncoderTarget.M5 = 0;
		EncoderTarget.M6 = 0;
		EncoderTarget.M7 = 0;
		EncoderTarget.M8 = 0;
	}
}

void LegPositionInit(void)
{
	MotorInit.M1 = -13000; // ����һ������ı۵Ļ�׼ֵ��������17000��ֵ��ͬ
	MotorInit.M2 = 13000;  // ����Ϊվ�����˶�ʱ����Ļ���߶��ǲ�ͬ��
	MotorInit.M3 = -13000;
	MotorInit.M4 = 13000;

	MotorInit.M5 = -13000;
	MotorInit.M6 = 13000;
	MotorInit.M7 = -13000;
	MotorInit.M8 = 13000;
}

// void AttitudeControl(control_t *controlT,RelAttitude_t *RelAttitudeT,RelRate_t * RelRateT,TarAttitude_t *TarAttitudeT )
//{
//
//	control.pitch = pidUpdate(&pid[BODY_PITCH],TarAttitudeT->pitch - RelAttitudeT->pitch);
//
//	//control.pitch = pidCaulate(&pid[BODY_PITCH], TarAttitudeT->pitch - RelAttitudeT->pitch, RelRateT->gyrox );
//	control.roll = pidCaulate(&pid[BODY_ROLL], TarAttitudeT->roll - RelAttitudeT->roll, RelRateT->gyroy );
// }

//************************************************************************************************************************//
// �����㼣·���滮�㷨����ϸ�������㼣·���滮����ĵ�
//************************************************************************************************************************//
void FootTrajectoryLeg1(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg1X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (lam / 2 + 1 / 2) * Ts) + TarX;
		TarTragectoryOut.Leg1Z = TarZ;
	}
	else
	{
		TarTragectoryOut.Leg1X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2;
		TarTragectoryOut.Leg1Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
}
void FootTrajectoryLeg2(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg2X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2 - TarX;
		TarTragectoryOut.Leg2Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
	else
	{
		TarTragectoryOut.Leg2X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (1 - lam) / 2 * Ts);
		TarTragectoryOut.Leg2Z = TarZ;
	}
}
void FootTrajectoryLeg3(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg3X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (lam / 2 + 1 / 2) * Ts) + TarX;
		TarTragectoryOut.Leg3Z = TarZ;
	}
	else
	{
		TarTragectoryOut.Leg3X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2;
		TarTragectoryOut.Leg3Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
}
void FootTrajectoryLeg4(float TarX, float TarZ, int timeCount)
{
	float deta_t;
	deta_t = (2 * PI * timeCount * detaT) / (lam * Ts);

	if (timeCount * detaT > lam * Ts)
	{
		TarTragectoryOut.Leg4X = TarX * (deta_t - sinf(deta_t)) / (2 * PI) - TarX / 2 - TarX;
		TarTragectoryOut.Leg4Z = -TarZ * (1 - cosf(deta_t)) / 2 + TarZ;
	}
	else
	{
		TarTragectoryOut.Leg4X = -TarX / ((1 - lam) * Ts) * (timeCount * detaT - (1 - lam) / 2 * Ts);
		TarTragectoryOut.Leg4Z = TarZ;
	}
}
