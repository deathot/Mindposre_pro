#include "show.h"

/**************************************************************************
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ)); // ��������50Hz��Ƶ������
		oled_show();									 // ��ʾ����
	}
}

/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
	//=============��1����ʾz����ٶ�===============//

	OLED_ShowString(0, 0, "GYRO-qZ:");
	if (gyro[2] < 0)
		OLED_ShowString(60, 0, "-"), OLED_ShowNumber(75, 0, -gyro[2], 5, 12);
	else
		OLED_ShowString(60, 0, "+"), OLED_ShowNumber(75, 0, gyro[2], 5, 12); // z�����������Ư������

	//=============��2����ʾ���A��״̬=======================//
	OLED_ShowString(0, 10, "A");
	if (MOTOR_A.Target < 0)
		OLED_ShowString(15, 10, "-"),
			OLED_ShowNumber(20, 10, -MOTOR_A.Target, 5, 12);
	else
		OLED_ShowString(15, 10, "+"),
			OLED_ShowNumber(20, 10, MOTOR_A.Target, 5, 12);

	if (MOTOR_A.Encoder < 0)
		OLED_ShowString(60, 10, "-"),
			OLED_ShowNumber(75, 10, -MOTOR_A.Encoder, 5, 12);
	else
		OLED_ShowString(60, 10, "+"),
			OLED_ShowNumber(75, 10, MOTOR_A.Encoder, 5, 12);
	//			//=============��3����ʾ���B��״̬=======================//
	OLED_ShowString(0, 20, "B");
	if (MOTOR_B.Target < 0)
		OLED_ShowString(15, 20, "-"),
			OLED_ShowNumber(20, 20, -MOTOR_B.Target, 5, 12);
	else
		OLED_ShowString(15, 20, "+"),
			OLED_ShowNumber(20, 20, MOTOR_B.Target, 5, 12);

	if (MOTOR_B.Encoder < 0)
		OLED_ShowString(60, 20, "-"),
			OLED_ShowNumber(75, 20, -MOTOR_B.Encoder, 5, 12);
	else
		OLED_ShowString(60, 20, "+"),
			OLED_ShowNumber(75, 20, MOTOR_B.Encoder, 5, 12);
	//			//=============��4����ʾ���C��״̬=======================//
	OLED_ShowString(0, 30, "C");
	if (MOTOR_C.Target < 0)
		OLED_ShowString(15, 30, "-"),
			OLED_ShowNumber(20, 30, -MOTOR_C.Target, 5, 12);
	else
		OLED_ShowString(15, 30, "+"),
			OLED_ShowNumber(20, 30, MOTOR_C.Target, 5, 12);

	if (MOTOR_C.Encoder < 0)
		OLED_ShowString(60, 30, "-"),
			OLED_ShowNumber(75, 30, -MOTOR_C.Encoder, 5, 12);
	else
		OLED_ShowString(60, 30, "+"),
			OLED_ShowNumber(75, 30, MOTOR_C.Encoder, 5, 12);

	//			//=============���ֳ���5����ʾ���D��״̬=======================//
	OLED_ShowString(0, 40, "D");
	if (MOTOR_D.Target < 0)
		OLED_ShowString(15, 40, "-"),
			OLED_ShowNumber(20, 40, -MOTOR_D.Target, 5, 12);
	else
		OLED_ShowString(15, 40, "+"),
			OLED_ShowNumber(20, 40, MOTOR_D.Target, 5, 12);

	if (MOTOR_D.Encoder < 0)
		OLED_ShowString(60, 40, "-"),
			OLED_ShowNumber(75, 40, -MOTOR_D.Encoder, 5, 12);
	else
		OLED_ShowString(60, 40, "+"),
			OLED_ShowNumber(75, 40, MOTOR_D.Encoder, 5, 12);

	//=============��������ʾ����=======================//
	// �������ģʽ
	if (control_mode == 8)
		OLED_ShowString(00, 50, "APP");
	else if (control_mode == 4)
		OLED_ShowString(00, 50, "PS2");
	else if (control_mode == 2)
		OLED_ShowString(00, 50, "R-C");
	else
		OLED_ShowString(00, 50, "ROS");

	// ����״̬
	switch (armingFlags)
	{
	case 0:
		OLED_ShowString(30, 50, "WAITING");
		break; // �ϵ��ʼ̬
	case 2:
		OLED_ShowString(30, 50, "LOCKED-");
		break; // ���㿪��(����)
	case 6:
		OLED_ShowString(30, 50, "READIED");
		break; // Ԥ���˶�̬
	case 10:
		OLED_ShowString(30, 50, "RUNNING");
		break; // ̤���˶�̬
	case 18:
		OLED_ShowString(30, 50, "STOPPED");
		break; // ��ͣ�˶�
	case 34:
		OLED_ShowString(30, 50, "SITDWON");
		break; // ����ػ�(����)
	}

	OLED_ShowNumber(90, 50, RC_Velocity, 3, 12);
	//=============ˢ����Ļ=======================//
	OLED_Refresh_Gram();
}
