#include "get_control.h"
int move_x, move_y, move_z, pose_x, pose_y; // ������̬���ƺ��˶����Ʊ���
float smooth_pose_x, smooth_pose_y;			// ��̬������ƽ�������ı���
short RC_Velocity = 200;					// �ٶ�

void getcontrol_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); // 100Hz����Ƶ��

		if (APP_ON_Flag)
			Get_RC(); // APPң��
		else if (PS2_ON_Flag)
			PS2_control(); // PS2�ֱ�����
		else if (Remote_ON_Flag)
			Remote_control(); // ��ģң�ؿ���
		else
			usart_control(); // ����3��ROS������

		control_transition(100);	 // �˶�״̬�л�ʱ�Ĺ��ɺ���
		Smooth_pose(pose_x, pose_y); // ��̬���Ƶ�ƽ������

		RC_Velocity = constrain(RC_Velocity, 50, 250); // �˶��ٶ��޷�
		move_x = move_x * 0.01 * RC_Velocity;
		move_z = move_z * 0.01 * RC_Velocity;
		if (move_x < 0)
			move_z = -move_z;
	}
}

/**************************************************************************
�������ܣ�ͨ��������������ָ��Ի����˽���ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move = 1;
	UpdataFlags(APP_KEY);	// ���½�����־��(APPģ���ֱ���λ)
	switch (Flag_Direction) // �������
	{
	case 1:
		move_x = 45;
		move_z = 0;
		Flag_Move = 1;
		break;
	case 2:
		move_x = 35;
		move_z = (-17);
		Flag_Move = 1;
		break;
	case 3:
		move_x = 0;
		move_z = (-22);
		Flag_Move = 1;
		break;
	case 4:
		move_x = (-35);
		move_z = (-17);
		Flag_Move = 1;
		break;
	case 5:
		move_x = (-45);
		move_z = 0;
		Flag_Move = 1;
		break;
	case 6:
		move_x = (-35);
		move_z = 17;
		Flag_Move = 1;
		break;
	case 7:
		move_x = 0;
		move_z = 22;
		Flag_Move = 1;
		break;
	case 8:
		move_x = 35;
		move_z = 17;
		Flag_Move = 1;
		break;
	default:
		move_x = 0;
		move_z = 0;
		Flag_Move = 0;
		break;
	}
	if (Flag_Move == 0)
	{
		if (Flag_Pose == 1)
			pose_x = -50, pose_y = 0;
		else if (Flag_Pose == 3)
			pose_x = 0, pose_y = 36;
		else if (Flag_Pose == 5)
			pose_x = 50, pose_y = 0;
		else if (Flag_Pose == 7)
			pose_x = 0, pose_y = -36;
		else
			pose_x = 0, pose_y = 0;
	}
}

/**************************************************************************
�������ܣ�ͨ��PS2�����ֱ��Ի����˽���ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
	static int last_lx = 0, last_ly = 0, last_rx = 0, last_ry = 0;

	UpdataFlags(PS2_KEY); // ���½�����־��(�ֱ�)

	if ((abs(last_lx - PS2_LX) > 60) || (abs(last_ly - PS2_LY) > 60) || (abs(last_rx - PS2_RX) > 60) || (abs(last_ry - PS2_RY) > 60))
	{
		move_x = -(last_ry - 128) / 3;
		move_z = -(last_rx - 128) / 5;

		pose_x = (last_lx - 128) / 3;
		pose_y = (last_ly - 128) / 5;
	}
	else
	{
		move_x = -(PS2_RY - 128) / 3;
		move_z = -(PS2_RX - 128) / 5;

		pose_x = (PS2_LY - 128) / 3;
		pose_y = (PS2_LX - 128) / 5;
	}

	last_lx = PS2_LX, last_ly = PS2_LY, last_rx = PS2_RX, last_ry = PS2_RY;
}

/**************************************************************************
�������ܣ�ͨ����ģң�ضԻ����˽���ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_control(void)
{
	static u8 Remote_flag = 0, open = 1, close = 1;
	static int count1 = 0, count2 = 0;
	int Yuzhi = 200;
	int LX, LY, RX;

	Remoter_Ch1 = constrain(Remoter_Ch1, 2200, 5000);
	Remoter_Ch2 = constrain(Remoter_Ch2, 2200, 5000);
	Remoter_Ch4 = constrain(Remoter_Ch4, 2200, 5000);

	LY = Remoter_Ch2 - 3500; // ǰ������
	LX = Remoter_Ch4 - 3500; // ����
	RX = Remoter_Ch1 - 3500; // �������㿪�͹�
	//				RC=Remoter_Ch3-3500;//����

	if (LX > -Yuzhi && LX < Yuzhi)
		LX = 0;
	if (LY > -Yuzhi && LY < Yuzhi)
		LY = 0;
	if (RX > -Yuzhi && RX < Yuzhi)
		RX = 0;

	move_x = (int)LY * 0.09;
	move_z = (int)-LX * 0.1;

	if (Remote_flag == 0)
	{
		if ((count1 < 300) && (RX < -800))
			count1++; // ��ģң���ұ�ҡ��������ߴ�3��
	}

	if (Remote_flag == 1)
	{
		if ((count2 < 300) && (RX > 800))
			count2++; // ��ģң���ұ�ҡ�������ұߴ�3��
	}

	if (count1 >= 300 && RX == 0 && open == 1)
		Remote_flag = 1, bee_count = 5, Remote_key_control_flag = 1, open = 0; // ��������3���������˿��� ;
	if (count2 >= 300 && RX == 0 && close == 1)
		Remote_flag = 2, bee_count = 3, Remote_key_control_flag = 2, close = 0; // ��������2���������˹ػ� ;

	UpdataFlags(Remote_KEY); // ���½�����־(��ģң��)
}
/**************************************************************************
�������ܣ�ͨ�����ڽ��յ�ָ��Ի����˽��п���
��ڲ�������
����  ֵ����
**************************************************************************/
void usart_control(void)
{ // u8 Move_flag = 1;
	if (USART_KEY == 0x04)
	{
		USART_KEY = 15;
	}
	if (start_up_15_second == 1 && USART_KEY == 0)
	{
		USART_KEY = 0x04;
	}
	UpdataFlags(USART_KEY); // ���½�����־��(�û�����ģ���ֱ���λ)
							// ���ڿ��������ڴ����жϺ����н��պ͸�ֵ���
}

/**************************************************************************
�������ܣ���ֹ̬���˶�̬֮���л�ʱ����һ��̤������
��ڲ�����̤����ʱ�䣬100����ʵ��ʱ���1��
����  ֵ����
**************************************************************************/
/*
���޿��������п�������״̬�л���       ��ֹ      ����  ̤����1�룩  ����  ��ʼ�˶�
���п��������޿�������״̬�л��� ���������е���  ����  ̤����1�룩  ����  ��ֹ
���м�����һ��̤�����ɵ����壬��Ϊ��״̬�л����̱�������Ļ�����ȶ�
*/
void control_transition(int time)
{
	static u8 change = 0, stop = 0;
	static int count = 0, last_control = 0;

	if ((move_x != 0) || (move_z != 0))
		change = 1;
	else if ((last_control == 1) && (change == 2))
		stop = 1; // �����һ���п�������һ��û�У�������ղ�
	else
		RUN_Control = 0, change = 0, count = 0;

	if (change == 1)
	{
		RUN_Control = 1;
		if (count <= time)
			count++;
	} // ������100����1�룬�ڽ��յ�����������̤��1�룬�ٽ����˶�
	if (stop == 1)
		RUN_Control = 1, count--, move_x = 1; // �����ղ���̤��1����پ�ֹ

	if (count > time)
		RUN_Control = 2, change = 2; // 1��̤����������ʼ��ʽ�˶�
	if (count < 0)
		RUN_Control = 0, change = 0, stop = 0; // 1��̤���������л��ؾ�ֹ̬

	if ((move_x != 0) || (move_z != 0))
		last_control = 1; // ��¼��һ���Ƿ��п������������ж��˶�����
	else
		last_control = 0;
}

/**************************************************************************
�������ܣ�������̬���Ƶ�ƽ������
��ڲ����������Ŀ����̬
����  ֵ����
**************************************************************************/
void Smooth_pose(float pose_x, float pose_y)
{
	float step = 0.4;

	if (pose_x > 0)
		smooth_pose_x += step;
	else if (pose_x < 0)
		smooth_pose_x -= step;
	else if (pose_x == 0)
		smooth_pose_x = smooth_pose_x * 0.95f;

	if (pose_y > 0)
		smooth_pose_y += step;
	else if (pose_y < 0)
		smooth_pose_y -= step;
	else if (pose_y == 0)
		smooth_pose_y = smooth_pose_y * 0.95f;

	if (pose_x != 0)
		smooth_pose_x = target_limit_float(smooth_pose_x, -float_abs(pose_x), float_abs(pose_x));
	if (pose_y != 0)
		smooth_pose_y = target_limit_float(smooth_pose_y, -float_abs(pose_y), float_abs(pose_y));
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
