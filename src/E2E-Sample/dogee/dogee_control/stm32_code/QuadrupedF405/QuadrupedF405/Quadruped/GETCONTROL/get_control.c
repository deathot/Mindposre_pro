#include "get_control.h"
int move_x, move_y, move_z, pose_x, pose_y; // 四足姿态控制和运动控制变量
float smooth_pose_x, smooth_pose_y;			// 姿态控制中平滑处理后的变量
short RC_Velocity = 200;					// 速度

void getcontrol_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); // 100Hz运行频率

		if (APP_ON_Flag)
			Get_RC(); // APP遥控
		else if (PS2_ON_Flag)
			PS2_control(); // PS2手柄控制
		else if (Remote_ON_Flag)
			Remote_control(); // 航模遥控控制
		else
			usart_control(); // 串口3（ROS）控制

		control_transition(100);	 // 运动状态切换时的过渡函数
		Smooth_pose(pose_x, pose_y); // 姿态控制的平滑处理

		RC_Velocity = constrain(RC_Velocity, 50, 250); // 运动速度限幅
		move_x = move_x * 0.01 * RC_Velocity;
		move_z = move_z * 0.01 * RC_Velocity;
		if (move_x < 0)
			move_z = -move_z;
	}
}

/**************************************************************************
函数功能：通过蓝牙接受数据指令对机器人进行遥控
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move = 1;
	UpdataFlags(APP_KEY);	// 更新解锁标志，(APP模拟手柄键位)
	switch (Flag_Direction) // 方向控制
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
函数功能：通过PS2有线手柄对机器人进行遥控
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
	static int last_lx = 0, last_ly = 0, last_rx = 0, last_ry = 0;

	UpdataFlags(PS2_KEY); // 更新解锁标志，(手柄)

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
函数功能：通过航模遥控对机器人进行遥控
入口参数：无
返回  值：无
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

	LY = Remoter_Ch2 - 3500; // 前进后退
	LX = Remoter_Ch4 - 3500; // 拐弯
	RX = Remoter_Ch1 - 3500; // 控制四足开和关
	//				RC=Remoter_Ch3-3500;//油门

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
			count1++; // 航模遥控右边摇杆往最左边打3秒
	}

	if (Remote_flag == 1)
	{
		if ((count2 < 300) && (RX > 800))
			count2++; // 航模遥控右边摇杆往最右边打3秒
	}

	if (count1 >= 300 && RX == 0 && open == 1)
		Remote_flag = 1, bee_count = 5, Remote_key_control_flag = 1, open = 0; // 蜂鸣器响3声，机器人开机 ;
	if (count2 >= 300 && RX == 0 && close == 1)
		Remote_flag = 2, bee_count = 3, Remote_key_control_flag = 2, close = 0; // 蜂鸣器响2声，机器人关机 ;

	UpdataFlags(Remote_KEY); // 更新解锁标志(航模遥控)
}
/**************************************************************************
函数功能：通过串口接收的指令对机器人进行控制
入口参数：无
返回  值：无
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
	UpdataFlags(USART_KEY); // 更新解锁标志，(用户按键模拟手柄键位)
							// 串口控制量都在串口中断函数中接收和赋值完成
}

/**************************************************************************
函数功能：静止态和运动态之间切换时增加一个踏步过渡
入口参数：踏步的时间，100等于实际时间的1秒
返回  值：无
**************************************************************************/
/*
从无控制量到有控制量的状态切换：       静止      》》  踏步（1秒）  》》  开始运动
从有控制量到无控制量的状态切换： 控制量从有到无  》》  踏步（1秒）  》》  静止
在中间增加一个踏步过渡的意义，是为了状态切换过程保持四足的机身更稳定
*/
void control_transition(int time)
{
	static u8 change = 0, stop = 0;
	static int count = 0, last_control = 0;

	if ((move_x != 0) || (move_z != 0))
		change = 1;
	else if ((last_control == 1) && (change == 2))
		stop = 1; // 如果上一次有控制量这一次没有，则进入收步
	else
		RUN_Control = 0, change = 0, count = 0;

	if (change == 1)
	{
		RUN_Control = 1;
		if (count <= time)
			count++;
	} // 这里数100次是1秒，在接收到控制量后先踏步1秒，再进行运动
	if (stop == 1)
		RUN_Control = 1, count--, move_x = 1; // 进入收步，踏步1秒后再静止

	if (count > time)
		RUN_Control = 2, change = 2; // 1秒踏步结束，开始正式运动
	if (count < 0)
		RUN_Control = 0, change = 0, stop = 0; // 1秒踏步结束，切换回静止态

	if ((move_x != 0) || (move_z != 0))
		last_control = 1; // 记录这一次是否有控制量，用于判断运动结束
	else
		last_control = 0;
}

/**************************************************************************
函数功能：四足姿态控制的平滑处理
入口参数：四足的目标姿态
返回  值：无
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
