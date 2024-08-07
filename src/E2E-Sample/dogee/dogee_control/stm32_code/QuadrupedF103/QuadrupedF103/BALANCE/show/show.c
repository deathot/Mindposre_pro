#include "show.h"

/**************************************************************************
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ)); // 此任务以50Hz的频率运行
		oled_show();									 // 显示屏打开
	}
}

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	//=============第1行显示z轴角速度===============//

	OLED_ShowString(0, 0, "GYRO-qZ:");
	if (gyro[2] < 0)
		OLED_ShowString(60, 0, "-"), OLED_ShowNumber(75, 0, -gyro[2], 5, 12);
	else
		OLED_ShowString(60, 0, "+"), OLED_ShowNumber(75, 0, gyro[2], 5, 12); // z轴陀螺仪零点漂移数据

	//=============第2行显示电机A的状态=======================//
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
	//			//=============第3行显示电机B的状态=======================//
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
	//			//=============第4行显示电机C的状态=======================//
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

	//			//=============麦轮车第5行显示电机D的状态=======================//
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

	//=============第六行显示内容=======================//
	// 四足控制模式
	if (control_mode == 8)
		OLED_ShowString(00, 50, "APP");
	else if (control_mode == 4)
		OLED_ShowString(00, 50, "PS2");
	else if (control_mode == 2)
		OLED_ShowString(00, 50, "R-C");
	else
		OLED_ShowString(00, 50, "ROS");

	// 四足状态
	switch (armingFlags)
	{
	case 0:
		OLED_ShowString(30, 50, "WAITING");
		break; // 上电初始态
	case 2:
		OLED_ShowString(30, 50, "LOCKED-");
		break; // 四足开机(收起)
	case 6:
		OLED_ShowString(30, 50, "READIED");
		break; // 预备运动态
	case 10:
		OLED_ShowString(30, 50, "RUNNING");
		break; // 踏步运动态
	case 18:
		OLED_ShowString(30, 50, "STOPPED");
		break; // 暂停运动
	case 34:
		OLED_ShowString(30, 50, "SITDWON");
		break; // 四足关机(收起)
	}

	OLED_ShowNumber(90, 50, RC_Velocity, 3, 12);
	//=============刷新屏幕=======================//
	OLED_Refresh_Gram();
}
