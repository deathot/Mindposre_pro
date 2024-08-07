#include "show.h"

unsigned char i, Flag_Show; // 计数变量
unsigned char Send_Count;	// 串口需要发送的数据个数
float Vol;
float Voltage_Count, Voltage_All;
int Voltage;	   // 电压采样相关变量
int Power_Voltage; // 电池电压，单位：毫伏
void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ)); // 50Hz运行频率
		Voltage_All += Get_battery_volt();				 // 多次采样累积
		if (++Voltage_Count == 100)
			Power_Voltage = Voltage_All / 100, Voltage_All = 0, Voltage_Count = 0; // 求平均值 获取电池电压
		Flag_Show = 0;
		if (Flag_Show == 0) // 使用MiniBalance APP和OLED显示屏
		{
			APP_Show();
			oled_show(); //===显示屏打开
		}
		//    	else                       //使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
		//  		{
		//				DataScope();             //开启MiniBalance上位机
		//			}
	}
}

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	//=============第1行显示3轴角度===============//
	OLED_ShowString(0, 0, "X:");
	if (Pitch < 0)
		OLED_ShowNumber(15, 0, Pitch + 360, 3, 12);
	else
		OLED_ShowNumber(15, 0, Pitch, 3, 12);

	OLED_ShowString(40, 0, "Y:");
	if (Roll < 0)
		OLED_ShowNumber(55, 0, Roll + 360, 3, 12);
	else
		OLED_ShowNumber(55, 0, Roll, 3, 12);

	OLED_ShowString(80, 0, "Z:");
	if (Yaw < 0)
		OLED_ShowNumber(95, 0, Yaw + 360, 3, 12);
	else
		OLED_ShowNumber(95, 0, Yaw, 3, 12);

	//		  OLED_ShowNumber(00,00, Remoter_Ch1,4,12);  //航模遥控通道数据
	//			OLED_ShowNumber(30,00, Remoter_Ch2,4,12);  //航模遥控通道数据
	//			OLED_ShowNumber(60,00, Remoter_Ch3,4,12);  //航模遥控通道数据
	//			OLED_ShowNumber(90,00, Remoter_Ch4,4,12);  //航模遥控通道数据

	//=============显示电机A的状态=======================//
	if (EncoderTarget.M1 < 0)
		OLED_ShowString(00, 10, "-"),
			OLED_ShowNumber(15, 10, -EncoderTarget.M1, 5, 12);
	else
		OLED_ShowString(0, 10, "+"),
			OLED_ShowNumber(15, 10, EncoderTarget.M1, 5, 12);

	if (EncoderState.M1 < 0)
		OLED_ShowString(80, 10, "-"),
			OLED_ShowNumber(95, 10, -EncoderState.M1, 5, 12);
	else
		OLED_ShowString(80, 10, "+"),
			OLED_ShowNumber(95, 10, EncoderState.M1, 5, 12);
	//=============显示电机B的状态=======================//
	if (EncoderTarget.M2 < 0)
		OLED_ShowString(00, 20, "-"),
			OLED_ShowNumber(15, 20, -EncoderTarget.M2, 5, 12);
	else
		OLED_ShowString(0, 20, "+"),
			OLED_ShowNumber(15, 20, EncoderTarget.M2, 5, 12);

	if (EncoderState.M2 < 0)
		OLED_ShowString(80, 20, "-"),
			OLED_ShowNumber(95, 20, -EncoderState.M2, 5, 12);
	else
		OLED_ShowString(80, 20, "+"),
			OLED_ShowNumber(95, 20, EncoderState.M2, 5, 12);
	//=============显示电机C的状态=======================//
	if (EncoderTarget.M3 < 0)
		OLED_ShowString(00, 30, "-"),
			OLED_ShowNumber(15, 30, -EncoderTarget.M3, 5, 12);
	else
		OLED_ShowString(0, 30, "+"),
			OLED_ShowNumber(15, 30, EncoderTarget.M3, 5, 12);

	if (EncoderState.M3 < 0)
		OLED_ShowString(80, 30, "-"),
			OLED_ShowNumber(95, 30, -EncoderState.M3, 5, 12);
	else
		OLED_ShowString(80, 30, "+"),
			OLED_ShowNumber(95, 30, EncoderState.M3, 5, 12);
	//=============显示电机D的状态=======================//
	if (EncoderTarget.M4 < 0)
		OLED_ShowString(00, 40, "-"),
			OLED_ShowNumber(15, 40, -EncoderTarget.M4, 5, 12);
	else
		OLED_ShowString(0, 40, "+"),
			OLED_ShowNumber(15, 40, EncoderTarget.M4, 5, 12);

	if (EncoderState.M4 < 0)
		OLED_ShowString(80, 40, "-"),
			OLED_ShowNumber(95, 40, -EncoderState.M4, 5, 12);
	else
		OLED_ShowString(80, 40, "+"),
			OLED_ShowNumber(95, 40, EncoderState.M4, 5, 12);
	//		//=============第五行显示状态=======================//
	OLED_ShowString(00, 50, "L");
	if (TarTargectoryLeftX > 0)
		OLED_ShowString(10, 50, "+"), OLED_ShowNumber(15, 50, TarTargectoryLeftX, 3, 12); //  TarTargectoryLeftX;
	else
		OLED_ShowString(10, 50, "-"), OLED_ShowNumber(15, 50, -TarTargectoryLeftX, 3, 12); //  TarTargectoryLeftX;

	OLED_ShowString(92, 50, "R");
	if (TarTargectoryRightX > 0)
		OLED_ShowString(102, 50, "+"), OLED_ShowNumber(107, 50, TarTargectoryRightX, 3, 12);
	else
		OLED_ShowString(102, 50, "-"), OLED_ShowNumber(107, 50, -TarTargectoryRightX, 3, 12);

	OLED_ShowString(56, 50, ".");
	OLED_ShowString(78, 50, "V");
	OLED_ShowNumber(43, 50, Power_Voltage / 100, 2, 12);
	OLED_ShowNumber(66, 50, Power_Voltage % 100, 2, 12);
	//=============刷新=======================//
	OLED_Refresh_Gram();
}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{

	static u8 flag;
	int app_2, app_3, app_4;
	app_4 = (Voltage - 1110) * 2 / 3;
	if (app_4 > 100)
		app_4 = 100; // 对电压数据进行处理

	app_2 = TarTargectoryLeftX;
	app_3 = TarTargectoryRightX;
	if (PID_Send == 1) // 发送PID参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)RC_Velocity, 0, 0, 0, 0, 0, 0, 0, 0); // 打印到APP上面
		PID_Send = 0;
	}
	if (flag == 0)													   //
		printf("{A%d:%d:%d:%d}$", (u8)app_2, (u8)app_3, (u8)app_4, 0); // 打印到APP上面
	else
		printf("{B%d:%d:%d:%d}$", (int)RelAttitude.pitch, (int)RelAttitude.roll, 0, 0); // 打印到APP上面 显示波形

	flag = !flag;
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DataScope(void)
{

	DataScope_Get_Channel_Data(LegTarPosition.Leg1Z, 1); // 显示目标值
	DataScope_Get_Channel_Data(LegTarPosition.Leg2Z, 2); // 显示实际值，便于PID参数调节
	DataScope_Get_Channel_Data(LegTarPosition.Leg3Z, 3);
	DataScope_Get_Channel_Data(LegTarPosition.Leg4Z, 4);

	Send_Count = DataScope_Data_Generate(4);
	for (i = 0; i < Send_Count; i++)
	{
		while ((USART1->SR & 0X40) == 0)
			;
		USART1->DR = DataScope_OutPut_Buffer[i];
	}
}
