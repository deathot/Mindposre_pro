#include "show.h"

unsigned char i, Flag_Show; // ��������
unsigned char Send_Count;	// ������Ҫ���͵����ݸ���
float Vol;
float Voltage_Count, Voltage_All;
int Voltage;	   // ��ѹ������ر���
int Power_Voltage; // ��ص�ѹ����λ������
void show_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ)); // 50Hz����Ƶ��
		Voltage_All += Get_battery_volt();				 // ��β����ۻ�
		if (++Voltage_Count == 100)
			Power_Voltage = Voltage_All / 100, Voltage_All = 0, Voltage_Count = 0; // ��ƽ��ֵ ��ȡ��ص�ѹ
		Flag_Show = 0;
		if (Flag_Show == 0) // ʹ��MiniBalance APP��OLED��ʾ��
		{
			APP_Show();
			oled_show(); //===��ʾ����
		}
		//    	else                       //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
		//  		{
		//				DataScope();             //����MiniBalance��λ��
		//			}
	}
}

/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
	//=============��1����ʾ3��Ƕ�===============//
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

	//		  OLED_ShowNumber(00,00, Remoter_Ch1,4,12);  //��ģң��ͨ������
	//			OLED_ShowNumber(30,00, Remoter_Ch2,4,12);  //��ģң��ͨ������
	//			OLED_ShowNumber(60,00, Remoter_Ch3,4,12);  //��ģң��ͨ������
	//			OLED_ShowNumber(90,00, Remoter_Ch4,4,12);  //��ģң��ͨ������

	//=============��ʾ���A��״̬=======================//
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
	//=============��ʾ���B��״̬=======================//
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
	//=============��ʾ���C��״̬=======================//
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
	//=============��ʾ���D��״̬=======================//
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
	//		//=============��������ʾ״̬=======================//
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
	//=============ˢ��=======================//
	OLED_Refresh_Gram();
}
/**************************************************************************
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{

	static u8 flag;
	int app_2, app_3, app_4;
	app_4 = (Voltage - 1110) * 2 / 3;
	if (app_4 > 100)
		app_4 = 100; // �Ե�ѹ���ݽ��д���

	app_2 = TarTargectoryLeftX;
	app_3 = TarTargectoryRightX;
	if (PID_Send == 1) // ����PID����
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$", (int)RC_Velocity, 0, 0, 0, 0, 0, 0, 0, 0); // ��ӡ��APP����
		PID_Send = 0;
	}
	if (flag == 0)													   //
		printf("{A%d:%d:%d:%d}$", (u8)app_2, (u8)app_3, (u8)app_4, 0); // ��ӡ��APP����
	else
		printf("{B%d:%d:%d:%d}$", (int)RelAttitude.pitch, (int)RelAttitude.roll, 0, 0); // ��ӡ��APP���� ��ʾ����

	flag = !flag;
}
/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void DataScope(void)
{

	DataScope_Get_Channel_Data(LegTarPosition.Leg1Z, 1); // ��ʾĿ��ֵ
	DataScope_Get_Channel_Data(LegTarPosition.Leg2Z, 2); // ��ʾʵ��ֵ������PID��������
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
