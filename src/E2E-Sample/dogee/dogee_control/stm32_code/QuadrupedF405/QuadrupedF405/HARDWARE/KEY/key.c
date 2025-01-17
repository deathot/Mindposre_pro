#include "key.h"

u8 Flag_Next; // 进入模式标志位
/**************************************************************************
函数功能：按键初始化
入口参数：无
返回  值：无
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		   // GPIO.B12.14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	   // 输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 高速100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化
}
u8 key_state;

void key_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); // 100Hz运行频率

		if (start_up_15_second == 1 && Remote_key_control_flag == 0 && APP_ON_Flag == 0 && PS2_ON_Flag == 0 && Remote_ON_Flag == 0) // 上电初始时只检测长按(按键启停只用在串口控制模式上)
		{
			if ((Long_Press() == 1))						// 长按用户按键2秒后
				bee_count = 5, Remote_key_control_flag = 1; // 蜂鸣器响3声，机器人开机
		}
		else if (Remote_key_control_flag == 1 && APP_ON_Flag == 0 && PS2_ON_Flag == 0 && Remote_ON_Flag == 0) // 启动后只检测双击(按键启停只用在串口控制模式上)
		{
			if (click_N_Double(60) == 2)					// 双击用户按键后
				bee_count = 3, Remote_key_control_flag = 2; // 蜂鸣器响2声，机器人关机
		}
	}
}

/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击
**************************************************************************/
u8 click_N_Double(u8 time)
{
	static u8 flag_key, count_key, double_key;
	static u16 count_single, Forever_count;
	if (KEY == 0)
		Forever_count++; // 长按标志位未置1
	else
		Forever_count = 0;
	if (0 == KEY && 0 == flag_key)
		flag_key = 1;
	if (0 == count_key)
	{
		if (flag_key == 1)
		{
			double_key++;
			count_key = 1;
		}
		if (double_key == 2)
		{
			double_key = 0;
			count_single = 0;
			return 2; // 双击执行的指令
		}
	}
	if (1 == KEY)
		flag_key = 0, count_key = 0;

	if (1 == double_key)
	{
		count_single++;
		if (count_single > time && Forever_count < time)
		{
			double_key = 0;
			count_single = 0;
			return 1; // 单击执行的指令
		}
		if (Forever_count > time)
		{
			double_key = 0;
			count_single = 0;
		}
	}
	return 0;
}

/**************************************************************************
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击
**************************************************************************/
u8 click(void)
{
	static u8 flag_key = 1; // 按键按松开标志
	if (flag_key && KEY == 0)
	{
		flag_key = 0;
		return 1; // 按键按下
	}
	else if (1 == KEY)
		flag_key = 1;
	return 0; // 无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 1：长按2s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count, Long_Press;
	if (Long_Press == 0 && KEY == 0)
		Long_Press_count++; // 长按标志位未置1
	else
		Long_Press_count = 0;
	if (Long_Press_count > 150)
	{
		Long_Press = 1;
		Long_Press_count = 0;
		return 1;
	}
	if (Long_Press == 1) // 长按标志位置1
	{
		Long_Press = 0;
	}
	return 0;
}
