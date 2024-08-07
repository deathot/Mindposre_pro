#include "BEE.h"
/**************************************************************************
函数功能：控制蜂鸣器IO初始化
入口参数：无
返回  值：无
**************************************************************************/
int bee_count = 0; // 蜂鸣器响多少声
void Bee_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		   // GPIOA12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 高速100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   // 初始化
}

void bee_task(void *pvParameters)
{
	while (1)
	{
		if (Power_Voltage < 2000 && start_up_15_second == 1) // 开机15秒后开始检测，如果电池电压小于20V
		{
			bee_flash();	 // 蜂鸣器快速鸣叫
			vTaskDelay(200); // 相对延时函数
		}
		else
		{
			bee(bee_count, Remote_key_control_flag);
			vTaskDelay(600); // 相对延时函数
		}
	}
}
/**************************************************************************
函数功能：按键控制机器人启停时的蜂鸣器反馈
入口参数：count：蜂鸣器响多少声/2，flag：启停标志位
返回  值：无
**************************************************************************/
void bee(int count, u8 flag)
{
	if (flag == 0) // 上电完成后，蜂鸣器响1声反馈已经获取到角度零点
	{
		if (count > 0)
			bee_flash(), bee_count--; // 蜂鸣器响1声
	}

	else if (flag == 1) // 按键/航模遥控 启动机器人过程
	{
		if (count > 0)
			bee_flash(), bee_count--; // 蜂鸣器响3声
		else if (count == 0)
			Remote_KEY = 4, USART_KEY = 4, START_COUNT++, bee_off(); // 响完3声后机器人开始站立
		if (START_COUNT > 5)
			Remote_KEY = 15, USART_KEY = 15, bee_count--; // 站立完成后进入预备控制态
	}
	else if (flag == 2) // 按键/航模遥控 关停机器人过程
	{
		if (count > 0)
			bee_flash(), bee_count--; // 蜂鸣器响1声
		else if (count == 0)
			Remote_KEY = 16, USART_KEY = 16, bee_off(); // 机器人的脚收起
	}
}

void bee_flash(void)
{
	PA12 = ~PA12; // 蜂鸣器发出嘀嘀嘀的声音
}

void bee_on(void)
{
	PA12 = 1; // 蜂鸣器长鸣
}

void bee_off(void)
{
	PA12 = 0; // 蜂鸣器关停
}
