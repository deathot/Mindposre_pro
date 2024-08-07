#include "system.h"

unsigned char temp_show;

void systemInit(void)
{
	JTAG_Set(JTAG_SWD_DISABLE); //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);		//=====打开SWD接口 可以利用主板的SWD接口调试

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 设置系统中断优先级分组4
	delay_init();									// 初始化延时函数
	LED_Init();										// 初始化与 LED 连接的硬件接口
	OLED_Init();									// OLED初始化
	KEY_Init();										// 按键初始化
	CAN1_Mode_Init(1, 2, 3, 6, 0);					//=====CAN初始化
	Encoder_Init_TIM2();							//=====编码器接口A初始化
	Encoder_Init_TIM3();							//=====编码器接口B初始化
	Encoder_Init_TIM4();							//=====编码器接口C初始化
	Encoder_Init_TIM5();							//=====编码器接口D初始化
	MiniBalance_Motor_Init();						// 初始化控制电机正反转引脚
	MiniBalance_PWM_Init(7199, 0);					// 初始化PWM 10KHZ，用于驱动电机
	IIC_Init();										// IIC初始化
	Flag_Mpu6050 = MPU6050_Init();					// MPU6050初始化
}
