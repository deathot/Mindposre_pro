#include "system.h"

/*相关标志位*/
uint8_t armingFlags = 0;													// 四足状态标志位
u8 APP_ON_Flag = 0, PS2_ON_Flag = 0, USART_ON_FLAG = 0, Remote_ON_Flag = 0; // 四足控制模式标志位
u8 APP_START_STOP_FLAG = 1;													// APP控制模式时用到的标志位，1是四足开机，0是四足关机
u8 start_up_15_second = 0;													// 四足上电开机15秒等待标志位
u8 Remote_key_control_flag = 0, START_COUNT = 0;							// 用户按键/航模遥控 控制四足开机和关机的标志位 /  用户按键/航模遥控 控制四足开机过程用到的标志位
u8 RUN_Control = 0;															// 预运动状态下用到的标志位
u8 Flag_Pose, Flag_Direction = 0, Turn_Flag, PID_Send = 0;					// 蓝牙遥控相关的标志位
u8 USART_KEY, APP_KEY, Remote_KEY;											// app、串口、航模遥控 控制模式下，模拟手柄的键值输入

/*底层硬件初始化*/
void systemInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 设置系统中断优先级分组4
	delay_init(168);								//=====初始化延时函数
	uart1_init(115200);								//=====串口1初始化与上位机通信
	usart2_init(9600);								//=====串口2初始化 蓝牙链接，与APP通信
	usart3_init(115200);							//=====串口3初始化与ROS通信
	LED_Init();										//=====初始化LED端口
	OLED_Init();									//=====显示屏初始化
	Adc_Init();										//=====采集电池电压引脚初始化
	KEY_Init();										//=====用户按键初始化

	IIC_Init();			  //=====IIC初始化
	MPU6050_initialize(); //=====MPU6050初始化
	DMP_Init();			  //=====初始化DMP

	Bee_Init();			 //=====蜂鸣器引脚初始化
	Encoder_Init_TIM2(); //=====初始化编码器A
	Encoder_Init_TIM3(); //=====初始化编码器B
	Encoder_Init_TIM4(); //=====初始化编码器C
	Encoder_Init_TIM5(); //=====初始化编码器D

	MiniBalance_PWM_Init(8399, 0); //=====初始化PWM 10KHZ，用于驱动电机
	MiniBalance_Motor_Init();	   //=====初始IO，用于驱动电机

	PS2_Init();	   //=====ps2驱动端口初始化
	PS2_SetInit(); //=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改

	CAN1_Mode_Init(1, 7, 6, 3, CAN_Mode_Normal); //=====CAN初始化   末位0=普通模式，1=回环模式

	TIM1_Cap_Init(0XFFFF, 72 - 1); //=====航模遥控初始化

	allPidInit();	   // pid 参数初始化
	LegPositionInit(); // 狗腿子位置初始化，回到上电初始化状态

	// 将四足状态标志位清0
	DISABLE_ARMING_FLAG(LOCKED);
	DISABLE_ARMING_FLAG(READY);
	DISABLE_ARMING_FLAG(RUNNING);
	DISABLE_ARMING_FLAG(STOP);
	DISABLE_ARMING_FLAG(SHUTDWON);
}
