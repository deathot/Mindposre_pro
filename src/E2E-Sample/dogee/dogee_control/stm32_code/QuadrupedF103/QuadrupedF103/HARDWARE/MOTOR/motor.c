#include "motor.h"

void MiniBalance_Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE); // 使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5; // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;								 // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								 // 50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);											 // 根据设定参数初始化GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12; // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					 // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 // 50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);								 // 根据设定参数初始化GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		  // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50M
	GPIO_Init(GPIOD, &GPIO_InitStructure);			  // 根据设定参数初始化GPIO

	GPIO_ResetBits(GPIOB, GPIO_Pin_4); // io输出0，防止电机乱转
	GPIO_ResetBits(GPIOB, GPIO_Pin_5); // io输出0，防止电机乱转
	GPIO_ResetBits(GPIOB, GPIO_Pin_0); // io输出0，防止电机乱转
	GPIO_ResetBits(GPIOB, GPIO_Pin_1); // io输出0，防止电机乱转

	GPIO_ResetBits(GPIOC, GPIO_Pin_4);	// io输出0，防止电机乱转
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);	// io输出0，防止电机乱转
	GPIO_ResetBits(GPIOC, GPIO_Pin_12); // io输出0，防止电机乱转
	GPIO_ResetBits(GPIOD, GPIO_Pin_2);	// io输出0，防止电机乱转
}
/**************************************************************************
函数功能：电机PWM引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // 使能GPIO外设时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;						// 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// 设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 1;				// 设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);				// 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // 选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;							  // 设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  // 输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH预装载使能
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH预装载使能
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH预装载使能
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH预装载使能

	TIM_ARRPreloadConfig(TIM8, ENABLE); // 使能TIMx在ARR上的预装载寄存器

	TIM_Cmd(TIM8, ENABLE); // 使能TIM8

	TIM_CtrlPWMOutputs(TIM8, ENABLE); // 高级定时器输出必须使能这句
}

/**************************************************************************
函数功能：使能开关的引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;			  // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		  // 上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  // 根据设定参数初始化GPIO
}
