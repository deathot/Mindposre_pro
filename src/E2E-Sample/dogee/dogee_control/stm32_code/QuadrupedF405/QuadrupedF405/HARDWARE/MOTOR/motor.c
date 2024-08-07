#include "motor.h"
long int Motor_A, Motor_B, Motor_C, Motor_D; // 主控控制四个电机的PWM变量

/**************************************************************************
函数功能：控制电机转速的PWM引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 使能GPIO外设时钟使能

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; // GPIOC6.7.8.9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									 // 复用输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									 // 推挽复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;								 // 速度100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;									 // 上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);											 // 初始化

	TIM_TimeBaseStructure.TIM_Period = arr;						// 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// 设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// 设置时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);				// 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // 选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	//	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);				  // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);				  // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);				  // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);				  // 根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_CtrlPWMOutputs(TIM8, ENABLE); // MOE 主输出使能

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH1预装载使能
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH1预装载使能
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH1预装载使能
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH4预装载使能

	TIM_ARRPreloadConfig(TIM8, ENABLE); // 使能TIMx在ARR上的预装载寄存器

	TIM_Cmd(TIM8, ENABLE); // 使能TIM
}

/**************************************************************************
函数功能：控制电机正反转引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE); // 使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5; // GPIOB0.1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;									 // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									 // 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;								 // 高速100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;									 // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);											 // 初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12; // GPIOC4.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;						 // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						 // 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					 // 高速100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						 // 上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);								 // 初始化

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	;												   // GPIOC4.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 高速100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);			   // 初始化

	INA1 = 0;
	INB1 = 0;
	INC1 = 0;
	IND1 = 0;
	INA2 = 0;
	INB2 = 0;
	INC2 = 0;
	IND2 = 0;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
	if (motor_a < 0)
		INA2 = 1, INA1 = 0;
	else
		INA2 = 0, INA1 = 1;
	PWMA = myabs(motor_a);

	if (motor_b < 0)
		INB2 = 1, INB1 = 0;
	else
		INB2 = 0, INB1 = 1;
	PWMB = myabs(motor_b);

	if (motor_c > 0)
		INC2 = 1, INC1 = 0;
	else
		INC2 = 0, INC1 = 1;
	PWMC = myabs(motor_c);

	if (motor_d > 0)
		IND2 = 1, IND1 = 0;
	else
		IND2 = 0, IND1 = 1;
	PWMD = myabs(motor_d);
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{
	u32 temp;
	if (a < 0)
		temp = -a;
	else
		temp = a;
	return temp;
}

/**************************************************************************
函数功能：PID位置控制器
入口参数：无
返回  值：无
**************************************************************************/
void PowerOutputControl(void)
{
	Motor_A = pidUpdate(&pid[MOTOR_A], EncoderState.M1 - EncoderTarget.M1);
	Motor_B = pidUpdate(&pid[MOTOR_B], EncoderState.M2 - EncoderTarget.M2);
	Motor_C = -pidUpdate(&pid[MOTOR_C], EncoderState.M3 - EncoderTarget.M3);
	Motor_D = -pidUpdate(&pid[MOTOR_D], EncoderState.M4 - EncoderTarget.M4);

	Set_Pwm(Motor_B, Motor_A, Motor_C, Motor_D);
}
