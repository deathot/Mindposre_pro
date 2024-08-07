#include "motor.h"
long int Motor_A, Motor_B, Motor_C, Motor_D; // ���ؿ����ĸ������PWM����

/**************************************************************************
�������ܣ����Ƶ��ת�ٵ�PWM���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ʹ��GPIO����ʱ��ʹ��

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; // GPIOC6.7.8.9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									 // �������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									 // ���츴��
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;								 // �ٶ�100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;									 // ����
	GPIO_Init(GPIOC, &GPIO_InitStructure);											 // ��ʼ��

	TIM_TimeBaseStructure.TIM_Period = arr;						// ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// ����ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);				// ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
	//	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // �������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);				  // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);				  // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);				  // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);				  // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_CtrlPWMOutputs(TIM8, ENABLE); // MOE �����ʹ��

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH1Ԥװ��ʹ��
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH1Ԥװ��ʹ��
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH1Ԥװ��ʹ��
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); // CH4Ԥװ��ʹ��

	TIM_ARRPreloadConfig(TIM8, ENABLE); // ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(TIM8, ENABLE); // ʹ��TIM
}

/**************************************************************************
�������ܣ����Ƶ������ת���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE); // ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5; // GPIOB0.1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;									 // ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									 // ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;								 // ����100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;									 // ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);											 // ��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12; // GPIOC4.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;						 // ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						 // ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					 // ����100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						 // ����
	GPIO_Init(GPIOC, &GPIO_InitStructure);								 // ��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	;												   // GPIOC4.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // ����100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOD, &GPIO_InitStructure);			   // ��ʼ��

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
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
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
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
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
�������ܣ�PIDλ�ÿ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void PowerOutputControl(void)
{
	Motor_A = pidUpdate(&pid[MOTOR_A], EncoderState.M1 - EncoderTarget.M1);
	Motor_B = pidUpdate(&pid[MOTOR_B], EncoderState.M2 - EncoderTarget.M2);
	Motor_C = -pidUpdate(&pid[MOTOR_C], EncoderState.M3 - EncoderTarget.M3);
	Motor_D = -pidUpdate(&pid[MOTOR_D], EncoderState.M4 - EncoderTarget.M4);

	Set_Pwm(Motor_B, Motor_A, Motor_C, Motor_D);
}
