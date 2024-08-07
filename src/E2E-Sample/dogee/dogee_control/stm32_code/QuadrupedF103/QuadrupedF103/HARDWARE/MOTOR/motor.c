#include "motor.h"

void MiniBalance_Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE); // ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5; // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;								 // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;								 // 50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);											 // �����趨������ʼ��GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12; // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;					 // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					 // 50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);								 // �����趨������ʼ��GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		  // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50M
	GPIO_Init(GPIOD, &GPIO_InitStructure);			  // �����趨������ʼ��GPIO

	GPIO_ResetBits(GPIOB, GPIO_Pin_4); // io���0����ֹ�����ת
	GPIO_ResetBits(GPIOB, GPIO_Pin_5); // io���0����ֹ�����ת
	GPIO_ResetBits(GPIOB, GPIO_Pin_0); // io���0����ֹ�����ת
	GPIO_ResetBits(GPIOB, GPIO_Pin_1); // io���0����ֹ�����ת

	GPIO_ResetBits(GPIOC, GPIO_Pin_4);	// io���0����ֹ�����ת
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);	// io���0����ֹ�����ת
	GPIO_ResetBits(GPIOC, GPIO_Pin_12); // io���0����ֹ�����ת
	GPIO_ResetBits(GPIOD, GPIO_Pin_2);	// io���0����ֹ�����ת
}
/**************************************************************************
�������ܣ����PWM���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // ʹ��GPIO����ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;						// ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 1;				// ����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);				// ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  // ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // �Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;							  // ���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  // �������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure); // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure); // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure); // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); // ����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); // CHԤװ��ʹ��
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); // CHԤװ��ʹ��
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); // CHԤװ��ʹ��
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); // CHԤװ��ʹ��

	TIM_ARRPreloadConfig(TIM8, ENABLE); // ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(TIM8, ENABLE); // ʹ��TIM8

	TIM_CtrlPWMOutputs(TIM8, ENABLE); // �߼���ʱ���������ʹ�����
}

/**************************************************************************
�������ܣ�ʹ�ܿ��ص����ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ʹ�ܶ˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;			  // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		  // ��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  // �����趨������ʼ��GPIO
}
