#include "timer.h"

int Remote_count = 0;	  // �������뺽ģң�ؼ���
u8 Remote_Check_flag = 0; // �ж��Ƿ���뺽ģң�صı�־λ

u32 Remoter_Ch1 = 3500, Remoter_Ch2 = 3500, Remoter_Ch3 = 3500, Remoter_Ch4 = 3500;			// ��ģң�زɼ���ر���
u32 L_Remoter_Ch1 = 3500, L_Remoter_Ch2 = 3500, L_Remoter_Ch3 = 3500, L_Remoter_Ch4 = 3500; // ��ģң�ؽ��ձ���

u8 TIM1CH1_CAPTURE_STA = 0; // ͨ��1���벶���־������λ�������־����6λ�������־
u16 TIM1CH1_CAPTURE_UPVAL;
u16 TIM1CH1_CAPTURE_DOWNVAL;

u8 TIM1CH2_CAPTURE_STA = 0; // ͨ��2���벶���־������λ�������־����6λ�������־
u16 TIM1CH2_CAPTURE_UPVAL;
u16 TIM1CH2_CAPTURE_DOWNVAL;

u8 TIM1CH3_CAPTURE_STA = 0; // ͨ��3���벶���־������λ�������־����6λ�������־
u16 TIM1CH3_CAPTURE_UPVAL;
u16 TIM1CH3_CAPTURE_DOWNVAL;

u8 TIM1CH4_CAPTURE_STA = 0; // ͨ��4���벶���־������λ�������־����6λ�������־
u16 TIM1CH4_CAPTURE_UPVAL;
u16 TIM1CH4_CAPTURE_DOWNVAL;

u32 TIM1_T1;
u32 TIM1_T2;
u32 TIM1_T3;
u32 TIM1_T4;
// ��ʱ��2���벶������
int pwmout1, pwmout2, pwmout3, pwmout4; // ���ռ�ձ�
/**************************************************************************
�������ܣ���ģң�س�ʼ������
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ��
����  ֵ����
**************************************************************************/
void TIM1_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // ʹ��TIM1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ʹ��GPIOA ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	   // ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

	// ��ʼ����ʱ�� TIM1
	TIM_TimeBaseStructure.TIM_Period = arr;						// �趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		// ����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);				// ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	// ��ʼ��TIM3���벶����� ͨ��1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;				// CC1S=01 	ѡ�������
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		// �����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			// ���������Ƶ,����Ƶ
	TIM_ICInitStructure.TIM_ICFilter = 0x00;						// IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// ��ʼ��TIM3���벶����� ͨ��2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; // CC1S=01 	ѡ�������
	//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
	//	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// ��ʼ��TIM3���벶����� ͨ��3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; // CC1S=01 	ѡ�������
	//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
	//	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// ��ʼ��TIM3���벶����� ͨ��4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; // CC1S=01 	ѡ�������
	//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //���������Ƶ,����Ƶ
	//	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	// �жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;		  // TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // �����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn; // TIM1�ж�
	//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure); // ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);
	TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE); // ��������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�
	// TIM_CtrlPWMOutputs(TIM1,ENABLE); //�߼���ʱ���������ʹ�����
	TIM_Cmd(TIM1, ENABLE); // ʹ�ܶ�ʱ��
}

/**************************************************************************
�������ܣ���ģң�ؽ����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM1_CC_IRQHandler(void)
{
	if ((TIM1CH1_CAPTURE_STA & 0X80) == 0) // ��δ�ɹ�����
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) // ����1���������¼�
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); // ����жϱ�־λ
			if (TIM1CH1_CAPTURE_STA & 0X40)			 // ����һ���½���
			{
				TIM1CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM1); // ��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM1CH1_CAPTURE_DOWNVAL < TIM1CH1_CAPTURE_UPVAL)
				{
					TIM1_T1 = 65535;
				}
				else
					TIM1_T1 = 0;
				Remoter_Ch1 = TIM1CH1_CAPTURE_DOWNVAL - TIM1CH1_CAPTURE_UPVAL + TIM1_T1; // �õ��ܵĸߵ�ƽ��ʱ��
				if (abs(Remoter_Ch1 - L_Remoter_Ch1) > 800)
					Remoter_Ch1 = L_Remoter_Ch1;
				L_Remoter_Ch1 = Remoter_Ch1;

				TIM1CH1_CAPTURE_STA = 0;							// �����־λ����
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); // ����Ϊ�����ز���
			}
			else // ��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM1CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM1);		 // ��ȡ����������
				TIM1CH1_CAPTURE_STA |= 0X40;						 // ����Ѳ���������
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling); // ����Ϊ�½��ز���
			}
		}
	}

	if ((TIM1CH2_CAPTURE_STA & 0X80) == 0) // ��δ�ɹ�����
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET) // ����2���������¼�
		{

			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2); // ����жϱ�־λ
			if (TIM1CH2_CAPTURE_STA & 0X40)			 // ����һ���½���
			{
				TIM1CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM1); // ��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM1CH2_CAPTURE_DOWNVAL < TIM1CH2_CAPTURE_UPVAL)
				{
					TIM1_T2 = 65535;
				}
				else
					TIM1_T2 = 0;
				Remoter_Ch2 = TIM1CH2_CAPTURE_DOWNVAL - TIM1CH2_CAPTURE_UPVAL + TIM1_T2; // �õ��ܵĸߵ�ƽ��ʱ��
				if (abs(Remoter_Ch2 - L_Remoter_Ch2) > 800)
					Remoter_Ch2 = L_Remoter_Ch2;
				L_Remoter_Ch2 = Remoter_Ch2;

				if (Remoter_Ch2 > 4200)
					Remote_Check_flag = 1; // ���ҡ����ǰ��

				TIM1CH2_CAPTURE_STA = 0;							// �����־λ����
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); // ����Ϊ�����ز���
			}
			else // ��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM1CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM1);		 // ��ȡ����������
				TIM1CH2_CAPTURE_STA |= 0X40;						 // ����Ѳ���������
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling); // ����Ϊ�½��ز���
			}
		}
	}

	if ((TIM1CH3_CAPTURE_STA & 0X80) == 0) // ��δ�ɹ�����
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET) // ����3���������¼�
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3); // ����жϱ�־λ
			if (TIM1CH3_CAPTURE_STA & 0X40)			 // ����һ���½���
			{
				TIM1CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM1); // ��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM1CH3_CAPTURE_DOWNVAL < TIM1CH3_CAPTURE_UPVAL)
				{
					TIM1_T3 = 65535;
				}
				else
					TIM1_T3 = 0;
				Remoter_Ch3 = TIM1CH3_CAPTURE_DOWNVAL - TIM1CH3_CAPTURE_UPVAL + TIM1_T3; // �õ��ܵĸߵ�ƽ��ʱ��
				if (abs(Remoter_Ch3 - L_Remoter_Ch3) > 800)
					Remoter_Ch3 = L_Remoter_Ch3;
				L_Remoter_Ch3 = Remoter_Ch3;
				TIM1CH3_CAPTURE_STA = 0;							// �����־λ����
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising); // ����Ϊ�����ز���
			}
			else // ��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM1CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM1);		 // ��ȡ����������
				TIM1CH3_CAPTURE_STA |= 0X40;						 // ����Ѳ���������
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling); // ����Ϊ�½��ز���
			}
		}
	}

	if ((TIM1CH4_CAPTURE_STA & 0X80) == 0) // ��δ�ɹ�����
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET) // ����4���������¼�
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC4); // ����жϱ�־λ
			if (TIM1CH4_CAPTURE_STA & 0X40)			 // ����һ���½���
			{
				TIM1CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM1); // ��¼�´�ʱ�Ķ�ʱ������ֵ
				if (TIM1CH4_CAPTURE_DOWNVAL < TIM1CH4_CAPTURE_UPVAL)
				{
					TIM1_T4 = 65535;
				}
				else
					TIM1_T4 = 0;
				Remoter_Ch4 = TIM1CH4_CAPTURE_DOWNVAL - TIM1CH4_CAPTURE_UPVAL + TIM1_T4; // �õ��ܵĸߵ�ƽ��ʱ��
				if (abs(Remoter_Ch4 - L_Remoter_Ch4) > 800)
					Remoter_Ch4 = L_Remoter_Ch4;
				L_Remoter_Ch4 = Remoter_Ch4;
				TIM1CH4_CAPTURE_STA = 0;							// �����־λ����
				TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); // ����Ϊ�����ز���
			}
			else // ��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
			{
				TIM1CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM1);		 // ��ȡ����������
				TIM1CH4_CAPTURE_STA |= 0X40;						 // ����Ѳ���������
				TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling); // ����Ϊ�½��ز���
			}
		}
	}
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

void Remote_Check_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ)); // 10Hz����Ƶ��

		if (Remote_ON_Flag == 0)
		{
			if (Remote_Check_flag == 1) // ���ҡ����ǰ�Ƶ�״̬
				Remote_count++;
			else
				Remote_count = 0; // �м�����ж���һ�����������

			if (Remote_count > 20)														 // ����2s�������ҡ����ǰ��
				APP_ON_Flag = 0, PS2_ON_Flag = 0, USART_ON_FLAG = 0, Remote_ON_Flag = 1; // ��ʽ���뺽ģң��ģʽ
		}

		Remote_Check_flag = 0; // ���㣬�����һ�ν�������ʱ����1˵���������ģ�����ж�
	}
}
