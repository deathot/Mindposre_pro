#include "usartx.h"
SEND_DATA Send_Data;	   // �������ݵ�
RECEIVE_DATA Receive_Data; // �������ݵ�
extern int Time_count;
/**************************************************************************
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while ((USART2->SR & 0x40) == 0)
		;
}
/**************************************************************************
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // ʹ��USARTʱ��
														   // USART_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // �����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	  // PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
	// USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										// ���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);										// ��ʼ������2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);									// �������ڽ����ж�
	USART_Cmd(USART2, ENABLE);														// ʹ�ܴ���2
}
/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
// int USART2_IRQHandler(void)
//{
//	int Usart_Receive;
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
//   return 0;
// }
/**************************************************************************
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while ((USART3->SR & 0x40) == 0)
		;
}
/**************************************************************************
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while ((USART1->SR & 0x40) == 0)
		;
}
/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  // ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // ʹ��USART3ʱ��
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); // ������ӳ��
	// USART_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // C10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // �����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	  // PC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
	// USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										// ���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
	USART_Init(USART3, &USART_InitStructure);										// ��ʼ������3
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);									// �������ڽ����ж�
	USART_Cmd(USART3, ENABLE);														// ʹ�ܴ���3
}
/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
// int USART3_IRQHandler(void)
//{

//	u8 Usart_Receive;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�ж��Ƿ���յ�����

//  return 0;
//}
/**************************************************************************
 *  �������ܣ�����1��ʼ��
 *
 *  ��ڲ�������
 *
 *  �� �� ֵ����
 **************************************************************************/
void uart1_init(u32 bound)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // ʹ��USARTʱ��

	// USART_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // �����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// USART_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	  // PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
	// USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										// ���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								// ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// �շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);										// ��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									// �������ڽ����ж�
	USART_Cmd(USART1, ENABLE);														// ʹ�ܴ���1
}
/**************************************************************************
 *  �������ܣ�����1�����ж�
 *
 *  ��ڲ�������
 *
 *  �� �� ֵ����
 **************************************************************************/
// int USART1_IRQHandler(void)
//{
//	u8 Usart_Receive;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //���յ�����
//		Usart_Receive = USART_ReceiveData(USART1);//��ȡ����

//		return 0;
//}

/**************************************************************************
�������ܣ����㷢�͵�����У��λ
��ڲ�����
����  ֵ������λ
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode)
{
	unsigned char check_sum = 0, k;
	// �������ݵ�У��
	if (Mode == 1)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Send_Data.buffer[k];
		}
	// �������ݵ�У��
	if (Mode == 0)
		for (k = 0; k < Count_Number; k++)
		{
			check_sum = check_sum ^ Receive_Data.buffer[k];
		}
	return check_sum;
}
