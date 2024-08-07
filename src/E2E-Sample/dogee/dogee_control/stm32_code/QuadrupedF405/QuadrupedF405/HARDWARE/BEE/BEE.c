#include "BEE.h"
/**************************************************************************
�������ܣ����Ʒ�����IO��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
int bee_count = 0; // �������������
void Bee_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;		   // GPIOA12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // ����100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   // ��ʼ��
}

void bee_task(void *pvParameters)
{
	while (1)
	{
		if (Power_Voltage < 2000 && start_up_15_second == 1) // ����15���ʼ��⣬�����ص�ѹС��20V
		{
			bee_flash();	 // ��������������
			vTaskDelay(200); // �����ʱ����
		}
		else
		{
			bee(bee_count, Remote_key_control_flag);
			vTaskDelay(600); // �����ʱ����
		}
	}
}
/**************************************************************************
�������ܣ��������ƻ�������ͣʱ�ķ���������
��ڲ�����count���������������/2��flag����ͣ��־λ
����  ֵ����
**************************************************************************/
void bee(int count, u8 flag)
{
	if (flag == 0) // �ϵ���ɺ󣬷�������1�������Ѿ���ȡ���Ƕ����
	{
		if (count > 0)
			bee_flash(), bee_count--; // ��������1��
	}

	else if (flag == 1) // ����/��ģң�� ���������˹���
	{
		if (count > 0)
			bee_flash(), bee_count--; // ��������3��
		else if (count == 0)
			Remote_KEY = 4, USART_KEY = 4, START_COUNT++, bee_off(); // ����3��������˿�ʼվ��
		if (START_COUNT > 5)
			Remote_KEY = 15, USART_KEY = 15, bee_count--; // վ����ɺ����Ԥ������̬
	}
	else if (flag == 2) // ����/��ģң�� ��ͣ�����˹���
	{
		if (count > 0)
			bee_flash(), bee_count--; // ��������1��
		else if (count == 0)
			Remote_KEY = 16, USART_KEY = 16, bee_off(); // �����˵Ľ�����
	}
}

void bee_flash(void)
{
	PA12 = ~PA12; // ���������������ֵ�����
}

void bee_on(void)
{
	PA12 = 1; // ����������
}

void bee_off(void)
{
	PA12 = 0; // ��������ͣ
}
