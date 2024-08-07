#include "key.h"

u8 Flag_Next; // ����ģʽ��־λ
/**************************************************************************
�������ܣ�������ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;		   // GPIO.B12.14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	   // ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // ����100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // ��ʼ��
}
u8 key_state;

void key_task(void *pvParameters)
{
	u32 lastWakeTime = getSysTickCnt();
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); // 100Hz����Ƶ��

		if (start_up_15_second == 1 && Remote_key_control_flag == 0 && APP_ON_Flag == 0 && PS2_ON_Flag == 0 && Remote_ON_Flag == 0) // �ϵ��ʼʱֻ��ⳤ��(������ֻͣ���ڴ��ڿ���ģʽ��)
		{
			if ((Long_Press() == 1))						// �����û�����2���
				bee_count = 5, Remote_key_control_flag = 1; // ��������3���������˿���
		}
		else if (Remote_key_control_flag == 1 && APP_ON_Flag == 0 && PS2_ON_Flag == 0 && Remote_ON_Flag == 0) // ������ֻ���˫��(������ֻͣ���ڴ��ڿ���ģʽ��)
		{
			if (click_N_Double(60) == 2)					// ˫���û�������
				bee_count = 3, Remote_key_control_flag = 2; // ��������2���������˹ػ�
		}
	}
}

/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬ 0���޶��� 1������ 2��˫��
**************************************************************************/
u8 click_N_Double(u8 time)
{
	static u8 flag_key, count_key, double_key;
	static u16 count_single, Forever_count;
	if (KEY == 0)
		Forever_count++; // ������־λδ��1
	else
		Forever_count = 0;
	if (0 == KEY && 0 == flag_key)
		flag_key = 1;
	if (0 == count_key)
	{
		if (flag_key == 1)
		{
			double_key++;
			count_key = 1;
		}
		if (double_key == 2)
		{
			double_key = 0;
			count_single = 0;
			return 2; // ˫��ִ�е�ָ��
		}
	}
	if (1 == KEY)
		flag_key = 0, count_key = 0;

	if (1 == double_key)
	{
		count_single++;
		if (count_single > time && Forever_count < time)
		{
			double_key = 0;
			count_single = 0;
			return 1; // ����ִ�е�ָ��
		}
		if (Forever_count > time)
		{
			double_key = 0;
			count_single = 0;
		}
	}
	return 0;
}

/**************************************************************************
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������
**************************************************************************/
u8 click(void)
{
	static u8 flag_key = 1; // �������ɿ���־
	if (flag_key && KEY == 0)
	{
		flag_key = 0;
		return 1; // ��������
	}
	else if (1 == KEY)
		flag_key = 1;
	return 0; // �ް�������
}
/**************************************************************************
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������2s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count, Long_Press;
	if (Long_Press == 0 && KEY == 0)
		Long_Press_count++; // ������־λδ��1
	else
		Long_Press_count = 0;
	if (Long_Press_count > 150)
	{
		Long_Press = 1;
		Long_Press_count = 0;
		return 1;
	}
	if (Long_Press == 1) // ������־λ��1
	{
		Long_Press = 0;
	}
	return 0;
}
